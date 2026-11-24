// SPDX-License-Identifier: Apache-2.0
// Capture/encoder initialization adapted from the in-tree IMX219 video example.
#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#include <inttypes.h>
#include <fcntl.h>
#include <unistd.h>
#include <sys/ioctl.h>
#include <sys/mman.h>
#include "esp_log.h"
#include "esp_timer.h"
#include "esp_system.h"
#include "esp_heap_caps.h"
#include "fullview_worker.h"
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "linux/videodev2.h"
#include "esp_video_init.h"
#include "esp_video_device.h"
#include "esp_video_ioctl.h"
#include "esp_h264_enc_single_hw.h"
#include "esp_h264_alloc.h"
#include "stream_bridge.h"
#include "profiles.h"
#include "encoder_alloc.h"
#include "imx219.h"
#include "image_controls.h"
void board_eth_start(void);

#define CAM_SCCB_I2C_PORT 0
#define CAM_SCCB_SCL_PIN 8
#define CAM_SCCB_SDA_PIN 7
#define CAM_SCCB_FREQ_HZ 100000
#define CAM_RESET_PIN (-1)
#define CAM_PWDN_PIN (-1)
#define CAM_DEV_PATH ESP_VIDEO_MIPI_CSI_DEVICE_NAME
#define BUFFER_COUNT 3
#define ENC_OUT_BYTES (512 * 1024)
#define ULTRA_ENC_OUT_BYTES (2 * 1024 * 1024)
#define AIM_SECONDS 4

static const char *TAG = "imx219_video";

static const esp_video_init_csi_config_t csi_config[] = {{
    .sccb_config = {
        .init_sccb = true,
        .i2c_config = { .port = CAM_SCCB_I2C_PORT, .scl_pin = CAM_SCCB_SCL_PIN, .sda_pin = CAM_SCCB_SDA_PIN },
        .freq = CAM_SCCB_FREQ_HZ,
    },
    .reset_pin = CAM_RESET_PIN,
    .pwdn_pin  = CAM_PWDN_PIN,
}};

/*
 * No .cam_motor entry, unlike imx708_video: the v2 module is fixed-focus and
 * there is no VCM at 0x0c to drive.
 */
static const esp_video_init_config_t cam_config = {
    .csi = csi_config,
};

static uint32_t mean_luma(const uint8_t *yuv, uint32_t w, uint32_t h)
{
    size_t line_bytes = (size_t)w * 3 / 2;
    uint64_t sum = 0;
    uint32_t n = 0;

    /*
     * Sample line PAIRS, not single lines - see the note above the function.
     */
    for (uint32_t y = 0; y + 1 < h; y += 64) {
        for (uint32_t p = 0; p < 2; p++) {
            const uint8_t *line = yuv + (size_t)(y + p) * line_bytes;
            for (size_t i = 0; i < line_bytes; i++) {
                if (i % 3 != 0) {           /* index 0 of each triple is chroma */
                    sum += line[i];
                    n++;
                }
            }
        }
    }
    return n ? (uint32_t)(sum / n) : 0;
}


/* Cached control state; ISP control application can lag the sampled image. */
static int ctrl_get(int fd, uint32_t id)
{
    struct v4l2_ext_control c = {.id = id};
    struct v4l2_ext_controls cs = {
        .ctrl_class = V4L2_CTRL_CLASS_USER, .count = 1, .controls = &c,
    };
    return ioctl(fd, VIDIOC_G_EXT_CTRLS, &cs) == 0 ? c.value : -1;
}

static bool run_profile(const stream_profile_t *profile)
{
    int fd = -1;
    bool initialized = false, streaming = false, success = false;
    uint8_t *buffer[BUFFER_COUNT] = {0};
    uint8_t *enc_out = NULL, *scaled = NULL;
    uint32_t scaled_size = 0;
    static fullview_scale_t geometry;
    uint32_t enc_out_size = 0;
    esp_h264_enc_handle_t enc = NULL;
    const int type = V4L2_BUF_TYPE_VIDEO_CAPTURE;
    ESP_LOGI(TAG, "Applying profile %s", profile->id);
    uint32_t w = profile->width, h = profile->height;
    ESP_LOGI(TAG, "encoder allocation: internal_free=%u largest=%u",
             (unsigned)heap_caps_get_free_size(MALLOC_CAP_INTERNAL),
             (unsigned)heap_caps_get_largest_free_block(MALLOC_CAP_INTERNAL));
    /* ---- Encoder ---------------------------------------------------------- */
    esp_h264_enc_cfg_hw_t enc_cfg = {
        .pic_type = ESP_H264_RAW_FMT_O_UYY_E_VYY,
        .gop      = profile->gop,
        .fps      = profile->fps,
        .res      = { .width = (uint16_t)w, .height = (uint16_t)h },
        .rc       = { .bitrate = profile->bitrate, .qp_min = profile->qp_min, .qp_max = profile->qp_max },
    };
    esp_h264_err_t herr = esp_h264_enc_hw_new(&enc_cfg, &enc);
    if (herr != ESP_H264_ERR_OK) {
        /*
         * The hardware core tops out at 1920x2032 and takes only the packed
         * YUV420 layout above, so a rejection here is a configuration problem,
         * not a missing chip.
         */
        ESP_LOGE(TAG, "esp_h264_enc_hw_new failed (%d) for %" PRIu32 "x%" PRIu32, herr, w, h);
        goto cleanup;
    }
    if (esp_h264_enc_open(enc) != ESP_H264_ERR_OK) {
        ESP_LOGE(TAG, "esp_h264_enc_open failed");
        goto cleanup;
    }
    ESP_LOGI(TAG, "H.264 %" PRIu32 "x%" PRIu32 " @ %u fps, %u bit/s, GOP %u, QP %u-%u",
             w, h, profile->fps, profile->bitrate, profile->gop, profile->qp_min, profile->qp_max);


    if (imx219_set_initial_format_index(profile->sensor_mode) != ESP_OK) goto cleanup;
    if (esp_video_init(&cam_config) != ESP_OK) {
        ESP_LOGE(TAG, "esp_video_init failed");
        goto cleanup;
    }

    initialized = true;
    fd = open(CAM_DEV_PATH, O_RDONLY);
    if (fd < 0) {
        ESP_LOGE(TAG, "open %s failed", CAM_DEV_PATH);
        goto cleanup;
    }

    /* Never reset the sensor after ISP/IPA has cached its initial control state. */
    esp_cam_sensor_format_t selected;
    if (ioctl(fd, VIDIOC_G_SENSOR_FMT, &selected) != 0 ||
        selected.width != profile->crop_width / 2 || selected.height != profile->crop_height / 2 ||
        selected.fps != profile->fps * profile->divisor) {
        ESP_LOGE(TAG, "initial sensor mode does not match %s", profile->id);
        goto cleanup;
    }
    struct timeval timeout = {.tv_sec = 2};
    if (ioctl(fd, VIDIOC_S_DQBUF_TIMEOUT, &timeout) != 0) goto cleanup;

    /*
     * Ask the ISP for YUV420 rather than the snapshot example's RGB565.
     *
     * This is the whole reason there is no colour conversion in this file. The
     * P4's H.264 core does not take RGB: it wants YUV420 in a packed layout
     * Espressif call O_UYY_E_VYY, and the ISP's YUV420 output *is* that layout
     * - the two blocks were designed to hand off to each other. Converting a
     * 2 Mpx frame from RGB565 in software 30 times a second would not fit in
     * the frame budget on this CPU; asking the ISP for the right format costs
     * nothing. It also drops the frame from 4.0 MB to 3.0 MB.
     */
    struct v4l2_format fmt = { .type = type };
    if (ioctl(fd, VIDIOC_G_FMT, &fmt) != 0) {
        ESP_LOGE(TAG, "G_FMT failed");
        goto cleanup;
    }
    fmt.fmt.pix.width = profile->crop_width / 2;
    fmt.fmt.pix.height = profile->crop_height / 2;
    fmt.fmt.pix.pixelformat = V4L2_PIX_FMT_YUV420;
    if (ioctl(fd, VIDIOC_S_FMT, &fmt) != 0) {
        ESP_LOGE(TAG, "the ISP would not give us YUV420 - the H.264 core takes nothing else");
        goto cleanup;
    }
    if (ioctl(fd, VIDIOC_G_FMT, &fmt) != 0) {
        ESP_LOGE(TAG, "G_FMT failed");
        goto cleanup;
    }


    w = fmt.fmt.pix.width; h = fmt.fmt.pix.height;
    uint32_t fourcc = fmt.fmt.pix.pixelformat;
    ESP_LOGI(TAG, "format %" PRIu32 "x%" PRIu32 " fourcc=%c%c%c%c sizeimage=%" PRIu32,
             w, h, (char)(fourcc & 0xff), (char)((fourcc >> 8) & 0xff),
             (char)((fourcc >> 16) & 0xff), (char)((fourcc >> 24) & 0xff),
             fmt.fmt.pix.sizeimage);
    if (fourcc != V4L2_PIX_FMT_YUV420) {
        ESP_LOGE(TAG, "not YUV420 after S_FMT - the encoder will produce garbage");
        goto cleanup;
    }
    if (w != profile->crop_width / 2 || h != profile->crop_height / 2 ||
        profile->width % 16 || profile->height % 16) {
        ESP_LOGE(TAG, "Capture/output dimensions do not match the profile");
        goto cleanup;
    }
    if (!profile->fullview_resize && (w != profile->width || h != profile->height)) goto cleanup;

    image_controls_begin(profile, fd);
    size_t frame_bytes = (size_t)w * h * 3 / 2;

    size_t encoded_frame_bytes = (size_t)profile->width * profile->height * 3 / 2;
    if (profile->fullview_resize) {
        if (profile->fullview_resize == 1 &&
            !fullview_scale_config(&geometry, w, h, profile->width, profile->height)) goto cleanup;
        if (profile->fullview_resize == 2 &&
            (profile->content_width != w || profile->content_height != h ||
             profile->content_x + w > profile->width || profile->content_y + h > profile->height ||
             (profile->content_x | profile->content_y) & 1)) goto cleanup;
        scaled = esp_h264_aligned_calloc(128, 1, encoded_frame_bytes, &scaled_size, ESP_H264_MEM_SPIRAM);
        if (!scaled) goto cleanup;
        if (profile->fullview_resize == 2) {
            for (size_t i = 0; i < encoded_frame_bytes; i += 3) {
                scaled[i] = 128; scaled[i + 1] = scaled[i + 2] = 16;
            }
        }
        ESP_LOGI(TAG, "Full-view processing %ux%u -> %ux%u",
                 (unsigned)w, (unsigned)h, profile->width, profile->height);
    }

    /* ---- Buffers ---------------------------------------------------------- */
    struct v4l2_requestbuffers req = { .count = BUFFER_COUNT, .type = type, .memory = V4L2_MEMORY_MMAP };
    if (ioctl(fd, VIDIOC_REQBUFS, &req) != 0) {
        ESP_LOGE(TAG, "REQBUFS failed - not enough PSRAM for %d x %" PRIu32 " byte frames?",
                 BUFFER_COUNT, fmt.fmt.pix.sizeimage);
        goto cleanup;
    }
    for (int i = 0; i < BUFFER_COUNT; i++) {
        struct v4l2_buffer b = { .type = type, .memory = V4L2_MEMORY_MMAP, .index = i };
        if (ioctl(fd, VIDIOC_QUERYBUF, &b) != 0 || b.length < frame_bytes) goto cleanup;
        buffer[i] = mmap(NULL, b.length, PROT_READ | PROT_WRITE, MAP_SHARED, fd, b.m.offset);
        if (buffer[i] == MAP_FAILED || ioctl(fd, VIDIOC_QBUF, &b) != 0) goto cleanup;
    }

    /*
     * The encoder's output buffer is written by DMA and then invalidated
     * wholesale by the component, so it has to be cache-line aligned in both
     * address and length - an ordinary malloc is not. esp_h264_aligned_calloc
     * rounds the length up for us and reports what it actually got.
     */
    enc_out = esp_h264_aligned_calloc(16, 1, profile->fullview_resize == 2 ? ULTRA_ENC_OUT_BYTES : ENC_OUT_BYTES, &enc_out_size, ESP_H264_MEM_SPIRAM);
    if (!enc_out) {
        ESP_LOGE(TAG, "no memory for the %d byte encoder output buffer", ENC_OUT_BYTES);
        goto cleanup;
    }

    if (ioctl(fd, VIDIOC_STREAMON, &type) != 0) {
        ESP_LOGE(TAG, "STREAMON failed");
        goto cleanup;
    }
    streaming = true;


    int64_t settle_end = esp_timer_get_time() + (profile->fps == 1 ? 15 : AIM_SECONDS) * 1000000LL;
    struct v4l2_buffer buf;
    uint32_t settled_luma = 0;
    while (esp_timer_get_time() < settle_end) {
        buf = (struct v4l2_buffer){.type = type, .memory = V4L2_MEMORY_MMAP};
        if (ioctl(fd, VIDIOC_DQBUF, &buf) != 0 || buf.index >= BUFFER_COUNT) goto cleanup;
        settled_luma = mean_luma(buffer[buf.index], w, h);
        if (ioctl(fd, VIDIOC_QBUF, &buf) != 0) goto cleanup;
    }
    ESP_LOGI(TAG, "settled mean luma %" PRIu32, settled_luma);
    stream_start(profile->width, profile->height, profile->fps);
    profiles_applied(profile);
    image_controls_set_ready(true);
    uint32_t captured = 0;
    int64_t next_sample = 0;
    while (profiles_requested() == profile) {
        image_controls_apply(fd);
        buf = (struct v4l2_buffer){.type = type, .memory = V4L2_MEMORY_MMAP};
        if (ioctl(fd, VIDIOC_DQBUF, &buf) != 0 || buf.index >= BUFFER_COUNT) goto cleanup;
        int64_t now = esp_timer_get_time();
        if (buf.bytesused < frame_bytes) {
            ioctl(fd, VIDIOC_QBUF, &buf);
            goto cleanup;
        }
        if (now >= next_sample || profile->fps == 1) {
            int exposure = ctrl_get(fd, V4L2_CID_EXPOSURE), gain = ctrl_get(fd, V4L2_CID_GAIN);
            image_controls_sample(exposure, gain);
            stream_image_sample(exposure, gain,
                                mean_luma(buffer[buf.index], w, h));
            next_sample = now + 1000000;
        }
        if ((captured++ % profile->divisor) != 0) {
            if (ioctl(fd, VIDIOC_QBUF, &buf) != 0) goto cleanup;
            continue;
        }
        uint32_t scale_us = 0;
        if (scaled) {
            int64_t scale_start = esp_timer_get_time();
            if (profile->fullview_resize == 2) {
                size_t src_stride = (size_t)w * 3 / 2;
                size_t dst_stride = (size_t)profile->width * 3 / 2;
                uint8_t *dst = scaled + profile->content_y * dst_stride + profile->content_x * 3 / 2;
                for (unsigned row = 0; row < h; ++row)
                    memcpy(dst + row * dst_stride, buffer[buf.index] + row * src_stride, src_stride);
            } else {
                fullview_resize(&geometry, buffer[buf.index], scaled);
            }
            scale_us = (uint32_t)(esp_timer_get_time() - scale_start);
        }
        esp_h264_enc_in_frame_t in = {
            .raw_data = {.buffer = scaled ? scaled : buffer[buf.index], .len = encoded_frame_bytes},
            .pts = (uint32_t)(now / 1000),
        };
        esp_h264_enc_out_frame_t out = {.raw_data = {.buffer = enc_out, .len = enc_out_size}};
        int64_t start = esp_timer_get_time();
        herr = esp_h264_enc_process(enc, &in, &out);
        uint32_t elapsed = (uint32_t)(esp_timer_get_time() - start);
        if (ioctl(fd, VIDIOC_QBUF, &buf) != 0) goto cleanup;
        stream_encoded(enc_out, out.length, (uint32_t)(now / 1000),
                       out.frame_type == ESP_H264_FRAME_TYPE_IDR, elapsed, scale_us,
                       herr == ESP_H264_ERR_OK);
    }
    success = true;
cleanup:
    image_controls_set_ready(false);
    /* RTSP must release its old session/config before camera buffers are rebuilt. */
    if (!stream_pause()) {
        ESP_LOGE(TAG, "RTSP stop timeout; restarting to recover");
        esp_restart();
    }
    if (streaming) ioctl(fd, VIDIOC_STREAMOFF, &type);
    free(scaled);
    free(enc_out);
    if (fd >= 0) {
        struct v4l2_requestbuffers release = {.count = 0, .type = type, .memory = V4L2_MEMORY_MMAP};
        ioctl(fd, VIDIOC_REQBUFS, &release);
        close(fd);
    }
    if (initialized) esp_video_deinit();
    /* Keep the large internal reference allocation until other teardown is done. */
    if (enc) { esp_h264_enc_close(enc); esp_h264_enc_del(enc); }
    return success;
}

void app_main(void)
{
    encoder_memory_init();
    fullview_worker_init();
    profiles_init();
    image_controls_init();
    stream_init();
    board_eth_start();
    unsigned failures = 0;
    while (true) {
        const stream_profile_t *profile = profiles_requested();
        if (!run_profile(profile)) {
            ESP_LOGE(TAG, "Profile %s failed; falling back to detail15", profile->id);
            profiles_rollback("camera_profile_initialization_or_capture_failed");
            if (++failures >= 2) esp_restart();
            vTaskDelay(pdMS_TO_TICKS(1000));
        } else {
            failures = 0;
        }
    }
}
