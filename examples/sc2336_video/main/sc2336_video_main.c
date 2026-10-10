/*
 * SPDX-FileCopyrightText: 2026 esp_cam_sensor_imx contributors
 *
 * SPDX-License-Identifier: Apache-2.0
 *
 * SC2336 video recording over USB serial, for the Wireless-Tag WT9932P4-TINY
 * with the WTDK-CAM-2336 camera sold for it. A bench tool: it proves the
 * MIPI-CSI -> ISP -> hardware H.264 path on ESP32-P4 v3.x silicon with a camera
 * the board is known to be wired for, before an IMX708 goes anywhere near it.
 *
 * The pipeline is imx708_video's, with the IMX708-specific parts taken out
 * (mode table, frame-length control, autofocus): stream YUV420 from the ISP,
 * encode with the hardware H.264 core, buffer the clip in PSRAM, then send it
 * in one framed payload. tools/capture.py picks it up and muxes an .mp4.
 * See components/esp_cam_sensor_imx/examples/imx708_video for the reasoning
 * behind each of those choices; it is not repeated here.
 */
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
#include "esp_heap_caps.h"
#include "driver/gpio.h"
#include "driver/i2c_master.h"
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "linux/videodev2.h"
#include "esp_video_init.h"
#include "esp_video_device.h"
#include "esp_video_ioctl.h"
#include "esp_cam_sensor_types.h"

#include "esp_h264_enc_single_hw.h"
#include "esp_h264_alloc.h"

#include "imx_serial_img.h"

/* ---- Board (WT9932P4-TINY) ---------------------------------------------- */
/* Camera SCCB pins and speed, as in the vendor BSP (WT_BSP boards/WT9932P4-TINY). */
#define CAM_SCCB_I2C_PORT   0
#define CAM_SCCB_SCL_PIN    8
#define CAM_SCCB_SDA_PIN    7
#define CAM_SCCB_FREQ_HZ    400000
#define CAM_I2C_ADDR        0x30

/*
 * IO0 and IO1 both reach the camera connector. The vendor BSP drives IO0 high
 * before touching the camera ("IO0 is connected to the camera's PWDN/LDO/RESET
 * pin"), the user guide's pin table calls IO1 the camera reset, and the
 * schematic swaps the two. Driving both high satisfies every reading: whichever
 * one is the enable is enabled, whichever is the reset is released.
 */
#define CAM_CTRL_GPIO_A     0
#define CAM_CTRL_GPIO_B     1

#define CAM_DEV_PATH        ESP_VIDEO_MIPI_CSI_DEVICE_NAME
#define BUFFER_COUNT        3

/* ---- Recording ---------------------------------------------------------- */
#define VIDEO_SECONDS       8
#define VIDEO_GOP_SECONDS   1
/* 1024x600 is under a third of 1080p's pixels; 2 Mbit/s is ample for it. */
#define VIDEO_BITRATE       2000000
#define VIDEO_QP_MIN        20
#define VIDEO_QP_MAX        45
#define REC_BUF_BYTES       (6 * 1024 * 1024)
#define ENC_OUT_BYTES       (256 * 1024)
#define VIDEO_FPS_MAX       60
#define MAX_FRAMES          ((VIDEO_SECONDS + 4) * VIDEO_FPS_MAX)
/* No autofocus on this module, so this only has to cover AE and AWB. */
#define AIM_SECONDS         3

static const char *TAG = "sc2336_video";

static const esp_video_init_csi_config_t csi_config[] = {{
    .sccb_config = {
        .init_sccb = true,
        .i2c_config = { .port = CAM_SCCB_I2C_PORT, .scl_pin = CAM_SCCB_SCL_PIN, .sda_pin = CAM_SCCB_SDA_PIN },
        .freq = CAM_SCCB_FREQ_HZ,
    },
    .reset_pin = -1,
    .pwdn_pin  = -1,
}};

static const esp_video_init_config_t cam_config = {
    .csi = csi_config,
};

typedef struct {
    uint32_t offset;
    uint32_t len;
    uint32_t pts_ms;
    uint8_t  is_idr;
} rec_frame_t;

static rec_frame_t s_frames[MAX_FRAMES];

/* Mean luma of an O_UYY_E_VYY frame; see imx708_video for the layout. */
static uint32_t mean_luma(const uint8_t *yuv, uint32_t w, uint32_t h)
{
    size_t line_bytes = (size_t)w * 3 / 2;
    uint64_t sum = 0;
    uint32_t n = 0;

    for (uint32_t y = 0; y < h; y += 64) {
        const uint8_t *line = yuv + (size_t)y * line_bytes;
        for (size_t i = 0; i < line_bytes; i++) {
            if (i % 3 != 0) {
                sum += line[i];
                n++;
            }
        }
    }
    return n ? (uint32_t)(sum / n) : 0;
}

static void camera_power_on(void)
{
    gpio_config_t io = {
        .pin_bit_mask = (1ULL << CAM_CTRL_GPIO_A) | (1ULL << CAM_CTRL_GPIO_B),
        .mode = GPIO_MODE_OUTPUT,
    };
    gpio_config(&io);
    gpio_set_level(CAM_CTRL_GPIO_A, 1);
    gpio_set_level(CAM_CTRL_GPIO_B, 1);
    /* 100 ms is what the vendor BSP waits before its I2C scan. */
    vTaskDelay(pdMS_TO_TICKS(100));
    ESP_LOGI(TAG, "camera control lines IO%d and IO%d driven high", CAM_CTRL_GPIO_A, CAM_CTRL_GPIO_B);
}

/*
 * List everything on the camera bus before esp_video goes looking for the
 * sensor. When detection fails, "nothing answered at all" (power, seating,
 * cable) and "something answered, just not at 0x30" (a different sensor) need
 * different fixes, and esp_video's own error cannot tell them apart.
 */
static void scan_camera_bus(void)
{
    i2c_master_bus_config_t cfg = {
        .i2c_port = CAM_SCCB_I2C_PORT,
        .scl_io_num = CAM_SCCB_SCL_PIN,
        .sda_io_num = CAM_SCCB_SDA_PIN,
        .clk_source = I2C_CLK_SRC_DEFAULT,
        .glitch_ignore_cnt = 7,
        .flags.enable_internal_pullup = true,
    };
    i2c_master_bus_handle_t bus = NULL;
    esp_err_t err = i2c_new_master_bus(&cfg, &bus);
    if (err != ESP_OK) {
        ESP_LOGE(TAG, "could not open the camera I2C bus for a scan: %s", esp_err_to_name(err));
        return;
    }
    int found = 0;
    for (uint16_t addr = 0x08; addr < 0x78; addr++) {
        if (i2c_master_probe(bus, addr, 50) == ESP_OK) {
            ESP_LOGI(TAG, "  I2C device at 0x%02x%s", addr,
                     addr == CAM_I2C_ADDR ? "  <- SC2336" : "");
            found++;
        }
    }
    ESP_LOGI(TAG, "I2C scan: %d device(s)", found);
    i2c_del_master_bus(bus);
}

void app_main(void)
{
    int fd = -1;
    uint8_t *rec = NULL;
    uint8_t *enc_out = NULL;
    uint32_t enc_out_size = 0;
    esp_h264_enc_handle_t enc = NULL;
    const int type = V4L2_BUF_TYPE_VIDEO_CAPTURE;
    bool streaming = false;

    ESP_LOGI(TAG, "SC2336 video over serial");

    camera_power_on();
    scan_camera_bus();

    if (esp_video_init(&cam_config) != ESP_OK) {
        ESP_LOGE(TAG, "esp_video_init failed");
        return;
    }

    fd = open(CAM_DEV_PATH, O_RDONLY);
    if (fd < 0) {
        ESP_LOGE(TAG, "open %s failed - no sensor detected", CAM_DEV_PATH);
        goto cleanup;
    }

    struct v4l2_format fmt = { .type = type };
    if (ioctl(fd, VIDIOC_G_FMT, &fmt) != 0) {
        ESP_LOGE(TAG, "G_FMT failed");
        goto cleanup;
    }
    fmt.fmt.pix.pixelformat = V4L2_PIX_FMT_YUV420;
    if (ioctl(fd, VIDIOC_S_FMT, &fmt) != 0) {
        ESP_LOGE(TAG, "the ISP would not give us YUV420 - the H.264 core needs it");
        goto cleanup;
    }
    if (ioctl(fd, VIDIOC_G_FMT, &fmt) != 0) {
        ESP_LOGE(TAG, "G_FMT failed");
        goto cleanup;
    }

    uint32_t w = fmt.fmt.pix.width, h = fmt.fmt.pix.height;
    uint32_t fourcc = fmt.fmt.pix.pixelformat;
    ESP_LOGI(TAG, "format %" PRIu32 "x%" PRIu32 " fourcc=%c%c%c%c",
             w, h, (char)(fourcc & 0xff), (char)((fourcc >> 8) & 0xff),
             (char)((fourcc >> 16) & 0xff), (char)((fourcc >> 24) & 0xff));
    if (fourcc != V4L2_PIX_FMT_YUV420) {
        ESP_LOGE(TAG, "not YUV420 after S_FMT - the encoder would produce garbage");
        goto cleanup;
    }

    /* Encode whole macroblock rows only: 600 lines -> 592. See imx708_video. */
    uint32_t enc_h = h & ~15u;
    if (enc_h != h) {
        ESP_LOGI(TAG, "encoding %" PRIu32 "x%" PRIu32 " - %" PRIu32 " lines trimmed to whole "
                 "16-pixel macroblock rows", w, enc_h, h - enc_h);
    }
    size_t frame_bytes = (size_t)w * enc_h * 3 / 2;

    /* The sensor's own rate for this mode, so the encoder is told the truth. */
    uint32_t sensor_fps = 30;
    esp_cam_sensor_format_t sf;
    if (ioctl(fd, VIDIOC_G_SENSOR_FMT, &sf) == 0 && sf.fps > 0) {
        sensor_fps = sf.fps;
        ESP_LOGI(TAG, "sensor mode %s, %" PRIu32 " fps", sf.name ? sf.name : "?", sensor_fps);
    } else {
        ESP_LOGW(TAG, "could not read the sensor mode - assuming %" PRIu32 " fps", sensor_fps);
    }

    /* ---- Buffers ---------------------------------------------------------- */
    uint8_t *buffer[BUFFER_COUNT] = {0};
    struct v4l2_requestbuffers req = { .count = BUFFER_COUNT, .type = type, .memory = V4L2_MEMORY_MMAP };
    if (ioctl(fd, VIDIOC_REQBUFS, &req) != 0) {
        ESP_LOGE(TAG, "REQBUFS failed");
        goto cleanup;
    }
    for (int i = 0; i < BUFFER_COUNT; i++) {
        struct v4l2_buffer b = { .type = type, .memory = V4L2_MEMORY_MMAP, .index = i };
        ioctl(fd, VIDIOC_QUERYBUF, &b);
        buffer[i] = mmap(NULL, b.length, PROT_READ | PROT_WRITE, MAP_SHARED, fd, b.m.offset);
        ioctl(fd, VIDIOC_QBUF, &b);
    }

    rec = heap_caps_malloc(REC_BUF_BYTES, MALLOC_CAP_SPIRAM | MALLOC_CAP_8BIT);
    if (!rec) {
        ESP_LOGE(TAG, "no PSRAM for a %d byte clip buffer", REC_BUF_BYTES);
        goto cleanup;
    }
    enc_out = esp_h264_aligned_calloc(16, 1, ENC_OUT_BYTES, &enc_out_size, ESP_H264_MEM_SPIRAM);
    if (!enc_out) {
        ESP_LOGE(TAG, "no memory for the %d byte encoder output buffer", ENC_OUT_BYTES);
        goto cleanup;
    }

    /* ---- Encoder ---------------------------------------------------------- */
    esp_h264_enc_cfg_hw_t enc_cfg = {
        .pic_type = ESP_H264_RAW_FMT_O_UYY_E_VYY,
        .gop      = (uint8_t)(sensor_fps * VIDEO_GOP_SECONDS),
        .fps      = (uint8_t)sensor_fps,
        .res      = { .width = (uint16_t)w, .height = (uint16_t)enc_h },
        .rc       = { .bitrate = VIDEO_BITRATE, .qp_min = VIDEO_QP_MIN, .qp_max = VIDEO_QP_MAX },
    };
    esp_h264_err_t herr = esp_h264_enc_hw_new(&enc_cfg, &enc);
    if (herr != ESP_H264_ERR_OK) {
        ESP_LOGE(TAG, "esp_h264_enc_hw_new failed (%d) for %" PRIu32 "x%" PRIu32, herr, w, enc_h);
        goto cleanup;
    }
    if (esp_h264_enc_open(enc) != ESP_H264_ERR_OK) {
        ESP_LOGE(TAG, "esp_h264_enc_open failed");
        goto cleanup;
    }
    ESP_LOGI(TAG, "H.264 %" PRIu32 "x%" PRIu32 " @ %" PRIu32 " fps, %d bit/s, GOP %d, QP %d-%d",
             w, enc_h, sensor_fps, VIDEO_BITRATE, (int)enc_cfg.gop, VIDEO_QP_MIN, VIDEO_QP_MAX);

    if (ioctl(fd, VIDIOC_STREAMON, &type) != 0) {
        ESP_LOGE(TAG, "STREAMON failed");
        goto cleanup;
    }
    streaming = true;

    /* ---- Aim window: let AE and AWB converge ------------------------------ */
    ESP_LOGI(TAG, "aim the camera - settling for %d s before recording...", AIM_SECONDS);
    uint32_t n_aim = 0;
    int64_t settle_end = esp_timer_get_time() + (int64_t)AIM_SECONDS * 1000000;
    struct v4l2_buffer buf;
    while (esp_timer_get_time() < settle_end) {
        buf = (struct v4l2_buffer){ .type = type, .memory = V4L2_MEMORY_MMAP };
        if (ioctl(fd, VIDIOC_DQBUF, &buf) != 0) {
            ESP_LOGE(TAG, "DQBUF failed during the aim window after %" PRIu32 " frames", n_aim);
            goto cleanup;
        }
        n_aim++;
        ioctl(fd, VIDIOC_QBUF, &buf);
    }
    ESP_LOGI(TAG, "%" PRIu32 " frames during the aim window", n_aim);

    /* ---- Record ----------------------------------------------------------- */
    ESP_LOGW(TAG, "RECORDING for up to %d s", VIDEO_SECONDS);

    size_t rec_len = 0;
    uint32_t n_frames = 0, n_idr = 0, n_failed = 0;
    uint32_t encode_us_max = 0;
    uint64_t encode_us_total = 0;
    int64_t t_first = 0;
    const char *stop_reason = "time";
    int64_t rec_end = esp_timer_get_time() + (int64_t)VIDEO_SECONDS * 1000000;

    while (esp_timer_get_time() < rec_end) {
        if (n_frames >= MAX_FRAMES) {
            stop_reason = "frame table full";
            break;
        }
        if (rec_len + enc_out_size > REC_BUF_BYTES) {
            stop_reason = "clip buffer full";
            break;
        }

        buf = (struct v4l2_buffer){ .type = type, .memory = V4L2_MEMORY_MMAP };
        if (ioctl(fd, VIDIOC_DQBUF, &buf) != 0) {
            ESP_LOGE(TAG, "DQBUF failed after %" PRIu32 " frames", n_frames);
            stop_reason = "capture stalled";
            break;
        }
        int64_t now = esp_timer_get_time();
        if (n_frames == 0) {
            t_first = now;
            ESP_LOGI(TAG, "first frame: mean luma %" PRIu32, mean_luma(buffer[buf.index], w, enc_h));
        }
        uint32_t pts_ms = (uint32_t)((now - t_first) / 1000);

        esp_h264_enc_in_frame_t in_frame = {
            .raw_data = { .buffer = buffer[buf.index], .len = frame_bytes },
            .pts = pts_ms,
        };
        esp_h264_enc_out_frame_t out_frame = {
            .raw_data = { .buffer = enc_out, .len = enc_out_size },
        };

        int64_t t_enc = esp_timer_get_time();
        herr = esp_h264_enc_process(enc, &in_frame, &out_frame);
        uint32_t enc_us = (uint32_t)(esp_timer_get_time() - t_enc);
        ioctl(fd, VIDIOC_QBUF, &buf);

        if (herr != ESP_H264_ERR_OK) {
            n_failed++;
            ESP_LOGW(TAG, "encode failed (%d) on frame %" PRIu32, herr, n_frames);
            continue;
        }

        memcpy(rec + rec_len, enc_out, out_frame.length);
        s_frames[n_frames] = (rec_frame_t){
            .offset = (uint32_t)rec_len,
            .len    = out_frame.length,
            .pts_ms = pts_ms,
            .is_idr = (out_frame.frame_type == ESP_H264_FRAME_TYPE_IDR),
        };
        if (s_frames[n_frames].is_idr) {
            n_idr++;
        }
        rec_len += out_frame.length;
        n_frames++;

        encode_us_total += enc_us;
        if (enc_us > encode_us_max) {
            encode_us_max = enc_us;
        }
    }

    ioctl(fd, VIDIOC_STREAMOFF, &type);
    streaming = false;

    if (n_frames == 0) {
        ESP_LOGE(TAG, "no frames encoded (%" PRIu32 " failures) - nothing to send", n_failed);
        goto cleanup;
    }

    uint32_t span_ms = s_frames[n_frames - 1].pts_ms;
    uint32_t dur_ms = span_ms + (n_frames > 1 ? span_ms / (n_frames - 1) : 1000 / sensor_fps);
    uint32_t fps_x10 = dur_ms ? (uint32_t)((uint64_t)n_frames * 10000 / dur_ms) : 0;

    ESP_LOGI(TAG, "recorded %" PRIu32 " frames (%" PRIu32 " IDR, %" PRIu32 " failed) "
             "in %" PRIu32 " ms - %" PRIu32 ".%" PRIu32 " fps, stopped on: %s",
             n_frames, n_idr, n_failed, dur_ms, fps_x10 / 10, fps_x10 % 10, stop_reason);
    ESP_LOGI(TAG, "encode %" PRIu32 " us mean, %" PRIu32 " us worst (%" PRIu32 " us per frame available)",
             (uint32_t)(encode_us_total / n_frames), encode_us_max, 1000000 / sensor_fps);
    ESP_LOGI(TAG, "clip %u bytes = %" PRIu32 " kbit/s actual (asked for %d)",
             (unsigned)rec_len,
             dur_ms ? (uint32_t)((uint64_t)rec_len * 8 / dur_ms) : 0, VIDEO_BITRATE / 1000);

    printf("VIDTABLE frames=%" PRIu32 " bytes=%u\n", n_frames, (unsigned)rec_len);
    for (uint32_t i = 0; i < n_frames; i++) {
        printf("VIDFRAME i=%" PRIu32 " t=%" PRIu32 " off=%" PRIu32 " len=%" PRIu32 " type=%s\n",
               i, s_frames[i].pts_ms, s_frames[i].offset, s_frames[i].len,
               s_frames[i].is_idr ? "idr" : "p");
    }
    fflush(stdout);

    char extra[64];
    snprintf(extra, sizeof(extra), "fps=%" PRIu32 ".%" PRIu32 " frames=%" PRIu32 " ms=%" PRIu32,
             fps_x10 / 10, fps_x10 % 10, n_frames, dur_ms);
    imx_serial_send_blob("sc2336", "h264", w, enc_h, rec, rec_len, extra);

cleanup:
    if (streaming) {
        ioctl(fd, VIDIOC_STREAMOFF, &type);
    }
    if (enc) {
        esp_h264_enc_close(enc);
        esp_h264_enc_del(enc);
    }
    if (enc_out) {
        free(enc_out);
    }
    if (rec) {
        free(rec);
    }
    if (fd >= 0) {
        close(fd);
    }
    esp_video_deinit();
    ESP_LOGI(TAG, "==== done - clip sent over serial, no card needed ====");
}
