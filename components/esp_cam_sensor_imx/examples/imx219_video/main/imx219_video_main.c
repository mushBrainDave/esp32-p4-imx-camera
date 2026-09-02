/*
 * SPDX-FileCopyrightText: 2026 esp_cam_sensor_imx contributors
 *
 * SPDX-License-Identifier: Apache-2.0
 *
 * IMX219 video recording over USB serial, for the Waveshare ESP32-P4-WIFI6.
 *
 * Streams the IMX219 (Raspberry Pi Camera Module v2 / NoIR v2) as YUV420,
 * encodes it with the P4's hardware H.264 core, buffers the whole clip in
 * PSRAM, and then sends it down the console UART in one framed payload - the
 * same transport imx219_snapshot uses for stills. tools/capture.py picks it up
 * and muxes it into a .mp4 you can double-click.
 *
 * Why H.264 rather than a burst of JPEGs. For a single still, H.264 is the
 * wrong tool: an I-frame is roughly a JPEG with extra ceremony and no size win,
 * which is why the snapshot example sends JPEG. For a sequence, temporal
 * prediction is the entire point.
 *
 * Why record first and send afterwards, rather than streaming as it encodes.
 * The console is 2 Mbaud - about 200 KB/s - which is well under any bitrate
 * worth recording at this resolution. Streaming live would force the bitrate
 * down to fit the cable and make picture quality a property of the UART.
 * Recording into PSRAM decouples them: the encoder runs at whatever bitrate
 * suits the picture, and the link only decides how long you wait afterwards.
 * The cost is that clip length is bounded by REC_BUF_BYTES rather than by
 * patience.
 *
 * This is imx708_video with the autofocus removed - the v2 module is
 * fixed-focus - and with the macroblock alignment problem moved from the height
 * to the WIDTH, which needs an entirely different fix. See the macroblock note
 * below.
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
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "linux/videodev2.h"
#include "esp_video_init.h"
#include "esp_video_device.h"

#include "esp_h264_enc_single_hw.h"
#include "esp_h264_alloc.h"

#include "imx_serial_img.h"

/* ---- Camera pins (Waveshare ESP32-P4-WIFI6) ----------------------------- */
#define CAM_SCCB_I2C_PORT   0
#define CAM_SCCB_SCL_PIN    8
#define CAM_SCCB_SDA_PIN    7
#define CAM_SCCB_FREQ_HZ    100000
#define CAM_RESET_PIN       (-1)
#define CAM_PWDN_PIN        (-1)

#define CAM_DEV_PATH        ESP_VIDEO_MIPI_CSI_DEVICE_NAME

/*
 * Three capture buffers, not the snapshot's two.
 *
 * Every frame here goes through the encoder before its buffer can be handed
 * back, so unlike a snapshot there is real work between DQBUF and QBUF. With
 * two buffers the camera has exactly one place to put the next frame while we
 * hold the other, and any encode that overruns the 33 ms frame interval drops a
 * frame. The third buffer absorbs that jitter. At YUV420 each one is
 * w*h*3/2 = 3.0 MB of the 32 MB PSRAM.
 */
#define BUFFER_COUNT        3

/* ---- Recording ---------------------------------------------------------- */
/*
 * How long to record, once AE and AWB have settled.
 *
 * The real limit is REC_BUF_BYTES, not this: at VIDEO_BITRATE the buffer holds
 * REC_BUF_BYTES*8/VIDEO_BITRATE seconds, and recording stops at whichever comes
 * first. The run reports which one ended it.
 */
#define VIDEO_SECONDS       8

/*
 * This example runs the driver's 1632x1232 mode at 30 fps (see the macroblock
 * note below for why it is not the 1640-wide default), so that is the frame
 * rate the encoder is told to expect - a little faster than the IMX708
 * example's 28. It affects rate control's bit budget per frame and the VUI
 * timing written into the SPS; it does not make frames arrive any faster. The
 * clip is timestamped from the frames that actually arrived, so a shortfall
 * shows up as a slower measured fps in the log and correct timing in the .mp4,
 * not as a sped-up video.
 */
#define VIDEO_FPS           30

/*
 * One IDR per second. An IDR frame resets prediction, so it is both the only
 * place playback can start and the only place a stream can recover after a
 * corrupt frame; the encoder emits SPS+PPS ahead of each one. Every IDR costs
 * roughly a JPEG's worth of bits, so a much shorter GOP spends the bitrate on
 * re-sending the scene rather than on detail.
 */
#define VIDEO_GOP           VIDEO_FPS

/*
 * 4 Mbit/s. The cropped frame is 1632x1232 = 2.01 Mpx, within a couple of
 * percent of the IMX708 example's 1920x1072 = 2.06 Mpx, so the same budget buys
 * the same quality per pixel. Deliberately well above what the 2 Mbaud console
 * could carry live (~1.6 Mbit/s) - see the note at the top about why recording
 * first makes the link a transfer-time question rather than a quality one.
 */
#define VIDEO_BITRATE       4000000

/*
 * QP bounds for rate control. 51 is the codec's maximum (worst quality); the
 * floor stops rate control from spending the whole budget on one easy frame.
 */
#define VIDEO_QP_MIN        20
#define VIDEO_QP_MAX        45

/* Clip buffer in PSRAM. 6 MB is ~12 s at VIDEO_BITRATE. */
#define REC_BUF_BYTES       (6 * 1024 * 1024)

/*
 * Per-frame encoder output buffer. Sized for the worst case, an IDR of a
 * detailed scene, with plenty of margin: the encoder reports
 * ESP_H264_ERR_OVERFLOW rather than scribbling past the end, but an overflowed
 * frame is a lost frame.
 */
#define ENC_OUT_BYTES       (512 * 1024)

/* Room for the frame table. Generous - a slow link is cheaper than a truncated log. */
#define MAX_FRAMES          ((VIDEO_SECONDS + 4) * VIDEO_FPS)

/*
 * Long enough for AE and AWB to converge before recording starts. Shorter than
 * the IMX708 example's 6 s, which also had to wait out a contrast-AF search.
 */
#define AIM_SECONDS         4

/*
 * Macroblock alignment - why this example needs a specific sensor mode.
 *
 * H.264 codes in 16x16 macroblocks, and the IMX219's default mode is
 * 1640x1232. The height is fine - 1232 is 77 macroblock rows exactly - but the
 * WIDTH is not: 1640 is 102.5 macroblocks.
 *
 * That is a different problem from the one imx708_video has, and it does NOT
 * have the same fix. There, 1080 was trimmed to 1072 simply by telling the
 * encoder a smaller height, because a shorter frame is a *prefix* of the buffer
 * the ISP filled - the bytes the encoder reads are exactly the bytes it wants.
 * A narrower frame is not a prefix of anything: the encoder takes its line
 * stride from the width it was given, so telling it 1632 while the ISP writes
 * 1640-pixel lines makes each line start 8 pixels further left than the one
 * before and the picture shears diagonally.
 *
 * So the crop has to happen where the stride is decided. Two candidates:
 *
 *   The ISP's crop, via VIDIOC_S_SELECTION. Does not work on this board.
 *   esp_video gates ESP_VIDEO_ISP_DEVICE_CROP on CONFIG_ESP32P4_REV_MIN_FULL
 *   >= 300, i.e. ESP32-P4 revision v3.0; this board's chip is v1.3, so
 *   S_SELECTION returns ESP_ERR_NOT_SUPPORTED and the video device has no
 *   set_selection at all. Measured, not assumed - it was tried first.
 *
 *   The sensor's readout window. Works everywhere, costs nothing at runtime,
 *   and is what the driver's 1632x1232 mode is: mode 0 with 8 columns trimmed,
 *   4 from each side, same VTS and therefore the same frame rate and exposure
 *   limits. sdkconfig.defaults selects it with
 *   CONFIG_CAMERA_IMX219_MIPI_IF_FORMAT_INDEX_DEFAULT=2.
 *
 * The check below is therefore a guard, not a fix: if the frame arriving here
 * is not a whole number of macroblocks, the mode index is wrong and the picture
 * would shear.
 */

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

/* One entry per encoded frame, kept so the host can build a sample table
 * without having to guess frame timing from a nominal frame rate. */
typedef struct {
    uint32_t offset;    /* byte offset of the frame in the clip buffer */
    uint32_t len;       /* Annex-B bytes, including any SPS/PPS in front of an IDR */
    uint32_t pts_ms;    /* milliseconds from the first recorded frame */
    uint8_t  is_idr;
} rec_frame_t;

static rec_frame_t s_frames[MAX_FRAMES];

/*
 * Mean luma of a YUV420 frame in the encoder's packed layout.
 *
 * A cheap "is there a picture in here at all" check that needs no decoder on
 * either end. The layout stores odd lines as u y y u y y... and even lines as
 * v y y v y y..., so every byte whose index within the line is not a multiple
 * of three is a luma sample. A mean near 0 or near 255 means the frame is black
 * or blown out; a mean that is plausible but identical for every frame means
 * nothing is being DMA'd in and the "video" is stale PSRAM.
 *
 * Sampled every 64th line PAIR, not every 64th line, and that detail is load
 * bearing: an even stride only ever lands on lines of one parity, and doing so
 * produced a perfectly regular 45/60 alternation every two frames here - in a
 * stream whose decoded luma was in fact steady to within 0.05 counts. Covering
 * both parities removes it. Left as one line in 64 otherwise, to keep this off
 * the frame budget.
 */
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

void app_main(void)
{
    int fd = -1;
    uint8_t *rec = NULL;
    uint8_t *enc_out = NULL;
    uint32_t enc_out_size = 0;
    esp_h264_enc_handle_t enc = NULL;
    const int type = V4L2_BUF_TYPE_VIDEO_CAPTURE;
    bool streaming = false;

    ESP_LOGI(TAG, "IMX219 video over serial");

    if (esp_video_init(&cam_config) != ESP_OK) {
        ESP_LOGE(TAG, "esp_video_init failed");
        return;
    }

    fd = open(CAM_DEV_PATH, O_RDONLY);
    if (fd < 0) {
        ESP_LOGE(TAG, "open %s failed", CAM_DEV_PATH);
        goto cleanup;
    }

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
    fmt.fmt.pix.pixelformat = V4L2_PIX_FMT_YUV420;
    if (ioctl(fd, VIDIOC_S_FMT, &fmt) != 0) {
        ESP_LOGE(TAG, "the ISP would not give us YUV420 - the H.264 core takes nothing else");
        goto cleanup;
    }
    if (ioctl(fd, VIDIOC_G_FMT, &fmt) != 0) {
        ESP_LOGE(TAG, "G_FMT failed");
        goto cleanup;
    }


    uint32_t w = fmt.fmt.pix.width, h = fmt.fmt.pix.height;
    uint32_t fourcc = fmt.fmt.pix.pixelformat;
    ESP_LOGI(TAG, "format %" PRIu32 "x%" PRIu32 " fourcc=%c%c%c%c sizeimage=%" PRIu32,
             w, h, (char)(fourcc & 0xff), (char)((fourcc >> 8) & 0xff),
             (char)((fourcc >> 16) & 0xff), (char)((fourcc >> 24) & 0xff),
             fmt.fmt.pix.sizeimage);
    if (fourcc != V4L2_PIX_FMT_YUV420) {
        ESP_LOGE(TAG, "not YUV420 after S_FMT - the encoder will produce garbage");
        goto cleanup;
    }
    if (w % 16 || h % 16) {
        ESP_LOGE(TAG, "%" PRIu32 "x%" PRIu32 " is not a whole number of 16-pixel "
                 "macroblocks. The encoder would take its line stride from the width "
                 "and the picture would shear. Set "
                 "CONFIG_CAMERA_IMX219_MIPI_IF_FORMAT_INDEX_DEFAULT=2 to select the "
                 "driver's 1632x1232 mode.", w, h);
        goto cleanup;
    }

    size_t frame_bytes = (size_t)w * h * 3 / 2;

    /* ---- Buffers ---------------------------------------------------------- */
    uint8_t *buffer[BUFFER_COUNT] = {0};
    struct v4l2_requestbuffers req = { .count = BUFFER_COUNT, .type = type, .memory = V4L2_MEMORY_MMAP };
    if (ioctl(fd, VIDIOC_REQBUFS, &req) != 0) {
        ESP_LOGE(TAG, "REQBUFS failed - not enough PSRAM for %d x %" PRIu32 " byte frames?",
                 BUFFER_COUNT, fmt.fmt.pix.sizeimage);
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
    /*
     * The encoder's output buffer is written by DMA and then invalidated
     * wholesale by the component, so it has to be cache-line aligned in both
     * address and length - an ordinary malloc is not. esp_h264_aligned_calloc
     * rounds the length up for us and reports what it actually got.
     */
    enc_out = esp_h264_aligned_calloc(16, 1, ENC_OUT_BYTES, &enc_out_size, ESP_H264_MEM_SPIRAM);
    if (!enc_out) {
        ESP_LOGE(TAG, "no memory for the %d byte encoder output buffer", ENC_OUT_BYTES);
        goto cleanup;
    }

    /* ---- Encoder ---------------------------------------------------------- */
    esp_h264_enc_cfg_hw_t enc_cfg = {
        .pic_type = ESP_H264_RAW_FMT_O_UYY_E_VYY,
        .gop      = VIDEO_GOP,
        .fps      = VIDEO_FPS,
        .res      = { .width = (uint16_t)w, .height = (uint16_t)h },
        .rc       = { .bitrate = VIDEO_BITRATE, .qp_min = VIDEO_QP_MIN, .qp_max = VIDEO_QP_MAX },
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
    ESP_LOGI(TAG, "H.264 %" PRIu32 "x%" PRIu32 " @ %d fps, %d bit/s, GOP %d, QP %d-%d",
             w, h, VIDEO_FPS, VIDEO_BITRATE, VIDEO_GOP, VIDEO_QP_MIN, VIDEO_QP_MAX);

    if (ioctl(fd, VIDIOC_STREAMON, &type) != 0) {
        ESP_LOGE(TAG, "STREAMON failed");
        goto cleanup;
    }
    streaming = true;

    /* ---- Aim window: let AE and AWB converge before recording -------------- */
    ESP_LOGI(TAG, "aim the camera - settling for %d s before recording...", AIM_SECONDS);
    ESP_LOGI(TAG, "   t (ms) | luma");
    int64_t settle_start = esp_timer_get_time();
    int64_t settle_end = settle_start + (int64_t)AIM_SECONDS * 1000000;
    uint32_t luma_prev = UINT32_MAX;
    struct v4l2_buffer buf;
    while (esp_timer_get_time() < settle_end) {
        buf = (struct v4l2_buffer){ .type = type, .memory = V4L2_MEMORY_MMAP };
        if (ioctl(fd, VIDIOC_DQBUF, &buf) != 0) {
            ESP_LOGE(TAG, "DQBUF failed during the aim window");
            goto cleanup;
        }
        /*
         * Same AE trace as imx219_snapshot, and the same reason: a luma that
         * moves and then flattens is the runtime proof that the IPA config
         * loaded, and a column that never moves means it did not - which
         * esp_video only reports upstream as "failed to get configuration to
         * initialize ISP controller", easy to miss in the boot log.
         */
        uint32_t luma = mean_luma(buffer[buf.index], w, h);
        if (luma_prev == UINT32_MAX || luma > luma_prev + 1 || luma + 1 < luma_prev) {
            ESP_LOGI(TAG, "   %6" PRIu32 " | %" PRIu32,
                     (uint32_t)((esp_timer_get_time() - settle_start) / 1000), luma);
            luma_prev = luma;
        }
        ioctl(fd, VIDIOC_QBUF, &buf);
    }

    /* ---- Record ----------------------------------------------------------- */
    ESP_LOGW(TAG, "RECORDING for up to %d s - hold still", VIDEO_SECONDS);

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
        /* Stop before the buffer can no longer hold a worst-case frame, rather
         * than after a partial one has been written into it. */
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
            ESP_LOGI(TAG, "first frame: mean luma %" PRIu32, mean_luma(buffer[buf.index], w, h));
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

        /*
         * Hand the capture buffer back the moment the encoder is done with it.
         * esp_h264_enc_process is synchronous - it blocks until the hardware
         * signals frame-done - so the DMA has finished reading by the time it
         * returns, and holding the buffer any longer only costs the camera a
         * place to put the next frame.
         */
        ioctl(fd, VIDIOC_QBUF, &buf);

        if (herr != ESP_H264_ERR_OK) {
            /* The component forces the next frame back to IDR after any
             * failure, so the stream resynchronises by itself; this frame is
             * simply missing, and the clip's timestamps will show the gap. */
            n_failed++;
            ESP_LOGW(TAG, "encode failed (%d) on frame %" PRIu32 "%s",
                     herr, n_frames,
                     herr == ESP_H264_ERR_OVERFLOW ? " - raise ENC_OUT_BYTES" : "");
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
    /* One frame's worth beyond the last timestamp, so the rate is frames per
     * second of clip rather than per gap between first and last. */
    uint32_t dur_ms = span_ms + (n_frames > 1 ? span_ms / (n_frames - 1) : 1000 / VIDEO_FPS);
    uint32_t fps_x10 = dur_ms ? (uint32_t)((uint64_t)n_frames * 10000 / dur_ms) : 0;

    ESP_LOGI(TAG, "recorded %" PRIu32 " frames (%" PRIu32 " IDR, %" PRIu32 " failed) "
             "in %" PRIu32 " ms - %" PRIu32 ".%" PRIu32 " fps, stopped on: %s",
             n_frames, n_idr, n_failed, dur_ms, fps_x10 / 10, fps_x10 % 10, stop_reason);
    ESP_LOGI(TAG, "encode %" PRIu32 " us mean, %" PRIu32 " us worst (%d us per frame available)",
             (uint32_t)(encode_us_total / n_frames), encode_us_max, 1000000 / VIDEO_FPS);
    ESP_LOGI(TAG, "clip %u bytes = %" PRIu32 " kbit/s actual (asked for %d)",
             (unsigned)rec_len,
             dur_ms ? (uint32_t)((uint64_t)rec_len * 8 / dur_ms) : 0, VIDEO_BITRATE / 1000);
    if (n_failed) {
        ESP_LOGW(TAG, "%" PRIu32 " frames were dropped by the encoder - the clip will "
                 "jump at those points", n_failed);
    }

    /*
     * The frame table.
     *
     * The host can find frame boundaries itself by scanning for Annex-B start
     * codes, and does, so this is not load-bearing for muxing. What it carries
     * that the bitstream does not is when each frame was actually captured:
     * with it the .mp4 plays at the speed the camera really delivered, gaps and
     * all, instead of at a nominal frame rate that was only ever a request.
     */
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
    ESP_LOGI(TAG, "sending %u bytes - about %u s at 2 Mbaud",
             (unsigned)rec_len, (unsigned)(rec_len / 200000 + 1));
    imx_serial_send_blob("imx219", "h264", w, h, rec, rec_len, extra);

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
