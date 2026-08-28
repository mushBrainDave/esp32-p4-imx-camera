/*
 * SPDX-FileCopyrightText: 2026 esp_cam_sensor_imx contributors
 *
 * SPDX-License-Identifier: Apache-2.0
 *
 * IMX219 bring-up smoke test for the ESP32-P4.
 *
 * This is deliberately minimal: it initialises esp_video (which probes the
 * MIPI-CSI bus and binds the IMX219 driver via auto-detect), opens the capture
 * device, prints what bound and what format it offers, then streams a handful
 * of frames and reports size + a crude average brightness per frame.
 *
 * Success milestones, in order:
 *   1. "esp_video_init OK"                  -> CSI + ISP + I2C came up
 *   2. "driver=... card=..." from QUERYCAP  -> the video device exists
 *   3. Non-zero frames with changing seq    -> the IMX219 is actually streaming
 *
 * If step 1 fails, it is almost always SCCB (I2C) pins / address or the sensor
 * power/reset lines. Fix EXAMPLE_* below for your board first.
 */
#include <stdio.h>
#include <string.h>
#include <inttypes.h>
#include <fcntl.h>
#include <unistd.h>
#include <sys/ioctl.h>
#include <sys/mman.h>
#include "esp_log.h"
#include "esp_heap_caps.h"
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "linux/videodev2.h"
#include "esp_video_init.h"
#include "esp_video_device.h"

/* ---- Board pins: Waveshare ESP32-P4-WIFI6 (SKU 32020) -------------------- *
 * Confirmed against Waveshare docs + the ESPP/esp-bsp board definition:
 *   - Camera SCCB shares the internal I2C bus (also used by the audio codec
 *     and touch controller): port 0, SCL = GPIO8, SDA = GPIO7.
 *   - The RPi-style 15-pin CSI connector does NOT route reset or power-down to
 *     the ESP32-P4; the sensor free-runs off its own onboard oscillator. So
 *     reset/pwdn are -1 and no host XCLK is driven.
 * If you move to a different board, re-check these.                          */
#define EXAMPLE_SCCB_I2C_PORT   0
#define EXAMPLE_SCCB_SCL_PIN    8
#define EXAMPLE_SCCB_SDA_PIN    7
#define EXAMPLE_SCCB_FREQ_HZ    100000
#define EXAMPLE_CAM_RESET_PIN   (-1)   /* not routed on RPi-style CSI header */
#define EXAMPLE_CAM_PWDN_PIN    (-1)   /* not routed on RPi-style CSI header */
/* -------------------------------------------------------------------------- */

#define EXAMPLE_CAM_DEV_PATH    ESP_VIDEO_MIPI_CSI_DEVICE_NAME  /* /dev/video0 */
#define BUFFER_COUNT            2
#define FRAMES_TO_CAPTURE       30

static const char *TAG = "imx219_capture";

static const esp_video_init_csi_config_t csi_config[] = {
    {
        .sccb_config = {
            .init_sccb = true,
            .i2c_config = {
                .port    = EXAMPLE_SCCB_I2C_PORT,
                .scl_pin = EXAMPLE_SCCB_SCL_PIN,
                .sda_pin = EXAMPLE_SCCB_SDA_PIN,
            },
            .freq = EXAMPLE_SCCB_FREQ_HZ,
        },
        .reset_pin = EXAMPLE_CAM_RESET_PIN,
        .pwdn_pin  = EXAMPLE_CAM_PWDN_PIN,
    },
};

static const esp_video_init_config_t cam_config = {
    .csi = csi_config,
};

static void print_fourcc(const char *label, uint32_t f)
{
    ESP_LOGI(TAG, "%s = %c%c%c%c (0x%08" PRIx32 ")", label,
             (char)(f & 0xff), (char)((f >> 8) & 0xff),
             (char)((f >> 16) & 0xff), (char)((f >> 24) & 0xff), f);
}

static esp_err_t capture_loop(int fd)
{
    const int type = V4L2_BUF_TYPE_VIDEO_CAPTURE;
    uint8_t *buffer[BUFFER_COUNT] = {0};
    struct v4l2_format fmt = { .type = type };

    /* Use whatever format/size the pipeline defaults to (driven by the sensor
     * mode + ISP). Keeps this test independent of ISP output tuning. */
    if (ioctl(fd, VIDIOC_G_FMT, &fmt) != 0) {
        ESP_LOGE(TAG, "VIDIOC_G_FMT failed");
        return ESP_FAIL;
    }
    ESP_LOGI(TAG, "format: %" PRIu32 "x%" PRIu32 ", %" PRIu32 " bytes/frame",
             fmt.fmt.pix.width, fmt.fmt.pix.height, fmt.fmt.pix.sizeimage);
    print_fourcc("pixelformat", fmt.fmt.pix.pixelformat);

    struct v4l2_requestbuffers req = {
        .count = BUFFER_COUNT,
        .type = type,
        .memory = V4L2_MEMORY_MMAP,
    };
    if (ioctl(fd, VIDIOC_REQBUFS, &req) != 0) {
        ESP_LOGE(TAG, "VIDIOC_REQBUFS failed");
        return ESP_FAIL;
    }

    for (int i = 0; i < BUFFER_COUNT; i++) {
        struct v4l2_buffer buf = { .type = type, .memory = V4L2_MEMORY_MMAP, .index = i };
        if (ioctl(fd, VIDIOC_QUERYBUF, &buf) != 0) {
            ESP_LOGE(TAG, "VIDIOC_QUERYBUF %d failed", i);
            return ESP_FAIL;
        }
        buffer[i] = mmap(NULL, buf.length, PROT_READ | PROT_WRITE, MAP_SHARED, fd, buf.m.offset);
        if (buffer[i] == MAP_FAILED) {
            ESP_LOGE(TAG, "mmap %d failed", i);
            return ESP_FAIL;
        }
        if (ioctl(fd, VIDIOC_QBUF, &buf) != 0) {
            ESP_LOGE(TAG, "VIDIOC_QBUF %d failed", i);
            return ESP_FAIL;
        }
    }

    if (ioctl(fd, VIDIOC_STREAMON, &type) != 0) {
        ESP_LOGE(TAG, "VIDIOC_STREAMON failed");
        return ESP_FAIL;
    }
    ESP_LOGI(TAG, "streaming started, capturing %d frames...", FRAMES_TO_CAPTURE);

    for (int n = 0; n < FRAMES_TO_CAPTURE; n++) {
        struct v4l2_buffer buf = { .type = type, .memory = V4L2_MEMORY_MMAP };
        if (ioctl(fd, VIDIOC_DQBUF, &buf) != 0) {
            ESP_LOGE(TAG, "VIDIOC_DQBUF failed at frame %d", n);
            return ESP_FAIL;
        }

        /* crude average brightness over the raw buffer (format-agnostic) */
        const uint8_t *p = buffer[buf.index];
        uint32_t step = buf.bytesused > 4096 ? buf.bytesused / 4096 : 1;
        uint64_t acc = 0;
        uint32_t cnt = 0;
        for (uint32_t i = 0; i < buf.bytesused; i += step) {
            acc += p[i];
            cnt++;
        }
        ESP_LOGI(TAG, "frame %2d: seq=%" PRIu32 " bytes=%" PRIu32 " avg=%" PRIu32,
                 n, buf.sequence, buf.bytesused, cnt ? (uint32_t)(acc / cnt) : 0);

        if (ioctl(fd, VIDIOC_QBUF, &buf) != 0) {
            ESP_LOGE(TAG, "VIDIOC_QBUF (recycle) failed at frame %d", n);
            return ESP_FAIL;
        }
    }

    if (ioctl(fd, VIDIOC_STREAMOFF, &type) != 0) {
        ESP_LOGE(TAG, "VIDIOC_STREAMOFF failed");
        return ESP_FAIL;
    }
    ESP_LOGI(TAG, "streaming stopped");
    return ESP_OK;
}

void app_main(void)
{
    ESP_LOGI(TAG, "IMX219 capture bring-up test");

    esp_err_t ret = esp_video_init(&cam_config);
    if (ret != ESP_OK) {
        ESP_LOGE(TAG, "esp_video_init failed: 0x%x  (check SCCB pins/addr, reset/pwdn, sensor power)", ret);
        return;
    }
    ESP_LOGI(TAG, "esp_video_init OK");

    int fd = open(EXAMPLE_CAM_DEV_PATH, O_RDONLY);
    if (fd < 0) {
        ESP_LOGE(TAG, "failed to open %s", EXAMPLE_CAM_DEV_PATH);
        return;
    }

    struct v4l2_capability cap;
    if (ioctl(fd, VIDIOC_QUERYCAP, &cap) == 0) {
        ESP_LOGI(TAG, "opened %s: driver=%s card=%s", EXAMPLE_CAM_DEV_PATH, cap.driver, cap.card);
    } else {
        ESP_LOGW(TAG, "VIDIOC_QUERYCAP failed (continuing)");
    }

    if (capture_loop(fd) == ESP_OK) {
        ESP_LOGI(TAG, "==== IMX219 bring-up PASSED ====");
    } else {
        ESP_LOGE(TAG, "==== IMX219 bring-up FAILED (see errors above) ====");
    }

    close(fd);
    esp_video_deinit();
}
