/*
 * SPDX-FileCopyrightText: 2026 esp_cam_sensor_imx contributors
 *
 * SPDX-License-Identifier: Apache-2.0
 *
 * IMX708 single-frame snapshot to microSD, for the Waveshare ESP32-P4-WIFI6.
 *
 * Mounts the SD card, streams the IMX708, waits a few seconds so you can aim
 * and the pipeline can settle, grabs one frame, and writes it as a 24-bit BMP
 * (/sdcard/imx708.bmp) you can open on any PC. The sensor outputs RGB565 via
 * the ISP; we expand that to BGR-888 for a maximally-compatible BMP.
 */
#include <stdio.h>
#include <string.h>
#include <inttypes.h>
#include <fcntl.h>
#include <unistd.h>
#include <sys/ioctl.h>
#include <sys/mman.h>
#include "esp_log.h"
#include "esp_check.h"
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "linux/videodev2.h"
#include "esp_video_init.h"
#include "esp_video_device.h"

#include "driver/sdmmc_host.h"
#include "esp_vfs_fat.h"
#include "sdmmc_cmd.h"
#include "sd_pwr_ctrl_by_on_chip_ldo.h"

/* ---- Camera pins (Waveshare ESP32-P4-WIFI6) ----------------------------- */
#define CAM_SCCB_I2C_PORT   0
#define CAM_SCCB_SCL_PIN    8
#define CAM_SCCB_SDA_PIN    7
#define CAM_SCCB_FREQ_HZ    100000
#define CAM_RESET_PIN       (-1)
#define CAM_PWDN_PIN        (-1)

/* ---- microSD (4-bit SDMMC slot 0, powered by on-chip LDO channel 4) ------ */
#define SD_MOUNT_POINT      "/sdcard"
#define SD_LDO_CHANNEL      4
#define SD_CLK_PIN          43
#define SD_CMD_PIN          44
#define SD_D0_PIN           39
#define SD_D1_PIN           40
#define SD_D2_PIN           41
#define SD_D3_PIN           42

#define OUT_PATH            SD_MOUNT_POINT "/imx708.bmp"
#define AIM_SECONDS         3
#define CAM_DEV_PATH        ESP_VIDEO_MIPI_CSI_DEVICE_NAME
#define BUFFER_COUNT        2

static const char *TAG = "imx708_snapshot";

static const esp_video_init_csi_config_t csi_config[] = {{
    .sccb_config = {
        .init_sccb = true,
        .i2c_config = { .port = CAM_SCCB_I2C_PORT, .scl_pin = CAM_SCCB_SCL_PIN, .sda_pin = CAM_SCCB_SDA_PIN },
        .freq = CAM_SCCB_FREQ_HZ,
    },
    .reset_pin = CAM_RESET_PIN,
    .pwdn_pin  = CAM_PWDN_PIN,
}};
static const esp_video_init_config_t cam_config = { .csi = csi_config };

static sd_pwr_ctrl_handle_t s_pwr_handle = NULL;
static sdmmc_card_t *s_card = NULL;

static esp_err_t sd_mount(void)
{
    /* The P4 gates SD power through on-chip LDO_VO4 — enable it first. */
    sd_pwr_ctrl_ldo_config_t ldo_cfg = { .ldo_chan_id = SD_LDO_CHANNEL };
    ESP_RETURN_ON_FALSE(sd_pwr_ctrl_new_on_chip_ldo(&ldo_cfg, &s_pwr_handle) == ESP_OK,
                        ESP_FAIL, TAG, "SD LDO init failed");

    sdmmc_host_t host = SDMMC_HOST_DEFAULT();
    host.slot = SDMMC_HOST_SLOT_0;
    host.max_freq_khz = SDMMC_FREQ_HIGHSPEED;
    host.pwr_ctrl_handle = s_pwr_handle;

    sdmmc_slot_config_t slot = SDMMC_SLOT_CONFIG_DEFAULT();
    slot.width = 4;
    slot.clk = SD_CLK_PIN; slot.cmd = SD_CMD_PIN;
    slot.d0 = SD_D0_PIN; slot.d1 = SD_D1_PIN; slot.d2 = SD_D2_PIN; slot.d3 = SD_D3_PIN;

    esp_vfs_fat_sdmmc_mount_config_t mount_cfg = {
        .format_if_mount_failed = false,
        .max_files = 4,
        .allocation_unit_size = 16 * 1024,
    };
    esp_err_t ret = esp_vfs_fat_sdmmc_mount(SD_MOUNT_POINT, &host, &slot, &mount_cfg, &s_card);
    if (ret != ESP_OK) {
        ESP_LOGE(TAG, "SD mount failed (0x%x). Card inserted / FAT-formatted?", ret);
        return ret;
    }
    ESP_LOGI(TAG, "SD mounted; card=%s", s_card->cid.name);
    return ESP_OK;
}

/* Expand one RGB565 (LE) pixel to BGR-888 (BMP byte order). */
static inline void rgb565_to_bgr(uint16_t px, uint8_t *bgr)
{
    uint8_t r5 = (px >> 11) & 0x1f, g6 = (px >> 5) & 0x3f, b5 = px & 0x1f;
    bgr[0] = (b5 << 3) | (b5 >> 2);
    bgr[1] = (g6 << 2) | (g6 >> 4);
    bgr[2] = (r5 << 3) | (r5 >> 2);
}

static esp_err_t save_bmp565(const char *path, const uint8_t *rgb565, uint32_t w, uint32_t h)
{
    const uint32_t row_bytes = w * 3;                 /* 24-bit, no padding for even widths */
    const uint32_t img_bytes = row_bytes * h;
    const uint32_t file_bytes = 54 + img_bytes;

    uint8_t hdr[54] = {0};
    hdr[0] = 'B'; hdr[1] = 'M';
    hdr[2] = file_bytes; hdr[3] = file_bytes >> 8; hdr[4] = file_bytes >> 16; hdr[5] = file_bytes >> 24;
    hdr[10] = 54;                                     /* pixel data offset */
    hdr[14] = 40;                                     /* DIB header size   */
    hdr[18] = w; hdr[19] = w >> 8; hdr[20] = w >> 16; hdr[21] = w >> 24;
    hdr[22] = h; hdr[23] = h >> 8; hdr[24] = h >> 16; hdr[25] = h >> 24;
    hdr[26] = 1;                                      /* planes */
    hdr[28] = 24;                                     /* bpp */
    hdr[34] = img_bytes; hdr[35] = img_bytes >> 8; hdr[36] = img_bytes >> 16; hdr[37] = img_bytes >> 24;

    FILE *f = fopen(path, "wb");
    ESP_RETURN_ON_FALSE(f, ESP_FAIL, TAG, "fopen %s failed", path);
    fwrite(hdr, 1, sizeof(hdr), f);

    uint8_t *row = malloc(row_bytes);
    if (!row) { fclose(f); return ESP_ERR_NO_MEM; }

    /* BMP is bottom-up: write source row (h-1) first, row 0 last. */
    for (int32_t y = h - 1; y >= 0; y--) {
        const uint16_t *src = (const uint16_t *)(rgb565 + (uint32_t)y * w * 2);
        for (uint32_t x = 0; x < w; x++) {
            rgb565_to_bgr(src[x], &row[x * 3]);
        }
        fwrite(row, 1, row_bytes, f);
    }
    free(row);
    fclose(f);
    ESP_LOGI(TAG, "wrote %s (%" PRIu32 " bytes)", path, file_bytes);
    return ESP_OK;
}

void app_main(void)
{
    ESP_LOGI(TAG, "IMX708 snapshot to SD");

    if (sd_mount() != ESP_OK) {
        return;
    }
    if (esp_video_init(&cam_config) != ESP_OK) {
        ESP_LOGE(TAG, "esp_video_init failed");
        return;
    }

    int fd = open(CAM_DEV_PATH, O_RDONLY);
    if (fd < 0) { ESP_LOGE(TAG, "open %s failed", CAM_DEV_PATH); return; }

    const int type = V4L2_BUF_TYPE_VIDEO_CAPTURE;
    struct v4l2_format fmt = { .type = type };
    ioctl(fd, VIDIOC_G_FMT, &fmt);
    uint32_t w = fmt.fmt.pix.width, h = fmt.fmt.pix.height;
    ESP_LOGI(TAG, "format %" PRIu32 "x%" PRIu32 " fourcc=0x%08" PRIx32, w, h, fmt.fmt.pix.pixelformat);

    uint8_t *buffer[BUFFER_COUNT] = {0};
    struct v4l2_requestbuffers req = { .count = BUFFER_COUNT, .type = type, .memory = V4L2_MEMORY_MMAP };
    ioctl(fd, VIDIOC_REQBUFS, &req);
    for (int i = 0; i < BUFFER_COUNT; i++) {
        struct v4l2_buffer b = { .type = type, .memory = V4L2_MEMORY_MMAP, .index = i };
        ioctl(fd, VIDIOC_QUERYBUF, &b);
        buffer[i] = mmap(NULL, b.length, PROT_READ | PROT_WRITE, MAP_SHARED, fd, b.m.offset);
        ioctl(fd, VIDIOC_QBUF, &b);
    }
    ioctl(fd, VIDIOC_STREAMON, &type);

    ESP_LOGI(TAG, "aim the camera — capturing in %d s...", AIM_SECONDS);
    uint32_t t_end = esp_log_timestamp() + AIM_SECONDS * 1000;
    struct v4l2_buffer buf;
    /* Keep recycling frames until the aim window elapses; keep the last one. */
    do {
        buf = (struct v4l2_buffer){ .type = type, .memory = V4L2_MEMORY_MMAP };
        if (ioctl(fd, VIDIOC_DQBUF, &buf) != 0) { ESP_LOGE(TAG, "DQBUF failed"); goto done; }
        if (esp_log_timestamp() < t_end) {
            ioctl(fd, VIDIOC_QBUF, &buf);            /* discard, keep streaming */
        }
    } while (esp_log_timestamp() < t_end);

    ESP_LOGI(TAG, "captured frame: %" PRIu32 " bytes", buf.bytesused);
    save_bmp565(OUT_PATH, buffer[buf.index], w, h);
    ioctl(fd, VIDIOC_QBUF, &buf);

done:
    ioctl(fd, VIDIOC_STREAMOFF, &type);
    close(fd);
    esp_video_deinit();
    esp_vfs_fat_sdcard_unmount(SD_MOUNT_POINT, s_card);
    if (s_pwr_handle) { sd_pwr_ctrl_del_on_chip_ldo(s_pwr_handle); }
    ESP_LOGI(TAG, "==== done — remove the SD card and open %s on your PC ====", "imx708.bmp");
}
