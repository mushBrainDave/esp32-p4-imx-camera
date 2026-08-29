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
#include <stdlib.h>
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
#include "esp_cache.h"

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
/*
 * Long enough for AE, AWB *and* the autofocus search to converge. Contrast AF
 * is not instantaneous: it moves the lens, waits for it to settle, reads the
 * ISP's definition statistic, and repeats for a coarse pass and then a fine one
 * (l1_scan_points_num + l2_scan_points_num in the IPA config). Capturing before
 * it finishes gives a frame from the middle of the search.
 */
#define AIM_SECONDS         6

/*
 * FOCUS_SWEEP: calibration mode. Step the VCM across its whole electrical range
 * instead of taking one picture, score each position, and write the frames out
 * as focus_<code>.bmp. Use it to find where this particular lens actually
 * reaches infinity and its closest macro distance, then put those numbers in
 * the IPA config's "af" min_pos / max_pos - the defaults there come from
 * libcamera's tuning data for the Camera Module 3, not from your module.
 *
 * It also answers the cruder question first: does the lens move at all? If
 * every position scores the same, the DW9807 is being written but nothing is
 * happening mechanically.
 */
#define FOCUS_SWEEP         0
#define FOCUS_SWEEP_START   0
#define FOCUS_SWEEP_END     1023
#define FOCUS_SWEEP_STEP    64
/* Frames to discard after each move, so the ISP is showing the new position. */
#define FOCUS_SETTLE_FRAMES 4

/*
 * AF_DEBUG_LOG: turn on the AF algorithm's own per-scan-point logging. Set to 0
 * if the extra output destabilises the run - it is emitted from inside the ISP
 * pipeline task, whose stack is a fixed 4 KB, and 64-bit printf formatting is
 * not cheap there.
 */
#define AF_DEBUG_LOG        1

#if FOCUS_SWEEP && CONFIG_ESP_VIDEO_ISP_PIPELINE_CONTROL_CAMERA_MOTOR
#error "FOCUS_SWEEP fights the closed-loop AF: the ISP pipeline keeps re-running its own search and moving the lens back. Set CONFIG_ESP_VIDEO_ISP_PIPELINE_CONTROL_CAMERA_MOTOR=n for a calibration build."
#endif
/*
 * Bring-up switches.
 *
 * TEST_PATTERN: ask the sensor for its internal colour bars. The bars are
 * generated after the pixel array, so they travel the whole MIPI -> CSI -> ISP
 * path. If the BMP shows clean bars, the transport is healthy and any bad
 * picture is optics/exposure/ISP tuning. If the bars are noise too, the fault
 * is upstream of the ISP (link rate, lane count, data type, sensor mode).
 *
 * POISON_BUFFERS: fill each capture buffer with a known byte before streaming
 * and count how much of it survives the capture. A frame that comes back ~100%
 * poison means nothing was DMA'd into it, and the "image" is just untouched
 * PSRAM rather than a corrupted photo.
 *
 * Note on the colour bars: the sensor's pattern generator clips them to its
 * own test-pattern window (regs 0x0620-0x0627, left at power-on defaults), so
 * the bars come out cropped at the left and right edges even when the capture
 * path is perfect. Judge transport health by the bars that ARE drawn being
 * clean and correctly placed, not by them reaching the frame edges.
 */

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
/*
 * The autofocus VCM (DW9807, I2C 0x0c) is a separate chip from the sensor, so
 * esp_video probes for it separately and needs its own entry here - without one
 * the motor auto-detect array is never walked and the lens is never driven, no
 * matter what is compiled in.
 *
 * Same bus, same pins, same init_sccb=true as the CSI config above: esp_video
 * notices port 0 is already up with matching pins and attaches to the existing
 * handle rather than trying to create it twice.
 */
static const esp_video_init_cam_motor_config_t motor_config[] = {{
    .sccb_config = {
        .init_sccb = true,
        .i2c_config = { .port = CAM_SCCB_I2C_PORT, .scl_pin = CAM_SCCB_SCL_PIN, .sda_pin = CAM_SCCB_SDA_PIN },
        .freq = CAM_SCCB_FREQ_HZ,
    },
    /* The Pi 15-pin CSI connector routes none of these to the host. */
    .reset_pin  = -1,
    .pwdn_pin   = -1,
    .signal_pin = -1,
}};

static const esp_video_init_config_t cam_config = {
    .csi = csi_config,
    .cam_motor = motor_config,
};

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


/* ---- Focus ------------------------------------------------------------- */

/*
 * The VCM is exposed on the same video device as the sensor, as the standard
 * V4L2_CID_FOCUS_ABSOLUTE control. The value is a raw 10-bit DAC code, not a
 * distance: bigger means the lens is pushed further out, i.e. focused closer.
 */
static int focus_get(int fd)
{
    struct v4l2_ext_control c = { .id = V4L2_CID_FOCUS_ABSOLUTE, .value = 0 };
    struct v4l2_ext_controls cs = { .ctrl_class = V4L2_CID_CAMERA_CLASS, .count = 1, .controls = &c };

    if (ioctl(fd, VIDIOC_G_EXT_CTRLS, &cs) != 0) {
        return -1;
    }
    return c.value;
}

#if FOCUS_SWEEP
static bool focus_set(int fd, int pos)
{
    struct v4l2_ext_control c = { .id = V4L2_CID_FOCUS_ABSOLUTE, .value = pos };
    struct v4l2_ext_controls cs = { .ctrl_class = V4L2_CID_CAMERA_CLASS, .count = 1, .controls = &c };

    return ioctl(fd, VIDIOC_S_EXT_CTRLS, &cs) == 0;
}
#endif /* FOCUS_SWEEP */

/*
 * Sharpness of the frame's centre band: mean absolute horizontal gradient of
 * the green channel, scaled by 1000 so it survives integer division.
 *
 * Green stands in for luma - it carries two thirds of the luminance and, on
 * this RGGB sensor, has twice the sampling density of red or blue, so it is the
 * least interpolated channel and the one that actually resolves fine detail.
 *
 * The region matches the AF windows in the IPA config rather than the whole
 * frame: corners are the softest part of this lens and would flatten the curve.
 * This is deliberately an independent measurement from the ISP's own AF
 * statistic - if the two disagree about where best focus is, the ISP AF windows
 * or edge threshold are the thing to look at, not the actuator.
 */
static uint32_t centre_sharpness(const uint8_t *rgb565, uint32_t w, uint32_t h)
{
    const uint32_t x0 = w / 8, x1 = w - w / 8;
    const uint32_t y0 = h * 5 / 16, y1 = h * 11 / 16;
    uint64_t sum = 0;
    uint32_t n = 0;

    for (uint32_t y = y0; y < y1; y++) {
        const uint16_t *row = (const uint16_t *)(rgb565 + (size_t)y * w * 2);
        for (uint32_t x = x0; x < x1 - 1; x++) {
            int g0 = (row[x] >> 5) & 0x3f;
            int g1 = (row[x + 1] >> 5) & 0x3f;
            sum += abs(g1 - g0);
            n++;
        }
    }

    return n ? (uint32_t)(sum * 1000 / n) : 0;
}

/*
 * Discard `settle` frames, then leave the next one dequeued in *out.
 *
 * The discards matter after a lens move: the sensor already had frames in
 * flight when the DAC was written, so the first buffers back still show the old
 * position. Returns false if the stream stalled.
 */
#if FOCUS_SWEEP
static bool grab_frame(int fd, int type, int settle, struct v4l2_buffer *out)
{
    for (int i = 0; i <= settle; i++) {
        *out = (struct v4l2_buffer){ .type = type, .memory = V4L2_MEMORY_MMAP };
        if (ioctl(fd, VIDIOC_DQBUF, out) != 0) {
            ESP_LOGE(TAG, "DQBUF failed");
            return false;
        }
        if (i < settle) {
            ioctl(fd, VIDIOC_QBUF, out);
        }
    }
    return true;
}
#endif /* FOCUS_SWEEP */

#if FOCUS_SWEEP
/*
 * Walk the VCM across its range, scoring and saving a frame at each stop.
 *
 * Read the printed table, not the pictures, to find the limits: sharpness rises
 * from the infinity end, peaks wherever your test subject actually is, and
 * falls off towards macro. The codes where the curve stops changing at each end
 * are the lens sitting against its mechanical stops - those bound the useful
 * range. Point it at something with fine detail (text, brickwork) a couple of
 * metres away, and keep the camera still: the metric cannot tell defocus from
 * motion blur.
 */
static void focus_sweep(int fd, int type, uint8_t **buffer, uint32_t w, uint32_t h)
{
    char path[48];
    struct v4l2_buffer buf;

    ESP_LOGW(TAG, "FOCUS SWEEP: %d..%d step %d - this is a calibration run, not a photo",
             FOCUS_SWEEP_START, FOCUS_SWEEP_END, FOCUS_SWEEP_STEP);
    ESP_LOGI(TAG, "  code | sharpness");

    for (int pos = FOCUS_SWEEP_START; pos <= FOCUS_SWEEP_END; pos += FOCUS_SWEEP_STEP) {
        if (!focus_set(fd, pos)) {
            ESP_LOGE(TAG, "failed to set focus code %d - is the DW9807 detected?", pos);
            return;
        }
        if (!grab_frame(fd, type, FOCUS_SETTLE_FRAMES, &buf)) {
            return;
        }

        uint32_t sharp = centre_sharpness(buffer[buf.index], w, h);
        ESP_LOGI(TAG, "  %4d | %" PRIu32, pos, sharp);

        snprintf(path, sizeof(path), SD_MOUNT_POINT "/focus_%04d.bmp", pos);
        save_bmp565(path, buffer[buf.index], w, h);
        ioctl(fd, VIDIOC_QBUF, &buf);
    }

    ESP_LOGW(TAG, "sweep done. A flat column means the lens never moved: check that "
                  "'detected DW9807 VCM' appeared at start-up.");
}
#endif /* FOCUS_SWEEP */

void app_main(void)
{
    ESP_LOGI(TAG, "IMX708 snapshot to SD");

    /*
     * Turn on the AF algorithm's own debug output. It prints one line per scan
     * point - "pos=N, definition: D, luminance: L" - which is the only way to
     * see what the search is actually deciding on: the ISP AF statistics are
     * consumed inside the pipeline task and never reach the application. Needs
     * CONFIG_LOG_MAXIMUM_LEVEL_DEBUG, or esp_log_write filters it out again.
     *
     * definition is the edge energy the search maximises. If it barely varies
     * across positions, the search has nothing to discriminate with and will
     * settle on whichever point it visited first, regardless of actual focus.
     *
     * This needs no sdkconfig change: the library was compiled with debug logs
     * enabled (its LOG_LOCAL_LEVEL is fixed at *its* build time, not ours), so
     * only the runtime per-tag level matters. Raising CONFIG_LOG_MAXIMUM_LEVEL
     * does nothing here and switches ESP_LOGD on across the whole firmware.
     */
#if AF_DEBUG_LOG
    esp_log_level_set("esp_ipa_af", ESP_LOG_DEBUG);
#endif

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
    uint32_t fourcc = fmt.fmt.pix.pixelformat;
    ESP_LOGI(TAG, "format %" PRIu32 "x%" PRIu32 " fourcc=%c%c%c%c (0x%08" PRIx32 ") sizeimage=%" PRIu32,
             w, h, (char)(fourcc & 0xff), (char)((fourcc >> 8) & 0xff),
             (char)((fourcc >> 16) & 0xff), (char)((fourcc >> 24) & 0xff),
             fourcc, fmt.fmt.pix.sizeimage);
    if (fourcc != V4L2_PIX_FMT_RGB565) {
        ESP_LOGE(TAG, "expected RGB565 from the ISP - the BMP below will be garbage");
    }

    uint8_t *buffer[BUFFER_COUNT] = {0};
#if POISON_BUFFERS
    uint32_t buf_len[BUFFER_COUNT] = {0};
#endif
    struct v4l2_requestbuffers req = { .count = BUFFER_COUNT, .type = type, .memory = V4L2_MEMORY_MMAP };
    ioctl(fd, VIDIOC_REQBUFS, &req);
    for (int i = 0; i < BUFFER_COUNT; i++) {
        struct v4l2_buffer b = { .type = type, .memory = V4L2_MEMORY_MMAP, .index = i };
        ioctl(fd, VIDIOC_QUERYBUF, &b);
        buffer[i] = mmap(NULL, b.length, PROT_READ | PROT_WRITE, MAP_SHARED, fd, b.m.offset);
#if POISON_BUFFERS
        buf_len[i] = b.length;
        /*
         * The buffer lives in PSRAM, so this memset lands in L2 and its tail
         * stays dirty there. Left alone, those dirty lines shadow - and on
         * eviction overwrite - what the DMA later writes, eating the bottom of
         * the frame in cache-line-sized holes. Push it out before streaming.
         */
        memset(buffer[i], POISON_BYTE, b.length);
        ESP_ERROR_CHECK(esp_cache_msync(buffer[i], b.length,
                                        ESP_CACHE_MSYNC_FLAG_DIR_C2M | ESP_CACHE_MSYNC_FLAG_UNALIGNED));
#endif
        ioctl(fd, VIDIOC_QBUF, &b);
    }
    ioctl(fd, VIDIOC_STREAMON, &type);

#if TEST_PATTERN
    {
        struct v4l2_ext_control c = { .id = V4L2_CID_TEST_PATTERN, .value = 1 };
        struct v4l2_ext_controls cs = { .ctrl_class = V4L2_CTRL_CLASS_USER, .count = 1, .controls = &c };
        if (ioctl(fd, VIDIOC_S_EXT_CTRLS, &cs) != 0) {
            ESP_LOGW(TAG, "test pattern not accepted - capturing the live scene instead");
        } else {
            ESP_LOGW(TAG, "SENSOR TEST PATTERN ON - expect colour bars, not a photo");
        }
    }
#endif

    /*
     * Where the lens is before anything has adjusted it. A -1 here means the
     * VCM was not detected, so every frame below will be at the lens's rest
     * position regardless of how the rest of the pipeline is tuned.
     */
    int focus_start = focus_get(fd);
    if (focus_start < 0) {
        ESP_LOGW(TAG, "no focus control on this device - autofocus is not running");
    } else {
        ESP_LOGI(TAG, "lens parked at code %d", focus_start);
    }

    struct v4l2_buffer buf;

#if FOCUS_SWEEP
    focus_sweep(fd, type, buffer, w, h);
    goto done;
#endif

    ESP_LOGI(TAG, "aim the camera — settling for %d s...", AIM_SECONDS);
    uint32_t t_end = esp_log_timestamp() + AIM_SECONDS * 1000;
    /*
     * Recycle frames for the aim window so the ISP's AE, AWB and autofocus can
     * converge, then keep the last one.
     */
    /*
     * Trace every lens move during the settle window. A converging search steps
     * a few times and stops; a lens that never moves prints nothing; one that
     * keeps stepping until the window closes is still hunting, which means the
     * capture below is a frame from the middle of a search rather than from a
     * decided focus position.
     */
    int focus_prev = focus_start;
    do {
        buf = (struct v4l2_buffer){ .type = type, .memory = V4L2_MEMORY_MMAP };
        if (ioctl(fd, VIDIOC_DQBUF, &buf) != 0) { ESP_LOGE(TAG, "DQBUF failed"); goto done; }
        if (focus_start >= 0) {
            int focus_now = focus_get(fd);
            /*
             * Ignore read failures rather than treating -1 as a new position:
             * otherwise one failed ioctl logs a move to -1 and the next frame
             * logs a move back, flapping once per frame for the whole window.
             */
            if (focus_now >= 0 && focus_now != focus_prev) {
                ESP_LOGI(TAG, "  t=%4" PRIu32 " ms  focus %d -> %d",
                         esp_log_timestamp() - (t_end - AIM_SECONDS * 1000), focus_prev, focus_now);
                focus_prev = focus_now;
            }
        }
        if (esp_log_timestamp() < t_end) {
            ioctl(fd, VIDIOC_QBUF, &buf);            /* discard, keep streaming */
        }
    } while (esp_log_timestamp() < t_end);

    ESP_LOGI(TAG, "captured frame: bytesused=%" PRIu32 " (expected %" PRIu32 ")",
             buf.bytesused, w * h * 2);

    if (focus_start >= 0) {
        int focus_end = focus_get(fd);
        /*
         * Sharpness of the frame we are about to keep, on the same scale the
         * sweep prints - so a soft-looking result can be compared against the
         * calibration table instead of argued about by eye.
         */
        ESP_LOGI(TAG, "focus: %d -> %d, centre sharpness %" PRIu32,
                 focus_start, focus_end, centre_sharpness(buffer[buf.index], w, h));
        if (focus_end == focus_start) {
            ESP_LOGW(TAG, "autofocus never moved the lens - check that the IPA config has "
                          "an \"af\" block and that ESP_VIDEO_ISP_PIPELINE_CONTROL_CAMERA_MOTOR is set");
        }
    }
#if POISON_BUFFERS
    {
        const uint8_t *p = buffer[buf.index];
        uint32_t n = buf_len[buf.index], untouched = 0;
        for (uint32_t i = 0; i < n; i++) {
            if (p[i] == POISON_BYTE) {
                untouched++;
            }
        }
        uint32_t pct = (uint32_t)((uint64_t)untouched * 100 / n);
        ESP_LOGI(TAG, "poison check: %" PRIu32 "%% of the buffer still reads 0x%02x", pct, POISON_BYTE);
        if (pct > 90) {
            ESP_LOGE(TAG, "the capture path wrote (almost) nothing - the BMP is stale memory, not a photo");
        }
    }
#endif
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
