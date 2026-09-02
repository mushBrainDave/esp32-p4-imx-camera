/*
 * SPDX-FileCopyrightText: 2026 esp_cam_sensor_imx contributors
 *
 * SPDX-License-Identifier: Apache-2.0
 *
 * IMX219 single-frame snapshot for the Waveshare ESP32-P4-WIFI6.
 *
 * Streams the IMX219 (Raspberry Pi Camera Module v2 / NoIR v2), holds for a few
 * seconds so the ISP's auto-exposure and white balance can converge, then keeps
 * one frame and ships it down the console UART as a JPEG. tools/capture.py at
 * the repo root receives it and writes the .jpg.
 *
 * This is the IMX708 snapshot example with the autofocus removed - the v2
 * module is fixed-focus and has no VCM on the bus - and an AE convergence trace
 * put in its place. That trace is the closest thing this sensor has to the
 * IMX708's focus log: a mean luma that moves for the first few frames and then
 * settles is the runtime proof that the IPA config actually loaded, and a
 * column that never moves at all means it did not.
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
#include "esp_heap_caps.h"
#include "driver/jpeg_encode.h"

#include "imx_serial_img.h"

/* ---- Camera pins (Waveshare ESP32-P4-WIFI6) ----------------------------- */
#define CAM_SCCB_I2C_PORT   0
#define CAM_SCCB_SCL_PIN    8
#define CAM_SCCB_SDA_PIN    7
#define CAM_SCCB_FREQ_HZ    100000
/* The Pi 15-pin CSI connector routes neither to the host; the sensor free-runs
   on its own oscillator, so there is no XCLK for us to drive either. */
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

#define OUT_PATH            SD_MOUNT_POINT "/imx219.bmp"

/*
 * Settling time before the keeper frame.
 *
 * Shorter than the IMX708 example's, which had to wait out a contrast-AF search
 * (a lens move plus a settle for each of ~20 scan points) on top of AE. Here
 * only AE and AWB have to converge, and both are frame-rate-limited feedback
 * loops rather than mechanical ones. The AE trace below prints what actually
 * happened, so if the luma column is still climbing when the window closes,
 * raise this rather than guessing.
 */
#define AIM_SECONDS         4

/*
 * Where captured frames go.
 *
 * IMAGE_OUT_SERIAL sends them down the same USB cable that carries the log, so
 * a capture no longer means powering off and carrying the microSD to a PC. The
 * console runs at 2 Mbaud (see sdkconfig.defaults) to make that bearable: a
 * JPEG is a couple of seconds, a raw frame about twenty.
 *
 * IMAGE_OUT_SD keeps a BMP-to-card path as a fallback for when the host end is
 * unavailable. Off by default; it costs ~8 s per frame.
 */
#define IMAGE_OUT_SERIAL    1
#define IMAGE_OUT_SD        0

#define JPEG_QUALITY        90

/*
 * TEST_PATTERN: ask the sensor for its internal colour bars. The bars are
 * generated after the pixel array, so they travel the whole MIPI -> CSI -> ISP
 * path. Clean bars mean the transport is healthy and any bad picture is
 * optics/exposure/ISP tuning; noisy bars mean the fault is upstream of the ISP
 * (link rate, lane count, data type, sensor mode).
 */
#define TEST_PATTERN        0

/*
 * POISON_BUFFERS: fill each capture buffer with a known byte before streaming
 * and count how much survives. A frame that comes back ~100% poison means
 * nothing was DMA'd into it and the "photo" is untouched PSRAM - which is worth
 * being able to prove, because untouched PSRAM has a mid-scale byte average
 * that is easy to mistake for a real mid-exposure frame.
 */
#define POISON_BUFFERS      0
#define POISON_BYTE         0xA5

#define CAM_DEV_PATH        ESP_VIDEO_MIPI_CSI_DEVICE_NAME
#define BUFFER_COUNT        2

static const char *TAG = "imx219_snapshot";

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
 * No .cam_motor entry, unlike the IMX708 example. The v2 module is fixed-focus:
 * an I2C scan with one plugged in answers at 0x10 (sensor), 0x18 (the board's
 * audio codec) and 0x64 (PMIC) only - there is no VCM at 0x0c to drive.
 */
static const esp_video_init_config_t cam_config = {
    .csi = csi_config,
};

#if IMAGE_OUT_SD
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
#endif /* IMAGE_OUT_SD */

/* ---- Serial image transport --------------------------------------------- */
#if IMAGE_OUT_SERIAL
/*
 * The wire format and the three ways it can be silently corrupted live in
 * components/imx_serial_img, so the examples cannot drift apart on them. Short
 * version, since it is what the receiver keys on:
 *
 *   IMGSTART name=<n> fmt=<jpeg|rgb565> w=<w> h=<h> len=<N> crc32=<hex>
 *   <exactly N raw bytes>
 *   IMGEND
 */
static esp_err_t serial_send_jpeg(const char *name, const uint8_t *rgb565, uint32_t w, uint32_t h)
{
    jpeg_encoder_handle_t enc = NULL;
    jpeg_encode_engine_cfg_t eng = { .timeout_ms = 5000 };
    esp_err_t ret = jpeg_new_encoder_engine(&eng, &enc);
    ESP_RETURN_ON_ERROR(ret, TAG, "jpeg engine");

    /* Worst case a JPEG can exceed a naive guess, so allow half the raw size. */
    size_t cap = w * h;
    jpeg_encode_memory_alloc_cfg_t mem = { .buffer_direction = JPEG_ENC_ALLOC_OUTPUT_BUFFER };
    size_t out_alloc = 0;
    uint8_t *out = jpeg_alloc_encoder_mem(cap, &mem, &out_alloc);
    if (!out) {
        jpeg_del_encoder_engine(enc);
        ESP_LOGE(TAG, "no memory for JPEG output");
        return ESP_ERR_NO_MEM;
    }

    /*
     * YUV422, not 420. 420 needs a height that is a multiple of its 16-pixel
     * MCU; 1232 is 77*16 so it would divide cleanly here, but 422 is kept to
     * match the other examples and it holds more chroma detail on a frame that
     * may still get measured.
     */
    jpeg_encode_cfg_t cfg = {
        .width = w,
        .height = h,
        .src_type = JPEG_ENCODE_IN_FORMAT_RGB565,
        .sub_sample = JPEG_DOWN_SAMPLING_YUV422,
        .image_quality = JPEG_QUALITY,
    };
    uint32_t out_len = 0;
    uint32_t t0 = esp_log_timestamp();
    ret = jpeg_encoder_process(enc, &cfg, rgb565, w * h * 2, out, out_alloc, &out_len);
    if (ret == ESP_OK) {
        ESP_LOGI(TAG, "JPEG q%d: %" PRIu32 " -> %" PRIu32 " bytes in %" PRIu32 " ms",
                 JPEG_QUALITY, w * h * 2, out_len, esp_log_timestamp() - t0);
        ret = imx_serial_send_blob(name, "jpeg", w, h, out, out_len, NULL);
    } else {
        ESP_LOGE(TAG, "JPEG encode failed: %s", esp_err_to_name(ret));
    }

    free(out);
    jpeg_del_encoder_engine(enc);
    return ret;
}
#endif /* IMAGE_OUT_SERIAL */

/*
 * Staging copy of the frame.
 *
 * The camera does not stop streaming while a frame is being sent. With
 * BUFFER_COUNT buffers and one of them dequeued in our hand, the driver runs
 * out of places to put incoming frames and recycles the one still being read -
 * so the tail of a long transfer arrives as newer frame data while the CRC was
 * computed over the original. Copying into private memory and requeueing
 * immediately decouples transfer time from the capture pipeline. PSRAM is
 * 32 MB; one frame is cheap.
 */
static uint8_t *s_stage = NULL;

static const uint8_t *stage_frame(int fd, struct v4l2_buffer *buf, uint32_t w, uint32_t h,
                                  uint8_t **buffer)
{
    size_t len = (size_t)w * h * 2;
    if (!s_stage) {
        s_stage = heap_caps_malloc(len, MALLOC_CAP_SPIRAM | MALLOC_CAP_8BIT);
    }
    if (!s_stage) {
        ESP_LOGE(TAG, "no PSRAM for the staging frame; sending from the live buffer");
        return buffer[buf->index];
    }
    memcpy(s_stage, buffer[buf->index], len);
    ioctl(fd, VIDIOC_QBUF, buf);   /* hand the capture buffer straight back */
    return s_stage;
}

/*
 * Read one V4L2 control, or -1 if it is not available.
 *
 * esp_video routes S_/G_EXT_CTRLS only - it has no VIDIOC_G_CTRL case at all -
 * so the ext-controls form is the only one that works here, and it is also the
 * path the AE itself drives these two controls through.
 */
static int ctrl_get(int fd, uint32_t id)
{
    struct v4l2_ext_control c = { .id = id };
    struct v4l2_ext_controls cs = { .ctrl_class = V4L2_CTRL_CLASS_USER, .count = 1, .controls = &c };

    if (ioctl(fd, VIDIOC_G_EXT_CTRLS, &cs) != 0) {
        return -1;
    }
    return c.value;
}

/*
 * Mean luma of the frame, on a 0-255 scale, from the RGB565 buffer.
 *
 * Rec.601 weights on the expanded channels. Subsampled to a few thousand pixels
 * because this runs once per frame inside the aim loop and the number only has
 * to be good enough to show AE converging, not to be photometric.
 */
static uint32_t frame_luma(const uint8_t *rgb565, uint32_t w, uint32_t h)
{
    const uint16_t *px = (const uint16_t *)rgb565;
    uint32_t n = w * h;
    uint32_t step = n > 4096 ? n / 4096 : 1;
    uint64_t sum = 0;
    uint32_t cnt = 0;

    for (uint32_t i = 0; i < n; i += step) {
        uint16_t p = px[i];
        uint32_t r = ((p >> 11) & 0x1f) << 3;
        uint32_t g = ((p >> 5) & 0x3f) << 2;
        uint32_t b = (p & 0x1f) << 3;
        sum += (r * 299 + g * 587 + b * 114) / 1000;
        cnt++;
    }
    return cnt ? (uint32_t)(sum / cnt) : 0;
}

void app_main(void)
{
    ESP_LOGI(TAG, "IMX219 snapshot");

#if IMAGE_OUT_SD
    if (sd_mount() != ESP_OK) {
        return;
    }
#endif
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
    ESP_LOGI(TAG, "format %" PRIu32 "x%" PRIu32 " fourcc=%c%c%c%c (0x%08" PRIx32 ")",
             w, h, (char)(fourcc & 0xff), (char)((fourcc >> 8) & 0xff),
             (char)((fourcc >> 16) & 0xff), (char)((fourcc >> 24) & 0xff), fourcc);
    if (fourcc != V4L2_PIX_FMT_RGB565) {
        ESP_LOGE(TAG, "expected RGB565 from the ISP - the frame below will be garbage");
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

    struct v4l2_buffer buf;

    /*
     * The AE's ceilings, so the trace below can be read against them. Exposure
     * is capped at the mode's frame length (VTS - 4), not at the register
     * width, and gain at the last entry of the driver's gain menu. A trace that
     * flattens with both of these reached is an AE that has run out of road in
     * a dark scene; one that flattens well short of them has decided to stop,
     * which is a tuning question, not a limit.
     */
    {
        struct v4l2_query_ext_ctrl q = { .id = V4L2_CID_EXPOSURE };
        if (ioctl(fd, VIDIOC_QUERY_EXT_CTRL, &q) == 0) {
            ESP_LOGI(TAG, "AE headroom: exposure %lld..%lld lines",
                     (long long)q.minimum, (long long)q.maximum);
        }
        struct v4l2_querymenu qm = { .id = V4L2_CID_GAIN };
        int n = 0;
        long long top = 0;
        for (qm.index = 0; ioctl(fd, VIDIOC_QUERYMENU, &qm) == 0; qm.index++) {
            top = qm.value;
            n++;
        }
        if (n) {
            ESP_LOGI(TAG, "AE headroom: gain menu 0..%d, max %lld milli", n - 1, top);
        }
    }

    ESP_LOGI(TAG, "aim the camera — settling for %d s...", AIM_SECONDS);
    ESP_LOGI(TAG, "   t (ms) | luma | exp | gain_idx");

    /*
     * Recycle frames for the aim window so AE and AWB converge, then keep the
     * last one. The luma column is the diagnostic: it should move over the
     * first frames and then flatten. A column that never moves at all means the
     * IPA config did not load and nothing is driving exposure - the same
     * condition esp_video reports upstream as "failed to get configuration to
     * initialize ISP controller", which is easy to miss in the boot log.
     */
    uint32_t t_start = esp_log_timestamp();
    uint32_t t_end = t_start + AIM_SECONDS * 1000;
    uint32_t luma_prev = UINT32_MAX;
    int exp_prev = -2, gain_prev = -2;   /* -1 is "no such control", so start outside it */
    uint32_t luma_first = 0, luma_last = 0;
    bool first = true;
    do {
        buf = (struct v4l2_buffer){ .type = type, .memory = V4L2_MEMORY_MMAP };
        if (ioctl(fd, VIDIOC_DQBUF, &buf) != 0) { ESP_LOGE(TAG, "DQBUF failed"); goto done; }

        uint32_t luma = frame_luma(buffer[buf.index], w, h);
        luma_last = luma;
        if (first) { luma_first = luma; first = false; }
        /*
         * Print on any movement, in luma OR in what AE is asking the sensor
         * for. Luma alone is ambiguous: a flat column can mean AE converged,
         * or that it is still pushing and the scene will not respond.
         */
        int exp = ctrl_get(fd, V4L2_CID_EXPOSURE);
        int gidx = ctrl_get(fd, V4L2_CID_GAIN);
        if (luma_prev == UINT32_MAX || luma > luma_prev + 1 || luma + 1 < luma_prev
                || exp != exp_prev || gidx != gain_prev) {
            ESP_LOGI(TAG, "   %6" PRIu32 " | %4" PRIu32 " | %4d | %d",
                     esp_log_timestamp() - t_start, luma, exp, gidx);
            luma_prev = luma;
            exp_prev = exp;
            gain_prev = gidx;
        }

        if (esp_log_timestamp() < t_end) {
            ioctl(fd, VIDIOC_QBUF, &buf);            /* discard, keep streaming */
        }
    } while (esp_log_timestamp() < t_end);

    ESP_LOGI(TAG, "captured frame: bytesused=%" PRIu32 " (expected %" PRIu32 ")",
             buf.bytesused, w * h * 2);
    if (luma_first == luma_last) {
        ESP_LOGW(TAG, "mean luma never moved (%" PRIu32 ") - AE is probably not running. "
                      "Check for 'failed to get configuration to initialize ISP controller' "
                      "above, which means the IMX219 IPA config did not load.",
                 luma_last);
    } else {
        ESP_LOGI(TAG, "AE settled: luma %" PRIu32 " -> %" PRIu32 ", exposure %d lines, gain index %d",
                 luma_first, luma_last, ctrl_get(fd, V4L2_CID_EXPOSURE), ctrl_get(fd, V4L2_CID_GAIN));
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
            ESP_LOGE(TAG, "the capture path wrote (almost) nothing - this is stale memory, not a photo");
        }
    }
#endif
    {
        const uint8_t *frame = stage_frame(fd, &buf, w, h, buffer);
#if IMAGE_OUT_SERIAL
        serial_send_jpeg("imx219", frame, w, h);
#endif
#if IMAGE_OUT_SD
        save_bmp565(OUT_PATH, frame, w, h);
#else
        (void)frame;
#endif
    }

done:
    ioctl(fd, VIDIOC_STREAMOFF, &type);
    close(fd);
    esp_video_deinit();
#if IMAGE_OUT_SD
    esp_vfs_fat_sdcard_unmount(SD_MOUNT_POINT, s_card);
    if (s_pwr_handle) { sd_pwr_ctrl_del_on_chip_ldo(s_pwr_handle); }
    ESP_LOGI(TAG, "==== done — remove the SD card and open imx219.bmp on your PC ====");
#else
    ESP_LOGI(TAG, "==== done — frame sent over serial, no card needed ====");
#endif
}
