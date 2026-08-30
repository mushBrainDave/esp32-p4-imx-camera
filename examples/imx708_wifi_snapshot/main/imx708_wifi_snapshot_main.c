/*
 * SPDX-FileCopyrightText: 2026 esp_cam_sensor_imx contributors
 *
 * SPDX-License-Identifier: Apache-2.0
 *
 * IMX708 stills over WiFi: camera + STA connect + an HTTP server on the board.
 *
 *   GET /               a page with the picture on it
 *   GET /snapshot.jpg   the newest frame, hardware-JPEG encoded
 *   GET /snapshot.raw   the newest frame as raw RGB565 (~4 MB)
 *   GET /bench          8 MB of pattern, no camera in the loop
 *   GET /stats          timings from the last request, as text
 *
 * Why a still first, before video: nothing has measured *throughput* over this
 * link. `c6_wifi_sta` measured latency - 21 ms round-trips - which says nothing
 * about sustained MB/s through SDIO -> C6 -> air. A single frame is a
 * self-contained blob with a known length, so it gives a byte-exact pass/fail
 * and a throughput number in one run, and that number is what decides how the
 * video path should be built. `/bench` separates the network from the camera:
 * if a JPEG is slower than 8 MB of PSRAM at the same size, the cost is in the
 * capture path, not the radio.
 *
 * No microSD here, deliberately: SD and esp_hosted share the SDMMC peripheral
 * and hit ESP-IDF issue 16233. Serial capture replaced the card months ago, and
 * now the network replaces that, so nothing is lost by leaving it out.
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
#include "esp_timer.h"
#include "esp_heap_caps.h"
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "freertos/semphr.h"
#include "linux/videodev2.h"
#include "esp_video_init.h"
#include "esp_video_device.h"
#include "driver/jpeg_encode.h"
#include "esp_http_server.h"

#include "imx_wifi.h"

/* ---- Camera pins (Waveshare ESP32-P4-WIFI6) ----------------------------- */
#define CAM_SCCB_I2C_PORT   0
#define CAM_SCCB_SCL_PIN    8
#define CAM_SCCB_SDA_PIN    7
#define CAM_SCCB_FREQ_HZ    100000
#define CAM_RESET_PIN       (-1)
#define CAM_PWDN_PIN        (-1)

#define CAM_DEV_PATH        ESP_VIDEO_MIPI_CSI_DEVICE_NAME
#define BUFFER_COUNT        2

#define JPEG_QUALITY        90

/*
 * How much of a response to hand LwIP at a time. Too small and the per-call
 * overhead dominates; too large and a chunk sits in PSRAM waiting on the
 * window. 16 KB is comfortably above the ~64 KB TCP window divided by a few
 * segments in flight, and is what the throughput figures below were measured
 * with - change it and re-measure rather than assuming.
 */
#define SEND_CHUNK          (16 * 1024)

/* Pure-network measurement, no camera and no encoder. */
#define BENCH_BYTES         (8 * 1024 * 1024)

static const char *TAG = "imx708_wifi";

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
 * esp_video probes for it separately and needs its own entry - without one the
 * motor auto-detect array is never walked and the lens is never driven.
 */
static const esp_video_init_cam_motor_config_t motor_config[] = {{
    .sccb_config = {
        .init_sccb = true,
        .i2c_config = { .port = CAM_SCCB_I2C_PORT, .scl_pin = CAM_SCCB_SCL_PIN, .sda_pin = CAM_SCCB_SDA_PIN },
        .freq = CAM_SCCB_FREQ_HZ,
    },
    .reset_pin  = -1,
    .pwdn_pin   = -1,
    .signal_pin = -1,
}};

static const esp_video_init_config_t cam_config = {
    .csi = csi_config,
    .cam_motor = motor_config,
};

/* ---- The published frame ------------------------------------------------ */
/*
 * A capture task keeps the pipeline running and republishes the newest frame
 * here; the HTTP handlers only ever read this copy.
 *
 * Serving straight from a dequeued V4L2 buffer would be simpler and wrong. The
 * camera does not stop while a response is on the wire, and with BUFFER_COUNT
 * buffers - one of them held by the handler - the driver runs out of places to
 * put incoming frames and recycles the one being read. That is exactly how the
 * serial path corrupted its first raw sends: clean at the top, junk at the
 * bottom, CRC computed over something else again.
 *
 * Keeping the stream running between requests also means AE, AWB and autofocus
 * stay converged, so a request returns a settled frame rather than the first
 * frame after a cold start.
 */
static uint8_t *s_frame;              /* newest complete frame, RGB565 */
static size_t s_frame_len;
static uint32_t s_frame_seq;          /* increments on every publish */
static SemaphoreHandle_t s_frame_lock;
static uint32_t s_w, s_h;

/* Last request's timings, for /stats. */
static struct {
    uint32_t bytes;
    uint32_t encode_ms;
    uint32_t send_ms;
    char what[16];
} s_last;

static void note_timing(const char *what, uint32_t bytes, uint32_t encode_ms, uint32_t send_ms)
{
    strncpy(s_last.what, what, sizeof(s_last.what) - 1);
    s_last.what[sizeof(s_last.what) - 1] = '\0';
    s_last.bytes = bytes;
    s_last.encode_ms = encode_ms;
    s_last.send_ms = send_ms;

    /*
     * bits per millisecond is kbit/s, so bytes*8/ms needs no scaling. Printed
     * as Mbit/s to one decimal in integer arithmetic.
     */
    uint32_t kbit_s = send_ms ? (uint32_t)(((uint64_t)bytes * 8) / send_ms) : 0;
    ESP_LOGI(TAG, "%s: %" PRIu32 " bytes, encode %" PRIu32 " ms, send %" PRIu32
                  " ms -> %" PRIu32 ".%" PRIu32 " Mbit/s",
             what, bytes, encode_ms, send_ms, kbit_s / 1000, (kbit_s / 100) % 10);
}

/* ---- Capture ------------------------------------------------------------ */

struct capture_ctx {
    int fd;
    uint8_t **buffer;   /* the mmap'd V4L2 buffer table, indexed by buf.index */
};

static void capture_task(void *arg)
{
    struct capture_ctx *ctx = arg;
    const int type = V4L2_BUF_TYPE_VIDEO_CAPTURE;

    while (1) {
        struct v4l2_buffer buf = { .type = type, .memory = V4L2_MEMORY_MMAP };
        if (ioctl(ctx->fd, VIDIOC_DQBUF, &buf) != 0) {
            ESP_LOGE(TAG, "DQBUF failed - capture task stopping");
            vTaskDelete(NULL);
            return;
        }

        if (xSemaphoreTake(s_frame_lock, pdMS_TO_TICKS(1000)) == pdTRUE) {
            memcpy(s_frame, ctx->buffer[buf.index], s_frame_len);
            s_frame_seq++;
            xSemaphoreGive(s_frame_lock);
        }
        /* Straight back to the driver: the publish above is a memcpy, so the
         * buffer is never held across anything slow. */
        ioctl(ctx->fd, VIDIOC_QBUF, &buf);
    }
}

/* ---- Handlers ----------------------------------------------------------- */

/*
 * Encode from the published frame under the lock, then send with the lock
 * released. The encode is tens of milliseconds and costs the capture task at
 * most a dropped frame; the send is hundreds of milliseconds and must never
 * block the pipeline.
 */
static esp_err_t snapshot_jpg_handler(httpd_req_t *req)
{
    jpeg_encoder_handle_t enc = NULL;
    jpeg_encode_engine_cfg_t eng = { .timeout_ms = 5000 };
    if (jpeg_new_encoder_engine(&eng, &enc) != ESP_OK) {
        httpd_resp_send_err(req, HTTPD_500_INTERNAL_SERVER_ERROR, "jpeg engine");
        return ESP_FAIL;
    }

    size_t cap = s_w * s_h;   /* half the raw size; a q90 frame is far under it */
    jpeg_encode_memory_alloc_cfg_t mem = { .buffer_direction = JPEG_ENC_ALLOC_OUTPUT_BUFFER };
    size_t out_alloc = 0;
    uint8_t *out = jpeg_alloc_encoder_mem(cap, &mem, &out_alloc);
    if (!out) {
        jpeg_del_encoder_engine(enc);
        httpd_resp_send_err(req, HTTPD_500_INTERNAL_SERVER_ERROR, "no memory");
        return ESP_FAIL;
    }

    /*
     * Subsampling is YUV422, not the more usual 420: 1080 is not a multiple of
     * the 16-pixel MCU height 420 needs (it is a multiple of 8, so 422 divides
     * cleanly), and 422 keeps more chroma detail.
     */
    jpeg_encode_cfg_t cfg = {
        .width = s_w,
        .height = s_h,
        .src_type = JPEG_ENCODE_IN_FORMAT_RGB565,
        .sub_sample = JPEG_DOWN_SAMPLING_YUV422,
        .image_quality = JPEG_QUALITY,
    };
    uint32_t out_len = 0;
    esp_err_t ret = ESP_FAIL;
    int64_t t0 = esp_timer_get_time();

    if (xSemaphoreTake(s_frame_lock, pdMS_TO_TICKS(3000)) == pdTRUE) {
        ret = jpeg_encoder_process(enc, &cfg, s_frame, s_frame_len, out, out_alloc, &out_len);
        xSemaphoreGive(s_frame_lock);
    }
    uint32_t encode_ms = (uint32_t)((esp_timer_get_time() - t0) / 1000);

    if (ret != ESP_OK) {
        free(out);
        jpeg_del_encoder_engine(enc);
        httpd_resp_send_err(req, HTTPD_500_INTERNAL_SERVER_ERROR, "encode failed");
        return ESP_FAIL;
    }

    httpd_resp_set_type(req, "image/jpeg");
    /* No caching: every request should get the live scene, not the last one. */
    httpd_resp_set_hdr(req, "Cache-Control", "no-store");

    int64_t t1 = esp_timer_get_time();
    ret = httpd_resp_send(req, (const char *)out, out_len);
    uint32_t send_ms = (uint32_t)((esp_timer_get_time() - t1) / 1000);

    note_timing("jpeg", out_len, encode_ms, send_ms);

    free(out);
    jpeg_del_encoder_engine(enc);
    return ret;
}

/*
 * The raw frame, chunked. Copies out under the lock rather than holding it for
 * the whole transfer - a 4 MB send is far too long to stall the capture task.
 */
static uint8_t *s_send_copy;

static esp_err_t snapshot_raw_handler(httpd_req_t *req)
{
    if (!s_send_copy) {
        s_send_copy = heap_caps_malloc(s_frame_len, MALLOC_CAP_SPIRAM | MALLOC_CAP_8BIT);
    }
    if (!s_send_copy) {
        httpd_resp_send_err(req, HTTPD_500_INTERNAL_SERVER_ERROR, "no PSRAM");
        return ESP_FAIL;
    }

    if (xSemaphoreTake(s_frame_lock, pdMS_TO_TICKS(3000)) != pdTRUE) {
        httpd_resp_send_err(req, HTTPD_500_INTERNAL_SERVER_ERROR, "frame busy");
        return ESP_FAIL;
    }
    memcpy(s_send_copy, s_frame, s_frame_len);
    xSemaphoreGive(s_frame_lock);

    httpd_resp_set_type(req, "application/octet-stream");
    /* The receiver needs the geometry to make sense of the bytes, and RGB565
     * carries none of its own. */
    char dim[32];
    snprintf(dim, sizeof(dim), "%" PRIu32 "x%" PRIu32, s_w, s_h);
    httpd_resp_set_hdr(req, "X-Frame-Size", dim);
    httpd_resp_set_hdr(req, "X-Frame-Format", "RGB565");

    int64_t t0 = esp_timer_get_time();
    size_t sent = 0;
    while (sent < s_frame_len) {
        size_t n = s_frame_len - sent;
        if (n > SEND_CHUNK) {
            n = SEND_CHUNK;
        }
        if (httpd_resp_send_chunk(req, (const char *)s_send_copy + sent, n) != ESP_OK) {
            ESP_LOGW(TAG, "raw send aborted after %u bytes", (unsigned)sent);
            return ESP_FAIL;
        }
        sent += n;
    }
    httpd_resp_send_chunk(req, NULL, 0);
    uint32_t send_ms = (uint32_t)((esp_timer_get_time() - t0) / 1000);

    note_timing("raw", (uint32_t)s_frame_len, 0, send_ms);
    return ESP_OK;
}

/*
 * Network only: one PSRAM buffer sent over and over. No camera, no encoder, no
 * per-chunk work at all, so whatever this reports is the ceiling the other two
 * endpoints are measured against.
 */
static esp_err_t bench_handler(httpd_req_t *req)
{
    uint8_t *block = heap_caps_malloc(SEND_CHUNK, MALLOC_CAP_SPIRAM | MALLOC_CAP_8BIT);
    if (!block) {
        httpd_resp_send_err(req, HTTPD_500_INTERNAL_SERVER_ERROR, "no PSRAM");
        return ESP_FAIL;
    }
    /* A counter, not zeros: a compressing middlebox or a driver that elides
     * runs of zeros would otherwise flatter the result. */
    for (size_t i = 0; i < SEND_CHUNK; i++) {
        block[i] = (uint8_t)(i * 31u + (i >> 8));
    }

    httpd_resp_set_type(req, "application/octet-stream");

    int64_t t0 = esp_timer_get_time();
    size_t sent = 0;
    while (sent < BENCH_BYTES) {
        size_t n = BENCH_BYTES - sent;
        if (n > SEND_CHUNK) {
            n = SEND_CHUNK;
        }
        if (httpd_resp_send_chunk(req, (const char *)block, n) != ESP_OK) {
            ESP_LOGW(TAG, "bench aborted after %u bytes", (unsigned)sent);
            free(block);
            return ESP_FAIL;
        }
        sent += n;
    }
    httpd_resp_send_chunk(req, NULL, 0);
    uint32_t send_ms = (uint32_t)((esp_timer_get_time() - t0) / 1000);

    note_timing("bench", BENCH_BYTES, 0, send_ms);
    free(block);
    return ESP_OK;
}

static esp_err_t stats_handler(httpd_req_t *req)
{
    char body[320];
    uint32_t kbit_s = s_last.send_ms
        ? (uint32_t)(((uint64_t)s_last.bytes * 8) / s_last.send_ms) : 0;
    int n = snprintf(body, sizeof(body),
                     "last=%s\nbytes=%" PRIu32 "\nencode_ms=%" PRIu32
                     "\nsend_ms=%" PRIu32 "\nmbit_s=%" PRIu32 ".%" PRIu32
                     "\nframes_published=%" PRIu32 "\nheap_free=%u\npsram_free=%u\n",
                     s_last.what[0] ? s_last.what : "none",
                     s_last.bytes, s_last.encode_ms, s_last.send_ms,
                     kbit_s / 1000, (kbit_s / 100) % 10, s_frame_seq,
                     (unsigned)heap_caps_get_free_size(MALLOC_CAP_INTERNAL),
                     (unsigned)heap_caps_get_free_size(MALLOC_CAP_SPIRAM));
    httpd_resp_set_type(req, "text/plain");
    return httpd_resp_send(req, body, n);
}

static esp_err_t index_handler(httpd_req_t *req)
{
    static const char page[] =
        "<!doctype html><meta name=viewport content=\"width=device-width,initial-scale=1\">"
        "<title>IMX708</title>"
        "<style>body{margin:0;background:#111;color:#eee;font:14px system-ui;text-align:center}"
        "img{max-width:100%;height:auto}a{color:#6cf}</style>"
        "<p><img id=v src=\"/snapshot.jpg\">"
        "<p><button onclick=\"document.getElementById('v').src='/snapshot.jpg?'+Date.now()\">"
        "refresh</button> &middot; <a href=/stats>stats</a> &middot; <a href=/bench>bench</a>";
    httpd_resp_set_type(req, "text/html");
    return httpd_resp_send(req, page, sizeof(page) - 1);
}

static httpd_handle_t start_server(void)
{
    httpd_config_t cfg = HTTPD_DEFAULT_CONFIG();
    /* The JPEG encoder and the handlers need more than the 4 KB default. */
    cfg.stack_size = 8192;
    cfg.lru_purge_enable = true;
    /* A browser opens several sockets; without headroom the page and the image
     * request each other out. */
    cfg.max_open_sockets = 4;
    cfg.send_wait_timeout = 10;
    cfg.recv_wait_timeout = 10;

    httpd_handle_t server = NULL;
    if (httpd_start(&server, &cfg) != ESP_OK) {
        ESP_LOGE(TAG, "httpd_start failed");
        return NULL;
    }

    static const httpd_uri_t uris[] = {
        { .uri = "/",             .method = HTTP_GET, .handler = index_handler },
        { .uri = "/snapshot.jpg", .method = HTTP_GET, .handler = snapshot_jpg_handler },
        { .uri = "/snapshot.raw", .method = HTTP_GET, .handler = snapshot_raw_handler },
        { .uri = "/bench",        .method = HTTP_GET, .handler = bench_handler },
        { .uri = "/stats",        .method = HTTP_GET, .handler = stats_handler },
    };
    for (size_t i = 0; i < sizeof(uris) / sizeof(uris[0]); i++) {
        httpd_register_uri_handler(server, &uris[i]);
    }
    return server;
}

/* ---- Bring-up ----------------------------------------------------------- */

void app_main(void)
{
    ESP_LOGI(TAG, "IMX708 snapshot server over WiFi");

    /*
     * Credentials before the camera. Discovering they are missing after the
     * sensor is streaming wastes the run and buries the message under ISP logs.
     */
    if (!imx_wifi_have_credentials()) {
        (void)imx_wifi_connect(NULL, 0);   /* prints what to write */
        printf("\n==== done - no credentials ====\n");
        return;
    }

    if (esp_video_init(&cam_config) != ESP_OK) {
        ESP_LOGE(TAG, "esp_video_init failed");
        printf("\n==== done - camera init failed ====\n");
        return;
    }

    int fd = open(CAM_DEV_PATH, O_RDONLY);
    if (fd < 0) {
        ESP_LOGE(TAG, "open %s failed", CAM_DEV_PATH);
        printf("\n==== done - camera open failed ====\n");
        return;
    }

    const int type = V4L2_BUF_TYPE_VIDEO_CAPTURE;
    struct v4l2_format fmt = { .type = type };
    ioctl(fd, VIDIOC_G_FMT, &fmt);
    s_w = fmt.fmt.pix.width;
    s_h = fmt.fmt.pix.height;
    s_frame_len = (size_t)s_w * s_h * 2;
    ESP_LOGI(TAG, "format %" PRIu32 "x%" PRIu32 ", %u bytes/frame",
             s_w, s_h, (unsigned)s_frame_len);
    if (fmt.fmt.pix.pixelformat != V4L2_PIX_FMT_RGB565) {
        ESP_LOGE(TAG, "expected RGB565 from the ISP - the JPEG will be garbage");
    }

    static uint8_t *buffer[BUFFER_COUNT];
    struct v4l2_requestbuffers req = { .count = BUFFER_COUNT, .type = type, .memory = V4L2_MEMORY_MMAP };
    ioctl(fd, VIDIOC_REQBUFS, &req);
    for (int i = 0; i < BUFFER_COUNT; i++) {
        struct v4l2_buffer b = { .type = type, .memory = V4L2_MEMORY_MMAP, .index = i };
        ioctl(fd, VIDIOC_QUERYBUF, &b);
        buffer[i] = mmap(NULL, b.length, PROT_READ | PROT_WRITE, MAP_SHARED, fd, b.m.offset);
        ioctl(fd, VIDIOC_QBUF, &b);
    }

    s_frame = heap_caps_malloc(s_frame_len, MALLOC_CAP_SPIRAM | MALLOC_CAP_8BIT);
    s_frame_lock = xSemaphoreCreateMutex();
    if (!s_frame || !s_frame_lock) {
        ESP_LOGE(TAG, "no PSRAM for the published frame");
        printf("\n==== done - out of memory ====\n");
        return;
    }

    ioctl(fd, VIDIOC_STREAMON, &type);

    /*
     * WiFi after the camera is streaming, so the AE/AWB/AF convergence that
     * needs a few seconds of frames overlaps the association instead of
     * following it.
     */
    char ip[16] = {0};
    esp_err_t werr = imx_wifi_connect(ip, sizeof(ip));

    /* static, not a stack local: app_main returns while the task is still
     * running, and the task dereferences this for the life of the program. */
    static struct capture_ctx ctx;
    ctx.fd = fd;
    ctx.buffer = buffer;
    if (xTaskCreatePinnedToCore(capture_task, "capture", 4096, &ctx,
                                5, NULL, tskNO_AFFINITY) != pdPASS) {
        ESP_LOGE(TAG, "could not start the capture task");
        printf("\n==== done - capture task failed ====\n");
        return;
    }

    if (werr != ESP_OK) {
        printf("\n==== done - camera up, WiFi failed ====\n");
        return;
    }

    if (!start_server()) {
        printf("\n==== done - camera and WiFi up, server failed ====\n");
        return;
    }

    ESP_LOGI(TAG, "serving on http://%s/", ip);
    ESP_LOGI(TAG, "  http://%s/snapshot.jpg   http://%s/snapshot.raw   http://%s/bench",
             ip, ip, ip);

    /* The marker tools/capture.py waits for. Everything past this point happens
     * in the server, so the serial capture can stop here. */
    printf("\n==== done - serving at http://%s/ ====\n", ip);
}
