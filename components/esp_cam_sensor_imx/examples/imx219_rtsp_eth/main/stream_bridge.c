// SPDX-License-Identifier: Apache-2.0
#include <string.h>
#include <stdio.h>
#include <inttypes.h>
#include "freertos/FreeRTOS.h"
#include "freertos/semphr.h"
#include "freertos/task.h"
#include "esp_heap_caps.h"
#include "esp_timer.h"
#include "esp_log.h"
#include "esp_rtsp.h"
#include "media_lib_adapter.h"
#include "stream_bridge.h"
#include "rtsp_socket.h"
#include "rtsp_timer.h"
#include "rtsp_alloc.h"
#include "stream_limits.h"

#define SLOTS 3
#define MAX_AU (2 * 1024 * 1024)
static const char *TAG = "rtsp_bridge";
typedef struct {
    uint8_t *data;
    unsigned len;
    uint32_t pts;
    int64_t queued_us;
} unit_t;
static unit_t units[SLOTS];
static SemaphoreHandle_t lock, pause_ack;
static bool supervisor_started, listening;
static unsigned head, count;
static bool playing, wait_idr = true, reset_requested, ready;
static char ip[16];
static int width, height, fps;
static uint32_t encoded, errors, delivered, resyncs, max_encode_us, max_unit;
static uint64_t encode_us_total, scale_us_total;
static uint32_t max_scale_us, max_pipeline_us;
static uint32_t queue_full_resets, queue_age_resets, callback_capacity_resets;
static uint32_t queue_high_water, queue_age_max_us, callback_gap_max_us;
static uint32_t callback_copy_max_us, enqueue_copy_max_us, callback_wait_max_us;
static uint64_t callback_copy_total_us, callback_wait_total_us;
static int64_t last_callback_us;
static int exposure_lines = -1, gain_index = -1;
static unsigned sampled_luma;
static int64_t sample_us;
static esp_rtsp_handle_t server;
static esp_rtsp_video_info_t video;
static esp_rtsp_data_cb_t callbacks;
static esp_rtsp_config_t config;
static h264_params_t h264_params;

bool stream_h264_params(h264_params_t *params)
{
    xSemaphoreTake(lock, portMAX_DELAY);
    bool available = ready && h264_params_ready(&h264_params);
    if (available) *params = h264_params;
    xSemaphoreGive(lock);
    return available;
}

void stream_init(void)
{
    rtsp_socket_init();
    rtsp_timer_init();
    ESP_ERROR_CHECK(media_lib_add_default_adapter());
    lock = xSemaphoreCreateMutex();
    configASSERT(lock);
    pause_ack = xSemaphoreCreateBinary();
    configASSERT(pause_ack);
    for (int i = 0; i < SLOTS; ++i) {
        units[i].data = heap_caps_malloc(MAX_AU, MALLOC_CAP_SPIRAM | MALLOC_CAP_8BIT);
        configASSERT(units[i].data);
    }
}
void stream_set_ip(const char *address)
{
    xSemaphoreTake(lock, portMAX_DELAY);
    if (strcmp(ip, address)) reset_requested = true;
    snprintf(ip, sizeof(ip), "%s", address);
    xSemaphoreGive(lock);
}
static int state_changed(esp_rtsp_state_t state, void *ctx)
{
    (void)ctx;
    xSemaphoreTake(lock, portMAX_DELAY);
    if (state == RTSP_STATE_PLAY || state == RTSP_STATE_TEARDOWN) {
        playing = state == RTSP_STATE_PLAY;
        count = 0;
        head = 0;
        wait_idr = true;
        last_callback_us = 0;
    }
    xSemaphoreGive(lock);
    ESP_LOGI(TAG, "session state=%d", state);
    return 0;
}
static int send_video(unsigned char *data, unsigned int *len, uint32_t *pts, void *ctx)
{
    (void)ctx;
    int64_t entered = esp_timer_get_time();
    xSemaphoreTake(lock, portMAX_DELAY);
    int64_t now = esp_timer_get_time();
    uint32_t wait_us = (uint32_t)(now - entered);
    callback_wait_total_us += wait_us;
    if (wait_us > callback_wait_max_us) callback_wait_max_us = wait_us;
    if (!count || !playing) {
        *len = 0;
        xSemaphoreGive(lock);
        return 0;
    }
    unit_t *u = &units[head];
    uint32_t age_us = (uint32_t)(now - u->queued_us);
    if (age_us > queue_age_max_us) queue_age_max_us = age_us;
    if (last_callback_us && now - last_callback_us > callback_gap_max_us)
        callback_gap_max_us = (uint32_t)(now - last_callback_us);
    last_callback_us = now;
    bool expired = age_us > stream_queue_age_limit_ms(fps) * 1000;
    bool undersized = *len < u->len;
    if (expired || undersized) {
        if (expired) queue_age_resets++;
        if (undersized) callback_capacity_resets++;
        count = 0;
        wait_idr = true;
        reset_requested = true;
        resyncs++;
        *len = 0;
        xSemaphoreGive(lock);
        return 0;
    }
    int64_t copy_start = esp_timer_get_time();
    memcpy(data, u->data, u->len);
    uint32_t copy_us = (uint32_t)(esp_timer_get_time() - copy_start);
    callback_copy_total_us += copy_us;
    if (copy_us > callback_copy_max_us) callback_copy_max_us = copy_us;
    *len = u->len;
    *pts = u->pts; /* esp_rtsp callback uses milliseconds, as capture service does. */
    head = (head + 1) % SLOTS;
    count--;
    delivered++;
    xSemaphoreGive(lock);
    return 0;
}
void stream_encoded(const uint8_t *data, unsigned len, uint32_t pts_ms,
                    bool idr, uint32_t elapsed, uint32_t scale_us, bool ok)
{
    xSemaphoreTake(lock, portMAX_DELAY);
    if (!ok || len > MAX_AU) {
        errors++;
        wait_idr = true;
        count = 0;
        if (playing) reset_requested = true;
        xSemaphoreGive(lock);
        return;
    }
    encoded++;
    encode_us_total += elapsed;
    scale_us_total += scale_us;
    if (scale_us > max_scale_us) max_scale_us = scale_us;
    if (elapsed + scale_us > max_pipeline_us) max_pipeline_us = elapsed + scale_us;
    if (elapsed > max_encode_us) max_encode_us = elapsed;
    if (len > max_unit) max_unit = len;
    if (!h264_params_ready(&h264_params))
        h264_params_ingest(&h264_params, data, len);
    if (!playing || reset_requested || !len) {
        xSemaphoreGive(lock);
        return;
    }
    if (count == SLOTS) {
        queue_full_resets++;
        uint32_t age_us = (uint32_t)(esp_timer_get_time() - units[head].queued_us);
        if (age_us > queue_age_max_us) queue_age_max_us = age_us;
        /* TCP backlog cannot be discarded in-place: supervisor closes session. */
        count = 0;
        wait_idr = true;
        reset_requested = true;
        resyncs++;
    }
    if (!reset_requested && (!wait_idr || idr)) {
        unit_t *u = &units[(head + count) % SLOTS];
        int64_t copy_start = esp_timer_get_time();
        memcpy(u->data, data, len);
        uint32_t copy_us = (uint32_t)(esp_timer_get_time() - copy_start);
        if (copy_us > enqueue_copy_max_us) enqueue_copy_max_us = copy_us;
        u->len = len;
        u->pts = pts_ms;
        u->queued_us = esp_timer_get_time();
        count++;
        if (count > queue_high_water) queue_high_water = count;
        wait_idr = false;
    }
    xSemaphoreGive(lock);
}
void stream_retire_connection(void)
{
    xSemaphoreTake(lock, portMAX_DELAY);
    playing = false;
    count = 0;
    wait_idr = true;
    reset_requested = true;
    xSemaphoreGive(lock);
}

static void supervisor(void *unused)
{
    (void)unused;
    char bound_ip[16] = "";
    uint32_t previous = 0;
    int64_t previous_time = esp_timer_get_time();
    unsigned tick = 0;
    while (true) {
        bool reset, can_start;
        xSemaphoreTake(lock, portMAX_DELAY);
        reset = reset_requested;
        reset_requested = false;
        can_start = ready && ip[0] && h264_params_ready(&h264_params);
        if (reset) { playing = false; count = 0; wait_idr = true; }
        snprintf(bound_ip, sizeof(bound_ip), "%s", ip);
        xSemaphoreGive(lock);
        if (reset && server) {
            ESP_LOGW(TAG, "closing RTSP sessions for link/backpressure recovery");
            xSemaphoreTake(lock, portMAX_DELAY);
            listening = false;
            xSemaphoreGive(lock);
            rtsp_socket_begin_stop();
            /* Socket EOF must reach the control worker before checking timer
             * quiescence; it can otherwise still be creating a PLAY timer. */
            ESP_ERROR_CHECK(rtsp_socket_wait_retired(3500) ? ESP_OK : ESP_ERR_TIMEOUT);
            ESP_ERROR_CHECK(rtsp_timer_wait_idle(3500) ? ESP_OK : ESP_ERR_TIMEOUT);
            ESP_LOGI(TAG, "RTSP video callbacks drained before server stop");
            ESP_ERROR_CHECK(esp_rtsp_server_stop(server));
            server = NULL;
            rtsp_socket_release_clients();
        }
        if (!server && can_start) {
            video = (esp_rtsp_video_info_t){
                .vcodec = RTSP_VCODEC_H264, .width = width, .height = height,
                .fps = fps, .len = MAX_AU,
            };
            callbacks = (esp_rtsp_data_cb_t){.send_video = send_video};
            config = (esp_rtsp_config_t){
                .video_enable = true, .audio_enable = false,
                .mode = RTSP_SERVER, .local_port = 8554, .local_addr = bound_ip,
                .stack_size = 12 * 1024, .task_prio = 5, .trans = RTSP_TRANSPORT_TCP,
                .video_info = &video, .data_cb = &callbacks, .state = state_changed,
            };
            server = esp_rtsp_server_start(&config);
            xSemaphoreTake(lock, portMAX_DELAY);
            listening = server != NULL;
            xSemaphoreGive(lock);
            ESP_LOGI(TAG, "RTSP %s rtsp://%s:8554/live", server ? "listening" : "failed", bound_ip);
        }
        xSemaphoreTake(lock, portMAX_DELAY);
        if (!ready && !server) xSemaphoreGive(pause_ack);
        xSemaphoreGive(lock);
        if (++tick % 5 == 0) {
            int64_t now = esp_timer_get_time();
            xSemaphoreTake(lock, portMAX_DELAY);
            ESP_LOGI(TAG, "STATS frames=%" PRIu32 " fps_x100=%" PRIu32
                     " errors=%" PRIu32 " sent=%" PRIu32 " queue=%u resets=%" PRIu32
                     " encode_mean_us=%" PRIu32 " max_us=%" PRIu32 " max_au=%" PRIu32
                     " internal_free=%u psram_free=%u",
                     encoded, (uint32_t)((encoded >= previous ? encoded - previous : encoded) * 100000000ULL / (now - previous_time)),
                     errors, delivered, count, resyncs,
                     encoded ? (uint32_t)(encode_us_total / encoded) : 0, max_encode_us, max_unit,
                     (unsigned)heap_caps_get_free_size(MALLOC_CAP_INTERNAL),
                     (unsigned)heap_caps_get_free_size(MALLOC_CAP_SPIRAM));
            previous = encoded;
            previous_time = now;
            xSemaphoreGive(lock);
        }
        vTaskDelay(pdMS_TO_TICKS(1000));
    }
}
void stream_start(int w, int h, int rate)
{
    xSemaphoreTake(lock, portMAX_DELAY);
    width = w;
    height = h;
    fps = rate;
    encoded = errors = delivered = resyncs = max_encode_us = max_unit = 0;
    encode_us_total = scale_us_total = 0;
    max_scale_us = max_pipeline_us = 0;
    queue_full_resets = queue_age_resets = callback_capacity_resets = 0;
    queue_high_water = queue_age_max_us = callback_gap_max_us = 0;
    callback_copy_max_us = enqueue_copy_max_us = callback_wait_max_us = 0;
    callback_copy_total_us = callback_wait_total_us = 0;
    last_callback_us = 0;
    rtsp_socket_stats_reset();
    rtsp_timer_stats_reset();
    rtsp_alloc_stats_reset();
    exposure_lines = gain_index = -1;
    sample_us = 0;
    memset(&h264_params, 0, sizeof(h264_params));
    ready = true;
    bool create = !supervisor_started;
    supervisor_started = true;
    xSemaphoreGive(lock);
    if (create) configASSERT(xTaskCreate(supervisor, "stream_super", 6144, NULL, 4, NULL) == pdPASS);
}

void stream_image_sample(int exposure, int gain, unsigned luma)
{
    xSemaphoreTake(lock, portMAX_DELAY);
    exposure_lines = exposure;
    gain_index = gain;
    sampled_luma = luma;
    sample_us = esp_timer_get_time();
    xSemaphoreGive(lock);
}

void stream_stats_json(char *buffer, unsigned size)
{
    rtsp_socket_stats_t socket = rtsp_socket_stats_get();
    rtsp_timer_stats_t timer = rtsp_timer_stats_get();
    rtsp_alloc_stats_t alloc = rtsp_alloc_stats_get();
    xSemaphoreTake(lock, portMAX_DELAY);
    snprintf(buffer, size,
             "{\"width\":%d,\"height\":%d,\"target_fps\":%d,\"h264_sdp_ready\":%s,"
             "\"encoded\":%" PRIu32 ",\"errors\":%" PRIu32 ",\"sent\":%" PRIu32 ","
             "\"queue_units\":%u,\"resyncs\":%" PRIu32 ",\"playing\":%s,\"rtsp_listening\":%s,"
             "\"encode_mean_us\":%" PRIu32 ",\"encode_max_us\":%" PRIu32 ",\"max_au_bytes\":%" PRIu32 ","
             "\"scale_mean_us\":%" PRIu32 ",\"scale_max_us\":%" PRIu32 ",\"pipeline_max_us\":%" PRIu32 ","
             "\"internal_free\":%u,\"internal_min\":%u,\"psram_free\":%u,"
             "\"internal_largest\":%u,\"uptime_ms\":%" PRIu64 ","
             "\"image_sample_valid\":%s,\"exposure_lines\":%d,\"exposure_us\":%d,"
             "\"gain_index\":%d,\"sampled_luma\":%u,\"image_sample_age_ms\":%" PRIi64 ","
             "\"delivery\":{\"queue_age_limit_ms\":%u,\"queue_full_resets\":%" PRIu32 ",\"queue_age_resets\":%" PRIu32 ","
             "\"callback_capacity_resets\":%" PRIu32 ",\"queue_high_water\":%" PRIu32 ","
             "\"queue_age_max_us\":%" PRIu32 ",\"callback_gap_max_us\":%" PRIu32 ","
             "\"callback_wait_max_us\":%" PRIu32 ",\"callback_copy_max_us\":%" PRIu32 ","
             "\"enqueue_copy_max_us\":%" PRIu32 ",\"send_calls\":%" PRIu64 ","
             "\"send_bytes\":%" PRIu64 ",\"send_total_us\":%" PRIu64 ",\"send_max_us\":%" PRIu32 ","
             "\"send_max_bytes\":%" PRIu32 ",\"send_short_writes\":%" PRIu32 ","
             "\"send_failures\":%" PRIu32 ",\"send_last_errno\":%d,"
             "\"timer_calls\":%" PRIu64 ",\"timer_total_us\":%" PRIu64 ",\"timer_max_us\":%" PRIu32 "},"
             "\"packetization\":{\"callback_copy_total_us\":%" PRIu64 ",\"callback_wait_total_us\":%" PRIu64 ","
             "\"alloc_calls\":%" PRIu64 ",\"alloc_bytes\":%" PRIu64 ",\"alloc_us\":%" PRIu64 ","
             "\"free_calls\":%" PRIu64 ",\"free_us\":%" PRIu64 ",\"alloc_max_us\":%" PRIu32 ","
             "\"free_max_us\":%" PRIu32 ",\"alloc_failures\":%" PRIu32 ","
             "\"small_allocs\":%" PRIu32 ",\"packet_allocs\":%" PRIu32 ",\"large_allocs\":%" PRIu32 "}}",
             width, height, fps, ready && h264_params_ready(&h264_params) ? "true" : "false",
             encoded, errors, delivered, count, resyncs,
             playing ? "true" : "false", listening && !reset_requested && ready ? "true" : "false",
             encoded ? (uint32_t)(encode_us_total / encoded) : 0, max_encode_us, max_unit,
             encoded ? (uint32_t)(scale_us_total / encoded) : 0, max_scale_us, max_pipeline_us,
             (unsigned)heap_caps_get_free_size(MALLOC_CAP_INTERNAL),
             (unsigned)heap_caps_get_minimum_free_size(MALLOC_CAP_INTERNAL),
             (unsigned)heap_caps_get_free_size(MALLOC_CAP_SPIRAM),
             (unsigned)heap_caps_get_largest_free_block(MALLOC_CAP_INTERNAL),
             (uint64_t)(esp_timer_get_time() / 1000),
             sample_us && ready && exposure_lines >= 0 && gain_index >= 0 ? "true" : "false",
             exposure_lines, exposure_lines < 0 ? -1 : exposure_lines * 18904 / 1000,
             gain_index, sampled_luma,
             sample_us ? (esp_timer_get_time() - sample_us) / 1000 : (int64_t)-1,
             stream_queue_age_limit_ms(fps), queue_full_resets, queue_age_resets, callback_capacity_resets,
             queue_high_water, queue_age_max_us, callback_gap_max_us,
             callback_wait_max_us, callback_copy_max_us, enqueue_copy_max_us,
             socket.calls, socket.bytes, socket.total_us, socket.max_us, socket.max_bytes,
             socket.short_writes, socket.failures, socket.last_errno,
             timer.calls, timer.total_us, timer.max_us,
             callback_copy_total_us, callback_wait_total_us,
             alloc.alloc_calls, alloc.alloc_bytes, alloc.alloc_us, alloc.free_calls, alloc.free_us,
             alloc.alloc_max_us, alloc.free_max_us, alloc.failures,
             alloc.small_calls, alloc.packet_calls, alloc.large_calls);
    xSemaphoreGive(lock);
}

bool stream_pause(void)
{
    if (!supervisor_started) return true;
    while (xSemaphoreTake(pause_ack, 0) == pdTRUE) {}
    xSemaphoreTake(lock, portMAX_DELAY);
    ready = false;
    playing = false;
    count = 0;
    wait_idr = true;
    reset_requested = true;
    xSemaphoreGive(lock);
    return xSemaphoreTake(pause_ack, pdMS_TO_TICKS(6000)) == pdTRUE;
}
