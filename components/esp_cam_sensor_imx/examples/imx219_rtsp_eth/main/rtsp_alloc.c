// SPDX-License-Identifier: Apache-2.0
#include <stddef.h>
#include <stdint.h>
#include <stdbool.h>
#include <string.h>
#include <errno.h>
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "esp_timer.h"
#include "rtsp_alloc.h"

void *__real_media_lib_malloc(size_t size);
void *__real_media_lib_calloc(size_t count, size_t size);
void __real_media_lib_free(void *pointer);
static portMUX_TYPE mux = portMUX_INITIALIZER_UNLOCKED;
static TaskHandle_t video_task;
static rtsp_alloc_stats_t stats;

void rtsp_alloc_scope_begin(void)
{
    taskENTER_CRITICAL(&mux);
    video_task = xTaskGetCurrentTaskHandle();
    taskEXIT_CRITICAL(&mux);
}
void rtsp_alloc_scope_end(void)
{
    taskENTER_CRITICAL(&mux);
    video_task = NULL;
    taskEXIT_CRITICAL(&mux);
}
static bool in_scope(void)
{
    TaskHandle_t current = xTaskGetCurrentTaskHandle();
    taskENTER_CRITICAL(&mux);
    bool tracked = current == video_task;
    taskEXIT_CRITICAL(&mux);
    return tracked;
}
void rtsp_alloc_stats_reset(void)
{
    taskENTER_CRITICAL(&mux);
    memset(&stats, 0, sizeof(stats));
    taskEXIT_CRITICAL(&mux);
}
rtsp_alloc_stats_t rtsp_alloc_stats_get(void)
{
    taskENTER_CRITICAL(&mux);
    rtsp_alloc_stats_t snapshot = stats;
    taskEXIT_CRITICAL(&mux);
    return snapshot;
}
static void record_alloc(size_t bytes, void *pointer, uint32_t elapsed)
{
    taskENTER_CRITICAL(&mux);
    stats.alloc_calls++;
    stats.alloc_bytes += bytes;
    stats.alloc_us += elapsed;
    if (elapsed > stats.alloc_max_us) stats.alloc_max_us = elapsed;
    if (!pointer) stats.failures++;
    if (bytes <= 128) stats.small_calls++;
    else if (bytes <= 1536) stats.packet_calls++;
    else stats.large_calls++;
    taskEXIT_CRITICAL(&mux);
}
void *__wrap_media_lib_malloc(size_t size)
{
    if (!in_scope()) return __real_media_lib_malloc(size);
    int64_t start = esp_timer_get_time();
    void *pointer = __real_media_lib_malloc(size);
    int saved_errno = errno;
    record_alloc(size, pointer, (uint32_t)(esp_timer_get_time() - start));
    errno = saved_errno;
    return pointer;
}
void *__wrap_media_lib_calloc(size_t count, size_t size)
{
    if (!in_scope()) return __real_media_lib_calloc(count, size);
    int64_t start = esp_timer_get_time();
    void *pointer = __real_media_lib_calloc(count, size);
    int saved_errno = errno;
    /* Preserve allocator overflow behavior; saturate diagnostic byte count. */
    size_t bytes = size && count > SIZE_MAX / size ? SIZE_MAX : count * size;
    record_alloc(bytes, pointer, (uint32_t)(esp_timer_get_time() - start));
    errno = saved_errno;
    return pointer;
}
void __wrap_media_lib_free(void *pointer)
{
    if (!in_scope()) { __real_media_lib_free(pointer); return; }
    int64_t start = esp_timer_get_time();
    __real_media_lib_free(pointer);
    int saved_errno = errno;
    uint32_t elapsed = (uint32_t)(esp_timer_get_time() - start);
    taskENTER_CRITICAL(&mux);
    stats.free_calls++;
    stats.free_us += elapsed;
    if (elapsed > stats.free_max_us) stats.free_max_us = elapsed;
    taskEXIT_CRITICAL(&mux);
    errno = saved_errno;
}
