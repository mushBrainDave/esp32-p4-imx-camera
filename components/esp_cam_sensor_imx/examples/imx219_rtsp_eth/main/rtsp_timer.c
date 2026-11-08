// SPDX-License-Identifier: Apache-2.0
#include <string.h>
#include "esp_timer.h"
#include "freertos/FreeRTOS.h"
#include "freertos/semphr.h"
#include "freertos/task.h"
#include "rtsp_timer.h"
#include "rtsp_alloc.h"

/* The pinned protocol library can clear its run flag before the periodic
 * cleanup callback executes. Its stop API then skips waiting for that callback.
 * Track only its named video timer, including the tail after timer_delete, so
 * the supervisor cannot free the server while the callback still uses it.
 */
typedef struct {
    bool used, deleting;
    unsigned running;
    esp_timer_handle_t handle;
    esp_timer_cb_t callback;
    void *arg;
} tracked_t;
static tracked_t timers[4];
static SemaphoreHandle_t timer_lock;
static rtsp_timer_stats_t stats;
void rtsp_timer_stats_reset(void)
{
    xSemaphoreTake(timer_lock, portMAX_DELAY);
    memset(&stats, 0, sizeof(stats));
    xSemaphoreGive(timer_lock);
}
rtsp_timer_stats_t rtsp_timer_stats_get(void)
{
    xSemaphoreTake(timer_lock, portMAX_DELAY);
    rtsp_timer_stats_t snapshot = stats;
    xSemaphoreGive(timer_lock);
    return snapshot;
}
esp_err_t __real_esp_timer_create(const esp_timer_create_args_t *, esp_timer_handle_t *);
esp_err_t __real_esp_timer_delete(esp_timer_handle_t);

void rtsp_timer_init(void)
{
    timer_lock = xSemaphoreCreateMutex();
    configASSERT(timer_lock);
}
static void dispatch(void *arg)
{
    tracked_t *t = arg;
    xSemaphoreTake(timer_lock, portMAX_DELAY);
    ++t->running;
    esp_timer_cb_t callback = t->callback;
    void *context = t->arg;
    xSemaphoreGive(timer_lock);
    int64_t start = esp_timer_get_time();
    rtsp_alloc_scope_begin();
    callback(context);
    rtsp_alloc_scope_end();
    uint32_t elapsed = (uint32_t)(esp_timer_get_time() - start);
    xSemaphoreTake(timer_lock, portMAX_DELAY);
    stats.calls++;
    stats.total_us += elapsed;
    if (elapsed > stats.max_us) stats.max_us = elapsed;
    if (--t->running == 0 && t->deleting) memset(t, 0, sizeof(*t));
    xSemaphoreGive(timer_lock);
}
esp_err_t __wrap_esp_timer_create(const esp_timer_create_args_t *args, esp_timer_handle_t *handle)
{
    if (!timer_lock || !args || !args->name || strcmp(args->name, "rtp_video_timer") ||
        args->dispatch_method != ESP_TIMER_TASK)
        return __real_esp_timer_create(args, handle);
    xSemaphoreTake(timer_lock, portMAX_DELAY);
    tracked_t *t = NULL;
    for (unsigned i = 0; i < 4; ++i) if (!timers[i].used) { t = &timers[i]; break; }
    if (!t) { xSemaphoreGive(timer_lock); return ESP_ERR_NO_MEM; }
    *t = (tracked_t){.used = true, .callback = args->callback, .arg = args->arg};
    esp_timer_create_args_t wrapped = *args;
    wrapped.callback = dispatch;
    wrapped.arg = t;
    esp_err_t err = __real_esp_timer_create(&wrapped, handle);
    if (err == ESP_OK) t->handle = *handle;
    else memset(t, 0, sizeof(*t));
    xSemaphoreGive(timer_lock);
    return err;
}
esp_err_t __wrap_esp_timer_delete(esp_timer_handle_t handle)
{
    if (!timer_lock) return __real_esp_timer_delete(handle);
    xSemaphoreTake(timer_lock, portMAX_DELAY);
    esp_err_t err = __real_esp_timer_delete(handle);
    if (err == ESP_OK) for (unsigned i = 0; i < 4; ++i) {
        tracked_t *t = &timers[i];
        if (t->used && t->handle == handle) {
            t->deleting = true;
            if (!t->running) memset(t, 0, sizeof(*t));
            break;
        }
    }
    xSemaphoreGive(timer_lock);
    return err;
}
bool rtsp_timer_wait_idle(unsigned timeout_ms)
{
    int64_t deadline = esp_timer_get_time() + (int64_t)timeout_ms * 1000;
    do {
        bool busy = false;
        xSemaphoreTake(timer_lock, portMAX_DELAY);
        for (unsigned i = 0; i < 4; ++i) busy |= timers[i].used;
        xSemaphoreGive(timer_lock);
        if (!busy) return true;
        vTaskDelay(pdMS_TO_TICKS(10));
    } while (esp_timer_get_time() < deadline);
    return false;
}
