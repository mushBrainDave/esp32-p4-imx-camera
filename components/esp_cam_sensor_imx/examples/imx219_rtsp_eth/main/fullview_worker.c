// SPDX-License-Identifier: Apache-2.0
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "freertos/semphr.h"
#include "fullview_worker.h"
static TaskHandle_t worker;
static SemaphoreHandle_t done;
static const fullview_scale_t *geometry;
static const uint8_t *input;
static uint8_t *output;
static void resize_worker(void *unused)
{
    (void)unused;
    while (true) {
        ulTaskNotifyTake(pdTRUE, portMAX_DELAY);
        fullview_scale_rows(geometry, input, output, geometry->out_pairs / 2, geometry->out_pairs);
        xSemaphoreGive(done);
    }
}
void fullview_worker_init(void)
{
    done = xSemaphoreCreateBinary();
    configASSERT(done);
    configASSERT(xTaskCreatePinnedToCore(resize_worker, "fullview_scale", 3072, NULL,
                                        3, &worker, 1) == pdPASS);
}
void fullview_resize(const fullview_scale_t *s, const uint8_t *src, uint8_t *dst)
{
    /* Camera task is the sole caller. Notification publishes a complete job;
     * semaphore completion returns buffer ownership before encode/QBUF/teardown.
     */
    geometry = s; input = src; output = dst;
    xTaskNotifyGive(worker);
    fullview_scale_rows(s, src, dst, 0, s->out_pairs / 2);
    configASSERT(xSemaphoreTake(done, portMAX_DELAY) == pdTRUE);
}
