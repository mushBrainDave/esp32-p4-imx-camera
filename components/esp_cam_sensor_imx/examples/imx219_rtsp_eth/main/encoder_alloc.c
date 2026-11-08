// SPDX-License-Identifier: Apache-2.0
#include <stdbool.h>
#include "freertos/FreeRTOS.h"
#include "esp_h264_alloc.h"
#include "esp_cache.h"
#include "esp_log.h"
#include "encoder_alloc.h"

/* Pinned esp_h264 1.4.1: 3 * 16 * (16 + 8) * macroblock_width + 7.
 * Reserve the largest reference block before network/camera allocations and
 * retain it across encoder deletion. Only these exact reference requests and
 * frees of this one pointer are intercepted. Re-audit on component/width changes.
 * One encoder exists at a time, owned by the camera task.
 */
#define REFERENCE_BYTES(width) (3U * 16U * 24U * ((width) / 16U) + 7U)
static void *reference;
static uint32_t reference_size;
static bool reference_in_use;
void *__real_esp_h264_aligned_malloc(uint32_t alignment, uint32_t n, uint32_t size,
                                    uint32_t *actual_size, uint32_t caps);
void __real_heap_caps_free(void *ptr);

void encoder_memory_init(void)
{
    configASSERT(!reference);
    reference = __real_esp_h264_aligned_malloc(16, 1, REFERENCE_BYTES(1664),
                                              &reference_size, ESP_H264_MEM_INTERNAL);
    configASSERT(reference);
    ESP_LOGI("encoder_alloc", "reserved %u internal reference bytes", (unsigned)reference_size);
}
void *__wrap_esp_h264_aligned_malloc(uint32_t alignment, uint32_t n, uint32_t size,
                                    uint32_t *actual_size, uint32_t caps)
{
    if (alignment == 16 && n == 1 && caps == ESP_H264_MEM_INTERNAL &&
        (size == REFERENCE_BYTES(832) || size == REFERENCE_BYTES(1280) || size == REFERENCE_BYTES(1536) ||
         size == REFERENCE_BYTES(1632) || size == REFERENCE_BYTES(1664))) {
        if (!reference || reference_in_use) {
            ESP_LOGE("encoder_alloc", "reference unavailable");
            *actual_size = 0;
            return NULL;
        }
        reference_in_use = true;
        *actual_size = reference_size;
        esp_cache_msync(reference, reference_size,
                        ESP_CACHE_MSYNC_FLAG_DIR_C2M | ESP_CACHE_MSYNC_FLAG_UNALIGNED);
        ESP_LOGI("encoder_alloc", "reference request %u uses reserved block", (unsigned)size);
        return reference;
    }
    return __real_esp_h264_aligned_malloc(alignment, n, size, actual_size, caps);
}
void __wrap_heap_caps_free(void *ptr)
{
    if (reference && ptr == reference) {
        configASSERT(reference_in_use);
        reference_in_use = false;
        return;
    }
    __real_heap_caps_free(ptr);
}
