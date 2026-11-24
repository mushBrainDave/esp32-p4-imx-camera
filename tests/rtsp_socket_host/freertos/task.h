#pragma once
#include <unistd.h>
#include <pthread.h>
#include <stdint.h>
typedef void *TaskHandle_t;
static inline TaskHandle_t xTaskGetCurrentTaskHandle(void) { return (TaskHandle_t)(uintptr_t)pthread_self(); }
#define pdMS_TO_TICKS(x) (x)
static inline void vTaskDelay(unsigned ms) { usleep(ms*1000); }
