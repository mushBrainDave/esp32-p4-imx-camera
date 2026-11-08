#pragma once
#include <pthread.h>
#include <stdlib.h>
typedef pthread_mutex_t *SemaphoreHandle_t;
static inline SemaphoreHandle_t xSemaphoreCreateMutex(void) {
    SemaphoreHandle_t p = malloc(sizeof(*p));
    if (p) pthread_mutex_init(p, NULL);
    return p;
}
static inline int xSemaphoreTake(SemaphoreHandle_t p, unsigned timeout) {
    (void)timeout; return pthread_mutex_lock(p) == 0;
}
static inline int xSemaphoreGive(SemaphoreHandle_t p) { return pthread_mutex_unlock(p) == 0; }
