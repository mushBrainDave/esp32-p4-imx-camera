#pragma once
#include <assert.h>
#define configASSERT assert
#define portMAX_DELAY 0xffffffffU
#include <pthread.h>
typedef pthread_mutex_t portMUX_TYPE;
#define portMUX_INITIALIZER_UNLOCKED PTHREAD_MUTEX_INITIALIZER
#define taskENTER_CRITICAL(mux) pthread_mutex_lock(mux)
#define taskEXIT_CRITICAL(mux) pthread_mutex_unlock(mux)
