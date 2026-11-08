// SPDX-License-Identifier: Apache-2.0
#pragma once
#include <stdint.h>
typedef struct {
    uint64_t alloc_calls, alloc_bytes, alloc_us, free_calls, free_us;
    uint32_t alloc_max_us, free_max_us, failures;
    uint32_t small_calls, packet_calls, large_calls;
} rtsp_alloc_stats_t;
/* Only SAL allocations on the active protocol video-timer task are measured. */
void rtsp_alloc_scope_begin(void);
void rtsp_alloc_scope_end(void);
void rtsp_alloc_stats_reset(void);
rtsp_alloc_stats_t rtsp_alloc_stats_get(void);
