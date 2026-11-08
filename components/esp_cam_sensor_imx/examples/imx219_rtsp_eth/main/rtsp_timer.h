// SPDX-License-Identifier: Apache-2.0
#pragma once
#include <stdbool.h>
#include <stdint.h>
typedef struct {
    uint64_t calls, total_us;
    uint32_t max_us;
} rtsp_timer_stats_t;
void rtsp_timer_stats_reset(void);
rtsp_timer_stats_t rtsp_timer_stats_get(void);
void rtsp_timer_init(void);
bool rtsp_timer_wait_idle(unsigned timeout_ms);
