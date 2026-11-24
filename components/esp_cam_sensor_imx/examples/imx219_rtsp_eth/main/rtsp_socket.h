// SPDX-License-Identifier: Apache-2.0
#pragma once
#include <stdbool.h>
#include <stdint.h>
typedef struct {
    uint64_t calls, bytes, total_us;
    uint32_t max_us, max_bytes, short_writes, failures;
    int last_errno;
} rtsp_socket_stats_t;
/* Profile-lifetime counters; recovery/reader disconnect does not clear them. */
void rtsp_socket_stats_reset(void);
rtsp_socket_stats_t rtsp_socket_stats_get(void);
void rtsp_socket_init(void);
void rtsp_socket_begin_stop(void);
bool rtsp_socket_wait_retired(unsigned timeout_ms);
void rtsp_socket_release_clients(void);
