// SPDX-License-Identifier: Apache-2.0
#include <string.h>
#include <errno.h>
#include <unistd.h>
#include "lwip/sockets.h"
#include "lwip/inet.h"
#include "rtsp_wire.h"
#include "rtsp_socket.h"
#include "stream_bridge.h"
#include "freertos/FreeRTOS.h"
#include "freertos/semphr.h"
#include "freertos/task.h"
#include "esp_timer.h"

ssize_t __real_media_lib_socket_send(int s, const void *data, size_t len, int flags);
ssize_t __real_media_lib_socket_write(int s, const void *data, size_t len);
int __real_media_lib_socket_close(int s);
int __real_media_lib_socket_accept(int s, struct sockaddr *addr, socklen_t *len);
int __real_media_lib_socket_listen(int s, int backlog);

/* Retain accepted descriptors after shutdown until the RTSP server has joined
 * its workers. Closing them earlier lets a stale video writer use a descriptor
 * that lwIP has reassigned to HTTP or a new RTSP session. Bounded to four clients;
 * the supervisor restarts/releases on retirement, including normal teardown.
 */
#define CLIENTS 4
static struct { int fd; bool retired; } clients[CLIENTS];
/* The pinned server and its transport both close the listener during stop.
 * Reserve it through their shared stop barrier so the second close cannot
 * target an HTTP socket that reused the descriptor. */
static int listener_fd = -1;
static SemaphoreHandle_t client_lock;
static bool stopping;
static rtsp_socket_stats_t stats;
void rtsp_socket_stats_reset(void)
{
    xSemaphoreTake(client_lock, portMAX_DELAY);
    memset(&stats, 0, sizeof(stats));
    xSemaphoreGive(client_lock);
}
rtsp_socket_stats_t rtsp_socket_stats_get(void)
{
    xSemaphoreTake(client_lock, portMAX_DELAY);
    rtsp_socket_stats_t snapshot = stats;
    xSemaphoreGive(client_lock);
    return snapshot;
}
void rtsp_socket_init(void)
{
    client_lock = xSemaphoreCreateMutex();
    configASSERT(client_lock);
    for (unsigned i = 0; i < CLIENTS; ++i) clients[i].fd = -1;
}
void rtsp_socket_begin_stop(void)
{
    int fds[CLIENTS];
    xSemaphoreTake(client_lock, portMAX_DELAY);
    stopping = true;
    for (unsigned i = 0; i < CLIENTS; ++i) fds[i] = clients[i].fd;
    xSemaphoreGive(client_lock);
    for (unsigned i = 0; i < CLIENTS; ++i)
        if (fds[i] >= 0) shutdown(fds[i], SHUT_RDWR);
}
bool rtsp_socket_wait_retired(unsigned timeout_ms)
{
    int64_t deadline = esp_timer_get_time() + (int64_t)timeout_ms * 1000;
    do {
        bool busy = false;
        xSemaphoreTake(client_lock, portMAX_DELAY);
        for (unsigned i = 0; i < CLIENTS; ++i)
            busy |= clients[i].fd >= 0 && !clients[i].retired;
        xSemaphoreGive(client_lock);
        if (!busy) return true;
        vTaskDelay(pdMS_TO_TICKS(10));
    } while (esp_timer_get_time() < deadline);
    return false;
}
void rtsp_socket_release_clients(void)
{
    /* Called only after esp_rtsp_server_stop has returned and all workers exited. */
    int fds[CLIENTS], listener;
    xSemaphoreTake(client_lock, portMAX_DELAY);
    listener = listener_fd;
    listener_fd = -1;
    for (unsigned i = 0; i < CLIENTS; ++i) {
        fds[i] = clients[i].fd;
        clients[i].fd = -1;
        clients[i].retired = false;
    }
    xSemaphoreGive(client_lock);
    for (unsigned i = 0; i < CLIENTS; ++i)
        if (fds[i] >= 0) { shutdown(fds[i], SHUT_RDWR); __real_media_lib_socket_close(fds[i]); }
    if (listener >= 0) __real_media_lib_socket_close(listener);
    xSemaphoreTake(client_lock, portMAX_DELAY);
    stopping = false;
    xSemaphoreGive(client_lock);
}
int __wrap_media_lib_socket_close(int fd)
{
    bool owned = false, notify = false;
    xSemaphoreTake(client_lock, portMAX_DELAY);
    owned = fd >= 0 && fd == listener_fd;
    for (unsigned i = 0; i < CLIENTS; ++i) {
        if (clients[i].fd == fd) {
            owned = true;
            notify = !clients[i].retired && !stopping;
            clients[i].retired = true;
            break;
        }
    }
    xSemaphoreGive(client_lock);
    if (!owned) return __real_media_lib_socket_close(fd);
    shutdown(fd, SHUT_RDWR);
    if (notify) stream_retire_connection();
    return 0;
}
int __wrap_media_lib_socket_listen(int fd, int backlog)
{
    int result = __real_media_lib_socket_listen(fd, backlog);
    if (result < 0) return result;
    xSemaphoreTake(client_lock, portMAX_DELAY);
    bool kept = !stopping && (listener_fd < 0 || listener_fd == fd);
    if (kept) listener_fd = fd;
    xSemaphoreGive(client_lock);
    if (!kept) { errno = EBUSY; return -1; }
    return result;
}
int __wrap_media_lib_socket_accept(int s, struct sockaddr *addr, socklen_t *len)
{
    int client = __real_media_lib_socket_accept(s, addr, len);
    if (client >= 0) {
        struct timeval timeout = {.tv_sec = 0, .tv_usec = 200000};
        if (setsockopt(client, SOL_SOCKET, SO_SNDTIMEO, &timeout, sizeof(timeout))) {
            __real_media_lib_socket_close(client);
            return -1;
        }
        bool kept = false;
        struct sockaddr_storage peer;
        socklen_t peer_len = sizeof(peer);
        bool wakeup = getpeername(client, (struct sockaddr *)&peer, &peer_len) == 0 &&
                      peer.ss_family == AF_INET &&
                      ((struct sockaddr_in *)&peer)->sin_addr.s_addr == htonl(INADDR_LOOPBACK);
        xSemaphoreTake(client_lock, portMAX_DELAY);
        bool busy = false;
        for (unsigned i = 0; i < CLIENTS; ++i) if (clients[i].fd >= 0) busy = true;
        /* The pinned library shares per-session worker state in one context.
         * A second session must not enter until the first has fully stopped.
         * Only its loopback stop-wakeup is allowed while stopping.
         */
        bool allowed = stopping ? wakeup : !busy;
        for (unsigned i = 0; allowed && i < CLIENTS; ++i) {
            if (clients[i].fd < 0) {
                clients[i].fd = client;
                clients[i].retired = false;
                kept = true;
                break;
            }
        }
        xSemaphoreGive(client_lock);
        if (!kept) { __real_media_lib_socket_close(client); return -1; }
    }
    return client;
}

/* Keep the larger SDP scratch buffers off the small video-timer stack. This
 * helper is reached only for RTSP text on the control worker's 12KiB stack. */
static __attribute__((noinline)) ssize_t send_control_response(
    int s, const void *data, size_t len, int flags, bool write_call, size_t *wire_bytes)
{
    *wire_bytes = 0;
    unsigned char copy[2048];
    if (len <= 1024 && len >= 8 && !memcmp(data, "RTSP/1.0", 8)) {
        h264_params_t params;
        size_t wire_len = len;
        bool changed = stream_h264_params(&params) &&
                       h264_sdp_response(data, len, &params, copy, sizeof(copy), &wire_len);
        if (!changed) {
            memcpy(copy, data, len);
            changed = rtsp_normalize_content_length(copy, len);
        }
        if (changed) {
            size_t offset = 0;
            while (offset < wire_len) {
                ssize_t n = write_call ? __real_media_lib_socket_write(s, copy + offset, wire_len - offset)
                                      : __real_media_lib_socket_send(s, copy + offset, wire_len - offset, flags);
                if (n <= 0) {
                    shutdown(s, SHUT_RDWR);
                    return -1;
                }
                offset += n;
                *wire_bytes += n;
            }
            return len;
        }
    }
    ssize_t result = write_call ? __real_media_lib_socket_write(s, data, len)
                                : __real_media_lib_socket_send(s, data, len, flags);
    if (result > 0) *wire_bytes = (size_t)result;
    return result;
}
static ssize_t control_send(int s, const void *data, size_t len, int flags, bool write_call,
                            size_t *wire_bytes)
{
    if (len >= 8 && len <= 1024 && !memcmp(data, "RTSP/1.0", 8))
        return send_control_response(s, data, len, flags, write_call, wire_bytes);
    *wire_bytes = 0;
    ssize_t result = write_call ? __real_media_lib_socket_write(s, data, len)
                                : __real_media_lib_socket_send(s, data, len, flags);
    if (result > 0) *wire_bytes = (size_t)result;
    return result;
}
static ssize_t measured_send(int s, const void *data, size_t len, int flags, bool write_call)
{
    int64_t start = esp_timer_get_time();
    size_t wire_bytes;
    ssize_t result = control_send(s, data, len, flags, write_call, &wire_bytes);
    int saved_errno = errno;
    uint32_t elapsed = (uint32_t)(esp_timer_get_time() - start);
    xSemaphoreTake(client_lock, portMAX_DELAY);
    stats.calls++;
    stats.total_us += elapsed;
    if (elapsed > stats.max_us) stats.max_us = elapsed;
    if (len > stats.max_bytes) stats.max_bytes = len;
    stats.bytes += wire_bytes;
    if (result >= 0 && (size_t)result < len) stats.short_writes++;
    if (result < 0) { stats.failures++; stats.last_errno = saved_errno; }
    xSemaphoreGive(client_lock);
    errno = saved_errno;
    return result;
}
ssize_t __wrap_media_lib_socket_send(int s, const void *data, size_t len, int flags)
{
    return measured_send(s, data, len, flags, false);
}
ssize_t __wrap_media_lib_socket_write(int s, const void *data, size_t len)
{
    return measured_send(s, data, len, 0, true);
}
