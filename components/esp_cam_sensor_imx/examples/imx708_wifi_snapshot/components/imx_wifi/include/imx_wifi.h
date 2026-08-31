/*
 * SPDX-FileCopyrightText: 2026 esp_cam_sensor_imx contributors
 *
 * SPDX-License-Identifier: Apache-2.0
 *
 * Bringing the board onto a WiFi network through the ESP32-C6.
 *
 * The P4 has no radio of its own: esp_wifi_remote forwards the esp_wifi calls
 * over SDIO to a C6 running esp_hosted slave firmware. Everything above that -
 * esp_netif, DHCP, LwIP - runs on the P4, so once this returns the sockets are
 * ordinary sockets and nothing else in an example needs to know the radio is a
 * separate chip.
 *
 * `examples/c6_wifi_sta` deliberately does not use this. It is the bring-up
 * probe for the radio itself, and a probe that depends on the code it is
 * probing is worth less; it stays self-contained on purpose.
 */
#pragma once

#include <stdbool.h>
#include <stddef.h>
#include "esp_err.h"

#ifdef __cplusplus
extern "C" {
#endif

/**
 * @brief Bring up WiFi and block until the board has an IP, or fail.
 *
 * Does the whole sequence: NVS, esp_netif, the default event loop, the STA
 * netif, credentials, connect, and the wait for DHCP. Retries a few times on a
 * disconnect, logging the reason code each time - that code is the entire
 * diagnostic when this fails, so it is logged rather than swallowed.
 *
 * Credentials come from `wifi_credentials.h` in this component's include
 * directory, which is gitignored and written by hand from the committed
 * `wifi_credentials.h.example`. Without it this returns ESP_ERR_INVALID_STATE
 * and says what to write.
 *
 * @param[out] ip_out   Buffer for the dotted-quad address, or NULL.
 * @param[in]  ip_len   Size of ip_out. 16 bytes is enough.
 *
 * @return ESP_OK once an IP is held.
 *         ESP_ERR_INVALID_STATE if no credentials were compiled in.
 *         ESP_ERR_WIFI_NOT_INIT if esp_wifi_init failed, which means the C6 did
 *         not answer - a radio problem, not a network one.
 *         ESP_ERR_TIMEOUT if the association never completed.
 */
esp_err_t imx_wifi_connect(char *ip_out, size_t ip_len);

/**
 * @brief Whether credentials were compiled in.
 *
 * Lets a caller print its own guidance before doing any other setup, rather
 * than discovering the problem after the camera is already streaming.
 */
bool imx_wifi_have_credentials(void);

#ifdef __cplusplus
}
#endif
