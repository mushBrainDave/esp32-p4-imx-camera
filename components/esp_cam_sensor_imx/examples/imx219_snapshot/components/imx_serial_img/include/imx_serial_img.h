/*
 * SPDX-FileCopyrightText: 2026 esp_cam_sensor_imx contributors
 *
 * SPDX-License-Identifier: Apache-2.0
 *
 * Shipping a binary payload out of the board down the console UART.
 *
 * The examples here have no other way off the board that does not involve
 * carrying a microSD card to a PC, so stills, focus-sweep frames and recorded
 * video all leave the same way. Getting binary through a console intact is
 * fiddly enough - three separate corruption modes, all silent, are documented
 * in imx_serial_send_blob() - that it lives in one place rather than being
 * copied into each example.
 */
#pragma once

#include <stddef.h>
#include <stdint.h>
#include "esp_err.h"

#ifdef __cplusplus
extern "C" {
#endif

/**
 * @brief Send one binary payload down the console UART, framed and checksummed.
 *
 * On the wire:
 *
 *     IMGSTART name=<n> fmt=<f> w=<w> h=<h> len=<N> crc32=<hex>[ <extra>]
 *     <exactly N raw bytes>
 *     IMGEND
 *
 * A text header and trailer so the payload can be found in a stream that is
 * otherwise log lines, and a length plus a CRC so the receiver knows it got all
 * of it rather than guessing from delimiters that could occur in the data.
 *
 * @param name   Base name the receiver saves it under. No spaces.
 * @param fmt    Payload format: "jpeg", "rgb565", "h264".
 * @param w      Frame width in pixels, for the receiver's benefit.
 * @param h      Frame height in pixels.
 * @param data   Payload. Must stay valid and unmodified for the whole call -
 *               at 2 Mbaud a megabyte takes about five seconds.
 * @param len    Payload length in bytes.
 * @param extra  Extra "key=value key=value" fields appended to the header, or
 *               NULL. Receivers ignore keys they do not know, so this is how a
 *               format carries what it needs (video adds fps= and frames=)
 *               without every reader having to be taught about it first.
 *
 * @return ESP_OK, or ESP_FAIL if the console would not take the whole payload.
 */
esp_err_t imx_serial_send_blob(const char *name, const char *fmt,
                               uint32_t w, uint32_t h,
                               const uint8_t *data, size_t len,
                               const char *extra);

#ifdef __cplusplus
}
#endif
