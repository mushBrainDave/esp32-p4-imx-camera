/*
 * SPDX-FileCopyrightText: 2026 esp_cam_sensor_imx contributors
 *
 * SPDX-License-Identifier: Apache-2.0
 */
#include <stdio.h>
#include <inttypes.h>
#include "esp_log.h"
#include "esp_rom_crc.h"
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#if CONFIG_ESP_CONSOLE_USB_SERIAL_JTAG
#include "driver/usb_serial_jtag.h"
#include "driver/usb_serial_jtag_vfs.h"
#else
#include "driver/uart_vfs.h"
#endif
#include "imx_serial_img.h"

static const char *TAG = "imx_serial_img";

static void set_tx_line_endings(esp_line_endings_t mode)
{
#if CONFIG_ESP_CONSOLE_USB_SERIAL_JTAG
    usb_serial_jtag_vfs_set_tx_line_endings(mode);
#else
    uart_vfs_dev_port_set_tx_line_endings(CONFIG_ESP_CONSOLE_UART_NUM, mode);
#endif
}

#if CONFIG_ESP_CONSOLE_USB_SERIAL_JTAG
/*
 * Without the driver, the USB-Serial/JTAG console writes straight to the
 * 64-byte FIFO and gives up on whatever does not fit within a short timeout.
 * That is the right trade for logs, but it silently drops payload bytes the
 * moment the host falls behind. With the driver installed and the VFS
 * switched to it, writes block on a ring buffer instead.
 */
static esp_err_t ensure_usj_driver(void)
{
    if (usb_serial_jtag_is_driver_installed()) {
        return ESP_OK;
    }
    usb_serial_jtag_driver_config_t cfg = {
        .tx_buffer_size = 16 * 1024,
        .rx_buffer_size = 256,
    };
    esp_err_t err = usb_serial_jtag_driver_install(&cfg);
    if (err != ESP_OK) {
        return err;
    }
    usb_serial_jtag_vfs_use_driver();
    return ESP_OK;
}
#endif

/* Small enough that the console TX ring drains rather than being handed
 * megabytes at once; large enough that the per-call overhead is noise. */
#define CHUNK_BYTES     4096
/* Yield every this many chunks. See the watchdog note in the send loop. */
#define YIELD_EVERY     32

esp_err_t imx_serial_send_blob(const char *name, const char *fmt,
                               uint32_t w, uint32_t h,
                               const uint8_t *data, size_t len,
                               const char *extra)
{
    uint32_t crc = esp_rom_crc32_le(0, data, len);

#if CONFIG_ESP_CONSOLE_USB_SERIAL_JTAG
    esp_err_t derr = ensure_usj_driver();
    if (derr != ESP_OK) {
        ESP_LOGE(TAG, "USB-Serial/JTAG driver install failed: %s", esp_err_to_name(derr));
        return derr;
    }
#endif

    /*
     * Two things would corrupt the payload if left alone. The console VFS
     * rewrites LF as CRLF, which would inject a byte at every 0x0A in the
     * payload; and any task that logs mid-transfer would interleave its line
     * into the middle of it. Silence both for the duration.
     */
    set_tx_line_endings(ESP_LINE_ENDINGS_LF);
    esp_log_level_t prev = esp_log_level_get("*");
    esp_log_level_set("*", ESP_LOG_NONE);

    printf("\nIMGSTART name=%s fmt=%s w=%" PRIu32 " h=%" PRIu32 " len=%u crc32=%08" PRIx32 "%s%s\n",
           name, fmt, w, h, (unsigned)len, crc,
           extra ? " " : "", extra ? extra : "");
    fflush(stdout);

    size_t sent = 0;
    unsigned chunks = 0;
    while (sent < len) {
        size_t n = len - sent;
        if (n > CHUNK_BYTES) {
            n = CHUNK_BYTES;
        }
        size_t wrote = fwrite(data + sent, 1, n, stdout);
        if (wrote != n) {
            break;
        }
        sent += wrote;

        /*
         * Yield periodically. A 4 MB payload takes ~21 s at 2 Mbaud, and
         * without this the task never blocks, so the idle task never runs and
         * the task watchdog fires - printing a warning and a backtrace straight
         * into the middle of the binary payload. That is not hypothetical: it
         * injected exactly four 917-byte blocks into a raw transfer, and
         * because the watchdog writes directly rather than through the log tag
         * system, silencing the logs above does not stop it.
         */
        if ((++chunks % YIELD_EVERY) == 0) {
            vTaskDelay(1);
        }
    }
    fflush(stdout);
    printf("\nIMGEND\n");
    fflush(stdout);

    esp_log_level_set("*", prev);
    set_tx_line_endings(ESP_LINE_ENDINGS_CRLF);

    if (sent != len) {
        ESP_LOGE(TAG, "%s: sent %u of %u bytes", name, (unsigned)sent, (unsigned)len);
        return ESP_FAIL;
    }
    ESP_LOGI(TAG, "sent %s (%s, %u bytes, crc32=%08" PRIx32 ")", name, fmt, (unsigned)len, crc);
    return ESP_OK;
}
