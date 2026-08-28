/*
 * SPDX-FileCopyrightText: 2026 esp_cam_sensor_imx contributors
 *
 * SPDX-License-Identifier: Apache-2.0
 *
 * Camera I2C / SCCB probe for the ESP32-P4 (Waveshare ESP32-P4-WIFI6).
 *
 * This talks to the sensor over I2C ONLY — no MIPI, no ISP, no esp_video, no
 * sensor driver. It exists to validate the boring-but-critical physical layer
 * before any streaming work:
 *
 *   - cable seated / correct orientation
 *   - sensor has power
 *   - the SCCB bus reaches the connector on GPIO8/GPIO7
 *   - the sensor answers at its I2C address and returns the right chip ID
 *
 * Works with ANY Raspberry Pi camera module you happen to have plugged in:
 * it scans the whole bus, then tries a chip-ID read at each known camera
 * address and prints which sensor (if any) it found.
 *
 * The board's I2C bus is shared with the audio codec + touch controller, so
 * even if the camera is silent you should still see THOSE devices ACK — which
 * proves the bus itself works and isolates the problem to the camera.
 */
#include <stdio.h>
#include <inttypes.h>
#include "esp_log.h"
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "driver/i2c_master.h"

#define I2C_PORT     0
#define I2C_SCL_PIN  8
#define I2C_SDA_PIN  7
#define I2C_FREQ_HZ  100000

static const char *TAG = "i2c_probe";

/* Known Raspberry Pi camera sensors: {i2c addr, chip-id reg (16-bit), expected id, name}. */
typedef struct {
    uint8_t  addr;
    uint16_t id_reg;
    uint16_t expected_id;
    const char *name;
} cam_candidate_t;

static const cam_candidate_t candidates[] = {
    {0x10, 0x0000, 0x0219, "IMX219 (Pi Cam v2)"},
    {0x1a, 0x0016, 0x0708, "IMX708 (Pi Cam v3)"},
    {0x1a, 0x0000, 0x0477, "IMX477 (Pi HQ Cam)"},
};

/* Read a 16-bit-addressed register, returning 2 bytes big-endian as a uint16. */
static esp_err_t read_reg16(i2c_master_dev_handle_t dev, uint16_t reg, uint16_t *out)
{
    uint8_t wbuf[2] = { (uint8_t)(reg >> 8), (uint8_t)(reg & 0xff) };
    uint8_t rbuf[2] = { 0 };
    esp_err_t ret = i2c_master_transmit_receive(dev, wbuf, sizeof(wbuf), rbuf, sizeof(rbuf), 100);
    if (ret == ESP_OK) {
        *out = (rbuf[0] << 8) | rbuf[1];
    }
    return ret;
}

void app_main(void)
{
    ESP_LOGI(TAG, "Camera I2C probe — SCL=GPIO%d SDA=GPIO%d port %d", I2C_SCL_PIN, I2C_SDA_PIN, I2C_PORT);

    i2c_master_bus_config_t bus_cfg = {
        .clk_source = I2C_CLK_SRC_DEFAULT,
        .i2c_port = I2C_PORT,
        .scl_io_num = I2C_SCL_PIN,
        .sda_io_num = I2C_SDA_PIN,
        .glitch_ignore_cnt = 7,
        .flags.enable_internal_pullup = true,   /* Pi modules also pull up on-board */
    };
    i2c_master_bus_handle_t bus = NULL;
    ESP_ERROR_CHECK(i2c_new_master_bus(&bus_cfg, &bus));

    /* ---- 1. Scan the whole bus ---- */
    ESP_LOGI(TAG, "Scanning bus 0x08..0x77 ...");
    int found = 0;
    for (uint8_t a = 0x08; a <= 0x77; a++) {
        if (i2c_master_probe(bus, a, 50) == ESP_OK) {
            ESP_LOGI(TAG, "  device ACK at 0x%02x", a);
            found++;
        }
    }
    ESP_LOGI(TAG, "scan done: %d device(s) responded", found);
    if (found == 0) {
        ESP_LOGE(TAG, "NOTHING on the bus. Check I2C pins, and that the board's 3V3 reaches the connector.");
    }

    /* ---- 2. Try a chip-ID read at each known camera address ---- */
    bool cam_found = false;
    for (size_t i = 0; i < sizeof(candidates) / sizeof(candidates[0]); i++) {
        const cam_candidate_t *c = &candidates[i];
        i2c_device_config_t dev_cfg = {
            .dev_addr_length = I2C_ADDR_BIT_LEN_7,
            .device_address = c->addr,
            .scl_speed_hz = I2C_FREQ_HZ,
        };
        i2c_master_dev_handle_t dev = NULL;
        if (i2c_master_bus_add_device(bus, &dev_cfg, &dev) != ESP_OK) {
            continue;
        }

        uint16_t id = 0;
        esp_err_t ret = read_reg16(dev, c->id_reg, &id);
        if (ret == ESP_OK) {
            if (id == c->expected_id) {
                ESP_LOGI(TAG, "==== FOUND %s: addr 0x%02x, reg 0x%04x = 0x%04x ✓ ====",
                         c->name, c->addr, c->id_reg, id);
                cam_found = true;
            } else {
                ESP_LOGW(TAG, "addr 0x%02x reg 0x%04x = 0x%04x (expected 0x%04x for %s)",
                         c->addr, c->id_reg, id, c->expected_id, c->name);
            }
        }
        i2c_master_bus_rm_device(dev);
    }

    if (cam_found) {
        ESP_LOGI(TAG, "==== CAMERA I2C LINK OK — physical layer validated ====");
    } else {
        ESP_LOGE(TAG, "No known camera answered. If the scan above listed the codec/touch but not the "
                      "camera, the bus works and the issue is the camera cable/orientation/power.");
    }

    while (1) {
        vTaskDelay(pdMS_TO_TICKS(5000));
    }
}
