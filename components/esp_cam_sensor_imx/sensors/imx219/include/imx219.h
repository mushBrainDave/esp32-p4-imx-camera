/*
 * SPDX-FileCopyrightText: 2026 esp_cam_sensor_imx contributors
 *
 * SPDX-License-Identifier: Apache-2.0
 */
#pragma once

#include "esp_cam_sensor.h"
#include "esp_cam_sensor_types.h"

#ifdef __cplusplus
extern "C" {
#endif

#define IMX219_SENSOR_NAME "IMX219"

/*
 * 7-bit SCCB/I2C address. The IMX219 is strapped to 0x10 on all Raspberry Pi
 * camera modules.
 */
#ifndef IMX219_SCCB_ADDR
#define IMX219_SCCB_ADDR   0x10
#endif

#define IMX219_PID         IMX219_CHIP_ID_VALUE
#define IMX219_CHIP_ID_VALUE 0x0219

/**
 * @brief Probe and initialise an IMX219 on the given interface.
 *
 * @param config Pointer to esp_cam_sensor_config_t.
 * @return Sensor device handle on success, NULL on failure.
 */
esp_cam_sensor_device_t *imx219_detect(esp_cam_sensor_config_t *config);

#ifdef __cplusplus
}
#endif
