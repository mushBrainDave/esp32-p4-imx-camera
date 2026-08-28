/*
 * SPDX-FileCopyrightText: 2026 esp_cam_sensor_imx contributors
 *
 * SPDX-License-Identifier: Apache-2.0
 */
#pragma once

#include "esp_cam_sensor.h"
#include "esp_cam_sensor_types.h"
#include "imx708_regs.h"

#ifdef __cplusplus
extern "C" {
#endif

#define IMX708_SENSOR_NAME "IMX708"

/* 7-bit SCCB/I2C address. IMX708 (Raspberry Pi Camera Module 3) is at 0x1a. */
#ifndef IMX708_SCCB_ADDR
#define IMX708_SCCB_ADDR   0x1a
#endif

/**
 * @brief Probe and initialise an IMX708 on the given interface.
 *
 * @param config Pointer to esp_cam_sensor_config_t.
 * @return Sensor device handle on success, NULL on failure.
 */
esp_cam_sensor_device_t *imx708_detect(esp_cam_sensor_config_t *config);

#ifdef __cplusplus
}
#endif
