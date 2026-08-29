/*
 * SPDX-FileCopyrightText: 2026 esp_cam_sensor_imx contributors
 *
 * SPDX-License-Identifier: Apache-2.0
 *
 * Dongwoon DW9807 voice-coil-motor (VCM) autofocus actuator.
 *
 * This is the lens driver on the Raspberry Pi Camera Module 3 (IMX708). It sits
 * on the same I2C bus as the sensor at address 0x0c, and it is an entirely
 * separate device from the IMX708 - the sensor driver does not and cannot move
 * the lens.
 */
#pragma once

#include "esp_cam_motor.h"

#ifdef __cplusplus
extern "C" {
#endif

/* 7-bit SCCB address. Fixed in silicon; the Camera Module 3 does not strap it. */
#define DW9807_SCCB_ADDR        0x0c

/*
 * 10-bit DAC, so the electrical range is 0..1023. Only part of that range moves
 * the lens through real focus distances - see CONFIG_CAM_MOTOR_DW9807_*_POS
 * and the note in dw9807.c about calibrating it.
 */
#define DW9807_MAX_DAC_CODE     1023

/**
 * @brief Detect and initialise a DW9807 on the given SCCB bus.
 *
 * Normally you do not call this: with CONFIG_CAM_MOTOR_DW9807_AUTO_DETECT set,
 * esp_video probes for it during esp_video_init() when the init config carries
 * a `cam_motor` entry and ESP_VIDEO_INIT_FLAGS_MOTOR.
 *
 * @param config Motor configuration; only `sccb_handle` is required, the pin
 *               fields may all be -1 (they are on the Pi 15-pin CSI connector).
 *
 * @return Motor device handle on success, NULL if no DW9807 answered.
 */
esp_cam_motor_device_t *dw9807_detect(esp_cam_motor_config_t *config);

#ifdef __cplusplus
}
#endif
