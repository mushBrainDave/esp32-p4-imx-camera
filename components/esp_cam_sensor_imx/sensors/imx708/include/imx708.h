/*
 * SPDX-FileCopyrightText: 2026 esp_cam_sensor_imx contributors
 *
 * SPDX-License-Identifier: Apache-2.0
 */
#pragma once

#include <stddef.h>
#include <stdint.h>

#include "esp_cam_sensor.h"
#include "esp_cam_sensor_types.h"

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

/**
 * @brief Number of modes the driver offers.
 *
 * The start-up mode is chosen at build time with
 * CAMERA_IMX708_MIPI_IF_FORMAT_INDEX_DEFAULT. These lookups exist for
 * applications that want to change it at run time, by passing the result to
 * esp_video's VIDIOC_S_SENSOR_FMT - which has no way of its own to enumerate
 * the modes a sensor supports.
 *
 * Do this before REQBUFS and STREAMON: setting a sensor format resizes the
 * stream buffers, and the pixel format then has to be renegotiated with
 * VIDIOC_S_FMT.
 *
 * @return Mode count, currently 3.
 */
size_t imx708_format_count(void);

/**
 * @brief Look a mode up by table index.
 *
 * Indices match CAMERA_IMX708_MIPI_IF_FORMAT_INDEX_DEFAULT: 0 is 1920x1080,
 * 1 is 1280x720, 2 is 640x480, all RAW10 at 28 fps.
 *
 * @param index Mode index, 0 .. imx708_format_count() - 1.
 * @return Pointer into the driver's static mode table, valid for the lifetime
 *         of the program, or NULL if the index is out of range.
 */
const esp_cam_sensor_format_t *imx708_format_by_index(size_t index);

/**
 * @brief Look a mode up by output size.
 *
 * @param width  Output width in pixels.
 * @param height Output height in pixels.
 * @return Pointer to the first mode with that size, valid for the lifetime of
 *         the program, or NULL if no mode matches. Should modes ever share a
 *         size, imx708_format_by_index() is the unambiguous selector.
 */
const esp_cam_sensor_format_t *imx708_format_by_size(uint16_t width, uint16_t height);

#ifdef __cplusplus
}
#endif
