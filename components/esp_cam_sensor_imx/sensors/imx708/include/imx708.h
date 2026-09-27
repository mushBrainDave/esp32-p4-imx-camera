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
 * @return Mode count. Enumerate rather than assuming a number: modes are
 *         appended over time, and the IMX708_FMT_* enum in imx708.c is what
 *         says how many there are.
 */
size_t imx708_format_count(void);

/**
 * @brief Look a mode up by table index.
 *
 * Indices match CAMERA_IMX708_MIPI_IF_FORMAT_INDEX_DEFAULT: 0 is 1920x1080,
 * 1 is 1280x720, 2 is 1024x768, 3 is 800x600, 4 is 640x480, all RAW10 at
 * 28 fps.
 *
 * An index is part of the driver's interface, so modes are only ever appended
 * to the IMX708_FMT_* enum in imx708.c, never inserted: a number written down
 * against one release keeps meaning the same mode in the next. That enum is
 * the authority if this list ever falls behind it.
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

/**
 * @brief Frame length, in lines, as an esp_cam_sensor parameter.
 *
 * The same control as imx708_set_frame_length(), for callers holding the
 * sensor handle: esp_cam_sensor_set_para_value() / get_para_value() with a
 * uint32_t, and esp_cam_sensor_query_para_desc() for the range. esp_video maps
 * no V4L2 control onto it, so it is unreachable through VIDIOC_S_EXT_CTRLS.
 */
#define IMX708_CID_FRAME_LENGTH  ESP_CAM_SENSOR_CLASS_ID(ESP_CAM_SENSOR_CID_CLASS_USER, 0x708)

/**
 * @brief Set the frame length (VTS), which sets the frame rate.
 *
 * Every current mode has a line time of 13.361 us (7824 pixels at
 * 585.6 MHz), so fps = 74847 / lines, and lines = 74847 / fps:
 *
 *   1336 lines = 56 fps (the floor)   2672 = 28 fps (every mode's default)
 *   4990 = 15 fps    7485 = 10 fps    65535 = 1.14 fps (the ceiling)
 *
 * Out-of-range values are clamped, not rejected; read the value back with
 * imx708_get_frame_length().
 *
 * The sensor delivers every rate in that range. The P4 pipeline behind it,
 * measured with imx708_snapshot: every mode is clean at 56 fps except
 * 640x480, whose frames tear from 48 fps up (clean at 40) with nothing logged.
 *
 * It is live: safe mid-stream, effective from the next frame. It also moves
 * the exposure ceiling (frame length - 48 lines), and V4L2_CID_EXPOSURE's
 * advertised maximum follows at once. Shortening the frame clamps an exposure
 * that no longer fits.
 *
 * Two things it does not do:
 *  - Raise AE's ceiling. The ISP pipeline reads the exposure range once, at
 *    esp_video_init(), so a longer frame gives a manually set exposure more
 *    room but AE keeps to the range it started with. A shorter frame is fine
 *    - AE's requests are clamped to the live range.
 *  - Survive a mode change. VIDIOC_S_SENSOR_FMT restores the mode's own frame
 *    length, so set this after it.
 *
 * Applies to the sensor esp_video (or anyone) detected through imx708_detect().
 *
 * @param lines Frame length in lines.
 * @return ESP_OK; ESP_ERR_INVALID_STATE if no IMX708 has been detected or no
 *         mode set yet; an SCCB error if the write failed.
 */
esp_err_t imx708_set_frame_length(uint32_t lines);

/**
 * @brief Read the frame length in force and the range it may be set to.
 *
 * @param lines Current frame length, or NULL.
 * @param min   Shortest frame the current mode allows, or NULL.
 * @param max   Longest, or NULL.
 * @return ESP_OK, or ESP_ERR_INVALID_STATE if no IMX708 has been detected.
 */
esp_err_t imx708_get_frame_length(uint32_t *lines, uint32_t *min, uint32_t *max);

#ifdef __cplusplus
}
#endif
