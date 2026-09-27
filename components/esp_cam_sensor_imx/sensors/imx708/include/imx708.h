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
 *   4990 = 15 fps    7485 = 10 fps    65535 = 1.14 fps
 *   149694 = 2 s     748470 = 10 s    8388480 = 112 s (the ceiling)
 *
 * Out-of-range values are clamped, not rejected; read the value back with
 * imx708_get_frame_length().
 *
 * Past 65535 lines the driver switches on the sensor's long-exposure shift,
 * which counts frame length and exposure in units of 2^n lines (n up to 7,
 * the smallest that fits). That has three visible effects:
 *  - The frame length is rounded down to a multiple of 2^n.
 *  - V4L2_CID_EXPOSURE's minimum and step scale by 2^n too: at 2 s
 *    (n = 2) exposure runs from 16 lines in steps of 8. esp_video rejects an
 *    off-step value, so re-query the range after changing the frame length.
 *  - Crossing a 2^n boundary rewrites three interdependent registers with no
 *    group hold, so the frame at the switch can have mixed timing. Discard it.
 * VIDIOC_G_PARM's frame rate bottoms out at 1 fps, since it counts whole
 * frames per second; below that the frame length is the real figure.
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
 *    length and clears the shift, so set this after it.
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

/**
 * @brief Values of V4L2_CID_TEST_PATTERN.
 *
 * Raspberry Pi's menu order, so a number means the same pattern on both
 * drivers - and 1 is still colour bars, as it was when bars were the only one.
 * esp_video passes the control through to the sensor unvalidated; anything
 * past IMX708_TEST_PATTERN_PN9 fails with ESP_ERR_INVALID_ARG.
 *
 * Every pattern is generated after the pixel array, so it travels the whole
 * MIPI -> CSI -> ISP path. It replaces the image, not the timing: frame
 * length, exposure and gain are untouched, and it survives a mode change.
 * The generator clips to its own window (0x0620-0x0627, left at power-on
 * defaults), so a pattern can stop short of the frame edges on a healthy link.
 *
 * PN9 is the one for link integrity: a deterministic pseudo-random sequence,
 * so corrupted, dropped or duplicated data breaks it where colour bars, being
 * flat, would hide the damage. It is only checkable on raw Bayer frames that
 * repeat - compare one frame with the next, or with a known-good capture.
 * After the ISP's demosaic and tone curve it is just noise.
 */
typedef enum {
    IMX708_TEST_PATTERN_OFF        = 0,
    IMX708_TEST_PATTERN_COLOR_BARS = 1,
    IMX708_TEST_PATTERN_SOLID      = 2, /*!< the four levels below, one per Bayer channel */
    IMX708_TEST_PATTERN_GREY_BARS  = 3, /*!< colour bars fading to grey down the full array height */
    IMX708_TEST_PATTERN_PN9        = 4,
} imx708_test_pattern_t;

/**
 * @brief The solid-colour pattern's per-channel levels, as esp_cam_sensor
 *        parameters.
 *
 * Linux's V4L2_CID_TEST_PATTERN_RED / GREENR / BLUE / GREENB: a uint32_t each,
 * 0 .. 0xfff (12 bits), default 0xfff - so the solid pattern is white until
 * told otherwise. For callers holding the sensor handle; esp_video maps no V4L2
 * control onto them, so an esp_video application uses
 * imx708_set_test_pattern_colour() instead.
 */
#define IMX708_CID_TEST_PATTERN_RED     ESP_CAM_SENSOR_CLASS_ID(ESP_CAM_SENSOR_CID_CLASS_USER, 0x709)
#define IMX708_CID_TEST_PATTERN_GREENR  ESP_CAM_SENSOR_CLASS_ID(ESP_CAM_SENSOR_CID_CLASS_USER, 0x70a)
#define IMX708_CID_TEST_PATTERN_BLUE    ESP_CAM_SENSOR_CLASS_ID(ESP_CAM_SENSOR_CID_CLASS_USER, 0x70b)
#define IMX708_CID_TEST_PATTERN_GREENB  ESP_CAM_SENSOR_CLASS_ID(ESP_CAM_SENSOR_CID_CLASS_USER, 0x70c)

/**
 * @brief Set the levels IMX708_TEST_PATTERN_SOLID draws.
 *
 * Takes effect at once if the solid pattern is showing, and is kept for when it
 * next is. Pick the pattern itself with V4L2_CID_TEST_PATTERN.
 *
 * @param r, gr, b, gb 12-bit levels for the red, green-on-red-row, blue and
 *                     green-on-blue-row pixels.
 * @return ESP_OK; ESP_ERR_INVALID_ARG if any level exceeds 0xfff (nothing is
 *         written); ESP_ERR_INVALID_STATE if no IMX708 has been detected; an
 *         SCCB error if the write failed.
 */
esp_err_t imx708_set_test_pattern_colour(uint16_t r, uint16_t gr, uint16_t b, uint16_t gb);

/**
 * @brief The MIPI link frequencies the sensor can run at.
 *
 * Raspberry Pi's three, in Raspberry Pi's order. They differ by under 1%, and
 * exist to move the link's harmonics off a radio channel - relevant on a board
 * whose ESP32-C6 transmits centimetres from the camera cable. Only the link
 * moves: pixel clock, frame rate, exposure and every mode are the same at all
 * three. The boot default is Kconfig's CAMERA_IMX708_LINK_FREQ.
 */
typedef enum {
    IMX708_LINK_FREQ_450MHZ = 0, /*!< 900 Mbit/s per lane, the default */
    IMX708_LINK_FREQ_447MHZ = 1, /*!< 894 Mbit/s per lane */
    IMX708_LINK_FREQ_453MHZ = 2, /*!< 906 Mbit/s per lane */
    IMX708_LINK_FREQ_MAX,
} imx708_link_freq_t;

/**
 * @brief Link frequency as an esp_cam_sensor parameter.
 *
 * An enumeration like V4L2_CID_LINK_FREQ's int menu: esp_cam_sensor_query_para_desc()
 * lists the frequencies in Hz, and the uint32_t set or read is an
 * imx708_link_freq_t index into that list. Same rules as
 * imx708_set_link_freq(). For callers holding the sensor handle; esp_video
 * maps no V4L2 control onto it.
 */
#define IMX708_CID_LINK_FREQ  ESP_CAM_SENSOR_CLASS_ID(ESP_CAM_SENSOR_CID_CLASS_USER, 0x70d)

/**
 * @brief Change the MIPI link frequency.
 *
 * Only while the sensor is not streaming - between VIDIOC_STREAMOFF and
 * VIDIOC_STREAMON with esp_video. esp_video sets up the P4's CSI receiver for
 * the link rate at every STREAMON and holds it until STREAMOFF, so a change
 * mid-stream would leave the receiver expecting the old rate. Made in between,
 * the sensor and the receiver both pick it up at the next STREAMON.
 *
 * It survives a mode change (VIDIOC_S_SENSOR_FMT re-applies it). It does not
 * survive a reboot: the next boot starts at the Kconfig default.
 *
 * @param hz 447000000, 450000000 or 453000000.
 * @return ESP_OK; ESP_ERR_INVALID_ARG for any other frequency;
 *         ESP_ERR_INVALID_STATE if no IMX708 has been detected or it is
 *         streaming; an SCCB error if the write failed.
 */
esp_err_t imx708_set_link_freq(uint32_t hz);

/**
 * @brief Read the link frequency in force, in Hz.
 *
 * @return ESP_OK; ESP_ERR_INVALID_ARG if hz is NULL; ESP_ERR_INVALID_STATE if
 *         no IMX708 has been detected.
 */
esp_err_t imx708_get_link_freq(uint32_t *hz);

#ifdef __cplusplus
}
#endif
