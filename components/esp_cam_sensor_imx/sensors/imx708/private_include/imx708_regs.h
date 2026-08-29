/*
 * SPDX-FileCopyrightText: 2026 esp_cam_sensor_imx contributors
 *
 * SPDX-License-Identifier: Apache-2.0
 *
 * Sony IMX708 (Raspberry Pi Camera Module 3 / NoIR 3) register map.
 * Addresses/values are the public Sony bring-up values, cross-referenced
 * against the Linux kernel driver (GPL-2.0) — facts only, no code copied.
 */
#pragma once

#ifdef __cplusplus
extern "C" {
#endif

/* Sentinels for imx708_reginfo_t arrays */
#define IMX708_REG_END              0xffff
#define IMX708_REG_DELAY            0xfffe

/* Identification — NOTE: chip id is at 0x0016, unlike IMX219's 0x0000 */
#define IMX708_REG_CHIP_ID_H        0x0016
#define IMX708_REG_CHIP_ID_L        0x0017
#define IMX708_CHIP_ID              0x0708

/* Core control */
#define IMX708_REG_MODE_SELECT      0x0100  /*!< 0=standby, 1=streaming */
#define IMX708_REG_ORIENTATION      0x0101  /*!< bit0 = h flip, bit1 = v flip */
#define IMX708_REG_CSI_LANE_MODE    0x0114  /*!< 0x01 = 2 lane */

/* V timing */
#define IMX708_REG_FRAME_LENGTH_H   0x0340  /*!< VTS (frame length lines) */
#define IMX708_REG_FRAME_LENGTH_L   0x0341
#define IMX708_FRAME_LENGTH_MAX     0xffff

/* Exposure / gain (all 16-bit, big-endian) */
#define IMX708_REG_EXPOSURE_H       0x0202
#define IMX708_REG_EXPOSURE_L       0x0203
#define IMX708_EXPOSURE_OFFSET      48
#define IMX708_EXPOSURE_MIN         1
#define IMX708_EXPOSURE_DEFAULT     0x0640
#define IMX708_REG_ANALOG_GAIN_H    0x0204
#define IMX708_REG_ANALOG_GAIN_L    0x0205
#define IMX708_ANA_GAIN_MIN         112     /*!< gain = 1024/(1024-code) */
#define IMX708_ANA_GAIN_MAX         960
#define IMX708_ANA_GAIN_DEFAULT     112
#define IMX708_REG_DIGITAL_GAIN_H   0x020e
#define IMX708_REG_DIGITAL_GAIN_L   0x020f
#define IMX708_DGTL_GAIN_MIN        0x0100
#define IMX708_DGTL_GAIN_MAX        0xffff
#define IMX708_DGTL_GAIN_DEFAULT    0x0100

/* Quad-Bayer re-mosaic low-pass filter. Only the full-resolution mode
   re-mosaics, so binned modes must explicitly disable it. */
#define IMX708_REG_LPF_INTENSITY_EN 0xc428
#define IMX708_LPF_INTENSITY_ENABLED   0x00
#define IMX708_LPF_INTENSITY_DISABLED  0x01

/* Test pattern */
#define IMX708_REG_TEST_PATTERN_H   0x0600
#define IMX708_REG_TEST_PATTERN_L   0x0601
#define IMX708_TEST_PATTERN_DISABLE 0x0000
#define IMX708_TEST_PATTERN_COLORBARS 0x0002

/* External input clock */
#define IMX708_INCLK_FREQ_HZ        24000000

#ifdef __cplusplus
}
#endif
