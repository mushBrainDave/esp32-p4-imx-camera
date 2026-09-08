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
#define IMX708_ORIENTATION_HMIRROR  0x01
#define IMX708_ORIENTATION_VFLIP    0x02
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

/*
 * Output size and digital crop.
 *
 * The IMX708 pipeline runs analog crop -> binning -> digital crop -> output
 * size. There is a scaler between the last two in the CCS register map, and it
 * does not work - see below - so output size is always written equal to the
 * digital crop.
 *
 * 0x0400..0x0407 are the CCS / SMIA++ standard scaling block, the same
 * addresses IMX219's datasheet documents. Raspberry Pi's imx708.c never writes
 * them in any of its four modes, and neither does its imx219.c, so there was
 * nothing to cross-reference them against the way the rest of this file is.
 *
 * MEASURED 2026-09-05, and the answer is no: the whole block is READ-ONLY on
 * this part. Every byte of 0x0400..0x0407 was written with its own value
 * XOR 0x02 and read back unchanged, while 0x0408..0x040F - the digital crop
 * immediately after it - took every write. The registers do exist and report
 * the CCS defaults (scaling_mode 0, scale_m 16, scale_n 16, i.e. "1:1, not
 * scaling"), but they are hardwired to them.
 *
 * So the IMX708 cannot downscale. Output size smaller than the digital crop
 * does not resample the crop, it crops again from the crop's origin - a
 * 2304x1296 crop with a 1152x648 output size emits the top-left 1152x648,
 * confirmed by cross-correlating such a frame against a 1920x1080 one of the
 * same scene. Resolution below the crop size has to come from binning, from
 * moving the analog readout window, or from the P4's own ISP downstream.
 *
 * The scaling registers are deliberately not defined below. Writing this
 * down rather than leaving it to be rediscovered is the whole point, but a
 * block that cannot be written does not need names, and having them would only
 * invite the next person to try. The addresses are above if a later revision
 * ever needs re-testing: write a byte, read it back, and see whether it moved.
 */
#define IMX708_REG_X_OUTPUT_SIZE_H  0x034c
#define IMX708_REG_X_OUTPUT_SIZE_L  0x034d
#define IMX708_REG_Y_OUTPUT_SIZE_H  0x034e
#define IMX708_REG_Y_OUTPUT_SIZE_L  0x034f

#define IMX708_REG_DIG_CROP_X_H     0x0408
#define IMX708_REG_DIG_CROP_X_L     0x0409
#define IMX708_REG_DIG_CROP_Y_H     0x040a
#define IMX708_REG_DIG_CROP_Y_L     0x040b
#define IMX708_REG_DIG_CROP_W_H     0x040c
#define IMX708_REG_DIG_CROP_W_L     0x040d
#define IMX708_REG_DIG_CROP_H_H     0x040e
#define IMX708_REG_DIG_CROP_H_L     0x040f

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
