/*
 * SPDX-FileCopyrightText: 2026 esp_cam_sensor_imx contributors
 *
 * SPDX-License-Identifier: Apache-2.0
 *
 * IMX219 register initialisation sequences.
 *
 * The register/value pairs below are the standard Sony IMX219 bring-up values
 * as published in the sensor documentation and used by the Raspberry Pi
 * firmware. Values are expressed as plain data (address, byte); 16-bit
 * registers are written big-endian (high byte first) as two entries.
 */
#pragma once

#include <stdint.h>
#include "imx219_regs.h"

#ifdef __cplusplus
extern "C" {
#endif

typedef struct {
    uint16_t reg;   /*!< register address, or IMX219_REG_END / IMX219_REG_DELAY */
    uint8_t  val;   /*!< value to write, or delay in ms for IMX219_REG_DELAY   */
} imx219_reginfo_t;

/* XCLK is 24 MHz on the Raspberry Pi camera modules. EXCK_FREQ = 24 * 256. */
#define IMX219_XCLK_FREQ_HZ   24000000

/*
 * Common registers, applied before any mode. Access-enable sequence for the
 * 0x3000-0x5fff range, undocumented tuning registers, line length (HTS=3448),
 * pixel increments and D-PHY/clock setup.
 */
static const imx219_reginfo_t imx219_common_regs[] = {
    {IMX219_REG_MODE_SELECT, 0x00},          /* ensure standby */

    /* Access code to reach registers 0x3000-0x5fff */
    {0x30eb, 0x05}, {0x30eb, 0x0c},
    {0x300a, 0xff}, {0x300b, 0xff},
    {0x30eb, 0x05}, {0x30eb, 0x09},

    /* Undocumented Sony tuning registers */
    {0x455e, 0x00}, {0x471e, 0x4b}, {0x4767, 0x0f}, {0x4750, 0x14},
    {0x4540, 0x00}, {0x47b4, 0x14}, {0x4713, 0x30}, {0x478b, 0x10},
    {0x478f, 0x10}, {0x4793, 0x10}, {0x4797, 0x0e}, {0x479b, 0x0e},

    /* Line length = 3448 (0x0d78) */
    {IMX219_REG_LINE_LENGTH_H, 0x0d}, {IMX219_REG_LINE_LENGTH_L, 0x78},
    {IMX219_REG_X_ODD_INC, 0x01},
    {IMX219_REG_Y_ODD_INC, 0x01},

    /* D-PHY auto timing; EXCK_FREQ = 24MHz * 256 = 0x1800 */
    {IMX219_REG_DPHY_CTRL, 0x00},
    {IMX219_REG_EXCK_FREQ_H, 0x18}, {IMX219_REG_EXCK_FREQ_L, 0x00},

    {IMX219_REG_END, 0x00},
};

/*
 * 2-lane PLL configuration. Fixed link frequency of 456 MHz (912 Mbps/lane)
 * across all modes; pixel rate 182.4 MHz.
 */
static const imx219_reginfo_t imx219_2lane_regs[] = {
    {IMX219_REG_VTPXCK_DIV, 5},
    {IMX219_REG_VTSYCK_DIV, 1},
    {IMX219_REG_PREPLLCK_VT_DIV, 3},
    {IMX219_REG_PREPLLCK_OP_DIV, 3},
    {IMX219_REG_PLL_VT_MPY_H, 0x00}, {IMX219_REG_PLL_VT_MPY_L, 57},  /* 0x0039 */
    {IMX219_REG_OPSYCK_DIV, 1},
    {IMX219_REG_PLL_OP_MPY_H, 0x00}, {IMX219_REG_PLL_OP_MPY_L, 114}, /* 0x0072 */
    {IMX219_REG_CSI_LANE_MODE, IMX219_CSI_2_LANE_MODE},
    {IMX219_REG_END, 0x00},
};

/* RAW10 output format */
static const imx219_reginfo_t imx219_raw10_regs[] = {
    {IMX219_REG_CSI_DATA_FORMAT_H, 0x0a}, {IMX219_REG_CSI_DATA_FORMAT_L, 0x0a},
    {IMX219_REG_OPPXCK_DIV, 10},
    {IMX219_REG_END, 0x00},
};

/* RAW8 output format */
static const imx219_reginfo_t imx219_raw8_regs[] = {
    {IMX219_REG_CSI_DATA_FORMAT_H, 0x08}, {IMX219_REG_CSI_DATA_FORMAT_L, 0x08},
    {IMX219_REG_OPPXCK_DIV, 8},
    {IMX219_REG_END, 0x00},
};

/*
 * Mode: 1640x1232, 2x2 binned, RAW10, ~30fps. Full field of view, half
 * resolution — the recommended bring-up mode (lowest bandwidth full-FOV mode).
 * Crop is the full array (0..3279, 0..2463); VTS = 0x06e3.
 */
static const imx219_reginfo_t imx219_mode_1640x1232_regs[] = {
    {IMX219_REG_X_ADD_STA_H, 0x00}, {IMX219_REG_X_ADD_STA_L, 0x00},   /* 0    */
    {IMX219_REG_X_ADD_END_H, 0x0c}, {IMX219_REG_X_ADD_END_L, 0xcf},   /* 3279 */
    {IMX219_REG_Y_ADD_STA_H, 0x00}, {IMX219_REG_Y_ADD_STA_L, 0x00},   /* 0    */
    {IMX219_REG_Y_ADD_END_H, 0x09}, {IMX219_REG_Y_ADD_END_L, 0x9f},   /* 2463 */
    {IMX219_REG_X_OUTPUT_SIZE_H, 0x06}, {IMX219_REG_X_OUTPUT_SIZE_L, 0x68}, /* 1640 */
    {IMX219_REG_Y_OUTPUT_SIZE_H, 0x04}, {IMX219_REG_Y_OUTPUT_SIZE_L, 0xd0}, /* 1232 */
    {IMX219_REG_BINNING_MODE_H, 0x01}, {IMX219_REG_BINNING_MODE_L, 0x01},   /* 2x2 normal */
    {IMX219_REG_VTS_H, 0x06}, {IMX219_REG_VTS_L, 0xe3},               /* 1763 */
    {IMX219_REG_END, 0x00},
};

/*
 * Mode: 3280x2464, full resolution, RAW10, ~15fps. VTS = 0x0dc6.
 * High bandwidth — bring this up only after the binned mode streams cleanly.
 */
static const imx219_reginfo_t imx219_mode_3280x2464_regs[] = {
    {IMX219_REG_X_ADD_STA_H, 0x00}, {IMX219_REG_X_ADD_STA_L, 0x00},   /* 0    */
    {IMX219_REG_X_ADD_END_H, 0x0c}, {IMX219_REG_X_ADD_END_L, 0xcf},   /* 3279 */
    {IMX219_REG_Y_ADD_STA_H, 0x00}, {IMX219_REG_Y_ADD_STA_L, 0x00},   /* 0    */
    {IMX219_REG_Y_ADD_END_H, 0x09}, {IMX219_REG_Y_ADD_END_L, 0x9f},   /* 2463 */
    {IMX219_REG_X_OUTPUT_SIZE_H, 0x0c}, {IMX219_REG_X_OUTPUT_SIZE_L, 0xd0}, /* 3280 */
    {IMX219_REG_Y_OUTPUT_SIZE_H, 0x09}, {IMX219_REG_Y_OUTPUT_SIZE_L, 0xa0}, /* 2464 */
    {IMX219_REG_BINNING_MODE_H, 0x00}, {IMX219_REG_BINNING_MODE_L, 0x00},   /* none */
    {IMX219_REG_VTS_H, 0x0d}, {IMX219_REG_VTS_L, 0xc6},               /* 3526 */
    {IMX219_REG_END, 0x00},
};

#ifdef __cplusplus
}
#endif
