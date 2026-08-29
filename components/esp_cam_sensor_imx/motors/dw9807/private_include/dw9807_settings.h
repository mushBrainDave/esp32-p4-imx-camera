/*
 * SPDX-FileCopyrightText: 2026 esp_cam_sensor_imx contributors
 *
 * SPDX-License-Identifier: Apache-2.0
 *
 * DW9807 register map.
 *
 * Unlike the DW9714 - which takes a bare 16-bit word with the DAC code packed
 * into it - the DW9807 is a conventional 8-bit-address / 8-bit-value device.
 * The DAC is written as a single 3-byte transfer starting at the MSB register,
 * relying on the chip's address auto-increment, so that the 10-bit code lands
 * atomically. Writing MSB and LSB as two transfers would step the lens through
 * a bogus intermediate position on every move that crosses a 256-code boundary.
 */
#pragma once

#include <stdint.h>

/* Control. 0x00 = active, 0x01 = power down (lens returns to its rest stop). */
#define DW9807_REG_CTL          0x02

/* DAC code, bits [9:8]. A 3-byte write from here auto-increments into the LSB. */
#define DW9807_REG_MSB          0x03
/* DAC code, bits [7:0]. */
#define DW9807_REG_LSB          0x04

/*
 * Status. Bit 0 is set while the chip is busy applying a previous write; the
 * datasheet requires polling it clear before writing a new position, otherwise
 * the write is dropped and the lens silently stays where it was.
 */
#define DW9807_REG_STATUS       0x05
#define DW9807_STATUS_BUSY      0x01
/* Bits 7:2 are reserved and read back as 0 - used as a weak presence check. */
#define DW9807_STATUS_RESERVED  0xfc

/* Ring-compensation (SAC) mode and resonance period. Left at reset defaults. */
#define DW9807_REG_MODE         0x06
#define DW9807_REG_RESONANCE    0x07

#define DW9807_CTL_ACTIVE       0x00
#define DW9807_CTL_POWER_DOWN   0x01

/* Time for the regulator to come up after CTL goes active, milliseconds. */
#define DW9807_POWER_UP_TIME_MS 2

/* How long to wait between busy-status polls, and how many polls to give up after. */
#define DW9807_BUSY_POLL_US     1000
#define DW9807_BUSY_POLL_MAX    10
