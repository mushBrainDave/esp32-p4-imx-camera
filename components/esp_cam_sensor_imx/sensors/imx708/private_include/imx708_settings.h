/*
 * SPDX-FileCopyrightText: 2026 esp_cam_sensor_imx contributors
 *
 * SPDX-License-Identifier: Apache-2.0
 *
 * IMX708 register initialisation sequences (standard Sony bring-up values).
 * The sensor's own tables are already byte-granular {addr, val8} pairs, so they
 * translate 1:1 — no 16-bit splitting needed. Applied in order:
 *   mode_common -> mode -> link_450Mhz, matching the sensor's bring-up order.
 */
#pragma once

#include <stdint.h>
#include "imx708_regs.h"

#ifdef __cplusplus
extern "C" {
#endif

typedef struct {
    uint16_t reg;   /*!< register address, or IMX708_REG_END / IMX708_REG_DELAY */
    uint8_t  val;   /*!< value, or delay in ms for IMX708_REG_DELAY */
} imx708_reginfo_t;

/* Common registers, applied before the link-frequency and mode registers. */
static const imx708_reginfo_t imx708_common_regs[] = {
    {0x0100, 0x00}, {0x0136, 0x18}, {0x0137, 0x00}, {0x33F0, 0x02},
    {0x33F1, 0x05}, {0x3062, 0x00}, {0x3063, 0x12}, {0x3068, 0x00},
    {0x3069, 0x12}, {0x306A, 0x00}, {0x306B, 0x30}, {0x3076, 0x00},
    {0x3077, 0x30}, {0x3078, 0x00}, {0x3079, 0x30}, {0x5E54, 0x0C},
    {0x6E44, 0x00}, {0xB0B6, 0x01}, {0xE829, 0x00}, {0xF001, 0x08},
    {0xF003, 0x08}, {0xF00D, 0x10}, {0xF00F, 0x10}, {0xF031, 0x08},
    {0xF033, 0x08}, {0xF03D, 0x10}, {0xF03F, 0x10}, {0x0112, 0x0A},
    {0x0113, 0x0A}, {0x0114, 0x01}, {0x0B8E, 0x01}, {0x0B8F, 0x00},
    {0x0B94, 0x01}, {0x0B95, 0x00}, {0x3400, 0x01}, {0x3478, 0x01},
    {0x3479, 0x1c}, {0x3091, 0x01}, {0x3092, 0x00}, {0x3419, 0x00},
    {0xBCF1, 0x02}, {0x3094, 0x01}, {0x3095, 0x01}, {0x3362, 0x00},
    {0x3363, 0x00}, {0x3364, 0x00}, {0x3365, 0x00}, {0x0138, 0x01},
    {IMX708_REG_END, 0x00},
};

/* Link frequency: 450 MHz (nominal default) -> 900 Mbps/lane on 2 lanes. */
static const imx708_reginfo_t imx708_link_450mhz_regs[] = {
    {0x030E, 0x01}, {0x030F, 0x2c},
    {IMX708_REG_END, 0x00},
};

/*
 * Modes. All three are the same 2x2-binned RAW10 readout of the full sensor,
 * differing only in the digital crop taken out of it. The analog readout
 * window, PLL, binning, Bayer phase and line/frame timing are identical, so
 * every mode reads out at 28 fps and has the same exposure ceiling: a smaller
 * mode buys CSI bandwidth, PSRAM and encode time and pays for it in field of
 * view. It cannot make the sensor faster, though it does recover frames the
 * pipeline drops at full size - imx708_video measures 27.3 fps at 1920x1080
 * against 28.0 at every smaller mode. Nothing here scales - 640x480 is a 640x480 window on the
 * middle of the picture, not the whole picture shrunk.
 *
 * Why 1920 and not the sensor's native 2304: at 2304 the captured line breaks
 * up. Scene data runs out around x=1918 and the columns at each edge duplicate
 * each other exactly 2048 pixels apart - a horizontal capacity limit in the P4
 * CSI/ISP path, not a sensor fault. It survived every change to frame rate,
 * cache size and line sync, and disappeared the moment the line got shorter.
 * 1920 is the width Espressif ship all their own P4 camera examples at. Costs
 * 17% of horizontal FOV; the analog readout still covers the full sensor.
 *
 * line_length is Sony's 0x1e90 (7824). frame_length is deliberately doubled to
 * 0x0a70 (2672), giving 28 fps instead of 56.
 *
 * That is an exposure decision, not a bandwidth one. Maximum exposure is
 * frame_length - 48 lines, so Sony's 1336 caps integration at 1288 lines
 * (17.2 ms) and forces the AE to buy the rest with analog gain - which is
 * noise. 2672 lines allows 2624 (35 ms), a full stop more light for free. For
 * a stills app frame rate is worth nothing and photons are worth everything.
 * At 1920x1080x28fps this is 116 MB/s into PSRAM, comfortably within budget.
 *
 * The tables are spliced from two shared halves with a per-mode crop block
 * between them, in the exact order the single 1920x1080 table was written in
 * when it was verified on hardware. A timing fix then lands in one place
 * instead of three, and cannot be applied to two modes out of three by
 * accident.
 */
#define IMX708_MODE_REGS_BEFORE_CROP \
    {0x0342, 0x1E}, {0x0343, 0x90},                 /* line length = 7824 */ \
    {0x0340, 0x0A}, {0x0341, 0x70},                 /* frame length = 2672 -> 28 fps */ \
    {0x0344, 0x00}, {0x0345, 0x00}, \
    {0x0346, 0x00}, {0x0347, 0x00}, \
    {0x0348, 0x11}, {0x0349, 0xFF}, \
    {0x034A, 0x0A}, {0x034B, 0x1F}, \
    {0x0220, 0x62}, {0x0222, 0x01}, \
    {0x0900, 0x01}, {0x0901, 0x22}, {0x0902, 0x08}, \
    {0x3200, 0x41}, {0x3201, 0x41}, \
    {0x32D5, 0x00}, {0x32D6, 0x00}, {0x32DB, 0x01}, {0x32DF, 0x00}, \
    {0x350C, 0x00}, {0x350D, 0x00}

#define IMX708_MODE_REGS_AFTER_CROP \
    {0x0301, 0x05}, {0x0303, 0x02}, {0x0305, 0x02}, {0x0306, 0x00}, \
    {0x0307, 0x7A}, {0x030B, 0x02}, {0x030D, 0x04}, {0x0310, 0x01}, \
    {0x3CA0, 0x00}, {0x3CA1, 0x3C}, {0x3CA4, 0x00}, {0x3CA5, 0x3C}, \
    {0x3CA6, 0x00}, {0x3CA7, 0x00}, {0x3CAA, 0x00}, {0x3CAB, 0x00}, \
    {0x3CB8, 0x00}, {0x3CB9, 0x1C}, {0x3CBA, 0x00}, {0x3CBB, 0x08}, \
    {0x3CBC, 0x00}, {0x3CBD, 0x1E}, {0x3CBE, 0x00}, {0x3CBF, 0x0A}, \
    {0x0202, 0x07}, {0x0203, 0xD0},                 /* exposure = 2000 lines (26.7 ms) */ \
    {0x0224, 0x01}, {0x0225, 0xF4}, \
    {0x3116, 0x01}, {0x3117, 0xF4}, \
    {0x0204, 0x00}, {0x0205, 0x70}, \
    {0x0216, 0x00}, {0x0217, 0x70}, \
    {0x0218, 0x01}, {0x0219, 0x00}, \
    {0x020E, 0x01}, {0x020F, 0x00}, \
    {0x3118, 0x00}, {0x3119, 0x70}, {0x311A, 0x01}, {0x311B, 0x00}, \
    {0x341a, 0x00}, {0x341b, 0x00}, {0x341c, 0x00}, {0x341d, 0x00}, \
    {0x341e, 0x00}, {0x341f, 0x90}, {0x3420, 0x00}, {0x3421, 0x6c}, \
    {0x3366, 0x00}, {0x3367, 0x00}, {0x3368, 0x00}, {0x3369, 0x00}

/* A 16-bit sensor register, high byte first, the way the tables write them. */
#define IMX708_REG16(addr, val) \
    {(addr),     (uint8_t)(((val) >> 8) & 0xFF)}, \
    {(addr) + 1, (uint8_t)(( val)       & 0xFF)}

/*
 * The per-mode block. Output size is written equal to the crop size, so a
 * smaller mode is a narrower window at native binned sampling rather than a
 * resampled full frame.
 *
 * It is written equal because it has to be: the sensor's scaler is read-only
 * (see imx708_regs.h), so an output size below the crop size is not a
 * reduction of the crop, it is a second crop from the crop's origin. Raspberry
 * Pi's own driver writes no scaling registers either in any of its four modes
 * (references/linux/imx708.c) - it reduces resolution by binning and by moving
 * the analog readout window. Its 1536x864 "720p" mode is a 3072x1728 analog
 * window binned 2x2: a crop as well, just a coarser-sampled one.
 */
#define IMX708_MODE_CROP_REGS(x, y, w, h) \
    IMX708_REG16(IMX708_REG_DIG_CROP_X_H, (x)),     /* digital crop x offset */ \
    IMX708_REG16(IMX708_REG_DIG_CROP_Y_H, (y)),     /* digital crop y offset */ \
    IMX708_REG16(IMX708_REG_DIG_CROP_W_H, (w)),     /* digital crop width    */ \
    IMX708_REG16(IMX708_REG_DIG_CROP_H_H, (h)),     /* digital crop height   */ \
    IMX708_REG16(IMX708_REG_X_OUTPUT_SIZE_H, (w)),  /* x output size         */ \
    IMX708_REG16(IMX708_REG_Y_OUTPUT_SIZE_H, (h))   /* y output size         */

/*
 * Crops are centred on the 2304x1296 binned field: offset = (2304 - w) / 2 and
 * (1296 - h) / 2. Every offset below is a multiple of 4, which preserves the
 * RGGB phase of the binned output.
 */

/* 1920x1080 - 83% of the field each way. The general-purpose mode. */
static const imx708_reginfo_t imx708_mode_1920x1080_regs[] = {
    IMX708_MODE_REGS_BEFORE_CROP,
    IMX708_MODE_CROP_REGS(192, 108, 1920, 1080),
    IMX708_MODE_REGS_AFTER_CROP,
    {IMX708_REG_END, 0x00},
};

/*
 * 1280x720 - 56% of the field each way, 44% of the pixels of 1920x1080. Both
 * axes are whole 16-pixel H.264 macroblocks (80x45), so unlike 1080 this needs
 * no height trim before the encoder.
 */
static const imx708_reginfo_t imx708_mode_1280x720_regs[] = {
    IMX708_MODE_REGS_BEFORE_CROP,
    IMX708_MODE_CROP_REGS(512, 288, 1280, 720),
    IMX708_MODE_REGS_AFTER_CROP,
    {IMX708_REG_END, 0x00},
};

/*
 * 1024x768 - 44% of the field horizontally and 59% vertically, 38% of the
 * pixels of 1920x1080. The only 4:3 mode whose axes are both whole 16-pixel
 * H.264 macroblocks (64x48), so alone among the modes here - 1080 included -
 * it needs no trim of any kind before the encoder.
 *
 * Being 4:3 it sees less across than 1280x720 and more up and down (59% of the
 * field against 56%), on fewer pixels. That is the reason to pick it over
 * mode 1 rather than a consolation: a tall subject frames better in it.
 */
static const imx708_reginfo_t imx708_mode_1024x768_regs[] = {
    IMX708_MODE_REGS_BEFORE_CROP,
    IMX708_MODE_CROP_REGS(640, 264, 1024, 768),
    IMX708_MODE_REGS_AFTER_CROP,
    {IMX708_REG_END, 0x00},
};

/*
 * 800x600 - 35% of the field horizontally and 46% vertically, 23% of the
 * pixels of 1920x1080. SVGA, and the middle 4:3 step between 1024x768 and
 * 640x480.
 *
 * 800 is 50 whole H.264 macroblocks, which is the half that matters: width
 * cannot be trimmed, because the encoder takes its line stride from the width
 * it is given, so encoding an N-wide buffer as anything narrower shears the
 * picture diagonally. 600 is 37.5 rows and so does need the height trim
 * 1920x1080 already uses - the encoder is told 592 and reads a prefix of the
 * buffer. Of the modes here only 1080 and this one need it.
 */
static const imx708_reginfo_t imx708_mode_800x600_regs[] = {
    IMX708_MODE_REGS_BEFORE_CROP,
    IMX708_MODE_CROP_REGS(752, 348, 800, 600),
    IMX708_MODE_REGS_AFTER_CROP,
    {IMX708_REG_END, 0x00},
};

/*
 * 640x480 - 28% of the field horizontally and 37% vertically, 15% of the
 * pixels of 1920x1080. Note this is a tight and differently framed window:
 * 4:3 cut out of a 16:9 field, so it is not "1920x1080 shrunk to VGA" and the
 * two modes do not see the same scene. Also whole macroblocks (40x30).
 */
static const imx708_reginfo_t imx708_mode_640x480_regs[] = {
    IMX708_MODE_REGS_BEFORE_CROP,
    IMX708_MODE_CROP_REGS(832, 408, 640, 480),
    IMX708_MODE_REGS_AFTER_CROP,
    {IMX708_REG_END, 0x00},
};

#ifdef __cplusplus
}
#endif
