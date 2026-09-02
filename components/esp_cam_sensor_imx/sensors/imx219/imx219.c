/*
 * SPDX-FileCopyrightText: 2026 esp_cam_sensor_imx contributors
 *
 * SPDX-License-Identifier: Apache-2.0
 *
 * Sony IMX219 (Raspberry Pi Camera Module v2) driver for the ESP32-P4
 * esp_cam_sensor framework.
 *
 * Structure follows the upstream esp_cam_sensor drivers (e.g. ov5647). The
 * IMX219-specific register values are the standard Sony bring-up values, cross
 * checked against the Linux kernel driver (GPL-2.0) but not copied from it.
 */
#include <string.h>
#include <inttypes.h>
#include <freertos/FreeRTOS.h>
#include <freertos/task.h>
#include "driver/gpio.h"
#include "esp_err.h"
#include "esp_log.h"
#include "esp_check.h"

#include "esp_cam_sensor.h"
#include "esp_cam_sensor_detect.h"
#include "imx219_settings.h"
#include "imx219.h"

#ifndef portTICK_RATE_MS
#define portTICK_RATE_MS portTICK_PERIOD_MS
#endif
#define delay_ms(ms) vTaskDelay((ms > portTICK_PERIOD_MS ? ms / portTICK_PERIOD_MS : 1))

/* Fixed sensor timing (2-lane): pixel rate 182.4 MHz, HTS 3448 pixels. */
#define IMX219_PIXEL_RATE_2LANE   182400000
#define IMX219_HTS                3448
#define IMX219_TLINE_NS           18904   /* HTS / pixel_rate, in ns */
/* 2-lane link freq 456 MHz -> 912 Mbps per lane */
#define IMX219_MIPI_CSI_LINE_RATE 912000000

#define IMX219_LINESYNC_ENABLE    0   /* set 1 to emit line-sync short packets */

static const char *TAG = "imx219";

/* ------------------------------------------------------------------ */
/* Supported output formats                                            */
/* ------------------------------------------------------------------ */
enum {
    IMX219_FMT_1640x1232_RAW10_30FPS = 0,
    IMX219_FMT_3280x2464_RAW10_15FPS,
    /* Appended, not inserted: the indices above are what
       CAMERA_IMX219_MIPI_IF_FORMAT_INDEX_DEFAULT selects between, so renumbering
       them would silently change the mode of an existing build. */
    IMX219_FMT_1632x1232_RAW10_30FPS,
};

static const esp_cam_sensor_isp_info_t imx219_isp_info[] = {
    [IMX219_FMT_1640x1232_RAW10_30FPS] = {
        .isp_v1_info = {
            .version = SENSOR_ISP_INFO_VERSION_DEFAULT,
            .pclk = IMX219_PIXEL_RATE_2LANE,
            .hts = IMX219_HTS,
            .vts = 1763,                        /* 0x06e3 */
            .exp_def = IMX219_EXPOSURE_DEFAULT,
            .gain_def = IMX219_ANA_GAIN_DEFAULT,
            .tline_ns = IMX219_TLINE_NS,
            .bayer_type = ESP_CAM_SENSOR_BAYER_RGGB,
        }
    },
    [IMX219_FMT_3280x2464_RAW10_15FPS] = {
        .isp_v1_info = {
            .version = SENSOR_ISP_INFO_VERSION_DEFAULT,
            .pclk = IMX219_PIXEL_RATE_2LANE,
            .hts = IMX219_HTS,
            .vts = 3526,                        /* 0x0dc6 */
            .exp_def = IMX219_EXPOSURE_DEFAULT,
            .gain_def = IMX219_ANA_GAIN_DEFAULT,
            .tline_ns = IMX219_TLINE_NS,
            .bayer_type = ESP_CAM_SENSOR_BAYER_RGGB,
        }
    },
    [IMX219_FMT_1632x1232_RAW10_30FPS] = {
        .isp_v1_info = {
            .version = SENSOR_ISP_INFO_VERSION_DEFAULT,
            .pclk = IMX219_PIXEL_RATE_2LANE,
            .hts = IMX219_HTS,
            /* Same VTS as the 1640 mode - only the readout width changed, so
               frame rate and every exposure limit are identical. */
            .vts = 1763,                        /* 0x06e3 */
            .exp_def = IMX219_EXPOSURE_DEFAULT,
            .gain_def = IMX219_ANA_GAIN_DEFAULT,
            .tline_ns = IMX219_TLINE_NS,
            .bayer_type = ESP_CAM_SENSOR_BAYER_RGGB,
        }
    },
};

static const esp_cam_sensor_format_t imx219_format_info[] = {
    [IMX219_FMT_1640x1232_RAW10_30FPS] = {
        .name = "MIPI_2lane_24Minput_RAW10_1640x1232_30fps",
        .format = ESP_CAM_SENSOR_PIXFORMAT_RAW10,
        .port = ESP_CAM_SENSOR_MIPI_CSI,
        .xclk = IMX219_XCLK_FREQ_HZ,
        .width = 1640,
        .height = 1232,
        .regs = imx219_mode_1640x1232_regs,
        .regs_size = ARRAY_SIZE(imx219_mode_1640x1232_regs),
        .fps = 30,
        .isp_info = &imx219_isp_info[IMX219_FMT_1640x1232_RAW10_30FPS],
        .mipi_info = {
            .mipi_clk = IMX219_MIPI_CSI_LINE_RATE,
            .lane_num = 2,
            .line_sync_en = IMX219_LINESYNC_ENABLE,
        },
        .reserved = NULL,
    },
    [IMX219_FMT_3280x2464_RAW10_15FPS] = {
        .name = "MIPI_2lane_24Minput_RAW10_3280x2464_15fps",
        .format = ESP_CAM_SENSOR_PIXFORMAT_RAW10,
        .port = ESP_CAM_SENSOR_MIPI_CSI,
        .xclk = IMX219_XCLK_FREQ_HZ,
        .width = 3280,
        .height = 2464,
        .regs = imx219_mode_3280x2464_regs,
        .regs_size = ARRAY_SIZE(imx219_mode_3280x2464_regs),
        .fps = 15,
        .isp_info = &imx219_isp_info[IMX219_FMT_3280x2464_RAW10_15FPS],
        .mipi_info = {
            .mipi_clk = IMX219_MIPI_CSI_LINE_RATE,
            .lane_num = 2,
            .line_sync_en = IMX219_LINESYNC_ENABLE,
        },
        .reserved = NULL,
    },
    [IMX219_FMT_1632x1232_RAW10_30FPS] = {
        .name = "MIPI_2lane_24Minput_RAW10_1632x1232_30fps",
        .format = ESP_CAM_SENSOR_PIXFORMAT_RAW10,
        .port = ESP_CAM_SENSOR_MIPI_CSI,
        .xclk = IMX219_XCLK_FREQ_HZ,
        .width = 1632,
        .height = 1232,
        .regs = imx219_mode_1632x1232_regs,
        .regs_size = ARRAY_SIZE(imx219_mode_1632x1232_regs),
        .fps = 30,
        .isp_info = &imx219_isp_info[IMX219_FMT_1632x1232_RAW10_30FPS],
        .mipi_info = {
            .mipi_clk = IMX219_MIPI_CSI_LINE_RATE,
            .lane_num = 2,
            .line_sync_en = IMX219_LINESYNC_ENABLE,
        },
        .reserved = NULL,
    },
};

/*
 * Which mode the sensor comes up in.
 *
 * Index 2 (1632x1232) exists because H.264 codes in 16x16 macroblocks and 1640
 * is 102.5 of them. Video builds want it; everything else is better off with
 * the full 1640-wide field of view, so the default stays index 0 and the video
 * example selects index 2 in its sdkconfig.defaults.
 */
/*
 * Fall back to mode 0 if the Kconfig symbol is absent. It will be, in any build
 * tree whose sdkconfig predates this option and has not been reconfigured -
 * and without this the failure is "CONFIG_..._INDEX_DEFAULT undeclared" from
 * inside the driver, which points at the wrong thing entirely. This way a stale
 * tree quietly keeps the behaviour it already had.
 */
#ifndef CONFIG_CAMERA_IMX219_MIPI_IF_FORMAT_INDEX_DEFAULT
#define CONFIG_CAMERA_IMX219_MIPI_IF_FORMAT_INDEX_DEFAULT 0
#endif

#define IMX219_DEFAULT_FORMAT_INDEX CONFIG_CAMERA_IMX219_MIPI_IF_FORMAT_INDEX_DEFAULT

#if IMX219_DEFAULT_FORMAT_INDEX >= 3
#error "CAMERA_IMX219_MIPI_IF_FORMAT_INDEX_DEFAULT is out of range - see the mode table above"
#endif

/* ------------------------------------------------------------------ */
/* Gain table                                                          */
/* ------------------------------------------------------------------ */
/*
 * esp_video drives AE gain as a *menu* control: it calls VIDIOC_QUERYMENU to
 * read each entry's total gain and binary-searches for the one nearest the
 * target, then sets the winning INDEX. So ESP_CAM_SENSOR_GAIN has to be an
 * enumeration of gain values in milli-units (1000 = 1.00x) - declaring it as a
 * plain number makes esp_video_cam_query_menu() reject it and the AE then
 * silently drives exposure only, which looks like a dark, grainy picture
 * rather than like an error.
 *
 * IMX219 analog gain is gain = 256 / (256 - code) for code 0..232, i.e. 1.0x
 * to 10.667x. These 42 entries step it at roughly 1/12 stop; the values are
 * the gains the codes actually achieve, not the ideal 1/12-stop targets, so
 * what the AE reads back is what the sensor is really doing.
 */
static const uint32_t imx219_total_gain_val_map[] = {
      1000,   1058,   1123,   1191,   1261,   1333,   1414,   1497,
      1590,   1684,   1778,   1882,   2000,   2116,   2246,   2370,
      2510,   2667,   2813,   3012,   3160,   3368,   3556,   3765,
      4000,   4267,   4491,   4741,   5020,   5333,   5689,   5953,
      6400,   6737,   7111,   7529,   8000,   8533,   8828,   9481,
     10240,  10667,
};

static const uint8_t imx219_ana_gain_code_map[] = {
       0,   14,   28,   41,   53,   64,   75,   85,
      95,  104,  112,  120,  128,  135,  142,  148,
     154,  160,  165,  171,  175,  180,  184,  188,
     192,  196,  199,  202,  205,  208,  211,  213,
     216,  218,  220,  222,  224,  226,  227,  229,
     231,  232,
};

/* Index whose code (104) is the nearest entry to the gain this driver has
   always written at set_format time, so the menu does not change the picture
   the no-AE path produces. */
#define IMX219_DEFAULT_GAIN_INDEX 9

/* Per-device state, so the ISP can read back what it last set. */
typedef struct {
    uint32_t exposure_val;      /*!< current exposure, in lines */
    uint32_t gain_index;        /*!< index into imx219_total_gain_val_map */
} imx219_para_t;

/* ------------------------------------------------------------------ */
/* SCCB helpers (16-bit register address, 8-bit value)                 */
/* ------------------------------------------------------------------ */
static esp_err_t imx219_read(esp_sccb_io_handle_t sccb, uint16_t reg, uint8_t *val)
{
    return esp_sccb_transmit_receive_reg_a16v8(sccb, reg, val);
}

static esp_err_t imx219_write(esp_sccb_io_handle_t sccb, uint16_t reg, uint8_t val)
{
    return esp_sccb_transmit_reg_a16v8(sccb, reg, val);
}

/* Write a 16-bit value big-endian to reg / reg+1 */
static esp_err_t imx219_write16(esp_sccb_io_handle_t sccb, uint16_t reg, uint16_t val)
{
    esp_err_t ret = imx219_write(sccb, reg, (val >> 8) & 0xff);
    if (ret == ESP_OK) {
        ret = imx219_write(sccb, reg + 1, val & 0xff);
    }
    return ret;
}

static esp_err_t imx219_write_array(esp_sccb_io_handle_t sccb, const imx219_reginfo_t *regs)
{
    esp_err_t ret = ESP_OK;
    int i = 0;
    while (ret == ESP_OK && regs[i].reg != IMX219_REG_END) {
        if (regs[i].reg == IMX219_REG_DELAY) {
            delay_ms(regs[i].val);
        } else {
            ret = imx219_write(sccb, regs[i].reg, regs[i].val);
        }
        i++;
    }
    ESP_LOGD(TAG, "wrote %d regs", i);
    return ret;
}

static esp_err_t imx219_set_reg_bits(esp_sccb_io_handle_t sccb, uint16_t reg,
                                     uint8_t offset, uint8_t length, uint8_t value)
{
    uint8_t reg_val = 0;
    esp_err_t ret = imx219_read(sccb, reg, &reg_val);
    if (ret != ESP_OK) {
        return ret;
    }
    uint8_t mask = ((1 << length) - 1) << offset;
    reg_val = (reg_val & ~mask) | ((value << offset) & mask);
    return imx219_write(sccb, reg, reg_val);
}

/* ------------------------------------------------------------------ */
/* Sensor operations                                                   */
/* ------------------------------------------------------------------ */
static esp_err_t imx219_get_sensor_id(esp_cam_sensor_device_t *dev, esp_cam_sensor_id_t *id)
{
    uint8_t h = 0, l = 0;
    esp_err_t ret = imx219_read(dev->sccb_handle, IMX219_REG_CHIP_ID_H, &h);
    ESP_RETURN_ON_FALSE(ret == ESP_OK, ret, TAG, "read chip id high failed");
    ret = imx219_read(dev->sccb_handle, IMX219_REG_CHIP_ID_L, &l);
    ESP_RETURN_ON_FALSE(ret == ESP_OK, ret, TAG, "read chip id low failed");
    id->pid = (h << 8) | l;
    return ESP_OK;
}

static esp_err_t imx219_set_stream(esp_cam_sensor_device_t *dev, int enable)
{
    esp_err_t ret = imx219_write(dev->sccb_handle, IMX219_REG_MODE_SELECT, enable ? 0x01 : 0x00);
    ESP_RETURN_ON_FALSE(ret == ESP_OK, ret, TAG, "set stream failed");
    dev->stream_status = enable;
    ESP_LOGD(TAG, "stream=%d", enable);
    return ret;
}

static esp_err_t imx219_soft_reset(esp_cam_sensor_device_t *dev)
{
    esp_err_t ret = imx219_set_reg_bits(dev->sccb_handle, IMX219_REG_SW_RESET, 0, 1, 0x01);
    delay_ms(5);
    return ret;
}

static esp_err_t imx219_hw_reset(esp_cam_sensor_device_t *dev)
{
    if (dev->reset_pin >= 0) {
        gpio_set_level(dev->reset_pin, 0);
        delay_ms(10);
        gpio_set_level(dev->reset_pin, 1);
        delay_ms(10);   /* datasheet t4+t5 ~6.2ms; be generous */
    }
    return ESP_OK;
}

/* Orientation register: bit0 = h flip (mirror), bit1 = v flip. */
static esp_err_t imx219_set_mirror(esp_cam_sensor_device_t *dev, int enable)
{
    return imx219_set_reg_bits(dev->sccb_handle, IMX219_REG_ORIENTATION, 0, 1, enable ? 1 : 0);
}

static esp_err_t imx219_set_vflip(esp_cam_sensor_device_t *dev, int enable)
{
    return imx219_set_reg_bits(dev->sccb_handle, IMX219_REG_ORIENTATION, 1, 1, enable ? 1 : 0);
}

/*
 * Largest legal coarse integration time for the mode currently selected.
 *
 * Unlike the IMX708, whose VTS is fixed, the IMX219's frame length differs per
 * mode (1763 binned, 3526 full), so the ceiling has to come from the mode
 * rather than from a constant. Exposing 65535 - the register width - would let
 * the AE drive integration time past the frame length, which the sensor cannot
 * honour: the value silently does not take effect and the AE loop then sees no
 * response to its own request.
 */
static uint32_t imx219_exposure_max(esp_cam_sensor_device_t *dev)
{
    uint32_t max = IMX219_EXPOSURE_MAX;
    if (dev->cur_format && dev->cur_format->isp_info) {
        uint32_t vts = dev->cur_format->isp_info->isp_v1_info.vts;
        if (vts > (IMX219_EXPOSURE_OFFSET + IMX219_EXPOSURE_MIN)) {
            max = vts - IMX219_EXPOSURE_OFFSET;
        }
    }
    return max > IMX219_EXPOSURE_MAX ? IMX219_EXPOSURE_MAX : max;
}

/* Exposure in lines (coarse integration time). */
static esp_err_t imx219_set_exposure(esp_cam_sensor_device_t *dev, uint32_t lines)
{
    uint32_t max = imx219_exposure_max(dev);
    if (lines < IMX219_EXPOSURE_MIN) {
        lines = IMX219_EXPOSURE_MIN;
    }
    if (lines > max) {
        lines = max;
    }
    esp_err_t ret = imx219_write16(dev->sccb_handle, IMX219_REG_EXPOSURE_H, (uint16_t)lines);
    if (ret == ESP_OK && dev->priv) {
        ((imx219_para_t *)dev->priv)->exposure_val = lines;
    }
    return ret;
}

/* Analog gain: 8-bit code, gain = 256 / (256 - code). */
static esp_err_t imx219_set_analog_gain(esp_cam_sensor_device_t *dev, uint32_t code)
{
    if (code > IMX219_ANA_GAIN_MAX) {
        code = IMX219_ANA_GAIN_MAX;
    }
    return imx219_write(dev->sccb_handle, IMX219_REG_ANALOG_GAIN, (uint8_t)code);
}

/* Total gain by menu index - what the ISP's AE actually calls. */
static esp_err_t imx219_set_gain_index(esp_cam_sensor_device_t *dev, uint32_t index)
{
    if (index >= ARRAY_SIZE(imx219_ana_gain_code_map)) {
        index = ARRAY_SIZE(imx219_ana_gain_code_map) - 1;
    }
    esp_err_t ret = imx219_write(dev->sccb_handle, IMX219_REG_ANALOG_GAIN,
                                 imx219_ana_gain_code_map[index]);
    if (ret == ESP_OK && dev->priv) {
        ((imx219_para_t *)dev->priv)->gain_index = index;
    }
    return ret;
}

/* Digital gain: 16-bit, 0x0100 = 1.0x. */
static esp_err_t imx219_set_digital_gain(esp_cam_sensor_device_t *dev, uint32_t val)
{
    if (val < IMX219_DGTL_GAIN_MIN) {
        val = IMX219_DGTL_GAIN_MIN;
    }
    if (val > IMX219_DGTL_GAIN_MAX) {
        val = IMX219_DGTL_GAIN_MAX;
    }
    return imx219_write16(dev->sccb_handle, IMX219_REG_DIGITAL_GAIN_H, (uint16_t)val);
}

static esp_err_t imx219_set_test_pattern(esp_cam_sensor_device_t *dev, int enable)
{
    return imx219_write16(dev->sccb_handle, IMX219_REG_TEST_PATTERN_H,
                          enable ? IMX219_TEST_PATTERN_COLORBARS : IMX219_TEST_PATTERN_DISABLE);
}

static esp_err_t imx219_query_para_desc(esp_cam_sensor_device_t *dev, esp_cam_sensor_param_desc_t *qdesc)
{
    esp_err_t ret = ESP_OK;
    switch (qdesc->id) {
    case ESP_CAM_SENSOR_VFLIP:
    case ESP_CAM_SENSOR_HMIRROR:
        qdesc->type = ESP_CAM_SENSOR_PARAM_TYPE_NUMBER;
        qdesc->number.minimum = 0;
        qdesc->number.maximum = 1;
        qdesc->number.step = 1;
        qdesc->default_value = 0;
        break;
    case ESP_CAM_SENSOR_EXPOSURE_VAL:
        qdesc->type = ESP_CAM_SENSOR_PARAM_TYPE_NUMBER;
        qdesc->number.minimum = IMX219_EXPOSURE_MIN;
        /* Report what set_exposure will actually accept, so the AE's model of
           the control matches the sensor's behaviour. */
        qdesc->number.maximum = imx219_exposure_max(dev);
        qdesc->number.step = 1;
        qdesc->default_value = IMX219_EXPOSURE_DEFAULT;
        break;
    case ESP_CAM_SENSOR_GAIN:
        /* Menu control: elements are total gain in milli-units, and the value
           set later is an INDEX into this table, not a register code. */
        qdesc->type = ESP_CAM_SENSOR_PARAM_TYPE_ENUMERATION;
        qdesc->enumeration.count = ARRAY_SIZE(imx219_total_gain_val_map);
        qdesc->enumeration.elements = imx219_total_gain_val_map;
        qdesc->default_value = IMX219_DEFAULT_GAIN_INDEX;
        break;
    case ESP_CAM_SENSOR_ANGAIN:
        qdesc->type = ESP_CAM_SENSOR_PARAM_TYPE_NUMBER;
        qdesc->number.minimum = IMX219_ANA_GAIN_MIN;
        qdesc->number.maximum = IMX219_ANA_GAIN_MAX;
        qdesc->number.step = 1;
        qdesc->default_value = IMX219_ANA_GAIN_DEFAULT;
        break;
    case ESP_CAM_SENSOR_DGAIN:
        qdesc->type = ESP_CAM_SENSOR_PARAM_TYPE_NUMBER;
        qdesc->number.minimum = IMX219_DGTL_GAIN_MIN;
        qdesc->number.maximum = IMX219_DGTL_GAIN_MAX;
        qdesc->number.step = 1;
        qdesc->default_value = IMX219_DGTL_GAIN_DEFAULT;
        break;
    default:
        ESP_LOGD(TAG, "id=%" PRIx32 " not supported", qdesc->id);
        ret = ESP_ERR_INVALID_ARG;
        break;
    }
    return ret;
}

static esp_err_t imx219_get_para_value(esp_cam_sensor_device_t *dev, uint32_t id, void *arg, size_t size)
{
    imx219_para_t *para = (imx219_para_t *)dev->priv;

    if (para == NULL || arg == NULL || size < sizeof(uint32_t)) {
        return ESP_ERR_INVALID_ARG;
    }

    switch (id) {
    case ESP_CAM_SENSOR_EXPOSURE_VAL:
        *(uint32_t *)arg = para->exposure_val;
        break;
    case ESP_CAM_SENSOR_GAIN:
        *(uint32_t *)arg = para->gain_index;
        break;
    default:
        return ESP_ERR_NOT_SUPPORTED;
    }
    return ESP_OK;
}

static esp_err_t imx219_set_para_value(esp_cam_sensor_device_t *dev, uint32_t id, const void *arg, size_t size)
{
    esp_err_t ret = ESP_OK;
    switch (id) {
    case ESP_CAM_SENSOR_VFLIP:
        ret = imx219_set_vflip(dev, *(const int *)arg);
        break;
    case ESP_CAM_SENSOR_HMIRROR:
        ret = imx219_set_mirror(dev, *(const int *)arg);
        break;
    case ESP_CAM_SENSOR_EXPOSURE_VAL:
        ret = imx219_set_exposure(dev, *(const uint32_t *)arg);
        break;
    case ESP_CAM_SENSOR_EXPOSURE_US: {
        uint32_t us = *(const uint32_t *)arg;
        uint32_t lines = (uint32_t)(((uint64_t)us * 1000) / IMX219_TLINE_NS);
        ret = imx219_set_exposure(dev, lines);
        break;
    }
    case ESP_CAM_SENSOR_GAIN:
        ret = imx219_set_gain_index(dev, *(const uint32_t *)arg);
        break;
    case ESP_CAM_SENSOR_ANGAIN:
        ret = imx219_set_analog_gain(dev, *(const uint32_t *)arg);
        break;
    case ESP_CAM_SENSOR_GROUP_EXP_GAIN: {
        /* The ISP prefers this: exposure and gain applied together, so a frame
           never lands with one updated and the other not. */
        const esp_cam_sensor_gh_exp_gain_t *g = (const esp_cam_sensor_gh_exp_gain_t *)arg;
        uint32_t lines;
        if (g->exposure_val != 0) {
            lines = g->exposure_val;
        } else if (g->exposure_us != 0) {
            lines = (uint32_t)(((uint64_t)g->exposure_us * 1000) / IMX219_TLINE_NS);
        } else {
            ret = ESP_ERR_INVALID_ARG;
            break;
        }
        ret = imx219_set_exposure(dev, lines);
        if (ret == ESP_OK) {
            ret = imx219_set_gain_index(dev, g->gain_index);
        }
        break;
    }
    case ESP_CAM_SENSOR_DGAIN:
        ret = imx219_set_digital_gain(dev, *(const uint32_t *)arg);
        break;
    default:
        ESP_LOGE(TAG, "set id=%" PRIx32 " not supported", id);
        ret = ESP_ERR_INVALID_ARG;
        break;
    }
    return ret;
}

static esp_err_t imx219_query_support_formats(esp_cam_sensor_device_t *dev, esp_cam_sensor_format_array_t *formats)
{
    ESP_CAM_SENSOR_NULL_POINTER_CHECK(TAG, dev);
    ESP_CAM_SENSOR_NULL_POINTER_CHECK(TAG, formats);
    formats->count = ARRAY_SIZE(imx219_format_info);
    formats->format_array = &imx219_format_info[0];
    return ESP_OK;
}

static esp_err_t imx219_query_support_capability(esp_cam_sensor_device_t *dev, esp_cam_sensor_capability_t *caps)
{
    ESP_CAM_SENSOR_NULL_POINTER_CHECK(TAG, dev);
    ESP_CAM_SENSOR_NULL_POINTER_CHECK(TAG, caps);
    caps->fmt_raw = 1;
    return ESP_OK;
}

static esp_err_t imx219_set_format(esp_cam_sensor_device_t *dev, const esp_cam_sensor_format_t *format)
{
    ESP_CAM_SENSOR_NULL_POINTER_CHECK(TAG, dev);
    esp_err_t ret = ESP_OK;

    if (format == NULL) {
        format = &imx219_format_info[IMX219_DEFAULT_FORMAT_INDEX];
    }

    /* Software reset, then apply: common -> 2-lane PLL -> pixel format -> mode. */
    ret = imx219_soft_reset(dev);
    ESP_RETURN_ON_FALSE(ret == ESP_OK, ret, TAG, "soft reset failed");
    ret = imx219_write_array(dev->sccb_handle, imx219_common_regs);
    ESP_RETURN_ON_FALSE(ret == ESP_OK, ret, TAG, "write common regs failed");
    ret = imx219_write_array(dev->sccb_handle, imx219_2lane_regs);
    ESP_RETURN_ON_FALSE(ret == ESP_OK, ret, TAG, "write 2-lane regs failed");
    ret = imx219_write_array(dev->sccb_handle, imx219_raw10_regs);
    ESP_RETURN_ON_FALSE(ret == ESP_OK, ret, TAG, "write raw10 regs failed");
    ret = imx219_write_array(dev->sccb_handle, (const imx219_reginfo_t *)format->regs);
    ESP_RETURN_ON_FALSE(ret == ESP_OK, ret, TAG, "write mode regs failed");

    /* Before the exposure write: its ceiling is derived from the mode's VTS,
       so cur_format has to already be the mode whose registers just went in. */
    dev->cur_format = format;

    /* Sensible default exposure so the first frames are not black. */
    imx219_set_exposure(dev, IMX219_EXPOSURE_DEFAULT);
    imx219_set_gain_index(dev, IMX219_DEFAULT_GAIN_INDEX);

    ESP_LOGI(TAG, "set format: %s", format->name);
    return ret;
}

static esp_err_t imx219_get_format(esp_cam_sensor_device_t *dev, esp_cam_sensor_format_t *format)
{
    ESP_CAM_SENSOR_NULL_POINTER_CHECK(TAG, dev);
    ESP_CAM_SENSOR_NULL_POINTER_CHECK(TAG, format);
    if (dev->cur_format == NULL) {
        return ESP_FAIL;
    }
    memcpy(format, dev->cur_format, sizeof(esp_cam_sensor_format_t));
    return ESP_OK;
}

static esp_err_t imx219_priv_ioctl(esp_cam_sensor_device_t *dev, uint32_t cmd, void *arg)
{
    ESP_CAM_SENSOR_NULL_POINTER_CHECK(TAG, dev);
    esp_err_t ret = ESP_OK;
    uint8_t regval = 0;
    esp_cam_sensor_reg_val_t *sensor_reg;

    switch (cmd) {
    case ESP_CAM_SENSOR_IOC_HW_RESET:
        ret = imx219_hw_reset(dev);
        break;
    case ESP_CAM_SENSOR_IOC_SW_RESET:
        ret = imx219_soft_reset(dev);
        break;
    case ESP_CAM_SENSOR_IOC_S_STREAM:
        ret = imx219_set_stream(dev, *(int *)arg);
        break;
    case ESP_CAM_SENSOR_IOC_S_TEST_PATTERN:
        ret = imx219_set_test_pattern(dev, *(int *)arg);
        break;
    case ESP_CAM_SENSOR_IOC_S_REG:
        sensor_reg = (esp_cam_sensor_reg_val_t *)arg;
        ret = imx219_write(dev->sccb_handle, sensor_reg->regaddr, sensor_reg->value);
        break;
    case ESP_CAM_SENSOR_IOC_G_REG:
        sensor_reg = (esp_cam_sensor_reg_val_t *)arg;
        ret = imx219_read(dev->sccb_handle, sensor_reg->regaddr, &regval);
        if (ret == ESP_OK) {
            sensor_reg->value = regval;
        }
        break;
    case ESP_CAM_SENSOR_IOC_G_CHIP_ID:
        ret = imx219_get_sensor_id(dev, (esp_cam_sensor_id_t *)arg);
        break;
    default:
        ret = ESP_ERR_INVALID_ARG;
        break;
    }
    return ret;
}

static esp_err_t imx219_power_on(esp_cam_sensor_device_t *dev)
{
    esp_err_t ret = ESP_OK;

    if (dev->pwdn_pin >= 0) {
        gpio_config_t conf = { 0 };
        conf.pin_bit_mask = 1LL << dev->pwdn_pin;
        conf.mode = GPIO_MODE_OUTPUT;
        ret = gpio_config(&conf);
        ESP_RETURN_ON_FALSE(ret == ESP_OK, ret, TAG, "pwdn pin config failed");
        gpio_set_level(dev->pwdn_pin, 1);   /* power up */
        delay_ms(10);
    }

    if (dev->reset_pin >= 0) {
        gpio_config_t conf = { 0 };
        conf.pin_bit_mask = 1LL << dev->reset_pin;
        conf.mode = GPIO_MODE_OUTPUT;
        ret = gpio_config(&conf);
        ESP_RETURN_ON_FALSE(ret == ESP_OK, ret, TAG, "reset pin config failed");
        gpio_set_level(dev->reset_pin, 0);
        delay_ms(10);
        gpio_set_level(dev->reset_pin, 1);
        delay_ms(10);   /* >= 6.2ms before first I2C access */
    }

    return ret;
}

static esp_err_t imx219_power_off(esp_cam_sensor_device_t *dev)
{
    if (dev->reset_pin >= 0) {
        gpio_set_level(dev->reset_pin, 0);
    }
    if (dev->pwdn_pin >= 0) {
        gpio_set_level(dev->pwdn_pin, 0);
    }
    return ESP_OK;
}

static esp_err_t imx219_delete(esp_cam_sensor_device_t *dev)
{
    ESP_LOGD(TAG, "del imx219 (%p)", dev);
    if (dev) {
        free(dev);
    }
    return ESP_OK;
}

static const esp_cam_sensor_ops_t imx219_ops = {
    .query_para_desc = imx219_query_para_desc,
    .get_para_value = imx219_get_para_value,
    .set_para_value = imx219_set_para_value,
    .query_support_formats = imx219_query_support_formats,
    .query_support_capability = imx219_query_support_capability,
    .set_format = imx219_set_format,
    .get_format = imx219_get_format,
    .priv_ioctl = imx219_priv_ioctl,
    .del = imx219_delete,
};

esp_cam_sensor_device_t *imx219_detect(esp_cam_sensor_config_t *config)
{
    if (config == NULL) {
        return NULL;
    }

    esp_cam_sensor_device_t *dev = calloc(1, sizeof(esp_cam_sensor_device_t) + sizeof(imx219_para_t));
    if (dev == NULL) {
        ESP_LOGE(TAG, "no memory for camera device");
        return NULL;
    }
    dev->priv = (uint8_t *)dev + sizeof(esp_cam_sensor_device_t);

    dev->name = (char *)IMX219_SENSOR_NAME;
    dev->sccb_handle = config->sccb_handle;
    dev->xclk_pin = config->xclk_pin;
    dev->reset_pin = config->reset_pin;
    dev->pwdn_pin = config->pwdn_pin;
    dev->sensor_port = config->sensor_port;
    dev->ops = &imx219_ops;
    dev->cur_format = &imx219_format_info[IMX219_DEFAULT_FORMAT_INDEX];

    if (config->sensor_port != ESP_CAM_SENSOR_MIPI_CSI) {
        ESP_LOGE(TAG, "only MIPI-CSI is supported");
        goto err;
    }

    if (imx219_power_on(dev) != ESP_OK) {
        ESP_LOGE(TAG, "power on failed");
        goto err;
    }

    if (imx219_get_sensor_id(dev, &dev->id) != ESP_OK) {
        ESP_LOGE(TAG, "get chip id failed");
        goto err;
    }
    if (dev->id.pid != IMX219_CHIP_ID_VALUE) {
        ESP_LOGE(TAG, "sensor is not IMX219, PID=0x%04x", dev->id.pid);
        goto err;
    }
    ESP_LOGI(TAG, "detected IMX219, PID=0x%04x", dev->id.pid);
    return dev;

err:
    imx219_power_off(dev);
    free(dev);
    return NULL;
}

#if CONFIG_CAMERA_IMX219_AUTO_DETECT_MIPI_INTERFACE_SENSOR
ESP_CAM_SENSOR_DETECT_FN(imx219_detect, ESP_CAM_SENSOR_MIPI_CSI, IMX219_SCCB_ADDR)
{
    ((esp_cam_sensor_config_t *)config)->sensor_port = ESP_CAM_SENSOR_MIPI_CSI;
    return imx219_detect(config);
}
#endif
