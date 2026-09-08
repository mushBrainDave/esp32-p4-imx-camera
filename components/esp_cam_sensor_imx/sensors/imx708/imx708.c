/*
 * SPDX-FileCopyrightText: 2026 esp_cam_sensor_imx contributors
 *
 * SPDX-License-Identifier: Apache-2.0
 *
 * Sony IMX708 (Raspberry Pi Camera Module 3 / NoIR 3) driver for the ESP32-P4
 * esp_cam_sensor framework. One 2x2 binned RAW10 readout at 28 fps, offered as
 * three centred digital crops of it: 1920x1080, 1280x720 and 640x480.
 * Autofocus VCM (I2C 0x0c) and PDAF are out of scope for now — the lens parks
 * at its rest position.
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
#include "imx708_settings.h"
#include "imx708.h"

#ifndef portTICK_RATE_MS
#define portTICK_RATE_MS portTICK_PERIOD_MS
#endif
#define delay_ms(ms) vTaskDelay((ms > portTICK_PERIOD_MS ? ms / portTICK_PERIOD_MS : 1))

/* Binned-mode timing (from the Sony reference mode table). */
#define IMX708_PIXEL_RATE         585600000
#define IMX708_HTS                7824    /* line_length_pix 0x1e90 */
#define IMX708_VTS                2672    /* frame_length 0x0a70    */
#define IMX708_TLINE_NS           13361   /* HTS / pixel_rate, ns   */
/* 450 MHz link freq -> 900 Mbps per lane, 2 lanes */
#define IMX708_MIPI_CSI_LINE_RATE 900000000

/*
 * Feeds has_line_start_packet / has_line_end_packet on the P4 ISP. With it off
 * the ISP has no per-line delimiter and free-runs on h_res, so any slip at the
 * start of a line stays for the whole line - a fixed, rate-invariant band of
 * wrong pixels at each edge. Espressif's own MIPI sensors all enable it.
 */
#define IMX708_LINESYNC_ENABLE    1

static const char *TAG = "imx708";

/* ------------------------------------------------------------------ */
/* Formats                                                             */
/* ------------------------------------------------------------------ */
/*
 * Mode indices. These are what CAMERA_IMX708_MIPI_IF_FORMAT_INDEX_DEFAULT
 * selects between and what imx708_format_by_index() hands out, so a mode's
 * number is part of the driver's interface: renumbering silently changes what
 * an existing build comes up in, with nothing to warn the person who wrote
 * that number down. **Append here, do not insert.**
 *
 * The table is ordered largest to smallest, and while this release was still
 * unreleased that ordering was maintained by inserting rather than appending -
 * 1024x768 at 2 and then 800x600 at 3, moving 640x480 twice. That was free
 * only because 0.2.0 released a single 1920x1080 mode, so index 1 and above
 * existed nowhere but this repository. It stops being free the moment this
 * release ships, and the ordering is a convenience where the numbering is an
 * interface. When the two conflict, append: the table stops being sorted
 * rather than a mode changing its number.
 */
enum {
    IMX708_FMT_1920x1080_RAW10_28FPS = 0,
    IMX708_FMT_1280x720_RAW10_28FPS,
    IMX708_FMT_1024x768_RAW10_28FPS,
    IMX708_FMT_800x600_RAW10_28FPS,
    IMX708_FMT_640x480_RAW10_28FPS,
    IMX708_FMT_MAX,
};

/*
 * One entry per mode even though all three are currently identical: every mode
 * is a digital crop of the same binned readout, so line and frame timing - and
 * with them the frame rate and the whole exposure range - do not move. Keeping
 * them separate is what lets a future mode change its own VTS without the
 * others silently inheriting it.
 */
static const esp_cam_sensor_isp_info_t imx708_isp_info[] = {
    [IMX708_FMT_1920x1080_RAW10_28FPS] = {
        .isp_v1_info = {
            .version = SENSOR_ISP_INFO_VERSION_DEFAULT,
            .pclk = IMX708_PIXEL_RATE,
            .hts = IMX708_HTS,
            .vts = IMX708_VTS,
            .exp_def = 2000,
            .gain_def = IMX708_ANA_GAIN_DEFAULT,
            .tline_ns = IMX708_TLINE_NS,
            .bayer_type = ESP_CAM_SENSOR_BAYER_RGGB,
        }
    },
    [IMX708_FMT_1280x720_RAW10_28FPS] = {
        .isp_v1_info = {
            .version = SENSOR_ISP_INFO_VERSION_DEFAULT,
            .pclk = IMX708_PIXEL_RATE,
            .hts = IMX708_HTS,
            .vts = IMX708_VTS,
            .exp_def = 2000,
            .gain_def = IMX708_ANA_GAIN_DEFAULT,
            .tline_ns = IMX708_TLINE_NS,
            .bayer_type = ESP_CAM_SENSOR_BAYER_RGGB,
        }
    },
    [IMX708_FMT_1024x768_RAW10_28FPS] = {
        .isp_v1_info = {
            .version = SENSOR_ISP_INFO_VERSION_DEFAULT,
            .pclk = IMX708_PIXEL_RATE,
            .hts = IMX708_HTS,
            .vts = IMX708_VTS,
            .exp_def = 2000,
            .gain_def = IMX708_ANA_GAIN_DEFAULT,
            .tline_ns = IMX708_TLINE_NS,
            .bayer_type = ESP_CAM_SENSOR_BAYER_RGGB,
        }
    },
    [IMX708_FMT_800x600_RAW10_28FPS] = {
        .isp_v1_info = {
            .version = SENSOR_ISP_INFO_VERSION_DEFAULT,
            .pclk = IMX708_PIXEL_RATE,
            .hts = IMX708_HTS,
            .vts = IMX708_VTS,
            .exp_def = 2000,
            .gain_def = IMX708_ANA_GAIN_DEFAULT,
            .tline_ns = IMX708_TLINE_NS,
            .bayer_type = ESP_CAM_SENSOR_BAYER_RGGB,
        }
    },
    [IMX708_FMT_640x480_RAW10_28FPS] = {
        .isp_v1_info = {
            .version = SENSOR_ISP_INFO_VERSION_DEFAULT,
            .pclk = IMX708_PIXEL_RATE,
            .hts = IMX708_HTS,
            .vts = IMX708_VTS,
            .exp_def = 2000,
            .gain_def = IMX708_ANA_GAIN_DEFAULT,
            .tline_ns = IMX708_TLINE_NS,
            .bayer_type = ESP_CAM_SENSOR_BAYER_RGGB,
        }
    },
};

static const esp_cam_sensor_format_t imx708_format_info[] = {
    [IMX708_FMT_1920x1080_RAW10_28FPS] = {
        /* 1920 wide keeps the line inside what the P4 CSI/ISP path can carry;
           see imx708_mode_1920x1080_regs for the evidence behind that. */
        .name = "MIPI_2lane_24Minput_RAW10_1920x1080_binned_28fps",
        .format = ESP_CAM_SENSOR_PIXFORMAT_RAW10,
        .port = ESP_CAM_SENSOR_MIPI_CSI,
        .xclk = IMX708_INCLK_FREQ_HZ,
        .width = 1920,
        .height = 1080,
        .regs = imx708_mode_1920x1080_regs,
        .regs_size = ARRAY_SIZE(imx708_mode_1920x1080_regs),
        .fps = 28,
        .isp_info = &imx708_isp_info[IMX708_FMT_1920x1080_RAW10_28FPS],
        .mipi_info = {
            .mipi_clk = IMX708_MIPI_CSI_LINE_RATE,
            .lane_num = 2,
            .line_sync_en = IMX708_LINESYNC_ENABLE,
        },
        .reserved = NULL,
    },
    [IMX708_FMT_1280x720_RAW10_28FPS] = {
        /* A centred crop of the same readout: 44% of the pixels, 56% of the
           field of view each way, and the same 28 fps. Both axes are whole
           H.264 macroblocks, so nothing has to be trimmed before the encoder. */
        .name = "MIPI_2lane_24Minput_RAW10_1280x720_binned_28fps",
        .format = ESP_CAM_SENSOR_PIXFORMAT_RAW10,
        .port = ESP_CAM_SENSOR_MIPI_CSI,
        .xclk = IMX708_INCLK_FREQ_HZ,
        .width = 1280,
        .height = 720,
        .regs = imx708_mode_1280x720_regs,
        .regs_size = ARRAY_SIZE(imx708_mode_1280x720_regs),
        .fps = 28,
        .isp_info = &imx708_isp_info[IMX708_FMT_1280x720_RAW10_28FPS],
        .mipi_info = {
            .mipi_clk = IMX708_MIPI_CSI_LINE_RATE,
            .lane_num = 2,
            .line_sync_en = IMX708_LINESYNC_ENABLE,
        },
        .reserved = NULL,
    },
    [IMX708_FMT_1024x768_RAW10_28FPS] = {
        /* 4:3, and the only mode whose width and height are both whole H.264
           macroblocks (64x48) - 1080 is not. Sees less across than mode 1 and
           more up and down, on fewer pixels. */
        .name = "MIPI_2lane_24Minput_RAW10_1024x768_binned_28fps",
        .format = ESP_CAM_SENSOR_PIXFORMAT_RAW10,
        .port = ESP_CAM_SENSOR_MIPI_CSI,
        .xclk = IMX708_INCLK_FREQ_HZ,
        .width = 1024,
        .height = 768,
        .regs = imx708_mode_1024x768_regs,
        .regs_size = ARRAY_SIZE(imx708_mode_1024x768_regs),
        .fps = 28,
        .isp_info = &imx708_isp_info[IMX708_FMT_1024x768_RAW10_28FPS],
        .mipi_info = {
            .mipi_clk = IMX708_MIPI_CSI_LINE_RATE,
            .lane_num = 2,
            .line_sync_en = IMX708_LINESYNC_ENABLE,
        },
        .reserved = NULL,
    },
    [IMX708_FMT_800x600_RAW10_28FPS] = {
        /* SVGA, the middle 4:3 step. 800 is 50 whole macroblocks; 600 is 37.5,
           so this and 1080 are the only modes needing the encoder's height
           trim. Width alignment is the half that matters - it cannot be
           trimmed without shearing the picture. */
        .name = "MIPI_2lane_24Minput_RAW10_800x600_binned_28fps",
        .format = ESP_CAM_SENSOR_PIXFORMAT_RAW10,
        .port = ESP_CAM_SENSOR_MIPI_CSI,
        .xclk = IMX708_INCLK_FREQ_HZ,
        .width = 800,
        .height = 600,
        .regs = imx708_mode_800x600_regs,
        .regs_size = ARRAY_SIZE(imx708_mode_800x600_regs),
        .fps = 28,
        .isp_info = &imx708_isp_info[IMX708_FMT_800x600_RAW10_28FPS],
        .mipi_info = {
            .mipi_clk = IMX708_MIPI_CSI_LINE_RATE,
            .lane_num = 2,
            .line_sync_en = IMX708_LINESYNC_ENABLE,
        },
        .reserved = NULL,
    },
    [IMX708_FMT_640x480_RAW10_28FPS] = {
        /* 15% of the pixels, and a much tighter window - 28% of the field
           horizontally, 37% vertically. 4:3 out of a 16:9 field, so this is a
           different framing of the scene rather than a smaller copy of it. */
        .name = "MIPI_2lane_24Minput_RAW10_640x480_binned_28fps",
        .format = ESP_CAM_SENSOR_PIXFORMAT_RAW10,
        .port = ESP_CAM_SENSOR_MIPI_CSI,
        .xclk = IMX708_INCLK_FREQ_HZ,
        .width = 640,
        .height = 480,
        .regs = imx708_mode_640x480_regs,
        .regs_size = ARRAY_SIZE(imx708_mode_640x480_regs),
        .fps = 28,
        .isp_info = &imx708_isp_info[IMX708_FMT_640x480_RAW10_28FPS],
        .mipi_info = {
            .mipi_clk = IMX708_MIPI_CSI_LINE_RATE,
            .lane_num = 2,
            .line_sync_en = IMX708_LINESYNC_ENABLE,
        },
        .reserved = NULL,
    },
};

/*
 * Fall back to mode 0 if the Kconfig symbol is absent. It will be, in any build
 * tree whose sdkconfig predates this option and has not been reconfigured -
 * and without this the failure is "CONFIG_..._INDEX_DEFAULT undeclared" from
 * inside the driver, which points at the wrong thing entirely. This way a stale
 * tree quietly keeps the behaviour it already had.
 */
#ifndef CONFIG_CAMERA_IMX708_MIPI_IF_FORMAT_INDEX_DEFAULT
#define CONFIG_CAMERA_IMX708_MIPI_IF_FORMAT_INDEX_DEFAULT 0
#endif

#define IMX708_DEFAULT_FORMAT_INDEX CONFIG_CAMERA_IMX708_MIPI_IF_FORMAT_INDEX_DEFAULT

/*
 * Stated against the enum rather than a literal, so it keeps itself honest as
 * modes are added. The Kconfig range is the one place still counting by hand,
 * and it is only an input check: a value that slips past it lands here.
 */
_Static_assert(IMX708_DEFAULT_FORMAT_INDEX < IMX708_FMT_MAX,
               "CAMERA_IMX708_MIPI_IF_FORMAT_INDEX_DEFAULT is out of range");
_Static_assert(IMX708_FMT_MAX == ARRAY_SIZE(imx708_format_info),
               "mode index enum and format table disagree");
_Static_assert(IMX708_FMT_MAX == ARRAY_SIZE(imx708_isp_info),
               "mode index enum and isp_info table disagree");

/* ------------------------------------------------------------------ */
/* Mode lookup for applications                                        */
/* ------------------------------------------------------------------ */
/*
 * esp_video can change the sensor mode while the application is running -
 * VIDIOC_S_SENSOR_FMT takes an esp_cam_sensor_format_t and resizes the stream
 * buffers around it - but it offers no way to come by one. VIDIOC_ENUM_FRAMESIZES
 * only ever reports the mode already in use, and the sensor handle itself is
 * out of reach for an application that let esp_video auto-detect the camera.
 * These give such an application something to pass.
 *
 * They return pointers into the static table above, and must keep doing so:
 * imx708_set_format() stores the caller's pointer in dev->cur_format and
 * esp_video caches it a second time, so handing back the address of a copy
 * would leave both reading freed memory as soon as the caller's frame went.
 */
size_t imx708_format_count(void)
{
    return ARRAY_SIZE(imx708_format_info);
}

const esp_cam_sensor_format_t *imx708_format_by_index(size_t index)
{
    if (index >= ARRAY_SIZE(imx708_format_info)) {
        return NULL;
    }
    return &imx708_format_info[index];
}

const esp_cam_sensor_format_t *imx708_format_by_size(uint16_t width, uint16_t height)
{
    for (size_t i = 0; i < ARRAY_SIZE(imx708_format_info); i++) {
        if (imx708_format_info[i].width == width && imx708_format_info[i].height == height) {
            return &imx708_format_info[i];
        }
    }
    return NULL;
}

/* ------------------------------------------------------------------ */
/* Gain table                                                          */
/* ------------------------------------------------------------------ */
/*
 * esp_video drives AE gain as a *menu* control: it calls VIDIOC_QUERYMENU to
 * read each entry's total gain and binary-searches for the one nearest the
 * target, then sets the winning INDEX. So ESP_CAM_SENSOR_GAIN has to be an
 * enumeration of gain values in milli-units (1000 = 1.00x) - declaring it as a
 * plain number makes the query fail and the AE silently drives exposure only.
 *
 * IMX708 analog gain is gain = 1024 / (1024 - code) for code 112..960, i.e.
 * 1.123x to 16x. These 47 entries step it at roughly 1/12 stop.
 */
static const uint32_t imx708_total_gain_val_map[] = {
     1123,  1189,  1261,  1335,  1414,  1499,  1588,  1681,
     1781,  1889,  2000,  2120,  2246,  2381,  2522,  2674,
     2829,  2994,  3180,  3368,  3568,  3779,  4000,  4231,
     4491,  4763,  5044,  5333,  5657,  5988,  6360,  6737,
     7111,  7529,  8000,  8463,  8982,  9481, 10039, 10667,
    11378, 12047, 12642, 13474, 14222, 15059, 16000,
};

static const uint16_t imx708_ana_gain_code_map[] = {
     112,  163,  212,  257,  300,  341,  379,  415,
     449,  482,  512,  541,  568,  594,  618,  641,
     662,  682,  702,  720,  737,  753,  768,  782,
     796,  809,  821,  832,  843,  853,  863,  872,
     880,  888,  896,  903,  910,  916,  922,  928,
     934,  939,  943,  948,  952,  956,  960,
};

/* ------------------------------------------------------------------ */
/* Bayer phase                                                         */
/* ------------------------------------------------------------------ */
/*
 * Bayer phase by flip state, indexed (vflip << 1) | hmirror.
 *
 * A flip does not mirror the picture behind a fixed colour mosaic. It reverses
 * the order pixels leave the array in, so the 2x2 tile the ISP has to
 * demosaic rotates with it. Report the wrong one and red and blue swap while
 * the greens zipper - and nothing anywhere returns an error, the frame is
 * simply wrong. Raspberry Pi's driver does the same thing by swapping the
 * media-bus code across four entries.
 *
 * The unflipped phase is RGGB for every mode here because all of them are
 * centred crops of one binned readout whose origin is a multiple of 4 in both
 * axes. A future mode that moved the analog window by an odd number of binned
 * pixels would start from a different phase and need its own base, so this is
 * a property of the readout rather than of the sensor.
 */
static const esp_cam_sensor_bayer_pattern_t imx708_bayer_by_flip[4] = {
    [0] = ESP_CAM_SENSOR_BAYER_RGGB,    /* no flip  */
    [1] = ESP_CAM_SENSOR_BAYER_GRBG,    /* h mirror */
    [2] = ESP_CAM_SENSOR_BAYER_GBRG,    /* v flip   */
    [3] = ESP_CAM_SENSOR_BAYER_BGGR,    /* both     */
};

/*
 * Per-device state.
 *
 * `format` and `isp_info` shadow whichever entry of imx708_format_info[] is in
 * use, and dev->cur_format points at the shadow rather than into the table.
 * That indirection is what lets bayer_type follow the flip bits: the static
 * table is const and shared between every device, while esp_video reads
 * bayer_type straight out of whatever cur_format points at. Every other field
 * is a verbatim copy of the static entry.
 *
 * The shadow lives in the same allocation as the device (see imx708_detect),
 * so it is valid for exactly as long as anything can hold a pointer to it.
 */
typedef struct {
    uint32_t exposure_val;      /*!< current exposure, in lines */
    uint32_t gain_index;        /*!< index into imx708_total_gain_val_map */
    uint8_t  hmirror;           /*!< 0x0101 bit 0 */
    uint8_t  vflip;             /*!< 0x0101 bit 1 */
    esp_cam_sensor_format_t   format;
    esp_cam_sensor_isp_info_t isp_info;
} imx708_para_t;

/* ------------------------------------------------------------------ */
/* SCCB helpers (16-bit reg addr, 8-bit value)                         */
/* ------------------------------------------------------------------ */
static esp_err_t imx708_read(esp_sccb_io_handle_t sccb, uint16_t reg, uint8_t *val)
{
    return esp_sccb_transmit_receive_reg_a16v8(sccb, reg, val);
}

static esp_err_t imx708_write(esp_sccb_io_handle_t sccb, uint16_t reg, uint8_t val)
{
    return esp_sccb_transmit_reg_a16v8(sccb, reg, val);
}

/* Write a 16-bit value big-endian to reg / reg+1 */
static esp_err_t imx708_write16(esp_sccb_io_handle_t sccb, uint16_t reg, uint16_t val)
{
    esp_err_t ret = imx708_write(sccb, reg, (val >> 8) & 0xff);
    if (ret == ESP_OK) {
        ret = imx708_write(sccb, reg + 1, val & 0xff);
    }
    return ret;
}

static esp_err_t imx708_write_array(esp_sccb_io_handle_t sccb, const imx708_reginfo_t *regs)
{
    esp_err_t ret = ESP_OK;
    int i = 0;
    while (ret == ESP_OK && regs[i].reg != IMX708_REG_END) {
        if (regs[i].reg == IMX708_REG_DELAY) {
            delay_ms(regs[i].val);
        } else {
            ret = imx708_write(sccb, regs[i].reg, regs[i].val);
        }
        i++;
    }
    ESP_LOGD(TAG, "wrote %d regs", i);
    return ret;
}

/* ------------------------------------------------------------------ */
/* Sensor operations                                                   */
/* ------------------------------------------------------------------ */
static esp_err_t imx708_get_sensor_id(esp_cam_sensor_device_t *dev, esp_cam_sensor_id_t *id)
{
    uint8_t h = 0, l = 0;
    esp_err_t ret = imx708_read(dev->sccb_handle, IMX708_REG_CHIP_ID_H, &h);
    ESP_RETURN_ON_FALSE(ret == ESP_OK, ret, TAG, "read chip id high failed");
    ret = imx708_read(dev->sccb_handle, IMX708_REG_CHIP_ID_L, &l);
    ESP_RETURN_ON_FALSE(ret == ESP_OK, ret, TAG, "read chip id low failed");
    id->pid = (h << 8) | l;
    return ESP_OK;
}

static esp_err_t imx708_set_stream(esp_cam_sensor_device_t *dev, int enable)
{
    esp_err_t ret = imx708_write(dev->sccb_handle, IMX708_REG_MODE_SELECT, enable ? 0x01 : 0x00);
    ESP_RETURN_ON_FALSE(ret == ESP_OK, ret, TAG, "set stream failed");
    dev->stream_status = enable;
    ESP_LOGD(TAG, "stream=%d", enable);
    return ret;
}

static esp_err_t imx708_hw_reset(esp_cam_sensor_device_t *dev)
{
    if (dev->reset_pin >= 0) {
        gpio_set_level(dev->reset_pin, 0);
        delay_ms(10);
        gpio_set_level(dev->reset_pin, 1);
        delay_ms(10);
    }
    return ESP_OK;
}

/*
 * Push the cached flip state to the sensor and to the shadow isp_info.
 *
 * Both bits are written together as a whole byte. The driver owns every bit of
 * 0x0101, so there is nothing to preserve, and a read-modify-write per bit
 * would leave the register momentarily holding a combination the caller never
 * asked for.
 *
 * Note what the ISP does and does not see. esp_video latches bayer_type into
 * its own CSI state inside update_format_config(), which runs at device open
 * and again on VIDIOC_S_SENSOR_FMT - not at STREAMON, and not when a control
 * is set. So the sensor obeys a flip immediately, while the ISP keeps
 * demosaicing on the phase it latched. Set flips before opening the device or
 * before the S_SENSOR_FMT that selects the mode; imx708_set_orientation()
 * warns when it is too late for that.
 */
static esp_err_t imx708_apply_orientation(esp_cam_sensor_device_t *dev)
{
    imx708_para_t *para = (imx708_para_t *)dev->priv;
    uint8_t regval = 0;

    if (para == NULL) {
        return ESP_ERR_INVALID_STATE;
    }
    if (para->hmirror) {
        regval |= IMX708_ORIENTATION_HMIRROR;
    }
    if (para->vflip) {
        regval |= IMX708_ORIENTATION_VFLIP;
    }

    esp_err_t ret = imx708_write(dev->sccb_handle, IMX708_REG_ORIENTATION, regval);
    if (ret != ESP_OK) {
        return ret;
    }

    para->isp_info.isp_v1_info.bayer_type =
        imx708_bayer_by_flip[(para->vflip ? 2 : 0) | (para->hmirror ? 1 : 0)];
    return ESP_OK;
}

static esp_err_t imx708_set_orientation(esp_cam_sensor_device_t *dev, int hmirror, int vflip)
{
    imx708_para_t *para = (imx708_para_t *)dev->priv;

    if (para == NULL) {
        return ESP_ERR_INVALID_STATE;
    }
    para->hmirror = hmirror ? 1 : 0;
    para->vflip = vflip ? 1 : 0;

    if (dev->stream_status) {
        ESP_LOGW(TAG, "flip changed while streaming: the sensor obeys now, but the ISP keeps "
                 "the Bayer phase it latched - re-set the sensor format to pick it up");
    }
    return imx708_apply_orientation(dev);
}

/*
 * Exposure ceiling. Integration time cannot exceed frame_length - 48 lines,
 * and frame_length is a per-mode value carried in isp_info, not a constant of
 * the sensor. Every mode shipped today is a crop of one readout and so shares
 * a VTS, but a mode that changed the frame rate would change it, and the
 * failure would be quiet: an exposure past frame_length silently does not take
 * effect, so the AE loop sees no response to its own request and keeps asking.
 */
static uint32_t imx708_exposure_max(esp_cam_sensor_device_t *dev)
{
    uint32_t vts = IMX708_VTS;

    if (dev && dev->cur_format && dev->cur_format->isp_info) {
        vts = dev->cur_format->isp_info->isp_v1_info.vts;
    }
    if (vts <= IMX708_EXPOSURE_OFFSET + IMX708_EXPOSURE_MIN) {
        return IMX708_EXPOSURE_MIN;
    }
    return vts - IMX708_EXPOSURE_OFFSET;
}

/* Exposure in lines (16-bit reg 0x0202). */
static esp_err_t imx708_set_exposure(esp_cam_sensor_device_t *dev, uint32_t lines)
{
    uint32_t max = imx708_exposure_max(dev);
    if (lines < IMX708_EXPOSURE_MIN) {
        lines = IMX708_EXPOSURE_MIN;
    }
    if (lines > max) {
        lines = max;
    }
    esp_err_t ret = imx708_write16(dev->sccb_handle, IMX708_REG_EXPOSURE_H, (uint16_t)lines);
    if (ret == ESP_OK && dev->priv) {
        ((imx708_para_t *)dev->priv)->exposure_val = lines;
    }
    return ret;
}

/* Analog gain: 16-bit reg 0x0204, code 112..960, gain = 1024/(1024-code). */
static esp_err_t imx708_set_analog_gain(esp_cam_sensor_device_t *dev, uint32_t code)
{
    if (code < IMX708_ANA_GAIN_MIN) {
        code = IMX708_ANA_GAIN_MIN;
    }
    if (code > IMX708_ANA_GAIN_MAX) {
        code = IMX708_ANA_GAIN_MAX;
    }
    return imx708_write16(dev->sccb_handle, IMX708_REG_ANALOG_GAIN_H, (uint16_t)code);
}

/* Total gain by menu index - what the ISP's AE actually calls. */
static esp_err_t imx708_set_gain_index(esp_cam_sensor_device_t *dev, uint32_t index)
{
    if (index >= ARRAY_SIZE(imx708_ana_gain_code_map)) {
        index = ARRAY_SIZE(imx708_ana_gain_code_map) - 1;
    }
    esp_err_t ret = imx708_write16(dev->sccb_handle, IMX708_REG_ANALOG_GAIN_H,
                                   imx708_ana_gain_code_map[index]);
    if (ret == ESP_OK && dev->priv) {
        ((imx708_para_t *)dev->priv)->gain_index = index;
    }
    return ret;
}

/* Digital gain: 16-bit reg 0x020e, 0x0100 = 1.0x. */
static esp_err_t imx708_set_digital_gain(esp_cam_sensor_device_t *dev, uint32_t val)
{
    if (val < IMX708_DGTL_GAIN_MIN) {
        val = IMX708_DGTL_GAIN_MIN;
    }
    if (val > IMX708_DGTL_GAIN_MAX) {
        val = IMX708_DGTL_GAIN_MAX;
    }
    return imx708_write16(dev->sccb_handle, IMX708_REG_DIGITAL_GAIN_H, (uint16_t)val);
}

static esp_err_t imx708_set_test_pattern(esp_cam_sensor_device_t *dev, int enable)
{
    return imx708_write16(dev->sccb_handle, IMX708_REG_TEST_PATTERN_H,
                          enable ? IMX708_TEST_PATTERN_COLORBARS : IMX708_TEST_PATTERN_DISABLE);
}

static esp_err_t imx708_query_para_desc(esp_cam_sensor_device_t *dev, esp_cam_sensor_param_desc_t *qdesc)
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
        qdesc->number.minimum = IMX708_EXPOSURE_MIN;
        qdesc->number.maximum = imx708_exposure_max(dev);
        qdesc->number.step = 1;
        qdesc->default_value = 1288;
        break;
    case ESP_CAM_SENSOR_GAIN:
        /* Menu control: elements are total gain in milli-units, and the value
           set later is an INDEX into this table, not a register code. */
        qdesc->type = ESP_CAM_SENSOR_PARAM_TYPE_ENUMERATION;
        qdesc->enumeration.count = ARRAY_SIZE(imx708_total_gain_val_map);
        qdesc->enumeration.elements = imx708_total_gain_val_map;
        qdesc->default_value = 0;
        break;
    case ESP_CAM_SENSOR_ANGAIN:
        qdesc->type = ESP_CAM_SENSOR_PARAM_TYPE_NUMBER;
        qdesc->number.minimum = IMX708_ANA_GAIN_MIN;
        qdesc->number.maximum = IMX708_ANA_GAIN_MAX;
        qdesc->number.step = 1;
        qdesc->default_value = IMX708_ANA_GAIN_DEFAULT;
        break;
    case ESP_CAM_SENSOR_DGAIN:
        qdesc->type = ESP_CAM_SENSOR_PARAM_TYPE_NUMBER;
        qdesc->number.minimum = IMX708_DGTL_GAIN_MIN;
        qdesc->number.maximum = IMX708_DGTL_GAIN_MAX;
        qdesc->number.step = 1;
        qdesc->default_value = IMX708_DGTL_GAIN_DEFAULT;
        break;
    default:
        ESP_LOGD(TAG, "id=%" PRIx32 " not supported", qdesc->id);
        ret = ESP_ERR_INVALID_ARG;
        break;
    }
    return ret;
}

static esp_err_t imx708_get_para_value(esp_cam_sensor_device_t *dev, uint32_t id, void *arg, size_t size)
{
    imx708_para_t *para = (imx708_para_t *)dev->priv;

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
    case ESP_CAM_SENSOR_HMIRROR:
        *(uint32_t *)arg = para->hmirror;
        break;
    case ESP_CAM_SENSOR_VFLIP:
        *(uint32_t *)arg = para->vflip;
        break;
    default:
        return ESP_ERR_NOT_SUPPORTED;
    }
    return ESP_OK;
}

static esp_err_t imx708_set_para_value(esp_cam_sensor_device_t *dev, uint32_t id, const void *arg, size_t size)
{
    esp_err_t ret = ESP_OK;
    switch (id) {
    case ESP_CAM_SENSOR_VFLIP: {
        const imx708_para_t *para = (const imx708_para_t *)dev->priv;
        ret = imx708_set_orientation(dev, para ? para->hmirror : 0, *(const int *)arg);
        break;
    }
    case ESP_CAM_SENSOR_HMIRROR: {
        const imx708_para_t *para = (const imx708_para_t *)dev->priv;
        ret = imx708_set_orientation(dev, *(const int *)arg, para ? para->vflip : 0);
        break;
    }
    case ESP_CAM_SENSOR_EXPOSURE_VAL:
        ret = imx708_set_exposure(dev, *(const uint32_t *)arg);
        break;
    case ESP_CAM_SENSOR_EXPOSURE_US: {
        uint32_t us = *(const uint32_t *)arg;
        uint32_t lines = (uint32_t)(((uint64_t)us * 1000) / IMX708_TLINE_NS);
        ret = imx708_set_exposure(dev, lines);
        break;
    }
    case ESP_CAM_SENSOR_GAIN:
        ret = imx708_set_gain_index(dev, *(const uint32_t *)arg);
        break;
    case ESP_CAM_SENSOR_ANGAIN:
        ret = imx708_set_analog_gain(dev, *(const uint32_t *)arg);
        break;
    case ESP_CAM_SENSOR_GROUP_EXP_GAIN: {
        /* The ISP prefers this: exposure and gain applied together, so a frame
           never lands with one updated and the other not. */
        const esp_cam_sensor_gh_exp_gain_t *g = (const esp_cam_sensor_gh_exp_gain_t *)arg;
        uint32_t lines;
        if (g->exposure_val != 0) {
            lines = g->exposure_val;
        } else if (g->exposure_us != 0) {
            lines = (uint32_t)(((uint64_t)g->exposure_us * 1000) / IMX708_TLINE_NS);
        } else {
            ret = ESP_ERR_INVALID_ARG;
            break;
        }
        ret = imx708_set_exposure(dev, lines);
        if (ret == ESP_OK) {
            ret = imx708_set_gain_index(dev, g->gain_index);
        }
        break;
    }
    case ESP_CAM_SENSOR_DGAIN:
        ret = imx708_set_digital_gain(dev, *(const uint32_t *)arg);
        break;
    default:
        ESP_LOGE(TAG, "set id=%" PRIx32 " not supported", id);
        ret = ESP_ERR_INVALID_ARG;
        break;
    }
    return ret;
}

static esp_err_t imx708_query_support_formats(esp_cam_sensor_device_t *dev, esp_cam_sensor_format_array_t *formats)
{
    ESP_CAM_SENSOR_NULL_POINTER_CHECK(TAG, dev);
    ESP_CAM_SENSOR_NULL_POINTER_CHECK(TAG, formats);
    formats->count = ARRAY_SIZE(imx708_format_info);
    formats->format_array = &imx708_format_info[0];
    return ESP_OK;
}

static esp_err_t imx708_query_support_capability(esp_cam_sensor_device_t *dev, esp_cam_sensor_capability_t *caps)
{
    ESP_CAM_SENSOR_NULL_POINTER_CHECK(TAG, dev);
    ESP_CAM_SENSOR_NULL_POINTER_CHECK(TAG, caps);
    caps->fmt_raw = 1;
    return ESP_OK;
}

/*
 * Point dev->cur_format at the per-device shadow of `format` rather than into
 * imx708_format_info[], so that imx708_apply_orientation() has a bayer_type it
 * is allowed to write. Callers hold this pointer indefinitely - esp_video
 * caches it in its own device state - so the shadow has to outlive them, which
 * it does by sitting in the device's own allocation.
 *
 * The static table stays the menu: imx708_format_by_index() and
 * query_support_formats() still hand out entries from it, and those entries
 * are still what a caller passes back in here.
 */
static esp_err_t imx708_select_format(esp_cam_sensor_device_t *dev, const esp_cam_sensor_format_t *format)
{
    imx708_para_t *para = (imx708_para_t *)dev->priv;

    if (para == NULL) {
        dev->cur_format = format;
        return ESP_ERR_INVALID_STATE;
    }

    para->format = *format;
    if (format->isp_info) {
        /*
         * memcpy, not assignment: isp_v1_info.version is declared const, which
         * makes the whole union non-assignable. The destination is inside the
         * device's calloc'd block, so it has no declared type of its own and
         * the copy is what gives it one.
         *
         * Skipped when the source is already the shadow. An application that
         * reads the current format back with VIDIOC_G_SENSOR_FMT and hands it
         * straight to VIDIOC_S_SENSOR_FMT - the way to make esp_video re-latch
         * the Bayer phase after a flip - arrives here with isp_info pointing at
         * the very object being written, and memcpy may not overlap.
         */
        if (format->isp_info != &para->isp_info) {
            memcpy(&para->isp_info, format->isp_info, sizeof(para->isp_info));
        }
        para->format.isp_info = &para->isp_info;
    }
    dev->cur_format = &para->format;
    return ESP_OK;
}

static esp_err_t imx708_set_format(esp_cam_sensor_device_t *dev, const esp_cam_sensor_format_t *format)
{
    ESP_CAM_SENSOR_NULL_POINTER_CHECK(TAG, dev);
    esp_err_t ret = ESP_OK;

    if (format == NULL) {
        format = &imx708_format_info[IMX708_DEFAULT_FORMAT_INDEX];
    }

    /* common -> mode -> link freq, the order the sensor is brought up in. The
       mode table carries the rest of the PLL block, so the link multiplier has
       to land after it or the mode's own PLL values fight it. */
    ret = imx708_write_array(dev->sccb_handle, imx708_common_regs);
    ESP_RETURN_ON_FALSE(ret == ESP_OK, ret, TAG, "write common regs failed");
    ret = imx708_write_array(dev->sccb_handle, (const imx708_reginfo_t *)format->regs);
    ESP_RETURN_ON_FALSE(ret == ESP_OK, ret, TAG, "write mode regs failed");
    ret = imx708_write_array(dev->sccb_handle, imx708_link_450mhz_regs);
    ESP_RETURN_ON_FALSE(ret == ESP_OK, ret, TAG, "write link regs failed");

    /* Binned modes don't re-mosaic, so the quad-Bayer LPF stays off. */
    ret = imx708_write(dev->sccb_handle, IMX708_REG_LPF_INTENSITY_EN, IMX708_LPF_INTENSITY_DISABLED);
    ESP_RETURN_ON_FALSE(ret == ESP_OK, ret, TAG, "disable quad-bayer LPF failed");

    ret = imx708_select_format(dev, format);
    ESP_RETURN_ON_FALSE(ret == ESP_OK, ret, TAG, "no device state to shadow the format into");

    /*
     * The mode tables leave 0x0101 alone, but a hardware reset clears it and
     * the shadow was just rebuilt from the const table, so its bayer_type is
     * back to the unflipped phase. Re-assert both from the cached state.
     */
    ret = imx708_apply_orientation(dev);
    ESP_RETURN_ON_FALSE(ret == ESP_OK, ret, TAG, "apply orientation failed");

    imx708_para_t *para = (imx708_para_t *)dev->priv;
    para->exposure_val = format->isp_info->isp_v1_info.exp_def;
    para->gain_index = 0;                    /* mode table writes min gain */

    ESP_LOGI(TAG, "set format: %s", format->name);
    return ret;
}

static esp_err_t imx708_get_format(esp_cam_sensor_device_t *dev, esp_cam_sensor_format_t *format)
{
    ESP_CAM_SENSOR_NULL_POINTER_CHECK(TAG, dev);
    ESP_CAM_SENSOR_NULL_POINTER_CHECK(TAG, format);
    if (dev->cur_format == NULL) {
        return ESP_FAIL;
    }
    memcpy(format, dev->cur_format, sizeof(esp_cam_sensor_format_t));
    return ESP_OK;
}

static esp_err_t imx708_priv_ioctl(esp_cam_sensor_device_t *dev, uint32_t cmd, void *arg)
{
    ESP_CAM_SENSOR_NULL_POINTER_CHECK(TAG, dev);
    esp_err_t ret = ESP_OK;
    uint8_t regval = 0;
    esp_cam_sensor_reg_val_t *sensor_reg;

    switch (cmd) {
    case ESP_CAM_SENSOR_IOC_HW_RESET:
        ret = imx708_hw_reset(dev);
        break;
    case ESP_CAM_SENSOR_IOC_S_STREAM:
        ret = imx708_set_stream(dev, *(int *)arg);
        break;
    case ESP_CAM_SENSOR_IOC_S_TEST_PATTERN:
        ret = imx708_set_test_pattern(dev, *(int *)arg);
        break;
    case ESP_CAM_SENSOR_IOC_S_REG:
        sensor_reg = (esp_cam_sensor_reg_val_t *)arg;
        ret = imx708_write(dev->sccb_handle, sensor_reg->regaddr, sensor_reg->value);
        break;
    case ESP_CAM_SENSOR_IOC_G_REG:
        sensor_reg = (esp_cam_sensor_reg_val_t *)arg;
        ret = imx708_read(dev->sccb_handle, sensor_reg->regaddr, &regval);
        if (ret == ESP_OK) {
            sensor_reg->value = regval;
        }
        break;
    case ESP_CAM_SENSOR_IOC_G_CHIP_ID:
        ret = imx708_get_sensor_id(dev, (esp_cam_sensor_id_t *)arg);
        break;
    default:
        ret = ESP_ERR_INVALID_ARG;
        break;
    }
    return ret;
}

static esp_err_t imx708_power_on(esp_cam_sensor_device_t *dev)
{
    esp_err_t ret = ESP_OK;

    if (dev->pwdn_pin >= 0) {
        gpio_config_t conf = { 0 };
        conf.pin_bit_mask = 1LL << dev->pwdn_pin;
        conf.mode = GPIO_MODE_OUTPUT;
        ret = gpio_config(&conf);
        ESP_RETURN_ON_FALSE(ret == ESP_OK, ret, TAG, "pwdn pin config failed");
        gpio_set_level(dev->pwdn_pin, 1);
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
        delay_ms(10);
    }

    return ret;
}

static esp_err_t imx708_power_off(esp_cam_sensor_device_t *dev)
{
    if (dev->reset_pin >= 0) {
        gpio_set_level(dev->reset_pin, 0);
    }
    if (dev->pwdn_pin >= 0) {
        gpio_set_level(dev->pwdn_pin, 0);
    }
    return ESP_OK;
}

static esp_err_t imx708_delete(esp_cam_sensor_device_t *dev)
{
    ESP_LOGD(TAG, "del imx708 (%p)", dev);
    if (dev) {
        free(dev);
    }
    return ESP_OK;
}

static const esp_cam_sensor_ops_t imx708_ops = {
    .query_para_desc = imx708_query_para_desc,
    .get_para_value = imx708_get_para_value,
    .set_para_value = imx708_set_para_value,
    .query_support_formats = imx708_query_support_formats,
    .query_support_capability = imx708_query_support_capability,
    .set_format = imx708_set_format,
    .get_format = imx708_get_format,
    .priv_ioctl = imx708_priv_ioctl,
    .del = imx708_delete,
};

esp_cam_sensor_device_t *imx708_detect(esp_cam_sensor_config_t *config)
{
    if (config == NULL) {
        return NULL;
    }

    esp_cam_sensor_device_t *dev = calloc(1, sizeof(esp_cam_sensor_device_t) + sizeof(imx708_para_t));
    if (dev == NULL) {
        ESP_LOGE(TAG, "no memory for camera device");
        return NULL;
    }
    dev->priv = (uint8_t *)dev + sizeof(esp_cam_sensor_device_t);

    dev->name = (char *)IMX708_SENSOR_NAME;
    dev->sccb_handle = config->sccb_handle;
    dev->xclk_pin = config->xclk_pin;
    dev->reset_pin = config->reset_pin;
    dev->pwdn_pin = config->pwdn_pin;
    dev->sensor_port = config->sensor_port;
    dev->ops = &imx708_ops;
    /* Shadow the default mode straight away: esp_video reads bayer_type out of
       cur_format at device open, which can happen without a set_format call. */
    imx708_select_format(dev, &imx708_format_info[IMX708_DEFAULT_FORMAT_INDEX]);

    if (config->sensor_port != ESP_CAM_SENSOR_MIPI_CSI) {
        ESP_LOGE(TAG, "only MIPI-CSI is supported");
        goto err;
    }

    if (imx708_power_on(dev) != ESP_OK) {
        ESP_LOGE(TAG, "power on failed");
        goto err;
    }

    if (imx708_get_sensor_id(dev, &dev->id) != ESP_OK) {
        ESP_LOGE(TAG, "get chip id failed");
        goto err;
    }
    if (dev->id.pid != IMX708_CHIP_ID) {
        ESP_LOGE(TAG, "sensor is not IMX708, PID=0x%04x", dev->id.pid);
        goto err;
    }
    ESP_LOGI(TAG, "detected IMX708, PID=0x%04x", dev->id.pid);


    return dev;

err:
    imx708_power_off(dev);
    free(dev);
    return NULL;
}

#if CONFIG_CAMERA_IMX708_AUTO_DETECT_MIPI_INTERFACE_SENSOR
ESP_CAM_SENSOR_DETECT_FN(imx708_detect, ESP_CAM_SENSOR_MIPI_CSI, IMX708_SCCB_ADDR)
{
    ((esp_cam_sensor_config_t *)config)->sensor_port = ESP_CAM_SENSOR_MIPI_CSI;
    return imx708_detect(config);
}
#endif
