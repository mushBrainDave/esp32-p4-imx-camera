/*
 * SPDX-FileCopyrightText: 2026 esp_cam_sensor_imx contributors
 *
 * SPDX-License-Identifier: Apache-2.0
 *
 * Dongwoon DW9807 VCM autofocus actuator - the lens driver on the Raspberry Pi
 * Camera Module 3 (IMX708), at I2C 0x0c.
 *
 * Structured against the esp_cam_sensor "motor" contract, which esp_video wires
 * up like this:
 *
 *   esp_ipa AF unit  --focus_pos-->  esp_video ISP pipeline
 *        ^                                    |
 *        |                            V4L2_CID_FOCUS_ABSOLUTE
 *   ISP AF statistics                         v
 *   (definition per window)  <---  this driver  --->  DW9807 DAC
 *
 * The AF algorithm is closed-loop contrast detection: it moves the lens, reads
 * back the ISP's "definition" (edge energy) statistic for the AF windows, and
 * hill-climbs. The windows, scan range and restart thresholds are configured in
 * the sensor's IPA JSON under "af"; everything below is just the actuator.
 *
 * PDAF is not involved - the IMX708's phase-detect pixels are not exposed
 * through this path.
 */
#include <string.h>
#include <inttypes.h>
#include <freertos/FreeRTOS.h>
#include <freertos/task.h>

#include "driver/gpio.h"
#include "esp_err.h"
#include "esp_check.h"
#include "esp_log.h"
#include "esp_rom_sys.h"
#include "esp_timer.h"
#include "esp_sccb_intf.h"
#include "esp_cam_motor.h"
#include "esp_cam_motor_detect.h"
#include "dw9807_settings.h"
#include "dw9807.h"

static const char *TAG = "dw9807";

/*
 * Position is reported to callers as the full electrical range of the 10-bit DAC.
 *
 * Only part of that range corresponds to real focus distances: on the Camera
 * Module 3 the lens reaches infinity somewhere around code 420 and its closest
 * macro distance around code 920 (those figures come from libcamera's imx708
 * tuning file, which maps dioptres to DAC codes). Codes below the infinity
 * point just push the lens against its rest stop and codes above the macro
 * point push it against the other one - both focus-wise useless but
 * electrically legal.
 *
 * That useful sub-range belongs in the IPA JSON ("af": min_pos / max_pos), not
 * here, so a scan does not spend its points outside it. This driver keeps
 * reporting the honest hardware range so a calibration sweep can still visit
 * the whole travel and find where the real limits are on your module.
 */
#define DW9807_MIN_POS          0
#define DW9807_MAX_POS          DW9807_MAX_DAC_CODE

#define DW9807_INIT_POS         CONFIG_CAM_MOTOR_DW9807_INIT_POS
#define DW9807_PERIOD_IN_US     CONFIG_CAM_MOTOR_DW9807_PERIOD_US

/*
 * Retracting the lens in one jump makes it hit the rest stop hard - audible as
 * a click, and not something to do on every power-down. Step it back instead.
 * The step is coarse because a FreeRTOS tick is 10 ms at the default 100 Hz, so
 * a fine ramp would cost most of a second in the teardown path.
 */
#define DW9807_RETRACT_STEP     128

static esp_err_t dw9807_write_reg(esp_sccb_io_handle_t sccb_handle, uint8_t reg, uint8_t val)
{
    return esp_sccb_transmit_reg_a8v8(sccb_handle, reg, val);
}

static esp_err_t dw9807_read_reg(esp_sccb_io_handle_t sccb_handle, uint8_t reg, uint8_t *val)
{
    return esp_sccb_transmit_receive_reg_a8v8(sccb_handle, reg, val);
}

/*
 * Wait for the chip to finish applying the previous DAC write. The datasheet
 * requires this: a write issued while busy is acknowledged on the bus but
 * discarded, so skipping the poll shows up as a lens that ignores some moves
 * rather than as an I2C error.
 */
static esp_err_t dw9807_wait_idle(esp_cam_motor_device_t *dev)
{
    uint8_t status = 0;

    for (int i = 0; i < DW9807_BUSY_POLL_MAX; i++) {
        esp_err_t ret = dw9807_read_reg(dev->sccb_handle, DW9807_REG_STATUS, &status);
        if (ret != ESP_OK) {
            return ret;
        }
        if (!(status & DW9807_STATUS_BUSY)) {
            return ESP_OK;
        }
        esp_rom_delay_us(DW9807_BUSY_POLL_US);
    }

    ESP_LOGW(TAG, "still busy after %d polls (status=0x%02x)", DW9807_BUSY_POLL_MAX, status);
    return ESP_ERR_TIMEOUT;
}

/* Move the lens. pos is a raw DAC code, clamped to the electrical range. */
static esp_err_t dw9807_set_pos_code(esp_cam_motor_device_t *dev, int pos)
{
    esp_err_t ret;

    if (pos > DW9807_MAX_POS) {
        pos = DW9807_MAX_POS;
    } else if (pos < DW9807_MIN_POS) {
        pos = DW9807_MIN_POS;
    }

    ret = dw9807_wait_idle(dev);
    if (ret != ESP_OK) {
        return ret;
    }

    /*
     * One 3-byte transfer {0x03, code[9:8], code[7:0]} - the chip auto-increments
     * from MSB into LSB, so the 10-bit code lands atomically. esp_sccb's a8v16
     * helper emits exactly that byte sequence (address, then value big-endian).
     * Writing the two registers separately would step the lens through a bogus
     * intermediate position on every move that crosses a 256-code boundary.
     */
    ret = esp_sccb_transmit_reg_a8v16(dev->sccb_handle, DW9807_REG_MSB, (uint16_t)pos & 0x03ff);
    if (ret != ESP_OK) {
        ESP_LOGE(TAG, "failed to write DAC code %d", pos);
        return ESP_CAM_MOTOR_ERR_FAILED_SET_POS;
    }

    /*
     * The write has been accepted, but the lens has not arrived yet. Record when
     * the move started; the ISP pipeline reads this back through
     * V4L2_CID_MOTOR_START_TIME and combines it with the format's step period to
     * decide when the AF statistics for the new position are trustworthy.
     * Reading definition from a frame captured mid-flight is what makes a
     * contrast-AF search wander.
     */
    dev->current_position = pos;
    dev->moving_start_time = esp_timer_get_time();

    return ESP_OK;
}

static esp_err_t dw9807_active(esp_cam_motor_device_t *dev)
{
    esp_err_t ret = dw9807_write_reg(dev->sccb_handle, DW9807_REG_CTL, DW9807_CTL_ACTIVE);
    ESP_RETURN_ON_ERROR(ret, TAG, "failed to leave power-down");

    vTaskDelay(pdMS_TO_TICKS(DW9807_POWER_UP_TIME_MS));

    /* Coming out of power-down the DAC is zeroed, so restore where we were. */
    return dw9807_set_pos_code(dev, dev->current_position);
}

static esp_err_t dw9807_standby(esp_cam_motor_device_t *dev)
{
    int pos = dev->current_position;

    /* Walk the lens back towards the rest stop instead of dropping it there. */
    while (pos > DW9807_RETRACT_STEP) {
        pos -= DW9807_RETRACT_STEP;
        if (dw9807_set_pos_code(dev, pos) != ESP_OK) {
            break;
        }
        vTaskDelay(1);
    }

    return dw9807_write_reg(dev->sccb_handle, DW9807_REG_CTL, DW9807_CTL_POWER_DOWN);
}

/*
 * A single working format.
 *
 * The DW9807 can also run its SAC (smooth actuator control) ring compensation
 * via the MODE and RESONANCE registers, which trades settling time against
 * overshoot - but the right resonance value is a property of this particular
 * lens assembly and is not published for the Camera Module 3. Linux's
 * dw9807-vcm driver leaves both registers at their reset defaults, so the
 * direct-drive path below is the configuration with known-good behaviour.
 * regs is empty for that reason, not by omission.
 */
static const esp_cam_motor_format_t dw9807_format_info[] = {
    {
        .name = "DIRECT_mode",
        .mode = ESP_CAM_MOTOR_DIRECT_MODE,
        .step_period = {
            /*
             * Time the AF algorithm should allow per DAC code of travel. The
             * DW9807 applies the code immediately; what actually takes time is
             * the lens settling on its spring, which is closer to constant than
             * proportional - so this is a linear approximation of a non-linear
             * thing. The default sizes it so a coarse scan step settles within
             * about one frame; raise it if the AF result comes out inconsistent
             * between runs on the same scene.
             */
            .period_in_us = DW9807_PERIOD_IN_US,
            .codes_per_step = 1,
        },
        .init_position = DW9807_INIT_POS,
        .regs = NULL,
        .regs_size = 0,
        .reserved = NULL,
    },
};

static esp_err_t dw9807_query_para_desc(esp_cam_motor_device_t *dev, esp_cam_motor_param_desc_t *qdesc)
{
    esp_err_t ret = ESP_OK;

    switch (qdesc->id) {
    case ESP_CAM_MOTOR_POSITION_CODE:
        qdesc->type = ESP_CAM_SENSOR_PARAM_TYPE_NUMBER;
        qdesc->number.minimum = DW9807_MIN_POS;
        qdesc->number.maximum = DW9807_MAX_POS;
        qdesc->number.step = 1;
        qdesc->default_value = DW9807_INIT_POS;
        break;
    case ESP_CAM_MOTOR_MOVING_START_TIME:
        qdesc->type = ESP_CAM_SENSOR_PARAM_TYPE_U8;
        qdesc->u8.size = sizeof(int64_t);
        break;
    default:
        ESP_LOGD(TAG, "id=%" PRIx32 " is not supported", qdesc->id);
        ret = ESP_ERR_INVALID_ARG;
        break;
    }

    return ret;
}

static esp_err_t dw9807_get_para_value(esp_cam_motor_device_t *dev, uint32_t id, void *arg, size_t size)
{
    esp_err_t ret = ESP_OK;

    switch (id) {
    case ESP_CAM_MOTOR_POSITION_CODE:
        /*
         * esp_video hands us &v4l2_ext_control.value - an int32_t - for every
         * NUMBER-typed control. Write the whole width: filling only 16 bits
         * leaves the top half of the caller's int holding whatever was there.
         */
        ESP_RETURN_ON_FALSE(arg && size >= sizeof(int32_t), ESP_ERR_INVALID_ARG, TAG, "para size err");
        *(int32_t *)arg = dev->current_position;
        break;
    case ESP_CAM_MOTOR_MOVING_START_TIME:
        ESP_RETURN_ON_FALSE(arg && size == sizeof(int64_t), ESP_ERR_INVALID_ARG, TAG, "para size err");
        *(int64_t *)arg = dev->moving_start_time;
        break;
    default:
        ret = ESP_ERR_NOT_SUPPORTED;
        break;
    }

    return ret;
}

static esp_err_t dw9807_set_para_value(esp_cam_motor_device_t *dev, uint32_t id, const void *arg, size_t size)
{
    esp_err_t ret = ESP_OK;

    switch (id) {
    case ESP_CAM_MOTOR_POSITION_CODE:
        ESP_RETURN_ON_FALSE(arg && size >= sizeof(int32_t), ESP_ERR_INVALID_ARG, TAG, "para size err");
        ret = dw9807_set_pos_code(dev, *(const int32_t *)arg);
        break;
    default:
        ESP_LOGE(TAG, "set id=%" PRIx32 " is not supported", id);
        ret = ESP_ERR_INVALID_ARG;
        break;
    }

    return ret;
}

static esp_err_t dw9807_query_support_formats(esp_cam_motor_device_t *dev, esp_cam_motor_fmt_array_t *formats)
{
    ESP_CAM_SENSOR_NULL_POINTER_CHECK(TAG, dev);
    ESP_CAM_SENSOR_NULL_POINTER_CHECK(TAG, formats);

    formats->count = ARRAY_SIZE(dw9807_format_info);
    formats->fmt_array = &dw9807_format_info[0];

    return ESP_OK;
}

static esp_err_t dw9807_set_format(esp_cam_motor_device_t *dev, const esp_cam_motor_format_t *format)
{
    ESP_CAM_SENSOR_NULL_POINTER_CHECK(TAG, dev);

    if (format == NULL) {
        format = &dw9807_format_info[0];
    }

    const uint8_t *regs = format->regs;
    for (int i = 0; i + 1 < format->regs_size; i += 2) {
        if (dw9807_write_reg(dev->sccb_handle, regs[i], regs[i + 1]) != ESP_OK) {
            ESP_LOGE(TAG, "failed to write format regs");
            return ESP_CAM_MOTOR_ERR_FAILED_SET_FORMAT;
        }
    }

    dev->cur_format = format;

    /*
     * Park the lens at the format's init position rather than just recording it.
     * Out of reset the DAC is at 0, which holds the lens hard against its rest
     * stop - past infinity, and out of focus at every distance. Frames captured
     * before the AF loop converges come from wherever the lens actually is, so
     * it should start somewhere usable.
     */
    if (dw9807_set_pos_code(dev, format->init_position) != ESP_OK) {
        ESP_LOGE(TAG, "failed to move to init position %d", format->init_position);
        return ESP_CAM_MOTOR_ERR_FAILED_SET_FORMAT;
    }

    return ESP_OK;
}

static esp_err_t dw9807_get_format(esp_cam_motor_device_t *dev, esp_cam_motor_format_t *format)
{
    ESP_CAM_SENSOR_NULL_POINTER_CHECK(TAG, dev);
    ESP_CAM_SENSOR_NULL_POINTER_CHECK(TAG, format);

    if (dev->cur_format == NULL) {
        return ESP_FAIL;
    }
    memcpy(format, dev->cur_format, sizeof(esp_cam_motor_format_t));

    return ESP_OK;
}

static esp_err_t dw9807_hw_power_on(esp_cam_motor_device_t *dev, bool en)
{
    /*
     * The Pi 15-pin CSI connector routes no power-down line to the host, so on
     * this board pwdn_pin is -1 and the VCM is simply always powered. Kept for
     * boards that do wire it.
     */
    if (dev->pwdn_pin >= 0) {
        gpio_config_t conf = {
            .pin_bit_mask = 1ULL << dev->pwdn_pin,
            .mode = GPIO_MODE_OUTPUT,
        };
        gpio_config(&conf);
        gpio_set_level(dev->pwdn_pin, en ? 1 : 0);
        vTaskDelay(pdMS_TO_TICKS(DW9807_POWER_UP_TIME_MS));
    }

    return ESP_OK;
}

static esp_err_t dw9807_priv_ioctl(esp_cam_motor_device_t *dev, uint32_t cmd, void *arg)
{
    ESP_CAM_SENSOR_NULL_POINTER_CHECK(TAG, dev);
    ESP_CAM_SENSOR_NULL_POINTER_CHECK(TAG, arg);

    switch (cmd) {
    case ESP_CAM_MOTOR_IOC_HW_POWER_ON:
        return dw9807_hw_power_on(dev, *(int *)arg);
    case ESP_CAM_MOTOR_IOC_SW_STANDBY:
        return *(int *)arg ? dw9807_standby(dev) : dw9807_active(dev);
    case ESP_CAM_MOTOR_IOC_S_REG: {
        esp_cam_motor_reg_val_t *rv = (esp_cam_motor_reg_val_t *)arg;
        return dw9807_write_reg(dev->sccb_handle, (uint8_t)rv->regaddr, (uint8_t)rv->value);
    }
    case ESP_CAM_MOTOR_IOC_G_REG: {
        esp_cam_motor_reg_val_t *rv = (esp_cam_motor_reg_val_t *)arg;
        uint8_t val = 0;
        esp_err_t ret = dw9807_read_reg(dev->sccb_handle, (uint8_t)rv->regaddr, &val);
        rv->value = val;
        return ret;
    }
    default:
        return ESP_ERR_INVALID_ARG;
    }
}

static esp_err_t dw9807_delete(esp_cam_motor_device_t *dev)
{
    ESP_LOGD(TAG, "del dw9807 (%p)", dev);
    if (dev) {
        /*
         * Free only - do NOT touch the bus here. It is tempting to retract the
         * lens on the way out, but esp_video's deinit path deletes the motor's
         * SCCB io *before* calling this (esp_video_init.c: esp_sccb_del_i2c_io
         * then esp_cam_motor_del_dev), so dev->sccb_handle is already freed by
         * the time we are called. Anything that transfers here is a
         * use-after-free on every esp_video_deinit().
         *
         * To park the lens gracefully, move it while the device is still alive:
         * either V4L2_CID_FOCUS_ABSOLUTE from the app, or
         * ESP_CAM_MOTOR_IOC_SW_STANDBY, which ramps it down and powers the coil
         * off. GPIO power-down is likewise left to that ioctl, since it pairs
         * with the standby write.
         */
        free(dev);
    }

    return ESP_OK;
}

static const esp_cam_motor_ops_t dw9807_ops = {
    .query_para_desc = dw9807_query_para_desc,
    .get_para_value = dw9807_get_para_value,
    .set_para_value = dw9807_set_para_value,
    .query_support_formats = dw9807_query_support_formats,
    .set_format = dw9807_set_format,
    .get_format = dw9807_get_format,
    .priv_ioctl = dw9807_priv_ioctl,
    .del = dw9807_delete,
};

esp_cam_motor_device_t *dw9807_detect(esp_cam_motor_config_t *config)
{
    uint8_t status = 0;
    esp_cam_motor_device_t *dev;

    if (config == NULL) {
        return NULL;
    }

    dev = calloc(1, sizeof(esp_cam_motor_device_t));
    if (dev == NULL) {
        ESP_LOGE(TAG, "no memory for motor");
        return NULL;
    }

    dev->name = (char *)TAG;
    dev->sccb_handle = config->sccb_handle;
    dev->reset_pin = config->reset_pin;
    dev->pwdn_pin = config->pwdn_pin;
    dev->signal_pin = config->signal_pin;
    dev->ops = &dw9807_ops;

    if (dw9807_hw_power_on(dev, true) != ESP_OK) {
        ESP_LOGE(TAG, "motor power on failed");
        goto err;
    }

    /*
     * The DW9807 has no chip-ID register, so presence is established by the bus
     * itself: bringing CTL out of power-down has to be acknowledged. If nothing
     * is at 0x0c the transfer NACKs and we bail out here.
     */
    if (dw9807_write_reg(dev->sccb_handle, DW9807_REG_CTL, DW9807_CTL_ACTIVE) != ESP_OK) {
        ESP_LOGD(TAG, "no device answered at 0x%02x", DW9807_SCCB_ADDR);
        goto err;
    }
    vTaskDelay(pdMS_TO_TICKS(DW9807_POWER_UP_TIME_MS));

    /*
     * Weak second check, to catch a *different* chip that happens to live at
     * 0x0c: an idle DW9807 reads status as 0x00, and its reserved bits are
     * always clear. Logged rather than silently assumed, because if this ever
     * trips on real hardware the raw value is the thing worth knowing.
     */
    if (dw9807_read_reg(dev->sccb_handle, DW9807_REG_STATUS, &status) != ESP_OK) {
        ESP_LOGE(TAG, "device at 0x%02x will not answer a register read", DW9807_SCCB_ADDR);
        goto err;
    }
    if (status & DW9807_STATUS_RESERVED) {
        ESP_LOGE(TAG, "device at 0x%02x is not a DW9807 (status=0x%02x, reserved bits set)",
                 DW9807_SCCB_ADDR, status);
        goto err;
    }

    if (dw9807_set_format(dev, NULL) != ESP_OK) {
        ESP_LOGE(TAG, "failed to set default format");
        goto err;
    }

    ESP_LOGI(TAG, "detected DW9807 VCM at 0x%02x, lens parked at code %d (range %d..%d)",
             DW9807_SCCB_ADDR, dev->current_position, DW9807_MIN_POS, DW9807_MAX_POS);

    return dev;

err:
    dw9807_hw_power_on(dev, false);
    free(dev);

    return NULL;
}

#if CONFIG_CAM_MOTOR_DW9807_AUTO_DETECT
ESP_CAM_MOTOR_DETECT_FN(dw9807_detect, NULL, DW9807_SCCB_ADDR)
{
    return dw9807_detect(config);
}
#endif
