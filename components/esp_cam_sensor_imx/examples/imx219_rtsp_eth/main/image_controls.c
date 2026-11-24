// SPDX-License-Identifier: Apache-2.0
#include <stdio.h>
#include <string.h>
#include <inttypes.h>
#include <sys/ioctl.h>
#include "freertos/FreeRTOS.h"
#include "freertos/semphr.h"
#include "esp_log.h"
#include "esp_timer.h"
#include "esp_video_isp_pipeline.h"
#include "linux/videodev2.h"
#include "image_controls.h"
#define TLINE_NS 18904U
#define MAX_GAINS 64
static SemaphoreHandle_t lock;
static const stream_profile_t *current;
static bool capable, ready, active_manual, requested_manual, need_auto_restore;
static uint32_t generation, applied, wanted_us, wanted_gain;
static uint32_t min_us, max_us, model_max_us, gains[MAX_GAINS];
static unsigned gain_count;
static int actual_lines = -1, actual_index = -1, saved_auto_lines, saved_auto_index;
static int64_t sampled_us;
static char error[64];
static int get_ctrl(int fd, uint32_t id)
{
    struct v4l2_ext_control c = {.id = id};
    struct v4l2_ext_controls cs = {.ctrl_class = V4L2_CTRL_CLASS_USER, .count = 1, .controls = &c};
    return ioctl(fd, VIDIOC_G_EXT_CTRLS, &cs) == 0 ? c.value : -1;
}
static bool set_pair(int fd, int lines, int index)
{
    struct v4l2_ext_control c[] = {{.id = V4L2_CID_EXPOSURE, .value = lines},
                                 {.id = V4L2_CID_GAIN, .value = index}};
    struct v4l2_ext_controls cs = {.ctrl_class = V4L2_CTRL_CLASS_USER, .count = 2, .controls = c};
    return ioctl(fd, VIDIOC_S_EXT_CTRLS, &cs) == 0;
}
void image_controls_init(void)
{
    lock = xSemaphoreCreateMutex();
    configASSERT(lock);
}
void image_controls_begin(const stream_profile_t *profile, int fd)
{
    xSemaphoreTake(lock, portMAX_DELAY);
    current = profile;
    ready = active_manual = requested_manual = need_auto_restore = false;
    generation = applied = wanted_us = wanted_gain = 0;
    actual_lines = actual_index = -1;
    sampled_us = 0;
    error[0] = 0;
    gain_count = model_max_us = 0;
    min_us = (4 * TLINE_NS + 999) / 1000;
    max_us = (uint64_t)profile->exposure_max_lines * TLINE_NS / 1000;
    struct v4l2_query_ext_ctrl q = {.id = V4L2_CID_GAIN};
    capable = ioctl(fd, VIDIOC_QUERY_EXT_CTRL, &q) == 0 && q.minimum == 0 &&
              q.maximum >= 0 && q.maximum < MAX_GAINS && q.type == V4L2_CTRL_TYPE_INTEGER_MENU;
    if (capable) {
        gain_count = (unsigned)q.maximum + 1;
        for (unsigned i = 0; i < gain_count; ++i) {
            struct v4l2_querymenu m = {.id = V4L2_CID_GAIN, .index = i};
            if (ioctl(fd, VIDIOC_QUERYMENU, &m) || m.value < 1 || m.value > UINT32_MAX) {
                capable = false; break;
            }
            gains[i] = (uint32_t)m.value;
        }
    }
    esp_video_isp_pipeline_agc_status_t agc;
    if (esp_video_isp_pipeline_get_agc_status(&agc) != ESP_OK ||
        agc != ESP_VIDEO_ISP_PIPELINE_AGC_ENABLE ||
        esp_video_isp_pipeline_get_agc_max_exposure(&model_max_us) != ESP_OK) capable = false;
    if (!capable) snprintf(error, sizeof(error), "image_controls_unavailable");
    ESP_LOGI("image_controls", "%s: sensor max=%" PRIu32 " us, AGC model max=%" PRIu32 " us, capable=%d",
             profile->id, max_us, model_max_us, capable);
    xSemaphoreGive(lock);
}
void image_controls_set_ready(bool enabled)
{
    xSemaphoreTake(lock, portMAX_DELAY);
    ready = enabled && capable;
    xSemaphoreGive(lock);
}
esp_err_t image_controls_request(const image_request_t *r)
{
    if (!r) return ESP_ERR_INVALID_ARG;
    xSemaphoreTake(lock, portMAX_DELAY);
    esp_err_t err = ESP_OK;
    if (!ready || current != profiles_requested() || (error[0] && r->manual)) err = ESP_ERR_INVALID_STATE;
    else if (r->manual && (r->exposure_us < min_us || r->exposure_us > max_us ||
             r->gain_milli < gains[0] || r->gain_milli > gains[gain_count - 1])) err = ESP_ERR_INVALID_ARG;
    else {
        requested_manual = r->manual;
        wanted_us = r->manual ? r->exposure_us : 0;
        wanted_gain = r->manual ? r->gain_milli : 0;
        generation++;
        error[0] = 0;
    }
    xSemaphoreGive(lock);
    return err;
}
void image_controls_apply(int fd)
{
    xSemaphoreTake(lock, portMAX_DELAY);
    if (!ready || generation == applied) { xSemaphoreGive(lock); return; }
    if (requested_manual) {
        esp_err_t err = esp_video_isp_pipeline_set_agc_status(ESP_VIDEO_ISP_PIPELINE_AGC_DISABLE);
        int old_lines = get_ctrl(fd, V4L2_CID_EXPOSURE), old_index = get_ctrl(fd, V4L2_CID_GAIN);
        if (err != ESP_OK || old_lines < 0 || old_index < 0) {
            if (!active_manual) esp_video_isp_pipeline_set_agc_status(ESP_VIDEO_ISP_PIPELINE_AGC_ENABLE);
            snprintf(error, sizeof(error), "manual_control_prepare_failed");
        } else {
            unsigned index = 0;
            uint32_t distance = UINT32_MAX;
            for (unsigned i = 0; i < gain_count; ++i) {
                uint32_t d = gains[i] > wanted_gain ? gains[i] - wanted_gain : wanted_gain - gains[i];
                if (d < distance) { distance = d; index = i; }
            }
            unsigned lines = ((uint64_t)wanted_us * 1000 + TLINE_NS / 2) / TLINE_NS;
            if (!active_manual && !need_auto_restore) {
                saved_auto_lines = old_lines; saved_auto_index = old_index;
                need_auto_restore = true;
            }
            if (set_pair(fd, lines, index)) {
                active_manual = true;
            } else {
                bool restored = set_pair(fd, old_lines, old_index);
                if (!active_manual && restored &&
                    esp_video_isp_pipeline_set_agc_status(ESP_VIDEO_ISP_PIPELINE_AGC_ENABLE) == ESP_OK)
                    need_auto_restore = false;
                snprintf(error, sizeof(error), restored ? "manual_control_apply_failed" : "manual_control_rollback_failed");
            }
        }
    } else {
        /* The ISP caches its last auto settings. Restore them before re-enabling
         * AGC so its model and the sensor begin auto mode from the same state.
         */
        if ((!need_auto_restore || set_pair(fd, saved_auto_lines, saved_auto_index)) &&
            esp_video_isp_pipeline_set_agc_status(ESP_VIDEO_ISP_PIPELINE_AGC_ENABLE) == ESP_OK)
            active_manual = need_auto_restore = false;
        else snprintf(error, sizeof(error), "auto_control_restore_failed");
    }
    applied = generation;
    xSemaphoreGive(lock);
}
void image_controls_sample(int lines, int index)
{
    xSemaphoreTake(lock, portMAX_DELAY);
    actual_lines = lines; actual_index = index;
    sampled_us = esp_timer_get_time();
    xSemaphoreGive(lock);
}
esp_err_t image_controls_json(char *buffer, unsigned size)
{
    xSemaphoreTake(lock, portMAX_DELAY);
    bool available = ready && current == profiles_requested();
    int n = snprintf(buffer, size,
        "{\"ready\":%s,\"profile\":\"%s\",\"requested_mode\":\"%s\",\"active_mode\":\"%s\","
        "\"pending\":%s,\"last_error\":\"%s\",\"scope\":\"until_profile_change_or_reboot\","
        "\"min_exposure_us\":%" PRIu32 ",\"max_exposure_us\":%" PRIu32 ",\"agc_model_max_exposure_us\":%" PRIu32 ","
        "\"requested_exposure_us\":%" PRIu32 ",\"requested_gain_milli\":%" PRIu32 ","
        "\"applied_exposure_lines\":%d,\"applied_exposure_us\":%d,\"applied_gain_milli\":%" PRIu32 ","
        "\"sample_age_ms\":%" PRIi64 ",\"gain_values_milli\":[",
        available ? "true" : "false", current ? current->id : "", requested_manual ? "manual" : "auto",
        error[0] ? "error" : active_manual ? "manual" : "auto", generation != applied ? "true" : "false", error,
        min_us, max_us, model_max_us, wanted_us, wanted_gain, actual_lines,
        actual_lines < 0 ? -1 : actual_lines * (int)TLINE_NS / 1000,
        actual_index >= 0 && (unsigned)actual_index < gain_count ? gains[actual_index] : 0,
        sampled_us ? (esp_timer_get_time() - sampled_us) / 1000 : (int64_t)-1);
    bool ok = n >= 0 && (unsigned)n < size;
    unsigned used = ok ? (unsigned)n : size;
    for (unsigned i = 0; ok && i < gain_count; ++i) {
        n = snprintf(buffer + used, size - used, "%s%" PRIu32, i ? "," : "", gains[i]);
        ok = n >= 0 && (unsigned)n < size - used;
        if (ok) used += n;
    }
    if (ok) { n = snprintf(buffer + used, size - used, "]}"); ok = n >= 0 && (unsigned)n < size - used; }
    xSemaphoreGive(lock);
    return ok ? ESP_OK : ESP_ERR_NO_MEM;
}
