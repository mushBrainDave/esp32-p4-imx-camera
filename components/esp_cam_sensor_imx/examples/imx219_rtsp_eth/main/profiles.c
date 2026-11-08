// SPDX-License-Identifier: Apache-2.0
#include <stdio.h>
#include <string.h>
#include <stdarg.h>
#include "freertos/FreeRTOS.h"
#include "freertos/semphr.h"
#include "nvs.h"
#include "nvs_flash.h"
#include "profiles.h"
static const stream_profile_t profiles[] = {
    {"detail15",   8, 1632, 1232, 15, 1, 4000000, 15,   8,   0, 3264, 2464, 0, 0, 0, 0, 0, 20, 45, 3522},
    {"720p30",     3, 1280,  720, 30, 1, 3000000, 30, 360, 512, 2560, 1440, 0, 0, 0, 0, 0, 20, 45, 1759},
    {"balanced30", 4, 1536,  864, 30, 1, 4000000, 30, 104, 368, 3072, 1728, 0, 0, 0, 0, 0, 20, 45, 1759},
    {"fullview15", 7, 1280, 960, 15, 1, 4000000, 15, 0, 0, 3280, 2464, 1, 1280, 960, 0, 0, 20, 45, 3522},
    {"fullview30", 0, 832, 624, 30, 1, 2000000, 30, 0, 0, 3280, 2464, 1, 832, 624, 0, 0, 20, 45, 1759},
    {"ultra1", 5, 1664, 1248, 1, 1, 12000000, 1, 0, 0, 3280, 2464, 2, 1640, 1232, 12, 8, 8, 8, 52896},
    {"ultra5", 6, 1664, 1248, 5, 1, 12000000, 1, 0, 0, 3280, 2464, 2, 1640, 1232, 12, 8, 8, 8, 10576},
};
static const stream_profile_t *requested = &profiles[0], *active;
static SemaphoreHandle_t lock;
static nvs_handle_t storage;
static char last_error[80];
const stream_profile_t *profiles_find(const char *id)
{
    if (!id) return NULL;
    for (unsigned i = 0; i < sizeof(profiles)/sizeof(profiles[0]); i++)
        if (!strcmp(id, profiles[i].id)) return &profiles[i];
    return NULL;
}
void profiles_init(void)
{
    /* Preserve existing NVS on errors; no implicit erase of network/user data. */
    ESP_ERROR_CHECK(nvs_flash_init());
    ESP_ERROR_CHECK(nvs_open("camera_profiles", NVS_READWRITE, &storage));
    lock = xSemaphoreCreateMutex();
    configASSERT(lock);
    char id[24]; size_t len = sizeof(id);
    if (nvs_get_str(storage, "selected", id, &len) == ESP_OK) {
        const stream_profile_t *saved = profiles_find(id);
        if (saved) requested = saved;
    }
}
const stream_profile_t *profiles_requested(void)
{
    xSemaphoreTake(lock, portMAX_DELAY);
    const stream_profile_t *p = requested;
    xSemaphoreGive(lock);
    return p;
}
esp_err_t profiles_request(const char *id)
{
    const stream_profile_t *p = profiles_find(id);
    if (!p) return ESP_ERR_INVALID_ARG;
    xSemaphoreTake(lock, portMAX_DELAY);
    if (requested == p) { xSemaphoreGive(lock); return ESP_OK; }
    esp_err_t err = nvs_set_str(storage, "selected", p->id);
    if (err == ESP_OK) err = nvs_commit(storage);
    if (err == ESP_OK) { requested = p; last_error[0] = 0; }
    xSemaphoreGive(lock);
    return err;
}
void profiles_applied(const stream_profile_t *p)
{
    xSemaphoreTake(lock, portMAX_DELAY);
    active = p;
    xSemaphoreGive(lock);
}
void profiles_rollback(const char *reason)
{
    xSemaphoreTake(lock, portMAX_DELAY);
    requested = &profiles[0];
    active = NULL;
    snprintf(last_error, sizeof(last_error), "%s", reason);
    ESP_ERROR_CHECK(nvs_set_str(storage, "selected", requested->id));
    ESP_ERROR_CHECK(nvs_commit(storage));
    xSemaphoreGive(lock);
}
static bool append_json(char *buffer, unsigned size, unsigned *used, const char *fmt, ...)
{
    if (*used >= size) return false;
    va_list ap;
    va_start(ap, fmt);
    int n = vsnprintf(buffer + *used, size - *used, fmt, ap);
    va_end(ap);
    if (n < 0 || (unsigned)n >= size - *used) return false;
    *used += (unsigned)n;
    return true;
}

esp_err_t profiles_json(char *buffer, unsigned size)
{
    unsigned used = 0;
    xSemaphoreTake(lock, portMAX_DELAY);
    bool ok = append_json(buffer, size, &used,
        "{\"requested\":\"%s\",\"active\":\"%s\",\"switching\":%s,\"last_error\":\"%s\",\"profiles\":[",
        requested->id, active ? active->id : "", active == requested ? "false" : "true", last_error);
    for (unsigned i = 0; ok && i < sizeof(profiles)/sizeof(profiles[0]); ++i) {
        const stream_profile_t *p = &profiles[i];
        ok = append_json(buffer, size, &used,
            "%s{\"id\":\"%s\",\"width\":%u,\"height\":%u,\"sensor_fps\":%u,\"fps\":%u,"
            "\"bitrate\":%u,\"gop\":%u,\"exposure_max_lines\":%u,\"qp_min\":%u,\"qp_max\":%u,\"rate_control\":\"%s\","
            "\"crop\":{\"x\":%u,\"y\":%u,\"width\":%u,\"height\":%u,\"binning\":2},"
            "\"full_sensor_view\":%s,\"processing\":\"%s\","
            "\"content\":{\"x\":%u,\"y\":%u,\"width\":%u,\"height\":%u}}",
            i ? "," : "", p->id, p->width, p->height, p->fps * p->divisor, p->fps,
            p->bitrate, p->gop, p->exposure_max_lines, p->qp_min, p->qp_max,
            p->qp_min == p->qp_max ? "fixed_qp" : "bitrate", p->crop_x, p->crop_y, p->crop_width, p->crop_height,
            p->fullview_resize ? "true" : "false", p->fullview_resize == 2 ? "native_pad" : p->fullview_resize ? "cpu_nearest_2x2" : "sensor_crop",
            p->content_x, p->content_y, p->fullview_resize ? p->content_width : p->width,
            p->fullview_resize ? p->content_height : p->height);
    }
    ok = ok && append_json(buffer, size, &used, "]}");
    xSemaphoreGive(lock);
    return ok ? ESP_OK : ESP_ERR_NO_MEM;
}
