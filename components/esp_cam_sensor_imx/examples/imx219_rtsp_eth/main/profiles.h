// SPDX-License-Identifier: Apache-2.0
#pragma once
#include <stdbool.h>
#include <stdint.h>
#include "esp_err.h"
typedef struct {
    const char *id;
    unsigned sensor_mode, width, height, fps, divisor, bitrate, gop;
    unsigned crop_x, crop_y, crop_width, crop_height;
    unsigned fullview_resize, content_width, content_height, content_x, content_y;
    unsigned qp_min, qp_max, exposure_max_lines;
} stream_profile_t;
void profiles_init(void);
const stream_profile_t *profiles_requested(void);
const stream_profile_t *profiles_find(const char *id);
esp_err_t profiles_request(const char *id);
void profiles_applied(const stream_profile_t *profile);
void profiles_rollback(const char *reason);
esp_err_t profiles_json(char *buffer, unsigned size);
