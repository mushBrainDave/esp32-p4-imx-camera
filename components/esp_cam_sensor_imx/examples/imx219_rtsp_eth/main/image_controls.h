// SPDX-License-Identifier: Apache-2.0
#pragma once
#include "profiles.h"
#include "image_request.h"
void image_controls_init(void);
void image_controls_begin(const stream_profile_t *profile, int fd);
void image_controls_set_ready(bool ready);
esp_err_t image_controls_request(const image_request_t *request);
void image_controls_apply(int fd);
void image_controls_sample(int exposure_lines, int gain_index);
esp_err_t image_controls_json(char *buffer, unsigned size);
