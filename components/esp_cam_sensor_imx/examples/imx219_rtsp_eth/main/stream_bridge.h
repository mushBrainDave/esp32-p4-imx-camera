// SPDX-License-Identifier: Apache-2.0
#pragma once
#include <stdbool.h>
#include <stdint.h>
#include "h264_sdp.h"
bool stream_h264_params(h264_params_t *params);
void stream_init(void);
void stream_set_ip(const char *ip);
void stream_start(int width, int height, int fps);
void stream_encoded(const uint8_t *data, unsigned len, uint32_t pts_ms,
                    bool idr, uint32_t encode_us, uint32_t scale_us, bool ok);
void stream_stats_json(char *buffer, unsigned size);
bool stream_pause(void);

void stream_image_sample(int exposure_lines, int gain_index, unsigned luma);

void stream_retire_connection(void);
