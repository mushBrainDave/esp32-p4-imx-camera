// SPDX-License-Identifier: Apache-2.0
#pragma once
#include <stdbool.h>
#include <stddef.h>
#include <stdint.h>
#define H264_PARAMETER_BYTES 256
typedef struct {
    uint8_t sps[H264_PARAMETER_BYTES], pps[H264_PARAMETER_BYTES];
    size_t sps_len, pps_len;
} h264_params_t;
bool h264_params_ready(const h264_params_t *params);
bool h264_params_ingest(h264_params_t *params, const uint8_t *data, size_t length);
/* Rewrites a complete pinned-library SDP response with cached parameter sets.
 * Output contains no trailing NUL; *written is the complete wire length. */
bool h264_sdp_response(const uint8_t *input, size_t length, const h264_params_t *params,
                       uint8_t *output, size_t capacity, size_t *written);
