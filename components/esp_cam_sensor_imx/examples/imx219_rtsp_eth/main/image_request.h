// SPDX-License-Identifier: Apache-2.0
#pragma once
#include <stdbool.h>
#include <stdint.h>
typedef struct { bool manual; uint32_t exposure_us, gain_milli; } image_request_t;
bool image_request_parse(const char *query, image_request_t *request);
