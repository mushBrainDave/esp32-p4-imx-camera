// SPDX-License-Identifier: Apache-2.0
#pragma once
#include <stdbool.h>
#include <stdint.h>
typedef struct {
    unsigned in_stride, out_stride, out_pairs;
    uint16_t x[640], y[480];
} fullview_scale_t;
bool fullview_scale_config(fullview_scale_t *s, unsigned iw, unsigned ih, unsigned ow, unsigned oh);
void fullview_scale_rows(const fullview_scale_t *s, const uint8_t *src, uint8_t *dst,
                        unsigned first, unsigned end);
