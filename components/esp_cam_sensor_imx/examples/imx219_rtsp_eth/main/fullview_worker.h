// SPDX-License-Identifier: Apache-2.0
#pragma once
#include "fullview_scale.h"
void fullview_worker_init(void);
void fullview_resize(const fullview_scale_t *s, const uint8_t *src, uint8_t *dst);
