// SPDX-License-Identifier: Apache-2.0
#pragma once
#include <stdbool.h>
#include <stddef.h>
/* Correct the pinned RTSP library's decimal-header whitespace without changing length. */
bool rtsp_normalize_content_length(unsigned char *data, size_t len);
