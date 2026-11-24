// SPDX-License-Identifier: Apache-2.0
#include <string.h>
#include "rtsp_wire.h"
bool rtsp_normalize_content_length(unsigned char *data, size_t len)
{
    static const char name[] = "\r\nContent-Length: ";
    if (len < 8 || memcmp(data, "RTSP/1.0", 8)) return false;
    for (size_t i = 0; i + sizeof(name) - 1 < len; i++) {
        if (i + 4 <= len && !memcmp(data + i, "\r\n\r\n", 4)) break;
        if (memcmp(data + i, name, sizeof(name) - 1)) continue;
        size_t start = i + sizeof(name) - 1, end = start;
        while (end < len && data[end] >= '0' && data[end] <= '9') end++;
        if (end == start) return false;
        size_t tail = end;
        while (tail < len && (data[tail] == ' ' || data[tail] == '\t')) tail++;
        if (tail == end || tail + 1 >= len || data[tail] != '\r' || data[tail + 1] != '\n') return false;
        /* 192[space] -> 0192: same decimal value, same byte count. */
        memmove(data + start + tail - end, data + start, end - start);
        memset(data + start, '0', tail - end);
        return true;
    }
    return false;
}
