// SPDX-License-Identifier: Apache-2.0
#include <assert.h>
#include <string.h>
#include "rtsp_wire.h"
int main(void)
{
    unsigned char s[] = "RTSP/1.0 200 OK\r\nContent-Length: 192 \r\n\r\nbody";
    assert(rtsp_normalize_content_length(s, sizeof(s) - 1));
    assert(!strcmp((char *)s, "RTSP/1.0 200 OK\r\nContent-Length: 0192\r\n\r\nbody"));
    unsigned char valid[] = "RTSP/1.0 200 OK\r\nContent-Length: 192\r\n\r\n";
    assert(!rtsp_normalize_content_length(valid, sizeof(valid) - 1));
    unsigned char body[] = "RTSP/1.0 200 OK\r\n\r\nContent-Length: 192 \r\n";
    assert(!rtsp_normalize_content_length(body, sizeof(body) - 1));
    unsigned char truncated[] = "RTSP/1.0 200 OK\r\nContent-Length: 192 ";
    assert(!rtsp_normalize_content_length(truncated, sizeof(truncated) - 1));
    unsigned char malformed[] = "RTSP/1.0 200 OK\r\nContent-Length: x \r\n\r\n";
    assert(!rtsp_normalize_content_length(malformed, sizeof(malformed) - 1));
    unsigned char binary[] = "$RTSP/1.0 200 OK\r\nContent-Length: 192 \r\n";
    assert(!rtsp_normalize_content_length(binary, sizeof(binary) - 1));
    for (size_t n = 0; n < 20; n++) assert(!rtsp_normalize_content_length(valid, n));
}
