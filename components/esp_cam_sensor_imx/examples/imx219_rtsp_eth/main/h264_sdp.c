// SPDX-License-Identifier: Apache-2.0
#include <stdio.h>
#include <string.h>
#include "h264_sdp.h"

static size_t find_bytes(const uint8_t *data, size_t length, const char *text)
{
    size_t size = strlen(text);
    for (size_t i = 0; i <= length && size <= length - i; ++i)
        if (!memcmp(data + i, text, size)) return i;
    return SIZE_MAX;
}
bool h264_params_ready(const h264_params_t *params)
{
    return params->sps_len >= 4 && params->sps_len <= H264_PARAMETER_BYTES &&
           params->pps_len >= 2 && params->pps_len <= H264_PARAMETER_BYTES;
}
static bool start_code(const uint8_t *data, size_t length, size_t at, size_t *after)
{
    if (at + 3 <= length && !data[at] && !data[at+1]) {
        if (data[at+2] == 1) { *after = at+3; return true; }
        if (at+4 <= length && !data[at+2] && data[at+3] == 1) {
            *after = at+4; return true;
        }
    }
    return false;
}
bool h264_params_ingest(h264_params_t *params, const uint8_t *data, size_t length)
{
    size_t at = 0;
    while (at < length) {
        size_t begin;
        if (!start_code(data, length, at, &begin)) { ++at; continue; }
        if (begin == length) break;
        unsigned type = data[begin] & 31;
        /* Encoder parameters precede the first slice. Never scan an entire
         * multi-megabyte slice while holding the stream mutex. */
        if (type == 1 || type == 5) break;
        size_t end = begin+1, next = 0;
        while (end < length && end-begin <= H264_PARAMETER_BYTES &&
               !start_code(data, length, end, &next)) ++end;
        if (end-begin > H264_PARAMETER_BYTES) break;
        size_t bytes = end-begin;
        if (type == 7 && bytes >= 4) {
            memcpy(params->sps, data+begin, bytes); params->sps_len = bytes;
        } else if (type == 8 && bytes >= 2) {
            memcpy(params->pps, data+begin, bytes); params->pps_len = bytes;
        }
        at = end;
    }
    return h264_params_ready(params);
}
static size_t base64(const uint8_t *data, size_t length, char *output)
{
    static const char alphabet[] = "ABCDEFGHIJKLMNOPQRSTUVWXYZabcdefghijklmnopqrstuvwxyz0123456789+/";
    size_t out = 0;
    for (size_t i = 0; i < length; i += 3) {
        uint32_t value = (uint32_t)data[i] << 16;
        if (i+1 < length) value |= (uint32_t)data[i+1] << 8;
        if (i+2 < length) value |= data[i+2];
        output[out++] = alphabet[value >> 18];
        output[out++] = alphabet[(value >> 12) & 63];
        output[out++] = i+1 < length ? alphabet[(value >> 6) & 63] : '=';
        output[out++] = i+2 < length ? alphabet[value & 63] : '=';
    }
    output[out] = 0;
    return out;
}
bool h264_sdp_response(const uint8_t *input, size_t length, const h264_params_t *params,
                       uint8_t *output, size_t capacity, size_t *written)
{
    if (!h264_params_ready(params) || length < 15 || memcmp(input, "RTSP/1.0 200 OK", 15)) return false;
    size_t header = find_bytes(input, length, "\r\n\r\n");
    if (header == SIZE_MAX) return false;
    size_t body = header+4;
    if (find_bytes(input, header, "Content-Type: application/sdp") == SIZE_MAX) return false;
    const char key[] = "\r\nContent-Length: ";
    size_t field = find_bytes(input, header, key);
    if (field == SIZE_MAX) return false;
    size_t digits = field+sizeof(key)-1, end = digits, declared = 0;
    while (end < header && input[end] >= '0' && input[end] <= '9') {
        if (declared > SIZE_MAX/10-10) return false;
        declared = declared*10+input[end++]-'0';
    }
    if (end == digits || declared != length-body) return false;
    while (end < header && (input[end] == ' ' || input[end] == '\t')) ++end;
    if (end+2 > length || memcmp(input+end, "\r\n", 2)) return false;
    size_t fmtp = find_bytes(input+body, length-body, "a=fmtp:");
    if (fmtp == SIZE_MAX || find_bytes(input+body, length-body, "sprop-parameter-sets=") != SIZE_MAX)
        return false;
    fmtp += body;
    size_t tail = find_bytes(input+fmtp, length-fmtp, "\r\n");
    if (tail == SIZE_MAX) return false;
    size_t insert = fmtp+tail;
    char sps[4*((H264_PARAMETER_BYTES+2)/3)+1], pps[sizeof(sps)], attributes[800];
    base64(params->sps, params->sps_len, sps);
    base64(params->pps, params->pps_len, pps);
    bool profile = find_bytes(input+fmtp, insert-fmtp, "profile-level-id=") != SIZE_MAX;
    int count = profile ? snprintf(attributes, sizeof(attributes), "%ssprop-parameter-sets=%s,%s;",
                                   input[insert-1] == ';' ? "" : ";", sps, pps)
                        : snprintf(attributes, sizeof(attributes),
                                   "%sprofile-level-id=%02x%02x%02x;sprop-parameter-sets=%s,%s;",
                                   input[insert-1] == ';' ? "" : ";",
                                   params->sps[1], params->sps[2], params->sps[3], sps, pps);
    if (count < 0 || (size_t)count >= sizeof(attributes)) return false;
    char size[32];
    int size_count = snprintf(size, sizeof(size), "%zu", length-body+(size_t)count);
    if (size_count <= 0 || (size_t)size_count >= sizeof(size)) return false;
    size_t total = digits+(size_t)size_count+(length-end)+(size_t)count;
    if (total > capacity) return false;
    size_t offset = 0;
#define APPEND(pointer, bytes) do { size_t n = (bytes); memcpy(output+offset, (pointer), n); offset += n; } while (0)
    APPEND(input, digits);
    APPEND(size, (size_t)size_count);
    APPEND(input+end, insert-end);
    APPEND(attributes, (size_t)count);
    APPEND(input+insert, length-insert);
#undef APPEND
    *written = offset;
    return true;
}
