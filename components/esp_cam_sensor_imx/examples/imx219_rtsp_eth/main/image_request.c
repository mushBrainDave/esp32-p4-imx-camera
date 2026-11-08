// SPDX-License-Identifier: Apache-2.0
#include <string.h>
#include "image_request.h"
static bool decimal(const char *s, uint32_t *result)
{
    uint32_t n = 0;
    if (!*s) return false;
    for (; *s; ++s) {
        if (*s < '0' || *s > '9' || n > (UINT32_MAX - (unsigned)(*s - '0')) / 10) return false;
        n = n * 10 + (unsigned)(*s - '0');
    }
    *result = n;
    return true;
}
bool image_request_parse(const char *query, image_request_t *request)
{
    char copy[160];
    if (!query || !request || !*query || strlen(query) >= sizeof(copy)) return false;
    strcpy(copy, query);
    image_request_t result = {0};
    unsigned seen = 0;
    char *p = copy;
    while (true) {
        char *next = strchr(p, '&');
        if (next) *next++ = 0;
        char *value = strchr(p, '=');
        if (!value) return false;
        *value++ = 0;
        unsigned bit;
        if (!strcmp(p, "mode")) {
            bit = 1;
            if (!strcmp(value, "manual")) result.manual = true;
            else if (strcmp(value, "auto")) return false;
        } else if (!strcmp(p, "exposure_us")) {
            bit = 2;
            if (!decimal(value, &result.exposure_us)) return false;
        } else if (!strcmp(p, "gain_milli")) {
            bit = 4;
            if (!decimal(value, &result.gain_milli)) return false;
        } else return false;
        if (seen & bit) return false;
        seen |= bit;
        if (!next) break;
        if (!*next) return false;
        p = next;
    }
    if (seen != (result.manual ? 7U : 1U)) return false;
    *request = result;
    return true;
}
