// SPDX-License-Identifier: Apache-2.0
#include <assert.h>
#include <stdio.h>
#include <string.h>
#include "image_request.h"
int main(void)
{
    image_request_t r;
    assert(image_request_parse("mode=auto", &r) && !r.manual);
    assert(image_request_parse("gain_milli=1000&mode=manual&exposure_us=10000", &r));
    assert(r.manual && r.exposure_us == 10000 && r.gain_milli == 1000);
    const char *bad[] = {"", "mode=", "mode=unknown", "mode=auto&gain_milli=1000",
        "mode=manual", "mode=manual&exposure_us=1", "mode=auto&mode=auto", "mode=auto&",
        "&mode=auto", "mode=auto&&", "mode=auto&x=y", "mode=auto=manual",
        "mode=manual&exposure_us=-1&gain_milli=1000", "mode=manual&exposure_us=+1&gain_milli=1000",
        "mode=manual&exposure_us=1.5&gain_milli=1000", "mode=manual&exposure_us=4294967296&gain_milli=1000",
        "mode=manual&exposure_us=1&gain_milli=", "mode=manual&exposure_us=1&gain_milli=1&gain_milli=2"};
    for (unsigned i = 0; i < sizeof(bad)/sizeof(bad[0]); ++i) assert(!image_request_parse(bad[i], &r));
    char huge[170]; memset(huge, 'x', sizeof(huge)); huge[169] = 0;
    assert(!image_request_parse(huge, &r));
    assert(!image_request_parse(NULL, &r) && !image_request_parse("mode=auto", NULL));
    puts("Image query validation passed (duplicates, malformed fields, overflow and size bounds)");
}
