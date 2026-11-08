// SPDX-License-Identifier: Apache-2.0
#include <assert.h>
#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#include "fullview_scale.h"
static void check(unsigned ow, unsigned oh)
{
    const unsigned iw = 1640, ih = 1232;
    const size_t in_len = iw * ih * 3 / 2, out_len = ow * oh * 3 / 2;
    uint8_t *input = malloc(in_len), *output = malloc(out_len + 32);
    assert(input && output);
    for (size_t i = 0; i < in_len; ++i) input[i] = (uint8_t)(i * 37 + i / 123);
    memset(output, 0xa5, out_len + 32);
    fullview_scale_t s;
    assert(fullview_scale_config(&s, iw, ih, ow, oh));
    assert(s.x[0] == 0 && s.x[ow / 2 - 1] == (iw / 2 - 1) * 3);
    assert(s.y[0] == 0 && s.y[oh / 2 - 1] == ih / 2 - 1);
    // Exercise the same disjoint row ranges as the two CPU workers.
    fullview_scale_rows(&s, input, output + 16, s.out_pairs / 2, s.out_pairs);
    fullview_scale_rows(&s, input, output + 16, 0, s.out_pairs / 2);
    for (unsigned y = 0; y < oh / 2; ++y) {
        unsigned sy = (unsigned)((double)y * (ih / 2 - 1) / (oh / 2 - 1) + .5);
        for (unsigned x = 0; x < ow / 2; ++x) {
            unsigned sx = (unsigned)((double)x * (iw / 2 - 1) / (ow / 2 - 1) + .5);
            for (unsigned row = 0; row < 2; ++row)
                assert(!memcmp(output + 16 + (y * 2 + row) * ow * 3 / 2 + x * 3,
                               input + (sy * 2 + row) * iw * 3 / 2 + sx * 3, 3));
        }
    }
    for (unsigned i = 0; i < 16; ++i) assert(output[i] == 0xa5 && output[16 + out_len + i] == 0xa5);
    free(input); free(output);
}
int main(void)
{
    fullview_scale_t s;
    assert(!fullview_scale_config(&s, 1641, 1232, 1280, 960));
    assert(!fullview_scale_config(&s, 1640, 1232, 1280, 961));
    assert(!fullview_scale_config(&s, 1640, 1232, 1640, 1232));
    check(1280, 960); check(832, 624);
    puts("Full-view endpoints, luma/chroma groups, worker split and bounds passed");
}
