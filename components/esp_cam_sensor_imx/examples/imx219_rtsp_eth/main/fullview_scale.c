// SPDX-License-Identifier: Apache-2.0
#include <stddef.h>
#include "fullview_scale.h"

bool fullview_scale_config(fullview_scale_t *s, unsigned iw, unsigned ih, unsigned ow, unsigned oh)
{
    if (!s || iw < 4 || ih < 4 || ow < 4 || oh < 4 || iw > 1640 || ih > 1232 ||
        ow > 1280 || oh > 960 || ow > iw || oh > ih || (iw | ih | ow | oh) & 1) return false;
    s->in_stride = iw * 3 / 2;
    s->out_stride = ow * 3 / 2;
    s->out_pairs = oh / 2;
    /* Nearest 2x2 groups; endpoints include the first and last sensor groups.
     * Keep both luma rows and their U/V samples together in OUYY/EVYY layout.
     */
    for (unsigned x = 0; x < ow / 2; ++x)
        s->x[x] = ((x * (iw / 2 - 1) + (ow / 2 - 1) / 2) / (ow / 2 - 1)) * 3;
    for (unsigned y = 0; y < oh / 2; ++y)
        s->y[y] = (y * (ih / 2 - 1) + (oh / 2 - 1) / 2) / (oh / 2 - 1);
    return true;
}
void fullview_scale_rows(const fullview_scale_t *s, const uint8_t *src, uint8_t *dst,
                        unsigned first, unsigned end)
{
    for (unsigned y = first; y < end; ++y) {
        const uint8_t *a = src + (size_t)s->y[y] * 2 * s->in_stride;
        const uint8_t *b = a + s->in_stride;
        uint8_t *c = dst + (size_t)y * 2 * s->out_stride;
        uint8_t *d = c + s->out_stride;
        for (unsigned x = 0, off = 0; off < s->out_stride; ++x, off += 3) {
            unsigned from = s->x[x];
            c[off] = a[from]; c[off + 1] = a[from + 1]; c[off + 2] = a[from + 2];
            d[off] = b[from]; d[off + 1] = b[from + 1]; d[off + 2] = b[from + 2];
        }
    }
}
