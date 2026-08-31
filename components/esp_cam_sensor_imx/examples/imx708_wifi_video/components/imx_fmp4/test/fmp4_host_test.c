/*
 * SPDX-FileCopyrightText: 2026 esp_cam_sensor_imx contributors
 *
 * SPDX-License-Identifier: Apache-2.0
 *
 * Host harness for components/imx_fmp4: feed it a real Annex-B stream off the
 * board and write the fragmented MP4 the firmware would have streamed, so the
 * container can be judged by ffprobe before anything is flashed.
 *
 * The muxer is ordinary C with no ESP-IDF in it, which is what makes this
 * possible - and worth doing, because a muxing bug and a transport bug look
 * identical from the far end of a WiFi link. Run it against a clip
 * imx708_video already recorded:
 *
 *   gcc -O2 -Icomponents/imx_fmp4/include \
 *       components/imx_fmp4/test/fmp4_host_test.c \
 *       components/imx_fmp4/imx_fmp4.c -o fmp4_host_test
 *   ./fmp4_host_test clip/imx708.h264 frag.mp4
 *   ffmpeg -v error -i frag.mp4 -f null -     # silence means every frame decoded
 *
 * Frames here are timed at a nominal 1000/FPS ms rather than from real capture
 * timestamps, which the firmware does have and does use. That only affects the
 * timeline, not whether the boxes are right.
 */
#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#include <stdint.h>
#include <stdbool.h>

#include "imx_fmp4.h"

#define FPS         28
#define TIMESCALE   1000

static long find_sc(const uint8_t *d, size_t n, size_t from)
{
    for (size_t i = from; i + 2 < n; i++) {
        if (d[i] == 0 && d[i + 1] == 0 && d[i + 2] == 1) {
            return (long)i;
        }
    }
    return -1;
}

int main(int argc, char **argv)
{
    if (argc < 3) {
        fprintf(stderr, "usage: %s in.h264 out.mp4\n", argv[0]);
        return 2;
    }
    FILE *f = fopen(argv[1], "rb");
    if (!f) {
        perror(argv[1]);
        return 1;
    }
    fseek(f, 0, SEEK_END);
    long n = ftell(f);
    fseek(f, 0, SEEK_SET);
    uint8_t *d = malloc(n);
    if (fread(d, 1, n, f) != (size_t)n) {
        return 1;
    }
    fclose(f);

    FILE *o = fopen(argv[2], "wb");
    if (!o) {
        perror(argv[2]);
        return 1;
    }

    /*
     * Walk access units the way the firmware does: an access unit is the run of
     * NALs from one coded slice up to (not including) the next. Start codes are
     * kept, because imx_fmp4_fragment() takes Annex-B.
     */
    uint8_t *out = malloc(4 * 1024 * 1024);
    bool init_written = false;
    uint32_t seq = 1;
    uint64_t dts = 0;
    size_t frames = 0, total = 0;

    long au_start = -1;
    bool au_has_slice = false;
    long pos = 0;

    while (1) {
        long sc = find_sc(d, n, pos);
        long nal_start = sc < 0 ? -1 : sc + 3;
        uint8_t t = nal_start >= 0 ? (d[nal_start] & 0x1f) : 0;

        bool is_slice = (t == 1 || t == 5);
        bool flush = (nal_start < 0) || (is_slice && au_has_slice);

        if (flush && au_start >= 0) {
            long au_end = (nal_start < 0) ? n : sc;
            /* Back over a four-byte start code belonging to the next AU. */
            while (au_end > au_start && d[au_end - 1] == 0) {
                au_end--;
            }
            size_t au_len = au_end - au_start;

            if (!init_written) {
                const uint8_t *sps, *pps;
                size_t sps_len, pps_len;
                if (imx_fmp4_find_param_sets(d + au_start, au_len, &sps, &sps_len, &pps, &pps_len)) {
                    char codec[16];
                    imx_fmp4_codec_string(sps, sps_len, codec, sizeof(codec));
                    imx_fmp4_cfg_t cfg = {
                        .width = 1920, .height = 1072, .timescale = TIMESCALE,
                        .sps = sps, .sps_len = sps_len, .pps = pps, .pps_len = pps_len,
                    };
                    uint8_t init[IMX_FMP4_INIT_MAX];
                    size_t init_len = imx_fmp4_init_segment(&cfg, init, sizeof(init));
                    if (!init_len) {
                        fprintf(stderr, "init segment failed\n");
                        return 1;
                    }
                    printf("codec %s, sps %zu B, pps %zu B, init segment %zu B\n",
                           codec, sps_len, pps_len, init_len);
                    fwrite(init, 1, init_len, o);
                    total += init_len;
                    init_written = true;
                }
            }

            if (init_written) {
                uint32_t dur = TIMESCALE / FPS;
                size_t len = imx_fmp4_fragment(d + au_start, au_len, seq, dts, dur,
                                               out, 4 * 1024 * 1024);
                if (!len) {
                    fprintf(stderr, "fragment %u failed (au_len %zu)\n", seq, au_len);
                    return 1;
                }
                fwrite(out, 1, len, o);
                total += len;
                dts += dur;
                seq++;
                frames++;
            }
            au_start = -1;
            au_has_slice = false;
        }

        if (nal_start < 0) {
            break;
        }
        if (au_start < 0) {
            au_start = sc;
        }
        if (is_slice) {
            au_has_slice = true;
        }
        pos = nal_start;
    }

    fclose(o);
    printf("%zu fragments, %zu bytes written\n", frames, total);
    return 0;
}
