/*
 * SPDX-FileCopyrightText: 2026 esp_cam_sensor_imx contributors
 *
 * SPDX-License-Identifier: Apache-2.0
 */
#include <stdio.h>
#include <string.h>

#include "imx_fmp4.h"

/* NAL unit types. 1 and 5 are the coded slices - one per frame here - and
 * everything else is metadata that either moves into the container (7, 8), is
 * dropped (9), or rides along inside the sample (6). */
#define NAL_SLICE   1
#define NAL_IDR     5
#define NAL_SPS     7
#define NAL_PPS     8
#define NAL_AUD     9

/* ---- Annex-B scanning --------------------------------------------------- */

typedef struct {
    const uint8_t *d;
    size_t n;
    size_t pos;
} nal_iter_t;

/* Start codes are three or four bytes (00 00 01, optionally preceded by an
 * extra 00). Scanning for the three-byte form and then trimming a trailing zero
 * off the previous NAL finds both without needing to know which was emitted. */
static long find_start_code(const uint8_t *d, size_t n, size_t from)
{
    if (n < 3) {
        return -1;
    }
    for (size_t i = from; i + 2 < n; i++) {
        if (d[i] == 0 && d[i + 1] == 0 && d[i + 2] == 1) {
            return (long)i;
        }
    }
    return -1;
}

static bool nal_next(nal_iter_t *it, const uint8_t **nal, size_t *nal_len)
{
    while (1) {
        long sc = find_start_code(it->d, it->n, it->pos);
        if (sc < 0) {
            it->pos = it->n;
            return false;
        }
        size_t s = (size_t)sc + 3;
        long next = find_start_code(it->d, it->n, s);
        size_t e = (next < 0) ? it->n : (size_t)next;
        it->pos = e;

        /* Back over the extra leading zero of a four-byte start code, and any
         * trailing_zero_8bits padding. */
        while (e > s && it->d[e - 1] == 0) {
            e--;
        }
        if (e > s) {
            *nal = it->d + s;
            *nal_len = e - s;
            return true;
        }
        if (next < 0) {
            return false;
        }
    }
}

bool imx_fmp4_find_param_sets(const uint8_t *annexb, size_t len,
                              const uint8_t **sps, size_t *sps_len,
                              const uint8_t **pps, size_t *pps_len)
{
    nal_iter_t it = { .d = annexb, .n = len };
    const uint8_t *nal;
    size_t nal_len;
    bool have_sps = false, have_pps = false;

    while (nal_next(&it, &nal, &nal_len)) {
        switch (nal[0] & 0x1f) {
        case NAL_SPS:
            *sps = nal;
            *sps_len = nal_len;
            have_sps = true;
            break;
        case NAL_PPS:
            *pps = nal;
            *pps_len = nal_len;
            have_pps = true;
            break;
        default:
            break;
        }
    }
    return have_sps && have_pps;
}

void imx_fmp4_codec_string(const uint8_t *sps, size_t sps_len, char *buf, size_t buf_len)
{
    if (sps_len < 4 || buf_len < 12) {
        if (buf_len) {
            buf[0] = '\0';
        }
        return;
    }
    snprintf(buf, buf_len, "avc1.%02X%02X%02X", sps[1], sps[2], sps[3]);
}

/* ---- A bounded big-endian writer ---------------------------------------- */
/*
 * Every field in MP4 is big-endian and most boxes carry a length that is only
 * known once their children are written, so the writer tracks an overflow flag
 * rather than returning an error from every call: one check at the end covers
 * the whole build, and a truncated buffer can never be mistaken for a short
 * box.
 */
typedef struct {
    uint8_t *p;
    size_t cap;
    size_t len;
    bool ovf;
} wr_t;

static void w_bytes(wr_t *w, const void *src, size_t n)
{
    if (w->ovf || w->len + n > w->cap) {
        w->ovf = true;
        return;
    }
    memcpy(w->p + w->len, src, n);
    w->len += n;
}

static void w_zero(wr_t *w, size_t n)
{
    if (w->ovf || w->len + n > w->cap) {
        w->ovf = true;
        return;
    }
    memset(w->p + w->len, 0, n);
    w->len += n;
}

static void w_u8(wr_t *w, uint8_t v)
{
    w_bytes(w, &v, 1);
}

static void w_u16(wr_t *w, uint16_t v)
{
    uint8_t b[2] = { (uint8_t)(v >> 8), (uint8_t)v };
    w_bytes(w, b, 2);
}

static void w_u32(wr_t *w, uint32_t v)
{
    uint8_t b[4] = { (uint8_t)(v >> 24), (uint8_t)(v >> 16), (uint8_t)(v >> 8), (uint8_t)v };
    w_bytes(w, b, 4);
}

static void w_u64(wr_t *w, uint64_t v)
{
    w_u32(w, (uint32_t)(v >> 32));
    w_u32(w, (uint32_t)v);
}

/* Returns where the size field went, for box_close to patch. */
static size_t box_open(wr_t *w, const char *type)
{
    size_t at = w->len;
    w_u32(w, 0);
    w_bytes(w, type, 4);
    return at;
}

static void box_close(wr_t *w, size_t at)
{
    if (w->ovf) {
        return;
    }
    uint32_t size = (uint32_t)(w->len - at);
    w->p[at + 0] = (uint8_t)(size >> 24);
    w->p[at + 1] = (uint8_t)(size >> 16);
    w->p[at + 2] = (uint8_t)(size >> 8);
    w->p[at + 3] = (uint8_t)size;
}

static void w_fullbox_flags(wr_t *w, uint8_t version, uint32_t flags)
{
    w_u8(w, version);
    w_u8(w, (uint8_t)(flags >> 16));
    w_u8(w, (uint8_t)(flags >> 8));
    w_u8(w, (uint8_t)flags);
}

static void w_unity_matrix(wr_t *w)
{
    static const uint32_t m[9] = { 0x00010000, 0, 0, 0, 0x00010000, 0, 0, 0, 0x40000000 };
    for (int i = 0; i < 9; i++) {
        w_u32(w, m[i]);
    }
}

/* ---- Init segment -------------------------------------------------------- */

static void w_avcc(wr_t *w, const imx_fmp4_cfg_t *cfg)
{
    /* AVCDecoderConfigurationRecord: the decoder's out-of-band setup, and the
     * reason SPS/PPS can be stripped from every frame afterwards. */
    size_t at = box_open(w, "avcC");
    w_u8(w, 1);                             /* configurationVersion */
    w_u8(w, cfg->sps[1]);                   /* AVCProfileIndication */
    w_u8(w, cfg->sps[2]);                   /* profile_compatibility */
    w_u8(w, cfg->sps[3]);                   /* AVCLevelIndication */
    w_u8(w, 0xff);                          /* 6 reserved bits + lengthSizeMinusOne = 3 */
    w_u8(w, 0xe1);                          /* 3 reserved bits + SPS count = 1 */
    w_u16(w, (uint16_t)cfg->sps_len);
    w_bytes(w, cfg->sps, cfg->sps_len);
    w_u8(w, 1);                             /* PPS count */
    w_u16(w, (uint16_t)cfg->pps_len);
    w_bytes(w, cfg->pps, cfg->pps_len);
    box_close(w, at);
}

static void w_avc1(wr_t *w, const imx_fmp4_cfg_t *cfg)
{
    static const char name[] = "esp32p4 h.264";
    size_t at = box_open(w, "avc1");
    w_zero(w, 6);                           /* reserved */
    w_u16(w, 1);                            /* data_reference_index */
    w_zero(w, 16);                          /* pre_defined / reserved */
    w_u16(w, (uint16_t)cfg->width);
    w_u16(w, (uint16_t)cfg->height);
    w_u32(w, 0x00480000);                   /* horizresolution, 72 dpi */
    w_u32(w, 0x00480000);                   /* vertresolution */
    w_u32(w, 0);                            /* reserved */
    w_u16(w, 1);                            /* frame_count */
    w_u8(w, (uint8_t)(sizeof(name) - 1));   /* compressorname: a Pascal string in 32 bytes */
    w_bytes(w, name, sizeof(name) - 1);
    w_zero(w, 31 - (sizeof(name) - 1));
    w_u16(w, 0x0018);                       /* depth */
    w_u16(w, 0xffff);                       /* pre_defined = -1 */
    w_avcc(w, cfg);
    box_close(w, at);
}

size_t imx_fmp4_init_segment(const imx_fmp4_cfg_t *cfg, uint8_t *out, size_t cap)
{
    if (!cfg || !cfg->sps || cfg->sps_len < 4 || !cfg->pps || !cfg->pps_len) {
        return 0;
    }

    wr_t w = { .p = out, .cap = cap };

    size_t ftyp = box_open(&w, "ftyp");
    w_bytes(&w, "isom", 4);                 /* major_brand */
    w_u32(&w, 0x200);                       /* minor_version */
    w_bytes(&w, "isomiso2avc1mp41", 16);    /* compatible_brands */
    box_close(&w, ftyp);

    size_t moov = box_open(&w, "moov");

    size_t mvhd = box_open(&w, "mvhd");
    w_fullbox_flags(&w, 0, 0);
    w_u32(&w, 0);                           /* creation_time */
    w_u32(&w, 0);                           /* modification_time */
    w_u32(&w, cfg->timescale);
    /* Duration 0: a live stream has no total to declare. A player reads it as
     * unknown and follows the fragments instead. */
    w_u32(&w, 0);
    w_u32(&w, 0x00010000);                  /* rate 1.0 */
    w_u16(&w, 0x0100);                      /* volume 1.0 */
    w_zero(&w, 10);                         /* reserved */
    w_unity_matrix(&w);
    w_zero(&w, 24);                         /* pre_defined */
    w_u32(&w, 2);                           /* next_track_ID */
    box_close(&w, mvhd);

    size_t trak = box_open(&w, "trak");

    size_t tkhd = box_open(&w, "tkhd");
    w_fullbox_flags(&w, 0, 7);              /* enabled | in movie | in preview */
    w_u32(&w, 0);                           /* creation_time */
    w_u32(&w, 0);                           /* modification_time */
    w_u32(&w, 1);                           /* track_ID */
    w_u32(&w, 0);                           /* reserved */
    w_u32(&w, 0);                           /* duration */
    w_zero(&w, 8);                          /* reserved */
    w_zero(&w, 8);                          /* layer, alternate_group, volume, reserved */
    w_unity_matrix(&w);
    w_u32(&w, cfg->width << 16);            /* 16.16 fixed point */
    w_u32(&w, cfg->height << 16);
    box_close(&w, tkhd);

    size_t mdia = box_open(&w, "mdia");

    size_t mdhd = box_open(&w, "mdhd");
    w_fullbox_flags(&w, 0, 0);
    w_u32(&w, 0);                           /* creation_time */
    w_u32(&w, 0);                           /* modification_time */
    w_u32(&w, cfg->timescale);
    w_u32(&w, 0);                           /* duration */
    w_u16(&w, 0x55c4);                      /* language 'und' */
    w_u16(&w, 0);                           /* pre_defined */
    box_close(&w, mdhd);

    size_t hdlr = box_open(&w, "hdlr");
    w_fullbox_flags(&w, 0, 0);
    w_u32(&w, 0);                           /* pre_defined */
    w_bytes(&w, "vide", 4);                 /* handler_type */
    w_zero(&w, 12);                         /* reserved */
    w_bytes(&w, "VideoHandler", 13);        /* name, NUL included */
    box_close(&w, hdlr);

    size_t minf = box_open(&w, "minf");

    size_t vmhd = box_open(&w, "vmhd");
    w_fullbox_flags(&w, 0, 1);
    w_zero(&w, 8);                          /* graphicsmode + opcolor */
    box_close(&w, vmhd);

    size_t dinf = box_open(&w, "dinf");
    size_t dref = box_open(&w, "dref");
    w_fullbox_flags(&w, 0, 0);
    w_u32(&w, 1);                           /* entry_count */
    size_t url = box_open(&w, "url ");
    w_fullbox_flags(&w, 0, 1);              /* flag 1: the media is in this file */
    box_close(&w, url);
    box_close(&w, dref);
    box_close(&w, dinf);

    /*
     * The sample table is present and empty, which is what makes this a
     * *fragmented* MP4: every box a player looks in for timing, sizes and
     * offsets declares zero entries, so there is nothing to read but the
     * fragments as they arrive.
     */
    size_t stbl = box_open(&w, "stbl");

    size_t stsd = box_open(&w, "stsd");
    w_fullbox_flags(&w, 0, 0);
    w_u32(&w, 1);                           /* entry_count */
    w_avc1(&w, cfg);
    box_close(&w, stsd);

    static const char *const empty[] = { "stts", "stsc", "stco" };
    for (size_t i = 0; i < sizeof(empty) / sizeof(empty[0]); i++) {
        size_t b = box_open(&w, empty[i]);
        w_fullbox_flags(&w, 0, 0);
        w_u32(&w, 0);                       /* entry_count */
        box_close(&w, b);
    }
    size_t stsz = box_open(&w, "stsz");
    w_fullbox_flags(&w, 0, 0);
    w_u32(&w, 0);                           /* sample_size, 0 = per-sample table */
    w_u32(&w, 0);                           /* sample_count */
    box_close(&w, stsz);

    box_close(&w, stbl);
    box_close(&w, minf);
    box_close(&w, mdia);
    box_close(&w, trak);

    /* mvex is what says fragments follow. Without it a player stops at the
     * empty sample table and reports a zero-length movie. */
    size_t mvex = box_open(&w, "mvex");
    size_t trex = box_open(&w, "trex");
    w_fullbox_flags(&w, 0, 0);
    w_u32(&w, 1);                           /* track_ID */
    w_u32(&w, 1);                           /* default_sample_description_index */
    w_u32(&w, 0);                           /* default_sample_duration */
    w_u32(&w, 0);                           /* default_sample_size */
    w_u32(&w, 0);                           /* default_sample_flags */
    box_close(&w, trex);
    box_close(&w, mvex);

    box_close(&w, moov);

    return w.ovf ? 0 : w.len;
}

/* ---- Fragments ----------------------------------------------------------- */

size_t imx_fmp4_fragment(const uint8_t *annexb, size_t len,
                         uint32_t seq, uint64_t decode_time, uint32_t duration,
                         uint8_t *out, size_t cap)
{
    /* First pass: how big the sample becomes once start codes turn into
     * lengths, and whether this access unit can be seeked to. The bitstream is
     * the authority on that, not the caller. */
    nal_iter_t it = { .d = annexb, .n = len };
    const uint8_t *nal;
    size_t nal_len;
    size_t sample_size = 0;
    bool is_idr = false, has_slice = false;

    while (nal_next(&it, &nal, &nal_len)) {
        uint8_t t = nal[0] & 0x1f;
        if (t == NAL_SPS || t == NAL_PPS || t == NAL_AUD) {
            continue;
        }
        if (t == NAL_IDR) {
            is_idr = true;
            has_slice = true;
        } else if (t == NAL_SLICE) {
            has_slice = true;
        }
        sample_size += 4 + nal_len;
    }
    if (!has_slice || sample_size == 0) {
        return 0;
    }

    wr_t w = { .p = out, .cap = cap };

    size_t moof = box_open(&w, "moof");

    size_t mfhd = box_open(&w, "mfhd");
    w_fullbox_flags(&w, 0, 0);
    w_u32(&w, seq);
    box_close(&w, mfhd);

    size_t traf = box_open(&w, "traf");

    size_t tfhd = box_open(&w, "tfhd");
    /* default-base-is-moof: sample offsets are relative to the start of this
     * moof, so a fragment means the same thing wherever it lands in the byte
     * stream. Any other base would need a file position this stream has not
     * got. */
    w_fullbox_flags(&w, 0, 0x020000);
    w_u32(&w, 1);                           /* track_ID */
    box_close(&w, tfhd);

    size_t tfdt = box_open(&w, "tfdt");
    w_fullbox_flags(&w, 1, 0);              /* version 1: 64-bit, so a long run cannot wrap */
    w_u64(&w, decode_time);
    box_close(&w, tfdt);

    size_t trun = box_open(&w, "trun");
    /* data-offset | sample-duration | sample-size | sample-flags, all present
     * for the one sample. */
    w_fullbox_flags(&w, 0, 0x000701);
    w_u32(&w, 1);                           /* sample_count */
    size_t data_offset_at = w.len;
    w_u32(&w, 0);                           /* data_offset, patched below */
    w_u32(&w, duration);
    w_u32(&w, (uint32_t)sample_size);
    /* 0x02000000 says "nothing depends on this and it is a sync sample";
     * 0x01010000 says the opposite, and is what stops a player seeking onto a
     * P frame and showing a smear. */
    w_u32(&w, is_idr ? 0x02000000u : 0x01010000u);
    box_close(&w, trun);

    box_close(&w, traf);
    box_close(&w, moof);

    /* data_offset counts from the first byte of the moof, and the sample starts
     * immediately after the mdat header. It cannot be written until the moof's
     * own size is known, which is why it is patched rather than computed. */
    if (!w.ovf) {
        uint32_t moof_size = (uint32_t)(w.len - moof);
        uint32_t data_offset = moof_size + 8;
        w.p[data_offset_at + 0] = (uint8_t)(data_offset >> 24);
        w.p[data_offset_at + 1] = (uint8_t)(data_offset >> 16);
        w.p[data_offset_at + 2] = (uint8_t)(data_offset >> 8);
        w.p[data_offset_at + 3] = (uint8_t)data_offset;
    }

    size_t mdat = box_open(&w, "mdat");
    it = (nal_iter_t){ .d = annexb, .n = len };
    while (nal_next(&it, &nal, &nal_len)) {
        uint8_t t = nal[0] & 0x1f;
        if (t == NAL_SPS || t == NAL_PPS || t == NAL_AUD) {
            continue;
        }
        w_u32(&w, (uint32_t)nal_len);
        w_bytes(&w, nal, nal_len);
    }
    box_close(&w, mdat);

    return w.ovf ? 0 : w.len;
}
