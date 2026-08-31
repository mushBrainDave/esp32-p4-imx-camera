/*
 * SPDX-FileCopyrightText: 2026 esp_cam_sensor_imx contributors
 *
 * SPDX-License-Identifier: Apache-2.0
 *
 * Fragmented MP4, built on the board, one frame at a time.
 *
 * The P4's H.264 encoder emits an Annex-B elementary stream: NAL units with
 * 00 00 01 start codes and nothing else. That is exactly what you want for a
 * stream you are concatenating frame by frame - it carries its own
 * synchronisation and can be cut anywhere - but no browser will play it. A
 * <video> element wants a container, and the only container a browser will
 * accept a *live* byte stream in is fragmented MP4 through Media Source
 * Extensions.
 *
 * fMP4 splits an ordinary MP4 in two. An init segment (ftyp + moov) declares
 * the codec, the geometry and the timescale but indexes nothing; after it, each
 * frame is a self-contained fragment (moof + mdat) carrying its own timing.
 * Nothing has to be known in advance, nothing has to be patched afterwards, and
 * the stream has no end - which is the whole difference from tools/mp4.py,
 * whose stco table of absolute file offsets can only be written once the
 * recording is complete.
 *
 * This is deliberately the same job as tools/mp4.py, in C, on the sending side.
 * Muxing already-encoded H.264 is bookkeeping - nothing here parses a slice or
 * touches a pixel - and the box layouts are mirrored from that file so the two
 * can be read against each other.
 */
#pragma once

#include <stdbool.h>
#include <stddef.h>
#include <stdint.h>

#ifdef __cplusplus
extern "C" {
#endif

/** Bytes an init segment can take. The real one is ~700; this is the cap a
 *  caller needs to allocate, not a measurement. */
#define IMX_FMP4_INIT_MAX       1024

/** Bytes a fragment adds in front of the frame data: a moof (100) plus an mdat
 *  header (8), plus 4 per NAL for the AVCC length prefixes that replace the
 *  Annex-B start codes. 256 covers any frame this encoder produces. */
#define IMX_FMP4_FRAG_OVERHEAD  256

typedef struct {
    uint32_t width;
    uint32_t height;
    uint32_t timescale;         /*!< Ticks per second for all timing. 1000 = milliseconds. */
    const uint8_t *sps;         /*!< SPS NAL, start code stripped, header byte included. */
    size_t sps_len;
    const uint8_t *pps;         /*!< PPS NAL, same form. */
    size_t pps_len;
} imx_fmp4_cfg_t;

/**
 * @brief Find the SPS and PPS inside an Annex-B access unit.
 *
 * The encoder prepends both to every IDR frame, which is what makes a stream
 * joinable partway through - but in MP4 they belong in the avcC box of the init
 * segment, not in the samples. Point a config at them once, from the first IDR,
 * and let imx_fmp4_fragment() drop them from every frame afterwards.
 *
 * The pointers returned are into @p annexb, so they are only valid while that
 * buffer is.
 *
 * @return true if both were found.
 */
bool imx_fmp4_find_param_sets(const uint8_t *annexb, size_t len,
                              const uint8_t **sps, size_t *sps_len,
                              const uint8_t **pps, size_t *pps_len);

/**
 * @brief The MSE codec string for an SPS, e.g. "avc1.42C01F".
 *
 * The three hex bytes are profile_idc, the constraint-set flags and level_idc,
 * read straight out of the SPS. MediaSource.isTypeSupported() is checked
 * against this string before any bytes are appended, so it has to describe what
 * the encoder actually produced rather than what it was asked for.
 *
 * @param buf  At least 16 bytes. Set to "" if the SPS is too short to read.
 */
void imx_fmp4_codec_string(const uint8_t *sps, size_t sps_len, char *buf, size_t buf_len);

/**
 * @brief Build the init segment: ftyp + moov, with an empty sample table.
 *
 * Sent once, before any fragment, and re-sent to every client that connects.
 *
 * @return Bytes written, or 0 if @p cap was too small (see IMX_FMP4_INIT_MAX).
 */
size_t imx_fmp4_init_segment(const imx_fmp4_cfg_t *cfg, uint8_t *out, size_t cap);

/**
 * @brief Wrap one Annex-B access unit as a moof + mdat fragment.
 *
 * SPS, PPS and access-unit delimiters are dropped - the first two are already
 * in the init segment - and every remaining NAL's start code is replaced by a
 * 4-byte big-endian length, which is how MP4 stores them. Whether the frame is
 * a sync sample is decided here, by looking for an IDR slice, rather than being
 * passed in: the bitstream is the authority on that.
 *
 * @param seq          Fragment sequence number, from 1, incrementing. A
 *                     decoder uses it only to notice a gap.
 * @param decode_time  Presentation time of this frame in timescale ticks,
 *                     absolute since the stream began. This, not the durations,
 *                     is what places the frame on the timeline.
 * @param duration     How long this frame is shown, in timescale ticks.
 *
 * @return Bytes written, or 0 if the unit held no coded slice or @p cap was too
 *         small.
 */
size_t imx_fmp4_fragment(const uint8_t *annexb, size_t len,
                         uint32_t seq, uint64_t decode_time, uint32_t duration,
                         uint8_t *out, size_t cap);

#ifdef __cplusplus
}
#endif
