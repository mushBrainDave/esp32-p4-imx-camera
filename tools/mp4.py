#!/usr/bin/env python3
"""Wrap a raw H.264 Annex-B stream in an MP4 container.

The board sends an Annex-B elementary stream - NAL units separated by 00 00 01
start codes - because that is what the ESP32-P4's encoder produces and it is
the only form in which a stream can be concatenated frame by frame. It is not a
file format: it carries no frame rate, no duration and no index, so most players
either refuse it or guess. Muxing it into MP4 turns it into something you can
double-click.

This is deliberately a self-contained implementation rather than a call out to
ffmpeg. Muxing already-encoded H.264 into MP4 is bookkeeping, not signal
processing - nothing here re-encodes or even parses a slice - and the whole job
is a few hundred lines of struct.pack, which is a much smaller thing to depend
on than a media toolchain being installed and on PATH.

    python tools/mp4.py clip.h264 clip.mp4 --fps 28

Frame timings can come from the capture log instead of a nominal frame rate:
capture.py passes the board's per-frame timestamps through, so the result plays
at the speed the camera actually delivered.
"""
import argparse
import struct
import sys

# NAL unit types we care about. 1 and 5 are the coded slices - one per frame
# here - and everything else is metadata that either moves into the container
# (7, 8) or rides along inside the sample (6).
NAL_SLICE = 1
NAL_IDR = 5
NAL_SEI = 6
NAL_SPS = 7
NAL_PPS = 8
NAL_AUD = 9


def split_nals(data):
    """Yield (start, end) byte ranges of each NAL unit payload, start codes stripped.

    Start codes are three or four bytes (00 00 01, optionally preceded by an
    extra 00). Scanning for the three-byte form and then backing over a leading
    zero finds both without needing to know which the encoder emitted.
    """
    n = len(data)
    starts = []
    i = 0
    while True:
        i = data.find(b'\x00\x00\x01', i)
        if i < 0:
            break
        starts.append(i + 3)
        i += 3
    for k, s in enumerate(starts):
        e = n if k + 1 == len(starts) else starts[k + 1] - 3
        # Trim the extra leading zero of a four-byte start code, and any
        # trailing_zero_8bits some encoders pad with.
        while e > s and data[e - 1] == 0:
            e -= 1
        if e > s:
            yield s, e


class _Bits:
    """Bit reader over RBSP, for the handful of SPS fields we need."""

    def __init__(self, data):
        self.d = data
        self.pos = 0

    def u(self, n):
        v = 0
        for _ in range(n):
            byte = self.d[self.pos >> 3]
            v = (v << 1) | ((byte >> (7 - (self.pos & 7))) & 1)
            self.pos += 1
        return v

    def ue(self):
        """Unsigned Exp-Golomb: count leading zeros, then read that many bits."""
        lz = 0
        while self.u(1) == 0:
            lz += 1
            if lz > 32:
                raise ValueError('malformed Exp-Golomb code')
        return (1 << lz) - 1 + (self.u(lz) if lz else 0)

    def se(self):
        k = self.ue()
        return (k + 1) // 2 if k % 2 else -(k // 2)


def _unescape(nal):
    """Strip emulation-prevention bytes: 00 00 03 -> 00 00.

    They exist so a NAL payload can never contain a start code. They have to go
    before the bit reader runs, or every field after the first one lands at the
    wrong offset.
    """
    out = bytearray()
    zeros = 0
    for b in nal:
        if zeros == 2 and b == 3:
            zeros = 0
            continue
        out.append(b)
        zeros = zeros + 1 if b == 0 else 0
    return bytes(out)


def parse_sps(sps):
    """Return (width, height) in displayed pixels from an SPS NAL (header byte included)."""
    b = _Bits(_unescape(sps[1:]))
    profile_idc = b.u(8)
    b.u(8)                      # constraint_set flags + reserved
    b.u(8)                      # level_idc
    b.ue()                      # seq_parameter_set_id

    chroma_format_idc = 1
    if profile_idc in (100, 110, 122, 244, 44, 83, 86, 118, 128, 138, 139, 134, 135):
        chroma_format_idc = b.ue()
        if chroma_format_idc == 3:
            b.u(1)              # separate_colour_plane_flag
        b.ue()                  # bit_depth_luma_minus8
        b.ue()                  # bit_depth_chroma_minus8
        b.u(1)                  # qpprime_y_zero_transform_bypass_flag
        if b.u(1):              # seq_scaling_matrix_present_flag
            for i in range(8 if chroma_format_idc != 3 else 12):
                if b.u(1):
                    size = 16 if i < 6 else 64
                    last = next_scale = 8
                    for _ in range(size):
                        if next_scale:
                            next_scale = (last + b.se() + 256) % 256
                        last = next_scale or last

    b.ue()                      # log2_max_frame_num_minus4
    poc_type = b.ue()
    if poc_type == 0:
        b.ue()                  # log2_max_pic_order_cnt_lsb_minus4
    elif poc_type == 1:
        b.u(1)                  # delta_pic_order_always_zero_flag
        b.se()                  # offset_for_non_ref_pic
        b.se()                  # offset_for_top_to_bottom_field
        for _ in range(b.ue()):
            b.se()

    b.ue()                      # max_num_ref_frames
    b.u(1)                      # gaps_in_frame_num_value_allowed_flag
    w_mbs = b.ue() + 1
    h_map_units = b.ue() + 1
    frame_mbs_only = b.u(1)
    if not frame_mbs_only:
        b.u(1)                  # mb_adaptive_frame_field_flag
    b.u(1)                      # direct_8x8_inference_flag

    crop_l = crop_r = crop_t = crop_b = 0
    if b.u(1):                  # frame_cropping_flag
        crop_l, crop_r, crop_t, crop_b = b.ue(), b.ue(), b.ue(), b.ue()

    # Crop offsets are in chroma sample units, so how many luma pixels each one
    # removes depends on the subsampling.
    sub_w = {0: 1, 1: 2, 2: 2, 3: 1}[chroma_format_idc]
    sub_h = {0: 1, 1: 2, 2: 1, 3: 1}[chroma_format_idc]
    crop_x = sub_w
    crop_y = sub_h * (2 - frame_mbs_only)

    width = w_mbs * 16 - (crop_l + crop_r) * crop_x
    height = h_map_units * 16 * (2 - frame_mbs_only) - (crop_t + crop_b) * crop_y
    return width, height


def collect_samples(data):
    """Group NAL units into access units - one per coded frame.

    Returns (samples, sps_list, pps_list). Each sample is the frame's NALs in
    AVCC form: a 4-byte big-endian length in front of each, which is how MP4
    stores them instead of start codes. SPS and PPS are pulled out entirely -
    they belong in the avcC box in the sample description, not in the frames -
    which is also why the same SPS/PPS repeated ahead of every IDR costs nothing
    in the output file.
    """
    samples = []            # list of (bytes, is_sync)
    sps_list, pps_list = [], []
    pending = []            # NALs seen since the last coded slice
    state = {'sync': False, 'vcl': False}

    def flush():
        # Only a coded slice makes a frame. Metadata NALs seen before the first
        # slice would otherwise be emitted as a sample of their own - a frame
        # with no picture in it, which shifts every timestamp after it by one.
        if pending and state['vcl']:
            samples.append((b''.join(pending), state['sync']))
        pending.clear()

    for s, e in split_nals(data):
        nal = data[s:e]
        ntype = nal[0] & 0x1F
        if ntype == NAL_SPS:
            if nal not in sps_list:
                sps_list.append(nal)
            continue
        if ntype == NAL_PPS:
            if nal not in pps_list:
                pps_list.append(nal)
            continue
        if ntype == NAL_AUD:
            continue
        if ntype in (NAL_SLICE, NAL_IDR):
            # A coded slice closes the previous access unit and opens this one.
            flush()
            state['sync'] = (ntype == NAL_IDR)
            state['vcl'] = True
        pending.append(struct.pack('>I', len(nal)) + nal)

    flush()
    return samples, sps_list, pps_list


def _box(kind, *payload):
    body = b''.join(payload)
    return struct.pack('>I', 8 + len(body)) + kind + body


def _full_box(kind, version, flags, *payload):
    return _box(kind, struct.pack('>BBBB', version, (flags >> 16) & 0xff,
                                  (flags >> 8) & 0xff, flags & 0xff), *payload)


def _avcc(sps_list, pps_list):
    """AVCDecoderConfigurationRecord - the decoder's out-of-band setup."""
    sps = sps_list[0]
    body = bytearray()
    body += bytes([1, sps[1], sps[2], sps[3]])
    body += bytes([0xFF])                       # 6 reserved bits + lengthSizeMinusOne = 3
    body += bytes([0xE0 | len(sps_list)])       # 3 reserved bits + SPS count
    for s in sps_list:
        body += struct.pack('>H', len(s)) + s
    body += bytes([len(pps_list)])
    for p in pps_list:
        body += struct.pack('>H', len(p)) + p
    return _box(b'avcC', bytes(body))


def _avc1(width, height, sps_list, pps_list):
    name = b'esp32p4 h.264'
    body = (b'\x00' * 6 + struct.pack('>H', 1)          # reserved, data_reference_index
            + b'\x00' * 16                              # pre_defined / reserved
            + struct.pack('>HH', width, height)
            + struct.pack('>II', 0x00480000, 0x00480000)  # 72 dpi h/v
            + struct.pack('>I', 0)                      # reserved
            + struct.pack('>H', 1)                      # frame_count
            + bytes([len(name)]) + name + b'\x00' * (31 - len(name))
            + struct.pack('>H', 0x0018)                 # depth
            + b'\xff\xff')                              # pre_defined = -1
    return _box(b'avc1', body, _avcc(sps_list, pps_list))


def _stts(durations):
    """Time-to-sample, run-length encoded over equal consecutive durations."""
    runs = []
    for d in durations:
        if runs and runs[-1][1] == d:
            runs[-1][0] += 1
        else:
            runs.append([1, d])
    body = struct.pack('>I', len(runs))
    for count, delta in runs:
        body += struct.pack('>II', count, delta)
    return _full_box(b'stts', 0, 0, body)


def build_mp4(data, timescale=1000, durations=None, fps=None,
              width=None, height=None):
    """Return the bytes of an MP4 wrapping the Annex-B stream in `data`.

    `durations` is one per sample, in `timescale` units; if it is None a
    constant duration from `fps` is used for every frame.
    """
    samples, sps_list, pps_list = collect_samples(data)
    if not samples:
        raise ValueError('no coded frames found - is this an Annex-B H.264 stream?')
    if not sps_list or not pps_list:
        raise ValueError('stream carries no SPS/PPS, so no decoder could be configured')

    sps_w, sps_h = parse_sps(sps_list[0])
    width = width or sps_w
    height = height or sps_h

    n = len(samples)
    if durations is None:
        step = max(1, round(timescale / (fps or 30.0)))
        durations = [step] * n
    elif len(durations) != n:
        raise ValueError(f'{len(durations)} timestamps for {n} frames')

    sizes = [len(s) for s, _ in samples]
    duration = sum(durations)

    mdat_payload = b''.join(s for s, _ in samples)
    mdat = _box(b'mdat', mdat_payload)
    ftyp = _box(b'ftyp', b'isom' + struct.pack('>I', 0x200) + b'isomiso2avc1mp41')

    # stco holds absolute file offsets, so the frame data has to be placed
    # before the header that indexes it - otherwise moov's size depends on the
    # offsets and the offsets depend on moov's size. Laying the file out as
    # ftyp/mdat/moov breaks that loop with no second pass, and is what a plain
    # (non-faststart) muxer produces anyway.
    base = len(ftyp) + (len(mdat) - len(mdat_payload))

    offsets = []
    off = base
    for sz in sizes:
        offsets.append(off)
        off += sz

    stbl = _box(
        b'stbl',
        _full_box(b'stsd', 0, 0, struct.pack('>I', 1), _avc1(width, height, sps_list, pps_list)),
        _stts(durations),
        # Sync-sample table: the frames a player may seek to. Omitting it would
        # declare every frame a random-access point, which is false for P frames
        # and makes seeking land on a smear.
        _full_box(b'stss', 0, 0, struct.pack('>I', sum(1 for _, k in samples if k)),
                  b''.join(struct.pack('>I', i + 1) for i, (_, k) in enumerate(samples) if k)),
        # One sample per chunk, so stco addresses each frame directly.
        _full_box(b'stsc', 0, 0, struct.pack('>I', 1), struct.pack('>III', 1, 1, 1)),
        _full_box(b'stsz', 0, 0, struct.pack('>I', 0), struct.pack('>I', n),
                  b''.join(struct.pack('>I', s) for s in sizes)),
        _full_box(b'stco', 0, 0, struct.pack('>I', n),
                  b''.join(struct.pack('>I', o) for o in offsets)),
    )

    unity_matrix = struct.pack('>9i', 0x10000, 0, 0, 0, 0x10000, 0, 0, 0, 0x40000000)

    moov = _box(
        b'moov',
        _full_box(b'mvhd', 0, 0,
                  struct.pack('>IIII', 0, 0, timescale, duration)
                  + struct.pack('>IH', 0x00010000, 0x0100)   # rate 1.0, volume 1.0
                  + b'\x00' * 10 + unity_matrix + b'\x00' * 24
                  + struct.pack('>I', 2)),                   # next_track_ID
        _box(
            b'trak',
            # flags 7 = enabled | in movie | in preview
            _full_box(b'tkhd', 0, 7,
                      struct.pack('>IIIII', 0, 0, 1, 0, duration)
                      + b'\x00' * 8 + struct.pack('>HHHH', 0, 0, 0, 0)
                      + unity_matrix
                      + struct.pack('>II', width << 16, height << 16)),
            _box(
                b'mdia',
                _full_box(b'mdhd', 0, 0,
                          struct.pack('>IIII', 0, 0, timescale, duration)
                          + struct.pack('>HH', 0x55C4, 0)),   # language 'und'
                _full_box(b'hdlr', 0, 0,
                          struct.pack('>I', 0) + b'vide' + b'\x00' * 12 + b'VideoHandler\x00'),
                _box(
                    b'minf',
                    _full_box(b'vmhd', 0, 1, struct.pack('>HHHH', 0, 0, 0, 0)),
                    _box(b'dinf', _full_box(b'dref', 0, 0, struct.pack('>I', 1),
                                            _full_box(b'url ', 0, 1))),
                    stbl,
                ),
            ),
        ),
    )

    return ftyp + mdat + moov


def durations_from_timestamps(ts_ms, timescale=1000, fallback_fps=30.0):
    """Per-sample durations from capture timestamps.

    The last frame has no successor to measure against, so it gets the mean of
    the others - a frame's worth of error at the very end of the clip, which is
    the only place it cannot matter.
    """
    if len(ts_ms) < 2:
        return [max(1, round(timescale / fallback_fps))] * len(ts_ms)
    scale = timescale / 1000.0
    d = [max(1, round((ts_ms[i + 1] - ts_ms[i]) * scale)) for i in range(len(ts_ms) - 1)]
    d.append(max(1, round(sum(d) / len(d))))
    return d


def main():
    ap = argparse.ArgumentParser(description=__doc__,
                                 formatter_class=argparse.RawDescriptionHelpFormatter)
    ap.add_argument('input', help='raw Annex-B .h264 stream')
    ap.add_argument('output', help='.mp4 to write')
    ap.add_argument('--fps', type=float, default=28.0,
                    help='frame rate to assume when no timestamps are supplied')
    args = ap.parse_args()

    with open(args.input, 'rb') as f:
        data = f.read()
    mp4 = build_mp4(data, fps=args.fps)
    with open(args.output, 'wb') as f:
        f.write(mp4)

    samples, sps, _ = collect_samples(data)
    w, h = parse_sps(sps[0])
    print(f'{args.output}: {len(samples)} frames, {w}x{h}, {len(mp4)} bytes')
    return 0


if __name__ == '__main__':
    sys.exit(main())
