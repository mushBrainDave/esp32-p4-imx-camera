#!/usr/bin/env python3
"""Flash the board and/or receive captured frames from it over the serial link.

The port can only have one owner. `idf.py monitor` holds it, so a monitor left
running in another terminal makes `idf.py flash` fail with "Access is denied" -
and a monitor cannot extract a binary payload anyway, since it mangles the bytes
it prints. This script is the single owner: it flashes (optionally), resets the
board, captures the stream, and pulls the images out of it.

Run it from anywhere; it works out which project to flash:

    # from the repo root - defaults to examples/imx708_snapshot
    python tools/capture.py --flash

    # from inside any example directory
    python ../../tools/capture.py --flash

    # or point at a project explicitly, from anywhere
    python tools/capture.py --flash --project examples/imx708_snapshot

    python tools/capture.py --seconds 400 --out sweep   # a FOCUS_SWEEP run

    # imx708_video: 6 s aiming, 8 s recording, then a couple of MB to shift
    python tools/capture.py --flash --project examples/imx708_video --out clip

Payloads are framed in the stream as:

    IMGSTART name=<n> fmt=<jpeg|rgb565|h264> w=<w> h=<h> len=<N> crc32=<hex> [k=v ...]
    <exactly N raw bytes>
    IMGEND

Trailing key=value pairs are per-format extras (video adds fps=, frames=, ms=);
unknown keys are ignored, so an old receiver still reads a new sender's frames.

JPEG frames are written as .jpg. Raw RGB565 frames are written as .bmp so they
can be opened directly - the sweep produces raw specifically because JPEG is
lossy enough to distort a sharpness measurement (~18% on this metric), but a
.raw file nobody can open is not much use either. H.264 is written as both the
raw .h264 elementary stream and a .mp4 muxed around it, because an Annex-B
stream is not a container and most players will not touch one.
"""
import argparse
import os
import re
import struct
import shutil
import subprocess
import sys
import time
import zlib

sys.path.insert(0, os.path.dirname(os.path.abspath(__file__)))
import mp4

try:
    import serial
except ImportError:
    sys.exit("pyserial not found. Run this with ESP-IDF's python, which has it:\n"
             "  ~/.espressif/python_env/idf5.4_py3.11_env/Scripts/python.exe tools/capture.py")

HDR = re.compile(rb'IMGSTART name=(\S+) fmt=(\S+) w=(\d+) h=(\d+) len=(\d+) crc32=([0-9a-f]+)([^\n]*)\n')
# The per-frame table imx708_video prints ahead of a clip. See parse_frame_table.
# \r?\n, not \n: these are ordinary console lines, and the console VFS is still
# rewriting LF to CRLF at that point - it is only switched to bare LF inside the
# payload transfer, which is why the IMGSTART line above ends in a plain \n.
VIDFRAME = re.compile(rb'VIDFRAME i=(\d+) t=(\d+) off=(\d+) len=(\d+) type=(\w+)\r?\n')
# Both examples print this when they have finished, so there is no need to sit
# out the rest of a --seconds that was guessed high.
DONE_MARK = b'==== done'


def is_project(d):
    """An ESP-IDF project is a directory whose CMakeLists.txt calls project()."""
    cml = os.path.join(d, 'CMakeLists.txt')
    if not os.path.isfile(cml):
        return False
    try:
        with open(cml, encoding='utf-8', errors='replace') as f:
            return re.search(r'^\s*project\s*\(', f.read(), re.M) is not None
    except OSError:
        return False


def resolve_project(explicit):
    """Which project to flash: what was asked for, else the CWD, else the default.

    The script lives in <repo>/tools, so the repo root is one level up - that is
    how it finds the default example no matter where it was invoked from.
    """
    if explicit:
        d = os.path.abspath(explicit)
        if not is_project(d):
            sys.exit(f'{d} does not look like an ESP-IDF project '
                     '(no CMakeLists.txt with a project() call).')
        return d
    cwd = os.getcwd()
    if is_project(cwd):
        return cwd
    repo = os.path.dirname(os.path.dirname(os.path.abspath(__file__)))
    default = os.path.join(repo, 'examples', 'imx708_snapshot')
    return default if is_project(default) else None


def rgb565_to_bmp(payload, w, h, path):
    """Expand RGB565 (LE) to a 24-bit BMP. BMP rows are stored bottom-up."""
    row_bytes = w * 3
    pad = (-row_bytes) % 4
    img_bytes = (row_bytes + pad) * h
    hdr = bytearray(54)
    hdr[0:2] = b'BM'
    struct.pack_into('<I', hdr, 2, 54 + img_bytes)
    struct.pack_into('<I', hdr, 10, 54)
    struct.pack_into('<I', hdr, 14, 40)
    struct.pack_into('<i', hdr, 18, w)
    struct.pack_into('<i', hdr, 22, h)
    struct.pack_into('<H', hdr, 26, 1)
    struct.pack_into('<H', hdr, 28, 24)
    struct.pack_into('<I', hdr, 34, img_bytes)
    with open(path, 'wb') as f:
        f.write(hdr)
        padding = b'\0' * pad
        for y in range(h - 1, -1, -1):
            src = payload[y * w * 2:(y + 1) * w * 2]
            row = bytearray(row_bytes)
            for x in range(w):
                px = src[2 * x] | (src[2 * x + 1] << 8)
                r5, g6, b5 = (px >> 11) & 0x1f, (px >> 5) & 0x3f, px & 0x1f
                row[3 * x + 0] = (b5 << 3) | (b5 >> 2)
                row[3 * x + 1] = (g6 << 2) | (g6 >> 4)
                row[3 * x + 2] = (r5 << 3) | (r5 >> 2)
            f.write(row)
            if pad:
                f.write(padding)


def parse_extra(raw):
    """Trailing 'k=v k=v' fields of a frame header, as a dict of str -> str."""
    out = {}
    for tok in raw.decode('ascii', 'replace').split():
        if '=' in tok:
            k, v = tok.split('=', 1)
            out[k] = v
    return out


def parse_frame_table(buf, expect):
    """Capture timestamps, in ms, from the VIDFRAME lines the board prints.

    These are what the frames actually cost in wall-clock time, which is not
    the same as the frame rate the encoder was configured with: if the pipeline
    dropped frames, using the nominal rate would play the clip back fast and
    hide the drop. Returns None if the table is absent or does not match the
    frame count in the header, in which case the caller falls back to the
    nominal rate rather than trusting a half-read table.
    """
    ts = [(int(m.group(1)), int(m.group(2))) for m in VIDFRAME.finditer(buf)]
    if not ts or (expect is not None and len(ts) != expect):
        return None
    if [i for i, _ in ts] != list(range(len(ts))):
        return None
    return [t for _, t in ts]


def write_h264(payload, path, extra, buf, notes):
    """Write the elementary stream, and an .mp4 around it if it can be muxed.

    The .h264 is written first and kept either way: if muxing fails it is the
    evidence, and it is also what you would hand to a decoder directly.
    Returns the path worth pointing the user at.
    """
    with open(path, 'wb') as f:
        f.write(payload)

    try:
        fps = float(extra.get('fps', 0)) or 28.0
    except ValueError:
        fps = 28.0
    frames = int(extra['frames']) if extra.get('frames', '').isdigit() else None
    ts = parse_frame_table(buf, frames)

    mp4_path = os.path.splitext(path)[0] + '.mp4'
    durations = mp4.durations_from_timestamps(ts, fallback_fps=fps) if ts else None
    data = None
    for attempt in ((durations, None) if durations else (None,)):
        try:
            data = mp4.build_mp4(payload, durations=attempt, fps=fps)
        except Exception as e:                              # noqa: BLE001
            # A timestamp count that disagrees with the number of frames in the
            # bitstream means one of the two is incomplete. Say so, then mux at
            # the nominal rate anyway: a clip with slightly wrong timing beats
            # no clip at all, and the .h264 is on disk either way.
            notes.append(f'{os.path.basename(path)}: {e}')
            continue
        timing = 'capture timestamps' if attempt else f'a nominal {fps:g} fps'
        notes.append(f'muxed {len(data)} bytes -> {mp4_path}, timed from {timing}')
        break

    if data is None:
        return path
    with open(mp4_path, 'wb') as f:
        f.write(data)
    return mp4_path


def main():
    ap = argparse.ArgumentParser(description=__doc__,
                                 formatter_class=argparse.RawDescriptionHelpFormatter)
    ap.add_argument('--port', default='COM3')
    ap.add_argument('--baud', type=int, default=2000000,
                    help='must match CONFIG_ESP_CONSOLE_UART_BAUDRATE (default 2000000)')
    ap.add_argument('--seconds', type=float, default=90.0,
                    help='ceiling on how long to listen. Capture stops as soon as the '
                         'board reports it is done, so this only matters when a run '
                         'never gets there; a sweep needs ~25 s per position.')
    ap.add_argument('--out', default='capture')
    ap.add_argument('--flash', action='store_true', help='run idf.py flash first')
    ap.add_argument('--project', default=None,
                    help='ESP-IDF project to flash. Defaults to the current directory '
                         'when it is a project, else examples/imx708_snapshot.')
    ap.add_argument('--keep-raw', action='store_true',
                    help='also keep the undecoded .raw payload alongside the .bmp')
    args = ap.parse_args()

    project = resolve_project(args.project)

    if args.flash:
        if not project:
            sys.exit('No ESP-IDF project found. Run this from a project directory '
                     'or pass --project <dir>.')
        print(f'--- flashing {project} on {args.port} ---')
        # idf.py is a Python script, not an executable; on Windows it can only be
        # run directly if the export step left a .exe/.bat shim on PATH. Fall
        # back to invoking it through the interpreter.
        exe = shutil.which('idf.py')
        if exe and os.path.splitext(exe)[1].lower() in ('.exe', '.bat', '.cmd'):
            cmd = [exe]
        else:
            idf_path = os.environ.get('IDF_PATH')
            if not idf_path:
                sys.exit('IDF_PATH is not set - run this from an activated ESP-IDF shell.')
            cmd = [sys.executable, os.path.join(idf_path, 'tools', 'idf.py')]
        r = subprocess.run(cmd + ['-p', args.port, 'flash'], cwd=project)
        if r.returncode != 0:
            sys.exit('flash failed. Is a serial monitor still holding the port?')

    os.makedirs(args.out, exist_ok=True)

    s = serial.Serial(args.port, args.baud, timeout=0.05)
    try:
        s.set_buffer_size(rx_size=1 << 20)
    except Exception:
        pass
    # Reset so the capture starts from boot: RTS drives EN, DTR drives IO0.
    # DTR low keeps IO0 high, so it runs the app rather than the ROM loader.
    s.dtr = False
    s.rts = True
    time.sleep(0.15)
    s.rts = False

    print(f'--- listening on {args.port} at {args.baud} for up to {args.seconds:.0f}s ---')
    buf = bytearray()
    t0 = time.time()
    while time.time() - t0 < args.seconds:
        d = s.read(65536)
        if d:
            buf += d
            # Both examples announce when they are finished, so a --seconds
            # guessed on the high side costs nothing.
            if DONE_MARK in d or DONE_MARK in buf[-len(d) - 32:]:
                print(f'--- board reported done after {time.time() - t0:.1f}s ---')
                break
    s.close()

    images, notes, pos = [], [], 0
    while True:
        m = HDR.search(buf, pos)
        if not m:
            break
        name, fmt = m.group(1).decode(), m.group(2).decode()
        w, h, ln = int(m.group(3)), int(m.group(4)), int(m.group(5))
        crc = int(m.group(6), 16)
        extra = parse_extra(m.group(7))
        start = m.end()
        payload = bytes(buf[start:start + ln])
        got = zlib.crc32(payload) & 0xffffffff
        ok = len(payload) == ln and got == crc
        if fmt == 'jpeg':
            path = os.path.join(args.out, f'{name}.jpg')
            open(path if ok else path + '.bad', 'wb').write(payload)
        elif fmt == 'h264':
            path = os.path.join(args.out, f'{name}.h264')
            if ok:
                # Only the log ahead of the header, never the payload: the
                # frame table is printed before the clip, and scanning binary
                # video for VIDFRAME lines could only ever find a false one.
                path = write_h264(payload, path, extra, buf[:m.start()], notes)
            else:
                # Keep it anyway: a truncated clip still plays up to the break,
                # and its length says where the transfer came apart.
                open(path + '.bad', 'wb').write(payload)
        else:
            path = os.path.join(args.out, f'{name}.bmp')
            if ok:
                rgb565_to_bmp(payload, w, h, path)
            if args.keep_raw or not ok:
                open(os.path.join(args.out, f'{name}.raw'), 'wb').write(payload)
        images.append((name, fmt, w, h, ln, len(payload), crc, got, ok, path))
        pos = start + ln

    # Log text is everything outside the frame payloads.
    text, pos = bytearray(), 0
    for m in HDR.finditer(buf):
        text += buf[pos:m.start()]
        pos = m.end() + int(m.group(5))
    text += buf[pos:]
    log = text.decode('utf-8', 'replace')
    open(os.path.join(args.out, 'log.txt'), 'w', encoding='utf-8', newline='').write(log)

    print(f'--- {len(buf)} bytes captured ---')
    for name, fmt, w, h, ln, got_len, crc, got, ok, path in images:
        flag = 'OK ' if ok else 'BAD'
        print(f'  [{flag}] {name:14s} {fmt:6s} {w}x{h} {ln} bytes '
              f'crc {crc:08x}/{got:08x} -> {path}')
    for note in notes:
        print(f'  {note}')
    if not images:
        print('  no images in the stream. Wrong --baud, or the run had not finished?')
    # The frame table is one line per frame and would bury everything else.
    for line in log.splitlines()[-15:]:
        if not line.startswith('VIDFRAME'):
            print(line)


if __name__ == '__main__':
    main()
