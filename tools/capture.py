#!/usr/bin/env python3
"""Flash the board and/or receive captured frames from it over the serial link.

The port can only have one owner. `idf.py monitor` holds it, so a monitor left
running in another terminal makes `idf.py flash` fail with "Access is denied" -
and a monitor cannot extract a binary payload anyway, since it mangles the bytes
it prints. This script is the single owner: it flashes (optionally), resets the
board, captures the stream, and pulls the images out of it.

    python tools/capture.py --flash                  # flash, then capture
    python tools/capture.py                          # just capture
    python tools/capture.py --seconds 400 --out sweep # a FOCUS_SWEEP run

Frames are framed in the stream as:

    IMGSTART name=<n> fmt=<jpeg|rgb565> w=<w> h=<h> len=<N> crc32=<hex>
    <exactly N raw bytes>
    IMGEND

JPEG frames are written as .jpg. Raw RGB565 frames are written as .bmp so they
can be opened directly - the sweep produces raw specifically because JPEG is
lossy enough to distort a sharpness measurement (~18% on this metric), but a
.raw file nobody can open is not much use either.
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

try:
    import serial
except ImportError:
    sys.exit("pyserial not found. Run this with ESP-IDF's python, which has it:\n"
             "  ~/.espressif/python_env/idf5.4_py3.11_env/Scripts/python.exe tools/capture.py")

HDR = re.compile(rb'IMGSTART name=(\S+) fmt=(\S+) w=(\d+) h=(\d+) len=(\d+) crc32=([0-9a-f]+)\n')


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


def main():
    ap = argparse.ArgumentParser(description=__doc__,
                                 formatter_class=argparse.RawDescriptionHelpFormatter)
    ap.add_argument('--port', default='COM3')
    ap.add_argument('--baud', type=int, default=2000000,
                    help='must match CONFIG_ESP_CONSOLE_UART_BAUDRATE (default 2000000)')
    ap.add_argument('--seconds', type=float, default=30.0,
                    help='how long to listen; a sweep needs ~25 s per position')
    ap.add_argument('--out', default='capture')
    ap.add_argument('--flash', action='store_true', help='run idf.py flash first')
    ap.add_argument('--keep-raw', action='store_true',
                    help='also keep the undecoded .raw payload alongside the .bmp')
    args = ap.parse_args()

    if args.flash:
        print(f'--- flashing on {args.port} ---')
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
        r = subprocess.run(cmd + ['-p', args.port, 'flash'])
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

    print(f'--- listening on {args.port} at {args.baud} for {args.seconds:.0f}s ---')
    buf = bytearray()
    t0 = time.time()
    while time.time() - t0 < args.seconds:
        d = s.read(65536)
        if d:
            buf += d
    s.close()

    images, pos = [], 0
    while True:
        m = HDR.search(buf, pos)
        if not m:
            break
        name, fmt = m.group(1).decode(), m.group(2).decode()
        w, h, ln = int(m.group(3)), int(m.group(4)), int(m.group(5))
        crc = int(m.group(6), 16)
        start = m.end()
        payload = bytes(buf[start:start + ln])
        got = zlib.crc32(payload) & 0xffffffff
        ok = len(payload) == ln and got == crc
        if fmt == 'jpeg':
            path = os.path.join(args.out, f'{name}.jpg')
            open(path if ok else path + '.bad', 'wb').write(payload)
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
    if not images:
        print('  no images in the stream. Wrong --baud, or the run had not finished?')
    for line in log.splitlines()[-15:]:
        print(line)


if __name__ == '__main__':
    main()
