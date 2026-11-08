#!/usr/bin/env python3
"""Fetch the pinned IPA compatibility checkout without changing an existing one."""
from pathlib import Path
import subprocess

ROOT = Path(__file__).resolve().parents[1]
DEST = ROOT / 'third_party/esp-video-components'
REVISION = '1f7d39e37b69be18ce977dbab83d1717abcebbe4'


def git(*args):
    return subprocess.check_output(['git', *map(str, args)], text=True).strip()


if not DEST.exists():
    subprocess.run(['git', 'clone', '--no-checkout', '--filter=blob:none',
                    'https://github.com/espressif/esp-video-components.git',
                    str(DEST)], check=True)
    subprocess.run(['git', '-C', str(DEST), 'fetch', '--depth=1', 'origin', REVISION],
                   check=True)
    subprocess.run(['git', '-C', str(DEST), 'checkout', '--detach', REVISION], check=True)
if git('-C', DEST, 'rev-parse', 'HEAD') != REVISION:
    raise SystemExit('Existing IPA checkout has a different revision; preserved it.')
if git('-C', DEST, 'status', '--porcelain'):
    raise SystemExit('Existing IPA checkout has local changes; preserved them.')
print('Pinned IPA dependency ready:', REVISION)
