#!/usr/bin/env python3
"""Remove ``override_path`` from the example manifests before a registry upload.

The examples under ``components/esp_cam_sensor_imx/examples`` declare the driver
with an ``override_path`` so that a checkout of this repository builds against
its own working tree instead of downloading a published version. That field is
meaningless once the example has been copied out of the component: the relative
path lands somewhere arbitrary in the user's filesystem, and the component
manager stops with ``the override_path you're using is pointing to directory
"..." that is not a component``.

``compote component pack`` copies the manifests through verbatim, so run this
first. It rewrites the files in place; CI runs it on a throwaway checkout, and a
manual upload should run it and then ``git checkout`` the manifests afterwards.

Usage:  python tools/strip_example_overrides.py [component_dir ...]
"""

from __future__ import annotations

import sys
from pathlib import Path

DEFAULT_COMPONENTS = ["components/esp_cam_sensor_imx"]


def strip(manifest: Path) -> bool:
    """Drop the override_path line. What is left is still a valid dependency -
    a mapping carrying just ``version:`` - so nothing else needs rewriting.
    Returns True if the file changed."""
    lines = manifest.read_text(encoding="utf-8").splitlines(keepends=True)
    out: list[str] = []
    changed = False

    for line in lines:
        if line.lstrip().startswith("override_path:"):
            changed = True
            continue
        out.append(line)

    if changed:
        manifest.write_text("".join(out), encoding="utf-8")
    return changed


def main(argv: list[str]) -> int:
    roots = argv[1:] or DEFAULT_COMPONENTS
    touched = 0

    for root in roots:
        examples = Path(root) / "examples"
        if not examples.is_dir():
            print(f"no examples directory under {root}", file=sys.stderr)
            continue
        for manifest in sorted(examples.glob("*/main/idf_component.yml")):
            if strip(manifest):
                print(f"stripped override_path from {manifest.as_posix()}")
                touched += 1

    if not touched:
        print("no override_path fields found - nothing to do")
    return 0


if __name__ == "__main__":
    raise SystemExit(main(sys.argv))
