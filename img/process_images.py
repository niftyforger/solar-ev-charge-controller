#!/usr/bin/env python3
"""Downsize images in this folder for the README and strip embedded metadata
(GPS, device model, timestamps, etc.). Run after adding new photos, before committing.

Usage:
    python img/process_images.py            # process every image in this folder
    python img/process_images.py foo.jpg     # process specific files only

Requires Pillow (`pip install pillow`).
"""
import sys
from pathlib import Path

from PIL import Image, ImageOps

MAX_DIM = 1600
JPEG_QUALITY = 85
EXTENSIONS = {".jpg", ".jpeg", ".png"}


def process(path: Path) -> bool:
    im = Image.open(path)
    im = ImageOps.exif_transpose(im)  # bake in EXIF rotation before the tag itself is dropped

    resized = max(im.size) > MAX_DIM
    if resized:
        im.thumbnail((MAX_DIM, MAX_DIM), Image.Resampling.LANCZOS)

    if not resized and not im.getexif():
        return False  # already small and metadata-free, skip re-encoding

    # No exif=/icc_profile= passed to save(), so none of the original metadata survives.
    if path.suffix.lower() in (".jpg", ".jpeg"):
        im.save(path, quality=JPEG_QUALITY, optimize=True)
    else:
        im.save(path, optimize=True)
    return True


def main(argv: list[str]) -> None:
    folder = Path(__file__).parent
    targets = (
        [Path(a) for a in argv]
        if argv
        else sorted(p for p in folder.iterdir() if p.suffix.lower() in EXTENSIONS)
    )
    for path in targets:
        try:
            before = path.stat().st_size
            changed = process(path)
            after = path.stat().st_size
        except Exception as e:
            print(f"{path.name}: skipped ({e})")
            continue
        status = f"{before} -> {after} bytes" if changed else "already clean, skipped"
        print(f"{path.name}: {status}")


if __name__ == "__main__":
    main(sys.argv[1:])
