#!/usr/bin/env python3
"""Render a map .pgm to an upscaled .png so it can be eyeballed, and report
the occupied/free/unknown pixel counts.

Deliberately dependency-free (stdlib zlib only) so it runs on the macOS side
where there is no ROS 2, no OpenCV and no numpy guarantee.

    python3 preview_map.py track.pgm            # -> track_preview.png
    python3 preview_map.py track.pgm -s 8       # 8x upscale
"""

import argparse
import struct
import sys
import zlib
from collections import Counter

# map_saver's trinary output only ever contains these three values.
OCCUPIED, UNKNOWN, FREE = 0, 205, 254


def read_pgm(path):
    """Parse a binary (P5) PGM. Returns (width, height, bytes)."""
    with open(path, 'rb') as f:
        tokens = []
        while len(tokens) < 4:
            line = f.readline()
            if not line:
                raise ValueError(f'{path}: truncated PGM header')
            if line.startswith(b'#'):  # map_saver writes no comments, but be safe
                continue
            tokens += line.split()
        magic, width, height, maxval = tokens[:4]
        if magic != b'P5':
            raise ValueError(f'{path}: not a binary PGM (got {magic!r}). '
                             'In GIMP, export with "Raw" data formatting, not ASCII.')
        width, height = int(width), int(height)
        data = f.read(width * height)
    if len(data) != width * height:
        raise ValueError(f'{path}: expected {width * height} pixels, got {len(data)}')
    return width, height, data


def write_png(path, width, height, data, scale):
    """Write an 8-bit greyscale PNG, nearest-neighbour upscaled by `scale`.

    Nearest-neighbour matters: the point is to see individual cells, so
    smoothing would hide exactly the single-pixel speckle we are looking for.
    """
    rows = []
    for y in range(height):
        row = bytearray()
        for x in range(width):
            row += bytes([data[y * width + x]]) * scale
        # 0x00 = "no filter" for this scanline
        rows.extend([b'\x00' + bytes(row)] * scale)

    def chunk(tag, payload):
        return (struct.pack('>I', len(payload)) + tag + payload
                + struct.pack('>I', zlib.crc32(tag + payload) & 0xffffffff))

    png = b'\x89PNG\r\n\x1a\n'
    png += chunk(b'IHDR', struct.pack('>IIBBBBB', width * scale, height * scale,
                                      8, 0, 0, 0, 0))
    png += chunk(b'IDAT', zlib.compress(b''.join(rows)))
    png += chunk(b'IEND', b'')
    with open(path, 'wb') as f:
        f.write(png)


def main():
    ap = argparse.ArgumentParser(description=__doc__,
                                 formatter_class=argparse.RawDescriptionHelpFormatter)
    ap.add_argument('pgm', help='input .pgm')
    ap.add_argument('-o', '--output', help='output .png (default: <input>_preview.png)')
    ap.add_argument('-s', '--scale', type=int, default=4, help='upscale factor (default 4)')
    args = ap.parse_args()

    width, height, data = read_pgm(args.pgm)
    out = args.output or args.pgm.rsplit('.', 1)[0] + '_preview.png'
    write_png(out, width, height, data, args.scale)

    counts = Counter(data)
    total = width * height
    print(f'{args.pgm}: {width} x {height} px, {width * 0.05:.2f} x {height * 0.05:.2f} m '
          f'at 0.05 m/px')
    for value, label in ((OCCUPIED, 'occupied (wall)'), (FREE, 'free'), (UNKNOWN, 'unknown')):
        n = counts.pop(value, 0)
        print(f'  {label:<16} {n:>7}  ({100.0 * n / total:5.2f}%)')
    if counts:
        # Anything else means the editor anti-aliased or dithered. nav2 will
        # bucket these by the yaml thresholds rather than error, so the map
        # still loads -- it just quietly disagrees with what you drew.
        print(f'  WARNING: {sum(counts.values())} pixel(s) with off-palette values '
              f'{sorted(counts)[:8]}{"..." if len(counts) > 8 else ""}')
        print('  Turn OFF anti-aliasing/feathering in your editor and use only '
              f'{OCCUPIED} (wall), {FREE} (free), {UNKNOWN} (unknown).')
    print(f'wrote {out}')


if __name__ == '__main__':
    sys.exit(main())
