#!/usr/bin/env python3
"""Minimal P6 PPM → PNG converter (pure stdlib: zlib + struct).

Usage: ppm_to_png.py in.ppm [out.png]
Scales the small 128×64 frames up 4× with nearest-neighbour so they are
viewable; PNG output is RGB, no alpha.
"""

import struct
import sys
import zlib


def read_ppm(path):
    data = open(path, "rb").read()
    tokens = []
    i = 0
    n = len(data)
    while len(tokens) < 4:
        while i < n and data[i:i + 1].isspace():
            i += 1
        if i < n and data[i:i + 1] == b"#":
            while i < n and data[i:i + 1] != b"\n":
                i += 1
            continue
        start = i
        while i < n and not data[i:i + 1].isspace():
            i += 1
        tokens.append(data[start:i])
    i += 1
    if tokens[0] != b"P6":
        raise ValueError("not a P6 PPM")
    w, h = int(tokens[1]), int(tokens[2])
    return w, h, data[i:]


def write_png(path, w, h, rgb):
    def chunk(tag, payload):
        c = tag + payload
        return struct.pack(">I", len(payload)) + c + struct.pack(">I", zlib.crc32(c))

    raw = b"".join(b"\x00" + rgb[y * w * 3:(y + 1) * w * 3] for y in range(h))
    png = (b"\x89PNG\r\n\x1a\n"
           + chunk(b"IHDR", struct.pack(">IIBBBBB", w, h, 8, 2, 0, 0, 0))
           + chunk(b"IDAT", zlib.compress(raw, 9))
           + chunk(b"IEND", b""))
    open(path, "wb").write(png)


def main(argv):
    if len(argv) < 2:
        sys.stderr.write(__doc__)
        return 2
    src = argv[1]
    dst = argv[2] if len(argv) > 2 else src.rsplit(".", 1)[0] + ".png"
    scale = 4
    w, h, rgb = read_ppm(src)
    if scale != 1:
        rows = []
        for y in range(h):
            row = bytearray()
            for x in range(w):
                px = rgb[(y * w + x) * 3:(y * w + x) * 3 + 3]
                row += px * scale
            for _ in range(scale):
                rows.append(bytes(row))
        rgb = b"".join(rows)
        w *= scale
        h *= scale
    write_png(dst, w, h, rgb)
    print("wrote %s (%dx%d)" % (dst, w, h))
    return 0


if __name__ == "__main__":
    sys.exit(main(sys.argv))
