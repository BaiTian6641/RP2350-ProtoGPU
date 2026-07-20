#!/usr/bin/env python3
"""PPM (P6) comparator for the golden-frame harness (sim/run_golden.sh).

Compares two binary PPM images pixel-by-pixel and reports diff stats:
  - pixels differed (count of pixels where any RGB channel differs)
  - max per-channel delta (0-255)

Default mode is PIXEL-IDENTICAL (0 differing pixels).  Bounded-diff escape
hatch for toolchain float variance (goldens are pinned to native g++ x86-64,
-O1, little-endian; a different compiler/libm may shift a few pixels):

  ppm_compare.py A.ppm B.ppm [--max-diff-pixels N] [--max-channel-delta D]

Exit codes: 0 = PASS (within bounds), 1 = FAIL, 2 = usage/parse error.
Output (one line, consumed by run_golden.sh):
  PASS pixels_differed=0 max_channel_delta=0 (of 8192 px)
  FAIL pixels_differed=17 max_channel_delta=23 (of 8192 px; limits: 0 px, 0 delta)
"""

import sys


def read_ppm(path):
    """Read a binary P6 PPM. Returns (width, height, maxval, pixel bytes)."""
    with open(path, "rb") as f:
        data = f.read()

    # Tokenise the header: magic, width, height, maxval — '# ...' comments
    # may appear between tokens (P6 spec).
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
    i += 1  # single whitespace byte after maxval

    if tokens[0] != b"P6":
        raise ValueError("not a P6 PPM (magic %r)" % tokens[0])
    w, h, maxval = int(tokens[1]), int(tokens[2]), int(tokens[3])
    raster = data[i:]
    expected = w * h * 3
    if len(raster) != expected:
        raise ValueError("raster size %d != expected %d (%dx%d)"
                         % (len(raster), expected, w, h))
    return w, h, maxval, raster


def main(argv):
    args = [a for a in argv if not a.startswith("--")]
    opts = {"--max-diff-pixels": 0, "--max-channel-delta": 0}
    for a in argv:
        if a.startswith("--"):
            for key in opts:
                if a.startswith(key + "="):
                    opts[key] = int(a.split("=", 1)[1])
    if len(args) != 3:  # argv[0] + two files
        sys.stderr.write(__doc__)
        return 2

    try:
        wA, hA, maxA, rasA = read_ppm(args[1])
        wB, hB, maxB, rasB = read_ppm(args[2])
    except (OSError, ValueError) as e:
        sys.stderr.write("ppm_compare: %s\n" % e)
        return 2

    if (wA, hA) != (wB, hB) or maxA != maxB:
        print("FAIL geometry mismatch: %dx%d@%d vs %dx%d@%d"
              % (wA, hA, maxA, wB, hB, maxB))
        return 1

    total = wA * hA
    diff_pixels = 0
    max_delta = 0
    for px in range(total):
        o = px * 3
        d0 = abs(rasA[o] - rasB[o])
        d1 = abs(rasA[o + 1] - rasB[o + 1])
        d2 = abs(rasA[o + 2] - rasB[o + 2])
        if d0 or d1 or d2:
            diff_pixels += 1
            m = d0 if d0 > d1 else d1
            if d2 > m:
                m = d2
            if m > max_delta:
                max_delta = m

    ok = (diff_pixels <= opts["--max-diff-pixels"]
          and max_delta <= opts["--max-channel-delta"])
    if ok:
        print("PASS pixels_differed=%d max_channel_delta=%d (of %d px)"
              % (diff_pixels, max_delta, total))
        return 0
    print("FAIL pixels_differed=%d max_channel_delta=%d (of %d px; limits: %d px, %d delta)"
          % (diff_pixels, max_delta, total,
             opts["--max-diff-pixels"], opts["--max-channel-delta"]))
    return 1


if __name__ == "__main__":
    sys.exit(main(sys.argv))
