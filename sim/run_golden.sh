#!/usr/bin/env bash
# Golden-frame harness for the desktop soft-GPU sim (milestone A5-2).
#
# Renders every benchmark scene with build/sim/protogpu_sim and compares the
# result against the frozen reference frames in sim/golden/<scene>.ppm.
# This is the pixel-regression gate for all firmware optimizations
# (docs/OPTIMIZATION_PLAN.md §2: F-01…, S-01…, V9 features).
#
# Usage:
#   sim/run_golden.sh                 render all scenes → compare vs golden
#   sim/run_golden.sh --write-golden  render all scenes → REPLACE sim/golden/
#   sim/run_golden.sh --self-test     negative test: flip pixels in a copy,
#                                     confirm the comparator FAILS (see below)
#
# Golden pinning: the references are exact only for
#   native g++  x86-64  -O1  little-endian
# (build_sim.sh pins -O1 for this reason).  Floating-point codegen varies
# between compilers/libm versions, so a different toolchain may shift a few
# pixels.  For that case there is a PER-SCENE BOUNDED-DIFF OVERRIDE below:
# MAX_DIFF_PIXELS (how many pixels may differ) and MAX_CHANNEL_DELTA (largest
# allowed per-channel difference, 0–255 in PPM space).  Default is
# PIXEL-IDENTICAL (0 / 0) — only raise a scene's bounds after its drift has
# been eyeballed and understood.
#
# Exit code: 0 = all scenes PASS, 1 = any FAIL / missing golden / missing sim.

set -u

SCRIPT_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
REPO_ROOT="$(cd "$SCRIPT_DIR/.." && pwd)"
BUILD_DIR="${BUILD_DIR:-$REPO_ROOT/build/sim}"
SIM="$BUILD_DIR/protogpu_sim"
GOLDEN_DIR="$SCRIPT_DIR/golden"
FRAMES_DIR="$BUILD_DIR/frames"
COMPARE="$SCRIPT_DIR/ppm_compare.py"

# Benchmark scenes (registry names; B5_lvgl is a placeholder — no frames yet).
SCENES=(cube B1_teapot B2_2d_layers B3_textures B4_psb_postfx B6_empty)

# ─── Per-scene bounded-diff overrides (default: pixel-identical 0/0) ────────
declare -A MAX_DIFF_PIXELS MAX_CHANNEL_DELTA
# MAX_DIFF_PIXELS[B3_textures]=40
# MAX_CHANNEL_DELTA[B3_textures]=2

limits_for() {
    local s="$1"
    MAXP="${MAX_DIFF_PIXELS[$s]:-0}"
    MAXD="${MAX_CHANNEL_DELTA[$s]:-0}"
}

if [ ! -x "$SIM" ]; then
    echo "ERROR: sim binary not found at $SIM"
    echo "       build it first: ./sim/build_sim.sh"
    exit 1
fi

render_scene() {  # <scene> <out.ppm>  — quiet render, returns sim exit code
    "$SIM" --scene "$1" "$2" > "$FRAMES_DIR/$1.log" 2>&1
}

# ─── --write-golden: regenerate the reference frames ────────────────────────
if [ "${1:-}" = "--write-golden" ]; then
    mkdir -p "$GOLDEN_DIR" "$FRAMES_DIR"
    echo "Writing golden references to $GOLDEN_DIR"
    echo "(pinned to: native g++ x86-64, -O1, little-endian)"
    rc=0
    for s in "${SCENES[@]}"; do
        if ! render_scene "$s" "$GOLDEN_DIR/$s.ppm"; then
            echo "  $s: RENDER FAILED (see $FRAMES_DIR/$s.log)"
            rc=1
            continue
        fi
        cksum=$(grep "FNV-1a checksum" "$FRAMES_DIR/$s.log" | awk '{print $NF}')
        echo "  $s: golden written  checksum $cksum"
    done
    exit $rc
fi

# ─── --self-test: prove the comparator catches modifications ────────────────
if [ "${1:-}" = "--self-test" ]; then
    s="cube"
    tmp="$FRAMES_DIR/selftest"
    mkdir -p "$tmp" "$GOLDEN_DIR"
    echo "Golden harness self-test (negative test)"
    echo "  scene: $s"

    if [ ! -f "$GOLDEN_DIR/$s.ppm" ]; then
        echo "  ERROR: no golden for $s — run --write-golden first"
        exit 1
    fi

    # 1. Render the scene and compare the unmodified frame vs its golden:
    #    this MUST pass (otherwise the harness or the build is broken).
    render_scene "$s" "$tmp/clean.ppm" || { echo "  ERROR: render failed"; exit 1; }
    clean_out=$(python3 "$COMPARE" "$tmp/clean.ppm" "$GOLDEN_DIR/$s.ppm")
    clean_rc=$?
    echo "  unmodified frame : $clean_out"

    # 2. Flip a few pixels in a copy of the golden and compare: MUST fail.
    python3 - "$GOLDEN_DIR/$s.ppm" "$tmp/flipped.ppm" <<'EOF'
import sys
data = bytearray(open(sys.argv[1], "rb").read())
# P6 header for these frames is "P6\n128 64\n255\n" (13 bytes); the raster
# follows.  Invert the RGB channels of 4 well-separated pixels.
hdr_end = data.find(b"255\n") + 4
w = 128
for (x, y) in [(5, 5), (64, 32), (100, 50), (30, 40)]:
    o = hdr_end + (y * w + x) * 3
    data[o] ^= 0xFF
    data[o + 1] ^= 0xFF
    data[o + 2] ^= 0xFF
open(sys.argv[2], "wb").write(bytes(data))
print("  flipped 4 pixels in copy: (5,5) (64,32) (100,50) (30,40)")
EOF
    flip_out=$(python3 "$COMPARE" "$tmp/flipped.ppm" "$GOLDEN_DIR/$s.ppm")
    flip_rc=$?
    echo "  flipped frame    : $flip_out"

    ok=1
    if [ $clean_rc -ne 0 ]; then echo "  SELF-TEST BROKEN: unmodified frame did not match golden"; ok=0; fi
    if [ $flip_rc -eq 0 ]; then echo "  SELF-TEST BROKEN: comparator missed flipped pixels"; ok=0; fi
    if [ $ok -eq 1 ]; then
        echo "SELF-TEST PASS: clean frame matches, flipped frame detected (comparator works)"
        exit 0
    fi
    echo "SELF-TEST FAIL"
    exit 1
fi

if [ "${1:-}" = "--help" ] || [ "${1:-}" = "-h" ]; then
    sed -n '2,30p' "$0"
    exit 0
fi

# ─── Default: render every scene and compare against golden ─────────────────
mkdir -p "$FRAMES_DIR"
echo "Golden-frame comparison (default: pixel-identical)"
echo "  sim:    $SIM"
echo "  golden: $GOLDEN_DIR"
echo "  frames: $FRAMES_DIR"
echo

fails=0
for s in "${SCENES[@]}"; do
    golden="$GOLDEN_DIR/$s.ppm"
    frame="$FRAMES_DIR/$s.ppm"

    if [ ! -f "$golden" ]; then
        echo "FAIL $s: no golden reference (run: $0 --write-golden)"
        fails=$((fails + 1))
        continue
    fi
    if ! render_scene "$s" "$frame"; then
        echo "FAIL $s: render failed (see $FRAMES_DIR/$s.log)"
        fails=$((fails + 1))
        continue
    fi

    limits_for "$s"
    out=$(python3 "$COMPARE" "$frame" "$golden" \
            --max-diff-pixels="$MAXP" --max-channel-delta="$MAXD")
    rc=$?
    cksum=$(grep "FNV-1a checksum" "$FRAMES_DIR/$s.log" | awk '{print $NF}')
    if [ $rc -eq 0 ]; then
        echo "PASS $s  [$out]  checksum $cksum"
    else
        echo "FAIL $s  [$out]  checksum $cksum  (log: $FRAMES_DIR/$s.log)"
        fails=$((fails + 1))
    fi
done

echo
if [ $fails -eq 0 ]; then
    echo "GOLDEN RESULT: PASS (${#SCENES[@]} scenes)"
    exit 0
fi
echo "GOLDEN RESULT: FAIL ($fails of ${#SCENES[@]} scenes)"
exit 1
