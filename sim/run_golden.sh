#!/usr/bin/env bash
# Read-only historical golden diagnostics for the live protocol9 native engine.
# Usage: sim/run_golden.sh [--self-test]
# References are never rewritten. Intentional depth/edge/alpha/target-space
# differences require review of the reported pixels alongside content oracles.
set -uo pipefail
SCRIPT_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
REPO_ROOT="$(cd "$SCRIPT_DIR/.." && pwd)"
BUILD_DIR="${BUILD_DIR:-$REPO_ROOT/build/sim}"
SIM="$BUILD_DIR/protogpu_sim"
GOLDEN_DIR="$SCRIPT_DIR/golden"
FRAMES_DIR="$BUILD_DIR/frames"
COMPARE="$SCRIPT_DIR/ppm_compare.py"
SCENES=(cube B1_teapot B2_2d_layers B3_textures B4_psb_postfx B6_empty B7_alpha3d B8_bilinear B9_multicam)
case "${1:-}" in
    --help|-h)
        echo "Usage: $0 [--self-test]"
        echo "Render nine scenes and diagnose differences from read-only historical goldens."
        echo "Build first: ./sim/build_sim.sh --build-only"
        exit 0 ;;
    --write-golden)
        echo "ERROR: golden regeneration is intentionally disabled; inspect differences instead." >&2
        exit 2 ;;
    ''|--self-test) ;;
    *) echo "ERROR: unknown argument $1" >&2; exit 2 ;;
esac
mkdir -p "$FRAMES_DIR"
if [[ "${1:-}" == --self-test ]]; then
    # Comparator self-test is independent of intentional renderer changes:
    # compare a reference with itself, then change a copy, never the reference.
    tmp="$FRAMES_DIR/selftest"
    mkdir -p "$tmp"
    reference="$GOLDEN_DIR/cube.ppm"
    python3 "$COMPARE" "$reference" "$reference" || exit 1
    python3 - "$reference" "$tmp/flipped.ppm" "$SCRIPT_DIR" <<'PY'
import sys
sys.path.insert(0, sys.argv[3])
from ppm_compare import read_ppm
w, h, maxval, pixels = read_ppm(sys.argv[1])
pixels = bytearray(pixels)
for x, y in [(5, 5), (w // 2, h // 2), (w - 6, h - 6), (30, 40)]:
    offset = (y * w + x) * 3
    pixels[offset:offset + 3] = bytes(c ^ 255 for c in pixels[offset:offset + 3])
with open(sys.argv[2], 'wb') as f:
    f.write(f'P6\n{w} {h}\n{maxval}\n'.encode() + pixels)
PY
    [[ $? -eq 0 ]] || exit 1
    python3 "$COMPARE" "$tmp/flipped.ppm" "$reference"
    result=$?
    if [[ $result -ne 1 ]]; then
        echo "SELF-TEST FAIL: expected pixel-difference failure, got $result"
        exit 1
    fi
    echo "SELF-TEST PASS: unchanged reference accepted, modified copy diagnosed"
    exit 0
fi
if [[ ! -x "$SIM" ]]; then
    echo "ERROR: missing $SIM; run ./sim/build_sim.sh --build-only" >&2
    exit 1
fi
echo "Historical golden diagnostics (pixel-identical limits; no automatic repinning)"
echo "  rendered frames and actual engine timing logs: $FRAMES_DIR"
echo "  Intentional differences: perspective depth/top-left edges/ordered alpha;"
echo "  Projection remains panel-space; original references are preserved."
fails=0
for scene in "${SCENES[@]}"; do
    frame="$FRAMES_DIR/$scene.ppm"
    log="$FRAMES_DIR/$scene.log"
    # The CLI writes the actual image before evaluating content oracles, so
    # image differences remain inspectable even when an oracle rejects it.
    rm -f -- "$frame"
    if "$SIM" --scene "$scene" "$frame" > "$log" 2>&1; then
        echo "CONTENT PASS $scene (log: $log)"
    else
        echo "CONTENT FAIL $scene (parser/render/oracle details: $log)"
        fails=$((fails + 1))
        [[ -f "$frame" ]] || continue
    fi
    if [[ ! -f "$GOLDEN_DIR/$scene.ppm" ]]; then
        echo "DIFF FAIL $scene: missing historical reference (render preserved at $frame)"
        fails=$((fails + 1))
        continue
    fi
    if python3 "$COMPARE" "$frame" "$GOLDEN_DIR/$scene.ppm"; then
        echo "DIFF PASS $scene"
    else
        echo "DIFF FAIL $scene: inspect $frame and $log; reference remains unchanged"
        fails=$((fails + 1))
    fi
done
if [[ $fails -eq 0 ]]; then
    echo "GOLDEN RESULT: PASS (${#SCENES[@]} scenes)"
    exit 0
fi
echo "GOLDEN RESULT: FAIL ($fails failed checks); review intentional differences, do not repin blindly"
exit 1
