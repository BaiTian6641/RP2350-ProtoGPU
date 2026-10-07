#!/usr/bin/env bash
# Native output encoder/timing/bounds regressions for all physical backends.
# PIO/DMA hardware execution is not simulated by these tests.
#
# Usage (from the repo root or anywhere else):
#   ./tests/display/run_tests.sh
#
# Exit code: 0 = build + run green, 1 = build or test failure.

set -u

SCRIPT_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
REPO_ROOT="$(cd "$SCRIPT_DIR/../.." && pwd)"
BUILD_DIR="${BUILD_DIR:-$REPO_ROOT/build/display-tests}"
CXX="${CXX:-g++}"

CXXFLAGS="-std=gnu++17 -Wall -Wextra -O1 -g"

echo "Display output-backend native tests"
echo "  compiler: $CXX"
echo "  build:    $BUILD_DIR"
echo

mkdir -p "$BUILD_DIR"

for suite in output_encoders led_array custom_array; do
    sources=("$SCRIPT_DIR/test_${suite}.cpp")
    if [ "$suite" != output_encoders ]; then
        sources+=("$REPO_ROOT/src/display/${suite}.cpp")
    fi
    if ! "$CXX" $CXXFLAGS \
            -I "$REPO_ROOT/src" \
            -I "$REPO_ROOT/ProtoGL/src" \
            "${sources[@]}" \
            -o "$BUILD_DIR/test_${suite}"; then
        echo "RESULT: FAIL ($suite build error)"
        exit 1
    fi
    if ! "$BUILD_DIR/test_${suite}"; then
        echo "RESULT: FAIL ($suite)"
        exit 1
    fi
done
