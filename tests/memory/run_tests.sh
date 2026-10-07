#!/usr/bin/env bash
# Memory-slice native tests (P11 asset service).
#
# Builds src/memory/mem_assets.cpp + the assert-based suite with the NATIVE
# host g++ (no Pico SDK, no cross compiler) and runs it. The backing store is
# a plain SRAM byte buffer implementing the same AssetBacking ops interface
# the QMI backend implements on target — no fake QMI device.
#
# Usage (from the repo root or anywhere else):
#   ./tests/memory/run_tests.sh
#
# Exit code: 0 if the build and all tests pass, 1 otherwise.

set -u

SCRIPT_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
REPO_ROOT="$(cd "$SCRIPT_DIR/../.." && pwd)"
BUILD_DIR="${BUILD_DIR:-$REPO_ROOT/build/memory-tests}"
CXX="${CXX:-g++}"

CXXFLAGS="-std=gnu++17 -Wall -Wextra -O1 -g"

echo "Memory asset-service native tests"
echo "  compiler: $CXX"
echo "  build:    $BUILD_DIR"
echo

mkdir -p "$BUILD_DIR"

if ! "$CXX" $CXXFLAGS -I "$REPO_ROOT/src" \
        "$REPO_ROOT/src/memory/mem_assets.cpp" \
        "$SCRIPT_DIR/test_asset_service.cpp" \
        -o "$BUILD_DIR/mem_asset_tests"; then
    echo "RESULT: FAIL (build error)"
    exit 1
fi

echo "build ok, running..."
echo

"$BUILD_DIR/mem_asset_tests"
status=$?

echo
if [ "$status" -eq 0 ]; then
    exit 0
fi
if [ "$status" -ne 1 ]; then
    echo "RESULT: FAIL (test binary exited with status $status)"
fi
exit 1
