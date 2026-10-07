#!/usr/bin/env bash
# Device-slice native tests (P08 allowlisted device service).
#
# Builds src/devices/device_service.cpp + the assert-based suite with the
# NATIVE host g++ (no Pico SDK, no cross compiler) and runs it. The HAL is a
# deterministic register-file/pad stand-in (bus NACK/timeout semantics), not
# a mock echo; success payloads on the real 0x3D peripheral and electrical
# limits remain HIL acceptance.
#
# Usage (from the repo root or anywhere else):
#   ./tests/devices/run_tests.sh
#
# Exit code: 0 if the build and all tests pass, 1 otherwise.

set -u

SCRIPT_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
REPO_ROOT="$(cd "$SCRIPT_DIR/../.." && pwd)"
BUILD_DIR="${BUILD_DIR:-$REPO_ROOT/build/device-tests}"
CXX="${CXX:-g++}"

CXXFLAGS="-std=gnu++17 -Wall -Wextra -O1 -g"

echo "Device service native tests"
echo "  compiler: $CXX"
echo "  build:    $BUILD_DIR"
echo

mkdir -p "$BUILD_DIR"

if ! "$CXX" $CXXFLAGS -I "$REPO_ROOT/src" -I "$REPO_ROOT/ProtoGL/src" \
        "$REPO_ROOT/src/devices/device_service.cpp" \
        "$SCRIPT_DIR/test_device_service.cpp" \
        -o "$BUILD_DIR/device_service_tests"; then
    echo "RESULT: FAIL (build error)"
    exit 1
fi

echo "build ok, running..."
echo

"$BUILD_DIR/device_service_tests"
status=$?

echo
if [ "$status" -eq 0 ]; then
    exit 0
fi
if [ "$status" -ne 1 ]; then
    echo "RESULT: FAIL (test binary exited with status $status)"
fi
exit 1
