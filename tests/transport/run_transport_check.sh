#!/usr/bin/env bash
# PglSpiTarget deterministic transport-state check (P06).
#
# Compiles the REAL pure transport logic from src/transport/pgl_spi_target.h
# (Detail namespace: sentinel/absorb decode, commit/reject decision, MSB
# prepack, double-buffered snapshot activation — the same structures the CS
# IRQ produces on hardware) and runs the behavior matrix in
# check_spi_target.cpp. No hardware, no mocks of this code under test.
#
# Usage: ./tests/transport/run_transport_check.sh
# Exit code: 0 = build + run green, 1 = build or test failure.

set -u

SCRIPT_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
REPO_ROOT="$(cd "$SCRIPT_DIR/../.." && pwd)"
BUILD_DIR="${BUILD_DIR:-$REPO_ROOT/build/transport-check}"
CXX="${CXX:-g++}"

mkdir -p "$BUILD_DIR"

echo "SPI target transport-state check"
echo "  compiler: $CXX"

set -e
"$CXX" -std=gnu++17 -Wall -Wextra -O1 -g \
    -I "$REPO_ROOT/src" \
    -I "$REPO_ROOT/ProtoGL/src" \
    "$SCRIPT_DIR/check_spi_target.cpp" \
    -o "$BUILD_DIR/check_spi_target"
set +e

echo "build ok, running..."
echo
"$BUILD_DIR/check_spi_target"
rc=$?
exit $rc
