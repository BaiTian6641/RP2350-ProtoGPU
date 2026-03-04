/**
 * @file gpu_selftest.h
 * @brief GPU self-test suite — standalone diagnostics without host machine.
 *
 * Provides three categories of tests:
 *   1. Math validation — FPU, sinf/cosf/sqrtf, integer arithmetic
 *   2. VRAM memory test — data bus, address bus, pattern fill for OPI + QSPI
 *   3. Display test — visual feedback on HUB75 (test patterns + result bars)
 *
 * The RunAll() function orchestrates all tests with real-time visual feedback
 * on the HUB75 panel: colored bars indicate pass (green), fail (red), running
 * (yellow), or not-applicable (dark gray).
 *
 * Individual VRAM test functions can also be called independently for
 * on-demand memory validation (e.g., triggered via I2C command).
 */

#pragma once

#include <cstdint>

// Forward declarations
class OpiPsramDriver;
class QspiPsramDriver;

namespace GpuSelfTest {

// ─── Test Results ───────────────────────────────────────────────────────────

/// Result from a VRAM memory test (OPI or QSPI).
struct VramTestResult {
    bool     present;       ///< true if the VRAM hardware was detected
    bool     pass;          ///< true if all tests passed (or not present)
    uint32_t testedBytes;   ///< total bytes tested
    uint32_t errorCount;    ///< number of byte-level mismatches
    uint32_t errorAddr;     ///< address/offset of first error (0 if none)
};

/// Aggregate result from the full self-test suite.
struct SelfTestResult {
    bool           mathPass;        ///< FPU / integer math validation
    bool           displayPass;     ///< HUB75 display test (always true if visible)
    VramTestResult opiResult;       ///< OPI PSRAM (PIO2) memory test
    VramTestResult qspiResult;      ///< QSPI CS1 memory test

    bool AllPass() const {
        return mathPass && displayPass && opiResult.pass && qspiResult.pass;
    }
};

// ─── Full Self-Test Suite ───────────────────────────────────────────────────

/**
 * @brief Run the full self-test suite with visual feedback on HUB75.
 *
 * Tests are run sequentially. The HUB75 display shows colored status bars
 * for each test. Results are also printed to serial (printf).
 *
 * Display layout (128×64 panel):
 *   Rows  0– 7: White header bar (animated)
 *   Rows  8–17: Math test result
 *   Rows 18–27: OPI VRAM test result
 *   Rows 28–37: QSPI VRAM test result
 *   Rows 38–47: Display test result
 *   Rows 48–63: Overall result (bright green / red)
 *
 * @param frontBuffer  Current HUB75 display framebuffer (RGB565).
 * @param backBuffer   Back framebuffer (unused, available for scratch).
 * @param zBuffer      Z-buffer (unused, available for scratch).
 * @param opiDriver    OPI PSRAM driver (nullptr = not present → skip).
 * @param qspiDriver   QSPI driver (nullptr = not present → skip).
 * @param width        Panel width in pixels.
 * @param height       Panel height in pixels.
 * @return Aggregate test results.
 */
SelfTestResult RunAll(uint16_t* frontBuffer, uint16_t* backBuffer,
                      uint16_t* zBuffer,
                      OpiPsramDriver* opiDriver, QspiPsramDriver* qspiDriver,
                      uint16_t width, uint16_t height);

// ─── Individual VRAM Tests ──────────────────────────────────────────────────

/**
 * @brief Run memory test on OPI PSRAM (PIO2 external memory).
 *
 * Tests: walking 1s/0s (data bus), address uniqueness, byte pattern fill.
 * Allocates a 1 KB test region, runs tests, then frees it.
 *
 * @param driver  OPI PSRAM driver (nullptr → returns "not present" result).
 * @return Test result with error details.
 */
VramTestResult TestOpiVram(OpiPsramDriver* driver);

/**
 * @brief Run memory test on QSPI CS1 memory (XIP-mapped).
 *
 * Tests: walking 1s/0s (data bus), address uniqueness, byte pattern fill.
 * Allocates a 1 KB test region, runs tests, then frees it.
 *
 * @param driver  QSPI driver (nullptr → returns "not present" result).
 * @return Test result with error details.
 */
VramTestResult TestQspiVram(QspiPsramDriver* driver);

/**
 * @brief Run both OPI and QSPI VRAM tests.
 *
 * Convenience wrapper for running both memory tests without the full
 * self-test suite (no display feedback, serial output only).
 *
 * @param opiDriver   OPI PSRAM driver (nullable).
 * @param qspiDriver  QSPI driver (nullable).
 * @param opiResult   Output: OPI test result.
 * @param qspiResult  Output: QSPI test result.
 */
void TestAllVram(OpiPsramDriver* opiDriver, QspiPsramDriver* qspiDriver,
                 VramTestResult& opiResult, VramTestResult& qspiResult);

}  // namespace GpuSelfTest
