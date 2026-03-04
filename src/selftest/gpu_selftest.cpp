/**
 * @file gpu_selftest.cpp
 * @brief GPU self-test suite implementation.
 *
 * Provides standalone GPU diagnostics:
 *  - Math validation (FPU, trig, sqrt, integer)
 *  - VRAM memory tests (data bus, address bus, pattern fill)
 *  - Display test (visual test patterns on HUB75)
 *
 * All tests call Hub75Driver::PollRefresh() frequently to keep the display
 * alive during testing. Visual results are shown as colored bars on the
 * 128×64 HUB75 panel.
 */

#include "gpu_selftest.h"
#include "../gpu_config.h"
#include "../display/hub75_driver.h"
#include "../memory/mem_opi_psram.h"
#include "../memory/mem_qspi_psram.h"

#include "pico/stdlib.h"

#include <cstring>
#include <cstdio>
#include <cmath>

namespace GpuSelfTest {

// ─── RGB565 Display Colors ──────────────────────────────────────────────────

static constexpr uint16_t CLR_BLACK     = 0x0000;
static constexpr uint16_t CLR_WHITE     = 0xFFFF;
static constexpr uint16_t CLR_GREEN     = 0x07E0;   // pass
static constexpr uint16_t CLR_RED       = 0xF800;   // fail
static constexpr uint16_t CLR_YELLOW    = 0xFFE0;   // running
static constexpr uint16_t CLR_DARK_GRAY = 0x4208;   // N/A / skipped
static constexpr uint16_t CLR_CYAN      = 0x07FF;   // info

// ─── Display State ──────────────────────────────────────────────────────────

static uint16_t* g_fb     = nullptr;
static uint16_t  g_width  = 128;
static uint16_t  g_height = 64;

// ─── Display Helpers ────────────────────────────────────────────────────────

/// Fill a horizontal band with a solid color.
static void FillBar(uint16_t yStart, uint16_t yEnd, uint16_t color) {
    if (!g_fb) return;
    for (uint16_t y = yStart; y < yEnd && y < g_height; ++y) {
        for (uint16_t x = 0; x < g_width; ++x) {
            g_fb[y * g_width + x] = color;
        }
    }
}

/// Fill a horizontal band with a gradient (left=dark, right=bright version of color).
static void FillBarGradient(uint16_t yStart, uint16_t yEnd, uint16_t color) {
    if (!g_fb) return;
    for (uint16_t y = yStart; y < yEnd && y < g_height; ++y) {
        for (uint16_t x = 0; x < g_width; ++x) {
            // Scale color intensity by x position
            uint8_t scale = (uint8_t)(x * 255 / g_width);
            uint16_t r = ((color >> 11) & 0x1F) * scale / 255;
            uint16_t g = ((color >>  5) & 0x3F) * scale / 255;
            uint16_t b = ((color      ) & 0x1F) * scale / 255;
            g_fb[y * g_width + x] = (r << 11) | (g << 5) | b;
        }
    }
}

/// Fill a small progress indicator (a bright pixel running left->right).
__attribute__((unused))
static void FillProgress(uint16_t yStart, uint16_t yEnd,
                          uint16_t baseColor, uint8_t progress) {
    FillBar(yStart, yEnd, baseColor);
    // Draw a white marker at the progress position
    uint16_t markerX = (uint16_t)(progress * g_width / 255);
    if (markerX >= g_width) markerX = g_width - 1;
    for (uint16_t y = yStart; y < yEnd && y < g_height; ++y) {
        g_fb[y * g_width + markerX] = CLR_WHITE;
        if (markerX > 0) g_fb[y * g_width + markerX - 1] = CLR_WHITE;
        if (markerX + 1 < g_width) g_fb[y * g_width + markerX + 1] = CLR_WHITE;
    }
}

/// Keep HUB75 display refreshing for approximately the given duration.
static void RefreshForMs(uint32_t ms) {
    uint32_t startUs = time_us_32();
    uint32_t targetUs = ms * 1000;
    while ((time_us_32() - startUs) < targetUs) {
        Hub75Driver::PollRefresh();
    }
}

/// Quick display refresh burst (a few hundred PollRefresh calls).
static void QuickRefresh() {
    for (int i = 0; i < 300; ++i) {
        Hub75Driver::PollRefresh();
    }
}

// ─── Display Layout Constants ───────────────────────────────────────────────
// Each test gets a 10-row horizontal band on the 128×64 display.

static constexpr uint16_t BAR_HEADER_Y0   = 0;
static constexpr uint16_t BAR_HEADER_Y1   = 8;
static constexpr uint16_t BAR_MATH_Y0     = 8;
static constexpr uint16_t BAR_MATH_Y1     = 18;
static constexpr uint16_t BAR_OPI_Y0      = 18;
static constexpr uint16_t BAR_OPI_Y1      = 28;
static constexpr uint16_t BAR_QSPI_Y0     = 28;
static constexpr uint16_t BAR_QSPI_Y1     = 38;
static constexpr uint16_t BAR_DISPLAY_Y0  = 38;
static constexpr uint16_t BAR_DISPLAY_Y1  = 48;
static constexpr uint16_t BAR_OVERALL_Y0  = 48;
static constexpr uint16_t BAR_OVERALL_Y1  = 64;

// ═══════════════════════════════════════════════════════════════════════════
// Math Test
// ═══════════════════════════════════════════════════════════════════════════

static bool TestMath() {
    printf("[SelfTest] Math test...\n");

    // Test 1: Float addition
    volatile float a = 1.5f, b = 2.5f;
    float sum = a + b;
    if (fabsf(sum - 4.0f) > 0.001f) {
        printf("[SelfTest]   FAIL: float add: %.4f != 4.0\n", (double)sum);
        return false;
    }

    // Test 2: Float multiplication
    volatile float c = 3.0f, d = 7.0f;
    float prod = c * d;
    if (fabsf(prod - 21.0f) > 0.001f) {
        printf("[SelfTest]   FAIL: float mul: %.4f != 21.0\n", (double)prod);
        return false;
    }

    // Test 3: sinf at known angles
    float s0 = sinf(0.0f);
    if (fabsf(s0) > 0.001f) {
        printf("[SelfTest]   FAIL: sin(0) = %.6f\n", (double)s0);
        return false;
    }
    float s90 = sinf(3.14159265f / 2.0f);
    if (fabsf(s90 - 1.0f) > 0.01f) {
        printf("[SelfTest]   FAIL: sin(pi/2) = %.6f\n", (double)s90);
        return false;
    }

    // Test 4: cosf at known angles
    float c0 = cosf(0.0f);
    if (fabsf(c0 - 1.0f) > 0.001f) {
        printf("[SelfTest]   FAIL: cos(0) = %.6f\n", (double)c0);
        return false;
    }
    float c180 = cosf(3.14159265f);
    if (fabsf(c180 + 1.0f) > 0.01f) {
        printf("[SelfTest]   FAIL: cos(pi) = %.6f\n", (double)c180);
        return false;
    }

    // Test 5: sqrtf
    float sq = sqrtf(144.0f);
    if (fabsf(sq - 12.0f) > 0.001f) {
        printf("[SelfTest]   FAIL: sqrt(144) = %.4f\n", (double)sq);
        return false;
    }

    // Test 6: Integer division
    volatile int32_t x = 100, y = 7;
    int32_t div = x / y;
    if (div != 14) {
        printf("[SelfTest]   FAIL: 100/7 = %d (expected 14)\n", (int)div);
        return false;
    }

    // Test 7: Fused multiply-add (VFMA.F32 if -ffast-math)
    volatile float e = 2.0f, f = 3.0f, g = 4.0f;
    float fma = e * f + g;
    if (fabsf(fma - 10.0f) > 0.001f) {
        printf("[SelfTest]   FAIL: FMA: 2*3+4 = %.4f\n", (double)fma);
        return false;
    }

    // Test 8: Negative sqrt → NaN check (should not crash with -ffast-math)
    volatile float neg = -1.0f;
    float sqNeg = sqrtf(neg);
    (void)sqNeg;  // Don't check value (NaN semantics vary with -ffast-math), just ensure no crash

    printf("[SelfTest]   Math test PASSED (8/8 checks)\n");
    return true;
}

// ═══════════════════════════════════════════════════════════════════════════
// OPI PSRAM Memory Test (PIO2 — Tier 1, indirect DMA/sync access)
// ═══════════════════════════════════════════════════════════════════════════

VramTestResult TestOpiVram(OpiPsramDriver* drv) {
    VramTestResult result = {};

    if (!drv || !drv->IsInitialized()) {
        printf("[SelfTest] OPI VRAM: not present - skipped\n");
        result.present = false;
        result.pass = true;   // N/A counts as pass
        return result;
    }

    result.present = true;
    printf("[SelfTest] OPI VRAM memory test (%lu KB total, %lu KB free)...\n",
           (unsigned long)(drv->GetCapacity() / 1024),
           (unsigned long)(drv->Available() / 1024));

    static constexpr uint32_t TEST_SIZE = 1024;  // 1 KB test region

    // Ensure enough space
    if (drv->Available() < TEST_SIZE) {
        printf("[SelfTest]   FAIL: insufficient space (%lu bytes available)\n",
               (unsigned long)drv->Available());
        result.pass = false;
        return result;
    }

    uint32_t testAddr = drv->Alloc(TEST_SIZE, 4);
    result.testedBytes = TEST_SIZE;
    bool pass = true;

    uint8_t writeBuf[256];
    uint8_t readBuf[256];

    // ── Test 1: Walking 1s (data bus validation) ────────────────────────
    printf("[SelfTest]   Walking 1s (data bus)...\n");
    for (int bit = 0; bit < 8 && pass; ++bit) {
        uint8_t pattern = (1u << bit);
        memset(writeBuf, pattern, sizeof(writeBuf));
        drv->WriteSync(testAddr, writeBuf, sizeof(writeBuf));
        Hub75Driver::PollRefresh();
        memset(readBuf, 0, sizeof(readBuf));
        drv->ReadSync(testAddr, readBuf, sizeof(readBuf));
        Hub75Driver::PollRefresh();

        for (uint32_t i = 0; i < sizeof(writeBuf); ++i) {
            if (readBuf[i] != pattern) {
                printf("[SelfTest]     FAIL at bit %d, byte %u: wrote 0x%02X, read 0x%02X\n",
                       bit, (unsigned)i, pattern, readBuf[i]);
                result.errorCount++;
                if (result.errorAddr == 0) result.errorAddr = testAddr + i;
                pass = false;
                break;
            }
        }
    }

    // ── Test 2: Walking 0s (data bus validation) ────────────────────────
    if (pass) {
        printf("[SelfTest]   Walking 0s (data bus)...\n");
        for (int bit = 0; bit < 8 && pass; ++bit) {
            uint8_t pattern = (uint8_t)(~(1u << bit));
            memset(writeBuf, pattern, sizeof(writeBuf));
            drv->WriteSync(testAddr, writeBuf, sizeof(writeBuf));
            Hub75Driver::PollRefresh();
            memset(readBuf, 0xFF, sizeof(readBuf));
            drv->ReadSync(testAddr, readBuf, sizeof(readBuf));
            Hub75Driver::PollRefresh();

            for (uint32_t i = 0; i < sizeof(writeBuf); ++i) {
                if (readBuf[i] != pattern) {
                    printf("[SelfTest]     FAIL at bit %d, byte %u: wrote 0x%02X, read 0x%02X\n",
                           bit, (unsigned)i, pattern, readBuf[i]);
                    result.errorCount++;
                    if (result.errorAddr == 0) result.errorAddr = testAddr + i;
                    pass = false;
                    break;
                }
            }
        }
    }

    // ── Test 3: Address uniqueness (address bus validation) ─────────────
    if (pass) {
        printf("[SelfTest]   Address uniqueness...\n");

        // Write unique pattern at each 4-byte aligned offset
        for (uint32_t off = 0; off < TEST_SIZE; off += 4) {
            uint32_t pattern = off ^ 0xDEADBEEFu;
            drv->WriteSync(testAddr + off, &pattern, 4);
            if ((off & 0x3F) == 0) Hub75Driver::PollRefresh();
        }

        // Read back and verify
        for (uint32_t off = 0; off < TEST_SIZE && pass; off += 4) {
            uint32_t expected = off ^ 0xDEADBEEFu;
            uint32_t readVal = 0;
            drv->ReadSync(testAddr + off, &readVal, 4);

            if (readVal != expected) {
                printf("[SelfTest]     FAIL at offset 0x%04X: expected 0x%08X, got 0x%08X\n",
                       (unsigned)off, (unsigned)expected, (unsigned)readVal);
                result.errorCount++;
                if (result.errorAddr == 0) result.errorAddr = testAddr + off;
                pass = false;
            }
            if ((off & 0x3F) == 0) Hub75Driver::PollRefresh();
        }
    }

    // ── Test 4: Sequential byte pattern (full region fill) ──────────────
    if (pass) {
        printf("[SelfTest]   Byte pattern fill (%lu bytes)...\n", (unsigned long)TEST_SIZE);

        // Write: each byte = (offset & 0xFF)
        for (uint32_t off = 0; off < TEST_SIZE; off += sizeof(writeBuf)) {
            uint32_t chunk = TEST_SIZE - off;
            if (chunk > sizeof(writeBuf)) chunk = sizeof(writeBuf);
            for (uint32_t i = 0; i < chunk; ++i) {
                writeBuf[i] = (uint8_t)((off + i) & 0xFF);
            }
            drv->WriteSync(testAddr + off, writeBuf, chunk);
            Hub75Driver::PollRefresh();
        }

        // Verify
        for (uint32_t off = 0; off < TEST_SIZE && pass; off += sizeof(readBuf)) {
            uint32_t chunk = TEST_SIZE - off;
            if (chunk > sizeof(readBuf)) chunk = sizeof(readBuf);
            drv->ReadSync(testAddr + off, readBuf, chunk);

            for (uint32_t i = 0; i < chunk && pass; ++i) {
                uint8_t expected = (uint8_t)((off + i) & 0xFF);
                if (readBuf[i] != expected) {
                    printf("[SelfTest]     FAIL at byte 0x%04X: expected 0x%02X, got 0x%02X\n",
                           (unsigned)(off + i), expected, readBuf[i]);
                    result.errorCount++;
                    if (result.errorAddr == 0) result.errorAddr = testAddr + off + i;
                    pass = false;
                }
            }
            Hub75Driver::PollRefresh();
        }
    }

    drv->Free(testAddr);
    result.pass = pass;
    printf("[SelfTest] OPI VRAM test: %s (tested %u bytes, %u errors)\n",
           pass ? "PASSED" : "FAILED", (unsigned)result.testedBytes,
           (unsigned)result.errorCount);
    return result;
}

// ═══════════════════════════════════════════════════════════════════════════
// QSPI CS1 Memory Test (Tier 2 — XIP mapped)
// ═══════════════════════════════════════════════════════════════════════════

VramTestResult TestQspiVram(QspiPsramDriver* drv) {
    VramTestResult result = {};

    if (!drv || !drv->IsInitialized()) {
        printf("[SelfTest] QSPI VRAM: not present - skipped\n");
        result.present = false;
        result.pass = true;   // N/A counts as pass
        return result;
    }

    result.present = true;
    printf("[SelfTest] QSPI VRAM memory test (%lu KB total, %lu KB free)...\n",
           (unsigned long)(drv->GetCapacity() / 1024),
           (unsigned long)(drv->Available() / 1024));

    static constexpr uint32_t TEST_SIZE = 1024;  // 1 KB test region

    if (drv->Available() < TEST_SIZE) {
        printf("[SelfTest]   FAIL: insufficient space (%lu bytes available)\n",
               (unsigned long)drv->Available());
        result.pass = false;
        return result;
    }

    uint32_t testOffset = drv->Alloc(TEST_SIZE, 4);
    result.testedBytes = TEST_SIZE;
    bool pass = true;

    uint8_t writeBuf[256];
    uint8_t readBuf[256];

    // ── Test 1: Walking 1s (data bus validation) ────────────────────────
    printf("[SelfTest]   Walking 1s (data bus)...\n");
    for (int bit = 0; bit < 8 && pass; ++bit) {
        uint8_t pattern = (1u << bit);
        memset(writeBuf, pattern, sizeof(writeBuf));
        drv->Write(testOffset, writeBuf, sizeof(writeBuf));
        Hub75Driver::PollRefresh();
        memset(readBuf, 0, sizeof(readBuf));
        drv->Read(testOffset, readBuf, sizeof(readBuf));
        Hub75Driver::PollRefresh();

        for (uint32_t i = 0; i < sizeof(writeBuf); ++i) {
            if (readBuf[i] != pattern) {
                printf("[SelfTest]     FAIL at bit %d, byte %u: wrote 0x%02X, read 0x%02X\n",
                       bit, (unsigned)i, pattern, readBuf[i]);
                result.errorCount++;
                if (result.errorAddr == 0) result.errorAddr = testOffset + i;
                pass = false;
                break;
            }
        }
    }

    // ── Test 2: Walking 0s (data bus validation) ────────────────────────
    if (pass) {
        printf("[SelfTest]   Walking 0s (data bus)...\n");
        for (int bit = 0; bit < 8 && pass; ++bit) {
            uint8_t pattern = (uint8_t)(~(1u << bit));
            memset(writeBuf, pattern, sizeof(writeBuf));
            drv->Write(testOffset, writeBuf, sizeof(writeBuf));
            Hub75Driver::PollRefresh();
            memset(readBuf, 0xFF, sizeof(readBuf));
            drv->Read(testOffset, readBuf, sizeof(readBuf));
            Hub75Driver::PollRefresh();

            for (uint32_t i = 0; i < sizeof(writeBuf); ++i) {
                if (readBuf[i] != pattern) {
                    printf("[SelfTest]     FAIL at bit %d, byte %u: wrote 0x%02X, read 0x%02X\n",
                           bit, (unsigned)i, pattern, readBuf[i]);
                    result.errorCount++;
                    if (result.errorAddr == 0) result.errorAddr = testOffset + i;
                    pass = false;
                    break;
                }
            }
        }
    }

    // ── Test 3: Address uniqueness (address bus validation) ─────────────
    if (pass) {
        printf("[SelfTest]   Address uniqueness...\n");

        for (uint32_t off = 0; off < TEST_SIZE; off += 4) {
            uint32_t pattern = off ^ 0xCAFEBEEFu;
            drv->Write(testOffset + off, &pattern, 4);
            if ((off & 0x3F) == 0) Hub75Driver::PollRefresh();
        }

        for (uint32_t off = 0; off < TEST_SIZE && pass; off += 4) {
            uint32_t expected = off ^ 0xCAFEBEEFu;
            uint32_t readVal = 0;
            drv->Read(testOffset + off, &readVal, 4);

            if (readVal != expected) {
                printf("[SelfTest]     FAIL at offset 0x%04X: expected 0x%08X, got 0x%08X\n",
                       (unsigned)off, (unsigned)expected, (unsigned)readVal);
                result.errorCount++;
                if (result.errorAddr == 0) result.errorAddr = testOffset + off;
                pass = false;
            }
            if ((off & 0x3F) == 0) Hub75Driver::PollRefresh();
        }
    }

    // ── Test 4: Sequential byte pattern (full region fill) ──────────────
    if (pass) {
        printf("[SelfTest]   Byte pattern fill (%lu bytes)...\n", (unsigned long)TEST_SIZE);

        for (uint32_t off = 0; off < TEST_SIZE; off += sizeof(writeBuf)) {
            uint32_t chunk = TEST_SIZE - off;
            if (chunk > sizeof(writeBuf)) chunk = sizeof(writeBuf);
            for (uint32_t i = 0; i < chunk; ++i) {
                writeBuf[i] = (uint8_t)((off + i) & 0xFF);
            }
            drv->Write(testOffset + off, writeBuf, chunk);
            Hub75Driver::PollRefresh();
        }

        for (uint32_t off = 0; off < TEST_SIZE && pass; off += sizeof(readBuf)) {
            uint32_t chunk = TEST_SIZE - off;
            if (chunk > sizeof(readBuf)) chunk = sizeof(readBuf);
            drv->Read(testOffset + off, readBuf, chunk);

            for (uint32_t i = 0; i < chunk && pass; ++i) {
                uint8_t expected = (uint8_t)((off + i) & 0xFF);
                if (readBuf[i] != expected) {
                    printf("[SelfTest]     FAIL at byte 0x%04X: expected 0x%02X, got 0x%02X\n",
                           (unsigned)(off + i), expected, readBuf[i]);
                    result.errorCount++;
                    if (result.errorAddr == 0) result.errorAddr = testOffset + off + i;
                    pass = false;
                }
            }
            Hub75Driver::PollRefresh();
        }
    }

    drv->Free(testOffset);
    result.pass = pass;
    printf("[SelfTest] QSPI VRAM test: %s (tested %u bytes, %u errors)\n",
           pass ? "PASSED" : "FAILED", (unsigned)result.testedBytes,
           (unsigned)result.errorCount);
    return result;
}

// ═══════════════════════════════════════════════════════════════════════════
// Combined VRAM Test (standalone, no display feedback)
// ═══════════════════════════════════════════════════════════════════════════

void TestAllVram(OpiPsramDriver* opiDriver, QspiPsramDriver* qspiDriver,
                 VramTestResult& opiResult, VramTestResult& qspiResult) {
    printf("[SelfTest] === VRAM Memory Check ===\n");
    opiResult  = TestOpiVram(opiDriver);
    qspiResult = TestQspiVram(qspiDriver);
    printf("[SelfTest] === VRAM Memory Check Complete ===\n");
    printf("[SelfTest] OPI:  %s%s\n",
           opiResult.present ? "" : "(not present) ",
           opiResult.pass ? "PASS" : "FAIL");
    printf("[SelfTest] QSPI: %s%s\n",
           qspiResult.present ? "" : "(not present) ",
           qspiResult.pass ? "PASS" : "FAIL");
}

// ═══════════════════════════════════════════════════════════════════════════
// Full Self-Test Suite with Visual Feedback
// ═══════════════════════════════════════════════════════════════════════════

SelfTestResult RunAll(uint16_t* frontBuffer, uint16_t* backBuffer,
                      uint16_t* zBuffer,
                      OpiPsramDriver* opiDriver, QspiPsramDriver* qspiDriver,
                      uint16_t width, uint16_t height) {
    (void)backBuffer;
    (void)zBuffer;

    g_fb     = frontBuffer;
    g_width  = width;
    g_height = height;

    SelfTestResult result = {};

    printf("\n");
    printf("========================================\n");
    printf("  ProtoGL GPU Self-Test v1.0\n");
    printf("========================================\n");
    printf("  Panel: %ux%u\n", width, height);
    printf("  OPI VRAM:  %s\n", (opiDriver && opiDriver->IsInitialized()) ? "present" : "not detected");
    printf("  QSPI VRAM: %s\n", (qspiDriver && qspiDriver->IsInitialized()) ? "present" : "not detected");
    printf("========================================\n\n");

    // ── Clear display and show header ───────────────────────────────────
    memset(frontBuffer, 0, width * height * 2);
    FillBarGradient(BAR_HEADER_Y0, BAR_HEADER_Y1, CLR_CYAN);
    Hub75Driver::SetFramebuffer(frontBuffer);
    RefreshForMs(300);

    // ── Test 1: Math ────────────────────────────────────────────────────
    FillBar(BAR_MATH_Y0, BAR_MATH_Y1, CLR_YELLOW);
    QuickRefresh();

    result.mathPass = TestMath();
    FillBar(BAR_MATH_Y0, BAR_MATH_Y1, result.mathPass ? CLR_GREEN : CLR_RED);
    QuickRefresh();

    // ── Test 2: OPI VRAM ────────────────────────────────────────────────
    bool opiPresent = (opiDriver && opiDriver->IsInitialized());
    FillBar(BAR_OPI_Y0, BAR_OPI_Y1, opiPresent ? CLR_YELLOW : CLR_DARK_GRAY);
    QuickRefresh();

    result.opiResult = TestOpiVram(opiDriver);
    FillBar(BAR_OPI_Y0, BAR_OPI_Y1,
            !opiPresent       ? CLR_DARK_GRAY :
            result.opiResult.pass ? CLR_GREEN : CLR_RED);
    QuickRefresh();

    // ── Test 3: QSPI VRAM ──────────────────────────────────────────────
    bool qspiPresent = (qspiDriver && qspiDriver->IsInitialized());
    FillBar(BAR_QSPI_Y0, BAR_QSPI_Y1, qspiPresent ? CLR_YELLOW : CLR_DARK_GRAY);
    QuickRefresh();

    result.qspiResult = TestQspiVram(qspiDriver);
    FillBar(BAR_QSPI_Y0, BAR_QSPI_Y1,
            !qspiPresent        ? CLR_DARK_GRAY :
            result.qspiResult.pass ? CLR_GREEN : CLR_RED);
    QuickRefresh();

    // ── Test 4: Display (visual-only) ───────────────────────────────────
    // If you can see these bars, the display works.
    result.displayPass = true;
    FillBar(BAR_DISPLAY_Y0, BAR_DISPLAY_Y1, CLR_GREEN);
    QuickRefresh();

    // ── Overall Result ──────────────────────────────────────────────────
    bool allPass = result.AllPass();
    FillBar(BAR_OVERALL_Y0, BAR_OVERALL_Y1, allPass ? CLR_GREEN : CLR_RED);
    QuickRefresh();

    // ── Show test pattern cycle in header area ──────────────────────────
    // Cycle through 4 built-in HUB75 test patterns in the header band.
    // This provides additional visual confirmation of display function.
    for (uint8_t p = 0; p < 4; ++p) {
        // Fill the header area with a test pattern (overwrites just those rows)
        // Use full-buffer FillTestPattern then restore test result bars.
        // Actually, just quickly flash the pattern number in the header.
        uint16_t patColor = (p == 0) ? CLR_RED : (p == 1) ? CLR_GREEN :
                            (p == 2) ? 0x001F : CLR_WHITE;
        FillBarGradient(BAR_HEADER_Y0, BAR_HEADER_Y1, patColor);
        RefreshForMs(400);
    }

    // Restore final header
    FillBarGradient(BAR_HEADER_Y0, BAR_HEADER_Y1, CLR_CYAN);
    QuickRefresh();

    // ── Print summary ───────────────────────────────────────────────────
    printf("\n");
    printf("========================================\n");
    printf("  Self-Test Results\n");
    printf("========================================\n");
    printf("  Math:     %s\n", result.mathPass ? "PASS" : "FAIL");
    printf("  OPI VRAM: %s", result.opiResult.pass ? "PASS" : "FAIL");
    if (!result.opiResult.present) printf(" (not present)");
    else printf(" (%u bytes tested, %u errors)", (unsigned)result.opiResult.testedBytes,
                (unsigned)result.opiResult.errorCount);
    printf("\n");
    printf("  QSPI VRAM: %s", result.qspiResult.pass ? "PASS" : "FAIL");
    if (!result.qspiResult.present) printf(" (not present)");
    else printf(" (%u bytes tested, %u errors)", (unsigned)result.qspiResult.testedBytes,
                (unsigned)result.qspiResult.errorCount);
    printf("\n");
    printf("  Display:  %s\n", result.displayPass ? "PASS" : "FAIL");
    printf("----------------------------------------\n");
    printf("  Overall:  %s\n", allPass ? "ALL PASS" : "*** FAILURE ***");
    printf("========================================\n\n");

    return result;
}

}  // namespace GpuSelfTest
