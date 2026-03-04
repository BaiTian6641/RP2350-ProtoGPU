/**
 * @file main.cpp
 * @brief ProtoGL GPU firmware entry point — RP2350 (ARM Cortex-M33 dual-core).
 *
 * Core 0: SPI receive → command parse → scene update → QuadTree build → tile raster (work-stealing)
 * Core 1: Tile raster (work-stealing, FIFO-triggered)
 *
 * Boot sequence:
 *  1. Initialize all subsystems (HUB75, SPI, I2C, VRAM probe, rasterizer)
 *  2. Wait briefly for host SPI data (~2 seconds)
 *  3. If no host detected → enter Self-Test Mode (standalone diagnostics)
 *  4. When host appears → launch Core 1 + enter normal render loop
 *
 * Self-Test Mode:
 *  - Math validation (FPU, trig, integer)
 *  - VRAM memory test (OPI + QSPI, if present)
 *  - Display test patterns on HUB75
 *  - Results shown as colored bars + serial output
 *  - Auto-exits when host SPI data is detected
 *
 * The PIO HUB75 driver runs entirely in hardware, consuming the framebuffer
 * via DMA with zero CPU intervention.
 */

#include <cstdio>
#include "pico/stdlib.h"
#include "pico/multicore.h"
#include "hardware/clocks.h"

#include "gpu_config.h"
#include "gpu_core.h"

/// Self-test wait timeout: how long to wait for host SPI data before
/// entering self-test mode. Set to 0 to skip self-test entirely.
static constexpr uint32_t SELFTEST_WAIT_MS = 2000;

// ─── Core 1 Entry Point ────────────────────────────────────────────────────

static void core1_entry() {
    GpuCore::Core1Main();
    // Should never return
    while (true) {
        tight_loop_contents();
    }
}

// ─── Core 0 Entry Point (main) ─────────────────────────────────────────────

int main() {
    // Initialize Pico SDK (stdio, clocks)
    stdio_init_all();

    printf("\n[ProtoGL GPU] RP2350 firmware starting...\n");
    printf("[ProtoGL GPU] System clock: %lu MHz\n", clock_get_hz(clk_sys) / 1000000);

    // Initialize all GPU subsystems on Core 0
    if (!GpuCore::Initialize()) {
        printf("[ProtoGL GPU] ERROR: Initialization failed!\n");
        while (true) {
            tight_loop_contents();
        }
    }

    // Report VRAM mode
    const char* vramModeStr = "unknown";
    switch (GpuCore::GetVramMode()) {
        case GpuCore::VramMode::SRAM_ONLY: vramModeStr = "SRAM_ONLY (VRAMless)"; break;
        case GpuCore::VramMode::OPI_ONLY:  vramModeStr = "OPI_ONLY"; break;
        case GpuCore::VramMode::QSPI_ONLY: vramModeStr = "QSPI_ONLY"; break;
        case GpuCore::VramMode::DUAL_VRAM: vramModeStr = "DUAL_VRAM"; break;
    }
    printf("[ProtoGL GPU] VRAM mode: %s\n", vramModeStr);
    printf("[ProtoGL GPU] Initialization complete.\n");

    // ── Self-Test / Host Detection ──────────────────────────────────────
    // Wait briefly for host SPI data. If no host is detected within the
    // timeout, enter self-test mode for standalone GPU diagnostics.
    // Self-test displays results on HUB75 and auto-exits when host appears.
    if (SELFTEST_WAIT_MS > 0 && !GpuCore::WaitForHost(SELFTEST_WAIT_MS)) {
        GpuCore::RunSelfTest();
        // RunSelfTest() returns when host SPI data is detected
        printf("[ProtoGL GPU] Exited self-test mode. Continuing to normal operation.\n");
    }

    printf("[ProtoGL GPU] Launching Core 1...\n");

    // Launch Core 1 for rasterization
    multicore_launch_core1(core1_entry);

    printf("[ProtoGL GPU] Core 1 launched. Entering main loop.\n");

    // Core 0 main loop
    GpuCore::Core0Main();

    // Should never reach here
    return 0;
}
