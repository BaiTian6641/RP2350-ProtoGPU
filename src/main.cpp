/**
 * @file main.cpp
 * @brief ProtoGL GPU firmware entry point — RP2350 (ARM Cortex-M33 dual-core).
 *
 * Both normal and headless self-test modes share the same entry flow:
 *   1. stdio_init_all()
 *   2. GpuCore::Initialize()  — sets up the shared ProtoGL pipeline
 *   3. multicore_launch_core1()  — tile scheduler work-stealing loop
 *   4. GpuCore::Core0Main()  — render loop (never returns)
 *
 * The difference between modes lives inside gpu_core.cpp:
 *   Normal:   HUB75 display,  scene from SPI host,  VRAM tiering
 *   Headless: SSD1331 OLED,   built-in scene,       SRAM-only
 */

#include <cstdio>
#include "pico/stdlib.h"
#include "pico/multicore.h"
#include "hardware/clocks.h"
#include "gpu_config.h"
#include "gpu_core.h"
#include "gpu_clock.h"

/// Core 1 entry — delegates to tile scheduler (never returns).
static void core1_entry() {
    GpuCore::Core1Main();
    while (true) { tight_loop_contents(); }
}

int main() {
    stdio_init_all();

#ifdef RP2350GPU_HEADLESS_SELFTEST
    // Headless self-test: boost core/system clock for stress/performance testing.
    // Keep peripheral timing unchanged (SPI/I2C stay on clk_peri domain).
    // We also skip PIO retiming since headless path doesn't use HUB75/Octal-PI0.
    GpuClock::Initialize();
    const uint16_t headlessTargetMHz = 360;
    const uint16_t actualMHz = GpuClock::SetFrequency(headlessTargetMHz, 0, false);
    printf("[ProtoGL GPU] Headless clock target: %u MHz, actual: %u MHz\n",
           headlessTargetMHz, actualMHz);
#endif

    printf("\n[ProtoGL GPU] RP2350 firmware starting...\n");
    printf("[ProtoGL GPU] System clock: %lu MHz\n",
           clock_get_hz(clk_sys) / 1000000);

#ifdef RP2350GPU_HEADLESS_SELFTEST
    printf("[ProtoGL GPU] *** HEADLESS SELF-TEST MODE ***\n");
    printf("[ProtoGL GPU] Pipeline: %ux%u render (tile raster) -> center-crop -> SSD1331 %ux%u\n",
           GpuConfig::PANEL_WIDTH, GpuConfig::PANEL_HEIGHT,
           GpuConfig::SSD1331_WIDTH, GpuConfig::SSD1331_HEIGHT);
    printf("[ProtoGL GPU] SSD1331 on SPI0: CS=GPIO%u SCK=GPIO%u MOSI=GPIO%u DC=GPIO%u RST=GPIO%u\n",
           GpuConfig::SSD1331_CS_PIN, GpuConfig::SSD1331_SCK_PIN,
           GpuConfig::SSD1331_MOSI_PIN, GpuConfig::SSD1331_DC_PIN,
           GpuConfig::SSD1331_RST_PIN);
#endif

    // Initialize all GPU subsystems (shared pipeline + mode-specific I/O)
    if (!GpuCore::Initialize()) {
        printf("[ProtoGL GPU] ERROR: Initialization failed!\n");
        while (true) { tight_loop_contents(); }
    }

    // Report VRAM mode
    const char* vramStr = "unknown";
    switch (GpuCore::GetVramMode()) {
        case GpuCore::VramMode::SRAM_ONLY: vramStr = "SRAM_ONLY (VRAMless)"; break;
        case GpuCore::VramMode::OPI_ONLY:  vramStr = "OPI_ONLY"; break;
        case GpuCore::VramMode::QSPI_ONLY: vramStr = "QSPI_ONLY"; break;
        case GpuCore::VramMode::DUAL_VRAM: vramStr = "DUAL_VRAM"; break;
    }
    printf("[ProtoGL GPU] VRAM mode: %s\n", vramStr);
    printf("[ProtoGL GPU] Initialization complete.\n");

    // Normal mode: wait for host, run HUB75 self-test if no host detected
#ifndef RP2350GPU_HEADLESS_SELFTEST
    {
        static constexpr uint32_t SELFTEST_WAIT_MS = 2000;
        if (SELFTEST_WAIT_MS > 0 && !GpuCore::WaitForHost(SELFTEST_WAIT_MS)) {
            GpuCore::RunSelfTest();
            printf("[ProtoGL GPU] Exited self-test mode. Continuing to normal operation.\n");
        }
    }
#endif

    // Launch Core 1 — enters tile scheduler work-stealing loop
    printf("[ProtoGL GPU] Launching Core 1...\n");
    multicore_launch_core1(core1_entry);
    printf("[ProtoGL GPU] Core 1 launched. Entering main loop.\n");

    // Core 0 main loop (never returns)
    GpuCore::Core0Main();
    return 0;
}
