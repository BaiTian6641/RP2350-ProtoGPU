/**
 * @file main.cpp
 * @brief ProtoGL GPU firmware entry point — RP2350 (ARM Cortex-M33 dual-core).
 *
 * Core 0: SPI receive → command parse → scene update → QuadTree build → raster top half
 * Core 1: Raster bottom half (launched after QuadTree is ready)
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
    printf("[ProtoGL GPU] System clock: %u MHz\n", clock_get_hz(clk_sys) / 1000000);

    // Initialize all GPU subsystems on Core 0
    if (!GpuCore::Initialize()) {
        printf("[ProtoGL GPU] ERROR: Initialization failed!\n");
        while (true) {
            tight_loop_contents();
        }
    }

    printf("[ProtoGL GPU] Initialization complete. Launching Core 1...\n");

    // Launch Core 1 for rasterization
    multicore_launch_core1(core1_entry);

    printf("[ProtoGL GPU] Core 1 launched. Entering main loop.\n");

    // Core 0 main loop
    GpuCore::Core0Main();

    // Should never reach here
    return 0;
}
