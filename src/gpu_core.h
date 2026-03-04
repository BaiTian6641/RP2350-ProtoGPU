/**
 * @file gpu_core.h
 * @brief GPU core lifecycle — initialization, Core 0 loop, Core 1 loop.
 *
 * Core 0: owns SPI receive, command parsing, scene state, QuadTree build,
 *         tile rasterisation (work-stealing), performs framebuffer swap.
 * Core 1: tile rasterisation (work-stealing, FIFO-triggered).
 *
 * Inter-core dispatch is handled by PglTileScheduler, which uses the
 * multicore FIFO for start/stop signalling and a lock-free atomic counter
 * for tile work-stealing between the two cores.
 */

#pragma once

#include <cstdint>

// Forward declarations for self-test VRAM access
class OpiPsramDriver;
class QspiPsramDriver;

namespace GpuCore {

// ─── Runtime VRAM Mode ──────────────────────────────────────────────────────

/// Detected VRAM configuration — determined at boot after probing.
enum class VramMode : uint8_t {
    SRAM_ONLY = 0,    ///< No external VRAM detected — pure internal SRAM
    OPI_ONLY  = 1,    ///< PIO2 external memory only (OPI PSRAM or QSPI MRAM)
    QSPI_ONLY = 2,    ///< QSPI CS1 memory only (XIP-mapped)
    DUAL_VRAM = 3,    ///< Both PIO2 + QSPI CS1 present
};

/**
 * @brief Initialize all GPU subsystems (called on Core 0 before Core 1 launch).
 *
 * Sets up:
 *  - HUB75 PIO + DMA (display driver)
 *  - Octal SPI PIO + DMA (data receiver)
 *  - I2C slave (configuration bus)
 *  - RDY GPIO (flow control)
 *  - Framebuffers (double-buffered RGB565)
 *  - Scene state (resource tables)
 *  - Tile scheduler (PglTileScheduler)
 *  - External VRAM probe + VRAMless fallback
 *
 * @return true on success.
 */
bool Initialize();

/**
 * @brief Core 0 main loop (never returns).
 *
 * Each iteration:
 *  1. Check SPI ring buffer for new frame data
 *  2. Parse command buffer → update scene state
 *  3. Transform vertices → project → build QuadTree
 *  4. Dispatch tile rasterisation (32 tiles, work-stealing between cores)
 *  5. Wait for both cores to finish (polling HUB75 refresh while idle)
 *  6. Apply screen-space post-processing shaders
 *  7. Swap framebuffer pointers
 *  8. Update RDY pin / FPS counter
 */
void Core0Main();

/**
 * @brief Core 1 main loop (never returns).
 *
 * Delegates to PglTileScheduler::Core1Main(), which waits on the multicore
 * FIFO for tile-pass or pair commands.  During a tile pass, Core 1 pulls
 * tiles from the shared atomic counter and rasterises them.
 */
void Core1Main();

/**
 * @brief Get the runtime-detected VRAM mode.
 *
 * Returns SRAM_ONLY when no external memory was detected at boot (VRAMless).
 * Can be called after Initialize().
 */
VramMode GetVramMode();

/**
 * @brief Wait for host SPI data to arrive.
 *
 * Polls the SPI ring buffer while refreshing the HUB75 display.
 * Returns true as soon as any SPI data is detected, or false on timeout.
 *
 * @param timeoutMs  Maximum wait time in milliseconds.
 * @return true if host data was detected within the timeout.
 */
bool WaitForHost(uint32_t timeoutMs);

/**
 * @brief Run the GPU self-test suite (no host required).
 *
 * Executes math validation, VRAM memory tests (if present), and display
 * pattern verification.  Results are shown on HUB75 as colored status bars
 * and printed to serial.
 *
 * Enters a display-hold loop after tests complete. Returns automatically
 * when host SPI data is detected (allowing normal boot to continue).
 */
void RunSelfTest();

/**
 * @brief Get pointers to the initialized VRAM drivers (for self-test).
 *
 * @param opiOut   Receives the OPI driver pointer (nullptr if not initialized).
 * @param qspiOut  Receives the QSPI driver pointer (nullptr if not initialized).
 */
void GetVramDrivers(OpiPsramDriver** opiOut, QspiPsramDriver** qspiOut);

}  // namespace GpuCore
