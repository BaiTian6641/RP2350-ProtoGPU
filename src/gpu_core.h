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

namespace GpuCore {

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

}  // namespace GpuCore
