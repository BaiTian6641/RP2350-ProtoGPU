/**
 * @file gpu_core.h
 * @brief GPU core lifecycle — initialization, Core 0 loop, Core 1 loop.
 *
 * Core 0: owns SPI receive, command parsing, scene state, QuadTree build,
 *         rasterizes top half of screen, performs framebuffer swap.
 * Core 1: waits for FIFO signal, rasterizes bottom half, signals completion.
 */

#pragma once

#include <cstdint>

namespace GpuCore {

/// Multi-core FIFO command words
static constexpr uint32_t FIFO_CMD_START_RENDER = 0x52454E44;  // "REND"
static constexpr uint32_t FIFO_CMD_RENDER_DONE  = 0x444F4E45;  // "DONE"

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
 *  4. Signal Core 1 to start rasterizing bottom half
 *  5. Rasterize top half (Y = 0 to PANEL_HEIGHT/2 - 1)
 *  6. Wait for Core 1 completion
 *  7. Swap framebuffer pointers
 *  8. Update RDY pin / FPS counter
 */
void Core0Main();

/**
 * @brief Core 1 main loop (never returns).
 *
 * Spins on multicore FIFO waiting for FIFO_CMD_START_RENDER.
 * When received:
 *  1. Rasterize bottom half (Y = PANEL_HEIGHT/2 to PANEL_HEIGHT - 1)
 *  2. Push FIFO_CMD_RENDER_DONE back to Core 0
 */
void Core1Main();

}  // namespace GpuCore
