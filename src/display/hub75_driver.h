/**
 * @file hub75_driver.h
 * @brief PIO-driven HUB75 display driver for RP2350.
 *
 * Drives a HUB75 LED matrix panel using a PIO state machine and DMA.
 * Once initialized, the display refreshes automatically (zero CPU overhead).
 * The CPU only needs to update the framebuffer pointer on swap.
 *
 * Features:
 *  - BCM (Binary Code Modulation) for 8-bit color depth per channel
 *  - 1/32 scan (64 rows via 5-bit address: A-E)
 *  - Double-buffered: display reads front buffer, rasterizer writes back buffer
 *  - DMA chain auto-restarts — continuous refresh at >120 Hz
 */

#pragma once

#include <cstdint>

namespace Hub75Driver {

/**
 * @brief Initialize the HUB75 PIO driver.
 *
 * Configures GPIO pins, loads the PIO program, sets up DMA chain,
 * and starts the display refresh loop.
 *
 * @param initialFramebuffer  Pointer to the first framebuffer (RGB565, W×H pixels).
 * @return true on success.
 */
bool Initialize(const uint16_t* initialFramebuffer);

/**
 * @brief Atomically update the framebuffer pointer for the next refresh cycle.
 *
 * The DMA chain will pick up the new pointer at the next frame boundary.
 * This is safe to call from any core (atomic pointer write).
 *
 * @param framebuffer  Pointer to the new front buffer (RGB565).
 */
void SetFramebuffer(const uint16_t* framebuffer);

/**
 * @brief Set display brightness via OE (Output Enable) pulse width.
 *
 * @param brightness  0–255 (0 = off, 255 = max brightness).
 */
void SetBrightness(uint8_t brightness);

/**
 * @brief Get the current display refresh rate in Hz.
 *        Measured from DMA completion IRQ timestamps.
 */
uint32_t GetRefreshRate();

/**
 * @brief Shut down the display driver (stops PIO + DMA).
 */
void Shutdown();

}  // namespace Hub75Driver
