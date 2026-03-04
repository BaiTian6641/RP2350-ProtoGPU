/**
 * @file hub75_driver.h
 * @brief PIO-driven HUB75 display driver for RP2350.
 *
 * Drives a HUB75 LED matrix panel using two PIO state machines and DMA.
 * SM0 (hub75_data) shifts pixel data with CLK side-set.
 * SM1 (hub75_row) handles LAT, OE, and row address.
 *
 * Features:
 *  - BCM (Binary Code Modulation) for 8-bit color depth per channel
 *  - 1/32 scan (64 rows via 5-bit address: A-E)
 *  - Double-buffered: display reads front buffer, rasterizer writes back buffer
 *  - PollRefresh() for cooperative multitasking (one BCM plane per call)
 *  - Test patterns for initial bringup verification
 */

#pragma once

#include <cstdint>

namespace Hub75Driver {

/**
 * @brief Initialize the HUB75 PIO driver.
 *
 * Loads PIO programs into pio0, configures SM0 (data) + SM1 (row),
 * sets up DMA channel, and starts both state machines.
 *
 * @param initialFramebuffer  Pointer to the first framebuffer (RGB565, W×H pixels).
 * @return true on success.
 */
bool Initialize(const uint16_t* initialFramebuffer);

/**
 * @brief Atomically update the framebuffer pointer for the next refresh cycle.
 *
 * @param framebuffer  Pointer to the new front buffer (RGB565).
 */
void SetFramebuffer(const uint16_t* framebuffer);

/**
 * @brief Set display brightness (applied during BCM extraction).
 *
 * @param brightness  0–255 (0 = off, 255 = max brightness).
 */
void SetBrightness(uint8_t brightness);

/**
 * @brief Get the current display refresh rate in Hz.
 *        Measured from refresh cycle timestamps.
 */
uint32_t GetRefreshRate();

/**
 * @brief Drive one BCM plane for one row pair (non-blocking incremental refresh).
 *
 * Call this frequently from the main loop. Each call drives one BCM plane
 * for one row pair, advancing the internal row/bit counter.  After
 * SCAN_ROWS × COLOR_DEPTH calls, one full panel refresh is complete.
 *
 * At 128px × 32 rows × 8 BCM bits = 256 calls per full refresh.
 * Target: ~3 ms per full refresh → ~333 Hz.
 */
void PollRefresh();

/**
 * @brief Fill a framebuffer with a test pattern.
 *
 * @param fb       Pointer to RGB565 framebuffer (PANEL_WIDTH × PANEL_HEIGHT).
 * @param pattern  0 = solid red, 1 = RGB gradient, 2 = checkerboard, 3 = color bars.
 */
void FillTestPattern(uint16_t* fb, uint8_t pattern);

/**
 * @brief Shut down the display driver (stops PIO + DMA, blanks display).
 */
void Shutdown();

}  // namespace Hub75Driver
