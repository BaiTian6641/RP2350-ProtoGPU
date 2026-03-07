/**
 * @file ssd1331_driver.h
 * @brief SSD1331 96×64 RGB OLED display driver over hardware SPI.
 *
 * Minimal driver for the headless self-test mode.  Displays an RGB565
 * framebuffer on a 96×64 SSD1331 OLED connected via SPI0.
 *
 * Pin assignments are defined in gpu_config.h (SSD1331_* constants).
 * Only compiled when RP2350GPU_HEADLESS_SELFTEST is defined.
 */

#pragma once

#include <cstdint>

namespace Ssd1331 {

/**
 * @brief Initialize the SSD1331 OLED display.
 *
 * Configures SPI0, sets up GPIO pins (CS, DC, RST), sends the full
 * initialization command sequence, and turns the display on.
 *
 * @return true on success (always true for now — no readback verification).
 */
bool Initialize();

/**
 * @brief Push an entire RGB565 framebuffer to the display.
 *
 * Sets the column/row window to the full 96×64 area and writes
 * all 12288 bytes (96×64×2) via SPI DMA.
 *
 * @param fb  Pointer to 96×64 RGB565 pixel data (12288 bytes).
 *            Pixel order: left-to-right, top-to-bottom.
 */
void PushFramebuffer(const uint16_t* fb);

/**
 * @brief Turn the display off (OLED power save).
 */
void DisplayOff();

/**
 * @brief Turn the display on.
 */
void DisplayOn();

/**
 * @brief Set display contrast (brightness).
 *
 * @param r  Red channel contrast (0–255).
 * @param g  Green channel contrast (0–255).
 * @param b  Blue channel contrast (0–255).
 */
void SetContrast(uint8_t r, uint8_t g, uint8_t b);

}  // namespace Ssd1331
