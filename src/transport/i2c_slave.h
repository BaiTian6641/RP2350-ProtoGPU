/**
 * @file i2c_slave.h
 * @brief I2C slave handler for RP2350 GPU configuration bus.
 *
 * Handles I2C register reads/writes from the ESP32-S3 host:
 *  - Configuration: brightness, panel config, scan rate, gamma, color order
 *  - Control: clear display, reset GPU
 *  - Status: FPS, dropped frames, free memory, temperature
 *  - Capability query: architecture, core count, SRAM, limits, flags
 *
 * Register map defined in PglTypes.h (PglI2CRegister enum).
 */

#pragma once

#include <cstdint>

namespace I2CSlave {

/**
 * @brief Initialize hardware I2C slave at the configured address.
 * @return true on success.
 */
bool Initialize();

/**
 * @brief Poll for pending I2C transactions (non-blocking).
 *
 * Call this periodically from the Core 0 main loop or from an IRQ handler.
 * Processes any pending register writes / read requests.
 */
void Poll();

/**
 * @brief Update the status response data (called by Core 0 each frame).
 * @param fps           Current render FPS.
 * @param droppedFrames Cumulative dropped frames (CRC errors, overflow).
 * @param freeMemory    Free SRAM in bytes.
 * @param temperature   Die temperature in °C (or 0 if unavailable).
 * @param renderBusy    True if currently rendering a frame.
 */
void UpdateStatus(uint16_t fps, uint16_t droppedFrames,
                  uint32_t freeMemory, int8_t temperature, bool renderBusy);

/**
 * @brief Get the last brightness value written by the host.
 */
uint8_t GetBrightness();

/**
 * @brief Check if the host has requested a GPU reset.
 *        Clears the flag after reading.
 */
bool ConsumeResetRequest();

/**
 * @brief Shut down I2C slave.
 */
void Shutdown();

}  // namespace I2CSlave
