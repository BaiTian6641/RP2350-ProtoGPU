/**
 * @file i2c_slave.h
 * @brief I2C slave handler for RP2350 GPU configuration bus.
 *
 * Handles I2C register reads/writes from the ESP32-S3 host:
 *  - Configuration: brightness, panel config, scan rate, gamma, color order
 *  - Control: clear display, reset GPU, set clock frequency
 *  - Status: FPS, dropped frames, free memory, temperature, GPU usage
 *  - Extended status: per-core usage, VRAM tiers, frame timing, clock
 *  - Capability query: architecture, core count, SRAM, limits, flags, VRAM
 *
 * Register map defined in PglTypes.h (PglI2CRegister enum).
 */

#pragma once

#include <cstdint>

struct SceneState;  // forward declaration for memory register readback
class DisplayManager;
class MemPoolManager;

namespace I2CSlave {

/**
 * @brief Initialize hardware I2C slave at the configured address.
 *
 * Also initialises the ADC temperature sensor and probes for external
 * VRAM (OPI PSRAM on PIO2, QSPI MRAM on QMI CS1).  Detected VRAM is
 * reported in the capability response and extended status.
 *
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
 * @brief Update the basic status response data (called by Core 0 each frame).
 * @param fps           Current render FPS.
 * @param droppedFrames Cumulative dropped frames (CRC errors, overflow).
 * @param freeMemory    Free SRAM in bytes.
 * @param temperature   Die temperature in °C (or 0 if unavailable).
 * @param renderBusy    True if currently rendering a frame.
 */
void UpdateStatus(uint16_t fps, uint16_t droppedFrames,
                  uint32_t freeMemory, int8_t temperature, bool renderBusy);

/**
 * @brief Update the extended status response (called by Core 0 each frame).
 *
 * All timing values are from the previous frame.  VRAM fields are updated
 * automatically by the memory tier manager when available.
 */
void UpdateExtendedStatus(uint16_t fps, uint16_t droppedFrames,
                          uint8_t gpuUsage, uint8_t core0Usage, uint8_t core1Usage,
                          uint8_t flags,
                          int16_t temperatureQ8, uint16_t currentClockMHz,
                          uint16_t sramFreeKB,
                          uint16_t frameTimeUs, uint16_t rasterTimeUs,
                          uint16_t transferTimeUs, uint16_t hub75RefreshHz);

/**
 * @brief Update VRAM tier fields in extended status.
 *
 * Called by the memory tier manager after probing / initialising VRAM.
 */
void UpdateVramStatus(uint16_t opiTotalKB, uint16_t opiFreeKB,
                      uint16_t qspiTotalKB, uint16_t qspiFreeKB,
                      uint8_t tierFlags);

/**
 * @brief Read the on-chip temperature sensor.
 * @return Temperature in degrees Celsius (approximate, ±5 °C).
 */
float ReadTemperature();

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
 * @brief Check if the host has requested a clock change.
 *        Returns the target MHz (0 = no pending request).
 *        Clears the request after reading.
 */
uint16_t ConsumeClockRequest(uint8_t* outVoltage = nullptr, uint8_t* outFlags = nullptr);

/**
 * @brief Set a pointer to the SceneState for I2C memory register readback.
 *
 * Must be called after SceneState is initialised and before memory
 * register reads (0x0C–0x0F) are used.  The pointer must remain valid
 * for the lifetime of the I2C slave.
 */
void SetSceneState(SceneState* scene);

/**
 * @brief Set display and pool manager pointers for M11 register support.
 *
 * Must be called after DisplayManager and MemPoolManager are initialised.
 */
void SetDisplayAndPools(DisplayManager* displayMgr, MemPoolManager* poolMgr);

/**
 * @brief Shut down I2C slave.
 */
void Shutdown();

}  // namespace I2CSlave
