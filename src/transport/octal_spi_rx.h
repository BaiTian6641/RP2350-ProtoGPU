/**
 * @file octal_spi_rx.h
 * @brief PIO-driven Octal SPI receiver for RP2350.
 *
 * Receives 8-bit parallel data from the ESP32-S3 LCD peripheral.
 * Data is clocked on the rising edge of SPI_CLK when SPI_CS is low.
 * DMA drains the PIO RX FIFO into a ring buffer in SRAM.
 *
 * The command parser reads complete frames (sync word → CRC) from the
 * ring buffer. Flow control is managed via the RDY GPIO pin.
 */

#pragma once

#include <cstdint>

namespace OctalSpiRx {

/**
 * @brief Initialize the Octal SPI PIO receiver.
 *
 * Configures GPIO pins (D0-D7, CLK, CS), loads the PIO program,
 * sets up DMA to drain RX FIFO → ring buffer.
 *
 * @return true on success.
 */
bool Initialize();

/**
 * @brief Try to extract a complete ProtoGL frame from the ring buffer.
 *
 * Scans for the sync word (0x55AA), validates totalLength from the
 * frame header, and checks that the full frame (including CRC) is
 * available in the ring buffer.
 *
 * @param[out] frameData   Set to pointer within ring buffer (valid until ConsumeFrame).
 * @param[out] frameLength Set to totalLength from frame header.
 * @return true if a complete frame is available.
 */
bool TryGetFrame(const uint8_t** frameData, uint32_t* frameLength);

/**
 * @brief Advance the ring buffer read pointer past the last returned frame.
 *        Must be called after TryGetFrame() returns true and the frame is processed.
 */
void ConsumeFrame();

/**
 * @brief Get available free space in the ring buffer (bytes).
 *        Used by Core 0 to manage the RDY backpressure pin.
 */
uint32_t GetFreeBytes();

/**
 * @brief Get total bytes received since initialization (for diagnostics).
 */
uint64_t GetTotalBytesReceived();

/**
 * @brief Shut down the receiver (stops PIO + DMA).
 */
void Shutdown();

}  // namespace OctalSpiRx
