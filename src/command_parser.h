/**
 * @file command_parser.h
 * @brief ProtoGL command buffer parser for the RP2350 GPU.
 *
 * Deserializes a ProtoGL frame (sync word → commands → CRC) received via
 * Octal SPI and updates the GPU's local SceneState (resource tables, draw list).
 *
 * Uses PglParser.h for alignment-safe reads — safe on both ARM Cortex-M33
 * (which supports unaligned access) and RISC-V Hazard3 (which may not).
 */

#pragma once

#include <cstdint>

// Forward declarations
struct SceneState;
class OpiPsramDriver;
class QspiPsramDriver;
class MemTierManager;
class MemPoolManager;
class DisplayManager;

namespace CommandParser {

/// Parse result codes
enum class ParseResult : uint8_t {
    Ok              = 0,  // Frame parsed successfully
    CrcError        = 1,  // CRC-16 mismatch
    InvalidSync     = 2,  // Missing or wrong sync word
    TruncatedFrame  = 3,  // totalLength exceeds available data
    UnknownOpcode   = 4,  // Encountered unrecognized opcode (non-fatal, skipped)
    ResourceFull    = 5,  // Mesh/material table full, create command rejected
};

/**
 * @brief Initialize the memory subsystem pointers used by memory opcodes.
 *
 * Must be called after memory drivers are initialized, before any frames
 * containing memory commands (0x30–0x3F) are parsed.
 *
 * @param opi    PIO2 external memory driver (may be nullptr if not present).
 * @param qspi   QMI CS1 driver (may be nullptr if not present).
 * @param tier   Tiered memory manager.
 * @param frontBuf  Pointer to the front (display) framebuffer.
 * @param backBuf   Pointer to the back (render) framebuffer.
 * @param fbPixels  Number of pixels per framebuffer.
 */
void InitMemory(OpiPsramDriver* opi, QspiPsramDriver* qspi,
                MemTierManager* tier,
                const uint16_t* frontBuf, const uint16_t* backBuf,
                uint32_t fbPixels);

/**
 * @brief Update the framebuffer pointers after a swap.
 *
 * Called by gpu_core.cpp each frame after the buffer swap so that
 * CMD_FRAMEBUFFER_CAPTURE captures the correct buffer.
 */
void UpdateFramebufferPtrs(const uint16_t* frontBuf, const uint16_t* backBuf);

/**
 * @brief Initialize M11 display and pool subsystem pointers.
 *
 * Must be called after DisplayManager and MemPoolManager are created.
 *
 * @param displayMgr  Display manager singleton.
 * @param poolMgr     Memory pool manager.
 */
void InitDisplayAndPools(DisplayManager* displayMgr, MemPoolManager* poolMgr);

/**
 * @brief Parse a complete ProtoGL frame and update the scene state.
 *
 * Steps:
 *  1. Validate sync word (0x55AA)
 *  2. Read frame header (frameNumber, totalLength, commandCount)
 *  3. Validate CRC-16 over entire frame
 *  4. Iterate commands: switch on opcode → dispatch to handler
 *  5. Return result
 *
 * @param frameData   Pointer to the start of the frame (sync word).
 * @param frameLength Total frame length (from frame header's totalLength).
 * @param scene       Scene state to update with parsed commands.
 * @return ParseResult::Ok on success.
 */
ParseResult Parse(const uint8_t* frameData, uint32_t frameLength,
                  SceneState* scene);

}  // namespace CommandParser
