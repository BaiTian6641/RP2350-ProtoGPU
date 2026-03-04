/**
 * @file gpu_config.h
 * @brief RP2350 GPU hardware configuration — pin assignments, clocks, limits.
 *
 * Edit these values to match your PCB layout. All pin numbers are RP2350 GPIO.
 */

#pragma once

#include <cstdint>

namespace GpuConfig {

// ─── System ─────────────────────────────────────────────────────────────────

static constexpr uint32_t SYSTEM_CLOCK_MHZ = 150;  // default; try 200 if stable

// ─── Octal SPI Receiver (PIO) ───────────────────────────────────────────────

static constexpr uint8_t SPI_DATA_BASE_PIN = 0;    // D0=GPIO0 .. D7=GPIO7
static constexpr uint8_t SPI_DATA_PIN_COUNT = 8;
static constexpr uint8_t SPI_CLK_PIN  = 8;
static constexpr uint8_t SPI_CS_PIN   = 9;

static constexpr uint32_t SPI_RING_BUFFER_SIZE = 32768;  // 32 KB

// ─── Flow Control ───────────────────────────────────────────────────────────

static constexpr uint8_t RDY_PIN = 10;  // Output: assert high when GPU can accept data

/// Backpressure thresholds (percentage of ring buffer)
static constexpr uint8_t RDY_ASSERT_THRESHOLD   = 50;  // assert RDY when >= 50% free
static constexpr uint8_t RDY_DEASSERT_THRESHOLD = 25;  // deassert when < 25% free

// ─── I2C Slave ──────────────────────────────────────────────────────────────

static constexpr uint8_t I2C_SDA_PIN  = 14;
static constexpr uint8_t I2C_SCL_PIN  = 15;
static constexpr uint8_t I2C_ADDRESS  = 0x3C;
static constexpr uint     I2C_INSTANCE = 0;  // i2c0

// ─── HUB75 Display ─────────────────────────────────────────────────────────

static constexpr uint8_t HUB75_R1_PIN    = 16;
static constexpr uint8_t HUB75_G1_PIN    = 17;
static constexpr uint8_t HUB75_B1_PIN    = 18;
static constexpr uint8_t HUB75_R2_PIN    = 19;
static constexpr uint8_t HUB75_G2_PIN    = 20;
static constexpr uint8_t HUB75_B2_PIN    = 21;
static constexpr uint8_t HUB75_ADDR_A    = 22;
static constexpr uint8_t HUB75_ADDR_B    = 23;
static constexpr uint8_t HUB75_ADDR_C    = 24;
static constexpr uint8_t HUB75_ADDR_D    = 25;
static constexpr uint8_t HUB75_ADDR_E    = 26;  // for 1/32 scan (64 rows)
static constexpr uint8_t HUB75_CLK_PIN   = 27;
static constexpr uint8_t HUB75_LAT_PIN   = 28;
static constexpr uint8_t HUB75_OE_PIN    = 29;

// Panel geometry
static constexpr uint16_t PANEL_WIDTH    = 128;
static constexpr uint16_t PANEL_HEIGHT   = 64;
static constexpr uint8_t  SCAN_ROWS      = 32;   // 1/32 scan
static constexpr uint8_t  COLOR_DEPTH    = 8;     // BCM bits per channel

// ─── Framebuffer ────────────────────────────────────────────────────────────

static constexpr uint32_t FRAMEBUF_PIXELS = PANEL_WIDTH * PANEL_HEIGHT;
static constexpr uint32_t FRAMEBUF_SIZE   = FRAMEBUF_PIXELS * 2;  // RGB565 = 2 bytes/pixel

// ─── Resource Limits (must be ≤ PGL_MAX_* from PglTypes.h) ─────────────────

static constexpr uint16_t MAX_VERTICES   = 2048;
static constexpr uint16_t MAX_TRIANGLES  = 1024;
static constexpr uint16_t MAX_MESHES     = 256;
static constexpr uint16_t MAX_MATERIALS  = 256;
static constexpr uint8_t  MAX_TEXTURES   = 64;
static constexpr uint8_t  MAX_DRAW_CALLS = 64;

// ─── Memory Pools (shared across all resource slots) ────────────────────────
// The RP2350 has only 520 KB SRAM; embedded per-slot arrays are infeasible.
// Instead, large data (vertices, indices, pixels, layout coords) are allocated
// from typed bump pools.  This keeps slot metadata small (~40 B) while
// supporting the full protocol resource-ID range.

static constexpr uint32_t VERTEX_POOL_SIZE       = 4096;   // PglVec3  — 48 KB
static constexpr uint32_t INDEX_POOL_SIZE        = 2048;   // PglIndex3 — 12 KB
static constexpr uint32_t UV_VERTEX_POOL_SIZE    = 2048;   // PglVec2  — 16 KB
static constexpr uint32_t UV_INDEX_POOL_SIZE     = 1024;   // PglIndex3 — 6 KB
static constexpr uint32_t TEXTURE_POOL_SIZE      = 32768;  // bytes   — 32 KB
static constexpr uint32_t LAYOUT_COORD_POOL_SIZE = 4096;   // PglVec2  — 32 KB
static constexpr uint32_t FRAME_VERTEX_POOL_SIZE = 2048;   // PglVec3  — 24 KB (per-frame)

// ─── QuadTree ───────────────────────────────────────────────────────────────

static constexpr uint8_t  QUADTREE_MAX_DEPTH    = 8;
static constexpr uint8_t  QUADTREE_MAX_ENTITIES = 16;
static constexpr uint16_t QUADTREE_MAX_NODES    = 4096;

}  // namespace GpuConfig
