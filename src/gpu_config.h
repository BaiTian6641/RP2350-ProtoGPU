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
static constexpr uint8_t  QUADTREE_MAX_ENTITIES = 8;   // entities per leaf (was 16; reduced for SRAM)
static constexpr uint16_t QUADTREE_MAX_NODES    = 512;  // total nodes (was 4096; spec §8.3 says 512)

// ─── External Memory: OPI PSRAM via PIO2 (Tier 1 — indirect, DMA cached) ───
// PIO2 is the last free PIO block (PIO0 = HUB75, PIO1 = Octal SPI RX).
// OPI PSRAM provides high-bandwidth bulk storage for textures, large meshes,
// and material data that don't fit in SRAM.  All access is through explicit
// DMA reads/writes — the CPU cannot dereference OPI PSRAM as memory.
// Set OPI_PSRAM_ENABLED = false if no OPI PSRAM is populated.

static constexpr bool     OPI_PSRAM_ENABLED       = false;  // Enable when hardware is present
static constexpr uint8_t  OPI_PSRAM_DATA_BASE_PIN = 34;     // DQ0 = GPIO34 .. DQ7 = GPIO41
static constexpr uint8_t  OPI_PSRAM_DATA_PIN_COUNT = 8;
static constexpr uint8_t  OPI_PSRAM_CLK_PIN       = 12;     // PSRAM CLK
static constexpr uint8_t  OPI_PSRAM_CS_PIN        = 11;     // PSRAM CS# (active low)
static constexpr uint32_t OPI_PSRAM_CLOCK_MHZ     = 75;     // PIO clock target (75–150 MHz)
static constexpr uint32_t OPI_PSRAM_CAPACITY      = 8 * 1024 * 1024;  // 8 MB (APS6408L)
static constexpr uint8_t  OPI_PSRAM_READ_LATENCY  = 5;      // Wait cycles after address

// ─── External Memory: QSPI PSRAM via QMI CS1 (Tier 2 — XIP mapped) ─────────
// Uses the RP2350's built-in QMI controller on CS1.  Memory-mapped with
// hardware XIP cache (4 KB, 2-way set associative).  Transparent to the CPU:
// any pointer into 0x11000000–0x11FFFFFF reads/writes PSRAM automatically.
// Ideal for frequently-accessed, non-pipeline-critical data (lookup tables,
// font atlases, inactive meshes, animation keyframes).
// Set QSPI_PSRAM_ENABLED = false if no QSPI PSRAM is populated on CS1.

static constexpr bool     QSPI_PSRAM_ENABLED      = false;  // Enable when hardware is present
static constexpr uint32_t QSPI_PSRAM_CAPACITY     = 8 * 1024 * 1024;  // 8 MB
static constexpr uint32_t QSPI_PSRAM_CLOCK_MHZ    = 75;     // QMI clock (75–133 MHz)
static constexpr uintptr_t QSPI_PSRAM_XIP_BASE    = 0x11000000;  // Fixed by RP2350 hardware
static constexpr uint8_t  QSPI_PSRAM_READ_LATENCY = 5;      // Dummy cycles for QPI read
static constexpr bool     QSPI_PSRAM_DDR          = false;   // True for DDR mode (2× bandwidth)

// ─── Memory Tiering ─────────────────────────────────────────────────────────
// The tiered memory manager (memory/mem_tier.h) places resources across the
// three tiers based on two metrics:
//   weight — rendering pipeline impact (higher = more critical)
//   score  — per-frame access frequency (higher = more accessed)
//
// SRAM cache budget: portion of SRAM reserved for caching OPI PSRAM data.
// This is separate from framebuffers, Z-buffer, QuadTree, and pool allocators.
// Only used when OPI_PSRAM_ENABLED = true.

static constexpr uint32_t MEM_TIER_SRAM_CACHE_BUDGET = 64 * 1024;  // 64 KB cache arena
static constexpr uint32_t MEM_TIER_CACHE_LINE_SIZE   = 4096;       // 4 KB per cache line
static constexpr uint8_t  MEM_TIER_ALPHA_WEIGHT      = 3;          // weight coefficient
static constexpr uint8_t  MEM_TIER_BETA_SCORE        = 1;          // score coefficient
static constexpr uint8_t  MEM_TIER_DEMOTION_FRAMES   = 30;         // ~0.5s at 60 FPS

}  // namespace GpuConfig
