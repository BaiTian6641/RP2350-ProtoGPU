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

// ─── External Memory: PIO2 Bus (Tier 1 — indirect, DMA) ────────────────────
// PIO2 is the last free PIO block (PIO0 = HUB75, PIO1 = Octal SPI RX).
// It can operate in three modes depending on the installed hardware:
//
//   Mode 1 — OPI PSRAM (APS6408L):
//     8-bit wide bus (DQ0–DQ7 = GPIO 34–41), single CS.
//     8 MB capacity, ~150 MB/s burst.  Indirect DMA access only.
//
//   Mode 2 — Dual QSPI MRAM (2 × MR10Q010):
//     4-bit wide bus (DQ0–DQ3 = GPIO 34–37), two chip selects.
//     CS0 = GPIO 11, CS1 = GPIO 38 (repurposed from upper OPI DQ4).
//     256 KB total (128 KB per chip), ~52 MB/s per chip.
//     No random-access penalty, non-volatile, unlimited endurance.
//     Address bit 17 selects chip (0x00000–0x1FFFF = CS0, 0x20000–0x3FFFF = CS1).
//
//   Mode 3 — Single QSPI MRAM (1 × MR10Q010):
//     4-bit wide bus (DQ0–DQ3 = GPIO 34–37), single CS = GPIO 11.
//     128 KB capacity, ~52 MB/s.  Same MRAM benefits as Mode 2.
//
// All modes use GPIO 12 for CLK.  Set PIO2_MEM_MODE to select the mode;
// set to NONE (0) if no external memory is on PIO2.

/// PIO2 external memory operating mode.
enum class Pio2MemMode : uint8_t {
    NONE              = 0,  ///< PIO2 not used for external memory
    OPI_PSRAM         = 1,  ///< 8-bit OPI: APS6408L (8 MB), DQ0-7 + CLK + CS
    DUAL_QSPI_MRAM   = 2,  ///< 4-bit QSPI × 2: MR10Q010 (256 KB), DQ0-3 + CLK + CS0 + CS1
    SINGLE_QSPI_MRAM = 3,  ///< 4-bit QSPI × 1: MR10Q010 (128 KB), DQ0-3 + CLK + CS
};

// ── Master mode switch ──────────────────────────────────────────────────────
static constexpr Pio2MemMode PIO2_MEM_MODE = Pio2MemMode::NONE;

// ── Shared pins (all modes) ─────────────────────────────────────────────────
static constexpr uint8_t  PIO2_MEM_CLK_PIN       = 12;     // CLK (all modes)
static constexpr uint8_t  PIO2_MEM_CS0_PIN       = 11;     // CS# / CS0# (all modes)
static constexpr uint8_t  PIO2_MEM_DATA_BASE_PIN = 34;     // DQ0 = GPIO34

// ── OPI PSRAM mode (Mode 1) ────────────────────────────────────────────────
static constexpr uint8_t  OPI_PSRAM_DATA_PIN_COUNT = 8;     // DQ0–DQ7 = GPIO 34–41
static constexpr uint32_t OPI_PSRAM_CLOCK_MHZ     = 75;     // PIO clock target (75–150 MHz)
static constexpr uint32_t OPI_PSRAM_CAPACITY      = 8 * 1024 * 1024;  // 8 MB (APS6408L)
static constexpr uint8_t  OPI_PSRAM_READ_LATENCY  = 5;      // Wait cycles after address

// ── QSPI MRAM mode (Mode 2 & 3) ────────────────────────────────────────────
static constexpr uint8_t  QSPI_MRAM_DATA_PIN_COUNT = 4;     // DQ0–DQ3 = GPIO 34–37
static constexpr uint8_t  QSPI_MRAM_CS1_PIN       = 38;     // Second CS for dual mode (GPIO38, reused from OPI DQ4)
static constexpr uint32_t QSPI_MRAM_PIO_CLOCK_MHZ = 104;    // MR10Q010 max QSPI clock
static constexpr uint32_t QSPI_MRAM_CHIP_CAPACITY = 128 * 1024;  // 128 KB per chip
static constexpr uint8_t  QSPI_MRAM_PIO_READ_LAT  = 8;      // Dummy cycles for MRAM quad read
static constexpr uint64_t QSPI_MRAM_PIO_RDID      = 0x076B111111ULL;  // MR10Q010 RDID

// ── Derived constants ───────────────────────────────────────────────────────
/// Total capacity for the configured PIO2 mode.
static constexpr uint32_t Pio2MemCapacity() {
    return (PIO2_MEM_MODE == Pio2MemMode::OPI_PSRAM)         ? OPI_PSRAM_CAPACITY
         : (PIO2_MEM_MODE == Pio2MemMode::DUAL_QSPI_MRAM)   ? (QSPI_MRAM_CHIP_CAPACITY * 2)
         : (PIO2_MEM_MODE == Pio2MemMode::SINGLE_QSPI_MRAM) ? QSPI_MRAM_CHIP_CAPACITY
         : 0;
}
/// Data bus width for the configured PIO2 mode.
static constexpr uint8_t Pio2DataPinCount() {
    return (PIO2_MEM_MODE == Pio2MemMode::OPI_PSRAM) ? OPI_PSRAM_DATA_PIN_COUNT
                                                       : QSPI_MRAM_DATA_PIN_COUNT;
}
/// Whether the configured PIO2 mode uses MRAM (no random-access penalty).
static constexpr bool Pio2IsMram() {
    return (PIO2_MEM_MODE == Pio2MemMode::DUAL_QSPI_MRAM)
        || (PIO2_MEM_MODE == Pio2MemMode::SINGLE_QSPI_MRAM);
}

// ── Legacy aliases (backward compat) ────────────────────────────────────────
static constexpr bool     OPI_PSRAM_ENABLED       = (PIO2_MEM_MODE == Pio2MemMode::OPI_PSRAM);
static constexpr uint8_t  OPI_PSRAM_DATA_BASE_PIN = PIO2_MEM_DATA_BASE_PIN;
static constexpr uint8_t  OPI_PSRAM_CLK_PIN       = PIO2_MEM_CLK_PIN;
static constexpr uint8_t  OPI_PSRAM_CS_PIN        = PIO2_MEM_CS0_PIN;

// ─── QSPI CS1 Auto-Detection (Tier 2 — XIP mapped) ─────────────────────────
// The RP2350's QMI CS1 slot supports auto-detection of installed memory.
// At boot the firmware probes with chip-specific RDID commands:
//   1. MRAM RDID (0x4B + mode byte 0xFF) → Everspin MR10Q010 (128 KB)
//   2. PSRAM RDID (0x9F)                  → AP Memory APS6408L (8 MB)
//
// The detected chip type determines:
//   - Driver init sequence (MRAM needs WREN; PSRAM needs reset + QPI enable)
//   - Capacity and clock configuration
//   - Memory tier placement policy:
//       MRAM:  No random-access penalty → LUTs, materials, textures demote
//              to QSPI more aggressively, freeing SRAM for rasterizer data.
//       PSRAM: Row-buffer miss penalty → random-access data prefers SRAM;
//              sequential/burst data (large textures, meshes) suit QSPI.
//
// Set QSPI_CS1_ENABLED = true to enable CS1 probing at boot.

/// Chip types identifiable on QMI CS1.
enum class QspiChipType : uint8_t {
    NONE            = 0,     ///< Nothing detected (CS1 unpopulated)
    MRAM_MR10Q010   = 1,     ///< Everspin MR10Q010: 128 KB MRAM, 104 MHz
    PSRAM_APS6408L  = 2,     ///< AP Memory APS6408L-SQR: 8 MB PSRAM, 133 MHz
    PSRAM_ESP       = 3,     ///< ESP-PSRAM64H: 8 MB PSRAM, 84 MHz
    UNKNOWN_DEVICE  = 0xFE,  ///< RDID responded but unrecognized
};

/// Hardware profile for a detected QSPI CS1 chip — populated at boot.
struct QspiChipProfile {
    QspiChipType type               = QspiChipType::NONE;
    uint32_t capacityBytes          = 0;       ///< Chip capacity in bytes
    uint32_t maxClockMHz            = 0;       ///< Maximum QSPI clock
    uint8_t  readLatencyDummy       = 0;       ///< Dummy cycles for quad read command
    bool     hasRandomAccessPenalty = true;    ///< false for MRAM, true for PSRAM
    bool     isNonVolatile          = false;   ///< true for MRAM (survives power cycle)
    bool     needsRefresh           = false;   ///< true for DRAM-based PSRAM
    bool     needsWrenBeforeWrite   = false;   ///< true for MRAM (issue 0x06 before writes)
    uint64_t deviceId               = 0;       ///< Raw RDID response bytes
};

// ── Known RDID signatures ───────────────────────────────────────────────────
static constexpr uint64_t QSPI_RDID_MR10Q010 = 0x076B111111ULL;  // MRAM: cmd 0x4B

// ── Built-in chip profiles (used by auto-detect at boot) ────────────────────
static constexpr QspiChipProfile PROFILE_MR10Q010 = {
    QspiChipType::MRAM_MR10Q010,
    128 * 1024,      // 128 KB
    104,             // 104 MHz
    8,               // 8 dummy cycles (FRQAD 0xEB)
    false,           // NO random-access penalty
    true,            // Non-volatile (20+ year retention)
    false,           // No refresh needed
    true,            // Needs WREN (0x06) before writes
    QSPI_RDID_MR10Q010,
};

static constexpr QspiChipProfile PROFILE_APS6408L = {
    QspiChipType::PSRAM_APS6408L,
    8 * 1024 * 1024, // 8 MB
    133,             // 133 MHz SDR
    6,               // 6 dummy cycles
    true,            // HAS random-access penalty (row-buffer miss)
    false,           // Volatile
    true,            // Needs periodic refresh
    false,           // No WREN for writes
    0x000D5D0000ULL, // APS6408L RDID via 0x9F (MFR=0x0D AP Memory)
};

static constexpr QspiChipProfile PROFILE_ESP_PSRAM = {
    QspiChipType::PSRAM_ESP,
    8 * 1024 * 1024, // 8 MB
    84,              // 84 MHz
    6,               // 6 dummy cycles
    true,            // HAS random-access penalty
    false,           // Volatile
    true,            // Needs refresh
    false,           // No WREN
    0,               // Vendor-specific
};

// ── CS1 compile-time flags ──────────────────────────────────────────────────
static constexpr bool      QSPI_CS1_ENABLED  = false;  // Enable CS1 probing at boot
static constexpr uintptr_t QSPI_CS1_XIP_BASE = 0x11000000;  // Fixed by RP2350 hardware

// ── Legacy aliases (backward compat — default to MR10Q010 values) ───────────
// Runtime code should use the detected QspiChipProfile from GetQspiChipProfile().
static constexpr bool      QSPI_MRAM_ENABLED      = QSPI_CS1_ENABLED;
static constexpr uint32_t  QSPI_MRAM_CAPACITY     = 128 * 1024;
static constexpr uint32_t  QSPI_MRAM_CLOCK_MHZ    = 104;
static constexpr uint64_t  QSPI_MRAM_RDID         = QSPI_RDID_MR10Q010;
static constexpr uintptr_t QSPI_MRAM_XIP_BASE     = QSPI_CS1_XIP_BASE;
static constexpr uint8_t   QSPI_MRAM_READ_LATENCY = 8;
static constexpr bool      QSPI_MRAM_DDR          = false;
static constexpr bool      QSPI_PSRAM_ENABLED     = QSPI_CS1_ENABLED;
static constexpr uint32_t  QSPI_PSRAM_CAPACITY    = QSPI_MRAM_CAPACITY;
static constexpr uint32_t  QSPI_PSRAM_CLOCK_MHZ   = QSPI_MRAM_CLOCK_MHZ;
static constexpr uintptr_t QSPI_PSRAM_XIP_BASE    = QSPI_CS1_XIP_BASE;
static constexpr uint8_t   QSPI_PSRAM_READ_LATENCY = QSPI_MRAM_READ_LATENCY;
static constexpr bool      QSPI_PSRAM_DDR         = QSPI_MRAM_DDR;

// ─── Memory Tiering ─────────────────────────────────────────────────────────
// The tiered memory manager (memory/mem_tier.h) places resources across the
// three tiers based on two metrics:
//   weight — rendering pipeline impact (higher = more critical)
//   score  — per-frame access frequency (higher = more accessed)
//
// SRAM cache budget: portion of SRAM reserved for caching PIO2 external memory data.
// This is separate from framebuffers, Z-buffer, QuadTree, and pool allocators.
// Only used when PIO2_MEM_MODE != NONE.

static constexpr uint32_t MEM_TIER_SRAM_CACHE_BUDGET = 64 * 1024;  // 64 KB cache arena
static constexpr uint32_t MEM_TIER_CACHE_LINE_SIZE   = 4096;       // 4 KB per cache line
static constexpr uint8_t  MEM_TIER_ALPHA_WEIGHT      = 3;          // weight coefficient
static constexpr uint8_t  MEM_TIER_BETA_SCORE        = 1;          // score coefficient
static constexpr uint8_t  MEM_TIER_DEMOTION_THRESHOLD = 30;        // frames (~0.5s @ 60 FPS)
static constexpr uint16_t MEM_TIER_PROMOTION_HYSTERESIS = 50;      // min priority to promote

}  // namespace GpuConfig
