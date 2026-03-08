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

// ─── Bidirectional Octal SPI (PIO) ─────────────────────────────────────────────────

static constexpr uint8_t SPI_DATA_BASE_PIN = 0;    // D0=GPIO0 .. D7=GPIO7
static constexpr uint8_t SPI_DATA_PIN_COUNT = 8;
static constexpr uint8_t SPI_CLK_PIN  = 8;
static constexpr uint8_t SPI_CS_PIN   = 9;

static constexpr uint32_t SPI_RING_BUFFER_SIZE = 32768;  // 32 KB

// ─── Bus Direction & Notification ─────────────────────────────────────────────────

static constexpr uint8_t DIR_PIN = 10;  // Input: host drives direction (high=host TX, low=GPU TX)
static constexpr uint8_t IRQ_PIN = 13;  // Output: active-low async notification to host

/// Backpressure thresholds (percentage of ring buffer)
/// IRQ is asserted (active-low) when ring buffer free space drops below DEASSERT threshold,
/// and deasserted when free space rises above ASSERT threshold.
static constexpr uint8_t IRQ_ASSERT_THRESHOLD   = 50;  // deassert IRQ (release) when >= 50% free
static constexpr uint8_t IRQ_DEASSERT_THRESHOLD = 25;  // assert IRQ (backpressure) when < 25% free

// ─── I2C Slave ──────────────────────────────────────────────────────────────
// GP14 = I2C1_SDA, GP15 = I2C1_SCL on the Pico 2 header.

static constexpr uint8_t I2C_SDA_PIN  = 14;
static constexpr uint8_t I2C_SCL_PIN  = 15;
static constexpr uint8_t I2C_ADDRESS  = 0x3C;
static constexpr unsigned int I2C_INSTANCE = 1;  // i2c1 (GP14/15 are I2C1)

// ─── HUB75 Display ─────────────────────────────────────────────────────────
// HUB75 is DISABLED for Pico 2 functional testing (no panel connected).
// All pins set to 0xFF — the driver detects this and becomes a no-op.
// UART0 TX uses GP16, RX uses GP17 (reclaimed from RGB data pins).

static constexpr uint8_t HUB75_R1_PIN    = 0xFF;  // disabled
static constexpr uint8_t HUB75_G1_PIN    = 0xFF;
static constexpr uint8_t HUB75_B1_PIN    = 0xFF;
static constexpr uint8_t HUB75_R2_PIN    = 0xFF;
static constexpr uint8_t HUB75_G2_PIN    = 0xFF;
static constexpr uint8_t HUB75_B2_PIN    = 0xFF;
static constexpr uint8_t HUB75_ADDR_A    = 0xFF;
static constexpr uint8_t HUB75_ADDR_B    = 0xFF;
static constexpr uint8_t HUB75_ADDR_C    = 0xFF;
static constexpr uint8_t HUB75_ADDR_D    = 0xFF;
static constexpr uint8_t HUB75_ADDR_E    = 0xFF;
static constexpr uint8_t HUB75_CLK_PIN   = 0xFF;
static constexpr uint8_t HUB75_LAT_PIN   = 0xFF;
static constexpr uint8_t HUB75_OE_PIN    = 0xFF;

// Panel geometry
// Headless self-test renders at 96×96 (matching SSD1331 width, square aspect).
// Normal mode renders at 128×64 (HUB75 LED panel).
#ifdef RP2350GPU_HEADLESS_SELFTEST
static constexpr uint16_t PANEL_WIDTH    = 96;
static constexpr uint16_t PANEL_HEIGHT   = 96;
#else
static constexpr uint16_t PANEL_WIDTH    = 128;
static constexpr uint16_t PANEL_HEIGHT   = 64;
#endif
static constexpr uint8_t  SCAN_ROWS      = 32;   // 1/32 scan (HUB75 only)
static constexpr uint8_t  COLOR_DEPTH    = 8;     // BCM bits per channel (HUB75 only)

// ─── Framebuffer ────────────────────────────────────────────────────────────

static constexpr uint32_t FRAMEBUF_PIXELS = PANEL_WIDTH * PANEL_HEIGHT;
static constexpr uint32_t FRAMEBUF_SIZE   = FRAMEBUF_PIXELS * 2;  // RGB565 = 2 bytes/pixel

// ─── Resource Limits (must be ≤ PGL_MAX_* from PglTypes.h) ─────────────────

static constexpr uint16_t MAX_VERTICES   = 1024;   // per-mesh vertex limit (transform scratch)
static constexpr uint16_t MAX_TRIANGLES  = 1280;   // max projected triangles per frame (teapot needs ~600 after cull)
static constexpr uint16_t MAX_MESHES     = 256;
static constexpr uint16_t MAX_MATERIALS  = 256;
static constexpr uint8_t  MAX_TEXTURES   = 64;
static constexpr uint8_t  MAX_DRAW_CALLS = 64;

// ─── Memory Pools (shared across all resource slots) ────────────────────────
// The RP2350 has only 520 KB SRAM; embedded per-slot arrays are infeasible.
// Instead, large data (vertices, indices, pixels, layout coords) are allocated
// from typed bump pools.  This keeps slot metadata small (~40 B) while
// supporting the full protocol resource-ID range.

static constexpr uint32_t VERTEX_POOL_SIZE       = 2048;   // PglVec3  — 24 KB
static constexpr uint32_t INDEX_POOL_SIZE        = 2048;   // PglIndex3 — 12 KB
static constexpr uint32_t UV_VERTEX_POOL_SIZE    = 2048;   // PglVec2  — 16 KB
static constexpr uint32_t UV_INDEX_POOL_SIZE     = 1024;   // PglIndex3 — 6 KB
static constexpr uint32_t TEXTURE_POOL_SIZE      = 32768;  // bytes   — 32 KB
static constexpr uint32_t LAYOUT_COORD_POOL_SIZE = 2048;   // PglVec2  — 16 KB
static constexpr uint32_t FRAME_VERTEX_POOL_SIZE = 2048;   // PglVec3  — 24 KB (per-frame)

// ─── QuadTree ───────────────────────────────────────────────────────────────

static constexpr uint8_t  QUADTREE_MAX_DEPTH    = 6;   // reduced for 128×64 panel (>6 unnecessary)
static constexpr uint8_t  QUADTREE_MAX_ENTITIES = 8;   // entities per leaf
static constexpr uint16_t QUADTREE_MAX_NODES    = 256;  // total nodes (128×64 panel needs few subdivisions)

// ─── External QSPI VRAM (PIO2 — Dual Channel, RP2350B QFN-80 Only) ─────────
// PIO2 is the last free PIO block (PIO0 = HUB75, PIO1 = Octal SPI bidir).
// It drives two independent QSPI channels, each with 2 chip selects:
//
//   Channel A (Tier 1): PIO2 SM0+SM1, data GPIO 34-37, CLK GPIO 12,
//                        CS0 GPIO 11, CS1 GPIO 38
//   Channel B (Tier 2): PIO2 SM2+SM3, data GPIO 39-42, CLK GPIO 43,
//                        CS0 GPIO 44, CS1 GPIO 45
//
// Each chip-select is auto-detected at boot via 3-step RDID probe:
//   MRAM (MR10Q010):  128 KB, 104 MHz, no random-access penalty, non-volatile
//   PSRAM (APS6408L): 8 MB, 133 MHz, row-buffer miss penalty, volatile
//
// RP2350A (QFN-60, 30 GPIO): No GPIO 34+, set QSPI_VRAM_MODE = NONE.
// RP2350B (QFN-80, 48 GPIO): Up to 2×2 = 4 external VRAM chips.
//
// All access is indirect via PIO2 DMA (no XIP).

/// QSPI VRAM operating mode.
enum class QspiVramMode : uint8_t {
    NONE           = 0,  ///< No external VRAM (RP2350A, or RP2350B unpopulated)
    SINGLE_CHANNEL = 1,  ///< Channel A only (1–2 chips)
    DUAL_CHANNEL   = 2,  ///< Channel A + Channel B (up to 2+2 chips)
};

// ── Master mode switch ──────────────────────────────────────────────────────
static constexpr QspiVramMode QSPI_VRAM_MODE = QspiVramMode::NONE;

// ── Channel A pins (Tier 1) ─────────────────────────────────────────────────
static constexpr uint8_t  QSPI_A_DATA_BASE_PIN = 34;     // DQ0 = GPIO34, DQ1-3 = 35-37
static constexpr uint8_t  QSPI_A_DATA_PIN_COUNT = 4;
static constexpr uint8_t  QSPI_A_CLK_PIN       = 12;
static constexpr uint8_t  QSPI_A_CS0_PIN       = 11;     // First chip
static constexpr uint8_t  QSPI_A_CS1_PIN       = 38;     // Second chip (optional)
static constexpr uint8_t  QSPI_A_CHIP_COUNT    = 0;      // 0, 1, or 2 (set per board)

// ── Channel B pins (Tier 2) ─────────────────────────────────────────────────
static constexpr uint8_t  QSPI_B_DATA_BASE_PIN = 39;     // DQ0 = GPIO39, DQ1-3 = 40-42
static constexpr uint8_t  QSPI_B_DATA_PIN_COUNT = 4;
static constexpr uint8_t  QSPI_B_CLK_PIN       = 43;
static constexpr uint8_t  QSPI_B_CS0_PIN       = 44;     // First chip
static constexpr uint8_t  QSPI_B_CS1_PIN       = 45;     // Second chip (optional)
static constexpr uint8_t  QSPI_B_CHIP_COUNT    = 0;      // 0, 1, or 2 (set per board)

// ── Chip profiles (shared between channels) ─────────────────────────────────
static constexpr uint32_t QSPI_MRAM_PIO_CLOCK_MHZ = 104;    // MR10Q010 max QSPI clock
static constexpr uint32_t QSPI_MRAM_CHIP_CAPACITY = 128 * 1024;  // 128 KB per chip
static constexpr uint8_t  QSPI_MRAM_PIO_READ_LAT  = 8;      // Dummy cycles for MRAM quad read
static constexpr uint64_t QSPI_MRAM_PIO_RDID      = 0x076B111111ULL;  // MR10Q010 RDID

static constexpr uint32_t QSPI_PSRAM_PIO_CLOCK_MHZ = 133;   // APS6408L max SDR clock
static constexpr uint32_t QSPI_PSRAM_CHIP_CAPACITY = 8 * 1024 * 1024;  // 8 MB per chip
static constexpr uint8_t  QSPI_PSRAM_PIO_READ_LAT  = 6;     // Dummy cycles for PSRAM quad read

// ── Derived constants ───────────────────────────────────────────────────────
/// Whether any external VRAM is configured.
static constexpr bool QspiVramEnabled() {
    return QSPI_VRAM_MODE != QspiVramMode::NONE;
}
/// Whether Channel B is active.
static constexpr bool QspiChannelBEnabled() {
    return QSPI_VRAM_MODE == QspiVramMode::DUAL_CHANNEL;
}

// ── Legacy aliases (backward compat) ────────────────────────────────────────
// Map old PIO2_MEM_MODE to new QSPI_VRAM_MODE for any code still referencing them.
using Pio2MemMode = QspiVramMode;
static constexpr QspiVramMode PIO2_MEM_MODE = QSPI_VRAM_MODE;
static constexpr bool OPI_PSRAM_ENABLED = false;  // OPI mode removed

// ─── QSPI VRAM Chip Auto-Detection ──────────────────────────────────────────
// At boot the firmware probes each chip-select with chip-specific RDID commands:
//   1. MRAM RDID (0x4B + mode byte 0xFF) → Everspin MR10Q010 (128 KB)
//   2. PSRAM RDID (0x9F)                  → AP Memory APS6408L (8 MB)
//
// Each of the 4 chip-selects (A-CS0, A-CS1, B-CS0, B-CS1) is probed
// independently, allowing mixed MRAM/PSRAM per channel.
//
// The detected chip type determines per-chip:
//   - Driver init sequence (MRAM needs WREN; PSRAM needs reset + QPI enable)
//   - Capacity and clock configuration
//   - Memory tier placement policy weights

/// Chip types identifiable on QSPI VRAM chip-selects.
enum class QspiChipType : uint8_t {
    NONE            = 0,     ///< Nothing detected (chip-select unpopulated)
    MRAM_MR10Q010   = 1,     ///< Everspin MR10Q010: 128 KB MRAM, 104 MHz
    PSRAM_APS6408L  = 2,     ///< AP Memory APS6408L-SQR: 8 MB PSRAM, 133 MHz
    PSRAM_ESP       = 3,     ///< ESP-PSRAM64H: 8 MB PSRAM, 84 MHz
    UNKNOWN_DEVICE  = 0xFE,  ///< RDID responded but unrecognized
};

/// Hardware profile for a detected QSPI VRAM chip — populated at boot.
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
static constexpr uint64_t QSPI_RDID_MR10Q010  = 0x076B111111ULL;  // MRAM: cmd 0x4B
static constexpr uint64_t QSPI_RDID_APS6408L  = 0x000D5D0000ULL;  // PSRAM: cmd 0x9F

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

// ── I2C1 HUD OLED Display ──────────────────────────────────────────────────
// Secondary status display (SSD1306/SSD1309, 128×64 mono) on I2C1.
// Shares the I2C1 bus with the host management I2C slave — uses a different
// address.  The HUD driver renders GPU status (FPS, VRAM, temp) when idle.

static constexpr uint8_t  HUD_I2C_INSTANCE     = 1;      // i2c1 (shared with management I2C)
static constexpr uint8_t  HUD_I2C_ADDRESS      = 0x3D;   // SSD1306 alternate address (A0 = VCC)
static constexpr uint16_t HUD_WIDTH            = 128;
static constexpr uint16_t HUD_HEIGHT           = 64;
static constexpr bool     HUD_ENABLED          = false;   // Enable GPU-attached HUD display

// ─── SSD1331 OLED Display (Headless Self-Test) ──────────────────────────────
// Used only when RP2350GPU_HEADLESS_SELFTEST is defined.
// 96×64 RGB565 OLED connected via hardware SPI0.
// GPIO 17 is repurposed from UART RX when this mode is active.

static constexpr uint8_t  SSD1331_CS_PIN   = 17;   // Chip Select (repurposed from UART RX)
static constexpr uint8_t  SSD1331_SCK_PIN  = 18;   // SPI0 SCK
static constexpr uint8_t  SSD1331_MOSI_PIN = 19;   // SPI0 TX (MOSI)
static constexpr uint8_t  SSD1331_DC_PIN   = 21;   // Data/Command select
static constexpr uint8_t  SSD1331_RST_PIN  = 22;   // Hardware reset

static constexpr uint16_t SSD1331_WIDTH    = 96;
static constexpr uint16_t SSD1331_HEIGHT   = 64;
// SSD1331 supports high-speed SPI transfers; 16 MHz is typically stable on
// short PCB traces with clean signal integrity.
static constexpr uint32_t SSD1331_SPI_BAUD = 32000000;  // 32 MHz SPI clock (requested)

// ─── Memory Tiering ─────────────────────────────────────────────────────────
// The tiered memory manager (memory/mem_tier.h) places resources across the
// three tiers based on two metrics:
//   weight — rendering pipeline impact (higher = more critical)
//   score  — per-frame access frequency (higher = more accessed)
//
// SRAM cache budget: portion of SRAM reserved for caching QSPI VRAM data.
// This is separate from framebuffers, Z-buffer, QuadTree, and pool allocators.
// Only used when QSPI_VRAM_MODE != NONE.

static constexpr uint32_t MEM_TIER_SRAM_CACHE_BUDGET = 64 * 1024;  // 64 KB cache arena
static constexpr uint32_t MEM_TIER_CACHE_LINE_SIZE   = 4096;       // 4 KB per cache line
static constexpr uint8_t  MEM_TIER_ALPHA_WEIGHT      = 3;          // weight coefficient
static constexpr uint8_t  MEM_TIER_BETA_SCORE        = 1;          // score coefficient
static constexpr uint8_t  MEM_TIER_DEMOTION_THRESHOLD = 30;        // frames (~0.5s @ 60 FPS)
static constexpr uint16_t MEM_TIER_PROMOTION_HYSTERESIS = 50;      // min priority to promote

// ─── Flash Persistence (M12) ────────────────────────────────────────────────
// RP2350 onboard flash region reserved for persisted GPU resources.
// Uses the last FLASH_PERSIST_SIZE bytes of the flash chip.
// The manifest and resource data share this region.

static constexpr uint32_t FLASH_TOTAL_SIZE     = 4 * 1024 * 1024;  // 4 MB (typical W25Q32)
static constexpr uint32_t FLASH_PERSIST_SIZE   = 512 * 1024;       // 512 KB reserved
static constexpr uint32_t FLASH_PERSIST_OFFSET =                    // Offset from XIP_BASE
    FLASH_TOTAL_SIZE - FLASH_PERSIST_SIZE;                          // = 0x00380000
static constexpr uint16_t FLASH_MAX_ENTRIES    = 64;                // Max manifest entries
static constexpr uint32_t FLASH_SECTOR_SIZE    = 4096;              // 4 KB erase sector
static constexpr uint32_t FLASH_PAGE_SIZE      = 256;               // 256 B program page
static constexpr uint8_t  FLASH_WRITEBACK_QUEUE_DEPTH = 4;          // Max pending writes

}  // namespace GpuConfig
