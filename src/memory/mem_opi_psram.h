/**
 * @file mem_opi_psram.h
 * @brief OPI PSRAM driver — PIO2-based 8-bit DDR PSRAM interface.
 *
 * Uses PIO2 (the last free PIO block) to drive an AP Memory APS6408L or
 * similar OPI PSRAM chip.  All data access is **indirect** — CPU cannot
 * read/write OPI PSRAM through normal memory loads/stores.  Instead:
 *
 *   Write: CPU → DMA → PIO2 TX FIFO → OPI PSRAM
 *   Read:  OPI PSRAM → PIO2 RX FIFO → DMA → SRAM buffer
 *
 * This driver exposes simple Read/Write/DMA-async operations that the
 * MemTierManager uses to shuttle data between SRAM cache lines and OPI PSRAM.
 *
 * Hardware:
 *   - PIO2, 2 state machines (SM0 = command/write, SM1 = read)
 *   - 2 DMA channels (one TX, one RX)
 *   - 8 data GPIOs (directly driven by PIO, directly sampled for reads)
 *   - 1 CLK GPIO (side-set from PIO)
 *   - 1 CS GPIO (set-pin from PIO)
 *
 * GPIO Assignment (provisional — adjust in gpu_config.h):
 *   GPIO 11 — PSRAM CS#
 *   GPIO 12 — PSRAM CLK
 *   GPIO 34–41 — PSRAM DQ0–DQ7
 *   (RP2350 GPIOs 30-47 are available on QFN-80 package)
 *
 * Performance characteristics:
 *   - Burst read:  ~120–150 MB/s (150 MHz DDR, 8-bit)
 *   - Burst write: ~120–150 MB/s
 *   - Random access: ~200 ns latency per command (6 bytes command overhead)
 *   - DMA transfer: zero CPU overhead during bulk transfer
 *
 * Protocol (APS6408L OPI mode):
 *   Command phase: 1 byte opcode on DQ[7:0]
 *   Address phase: 3 bytes address on DQ[7:0]
 *   Latency phase: configurable wait cycles (typically 3–6)
 *   Data phase:    N bytes on DQ[7:0], DDR (both edges)
 *
 * ┌─────────────────────────────────────────────────────────────────────┐
 * │                    OPI PSRAM Bus Timing                            │
 * │                                                                    │
 * │  CLK:  ─┐ ┌─┐ ┌─┐ ┌─┐ ┌─┐ ┌─┐ ┌─┐ ┌─┐ ┌─               │
 * │         └─┘ └─┘ └─┘ └─┘ └─┘ └─┘ └─┘ └─┘                  │
 * │  CS:   ──┐                                    ┌──           │
 * │          └────────────────────────────────────┘             │
 * │  DQ:   CMD  ADDR[2:0]  WAIT...  DATA[N-1:0]               │
 * │        (1B)  (3B)      (lat)    (burst)                     │
 * └─────────────────────────────────────────────────────────────────────┘
 */

#pragma once

#include <cstdint>
#include <cstddef>

// ─── OPI PSRAM Configuration ────────────────────────────────────────────────

struct OpiPsramConfig {
    // PIO instance (must be pio2 — pio0=HUB75, pio1=OctalSPI RX)
    uint8_t  pioInstance    = 2;

    // Data pins (8 consecutive GPIOs)
    uint8_t  dataBasePin    = 34;   // DQ0 = GPIO34, DQ7 = GPIO41
    uint8_t  dataPinCount   = 8;

    // Control pins
    uint8_t  clkPin         = 12;   // PSRAM CLK
    uint8_t  csPin          = 11;   // PSRAM CS# (active low)

    // Timing
    uint8_t  readLatency    = 5;    // Wait cycles between address and read data
    uint8_t  writeLatency   = 0;    // Wait cycles between address and write data
    uint32_t clockMHz       = 75;   // PIO clock divisor target (75–150 MHz)

    // Capacity
    uint32_t capacityBytes  = 8 * 1024 * 1024;  // 8 MB default (APS6408L = 8 MB)
};

// ─── DMA Transfer Status ────────────────────────────────────────────────────

enum class OpiDmaStatus : uint8_t {
    IDLE     = 0,
    READING  = 1,
    WRITING  = 2,
    COMPLETE = 3,
    ERROR    = 4,
};

// ─── OPI PSRAM Driver ───────────────────────────────────────────────────────

class OpiPsramDriver {
public:
    OpiPsramDriver() = default;
    ~OpiPsramDriver();

    /// Initialize PIO2 programs, DMA channels, and GPIO.
    /// Returns true on success.
    bool Initialize(const OpiPsramConfig& config);

    /// Shut down PIO/DMA and release resources.
    void Shutdown();

    bool IsInitialized() const { return initialized_; }

    // ── Synchronous (blocking) Transfers ────────────────────────────────

    /// Read `length` bytes from OPI PSRAM address `srcAddr` into `dest`.
    /// Blocks until DMA completes.  Use for small/urgent reads.
    bool ReadSync(uint32_t srcAddr, void* dest, uint32_t length);

    /// Write `length` bytes from `src` to OPI PSRAM address `destAddr`.
    /// Blocks until DMA completes.
    bool WriteSync(uint32_t destAddr, const void* src, uint32_t length);

    // ── Asynchronous (non-blocking) DMA Transfers ───────────────────────

    /// Start a DMA read from OPI PSRAM into SRAM.  Returns immediately.
    /// Check status with GetDmaStatus() or wait with WaitDma().
    bool ReadAsync(uint32_t srcAddr, void* dest, uint32_t length);

    /// Start a DMA write from SRAM to OPI PSRAM.  Returns immediately.
    bool WriteAsync(uint32_t destAddr, const void* src, uint32_t length);

    /// Poll DMA completion status.
    OpiDmaStatus GetDmaStatus() const;

    /// Block until current DMA transfer completes.
    void WaitDma();

    // ── Bulk Prefetch (used by MemTierManager) ──────────────────────────

    /// Queue a prefetch: read OPI data into the given SRAM cache line.
    /// Non-blocking.  The cache line's `locked` flag is set until complete.
    bool Prefetch(uint32_t srcAddr, uint8_t* sramDest, uint32_t length);

    // ── Allocation ──────────────────────────────────────────────────────

    /// Simple bump allocator for OPI address space.
    /// Returns byte address in OPI PSRAM, or 0xFFFFFFFF on OOM.
    uint32_t Alloc(uint32_t size, uint32_t alignment = 4);

    /// Free all allocations (reset bump pointer).
    void FreeAll();

    /// Bytes remaining in OPI PSRAM.
    uint32_t Available() const;

    // ── Diagnostics ─────────────────────────────────────────────────────

    uint32_t GetTotalReads() const  { return totalReads_; }
    uint32_t GetTotalWrites() const { return totalWrites_; }
    uint32_t GetCapacity() const    { return config_.capacityBytes; }

private:
    OpiPsramConfig config_{};
    bool           initialized_ = false;

    // PIO / DMA handles (RP2350-specific, opaque here)
    uint32_t pioProgOffset_ = 0;
    uint8_t  smCommand_     = 0;   // SM0: command/write
    uint8_t  smRead_        = 0;   // SM1: read data
    int      dmaChannelTx_  = -1;
    int      dmaChannelRx_  = -1;

    // Bump allocator state
    uint32_t allocNext_     = 0;

    // Statistics
    uint32_t totalReads_    = 0;
    uint32_t totalWrites_   = 0;

    // Internal: issue OPI command sequence via PIO
    void IssueCommand(uint8_t opcode, uint32_t address);
    void SetDataDirection(bool output);
};

