/**
 * @file mem_opi_psram.h
 * @brief PIO2 external memory driver — supports OPI PSRAM and QSPI MRAM modes.
 *
 * Uses PIO2 (the last free PIO block) for Tier 1 external memory access.
 * All data access is **indirect** — CPU cannot read/write through memory
 * loads/stores.  Instead:
 *
 *   Write: CPU → DMA → PIO2 TX FIFO → External chip
 *   Read:  External chip → PIO2 RX FIFO → DMA → SRAM buffer
 *
 * Three operating modes (selected via gpu_config.h `PIO2_MEM_MODE`):
 *
 *   Mode              Bus     Chips  Capacity  Bandwidth   Random Access
 *   ────────────────  ──────  ─────  ────────  ──────────  ─────────────
 *   OPI_PSRAM         8-bit   1×     8 MB      ~150 MB/s   Row-buffer penalty
 *   DUAL_QSPI_MRAM   4-bit   2×     256 KB    ~52 MB/s×2  Uniform (no penalty)
 *   SINGLE_QSPI_MRAM 4-bit   1×     128 KB    ~52 MB/s    Uniform (no penalty)
 *
 * ── OPI PSRAM Mode (APS6408L) ──────────────────────────────────────────
 *
 *   GPIO 11 — PSRAM CS#
 *   GPIO 12 — PSRAM CLK
 *   GPIO 34–41 — PSRAM DQ0–DQ7 (8 data lines)
 *
 *   Protocol: 1B opcode + 3B address + latency wait + N bytes data (DDR)
 *   Burst read/write: ~120–150 MB/s at 150 MHz DDR
 *
 * ── QSPI MRAM Mode (MR10Q010) ─────────────────────────────────────────
 *
 *   GPIO 11 — MRAM CS0#     (chip 0, always present)
 *   GPIO 38 — MRAM CS1#     (chip 1, dual mode only; reuses OPI DQ4 pin)
 *   GPIO 12 — MRAM CLK
 *   GPIO 34–37 — MRAM DQ0–DQ3 (4 data lines, quad SPI)
 *
 *   Protocol: MRAM uses standard SPI commands:
 *     Read:  0xEB (Quad Read) + 3B addr + 8 dummy cycles + N bytes
 *     Write: 0x06 (WREN) then 0x38 (Quad Write) + 3B addr + N bytes
 *     RDID:  0x4B + mode 0xFF → 5 response bytes (0x076B111111)
 *
 *   Dual-chip addressing:
 *     Address bit 17 selects the chip:
 *       0x00000–0x1FFFF → CS0 (chip 0, 128 KB)
 *       0x20000–0x3FFFF → CS1 (chip 1, 128 KB)
 *     Driver asserts the appropriate CS based on address range.
 *
 *   Key advantages over PSRAM:
 *     - No random-access penalty (uniform latency for any address)
 *     - Non-volatile (data survives power cycle — persistent LUTs, fonts)
 *     - Unlimited write endurance (no wear-out, unlike flash)
 *     - Simpler DMA scheduling (no row-buffer miss stalls)
 *
 * ── Bus Timing ─────────────────────────────────────────────────────────
 *
 * ┌─────────────────────────────────────────────────────────────────────┐
 * │  OPI PSRAM:                                                        │
 * │  CLK:  ─┐ ┌─┐ ┌─┐ ┌─┐ ┌─┐ ┌─┐ ┌─┐ ┌─┐ ┌─               │
 * │         └─┘ └─┘ └─┘ └─┘ └─┘ └─┘ └─┘ └─┘                  │
 * │  CS:   ──┐                                    ┌──           │
 * │          └────────────────────────────────────┘             │
 * │  DQ[7:0]: CMD  ADDR[2:0]  WAIT...  DATA[N-1:0]            │
 * │           (1B)  (3B)      (lat)    (burst, DDR)             │
 * ├─────────────────────────────────────────────────────────────────────┤
 * │  QSPI MRAM:                                                       │
 * │  CLK:  ─┐ ┌─┐ ┌─┐ ┌─┐ ┌─┐ ┌─┐ ┌─┐ ┌─┐ ┌─               │
 * │         └─┘ └─┘ └─┘ └─┘ └─┘ └─┘ └─┘ └─┘                  │
 * │  CS0/1: ─┐                                    ┌──           │
 * │          └────────────────────────────────────┘             │
 * │  DQ[3:0]: CMD   ADDR[2:0]  DUMMY(8)  DATA[N-1:0]          │
 * │           (2c)   (6c)      (8c)      (N/2 clocks)          │
 * └─────────────────────────────────────────────────────────────────────┘
 */

#pragma once

#include <cstdint>
#include <cstddef>
#include <cstring>

// ─── DMA completion callback type ───────────────────────────────────────────

/// Callback invoked from DMA IRQ when an async transfer completes.
/// @param ok      true if the transfer completed without error
/// @param userCtx Opaque context pointer set by caller
typedef void (*OpiDmaCallback)(bool ok, void* userCtx);

// ─── PIO2 External Memory Configuration ─────────────────────────────────────

/// Operating mode for the PIO2 external memory driver.
/// Must match gpu_config.h Pio2MemMode.
enum class Pio2MemMode : uint8_t {
    NONE              = 0,
    OPI_PSRAM         = 1,
    DUAL_QSPI_MRAM   = 2,
    SINGLE_QSPI_MRAM = 3,
};

struct Pio2MemConfig {
    // ── Mode selection ──────────────────────────────────────────────────
    Pio2MemMode mode = Pio2MemMode::NONE;

    // PIO instance (must be pio2 — pio0=HUB75, pio1=OctalSPI RX)
    uint8_t  pioInstance    = 2;

    // ── Data pins ───────────────────────────────────────────────────────
    uint8_t  dataBasePin    = 34;   // DQ0 = GPIO34
    uint8_t  dataPinCount   = 8;    // 8 for OPI, 4 for QSPI (set from mode)

    // ── Control pins ────────────────────────────────────────────────────
    uint8_t  clkPin         = 12;   // CLK (all modes)
    uint8_t  cs0Pin         = 11;   // CS# (OPI/single MRAM) or CS0# (dual MRAM)
    uint8_t  cs1Pin         = 38;   // CS1# (dual MRAM only, reuses OPI DQ4 pin)

    // ── Timing ──────────────────────────────────────────────────────────
    uint8_t  readLatency    = 5;    // Wait cycles: OPI=5, MRAM=8
    uint8_t  writeLatency   = 0;    // Wait cycles for writes (OPI only)
    uint32_t clockMHz       = 75;   // PIO clock: OPI=75–150, MRAM=104

    // ── Capacity ────────────────────────────────────────────────────────
    uint32_t capacityBytes  = 8 * 1024 * 1024;  // Total: 8 MB OPI / 256 KB dual / 128 KB single
    uint32_t chipCapacity   = 0;    // Per-chip capacity (dual MRAM only)
    uint8_t  chipCount      = 1;    // 1 for OPI/single, 2 for dual MRAM

    // ── MRAM-specific ───────────────────────────────────────────────────
    bool     isMram              = false;  ///< true when mode is *_QSPI_MRAM
    bool     hasRandomAccessPenalty = true; ///< false for MRAM, true for PSRAM
    bool     isNonVolatile       = false;  ///< true for MRAM
    bool     needsWrenBeforeWrite = false; ///< true for MRAM (0x06 before each write)
    uint64_t expectedRdid        = 0;     ///< MRAM RDID: 0x076B111111
};

// Legacy alias for backward compatibility
using OpiPsramConfig = Pio2MemConfig;

// ─── DMA Transfer Status ────────────────────────────────────────────────────

enum class OpiDmaStatus : uint8_t {
    IDLE     = 0,
    READING  = 1,
    WRITING  = 2,
    COMPLETE = 3,
    ERROR    = 4,
};

// ─── PIO2 External Memory Driver ────────────────────────────────────────────

class OpiPsramDriver {
public:
    OpiPsramDriver() = default;
    ~OpiPsramDriver();

    /// Initialize PIO2 programs, DMA channels, and GPIO.
    /// Selects PIO program and pin configuration based on config.mode:
    ///   OPI_PSRAM:         Load 8-bit OPI program, configure DQ0–DQ7
    ///   DUAL_QSPI_MRAM:   Load 4-bit QSPI program, configure DQ0–DQ3 + CS0 + CS1
    ///   SINGLE_QSPI_MRAM: Load 4-bit QSPI program, configure DQ0–DQ3 + CS0
    /// Returns true on success.
    bool Initialize(const Pio2MemConfig& config);

    /// Shut down PIO/DMA and release resources.
    void Shutdown();

    bool IsInitialized() const { return initialized_; }
    Pio2MemMode GetMode() const { return config_.mode; }

    // ── Synchronous (blocking) Transfers ────────────────────────────────

    /// Read `length` bytes from external address `srcAddr` into `dest`.
    /// In dual MRAM mode, address bit 17 selects the chip automatically.
    /// Blocks until DMA completes.
    bool ReadSync(uint32_t srcAddr, void* dest, uint32_t length);

    /// Write `length` bytes from `src` to external address `destAddr`.
    /// MRAM modes: issues WREN (0x06) before write automatically.
    /// In dual MRAM mode, address bit 17 selects the chip.
    /// Blocks until DMA completes.
    bool WriteSync(uint32_t destAddr, const void* src, uint32_t length);

    // ── Asynchronous (non-blocking) DMA Transfers ───────────────────────

    /// Start a DMA read into SRAM.  Returns immediately.
    /// Check status with GetDmaStatus() or wait with WaitDma().
    /// Cross-chip boundary reads in dual MRAM mode are now handled via
    /// DMA chaining (first chip completes, second chip starts automatically).
    bool ReadAsync(uint32_t srcAddr, void* dest, uint32_t length);

    /// Start a DMA write from SRAM.  Returns immediately.
    bool WriteAsync(uint32_t destAddr, const void* src, uint32_t length);

    /// Poll DMA completion status.
    OpiDmaStatus GetDmaStatus() const;

    /// Block until current DMA transfer completes.
    void WaitDma();

    // ── IRQ-Based DMA Completion Callback ────────────────────────────────

    /// Set a callback to be invoked from DMA IRQ when the current async
    /// transfer completes.  Set to nullptr to disable (revert to polling).
    /// The callback runs in IRQ context — keep it short (flag, semaphore give).
    void SetDmaCallback(OpiDmaCallback cb, void* userCtx = nullptr);

    /// Clear any pending callback.
    void ClearDmaCallback();

    // ── Bulk Prefetch (used by MemTierManager) ──────────────────────────

    /// Queue a prefetch: read external data into the given SRAM cache line.
    /// Non-blocking.  The cache line's `locked` flag is set until complete.
    bool Prefetch(uint32_t srcAddr, uint8_t* sramDest, uint32_t length);

    // ── Allocation ──────────────────────────────────────────────────────

    /// Allocate `size` bytes from external address space.
    /// Returns byte address, or 0xFFFFFFFF on OOM.
    /// Uses a first-fit free-list allocator with block coalescing.
    /// Dual MRAM: allocates across both chips (addr bit 17 = chip select).
    uint32_t Alloc(uint32_t size, uint32_t alignment = 4);

    /// Free a previously allocated block at `addr`.
    /// Coalesces with adjacent free blocks to reduce fragmentation.
    /// Pass the exact address returned by Alloc().
    void Free(uint32_t addr);

    /// Free all allocations (reset the allocator to fully free).
    void FreeAll();

    /// Bytes remaining (sum of all free blocks).
    uint32_t Available() const;

    // ── Diagnostics ─────────────────────────────────────────────────────

    uint32_t GetTotalReads() const  { return totalReads_; }
    uint32_t GetTotalWrites() const { return totalWrites_; }
    uint32_t GetCapacity() const    { return config_.capacityBytes; }
    uint8_t  GetChipCount() const   { return config_.chipCount; }
    bool     IsMram() const         { return config_.isMram; }
    bool     HasRandomAccessPenalty() const { return config_.hasRandomAccessPenalty; }
    bool     IsNonVolatile() const  { return config_.isNonVolatile; }

private:
    Pio2MemConfig config_{};
    bool          initialized_ = false;
    bool          rdidVerified_ = false;  ///< True if RDID matched expected value

    // PIO / DMA handles (RP2350-specific, opaque here)
    uint32_t pioProgOffset_ = 0;
    uint8_t  smCommand_     = 0;   // SM0: command/write
    uint8_t  smRead_        = 0;   // SM1: read data
    int      dmaChannelTx_  = -1;
    int      dmaChannelRx_  = -1;

    // DMA IRQ callback state
    OpiDmaCallback dmaCallback_   = nullptr;
    void*          dmaCallbackCtx_ = nullptr;
    uint8_t        dmaIrqIndex_    = 0;  ///< 0 or 1: which DMA IRQ bank we claimed
    bool           irqConfigured_  = false;

    // Dual-chip async state (for cross-boundary reads)
    struct PendingChip1 {
        uint32_t srcAddr   = 0;
        uint8_t* dest      = nullptr;
        uint32_t length    = 0;
        uint8_t  csPin     = 0;
        bool     pending   = false;
    } pendingChip1_;       ///< Deferred second-chip read in dual MRAM mode

    // ── Free-list allocator ──────────────────────────────────────────────
    //
    // Manages external address space as a linked list of free blocks.
    // Coalesces adjacent free blocks on Free() to reduce fragmentation.
    //
    static constexpr uint16_t MAX_FREE_BLOCKS = 64;

    struct FreeBlock {
        uint32_t addr = 0;
        uint32_t size = 0;
    };
    FreeBlock freeList_[MAX_FREE_BLOCKS];
    uint16_t  freeBlockCount_ = 0;

    void InitFreeList();
    void CoalesceFreeList();

    // Statistics
    uint32_t totalReads_    = 0;
    uint32_t totalWrites_   = 0;

    // ── Mode-specific internal methods ──────────────────────────────────

    /// OPI PSRAM: issue 8-bit OPI command sequence via PIO
    void IssueOpiCommand(uint8_t opcode, uint32_t address);

    /// QSPI MRAM: issue quad SPI command via PIO
    /// chipIndex: 0 or 1 (selects CS0 or CS1 in dual mode)
    void IssueQspiCommand(uint8_t opcode, uint32_t address, uint8_t chipIndex = 0);

    /// QSPI MRAM: send WREN (0x06) in SPI mode before a write
    void IssueMramWren(uint8_t chipIndex = 0);

    /// Select which chip CS to assert based on address (dual MRAM only)
    /// Returns: 0 for CS0 (addr < chipCapacity), 1 for CS1
    uint8_t ChipIndexForAddress(uint32_t addr) const;

    /// Get the intra-chip address (strip the chip-select bit for dual mode)
    uint32_t ChipLocalAddress(uint32_t addr) const;

    /// Switch PIO data direction (output for writes, input for reads)
    void SetDataDirection(bool output);

    /// Load the appropriate PIO program based on mode
    bool LoadPioProgram();

    /// Verify chip identity via RDID command (PIO-based).
    /// OPI PSRAM: 0x9F → manufacturer ID check
    /// QSPI MRAM: 0x4B + mode 0xFF → 5-byte RDID check
    bool VerifyChipId();

    /// Setup DMA IRQ handler for completion callbacks.
    void SetupDmaIrq();

    /// Static IRQ trampoline — routes to the active driver instance.
    static void DmaIrqHandler();

    /// The singleton instance pointer for IRQ routing.
    static OpiPsramDriver* s_irqInstance_;

    /// Start the second-chip portion of a cross-boundary async read.
    void StartPendingChip1Read();
};

