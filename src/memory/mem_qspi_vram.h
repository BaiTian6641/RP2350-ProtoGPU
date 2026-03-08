/**
 * @file mem_qspi_vram.h
 * @brief Unified QSPI VRAM driver — dual-channel PIO2 interface for MRAM/PSRAM.
 *
 * Replaces the former mem_opi_psram.h (PIO2 OPI driver) and mem_qspi_psram.h
 * (QMI CS1 XIP driver).  All external VRAM is now PIO2-driven indirect
 * (no XIP memory-mapping).
 *
 * Hardware topology (RP2350B QFN-80):
 *
 *   Channel A (Tier 1 — QSPI_A):
 *     PIO2 SM0 (cmd/write) + SM1 (read), data GPIO 34-37 (4-bit),
 *     CLK GPIO 12, CS0 GPIO 11, CS1 GPIO 38.
 *     Up to 2 chips on CS0+CS1, each auto-detected.
 *
 *   Channel B (Tier 2 — QSPI_B):
 *     PIO2 SM2 (cmd/write) + SM3 (read), data GPIO 39-42 (4-bit),
 *     CLK GPIO 43, CS0 GPIO 44, CS1 GPIO 45.
 *     Up to 2 chips on CS0+CS1, each auto-detected.
 *
 * Per-chip-select auto-detection at boot:
 *   1. MRAM RDID  (0x4B + mode 0xFF) → MR10Q010: 128 KB, 104 MHz, non-volatile
 *   2. PSRAM RDID (0x9F)              → APS6408L: 8 MB, 133 MHz, volatile
 *
 * Maximum configuration: 2 channels × 2 CS = 4 chips.
 * Mixed MRAM/PSRAM per channel is supported.
 *
 * All access is indirect via DMA:
 *   QspiVramDriver::ReadSync(channel, addr, dest, len)
 *   QspiVramDriver::WriteSync(channel, addr, src, len)
 *   QspiVramDriver::ReadAsync(channel, addr, dest, len)  // non-blocking
 *
 * QSPI bus timing (per channel):
 *   CLK:  ─┐ ┌─┐ ┌─┐ ┌─┐ ┌─┐ ┌─┐ ┌─┐ ┌─┐ ┌─
 *          └─┘ └─┘ └─┘ └─┘ └─┘ └─┘ └─┘ └─┘
 *   CS:   ─┐                                    ┌──
 *          └────────────────────────────────────┘
 *   DQ[3:0]: CMD   ADDR[2:0]  DUMMY(N)  DATA[...]
 *            (2c)   (6c)      (chip)    (burst)
 *
 * RP2350A (QFN-60, 30 GPIO): No GPIO 34+.  Set QSPI_VRAM_MODE = NONE.
 * RP2350B (QFN-80, 48 GPIO): Full 2×2 support.
 */

#pragma once

#include <cstdint>
#include <cstddef>
#include <cstring>

// ─── Constants ──────────────────────────────────────────────────────────────

/// Maximum number of QSPI channels (A + B).
static constexpr uint8_t QSPI_VRAM_MAX_CHANNELS = 2;

/// Maximum chip-selects per channel.
static constexpr uint8_t QSPI_VRAM_MAX_CS_PER_CHANNEL = 2;

/// Total maximum chips across all channels.
static constexpr uint8_t QSPI_VRAM_MAX_CHIPS =
    QSPI_VRAM_MAX_CHANNELS * QSPI_VRAM_MAX_CS_PER_CHANNEL;

// ─── DMA Completion Callback ────────────────────────────────────────────────

/// Callback invoked from DMA IRQ when an async transfer completes.
/// @param ok        true if transfer completed without error
/// @param channel   0 = Channel A, 1 = Channel B
/// @param userCtx   Opaque context pointer set by caller
typedef void (*QspiVramDmaCallback)(bool ok, uint8_t channel, void* userCtx);

// ─── QSPI VRAM Channel ID ──────────────────────────────────────────────────

enum class QspiChannel : uint8_t {
    A = 0,  ///< Channel A (Tier 1): PIO2 SM0+SM1
    B = 1,  ///< Channel B (Tier 2): PIO2 SM2+SM3
};

// ─── Per-Chip Configuration ─────────────────────────────────────────────────

/// Configuration for a single chip on a chip-select line.
/// Populated from auto-detection at boot (RDID probe).
struct QspiVramChipConfig {
    uint8_t  chipType       = 0;       ///< QspiChipType enum value (from gpu_config.h)
    uint32_t capacityBytes  = 0;       ///< Chip capacity in bytes
    uint32_t clockMHz       = 0;       ///< Operational QSPI clock
    uint8_t  readLatency    = 0;       ///< Dummy cycles for quad read
    bool     isMram              = false;
    bool     hasRandomAccessPenalty = true;
    bool     isNonVolatile       = false;
    bool     needsWrenBeforeWrite = false;
    uint64_t expectedRdid        = 0;
    bool     detected            = false;  ///< true if chip responded to RDID
};

// ─── Per-Channel Configuration ──────────────────────────────────────────────

/// Configuration for one QSPI channel (A or B).
struct QspiVramChannelConfig {
    QspiChannel channel     = QspiChannel::A;

    // PIO instance (must be pio2 — pio0=HUB75, pio1=OctalSPI RX)
    uint8_t  pioInstance    = 2;

    // ── Pin assignments ─────────────────────────────────────────────────
    uint8_t  dataBasePin    = 34;   // DQ0 base GPIO
    uint8_t  dataPinCount   = 4;    // Always 4 for QSPI
    uint8_t  clkPin         = 12;
    uint8_t  cs0Pin         = 11;
    uint8_t  cs1Pin         = 38;   // 0xFF if no second chip-select

    // ── Chip count (0, 1, or 2 populated CS lines) ──────────────────────
    uint8_t  chipCount      = 0;

    // ── Per-chip profiles (populated by auto-detect) ────────────────────
    QspiVramChipConfig chips[QSPI_VRAM_MAX_CS_PER_CHANNEL];

    // ── Aggregate capacity (sum of detected chips) ──────────────────────
    uint32_t totalCapacity  = 0;
};

// ─── DMA Transfer Status ────────────────────────────────────────────────────

enum class QspiVramDmaStatus : uint8_t {
    IDLE     = 0,
    READING  = 1,
    WRITING  = 2,
    COMPLETE = 3,
    ERROR    = 4,
};

// ─── Legacy Aliases ─────────────────────────────────────────────────────────

using OpiDmaCallback = QspiVramDmaCallback;
using OpiDmaStatus   = QspiVramDmaStatus;

// ─── Unified QSPI VRAM Driver ──────────────────────────────────────────────

/**
 * @brief Unified dual-channel QSPI VRAM driver using PIO2.
 *
 * Manages Channel A (SM0+SM1) and optionally Channel B (SM2+SM3).
 * Each channel supports 1–2 chip-selects with auto-detected MRAM/PSRAM chips.
 *
 * Replaces the former `OpiPsramDriver` (PIO2 OPI/QSPI) and
 * `QspiPsramDriver` (QMI CS1 XIP).  All external memory now uses PIO2-driven
 * DMA transfers through this single driver.
 *
 * Usage:
 *   QspiVramDriver vram;
 *   QspiVramChannelConfig cfgA = { ... };  // from gpu_config.h + auto-detect
 *   vram.InitChannel(cfgA);
 *   vram.ReadSync(QspiChannel::A, 0x0000, dest, 4096);
 */
class QspiVramDriver {
public:
    QspiVramDriver() = default;
    ~QspiVramDriver();

    // ── Channel Initialization ──────────────────────────────────────────

    /// Initialize one QSPI channel.
    /// Loads PIO2 programs, configures DMA, probes chip-selects.
    /// Can be called once per channel (A then B, or A only for SINGLE_CHANNEL).
    /// Returns true on success.
    bool InitChannel(const QspiVramChannelConfig& config);

    /// Shut down a channel — release PIO/DMA/GPIO resources.
    void ShutdownChannel(QspiChannel ch);

    /// Shut down all channels.
    void Shutdown();

    bool IsChannelInitialized(QspiChannel ch) const {
        return channels_[static_cast<uint8_t>(ch)].initialized;
    }
    bool IsAnyChannelInitialized() const {
        return channels_[0].initialized || channels_[1].initialized;
    }

    // ── Synchronous (blocking) Transfers ────────────────────────────────

    /// Read `length` bytes from channel's external address into `dest`.
    /// In dual-CS mode, address ranges are mapped across chips:
    ///   addr < chip0.capacity → CS0
    ///   addr >= chip0.capacity → CS1 (shifted by chip0.capacity)
    /// Blocks until DMA completes.
    bool ReadSync(QspiChannel ch, uint32_t srcAddr,
                  void* dest, uint32_t length);

    /// Write `length` bytes to channel's external address.
    /// MRAM chips: WREN (0x06) issued automatically before each write.
    /// Blocks until DMA completes.
    bool WriteSync(QspiChannel ch, uint32_t destAddr,
                   const void* src, uint32_t length);

    // ── Asynchronous (non-blocking) DMA Transfers ───────────────────────

    /// Start a DMA read.  Returns immediately.
    /// Cross-chip boundary reads are handled via DMA chaining.
    bool ReadAsync(QspiChannel ch, uint32_t srcAddr,
                   void* dest, uint32_t length);

    /// Start a DMA write.  Returns immediately.
    bool WriteAsync(QspiChannel ch, uint32_t destAddr,
                    const void* src, uint32_t length);

    /// Poll DMA completion status for a channel.
    QspiVramDmaStatus GetDmaStatus(QspiChannel ch) const;

    /// Block until current DMA transfer on a channel completes.
    void WaitDma(QspiChannel ch);

    // ── IRQ-Based DMA Completion Callback ────────────────────────────────

    /// Set a callback for DMA completion on a channel.
    /// Runs in IRQ context — keep it short.
    void SetDmaCallback(QspiChannel ch, QspiVramDmaCallback cb,
                        void* userCtx = nullptr);

    /// Clear pending callback for a channel.
    void ClearDmaCallback(QspiChannel ch);

    // ── Bulk Prefetch (used by MemTierManager) ──────────────────────────

    /// Queue a prefetch: non-blocking DMA read into SRAM cache line.
    bool Prefetch(QspiChannel ch, uint32_t srcAddr,
                  uint8_t* sramDest, uint32_t length);

    // ── Allocation (per-channel address space) ──────────────────────────

    /// Allocate `size` bytes from a channel's address space.
    /// Returns byte address, or 0xFFFFFFFF on OOM.
    /// Uses first-fit free-list with block coalescing.
    /// Dual CS: allocation spans both chips transparently.
    uint32_t Alloc(QspiChannel ch, uint32_t size, uint32_t alignment = 4);

    /// Free a previously allocated block.
    void Free(QspiChannel ch, uint32_t addr);

    /// Free all allocations on a channel.
    void FreeAll(QspiChannel ch);

    /// Available bytes on a channel (sum of free blocks).
    uint32_t Available(QspiChannel ch) const;

    // ── Diagnostics ─────────────────────────────────────────────────────

    uint32_t GetTotalReads(QspiChannel ch) const;
    uint32_t GetTotalWrites(QspiChannel ch) const;
    uint32_t GetCapacity(QspiChannel ch) const;
    uint8_t  GetChipCount(QspiChannel ch) const;

    /// Returns true if ALL chips on the channel are MRAM.
    bool IsMram(QspiChannel ch) const;

    /// Returns true if ANY chip on the channel has random-access penalty.
    bool HasRandomAccessPenalty(QspiChannel ch) const;

    /// Returns true if ALL chips on the channel are non-volatile.
    bool IsNonVolatile(QspiChannel ch) const;

    /// Get the per-chip config for a specific chip-select on a channel.
    const QspiVramChipConfig& GetChipConfig(QspiChannel ch, uint8_t csIndex) const;

    /// Get the full channel config.
    const QspiVramChannelConfig& GetChannelConfig(QspiChannel ch) const;

    // ── Legacy compatibility (single-channel as "OPI driver") ───────────

    /// Legacy: initialize Channel A only with OPI-style config.
    /// Internally converts to QspiVramChannelConfig and calls InitChannel().
    bool Initialize(const QspiVramChannelConfig& config) {
        return InitChannel(config);
    }

    bool IsInitialized() const { return IsAnyChannelInitialized(); }
    uint32_t GetCapacity() const { return GetCapacity(QspiChannel::A); }
    uint32_t Available() const { return Available(QspiChannel::A); }
    bool IsMram() const { return IsMram(QspiChannel::A); }
    bool HasRandomAccessPenalty() const { return HasRandomAccessPenalty(QspiChannel::A); }
    bool IsNonVolatile() const { return IsNonVolatile(QspiChannel::A); }

private:
    // ── Per-Channel Internal State ──────────────────────────────────────

    struct ChannelState {
        QspiVramChannelConfig config{};
        bool          initialized = false;
        bool          rdidVerified[QSPI_VRAM_MAX_CS_PER_CHANNEL] = {};

        // PIO / DMA handles
        uint32_t pioProgOffset = 0;
        uint8_t  smCommand     = 0;   // SM for command/write
        uint8_t  smRead        = 0;   // SM for read data
        int      dmaChannelTx  = -1;
        int      dmaChannelRx  = -1;

        // DMA IRQ callback
        QspiVramDmaCallback dmaCallback    = nullptr;
        void*               dmaCallbackCtx = nullptr;
        uint8_t             dmaIrqIndex    = 0;
        bool                irqConfigured  = false;

        // Cross-chip async state (for boundary reads in dual-CS mode)
        struct PendingChip1 {
            uint32_t srcAddr   = 0;
            uint8_t* dest      = nullptr;
            uint32_t length    = 0;
            uint8_t  csPin     = 0;
            bool     pending   = false;
        } pendingChip1;

        // ── Free-list allocator ─────────────────────────────────────────
        static constexpr uint16_t MAX_FREE_BLOCKS = 64;

        struct FreeBlock {
            uint32_t addr = 0;
            uint32_t size = 0;
        };
        FreeBlock freeList[MAX_FREE_BLOCKS];
        uint16_t  freeBlockCount = 0;

        // Statistics
        uint32_t totalReads  = 0;
        uint32_t totalWrites = 0;
    };

    ChannelState channels_[QSPI_VRAM_MAX_CHANNELS];

    // ── Free-list helpers ───────────────────────────────────────────────

    void InitFreeList(ChannelState& ch);
    void CoalesceFreeList(ChannelState& ch);

    // ── Mode-specific internal methods ──────────────────────────────────

    /// Issue QSPI command via PIO for the specified channel.
    /// chipIndex: 0 or 1 (selects CS0 or CS1)
    void IssueQspiCommand(ChannelState& ch, uint8_t opcode,
                          uint32_t address, uint8_t chipIndex = 0);

    /// MRAM: send WREN (0x06) in SPI mode before a write.
    void IssueMramWren(ChannelState& ch, uint8_t chipIndex = 0);

    /// Select chip-select index based on address (dual-CS mode).
    /// addr < chip0.capacity → CS0, else → CS1.
    uint8_t ChipIndexForAddress(const ChannelState& ch, uint32_t addr) const;

    /// Get chip-local address (strip chip0.capacity offset for CS1).
    uint32_t ChipLocalAddress(const ChannelState& ch, uint32_t addr) const;

    /// Switch PIO data direction for a channel.
    void SetDataDirection(ChannelState& ch, bool output);

    /// Load the QSPI PIO program for a channel.
    bool LoadPioProgram(ChannelState& ch);

    /// Verify chip identity via RDID for each detected CS.
    bool VerifyChipId(ChannelState& ch, uint8_t chipIndex);

    /// Setup DMA IRQ handler for completion callbacks.
    void SetupDmaIrq(ChannelState& ch);

    /// Per-channel IRQ trampoline instances.
    static void DmaIrqHandlerA();
    static void DmaIrqHandlerB();

    /// Singleton instance pointer for IRQ routing.
    static QspiVramDriver* s_irqInstance_;

    /// Start second-chip portion of cross-boundary async read.
    void StartPendingChip1Read(ChannelState& ch);
};

// ─── Legacy Aliases ─────────────────────────────────────────────────────────

/// Legacy: OpiPsramDriver is now QspiVramDriver.
/// Code that uses OpiPsramDriver can continue to compile, but should migrate.
using OpiPsramDriver  = QspiVramDriver;

/// Legacy: QspiPsramDriver is now QspiVramDriver.
/// The QMI CS1 XIP path has been removed — all external memory is PIO2-driven.
using QspiPsramDriver = QspiVramDriver;

/// Legacy: Pio2MemConfig → QspiVramChannelConfig
using Pio2MemConfig   = QspiVramChannelConfig;
using OpiPsramConfig  = QspiVramChannelConfig;
