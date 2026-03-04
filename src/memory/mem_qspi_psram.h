/**
 * @file mem_qspi_psram.h
 * @brief QSPI CS1 memory driver — QMI XIP interface for MRAM or PSRAM.
 *
 * Uses the RP2350's built-in QMI controller on CS1 to attach external memory.
 * Supports auto-detected chip types:
 *
 *   MR10Q010 (MRAM):  128 KB, 104 MHz, no random-access penalty, non-volatile
 *   APS6408L (PSRAM): 8 MB, 133 MHz, row-buffer miss penalty, volatile
 *
 * Both use XIP memory-mapping at 0x11000000:
 *   - Reads are transparent: `uint8_t val = *(uint8_t*)0x11000000;`
 *   - QMI hardware cache (4 KB, 2-way set associative) handles caching
 *   - Writes go through the QMI write-through path
 *
 * Chip-specific differences handled by the driver:
 *
 *   Feature              MRAM (MR10Q010)        PSRAM (APS6408L)
 *   ─────────────────    ──────────────────     ──────────────────
 *   RDID command         0x4B + mode 0xFF       0x9F
 *   Init sequence        WAKE → EQPI → WREN    Reset → QPI enable
 *   Write prep           WREN (0x06) each       None (auto)
 *   Random access        Uniform (no penalty)   Row-buffer miss penalty
 *   Persistence          Non-volatile           Volatile
 *   Capacity             128 KB                 8 MB
 *   Max clock            104 MHz                133 MHz
 *   Tier policy effect   Lower base weights     Conservative weights
 *                        for random-read data   (random data stays in SRAM)
 *
 * Performance:
 *   - QMI cache hit:  ~2 cycles (nearly as fast as SRAM)
 *   - MRAM cache miss: ~100–200 ns at 104 MHz (uniform random)
 *   - PSRAM cache miss: ~100–300 ns at 133 MHz (row-buffer dependent)
 *   - Burst: 52 MB/s (MRAM) / 75 MB/s (PSRAM)
 *
 * Address map:
 *   0x10000000 — CS0: Flash (XIP, read-only)
 *   0x11000000 — CS1: MRAM/PSRAM (XIP read + write-through)
 */

#pragma once

#include <cstdint>
#include <cstddef>
#include <cstring>

// ─── QSPI CS1 Configuration ─────────────────────────────────────────────────

/// Configuration for the QSPI CS1 memory driver.
/// Fields are populated from the auto-detected QspiChipProfile at boot.
struct QspiCs1Config {
    // QMI chip select (must be CS1 — CS0 is flash)
    uint8_t  chipSelect     = 1;

    // Chip identification (set from auto-detect)
    uint8_t  chipType       = 0;   ///< QspiChipType enum value

    // QSPI clock speed (detected: MR10Q010=104, APS6408L=133)
    uint32_t clockMHz       = 104;

    // Memory capacity (detected: MR10Q010=128KB, APS6408L=8MB)
    uint32_t capacityBytes  = 128 * 1024;

    // XIP base address (fixed by RP2350 hardware)
    uintptr_t xipBase       = 0x11000000;

    // Timing parameters for QMI configuration
    uint8_t  readLatency    = 8;    ///< Dummy cycles for quad read (chip-dependent)
    uint8_t  pageSize       = 0;    ///< 0 = continuous R/W (MRAM); non-zero for PSRAM

    // RDID for verification
    uint64_t expectedRdid   = 0x076B111111ULL;

    // Chip behavioral flags (from auto-detect profile)
    bool     hasRandomAccessPenalty = false;  ///< false=MRAM, true=PSRAM
    bool     isNonVolatile         = false;   ///< true=MRAM (data survives reboot)
    bool     needsWrenBeforeWrite  = true;    ///< true=MRAM (0x06 before each write)
    bool     needsRefresh          = false;   ///< true=PSRAM (DRAM refresh)
};

// Legacy aliases for code using the old names
using QspiMramConfig  = QspiCs1Config;
using QspiPsramConfig = QspiCs1Config;

// ─── QSPI CS1 Memory Driver ────────────────────────────────────────────────

class QspiPsramDriver {
public:
    QspiPsramDriver() = default;
    ~QspiPsramDriver();

    /// Initialize QMI CS1 for the detected chip.
    /// Configures QMI timing registers based on config (clock, latency).
    /// Runs chip-specific init sequence:
    ///   MRAM:  WAKE (0xAB) → EQPI → WREN (0x06)
    ///   PSRAM: Reset (0x66/0x99) → QPI enable (0x38/0x35)
    /// Verifies chip responds to RDID.
    /// Returns true on success.
    bool Initialize(const QspiCs1Config& config);

    /// Shut down QMI CS1 configuration.
    void Shutdown();

    bool IsInitialized() const { return initialized_; }

    // ── Direct XIP Access (the main interface) ──────────────────────────

    /// Get the XIP-mapped base pointer.  Data at offset N is at base+N.
    /// The QMI hardware cache makes reads transparent to the CPU.
    ///
    /// Example:
    ///   uint8_t* base = qspi.GetXipBase();
    ///   uint32_t texel = *(uint16_t*)(base + textureOffset);
    ///
    uint8_t* GetXipBase() const {
        return reinterpret_cast<uint8_t*>(config_.xipBase);
    }

    /// Get a typed pointer to data at `offset` in QSPI MRAM.
    template <typename T>
    const T* Map(uint32_t offset) const {
        return reinterpret_cast<const T*>(config_.xipBase + offset);
    }

    /// Writable map (write-through — goes to MRAM immediately, no WIP delay).
    template <typename T>
    T* MapWritable(uint32_t offset) {
        return reinterpret_cast<T*>(config_.xipBase + offset);
    }

    // ── Allocation ──────────────────────────────────────────────────────

    /// Allocate `size` bytes from QSPI address space.
    /// Returns byte offset from XIP base, or 0xFFFFFFFF on OOM.
    /// Uses a first-fit free-list allocator with block coalescing.
    uint32_t Alloc(uint32_t size, uint32_t alignment = 4);

    /// Free a previously allocated block at `offset`.
    /// Coalesces with adjacent free blocks to reduce fragmentation.
    void Free(uint32_t offset);

    /// Free all allocations (reset allocator to fully free).
    void FreeAll();

    /// Bytes remaining (sum of all free blocks).
    uint32_t Available() const;

    // ── Bulk Write (for initial data upload) ────────────────────────────

    /// Read data from QSPI memory via XIP memory-mapping.
    /// Transparent memcpy from the XIP-mapped address (QMI HW cache handles caching).
    void Read(uint32_t offset, void* dst, uint32_t length) const {
        std::memcpy(dst,
                    reinterpret_cast<const uint8_t*>(config_.xipBase) + offset,
                    length);
    }

    /// Write data to QSPI memory via XIP write-through.
    /// Memcpy to the XIP-mapped address.
    /// MRAM: WREN was issued during InitChip(); the QMI write-through path
    ///       handles subsequent writes at bus speed (no WIP delay).
    ///       The WREN latch remains active in QPI mode — no re-issue needed
    ///       for bursts routed through the QMI controller.
    /// PSRAM: writes complete at bus speed (no WREN needed).
    void Write(uint32_t offset, const void* src, uint32_t length) {
        std::memcpy(reinterpret_cast<uint8_t*>(config_.xipBase) + offset,
                    src, length);
    }

    // ── Cache Management ────────────────────────────────────────────────

    /// Invalidate the QMI hardware cache.
    /// Call after bulk writes to ensure subsequent reads see updated data.
    void InvalidateCache();

    /// Flush the QMI write buffer (ensure all writes are committed).
    void FlushWrites();

    // ── Diagnostics ─────────────────────────────────────────────────────

    uint32_t GetCapacity() const { return config_.capacityBytes; }
    uintptr_t GetXipBaseAddr() const { return config_.xipBase; }
    bool HasRandomAccessPenalty() const { return config_.hasRandomAccessPenalty; }
    bool IsNonVolatile() const { return config_.isNonVolatile; }

private:
    QspiCs1Config config_{};
    bool          initialized_ = false;

    // ── Free-list allocator ──────────────────────────────────────────────
    static constexpr uint16_t MAX_FREE_BLOCKS = 64;

    struct FreeBlock {
        uint32_t addr = 0;
        uint32_t size = 0;
    };
    FreeBlock freeList_[MAX_FREE_BLOCKS];
    uint16_t  freeBlockCount_ = 0;

    void InitFreeList();
    void CoalesceFreeList();

    /// Configure QMI timing registers for the detected chip.
    bool ConfigureQmiTiming();

    /// Send chip-specific initialization sequence.
    /// MRAM path:  WAKE (0xAB) → EQPI → WREN (0x06)
    /// PSRAM path: Reset (0x66/0x99) → QPI enable (0x38/0x35)
    bool InitChip();

    /// Read chip ID via RDID.
    /// MRAM:  cmd 0x4B + mode byte 0xFF → 5 response bytes
    /// PSRAM: cmd 0x9F → 3 response bytes
    bool VerifyChipId();
};

