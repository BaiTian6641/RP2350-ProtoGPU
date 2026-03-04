/**
 * @file mem_qspi_psram.h
 * @brief QSPI PSRAM driver — QMI CS1 hardware XIP interface.
 *
 * Uses the RP2350's built-in QMI (QSPI Memory Interface) controller on the
 * CS1 chip-select to attach a QSPI PSRAM chip (e.g., APS6408L in QPI mode,
 * Winbond W25Q series, or ISSI IS66WVS).
 *
 * Unlike the OPI PSRAM driver, QSPI PSRAM is **memory-mapped** via XIP:
 *   - The QMI hardware maps PSRAM into the CPU's address space at 0x11000000
 *   - Reads are transparent: `uint8_t val = *(uint8_t*)0x11000000;`
 *   - The QMI hardware cache (4 KB, 2-way set associative on RP2350) handles
 *     caching automatically — no software intervention needed
 *   - Writes go through the QMI write-through path
 *
 * This means the CPU can use normal C/C++ pointers to read QSPI PSRAM data,
 * making it ideal for frequently-accessed, non-critical data:
 *   - Lookup tables (gamma, CIE, noise permutation)
 *   - Font atlases
 *   - Inactive mesh cold storage
 *   - Animation keyframe data
 *
 * Performance characteristics:
 *   - QMI cache hit:  ~2 cycles (nearly as fast as SRAM)
 *   - QMI cache miss: ~100–300 ns (4-bit QSPI read at 75 MHz, + latency)
 *   - Burst read:     ~37.5 MB/s (75 MHz, 4-bit, SDR) or ~75 MB/s (DDR)
 *   - Burst write:    ~37.5 MB/s (write-through, no cache benefit)
 *
 * The QMI hardware cache means that a hot working set (≤4 KB) is served at
 * near-SRAM speed.  For data with good spatial locality (sequential reads,
 * small lookup tables), effective throughput approaches SRAM.
 *
 * ┌─────────────────────────────────────────────────────────────────────┐
 * │                    QSPI PSRAM Address Map                          │
 * │                                                                    │
 * │  0x10000000 — CS0: Flash (XIP, read-only, code + const data)      │
 * │  0x11000000 — CS1: PSRAM (XIP read + write-through)               │
 * │             └─ 0x11000000 + capacity (e.g., 0x11800000 for 8 MB)  │
 * └─────────────────────────────────────────────────────────────────────┘
 */

#pragma once

#include <cstdint>
#include <cstddef>

// ─── QSPI PSRAM Configuration ───────────────────────────────────────────────

struct QspiPsramConfig {
    // QMI chip select (must be CS1 for PSRAM — CS0 is flash)
    uint8_t  chipSelect     = 1;

    // QSPI clock speed
    uint32_t clockMHz       = 75;   // Safe default; some chips support 133 MHz

    // Memory capacity
    uint32_t capacityBytes  = 8 * 1024 * 1024;  // 8 MB default

    // XIP base address (fixed by RP2350 hardware)
    uintptr_t xipBase       = 0x11000000;

    // Timing parameters for QMI configuration
    uint8_t  readLatency    = 5;    // Dummy cycles for QPI read command
    uint8_t  pageSize       = 0;    // 0 = continuous read (no page boundary penalty)

    // PSRAM type
    bool     isDDR          = false; // True for DDR QSPI (doubles bandwidth)
};

// ─── QSPI PSRAM Driver ─────────────────────────────────────────────────────

class QspiPsramDriver {
public:
    QspiPsramDriver() = default;
    ~QspiPsramDriver();

    /// Initialize QMI CS1 for PSRAM access.
    /// Configures QMI timing, sends PSRAM reset/QPI-enable sequence,
    /// verifies PSRAM responds correctly.
    /// Returns true on success.
    bool Initialize(const QspiPsramConfig& config);

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

    /// Get a typed pointer to data at `offset` in QSPI PSRAM.
    template <typename T>
    const T* Map(uint32_t offset) const {
        return reinterpret_cast<const T*>(config_.xipBase + offset);
    }

    /// Writable map (write-through — goes to PSRAM immediately).
    template <typename T>
    T* MapWritable(uint32_t offset) {
        return reinterpret_cast<T*>(config_.xipBase + offset);
    }

    // ── Allocation ──────────────────────────────────────────────────────

    /// Simple bump allocator.  Returns byte offset from XIP base,
    /// or 0xFFFFFFFF on OOM.
    uint32_t Alloc(uint32_t size, uint32_t alignment = 4);

    /// Free all allocations (reset bump pointer).
    void FreeAll();

    /// Bytes remaining.
    uint32_t Available() const;

    // ── Bulk Write (for initial data upload) ────────────────────────────

    /// Write data to QSPI PSRAM via XIP write-through.
    /// This is a simple memcpy to the XIP-mapped address.
    void Write(uint32_t offset, const void* src, uint32_t length) {
        std::memcpy(reinterpret_cast<uint8_t*>(config_.xipBase) + offset,
                    src, length);
    }

    // ── Cache Management ────────────────────────────────────────────────

    /// Invalidate the QMI hardware cache.
    /// Call after bulk writes to ensure subsequent reads see updated data.
    void InvalidateCache();

    /// Flush the QMI write buffer (ensure all writes are committed to PSRAM).
    void FlushWrites();

    // ── Diagnostics ─────────────────────────────────────────────────────

    uint32_t GetCapacity() const { return config_.capacityBytes; }
    uintptr_t GetXipBaseAddr() const { return config_.xipBase; }

private:
    QspiPsramConfig config_{};
    bool            initialized_ = false;
    uint32_t        allocNext_   = 0;

    /// Configure QMI timing registers for the PSRAM chip.
    bool ConfigureQmiTiming();

    /// Send the PSRAM initialization sequence (reset + enable QPI mode).
    bool InitPsramChip();

    /// Read PSRAM ID register to verify the chip responds.
    bool VerifyPsramId();
};

