/**
 * @file hub75_driver.h
 * @brief HUB75 backend — autonomous whole-scan PIO/DMA engine (P07).
 *
 * Conventional 128x64 or 64x64, 1:32-scan HUB75 panel: 32 row-pairs
 * (r, r+32), six RGB outputs (R1G1B1 upper, R2G2B2 lower), CLK, LAT, OE,
 * five address lines.  Eight BCM planes per row-pair => 256 records/scan.
 *
 * The RGB565 source is borrowed via Present() and converted exactly once
 * into an immutable encoded scan bank inside the caller-owned display
 * workspace (GpuConfig::DISPLAY_WORKSPACE_BYTES, shared with the other
 * backends — no per-driver static scan buffers):
 *
 *   data bank 0/1 : ScanDataBytes(width) each   (byte-packed planes)
 *   ctrl bank 0/1 : kControlBankBytes each      (256 x uint32 records)
 *
 * Data bytes: bits [5:0] = R1 G1 B1 R2 G2 B2, bits [7:6] = 0 pad.  The two
 * PIO programs (src/display/hub75.pio, 22 words total) shift, latch and
 * dwell; two DMA channels (byte data + control words, DREQ-paced) feed
 * them. The CPU only acts at the whole-scan boundary IRQ: it re-arms both
 * channels and atomically switches armed banks. Displayed is reported at the
 * end of the newly activated bank's complete scan, not at its first row.
 * The data FIFO has consumed the retiring bank at each gate, so the next
 * frame cannot mix rows/planes with the previous bank. With no new frame
 * armed, the latest complete scan repeats with zero per-record CPU work.
 *
 * The pure encoding/accounting functions below are SDK-free and exercised by
 * tests/display/test_output_encoders.cpp on the native host.
 */

#pragma once

#include <cstddef>
#include <cstdint>

#include "display_driver.h"
#include "pixel_mapper.h"
#include "../gpu_config.h"

namespace Hub75 {

// ─── Fixed panel/schedule geometry ─────────────────────────────────────────
constexpr uint16_t kPanelHeight = 64;          ///< fixed for the 1:32 reference
constexpr uint8_t  kScanRows    = 32;          ///< 1:32 scan -> 32 row-pairs
constexpr uint8_t  kPlanes      = 8;           ///< BCM planes (COLOR_DEPTH)
constexpr uint16_t kRecordCount = kScanRows * kPlanes;  ///< 256 records/scan
constexpr uint8_t  kCyclesPerPixel = 3;        ///< hub75_data: out/nop/jmp
constexpr uint32_t kPixelClockHz = GpuConfig::HUB75_PIXEL_CLOCK_HZ;
constexpr uint32_t kBaseDwellNs  = GpuConfig::HUB75_BASE_DWELL_NS;

// ─── Sizing / workspace accounting ─────────────────────────────────────────
constexpr bool SupportedWidth(uint16_t width) { return width == 128 || width == 64; }
constexpr uint32_t ScanDataBytes(uint16_t width) { return uint32_t(kRecordCount) * width; }
constexpr uint32_t ControlBankBytes() { return uint32_t(kRecordCount) * sizeof(uint32_t); }
constexpr uint32_t RequiredWorkspaceBytes(uint16_t width) {
    return 2 * ScanDataBytes(width) + 2 * ControlBankBytes();
}
static_assert(RequiredWorkspaceBytes(128) <= GpuConfig::DISPLAY_WORKSPACE_BYTES,
              "two encoded 128x64 scans + control records must fit the shared workspace");

// ─── Clock-derived quantities (recomputed on Resume) ───────────────────────
/// PIO clkdiv (8.8 fixed point, rounded up) so the pixel clock never exceeds
/// kPixelClockHz: f_pixel = hz / (3 * div).
constexpr uint32_t PixelClockDivFrac8(uint32_t hz) {
    const uint64_t num = uint64_t(hz) * 256;
    const uint64_t den = uint64_t(kCyclesPerPixel) * kPixelClockHz;
    return uint32_t((num + den - 1) / den);
}
/// SM cycles (row SM runs at clk_sys) that cover kBaseDwellNs, rounded up.
constexpr uint32_t BaseDwellCycles(uint32_t hz) {
    return uint32_t((uint64_t(hz) * kBaseDwellNs + 999999999ull) / 1000000000ull);
}
/// Lit SM cycles for a BCM plane at a brightness (row SM clkdiv = 1).
constexpr uint32_t DwellCycles(uint8_t plane, uint8_t brightness, uint32_t hz) {
    const uint64_t base = uint64_t(BaseDwellCycles(hz)) << plane;
    return uint32_t((base * brightness + 127u) / 255u);
}
/// Row-SM encoding: stored 0 => OE never asserted (brightness 0 never lights
/// a row); stored s > 0 => exactly s + 1 lit cycles (JMP X-- loop).  A
/// computed dwell of 1 cycle is rounded down to "dark" — monotonic, and far
/// below the 160 ns base dwell.
constexpr uint32_t EncodeDwell(uint32_t cycles) { return cycles >= 2 ? cycles - 1 : 0; }

/// Control record: bits [4:0] = row-pair address, bits [31:5] = encoded dwell.
/// Record order is row-major: record = row * kPlanes + plane.
constexpr uint32_t EncodeControlRecord(uint16_t recordIndex, uint8_t brightness, uint32_t hz) {
    const uint32_t row = recordIndex / kPlanes, plane = recordIndex % kPlanes;
    return row | (EncodeDwell(DwellCycles(uint8_t(plane), brightness, hz)) << 5);
}
/// Fill one immutable control bank (256 words).  Pure function of plane
/// weights — it carries no frame content, so Resume()/SetBrightness() can
/// rebuild it without the released RGB source.
inline void EncodeControlBank(uint32_t* bank, uint8_t brightness, uint32_t hz) {
    for (uint16_t i = 0; i < kRecordCount; ++i) bank[i] = EncodeControlRecord(i, brightness, hz);
}

// ─── Pixel encoding ────────────────────────────────────────────────────────
/// 5/6-bit channel -> 8-bit BCM intensity (bit-replication expand).
constexpr uint8_t Expand5(uint16_t v5) { return uint8_t((v5 << 3) | (v5 >> 2)); }
constexpr uint8_t Expand6(uint16_t v6) { return uint8_t((v6 << 2) | (v6 >> 4)); }

/// One byte-packed output column: bit0 R1, bit1 G1, bit2 B1 (upper row),
/// bit3 R2, bit4 G2, bit5 B2 (lower row), bits [7:6] = 0 pad.  OUT shifts
/// right, so OSR bit 0 drives the R1 pin first.
inline uint8_t EncodeByte(uint16_t upper565, uint16_t lower565, uint8_t plane) {
    const uint8_t r1 = uint8_t((Expand5(uint8_t(upper565 >> 11)) >> plane) & 1u);
    const uint8_t g1 = uint8_t((Expand6(uint8_t((upper565 >> 5) & 63u)) >> plane) & 1u);
    const uint8_t b1 = uint8_t((Expand5(uint8_t(upper565 & 31u)) >> plane) & 1u);
    const uint8_t r2 = uint8_t((Expand5(uint8_t(lower565 >> 11)) >> plane) & 1u);
    const uint8_t g2 = uint8_t((Expand6(uint8_t((lower565 >> 5) & 63u)) >> plane) & 1u);
    const uint8_t b2 = uint8_t((Expand5(uint8_t(lower565 & 31u)) >> plane) & 1u);
    return uint8_t(r1 | (g1 << 1) | (b1 << 2) | (r2 << 3) | (g2 << 4) | (b2 << 5));
}

/// Encode one record (one BCM plane of one row-pair) = panelWidth bytes.
/// The sampler has already been validated once; physical panel index is
/// row-major, so row-pair r maps rows r (upper) and r + kScanRows (lower).
inline void EncodeRecord(const DisplayPixelSampler& sampler, uint16_t panelWidth,
                         uint8_t row, uint8_t plane, uint8_t* out) {
    const uint32_t upperBase = uint32_t(row) * panelWidth;
    const uint32_t lowerBase = uint32_t(row + kScanRows) * panelWidth;
    for (uint16_t c = 0; c < panelWidth; ++c)
        out[c] = EncodeByte(sampler.Get(upperBase + c), sampler.Get(lowerBase + c), plane);
}

// ─── Bounded, resumable whole-bank conversion ──────────────────────────────
struct EncodeCursor {
    uint16_t record = 0;   ///< next record to encode; kRecordCount when done
    bool Done() const { return record >= kRecordCount; }
};

/// Encode at most maxRecords records into bank (immutable once armed).
/// Returns the number encoded; 0 when the cursor was already done, in which
/// case the bank and the source are not touched again (bounded source hold).
inline uint16_t EncodeSlice(const DisplayPixelSampler& sampler, uint16_t panelWidth,
                            uint8_t* bank, EncodeCursor& cursor, uint16_t maxRecords) {
    uint16_t done = 0;
    while (done < maxRecords && cursor.record < kRecordCount) {
        const uint16_t rec = cursor.record;
        EncodeRecord(sampler, panelWidth, uint8_t(rec / kPlanes), uint8_t(rec % kPlanes),
                     bank + uint32_t(rec) * panelWidth);
        ++cursor.record;
        ++done;
    }
    return done;
}

} // namespace Hub75

#if defined(PICO_ON_DEVICE)

class Hub75PanelDriver final : public DisplayDriver {
public:
    Hub75PanelDriver() = default;

    PglRuntime::Result Init(HardwareResources& resources, const DisplayConfig& config,
                            void* workspace, size_t bytes) override;
    PglRuntime::Result Present(const DisplaySurface& surface, const DisplayMapping& mapping) override;
    void PollRefresh() override;
    bool PopEvent(DisplayEvent& event) override;
    PglRuntime::Result Quiesce() override;
    PglRuntime::Result Resume(uint32_t systemClockHz) override;
    PglRuntime::Result SetBrightness(uint8_t brightness) override;
    DisplayCapabilities GetCaps() const override;
    void Shutdown() override;

    /// Bounded slice encoded per PollRefresh() call (8 records <= 1 KiB).
    static constexpr uint16_t kEncodeSliceRecords = 8;
    /// Scan-boundary watchdog: no scan-complete for this long => fault-blank.
    static constexpr uint64_t kScanWatchdogUs = 100000;

private:
    enum class Phase : uint8_t { Idle, Encoding, Armed, Scanning };

    void ReleaseAllClaims();
    void ForceBlank();          ///< abort DMA/SMs and drive OE high via SIO
    void PushEvent(PglRuntime::Completion completion, PglRuntime::Result result,
                   uint64_t timestampUs, bool sourceReleased);
    void OnScanComplete();      ///< ISR context, whole-scan gate
    static void ScanIrqHandler();

    uint8_t* DataBank(uint8_t index) const { return dataBanks_ + uint32_t(index) * scanBytes_; }
    uint32_t* CtrlBank(uint8_t index) const {
        return reinterpret_cast<uint32_t*>(ctrlBanks_ + uint32_t(index) * Hub75::ControlBankBytes());
    }

    HardwareResources* resources_ = nullptr;
    DisplayConfig config_ = {};
    uint8_t* dataBanks_ = nullptr;   ///< workspace: two byte-packed scan banks
    uint8_t* ctrlBanks_ = nullptr;   ///< workspace: two 1 KiB control banks
    uint32_t scanBytes_ = 0;
    uint32_t clockHz_ = 0;
    uint8_t brightness_ = 255;

    HardwareResources::PioLease dataLease_, rowLease_;
    int dmaData_ = -1, dmaCtrl_ = -1;
    uint8_t pioBlock_ = 1;           ///< coordinated two-SM claim, one block
    static constexpr uint8_t kIrqFlagMask = 0x0f;  ///< flags 0..3 (see hub75.pio)

    bool inited_ = false, quiesced_ = false, faulted_ = false;

    // Bank ownership: ACTIVE bank is scanned, the other is FREE for encoding.
    volatile uint8_t activeDataBank_ = 0, activeCtrlBank_ = 0;
    volatile uint8_t armedDataBank_ = 0xff, armedCtrlBank_ = 0xff;  // 0xff = none
    volatile Phase phase_ = Phase::Idle;   // main <-> scan ISR handoff

    // Borrowed source + one validated sampler for the whole conversion.
    DisplaySurface surface_ = {};
    DisplayMapping mapping_ = {};
    alignas(DisplayPixelSampler) uint8_t samplerStorage_[sizeof(DisplayPixelSampler)] = {};
    DisplayPixelSampler* sampler_ = nullptr;
    Hub75::EncodeCursor cursor_ = {};
    uint32_t pendingFrame_ = 0;

    // Whole-scan ISR state.
    volatile uint32_t scanCount_ = 0;
    volatile uint64_t lastScanUs_ = 0;
    uint32_t totalScans_ = 0, forcedBlanks_ = 0;

    DisplayEvent events_[4] = {};
    volatile uint8_t eventHead_ = 0, eventTail_ = 0;
};

#endif // PICO_ON_DEVICE
