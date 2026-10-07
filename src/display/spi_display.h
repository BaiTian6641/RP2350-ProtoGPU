/**
 * @file spi_display.h
 * @brief PIO+DMA SSD1331 96x64 RGB565 OLED backend (P07).
 *
 * Real PIO serial transmitter (src/display/spi_display.pio, 2 words) paced
 * by one DREQ-driven DMA channel; CS/DC/RES are driver-owned SIO pins.
 * Protocol facts (Solomon Systech SSD1331 Rev 1.2):
 *   - command arguments are sent with DC LOW (unlike ST7789 helpers);
 *   - window: 15 00 5F (columns 0..95), 75 00 3F (rows 0..63);
 *   - A0 72: horizontal increment, reversed column map, normal RGB order,
 *     reversed COM scan, odd/even COM split, 65K/16-bit format;
 *   - pixel data is RGB565, HIGH BYTE FIRST, DC high, MSB-first;
 *   - vendor serial ceiling 6.67 MHz (Table 21); this backend runs <= 4 MHz
 *     with exact integer PIO dividers and >= 1 us CS/DC guards.
 *
 * The RGB565 source is borrowed via Present(), converted exactly once into
 * the caller-owned workspace (big-endian byte order) in bounded PollRefresh
 * slices, then DMAed.  Source release and the completion event happen only
 * after DMA depletion AND a rearmed-TXSTALL shifter-idle proof — DMA
 * completion alone is not serial completion.  The panel has no TE/FR line,
 * so completion is honestly Completion::Transferred (GDDRAM write finished),
 * never Displayed.
 */

#pragma once

#include <cstddef>
#include <cstdint>

#include "display_driver.h"
#include "pixel_mapper.h"
#include "../gpu_config.h"

namespace SpiDisplay {

constexpr uint16_t kWidth  = GpuConfig::SSD1331_WIDTH;   // 96
constexpr uint16_t kHeight = GpuConfig::SSD1331_HEIGHT;  // 64
constexpr uint32_t kFrameBytes = uint32_t(kWidth) * kHeight * 2;  // 12288
constexpr uint32_t kMaxSclkHz = GpuConfig::SSD1331_SPI_BAUD;      // 4 MHz

/// Exact integer PIO divider: f_SCLK = hz / (2 * div) <= kMaxSclkHz.
/// Divider is rounded up so the ceiling is never exceeded.
constexpr uint32_t ClockDivInt(uint32_t hz) {
    const uint32_t div = uint32_t((uint64_t(hz) + 2 * kMaxSclkHz - 1) /
                                  (2 * kMaxSclkHz));
    return div ? div : 1;
}
/// Resulting SCLK frequency for a system clock (integer division truncation
/// only lowers it further below the ceiling).
constexpr uint32_t ActualSclkHz(uint32_t hz) { return hz / (2 * ClockDivInt(hz)); }

/// Bounded, resumable pack of RGB565 logical pixels into the wire image:
/// for each packed pixel i, dest[2i] = high byte, dest[2i+1] = low byte.
/// The sampler was validated once; out-of-source samples pack as black.
/// Physical panel index is row-major (y * kWidth + x).
inline void PackRgb565BE(const DisplayPixelSampler& sampler, uint8_t* dest,
                         uint32_t firstPixel, uint32_t pixelCount) {
    for (uint32_t i = 0; i < pixelCount; ++i) {
        const uint16_t v = sampler.Get(firstPixel + i);
        dest[2 * i]     = uint8_t(v >> 8);
        dest[2 * i + 1] = uint8_t(v & 0xffu);
    }
}

} // namespace SpiDisplay

#if defined(PICO_ON_DEVICE)

class Ssd1331PioDriver final : public DisplayDriver {
public:
    Ssd1331PioDriver() = default;

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

    /// Pixels packed per PollRefresh() call (1024 px = 2 KiB wire bytes).
    static constexpr uint32_t kPackSlicePixels = 1024;
    /// Hard bound for a full-frame transfer + shifter drain at the slowest
    /// supported clock (75 MHz -> 3.75 MHz SCLK -> ~27 ms), plus margin.
    static constexpr uint64_t kTransferTimeoutUs = 100000;

private:
    enum class Phase : uint8_t { Idle, Packing, Transferring };

    void ReleaseAllClaims();
    void StopTransfer();
    void ConfigureStateMachine();
    PglRuntime::Result ApplyBrightness(uint8_t brightness, bool ensureOn);
    bool WriteBytes(const uint8_t* bytes, size_t count);   ///< DC as preset, CS framed
    bool WaitShifterIdle(uint32_t timeoutUs);              ///< rearm + await TXSTALL
    void StartDmaTransfer();
    void FinishTransfer();
    void PushEvent(PglRuntime::Completion completion, PglRuntime::Result result,
                   uint64_t timestampUs, bool sourceReleased);

    HardwareResources* resources_ = nullptr;
    DisplayConfig config_ = {};
    uint8_t* frameBuffer_ = nullptr;   ///< workspace: packed big-endian image
    uint32_t clockHz_ = 0;
    uint8_t brightness_ = 255;
    bool pendingBrightness_ = false;

    HardwareResources::PioLease lease_;
    int dma_ = -1;
    uint8_t pioBlock_ = 1;

    bool inited_ = false, quiesced_ = false, faulted_ = false;
    Phase phase_ = Phase::Idle;

    DisplaySurface surface_ = {};
    DisplayMapping mapping_ = {};
    alignas(DisplayPixelSampler) uint8_t samplerStorage_[sizeof(DisplayPixelSampler)] = {};
    DisplayPixelSampler* sampler_ = nullptr;
    uint32_t packCursor_ = 0;
    uint32_t pendingFrame_ = 0;
    uint64_t transferStartUs_ = 0;

    DisplayEvent events_[4] = {};
    volatile uint8_t eventHead_ = 0, eventTail_ = 0;
};

#endif // PICO_ON_DEVICE
