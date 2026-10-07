/**
 * @file custom_array.h
 * @brief Custom 8-bit parallel RGB888 array backend — portable API (P08).
 *
 * The portable section (namespace CustomArray) compiles on target and
 * natively (tests/display). The DisplayDriver implementation lives in
 * custom_array.cpp under PICO_ON_DEVICE; the factory entry point is
 * DisplayDriver& CustomArrayBackend() (declared in display_driver.h).
 *
 * Wire profile (see custom_array.pio):
 *   DATA0..7 = GPIO6..13, CLK = GPIO14 (PIO side-set), LATCH = GPIO15 and
 *   OE = GPIO16 (CPU SIO, OE active low). One byte per two SM cycles;
 *   byte clock = SM clock / 2 = 4 MHz at an 8 MHz SM clock. The complete
 *   stream carries exactly 3 bytes per pixel in R, G, B order; zero pad
 *   bytes lead the stream so the bytes held after the complete count are
 *   exactly the final pixel. LATCH pulses only after the final clock of
 *   the complete stream has actually finished; OE asserts only after that
 *   latch and stays blanked through every transfer and blocked wait.
 */

#pragma once

#include <cstddef>
#include <cstdint>

#include "../gpu_config.h"
#include "pixel_mapper.h"

namespace CustomArray {

// ─── Timing model (mirrors custom_array.pio) ────────────────────────────────

constexpr uint32_t kByteClockHz = GpuConfig::CUSTOM_PIXEL_CLOCK_HZ;  // 4 MHz
constexpr uint32_t kSmClockHz = 2 * kByteClockHz;                    // 8 MHz
constexpr uint32_t kBytesPerPixel = 3;                               // R, G, B
constexpr uint32_t kPioInstructions = 2;                             // custom_rgb888 program

// ─── Clock divider ──────────────────────────────────────────────────────────

struct ClockDivider {
    uint16_t integer;
    uint8_t  fraction;  // frac8 (1/256 units)
};

/// Exact clk_sys -> 8 MHz divider; false when not representable in int.frac8
/// (the 4 MHz byte clock would then drift, so the driver must refuse it).
inline bool ComputeClockDivider(uint32_t systemClockHz, ClockDivider& divider) {
    if (!systemClockHz) return false;
    const uint64_t d256 = uint64_t(systemClockHz) * 256u;
    if (d256 % kSmClockHz) return false;
    const uint64_t v = d256 / kSmClockHz;
    if (v < 256 || v > 0xFFFFFFull) return false;
    divider.integer  = static_cast<uint16_t>(v >> 8);
    divider.fraction = static_cast<uint8_t>(v & 0xFFu);
    return true;
}

// ─── Stream geometry ────────────────────────────────────────────────────────

constexpr uint32_t ByteCount(uint32_t pixels) { return pixels * kBytesPerPixel; }

/// Zero bytes inserted before pixel data so the total stream is a whole
/// number of 32-bit FIFO words and the trailing bytes are the final pixel.
constexpr uint32_t LeadingPadBytes(uint32_t pixels) {
    return (4u - (ByteCount(pixels) & 3u)) & 3u;
}

constexpr uint32_t WordCount(uint32_t pixels) {
    return (LeadingPadBytes(pixels) + ByteCount(pixels)) / 4u;
}

/// Largest pixel count whose padded stream fits the workspace. For
/// words = workspace/4, p = words*4/3 always satisfies
/// LeadingPadBytes(p) + 3p == words*4 exactly (pad == (words*4) mod 3).
constexpr uint32_t MaxPixels(size_t workspaceBytes) {
    const size_t words = workspaceBytes / 4u;
    return static_cast<uint32_t>((words * 4u) / 3u);
}

/// Pure validation before any pin is claimed.
inline PglRuntime::Result ValidateConfig(const DisplayConfig& config, size_t workspaceBytes,
                                         uint32_t& pixelCount) {
    using R = PglRuntime::Result;
    if (config.type != DisplayType::CustomRgb888) return R::InvalidValue;
    if (!config.width || !config.height) return R::InvalidValue;
    const uint64_t pixels = uint64_t(config.width) * config.height;
    if (!pixels || pixels > MaxPixels(workspaceBytes)) return R::Capacity;
    pixelCount = static_cast<uint32_t>(pixels);
    return R::Ok;
}

/// Bounded transfer-time budget: 0.25 us per byte at 4 MHz plus margin.
constexpr uint32_t StreamBudgetUs(uint32_t pixels) {
    return (LeadingPadBytes(pixels) + ByteCount(pixels)) / 4u + 500u;
}

/// Pack the byte at stream position `byteIndex` of a padded stream into the
/// little-endian DMA word layout (first wire byte = word LSB). Exposed for
/// deterministic encoder tests; the driver DMAs the byte buffer directly.
inline uint32_t PackStreamWord(const uint8_t* stream, uint32_t wordIndex) {
    const uint8_t* b = stream + size_t(wordIndex) * 4u;
    return uint32_t(b[0]) | (uint32_t(b[1]) << 8) | (uint32_t(b[2]) << 16) | (uint32_t(b[3]) << 24);
}

// ─── Encoding ───────────────────────────────────────────────────────────────

/// Expand RGB565 to 8-bit channels by top-bit replication.
constexpr void ExpandRgb888(uint16_t rgb565, uint8_t& r, uint8_t& g, uint8_t& b) {
    const uint32_t r5 = (rgb565 >> 11) & 31u;
    const uint32_t g6 = (rgb565 >> 5) & 63u;
    const uint32_t b5 = rgb565 & 31u;
    r = static_cast<uint8_t>((r5 << 3) | (r5 >> 2));
    g = static_cast<uint8_t>((g6 << 2) | (g6 >> 4));
    b = static_cast<uint8_t>((b5 << 3) | (b5 >> 2));
}

/// Encode pixels [firstPixel, firstPixel+count) through the cached sampler,
/// writing 3 bytes per pixel in R, G, B order to out[0..3*count).
/// Brightness endpoints exact: 0 -> all-zero bytes, 255 -> unmodified color.
/// Out-of-range samples encode as black via the sampler.
void EncodeRgb888Slice(const DisplayPixelSampler& sampler, uint8_t brightness,
                       uint32_t firstPixel, uint32_t count, uint8_t* out);

} // namespace CustomArray
