/**
 * @file led_array.h
 * @brief WS2812B-V5/W LED array backend — portable timing/encoding API (P08).
 *
 * The portable section of this header (namespace LedArray) is compiled both
 * on target and natively (tests/display). The DisplayDriver implementation
 * lives in led_array.cpp and exists only under PICO_ON_DEVICE; the factory
 * entry point is DisplayDriver& LedArrayBackend() (declared in
 * display_driver.h).
 *
 * Timing reference: Worldsemi WS2812B-V5/W datasheet V6.1 (2022-09-08).
 *   T0H 220..380 ns, T1H 580..1000 ns, T0L/T1L 580..1000 ns, reset > 280 us.
 * Explicit 2/2/4 profile at a 6.4 MHz SM clock (NOT the SDK stock 3/3/4,
 * whose 500 ns T1L violates the V5/W minimum):
 *   T0H 312.5 ns, T1H 625 ns, T0L 937.5 ns, T1L 625 ns, bit 1.25 us.
 * The average divider is exact (int.frac8) at clk_sys
 * 75/100/125/150/240/250/288/300/336 MHz; native tests bound pulse-width
 * jitter at those profiles. Calculated timing is not physical qualification.
 */

#pragma once

#include <cstddef>
#include <cstdint>

#include "../gpu_config.h"
#include "pixel_mapper.h"

namespace LedArray {

// ─── Timing model (mirrors led_array.pio) ───────────────────────────────────

constexpr uint32_t kSmClockHz = GpuConfig::LED_PIO_CLOCK_HZ;  // 6 400 000
constexpr uint32_t kT1Cycles = 2, kT2Cycles = 2, kT3Cycles = 4;
constexpr uint32_t kCyclesPerBit = kT1Cycles + kT2Cycles + kT3Cycles;  // 8
constexpr uint32_t kBitsPerPixel = 24;                         // GRB24
constexpr uint32_t kResetLowUs = GpuConfig::LED_RESET_US;      // 300 (> 280)
constexpr uint32_t kBytesPerPixel = 4;                         // one 32-bit FIFO word
constexpr uint32_t kPioInstructions = 4;                       // ws2812b_v5 program

/// Bit timing derived from the cycle profile, in picoseconds.
struct BitTimingPs {
    uint32_t t0h, t1h, t0l, t1l, bit;
};

constexpr BitTimingPs GetBitTimingPs() {
    constexpr uint64_t kPsPerCycle = 1000000000000ull / kSmClockHz;  // 156 250
    return BitTimingPs{
        uint32_t(kT1Cycles * kPsPerCycle),                 // T0H = 312 500
        uint32_t((kT1Cycles + kT2Cycles) * kPsPerCycle),   // T1H = 625 000
        uint32_t((kT3Cycles + kT2Cycles) * kPsPerCycle),   // T0L = 937 500
        uint32_t(kT3Cycles * kPsPerCycle),                 // T1L = 625 000
        uint32_t(kCyclesPerBit * kPsPerCycle),             // 1 250 000
    };
}

// Vendor V5/W windows, checked at compile time so a profile edit cannot
// silently drift out of the datasheet range.
static_assert(GetBitTimingPs().t0h >= 220000 && GetBitTimingPs().t0h <= 380000, "T0H window");
static_assert(GetBitTimingPs().t1h >= 580000 && GetBitTimingPs().t1h <= 1000000, "T1H window");
static_assert(GetBitTimingPs().t0l >= 580000 && GetBitTimingPs().t0l <= 1000000, "T0L window");
static_assert(GetBitTimingPs().t1l >= 580000 && GetBitTimingPs().t1l <= 1000000, "T1L window");
static_assert(kResetLowUs > 280, "reset low must exceed the 280 us V5/W minimum");

// ─── Clock divider ──────────────────────────────────────────────────────────

struct ClockDivider {
    uint16_t integer;
    uint8_t  fraction;  // frac8 (1/256 units)
};

/// Compute the exact clk_sys -> 6.4 MHz divider. Returns false when the
/// system clock cannot be represented exactly in int.frac8 (the waveform
/// would then leave the computed V5/W windows, so the driver must refuse it).
inline bool ComputeClockDivider(uint32_t systemClockHz, ClockDivider& divider) {
    if (!systemClockHz) return false;
    const uint64_t d256 = uint64_t(systemClockHz) * 256u;
    if (d256 % kSmClockHz) return false;
    const uint64_t v = d256 / kSmClockHz;
    if (v < 256 || v > 0xFFFFFFull) return false;  // SDK divider range 1..65535.996
    divider.integer  = static_cast<uint16_t>(v >> 8);
    divider.fraction = static_cast<uint8_t>(v & 0xFFu);
    return true;
}

// ─── Capacity / configuration (validated before any pin is claimed) ─────────

constexpr uint32_t MaxPixels(size_t workspaceBytes) {
    return static_cast<uint32_t>(workspaceBytes / kBytesPerPixel);
}

/// Pure validation: device count/width/height/workspace bounds. Runs before
/// any GPIO/PIO/DMA claim so over-capacity or bad config fails with pins
/// untouched.
inline PglRuntime::Result ValidateConfig(const DisplayConfig& config, size_t workspaceBytes,
                                         uint32_t& pixelCount) {
    using R = PglRuntime::Result;
    if (config.type != DisplayType::LedV5) return R::InvalidValue;
    if (!config.width || !config.height) return R::InvalidValue;
    const uint64_t pixels = uint64_t(config.width) * config.height;
    if (!pixels || pixels > MaxPixels(workspaceBytes)) return R::Capacity;
    pixelCount = static_cast<uint32_t>(pixels);
    return R::Ok;
}

/// Bounded stream-time budget: 24 bits * 1.25 us per pixel plus margin.
constexpr uint32_t StreamBudgetUs(uint32_t pixels) {
    return pixels * (kBitsPerPixel * kCyclesPerBit * 1000000u / kSmClockHz) + 500u;
}

// ─── Encoding (RGB565 -> GRB24 word, MSB first in bits [31:8]) ─────────────

/// Expand one RGB565 pixel to the 24-bit GRB stream word. 5/6-bit channels
/// are expanded to 8 bits by top-bit replication (exact, order-preserving).
/// Word layout: [31:24]=G [23:16]=R [15:8]=B [7:0]=0 — MSB of G is the first
/// bit on the wire, matching the PIO shift-left/autopull-24 configuration.
constexpr uint32_t EncodeGrbWord(uint16_t rgb565) {
    const uint32_t r5 = (rgb565 >> 11) & 31u;
    const uint32_t g6 = (rgb565 >> 5) & 63u;
    const uint32_t b5 = rgb565 & 31u;
    const uint32_t r8 = (r5 << 3) | (r5 >> 2);
    const uint32_t g8 = (g6 << 2) | (g6 >> 4);
    const uint32_t b8 = (b5 << 3) | (b5 >> 2);
    return (g8 << 24) | (r8 << 16) | (b8 << 8);
}

/// Encode pixels [firstPixel, firstPixel+count) through the cached sampler
/// into outWords[0..count). Brightness endpoints are exact: 0 -> all-zero
/// stream, 255 -> unmodified color. Samples outside the surface (scatter
/// holes, counts beyond the mapping) encode as black via the sampler.
/// Returns the number of words written (== count).
uint32_t EncodeGrbSlice(const DisplayPixelSampler& sampler, uint8_t brightness,
                        uint32_t firstPixel, uint32_t count, uint32_t* outWords);

} // namespace LedArray
