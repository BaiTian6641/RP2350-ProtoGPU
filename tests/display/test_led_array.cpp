// LED array (WS2812B-V5/W) native encoder/timing tests (P08).
//
// Exercises the REAL portable encoding, timing and validation code from
// src/display/led_array.{h,cpp} (same translation units the target driver
// uses) plus the parent-owned DisplayPixelSampler mapping semantics the
// driver caches per borrowed surface. No PIO/DMA is faked; hardware
// qualification remains an open gate.
//
// Build and run via tests/display/run_tests.sh (native g++, no Pico SDK).

#include "display/led_array.h"

#include <cstdint>
#include <cstdio>
#include <cstring>
#include <initializer_list>

namespace {

int gChecks = 0;
int gFailures = 0;

#define CHECK(cond)                                                            \
    do {                                                                       \
        ++gChecks;                                                             \
        if (!(cond)) {                                                         \
            ++gFailures;                                                       \
            std::printf("FAIL %s:%d: CHECK(%s)\n", __FILE__, __LINE__, #cond); \
        }                                                                      \
    } while (0)

// Deterministic non-repeating RGB565 pattern (every value distinct).
uint16_t PatternPixel(uint32_t i) {
    return static_cast<uint16_t>(0x1234u + i * 0x0401u);
}

void FillPattern(uint16_t* pixels, uint32_t count) {
    for (uint32_t i = 0; i < count; ++i) pixels[i] = PatternPixel(i);
}

DisplaySurface MakeSurface(const uint16_t* pixels, uint16_t w, uint16_t h) {
    DisplaySurface s{};
    s.pixels = pixels;
    s.pixelCapacity = size_t(w) * h;
    s.width = w;
    s.height = h;
    s.stride = w;
    s.frame = 7;
    return s;
}

DisplayConfig MakeConfig(uint16_t w, uint16_t h, uint8_t brightness = 255) {
    DisplayConfig c{};
    c.type = DisplayType::LedV5;
    c.width = w;
    c.height = h;
    c.brightness = brightness;
    return c;
}

// ─── Timing arithmetic (vendor V5/W windows) ────────────────────────────────

void TestTimingModel() {
    const LedArray::BitTimingPs t = LedArray::GetBitTimingPs();
    CHECK(t.t0h == 312500);   // 2 cycles at 6.4 MHz
    CHECK(t.t1h == 625000);   // 4 cycles
    CHECK(t.t0l == 937500);   // 6 cycles
    CHECK(t.t1l == 625000);   // 4 cycles
    CHECK(t.bit == 1250000);  // 800 kbit/s
    // Vendor WS2812B-V5/W windows (datasheet V6.1).
    CHECK(t.t0h >= 220000 && t.t0h <= 380000);
    CHECK(t.t1h >= 580000 && t.t1h <= 1000000);
    CHECK(t.t0l >= 580000 && t.t0l <= 1000000);
    CHECK(t.t1l >= 580000 && t.t1l <= 1000000);
    CHECK(LedArray::kResetLowUs == 300);   // > 280 us minimum
    CHECK(LedArray::kResetLowUs > 280);
    CHECK(LedArray::kCyclesPerBit == 8);
    CHECK(LedArray::StreamBudgetUs(256) == 256 * 30 + 500);
}

void TestClockDividerQuantization() {
    LedArray::ClockDivider d{};
    // All nine admitted clock profiles quantize exactly in int.frac8.
    CHECK(LedArray::ComputeClockDivider(150000000, d));
    CHECK(d.integer == 23 && d.fraction == 112);   // 23.4375
    CHECK(LedArray::ComputeClockDivider(100000000, d));
    CHECK(d.integer == 15 && d.fraction == 160);   // 15.625
    CHECK(LedArray::ComputeClockDivider(75000000, d));
    CHECK(d.integer == 11 && d.fraction == 184);   // 11.71875
    // The represented divider really reproduces the system clock.
    for (uint32_t sys : {75000000u, 100000000u, 125000000u, 150000000u,
                         240000000u, 250000000u, 288000000u, 300000000u,
                         336000000u}) {
        CHECK(LedArray::ComputeClockDivider(sys, d));
        const uint64_t represented = (uint64_t(d.integer) * 256 + d.fraction);
        CHECK(represented * LedArray::kSmClockHz == uint64_t(sys) * 256u);
        // Fractional-divider jitter: every pulse's extreme sys-cycle count
        // must remain inside vendor windows, not merely its average period.
        const uint32_t cycles[4] = {2, 4, 6, 4};
        const uint64_t minimumPs[4] = {220000, 580000, 580000, 580000};
        const uint64_t maximumPs[4] = {380000, 1000000, 1000000, 1000000};
        for (uint32_t pulse = 0; pulse < 4; ++pulse) {
            const uint64_t tick256 = represented * cycles[pulse];
            const uint64_t shortest = (tick256 / 256u) * 1000000000000ull / sys;
            const uint64_t longest = ((tick256 + 255u) / 256u) * 1000000000000ull / sys;
            CHECK(shortest >= minimumPs[pulse]);
            CHECK(longest <= maximumPs[pulse]);
        }
    }
    CHECK(LedArray::ComputeClockDivider(240000000, d));
    CHECK(d.integer == 37 && d.fraction == 128);
    CHECK(LedArray::ComputeClockDivider(288000000, d));
    CHECK(d.integer == 45 && d.fraction == 0);
    CHECK(LedArray::ComputeClockDivider(336000000, d));
    CHECK(d.integer == 52 && d.fraction == 128);
    // Inexact or out-of-range clocks must be refused, not approximated.
    CHECK(!LedArray::ComputeClockDivider(0, d));
    CHECK(!LedArray::ComputeClockDivider(100000001, d));  // not divisible
    CHECK(!LedArray::ComputeClockDivider(3000000, d));    // divider < 1
    CHECK(!LedArray::ComputeClockDivider(123456789, d));
}

// ─── Configuration validation (before any pin claim) ────────────────────────

void TestValidateConfig() {
    uint32_t pixels = 0;
    DisplayConfig c = MakeConfig(16, 16);
    CHECK(LedArray::ValidateConfig(c, 1024, pixels) == PglRuntime::Result::Ok);
    CHECK(pixels == 256);

    c.type = DisplayType::Hub75;  // wrong backend
    CHECK(LedArray::ValidateConfig(c, 1024, pixels) == PglRuntime::Result::InvalidValue);
    c = MakeConfig(0, 16);
    CHECK(LedArray::ValidateConfig(c, 1024, pixels) == PglRuntime::Result::InvalidValue);
    c = MakeConfig(16, 0);
    CHECK(LedArray::ValidateConfig(c, 1024, pixels) == PglRuntime::Result::InvalidValue);

    // Over capacity must fail: 300x300 = 90000 > 68 KiB / 4 = 17408.
    c = MakeConfig(300, 300);
    CHECK(LedArray::ValidateConfig(c, 68 * 1024, pixels) == PglRuntime::Result::Capacity);
    // Exact capacity boundary is accepted.
    CHECK(LedArray::MaxPixels(68 * 1024) == 17408);
    c = MakeConfig(128, 136);  // 17408 pixels exactly
    CHECK(LedArray::ValidateConfig(c, 68 * 1024, pixels) == PglRuntime::Result::Ok);
    CHECK(pixels == 17408);
    // ... but a 1023-byte workspace cannot hold 256 pixels.
    c = MakeConfig(16, 16);
    CHECK(LedArray::ValidateConfig(c, 1023, pixels) == PglRuntime::Result::Capacity);
}

// ─── GRB word encoding ──────────────────────────────────────────────────────

void TestGrbChannelOrder() {
    // Word layout [31:24]=G [23:16]=R [15:8]=B [7:0]=0, G MSB first on wire.
    CHECK(LedArray::EncodeGrbWord(0xFFFF) == 0xFFFFFF00u);
    CHECK(LedArray::EncodeGrbWord(0xF800) == 0x00FF0000u);  // red only
    CHECK(LedArray::EncodeGrbWord(0x07E0) == 0xFF000000u);  // green only
    CHECK(LedArray::EncodeGrbWord(0x001F) == 0x0000FF00u);  // blue only
    CHECK(LedArray::EncodeGrbWord(0x0000) == 0x00000000u);
    // Mixed channels with non-repeating nibble pattern.
    const uint16_t mixed = static_cast<uint16_t>((22u << 11) | (44u << 5) | 9u);
    const uint32_t r8 = (22u << 3) | (22u >> 2);  // 0xB5
    const uint32_t g8 = (44u << 2) | (44u >> 4);  // 0xB2
    const uint32_t b8 = (9u << 3) | (9u >> 2);    // 0x4A
    CHECK(LedArray::EncodeGrbWord(mixed) == ((g8 << 24) | (r8 << 16) | (b8 << 8)));
    // First bit on the wire is the green channel MSB.
    CHECK(((LedArray::EncodeGrbWord(mixed) >> 24) & 0xFFu) == g8);
}

void TestRowMajorNonRepeating() {
    uint16_t fb[32];
    FillPattern(fb, 32);
    DisplaySurface surf = MakeSurface(fb, 8, 4);
    DisplayConfig cfg = MakeConfig(8, 4);
    DisplayMapping map{};
    DisplayPixelSampler sampler(surf, map, cfg);
    CHECK(sampler.Valid());

    uint32_t words[32] = {};
    CHECK(LedArray::EncodeGrbSlice(sampler, 255, 0, 32, words) == 32);
    uint32_t distinct = 0;
    for (uint32_t i = 0; i < 32; ++i) {
        CHECK(words[i] == LedArray::EncodeGrbWord(fb[i]));  // row-major order
        bool seen = false;
        for (uint32_t j = 0; j < i; ++j) seen = seen || words[j] == words[i];
        if (!seen) ++distinct;
    }
    CHECK(distinct == 32);  // non-repeating pattern stays non-repeating
}

void TestRectangularMapping() {
    uint16_t fb[32];
    FillPattern(fb, 32);
    DisplaySurface surf = MakeSurface(fb, 8, 4);
    DisplayConfig cfg = MakeConfig(4, 2);  // 8 LEDs over the 8x4 surface
    DisplayMapping map{};
    map.rectangular = true;
    map.rectangle.size = {8.0f, 4.0f};
    map.rectangle.position = {0.0f, 0.0f};
    map.rectangle.rowCount = 2;
    map.rectangle.colCount = 4;
    DisplayPixelSampler sampler(surf, map, cfg);
    CHECK(sampler.Valid());

    uint32_t words[8] = {};
    LedArray::EncodeGrbSlice(sampler, 255, 0, 8, words);
    for (uint32_t i = 0; i < 8; ++i) {
        const uint16_t sx = static_cast<uint16_t>(1 + 2 * (i % 4));  // cell centers
        const uint16_t sy = static_cast<uint16_t>(1 + 2 * (i / 4));
        CHECK(words[i] == LedArray::EncodeGrbWord(fb[sy * 8 + sx]));
    }
}

void TestScatterHoles() {
    uint16_t fb[16];
    FillPattern(fb, 16);
    DisplaySurface surf = MakeSurface(fb, 4, 4);
    DisplayConfig cfg = MakeConfig(3, 2);  // 6 LEDs, only 3 mapped
    const PglVec2 coords[3] = {{0.5f, 0.5f}, {99.0f, 1.0f}, {-1.0f, 2.0f}};
    DisplayMapping map{};
    map.coordinates = coords;
    map.count = 3;
    DisplayPixelSampler sampler(surf, map, cfg);
    CHECK(sampler.Valid());

    uint32_t words[6] = {};
    LedArray::EncodeGrbSlice(sampler, 255, 0, 6, words);
    CHECK(words[0] == LedArray::EncodeGrbWord(fb[0]));  // (0,0)
    CHECK(words[1] == 0);  // out-of-surface x -> black hole
    CHECK(words[2] == 0);  // negative x -> black hole
    CHECK(words[3] == 0);  // beyond mapping count -> black
    CHECK(words[4] == 0);
    CHECK(words[5] == 0);
}

void TestReversedMapping() {
    uint16_t fb[4];
    FillPattern(fb, 4);
    DisplaySurface surf = MakeSurface(fb, 4, 1);
    DisplayConfig cfg = MakeConfig(4, 1);
    DisplayMapping map{};
    map.reversed = true;
    DisplayPixelSampler sampler(surf, map, cfg);
    CHECK(sampler.Valid());

    uint32_t words[4] = {};
    LedArray::EncodeGrbSlice(sampler, 255, 0, 4, words);
    for (uint32_t i = 0; i < 4; ++i) {
        CHECK(words[i] == LedArray::EncodeGrbWord(fb[3 - i]));
    }
}

void TestFlipMapping() {
    uint16_t fb[4];
    FillPattern(fb, 4);
    DisplaySurface surf = MakeSurface(fb, 2, 2);
    DisplayMapping map{};

    DisplayConfig cfgH = MakeConfig(2, 2);
    cfgH.flipH = true;
    DisplayPixelSampler sampH(surf, map, cfgH);
    uint32_t words[4] = {};
    LedArray::EncodeGrbSlice(sampH, 255, 0, 4, words);
    // Row-major samples with horizontal mirror: (0,0)->(1,0), (1,0)->(0,0), ...
    CHECK(words[0] == LedArray::EncodeGrbWord(fb[1]));
    CHECK(words[1] == LedArray::EncodeGrbWord(fb[0]));
    CHECK(words[2] == LedArray::EncodeGrbWord(fb[3]));
    CHECK(words[3] == LedArray::EncodeGrbWord(fb[2]));

    DisplayConfig cfgV = MakeConfig(2, 2);
    cfgV.flipV = true;
    DisplayPixelSampler sampV(surf, map, cfgV);
    LedArray::EncodeGrbSlice(sampV, 255, 0, 4, words);
    CHECK(words[0] == LedArray::EncodeGrbWord(fb[2]));
    CHECK(words[1] == LedArray::EncodeGrbWord(fb[3]));
    CHECK(words[2] == LedArray::EncodeGrbWord(fb[0]));
    CHECK(words[3] == LedArray::EncodeGrbWord(fb[1]));
}

// ─── Brightness endpoints ───────────────────────────────────────────────────

void TestBrightnessEndpoints() {
    uint16_t fb[8];
    FillPattern(fb, 8);
    DisplaySurface surf = MakeSurface(fb, 8, 1);
    DisplayConfig cfg = MakeConfig(8, 1);
    DisplayMapping map{};
    DisplayPixelSampler sampler(surf, map, cfg);

    uint32_t words[8] = {};
    // 0 must be exactly black, not a residual level.
    LedArray::EncodeGrbSlice(sampler, 0, 0, 8, words);
    for (uint32_t i = 0; i < 8; ++i) CHECK(words[i] == 0);
    // 255 must be the exact unscaled expansion.
    LedArray::EncodeGrbSlice(sampler, 255, 0, 8, words);
    for (uint32_t i = 0; i < 8; ++i) CHECK(words[i] == LedArray::EncodeGrbWord(fb[i]));
    // Mid-scale independently recomputed with the documented rounding.
    LedArray::EncodeGrbSlice(sampler, 128, 0, 8, words);
    for (uint32_t i = 0; i < 8; ++i) {
        const uint32_t r5 = ((fb[i] >> 11) & 31u), g6 = ((fb[i] >> 5) & 63u), b5 = (fb[i] & 31u);
        const uint32_t rs = (r5 * 128 + 127) / 255, gs = (g6 * 128 + 127) / 255, bs = (b5 * 128 + 127) / 255;
        const uint32_t expect = (((gs << 2) | (gs >> 4)) << 24) |
                                (((rs << 3) | (rs >> 2)) << 16) |
                                (((bs << 3) | (bs >> 2)) << 8);
        CHECK(words[i] == expect);
    }
}

// ─── Incremental slices == one-shot (driver encodes in bounded slices) ──────

void TestSliceEquivalence() {
    uint16_t fb[32];
    FillPattern(fb, 32);
    DisplaySurface surf = MakeSurface(fb, 8, 4);
    DisplayConfig cfg = MakeConfig(8, 4);
    DisplayMapping map{};
    DisplayPixelSampler sampler(surf, map, cfg);

    uint32_t oneShot[32] = {};
    uint32_t sliced[32] = {};
    LedArray::EncodeGrbSlice(sampler, 200, 0, 32, oneShot);
    uint32_t cursor = 0;
    while (cursor < 32) {
        const uint32_t n = (32 - cursor) < 5 ? (32 - cursor) : 5;  // awkward slice size
        LedArray::EncodeGrbSlice(sampler, 200, cursor, n, sliced + cursor);
        cursor += n;
    }
    CHECK(std::memcmp(oneShot, sliced, sizeof(oneShot)) == 0);
}

} // namespace

int main() {
    TestTimingModel();
    TestClockDividerQuantization();
    TestValidateConfig();
    TestGrbChannelOrder();
    TestRowMajorNonRepeating();
    TestRectangularMapping();
    TestScatterHoles();
    TestReversedMapping();
    TestFlipMapping();
    TestBrightnessEndpoints();
    TestSliceEquivalence();

    std::printf("led_array: %d checks, %d failures\n", gChecks, gFailures);
    if (gFailures) {
        std::printf("RESULT: FAIL\n");
        return 1;
    }
    std::printf("RESULT: PASS\n");
    return 0;
}
