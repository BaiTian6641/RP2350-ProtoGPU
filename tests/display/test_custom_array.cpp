// Custom 8-bit parallel RGB888 array native encoder tests (P08).
//
// Exercises the REAL portable stream geometry, divider and encoding code
// from src/display/custom_array.{h,cpp} plus the parent-owned
// DisplayPixelSampler mapping semantics the driver caches per borrowed
// surface. No PIO/DMA is faked; hardware qualification remains an open gate.
//
// Build and run via tests/display/run_tests.sh (native g++, no Pico SDK).

#include "display/custom_array.h"

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
    c.type = DisplayType::CustomRgb888;
    c.width = w;
    c.height = h;
    c.brightness = brightness;
    return c;
}

// Expected byte triple for one pixel at brightness 255 (R, G, B order).
void ExpectTriple(uint16_t rgb565, uint8_t& r, uint8_t& g, uint8_t& b) {
    // Independent arithmetic oracle; never call the encoder under test.
    const uint32_t red = rgb565 / 2048u;
    const uint32_t green = (rgb565 / 32u) % 64u;
    const uint32_t blue = rgb565 % 32u;
    r = uint8_t(red * 8u + red / 4u);
    g = uint8_t(green * 4u + green / 16u);
    b = uint8_t(blue * 8u + blue / 4u);
}

// ─── Clock / stream geometry ────────────────────────────────────────────────

void TestClockModel() {
    CHECK(CustomArray::kByteClockHz == 4000000);
    CHECK(CustomArray::kSmClockHz == 2 * CustomArray::kByteClockHz);

    CustomArray::ClockDivider d{};
    CHECK(CustomArray::ComputeClockDivider(150000000, d));
    CHECK(d.integer == 18 && d.fraction == 192);  // 18.75
    CHECK(CustomArray::ComputeClockDivider(100000000, d));
    CHECK(d.integer == 12 && d.fraction == 128);  // 12.5
    CHECK(CustomArray::ComputeClockDivider(75000000, d));
    CHECK(d.integer == 9 && d.fraction == 96);    // 9.375
    for (uint32_t sys : {75000000u, 100000000u, 125000000u, 150000000u,
                         240000000u, 250000000u, 288000000u, 300000000u,
                         336000000u}) {
        CHECK(CustomArray::ComputeClockDivider(sys, d));
        const uint64_t represented = (uint64_t(d.integer) * 256 + d.fraction);
        CHECK(represented * CustomArray::kSmClockHz == uint64_t(sys) * 256u);
        // The two-instruction byte cell retains exactly 4 MHz, including
        // the new 48 MHz-aligned profiles; no divider truncation is allowed.
        CHECK(uint64_t(sys) * 256u <= represented * 2u * 4000000u);
    }
    CHECK(CustomArray::ComputeClockDivider(240000000, d));
    CHECK(d.integer == 30 && d.fraction == 0);
    CHECK(CustomArray::ComputeClockDivider(288000000, d));
    CHECK(d.integer == 36 && d.fraction == 0);
    CHECK(CustomArray::ComputeClockDivider(336000000, d));
    CHECK(d.integer == 42 && d.fraction == 0);
    CHECK(!CustomArray::ComputeClockDivider(0, d));
    CHECK(!CustomArray::ComputeClockDivider(100000001, d));
    CHECK(!CustomArray::ComputeClockDivider(3000000, d));
}

void TestStreamGeometry() {
    // Complete byte counts and leading pad for pixel counts 1..6.
    for (uint32_t p = 1; p <= 6; ++p) {
        CHECK(CustomArray::ByteCount(p) == 3 * p);
        const uint32_t pad = CustomArray::LeadingPadBytes(p);
        CHECK(pad == (4 - (3 * p) % 4) % 4);
        CHECK((pad + 3 * p) % 4 == 0);
        CHECK(CustomArray::WordCount(p) == (pad + 3 * p) / 4);
    }
    CHECK(CustomArray::LeadingPadBytes(1) == 1);
    CHECK(CustomArray::LeadingPadBytes(2) == 2);
    CHECK(CustomArray::LeadingPadBytes(3) == 3);
    CHECK(CustomArray::LeadingPadBytes(4) == 0);
    CHECK(CustomArray::WordCount(4) == 3);

    // 68 KiB workspace boundary: 23210 pixels fit exactly, 23211 do not.
    CHECK(CustomArray::MaxPixels(68 * 1024) == 23210);
    CHECK(CustomArray::WordCount(23210) * 4u <= 68u * 1024u);
    CHECK(CustomArray::WordCount(23211) * 4u > 68u * 1024u);
    // Bounded transfer budget: 0.25 us per stream byte plus margin.
    CHECK(CustomArray::StreamBudgetUs(4) == 3 + 500);
}

void TestValidateConfig() {
    uint32_t pixels = 0;
    DisplayConfig c = MakeConfig(16, 16);
    CHECK(CustomArray::ValidateConfig(c, 1024, pixels) == PglRuntime::Result::Ok);
    CHECK(pixels == 256);

    c.type = DisplayType::LedV5;  // wrong backend
    CHECK(CustomArray::ValidateConfig(c, 1024, pixels) == PglRuntime::Result::InvalidValue);
    c = MakeConfig(0, 8);
    CHECK(CustomArray::ValidateConfig(c, 1024, pixels) == PglRuntime::Result::InvalidValue);
    c = MakeConfig(8, 0);
    CHECK(CustomArray::ValidateConfig(c, 1024, pixels) == PglRuntime::Result::InvalidValue);

    c = MakeConfig(300, 300);  // 90000 pixels > 23210
    CHECK(CustomArray::ValidateConfig(c, 68 * 1024, pixels) == PglRuntime::Result::Capacity);
    c = MakeConfig(23210 / 100, 100);  // 23200 fits
    CHECK(CustomArray::ValidateConfig(c, 68 * 1024, pixels) == PglRuntime::Result::Ok);
    c = MakeConfig(16, 16);
    CHECK(CustomArray::ValidateConfig(c, 767, pixels) == PglRuntime::Result::Capacity);
}

// ─── Byte order, packing, padded tails ──────────────────────────────────────

void TestChannelByteOrder() {
    // Per pixel exactly three bytes on the wire in R, G, B order.
    const uint16_t mixed = static_cast<uint16_t>((22u << 11) | (44u << 5) | 9u);
    uint8_t r, g, b;
    ExpectTriple(mixed, r, g, b);
    CHECK(r == 0xB5 && g == 0xB2 && b == 0x4A);

    uint16_t fb[1] = {mixed};
    DisplaySurface surf = MakeSurface(fb, 1, 1);
    DisplayConfig cfg = MakeConfig(1, 1);
    DisplayMapping map{};
    DisplayPixelSampler sampler(surf, map, cfg);
    CHECK(sampler.Valid());

    // One pixel: pad = 1 leading zero byte, then R, G, B.
    uint8_t stream[4] = {0xEE, 0xEE, 0xEE, 0xEE};
    stream[0] = 0;  // driver zeroes the leading pad
    CustomArray::EncodeRgb888Slice(sampler, 255, 0, 1, stream + 1);
    CHECK(stream[1] == 0xB5);  // R first
    CHECK(stream[2] == 0xB2);  // G second
    CHECK(stream[3] == 0x4A);  // B third
    // Little-endian DMA word: first wire byte is the word LSB.
    CHECK(CustomArray::PackStreamWord(stream, 0) == 0x4AB2B500u);
}

void TestFullCountPaddedTails() {
    // Five distinct pixels: pad = 1, four full words, no trailing garbage.
    uint16_t fb[5];
    FillPattern(fb, 5);
    DisplaySurface surf = MakeSurface(fb, 5, 1);
    DisplayConfig cfg = MakeConfig(5, 1);
    DisplayMapping map{};
    DisplayPixelSampler sampler(surf, map, cfg);

    uint8_t stream[16];
    std::memset(stream, 0xEE, sizeof(stream));
    stream[0] = 0;  // leading pad
    CustomArray::EncodeRgb888Slice(sampler, 255, 0, 5, stream + 1);

    uint8_t r[5], g[5], b[5];
    for (uint32_t i = 0; i < 5; ++i) ExpectTriple(fb[i], r[i], g[i], b[i]);
    CHECK(CustomArray::PackStreamWord(stream, 0) ==
          (uint32_t(0) | (uint32_t(r[0]) << 8) | (uint32_t(g[0]) << 16) | (uint32_t(b[0]) << 24)));
    CHECK(CustomArray::PackStreamWord(stream, 1) ==
          (uint32_t(r[1]) | (uint32_t(g[1]) << 8) | (uint32_t(b[1]) << 16) | (uint32_t(r[2]) << 24)));
    CHECK(CustomArray::PackStreamWord(stream, 2) ==
          (uint32_t(g[2]) | (uint32_t(b[2]) << 8) | (uint32_t(r[3]) << 16) | (uint32_t(g[3]) << 24)));
    // Final word holds exactly the end of the complete byte count.
    CHECK(CustomArray::PackStreamWord(stream, 3) ==
          (uint32_t(b[3]) | (uint32_t(r[4]) << 8) | (uint32_t(g[4]) << 16) | (uint32_t(b[4]) << 24)));

    // Four pixels: no pad at all, word 0 starts with pixel 0 red.
    DisplaySurface surf4 = MakeSurface(fb, 4, 1);
    DisplayConfig cfg4 = MakeConfig(4, 1);
    DisplayPixelSampler sampler4(surf4, map, cfg4);
    uint8_t stream4[12];
    CustomArray::EncodeRgb888Slice(sampler4, 255, 0, 4, stream4);
    CHECK(CustomArray::PackStreamWord(stream4, 0) ==
          (uint32_t(r[0]) | (uint32_t(g[0]) << 8) | (uint32_t(b[0]) << 16) | (uint32_t(r[1]) << 24)));
}

// ─── Mapping behavior through the cached sampler ────────────────────────────

void TestRowMajorNonRepeating() {
    uint16_t fb[24];
    FillPattern(fb, 24);
    DisplaySurface surf = MakeSurface(fb, 6, 4);
    DisplayConfig cfg = MakeConfig(6, 4);
    DisplayMapping map{};
    DisplayPixelSampler sampler(surf, map, cfg);
    CHECK(sampler.Valid());

    uint8_t stream[72];
    CustomArray::EncodeRgb888Slice(sampler, 255, 0, 24, stream);
    uint32_t distinct = 0;
    for (uint32_t i = 0; i < 24; ++i) {
        uint8_t r, g, b;
        ExpectTriple(fb[i], r, g, b);
        CHECK(stream[3 * i] == r && stream[3 * i + 1] == g && stream[3 * i + 2] == b);
        const uint32_t packed = (uint32_t(stream[3 * i]) << 0) +
            (uint32_t(stream[3 * i + 1]) << 12) + (uint32_t(stream[3 * i + 2]) << 20);
        bool seen = false;
        for (uint32_t j = 0; j < i; ++j) {
            const uint32_t prev = (uint32_t(stream[3 * j]) << 0) +
                (uint32_t(stream[3 * j + 1]) << 12) + (uint32_t(stream[3 * j + 2]) << 20);
            seen = seen || prev == packed;
        }
        if (!seen) ++distinct;
    }
    CHECK(distinct == 24);
}

void TestScatterHolesAndReversal() {
    uint16_t fb[16];
    FillPattern(fb, 16);
    DisplaySurface surf = MakeSurface(fb, 4, 4);
    DisplayConfig cfg = MakeConfig(3, 2);  // 6 outputs, 3 mapped
    const PglVec2 coords[3] = {{1.0f, 1.0f}, {50.0f, 0.0f}, {2.0f, 3.0f}};
    DisplayMapping map{};
    map.coordinates = coords;
    map.count = 3;
    DisplayPixelSampler sampler(surf, map, cfg);
    CHECK(sampler.Valid());

    uint8_t stream[18];
    CustomArray::EncodeRgb888Slice(sampler, 255, 0, 6, stream);
    uint8_t r, g, b;
    ExpectTriple(fb[1 * 4 + 1], r, g, b);  // (1,1)
    CHECK(stream[0] == r && stream[1] == g && stream[2] == b);
    CHECK(stream[3] == 0 && stream[4] == 0 && stream[5] == 0);      // hole: x=50
    ExpectTriple(fb[3 * 4 + 2], r, g, b);  // (2,3)
    CHECK(stream[6] == r && stream[7] == g && stream[8] == b);
    for (uint32_t i = 9; i < 18; ++i) CHECK(stream[i] == 0);         // beyond count

    // Reversed order over the same mapping.
    DisplayMapping rev = map;
    rev.reversed = true;
    DisplayPixelSampler rsampler(surf, rev, cfg);
    uint8_t rstream[18];
    CustomArray::EncodeRgb888Slice(rsampler, 255, 0, 6, rstream);
    // Explicit reversal check: output i reads logical index 5 - i.
    for (uint32_t i = 0; i < 6; ++i) {
        CHECK(rstream[3 * i] == stream[3 * (5 - i)]);
        CHECK(rstream[3 * i + 1] == stream[3 * (5 - i) + 1]);
        CHECK(rstream[3 * i + 2] == stream[3 * (5 - i) + 2]);
    }
}

void TestFlipMapping() {
    uint16_t fb[4];
    FillPattern(fb, 4);
    DisplaySurface surf = MakeSurface(fb, 2, 2);
    DisplayMapping map{};
    DisplayConfig cfg = MakeConfig(2, 2);
    cfg.flipH = true;
    cfg.flipV = true;
    DisplayPixelSampler sampler(surf, map, cfg);

    uint8_t stream[12];
    CustomArray::EncodeRgb888Slice(sampler, 255, 0, 4, stream);
    // Both flips: (0,0) -> (1,1), (1,0) -> (0,1), (0,1) -> (1,0), (1,1) -> (0,0).
    const uint8_t expectIndex[4] = {3, 2, 1, 0};
    for (uint32_t i = 0; i < 4; ++i) {
        uint8_t r, g, b;
        ExpectTriple(fb[expectIndex[i]], r, g, b);
        CHECK(stream[3 * i] == r && stream[3 * i + 1] == g && stream[3 * i + 2] == b);
    }
}

// ─── Brightness endpoints ───────────────────────────────────────────────────

void TestBrightnessEndpoints() {
    uint16_t fb[8];
    FillPattern(fb, 8);
    DisplaySurface surf = MakeSurface(fb, 8, 1);
    DisplayConfig cfg = MakeConfig(8, 1);
    DisplayMapping map{};
    DisplayPixelSampler sampler(surf, map, cfg);

    uint8_t stream[24];
    CustomArray::EncodeRgb888Slice(sampler, 0, 0, 8, stream);
    for (uint32_t i = 0; i < 24; ++i) CHECK(stream[i] == 0);  // exact black

    CustomArray::EncodeRgb888Slice(sampler, 255, 0, 8, stream);
    for (uint32_t i = 0; i < 8; ++i) {
        uint8_t r, g, b;
        ExpectTriple(fb[i], r, g, b);
        CHECK(stream[3 * i] == r && stream[3 * i + 1] == g && stream[3 * i + 2] == b);
    }

    CustomArray::EncodeRgb888Slice(sampler, 128, 0, 8, stream);
    for (uint32_t i = 0; i < 8; ++i) {
        const uint32_t r5 = ((fb[i] >> 11) & 31u), g6 = ((fb[i] >> 5) & 63u), b5 = (fb[i] & 31u);
        const uint32_t rs = (r5 * 128 + 127) / 255, gs = (g6 * 128 + 127) / 255, bs = (b5 * 128 + 127) / 255;
        CHECK(stream[3 * i] == ((rs << 3) | (rs >> 2)));
        CHECK(stream[3 * i + 1] == ((gs << 2) | (gs >> 4)));
        CHECK(stream[3 * i + 2] == ((bs << 3) | (bs >> 2)));
    }
}

// ─── Incremental slices == one-shot ─────────────────────────────────────────

void TestSliceEquivalence() {
    uint16_t fb[16];
    FillPattern(fb, 16);
    DisplaySurface surf = MakeSurface(fb, 4, 4);
    DisplayConfig cfg = MakeConfig(4, 4);
    DisplayMapping map{};
    DisplayPixelSampler sampler(surf, map, cfg);

    uint8_t oneShot[48] = {};
    uint8_t sliced[48] = {};
    CustomArray::EncodeRgb888Slice(sampler, 200, 0, 16, oneShot);
    uint32_t cursor = 0;
    while (cursor < 16) {
        const uint32_t n = (16 - cursor) < 5 ? (16 - cursor) : 5;
        CustomArray::EncodeRgb888Slice(sampler, 200, cursor, n, sliced + 3 * cursor);
        cursor += n;
    }
    CHECK(std::memcmp(oneShot, sliced, sizeof(oneShot)) == 0);
}

} // namespace

int main() {
    TestClockModel();
    TestStreamGeometry();
    TestValidateConfig();
    TestChannelByteOrder();
    TestFullCountPaddedTails();
    TestRowMajorNonRepeating();
    TestScatterHolesAndReversal();
    TestFlipMapping();
    TestBrightnessEndpoints();
    TestSliceEquivalence();

    std::printf("custom_array: %d checks, %d failures\n", gChecks, gFailures);
    if (gFailures) {
        std::printf("RESULT: FAIL\n");
        return 1;
    }
    std::printf("RESULT: PASS\n");
    return 0;
}
