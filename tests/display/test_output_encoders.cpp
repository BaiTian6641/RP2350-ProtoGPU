// Native behavior/bounds regressions for the P07 output backends' pure
// encoding/accounting logic (src/display/hub75_driver.h, spi_display.h).
//
// Deterministic, consumer-visible properties only:
//   * HUB75 byte/bit/row-plane encoder layout (unique byte-packed planes,
//     2 pad bits, exact bit extraction, row-pair r/r+32 pairing)
//   * no old/new frame mixing inside one encoded bank
//   * bounded sliced conversion == one-shot; done cursor reads nothing more
//   * control records: 256/scan, row/plane decomposition, exact dwell
//     doubling, brightness 0 => every dwell 0 (never lights a row),
//     monotonic in brightness, halves when the clock halves (Resume recalc)
//   * workspace accounting fits the shared 68 KiB budget; 64-wide variant
//   * PIO divider math: pixel clock <= 12 MHz, SPI SCLK <= 4 MHz integer
//     dividers across all nine 75..336 MHz profiles
//   * SSD1331 packer: RGB565 high-byte-first, source stride honored,
//     out-of-source mapping samples pack black, slices == one-shot

#include "display/hub75_driver.h"
#include "display/spi_display.h"

#include <cstdio>
#include <cstring>
#include <initializer_list>

namespace {

int g_failures = 0;
constexpr uint32_t kSystemClocks[] = {
    75000000u, 100000000u, 125000000u, 150000000u, 240000000u,
    250000000u, 288000000u, 300000000u, 336000000u
};

#define CHECK(cond)                                                        \
    do {                                                                   \
        if (!(cond)) {                                                     \
            std::printf("FAIL %s:%d: %s\n", __FILE__, __LINE__, #cond);    \
            ++g_failures;                                                  \
        }                                                                  \
    } while (0)

// ─── Fixtures ───────────────────────────────────────────────────────────────

constexpr uint16_t kPanelW = 128, kPanelH = 64;
uint16_t g_surface[kPanelH][kPanelW];

DisplaySurface MakeSurface(uint16_t width = kPanelW, uint16_t height = kPanelH,
                           uint16_t stride = kPanelW) {
    DisplaySurface s = {};
    s.pixels = &g_surface[0][0];
    s.pixelCapacity = uint32_t(stride) * height;
    s.width = width;
    s.height = height;
    s.stride = stride;
    s.frame = 7;
    return s;
}

DisplayConfig PanelConfig(uint16_t width = kPanelW) {
    DisplayConfig c = {};
    c.type = DisplayType::Hub75;
    c.width = width;
    c.height = kPanelH;
    c.brightness = 255;
    return c;
}

void Fill(uint16_t value) {
    for (uint16_t y = 0; y < kPanelH; ++y)
        for (uint16_t x = 0; x < kPanelW; ++x) g_surface[y][x] = value;
}

uint8_t g_bankA[Hub75::ScanDataBytes(kPanelW)];
uint8_t g_bankB[Hub75::ScanDataBytes(kPanelW)];

// ─── HUB75: byte/bit layout ─────────────────────────────────────────────────

void TestByteLayout() {
    Fill(0x0000);
    g_surface[0][0] = 0xffff;   // upper pixel of record 0, column 0
    g_surface[32][0] = 0x0000;  // lower pixel
    DisplaySurface s = MakeSurface();
    DisplayMapping m = {};
    DisplayPixelSampler sampler(s, m, PanelConfig());
    CHECK(sampler.Valid());

    // All-white upper, black lower: every plane -> bits R1|G1|B1 = 0x07,
    // pad bits [7:6] always zero.
    for (uint8_t plane = 0; plane < Hub75::kPlanes; ++plane) {
        const uint8_t b = Hub75::EncodeByte(0xffff, 0x0000, plane);
        CHECK(b == 0x07);
        CHECK((b & 0xc0) == 0);
    }
    // Black upper, white lower -> bits R2|G2|B2 = 0x38.
    for (uint8_t plane = 0; plane < Hub75::kPlanes; ++plane)
        CHECK(Hub75::EncodeByte(0x0000, 0xffff, plane) == 0x38);

    // Exact bit extraction: r5 = 0b10110 -> r8 = 0xB5; plane p must expose
    // bit p of 0xB5 in the R1 position (bit 0).
    const uint16_t pixel = uint16_t(0b10110u << 11);
    CHECK(Hub75::Expand5(0b10110) == 0xb5);
    for (uint8_t plane = 0; plane < 8; ++plane) {
        const uint8_t expect = uint8_t((0xb5 >> plane) & 1u);
        CHECK(Hub75::EncodeByte(pixel, 0x0000, plane) == expect);
    }
    // 6-bit green expand: g6 = 0b101101 -> g8 = 0xB6 (bit in position 1).
    CHECK(Hub75::Expand6(0b101101) == 0xb6);
    for (uint8_t plane = 0; plane < 8; ++plane) {
        const uint8_t expect = uint8_t(((0xb6 >> plane) & 1u) << 1);
        CHECK(Hub75::EncodeByte(uint16_t(0b101101u << 5), 0x0000, plane) == expect);
    }

    // EncodeRecord pairs row r with row r + 32 in column order.
    Fill(0x0000);
    g_surface[5][3] = 0xffff;    // upper row 5, column 3
    g_surface[37][9] = 0xffff;   // lower row 5 + 32, column 9
    uint8_t record[128] = {};
    DisplayPixelSampler s2(s, m, PanelConfig());
    Hub75::EncodeRecord(s2, kPanelW, 5, 0, record);
    CHECK(record[3] == 0x07);    // upper contribution
    CHECK(record[9] == 0x38);    // lower contribution
    uint32_t nonzero = 0;
    for (uint16_t c = 0; c < kPanelW; ++c) nonzero += record[c] ? 1 : 0;
    CHECK(nonzero == 2);
    // ...and row 4 (unrelated pair) stays empty.
    uint8_t record4[128] = {};
    Hub75::EncodeRecord(s2, kPanelW, 4, 0, record4);
    for (uint16_t c = 0; c < kPanelW; ++c) CHECK(record4[c] == 0);
}

// ─── HUB75: plane uniqueness / no frame mixing ──────────────────────────────

void TestPlaneUniquenessAndMixing() {
    // A pixel whose 8-bit red expansion is exactly 0x08 lights only plane 3.
    Fill(0);
    for (uint16_t y = 0; y < Hub75::kScanRows; ++y)
        for (uint16_t x = 0; x < kPanelW; ++x) g_surface[y][x] = uint16_t(1u << 11);
    DisplaySurface s = MakeSurface();
    DisplayMapping m = {};
    DisplayPixelSampler sampler(s, m, PanelConfig());
    Hub75::EncodeCursor cursor;
    const uint16_t total =
        Hub75::EncodeSlice(sampler, kPanelW, g_bankA, cursor, Hub75::kRecordCount);
    CHECK(total == Hub75::kRecordCount);
    CHECK(cursor.Done());
    for (uint16_t rec = 0; rec < Hub75::kRecordCount; ++rec) {
        const uint8_t plane = uint8_t(rec % Hub75::kPlanes);
        // Red bit 3 set => R1 bit only; G8/B8 of value 1<<11 are 0.
        const uint8_t want = plane == 3 ? 0x01 : 0x00;
        for (uint16_t c = 0; c < kPanelW; ++c)
            CHECK(g_bankA[uint32_t(rec) * kPanelW + c] == want);
    }

    // Frame isolation: bank A = solid red, bank B = solid blue.  No record in
    // either bank may contain the other frame's channel bits.
    Fill(0xf800);  // red
    DisplayPixelSampler sa(s, m, PanelConfig());
    Hub75::EncodeCursor ca;
    Hub75::EncodeSlice(sa, kPanelW, g_bankA, ca, Hub75::kRecordCount);
    Fill(0x001f);  // blue
    DisplayPixelSampler sb(s, m, PanelConfig());
    Hub75::EncodeCursor cb;
    Hub75::EncodeSlice(sb, kPanelW, g_bankB, cb, Hub75::kRecordCount);
    for (uint32_t i = 0; i < Hub75::ScanDataBytes(kPanelW); ++i) {
        CHECK((g_bankA[i] & 0x36) == 0);   // G1|B1|G2|B2 absent in the red bank
        CHECK((g_bankB[i] & 0x1b) == 0);   // no red/green bits in the blue bank
        CHECK(g_bankA[i] == 0x09);         // R1|R2 set on every plane (r8=0xff)
        CHECK(g_bankB[i] == 0x24);         // B1|B2 set on every plane (b8=0xff)
    }
}

// ─── HUB75: bounded slices, bounded source hold ─────────────────────────────

void TestSlicedEncoding() {
    // Deterministic pseudo-gradient content.
    for (uint16_t y = 0; y < kPanelH; ++y)
        for (uint16_t x = 0; x < kPanelW; ++x)
            g_surface[y][x] = uint16_t((y * 977 + x * 131) & 0xffff);
    DisplaySurface s = MakeSurface();
    DisplayMapping m = {};

    DisplayPixelSampler oneShot(s, m, PanelConfig());
    Hub75::EncodeCursor c0;
    Hub75::EncodeSlice(oneShot, kPanelW, g_bankA, c0, Hub75::kRecordCount);

    for (uint16_t slice : { uint16_t(1), uint16_t(7), uint16_t(256) }) {
        DisplayPixelSampler sliced(s, m, PanelConfig());
        Hub75::EncodeCursor c1;
        std::memset(g_bankB, 0xaa, sizeof(g_bankB));
        uint16_t calls = 0;
        while (!c1.Done()) {
            const uint16_t n =
                Hub75::EncodeSlice(sliced, kPanelW, g_bankB, c1, slice);
            CHECK(n == slice || c1.Done());
            CHECK(++calls <= Hub75::kRecordCount);
        }
        CHECK(c1.record == Hub75::kRecordCount);
        CHECK(std::memcmp(g_bankA, g_bankB, sizeof(g_bankA)) == 0);
        // Done cursor: a further slice touches nothing (bounded source hold).
        const uint16_t extra = Hub75::EncodeSlice(sliced, kPanelW, g_bankB, c1, 8);
        CHECK(extra == 0);
    }
}

// ─── HUB75: control records / dwells ────────────────────────────────────────

void TestControlRecords() {
    uint32_t bank[Hub75::kRecordCount];

    Hub75::EncodeControlBank(bank, 255, 150000000);
    CHECK(Hub75::BaseDwellCycles(150000000) == 24);   // 160 ns at 150 MHz
    CHECK(Hub75::BaseDwellCycles(100000000) == 16);
    CHECK(Hub75::BaseDwellCycles(75000000) == 12);
    // The physical base pulse must cover 160 ns even off exact clock ratios.
    CHECK(Hub75::BaseDwellCycles(150000001) == 25);
    CHECK(Hub75::BaseDwellCycles(75000001) == 13);

    // Record 0: row 0, plane 0 -> dwell 24 cycles -> stored 23.
    CHECK(bank[0] == (0u | (23u << 5)));
    // Record 1: row 0, plane 1 -> dwell 48 -> stored 47.
    CHECK(bank[1] == (0u | (47u << 5)));
    // Record 8: row 1, plane 0.
    CHECK(bank[8] == (1u | (23u << 5)));
    // Plane dwell doubles exactly.
    for (uint8_t p = 1; p < Hub75::kPlanes; ++p)
        CHECK(Hub75::DwellCycles(p, 255, 150000000) ==
              2 * Hub75::DwellCycles(p - 1, 255, 150000000));
    // Row field covers 0..31, each exactly kPlanes times; dwell in 27 bits.
    uint8_t rowSeen[32] = {};
    for (uint16_t i = 0; i < Hub75::kRecordCount; ++i) {
        const uint32_t row = bank[i] & 31u;
        ++rowSeen[row];
        CHECK((bank[i] >> 5) < (1u << 27));
        CHECK(row == i / Hub75::kPlanes);
    }
    for (uint8_t r = 0; r < 32; ++r) CHECK(rowSeen[r] == Hub75::kPlanes);

    // Brightness 0: every encoded dwell is 0 -> the row SM never asserts OE.
    Hub75::EncodeControlBank(bank, 0, 150000000);
    for (uint16_t i = 0; i < Hub75::kRecordCount; ++i) CHECK((bank[i] >> 5) == 0);

    // Monotonic non-decreasing in brightness for every plane.
    for (uint8_t p = 0; p < Hub75::kPlanes; ++p) {
        uint32_t prev = 0;
        for (uint32_t b = 0; b <= 255; ++b) {
            const uint32_t d = Hub75::DwellCycles(p, uint8_t(b), 150000000);
            CHECK(d >= prev);
            prev = d;
        }
    }
    // Half the clock halves the dwell (Resume recalculates, content-free).
    for (uint8_t p = 0; p < Hub75::kPlanes; ++p)
        CHECK(Hub75::DwellCycles(p, 255, 75000000) * 2 ==
              Hub75::DwellCycles(p, 255, 150000000));
    // Resume rebuilds the actual packed control bank at each clock. Decode
    // every row/plane and brightness, checking quantization without overflow.
    for (uint32_t hz : kSystemClocks) {
        for (uint32_t b = 0; b <= 255; ++b) {
            Hub75::EncodeControlBank(bank, uint8_t(b), hz);
            for (uint16_t i = 0; i < Hub75::kRecordCount; ++i) {
                const uint32_t dwell = Hub75::DwellCycles(i % 8, uint8_t(b), hz);
                CHECK((bank[i] & 31u) == i / 8u);
                CHECK((bank[i] >> 5) == (dwell >= 2u ? dwell - 1u : 0u));
                if (b == 255u) {
                    // Base dwell covers 160 ns, rounded up by <1 sys cycle.
                    const uint64_t duration = uint64_t(dwell) * 1000000000ull;
                    const uint64_t target = uint64_t(160u << (i % 8)) * hz;
                    CHECK(duration >= target);
                    CHECK(duration - target < (1000000000ull << (i % 8)));
                }
            }
        }
    }
}

// ─── HUB75: sizing / dividers ───────────────────────────────────────────────

void TestHub75SizingAndDividers() {
    CHECK(Hub75::ScanDataBytes(128) == 32768);
    CHECK(Hub75::ControlBankBytes() == 1024);
    CHECK(Hub75::RequiredWorkspaceBytes(128) == 67584);
    CHECK(Hub75::RequiredWorkspaceBytes(128) <= GpuConfig::DISPLAY_WORKSPACE_BYTES);
    CHECK(Hub75::ScanDataBytes(64) == 16384);
    CHECK(Hub75::RequiredWorkspaceBytes(64) == 34816);
    CHECK(Hub75::SupportedWidth(128) && Hub75::SupportedWidth(64));
    CHECK(!Hub75::SupportedWidth(96));

    // Pixel clock = hz / (3 * div) must stay <= 12 MHz at every profile clock.
    for (uint32_t hz : kSystemClocks) {
        const uint32_t div8 = Hub75::PixelClockDivFrac8(hz);
        const uint64_t actual = uint64_t(hz) * 256 / (3ull * div8);
        CHECK(actual <= Hub75::kPixelClockHz);
        CHECK(actual > Hub75::kPixelClockHz * 9 / 10);  // stays near target
    }
}

// ─── SSD1331: dividers / packing ────────────────────────────────────────────

void TestSpiDividers() {
    CHECK(SpiDisplay::kFrameBytes == 12288);
    CHECK(SpiDisplay::kFrameBytes <= GpuConfig::DISPLAY_WORKSPACE_BYTES);
    for (uint32_t hz : kSystemClocks) {
        const uint32_t sclk = SpiDisplay::ActualSclkHz(hz);
        CHECK(sclk <= SpiDisplay::kMaxSclkHz);        // <= 4 MHz
        CHECK(sclk <= 6666666u);                      // vendor serial ceiling
        CHECK(sclk >= SpiDisplay::kMaxSclkHz * 7 / 8);
    }
    CHECK(SpiDisplay::ClockDivInt(150000000) == 19);  // exact integer dividers
    CHECK(SpiDisplay::ClockDivInt(100000000) == 13);
    CHECK(SpiDisplay::ClockDivInt(75000000) == 10);
    CHECK(SpiDisplay::ClockDivInt(240000000) == 30);
    CHECK(SpiDisplay::ClockDivInt(288000000) == 36);
    CHECK(SpiDisplay::ClockDivInt(336000000) == 42);
    CHECK(SpiDisplay::ClockDivInt(UINT32_MAX) == 537);  // no ceil-add overflow
    CHECK(SpiDisplay::ActualSclkHz(UINT32_MAX) <= SpiDisplay::kMaxSclkHz);
}

uint16_t g_spiSurface[64][112];   // stride 112 > width 96
uint8_t g_packed[SpiDisplay::kFrameBytes];
uint8_t g_packedRef[SpiDisplay::kFrameBytes];

void TestSpiPacking() {
    // Stride-honoring content: value encodes (y, x).
    for (uint16_t y = 0; y < SpiDisplay::kHeight; ++y)
        for (uint16_t x = 0; x < 112; ++x)
            g_spiSurface[y][x] = uint16_t(0xa000 | ((y & 7) << 8) | (x & 0xff));

    DisplaySurface s = {};
    s.pixels = &g_spiSurface[0][0];
    s.pixelCapacity = 112 * SpiDisplay::kHeight;
    s.width = SpiDisplay::kWidth;
    s.height = SpiDisplay::kHeight;
    s.stride = 112;
    DisplayMapping m = {};
    DisplayConfig c = {};
    c.type = DisplayType::SpiSsd1331;
    c.width = SpiDisplay::kWidth;
    c.height = SpiDisplay::kHeight;

    DisplayPixelSampler sampler(s, m, c);
    CHECK(sampler.Valid());
    SpiDisplay::PackRgb565BE(sampler, g_packedRef, 0,
                             uint32_t(SpiDisplay::kWidth) * SpiDisplay::kHeight);

    // High byte first for every pixel, stride respected (x < 96 window).
    for (uint16_t y = 0; y < SpiDisplay::kHeight; ++y) {
        for (uint16_t x = 0; x < SpiDisplay::kWidth; ++x) {
            const uint32_t i = uint32_t(y) * SpiDisplay::kWidth + x;
            const uint16_t v = g_spiSurface[y][x];
            CHECK(g_packedRef[2 * i] == uint8_t(v >> 8));
            CHECK(g_packedRef[2 * i + 1] == uint8_t(v & 0xff));
        }
    }
    // Spot-check a known pattern: pixel (x=1, y=1) -> 0xa101 -> A1 01.
    CHECK(g_packedRef[2 * (SpiDisplay::kWidth + 1)] == 0xa1);
    CHECK(g_packedRef[2 * (SpiDisplay::kWidth + 1) + 1] == 0x01);
    // First/last pixel of the frame.
    CHECK(g_packedRef[0] == uint8_t(g_spiSurface[0][0] >> 8));
    CHECK(g_packedRef[SpiDisplay::kFrameBytes - 1] ==
          uint8_t(g_spiSurface[SpiDisplay::kHeight - 1][SpiDisplay::kWidth - 1] & 0xff));

    // Bounded slices concatenate to the one-shot image.
    std::memset(g_packed, 0, sizeof(g_packed));
    uint32_t cursor = 0;
    while (cursor < uint32_t(SpiDisplay::kWidth) * SpiDisplay::kHeight) {
        const uint32_t n = 1024;
        SpiDisplay::PackRgb565BE(sampler, g_packed + 2 * cursor, cursor, n);
        cursor += n;
    }
    CHECK(std::memcmp(g_packed, g_packedRef, sizeof(g_packed)) == 0);

    // Rectangular mapping reaching outside the source packs black there.
    PglRectLayoutData rect = {};
    rect.size = { 480.f, 320.f };       // 96x64 cells of 5x5 source pixels
    rect.position = { -20.f, -10.f };   // top-left samples fall outside
    rect.rowCount = SpiDisplay::kHeight;
    rect.colCount = SpiDisplay::kWidth;
    DisplayMapping rm = {};
    rm.rectangular = true;
    rm.rectangle = rect;
    DisplayPixelSampler outside(s, rm, c);
    CHECK(outside.Valid());
    SpiDisplay::PackRgb565BE(outside, g_packed, 0,
                             uint32_t(SpiDisplay::kWidth) * SpiDisplay::kHeight);
    // Panel pixel (0,0): source (-17.5, -7.5) -> black.
    CHECK(g_packed[0] == 0 && g_packed[1] == 0);
    // Panel pixel (95,63): source (457.5+? , ...) also outside -> black.
    CHECK(g_packed[SpiDisplay::kFrameBytes - 2] == 0);
    CHECK(g_packed[SpiDisplay::kFrameBytes - 1] == 0);
    // A clearly inside sample (col 8, row 4 -> source x=22.5, y=12.5) is not black.
    const uint32_t inside = 4 * SpiDisplay::kWidth + 8;
    CHECK(g_packed[2 * inside] == uint8_t(g_spiSurface[12][22] >> 8));
}

} // namespace

int main() {
    TestByteLayout();
    TestPlaneUniquenessAndMixing();
    TestSlicedEncoding();
    TestControlRecords();
    TestHub75SizingAndDividers();
    TestSpiDividers();
    TestSpiPacking();

    if (g_failures) {
        std::printf("RESULT: FAIL (%d checks)\n", g_failures);
        return 1;
    }
    std::printf("RESULT: PASS (output encoder/state checks)\n");
    return 0;
}
