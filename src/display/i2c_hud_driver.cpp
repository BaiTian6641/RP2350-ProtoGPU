/**
 * @file i2c_hud_driver.cpp
 * @brief I2C HUD OLED display driver implementation — SSD1306/SSD1309 (M11).
 *
 * Uses I2C1 (shared with the GPU I2C slave, but at a different address 0x3D).
 * The GPU I2C slave runs at 0x3C; the HUD OLED runs at 0x3D.
 *
 * Note: The RP2350 I2C peripheral can be both master and slave simultaneously
 * on the same bus if addresses differ. However, the I2C slave IRQ handler
 * already occupies I2C1. This driver uses blocking I2C master writes to the
 * OLED device address only when the bus is idle (between slave transactions).
 *
 * For board revisions where the OLED is on a separate I2C bus (I2C0), the
 * i2c instance can be changed in gpu_config.h.
 */

#include "i2c_hud_driver.h"
#include "pico/stdlib.h"
#include "hardware/i2c.h"

#include <cstdio>
#include <cstring>

// ─── Built-in 5×7 Bitmap Font ──────────────────────────────────────────────
// ASCII 0x20 – 0x7E (95 printable characters), 5 bytes per character.
// Each byte is a column, LSB = top pixel.

static const uint8_t FONT_5X7[][5] = {
    {0x00,0x00,0x00,0x00,0x00}, // ' '
    {0x00,0x00,0x5F,0x00,0x00}, // '!'
    {0x00,0x07,0x00,0x07,0x00}, // '"'
    {0x14,0x7F,0x14,0x7F,0x14}, // '#'
    {0x24,0x2A,0x7F,0x2A,0x12}, // '$'
    {0x23,0x13,0x08,0x64,0x62}, // '%'
    {0x36,0x49,0x55,0x22,0x50}, // '&'
    {0x00,0x05,0x03,0x00,0x00}, // '''
    {0x00,0x1C,0x22,0x41,0x00}, // '('
    {0x00,0x41,0x22,0x1C,0x00}, // ')'
    {0x08,0x2A,0x1C,0x2A,0x08}, // '*'
    {0x08,0x08,0x3E,0x08,0x08}, // '+'
    {0x00,0x50,0x30,0x00,0x00}, // ','
    {0x08,0x08,0x08,0x08,0x08}, // '-'
    {0x00,0x60,0x60,0x00,0x00}, // '.'
    {0x20,0x10,0x08,0x04,0x02}, // '/'
    {0x3E,0x51,0x49,0x45,0x3E}, // '0'
    {0x00,0x42,0x7F,0x40,0x00}, // '1'
    {0x42,0x61,0x51,0x49,0x46}, // '2'
    {0x21,0x41,0x45,0x4B,0x31}, // '3'
    {0x18,0x14,0x12,0x7F,0x10}, // '4'
    {0x27,0x45,0x45,0x45,0x39}, // '5'
    {0x3C,0x4A,0x49,0x49,0x30}, // '6'
    {0x01,0x71,0x09,0x05,0x03}, // '7'
    {0x36,0x49,0x49,0x49,0x36}, // '8'
    {0x06,0x49,0x49,0x29,0x1E}, // '9'
    {0x00,0x36,0x36,0x00,0x00}, // ':'
    {0x00,0x56,0x36,0x00,0x00}, // ';'
    {0x00,0x08,0x14,0x22,0x41}, // '<'
    {0x14,0x14,0x14,0x14,0x14}, // '='
    {0x41,0x22,0x14,0x08,0x00}, // '>'
    {0x02,0x01,0x51,0x09,0x06}, // '?'
    {0x32,0x49,0x79,0x41,0x3E}, // '@'
    {0x7E,0x11,0x11,0x11,0x7E}, // 'A'
    {0x7F,0x49,0x49,0x49,0x36}, // 'B'
    {0x3E,0x41,0x41,0x41,0x22}, // 'C'
    {0x7F,0x41,0x41,0x22,0x1C}, // 'D'
    {0x7F,0x49,0x49,0x49,0x41}, // 'E'
    {0x7F,0x09,0x09,0x01,0x01}, // 'F'
    {0x3E,0x41,0x41,0x51,0x32}, // 'G'
    {0x7F,0x08,0x08,0x08,0x7F}, // 'H'
    {0x00,0x41,0x7F,0x41,0x00}, // 'I'
    {0x20,0x40,0x41,0x3F,0x01}, // 'J'
    {0x7F,0x08,0x14,0x22,0x41}, // 'K'
    {0x7F,0x40,0x40,0x40,0x40}, // 'L'
    {0x7F,0x02,0x04,0x02,0x7F}, // 'M'
    {0x7F,0x04,0x08,0x10,0x7F}, // 'N'
    {0x3E,0x41,0x41,0x41,0x3E}, // 'O'
    {0x7F,0x09,0x09,0x09,0x06}, // 'P'
    {0x3E,0x41,0x51,0x21,0x5E}, // 'Q'
    {0x7F,0x09,0x19,0x29,0x46}, // 'R'
    {0x46,0x49,0x49,0x49,0x31}, // 'S'
    {0x01,0x01,0x7F,0x01,0x01}, // 'T'
    {0x3F,0x40,0x40,0x40,0x3F}, // 'U'
    {0x1F,0x20,0x40,0x20,0x1F}, // 'V'
    {0x7F,0x20,0x18,0x20,0x7F}, // 'W'
    {0x63,0x14,0x08,0x14,0x63}, // 'X'
    {0x03,0x04,0x78,0x04,0x03}, // 'Y'
    {0x61,0x51,0x49,0x45,0x43}, // 'Z'
    {0x00,0x00,0x7F,0x41,0x41}, // '['
    {0x02,0x04,0x08,0x10,0x20}, // '\'
    {0x41,0x41,0x7F,0x00,0x00}, // ']'
    {0x04,0x02,0x01,0x02,0x04}, // '^'
    {0x40,0x40,0x40,0x40,0x40}, // '_'
    {0x00,0x01,0x02,0x04,0x00}, // '`'
    {0x20,0x54,0x54,0x54,0x78}, // 'a'
    {0x7F,0x48,0x44,0x44,0x38}, // 'b'
    {0x38,0x44,0x44,0x44,0x20}, // 'c'
    {0x38,0x44,0x44,0x48,0x7F}, // 'd'
    {0x38,0x54,0x54,0x54,0x18}, // 'e'
    {0x08,0x7E,0x09,0x01,0x02}, // 'f'
    {0x08,0x14,0x54,0x54,0x3C}, // 'g'
    {0x7F,0x08,0x04,0x04,0x78}, // 'h'
    {0x00,0x44,0x7D,0x40,0x00}, // 'i'
    {0x20,0x40,0x44,0x3D,0x00}, // 'j'
    {0x00,0x7F,0x10,0x28,0x44}, // 'k'
    {0x00,0x41,0x7F,0x40,0x00}, // 'l'
    {0x7C,0x04,0x18,0x04,0x78}, // 'm'
    {0x7C,0x08,0x04,0x04,0x78}, // 'n'
    {0x38,0x44,0x44,0x44,0x38}, // 'o'
    {0x7C,0x14,0x14,0x14,0x08}, // 'p'
    {0x08,0x14,0x14,0x18,0x7C}, // 'q'
    {0x7C,0x08,0x04,0x04,0x08}, // 'r'
    {0x48,0x54,0x54,0x54,0x20}, // 's'
    {0x04,0x3F,0x44,0x40,0x20}, // 't'
    {0x3C,0x40,0x40,0x20,0x7C}, // 'u'
    {0x1C,0x20,0x40,0x20,0x1C}, // 'v'
    {0x3C,0x40,0x30,0x40,0x3C}, // 'w'
    {0x44,0x28,0x10,0x28,0x44}, // 'x'
    {0x0C,0x50,0x50,0x50,0x3C}, // 'y'
    {0x44,0x64,0x54,0x4C,0x44}, // 'z'
    {0x00,0x08,0x36,0x41,0x00}, // '{'
    {0x00,0x00,0x7F,0x00,0x00}, // '|'
    {0x00,0x41,0x36,0x08,0x00}, // '}'
    {0x08,0x08,0x2A,0x1C,0x08}, // '~'
};

// ─── SSD1306 Commands ───────────────────────────────────────────────────────

static constexpr uint8_t SSD1306_SET_DISPLAY_OFF     = 0xAE;
static constexpr uint8_t SSD1306_SET_DISPLAY_ON      = 0xAF;
static constexpr uint8_t SSD1306_SET_DISP_CLK_DIV    = 0xD5;
static constexpr uint8_t SSD1306_SET_MULTIPLEX        = 0xA8;
static constexpr uint8_t SSD1306_SET_DISPLAY_OFFSET  = 0xD3;
static constexpr uint8_t SSD1306_SET_START_LINE       = 0x40;
static constexpr uint8_t SSD1306_SET_CHARGE_PUMP     = 0x8D;
static constexpr uint8_t SSD1306_SET_MEM_ADDR_MODE   = 0x20;
static constexpr uint8_t SSD1306_SET_SEG_REMAP        = 0xA1;
static constexpr uint8_t SSD1306_SET_COM_SCAN_DEC    = 0xC8;
static constexpr uint8_t SSD1306_SET_COM_PINS        = 0xDA;
static constexpr uint8_t SSD1306_SET_CONTRAST        = 0x81;
static constexpr uint8_t SSD1306_SET_PRECHARGE       = 0xD9;
static constexpr uint8_t SSD1306_SET_VCOM_DESEL      = 0xDB;
static constexpr uint8_t SSD1306_ENTIRE_DISPLAY_ON    = 0xA4;
static constexpr uint8_t SSD1306_SET_NORMAL_DISPLAY  = 0xA6;
static constexpr uint8_t SSD1306_SET_COLUMN_ADDR     = 0x21;
static constexpr uint8_t SSD1306_SET_PAGE_ADDR       = 0x22;

// I2C control bytes
static constexpr uint8_t SSD1306_CTRL_CMD   = 0x00;  // Co=0, D/C#=0 → command
static constexpr uint8_t SSD1306_CTRL_DATA  = 0x40;  // Co=0, D/C#=1 → data

// ─── I2C instance (same bus as GPU slave, different address) ────────────────

static i2c_inst_t* GetI2CInstance() {
    // Use the same I2C instance as the GPU slave (i2c1 on GP14/GP15)
    return (GpuConfig::I2C_INSTANCE == 1) ? i2c1 : i2c0;
}

// ─── Low-Level I2C Communication ────────────────────────────────────────────

bool I2cHudDriver::SendCommand(uint8_t cmd) {
    uint8_t buf[2] = { SSD1306_CTRL_CMD, cmd };
    int ret = i2c_write_blocking(GetI2CInstance(), i2cAddr_, buf, 2, false);
    return ret == 2;
}

bool I2cHudDriver::SendCommandByte(uint8_t cmd, uint8_t val) {
    uint8_t buf[3] = { SSD1306_CTRL_CMD, cmd, val };
    int ret = i2c_write_blocking(GetI2CInstance(), i2cAddr_, buf, 3, false);
    return ret == 3;
}

bool I2cHudDriver::SendData(const uint8_t* data, uint16_t len) {
    // Send in chunks with the data control byte prefix
    // Max I2C chunk: 128 + 1 = 129 bytes (1 page + control byte)
    uint8_t txBuf[HUD_WIDTH + 1];
    txBuf[0] = SSD1306_CTRL_DATA;

    while (len > 0) {
        uint16_t chunk = (len > HUD_WIDTH) ? HUD_WIDTH : len;
        memcpy(txBuf + 1, data, chunk);
        int ret = i2c_write_blocking(GetI2CInstance(), i2cAddr_, txBuf, chunk + 1, false);
        if (ret < 0) return false;
        data += chunk;
        len -= chunk;
    }
    return true;
}

// ─── Display Initialization ─────────────────────────────────────────────────

bool I2cHudDriver::InitDisplay() {
    // SSD1306 init sequence for 128×64
    const uint8_t initSeq[] = {
        SSD1306_SET_DISPLAY_OFF,
        SSD1306_SET_DISP_CLK_DIV, 0x80,     // Suggested ratio 0x80
        SSD1306_SET_MULTIPLEX, 63,            // 64 lines
        SSD1306_SET_DISPLAY_OFFSET, 0x00,
        SSD1306_SET_START_LINE,               // Start line 0
        SSD1306_SET_CHARGE_PUMP, 0x14,        // Enable charge pump (internal VCC)
        SSD1306_SET_MEM_ADDR_MODE, 0x00,      // Horizontal addressing mode
        SSD1306_SET_SEG_REMAP,                // Column 127 = SEG0
        SSD1306_SET_COM_SCAN_DEC,             // Scan from COM63 to COM0
        SSD1306_SET_COM_PINS, 0x12,           // Alternative COM pin config
        SSD1306_SET_CONTRAST, 0xCF,           // Contrast (0x00-0xFF)
        SSD1306_SET_PRECHARGE, 0xF1,          // Pre-charge: phase1=1, phase2=15
        SSD1306_SET_VCOM_DESEL, 0x40,         // VCOMH deselect level
        SSD1306_ENTIRE_DISPLAY_ON,            // Display RAM contents
        SSD1306_SET_NORMAL_DISPLAY,           // Normal (not inverted)
        SSD1306_SET_DISPLAY_ON                // Turn on
    };

    for (uint8_t cmd : initSeq) {
        if (!SendCommand(cmd)) {
            printf("[I2C_HUD] Init command 0x%02X failed\n", cmd);
            return false;
        }
    }

    return true;
}

// ─── DisplayDriver Interface ────────────────────────────────────────────────

bool I2cHudDriver::Init(const uint16_t* initialFramebuffer) {
    (void)initialFramebuffer;

    // Note: I2C bus (i2c1) is already initialized by the I2C slave driver.
    // We only initialize the display IC itself here.

    if (!InitDisplay()) {
        printf("[I2C_HUD] SSD1306 initialization failed at addr 0x%02X\n", i2cAddr_);
        return false;
    }

    // Clear the display
    Clear();

    // Send initial blank framebuffer
    // Set column address 0-127, page address 0-7
    SendCommand(SSD1306_SET_COLUMN_ADDR);
    SendCommand(0);
    SendCommand(HUD_WIDTH - 1);
    SendCommand(SSD1306_SET_PAGE_ADDR);
    SendCommand(0);
    SendCommand(HUD_PAGES - 1);
    SendData(hudBuffer_, HUD_BUFSIZE);

    initialized_ = true;
    currentPage_ = 0;

    printf("[I2C_HUD] Initialized: %ux%u mono OLED at I2C addr 0x%02X\n",
           HUD_WIDTH, HUD_HEIGHT, i2cAddr_);
    return true;
}

void I2cHudDriver::Shutdown() {
    if (!initialized_) return;
    SendCommand(SSD1306_SET_DISPLAY_OFF);
    initialized_ = false;
    printf("[I2C_HUD] Shutdown complete\n");
}

void I2cHudDriver::SetFramebuffer(const uint16_t* framebuffer) {
    if (!framebuffer) {
        customFbActive_ = false;
        return;
    }

    // Convert RGB565 → monochrome via luminance threshold
    // Y = (R*77 + G*150 + B*29) >> 8, threshold = 128
    customFbActive_ = true;
    memset(hudBuffer_, 0, HUD_BUFSIZE);

    for (uint16_t y = 0; y < HUD_HEIGHT; ++y) {
        for (uint16_t x = 0; x < HUD_WIDTH; ++x) {
            uint16_t px = framebuffer[y * HUD_WIDTH + x];
            uint8_t r = (px >> 8) & 0xF8;
            uint8_t g = (px >> 3) & 0xFC;
            uint8_t b = (px << 3) & 0xF8;
            uint8_t lum = (r * 77 + g * 150 + b * 29) >> 8;
            if (lum >= 128) {
                uint16_t page = y / 8;
                uint8_t bit = y & 7;
                hudBuffer_[page * HUD_WIDTH + x] |= (1 << bit);
            }
        }
    }
}

void I2cHudDriver::SetBrightness(uint8_t brightness) {
    brightness_ = brightness;
    if (initialized_) {
        SendCommandByte(SSD1306_SET_CONTRAST, brightness);
    }
}

void I2cHudDriver::PollRefresh() {
    if (!initialized_) return;

    // Send one page per call (128 bytes)
    // Set page address range to current page only
    SendCommand(SSD1306_SET_COLUMN_ADDR);
    SendCommand(0);
    SendCommand(HUD_WIDTH - 1);
    SendCommand(SSD1306_SET_PAGE_ADDR);
    SendCommand(currentPage_);
    SendCommand(currentPage_);

    SendData(&hudBuffer_[currentPage_ * HUD_WIDTH], HUD_WIDTH);

    currentPage_++;
    if (currentPage_ >= HUD_PAGES) {
        currentPage_ = 0;
        refreshCount_++;

        // Measure refresh rate every 8 full refreshes
        if ((refreshCount_ & 0x07) == 0) {
            uint32_t now = time_us_32();
            if (lastRefreshUs_ != 0) {
                uint32_t elapsed = now - lastRefreshUs_;
                measuredHz_ = 8000000u / elapsed;
            }
            lastRefreshUs_ = now;
        }
    }
}

DisplayTimingInfo I2cHudDriver::GetTimingInfo() const {
    DisplayTimingInfo info = {};
    info.refreshHz      = measuredHz_;
    info.lastFrameTimeUs = 0;
    info.totalRefreshes = refreshCount_;
    return info;
}

void I2cHudDriver::GetCaps(PglDisplayCaps& caps) const {
    caps.displayType   = PGL_DISPLAY_I2C_HUD;
    caps.width         = HUD_WIDTH;
    caps.height        = HUD_HEIGHT;
    caps.pixelFormat   = PGL_DISPLAY_FMT_MONO1;
    caps.maxBrightness = 255;
    caps.flags         = PGL_DISPLAY_FLAG_PARTIAL_UPDATE;
    caps.refreshHz     = static_cast<uint16_t>(measuredHz_);
    caps.framebufKB    = static_cast<uint16_t>((HUD_BUFSIZE + 1023) / 1024);  // 1 KB
    caps.pioUsage      = 0;  // No PIO — pure I2C
    caps.dmaUsage      = 0;  // No DMA
    caps.reserved[0]   = 0;
    caps.reserved[1]   = 0;
}

void I2cHudDriver::SetRegion(uint16_t x, uint16_t y, uint16_t w, uint16_t h) {
    regionX_ = (x < HUD_WIDTH)  ? x : 0;
    regionY_ = (y < HUD_HEIGHT) ? y : 0;
    regionW_ = (x + w <= HUD_WIDTH)  ? w : HUD_WIDTH  - regionX_;
    regionH_ = (y + h <= HUD_HEIGHT) ? h : HUD_HEIGHT - regionY_;
}

// ─── Drawing Primitives ─────────────────────────────────────────────────────

void I2cHudDriver::Clear() {
    memset(hudBuffer_, 0, HUD_BUFSIZE);
}

void I2cHudDriver::SetPixel(uint16_t x, uint16_t y, bool on) {
    if (x >= HUD_WIDTH || y >= HUD_HEIGHT) return;
    uint16_t page = y / 8;
    uint8_t bit = y & 7;
    uint16_t idx = page * HUD_WIDTH + x;
    if (on) {
        hudBuffer_[idx] |= (1 << bit);
    } else {
        hudBuffer_[idx] &= ~(1 << bit);
    }
}

void I2cHudDriver::DrawHLine(uint16_t x, uint16_t y, uint16_t w, bool on) {
    for (uint16_t i = 0; i < w && (x + i) < HUD_WIDTH; ++i) {
        SetPixel(x + i, y, on);
    }
}

void I2cHudDriver::DrawProgressBar(uint16_t x, uint16_t y, uint16_t w, uint16_t h,
                                    uint8_t percent) {
    if (percent > 100) percent = 100;

    // Border
    DrawHLine(x, y, w);
    DrawHLine(x, y + h - 1, w);
    for (uint16_t yy = y; yy < y + h; ++yy) {
        SetPixel(x, yy, true);
        SetPixel(x + w - 1, yy, true);
    }

    // Fill
    uint16_t fillW = (uint32_t)(w - 2) * percent / 100;
    for (uint16_t yy = y + 1; yy < y + h - 1; ++yy) {
        for (uint16_t xx = x + 1; xx < x + 1 + fillW; ++xx) {
            SetPixel(xx, yy, true);
        }
    }
}

void I2cHudDriver::DrawText(uint16_t x, uint16_t y, const char* str, bool invert) {
    while (*str && x + 5 <= HUD_WIDTH) {
        char ch = *str++;
        if (ch < 0x20 || ch > 0x7E) ch = '?';

        const uint8_t* glyph = FONT_5X7[ch - 0x20];

        for (uint8_t col = 0; col < 5; ++col) {
            uint8_t colData = glyph[col];
            if (invert) colData = ~colData;

            for (uint8_t row = 0; row < 7; ++row) {
                bool on = (colData >> row) & 1;
                if (invert) on = !on;
                SetPixel(x + col, y + row, on);
            }
        }

        // 1-pixel spacing between characters
        x += 6;
    }
}

// ─── Status Rendering ───────────────────────────────────────────────────────

/// Simple integer-to-string (max 6 digits + sign + null)
static void IntToStr(int val, char* buf, int bufLen) {
    if (bufLen < 2) { buf[0] = '\0'; return; }

    bool neg = val < 0;
    if (neg) val = -val;

    char tmp[12];
    int i = 0;
    do {
        tmp[i++] = '0' + (val % 10);
        val /= 10;
    } while (val > 0 && i < 10);

    int j = 0;
    if (neg && j < bufLen - 1) buf[j++] = '-';
    while (i > 0 && j < bufLen - 1) {
        buf[j++] = tmp[--i];
    }
    buf[j] = '\0';
}

void I2cHudDriver::RenderStatus(uint16_t fps, uint16_t vramUsedKB, uint16_t vramTotalKB,
                                 uint8_t cpuPercent, int8_t tempC, uint16_t frameTimeUs) {
    if (customFbActive_) return;  // Host has provided custom framebuffer

    Clear();

    char line[24];

    // Line 0: FPS
    char fpsStr[8];
    IntToStr(fps, fpsStr, sizeof(fpsStr));
    snprintf(line, sizeof(line), "FPS: %s", fpsStr);
    DrawText(0, 0, line);

    // Line 1: Frame time
    char ftStr[8];
    IntToStr(frameTimeUs, ftStr, sizeof(ftStr));
    snprintf(line, sizeof(line), "FT: %sus", ftStr);
    DrawText(0, 9, line);

    // Line 2: VRAM usage bar
    uint8_t vramPct = (vramTotalKB > 0)
                    ? static_cast<uint8_t>((uint32_t)vramUsedKB * 100 / vramTotalKB)
                    : 0;
    snprintf(line, sizeof(line), "VRAM:");
    DrawText(0, 18, line);
    DrawProgressBar(36, 18, 80, 7, vramPct);

    // Line 3: CPU usage bar
    snprintf(line, sizeof(line), "CPU:");
    DrawText(0, 27, line);
    DrawProgressBar(30, 27, 86, 7, cpuPercent);

    // Line 4: Temperature
    char tempStr[8];
    IntToStr(tempC, tempStr, sizeof(tempStr));
    snprintf(line, sizeof(line), "Temp: %sC", tempStr);
    DrawText(0, 36, line);

    // Line 5: VRAM KB
    char usedStr[8], totalStr[8];
    IntToStr(vramUsedKB, usedStr, sizeof(usedStr));
    IntToStr(vramTotalKB, totalStr, sizeof(totalStr));
    snprintf(line, sizeof(line), "%s/%sKB", usedStr, totalStr);
    DrawText(0, 45, line);

    // Separator line
    DrawHLine(0, 55, HUD_WIDTH);

    // Line 6: ProtoGL version tag
    DrawText(0, 57, "ProtoGL v0.7");
}
