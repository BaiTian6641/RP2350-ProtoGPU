/**
 * @file ssd1331_driver.cpp
 * @brief SSD1331 96×64 RGB OLED — hardware SPI0 driver implementation.
 *
 * Uses RP2350 hardware SPI0 with blocking writes for command sequences
 * and DMA-backed bulk pixel transfer for frame updates. The display is
 * configured for:
 *   - 65K colour mode (RGB565, 16-bit per pixel)
 *   - Horizontal address increment (left-to-right, top-to-bottom)
 *   - Normal display orientation (no remap/flip)
 *
 * Only compiled when RP2350GPU_HEADLESS_SELFTEST is defined.
 */

#ifdef RP2350GPU_HEADLESS_SELFTEST

#include "ssd1331_driver.h"
#include "../gpu_config.h"

#include "pico/stdlib.h"
#include "hardware/dma.h"
#include "hardware/spi.h"
#include "hardware/gpio.h"

#include <cstdio>
#include <cstring>

// ─── Hardware Instance ──────────────────────────────────────────────────────

static spi_inst_t* const SPI_PORT = spi0;

// SPI TX DMA state (claimed during Initialize)
static int s_spiTxDmaChannel = -1;
static dma_channel_config s_spiTxDmaConfig;

// Full-frame RGB565 byte-swapped TX buffer for SSD1331 (96x64x2 = 12288 bytes)
static constexpr uint32_t FRAME_PIXELS =
    GpuConfig::SSD1331_WIDTH * GpuConfig::SSD1331_HEIGHT;
static constexpr uint32_t FRAME_BYTES = FRAME_PIXELS * 2;
static uint8_t s_frameTxBuf[FRAME_BYTES];

// ─── Low-Level Helpers ──────────────────────────────────────────────────────

static inline void CsLow()  { gpio_put(GpuConfig::SSD1331_CS_PIN, 0); }
static inline void CsHigh() { gpio_put(GpuConfig::SSD1331_CS_PIN, 1); }
static inline void DcCmd()  { gpio_put(GpuConfig::SSD1331_DC_PIN, 0); }
static inline void DcData() { gpio_put(GpuConfig::SSD1331_DC_PIN, 1); }

/// Send a single command byte (DC=low).
static void SendCmd(uint8_t cmd) {
    DcCmd();
    CsLow();
    spi_write_blocking(SPI_PORT, &cmd, 1);
    CsHigh();
}

/// Send a command byte followed by one or more argument bytes.
static void SendCmdArgs(uint8_t cmd, const uint8_t* args, uint8_t argLen) {
    DcCmd();
    CsLow();
    spi_write_blocking(SPI_PORT, &cmd, 1);
    // Arguments are still sent in command mode for SSD1331
    spi_write_blocking(SPI_PORT, args, argLen);
    CsHigh();
}

/// Send raw data bytes (DC=high).
static void SendData(const uint8_t* data, uint32_t len) {
    DcData();
    CsLow();
    spi_write_blocking(SPI_PORT, data, len);
    CsHigh();
}

/// Send raw data bytes (DC=high) using SPI TX DMA.
static void SendDataDma(const uint8_t* data, uint32_t len) {
    DcData();
    CsLow();

    dma_channel_configure(
        s_spiTxDmaChannel,
        &s_spiTxDmaConfig,
        &spi_get_hw(SPI_PORT)->dr,  // write address (SPI data register)
        data,                       // read address
        len,                        // transfer count (bytes)
        true);                      // start immediately

    dma_channel_wait_for_finish_blocking(s_spiTxDmaChannel);
    while (spi_is_busy(SPI_PORT)) {
        tight_loop_contents();
    }

    CsHigh();
}

// ─── SSD1331 Command Definitions ────────────────────────────────────────────

// Drawing commands
static constexpr uint8_t SSD1331_CMD_DRAWLINE       = 0x21;
static constexpr uint8_t SSD1331_CMD_DRAWRECT       = 0x22;
static constexpr uint8_t SSD1331_CMD_CLEAR          = 0x25;
static constexpr uint8_t SSD1331_CMD_FILL           = 0x26;
static constexpr uint8_t SSD1331_CMD_SCROLLSETUP    = 0x27;
static constexpr uint8_t SSD1331_CMD_SCROLLSTOP     = 0x2E;
static constexpr uint8_t SSD1331_CMD_SCROLLSTART    = 0x2F;

// Addressing
static constexpr uint8_t SSD1331_CMD_SETCOLUMN      = 0x15;
static constexpr uint8_t SSD1331_CMD_SETROW         = 0x75;

// Hardware config
static constexpr uint8_t SSD1331_CMD_SETREMAP       = 0xA0;
static constexpr uint8_t SSD1331_CMD_STARTLINE      = 0xA1;
static constexpr uint8_t SSD1331_CMD_DISPLAYOFFSET  = 0xA2;
static constexpr uint8_t SSD1331_CMD_NORMALDISPLAY  = 0xA4;
static constexpr uint8_t SSD1331_CMD_DISPLAYALLON   = 0xA5;
static constexpr uint8_t SSD1331_CMD_DISPLAYALLOFF  = 0xA6;
static constexpr uint8_t SSD1331_CMD_INVERTDISPLAY  = 0xA7;
static constexpr uint8_t SSD1331_CMD_SETMULTIPLEX   = 0xA8;
static constexpr uint8_t SSD1331_CMD_SETMASTER      = 0xAD;
static constexpr uint8_t SSD1331_CMD_DISPLAYOFF     = 0xAE;
static constexpr uint8_t SSD1331_CMD_DISPLAYON      = 0xAF;
static constexpr uint8_t SSD1331_CMD_POWERMODE      = 0xB0;
static constexpr uint8_t SSD1331_CMD_PRECHARGE      = 0xB1;
static constexpr uint8_t SSD1331_CMD_CLOCKDIV       = 0xB3;
static constexpr uint8_t SSD1331_CMD_PRECHARGEA     = 0x8A;
static constexpr uint8_t SSD1331_CMD_PRECHARGEB     = 0x8B;
static constexpr uint8_t SSD1331_CMD_PRECHARGEC     = 0x8C;
static constexpr uint8_t SSD1331_CMD_PRECHARGELEVEL  = 0xBB;
static constexpr uint8_t SSD1331_CMD_VCOMH           = 0xBE;
static constexpr uint8_t SSD1331_CMD_MASTERCURRENT   = 0x87;
static constexpr uint8_t SSD1331_CMD_CONTRASTA       = 0x81;
static constexpr uint8_t SSD1331_CMD_CONTRASTB       = 0x82;
static constexpr uint8_t SSD1331_CMD_CONTRASTC       = 0x83;

// ─── Initialization ─────────────────────────────────────────────────────────

bool Ssd1331::Initialize() {
    // ── Configure GPIO pins ──
    // CS — manual chip select
    gpio_init(GpuConfig::SSD1331_CS_PIN);
    gpio_set_dir(GpuConfig::SSD1331_CS_PIN, GPIO_OUT);
    CsHigh();

    // DC — data/command select
    gpio_init(GpuConfig::SSD1331_DC_PIN);
    gpio_set_dir(GpuConfig::SSD1331_DC_PIN, GPIO_OUT);
    DcData();

    // RST — hardware reset (active low)
    gpio_init(GpuConfig::SSD1331_RST_PIN);
    gpio_set_dir(GpuConfig::SSD1331_RST_PIN, GPIO_OUT);

    // ── Hardware reset pulse ──
    gpio_put(GpuConfig::SSD1331_RST_PIN, 1);
    sleep_ms(10);
    gpio_put(GpuConfig::SSD1331_RST_PIN, 0);
    sleep_ms(50);
    gpio_put(GpuConfig::SSD1331_RST_PIN, 1);
    sleep_ms(50);

    // ── Initialize SPI0 ──
    spi_init(SPI_PORT, GpuConfig::SSD1331_SPI_BAUD);
    gpio_set_function(GpuConfig::SSD1331_SCK_PIN, GPIO_FUNC_SPI);
    gpio_set_function(GpuConfig::SSD1331_MOSI_PIN, GPIO_FUNC_SPI);
    // We don't use MISO (no readback)

    // ── Configure DMA channel for SPI0 TX ──
    s_spiTxDmaChannel = dma_claim_unused_channel(false);
    if (s_spiTxDmaChannel < 0) {
        printf("[SSD1331] ERROR: no DMA channel available for SPI TX\n");
        return false;
    }

    s_spiTxDmaConfig = dma_channel_get_default_config(s_spiTxDmaChannel);
    channel_config_set_transfer_data_size(&s_spiTxDmaConfig, DMA_SIZE_8);
    channel_config_set_dreq(&s_spiTxDmaConfig, spi_get_dreq(SPI_PORT, true));
    channel_config_set_read_increment(&s_spiTxDmaConfig, true);
    channel_config_set_write_increment(&s_spiTxDmaConfig, false);

    printf("[SSD1331] SPI0 initialized at %lu Hz\n",
           (unsigned long)spi_get_baudrate(SPI_PORT));

    // ── SSD1331 initialization sequence ──
    // Display off during config
    SendCmd(SSD1331_CMD_DISPLAYOFF);

    // Set remap & colour depth: 65K colour (RGB565), horizontal increment
    // Bit 0 = 1: column address 0 mapped to SEG0 (normal)
    // Bit 1 = 0: RAM column 0-95
    // Bit 2 = 1: colour order C→B→A (RGB)
    // Bit 4 = 0: scan from COM0 to COM[N-1]
    // Bit 5 = 1: enable COM split odd/even
    // Bit 6 = 1: 65K colour depth (RGB565)
    {
        uint8_t args[] = { 0x72 };  // 0b01110010
        SendCmdArgs(SSD1331_CMD_SETREMAP, args, 1);
    }

    // Start line = 0
    { uint8_t args[] = { 0x00 }; SendCmdArgs(SSD1331_CMD_STARTLINE, args, 1); }

    // Display offset = 0
    { uint8_t args[] = { 0x00 }; SendCmdArgs(SSD1331_CMD_DISPLAYOFFSET, args, 1); }

    // Normal display (not all-on, not all-off, not inverted)
    SendCmd(SSD1331_CMD_NORMALDISPLAY);

    // Multiplex ratio = 63 (64 lines, 0-indexed)
    { uint8_t args[] = { 0x3F }; SendCmdArgs(SSD1331_CMD_SETMULTIPLEX, args, 1); }

    // Master config: external Vcc supply
    { uint8_t args[] = { 0x8E }; SendCmdArgs(SSD1331_CMD_SETMASTER, args, 1); }

    // Power save mode: disabled
    { uint8_t args[] = { 0x0B }; SendCmdArgs(SSD1331_CMD_POWERMODE, args, 1); }

    // Phase 1/2 period: phase1=1 DCLK, phase2=15 DCLKs
    { uint8_t args[] = { 0x31 }; SendCmdArgs(SSD1331_CMD_PRECHARGE, args, 1); }

    // Clock divider / oscillator frequency
    // Bits 3:0 = divider (0 = /1), Bits 7:4 = osc freq (0xF = max)
    { uint8_t args[] = { 0xF0 }; SendCmdArgs(SSD1331_CMD_CLOCKDIV, args, 1); }

    // Precharge speed for colour channels
    { uint8_t args[] = { 0x64 }; SendCmdArgs(SSD1331_CMD_PRECHARGEA, args, 1); }
    { uint8_t args[] = { 0x78 }; SendCmdArgs(SSD1331_CMD_PRECHARGEB, args, 1); }
    { uint8_t args[] = { 0x64 }; SendCmdArgs(SSD1331_CMD_PRECHARGEC, args, 1); }

    // Pre-charge level: ~0.5 × Vcc
    { uint8_t args[] = { 0x3A }; SendCmdArgs(SSD1331_CMD_PRECHARGELEVEL, args, 1); }

    // VCOMH deselect level: ~0.83 × Vcc
    { uint8_t args[] = { 0x3E }; SendCmdArgs(SSD1331_CMD_VCOMH, args, 1); }

    // Master current attenuation: full
    { uint8_t args[] = { 0x06 }; SendCmdArgs(SSD1331_CMD_MASTERCURRENT, args, 1); }

    // Contrast for channels A, B, C
    { uint8_t args[] = { 0x91 }; SendCmdArgs(SSD1331_CMD_CONTRASTA, args, 1); }
    { uint8_t args[] = { 0x50 }; SendCmdArgs(SSD1331_CMD_CONTRASTB, args, 1); }
    { uint8_t args[] = { 0x7D }; SendCmdArgs(SSD1331_CMD_CONTRASTC, args, 1); }

    // Disable scrolling
    SendCmd(SSD1331_CMD_SCROLLSTOP);

    // Turn on the OLED panel
    SendCmd(SSD1331_CMD_DISPLAYON);

        printf("[SSD1331] Display initialized (%ux%u, 65K colour, SPI TX DMA ch%d)\n",
            GpuConfig::SSD1331_WIDTH, GpuConfig::SSD1331_HEIGHT,
            s_spiTxDmaChannel);
    return true;
}

// ─── Framebuffer Push ───────────────────────────────────────────────────────

void Ssd1331::PushFramebuffer(const uint16_t* fb) {
    // Set column range: 0 → 95
    {
        uint8_t args[] = { 0x00, static_cast<uint8_t>(GpuConfig::SSD1331_WIDTH - 1) };
        SendCmdArgs(SSD1331_CMD_SETCOLUMN, args, 2);
    }

    // Set row range: 0 → 63
    {
        uint8_t args[] = { 0x00, static_cast<uint8_t>(GpuConfig::SSD1331_HEIGHT - 1) };
        SendCmdArgs(SSD1331_CMD_SETROW, args, 2);
    }

    // SSD1331 expects big-endian RGB565 (MSB first).
    // RP2350 is little-endian, so we byte-swap into a dedicated TX buffer,
    // then push the full frame in one DMA transfer.
    for (uint32_t i = 0; i < FRAME_PIXELS; ++i) {
        uint16_t px = fb[i];
        s_frameTxBuf[i * 2 + 0] = static_cast<uint8_t>(px >> 8);    // MSB
        s_frameTxBuf[i * 2 + 1] = static_cast<uint8_t>(px & 0xFF);  // LSB
    }

    SendDataDma(s_frameTxBuf, FRAME_BYTES);
}

// ─── Display Power ──────────────────────────────────────────────────────────

void Ssd1331::DisplayOff() {
    SendCmd(SSD1331_CMD_DISPLAYOFF);
}

void Ssd1331::DisplayOn() {
    SendCmd(SSD1331_CMD_DISPLAYON);
}

// ─── Contrast ───────────────────────────────────────────────────────────────

void Ssd1331::SetContrast(uint8_t r, uint8_t g, uint8_t b) {
    { uint8_t args[] = { r }; SendCmdArgs(SSD1331_CMD_CONTRASTA, args, 1); }
    { uint8_t args[] = { g }; SendCmdArgs(SSD1331_CMD_CONTRASTB, args, 1); }
    { uint8_t args[] = { b }; SendCmdArgs(SSD1331_CMD_CONTRASTC, args, 1); }
}

#endif  // RP2350GPU_HEADLESS_SELFTEST
