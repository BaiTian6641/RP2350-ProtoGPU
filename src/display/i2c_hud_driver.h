/**
 * @file i2c_hud_driver.h
 * @brief I2C HUD OLED display driver — SSD1306/SSD1309 128×64 monochrome (M11).
 *
 * Drives a secondary status overlay on a small I2C OLED display. The HUD
 * renders GPU diagnostic info (FPS, VRAM usage, temperature, CPU load)
 * as text using a built-in 5×7 bitmap font.
 *
 * Hardware:
 *   - SSD1306 or SSD1309, 128×64 pixels, 1-bit (monochrome)
 *   - I2C1 @ address 0x3D (configurable — distinct from GPU slave @ 0x3C)
 *   - No PIO consumed (pure I2C), no DMA channels
 *
 * Refresh model:
 *   The HUD framebuffer is a 128×64 / 8 = 1024-byte monochrome buffer.
 *   PollRefresh() sends one page (128 bytes) per call = 8 calls for full
 *   refresh.  At I2C 400 kHz this is ~2.5 ms per page ≈ 50 Hz full update.
 *
 *   The HUD auto-renders status text each frame unless the host provides
 *   a custom framebuffer via SetFramebuffer() (the RGB565 input is converted
 *   to 1-bit internally via luminance threshold).
 *
 * DisplayManager slot: typically slot 1 (slot 0 = HUB75).
 */

#pragma once

#include "display_driver.h"
#include "../gpu_config.h"

#include <cstdint>

class I2cHudDriver final : public DisplayDriver {
public:
    // Display dimensions
    static constexpr uint16_t HUD_WIDTH  = 128;
    static constexpr uint16_t HUD_HEIGHT = 64;
    static constexpr uint16_t HUD_PAGES  = HUD_HEIGHT / 8;  // 8 pages
    static constexpr uint16_t HUD_BUFSIZE = HUD_WIDTH * HUD_PAGES;  // 1024 bytes

    // I2C configuration
    static constexpr uint8_t  DEFAULT_I2C_ADDR = 0x3D;
    static constexpr uint32_t I2C_BAUD_HZ      = 400000;  // 400 kHz Fast Mode

    I2cHudDriver() = default;

    // ── DisplayDriver interface ─────────────────────────────────────────

    uint8_t GetDisplayType() const override { return PGL_DISPLAY_I2C_HUD; }
    const char* GetName() const override { return "I2C_HUD"; }

    bool Init(const uint16_t* initialFramebuffer) override;
    void Shutdown() override;

    void SetFramebuffer(const uint16_t* framebuffer) override;
    void SetBrightness(uint8_t brightness) override;

    void PollRefresh() override;

    uint16_t GetWidth() const override { return HUD_WIDTH; }
    uint16_t GetHeight() const override { return HUD_HEIGHT; }
    uint8_t GetPixelFormat() const override { return PGL_DISPLAY_FMT_MONO1; }

    DisplayTimingInfo GetTimingInfo() const override;
    void GetCaps(PglDisplayCaps& caps) const override;

    void SetRegion(uint16_t x, uint16_t y, uint16_t w, uint16_t h) override;

    // ── HUD-Specific API ────────────────────────────────────────────────

    /**
     * @brief Render GPU status text into the HUD framebuffer.
     *
     * Called by gpu_core each frame (or on demand). Clears the buffer
     * and prints: FPS, VRAM usage %, CPU load %, temperature, frame time.
     *
     * @param fps         Current frames per second.
     * @param vramUsedKB  VRAM used in KB.
     * @param vramTotalKB VRAM total in KB.
     * @param cpuPercent  CPU usage 0–100.
     * @param tempC       Die temperature in °C.
     * @param frameTimeUs Frame time in microseconds.
     */
    void RenderStatus(uint16_t fps, uint16_t vramUsedKB, uint16_t vramTotalKB,
                      uint8_t cpuPercent, int8_t tempC, uint16_t frameTimeUs);

    /**
     * @brief Draw a text string at (x, y) in the monochrome buffer.
     *
     * Uses the built-in 5×7 bitmap font. Characters are drawn left-to-right
     * with 1-pixel spacing (6 pixels per character cell).
     *
     * @param x     Column (0-based pixel).
     * @param y     Row (0-based pixel, top of character).
     * @param str   Null-terminated ASCII string.
     * @param invert  If true, draw black text on white background.
     */
    void DrawText(uint16_t x, uint16_t y, const char* str, bool invert = false);

    /**
     * @brief Clear the HUD framebuffer to black.
     */
    void Clear();

    /**
     * @brief Set a single pixel in the monochrome buffer.
     */
    void SetPixel(uint16_t x, uint16_t y, bool on);

    /**
     * @brief Draw a horizontal line.
     */
    void DrawHLine(uint16_t x, uint16_t y, uint16_t w, bool on = true);

    /**
     * @brief Draw a progress bar.
     */
    void DrawProgressBar(uint16_t x, uint16_t y, uint16_t w, uint16_t h,
                         uint8_t percent);

    /// Get direct access to the monochrome framebuffer (1024 bytes).
    uint8_t* GetBuffer() { return hudBuffer_; }
    const uint8_t* GetBuffer() const { return hudBuffer_; }

private:
    // Monochrome framebuffer (page-organized: byte = 8 vertical pixels)
    uint8_t hudBuffer_[HUD_BUFSIZE] = {};

    // I2C state
    uint8_t i2cAddr_      = DEFAULT_I2C_ADDR;
    bool    initialized_  = false;
    uint8_t brightness_   = 0xFF;

    // Cooperative refresh state: sends one page per PollRefresh() call
    uint8_t currentPage_  = 0;
    uint32_t refreshCount_ = 0;
    uint32_t lastRefreshUs_ = 0;
    uint32_t measuredHz_   = 0;

    // Partial update region
    uint16_t regionX_ = 0;
    uint16_t regionY_ = 0;
    uint16_t regionW_ = HUD_WIDTH;
    uint16_t regionH_ = HUD_HEIGHT;

    // Whether host has provided a custom framebuffer
    bool customFbActive_ = false;

    // ── Internal SSD1306 commands ───────────────────────────────────────

    bool SendCommand(uint8_t cmd);
    bool SendCommandByte(uint8_t cmd, uint8_t val);
    bool SendData(const uint8_t* data, uint16_t len);
    bool InitDisplay();
};
