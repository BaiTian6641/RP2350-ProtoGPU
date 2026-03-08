/**
 * @file display_driver.h
 * @brief Abstract base class for all GPU display outputs — M11 Display Abstraction.
 *
 * Every display output (HUB75 LED panel, I2C HUD OLED, SPI LCD, DVI-D)
 * implements this interface.  The DisplayManager routes commands to the
 * active driver(s) through this ABC.
 *
 * Design rationale:
 *   - Virtual dispatch cost is negligible vs. PIO/DMA display refresh times.
 *   - Keeps gpu_core.cpp display-agnostic (swappable at runtime via commands).
 *   - PglDisplayCaps provides host with introspection per display slot.
 *   - PollRefresh() is the cooperative multitasking hook (HUB75 BCM, OLED
 *     scanline, etc.) — called from the Core 0 main loop.
 *
 * Lifecycle: Init() → [SetFramebuffer() / PollRefresh() / ...] → Shutdown()
 */

#pragma once

#include <cstdint>
#include <PglTypes.h>

// ─── Display Timing Info ────────────────────────────────────────────────────

struct DisplayTimingInfo {
    uint32_t refreshHz;         ///< Measured display refresh rate in Hz
    uint32_t lastFrameTimeUs;   ///< Time of last full refresh cycle in microseconds
    uint32_t totalRefreshes;    ///< Cumulative refresh cycle count
};

// ─── Display Driver Abstract Base Class ─────────────────────────────────────

class DisplayDriver {
public:
    virtual ~DisplayDriver() = default;

    // ── Identification ──────────────────────────────────────────────────

    /// Return the PglDisplayType enum value for this driver.
    virtual uint8_t GetDisplayType() const = 0;

    /// Return the human-readable driver name (for debug logging).
    virtual const char* GetName() const = 0;

    // ── Lifecycle ───────────────────────────────────────────────────────

    /**
     * @brief Initialize the display hardware.
     *
     * Sets up PIO state machines, DMA channels, I2C peripherals, or SPI
     * bus as needed.  Called once during gpu_core Initialize().
     *
     * @param initialFramebuffer  Pointer to the initial RGB565 framebuffer
     *                            (may be nullptr if driver allocates its own).
     * @return true on success, false on hardware failure.
     */
    virtual bool Init(const uint16_t* initialFramebuffer) = 0;

    /**
     * @brief Shut down the display hardware.
     *
     * Stops PIO/DMA, releases GPIO, blanks the display.
     * Safe to call multiple times.
     */
    virtual void Shutdown() = 0;

    // ── Framebuffer Management ──────────────────────────────────────────

    /**
     * @brief Atomically update the framebuffer pointer for display refresh.
     *
     * @param framebuffer  Pointer to the new front buffer (RGB565).
     */
    virtual void SetFramebuffer(const uint16_t* framebuffer) = 0;

    /**
     * @brief Set display brightness.
     *
     * @param brightness  0–255 (0 = off, 255 = full brightness).
     */
    virtual void SetBrightness(uint8_t brightness) = 0;

    // ── Cooperative Refresh ─────────────────────────────────────────────

    /**
     * @brief Drive one incremental refresh step (non-blocking).
     *
     * For HUB75: advances one BCM plane / row pair.
     * For I2C OLED: sends one scanline batch.
     * For SPI LCD: issues one partial-update DMA.
     *
     * Call this frequently from the Core 0 main loop.
     */
    virtual void PollRefresh() = 0;

    // ── Display Info ────────────────────────────────────────────────────

    /**
     * @brief Get the native display width in pixels.
     */
    virtual uint16_t GetWidth() const = 0;

    /**
     * @brief Get the native display height in pixels.
     */
    virtual uint16_t GetHeight() const = 0;

    /**
     * @brief Get the native pixel format.
     */
    virtual uint8_t GetPixelFormat() const = 0;

    /**
     * @brief Get display timing information.
     */
    virtual DisplayTimingInfo GetTimingInfo() const = 0;

    /**
     * @brief Fill a PglDisplayCaps structure describing this driver's capabilities.
     *
     * Used by the host to query what a display slot can do
     * (via PGL_REG_DISPLAY_CAPS I2C register).
     *
     * @param[out] caps  Capabilities structure to fill.
     */
    virtual void GetCaps(PglDisplayCaps& caps) const = 0;

    // ── Optional: Partial Update Region ─────────────────────────────────

    /**
     * @brief Set a partial update region (for OLED/LCD displays).
     *
     * Not all drivers support this — the default implementation is a no-op.
     * HUB75 always refreshes the full panel.
     *
     * @param x  Left edge of region.
     * @param y  Top edge of region.
     * @param w  Width of region.
     * @param h  Height of region.
     */
    virtual void SetRegion(uint16_t x, uint16_t y, uint16_t w, uint16_t h) {
        (void)x; (void)y; (void)w; (void)h;
        // Default: no-op (full-panel refresh drivers ignore this)
    }

    // ── Optional: Test Patterns ─────────────────────────────────────────

    /**
     * @brief Fill a framebuffer with a test pattern (for bringup).
     *
     * @param fb       Pointer to RGB565 framebuffer.
     * @param pattern  0 = solid red, 1 = RGB gradient, 2 = checkerboard, 3 = color bars.
     */
    virtual void FillTestPattern(uint16_t* fb, uint8_t pattern) {
        (void)fb; (void)pattern;
        // Default: no-op
    }
};
