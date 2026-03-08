/**
 * @file display_manager.h
 * @brief DisplayManager — singleton registry and routing for display drivers (M11).
 *
 * Manages up to PGL_MAX_DISPLAYS (4) display slots. Each slot can hold one
 * DisplayDriver instance. The primary display (slot 0) receives the rasterized
 * framebuffer; secondary displays (HUD OLED, etc.) can run independently.
 *
 * Host interaction:
 *   CMD_DISPLAY_CONFIGURE (0x90) → ConfigureDisplay()
 *   CMD_DISPLAY_SET_REGION (0x91) → SetRegion()
 *   PGL_REG_DISPLAY_MODE   (I2C) → GetActiveDisplayType() / SelectDisplay()
 *   PGL_REG_DISPLAY_CAPS   (I2C) → GetCaps()
 *
 * PIO resource tracking ensures no two drivers claim the same PIO/SM.
 */

#pragma once

#include <cstdint>
#include <PglTypes.h>
#include "display_driver.h"

class DisplayManager {
public:
    /// Maximum number of display slots.
    static constexpr uint8_t MAX_SLOTS = PGL_MAX_DISPLAYS;  // 4

    DisplayManager() = default;

    // ── Initialization ──────────────────────────────────────────────────

    /**
     * @brief Initialize the display manager (call once during gpu_core init).
     *
     * Clears all slots. Does NOT initialize any drivers — that happens via
     * RegisterDriver() or ConfigureDisplay().
     */
    void Init();

    // ── Driver Registration (boot-time) ─────────────────────────────────

    /**
     * @brief Register a display driver in a specific slot.
     *
     * Does NOT call Init() on the driver. Caller is responsible for
     * initialization order (HUB75 before I2C HUD, etc.).
     *
     * @param slot    Display slot index (0 – MAX_SLOTS-1).
     * @param driver  Pointer to a DisplayDriver instance (caller owns lifetime).
     * @return true if slot was available (or already held the same driver).
     */
    bool RegisterDriver(uint8_t slot, DisplayDriver* driver);

    /**
     * @brief Unregister and optionally shut down a driver.
     *
     * @param slot      Display slot index.
     * @param shutdown  If true, calls driver->Shutdown() before clearing.
     */
    void UnregisterDriver(uint8_t slot, bool shutdown = true);

    // ── Runtime Configuration (from host commands) ──────────────────────

    /**
     * @brief Configure a display slot from a CMD_DISPLAY_CONFIGURE payload.
     *
     * If the display type matches the registered driver, applies brightness
     * and flags. If the type is PGL_DISPLAY_NONE, shuts down the slot.
     *
     * @param cmd  Wire-format command payload.
     * @return true on success.
     */
    bool ConfigureDisplay(const PglCmdDisplayConfigure& cmd);

    /**
     * @brief Set partial-update region for a display slot.
     *
     * @param cmd  Wire-format command payload.
     * @return true if the slot has an active driver.
     */
    bool SetRegion(const PglCmdDisplaySetRegion& cmd);

    // ── Framebuffer Routing ─────────────────────────────────────────────

    /**
     * @brief Set the framebuffer on the primary display (slot 0).
     *
     * Called by gpu_core after frame swap.
     */
    void SetPrimaryFramebuffer(const uint16_t* framebuffer);

    /**
     * @brief Poll all active displays for cooperative refresh.
     *
     * Called from the Core 0 main loop. Iterates all slots and calls
     * PollRefresh() on each active driver.
     */
    void PollAllDisplays();

    // ── Query / I2C Register Support ────────────────────────────────────

    /**
     * @brief Get the driver in a specific slot (nullptr if empty).
     */
    DisplayDriver* GetDriver(uint8_t slot) const;

    /**
     * @brief Get the active display type (PglDisplayType) for a slot.
     */
    uint8_t GetDisplayType(uint8_t slot) const;

    /**
     * @brief Fill a PglDisplayCaps for I2C readback.
     *
     * @param slot  Display slot index.
     * @param[out] caps  Capabilities to fill.
     * @return true if the slot has a driver.
     */
    bool GetCaps(uint8_t slot, PglDisplayCaps& caps) const;

    /**
     * @brief Get total number of registered (active) displays.
     */
    uint8_t GetActiveCount() const;

    /**
     * @brief The currently selected display slot for I2C register queries.
     *        Written by host via PGL_REG_DISPLAY_MODE.
     */
    uint8_t selectedSlot = 0;

private:
    struct Slot {
        DisplayDriver* driver    = nullptr;
        bool           enabled   = false;    ///< Configured and Init()'d
        uint8_t        brightness = 255;
    };

    Slot slots_[MAX_SLOTS] = {};
};
