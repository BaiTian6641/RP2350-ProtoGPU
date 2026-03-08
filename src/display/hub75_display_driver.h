/**
 * @file hub75_display_driver.h
 * @brief Hub75DisplayDriver — DisplayDriver class wrapper for the HUB75 namespace (M11).
 *
 * Wraps the existing Hub75Driver namespace (PIO + DMA + BCM) in the
 * DisplayDriver abstract base class.  The namespace functions remain the
 * working implementation; this class delegates to them.
 *
 * Slot 0 in DisplayManager.  PIO0 SM0 (data) + SM1 (row), 1 DMA channel.
 */

#pragma once

#include "display_driver.h"
#include "hub75_driver.h"
#include "../gpu_config.h"

class Hub75DisplayDriver final : public DisplayDriver {
public:
    Hub75DisplayDriver() = default;

    // ── DisplayDriver interface ─────────────────────────────────────────

    uint8_t GetDisplayType() const override {
        return PGL_DISPLAY_HUB75;
    }

    const char* GetName() const override {
        return "HUB75";
    }

    bool Init(const uint16_t* initialFramebuffer) override {
        return Hub75Driver::Initialize(initialFramebuffer);
    }

    void Shutdown() override {
        Hub75Driver::Shutdown();
    }

    void SetFramebuffer(const uint16_t* framebuffer) override {
        Hub75Driver::SetFramebuffer(framebuffer);
    }

    void SetBrightness(uint8_t brightness) override {
        Hub75Driver::SetBrightness(brightness);
    }

    void PollRefresh() override {
        Hub75Driver::PollRefresh();
    }

    uint16_t GetWidth() const override {
        return GpuConfig::PANEL_WIDTH;
    }

    uint16_t GetHeight() const override {
        return GpuConfig::PANEL_HEIGHT;
    }

    uint8_t GetPixelFormat() const override {
        return PGL_DISPLAY_FMT_RGB565;
    }

    DisplayTimingInfo GetTimingInfo() const override {
        DisplayTimingInfo info = {};
        info.refreshHz       = Hub75Driver::GetRefreshRate();
        info.lastFrameTimeUs = 0;  // Not tracked at BCM level
        info.totalRefreshes  = 0;  // Can be added if needed
        return info;
    }

    void GetCaps(PglDisplayCaps& caps) const override {
        caps.displayType   = PGL_DISPLAY_HUB75;
        caps.width         = GpuConfig::PANEL_WIDTH;     // 128
        caps.height        = GpuConfig::PANEL_HEIGHT;    // 64
        caps.pixelFormat   = PGL_DISPLAY_FMT_RGB565;
        caps.maxBrightness = 255;
        caps.flags         = 0;  // No partial update, no vsync callback
        caps.refreshHz     = static_cast<uint16_t>(Hub75Driver::GetRefreshRate());
        caps.framebufKB    = static_cast<uint16_t>(
                              (GpuConfig::PANEL_WIDTH * GpuConfig::PANEL_HEIGHT * 2) / 1024);
        caps.pioUsage      = 2;  // SM0 (data) + SM1 (row)
        caps.dmaUsage      = 1;  // 1 DMA channel
        caps.reserved[0]   = 0;
        caps.reserved[1]   = 0;
    }

    void FillTestPattern(uint16_t* fb, uint8_t pattern) override {
        Hub75Driver::FillTestPattern(fb, pattern);
    }

    // SetRegion(): uses default no-op (HUB75 always full-panel refresh)
};
