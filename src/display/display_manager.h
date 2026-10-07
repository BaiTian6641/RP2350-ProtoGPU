#pragma once
#include "display_driver.h"
#include "../gpu_config.h"

class DisplayManager {
public:
    explicit DisplayManager(HardwareResources& resources) : resources_(resources) {}
    PglRuntime::Result Configure(DisplayConfig config);
    PglRuntime::Result Present(const DisplaySurface&, const DisplayMapping&);
    void Poll();
    bool PopEvent(DisplayEvent&);
    PglRuntime::Result Quiesce();
    PglRuntime::Result Resume(uint32_t systemClockHz);
    PglRuntime::Result SetBrightness(uint8_t brightness);
    void Shutdown();
    DisplayCapabilities GetCaps() const;
    const DisplayConfig& Config() const { return config_; }
    bool ReadingSource() const { return readingSource_; }
    static PglRuntime::Result Validate(DisplayConfig&);
private:
    static DisplayDriver* Find(DisplayType);
    HardwareResources& resources_;
    DisplayConfig config_;
    DisplayDriver* driver_ = nullptr;
    bool initialized_ = false, readingSource_ = false;
    bool noOutputEvent_ = false;
    DisplayEvent renderEvent_;
    alignas(4) uint8_t workspace_[GpuConfig::DISPLAY_WORKSPACE_BYTES] = {};
};
