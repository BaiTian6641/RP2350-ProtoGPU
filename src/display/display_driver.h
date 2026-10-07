#pragma once

#include <cstddef>
#include <cstdint>
#include <PglTypes.h>
#include <PglRuntimeProtocol.h>
#include "../hardware_resources.h"

// The workspace belongs to DisplayManager and is shared by alternative
// backends. Reconfiguration drains the old reader before the new backend
// receives it. RGB565 input remains borrowed until a sourceReleased event.
enum class DisplayType : uint8_t { None = 0, Hub75 = 1, SpiSsd1331 = 3, LedV5 = 6, CustomRgb888 = 7 };

struct DisplayConfig {
    DisplayType type = DisplayType::SpiSsd1331;
    uint16_t width = 96, height = 64;
    uint8_t brightness = 255;
    uint8_t layoutId = 0;
    bool flipH = false, flipV = false;
};

struct DisplaySurface {
    const uint16_t* pixels = nullptr;
    size_t pixelCapacity = 0;
    uint16_t width = 0, height = 0, stride = 0;
    uint32_t frame = 0;
};

struct DisplayMapping {
    const PglVec2* coordinates = nullptr;
    uint16_t count = 0;
    bool reversed = false;
    bool rectangular = false;
    PglRectLayoutData rectangle = {};
};

struct DisplayEvent {
    uint32_t frame = 0;
    PglRuntime::Completion completion = PglRuntime::Completion::Unknown;
    PglRuntime::Result result = PglRuntime::Result::Ok;
    uint64_t timestampUs = 0;
    bool sourceReleased = false;
};

struct DisplayCapabilities {
    DisplayType type = DisplayType::None;
    uint16_t width = 0, height = 0;
    bool displayCompletion = false;
    uint8_t pioStateMachines = 0, dmaChannels = 0;
    uint16_t pioInstructions = 0;
    uint32_t workspaceBytes = 0;
};

class DisplayDriver {
public:
    virtual ~DisplayDriver() = default;
    virtual PglRuntime::Result Init(HardwareResources& resources, const DisplayConfig& config,
                                    void* workspace, size_t bytes) = 0;
    virtual PglRuntime::Result Present(const DisplaySurface& surface, const DisplayMapping& mapping) = 0;
    virtual void PollRefresh() = 0;
    virtual bool PopEvent(DisplayEvent& event) = 0;
    virtual PglRuntime::Result Quiesce() = 0;
    virtual PglRuntime::Result Resume(uint32_t systemClockHz) = 0;
    virtual PglRuntime::Result SetBrightness(uint8_t brightness) = 0;
    virtual DisplayCapabilities GetCaps() const = 0;
    virtual void Shutdown() = 0;
};

#if defined(PICO_ON_DEVICE)
DisplayDriver& Hub75Backend();
DisplayDriver& SpiDisplayBackend();
DisplayDriver& LedArrayBackend();
DisplayDriver& CustomArrayBackend();
#endif
