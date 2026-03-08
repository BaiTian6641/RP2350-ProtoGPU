/**
 * @file display_manager.cpp
 * @brief DisplayManager implementation — M11 display abstraction layer.
 */

#include "display_manager.h"
#include <cstdio>
#include <cstring>

// ─── Initialization ─────────────────────────────────────────────────────────

void DisplayManager::Init() {
    for (uint8_t i = 0; i < MAX_SLOTS; ++i) {
        slots_[i].driver     = nullptr;
        slots_[i].enabled    = false;
        slots_[i].brightness = 255;
    }
    selectedSlot = 0;
    printf("[DisplayManager] Initialized (%u slots)\n", MAX_SLOTS);
}

// ─── Driver Registration ────────────────────────────────────────────────────

bool DisplayManager::RegisterDriver(uint8_t slot, DisplayDriver* driver) {
    if (slot >= MAX_SLOTS || !driver) return false;

    if (slots_[slot].driver == driver) return true;  // Already registered

    if (slots_[slot].driver != nullptr) {
        printf("[DisplayManager] WARNING: slot %u already occupied by %s, replacing\n",
               slot, slots_[slot].driver->GetName());
        if (slots_[slot].enabled) {
            slots_[slot].driver->Shutdown();
        }
    }

    slots_[slot].driver  = driver;
    slots_[slot].enabled = false;  // Must be Init()'d separately
    printf("[DisplayManager] Registered %s in slot %u\n", driver->GetName(), slot);
    return true;
}

void DisplayManager::UnregisterDriver(uint8_t slot, bool shutdown) {
    if (slot >= MAX_SLOTS) return;

    Slot& s = slots_[slot];
    if (s.driver) {
        if (shutdown && s.enabled) {
            s.driver->Shutdown();
        }
        printf("[DisplayManager] Unregistered %s from slot %u\n",
               s.driver->GetName(), slot);
        s.driver  = nullptr;
        s.enabled = false;
    }
}

// ─── Runtime Configuration ──────────────────────────────────────────────────

bool DisplayManager::ConfigureDisplay(const PglCmdDisplayConfigure& cmd) {
    uint8_t slot = cmd.displayId;
    if (slot >= MAX_SLOTS) {
        printf("[DisplayManager] ConfigureDisplay: invalid slot %u\n", slot);
        return false;
    }

    Slot& s = slots_[slot];

    // PGL_DISPLAY_NONE → shut down
    if (cmd.displayType == PGL_DISPLAY_NONE) {
        if (s.driver && s.enabled) {
            s.driver->Shutdown();
            s.enabled = false;
            printf("[DisplayManager] Slot %u disabled\n", slot);
        }
        return true;
    }

    // Must have a registered driver
    if (!s.driver) {
        printf("[DisplayManager] ConfigureDisplay: no driver in slot %u\n", slot);
        return false;
    }

    // Verify the display type matches
    if (s.driver->GetDisplayType() != cmd.displayType) {
        printf("[DisplayManager] ConfigureDisplay: type mismatch — slot %u has %s (type 0x%02X)"
               " but requested 0x%02X\n",
               slot, s.driver->GetName(), s.driver->GetDisplayType(), cmd.displayType);
        return false;
    }

    // Apply brightness
    if (cmd.brightness > 0) {
        s.brightness = cmd.brightness;
        s.driver->SetBrightness(cmd.brightness);
    }

    // Mark as configured/enabled (driver was already Init()'d at boot)
    s.enabled = true;

    printf("[DisplayManager] Slot %u configured: %s, %ux%u, brightness=%u\n",
           slot, s.driver->GetName(),
           cmd.width ? cmd.width : s.driver->GetWidth(),
           cmd.height ? cmd.height : s.driver->GetHeight(),
           s.brightness);

    return true;
}

bool DisplayManager::SetRegion(const PglCmdDisplaySetRegion& cmd) {
    uint8_t slot = cmd.displayId;
    if (slot >= MAX_SLOTS) return false;

    Slot& s = slots_[slot];
    if (!s.driver || !s.enabled) return false;

    s.driver->SetRegion(cmd.x, cmd.y, cmd.w, cmd.h);
    return true;
}

// ─── Framebuffer Routing ────────────────────────────────────────────────────

void DisplayManager::SetPrimaryFramebuffer(const uint16_t* framebuffer) {
    if (slots_[0].driver && slots_[0].enabled) {
        slots_[0].driver->SetFramebuffer(framebuffer);
    }
}

void DisplayManager::PollAllDisplays() {
    for (uint8_t i = 0; i < MAX_SLOTS; ++i) {
        if (slots_[i].driver && slots_[i].enabled) {
            slots_[i].driver->PollRefresh();
        }
    }
}

// ─── Query / I2C Register Support ───────────────────────────────────────────

DisplayDriver* DisplayManager::GetDriver(uint8_t slot) const {
    if (slot >= MAX_SLOTS) return nullptr;
    return slots_[slot].driver;
}

uint8_t DisplayManager::GetDisplayType(uint8_t slot) const {
    if (slot >= MAX_SLOTS || !slots_[slot].driver) {
        return PGL_DISPLAY_NONE;
    }
    return slots_[slot].driver->GetDisplayType();
}

bool DisplayManager::GetCaps(uint8_t slot, PglDisplayCaps& caps) const {
    if (slot >= MAX_SLOTS || !slots_[slot].driver) {
        std::memset(&caps, 0, sizeof(caps));
        caps.displayType = PGL_DISPLAY_NONE;
        return false;
    }
    slots_[slot].driver->GetCaps(caps);
    return true;
}

uint8_t DisplayManager::GetActiveCount() const {
    uint8_t count = 0;
    for (uint8_t i = 0; i < MAX_SLOTS; ++i) {
        if (slots_[i].driver && slots_[i].enabled) ++count;
    }
    return count;
}
