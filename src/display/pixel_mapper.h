#pragma once
#include "display_driver.h"
#include <cstring>

inline bool ValidDisplaySurface(const DisplaySurface& surface) {
    if (!surface.pixels || !surface.width || !surface.height || surface.stride < surface.width) return false;
    return size_t(surface.height - 1) * surface.stride + surface.width <= surface.pixelCapacity;
}
inline bool FiniteMappingCoordinate(float value) {
    uint32_t bits; std::memcpy(&bits, &value, sizeof(bits));
    return (bits & 0x7f800000u) != 0x7f800000u;
}

// Validate once per borrowed surface, not once per pixel. Rectangle step sizes
// are likewise computed once. The descriptor and coordinate span stay immutable
// until the backend reports sourceReleased.
class DisplayPixelSampler {
public:
    DisplayPixelSampler(const DisplaySurface& surface, const DisplayMapping& mapping, const DisplayConfig& config)
        : surface_(surface), mapping_(mapping), config_(config), total_(uint32_t(config.width) * config.height) {
        valid_ = ValidDisplaySurface(surface) && config.width && config.height;
        if (mapping.coordinates && !mapping.count) valid_ = false;
        if (mapping.rectangular) {
            const auto& r = mapping.rectangle;
            valid_ = valid_ && r.colCount && r.rowCount && FiniteMappingCoordinate(r.size.x) &&
                FiniteMappingCoordinate(r.size.y) && FiniteMappingCoordinate(r.position.x) && FiniteMappingCoordinate(r.position.y);
            if (valid_) { stepX_ = r.size.x / r.colCount; stepY_ = r.size.y / r.rowCount; }
        }
        if (valid_ && mapping.coordinates) {
            for (uint16_t i = 0; i < mapping.count; ++i) {
                if (!FiniteMappingCoordinate(mapping.coordinates[i].x) || !FiniteMappingCoordinate(mapping.coordinates[i].y)) { valid_ = false; break; }
            }
        }
    }
    bool Valid() const { return valid_; }
    uint16_t Get(uint32_t physicalIndex) const {
        if (!valid_ || physicalIndex >= total_) return 0;
        uint32_t index = mapping_.reversed ? total_ - 1 - physicalIndex : physicalIndex;
        if (mapping_.count && index >= mapping_.count) return 0;
        float x, y;
        if (mapping_.coordinates) {
            x = mapping_.coordinates[index].x; y = mapping_.coordinates[index].y;
        } else if (mapping_.rectangular) {
            const auto& r = mapping_.rectangle;
            if (index >= uint32_t(r.colCount) * r.rowCount) return 0;
            x = r.position.x + (float(index % r.colCount) + 0.5f) * stepX_;
            y = r.position.y + (float(index / r.colCount) + 0.5f) * stepY_;
        } else { x = float(index % config_.width); y = float(index / config_.width); }
        if (x < 0 || y < 0 || x >= surface_.width || y >= surface_.height) return 0;
        uint16_t sx = static_cast<uint16_t>(x), sy = static_cast<uint16_t>(y);
        if (config_.flipH) sx = surface_.width - 1 - sx;
        if (config_.flipV) sy = surface_.height - 1 - sy;
        return surface_.pixels[size_t(sy) * surface_.stride + sx];
    }
private:
    const DisplaySurface& surface_;
    const DisplayMapping& mapping_;
    const DisplayConfig& config_;
    uint32_t total_;
    bool valid_ = false;
    float stepX_ = 0, stepY_ = 0;
};

inline uint16_t ScaleDisplayBrightness(uint16_t color, uint8_t brightness) {
    if (brightness == 255) return color;
    const uint32_t r = (((color >> 11) & 31u) * brightness + 127u) / 255u;
    const uint32_t g = (((color >> 5) & 63u) * brightness + 127u) / 255u;
    const uint32_t b = ((color & 31u) * brightness + 127u) / 255u;
    return static_cast<uint16_t>((r << 11) | (g << 5) | b);
}
