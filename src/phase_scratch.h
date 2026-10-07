#pragma once

#include "gpu_config.h"
#include <PglTypes.h>
#include <cstddef>
#include <cstdint>
#include <new>
#include <type_traits>

namespace PhaseScratch {

// Parse and camera preparation run on core 0 at mutually exclusive drained
// boundaries. Tile workers and output readers never retain these objects.
// Pointer-width scaling keeps native fixtures bounded without imposing the
// native metadata ABI on the 32-bit firmware's 13 KiB reservation.
constexpr size_t CapacityBytes = 13 * 1024 * sizeof(void*) / sizeof(uint32_t);
void* Acquire();
void Release();

template <typename T>
class Lease {
public:
    Lease() {
        static_assert(sizeof(T) <= CapacityBytes, "phase scratch capacity exceeded");
        static_assert(alignof(T) <= alignof(std::max_align_t), "phase scratch alignment exceeded");
        static_assert(std::is_trivially_destructible<T>::value, "scratch must not own persistent objects");
        void* memory = Acquire();
        // Default-initialize, not value-initialize: only occupied entries are
        // written by the consumer; transform buffers are never blanket-cleared.
        if (memory) value_ = ::new (memory) T;
    }
    ~Lease() {
        if (value_) {
            value_->~T();
            Release();
        }
    }
    Lease(const Lease&) = delete;
    Lease& operator=(const Lease&) = delete;
    explicit operator bool() const { return value_ != nullptr; }
    T& operator*() const { return *value_; }
    T* operator->() const { return value_; }

private:
    T* value_ = nullptr;
};

struct ViewVertices {
    PglVec3 values[GpuConfig::MAX_VERTICES];
};

// World vertices are consumed only while preparing projected triangles. The
// same storage then becomes the target-strided depth plane (also used as the
// retired-depth post-FX workspace). Every switch explicitly starts the new
// C++17 object lifetime; no cast from a caller's uint16_t array is permitted.
class DepthWorkspace {
public:
    DepthWorkspace() { BeginDepth(); }
    DepthWorkspace(const DepthWorkspace&) = delete;
    DepthWorkspace& operator=(const DepthWorkspace&) = delete;

    PglVec3* BeginWorld() {
        return (::new (static_cast<void*>(&storage_.world)) WorldVertices)->values;
    }
    uint16_t* BeginDepth() {
        return (::new (static_cast<void*>(&storage_.depth)) DepthPlane)->values;
    }
    // Valid only after BeginDepth(), until the next BeginWorld(). Callers must
    // obtain a fresh pointer after preparation instead of retaining a pointer
    // across a union-member lifetime switch.
    uint16_t* DepthPixels() { return storage_.depth.values; }
    const uint16_t* DepthPixels() const { return storage_.depth.values; }

private:
    struct WorldVertices { PglVec3 values[GpuConfig::MAX_VERTICES]; };
    struct DepthPlane { uint16_t values[GpuConfig::FRAMEBUF_PIXELS]; };
    union alignas(4) Storage {
        WorldVertices world;
        DepthPlane depth;
        Storage() {}
    } storage_;
};

static_assert(sizeof(ViewVertices) <= CapacityBytes, "view vertices must fit phase storage");
static_assert(sizeof(DepthWorkspace) == GpuConfig::FRAMEBUF_SIZE,
              "world vertices must fit the existing depth reservation");

} // namespace PhaseScratch
