#include "phase_scratch.h"
#include <atomic>

namespace PhaseScratch {
namespace {
alignas(std::max_align_t) unsigned char storage[CapacityBytes];
std::atomic_flag leased = ATOMIC_FLAG_INIT;
}

void* Acquire() {
    return leased.test_and_set(std::memory_order_acquire) ? nullptr : storage;
}

void Release() {
    leased.clear(std::memory_order_release);
}

} // namespace PhaseScratch
