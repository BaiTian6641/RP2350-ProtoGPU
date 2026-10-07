/**
 * @file pgl_scheduler_rp2350.cpp
 * @brief RP2350 device transport backend for PglTileScheduler — Pico SDK
 *        multicore SIO FIFO + SRAM-safe maintenance park.
 *
 * SDK ownership (authoritative for the whole firmware — see also the header):
 *   - This backend owns the inter-core SIO FIFO EXCLUSIVELY.  Every
 *     multicore_fifo_* call in the firmware lives here.  The SDK multicore
 *     lockout (multicore_lockout_start/blocking_end etc., used internally by
 *     flash_safe_execute) multiplexes the same FIFO and MUST NOT be used
 *     alongside this scheduler — use EnterMaintenance()/ExitMaintenance()
 *     around flash erase/program, QMI setup and clock/voltage transitions
 *     instead.
 *   - Core 1 launch is the parent's: main.cpp calls
 *     multicore_launch_core1(core1_entry) → GpuCore::Core1Main() →
 *     PglTileScheduler::Core1Main().  StartWorkerThread() below only
 *     confirms that arrangement (there is no SDK API to query the other
 *     core's entry point); the scheduler never launches cores itself.
 *   - The FIFO is used strictly as the tagged word stream defined in
 *     pgl_tile_scheduler.h; a FIFO token is a wakeup/control indication,
 *     never sufficient proof of context lifetime (the epoch + slot-index
 *     validation in the core provides that).
 *
 * SRAM safety: while parked, the worker executes ONLY
 * PglSchedParkWaitDevice() below, marked __no_inline_not_in_flash_func —
 * resident in SRAM, WFE-based, touching no XIP, no FIFO, no heap.  Flash/XIP may be
 * turned off and QMI/PLL/voltage reconfigured while the worker is inside
 * that loop; ExitMaintenance() releases it (SEV) only after restoration.
 *
 * Memory ordering: the SDK FIFO primitives do not emit barriers, so each
 * push/pop is bracketed with __dmb() and the scheduler core additionally
 * uses release/acquire atomics for descriptor publication (Cortex-M33
 * SRAM accesses are single-copy atomic; LDREX/STREX serialise the claim
 * counters).  FIFO writes are followed by __sev() inside the SDK inline
 * push, which wakes the peer's WFE.
 *
 * Do NOT compile this file into native/sim targets (it needs the Pico SDK).
 */

#include "pgl_tile_scheduler.h"

#include "pico/stdlib.h"
#include "pico/multicore.h"

namespace PglSchedPlatform {

void TransportInit(PglTileScheduler* /*self*/) {
    // Drop any words left by a previous session (e.g. after core-1 reset).
    multicore_fifo_drain();
}

void NoteControlThread(PglTileScheduler* /*self*/) {
    // Control identity is the physical core; nothing to record.
}

bool IsControlContext(PglTileScheduler* /*self*/) {
    return get_core_num() == 0;
}

void SendCommandWord(PglTileScheduler* /*self*/, uint32_t word) {
    __dmb();
    multicore_fifo_push_blocking(word);   // includes __sev()
}

uint32_t WaitCommandWord(PglTileScheduler* /*self*/) {
    const uint32_t w = multicore_fifo_pop_blocking();
    __dmb();
    return w;
}

void PushCompletionWord(PglTileScheduler* /*self*/, uint32_t word) {
    __dmb();
    multicore_fifo_push_blocking(word);
}

bool TryPopCompletionWord(PglTileScheduler* /*self*/, uint32_t* word) {
    // There is exactly one consumer on this core, so rvalid cannot become
    // false between this check and the read.  A partial record never waits
    // here for its second word and never reads an empty FIFO.
    if (!multicore_fifo_rvalid()) return false;
    *word = multicore_fifo_pop_blocking();
    __dmb();
    return true;
}

bool StartWorkerThread(PglTileScheduler* /*self*/) {
    // Core 1 was launched by the parent (multicore_launch_core1 → Core1Main)
    // before this call; the SDK offers no query, so this confirms intent.
    return true;
}

void JoinWorker(PglTileScheduler* /*self*/) {
    // A physical core cannot be joined; Shutdown() parks it instead.
}

bool WorkerJoinable() { return false; }

void ParkWake(PglTileScheduler* /*self*/) {
    __sev();   // wake the parked worker's WFE
}

}  // namespace PglSchedPlatform

/// SRAM-resident park wait — the ONLY code the worker executes while the
/// scheduler is in maintenance.  Must stay free of flash/XIP references:
/// no calls into flash-resident functions, no literal-pool surprises beyond
/// immediate constants (verified at integration via the map file).
static void __no_inline_not_in_flash_func(PglSchedParkWaitDevice)(
        std::atomic<uint32_t>* flag, uint32_t waitValue,
        std::atomic<uint32_t>* parked) {
    __atomic_store_n(reinterpret_cast<volatile uint32_t*>(parked),
                     1u, __ATOMIC_RELEASE);
    while (__atomic_load_n(reinterpret_cast<volatile uint32_t*>(flag),
                           __ATOMIC_ACQUIRE) == waitValue) {
        __wfe();
    }
    __dmb();
    __atomic_store_n(reinterpret_cast<volatile uint32_t*>(parked),
                     0u, __ATOMIC_RELEASE);
}

namespace PglSchedPlatform {

void ParkWait(PglTileScheduler* /*self*/, std::atomic<uint32_t>* flag,
              uint32_t waitValue, std::atomic<uint32_t>* parked) {
    PglSchedParkWaitDevice(flag, waitValue, parked);
}

}  // namespace PglSchedPlatform
