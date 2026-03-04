/**
 * @file pgl_job_scheduler_rp2350.h
 * @brief RP2350 dual-core job scheduler using multicore FIFO.
 *
 * Implements PglJobScheduler by dispatching one job to Core 1 via the RP2350
 * hardware FIFO and executing the remaining jobs on the calling core (Core 0).
 *
 * Design:
 *  - Core 1 runs a permanent spin loop waiting on FIFO for job pointers.
 *  - Submit(jobs, N) sends jobs[N-1] to Core 1, then executes jobs[0..N-2]
 *    on Core 0 inline (work-stealing pattern).
 *  - WaitAll() polls multicore_fifo_rvalid() while calling the idle function
 *    (typically Hub75Driver::PollRefresh to keep the display alive).
 *
 * See docs/Shader_Backend_And_Scheduler_Design.md §3.3.1.
 */

#pragma once

#include <PglJobScheduler.h>
#include <cstdint>

class PglJobScheduler_RP2350 : public PglJobScheduler {
public:
    /// Initialise the scheduler.  Must be called on Core 0 before Core 1 is
    /// launched.  After this, call Core1Main() on Core 1.
    void Initialize();

    /// Core 1 entry point — permanent job-processing loop (never returns).
    /// Must be called exactly once, on Core 1.
    void Core1Main();

    // ── PglJobScheduler interface ───────────────────────────────────────

    uint8_t WorkerCount() const override { return 1; }

    void Submit(const PglJob* jobs, uint8_t count) override;

    void WaitAll(void (*idleFunc)() = nullptr) override;

private:
    /// Number of jobs dispatched to Core 1 that haven't been acknowledged yet.
    volatile uint8_t pendingOnCore1_ = 0;
};
