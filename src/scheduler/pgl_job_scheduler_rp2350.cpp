/**
 * @file pgl_job_scheduler_rp2350.cpp
 * @brief RP2350 dual-core job scheduler implementation.
 *
 * Uses the RP2350 multicore hardware FIFO (8-deep, 32-bit queue between cores)
 * for zero-overhead inter-core job dispatch.
 *
 * Protocol:
 *   Core 0 → Core 1:  push (uint32_t)(PglJob*)   — job pointer to execute
 *   Core 1 → Core 0:  push FIFO_JOB_DONE         — job completed
 *
 * Special sentinel:
 *   FIFO_JOB_NONE (0) is never sent — used as null marker.
 */

#include "pgl_job_scheduler_rp2350.h"

#include "pico/stdlib.h"
#include "pico/multicore.h"

#include <cstdio>

// ─── FIFO Protocol Constants ────────────────────────────────────────────────

static constexpr uint32_t FIFO_JOB_DONE = 0x444F4E45u;  // "DONE"

// ─── Initialize ─────────────────────────────────────────────────────────────

void PglJobScheduler_RP2350::Initialize() {
    pendingOnCore1_ = 0;
    printf("[Scheduler] RP2350 dual-core scheduler initialised\n");
}

// ─── Submit ─────────────────────────────────────────────────────────────────

void PglJobScheduler_RP2350::Submit(const PglJob* jobs, uint8_t count) {
    if (count == 0) return;

    // Dispatch the LAST job to Core 1 (if more than 1 job)
    if (count >= 2) {
        // Send job pointer to Core 1 via FIFO
        const PglJob* core1Job = &jobs[count - 1];
        multicore_fifo_push_blocking(reinterpret_cast<uint32_t>(core1Job));
        pendingOnCore1_ = 1;

        // Execute all other jobs on Core 0 (inline)
        for (uint8_t i = 0; i < count - 1; ++i) {
            if (jobs[i].func) {
                jobs[i].func(jobs[i].ctx);
            }
        }
    } else {
        // Only 1 job — run it on Core 0 directly
        pendingOnCore1_ = 0;
        if (jobs[0].func) {
            jobs[0].func(jobs[0].ctx);
        }
    }
}

// ─── WaitAll ────────────────────────────────────────────────────────────────

void PglJobScheduler_RP2350::WaitAll(void (*idleFunc)()) {
    if (pendingOnCore1_ == 0) return;

    // Poll FIFO for Core 1's completion signal, calling idleFunc while waiting
    while (!multicore_fifo_rvalid()) {
        if (idleFunc) idleFunc();
    }

    uint32_t response = multicore_fifo_pop_blocking();
    (void)response;  // Expected: FIFO_JOB_DONE

    pendingOnCore1_ = 0;
}

// ─── Core 1 Main Loop ──────────────────────────────────────────────────────

void PglJobScheduler_RP2350::Core1Main() {
    printf("[Scheduler] Core 1 entering job loop\n");

    while (true) {
        // Wait for a job pointer from Core 0
        uint32_t raw = multicore_fifo_pop_blocking();

        // Interpret as PglJob pointer
        const PglJob* job = reinterpret_cast<const PglJob*>(raw);
        if (job && job->func) {
            job->func(job->ctx);
        }

        // Signal completion
        multicore_fifo_push_blocking(FIFO_JOB_DONE);
    }
}
