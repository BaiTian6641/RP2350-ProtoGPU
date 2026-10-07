/**
 * @file pgl_tile_scheduler.cpp
 * @brief Bounded dual-core scheduler — platform-neutral core (P04).
 *
 * Implements the tagged command/epoch protocol, fixed job storage,
 * exactly-once tile claiming, completion routing and the maintenance
 * rendezvous described in pgl_tile_scheduler.h.  This translation unit is
 * deliberately free of Pico SDK / std::thread dependencies: the word-stream
 * transport and worker lifecycle live in the platform backends
 * (pgl_scheduler_rp2350.cpp / pgl_scheduler_native.cpp), so the SAME
 * protocol executes on device (SIO FIFO) and natively (fixed rings).
 *
 * The only renderer dependency is the RasterizeTile() adapter used by the
 * Rasterizer* DispatchTilePass() overload; DispatchTilePassCustom() keeps
 * the scheduler core renderer-agnostic.
 */

#include "pgl_tile_scheduler.h"

#include "../render/rasterizer.h"

#include <cstdio>

// ─── Wire protocol constants ────────────────────────────────────────────────

namespace PglSchedProto {
    /// Command tags: [31:28] kind, [27:0] epoch.
    static constexpr uint32_t KIND_JOB  = 0x10000000u;
    static constexpr uint32_t KIND_PARK = 0x20000000u;
    static constexpr uint32_t KIND_QUIT = 0x30000000u;
    static constexpr uint32_t KIND_MASK = 0xF0000000u;
    static constexpr uint32_t EPOCH_MASK = 0x0FFFFFFFu;

    /// Completion word 0: [31:28] magic, [27:0] epoch.  Word 1: result code.
    static constexpr uint32_t CPL_MAGIC = 0xC0000000u;

    /// Protocol-level result codes (completion word 1).
    static constexpr int32_t RSLT_OK             = 0;
    static constexpr int32_t RSLT_CANCELLED      = 1;
    static constexpr int32_t RSLT_PROTOCOL_FAULT = 2;
    static constexpr int32_t RSLT_PARKED         = 3;
}

using namespace PglSchedProto;

// ─── Active instance (PairDispatch routing) ─────────────────────────────────

PglTileScheduler* PglTileScheduler::activeInstance_ = nullptr;

// ─── Initialize ─────────────────────────────────────────────────────────────

void PglTileScheduler::Initialize() {
    if (workerRunning_) {
        // Live worker: re-initialisation would orphan in-flight commands.
        // Shutdown() first (documented in the header).
        return;
    }
    for (JobDesc& s : slots_) {
        s.state.store(ST_FREE, std::memory_order_relaxed);
        s.epoch       = 0;
        s.kind        = KIND_NONE;
        s.cplReceived = 0;
        s.cplResult   = 0;
        s.cancelRequested = false;
    }
    epochCounter_ = 1;
    completionTag_ = 0;
    completionTagPending_ = false;
    stats_        = PglSchedStats{};
    syncActive_   = false;
    parked_       = false;
    parkRelease_.store(0, std::memory_order_relaxed);
    workerParkedFlag_.store(0, std::memory_order_relaxed);

    activeInstance_ = this;
    PglSchedPlatform::TransportInit(this);
    PglSchedPlatform::NoteControlThread(this);
    initialized_ = true;

    printf("[TileScheduler] Initialised — tiles %ux%u px, <=%u cells/pass, %u job slots\n",
           TileConfig::TILE_W, TileConfig::TILE_H,
           TileConfig::MAX_TILES, JOB_SLOTS);
}

// ─── StartWorker / Shutdown ─────────────────────────────────────────────────

bool PglTileScheduler::StartWorker() {
    if (!initialized_) return false;
    if (workerRunning_) return true;
    if (!PglSchedPlatform::StartWorkerThread(this)) return false;
    workerRunning_ = true;
    return true;
}

void PglTileScheduler::Shutdown() {
    if (!workerRunning_) return;

    if (!PglSchedPlatform::WorkerJoinable()) {
        // Device: a core cannot be joined.  Park the worker permanently in
        // the SRAM-safe maintenance wait; the parent may then
        // multicore_reset_core1() and relaunch (see file header).
        (void)EnterMaintenance();
        return;
    }

    // Native: require a drained scheduler (jobs are finite by contract;
    // the caller collects them via WaitJob/PollJob before Shutdown).
    if (parked_) ExitMaintenance();
    if (syncActive_ || OutstandingJobs() != 0) return;

    const uint32_t epoch = NextEpoch() & EPOCH_MASK;
    PglSchedPlatform::SendCommandWord(this, KIND_QUIT | epoch);
    while (!WaitSpecificCompletion(epoch, RSLT_OK)) {
    }
    PglSchedPlatform::JoinWorker(this);   // joins the thread, resets channels
    completionTagPending_ = false;
    workerRunning_ = false;
}

// ─── Entry points ───────────────────────────────────────────────────────────

void PglTileScheduler::Core1Main() {
    RunWorkerLoop();   // returns only after QUIT (device: never sent)
}

void PglTileScheduler::WorkerThreadBody() {
    RunWorkerLoop();
}

// ─── Slot management ────────────────────────────────────────────────────────

PglTileScheduler::JobDesc* PglTileScheduler::AllocSlot() {
    // Opportunistically route arrived completions first: this releases the
    // slots of acknowledged cancels (and records async results) before the
    // bounded storage is scanned.
    PumpCompletions(nullptr, nullptr);
    for (uint8_t i = 0; i < JOB_SLOTS; ++i) {
        uint32_t expect = ST_FREE;
        if (slots_[i].state.compare_exchange_strong(
                expect, ST_FILLING,
                std::memory_order_acq_rel, std::memory_order_acquire)) {
            return &slots_[i];
        }
    }
    return nullptr;
}

void PglTileScheduler::FreeSlot(JobDesc* d) {
    d->cplReceived = 0;
    d->kind        = KIND_NONE;
    d->cancelRequested = false;
    d->state.store(ST_FREE, std::memory_order_release);
}

uint8_t PglTileScheduler::OutstandingJobs() const {
    uint8_t n = 0;
    for (uint8_t i = 0; i < JOB_SLOTS; ++i) {
        if (slots_[i].state.load(std::memory_order_acquire) != ST_FREE) ++n;
    }
    return n;
}

PglTileScheduler::JobDesc* PglTileScheduler::FindSlotByEpoch(uint32_t epoch28) {
    for (uint8_t i = 0; i < JOB_SLOTS; ++i) {
        const uint32_t st = slots_[i].state.load(std::memory_order_acquire);
        if (st != ST_FREE &&
            (slots_[i].epoch & EPOCH_MASK) == epoch28) {
            return &slots_[i];
        }
    }
    return nullptr;
}

// ─── Publication ────────────────────────────────────────────────────────────
//
// Control core: all descriptor fields are written while the slot is FILLING
// (visible only to the control core), then state -> QUEUED with release.
// The release-store synchronises-with the worker's acquire-load of the same
// atomic, so every field written before it is visible to the worker.  The
// command words follow (the transport itself carries a release fence on the
// device backend); the epoch tag + slot index let the worker re-validate.

void PglTileScheduler::PublishJob(JobDesc* d) {
    d->state.store(ST_QUEUED, std::memory_order_release);
    std::atomic_thread_fence(std::memory_order_release);
    PglSchedPlatform::SendCommandWord(this, KIND_JOB | (d->epoch & EPOCH_MASK));
    PglSchedPlatform::SendCommandWord(
        this, static_cast<uint32_t>(d - &slots_[0]));   // slot index
}

// Worker side: re-validate the slot index and epoch before touching the
// context — a stale/corrupt command can never resurrect a freed context.
PglTileScheduler::JobDesc* PglTileScheduler::ValidateJobPointer(uint32_t raw,
                                                                 uint32_t epoch) {
    if (raw >= JOB_SLOTS) return nullptr;
    JobDesc* d = &slots_[raw];
    const uint32_t st = d->state.load(std::memory_order_acquire);
    if (st != ST_QUEUED && st != ST_CANCELLING) return nullptr;
    if ((d->epoch & EPOCH_MASK) != epoch) return nullptr;
    return d;
}

// ─── Worker loop ────────────────────────────────────────────────────────────

void PglTileScheduler::RunWorkerLoop() {
    while (true) {
        const uint32_t tag   = PglSchedPlatform::WaitCommandWord(this);
        const uint32_t kind  = tag & KIND_MASK;
        const uint32_t epoch = tag & EPOCH_MASK;

        if (kind == KIND_QUIT) {
            PglSchedPlatform::PushCompletionWord(this, CPL_MAGIC | epoch);
            PglSchedPlatform::PushCompletionWord(this, RSLT_OK);
            return;
        }

        if (kind == KIND_PARK) {
            // Acknowledge the command, then enter the platform wait.  The
            // parked flag is published INSIDE the SRAM-resident function;
            // the control owner waits for both the record and that flag.
            PglSchedPlatform::PushCompletionWord(this, CPL_MAGIC | epoch);
            PglSchedPlatform::PushCompletionWord(this, RSLT_PARKED);
            PglSchedPlatform::ParkWait(this, &parkRelease_, 1u,
                                       &workerParkedFlag_);
            continue;
        }

        if (kind != KIND_JOB) {
            PglSchedPlatform::PushCompletionWord(this, CPL_MAGIC | epoch);
            PglSchedPlatform::PushCompletionWord(this, RSLT_PROTOCOL_FAULT);
            continue;
        }

        const uint32_t raw = PglSchedPlatform::WaitCommandWord(this);
        JobDesc* d = ValidateJobPointer(raw, epoch);
        if (!d) {
            PglSchedPlatform::PushCompletionWord(this, CPL_MAGIC | epoch);
            PglSchedPlatform::PushCompletionWord(this, RSLT_PROTOCOL_FAULT);
            continue;
        }

        // Claim the job.  Loses cleanly to a pre-start cancel.
        uint32_t expect = ST_QUEUED;
        if (!d->state.compare_exchange_strong(
                expect, ST_ACTIVE,
                std::memory_order_acq_rel, std::memory_order_acquire)) {
            const int32_t r = (expect == ST_CANCELLING)
                                  ? RSLT_CANCELLED : RSLT_PROTOCOL_FAULT;
            if (expect == ST_CANCELLING) {
                d->state.store(ST_DONE, std::memory_order_release);
            }
            PglSchedPlatform::PushCompletionWord(this, CPL_MAGIC | epoch);
            PglSchedPlatform::PushCompletionWord(this, r);
            continue;
        }

        ExecuteWorkerJob(d);

        // Release-store publishes all work (framebuffer writes, counters)
        // before the completion words are pushed.
        d->state.store(ST_DONE, std::memory_order_release);
        PglSchedPlatform::PushCompletionWord(this, CPL_MAGIC | epoch);
        PglSchedPlatform::PushCompletionWord(this, RSLT_OK);
    }
}

void PglTileScheduler::ExecuteWorkerJob(JobDesc* d) {
    if (d->kind == KIND_TILE_PASS) {
        ProcessTiles(d, /*isWorker=*/true, /*service=*/nullptr);
    } else if (d->funcB) {
        d->funcB(d->ctxB);
    }
}

// ─── Tile processing (both workers) ─────────────────────────────────────────
//
// Exactly-once claiming: the atomic fetch_add hands each tile index to
// exactly one core.  The service callback fires ONLY on the control core,
// between bounded tile claims (and the renderer calls it at bounded
// row/triangle slices inside the tile via the visitor's service argument).

void PglTileScheduler::ProcessTiles(JobDesc* d, bool isWorker,
                                    void (*service)()) {
    const uint32_t count = d->tileCount;
    while (true) {
        const uint32_t idx =
            d->nextTile.fetch_add(1, std::memory_order_relaxed);
        if (idx >= count) break;   // all tiles claimed

        const uint8_t  linearId = morton_[idx];
        const uint16_t tileCol  = linearId % d->gridCols;
        const uint16_t tileRow  = linearId / d->gridCols;

        d->visitor(d->ctx, tileCol, tileRow,
                   TileConfig::TILE_W, TileConfig::TILE_H,
                   isWorker ? nullptr : service);

        d->tilesDone.fetch_add(1, std::memory_order_relaxed);

        if (!isWorker && service) service();   // bounded service point
    }
}

// ─── Completion routing (control core) ──────────────────────────────────────

bool PglTileScheduler::ReadCompletion(uint32_t* epoch28, int32_t* result) {
    uint32_t word;
    while (PglSchedPlatform::TryPopCompletionWord(this, &word)) {
        if (!completionTagPending_) {
            completionTag_ = word;
            completionTagPending_ = true;
            continue;
        }

        const uint32_t tag = completionTag_;
        completionTagPending_ = false;
        // Even a malformed header occupies a whole two-word record.  Do
        // not reinterpret its payload (including zero) as the next header.
        if ((tag & KIND_MASK) != CPL_MAGIC || word > RSLT_PARKED) {
            ++stats_.protocolFaults;
            continue;
        }
        *epoch28 = tag & EPOCH_MASK;
        *result = static_cast<int32_t>(word);
        return true;
    }
    return false;
}

void PglTileScheduler::RouteCompletion(uint32_t epoch28, int32_t result) {
    // The worker reports command faults through the stream.  Diagnostics
    // remain exclusively control-owned, including while a job is running.
    const bool reportedFault = result == RSLT_PROTOCOL_FAULT;
    if (reportedFault) ++stats_.protocolFaults;
    JobDesc* owner = FindSlotByEpoch(epoch28);
    if (!owner) {
        ++stats_.staleCompletions;
        return;
    }
    // An epoch match alone is not proof that the worker has relinquished
    // this context.  Acquire DONE before accepting any terminal result.
    if (owner->state.load(std::memory_order_acquire) != ST_DONE ||
        owner->cplReceived || result == RSLT_PARKED ||
        (owner->cancelRequested != (result == RSLT_CANCELLED))) {
        if (!reportedFault) ++stats_.protocolFaults;
        return;
    }
    if (owner->cancelRequested) {
        // Only this slot's checked cancellation acknowledgement can free
        // it.  CancelJob's handle is already dead, even before this record.
        FreeSlot(owner);
        return;
    }
    owner->cplResult = result;
    owner->cplReceived = 1;
}

bool PglTileScheduler::PumpCompletions(JobDesc* target, int32_t* targetResult) {
    uint32_t epoch;
    int32_t result;
    while (ReadCompletion(&epoch, &result)) {
        RouteCompletion(epoch, result);
    }
    // A service callback/async allocation may already have routed the
    // target's record.  Completion is persistent until its owner collects it.
    const bool found = target && target->cplReceived;
    if (found && targetResult) *targetResult = target->cplResult;
    return found;
}

PglSchedResult PglTileScheduler::WaitCompletion(JobDesc* d, void (*service)()) {
    int32_t res = RSLT_PROTOCOL_FAULT;
    while (!PumpCompletions(d, &res)) {
        if (service) service();   // bounded service while the worker runs
    }
    return MapResult(d, res);
}

PglSchedResult PglTileScheduler::MapResult(JobDesc* d, int32_t protocolResult) {
    switch (protocolResult) {
        case RSLT_OK:
            // Checked completion: every claimed tile must be done.
            if (d->kind == KIND_TILE_PASS) {
                const uint32_t done = d->tilesDone.load(std::memory_order_acquire);
                if (done != d->tileCount) return PglSchedResult::Incomplete;
                stats_.tilesProcessed += done;
            }
            return PglSchedResult::Ok;
        case RSLT_CANCELLED:
            return PglSchedResult::Cancelled;
        default:
            return PglSchedResult::ProtocolFault;
    }
}

/// Lifecycle commands have no descriptor, but use the same partial-record
/// assembler and route unrelated records through the normal checked path.
bool PglTileScheduler::WaitSpecificCompletion(uint32_t epoch28,
                                               int32_t expectedResult) {
    uint32_t epoch;
    int32_t result;
    while (ReadCompletion(&epoch, &result)) {
        if (epoch == epoch28) {
            if (result == expectedResult) return true;
            ++stats_.protocolFaults;
            continue;
        }
        RouteCompletion(epoch, result);
    }
    return false;
}

// ─── Dispatch: tile passes ──────────────────────────────────────────────────

PglSchedResult PglTileScheduler::DispatchTilePassImpl(
        void* ctx, PglTileVisitor visitor, const PglRasterTileCtx* adapter,
        uint16_t w, uint16_t h, void (*service)()) {
    if (!initialized_ || !workerRunning_) return PglSchedResult::NotInitialized;
    if (!PglSchedPlatform::IsControlContext(this)) return PglSchedResult::WrongCore;
    if (!visitor || !TileConfig::ExtentValid(w, h)) return PglSchedResult::Rejected;
    if (syncActive_ || parked_) return PglSchedResult::Busy;

    JobDesc* d = AllocSlot();
    if (!d) {
        ++stats_.queueFull;
        return PglSchedResult::QueueFull;
    }
    syncActive_ = true;   // nonreentrancy guard (incl. service callbacks)

    const uint16_t cols = TileConfig::ColsFor(w);
    const uint16_t rows = TileConfig::RowsFor(h);
    const uint16_t count = TileConfig::BuildMortonOrder(
        cols, rows, morton_, TileConfig::MAX_TILES);

    d->epoch     = NextEpoch();
    d->kind      = KIND_TILE_PASS;
    d->gridCols  = cols;
    d->tileCount = count;
    d->nextTile.store(0, std::memory_order_relaxed);
    d->tilesDone.store(0, std::memory_order_relaxed);
    d->cplReceived = 0;
    if (adapter) {
        d->adapter = *adapter;                 // fixed storage copy
        d->ctx     = &d->adapter;
    } else {
        d->ctx     = ctx;
    }
    d->visitor = visitor;

    PublishJob(d);

    // Control core joins the same exactly-once claim loop.
    ProcessTiles(d, /*isWorker=*/false, service);

    const PglSchedResult r = WaitCompletion(d, service);
    syncActive_ = false;
    FreeSlot(d);
    ++stats_.dispatches;
    return r;
}

PglSchedResult PglTileScheduler::DispatchTilePassCustom(
        void* ctx, PglTileVisitor visitor, uint16_t w, uint16_t h,
        void (*service)()) {
    return DispatchTilePassImpl(ctx, visitor, nullptr, w, h, service);
}

// ─── Rasterizer adapter ─────────────────────────────────────────────────────

namespace {
void RasterTileVisitor(void* vctx, uint16_t tileCol, uint16_t tileRow,
                       uint16_t tileW, uint16_t tileH, void (*service)()) {
    auto* c = static_cast<PglRasterTileCtx*>(vctx);
    // `service` is non-null only for control-core-claimed tiles; the worker
    // always passes nullptr (renderer never branches on core id).
    c->rasterizer->RasterizeTile(c->framebuffer, c->zBuffer,
                                 tileCol, tileRow, tileW, tileH, service);
}
}

PglSchedResult PglTileScheduler::DispatchTilePass(
        Rasterizer* rasterizer, uint16_t* fb, uint16_t* zBuf,
        uint16_t w, uint16_t h, void (*service)()) {
    if (!rasterizer || !fb || !zBuf) return PglSchedResult::Rejected;
    const PglRasterTileCtx adapter{ rasterizer, fb, zBuf };
    return DispatchTilePassImpl(nullptr, RasterTileVisitor, &adapter,
                                w, h, service);
}

// ─── Dispatch: synchronous pair ─────────────────────────────────────────────

PglSchedResult PglTileScheduler::DispatchPair(void (*funcA)(void*), void* ctxA,
                                               void (*funcB)(void*), void* ctxB,
                                               void (*service)()) {
    if (!initialized_ || !workerRunning_) return PglSchedResult::NotInitialized;
    if (!PglSchedPlatform::IsControlContext(this)) return PglSchedResult::WrongCore;
    if (syncActive_ || parked_) return PglSchedResult::Busy;

    if (!funcB) {
        // No worker side: pure control-core work, nothing to rendezvous.
        if (funcA) funcA(ctxA);
        return PglSchedResult::Ok;
    }

    JobDesc* d = AllocSlot();
    if (!d) {
        ++stats_.queueFull;
        return PglSchedResult::QueueFull;
    }
    syncActive_ = true;

    d->epoch       = NextEpoch();
    d->kind        = KIND_PAIR_SYNC;
    d->funcB       = funcB;
    d->ctxB        = ctxB;
    d->tileCount   = 0;
    d->cplReceived = 0;

    PublishJob(d);

    if (funcA) funcA(ctxA);   // control-core half, concurrent with funcB

    const PglSchedResult r = WaitCompletion(d, service);
    syncActive_ = false;
    FreeSlot(d);
    ++stats_.dispatches;
    return r;
}

// ─── Dispatch: asynchronous worker jobs ─────────────────────────────────────

PglSchedResult PglTileScheduler::SubmitPairAsync(void (*func)(void*), void* ctx,
                                                  PglJobHandle* outHandle) {
    if (!initialized_ || !workerRunning_) return PglSchedResult::NotInitialized;
    if (!PglSchedPlatform::IsControlContext(this)) return PglSchedResult::WrongCore;
    if (!func || !outHandle) return PglSchedResult::Rejected;
    if (parked_) return PglSchedResult::Busy;

    JobDesc* d = AllocSlot();
    if (!d) {
        ++stats_.queueFull;
        return PglSchedResult::QueueFull;
    }

    d->epoch       = NextEpoch();
    d->kind        = KIND_PAIR_ASYNC;
    d->funcB       = func;
    d->ctxB        = ctx;
    d->tileCount   = 0;
    d->cplReceived = 0;

    const uint8_t slotIdx = static_cast<uint8_t>(d - &slots_[0]);
    PublishJob(d);
    *outHandle = MakeHandle(slotIdx, d->epoch);
    ++stats_.dispatches;
    return PglSchedResult::Ok;
}

PglSchedResult PglTileScheduler::PollJob(PglJobHandle handle) {
    if (!initialized_) return PglSchedResult::NotInitialized;
    if (!PglSchedPlatform::IsControlContext(this)) return PglSchedResult::WrongCore;
    if (handle == PGL_JOB_HANDLE_INVALID) return PglSchedResult::Rejected;

    const uint8_t slotIdx = static_cast<uint8_t>((handle >> 28) - 1);
    const uint32_t epoch28 = handle & EPOCH_MASK;
    if ((handle >> 28) == 0 || slotIdx >= JOB_SLOTS) return PglSchedResult::Rejected;

    JobDesc* d = &slots_[slotIdx];
    if ((d->epoch & EPOCH_MASK) != epoch28 ||
        d->state.load(std::memory_order_acquire) == ST_FREE ||
        d->cancelRequested) {
        return PglSchedResult::Rejected;   // stale handle (slot freed/reused)
    }

    PumpCompletions(nullptr, nullptr);   // opportunistic drain
    if (!d->cplReceived) return PglSchedResult::Busy;

    const int32_t res = d->cplResult;
    const PglSchedResult r = MapResult(d, res);
    FreeSlot(d);
    return r;
}

PglSchedResult PglTileScheduler::WaitJob(PglJobHandle handle, void (*service)()) {
    PglSchedResult r;
    while ((r = PollJob(handle)) == PglSchedResult::Busy) {
        if (service) service();
    }
    return r;
}

PglSchedResult PglTileScheduler::CancelJob(PglJobHandle handle) {
    if (!initialized_) return PglSchedResult::NotInitialized;
    if (!PglSchedPlatform::IsControlContext(this)) return PglSchedResult::WrongCore;
    if (handle == PGL_JOB_HANDLE_INVALID) return PglSchedResult::Rejected;

    const uint8_t slotIdx = static_cast<uint8_t>((handle >> 28) - 1);
    const uint32_t epoch28 = handle & EPOCH_MASK;
    if ((handle >> 28) == 0 || slotIdx >= JOB_SLOTS) return PglSchedResult::Rejected;

    JobDesc* d = &slots_[slotIdx];
    if ((d->epoch & EPOCH_MASK) != epoch28) return PglSchedResult::Rejected;

    uint32_t expect = ST_QUEUED;
    if (!d->state.compare_exchange_strong(
            expect, ST_CANCELLING,
            std::memory_order_acq_rel, std::memory_order_acquire)) {
        // Already ACTIVE (or DONE): too late — collect with WaitJob.
        return PglSchedResult::Busy;
    }

    // The cancel is now guaranteed: the worker's claim (QUEUED -> ACTIVE)
    // can no longer succeed, so the job will never run.  The worker's
    // tagged acknowledgement releases the slot when it is routed (see
    // PumpCompletions) — no blocking wait here, so cancel stays safe even
    // while the worker is occupied by an earlier bounded job.
    ++stats_.cancels;
    d->cancelRequested = true;
    return PglSchedResult::Cancelled;
}

// ─── Maintenance rendezvous ─────────────────────────────────────────────────

PglSchedResult PglTileScheduler::EnterMaintenance() {
    if (!initialized_ || !workerRunning_) return PglSchedResult::NotInitialized;
    if (!PglSchedPlatform::IsControlContext(this)) return PglSchedResult::WrongCore;
    if (parked_ || syncActive_) return PglSchedResult::Busy;
    PumpCompletions(nullptr, nullptr);   // collect acknowledged cancels
    if (OutstandingJobs() != 0) return PglSchedResult::Busy;   // drained boundary

    const uint32_t epoch = NextEpoch() & EPOCH_MASK;
    parkRelease_.store(1, std::memory_order_release);   // arm BEFORE command
    PglSchedPlatform::SendCommandWord(this, KIND_PARK | epoch);

    while (!WaitSpecificCompletion(epoch, RSLT_PARKED)) {
        // Worker is idle (drained); wait for the entire acknowledgement.
    }
    // The FIFO acknowledgement precedes entry into the RAM wait.  Do not
    // permit XIP/clock maintenance until the worker is executing there.
    while (workerParkedFlag_.load(std::memory_order_acquire) == 0) {
    }
    parked_ = true;
    return PglSchedResult::Ok;
}

void PglTileScheduler::ExitMaintenance() {
    if (!parked_) return;
    parkRelease_.store(0, std::memory_order_release);
    PglSchedPlatform::ParkWake(this);
    // Bounded: the worker leaves its SRAM-resident wait after the wake.
    while (workerParkedFlag_.load(std::memory_order_acquire) != 0) {
    }
    parked_ = false;
}

// ─── PairDispatch (instance-free worker pair) ───────────────────────────────

namespace PairDispatch {

PglSchedResult Run(void (*core0Func)(void*), void* core0Ctx,
                   void (*core1Func)(void*), void* core1Ctx,
                   void (*idleFunc)()) {
    PglTileScheduler* s = PglTileScheduler::ActiveInstance();
    if (s && s->IsWorkerRunning()) {
        return s->DispatchPair(core0Func, core0Ctx, core1Func, core1Ctx,
                               idleFunc);
    }
    // No live scheduler (never initialised / worker shut down): deterministic
    // in-order serial execution — byte-identical under the disjoint-output
    // contract.  Not a protocol path; never taken on firmware.
    (void)idleFunc;
    if (core0Func) core0Func(core0Ctx);
    if (core1Func) core1Func(core1Ctx);
    return PglSchedResult::Ok;
}

}  // namespace PairDispatch
