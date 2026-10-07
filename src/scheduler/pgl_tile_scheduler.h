/**
 * @file pgl_tile_scheduler.h
 * @brief Bounded bare-metal dual-core scheduler — RP2350 + native (P04).
 *
 * Single tagged command/epoch protocol over a fixed set of job descriptors.
 * There is exactly ONE worker protocol and ONE FIFO/doorbell owner; the old
 * PglJobScheduler_RP2350 adapter (untagged pointer FIFO) is removed.
 *
 * Architecture
 * ────────────
 * Core 0 is the control owner and the only side allowed to call dispatch
 * APIs (enforced: device checks get_core_num(), native checks thread id).
 * The worker (Core 1 on device, a std::thread on native) runs a permanent
 * loop consuming tagged command words:
 *
 *   control → worker (command stream, 32-bit words):
 *       [ KIND_JOB  | epoch28 ] [ job slot index ]
 *       [ KIND_PARK | epoch28 ]                       (maintenance rendezvous)
 *       [ KIND_QUIT | epoch28 ]                       (native shutdown only)
 *   worker → control (completion stream):
 *       [ CPL_MAGIC | epoch28 ] [ result code ]
 * Both completion words may arrive separately.  The control owner retains
 * a partial record until its payload is available; an empty FIFO/ring is
 * never interpreted as a result word.
 *
 * Every command and completion is tagged with a 28-bit epoch.  A completion
 * whose epoch matches no live job is STALE: it is counted and ignored.  A
 * job record is accepted only after acquiring DONE and checking its result
 * against the control owner's cancellation state, so stale/cancel records
 * cannot release a later job or its context.  The worker validates the slot
 * index and epoch before touching a context; no raw pointers cross the wire.
 * Context storage lives in the fixed descriptor array inside the scheduler —
 * stable until BOTH sides have finished, never stack-transient.
 *
 * Publication / ordering
 * ──────────────────────
 * `volatile` alone is not publication.  Descriptors are published with
 * release stores (state -> QUEUED) followed by the tagged command word;
 * the worker acquires the state before reading any field.  Completion is a
 * release store (state -> DONE) followed by the tagged completion word; the
 * control core acquires before reading results.  Tile claiming uses atomic
 * fetch_add — each tile index is handed out exactly once.
 *
 * Bounded storage / nonreentrancy
 * ───────────────────────────────
 * JOB_SLOTS = 4 fixed descriptors; no heap, no RTOS, no allocation on any
 * dispatch path (device AND native — the native backend uses fixed word
 * rings; only std::thread startup allocates, outside dispatch).  Synchronous
 * dispatch (DispatchTilePass and DispatchPair) is nonreentrant: a nested call
 * (e.g. from a service callback) returns PglSchedResult::Busy.
 *
 * Service model
 * ─────────────
 * The service callback (e.g. HUB75 refresh, transport pump) runs ONLY on
 * the control core: between bounded tile claims inside a tile pass, and
 * while waiting for worker completion.  Worker-side tile visitors receive
 * nullptr.  Pair workers must have a bounded work cost (WCET contract —
 * see DispatchPair); unbounded work must be decomposed into tiles/bands by
 * the caller or submitted async (SubmitPairAsync + PollJob/WaitJob) so the
 * control loop stays responsive.
 *
 * Maintenance rendezvous (quiescence)
 * ───────────────────────────────────
 * EnterMaintenance() requires a drained scheduler (no live jobs) and parks
 * the worker in an SRAM-safe wait: on device the park loop is
 * __no_inline_not_in_flash_func (SRAM-resident, WFE, no XIP/FIFO/heap),
 * so QMI/flash erase-program and clock/voltage transitions are safe while
 * parked.  ExitMaintenance() releases the worker (SEV / condvar) and blocks
 * until it has actually left the SRAM loop.  While parked, all dispatch
 * APIs return Busy.
 *
 * SDK ownership (device)
 * ──────────────────────
 * - The scheduler owns the RP2350 multicore SIO FIFO EXCLUSIVELY.  No other
 *   code may call multicore_fifo_* — in particular the SDK multicore
 *   lockout (multicore_lockout_*, used by flash_safe_execute) shares this
 *   FIFO and MUST NOT be used; use EnterMaintenance()/ExitMaintenance()
 *   around flash/QMI/clock work instead.
 * - Core 1 launch stays with the parent: multicore_launch_core1() →
 *   GpuCore::Core1Main() → PglTileScheduler::Core1Main() (never returns).
 *   Call Initialize() on core 0 BEFORE the launch and StartWorker() after.
 * - Shutdown() on device parks the worker permanently in the SRAM-safe wait
 *   (a core cannot be joined); the parent MAY then multicore_reset_core1()
 *   and relaunch.  On native, Shutdown() sends QUIT and joins the thread;
 *   StartWorker() starts a fresh one.
 *
 * Native build
 * ────────────
 * pgl_tile_scheduler.cpp + pgl_scheduler_native.cpp compile with host
 * C++17 (-pthread).  Real dual-worker execution: the worker is a genuine
 * second thread, so native tests exercise actual concurrency (claim races,
 * skew, quiescence, restart) instead of an inline serial stub.
 */

#pragma once

#include <atomic>
#include <cstdint>

// Forward declarations (avoid pulling large headers into every includer)
class Rasterizer;
struct SceneState;

// ─── Tile Configuration ─────────────────────────────────────────────────────

namespace TileConfig {
    static constexpr uint16_t TILE_W    = 16;
    static constexpr uint16_t TILE_H    = 16;

    /// Hard bound on tile-grid cells per pass (profile limit).
    static constexpr uint16_t MAX_TILES = 64;

    /// Runtime grid derivation from the ACTUAL target extent — the grid is
    /// not a compile-time 8x4 constant; any extent up to MAX_TILES cells is
    /// accepted (edge tiles are partial; the renderer clamps them).
    static constexpr uint16_t ColsFor(uint16_t w) {
        return static_cast<uint16_t>((w + TILE_W - 1) / TILE_W);
    }
    static constexpr uint16_t RowsFor(uint16_t h) {
        return static_cast<uint16_t>((h + TILE_H - 1) / TILE_H);
    }
    static constexpr uint16_t CountFor(uint16_t w, uint16_t h) {
        return static_cast<uint16_t>(ColsFor(w) * RowsFor(h));
    }
    static constexpr bool ExtentValid(uint16_t w, uint16_t h) {
        return w > 0 && h > 0 && CountFor(w, h) <= MAX_TILES;
    }

    /// Fill out[0..count) with linear tile ids in Morton (Z-curve) order for
    /// a cols×rows grid (improves spatial locality when two workers steal
    /// adjacent tiles).  Returns the tile count, or 0 when the grid is empty
    /// or exceeds `capacity`.  Bounded: at most 64 output entries, computed
    /// by deinterleaving — no static per-resolution tables.
    inline uint16_t BuildMortonOrder(uint16_t cols, uint16_t rows,
                                     uint8_t* out, uint16_t capacity) {
        const uint16_t count = static_cast<uint16_t>(cols * rows);
        if (count == 0 || count > capacity) return 0;
        const uint16_t maxDim = (cols > rows) ? cols : rows;
        uint8_t bits = 0;
        while ((1u << bits) < maxDim) ++bits;
        uint16_t n = 0;
        const uint32_t codeLimit = 1u << (2 * bits);
        for (uint32_t code = 0; code < codeLimit && n < count; ++code) {
            uint16_t x = 0, y = 0;
            for (uint8_t b = 0; b < bits; ++b) {
                x |= static_cast<uint16_t>(((code >> (2 * b))     & 1u) << b);
                y |= static_cast<uint16_t>(((code >> (2 * b + 1)) & 1u) << b);
            }
            if (x < cols && y < rows) {
                out[n++] = static_cast<uint8_t>(y * cols + x);
            }
        }
        return n;
    }
}

// ─── Results / handles / stats ──────────────────────────────────────────────

/// Finite dispatch/completion result.  Every dispatch API returns one of
/// these — there is no untagged "DONE" response anywhere in the protocol.
enum class PglSchedResult : uint8_t {
    Ok = 0,        ///< Completed; both sides finished, all work units done
    Busy,          ///< Nonreentrant dispatch active, worker parked, or job
                   ///< already running (cancel) / still running (poll)
    NotInitialized,///< Initialize()/StartWorker() not done (or after Shutdown)
    QueueFull,     ///< No free fixed job descriptor slot
    Rejected,      ///< Invalid arguments (null visitor, bad extent > MAX_TILES)
    WrongCore,     ///< Dispatch API called from the worker core/thread
    Cancelled,     ///< Job was cancelled before the worker started it
    Incomplete,    ///< Worker finished but not all work units completed
    ProtocolFault, ///< Tagged-protocol violation (bad tag/slot/epoch)
};

/// Opaque handle for an asynchronously submitted job.
/// Layout: [31:28] = slot index + 1, [27:0] = epoch.  0 = invalid.
using PglJobHandle = uint32_t;
static constexpr PglJobHandle PGL_JOB_HANDLE_INVALID = 0;

/// Scheduler diagnostics (control-core maintained; read on control core).
struct PglSchedStats {
    uint32_t dispatches       = 0;  ///< accepted jobs
    uint32_t tilesProcessed   = 0;  ///< rasterized tiles (both workers)
    uint32_t staleCompletions = 0;  ///< complete records with no live job (ignored)
    uint32_t protocolFaults   = 0;  ///< tag/pointer/epoch violations
    uint32_t queueFull        = 0;  ///< QueueFull rejections
    uint32_t cancels          = 0;  ///< successful pre-start cancels
};

/// Tile visitor invoked once per claimed tile.  `service` is non-null ONLY
/// on the control core (bounded service point between row/triangle slices
/// inside the renderer); the worker always receives nullptr.
using PglTileVisitor = void (*)(void* ctx, uint16_t tileCol, uint16_t tileRow,
                                uint16_t tileW, uint16_t tileH,
                                void (*service)());

/// Context storage for the Rasterizer* overload (internal — the pointer
/// handed to the visitor always refers to fixed scheduler storage).
struct PglRasterTileCtx {
    Rasterizer* rasterizer;
    uint16_t*   framebuffer;
    uint16_t*   zBuffer;
};

// ─── Tile Scheduler ─────────────────────────────────────────────────────────

class PglTileScheduler {
public:
    /// Fixed bounded job storage.
    static constexpr uint8_t JOB_SLOTS = 4;

    // ── Lifecycle ───────────────────────────────────────────────────────

    /// Initialise/reset the scheduler.  Control core only, BEFORE the
    /// worker is started.  No-op while a worker is running (Shutdown first).
    void Initialize();

    /// Start the worker.  Native: spawns the worker std::thread (startup
    /// allocation happens here, never in dispatch).  Device: records that
    /// the parent has launched Core 1 (multicore_launch_core1 → Core1Main);
    /// call after the launch.  Idempotent while running.
    bool StartWorker();

    bool IsWorkerRunning() const { return workerRunning_; }

    /// Native: QUIT + join (restartable via StartWorker).  Requires a
    /// drained scheduler.  Device: parks the worker permanently in the
    /// SRAM-safe maintenance wait (see file header; reset+relaunch to undo).
    void Shutdown();

    /// Core 1 entry point (device) — permanent command loop, never returns
    /// except after a QUIT command (native thread body uses the same loop).
    void Core1Main();

    /// Native worker thread body.  INTERNAL — platform backend only.
    void WorkerThreadBody();

    // ── Synchronous dispatch (control core, blocking) ───────────────────

    /// Tile rasterisation pass over the actual target extent (w×h).
    /// The grid is ceil(w/16)×ceil(h/16) (<= MAX_TILES cells); both workers
    /// steal tiles from one atomic counter in Morton order — every tile is
    /// visited exactly once, verified before Ok is returned.  `service`
    /// runs on the control core between tile claims and while waiting.
    /// Blocks until both sides finish; the pass context stays valid for
    /// the whole call (fixed storage, never stack-transient).
    PglSchedResult DispatchTilePass(Rasterizer* rasterizer,
                                    uint16_t* fb, uint16_t* zBuf,
                                    uint16_t w, uint16_t h,
                                    void (*service)());

    /// Renderer-agnostic tile pass (same semantics as above).
    PglSchedResult DispatchTilePassCustom(void* ctx, PglTileVisitor visitor,
                                          uint16_t w, uint16_t h,
                                          void (*service)());

    /// Run funcA on the control core and funcB on the worker, returning
    /// after BOTH complete.  WCET CONTRACT: funcA/funcB must each complete
    /// within a bounded slice (the caller decomposes heavy work into
    /// tiles/bands, or uses SubmitPairAsync so the control loop continues).
    /// The two functions must share NO mutable state (disjoint outputs,
    /// read-only inputs) — then dual-core and serial results are identical.
    PglSchedResult DispatchPair(void (*funcA)(void*), void* ctxA,
                                void (*funcB)(void*), void* ctxB,
                                void (*service)());

    // ── Asynchronous worker jobs (control core, non-blocking) ───────────

    /// Queue `func(ctx)` for execution on the WORKER only and return
    /// immediately with a job handle.  The caller keeps control-loop
    /// responsiveness and collects the finite result via PollJob/WaitJob.
    /// The caller owns dependency safety: the job's inputs must be
    /// published/immutable and its outputs disjoint from any active work.
    PglSchedResult SubmitPairAsync(void (*func)(void*), void* ctx,
                                   PglJobHandle* outHandle);

    /// Non-blocking job status.  Returns Busy while queued/running; on a
    /// terminal result (Ok/Cancelled/Incomplete/ProtocolFault) the job slot
    /// is released and the handle becomes invalid.
    PglSchedResult PollJob(PglJobHandle handle);

    /// Blocking wait with a service callback (same finite results).
    PglSchedResult WaitJob(PglJobHandle handle, void (*service)());

    /// Cancel a queued async job before the worker starts it.  Returns
    /// Cancelled on success: the job is guaranteed never to run (the
    /// worker's claim can no longer succeed) and the handle is dead — do
    /// not poll it; the slot is released when the worker's tagged
    /// acknowledgement is routed.  Non-blocking, safe while the worker is
    /// occupied.  Returns Busy if the worker already started (collect the
    /// finite result with WaitJob instead).
    PglSchedResult CancelJob(PglJobHandle handle);

    // ── Maintenance rendezvous (quiescence) ─────────────────────────────

    /// Park the worker in an SRAM-safe wait (device: SRAM-resident WFE loop,
    /// no XIP/FIFO/heap access — safe for QMI/flash/clock transitions).
    /// Requires a drained scheduler (no live jobs) and blocks until the
    /// worker is actually parked.  All dispatch APIs return Busy while
    /// parked.  Control core only.
    PglSchedResult EnterMaintenance();

    /// Release the parked worker and block until it has left the SRAM-safe
    /// wait.  Call only after XIP/clocks are restored.
    void ExitMaintenance();

    bool IsWorkerParked() const {
        return workerParkedFlag_.load(std::memory_order_acquire) != 0;
    }

    // ── Diagnostics / singleton ─────────────────────────────────────────

    const PglSchedStats& GetStats() const { return stats_; }

    /// The instance PairDispatch::Run routes through (set by Initialize).
    static PglTileScheduler* ActiveInstance() { return activeInstance_; }

private:
    // ── Job descriptor (fixed storage) ──────────────────────────────────

    enum JobState : uint32_t {
        ST_FREE = 0,   ///< slot available
        ST_FILLING,    ///< control core is populating the descriptor
        ST_QUEUED,     ///< published; worker may claim it
        ST_ACTIVE,     ///< worker is executing it
        ST_CANCELLING, ///< control cancelled before worker start
        ST_DONE,       ///< worker finished; complete acknowledgement still required
    };

    enum JobKind : uint32_t {
        KIND_NONE = 0,
        KIND_TILE_PASS,
        KIND_PAIR_SYNC,
        KIND_PAIR_ASYNC,
    };

    struct JobDesc {
        std::atomic<uint32_t> state{ST_FREE};
        uint32_t epoch      = 0;   ///< full epoch (tag carries low 28 bits)
        uint32_t kind       = KIND_NONE;
        // Immutable-after-publication work description:
        void*          ctx      = nullptr;
        PglTileVisitor visitor  = nullptr;
        void (*funcB)(void*)    = nullptr;
        void*          ctxB     = nullptr;
        uint16_t gridCols       = 0;
        uint16_t tileCount      = 0;
        PglRasterTileCtx adapter{};  ///< storage for the Rasterizer* overload
        // Work counters (both cores):
        std::atomic<uint32_t> nextTile{0};
        std::atomic<uint32_t> tilesDone{0};
        // Completion bookkeeping (control core only):
        uint8_t cplReceived = 0;
        bool cancelRequested = false;
        int32_t cplResult   = 0;
    };

    // ── Internals ───────────────────────────────────────────────────────

    JobDesc*  AllocSlot();
    void      FreeSlot(JobDesc* d);
    uint8_t   OutstandingJobs() const;
    uint32_t  NextEpoch() { return epochCounter_++; }
    void      PublishJob(JobDesc* d);
    JobDesc*  ValidateJobPointer(uint32_t raw, uint32_t epoch);
    JobDesc*  FindSlotByEpoch(uint32_t epoch28);

    PglSchedResult DispatchTilePassImpl(void* ctx, PglTileVisitor visitor,
                                        const PglRasterTileCtx* adapter,
                                        uint16_t w, uint16_t h,
                                        void (*service)());

    void ProcessTiles(JobDesc* d, bool isWorker, void (*service)());
    void ExecuteWorkerJob(JobDesc* d);
    void RunWorkerLoop();

    /// Assemble a complete two-word record without waiting for its payload.
    bool ReadCompletion(uint32_t* epoch28, int32_t* result);
    void RouteCompletion(uint32_t epoch28, int32_t result);
    /// Drain available completion words, routing each to its job slot.
    /// Returns true (and the result) when `target`'s completion arrived.
    bool PumpCompletions(JobDesc* target, int32_t* targetResult);
    PglSchedResult WaitCompletion(JobDesc* d, void (*service)());
    PglSchedResult MapResult(JobDesc* d, int32_t protocolResult);
    bool WaitSpecificCompletion(uint32_t epoch28, int32_t expectedResult);

    static PglJobHandle MakeHandle(uint8_t slotIdx, uint32_t epoch) {
        return (static_cast<uint32_t>(slotIdx + 1) << 28) | (epoch & 0x0FFFFFFFu);
    }

    // ── State ───────────────────────────────────────────────────────────

    JobDesc slots_[JOB_SLOTS];

    /// Morton table for the current grid (control core recomputes it before
    /// publishing a tile pass; at most one tile pass is outstanding because
    /// synchronous dispatch is nonreentrant and async jobs are pair-only).
    uint8_t morton_[TileConfig::MAX_TILES] = {};

    uint32_t epochCounter_ = 1;
    uint32_t completionTag_ = 0;   ///< control-owned partial record
    bool completionTagPending_ = false;

    std::atomic<uint32_t> parkRelease_{0};      ///< 1 = worker must stay parked
    std::atomic<uint32_t> workerParkedFlag_{0}; ///< worker is inside the park wait

    bool initialized_  = false;
    bool workerRunning_ = false;
    bool syncActive_   = false;  ///< synchronous dispatch in progress (nonreentrancy)
    bool parked_       = false;  ///< control-side view of the maintenance park

    PglSchedStats stats_;

    static PglTileScheduler* activeInstance_;
};

// ─── Pair Dispatch (dual-core worker pair) ──────────────────────────────────

/// Runs two worker functions as a core pair and returns only after BOTH have
/// completed.  Routes through PglTileScheduler::ActiveInstance() when a
/// scheduler with a running worker exists (firmware: always; native sim:
/// after Initialize+StartWorker — real dual-thread execution).  Returns Busy
/// while the worker is parked for maintenance.
///
/// Fallback: with NO live scheduler (e.g. a minimal harness that never
/// initialised one) both workers run sequentially on the calling thread, in
/// order.  Under the shared-no-mutable-state contract serial execution is
/// byte-identical to dual-core execution; the fallback is deterministic, not
/// a protocol path, and never triggers on firmware.
///
/// Contract (either backend): the two workers must share NO mutable state —
/// each writes its own disjoint output region and every input buffer stays
/// read-only for the whole call.  Each worker has a bounded work cost (see
/// PglTileScheduler::DispatchPair).
namespace PairDispatch {
    PglSchedResult Run(void (*core0Func)(void*), void* core0Ctx,
                       void (*core1Func)(void*), void* core1Ctx,
                       void (*idleFunc)());
}

// ─── Platform transport hooks (INTERNAL — backend implementations only) ─────
//
// Implemented per platform:
//   src/scheduler/pgl_scheduler_rp2350.cpp  — Pico SDK multicore FIFO (device)
//   src/scheduler/pgl_scheduler_native.cpp  — std::thread + fixed rings
//
// The command and completion channels are 32-bit WORD streams identical on
// both platforms, so the tagged protocol above is exercised natively.

namespace PglSchedPlatform {
    void     TransportInit(PglTileScheduler* self);     ///< reset channels
    void     NoteControlThread(PglTileScheduler* self); ///< capture control identity
    bool     IsControlContext(PglTileScheduler* self);  ///< dispatch-API gate

    void     SendCommandWord(PglTileScheduler* self, uint32_t word);
    uint32_t WaitCommandWord(PglTileScheduler* self);   ///< worker-side blocking

    void     PushCompletionWord(PglTileScheduler* self, uint32_t word);
    /// Non-blocking single-consumer read; false leaves `word` untouched.
    bool     TryPopCompletionWord(PglTileScheduler* self, uint32_t* word);

    bool     StartWorkerThread(PglTileScheduler* self);
    void     JoinWorker(PglTileScheduler* self);        ///< native only
    bool     WorkerJoinable();                          ///< device: false

    /// SRAM-safe park wait while `*flag == waitValue`.  Publish parked=1
    /// ONLY after entering the SRAM function, parked=0 before leaving it.
    /// Native uses the same handshake around its condition-variable wait.
    void     ParkWait(PglTileScheduler* self, std::atomic<uint32_t>* flag,
                      uint32_t waitValue, std::atomic<uint32_t>* parked);
    void     ParkWake(PglTileScheduler* self);
}

#ifdef PGL_SCHEDULER_TEST_HOOKS
/// Native transport backpressure: pause the real producer after a header,
/// before its real payload.  epoch28=0 selects the next completion.
namespace PglSchedTest {
    void HoldCompletionAfterTag(PglTileScheduler* self, uint32_t epoch28 = 0);
    bool WaitForHeldCompletionTag(PglTileScheduler* self, bool consumed = false);
    void ReleaseCompletionPayload(PglTileScheduler* self);
}
#endif
