/**
 * @file check_scheduler.cpp
 * @brief Native behavior checks for the bounded dual-core scheduler (P04).
 *
 * Exercises the REAL scheduler (pgl_tile_scheduler.cpp) with the REAL
 * native dual-worker backend (pgl_scheduler_native.cpp — a second
 * std::thread), not a serial stub:
 *
 *   1. Exactly-once tile claims across two workers, full extents and
 *      partial-edge extents, both workers participating under skew.
 *   2. Morton table is a permutation of the tile ids.
 *   3. Grid/extent validation (>64 cells, zero dims rejected).
 *   4. Finite results: NotInitialized, Busy (nonreentrant dispatch),
 *      WrongCore (dispatch from the worker thread).
 *   5. Async submit / poll / wait with a blocked worker; queue-full bound.
 *   6. Pre-start cancel: job never runs, finite Cancelled result.
 *   7. Stale completion words are ignored (cannot complete a live job);
 *      bad-magic words counted as protocol faults.
 *   8. Pair skew: control stays responsive via the service callback while
 *      the worker runs a slow bounded job.
 *   9. Maintenance rendezvous: park blocks dispatch, release resumes.
 *  10. Shutdown/restart of the native worker thread; PairDispatch serial
 *      fallback only when no worker is running.
 *  11. Real worker completion publication paused between header/payload:
 *      poll remains Busy, cancel acknowledgements cannot free other slots,
 *      and maintenance waits for the payload AND entry into the park wait.
 *
 * Exit code 0 = all checks passed.
 */

#include "scheduler/pgl_tile_scheduler.h"

#include <atomic>
#include <chrono>
#include <cstdio>
#include <thread>
#include <utility>

static int g_failures = 0;

#define CHECK(cond)                                                        \
    do {                                                                   \
        if (!(cond)) {                                                     \
            std::printf("FAIL %s:%d  CHECK(%s)\n", __FILE__, __LINE__,     \
                        #cond);                                            \
            ++g_failures;                                                  \
        }                                                                  \
    } while (0)

// ─── Shared fixtures ────────────────────────────────────────────────────────

struct TileLog {
    std::atomic<uint32_t> claims[TileConfig::MAX_TILES];
    std::atomic<uint32_t> controlTiles{0};
    std::atomic<uint32_t> workerTiles{0};
    uint16_t cols = 0;
    int sleepUs = 0;

    void Reset(uint16_t c, int sleep) {
        for (auto& a : claims) a.store(0, std::memory_order_relaxed);
        controlTiles.store(0);
        workerTiles.store(0);
        cols = c;
        sleepUs = sleep;
    }
};

static void LogVisitor(void* vctx, uint16_t col, uint16_t row,
                       uint16_t /*tileW*/, uint16_t /*tileH*/,
                       void (*service)()) {
    auto* l = static_cast<TileLog*>(vctx);
    if (l->sleepUs > 0) {
        std::this_thread::sleep_for(std::chrono::microseconds(l->sleepUs));
    }
    l->claims[row * l->cols + col].fetch_add(1, std::memory_order_relaxed);
    // The scheduler hands the service callback ONLY to the control core.
    if (service) l->controlTiles.fetch_add(1);
    else         l->workerTiles.fetch_add(1);
}

struct PairCtx {
    std::atomic<int> ran{0};
    std::atomic<int> release{0};
};

static void MarkRan(void* v) { static_cast<PairCtx*>(v)->ran.store(1); }

static void SleepJob(void* v) {
    static_cast<PairCtx*>(v)->ran.store(1);
    std::this_thread::sleep_for(std::chrono::milliseconds(3));
}

static void BlockingJob(void* v) {
    auto* c = static_cast<PairCtx*>(v);
    c->ran.store(1);
    // Bounded wait (5 s worst case) so a test failure cannot hang forever.
    for (int i = 0; i < 50000 &&
                    !c->release.load(std::memory_order_acquire); ++i) {
        std::this_thread::sleep_for(std::chrono::microseconds(100));
    }
}

static void CountRan(void* v) {
    static_cast<PairCtx*>(v)->ran.fetch_add(1, std::memory_order_relaxed);
}

struct PayloadCtx {
    uint32_t visits = 0;
    uint32_t output[4] = {};
};

static void WritePayload(void* v) {
    auto* c = static_cast<PayloadCtx*>(v);
    ++c->visits;
    c->output[0] = 0x12345678u;
    c->output[1] = 0x89ABCDEFu;
    c->output[2] = 0x2468ACE0u;
    c->output[3] = 0x13579BDFu;
}

static std::atomic<uint32_t> g_serviceCalls{0};
static void CountingService() { g_serviceCalls.fetch_add(1); }

// Reentrancy probes (function pointers carry no context — file-scope state).
static PglTileScheduler* g_probeSched = nullptr;
static TileLog*          g_probeLog = nullptr;
static PglSchedResult    g_probeResult = PglSchedResult::Ok;

static void NestedDispatchFromPair(void* v) {
    static_cast<PairCtx*>(v)->ran.store(1);
    g_probeResult = g_probeSched->DispatchTilePassCustom(
        g_probeLog, LogVisitor, 64, 64, nullptr);
}

static void ReentrantService() {
    g_probeResult = g_probeSched->DispatchTilePassCustom(
        g_probeLog, LogVisitor, 16, 16, nullptr);
}

static void WrongCoreJob(void*) {
    g_probeResult = g_probeSched->DispatchPair(nullptr, nullptr,
                                               nullptr, nullptr, nullptr);
}

static void DrainCompletionFromPair(void* v) {
    PglJobHandle h = PGL_JOB_HANDLE_INVALID;
    CHECK(g_probeSched->SubmitPairAsync(CountRan, v, &h) == PglSchedResult::Ok);
    CHECK(g_probeSched->WaitJob(h, nullptr) == PglSchedResult::Ok);
}

// ─── Tests ──────────────────────────────────────────────────────────────────

static void TestPartialCompletion(PglTileScheduler& sched) {
    const uint32_t faults = sched.GetStats().protocolFaults;
    const uint32_t stale = sched.GetStats().staleCompletions;
    PglJobHandle previous = PGL_JOB_HANDLE_INVALID;
    for (unsigned i = 0; i < 12; ++i) {
        PayloadCtx output;
        PglJobHandle h = PGL_JOB_HANDLE_INVALID;
        // Backpressure stalls the actual native worker after publishing
        // its real header.  No synthetic job result is supplied by a test.
        PglSchedTest::HoldCompletionAfterTag(&sched);
        CHECK(sched.SubmitPairAsync(WritePayload, &output, &h)
              == PglSchedResult::Ok);
        CHECK(PglSchedTest::WaitForHeldCompletionTag(&sched));
        for (unsigned poll = 0; poll < 4; ++poll) {
            CHECK(sched.PollJob(h) == PglSchedResult::Busy);
            CHECK(sched.GetStats().protocolFaults == faults);
            CHECK(sched.GetStats().staleCompletions == stale);
        }
        if (previous != PGL_JOB_HANDLE_INVALID) {
            CHECK(sched.PollJob(previous) == PglSchedResult::Rejected);
        }
        PglSchedTest::ReleaseCompletionPayload(&sched);
        CHECK(sched.WaitJob(h, CountingService) == PglSchedResult::Ok);
        // These non-atomic outputs are read only after checked completion,
        // exercising the descriptor's release/acquire publication.
        CHECK(output.visits == 1);
        CHECK(output.output[0] == 0x12345678u);
        CHECK(output.output[1] == 0x89ABCDEFu);
        CHECK(output.output[2] == 0x2468ACE0u);
        CHECK(output.output[3] == 0x13579BDFu);
        previous = h;
    }
    CHECK(sched.GetStats().protocolFaults == faults);
    CHECK(sched.GetStats().staleCompletions == stale);
}

static void TestPartialCancelCompletion(PglTileScheduler& sched) {
    const uint32_t faults = sched.GetStats().protocolFaults;
    PairCtx first, cancelled, later;
    PglJobHandle firstHandle = PGL_JOB_HANDLE_INVALID;
    PglJobHandle cancelledHandle = PGL_JOB_HANDLE_INVALID;
    PglJobHandle laterHandle = PGL_JOB_HANDLE_INVALID;
    CHECK(sched.SubmitPairAsync(BlockingJob, &first, &firstHandle)
          == PglSchedResult::Ok);
    for (int i = 0; i < 50000 && !first.ran.load(); ++i) {
        std::this_thread::sleep_for(std::chrono::microseconds(100));
    }
    CHECK(first.ran.load() == 1);
    CHECK(sched.SubmitPairAsync(CountRan, &cancelled, &cancelledHandle)
          == PglSchedResult::Ok);
    CHECK(sched.CancelJob(cancelledHandle) == PglSchedResult::Cancelled);
    CHECK(sched.PollJob(cancelledHandle) == PglSchedResult::Rejected);
    PglSchedTest::HoldCompletionAfterTag(&sched, cancelledHandle & 0x0FFFFFFFu);
    first.release.store(1, std::memory_order_release);
    CHECK(sched.WaitJob(firstHandle, nullptr) == PglSchedResult::Ok);
    CHECK(PglSchedTest::WaitForHeldCompletionTag(&sched));
    // The first slot is reusable, but the cancelled slot must retain its
    // lifetime until the real CANCELLED payload arrives.
    CHECK(sched.SubmitPairAsync(CountRan, &later, &laterHandle)
          == PglSchedResult::Ok);
    CHECK((laterHandle >> 28) != (cancelledHandle >> 28));
    CHECK(sched.PollJob(laterHandle) == PglSchedResult::Busy);
    CHECK(sched.PollJob(cancelledHandle) == PglSchedResult::Rejected);
    PglSchedTest::ReleaseCompletionPayload(&sched);
    CHECK(sched.WaitJob(laterHandle, nullptr) == PglSchedResult::Ok);
    CHECK(cancelled.ran.load() == 0);
    CHECK(later.ran.load() == 1);
    CHECK(sched.GetStats().protocolFaults == faults);

    // Force the cancelled descriptor's slot to be reused, then replay its
    // retired acknowledgement while the later job is genuinely queued.
    PairCtx blocker, reused;
    PglJobHandle blockerHandle = PGL_JOB_HANDLE_INVALID;
    PglJobHandle reusedHandle = PGL_JOB_HANDLE_INVALID;
    CHECK(sched.SubmitPairAsync(BlockingJob, &blocker, &blockerHandle)
          == PglSchedResult::Ok);
    for (int i = 0; i < 50000 && !blocker.ran.load(); ++i) {
        std::this_thread::sleep_for(std::chrono::microseconds(100));
    }
    CHECK(blocker.ran.load() == 1);
    CHECK(sched.SubmitPairAsync(CountRan, &reused, &reusedHandle)
          == PglSchedResult::Ok);
    CHECK((reusedHandle >> 28) == (cancelledHandle >> 28));
    const uint32_t stale = sched.GetStats().staleCompletions;
    PglSchedPlatform::PushCompletionWord(
        &sched, 0xC0000000u | (cancelledHandle & 0x0FFFFFFFu));
    PglSchedPlatform::PushCompletionWord(&sched, 1u);
    CHECK(sched.PollJob(reusedHandle) == PglSchedResult::Busy);
    CHECK(sched.GetStats().staleCompletions == stale + 1);
    CHECK(sched.GetStats().protocolFaults == faults);
    blocker.release.store(1, std::memory_order_release);
    CHECK(sched.WaitJob(blockerHandle, nullptr) == PglSchedResult::Ok);
    CHECK(sched.WaitJob(reusedHandle, nullptr) == PglSchedResult::Ok);
    CHECK(reused.ran.load() == 1);
}

static void TestPrematureResultIgnored(PglTileScheduler& sched) {
    PairCtx live;
    PglJobHandle h = PGL_JOB_HANDLE_INVALID;
    CHECK(sched.SubmitPairAsync(BlockingJob, &live, &h) == PglSchedResult::Ok);
    for (int i = 0; i < 50000 && !live.ran.load(); ++i) {
        std::this_thread::sleep_for(std::chrono::microseconds(100));
    }
    CHECK(live.ran.load() == 1);
    const uint32_t faults = sched.GetStats().protocolFaults;
    // A correct epoch with a premature CANCELLED result is not permission
    // to free a running context.  The real producer is blocked in the job,
    // so fault injection cannot interleave with its two-word completion.
    PglSchedPlatform::PushCompletionWord(&sched, 0xC0000000u | (h & 0x0FFFFFFFu));
    PglSchedPlatform::PushCompletionWord(&sched, 1u);
    CHECK(sched.PollJob(h) == PglSchedResult::Busy);
    CHECK(sched.GetStats().protocolFaults == faults + 1);
    live.release.store(1, std::memory_order_release);
    CHECK(sched.WaitJob(h, nullptr) == PglSchedResult::Ok);
    CHECK(live.ran.load() == 1);
}

static void TestPartialMaintenanceCompletion(PglTileScheduler& sched) {
    const uint32_t faults = sched.GetStats().protocolFaults;
    PglSchedTest::HoldCompletionAfterTag(&sched);
    bool consumed = false;
    bool parkedBeforePayload = true;
    std::thread releaser([&] {
        // Wait for the real control owner to consume the isolated header.
        consumed = PglSchedTest::WaitForHeldCompletionTag(&sched, true);
        parkedBeforePayload = sched.IsWorkerParked();
        PglSchedTest::ReleaseCompletionPayload(&sched);
    });
    CHECK(sched.EnterMaintenance() == PglSchedResult::Ok);
    releaser.join();
    CHECK(consumed);
    CHECK(!parkedBeforePayload);
    CHECK(sched.IsWorkerParked());
    CHECK(sched.GetStats().protocolFaults == faults);
    sched.ExitMaintenance();
    CHECK(!sched.IsWorkerParked());
}

static void TestAlreadyRoutedCompletion(PglTileScheduler& sched) {
    g_probeSched = &sched;
    PairCtx side;
    PayloadCtx worker;
    const uint32_t faults = sched.GetStats().protocolFaults;
    // The worker's synchronous record precedes the nested async record.
    // Collecting the async job from the control half therefore routes the
    // outer record before its synchronous WaitCompletion begins.
    CHECK(sched.DispatchPair(DrainCompletionFromPair, &side,
                             WritePayload, &worker, nullptr) == PglSchedResult::Ok);
    CHECK(side.ran.load() == 1);
    CHECK(worker.visits == 1);
    CHECK(sched.GetStats().protocolFaults == faults);
}

static void TestWorkerFaultIsControlOwned(PglTileScheduler& sched) {
    const uint32_t faults = sched.GetStats().protocolFaults;
    const uint32_t stale = sched.GetStats().staleCompletions;
    PglSchedTest::HoldCompletionAfterTag(&sched);
    // An invalid command makes the real worker publish ProtocolFault.
    // Its diagnostics must not mutate control-owned stats concurrently,
    // and a header alone must not count as a complete report.
    PglSchedPlatform::SendCommandWord(&sched, 0xF0777777u);
    CHECK(PglSchedTest::WaitForHeldCompletionTag(&sched));
    CHECK(sched.GetStats().protocolFaults == faults);
    PairCtx live;
    PglJobHandle h = PGL_JOB_HANDLE_INVALID;
    CHECK(sched.SubmitPairAsync(CountRan, &live, &h) == PglSchedResult::Ok);
    CHECK(sched.PollJob(h) == PglSchedResult::Busy);
    CHECK(sched.GetStats().protocolFaults == faults);
    PglSchedTest::ReleaseCompletionPayload(&sched);
    CHECK(sched.WaitJob(h, nullptr) == PglSchedResult::Ok);
    CHECK(live.ran.load() == 1);
    CHECK(sched.GetStats().protocolFaults == faults + 1);
    CHECK(sched.GetStats().staleCompletions == stale + 1);
}

static void TestMortonPermutation() {
    uint8_t order[TileConfig::MAX_TILES];
    const std::pair<uint16_t, uint16_t> grids[] = {
        {8, 4}, {7, 5}, {6, 6}, {1, 1}, {8, 8},
    };
    for (const auto& grid : grids) {
        const uint16_t cols = grid.first, rows = grid.second;
        const uint16_t count = TileConfig::BuildMortonOrder(
            cols, rows, order, TileConfig::MAX_TILES);
        CHECK(count == cols * rows);
        bool seen[TileConfig::MAX_TILES] = {};
        bool permutation = true;
        for (uint16_t i = 0; i < count; ++i) {
            if (order[i] >= count || seen[order[i]]) {
                permutation = false;
                break;
            }
            seen[order[i]] = true;
        }
        CHECK(permutation);
    }
}

static void TestExactlyOnceTiles(PglTileScheduler& sched) {
    TileLog log;

    // Full 64-cell grid (max), skewed per-tile cost so both workers share.
    log.Reset(TileConfig::ColsFor(128), 40);
    g_serviceCalls.store(0);
    PglSchedResult r = sched.DispatchTilePassCustom(&log, LogVisitor,
                                                    128, 128, CountingService);
    CHECK(r == PglSchedResult::Ok);
    for (int i = 0; i < 64; ++i) {
        CHECK(log.claims[i].load() == 1);            // exactly-once
    }
    CHECK(log.controlTiles.load() + log.workerTiles.load() == 64);
    CHECK(log.controlTiles.load() > 0);              // control participated
    CHECK(log.workerTiles.load() > 0);               // worker participated
    CHECK(g_serviceCalls.load() > 0);                // service between tiles

    // Partial-edge extent: 100x70 -> 7x5 = 35 cells.
    log.Reset(TileConfig::ColsFor(100), 40);
    r = sched.DispatchTilePassCustom(&log, LogVisitor, 100, 70, nullptr);
    CHECK(r == PglSchedResult::Ok);
    for (int i = 0; i < 35; ++i) CHECK(log.claims[i].load() == 1);
    for (int i = 35; i < 64; ++i) CHECK(log.claims[i].load() == 0);
    CHECK(log.controlTiles.load() + log.workerTiles.load() == 35);

    // Epoch advance + slot reuse across many back-to-back passes.
    for (int i = 0; i < 25; ++i) {
        const uint16_t w = (i & 1) ? 128 : 96;
        const uint16_t h = (i & 1) ? 64 : 96;
        log.Reset(TileConfig::ColsFor(w), 0);
        r = sched.DispatchTilePassCustom(&log, LogVisitor, w, h, nullptr);
        CHECK(r == PglSchedResult::Ok);
        const uint16_t count = TileConfig::CountFor(w, h);
        uint32_t total = 0;
        for (uint16_t t = 0; t < count; ++t) total += log.claims[t].load();
        CHECK(total == count);
    }
    CHECK(sched.GetStats().dispatches >= 27);
    CHECK(sched.GetStats().protocolFaults == 0);
}

static void TestValidation(PglTileScheduler& sched) {
    TileLog log;
    log.Reset(1, 0);

    CHECK(sched.DispatchTilePassCustom(&log, LogVisitor, 0, 64, nullptr)
          == PglSchedResult::Rejected);
    CHECK(sched.DispatchTilePassCustom(&log, LogVisitor, 64, 0, nullptr)
          == PglSchedResult::Rejected);
    // 1024x64 -> 64x4 = 256 cells > MAX_TILES.
    CHECK(sched.DispatchTilePassCustom(&log, LogVisitor, 1024, 64, nullptr)
          == PglSchedResult::Rejected);
    CHECK(sched.DispatchTilePassCustom(&log, nullptr, 64, 64, nullptr)
          == PglSchedResult::Rejected);
    // Smallest valid grid: 1 tile.
    CHECK(sched.DispatchTilePassCustom(&log, LogVisitor, 16, 16, nullptr)
          == PglSchedResult::Ok);
    CHECK(log.claims[0].load() == 1);
}

static void TestNotInitialized() {
    PglTileScheduler orphan;   // never Initialize()d
    TileLog log;
    log.Reset(1, 0);
    CHECK(orphan.DispatchTilePassCustom(&log, LogVisitor, 64, 64, nullptr)
          == PglSchedResult::NotInitialized);
    CHECK(!orphan.StartWorker());   // requires Initialize first
}

static void TestNonReentrant(PglTileScheduler& sched) {
    TileLog log;
    log.Reset(TileConfig::ColsFor(64), 0);
    g_probeSched = &sched;
    g_probeLog = &log;

    // Nested synchronous dispatch from inside a control-core pair function.
    PairCtx outer, worker;
    g_probeResult = PglSchedResult::Ok;
    PglSchedResult r = sched.DispatchPair(NestedDispatchFromPair, &outer,
                                          MarkRan, &worker, nullptr);
    CHECK(r == PglSchedResult::Ok);
    CHECK(g_probeResult == PglSchedResult::Busy);   // nonreentrant guard

    // Reentrancy from a service callback must also fail.
    g_probeResult = PglSchedResult::Ok;
    PairCtx slow;
    r = sched.DispatchPair(nullptr, nullptr, SleepJob, &slow,
                           ReentrantService);
    CHECK(r == PglSchedResult::Ok);
    CHECK(g_probeResult == PglSchedResult::Busy);
}

static void TestWrongCore(PglTileScheduler& sched) {
    g_probeSched = &sched;
    g_probeResult = PglSchedResult::Ok;
    PglJobHandle h = PGL_JOB_HANDLE_INVALID;
    CHECK(sched.SubmitPairAsync(WrongCoreJob, nullptr, &h)
          == PglSchedResult::Ok);
    CHECK(sched.WaitJob(h, nullptr) == PglSchedResult::Ok);
    CHECK(g_probeResult == PglSchedResult::WrongCore);
}

static void TestAsyncPollWaitCancel(PglTileScheduler& sched) {
    // Poll reports Busy while the worker is blocked; WaitJob collects Ok.
    PairCtx a;
    PglJobHandle ha = PGL_JOB_HANDLE_INVALID;
    CHECK(sched.SubmitPairAsync(BlockingJob, &a, &ha) == PglSchedResult::Ok);
    CHECK(ha != PGL_JOB_HANDLE_INVALID);
    for (int i = 0; i < 50000 && !a.ran.load(); ++i) {
        std::this_thread::sleep_for(std::chrono::microseconds(100));
    }
    CHECK(a.ran.load() == 1);                        // worker started job A
    CHECK(sched.PollJob(ha) == PglSchedResult::Busy);

    // Queue job B behind A and cancel it before the worker can start it.
    // Cancel is non-blocking and guaranteed: B must never run.
    PairCtx b;
    PglJobHandle hb = PGL_JOB_HANDLE_INVALID;
    CHECK(sched.SubmitPairAsync(SleepJob, &b, &hb) == PglSchedResult::Ok);
    CHECK(sched.CancelJob(hb) == PglSchedResult::Cancelled);
    CHECK(b.ran.load() == 0);                        // cancelled job never ran

    // Cancelling the RUNNING job reports Busy (too late).
    CHECK(sched.CancelJob(ha) == PglSchedResult::Busy);

    a.release.store(1, std::memory_order_release);
    CHECK(sched.WaitJob(ha, nullptr) == PglSchedResult::Ok);

    // Give the worker time to consume (and skip) the cancelled command;
    // B must still not have run.
    std::this_thread::sleep_for(std::chrono::milliseconds(20));
    CHECK(b.ran.load() == 0);

    // A collected job's handle is stale.
    CHECK(sched.PollJob(ha) == PglSchedResult::Rejected);

    // QueueFull: 1 running + 3 queued occupies all 4 fixed slots.
    PairCtx c[4];
    PglJobHandle hs[4];
    CHECK(sched.SubmitPairAsync(BlockingJob, &c[0], &hs[0])
          == PglSchedResult::Ok);
    for (int i = 0; i < 50000 && !c[0].ran.load(); ++i) {
        std::this_thread::sleep_for(std::chrono::microseconds(100));
    }
    CHECK(c[0].ran.load() == 1);
    for (int i = 1; i < 4; ++i) {
        CHECK(sched.SubmitPairAsync(SleepJob, &c[i], &hs[i])
              == PglSchedResult::Ok);
    }
    PglJobHandle hx = PGL_JOB_HANDLE_INVALID;
    CHECK(sched.SubmitPairAsync(SleepJob, nullptr, &hx)
          == PglSchedResult::QueueFull);
    for (int i = 0; i < 4; ++i) c[i].release.store(1, std::memory_order_release);
    for (int i = 0; i < 4; ++i) {
        CHECK(sched.WaitJob(hs[i], nullptr) == PglSchedResult::Ok);
    }
}

static void TestStaleCompletionIgnored(PglTileScheduler& sched) {
    // Inject a stale completion (epoch matching no live job) and a bad-magic
    // word pair directly into the worker->control channel.
    const uint32_t stale  = sched.GetStats().staleCompletions;
    const uint32_t faults = sched.GetStats().protocolFaults;
    PglSchedPlatform::PushCompletionWord(&sched, 0xC0000000u | 0x0777777u);
    PglSchedPlatform::PushCompletionWord(&sched, 0u);
    PglSchedPlatform::PushCompletionWord(&sched, 0xDEADBEEFu);  // bad magic
    PglSchedPlatform::PushCompletionWord(&sched, 0u);

    TileLog log;
    log.Reset(1, 0);
    // The live dispatch must complete normally: the stale word cannot
    // satisfy its completion wait.
    CHECK(sched.DispatchTilePassCustom(&log, LogVisitor, 16, 16, nullptr)
          == PglSchedResult::Ok);
    CHECK(log.claims[0].load() == 1);
    CHECK(sched.GetStats().staleCompletions == stale + 1);
    CHECK(sched.GetStats().protocolFaults == faults + 1);
}

static void TestPairSkewAndService(PglTileScheduler& sched) {
    PairCtx fast, slow;   // control half fast, worker half slow but bounded
    g_serviceCalls.store(0);
    const auto t0 = std::chrono::steady_clock::now();
    PglSchedResult r = sched.DispatchPair(MarkRan, &fast, SleepJob, &slow,
                                          CountingService);
    const auto dt = std::chrono::steady_clock::now() - t0;
    CHECK(r == PglSchedResult::Ok);
    CHECK(fast.ran.load() == 1);
    CHECK(slow.ran.load() == 1);
    // Control stayed responsive: the service callback ran while the worker
    // was still executing its (slower) half.
    CHECK(g_serviceCalls.load() > 0);
    CHECK(dt >= std::chrono::milliseconds(2));   // actually waited for worker
}

static void TestMaintenance(PglTileScheduler& sched) {
    TileLog log;
    log.Reset(1, 0);

    CHECK(sched.EnterMaintenance() == PglSchedResult::Ok);
    CHECK(sched.IsWorkerParked());
    CHECK(sched.DispatchTilePassCustom(&log, LogVisitor, 16, 16, nullptr)
          == PglSchedResult::Busy);
    PglJobHandle h = PGL_JOB_HANDLE_INVALID;
    CHECK(sched.SubmitPairAsync(SleepJob, nullptr, &h)
          == PglSchedResult::Busy);
    CHECK(sched.EnterMaintenance() == PglSchedResult::Busy);  // already parked
    sched.ExitMaintenance();
    CHECK(!sched.IsWorkerParked());
    CHECK(sched.DispatchTilePassCustom(&log, LogVisitor, 16, 16, nullptr)
          == PglSchedResult::Ok);
    CHECK(log.claims[0].load() == 1);
}

static void TestShutdownRestart(PglTileScheduler& sched) {
    TileLog log;
    log.Reset(1, 0);

    sched.Shutdown();
    CHECK(!sched.IsWorkerRunning());
    CHECK(sched.DispatchTilePassCustom(&log, LogVisitor, 16, 16, nullptr)
          == PglSchedResult::NotInitialized);

    // PairDispatch with no live worker: deterministic serial fallback.
    PairCtx a, b;
    CHECK(PairDispatch::Run(MarkRan, &a, MarkRan, &b, nullptr)
          == PglSchedResult::Ok);
    CHECK(a.ran.load() == 1 && b.ran.load() == 1);

    CHECK(sched.StartWorker());
    CHECK(sched.IsWorkerRunning());
    CHECK(sched.DispatchTilePassCustom(&log, LogVisitor, 16, 16, nullptr)
          == PglSchedResult::Ok);
    CHECK(log.claims[0].load() == 1);

    // PairDispatch with a live worker routes through the scheduler.
    PairCtx c, d;
    CHECK(PairDispatch::Run(MarkRan, &c, MarkRan, &d, nullptr)
          == PglSchedResult::Ok);
    CHECK(c.ran.load() == 1 && d.ran.load() == 1);
}

// ─── Main ───────────────────────────────────────────────────────────────────

int main() {
    std::printf("scheduler native behavior check\n");

    TestMortonPermutation();
    TestNotInitialized();

    PglTileScheduler sched;
    sched.Initialize();
    CHECK(sched.StartWorker());

    TestExactlyOnceTiles(sched);
    TestValidation(sched);
    TestNonReentrant(sched);
    TestWrongCore(sched);
    TestAsyncPollWaitCancel(sched);
    TestPartialCompletion(sched);
    TestPartialCancelCompletion(sched);
    TestPartialMaintenanceCompletion(sched);
    TestAlreadyRoutedCompletion(sched);
    TestStaleCompletionIgnored(sched);
    TestPrematureResultIgnored(sched);
    TestWorkerFaultIsControlOwned(sched);
    TestPairSkewAndService(sched);
    TestMaintenance(sched);
    TestShutdownRestart(sched);

    sched.Shutdown();

    const PglSchedStats& st = sched.GetStats();
    std::printf("stats: dispatches=%u tiles=%u stale=%u faults=%u "
                "queueFull=%u cancels=%u\n",
                st.dispatches, st.tilesProcessed, st.staleCompletions,
                st.protocolFaults, st.queueFull, st.cancels);

    if (g_failures) {
        std::printf("RESULT: FAIL (%d check%s failed)\n", g_failures,
                    g_failures == 1 ? "" : "s");
        return 1;
    }
    std::printf("RESULT: PASS\n");
    return 0;
}
