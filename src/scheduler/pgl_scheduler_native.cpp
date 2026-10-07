/**
 * @file pgl_scheduler_native.cpp
 * @brief Native (desktop) transport backend for PglTileScheduler — real
 *        dual-worker execution on a second std::thread.
 *
 * This backend exists so the native sim and the scheduler behavior tests
 * exercise the ACTUAL tagged command/epoch protocol and real concurrency
 * (claim races, skewed workloads, quiescence, shutdown/restart) instead of
 * the old inline serial stub.
 *
 * Memory discipline mirrors the firmware contract: the command/completion
 * channels are FIXED word rings (no heap in any dispatch path); the only
 * allocation is std::thread startup in StartWorkerThread(), which the
 * contract permits outside hot dispatch.  All blocking waits are
 * condition-variable based (no spin loops burning the host CPU).
 *
 * Build: compile together with pgl_tile_scheduler.cpp, link with -pthread.
 * Do NOT compile this file into the Pico firmware target.
 */

#include "pgl_tile_scheduler.h"

#include <condition_variable>
#ifdef PGL_SCHEDULER_TEST_HOOKS
#include <chrono>
#endif
#include <mutex>
#include <thread>

namespace {

/// Fixed word ring capacity.  In-flight words are bounded by the protocol:
/// at most JOB_SLOTS job commands (2 words each) + one park/quit command,
/// and the same number of 2-word completions.  16 slots is 2x headroom.
constexpr unsigned RING_SIZE = 16;
constexpr unsigned RING_MASK = RING_SIZE - 1;

struct WordRing {
    uint32_t words[RING_SIZE] = {};
    unsigned head = 0;   // write
    unsigned tail = 0;   // read

    bool Empty() const { return head == tail; }

    bool Full() const { return ((head + 1) & RING_MASK) == tail; }

    // The transport waits for space before publishing a word.  Never drop
    // either half of a record: device SIO FIFO has the same backpressure.
    void Push(uint32_t w) {
        const unsigned next = (head + 1) & RING_MASK;
        words[head] = w;
        head = next;
    }

    uint32_t Pop() {
        const uint32_t w = words[tail];
        tail = (tail + 1) & RING_MASK;
        return w;
    }
};

struct NativeWorker {
    PglTileScheduler* owner = nullptr;
    WordRing cmd;              // control -> worker
    WordRing cpl;              // worker  -> control
    std::mutex mtx;            // guards both rings
    std::condition_variable cv; // worker command wait + park release
    std::thread thread;
    std::thread::id controlId;
    bool running = false;
#ifdef PGL_SCHEDULER_TEST_HOOKS
    bool holdCompletion = false;
    uint32_t holdEpoch = 0;
    bool completionHeld = false;
#endif
};

/// Up to two concurrent scheduler instances (sim + tests use one).
NativeWorker g_workers[2];

NativeWorker* FindWorker(PglTileScheduler* self, bool allocate) {
    NativeWorker* freeOne = nullptr;
    for (NativeWorker& w : g_workers) {
        if (w.owner == self) return &w;
        if (!w.owner && !freeOne) freeOne = &w;
    }
    return allocate ? freeOne : nullptr;
}

}  // namespace

namespace PglSchedPlatform {

void TransportInit(PglTileScheduler* self) {
    NativeWorker* w = FindWorker(self, /*allocate=*/true);
    if (!w) return;   // more than two schedulers — unsupported
    std::lock_guard<std::mutex> lk(w->mtx);
    w->cmd = WordRing{};
    w->cpl = WordRing{};
#ifdef PGL_SCHEDULER_TEST_HOOKS
    w->holdCompletion = false;
    w->holdEpoch = 0;
    w->completionHeld = false;
#endif
    w->owner = self;
}

void NoteControlThread(PglTileScheduler* self) {
    NativeWorker* w = FindWorker(self, false);
    if (w) w->controlId = std::this_thread::get_id();
}

bool IsControlContext(PglTileScheduler* self) {
    NativeWorker* w = FindWorker(self, false);
    return !w || std::this_thread::get_id() == w->controlId;
}

void SendCommandWord(PglTileScheduler* self, uint32_t word) {
    NativeWorker* w = FindWorker(self, false);
    if (!w) return;
    {
        std::unique_lock<std::mutex> lk(w->mtx);
        w->cv.wait(lk, [&] { return !w->cmd.Full(); });
        w->cmd.Push(word);
    }
    w->cv.notify_all();
}

uint32_t WaitCommandWord(PglTileScheduler* self) {
    NativeWorker* w = FindWorker(self, false);
    std::unique_lock<std::mutex> lk(w->mtx);
    w->cv.wait(lk, [&] { return !w->cmd.Empty(); });
    const uint32_t word = w->cmd.Pop();
    lk.unlock();
    w->cv.notify_all();
    return word;
}

void PushCompletionWord(PglTileScheduler* self, uint32_t word) {
    NativeWorker* w = FindWorker(self, false);
    if (!w) return;
    // Each word is independently published, just like the device FIFO.
    // The scheduler, not a backend-specific assumption, assembles records.
    std::unique_lock<std::mutex> lk(w->mtx);
    w->cv.wait(lk, [&] { return !w->cpl.Full(); });
    w->cpl.Push(word);
#ifdef PGL_SCHEDULER_TEST_HOOKS
    if (w->holdCompletion && (word & 0xF0000000u) == 0xC0000000u &&
        (w->holdEpoch == 0 || w->holdEpoch == (word & 0x0FFFFFFFu))) {
        w->holdCompletion = false;
        w->completionHeld = true;
        w->cv.notify_all();
        w->cv.wait(lk, [&] { return !w->completionHeld; });
    }
#endif
}

bool TryPopCompletionWord(PglTileScheduler* self, uint32_t* word) {
    NativeWorker* w = FindWorker(self, false);
    if (!w) return false;
    {
        std::lock_guard<std::mutex> lk(w->mtx);
        if (w->cpl.Empty()) return false;
        *word = w->cpl.Pop();
    }
    w->cv.notify_all();
    return true;
}

bool StartWorkerThread(PglTileScheduler* self) {
    NativeWorker* w = FindWorker(self, /*allocate=*/true);
    if (!w) return false;
    if (w->running) return true;
    w->owner = self;
    // The only allocation in the backend — worker startup, outside dispatch.
    w->thread = std::thread([self] { self->WorkerThreadBody(); });
    w->running = true;
    return true;
}

void JoinWorker(PglTileScheduler* self) {
    NativeWorker* w = FindWorker(self, false);
    if (!w || !w->running) return;
    if (w->thread.joinable()) w->thread.join();
    {
        std::lock_guard<std::mutex> lk(w->mtx);
        w->cmd = WordRing{};
        w->cpl = WordRing{};
    }
    w->running = false;
}

bool WorkerJoinable() { return true; }

void ParkWait(PglTileScheduler* self, std::atomic<uint32_t>* flag,
              uint32_t waitValue, std::atomic<uint32_t>* parked) {
    NativeWorker* w = FindWorker(self, false);
    std::unique_lock<std::mutex> lk(w->mtx);
    parked->store(1, std::memory_order_release);
    w->cv.wait(lk, [&] {
        return flag->load(std::memory_order_acquire) != waitValue;
    });
    parked->store(0, std::memory_order_release);
}

void ParkWake(PglTileScheduler* self) {
    NativeWorker* w = FindWorker(self, false);
    if (!w) return;
    {
        std::lock_guard<std::mutex> lk(w->mtx);
        // Touch the mutex so the predicate re-evaluation observes the
        // flag store that happened-before ParkWake().
    }
    w->cv.notify_all();
}

}  // namespace PglSchedPlatform

#ifdef PGL_SCHEDULER_TEST_HOOKS
namespace PglSchedTest {

void HoldCompletionAfterTag(PglTileScheduler* self, uint32_t epoch28) {
    NativeWorker* w = FindWorker(self, false);
    std::lock_guard<std::mutex> lk(w->mtx);
    w->holdEpoch = epoch28 & 0x0FFFFFFFu;
    w->holdCompletion = true;
}

bool WaitForHeldCompletionTag(PglTileScheduler* self, bool consumed) {
    NativeWorker* w = FindWorker(self, false);
    std::unique_lock<std::mutex> lk(w->mtx);
    return w->cv.wait_for(lk, std::chrono::seconds(2), [&] {
        return w->completionHeld && (!consumed || w->cpl.Empty());
    });
}

void ReleaseCompletionPayload(PglTileScheduler* self) {
    NativeWorker* w = FindWorker(self, false);
    {
        std::lock_guard<std::mutex> lk(w->mtx);
        w->holdCompletion = false;
        w->completionHeld = false;
    }
    w->cv.notify_all();
}

}  // namespace PglSchedTest
#endif
