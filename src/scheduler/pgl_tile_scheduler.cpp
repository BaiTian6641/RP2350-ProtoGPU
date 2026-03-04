/**
 * @file pgl_tile_scheduler.cpp
 * @brief Bare-metal tile-based dual-core scheduler — RP2350 implementation.
 *
 * Implements the lock-free work-stealing tile scheduler described in
 * pgl_tile_scheduler.h.
 *
 * Inter-core protocol (FIFO):
 *   Core 0 → Core 1:
 *     - For tile pass:    push FIFO_CMD_TILE_PASS, push &passCtx_
 *     - For generic pair: push FIFO_CMD_PAIR, push func ptr, push ctx ptr
 *   Core 1 → Core 0:
 *     - push FIFO_DONE after completing its work
 *
 * Within a tile pass, both cores use __atomic_fetch_add on
 * passCtx_.nextTile to claim the next tile index.  No locks or
 * spinning on shared variables — the FIFO is only used for start/stop.
 *
 * Memory requirements:
 *   - passCtx_: ~24 bytes on Core 0's stack
 *   - Candidate list per tile: 128 × 2 = 256 bytes (on each core's stack)
 *   - Total per-core stack overhead: < 2 KB
 */

#include "pgl_tile_scheduler.h"

#include "pico/stdlib.h"
#include "pico/multicore.h"

#include "../render/rasterizer.h"

#include <cstdio>

// ─── FIFO Protocol Constants ────────────────────────────────────────────────

/// Command: begin a tile rasterisation pass.  Next FIFO word is &passCtx_.
static constexpr uint32_t FIFO_CMD_TILE_PASS = 0x54494C45u;  // "TILE"

/// Command: run a generic function on Core 1.  Next two words are func + ctx.
static constexpr uint32_t FIFO_CMD_PAIR      = 0x50414952u;  // "PAIR"

/// Response: Core 1 has finished its work.
static constexpr uint32_t FIFO_DONE          = 0x444F4E45u;  // "DONE"

// ─── Initialize ─────────────────────────────────────────────────────────────

void PglTileScheduler::Initialize() {
    passCtx_ = {};
    printf("[TileScheduler] Initialised — %u tiles (%ux%u grid, %ux%u px)\n",
           TileConfig::TILE_COUNT, TileConfig::COLS, TileConfig::ROWS,
           TileConfig::TILE_W, TileConfig::TILE_H);
}

// ─── Core 1 Main Loop ──────────────────────────────────────────────────────
//
// Permanent loop (never returns).  Waits for a FIFO command from Core 0,
// dispatches the appropriate work, and signals completion.

void PglTileScheduler::Core1Main() {
    printf("[TileScheduler] Core 1 entering tile loop\n");

    while (true) {
        // Wait for command from Core 0
        uint32_t cmd = multicore_fifo_pop_blocking();

        if (cmd == FIFO_CMD_TILE_PASS) {
            // Read the context pointer
            uint32_t raw = multicore_fifo_pop_blocking();
            TilePassContext* ctx = reinterpret_cast<TilePassContext*>(raw);

            // Process tiles until all are claimed
            ProcessTiles(ctx);

            // Signal completion
            __atomic_fetch_add(&ctx->coresFinished, 1, __ATOMIC_RELEASE);
            multicore_fifo_push_blocking(FIFO_DONE);

        } else if (cmd == FIFO_CMD_PAIR) {
            // Generic pair dispatch: read func + ctx pointers
            uint32_t funcRaw = multicore_fifo_pop_blocking();
            uint32_t ctxRaw  = multicore_fifo_pop_blocking();

            auto func = reinterpret_cast<void(*)(void*)>(funcRaw);
            void* ctx = reinterpret_cast<void*>(ctxRaw);

            if (func) func(ctx);

            multicore_fifo_push_blocking(FIFO_DONE);

        } else {
            // Unknown command — signal done to avoid deadlock
            printf("[TileScheduler] Core 1: unknown cmd 0x%08lX\n",
                   static_cast<unsigned long>(cmd));
            multicore_fifo_push_blocking(FIFO_DONE);
        }
    }
}

// ─── DispatchTilePass ───────────────────────────────────────────────────────
//
// Called by Core 0 to begin a dual-core tile rasterisation pass.
//
// Flow:
//   1. Populate passCtx_ with the frame's render state
//   2. Reset counters (nextTile=0, coresFinished=0)
//   3. Send passCtx_ pointer to Core 1 via FIFO
//   4. Core 0 enters ProcessTiles() itself
//   5. After Core 0 finishes its tiles, poll for Core 1 completion
//      (calling idleFunc, e.g. HUB75 refresh, while waiting)

void PglTileScheduler::DispatchTilePass(Rasterizer* rasterizer,
                                         uint16_t* fb, float* zBuf,
                                         uint16_t w, uint16_t h,
                                         void (*idleFunc)()) {
    // Populate shared context
    passCtx_.rasterizer   = rasterizer;
    passCtx_.framebuffer  = fb;
    passCtx_.zBuffer      = zBuf;
    passCtx_.panelW       = w;
    passCtx_.panelH       = h;

    // Reset work counters (atomic stores with release to ensure visibility)
    __atomic_store_n(&passCtx_.nextTile,      0, __ATOMIC_RELEASE);
    __atomic_store_n(&passCtx_.coresFinished,  0, __ATOMIC_RELEASE);

    // Signal Core 1: tile pass with context pointer
    multicore_fifo_push_blocking(FIFO_CMD_TILE_PASS);
    multicore_fifo_push_blocking(reinterpret_cast<uint32_t>(&passCtx_));

    // Core 0 processes tiles concurrently
    ProcessTiles(&passCtx_);

    // Mark Core 0 as finished
    __atomic_fetch_add(&passCtx_.coresFinished, 1, __ATOMIC_RELEASE);

    // Wait for Core 1's DONE signal, running idleFunc while polling
    while (!multicore_fifo_rvalid()) {
        if (idleFunc) idleFunc();
    }
    uint32_t response = multicore_fifo_pop_blocking();
    (void)response;  // expected: FIFO_DONE
}

// ─── DispatchPair ───────────────────────────────────────────────────────────
//
// Generic two-function dispatch: Core 0 runs one function while Core 1 runs
// another.  Used for non-tile workloads (e.g. parallel shader bands).

void PglTileScheduler::DispatchPair(void (*core0Func)(void*), void* core0Ctx,
                                     void (*core1Func)(void*), void* core1Ctx,
                                     void (*idleFunc)()) {
    // Send pair command + func + ctx to Core 1
    multicore_fifo_push_blocking(FIFO_CMD_PAIR);
    multicore_fifo_push_blocking(reinterpret_cast<uint32_t>(core1Func));
    multicore_fifo_push_blocking(reinterpret_cast<uint32_t>(core1Ctx));

    // Run Core 0's function inline
    if (core0Func) core0Func(core0Ctx);

    // Wait for Core 1
    while (!multicore_fifo_rvalid()) {
        if (idleFunc) idleFunc();
    }
    uint32_t response = multicore_fifo_pop_blocking();
    (void)response;  // expected: FIFO_DONE
}

// ─── ProcessTiles ───────────────────────────────────────────────────────────
//
// Work-stealing tile loop.  Called by BOTH cores simultaneously.
//
// Each iteration:
//   1. Atomically claim the next tile index
//   2. If index >= TILE_COUNT, all tiles are claimed — return
//   3. Look up the Morton-order tile → decode to (col, row)
//   4. Call Rasterizer::RasterizeTile() for that tile
//
// The __atomic_fetch_add guarantees each tile is processed exactly once
// without locks.

void PglTileScheduler::ProcessTiles(TilePassContext* ctx) {
    while (true) {
        // Claim next tile (atomic increment — lock-free)
        uint32_t idx = __atomic_fetch_add(&ctx->nextTile, 1, __ATOMIC_RELAXED);

        if (idx >= TileConfig::TILE_COUNT) {
            break;  // All tiles claimed
        }

        // Decode Morton-order index to (col, row) in the tile grid
        uint8_t linearId = TileConfig::MORTON_ORDER[idx];
        uint16_t tileCol = linearId % TileConfig::COLS;
        uint16_t tileRow = linearId / TileConfig::COLS;

        // Rasterize this tile
        ctx->rasterizer->RasterizeTile(
            ctx->framebuffer, ctx->zBuffer,
            tileCol, tileRow,
            TileConfig::TILE_W, TileConfig::TILE_H);
    }
}
