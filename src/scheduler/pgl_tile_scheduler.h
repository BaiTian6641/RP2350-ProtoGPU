/**
 * @file pgl_tile_scheduler.h
 * @brief Bare-metal tile-based dual-core scheduler — RP2350 specialised.
 *
 * Replaces the abstract PglJobScheduler with a highly optimised tile-work-queue
 * designed specifically for the RP2350's dual Cortex-M33 cores.
 *
 * Architecture:
 *   ┌──────────────────────────────────────────────────────────┐
 *   │  128×64 panel → 8×4 grid of 16×16 tiles (32 tiles)      │
 *   │                                                          │
 *   │  work queue: atomic tile index (0..31)                   │
 *   │                                                          │
 *   │  Core 0:  while (tile = next_tile++) < 32: rasterize(tile)│
 *   │  Core 1:  while (tile = next_tile++) < 32: rasterize(tile)│
 *   │                                                          │
 *   │  Both cores pull from the same atomic counter — whoever  │
 *   │  finishes a tile first grabs the next one (work-stealing)│
 *   └──────────────────────────────────────────────────────────┘
 *
 * Why tiles instead of Y-band splits:
 *   - Natural load balance: complex tiles (many triangles) and empty tiles
 *     are spread across cores dynamically.  The old 50/50 Y-split can leave
 *     one core idle if all geometry is in one half.
 *   - Cache locality: a 16×16 tile touches 16×16×2 = 512 bytes of framebuffer
 *     and 16×16×4 = 1024 bytes of Z-buffer.  Both fit in L1/TCM.
 *   - QuadTree affinity: the QuadTree is queried per-tile (16×16 AABB) rather
 *     than per-scanline, amortising traversal cost across 256 pixels.
 *
 * Design decisions (bare-metal, no RTOS):
 *   - No virtual dispatch, no heap, no OS primitives.
 *   - Synchronisation uses a single volatile atomic counter + FIFO.
 *   - Core 1 starts/stops via a lightweight FIFO signal (not a semaphore).
 *   - Per-tile triangle candidate lists are cached in scratch buffers to avoid
 *     redundant QuadTree queries when tiles share the same spatial region.
 *
 * Tile order:  Morton / Z-order curve for improved spatial locality when both
 * cores fetch adjacent tiles.  For 8×4 this is a static lookup table.
 *
 * Performance target: < 1 µs scheduling overhead per tile (~32 µs total).
 */

#pragma once

#include <cstdint>

// Forward declarations (avoid pulling large headers into every includer)
class Rasterizer;
struct SceneState;

// ─── Tile Configuration ─────────────────────────────────────────────────────

namespace TileConfig {
    static constexpr uint16_t TILE_W     = 16;
    static constexpr uint16_t TILE_H     = 16;
    static constexpr uint16_t COLS       = 8;   // 128 / 16
    static constexpr uint16_t ROWS       = 4;   //  64 / 16
    static constexpr uint16_t TILE_COUNT = COLS * ROWS;  // 32

    /// Morton-order (Z-curve) tile indices for 8×4 grid.
    /// Improves spatial locality when two cores steal adjacent tiles.
    /// Generated offline: interleave X and Y bits.
    static constexpr uint8_t MORTON_ORDER[TILE_COUNT] = {
        // (col,row) pairs in Morton order, mapped to linear tileid = row*8+col
         0,  1,  8,  9,  2,  3, 10, 11,   //  Z-curve quadrant 0,1
        16, 17, 24, 25, 18, 19, 26, 27,   //  Z-curve quadrant 2,3
         4,  5, 12, 13,  6,  7, 14, 15,   //  Z-curve quadrant 4,5
        20, 21, 28, 29, 22, 23, 30, 31    //  Z-curve quadrant 6,7
    };
}

// ─── Tile Job Context ───────────────────────────────────────────────────────

/// Shared context for a rasterisation pass — read by both cores.
/// Allocated on the stack by Core 0 before signalling Core 1.
struct TilePassContext {
    Rasterizer*     rasterizer;
    uint16_t*       framebuffer;
    float*          zBuffer;
    uint16_t        panelW;
    uint16_t        panelH;

    /// Atomic tile counter — both cores increment this to claim the next tile.
    /// Uses volatile + __atomic builtins (no locks, no OS).
    volatile uint32_t nextTile;

    /// Completion counter — each core increments after finishing all its tiles.
    volatile uint32_t coresFinished;
};

// ─── Tile Scheduler ─────────────────────────────────────────────────────────

class PglTileScheduler {
public:
    // ── Lifecycle ───────────────────────────────────────────────────────

    /// Initialise the scheduler.  Called once on Core 0 at boot.
    void Initialize();

    /// Core 1 entry point — permanent loop, never returns.
    /// Waits for FIFO signal, then processes tiles until pass is done.
    void Core1Main();

    // ── Frame Dispatch ──────────────────────────────────────────────────

    /// Begin a new tile-based rasterisation pass.
    ///
    /// Populates the shared pass context, signals Core 1 to start, then
    /// Core 0 also enters the tile loop.  Both cores steal tiles from the
    /// shared atomic counter until all 32 tiles are processed.
    ///
    /// @param rasterizer  Rasterizer instance (already prepared via PrepareFrame)
    /// @param fb          Framebuffer to write into
    /// @param zBuf        Z-buffer (pre-cleared or reused)
    /// @param w           Panel width (128)
    /// @param h           Panel height (64)
    /// @param idleFunc    Called by Core 0 while waiting for Core 1 to finish
    ///                    (e.g. Hub75Driver::PollRefresh)
    void DispatchTilePass(Rasterizer* rasterizer,
                          uint16_t* fb, float* zBuf,
                          uint16_t w, uint16_t h,
                          void (*idleFunc)());

    // ── Generic Job Dispatch (for shaders, etc.) ────────────────────────

    /// Submit a generic function to Core 1 and run another on Core 0.
    /// Lightweight alternative to the full tile pass for non-tile work
    /// (e.g. parallel shader bands).
    ///
    /// @param core0Func  Function to run on Core 0
    /// @param core0Ctx   Context for Core 0's function
    /// @param core1Func  Function to run on Core 1 (sent via FIFO)
    /// @param core1Ctx   Context for Core 1's function
    /// @param idleFunc   Called while waiting for Core 1
    void DispatchPair(void (*core0Func)(void*), void* core0Ctx,
                      void (*core1Func)(void*), void* core1Ctx,
                      void (*idleFunc)());

private:
    /// Shared pass context — lives in SRAM, pointer sent to Core 1 via FIFO.
    TilePassContext passCtx_;

    /// Process tiles from the shared counter until all are done.
    /// Called by both Core 0 and Core 1.
    static void ProcessTiles(TilePassContext* ctx);
};

