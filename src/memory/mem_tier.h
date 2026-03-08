/**
 * @file mem_tier.h
 * @brief Tiered Memory Manager — SRAM / QSPI-A / QSPI-B placement engine.
 *
 * The RP2350 GPU has three memory tiers with different characteristics:
 *
 *   Tier 0 — Internal SRAM (520 KB, 1-cycle, 32-bit bus)
 *       Fastest. For rasterization-critical data: Z-buffer, QuadTree,
 *       framebuffers, and the hottest textures/materials.
 *       Direct pointer access.
 *
 *   Tier 1 — QSPI Channel A (PIO2 SM0+SM1, indirect DMA)
 *       Up to 2 chips (CS0+CS1) on data GPIO 34-37, CLK 12.
 *       Auto-detected: MRAM (128 KB, 104 MHz) or PSRAM (8 MB, 133 MHz).
 *       NOT memory-mapped. All access through explicit DMA read/write.
 *       Good for bulk data prefetched into SRAM cache before use.
 *       MRAM: no random-access penalty, non-volatile.
 *       PSRAM: high bandwidth but row-buffer miss penalty.
 *       Indirect access: DMA → SRAM cache line → CPU reads cache.
 *
 *   Tier 2 — QSPI Channel B (PIO2 SM2+SM3, indirect DMA)
 *       Up to 2 chips (CS0+CS1) on data GPIO 39-42, CLK 43.
 *       Same auto-detection and access model as Channel A.
 *       Lower priority tier — resources placed here are colder.
 *       Mixed MRAM/PSRAM per channel is allowed.
 *       Ideal for: lookup tables (gamma/CIE), font atlases, material
 *       parameter banks, small textures (64×64 RGB565), keyframe data.
 *       Indirect access: DMA → SRAM cache line → CPU reads cache.
 *
 * ┌────────────────────────────────────────────────────────────────────────┐
 * │                     PLACEMENT POLICY                                  │
 * │                                                                       │
 * │  Each resource has two scores:                                        │
 * │    weight  — impact on rendering pipeline (0–255)                     │
 * │    score   — access frequency per frame (0–255, updated at runtime)   │
 * │                                                                       │
 * │  Combined priority = α·weight + β·score  (configurable)               │
 * │                                                                       │
 * │  Placement rules:                                                     │
 * │    ┌─────────┬─────────┬──────────────────────────────────────┐       │
 * │    │ Weight  │ Score   │ Tier Assignment                      │       │
 * │    ├─────────┼─────────┼──────────────────────────────────────┤       │
 * │    │ HIGH    │ HIGH    │ Tier 0 (SRAM) — hot critical path    │       │
 * │    │ HIGH    │ LOW     │ Tier 1 (QSPI-A+cache) — critical    │       │
 * │    │         │         │   but infrequent; DMA prefetch       │       │
 * │    │ LOW     │ HIGH    │ Tier 2 (QSPI-B+cache) — frequent    │       │
 * │    │         │         │   but non-critical; DMA prefetch     │       │
 * │    │ LOW     │ LOW     │ Tier 2 (QSPI-B) — cold storage      │       │
 * │    └─────────┴─────────┴──────────────────────────────────────┘       │
 * │                                                                       │
 * │  HARD CONSTRAINT: Data that participates in the rasterization inner   │
 * │  loop (Z-buffer, QuadTree, active framebuffer) is ALWAYS in SRAM.    │
 * │  The memory tier ONLY governs resource data (textures, materials,     │
 * │  mesh geometry, lookup tables).                                       │
 * └────────────────────────────────────────────────────────────────────────┘
 *
 * Promotion / Demotion:
 *   At the start of each frame (or every N frames), the memory manager
 *   re-evaluates resource placement based on updated scores.
 *   - Promotion: move resource from a slower tier to a faster tier
 *     (e.g., a texture that started being used by a draw call).
 *   - Demotion: move to a slower tier to free SRAM for hotter resources.
 *   - Eviction: drop from cache entirely if tier is full and resource is cold.
 *
 *   DMA prefetch for QSPI VRAM data is scheduled from the draw list:
 *   After command parsing, before rasterization, the manager scans the
 *   active draw list and prefetches any QSPI-A/B-resident textures/materials
 *   into SRAM cache lines via DMA (overlapped with QuadTree rebuild).
 */

#pragma once

#include <cstdint>
#include <cstring>
#include <cstdio>

// ─── Memory Tier Enum ───────────────────────────────────────────────────────

enum class MemTier : uint8_t {
    SRAM    = 0,     ///< Internal SRAM — single-cycle, direct pointer
    QSPI_A  = 1,     ///< QSPI Channel A (PIO2 SM0+SM1) — DMA indirect, cached in SRAM
    QSPI_B  = 2,     ///< QSPI Channel B (PIO2 SM2+SM3) — DMA indirect, cached in SRAM
    NONE    = 0xFF,

    // Legacy aliases
    OPI_PSRAM = QSPI_A,  ///< @deprecated Use QSPI_A
    QSPI_XIP  = QSPI_B,  ///< @deprecated Use QSPI_B
};

// ─── Resource Class (determines base weight) ────────────────────────────────

enum class ResClass : uint8_t {
    FRAMEBUFFER   = 0,   ///< weight=255, ALWAYS SRAM (hardcoded)
    Z_BUFFER      = 1,   ///< weight=255, ALWAYS SRAM (hardcoded)
    QUADTREE      = 2,   ///< weight=255, ALWAYS SRAM (hardcoded)
    VERTEX_DATA   = 3,   ///< weight=200, active mesh vertices
    INDEX_DATA    = 4,   ///< weight=200, active mesh indices
    TEXTURE       = 5,   ///< weight=128, image data
    MATERIAL_PARAM = 6,  ///< weight=160, material parameter blocks
    LAYOUT_COORDS = 7,   ///< weight=100, pixel layout coordinate arrays
    UV_DATA       = 8,   ///< weight=140, UV coordinates
    LOOKUP_TABLE  = 9,   ///< weight=60,  gamma/CIE/noise tables
    FONT_ATLAS    = 10,  ///< weight=40,  text rendering data
    COLD_MESH     = 11,  ///< weight=20,  inactive mesh storage
};

// ─── Resource Placement Record ──────────────────────────────────────────────

struct MemRecord {
    uint16_t  resourceId  = 0;       ///< ProtoGL resource handle
    ResClass  resClass    = ResClass::COLD_MESH;
    MemTier   currentTier = MemTier::NONE;

    uint8_t   weight      = 0;       ///< Pipeline impact (0–255). Higher = more critical.
    uint8_t   score       = 0;       ///< Access frequency (0–255). Updated per frame.

    /// Combined priority for sorting.  Higher = should be in faster tier.
    uint16_t  priority    = 0;

    // Pointers to the data in each tier (nullptr if not resident)
    void*     sramPtr     = nullptr;  ///< SRAM cache / primary copy
    uint32_t  qspiAAddr   = 0;        ///< QSPI Channel A byte address (0 = not allocated)
    uint32_t  qspiBAddr   = 0;        ///< QSPI Channel B byte address (0 = not allocated)

    uint32_t  dataSize    = 0;        ///< Size in bytes

    bool      dirty       = false;    ///< True if SRAM copy newer than external copy
    bool      pinned      = false;    ///< True if must stay in SRAM (rasterizer-critical)
    uint8_t   framesSinceAccess = 0;  ///< Frames since last access (for demotion)

    /// Recompute priority from weight + score.
    void UpdatePriority(uint8_t alpha = 3, uint8_t beta = 1) {
        priority = static_cast<uint16_t>(alpha) * weight
                 + static_cast<uint16_t>(beta)  * score;
    }
};

// ─── Configuration ──────────────────────────────────────────────────────────

struct MemTierConfig {
    // SRAM cache budget for resource tiering (separate from framebuf/ZBuf/QuadTree)
    uint32_t sramCacheBudget     = 64 * 1024;   ///< 64 KB SRAM reserved for tier cache

    // SRAM cache line size for PIO2 external memory prefetch
    uint32_t cacheLineSize       = 4096;         ///< 4 KB cache lines

    // QSPI Channel A total capacity (0 = not present)
    // Sum of detected chips on Channel A (up to 2 CS).
    uint32_t qspiACapacity       = 0;

    // QSPI Channel A chip characteristics (from auto-detect)
    bool     qspiAIsMram         = false;         ///< true when all Ch-A chips are MRAM
    bool     qspiAHasRandomAccessPenalty = true;   ///< false for all-MRAM, true if any PSRAM

    // QSPI Channel B total capacity (0 = not present)
    // Sum of detected chips on Channel B (up to 2 CS).
    uint32_t qspiBCapacity       = 0;

    // QSPI Channel B chip characteristics (from auto-detect)
    bool     qspiBIsMram         = false;         ///< true when all Ch-B chips are MRAM
    bool     qspiBHasRandomAccessPenalty = true;   ///< false for all-MRAM, true if any PSRAM
    bool     qspiBIsNonVolatile         = false;   ///< true for MRAM (data survives power)

    // Priority weighting factors
    uint8_t  alphaWeight         = 3;            ///< weight coefficient
    uint8_t  betaScore           = 1;            ///< score coefficient

    // Demotion threshold: frames since last access before demotion is considered
    uint8_t  demotionThreshold   = 30;           ///< ~0.5 sec at 60 FPS

    // Promotion hysteresis: priority must exceed tier threshold by this margin
    //   to avoid ping-pong between tiers
    uint16_t promotionHysteresis = 50;
};

// ─── SRAM Cache Line ────────────────────────────────────────────────────────

struct CacheLine {
    uint16_t  resourceId   = 0xFFFF;  ///< Which resource owns this line (0xFFFF = free)
    uint32_t  srcOffset    = 0;       ///< Offset into the resource's external data
    uint32_t  size         = 0;       ///< Bytes valid in this line
    uint8_t*  data         = nullptr; ///< Points into the SRAM cache arena
    bool      dirty        = false;   ///< Modified in SRAM; needs writeback
    bool      locked       = false;   ///< Currently in use by DMA or rasterizer
    uint8_t   lastUsedFrame = 0;      ///< Frame counter (wrapping) for LRU
};

// ─── Forward Declarations ───────────────────────────────────────────────────

class QspiVramDriver;   // Unified dual-channel QSPI VRAM driver (mem_qspi_vram.h)

// Legacy aliases
using OpiPsramDriver  = QspiVramDriver;
using QspiPsramDriver = QspiVramDriver;

// ─── Memory Tier Manager ────────────────────────────────────────────────────

class MemTierManager {
public:
    static constexpr uint16_t MAX_RECORDS    = 512;
    static constexpr uint16_t MAX_CACHE_LINES = 16;  // cache lines in SRAM arena

    MemTierManager() = default;

    bool Initialize(const MemTierConfig& config,
                    QspiVramDriver* vram = nullptr);

    // ── Resource Registration ───────────────────────────────────────────

    /// Register a resource with the memory manager.
    /// Returns the assigned MemRecord index, or 0xFFFF on failure.
    uint16_t Register(uint16_t resourceId, ResClass resClass, uint32_t dataSize);

    /// Unregister and free all tier storage for a resource.
    void Unregister(uint16_t resourceId);

    // ── Data Access ─────────────────────────────────────────────────────

    /// Get a readable pointer to resource data.
    /// If the resource is in SRAM, returns the pointer directly.
    /// If in QSPI-A or QSPI-B, triggers a DMA prefetch and returns nullptr
    /// (caller must call again after DMA completes, or use async callback).
    const void* Read(uint16_t resourceId);

    /// Get a writable pointer in SRAM (promotes to SRAM if needed).
    /// Marks the SRAM copy as dirty.
    void* Write(uint16_t resourceId);

    /// Record an access (increments score for this frame).
    void RecordAccess(uint16_t resourceId);

    // ── Frame Lifecycle ─────────────────────────────────────────────────

    /// Called at BeginFrame: decay scores, check for promotions/demotions.
    void BeginFrame();

    /// Called after command parsing: scan draw list and prefetch QSPI VRAM
    /// data into SRAM cache via DMA.  This overlaps with QuadTree rebuild.
    void PrefetchForDrawList(const uint16_t* meshIds, const uint16_t* materialIds,
                             const uint16_t* textureIds, uint16_t drawCallCount);

    /// Called after rasterization: flush dirty SRAM cache lines back to
    /// external memory if needed.  Unlock all cache lines.
    void EndFrame();

    // ── Diagnostics ─────────────────────────────────────────────────────

    void PrintStats() const;

    /// Get current SRAM cache utilization (bytes used / budget).
    uint32_t GetSramCacheUsed() const;

    /// Get tier assignment for a resource.
    MemTier GetTier(uint16_t resourceId) const;

    // ── Resource Record Access (used by command parser) ─────────────────

    /// Find a resource record by ID.  Returns nullptr if not found.
    MemRecord*       FindRecord(uint16_t resourceId);
    const MemRecord* FindRecord(uint16_t resourceId) const;

    /// Force-promote a resource to a faster tier (e.g., QSPI-A→SRAM).
    void Promote(MemRecord& rec, MemTier targetTier);

    /// Force-demote a resource to a slower tier (e.g., SRAM→QSPI-B).
    void Demote(MemRecord& rec, MemTier targetTier);

    // ── Defragmentation (M12) ───────────────────────────────────────────

    /**
     * @brief Compact free-space fragments within a memory tier.
     *
     * Walks all records residing in the target tier, sorts them by address,
     * and slides allocations downward to close gaps.  Data is moved via
     * memmove (SRAM) or DMA read-then-write (QSPI).
     *
     * @param tier       Target tier to defragment (SRAM, QSPI_A, or QSPI_B).
     *                   If MemTier::NONE, defragments all tiers sequentially.
     * @param mode       0 = incremental (move at most maxMoveKB, resume next call),
     *                   1 = urgent (compact fully in one blocking pass).
     * @param maxMoveKB  Budget for incremental mode (ignored if mode == 1).
     * @param[out] movedKB  Actual kilobytes relocated (may be less than budget).
     * @param[out] fragmentCount  Number of remaining free-space gaps after defrag.
     * @param[out] largestFreeKB  Largest contiguous free block after defrag.
     * @return true if defrag completed (no more gaps), false if work remains.
     */
    bool Defragment(MemTier tier, uint8_t mode, uint16_t maxMoveKB,
                    uint16_t& movedKB, uint16_t& fragmentCount,
                    uint16_t& largestFreeKB);

private:
    MemTierConfig    config_{};
    QspiVramDriver*  vram_  = nullptr;

    MemRecord   records_[MAX_RECORDS];
    uint16_t    recordCount_ = 0;

    CacheLine   cacheLines_[MAX_CACHE_LINES];
    uint8_t*    cacheArena_ = nullptr;  // SRAM cache arena (config_.sramCacheBudget)

    uint32_t    frameCounter_ = 0;

    // ── Internal helpers ────────────────────────────────────────────────

    MemTier  ComputeIdealTier(const MemRecord& rec) const;

    CacheLine* AllocCacheLine(uint16_t resourceId, uint32_t size);
    void       FreeCacheLine(CacheLine* line);
    CacheLine* FindLRUCacheLine();

    /// Assign base weight from resource class.
    /// When QSPI VRAM is MRAM (no random-access penalty), random-read resources
    /// get lower weights — making them candidates for QSPI-B demotion from SRAM.
    uint8_t BaseWeight(ResClass rc) const;
};

// ─── Inline: Chip-Type-Aware Base Weight Lookup ─────────────────────────────
//
// Two weight tables: one for PSRAM-like chips (conservative — random-access
// resources keep high weight to stay in SRAM), one for MRAM (aggressive —
// random-read resources get lower weight since MRAM handles scattered reads
// at full bus speed, freeing SRAM for rasterizer-critical data).
//
// Weight comparison (↓ = more likely to demote from SRAM to QSPI-B):
//
//   ResClass         PSRAM Weight   MRAM Weight   Rationale
//   ─────────────    ────────────   ───────────   ──────────────────────────
//   FRAMEBUFFER      255 (pinned)   255 (pinned)  Always SRAM
//   Z_BUFFER         255 (pinned)   255 (pinned)  Always SRAM
//   QUADTREE         255 (pinned)   255 (pinned)  Always SRAM
//   VERTEX_DATA      200            200            Sequential pipeline access
//   INDEX_DATA       200            200            Sequential pipeline access
//   TEXTURE          128            100 ↓          MRAM handles texel sampling
//   MATERIAL_PARAM   160            80  ↓          Random param reads excel
//   LAYOUT_COORDS    100            50  ↓          Per-pixel read pattern
//   UV_DATA          140            140            Tightly coupled to vertices
//   LOOKUP_TABLE     60             30  ↓          Perfect MRAM fit
//   FONT_ATLAS       40             20  ↓          Persistent, read-only
//   COLD_MESH        20             10  ↓          MRAM non-volatile = no reload

inline uint8_t MemTierManager::BaseWeight(ResClass rc) const {
    // ── Conservative weights (PSRAM or no external QSPI) ────────────────
    // Random-access resources keep high weight → stay in SRAM longer
    // because PSRAM has row-buffer miss penalty on scattered reads.
    static constexpr uint8_t psramWeights[] = {
        255, 255, 255,   // FRAMEBUFFER, Z_BUFFER, QUADTREE (pinned)
        200, 200,        // VERTEX_DATA, INDEX_DATA
        128,             // TEXTURE
        160,             // MATERIAL_PARAM
        100,             // LAYOUT_COORDS
        140,             // UV_DATA
        60,              // LOOKUP_TABLE
        40,              // FONT_ATLAS
        20,              // COLD_MESH
    };

    // ── MRAM-optimized weights ──────────────────────────────────────────
    // MRAM has NO random-access penalty:  scattered reads run at full bus
    // speed (52 MB/s @ 104 MHz).  Resources with random access patterns
    // (LUTs, material params, textures during sampling) get significantly
    // lower weights, making them candidates for QSPI demotion.  This frees
    // SRAM for rasterizer-critical data (vertices, indices, Z-buffer).
    //
    // Pinned resources (FB, Z, QT) and sequential-access data (vertices,
    // indices, UVs) keep the same weights — they need SRAM's single-cycle
    // latency regardless of external chip type.
    static constexpr uint8_t mramWeights[] = {
        255, 255, 255,   // FRAMEBUFFER, Z_BUFFER, QUADTREE (pinned, unchanged)
        200, 200,        // VERTEX_DATA, INDEX_DATA (sequential → keep in SRAM)
        100,             // TEXTURE (↓28: MRAM handles scattered texel reads)
        80,              // MATERIAL_PARAM (↓80: random param lookups excel in MRAM)
        50,              // LAYOUT_COORDS (↓50: per-pixel reads suit MRAM)
        140,             // UV_DATA (unchanged: tightly coupled to vertex pipeline)
        30,              // LOOKUP_TABLE (↓30: perfect MRAM candidate)
        20,              // FONT_ATLAS (↓20: persistent in MRAM, read-only)
        10,              // COLD_MESH (↓10: MRAM non-volatile = zero reload cost)
    };

    uint8_t idx = static_cast<uint8_t>(rc);
    if (idx >= 12) return 0;

    // Select weight table based on detected QSPI VRAM chip characteristics.
    // If either channel has random-access penalty (PSRAM), use conservative
    // weights.  If all detected chips are MRAM, use aggressive weights.
    bool anyPenalty = config_.qspiAHasRandomAccessPenalty
                   || config_.qspiBHasRandomAccessPenalty;
    return anyPenalty ? psramWeights[idx] : mramWeights[idx];
}
