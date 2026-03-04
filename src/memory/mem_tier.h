/**
 * @file mem_tier.h
 * @brief Tiered Memory Manager — SRAM / OPI PSRAM / QSPI PSRAM placement engine.
 *
 * The RP2350 GPU has three memory tiers with different characteristics:
 *
 *   Tier 0 — Internal SRAM (520 KB, 1-cycle, 32-bit bus)
 *       Fastest. For rasterization-critical data: Z-buffer, QuadTree,
 *       framebuffers, and the hottest textures/materials.
 *       Direct pointer access.
 *
 *   Tier 1 — OPI PSRAM via PIO2 (8–16 MB, ~150 MB/s burst, indirect)
 *       High bandwidth, but NOT memory-mapped. All access goes through
 *       explicit DMA read/write commands. Good for bulk data that can be
 *       prefetched into an SRAM cache before use. Textures, large meshes,
 *       cold material parameter banks.
 *       Indirect access: DMA → SRAM cache line → CPU reads cache.
 *
 *   Tier 2 — QSPI PSRAM via QMI CS1 (4–16 MB, ~75 MB/s, XIP cached)
 *       Memory-mapped with hardware XIP cache. Transparent to the CPU
 *       (reads look like normal memory loads). Lower burst bandwidth than
 *       OPI but better random-access latency due to hardware cache.
 *       Frequently-read, non-pipeline-critical data: lookup tables,
 *       font atlases, inactive meshes, animation keyframes.
 *       Direct access: XIP-mapped pointer (cached by QMI hardware).
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
 * │    │ HIGH    │ LOW     │ Tier 1 (OPI+cache) — critical but    │       │
 * │    │         │         │   infrequent; DMA prefetch before use│       │
 * │    │ LOW     │ HIGH    │ Tier 2 (QSPI XIP) — frequent reads  │       │
 * │    │         │         │   but non-critical; HW cache handles │       │
 * │    │ LOW     │ LOW     │ Tier 2 (QSPI XIP) — cold storage    │       │
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
 *   DMA prefetch for OPI data is scheduled from the draw list:
 *   After command parsing, before rasterization, the manager scans the
 *   active draw list and prefetches any OPI-resident textures/materials
 *   into SRAM cache lines via DMA (overlapped with QuadTree rebuild).
 */

#pragma once

#include <cstdint>
#include <cstring>
#include <cstdio>

// ─── Memory Tier Enum ───────────────────────────────────────────────────────

enum class MemTier : uint8_t {
    SRAM     = 0,   ///< Internal SRAM — single-cycle, direct pointer
    OPI_PSRAM = 1,  ///< OPI PSRAM via PIO2 — DMA indirect, cached in SRAM
    QSPI_XIP  = 2,  ///< QSPI PSRAM via QMI CS1 — XIP memory-mapped, HW cache
    NONE      = 0xFF
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
    uint32_t  opiAddr     = 0;        ///< OPI PSRAM byte address (0 = not allocated)
    void*     qspiPtr     = nullptr;  ///< QSPI XIP mapped pointer (nullptr = not allocated)

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

    // SRAM cache line size for OPI prefetch
    uint32_t cacheLineSize       = 4096;         ///< 4 KB cache lines

    // OPI PSRAM total capacity (0 = not present)
    uint32_t opiCapacity         = 0;            ///< e.g., 8 * 1024 * 1024 = 8 MB

    // QSPI PSRAM total capacity (0 = not present)
    uint32_t qspiCapacity        = 0;            ///< e.g., 8 * 1024 * 1024 = 8 MB

    // QSPI XIP base address (RP2350: 0x11000000 for CS1)
    uintptr_t qspiXipBase        = 0x11000000;

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

class OpiPsramDriver;   // PIO2-based OPI PSRAM driver (mem_opi_psram.h)
class QspiPsramDriver;  // QMI CS1 QSPI PSRAM driver (mem_qspi_psram.h)

// ─── Memory Tier Manager ────────────────────────────────────────────────────

class MemTierManager {
public:
    static constexpr uint16_t MAX_RECORDS    = 512;
    static constexpr uint16_t MAX_CACHE_LINES = 16;  // cache lines in SRAM arena

    MemTierManager() = default;

    bool Initialize(const MemTierConfig& config,
                    OpiPsramDriver* opi = nullptr,
                    QspiPsramDriver* qspi = nullptr);

    // ── Resource Registration ───────────────────────────────────────────

    /// Register a resource with the memory manager.
    /// Returns the assigned MemRecord index, or 0xFFFF on failure.
    uint16_t Register(uint16_t resourceId, ResClass resClass, uint32_t dataSize);

    /// Unregister and free all tier storage for a resource.
    void Unregister(uint16_t resourceId);

    // ── Data Access ─────────────────────────────────────────────────────

    /// Get a readable pointer to resource data.
    /// If the resource is in SRAM, returns the pointer directly.
    /// If in OPI PSRAM, triggers a DMA prefetch and returns nullptr (caller
    /// must call again after DMA completes, or use the async callback).
    /// If in QSPI XIP, returns the XIP-mapped pointer (transparent cache).
    const void* Read(uint16_t resourceId);

    /// Get a writable pointer in SRAM (promotes to SRAM if needed).
    /// Marks the SRAM copy as dirty.
    void* Write(uint16_t resourceId);

    /// Record an access (increments score for this frame).
    void RecordAccess(uint16_t resourceId);

    // ── Frame Lifecycle ─────────────────────────────────────────────────

    /// Called at BeginFrame: decay scores, check for promotions/demotions.
    void BeginFrame();

    /// Called after command parsing: scan draw list and prefetch OPI data
    /// into SRAM cache via DMA.  This overlaps with QuadTree rebuild.
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

private:
    MemTierConfig   config_{};
    OpiPsramDriver* opi_   = nullptr;
    QspiPsramDriver* qspi_ = nullptr;

    MemRecord   records_[MAX_RECORDS];
    uint16_t    recordCount_ = 0;

    CacheLine   cacheLines_[MAX_CACHE_LINES];
    uint8_t*    cacheArena_ = nullptr;  // SRAM cache arena (config_.sramCacheBudget)

    uint32_t    frameCounter_ = 0;

    // ── Internal helpers ────────────────────────────────────────────────

    MemRecord*       FindRecord(uint16_t resourceId);
    const MemRecord* FindRecord(uint16_t resourceId) const;

    MemTier  ComputeIdealTier(const MemRecord& rec) const;
    void     Promote(MemRecord& rec, MemTier targetTier);
    void     Demote(MemRecord& rec, MemTier targetTier);

    CacheLine* AllocCacheLine(uint16_t resourceId, uint32_t size);
    void       FreeCacheLine(CacheLine* line);
    CacheLine* FindLRUCacheLine();

    /// Assign base weight from resource class.
    static uint8_t BaseWeight(ResClass rc);
};

// ─── Inline: Base Weight Lookup ─────────────────────────────────────────────

inline uint8_t MemTierManager::BaseWeight(ResClass rc) {
    switch (rc) {
        case ResClass::FRAMEBUFFER:    return 255;
        case ResClass::Z_BUFFER:       return 255;
        case ResClass::QUADTREE:       return 255;
        case ResClass::VERTEX_DATA:    return 200;
        case ResClass::INDEX_DATA:     return 200;
        case ResClass::MATERIAL_PARAM: return 160;
        case ResClass::UV_DATA:        return 140;
        case ResClass::TEXTURE:        return 128;
        case ResClass::LAYOUT_COORDS:  return 100;
        case ResClass::LOOKUP_TABLE:   return 60;
        case ResClass::FONT_ATLAS:     return 40;
        case ResClass::COLD_MESH:      return 20;
        default:                       return 0;
    }
}

#endif // — end of header guard is handled by #pragma once above
