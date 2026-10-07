/**
 * @file mem_assets.h
 * @brief Bounded typed asset service over an external byte store (P11).
 *
 * One AssetService instance owns ONE backing store (an AssetBacking ops
 * table) and ONE caller-provided SRAM staging arena. It provides:
 *
 *   - Typed asset identity: AssetHandle = {slot index, AssetClass,
 *     generation}. A stale handle (old generation) or a handle whose class
 *     does not match the slot is rejected with InvalidHandle — no ID-class
 *     collision, no ABA on slot reuse within the 256-generation window.
 *   - Finite backing allocation: a bounded, coalescing free-range allocator
 *     over the store's byte offsets. Offset 0 is a VALID store address.
 *     Allocation failure is explicit (NoStoreCapacity), never silent.
 *   - Complete-range reads/writes: every transfer either moves exactly the
 *     requested bytes or fails; bounds are checked against the asset size
 *     and the store capacity before the backing is touched.
 *   - Lease-aware SRAM spans: prefetchSpan() stages an asset range into the
 *     SRAM arena and pins it (lease). Leased spans can never be evicted,
 *     migrated or destroyed underneath the caller. Released spans stay
 *     cached and may be evicted (dirty ones are written back first) or
 *     moved by compactStaging().
 *
 * Absent-device semantics: binding a zero-capacity (or null-ops) backing is
 * legal; every asset operation then reports NoBacking, capacityBytes() is 0,
 * and no resources are held. The SRAM-only profile simply never binds a
 * backing and is unaffected.
 *
 * Threading: not internally synchronized. The firmware calls it from the
 * scheduler's resource-critical section (core0 only); the backing driver
 * (mem_qmi_psram) enforces single-owner init separately.
 *
 * This service performs NO platform allocation and NO per-sample copies:
 * all metadata is fixed-size static tables; staging spans are explicit,
 * caller-requested and bounded.
 */

#pragma once

#include <cstddef>
#include <cstdint>

namespace gpumem {

// ─── Bounded capacities (parent may override in gpu_config.h) ───────────────

#ifndef GPU_MEM_MAX_ASSETS
/// Maximum simultaneous assets (meshes + textures + others). Covers the
/// published metadata profile (64 meshes / 16 textures) with headroom.
#define GPU_MEM_MAX_ASSETS 96u
#endif

#ifndef GPU_MEM_MAX_SPANS
/// Maximum SRAM span records (leased + cached) at any time.
#define GPU_MEM_MAX_SPANS 32u
#endif

#ifndef GPU_MEM_ASSET_ALIGN_BYTES
/// Backing-store allocation granularity (also QMI burst-friendly).
#define GPU_MEM_ASSET_ALIGN_BYTES 16u
#endif

#ifndef GPU_MEM_SPAN_ALIGN_BYTES
/// SRAM staging span alignment.
#define GPU_MEM_SPAN_ALIGN_BYTES 16u
#endif

static_assert(GPU_MEM_MAX_ASSETS >= 1, "need at least one asset slot");
static_assert(GPU_MEM_MAX_SPANS >= 1, "need at least one span record");
// Handles/records index with uint16_t; 0xFFFF is the invalid-handle sentinel.
static_assert(GPU_MEM_MAX_ASSETS <= 0xFFFEu, "asset slots must fit uint16_t");
static_assert(GPU_MEM_MAX_SPANS <= 0xFFFEu, "span records must fit uint16_t");
static_assert((GPU_MEM_ASSET_ALIGN_BYTES & (GPU_MEM_ASSET_ALIGN_BYTES - 1)) == 0,
              "asset alignment must be a power of two");
static_assert((GPU_MEM_SPAN_ALIGN_BYTES & (GPU_MEM_SPAN_ALIGN_BYTES - 1)) == 0,
              "span alignment must be a power of two");

// ─── Public types ───────────────────────────────────────────────────────────

enum class AssetClass : uint8_t {
    // Numbering matches the packed scene-token classes used by the parent's
    // MeshSlot/TextureSlot: Mesh=0, Material=1, Texture=2.
    Mesh        = 0,
    Material    = 1,
    Texture     = 2,
    GlyphAtlas  = 3,
    LookupTable = 4,
};

struct AssetHandle {
    uint16_t index      = 0xFFFF;   ///< slot index; 0xFFFF = invalid handle
    uint8_t  assetClass = 0xFF;     ///< AssetClass
    uint8_t  generation = 0;        ///< per-slot generation

    bool isValid() const { return index != 0xFFFF; }
};
/// Opaque 32-bit token form of AssetHandle for scene slots (one word per
/// mesh/texture slot). Layout: [31:16]=slot index, [15:8]=AssetClass,
/// [7:0]=generation. kAssetTokenInvalid is the empty slot; token 0 is a
/// VALID handle (slot 0, Mesh, generation 0).
using AssetToken = uint32_t;
inline constexpr AssetToken kAssetTokenInvalid = 0xFFFFFFFFu;

inline constexpr AssetToken assetTokenFromHandle(const AssetHandle& h) {
    return (static_cast<uint32_t>(h.index) << 16) |
           (static_cast<uint32_t>(h.assetClass) << 8) |
           static_cast<uint32_t>(h.generation);
}

inline constexpr AssetHandle assetHandleFromToken(AssetToken token) {
    AssetHandle h;
    h.index      = static_cast<uint16_t>((token >> 16) & 0xFFFFu);
    h.assetClass = static_cast<uint8_t>((token >> 8) & 0xFFu);
    h.generation = static_cast<uint8_t>(token & 0xFFu);
    return h;
}

enum class AssetStatus : uint8_t {
    Ok = 0,
    NoBacking,        ///< no usable backing store (absent device / capacity 0)
    NoSlots,          ///< asset slot table full (GPU_MEM_MAX_ASSETS)
    NoStoreCapacity,  ///< backing store exhausted or fragmented
    InvalidHandle,    ///< stale generation, wrong class, or inactive slot
    InvalidRange,     ///< offset/bytes outside the asset or zero-length
    StagingFull,      ///< SRAM staging cannot fit the span; nothing evictable
    LeaseActive,      ///< destroy refused: asset has live span leases
    BackingError,     ///< backing transfer failed
    BadArgument,      ///< null pointer / nonsense argument
};

/// Backing store ops. offset 0 is a valid store address. The service checks
/// all bounds before calling, so implementations may assume
/// offset + bytes <= capacityBytes and bytes > 0. Implementations must
/// transfer the complete range or return false; partial success is not
/// representable.
struct AssetBacking {
    void* ctx = nullptr;
    uint32_t capacityBytes = 0;
    bool (*read)(void* ctx, uint32_t offset, void* dst, uint32_t bytes) = nullptr;
    bool (*write)(void* ctx, uint32_t offset, const void* src, uint32_t bytes) = nullptr;
};

/// A leased (or cached) SRAM staging span. `data` is valid only while the
/// caller holds the lease (between prefetchSpan() and releaseSpan()).
struct AssetSpan {
    AssetHandle handle;
    uint8_t*    data   = nullptr;  ///< SRAM staging pointer
    uint32_t    offset = 0;        ///< asset-relative byte offset mirrored here
    uint32_t    bytes  = 0;
};

struct AssetInfo {
    AssetClass assetClass = AssetClass::Mesh;
    uint32_t   bytes      = 0;
};

struct AssetServiceStats {
    uint32_t storeCapacityBytes      = 0;
    uint32_t storeFreeBytes          = 0;
    uint32_t storeLargestFreeBytes   = 0;
    uint16_t assetCount              = 0;
    uint16_t spanCount               = 0;  ///< active span records (leased + cached)
    uint16_t leasedSpanCount         = 0;
    uint32_t stagingBytes            = 0;
    uint32_t stagingFreeBytes        = 0;
    uint32_t stagingLargestFreeBytes = 0;
};

// ─── Bounded coalescing free-range allocator (backing + staging share it) ───
//
// Sorted intrusive free-range list over a fixed node pool. No platform
// allocation, fully deterministic, exact accounting. Free nodes are bounded:
// a 1-D free list never needs more free ranges than allocations + 1; the
// pools below carry twice that. Offset 0 is a valid allocation result —
// success is reported by the return value, not the offset.
//
// @tparam MaxRanges  size of the free-range node pool.
template <size_t MaxRanges>
class RangeAllocator {
public:
    static_assert(MaxRanges >= 2, "pool must hold at least two ranges");
    static_assert(MaxRanges <= 0xFFFE, "range indices are uint16_t");

    void reset(uint32_t capacityBytes) {
        mCapacity = capacityBytes;
        mFreeBytes = 0;
        mHead = kNil;
        mFreeStack = kNil;
        for (uint16_t i = 0; i < static_cast<uint16_t>(MaxRanges); ++i) {
            recycleNode(i);
        }
        if (capacityBytes > 0) {
            const uint16_t idx = takeNode();
            mNodes[idx].offset = 0;  // offset 0 is valid
            mNodes[idx].bytes = capacityBytes;
            mNodes[idx].next = kNil;
            mHead = idx;
            mFreeBytes = capacityBytes;
        }
    }

    uint32_t capacityBytes() const { return mCapacity; }
    uint32_t freeBytes() const { return mFreeBytes; }

    uint32_t largestFree() const {
        uint32_t largest = 0;
        for (uint16_t idx = mHead; idx != kNil; idx = mNodes[idx].next) {
            if (mNodes[idx].bytes > largest) largest = mNodes[idx].bytes;
        }
        return largest;
    }

    /// First-fit with alignment. Carves exactly `bytes` at an aligned offset.
    /// Returns false (state unchanged) when nothing fits or the bounded node
    /// pool is exhausted.
    bool allocate(uint32_t bytes, uint32_t alignment, uint32_t& outOffset) {
        if (bytes == 0 || bytes > mFreeBytes) return false;
        uint16_t prevIdx = kNil;
        uint16_t idx = mHead;
        while (idx != kNil) {
            Node& n = mNodes[idx];
            const uint32_t aligned = alignUp(n.offset, alignment);
            if (aligned >= n.offset && aligned - n.offset <= n.bytes) {
                const uint32_t prefix = aligned - n.offset;
                if (bytes <= n.bytes - prefix) {
                    const uint32_t suffix = n.bytes - prefix - bytes;
                    // A three-way carve needs one spare node; if the bounded
                    // pool cannot supply it, keep scanning other ranges.
                    const bool needsExtraNode = (prefix > 0 && suffix > 0);
                    if (!needsExtraNode || mFreeStack != kNil) {
                        outOffset = aligned;
                        mFreeBytes -= bytes;
                        carve(idx, prevIdx, prefix, suffix);
                        return true;
                    }
                }
            }
            prevIdx = idx;
            idx = n.next;
        }
        return false;
    }

    /// Allocate an exact offset/bytes (used when rebuilding a known layout).
    bool allocateAt(uint32_t offset, uint32_t bytes) {
        if (bytes == 0 || bytes > mFreeBytes) return false;
        uint16_t prevIdx = kNil;
        uint16_t idx = mHead;
        while (idx != kNil && mNodes[idx].offset <= offset) {
            Node& n = mNodes[idx];
            const uint32_t prefix = offset - n.offset;
            if (prefix <= n.bytes && bytes <= n.bytes - prefix) {
                const uint32_t suffix = n.bytes - prefix - bytes;
                if (prefix > 0 && suffix > 0 && mFreeStack == kNil) {
                    return false;
                }
                mFreeBytes -= bytes;
                carve(idx, prevIdx, prefix, suffix);
                return true;
            }
            prevIdx = idx;
            idx = n.next;
        }
        return false;
    }

    /// Return a previously allocated range; coalesces with free neighbours.
    /// Caller guarantees [offset, offset+bytes) is currently allocated.
    /// Returns false (range NOT freed, leak visible in stats) only if the
    /// bounded node pool is exhausted — unreachable with the pool sizing
    /// used by AssetService.
    bool release(uint32_t offset, uint32_t bytes) {
        if (bytes == 0) return false;
        uint16_t prevIdx = kNil;
        uint16_t idx = mHead;
        while (idx != kNil && mNodes[idx].offset < offset) {
            prevIdx = idx;
            idx = mNodes[idx].next;
        }
        // Merge into the previous range when adjacent.
        if (prevIdx != kNil &&
            mNodes[prevIdx].offset + mNodes[prevIdx].bytes == offset) {
            mNodes[prevIdx].bytes += bytes;
            mFreeBytes += bytes;
            if (idx != kNil &&
                mNodes[prevIdx].offset + mNodes[prevIdx].bytes == mNodes[idx].offset) {
                mNodes[prevIdx].bytes += mNodes[idx].bytes;
                mNodes[prevIdx].next = mNodes[idx].next;
                recycleNode(idx);
            }
            return true;
        }
        // Merge into the next range when adjacent.
        if (idx != kNil && offset + bytes == mNodes[idx].offset) {
            mNodes[idx].offset = offset;
            mNodes[idx].bytes += bytes;
            mFreeBytes += bytes;
            return true;
        }
        const uint16_t fresh = takeNode();
        if (fresh == kNil) return false;  // bounded pool exhausted
        mNodes[fresh].offset = offset;
        mNodes[fresh].bytes = bytes;
        mNodes[fresh].next = idx;
        if (prevIdx != kNil) mNodes[prevIdx].next = fresh;
        else mHead = fresh;
        mFreeBytes += bytes;
        return true;
    }

private:
    static constexpr uint16_t kNil = 0xFFFF;

    struct Node {
        uint32_t offset;
        uint32_t bytes;
        uint16_t next;
    };

    static uint32_t alignUp(uint32_t value, uint32_t alignment) {
        return (value + alignment - 1u) & ~(alignment - 1u);
    }

    uint16_t takeNode() {
        const uint16_t idx = mFreeStack;
        if (idx != kNil) mFreeStack = mNodes[idx].next;
        return idx;
    }

    void recycleNode(uint16_t idx) {
        mNodes[idx].next = mFreeStack;
        mFreeStack = idx;
    }

    /// Carve the allocation out of node idx, leaving `prefix` free bytes
    /// before and `suffix` free bytes after it.
    void carve(uint16_t idx, uint16_t prevIdx, uint32_t prefix, uint32_t suffix) {
        Node& n = mNodes[idx];
        const uint32_t aligned = n.offset + prefix;
        const uint32_t bytes = n.bytes - prefix - suffix;
        if (prefix == 0 && suffix == 0) {
            if (prevIdx != kNil) mNodes[prevIdx].next = n.next;
            else mHead = n.next;
            recycleNode(idx);
        } else if (prefix == 0) {
            n.offset = aligned + bytes;
            n.bytes = suffix;
        } else if (suffix == 0) {
            n.bytes = prefix;
        } else {
            const uint16_t extra = takeNode();  // availability pre-checked
            mNodes[extra].offset = aligned + bytes;
            mNodes[extra].bytes = suffix;
            mNodes[extra].next = n.next;
            n.next = extra;
            n.bytes = prefix;
        }
    }

    Node mNodes[MaxRanges];
    uint32_t mCapacity = 0;
    uint32_t mFreeBytes = 0;
    uint16_t mHead = kNil;
    uint16_t mFreeStack = kNil;
};

// ─── AssetService ───────────────────────────────────────────────────────────

class AssetService {
public:
    AssetService() = default;

    /// Bind the backing store and the SRAM staging arena. Call once at init.
    /// Rebinding first drains the current state exactly like reset()
    /// (LeaseActive / BackingError leave everything unchanged), then swaps.
    /// A zero-capacity or null-ops backing is legal and yields NoBacking
    /// semantics; a null arena with stagingBytes > 0 is a BadArgument
    /// (no state change).
    AssetStatus bind(const AssetBacking& backing, void* stagingArena, size_t stagingBytes);

    /// Drop all assets and spans and rebuild the free lists. The caller MUST
    /// have released every lease first: with any live lease this returns
    /// LeaseActive and changes nothing. Dirty cached spans are written back
    /// to the backing first; a failed write-back returns BackingError with
    /// all state (including the dirty span) intact — dirty data is never
    /// silently discarded. Quarantined slots (generation-exhausted) are
    /// re-enabled with generation 0: reset() is an epoch boundary and any
    /// pre-reset handle invalidation beyond that is the caller's session
    /// scope. Outstanding AssetSpan pointers become invalid.
    AssetStatus reset();

    bool     hasBacking() const;
    uint32_t capacityBytes() const { return mBacking.capacityBytes; }

    // ─── Asset lifecycle ────────────────────────────────────────────────

    /// Allocate `bytes` in the backing store and return a typed handle.
    /// The backing content is undefined until written via write() or a
    /// dirty span flush.
    AssetStatus create(AssetClass assetClass, uint32_t bytes, AssetHandle& out);

    /// Free the backing range. Refused with LeaseActive while any span of
    /// this asset is leased. Cached (unleased) spans are dropped (the asset's
    /// content dies with it). On success the slot's generation advances,
    /// invalidating all previous handles; when the 8-bit generation would
    /// wrap, the slot is QUARANTINED until reset() instead, so a stale token
    /// can never alias a recycled slot within an epoch.
    AssetStatus destroy(const AssetHandle& handle);

    /// Complete-range transfers between the backing store and caller SRAM
    /// buffers. [assetOffset, assetOffset+bytes) must lie inside the asset.
    AssetStatus write(const AssetHandle& handle, uint32_t assetOffset,
                      const void* src, uint32_t bytes);
    AssetStatus read(const AssetHandle& handle, uint32_t assetOffset,
                     void* dst, uint32_t bytes) const;

    AssetStatus info(const AssetHandle& handle, AssetInfo& out) const;

    // ─── Lease-aware SRAM staging ───────────────────────────────────────────

    /// Stage [assetOffset, assetOffset+bytes) of the asset into the SRAM
    /// arena and lease it. Identical existing spans are shared (lease count
    /// incremented); every successful call requires its own releaseSpan().
    /// Release an existing lease before reusing its output descriptor.
    /// Cached spans may be evicted to reclaim either aligned space or span
    /// records. StagingFull means live leases prevent admission; BackingError
    /// means a read or required dirty write-back failed. Failed write-back
    /// keeps the cached span and its dirty contents intact.
    AssetStatus prefetchSpan(const AssetHandle& handle, uint32_t assetOffset,
                             uint32_t bytes, AssetSpan& out);

    /// Write a dirty span back to the backing store (Ok and no-op when not
    /// dirty). Works on leased and cached spans.
    AssetStatus flushSpan(const AssetSpan& span);

    /// Release one lease. Pass dirty=true when the caller modified the span
    /// contents; dirty spans are written back before any later eviction.
    /// span.data is cleared on success; using it afterwards is a caller bug.
    AssetStatus releaseSpan(AssetSpan& span, bool dirty);

    /// Evict unleased cached spans (flushing dirty ones) until the staging
    /// arena's largest free run is at least minBytes or nothing evictable
    /// remains. Leased spans are NEVER evicted. Returns Ok when the goal was
    /// met, StagingFull when live leases prevented it, BackingError when a
    /// write-back failed (no data lost; the span stays cached+dirty).
    /// freedBytes receives only newly freed span bytes, not pre-existing free
    /// space, including when the operation stops with an error.
    AssetStatus evictCached(uint32_t minBytes, uint32_t* freedBytes);

    /// Move unleased cached spans toward the front of the staging arena to
    /// coalesce free space. Leased spans are never moved (their pointers
    /// stay valid). Returns the number of spans relocated.
    uint32_t compactStaging();

    AssetServiceStats stats() const;

private:
    struct Slot {
        uint32_t offset = 0;       ///< backing store byte offset (0 valid)
        uint32_t bytes  = 0;
        uint8_t  assetClass = 0;
        uint8_t  generation = 0;
        uint8_t  active = 0;
        /// Generation exhausted (would wrap): slot is quarantined — never
        /// reused, so no stale token can alias — until reset() re-enables it.
        uint8_t  quarantined = 0;
    };

    struct SpanRecord {
        uint32_t assetOffset   = 0;
        uint32_t bytes         = 0;
        uint32_t stagingOffset = 0;
        uint16_t assetIndex    = 0;
        uint16_t leaseCount    = 0;  ///< >0 pinned; 0 cached/evictable
        uint8_t  active        = 0;
        uint8_t  dirty         = 0;
    };

    static constexpr size_t kStoreRangePool = 2u * GPU_MEM_MAX_ASSETS + 1u;
    static constexpr size_t kStagingRangePool = GPU_MEM_MAX_SPANS + 2u;

    const Slot* resolve(const AssetHandle& handle) const;
    Slot* resolve(const AssetHandle& handle);
    SpanRecord* findSpan(const AssetSpan& span);
    const SpanRecord* findSpan(const AssetSpan& span) const;
    SpanRecord* findSpanRecord(const AssetHandle& handle, uint32_t assetOffset,
                               uint32_t bytes);
    SpanRecord* freeSpanRecord();
    AssetStatus evictOneCached();    ///< flush+drop one unleased span or report why not
    AssetStatus drainAndFlush();  ///< LeaseActive if leased; write-back of dirty cached spans
    void clearTables();           ///< unconditional table/free-list rebuild

    AssetBacking mBacking;
    uint8_t*  mStaging = nullptr;
    uint32_t  mStagingBytes = 0;
    uint16_t  mAssetCount = 0;

    Slot mSlots[GPU_MEM_MAX_ASSETS];
    SpanRecord mSpans[GPU_MEM_MAX_SPANS];
    RangeAllocator<kStoreRangePool> mStoreRanges;
    RangeAllocator<kStagingRangePool> mStagingRanges;
};

} // namespace gpumem
