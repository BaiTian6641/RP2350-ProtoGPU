/**
 * @file mem_assets.cpp
 * @brief AssetService implementation — see mem_assets.h for the contract.
 *
 * No platform allocation, no hardware access: the backing store is an ops
 * table (mem_qmi_psram on target, a plain SRAM span in native tests), so the
 * exact same allocator/lease algorithm runs in both.
 */

#include "mem_assets.h"

#include <cstring>

namespace gpumem {

namespace {

inline uint32_t alignUpU32(uint32_t value, uint32_t alignment) {
    return (value + alignment - 1u) & ~(alignment - 1u);
}

} // namespace

// ─── Setup ──────────────────────────────────────────────────────────────────

AssetStatus AssetService::bind(const AssetBacking& backing, void* stagingArena,
                               size_t stagingBytes) {
    if (stagingBytes > 0 && !stagingArena) return AssetStatus::BadArgument;
    if (stagingBytes > 0xFFFFFFFFu) return AssetStatus::BadArgument;

    // Rebinding drains the current state first (leases must be released,
    // dirty cached spans written back to the CURRENT backing).
    const AssetStatus drained = drainAndFlush();
    if (drained != AssetStatus::Ok) return drained;

    mBacking = backing;
    if (!mBacking.read || !mBacking.write) {
        mBacking.read = nullptr;
        mBacking.write = nullptr;
        mBacking.capacityBytes = 0;  // null ops => no usable backing
    }
    mStaging = static_cast<uint8_t*>(stagingArena);
    mStagingBytes = mStaging ? static_cast<uint32_t>(stagingBytes) : 0;
    clearTables();
    return AssetStatus::Ok;
}

AssetStatus AssetService::drainAndFlush() {
    for (size_t i = 0; i < GPU_MEM_MAX_SPANS; ++i) {
        if (mSpans[i].active && mSpans[i].leaseCount > 0) {
            return AssetStatus::LeaseActive;  // caller must release leases first
        }
    }
    if (hasBacking()) {
        for (size_t i = 0; i < GPU_MEM_MAX_SPANS; ++i) {
            SpanRecord& rec = mSpans[i];
            if (!rec.active || !rec.dirty) continue;
            const Slot& slot = mSlots[rec.assetIndex];
            if (!mBacking.write(mBacking.ctx, slot.offset + rec.assetOffset,
                                mStaging + rec.stagingOffset, rec.bytes)) {
                return AssetStatus::BackingError;  // dirty span kept, nothing lost
            }
            rec.dirty = 0;
        }
    }
    return AssetStatus::Ok;
}

void AssetService::clearTables() {
    // Invalidate every outstanding handle: advance the generation of any
    // still-active slot (inactive slots were already bumped by destroy()).
    // Quarantined slots are re-enabled at generation 0 — reset()/bind() is an
    // epoch boundary; cross-epoch staleness is the caller's session scope.
    for (size_t i = 0; i < GPU_MEM_MAX_ASSETS; ++i) {
        if (mSlots[i].quarantined) {
            mSlots[i] = Slot{};  // generation 0, quarantine cleared
        } else {
            const uint8_t generation =
                static_cast<uint8_t>(mSlots[i].generation + (mSlots[i].active ? 1 : 0));
            mSlots[i] = Slot{};
            mSlots[i].generation = generation;
        }
    }
    for (size_t i = 0; i < GPU_MEM_MAX_SPANS; ++i) {
        mSpans[i] = SpanRecord{};
    }
    mAssetCount = 0;
    mStoreRanges.reset(mBacking.capacityBytes);
    mStagingRanges.reset(mStagingBytes);
}

AssetStatus AssetService::reset() {
    const AssetStatus drained = drainAndFlush();
    if (drained != AssetStatus::Ok) return drained;
    clearTables();
    return AssetStatus::Ok;
}

bool AssetService::hasBacking() const {
    return mBacking.read != nullptr && mBacking.capacityBytes > 0;
}

// ─── Handle resolution ──────────────────────────────────────────────────────

const AssetService::Slot* AssetService::resolve(const AssetHandle& handle) const {
    if (handle.index >= GPU_MEM_MAX_ASSETS) return nullptr;
    const Slot& slot = mSlots[handle.index];
    if (!slot.active) return nullptr;
    if (slot.generation != handle.generation) return nullptr;   // stale handle
    if (slot.assetClass != handle.assetClass) return nullptr;   // class collision
    return &slot;
}

AssetService::Slot* AssetService::resolve(const AssetHandle& handle) {
    return const_cast<Slot*>(static_cast<const AssetService*>(this)->resolve(handle));
}

// ─── Asset lifecycle ────────────────────────────────────────────────────────

AssetStatus AssetService::create(AssetClass assetClass, uint32_t bytes,
                                 AssetHandle& out) {
    out = AssetHandle{};
    if (!hasBacking()) return AssetStatus::NoBacking;
    if (bytes == 0) return AssetStatus::BadArgument;

    uint16_t index = 0;
    for (; index < GPU_MEM_MAX_ASSETS; ++index) {
        if (!mSlots[index].active && !mSlots[index].quarantined) break;
    }
    if (index >= GPU_MEM_MAX_ASSETS) return AssetStatus::NoSlots;

    uint32_t offset = 0;
    if (!mStoreRanges.allocate(bytes, GPU_MEM_ASSET_ALIGN_BYTES, offset)) {
        return AssetStatus::NoStoreCapacity;
    }

    Slot& slot = mSlots[index];
    slot.offset = offset;  // offset 0 is a valid store address
    slot.bytes = bytes;
    slot.assetClass = static_cast<uint8_t>(assetClass);
    slot.active = 1;
    ++mAssetCount;

    out.index = index;
    out.assetClass = static_cast<uint8_t>(assetClass);
    out.generation = slot.generation;
    return AssetStatus::Ok;
}

AssetStatus AssetService::destroy(const AssetHandle& handle) {
    Slot* slot = resolve(handle);
    if (!slot) return AssetStatus::InvalidHandle;

    // Live leases pin the asset: refuse. Cached spans are dropped (their
    // content is either clean or dirty — the asset is being destroyed, so
    // write-back would be pointless).
    for (size_t i = 0; i < GPU_MEM_MAX_SPANS; ++i) {
        SpanRecord& rec = mSpans[i];
        if (!rec.active || rec.assetIndex != handle.index) continue;
        if (rec.leaseCount > 0) return AssetStatus::LeaseActive;
    }
    for (size_t i = 0; i < GPU_MEM_MAX_SPANS; ++i) {
        SpanRecord& rec = mSpans[i];
        if (!rec.active || rec.assetIndex != handle.index) continue;
        mStagingRanges.release(rec.stagingOffset, rec.bytes);
        rec = SpanRecord{};
    }

    mStoreRanges.release(slot->offset, slot->bytes);
    slot->active = 0;
    slot->offset = 0;
    slot->bytes = 0;
    // Invalidate every outstanding handle to this slot. If the 8-bit
    // generation would wrap, quarantine the slot until reset() instead —
    // a stale token must never alias a recycled slot within an epoch.
    if (slot->generation == 0xFFu) slot->quarantined = 1;
    else ++slot->generation;
    --mAssetCount;
    return AssetStatus::Ok;
}

AssetStatus AssetService::write(const AssetHandle& handle, uint32_t assetOffset,
                                const void* src, uint32_t bytes) {
    const Slot* slot = resolve(handle);
    if (!slot) return AssetStatus::InvalidHandle;
    if (!src) return AssetStatus::BadArgument;
    if (bytes == 0 || assetOffset >= slot->bytes ||
        bytes > slot->bytes - assetOffset) {
        return AssetStatus::InvalidRange;
    }
    // Bounds vs. store capacity are guaranteed by allocation; the backing
    // re-checks them defensively.
    if (!mBacking.write(mBacking.ctx, slot->offset + assetOffset, src, bytes)) {
        return AssetStatus::BackingError;
    }
    return AssetStatus::Ok;
}

AssetStatus AssetService::read(const AssetHandle& handle, uint32_t assetOffset,
                               void* dst, uint32_t bytes) const {
    const Slot* slot = resolve(handle);
    if (!slot) return AssetStatus::InvalidHandle;
    if (!dst) return AssetStatus::BadArgument;
    if (bytes == 0 || assetOffset >= slot->bytes ||
        bytes > slot->bytes - assetOffset) {
        return AssetStatus::InvalidRange;
    }
    if (!mBacking.read(mBacking.ctx, slot->offset + assetOffset, dst, bytes)) {
        return AssetStatus::BackingError;
    }
    return AssetStatus::Ok;
}

AssetStatus AssetService::info(const AssetHandle& handle, AssetInfo& out) const {
    const Slot* slot = resolve(handle);
    if (!slot) return AssetStatus::InvalidHandle;
    out.assetClass = static_cast<AssetClass>(slot->assetClass);
    out.bytes = slot->bytes;
    return AssetStatus::Ok;
}

// ─── Span records ───────────────────────────────────────────────────────────

AssetService::SpanRecord* AssetService::findSpan(const AssetSpan& span) {
    return const_cast<SpanRecord*>(
        static_cast<const AssetService*>(this)->findSpan(span));
}

const AssetService::SpanRecord* AssetService::findSpan(const AssetSpan& span) const {
    if (!span.data || !mStaging) return nullptr;
    for (size_t i = 0; i < GPU_MEM_MAX_SPANS; ++i) {
        const SpanRecord& rec = mSpans[i];
        if (!rec.active) continue;
        if (rec.assetIndex != span.handle.index) continue;
        if (rec.assetOffset != span.offset || rec.bytes != span.bytes) continue;
        if (mStaging + rec.stagingOffset != span.data) continue;
        return &rec;
    }
    return nullptr;
}

AssetService::SpanRecord* AssetService::findSpanRecord(const AssetHandle& handle,
                                                       uint32_t assetOffset,
                                                       uint32_t bytes) {
    for (size_t i = 0; i < GPU_MEM_MAX_SPANS; ++i) {
        SpanRecord& rec = mSpans[i];
        if (rec.active && rec.assetIndex == handle.index &&
            rec.assetOffset == assetOffset && rec.bytes == bytes) {
            return &rec;
        }
    }
    return nullptr;
}

AssetService::SpanRecord* AssetService::freeSpanRecord() {
    for (size_t i = 0; i < GPU_MEM_MAX_SPANS; ++i) {
        if (!mSpans[i].active) return &mSpans[i];
    }
    return nullptr;
}


// ─── Lease-aware SRAM staging ───────────────────────────────────────────────

AssetStatus AssetService::prefetchSpan(const AssetHandle& handle,
                                       uint32_t assetOffset, uint32_t bytes,
                                       AssetSpan& out) {
    out = AssetSpan{};
    const Slot* slot = resolve(handle);
    if (!slot) return AssetStatus::InvalidHandle;
    if (bytes == 0 || assetOffset >= slot->bytes ||
        bytes > slot->bytes - assetOffset) {
        return AssetStatus::InvalidRange;
    }
    if (!mStaging || mStagingBytes == 0) return AssetStatus::StagingFull;

    // Identical span already staged? Share it with one more lease.
    SpanRecord* rec = findSpanRecord(handle, assetOffset, bytes);
    if (rec) {
        if (rec->leaseCount == 0xFFFFu) return AssetStatus::BadArgument;
        ++rec->leaseCount;
        out.handle = handle;
        out.data = mStaging + rec->stagingOffset;
        out.offset = assetOffset;
        out.bytes = bytes;
        return AssetStatus::Ok;
    }

    uint32_t stagingOffset = 0;
    SpanRecord* fresh = nullptr;
    for (;;) {
        fresh = freeSpanRecord();
        if (fresh &&
            mStagingRanges.allocate(bytes, GPU_MEM_SPAN_ALIGN_BYTES, stagingOffset)) {
            break;
        }
        // Both a record and an aligned range are required. A large free run
        // alone cannot satisfy a full record table or an unaligned hole.
        const AssetStatus evicted = evictOneCached();
        if (evicted != AssetStatus::Ok) return evicted;
    }

    if (!mBacking.read(mBacking.ctx, slot->offset + assetOffset,
                       mStaging + stagingOffset, bytes)) {
        mStagingRanges.release(stagingOffset, bytes);
        return AssetStatus::BackingError;
    }

    fresh->assetOffset = assetOffset;
    fresh->bytes = bytes;
    fresh->stagingOffset = stagingOffset;
    fresh->assetIndex = handle.index;
    fresh->leaseCount = 1;
    fresh->active = 1;
    fresh->dirty = 0;

    out.handle = handle;
    out.data = mStaging + stagingOffset;
    out.offset = assetOffset;
    out.bytes = bytes;
    return AssetStatus::Ok;
}

AssetStatus AssetService::flushSpan(const AssetSpan& span) {
    SpanRecord* rec = findSpan(span);
    if (!rec) return AssetStatus::InvalidHandle;
    if (!rec->dirty) return AssetStatus::Ok;
    const Slot& slot = mSlots[rec->assetIndex];
    if (!mBacking.write(mBacking.ctx, slot.offset + rec->assetOffset,
                        mStaging + rec->stagingOffset, rec->bytes)) {
        return AssetStatus::BackingError;  // span stays dirty — nothing lost
    }
    rec->dirty = 0;
    return AssetStatus::Ok;
}

AssetStatus AssetService::releaseSpan(AssetSpan& span, bool dirty) {
    if (!span.data) return AssetStatus::BadArgument;
    SpanRecord* rec = findSpan(span);
    if (!rec) return AssetStatus::InvalidHandle;
    if (rec->leaseCount == 0) return AssetStatus::InvalidHandle;  // already released
    if (dirty) rec->dirty = 1;
    --rec->leaseCount;
    span.data = nullptr;  // lease over; pointer no longer guaranteed
    return AssetStatus::Ok;
}

AssetStatus AssetService::evictOneCached() {
    bool writebackFailed = false;
    for (size_t i = 0; i < GPU_MEM_MAX_SPANS; ++i) {
        SpanRecord& rec = mSpans[i];
        if (!rec.active || rec.leaseCount != 0) continue;  // leased: untouchable
        if (rec.dirty) {
            const Slot& slot = mSlots[rec.assetIndex];
            if (!mBacking.write(mBacking.ctx, slot.offset + rec.assetOffset,
                                mStaging + rec.stagingOffset, rec.bytes)) {
                writebackFailed = true;
                continue;  // write-back failed: keep span, try another
            }
        }
        mStagingRanges.release(rec.stagingOffset, rec.bytes);
        rec = SpanRecord{};
        return AssetStatus::Ok;
    }
    return writebackFailed ? AssetStatus::BackingError : AssetStatus::StagingFull;
}

AssetStatus AssetService::evictCached(uint32_t minBytes, uint32_t* freedBytes) {
    const uint32_t before = mStagingRanges.freeBytes();
    while (mStagingRanges.largestFree() < minBytes) {
        const AssetStatus evicted = evictOneCached();
        if (evicted != AssetStatus::Ok) {
            if (freedBytes) *freedBytes = mStagingRanges.freeBytes() - before;
            return evicted;
        }
    }
    if (freedBytes) *freedBytes = mStagingRanges.freeBytes() - before;
    return AssetStatus::Ok;
}

uint32_t AssetService::compactStaging() {
    if (!mStaging || mStagingBytes == 0) return 0;

    // Collect active span indices sorted by stagingOffset (insertion sort;
    // GPU_MEM_MAX_SPANS is small).
    uint16_t order[GPU_MEM_MAX_SPANS];
    size_t count = 0;
    for (size_t i = 0; i < GPU_MEM_MAX_SPANS; ++i) {
        if (!mSpans[i].active) continue;
        size_t pos = count;
        while (pos > 0 &&
               mSpans[order[pos - 1]].stagingOffset > mSpans[i].stagingOffset) {
            order[pos] = order[pos - 1];
            --pos;
        }
        order[pos] = static_cast<uint16_t>(i);
        ++count;
    }

    // Pack left. Leased spans stay pinned in place; unleased spans move only
    // to lower offsets (memmove-safe, never across a pinned span).
    uint32_t next = 0;
    uint32_t moved = 0;
    for (size_t k = 0; k < count; ++k) {
        SpanRecord& rec = mSpans[order[k]];
        if (rec.leaseCount > 0) {
            const uint32_t end = alignUpU32(rec.stagingOffset + rec.bytes,
                                            GPU_MEM_SPAN_ALIGN_BYTES);
            if (end > next) next = end;
            continue;
        }
        if (rec.stagingOffset != next) {
            std::memmove(mStaging + next, mStaging + rec.stagingOffset, rec.bytes);
            rec.stagingOffset = next;
            ++moved;
        }
        next = alignUpU32(next + rec.bytes, GPU_MEM_SPAN_ALIGN_BYTES);
    }

    if (moved == 0) return 0;

    // Rebuild the free list from the final layout.
    mStagingRanges.reset(mStagingBytes);
    for (size_t k = 0; k < count; ++k) {
        const SpanRecord& rec = mSpans[order[k]];
        mStagingRanges.allocateAt(rec.stagingOffset, rec.bytes);
    }
    return moved;
}

// ─── Stats ──────────────────────────────────────────────────────────────────

AssetServiceStats AssetService::stats() const {
    AssetServiceStats s;
    s.storeCapacityBytes = mBacking.capacityBytes;
    s.storeFreeBytes = mStoreRanges.freeBytes();
    s.storeLargestFreeBytes = mStoreRanges.largestFree();
    s.assetCount = mAssetCount;
    uint16_t spans = 0;
    uint16_t leased = 0;
    for (size_t i = 0; i < GPU_MEM_MAX_SPANS; ++i) {
        if (!mSpans[i].active) continue;
        ++spans;
        if (mSpans[i].leaseCount > 0) ++leased;
    }
    s.spanCount = spans;
    s.leasedSpanCount = leased;
    s.stagingBytes = mStagingBytes;
    s.stagingFreeBytes = mStagingRanges.freeBytes();
    s.stagingLargestFreeBytes = mStagingRanges.largestFree();
    return s;
}

} // namespace gpumem
