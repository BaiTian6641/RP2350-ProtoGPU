/**
 * @file mem_tier.cpp
 * @brief Tiered Memory Manager implementation — placement, promotion, demotion, cache.
 *
 * Implements the MemTierManager class declared in mem_tier.h.
 * Manages resource placement across three memory tiers:
 *   Tier 0 — SRAM (520 KB, 1-cycle, direct)
 *   Tier 1 — QSPI Channel A (PIO2 SM0+SM1, indirect DMA)
 *   Tier 2 — QSPI Channel B (PIO2 SM2+SM3, indirect DMA)
 *
 * Both external tiers live on the single dual-channel QspiVramDriver
 * (mem_qspi_vram.h).  There is no XIP memory-mapping: all external access
 * is explicit per-channel DMA (ReadSync/WriteSync/Prefetch).
 *
 * Core algorithms:
 *   - Weight-based initial placement (from ResClass → base weight)
 *   - Score-based runtime adaptation (access frequency tracking)
 *   - Promotion: slow tier → fast tier when priority rises
 *   - Demotion: fast tier → slow tier when priority drops or SRAM pressure
 *   - DMA prefetch pipeline for QSPI-resident data before rasterization
 *   - LRU cache line management for the SRAM cache arena
 */

#include "mem_tier.h"
#include "mem_qspi_vram.h"
#include "../gpu_config.h"

#include <PglTypes.h>  // PGL_DEFRAG_URGENT

#include <cstdio>
#include <cstring>
#include <algorithm>

// ─── Initialization ─────────────────────────────────────────────────────────

bool MemTierManager::Initialize(const MemTierConfig& config,
                                 QspiVramDriver* vram) {
    config_ = config;
    vram_   = vram;

    recordCount_ = 0;
    frameCounter_ = 0;

    // Clear record array
    for (uint16_t i = 0; i < MAX_RECORDS; ++i) {
        records_[i] = MemRecord{};
    }

    // Allocate SRAM cache arena
    // Only allocate when external memory is configured — saves 64 KB BSS otherwise.
    if constexpr (GpuConfig::QspiVramEnabled()) {
        static uint8_t s_cacheArena[GpuConfig::MEM_TIER_SRAM_CACHE_BUDGET]
            __attribute__((aligned(4)));
        cacheArena_ = s_cacheArena;
    } else {
        cacheArena_ = nullptr;
    }

    // Initialize cache lines
    uint32_t lineCount = config_.sramCacheBudget / config_.cacheLineSize;
    if (lineCount > MAX_CACHE_LINES) lineCount = MAX_CACHE_LINES;

    for (uint16_t i = 0; i < MAX_CACHE_LINES; ++i) {
        cacheLines_[i] = CacheLine{};  // reset
        if (cacheArena_ && i < lineCount) {
            cacheLines_[i].data = cacheArena_ + (i * config_.cacheLineSize);
        }
    }

    printf("[MemTier] Initialized: sramCache=%lu KB (%lu lines × %lu B), "
           "qspiA=%s (%lu KB), qspiB=%s (%lu KB)\n",
           (unsigned long)(config_.sramCacheBudget / 1024),
           (unsigned long)lineCount,
           (unsigned long)config_.cacheLineSize,
           (vram_ && vram_->IsChannelInitialized(QspiChannel::A)) ? "present" : "none",
           (unsigned long)(config_.qspiACapacity / 1024),
           (vram_ && vram_->IsChannelInitialized(QspiChannel::B)) ? "present" : "none",
           (unsigned long)(config_.qspiBCapacity / 1024));

    return true;
}

// ─── Tier ↔ Channel Mapping ─────────────────────────────────────────────────
// The unified QspiVramDriver manages both external tiers; each MemTier maps
// to one QSPI channel on that single driver instance.

static QspiChannel ChannelForTier(MemTier tier) {
    return (tier == MemTier::QSPI_B) ? QspiChannel::B : QspiChannel::A;
}

// ─── Resource Registration ──────────────────────────────────────────────────

uint16_t MemTierManager::Register(uint16_t resourceId, ResClass resClass,
                                   uint32_t dataSize) {
    // Check if already registered
    MemRecord* existing = FindRecord(resourceId);
    if (existing) {
        // Update existing record
        existing->resClass = resClass;
        existing->weight = BaseWeight(resClass);
        existing->dataSize = dataSize;
        existing->UpdatePriority(config_.alphaWeight, config_.betaScore);
        return static_cast<uint16_t>(existing - records_);
    }

    if (recordCount_ >= MAX_RECORDS) {
        printf("[MemTier] ERROR: Record table full, cannot register resource %u\n",
               resourceId);
        return 0xFFFF;
    }

    // Find a free slot
    uint16_t idx = recordCount_++;
    MemRecord& rec = records_[idx];
    rec.resourceId = resourceId;
    rec.resClass = resClass;
    rec.weight = BaseWeight(resClass);
    rec.score = 0;
    rec.dataSize = dataSize;
    rec.currentTier = MemTier::NONE;
    rec.sramPtr = nullptr;
    rec.qspiAAddr = 0;
    rec.qspiBAddr = 0;
    rec.dirty = false;
    rec.pinned = false;
    rec.framesSinceAccess = 0;
    rec.UpdatePriority(config_.alphaWeight, config_.betaScore);

    // Compute ideal initial placement
    MemTier ideal = ComputeIdealTier(rec);

    // Perform initial placement
    if (ideal == MemTier::SRAM) {
        // SRAM: no external allocation needed.
        // The actual SRAM pointer is managed by the SceneState pools — the tier
        // manager just tracks which tier the resource "belongs to."
        rec.currentTier = MemTier::SRAM;
    } else if (ideal == MemTier::QSPI_A && vram_ &&
               vram_->IsChannelInitialized(QspiChannel::A)) {
        // Try to allocate in QSPI Channel A external memory
        uint32_t addr = vram_->Alloc(QspiChannel::A, dataSize, 4);
        if (addr != 0xFFFFFFFF) {
            rec.qspiAAddr = addr;
            rec.currentTier = MemTier::QSPI_A;
        } else {
            // Fallback to SRAM if Channel A is full
            rec.currentTier = MemTier::SRAM;
        }
    } else if (ideal == MemTier::QSPI_B && vram_ &&
               vram_->IsChannelInitialized(QspiChannel::B)) {
        // Try to allocate in QSPI Channel B external memory
        uint32_t addr = vram_->Alloc(QspiChannel::B, dataSize, 4);
        if (addr != 0xFFFFFFFF) {
            rec.qspiBAddr = addr;
            rec.currentTier = MemTier::QSPI_B;
        } else {
            // Fallback to SRAM if Channel B is full
            rec.currentTier = MemTier::SRAM;
        }
    } else {
        rec.currentTier = MemTier::SRAM;
    }

    return idx;
}

void MemTierManager::Unregister(uint16_t resourceId) {
    MemRecord* rec = FindRecord(resourceId);
    if (!rec) return;

    // Free cache lines associated with this resource
    for (uint16_t i = 0; i < MAX_CACHE_LINES; ++i) {
        if (cacheLines_[i].resourceId == resourceId) {
            FreeCacheLine(&cacheLines_[i]);
        }
    }

    // Free external memory allocations (size from the record for the free-list)
    if (rec->qspiAAddr != 0 && rec->qspiAAddr != 0xFFFFFFFF && vram_) {
        vram_->Free(QspiChannel::A, rec->qspiAAddr, rec->dataSize);
    }
    if (rec->qspiBAddr != 0 && rec->qspiBAddr != 0xFFFFFFFF && vram_) {
        vram_->Free(QspiChannel::B, rec->qspiBAddr, rec->dataSize);
    }

    // Zero out the record
    *rec = MemRecord{};

    // Compact: if this was the last record, shrink
    uint16_t idx = static_cast<uint16_t>(rec - records_);
    if (idx == recordCount_ - 1) {
        recordCount_--;
    }
}

// ─── Record Lookup ──────────────────────────────────────────────────────────

MemRecord* MemTierManager::FindRecord(uint16_t resourceId) {
    for (uint16_t i = 0; i < recordCount_; ++i) {
        if (records_[i].resourceId == resourceId) {
            return &records_[i];
        }
    }
    return nullptr;
}

const MemRecord* MemTierManager::FindRecord(uint16_t resourceId) const {
    for (uint16_t i = 0; i < recordCount_; ++i) {
        if (records_[i].resourceId == resourceId) {
            return &records_[i];
        }
    }
    return nullptr;
}

// ─── Tier Computation ───────────────────────────────────────────────────────

MemTier MemTierManager::ComputeIdealTier(const MemRecord& rec) const {
    // Hard constraint: pinned resources always stay in SRAM
    if (rec.pinned) return MemTier::SRAM;

    // Hard constraint: framebuffer/zbuffer/quadtree ALWAYS SRAM
    if (rec.resClass == ResClass::FRAMEBUFFER ||
        rec.resClass == ResClass::Z_BUFFER ||
        rec.resClass == ResClass::QUADTREE) {
        return MemTier::SRAM;
    }

    // Weight-based placement:
    //   HIGH weight + HIGH score → SRAM (hot critical-path data)
    //   HIGH weight + LOW score  → QSPI-A (critical but infrequent, DMA prefetch)
    //   LOW weight  + HIGH score → QSPI-B (frequent but non-critical, DMA prefetch)
    //   LOW weight  + LOW score  → QSPI-B (cold storage)

    uint16_t priority = rec.priority;

    // Tier thresholds (configurable via config_)
    // Higher priority → faster tier
    static constexpr uint16_t SRAM_THRESHOLD   = 500;   // priority >= 500 → SRAM
    static constexpr uint16_t QSPI_A_THRESHOLD = 200;   // priority >= 200 → QSPI-A

    if (priority >= SRAM_THRESHOLD) {
        return MemTier::SRAM;
    }

    // If QSPI Channel A is available, use it for medium-priority data
    if (config_.qspiACapacity > 0 && priority >= QSPI_A_THRESHOLD) {
        return MemTier::QSPI_A;
    }

    // If QSPI Channel B is available, use it for low-priority data
    if (config_.qspiBCapacity > 0) {
        return MemTier::QSPI_B;
    }

    // No external memory — everything stays in SRAM
    return MemTier::SRAM;
}

// ─── Data Access ────────────────────────────────────────────────────────────

const void* MemTierManager::Read(uint16_t resourceId) {
    MemRecord* rec = FindRecord(resourceId);
    if (!rec || rec->currentTier == MemTier::NONE) return nullptr;

    RecordAccess(resourceId);

    switch (rec->currentTier) {
        case MemTier::SRAM:
            return rec->sramPtr;

        case MemTier::QSPI_A:
        case MemTier::QSPI_B: {
            // Data is in external QSPI memory — check if cached in SRAM.
            // Both channels are indirect-DMA only in the QspiVram model
            // (no XIP memory-mapping), so both tiers resolve via the cache.
            for (uint16_t i = 0; i < MAX_CACHE_LINES; ++i) {
                CacheLine& cl = cacheLines_[i];
                if (cl.resourceId == resourceId && !cl.locked) {
                    cl.lastUsedFrame = static_cast<uint8_t>(frameCounter_ & 0xFF);
                    return cl.data;
                }
            }
            // Not cached — need to prefetch
            // Return nullptr to signal caller that data isn't ready.
            // Caller should call PrefetchForDrawList() or ReadSync manually.
            return nullptr;
        }

        default:
            return nullptr;
    }
}

void* MemTierManager::Write(uint16_t resourceId) {
    MemRecord* rec = FindRecord(resourceId);
    if (!rec) return nullptr;

    RecordAccess(resourceId);

    // Writing always goes to SRAM (promotes if needed)
    if (rec->currentTier != MemTier::SRAM) {
        Promote(*rec, MemTier::SRAM);
    }

    rec->dirty = true;
    return rec->sramPtr;
}

void MemTierManager::RecordAccess(uint16_t resourceId) {
    MemRecord* rec = FindRecord(resourceId);
    if (!rec) return;

    // Increment score (saturating)
    if (rec->score < 255) rec->score++;
    rec->framesSinceAccess = 0;
    rec->UpdatePriority(config_.alphaWeight, config_.betaScore);
}

// ─── Frame Lifecycle ────────────────────────────────────────────────────────

void MemTierManager::BeginFrame() {
    frameCounter_++;

    for (uint16_t i = 0; i < recordCount_; ++i) {
        MemRecord& rec = records_[i];
        if (rec.currentTier == MemTier::NONE) continue;

        // Decay score: exponential moving average
        // score = score * 7/8 (fast decay for volatile access patterns)
        rec.score = (rec.score * 7) >> 3;

        // Track frames since last access
        if (rec.framesSinceAccess < 255) rec.framesSinceAccess++;

        // Recalculate priority
        rec.UpdatePriority(config_.alphaWeight, config_.betaScore);

        // ── Check for demotion ──────────────────────────────────────────
        if (!rec.pinned &&
            rec.framesSinceAccess >= config_.demotionThreshold &&
            rec.currentTier == MemTier::SRAM) {

            MemTier ideal = ComputeIdealTier(rec);
            if (ideal != MemTier::SRAM) {
                Demote(rec, ideal);
            }
        }

        // ── Check for promotion ─────────────────────────────────────────
        if (rec.currentTier != MemTier::SRAM &&
            rec.priority >= config_.promotionHysteresis) {

            MemTier ideal = ComputeIdealTier(rec);
            if (static_cast<uint8_t>(ideal) < static_cast<uint8_t>(rec.currentTier)) {
                Promote(rec, ideal);
            }
        }
    }
}

void MemTierManager::PrefetchForDrawList(const uint16_t* meshIds,
                                          const uint16_t* materialIds,
                                          const uint16_t* textureIds,
                                          uint16_t drawCallCount) {
    if (!vram_ || !vram_->IsAnyChannelInitialized()) return;

    // Prefetch the first cache line of one resource from its external tier.
    auto prefetchResource = [&](uint16_t resourceId) {
        MemRecord* rec = FindRecord(resourceId);
        if (!rec || rec->dataSize == 0) return;

        MemTier tier = rec->currentTier;
        if (tier != MemTier::QSPI_A && tier != MemTier::QSPI_B) return;

        QspiChannel ch = ChannelForTier(tier);
        uint32_t addr = (tier == MemTier::QSPI_A) ? rec->qspiAAddr : rec->qspiBAddr;
        if (addr == 0 || !vram_->IsChannelInitialized(ch)) return;

        CacheLine* cl = AllocCacheLine(rec->resourceId, rec->dataSize);
        if (cl) {
            cl->locked = true;
            uint32_t chunkSize = (rec->dataSize < config_.cacheLineSize)
                               ? rec->dataSize : config_.cacheLineSize;
            vram_->Prefetch(ch, addr, cl->data, chunkSize);
            cl->size = chunkSize;
            cl->srcOffset = 0;
        }
    };

    for (uint16_t i = 0; i < drawCallCount; ++i) {
        // Prefetch mesh, material, and texture data
        if (meshIds)     prefetchResource(meshIds[i]);
        if (materialIds) prefetchResource(materialIds[i]);
        if (textureIds)  prefetchResource(textureIds[i]);
    }

    // Wait for all prefetches to complete before rasterization starts
    if (vram_->IsChannelInitialized(QspiChannel::A)) {
        vram_->WaitDma(QspiChannel::A);
    }
    if (vram_->IsChannelInitialized(QspiChannel::B)) {
        vram_->WaitDma(QspiChannel::B);
    }

    // Unlock all cache lines (rasterizer can now read them)
    for (uint16_t i = 0; i < MAX_CACHE_LINES; ++i) {
        cacheLines_[i].locked = false;
    }
}

void MemTierManager::EndFrame() {
    // Flush dirty SRAM cache lines back to external memory
    for (uint16_t i = 0; i < MAX_CACHE_LINES; ++i) {
        CacheLine& cl = cacheLines_[i];
        if (!cl.dirty || cl.resourceId == 0xFFFF) continue;

        MemRecord* rec = FindRecord(cl.resourceId);
        if (!rec) {
            cl.dirty = false;
            continue;
        }

        // Write back to the appropriate tier
        if (rec->currentTier == MemTier::QSPI_A && vram_ &&
            vram_->IsChannelInitialized(QspiChannel::A)) {
            vram_->WriteSync(QspiChannel::A, rec->qspiAAddr + cl.srcOffset,
                             cl.data, cl.size);
        } else if (rec->currentTier == MemTier::QSPI_B && vram_ &&
                   vram_->IsChannelInitialized(QspiChannel::B)) {
            vram_->WriteSync(QspiChannel::B, rec->qspiBAddr + cl.srcOffset,
                             cl.data, cl.size);
        }

        cl.dirty = false;
    }

    // Unlock all cache lines
    for (uint16_t i = 0; i < MAX_CACHE_LINES; ++i) {
        cacheLines_[i].locked = false;
    }
}

// ─── Promotion / Demotion ───────────────────────────────────────────────────

void MemTierManager::Promote(MemRecord& rec, MemTier targetTier) {
    if (rec.currentTier == targetTier) return;

    // Promotion = move to a FASTER tier (lower number)
    MemTier oldTier = rec.currentTier;

    // When promoting to SRAM from an external tier, load the data via cache
    if (targetTier == MemTier::SRAM && rec.dataSize > 0) {
        if ((oldTier == MemTier::QSPI_A || oldTier == MemTier::QSPI_B) && vram_) {
            QspiChannel ch = ChannelForTier(oldTier);
            uint32_t addr = (oldTier == MemTier::QSPI_A) ? rec.qspiAAddr
                                                         : rec.qspiBAddr;
            if (vram_->IsChannelInitialized(ch) &&
                addr != 0 && addr != 0xFFFFFFFF) {
                // Allocate a SRAM cache line and DMA copy from the QSPI channel
                CacheLine* cl = AllocCacheLine(rec.resourceId, rec.dataSize);
                if (cl && cl->data) {
                    uint32_t copySize = (rec.dataSize <= config_.cacheLineSize)
                                            ? rec.dataSize : config_.cacheLineSize;
                    vram_->ReadSync(ch, addr, cl->data, copySize);
                    cl->srcOffset = 0;
                    cl->size = copySize;
                    cl->dirty = false;
                    cl->lastUsedFrame = static_cast<uint8_t>(frameCounter_ & 0xFF);
                    rec.sramPtr = cl->data;
                }
            }
        }
    }

    rec.currentTier = targetTier;

    printf("[MemTier] Promote resource %u: tier %u → %u (priority=%u)\n",
           rec.resourceId,
           static_cast<uint8_t>(oldTier),
           static_cast<uint8_t>(targetTier),
           rec.priority);
}

void MemTierManager::Demote(MemRecord& rec, MemTier targetTier) {
    if (rec.currentTier == targetTier) return;
    if (rec.pinned) return;

    MemTier oldTier = rec.currentTier;

    // If demoting from SRAM to an external tier, write data out first
    if (oldTier == MemTier::SRAM &&
        (targetTier == MemTier::QSPI_A || targetTier == MemTier::QSPI_B)) {
        QspiChannel ch = ChannelForTier(targetTier);
        // Can't demote to a tier whose channel isn't up — keep the record in
        // SRAM rather than flipping the tier while its data has no home.
        if (!vram_ || !vram_->IsChannelInitialized(ch)) return;

        if (rec.sramPtr && rec.dataSize > 0) {
            uint32_t& tierAddr = (targetTier == MemTier::QSPI_A)
                                 ? rec.qspiAAddr : rec.qspiBAddr;
            // Allocate in the target tier if not already allocated
            if (tierAddr == 0) {
                tierAddr = vram_->Alloc(ch, rec.dataSize, 4);
            }
            if (tierAddr != 0xFFFFFFFF && tierAddr != 0) {
                vram_->WriteSync(ch, tierAddr, rec.sramPtr, rec.dataSize);
            } else {
                // External tier full — can't demote
                return;
            }
        }
    }

    rec.currentTier = targetTier;
    rec.dirty = false;

    printf("[MemTier] Demote resource %u: tier %u → %u (priority=%u, "
           "framesSince=%u)\n",
           rec.resourceId,
           static_cast<uint8_t>(oldTier),
           static_cast<uint8_t>(targetTier),
           rec.priority,
           rec.framesSinceAccess);
}

// ─── SRAM Cache Line Management ─────────────────────────────────────────────

CacheLine* MemTierManager::AllocCacheLine(uint16_t resourceId, uint32_t size) {
    // First, check if we already have a cache line for this resource
    for (uint16_t i = 0; i < MAX_CACHE_LINES; ++i) {
        if (cacheLines_[i].resourceId == resourceId) {
            return &cacheLines_[i];
        }
    }

    // Find a free cache line
    for (uint16_t i = 0; i < MAX_CACHE_LINES; ++i) {
        if (cacheLines_[i].resourceId == 0xFFFF && cacheLines_[i].data != nullptr) {
            cacheLines_[i].resourceId = resourceId;
            cacheLines_[i].lastUsedFrame = static_cast<uint8_t>(frameCounter_ & 0xFF);
            return &cacheLines_[i];
        }
    }

    // No free lines — evict LRU
    CacheLine* lru = FindLRUCacheLine();
    if (lru) {
        // Write back if dirty
        if (lru->dirty && lru->resourceId != 0xFFFF) {
            MemRecord* rec = FindRecord(lru->resourceId);
            if (rec && vram_) {
                if (rec->currentTier == MemTier::QSPI_A &&
                    rec->qspiAAddr != 0 && rec->qspiAAddr != 0xFFFFFFFF &&
                    vram_->IsChannelInitialized(QspiChannel::A)) {
                    vram_->WriteSync(QspiChannel::A,
                                     rec->qspiAAddr + lru->srcOffset,
                                     lru->data, lru->size);
                } else if (rec->currentTier == MemTier::QSPI_B &&
                           rec->qspiBAddr != 0 && rec->qspiBAddr != 0xFFFFFFFF &&
                           vram_->IsChannelInitialized(QspiChannel::B)) {
                    vram_->WriteSync(QspiChannel::B,
                                     rec->qspiBAddr + lru->srcOffset,
                                     lru->data, lru->size);
                }
            }
        }

        // Reuse the line
        lru->resourceId = resourceId;
        lru->srcOffset = 0;
        lru->size = 0;
        lru->dirty = false;
        lru->locked = false;
        lru->lastUsedFrame = static_cast<uint8_t>(frameCounter_ & 0xFF);
        return lru;
    }

    return nullptr;  // Should not happen if MAX_CACHE_LINES > 0
}

void MemTierManager::FreeCacheLine(CacheLine* line) {
    if (!line) return;
    line->resourceId = 0xFFFF;
    line->srcOffset = 0;
    line->size = 0;
    line->dirty = false;
    line->locked = false;
}

CacheLine* MemTierManager::FindLRUCacheLine() {
    CacheLine* best = nullptr;
    uint8_t oldestAge = 0;
    uint8_t currentFrame = static_cast<uint8_t>(frameCounter_ & 0xFF);

    for (uint16_t i = 0; i < MAX_CACHE_LINES; ++i) {
        CacheLine& cl = cacheLines_[i];
        if (cl.locked || cl.data == nullptr) continue;  // skip locked and uninitialized

        uint8_t age = static_cast<uint8_t>(currentFrame - cl.lastUsedFrame);
        if (!best || age > oldestAge) {
            best = &cl;
            oldestAge = age;
        }
    }

    return best;
}

// ─── Diagnostics ────────────────────────────────────────────────────────────

void MemTierManager::PrintStats() const {
    uint16_t sramCount = 0, qspiACount = 0, qspiBCount = 0;
    uint32_t sramBytes = 0, qspiABytes = 0, qspiBBytes = 0;

    for (uint16_t i = 0; i < recordCount_; ++i) {
        const MemRecord& rec = records_[i];
        switch (rec.currentTier) {
            case MemTier::SRAM:
                sramCount++;
                sramBytes += rec.dataSize;
                break;
            case MemTier::QSPI_A:
                qspiACount++;
                qspiABytes += rec.dataSize;
                break;
            case MemTier::QSPI_B:
                qspiBCount++;
                qspiBBytes += rec.dataSize;
                break;
            default:
                break;
        }
    }

    uint16_t cacheUsed = 0;
    for (uint16_t i = 0; i < MAX_CACHE_LINES; ++i) {
        if (cacheLines_[i].resourceId != 0xFFFF) cacheUsed++;
    }

    printf("[MemTier] Resources: %u total (%u SRAM/%lu KB, %u QSPI-A/%lu KB, "
           "%u QSPI-B/%lu KB). Cache: %u/%u lines used.\n",
           recordCount_,
           sramCount, (unsigned long)(sramBytes / 1024),
           qspiACount, (unsigned long)(qspiABytes / 1024),
           qspiBCount, (unsigned long)(qspiBBytes / 1024),
           cacheUsed, MAX_CACHE_LINES);
}

uint32_t MemTierManager::GetSramCacheUsed() const {
    uint32_t used = 0;
    for (uint16_t i = 0; i < MAX_CACHE_LINES; ++i) {
        if (cacheLines_[i].resourceId != 0xFFFF) {
            used += cacheLines_[i].size;
        }
    }
    return used;
}

MemTier MemTierManager::GetTier(uint16_t resourceId) const {
    const MemRecord* rec = FindRecord(resourceId);
    return rec ? rec->currentTier : MemTier::NONE;
}

// ─── Defragmentation (M12) ──────────────────────────────────────────────────
//
// Algorithm:
//   1. Collect all records that reside in the target tier.
//   2. Sort them by their base address within that tier.
//   3. Walk sorted list: if record[i].address > expectedNextAddress,
//      there is a gap.  Slide record[i] down to close the gap.
//   4. For SRAM: memmove() the data.
//      For QSPI: DMA read → staging buffer → DMA write at new address.
//   5. Track total bytes moved; stop early if budget exceeded (incremental).
//   6. After sweep, count remaining fragments and largest free block.
//
// The defragmenter runs on Core 0 in the command-parse window. It must NOT
// touch records that are pinned (rasterizer-critical) or locked cache lines.
//
// Thread safety: called from ParseCommand context (Core 0 only), never from
// Core 1. Records and pointers are updated atomically from Core 0's view.

bool MemTierManager::Defragment(MemTier tier, uint8_t mode, uint16_t maxMoveKB,
                                 uint16_t& movedKB, uint16_t& fragmentCount,
                                 uint16_t& largestFreeKB) {
    movedKB       = 0;
    fragmentCount = 0;
    largestFreeKB = 0;

    // ── Helper: defrag one specific tier ────────────────────────────────
    auto defragOneTier = [&](MemTier targetTier) -> bool {
        // Collect indices of records in this tier
        uint16_t indices[MAX_RECORDS];
        uint16_t count = 0;

        for (uint16_t i = 0; i < recordCount_; ++i) {
            if (records_[i].currentTier == targetTier) {
                indices[count++] = i;
            }
        }

        if (count == 0) return true;  // nothing to defrag

        // Sort by address within the tier
        // For SRAM, sort by sramPtr; for QSPI-A/B, sort by qspiAAddr/qspiBAddr
        if (targetTier == MemTier::SRAM) {
            // Insertion sort (records are few, avoid pulling in std::sort)
            for (uint16_t i = 1; i < count; ++i) {
                uint16_t key = indices[i];
                uintptr_t keyAddr = reinterpret_cast<uintptr_t>(records_[key].sramPtr);
                int16_t j = static_cast<int16_t>(i) - 1;
                while (j >= 0 && reinterpret_cast<uintptr_t>(
                           records_[indices[j]].sramPtr) > keyAddr) {
                    indices[j + 1] = indices[j];
                    j--;
                }
                indices[j + 1] = key;
            }
        } else {
            // Sort by QSPI address
            for (uint16_t i = 1; i < count; ++i) {
                uint16_t key = indices[i];
                uint32_t keyAddr = (targetTier == MemTier::QSPI_A)
                    ? records_[key].qspiAAddr : records_[key].qspiBAddr;
                int16_t j = static_cast<int16_t>(i) - 1;
                while (j >= 0) {
                    uint32_t jAddr = (targetTier == MemTier::QSPI_A)
                        ? records_[indices[j]].qspiAAddr
                        : records_[indices[j]].qspiBAddr;
                    if (jAddr <= keyAddr) break;
                    indices[j + 1] = indices[j];
                    j--;
                }
                indices[j + 1] = key;
            }
        }

        // Budget tracking
        uint32_t budgetBytes = (mode == PGL_DEFRAG_URGENT)
            ? 0xFFFFFFFF  // no limit
            : static_cast<uint32_t>(maxMoveKB) * 1024;
        uint32_t movedBytes = 0;
        bool budgetExhausted = false;

        // ── SRAM compaction ─────────────────────────────────────────────
        if (targetTier == MemTier::SRAM) {
            // Determine the compaction arena base.
            // For SRAM resources, the base is the start of the cache arena.
            // We compact relative to the arena, not absolute addresses.
            uint8_t* arenaBase = cacheArena_;
            if (!arenaBase) return true;  // no cache arena → nothing to compact

            uint8_t* nextFree = arenaBase;

            for (uint16_t k = 0; k < count; ++k) {
                MemRecord& rec = records_[indices[k]];
                if (rec.pinned) {
                    // Pinned records cannot be moved — skip and advance nextFree
                    // past this record's extent
                    uint8_t* recEnd = static_cast<uint8_t*>(rec.sramPtr) + rec.dataSize;
                    if (recEnd > nextFree) nextFree = recEnd;
                    continue;
                }

                uint8_t* currentAddr = static_cast<uint8_t*>(rec.sramPtr);
                if (currentAddr > nextFree) {
                    // Gap found — slide record down
                    uint32_t moveSize = rec.dataSize;

                    // Check budget
                    if (movedBytes + moveSize > budgetBytes) {
                        budgetExhausted = true;
                        break;
                    }

                    memmove(nextFree, currentAddr, moveSize);
                    rec.sramPtr = nextFree;
                    movedBytes += moveSize;
                }

                nextFree = static_cast<uint8_t*>(rec.sramPtr) + rec.dataSize;
            }
        }
        // ── QSPI compaction ─────────────────────────────────────────────
        else if (targetTier == MemTier::QSPI_A || targetTier == MemTier::QSPI_B) {
            // QSPI compaction uses DMA staging through a cache line.
            // We need vram_ driver to perform reads and writes.
            QspiChannel ch = ChannelForTier(targetTier);
            if (!vram_ || !vram_->IsChannelInitialized(ch)) {
                return true;  // tier's channel not up → nothing to compact
            }

            uint32_t nextFreeAddr = 0;

            for (uint16_t k = 0; k < count; ++k) {
                MemRecord& rec = records_[indices[k]];
                uint32_t currentAddr = (targetTier == MemTier::QSPI_A)
                    ? rec.qspiAAddr : rec.qspiBAddr;

                if (currentAddr > nextFreeAddr) {
                    // Gap found — relocate via DMA staging:
                    //   1. DMA read from currentAddr into staging buffer
                    //   2. DMA write from staging buffer to nextFreeAddr
                    // This is expensive and must respect budgetBytes.
                    uint32_t moveSize = rec.dataSize;

                    if (movedBytes + moveSize > budgetBytes) {
                        budgetExhausted = true;
                        break;
                    }

                    // Find a cache line to use as staging buffer
                    CacheLine* staging = FindLRUCacheLine();
                    if (!staging || !staging->data) {
                        printf("[MemTier] Defrag: no staging cache line available\n");
                        budgetExhausted = true;
                        break;
                    }

                    // Detach the staging line from whatever it was caching —
                    // flush it first if dirty so no writeback data is lost.
                    if (staging->resourceId != 0xFFFF && staging->dirty) {
                        MemRecord* owner = FindRecord(staging->resourceId);
                        if (owner &&
                            (owner->currentTier == MemTier::QSPI_A ||
                             owner->currentTier == MemTier::QSPI_B)) {
                            QspiChannel ownerCh = ChannelForTier(owner->currentTier);
                            uint32_t ownerAddr = (owner->currentTier == MemTier::QSPI_A)
                                ? owner->qspiAAddr : owner->qspiBAddr;
                            if (ownerAddr != 0 && ownerAddr != 0xFFFFFFFF &&
                                vram_->IsChannelInitialized(ownerCh)) {
                                vram_->WriteSync(ownerCh,
                                                 ownerAddr + staging->srcOffset,
                                                 staging->data, staging->size);
                            }
                        }
                    }
                    FreeCacheLine(staging);

                    // Move in chunks of cache line size
                    uint32_t remaining = moveSize;
                    uint32_t srcOff = 0;
                    bool moveFailed = false;
                    while (remaining > 0) {
                        uint32_t chunkSize = (remaining > config_.cacheLineSize)
                            ? config_.cacheLineSize : remaining;

                        if (!vram_->ReadSync(ch, currentAddr + srcOff,
                                             staging->data, chunkSize) ||
                            !vram_->WriteSync(ch, nextFreeAddr + srcOff,
                                              staging->data, chunkSize)) {
                            printf("[MemTier] Defrag: DMA move failed at +%lu\n",
                                   (unsigned long)srcOff);
                            moveFailed = true;
                            break;
                        }

                        srcOff += chunkSize;
                        remaining -= chunkSize;
                    }
                    if (moveFailed) break;

                    // Update record address
                    if (targetTier == MemTier::QSPI_A) {
                        rec.qspiAAddr = nextFreeAddr;
                    } else {
                        rec.qspiBAddr = nextFreeAddr;
                    }
                    movedBytes += moveSize;
                }

                nextFreeAddr = ((targetTier == MemTier::QSPI_A)
                    ? rec.qspiAAddr : rec.qspiBAddr) + rec.dataSize;
            }
        }

        movedKB += static_cast<uint16_t>(movedBytes / 1024);

        // ── Count remaining fragments and largest free block ────────────
        if (targetTier == MemTier::SRAM && cacheArena_) {
            uint8_t* arenaEnd = cacheArena_ + config_.sramCacheBudget;
            uint8_t* expected = cacheArena_;
            uint32_t largestGap = 0;

            for (uint16_t k = 0; k < count; ++k) {
                MemRecord& rec = records_[indices[k]];
                uint8_t* addr = static_cast<uint8_t*>(rec.sramPtr);
                if (addr > expected) {
                    fragmentCount++;
                    uint32_t gap = static_cast<uint32_t>(addr - expected);
                    if (gap > largestGap) largestGap = gap;
                }
                expected = addr + rec.dataSize;
            }
            // Trailing free space
            if (expected < arenaEnd) {
                uint32_t gap = static_cast<uint32_t>(arenaEnd - expected);
                if (gap > largestGap) largestGap = gap;
                if (count > 0) fragmentCount++;  // trailing gap is a fragment
            }
            largestFreeKB = static_cast<uint16_t>(largestGap / 1024);
        } else if (targetTier == MemTier::QSPI_A || targetTier == MemTier::QSPI_B) {
            uint32_t totalCap = (targetTier == MemTier::QSPI_A)
                ? config_.qspiACapacity : config_.qspiBCapacity;
            uint32_t expectedAddr = 0;
            uint32_t largestGap = 0;

            for (uint16_t k = 0; k < count; ++k) {
                MemRecord& rec = records_[indices[k]];
                uint32_t addr = (targetTier == MemTier::QSPI_A)
                    ? rec.qspiAAddr : rec.qspiBAddr;
                if (addr > expectedAddr) {
                    fragmentCount++;
                    uint32_t gap = addr - expectedAddr;
                    if (gap > largestGap) largestGap = gap;
                }
                expectedAddr = addr + rec.dataSize;
            }
            if (expectedAddr < totalCap) {
                uint32_t gap = totalCap - expectedAddr;
                if (gap > largestGap) largestGap = gap;
                if (count > 0) fragmentCount++;
            }
            largestFreeKB = static_cast<uint16_t>(largestGap / 1024);
        }

        return !budgetExhausted;
    };

    // ── Dispatch: defrag one tier or all tiers ──────────────────────────
    bool allDone = true;

    if (tier == MemTier::NONE) {
        // Defrag all populated tiers
        if (!defragOneTier(MemTier::SRAM))   allDone = false;
        if (!defragOneTier(MemTier::QSPI_A)) allDone = false;
        if (!defragOneTier(MemTier::QSPI_B)) allDone = false;
    } else {
        allDone = defragOneTier(tier);
    }

    printf("[MemTier] Defrag: tier=%u mode=%u moved=%u KB, fragments=%u, largestFree=%u KB, done=%d\n",
           static_cast<unsigned>(tier), mode, movedKB, fragmentCount,
           largestFreeKB, allDone);

    return allDone;
}
