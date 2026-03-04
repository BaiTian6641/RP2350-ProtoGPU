/**
 * @file mem_tier.cpp
 * @brief Tiered Memory Manager implementation — placement, promotion, demotion, cache.
 *
 * Implements the MemTierManager class declared in mem_tier.h.
 * Manages resource placement across three memory tiers:
 *   Tier 0 — SRAM (520 KB, 1-cycle, direct)
 *   Tier 1 — PIO2 External (OPI PSRAM / QSPI MRAM, indirect DMA)
 *   Tier 2 — QSPI CS1 XIP (memory-mapped, HW cache)
 *
 * Core algorithms:
 *   - Weight-based initial placement (from ResClass → base weight)
 *   - Score-based runtime adaptation (access frequency tracking)
 *   - Promotion: slow tier → fast tier when priority rises
 *   - Demotion: fast tier → slow tier when priority drops or SRAM pressure
 *   - DMA prefetch pipeline for PIO2-resident data before rasterization
 *   - LRU cache line management for the SRAM cache arena
 */

#include "mem_tier.h"
#include "mem_opi_psram.h"
#include "mem_qspi_psram.h"
#include "../gpu_config.h"

#include <cstdio>
#include <cstring>
#include <algorithm>

// ─── Initialization ─────────────────────────────────────────────────────────

bool MemTierManager::Initialize(const MemTierConfig& config,
                                 OpiPsramDriver* opi,
                                 QspiPsramDriver* qspi) {
    config_ = config;
    opi_    = opi;
    qspi_  = qspi;

    recordCount_ = 0;
    frameCounter_ = 0;

    // Clear record array
    for (uint16_t i = 0; i < MAX_RECORDS; ++i) {
        records_[i] = MemRecord{};
    }

    // Allocate SRAM cache arena
    // In a real deployment, this would be a static array or linker-placed region.
    // For now, we use a static buffer sized to the configured budget.
    static uint8_t s_cacheArena[GpuConfig::MEM_TIER_SRAM_CACHE_BUDGET]
        __attribute__((aligned(4)));
    cacheArena_ = s_cacheArena;

    // Initialize cache lines
    uint32_t lineCount = config_.sramCacheBudget / config_.cacheLineSize;
    if (lineCount > MAX_CACHE_LINES) lineCount = MAX_CACHE_LINES;

    for (uint16_t i = 0; i < MAX_CACHE_LINES; ++i) {
        cacheLines_[i] = CacheLine{};  // reset
        if (i < lineCount) {
            cacheLines_[i].data = cacheArena_ + (i * config_.cacheLineSize);
        }
    }

    printf("[MemTier] Initialized: sramCache=%lu KB (%lu lines × %lu B), "
           "opi=%s (%lu KB), qspi=%s (%lu KB)\n",
           (unsigned long)(config_.sramCacheBudget / 1024),
           (unsigned long)lineCount,
           (unsigned long)config_.cacheLineSize,
           opi_ ? "present" : "none",
           (unsigned long)(config_.opiCapacity / 1024),
           qspi_ ? "present" : "none",
           (unsigned long)(config_.qspiCapacity / 1024));

    return true;
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
    rec.opiAddr = 0;
    rec.qspiPtr = nullptr;
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
    } else if (ideal == MemTier::OPI_PSRAM && opi_ && opi_->IsInitialized()) {
        // Try to allocate in PIO2 external memory
        uint32_t addr = opi_->Alloc(dataSize, 4);
        if (addr != 0xFFFFFFFF) {
            rec.opiAddr = addr;
            rec.currentTier = MemTier::OPI_PSRAM;
        } else {
            // Fallback to SRAM if OPI is full
            rec.currentTier = MemTier::SRAM;
        }
    } else if (ideal == MemTier::QSPI_XIP && qspi_ && qspi_->IsInitialized()) {
        // Try to allocate in QSPI XIP
        uint32_t offset = qspi_->Alloc(dataSize, 4);
        if (offset != 0xFFFFFFFF) {
            rec.qspiPtr = reinterpret_cast<void*>(config_.qspiXipBase + offset);
            rec.currentTier = MemTier::QSPI_XIP;
        } else {
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

    // Free external memory allocations
    if (rec->opiAddr != 0 && rec->opiAddr != 0xFFFFFFFF && opi_) {
        opi_->Free(rec->opiAddr);
    }
    if (rec->qspiPtr != nullptr && qspi_) {
        uint32_t offset = static_cast<uint32_t>(
            reinterpret_cast<uintptr_t>(rec->qspiPtr) - config_.qspiXipBase);
        qspi_->Free(offset);
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
    //   HIGH weight + LOW score  → PIO2 (critical but infrequent, DMA prefetch)
    //   LOW weight  + HIGH score → QSPI XIP (frequent but non-critical, HW cache)
    //   LOW weight  + LOW score  → QSPI XIP (cold storage)

    uint16_t priority = rec.priority;

    // Tier thresholds (configurable via config_)
    // Higher priority → faster tier
    static constexpr uint16_t SRAM_THRESHOLD = 500;     // priority >= 500 → SRAM
    static constexpr uint16_t OPI_THRESHOLD  = 200;     // priority >= 200 → PIO2

    if (priority >= SRAM_THRESHOLD) {
        return MemTier::SRAM;
    }

    // If PIO2 external memory is available, use it for medium-priority data
    if (config_.opiCapacity > 0 && priority >= OPI_THRESHOLD) {
        return MemTier::OPI_PSRAM;
    }

    // If QSPI is available, use it for low-priority data
    if (config_.qspiCapacity > 0) {
        return MemTier::QSPI_XIP;
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

        case MemTier::OPI_PSRAM: {
            // Data is in PIO2 external memory — check if cached in SRAM
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

        case MemTier::QSPI_XIP:
            // XIP-mapped: return the memory-mapped pointer directly
            return rec->qspiPtr;

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
    if (!opi_ || !opi_->IsInitialized()) return;

    for (uint16_t i = 0; i < drawCallCount; ++i) {
        // Prefetch mesh data
        if (meshIds) {
            MemRecord* rec = FindRecord(meshIds[i]);
            if (rec && rec->currentTier == MemTier::OPI_PSRAM && rec->opiAddr != 0) {
                CacheLine* cl = AllocCacheLine(rec->resourceId, rec->dataSize);
                if (cl) {
                    cl->locked = true;
                    uint32_t chunkSize = (rec->dataSize < config_.cacheLineSize)
                                       ? rec->dataSize : config_.cacheLineSize;
                    opi_->Prefetch(rec->opiAddr, cl->data, chunkSize);
                    cl->size = chunkSize;
                    cl->srcOffset = 0;
                }
            }
        }

        // Prefetch material data
        if (materialIds) {
            MemRecord* rec = FindRecord(materialIds[i]);
            if (rec && rec->currentTier == MemTier::OPI_PSRAM && rec->opiAddr != 0) {
                CacheLine* cl = AllocCacheLine(rec->resourceId, rec->dataSize);
                if (cl) {
                    cl->locked = true;
                    uint32_t chunkSize = (rec->dataSize < config_.cacheLineSize)
                                       ? rec->dataSize : config_.cacheLineSize;
                    opi_->Prefetch(rec->opiAddr, cl->data, chunkSize);
                    cl->size = chunkSize;
                    cl->srcOffset = 0;
                }
            }
        }

        // Prefetch texture data
        if (textureIds) {
            MemRecord* rec = FindRecord(textureIds[i]);
            if (rec && rec->currentTier == MemTier::OPI_PSRAM && rec->opiAddr != 0) {
                CacheLine* cl = AllocCacheLine(rec->resourceId, rec->dataSize);
                if (cl) {
                    cl->locked = true;
                    uint32_t chunkSize = (rec->dataSize < config_.cacheLineSize)
                                       ? rec->dataSize : config_.cacheLineSize;
                    opi_->Prefetch(rec->opiAddr, cl->data, chunkSize);
                    cl->size = chunkSize;
                    cl->srcOffset = 0;
                }
            }
        }
    }

    // Wait for all prefetches to complete before rasterization starts
    if (opi_) {
        opi_->WaitDma();
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
        if (rec->currentTier == MemTier::OPI_PSRAM && opi_ && opi_->IsInitialized()) {
            opi_->WriteSync(rec->opiAddr + cl.srcOffset, cl.data, cl.size);
        } else if (rec->currentTier == MemTier::QSPI_XIP && qspi_ && qspi_->IsInitialized()) {
            uint32_t offset = static_cast<uint32_t>(
                reinterpret_cast<uintptr_t>(rec->qspiPtr) - config_.qspiXipBase);
            qspi_->Write(offset + cl.srcOffset, cl.data, cl.size);
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
        if (oldTier == MemTier::OPI_PSRAM && opi_ && opi_->IsInitialized() &&
            rec.opiAddr != 0 && rec.opiAddr != 0xFFFFFFFF) {
            // Allocate a SRAM cache line and DMA copy from OPI
            CacheLine* cl = AllocCacheLine(rec.resourceId, rec.dataSize);
            if (cl && cl->data) {
                uint32_t copySize = (rec.dataSize <= config_.cacheLineSize)
                                        ? rec.dataSize : config_.cacheLineSize;
                opi_->ReadSync(rec.opiAddr, cl->data, copySize);
                cl->srcOffset = 0;
                cl->size = copySize;
                cl->dirty = false;
                cl->lastUsedFrame = static_cast<uint8_t>(frameCounter_ & 0xFF);
                rec.sramPtr = cl->data;
            }
        } else if (oldTier == MemTier::QSPI_XIP && qspi_ && qspi_->IsInitialized() &&
                   rec.qspiPtr != nullptr) {
            // QSPI XIP is memory-mapped — allocate cache line and memcpy
            CacheLine* cl = AllocCacheLine(rec.resourceId, rec.dataSize);
            if (cl && cl->data) {
                uint32_t copySize = (rec.dataSize <= config_.cacheLineSize)
                                        ? rec.dataSize : config_.cacheLineSize;
                memcpy(cl->data, rec.qspiPtr, copySize);
                cl->srcOffset = 0;
                cl->size = copySize;
                cl->dirty = false;
                cl->lastUsedFrame = static_cast<uint8_t>(frameCounter_ & 0xFF);
                rec.sramPtr = cl->data;
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

    // If demoting from SRAM to PIO2, write data to external memory first
    if (oldTier == MemTier::SRAM && targetTier == MemTier::OPI_PSRAM) {
        if (opi_ && opi_->IsInitialized() && rec.sramPtr && rec.dataSize > 0) {
            // Allocate in OPI if not already allocated
            if (rec.opiAddr == 0) {
                rec.opiAddr = opi_->Alloc(rec.dataSize, 4);
            }
            if (rec.opiAddr != 0xFFFFFFFF && rec.opiAddr != 0) {
                opi_->WriteSync(rec.opiAddr, rec.sramPtr, rec.dataSize);
            } else {
                // OPI full — can't demote
                return;
            }
        }
    }

    // If demoting from SRAM to QSPI, write data via XIP write-through
    if (oldTier == MemTier::SRAM && targetTier == MemTier::QSPI_XIP) {
        if (qspi_ && qspi_->IsInitialized() && rec.sramPtr && rec.dataSize > 0) {
            if (!rec.qspiPtr) {
                uint32_t offset = qspi_->Alloc(rec.dataSize, 4);
                if (offset != 0xFFFFFFFF) {
                    rec.qspiPtr = reinterpret_cast<void*>(config_.qspiXipBase + offset);
                } else {
                    return;  // QSPI full
                }
            }
            uint32_t offset = static_cast<uint32_t>(
                reinterpret_cast<uintptr_t>(rec.qspiPtr) - config_.qspiXipBase);
            qspi_->Write(offset, rec.sramPtr, rec.dataSize);
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
            if (rec) {
                if (opi_ && opi_->IsInitialized() &&
                    rec->currentTier == MemTier::OPI_PSRAM &&
                    rec->opiAddr != 0 && rec->opiAddr != 0xFFFFFFFF) {
                    opi_->WriteSync(rec->opiAddr + lru->srcOffset, lru->data, lru->size);
                } else if (qspi_ && qspi_->IsInitialized() &&
                           rec->currentTier == MemTier::QSPI_XIP && rec->qspiPtr) {
                    uint32_t offset = static_cast<uint32_t>(
                        reinterpret_cast<uintptr_t>(rec->qspiPtr) - config_.qspiXipBase)
                        + lru->srcOffset;
                    qspi_->Write(offset, lru->data, lru->size);
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
    uint16_t sramCount = 0, opiCount = 0, qspiCount = 0;
    uint32_t sramBytes = 0, opiBytes = 0, qspiBytes = 0;

    for (uint16_t i = 0; i < recordCount_; ++i) {
        const MemRecord& rec = records_[i];
        switch (rec.currentTier) {
            case MemTier::SRAM:
                sramCount++;
                sramBytes += rec.dataSize;
                break;
            case MemTier::OPI_PSRAM:
                opiCount++;
                opiBytes += rec.dataSize;
                break;
            case MemTier::QSPI_XIP:
                qspiCount++;
                qspiBytes += rec.dataSize;
                break;
            default:
                break;
        }
    }

    uint16_t cacheUsed = 0;
    for (uint16_t i = 0; i < MAX_CACHE_LINES; ++i) {
        if (cacheLines_[i].resourceId != 0xFFFF) cacheUsed++;
    }

    printf("[MemTier] Resources: %u total (%u SRAM/%lu KB, %u PIO2/%lu KB, "
           "%u QSPI/%lu KB). Cache: %u/%u lines used.\n",
           recordCount_,
           sramCount, (unsigned long)(sramBytes / 1024),
           opiCount, (unsigned long)(opiBytes / 1024),
           qspiCount, (unsigned long)(qspiBytes / 1024),
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
