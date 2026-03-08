/**
 * @file flash_persist.cpp
 * @brief Flash persistence manager implementation (M12).
 *
 * Manages non-volatile resource storage in RP2350 onboard flash.
 * Uses the last 512 KB of flash for a manifest + data region.
 *
 * Flash operations use the Pico SDK flash API:
 *   flash_range_erase()   — erase 4KB sectors
 *   flash_range_program() — program 256B pages
 *
 * Both require interrupts disabled on the calling core and must not be called
 * while the other core is executing from flash. The manager handles this via
 * critical sections.
 */

#include "flash_persist.h"
#include "mem_tier.h"
#include "../gpu_config.h"

#include "hardware/flash.h"
#include "hardware/sync.h"
#include "pico/stdlib.h"

#include <cstdio>
#include <cstring>

// ─── XIP Base Address ───────────────────────────────────────────────────────
// Flash is memory-mapped at XIP_BASE (0x10000000 on RP2350).
#ifndef XIP_BASE
#define XIP_BASE 0x10000000u
#endif

/// Base address of the persistence region in the flash address space.
static constexpr uint32_t PERSIST_FLASH_OFFSET = GpuConfig::FLASH_PERSIST_OFFSET;

/// Data region starts after the manifest (header + max entries), rounded up to sector.
static constexpr uint32_t MANIFEST_TOTAL_SIZE =
    sizeof(FlashManifestHeader) +
    sizeof(FlashManifestEntry) * GpuConfig::FLASH_MAX_ENTRIES;

/// Round up to sector boundary.
static constexpr uint32_t DATA_REGION_OFFSET =
    ((MANIFEST_TOTAL_SIZE + GpuConfig::FLASH_SECTOR_SIZE - 1)
     / GpuConfig::FLASH_SECTOR_SIZE) * GpuConfig::FLASH_SECTOR_SIZE;

/// Total data region size.
static constexpr uint32_t DATA_REGION_SIZE =
    GpuConfig::FLASH_PERSIST_SIZE - DATA_REGION_OFFSET;

// ─── CRC-32 (IEEE 802.3 polynomial) ────────────────────────────────────────

static const uint32_t crc32_table[256] = {
    0x00000000, 0x77073096, 0xEE0E612C, 0x990951BA, 0x076DC419, 0x706AF48F,
    0xE963A535, 0x9E6495A3, 0x0EDB8832, 0x79DCB8A4, 0xE0D5E91B, 0x97D2D988,
    0x09B64C2B, 0x7EB17CBB, 0xE7B82D09, 0x90BF1D9F, 0x1DB71064, 0x6AB020F2,
    0xF3B97148, 0x84BE41DE, 0x1ADAD47D, 0x6DDDE4EB, 0xF4D4B551, 0x83D385C7,
    0x136C9856, 0x646BA8C0, 0xFD62F97A, 0x8A65C9EC, 0x14015C4F, 0x63066CD9,
    0xFA0F3D63, 0x8D080DF5, 0x3B6E20C8, 0x4C69105E, 0xD56041E4, 0xA2677172,
    0x3C03E4D1, 0x4B04D4E3, 0xD20D85FD, 0xA50AB56B, 0x35B5A8FA, 0x42B2986C,
    0xDBBBB9D6, 0xACBCB9C2, 0x32D86CE3, 0x45DF5C75, 0xDCD60DCF, 0xABD13D59,
    0x26D930AC, 0x51DE003A, 0xC8D75180, 0xBFD06116, 0x21B4F6B5, 0x56B3C423,
    0xCFBA9599, 0xB8BDA50F, 0x2802B89E, 0x5F058808, 0xC60CD9B2, 0xB10BE924,
    0x2F6F7C87, 0x58684C11, 0xC1611DAB, 0xB6662D3D, 0x76DC4190, 0x01DB7106,
    0x98D220BC, 0xEFD5102A, 0x71B18589, 0x06B6B51F, 0x9FBFE4A5, 0xE8B8D433,
    0x7807C9A2, 0x0F00F934, 0x9609A88E, 0xE10E9818, 0x7F6A0D6B, 0x086D3D2D,
    0x91646C97, 0xE6635C01, 0x6B6B51F4, 0x1C6C6162, 0x856530D8, 0xF262004E,
    0x6C0695ED, 0x1B01A57B, 0x8208F4C1, 0xF50FC457, 0x65B0D9C6, 0x12B7E950,
    0x8BBEB8EA, 0xFCB9887C, 0x62DD1DDF, 0x15DA2D49, 0x8CD37CF3, 0xFBD44C65,
    0x4DB26158, 0x3AB551CE, 0xA3BC0074, 0xD4BB30E2, 0x4ADFA541, 0x3DD895D7,
    0xA4D1C46D, 0xD3D6F4FB, 0x4369E96A, 0x346ED9FC, 0xAD678846, 0xDA60B8D0,
    0x44042D73, 0x33031DE5, 0xAA0A4C5F, 0xDD0D7822, 0x3B6E20C8, 0x4C69105E,
    0xD56041E4, 0xA2677172, 0x3C03E4D1, 0x4B04D4E3, 0xD20D85FD, 0xA50AB56B,
    0x35B5A8FA, 0x42B2986C, 0xDBBBB9D6, 0xACBCB9C2, 0x32D86CE3, 0x45DF5C75,
    0xDCD60DCF, 0xABD13D59, 0x26D930AC, 0x51DE003A, 0xC8D75180, 0xBFD06116,
    0x21B4F6B5, 0x56B3C423, 0xCFBA9599, 0xB8BDA50F, 0x2802B89E, 0x5F058808,
    0xC60CD9B2, 0xB10BE924, 0x2F6F7C87, 0x58684C11, 0xC1611DAB, 0xB6662D3D,
    0x76DC4190, 0x01DB7106, 0x98D220BC, 0xEFD5102A, 0x71B18589, 0x06B6B51F,
    0x9FBFE4A5, 0xE8B8D433, 0x7807C9A2, 0x0F00F934, 0x9609A88E, 0xE10E9818,
    0x7F6A0D6B, 0x086D3D2D, 0x91646C97, 0xE6635C01, 0x6B6B51F4, 0x1C6C6162,
    0x856530D8, 0xF262004E, 0x6C0695ED, 0x1B01A57B, 0x8208F4C1, 0xF50FC457,
    0x65B0D9C6, 0x12B7E950, 0x8BBEB8EA, 0xFCB9887C, 0x62DD1DDF, 0x15DA2D49,
    0x8CD37CF3, 0xFBD44C65, 0x4DB26158, 0x3AB551CE, 0xA3BC0074, 0xD4BB30E2,
    0x4ADFA541, 0x3DD895D7, 0xA4D1C46D, 0xD3D6F4FB, 0x4369E96A, 0x346ED9FC,
    0xAD678846, 0xDA60B8D0, 0x44042D73, 0x33031DE5, 0xAA0A4C5F, 0xDD0D7822,
    0x3B6E20C8, 0x4C69105E, 0xD56041E4, 0xA2677172, 0x3C03E4D1, 0x4B04D4E3,
    0xD20D85FD, 0xA50AB56B, 0x35B5A8FA, 0x42B2986C, 0xDBBBB9D6, 0xACBCB9C2,
    0x32D86CE3, 0x45DF5C75, 0xDCD60DCF, 0xABD13D59, 0x26D930AC, 0x51DE003A,
    0xC8D75180, 0xBFD06116, 0x21B4F6B5, 0x56B3C423, 0xCFBA9599, 0xB8BDA50F,
    0x2802B89E, 0x5F058808, 0xC60CD9B2, 0xB10BE924, 0x2F6F7C87, 0x58684C11,
    0xC1611DAB, 0xB6662D3D, 0x76DC4190, 0x01DB7106, 0x98D220BC, 0xEFD5102A,
    0x71B18589, 0x06B6B51F, 0x9FBFE4A5, 0xE8B8D433, 0x7807C9A2, 0x0F00F934,
    0x9609A88E, 0xE10E9818, 0x7F6A0D6B, 0x086D3D2D, 0x91646C97, 0xE6635C01,
    0x6B6B51F4, 0x1C6C6162, 0x856530D8, 0xF262004E, 0x6C0695ED, 0x1B01A57B,
    0x8208F4C1, 0xF50FC457, 0x65B0D9C6, 0x12B7E950, 0x8BBEB8EA, 0xFCB9887C,
    0x62DD1DDF, 0x15DA2D49, 0x8CD37CF3, 0xFBD44C65,
};

uint32_t FlashPersistManager::ComputeCRC32(const void* data, uint32_t len) {
    const uint8_t* buf = static_cast<const uint8_t*>(data);
    uint32_t crc = 0xFFFFFFFF;
    for (uint32_t i = 0; i < len; ++i) {
        crc = crc32_table[(crc ^ buf[i]) & 0xFF] ^ (crc >> 8);
    }
    return crc ^ 0xFFFFFFFF;
}

// ─── Flash Access Helpers ───────────────────────────────────────────────────

const uint8_t* FlashPersistManager::FlashRead(uint32_t offset) {
    // XIP memory-mapped read: flash data is at XIP_BASE + PERSIST_FLASH_OFFSET + offset
    return reinterpret_cast<const uint8_t*>(
        XIP_BASE + PERSIST_FLASH_OFFSET + offset);
}

bool FlashPersistManager::FlashWrite(uint32_t offset, const void* data, uint32_t len) {
    if (len == 0) return true;

    uint32_t flashOff = PERSIST_FLASH_OFFSET + offset;

    // Erase sectors covering the write range
    uint32_t eraseStart = (flashOff / GpuConfig::FLASH_SECTOR_SIZE)
                        * GpuConfig::FLASH_SECTOR_SIZE;
    uint32_t eraseEnd   = ((flashOff + len + GpuConfig::FLASH_SECTOR_SIZE - 1)
                        / GpuConfig::FLASH_SECTOR_SIZE)
                        * GpuConfig::FLASH_SECTOR_SIZE;
    uint32_t eraseLen   = eraseEnd - eraseStart;

    // Critical section: disable interrupts for flash operations.
    // On RP2350, flash_range_erase/program require this.
    uint32_t ints = save_and_disable_interrupts();

    flash_range_erase(eraseStart, eraseLen);

    // Program in page-aligned chunks
    const uint8_t* src = static_cast<const uint8_t*>(data);
    uint32_t remaining = len;
    uint32_t writeOff  = flashOff;

    // Align to page boundary if needed (use a staging buffer)
    static uint8_t pageBuffer[GpuConfig::FLASH_PAGE_SIZE]
        __attribute__((aligned(4)));

    while (remaining > 0) {
        uint32_t pageOff = writeOff % GpuConfig::FLASH_PAGE_SIZE;
        uint32_t chunkSize = GpuConfig::FLASH_PAGE_SIZE - pageOff;
        if (chunkSize > remaining) chunkSize = remaining;

        // If we're writing a partial page, read-modify-write
        if (pageOff > 0 || chunkSize < GpuConfig::FLASH_PAGE_SIZE) {
            memset(pageBuffer, 0xFF, GpuConfig::FLASH_PAGE_SIZE);
            memcpy(pageBuffer + pageOff, src, chunkSize);
            uint32_t pageBase = (writeOff / GpuConfig::FLASH_PAGE_SIZE)
                              * GpuConfig::FLASH_PAGE_SIZE;
            flash_range_program(pageBase, pageBuffer, GpuConfig::FLASH_PAGE_SIZE);
        } else {
            flash_range_program(writeOff, src, GpuConfig::FLASH_PAGE_SIZE);
        }

        src       += chunkSize;
        writeOff  += chunkSize;
        remaining -= chunkSize;
    }

    restore_interrupts(ints);
    return true;
}

bool FlashPersistManager::FlashErase(uint32_t offset, uint32_t len) {
    uint32_t flashOff = PERSIST_FLASH_OFFSET + offset;
    uint32_t eraseStart = (flashOff / GpuConfig::FLASH_SECTOR_SIZE)
                        * GpuConfig::FLASH_SECTOR_SIZE;
    uint32_t eraseEnd   = ((flashOff + len + GpuConfig::FLASH_SECTOR_SIZE - 1)
                        / GpuConfig::FLASH_SECTOR_SIZE)
                        * GpuConfig::FLASH_SECTOR_SIZE;

    uint32_t ints = save_and_disable_interrupts();
    flash_range_erase(eraseStart, eraseEnd - eraseStart);
    restore_interrupts(ints);
    return true;
}

// ─── Initialization ─────────────────────────────────────────────────────────

bool FlashPersistManager::Initialize(MemTierManager* tierMgr) {
    tierMgr_ = tierMgr;

    // Read existing manifest from flash
    const uint8_t* flashPtr = FlashRead(0);
    memcpy(&manifestHeader_, flashPtr, sizeof(FlashManifestHeader));

    // Validate manifest
    bool validManifest = false;
    if (manifestHeader_.magic == FLASH_MANIFEST_MAGIC &&
        manifestHeader_.version == FLASH_MANIFEST_VERSION &&
        manifestHeader_.entryCount <= MAX_ENTRIES) {
        // Verify CRC (over header excluding the CRC field itself)
        uint32_t savedCrc = manifestHeader_.crc32;
        manifestHeader_.crc32 = 0;
        uint32_t computedCrc = ComputeCRC32(&manifestHeader_,
                                             sizeof(FlashManifestHeader));
        manifestHeader_.crc32 = savedCrc;

        if (computedCrc == savedCrc) {
            validManifest = true;
        }
    }

    if (validManifest) {
        // Load entries
        const uint8_t* entryPtr = flashPtr + sizeof(FlashManifestHeader);
        memcpy(entries_, entryPtr,
               sizeof(FlashManifestEntry) * manifestHeader_.entryCount);

        printf("[FlashPersist] Loaded manifest: %u entries, %lu bytes used\n",
               manifestHeader_.entryCount,
               (unsigned long)manifestHeader_.dataUsedBytes);
    } else {
        // Initialize empty manifest
        memset(&manifestHeader_, 0, sizeof(FlashManifestHeader));
        manifestHeader_.magic           = FLASH_MANIFEST_MAGIC;
        manifestHeader_.version         = FLASH_MANIFEST_VERSION;
        manifestHeader_.entryCount      = 0;
        manifestHeader_.dataStartOffset = DATA_REGION_OFFSET;
        manifestHeader_.dataUsedBytes   = 0;

        memset(entries_, 0, sizeof(entries_));

        printf("[FlashPersist] No valid manifest found — initialized empty\n");
    }

    // Clear writeback queue
    for (uint8_t i = 0; i < MAX_QUEUE; ++i) {
        writebackQueue_[i] = {};
    }
    queueHead_  = 0;
    queueTail_  = 0;
    queueCount_ = 0;

    currentState_   = PGL_PERSIST_IDLE;
    lastResourceId_ = 0;
    initialized_    = true;

    return true;
}

// ─── Manifest I/O ───────────────────────────────────────────────────────────

bool FlashPersistManager::WriteManifest() {
    // Compute CRC over header (with crc32 field zeroed)
    manifestHeader_.crc32 = 0;
    manifestHeader_.crc32 = ComputeCRC32(&manifestHeader_,
                                          sizeof(FlashManifestHeader));

    // Build write buffer: header + entries
    static uint8_t manifestBuf[sizeof(FlashManifestHeader) +
                                sizeof(FlashManifestEntry) * GpuConfig::FLASH_MAX_ENTRIES]
        __attribute__((aligned(4)));

    memcpy(manifestBuf, &manifestHeader_, sizeof(FlashManifestHeader));
    memcpy(manifestBuf + sizeof(FlashManifestHeader),
           entries_,
           sizeof(FlashManifestEntry) * manifestHeader_.entryCount);

    uint32_t totalSize = sizeof(FlashManifestHeader) +
        sizeof(FlashManifestEntry) * manifestHeader_.entryCount;

    return FlashWrite(0, manifestBuf, totalSize);
}

// ─── Entry Lookup ───────────────────────────────────────────────────────────

int FlashPersistManager::FindEntry(uint8_t resourceClass,
                                    uint16_t resourceId) const {
    for (uint16_t i = 0; i < manifestHeader_.entryCount; ++i) {
        if ((entries_[i].flags & FLASH_ENTRY_VALID) &&
            entries_[i].resourceClass == resourceClass &&
            entries_[i].resourceId == resourceId) {
            return static_cast<int>(i);
        }
    }
    return -1;
}

bool FlashPersistManager::IsResourcePersisted(uint8_t resourceClass,
                                               uint16_t resourceId) const {
    return FindEntry(resourceClass, resourceId) >= 0;
}

// ─── Data Space Allocation ──────────────────────────────────────────────────

uint32_t FlashPersistManager::AllocDataSpace(uint32_t size) {
    // Simple bump allocator within the data region.
    // Fragmentation is not an issue since EraseAll resets everything.
    uint32_t offset = manifestHeader_.dataStartOffset +
                      manifestHeader_.dataUsedBytes;
    uint32_t endOffset = offset + size;

    if (endOffset > GpuConfig::FLASH_PERSIST_SIZE) {
        printf("[FlashPersist] ERROR: data region full (%lu + %lu > %lu)\n",
               (unsigned long)offset, (unsigned long)size,
               (unsigned long)GpuConfig::FLASH_PERSIST_SIZE);
        return 0xFFFFFFFF;
    }

    return manifestHeader_.dataUsedBytes;  // offset relative to data region start
}

// ─── Persist Resource ───────────────────────────────────────────────────────

bool FlashPersistManager::PersistResource(uint8_t resourceClass,
                                           uint16_t resourceId,
                                           uint8_t flags,
                                           const void* dataPtr,
                                           uint32_t dataSize) {
    if (!initialized_ || !dataPtr || dataSize == 0) return false;

    // Check if already persisted
    int existingIdx = FindEntry(resourceClass, resourceId);
    if (existingIdx >= 0 && !(flags & PGL_PERSIST_OVERWRITE)) {
        printf("[FlashPersist] Resource %u:%u already persisted (use OVERWRITE flag)\n",
               resourceClass, resourceId);
        return false;
    }

    // Check queue capacity
    if (queueCount_ >= MAX_QUEUE) {
        printf("[FlashPersist] Writeback queue full (%u/%u)\n",
               queueCount_, MAX_QUEUE);
        return false;
    }

    // Enqueue
    WritebackRequest& req = writebackQueue_[queueTail_];
    req.active        = true;
    req.resourceClass = resourceClass;
    req.resourceId    = resourceId;
    req.flags         = flags;
    req.dataPtr       = dataPtr;
    req.dataSize      = dataSize;

    queueTail_ = (queueTail_ + 1) % MAX_QUEUE;
    queueCount_++;

    printf("[FlashPersist] Queued persist: class=%u id=%u size=%lu (%u in queue)\n",
           resourceClass, resourceId, (unsigned long)dataSize, queueCount_);

    return true;
}

// ─── Writeback Queue Processing ─────────────────────────────────────────────

uint8_t FlashPersistManager::ProcessWritebackQueue() {
    if (queueCount_ == 0) return 0;

    WritebackRequest& req = writebackQueue_[queueHead_];
    if (!req.active) {
        // Stale entry — skip
        queueHead_ = (queueHead_ + 1) % MAX_QUEUE;
        queueCount_--;
        return queueCount_;
    }

    currentState_ = PGL_PERSIST_WRITING;

    // Check for existing entry (overwrite case)
    int existingIdx = FindEntry(req.resourceClass, req.resourceId);

    // Allocate data space
    uint32_t dataOffset = AllocDataSpace(req.dataSize);
    if (dataOffset == 0xFFFFFFFF) {
        printf("[FlashPersist] Failed to allocate data space for %u:%u\n",
               req.resourceClass, req.resourceId);
        currentState_ = PGL_PERSIST_ERROR;
        req.active = false;
        queueHead_ = (queueHead_ + 1) % MAX_QUEUE;
        queueCount_--;
        return queueCount_;
    }

    // Compute CRC of resource data
    uint32_t crc = ComputeCRC32(req.dataPtr, req.dataSize);

    // Write data to flash
    uint32_t flashDataOffset = manifestHeader_.dataStartOffset + dataOffset;
    if (!FlashWrite(flashDataOffset, req.dataPtr, req.dataSize)) {
        printf("[FlashPersist] Flash write failed for %u:%u\n",
               req.resourceClass, req.resourceId);
        currentState_ = PGL_PERSIST_ERROR;
        req.active = false;
        queueHead_ = (queueHead_ + 1) % MAX_QUEUE;
        queueCount_--;
        return queueCount_;
    }

    // Update or create manifest entry
    FlashManifestEntry entry = {};
    entry.resourceClass = req.resourceClass;
    entry.resourceId    = req.resourceId;
    entry.flags         = FLASH_ENTRY_VALID;
    entry.dataOffset    = dataOffset;
    entry.dataSize      = req.dataSize;
    entry.originalSize  = req.dataSize;
    entry.crc32         = crc;
    entry.tier          = 0;  // Will be set from original tier if available

    if (existingIdx >= 0) {
        // Overwrite existing entry (note: old data space is wasted — no GC)
        entries_[existingIdx] = entry;
    } else if (manifestHeader_.entryCount < MAX_ENTRIES) {
        entries_[manifestHeader_.entryCount] = entry;
        manifestHeader_.entryCount++;
    } else {
        printf("[FlashPersist] Manifest full (%u entries)\n", MAX_ENTRIES);
        currentState_ = PGL_PERSIST_ERROR;
        req.active = false;
        queueHead_ = (queueHead_ + 1) % MAX_QUEUE;
        queueCount_--;
        return queueCount_;
    }

    // Update data usage
    manifestHeader_.dataUsedBytes += req.dataSize;

    // Write updated manifest
    WriteManifest();

    lastResourceId_ = req.resourceId;
    currentState_   = PGL_PERSIST_IDLE;

    printf("[FlashPersist] Persisted %u:%u (%lu bytes, CRC=0x%08lX)\n",
           req.resourceClass, req.resourceId,
           (unsigned long)req.dataSize, (unsigned long)crc);

    // Dequeue
    req.active = false;
    queueHead_ = (queueHead_ + 1) % MAX_QUEUE;
    queueCount_--;

    return queueCount_;
}

// ─── Restore Resource ───────────────────────────────────────────────────────

const void* FlashPersistManager::RestoreResource(uint8_t resourceClass,
                                                  uint16_t resourceId,
                                                  void* destPtr,
                                                  uint32_t destSize) {
    if (!initialized_) return nullptr;

    int idx = FindEntry(resourceClass, resourceId);
    if (idx < 0) {
        printf("[FlashPersist] Resource %u:%u not found in manifest\n",
               resourceClass, resourceId);
        return nullptr;
    }

    currentState_ = PGL_PERSIST_RESTORING;

    const FlashManifestEntry& entry = entries_[idx];

    // Read data from flash (XIP mapped)
    const uint8_t* flashData = FlashRead(
        manifestHeader_.dataStartOffset + entry.dataOffset);

    // Verify CRC
    uint32_t crc = ComputeCRC32(flashData, entry.dataSize);
    if (crc != entry.crc32) {
        printf("[FlashPersist] CRC mismatch for %u:%u (expected 0x%08lX, got 0x%08lX)\n",
               resourceClass, resourceId,
               (unsigned long)entry.crc32, (unsigned long)crc);
        currentState_ = PGL_PERSIST_ERROR;
        return nullptr;
    }

    // Copy to destination
    if (destPtr) {
        uint32_t copySize = (destSize > 0 && destSize < entry.dataSize)
            ? destSize : entry.dataSize;
        memcpy(destPtr, flashData, copySize);
        lastResourceId_ = resourceId;
        currentState_   = PGL_PERSIST_IDLE;
        printf("[FlashPersist] Restored %u:%u (%lu bytes)\n",
               resourceClass, resourceId, (unsigned long)copySize);
        return destPtr;
    }

    // No destination provided — return XIP-mapped pointer directly.
    // This is read-only and may be slow for random access.
    lastResourceId_ = resourceId;
    currentState_   = PGL_PERSIST_IDLE;
    printf("[FlashPersist] Restored %u:%u (%lu bytes, XIP-mapped)\n",
           resourceClass, resourceId, (unsigned long)entry.dataSize);
    return flashData;
}

// ─── Auto-Restore ───────────────────────────────────────────────────────────

uint16_t FlashPersistManager::AutoRestore() {
    if (!initialized_ || manifestHeader_.entryCount == 0) return 0;

    uint16_t restored = 0;

    printf("[FlashPersist] Auto-restoring %u persisted resources...\n",
           manifestHeader_.entryCount);

    for (uint16_t i = 0; i < manifestHeader_.entryCount; ++i) {
        const FlashManifestEntry& entry = entries_[i];
        if (!(entry.flags & FLASH_ENTRY_VALID)) continue;

        // Verify CRC before restoring
        const uint8_t* flashData = FlashRead(
            manifestHeader_.dataStartOffset + entry.dataOffset);
        uint32_t crc = ComputeCRC32(flashData, entry.dataSize);
        if (crc != entry.crc32) {
            printf("[FlashPersist] Auto-restore: CRC failed for %u:%u — skipping\n",
                   entry.resourceClass, entry.resourceId);
            continue;
        }

        // If tier manager is available, register and place the resource
        if (tierMgr_) {
            uint16_t recIdx = tierMgr_->Register(
                entry.resourceId,
                static_cast<ResClass>(entry.resourceClass),
                entry.dataSize);

            if (recIdx != 0xFFFF) {
                // Get writable pointer and copy data
                void* dest = tierMgr_->Write(entry.resourceId);
                if (dest) {
                    memcpy(dest, flashData, entry.dataSize);
                    restored++;
                }
            }
        } else {
            // No tier manager — resource is available via XIP-mapped read
            restored++;
        }
    }

    printf("[FlashPersist] Auto-restore complete: %u/%u resources restored\n",
           restored, manifestHeader_.entryCount);

    return restored;
}

// ─── Status Query ───────────────────────────────────────────────────────────

void FlashPersistManager::GetStatus(PglMemPersistStatusResponse& status) const {
    status.state            = currentState_;
    status.queueDepth       = queueCount_;
    status.manifestEntries  = manifestHeader_.entryCount;
    status.manifestCapacity = MAX_ENTRIES;
    status.flashUsedBytes   = manifestHeader_.dataUsedBytes;
    status.lastResourceId   = lastResourceId_;
}

// ─── Erase All ──────────────────────────────────────────────────────────────

void FlashPersistManager::EraseAll() {
    printf("[FlashPersist] Erasing entire persistence region (%lu KB)...\n",
           (unsigned long)(GpuConfig::FLASH_PERSIST_SIZE / 1024));

    FlashErase(0, GpuConfig::FLASH_PERSIST_SIZE);

    // Re-initialize empty manifest
    memset(&manifestHeader_, 0, sizeof(FlashManifestHeader));
    manifestHeader_.magic           = FLASH_MANIFEST_MAGIC;
    manifestHeader_.version         = FLASH_MANIFEST_VERSION;
    manifestHeader_.entryCount      = 0;
    manifestHeader_.dataStartOffset = DATA_REGION_OFFSET;
    manifestHeader_.dataUsedBytes   = 0;

    memset(entries_, 0, sizeof(entries_));

    WriteManifest();

    printf("[FlashPersist] Persistence region erased and manifest reset\n");
}
