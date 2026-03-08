/**
 * @file flash_persist.h
 * @brief Flash persistence manager — manifest + writeback queue (M12).
 *
 * Provides non-volatile storage for GPU resources in the RP2350 onboard flash.
 * Resources can be persisted (saved) and restored (loaded) across power cycles.
 *
 * Flash layout (last 512 KB of flash):
 *   ┌────────────────────────────────────────────────┐ FLASH_PERSIST_OFFSET
 *   │ ManifestHeader (32 bytes)                      │
 *   ├────────────────────────────────────────────────┤
 *   │ ManifestEntry[0] (32 bytes)                    │
 *   │ ManifestEntry[1]                               │
 *   │ ...                                            │
 *   │ ManifestEntry[63]                              │ (max 64 entries)
 *   ├────────────────────────────────────────────────┤ +2080 bytes (rounded to 4KB sector)
 *   │ Data region (remaining ~508 KB)                │
 *   │ [resource data stored sequentially]            │
 *   └────────────────────────────────────────────────┘ end of flash
 *
 * Wire protocol:
 *   CMD_PERSIST_RESOURCE  (0x46) → PersistResource()
 *   CMD_RESTORE_RESOURCE  (0x47) → RestoreResource()
 *   CMD_QUERY_PERSISTENCE (0x48) → QueryPersistence()
 *   PGL_REG_MEM_PERSIST_STATUS (I2C) → GetStatus()
 *
 * Safety:
 *   - Flash writes are sector-aligned (4 KB erase, 256 B program).
 *   - CRC-32 protects both manifest header and each entry.
 *   - A writeback queue buffers up to 4 pending persist operations.
 *   - Flash operations must be called with interrupts disabled on the
 *     executing core (Pico SDK requirement). The manager handles this.
 *   - MRAM zero-cost path: if the resource resides in QSPI MRAM (non-volatile),
 *     the persist operation is a no-op (already persistent).
 */

#pragma once

#include <cstdint>
#include <PglTypes.h>
#include "../gpu_config.h"

// Forward declarations
class MemTierManager;

// ─── Flash Manifest Structures ──────────────────────────────────────────────

/// Manifest header stored at the start of the persistence region.
struct FlashManifestHeader {
    uint32_t magic;             ///< Magic number: 'P','G','L','F' = 0x464C4750
    uint16_t version;           ///< Manifest format version (1)
    uint16_t entryCount;        ///< Number of valid entries
    uint32_t dataStartOffset;   ///< Byte offset (from persist region) where data begins
    uint32_t dataUsedBytes;     ///< Total bytes used in data region
    uint32_t crc32;             ///< CRC-32 of header (excluding this field)
    uint8_t  reserved[12];      ///< Pad to 32 bytes
};
static_assert(sizeof(FlashManifestHeader) == 32, "FlashManifestHeader must be 32 bytes");

static constexpr uint32_t FLASH_MANIFEST_MAGIC   = 0x464C4750;  // 'PGLF'
static constexpr uint16_t FLASH_MANIFEST_VERSION  = 1;

/// Single manifest entry describing a persisted resource.
struct FlashManifestEntry {
    uint8_t  resourceClass;     ///< PglMemResourceClass
    uint8_t  flags;             ///< bit0: valid, bit1: compressed
    uint16_t resourceId;        ///< Resource handle
    uint32_t dataOffset;        ///< Byte offset into data region
    uint32_t dataSize;          ///< Size of persisted data in bytes
    uint32_t originalSize;      ///< Original (uncompressed) size
    uint32_t crc32;             ///< CRC-32 of the persisted data
    uint8_t  tier;              ///< Original memory tier
    uint8_t  reserved[7];       ///< Pad to 32 bytes
};
static_assert(sizeof(FlashManifestEntry) == 32, "FlashManifestEntry must be 32 bytes");

enum FlashEntryFlags : uint8_t {
    FLASH_ENTRY_VALID      = 0x01,
    FLASH_ENTRY_COMPRESSED = 0x02,
};

// ─── Writeback Queue ────────────────────────────────────────────────────────

struct WritebackRequest {
    bool     active;
    uint8_t  resourceClass;
    uint16_t resourceId;
    uint8_t  flags;             ///< PglPersistFlags
    const void* dataPtr;        ///< Pointer to resource data in SRAM
    uint32_t dataSize;
};

// ─── Flash Persistence Manager ──────────────────────────────────────────────

class FlashPersistManager {
public:
    static constexpr uint16_t MAX_ENTRIES = GpuConfig::FLASH_MAX_ENTRIES;
    static constexpr uint8_t  MAX_QUEUE   = GpuConfig::FLASH_WRITEBACK_QUEUE_DEPTH;

    FlashPersistManager() = default;

    // ── Initialization ──────────────────────────────────────────────────

    /**
     * @brief Initialize the persistence manager.
     *
     * Reads the flash manifest (if valid) and populates the in-memory
     * entry table. If no valid manifest is found, initializes an empty one.
     *
     * @param tierMgr  Pointer to the memory tier manager (for data access).
     * @return true on success.
     */
    bool Initialize(MemTierManager* tierMgr = nullptr);

    // ── Resource Persistence ────────────────────────────────────────────

    /**
     * @brief Queue a resource for flash writeback.
     *
     * If the resource is already persisted and overwrite is not set, returns
     * false. The actual flash write happens during ProcessWritebackQueue().
     *
     * @param resourceClass  PglMemResourceClass.
     * @param resourceId     Resource handle.
     * @param flags          PglPersistFlags (bit0 = overwrite).
     * @param dataPtr        Pointer to resource data.
     * @param dataSize       Size of data in bytes.
     * @return true if queued successfully.
     */
    bool PersistResource(uint8_t resourceClass, uint16_t resourceId,
                         uint8_t flags, const void* dataPtr, uint32_t dataSize);

    /**
     * @brief Restore a resource from flash to memory.
     *
     * Looks up the resource in the manifest and copies its data to the
     * provided destination buffer (or allocates via tier manager).
     *
     * @param resourceClass  PglMemResourceClass.
     * @param resourceId     Resource handle.
     * @param destPtr        Destination buffer (nullptr = allocate via tier manager).
     * @param destSize       Size of destination buffer.
     * @return Pointer to restored data, or nullptr on failure.
     */
    const void* RestoreResource(uint8_t resourceClass, uint16_t resourceId,
                                void* destPtr = nullptr, uint32_t destSize = 0);

    /**
     * @brief Check if a resource is persisted in flash.
     */
    bool IsResourcePersisted(uint8_t resourceClass, uint16_t resourceId) const;

    // ── Writeback Queue Processing ──────────────────────────────────────

    /**
     * @brief Process pending writeback requests.
     *
     * Call this periodically (e.g., once per frame) to drain the queue.
     * Performs at most one flash write per call to avoid stalling the pipeline.
     *
     * @return Number of requests remaining in queue.
     */
    uint8_t ProcessWritebackQueue();

    // ── Boot-Time Auto-Restore ──────────────────────────────────────────

    /**
     * @brief Restore all persisted resources at boot time.
     *
     * Iterates the manifest and restores each valid entry to its original
     * memory tier. Called during GpuCore::Initialize() after tier manager init.
     *
     * @return Number of resources restored.
     */
    uint16_t AutoRestore();

    // ── Status / Query ──────────────────────────────────────────────────

    /**
     * @brief Fill a PglMemPersistStatusResponse for I2C readback.
     */
    void GetStatus(PglMemPersistStatusResponse& status) const;

    /**
     * @brief Get total number of persisted entries.
     */
    uint16_t GetEntryCount() const { return manifestHeader_.entryCount; }

    /**
     * @brief Get flash bytes used.
     */
    uint32_t GetFlashUsed() const { return manifestHeader_.dataUsedBytes; }

    /**
     * @brief Erase all persisted data and reset manifest.
     */
    void EraseAll();

private:
    MemTierManager* tierMgr_ = nullptr;
    bool initialized_ = false;

    // In-memory copy of manifest
    FlashManifestHeader manifestHeader_ = {};
    FlashManifestEntry  entries_[MAX_ENTRIES] = {};

    // Writeback queue
    WritebackRequest writebackQueue_[MAX_QUEUE] = {};
    uint8_t queueHead_ = 0;
    uint8_t queueTail_ = 0;
    uint8_t queueCount_ = 0;

    // Last completed resource (for status reporting)
    uint16_t lastResourceId_ = 0;
    uint8_t  currentState_   = PGL_PERSIST_IDLE;

    // ── Internal helpers ────────────────────────────────────────────────

    /// Compute CRC-32 of a data block.
    static uint32_t ComputeCRC32(const void* data, uint32_t len);

    /// Read flash at offset (via XIP).
    static const uint8_t* FlashRead(uint32_t offset);

    /// Write data to flash (handles sector erase + page program).
    static bool FlashWrite(uint32_t offset, const void* data, uint32_t len);

    /// Erase flash sectors covering [offset, offset+len).
    static bool FlashErase(uint32_t offset, uint32_t len);

    /// Write manifest header + entries to flash.
    bool WriteManifest();

    /// Find entry by class + id. Returns index or -1.
    int FindEntry(uint8_t resourceClass, uint16_t resourceId) const;

    /// Allocate space in data region. Returns offset or 0xFFFFFFFF on failure.
    uint32_t AllocDataSpace(uint32_t size);
};

#endif // FLASH_PERSIST_H — intentionally not used, #pragma once above
