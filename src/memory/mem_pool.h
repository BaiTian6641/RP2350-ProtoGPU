/**
 * @file mem_pool.h
 * @brief Fixed-size block pool allocator — O(1) alloc/free, zero fragmentation (M11).
 *
 * Each MemPool is a contiguous allocation from a memory tier, subdivided into
 * fixed-size blocks. A free-list provides O(1) alloc/free with no external
 * fragmentation within the pool.
 *
 * Wire protocol:
 *   CMD_MEM_POOL_CREATE  (0x38) → MemPoolManager::CreatePool()
 *   CMD_MEM_POOL_ALLOC   (0x39) → MemPoolManager::Alloc()
 *   CMD_MEM_POOL_FREE    (0x3A) → MemPoolManager::Free()
 *   CMD_MEM_POOL_DESTROY (0x3B) → MemPoolManager::DestroyPool()
 *   PGL_REG_MEM_POOL_STATUS (I2C) → MemPoolManager::GetStatus()
 *
 * Limits: PGL_MAX_MEM_POOLS = 16 concurrent pools.
 *
 * Memory layout of a pool:
 *   ┌──────────────────────────────────────────────────┐
 *   │ Block 0   │ Block 1   │ ... │ Block N-1          │
 *   └──────────────────────────────────────────────────┘
 *   Each block has `blockSize` bytes. When free, the first 2 bytes
 *   store the next-free index (intrusive free-list).
 *   When allocated, the full block is available to the user.
 */

#pragma once

#include <cstdint>
#include <cstdio>
#include <cstring>
#include <cstdlib>
#include <PglTypes.h>

// ─── Single Pool Instance ───────────────────────────────────────────────────

struct MemPool {
    bool     active      = false;
    uint8_t  tier        = 0;           ///< PglMemTier where pool resides
    uint16_t blockSize   = 0;           ///< Bytes per block
    uint16_t blockCount  = 0;           ///< Total blocks
    uint16_t freeCount   = 0;           ///< Currently free
    uint16_t tag         = 0;           ///< User-defined tag
    uint16_t freeHead    = 0xFFFF;      ///< Head of free-list (block index, 0xFFFF = empty)
    uint8_t* memory      = nullptr;     ///< Backing allocation (blockSize × blockCount bytes)

    /// Initialize the pool: allocate memory and build the free-list.
    bool Init(uint8_t poolTier, uint16_t blkSize, uint16_t blkCount, uint16_t poolTag) {
        if (blkSize < 2) blkSize = 2;  // Minimum: 2 bytes for free-list link

        uint32_t totalBytes = static_cast<uint32_t>(blkSize) * blkCount;
        if (totalBytes == 0 || totalBytes > 256 * 1024) return false;  // Sanity cap: 256 KB

        memory = static_cast<uint8_t*>(malloc(totalBytes));
        if (!memory) return false;

        tier       = poolTier;
        blockSize  = blkSize;
        blockCount = blkCount;
        freeCount  = blkCount;
        tag        = poolTag;
        active     = true;

        // Build intrusive free-list: each free block's first 2 bytes = next index
        for (uint16_t i = 0; i < blkCount; ++i) {
            uint16_t nextIdx = (i + 1 < blkCount) ? (i + 1) : 0xFFFF;
            uint8_t* blk = memory + static_cast<uint32_t>(i) * blkSize;
            memcpy(blk, &nextIdx, sizeof(uint16_t));
        }
        freeHead = 0;

        return true;
    }

    /// Allocate one block. Returns block index, or 0xFFFF on failure.
    uint16_t Alloc() {
        if (freeHead == 0xFFFF || freeCount == 0) return 0xFFFF;

        uint16_t idx = freeHead;
        uint8_t* blk = memory + static_cast<uint32_t>(idx) * blockSize;

        // Follow the free-list link
        memcpy(&freeHead, blk, sizeof(uint16_t));
        freeCount--;

        // Zero the block for the caller
        memset(blk, 0, blockSize);

        return idx;
    }

    /// Free one block by index. Returns true on success.
    bool Free(uint16_t blockIndex) {
        if (blockIndex >= blockCount) return false;

        uint8_t* blk = memory + static_cast<uint32_t>(blockIndex) * blockSize;

        // Push onto free-list head
        memcpy(blk, &freeHead, sizeof(uint16_t));
        freeHead = blockIndex;
        freeCount++;

        return true;
    }

    /// Get a pointer to a block's data by index. Returns nullptr on invalid index.
    uint8_t* GetBlock(uint16_t blockIndex) {
        if (!active || blockIndex >= blockCount || !memory) return nullptr;
        return memory + static_cast<uint32_t>(blockIndex) * blockSize;
    }

    /// Destroy the pool: free backing memory.
    void Destroy() {
        if (memory) {
            free(memory);
            memory = nullptr;
        }
        active    = false;
        freeHead  = 0xFFFF;
        freeCount = 0;
    }
};

// ─── Pool Manager (up to PGL_MAX_MEM_POOLS) ────────────────────────────────

class MemPoolManager {
public:
    static constexpr uint16_t MAX_POOLS = PGL_MAX_MEM_POOLS;  // 16

    MemPoolManager() = default;

    /**
     * @brief Create a new pool.
     *
     * @param tier        PglMemTier where pool should reside.
     * @param blockSize   Bytes per block (must be >= 2).
     * @param blockCount  Number of blocks to pre-allocate.
     * @param tag         User-defined tag.
     * @return Pool handle (0-based index), or 0xFFFF on failure.
     */
    uint16_t CreatePool(uint8_t tier, uint16_t blockSize,
                        uint16_t blockCount, uint16_t tag) {
        // Find a free slot
        for (uint16_t i = 0; i < MAX_POOLS; ++i) {
            if (!pools_[i].active) {
                if (pools_[i].Init(tier, blockSize, blockCount, tag)) {
                    printf("[MemPool] Created pool %u: tier=%u, blkSize=%u, blkCount=%u, tag=0x%04X\n",
                           i, tier, blockSize, blockCount, tag);
                    return i;
                }
                printf("[MemPool] Failed to init pool %u (OOM?)\n", i);
                return 0xFFFF;
            }
        }
        printf("[MemPool] No free pool slots (max %u)\n", MAX_POOLS);
        return 0xFFFF;
    }

    /**
     * @brief Allocate one block from a pool.
     *
     * @param poolHandle  Pool handle (from CreatePool).
     * @return Block index within the pool, or 0xFFFF on failure.
     */
    uint16_t Alloc(uint16_t poolHandle) {
        if (poolHandle >= MAX_POOLS || !pools_[poolHandle].active) return 0xFFFF;
        return pools_[poolHandle].Alloc();
    }

    /**
     * @brief Free one block back to a pool.
     *
     * @param poolHandle  Pool handle.
     * @param blockIndex  Block index (from Alloc).
     * @return true on success.
     */
    bool Free(uint16_t poolHandle, uint16_t blockIndex) {
        if (poolHandle >= MAX_POOLS || !pools_[poolHandle].active) return false;
        return pools_[poolHandle].Free(blockIndex);
    }

    /**
     * @brief Destroy a pool and free its backing memory.
     *
     * @param poolHandle  Pool handle.
     */
    void DestroyPool(uint16_t poolHandle) {
        if (poolHandle >= MAX_POOLS) return;
        if (pools_[poolHandle].active) {
            printf("[MemPool] Destroying pool %u (tag=0x%04X)\n",
                   poolHandle, pools_[poolHandle].tag);
            pools_[poolHandle].Destroy();
        }
    }

    /**
     * @brief Get status of a pool for I2C readback.
     *
     * @param poolHandle  Pool handle.
     * @param[out] resp   Response to fill.
     */
    void GetStatus(uint16_t poolHandle, PglMemPoolStatusResponse& resp) const {
        memset(&resp, 0, sizeof(resp));
        resp.poolHandle = static_cast<PglPool>(poolHandle);

        if (poolHandle >= MAX_POOLS || !pools_[poolHandle].active) {
            resp.status = PGL_POOL_INVALID_HANDLE;
            return;
        }

        const MemPool& p = pools_[poolHandle];
        resp.tier       = p.tier;
        resp.blockSize  = p.blockSize;
        resp.blockCount = p.blockCount;
        resp.freeCount  = p.freeCount;
        resp.tag        = p.tag;
        resp.status     = (p.freeCount == 0) ? PGL_POOL_EXHAUSTED : PGL_POOL_OK;
    }

    /**
     * @brief Get a pointer to a specific block.
     */
    uint8_t* GetBlock(uint16_t poolHandle, uint16_t blockIndex) {
        if (poolHandle >= MAX_POOLS) return nullptr;
        return pools_[poolHandle].GetBlock(blockIndex);
    }

    /**
     * @brief Destroy all pools (called on GPU reset).
     */
    void DestroyAll() {
        for (uint16_t i = 0; i < MAX_POOLS; ++i) {
            if (pools_[i].active) pools_[i].Destroy();
        }
    }

    /// Get a pool reference (for direct inspection).
    const MemPool* GetPool(uint16_t poolHandle) const {
        if (poolHandle >= MAX_POOLS) return nullptr;
        return &pools_[poolHandle];
    }

private:
    MemPool pools_[MAX_POOLS] = {};
};
