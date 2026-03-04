/**
 * @file octal_spi_rx.cpp
 * @brief PIO-driven Octal SPI receiver implementation for RP2350.
 *
 * Architecture:
 *   PIO1/SM0 — octal_spi_rx program samples D0-D7 on CLK rising edge when CS low
 *   DMA channel — drains PIO RX FIFO into a 32 KB ring buffer (hardware ring mode)
 *
 * Ring buffer uses DMA ring mode: the DMA write address wraps at a power-of-2
 * boundary.  The write position is derived from the DMA transfer count register.
 *
 * Frame extraction:
 *   1. Compute available bytes from DMA write pointer vs software read pointer
 *   2. Scan for PGL_SYNC_WORD (0x55AA, little-endian)
 *   3. Read PglFrameHeader to get totalLength
 *   4. If enough data → validate CRC → return frame to caller
 *   5. ConsumeFrame() advances readPos past the frame
 *
 * Ring buffer wrap-around handling:
 *   When a frame straddles the ring buffer boundary, we memcpy into a staging
 *   buffer to present a contiguous frame to the parser.
 */

#include "octal_spi_rx.h"
#include "../gpu_config.h"

// Shared ProtoGL headers (from lib/ProtoGL/src/)
#include <PglTypes.h>
#include <PglParser.h>
#include <PglCRC16.h>

#include "pico/stdlib.h"
#include "hardware/pio.h"
#include "hardware/dma.h"
#include "hardware/gpio.h"

// Generated PIO header (from octal_spi_rx.pio via pico_generate_pio_header)
#include "octal_spi_rx.pio.h"

#include <cstring>
#include <cstdio>

// ─── Constants ──────────────────────────────────────────────────────────────

static constexpr uint32_t RING_SIZE = GpuConfig::SPI_RING_BUFFER_SIZE;  // 32768
// DMA ring mode requires log2(size).  32768 = 2^15.
static constexpr uint     RING_LOG2 = 15;
// Verify at compile time
static_assert((1u << RING_LOG2) == RING_SIZE, "RING_SIZE must be 2^RING_LOG2");

// Maximum frame size (half the ring buffer — anything larger is implausible)
static constexpr uint32_t MAX_FRAME_SIZE = RING_SIZE / 2;

// ─── State ──────────────────────────────────────────────────────────────────

static PIO  pio_instance = pio1;  // Use PIO1 (PIO0 is used by HUB75)
static uint sm = 0;
static uint prog_offset = 0;
static int  dma_channel = -1;

/// Ring buffer in SRAM — must be aligned to RING_SIZE for DMA ring mode
static uint8_t __attribute__((aligned(32768))) ringBuffer[RING_SIZE];

/// Software read position (advanced by ConsumeFrame)
static uint32_t readPos = 0;

static uint64_t totalBytesReceived = 0;

/// Staging buffer for frames that wrap around the ring boundary
static uint8_t stagingBuffer[MAX_FRAME_SIZE];

/// Frame extraction state
static const uint8_t* pendingFramePtr = nullptr;
static uint32_t pendingFrameLength = 0;
static uint32_t pendingFrameRingStart = 0;

// ─── DMA Write Position ────────────────────────────────────────────────────

/**
 * @brief Get the current DMA write position in the ring buffer.
 *
 * DMA in ring mode: the write address wraps within the ring.
 * We read the current write address from the DMA hardware and compute
 * the offset from the ring buffer base.
 */
static inline uint32_t GetDmaWritePos() {
    // DMA write address register gives the current destination pointer
    uintptr_t writeAddr = (uintptr_t)dma_channel_hw_addr(dma_channel)->write_addr;
    uintptr_t baseAddr  = (uintptr_t)ringBuffer;
    return static_cast<uint32_t>((writeAddr - baseAddr) & (RING_SIZE - 1));
}

/**
 * @brief Get available bytes in the ring buffer (write ahead of read).
 */
static inline uint32_t Available() {
    uint32_t wp = GetDmaWritePos();
    return (wp >= readPos) ? (wp - readPos) : (RING_SIZE - readPos + wp);
}

// ─── Ring Buffer Read Helpers ───────────────────────────────────────────────

/**
 * @brief Read a byte from the ring buffer at a given offset.
 */
static inline uint8_t RingReadByte(uint32_t pos) {
    return ringBuffer[pos & (RING_SIZE - 1)];
}

/**
 * @brief Copy `len` bytes from ring buffer starting at `pos` into `dest`.
 *        Handles wrap-around transparently.
 */
static void RingCopy(uint8_t* dest, uint32_t pos, uint32_t len) {
    pos &= (RING_SIZE - 1);
    uint32_t firstChunk = RING_SIZE - pos;
    if (firstChunk >= len) {
        memcpy(dest, ringBuffer + pos, len);
    } else {
        memcpy(dest, ringBuffer + pos, firstChunk);
        memcpy(dest + firstChunk, ringBuffer, len - firstChunk);
    }
}

/**
 * @brief Scan ring buffer for the sync word 0x55AA (little-endian: 0xAA, 0x55).
 *
 * @param startPos  Ring buffer position to start scanning.
 * @param maxScan   Maximum number of bytes to scan.
 * @return Ring buffer position of the sync word, or UINT32_MAX if not found.
 */
static uint32_t RingFindSync(uint32_t startPos, uint32_t maxScan) {
    if (maxScan < 2) return UINT32_MAX;

    for (uint32_t i = 0; i <= maxScan - 2; i++) {
        uint32_t pos = (startPos + i) & (RING_SIZE - 1);
        if (ringBuffer[pos] == 0xAA) {
            uint32_t nextPos = (pos + 1) & (RING_SIZE - 1);
            if (ringBuffer[nextPos] == 0x55) {
                return (startPos + i) & (RING_SIZE - 1);
            }
        }
    }
    return UINT32_MAX;
}

// ─── Initialization ─────────────────────────────────────────────────────────

bool OctalSpiRx::Initialize() {
    // --- GPIO setup ---
    // D0-D7 as inputs with pull-down
    for (uint8_t i = 0; i < GpuConfig::SPI_DATA_PIN_COUNT; i++) {
        uint8_t pin = GpuConfig::SPI_DATA_BASE_PIN + i;
        gpio_init(pin);
        gpio_set_dir(pin, GPIO_IN);
        gpio_pull_down(pin);
    }

    // CLK input with pull-down
    gpio_init(GpuConfig::SPI_CLK_PIN);
    gpio_set_dir(GpuConfig::SPI_CLK_PIN, GPIO_IN);
    gpio_pull_down(GpuConfig::SPI_CLK_PIN);

    // CS input with pull-up (active low)
    gpio_init(GpuConfig::SPI_CS_PIN);
    gpio_set_dir(GpuConfig::SPI_CS_PIN, GPIO_IN);
    gpio_pull_up(GpuConfig::SPI_CS_PIN);

    // --- PIO setup ---
    // Load PIO program into PIO1 (PIO0 used by HUB75)
    prog_offset = pio_add_program(pio_instance, &octal_spi_rx_program);
    sm = pio_claim_unused_sm(pio_instance, true);

    // Initialize using the C helper from the .pio file
    octal_spi_rx_program_init(
        pio_instance, sm, prog_offset,
        GpuConfig::SPI_DATA_BASE_PIN,   // D0 = GPIO0
        GpuConfig::SPI_CLK_PIN,          // CLK = GPIO8
        GpuConfig::SPI_CS_PIN            // CS  = GPIO9
    );

    printf("[OctalSPI] PIO1/SM%u loaded at offset %u\n", sm, prog_offset);

    // --- DMA setup ---
    // DMA channel: drain PIO RX FIFO → ring buffer with hardware ring wrapping
    dma_channel = dma_claim_unused_channel(true);

    dma_channel_config c = dma_channel_get_default_config(dma_channel);

    // 8-bit transfers (one byte per PIO autopush)
    channel_config_set_transfer_data_size(&c, DMA_SIZE_8);

    // DREQ: pace to PIO1 RX FIFO of our SM
    channel_config_set_dreq(&c, pio_get_dreq(pio_instance, sm, false));

    // Read from PIO RX FIFO (no increment)
    channel_config_set_read_increment(&c, false);

    // Write to ring buffer (increment, with ring wrapping)
    channel_config_set_write_increment(&c, true);

    // Ring mode on write: wrap write address at 2^RING_LOG2 boundary.
    // The DMA write pointer automatically wraps within the ring buffer.
    channel_config_set_ring(&c, true, RING_LOG2);

    // Configure and start: transfer a very large count (effectively infinite)
    // The ring wrap makes this safe — DMA just cycles through the buffer.
    // We use 0xFFFFFFFF transfers; the ring wrap prevents overrun.
    dma_channel_configure(
        dma_channel,
        &c,
        ringBuffer,                              // write to ring buffer base
        &pio_instance->rxf[sm],                  // read from PIO RX FIFO
        0xFFFFFFFF,                              // "infinite" transfers
        true                                     // start immediately
    );

    // Clear ring buffer and state
    memset(ringBuffer, 0, RING_SIZE);
    readPos = 0;
    totalBytesReceived = 0;
    pendingFramePtr = nullptr;
    pendingFrameLength = 0;

    printf("[OctalSPI] DMA channel %d, ring buffer %u bytes, ready\n",
           dma_channel, RING_SIZE);

    return true;
}

// ─── Frame Extraction ───────────────────────────────────────────────────────

bool OctalSpiRx::TryGetFrame(const uint8_t** frameData, uint32_t* frameLength) {
    uint32_t avail = Available();

    // Need at least a frame header + footer
    if (avail < sizeof(PglFrameHeader) + 2) {
        return false;
    }

    // Scan for sync word starting at readPos
    uint32_t syncPos = RingFindSync(readPos, avail);
    if (syncPos == UINT32_MAX) {
        // No sync word found — discard all scanned data
        readPos = (readPos + avail) & (RING_SIZE - 1);
        return false;
    }

    // Skip garbage bytes before the sync word
    uint32_t skipBytes = (syncPos >= readPos)
        ? (syncPos - readPos)
        : (RING_SIZE - readPos + syncPos);
    readPos = syncPos;
    avail -= skipBytes;

    // Read frame header to get totalLength
    if (avail < sizeof(PglFrameHeader)) {
        return false;  // header not fully received yet
    }

    PglFrameHeader hdr;
    RingCopy(reinterpret_cast<uint8_t*>(&hdr), readPos, sizeof(hdr));

    // Sanity check
    if (hdr.syncWord != 0x55AA || hdr.totalLength > MAX_FRAME_SIZE) {
        // Invalid header — skip this sync word and try the next one
        readPos = (readPos + 2) & (RING_SIZE - 1);
        return false;
    }

    if (avail < hdr.totalLength) {
        return false;  // frame not fully received yet
    }

    // Full frame available. Check if it wraps around the ring boundary.
    uint32_t endPos = (readPos + hdr.totalLength) & (RING_SIZE - 1);
    bool wraps = (readPos + hdr.totalLength) > RING_SIZE;

    if (wraps) {
        // Frame straddles the boundary — copy to staging buffer
        RingCopy(stagingBuffer, readPos, hdr.totalLength);
        *frameData = stagingBuffer;
    } else {
        // Contiguous — point directly into ring buffer
        *frameData = ringBuffer + readPos;
    }

    *frameLength = hdr.totalLength;
    pendingFramePtr = *frameData;
    pendingFrameLength = hdr.totalLength;
    pendingFrameRingStart = readPos;

    return true;
}

void OctalSpiRx::ConsumeFrame() {
    readPos = (pendingFrameRingStart + pendingFrameLength) & (RING_SIZE - 1);
    totalBytesReceived += pendingFrameLength;
    pendingFrameLength = 0;
    pendingFramePtr = nullptr;
}

// ─── Free Space Query ───────────────────────────────────────────────────────

uint32_t OctalSpiRx::GetFreeBytes() {
    uint32_t used = Available();
    return RING_SIZE - used;
}

uint64_t OctalSpiRx::GetTotalBytesReceived() {
    return totalBytesReceived;
}

// ─── Shutdown ───────────────────────────────────────────────────────────────

void OctalSpiRx::Shutdown() {
    // Stop DMA
    if (dma_channel >= 0) {
        dma_channel_abort(dma_channel);
        dma_channel_unclaim(dma_channel);
        dma_channel = -1;
    }

    // Stop and unclaim PIO SM
    pio_sm_set_enabled(pio_instance, sm, false);
    pio_sm_unclaim(pio_instance, sm);
    pio_remove_program(pio_instance, &octal_spi_rx_program, prog_offset);

    printf("[OctalSPI] Shutdown complete\n");
}
