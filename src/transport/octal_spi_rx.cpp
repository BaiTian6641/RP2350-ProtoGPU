/**
 * @file octal_spi_rx.cpp
 * @brief PIO-driven Octal SPI receiver implementation for RP2350.
 *
 * TODO(M1/Week 4): Full implementation. This skeleton maps out the
 * correct Pico SDK API calls.
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

// ─── State ──────────────────────────────────────────────────────────────────

static PIO  pio_instance = pio1;  // Use PIO1 (PIO0 is used by HUB75)
static uint sm = 0;
static int  dma_channel = -1;

/// Ring buffer in SRAM
static uint8_t  ringBuffer[GpuConfig::SPI_RING_BUFFER_SIZE];
static uint32_t writePos = 0;  // updated by DMA / IRQ
static uint32_t readPos  = 0;  // updated by command parser

static uint64_t totalBytesReceived = 0;

/// Frame extraction state
static uint32_t pendingFrameOffset = 0;
static uint32_t pendingFrameLength = 0;

// ─── Initialization ─────────────────────────────────────────────────────────

bool OctalSpiRx::Initialize() {
    // --- GPIO setup ---
    // Configure D0-D7 as inputs
    for (uint8_t i = 0; i < GpuConfig::SPI_DATA_PIN_COUNT; i++) {
        uint8_t pin = GpuConfig::SPI_DATA_BASE_PIN + i;
        gpio_init(pin);
        gpio_set_dir(pin, GPIO_IN);
        gpio_pull_down(pin);  // default low when no host connected
    }

    // CLK and CS as inputs
    gpio_init(GpuConfig::SPI_CLK_PIN);
    gpio_set_dir(GpuConfig::SPI_CLK_PIN, GPIO_IN);
    gpio_pull_down(GpuConfig::SPI_CLK_PIN);

    gpio_init(GpuConfig::SPI_CS_PIN);
    gpio_set_dir(GpuConfig::SPI_CS_PIN, GPIO_IN);
    gpio_pull_up(GpuConfig::SPI_CS_PIN);  // CS active low — default inactive

    // --- PIO setup ---
    // TODO(M1/Week 4): Load PIO program and configure state machine
    // The PIO program samples 8 GPIO pins (D0-D7) on rising edge of CLK
    // when CS is low. Each sample yields 1 byte into the RX FIFO.
    //
    // uint offset = pio_add_program(pio_instance, &octal_spi_rx_program);
    // sm = pio_claim_unused_sm(pio_instance, true);
    // octal_spi_rx_program_init(pio_instance, sm, offset,
    //                           GpuConfig::SPI_DATA_BASE_PIN,
    //                           GpuConfig::SPI_CLK_PIN,
    //                           GpuConfig::SPI_CS_PIN);

    // --- DMA setup ---
    // TODO(M1/Week 4): DMA channel draining PIO RX FIFO → ring buffer
    // dma_channel = dma_claim_unused_channel(true);
    // dma_channel_config c = dma_channel_get_default_config(dma_channel);
    // channel_config_set_transfer_data_size(&c, DMA_SIZE_8);
    // channel_config_set_dreq(&c, pio_get_dreq(pio_instance, sm, false));
    // channel_config_set_read_increment(&c, false);
    // channel_config_set_write_increment(&c, true);
    // channel_config_set_ring(&c, true, /* log2(SPI_RING_BUFFER_SIZE) */);
    //
    // dma_channel_configure(dma_channel, &c,
    //     ringBuffer,                              // write addr
    //     &pio_instance->rxf[sm],                  // read addr (PIO RX FIFO)
    //     GpuConfig::SPI_RING_BUFFER_SIZE,         // count
    //     true);                                   // start

    // Initialize ring buffer state
    writePos = 0;
    readPos  = 0;
    totalBytesReceived = 0;

    return true;
}

// ─── Frame Extraction ───────────────────────────────────────────────────────

bool OctalSpiRx::TryGetFrame(const uint8_t** frameData, uint32_t* frameLength) {
    // TODO(M1/Week 4): Read actual DMA write pointer from hardware
    // writePos = GpuConfig::SPI_RING_BUFFER_SIZE - dma_channel_hw_addr(dma_channel)->transfer_count;

    // Available bytes in ring buffer
    uint32_t available = (writePos >= readPos)
        ? (writePos - readPos)
        : (GpuConfig::SPI_RING_BUFFER_SIZE - readPos + writePos);

    if (available < sizeof(PglFrameHeader) + sizeof(PglFrameFooter)) {
        return false;  // not enough data for a minimal frame
    }

    // Scan for sync word starting at readPos
    // Note: this simplified version assumes no wrap-around mid-frame.
    // TODO(M1/Week 4): Handle ring buffer wrap-around properly.
    const uint8_t* searchStart = ringBuffer + readPos;
    int32_t syncOffset = PglFindSyncWord(searchStart, available);

    if (syncOffset < 0) {
        // No sync word found — discard scanned data
        readPos = (readPos + available) % GpuConfig::SPI_RING_BUFFER_SIZE;
        return false;
    }

    // Skip any garbage bytes before the sync word
    if (syncOffset > 0) {
        readPos = (readPos + static_cast<uint32_t>(syncOffset))
                % GpuConfig::SPI_RING_BUFFER_SIZE;
    }

    // Read frame header to get totalLength
    const uint8_t* headerPtr = ringBuffer + readPos;
    PglFrameHeader hdr;
    PglReadStruct(headerPtr, hdr);

    if (hdr.totalLength > GpuConfig::SPI_RING_BUFFER_SIZE / 2) {
        // Implausible frame size — skip this sync word and try next
        readPos = (readPos + 2) % GpuConfig::SPI_RING_BUFFER_SIZE;
        return false;
    }

    if (available < hdr.totalLength) {
        return false;  // frame not fully received yet
    }

    // Full frame available — validate CRC
    *frameData = ringBuffer + readPos;
    *frameLength = hdr.totalLength;

    pendingFrameOffset = readPos;
    pendingFrameLength = hdr.totalLength;

    return true;
}

void OctalSpiRx::ConsumeFrame() {
    readPos = (pendingFrameOffset + pendingFrameLength)
            % GpuConfig::SPI_RING_BUFFER_SIZE;
    totalBytesReceived += pendingFrameLength;
    pendingFrameLength = 0;
}

// ─── Free Space Query ───────────────────────────────────────────────────────

uint32_t OctalSpiRx::GetFreeBytes() {
    uint32_t used = (writePos >= readPos)
        ? (writePos - readPos)
        : (GpuConfig::SPI_RING_BUFFER_SIZE - readPos + writePos);
    return GpuConfig::SPI_RING_BUFFER_SIZE - used;
}

uint64_t OctalSpiRx::GetTotalBytesReceived() {
    return totalBytesReceived;
}

// ─── Shutdown ───────────────────────────────────────────────────────────────

void OctalSpiRx::Shutdown() {
    // TODO(M1): Stop DMA, unclaim PIO SM
}
