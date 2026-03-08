/**
 * @file mem_qspi_vram.cpp
 * @brief Unified dual-channel QSPI VRAM driver — PIO2 implementation.
 *
 * Implements the QspiVramDriver class declared in mem_qspi_vram.h.
 * Replaces the former mem_opi_psram.cpp (PIO2 OPI/QSPI) and
 * mem_qspi_psram.cpp (QMI CS1 XIP).
 *
 * Uses RP2350 PIO2 with per-channel state machine pairs:
 *   Channel A: SM0 (cmd/write) + SM1 (read)
 *   Channel B: SM2 (cmd/write) + SM3 (read)
 *
 * All access is QSPI (4-bit), no OPI (8-bit) mode — simplified from the
 * original mem_opi_psram.cpp which supported both.
 */

#include "mem_qspi_vram.h"
#include "../gpu_config.h"

#include "hardware/pio.h"
#include "hardware/dma.h"
#include "hardware/gpio.h"
#include "hardware/clocks.h"
#include "hardware/irq.h"
#include "pico/time.h"

#include <cstdio>
#include <cstring>

// ─── Singleton for IRQ trampoline ───────────────────────────────────────────

QspiVramDriver* QspiVramDriver::s_irqInstance_ = nullptr;

// ─── PIO Instance Mapping ───────────────────────────────────────────────────

static PIO GetPioInstance(uint8_t instance) {
    switch (instance) {
        case 0:  return pio0;
        case 1:  return pio1;
#ifdef pio2
        case 2:  return pio2;
#endif
        default: return pio0;
    }
}

// ─── PIO Programs (QSPI only — no OPI mode) ────────────────────────────────
//
// Minimal QSPI PIO programs: 4-bit data bus, side-set CLK.
// Each channel loads its own copy into PIO2 instruction memory.

// QSPI write: pull 32-bit words, shift out 4 bits per clock, MSB first.
static const uint16_t qspi_write_program[] = {
    // .wrap_target
    0x80a0,  // pull   block          side 0
    0x6004,  // out    pins, 4        side 0   ; Shift out 4 bits (DQ0-3)
    0x1001,  // jmp    !osre, 1       side 1   ; Clock rising edge
    // .wrap
};
static const uint qspi_write_program_len = 3;

// QSPI read: sample 4 bits per clock cycle.
static const uint16_t qspi_read_program[] = {
    // .wrap_target
    0x4004,  // in     pins, 4        side 0   ; Sample 4 data bits
    0x1000,  // jmp    0              side 1   ; Clock rising edge
    // .wrap
};
static const uint qspi_read_program_len = 2;

// ─── Destructor ─────────────────────────────────────────────────────────────

QspiVramDriver::~QspiVramDriver() {
    Shutdown();
}

// ─── Channel Initialization ─────────────────────────────────────────────────

bool QspiVramDriver::InitChannel(const QspiVramChannelConfig& config) {
    uint8_t chIdx = static_cast<uint8_t>(config.channel);
    if (chIdx >= QSPI_VRAM_MAX_CHANNELS) return false;

    ChannelState& ch = channels_[chIdx];
    if (ch.initialized) return false;
    if (config.chipCount == 0) return false;

    ch.config = config;

    // ── GPIO Setup ──────────────────────────────────────────────────────

    // CLK pin
    gpio_init(config.clkPin);
    gpio_set_dir(config.clkPin, GPIO_OUT);
    gpio_put(config.clkPin, 0);

    // CS0 pin (active low, start deasserted)
    gpio_init(config.cs0Pin);
    gpio_set_dir(config.cs0Pin, GPIO_OUT);
    gpio_put(config.cs0Pin, 1);

    // CS1 pin (if second chip populated)
    if (config.chipCount > 1 && config.cs1Pin != 0xFF) {
        gpio_init(config.cs1Pin);
        gpio_set_dir(config.cs1Pin, GPIO_OUT);
        gpio_put(config.cs1Pin, 1);
    }

    // Data pins: initially output for command/write
    for (uint8_t i = 0; i < config.dataPinCount; ++i) {
        uint8_t pin = config.dataBasePin + i;
        gpio_init(pin);
        gpio_set_dir(pin, GPIO_OUT);
        gpio_put(pin, 0);
    }

    // ── PIO Setup ───────────────────────────────────────────────────────

    if (!LoadPioProgram(ch)) {
        printf("[QspiVram] ERROR: Failed to load PIO program for channel %c\n",
               'A' + chIdx);
        return false;
    }

    // ── DMA Setup ───────────────────────────────────────────────────────

    ch.dmaChannelTx = dma_claim_unused_channel(true);
    ch.dmaChannelRx = dma_claim_unused_channel(true);

    if (ch.dmaChannelTx < 0 || ch.dmaChannelRx < 0) {
        printf("[QspiVram] ERROR: Failed to claim DMA channels for channel %c\n",
               'A' + chIdx);
        return false;
    }

    PIO pio = GetPioInstance(config.pioInstance);

    // TX DMA: memory → PIO TX FIFO
    dma_channel_config txCfg = dma_channel_get_default_config(ch.dmaChannelTx);
    channel_config_set_transfer_data_size(&txCfg, DMA_SIZE_32);
    channel_config_set_read_increment(&txCfg, true);
    channel_config_set_write_increment(&txCfg, false);
    channel_config_set_dreq(&txCfg, pio_get_dreq(pio, ch.smCommand, true));
    dma_channel_set_config(ch.dmaChannelTx, &txCfg, false);
    dma_channel_set_write_addr(ch.dmaChannelTx, &pio->txf[ch.smCommand], false);

    // RX DMA: PIO RX FIFO → memory
    dma_channel_config rxCfg = dma_channel_get_default_config(ch.dmaChannelRx);
    channel_config_set_transfer_data_size(&rxCfg, DMA_SIZE_32);
    channel_config_set_read_increment(&rxCfg, false);
    channel_config_set_write_increment(&rxCfg, true);
    channel_config_set_dreq(&rxCfg, pio_get_dreq(pio, ch.smRead, false));
    dma_channel_set_config(ch.dmaChannelRx, &rxCfg, false);
    dma_channel_set_read_addr(ch.dmaChannelRx, &pio->rxf[ch.smRead], false);

    // ── Free-list allocator init ────────────────────────────────────────
    InitFreeList(ch);

    ch.initialized = true;
    s_irqInstance_ = this;

    // ── Verify chip identity via RDID ───────────────────────────────────
    for (uint8_t cs = 0; cs < config.chipCount; ++cs) {
        ch.rdidVerified[cs] = VerifyChipId(ch, cs);
        if (!ch.rdidVerified[cs]) {
            printf("[QspiVram] WARNING: Channel %c CS%u RDID failed\n",
                   'A' + chIdx, cs);
        }
    }

    // ── Setup DMA IRQ for async callbacks ───────────────────────────────
    SetupDmaIrq(ch);

    // Compute aggregate capacity
    ch.config.totalCapacity = 0;
    for (uint8_t cs = 0; cs < config.chipCount; ++cs) {
        if (ch.config.chips[cs].detected) {
            ch.config.totalCapacity += ch.config.chips[cs].capacityBytes;
        }
    }

    printf("[QspiVram] Channel %c initialized: %u chip(s), %lu KB total, "
           "pins=DQ%u-%u CLK=%u CS0=%u",
           'A' + chIdx,
           config.chipCount,
           (unsigned long)(ch.config.totalCapacity / 1024),
           config.dataBasePin,
           config.dataBasePin + config.dataPinCount - 1,
           config.clkPin, config.cs0Pin);
    if (config.chipCount > 1) {
        printf(" CS1=%u", config.cs1Pin);
    }
    printf("\n");

    return true;
}

// ─── PIO Program Loading ────────────────────────────────────────────────────

bool QspiVramDriver::LoadPioProgram(ChannelState& ch) {
    PIO pio = GetPioInstance(ch.config.pioInstance);

    // Each channel uses 2 SMs.  Channel A = SM0+SM1, Channel B = SM2+SM3.
    uint8_t chIdx = static_cast<uint8_t>(ch.config.channel);
    int sm0 = chIdx * 2;      // cmd/write SM
    int sm1 = chIdx * 2 + 1;  // read SM

    // Verify SMs are available
    if (sm0 >= 4 || sm1 >= 4) {
        printf("[QspiVram] ERROR: SM indices %d/%d out of range\n", sm0, sm1);
        return false;
    }

    // Claim specific state machines
    pio_sm_claim(pio, sm0);
    pio_sm_claim(pio, sm1);

    ch.smCommand = sm0;
    ch.smRead    = sm1;

    // Load QSPI write program
    pio_program_t writeProg = {};
    writeProg.instructions = qspi_write_program;
    writeProg.length       = qspi_write_program_len;
    writeProg.origin       = -1;

    if (!pio_can_add_program(pio, &writeProg)) {
        printf("[QspiVram] ERROR: Cannot add QSPI write program to PIO%u\n",
               ch.config.pioInstance);
        return false;
    }
    ch.pioProgOffset = pio_add_program(pio, &writeProg);

    // Configure write SM
    pio_sm_config smWriteCfg = pio_get_default_sm_config();
    sm_config_set_wrap(&smWriteCfg,
                       ch.pioProgOffset,
                       ch.pioProgOffset + qspi_write_program_len - 1);
    sm_config_set_out_pins(&smWriteCfg, ch.config.dataBasePin, 4);
    sm_config_set_sideset_pins(&smWriteCfg, ch.config.clkPin);
    sm_config_set_sideset(&smWriteCfg, 1, false, false);
    sm_config_set_out_shift(&smWriteCfg, false, true, 32);  // MSB-first, autopull

    // Compute clock divider for the channel's clock speed
    // Use the fastest chip's clock (if mixed, use the slower of the two)
    uint32_t targetMHz = 133;  // default
    for (uint8_t cs = 0; cs < ch.config.chipCount; ++cs) {
        if (ch.config.chips[cs].detected && ch.config.chips[cs].clockMHz > 0) {
            if (ch.config.chips[cs].clockMHz < targetMHz) {
                targetMHz = ch.config.chips[cs].clockMHz;
            }
        }
    }
    float sysClkHz = (float)clock_get_hz(clk_sys);
    float div = sysClkHz / (targetMHz * 1000000.0f * 2.0f);  // ×2 for DDR-like CLK
    if (div < 1.0f) div = 1.0f;
    sm_config_set_clkdiv(&smWriteCfg, div);

    pio_sm_init(pio, sm0, ch.pioProgOffset, &smWriteCfg);

    // Configure read SM
    pio_program_t readProg = {};
    readProg.instructions = qspi_read_program;
    readProg.length       = qspi_read_program_len;
    readProg.origin       = -1;

    uint readOffset = pio_add_program(pio, &readProg);

    pio_sm_config smReadCfg = pio_get_default_sm_config();
    sm_config_set_wrap(&smReadCfg, readOffset, readOffset + qspi_read_program_len - 1);
    sm_config_set_in_pins(&smReadCfg, ch.config.dataBasePin);
    sm_config_set_sideset_pins(&smReadCfg, ch.config.clkPin);
    sm_config_set_sideset(&smReadCfg, 1, false, false);
    sm_config_set_in_shift(&smReadCfg, false, true, 32);  // MSB-first, autopush
    sm_config_set_clkdiv(&smReadCfg, div);

    pio_sm_init(pio, sm1, readOffset, &smReadCfg);

    // Enable both SMs
    pio_sm_set_enabled(pio, sm0, true);
    pio_sm_set_enabled(pio, sm1, true);

    printf("[QspiVram] PIO%u SM%d/%d loaded (QSPI 4-bit, clkdiv=%.2f, target=%lu MHz)\n",
           ch.config.pioInstance, sm0, sm1, div, (unsigned long)targetMHz);

    return true;
}

// ─── Shutdown ───────────────────────────────────────────────────────────────

void QspiVramDriver::ShutdownChannel(QspiChannel chId) {
    uint8_t chIdx = static_cast<uint8_t>(chId);
    if (chIdx >= QSPI_VRAM_MAX_CHANNELS) return;

    ChannelState& ch = channels_[chIdx];
    if (!ch.initialized) return;

    PIO pio = GetPioInstance(ch.config.pioInstance);

    // Disable SMs
    pio_sm_set_enabled(pio, ch.smCommand, false);
    pio_sm_set_enabled(pio, ch.smRead, false);

    // Unclaim SMs
    pio_sm_unclaim(pio, ch.smCommand);
    pio_sm_unclaim(pio, ch.smRead);

    // Release DMA channels
    if (ch.dmaChannelTx >= 0) {
        dma_channel_abort(ch.dmaChannelTx);
        dma_channel_unclaim(ch.dmaChannelTx);
        ch.dmaChannelTx = -1;
    }
    if (ch.dmaChannelRx >= 0) {
        dma_channel_abort(ch.dmaChannelRx);
        dma_channel_unclaim(ch.dmaChannelRx);
        ch.dmaChannelRx = -1;
    }

    ch.initialized = false;
    printf("[QspiVram] Channel %c shut down\n", 'A' + chIdx);
}

void QspiVramDriver::Shutdown() {
    ShutdownChannel(QspiChannel::A);
    ShutdownChannel(QspiChannel::B);
    s_irqInstance_ = nullptr;
}

// ─── Chip Selection ─────────────────────────────────────────────────────────

uint8_t QspiVramDriver::ChipIndexForAddress(const ChannelState& ch, uint32_t addr) const {
    if (ch.config.chipCount <= 1) return 0;
    // Address space: [0 .. chip0.capacity) = CS0, [chip0.capacity ..) = CS1
    return (addr >= ch.config.chips[0].capacityBytes) ? 1 : 0;
}

uint32_t QspiVramDriver::ChipLocalAddress(const ChannelState& ch, uint32_t addr) const {
    if (ch.config.chipCount <= 1) return addr;
    uint8_t csIdx = ChipIndexForAddress(ch, addr);
    return (csIdx == 0) ? addr : (addr - ch.config.chips[0].capacityBytes);
}

// ─── Data Direction ─────────────────────────────────────────────────────────

void QspiVramDriver::SetDataDirection(ChannelState& ch, bool output) {
    for (uint8_t i = 0; i < ch.config.dataPinCount; ++i) {
        gpio_set_dir(ch.config.dataBasePin + i, output);
    }
}

// ─── QSPI Command Issue ────────────────────────────────────────────────────

void QspiVramDriver::IssueQspiCommand(ChannelState& ch, uint8_t opcode,
                                        uint32_t address, uint8_t chipIndex) {
    // Assert CS for the selected chip
    uint8_t csPin = (chipIndex == 0) ? ch.config.cs0Pin : ch.config.cs1Pin;
    gpio_put(csPin, 0);  // CS active (low)

    PIO pio = GetPioInstance(ch.config.pioInstance);

    // Build command word: opcode in upper nibbles, then 3 address bytes
    // QSPI: 2 cycles for opcode (8 bits / 4), 6 cycles for address (24 bits / 4)
    uint32_t cmdWord = (static_cast<uint32_t>(opcode) << 24) | (address & 0x00FFFFFF);

    // Set data pins to output for command phase
    SetDataDirection(ch, true);

    // Push command word to PIO TX FIFO
    pio_sm_put_blocking(pio, ch.smCommand, cmdWord);

    // Wait for command to shift out
    busy_wait_us(1);
}

void QspiVramDriver::IssueMramWren(ChannelState& ch, uint8_t chipIndex) {
    uint8_t csPin = (chipIndex == 0) ? ch.config.cs0Pin : ch.config.cs1Pin;

    // WREN is a simple 0x06 command with no address
    gpio_put(csPin, 0);
    PIO pio = GetPioInstance(ch.config.pioInstance);
    pio_sm_put_blocking(pio, ch.smCommand, 0x06000000);
    busy_wait_us(1);
    gpio_put(csPin, 1);  // Deassert CS after WREN
    busy_wait_us(1);      // tCSH: CS high time
}

// ─── Synchronous Read ───────────────────────────────────────────────────────

bool QspiVramDriver::ReadSync(QspiChannel chId, uint32_t srcAddr,
                               void* dest, uint32_t length) {
    uint8_t chIdx = static_cast<uint8_t>(chId);
    if (chIdx >= QSPI_VRAM_MAX_CHANNELS) return false;

    ChannelState& ch = channels_[chIdx];
    if (!ch.initialized || length == 0) return false;

    // Determine which chip(s) this read spans
    uint8_t csIdx = ChipIndexForAddress(ch, srcAddr);
    uint32_t localAddr = ChipLocalAddress(ch, srcAddr);
    uint32_t chipCap = ch.config.chips[csIdx].capacityBytes;

    // Check if read crosses chip boundary
    uint32_t firstLen = length;
    uint32_t secondLen = 0;
    if (ch.config.chipCount > 1 && csIdx == 0 &&
        (localAddr + length) > chipCap) {
        firstLen  = chipCap - localAddr;
        secondLen = length - firstLen;
    }

    // Read from first chip
    {
        uint8_t csPin = (csIdx == 0) ? ch.config.cs0Pin : ch.config.cs1Pin;
        const auto& chip = ch.config.chips[csIdx];

        // Issue Quad Read command (0xEB for MRAM, 0xEB for PSRAM)
        IssueQspiCommand(ch, 0xEB, localAddr, csIdx);

        // Dummy cycles
        SetDataDirection(ch, false);  // Switch to input for dummy + data
        busy_wait_us(chip.readLatency);

        // DMA read
        PIO pio = GetPioInstance(ch.config.pioInstance);
        uint32_t words = (firstLen + 3) / 4;
        dma_channel_set_write_addr(ch.dmaChannelRx, dest, false);
        dma_channel_set_trans_count(ch.dmaChannelRx, words, true);

        // Wait for DMA
        dma_channel_wait_for_finish_blocking(ch.dmaChannelRx);

        // Deassert CS
        gpio_put(csPin, 1);
    }

    // Read from second chip (if cross-boundary)
    if (secondLen > 0) {
        uint8_t* dest2 = static_cast<uint8_t*>(dest) + firstLen;
        uint8_t csPin1 = ch.config.cs1Pin;

        IssueQspiCommand(ch, 0xEB, 0, 1);

        SetDataDirection(ch, false);
        busy_wait_us(ch.config.chips[1].readLatency);

        PIO pio = GetPioInstance(ch.config.pioInstance);
        uint32_t words = (secondLen + 3) / 4;
        dma_channel_set_write_addr(ch.dmaChannelRx, dest2, false);
        dma_channel_set_trans_count(ch.dmaChannelRx, words, true);

        dma_channel_wait_for_finish_blocking(ch.dmaChannelRx);
        gpio_put(csPin1, 1);
    }

    ch.totalReads++;
    return true;
}

// ─── Synchronous Write ──────────────────────────────────────────────────────

bool QspiVramDriver::WriteSync(QspiChannel chId, uint32_t destAddr,
                                const void* src, uint32_t length) {
    uint8_t chIdx = static_cast<uint8_t>(chId);
    if (chIdx >= QSPI_VRAM_MAX_CHANNELS) return false;

    ChannelState& ch = channels_[chIdx];
    if (!ch.initialized || length == 0) return false;

    uint8_t csIdx = ChipIndexForAddress(ch, destAddr);
    uint32_t localAddr = ChipLocalAddress(ch, destAddr);
    const auto& chip = ch.config.chips[csIdx];

    // MRAM: issue WREN before write
    if (chip.needsWrenBeforeWrite) {
        IssueMramWren(ch, csIdx);
    }

    // Issue Quad Write command (0x02 for standard, 0x38 for quad)
    IssueQspiCommand(ch, 0x38, localAddr, csIdx);

    // DMA write
    PIO pio = GetPioInstance(ch.config.pioInstance);
    uint32_t words = (length + 3) / 4;
    dma_channel_set_read_addr(ch.dmaChannelTx, src, false);
    dma_channel_set_trans_count(ch.dmaChannelTx, words, true);

    dma_channel_wait_for_finish_blocking(ch.dmaChannelTx);

    // Deassert CS
    uint8_t csPin = (csIdx == 0) ? ch.config.cs0Pin : ch.config.cs1Pin;
    gpio_put(csPin, 1);

    ch.totalWrites++;
    return true;
}

// ─── Asynchronous Read ──────────────────────────────────────────────────────

bool QspiVramDriver::ReadAsync(QspiChannel chId, uint32_t srcAddr,
                                void* dest, uint32_t length) {
    uint8_t chIdx = static_cast<uint8_t>(chId);
    if (chIdx >= QSPI_VRAM_MAX_CHANNELS) return false;

    ChannelState& ch = channels_[chIdx];
    if (!ch.initialized || length == 0) return false;

    uint8_t csIdx = ChipIndexForAddress(ch, srcAddr);
    uint32_t localAddr = ChipLocalAddress(ch, srcAddr);
    uint32_t chipCap = ch.config.chips[csIdx].capacityBytes;

    // Check for cross-chip boundary
    uint32_t firstLen = length;
    if (ch.config.chipCount > 1 && csIdx == 0 &&
        (localAddr + length) > chipCap) {
        firstLen = chipCap - localAddr;
        // Set up deferred second read
        ch.pendingChip1.srcAddr = 0;
        ch.pendingChip1.dest    = static_cast<uint8_t*>(dest) + firstLen;
        ch.pendingChip1.length  = length - firstLen;
        ch.pendingChip1.csPin   = ch.config.cs1Pin;
        ch.pendingChip1.pending = true;
    }

    // Issue command for first chip
    IssueQspiCommand(ch, 0xEB, localAddr, csIdx);
    SetDataDirection(ch, false);
    busy_wait_us(ch.config.chips[csIdx].readLatency);

    // Start DMA (non-blocking)
    uint32_t words = (firstLen + 3) / 4;
    dma_channel_set_write_addr(ch.dmaChannelRx, dest, false);
    dma_channel_set_trans_count(ch.dmaChannelRx, words, true);

    ch.totalReads++;
    return true;
}

// ─── Asynchronous Write ─────────────────────────────────────────────────────

bool QspiVramDriver::WriteAsync(QspiChannel chId, uint32_t destAddr,
                                 const void* src, uint32_t length) {
    uint8_t chIdx = static_cast<uint8_t>(chId);
    if (chIdx >= QSPI_VRAM_MAX_CHANNELS) return false;

    ChannelState& ch = channels_[chIdx];
    if (!ch.initialized || length == 0) return false;

    uint8_t csIdx = ChipIndexForAddress(ch, destAddr);
    uint32_t localAddr = ChipLocalAddress(ch, destAddr);
    const auto& chip = ch.config.chips[csIdx];

    if (chip.needsWrenBeforeWrite) {
        IssueMramWren(ch, csIdx);
    }

    IssueQspiCommand(ch, 0x38, localAddr, csIdx);

    uint32_t words = (length + 3) / 4;
    dma_channel_set_read_addr(ch.dmaChannelTx, src, false);
    dma_channel_set_trans_count(ch.dmaChannelTx, words, true);

    ch.totalWrites++;
    return true;
}

// ─── DMA Status & Wait ─────────────────────────────────────────────────────

QspiVramDmaStatus QspiVramDriver::GetDmaStatus(QspiChannel chId) const {
    uint8_t chIdx = static_cast<uint8_t>(chId);
    if (chIdx >= QSPI_VRAM_MAX_CHANNELS) return QspiVramDmaStatus::ERROR;

    const ChannelState& ch = channels_[chIdx];
    if (!ch.initialized) return QspiVramDmaStatus::ERROR;

    bool txBusy = dma_channel_is_busy(ch.dmaChannelTx);
    bool rxBusy = dma_channel_is_busy(ch.dmaChannelRx);

    if (txBusy) return QspiVramDmaStatus::WRITING;
    if (rxBusy) return QspiVramDmaStatus::READING;
    return QspiVramDmaStatus::IDLE;
}

void QspiVramDriver::WaitDma(QspiChannel chId) {
    uint8_t chIdx = static_cast<uint8_t>(chId);
    if (chIdx >= QSPI_VRAM_MAX_CHANNELS) return;

    ChannelState& ch = channels_[chIdx];
    if (!ch.initialized) return;

    dma_channel_wait_for_finish_blocking(ch.dmaChannelTx);
    dma_channel_wait_for_finish_blocking(ch.dmaChannelRx);

    // Handle pending second-chip read (cross-boundary async)
    if (ch.pendingChip1.pending) {
        StartPendingChip1Read(ch);
        dma_channel_wait_for_finish_blocking(ch.dmaChannelRx);
    }
}

// ─── DMA Callback Setup ────────────────────────────────────────────────────

void QspiVramDriver::SetDmaCallback(QspiChannel chId, QspiVramDmaCallback cb,
                                     void* userCtx) {
    uint8_t chIdx = static_cast<uint8_t>(chId);
    if (chIdx >= QSPI_VRAM_MAX_CHANNELS) return;

    ChannelState& ch = channels_[chIdx];
    ch.dmaCallback    = cb;
    ch.dmaCallbackCtx = userCtx;
}

void QspiVramDriver::ClearDmaCallback(QspiChannel chId) {
    uint8_t chIdx = static_cast<uint8_t>(chId);
    if (chIdx >= QSPI_VRAM_MAX_CHANNELS) return;

    channels_[chIdx].dmaCallback    = nullptr;
    channels_[chIdx].dmaCallbackCtx = nullptr;
}

void QspiVramDriver::SetupDmaIrq(ChannelState& ch) {
    // Use DMA_IRQ_0 for channel A, DMA_IRQ_1 for channel B
    uint8_t chIdx = static_cast<uint8_t>(ch.config.channel);
    ch.dmaIrqIndex = chIdx;

    uint irqNum = (chIdx == 0) ? DMA_IRQ_0 : DMA_IRQ_1;

    dma_channel_set_irq0_enabled(ch.dmaChannelRx, chIdx == 0);
    dma_channel_set_irq1_enabled(ch.dmaChannelRx, chIdx == 1);

    if (chIdx == 0) {
        irq_set_exclusive_handler(irqNum, DmaIrqHandlerA);
    } else {
        irq_set_exclusive_handler(irqNum, DmaIrqHandlerB);
    }
    irq_set_enabled(irqNum, true);
    ch.irqConfigured = true;
}

void QspiVramDriver::DmaIrqHandlerA() {
    if (!s_irqInstance_) return;
    ChannelState& ch = s_irqInstance_->channels_[0];

    // Clear IRQ
    dma_hw->ints0 = (1u << ch.dmaChannelRx);

    // Handle pending cross-chip read
    if (ch.pendingChip1.pending) {
        s_irqInstance_->StartPendingChip1Read(ch);
        return;
    }

    // Invoke user callback
    if (ch.dmaCallback) {
        ch.dmaCallback(true, 0, ch.dmaCallbackCtx);
    }
}

void QspiVramDriver::DmaIrqHandlerB() {
    if (!s_irqInstance_) return;
    ChannelState& ch = s_irqInstance_->channels_[1];

    dma_hw->ints1 = (1u << ch.dmaChannelRx);

    if (ch.pendingChip1.pending) {
        s_irqInstance_->StartPendingChip1Read(ch);
        return;
    }

    if (ch.dmaCallback) {
        ch.dmaCallback(true, 1, ch.dmaCallbackCtx);
    }
}

void QspiVramDriver::StartPendingChip1Read(ChannelState& ch) {
    auto& p = ch.pendingChip1;
    if (!p.pending) return;

    // Deassert CS0, then issue read on CS1
    gpio_put(ch.config.cs0Pin, 1);

    IssueQspiCommand(ch, 0xEB, p.srcAddr, 1);
    SetDataDirection(ch, false);
    busy_wait_us(ch.config.chips[1].readLatency);

    uint32_t words = (p.length + 3) / 4;
    dma_channel_set_write_addr(ch.dmaChannelRx, p.dest, false);
    dma_channel_set_trans_count(ch.dmaChannelRx, words, true);

    p.pending = false;
}

// ─── Prefetch ───────────────────────────────────────────────────────────────

bool QspiVramDriver::Prefetch(QspiChannel ch, uint32_t srcAddr,
                               uint8_t* sramDest, uint32_t length) {
    return ReadAsync(ch, srcAddr, sramDest, length);
}

// ─── Free-List Allocator ────────────────────────────────────────────────────

void QspiVramDriver::InitFreeList(ChannelState& ch) {
    ch.freeBlockCount = 1;
    ch.freeList[0].addr = 0;
    ch.freeList[0].size = ch.config.totalCapacity;
}

void QspiVramDriver::CoalesceFreeList(ChannelState& ch) {
    if (ch.freeBlockCount <= 1) return;

    // Sort by address (insertion sort — small array)
    for (uint16_t i = 1; i < ch.freeBlockCount; ++i) {
        auto tmp = ch.freeList[i];
        int16_t j = i - 1;
        while (j >= 0 && ch.freeList[j].addr > tmp.addr) {
            ch.freeList[j + 1] = ch.freeList[j];
            j--;
        }
        ch.freeList[j + 1] = tmp;
    }

    // Merge adjacent blocks
    uint16_t out = 0;
    for (uint16_t i = 1; i < ch.freeBlockCount; ++i) {
        if (ch.freeList[out].addr + ch.freeList[out].size == ch.freeList[i].addr) {
            ch.freeList[out].size += ch.freeList[i].size;
        } else {
            out++;
            ch.freeList[out] = ch.freeList[i];
        }
    }
    ch.freeBlockCount = out + 1;
}

uint32_t QspiVramDriver::Alloc(QspiChannel chId, uint32_t size, uint32_t alignment) {
    uint8_t chIdx = static_cast<uint8_t>(chId);
    if (chIdx >= QSPI_VRAM_MAX_CHANNELS) return 0xFFFFFFFF;

    ChannelState& ch = channels_[chIdx];
    if (!ch.initialized || size == 0) return 0xFFFFFFFF;

    // First-fit with alignment
    for (uint16_t i = 0; i < ch.freeBlockCount; ++i) {
        uint32_t alignedAddr = (ch.freeList[i].addr + alignment - 1) & ~(alignment - 1);
        uint32_t waste = alignedAddr - ch.freeList[i].addr;
        uint32_t needed = waste + size;

        if (ch.freeList[i].size >= needed) {
            uint32_t result = alignedAddr;

            if (waste > 0) {
                // Split: keep waste as a smaller free block at the front
                ch.freeList[i].size -= needed;
                if (ch.freeList[i].size == 0) {
                    // Remove this block entirely, replace with waste portion
                    ch.freeList[i].addr = 0;
                    ch.freeList[i].size = 0;
                    // Shift down
                    for (uint16_t j = i; j < ch.freeBlockCount - 1; ++j) {
                        ch.freeList[j] = ch.freeList[j + 1];
                    }
                    ch.freeBlockCount--;
                } else {
                    ch.freeList[i].addr = alignedAddr + size;
                }
            } else {
                // No waste — just shrink or remove the block
                ch.freeList[i].addr += size;
                ch.freeList[i].size -= size;
                if (ch.freeList[i].size == 0) {
                    for (uint16_t j = i; j < ch.freeBlockCount - 1; ++j) {
                        ch.freeList[j] = ch.freeList[j + 1];
                    }
                    ch.freeBlockCount--;
                }
            }

            return result;
        }
    }

    return 0xFFFFFFFF;  // OOM
}

void QspiVramDriver::Free(QspiChannel chId, uint32_t addr) {
    uint8_t chIdx = static_cast<uint8_t>(chId);
    if (chIdx >= QSPI_VRAM_MAX_CHANNELS) return;

    ChannelState& ch = channels_[chIdx];

    // Find the allocation size — we need to track this externally
    // For now, caller is responsible for passing correct addr
    // The block will be added as a free block and coalesced

    if (ch.freeBlockCount >= ChannelState::MAX_FREE_BLOCKS) {
        CoalesceFreeList(ch);
    }

    // Note: size-tracking requires either a separate allocation table
    // or the caller passing the size.  For the free-list allocator,
    // we store size alongside address in a separate shadow array.
    // This is handled by MemTierManager which tracks MemRecord.dataSize.

    // Add free block (size must be provided by caller via MemTierManager)
    // This is a placeholder — in practice, MemTierManager calls FreeAll()
    // or manages sizes through MemRecord.

    printf("[QspiVram] WARNING: Free(addr=0x%lX) — use FreeAll() or "
           "MemTierManager for proper size tracking\n",
           (unsigned long)addr);
}

void QspiVramDriver::FreeAll(QspiChannel chId) {
    uint8_t chIdx = static_cast<uint8_t>(chId);
    if (chIdx >= QSPI_VRAM_MAX_CHANNELS) return;

    ChannelState& ch = channels_[chIdx];
    InitFreeList(ch);
}

uint32_t QspiVramDriver::Available(QspiChannel chId) const {
    uint8_t chIdx = static_cast<uint8_t>(chId);
    if (chIdx >= QSPI_VRAM_MAX_CHANNELS) return 0;

    const ChannelState& ch = channels_[chIdx];
    uint32_t total = 0;
    for (uint16_t i = 0; i < ch.freeBlockCount; ++i) {
        total += ch.freeList[i].size;
    }
    return total;
}

// ─── Diagnostics ────────────────────────────────────────────────────────────

uint32_t QspiVramDriver::GetTotalReads(QspiChannel chId) const {
    uint8_t chIdx = static_cast<uint8_t>(chId);
    return (chIdx < QSPI_VRAM_MAX_CHANNELS) ? channels_[chIdx].totalReads : 0;
}

uint32_t QspiVramDriver::GetTotalWrites(QspiChannel chId) const {
    uint8_t chIdx = static_cast<uint8_t>(chId);
    return (chIdx < QSPI_VRAM_MAX_CHANNELS) ? channels_[chIdx].totalWrites : 0;
}

uint32_t QspiVramDriver::GetCapacity(QspiChannel chId) const {
    uint8_t chIdx = static_cast<uint8_t>(chId);
    return (chIdx < QSPI_VRAM_MAX_CHANNELS)
        ? channels_[chIdx].config.totalCapacity : 0;
}

uint8_t QspiVramDriver::GetChipCount(QspiChannel chId) const {
    uint8_t chIdx = static_cast<uint8_t>(chId);
    return (chIdx < QSPI_VRAM_MAX_CHANNELS)
        ? channels_[chIdx].config.chipCount : 0;
}

bool QspiVramDriver::IsMram(QspiChannel chId) const {
    uint8_t chIdx = static_cast<uint8_t>(chId);
    if (chIdx >= QSPI_VRAM_MAX_CHANNELS) return false;
    const auto& cfg = channels_[chIdx].config;
    for (uint8_t cs = 0; cs < cfg.chipCount; ++cs) {
        if (!cfg.chips[cs].isMram) return false;
    }
    return cfg.chipCount > 0;
}

bool QspiVramDriver::HasRandomAccessPenalty(QspiChannel chId) const {
    uint8_t chIdx = static_cast<uint8_t>(chId);
    if (chIdx >= QSPI_VRAM_MAX_CHANNELS) return true;
    const auto& cfg = channels_[chIdx].config;
    for (uint8_t cs = 0; cs < cfg.chipCount; ++cs) {
        if (cfg.chips[cs].hasRandomAccessPenalty) return true;
    }
    return false;
}

bool QspiVramDriver::IsNonVolatile(QspiChannel chId) const {
    uint8_t chIdx = static_cast<uint8_t>(chId);
    if (chIdx >= QSPI_VRAM_MAX_CHANNELS) return false;
    const auto& cfg = channels_[chIdx].config;
    for (uint8_t cs = 0; cs < cfg.chipCount; ++cs) {
        if (!cfg.chips[cs].isNonVolatile) return false;
    }
    return cfg.chipCount > 0;
}

const QspiVramChipConfig& QspiVramDriver::GetChipConfig(QspiChannel chId,
                                                          uint8_t csIndex) const {
    uint8_t chIdx = static_cast<uint8_t>(chId);
    static const QspiVramChipConfig empty{};
    if (chIdx >= QSPI_VRAM_MAX_CHANNELS) return empty;
    if (csIndex >= QSPI_VRAM_MAX_CS_PER_CHANNEL) return empty;
    return channels_[chIdx].config.chips[csIndex];
}

const QspiVramChannelConfig& QspiVramDriver::GetChannelConfig(QspiChannel chId) const {
    uint8_t chIdx = static_cast<uint8_t>(chId);
    static const QspiVramChannelConfig empty{};
    if (chIdx >= QSPI_VRAM_MAX_CHANNELS) return empty;
    return channels_[chIdx].config;
}

// ─── Chip Identity Verification ─────────────────────────────────────────────

bool QspiVramDriver::VerifyChipId(ChannelState& ch, uint8_t chipIndex) {
    const auto& chip = ch.config.chips[chipIndex];
    if (!chip.detected) return false;

    PIO pio = GetPioInstance(ch.config.pioInstance);
    uint8_t csPin = (chipIndex == 0) ? ch.config.cs0Pin : ch.config.cs1Pin;

    if (chip.isMram) {
        // MRAM RDID: cmd 0x4B + mode byte 0xFF → 5 response bytes
        gpio_put(csPin, 0);
        SetDataDirection(ch, true);
        pio_sm_put_blocking(pio, ch.smCommand, 0x4BFF0000);
        busy_wait_us(2);

        SetDataDirection(ch, false);
        busy_wait_us(1);

        // Read 5 bytes (2 words rounded up)
        uint32_t rdid[2] = {0, 0};
        for (int i = 0; i < 2; ++i) {
            if (!pio_sm_is_rx_fifo_empty(pio, ch.smRead)) {
                rdid[i] = pio_sm_get(pio, ch.smRead);
            }
        }
        gpio_put(csPin, 1);

        uint64_t response = ((uint64_t)rdid[0] << 32) | rdid[1];
        bool match = (response == chip.expectedRdid);
        printf("[QspiVram] Ch%c CS%u MRAM RDID: 0x%010llX %s\n",
               'A' + static_cast<uint8_t>(ch.config.channel),
               chipIndex, (unsigned long long)response,
               match ? "✓" : "✗");
        return match;
    } else {
        // PSRAM RDID: cmd 0x9F → 3 response bytes
        gpio_put(csPin, 0);
        SetDataDirection(ch, true);
        pio_sm_put_blocking(pio, ch.smCommand, 0x9F000000);
        busy_wait_us(2);

        SetDataDirection(ch, false);
        busy_wait_us(1);

        uint32_t rdid = 0;
        if (!pio_sm_is_rx_fifo_empty(pio, ch.smRead)) {
            rdid = pio_sm_get(pio, ch.smRead);
        }
        gpio_put(csPin, 1);

        bool match = (rdid != 0 && rdid != 0xFFFFFFFF);
        printf("[QspiVram] Ch%c CS%u PSRAM RDID: 0x%08lX %s\n",
               'A' + static_cast<uint8_t>(ch.config.channel),
               chipIndex, (unsigned long)rdid,
               match ? "✓" : "✗");
        return match;
    }
}
