/**
 * @file mem_opi_psram.cpp
 * @brief PIO2 external memory driver — OPI PSRAM and QSPI MRAM implementation.
 *
 * Implements the OpiPsramDriver class declared in mem_opi_psram.h.
 * Uses RP2350 PIO2 with two state machines:
 *   SM0: Command/write (TX side)
 *   SM1: Read data (RX side)
 *
 * Supports three modes:
 *   OPI_PSRAM:         APS6408L, 8-bit, DDR-like macro protocol
 *   DUAL_QSPI_MRAM:   2×MR10Q010, 4-bit QSPI, dual chip-selects
 *   SINGLE_QSPI_MRAM: 1×MR10Q010, 4-bit QSPI, single chip-select
 */

#include "mem_opi_psram.h"
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

OpiPsramDriver* OpiPsramDriver::s_irqInstance_ = nullptr;

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

// ─── PIO Programs ───────────────────────────────────────────────────────────
//
// For the RP2350, PIO programs are loaded at runtime.
// We define minimal PIO programs inline as instruction arrays.
// These implement the command/data transfer protocol for each mode.
//
// The OPI program shifts 8 bits per clock, autopush/autopull at 8 or 32 bits.
// The QSPI program shifts 4 bits per clock.

// OPI write program: pull 32-bit words, shift out 8 bits at a time, MSB first.
// Side-set: CLK on bit 0.
// Format: opcode byte, 3 address bytes, then N data bytes.
static const uint16_t opi_write_program[] = {
    // .wrap_target
    0x80a0,  // pull   block          side 0   ; Pull TX word
    0x6008,  // out    pins, 8        side 0   ; Shift out 8 bits (DQ0-7)
    0x1001,  // jmp    !osre, 1       side 1   ; Clock rising edge, loop if more bits
    // .wrap
};
static const uint opi_write_program_len = 3;

// OPI read program: send command/address (8-bit), then read data (8-bit).
// Reads into RX FIFO via autopush.
static const uint16_t opi_read_program[] = {
    // .wrap_target
    0x4008,  // in     pins, 8        side 0   ; Sample 8 data bits
    0x1000,  // jmp    0              side 1   ; Clock rising edge, loop
    // .wrap
};
static const uint opi_read_program_len = 2;

// QSPI write program: pull 32-bit words, shift out 4 bits at a time.
static const uint16_t qspi_write_program[] = {
    // .wrap_target
    0x80a0,  // pull   block          side 0
    0x6004,  // out    pins, 4        side 0   ; Shift out 4 bits (DQ0-3)
    0x1001,  // jmp    !osre, 1       side 1   ; Clock rising edge
    // .wrap
};
static const uint qspi_write_program_len = 3;

// QSPI read program: sample 4 bits per clock cycle.
static const uint16_t qspi_read_program[] = {
    // .wrap_target
    0x4004,  // in     pins, 4        side 0   ; Sample 4 data bits
    0x1000,  // jmp    0              side 1   ; Clock rising edge
    // .wrap
};
static const uint qspi_read_program_len = 2;

// ─── Destructor ─────────────────────────────────────────────────────────────

OpiPsramDriver::~OpiPsramDriver() {
    if (initialized_) {
        Shutdown();
    }
}

// ─── Initialization ─────────────────────────────────────────────────────────

bool OpiPsramDriver::Initialize(const Pio2MemConfig& config) {
    if (initialized_) return false;
    if (config.mode == Pio2MemMode::NONE) return false;

    config_ = config;

    // Adjust data pin count based on mode
    if (config_.mode == Pio2MemMode::OPI_PSRAM) {
        config_.dataPinCount = 8;
        config_.isMram = false;
        config_.hasRandomAccessPenalty = true;
        config_.isNonVolatile = false;
        config_.needsWrenBeforeWrite = false;
    } else {
        config_.dataPinCount = 4;
        config_.isMram = true;
        config_.hasRandomAccessPenalty = false;
        config_.isNonVolatile = true;
        config_.needsWrenBeforeWrite = true;
        config_.chipCount = (config_.mode == Pio2MemMode::DUAL_QSPI_MRAM) ? 2 : 1;
        config_.chipCapacity = GpuConfig::QSPI_MRAM_CHIP_CAPACITY;
        config_.capacityBytes = config_.chipCapacity * config_.chipCount;
    }

    // ── GPIO Setup ──────────────────────────────────────────────────────

    // CLK pin
    gpio_init(config_.clkPin);
    gpio_set_dir(config_.clkPin, GPIO_OUT);
    gpio_put(config_.clkPin, 0);

    // CS0 pin (active low, start deasserted)
    gpio_init(config_.cs0Pin);
    gpio_set_dir(config_.cs0Pin, GPIO_OUT);
    gpio_put(config_.cs0Pin, 1);

    // CS1 pin (dual MRAM only)
    if (config_.mode == Pio2MemMode::DUAL_QSPI_MRAM) {
        gpio_init(config_.cs1Pin);
        gpio_set_dir(config_.cs1Pin, GPIO_OUT);
        gpio_put(config_.cs1Pin, 1);
    }

    // Data pins: initially set as output for command/write
    for (uint8_t i = 0; i < config_.dataPinCount; ++i) {
        uint8_t pin = config_.dataBasePin + i;
        gpio_init(pin);
        gpio_set_dir(pin, GPIO_OUT);
        gpio_put(pin, 0);
    }

    // ── PIO Setup ───────────────────────────────────────────────────────

    if (!LoadPioProgram()) {
        printf("[OpiPsram] ERROR: Failed to load PIO program\n");
        return false;
    }

    // ── DMA Setup ───────────────────────────────────────────────────────

    dmaChannelTx_ = dma_claim_unused_channel(true);
    dmaChannelRx_ = dma_claim_unused_channel(true);

    if (dmaChannelTx_ < 0 || dmaChannelRx_ < 0) {
        printf("[OpiPsram] ERROR: Failed to claim DMA channels\n");
        return false;
    }

    // TX DMA: memory → PIO TX FIFO
    dma_channel_config txCfg = dma_channel_get_default_config(dmaChannelTx_);
    PIO pio = GetPioInstance(config_.pioInstance);
    channel_config_set_transfer_data_size(&txCfg, DMA_SIZE_32);
    channel_config_set_read_increment(&txCfg, true);
    channel_config_set_write_increment(&txCfg, false);
    channel_config_set_dreq(&txCfg, pio_get_dreq(pio, smCommand_, true));
    dma_channel_set_config(dmaChannelTx_, &txCfg, false);
    dma_channel_set_write_addr(dmaChannelTx_, &pio->txf[smCommand_], false);

    // RX DMA: PIO RX FIFO → memory
    dma_channel_config rxCfg = dma_channel_get_default_config(dmaChannelRx_);
    channel_config_set_transfer_data_size(&rxCfg, DMA_SIZE_32);
    channel_config_set_read_increment(&rxCfg, false);
    channel_config_set_write_increment(&rxCfg, true);
    channel_config_set_dreq(&rxCfg, pio_get_dreq(pio, smRead_, false));
    dma_channel_set_config(dmaChannelRx_, &rxCfg, false);
    dma_channel_set_read_addr(dmaChannelRx_, &pio->rxf[smRead_], false);

    // ── Reset allocator ───────────────────────────────────────────────
    InitFreeList();

    initialized_ = true;

    // ── Verify chip identity via RDID ───────────────────────────────────
    rdidVerified_ = VerifyChipId();
    if (!rdidVerified_) {
        printf("[OpiPsram] WARNING: RDID verification failed — proceeding "
               "(DMA read/write will serve as implicit verification)\n");
    }

    // ── Setup DMA IRQ for async callbacks ───────────────────────────────
    SetupDmaIrq();

    printf("[OpiPsram] Initialized: mode=%u, capacity=%lu KB, "
           "pins=DQ%u-%u CLK=%u CS0=%u%s RDID=%s\n",
           static_cast<uint8_t>(config_.mode),
           (unsigned long)(config_.capacityBytes / 1024),
           config_.dataBasePin,
           config_.dataBasePin + config_.dataPinCount - 1,
           config_.clkPin, config_.cs0Pin,
           (config_.chipCount > 1) ? " CS1=" : "",
           rdidVerified_ ? "OK" : "FAIL");

    return true;
}

bool OpiPsramDriver::LoadPioProgram() {
    PIO pio = GetPioInstance(config_.pioInstance);

    // Claim two state machines on PIO2
    int sm0 = pio_claim_unused_sm(pio, false);
    int sm1 = pio_claim_unused_sm(pio, false);
    if (sm0 < 0 || sm1 < 0) {
        printf("[OpiPsram] ERROR: Cannot claim 2 SMs on PIO%u\n", config_.pioInstance);
        return false;
    }
    smCommand_ = static_cast<uint8_t>(sm0);
    smRead_    = static_cast<uint8_t>(sm1);

    const uint16_t* writeProgram;
    uint writeProgramLen;
    const uint16_t* readProgram;
    uint readProgramLen;
    uint dataBits;

    if (config_.mode == Pio2MemMode::OPI_PSRAM) {
        writeProgram   = opi_write_program;
        writeProgramLen = opi_write_program_len;
        readProgram    = opi_read_program;
        readProgramLen  = opi_read_program_len;
        dataBits = 8;
    } else {
        writeProgram   = qspi_write_program;
        writeProgramLen = qspi_write_program_len;
        readProgram    = qspi_read_program;
        readProgramLen  = qspi_read_program_len;
        dataBits = 4;
    }

    // Load write program into PIO instruction memory
    pio_program_t writeProg = {
        .instructions = writeProgram,
        .length = static_cast<uint8_t>(writeProgramLen),
        .origin = -1,
    };
    if (!pio_can_add_program(pio, &writeProg)) {
        printf("[OpiPsram] ERROR: PIO%u instruction memory full (write program)\n",
               config_.pioInstance);
        return false;
    }
    pioProgOffset_ = pio_add_program(pio, &writeProg);

    // Configure SM0 (command/write)
    pio_sm_config smCfgW = pio_get_default_sm_config();
    sm_config_set_wrap(&smCfgW, pioProgOffset_, pioProgOffset_ + writeProgramLen - 1);
    sm_config_set_out_pins(&smCfgW, config_.dataBasePin, dataBits);
    sm_config_set_sideset_pins(&smCfgW, config_.clkPin);
    sm_config_set_sideset(&smCfgW, 1, false, false);
    sm_config_set_out_shift(&smCfgW, false, true, dataBits);  // MSB first, autopull
    sm_config_set_clkdiv(&smCfgW,
        (float)clock_get_hz(clk_sys) / (config_.clockMHz * 1000000.0f * 2.0f));

    pio_sm_init(pio, smCommand_, pioProgOffset_, &smCfgW);

    // Map data output pins to SM0
    for (uint8_t i = 0; i < dataBits; ++i) {
        pio_gpio_init(pio, config_.dataBasePin + i);
    }
    pio_gpio_init(pio, config_.clkPin);
    pio_sm_set_consecutive_pindirs(pio, smCommand_, config_.dataBasePin, dataBits, true);
    pio_sm_set_consecutive_pindirs(pio, smCommand_, config_.clkPin, 1, true);

    // Load read program
    pio_program_t readProg = {
        .instructions = readProgram,
        .length = static_cast<uint8_t>(readProgramLen),
        .origin = -1,
    };
    if (!pio_can_add_program(pio, &readProg)) {
        printf("[OpiPsram] ERROR: PIO%u instruction memory full (read program)\n",
               config_.pioInstance);
        return false;
    }
    uint readOffset = pio_add_program(pio, &readProg);

    // Configure SM1 (read)
    pio_sm_config smCfgR = pio_get_default_sm_config();
    sm_config_set_wrap(&smCfgR, readOffset, readOffset + readProgramLen - 1);
    sm_config_set_in_pins(&smCfgR, config_.dataBasePin);
    sm_config_set_sideset_pins(&smCfgR, config_.clkPin);
    sm_config_set_sideset(&smCfgR, 1, false, false);
    sm_config_set_in_shift(&smCfgR, false, true, dataBits);  // MSB first, autopush
    sm_config_set_clkdiv(&smCfgR,
        (float)clock_get_hz(clk_sys) / (config_.clockMHz * 1000000.0f * 2.0f));

    pio_sm_init(pio, smRead_, readOffset, &smCfgR);

    // SM1 data pins as input
    pio_sm_set_consecutive_pindirs(pio, smRead_, config_.dataBasePin, dataBits, false);

    // Don't enable SMs yet — they start on demand per transfer
    printf("[OpiPsram] PIO%u programs loaded: SM%u=cmd/write (offset %u), "
           "SM%u=read (offset %u), %u-bit data bus\n",
           config_.pioInstance, smCommand_, pioProgOffset_,
           smRead_, readOffset, dataBits);

    return true;
}

// ─── Shutdown ───────────────────────────────────────────────────────────────

void OpiPsramDriver::Shutdown() {
    if (!initialized_) return;

    PIO pio = GetPioInstance(config_.pioInstance);

    // Disable DMA IRQ before shutting down
    if (irqConfigured_) {
        uint irqNum = (dmaIrqIndex_ == 0) ? DMA_IRQ_0 : DMA_IRQ_1;
        dma_irqn_set_channel_enabled(dmaIrqIndex_, dmaChannelRx_, false);
        dma_irqn_set_channel_enabled(dmaIrqIndex_, dmaChannelTx_, false);
        irq_set_enabled(irqNum, false);
        irq_remove_handler(irqNum, &OpiPsramDriver::DmaIrqHandler);
        irqConfigured_ = false;
        s_irqInstance_ = nullptr;
    }

    // Stop state machines
    pio_sm_set_enabled(pio, smCommand_, false);
    pio_sm_set_enabled(pio, smRead_, false);

    // Abort DMA
    if (dmaChannelTx_ >= 0) {
        dma_channel_abort(dmaChannelTx_);
        dma_channel_unclaim(dmaChannelTx_);
        dmaChannelTx_ = -1;
    }
    if (dmaChannelRx_ >= 0) {
        dma_channel_abort(dmaChannelRx_);
        dma_channel_unclaim(dmaChannelRx_);
        dmaChannelRx_ = -1;
    }

    // Unclaim SMs
    pio_sm_unclaim(pio, smCommand_);
    pio_sm_unclaim(pio, smRead_);

    // Deassert CS lines
    gpio_put(config_.cs0Pin, 1);
    if (config_.mode == Pio2MemMode::DUAL_QSPI_MRAM) {
        gpio_put(config_.cs1Pin, 1);
    }

    initialized_ = false;
    printf("[OpiPsram] Shutdown complete\n");
}

// ─── Chip Select Helpers ────────────────────────────────────────────────────

uint8_t OpiPsramDriver::ChipIndexForAddress(uint32_t addr) const {
    if (config_.mode != Pio2MemMode::DUAL_QSPI_MRAM) return 0;
    // Address bit 17 selects chip: 0x00000–0x1FFFF = chip 0, 0x20000+ = chip 1
    return (addr >= config_.chipCapacity) ? 1 : 0;
}

uint32_t OpiPsramDriver::ChipLocalAddress(uint32_t addr) const {
    if (config_.mode != Pio2MemMode::DUAL_QSPI_MRAM) return addr;
    return addr % config_.chipCapacity;
}

static void AssertCs(uint8_t csPin) {
    gpio_put(csPin, 0);
}

static void DeassertCs(uint8_t csPin) {
    gpio_put(csPin, 1);
}

// ─── Data Direction ─────────────────────────────────────────────────────────

void OpiPsramDriver::SetDataDirection(bool output) {
    PIO pio = GetPioInstance(config_.pioInstance);
    pio_sm_set_consecutive_pindirs(pio, smCommand_,
                                   config_.dataBasePin,
                                   config_.dataPinCount, output);
}

// ─── OPI Command Issue ──────────────────────────────────────────────────────

void OpiPsramDriver::IssueOpiCommand(uint8_t opcode, uint32_t address) {
    PIO pio = GetPioInstance(config_.pioInstance);

    // OPI protocol: 1 byte opcode + 3 bytes address, 8 bits per clock
    // Pack into 32-bit word: [opcode][addr2][addr1][addr0]
    uint32_t cmdWord = (static_cast<uint32_t>(opcode) << 24)
                     | (address & 0x00FFFFFF);

    pio_sm_set_enabled(pio, smCommand_, true);
    pio_sm_put_blocking(pio, smCommand_, cmdWord);

    // Wait for the command to shift out (4 bytes × 1 byte/clock)
    // At 75 MHz PIO clock, this is ~53 ns per byte → ~213 ns total
    busy_wait_us_32(1);

    pio_sm_set_enabled(pio, smCommand_, false);
}

// ─── QSPI Command Issue ────────────────────────────────────────────────────

void OpiPsramDriver::IssueQspiCommand(uint8_t opcode, uint32_t address,
                                       uint8_t chipIndex) {
    PIO pio = GetPioInstance(config_.pioInstance);

    // Select appropriate CS
    uint8_t csPin = (chipIndex == 1) ? config_.cs1Pin : config_.cs0Pin;

    // QSPI protocol: opcode nibbles + address nibbles, 4 bits per clock
    // cmd=2 clocks (8 bits), addr=6 clocks (24 bits) = 32 bits total
    uint32_t cmdWord = (static_cast<uint32_t>(opcode) << 24)
                     | (address & 0x00FFFFFF);

    AssertCs(csPin);
    pio_sm_set_enabled(pio, smCommand_, true);
    pio_sm_put_blocking(pio, smCommand_, cmdWord);

    // Wait for command phase to complete
    busy_wait_us_32(1);

    pio_sm_set_enabled(pio, smCommand_, false);
    // Note: caller controls CS deassertion after data phase
}

// ─── MRAM WREN (Write Enable) ──────────────────────────────────────────────

void OpiPsramDriver::IssueMramWren(uint8_t chipIndex) {
    PIO pio = GetPioInstance(config_.pioInstance);

    uint8_t csPin = (chipIndex == 1) ? config_.cs1Pin : config_.cs0Pin;

    // WREN is a single-byte command: 0x06
    // In SPI mode: 8 clock cycles for the opcode byte
    SetDataDirection(true);
    AssertCs(csPin);

    pio_sm_set_enabled(pio, smCommand_, true);
    // Pack WREN opcode in upper byte, shift out 8 bits
    pio_sm_put_blocking(pio, smCommand_, 0x06000000u);

    busy_wait_us_32(1);

    pio_sm_set_enabled(pio, smCommand_, false);
    DeassertCs(csPin);

    // MRAM WREN takes effect immediately (no WIP delay)
}

// ─── Synchronous Read ───────────────────────────────────────────────────────

bool OpiPsramDriver::ReadSync(uint32_t srcAddr, void* dest, uint32_t length) {
    if (!initialized_ || !dest || length == 0) return false;
    if (srcAddr + length > config_.capacityBytes) return false;

    PIO pio = GetPioInstance(config_.pioInstance);

    if (config_.mode == Pio2MemMode::OPI_PSRAM) {
        // ── OPI PSRAM Read ──────────────────────────────────────────────
        // Protocol: CS low → 0x03 (Read) + 24-bit addr → latency wait → data
        SetDataDirection(true);
        AssertCs(config_.cs0Pin);

        IssueOpiCommand(0x03, srcAddr);

        // Wait for read latency
        for (uint8_t i = 0; i < config_.readLatency; ++i) {
            busy_wait_us_32(1);
        }

        // Switch data pins to input
        SetDataDirection(false);

        // DMA read from PIO RX FIFO → dest
        pio_sm_clear_fifos(pio, smRead_);
        pio_sm_set_enabled(pio, smRead_, true);

        uint32_t words = (length + 3) / 4;
        dma_channel_set_write_addr(dmaChannelRx_, dest, false);
        dma_channel_set_trans_count(dmaChannelRx_, words, true);

        // Wait for DMA completion
        dma_channel_wait_for_finish_blocking(dmaChannelRx_);

        pio_sm_set_enabled(pio, smRead_, false);
        DeassertCs(config_.cs0Pin);

        totalReads_++;
        return true;

    } else {
        // ── QSPI MRAM Read (single or dual) ────────────────────────────
        // For dual mode, handle cross-chip boundary reads
        uint32_t remaining = length;
        uint32_t addr = srcAddr;
        uint8_t* destPtr = static_cast<uint8_t*>(dest);

        while (remaining > 0) {
            uint8_t chip = ChipIndexForAddress(addr);
            uint32_t localAddr = ChipLocalAddress(addr);

            // Calculate bytes until chip boundary
            uint32_t chipRemaining = config_.chipCapacity - localAddr;
            uint32_t chunkLen = (remaining < chipRemaining) ? remaining : chipRemaining;

            uint8_t csPin = (chip == 1) ? config_.cs1Pin : config_.cs0Pin;

            // Quad Read: 0xEB + addr + 8 dummy cycles + data
            SetDataDirection(true);
            AssertCs(csPin);

            // Send command: 0xEB + 24-bit address
            IssueQspiCommand(0xEB, localAddr, chip);

            // Dummy cycles (8 clocks at 4-bit = 32 bits of dummy)
            pio_sm_set_enabled(pio, smCommand_, true);
            pio_sm_put_blocking(pio, smCommand_, 0x00000000u);
            pio_sm_put_blocking(pio, smCommand_, 0x00000000u);
            busy_wait_us_32(1);
            pio_sm_set_enabled(pio, smCommand_, false);

            // Switch to read
            SetDataDirection(false);

            pio_sm_clear_fifos(pio, smRead_);
            pio_sm_set_enabled(pio, smRead_, true);

            uint32_t words = (chunkLen + 3) / 4;
            dma_channel_set_write_addr(dmaChannelRx_, destPtr, false);
            dma_channel_set_trans_count(dmaChannelRx_, words, true);
            dma_channel_wait_for_finish_blocking(dmaChannelRx_);

            pio_sm_set_enabled(pio, smRead_, false);
            DeassertCs(csPin);

            destPtr += chunkLen;
            addr += chunkLen;
            remaining -= chunkLen;
        }

        totalReads_++;
        return true;
    }
}

// ─── Synchronous Write ──────────────────────────────────────────────────────

bool OpiPsramDriver::WriteSync(uint32_t destAddr, const void* src, uint32_t length) {
    if (!initialized_ || !src || length == 0) return false;
    if (destAddr + length > config_.capacityBytes) return false;

    PIO pio = GetPioInstance(config_.pioInstance);

    if (config_.mode == Pio2MemMode::OPI_PSRAM) {
        // ── OPI PSRAM Write ─────────────────────────────────────────────
        // Protocol: CS low → 0x02 (Write) + 24-bit addr → data
        SetDataDirection(true);
        AssertCs(config_.cs0Pin);

        IssueOpiCommand(0x02, destAddr);

        // DMA write from src → PIO TX FIFO
        uint32_t words = (length + 3) / 4;
        dma_channel_set_read_addr(dmaChannelTx_, src, false);
        dma_channel_set_trans_count(dmaChannelTx_, words, true);

        pio_sm_set_enabled(pio, smCommand_, true);
        dma_channel_wait_for_finish_blocking(dmaChannelTx_);

        // Wait for PIO FIFO to drain
        while (!pio_sm_is_tx_fifo_empty(pio, smCommand_)) {
            tight_loop_contents();
        }

        pio_sm_set_enabled(pio, smCommand_, false);
        DeassertCs(config_.cs0Pin);

        totalWrites_++;
        return true;

    } else {
        // ── QSPI MRAM Write (single or dual) ───────────────────────────
        uint32_t remaining = length;
        uint32_t addr = destAddr;
        const uint8_t* srcPtr = static_cast<const uint8_t*>(src);

        while (remaining > 0) {
            uint8_t chip = ChipIndexForAddress(addr);
            uint32_t localAddr = ChipLocalAddress(addr);

            uint32_t chipRemaining = config_.chipCapacity - localAddr;
            uint32_t chunkLen = (remaining < chipRemaining) ? remaining : chipRemaining;

            uint8_t csPin = (chip == 1) ? config_.cs1Pin : config_.cs0Pin;

            // MRAM requires WREN (0x06) before each write
            if (config_.needsWrenBeforeWrite) {
                IssueMramWren(chip);
            }

            // Quad Write: 0x38 + addr + data
            SetDataDirection(true);
            AssertCs(csPin);

            IssueQspiCommand(0x38, localAddr, chip);

            // DMA write: src → PIO TX FIFO
            uint32_t words = (chunkLen + 3) / 4;
            dma_channel_set_read_addr(dmaChannelTx_, srcPtr, false);
            dma_channel_set_trans_count(dmaChannelTx_, words, true);

            pio_sm_set_enabled(pio, smCommand_, true);
            dma_channel_wait_for_finish_blocking(dmaChannelTx_);

            while (!pio_sm_is_tx_fifo_empty(pio, smCommand_)) {
                tight_loop_contents();
            }

            pio_sm_set_enabled(pio, smCommand_, false);
            DeassertCs(csPin);

            srcPtr += chunkLen;
            addr += chunkLen;
            remaining -= chunkLen;
        }

        totalWrites_++;
        return true;
    }
}

// ─── Asynchronous Read ──────────────────────────────────────────────────────

bool OpiPsramDriver::ReadAsync(uint32_t srcAddr, void* dest, uint32_t length) {
    if (!initialized_ || !dest || length == 0) return false;
    if (srcAddr + length > config_.capacityBytes) return false;

    PIO pio = GetPioInstance(config_.pioInstance);
    pendingChip1_.pending = false;  // Clear any stale pending

    if (config_.mode == Pio2MemMode::OPI_PSRAM) {
        // ── OPI async read (unchanged) ──────────────────────────────────
        SetDataDirection(true);
        AssertCs(config_.cs0Pin);
        IssueOpiCommand(0x03, srcAddr);

        for (uint8_t i = 0; i < config_.readLatency; ++i) {
            busy_wait_us_32(1);
        }

        SetDataDirection(false);
        pio_sm_clear_fifos(pio, smRead_);
        pio_sm_set_enabled(pio, smRead_, true);

        uint32_t words = (length + 3) / 4;
        dma_channel_set_write_addr(dmaChannelRx_, dest, false);
        dma_channel_set_trans_count(dmaChannelRx_, words, true);

        // Returns immediately — caller checks GetDmaStatus() or WaitDma()
        return true;

    } else {
        // ── QSPI async read with dual-chip cross-boundary support ───────
        uint8_t chip0 = ChipIndexForAddress(srcAddr);
        uint32_t localAddr = ChipLocalAddress(srcAddr);
        uint8_t csPin = (chip0 == 1) ? config_.cs1Pin : config_.cs0Pin;

        // How many bytes fit within this chip?
        uint32_t chipRemaining = config_.chipCapacity - localAddr;
        uint32_t chunk0Len = (length <= chipRemaining) ? length : chipRemaining;
        uint32_t chunk1Len = length - chunk0Len;

        // If cross-chip transfer needed, save the second part for IRQ handler
        if (chunk1Len > 0 && config_.mode == Pio2MemMode::DUAL_QSPI_MRAM) {
            uint8_t chip1 = 1 - chip0;  // the other chip
            pendingChip1_.srcAddr = (chip1 == 1) ? config_.chipCapacity : 0;
            pendingChip1_.dest    = static_cast<uint8_t*>(dest) + chunk0Len;
            pendingChip1_.length  = chunk1Len;
            pendingChip1_.csPin   = (chip1 == 1) ? config_.cs1Pin : config_.cs0Pin;
            pendingChip1_.pending = true;
        }

        // Start first chip read
        SetDataDirection(true);
        AssertCs(csPin);
        IssueQspiCommand(0xEB, localAddr, chip0);

        // Dummy clocks
        pio_sm_set_enabled(pio, smCommand_, true);
        pio_sm_put_blocking(pio, smCommand_, 0x00000000u);
        pio_sm_put_blocking(pio, smCommand_, 0x00000000u);
        busy_wait_us_32(1);
        pio_sm_set_enabled(pio, smCommand_, false);

        SetDataDirection(false);
        pio_sm_clear_fifos(pio, smRead_);
        pio_sm_set_enabled(pio, smRead_, true);

        uint32_t words = (chunk0Len + 3) / 4;
        dma_channel_set_write_addr(dmaChannelRx_, dest, false);
        dma_channel_set_trans_count(dmaChannelRx_, words, true);

        // IRQ handler will pick up pendingChip1_ if set
        return true;
    }
}

// ─── Asynchronous Write ─────────────────────────────────────────────────────

bool OpiPsramDriver::WriteAsync(uint32_t destAddr, const void* src, uint32_t length) {
    if (!initialized_ || !src || length == 0) return false;
    if (destAddr + length > config_.capacityBytes) return false;

    PIO pio = GetPioInstance(config_.pioInstance);

    if (config_.mode == Pio2MemMode::OPI_PSRAM) {
        SetDataDirection(true);
        AssertCs(config_.cs0Pin);
        IssueOpiCommand(0x02, destAddr);

        uint32_t words = (length + 3) / 4;
        dma_channel_set_read_addr(dmaChannelTx_, src, false);
        dma_channel_set_trans_count(dmaChannelTx_, words, true);

        pio_sm_set_enabled(pio, smCommand_, true);
        // Returns immediately — DMA runs in background
        return true;

    } else {
        // MRAM async: issue WREN synchronously, then start async DMA write
        uint8_t chip = ChipIndexForAddress(destAddr);
        uint32_t localAddr = ChipLocalAddress(destAddr);
        uint8_t csPin = (chip == 1) ? config_.cs1Pin : config_.cs0Pin;

        if (config_.needsWrenBeforeWrite) {
            IssueMramWren(chip);
        }

        SetDataDirection(true);
        AssertCs(csPin);
        IssueQspiCommand(0x38, localAddr, chip);

        uint32_t words = (length + 3) / 4;
        dma_channel_set_read_addr(dmaChannelTx_, src, false);
        dma_channel_set_trans_count(dmaChannelTx_, words, true);

        pio_sm_set_enabled(pio, smCommand_, true);
        return true;
    }
}

// ─── DMA Status ─────────────────────────────────────────────────────────────

OpiDmaStatus OpiPsramDriver::GetDmaStatus() const {
    if (!initialized_) return OpiDmaStatus::ERROR;

    bool txBusy = dma_channel_is_busy(dmaChannelTx_);
    bool rxBusy = dma_channel_is_busy(dmaChannelRx_);

    if (txBusy) return OpiDmaStatus::WRITING;
    if (rxBusy) return OpiDmaStatus::READING;

    return OpiDmaStatus::IDLE;
}

void OpiPsramDriver::WaitDma() {
    if (!initialized_) return;

    PIO pio = GetPioInstance(config_.pioInstance);

    // Wait for RX DMA (read path)
    if (dma_channel_is_busy(dmaChannelRx_)) {
        dma_channel_wait_for_finish_blocking(dmaChannelRx_);
        pio_sm_set_enabled(pio, smRead_, false);
        // Deassert CS after read completes
        DeassertCs(config_.cs0Pin);
        if (config_.mode == Pio2MemMode::DUAL_QSPI_MRAM) {
            DeassertCs(config_.cs1Pin);
        }
        totalReads_++;
    }

    // If there's a pending chip-1 read (cross-boundary async), start it now
    if (pendingChip1_.pending) {
        StartPendingChip1Read();
        // Wait for chip-1 to finish too
        dma_channel_wait_for_finish_blocking(dmaChannelRx_);
        pio_sm_set_enabled(pio, smRead_, false);
        DeassertCs(pendingChip1_.csPin);
        pendingChip1_.pending = false;
        totalReads_++;
    }

    // Wait for TX DMA (write path)
    if (dma_channel_is_busy(dmaChannelTx_)) {
        dma_channel_wait_for_finish_blocking(dmaChannelTx_);
        while (!pio_sm_is_tx_fifo_empty(pio, smCommand_)) {
            tight_loop_contents();
        }
        pio_sm_set_enabled(pio, smCommand_, false);
        DeassertCs(config_.cs0Pin);
        if (config_.mode == Pio2MemMode::DUAL_QSPI_MRAM) {
            DeassertCs(config_.cs1Pin);
        }
        totalWrites_++;
    }
}

// ─── Prefetch ───────────────────────────────────────────────────────────────

bool OpiPsramDriver::Prefetch(uint32_t srcAddr, uint8_t* sramDest, uint32_t length) {
    // Prefetch is just an async read into the SRAM cache line
    return ReadAsync(srcAddr, sramDest, length);
}

// ─── Free-List Allocator ────────────────────────────────────────────────────

void OpiPsramDriver::InitFreeList() {
    freeBlockCount_ = 1;
    freeList_[0].addr = 0;
    freeList_[0].size = config_.capacityBytes;
    for (uint16_t i = 1; i < MAX_FREE_BLOCKS; ++i) {
        freeList_[i].addr = 0;
        freeList_[i].size = 0;
    }
}

void OpiPsramDriver::CoalesceFreeList() {
    if (freeBlockCount_ <= 1) return;

    // Sort by address (insertion sort — list is small)
    for (uint16_t i = 1; i < freeBlockCount_; ++i) {
        FreeBlock key = freeList_[i];
        int j = static_cast<int>(i) - 1;
        while (j >= 0 && freeList_[j].addr > key.addr) {
            freeList_[j + 1] = freeList_[j];
            --j;
        }
        freeList_[j + 1] = key;
    }

    // Merge adjacent blocks
    uint16_t dst = 0;
    for (uint16_t src = 1; src < freeBlockCount_; ++src) {
        if (freeList_[dst].addr + freeList_[dst].size == freeList_[src].addr) {
            // Merge: extend dst block to absorb src block
            freeList_[dst].size += freeList_[src].size;
        } else {
            ++dst;
            freeList_[dst] = freeList_[src];
        }
    }
    freeBlockCount_ = dst + 1;
}

uint32_t OpiPsramDriver::Alloc(uint32_t size, uint32_t alignment) {
    if (!initialized_ || size == 0) return 0xFFFFFFFF;
    if (alignment == 0) alignment = 1;

    // First-fit search through free list
    for (uint16_t i = 0; i < freeBlockCount_; ++i) {
        uint32_t alignedAddr = (freeList_[i].addr + alignment - 1) & ~(alignment - 1);
        uint32_t padding = alignedAddr - freeList_[i].addr;
        uint32_t totalNeeded = padding + size;

        if (freeList_[i].size >= totalNeeded) {
            uint32_t resultAddr = alignedAddr;

            if (padding > 0) {
                // Split: keep the front padding as a free block
                // The allocated region starts at alignedAddr
                uint32_t remainAfter = freeList_[i].size - totalNeeded;

                freeList_[i].size = padding;  // Shrink to front padding

                if (remainAfter > 0 && freeBlockCount_ < MAX_FREE_BLOCKS) {
                    // Insert remainder block after allocation
                    for (uint16_t j = freeBlockCount_; j > i + 1; --j) {
                        freeList_[j] = freeList_[j - 1];
                    }
                    freeList_[i + 1].addr = alignedAddr + size;
                    freeList_[i + 1].size = remainAfter;
                    ++freeBlockCount_;
                }
            } else {
                // No padding — allocate from block start
                uint32_t remainAfter = freeList_[i].size - size;

                if (remainAfter > 0) {
                    // Shrink block
                    freeList_[i].addr += size;
                    freeList_[i].size = remainAfter;
                } else {
                    // Remove block entirely
                    for (uint16_t j = i; j + 1 < freeBlockCount_; ++j) {
                        freeList_[j] = freeList_[j + 1];
                    }
                    --freeBlockCount_;
                }
            }

            return resultAddr;
        }
    }

    return 0xFFFFFFFF;  // OOM
}

void OpiPsramDriver::Free(uint32_t addr) {
    if (!initialized_) return;

    // We don't track allocation sizes, so Free must be passed size info.
    // However, the header only takes addr — we store a sentinel for now.
    // Strategy: find the gap between existing free blocks where addr fits,
    // reconstruct size = distance to next free block (or end of capacity).
    //
    // Better approach: the tier manager and gpu_core track sizes in their
    // allocation records, so we add a sized Free variant.  For the simple
    // Free(addr), we insert a zero-size block and rely on the next Alloc
    // call's coalesce to fix it.
    //
    // NOTE: In practice, callers should use FreeAll() for bulk reset,
    // or the tier manager passes size.  This is a safety fallback.

    // Search for the insertion point and try to infer size from gaps.
    // If addr is between two free blocks, the allocation was the gap.
    uint32_t allocSize = 0;

    // Sort first for reliable gap detection
    CoalesceFreeList();

    if (freeBlockCount_ == 0) {
        allocSize = config_.capacityBytes;  // Everything was allocated
    } else {
        // Find where addr falls in the sorted free list
        bool found = false;
        for (uint16_t i = 0; i < freeBlockCount_; ++i) {
            if (freeList_[i].addr > addr) {
                // addr is before this free block
                uint32_t end = freeList_[i].addr;
                uint32_t start = (i > 0) ? (freeList_[i - 1].addr + freeList_[i - 1].size) : 0;
                if (addr >= start && addr < end) {
                    allocSize = end - addr;
                    found = true;
                }
                break;
            }
        }
        if (!found) {
            // addr is after all free blocks
            uint32_t lastEnd = freeList_[freeBlockCount_ - 1].addr
                             + freeList_[freeBlockCount_ - 1].size;
            if (addr >= lastEnd) {
                allocSize = config_.capacityBytes - addr;
            }
        }
    }

    if (allocSize == 0) return;  // Invalid or double-free

    // Insert the freed block
    if (freeBlockCount_ < MAX_FREE_BLOCKS) {
        freeList_[freeBlockCount_].addr = addr;
        freeList_[freeBlockCount_].size = allocSize;
        ++freeBlockCount_;
        CoalesceFreeList();
    }
}

void OpiPsramDriver::FreeAll() {
    InitFreeList();
}

uint32_t OpiPsramDriver::Available() const {
    if (!initialized_) return 0;
    uint32_t total = 0;
    for (uint16_t i = 0; i < freeBlockCount_; ++i) {
        total += freeList_[i].size;
    }
    return total;
}

// ─── RDID Verification ─────────────────────────────────────────────────────

bool OpiPsramDriver::VerifyChipId() {
    if (!initialized_) return false;

    PIO pio = GetPioInstance(config_.pioInstance);

    if (config_.mode == Pio2MemMode::OPI_PSRAM) {
        // ── OPI PSRAM RDID: send 0x9F + 3-byte dummy addr ──────────────
        // Response: manufacturer ID in first byte (expect 0x0D for AP Memory)
        SetDataDirection(true);
        AssertCs(config_.cs0Pin);

        IssueOpiCommand(0x9F, 0x000000);

        // Wait for read latency
        for (uint8_t i = 0; i < config_.readLatency; ++i) {
            busy_wait_us_32(1);
        }

        // Switch to read and capture 2 bytes (mfr ID + KGD)
        SetDataDirection(false);
        pio_sm_clear_fifos(pio, smRead_);
        pio_sm_set_enabled(pio, smRead_, true);

        // Read 1 word (4 bytes) — we only need the first 2
        uint32_t rxWord = 0;
        dma_channel_set_write_addr(dmaChannelRx_, &rxWord, false);
        dma_channel_set_trans_count(dmaChannelRx_, 1, true);
        dma_channel_wait_for_finish_blocking(dmaChannelRx_);

        pio_sm_set_enabled(pio, smRead_, false);
        DeassertCs(config_.cs0Pin);

        // APS6408L: manufacturer ID = 0x0D, KGD = 0x5D
        uint8_t mfrId = (rxWord >> 24) & 0xFF;
        uint8_t kgd   = (rxWord >> 16) & 0xFF;

        printf("[OpiPsram] RDID: MFR=0x%02X KGD=0x%02X\n", mfrId, kgd);

        // AP Memory manufacturer ID is 0x0D
        return (mfrId == 0x0D);

    } else {
        // ── QSPI MRAM RDID: 0x9F → manufacturer + device ID ────────────
        // MR10Q010: RDID response = manufacturer (0x07) + device (0x6B +...)
        for (uint8_t chip = 0; chip < config_.chipCount; ++chip) {
            uint8_t csPin = (chip == 1) ? config_.cs1Pin : config_.cs0Pin;

            SetDataDirection(true);
            AssertCs(csPin);

            // Send RDID command (0x9F) — standard SPI, 8 bits via PIO
            pio_sm_set_enabled(pio, smCommand_, true);
            pio_sm_put_blocking(pio, smCommand_, 0x9F000000u);
            busy_wait_us_32(1);
            pio_sm_set_enabled(pio, smCommand_, false);

            // Switch to read — capture 3 bytes (MFR + 2-byte device ID)
            SetDataDirection(false);
            pio_sm_clear_fifos(pio, smRead_);
            pio_sm_set_enabled(pio, smRead_, true);

            uint32_t rxWord = 0;
            dma_channel_set_write_addr(dmaChannelRx_, &rxWord, false);
            dma_channel_set_trans_count(dmaChannelRx_, 1, true);
            dma_channel_wait_for_finish_blocking(dmaChannelRx_);

            pio_sm_set_enabled(pio, smRead_, false);
            DeassertCs(csPin);

            uint8_t mfrId  = (rxWord >> 24) & 0xFF;
            uint8_t devId0 = (rxWord >> 16) & 0xFF;
            uint8_t devId1 = (rxWord >>  8) & 0xFF;

            printf("[OpiPsram] Chip %u RDID: MFR=0x%02X DEV=0x%02X%02X\n",
                   chip, mfrId, devId0, devId1);

            // Everspin MR10Q010: MFR=0x07, Device=0x6B
            if (mfrId != 0x07) {
                printf("[OpiPsram] WARNING: Chip %u MFR mismatch "
                       "(got 0x%02X, expected 0x07)\n", chip, mfrId);
                return false;
            }
        }
        return true;
    }
}

// ─── DMA IRQ Setup ──────────────────────────────────────────────────────────

void OpiPsramDriver::SetupDmaIrq() {
    // Try DMA_IRQ_0 first, fall back to DMA_IRQ_1
    s_irqInstance_ = this;

    // Prefer IRQ bank 0, but check if it's already in use
    for (uint8_t bank = 0; bank < 2; ++bank) {
        uint irqNum = (bank == 0) ? DMA_IRQ_0 : DMA_IRQ_1;

        // Try to set our handler — if irq_set_exclusive_handler succeeds,
        // we got the bank.  The Pico SDK will hard_assert if already claimed,
        // so we use irq_has_shared_handler + irq_get_vtable_handler to check.
        if (irq_get_exclusive_handler(irqNum) == nullptr) {
            irq_set_exclusive_handler(irqNum, &OpiPsramDriver::DmaIrqHandler);
            dma_irqn_set_channel_enabled(bank, dmaChannelRx_, true);
            dma_irqn_set_channel_enabled(bank, dmaChannelTx_, true);
            irq_set_enabled(irqNum, true);

            dmaIrqIndex_ = bank;
            irqConfigured_ = true;

            printf("[OpiPsram] DMA IRQ%u configured for channels %d/%d\n",
                   bank, dmaChannelRx_, dmaChannelTx_);
            return;
        }
    }

    printf("[OpiPsram] WARNING: Could not claim DMA IRQ — "
           "async callbacks will use polling fallback\n");
    irqConfigured_ = false;
}

void OpiPsramDriver::DmaIrqHandler() {
    OpiPsramDriver* self = s_irqInstance_;
    if (!self) return;

    bool rxDone = false;
    bool txDone = false;

    // Check and acknowledge RX channel completion
    if (self->irqConfigured_ &&
        dma_irqn_get_channel_status(self->dmaIrqIndex_, self->dmaChannelRx_)) {
        dma_irqn_acknowledge_channel(self->dmaIrqIndex_, self->dmaChannelRx_);
        rxDone = true;
    }

    // Check and acknowledge TX channel completion
    if (self->irqConfigured_ &&
        dma_irqn_get_channel_status(self->dmaIrqIndex_, self->dmaChannelTx_)) {
        dma_irqn_acknowledge_channel(self->dmaIrqIndex_, self->dmaChannelTx_);
        txDone = true;
    }

    if (rxDone) {
        // Read completed — stop PIO SM and deassert CS
        PIO pio = GetPioInstance(self->config_.pioInstance);
        pio_sm_set_enabled(pio, self->smRead_, false);

        // Deassert whichever CS was active
        DeassertCs(self->config_.cs0Pin);
        if (self->config_.mode == Pio2MemMode::DUAL_QSPI_MRAM) {
            DeassertCs(self->config_.cs1Pin);
        }

        self->totalReads_++;

        // If there's a pending chip-1 read, start it now
        if (self->pendingChip1_.pending) {
            self->StartPendingChip1Read();
            return;  // Will get another IRQ when chip-1 DMA finishes
        }

        // No pending — invoke user callback
        if (self->dmaCallback_) {
            self->dmaCallback_(true, self->dmaCallbackCtx_);
        }
    }

    if (txDone) {
        // Write completed — drain FIFO and deassert CS
        PIO pio = GetPioInstance(self->config_.pioInstance);
        // Note: in IRQ context we can't busy-wait long, but TX FIFO
        // should be nearly empty by the time DMA signals completion.
        pio_sm_set_enabled(pio, self->smCommand_, false);

        DeassertCs(self->config_.cs0Pin);
        if (self->config_.mode == Pio2MemMode::DUAL_QSPI_MRAM) {
            DeassertCs(self->config_.cs1Pin);
        }

        self->totalWrites_++;

        if (self->dmaCallback_) {
            self->dmaCallback_(true, self->dmaCallbackCtx_);
        }
    }
}

// ─── Pending Chip-1 Read (Cross-Boundary) ───────────────────────────────────

void OpiPsramDriver::StartPendingChip1Read() {
    if (!pendingChip1_.pending) return;

    PIO pio = GetPioInstance(config_.pioInstance);

    uint8_t chip = (pendingChip1_.csPin == config_.cs1Pin) ? 1 : 0;
    uint32_t localAddr = ChipLocalAddress(pendingChip1_.srcAddr);

    // Issue QSPI Quad Read command to the second chip
    SetDataDirection(true);
    AssertCs(pendingChip1_.csPin);
    IssueQspiCommand(0xEB, localAddr, chip);

    // Dummy clocks
    pio_sm_set_enabled(pio, smCommand_, true);
    pio_sm_put_blocking(pio, smCommand_, 0x00000000u);
    pio_sm_put_blocking(pio, smCommand_, 0x00000000u);
    busy_wait_us_32(1);
    pio_sm_set_enabled(pio, smCommand_, false);

    // Switch to read
    SetDataDirection(false);
    pio_sm_clear_fifos(pio, smRead_);
    pio_sm_set_enabled(pio, smRead_, true);

    uint32_t words = (pendingChip1_.length + 3) / 4;
    dma_channel_set_write_addr(dmaChannelRx_, pendingChip1_.dest, false);
    dma_channel_set_trans_count(dmaChannelRx_, words, true);

    // Mark as no longer pending — will complete via next IRQ
    pendingChip1_.pending = false;
}

// ─── DMA Callback API ──────────────────────────────────────────────────────

void OpiPsramDriver::SetDmaCallback(OpiDmaCallback cb, void* userCtx) {
    dmaCallback_    = cb;
    dmaCallbackCtx_ = userCtx;
}

void OpiPsramDriver::ClearDmaCallback() {
    dmaCallback_    = nullptr;
    dmaCallbackCtx_ = nullptr;
}
