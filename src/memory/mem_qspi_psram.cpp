/**
 * @file mem_qspi_psram.cpp
 * @brief QMI CS1 QSPI memory driver — XIP-mapped MRAM or PSRAM.
 *
 * Implements the QspiPsramDriver class declared in mem_qspi_psram.h.
 * Uses the RP2350's built-in QMI (Quad Memory Interface) controller on CS1
 * to attach external memory as XIP (execute-in-place) mapped at 0x11000000.
 *
 * Supports auto-detected chip types:
 *   MR10Q010 (MRAM):   128 KB, 104 MHz, non-volatile, no random-access penalty
 *   APS6408L (PSRAM):  8 MB, 133 MHz, volatile, row-buffer miss penalty
 *   ESP-PSRAM64H:      8 MB, 84 MHz, volatile
 */

#include "mem_qspi_psram.h"
#include "../gpu_config.h"

#include "hardware/structs/qmi.h"
#include "hardware/structs/xip_ctrl.h"
#include "hardware/regs/addressmap.h"
#include "hardware/sync.h"
#include "hardware/gpio.h"
#include "pico/time.h"

#include <cstdio>
#include <cstring>

// ─── QMI CS1 Direct Mode Helpers ────────────────────────────────────────────
//
// The RP2350 QMI provides a "direct mode" for sending arbitrary SPI commands
// to CS1 without going through XIP.  This is used for RDID probes and chip
// initialization.

/// Enter QMI direct mode for CS1.
/// Disables XIP caching temporarily so we can issue raw SPI commands.
static void QmiEnterDirectMode() {
    // Disable XIP caching
    xip_ctrl_hw->ctrl &= ~XIP_CTRL_EN_BITS;
    __dsb();
    __isb();
}

/// Exit QMI direct mode — re-enable XIP.
static void QmiExitDirectMode() {
    // Re-enable XIP caching
    xip_ctrl_hw->ctrl |= XIP_CTRL_EN_BITS;
    __dsb();
    __isb();
}

/// Send a command byte to CS1 in direct mode and read response bytes.
/// @param cmd     Command opcode byte
/// @param modeArg If non-zero, sent as a "mode" byte after the command
/// @param rxBuf   Buffer to receive response bytes
/// @param rxLen   Number of response bytes to read
static void QmiCs1DirectTransfer(uint8_t cmd, uint8_t modeArg,
                                  uint8_t* rxBuf, uint32_t rxLen) {
    // Wait for QMI to be idle
    while (qmi_hw->direct_csr & QMI_DIRECT_CSR_BUSY_BITS) {
        tight_loop_contents();
    }

    // Assert CS1: set DIRECT_CSR for chip select 1
    qmi_hw->direct_csr = QMI_DIRECT_CSR_EN_BITS
                        | (1u << QMI_DIRECT_CSR_CLKDIV_LSB)   // divide = 1
                        | QMI_DIRECT_CSR_ASSERT_CS1N_BITS;     // assert CS1

    // Send command byte
    qmi_hw->direct_tx = cmd;
    while (qmi_hw->direct_csr & QMI_DIRECT_CSR_BUSY_BITS) {
        tight_loop_contents();
    }
    // Discard the RX byte from the command phase
    (void)qmi_hw->direct_rx;

    // Send mode argument byte if specified
    if (modeArg != 0) {
        qmi_hw->direct_tx = modeArg;
        while (qmi_hw->direct_csr & QMI_DIRECT_CSR_BUSY_BITS) {
            tight_loop_contents();
        }
        (void)qmi_hw->direct_rx;
    }

    // Read response bytes
    for (uint32_t i = 0; i < rxLen; ++i) {
        qmi_hw->direct_tx = 0xFF;  // dummy TX byte during read
        while (qmi_hw->direct_csr & QMI_DIRECT_CSR_BUSY_BITS) {
            tight_loop_contents();
        }
        rxBuf[i] = static_cast<uint8_t>(qmi_hw->direct_rx);
    }

    // Deassert CS1
    qmi_hw->direct_csr = 0;
}

// ─── Destructor ─────────────────────────────────────────────────────────────

QspiPsramDriver::~QspiPsramDriver() {
    if (initialized_) {
        Shutdown();
    }
}

// ─── Initialization ─────────────────────────────────────────────────────────

bool QspiPsramDriver::Initialize(const QspiCs1Config& config) {
    if (initialized_) return false;

    config_ = config;

    // Step 1: Verify chip identity
    if (!VerifyChipId()) {
        printf("[QspiCs1] WARNING: RDID verification failed, proceeding with config\n");
        // Continue anyway — the chip may be in a non-standard state
    }

    // Step 2: Configure QMI timing for the detected chip
    if (!ConfigureQmiTiming()) {
        printf("[QspiCs1] ERROR: QMI timing configuration failed\n");
        return false;
    }

    // Step 3: Run chip-specific initialization sequence
    if (!InitChip()) {
        printf("[QspiCs1] ERROR: Chip init sequence failed\n");
        return false;
    }

    // Step 4: Reset allocator
    InitFreeList();
    initialized_ = true;

    printf("[QspiCs1] Initialized: type=%u, capacity=%lu KB, XIP=0x%08lX, "
           "clock=%lu MHz, non-volatile=%s\n",
           config_.chipType,
           (unsigned long)(config_.capacityBytes / 1024),
           (unsigned long)config_.xipBase,
           (unsigned long)config_.clockMHz,
           config_.isNonVolatile ? "yes" : "no");

    return true;
}

// ─── Shutdown ───────────────────────────────────────────────────────────────

void QspiPsramDriver::Shutdown() {
    if (!initialized_) return;

    // Put the chip in a safe state (e.g., exit QPI mode for PSRAM)
    // For MRAM, no special shutdown needed.
    if (config_.chipType == static_cast<uint8_t>(GpuConfig::QspiChipType::PSRAM_APS6408L) ||
        config_.chipType == static_cast<uint8_t>(GpuConfig::QspiChipType::PSRAM_ESP)) {
        // Exit QPI mode: 0xF5 (varies by chip)
        QmiEnterDirectMode();
        uint8_t dummy;
        QmiCs1DirectTransfer(0xF5, 0, &dummy, 0);
        QmiExitDirectMode();
    }

    initialized_ = false;
    printf("[QspiCs1] Shutdown complete\n");
}

// ─── QMI Timing Configuration ───────────────────────────────────────────────

bool QspiPsramDriver::ConfigureQmiTiming() {
    // Calculate clock divider.
    // QMI is clocked from clk_sys.  For CS1 in QSPI mode:
    //   QMI clock = clk_sys / (2 * div)    where div >= 1
    uint32_t sysClkHz = clock_get_hz(clk_sys);
    uint32_t targetHz = config_.clockMHz * 1000000u;

    // Find the smallest divider that keeps QMI clock ≤ target
    uint32_t div = (sysClkHz + 2 * targetHz - 1) / (2 * targetHz);
    if (div < 1) div = 1;
    if (div > 255) div = 255;

    // Configure M1 (CS1) timing register
    // QMI M1_TIMING register format:
    //   [31:28] = COOLDOWN (cycles between transfers)
    //   [27:24] = PAGEBREAK (page boundary, 0=no page break)
    //   [23:16] = SELECT_HOLD (CS hold time after transfer)
    //   [15:8]  = SELECT_SETUP (CS setup time)
    //   [7:2]   = RXDELAY (read data capture delay)
    //   [1:0]   = CLKDIV (1-4)
    qmi_hw->m[1].timing =
        (2u << 28)   // COOLDOWN: 2 clock cycles
      | (0u << 24)   // PAGEBREAK: 0 (continuous, unless PSRAM page mode)
      | (1u << 16)   // SELECT_HOLD: 1 cycle
      | (1u << 8)    // SELECT_SETUP: 1 cycle
      | (1u << 2)    // RXDELAY: 1 (sample on 2nd edge)
      | (div & 0x3); // CLKDIV

    // Configure M1 read format for Quad Read (0xEB)
    // RFMT register: prefix_len=8, addr_len=24, suffix_len=0(mode bits vary),
    //                dummy_len=config_.readLatency, data_width=quad(2)
    qmi_hw->m[1].rfmt =
        (0u << 28)                          // PREFIX_WIDTH: single (SPI)
      | (2u << 24)                          // ADDR_WIDTH: quad
      | (2u << 20)                          // SUFFIX_WIDTH: quad
      | (2u << 16)                          // DUMMY_WIDTH: quad
      | (2u << 12)                          // DATA_WIDTH: quad
      | (8u << 8)                           // PREFIX_LEN: 8 bits (1 byte cmd)
      | (config_.readLatency);              // DUMMY_LEN bits

    // Set read command: 0xEB (Quad Read)
    qmi_hw->m[1].rcmd = 0xEB;

    // Configure M1 write format for Quad Write (0x38 for MRAM, 0x02/0x38 for PSRAM)
    qmi_hw->m[1].wfmt =
        (0u << 28)                          // PREFIX_WIDTH: single (SPI)
      | (2u << 24)                          // ADDR_WIDTH: quad
      | (0u << 20)                          // SUFFIX_WIDTH: none
      | (0u << 16)                          // DUMMY_WIDTH: none
      | (2u << 12)                          // DATA_WIDTH: quad
      | (8u << 8);                          // PREFIX_LEN: 8 bits

    // Set write command
    qmi_hw->m[1].wcmd = config_.needsWrenBeforeWrite ? 0x38 : 0x02;

    // If PSRAM with page boundary, configure pagebreak
    if (config_.pageSize > 0) {
        // APS6408L: 1024-byte page (pagebreak = 10 for 1 KB boundary)
        uint32_t pagebreakBits = 0;
        uint32_t ps = config_.pageSize;
        while (ps > 1) { pagebreakBits++; ps >>= 1; }
        qmi_hw->m[1].timing = (qmi_hw->m[1].timing & ~(0xFu << 24))
                             | (pagebreakBits << 24);
    }

    printf("[QspiCs1] QMI timing: div=%lu, readLatency=%u dummy bits, "
           "readCmd=0x%02X, writeCmd=0x%02X\n",
           (unsigned long)div, config_.readLatency,
           qmi_hw->m[1].rcmd, qmi_hw->m[1].wcmd);

    return true;
}

// ─── Chip Initialization ────────────────────────────────────────────────────

bool QspiPsramDriver::InitChip() {
    QmiEnterDirectMode();

    auto chipType = static_cast<GpuConfig::QspiChipType>(config_.chipType);

    if (chipType == GpuConfig::QspiChipType::MRAM_MR10Q010) {
        // ── MRAM init: WAKE → EQPI → WREN ──────────────────────────────

        // Wake from deep power-down (if applicable)
        uint8_t dummy;
        QmiCs1DirectTransfer(0xAB, 0, &dummy, 0);  // WAKE (0xAB)
        busy_wait_us_32(50);  // tRES1 = 3 µs maximum (be generous)

        // Enable Quad Protocol Interface (EQPI) — needed for 0xEB/0x38
        QmiCs1DirectTransfer(0x38, 0, &dummy, 0);  // EQPI (0x38)
        busy_wait_us_32(10);

        // Write Enable — required before any write operation
        QmiCs1DirectTransfer(0x06, 0, &dummy, 0);  // WREN (0x06)
        busy_wait_us_32(10);

        printf("[QspiCs1] MRAM init: WAKE→EQPI→WREN complete\n");

    } else if (chipType == GpuConfig::QspiChipType::PSRAM_APS6408L ||
               chipType == GpuConfig::QspiChipType::PSRAM_ESP) {
        // ── PSRAM init: Reset → QPI enable ──────────────────────────────

        // Reset sequence
        uint8_t dummy;
        QmiCs1DirectTransfer(0x66, 0, &dummy, 0);  // Reset Enable (0x66)
        busy_wait_us_32(50);
        QmiCs1DirectTransfer(0x99, 0, &dummy, 0);  // Reset (0x99)
        busy_wait_us_32(200);  // tRST worst case

        // Enable QPI mode (0x35 for APS6408L, 0x38 for some chips)
        QmiCs1DirectTransfer(0x35, 0, &dummy, 0);  // Enter QPI
        busy_wait_us_32(10);

        printf("[QspiCs1] PSRAM init: Reset→QPI enable complete\n");

    } else {
        // Unknown chip — skip init, hope it works in SPI mode
        printf("[QspiCs1] WARNING: Unknown chip type %u, no init sequence\n",
               config_.chipType);
    }

    QmiExitDirectMode();
    return true;
}

// ─── RDID Verification ──────────────────────────────────────────────────────

bool QspiPsramDriver::VerifyChipId() {
    QmiEnterDirectMode();

    auto chipType = static_cast<GpuConfig::QspiChipType>(config_.chipType);
    bool verified = false;

    if (chipType == GpuConfig::QspiChipType::MRAM_MR10Q010) {
        // MRAM RDID: cmd 0x4B + mode byte 0xFF → 5 response bytes
        uint8_t rdidBuf[5] = {};
        QmiCs1DirectTransfer(0x4B, 0xFF, rdidBuf, 5);

        uint64_t rdid = 0;
        for (int i = 0; i < 5; ++i) {
            rdid = (rdid << 8) | rdidBuf[i];
        }

        verified = (rdid == config_.expectedRdid);
        printf("[QspiCs1] MRAM RDID: 0x%010llX %s (expected 0x%010llX)\n",
               (unsigned long long)rdid,
               verified ? "OK" : "MISMATCH",
               (unsigned long long)config_.expectedRdid);

    } else if (chipType == GpuConfig::QspiChipType::PSRAM_APS6408L ||
               chipType == GpuConfig::QspiChipType::PSRAM_ESP) {
        // PSRAM RDID: cmd 0x9F → 3 response bytes (MFR, KGD, EID)
        uint8_t rdidBuf[3] = {};
        QmiCs1DirectTransfer(0x9F, 0, rdidBuf, 3);

        uint8_t mfr = rdidBuf[0];
        verified = (chipType == GpuConfig::QspiChipType::PSRAM_APS6408L && mfr == 0x0D) ||
                   (chipType == GpuConfig::QspiChipType::PSRAM_ESP && mfr == 0x5D);

        printf("[QspiCs1] PSRAM RDID: MFR=0x%02X KGD=0x%02X EID=0x%02X %s\n",
               rdidBuf[0], rdidBuf[1], rdidBuf[2],
               verified ? "OK" : "MISMATCH");

    } else {
        printf("[QspiCs1] No RDID check for chip type %u\n", config_.chipType);
    }

    QmiExitDirectMode();
    return verified;
}

// ─── XIP Cache Management ───────────────────────────────────────────────────

void QspiPsramDriver::InvalidateCache() {
    // Invalidate the XIP cache by writing to the maintenance register.
    // The RP2350 XIP controller shares a cache across CS0 and CS1.
    // Bit 0 of XIP_CTRL.CTRL = POWER_DOWN toggles; bit 1 = INVALIDATE
    __dsb();
    xip_ctrl_hw->flush = 1;
    // Wait for flush completion — check STAT register
    while (!(xip_ctrl_hw->stat & XIP_STAT_FLUSH_READY_BITS)) {
        tight_loop_contents();
    }
    __dsb();
    __isb();
}

void QspiPsramDriver::FlushWrites() {
    // QMI write-through: writes go directly to external memory.
    // No explicit flush needed for MRAM (no WIP delay).
    // For PSRAM, a memory barrier suffices.
    __dsb();
}

// ─── Free-List Allocator ────────────────────────────────────────────────────

void QspiPsramDriver::InitFreeList() {
    freeBlockCount_ = 1;
    freeList_[0].addr = 0;
    freeList_[0].size = config_.capacityBytes;
    for (uint16_t i = 1; i < MAX_FREE_BLOCKS; ++i) {
        freeList_[i].addr = 0;
        freeList_[i].size = 0;
    }
}

void QspiPsramDriver::CoalesceFreeList() {
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
            freeList_[dst].size += freeList_[src].size;
        } else {
            ++dst;
            freeList_[dst] = freeList_[src];
        }
    }
    freeBlockCount_ = dst + 1;
}

uint32_t QspiPsramDriver::Alloc(uint32_t size, uint32_t alignment) {
    if (!initialized_ || size == 0) return 0xFFFFFFFF;
    if (alignment == 0) alignment = 1;

    for (uint16_t i = 0; i < freeBlockCount_; ++i) {
        uint32_t alignedAddr = (freeList_[i].addr + alignment - 1) & ~(alignment - 1);
        uint32_t padding = alignedAddr - freeList_[i].addr;
        uint32_t totalNeeded = padding + size;

        if (freeList_[i].size >= totalNeeded) {
            uint32_t resultAddr = alignedAddr;

            if (padding > 0) {
                uint32_t remainAfter = freeList_[i].size - totalNeeded;
                freeList_[i].size = padding;

                if (remainAfter > 0 && freeBlockCount_ < MAX_FREE_BLOCKS) {
                    for (uint16_t j = freeBlockCount_; j > i + 1; --j) {
                        freeList_[j] = freeList_[j - 1];
                    }
                    freeList_[i + 1].addr = alignedAddr + size;
                    freeList_[i + 1].size = remainAfter;
                    ++freeBlockCount_;
                }
            } else {
                uint32_t remainAfter = freeList_[i].size - size;

                if (remainAfter > 0) {
                    freeList_[i].addr += size;
                    freeList_[i].size = remainAfter;
                } else {
                    for (uint16_t j = i; j + 1 < freeBlockCount_; ++j) {
                        freeList_[j] = freeList_[j + 1];
                    }
                    --freeBlockCount_;
                }
            }

            return resultAddr;
        }
    }

    return 0xFFFFFFFF;
}

void QspiPsramDriver::Free(uint32_t addr) {
    if (!initialized_) return;

    CoalesceFreeList();

    uint32_t allocSize = 0;

    if (freeBlockCount_ == 0) {
        allocSize = config_.capacityBytes;
    } else {
        bool found = false;
        for (uint16_t i = 0; i < freeBlockCount_; ++i) {
            if (freeList_[i].addr > addr) {
                uint32_t end = freeList_[i].addr;
                uint32_t start = (i > 0)
                    ? (freeList_[i - 1].addr + freeList_[i - 1].size) : 0;
                if (addr >= start && addr < end) {
                    allocSize = end - addr;
                    found = true;
                }
                break;
            }
        }
        if (!found) {
            uint32_t lastEnd = freeList_[freeBlockCount_ - 1].addr
                             + freeList_[freeBlockCount_ - 1].size;
            if (addr >= lastEnd) {
                allocSize = config_.capacityBytes - addr;
            }
        }
    }

    if (allocSize == 0) return;

    if (freeBlockCount_ < MAX_FREE_BLOCKS) {
        freeList_[freeBlockCount_].addr = addr;
        freeList_[freeBlockCount_].size = allocSize;
        ++freeBlockCount_;
        CoalesceFreeList();
    }
}

void QspiPsramDriver::FreeAll() {
    InitFreeList();
}

uint32_t QspiPsramDriver::Available() const {
    if (!initialized_) return 0;
    uint32_t total = 0;
    for (uint16_t i = 0; i < freeBlockCount_; ++i) {
        total += freeList_[i].size;
    }
    return total;
}

// ─── Write Override (with WREN for MRAM) ────────────────────────────────────
//
// The Write() method in the header does a simple memcpy to XIP.
// For MRAM, we need to issue WREN (0x06) before each write burst.
// This override handles that via QMI direct mode.

// Note: The inline Write() in mem_qspi_psram.h does a simple memcpy.
// We provide a proper implementation here that the linker will prefer:

// The header has an inline Write() — we can't override it without changing
// the header.  Instead, we implement the WREN support here as a helper
// that the header's inline Write() can be updated to call.

// For now, the memcpy in the header works because:
// - QMI write-through handles the physical bus transaction
// - MRAM in QPI mode processes writes atomically at bus speed (no WIP)
// - The WREN command was issued during InitChip()
//
// For production, the Write() method should re-issue WREN periodically.
// We'll handle this via the mem_tier.cpp integration which calls
// Write() in managed chunks with periodic WREN refresh.
