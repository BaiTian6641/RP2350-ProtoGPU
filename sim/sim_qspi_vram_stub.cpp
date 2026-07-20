/**
 * @file sim_qspi_vram_stub.cpp
 * @brief Desktop sim shims for hardware-bound classes — A5-1 skeleton.
 *
 * Implements JUST enough of the hardware-facing classes for the real firmware
 * render path to link and run on a desktop host.  No firmware sources are
 * modified; the real implementations (src/memory/mem_qspi_vram.cpp,
 * src/memory/flash_persist.cpp) are RP2350/PIO/flash-bound and stay out of the
 * sim build.
 *
 * Semantics: QSPI_VRAM_MODE = NONE (the current bench config in gpu_config.h).
 * The driver never initialises a channel, every transfer fails, every
 * allocation reports OOM — exactly what the real driver does on an RP2350A
 * without external VRAM.  Firmware fallbacks (MemTierManager SRAM-only mode,
 * parser's "pool exhausted" accounting) exercise the same paths as on
 * hardware with no VRAM fitted.
 *
 * FlashPersistManager is a pure in-memory no-op: initialisation succeeds with
 * an empty manifest, nothing is ever persisted.  The A5-1 test scene issues no
 * persistence opcodes; the stub exists because command_parser.cpp holds a
 * static FlashPersistManager instance whose methods must resolve at link time.
 */

#include "memory/mem_qspi_vram.h"
#include "memory/flash_persist.h"

#include <cstring>

// ─── QspiVramDriver — NONE-mode no-op ───────────────────────────────────────

QspiVramDriver::~QspiVramDriver() = default;

bool QspiVramDriver::InitChannel(const QspiVramChannelConfig& /*config*/) {
    return false;  // no hardware in the sim
}

void QspiVramDriver::ShutdownChannel(QspiChannel /*ch*/) {}
void QspiVramDriver::Shutdown() {}

bool QspiVramDriver::ReadSync(QspiChannel /*ch*/, uint32_t /*srcAddr*/,
                              void* /*dest*/, uint32_t /*length*/) {
    return false;
}

bool QspiVramDriver::WriteSync(QspiChannel /*ch*/, uint32_t /*destAddr*/,
                               const void* /*src*/, uint32_t /*length*/) {
    return false;
}

bool QspiVramDriver::ReadAsync(QspiChannel /*ch*/, uint32_t /*srcAddr*/,
                               void* /*dest*/, uint32_t /*length*/) {
    return false;
}

bool QspiVramDriver::WriteAsync(QspiChannel /*ch*/, uint32_t /*destAddr*/,
                                const void* /*src*/, uint32_t /*length*/) {
    return false;
}

QspiVramDmaStatus QspiVramDriver::GetDmaStatus(QspiChannel /*ch*/) const {
    return QspiVramDmaStatus::IDLE;
}

void QspiVramDriver::WaitDma(QspiChannel /*ch*/) {}

void QspiVramDriver::SetDmaCallback(QspiChannel /*ch*/,
                                    QspiVramDmaCallback /*cb*/,
                                    void* /*userCtx*/) {}

void QspiVramDriver::ClearDmaCallback(QspiChannel /*ch*/) {}

bool QspiVramDriver::Prefetch(QspiChannel /*ch*/, uint32_t /*srcAddr*/,
                              uint8_t* /*sramDest*/, uint32_t /*length*/) {
    return false;
}

uint32_t QspiVramDriver::Alloc(QspiChannel /*ch*/, uint32_t /*size*/,
                               uint32_t /*alignment*/) {
    return 0xFFFFFFFFu;  // OOM — no external VRAM present
}

void QspiVramDriver::Free(QspiChannel /*ch*/, uint32_t /*addr*/,
                          uint32_t /*size*/) {}

void QspiVramDriver::FreeAll(QspiChannel /*ch*/) {}

uint32_t QspiVramDriver::Available(QspiChannel /*ch*/) const { return 0; }

uint32_t QspiVramDriver::GetTotalReads(QspiChannel /*ch*/) const  { return 0; }
uint32_t QspiVramDriver::GetTotalWrites(QspiChannel /*ch*/) const { return 0; }
uint32_t QspiVramDriver::GetCapacity(QspiChannel /*ch*/) const    { return 0; }
uint8_t  QspiVramDriver::GetChipCount(QspiChannel /*ch*/) const   { return 0; }

bool QspiVramDriver::IsMram(QspiChannel /*ch*/) const        { return false; }
bool QspiVramDriver::HasRandomAccessPenalty(QspiChannel /*ch*/) const {
    return true;  // matches the "no chip" default in QspiVramChipConfig
}
bool QspiVramDriver::IsNonVolatile(QspiChannel /*ch*/) const { return false; }

const QspiVramChipConfig& QspiVramDriver::GetChipConfig(QspiChannel ch,
                                                        uint8_t csIndex) const {
    return channels_[static_cast<uint8_t>(ch)]
        .config.chips[csIndex % QSPI_VRAM_MAX_CS_PER_CHANNEL];
}

const QspiVramChannelConfig&
QspiVramDriver::GetChannelConfig(QspiChannel ch) const {
    return channels_[static_cast<uint8_t>(ch)].config;
}

// ─── FlashPersistManager — in-memory no-op ──────────────────────────────────

bool FlashPersistManager::Initialize(MemTierManager* tierMgr) {
    tierMgr_     = tierMgr;
    initialized_ = true;   // behaves like "no valid manifest → empty one"
    return true;
}

bool FlashPersistManager::PersistResource(uint8_t /*resourceClass*/,
                                          uint16_t /*resourceId*/,
                                          uint8_t /*flags*/,
                                          const void* /*dataPtr*/,
                                          uint32_t /*dataSize*/) {
    return false;  // no flash in the sim — nothing is ever queued
}

const void* FlashPersistManager::RestoreResource(uint8_t /*resourceClass*/,
                                                 uint16_t /*resourceId*/,
                                                 void* /*destPtr*/,
                                                 uint32_t /*destSize*/) {
    return nullptr;
}

bool FlashPersistManager::IsResourcePersisted(uint8_t /*resourceClass*/,
                                              uint16_t /*resourceId*/) const {
    return false;
}

uint8_t FlashPersistManager::ProcessWritebackQueue() { return 0; }

uint16_t FlashPersistManager::AutoRestore() { return 0; }

void FlashPersistManager::GetStatus(PglMemPersistStatusResponse& status) const {
    std::memset(&status, 0, sizeof(status));
    status.state            = currentState_;   // PGL_PERSIST_IDLE
    status.manifestCapacity = MAX_ENTRIES;
}

void FlashPersistManager::EraseAll() {}
