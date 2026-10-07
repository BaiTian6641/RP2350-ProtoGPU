/**
 * @file device_service.h
 * @brief Bounded, allowlisted attached-device service for the RP2350 GPU (P08).
 *
 * Exactly three core-0 services exist; nothing else is reachable from the
 * host wire protocol (no arbitrary address/register/pin escape):
 *
 *   device 0 — on-chip temperature ADC. Read returns the RAW 12-bit ADC
 *              count (2 bytes LE) plus the sample timestamp. This is NOT a
 *              calibrated temperature; conversion/calibration is the host's
 *              (or the thermal policy's configured) business.
 *   device 1 — aux I2C0 master, SDA=GP20 / SCL=GP21, FIXED slave address
 *              0x3D at 400 kHz. Read / Write / ReadWrite (repeated start).
 *              The slave address is compiled in — the host cannot select
 *              another. No implicit master/slave role switch exists; the
 *              old shared I2C slave is not part of this runtime.
 *   device 2 — GP26 digital line. Read (input level 0/1), Write (drive 0/1;
 *              the pin is driven ONLY by an explicit write), ReadWrite
 *              (drive then read back).
 *
 * Wire contract (parent decodes Command::Device with DecodeDeviceArgs):
 *   args[0] bytes {deviceId, operation, txLength, rxLength}
 *   args[1..3]   up to 12 TX bytes
 * Replies are PglRuntime::DeviceReply records (requestId, timestampUs,
 * deviceId, operation, length, up to 16 RX bytes) plus a Result.
 *
 * Bounds and ownership:
 *  - One bounded request queue + one bounded reply queue (kQueueDepth each).
 *    Submit on a full queue → Busy. Cancel removes a queued (never-started)
 *    request and emits a Cancelled reply.
 *  - Every request carries a submit-time deadline (now + timeoutUs). An
 *    expired queued request resolves as Timeout WITHOUT touching hardware.
 *  - Poll() executes at most ONE request per call, and every I2C phase is
 *    bounded by the SDK timeout API clamped to the remaining budget, capped
 *    at kI2cSliceUs (1 ms) — the declared worst service slice. Requests can
 *    never starve render/host beyond that.
 *  - All hardware access happens from Poll() on core 0. No ISR touches
 *    device hardware. Empty/invalid requests produce explicit error replies
 *    (InvalidValue); there is no silent fallback for an unknown sensor.
 *
 * Clock retime: I2C baud on RP2350 derives from clk_sys, so after a clock
 * profile change the parent calls RetimeForClock() (via the GpuClock
 * ApplyHooks.retimeClients hook) at the same quiescent point — it refuses
 * while any request is queued/in flight. The ADC runs from the fixed 48 MHz
 * clk_adc and needs no retime; GP26 is untimed.
 *
 * Target glue (InitTarget/ShutdownTarget, PICO_ON_DEVICE only) performs real
 * HardwareResources claims (Owner::Devices): GP20/21/26 + I2C0 bus + ADC
 * resource, with full rollback on any conflict, safe GPIO defaults on
 * shutdown, and re-init support. Pico-2-reserved GP23/24/25/29 can never be
 * claimed here: they are outside the registry's allowed mask and none of the
 * allowlisted services uses them.
 *
 * Native tests exercise the real validation/queue/deadline/cancel logic with
 * a HAL that models hardware absence/failure — payload success paths on real
 * silicon are HIL acceptance, not mocked echoes.
 */

#pragma once

#include <cstdint>
#include <PglRuntimeProtocol.h>

#include "gpu_config.h"

class HardwareResources;

namespace gpudev {

// ─── Allowlist ──────────────────────────────────────────────────────────────

constexpr uint8_t kDeviceTempAdc = 0;
constexpr uint8_t kDeviceAuxI2c  = 1;
constexpr uint8_t kDeviceGpio    = 2;
constexpr uint8_t kDeviceCount   = 3;

constexpr uint8_t  kMaxTxBytes   = 12;   // args[1..3]
constexpr uint8_t  kMaxRxBytes   = sizeof(PglRuntime::DeviceReply::data);  // 16
constexpr uint8_t  kQueueDepth   = 4;    // bounded pending requests
constexpr uint8_t  kReplyDepth   = 4;    // bounded completed replies
constexpr uint32_t kI2cSliceUs   = GpuConfig::DEVICE_TIMEOUT_US;  // 1 ms declared slice

constexpr uint8_t  kAuxI2cAddress = GpuConfig::DEVICE_I2C_ADDRESS;  // 0x3D fixed
constexpr uint8_t  kGpioPin       = GpuConfig::DEVICE_GPIO_PIN;     // GP26

/// HardwareResources bus indices (matches its "I2C0/1, SPI0/1, UART0/1,
/// ADC, PWM" registry layout).
constexpr uint8_t kBusI2c0 = 0;
constexpr uint8_t kBusAdc  = 6;

// ─── Request ────────────────────────────────────────────────────────────────

struct DeviceRequest {
    uint8_t deviceId = 0;
    PglRuntime::DeviceOperation operation = PglRuntime::DeviceOperation::Read;
    uint8_t txLen = 0;                 ///< 0..kMaxTxBytes
    uint8_t rxLen = 0;                 ///< 0..kMaxRxBytes
    uint8_t tx[kMaxTxBytes] = {};
};

/// Pure decode of control args into a request. Checks device id, operation
/// enum and TX/RX count fields only — per-device semantic validation happens
/// in the service. BadPacket on malformed args, InvalidValue on unknown
/// device/operation or out-of-range counts.
PglRuntime::Result DecodeDeviceArgs(const uint32_t args[4], DeviceRequest& out);

// ─── Hardware abstraction (target SDK calls vs native logic tests) ──────────
// Every entry point must be bounded: I2C calls receive a microsecond budget
// and must use the SDK timeout APIs; none may sleep or wait unboundedly.
struct DeviceHal {
    void* ctx = nullptr;
    /// device 0: raw 12-bit ADC count of the on-chip temperature sensor.
    bool (*readTempRaw)(void* ctx, uint16_t& outCount) = nullptr;
    /// device 1: fixed-address I2C. Return bytes transferred, or
    /// kHalTimeout / kHalIoError. `nostop` keeps the bus for a repeated start.
    int (*i2cWrite)(void* ctx, const uint8_t* tx, uint8_t len, bool nostop,
                    uint32_t budgetUs) = nullptr;
    int (*i2cRead)(void* ctx, uint8_t* rx, uint8_t len, uint32_t budgetUs) = nullptr;
    /// device 2: drive explicit 0/1 (switches the pad to output), read level.
    bool (*gpioWrite)(void* ctx, uint8_t value) = nullptr;
    bool (*gpioRead)(void* ctx, uint8_t& outValue) = nullptr;
    /// Re-derive I2C timing for a new clk_sys (quiescent; service guarantees
    /// no queued/in-flight request when this is called).
    bool (*retime)(void* ctx, uint32_t newSysHz) = nullptr;
    /// Wall clock (us) for deadlines/timestamps.
    uint64_t (*timeUs)(void* ctx) = nullptr;
};

constexpr int kHalTimeout = -1;
constexpr int kHalIoError = -2;

// ─── Service ────────────────────────────────────────────────────────────────

class DeviceService {
public:
    DeviceService();

    /// Install the HAL and reset all queues/state. No hardware is touched
    /// here; target hardware setup lives in InitTarget(). Re-attach is legal
    /// only when idle (returns false otherwise).
    bool AttachHal(const DeviceHal& hal);
    bool HasHal() const { return hal_.timeUs != nullptr; }

    /// Host entry: validate + enqueue. Always produces a reply (even for
    /// invalid requests, result=InvalidValue) unless the queue is full.
    /// Ok (queued) / Busy (queue full) / BadState (no HAL attached).
    PglRuntime::Result Submit(const DeviceRequest& req, uint32_t& outRequestId);

    /// Cancel a QUEUED (never-started) request; emits a Cancelled reply.
    /// InvalidHandle when the id is unknown or already completed.
    PglRuntime::Result Cancel(uint32_t requestId);
    /// Core-0 service point. Executes at most one queued request, bounded by
    /// kI2cSliceUs of hardware time; expired requests resolve as Timeout
    /// without touching hardware. Never called from an ISR.
    void Poll();

    /// Drain one completed reply (oldest first). False when empty.
    bool PopReply(PglRuntime::DeviceReply& outReply, PglRuntime::Result& outResult);

    /// DeviceIdle gate for clock/maintenance: no queued request and no
    /// reply-blocking in-flight work.
    bool IsIdle() const { return reqCount_ == 0; }
    uint8_t PendingCount() const { return reqCount_; }
    uint8_t ReplyCount() const { return replyCount_; }

    /// Clock-change retime (parent: GpuClock ApplyHooks.retimeClients).
    /// Refuses (false) unless idle; re-derives I2C timing at the new clk_sys.
    bool RetimeForClock(uint32_t newSysHz);

private:
    PglRuntime::Result Validate(const DeviceRequest& req) const;
    void Execute(const DeviceRequest& req, uint64_t deadlineUs,
                 PglRuntime::DeviceReply& reply, PglRuntime::Result& result);
    void PushReply(const PglRuntime::DeviceReply& reply, PglRuntime::Result result);

    struct Slot {
        DeviceRequest req;
        uint32_t id = 0;
        uint64_t deadlineUs = 0;
    };

    DeviceHal hal_{};
    Slot queue_[kQueueDepth];
    uint8_t reqHead_ = 0, reqCount_ = 0;
    PglRuntime::DeviceReply replies_[kReplyDepth];
    PglRuntime::Result replyResults_[kReplyDepth];
    uint8_t replyHead_ = 0, replyCount_ = 0;
    uint32_t nextId_ = 1;
    uint32_t timeoutUs_ = kI2cSliceUs;  ///< per-request deadline budget
};

// ─── Target glue (RP2350 on-device builds only) ─────────────────────────────

#if defined(PICO_ON_DEVICE)
/// Real SDK-backed HAL (I2C0 @ 0x3D/400 kHz, ADC ch4, GP26).
DeviceHal TargetHal();

/// Claim GP20/21 (I2C0), GP26 and the I2C0/ADC buses with Owner::Devices,
/// configure hardware (I2C 400 kHz, ADC temp sensor, GP26 safe input) and
/// attach the target HAL. Any claim conflict or setup failure rolls back
/// every partial claim and leaves hardware untouched → returns the error.
/// BadState when already initialized.
PglRuntime::Result InitTarget(DeviceService& service, HardwareResources& resources);

/// Safe GPIO defaults (GP26 input, I2C pads released to inputs, temp sensor
/// off), release all Owner::Devices leases, detach the HAL. Re-init via
/// InitTarget() is supported afterwards.
void ShutdownTarget(DeviceService& service, HardwareResources& resources);
#endif

}  // namespace gpudev
