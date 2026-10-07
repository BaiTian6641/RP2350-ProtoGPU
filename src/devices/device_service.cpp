/**
 * @file device_service.cpp
 * @brief Bounded allowlisted attached-device service implementation (P08).
 *
 * Validation, queueing, deadline, cancel and reply logic is portable and
 * unit-tested natively. Target glue (InitTarget/ShutdownTarget/TargetHal)
 * uses the pinned SDK I2C timeout APIs, ADC and GPIO directly and is
 * compiled only for PICO_ON_DEVICE.
 */

#include "device_service.h"

#include <cstring>

#if defined(PICO_ON_DEVICE)
#include "pico/stdlib.h"
#include "hardware/i2c.h"
#include "hardware/adc.h"
#include "hardware/gpio.h"
#include "hardware/clocks.h"
#include "hardware_resources.h"
#endif

namespace gpudev {

// ─── Wire decode (pure logic) ───────────────────────────────────────────────

PglRuntime::Result DecodeDeviceArgs(const uint32_t args[4], DeviceRequest& out) {
    using Result = PglRuntime::Result;
    if (!args) return Result::BadPacket;

    DeviceRequest req;
    req.deviceId  = static_cast<uint8_t>(args[0] & 0xFFu);
    const uint8_t op = static_cast<uint8_t>((args[0] >> 8) & 0xFFu);
    req.txLen     = static_cast<uint8_t>((args[0] >> 16) & 0xFFu);
    req.rxLen     = static_cast<uint8_t>((args[0] >> 24) & 0xFFu);

    if (req.deviceId >= kDeviceCount) return Result::InvalidValue;
    if (op < static_cast<uint8_t>(PglRuntime::DeviceOperation::Read) ||
        op > static_cast<uint8_t>(PglRuntime::DeviceOperation::ReadWrite)) {
        return Result::InvalidValue;
    }
    req.operation = static_cast<PglRuntime::DeviceOperation>(op);
    if (req.txLen > kMaxTxBytes || req.rxLen > kMaxRxBytes) return Result::InvalidValue;

    // args[1..3] little-endian bytes, exactly as stored by Store32 on encode.
    for (uint8_t i = 0; i < req.txLen; ++i) {
        req.tx[i] = static_cast<uint8_t>((args[1 + i / 4] >> ((i % 4) * 8)) & 0xFFu);
    }
    out = req;
    return Result::Ok;
}

// ─── Service ────────────────────────────────────────────────────────────────

DeviceService::DeviceService() = default;

bool DeviceService::AttachHal(const DeviceHal& hal) {
    if (!IsIdle()) return false;  // never swap hardware access mid-request
    hal_ = hal;
    reqCount_ = 0; reqHead_ = 0;
    replyCount_ = 0; replyHead_ = 0;
    nextId_ = 1;
    return true;
}

PglRuntime::Result DeviceService::Validate(const DeviceRequest& req) const {
    using Result = PglRuntime::Result;
    using Op = PglRuntime::DeviceOperation;
    if (req.deviceId >= kDeviceCount) return Result::InvalidValue;
    if (req.txLen > kMaxTxBytes || req.rxLen > kMaxRxBytes) return Result::InvalidValue;

    switch (req.deviceId) {
        case kDeviceTempAdc:
            // Raw-count read only; there is no writable sensor state.
            if (req.operation != Op::Read || req.txLen != 0 ||
                req.rxLen != 2) return Result::InvalidValue;
            return Result::Ok;
        case kDeviceAuxI2c:
            switch (req.operation) {
                case Op::Read:
                    if (req.txLen != 0 || req.rxLen < 1) return Result::InvalidValue;
                    return Result::Ok;
                case Op::Write:
                    if (req.txLen < 1 || req.rxLen != 0) return Result::InvalidValue;
                    return Result::Ok;
                case Op::ReadWrite:
                    if (req.txLen < 1 || req.rxLen < 1) return Result::InvalidValue;
                    return Result::Ok;
            }
            return Result::InvalidValue;
        case kDeviceGpio:
            switch (req.operation) {
                case Op::Read:
                    if (req.txLen != 0 || req.rxLen != 1) return Result::InvalidValue;
                    return Result::Ok;
                case Op::Write:
                    if (req.txLen != 1 || req.rxLen != 0 || req.tx[0] > 1) {
                        return Result::InvalidValue;  // explicit 0/1 drive only
                    }
                    return Result::Ok;
                case Op::ReadWrite:
                    if (req.txLen != 1 || req.rxLen != 1 || req.tx[0] > 1) {
                        return Result::InvalidValue;
                    }
                    return Result::Ok;
            }
            return Result::InvalidValue;
    }
    return Result::InvalidValue;
}

void DeviceService::PushReply(const PglRuntime::DeviceReply& reply,
                              PglRuntime::Result result) {
    if (replyCount_ >= kReplyDepth) return;  // Submit/Poll backpressure prevents this
    const uint8_t idx = static_cast<uint8_t>((replyHead_ + replyCount_) % kReplyDepth);
    replies_[idx] = reply;
    replyResults_[idx] = result;
    ++replyCount_;
}

PglRuntime::Result DeviceService::Submit(const DeviceRequest& req, uint32_t& outRequestId) {
    using Result = PglRuntime::Result;
    outRequestId = 0;
    if (!HasHal()) return Result::BadState;
    if (reqCount_ >= kQueueDepth || replyCount_ >= kReplyDepth) return Result::Busy;

    const uint32_t id = nextId_++;
    const uint64_t now = hal_.timeUs(hal_.ctx);

    const Result valid = Validate(req);
    if (valid != Result::Ok) {
        // Explicit error reply — no silent fallback for unknown/bad requests.
        PglRuntime::DeviceReply reply;
        reply.requestId = id;
        reply.timestampUs = now;
        reply.deviceId = req.deviceId;
        reply.operation = req.operation;
        reply.length = 0;
        PushReply(reply, valid);
        outRequestId = id;
        return valid;
    }

    Slot& slot = queue_[(reqHead_ + reqCount_) % kQueueDepth];
    slot.req = req;
    slot.id = id;
    slot.deadlineUs = now + timeoutUs_;
    ++reqCount_;
    outRequestId = id;
    return Result::Ok;
}

PglRuntime::Result DeviceService::Cancel(uint32_t requestId) {
    using Result = PglRuntime::Result;
    if (!HasHal()) return Result::BadState;
    if (replyCount_ >= kReplyDepth) return Result::Busy;
    for (uint8_t i = 0; i < reqCount_; ++i) {
        const uint8_t idx = static_cast<uint8_t>((reqHead_ + i) % kQueueDepth);
        if (queue_[idx].id == requestId) {
            PglRuntime::DeviceReply reply;
            reply.requestId = requestId;
            reply.timestampUs = hal_.timeUs(hal_.ctx);
            reply.deviceId = queue_[idx].req.deviceId;
            reply.operation = queue_[idx].req.operation;
            reply.length = 0;
            // Compact the ring (bounded, kQueueDepth entries).
            for (uint8_t j = i; j + 1 < reqCount_; ++j) {
                const uint8_t a = static_cast<uint8_t>((reqHead_ + j) % kQueueDepth);
                const uint8_t b = static_cast<uint8_t>((reqHead_ + j + 1) % kQueueDepth);
                queue_[a] = queue_[b];
            }
            --reqCount_;
            PushReply(reply, Result::Cancelled);
            return Result::Ok;
        }
    }
    return Result::InvalidHandle;  // unknown or already completed
}
void DeviceService::Execute(const DeviceRequest& req, uint64_t deadlineUs,
                            PglRuntime::DeviceReply& reply,
                            PglRuntime::Result& result) {
    using Result = PglRuntime::Result;
    using Op = PglRuntime::DeviceOperation;
    reply.length = 0;
    result = Result::Ok;

    switch (req.deviceId) {
        case kDeviceTempAdc: {
            uint16_t count = 0;
            if (!hal_.readTempRaw || !hal_.readTempRaw(hal_.ctx, count)) {
                result = Result::Io;
                return;
            }
            reply.data[0] = static_cast<uint8_t>(count & 0xFFu);
            reply.data[1] = static_cast<uint8_t>(count >> 8);
            reply.length = 2;
            return;
        }
        case kDeviceGpio: {
            if (req.operation == Op::Write || req.operation == Op::ReadWrite) {
                if (!hal_.gpioWrite || !hal_.gpioWrite(hal_.ctx, req.tx[0])) {
                    result = Result::Io;
                    return;
                }
            }
            if (req.operation == Op::Read || req.operation == Op::ReadWrite) {
                uint8_t value = 0;
                if (!hal_.gpioRead || !hal_.gpioRead(hal_.ctx, value)) {
                    result = Result::Io;
                    return;
                }
                reply.data[0] = value;
                reply.length = 1;
            }
            return;
        }
        case kDeviceAuxI2c: {
            // clamped to kI2cSliceUs — the declared worst service slice.
            uint64_t now = hal_.timeUs(hal_.ctx);
            uint32_t remaining = (deadlineUs > now)
                ? static_cast<uint32_t>(deadlineUs - now) : 0u;
            if (remaining == 0) { result = Result::Timeout; return; }
            if (remaining > kI2cSliceUs) remaining = kI2cSliceUs;

            if (req.operation == Op::Write || req.operation == Op::ReadWrite) {
                if (!hal_.i2cWrite) { result = Result::BadState; return; }
                const int wrote = hal_.i2cWrite(hal_.ctx, req.tx, req.txLen,
                                                req.operation == Op::ReadWrite,
                                                remaining);
                if (wrote == kHalTimeout) { result = Result::Timeout; return; }
                if (wrote != req.txLen) { result = Result::Io; return; }
            }
            if (req.operation == Op::Read || req.operation == Op::ReadWrite) {
                if (!hal_.i2cRead) { result = Result::BadState; return; }
                now = hal_.timeUs(hal_.ctx);
                remaining = (deadlineUs > now)
                    ? static_cast<uint32_t>(deadlineUs - now) : 0u;
                if (remaining == 0) { result = Result::Timeout; return; }
                if (remaining > kI2cSliceUs) remaining = kI2cSliceUs;
                const int read = hal_.i2cRead(hal_.ctx, reply.data, req.rxLen,
                                              remaining);
                if (read == kHalTimeout) { result = Result::Timeout; return; }
                if (read != req.rxLen) { result = Result::Io; return; }
                reply.length = req.rxLen;
            }
            return;
        }
    }
    result = Result::InvalidValue;  // unreachable: Submit validated
}

void DeviceService::Poll() {
    using Result = PglRuntime::Result;
    if (!HasHal() || reqCount_ == 0) return;
    if (replyCount_ >= kReplyDepth) return;  // host must drain replies first

    Slot& slot = queue_[reqHead_];
    const uint64_t now = hal_.timeUs(hal_.ctx);

    PglRuntime::DeviceReply reply;
    reply.requestId = slot.id;
    reply.deviceId = slot.req.deviceId;
    reply.operation = slot.req.operation;
    reply.length = 0;

    Result result = Result::Ok;
    if (now >= slot.deadlineUs) {
        // Expired while queued: resolve finitely WITHOUT touching hardware.
        result = Result::Timeout;
        reply.timestampUs = now;
    } else {
        Execute(slot.req, slot.deadlineUs, reply, result);
        reply.timestampUs = hal_.timeUs(hal_.ctx);
    }

    PushReply(reply, result);
    reqHead_ = static_cast<uint8_t>((reqHead_ + 1) % kQueueDepth);
    --reqCount_;
}

bool DeviceService::PopReply(PglRuntime::DeviceReply& outReply,
                             PglRuntime::Result& outResult) {
    if (replyCount_ == 0) return false;
    outReply = replies_[replyHead_];
    outResult = replyResults_[replyHead_];
    replyHead_ = static_cast<uint8_t>((replyHead_ + 1) % kReplyDepth);
    --replyCount_;
    return true;
}

bool DeviceService::RetimeForClock(uint32_t newSysHz) {
    if (!IsIdle()) return false;  // caller violated the quiescent contract
    if (!hal_.retime) return true;  // nothing clk_sys-derived attached
    return hal_.retime(hal_.ctx, newSysHz);
}

// ─── Target glue ────────────────────────────────────────────────────────────

#if defined(PICO_ON_DEVICE)
namespace {

bool HalReadTempRaw(void*, uint16_t& outCount) {
    // On-chip temperature sensor is ADC input 4. 12-bit raw count; the
    // service deliberately reports the count, not a calibrated temperature.
    adc_select_input(4);
    outCount = adc_read();
    return true;
}

i2c_inst_t* AuxI2c() { return i2c_get_instance(GpuConfig::DEVICE_I2C_INSTANCE); }

int HalI2cWrite(void*, const uint8_t* tx, uint8_t len, bool nostop, uint32_t budgetUs) {
    const int r = i2c_write_timeout_us(AuxI2c(), kAuxI2cAddress, tx, len, nostop,
                                       budgetUs);
    if (r == PICO_ERROR_TIMEOUT) return kHalTimeout;
    if (r < 0) return kHalIoError;
    return r;
}

int HalI2cRead(void*, uint8_t* rx, uint8_t len, uint32_t budgetUs) {
    const int r = i2c_read_timeout_us(AuxI2c(), kAuxI2cAddress, rx, len, false,
                                      budgetUs);
    if (r == PICO_ERROR_TIMEOUT) return kHalTimeout;
    if (r < 0) return kHalIoError;
    return r;
}

bool HalGpioWrite(void*, uint8_t value) {
    // Drive only on explicit write: set the level first, then switch the pad
    // to output so no transient opposite level is emitted.
    gpio_put(kGpioPin, value);
    gpio_set_dir(kGpioPin, GPIO_OUT);
    return true;
}

bool HalGpioRead(void*, uint8_t& outValue) {
    outValue = gpio_get(kGpioPin) ? 1u : 0u;
    return true;
}
bool HalRetime(void*, uint32_t newSysHz) {
    if (!newSysHz || clock_get_hz(clk_sys) != newSysHz) return false;
    // I2C baud on RP2350 derives from clk_sys; re-init at the same 400 kHz
    // target recomputes the divisor for the new clock. Quiescent: the
    // service guarantees no queued/in-flight request here.
    const uint32_t actualBaud = i2c_init(AuxI2c(), GpuConfig::DEVICE_I2C_BAUD);
    return actualBaud && actualBaud <= GpuConfig::DEVICE_I2C_BAUD;
}

uint64_t HalTimeUs(void*) { return time_us_64(); }

constexpr uint64_t kI2cPinMask =
    (uint64_t(1) << GpuConfig::DEVICE_I2C_SDA) | (uint64_t(1) << GpuConfig::DEVICE_I2C_SCL);
constexpr uint64_t kGpioPinMask = uint64_t(1) << kGpioPin;

void ReleaseAllLeases(HardwareResources& resources) {
    resources.ReleaseGpios(kI2cPinMask | kGpioPinMask, HardwareResources::Owner::Devices);
    resources.ReleaseBus(kBusI2c0, HardwareResources::Owner::Devices);
    resources.ReleaseBus(kBusAdc, HardwareResources::Owner::Devices);
}

}  // namespace

DeviceHal TargetHal() {
    DeviceHal hal;
    hal.readTempRaw = &HalReadTempRaw;
    hal.i2cWrite    = &HalI2cWrite;
    hal.i2cRead     = &HalI2cRead;
    hal.gpioWrite   = &HalGpioWrite;
    hal.gpioRead    = &HalGpioRead;
    hal.retime      = &HalRetime;
    hal.timeUs      = &HalTimeUs;
    return hal;
}

PglRuntime::Result InitTarget(DeviceService& service, HardwareResources& resources) {
    using Result = PglRuntime::Result;
    using Owner = HardwareResources::Owner;
    if (service.HasHal()) return Result::BadState;

    // Claims first; any conflict rolls back every partial claim before a
    // single pad is touched. GP23/24/25/29 are outside the registry's
    // allowed mask, so the allowlisted pins (20/21/26) can never alias them.
    Result r = resources.ClaimGpios(kI2cPinMask | kGpioPinMask, Owner::Devices);
    if (r != Result::Ok) return r;
    r = resources.ClaimBus(kBusI2c0, Owner::Devices);
    if (r != Result::Ok) {
        resources.ReleaseGpios(kI2cPinMask | kGpioPinMask, Owner::Devices);
        return r;
    }
    r = resources.ClaimBus(kBusAdc, Owner::Devices);
    if (r != Result::Ok) {
        resources.ReleaseBus(kBusI2c0, Owner::Devices);
        resources.ReleaseGpios(kI2cPinMask | kGpioPinMask, Owner::Devices);
        return r;
    }

    // Hardware setup (all leases held).
    i2c_init(AuxI2c(), GpuConfig::DEVICE_I2C_BAUD);
    gpio_set_function(GpuConfig::DEVICE_I2C_SDA, GPIO_FUNC_I2C);
    gpio_set_function(GpuConfig::DEVICE_I2C_SCL, GPIO_FUNC_I2C);
    gpio_pull_up(GpuConfig::DEVICE_I2C_SDA);
    gpio_pull_up(GpuConfig::DEVICE_I2C_SCL);

    // GP26 safe default: input, no pulls, never driven until an explicit
    // host Write.
    gpio_init(kGpioPin);
    gpio_disable_pulls(kGpioPin);
    gpio_set_dir(kGpioPin, GPIO_IN);

    adc_init();
    adc_set_temp_sensor_enabled(true);

    if (!service.AttachHal(TargetHal())) {
        ReleaseAllLeases(resources);
        return Result::BadState;
    }
    return Result::Ok;
}

void ShutdownTarget(DeviceService& service, HardwareResources& resources) {
    // Safe GPIO defaults: GP26 and the I2C pads back to plain inputs with no
    // pulls; temp sensor off; I2C peripheral de-initialised.
    adc_set_temp_sensor_enabled(false);
    gpio_set_dir(kGpioPin, GPIO_IN);
    gpio_disable_pulls(kGpioPin);
    i2c_deinit(AuxI2c());
    gpio_set_function(GpuConfig::DEVICE_I2C_SDA, GPIO_FUNC_NULL);
    gpio_set_function(GpuConfig::DEVICE_I2C_SCL, GPIO_FUNC_NULL);
    gpio_disable_pulls(GpuConfig::DEVICE_I2C_SDA);
    gpio_disable_pulls(GpuConfig::DEVICE_I2C_SCL);

    ReleaseAllLeases(resources);

    // Caller drained the service at the maintenance point; detach clears the
    // HAL so re-init goes through InitTarget() again.
    service.AttachHal(DeviceHal{});
}

#endif  // PICO_ON_DEVICE

}  // namespace gpudev
