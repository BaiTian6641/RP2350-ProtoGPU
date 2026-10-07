// DeviceService native behavior tests (P08 device slice).
//
// Exercises the REAL validation / queue / deadline / cancel / budget logic in
// src/devices/device_service.cpp. The HAL below is a deterministic hardware
// stand-in in the same sense as the memory slice's RamBacking: a fixed
// 16-register I2C register file, a 12-bit ADC sample register and a 1-bit
// GPIO pad — it models bus semantics (NACK, timeout, repeated start), never
// echoes expected replies. Success payloads on the real 0x3D peripheral,
// pulse/current/temperature limits remain HIL acceptance, not covered here.
//
// Build and run via tests/devices/run_tests.sh (native g++, no Pico SDK).

#include "devices/device_service.h"

#include <cstdint>
#include <cstdio>
#include <cstring>

using namespace gpudev;
using PglRuntime::Result;
using PglRuntime::DeviceOperation;

namespace {

int gChecks = 0;
int gFailures = 0;

#define CHECK(cond)                                                            \
    do {                                                                       \
        ++gChecks;                                                             \
        if (!(cond)) {                                                         \
            ++gFailures;                                                       \
            std::printf("FAIL %s:%d: CHECK(%s)\n", __FILE__, __LINE__, #cond); \
        }                                                                      \
    } while (0)

// ─── Deterministic hardware stand-in ────────────────────────────────────────
// Register-file I2C slave: byte 0 of a write selects the register, following
// bytes program it; a read returns current register contents (auto-increment,
// like real register-file peripherals). Repeated-start (nostop) writes keep
// the register pointer for the following read. NACK and timeout are
// independently forceable. The ADC is a sample register; GPIO is a pad bit
// with an output-enable flag. All budgets/calls are recorded.

struct FakeHw {
    uint64_t nowUs = 0;
    // I2C register file
    uint8_t regs[16] = {};
    uint8_t regPtr = 0;
    bool    nack = false;        // address not acknowledged (device absent)
    bool    hang = false;        // never responds (timeout)
    int     i2cWriteCalls = 0;
    int     i2cReadCalls = 0;
    bool    lastNostop = false;
    uint32_t lastWriteBudgetUs = 0;
    uint32_t lastReadBudgetUs = 0;
    uint64_t advanceOnI2cUs = 0;  // simulated bus time consumed per phase
    // ADC
    uint16_t adcCount = 0x5A5;
    int      adcCalls = 0;
    bool     adcOk = true;
    // GPIO pad
    uint8_t  padLevel = 0;
    bool     padDriven = false;
    int      gpioWriteCalls = 0;
    int      gpioReadCalls = 0;
    // Retime
    int      retimeCalls = 0;
    uint32_t lastRetimeHz = 0;
    bool     retimeOk = true;
};

bool HwReadTempRaw(void* ctx, uint16_t& outCount) {
    FakeHw* h = static_cast<FakeHw*>(ctx);
    ++h->adcCalls;
    if (!h->adcOk) return false;
    outCount = h->adcCount & 0x0FFF;
    return true;
}

int HwI2cWrite(void* ctx, const uint8_t* tx, uint8_t len, bool nostop, uint32_t budgetUs) {
    FakeHw* h = static_cast<FakeHw*>(ctx);
    ++h->i2cWriteCalls;
    h->lastNostop = nostop;
    h->lastWriteBudgetUs = budgetUs;
    h->nowUs += h->advanceOnI2cUs;
    if (h->hang) return kHalTimeout;
    if (h->nack || len == 0) return kHalIoError;
    h->regPtr = tx[0] & 0x0F;
    for (uint8_t i = 1; i < len; ++i) h->regs[h->regPtr++ & 0x0F] = tx[i];
    if (!nostop) h->regPtr = 0;
    return len;
}

int HwI2cRead(void* ctx, uint8_t* rx, uint8_t len, uint32_t budgetUs) {
    FakeHw* h = static_cast<FakeHw*>(ctx);
    ++h->i2cReadCalls;
    h->lastReadBudgetUs = budgetUs;
    h->nowUs += h->advanceOnI2cUs;
    if (h->hang) return kHalTimeout;
    if (h->nack) return kHalIoError;
    for (uint8_t i = 0; i < len; ++i) rx[i] = h->regs[h->regPtr++ & 0x0F];
    h->regPtr = 0;
    return len;
}

bool HwGpioWrite(void* ctx, uint8_t value) {
    FakeHw* h = static_cast<FakeHw*>(ctx);
    ++h->gpioWriteCalls;
    if (value > 1) return false;
    h->padLevel = value;
    h->padDriven = true;
    return true;
}

bool HwGpioRead(void* ctx, uint8_t& outValue) {
    FakeHw* h = static_cast<FakeHw*>(ctx);
    ++h->gpioReadCalls;
    outValue = h->padLevel;
    return true;
}

bool HwRetime(void* ctx, uint32_t newSysHz) {
    FakeHw* h = static_cast<FakeHw*>(ctx);
    ++h->retimeCalls;
    h->lastRetimeHz = newSysHz;
    return h->retimeOk;
}

uint64_t HwTimeUs(void* ctx) { return static_cast<FakeHw*>(ctx)->nowUs; }

DeviceHal MakeHal(FakeHw& h) {
    DeviceHal hal;
    hal.ctx = &h;
    hal.readTempRaw = &HwReadTempRaw;
    hal.i2cWrite = &HwI2cWrite;
    hal.i2cRead = &HwI2cRead;
    hal.gpioWrite = &HwGpioWrite;
    hal.gpioRead = &HwGpioRead;
    hal.retime = &HwRetime;
    hal.timeUs = &HwTimeUs;
    return hal;
}

DeviceRequest Req(uint8_t dev, DeviceOperation op, uint8_t txLen, uint8_t rxLen,
                  const uint8_t* tx = nullptr) {
    DeviceRequest r;
    r.deviceId = dev;
    r.operation = op;
    r.txLen = txLen;
    r.rxLen = rxLen;
    if (tx) std::memcpy(r.tx, tx, txLen <= kMaxTxBytes ? txLen : kMaxTxBytes);
    return r;
}

// ─── Wire decode ────────────────────────────────────────────────────────────

void TestDecodeArgs() {
    // args0 bytes {deviceId, operation, txLen, rxLen}; args1..3 LE TX bytes.
    const uint8_t tx[12] = {1, 2, 3, 4, 5, 6, 7, 8, 9, 10, 11, 12};
    uint32_t args[4] = {};
    args[0] = 1u | (2u << 8) | (12u << 16) | (16u << 24);
    for (int i = 0; i < 12; ++i) args[1 + i / 4] |= uint32_t(tx[i]) << ((i % 4) * 8);

    DeviceRequest r;
    CHECK(DecodeDeviceArgs(args, r) == Result::Ok);
    CHECK(r.deviceId == 1);
    CHECK(r.operation == DeviceOperation::Write);
    CHECK(r.txLen == 12 && r.rxLen == 16);
    CHECK(std::memcmp(r.tx, tx, 12) == 0);

    CHECK(DecodeDeviceArgs(nullptr, r) == Result::BadPacket);

    uint32_t bad[4] = {};
    bad[0] = 3u;  // unknown device
    CHECK(DecodeDeviceArgs(bad, r) == Result::InvalidValue);
    bad[0] = 0u | (0u << 8);  // operation 0 is not a valid enum
    CHECK(DecodeDeviceArgs(bad, r) == Result::InvalidValue);
    bad[0] = 0u | (4u << 8);  // operation 4 out of range
    CHECK(DecodeDeviceArgs(bad, r) == Result::InvalidValue);
    bad[0] = 0u | (1u << 8) | (13u << 16);  // txLen > 12
    CHECK(DecodeDeviceArgs(bad, r) == Result::InvalidValue);
    bad[0] = 0u | (1u << 8) | (17u << 24);  // rxLen > 16
    CHECK(DecodeDeviceArgs(bad, r) == Result::InvalidValue);
}

// ─── Validation: explicit error replies, no silent fallback ─────────────────

void TestValidationMatrix() {
    FakeHw h;
    DeviceService s;
    CHECK(s.AttachHal(MakeHal(h)));

    struct Case { DeviceRequest req; };
    const uint8_t one[1] = {1};
    const uint8_t two[2] = {1, 0};
    const uint8_t bad2[1] = {2};
    const Case cases[] = {
        { Req(kDeviceTempAdc, DeviceOperation::Write, 1, 0, one) },   // sensor not writable
        { Req(kDeviceTempAdc, DeviceOperation::Read, 0, 1) },         // raw count is 2 bytes
        { Req(kDeviceTempAdc, DeviceOperation::Read, 1, 2, one) },    // no TX for ADC
        { Req(kDeviceTempAdc, DeviceOperation::ReadWrite, 0, 2) },    // ReadWrite n/a
        { Req(kDeviceAuxI2c, DeviceOperation::Read, 1, 4, one) },     // Read takes no TX
        { Req(kDeviceAuxI2c, DeviceOperation::Read, 0, 0) },          // empty read
        { Req(kDeviceAuxI2c, DeviceOperation::Write, 0, 0) },         // empty write
        { Req(kDeviceAuxI2c, DeviceOperation::Write, 1, 1, one) },    // write takes no RX
        { Req(kDeviceAuxI2c, DeviceOperation::ReadWrite, 0, 4) },     // missing TX phase
        { Req(kDeviceAuxI2c, DeviceOperation::ReadWrite, 1, 0, one) },// missing RX phase
        { Req(kDeviceGpio, DeviceOperation::Read, 0, 2) },            // level is 1 byte
        { Req(kDeviceGpio, DeviceOperation::Write, 2, 0, two) },      // exactly 1 byte
        { Req(kDeviceGpio, DeviceOperation::Write, 1, 0, bad2) },     // drive only 0/1
        { Req(kDeviceGpio, DeviceOperation::ReadWrite, 1, 0, one) },  // missing readback
        { Req(9, DeviceOperation::Read, 0, 1) },                      // unknown sensor
    };
    uint32_t id = 0;
    PglRuntime::DeviceReply reply;
    Result res;
    for (const Case& c : cases) {
        CHECK(s.Submit(c.req, id) == Result::InvalidValue);
        CHECK(id != 0);
        CHECK(s.PendingCount() == 0);  // invalid requests never queue
        // Each invalid request yields exactly one explicit error reply.
        CHECK(s.PopReply(reply, res));
        CHECK(res == Result::InvalidValue);
        CHECK(reply.length == 0);
        CHECK(reply.requestId == id);
    }
    CHECK(s.ReplyCount() == 0);
    CHECK(h.adcCalls == 0 && h.i2cWriteCalls == 0 && h.i2cReadCalls == 0 &&
          h.gpioWriteCalls == 0 && h.gpioReadCalls == 0);  // hardware untouched
}

// ─── Temperature ADC: raw count + timestamp, not calibrated ─────────────────

void TestTempAdcRaw() {
    FakeHw h;
    h.adcCount = 0x0ABC;
    h.nowUs = 4242;
    DeviceService s;
    CHECK(s.AttachHal(MakeHal(h)));

    uint32_t id = 0;
    CHECK(s.Submit(Req(kDeviceTempAdc, DeviceOperation::Read, 0, 2), id) == Result::Ok);
    s.Poll();
    PglRuntime::DeviceReply reply;
    Result res;
    CHECK(s.PopReply(reply, res));
    CHECK(res == Result::Ok);
    CHECK(reply.requestId == id);
    CHECK(reply.deviceId == kDeviceTempAdc);
    CHECK(reply.length == 2);
    CHECK(reply.data[0] == 0xBC && reply.data[1] == 0x0A);  // raw 12-bit LE
    CHECK(reply.timestampUs >= 4242);
    CHECK(h.adcCalls == 1);

    // ADC failure maps to Io with an empty payload.
    h.adcOk = false;
    CHECK(s.Submit(Req(kDeviceTempAdc, DeviceOperation::Read, 0, 2), id) == Result::Ok);
    s.Poll();
    CHECK(s.PopReply(reply, res));
    CHECK(res == Result::Io);
    CHECK(reply.length == 0);
}

// ─── I2C register file: write/read/readwrite through the real plumbing ──────

void TestI2cRegisterFile() {
    FakeHw h;
    DeviceService s;
    CHECK(s.AttachHal(MakeHal(h)));
    uint32_t id = 0;

    // Write {reg=3, v0, v1} to the fixed 0x3D device.
    const uint8_t wr[3] = {3, 0xDE, 0xAD};
    CHECK(s.Submit(Req(kDeviceAuxI2c, DeviceOperation::Write, 3, 0, wr), id) == Result::Ok);
    s.Poll();
    PglRuntime::DeviceReply reply;
    Result res;
    CHECK(s.PopReply(reply, res));
    CHECK(res == Result::Ok && reply.length == 0);
    CHECK(h.regs[3] == 0xDE && h.regs[4] == 0xAD);
    CHECK(!h.lastNostop);  // plain write ends with STOP

    // Read 2 bytes starting at register 3 via ReadWrite (repeated start).
    const uint8_t sel[1] = {3};
    CHECK(s.Submit(Req(kDeviceAuxI2c, DeviceOperation::ReadWrite, 1, 2, sel), id) == Result::Ok);
    s.Poll();
    CHECK(s.PopReply(reply, res));
    CHECK(res == Result::Ok);
    CHECK(reply.length == 2);
    CHECK(reply.data[0] == 0xDE && reply.data[1] == 0xAD);
    CHECK(h.lastNostop == false || h.i2cReadCalls == 1);  // read phase seen
    CHECK(h.i2cWriteCalls == 2 && h.i2cReadCalls == 1);

    // Budget clamp: every phase is capped at the declared 1 ms slice.
    CHECK(h.lastWriteBudgetUs <= kI2cSliceUs);
    CHECK(h.lastReadBudgetUs <= kI2cSliceUs);

    // Device absent (NACK) maps to Io, empty payload.
    h.nack = true;
    const uint8_t rd[1] = {0};
    CHECK(s.Submit(Req(kDeviceAuxI2c, DeviceOperation::ReadWrite, 1, 1, rd), id) == Result::Ok);
    s.Poll();
    CHECK(s.PopReply(reply, res));
    CHECK(res == Result::Io && reply.length == 0);

    // Hung device resolves finitely as Timeout (never blocks the host).
    h.nack = false;
    h.hang = true;
    CHECK(s.Submit(Req(kDeviceAuxI2c, DeviceOperation::Read, 0, 4), id) == Result::Ok);
    s.Poll();
    CHECK(s.PopReply(reply, res));
    CHECK(res == Result::Timeout && reply.length == 0);
    h.hang = false;
}

// ─── GPIO: drive only on explicit write ─────────────────────────────────────

void TestGpioBounds() {
    FakeHw h;
    DeviceService s;
    CHECK(s.AttachHal(MakeHal(h)));
    uint32_t id = 0;

    // Read of an undriven pad: input level, no drive occurred.
    CHECK(s.Submit(Req(kDeviceGpio, DeviceOperation::Read, 0, 1), id) == Result::Ok);
    s.Poll();
    PglRuntime::DeviceReply reply;
    Result res;
    CHECK(s.PopReply(reply, res));
    CHECK(res == Result::Ok && reply.length == 1 && reply.data[0] == 0);
    CHECK(!h.padDriven);
    CHECK(h.gpioWriteCalls == 0);

    // Explicit write drives 1; readback reports the driven level.
    const uint8_t one[1] = {1};
    CHECK(s.Submit(Req(kDeviceGpio, DeviceOperation::ReadWrite, 1, 1, one), id) == Result::Ok);
    s.Poll();
    CHECK(s.PopReply(reply, res));
    CHECK(res == Result::Ok && reply.length == 1 && reply.data[0] == 1);
    CHECK(h.padDriven && h.padLevel == 1);

    const uint8_t zero[1] = {0};
    CHECK(s.Submit(Req(kDeviceGpio, DeviceOperation::Write, 1, 0, zero), id) == Result::Ok);
    s.Poll();
    CHECK(s.PopReply(reply, res));
    CHECK(res == Result::Ok && h.padLevel == 0);
}

// ─── Queue bounds, ordering, cancel ownership ────────────────────────────────

void TestQueueBusyAndOrder() {
    FakeHw h;
    DeviceService s;
    CHECK(s.AttachHal(MakeHal(h)));

    // Fill the bounded queue; the next submit must be Busy.
    uint32_t ids[kQueueDepth] = {};
    for (int i = 0; i < kQueueDepth; ++i) {
        CHECK(s.Submit(Req(kDeviceGpio, DeviceOperation::Read, 0, 1), ids[i]) == Result::Ok);
    }
    uint32_t extra = 0;
    CHECK(s.Submit(Req(kDeviceGpio, DeviceOperation::Read, 0, 1), extra) == Result::Busy);
    CHECK(extra == 0);
    CHECK(s.PendingCount() == kQueueDepth);

    // Poll executes at most ONE request per call (bounded service slice).
    s.Poll();
    CHECK(s.PendingCount() == kQueueDepth - 1);
    CHECK(s.ReplyCount() == 1);

    // Replies arrive FIFO with their request ids.
    PglRuntime::DeviceReply reply;
    Result res;
    CHECK(s.PopReply(reply, res));
    CHECK(reply.requestId == ids[0] && res == Result::Ok);
}

void TestCancelOwnership() {
    FakeHw h;
    DeviceService s;
    CHECK(s.AttachHal(MakeHal(h)));

    uint32_t idA = 0, idB = 0;
    CHECK(s.Submit(Req(kDeviceGpio, DeviceOperation::Read, 0, 1), idA) == Result::Ok);
    CHECK(s.Submit(Req(kDeviceGpio, DeviceOperation::Read, 0, 1), idB) == Result::Ok);

    // Unknown id and double-cancel are explicit errors.
    CHECK(s.Cancel(0xDEADBEEF) == Result::InvalidHandle);
    CHECK(s.Cancel(idB) == Result::Ok);
    CHECK(s.Cancel(idB) == Result::InvalidHandle);
    CHECK(s.PendingCount() == 1);

    // B's Cancelled reply is emitted at cancel time; A still completes
    // through Poll without B ever touching hardware.
    const int gpioReadsBefore = h.gpioReadCalls;
    s.Poll();
    PglRuntime::DeviceReply reply;
    Result res;
    CHECK(s.PopReply(reply, res));
    CHECK(reply.requestId == idB && res == Result::Cancelled && reply.length == 0);
    CHECK(s.PopReply(reply, res));
    CHECK(reply.requestId == idA && res == Result::Ok);
    CHECK(h.gpioReadCalls == gpioReadsBefore + 1);  // only A executed

    // A completed id can no longer be cancelled.
    CHECK(s.Cancel(idA) == Result::InvalidHandle);
}

// ─── Deadlines ──────────────────────────────────────────────────────────────

void TestQueuedDeadlineExpires() {
    FakeHw h;
    DeviceService s;
    CHECK(s.AttachHal(MakeHal(h)));

    uint32_t id = 0;
    CHECK(s.Submit(Req(kDeviceAuxI2c, DeviceOperation::Read, 0, 4), id) == Result::Ok);
    // Queue a second request behind it so the first expires while waiting:
    // advance time beyond the per-request deadline before any Poll.
    h.nowUs += kI2cSliceUs + 1;
    s.Poll();
    PglRuntime::DeviceReply reply;
    Result res;
    CHECK(s.PopReply(reply, res));
    CHECK(res == Result::Timeout);
    CHECK(reply.length == 0);
    CHECK(h.i2cReadCalls == 0 && h.i2cWriteCalls == 0);  // hardware untouched
}

void TestMidTransactionDeadline() {
    FakeHw h;
    DeviceService s;
    CHECK(s.AttachHal(MakeHal(h)));

    // ReadWrite whose write phase consumes the remaining deadline: the read
    // phase must resolve as Timeout, never exceeding the budget.
    h.advanceOnI2cUs = kI2cSliceUs;  // each phase eats the whole slice
    const uint8_t sel[1] = {0};
    uint32_t id = 0;
    CHECK(s.Submit(Req(kDeviceAuxI2c, DeviceOperation::ReadWrite, 1, 2, sel), id) == Result::Ok);
    s.Poll();
    PglRuntime::DeviceReply reply;
    Result res;
    CHECK(s.PopReply(reply, res));
    CHECK(res == Result::Timeout);
    CHECK(reply.length == 0);
    CHECK(h.i2cWriteCalls == 1);
    CHECK(h.i2cReadCalls == 0);  // second phase refused after expiry
}

// ─── Clock retime quiescence ────────────────────────────────────────────────

void TestRetimeQuiescent() {
    FakeHw h;
    DeviceService s;
    CHECK(s.AttachHal(MakeHal(h)));

    uint32_t id = 0;
    CHECK(s.Submit(Req(kDeviceGpio, DeviceOperation::Read, 0, 1), id) == Result::Ok);
    CHECK(!s.RetimeForClock(100000000u));  // refuses while a request is queued
    CHECK(h.retimeCalls == 0);
    s.Poll();
    PglRuntime::DeviceReply reply;
    Result res;
    CHECK(s.PopReply(reply, res));
    CHECK(s.IsIdle());
    CHECK(s.RetimeForClock(100000000u));
    CHECK(h.retimeCalls == 1 && h.lastRetimeHz == 100000000u);

    h.retimeOk = false;
    CHECK(!s.RetimeForClock(75000000u));  // retime failure is visible
}

// ─── HAL lifecycle ──────────────────────────────────────────────────────────

void TestHalLifecycle() {
    FakeHw h;
    DeviceService s;
    uint32_t id = 0;
    // No HAL attached: BadState, nothing queued.
    CHECK(s.Submit(Req(kDeviceGpio, DeviceOperation::Read, 0, 1), id) == Result::BadState);

    CHECK(s.AttachHal(MakeHal(h)));
    CHECK(s.Submit(Req(kDeviceGpio, DeviceOperation::Read, 0, 1), id) == Result::Ok);
    // Re-attach while busy is refused (no hardware swap mid-request).
    CHECK(!s.AttachHal(MakeHal(h)));
    s.Poll();
    PglRuntime::DeviceReply reply;
    Result res;
    CHECK(s.PopReply(reply, res));
    // Detach (shutdown path): queues reset, service back to BadState.
    CHECK(s.AttachHal(DeviceHal{}));
    CHECK(!s.HasHal());
    CHECK(s.Submit(Req(kDeviceGpio, DeviceOperation::Read, 0, 1), id) == Result::BadState);
}

}  // namespace

int main() {
    TestDecodeArgs();
    TestValidationMatrix();
    TestTempAdcRaw();
    TestI2cRegisterFile();
    TestGpioBounds();
    TestQueueBusyAndOrder();
    TestCancelOwnership();
    TestQueuedDeadlineExpires();
    TestMidTransactionDeadline();
    TestRetimeQuiescent();
    TestHalLifecycle();

    std::printf("device_service tests: %d checks, %d failures\n", gChecks, gFailures);
    if (gFailures == 0) {
        std::printf("RESULT: PASS\n");
        return 0;
    }
    std::printf("RESULT: FAIL\n");
    return 1;
}
