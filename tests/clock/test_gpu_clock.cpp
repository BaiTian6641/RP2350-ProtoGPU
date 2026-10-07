// GpuClock native behavior tests (P09 clock slice).
//
// Exercises the REAL state machine / gate / thermal / ordering logic in
// src/gpu_clock.cpp with the Platform seam standing in for hardware: the
// fake records the physical transition calls the shipping code makes and
// supplies clock readings — there is no profile logic in the fake itself.
// Actual PLL/QMI register execution and electrical profile qualification
// remain target/HIL acceptance, explicitly not covered here.
//
// Build and run via tests/clock/run_tests.sh (native g++, no Pico SDK).

#include "gpu_clock.h"

#include <cstdint>
#include <cstdio>
#include <cstring>

using namespace GpuClock;
using PglRuntime::Result;

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

// ─── Fake platform: records transition calls, supplies clock readings ───────
struct Fake {
    uint32_t hz = 150000000u;
    uint16_t millivolts = 1100;
    uint32_t peripheralHz = 48000000;
    uint32_t referenceHz = 12000000;
    uint32_t usbHz = 48000000;
    uint32_t adcHz = 48000000;
    uint32_t hstxHz = 48000000;
    bool domainsOk = true;
    bool setVoltageOk = true;
    // Recorded calls (order matters): "commit", "qmiPre", "pll", "ticks",
    // "qmiFin", "retime".
    char events[16][8] = {};
    int  eventCount = 0;
    uint8_t lastCommitDiv = 0;
    int  commitCount = 0;
    bool switchOk = true;
    bool commitOk = true;
    bool timebaseOk = true;
    bool qmiPrepareOk = true;
    bool qmiFinalizeOk = true;
    bool verifyDomainsCalled = false;
    bool retimeOk = true;
    // Hooks live in the same struct so order is observable across seams.
    bool hooksInstalled = false;

    void event(const char* name) {
        if (eventCount < 16) std::strncpy(events[eventCount], name, 7), events[eventCount][7] = 0;
        ++eventCount;
    }
    bool eventIs(int i, const char* name) const {
        return i < eventCount && std::strcmp(events[i], name) == 0;
    }
};

uint32_t FakeHz(void* ctx) { return static_cast<Fake*>(ctx)->hz; }

bool FakeSwitch(void* ctx, uint32_t vcoHz, uint8_t pd1, uint8_t pd2) {
    Fake* f = static_cast<Fake*>(ctx);
    f->event("pll");
    if (!f->switchOk) return false;
    f->hz = vcoHz / (static_cast<uint32_t>(pd1) * pd2);
    return true;
}

bool FakeCommit(void* ctx, uint8_t divisor) {
    Fake* f = static_cast<Fake*>(ctx);
    f->event("commit");
    ++f->commitCount;
    f->lastCommitDiv = divisor;
    return f->commitOk;
}

bool FakeTimebase(void* ctx) {
    Fake* f = static_cast<Fake*>(ctx);
    f->event("ticks");
    return f->timebaseOk;
}

uint64_t FakeTimeUs(void*) { return 0; }


bool FakeSetMillivolts(void* ctx, uint16_t millivolts) {
    Fake* f = static_cast<Fake*>(ctx);
    f->event("vsel");
    if (!f->setVoltageOk || (millivolts != 1100 && millivolts != 1200)) return false;
    f->millivolts = millivolts;
    return true;
}
uint16_t FakeMillivolts(void* ctx) { return static_cast<Fake*>(ctx)->millivolts; }
uint32_t FakePeripheralHz(void* ctx) { return static_cast<Fake*>(ctx)->peripheralHz; }
uint32_t FakeReferenceHz(void* ctx) { return static_cast<Fake*>(ctx)->referenceHz; }
uint32_t FakeUsbHz(void* ctx) { return static_cast<Fake*>(ctx)->usbHz; }
uint32_t FakeAdcHz(void* ctx) { return static_cast<Fake*>(ctx)->adcHz; }
uint32_t FakeHstxHz(void* ctx) { return static_cast<Fake*>(ctx)->hstxHz; }
bool FakeVerifyDomains(void* ctx) {
    Fake* f = static_cast<Fake*>(ctx);
    f->verifyDomainsCalled = true;
    return f->domainsOk;
}
bool FakeQmiPrepare(void* ctx, uint32_t) {
    Fake* f = static_cast<Fake*>(ctx);
    f->event("qmiPre");
    return f->qmiPrepareOk;
}

bool FakeQmiFinalize(void* ctx, uint32_t) {
    Fake* f = static_cast<Fake*>(ctx);
    f->event("qmiFin");
    return f->qmiFinalizeOk;
}

bool FakeRetime(void* ctx, uint32_t) {
    Fake* f = static_cast<Fake*>(ctx);
    f->event("retime");
    return f->retimeOk;
}

Platform MakePlatform(Fake& f) {
    Platform p;
    p.ctx = &f;
    p.sysClockHz = &FakeHz;
    p.switchSysClockPll = &FakeSwitch;
    p.commitFlashDivisor = &FakeCommit;
    p.preserveTimebase = &FakeTimebase;
    p.timeUs = &FakeTimeUs;
    p.setCoreMillivolts = &FakeSetMillivolts;
    p.coreMillivolts = &FakeMillivolts;
    p.peripheralHz = &FakePeripheralHz;
    p.referenceHz = &FakeReferenceHz;
    p.usbHz = &FakeUsbHz;
    p.adcHz = &FakeAdcHz;
    p.hstxHz = &FakeHstxHz;
    p.verifyFixedDomains = &FakeVerifyDomains;
    return p;
}
ApplyHooks MakeHooks(Fake& f) {
    ApplyHooks h;
    h.ctx = &f;
    h.qmiPrepare = &FakeQmiPrepare;
    h.qmiFinalize = &FakeQmiFinalize;
    h.retimeClients = &FakeRetime;
    f.hooksInstalled = true;
    return h;
}

SafeGates AllSafe() {
    SafeGates g;
    g.workersParked = true;
    g.hostIdle = true;
    g.reservationArmed = false;
    g.displayDrained = true;
    g.devicesDrained = true;
    g.memoryDrained = true;
    return g;
}

// ─── Tests ──────────────────────────────────────────────────────────────────

void TestProfileTable() {
    CHECK(kProfileCount == PglRuntime::ClockProfileCount);
    CHECK(PglRuntime::ClockProfileMask == 0x01ff);
    static const uint16_t frequencies[9] = {150,100,75,125,240,288,250,300,336};
    for (uint8_t id = 0; id < kProfileCount; ++id) {
        CHECK(IsValidProfile(id));
        CHECK(GetProfile(id).freqMHz == frequencies[id]);
        CHECK(GetProfile(id).coreMillivolts == (frequencies[id] > 150 ? 1200 : 1100));
        CHECK(PglRuntime::ClockProfileFrequencyMHz(id) == frequencies[id]);
        CHECK(PglRuntime::ClockProfileForMHz(frequencies[id]) == id);
        CHECK(PglRuntime::ClockProfileCoreMillivolts(id) == GetProfile(id).coreMillivolts);
    }
    CHECK(!IsValidProfile(9) && !IsValidProfile(kProfileInvalid));
    CHECK(PglRuntime::ClockProfileForMHz(200) == 0xff &&
          PglRuntime::ClockProfileForMHz(225) == 0xff);
    CHECK(PglRuntime::ClockProfileFrequencyMHz(9) == 0);
    CHECK(FlashClkDivFor(150000000u) == 4);
    CHECK(FlashClkDivFor(100000000u) == 4);
    CHECK(FlashClkDivFor(75000000u) == 4);
    CHECK(FlashClkDivFor(125000000u) == 4);
    CHECK(FlashClkDivFor(240000000u) == 7);
    CHECK(FlashClkDivFor(288000000u) == 8);
    CHECK(FlashClkDivFor(250000000u) == 7);
    CHECK(FlashClkDivFor(300000000u) == 8);
    CHECK(FlashClkDivFor(336000000u) == 9);
    CHECK(FlashClkDivFor(0u) == kFlashMinClkDiv);
    CHECK(FlashClkDivFor(400000000u) == 11);
}

void TestGateMatrix() {
    SafeGates g = AllSafe();
    CHECK(GatesSafe(g));
    g.workersParked = false;   CHECK(!GatesSafe(g)); g = AllSafe();
    g.hostIdle = false;        CHECK(!GatesSafe(g)); g = AllSafe();
    g.reservationArmed = true; CHECK(!GatesSafe(g)); g = AllSafe();
    g.displayDrained = false;  CHECK(!GatesSafe(g)); g = AllSafe();
    g.devicesDrained = false;  CHECK(!GatesSafe(g)); g = AllSafe();
    g.memoryDrained = false;   CHECK(!GatesSafe(g));
}

void TestInitAndIdempotent() {
    Fake f;
    Initialize(MakePlatform(f));
    Snapshot s = GetSnapshot();
    CHECK(s.requested == kProfileBaseline150);
    CHECK(s.actual == kProfileBaseline150);
    CHECK(s.actualHz == 150000000u);
    CHECK(s.transition == Transition::Idle);
    CHECK(s.override == Override::None);
    CHECK(!s.thermalEnabled);  // policy OFF until configured

    // Idempotent request of the current profile: Ok, still nothing pending.
    CHECK(RequestProfile(kProfileBaseline150) == Result::Ok);
    CHECK(GetSnapshot().transition == Transition::Idle);
    CHECK(TryApply(AllSafe(), MakeHooks(f)) == Result::Ok);
    CHECK(f.eventCount == 0);  // no physical work for a no-op apply
}

void TestUnknownProfileRejected() {
    Fake f;
    Initialize(MakePlatform(f));
    CHECK(RequestProfile(kProfileCount) == Result::InvalidValue);
    CHECK(RequestProfile(200) == Result::InvalidValue);
    CHECK(GetSnapshot().requested == kProfileBaseline150);
    CHECK(GetSnapshot().transition == Transition::Idle);
    CHECK(GetSnapshot().lastResult == Result::InvalidValue);
    CHECK(RequestProfile(kProfile100) == Result::Ok);
    CHECK(GetSnapshot().requested == kProfile100);
    CHECK(GetSnapshot().transition == Transition::AwaitingSafePoint);
}

void TestVoltageSequencingAllProfiles() {
    Fake f;
    Initialize(MakePlatform(f));
    for (uint8_t id = 0; id < kProfileCount; ++id) {
        CHECK(RequestProfile(id) == Result::Ok);
        f.eventCount = 0;
        f.commitCount = 0;
        CHECK(TryApply(AllSafe(), MakeHooks(f)) == Result::Ok);
        Snapshot s = GetSnapshot();
        CHECK(s.actual == id && s.actualHz == GetProfile(id).freqMHz * 1000000u);
        const uint16_t required = GetProfile(id).coreMillivolts;
        const bool voltageAlreadySatisfied = f.millivolts == required;
        if (required > 1100 && !voltageAlreadySatisfied) CHECK(f.eventIs(0, "vsel"));
        if (required == 1100 && id && !voltageAlreadySatisfied) CHECK(f.eventIs(f.eventCount - 1, "vsel"));
        CHECK(f.millivolts == required);
        f.hz = s.actualHz;
    }
}

void TestUnknownActualNormalizesAt1V2BeforeOverclock() {
    Fake f;
    f.hz = 123000000u;
    Initialize(MakePlatform(f));
    CHECK(GetSnapshot().actual == kProfileInvalid);
    CHECK(RequestProfile(kProfile336) == Result::Ok);
    f.eventCount = 0;
    CHECK(TryApply(AllSafe(), MakeHooks(f)) == Result::Ok);
    CHECK(f.eventIs(0, "vsel"));
    CHECK(f.millivolts == 1200 && f.hz == 336000000u);
}

void TestDomainAndVoltageFailureBoundaries() {
    Fake f;
    Initialize(MakePlatform(f));
    CHECK(RequestProfile(kProfile336) == Result::Ok);
    f.domainsOk = false;
    CHECK(TryApply(AllSafe(), MakeHooks(f)) == Result::Io);
    CHECK(GetSnapshot().actual == kProfileBaseline150);
    CHECK(f.millivolts == 1100 && f.eventCount == 0);
    f.domainsOk = true;
    f.setVoltageOk = false;
    CHECK(TryApply(AllSafe(), MakeHooks(f)) == Result::Io);
    CHECK(GetSnapshot().actual == kProfileBaseline150);
    CHECK(f.millivolts == 1100);
    f.setVoltageOk = true;
    CHECK(TryApply(AllSafe(), MakeHooks(f)) == Result::Ok);
    CHECK(f.millivolts == 1200 && f.hz == 336000000u);
}

void TestClockConfigurationPublication() {
    Fake f;
    Initialize(MakePlatform(f));
    PglRuntime::ClockConfiguration cfg{};
    CHECK(GetClockConfiguration(0xff,cfg) == Result::Ok);
    CHECK(cfg.profileId == kProfileBaseline150 && cfg.flags == uint8_t(PglRuntime::Available));
    CHECK(cfg.supportedProfilesMask == PglRuntime::ClockProfileMask);
    CHECK(cfg.actualCoreMillivolts == 1100 && cfg.systemHz == 150000000u);
    CHECK(cfg.peripheralHz == cfg.usbHz && cfg.peripheralHz == 48000000u);
    CHECK(cfg.referenceHz == 12000000u && cfg.hstxHz == 48000000u);
    CHECK((cfg.domainFlags & PglRuntime::VerifiedFixedDomains) != 0);
    CHECK(RequestProfile(kProfile300) == Result::Ok);
    CHECK(TryApply(AllSafe(), MakeHooks(f)) == Result::Ok);
    CHECK(GetClockConfiguration(uint8_t(PglRuntime::ClockProfile::MHz336),cfg) == Result::Ok);
    CHECK((cfg.flags & PglRuntime::Overclock) && !(cfg.flags & PglRuntime::UserBoardReference));
    CHECK(cfg.frequencyHz == 336000000u && cfg.vcoHz == 1344000000u);
    CHECK(GetClockConfiguration(9,cfg) == Result::InvalidValue);
    CHECK(GetClockConfiguration(200,cfg) == Result::InvalidValue);
    CHECK(GetClockConfiguration(uint8_t(PglRuntime::ClockProfile::MHz300),cfg) == Result::Ok);
    CHECK((cfg.flags & PglRuntime::UserBoardReference) && cfg.coreMillivolts == 1200);
    CHECK(GetClockConfiguration(0xff,cfg) == Result::Ok);
    CHECK(cfg.profileId == kProfile300 && cfg.actualCoreMillivolts == 1200);
}

void TestBusyGateNeverApplies() {
    Fake f;
    Initialize(MakePlatform(f));
    CHECK(RequestProfile(kProfile100) == Result::Ok);

    // Every gate, when false, must yield Busy with ZERO physical calls.
    for (int gate = 0; gate < 6; ++gate) {
        SafeGates g = AllSafe();
        switch (gate) {
            case 0: g.workersParked = false; break;
            case 1: g.hostIdle = false; break;
            case 2: g.reservationArmed = true; break;
            case 3: g.displayDrained = false; break;
            case 4: g.devicesDrained = false; break;
            case 5: g.memoryDrained = false; break;
        }
        CHECK(TryApply(g, MakeHooks(f)) == Result::Busy);
        CHECK(f.eventCount == 0);
        Snapshot s = GetSnapshot();
        CHECK(s.actual == kProfileBaseline150);      // not silently applied
        CHECK(s.transition == Transition::AwaitingSafePoint);  // still pending
    }

    // Gates satisfied: the deferred apply now runs to completion.
    CHECK(TryApply(AllSafe(), MakeHooks(f)) == Result::Ok);
    Snapshot s = GetSnapshot();
    CHECK(s.actual == kProfile100);
    CHECK(s.actualHz == 100000000u);
    CHECK(s.transition == Transition::Idle);
}

void TestTransitionOrderingDecrease() {
    Fake f;  // 150 -> 75 (decrease)
    Initialize(MakePlatform(f));
    CHECK(RequestProfile(kProfile75) == Result::Ok);
    CHECK(TryApply(AllSafe(), MakeHooks(f)) == Result::Ok);

    // Decrease: no pre-switch flash commit; the direction-agnostic PSRAM
    // prepare hook still runs first, and qmiFinalize's boot2 restore is
    // still followed by the mandatory re-commit before clients retime.
    CHECK(f.eventCount == 6);
    CHECK(f.eventIs(0, "qmiPre"));
    CHECK(f.eventIs(1, "pll"));
    CHECK(f.eventIs(2, "ticks"));
    CHECK(f.eventIs(3, "qmiFin"));
    CHECK(f.eventIs(4, "commit"));   // AFTER qmiFinalize (boot2 restore)
    CHECK(f.eventIs(5, "retime"));
    CHECK(f.lastCommitDiv == 4);
    CHECK(f.millivolts == 1100);
    CHECK(GetSnapshot().actual == kProfile75);
}

void TestTransitionOrderingIncrease() {
    Fake f;
    f.hz = 75000000u;  // boot at 75 MHz so the apply is an increase
    Initialize(MakePlatform(f));
    CHECK(GetSnapshot().actual == kProfile75);
    CHECK(RequestProfile(kProfileBaseline150) == Result::Ok);
    CHECK(TryApply(AllSafe(), MakeHooks(f)) == Result::Ok);

    // Increase: conservative M0 divisor committed BEFORE the PLL switch,
    // and re-committed AFTER qmiFinalize's flash_start_xip boot2 restore.
    CHECK(f.eventCount == 7);
    CHECK(f.eventIs(0, "commit"));
    CHECK(f.eventIs(1, "qmiPre"));
    CHECK(f.eventIs(2, "pll"));
    CHECK(f.eventIs(3, "ticks"));
    CHECK(f.eventIs(4, "qmiFin"));
    CHECK(f.eventIs(5, "commit"));
    CHECK(f.eventIs(6, "retime"));
    CHECK(f.commitCount == 2);
    CHECK(f.lastCommitDiv == 4);
    CHECK(f.millivolts == 1100);
    CHECK(GetSnapshot().actual == kProfileBaseline150);
    CHECK(GetSnapshot().actualHz == 150000000u);
}

void TestPrepareFailureRecovery() {
    Fake f;
    Initialize(MakePlatform(f));
    CHECK(RequestProfile(kProfile100) == Result::Ok);

    f.qmiPrepareOk = false;
    CHECK(TryApply(AllSafe(), MakeHooks(f)) == Result::Io);
    Snapshot s = GetSnapshot();
    CHECK(s.actual == kProfileBaseline150);  // clock never switched
    CHECK(s.transition == Transition::AwaitingSafePoint);  // request retained
    CHECK(f.eventIs(0, "qmiPre"));

    // Recovery: same pending request applies once the blocker clears.
    f.qmiPrepareOk = true;
    CHECK(TryApply(AllSafe(), MakeHooks(f)) == Result::Ok);
    CHECK(GetSnapshot().actual == kProfile100);
}

void TestVerifyFaultLatches() {
    Fake f;
    Initialize(MakePlatform(f));
    CHECK(RequestProfile(kProfile100) == Result::Ok);

    f.switchOk = false;  // switch reports failure: intermediate state unknown
    CHECK(TryApply(AllSafe(), MakeHooks(f)) == Result::Io);
    Snapshot s = GetSnapshot();
    CHECK(s.transition == Transition::Fault);
    CHECK(s.actual == kProfileInvalid);
    // Fault latches: further applies are refused until re-init/reset.
    f.switchOk = true;
    CHECK(TryApply(AllSafe(), MakeHooks(f)) == Result::BadState);
}

void TestTimebaseFailureFaults() {
    Fake f;
    Initialize(MakePlatform(f));
    CHECK(RequestProfile(kProfile75) == Result::Ok);
    f.timebaseOk = false;
    CHECK(TryApply(AllSafe(), MakeHooks(f)) == Result::Io);
    CHECK(GetSnapshot().transition == Transition::Fault);  // deadlines unsafe
}

void TestUnlistedBootClockNormalizes() {
    Fake f;
    f.hz = 123000000u;  // not a published profile
    Initialize(MakePlatform(f));
    Snapshot s = GetSnapshot();
    CHECK(s.actual == kProfileInvalid);
    CHECK(s.requested == kProfileBaseline150);
    CHECK(s.transition == Transition::AwaitingSafePoint);
    CHECK(TryApply(AllSafe(), MakeHooks(f)) == Result::Ok);
    CHECK(GetSnapshot().actual == kProfileBaseline150);
}

void TestThermalPolicyValidation() {
    Fake f;
    Initialize(MakePlatform(f));

    ThermalPolicy p;
    p.enabled = true;
    p.throttleOnC = 70;
    p.recoverC = 60;
    p.criticalC = 90;
    p.throttleProfile = kProfile75;
    CHECK(ConfigureThermal(p) == Result::Ok);
    CHECK(GetSnapshot().thermalEnabled);

    // Every invalid shape is rejected and the previous policy retained.
    ThermalPolicy bad = p;
    bad.recoverC = 70;  // hysteresis collapsed (recover !< on)
    CHECK(ConfigureThermal(bad) == Result::InvalidValue);
    bad = p; bad.recoverC = 69;  // below minimum hysteresis
    CHECK(ConfigureThermal(bad) == Result::InvalidValue);
    bad = p; bad.criticalC = 70;  // critical not above throttle-on
    CHECK(ConfigureThermal(bad) == Result::InvalidValue);
    bad = p; bad.throttleOnC = 126;  // outside sensor range
    CHECK(ConfigureThermal(bad) == Result::InvalidValue);
    bad = p; bad.recoverC = -41;
    CHECK(ConfigureThermal(bad) == Result::InvalidValue);
    bad = p; bad.throttleProfile = 9;  // unknown profile
    CHECK(ConfigureThermal(bad) == Result::InvalidValue);
    CHECK(GetSnapshot().thermalEnabled);  // retained
}

void TestThermalOverrideLifecycle() {
    Fake f;
    Initialize(MakePlatform(f));
    CHECK(RequestProfile(kProfile100) == Result::Ok);
    CHECK(TryApply(AllSafe(), MakeHooks(f)) == Result::Ok);
    CHECK(GetSnapshot().actual == kProfile100);

    ThermalPolicy p;
    p.enabled = true;
    p.throttleOnC = 70;
    p.recoverC = 60;
    p.criticalC = 90;
    p.throttleProfile = kProfile75;
    CHECK(ConfigureThermal(p) == Result::Ok);

    // Below threshold: no override.
    CHECK(ThermalSample(65) == Result::Ok);
    CHECK(GetSnapshot().override == Override::None);

    // At threshold: override engages, REQUESTED IS NOT LOST, change deferred.
    CHECK(ThermalSample(70) == Result::Ok);
    Snapshot s = GetSnapshot();
    CHECK(s.override == Override::ThermalThrottle);
    CHECK(s.requested == kProfile100);
    CHECK(EffectiveTarget() == kProfile75);
    CHECK(s.transition == Transition::AwaitingSafePoint);
    CHECK(s.actual == kProfile100);  // not applied yet

    // Hysteresis band: 61..69 retains the override (no flapping).
    CHECK(ThermalSample(69) == Result::Ok);
    CHECK(GetSnapshot().override == Override::ThermalThrottle);
    CHECK(ThermalSample(61) == Result::Ok);
    CHECK(GetSnapshot().override == Override::ThermalThrottle);

    // Routine thermal change goes through the SAME gate.
    f.eventCount = 0;
    SafeGates unsafe = AllSafe();
    unsafe.displayDrained = false;
    CHECK(TryApply(unsafe, MakeHooks(f)) == Result::Busy);
    CHECK(GetSnapshot().actual == kProfile100);
    CHECK(TryApply(AllSafe(), MakeHooks(f)) == Result::Ok);
    CHECK(GetSnapshot().actual == kProfile75);

    // Recovery at/below recoverC: override releases back to the requested
    // profile (again deferred through the gate).
    CHECK(ThermalSample(60) == Result::Ok);
    s = GetSnapshot();
    CHECK(s.override == Override::None);
    CHECK(s.requested == kProfile100);
    CHECK(s.transition == Transition::AwaitingSafePoint);
    CHECK(TryApply(AllSafe(), MakeHooks(f)) == Result::Ok);
    CHECK(GetSnapshot().actual == kProfile100);
}

void TestThermalDisabledNeverActs() {
    Fake f;
    Initialize(MakePlatform(f));
    // No ConfigureThermal: samples are recorded but never act.
    CHECK(ThermalSample(120) == Result::Ok);
    Snapshot s = GetSnapshot();
    CHECK(s.override == Override::None);
    CHECK(s.transition == Transition::Idle);
    CHECK(s.lastTemperatureC == 120);
    CHECK(!s.criticalFaultPending);  // even extreme samples: policy is OFF
}

void TestCriticalFaultThroughParent() {
    Fake f;
    Initialize(MakePlatform(f));
    ThermalPolicy p;
    p.enabled = true;
    p.throttleOnC = 70;
    p.recoverC = 60;
    p.criticalC = 90;
    p.throttleProfile = kProfile75;
    CHECK(ConfigureThermal(p) == Result::Ok);

    // Critical sample: NO clock action mid-waveform — only the parent flag.
    CHECK(ThermalSample(95) == Result::Ok);
    Snapshot s = GetSnapshot();
    CHECK(s.criticalFaultPending);
    CHECK(s.transition == Transition::Idle);
    CHECK(s.actual == kProfileBaseline150);
    CHECK(ConsumeCriticalFault());        // exactly once
    CHECK(!ConsumeCriticalFault());
    ClearCriticalFault();
    CHECK(!GetSnapshot().criticalFaultPending);
}

void TestDisableThermalRestoresRequested() {
    Fake f;
    Initialize(MakePlatform(f));
    CHECK(RequestProfile(kProfile100) == Result::Ok);
    CHECK(TryApply(AllSafe(), MakeHooks(f)) == Result::Ok);

    ThermalPolicy p;
    p.enabled = true;
    p.throttleOnC = 70; p.recoverC = 60; p.criticalC = 90;
    p.throttleProfile = kProfile75;
    CHECK(ConfigureThermal(p) == Result::Ok);
    CHECK(ThermalSample(80) == Result::Ok);
    CHECK(TryApply(AllSafe(), MakeHooks(f)) == Result::Ok);
    CHECK(GetSnapshot().actual == kProfile75);  // throttled

    // Disabling the policy clears the override; requested profile returns
    // through the normal gate.
    ThermalPolicy off;  // enabled = false
    CHECK(ConfigureThermal(off) == Result::Ok);
    Snapshot s = GetSnapshot();
    CHECK(s.override == Override::None);
    CHECK(s.requested == kProfile100);
    CHECK(s.transition == Transition::AwaitingSafePoint);
    CHECK(TryApply(AllSafe(), MakeHooks(f)) == Result::Ok);
    CHECK(GetSnapshot().actual == kProfile100);
}

// ─── Idle wait ──────────────────────────────────────────────────────────────

struct IdleProbe {
    int calls = 0;
    bool pending = false;
};

bool ProbeHasWork(void* ctx) {
    IdleProbe* p = static_cast<IdleProbe*>(ctx);
    ++p->calls;
    return p->pending;
}

void TestSleepUntilEventClosure() {
    IdleProbe p;
    p.pending = true;
    // Work pending: must NOT sleep; predicate consulted exactly once.
    CHECK(!SleepUntilEvent(&p, &ProbeHasWork));
    CHECK(p.calls == 1);
    // Native builds never sleep; the closure contract is predicate-first.
    p.pending = false;
    CHECK(!SleepUntilEvent(&p, &ProbeHasWork));
    CHECK(p.calls == 2);
}

}  // namespace

int main() {
    TestProfileTable();
    TestGateMatrix();
    TestInitAndIdempotent();
    TestUnknownProfileRejected();
    TestBusyGateNeverApplies();
    TestTransitionOrderingDecrease();
    TestTransitionOrderingIncrease();
    TestPrepareFailureRecovery();
    TestVerifyFaultLatches();
    TestTimebaseFailureFaults();
    TestUnlistedBootClockNormalizes();
    TestThermalPolicyValidation();
    TestThermalOverrideLifecycle();
    TestThermalDisabledNeverActs();
    TestCriticalFaultThroughParent();
    TestVoltageSequencingAllProfiles();
    TestUnknownActualNormalizesAt1V2BeforeOverclock();
    TestDomainAndVoltageFailureBoundaries();
    TestClockConfigurationPublication();
    TestDisableThermalRestoresRequested();
    TestSleepUntilEventClosure();

    std::printf("gpu_clock tests: %d checks, %d failures\n", gChecks, gFailures);
    if (gFailures == 0) {
        std::printf("RESULT: PASS\n");
        return 0;
    }
    std::printf("RESULT: FAIL\n");
    return 1;
}
