#pragma once

#include <cstdint>
#include <PglRuntimeProtocol.h>

#if defined(PICO_ON_DEVICE)
#include "hardware/pio.h"
#endif

// Claims occur on core 0 during initialization or a drained maintenance point.
// IRQs consume existing leases; they never allocate or transfer ownership.
class HardwareResources {
public:
    enum class Owner : uint8_t { None = 0, Host, Display, Devices, Memory, Debug };

    explicit HardwareResources(uint64_t allowedGpios) : allowedGpios_(allowedGpios) {}
    PglRuntime::Result ClaimGpios(uint64_t pins, Owner owner);
    void ReleaseGpios(uint64_t pins, Owner owner);
    bool OwnsGpios(uint64_t pins, Owner owner) const;
    uint64_t ClaimedGpios() const;
    PglRuntime::Result ClaimBus(uint8_t bus, Owner owner);
    void ReleaseBus(uint8_t bus, Owner owner);
    PglRuntime::Result ClaimQmiWindow(uint8_t window, Owner owner);
    void ReleaseQmiWindow(uint8_t window, Owner owner);

#if defined(PICO_ON_DEVICE)
    struct PioLease {
        PIO pio = nullptr;
        const pio_program* program = nullptr;
        uint8_t sm = 0xff, offset = 0xff;
        Owner owner = Owner::None;
    };
    PglRuntime::Result ClaimPio(uint8_t block, const pio_program* program, Owner owner, PioLease& lease);
    void ReleasePio(PioLease& lease);
    PglRuntime::Result ClaimPioIrqs(uint8_t block, uint8_t flags, Owner owner);
    void ReleasePioIrqs(uint8_t block, uint8_t flags, Owner owner);
    PglRuntime::Result ClaimDma(Owner owner, int& channel);
    void ReleaseDma(int& channel, Owner owner);
    // Drivers install shared IRQ handlers and acknowledge only their leased
    // channel/PIO flag. Never install an exclusive DMA IRQ handler.
#endif

private:
    uint64_t allowedGpios_;
    Owner gpioOwners_[48] = {};
    Owner busOwners_[8] = {}; // I2C0/1, SPI0/1, UART0/1, ADC, PWM
    Owner qmiOwners_[2] = {};
#if defined(PICO_ON_DEVICE)
    Owner dmaOwners_[16] = {};
    Owner pioIrqOwners_[3][8] = {};
#endif
};
