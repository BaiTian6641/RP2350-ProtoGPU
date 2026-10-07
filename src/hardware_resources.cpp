#include "hardware_resources.h"

#if defined(PICO_ON_DEVICE)
#include "hardware/dma.h"
#endif

PglRuntime::Result HardwareResources::ClaimGpios(uint64_t pins, Owner owner) {
    if (!pins || owner == Owner::None || (pins & ~allowedGpios_)) return PglRuntime::Result::InvalidValue;
    for (uint8_t pin = 0; pin < 48; ++pin) {
        if ((pins & (uint64_t(1) << pin)) && gpioOwners_[pin] != Owner::None) return PglRuntime::Result::Conflict;
    }
    for (uint8_t pin = 0; pin < 48; ++pin) if (pins & (uint64_t(1) << pin)) gpioOwners_[pin] = owner;
    return PglRuntime::Result::Ok;
}

void HardwareResources::ReleaseGpios(uint64_t pins, Owner owner) {
    for (uint8_t pin = 0; pin < 48; ++pin) {
        if ((pins & (uint64_t(1) << pin)) && gpioOwners_[pin] == owner) gpioOwners_[pin] = Owner::None;
    }
}

bool HardwareResources::OwnsGpios(uint64_t pins, Owner owner) const {
    if (owner == Owner::None || (pins & ~allowedGpios_)) return false;
    for (uint8_t pin = 0; pin < 48; ++pin) {
        if ((pins & (uint64_t(1) << pin)) && gpioOwners_[pin] != owner) return false;
    }
    return true;
}

uint64_t HardwareResources::ClaimedGpios() const {
    uint64_t mask = 0;
    for (uint8_t pin = 0; pin < 48; ++pin) if (gpioOwners_[pin] != Owner::None) mask |= uint64_t(1) << pin;
    return mask;
}

PglRuntime::Result HardwareResources::ClaimBus(uint8_t bus, Owner owner) {
    if (bus >= 8 || owner == Owner::None) return PglRuntime::Result::InvalidValue;
    if (busOwners_[bus] != Owner::None) return PglRuntime::Result::Conflict;
    busOwners_[bus] = owner;
    return PglRuntime::Result::Ok;
}

void HardwareResources::ReleaseBus(uint8_t bus, Owner owner) {
    if (bus < 8 && busOwners_[bus] == owner) busOwners_[bus] = Owner::None;
}

PglRuntime::Result HardwareResources::ClaimQmiWindow(uint8_t window, Owner owner) {
    if (window >= 2 || owner == Owner::None) return PglRuntime::Result::InvalidValue;
    if (qmiOwners_[window] != Owner::None) return PglRuntime::Result::Conflict;
    qmiOwners_[window] = owner;
    return PglRuntime::Result::Ok;
}

void HardwareResources::ReleaseQmiWindow(uint8_t window, Owner owner) {
    if (window < 2 && qmiOwners_[window] == owner) qmiOwners_[window] = Owner::None;
}

#if defined(PICO_ON_DEVICE)
PglRuntime::Result HardwareResources::ClaimPio(uint8_t block, const pio_program* program, Owner owner, PioLease& lease) {
    if (block >= 3 || !program || !program->length || program->length > 32 || owner == Owner::None || lease.pio) return PglRuntime::Result::InvalidValue;
    PIO pio = block == 0 ? pio0 : block == 1 ? pio1 : pio2;
    if (!pio_can_add_program(pio, program)) return PglRuntime::Result::Capacity;
    int sm = pio_claim_unused_sm(pio, false);
    if (sm < 0) return PglRuntime::Result::Capacity;
    int offset = pio_add_program(pio, program);
    if (offset < 0) {
        pio_sm_unclaim(pio, sm);
        return PglRuntime::Result::Capacity;
    }
    lease.pio = pio; lease.program = program; lease.sm = static_cast<uint8_t>(sm);
    lease.offset = static_cast<uint8_t>(offset); lease.owner = owner;
    return PglRuntime::Result::Ok;
}

void HardwareResources::ReleasePio(PioLease& lease) {
    if (!lease.pio) return;
    pio_sm_set_enabled(lease.pio, lease.sm, false);
    pio_sm_clear_fifos(lease.pio, lease.sm);
    pio_remove_program(lease.pio, lease.program, lease.offset);
    pio_sm_unclaim(lease.pio, lease.sm);
    lease = {};
}

PglRuntime::Result HardwareResources::ClaimPioIrqs(uint8_t block, uint8_t flags, Owner owner) {
    if (block >= 3 || !flags || owner == Owner::None) return PglRuntime::Result::InvalidValue;
    for (uint8_t bit = 0; bit < 8; ++bit) if ((flags & (1u << bit)) && pioIrqOwners_[block][bit] != Owner::None) return PglRuntime::Result::Conflict;
    for (uint8_t bit = 0; bit < 8; ++bit) if (flags & (1u << bit)) pioIrqOwners_[block][bit] = owner;
    return PglRuntime::Result::Ok;
}

void HardwareResources::ReleasePioIrqs(uint8_t block, uint8_t flags, Owner owner) {
    if (block >= 3) return;
    for (uint8_t bit = 0; bit < 8; ++bit) if ((flags & (1u << bit)) && pioIrqOwners_[block][bit] == owner) pioIrqOwners_[block][bit] = Owner::None;
}

PglRuntime::Result HardwareResources::ClaimDma(Owner owner, int& channel) {
    if (owner == Owner::None || channel >= 0) return PglRuntime::Result::InvalidValue;
    int candidate = dma_claim_unused_channel(false);
    if (candidate < 0) return PglRuntime::Result::Capacity;
    dmaOwners_[candidate] = owner;
    channel = candidate;
    return PglRuntime::Result::Ok;
}

void HardwareResources::ReleaseDma(int& channel, Owner owner) {
    if (channel < 0 || channel >= 16 || dmaOwners_[channel] != owner) return;
    // These leases do not permit chaining into a different owner's channel.
    // Drivers disable their entire owned chain before releasing any member.
    dma_channel_cleanup(channel);
    dma_channel_unclaim(channel);
    dmaOwners_[channel] = Owner::None;
    channel = -1;
}
#endif
