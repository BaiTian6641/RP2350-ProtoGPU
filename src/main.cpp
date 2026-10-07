#include "gpu_core.h"
#include "gpu_clock.h"
#include "pico/stdlib.h"
#include "hardware/clocks.h"
#include "hardware/watchdog.h"
#include "pico/runtime_init.h"
#include "pgl_build_identity.h"
#include <cstdio>

// Volatile keeps this authoritative, externally named value in the final ELF;
// the image packager verifies it against the deterministic identity manifest.
extern "C" const volatile uint32_t pgl_build_id = PGL_FIRMWARE_BUILD_ID;

// The SDK's default runtime clock init sources clk_peri and HSTX from clk_sys.
// UART/SPI peripherals must stay on the independent PLL_USB 48 MHz domain while
// clk_sys varies 75..336 MHz; HSTX is unused but remains explicitly fixed.
void ConfigureFixedPeripheralClocks() {
    clock_configure_undivided(clk_peri, 0,
        CLOCKS_CLK_PERI_CTRL_AUXSRC_VALUE_CLKSRC_PLL_USB, 48000000u);
    clock_configure_undivided(clk_hstx, 0,
        CLOCKS_CLK_HSTX_CTRL_AUXSRC_VALUE_CLKSRC_PLL_USB, 48000000u);
}
PICO_RUNTIME_INIT_FUNC(ConfigureFixedPeripheralClocks, "00501");

int main() {
    stdio_init_all();
    GpuClock::Initialize();
    std::printf("ProtoGPU protocol9 build=%08lx profile=%s\n", static_cast<unsigned long>(pgl_build_id), PGL_BUILD_PROFILE);
    if (!GpuCore::Initialize()) {
        std::printf("ProtoGPU initialization failed; outputs remain safe\n");
        watchdog_reboot(0, 0, 100);
        for (;;) tight_loop_contents();
    }
    GpuCore::Core0Main();
}
