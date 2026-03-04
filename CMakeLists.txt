cmake_minimum_required(VERSION 3.20)

# ─── Pico SDK import (must be before project()) ─────────────────────────────
include(pico_sdk_import.cmake)

project(protogl_gpu C CXX ASM)

set(CMAKE_C_STANDARD 11)
set(CMAKE_CXX_STANDARD 17)

# ─── Initialize the Pico SDK ────────────────────────────────────────────────
pico_sdk_init()

# ─── ProtoGL shared headers (wire-format types, parser, CRC) ────────────────
set(PROTOGL_SHARED_DIR "${CMAKE_CURRENT_SOURCE_DIR}/../../src")

# ─── Source files ────────────────────────────────────────────────────────────
add_executable(protogl_gpu
    src/main.cpp
    src/gpu_core.cpp
    src/gpu_clock.cpp
    src/command_parser.cpp
    src/display/hub75_driver.cpp
    src/transport/octal_spi_rx.cpp
    src/transport/i2c_slave.cpp
    src/math/pgl_math.cpp
    src/render/rasterizer.cpp
    src/render/quadtree.cpp
    src/render/triangle2d.cpp
    src/render/screenspace_effects.cpp
    src/render/pgl_shader_vm.cpp
    src/scheduler/pgl_job_scheduler_rp2350.cpp
    src/scheduler/pgl_tile_scheduler.cpp
    src/memory/mem_opi_psram.cpp
    src/memory/mem_qspi_psram.cpp
    src/memory/mem_tier.cpp
    src/selftest/gpu_selftest.cpp
)

target_include_directories(protogl_gpu PRIVATE
    src
    ${PROTOGL_SHARED_DIR}
)

# ─── Pico SDK libraries ─────────────────────────────────────────────────────
target_link_libraries(protogl_gpu
    pico_stdlib
    pico_multicore
    pico_i2c_slave
    hardware_pio
    hardware_dma
    hardware_i2c
    hardware_gpio
    hardware_clocks    hardware_adc
    hardware_xip_cache)

# ─── PIO programs ───────────────────────────────────────────────────────────
pico_generate_pio_header(protogl_gpu
    ${CMAKE_CURRENT_SOURCE_DIR}/src/display/hub75.pio
)
pico_generate_pio_header(protogl_gpu
    ${CMAKE_CURRENT_SOURCE_DIR}/src/transport/octal_spi_rx.pio
)

# ─── Serial output for debugging ────────────────────────────────────────────
# UART0 TX on GPIO 16, RX on GPIO 17.
# HUB75 is disabled, so those pins are free for UART.
# USB stdio disabled — USB CDC caused processor hangs during PIO/DMA init.
pico_enable_stdio_usb(protogl_gpu 0)
pico_enable_stdio_uart(protogl_gpu 1)

target_compile_definitions(protogl_gpu PRIVATE
    PICO_DEFAULT_UART=0
    PICO_DEFAULT_UART_TX_PIN=16
    PICO_DEFAULT_UART_RX_PIN=17
)

# ─── Generate UF2 for flashing ──────────────────────────────────────────────
pico_add_extra_outputs(protogl_gpu)

# ─── Compiler warnings ──────────────────────────────────────────────────────
target_compile_options(protogl_gpu PRIVATE
    -Wall
    -Wextra
    -Wno-unused-parameter
    -O2
    -ffast-math            # Enables VFMA.F32 contraction from a*b+c patterns,
                           # reciprocal sqrt approx, no-errno — safe for renderer.
    -ffunction-sections    # Place each function in its own section for linker GC
    -fdata-sections        # Place each data item in its own section for linker GC
)

# Remove unused code/data sections
target_link_options(protogl_gpu PRIVATE
    -Wl,--gc-sections
)
