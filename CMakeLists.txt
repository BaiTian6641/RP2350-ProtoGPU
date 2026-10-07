cmake_minimum_required(VERSION 3.20)

if(NOT PICO_SDK_PATH AND NOT DEFINED ENV{PICO_SDK_PATH})
    set(PICO_SDK_PATH "${CMAKE_CURRENT_SOURCE_DIR}/third_party/pico-sdk")
endif()
set(PICO_PLATFORM rp2350-arm-s CACHE STRING "RP2350 Arm platform")
include(pico_sdk_import.cmake)
project(protogl_gpu C CXX ASM)
set(CMAKE_C_STANDARD 11)
set(CMAKE_CXX_STANDARD 17)
set(CMAKE_EXPORT_COMPILE_COMMANDS ON)

set(BOOT_STORAGE FLASH_LOCAL CACHE STRING "RAM_HOST or FLASH_LOCAL")
set_property(CACHE BOOT_STORAGE PROPERTY STRINGS RAM_HOST FLASH_LOCAL)
set(DEFAULT_DISPLAY SPI CACHE STRING "SPI, HUB, LED, CUSTOM, or NONE")
set_property(CACHE DEFAULT_DISPLAY PROPERTY STRINGS SPI HUB LED CUSTOM NONE)
set(PSRAM OFF CACHE STRING "Selected-CS QMI optional asset RAM")
set_property(CACHE PSRAM PROPERTY STRINGS OFF ON)
option(HOSTLESS_DEMO "Built-in graphics-command cube diagnostic" OFF)

if(NOT BOOT_STORAGE MATCHES "^(RAM_HOST|FLASH_LOCAL)$")
    message(FATAL_ERROR "Invalid BOOT_STORAGE=${BOOT_STORAGE}")
endif()
if(NOT PSRAM MATCHES "^(OFF|ON)$")
    message(FATAL_ERROR "PSRAM must be OFF or ON")
endif()
if(DEFAULT_DISPLAY STREQUAL SPI)
    set(DISPLAY_ID 3)
elseif(DEFAULT_DISPLAY STREQUAL HUB)
    set(DISPLAY_ID 1)
elseif(DEFAULT_DISPLAY STREQUAL LED)
    set(DISPLAY_ID 6)
elseif(DEFAULT_DISPLAY STREQUAL CUSTOM)
    set(DISPLAY_ID 7)
elseif(DEFAULT_DISPLAY STREQUAL NONE)
    set(DISPLAY_ID 0)
else()
    message(FATAL_ERROR "Unknown DEFAULT_DISPLAY=${DEFAULT_DISPLAY}")
endif()
if(PSRAM STREQUAL ON AND DEFAULT_DISPLAY MATCHES "^(HUB|CUSTOM)$")
    message(FATAL_ERROR "Reference ${DEFAULT_DISPLAY} routing consumes QMI CS8; choose a nonconflicting board/output profile")
endif()

pico_sdk_init()
find_package(Python3 REQUIRED COMPONENTS Interpreter)
add_executable(protogl_gpu
    src/main.cpp src/gpu_core.cpp src/command_parser.cpp src/gpu_clock.cpp
    src/hardware_resources.cpp
    src/phase_scratch.cpp
    src/display/display_manager.cpp src/display/hub75_driver.cpp
    src/display/spi_display.cpp src/display/led_array.cpp src/display/custom_array.cpp
    src/transport/pgl_spi_target.cpp src/devices/device_service.cpp
    src/math/pgl_math.cpp src/render/rasterizer.cpp src/render/triangle2d.cpp
    src/render/rasterizer_2d.cpp src/render/pgl_shader_vm.cpp
    src/render/screenspace_effects.cpp src/render/frame_renderer.cpp
    src/scheduler/pgl_tile_scheduler.cpp src/scheduler/pgl_scheduler_rp2350.cpp
    src/memory/mem_assets.cpp src/memory/scene_assets.cpp
    ProtoGC/src/pgc_pico.cpp
)
if(PSRAM STREQUAL ON)
    target_sources(protogl_gpu PRIVATE src/memory/mem_qmi_psram.cpp)
    target_link_libraries(protogl_gpu hardware_psram hardware_flash hardware_xip_cache)
endif()
set(PGL_GEN "${CMAKE_BINARY_DIR}/generated")
file(MAKE_DIRECTORY "${PGL_GEN}")
add_custom_target(pgl_build_identity ALL
    COMMAND ${Python3_EXECUTABLE} ${CMAKE_SOURCE_DIR}/tools/pgl_build_identity.py
        --repo ${CMAKE_SOURCE_DIR} --boot-storage ${BOOT_STORAGE} --display ${DEFAULT_DISPLAY}
        --psram ${PSRAM} --diagnostic $<IF:$<BOOL:${HOSTLESS_DEMO}>,ON,OFF>
        --header ${PGL_GEN}/pgl_build_identity.h --json ${PGL_GEN}/pgl_build_identity.json
    BYPRODUCTS ${PGL_GEN}/pgl_build_identity.h ${PGL_GEN}/pgl_build_identity.json
    VERBATIM)
add_dependencies(protogl_gpu pgl_build_identity)
target_include_directories(protogl_gpu PRIVATE src ProtoGL/src ProtoGC/src ${PGL_GEN})
target_link_libraries(protogl_gpu
    pico_stdlib pico_multicore hardware_pio hardware_dma hardware_gpio
    hardware_i2c hardware_adc hardware_clocks hardware_uart hardware_pll hardware_vreg hardware_ticks)
target_compile_definitions(protogl_gpu PRIVATE
    PROTOGPU_RAM_HOST=$<STREQUAL:${BOOT_STORAGE},RAM_HOST>
    PROTOGPU_ENABLE_PSRAM=$<STREQUAL:${PSRAM},ON>
    PROTOGPU_DEFAULT_DISPLAY=${DISPLAY_ID}
    PROTOGPU_HOSTLESS_DEMO=$<BOOL:${HOSTLESS_DEMO}>
    PGC_BACKEND_PICO PROTOGC_OVERRIDE_NEW=0
    PICO_DEFAULT_UART=0 PICO_DEFAULT_UART_TX_PIN=28 PICO_DEFAULT_UART_RX_PIN=-1
    PICO_RUNTIME_SKIP_INIT_PSRAM=1 PICO_RUNTIME_NO_INIT_PSRAM=1
    PICO_AUTO_DETECT_PSRAM=0 PICO_AUTO_DETECT_PSRAM_CS=0 PICO_AUTO_DETECT_PSRAM_SIZE=0
    PICO_STACK_SIZE=4096 PICO_CORE1_STACK_SIZE=4096)
if(BOOT_STORAGE STREQUAL RAM_HOST)
    pico_set_binary_type(protogl_gpu no_flash)
else()
    pico_set_binary_type(protogl_gpu default)
endif()
foreach(PROGRAM IN ITEMS
    src/display/hub75.pio src/display/spi_display.pio
    src/display/led_array.pio src/display/custom_array.pio src/transport/pgl_spi_target.pio)
    pico_generate_pio_header(protogl_gpu ${CMAKE_SOURCE_DIR}/${PROGRAM})
endforeach()
pico_enable_stdio_usb(protogl_gpu 0)
pico_enable_stdio_uart(protogl_gpu 1)
pico_set_program_name(protogl_gpu "ProtoGPU runtime")
pico_set_program_version(protogl_gpu "9.0")
target_compile_options(protogl_gpu PRIVATE -Wall -Wextra
    $<IF:$<STREQUAL:${BOOT_STORAGE},RAM_HOST>,-Os,-O2>
    -ffunction-sections -fdata-sections -flto)
target_link_options(protogl_gpu PRIVATE -flto -Wl,--gc-sections)
# SDK --wrap entrypoints need ordinary symbols until final link, not LTO-only definitions.
# The SDK's naked __aeabi_read_tp uses assembler literal-pool loads. Keep its
# translation unit separate: LTO can move the pool beyond Thumb load reach.
set_source_files_properties(
    ${PICO_SDK_PATH}/src/rp2_common/pico_stdio/stdio.c
    ${PICO_SDK_PATH}/src/rp2_common/pico_printf/printf.c
    ${PICO_SDK_PATH}/src/rp2_common/pico_float/float_math.c
    ${PICO_SDK_PATH}/src/rp2_common/pico_double/double_math.c
    ${PICO_SDK_PATH}/src/rp2_common/pico_malloc/malloc.c
    ${PICO_SDK_PATH}/src/rp2_common/pico_thread_local/thread_local.c
    PROPERTIES COMPILE_OPTIONS -fno-lto)
pico_add_extra_outputs(protogl_gpu)
add_custom_command(TARGET protogl_gpu POST_BUILD
    COMMAND ${Python3_EXECUTABLE} ${CMAKE_SOURCE_DIR}/tools/pgl_image_package.py
        --elf $<TARGET_FILE:protogl_gpu> --bin ${CMAKE_BINARY_DIR}/protogl_gpu.bin
        --map ${CMAKE_BINARY_DIR}/protogl_gpu.elf.map --identity ${PGL_GEN}/pgl_build_identity.json
        --boot-storage ${BOOT_STORAGE} --manifest ${CMAKE_BINARY_DIR}/pgl_image_manifest.json
        --manifest-bin ${CMAKE_BINARY_DIR}/pgl_image_manifest.bin
        --manifest-header ${PGL_GEN}/pgl_image_manifest.h --report ${CMAKE_BINARY_DIR}/pgl_image_report.txt
    COMMAND ${Python3_EXECUTABLE} ${CMAKE_SOURCE_DIR}/tools/pgl_budget_report.py
        --elf $<TARGET_FILE:protogl_gpu> --map ${CMAKE_BINARY_DIR}/protogl_gpu.elf.map
        --identity ${PGL_GEN}/pgl_build_identity.json --boot-storage ${BOOT_STORAGE}
        --json ${CMAKE_BINARY_DIR}/pgl_budget.json --report ${CMAKE_BINARY_DIR}/pgl_budget.txt
    VERBATIM)
