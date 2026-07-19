# Distributed under the OSI-approved BSD 3-Clause License.  See accompanying
# file Copyright.txt or https://cmake.org/licensing for details.

cmake_minimum_required(VERSION 3.5)

file(MAKE_DIRECTORY
  "/home/polar/MCUGPU/RP2350-ProtoGPU/build-headless/_deps/picotool-src"
  "/home/polar/MCUGPU/RP2350-ProtoGPU/build-headless/_deps/picotool-build"
  "/home/polar/MCUGPU/RP2350-ProtoGPU/build-headless/_deps"
  "/home/polar/MCUGPU/RP2350-ProtoGPU/build-headless/pico-sdk/src/rp2350/boot_stage2/picotool/tmp"
  "/home/polar/MCUGPU/RP2350-ProtoGPU/build-headless/pico-sdk/src/rp2350/boot_stage2/picotool/src/picotoolBuild-stamp"
  "/home/polar/MCUGPU/RP2350-ProtoGPU/build-headless/pico-sdk/src/rp2350/boot_stage2/picotool/src"
  "/home/polar/MCUGPU/RP2350-ProtoGPU/build-headless/pico-sdk/src/rp2350/boot_stage2/picotool/src/picotoolBuild-stamp"
)

set(configSubDirs )
foreach(subDir IN LISTS configSubDirs)
    file(MAKE_DIRECTORY "/home/polar/MCUGPU/RP2350-ProtoGPU/build-headless/pico-sdk/src/rp2350/boot_stage2/picotool/src/picotoolBuild-stamp/${subDir}")
endforeach()
if(cfgdir)
  file(MAKE_DIRECTORY "/home/polar/MCUGPU/RP2350-ProtoGPU/build-headless/pico-sdk/src/rp2350/boot_stage2/picotool/src/picotoolBuild-stamp${cfgdir}") # cfgdir has leading slash
endif()
