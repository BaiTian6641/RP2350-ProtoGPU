/**
 * @file mem_qspi_psram.h
 * @brief LEGACY compatibility shim — superseded by mem_qspi_vram.h.
 *
 * The old QMI CS1 XIP driver class was removed; external VRAM is unified in
 * QspiVramDriver (dual-channel PIO2, MRAM/PSRAM auto-detect).  This header
 * remains only so existing includers resolve the driver alias:
 *
 *   using QspiPsramDriver = QspiVramDriver;   // defined in mem_qspi_vram.h
 *
 * The former implementation (mem_qspi_psram.cpp) is not compiled (not in
 * CMake) and will be deleted in a future hygiene pass.
 */

#pragma once

#include "mem_qspi_vram.h"
