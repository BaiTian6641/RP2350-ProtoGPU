/**
 * @file perf_counters.h
 * @brief Lightweight per-stage cycle/microsecond profiling using Cortex-M33 DWT.
 *
 * Usage:
 *   PerfCounters::Initialize();              // once at startup
 *   PerfCounters::Begin(PerfStage::Parse);
 *   ... do work ...
 *   PerfCounters::End(PerfStage::Parse);
 *   PerfCounters::PrintReport();             // dumps all stages to UART
 *
 * DWT->CYCCNT is a free-running 32-bit hardware cycle counter on Cortex-M33.
 * At 150 MHz it wraps every ~28.6 seconds — more than enough for per-frame
 * measurements.
 *
 * Milestone: M5 Week 14 (Profiling & Cortex-M33 Optimization).
 */

#pragma once

#include <cstdint>
#include <cstdio>

#include "../gpu_config.h"

// Cortex-M33 Debug Watchpoint and Trace registers
#ifndef DWT_BASE
#define DWT_BASE   (0xE0001000UL)
#endif
#ifndef CoreDebug_BASE
#define CoreDebug_BASE (0xE000EDF0UL)
#endif

#define DWT_CTRL_REG        (*(volatile uint32_t*)(DWT_BASE + 0x000))
#define DWT_CYCCNT_REG      (*(volatile uint32_t*)(DWT_BASE + 0x004))
#define CoreDebug_DEMCR_REG (*(volatile uint32_t*)(CoreDebug_BASE + 0x000))

#define DWT_CTRL_CYCCNTENA  (1UL << 0)
#define DEMCR_TRCENA        (1UL << 24)

// ─── Profiling Stages ───────────────────────────────────────────────────────

enum class PerfStage : uint8_t {
    SpiReceive    = 0,  // Ring buffer drain → frame extraction
    Parse         = 1,  // Command parser
    Transform     = 2,  // PrepareFrame: vertex transform + project + QuadTree build
    RasterTop     = 3,  // Core 0 RasterizeRange (top half)
    RasterBottom  = 4,  // Core 1 RasterizeRange (bottom half)
    Shaders       = 5,  // Screen-space post-processing
    Swap          = 6,  // Framebuffer swap + I2C status update
    FrameTotal    = 7,  // Full frame (SPI receive → swap)
    Count         = 8   // sentinel
};

static constexpr const char* PerfStageNames[] = {
    "SPI Recv",
    "Parse",
    "Transform",
    "Raster Top",
    "Raster Bot",
    "Shaders",
    "Swap",
    "Frame Total"
};

// ─── Per-Stage Accumulator ──────────────────────────────────────────────────

struct PerfAccumulator {
    uint32_t lastCycles   = 0;   // most recent measurement (cycles)
    uint32_t minCycles    = 0xFFFFFFFF;
    uint32_t maxCycles    = 0;
    uint64_t totalCycles  = 0;   // accumulated over all samples
    uint32_t sampleCount  = 0;

    uint32_t startCycle   = 0;   // scratch: cycle count at Begin()

    void Reset() {
        lastCycles  = 0;
        minCycles   = 0xFFFFFFFF;
        maxCycles   = 0;
        totalCycles = 0;
        sampleCount = 0;
        startCycle  = 0;
    }

    void Record(uint32_t cycles) {
        lastCycles = cycles;
        totalCycles += cycles;
        if (cycles < minCycles) minCycles = cycles;
        if (cycles > maxCycles) maxCycles = cycles;
        sampleCount++;
    }

    uint32_t AverageUs() const {
        if (sampleCount == 0) return 0;
        uint64_t avgCycles = totalCycles / sampleCount;
        return static_cast<uint32_t>(avgCycles / (GpuConfig::SYSTEM_CLOCK_MHZ));
    }

    uint32_t LastUs() const {
        return lastCycles / GpuConfig::SYSTEM_CLOCK_MHZ;
    }

    uint32_t MinUs() const {
        return minCycles / GpuConfig::SYSTEM_CLOCK_MHZ;
    }

    uint32_t MaxUs() const {
        return maxCycles / GpuConfig::SYSTEM_CLOCK_MHZ;
    }
};

// ─── Profiler Singleton ─────────────────────────────────────────────────────

namespace PerfCounters {

inline PerfAccumulator stages[static_cast<uint8_t>(PerfStage::Count)];

/// Enable DWT cycle counter.  Call once from Core 0 before main loop.
inline void Initialize() {
    CoreDebug_DEMCR_REG |= DEMCR_TRCENA;    // enable trace block
    DWT_CYCCNT_REG = 0;                       // reset counter
    DWT_CTRL_REG |= DWT_CTRL_CYCCNTENA;      // start counting

    for (auto& s : stages) s.Reset();
    printf("[Perf] DWT cycle counter enabled @ %u MHz\n",
           GpuConfig::SYSTEM_CLOCK_MHZ);
}

/// Mark the beginning of a profiling stage.
inline void Begin(PerfStage stage) {
    stages[static_cast<uint8_t>(stage)].startCycle = DWT_CYCCNT_REG;
}

/// Mark the end of a profiling stage and record elapsed cycles.
inline void End(PerfStage stage) {
    uint32_t now = DWT_CYCCNT_REG;
    PerfAccumulator& acc = stages[static_cast<uint8_t>(stage)];
    uint32_t elapsed = now - acc.startCycle;  // handles wrap via unsigned arithmetic
    acc.Record(elapsed);
}

/// Print a full timing report to UART.  Call periodically (e.g. once/second).
inline void PrintReport() {
    printf("[Perf] ──────────────────────────────────────────────\n");
    printf("[Perf] Stage           Last(µs)  Avg(µs)  Min(µs)  Max(µs)  N\n");
    for (uint8_t i = 0; i < static_cast<uint8_t>(PerfStage::Count); ++i) {
        const PerfAccumulator& acc = stages[i];
        if (acc.sampleCount == 0) continue;
        printf("[Perf] %-14s %6u    %6u    %6u    %6u   %u\n",
               PerfStageNames[i],
               acc.LastUs(), acc.AverageUs(),
               acc.MinUs(), acc.MaxUs(),
               acc.sampleCount);
    }
    printf("[Perf] ──────────────────────────────────────────────\n");
}

/// Reset all accumulators.  Call after printing report to start a fresh window.
inline void ResetAll() {
    for (auto& s : stages) s.Reset();
}

}  // namespace PerfCounters
