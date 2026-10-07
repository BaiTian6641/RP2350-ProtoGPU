#pragma once
#include <cstddef>
#include <cstdint>
#include <PglRuntimeProtocol.h>
struct SceneState;

namespace CommandParser {
struct BatchInfo {
    uint32_t frameNumber = 0;
    uint32_t frameTimeUs = 0;
    uint16_t commands = 0;
    bool resourceOnly = false;
};

// Core0 only, after prior CPU/conversion readers retire. Entire batch is
// decoded/validated/reserved before persistent metadata changes. Failure frees
// only new reservations and preserves the previous scene and visible output.
PglRuntime::Result Parse(const uint8_t* bytes, size_t length, SceneState* scene,
                         BatchInfo& info, bool resourceOnly = false);
uint16_t GetParserErrorCount();
uint32_t GetParserErrorMask();
void ClearErrors();
} // namespace CommandParser
