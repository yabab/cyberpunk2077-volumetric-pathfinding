// vnav - the single logging entry point for the whole plugin.
//
// Every log site in this project routes through Log(). To silence a shipping build, flip
// kLoggingEnabled to false: the format call and the string construction both disappear, because
// the guard is checked before any argument is formatted.
#pragma once

#include <cstdarg>
#include <cstdio>

namespace vnav
{
// OFF by default, which is the shipping state.
//
// Turn it back on with -DVNAV_LOGGING=ON (CMake) rather than by editing this line. The point of
// the switch is that "ship it quiet" and "debug it loud" stop competing for the same constant:
// forgetting to flip it back before release is exactly the mistake the checklist exists to
// prevent, and forgetting to flip it on before a test run wastes a whole session.
//
// Both the snprintf and the argument formatting vanish when this is false - the guard is checked
// before anything is formatted - so an off build pays nothing, not even for building the string.
#ifndef VNAV_LOGGING
#define VNAV_LOGGING 0
#endif

inline constexpr bool kLoggingEnabled = VNAV_LOGGING != 0;

// Implemented in Plugin.cpp, where the RED4ext logger handle lives.
void LogRaw(const char* aMessage);

template<typename... Args>
inline void Log([[maybe_unused]] const char* aFormat, [[maybe_unused]] Args... aArgs)
{
    if constexpr (kLoggingEnabled)
    {
        char buffer[512];
        std::snprintf(buffer, sizeof(buffer), aFormat, aArgs...);
        LogRaw(buffer);
    }
}
} // namespace vnav
