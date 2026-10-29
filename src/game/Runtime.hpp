// vnav - process-wide runtime shared by the script bindings and the C API.
#pragma once

#include <cstdint>
#include <deque>
#include <memory>
#include <mutex>
#include <string>
#include <unordered_map>

#include <vnav/NavService.hpp>

#include "game/RedWorldProbe.hpp"

namespace vnav::game
{
// Handles handed to scripts. Async handles are NavService job ids; results produced by the
// synchronous API have no job behind them, so they get ids from a disjoint space.
inline constexpr uint32_t kSyncHandleBit = 0x80000000u;

class Runtime
{
public:
    static Runtime& Get();

    // Called once the game reaches its running state, on the game thread.
    void OnGameRunning();

    // Called on shutdown / world teardown.
    void OnGameShutdown();
    void OnWorldDetached();

    NavService& Nav() { return *m_nav; }
    RedWorldProbe& Probe() { return m_probe; }
    bool IsRunning() const { return m_running; }

    // Game thread. Refreshes the probe, then services the path service for one frame.
    void Pump(int32_t aBudgetMicros);

    // Driven by RED4ext's Running-state OnUpdate, which is a per-frame game-thread callback.
    void PumpNative(int32_t aBudgetMicros);

    // Driven by the redscript shim. Becomes a no-op as soon as the native pump proves it is
    // running, so shipping both drivers cannot double the per-frame sampling budget.
    void PumpFromScript(int32_t aBudgetMicros);

    bool NativePumpActive() const { return m_nativePumpSeen; }

    // Turns a mod-relative path into an absolute one under the game install. Absolute paths are
    // returned unchanged, so a caller can still point somewhere else deliberately.
    std::string ResolveGamePath(const std::string& aRelative) const;

    // --- per-volume query defaults ---------------------------------------------------------
    // Search tuning lives on the volume rather than on every call: a volume already represents
    // one agent profile, and it keeps the script API from needing a dozen parameters per query.
    PathQuery QueryFor(const std::string& aVolume) const;
    void SetQueryDefaults(const std::string& aVolume, const PathQuery& aQuery);

    // --- script-visible results ------------------------------------------------------------
    // Scripts read paths back by handle + index rather than by marshalling an array across the
    // RTTI boundary. Latched results also outlive the job, so a caller that polls late still
    // gets its path instead of Status::Invalid.
    uint32_t StoreSyncResult(const PathResult& aResult);
    void LatchResult(uint32_t aHandle, const PathResult& aResult);
    bool GetResult(uint32_t aHandle, PathResult& aOut) const;
    void ReleaseResult(uint32_t aHandle);

    // Latched result first, then a live job (latching it on the way through). Returns false
    // while a job is still running, with aOut.status set to the in-flight status.
    bool ResolveResult(uint32_t aHandle, PathResult& aOut);

private:
    Runtime() = default;

    void TrimResults();

    std::unique_ptr<NavService> m_nav;
    RedWorldProbe m_probe;
    bool m_running = false;
    bool m_nativePumpSeen = false;

    mutable std::mutex m_mutex;
    std::unordered_map<std::string, PathQuery> m_queryDefaults;
    std::unordered_map<uint32_t, PathResult> m_results;
    std::deque<uint32_t> m_resultOrder;
    uint32_t m_nextSyncHandle = 1;

    // Results are latched on completion, so this bounds how many finished paths a forgetful
    // script can keep alive.
    static constexpr size_t kMaxStoredResults = 256;
};
} // namespace vnav::game
