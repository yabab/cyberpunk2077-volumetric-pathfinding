#include "game/Runtime.hpp"

#include <algorithm>
#include <filesystem>

#include <Windows.h>

namespace vnav::game
{
Runtime& Runtime::Get()
{
    static Runtime s;
    return s;
}

void Runtime::OnGameRunning()
{
    if (m_running)
    {
        return;
    }

    if (!m_nav)
    {
        m_nav = std::make_unique<NavService>();
    }

    // Whichever thread reaches the running state is the thread that will pump, and it is the
    // only thread allowed to touch the physics scene. FindPathSync keys off this.
    m_nav->SetGameThread();

    m_probe.Initialize();
    m_nav->SetProbe(&m_probe);
    m_running = true;
}

void Runtime::OnWorldDetached()
{
    if (!m_nav)
    {
        return;
    }

    // Every cached chunk describes geometry that just went away.
    m_nav->InvalidateAllVolumes();
    m_probe.Invalidate();
}

void Runtime::OnGameShutdown()
{
    m_running = false;

    if (m_nav)
    {
        // Detach the probe before tearing down: workers may still be finishing, and the probe
        // must not be reachable once the game is going away.
        m_nav->SetProbe(nullptr);
        m_nav->Shutdown();
        m_nav.reset();
    }
    m_probe.Invalidate();

    std::lock_guard lock(m_mutex);
    m_results.clear();
    m_resultOrder.clear();
    m_queryDefaults.clear();
}

void Runtime::Pump(int32_t aBudgetMicros)
{
    if (!m_running || !m_nav)
    {
        return;
    }

    // Cheap, and it keeps the system pointer correct across world transitions without paying a
    // lookup on every one of the hundreds of probes a frame can issue.
    m_probe.RefreshFrame();
    m_nav->Pump(aBudgetMicros);
}

std::string Runtime::ResolveGamePath(const std::string& aRelative) const
{
    if (aRelative.size() > 1 && (aRelative[1] == ':' || aRelative[0] == '\\' || aRelative[0] == '/'))
    {
        return aRelative; // already absolute
    }

    // The running module is <root>\bin\x64\Cyberpunk2077.exe, so the root is three levels up.
    wchar_t buffer[MAX_PATH] = {};
    const DWORD length = GetModuleFileNameW(nullptr, buffer, MAX_PATH);
    if (length == 0 || length >= MAX_PATH)
    {
        return aRelative;
    }

    std::filesystem::path root = std::filesystem::path(buffer).parent_path().parent_path().parent_path();
    return (root / aRelative).string();
}

void Runtime::PumpNative(int32_t aBudgetMicros)
{
    m_nativePumpSeen = true;
    Pump(aBudgetMicros);
}

void Runtime::PumpFromScript(int32_t aBudgetMicros)
{
    // The native Running-state OnUpdate is the preferred driver. The redscript shim ships as a
    // fallback for the case where it turns out not to fire, and stands down once it does.
    if (m_nativePumpSeen)
    {
        return;
    }
    Pump(aBudgetMicros);
}

PathQuery Runtime::QueryFor(const std::string& aVolume) const
{
    std::lock_guard lock(m_mutex);
    auto it = m_queryDefaults.find(aVolume);
    return it == m_queryDefaults.end() ? PathQuery{} : it->second;
}

void Runtime::SetQueryDefaults(const std::string& aVolume, const PathQuery& aQuery)
{
    std::lock_guard lock(m_mutex);
    m_queryDefaults[aVolume] = aQuery;
}

uint32_t Runtime::StoreSyncResult(const PathResult& aResult)
{
    std::lock_guard lock(m_mutex);
    const uint32_t handle = kSyncHandleBit | (m_nextSyncHandle++);
    m_results[handle] = aResult;
    m_resultOrder.push_back(handle);
    TrimResults();
    return handle;
}

void Runtime::LatchResult(uint32_t aHandle, const PathResult& aResult)
{
    std::lock_guard lock(m_mutex);
    if (m_results.find(aHandle) == m_results.end())
    {
        m_resultOrder.push_back(aHandle);
    }
    m_results[aHandle] = aResult;
    TrimResults();
}

bool Runtime::GetResult(uint32_t aHandle, PathResult& aOut) const
{
    std::lock_guard lock(m_mutex);
    auto it = m_results.find(aHandle);
    if (it == m_results.end())
    {
        return false;
    }
    aOut = it->second;
    return true;
}

bool Runtime::ResolveResult(uint32_t aHandle, PathResult& aOut)
{
    if (GetResult(aHandle, aOut))
    {
        return true;
    }

    // Sync handles never have a job behind them, so a miss above is final.
    if ((aHandle & kSyncHandleBit) != 0 || !m_running || !m_nav)
    {
        aOut = PathResult{};
        return false;
    }

    const Status s = m_nav->Poll(aHandle, aOut);
    if (IsTerminal(s))
    {
        // Latch it so the caller can still read the path after the job is reaped.
        LatchResult(aHandle, aOut);
        return true;
    }

    aOut = PathResult{};
    aOut.status = s;
    return false;
}

void Runtime::ReleaseResult(uint32_t aHandle)
{
    std::lock_guard lock(m_mutex);
    m_results.erase(aHandle);
    m_resultOrder.erase(std::remove(m_resultOrder.begin(), m_resultOrder.end(), aHandle), m_resultOrder.end());
}

void Runtime::TrimResults()
{
    // Caller holds m_mutex.
    while (m_resultOrder.size() > kMaxStoredResults)
    {
        m_results.erase(m_resultOrder.front());
        m_resultOrder.pop_front();
    }
}
} // namespace vnav::game
