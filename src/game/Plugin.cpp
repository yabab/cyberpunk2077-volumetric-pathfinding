// vnav - RED4ext plugin entry points.

#include <RED4ext/RED4ext.hpp>

#include "game/Log.hpp"
#include "game/Runtime.hpp"

namespace vnav::game
{
void RegisterScriptBindings();
}

namespace
{
const RED4ext::v1::Sdk* g_sdk = nullptr;
RED4ext::v1::PluginHandle g_handle = nullptr;

// Registration happens after every base type exists, so parameter types like Vector4 and CName
// resolve. Named distinctly from RED4ext's magic RegisterTypes/PostRegisterTypes exports so it
// can only ever be invoked once, via the callback we install below.
void OnPostRegisterTypes()
{
    vnav::game::RegisterScriptBindings();
    vnav::Log("script bindings registered");
}

bool OnRunningEnter(RED4ext::CGameApplication*)
{
    vnav::game::Runtime::Get().OnGameRunning();

    auto& probe = vnav::game::Runtime::Get().Probe();
    vnav::Log("running; probe %s (%s path)", probe.IsReady() ? "ready" : "UNAVAILABLE",
              probe.UsingFilterPath() ? "QueryFilter" : "per-group fallback");
    return true;
}

// Returns false so RED4ext keeps calling it: for the Running state this is a per-frame,
// game-thread callback, which is exactly what the sampler needs. The redscript shim in
// r6/scripts is a fallback for the case where this does not fire, and stands itself down as
// soon as this one does.
bool OnRunningUpdate(RED4ext::CGameApplication*)
{
    vnav::game::Runtime::Get().PumpNative(-1);
    return false;
}

bool OnRunningExit(RED4ext::CGameApplication*)
{
    vnav::Log("world detached; dropping cached chunks");
    vnav::game::Runtime::Get().OnWorldDetached();
    return true;
}

bool OnShutdownEnter(RED4ext::CGameApplication*)
{
    vnav::Log("shutting down");
    vnav::game::Runtime::Get().OnGameShutdown();
    return true;
}
} // namespace

namespace vnav
{
void LogRaw(const char* aMessage)
{
    if (g_sdk && g_sdk->logger && g_handle)
    {
        g_sdk->logger->Info(g_handle, aMessage);
    }
}
} // namespace vnav

RED4EXT_C_EXPORT bool RED4EXT_CALL Main(RED4ext::v1::PluginHandle aHandle, RED4ext::v1::EMainReason aReason,
                                        const RED4ext::v1::Sdk* aSdk)
{
    switch (aReason)
    {
    case RED4ext::v1::EMainReason::Load:
    {
        g_sdk = aSdk;
        g_handle = aHandle;

        if (auto* rtti = RED4ext::CRTTISystem::Get())
        {
            rtti->AddPostRegisterCallback(OnPostRegisterTypes);
        }

        RED4ext::v1::GameState running{&OnRunningEnter, &OnRunningUpdate, &OnRunningExit};
        aSdk->gameStates->Add(aHandle, RED4ext::EGameStateType::Running, &running);

        RED4ext::v1::GameState shutdown{&OnShutdownEnter, nullptr, nullptr};
        aSdk->gameStates->Add(aHandle, RED4ext::EGameStateType::Shutdown, &shutdown);

        vnav::Log("loaded");
        break;
    }

    case RED4ext::v1::EMainReason::Unload:
    {
        vnav::game::Runtime::Get().OnGameShutdown();
        g_sdk = nullptr;
        g_handle = nullptr;
        break;
    }
    }

    return true;
}

RED4EXT_C_EXPORT void RED4EXT_CALL Query(RED4ext::v1::PluginInfo* aInfo)
{
    aInfo->name = L"PathfindingUpdate";
    aInfo->author = L"Ronildson Scarani Sillas Palermo";
    aInfo->version = RED4EXT_V1_SEMVER(0, 1, 0);
    aInfo->runtime = RED4EXT_V1_RUNTIME_VERSION_LATEST;
    aInfo->sdk = RED4EXT_V1_SDK_VERSION_CURRENT;
}

RED4EXT_C_EXPORT uint32_t RED4EXT_CALL Supports()
{
    return RED4EXT_API_VERSION_1;
}
