// vnav - IWorldProbe backed by the game's SpatialQueriesSystem. GAME THREAD ONLY.
#pragma once

#include <cstdint>
#include <vector>

#include <RED4ext/RED4ext.hpp>

// RED4ext.hpp is only the umbrella for the core runtime; reflected game types have to be
// pulled in individually.
#include <RED4ext/Scripting/Natives/Generated/EulerAngles.hpp>
#include <RED4ext/Scripting/Natives/Generated/Quaternion.hpp>
#include <RED4ext/Scripting/Natives/Generated/Vector4.hpp>
#include <RED4ext/Scripting/Natives/Generated/physics/QueryFilter.hpp>

#include <vnav/Config.hpp>
#include <vnav/Types.hpp>
#include <vnav/WorldProbe.hpp>

namespace vnav::game
{
// Turns "is this box of space free" into a physics scene query.
//
// Everything resolvable is resolved once and cached: the RTTI class, the function pointers, the
// system instance, the collision filter, AND the parameter types. A probe therefore costs one
// dispatch and no lookups. That matters - a single chunk can be hundreds of probes.
//
// HOW FAR "DIRECT" CAN GO, since this was investigated and the answer is not obvious:
// the game's own native dispatcher cannot be bypassed. CBaseFunction::Execute reaches it through
// UniversalRelocFunc, and the SDK keeps ExecuteNative/GetHandler private with no native function
// pointer exposed - reaching the real address would mean signature-scanning, which is fragile and
// breaks on every patch. What CAN be removed is our own wrapper: RED4ext::ExecuteFunction rebuilds
// the argument list and re-resolves every parameter type on each call. So the args are built once
// with their types pre-filled and the CStack is constructed here, leaving only the dispatch.
//
// Two backends, chosen at runtime:
//   1. OverlapByQueryFilter - one call covers every collision group in the mask. Preferred.
//   2. Overlap              - one collision group per call, so N calls per probe. Used only if
//                             the QueryFilter type or its AddGroup helper cannot be resolved.
class RedWorldProbe final : public IWorldProbe
{
public:
    // Resolve RTTI. Safe to call repeatedly; returns true once the probe is usable.
    bool Initialize();

    // Re-fetch the system instance. Called once per frame from the pump rather than per probe,
    // because the instance changes across world load/unload but never mid-frame.
    void RefreshFrame();

    // Forget everything resolved against the current session. Call on world detach.
    void Invalidate();

    bool IsReady() const { return m_system != nullptr && m_overlapByFilter != nullptr; }

    bool OverlapBox(const Vec3& aCenter, const Vec3& aHalfExtents, CollisionMask aMask) override;

    int64_t ProbeCount() const { return m_probeCount; }
    bool UsingFilterPath() const { return m_overlapByFilter != nullptr; }

private:
    struct FilterEntry
    {
        uint32_t mask;
        RED4ext::QueryFilter filter;
    };

    const RED4ext::QueryFilter* GetFilter(CollisionMask aMask);
    bool BuildFilter(CollisionMask aMask, RED4ext::QueryFilter& aOut) const;
    bool OverlapSingleGroup(const RED4ext::Vector4& aDim, const RED4ext::Vector4& aPos, RED4ext::CName aGroup);

    RED4ext::CClass* m_systemClass = nullptr;
    RED4ext::CBaseFunction* m_overlapByFilter = nullptr; // OverlapByQueryFilter (preferred)
    RED4ext::CBaseFunction* m_overlapByGroup = nullptr;  // Overlap (fallback)
    RED4ext::CBaseFunction* m_addGroup = nullptr;        // QueryFilter::AddGroup
    RED4ext::IScriptable* m_system = nullptr;

    std::vector<FilterEntry> m_filterCache;

    // Reused across probes so a scene query does not allocate. Game thread only, so no guard.
    RED4ext::StackArgs_t m_args;
    alignas(16) uint8_t m_traceResult[0x60]{}; // physics::TraceResult

    // --- pre-bound call state for the filter path ------------------------------------------
    // Built once by BindFilterCall(), then only the values change per probe. Holding the operands
    // as members (rather than as locals passed by address) is what lets the arg list stay valid
    // across calls instead of being rebuilt each time.
    bool BindFilterCall();

    RED4ext::StackArgs_t m_filterArgs;
    RED4ext::Vector4 m_argDim{};
    RED4ext::Vector4 m_argPos{};
    RED4ext::Quaternion m_argRot{0.0f, 0.0f, 0.0f, 1.0f}; // axis-aligned, never changes
    RED4ext::QueryFilter m_argFilter{};                   // refreshed only when the mask changes
    uint32_t m_argFilterMask = 0xFFFFFFFFu;               // sentinel: no filter bound yet
    bool m_filterCallBound = false;

    int64_t m_probeCount = 0;
    bool m_resolved = false;
};
} // namespace vnav::game
