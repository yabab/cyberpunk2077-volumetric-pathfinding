#include "game/RedWorldProbe.hpp"

#include <array>

namespace vnav::game
{
namespace
{
struct GroupBinding
{
    CollisionMask bit;
    const char* name;
};

// Collision group names verified against vanilla AI code that does exactly this kind of query
// (cyberpunk/ai/commands/aiLineTrace.swift, aiFindTeleportPositionForTakedown.swift).
// Ordered most-likely-to-hit first, which matters for the single-group fallback path: it can
// stop at the first hit.
constexpr std::array<GroupBinding, 7> kGroups{{
    {CollisionMask::Static, "Static"},
    {CollisionMask::Terrain, "Terrain"},
    {CollisionMask::Collider, "Collider"},
    {CollisionMask::Destructible, "Destructible"},
    {CollisionMask::Vehicle, "Vehicle"},
    {CollisionMask::Dynamic, "Dynamic"},
    {CollisionMask::PlayerBlocker, "PlayerBlocker"},
}};

RED4ext::GameInstance* GetGameInstance()
{
    auto* engine = RED4ext::CGameEngine::Get();
    if (!engine || !engine->framework)
    {
        return nullptr;
    }
    return engine->framework->gameInstance;
}

RED4ext::CClass* ResolveClass(RED4ext::CRTTISystem* aRtti, std::initializer_list<const char*> aNames)
{
    for (const char* name : aNames)
    {
        if (auto* cls = aRtti->GetClass(name))
        {
            return cls;
        }
    }
    return nullptr;
}
} // namespace

bool RedWorldProbe::Initialize()
{
    if (m_resolved)
    {
        return IsReady();
    }

    auto* rtti = RED4ext::CRTTISystem::Get();
    if (!rtti)
    {
        return false;
    }

    // The reflected name is gameSpatialQueriesSystem; SpatialQueriesSystem is its script alias.
    m_systemClass = ResolveClass(rtti, {"gameSpatialQueriesSystem", "SpatialQueriesSystem"});
    if (!m_systemClass)
    {
        return false;
    }

    m_overlapByFilter = m_systemClass->GetFunction("OverlapByQueryFilter");
    m_overlapByGroup = m_systemClass->GetFunction("Overlap");

    // QueryFilter's bit layout is internal, so the only correct way to build one is to let the
    // game's own AddGroup do it.
    if (auto* filterClass = ResolveClass(rtti, {"physicsQueryFilter", "QueryFilter"}))
    {
        m_addGroup = filterClass->GetFunction("AddGroup");
    }

    if (!m_addGroup)
    {
        // Without AddGroup a QueryFilter cannot be populated, so fall back to per-group probes.
        m_overlapByFilter = nullptr;
    }

    m_resolved = true;
    RefreshFrame();
    return IsReady() || m_overlapByGroup != nullptr;
}

void RedWorldProbe::RefreshFrame()
{
    if (!m_systemClass)
    {
        return;
    }

    auto* gi = GetGameInstance();
    m_system = gi ? gi->GetSystem(m_systemClass) : nullptr;
}

void RedWorldProbe::Invalidate()
{
    m_system = nullptr;
    m_filterCache.clear();

    // The bound arg list holds RTTI type pointers from the session being torn down, so it has to
    // be rebuilt rather than reused against the next one.
    m_filterArgs.clear();
    m_filterCallBound = false;
    m_argFilterMask = 0xFFFFFFFFu;
}

bool RedWorldProbe::BuildFilter(CollisionMask aMask, RED4ext::QueryFilter& aOut) const
{
    if (!m_addGroup)
    {
        return false;
    }

    aOut = RED4ext::QueryFilter{};

    bool any = false;
    for (const GroupBinding& g : kGroups)
    {
        if (!HasFlag(aMask, g.bit))
        {
            continue;
        }

        RED4ext::CName group(g.name);
        RED4ext::StackArgs_t args;
        args.emplace_back(nullptr, &aOut);  // out filter
        args.emplace_back(nullptr, &group); // group name

        // AddGroup is static, hence a null instance. The cast disambiguates between the
        // void*-instance and CClass*-context overloads.
        if (RED4ext::ExecuteFunction(static_cast<void*>(nullptr), m_addGroup, nullptr, args))
        {
            any = true;
        }
    }
    return any;
}

const RED4ext::QueryFilter* RedWorldProbe::GetFilter(CollisionMask aMask)
{
    const auto raw = static_cast<uint32_t>(aMask);
    for (const FilterEntry& e : m_filterCache)
    {
        if (e.mask == raw)
        {
            return &e.filter;
        }
    }

    FilterEntry entry{raw, {}};
    if (!BuildFilter(aMask, entry.filter))
    {
        return nullptr;
    }

    m_filterCache.push_back(entry);
    return &m_filterCache.back().filter;
}

// Build the argument list once, with parameter types already filled in.
//
// ExecuteFunction fills a null arg type from aFunc->params[i]->type on every call, and explicitly
// skips any arg whose type is already set. Pre-filling them therefore turns that loop into a
// no-op - and because the values are members, the list itself never has to be rebuilt.
bool RedWorldProbe::BindFilterCall()
{
    if (m_filterCallBound)
    {
        return true;
    }
    if (!m_overlapByFilter || m_overlapByFilter->params.size() < 5)
    {
        return false;
    }

    const auto typeOf = [&](uint32_t i) { return m_overlapByFilter->params[i]->type; };

    m_filterArgs.clear();
    m_filterArgs.emplace_back(typeOf(0), &m_argDim);
    m_filterArgs.emplace_back(typeOf(1), &m_argPos);
    m_filterArgs.emplace_back(typeOf(2), &m_argRot);
    m_filterArgs.emplace_back(typeOf(3), &m_argFilter);
    m_filterArgs.emplace_back(typeOf(4), m_traceResult);

    m_filterCallBound = true;
    return true;
}

bool RedWorldProbe::OverlapSingleGroup(const RED4ext::Vector4& aDim, const RED4ext::Vector4& aPos,
                                       RED4ext::CName aGroup)
{
    RED4ext::EulerAngles rotation{0.0f, 0.0f, 0.0f};
    bool hit = false;

    m_args.clear();
    m_args.emplace_back(nullptr, const_cast<RED4ext::Vector4*>(&aDim));
    m_args.emplace_back(nullptr, const_cast<RED4ext::Vector4*>(&aPos));
    m_args.emplace_back(nullptr, &rotation);
    m_args.emplace_back(nullptr, &aGroup);
    m_args.emplace_back(nullptr, m_traceResult);

    if (!RED4ext::ExecuteFunction(m_system, m_overlapByGroup, &hit, m_args))
    {
        return false;
    }
    return hit;
}

bool RedWorldProbe::OverlapBox(const Vec3& aCenter, const Vec3& aHalfExtents, CollisionMask aMask)
{
    // No world attached: report empty space rather than blocking everything, matching
    // NullWorldProbe. An over-blocked map would poison the chunk cache for the whole session.
    if (!m_system)
    {
        return false;
    }

    ++m_probeCount;

    // The game's Overlap takes half-extents in primitiveDimension, same as ours.
    const RED4ext::Vector4 dim{aHalfExtents.x, aHalfExtents.y, aHalfExtents.z, 0.0f};
    const RED4ext::Vector4 pos{aCenter.x, aCenter.y, aCenter.z, 1.0f};

    if (m_overlapByFilter && BindFilterCall())
    {
        // Refresh the bound filter only when the caller changes mask, which in practice is never
        // within a volume. The call may take it by reference, so it stays a member we own.
        const auto raw = static_cast<uint32_t>(aMask);
        if (raw != m_argFilterMask)
        {
            if (const RED4ext::QueryFilter* filter = GetFilter(aMask))
            {
                m_argFilter = *filter;
                m_argFilterMask = raw;
            }
        }

        if (raw == m_argFilterMask)
        {
            m_argDim = dim;
            m_argPos = pos;

            bool hit = false;
            RED4ext::CStackType result;
            if (m_overlapByFilter->returnType)
            {
                result.type = m_overlapByFilter->returnType->type;
                result.value = &hit;
            }

            // What ExecuteFunction would do, minus rebuilding the argument list and re-resolving
            // every parameter type on every probe. m_filterArgs already carries both.
            RED4ext::CStack stack(m_system, m_filterArgs.data(), static_cast<uint32_t>(m_filterArgs.size()),
                                  m_overlapByFilter->returnType ? &result : nullptr);
            if (m_overlapByFilter->Execute(&stack))
            {
                return hit;
            }
        }
    }

    if (!m_overlapByGroup)
    {
        return false;
    }

    // Fallback: one call per collision group, stopping at the first hit.
    for (const GroupBinding& g : kGroups)
    {
        if (HasFlag(aMask, g.bit) && OverlapSingleGroup(dim, pos, RED4ext::CName(g.name)))
        {
            return true;
        }
    }
    return false;
}
} // namespace vnav::game
