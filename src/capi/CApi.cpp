// vnav - implementation of the stable C ABI declared in include/vnav/vnav_capi.h.
//
// Every entry point here is a thin, defensive shim: no STL crosses the boundary, null pointers
// are tolerated, and nothing throws. A consumer plugin binding by GetProcAddress must not be
// able to take the game down by passing something unexpected.

#include <cstring>
#include <string>

#include <vnav/Config.hpp>
#include <vnav/NavService.hpp>
#include <vnav/vnav_capi.h>

#include "game/Runtime.hpp"

namespace
{
using namespace vnav;
using vnav::game::Runtime;

VNav_Vec3 ToC(const Vec3& v)
{
    return VNav_Vec3{v.x, v.y, v.z};
}

Vec3 FromC(const VNav_Vec3& v)
{
    return Vec3{v.x, v.y, v.z};
}

bool Usable()
{
    return Runtime::Get().IsRunning();
}

VolumeConfig ToConfig(const VNav_VolumeDesc& d)
{
    VolumeConfig cfg;
    cfg.voxelSize = d.voxelSize;
    cfg.agentRadius = d.agentRadius;
    cfg.margin = d.margin;
    cfg.bounds = AABB{FromC(d.boundsMin), FromC(d.boundsMax)};
    cfg.collision = d.collisionMask == 0 ? CollisionMask::World : static_cast<CollisionMask>(d.collisionMask);
    cfg.maxProbeDepth = d.maxProbeDepth;
    cfg.maxProbesPerChunk = d.maxProbesPerChunk;
    cfg.maxResidentChunks = d.maxResidentChunks;
    cfg.chunkTtlSeconds = d.chunkTtlSeconds;
    return cfg;
}

PathQuery ToQuery(const VNav_QueryDesc& d)
{
    PathQuery q;
    q.start = FromC(d.start);
    q.goal = FromC(d.goal);
    q.snapRadius = d.snapRadius;
    q.allowPartial = d.allowPartial != 0;
    q.maxExpansions = d.maxExpansions;
    q.maxProbesPerQuery = d.maxProbesPerQuery;
    q.heuristicWeight = d.heuristicWeight;
    q.tightnessPenalty = d.tightnessPenalty;
    q.verticalCostMultiplier = d.verticalCostMultiplier;
    q.smoothing = static_cast<Smoothing>(d.smoothing < 0 || d.smoothing > 2 ? 1 : d.smoothing);
    q.splineSamplesPerSegment = d.splineSamplesPerSegment;
    q.prefetchRadiusChunks = d.prefetchRadiusChunks;
    q.timeoutSeconds = d.timeoutSeconds;
    return q;
}

// -------------------------------------------------------------------------------------------

void CApi_VolumeDescDefaults(VNav_VolumeDesc* aOut)
{
    if (!aOut)
    {
        return;
    }
    const VolumeConfig d;
    aOut->voxelSize = d.voxelSize;
    aOut->agentRadius = d.agentRadius;
    aOut->margin = d.margin;
    // Default bounds are the full float range, which is not expressible as a useful literal for
    // a caller; hand back something large but finite and expect it to be overridden.
    aOut->boundsMin = VNav_Vec3{-100000.0f, -100000.0f, -100000.0f};
    aOut->boundsMax = VNav_Vec3{100000.0f, 100000.0f, 100000.0f};
    aOut->collisionMask = VNAV_COLLISION_WORLD;
    aOut->maxProbeDepth = d.maxProbeDepth;
    aOut->maxProbesPerChunk = d.maxProbesPerChunk;
    aOut->maxResidentChunks = d.maxResidentChunks;
    aOut->chunkTtlSeconds = d.chunkTtlSeconds;
}

void CApi_QueryDescDefaults(VNav_QueryDesc* aOut)
{
    if (!aOut)
    {
        return;
    }
    const PathQuery d;
    aOut->start = VNav_Vec3{0.0f, 0.0f, 0.0f};
    aOut->goal = VNav_Vec3{0.0f, 0.0f, 0.0f};
    aOut->snapRadius = d.snapRadius;
    aOut->allowPartial = d.allowPartial ? 1 : 0;
    aOut->maxExpansions = d.maxExpansions;
    aOut->maxProbesPerQuery = d.maxProbesPerQuery;
    aOut->heuristicWeight = d.heuristicWeight;
    aOut->tightnessPenalty = d.tightnessPenalty;
    aOut->verticalCostMultiplier = d.verticalCostMultiplier;
    aOut->smoothing = static_cast<int32_t>(d.smoothing);
    aOut->splineSamplesPerSegment = d.splineSamplesPerSegment;
    aOut->prefetchRadiusChunks = d.prefetchRadiusChunks;
    aOut->timeoutSeconds = d.timeoutSeconds;
}

int32_t CApi_CreateVolume(const char* aName, const VNav_VolumeDesc* aDesc)
{
    if (!aName || !aDesc || !Usable())
    {
        return 0;
    }
    return Runtime::Get().Nav().CreateVolume(aName, ToConfig(*aDesc)) ? 1 : 0;
}

int32_t CApi_DestroyVolume(const char* aName)
{
    if (!aName || !Usable())
    {
        return 0;
    }
    return Runtime::Get().Nav().DestroyVolume(aName) ? 1 : 0;
}

int32_t CApi_HasVolume(const char* aName)
{
    if (!aName || !Usable())
    {
        return 0;
    }
    return Runtime::Get().Nav().HasVolume(aName) ? 1 : 0;
}

int32_t CApi_AttachBake(const char* aVolume, const char* aPath)
{
    if (!aVolume || !aPath || !Usable())
    {
        return 0;
    }

    std::string err;
    return Runtime::Get().Nav().AttachBake(aVolume, Runtime::Get().ResolveGamePath(aPath), err) ? 1 : 0;
}

int32_t CApi_HasBake(const char* aVolume)
{
    if (!aVolume || !Usable())
    {
        return 0;
    }
    return Runtime::Get().Nav().HasBake(aVolume) ? 1 : 0;
}

uint32_t CApi_SubmitAsync(const char* aVolume, const VNav_QueryDesc* aQuery)
{
    if (!aVolume || !aQuery || !Usable())
    {
        return 0;
    }
    return Runtime::Get().Nav().SubmitAsync(aVolume, ToQuery(*aQuery));
}

uint32_t CApi_FindPathSync(const char* aVolume, const VNav_QueryDesc* aQuery)
{
    if (!aVolume || !aQuery || !Usable())
    {
        return 0;
    }

    PathResult result;
    Runtime::Get().Nav().FindPathSync(aVolume, ToQuery(*aQuery), result);
    return Runtime::Get().StoreSyncResult(result);
}

int32_t CApi_GetInfo(uint32_t aHandle, VNav_PathInfo* aInfo)
{
    if (!Usable())
    {
        return VNAV_STATUS_INVALID;
    }

    PathResult r;
    const bool terminal = Runtime::Get().ResolveResult(aHandle, r);

    if (aInfo)
    {
        std::memset(aInfo, 0, sizeof(*aInfo));
        aInfo->status = static_cast<int32_t>(r.status);
        if (terminal)
        {
            aInfo->pointCount = static_cast<int32_t>(r.points.size());
            aInfo->length = r.length;
            aInfo->snappedStart = ToC(r.snappedStart);
            aInfo->snappedGoal = ToC(r.snappedGoal);
            aInfo->startWasSnapped = r.startWasSnapped ? 1 : 0;
            aInfo->goalWasSnapped = r.goalWasSnapped ? 1 : 0;
            aInfo->expansions = r.expansions;
            aInfo->probes = r.probes;
            aInfo->chunksSampled = r.chunksSampled;
            aInfo->parkCount = r.parkCount;
            aInfo->elapsedMs = r.elapsedMs;
        }
    }

    return static_cast<int32_t>(r.status);
}

int32_t CApi_CopyPoints(uint32_t aHandle, VNav_Vec3* aDst, int32_t aMaxPoints)
{
    if (!aDst || aMaxPoints <= 0 || !Usable())
    {
        return 0;
    }

    PathResult r;
    if (!Runtime::Get().ResolveResult(aHandle, r))
    {
        return 0;
    }

    const auto count = static_cast<int32_t>(r.points.size());
    const int32_t n = count < aMaxPoints ? count : aMaxPoints;
    for (int32_t i = 0; i < n; ++i)
    {
        aDst[i] = ToC(r.points[static_cast<size_t>(i)]);
    }
    return n;
}

int32_t CApi_Cancel(uint32_t aHandle)
{
    if (!Usable() || (aHandle & vnav::game::kSyncHandleBit) != 0)
    {
        return 0;
    }
    return Runtime::Get().Nav().Cancel(aHandle) ? 1 : 0;
}

void CApi_Release(uint32_t aHandle)
{
    Runtime::Get().ReleaseResult(aHandle);
}

void CApi_InvalidateBox(const char* aVolume, VNav_Vec3 aCenter, VNav_Vec3 aHalfExtents)
{
    if (!aVolume || !Usable())
    {
        return;
    }
    Runtime::Get().Nav().InvalidateBox(aVolume, AABB::FromCenterExtents(FromC(aCenter), FromC(aHalfExtents)));
}

void CApi_InvalidateAll(const char* aVolume)
{
    if (!aVolume || !Usable())
    {
        return;
    }
    Runtime::Get().Nav().InvalidateAll(aVolume);
}

void CApi_Pump(int32_t aBudgetMicros)
{
    Runtime::Get().Pump(aBudgetMicros);
}

int32_t CApi_Stat(const char* aKey)
{
    if (!aKey || !Usable())
    {
        return 0;
    }

    const Stats s = Runtime::Get().Nav().GetStats();
    const std::string k = aKey;

    if (k == "queries") { return static_cast<int32_t>(s.totalQueries); }
    if (k == "succeeded") { return static_cast<int32_t>(s.succeeded); }
    if (k == "partial") { return static_cast<int32_t>(s.partial); }
    if (k == "failed") { return static_cast<int32_t>(s.failed); }
    if (k == "probes") { return static_cast<int32_t>(s.totalProbes); }
    if (k == "chunksSampled") { return static_cast<int32_t>(s.totalChunksSampled); }
    if (k == "residentChunks") { return s.residentChunks; }
    if (k == "residentKB") { return static_cast<int32_t>(s.residentBytes / 1024); }
    if (k == "jobsPending") { return s.jobsPending; }
    if (k == "jobsParked") { return s.jobsParked; }
    if (k == "jobsRunning") { return s.jobsRunning; }
    if (k == "pendingRequests") { return s.pendingChunkRequests; }
    if (k == "usingFilterPath") { return Runtime::Get().Probe().UsingFilterPath() ? 1 : 0; }
    if (k == "nativePump") { return Runtime::Get().NativePumpActive() ? 1 : 0; }
    if (k == "chunksFromBake") { return static_cast<int32_t>(s.chunksFromBake); }
    return 0;
}

int32_t CApi_IsReady()
{
    return (Usable() && Runtime::Get().Probe().IsReady()) ? 1 : 0;
}

const VNav_Api_v1 g_api_v1{
    static_cast<uint32_t>(sizeof(VNav_Api_v1)),
    &CApi_VolumeDescDefaults,
    &CApi_QueryDescDefaults,
    &CApi_CreateVolume,
    &CApi_DestroyVolume,
    &CApi_HasVolume,
    &CApi_AttachBake,
    &CApi_HasBake,
    &CApi_SubmitAsync,
    &CApi_FindPathSync,
    &CApi_GetInfo,
    &CApi_CopyPoints,
    &CApi_Cancel,
    &CApi_Release,
    &CApi_InvalidateBox,
    &CApi_InvalidateAll,
    &CApi_Pump,
    &CApi_Stat,
    &CApi_IsReady,
};
} // namespace

extern "C" __declspec(dllexport) const void* __cdecl VNav_GetApi(uint32_t aVersion)
{
    return aVersion == 1 ? static_cast<const void*>(&g_api_v1) : nullptr;
}
