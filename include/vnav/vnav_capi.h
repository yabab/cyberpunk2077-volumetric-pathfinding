/*
 * vnav - stable C ABI for other RED4ext plugins.
 *
 * This is the cross-plugin surface. It is plain C with no STL in any signature, so a consumer
 * can bind to it without matching our MSVC version, CRT or standard-library ABI:
 *
 *     HMODULE dll = GetModuleHandleW(L"PathfindingUpdate.dll");
 *     auto getApi  = (VNav_GetApiFn)GetProcAddress(dll, "VNav_GetApi");
 *     auto* api    = (const VNav_Api_v1*)getApi(1);
 *     if (api && api->structSize >= sizeof(VNav_Api_v1)) { ... }
 *
 * Check `structSize` before calling anything added after v1: the struct only ever grows at the
 * end within a version, so an older consumer keeps working against a newer plugin.
 *
 * THREADING: everything here is safe from any thread EXCEPT Pump, which must be called on the
 * game thread. In practice you never need to call Pump - the plugin drives it per frame itself.
 */
#ifndef VNAV_CAPI_H
#define VNAV_CAPI_H

#include <stdint.h>

#ifdef __cplusplus
extern "C" {
#endif

/* Mirrors vnav::Status. Anything >= VNAV_STATUS_SUCCESS is terminal. */
#define VNAV_STATUS_INVALID (-1)
#define VNAV_STATUS_PENDING 0
#define VNAV_STATUS_WAITING_FOR_WORLD 1
#define VNAV_STATUS_RUNNING 2
#define VNAV_STATUS_SUCCESS 3
#define VNAV_STATUS_PARTIAL 4
#define VNAV_STATUS_UNREACHABLE 5
#define VNAV_STATUS_CANCELLED 6
#define VNAV_STATUS_BUDGET_EXHAUSTED 7
#define VNAV_STATUS_BAD_INPUT 8
#define VNAV_STATUS_NO_VOLUME 9
#define VNAV_STATUS_TIMED_OUT 10

/* Mirrors vnav::CollisionMask. */
#define VNAV_COLLISION_STATIC 0x01u
#define VNAV_COLLISION_TERRAIN 0x02u
#define VNAV_COLLISION_COLLIDER 0x04u
#define VNAV_COLLISION_DESTRUCTIBLE 0x08u
#define VNAV_COLLISION_DYNAMIC 0x10u
#define VNAV_COLLISION_VEHICLE 0x20u
#define VNAV_COLLISION_PLAYER_BLOCKER 0x40u
#define VNAV_COLLISION_WORLD                                                                                           \
    (VNAV_COLLISION_STATIC | VNAV_COLLISION_TERRAIN | VNAV_COLLISION_COLLIDER | VNAV_COLLISION_DESTRUCTIBLE)

/* Mirrors vnav::Smoothing. */
#define VNAV_SMOOTHING_NONE 0
#define VNAV_SMOOTHING_STRING_PULL 1
#define VNAV_SMOOTHING_SPLINE 2

typedef struct VNav_Vec3
{
    float x, y, z;
} VNav_Vec3;

typedef struct VNav_VolumeDesc
{
    float voxelSize;
    float agentRadius;
    float margin;
    VNav_Vec3 boundsMin;
    VNav_Vec3 boundsMax;
    uint32_t collisionMask;
    int32_t maxProbeDepth;
    int32_t maxProbesPerChunk;
    int32_t maxResidentChunks;
    float chunkTtlSeconds;
} VNav_VolumeDesc;

typedef struct VNav_QueryDesc
{
    VNav_Vec3 start;
    VNav_Vec3 goal;
    float snapRadius;
    int32_t allowPartial; /* 0 or 1 */
    int32_t maxExpansions;
    int32_t maxProbesPerQuery;
    float heuristicWeight;
    float tightnessPenalty;
    float verticalCostMultiplier;
    int32_t smoothing;
    int32_t splineSamplesPerSegment;
    int32_t prefetchRadiusChunks;
    float timeoutSeconds;
} VNav_QueryDesc;

typedef struct VNav_PathInfo
{
    int32_t status;
    int32_t pointCount;
    float length;
    VNav_Vec3 snappedStart;
    VNav_Vec3 snappedGoal;
    int32_t startWasSnapped;
    int32_t goalWasSnapped;
    int32_t expansions;
    int32_t probes;
    int32_t chunksSampled;
    int32_t parkCount;
    float elapsedMs;
} VNav_PathInfo;

typedef struct VNav_Api_v1
{
    /* sizeof(VNav_Api_v1) as built by the plugin. Check before calling later additions. */
    uint32_t structSize;

    /* Fill a descriptor with the library defaults. Always call these first, then override the
       fields you care about - it keeps a consumer source-compatible when defaults change. */
    void (*VolumeDescDefaults)(VNav_VolumeDesc* aOut);
    void (*QueryDescDefaults)(VNav_QueryDesc* aOut);

    int32_t (*CreateVolume)(const char* aName, const VNav_VolumeDesc* aDesc);
    int32_t (*DestroyVolume)(const char* aName);
    int32_t (*HasVolume)(const char* aName);

    /* Attaches a baked static world (.vnavbake) to a volume. The bake sees geometry the physics
       scene has not streamed in, so it is what makes long-range planning correct rather than
       merely fast. aPath is relative to the game root unless absolute. */
    int32_t (*AttachBake)(const char* aVolume, const char* aPath);
    int32_t (*HasBake)(const char* aVolume);

    /* Both return a handle, or 0 on failure. Handles from either call are read back with
       GetInfo / CopyPoints and freed with Release. */
    uint32_t (*SubmitAsync)(const char* aVolume, const VNav_QueryDesc* aQuery);
    uint32_t (*FindPathSync)(const char* aVolume, const VNav_QueryDesc* aQuery);

    /* Current status; fills aInfo once terminal. aInfo may be NULL. */
    int32_t (*GetInfo)(uint32_t aHandle, VNav_PathInfo* aInfo);

    /* Copies up to aMaxPoints waypoints into aDst; returns how many were written. */
    int32_t (*CopyPoints)(uint32_t aHandle, VNav_Vec3* aDst, int32_t aMaxPoints);

    int32_t (*Cancel)(uint32_t aHandle);
    void (*Release)(uint32_t aHandle);

    void (*InvalidateBox)(const char* aVolume, VNav_Vec3 aCenter, VNav_Vec3 aHalfExtents);
    void (*InvalidateAll)(const char* aVolume);

    /* GAME THREAD ONLY, and normally unnecessary - the plugin pumps itself every frame. */
    void (*Pump)(int32_t aBudgetMicros);

    /* Same keys as the redscript VNavStat(): "queries", "residentChunks", "jobsParked", ... */
    int32_t (*Stat)(const char* aKey);
    int32_t (*IsReady)(void);
} VNav_Api_v1;

/* Returns NULL for an unsupported version. */
typedef const void*(__cdecl* VNav_GetApiFn)(uint32_t aVersion);

__declspec(dllexport) const void* __cdecl VNav_GetApi(uint32_t aVersion);

#ifdef __cplusplus
} /* extern "C" */
#endif

#endif /* VNAV_CAPI_H */
