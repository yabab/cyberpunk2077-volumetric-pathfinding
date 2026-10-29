// vnav - volumetric 3D pathfinding.
//
// Core geometry / value types. Deliberately free of any engine dependency: this header
// (and everything else under vnav_core) must compile with nothing but the standard library.
#pragma once

#include <cmath>
#include <cstdint>
#include <limits>

namespace vnav
{
// Cyberpunk 2077 is Z-up. All vectors in this library follow that convention.
struct Vec3
{
    float x = 0.0f;
    float y = 0.0f;
    float z = 0.0f;

    constexpr Vec3() = default;
    constexpr Vec3(float aX, float aY, float aZ) : x(aX), y(aY), z(aZ) {}

    constexpr Vec3 operator+(const Vec3& a) const { return {x + a.x, y + a.y, z + a.z}; }
    constexpr Vec3 operator-(const Vec3& a) const { return {x - a.x, y - a.y, z - a.z}; }
    constexpr Vec3 operator*(float a) const { return {x * a, y * a, z * a}; }

    constexpr float Dot(const Vec3& a) const { return x * a.x + y * a.y + z * a.z; }
    float Length() const { return std::sqrt(Dot(*this)); }
    constexpr float LengthSq() const { return Dot(*this); }

    Vec3 Normalized() const
    {
        const float len = Length();
        return len > 1e-6f ? Vec3{x / len, y / len, z / len} : Vec3{};
    }

    bool IsFinite() const
    {
        return std::isfinite(x) && std::isfinite(y) && std::isfinite(z);
    }
};

// Integer voxel coordinate.
struct IVec3
{
    int32_t x = 0;
    int32_t y = 0;
    int32_t z = 0;

    constexpr IVec3() = default;
    constexpr IVec3(int32_t aX, int32_t aY, int32_t aZ) : x(aX), y(aY), z(aZ) {}

    constexpr bool operator==(const IVec3& a) const { return x == a.x && y == a.y && z == a.z; }
    constexpr bool operator!=(const IVec3& a) const { return !(*this == a); }
    constexpr IVec3 operator+(const IVec3& a) const { return {x + a.x, y + a.y, z + a.z}; }
    constexpr IVec3 operator-(const IVec3& a) const { return {x - a.x, y - a.y, z - a.z}; }
};

struct AABB
{
    Vec3 min{std::numeric_limits<float>::lowest(), std::numeric_limits<float>::lowest(),
             std::numeric_limits<float>::lowest()};
    Vec3 max{std::numeric_limits<float>::max(), std::numeric_limits<float>::max(),
             std::numeric_limits<float>::max()};

    constexpr AABB() = default;
    constexpr AABB(const Vec3& aMin, const Vec3& aMax) : min(aMin), max(aMax) {}

    constexpr bool Contains(const Vec3& p) const
    {
        return p.x >= min.x && p.x <= max.x && p.y >= min.y && p.y <= max.y && p.z >= min.z && p.z <= max.z;
    }

    constexpr bool Overlaps(const AABB& o) const
    {
        return min.x <= o.max.x && max.x >= o.min.x && min.y <= o.max.y && max.y >= o.min.y && min.z <= o.max.z &&
               max.z >= o.min.z;
    }

    static AABB FromCenterExtents(const Vec3& c, const Vec3& halfExtents)
    {
        return {c - halfExtents, c + halfExtents};
    }
};

// Result of a path request. Ordering matters: everything > Running is terminal.
enum class Status : int32_t
{
    Invalid = -1,        // unknown handle (finished + reaped, or never existed)
    Pending = 0,         // queued, not yet picked up
    WaitingForWorld = 1, // parked, needs chunks sampled on the game thread
    Running = 2,         // a worker is actively searching

    Success = 3,         // full path from (snapped) start to (snapped) goal
    PartialSuccess = 4,  // best effort toward the goal; goal not reached
    Unreachable = 5,     // search exhausted the reachable set without reaching the goal
    Cancelled = 6,
    BudgetExhausted = 7, // hit maxExpansions / probe budget with nothing usable
    BadInput = 8,        // non-finite coords, start == goal, outside volume bounds
    NoVolume = 9,        // named volume does not exist
    TimedOut = 10,       // sync-off-thread wait expired
};

constexpr bool IsTerminal(Status s)
{
    return static_cast<int32_t>(s) >= static_cast<int32_t>(Status::Success);
}

constexpr bool IsUsable(Status s)
{
    return s == Status::Success || s == Status::PartialSuccess;
}

const char* ToString(Status s);

// Chunks are 16^3 voxels. Small enough that eviction and demand-sampling stay fine-grained,
// large enough that "the sky is empty" collapses to a single physics probe.
inline constexpr int32_t kChunkDim = 16;
inline constexpr int32_t kChunkVoxels = kChunkDim * kChunkDim * kChunkDim; // 4096

// Voxels store distance-to-nearest-solid, saturating here. 255 voxels is far enough that no
// agent profile can care about the difference, so the cap is never a real limit - it just keeps
// a voxel down to one byte.
inline constexpr uint8_t kMaxClearance = 255;

// Floor division / modulo that behave correctly for negative coordinates. Plain `/` and `%`
// truncate toward zero, which mirrors the grid about the origin and puts a seam through the
// middle of the map.
constexpr int32_t FloorDiv(int32_t a, int32_t b)
{
    const int32_t q = a / b;
    return (a % b != 0 && ((a < 0) != (b < 0))) ? q - 1 : q;
}

constexpr int32_t FloorMod(int32_t a, int32_t b)
{
    const int32_t r = a % b;
    return (r != 0 && ((r < 0) != (b < 0))) ? r + b : r;
}

// Pack a chunk coordinate into a 64-bit key. 21 signed bits per axis covers +/-1,048,576
// chunks, i.e. +/-16 million voxels per axis - orders of magnitude beyond Night City.
constexpr uint64_t PackChunkKey(const IVec3& c)
{
    return (static_cast<uint64_t>(static_cast<uint32_t>(c.x) & 0x1FFFFFu) << 42) |
           (static_cast<uint64_t>(static_cast<uint32_t>(c.y) & 0x1FFFFFu) << 21) |
           (static_cast<uint64_t>(static_cast<uint32_t>(c.z) & 0x1FFFFFu));
}

constexpr int32_t Unpack21(uint64_t v)
{
    const auto raw = static_cast<int32_t>(v & 0x1FFFFFu);
    return (raw & 0x100000) ? (raw | ~0x1FFFFF) : raw; // sign-extend from bit 20
}

constexpr IVec3 UnpackChunkKey(uint64_t k)
{
    return {Unpack21(k >> 42), Unpack21(k >> 21), Unpack21(k)};
}

// Pack a voxel coordinate the same way, for node maps in the planner.
constexpr uint64_t PackVoxelKey(const IVec3& v)
{
    return PackChunkKey(v);
}

constexpr IVec3 UnpackVoxelKey(uint64_t k)
{
    return UnpackChunkKey(k);
}

// --- 4D search state keys (position + yaw) ---------------------------------------------------
//
// The 3D key above spends 21 bits per axis and so uses 63 of 64 bits, leaving nowhere to put an
// orientation. A yaw-aware search therefore needs its own packing: 19 bits per axis plus 6 bits
// of yaw bucket. 19 bits signed is +-262144 voxels, which at 1 m is +-262 km - two orders of
// magnitude beyond the +-16 km the game world spans - so nothing real is given up for it.
//
// Used ONLY as a node-map key inside the planner. Chunk lookups still go through PackChunkKey.
inline constexpr int32_t kMaxYawBuckets = 64;

constexpr int32_t Unpack19(uint64_t v)
{
    const auto raw = static_cast<int32_t>(v & 0x7FFFFu);
    return (raw & 0x40000) ? (raw | ~0x7FFFF) : raw; // sign-extend from bit 18
}

constexpr uint64_t PackStateKey(const IVec3& v, int32_t aYaw)
{
    return (static_cast<uint64_t>(static_cast<uint32_t>(v.x) & 0x7FFFFu) << 44) |
           (static_cast<uint64_t>(static_cast<uint32_t>(v.y) & 0x7FFFFu) << 25) |
           (static_cast<uint64_t>(static_cast<uint32_t>(v.z) & 0x7FFFFu) << 6) |
           (static_cast<uint64_t>(aYaw) & 0x3Fu);
}

constexpr IVec3 UnpackStateVoxel(uint64_t k)
{
    return {Unpack19(k >> 44), Unpack19(k >> 25), Unpack19(k >> 6)};
}

constexpr int32_t UnpackStateYaw(uint64_t k)
{
    return static_cast<int32_t>(k & 0x3Fu);
}
} // namespace vnav
