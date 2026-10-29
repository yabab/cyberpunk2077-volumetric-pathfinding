// vnav - the single seam between the pathfinder and the world.
#pragma once

#include <vnav/Config.hpp>
#include <vnav/Types.hpp>

namespace vnav
{
// The ONLY way vnav_core learns anything about the world.
//
// THREADING CONTRACT: implementations are called exclusively from NavService::Pump(), which the
// host must call on the game thread. The core never calls this from a worker. Do not relax that
// - REDengine drives PhysX on its own schedule and scene queries issued from an arbitrary thread
// race the simulation step.
//
// Keeping this interface to one method is deliberate. It is what lets vnav_core build and be
// unit-tested with no game attached (see tests/TestCore.cpp), and it is the seam where a direct
// native call can later replace the RTTI-dispatched one without the core noticing.
struct IWorldProbe
{
    virtual ~IWorldProbe() = default;

    // Returns true if anything matching `mask` intersects the axis-aligned box.
    // `halfExtents` is a half-size, matching the game's own Overlap convention.
    virtual bool OverlapBox(const Vec3& center, const Vec3& halfExtents, CollisionMask mask) = 0;
};

// A probe that reports empty space everywhere. Used when no world is attached (loading screens,
// main menu) so the service degrades to "no geometry" instead of crashing or blocking.
struct NullWorldProbe final : IWorldProbe
{
    bool OverlapBox(const Vec3&, const Vec3&, CollisionMask) override { return false; }
};
} // namespace vnav
