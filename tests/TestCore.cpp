// vnav core tests - no game, no RED4ext. Builds and runs on the command line.
//
// This exists because the redscript half of this workspace cannot be typechecked without
// launching the game. Everything that CAN be verified offline is verified here: the voxel
// sampler, the search, the suspend/resume loop, and the threading contract.

#include <algorithm>
#include <atomic>
#include <chrono>
#include <cmath>
#include <cstdio>
#include <limits>
#include <memory>
#include <string>
#include <thread>
#include <vector>

#include <vnav/BakedNav.hpp>
#include <vnav/NavService.hpp>
#include <vnav/Types.hpp>
#include <vnav/WorldProbe.hpp>

#include "core/Chunk.hpp"
#include "core/Sampler.hpp"
#include "core/Volume.hpp"

using namespace vnav;

namespace
{
int g_failures = 0;
int g_checks = 0;

void Check(bool aCond, const char* aWhat, const char* aFile, int aLine)
{
    ++g_checks;
    if (!aCond)
    {
        ++g_failures;
        std::printf("  FAIL  %s\n        at %s:%d\n", aWhat, aFile, aLine);
    }
}

#define CHECK(cond) Check((cond), #cond, __FILE__, __LINE__)

// A world made of axis-aligned solid boxes. Exact, deterministic, and it counts probes so the
// sampler's cost can be asserted on rather than guessed at.
struct BoxWorld final : IWorldProbe
{
    std::vector<AABB> solids;
    std::atomic<int64_t> probeCount{0};

    bool OverlapBox(const Vec3& aCenter, const Vec3& aHalf, CollisionMask) override
    {
        probeCount.fetch_add(1, std::memory_order_relaxed);
        const AABB q{aCenter - aHalf, aCenter + aHalf};
        for (const AABB& s : solids)
        {
            if (q.Overlaps(s))
            {
                return true;
            }
        }
        return false;
    }
};

VolumeConfig MakeConfig(float aVoxel, float aRadius, float aMargin, float aHalfExtent)
{
    VolumeConfig cfg;
    cfg.voxelSize = aVoxel;
    cfg.agentRadius = aRadius;
    cfg.margin = aMargin;
    cfg.bounds = AABB{Vec3{-aHalfExtent, -aHalfExtent, -aHalfExtent}, Vec3{aHalfExtent, aHalfExtent, aHalfExtent}};
    return cfg;
}

// Drive an async job to completion the way a game frame would.
Status RunAsync(NavService& aSvc, uint32_t aJobId, PathResult& aOut, int aMaxFrames = 5000)
{
    for (int i = 0; i < aMaxFrames; ++i)
    {
        aSvc.Pump(1000000); // generous budget: tests should not be timing-sensitive
        const Status s = aSvc.Poll(aJobId, aOut);
        if (IsTerminal(s))
        {
            return s;
        }
        std::this_thread::sleep_for(std::chrono::microseconds(200));
    }
    return Status::TimedOut;
}

void Section(const char* aName)
{
    std::printf("\n[%s]\n", aName);
}

// -------------------------------------------------------------------------------------------

void TestCoordinateMath()
{
    Section("coordinate math");

    // Negative coordinates must not mirror about the origin - a truncating divide would put a
    // seam through the middle of the map.
    CHECK(FloorDiv(-1, 16) == -1);
    CHECK(FloorDiv(-16, 16) == -1);
    CHECK(FloorDiv(-17, 16) == -2);
    CHECK(FloorDiv(15, 16) == 0);
    CHECK(FloorMod(-1, 16) == 15);
    CHECK(FloorMod(-16, 16) == 0);
    CHECK(FloorMod(17, 16) == 1);

    const IVec3 coords[] = {{0, 0, 0}, {1, -2, 3}, {-1000, 4000, -32768}, {524287, -524288, 12345}};
    for (const IVec3& c : coords)
    {
        CHECK(UnpackChunkKey(PackChunkKey(c)) == c);
    }
}

void TestEmptyWorld()
{
    Section("empty world / straight line");

    BoxWorld world;
    NavService svc;
    svc.SetGameThread();
    svc.SetProbe(&world);

    CHECK(svc.CreateVolume("test", MakeConfig(1.0f, 0.4f, 0.5f, 200.0f)));
    CHECK(!svc.CreateVolume("test", MakeConfig(1.0f, 0.4f, 0.5f, 200.0f))); // duplicate rejected

    PathQuery q;
    q.start = {0.5f, 0.5f, 50.5f};
    q.goal = {40.5f, 0.5f, 50.5f};

    PathResult r;
    const Status s = svc.FindPathSync("test", q, r);

    CHECK(s == Status::Success);
    CHECK(r.points.size() >= 2);
    // Open air: string-pulling should collapse this to essentially a straight run.
    CHECK(r.points.size() <= 4);
    CHECK(r.length > 39.0f && r.length < 45.0f);

    // Open sky collapses to one probe per chunk, so a 40m run must not cost hundreds.
    std::printf("  probes=%lld chunks=%d expansions=%d len=%.2f\n",
                static_cast<long long>(world.probeCount.load()), r.chunksSampled, r.expansions,
                static_cast<double>(r.length));
    CHECK(world.probeCount.load() < 100);
}

void TestWallWithHole()
{
    Section("wall with a hole");

    BoxWorld world;
    // A wall across x = 20, with a 6x6 window centred on (y=0, z=50).
    world.solids.push_back(AABB{{19.0f, -60.0f, 0.0f}, {21.0f, -3.0f, 100.0f}});
    world.solids.push_back(AABB{{19.0f, 3.0f, 0.0f}, {21.0f, 60.0f, 100.0f}});
    world.solids.push_back(AABB{{19.0f, -3.0f, 0.0f}, {21.0f, 3.0f, 47.0f}});
    world.solids.push_back(AABB{{19.0f, -3.0f, 53.0f}, {21.0f, 3.0f, 100.0f}});

    NavService svc;
    svc.SetGameThread();
    svc.SetProbe(&world);
    CHECK(svc.CreateVolume("test", MakeConfig(1.0f, 0.4f, 0.5f, 120.0f)));

    PathQuery q;
    q.start = {5.0f, 0.0f, 50.0f};
    q.goal = {35.0f, 0.0f, 50.0f};

    PathResult r;
    const Status s = svc.FindPathSync("test", q, r);

    CHECK(s == Status::Success);
    CHECK(r.points.size() >= 2);

    // Every waypoint must be outside the wall, and the path has to actually cross it.
    bool crossed = false;
    bool insideSolid = false;
    for (const Vec3& p : r.points)
    {
        for (const AABB& solid : world.solids)
        {
            if (solid.Contains(p))
            {
                insideSolid = true;
            }
        }
    }
    if (!r.points.empty())
    {
        crossed = r.points.front().x < 19.0f && r.points.back().x > 21.0f;
    }
    CHECK(!insideSolid);
    CHECK(crossed);
    std::printf("  points=%zu len=%.2f expansions=%d chunks=%d probes=%d (%.0f probes/chunk)\n", r.points.size(),
                static_cast<double>(r.length), r.expansions, r.chunksSampled, r.probes,
                r.chunksSampled ? static_cast<double>(r.probes) / r.chunksSampled : 0.0);
}

void TestSealedGoal()
{
    Section("sealed goal is unreachable");

    BoxWorld world;
    // A hollow shell around the goal: six slabs forming a closed box from 30..40 on each axis.
    const float lo = 30.0f;
    const float hi = 40.0f;
    const float t = 2.0f;
    world.solids.push_back(AABB{{lo, lo, lo}, {hi, hi, lo + t}});
    world.solids.push_back(AABB{{lo, lo, hi - t}, {hi, hi, hi}});
    world.solids.push_back(AABB{{lo, lo, lo}, {hi, lo + t, hi}});
    world.solids.push_back(AABB{{lo, hi - t, lo}, {hi, hi, hi}});
    world.solids.push_back(AABB{{lo, lo, lo}, {lo + t, hi, hi}});
    world.solids.push_back(AABB{{hi - t, lo, lo}, {hi, hi, hi}});

    NavService svc;
    svc.SetGameThread();
    svc.SetProbe(&world);
    // Tight bounds keep the exhaustive search that proves unreachability cheap.
    VolumeConfig cfg = MakeConfig(2.0f, 0.5f, 0.5f, 64.0f);
    CHECK(svc.CreateVolume("test", cfg));

    PathQuery q;
    q.start = {0.0f, 0.0f, 0.0f};
    q.goal = {35.0f, 35.0f, 35.0f};
    q.allowPartial = false;
    q.snapRadius = 1.0f; // do not let snapping escape the shell

    PathResult r;
    const Status s = svc.FindPathSync("test", q, r);

    std::printf("  status=%s expansions=%d probes=%d\n", ToString(s), r.expansions, r.probes);
    CHECK(s == Status::Unreachable || s == Status::BudgetExhausted);
    CHECK(r.points.empty());
}

void TestPartialPath()
{
    Section("partial path toward an unreachable goal");

    BoxWorld world;
    world.solids.push_back(AABB{{20.0f, -80.0f, -80.0f}, {22.0f, 80.0f, 80.0f}}); // full-height wall

    NavService svc;
    svc.SetGameThread();
    svc.SetProbe(&world);
    CHECK(svc.CreateVolume("test", MakeConfig(2.0f, 0.5f, 0.5f, 60.0f)));

    PathQuery q;
    q.start = {0.0f, 0.0f, 0.0f};
    q.goal = {50.0f, 0.0f, 0.0f};
    q.allowPartial = true;

    PathResult r;
    const Status s = svc.FindPathSync("test", q, r);

    std::printf("  status=%s points=%zu len=%.2f\n", ToString(s), r.points.size(),
                static_cast<double>(r.length));
    CHECK(s == Status::PartialSuccess);
    CHECK(r.points.size() >= 2);
    // It should have made real progress toward the wall rather than returning the start.
    CHECK(r.points.back().x > 5.0f);
}

void TestSnapping()
{
    Section("endpoint snapping");

    BoxWorld world;
    world.solids.push_back(AABB{{-2.0f, -2.0f, -2.0f}, {2.0f, 2.0f, 2.0f}}); // start is buried

    NavService svc;
    svc.SetGameThread();
    svc.SetProbe(&world);
    CHECK(svc.CreateVolume("test", MakeConfig(1.0f, 0.4f, 0.5f, 100.0f)));

    PathQuery q;
    q.start = {0.0f, 0.0f, 0.0f}; // inside the block
    q.goal = {30.0f, 0.0f, 0.0f};
    q.snapRadius = 6.0f;

    PathResult r;
    const Status s = svc.FindPathSync("test", q, r);

    std::printf("  status=%s snapped=%d start=(%.1f,%.1f,%.1f)\n", ToString(s), r.startWasSnapped ? 1 : 0,
                static_cast<double>(r.snappedStart.x), static_cast<double>(r.snappedStart.y),
                static_cast<double>(r.snappedStart.z));
    CHECK(s == Status::Success);
    CHECK(r.startWasSnapped);
    CHECK(!r.goalWasSnapped);

    // With no snap budget the same query must fail rather than silently start inside geometry.
    PathQuery q2 = q;
    q2.snapRadius = 0.0f;
    PathResult r2;
    const Status s2 = svc.FindPathSync("test", q2, r2);
    CHECK(s2 == Status::Unreachable);
}

void TestAsyncRoundTrip()
{
    Section("async submit / pump / poll");

    BoxWorld world;
    world.solids.push_back(AABB{{19.0f, -60.0f, -60.0f}, {21.0f, 60.0f, 4.0f}}); // wall with a gap above

    NavService svc;
    svc.SetGameThread();
    svc.SetProbe(&world);
    CHECK(svc.CreateVolume("test", MakeConfig(1.0f, 0.4f, 0.5f, 150.0f)));

    PathQuery q;
    q.start = {0.0f, 0.0f, 0.0f};
    q.goal = {40.0f, 0.0f, 0.0f};

    std::atomic<int> callbackCount{0};
    std::atomic<uint32_t> callbackId{0};

    const uint32_t id = svc.SubmitAsync("test", q, [&](uint32_t aId, const PathResult&) {
        callbackCount.fetch_add(1);
        callbackId.store(aId);
    });
    CHECK(id != 0);

    PathResult r;
    const Status s = RunAsync(svc, id, r);

    std::printf("  status=%s points=%zu parks=%d chunks=%d\n", ToString(s), r.points.size(), r.parkCount,
                r.chunksSampled);
    CHECK(s == Status::Success);
    CHECK(r.points.size() >= 2);

    // The callback must have fired exactly once, on the pumping (game) thread.
    svc.Pump();
    CHECK(callbackCount.load() == 1);
    CHECK(callbackId.load() == id);

    // Polling a terminal job is non-consuming.
    PathResult again;
    CHECK(svc.Poll(id, again) == Status::Success);
    CHECK(again.points.size() == r.points.size());
}

void TestCancel()
{
    Section("cancellation");

    BoxWorld world;
    NavService svc;
    svc.SetGameThread();
    svc.SetProbe(&world);
    CHECK(svc.CreateVolume("test", MakeConfig(1.0f, 0.4f, 0.5f, 400.0f)));

    PathQuery q;
    q.start = {0.0f, 0.0f, 0.0f};
    q.goal = {300.0f, 300.0f, 100.0f};

    const uint32_t id = svc.SubmitAsync("test", q);
    CHECK(id != 0);
    CHECK(svc.Cancel(id));

    PathResult r;
    Status s = Status::Pending;
    for (int i = 0; i < 5000 && !IsTerminal(s); ++i)
    {
        svc.Pump(1000000);
        s = svc.Poll(id, r);
        std::this_thread::sleep_for(std::chrono::microseconds(200));
    }
    CHECK(s == Status::Cancelled);
    CHECK(!svc.Cancel(id)); // already terminal
}

void TestInvalidation()
{
    Section("invalidation re-samples changed geometry");

    BoxWorld world;
    NavService svc;
    svc.SetGameThread();
    svc.SetProbe(&world);
    CHECK(svc.CreateVolume("test", MakeConfig(1.0f, 0.4f, 0.5f, 100.0f)));

    PathQuery q;
    q.start = {0.0f, 0.0f, 0.0f};
    q.goal = {40.0f, 0.0f, 0.0f};

    PathResult before;
    CHECK(svc.FindPathSync("test", q, before) == Status::Success);
    const float openLength = before.length;

    // Drop a wall in and tell the service the world changed.
    world.solids.push_back(AABB{{19.0f, -8.0f, -8.0f}, {21.0f, 8.0f, 8.0f}});
    svc.InvalidateBox("test", AABB{{15.0f, -12.0f, -12.0f}, {25.0f, 12.0f, 12.0f}});

    PathResult after;
    const Status s = svc.FindPathSync("test", q, after);
    std::printf("  before=%.2f after=%.2f status=%s\n", static_cast<double>(openLength),
                static_cast<double>(after.length), ToString(s));

    CHECK(s == Status::Success);
    // Detouring around the obstacle must cost more than the straight shot did.
    CHECK(after.length > openLength + 1.0f);

    // Without invalidation the stale chunks would still say "open"; prove the cache is honest by
    // clearing everything and confirming the same answer.
    svc.InvalidateAll("test");
    PathResult fresh;
    CHECK(svc.FindPathSync("test", q, fresh) == Status::Success);
    CHECK(fresh.length > openLength + 1.0f);
}

void TestBadInput()
{
    Section("bad input is rejected, not crashed on");

    BoxWorld world;
    NavService svc;
    svc.SetGameThread();
    svc.SetProbe(&world);
    CHECK(svc.CreateVolume("test", MakeConfig(1.0f, 0.4f, 0.5f, 50.0f)));

    PathResult r;

    PathQuery outside;
    outside.start = {0.0f, 0.0f, 0.0f};
    outside.goal = {5000.0f, 0.0f, 0.0f}; // beyond volume bounds
    CHECK(svc.FindPathSync("test", outside, r) == Status::BadInput);

    PathQuery nan;
    nan.start = {0.0f, 0.0f, 0.0f};
    nan.goal = {std::numeric_limits<float>::quiet_NaN(), 0.0f, 0.0f};
    CHECK(svc.FindPathSync("test", nan, r) == Status::BadInput);

    PathQuery ok;
    ok.start = {0.0f, 0.0f, 0.0f};
    ok.goal = {10.0f, 0.0f, 0.0f};
    CHECK(svc.FindPathSync("nope", ok, r) == Status::NoVolume);
    CHECK(svc.SubmitAsync("nope", ok) == 0);

    // Invalid volume configs must be refused up front.
    VolumeConfig bad = MakeConfig(1.0f, 0.4f, 0.5f, 50.0f);
    bad.voxelSize = 0.0f;
    CHECK(!svc.CreateVolume("bad", bad));
}

void TestNoProbe()
{
    Section("no world attached");

    NavService svc;
    svc.SetGameThread();
    svc.SetProbe(nullptr);
    CHECK(svc.CreateVolume("test", MakeConfig(1.0f, 0.4f, 0.5f, 100.0f)));

    PathQuery q;
    q.start = {0.0f, 0.0f, 0.0f};
    q.goal = {20.0f, 0.0f, 0.0f};

    // Sync on the game thread falls back to the null probe: empty space everywhere.
    PathResult r;
    CHECK(svc.FindPathSync("test", q, r) == Status::Success);

    // Async must not spin the CPU or crash while parked with no world; it just waits.
    const uint32_t id = svc.SubmitAsync("test", q);
    CHECK(id != 0);
    for (int i = 0; i < 20; ++i)
    {
        svc.Pump();
        std::this_thread::sleep_for(std::chrono::milliseconds(1));
    }
    PathResult pr;
    const Status s = svc.Poll(id, pr);
    CHECK(s != Status::Invalid);
    svc.Cancel(id);
}

void TestConcurrentSubmitters()
{
    Section("concurrent submitters (threading contract)");

    BoxWorld world;
    world.solids.push_back(AABB{{19.0f, -40.0f, -40.0f}, {21.0f, 40.0f, 2.0f}});
    world.solids.push_back(AABB{{-40.0f, 19.0f, -40.0f}, {40.0f, 21.0f, 2.0f}});

    NavService svc;
    svc.SetGameThread();
    svc.SetProbe(&world);
    CHECK(svc.CreateVolume("a", MakeConfig(1.0f, 0.4f, 0.5f, 120.0f)));
    CHECK(svc.CreateVolume("b", MakeConfig(2.0f, 1.0f, 1.0f, 120.0f)));

    constexpr int kThreads = 6;
    constexpr int kPerThread = 12;

    std::atomic<bool> stop{false};
    std::atomic<int> submitted{0};
    std::vector<std::vector<uint32_t>> ids(kThreads);
    std::vector<std::thread> submitters;

    for (int t = 0; t < kThreads; ++t)
    {
        submitters.emplace_back([&, t] {
            for (int i = 0; i < kPerThread; ++i)
            {
                PathQuery q;
                const float f = static_cast<float>(i);
                q.start = {-30.0f + f, -30.0f, 0.0f};
                q.goal = {30.0f, 30.0f - f, 0.0f};
                q.timeoutSeconds = 30.0f;

                const uint32_t id = svc.SubmitAsync((t % 2) ? "a" : "b", q);
                if (id != 0)
                {
                    ids[static_cast<size_t>(t)].push_back(id);
                    submitted.fetch_add(1);
                }

                // Hammer the read-side APIs from a non-game thread at the same time.
                PathResult tmp;
                svc.Poll(id, tmp);
                svc.GetStats();
                svc.HasVolume("a");
            }
        });
    }

    // Meanwhile the "game thread" pumps, exactly as it would in a frame loop.
    for (int frame = 0; frame < 6000 && !stop.load(); ++frame)
    {
        svc.Pump(1000000);
        std::this_thread::sleep_for(std::chrono::microseconds(200));

        if (frame > 40)
        {
            bool allDone = submitted.load() >= kThreads * kPerThread;
            if (allDone)
            {
                for (const auto& list : ids)
                {
                    for (uint32_t id : list)
                    {
                        PathResult tmp;
                        if (!IsTerminal(svc.Poll(id, tmp)))
                        {
                            allDone = false;
                            break;
                        }
                    }
                    if (!allDone)
                    {
                        break;
                    }
                }
            }
            if (allDone)
            {
                stop.store(true);
            }
        }
    }

    for (auto& t : submitters)
    {
        t.join();
    }
    svc.Pump(1000000);

    int terminal = 0;
    int usable = 0;
    int total = 0;
    for (const auto& list : ids)
    {
        for (uint32_t id : list)
        {
            ++total;
            PathResult r;
            const Status s = svc.Poll(id, r);
            if (IsTerminal(s))
            {
                ++terminal;
            }
            if (IsUsable(s))
            {
                ++usable;
            }
        }
    }

    std::printf("  submitted=%d terminal=%d usable=%d\n", total, terminal, usable);
    CHECK(total == kThreads * kPerThread);
    CHECK(terminal == total);
    CHECK(usable == total);
}

void TestSyncOffGameThread()
{
    Section("sync from a non-game thread");

    BoxWorld world;
    NavService svc;
    svc.SetGameThread();
    svc.SetProbe(&world);
    CHECK(svc.CreateVolume("test", MakeConfig(1.0f, 0.4f, 0.5f, 100.0f)));

    std::atomic<bool> done{false};
    Status result = Status::Invalid;

    std::thread caller([&] {
        PathQuery q;
        q.start = {0.0f, 0.0f, 0.0f};
        q.goal = {30.0f, 0.0f, 0.0f};
        q.timeoutSeconds = 15.0f;

        PathResult r;
        result = svc.FindPathSync("test", q, r);
        done.store(true);
    });

    for (int i = 0; i < 6000 && !done.load(); ++i)
    {
        svc.Pump(1000000);
        std::this_thread::sleep_for(std::chrono::microseconds(200));
    }
    caller.join();

    std::printf("  status=%s\n", ToString(result));
    CHECK(result == Status::Success);
}

void TestShutdownWithLiveJobs()
{
    Section("shutdown with jobs in flight");

    BoxWorld world;
    auto svc = std::make_unique<NavService>();
    svc->SetGameThread();
    svc->SetProbe(&world);
    CHECK(svc->CreateVolume("test", MakeConfig(1.0f, 0.4f, 0.5f, 400.0f)));

    PathQuery q;
    q.start = {0.0f, 0.0f, 0.0f};
    q.goal = {350.0f, 350.0f, 50.0f};

    for (int i = 0; i < 32; ++i)
    {
        svc->SubmitAsync("test", q);
    }

    // Destroying the service while workers are mid-search must not hang or crash.
    svc.reset();
    CHECK(true);
}
// -------------------------------------------------------------------------------------------
// Baked volumes
// -------------------------------------------------------------------------------------------

// The bake stores occupancy and recovers clearance with a distance transform at load. That
// transform reaches across chunk boundaries, which is the part most likely to be silently wrong:
// a wall one voxel outside a chunk still has to shorten the clearance inside it. So the results
// are checked against brute force over the solids actually written, not against themselves.
void TestBakedNav()
{
    std::printf("\nTestBakedNav\n");

    const std::string path = "vnav_test_bake.vnavbake";

    // A solid slab filling the top layer of chunk (0,0,0), and a lone solid voxel inside
    // chunk (0,0,1). Chunk (0,0,1) is therefore only reachable-correct if the loader consults
    // its neighbour below.
    std::vector<IVec3> solids;
    for (int32_t y = 0; y < kChunkDim; ++y)
    {
        for (int32_t x = 0; x < kChunkDim; ++x)
        {
            solids.push_back(IVec3{x, y, kChunkDim - 1}); // top layer of chunk (0,0,0)
        }
    }
    solids.push_back(IVec3{3, 3, kChunkDim + 9}); // inside chunk (0,0,1)

    // A distant solid, purely to stretch the covered region past the empty chunks between. It is
    // what makes "inside the region but absent from the index" a case that can be tested at all.
    solids.push_back(IVec3{5, 5, 6 * kChunkDim + 2}); // inside chunk (0,0,6)

    {
        BakedNavWriter writer;
        std::string err;
        CHECK(writer.Open(path, 1.0f, AABB{Vec3{-64.0f, -64.0f, -64.0f}, Vec3{192.0f, 64.0f, 64.0f}}, err));

        auto emit = [&](const IVec3& chunkCoord)
        {
            std::vector<uint8_t> occ(static_cast<size_t>(kOccupancyBytes), 0);
            for (const IVec3& s : solids)
            {
                if (Volume::VoxelToChunk(s) != chunkCoord)
                {
                    continue;
                }
                const IVec3 l = Volume::VoxelLocal(s);
                const int32_t idx = Chunk::LocalIndex(l.x, l.y, l.z);
                occ[static_cast<size_t>(idx >> 3)] |= static_cast<uint8_t>(1u << (idx & 7));
            }
            CHECK(writer.AddChunk(chunkCoord, occ.data()));
        };

        emit(IVec3{0, 0, 0});
        emit(IVec3{0, 0, 1});
        emit(IVec3{0, 0, 2}); // all open - the writer drops it, which is the point
        emit(IVec3{0, 0, 6});

        CHECK(writer.Close(err));
        CHECK(writer.EmptyChunksSkipped() == 1);
    }

    BakedNav nav;
    std::string err;
    CHECK(nav.Open(path, err));
    CHECK(nav.IsOpen());
    CHECK(nav.Covers(IVec3{0, 0, 1}));
    CHECK(!nav.Covers(IVec3{50, 50, 50}));

    const uint8_t halo = 6;
    auto chunk = nav.Load(IVec3{0, 0, 1}, halo);
    CHECK(chunk != nullptr);

    // Brute-force reference over every solid written anywhere.
    int mismatches = 0;
    int sawShortened = 0;
    for (int32_t lz = 0; lz < kChunkDim; ++lz)
    {
        for (int32_t ly = 0; ly < kChunkDim; ++ly)
        {
            for (int32_t lx = 0; lx < kChunkDim; ++lx)
            {
                const IVec3 world{lx, ly, kChunkDim + lz};

                double best = 1e30;
                for (const IVec3& s : solids)
                {
                    const double dx = world.x - s.x;
                    const double dy = world.y - s.y;
                    const double dz = world.z - s.z;
                    best = std::min(best, std::sqrt(dx * dx + dy * dy + dz * dz));
                }

                const uint8_t expected = best >= static_cast<double>(halo)
                                             ? kMaxClearance
                                             : static_cast<uint8_t>(static_cast<int32_t>(best));
                const uint8_t actual = chunk->ClearanceLocal(Chunk::LocalIndex(lx, ly, lz));

                if (actual != expected)
                {
                    ++mismatches;
                }
                if (expected != kMaxClearance)
                {
                    ++sawShortened;
                }
            }
        }
    }

    CHECK(mismatches == 0);
    CHECK(sawShortened > 0); // the halo really did reach across the boundary

    // Inside the baked region but absent from the index: open sky, not "unknown".
    auto open = nav.Load(IVec3{0, 0, 3}, 2);
    CHECK(open != nullptr);
    CHECK(open->ClearanceLocal(Chunk::LocalIndex(8, 8, 8)) == kMaxClearance);

    // Outside the baked region entirely: nullptr, so the caller knows to probe instead of
    // assuming the space is clear.
    CHECK(nav.Load(IVec3{99, 99, 99}, halo) == nullptr);

    nav.Close();
    std::remove(path.c_str());
}

// The whole point of baking: planning through real geometry with NO probe attached. In the game
// that is the case where the physics scene has not streamed the buildings in yet and probing
// would confidently report open air.
void TestBakedRoutingWithoutProbe()
{
    std::printf("\nTestBakedRoutingWithoutProbe\n");

    const std::string path = "vnav_test_route.vnavbake";
    const AABB region{Vec3{-64.0f, -64.0f, -64.0f}, Vec3{64.0f, 64.0f, 64.0f}};

    // A wall across x = 20 with a small window in it.
    const auto isSolid = [](const IVec3& v)
    {
        if (v.x != 20)
        {
            return false;
        }
        const bool window = v.y >= 0 && v.y <= 3 && v.z >= 0 && v.z <= 3;
        return !window;
    };

    {
        BakedNavWriter writer;
        std::string err;
        CHECK(writer.Open(path, 1.0f, region, err));

        for (int32_t cz = -4; cz < 4; ++cz)
        {
            for (int32_t cy = -4; cy < 4; ++cy)
            {
                for (int32_t cx = -4; cx < 4; ++cx)
                {
                    std::vector<uint8_t> occ(static_cast<size_t>(kOccupancyBytes), 0);
                    for (int32_t lz = 0; lz < kChunkDim; ++lz)
                    {
                        for (int32_t ly = 0; ly < kChunkDim; ++ly)
                        {
                            for (int32_t lx = 0; lx < kChunkDim; ++lx)
                            {
                                const IVec3 world{cx * kChunkDim + lx, cy * kChunkDim + ly, cz * kChunkDim + lz};
                                if (!isSolid(world))
                                {
                                    continue;
                                }
                                const int32_t idx = Chunk::LocalIndex(lx, ly, lz);
                                occ[static_cast<size_t>(idx >> 3)] |= static_cast<uint8_t>(1u << (idx & 7));
                            }
                        }
                    }
                    CHECK(writer.AddChunk(IVec3{cx, cy, cz}, occ.data()));
                }
            }
        }

        CHECK(writer.Close(err));
    }

    ServiceConfig sc;
    auto svc = std::make_unique<NavService>(sc);
    svc->SetGameThread();

    // Deliberately no probe. Every chunk has to come from the bake.
    svc->SetProbe(nullptr);

    VolumeConfig vc;
    vc.voxelSize = 1.0f;
    vc.agentRadius = 0.5f;
    vc.margin = 0.5f;
    vc.bounds = region;
    CHECK(svc->CreateVolume("baked", vc));

    std::string err;
    CHECK(svc->AttachBake("baked", path, err));
    CHECK(svc->HasBake("baked"));

    // Deliberately off the window's axis, so a straight run would hit the wall and the route has
    // to divert through the opening.
    PathQuery q;
    q.start = Vec3{5.5f, -12.5f, -12.5f};
    q.goal = Vec3{35.5f, -12.5f, -12.5f};
    q.timeoutSeconds = 10.0f;

    const uint32_t job = svc->SubmitAsync("baked", q);
    CHECK(job != 0);

    PathResult result;
    Status status = Status::Pending;
    for (int i = 0; i < 4000 && !IsTerminal(status); ++i)
    {
        svc->Pump(4000);
        status = svc->Poll(job, result);
        std::this_thread::sleep_for(std::chrono::milliseconds(1));
    }

    CHECK(status == Status::Success);
    CHECK(!result.points.empty());

    const Stats stats = svc->GetStats();
    CHECK(stats.chunksFromBake > 0);
    CHECK(stats.totalProbes == 0); // nothing was probed - it all came from the bake

    // The wall is real: the straight line is 30 m, and going via the window is ~45 m.
    CHECK(result.length > 35.0f);

    // Every waypoint must be clear of the wall plane except where the window is.
    for (const Vec3& p : result.points)
    {
        const bool atWall = p.x > 20.0f && p.x < 21.0f;
        if (atWall)
        {
            CHECK(p.y > -1.0f && p.y < 5.0f && p.z > -1.0f && p.z < 5.0f);
        }
    }

    svc.reset();
    std::remove(path.c_str());
}

// The codec must be exact - it carries occupancy, and a wrong bit is a wall that is not there
// or, worse, one that is missing.
void TestBakeCodec()
{
    std::printf("\nTestBakeCodec\n");

    std::vector<uint8_t> original(static_cast<size_t>(kOccupancyBytes));
    std::vector<uint8_t> decoded(static_cast<size_t>(kOccupancyBytes));
    std::vector<uint8_t> payload;

    // Uniform, sparse, striped, and pseudo-random - the last one forces the raw fallback.
    for (int pattern = 0; pattern < 4; ++pattern)
    {
        uint32_t seed = 12345u;
        for (int32_t i = 0; i < kOccupancyBytes; ++i)
        {
            switch (pattern)
            {
            case 0: original[static_cast<size_t>(i)] = 0xFF; break;
            case 1: original[static_cast<size_t>(i)] = (i % 64 == 0) ? 0x01 : 0x00; break;
            case 2: original[static_cast<size_t>(i)] = (i % 2) ? 0xF0 : 0x0F; break;
            default:
                seed = seed * 1664525u + 1013904223u;
                original[static_cast<size_t>(i)] = static_cast<uint8_t>(seed >> 24);
                break;
            }
        }

        const size_t bytes = bakecodec::Encode(original.data(), payload);
        CHECK(bytes <= static_cast<size_t>(kOccupancyBytes)); // never expands
        CHECK(bakecodec::Decode(payload.data(), bytes, decoded.data()));
        CHECK(decoded == original);
    }
}
void TestPerWaypointClearance()
{
    Section("per-waypoint clearance");

    BoxWorld world;
    // A slab well off to one side, so waypoints near it must report less room than open sky.
    world.solids.push_back(AABB{{-100.0f, 8.0f, -100.0f}, {100.0f, 40.0f, 100.0f}});

    NavService svc;
    svc.SetGameThread();
    svc.SetProbe(&world);
    CHECK(svc.CreateVolume("clear", MakeConfig(1.0f, 0.5f, 0.5f, 120.0f)));

    PathQuery q;
    q.start = {-30.0f, 0.0f, 0.0f};
    q.goal = {30.0f, 0.0f, 0.0f};

    PathResult r;
    CHECK(svc.FindPathSync("clear", q, r) == Status::Success);

    // The contract callers rely on: one clearance per point, always.
    CHECK(r.clearances.size() == r.points.size());

    for (float c : r.clearances)
    {
        CHECK(c >= 0.0f);
    }

    // A path 8 m clear of the only obstacle must report real room, not a floor value.
    float minC = 1e9f;
    for (float c : r.clearances)
    {
        minC = std::min(minC, c);
    }
    CHECK(minC > 1.0f);

    // A failed query must not hand back clearances describing points it already dropped.
    PathQuery bad = q;
    bad.start = {9999.0f, 9999.0f, 9999.0f};
    PathResult rb;
    svc.FindPathSync("clear", bad, rb);
    CHECK(rb.clearances.size() == rb.points.size());

    // --- graded distance, which ONLY the baked path can provide -------------------------------
    //
    // A physics probe answers one bit, so probe-sampled chunks carry just three levels (blocked /
    // squeezing / open) and anything not scraping a wall saturates. A bake carries a real EDT.
    // This asserts the difference rather than assuming it: two parallel routes at different
    // heights above the same floor must report DIFFERENT clearance, which is impossible under the
    // tri-state approximation.
    const std::string bakePath = "vnav_test_clearance.vnavbake";
    const AABB region{Vec3{-64.0f, -64.0f, -64.0f}, Vec3{64.0f, 64.0f, 64.0f}};

    {
        BakedNavWriter writer;
        std::string err;
        CHECK(writer.Open(bakePath, 1.0f, region, err));

        for (int32_t cz = -4; cz < 4; ++cz)
        {
            for (int32_t cy = -4; cy < 4; ++cy)
            {
                for (int32_t cx = -4; cx < 4; ++cx)
                {
                    std::vector<uint8_t> occ(static_cast<size_t>(kOccupancyBytes), 0);
                    for (int32_t lz = 0; lz < kChunkDim; ++lz)
                    {
                        for (int32_t ly = 0; ly < kChunkDim; ++ly)
                        {
                            for (int32_t lx = 0; lx < kChunkDim; ++lx)
                            {
                                // A single solid floor plane at z == 0.
                                if (cz * kChunkDim + lz != 0)
                                {
                                    continue;
                                }
                                const int32_t idx = Chunk::LocalIndex(lx, ly, lz);
                                occ[static_cast<size_t>(idx >> 3)] |= static_cast<uint8_t>(1u << (idx & 7));
                            }
                        }
                    }
                    CHECK(writer.AddChunk(IVec3{cx, cy, cz}, occ.data()));
                }
            }
        }
        CHECK(writer.Close(err));
    }

    ServiceConfig sc;
    NavService bsvc(sc);
    bsvc.SetGameThread();
    bsvc.SetProbe(nullptr); // every chunk must come from the bake

    VolumeConfig bvc;
    bvc.voxelSize = 1.0f;
    bvc.agentRadius = 0.5f;
    bvc.margin = 0.5f;
    bvc.bounds = region;
    CHECK(bsvc.CreateVolume("bakedclear", bvc));

    std::string berr;
    CHECK(bsvc.AttachBake("bakedclear", bakePath, berr));

    const auto minClearanceAt = [&](float aZ)
    {
        PathQuery bq;
        bq.start = Vec3{-20.5f, 0.5f, aZ};
        bq.goal = Vec3{20.5f, 0.5f, aZ};
        bq.maxProbesPerQuery = 100000000;

        PathResult br;
        const Status bs = bsvc.FindPathSync("bakedclear", bq, br);
        CHECK(bs == Status::Success);
        CHECK(br.clearances.size() == br.points.size());

        float m = 1e9f;
        for (float c : br.clearances)
        {
            m = std::min(m, c);
        }
        return m;
    };

    // Grading only exists BELOW the tight-threshold cut-off. With agentRadius 0.5 + margin 0.5 the
    // halo is ~2 voxels, so a route hugging the floor grades while one well clear of it saturates.
    // Asserting both is what pins the actual contract: a tightness signal with a near cut-off,
    // not a rangefinder.
    const float hugging = minClearanceAt(1.5f); // 1 voxel above the floor - inside the cut-off
    const float clearOfIt = minClearanceAt(4.5f); // 4 voxels up - beyond it, saturated
    std::printf("  baked: z=1.5 -> %.2f m (graded),  z=4.5 -> %.2f m (saturated)\n",
                static_cast<double>(hugging), static_cast<double>(clearOfIt));

    CHECK(hugging > 0.0f);
    CHECK(clearOfIt > hugging);
    // Saturation really is reached this close in - this is the surprising half of the contract.
    CHECK(clearOfIt > 100.0f);
    // ...and the graded half is a genuinely small number, not another saturated one.
    CHECK(hugging < 5.0f);

}

void TestTurnClearance()
{
    Section("turn clearance for an elongated agent");

    // An L-shaped corridor. A sphere rounds the inside corner happily; a long body has to rotate
    // 90 degrees there, sweeping its nose and tail well outside the capsule the search validated.
    BoxWorld world;
    const float t = 40.0f;
    // Leg A runs along +x at y in [-4,4]; leg B runs along +y at x in [16,24].
    world.solids.push_back(AABB{{-t, 4.0f, -t}, {16.0f, t, t}});    // outer wall of the bend
    world.solids.push_back(AABB{{-t, -t, -t}, {24.0f, -4.0f, t}});  // near wall of leg A
    world.solids.push_back(AABB{{24.0f, -t, -t}, {t, t, t}});       // far wall of leg B
    world.solids.push_back(AABB{{-t, -t, -t}, {t, t, -4.0f}});      // floor
    world.solids.push_back(AABB{{-t, -t, 4.0f}, {t, t, t}});        // ceiling

    NavService svc;
    svc.SetGameThread();
    svc.SetProbe(&world);
    CHECK(svc.CreateVolume("bend", MakeConfig(1.0f, 1.0f, 0.5f, 64.0f)));

    PathQuery base;
    base.start = {0.0f, 0.0f, 0.0f};
    base.goal = {20.0f, 20.0f, 0.0f};
    base.maxProbesPerQuery = 100000000;

    // Sphere agent: the historical behaviour, and the control for the comparison.
    PathResult sphere;
    const Status ss = svc.FindPathSync("bend", base, sphere);
    CHECK(IsUsable(ss));
    CHECK(sphere.infeasibleTurns == 0); // never reported for a sphere agent

    // Same route, but the agent is now 16 m long in an 8 m corridor. It cannot possibly pivot
    // through the bend, and the library must say so instead of returning a path it cannot fly.
    PathQuery longAgent = base;
    longAgent.agentLength = 16.0f;

    PathResult elongated;
    const Status ls = svc.FindPathSync("bend", longAgent, elongated);
    std::printf("  sphere: %s turns=%d pts=%zu   16m body: %s turns=%d pts=%zu\n", ToString(ss),
                sphere.infeasibleTurns, sphere.points.size(), ToString(ls), elongated.infeasibleTurns,
                elongated.points.size());
    CHECK(IsUsable(ls));
    CHECK(elongated.infeasibleTurns > 0);

    // A body that comfortably fits the corridor must NOT be flagged - otherwise the check is just
    // rejecting everything and the test above would pass for the wrong reason.
    PathQuery shortAgent = base;
    shortAgent.agentLength = 2.0f;

    PathResult small;
    const Status shs = svc.FindPathSync("bend", shortAgent, small);
    CHECK(IsUsable(shs));
    std::printf("  2m body: %s turns=%d\n", ToString(shs), small.infeasibleTurns);
    CHECK(small.infeasibleTurns == 0);
}

void TestYawSearch()
{
    Section("4D (x,y,z,yaw) search - the large non-sphere tier");

    // Same L-bend as the turn-clearance test. The two tiers should disagree about it, and that
    // disagreement is the whole point of offering both.
    BoxWorld world;
    const float t = 40.0f;
    world.solids.push_back(AABB{{-t, 4.0f, -t}, {16.0f, t, t}});
    world.solids.push_back(AABB{{-t, -t, -t}, {24.0f, -4.0f, t}});
    world.solids.push_back(AABB{{24.0f, -t, -t}, {t, t, t}});
    world.solids.push_back(AABB{{-t, -t, -t}, {t, t, -4.0f}});
    world.solids.push_back(AABB{{-t, -t, 4.0f}, {t, t, t}});

    NavService svc;
    svc.SetGameThread();
    svc.SetProbe(&world);
    CHECK(svc.CreateVolume("yaw", MakeConfig(1.0f, 1.0f, 0.5f, 64.0f)));

    PathQuery base;
    base.start = {0.0f, 0.0f, 0.0f};
    base.goal = {20.0f, 20.0f, 0.0f};
    base.maxProbesPerQuery = 100000000;
    base.maxExpansions = 4000000;

    // A sphere rounds the bend without trouble - the control.
    PathResult sphere;
    CHECK(IsUsable(svc.FindPathSync("yaw", base, sphere)));
    CHECK(sphere.yaws.empty()); // 3D never emits an orientation plan

    // Tier 1 (agentLength only): returns the route, and reports that the corners do not work.
    PathQuery tier1 = base;
    tier1.agentLength = 16.0f;
    PathResult r1;
    const Status s1 = svc.FindPathSync("yaw", tier1, r1);
    CHECK(IsUsable(s1));
    CHECK(r1.infeasibleTurns > 0);
    CHECK(r1.yaws.empty());

    // Tier 2 (4D): searches orientation, so it never proposes the route at all. A 16 m body can
    // lie along either leg but cannot rotate between them in an 8 m corridor, and unlike tier 1
    // the search knows that rather than discovering it afterwards.
    PathQuery tier2 = tier1;
    tier2.yawBuckets = 8;
    PathResult r2;
    const Status s2 = svc.FindPathSync("yaw", tier2, r2);
    std::printf("  16m body: tier1 %s (turns=%d)   tier2/4D %s\n", ToString(s1), r1.infeasibleTurns,
                ToString(s2));
    CHECK(s2 != Status::Success);

    // ...and it must not simply reject everything: a body that fits rounds the bend and comes back
    // with a full orientation plan.
    PathQuery small = base;
    small.agentLength = 2.0f;
    small.yawBuckets = 8;
    PathResult r3;
    const Status s3 = svc.FindPathSync("yaw", small, r3);
    CHECK(IsUsable(s3));
    CHECK(!r3.yaws.empty());
    CHECK(r3.yaws.size() == r3.points.size()); // the contract callers rely on
    std::printf("  2m body 4D: %s pts=%zu yaws=%zu exp=%d\n", ToString(s3), r3.points.size(),
                r3.yaws.size(), r3.expansions);

    // The plan has to actually turn: entering along +x and leaving along +y means the heading must
    // change by roughly a right angle somewhere, or the fourth dimension is doing nothing.
    float minYaw = 1e9f;
    float maxYaw = -1e9f;
    for (float y : r3.yaws)
    {
        minYaw = std::min(minYaw, y);
        maxYaw = std::max(maxYaw, y);
    }
    std::printf("  yaw range: %.2f .. %.2f rad\n", static_cast<double>(minYaw), static_cast<double>(maxYaw));
    CHECK(maxYaw - minYaw > 1.0f);
}

// --- bidirectional search -------------------------------------------------------------------

// Shortest distance from a point to a solid box, 0 inside it.
float DistToBox(const Vec3& aP, const AABB& aB)
{
    const float dx = std::max({aB.min.x - aP.x, 0.0f, aP.x - aB.max.x});
    const float dy = std::max({aB.min.y - aP.y, 0.0f, aP.y - aB.max.y});
    const float dz = std::max({aB.min.z - aP.z, 0.0f, aP.z - aB.max.z});
    return std::sqrt(dx * dx + dy * dy + dz * dz);
}

// Walk the emitted path densely and confirm it never comes closer to a solid than the agent
// radius. This is the check that actually matters for bidirectional: CanStep is direction
// -sensitive, so a backward frontier that evaluated edges in the wrong order would still report
// Success while threading the agent through a corner. Only geometry catches that.
float MinClearanceAlongPath(const std::vector<Vec3>& aPts, const std::vector<AABB>& aSolids, float aStep)
{
    float worst = 1e9f;
    for (size_t i = 1; i < aPts.size(); ++i)
    {
        const Vec3 a = aPts[i - 1];
        const Vec3 b = aPts[i];
        const float len = (b - a).Length();
        const int steps = std::max(1, static_cast<int>(len / aStep));
        for (int k = 0; k <= steps; ++k)
        {
            const float t = static_cast<float>(k) / static_cast<float>(steps);
            const Vec3 p{a.x + (b.x - a.x) * t, a.y + (b.y - a.y) * t, a.z + (b.z - a.z) * t};
            for (const AABB& s : aSolids)
            {
                worst = std::min(worst, DistToBox(p, s));
            }
        }
    }
    return worst;
}

void TestFilletSmoothing()
{
    Section("fillet smoothing");

    // A wall forcing one hard corner, with room around it for a fillet to use.
    BoxWorld world;
    world.solids.push_back(AABB{{18.0f, -60.0f, -60.0f}, {22.0f, 4.0f, 60.0f}});

    NavService svc;
    svc.SetGameThread();
    svc.SetProbe(&world);
    CHECK(svc.CreateVolume("fillet", MakeConfig(1.0f, 0.5f, 0.5f, 90.0f)));

    PathQuery base;
    base.start = {0.0f, 0.0f, 0.0f};
    base.goal = {40.0f, 0.0f, 0.0f};
    base.maxProbesPerQuery = 100000000;

    PathResult pulled;
    base.smoothing = Smoothing::StringPull;
    CHECK(svc.FindPathSync("fillet", base, pulled) == Status::Success);

    PathQuery filletQ = base;
    filletQ.smoothing = Smoothing::Fillet;
    filletQ.maxTurnRadius = 6.0f;

    PathResult filleted;
    CHECK(svc.FindPathSync("fillet", filletQ, filleted) == Status::Success);

    // Rounding a corner adds points and cannot lengthen the path - cutting a corner is strictly
    // shorter than going through it.
    CHECK(filleted.points.size() > pulled.points.size());
    CHECK(filleted.length <= pulled.length + 0.01f);

    // Sharpest corner in each. The whole point is that this drops.
    const auto sharpestTurn = [](const std::vector<Vec3>& p)
    {
        float worst = 0.0f;
        for (size_t i = 1; i + 1 < p.size(); ++i)
        {
            const Vec3 a = (p[i] - p[i - 1]).Normalized();
            const Vec3 b = (p[i + 1] - p[i]).Normalized();
            worst = std::max(worst, std::acos(std::clamp(a.Dot(b), -1.0f, 1.0f)));
        }
        return worst;
    };

    const float beforeTurn = sharpestTurn(pulled.points);
    const float afterTurn = sharpestTurn(filleted.points);
    std::printf("  pts %zu -> %zu, len %.2f -> %.2f, sharpest turn %.2f -> %.2f rad\n", pulled.points.size(),
                filleted.points.size(), static_cast<double>(pulled.length), static_cast<double>(filleted.length),
                static_cast<double>(beforeTurn), static_cast<double>(afterTurn));
    CHECK(afterTurn < beforeTurn);

    // Rounding must not cut INTO the wall. This is the failure mode that makes a Catmull-Rom pass
    // unusable here, so it is the thing actually worth asserting.
    const float clear = MinClearanceAlongPath(filleted.points, world.solids, 0.2f);
    CHECK(clear >= 0.5f * 0.9f);

    // Straight runs have nothing to round, so they must come back untouched rather than gaining
    // spurious points.
    BoxWorld empty;
    NavService svc2;
    svc2.SetGameThread();
    svc2.SetProbe(&empty);
    CHECK(svc2.CreateVolume("open", MakeConfig(1.0f, 0.5f, 0.5f, 90.0f)));

    PathQuery straight = filletQ;
    straight.start = {0.0f, 0.0f, 0.0f};
    straight.goal = {40.0f, 0.0f, 0.0f};

    PathResult sr;
    CHECK(svc2.FindPathSync("open", straight, sr) == Status::Success);
    CHECK(sr.points.size() <= 4);
}

void TestDynamicObstacles()
{
    Section("dynamic obstacle layer");

    BoxWorld world; // completely empty static world
    NavService svc;
    svc.SetGameThread();
    svc.SetProbe(&world);
    CHECK(svc.CreateVolume("dyn", MakeConfig(1.0f, 0.5f, 0.5f, 120.0f)));

    PathQuery q;
    q.start = {-30.0f, 0.0f, 0.0f};
    q.goal = {30.0f, 0.0f, 0.0f};
    q.maxProbesPerQuery = 100000000;

    PathResult before;
    CHECK(svc.FindPathSync("dyn", q, before) == Status::Success);
    const float straightLen = before.length;

    // Drop a wall in as a dynamic obstacle, leaving a gap well off the direct line. Nothing about
    // the static world changed, so any detour has to come from the dynamic layer alone.
    std::vector<AABB> boxes;
    boxes.push_back(AABB{{-2.0f, -40.0f, -40.0f}, {2.0f, 12.0f, 40.0f}});
    boxes.push_back(AABB{{-2.0f, 20.0f, -40.0f}, {2.0f, 40.0f, 40.0f}});
    svc.SetDynamicObstacles("dyn", boxes);
    CHECK(svc.DynamicObstacleCount("dyn") == 2);

    PathResult during;
    const Status ds = svc.FindPathSync("dyn", q, during);
    CHECK(IsUsable(ds));

    // It must actually go around: a longer path, and no waypoint inside an obstacle.
    CHECK(during.length > straightLen * 1.05f);

    bool insideDynamic = false;
    for (const Vec3& p : during.points)
    {
        for (const AABB& b : boxes)
        {
            if (b.Contains(p))
            {
                insideDynamic = true;
            }
        }
    }
    CHECK(!insideDynamic);

    // Line of sight must respect them too. Without that check, string-pulling would shortcut a
    // straight line right through the wall that every node test had just rejected.
    const float clear = MinClearanceAlongPath(during.points, boxes, 0.2f);
    CHECK(clear > 0.0f);

    std::printf("  static-only len=%.2f   with dynamic wall len=%.2f (min clearance %.2f)\n",
                static_cast<double>(straightLen), static_cast<double>(during.length),
                static_cast<double>(clear));

    // Clearing restores the straight run - the layer must not have leaked into the chunk cache.
    svc.SetDynamicObstacles("dyn", {});
    CHECK(svc.DynamicObstacleCount("dyn") == 0);

    PathResult after;
    CHECK(svc.FindPathSync("dyn", q, after) == Status::Success);
    CHECK(std::fabs(after.length - straightLen) < 0.01f);
}

void TestHierarchicalGuidance()
{
    Section("hierarchical coarse guidance");

    // A THICK barrier, with the only way past it far off the direct line. Thickness is the point:
    // the coarse graph works in 16 m chunks and only treats a wholly-Solid chunk as blocked, so a
    // thin wall produces Mixed chunks it considers passable and guidance has nothing to say. City
    // buildings and terrain are chunk-scale, which is where this pays.
    const AABB region{Vec3{-128.0f, -128.0f, -128.0f}, Vec3{128.0f, 128.0f, 128.0f}};
    const std::string path = "vnav_coarse.vnavbake";

    const auto isSolid = [](const IVec3& v)
    {
        return v.x >= -24 && v.x <= 24 && v.y <= 64; // slab, open only above y = 64
    };

    {
        BakedNavWriter w;
        std::string err;
        CHECK(w.Open(path, 1.0f, region, err));

        // Only chunks that actually contain geometry need writing - absent-but-covered reads as
        // open sky, which is also what makes the coarse graph cheap to consult.
        for (int32_t cz = -8; cz < 8; ++cz)
        {
            for (int32_t cy = -8; cy <= 4; ++cy)
            {
                for (int32_t cx = -2; cx <= 1; ++cx)
                {
                    std::vector<uint8_t> occ(static_cast<size_t>(kOccupancyBytes), 0);
                    bool any = false;
                    for (int32_t lz = 0; lz < kChunkDim; ++lz)
                    {
                        for (int32_t ly = 0; ly < kChunkDim; ++ly)
                        {
                            for (int32_t lx = 0; lx < kChunkDim; ++lx)
                            {
                                const IVec3 v{cx * kChunkDim + lx, cy * kChunkDim + ly, cz * kChunkDim + lz};
                                if (!isSolid(v))
                                {
                                    continue;
                                }
                                const int32_t idx = Chunk::LocalIndex(lx, ly, lz);
                                occ[static_cast<size_t>(idx >> 3)] |= static_cast<uint8_t>(1u << (idx & 7));
                                any = true;
                            }
                        }
                    }
                    if (any)
                    {
                        CHECK(w.AddChunk(IVec3{cx, cy, cz}, occ.data()));
                    }
                }
            }
        }
        CHECK(w.Close(err));
    }

    const auto run = [&](bool aGuided, PathResult& aOut)
    {
        NavService svc;
        svc.SetGameThread();
        svc.SetProbe(nullptr); // bake only, so the comparison is purely about search effort

        VolumeConfig vc = MakeConfig(1.0f, 0.5f, 0.5f, 128.0f);
        vc.bounds = region;
        CHECK(svc.CreateVolume("c", vc));

        std::string err;
        CHECK(svc.AttachBake("c", path, err));

        PathQuery q;
        q.start = {-100.0f, 0.0f, 0.0f};
        q.goal = {100.0f, 0.0f, 0.0f};
        q.coarseGuidance = aGuided;
        q.maxExpansions = 4000000;
        q.bidirectional = false; // isolate the effect of guidance from the effect of meeting
        return svc.FindPathSync("c", q, aOut);
    };

    PathResult plain;
    PathResult guided;
    const Status ps = run(false, plain);
    const Status gs = run(true, guided);

    std::printf("  unguided: %s exp=%d len=%.1f\n", ToString(ps), plain.expansions,
                static_cast<double>(plain.length));
    std::printf("  guided:   %s exp=%d len=%.1f  (coarse exp=%d, corridor=%d chunks)\n", ToString(gs),
                guided.expansions, static_cast<double>(guided.length), guided.coarseExpansions,
                guided.corridorChunks);

    CHECK(IsUsable(ps));
    CHECK(IsUsable(gs));

    // Guidance must actually have engaged, not silently no-opped.
    CHECK(guided.coarseExpansions > 0);
    CHECK(guided.corridorChunks > 0);
    CHECK(plain.corridorChunks == 0);

    // The point of the whole exercise: reach the same place for less search.
    CHECK(guided.expansions < plain.expansions);

    // And the coarse pass has to be cheap relative to what it saves, or it is not worth having.
    CHECK(guided.coarseExpansions < plain.expansions / 10);

    // Guidance is a cost BIAS, so the route may be slightly longer - but it must not be absurd,
    // which is what would show if the corridor were pushing the path somewhere silly.
    CHECK(guided.length < plain.length * 1.5f);

    std::remove(path.c_str());
}

void TestBakeGameBuildStamp()
{
    Section("bake game-build stamp");

    const AABB region{Vec3{-32.0f, -32.0f, -32.0f}, Vec3{32.0f, 32.0f, 32.0f}};

    const auto writeBake = [&](const std::string& aPath, const std::string& aBuild)
    {
        BakedNavWriter w;
        std::string err;
        CHECK(w.Open(aPath, 1.0f, region, err));
        if (!aBuild.empty())
        {
            w.SetGameBuild(aBuild);
        }
        std::vector<uint8_t> occ(static_cast<size_t>(kOccupancyBytes), 0);
        const int32_t idx = Chunk::LocalIndex(1, 1, 1);
        occ[static_cast<size_t>(idx >> 3)] |= static_cast<uint8_t>(1u << (idx & 7));
        CHECK(w.AddChunk(IVec3{0, 0, 0}, occ.data()));
        CHECK(w.Close(err));
    };

    const std::string stamped = "vnav_stamped.vnavbake";
    const std::string unstamped = "vnav_unstamped.vnavbake";
    writeBake(stamped, "2.21-hotfix1");
    writeBake(unstamped, "");

    // Round-trips through the header's spare bytes.
    {
        BakedNav b;
        std::string err;
        CHECK(b.Open(stamped, err));
        CHECK(b.GameBuild() == "2.21-hotfix1");
        b.Close();
    }

    // An unstamped bake must still LOAD - refusing those would break every bake already shipped,
    // which is exactly why the stamp went in the spare bytes instead of behind a version bump.
    {
        BakedNav b;
        std::string err;
        CHECK(b.Open(unstamped, err));
        CHECK(b.GameBuild().empty());
        b.Close();
    }

    const auto makeService = [&]()
    {
        auto svc = std::make_unique<NavService>();
        svc->SetGameThread();
        svc->SetProbe(nullptr);
        VolumeConfig vc = MakeConfig(1.0f, 0.5f, 0.5f, 32.0f);
        vc.bounds = region;
        CHECK(svc->CreateVolume("b", vc));
        return svc;
    };

    // Matching build: attaches.
    {
        auto svc = makeService();
        std::string err;
        CHECK(svc->AttachBakeChecked("b", stamped, "2.21-hotfix1", err));
        CHECK(svc->HasBake("b"));
        CHECK(svc->BakeGameBuild("b") == "2.21-hotfix1");
    }

    // Mismatched build: REFUSED. This is the whole point - geometry may have moved, and planning
    // against a stale bake routes confidently through walls that are no longer there.
    {
        auto svc = makeService();
        std::string err;
        CHECK(!svc->AttachBakeChecked("b", stamped, "2.30", err));
        CHECK(!svc->HasBake("b")); // and it must not be briefly live either
        CHECK(!err.empty());
        std::printf("  mismatch refused: %s\n", err.c_str());
    }

    // Unstamped: attaches, but says so rather than accepting silently.
    {
        auto svc = makeService();
        std::string err;
        CHECK(svc->AttachBakeChecked("b", unstamped, "2.21-hotfix1", err));
        CHECK(svc->HasBake("b"));
        CHECK(!err.empty());
        std::printf("  unstamped warned: %s\n", err.c_str());
    }

    // No expectation supplied: plain attach, no checking.
    {
        auto svc = makeService();
        std::string err;
        CHECK(svc->AttachBakeChecked("b", stamped, "", err));
        CHECK(svc->HasBake("b"));
    }

    std::remove(stamped.c_str());
    std::remove(unstamped.c_str());
}

void TestResumeIndex()
{
    Section("resume index for continuous replanning");

    // A path running along +x. The agent is most of the way down it by the time the query returns.
    const std::vector<Vec3> path{{0.0f, 0.0f, 0.0f},  {10.0f, 0.0f, 0.0f},
                                 {20.0f, 0.0f, 0.0f}, {30.0f, 0.0f, 0.0f}};

    // The bug this exists to stop: naively steering at index 1 sends the agent BACKWARDS.
    CHECK(NavService::ResumeIndex(path, Vec3{25.0f, 0.0f, 0.0f}) == 3);
    CHECK(NavService::ResumeIndex(path, Vec3{15.0f, 0.0f, 0.0f}) == 2);
    CHECK(NavService::ResumeIndex(path, Vec3{5.0f, 0.0f, 0.0f}) == 1);

    // At the very start it must still target the first real waypoint, not index 0 (itself).
    CHECK(NavService::ResumeIndex(path, Vec3{0.0f, 0.0f, 0.0f}) == 1);

    // Off-path: the agent has drifted, but the answer still has to be ahead of it rather than a
    // rewind to the nearest WAYPOINT, which would be the whole bug again.
    CHECK(NavService::ResumeIndex(path, Vec3{24.0f, 8.0f, 3.0f}) == 3);

    // Past the end: clamp to the last segment rather than running off the array.
    CHECK(NavService::ResumeIndex(path, Vec3{100.0f, 0.0f, 0.0f}) == 3);

    // Degenerate inputs must not index out of range.
    CHECK(NavService::ResumeIndex({}, Vec3{0.0f, 0.0f, 0.0f}) == 0);
    CHECK(NavService::ResumeIndex({Vec3{1.0f, 2.0f, 3.0f}}, Vec3{0.0f, 0.0f, 0.0f}) == 0);

    // A path with a repeated point (a 4D rotation in place) has a zero-length segment; the
    // projection must not divide by it.
    const std::vector<Vec3> withRepeat{{0.0f, 0.0f, 0.0f}, {5.0f, 0.0f, 0.0f}, {5.0f, 0.0f, 0.0f},
                                       {10.0f, 0.0f, 0.0f}};
    const int32_t ri = NavService::ResumeIndex(withRepeat, Vec3{7.0f, 0.0f, 0.0f});
    CHECK(ri == 3);
    std::printf("  agent at 25/30 along the path -> resume at waypoint %d (naive would be 1)\n",
                NavService::ResumeIndex(path, Vec3{25.0f, 0.0f, 0.0f}));
}

void TestResumableSampling()
{
    Section("resumable chunk sampling");

    // Geometry complex enough to need real refinement, so the walk genuinely suspends many times.
    BoxWorld world;
    world.solids.push_back(AABB{{-40.0f, -40.0f, -40.0f}, {40.0f, 40.0f, 2.0f}});   // floor slab
    world.solids.push_back(AABB{{3.0f, -40.0f, -40.0f}, {5.0f, 40.0f, 40.0f}});     // wall
    world.solids.push_back(AABB{{-40.0f, 6.0f, -40.0f}, {40.0f, 9.0f, 40.0f}});     // second wall
    world.solids.push_back(AABB{{10.0f, 10.0f, 10.0f}, {11.0f, 11.0f, 11.0f}});     // fleck

    VolumeConfig cfg = MakeConfig(1.0f, 0.5f, 0.5f, 64.0f);
    cfg.maxProbesPerChunk = 100000; // let it refine fully, so there is real work to suspend
    Volume vol("resume", cfg);

    const IVec3 coord{0, 0, 0};

    SampleStats atomicStats;
    auto atomicChunk = SampleChunk(vol, coord, world, atomicStats);

    // Deliberately a tiny slice: it forces a suspend every few probes, which is the state the
    // resume logic is most likely to get wrong.
    ChunkSampler sampler(vol, coord);
    int32_t pumps = 0;
    while (!sampler.Step(world, 7))
    {
        ++pumps;
        CHECK(pumps < 100000); // must terminate
    }
    auto resumedChunk = sampler.Take();

    std::printf("  atomic: %d probes in 1 pass   resumable: %d probes over %d suspensions\n",
                atomicStats.probes, sampler.Probes(), pumps);

    // It has to have actually suspended, or this proves nothing.
    CHECK(pumps > 10);

    // The point of the whole exercise: splitting the work across frames must not change the map.
    CHECK(sampler.Probes() == atomicStats.probes);
    CHECK(atomicChunk->state == resumedChunk->state);

    int32_t mismatches = 0;
    for (int32_t i = 0; i < kChunkVoxels; ++i)
    {
        if (atomicChunk->ClearanceLocal(i) != resumedChunk->ClearanceLocal(i))
        {
            ++mismatches;
        }
    }
    CHECK(mismatches == 0);

    // Budget exhaustion must stay a whole-level decision after the split - breadth-first is what
    // keeps an over-budget chunk uniformly coarse instead of sealing one arbitrary octant.
    VolumeConfig poor = cfg;
    poor.maxProbesPerChunk = 300;
    Volume poorVol("poor", poor);

    SampleStats poorAtomic;
    auto poorChunkA = SampleChunk(poorVol, coord, world, poorAtomic);

    ChunkSampler poorSampler(poorVol, coord);
    while (!poorSampler.Step(world, 3))
    {
    }
    auto poorChunkB = poorSampler.Take();

    CHECK(poorSampler.BudgetExhausted() == poorAtomic.budgetExhausted);
    CHECK(poorChunkA->state == poorChunkB->state);

    int32_t poorMismatches = 0;
    for (int32_t i = 0; i < kChunkVoxels; ++i)
    {
        if (poorChunkA->ClearanceLocal(i) != poorChunkB->ClearanceLocal(i))
        {
            ++poorMismatches;
        }
    }
    CHECK(poorMismatches == 0);
    std::printf("  over-budget chunk: exhausted=%d, identical under both paths=%d\n",
                poorAtomic.budgetExhausted ? 1 : 0, poorMismatches == 0 ? 1 : 0);
}

void TestProbeBudgetOverBlocking()
{
    Section("probe budget over-blocks narrow space");

    // Narrow interiors are where the octree sampler's per-chunk budget silently lies. Fully
    // refining a 16^3 chunk costs ~4681 probes; the default allows 2048, and whatever is left
    // unresolved is marked SOLID. So open space reads as filled in.
    //
    // Pinned as a test because the failure is invisible from the outside - it looks like correct
    // pathfinding around an obstacle that is not there.
    BoxWorld world;
    const float t = 40.0f;
    world.solids.push_back(AABB{{-t, -t, -t}, {t, -2.0f, t}});
    world.solids.push_back(AABB{{-t, 2.0f, -t}, {t, t, t}});
    world.solids.push_back(AABB{{-t, -t, -t}, {t, t, -2.0f}});
    world.solids.push_back(AABB{{-t, -t, 2.0f}, {t, t, t}});

    PathQuery q;
    q.start = {-20.0f, 0.0f, 0.0f};
    q.goal = {20.0f, 0.0f, 0.0f};
    q.maxProbesPerQuery = 100000000;

    const auto run = [&](int32_t aBudget)
    {
        NavService svc;
        svc.SetGameThread();
        svc.SetProbe(&world);
        VolumeConfig cfg = MakeConfig(1.0f, 0.5f, 0.5f, 64.0f);
        cfg.maxProbesPerChunk = aBudget;
        CHECK(svc.CreateVolume("pb", cfg));

        PathResult r;
        return svc.FindPathSync("pb", q, r);
    };

    const Status cheap = run(2048);                                    // the OLD default
    const Status shipped = run(VolumeConfig{}.maxProbesPerChunk);      // the current one

    std::printf("  same 4 m corridor: budget 2048 -> %s,  default %d -> %s\n", ToString(cheap),
                VolumeConfig{}.maxProbesPerChunk, ToString(shipped));

    // Both halves matter. The first is the finding - a budget below full refinement turns open
    // space into a wall - and the second is that the shipped default is now above that line.
    CHECK(cheap != Status::Success);
    CHECK(shipped == Status::Success);
}

void TestEscapeQuery()
{
    Section("escape / unstick query");

    // A tight alcove: a pocket just big enough to sit in, opening into open air on +x.
    BoxWorld world;
    const float t = 60.0f;
    world.solids.push_back(AABB{{-t, -t, -t}, {t, -2.0f, t}});   // -y wall
    world.solids.push_back(AABB{{-t, 2.0f, -t}, {t, t, t}});     // +y wall
    world.solids.push_back(AABB{{-t, -t, -t}, {t, t, -2.0f}});   // floor
    world.solids.push_back(AABB{{-t, -t, 2.0f}, {t, t, t}});     // ceiling
    world.solids.push_back(AABB{{-t, -t, -t}, {-10.0f, t, t}});  // closed end at -x

    NavService svc;
    svc.SetGameThread();
    svc.SetProbe(&world);

    // ⚠️ A 4 m corridor at 1 m voxels forces the octree sampler to refine to leaf level almost
    // everywhere, which costs ~4681 probes for a 16^3 chunk - more than the 2048 default. On
    // exhaustion the sampler marks whatever it has not resolved as SOLID (deliberately
    // conservative), so the corridor reads as filled in and every query fails. Measured: the same
    // world returns Unreachable at the default and Success here, with nothing else changed.
    VolumeConfig cfg = MakeConfig(1.0f, 0.5f, 0.5f, 90.0f);
    cfg.maxProbesPerChunk = 100000;
    CHECK(svc.CreateVolume("esc", cfg));

    // Wedged deep in the pocket. Just needing to fit is satisfied where it already stands, so
    // this is the case minDistance exists for.
    EscapeQuery q;
    q.from = {-8.0f, 0.0f, 0.0f};
    q.searchRadius = 24.0f;
    q.maxProbesPerQuery = 100000000;

    EscapeResult r;
    const Status es = svc.FindEscape("esc", q, r);
    std::printf("  nearest: %s dist=%.2f clr=%.2f chunks=%d probes=%d\n", ToString(es),
                static_cast<double>(r.distance), static_cast<double>(r.clearance), r.chunksSampled, r.probes);
    CHECK(es == Status::Success);
    CHECK(r.distance < 1.5f); // correct, and useless on its own

    // Demanding real distance turns it into "go somewhere else", which is what breaks a local
    // minimum. It has to head down the corridor, the only direction with anywhere to go.
    EscapeQuery far = q;
    far.minDistance = 12.0f;

    EscapeResult rf;
    CHECK(svc.FindEscape("esc", far, rf) == Status::Success);
    CHECK(rf.distance >= 12.0f);
    CHECK(rf.position.x > q.from.x); // away from the closed end
    std::printf("  wedged: nearest=%.2f m   with minDistance=12: %.2f m at x=%.1f\n",
                static_cast<double>(r.distance), static_cast<double>(rf.distance),
                static_cast<double>(rf.position.x));

    // The answer must actually be free space, not merely a voxel index that passed a bound.
    for (const AABB& s : world.solids)
    {
        CHECK(!s.Contains(rf.position));
    }

    // Asking for more room than the corridor has must fail cleanly rather than return a wrong
    // answer - a stuck agent needs to know recovery is impossible, not be sent into a wall.
    EscapeQuery roomy = q;
    roomy.minClearance = 20.0f;

    EscapeResult rr;
    const Status rs = svc.FindEscape("esc", roomy, rr);
    CHECK(rs == Status::Unreachable);
    std::printf("  demanding 20 m of room in a 4 m corridor -> %s\n", ToString(rs));

    // An impossible query is rejected up front rather than scanned for.
    EscapeQuery bad = q;
    bad.minDistance = bad.searchRadius + 1.0f;
    EscapeResult rb;
    CHECK(svc.FindEscape("esc", bad, rb) == Status::BadInput);
}

void TestBidirectional()
{
    Section("bidirectional: validity, parity with unidirectional, fast unreachable");

    // A wall with a single window, deliberately placed OFF the straight start->goal line so the
    // route has to bend through it. A centred window makes this test worthless: the answer is the
    // straight line and both searches "pass" without the frontiers ever meeting in a constriction.
    // Window is y in [8,20], z in [6,12]; start and goal sit at y=0.
    const std::vector<AABB> solids = {
        AABB{Vec3{-2.0f, -30.0f, -30.0f}, Vec3{2.0f, 30.0f, 6.0f}},  // below the window
        AABB{Vec3{-2.0f, -30.0f, 12.0f}, Vec3{2.0f, 30.0f, 30.0f}},  // above it
        AABB{Vec3{-2.0f, -30.0f, 6.0f}, Vec3{2.0f, 8.0f, 12.0f}},    // beside it, -y side
        AABB{Vec3{-2.0f, 20.0f, 6.0f}, Vec3{2.0f, 30.0f, 12.0f}},    // beside it, +y side
    };

    const float radius = 1.0f;
    const Vec3 start{-25.0f, 0.0f, 9.0f};
    const Vec3 goal{25.0f, 0.0f, 9.0f};

    PathResult uni;
    PathResult bi;

    for (int pass = 0; pass < 2; ++pass)
    {
        const bool bidir = (pass == 1);

        BoxWorld world;
        world.solids = solids;

        NavService svc;
        svc.SetGameThread();
        svc.SetProbe(&world);
        CHECK(svc.CreateVolume("bidir", MakeConfig(1.0f, radius, 0.5f, 200.0f)));

        PathQuery q;
        q.start = start;
        q.goal = goal;
        q.bidirectional = bidir;

        PathResult& out = bidir ? bi : uni;
        const Status s = svc.FindPathSync("bidir", q, out);

        CHECK(s == Status::Success);
        CHECK(out.points.size() >= 2);

        // The path must actually clear the geometry, not merely be reported as valid.
        const float clear = MinClearanceAlongPath(out.points, solids, 0.25f);
        CHECK(clear >= radius * 0.9f);
    }

    // The route must actually have been forced off the straight line, or this proves nothing.
    const float straight = (goal - start).Length();
    CHECK(uni.length > straight * 1.05f);
    CHECK(bi.length > straight * 1.05f);

    // Bidirectional may pick a different but equivalent route; it must not be dramatically worse.
    CHECK(bi.length <= uni.length * 1.35f);
    std::printf("  straight=%.2f   uni len=%.2f exp=%d   bi len=%.2f exp=%d\n", static_cast<double>(straight),
                static_cast<double>(uni.length), uni.expansions, static_cast<double>(bi.length), bi.expansions);

    // A goal sealed inside a closed box is the case bidirectional is meant to answer cheaply: the
    // backward frontier drains almost immediately instead of the forward search having to explore
    // everything reachable before it can conclude anything.
    {
        BoxWorld world;
        world.solids = {
            AABB{Vec3{18.0f, -8.0f, -8.0f}, Vec3{22.0f, 8.0f, 8.0f}},
            AABB{Vec3{34.0f, -8.0f, -8.0f}, Vec3{38.0f, 8.0f, 8.0f}},
            AABB{Vec3{18.0f, -8.0f, -8.0f}, Vec3{38.0f, -4.0f, 8.0f}},
            AABB{Vec3{18.0f, 4.0f, -8.0f}, Vec3{38.0f, 8.0f, 8.0f}},
            AABB{Vec3{18.0f, -8.0f, -8.0f}, Vec3{38.0f, 8.0f, -4.0f}},
            AABB{Vec3{18.0f, -8.0f, 4.0f}, Vec3{38.0f, 8.0f, 8.0f}},
        };

        PathResult sealedUni;
        PathResult sealedBi;

        for (int pass = 0; pass < 2; ++pass)
        {
            const bool bidir = (pass == 1);

            NavService svc;
            svc.SetGameThread();
            svc.SetProbe(&world);
            CHECK(svc.CreateVolume("sealed2", MakeConfig(1.0f, 0.4f, 0.5f, 120.0f)));

            PathQuery q;
            q.start = Vec3{0.0f, 0.0f, 0.0f};
            q.goal = Vec3{28.0f, 0.0f, 0.0f}; // inside the sealed box
            q.bidirectional = bidir;
            q.snapRadius = 0.0f;              // do not let snapping walk the goal back outside
            q.allowPartial = false;

            PathResult& out = bidir ? sealedBi : sealedUni;
            const Status s = svc.FindPathSync("sealed2", q, out);
            out.status = s;
            CHECK(!IsUsable(s));
            std::printf("  sealed goal, %s -> %s after %d expansions\n", bidir ? "bidir" : "uni ", ToString(s),
                        out.expansions);
        }

        // The point of the exercise. Unidirectional cannot distinguish "no route" from "gave up":
        // it has to exhaust everything reachable from the start, so it runs out of budget first.
        // Bidirectional drains the small sealed pocket around the goal and returns a definitive
        // Unreachable, for strictly less work.
        CHECK(sealedBi.status == Status::Unreachable);
        CHECK(sealedBi.expansions < sealedUni.expansions);
    }

    // Where does bidirectional actually pay off? Two frontiers cost roughly double on a short
    // hop, so the win has to come from long routes, where a single frontier grows as a large
    // sphere while two shallow ones do not. A slalom of offset windows keeps the search honest -
    // in open space a weighted heuristic converges almost straight and measures nothing.
    {
        BoxWorld world;
        for (int i = 0; i < 6; ++i)
        {
            const float x = -250.0f + static_cast<float>(i) * 100.0f;
            const float lo = (i % 2 == 0) ? 10.0f : -60.0f;  // window slides side to side
            const float hi = lo + 50.0f;
            world.solids.push_back(AABB{Vec3{x - 2.0f, -300.0f, -60.0f}, Vec3{x + 2.0f, lo, 60.0f}});
            world.solids.push_back(AABB{Vec3{x - 2.0f, hi, -60.0f}, Vec3{x + 2.0f, 300.0f, 60.0f}});
        }

        int uniExp = 0;
        int biExp = 0;

        for (int pass = 0; pass < 2; ++pass)
        {
            const bool bidir = (pass == 1);

            NavService svc;
            svc.SetGameThread();
            svc.SetProbe(&world);
            CHECK(svc.CreateVolume("slalom", MakeConfig(1.0f, 1.0f, 0.5f, 600.0f)));

            PathQuery q;
            q.start = Vec3{-320.0f, 0.0f, 0.0f};
            q.goal = Vec3{320.0f, 0.0f, 0.0f};
            q.bidirectional = bidir;
            q.maxExpansions = 2000000;
            q.maxProbesPerQuery = 100000000;

            PathResult r;
            const Status s = svc.FindPathSync("slalom", q, r);
            CHECK(IsUsable(s));

            (bidir ? biExp : uniExp) = r.expansions;
            std::printf("  slalom 640m, %s -> %s len=%.1f exp=%d\n", bidir ? "bidir" : "uni  ", ToString(s),
                        static_cast<double>(r.length), r.expansions);
        }

        // Recorded rather than asserted as a hard ratio: the crossover is what decides whether
        // bidirectional should be the default, and pinning an exact factor here would just make
        // the suite brittle against future cost-model tweaks.
        std::printf("  long-route expansion ratio bi/uni = %.2f\n",
                    uniExp > 0 ? static_cast<double>(biExp) / static_cast<double>(uniExp) : 0.0);
    }
}
} // namespace

int main()
{
    // Unbuffered: if a test faults, the output up to that point must survive.
    std::setvbuf(stdout, nullptr, _IONBF, 0);
    std::printf("vnav core tests\n===============\n");

    TestCoordinateMath();
    TestEmptyWorld();
    TestWallWithHole();
    TestSealedGoal();
    TestPartialPath();
    TestSnapping();
    TestAsyncRoundTrip();
    TestCancel();
    TestInvalidation();
    TestBadInput();
    TestNoProbe();
    TestConcurrentSubmitters();
    TestSyncOffGameThread();
    TestShutdownWithLiveJobs();
    TestPerWaypointClearance();
    TestTurnClearance();
    TestYawSearch();
    TestFilletSmoothing();
    TestDynamicObstacles();
    TestHierarchicalGuidance();
    TestBakeGameBuildStamp();
    TestResumeIndex();
    TestResumableSampling();
    TestProbeBudgetOverBlocking();
    TestEscapeQuery();
    TestBidirectional();
    TestBakeCodec();
    TestBakedNav();
    TestBakedRoutingWithoutProbe();

    std::printf("\n===============\n%d checks, %d failures\n", g_checks, g_failures);
    return g_failures == 0 ? 0 : 1;
}
