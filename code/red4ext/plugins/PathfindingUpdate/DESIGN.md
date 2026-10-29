# vnav — Volumetric 3D Pathfinding for Cyberpunk 2077

A reusable C++ library, shipped as a RED4ext plugin, that plans paths through **open 3D space**
(air, shafts, atriums, interiors) for **flying agents**. Sync and async APIs, thread-safe by
construction.

---

## 1. Why this has to exist (research findings)

Everything the game ships is **ground navmesh only**. Verified against the decompiled scripts:

| Evidence | Location | Meaning |
|---|---|---|
| `enum NavGenAgentSize { Human = 0 }` — a **single** value | `orphans.swift:5515` | There is one agent class in the whole nav system, and it walks. |
| Every path call is named `...OnlyHumanNavmesh` | `core/systems/navigationSystem.swift` | `CalculatePathOnlyHumanNavmesh`, `FindPointInSphereOnlyHumanNavmesh`, … |
| `AINavigationSystem` paths are `...ForCharacter(… owner: ref<Entity>)` | `cyberpunk/ai/aiNavigationSystem.swift` | Same navmesh, just agent-parameterised. |
| `GetNearestNavmeshPointBelow` steps `origin.Z -= radius` in a loop | `navigationSystem.swift:24` | The API's own idea of "find me a spot" is *drop to the floor*. |
| No native class matching `*Volume*`/`*Airspace*`/`*Flight*`/`*Octree*` | whole tree | No volumetric representation exists to wrap. |

So there is no aerial graph to query and nothing to extend. **The free-space representation has
to be built at runtime**, and the only source of ground truth for "is this cubic metre empty" is
the physics scene.

### The one primitive we build on

```swift
SpatialQueriesSystem.OverlapByQueryFilter(
    primitiveDimension: Vector4,   // half-extents
    position: Vector4,
    rotation: Quaternion,
    queryFilter: QueryFilter,
    out result: TraceResult) -> Bool          // true == something is in the box
```

Collision groups confirmed in use by vanilla AI code (`aiLineTrace.swift`,
`aiFindTeleportPositionForTakedown.swift`): `Static`, `Terrain`, `Collider`, `Destructible`,
`Dynamic`, `Vehicle`, `PlayerBlocker`, `AI`, `Interaction`. Our default world filter is
`Static | Terrain | Collider | Destructible`.

**This call is synchronous and must be made from the game thread.** REDengine drives PhysX on its
own schedule; issuing scene queries from an arbitrary worker races the simulation step. That single
constraint dictates the entire architecture below.

---

## 2. Architecture

Three layers, strictly one-directional:

```
        ┌──────────────────────────────────────────────────────────┐
        │  Consumers: redscript · CET · other RED4ext plugins      │
        └──────────────┬──────────────────────┬────────────────────┘
                       │ RTTI globals         │ C ABI (VNav_GetApi)
        ┌──────────────▼──────────────────────▼────────────────────┐
  L1    │  vnav::game   RedWorldProbe · ScriptBindings · Plugin    │  <- knows RED4ext
        └──────────────┬───────────────────────────────────────────┘
                       │ IWorldProbe  (the ONLY seam to the world)
        ┌──────────────▼───────────────────────────────────────────┐
  L0    │  vnav::core   Volume · ChunkStore · Sampler · Planner    │  <- knows nothing
        │               NavService (jobs, workers, budgets)         │
        └──────────────────────────────────────────────────────────┘
```

`vnav_core` is a static library with **zero** RED4ext/game includes. It is compiled and unit-tested
standalone against a synthetic probe (`tests/`), which matters a great deal here: this workspace has
no way to typecheck redscript without launching the game, so the half of the system that *can* be
verified offline is deliberately made as large as possible.

---

## 3. Representation

### Volumes

The unit of configuration is a **NavVolume**: a named, independently-configured region of space.

```
CreateVolume("drone", voxelSize=0.5, agentRadius=0.4, margin=0.6, bounds=…)
CreateVolume("av",    voxelSize=2.5, agentRadius=4.0, margin=3.0, bounds=…)
```

Volumes exist because agent scale in this game spans an order of magnitude (a `ShooterFlyingDrone`
vs. an AV), and one voxel size cannot serve both. Each volume owns its own chunk store, so a drone
never pays for an AV's coarse map and vice-versa. This is also what makes "pathfinding in volumes"
literal: a volume's `bounds` is a hard box outside which that volume will not plan.

### Voxels and chunks

- Voxel grid is axis-aligned, Z-up (Cyberpunk convention), origin at world origin.
- `CHUNK_DIM = 16` → 4096 voxels per chunk, indexed `(z*16 + y)*16 + x`.
- A chunk stores **two 512-byte bitsets**:
  - `blocked` — occupancy probed with the box inflated by `agentRadius`.
  - `tight` — occupancy probed with the box inflated by `agentRadius + margin`.
- Chunks are one of three states: `Empty`, `Solid`, `Mixed`. `Empty` and `Solid` are **process-wide
  singletons** with no payload, so open sky costs a map entry and nothing else.

**Agent radius is baked into the data by inflating the probe box.** This is the configuration-space
trick: after inflation, "can the agent occupy this voxel" is a single bit test, and the planner never
does neighbourhood clearance queries. The `tight` bit is what is left over — free, but with less than
`margin` of slack — and the planner charges a cost multiplier for it so paths prefer open air instead
of scraping along walls.

### Why not a distance field

A clearance/distance field would decouple agent radius from the data, but computing it for a chunk
requires all 26 neighbours to be sampled first — a 27× amplification of the expensive operation
(physics probes). Per-volume inflation gets the same behaviour for one extra probe per solid leaf.

---

## 3b. The baked static world

### The problem probing cannot solve

The original design sampled the world exclusively by probing the physics scene. That has a defect
that is not a performance issue but a correctness one:

**Static collision lives in `worldCollisionNode`, which is a node inside a streaming sector.**
Physics actors exist only for sectors currently streamed in. A probe 400 m away therefore does not
return "solid" for a building — it returns *empty*, because nothing is there yet. Every path
planned beyond the streaming radius is confidently wrong.

No amount of probing fixes this. The information is not in the running process.

### Extracting it offline

It is, however, in the archives. The pipeline is two offline stages:

```
game archives                                        (61 GB)
  │  WolvenKit `navbake`   ── walks .streamingsector, parses worldCollisionNode
  │                           compiled buffers, resolves mesh hashes via the
  │                           geometry cache, composes actor × shape transforms
  ▼
.vnavgeo    world-space collision shapes + de-duplicated mesh library
  │  vnav_bake             ── tile-parallel rasterisation into voxel occupancy
  ▼
.vnavbake   sparse per-chunk occupancy bitsets
```

Actor positions turned out to be **absolute world coordinates** in 15.17 fixed point (±16 km,
comfortably the whole map), so no sector offset is involved.

### Occupancy on disk, clearance at load

The planner wants clearance — distance to nearest solid — because that is what decouples the data
from the agent: `fits(v, r)` becomes `clearance[v] >= threshold(r)`, so one bake serves every
agent size. The obvious move is to store the clearance field.

That was built, measured, and rejected. **A clearance field is a gradient, and in a city almost
every voxel is on one**, so it barely compresses — 1819 bytes/chunk, multiple GB for the map at
1 m voxels. Lowering the saturation cap helps only because it flattens the gradient back out,
and it does so by throwing away the range that made the data agent-agnostic in the first place.

So the bake stores **occupancy, one bit per voxel**, and the distance transform (exact,
separable, Felzenszwalb) runs when a chunk is loaded. Measured on the same sample: **227
bytes/chunk**, and the bake got 5× faster because it no longer transforms anything.

The load-time transform is also strictly *more* accurate. A stored field must saturate at some
fixed cap; the load-time one reaches exactly as far as the asking volume's own tight threshold, so
there is no saturation to lose information to and no global ceiling on agent size.

The cost is that a chunk load must gather its 3×3×3 neighbourhood. Loads are spatially local, so
a decoded-occupancy cache turns that into roughly one read in practice.

### Two invariants worth stating

- **Absent means open sky.** Most of a city's volume is air, and not storing it is what keeps the
  index proportional to surfaces rather than to volume. Solid chunks *are* stored, because
  otherwise bedrock would read as flyable.
- **Coverage is the region the baker processed, not the chunks it stored.** Deriving it from
  stored chunks would declare the air above a city un-baked — precisely where the bake is most
  valuable. The claimed region goes in the header.

### Layering with the probe

The bake describes the world as shipped. Doors, vehicles and destruction are not in it, so the
probe stays on underneath. `InvalidateBox` marks the affected chunk coordinates **probe-only**:
once the world has changed somewhere, the bake stops answering for that place, or it would keep
reinstating the door it just opened.

## 4. Sampling: adaptive octree probing

Voxelising a 16³ chunk with one probe per voxel would be 4096 physics queries. Instead the sampler
walks an octree over the chunk and exploits the one thing a boolean overlap can prove: **emptiness
is hereditary.**

```
probe(node, inflation = agentRadius + margin):
    if !OverlapBox(node.center, node.halfExtents + inflation):
        mark all voxels in node free in BOTH bitsets     # <- the big win
        return
    if node.size == 1 or depth == maxProbeDepth or probeBudget exhausted:
        mark solid in `tight`
        one extra probe at `agentRadius` inflation decides `blocked`
        return
    recurse into 8 children
```

Cost is proportional to the **surface area** of geometry in the chunk, not its volume. Open sky is
one probe per chunk. A chunk with a wall through it is a few hundred.

Two hard bounds keep this deterministic, because a boolean overlap can never prove *solidity* and so
solid regions would otherwise subdivide all the way down for no information:

- `maxProbeDepth` — floor on leaf size (default 4 = full 1-voxel resolution).
- `maxProbesPerChunk` — hard cap (default 512). On exhaustion, **unresolved space is marked solid.**

Both failure directions are conservative: we over-block, never under-block. An agent may fail to find
a tight path; it will never be routed through a wall.

---

## 5. Search: Lazy Theta\*

Grid A\* produces staircase paths, which look absurd on something that flies. **Lazy Theta\*** is the
standard choice for UAV planning and is what we use:

- 26-connected voxel neighbourhood, Euclidean heuristic.
- On expansion, optimistically set `parent(neighbour) = parent(current)`; validate line-of-sight only
  when the node is **popped**, falling back to the grid parent if the LOS fails. One LOS check per
  expanded node instead of per generated node.
- LOS = 3D supercover DDA (Amanatides–Woo) over the occupancy bits.
- Edge cost = Euclidean × `tightnessPenalty` (if either endpoint is `tight`) × `verticalCostMultiplier`
  (applied to the vertical component — flying agents generally prefer level flight to bobbing).

Post-processing, in order: **string-pull** (drop waypoints joined by clear LOS) → optional
**Catmull-Rom resample** for a flyable curve.

Practical behaviour the API has to get right:

- **Start/goal snapping.** Both are routinely inside geometry (a drone hugging a wall, a goal on the
  floor). Both are snapped to the nearest free voxel within `snapRadius`, and the snapped positions
  are returned so the caller can see what happened.
- **Partial paths.** If the goal is unreachable or the budget runs out, return the best path found
  toward the closest-approach node (`allowPartial`, default on) with status `PartialSuccess`. A
  flying agent that gets 80% of the way there is far more useful than a hard failure.

---

## 6. Concurrency

The premise: **`IWorldProbe` may only be called from the game thread; everything else may run
anywhere.** The design separates *acquiring* world data from *searching* it.

### Threads

| Thread | May call | Drives |
|---|---|---|
| Game/main | `IWorldProbe`, script callbacks | `Pump(budgetMicros)` |
| Workers (`clamp(hw/4, 1, 4)`) | core only, never the game | Planning slices |
| Any caller | submit / poll / cancel | — |

### The shared-state contract

| State | Shape | Rule |
|---|---|---|
| Chunk payload | `shared_ptr<const Chunk>` | Built privately, published once, **never mutated after publish**. |
| Chunk store | 64 shards, each `shared_mutex` + map | Reads take a shared lock on one shard; publishes take a unique lock on one shard. |
| Job record | `shared_ptr<PathJob>` in a sharded registry | Same discipline. |
| Job status | `std::atomic<Status>` | Result is fully written, **then** status stores with `release`. Readers acquire. |
| Sample queue | mutex + vector + dedup set | Workers produce, game thread consumes. |
| Completion callbacks | queue drained in `Pump` | **Script callbacks never fire on a worker thread.** |

Two rules are load-bearing and are asserted in code:

1. **A job pins every chunk it reads** — it holds `shared_ptr` copies for its whole lifetime. Its view
   of the world is therefore immutable and self-consistent, and nothing it is walking can be evicted
   or freed underneath it. This is the C++ expression of the same lesson the redscript side of this
   workspace paid for: *do not read a shared mutable container; hold an immutable snapshot behind one
   handle.*
2. **The core never calls the game.** The only entry point that touches `IWorldProbe` is `Pump`.

### Suspend / resume

A worker cannot sample. When the search reaches a voxel whose chunk is not resident, the job
**parks**:

```
worker:  plan slice → hits missing chunk → record need + 3×3×3 halo → status = WaitingForWorld
main:    Pump() → drain sample queue within budget → publish chunks
                → any parked job whose needs are resident → re-queue to workers
                → drain completion callbacks
worker:  resume — open list and node map were never torn down
```

Because the planner state lives in the job, resuming is free. To keep parks rare, a job **prefetches
the corridor** (chunks along the straight start→goal segment, radius `prefetchRadiusChunks`) before
its first slice.

### Sync API

- **Called on the game thread** — runs the same loop but services its own samples inline, bounded by
  `maxProbesPerQuery` / `maxExpansions`. No parking, no deadlock.
- **Called off the game thread** — submits and blocks on a condition variable with a mandatory
  timeout, relying on the game thread to keep pumping. Documented as the risky path.

### Pumping

`Pump()` is driven from redscript by a `DelayCallbackNextFrame` self-rescheduling loop
(`DelaySystem.DelayCallbackNextFrame` — verified present, `orphans.swift:11826`). This is a
guaranteed main-thread per-frame callback with no address hooks. `Pump` is also public API, so any
host can drive it directly. It is safe to call with a null probe (loading screens): jobs simply stay
parked.

---

## 7. Invalidation

World geometry streams, doors open, vehicles move. V1 handles this explicitly rather than pretending
to be clever:

- `InvalidateBox(volume, aabb)` — drop chunks overlapping a box.
- `InvalidateAll(volume)` — drop everything (wired to world detach / game load).
- Optional per-chunk TTL (`chunkTtlSeconds`, 0 = never).
- LRU eviction against `maxResidentChunks`, run on the game thread during `Pump`.

In-flight jobs are unaffected: they pinned their chunks.

---

## 8. Public API

Three surfaces over one implementation.

**C++** — `include/vnav/*.hpp`, for anything linking the core directly.

**C ABI** — `VNav_GetApi(version)` exported from the DLL, returning a function-pointer table. This is
the real cross-plugin story: another RED4ext plugin can `GetProcAddress` it without matching our
MSVC version or STL ABI.

**Redscript** — RTTI globals (`VNav_CreateVolume`, `VNav_FindPathAsync`, `VNav_PollPath`,
`VNav_FindPathSync`, `VNav_Cancel`, `VNav_InvalidateBox`, `VNav_Pump`, `VNav_GetStats`) plus a
native result class `VNavPathResult`. Configuration is passed as scalar parameters rather than
registered structs — fewer RTTI moving parts, and tolerant of the library gaining options later.

---

## 9. Deliberate V1 limits

Named so they are choices, not surprises:

- **Static world assumption between invalidations.** No automatic rebuild when a door opens.
- **No hierarchical/coarse graph.** Very long paths are bounded by `maxExpansions` and will return
  partials. A coarse level over `Empty`/`Solid` chunk flags is the natural next step.
- **No dynamic obstacle avoidance.** The library plans; it does not steer. Local avoidance of moving
  entities belongs to the consumer.
- **No path following / movement.** Out of scope by design — this is a planner.
- **Probes go through RTTI `ExecuteFunction`.** Class/function/instance pointers are resolved once and
  cached, but the per-call stack setup remains. If profiling says it matters, `IWorldProbe` is the
  seam where a direct native call gets swapped in without touching the core.
