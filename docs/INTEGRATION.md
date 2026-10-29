# vnav integration guide

How to build on PathfindingUpdate (vnav) from your own mod. This is the reference for **what is
available and how to call it**; `README.md` covers what the system is and why, `DESIGN.md` covers
architecture, and the shipped `r6/scripts/PathfindingUpdate/VolumetricNav.reds` carries the
per-function prose next to each declaration.

---

## 1. Pick your surface

vnav ships three call surfaces over one implementation. They are **not** equally complete:

| You are writing | Use | Completeness |
|---|---|---|
| redscript (`r6/scripts/…`) | **Native globals** (`VNav…`) + helper classes | **Full** — every feature |
| CET Lua (`init.lua`) | Same globals, callable as bare Lua functions | **Full** (with the Lua gotchas in §5) |
| Another RED4ext plugin (C/C++) | **C ABI** `VNav_GetApi(1)` | Core planning only — see the gap list in §6 |
| Another RED4ext plugin (C++, ABI-tolerant) | link `vnav_core` | **Full** |

**Dependency model:** the `VNav*` natives are declared by *this* mod's `VolumetricNav.reds` and
implemented by `PathfindingUpdate.dll`. Your mod depends on PathfindingUpdate being installed;
declare a dependency and call the globals by name. **Do not re-declare the natives in your own
`.reds`** — duplicate native declarations are a compile error.

---

## 2. Install and prerequisites

- **Game**: built and verified on 2.5. The DLL is compiled against the engine ABI via
  RED4ext.SDK, so a future game patch needs a rebuild (see README "Building").
- **RED4ext** (any current release).
- **The bake**: `red4ext/plugins/PathfindingUpdate/nightcity.vnavbake` (173 MB, stamped game
  build `2.5`, 1 m voxels). It ships with the mod. Without it the library still answers — but by
  probing, which cannot see geometry that has not streamed in, so anything beyond streaming
  radius is confidently *wrong*. Long-range planning requires the bake.
- **A redscript consumer compiles against this mod's script files** — if PathfindingUpdate is
  missing, your `VNav…` calls fail to compile (loudly, which is the point).

---

## 3. Concepts in sixty seconds

- **Volume** — a named, independently configured planning region. One volume per *agent profile*
  (a drone and an AV want two volumes: agent radius is baked into the sampled data).
  Volumes own their own chunk stores; nothing is shared between them.
- **Bake vs probe** — the bake is the authoritative static world; physics probes fill in what the
  bake cannot know (doors, destruction, moved vehicles). Probes run on the game thread only.
- **Clearance** — distance to the nearest solid; "does a radius-r agent fit here" is one compare.
  Read back per waypoint via `VNavPathClearance` (it saturates — see §8).
- **Job handle** — `VNavFindPathAsync`/`Sync` return a `Uint32` handle (0 = failure). Reads and
  `VNavPathRelease` go through it. Handles are plain integers on every surface.
- **Status** — poll `VNavPathStatus`; anything `>= Success` is terminal. `PartialSuccess` is a
  *usable* result on purpose (80% of the way beats refusing to move).

---

## 4. Lifecycle walkthrough

```swift
// 1. Create the volume (one per agent profile). bounds bound the search space.
VNavCreateVolume(n"drone", 1.0, 0.5, 1.0, boundsMin, boundsMax, VNav.WorldMask());

// 2. Attach the baked world. The bake's voxel size must match the volume's voxelSize.
//    Prefer the checked variant: it refuses a bake stamped for another game build.
VNavAttachBakeChecked(n"drone", "red4ext/plugins/PathfindingUpdate/nightcity.vnavbake", "2.5");

// 3. Tune once, per volume (read-modify-write; call in any order, nothing resets).
VNavSetVolumeTuning(n"drone", 1.0, 1.35, 1.2, 4.0, 60000, 1);
VNavSetAgentLength(n"drone", 2.4, 5);        // optional: elongated bodies
VNavSetCoarseGuidance(n"drone", true, 4.0);  // default on

// 4. Plan. Async is the intended path.
let handle: Uint32 = VNavFindPathAsync(n"drone", startPos, goalPos);

// 5. Poll from your update/DelaySystem callback until terminal, then read.
let status: Int32 = VNavPathStatus(handle);
if VNav.IsTerminal(status) {
    if VNav.IsUsable(status) {
        let points: array<Vector4> = VNav.GetPoints(handle);
        // fly along points...
    }
    VNavPathRelease(handle);   // 6. always release
}
```

Pumping (driving the sampler) is **not your job**: the plugin pumps itself every frame from
RED4ext's running-state update, and a redscript fallback pump stands itself down once the native
one is observed. `VNavPump` exists for hosts that need to drive it manually (e.g. freezing the
sampler during a cutscene).

---

## 5. Redscript / Lua reference — every function

Signatures below are redscript. From CET Lua the same names are global functions; pass the same
arguments (see the Lua notes at the end of this section).

### Volumes and the bake

| Function | Returns | Notes |
|---|---|---|
| `VNavCreateVolume(name: CName, voxelSize: Float, agentRadius: Float, margin: Float, boundsMin: Vector4, boundsMax: Vector4, collisionMask: Uint32) -> Bool` | ok | `collisionMask` 0 = default world mask. Defaults: voxel 1.0, radius 0.5, margin 1.0. |
| `VNavDestroyVolume(name: CName) -> Bool` | ok | Drops the volume and its chunks. |
| `VNavHasVolume(name: CName) -> Bool` | exists | |
| `VNavAttachBake(volume: CName, path: String) -> Bool` | ok | Path relative to game root. Bake voxel size must equal the volume's `voxelSize`. |
| `VNavAttachBakeChecked(volume: CName, path: String, gameBuild: String) -> Bool` | ok | **Prefer this.** Refuses a bake stamped for a different game build. `gameBuild` `""` = skip check; an *unstamped* bake attaches with a warning rather than being refused. |
| `VNavHasBake(volume: CName) -> Bool` | attached | |

### Search tuning (stored per volume, applied to every query on it)

All setters are read-modify-write: they change only what they name.

| Function | Fields / defaults |
|---|---|
| `VNavSetVolumeTuning(name, heuristicWeight: Float, tightnessPenalty: Float, verticalCostMultiplier: Float, snapRadius: Float, maxExpansions: Int32, smoothing: Int32) -> Bool` | defaults `1.0, 1.35, 1.2, 4.0, 60000, 1`. `smoothing`: 0 raw voxels, 1 string-pull (default), 2 spline, 3 fillet. |
| `VNavSetSearchPersistence(name, maxParks: Int32, noProgressParks: Int32, timeoutSeconds: Float) -> Bool` | defaults `0 (unlimited), 512, 30`. Async queries only; sync always fails fast. |
| `VNavSetAgentLength(name, agentLength: Float, turnSamples: Int32) -> Bool` | defaults `0 (sphere), 5`. Turn clearance for elongated bodies. `agentRadius` stays the *cross-section* half-extent — see §8. |
| `VNavSetYawSearch(name, yawBuckets: Int32, yawCost: Float, yawFollowsTravel: Bool, yawTravelCone: Float) -> Bool` | defaults `0 (3D), 0.5, true, 0.785398`. `yawBuckets >= 4` enables 4D planning (needs `agentLength > 0`). Expensive; for tight interiors. |
| `VNavSetCoarseGuidance(name, enabled: Bool, penalty: Float) -> Bool` | defaults `true, 4.0`. Chunk-level guidance; needs a bake; helps only where barriers are chunk-scale. |
| `VNavSetBidirectional(name, enabled: Bool) -> Bool` | default `true`. Off is ~2x cheaper on short hops; on is the difference between working and not on long routes. |

### Planning and reading results

| Function | Returns | Notes |
|---|---|---|
| `VNavFindPathAsync(volume: CName, start: Vector4, goal: Vector4) -> Uint32` | handle or 0 | Plans on workers; samples a frame at a time. **The intended path.** |
| `VNavFindPathSync(volume: CName, start: Vector4, goal: Vector4) -> Uint32` | handle or 0 | Blocks the caller. Short hops through already-mapped space only. |
| `VNavPathStatus(handle: Uint32) -> Int32` | `VNavStatus` | Poll until `VNav.IsTerminal(status)`. |
| `VNavPathPointCount(handle: Uint32) -> Int32` | n | |
| `VNavPathPoint(handle: Uint32, index: Int32) -> Vector4` | waypoint | |
| `VNavPathLength(handle: Uint32) -> Float` | metres | |
| `VNavPathClearance(handle: Uint32, index: Int32) -> Float` | metres | Free radius at the waypoint. **Saturates** (§8). 0 if index out of range. |
| `VNavPathYaw(handle: Uint32, index: Int32) -> Float` | radians | 4D plans only; 0 for 3D. |
| `VNavPathInfeasibleTurns(handle: Uint32) -> Int32` | count | Corners the configured body cannot rotate through (needs `agentLength`). |
| `VNavPathResumeIndex(handle: Uint32, agentPosition: Vector4) -> Int32` | waypoint index | **Use on every replan** — see §8. |
| `VNavCancelPath(handle: Uint32) -> Bool` | ok | Async handles only. |
| `VNavPathRelease(handle: Uint32) -> Void` | | Always call when done. |

### Escape / unstick (when there is no path at all)

```swift
let status = VNavFindEscape(n"drone", pos, 30.0, 0.0, 8.0);
if status == EnumInt(VNavStatus.Success) {
    let target = VNavEscapePosition();   // read immediately after the call
    let dist   = VNavEscapeDistance();
}
```

| Function | Notes |
|---|---|
| `VNavFindEscape(volume: CName, from: Vector4, searchRadius: Float, minClearance: Float, minDistance: Float) -> Int32` | Game thread, synchronous, bounded. **`minDistance` is load-bearing**: without it the best answer is usually "where you already are". |
| `VNavEscapePosition() -> Vector4` / `VNavEscapeDistance() -> Float` | Valid only immediately after `VNavFindEscape`. |

### Dynamic obstacles (things that move)

Use this layer — **not** `VNavInvalidateBox` — for moving things (invalidate re-samples at chunk
granularity and poisons the cached map; see `VolumetricNav.reds` for the full rationale).

| Function | Notes |
|---|---|
| `VNavDynamicBegin() -> Void` | Start a batch. |
| `VNavDynamicAdd(center: Vector4, halfExtents: Vector4) -> Void` | One per obstacle; inflated by the volume's `agentRadius` automatically. |
| `VNavDynamicCommit(volume: CName) -> Int32` | Publishes the batch as one snapshot; returns the count in effect; committing empty clears. A search pins the snapshot it started with. |

Ready-made producer (ships with the mod, `DynamicObstacles.reds`):

```swift
VNavDynamic.Refresh(n"av", player, 60.0);              // Tier 1: prevent — search routes around
if VNavDynamic.PathHitsTraffic(gi, points, 3.0, hit) { // Tier 2: react — reject and replan
    // ...
}
```

### World maintenance, pump, stats

| Function | Notes |
|---|---|
| `VNavInvalidateBox(volume: CName, center: Vector4, halfExtents: Vector4) -> Void` | Geometry that changed and *stays* changed (door, destruction). |
| `VNavInvalidateAll(volume: CName) -> Void` | |
| `VNavPump(budgetMicros: Int32) -> Void` | Manual pump; normally unnecessary. `-1` = default budget. |
| `VNavIsReady() -> Bool` | Volume machinery up and the physics probe resolved. |
| `VNavStat(key: CName) -> Int32` | See keys below. |

`VNavStat` keys: `queries`, `succeeded`, `partial`, `failed`, `probes`, `chunksSampled`,
`chunksFromBake`, `residentChunks`, `residentKB`, `jobsPending`, `jobsParked`, `jobsRunning`,
`pendingRequests`, `usingFilterPath` (1 = fast QueryFilter probe path), `nativePump` (1 = the
plugin's own pump drives the sampler).

### Status codes (`VNavStatus`)

`Invalid -1 · Pending 0 · WaitingForWorld 1 · Running 2 · Success 3 · PartialSuccess 4 ·
Unreachable 5 · Cancelled 6 · BudgetExhausted 7 · BadInput 8 · NoVolume 9 · TimedOut 10`

`VNav.IsTerminal(status)` — anything `>= Success`. `VNav.IsUsable(status)` — `Success` or
`PartialSuccess`.

### Helper classes

- `VNav` — `IsTerminal`, `IsUsable`, `WorldMask()` (15), `WorldAndVehiclesMask()` (47),
  `GetPoints(handle) -> array<Vector4>`, `CreateVolumeAround(name, center, halfExtent, voxelSize,
  agentRadius, margin)`.
- `VNavDynamic` — `Refresh(volume, instigator, radius) -> Int32`, `Clear(volume) -> Int32`,
  `PathHitsTraffic(gi, path, clearance, out intersection) -> Bool` (vanilla traffic check;
  reject-only, never routes around).
- `VNavObstacleSize` — `Vehicle()` 5.6 x 2.6 x 2.0 m, `Puppet()`, `Unknown()` half-extents.

### CET Lua notes

- The natives are **global functions** in Lua: `VNavFindPathAsync("drone", from, to)`. Names are
  bare identifiers (CET resolves globals through its sandbox fallback table — `VNavIsReady`
  works; `VNavIsReady = VNavIsReady` style indirection does not).
- `CName` parameters take Lua strings; handles are plain integers; `Vector4` values are the CET
  type (`ToVector4{...}` or `Vector4.new(...)`).
- None of the vnav functions use optional parameters — every call takes its full argument list.
  (This is deliberate: CET's Lua→redscript bridge cannot pass "unset" optionals.)
- The shipped `bin/.../VNavTest/init.lua` is a complete working example — every function above
  is exercised from Lua there, with UI, drawing, flight recording and error handling.

---

## 6. C ABI reference (RED4ext plugins)

`include/vnav/vnav_capi.h` — plain C, no STL in any signature:

```cpp
auto getApi = (VNav_GetApiFn)GetProcAddress(GetModuleHandleW(L"PathfindingUpdate.dll"), "VNav_GetApi");
auto* api   = (const VNav_Api_v1*)getApi(1);
if (api && api->structSize >= sizeof(VNav_Api_v1)) { /* ... */ }
```

Always fill descriptors from `VolumeDescDefaults` / `QueryDescDefaults`, then override — that
keeps you source-compatible when defaults change. The struct only grows at the end within a
version, so check `structSize` before using anything added later.

| Function pointer | Mirrors |
|---|---|
| `VolumeDescDefaults`, `QueryDescDefaults` | library defaults |
| `CreateVolume`, `DestroyVolume`, `HasVolume` | `VNavCreateVolume/DestroyVolume/HasVolume` (extra `VolumeDesc` fields: `maxProbeDepth` 4, `maxProbesPerChunk` 9216, `maxResidentChunks` 32768, `chunkTtlSeconds` 0) |
| `AttachBake`, `HasBake` | `VNavAttachBake/HasBake` |
| `SubmitAsync`, `FindPathSync` | `VNavFindPathAsync/Sync` (returns handle or 0) |
| `GetInfo(handle, VNav_PathInfo*)` | status + length, snapped endpoints, expansions/probes/parks, elapsed |
| `CopyPoints(handle, dst, max)` | `VNavPathPoint(PointCount)` in one call |
| `Cancel`, `Release` | `VNavCancelPath/VNavPathRelease` |
| `InvalidateBox`, `InvalidateAll` | same |
| `Pump` | `VNavPump` (game thread; normally unnecessary) |
| `Stat`, `IsReady` | `VNavStat/VNavIsReady` (same keys) |

`VNav_QueryDesc` carries the per-query tuning: `snapRadius, allowPartial, maxExpansions,
maxProbesPerQuery, heuristicWeight, tightnessPenalty, verticalCostMultiplier, smoothing,
splineSamplesPerSegment, prefetchRadiusChunks, timeoutSeconds`.

**Not in the C ABI (v1)** — use redscript or link `vnav_core` for these:

- `AttachBakeChecked` (game-build stamp verification)
- per-volume tuning setters (`SetVolumeTuning`, `SetSearchPersistence`, `SetAgentLength`,
  `SetYawSearch`, `SetCoarseGuidance`, `SetBidirectional`)
- escape/unstick (`FindEscape`, `EscapePosition`, `EscapeDistance`)
- per-waypoint detail (`PathClearance`, `PathYaw`, `PathInfeasibleTurns`) and `PathResumeIndex`
- `IsFree`
- the dynamic-obstacle layer (`DynamicBegin/Add/Commit`)

A `VNav_Api_v2` covering these is the natural next step; the version check in `VNav_GetApi`
already separates the two.

---

## 7. C++ surface

Link `vnav_core` and include `vnav/NavService.hpp`, `vnav/Config.hpp`, `vnav/BakedNav.hpp`. This
is the full surface: `NavService` (`CreateVolume`, `SubmitAsync`, `FindPathSync`, `Poll`,
`FindEscape`, `IsFree`, `AttachBake[Checked]`, `SetDynamicObstacles`, `InvalidateBox/All`,
`ResumeIndex`, `GetStats`, `Pump`, `Shutdown`), configured by `VolumeConfig`, `PathQuery`,
`EscapeQuery`, answered by `PathResult`, `EscapeResult`, `Stats`. You own threading (the
contract in §8 applies verbatim) and must feed `Pump` from your own game-thread tick.

---

## 8. Contracts and known limits (read before shipping)

- **Threading.** `IWorldProbe` — and therefore `Pump` — is **game thread only**; everything else
  is safe from any thread. Script callbacks always fire from `Pump`, never from a worker.
- **`agentRadius` sizing.** Line-of-sight sweeps a capsule along every straight segment, so a long
  vehicle's *length* is already covered on straights. Size the radius to the vehicle's
  **cross-section** (max of half-width/half-height), not half its length. Bends are the real gap —
  use `VNavSetAgentLength` for those. Sizing too large is not safer; it just refuses gaps the
  agent could fly through.
- **Always `VNavPathResumeIndex` on replan.** Queries take 0.2–1.7 s; starting the next path at
  waypoint 0 turns the agent around and flies it *back* to where it asked. Measured: 72% of
  adoptions reversed before the fix; 206 → 0 after. Sample replan cadence to distance travelled,
  not wall-clock (1 Hz replanning on a 9 m/s agent throws away everything past one segment).
- **`PartialSuccess` is usable.** Treat `Unreachable`/`BudgetExhausted`/`BadInput` as failures.
- **Clearance saturates** at the volume's `agentRadius + margin`. Read it as "at least this much
  room", never as a rangefinder. Raise `margin` for more range. Never an over-estimate.
- **Long-range ceiling: ~250 m** in built-up areas (expansion budget, not distance); open badlands
  goes considerably further. Past that you get `PartialSuccess` — fly what you got and replan, or
  raise `maxExpansions`.
- **Invalidation vs dynamic obstacles.** `VNavInvalidateBox` is for geometry that changed and
  stays changed — it re-samples at chunk (16 m) granularity and marks the area probe-only. Moving
  things belong in the dynamic layer (§5), which swaps one snapshot handle.
- **The bake is a game-build snapshot.** Use `VNavAttachBakeChecked` with the running build string
  ("2.5") so a future patch's geometry drift becomes a loud attach failure instead of silent
  wrong paths. Bake voxel size must match the volume's `voxelSize`.
- **Underground reads as open** where no collision exists (terrain is a surface, not a volume) —
  harmless for a flying agent starting in valid free space; a start point below ground is not
  walled in.
- **No steering, no avoidance.** This plans; movement and local avoidance are yours.
- **Handle hygiene.** `VNavPathRelease` every handle. A finished job's result is reaped ~30 s
  after completion; once you have read it, it is latched (up to 256 results) and stays readable
  until you release it.

---

## 9. Versioning

- **redscript/Lua surface**: additive. New functions appear; existing signatures do not change
  (that is why tuning setters are separate functions rather than more parameters).
- **C ABI**: versioned by `VNav_GetApi(version)`; within `VNav_Api_v1` the struct grows only at
  the end — check `structSize`.
- **Bake format**: header-versioned (`vnav_bake --inspect` prints it); the shipped bake is
  version 2, 1 m voxels, stamped game build 2.5.
- **Game patches**: the DLL is compiled against the engine ABI (RED4ext.SDK pin) — a patch may
  require a rebuild of the plugin, independent of your mod. Your integration code does not
  change.
