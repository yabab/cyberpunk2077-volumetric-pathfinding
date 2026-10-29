// vnav - redscript bindings for PathfindingUpdate.dll (volumetric 3D pathfinding).
//
// INTEGRATION REFERENCE: docs/INTEGRATION.md (in the mod's repository/release) documents every
// function below — all four surfaces (redscript, CET Lua, the C ABI for RED4ext plugins, and the
// C++ core), defaults, recipes, and the contracts to read before shipping. This file carries the
// per-function detail; the guide is the map.
//
// The DLL registers every function below as an RTTI global; these declarations just make them
// visible to redscript. If the plugin is not installed, compilation of this file fails - which
// is the intended behaviour, since nothing here works without it.

// =============================================================================================
// Native API
// =============================================================================================

// Volumes. One volume == one agent profile: agent radius is baked into the sampled data, so a
// drone and an AV want two volumes, not one.
//   voxelSize    - metres per voxel. Pick roughly the agent radius.
//   agentRadius  - collision radius; probe boxes are inflated by it.
//   margin       - extra slack. Space that is free at agentRadius but not at agentRadius+margin
//                  is traversable but penalised, so paths prefer open air.
//   collisionMask- 0 for the default (Static|Terrain|Collider|Destructible).
public native func VNavCreateVolume(name: CName, voxelSize: Float, agentRadius: Float, margin: Float, boundsMin: Vector4, boundsMax: Vector4, collisionMask: Uint32) -> Bool;
public native func VNavDestroyVolume(name: CName) -> Bool;
public native func VNavHasVolume(name: CName) -> Bool;

// Attaches a baked static world (.vnavbake) built offline from the game archives.
//
// This is not just an optimisation. The runtime physics probe can only see collision that is
// currently STREAMED IN, so a query beyond the streaming radius reads back as open air and
// happily routes through buildings that have not loaded. A bake sees the whole map at once.
// The probe stays on underneath it, handling doors, vehicles and destruction.
//
// The path is relative to the game root, e.g. "red4ext/plugins/PathfindingUpdate/nightcity.vnavbake".
// The bake's voxel size must match the volume's.
public native func VNavAttachBake(volume: CName, path: String) -> Bool;

// Same, but refuses a bake built from a different game build.
//
// ⚠️ Prefer this one. A bake describes the world as it SHIPPED. If CDPR patches the game and
// geometry moves, an old bake keeps planning confidently through walls that are no longer there —
// silently, because nothing about the data looks wrong. This turns that into a load-time failure.
//
// Pass the running game's build string. An UNSTAMPED bake (one written before stamping existed)
// cannot be checked, so it attaches anyway and logs a warning — refusing those would break every
// bake already shipped. Pass "" to skip checking entirely (equivalent to VNavAttachBake).
public native func VNavAttachBakeChecked(volume: CName, path: String, gameBuild: String) -> Bool;
public native func VNavHasBake(volume: CName) -> Bool;

// Search tuning, stored per volume and applied to every query against it.
//   smoothing: 0 = raw voxels, 1 = string-pull (default), 2 = spline
public native func VNavSetVolumeTuning(name: CName, heuristicWeight: Float, tightnessPenalty: Float, verticalCostMultiplier: Float, snapRadius: Float, maxExpansions: Int32, smoothing: Int32) -> Bool;

// How hard an ASYNC search is allowed to work before giving up. Sync queries ignore this: they
// block the caller, so they stay bounded and fail fast.
//   maxParks        0 = unlimited (default). Otherwise a hard ceiling on game-thread round trips.
//   noProgressParks give up after this many CONSECUTIVE parks that got no closer to the goal.
//                   Bounds lack of progress rather than effort, so a long search through complex
//                   geometry runs as long as it needs while a trapped one still terminates.
//   timeoutSeconds  wall-clock backstop, 0 disables.
public native func VNavSetSearchPersistence(name: CName, maxParks: Int32, noProgressParks: Int32, timeoutSeconds: Float) -> Bool;

// ---------------------------------------------------------------------------------------------
// Non-sphere agents — TIER 1 of 2 (the cheap one). See VNavSetYawSearch for the full 4D version.
// ---------------------------------------------------------------------------------------------
//
// Length of the agent along its direction of travel, in metres. 0 (default) = treat it as a
// sphere, the historical behaviour.
//
// ⚠️ agentRadius stays the CROSS-SECTION half-extent (larger of half-width / half-height), NOT
// half the length. Line-of-sight already sweeps a sphere along every segment, and a swept sphere
// IS a capsule — so a long body is already covered on straight runs. Sizing the radius to the
// length only makes the planner refuse gaps the agent could fly straight through.
//
// The real gap is at BENDS, where a rotating long body sweeps outside that capsule. Setting this
// makes the smoother verify the body can actually rotate through each corner it creates, and
// report whatever it could not fix via VNavPathInfeasibleTurns.
//
// Scope: constrains SMOOTHING, not the search — the search still plans for the capsule. Cheap.
//   turnSamples  intermediate orientations tested per corner (5 is a good default).
public native func VNavSetAgentLength(name: CName, agentLength: Float, turnSamples: Int32) -> Bool;

// ---------------------------------------------------------------------------------------------
// Non-sphere agents — TIER 2 of 2 (the expensive one). Orientation as a real search dimension.
// ---------------------------------------------------------------------------------------------
//
// Plans over (x, y, z, yaw) instead of (x, y, z). Requires VNavSetAgentLength first — for a
// sphere every yaw is identical, so this silently falls back to 3D without it.
//
// What it buys over tier 1: agentLength only constrains SMOOTHING, so it can flag a corner after
// the fact but never discover that a long body has to be aligned a particular way to fit at all.
// This searches orientation, so it can route a body lengthwise through a slot too narrow to cross
// sideways, decide to rotate in place before entering one, and refuse a route outright when no
// orientation works. Measured on an L-bend with a 16 m body in an 8 m corridor: tier 1 returns
// Success and reports 2 bad corners; this returns Unreachable, because it is.
//
// What it costs: the state space is multiplied by yawBuckets, and any-angle shortcutting is off
// (a straight-line shortcut is not validated against orientation), so paths come back as grid
// moves. Use it for tight interior spaces; tier 1 gets most of the benefit in open airspace.
//
//   yawBuckets        0 = 3D (default). >= 4 enables 4D. 8 is a sensible start; capped at 64.
//   yawCost           cost of one bucket of rotation, in metres-equivalent.
//   yawFollowsTravel  keep the nose pointing roughly where it is going. Leave this ON: with it
//                     off, yaw is unconstrained wherever the body fits at any angle, so the
//                     search never rotates and every returned yaw is the starting one.
//   yawTravelCone     half-angle (radians) allowed between heading and horizontal travel.
//                     Must be at least one bucket wide. 0.785 (45°) suits 8 buckets.
public native func VNavSetYawSearch(name: CName, yawBuckets: Int32, yawCost: Float, yawFollowsTravel: Bool, yawTravelCone: Float) -> Bool;

// Hierarchical guidance: plan a coarse route over whole 16 m CHUNKS first, then bias the voxel
// search toward it. On by default. Requires an attached bake — the coarse graph is read straight
// out of the bake index (one state byte per chunk, absent = open sky), so it costs no probing, no
// decoding and no file reads.
//
// Measured on a 250 m route past a thick barrier: 900,725 expansions unguided vs 249,745 guided —
// 3.6x less search for 311 coarse expansions, same path length.
//
// ⚠️ Only helps where barriers are CHUNK-SCALE. The coarse graph blocks on wholly-solid chunks, so
// a thin wall yields Mixed chunks it considers passable and guidance has nothing to say. Buildings
// and terrain qualify; a 2 m fence does not.
//
//   penalty  cost multiplier for voxel steps that leave the corridor. A bias, deliberately NOT a
//            restriction: 16 m blocks are too coarse to be trusted absolutely, so the search stays
//            free to leave the corridor when that is the only way through. 1.0 disables the bias.
public native func VNavSetCoarseGuidance(name: CName, enabled: Bool, penalty: Float) -> Bool;

// Search from both ends and meet in the middle. On by default.
// Measured: ~1.9x the expansions on a 50m hop, but on a 640m route through obstacles the
// one-directional search does not finish at all (budget exhausted -> partial path) while this
// completes for ~35% less work. It also detects an unreachable goal far sooner, because the
// small frontier sealed in around the goal drains almost immediately.
public native func VNavSetBidirectional(name: CName, enabled: Bool) -> Bool;

// Both return a handle, or 0 on failure. Async is strongly preferred: it plans on worker
// threads and samples the world a frame at a time. Sync blocks and is only appropriate for
// short hops through space that has already been mapped.
public native func VNavFindPathAsync(volume: CName, start: Vector4, goal: Vector4) -> Uint32;
public native func VNavFindPathSync(volume: CName, start: Vector4, goal: Vector4) -> Uint32;

// Reading a result back. Poll VNavPathStatus until it reports a terminal value.
public native func VNavPathStatus(handle: Uint32) -> Int32;
public native func VNavPathPointCount(handle: Uint32) -> Int32;
public native func VNavPathPoint(handle: Uint32, index: Int32) -> Vector4;
// Free radius at waypoint `index`, in metres — how much room the agent actually has there.
// The search tested this value anyway, so reading it back is free. Use it to cut corners, bank
// into a turn, or modulate speed against real headroom rather than re-querying the volume.
//
// ⚠️ This SATURATES, and usually quite close in — read it as "at least this much room", never as
// "exactly this much". It is a tightness signal, not a rangefinder. Two reasons, both deliberate:
//   1. Clearance is only computed out to the volume's tight threshold (agentRadius + margin);
//      past that everything is equally traversable so it is not measured. At agentRadius 0.5 with
//      margin 0.5 that is ~2 m, so a waypoint 4 m from anything already reads saturated. Raise
//      `margin` when creating the volume to buy more range.
//   2. Without a bake, a physics probe only answers "is this box clear?", so the sampler writes
//      just three levels: 0 (blocked), ~agentRadius+margin (squeezing), or saturated. A bake does
//      grade properly, but still only below the cut-off in (1).
// Never an over-estimate, which is the safe direction. Returns 0 if the index is out of range.
public native func VNavPathClearance(handle: Uint32, index: Int32) -> Float;

// Corners the configured agent body cannot rotate through (see VNavSetAgentLength). The smoother
// already avoids creating these, so non-zero means the ROUTE ITSELF is too tight for the body —
// treat it as "this path exists but my agent cannot fly it", not as a smoothing artefact.
// Always 0 for a sphere agent (agentLength 0).
public native func VNavPathInfeasibleTurns(handle: Uint32) -> Int32;

// Body heading at waypoint `index`, in radians about the vertical axis. Only meaningful for a 4D
// plan (see VNavSetYawSearch) — a 3D plan never decided an orientation, so this returns 0.
// For a 4D plan this is part of the answer, not a convenience: the search may have chosen to
// rotate in place before entering a gap, and that shows up here as consecutive waypoints at the
// same position with different yaw.
public native func VNavPathYaw(handle: Uint32, index: Int32) -> Float;

public native func VNavPathLength(handle: Uint32) -> Float;
public native func VNavPathRelease(handle: Uint32) -> Void;
public native func VNavCancelPath(handle: Uint32) -> Bool;

// ---------------------------------------------------------------------------------------------
// Escape / unstick — recovery when there is no path at all
// ---------------------------------------------------------------------------------------------
//
// A failed path query is not actionable: the agent is told Unreachable and has nowhere to go, so
// it stops and never moves again. Measured in testing as 0 m of movement over 32 s, terminal.
// This finds somewhere to fly TO, chosen purely on room rather than on progress toward a goal that
// is currently unreachable.
//
// Returns a VNavStatus: Success, Unreachable (nothing qualifying in range), BudgetExhausted, or
// BadInput. Read the answer with VNavEscapePosition / VNavEscapeDistance immediately afterwards.
//
//   searchRadius  how far to look, in metres. Bounds the work.
//   minClearance  room required at the destination. 0 = "anywhere the agent fits", right when
//                 wedged in geometry.
//   minDistance   ⚠️ refuse answers closer than this. Load-bearing for the second kind of stuck:
//                 if the agent is sitting in OPEN space where every route to the goal fails, the
//                 nearest qualifying position is where it already is — a correct answer that
//                 changes nothing. Demanding real distance turns this into "go somewhere else and
//                 replan from there", which is what actually escapes a local minimum.
//
// Game thread only, and synchronous — the work is bounded, and a stuck agent is not busy.
// Can the agent occupy this exact point? Samples the containing chunk if needed, so this is a real
// answer rather than "not loaded, therefore no".
//
// ⚠️ Use this before committing to an OFFSET goal. "Hover N metres above the target" is
// unremarkable outdoors and puts the goal inside the ceiling indoors — and the query failure that
// follows looks like a pathfinding bug rather than a bad goal. This was a direct contributor to an
// agent stranding permanently in testing.
public native func VNavIsFree(volume: CName, point: Vector4) -> Bool;

public native func VNavFindEscape(volume: CName, from: Vector4, searchRadius: Float, minClearance: Float, minDistance: Float) -> Int32;
public native func VNavEscapePosition() -> Vector4;
public native func VNavEscapeDistance() -> Float;

// ---------------------------------------------------------------------------------------------
// Replanning — where to resume a path that just arrived
// ---------------------------------------------------------------------------------------------
//
// ⚠️ USE THIS WHENEVER YOU REPLAN CONTINUOUSLY. Skipping it produces a bug that looks like broken
// physics or steering, not like a pathfinding mistake: the query takes 0.2–1.7 s to come back, the
// agent has moved on meanwhile, and starting the new path at waypoint 0 makes it turn around and
// fly BACK to where it was when it asked.
//
// Measured in the test harness before the fix: 122 path adoptions, 72% followed by a ~180° reversal
// (heading dot ≈ −1.00), peak deviation 7.9 m. Adding this took reversals from 206 to 0.
//
// Returns the waypoint index to steer at — the far end of the segment nearest the agent, so
// progress already made is kept. Pure geometry over the path; safe to call any time.
public native func VNavPathResumeIndex(handle: Uint32, agentPosition: Vector4) -> Int32;

// ---------------------------------------------------------------------------------------------
// Dynamic obstacles — things that MOVE
// ---------------------------------------------------------------------------------------------
//
// Other vehicles, NPCs, opened doors, moving platforms. Rebuild the whole set as often as you
// like (every frame is fine): it builds a spatial hash and swaps a single handle.
//
// ⚠️ Do NOT use VNavInvalidateBox for moving things. That re-samples at CHUNK granularity — 16
// voxels a side — and writes the result into the cached STATIC map, so a car moving one metre
// would re-voxelise whole chunks every frame and permanently mark them "probe only", undoing the
// bake in that area. These boxes instead live in a separate layer consulted after the static
// test, which leaves chunks immutable and the bake intact. Invalidate is for geometry that
// changed and STAYS changed; this is for geometry that is still moving.
//
// Boxes are inflated by the volume's agentRadius automatically — pass the obstacle's own extents.
// A search pins the set when it starts, so obstacles never appear or vanish mid-query.
//
// Usage: Begin, then Add once per obstacle, then Commit. Commit returns the count now in effect;
// committing with nothing added clears the layer.
public native func VNavDynamicBegin() -> Void;
public native func VNavDynamicAdd(center: Vector4, halfExtents: Vector4) -> Void;
public native func VNavDynamicCommit(volume: CName) -> Int32;

// Cached geometry goes stale when the world changes. Call after anything that alters
// collision: doors, destruction, a vehicle parking somewhere new.
public native func VNavInvalidateBox(volume: CName, center: Vector4, halfExtents: Vector4) -> Void;
public native func VNavInvalidateAll(volume: CName) -> Void;

public native func VNavPump(budgetMicros: Int32) -> Void;
public native func VNavIsReady() -> Bool;
public native func VNavStat(key: CName) -> Int32;

// =============================================================================================
// Status
// =============================================================================================

enum VNavStatus {
  Invalid = -1,
  Pending = 0,
  WaitingForWorld = 1,
  Running = 2,
  Success = 3,
  PartialSuccess = 4,
  Unreachable = 5,
  Cancelled = 6,
  BudgetExhausted = 7,
  BadInput = 8,
  NoVolume = 9,
  TimedOut = 10,
}

public class VNav {
  // Terminal means the job is over, one way or another. Stop polling.
  public static func IsTerminal(status: Int32) -> Bool {
    return status >= EnumInt(VNavStatus.Success);
  }

  // Usable means there are waypoints worth flying. PartialSuccess counts: for a flying agent,
  // getting most of the way there beats refusing to move.
  public static func IsUsable(status: Int32) -> Bool {
    return status == EnumInt(VNavStatus.Success) || status == EnumInt(VNavStatus.PartialSuccess);
  }

  // Collision group bits, matching the native CollisionMask.
  public static func WorldMask() -> Uint32 { return 15u; }               // Static|Terrain|Collider|Destructible
  public static func WorldAndVehiclesMask() -> Uint32 { return 47u; }    // + Vehicle

  // Copy a finished path into an array. Returns an empty array unless the handle is usable.
  public static func GetPoints(handle: Uint32) -> array<Vector4> {
    let points: array<Vector4>;
    if !VNav.IsUsable(VNavPathStatus(handle)) {
      return points;
    }
    let count: Int32 = VNavPathPointCount(handle);
    let i: Int32 = 0;
    while i < count {
      ArrayPush(points, VNavPathPoint(handle, i));
      i += 1;
    };
    return points;
  }

  // Convenience: create a volume centred on a point, sized by a half-extent.
  public static func CreateVolumeAround(name: CName, center: Vector4, halfExtent: Float, voxelSize: Float, agentRadius: Float, margin: Float) -> Bool {
    let lo: Vector4 = new Vector4(center.X - halfExtent, center.Y - halfExtent, center.Z - halfExtent, 1.0);
    let hi: Vector4 = new Vector4(center.X + halfExtent, center.Y + halfExtent, center.Z + halfExtent, 1.0);
    return VNavCreateVolume(name, voxelSize, agentRadius, margin, lo, hi, VNav.WorldMask());
  }
}

// =============================================================================================
// Fallback pump
// =============================================================================================
//
// The plugin drives its own per-frame update from RED4ext's Running-state OnUpdate, which is a
// game-thread callback. This shim exists only in case that does not fire: it stands itself down
// the moment the native pump reports itself active, so the two can never both run.

public class VNavPumpCallback extends DelayCallback {

  public let m_game: GameInstance;

  public static func Create(game: GameInstance) -> ref<VNavPumpCallback> {
    let self: ref<VNavPumpCallback> = new VNavPumpCallback();
    self.m_game = game;
    return self;
  }

  public func Call() -> Void {
    // Native pump confirmed: stop rescheduling and let this chain die.
    if VNavStat(n"nativePump") == 1 {
      return;
    };

    VNavPump(-1);

    let delaySystem: ref<DelaySystem> = GameInstance.GetDelaySystem(this.m_game);
    if IsDefined(delaySystem) {
      delaySystem.DelayCallbackNextFrame(VNavPumpCallback.Create(this.m_game));
    };
  }
}

public class VNavPumpSystem extends ScriptableSystem {

  private func OnAttach() -> Void {
    let game: GameInstance = this.GetGameInstance();
    let delaySystem: ref<DelaySystem> = GameInstance.GetDelaySystem(game);
    if IsDefined(delaySystem) {
      delaySystem.DelayCallbackNextFrame(VNavPumpCallback.Create(game));
    };
  }
}
