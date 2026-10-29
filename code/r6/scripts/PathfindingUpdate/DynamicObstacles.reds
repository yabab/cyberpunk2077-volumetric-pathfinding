// vnav - reference producer for the dynamic obstacle layer.
//
// WHY THIS FILE EXISTS
//
// The library ships the dynamic obstacle LAYER (VNavDynamicBegin/Add/Commit): a spatially-hashed,
// immutable snapshot consulted as a second predicate after the static clearance test. That half is
// done and tested. What it does not ship is anything that FEEDS it - so out of the box the layer
// is a correctly-plumbed empty pipe, and the planner happily routes through traffic.
//
// This is the missing half: enumerate what is moving nearby and publish it as obstacles.
//
// WHY IT HAS TO WORK AT ENTITY LEVEL (this was checked, not assumed)
//
// The obvious idea is to let the physics probe find moving things - CollisionMask::Vehicle exists,
// after all. It cannot work, for two independent reasons:
//
//   1. TraceResult carries { position, normal, material } and NO entity handle. An overlap can
//      tell you something is there; it can never tell you what, and it cannot give you extents.
//      There is nothing to build an AABB from.
//   2. Even if it could, probe results feed the CACHED voxel grid, which is chunk-granular and
//      shared. Writing a car into it re-voxelises whole chunks every frame and marks them
//      probe-only, permanently undoing the bake in that area. That is precisely the problem the
//      separate dynamic layer exists to avoid.
//
// So the producer must enumerate entities, which is what this does.
//
// TWO TIERS, and they are complementary rather than alternatives:
//
//   Tier 1  VNavDynamic.Refresh()      - PREVENTIVE. Publishes obstacles, so the SEARCH routes
//                                        around them. This is the real solution.
//   Tier 2  VNavDynamic.PathHitsTraffic() - REACTIVE. One vanilla native call that validates a
//                                        finished path against live traffic. Nearly free, and it
//                                        catches what moved since the last Refresh - but it can
//                                        only reject a path, never route around.
//
// Use Tier 1 as the mechanism and Tier 2 as the safety net just before committing to a path.

// Half-extents used for obstacles, in metres. Deliberately approximate: the planner inflates every
// box by the volume's agentRadius anyway, and an obstacle that is a little too big costs a slightly
// wider berth while one that is too small costs a collision. Erring large is the safe direction.
public class VNavObstacleSize {
  public static func Vehicle() -> Vector4 { return new Vector4(2.8, 1.3, 1.0, 0.0); }   // ~5.6 x 2.6 x 2.0 m
  public static func Puppet() -> Vector4 { return new Vector4(0.5, 0.5, 1.0, 0.0); }    // a standing human
  public static func Unknown() -> Vector4 { return new Vector4(0.75, 0.75, 0.75, 0.0); }
}

public class VNavDynamic {

  // Republish the dynamic obstacle set around `instigator`.
  //
  // Returns the number of obstacles now in effect (0 clears the layer). Cheap enough to call every
  // frame - it builds a spatial hash and swaps one handle - but every ~0.2 s is plenty for traffic
  // at city speeds, and the search pins the snapshot when it starts so a stale frame cannot tear.
  //
  // ⚠️ SCOPE, stated because it is easy to over-trust: this enumerates what the TARGETING system
  // considers a target near the instigator. That reliably covers live NPCs and occupied/traffic
  // vehicles - the things that actually move - but it is not a guarantee of every physical object.
  // Empty parked cars in particular may not be targetable. Pair it with Tier 2 rather than assuming
  // the snapshot is complete.
  public static func Refresh(volume: CName, instigator: ref<GameObject>, radius: Float) -> Int32 {
    if !IsDefined(instigator) {
      return 0;
    }

    // Guard the lookup, not the result.
    let targeting: ref<TargetingSystem> = GameInstance.GetTargetingSystem(instigator.GetGame());
    if !IsDefined(targeting) {
      return 0;
    }

    let query: TargetSearchQuery = TSQ_ALL();
    query.maxDistance = radius;
    query.filterObjectByDistance = true;
    query.ignoreInstigator = true;

    let parts: array<TS_TargetPartInfo>;
    targeting.GetTargetParts(instigator, query, parts);

    VNavDynamicBegin();

    // GetTargetParts returns PARTS, and one entity can expose several targetable components - a
    // vehicle has a handful. Publishing each part would stack duplicate boxes on the same car, so
    // dedupe by entity. This array is a local, never a field: an array in a shared mutable field is
    // the concurrency mistake this workspace has paid for repeatedly.
    let seen: array<EntityID>;
    let count: Int32 = 0;
    let i: Int32 = 0;

    while i < ArraySize(parts) {
      let component: wref<TargetingComponent> = TS_TargetPartInfo.GetComponent(parts[i]);
      if IsDefined(component) {
        let entity: wref<Entity> = component.GetEntity();
        if IsDefined(entity) {
          let id: EntityID = entity.GetEntityID();
          if !ArrayContains(seen, id) {
            ArrayPush(seen, id);
            VNavDynamicAdd(entity.GetWorldPosition(), VNavDynamic.HalfExtentsFor(entity));
            count += 1;
          };
        };
      };
      i += 1;
    };

    return VNavDynamicCommit(volume);
  }

  // Clear the layer. Call when the consumer stops planning, or the last snapshot stays in effect
  // and keeps blocking space that nothing occupies any more.
  public static func Clear(volume: CName) -> Int32 {
    VNavDynamicBegin();
    return VNavDynamicCommit(volume);
  }

  // Tier 2: does this path cross live traffic?
  //
  // Vanilla's own check, and it takes exactly the shape VNav.GetPoints() returns. Costs one native
  // call, needs no enumeration, and is accurate for traffic specifically - so it is worth running
  // on a finished path even when Tier 1 is already active, because traffic moves between refreshes.
  //
  // Only answers for TRAFFIC, and only yes/no plus one intersection point. It cannot route around
  // anything; on a hit, replan (optionally after a Refresh) rather than trying to patch the path.
  public static func PathHitsTraffic(gi: GameInstance, path: array<Vector4>, clearance: Float, out intersection: Vector4) -> Bool {
    if ArraySize(path) < 2 {
      return false;
    }
    let traffic: ref<TrafficSystem> = GameInstance.GetTrafficSystem(gi);
    if !IsDefined(traffic) {
      return false;
    }
    return traffic.IsPathIntersectingWithTraffic(path, clearance, intersection);
  }

  private static func HalfExtentsFor(entity: wref<Entity>) -> Vector4 {
    let vehicle: ref<VehicleObject> = entity as VehicleObject;
    if IsDefined(vehicle) {
      return VNavObstacleSize.Vehicle();
    };
    let puppet: ref<ScriptedPuppet> = entity as ScriptedPuppet;
    if IsDefined(puppet) {
      return VNavObstacleSize.Puppet();
    };
    return VNavObstacleSize.Unknown();
  }
}
