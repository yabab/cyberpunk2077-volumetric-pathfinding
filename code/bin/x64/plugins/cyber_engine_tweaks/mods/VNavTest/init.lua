-- VNavTest - in-game test harness for PathfindingUpdate (volumetric 3D pathfinding).
--
-- WHY CET AND NOT REDSCRIPT
-- Tuning agent radius is the whole point of this harness, and redscript needs a game restart per
-- change. Here a slider re-plans live. That is only cheap because the bake stores occupancy, not
-- per-agent data: changing radius re-derives clearance from the same chunks instead of re-probing
-- the world.
--
-- HOW TO USE IT
-- The panel is ordered so each section only makes sense once the one above it is green. If
-- something fails, it fails at the highest section that is still red - which is the point.
--   1. Plugin   - is the DLL loaded and are the natives registered?
--   2. Volume   - does the bake attach, and does it cover where you are standing?
--   3. Plan     - does a query come back at all?
--   4. Draw     - does the route look sane in the world, or does it clip a building?
--   5. AV       - fly something along it.

local M = {
    open = true,

    -- Single switch for the harness's console output (see note() below). Off for shipping: the
    -- panel reports every status through M.lastError / M.logStatus, so console prints are pure
    -- debug output.
    debug = false,

    -- Volume. voxelSize is locked to the bake's; a mismatch is refused at attach time.
    volumeName = "avtest",
    voxelSize = 1.0,
    agentRadius = 2.5,
    margin = 1.5,
    halfExtent = 4000.0,
    bakePath = "red4ext/plugins/PathfindingUpdate/nightcity.vnavbake",

    volumeReady = false,
    bakeReady = false,
    lastError = "",

    -- Query
    handle = 0,
    lastStatus = -1,
    lastLength = 0.0,
    lastPoints = {},
    lastElapsed = 0.0,
    goal = nil,
    goalIsPlayer = false,
    hoverHeight = 12.0, -- metres above the player the goal sits when "goal follows player" is on

    autoReplan = false,
    replanInterval = 1.0,
    replanTimer = 0.0,
    hoverApplied = 0.0, -- H2: hover offset that actually survived the open-space check

    -- H4: replan on metres travelled, not only on wall-clock. Evidence for why: 483 paths were
    -- computed in one session and roughly ONE SEGMENT of each was flown, so almost all of that
    -- planning was thrown away. Distance is what actually invalidates a path; time is a proxy
    -- that fires hardest exactly when the agent is stationary and the path is still perfectly good.
    replanOnDistance = true,
    replanDistance = 15.0,   -- metres of agent travel since the last plan
    replanGoalDistance = 10.0, -- or metres the goal has moved since the last plan
    planFromPos = nil,
    planGoalPos = nil,

    -- H1: stranding detection. "No usable path AND not moving" for this long means the agent is
    -- stuck in a way replanning will not fix, because replanning is what already failed.
    strandSeconds = 4.0,
    strandTimer = 0.0,
    strandPos = nil,
    strandCount = 0,
    escapeStatus = -1,
    escapeInfo = "",
    autoEscape = true,

    -- H3: search tuning, previously exposed by the plugin but unreachable from this panel - so
    -- the expansion ceiling could not be tested live.
    heuristicWeight = 1.0,
    tightnessPenalty = 1.5,
    verticalCostMultiplier = 1.0,
    snapRadius = 4.0,
    maxExpansions = 60000,
    smoothing = 1,
    coarseGuidance = true,
    coarsePenalty = 4.0,

    -- Agent shape / 4D search (VNavSetAgentLength, VNavSetYawSearch).
    -- agentLength 0 = treat the agent as a sphere; the 4D search is meaningless for a sphere and
    -- the plugin silently falls back to 3D, so raise this before expecting yaw to do anything.
    agentLength = 0.0,
    turnSamples = 5,
    yawBuckets = 0, -- 0 = plain 3D. >= 4 to enable, and only with agentLength > 0.
    yawCost = 0.5,
    yawFollowsTravel = true,
    yawTravelCone = 60.0,

    bidirectional = true,
    maxParks = 512,
    noProgressParks = 64,
    searchTimeout = 5.0,

    -- Sync vs async planning. Sync blocks the game thread until the search finishes, which is the
    -- point: it makes a slow query visible as a hitch instead of hiding it behind the pump.
    planSync = false,
    lastInfeasibleTurns = 0,
    lastClearances = {},
    lastYaws = {},

    -- Dynamic obstacle layer (VNavDynamicBegin/Add/Commit). Boxes are staged here and published
    -- as one immutable snapshot on commit.
    dynBoxes = {},
    dynCommitted = 0,
    dynHalf = 3.0,
    dynAutoCommit = false,
    dynRingRadius = 20.0,

    pumpBudgetMicros = 2000,
    invalidateHalf = 8.0,
    gameBuild = "2.5", -- expected build for VNavAttachBakeChecked; "" skips the check entirely.
    -- Matches the stamp written by `vnav_bake --gamebuild 2.5`, so the checked attach is live by
    -- default. Clear the field in the panel to attach any bake unverified (e.g. one you just
    -- re-baked under a different game version).

    -- Draw
    -- Debug drawing (path lines / radius rings / goal marker). Every one of these is OFF until
    -- ticked in the panel - nothing draws without an explicit opt-in. The in-world drawing also
    -- stays behind the overlay unless drawWithOverlayClosed is on.
    drawWithOverlayClosed = false,
    drawPath = false,
    drawRadius = false,
    drawGoal = false,
    overlayOpen = false, -- tracked from onOverlayOpen/onOverlayClose
    drawEvery = 4, -- spheres are costly; only ring every Nth waypoint

    -- AV
    avRecord = "Vehicle.av_militech_manticore",
    avEntityId = nil,
    avEntity = nil, -- cached handle, resolved lazily; see StepAv
    avFollow = false,
    avSpeed = 18.0,
    avIndex = 1,
    avPos = nil,
    avRecordCandidates = {},

    -- How close the follower must get to a waypoint before it retargets the next one. This is
    -- the corner-cutting knob: the AV never actually reaches a waypoint, it peels off toward the
    -- next one from this far away, so the flown curve cuts inside the planned polyline by up to
    -- roughly this much. Lazy Theta* paths are taut against the clearance boundary already, so
    -- any corner cut leaves the corridor the planner validated.
    waypointTolerance = 2.0,

    -- Live distance from the AV to the nearest point on the planned path, and the worst seen
    -- since the last reset. This is the number that decides whether clipping is the follower's
    -- fault or the agent-shape model's.
    devNow = 0.0,
    devMax = 0.0,

    -- CSV recorder. Every row carries the full tuning config, so ONE file can hold a whole
    -- sweep and still be groupable by (tolerance, speed, radius) afterwards - no need to start
    -- a new file per setting, and no way to mix up which run had which values.
    logEnabled = false,
    logInterval = 0.1, -- 10 Hz; frame-rate logging is a huge file for no extra signal
    logTimer = 0.0,
    logClock = 0.0,
    logFile = nil,
    logRows = 0,
    logName = "vnavtest_run.csv",
    logStatus = "not started",
}

local STATUS = {
    [-1] = "Invalid", [0] = "Pending", [1] = "WaitingForWorld", [2] = "Running",
    [3] = "Success", [4] = "PartialSuccess", [5] = "Unreachable", [6] = "Cancelled",
    [7] = "BudgetExhausted", [8] = "BadInput", [9] = "NoVolume", [10] = "TimedOut",
}

-- The plugin registers its API as RTTI globals, which CET surfaces as plain Lua globals. Do NOT
-- resolve them by indexing a table named "_G" - CET runs mod scripts in a sandboxed environment
-- and does not necessarily expose that table under that name, so `_G[name]` throws
-- "attempt to index global '_G' (a nil value)" on literally every call, every frame (confirmed:
-- it filled VNavTest.log to 5 MB and rotated twice before anyone touched a button).
--
-- Fix: a BARE identifier reference (`VNavIsReady`, not `_G["VNavIsReady"]` or any other table
-- lookup by an assumed name) is safe language-level global resolution - it evaluates to nil,
-- never throws, when the name is undefined - regardless of what CET calls its environment table
-- internally, and regardless of load order between the native plugin and this script. Re-resolved
-- fresh on every call rather than cached, so a DLL that finishes registering after this script
-- starts is still picked up on the next call rather than being stuck nil forever.
local function api(name)
    local fn
    if name == "VNavIsReady" then fn = VNavIsReady
    elseif name == "VNavCreateVolume" then fn = VNavCreateVolume
    elseif name == "VNavDestroyVolume" then fn = VNavDestroyVolume
    elseif name == "VNavAttachBake" then fn = VNavAttachBake
    elseif name == "VNavFindPathAsync" then fn = VNavFindPathAsync
    elseif name == "VNavPathStatus" then fn = VNavPathStatus
    elseif name == "VNavPathPointCount" then fn = VNavPathPointCount
    elseif name == "VNavPathPoint" then fn = VNavPathPoint
    elseif name == "VNavPathLength" then fn = VNavPathLength
    elseif name == "VNavPathRelease" then fn = VNavPathRelease
    elseif name == "VNavStat" then fn = VNavStat
    elseif name == "VNavIsFree" then fn = VNavIsFree
    elseif name == "VNavFindEscape" then fn = VNavFindEscape
    elseif name == "VNavEscapePosition" then fn = VNavEscapePosition
    elseif name == "VNavEscapeDistance" then fn = VNavEscapeDistance
    elseif name == "VNavSetVolumeTuning" then fn = VNavSetVolumeTuning
    elseif name == "VNavPathResumeIndex" then fn = VNavPathResumeIndex
    elseif name == "VNavSetCoarseGuidance" then fn = VNavSetCoarseGuidance
    elseif name == "VNavAttachBakeChecked" then fn = VNavAttachBakeChecked
    elseif name == "VNavHasVolume" then fn = VNavHasVolume
    elseif name == "VNavHasBake" then fn = VNavHasBake
    elseif name == "VNavFindPathSync" then fn = VNavFindPathSync
    elseif name == "VNavCancelPath" then fn = VNavCancelPath
    elseif name == "VNavPathClearance" then fn = VNavPathClearance
    elseif name == "VNavPathYaw" then fn = VNavPathYaw
    elseif name == "VNavPathInfeasibleTurns" then fn = VNavPathInfeasibleTurns
    elseif name == "VNavSetAgentLength" then fn = VNavSetAgentLength
    elseif name == "VNavSetYawSearch" then fn = VNavSetYawSearch
    elseif name == "VNavSetBidirectional" then fn = VNavSetBidirectional
    elseif name == "VNavSetSearchPersistence" then fn = VNavSetSearchPersistence
    elseif name == "VNavDynamicBegin" then fn = VNavDynamicBegin
    elseif name == "VNavDynamicAdd" then fn = VNavDynamicAdd
    elseif name == "VNavDynamicCommit" then fn = VNavDynamicCommit
    elseif name == "VNavInvalidateBox" then fn = VNavInvalidateBox
    elseif name == "VNavInvalidateAll" then fn = VNavInvalidateAll
    elseif name == "VNavPump" then fn = VNavPump
    end
    if type(fn) ~= "function" then
        return nil
    end
    return fn
end

local function apiPresent()
    return api("VNavIsReady") ~= nil
end

local function v4(x, y, z)
    return ToVector4{ x = x, y = y, z = z, w = 1.0 }
end

-- Single logging entry point for the harness. The concatenation is inside the gate, so a
-- disabled note() builds nothing.
local function note(msg)
    if M.debug then
        -- print("[VNavTest] " .. msg) -- sink silenced for shipping; uncomment to debug
    end
end

-- Debug drawing is an OPTIONAL dependency on the Debug Draw mod
-- (https://github.com/jackhumbert/cp2077_debug_draw). Debug Draw declares no redscript module,
-- so `@if(ModuleExists(...))` cannot gate it and a redscript shim would break compilation without
-- it; hence the calls are made from Lua, resolved once at runtime. Missing mod (or a failure) =
-- drawing silently off, nothing else affected.
local ddState = nil -- nil = unresolved, false = unavailable, else the DebugDraw class table
local function debugDraw()
    if ddState == nil then
        local ok, dd = pcall(function()
            local c = DebugDraw
            if c ~= nil and c.Path ~= nil and c.Sphere ~= nil and c.Rgba ~= nil then
                return c
            end
            return nil
        end)
        ddState = (ok and dd) or false
        if not ddState then
            note("Debug Draw mod not found - debug drawing disabled")
        end
    end
    return ddState or nil
end

local function playerPos()
    local p = Game.GetPlayer()
    if not p then return nil end
    return p:GetWorldPosition()
end

local function dist(a, b)
    local dx, dy, dz = a.x - b.x, a.y - b.y, a.z - b.z
    return math.sqrt(dx * dx + dy * dy + dz * dz)
end

-- Lua 5.1/LuaJIT spell this math.atan2; 5.3+ folded it into a two-argument math.atan. CET's
-- runtime has moved between the two, so bind whichever exists rather than assuming.
local atan2 = math.atan2 or function(y, x) return math.atan(y, x) end

-- Distance from p to the segment ab, clamped to the segment (not the infinite line).
local function pointSegDist(p, a, b)
    local abx, aby, abz = b.x - a.x, b.y - a.y, b.z - a.z
    local apx, apy, apz = p.x - a.x, p.y - a.y, p.z - a.z
    local ab2 = abx * abx + aby * aby + abz * abz

    local t = 0.0
    if ab2 > 1e-6 then
        t = (apx * abx + apy * aby + apz * abz) / ab2
        if t < 0.0 then t = 0.0 elseif t > 1.0 then t = 1.0 end
    end

    local dx, dy, dz = apx - abx * t, apy - aby * t, apz - abz * t
    return math.sqrt(dx * dx + dy * dy + dz * dz)
end

-- How far the AV has strayed from the corridor the planner actually validated. Minimum over all
-- segments rather than just the active one, so a follower that overshoots past a waypoint still
-- measures against the nearest part of the path instead of reporting a spurious jump.
local function pathDeviation(p, pts)
    if #pts < 2 then return 0.0 end
    local best = math.huge
    for i = 1, #pts - 1 do
        local d = pointSegDist(p, pts[i], pts[i + 1])
        if d < best then best = d end
    end
    return best
end

-- Which waypoint should a follower at `p` be heading for on a freshly adopted path?
--
-- NOT 1. A path is planned from where the agent was when the query was SUBMITTED, and an async
-- query takes time to come back (measured: up to 1.7 s). At 9 m/s that leaves waypoint 1 about
-- 8 m BEHIND the agent. Retargeting it makes the follower turn around and fly back at full
-- speed, and since a replan lands every second the agent just saws back and forth over the same
-- stretch, never advancing. Measured before this fix: 122 path adoptions, 72% immediately
-- followed by a 180-degree reversal.
--
-- So snap to the nearest segment and aim at its far end - i.e. resume from where the agent
-- actually is, not where it used to be.
local function resumeIndex(p, pts)
    if #pts < 2 then return 1 end
    local best, bestI = math.huge, 1
    for i = 1, #pts - 1 do
        local d = pointSegDist(p, pts[i], pts[i + 1])
        if d < best then best, bestI = d, i end
    end
    return math.min(bestI + 1, #pts)
end

-- ------------------------------------------------------------------------------------------
-- CSV recorder
-- ------------------------------------------------------------------------------------------

-- `io` is assumed present but NOT assumed to behave - CET sandboxes its Lua environment and we
-- have already been bitten once by assuming a standard global exists (see the _G note above).
-- Every filesystem touch is pcall'd and the outcome is surfaced in the UI, so a sandbox that
-- refuses file writes says so on screen instead of silently recording nothing all session.
local LOG_HEADER =
    "t,event,devNow,devMax,tolerance,speed,agentRadius,margin,hoverHeight," ..
    "wpIndex,wpCount,pathLen,status,avx,avy,avz,px,py,pz," ..
    -- H1/H2 verification. hoverApplied below hoverHeight means the raised goal was inside
    -- geometry and got stepped down; strandCount rising means the escape path fired.
    "hoverApplied,strandCount\n"

function M.StartLog()
    M.StopLog()

    local ok, f = pcall(function() return io.open(M.logName, "w") end)
    if not ok or not f then
        M.logFile = nil
        M.logEnabled = false
        M.logStatus = "FAILED to open " .. tostring(M.logName) .. " (sandbox may block io)"
        return
    end

    M.logFile = f
    pcall(function() f:write(LOG_HEADER) f:flush() end)

    M.logRows = 0
    M.logClock = 0.0
    M.logTimer = 0.0
    M.logStatus = "recording -> " .. M.logName
end

function M.StopLog()
    if M.logFile then
        pcall(function() M.logFile:flush() M.logFile:close() end)
        M.logFile = nil
        M.logStatus = string.format("stopped, %d rows in %s", M.logRows, M.logName)
    end
end

-- aEvent is "sample" for the periodic rows and "clip" for ones you marked by hand. Flushed on
-- every write: at 10 Hz the cost is irrelevant, and it means a crash or alt-F4 mid-run still
-- leaves a complete file behind.
function M.LogRow(aEvent)
    if not M.logFile then return end

    local p = playerPos()
    local a = M.avPos

    local line = string.format(
        "%.3f,%s,%.3f,%.3f,%.2f,%.2f,%.2f,%.2f,%.2f,%d,%d,%.2f,%s,%.2f,%.2f,%.2f,%.2f,%.2f,%.2f,%.2f,%d\n",
        M.logClock, aEvent, M.devNow, M.devMax,
        M.waypointTolerance, M.avSpeed, M.agentRadius, M.margin, M.hoverHeight,
        M.avIndex, #M.lastPoints, M.lastLength,
        STATUS[M.lastStatus] or tostring(M.lastStatus),
        a and a.x or 0.0, a and a.y or 0.0, a and a.z or 0.0,
        p and p.x or 0.0, p and p.y or 0.0, p and p.z or 0.0,
        M.hoverApplied, M.strandCount)

    local ok = pcall(function() M.logFile:write(line) M.logFile:flush() end)
    if ok then
        M.logRows = M.logRows + 1
    else
        M.logStatus = "write failed; recording stopped"
        M.logEnabled = false
        M.logFile = nil
    end
end

function M.TickLog(dt)
    if not M.logEnabled or not M.logFile then return end

    M.logClock = M.logClock + dt
    M.logTimer = M.logTimer + dt
    if M.logTimer >= M.logInterval then
        M.logTimer = 0.0
        M.LogRow("sample")
    end
end

-- ------------------------------------------------------------------------------------------
-- Volume
-- ------------------------------------------------------------------------------------------

function M.CreateVolume()
    M.lastError = ""

    if not apiPresent() then
        M.lastError = "VNav natives not registered - is PathfindingUpdate.dll loaded?"
        return
    end

    local p = playerPos()
    if not p then
        M.lastError = "no player"
        return
    end

    -- The bake spans the whole map, so the volume is centred on the player only to bound the
    -- search space; it is not a streaming window.
    local lo = v4(p.x - M.halfExtent, p.y - M.halfExtent, p.z - M.halfExtent)
    local hi = v4(p.x + M.halfExtent, p.y + M.halfExtent, p.z + M.halfExtent)

    api("VNavDestroyVolume")(M.volumeName)

    M.volumeReady = api("VNavCreateVolume")(M.volumeName, M.voxelSize, M.agentRadius, M.margin, lo, hi, 0)
    if not M.volumeReady then
        M.lastError = "CreateVolume failed (bad config?)"
        return
    end

    -- Checked attach: refuses a bake stamped for a different game build, which is what turns
    -- "the geometry moved in a patch" from a silent wrong-path bug into a loud failure. With
    -- M.gameBuild == "" the check is skipped and this is a plain attach.
    M.bakeReady = api("VNavAttachBakeChecked")(M.volumeName, M.bakePath, M.gameBuild)
    if not M.bakeReady then
        M.lastError = "AttachBake refused - wrong path, voxelSize mismatch, or the bake is stamped for another game build"
    end
end

-- ------------------------------------------------------------------------------------------
-- Planning
-- ------------------------------------------------------------------------------------------

function M.Plan()
    if not M.volumeReady then
        M.lastError = "no volume"
        return
    end

    local from = playerPos()

    -- "Goal follows player" targets a point HOVERHEIGHT metres above the player, not the
    -- player's exact coordinate. Without this the goal is literally your feet, at ground level -
    -- the AV was never malfunctioning, it was correctly flying to where it was told to go. This
    -- is also the reason "agent radius" cannot fix this: that value is clearance against the
    -- BAKED STATIC WORLD only (buildings, terrain). The player is not an obstacle in this
    -- planner at all, just a target coordinate, so no radius setting produces standoff distance.
    local to = M.goal
    if M.goalIsPlayer then
        local pp = playerPos()
        -- H2: the hover offset is applied ONLY if the raised point is actually open.
        --
        -- Applying it unconditionally is fine outdoors and shoves the goal into the ceiling
        -- indoors, which manufactures an unreachable goal - a direct contributor to the AV
        -- stranding permanently in testing. The failure then looks like a pathfinding bug rather
        -- than a bad goal, which is what made it expensive to find.
        --
        -- Step down rather than giving up: half the height, then a quarter, then the player.
        if pp then
            local isFree = api("VNavIsFree")
            local h = M.hoverHeight
            local chosen = 0.0
            if isFree then
                for _, f in ipairs({1.0, 0.5, 0.25}) do
                    local cand = h * f
                    if isFree(M.volumeName, v4(pp.x, pp.y, pp.z + cand)) then
                        chosen = cand
                        break
                    end
                end
            else
                chosen = h -- no native available; behave as before
            end
            M.hoverApplied = chosen
            to = v4(pp.x, pp.y, pp.z + chosen)
        else
            to = nil
        end
        M.goal = to -- keep the UI's straight-line/detour stats meaningful during Chase mode too
    end

    if not from or not to then
        M.lastError = "no start or goal set"
        return
    end

    -- If an AV is flying, it plans from ITS position, not the player's.
    if M.avPos then
        from = M.avPos
    end

    if M.handle ~= 0 then
        api("VNavPathRelease")(M.handle)
        M.handle = 0
    end

    -- H4 bookkeeping: remember what this plan was made FOR, so staleness can be measured as
    -- distance rather than guessed at from elapsed time.
    M.planFromPos = v4(from.x, from.y, from.z)
    M.planGoalPos = v4(to.x, to.y, to.z)

    -- Sync blocks the game thread until the search finishes; async returns a handle immediately and
    -- the pump advances it. Both return a handle, so PollPath harvests them identically - a sync
    -- query is simply already terminal by the time it gets there.
    --
    -- Worth having both: `submit` below is submission cost for async and TOTAL cost for sync, so
    -- flipping this is how you see what a query actually costs rather than what it costs to ask.
    local t0 = os.clock()
    if M.planSync then
        local fSync = api("VNavFindPathSync")
        M.handle = fSync and fSync(M.volumeName, v4(from.x, from.y, from.z), v4(to.x, to.y, to.z)) or 0
    else
        M.handle = api("VNavFindPathAsync")(M.volumeName, v4(from.x, from.y, from.z), v4(to.x, to.y, to.z))
    end
    M.lastElapsed = (os.clock() - t0) * 1000.0

    if M.handle == 0 then
        M.lastError = "SubmitAsync returned 0"
    end
end

function M.PollPath()
    if M.handle == 0 then return end

    local status = api("VNavPathStatus")(M.handle)
    M.lastStatus = status

    -- 3 = Success, 4 = PartialSuccess. Anything >= 3 is terminal.
    if status < 3 then return end

    if status == 3 or status == 4 then
        local n = api("VNavPathPointCount")(M.handle)
        local pts = {}
        for i = 0, n - 1 do
            pts[#pts + 1] = api("VNavPathPoint")(M.handle, i)
        end
        M.lastPoints = pts
        M.lastLength = api("VNavPathLength")(M.handle)

        -- Per-waypoint clearance and yaw, harvested BEFORE the handle is released a few lines
        -- down - these read through the handle, so afterwards they return nothing.
        --
        -- Clearance SATURATES at the volume's tight threshold: it is a three-level signal
        -- (blocked / squeezing / open), not a rangefinder. A long run of identical values at the
        -- top of the range means "open", not "measured exactly this far from a wall".
        local clr, yaw = {}, {}
        local fClr, fYaw = api("VNavPathClearance"), api("VNavPathYaw")
        for i = 0, n - 1 do
            clr[#clr + 1] = fClr and fClr(M.handle, i) or 0.0
            yaw[#yaw + 1] = fYaw and fYaw(M.handle, i) or 0.0
        end
        M.lastClearances = clr
        M.lastYaws = yaw

        local fTurns = api("VNavPathInfeasibleTurns")
        M.lastInfeasibleTurns = fTurns and fTurns(M.handle) or 0

        -- Resume from where the AV actually is, NOT from waypoint 1 - see resumeIndex. If the AV
        -- has not spawned yet there is nothing to resume from, so start at the beginning.
        M.avIndex = M.avPos and resumeIndex(M.avPos, pts) or 1
    else
        M.lastPoints = {}
        M.lastLength = 0.0
    end

    api("VNavPathRelease")(M.handle)
    M.handle = 0
end

-- ------------------------------------------------------------------------------------------
-- Drawing
-- ------------------------------------------------------------------------------------------

-- Waypoint rings are drawn at the agent's radius on purpose: if a ring visibly intersects a wall,
-- the clearance threshold and the geometry disagree, and that is the bug worth catching.
--
-- Gating, all of it deliberate: every drawing is behind its own flag (all default OFF), and
-- nothing draws outside the CET overlay unless "draw while overlay closed" is ticked.
function M.Draw()
    if not M.overlayOpen and not M.drawWithOverlayClosed then return end

    local wantLines = M.drawPath and #M.lastPoints >= 2
    local wantRings = M.drawRadius and #M.lastPoints >= 1
    local wantGoal = M.drawGoal and M.goal
    if not (wantLines or wantRings or wantGoal) then return end

    local dd = debugDraw()
    if not dd then return end

    -- Debug Draw shapes last one frame, so this runs every frame (see the update hook) and
    -- stopping clears within 0.1 s. Only the REQUIRED parameters are passed: CET Lua cannot pass
    -- a script function's `opt` parameters (seconds/overlay/channel/thickness stay at defaults).
    -- Any error disables drawing for the session instead of throwing every frame.
    local ok, err = pcall(function()
        if wantLines then
            dd.Path(M.lastPoints, dd.Rgba(80, 255, 120))
        end

        if wantRings then
            local cyan = dd.Rgba(80, 200, 255)
            for i = 1, #M.lastPoints, math.max(1, M.drawEvery) do
                dd.Sphere(M.lastPoints[i], M.agentRadius, cyan)
            end
        end

        -- Goal marker has its own flag. Plan() keeps M.goal current for "goal follows player"
        -- too - seeing this marker floating above you rather than at your feet IS the
        -- confirmation that the hover-height fix is doing what it says.
        if wantGoal then
            dd.Sphere(M.goal, 2.0, dd.Rgba(255, 200, 60))
        end
    end)
    if not ok then
        ddState = false
        note("Debug Draw call failed, drawing disabled: " .. tostring(err))
    end
end

-- ------------------------------------------------------------------------------------------
-- AV
-- ------------------------------------------------------------------------------------------

-- Discovery rather than a hardcoded guess: vehicle record names differ between game versions and
-- installed mods, so the harness lists what this install actually has.
function M.FindAvRecords()
    M.avRecordCandidates = {}
    local ok, records = pcall(function() return TweakDB:GetRecords("gamedataVehicle_Record") end)
    if not ok or not records then
        M.lastError = "TweakDB:GetRecords failed - type name may differ on this version"
        return
    end
    for _, r in ipairs(records) do
        local ok2, id = pcall(function() return tostring(r:GetID():GetTDBID()) end)
        if not ok2 or type(id) ~= "string" then
            id = tostring(r)
        end
        if id:lower():find("av_", 1, true) then
            M.avRecordCandidates[#M.avRecordCandidates + 1] = id
        end
    end

    if #M.avRecordCandidates == 0 then
        M.lastError = string.format("no records matched 'av_' (scanned %d)", #records)
    end
end

function M.SpawnAv()
    local p = playerPos()
    if not p then return end

    local spec = DynamicEntitySpec.new()
    spec.recordID = TweakDBID.new(M.avRecord)
    spec.position = v4(p.x, p.y, p.z + 25.0)
    -- Identity built directly. EulerAngles:ToQuat() is not consistently available.
    spec.orientation = ToQuaternion{ i = 0, j = 0, k = 0, r = 1 }
    spec.alwaysSpawned = true
    spec.spawnInView = true

    local sys = Game.GetDynamicEntitySystem()
    if not sys then
        M.lastError = "DynamicEntitySystem unavailable - is Codeware installed?"
        return
    end

    M.avEntityId = sys:CreateEntity(spec)
    M.avEntity = nil        -- CreateEntity queues the spawn; the entity does not exist yet
    M.avPos = spec.position -- optimistic placeholder so Plan() has somewhere to path from
    M.avIndex = 1
end

function M.DespawnAv()
    if not M.avEntityId then return end
    local sys = Game.GetDynamicEntitySystem()
    if sys then sys:DeleteEntity(M.avEntityId) end
    M.avEntityId = nil
    M.avEntity = nil
    M.avPos = nil
end

-- Kinematic movement, deliberately. Vehicle AI would fight the path and make it impossible to
-- tell whether a bad route came from the planner or from the driver.
function M.StepAv(dt)
    if not M.avEntityId then return end

    -- CreateEntity is a request, not a synchronous spawn - the entity can take a frame or more
    -- to exist. Resolve the handle once it does, and snap avPos to where it actually landed
    -- rather than trust the spawn request, which the game may have adjusted (ground clamp,
    -- collision push-out). Without this, checking "fly path" immediately after "Spawn" would let
    -- avPos start walking the route before there was anything to move, and the AV would appear
    -- partway down the path in one jump once it finally spawned.
    if not M.avEntity then
        local ent = Game.FindEntityByID(M.avEntityId)
        if ent then
            M.avEntity = ent
            local ok, pos = pcall(function() return ent:GetWorldPosition() end)
            if ok and pos then
                M.avPos = pos
            end
        end
    end

    if not M.avFollow or not M.avPos or #M.lastPoints < 1 then return end

    local target = M.lastPoints[math.min(M.avIndex, #M.lastPoints)]
    local d = dist(M.avPos, target)

    if d < M.waypointTolerance and M.avIndex < #M.lastPoints then
        M.avIndex = M.avIndex + 1
        target = M.lastPoints[M.avIndex]
        d = dist(M.avPos, target)
    end

    if d > 0.01 then
        local step = math.min(M.avSpeed * dt, d)
        local nx = M.avPos.x + (target.x - M.avPos.x) / d * step
        local ny = M.avPos.y + (target.y - M.avPos.y) / d * step
        local nz = M.avPos.z + (target.z - M.avPos.z) / d * step
        M.avPos = v4(nx, ny, nz)
    end

    -- Measured AFTER the move, so it reflects where the AV actually is this frame.
    M.devNow = pathDeviation(M.avPos, M.lastPoints)
    if M.devNow > M.devMax then
        M.devMax = M.devNow
    end

    if M.avEntity then
        local yaw = 0.0
        if d > 0.01 then
            yaw = math.deg(atan2(target.y - M.avPos.y, target.x - M.avPos.x)) - 90.0
        end
        -- pcall: the handle goes stale if the entity streams out or despawns underneath us.
        -- Drop it on failure so the resolve step above tries again next frame instead of
        -- spamming a teleport call against nothing every frame forever.
        local ok = pcall(function()
            Game.GetTeleportationFacility():Teleport(M.avEntity, M.avPos, EulerAngles.new(0, 0, yaw))
        end)
        if not ok then
            M.avEntity = nil
        end
    end
end

-- ------------------------------------------------------------------------------------------
-- UI
-- ------------------------------------------------------------------------------------------

local function statLine(key)
    local f = api("VNavStat")
    return f and f(key) or 0
end

-- ------------------------------------------------------------------------------------------
-- Dynamic obstacles
--
-- The layer takes a whole set at once: Begin, then Add each box, then Commit publishes them as one
-- immutable snapshot. Boxes are staged in Lua first so the publish stays a single atomic swap - a
-- search that is already running keeps the snapshot it started with and cannot see a half-built set.
-- ------------------------------------------------------------------------------------------

function M.DynCommit()
    local fBegin, fAdd, fCommit = api("VNavDynamicBegin"), api("VNavDynamicAdd"), api("VNavDynamicCommit")
    if not (fBegin and fAdd and fCommit) then
        M.lastError = "dynamic obstacle natives unavailable"
        return
    end

    fBegin()
    for _, b in ipairs(M.dynBoxes) do
        fAdd(b.pos, b.half)
    end
    M.dynCommitted = fCommit(M.volumeName)
end

function M.DynAddAtPlayer()
    local p = playerPos()
    if not p then return end
    local h = M.dynHalf
    M.dynBoxes[#M.dynBoxes + 1] = { pos = v4(p.x, p.y, p.z), half = v4(h, h, h) }
end

-- A ring of boxes around the player, which is the quickest way to SEE the layer working: plan
-- through it and the path should bend around the ring rather than through it.
function M.DynRing(count)
    local p = playerPos()
    if not p then return end
    local h = M.dynHalf
    for i = 0, count - 1 do
        local a = (i / count) * math.pi * 2.0
        M.dynBoxes[#M.dynBoxes + 1] = {
            pos = v4(p.x + math.cos(a) * M.dynRingRadius, p.y + math.sin(a) * M.dynRingRadius, p.z),
            half = v4(h, h, h),
        }
    end
end

function M.DynClear()
    M.dynBoxes = {}
    M.DynCommit() -- an empty commit is how the layer is cleared; dropping the list alone leaves
                  -- the last snapshot live and blocking space nothing occupies
end

-- The panel is gated on overlay state explicitly: CET 1.38 calls onDraw every frame whether or
-- not its overlay is up, so without this gate the window is on screen during gameplay. (On older
-- CET builds onDraw only fired with the overlay visible, and this gate is simply always true.)
function M.DrawUI()
    if not M.overlayOpen or not M.open then return end

    ImGui.SetNextWindowSize(420, 640, ImGuiCond.FirstUseEver)

    -- Single-argument Begin on purpose: the two-argument form returns (visible, open) in some
    -- CET builds and just (visible) in others, and getting that wrong desynchronises Begin/End.
    -- ImGui requires End() on every path regardless of what Begin returned.
    local visible = ImGui.Begin("VNav Test")
    if not visible then ImGui.End() return end

    -- 1. Plugin -------------------------------------------------------------------------
    if ImGui.CollapsingHeader("1. Plugin", ImGuiTreeNodeFlags.DefaultOpen) then
        if not apiPresent() then
            ImGui.TextColored(1, 0.3, 0.3, 1, "natives NOT registered")
            ImGui.TextWrapped("PathfindingUpdate.dll did not load or did not register. Check red4ext logs.")
        else
            local ready = api("VNavIsReady")()
            ImGui.Text("natives registered")
            ImGui.Text(string.format("probe ready:     %s", tostring(ready)))
            ImGui.Text(string.format("native pump:     %d", statLine("nativePump")))
            ImGui.Text(string.format("QueryFilter path: %d", statLine("usingFilterPath")))
        end
    end

    -- 2. Volume -------------------------------------------------------------------------
    if ImGui.CollapsingHeader("2. Volume + bake", ImGuiTreeNodeFlags.DefaultOpen) then
        M.agentRadius = ImGui.SliderFloat("agent radius (m)", M.agentRadius, 0.25, 8.0)
        M.margin = ImGui.SliderFloat("margin (m)", M.margin, 0.0, 6.0)
        M.bakePath = ImGui.InputText("bake path", M.bakePath, 200)

        if ImGui.Button("Create volume + attach bake") then M.CreateVolume() end
        ImGui.SameLine()
        if ImGui.Button("Re-apply radius") then
            -- Cheap: the bake is agent-agnostic, so this re-derives clearance rather than
            -- re-sampling the world.
            M.CreateVolume()
            M.Plan()
        end

        -- Ask the plugin rather than trusting our own flags: these disagree exactly when something
        -- went wrong, which is the case worth seeing.
        local fHasVol, fHasBake = api("VNavHasVolume"), api("VNavHasBake")
        ImGui.Text(string.format("volume: %s (plugin says %s)   bake: %s (plugin says %s)",
            tostring(M.volumeReady), fHasVol and tostring(fHasVol(M.volumeName)) or "?",
            tostring(M.bakeReady),  fHasBake and tostring(fHasBake(M.volumeName)) or "?"))

        if ImGui.Button("Destroy volume") then
            local f = api("VNavDestroyVolume")
            if f then
                f(M.volumeName)
                M.volumeReady = false
                M.bakeReady = false
                M.lastPoints = {}
            end
        end
        ImGui.SameLine()
        if ImGui.Button("Attach bake (checked)") then
            -- Refuses a bake stamped for a different game build. The shipped nightcity.vnavbake is
            -- stamped `2.5` (vnav_bake --gamebuild 2.5), so with the default field below this
            -- refuses on a mismatch instead of planning against stale geometry.
            local f = api("VNavAttachBakeChecked")
            if f then
                M.bakeReady = f(M.volumeName, M.bakePath, M.gameBuild)
                M.lastError = M.bakeReady and "" or "checked attach refused the bake"
            end
        end
        M.gameBuild = ImGui.InputText("expected game build ('' = skip check)", M.gameBuild, 40)

        -- Invalidation: drop cached chunks so the next query re-samples. InvalidateBox is the
        -- targeted version (a door opened); InvalidateAll throws the whole cache away.
        if ImGui.Button("Invalidate box at player") then
            local f, p = api("VNavInvalidateBox"), playerPos()
            if f and p then
                local h = M.invalidateHalf
                f(M.volumeName, v4(p.x, p.y, p.z), v4(h, h, h))
            end
        end
        ImGui.SameLine()
        if ImGui.Button("Invalidate ALL") then
            local f = api("VNavInvalidateAll")
            if f then f(M.volumeName) end
        end
        M.invalidateHalf = ImGui.SliderFloat("invalidate half-extent (m)", M.invalidateHalf, 1.0, 64.0)

        -- Manual pump. The plugin pumps itself each frame; this is for watching a starved budget
        -- resolve, or forcing progress while paused.
        if ImGui.Button("Pump once") then
            local f = api("VNavPump")
            if f then f(M.pumpBudgetMicros) end
        end
        ImGui.SameLine()
        M.pumpBudgetMicros = ImGui.SliderInt("pump budget (us)", M.pumpBudgetMicros, 100, 20000)
        ImGui.Text(string.format("chunks from bake: %d", statLine("chunksFromBake")))
        ImGui.Text(string.format("chunks total:     %d", statLine("chunksSampled")))
        ImGui.Text(string.format("probes:           %d", statLine("probes")))
        ImGui.Text(string.format("resident:  %d chunks / %d KB",
            statLine("residentChunks"), statLine("residentKB")))
    end

    -- 3. Plan ---------------------------------------------------------------------------
    if ImGui.CollapsingHeader("3. Plan", ImGuiTreeNodeFlags.DefaultOpen) then
        if ImGui.Button("Set goal = here") then
            local p = playerPos()
            if p then M.goal = v4(p.x, p.y, p.z) end
            M.goalIsPlayer = false
        end
        ImGui.SameLine()
        M.goalIsPlayer = ImGui.Checkbox("goal follows player", M.goalIsPlayer)
        M.hoverHeight = ImGui.SliderFloat("hover height above player (m)", M.hoverHeight, 0.0, 60.0)
        ImGui.TextWrapped("This is what stops the AV landing on you - it targets a point in the "
            .. "air above you, not your exact position. 'agent radius' below cannot do this: it "
            .. "is clearance against the baked WORLD only, the player isn't an obstacle to it.")

        ImGui.Text(string.format("hover applied: %.1f m of %.1f m requested", M.hoverApplied, M.hoverHeight))
        if M.hoverApplied < M.hoverHeight - 0.01 then
            ImGui.TextColored(1, 0.8, 0.3, 1, "hover reduced: full height was inside geometry")
        end

        if ImGui.Button("Plan now") then M.Plan() end
        ImGui.SameLine()
        if ImGui.Button("Cancel in-flight") then
            local f = api("VNavCancelPath")
            if f and M.handle ~= 0 then
                f(M.handle)
                api("VNavPathRelease")(M.handle)
                M.handle = 0
                M.lastError = "cancelled"
            end
        end
        ImGui.SameLine()
        M.autoReplan = ImGui.Checkbox("auto re-plan", M.autoReplan)

        -- Sync blocks the game thread; async returns a handle and the pump advances it. Flipping
        -- this changes what "submit" below measures: submission cost vs the whole search.
        M.planSync = ImGui.Checkbox("plan SYNCHRONOUSLY (blocks the game thread)", M.planSync)

        -- H4: distance beats wall-clock. 483 paths were computed in one session and about one
        -- segment of each was flown; a path goes stale because things MOVED, not because time passed.
        M.replanOnDistance = ImGui.Checkbox("re-plan on distance travelled (not just time)", M.replanOnDistance)
        if M.replanOnDistance then
            M.replanDistance = ImGui.SliderFloat("agent moved (m)", M.replanDistance, 2.0, 100.0)
            M.replanGoalDistance = ImGui.SliderFloat("goal moved (m)", M.replanGoalDistance, 2.0, 100.0)
            ImGui.Text(string.format("backstop: %.1f s", M.replanInterval * 8.0))
        end
        M.replanInterval = ImGui.SliderFloat("interval (s)", M.replanInterval, 0.2, 5.0)

        -- H3: search tuning. The plugin has always exposed these; the panel did not, so the
        -- expansion ceiling could not be exercised live.
        ImGui.Separator()
        if ImGui.CollapsingHeader("Search tuning") then
            M.heuristicWeight = ImGui.SliderFloat("heuristic weight", M.heuristicWeight, 1.0, 3.0)
            M.tightnessPenalty = ImGui.SliderFloat("tightness penalty", M.tightnessPenalty, 1.0, 5.0)
            M.verticalCostMultiplier = ImGui.SliderFloat("vertical cost x", M.verticalCostMultiplier, 0.5, 4.0)
            M.snapRadius = ImGui.SliderFloat("snap radius (m)", M.snapRadius, 0.0, 16.0)
            M.maxExpansions = ImGui.SliderInt("max expansions", M.maxExpansions, 1000, 4000000)
            M.smoothing = ImGui.SliderInt("smoothing (0 raw 1 pull 2 spline 3 fillet)", M.smoothing, 0, 3)

            if ImGui.Button("Apply tuning") then
                local fn = api("VNavSetVolumeTuning")
                if fn then
                    local ok = fn(M.volumeName, M.heuristicWeight, M.tightnessPenalty,
                                  M.verticalCostMultiplier, M.snapRadius, M.maxExpansions, M.smoothing)
                    M.lastError = ok and "" or "VNavSetVolumeTuning rejected these values"
                else
                    M.lastError = "VNavSetVolumeTuning unavailable"
                end
            end

            ImGui.Separator()
            M.coarseGuidance = ImGui.Checkbox("hierarchical guidance (needs a bake)", M.coarseGuidance)
            M.coarsePenalty = ImGui.SliderFloat("off-corridor penalty", M.coarsePenalty, 1.0, 16.0)
            ImGui.SameLine()
            if ImGui.Button("Apply guidance") then
                local fn = api("VNavSetCoarseGuidance")
                if fn then
                    fn(M.volumeName, M.coarseGuidance, M.coarsePenalty)
                else
                    M.lastError = "VNavSetCoarseGuidance unavailable"
                end
            end
        end

        -- H1: stranding recovery.
        ImGui.Separator()
        M.autoEscape = ImGui.Checkbox("auto-escape when stranded", M.autoEscape)
        M.strandSeconds = ImGui.SliderFloat("stranded after (s)", M.strandSeconds, 1.0, 20.0)
        ImGui.Text(string.format("strand events: %d", M.strandCount))
        if M.escapeInfo ~= "" then
            ImGui.TextColored(1, 0.8, 0.3, 1, M.escapeInfo)
        end

        -- Agent shape / 4D --------------------------------------------------------------
        ImGui.Separator()
        if ImGui.CollapsingHeader("Agent shape + 4D yaw search") then
            ImGui.TextWrapped("agentLength turns the agent from a sphere into a capsule, so the "
                .. "planner starts checking whether the BODY fits and whether it can TURN. "
                .. "0 disables it. The 4D search is meaningless for a sphere and silently falls "
                .. "back to 3D, so raise agentLength before expecting yaw to do anything.")
            M.agentLength = ImGui.SliderFloat("agent length (m)", M.agentLength, 0.0, 20.0)
            M.turnSamples = ImGui.SliderInt("turn samples", M.turnSamples, 2, 16)
            if ImGui.Button("Apply agent length") then
                local f = api("VNavSetAgentLength")
                if f then
                    M.lastError = f(M.volumeName, M.agentLength, M.turnSamples) and ""
                        or "VNavSetAgentLength rejected these values"
                end
            end

            ImGui.Separator()
            M.yawBuckets = ImGui.SliderInt("yaw buckets (0 = 3D, >=4 enables)", M.yawBuckets, 0, 32)
            M.yawCost = ImGui.SliderFloat("yaw turn cost", M.yawCost, 0.0, 5.0)
            M.yawFollowsTravel = ImGui.Checkbox("yaw follows travel", M.yawFollowsTravel)
            M.yawTravelCone = ImGui.SliderFloat("travel cone (deg)", M.yawTravelCone, 5.0, 180.0)
            if ImGui.Button("Apply yaw search") then
                local f = api("VNavSetYawSearch")
                if f then
                    M.lastError = f(M.volumeName, M.yawBuckets, M.yawCost,
                                    M.yawFollowsTravel, M.yawTravelCone) and ""
                        or "VNavSetYawSearch rejected these values"
                end
            end
            ImGui.TextWrapped("Cost: the state space is multiplied by yawBuckets, so 16 buckets "
                .. "is a 16x bigger search. Watch expansions and submit time when raising it.")

            ImGui.Separator()
            M.bidirectional = ImGui.Checkbox("bidirectional (meet in the middle)", M.bidirectional)
            ImGui.SameLine()
            if ImGui.Button("Apply##bidi") then
                local f = api("VNavSetBidirectional")
                if f then f(M.volumeName, M.bidirectional) end
            end

            M.maxParks = ImGui.SliderInt("max parks", M.maxParks, 16, 4096)
            M.noProgressParks = ImGui.SliderInt("no-progress parks", M.noProgressParks, 4, 512)
            M.searchTimeout = ImGui.SliderFloat("timeout (s)", M.searchTimeout, 0.5, 30.0)
            if ImGui.Button("Apply persistence") then
                local f = api("VNavSetSearchPersistence")
                if f then
                    M.lastError = f(M.volumeName, M.maxParks, M.noProgressParks, M.searchTimeout) and ""
                        or "VNavSetSearchPersistence rejected these values"
                end
            end
            ImGui.TextWrapped("A park is the search suspending to wait for chunks. These bound how "
                .. "long an async query keeps trying before giving up with a partial result.")
        end

        -- Dynamic obstacles -------------------------------------------------------------
        ImGui.Separator()
        if ImGui.CollapsingHeader("Dynamic obstacles") then
            ImGui.TextWrapped("Boxes are staged here, then Commit publishes them as ONE immutable "
                .. "snapshot. A search already running keeps the snapshot it started with, so it "
                .. "can never see a half-built set.")

            M.dynHalf = ImGui.SliderFloat("box half-extent (m)", M.dynHalf, 0.5, 20.0)
            M.dynRingRadius = ImGui.SliderFloat("ring radius (m)", M.dynRingRadius, 5.0, 80.0)

            if ImGui.Button("Add box at player") then M.DynAddAtPlayer() end
            ImGui.SameLine()
            if ImGui.Button("Ring of 8") then M.DynRing(8) end
            ImGui.SameLine()
            if ImGui.Button("Clear") then M.DynClear() end

            if ImGui.Button("Commit to volume") then M.DynCommit() end
            ImGui.SameLine()
            M.dynAutoCommit = ImGui.Checkbox("re-commit every frame", M.dynAutoCommit)

            ImGui.Text(string.format("staged: %d    committed: %d", #M.dynBoxes, M.dynCommitted))
            ImGui.TextWrapped("To SEE it work: stand clear, 'Ring of 8', Commit, then plan across "
                .. "the ring. The path should bend around the boxes rather than through them.")
        end

        ImGui.Separator()
        ImGui.Text(string.format("status:   %s", STATUS[M.lastStatus] or tostring(M.lastStatus)))
        ImGui.Text(string.format("points:   %d", #M.lastPoints))
        ImGui.Text(string.format("length:   %.1f m", M.lastLength))
        ImGui.Text(string.format("submit:   %.2f ms%s", M.lastElapsed,
            M.planSync and "  (SYNC - whole search)" or "  (async - submission only)"))

        -- Infeasible turns: corners an elongated agent cannot physically make. Non-zero means the
        -- path is geometrically valid but not flyable by a body this long - raise turn samples or
        -- shorten the agent.
        if M.lastInfeasibleTurns > 0 then
            ImGui.TextColored(1, 0.5, 0.3, 1,
                string.format("infeasible turns: %d", M.lastInfeasibleTurns))
        else
            ImGui.Text("infeasible turns: 0")
        end

        -- Clearance / yaw along the path. Clearance SATURATES at the volume's tight threshold, so
        -- read it as blocked / squeezing / open rather than as a distance.
        if #M.lastClearances > 0 then
            local lo, hi = 1e9, -1e9
            for _, c in ipairs(M.lastClearances) do
                if c < lo then lo = c end
                if c > hi then hi = c end
            end
            ImGui.Text(string.format("clearance: min %.2f  max %.2f m (saturates)", lo, hi))
            if lo <= 0.01 then
                ImGui.TextColored(1, 0.5, 0.3, 1, "  a waypoint reports ZERO clearance")
            end
        end
        if M.yawBuckets >= 4 and #M.lastYaws > 0 then
            ImGui.Text(string.format("yaw[0]: %.1f deg   yaw[last]: %.1f deg",
                M.lastYaws[1], M.lastYaws[#M.lastYaws]))
        end

        local p = playerPos()
        if p and M.goal then
            ImGui.Text(string.format("straight: %.1f m", dist(p, M.goal)))
        end
        if M.lastLength > 0 and p and M.goal then
            local straight = dist(p, M.goal)
            if straight > 1.0 then
                ImGui.Text(string.format("detour:   %.2fx", M.lastLength / straight))
            end
        end
    end

    -- 4. Draw ---------------------------------------------------------------------------
    if ImGui.CollapsingHeader("4. Draw", ImGuiTreeNodeFlags.DefaultOpen) then
        M.drawPath = ImGui.Checkbox("draw path", M.drawPath)
        M.drawRadius = ImGui.Checkbox("draw agent radius rings", M.drawRadius)
        M.drawGoal = ImGui.Checkbox("draw goal marker", M.drawGoal)
        M.drawWithOverlayClosed = ImGui.Checkbox("draw while overlay closed", M.drawWithOverlayClosed)
        M.drawEvery = ImGui.SliderInt("ring every N points", M.drawEvery, 1, 20)
        ImGui.TextWrapped("A ring that clips a wall means clearance and geometry disagree. Drawing never happens unless ticked here, and never outside the overlay unless the last box is on.")
    end

    -- 5. AV -----------------------------------------------------------------------------
    if ImGui.CollapsingHeader("5. AV") then
        M.avRecord = ImGui.InputText("record", M.avRecord, 120)
        if ImGui.Button("Find AV records") then M.FindAvRecords() end
        for _, id in ipairs(M.avRecordCandidates) do
            if ImGui.Selectable(id) then M.avRecord = id end
        end

        if ImGui.Button("Spawn") then M.SpawnAv() end
        ImGui.SameLine()
        if ImGui.Button("Despawn") then M.DespawnAv() end
        ImGui.SameLine()
        M.avFollow = ImGui.Checkbox("fly path", M.avFollow)

        M.avSpeed = ImGui.SliderFloat("speed (m/s)", M.avSpeed, 2.0, 60.0)
        M.waypointTolerance = ImGui.SliderFloat("waypoint tolerance (m)", M.waypointTolerance, 0.1, 5.0)
        ImGui.Text(string.format("waypoint %d / %d", M.avIndex, #M.lastPoints))
        if M.avPos then
            ImGui.Text(string.format("av: %.1f %.1f %.1f", M.avPos.x, M.avPos.y, M.avPos.z))
        end

        ImGui.Separator()
        ImGui.Text(string.format("path deviation:  now %.2f m   max %.2f m", M.devNow, M.devMax))
        if ImGui.Button("Reset max deviation") then
            M.devMax = 0.0
        end
        ImGui.TextWrapped("Deviation is how far the AV strayed from the corridor the planner "
            .. "actually validated. Turn AUTO RE-PLAN OFF before reading it: re-planning starts "
            .. "the new path at the AV's current position, so it resets deviation to ~0 every "
            .. "interval and hides exactly what you are trying to measure.")
        ImGui.TextWrapped("If max deviation tracks the tolerance slider, corner-cutting is your "
            .. "clipping. If it stays near 0 and it still clips, the agent-shape model is.")

        ImGui.Separator()
        ImGui.Text("Recorder")
        M.logName = ImGui.InputText("csv name", M.logName, 120)

        if not M.logEnabled then
            if ImGui.Button("Start recording") then
                M.StartLog()
                M.logEnabled = M.logFile ~= nil
            end
        else
            if ImGui.Button("Stop recording") then
                M.logEnabled = false
                M.StopLog()
            end
        end

        ImGui.SameLine()
        -- The reason this button exists: it puts YOUR eyes on the same timeline as the numbers.
        -- Press it the moment you see a clip and the row is tagged, so the deviation and
        -- position at that instant can be picked out of thousands of ordinary samples.
        if ImGui.Button("Mark clip") then
            if M.logFile then
                M.LogRow("clip")
            else
                M.lastError = "not recording - press Start recording first"
            end
        end

        ImGui.Text(string.format("%s  (%d rows)", M.logStatus, M.logRows))
        ImGui.TextWrapped("Every row carries the full config, so one file can hold the whole "
            .. "sweep - change tolerance/speed/radius mid-run without starting a new file.")

        ImGui.Separator()
        ImGui.TextWrapped("One-button loop: goal = player, replan on a timer, AV flies it.")
        if ImGui.Button("Chase player") then
            M.goalIsPlayer = true
            M.autoReplan = true
            M.avFollow = true
            if M.handle == 0 then M.Plan() end
        end
        ImGui.SameLine()
        if ImGui.Button("Stop") then
            M.avFollow = false
            M.autoReplan = false
        end
    end

    if M.lastError ~= "" then
        ImGui.Separator()
        ImGui.TextColored(1, 0.4, 0.4, 1, M.lastError)
    end

    ImGui.End()
end

-- ------------------------------------------------------------------------------------------

registerForEvent("onInit", function()
    note("loaded")
end)

-- H1: stranding detection and recovery.
--
-- "No usable path AND no movement" is a state replanning cannot fix, because replanning is exactly
-- what already failed - so left alone the agent sits there forever. Measured in testing as 0 m of
-- movement over 32 s, terminal. The recovery is to stop asking for the goal and just ask for
-- somewhere to BE, which is what VNavFindEscape answers.
function M.TickStranding(dt)
    if not M.avFollow or not M.volumeReady then
        M.strandTimer = 0.0
        return
    end

    local pos = M.avPos
    if not pos then
        M.strandTimer = 0.0
        return
    end

    -- Moving at all counts as not stranded, regardless of whether a path exists.
    local moved = 1e9
    if M.strandPos then
        moved = dist(pos, M.strandPos)
    end
    M.strandPos = pos

    local havePath = (#M.lastPoints >= 2) and (M.lastStatus == 3 or M.lastStatus == 4)
    if moved > (M.avSpeed * dt * 0.25) or havePath then
        M.strandTimer = 0.0
        return
    end

    M.strandTimer = M.strandTimer + dt
    if M.strandTimer < M.strandSeconds then
        return
    end
    M.strandTimer = 0.0
    M.strandCount = M.strandCount + 1

    if not M.autoEscape then
        M.escapeInfo = "stranded (auto-escape off)"
        return
    end

    local findEscape = api("VNavFindEscape")
    if not findEscape then
        M.escapeInfo = "stranded, but VNavFindEscape is unavailable"
        return
    end

    -- minDistance is the load-bearing argument. Without it the nearest qualifying position is
    -- usually where the agent already is - a correct answer that changes nothing.
    M.escapeStatus = findEscape(M.volumeName, v4(pos.x, pos.y, pos.z), 30.0, 0.0, 8.0)
    if M.escapeStatus == 3 then
        local p = api("VNavEscapePosition")()
        local d = api("VNavEscapeDistance")()
        -- Fly the escape point directly as a one-waypoint path, rather than planning to it: the
        -- planner is what just failed, so routing through it again would fail the same way.
        M.lastPoints = { v4(pos.x, pos.y, pos.z), p }
        M.avIndex = 2
        M.lastStatus = 3
        M.escapeInfo = string.format("escaped %.1f m (x%d)", d, M.strandCount)
    else
        M.escapeInfo = string.format("escape failed: status %d (x%d)", M.escapeStatus, M.strandCount)
    end
end

-- H4: should we replan yet? Distance-based, with the timer kept as a backstop.
function M.ShouldReplan(dt)
    M.replanTimer = M.replanTimer + dt

    if not M.replanOnDistance then
        return M.replanTimer >= M.replanInterval
    end

    -- A path stops being useful when the agent has moved away from it or the goal has moved away
    -- from its end - not when a timer expires. Standing still with a good path should replan
    -- rarely; sprinting away from one should replan immediately.
    if M.planFromPos and M.avPos and dist(M.avPos, M.planFromPos) >= M.replanDistance then
        return true
    end
    if M.planGoalPos and M.goal and dist(M.goal, M.planGoalPos) >= M.replanGoalDistance then
        return true
    end

    -- Backstop, deliberately much longer than the plain interval: catches "nothing moved but the
    -- world did" without reinstating the every-second churn this replaces.
    return M.replanTimer >= (M.replanInterval * 8.0)
end

registerForEvent("onUpdate", function(dt)
    if not apiPresent() then return end

    M.PollPath()

    if M.autoReplan and M.volumeReady then
        if M.ShouldReplan(dt) then
            M.replanTimer = 0.0
            if M.handle == 0 then M.Plan() end
        end
    end

    -- Re-publishing every frame is the stress test for the snapshot swap: a search running
    -- concurrently must keep the set it started with rather than tearing.
    if M.dynAutoCommit and M.volumeReady then
        M.DynCommit()
    end

    M.StepAv(dt)
    M.TickStranding(dt)
    M.TickLog(dt)
    M.Draw()
end)

-- Without this the file is left open when the game exits. Writes are flushed per row so the
-- data survives regardless, but closing here keeps the handle from outliving the session.
registerForEvent("onShutdown", function()
    M.logEnabled = false
    M.StopLog()
end)

registerForEvent("onDraw", function()
    M.DrawUI()
end)

registerForEvent("onOverlayOpen", function()
    M.overlayOpen = true
    M.open = true
end)

registerForEvent("onOverlayClose", function()
    M.overlayOpen = false
end)

return M
