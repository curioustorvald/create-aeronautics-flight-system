-- Aer Togekiss FMS -- CONFIGURATION
--
-- Everything the flight-management computer (#8 autopilot.lua) can be RETUNED or REWIRED
-- with: rednet protocol names, the database filenames, sign/frame conventions, cruise
-- speeds, the path planner's physics, the landing profiles and the instrument scales.
-- The program itself holds only logic. Same split, and for the same reasons, as the
-- attitude controller's pid/uacconf.lua:
--
--   * a program update never overwrites this airframe's calibration, and
--   * the knobs are editable without scrolling a 4000-line program.
--
-- THERE ARE DELIBERATELY NO FALLBACK COPIES OF THESE VALUES IN autopilot.lua. A second set
-- of defaults would let an edit here silently do nothing, which is exactly what makes an
-- empirically-calibrated sign untrustworthy. A missing or truncated file is a hard BOOT
-- error naming the missing knob (see the canary list in autopilot.lua's loader), not a nil
-- arithmetic crash halfway through a flight.
--
-- The comments ARE the configuration -- most of these numbers were arrived at in the air.
-- Keep them with the value they explain.
--
-- Loaded by `require("apconf")`, which resolves because startup.lua runs
-- shell.run("autopilot.lua") and so the program directory is this one. Must end with
-- `return CFG`.

local CFG = {}

-- ============================================================================
-- LINKS -- who this computer talks to. Every one of these must match the name at the
-- other end; they are the first thing to check when a channel goes quiet.
-- ============================================================================

CFG.APNAV_PROTOCOL = "aertogekiss_navap" -- must match navcom.lua's APNAV_PROTOCOL
-- Vertical channel to the Stabiliser+Altitude control during an automatic landing.
-- Must match control.lua's AUTOLAND_PROTOCOL.
CFG.AUTOLAND_PROTOCOL = "aertogekiss_autoland"
-- AS -> FMS: the altitude selector pings this when the pilot taps a panel while it is
-- overshadowed by a landing, asking for ALT-AP back (see overrideLoop / LND.paused).
CFG.ALTOVERRIDE_PROTOCOL = "aertogekiss_altoverride"

CFG.POINTS_FILE = "points.db"
CFG.COURSE_FILE = "course.db"
CFG.LANDING_FILE = "landing.db"

-- Gear squat switch (weight on wheels), broadcast by #15 manconrelay_rotation.lua. Ground
-- TRUTH for a touchdown, where AGL is only an inference -- see the TD table in autopilot.lua
-- for the two guards (freshness, and a hold time) that stop a stale or bouncing contact
-- cutting a descent at altitude.
CFG.TOUCHDOWN = {
    PROTOCOL = "aertogekiss_touchdown_detection", -- must match manconrelay_rotation.lua
    HOLD     = 0.25,   -- s the signal must stay on before it counts as down
    STALE    = 3.0,    -- s of silence before the sensor is treated as offline
}

-- Navigation-database updates from the #24 nav data server. The server is master and the
-- flow is one-way; see the NDU block in autopilot.lua for what is and is not safe to swap
-- under a running flight.
CFG.NAVDATA = {
    PROTOCOL    = "aer_navdata", -- must match nav_update_server.lua's NAVDATA_PROTOCOL
    PULL_EVERY  = 5,   -- s between "send me the set" calls, while we have never synced
    STALE_AFTER = 95,  -- s of silence before the server is called silent (> 3 broadcasts)
    TMP         = ".new",     -- suffix of the staging file each .db is written through
}

-- The AP-NAV panel switch wired to THIS computer, re-announced continuously so the UAC
-- latches it (a dropped packet must not disengage the autopilot).
CFG.APSWITCH = {
    PROTOCOL = "aertogekiss_apswitch_1",
    SIDE     = "front",   -- redstone input side the switch is wired to
    INTERVAL = 0.1,       -- s between announcements
}


-- ============================================================================
-- RATES
-- ============================================================================
CFG.PING_INTERVAL = 0.2  -- s; how often the current target is re-broadcast. Matches TICK_INTERVAL
                           -- because cruiseCap now carries a LIVE commanded speed (with the nose-
                           -- alignment / cross-track gates folded in), not just a slow keep-alive --
                           -- at 1 s the craft would accelerate for a whole second before a gate bit.
CFG.TICK_INTERVAL = 0.05  -- s; how often position vs target is checked

-- ============================================================================
-- FRAME / SIGNS
-- ============================================================================
-- Heading measurement -- MUST match navcom.lua's HEADING_SIGN / HEADING_OFFSET,
-- since this is the same aircraft and the same sublevel orientation. If navcom's
-- convention is ever recalibrated, change it here too: NAVCOM would still fly the
-- craft to the right heading (it owns those constants), but this alignment gate
-- would open at the wrong moment.
CFG.HEADING_SIGN   = 1
CFG.HEADING_OFFSET = math.pi

-- THE ALTITUDE DATUM -- how far the pose `sublevel` reports sits ABOVE THE BOTTOM OF THE
-- HULL, in metres. MUST MATCH this airframe's pid/uacconf.lua, gpwsconf.lua, atcconf.lua and
-- responder.lua: it is the same aeroplane and the same hull, and two computers bolted to it
-- that disagree about where its feet are will disagree about whether it has landed.
--
-- autopilot.lua subtracts it the moment the pose is read, so NAV.posY -- and therefore
-- LND.agl, the glideslope deviation, the terrain shading on the HSI and the position this
-- computer reports to the terrain server -- is the world Y of the part of the aeroplane that
-- touches the ground. That is the only reading that can be compared with landing.db, whose
-- altitudes are the SURVEYED SURFACE Y of a pad or a threshold (TMV/TMA are 64, i.e. standing
-- on the block at Y=63). Uncorrected, the descent was flown to a pad elevation this many
-- metres underground, and only the squat switch ended it.
--
-- HOW TO MEASURE IT: set it to 0, park on ground of a known Y, read the altitude off the PFD,
-- and take (what it read) - (the Y the hull rests on). It is a property of the BUILD.
CFG.ALT_OFFSET = 9
-- Measured 2026-08-29: parked on the Y=63 airstrip, Aer Togekiss reported alt 73.

-- ============================================================================
-- WAYPOINT PASS RADII
-- ============================================================================
-- Pass radii: how close counts as "at" a waypoint before we advance to the
-- next one. Contextual per navigation-planning convention: VORs are flown
-- past at a distance (fly-by fix), aerodromes/POIs mid-route get a moderate
-- radius, and the very last point of a plan is drawn in tight since that is
-- an actual arrival.
-- ARRIVED_FRACTION: inside this fraction of the final point's own pass radius, the phase sent
-- to navcom becomes ARRIVED rather than TERMINAL. Scales with the same contextual radius as
-- everything else here, so a VOR final gets a looser "arrived" than a POI final does.
-- (Tabled, not one local each -- see the 200-locals note at the top of the tuning section.)
CFG.RAD = {
    VOR_RADIUS         = 240,
    POI_ENROUTE_RADIUS = 120,
    POI_FINAL_RADIUS   = 60,
    ARRIVED_FRACTION   = 0.25,
}

-- CRUISE SPEED ---------------------------------------------------------------
-- Pilot-selectable cruise-speed cap, sent to NAVCOM as msg.cruiseCap and applied only
-- by its ENROUTE law (so the status-screen button "only works in cruising mode"). Three
-- presets; COAST is a plain 0.
CFG.SPEED_FULL = 320   -- m/s "FULL" cruise (kept below navcom's CRUISE_SPEED ceiling) Choose fastest speed that does not outrun chunk loading
CFG.SPEED_SLOW = 50   -- m/s "SLOW"; mirror of navcom.lua's APPROACH_SPEED
-- Cycle order when the button is tapped.
CFG.NEXT_CRUISE = { full = "slow", slow = "coast", coast = "full" }

-- DYNAMIC CRUISE (pre-planned clothoid path + baked speed profile) -----------
-- The en-route path is now planned ONCE, when the pilot confirms a plan (setDirectTo /
-- setRoute / RTR.setSolvedRoute call planPath()): each interior waypoint gets an explicit
-- LineSegment -> ClothoidSegment -> CircularArc -> ClothoidSegment -> LineSegment fly-by
-- transition sized from a real turn radius R = V^2/(g*tan(bank)), and the whole route is
-- walked into a dense arc-length-sampled polyline (PLAN.samples) with the speed profile
-- baked in. Runtime (dynamicSpeedTarget/broadcastCurrent/drawCourseHSI) just looks values
-- up on that polyline -- no per-tick curve math. See the PLAN table below planPath().
--
-- This function is still the SINGLE source of commanded speed: the destination-arrival
-- brake and cross-track gate are applied HERE, not in the UAC, so the CMD on the HSI is
-- exactly what the aircraft is flown to. The UAC just tracks it as cruiseCap (schedSpeed=true).
-- ROUTE START: point the nose at the first target BEFORE applying any throttle. While the
-- heading error to the target exceeds LG.ALIGN_HDG_DEG the commanded speed is 0, so the craft
-- simply yaws onto the bearing instead of accelerating off on the old heading and arcing round.
-- Cleared for good once aligned (per plan, not per leg).
--
-- ADVANCE_*_MULT -- turn-aware pass radius, used only as the FALLBACK advance test when
-- DC.CURVE is off (PLAN.passed, from the baked transition midpoints, is the primary test
-- while curve-following). 1.0 / 1.0 disables (plain radiusFor).
--
-- NOTE (200-locals): CC's Lua VM caps a function at 200 locals and counts a nested function's
-- own locals ON TOP of the enclosing scope's, so late/large functions (drawLandingHSI etc.) blow
-- up once the file has too many TOP-LEVEL locals. Keep tuning knobs in tables like this one --
-- add new ones as DC.X, never `local X`. (luac -p does NOT model this; it will pass regardless.)
CFG.DC = {
    DYNAMIC_CRUISE = true,         -- false -> fall back to the flat pilot cruise cap
    CURVE          = true,         -- en-route: steer to a carrot on the planned clothoid path
                                   -- (smooth continuous bearing, jerk-controlled rounded corners)
                                   -- instead of straight at the fix. false -> aim at the waypoint.
    CURVE_LOOKAHEAD = 200,         -- m: how far ahead on the path the carrot sits. Larger = smoother /
                                   -- cuts corners more; smaller = tracks the waypoints tighter.
                                   -- NOW A FLOOR ONLY -- the working value is the TIME below.
    -- ---------------------------------------------------------------------------------------
    -- CARROT LOOKAHEAD AS A TIME (2026-08-20). This is the fix for "the yaw sways in straight
    -- flight", and it is a GUIDANCE fault, not an attitude one -- #1's yaw cascade was retuned
    -- first and simulates clean (no ring at all); the nose was swaying because the POINT IT IS
    -- AIMED AT was swinging.
    --
    -- WHY A FIXED DISTANCE IS THE BUG. Pure pursuit's outer loop gain is V/L: at a fixed carrot
    -- distance the guidance gain rises in direct proportion to speed. 200 m is 2.5 s of
    -- lookahead for #16 Citation at 80 m/s (gain 0.40 /s) and 1.67 s for #39 Mantine at 120
    -- (0.60 /s) -- both fine -- but only 0.62 s for THIS aeroplane at 320 (gain 1.60 /s).
    -- Against the UAC's lateral velocity loop, whose gain is STRAFE_VEL_KP_NORM/tau ~ 1.18 /s,
    -- that puts the OUTER loop faster than the INNER one it commands, which is the same fault
    -- the strafe cascade itself had and cannot ever be stable. 0.62 s of lookahead is also
    -- barely more than the ~0.5 s transport delay measured on this rig (see CFG.ATT_KP_YAW in
    -- 1/pid/uacconf.lua) -- the craft is chasing a point it reaches before it can respond to it.
    --
    -- Simulated, 320 m/s cruise, 20 m lateral upset, with the 0.5 s delay in the loop:
    --
    --     L  200 m (0.62 s)   weave 3.3 s, zeta 0.01, +-5.7 deg of nose sway   <- the report
    --     L  400 m (1.25 s)   weave 3.7 s, marginal,  +-2.9 deg
    --     L  800 m (2.50 s)   no weave,               +-1.4 deg
    --     L 1100 m (3.44 s)   no weave,               +-1.0 deg                <- chosen
    --     L 1600 m (5.00 s)   no weave,               +-0.7 deg
    --
    -- Note the weave got SLOWER, not better, when the UAC's inner loops were fixed (2.8 s ->
    -- 3.3 s): damping the inner loop and giving it a feedforward makes it TRACK the guidance
    -- demand faithfully, which exposes an outer loop that sloppy inner tracking had been
    -- masking. "Longer period after the inner fix" is the signature of an outer-loop problem.
    --
    -- 3.4 s keeps the outer:inner ratio at 0.25 -- the same 4:1 separation the strafe cascade
    -- is tuned to. CORNER CUTTING is the cost: on the 3000 m radius this craft's corners are
    -- capped at (DC.MAX_RADIUS), a 1100 m carrot cuts ~50 m, comfortably inside PASS_CORRIDOR
    -- 400. The floor above keeps slow flight exactly as it was: below 59 m/s the 200 m distance
    -- still wins, so approach and the pad phases are untouched.
    -- Raise the TIME if the track still weaves; lower it if corners are cut too wide. On a
    -- slower airframe this is a no-op until its cruise exceeds CURVE_LOOKAHEAD/this.
    CURVE_LOOKAHEAD_T = 3.4,       -- s of flight ahead; actual carrot = max(CURVE_LOOKAHEAD, this*V)
    GRAVITY        = 11.0,         -- m/s^2: Minecraft's gravity, used in the turn-radius formula.
    MAX_BANK_DEG   = 45,           -- deg: the bank angle the turn-radius formula assumes --
                                   -- R = V^2/(GRAVITY*tan(MAX_BANK_DEG)). One physical knob that
                                   -- means the same thing whether the UAC is presently flying
                                   -- VTOL/HYBRID/FIXEDWING, so corner geometry "just works" across
                                   -- all three without the FMS needing to know the flight mode.
                                   -- Retune in-game like every other knob here; the old flat
                                   -- A_LAT_MAX=20 this replaces implied a much steeper ~61 deg.
    JERK_MAX       = 8,            -- m/s^3: caps the rate lateral accel may ramp through a
                                   -- clothoid -- this is what makes the turn jerk-controlled.
                                   -- Clothoid length Ls = V^3/(R*JERK_MAX). Lower = longer, gentler
                                   -- spiral entry/exit; higher = shorter spiral, closer to a plain
                                   -- circular fillet.
    MIN_RADIUS     = 40,           -- m: floor on a corner's turn radius.
    MAX_RADIUS     = 3000,         -- m: ceiling on a corner's turn radius (sanity cap for a very
                                   -- gentle turn at high speed).
    LEG_FIT_FRACTION = 0.45,       -- max fraction of EITHER adjacent leg's length a corner's
                                   -- tangent length may consume. Kept well under 0.5 so two
                                   -- corners sharing one leg (fed from each end) can never overlap
                                   -- even if both want their full share.
    PASS_CORRIDOR  = 400,          -- m: how far off the planned path the aircraft may be and
                                   -- still have its position along that path believed. PLAN.passed
                                   -- advances NAV.index by projecting the aircraft onto the baked
                                   -- polyline; out beyond this the nearest sample is an arbitrary
                                   -- point (a plan whose first leg has not been joined yet, a
                                   -- diversion, a teleport) and could claim several waypoints at
                                   -- once, so the projection is ignored and only the pass-radius
                                   -- test advances. Comfortably wider than any legitimate
                                   -- cross-track (see XTK_SLOW_HI) and than the corner geometry.
    SAMPLE_SPACING = 5,            -- m: density of the baked polyline. Smaller = smoother carrot/
                                   -- speed/course lookups and more precise geometry, at the cost of
                                   -- more samples (memory) for a long route.
    PLAN_MAX_ITERS = 6,            -- global speed<->geometry passes before planPath() gives up and
                                   -- uses whatever it has (see the iterative solver in planPath).
    PLAN_CONV_EPS  = 2,            -- m: a corner's radius changing less than this between passes
                                   -- counts as converged -- stops iterating early on easy routes.
    BRAKE_DECEL    = 12,            -- m/s^2: deceleration the CORNER speed profile plans with (how
                                   -- early/hard it brakes INTO a turn). Braking distance to a slow
                                   -- point = (v^2 - v_target^2)/(2*DECEL): at 3 it was ~2000 m to a
                                   -- stop from 110 (half of a 4000 m leg spent slowing!); 8 -> ~750 m.
                                   -- The profile ALREADY cruises the middle of a long leg -- this only
                                   -- sets how LATE/short the braking into a turn is. Set it to what the
                                   -- craft can REALLY brake at: too high and it overshoots the turn;
                                   -- too low and it crawls in early. (See ARRIVE_DECEL for the
                                   -- destination stop specifically -- it is NOT governed by this.)
    ARRIVE_DECEL   = 60,           -- m/s^2: deceleration for the FINAL destination stop specifically --
                                   -- deliberately STEEPER than BRAKE_DECEL. A gentle corner-speed
                                   -- profile can legitimately let the craft carry full cruise almost to
                                   -- the last waypoint (that's the point of curvature-based cornering),
                                   -- but that means the LAST leg alone has to shed all of it; using the
                                   -- lazy BRAKE_DECEL there let the craft arrive "hot" (still 80+ m/s)
                                   -- whenever the final leg was shorter than BRAKE_DECEL's stopping
                                   -- distance. Baked into the profile as an ADDITIONAL backward-only
                                   -- sweep from the route end (PLAN.build), min'd against the corner
                                   -- profile, plus the runtime destination brake in dynamicSpeedTarget --
                                   -- so nothing downstream of the last corner can out-run it. Keep
                                   -- DECEL_MAX >= this or the NAV.cmdSpeed slew becomes the bottleneck
                                   -- instead. Raise/lower to match the craft's REAL stopping power.
    TURN_GENTLE    = math.rad(12), -- turn <= this: full advance radius (fly-by); see ADVANCE_*_MULT.
    TURN_SHARP     = math.rad(60), -- turn >= this: tight advance radius. (Speed is curvature-based now.)
    FINAL_ARRIVE_SPEED = 12,       -- commanded speed AT the destination; TERMINAL takes over
    ACCEL_MAX      = 40,            -- m/s^2: max rate the COMMANDED speed may RISE (NAV.cmdSpeed slew).
                                   -- Eases the throttle up OUT of a turn instead of jumping to cruise
                                   -- while still yawing onto the new leg. Lower = gentler spool-up.
    DECEL_MAX      = 64,           -- m/s^2: max rate the COMMANDED speed may FALL. Rounds off sudden
                                   -- steps in the raw envelope into a smooth curve. Keep >= ARRIVE_DECEL
                                   -- (the steepest of the two) so real slowdowns aren't delayed by this
                                   -- slew rather than by the actual physics; lower = smoother but slows later.
    XTK_SLOW_LO    = 50,           -- m off the leg: below this, no cross-track slowdown
    XTK_SLOW_HI    = 180,           -- m off the leg: at/above this, the full XTK_SLOW_MIN cut
    XTK_SLOW_MIN   = 0.30,         -- floor speed fraction when far off the leg (fly in to intercept)
    ADVANCE_GENTLE_MULT = 1.3333,  -- gentle turn: advance this * base pass radius (cut the corner)
    ADVANCE_SHARP_MULT  = 0.7,     -- sharp turn: hold in to this * base radius
}

-- RESUME MID-ROUTE -----------------------------------------------------------
-- FLY ROUTE can join a plan at the nearest sensible waypoint instead of always flying it
-- from the first point (see bestResumeIndex). "Nearest" is heading-aware: a waypoint behind
-- the aircraft costs a U-turn, so its straight-line distance is inflated -- by this weight
-- times the backward component of the displacement -- before the cheapest waypoint is
-- chosen. So the penalty in metres = weight * (how far behind us the waypoint is): a
-- waypoint dead astern at distance d is judged as costing d*(1+weight); one abeam or ahead
-- pays nothing. At 1.0 a waypoint dead behind must be nearer than HALF the distance of one
-- dead ahead to still win -- i.e. turning back has to be substantially cheaper. Raise it to
-- lean harder toward "press on"; drop toward 0 to just pick the geometrically nearest.
CFG.RESUME_BACKTRACK_WEIGHT = 1.0

-- COURSE REFERENCES -----------------------------------------------------------
-- A course.db entry may name another course instead of a point, and that course's own
-- sequence is spliced in where the reference stood: `WAX-TMU=WAX,WAU-TMU` is "WAX, then fly
-- WAU-TMU" without restating WAU-TMU's fixes. How deep such references may nest before the
-- rest is ignored -- a name already on the reference stack is skipped regardless, so a cycle
-- in course.db cannot hang the FMS; this only bounds a silly chain. Same reasoning and same
-- default as landing.db's APPR_MAX_REF (CFG.LG), for the analogous pattern-to-pattern case.
CFG.COURSE_MAX_REF = 4

-- ============================================================================
-- LANDING (pads and runways)
-- ============================================================================
-- Landing gates/profile, tabled rather than one local each (see the 200-locals note below).
--   ALIGN/FINE_HDG_DEG, POS_TOL_M : step 1 coarse heading, then step 3's tight heading+position
--                                   that begins the descent.
--   RECAPTURE_*  : once descending, allow a little more slop before we call it "drifted" and
--                  pause to re-capture the pad -- the descent burn nods the craft and the
--                  vertical thrusters have no vectoring, so X/Z gets shoved.
--   DESCEND_*/FLARE_AGL/TOUCHDOWN_AGL : descent profile, quick up high easing to a gentle rate.
--                  Keep DESCEND_FAST within what the vertical channel can fly (its DESCEND_MAX).
--                  TOUCHDOWN_AGL is now only ONE of the touchdown tests -- see the TD table.
--   TD_SETTLE_*  : touchdown backup for when the pad altitude is a touch off -- if we are
--                  commanding descent but the craft stopped sinking near the pad, that is a landing.
--   PAD_PROBE_M  : A HELIPAD IN landing.db IS ONLY A 3D POINT -- nothing promises there is
--                  anything solid under it. WCL sits at 226 with open air below and is meant to
--                  be STATION-KEPT, not landed on. So reaching the pad altitude is no longer a
--                  touchdown by itself (the AGL test is left to the runway machine, where the
--                  ground is not in doubt): the descent presses on this many metres BELOW the
--                  site looking for a surface -- which a real pad simply blocks, tripping the
--                  squat contact or the settle test -- and if it finds none it climbs the couple
--                  of metres back to the site altitude and HOLDS the published point (phase
--                  HOVER) instead of cutting the engines in mid-air. Measured from the SITE
--                  ALTITUDE down (so the probe actually runs from TOUCHDOWN_AGL, where the
--                  descent used to stop, to PAD_PROBE_M below the pad). Keep it a few metres:
--                  it is slop for a mis-surveyed pad, not a search. 0 still probes the last
--                  TOUCHDOWN_AGL metres and then holds the published altitude exactly.
--                  KEEP TD_SETTLE_AGL >= PAD_PROBE_M: the settle test is the only touchdown
--                  test left to an airframe with no squat contact, and it is deaf outside its
--                  own AGL window -- a probe that reaches below that window would press the
--                  craft into a surface nothing can then report.
CFG.LG = {
    ALIGN_HDG_DEG = 15, FINE_HDG_DEG = 2, POS_TOL_M = 3,
    RECAPTURE_M = 4, RECAPTURE_HDG_DEG = 8,
    DESCEND_FAST = 3.0, DESCEND_SLOW = 0.8, FLARE_AGL = 10.0, TOUCHDOWN_AGL = 1.0,
    TD_SETTLE_VEL = 0.15, TD_SETTLE_AGL = 2.0, TD_SETTLE_TICKS = 3,
    PAD_PROBE_M = 2.0,

    -- RUNWAY APPROACH (landing.db type=rwy) ---------------------------------------------
    -- A runway is NOT flown like a helipad. Instead of stopping over the point and sinking
    -- straight down, the FMS flies a straight-in approach along the runway's landing
    -- direction, descending on a fixed GLIDESLOPE, touches down in the touchdown zone and
    -- brakes along the runway that is left. landing.db's point is the THRESHOLD and the
    -- runway runs from it for rwyLen metres in the site's direction (TMV=609,598 dir s,
    -- 295 m -> it ends at 609,893; TMA is the same strip flown the other way).
    -- ALL of this is FMS-side, and deliberately so: the approach only ever names a point to
    -- fly to, a speed to fly at and a vertical speed -- never an attitude -- so VTOL, HYBRID
    -- and FIXEDWING all fly the IDENTICAL profile with whatever actuators they have. See
    -- LND.rwyTick, and `navGlide` in uac.lua for the one thing the UAC had to be taught.
    RWY_GS_DEG      = 4,     -- deg: the glideslope. Steeper = a shorter final, but a faster
                             -- sink rate for a given approach speed (see RWY_VS_MAX).
    RWY_VS_MAX      = 3.5,   -- m/s: steepest descent the approach may command. MUST stay <=
                             -- the UAC's CFG.AL_DESC_MAX (= 0.75*CLIMB_AT_FULL, 3.75 today),
                             -- which would otherwise clamp it and silently flatten the
                             -- glideslope. THIS AIRFRAME'S DESCENT AUTHORITY IS THE BINDING
                             -- CONSTRAINT ON THE WHOLE APPROACH -- everything below is sized
                             -- around it.
    RWY_GS_MARGIN   = 0.75,  -- fraction of that descent authority the NOMINAL glideslope is
                             -- allowed to use; the remainder is what is left over to CATCH the
                             -- glidepath when we arrive above it (the normal case -- we join at
                             -- whatever altitude ALT-AP was holding). At 1.0 the steepest
                             -- possible descent would be exactly the glideslope itself, so a
                             -- high join could only ever fly PARALLEL to the path and never
                             -- rejoin it -- do not raise this to 1.
    RWY_APP_SPEED   = 80,    -- m/s: wanted speed on final, before the caps that follow.
    RWY_BRAKE_DECEL = 20.0,   -- m/s^2 the rollout is ASSUMED to brake at. THIS is where "how
                             -- NOTE 2026-09-08: the sizing used to compute a*a*rollout where it
                             -- meant 2*a*rollout (v^2 = 2as), so at these values it believed the
                             -- aircraft could stop in a tenth of the room it needs and the
                             -- rollout cap almost never bound. It is 2*a now, as this comment
                             -- and the autobrake always said -- so THIS NUMBER BITES FOR THE
                             -- FIRST TIME. If a landing now crawls down the centreline, this is
                             -- the knob: set it to what the craft really stops at.
                             -- long is the runway" enters the approach: the approach speed is
                             -- capped at sqrt(2*a*rollout), so a short strip is automatically
                             -- flown slower. Set it to what the craft can REALLY stop at --
                             -- too high and it runs off the far end.
    RWY_AIM_FRAC    = 0.15,  -- touchdown aim point, as a fraction of runway length past the
    RWY_AIM_MAX     = 60,    -- threshold, capped at this many metres. These two are now only
                             -- the SMALLEST the aim point may be -- see RWY_TCH.
    -- THRESHOLD CROSSING HEIGHT (2026-09-08). The glidepath is zero AGL at the aim point, so
    -- the height the aircraft crosses the threshold at is aim*tan(gs) -- it used to fall out of
    -- the two knobs above at 0.7 m (NUS) to 3.1 m (TMV) on every runway in landing.db, i.e. the
    -- aircraft crossed the threshold at knee height, having started its RWY_FLARE_AGL flare
    -- 27-61 m BEFORE the strip. It is the INPUT now, and LND.rwyGlide gives way to it: first by
    -- moving the aim point out to where the published RWY_GS_DEG crosses at this height, capped
    -- at RWY_AIM_MAX_FRAC of the strip; then, for whatever is left, by STEEPENING the approach.
    -- A short runway therefore gets a steep approach and, through the approach-speed cap
    -- (RWY_VS_MAX*RWY_GS_MARGIN/tan gs), a slow one -- which is what you would fly onto it by
    -- hand. Never effective below RWY_FLARE_AGL whatever is set here: a flare that begins short
    -- of the threshold is the fault this exists to fix.
    RWY_TCH         = 6.0,   -- m: height above the surface at the THRESHOLD. Must exceed
                             -- RWY_FLARE_AGL (5) or the flare starts before the runway again;
                             -- the metre of margin is what puts the flare a few metres INSIDE.
    RWY_AIM_MAX_FRAC = 0.35, -- most of the strip the aim point may be moved down before the
                             -- glideslope steepens instead. Raise it for a shallower approach
                             -- onto a short strip at the price of the runway left to stop in.
    RWY_STOP_MARGIN = 30,    -- m of runway deliberately left unused at the far end.
    RWY_FINAL_FIX   = true,  -- plan a straight-in final leg (see LND.rwyPrep). false -> the
                             -- approach starts from wherever the plan already ends and
                             -- INTERCEPT does all of the lining up.
    RWY_FINAL_M     = 3000,  -- m: how far back down the extended centreline that FINAL fix
                             -- sits -- i.e. the length of the straight-in leg, and so the
                             -- height available to lose on it. THE PILOT'S RULE OF THUMB:
                             -- be no more than RWY_VS_MAX * RWY_FINAL_M / approach-speed above
                             -- FIELD elevation when the FINAL fix goes by -- about 280 m with
                             -- these numbers. ALT-AP still owns the cruise altitude (the FMS
                             -- only takes the vertical channel once the approach arms), so
                             -- being far higher than that simply cannot be flown down in the
                             -- distance left, and the approach will run through to MISSED.
                             -- Shortened when the waypoint before the runway is closer than
                             -- 1.2x this, since there is no room for the full leg there.
    RWY_FINAL_MIN   = 600,   -- m: ...but a final leg shorter than this is not worth flying --
                             -- there would be no room to settle on the centreline -- so the
                             -- FINAL fix is dropped entirely and INTERCEPT does the lining up.
    RWY_ARM_M       = 3600,  -- m from the threshold at which the approach takes the aircraft
                             -- over. Must exceed RWY_FINAL_M so passing the FINAL fix arms it.
    RWY_CARROT_M    = 200,   -- m: how far ahead the steering point is placed. It sits in the
                             -- LOCALIZER's commanded direction (below), not on the centreline,
                             -- so this is only how far ahead the UAC is asked to aim -- the
                             -- intercept angle comes from RWY_LOC_*, not from this.
    -- LOCALIZER -- how hard the approach pulls back onto the centreline. This is a commanded
    -- lateral closure RATE (m/s) per metre of cross-track error, so RWY_LOC_KP is 1/s and its
    -- reciprocal is roughly the time constant: 0.20 closes an offset with a ~5 s time constant.
    -- Raise it for a crisper capture, lower it if the aircraft weaves along the centreline.
    -- (An earlier version simply pure-pursued a point on the centreline, which works out to an
    -- intercept angle of atan(xtk/RWY_CARROT_M) -- 3.6 deg at 25 m off -- and was far too soft
    -- to centre at all: the aircraft would fly a track parallel to the runway and land wide.)
    RWY_LOC_KP      = 0.20,  -- m/s of lateral closure per metre off the centreline
    RWY_LOC_KD      = 0.50,  -- damping on how fast that error is ALREADY closing (dimensionless)
    RWY_LOC_MAX_DEG = 30,    -- deg: hard cap on the intercept angle, however far off we are
    RWY_CAPT_XTK    = 25,    -- m off the centreline: inside this we are ESTABLISHED and may
    RWY_LOSE_XTK    = 60,    -- go below the approach minimum; outside this an established
                             -- approach gives it up again (hysteresis, so a metre of wander
                             -- cannot flip the phase back and forth).
    RWY_MIN_AGL     = 30,    -- m AGL: the approach minimum. While still lining up we descend
                             -- normally -- that is what the length of the final leg is FOR --
                             -- but never below this until the centreline is captured.
    RWY_FLARE_AGL   = 5,     -- m AGL: below this the glideslope rate eases into RWY_TD_VS.
    RWY_TD_VS       = 1.5,   -- m/s: sink rate flown onto the surface at touchdown.
                             -- THESE TWO SET THE FLOAT, and on a short strip the float is the
                             -- single biggest consumer of runway: easing from the glideslope
                             -- sink to RWY_TD_VS takes 2*FLARE_AGL/(V*tan(GS) + TD_VS) seconds,
                             -- so a 5 m flare at 30 m/s floats ~80 m before the wheels arrive.
                             -- A higher/earlier flare is gentler on the gear and much longer;
                             -- the approach-speed sizing accounts for whichever you choose, so
                             -- softening these buys comfort at the price of approach speed.
    RWY_VS_UP_MAX   = 1.5,   -- m/s: the most the approach may CLIMB to rejoin the glidepath
                             -- from below. Without this the vertical law can only level off and
                             -- wait for the path to come down, which is not a capture at all.
                             -- Never applied inside the flare.
    RWY_ROLLOUT_FRAC = 0.70, -- fraction of the usable rollout the approach speed is sized to
                             -- need, the rest being deliberate go-around slack. 1.0 puts the
                             -- planned touchdown exactly ON the point of no return, so any
                             -- float at all triggers a missed approach; lower it for a slower,
                             -- more forgiving approach onto a short strip.
    RWY_GS_KP       = 0.06,  -- extra m/s of sink per metre ABOVE the nominal glidepath (and
                             -- less sink below it). ~17 s to close the deviation.
    RWY_STOP_SPEED  = 0.6,   -- m/s ground speed under which the rollout is finished.

    -- AUTOMATIC GO-AROUND. A runway may publish a go-around circuit in landing.db as its own
    -- record, in either of two shapes. On a missed approach the FMS flies it as an ordinary
    -- ROUTE and the approach re-arms at the end of it, so the aircraft simply tries again.
    -- With no route published for a runway, a missed approach levels off and holds as before.
    --
    --   `<id>=goaround-wp,minimumAltitude,<waypoints...>`   PREFERRED
    -- The whole pattern, drawn fix by fix, starting AND ending at the runway it serves -- which
    -- is also how it is matched to that runway, by NAME, with nothing inferred. The FMS flies
    -- it exactly as it flies any published course (jerk-limited corners, straight-in final at
    -- the end) and descends by extending the glidepath back along whatever is left of the
    -- pattern: it holds the published minimum until that glidepath sinks below it, then rides
    -- it in. So there is no geometry to get right -- draw the circuit you want flown and pick a
    -- minimum altitude that clears the terrain. Waypoints must exist in points.db.
    --
    --   `<id>=goaround,startX,startZ,endX,endZ,altitude`
    -- One straight LEG, and the FMS invents the rest of the circuit around it: climb away, a
    -- curvature-limited join out to the start, down the leg, back onto the runway line. The id
    -- is arbitrary -- a route is matched to a runway by its LEG DIRECTION (see RWY_GA_ALIGN),
    -- not by its name. GEOMETRY YOU HAVE TO GET RIGHT: the leg must END far enough out that the
    -- altitude can be flown off before the aim point -- about (altitude - field elevation)/
    -- tan(RWY_GS_DEG) metres, plus the aim distance. End it closer than that and the aircraft
    -- arrives above the glidepath, cannot get down, and goes around again until RWY_GA_MAX
    -- stops it.
    --
    -- WHEN A RUNWAY PUBLISHES BOTH, THE WAYPOINT CIRCUIT IS ALWAYS THE ONE FLOWN. It is a drawn
    -- procedure rather than a deduced one; the straight-leg record is then dead weight kept for
    -- aircraft on older software, and is only fallen back on if the waypoint list names points
    -- this aircraft does not have.
    RWY_GA_ENABLE   = true,  -- false -> missed approaches always just level off and hold
    RWY_GA_MAX      = 2,     -- attempts per landing before it gives up and holds instead.
                             -- Reset only when the pilot commits to a new plan.
    RWY_GA_ALIGN    = 0.5,   -- STRAIGHT-LEG FORM ONLY (the waypoint form names its runway):
                             -- min cos(angle) between a route's leg and the landing direction
                             -- for that route to be considered this runway's. 0.5 = within
                             -- 60 deg, which separates the two ends of one strip cleanly.
    RWY_GA_CLIMB_VS = 3.0,   -- m/s: max climb commanded while joining the circuit. Keep at or
                             -- under the UAC's CFG.AL_CLIMB_MAX or that will clamp it.
    RWY_GA_ALT_KP   = 0.10,  -- commanded m/s per metre of error onto the circuit altitude.
    -- The two below size the invented part of a STRAIGHT-LEG circuit. A waypoint circuit has
    -- nothing invented in it, so neither applies to one.
    RWY_GA_RUNOUT_M = 2500,  -- m: how much further than the minimum the FMS may run straight
                             -- before turning in, searched for the shortest total circuit. This
                             -- is what lets it run PAST a leg and turn back onto it, instead of
                             -- looping 360 deg because it tried to turn in from close abeam.
    RWY_GA_STRAIGHT_M = 150, -- m of MINIMUM straight run at the missed-approach heading
                             -- before any turning: clears the runway and lets the speed settle.
                             -- KEEP IT SHORT. It is a floor on the search above, and setting it
                             -- long throws away the room the join needs: turning onto a leg that
                             -- runs the same way a few hundred metres to one side is a gentle S
                             -- from 150 m out and a full 360 deg loop from 200 m out.
    RWY_GA_SPEED    = 80,    -- m/s cap while flying the circuit. Its legs are hundreds of
                             -- metres, not kilometres, so the en-route cruise cap (which may be
                             -- hundreds of m/s) is not the right number to join one at.

    -- PUBLISHED APPROACH PATTERNS. A landing site may publish one or more ARRIVALS in
    -- landing.db, each a list of fixes ending at the site it serves:
    --
    --   `<id>=appr,minimumAltitude,<fixes...>`
    --
    -- On preparation to the landing the FMS chooses ONE of them -- the cheapest to fly from
    -- where the plan arrives and pointing the way it arrives (APR.cost) -- and splices its fixes
    -- into the plan ahead of the site. From there it is an ordinary route, with one exception:
    -- ENTERING the pattern hands the vertical channel to the FMS, which holds the published
    -- minimum and then rides the glidepath down, the same profile a `goaround-wp` circuit is
    -- flown on. That is not decoration -- these procedures put the last fix a few hundred metres
    -- from the threshold, and cruise altitude cannot be flown off in that distance.
    --
    -- A fix list may name another pattern's id: another APPROACH is spliced in where the
    -- reference stood (a family of arrivals sharing one tail), while a GO-AROUND names the
    -- runway it serves and becomes this approach's MISSED-APPROACH procedure -- the pairing an
    -- approach chart prints at the bottom, which LND.gaFor then flies in preference to anything
    -- it could match or infer for itself.
    --
    -- With no approach published for a destination, or with this switched off, a landing is
    -- flown exactly as it was before: straight at the site from wherever the plan ends.
    APPR_ENABLE  = true,   -- false -> published approaches are ignored entirely
    APPR_SPEED   = 80,     -- m/s cap while flying the pattern. Same argument as RWY_GA_SPEED:
                           -- its legs are hundreds of metres, and it ends by descending onto the
                           -- glidepath and handing over to the approach, which this airframe's
                           -- descent authority caps at about this speed anyway
                           -- (RWY_VS_MAX * RWY_GS_MARGIN / tan(RWY_GS_DEG)).
    APPR_ARM_M   = 12000,  -- m of plan STILL TO FLY at which the pattern is entered (vertical
                           -- channel taken, speed capped). Reaching the pattern's first fix is
                           -- not enough on its own: a DIRECT-TO's plan is nothing BUT the
                           -- pattern, so without this such a landing would be flown at the
                           -- published minimum, at approach speed, from the far side of the
                           -- world. SET IT WELL ABOVE THE LONGEST PUBLISHED PATTERN (the ones
                           -- around TMU run to ~5 km): the intent is that a plan which flies to
                           -- a pattern arms exactly where the pattern starts, and that this
                           -- distance only ever bites on a direct-to commanded from far out.
                           -- It also has to leave room to slow from cruise --
                           -- (SPEED_FULL^2 - APPR_SPEED^2)/(2*DC.BRAKE_DECEL), ~4300 m here --
                           -- and to lose the height down to the minimum, which is the slower of
                           -- the two: (cruise alt - minimum)/RWY_VS_MAX seconds at APPR_SPEED,
                           -- ~1700 m from a 300 m cruise. Both fit inside the figure above.
    APPR_TURN_WEIGHT = 1.5, -- how hard the CHOICE punishes an awkward entry, in turn radii of
                           -- track per radian of turning (see APR.cost; the radius is
                           -- APPR_SPEED^2/(DC.GRAVITY*tan(DC.MAX_BANK_DEG)), ~145 m here).
                           -- 0 = choose on distance alone and ignore heading entirely; higher =
                           -- prefer the pattern already ahead of the aircraft ever more strongly
                           -- over a shorter one behind it.
    APPR_STRAIGHT_IN_DEG = 20, -- deg: if the plan's LAST leg already runs within this of the
                           -- landing direction, that plan IS an approach and no pattern is
                           -- spliced into it. This is what keeps a course written with its own
                           -- arrival spelled out (RNF-TMA ends ...,TAIMW,TMA, straight up the
                           -- centreline) flying exactly as it always has -- and what stops a
                           -- published pattern turning the aircraft away from a runway it had
                           -- already lined up on. It is NOT consulted when the plan's last fix
                           -- belongs to one of the site's published patterns and APR.established
                           -- has already found the plan is not flying that pattern: an alignment
                           -- that comes from standing on the procedure's own last leg after
                           -- blundering onto it is not a final approach (see APR.onPattern).
    APPR_JOIN_MAX_DEG = 90, -- deg: the sharpest turn allowed when joining a pattern anywhere
                           -- OTHER than at its published entry fix -- applied both to swinging
                           -- onto the run-in and to the corner at the fix itself. This is what
                           -- separates a legitimate "direct to <fix>" join, which every plan
                           -- built out of the same fixes the procedure uses makes on its own,
                           -- from blundering onto a fix of the procedure off a leg that meets it
                           -- across and having to reverse to follow it. Measured around WAX,
                           -- which is what it was added for: the clean joins are 0-13 deg (a
                           -- route in from ZTP meets KOBUL 13 deg off the procedure's track) and
                           -- the bad ones are 99-156 deg (the auto-router's path from the north
                           -- reaches KOBUL needing 156 deg), so anything from 30 to 95 separates
                           -- them. 90 also reads as a rule: you may not join a procedure by
                           -- turning more than a right angle onto it. Lower = fly more published
                           -- procedures whole; 180 = accept any join and behave as before.
    APPR_MAX_REF = 4,      -- how deep pattern-to-pattern references may nest before the rest is
                           -- ignored. A reference already on the stack is skipped, so a cycle in
                           -- landing.db cannot hang the FMS; this only bounds a silly chain.
}

-- ============================================================================
-- RUNWAY DEPARTURE (DEP in autopilot.lua; _docs/02 section 7b)
-- ============================================================================
-- A plan committed while this aircraft STANDS ON A RUNWAY is flown off it, whatever the flight
-- mode: nose onto the runway heading, onto the centreline (VTOL/HYBRID strafe there; FIXEDWING
-- cannot, and rolls from where it is), a take-off roll to VR, then a climb straight out past the
-- far end and only then a curvature-limited turn for the first fix beyond the aerodrome -- so a
-- fix beside or behind the strip is arced round to, never turned across the runway for.
-- Departures are flown only in a direction landing.db PUBLISHES; standing too far down the strip
-- to reach VR, the aircraft backtracks to the threshold first. The FMS holds the vertical from
-- the line-up (`landed`, on the gear) through a commanded climb to CLIMB_AGL, then hands it to
-- ALT-AP with `cancel`, which restores the selector's pick. AP-NAV gates all of it: committed
-- with AP-NAV off it waits, owning nothing; switched off once started, the departure is given up
-- and the plan is flown as an ordinary one.
--
-- UNFLOWN (2026-10-15): every number below is an estimate from this airframe's other knobs,
-- not a measurement. VR, ROLL_ACCEL and SPEED are the ones to settle in the air.
CFG.DEP = {
    ENABLE       = true,
    -- RECOGNISING "STANDING ON A RUNWAY" ----------------------------------------------------
    STRIP_HALFW  = 25,     -- m either side of the centreline that is still the strip
    STRIP_MARGIN = 30,     -- m beyond either end that is still the strip. Also what marks a plan's
                           -- leading fixes as THIS aerodrome (WAX-TMA starts at WAX's own
                           -- threshold), which are dropped rather than flown to.
    GROUND_AGL   = 3.0,    -- m: hull bottom within this of the runway's surveyed elevation.
                           -- A live squat switch saying "airborne" vetoes it whatever this says.
    MAX_GS       = 3.0,    -- m/s: only from a near-standstill -- a low pass is not a take-off
    -- THE TAKE-OFF ROLL ---------------------------------------------------------------------
    VR           = 30,     -- m/s: rotation speed. Rotates here, or RWY_STOP_MARGIN short of the
                           -- far end whatever the speed. With ROLL_ACCEL this sets the roll,
                           -- VR^2/(2*ROLL_ACCEL) = 75 m, and a line-up point with less than that
                           -- left before RWY_STOP_MARGIN backtracks to the threshold instead.
    ROLL_ACCEL   = 6.0,    -- m/s^2: how fast the roll's commanded speed rises
    XTK_TOL      = 2.0,    -- m off the centreline that counts as lined up
    CENTRE_T     = 10,     -- s lined up on everything BUT the centreline before rolling anyway.
    BACKTRACK_WEIGHT = 3.0, -- what a metre of backtrack costs against a metre flown, when choosing
                           -- between the two ends of a strip published both ways
    -- THE CLIMB-OUT -------------------------------------------------------------------------
    SPEED        = 40,     -- m/s cap from rotation to CLIMB_AGL, and the speed the turn off the
                           -- runway is BUILT for: radius SPEED^2/(DC.GRAVITY*tan(MAX_BANK_DEG)),
                           -- 145 m here. Keep it slow -- a fix a few hundred metres behind the
                           -- strip is a teardrop of this radius.
    CLIMB_VS     = 3.0,    -- m/s commanded until CLIMB_AGL. Keep at or under the UAC's
                           -- CFG.AL_CLIMB_MAX (3.75 on this airframe) or it will be clamped.
    CLIMB_AGL    = 30,     -- m above the runway at which ALT-AP gets the vertical back
    STRAIGHT_M   = 150,    -- m of straight climb-out past the far end before any turning
    RUNOUT_M     = 2500,   -- m further it may run straight if that makes the turn shorter (the
                           -- go-around's search, RWY_GA_RUNOUT_M, for the same reason)
}

-- ============================================================================
-- HSI INSTRUMENT (the external monitor)
-- ============================================================================
-- ============================================================================
-- HSI display constants, kept on one table (like RTR) so the whole instrument section costs a
-- single main-chunk local instead of six -- the file is close to Lua's 200-local ceiling.
CFG.HSI = {
    -- COURSE mode (CDI):
    CDI_FULLSCALE_M = 80,   -- metres of cross-track at full needle deflection
    CDI_ONCOURSE_M  = 10,  -- inside this the needle turns green
    -- TO/FROM: the course arrow is drawn as a compass needle, the tip that points at the
    -- waypoint lit and the other dark. Within this many degrees of abeam the question has no
    -- meaningful answer (the classic VOR ambiguity zone) and both tips go grey rather than
    -- flickering; the number is |cos| of the angle between the course and the bearing, so
    -- 0.09 ~ 5 deg either side of 90.
    TOFROM_DEAD     = 0.09,
    -- LANDING mode (map):
    HDG_BAR_FS_DEG  = 45,   -- heading error (deg) at full-width bar deflection
    HDG_ALIGN_DEG   = 5,    -- within this the bar marker turns green
    -- Runway approach: the glideslope pointer up the right-hand edge. It marks WHERE THE
    -- GLIDEPATH IS (fly toward it), same convention as the CDI needle.
    GS_FULLSCALE_M  = 40,   -- metres off the glidepath at full-scale deflection
    GS_ONPATH_M     = 5,    -- inside this the pointer turns green
    REDRAW          = 0.05, -- s between instrument repaints (display only -- the navigation
                            -- tick that feeds it is CFG.TICK_INTERVAL)
    -- Map range steps (metres to the ring edge): the smallest that still frames the target
    -- wins, so the target creeps inward as you close and the scale only jumps occasionally.
    -- The top two steps are what TERR.COURSE_MAX's comment means by "the ladder tops
    -- out at 5 km"; they were added to the program's own copy of this table back when it
    -- carried one, and only reached here when that copy was deleted (2026-09-16).
    RANGES          = {5, 10, 25, 50, 100, 250, 500, 1000, 2500, 5000},
    DEFAULT_RANGE   = 50,   -- ring scale when there is no target to frame

    -- ========================================================================
    -- PLAN VIEW -- the instrument's second normal-flight view
    -- ========================================================================
    -- The rose answers "where am I relative to the course I am on". This answers "what does
    -- the rest of the flight look like": the baked path itself, the waypoints on it, and the
    -- same course-deviation reading collapsed onto one row along the bottom. The pilot
    -- toggles between the two by TOUCHING THE HSI MONITOR; a landing overrides both and
    -- cannot be switched out of (see autopilot.lua's drawHSI).
    --
    -- IT PAINTS TERRAIN, BUT NOT THE SAME TERRAIN THE ROSE DOES. The rose's overlay is a
    -- LANDSCAPE: every surveyed chunk tinted by its own elevation, hazard bands layered over
    -- the top. Under a route map that is a wash of colour the path has to compete with, and
    -- the elevation of ground the aircraft clears by two hundred metres is not what this view
    -- is for. So this one paints ONLY the bands that mean something is in the way -- warning,
    -- caution, and a floating mass overhead -- over black, and lets the black mean "nothing
    -- here to hit". Two instruments, two questions: the rose says what the land IS, the PLAN
    -- view says what is IN THE WAY.
    --
    -- One consequence worth knowing: an island BELOW the aircraft is not drawn. It is not a
    -- hazard while you are above it, and it becomes one (red, then the whole cell) the moment
    -- a descent brings it within CFG.TERR.WARN_M. If you want to see the island itself, that
    -- is the rose.
    PLAN = {
        ENABLE = true,      -- false takes the second view off the instrument entirely: the
                            -- touch does nothing and the rose is all there is. Set it false
                            -- on an airframe whose HSI monitor is also something a pilot
                            -- leans on.

        -- REBUILD RATE, not repaint rate. The path is rasterised into a subpixel bitmap and
        -- then re-blitted at REDRAW above (20 Hz); this is how often that bitmap is rebuilt.
        -- The compass card and the waypoint glyphs are cached in the SAME rebuild even
        -- though they are cheap, because they are spatially tied to the path: recomputing
        -- them every frame would make them swim against a path that only moves five or ten
        -- times a second, which reads as a fault rather than as a saving.
        --
        -- 0.2 s, which is CFG.TERR.REDRAW exactly -- this view now classifies terrain as well
        -- as rasterising a path, so there is no case for it running faster than the overlay
        -- that does only the first. RAISE IT FIRST if this instrument ever looks like it is
        -- costing the FMS its tick; TRAFFIC.ENABLE and TERRAIN below are the next two levers,
        -- in that order.
        REDRAW = 0.2,

        -- AUTOZOOM. There is no range selector on this instrument and deliberately so -- one
        -- dial that reads correctly from a hover to 320 m/s is worth more than a knob nobody
        -- touches. Three things ask for a range and the SMALLEST wins, then the answer is
        -- snapped up CFG.HSI.RANGES so the scale jumps in the same familiar steps the
        -- landing map uses:
        --
        --   1. the ground speed        -- the boring default: as far ahead as the aircraft
        --      x LOOK_T                   gets in LOOK_T seconds. A long straight leg is
        --                                 exactly this and nothing else, i.e. zoomed out.
        --   2. the tightest corner     -- a corner of radius R needs about R of range to
        --      within that horizon,       read as a corner rather than as a kink, so the
        --      x CORNER_K                 tightest one ahead pulls the scale in. This is the
        --                                 "zoom in on the interesting bit" rule.
        --   3. the distance to the     -- what makes an arrival close in on itself without
        --      LAST waypoint x END_K      anything being selected. Far away it constrains
        --                                 nothing, which is the point.
        --
        -- LOOK_T is the SAME 30 s as CFG.TERR.COURSE_LOOK, and for the same arithmetic: the
        -- ladder tops out at 5 km, and at 60 s even the slowest airframe in the fleet (80 m/s
        -- x 60 s = 4.8 km) pins to that top step, which is a knob that does nothing. At 30 s
        -- Citation and Argo settle on the 2500 m ring and the fast hulls run out of ladder --
        -- and running out of ladder on a long straight leg is the RIGHT answer here, unlike
        -- on the terrain overlay: "zoomed out on the boring part" is what was asked for. This
        -- number earns its keep on the way into and out of a corner, not in the cruise.
        LOOK_T    = 30,     -- s of flight the map tries to frame ahead on a boring leg
        CORNER_K  = 2.2,    -- range asked for by a corner, as a multiple of its radius --
                            -- claimed ONLY ONCE THE CORNER IS INSIDE THAT RANGE, or a corner
                            -- three kilometres ahead would pull the map down to a ring that
                            -- cannot show it (see autopilot.lua's CRS.autoRange). Corner
                            -- radius is clamped to CFG.DC.MIN_RADIUS..MAX_RADIUS (40..3000 m),
                            -- so this spans 88 m to past the top of the ladder.
        END_K     = 1.8,    -- ...and by the destination, as a multiple of the distance to it.
                            -- Above 1 so the destination is framed with the run-in to it
                            -- rather than sitting on the ring edge.
        -- THE BAND IS DELIBERATELY NARROWER THAN THE LADDER, and that is what "moderate
        -- autozoom" means here: the view carries a route, a terrain picture and traffic at
        -- once, and the two ends of the ladder are where one or more of the three stops being
        -- readable. Widen these two numbers and nothing breaks -- the view simply spends more
        -- of its time showing one thing well and the others badly.
        MIN_RANGE = 250,    -- m: never zoom in past this, and it is NOT a free number -- it is
                            -- CFG.TERR.MIN_RANGE, the range below which the terrain layer
                            -- takes itself off the screen (at that scale the ground under a
                            -- landing aeroplane is solid red and says only that it is about to
                            -- land). Below it this view would draw a route over a blank where
                            -- the hazard layer used to be, which reads as "nothing in the way"
                            -- -- a lie, and the worst kind. KEEP THE TWO EQUAL. The code
                            -- enforces the floor as well, so a mismatch is blank, not wrong.
        MAX_RANGE = 2500,   -- m: ...nor out past this -- one step down from the top of RANGES.
                            -- At 5 km a character cell is ~100 m across: the corners of a
                            -- route vanish, and a traffic symbol and its altitude tag cover
                            -- two hundred metres of ground. The ladder still HAS the 5 km
                            -- step; the terrain overlay under the rose uses it.
        HYST      = 0.9,    -- Zooming OUT is immediate (you always want to see far enough);
                            -- zooming IN waits until the wanted range has fallen to this
                            -- fraction of the step below the one currently selected. Without
                            -- it a value sitting on a ladder boundary flaps between two
                            -- scales at the rebuild rate, which is the one thing a moving map
                            -- must not do.
                            --
                            -- 0.9 IS NOT TIMIDITY, IT IS THE LADDER. The steps are 2.5x
                            -- apart, so a deadband of x on the lower step is a lockout of
                            -- x/2.5 measured from the upper one: at 0.75 the map cannot leave
                            -- the 2500 m step until something wants less than 750 m, and a
                            -- 400 m corner -- which asks for 880 -- never moves it at all.
                            -- The wanted range lands wherever the geometry puts it, and it
                            -- lands just inside a step as often as not. 10% still gives a
                            -- [900, 1000] band around that boundary, which at a 5 Hz rebuild
                            -- is ample. Raise it toward 1.0 only if the ladder is ever made
                            -- finer; lower it only after watching the map flap.

        -- DRAWING
        SUBPIXEL   = true,  -- draw the path at 2x3 samples per character cell (the same
                            -- glyph trick CFG.TERR.SUBPIXEL uses, and it shares the program's
                            -- glyph tables). False falls back to whole cells: a sixth of the
                            -- samples, no cheaper to draw, and kept only so the comparison can
                            -- be made in the air.
        MAX_POINTS = 900,   -- hard ceiling on path samples considered per rebuild, whatever
                            -- the route's length. The polyline is CFG.DC.SAMPLE_SPACING (5 m)
                            -- dense, so a 30 km route is 6 000 points and only the ceiling
                            -- bounds the cost. Skipping samples costs accuracy as the CHORD
                            -- error c^2/(8R): at the ceiling a 30 km route gives 25 m chords,
                            -- which on the tightest legal corner (R = 40 m) is 2 m -- about
                            -- one subpixel at the range such a corner is drawn at. Free.
        LABELS     = 8,     -- max waypoint NAMES drawn. Every visible waypoint always gets
                            -- its glyph; the names go to the one being flown to, then the
                            -- destination, then the rest in route order, and a name that
                            -- would land on another mark is dropped rather than overprinted.

        -- THE TERRAIN LAYER. Hazard and obstacle bands only (see the header), sampled one
        -- per CHARACTER CELL and painted as the cells' BACKGROUND, with the path drawn over
        -- it in subpixels. The split is the point: a hazard is a blob and loses nothing at
        -- cell resolution, a path is a line and needs the subpixels, and between them they
        -- spend a cell's two colours exactly once each. It costs no extra radio -- the query
        -- this view puts on the air is the same one the rose's overlay does, at this view's
        -- own range.
        --
        -- The bands are CFG.TERR's own (WARN / CAUTION / OVER / OVER_LO) in CFG.TERR.COL's
        -- own colours, so red means here what it means on the rose. What is dropped is the
        -- gradient: this layer is an ANNUNCIATOR, flat colour per band, with only the coarse
        -- near/far half-step below kept as a checkerboard. Read the gradient off the rose.
        TERRAIN    = true,
        THIN_LVL   = 3,     -- 0..6: a CAUTION cell dimmer than this (further from the warning
                            -- band) and an overhead mass further away than this are drawn as
                            -- a CHECKERBOARD rather than solid. Without it every cell under
                            -- an island is a solid magenta wash and the route is lost in it.
                            -- 0 makes everything solid; 7 makes everything a checkerboard.

        -- TRAFFIC. Receive-only, off the fleet-wide atc_broadcast the responders already
        -- transmit on -- the same reports #44 and #64 paint. Nothing is transmitted back;
        -- adding a SENDER to that protocol is what the one-broadcaster rule forbids, and a
        -- receiver is free.
        --
        -- IT IS NOT A TCAS. Nothing in this fleet computes a resolution advisory: every
        -- responder hardcodes tcas_status = 'CLEAR'. PROX_M / PROX_V below decide what turns
        -- a symbol amber ON THIS DISPLAY and alert nobody; the `!` glyph is reserved for the
        -- day a responder sends something other than CLEAR, so that when one does, this
        -- already shows it.
        TRAFFIC = {
            ENABLE   = true,
            PROTOCOL = "atc_broadcast",  -- FLEET-WIDE, like aer_navdata and aer_terrain. Do
                                         -- NOT rename this per aircraft: it is the channel
                                         -- every responder on the fleet reports on and the
                                         -- one the ATC scopes listen to.
            -- Own-ship is filtered out by callsign, and the callsign used is CFG.TERR.CALLSIGN
            -- -- the one already in this file, which its own comment says is the responder's.
            -- One hull, one name: a second copy here is a second thing to get wrong, and
            -- getting it wrong plots the aeroplane as a permanent contact sitting on top of
            -- itself.

            -- Ageing, in seconds since the last report. Same two-stage shape #64 uses and the
            -- same numbers: a responder that misses a second of reports has not gone
            -- anywhere, and a blip that vanishes and reappears is harder to read than one
            -- that goes grey.
            COAST_AFTER = 3,
            DROP_AFTER  = 20,

            -- The vertical filter, and it is the one knob here that does NOT translate from a
            -- real aeroplane. An ND shows +/- 2700 ft, which is 820 m -- more than twice the
            -- height of this world (384 m floor to build limit), i.e. no filter at all. 120 m
            -- is the honest translation: near enough a third of the world, so an aircraft in
            -- the same layer of air is shown and one on the deck while you are at altitude is
            -- not.
            ALT_FILTER_M = 120,

            -- What turns a symbol amber: inside BOTH of these. Display thresholds, not
            -- alerting ones -- see the note above.
            PROX_M = 1000,   -- m laterally
            PROX_V = 40,     -- m vertically. Deliberately tighter than ALT_FILTER_M: being
                             -- shown and being close are different questions.
            MAX    = 24,     -- most contacts drawn in one frame, nearest first. A ceiling
                             -- rather than a limit anyone expects to reach; it exists so a
                             -- misbehaving transmitter cannot make the instrument the slowest
                             -- thing on the computer.
        },
    },
}

-- ============================================================================
-- TERRAIN OVERLAY -- the ground, painted under the HSI
-- ============================================================================
-- The HSI's terrain picture, fed by the #68 terrain data server over the fleet-wide
-- aer_terrain protocol. It is a DISPLAY and nothing else: it never speaks, never inhibits
-- anything and never touches the flight path. The alerting half of the same database lives
-- on the GPWS (#67) and is configured separately in gpwsconf.lua.
--
-- WHAT THE COLOURS MEAN. Two things at once, layered, the way an EGPWS peaks-mode display
-- does it -- and the layering is the point, because the two answer different questions:
--
--   THE HAZARD BANDS are RELATIVE to this aircraft, and they win wherever they apply:
--     red     within WARN_M below us, or at/above us, or inside a floating mass
--     amber   within CAUTION_M -- we clear it, but not by enough to be relaxed about
--   Amber shades into red as the clearance closes, so the boundary is a gradient and not
--   a line. These two are the only colours that mean ACT, which is why they are the only
--   saturated ones on the display.
--
--   THE HEIGHTMAP is ABSOLUTE, and it paints everything else: charted ground is tinted by
--   its own elevation between TINT_LO_Y and TINT_HI_Y, lush green through olive and khaki
--   to arid sand, dithered into a continuous gradient. It says what the LAND is, not where
--   we are -- so the shape of a valley stays the same shape whatever altitude we fly it at,
--   which is exactly what the old relative-only scheme could not do.
--
-- The trade is deliberate and worth stating: below the caution band the colour no longer
-- tells you how much room is under you. It tells you what the ground is. Clearance is what
-- the GPWS is for, and it has not changed.
--
--   navy    water, at any altitude, flat -- a sea is a sea and never a hazard colour
--   magenta a floating mass ABOVE us. Solid while its underside is within CAUTION_M, then
--           fading toward black as it gets further overhead until it is not worth drawing.
--           Off both scales on purpose: this world's obstacles float, an aeroplane can quite
--           reasonably be flying UNDER one, and colouring that red would be a lie.
--
-- black means one thing only: nobody has surveyed that chunk.
CFG.TERR = {
    ENABLE   = true,
    PROTOCOL = "aer_terrain",  -- FLEET-WIDE, like aer_navdata. Must match terrainconf.lua's
                               -- CFG.LINKS.TERRAIN. Do NOT rename this per aircraft: one
                               -- database serves the whole fleet.
    CALLSIGN = "TOGEKISS",     -- how this aircraft names itself on the server's client list
                               -- and in its activity roster (which is what tells the offline
                               -- builder where to re-survey). Same name as responder.lua's.

    -- Freshness. STALE is measured from the last word heard from the server, which beacons
    -- every 15 s. GRID_STALE is how old the terrain picture may be before it is taken off
    -- the screen rather than shown: a stale terrain map is worse than none, because it
    -- looks exactly like a current one.
    STALE      = 45,
    GRID_STALE = 6,

    -- Asking. One map query every REQ_EVERY seconds; the answer is a grid the 20 Hz
    -- instrument then re-reads against LIVE altitude, so the colours track a climb or a
    -- descent immediately while the radio runs at one hertz.
    REQ_EVERY = 1.0,
    GRID      = 96,    -- cells per side asked for (the server caps this at its MAP_CELLS).
                       -- Sized to the SUBPIXELS, not the characters: the instrument draws
                       -- 2x3 per character cell, so a 57-wide monitor puts 114 samples
                       -- across the ring and a grid coarser than that has nothing new to
                       -- say. 96 x 96 packed is a 9 kB reply -- less than the 40 x 40 x 4
                       -- this used to ask for, because PACK below spends a quarter of the
                       -- bytes per cell.
    PACK      = true,  -- ask for the one-byte-per-cell format (terrain level + water packed
                       -- into a byte, obstacles sent sparsely). Set false to fall back to
                       -- the four-byte records, which cost four times the wire for exactly
                       -- the same picture; it exists so the older format stays exercised.
    MARGIN    = 1.25,  -- ask for this much more ground than the ring shows. The grid is cut
                       -- where the aircraft WAS: at 320 m/s a one-second-old grid is 320 m
                       -- behind, and without the margin the far edge of the ring would be
                       -- blank for the second half of every cycle.
    SUB       = 3,     -- chunks per axis sampled inside one grid cell, server-side. It
                       -- reduces this itself if the grid would cost too much.
    CELL_MIN  = 16,    -- m per grid cell: one chunk, the database's own resolution -- asking
                       -- for finer just repeats the same record.
    CELL_MAX  = 512,

    -- Range. In LANDING mode the overlay uses the map's own range so both agree. That range
    -- is chosen to frame the pad and shrinks to metres at touchdown, where a terrain picture
    -- is meaningless and would be solid red (we are, after all, landing on the terrain), so
    -- below MIN_RANGE the overlay takes itself off the screen.
    MIN_RANGE = 250,
    -- In COURSE mode the rose has no range of its own, so the overlay picks one from the
    -- ground speed -- the ground reachable in COURSE_LOOK seconds -- snapped up to the HSI's
    -- own range ladder (CFG.HSI.RANGES) so the scale jumps in familiar steps instead of
    -- breathing. One dial then reads correctly from a hover to 320 m/s with nothing for the
    -- pilot to select.
    --
    -- 30 s, not the 60 a forward-looking GPWS uses, because the ladder tops out at 5 km and
    -- 60 s pins EVERY airframe in the fleet to that top step -- 80 m/s x 60 s is already
    -- 4.8 km -- which is a knob that does nothing. At 30 s the slow airframes (Citation and
    -- Argo, 80 m/s) settle on a 2.5 km ring where the picture is worth looking at, and this
    -- one still runs out of ladder at cruise, which for a 320 m/s aeroplane is the right
    -- answer: show as far as the display can.
    COURSE_LOOK = 30,
    COURSE_MIN  = 500,
    COURSE_MAX  = 2500,

    -- The bands, in metres of vertical clearance. Wider than the GPWS's alerting thresholds
    -- (gpwsconf.lua CAUTION_M / WARNING_M) on purpose -- the instrument's job is to show a
    -- hazard well before the annunciator shouts about it -- but NOT much wider, and that
    -- ceiling is set by the world rather than by taste.
    --
    -- MINECRAFT IS 384 METRES TALL. Bands sized for a real EGPWS (which thinks in thousands
    -- of feet) do not survive the move: at CAUTION_M = 150, ordinary cruise at 200 m over
    -- sea-level ground at 63 m is 137 m of clearance and paints the whole screen yellow, and
    -- a display that is yellow all the time is a display nobody looks at. The same arithmetic
    -- kills the overhead bands from the other end: an island's underside and the ground below
    -- it are rarely more than ~200 m apart here, so with a 150 m caution band every cell
    -- under an island is a caution and `purple` -- "there is something above you, it is not a
    -- factor" -- can never be reached at all.
    --
    -- The pair below is FLOWN, not derived, and it was tightened from the 30/80 this
    -- shipped with after flying it. Keep them small enough that cruise over open country
    -- reads as landscape rather than as a warning, and large enough that a ridge coming up
    -- announces itself before the GPWS does. The numbers are deliberately not quoted
    -- anywhere else in this comment, because they will be tuned again and a comment naming
    -- a stale value is worse than no comment.
    WARN_M    = 14,
    CAUTION_M = 32,

    -- CONTEXT_M is the far edge of the ramp: charted ground further below than this is not
    -- painted at all. A real EGPWS blanks terrain about 600 m below the aircraft, and 600 m
    -- is MORE THAN THIS WORLD IS TALL -- floor to build limit is 384 m -- so the faithful
    -- translation of that rule is "never blank surveyed ground", and 400 is how you write
    -- that. It matters: at 250 an aeroplane at 360 m over the sea had a completely black
    -- screen, which is indistinguishable from the terrain server having died.
    --
    -- With it set past the height of the world, black on the display means exactly one
    -- thing, and it is a useful thing: nobody has surveyed that chunk.
    CONTEXT_M = 400,

    -- ============================================================================
    -- THE HEIGHTMAP WINDOW -- which elevations the tint spreads itself over
    -- ============================================================================
    -- Ground at or below TINT_LO_Y takes the lushest green, at or above TINT_HI_Y the most
    -- arid sand, and the four tint steps (plus the dither between them) spread evenly across
    -- the span. Get this wrong and the map is one flat colour: too wide a window and all the
    -- land lands in the bottom step, too narrow and everything saturates at sand.
    --
    -- MEASURED, not guessed. Sampling 292k charted chunks of this world: 38% is water (which
    -- the tint never touches), and of the land the terrain top runs p25 = Y63, p50 = Y79,
    -- p75 = Y95, p95 = Y135, p99 = Y167. Sea level to the 95th percentile is therefore the
    -- window that puts the most ground on the most steps; the rare 5% above it saturating at
    -- sand is correct, since that IS the high ground.
    --
    -- Re-measure if the world grows a mountain range: the tint is only as informative as this
    -- window is honest.
    TINT_LO_Y = 63,
    TINT_HI_Y = 136,

    -- ============================================================================
    -- DITHER -- how the five-colour ramp is made to look like far more than five
    -- ============================================================================
    -- Sub-steps blended between each adjacent pair of ramp colours, 0..6. This is where the
    -- extra shades come from, and it costs NOT ONE palette slot: all sixteen are already
    -- spoken for (see CFG.PALETTE.HSI), so more distinct colours are simply not available.
    --
    -- What is available is the rest of the character cell. Every terrain cell used to be a
    -- space with a background colour, using one of the three things blit can set. CC's 2x3
    -- subpixel glyphs (128..159) let a cell carry SIX pixels split between a foreground and
    -- a background colour, so two neighbouring ramp colours can be mixed inside one cell at
    -- seven densities. Five solid steps plus six blends between each adjacent pair is about
    -- twenty-five distinguishable tones out of five slots.
    --
    -- The blend is by CLEARANCE, so the tone within a band says where in that band the
    -- ground sits -- which is what turns a flat amber patch into a slope. The gradation is
    -- naturally finest where the bands are narrowest, i.e. close to the aircraft, which is
    -- where a metre matters: with the bands as tuned, a step of tone is a couple of metres
    -- inside WARN and tens of metres out in CLEAR_LO.
    --
    -- 6 is the maximum the glyphs can express. Lower values quantise it (3 gives half-steps,
    -- 2 gives thirds); 0 turns dithering off entirely and every cell goes back to a flat
    -- block of its band colour, which is the exact behaviour this shipped with. If the
    -- dither ever reads as speckle rather than as shading on a real monitor, set it to 0 --
    -- one value, no other change needed.
    DITHER = 6,

    -- ============================================================================
    -- SUBPIXEL -- 2x3 samples per character cell instead of one
    -- ============================================================================
    -- CC's block glyphs address six pixels inside one character. Off, a character is one
    -- terrain sample. On, it is SIX -- twice the horizontal resolution and three times the
    -- vertical, which is what makes a ridgeline read as a line rather than as a staircase.
    --
    -- THE CATCH, and it is the whole design: a character cell can only ever hold TWO
    -- colours. Six samples therefore get quantised down to two, the classic colour-clash
    -- problem. For a heightmap that is usually free -- neighbouring samples fall in the same
    -- or adjacent bands -- and where it is not, the quantiser keeps the MOST SEVERE colour
    -- present and maps the rest to whichever of the two chosen colours is nearest. A hazard
    -- subpixel is never the one that gets dropped.
    --
    -- AND WHERE THE SIX SAMPLES AGREE, the cell falls back to DITHER instead, shading itself
    -- by where it sits inside its band. So the two features do not fight over the same six
    -- pixels: varied ground spends them on detail, flat ground spends them on gradation.
    --
    -- WHAT IT COSTS, measured rather than guessed: a raster build goes from 0.65 ms to
    -- 3.1 ms on a desktop Lua, so call it five times the work -- not six, because the six
    -- subpixels of a character usually land in the same grid cell and it is classified once
    -- (see TERR.at's memo). Cobalt is an order of magnitude slower again, and a build runs
    -- at CFG.TERR.REDRAW, so this is real load on the computer.
    --
    -- Two levers if the instrument ever feels heavy, in order: raise CFG.TERR.REDRAW, which
    -- costs nothing but how fast the ground recolours during a climb; or set this false,
    -- which keeps the picture correct and makes it blockier.
    SUBPIXEL = true,

    -- Floating masses below this class are ignored, exactly as the GPWS ignores them: the
    -- builder honestly charts one stray block a player left at altitude as an obstacle, and
    -- painting the map for it would be noise. 0 none, 1 thin, 2 medium, 3 large.
    MIN_MASS = 2,

    REDRAW = 0.2,   -- s between rebuilds of the raster. The instrument repaints at
                    -- CFG.HSI.REDRAW (20 Hz) and re-blits the cached rows; rebuilding the
                    -- classification that often would cost 28000 chunk decodes a second for
                    -- a picture that cannot change meaningfully in 50 ms.

    -- The bands, in order of severity, mapped to palette SLOTS by name -- resolved against
    -- `colours` in the program, exactly as CFG.PALETTE is. The slot names are just the
    -- sixteen boxes CC gives a monitor; what each one actually looks like is set in
    -- CFG.PALETTE.HSI below, so read "lightGrey" here as "the amber box", not as grey.
    --
    -- WHICH SLOTS ARE FREE, and why these. The instrument spends eight of the sixteen:
    -- black, white, grey, orange, cyan, blue, lime and yellow (the CDI needle). That leaves
    -- eight, which is exactly what the ramp plus the overhead pair needs.
    --
    -- WATER USED TO SHARE `blue` WITH THE COURSE NEEDLE'S AWAY TIP, and that was wrong in
    -- the air: measured, the two were a 1.00:1 contrast ratio -- the same colour -- so the
    -- away tip simply vanished over the sea. The eighth slot came from `brown`, which the
    -- instrument was spending on ONE cell (the far end of a runway); that marker is drawn in
    -- white now and reads at least as well against the orange strip.
    COL = {
        WARN    = "red",        -- at or above us, or about to be
        CAUTION = "lightGrey",  -- (the amber box)
        TINT4   = "purple",     -- (arid sand)   highest ground
        TINT3   = "pink",       -- (khaki)
        TINT2   = "brown",      -- (olive)
        TINT1   = "green",      -- (lush green)  lowest ground
        WATER   = "lightBlue",  -- (navy)
        OVER    = "magenta",    -- a ceiling within CAUTION_M
        OVER_LO = "magenta",    -- ...and further off. SAME SLOT on purpose -- it is the same
                                -- hazard, and the dither to black carries the distance, which
                                -- is what freed a slot for the tint.
    },
    -- Fill density per band: 1 = every cell, 2 = checkerboard, 3 = one cell in four.
    --
    -- All solid now. The thinning was there to stop a flat green wash burying the rose, and
    -- the ramp does that job better by being DARK at the safe end -- so the ground reads as
    -- ground instead of as a stipple, and black on the screen means one thing only: nobody
    -- has surveyed that chunk. The knob stays because a busier airframe may want the texture
    -- back; set CLEAR/CLEAR_LO/WATER to 2 for the old look.
    DENSITY = {
        WARN = 1, CAUTION = 1, TINT4 = 1, TINT3 = 1, TINT2 = 1, TINT1 = 1,
        WATER = 1, OVER = 1, OVER_LO = 1,
    },

    -- Record flag bits, as terrainbuild.jar writes them and terrainconf.lua mirrors them.
    -- All three must agree; they are the wire format.
    FLAG = { MASS = 3, WATER = 4, MULTI = 8, TREES = 16, SPIKE = 32, VOID = 64, NODATA = 128 },
}

-- ============================================================================
-- TERRAIN AVOIDANCE -- an automatic altitude floor, fed by the same #68 database
-- ============================================================================
-- CFG.TERR above is a DISPLAY and, by its own header, "nothing else". This is the other
-- half of that sentence: the one place in the FMS that is allowed to read the terrain
-- database and DO something about it, rather than just paint it. It never touches the
-- lateral path -- climbing over a ridge is a much smaller, better-understood claim than
-- steering round one would be -- and it never runs during a landing or a go-around, whose
-- vertical profiles are already authored (an approach minimum, a glideslope, a published
-- circuit) and hand-verified against the ground the way this cannot be.
--
-- THE MECHANISM IS NOT NEW. Taking the vertical channel away from ALT-AP is exactly what
-- an autoland already does, on the same AUTOLAND_PROTOCOL, with the same "hold THIS
-- altitude outright" command the HOVER phase uses to sit on a published site altitude (see
-- autopilot.lua's broadcastVertical). uacconf.lua's own note on CFG.MANCON_SCRAMBLE_OVERRIDE
-- says as much already -- the scramble detent is documented there as what "makes it usable
-- as a hand-flown go-around and as a terrain escape", i.e. an override the pilot can reach
-- for at the exact moment ALT-AP or an autoland is holding the craft somewhere it must not
-- stay. That override, and the gentler one on the ALT-AP panel (tap ALT during OVER -- see
-- overrideLoop), both work against this exactly as they already work against a landing:
-- nothing new needed either place.
--
-- AP-NAV GATE (2026-09-05). Engages ONLY while AP-NAV is on -- read directly off this
-- computer's own CFG.APSWITCH.SIDE redstone input, the same signal apAnnounceLoop already
-- re-announces as apswitch_1 -- and stands down AT ONCE the instant it goes off, mid-hold or
-- not, ahead of every other check in TAVOID.tick (so even a stale/unknown corridor reading
-- cannot leave a hold running past it). AP-NAV off means the pilot has taken the horizontal;
-- full manual is full manual, with no carve-out for "but it was already climbing" -- the same
-- rule the override paths above already follow, just checked before the fact here rather
-- than after it. The disengage on this path is a "cancel", not a "release" -- see
-- autopilot.lua's TAVOID.tick and the fix noted in CLAUDE.FINISHED.md #12 for why the
-- distinction matters (it is the one that dropped the pilot's ALT-AP pick on the floor).
--
-- WHAT IT ASKS FOR. A straight corridor along the current ground track (or the nose, below
-- MIN_GS), the same shape of request GPWS's Mode 2/FLTA already send #68 -- see gpwsconf.lua's
-- CFG.TERRAIN for the twin of every knob below and the reasoning behind it, which is not
-- repeated here. The look is cut at the FMS's own next waypoint plus LEG_MARGIN: the corridor
-- is a straight line and a route rarely is, so ground the plan turns away from before reaching
-- is not this aircraft's problem (see gpwsconf.lua's LEG_MARGIN for the arrival that motivated
-- this the first time).
--
-- WHAT IT DOES WITH THE ANSWER. The required altitude is the highest (charted top + CLEAR_M)
-- of anything in the corridor window -- terrain or an obstacle heavy enough to clear MIN_MASS,
-- floor and ceiling both climbed over, never threaded underneath (this is deliberately the
-- simple, conservative half of what a real TAWS escape manoeuvre would consider). Below that
-- altitude and not yet engaged: take the vertical channel and hold it there. Already engaged:
-- ratchet the held altitude UP if a taller hazard slides into the window, but never let it
-- SINK back down except by a full release -- once actually above (required + RELEASE_MARGIN)
-- with a FRESH, on-track, charted reading, hand the channel back to ALT-AP. That asymmetry is
-- the point: the corridor is read continuously as the aircraft flies (so passing a peak drops
-- the requirement and eventually releases on its own), but nothing here ever DESCENDS the
-- aircraft -- only ALT-AP or the pilot do that, same as before this existed.
--
-- FAIL SILENT, THE SAME WAY GPWS DOES, BUT IN BOTH DIRECTIONS. No server, a stale corridor, a
-- profile off the current track, or a window with nothing charted in it at all: the answer is
-- "don't know", and "don't know" changes NOTHING -- an aircraft not yet engaged stays not
-- engaged (never invents a climb from a guess) and one already engaged stays exactly where it
-- is (never releases into ground it cannot presently see). Only a FRESH, honest, in-range
-- reading may ever raise the floor or take it away again.
--
-- WHAT IT DELIBERATELY DOES NOT DO. No lateral avoidance -- a climb is one axis and a well
-- understood one; a reroute would need to re-plan the baked clothoid path around an obstacle
-- mid-flight, which is a much bigger claim this file is not making yet. No help during a
-- landing/go-around -- see LND.active/LND.ga above. And no substitute for GPWS where one is
-- fitted -- this can only ever climb over what the database has charted; the annunciator's
-- independent read of the same ground stays exactly as necessary as it always was.
--
-- ROLLOUT. autopilot.lua is the ONE program every airframe's FMS runs, so CFG.TAVOID is a
-- required section on every apconf.lua the moment this exists anywhere (same reasoning as
-- CFG.TERR) -- but ENABLE is the per-aircraft flight-test gate, exactly like Scout's
-- CFG.TRI_ENABLE. Flown here first because Aer Togekiss is the primary VTOL development
-- platform; leave it false elsewhere until it has actually been flown.
--
-- CLIMB_AT_FULL CAVEAT: 1/pid/uacconf.lua's CFG.CLIMB_AT_FULL is still an ESTIMATE (see
-- CLAUDE.TODO.md #1), and MIN_RANGE/LOOK_AHEAD_TIME below are sized generously to give the
-- real cascade -- whatever it turns out to be -- room to climb an unknown deficit before the
-- corridor runs out. Re-check them once #1 is measured; a slower real climb than assumed
-- would want more range, not less.
CFG.TAVOID = {
    ENABLE = true,   -- the master arm; false leaves the section required but the feature inert

    -- The clearance the FMS climbs to hold over the worst charted thing in the window, and
    -- how much MORE than that it wants to see before handing the channel back -- the gap
    -- between the two is the hysteresis band that stops a release re-triggering itself at
    -- once. Same physical meaning as gpwsconf.lua's CFG.TERRAIN.CLEAR_M (20), which is why it
    -- is the same number: an aircraft that annunciates AND acts at two different margins would
    -- be a confusing aeroplane to fly.
    CLEAR_M        = 20,
    RELEASE_MARGIN = 15,

    -- Freshness -- see gpwsconf.lua's twins for the reasoning. STALE is the server link
    -- (shared with CFG.TERR.STALE, same server, same beacon); PROF_STALE is how old the
    -- cached corridor itself may be before it stops being believed.
    STALE      = 45,
    PROF_STALE = 4,

    -- Asking. One corridor request a second; the navTick re-reads the cached answer against
    -- LIVE altitude at 20 Hz, so acting runs at tick rate while the radio runs at 1 Hz.
    REQ_EVERY = 1.0,
    STEP      = 16,     -- m between samples -- one chunk, the database's own grid
    WIDTH     = 60,      -- m either side of track the server reduces over. Wider than
                         -- gpwsconf.lua's 48: this hull is described in CLAUDE.md as "a
                         -- massive, superwide VTOL", and the corridor should be at least as
                         -- wide as the airframe it is meant to keep off a ridge.
    MIN_RANGE       = 3000,  -- m of corridor asked for even when slow or stopped
    RANGE_MARGIN    = 1.3,   -- ask for this much more than LOOK_AHEAD_TIME needs
    LOOK_AHEAD_TIME = 40,    -- s of flight time the corridor should cover at the current
                             -- ground speed -- generous on purpose (see the CLIMB_AT_FULL
                             -- caveat above); at SPEED_FULL this and MAX_SAMPLES both want to
                             -- bind, which is the point -- ask for as much as the server will
                             -- serve rather than under-ask at 320 m/s.
    MAX_SAMPLES     = 700,   -- never ask for more than the server will serve (matches gpwsconf)

    -- Using. MIN_GS is the speed below which the profile is cut along the nose rather than a
    -- ground track that is mostly noise. TRACK_COS_MIN replaces gpwsconf.lua's angle-based
    -- TRACK_TOL with its cosine (autopilot.lua has no wrap180 helper and does not need one for
    -- a dot-product test) -- 0.866 is cos(30 deg), the same tolerance. BEHIND is how far the
    -- aircraft may have drifted backwards along the cut profile and still trust it.
    MIN_GS        = 3,
    TRACK_COS_MIN = 0.866,
    BEHIND        = 200,

    -- How far PAST the FMS's own active waypoint the look still bothers with, in metres --
    -- see the header above. With no plan being flown the full reach is used.
    LEG_MARGIN = 300,

    -- Obstacles. Ignores the thin mass class, same reasoning as gpwsconf.lua's MIN_MASS: one
    -- stray block a player left at altitude is not worth a climb.
    MIN_MASS = 2,

    -- After the pilot taps ALT-AP to take the channel back (overrideLoop), how long the
    -- system stands down even if the corridor still says the same ground is unsafe. A pilot
    -- override that gets fought at once is not an override; the MANCON scramble detent and,
    -- where the airframe carries one, the GPWS remain armed underneath for the whole cooldown.
    OVERRIDE_COOLDOWN = 30,

    -- Record flag bits -- the wire format, mirrored (not shared) from terrainconf.lua/CFG.TERR,
    -- same convention as everywhere else this database is read.
    FLAG = { MASS = 3, WATER = 4, MULTI = 8, TREES = 16, SPIKE = 32, VOID = 64, NODATA = 128 },
}

-- ============================================================================
-- AUTOMATED ATC -- the client half of the controller on computer #87 (atcctl.lua)
-- ============================================================================
-- Inbound to a landing facility that sites.db puts in a SITE, the FMS asks the controller
-- once a second whether it may land there. The controller watches every responder's position
-- report and answers CLEARED, DIVERT (to another facility of the same site -- flown as a
-- DIRECT-TO-LAND, published approach and all), HOLD, or "uncontrolled". See autopilot.lua's
-- AUTOMATED ATC CLIENT section and _docs/02-fms-autopilot.md section 8a.
--
-- FAIL SILENT. With no controller on the air nothing here changes anything -- the aircraft
-- lands where the plan says, exactly as before this existed -- and a controller that goes
-- quiet changes nothing it had already said. With ENABLE false the FMS never asks at all.
--
-- The pilot has the last word: RELEASE ATC on the status screen (in place of the SPD button
-- during a hold) flies the rest of the plan unmanaged, and AP-NAV off or any new plan ends
-- whatever the controller said.
--
-- The controller tells this aircraft's own responder from somebody parked on the facility by
-- CALLSIGN, and the name it uses is CFG.TERR.CALLSIGN above. It must match this hull's
-- responder.lua CALLSIGN, or the aircraft will be told to hold for itself once it lands.
CFG.ATC = {
    ENABLE   = true,
    PROTOCOL = "aer_atc",   -- fleet-wide; must match atcctlconf.lua's CFG.LINKS.CTL

    -- WIDE: facilities sites.db marks (narrow) are refused to this airframe -- the controller
    -- diverts it to one that takes it, or holds it. It cannot see a wingspan; this says it.
    -- RUNWAY_ONLY: never offer this airframe a helipad (a FIXEDWING cannot land on one).
    WIDE        = true,     -- "a massive, superwide VTOL" -- sites.db's own example of wide
    RUNWAY_ONLY = false,

    -- When to call up: once the facility is within CONTACT_M plus the distance it takes to
    -- stop from the current ground speed (v^2 / 2*DC.DECEL_MAX), so a HOLD always has at
    -- least this much room to be flown short of the field. Larger = decided earlier, and the
    -- facility reserved for longer; the stop term is what keeps it honest at cruise speed.
    CONTACT_M = 2000,

    INTERVAL = 1.0,  -- s between requests. The controller forgets a request after its own
                     -- CFG.REQ.STALE (5 s), so this must stay well inside that.
    STALE    = 5,    -- s without an answer before the controller counts as silent. Silence
                     -- changes nothing (a hold stays held); it only changes what is shown.

    -- How this airframe HOLDS:
    --   "hover" -- brake to a standstill on the planned path: cruiseCap 0 plus the `align`
    --              flag, so the UAC brakes the whole velocity vector (the route-start ALIGN
    --              hold's mechanism). VTOL/HYBRID only -- the UAC honours the flag only with
    --              STRAFE_ENABLED, which it derives from FLIGHT_MODE ~= "FIXEDWING".
    --   "orbit" -- circle ORBIT_R to the right of the track at ORBIT_SPEED, for an airframe
    --              that cannot stop. ORBIT_R and ORBIT_SPEED are then required.
    HOLD = "hover",
}

-- Palettes. CC's default colours are too bright/flat for an instrument face, so both
-- screens repaint a few slots (in a pcall -- a non-colour monitor simply keeps its own).
--
-- ALL SIXTEEN ARE SPOKEN FOR, and the way they are spent is a luminance budget, not a list
-- of favourite colours. Every line on this instrument is drawn OVER the heightmap, so the
-- two cannot share a brightness range: the tint is confined to the DARK third (luminance
-- 0.03 to 0.19) and every instrument mark sits above it, clearing the tint's ceiling by at
-- least 1.5x and mostly by more than 2.5x. That is why the greens are deep and the sand is a
-- dark khaki rather than a beach: a brighter heightmap is a prettier still picture and an
-- unreadable instrument, which is a trade this display has already made twice and regretted.
--
-- The resolution the tint gives up in brightness it takes back in DITHERING. Measured over
-- the real window: ELEVEN distinct tones, all inside that dark third. Not nineteen, which is
-- what four steps blended six ways would give -- the binding limit is the DATABASE, whose
-- vertical resolution is 8 m per level, so a 73 m window holds only about nine distinct
-- ground heights to colour. Widening DITHER past that buys nothing; the way to more relief
-- is a narrower window, not a finer blend.
--
-- WHITE IS NOT A TERRAIN COLOUR, though it is technically free (it is only ever drawn
-- through hText, which paints its own black background, so it could be). Spending it would
-- buy a fifth tint step and cost the legibility of every cardinal, the aircraft symbol and
-- every number on the screen, which all become dark khaki. Four steps is the better trade.
--
-- Saturation does the rest of the work: the two hazard colours are the only saturated ones,
-- so amber reads as a warning and khaki reads as a hillside even where they are close in hue.
CFG.PALETTE = {
    TERM = { orange = 0xffbb00, brown = 0x553322, green = 0x116611 },
    HSI  = {
        -- the instrument, all of it above the tint's ceiling
        cyan   = 0x47daff,   -- course needle, lit (toward) tip; target glyph; DTK/GS
        blue   = 0x51bad5,   -- ...and its away tip: same hue, clearly dimmer, still clear of
                             -- the ground. Was 0x2f8fa8, which the heightmap would have eaten.
        grey   = 0xbbc7d0,   -- rose, card ticks, deviation dots, labels
        orange = 0xffbb00,   -- lubber, target, runway, alerts
        lime   = 0x58ec3b,   -- on-course / on-path / aim point / CMD
        yellow = 0xf5ee40,   -- off-course CDI needle
        -- the two that mean ACT
        lightGrey = 0xc18f1b,  -- amber caution
        red       = 0xf01c10,  -- warning
        -- the heightmap, lowest ground to highest
        green  = 0x103b16,   -- lush green
        brown  = 0x2c4e1c,   -- upland green
        pink   = 0x586130,   -- hill khaki
        purple = 0x867559,   -- arid sand
        -- ...and the sea, and what hangs over it
        lightBlue = 0x16324e,
        magenta   = 0xd040d0 },
}
CFG.HSI_TEXT_SCALE = 0.5   -- the instrument monitor's text scale (smaller = more cells)

return CFG

