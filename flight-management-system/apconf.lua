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
    CURVE_LOOKAHEAD = 400,         -- m: how far ahead on the path the carrot sits. Larger = smoother /
                                   -- cuts corners more; smaller = tracks the waypoints tighter.
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
    ARRIVE_DECEL   = 30,           -- m/s^2: deceleration for the FINAL destination stop specifically --
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
    TURN_SHARP     = math.rad(90), -- turn >= this: tight advance radius. (Speed is curvature-based now.)
    FINAL_ARRIVE_SPEED = 12,       -- commanded speed AT the destination; TERMINAL takes over
    ACCEL_MAX      = 12,            -- m/s^2: max rate the COMMANDED speed may RISE (NAV.cmdSpeed slew).
                                   -- Eases the throttle up OUT of a turn instead of jumping to cruise
                                   -- while still yawing onto the new leg. Lower = gentler spool-up.
    DECEL_MAX      = 32,           -- m/s^2: max rate the COMMANDED speed may FALL. Rounds off sudden
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
CFG.LG = {
    ALIGN_HDG_DEG = 15, FINE_HDG_DEG = 2, POS_TOL_M = 3,
    RECAPTURE_M = 4, RECAPTURE_HDG_DEG = 8,
    DESCEND_FAST = 3.0, DESCEND_SLOW = 0.8, FLARE_AGL = 10.0, TOUCHDOWN_AGL = 1.0,
    TD_SETTLE_VEL = 0.15, TD_SETTLE_AGL = 2.0, TD_SETTLE_TICKS = 3,

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
    RWY_APP_SPEED   = 40,    -- m/s: wanted speed on final, before the caps that follow.
    RWY_BRAKE_DECEL = 4.0,   -- m/s^2 the rollout is ASSUMED to brake at. THIS is where "how
                             -- long is the runway" enters the approach: the approach speed is
                             -- capped at sqrt(2*a*rollout), so a short strip is automatically
                             -- flown slower. Set it to what the craft can REALLY stop at --
                             -- too high and it runs off the far end.
    RWY_AIM_FRAC    = 0.15,  -- touchdown aim point, as a fraction of runway length past the
    RWY_AIM_MAX     = 60,    -- threshold, capped at this many metres.
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
    RWY_CARROT_M    = 400,   -- m: how far ahead the steering point is placed. It sits in the
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
    RWY_GA_SPEED    = 40,    -- m/s cap while flying the circuit. Its legs are hundreds of
                             -- metres, not kilometres, so the en-route cruise cap (which may be
                             -- hundreds of m/s) is not the right number to join one at.
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
    RANGES          = {5, 10, 25, 50, 100, 250, 500, 1000, 2500, 5000},
    DEFAULT_RANGE   = 50,   -- ring scale when there is no target to frame
}

-- Palettes. CC's default colours are too bright/flat for an instrument face, so both
-- screens repaint a few slots (in a pcall -- a non-colour monitor simply keeps its own).
-- HSI.blue is deliberately the SAME HUE as HSI.cyan, well down in brightness: they are the
-- two halves of one course needle, not two different things.
CFG.PALETTE = {
    TERM = { orange = 0xffbb00, brown = 0x553322, green = 0x116611 },
    HSI  = { cyan = 0x33bbdd, orange = 0xffbb00, grey = 0x555555, blue = 0x0e4657 },
}
CFG.HSI_TEXT_SCALE = 0.5   -- the instrument monitor's text scale (smaller = more cells)

return CFG

