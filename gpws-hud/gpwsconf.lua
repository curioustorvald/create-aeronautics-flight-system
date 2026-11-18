-- Aer Togekiss HUD / GPWS -- CONFIGURATION
--
-- Everything the head-up display and ground-proximity warning computer (#67 gpws.lua) can
-- be RETUNED or REWIRED with: rednet protocol names, the height-datum policy, every alert
-- envelope, the voice pack, and the HUD's scales and palette. The program itself holds only
-- logic.
--
-- Same split, and for the same reasons, as the FMS's apconf.lua, the UAC's pid/uacconf.lua
-- and the radar's atcconf.lua:
--
--   * a program update never overwrites this airframe's calibration, and
--   * the knobs are editable without scrolling the program.
--
-- THERE ARE DELIBERATELY NO FALLBACK COPIES OF THESE VALUES IN gpws.lua. A second set of
-- defaults would let an edit here silently do nothing, which is exactly what makes an
-- empirically-flown envelope untrustworthy. A missing or truncated file is a hard BOOT
-- error naming the missing knob (see the canary list in gpws.lua's loader), not a nil
-- arithmetic crash on short final.
--
-- Loaded by `require("gpwsconf")`, which resolves because startup.lua runs
-- shell.run("gpws.lua") and so the program directory is this one. Must end with
-- `return CFG`.

local CFG = {}

-- ============================================================================
-- THE AIRCRAFT
-- ============================================================================

CFG.SHIP = {
    NAME     = "AER TOGEKISS",
    CALLSIGN = "TOGEKISS",

    -- Heading measurement. MUST MATCH this airframe's responder.lua, apconf.lua and
    -- atcconf.lua. When the HUD and the responder bolted to the same hull disagree about
    -- which way the nose points, the pilot is flying one number and the fleet is watching
    -- another. Aer Togekiss's logical forward is 180 degrees from its thrust-forward,
    -- hence the offset.
    HEADING_SIGN   = 1,
    HEADING_OFFSET = math.pi,

    -- THE ALTITUDE DATUM -- how far the pose `sublevel` reports sits ABOVE THE BOTTOM OF THE
    -- HULL, in metres. MUST MATCH this airframe's pid/uacconf.lua, apconf.lua and
    -- responder.lua for the same reason the heading knobs above must: it is one aeroplane,
    -- and two computers bolted to it that disagree about where its feet are will disagree
    -- about how much air is under them.
    --
    -- IT MATTERS MOST ON THIS COMPUTER. ST.ra -- the radio altitude every GPWS envelope is
    -- measured in -- is ST.y minus a datum that is the world Y of a piece of GROUND: a
    -- surveyed pad or runway elevation out of landing.db (SITE), or the terrain server's
    -- profile (TERR). Without this subtraction the clearance over-read by the full height of
    -- the aeroplane on every approach, and a clearance that over-reads is a GPWS that calls
    -- LATE, which is the one failure this computer exists to prevent.
    --
    -- HOW TO MEASURE IT: set it to 0, park on ground of a known Y, read the altitude off the
    -- HUD, and take (what it read) - (the Y the hull rests on). It is a property of the BUILD.
    ALT_OFFSET     = 9,   -- measured 2026-08-29
}

-- ============================================================================
-- LINKS -- who this computer listens to.
--
-- EVERY ONE OF THESE IS RECEIVE-ONLY. The HUD/GPWS transmits nothing to the aircraft; it
-- watches the channels the flight system already broadcasts and reads its own state off
-- sublevel. Nothing it does can move a thruster, which is the point: a warning system that
-- can fly the aeroplane is a second autopilot nobody asked for.
--
-- (The one exception is the nav-data acknowledgement to #24, which is a roster ping, not a
-- command -- exactly as the radar station sends one.)
-- ============================================================================

CFG.LINKS = {
    APSWITCH_NAV = "aertogekiss_apswitch_1",           -- FMS  -> AP-NAV switch state
    APSWITCH_ALT = "aertogekiss_apswitch_2",           -- AS   -> AP-ALT switch state
    ALTAP        = "aertogekiss_altap",                -- AS   -> selected altitude
    AUTOLAND     = "aertogekiss_autoland",             -- FMS  -> vertical channel commands
    APNAV        = "aertogekiss_navap",                -- FMS  -> active target / phase
    ENGINE       = "aertogekiss_engineswitch",         -- #11  -> master/alt/att/extra lift
    TOUCHDOWN    = "aertogekiss_touchdown_detection",  -- #15  -> squat switch
    MANCON       = "aertogekiss_mancon",               -- #6   -> commanded VS (hand-flown)
    NAVDATA      = "aer_navdata",                      -- #24  -> the fleet navigation database
    TERRAIN      = "aer_terrain",                      -- #68 <-> the terrain database
}

-- TERRAIN is the ONE link on this computer that is not purely passive: the aircraft has to
-- ask, because no server can broadcast 2.06 million chunks and no aircraft could hold them.
-- What goes out is a position, a ground track and a request for the ground along it. That
-- is a query, not a command -- there is still nothing this computer can say to anything
-- that moves a thruster.

-- How long a channel may stay quiet before the HUD stops believing it. A stale field is
-- DASHED OUT, never drawn as its last value: a frozen number on a head-up display is worse
-- than no number, because the pilot cannot tell it has stopped moving.
CFG.STALE = {
    FIX      = 0.5,   -- s without a sublevel pose. Everything on the HUD is measured from it.
    APSWITCH = 1.0,   -- announced every 0.1 s by both brains
    ALTAP    = 4.0,   -- announced every 1 s, and only while an altitude is selected
    AUTOLAND = 1.0,   -- streamed every tick during a landing; silence means "no landing"
    APNAV    = 2.0,   -- re-broadcast every CFG.PING_INTERVAL (0.2 s) while a route is active
    ENGINE   = 1.0,   -- announced every 0.1 s
    TOUCHDOWN = 3.0,  -- same figure apconf.lua's CFG.TOUCHDOWN.STALE uses
    MANCON   = 1.0,   -- streamed by the VS stick
}

-- ============================================================================
-- NAVIGATION DATABASE
--
-- Read only, and kept current by subscribing to #24 exactly as an FMS or the radar does.
-- GPWS needs it for ONE thing: the published elevation of the field it is arriving at or
-- departing from. That is the only ground truth this aircraft has -- see CFG.DATUM.
-- ============================================================================

CFG.POINTS_FILE  = "points.db"
CFG.COURSE_FILE  = "course.db"
CFG.LANDING_FILE = "landing.db"

CFG.NAVDATA = {
    ENABLE      = true,
    PULL_EVERY  = 5,    -- s between "send me the set" calls, while we have never synced
    STALE_AFTER = 95,   -- s of silence before the server is called silent (> 3 broadcasts)
    TMP         = ".new",   -- suffix of the staging file each .db is written through
}

-- ============================================================================
-- TERRAIN -- ARMED.
--
-- This aircraft now HAS terrain data, and the modes that need it are live: 2 (terrain
-- closure), 4's true clearance form, and the forward-looking TERRAIN AHEAD / OBSTACLE
-- AHEAD calls. The stub that used to sit here is gone.
--
-- WHERE IT COMES FROM. Computer #68 on the ground, over "aer_terrain". It reads a database
-- built by terrainbuild.jar straight out of the world's region files -- one record per
-- chunk (16 m) and one level per 8 m of Y, over all 2596 generated regions. There is still
-- no radio altimeter and no scanner on this hull; what there is, is a survey.
--
-- WHAT AN ANSWER IS WORTH. Every height in the database is the TOP of its 8 m cell and the
-- HIGHEST ground in its 16 m chunk. It therefore errs HIGH, always, by up to 8 m plus the
-- relief inside one chunk. That is the safe direction -- clearance reads low, calls come
-- early -- but it means terrain height is not a survey of the exact spot under the wheels,
-- and it is why SITE still outranks it as a datum near a field (see CFG.DATUM).
--
-- WHAT HAPPENS WHEN THE LINK DROPS. Everything goes quiet. available() goes false the
-- moment the server has been silent for STALE seconds, and every terrain mode returns to
-- the behaviour it had when this was a stub. A warning computed from a stale profile is
-- exactly the kind of confident wrong answer this system is built to refuse.
-- ============================================================================

CFG.TERRAIN = {
    ENABLE     = true,    -- the master arm for every terrain-derived mode
    -- s of ground track the forward-looking modes scan.
    --
    -- 7 was the flown figure; the extra second pays for CFG.VOICE.MIN_DWELL. The speaker
    -- no longer starts a phrase until the envelope has held for half a second, so every
    -- terrain call is heard that much later than it is computed. Looking a second further
    -- ahead hands the time back with margin, and puts the words in the pilot's ear at the
    -- same range from the ridge as before the dwell gate existed.
    LOOK_AHEAD = 8,

    -- CLEAR_M is the vertical buffer the aeroplane is expected to pass terrain BY. It is
    -- not an alert threshold -- see CFG.GPWS.MODE2 for why that distinction is the whole
    -- fix -- it is the margin the escape calculation aims for.
    CLEAR_M = 20,

    -- s ahead a sample must be before it counts as "ahead" at all. Terrain directly
    -- underneath is not something to be climbed over, it is something already being flown
    -- past, and dividing by a time approaching zero makes every overflight an emergency.
    -- Height over the ground right now is Mode 4's question, not Mode 2's.
    MIN_T = 2.0,

    -- Use the database as a height datum when no field is near. This is what gives the HUD
    -- an RA number en route instead of dashes, and it is the single biggest change arming
    -- terrain makes: modes that used to inhibit themselves over unknown ground now run for
    -- most of a flight. Set false to keep the old behaviour and use terrain for the
    -- forward-looking modes only.
    DATUM      = true,

    -- Freshness. STALE is measured from the last word heard from the server, which beacons
    -- every 15 s, so it must comfortably clear that. PROF_STALE is how old the cached
    -- corridor may be before the forward-looking modes stop believing it.
    STALE      = 45,
    PROF_STALE = 4,

    -- Asking. One corridor request every REQ_EVERY seconds; the answer is a profile the
    -- 20 Hz core tick then re-reads against live altitude, so the alert rate is not tied to
    -- the query rate.
    REQ_EVERY  = 0.5,
    STEP       = 16,      -- m between profile samples -- one chunk, the database's own grid
    WIDTH      = 48,      -- m either side of track the server reduces over. A VTOL that
                          -- holds heading while translating still has the ridge beside it
                          -- charted.
    MIN_RANGE    = 2000,  -- m of profile to ask for even when slow or stopped
    RANGE_MARGIN = 1.5,   -- ask for this much more than LOOK_AHEAD needs, so a profile
                          -- stays usable while the next one is in flight
    MAX_SAMPLES  = 700,   -- ...and never ask for more than the server will serve

    -- Using. MIN_GS is the speed below which the profile is cut along the NOSE rather than
    -- along a ground track that is mostly noise. TRACK_TOL is how far the aircraft may turn
    -- off the profile's track before it is discarded rather than believed; BEHIND is how
    -- far it may drift backwards along it (station-keeping, a hover taxi).
    MIN_GS     = 3,
    TRACK_TOL  = 30,
    BEHIND     = 200,

    -- Obstacles. MIN_MASS ignores the thin class -- one stray block a player left at
    -- altitude is charted honestly by the builder and is not worth a callout.
    --
    -- OBST_CLEAR is how much vertical room counts as CLEARING a floating mass: above its
    -- top by this much, or below its base by this much, and it is not in the way. It
    -- replaced a symmetric +-120 m "near enough" band, which was the wrong shape entirely
    -- -- flying the BZR-WAX arrival at 200 m over islands topping out at 127 m, a full
    -- 73 m above them, that band called OBSTACLE AHEAD at every one of them. The question
    -- is not whether a mass is nearby, it is whether the aeroplane will pass through it.
    MIN_MASS   = 2,
    OBST_CLEAR = 30,

    -- How far ahead the aircraft's CURRENT vertical speed is believed when projecting
    -- clearance. Beyond this the profile is read at present altitude.
    --
    -- This bound is the difference between a prediction and a fantasy. Extrapolating a
    -- steady 3 m/s descent across the full 30 s look-ahead subtracts 90 m from every
    -- clearance in the window, so every ordinary arrival reads as an impending impact and
    -- the annunciator never stops. Ten seconds is about as far as a descent can be assumed
    -- to continue unchanged; past that, aeroplanes level off.
    VS_HORIZON = 10,

    -- How far PAST the active waypoint the forward look still bothers with.
    --
    -- The look-ahead runs along the present ground track, which is a straight line, and the
    -- aeroplane is not going to fly one: on the BZR-WAX arrival the legs are 500-1000 m and
    -- a 30 s look at 120 m/s reaches 3.6 km, straight through high ground the aircraft turns
    -- away from two waypoints earlier. It annunciated at terrain it was never going to
    -- reach. So when the FMS is navigating -- it broadcasts its active target on APNAV, and
    -- this computer already listens -- the look is cut at that waypoint plus this margin.
    --
    -- With no FMS target (hand-flown, or the autopilot off) the full LOOK_AHEAD is used:
    -- with no plan to consult, the current track IS the best available guess.
    LEG_MARGIN = 300,

    -- groundAt() serves out of a small tile cache; one tile is a 32x32-chunk region,
    -- 4 kB on the wire. TILE_EVERY rate-limits the fetches so a flight across unsurveyed
    -- ground cannot turn into a request storm.
    TILE_CACHE = 12,
    TILE_EVERY = 0.25,

    -- Record flag bits, as terrainbuild.jar writes them and terrainconf.lua mirrors them.
    -- All three must agree; they are the wire format.
    FLAG = { MASS = 3, WATER = 4, MULTI = 8, TREES = 16, SPIKE = 32, VOID = 64, NODATA = 128 },
}

-- ============================================================================
-- HEIGHT DATUM -- what "RA" on this HUD actually means.
--
-- There is no radio altimeter on this airframe. The height every GPWS mode is driven by is
-- a GEOMETRIC height above a datum this computer can justify, and the HUD always names
-- which, so the pilot knows what the number is worth:
--
--   SITE -- the published elevation of a landing.db field, used ONLY WHERE THAT ELEVATION
--           IS ACTUALLY THE GROUND: over the pad, or over the runway strip. Surveyed data
--           about the exact patch being landed on, and better than the database there
--           because it is the real surface rather than the highest block in a 16 m chunk.
--   TERR -- the terrain database's ground directly beneath, from #68. Good to the 8 m cell
--           plus whatever relief a chunk hides, and biased HIGH, so RA derived from it reads
--           LOW and every envelope trips early. This is what covers everywhere else.
--   DEP  -- the elevation the wheels last left. Trustworthy near where it was taken and
--           nowhere else, so it expires with DISTANCE rather than with time.
--   nil  -- unsurveyed ground with no field and no latch. RA reads dashes and every
--           height-dependent mode inhibits itself.
--
-- THE FOOTPRINT IS THE FIX, AND IT MATTERS. A published elevation was previously believed
-- anywhere within HELI_RADIUS (600 m) or RWY_RADIUS (4000 m) of the field, and it OUTRANKED
-- the terrain database. On the arrival into WAX -- runway, elevation 116 -- that told the
-- GPWS it had 84 m of clearance for four kilometres, while the aeroplane was 25 m above
-- trees topping 175 m. The error is in the OPTIMISTIC direction, which is the one a GPWS
-- must never make, and it threw away accurate surveyed terrain to make it.
--
-- A field elevation is one number. It describes the tarmac and nothing else. Off the
-- tarmac, the terrain database knows better and now says so.
-- ============================================================================

CFG.DATUM = {
    -- THE FOOTPRINT: where a published elevation IS the ground.
    -- A pad is a square PAD_HALF either side of its point; a runway is a strip RWY_HALF_W
    -- either side of the centreline, running from its threshold to threshold + the published
    -- length, with RWY_OVERRUN of slack at each end. landing.db's point is the THRESHOLD and
    -- its direction is the landing direction -- the same geometry autopilot.lua flies
    -- (see its DIR_VECTORS and LND.endX).
    PAD_HALF    = 16,     -- m: a 32 m square over a helipad
    RWY_HALF_W  = 16,     -- m: a 32 m wide strip along the runway
    RWY_OVERRUN = 32,     -- m of strip beyond each end, for the flare and the rollout

    -- THE ARMING RADIUS: how near a field counts as OPERATING AT it. This is NOT a claim
    -- about elevation -- it is what tells the approach logic and the decision-height callout
    -- that there is a field in play. The runway figure matches apconf.lua's RWY_ARM_M, so
    -- the GPWS arms at the same moment the FMS's approach does.
    HELI_RADIUS = 600,
    RWY_RADIUS  = 4000,

    DEP_RADIUS  = 1500,   -- m from where the departure elevation was latched, beyond which
                          -- the ground it described is no longer under us
    SQUAT_TRUST = 3.0,    -- s the squat switch may be silent and still latch the datum
}

-- ============================================================================
-- GPWS -- the alert envelopes.
--
-- Sink rates are m/s and heights are metres, both POSITIVE numbers here; the program applies
-- the signs. Every envelope was chosen to sit clear of what the flight system itself
-- commands (apconf.lua's LG block: 3.0 m/s on a pad descent, 3.5 m/s on the glideslope,
-- 1.5 m/s at touchdown, 45 deg of planned bank) -- a warning that fires on a normal
-- autoland is a warning the pilot learns to ignore.
-- ============================================================================

CFG.GPWS = {}

-- MINIMUM DISPLAY TIME. Once an envelope trips, its banner stays up at least this long even
-- if the condition has already gone away.
--
-- Two reasons, and the second is the one that made it necessary. A one-tick trip painted a
-- banner for a single 20 Hz frame, which at HUD.REDRAW is a banner nobody sees. And the
-- speaker now refuses to start a phrase for a call shorter than CFG.VOICE.MIN_DWELL -- so
-- without a floor here, a brief trip would produce no word AND no readable banner, and the
-- mode would fire in complete silence. This is the visual half of that pair; keep the two
-- numbers equal unless there is a reason not to.
CFG.GPWS.MIN_SHOW = 0.5

-- Master inhibits. Nothing sounds while any of these hold.
-- ============================================================================
-- VERTICAL RATE FILTER -- what the projection is allowed to believe.
--
-- Aer Togekiss hunts on the vertical: its VS swings -5 to +5 m/s on about a two-second
-- period. Fed raw into the terrain projection that is ruinous -- every negative half-cycle
-- extrapolates to fifty metres of descent over VS_HORIZON and trips TERRAIN, and the
-- positive half-cycle clears it, twice a second.
--
-- THE FIX IS TO ESTIMATE THE RATE FROM ALTITUDE, NOT TO SMOOTH THE REPORTED RATE, and the
-- arithmetic is why. A +-5 m/s oscillation at 2 s displaces the aircraft by only
-- +-5/(2*pi/2) = +-1.6 metres. In the velocity signal the oscillation is the whole signal;
-- in the altitude signal it is a 1.6 m wobble on a flat line. An estimator driven by
-- position therefore rejects it almost for free, where a low-pass on velocity has to trade
-- lag against ripple and lose.
--
-- The estimator is a critically-damped alpha-beta filter -- the steady-state Kalman gains
-- for a constant-velocity model with white acceleration noise -- so it is a Kalman filter
-- with the covariance propagation solved once, in closed form, instead of every tick. One
-- knob: TAU.
--
-- WHAT IT COSTS. Lag, and TAU buys one against the other. Measured on this airframe's
-- -5/+5 m/s two-second swing, sampled at 20 Hz (gpws-test/test_vsfilter.lua):
--
--     TAU    residual in the projection    genuine descent 90% seen
--     0.8            7.0 m                        3.1 s
--     1.0            4.7 m                        3.9 s
--     1.5            2.3 m                        5.9 s      <- chosen
--     2.0            1.3 m                        7.9 s
--     3.0            0.6 m                       11.7 s
--     4.0            0.4 m                       15.6 s
--
-- The residual has to be small against CLEAR_M and the lag has to be small against
-- VS_HORIZON. At 1.5 s the oscillation leaks 2.3 m into a 20 m warning threshold -- under
-- the 3 m hysteresis, so it cannot chatter -- while a descent that genuinely starts is
-- recognised in under six seconds of the ten the projection looks over. Past 2 s the lag
-- starts eating the warning; under 1 s the leak starts eating the threshold.
--
-- Modes 1 and 3 deliberately keep the RAW rate: Mode 1 is *about* instantaneous sink rate
-- and carries its own envelope and hysteresis, and blunting it would blunt the one mode
-- that works with no terrain at all. The filtered rate is for the things that need a
-- SUSTAINED trend -- the terrain projection, and the landing/approach inhibits, where a
-- momentary excursion must not be allowed to drop an inhibit and let a warning escape.
-- ============================================================================

CFG.GPWS.VS_FILTER = {
    TAU   = 1.5,    -- s. See the table above before changing it.
    RESET = 1.0,    -- s of missing fixes after which the estimator restarts from scratch
    JUMP  = 50,     -- m of residual that means the aircraft MOVED rather than drifted
                    -- (a sublevel re-anchor, a teleport) -- restart rather than chase it
}

CFG.GPWS.INHIBIT = {
    ON_GROUND     = true,   -- squat switch made, or the FMS says LANDED
    MASTER_OFF    = true,   -- engine master switch off: the aircraft is parked
    -- The height-based ground test, and the two guards that stop it being a hole in the
    -- system. Below RA_BELOW we are probably on the pad -- but ONLY IF WE ARE ALSO STOPPED.
    -- Without the speed guards, arriving 2 m under a field's published elevation at 300 m/s
    -- would silence the GPWS at exactly the moment it had something to say, which is the
    -- inhibit working precisely backwards. Ground truth (the squat switch, or the FMS's own
    -- touchdown call) needs no such guard and is tested separately.
    RA_BELOW      = 2.0,    -- m: below this we may be on the pad...
    GROUND_GS     = 3.0,    -- m/s: ...but only while going nowhere...
    GROUND_VS     = 0.5,    -- m/s: ...and only while not climbing or sinking
    AFTER_BOOT    = 3.0,    -- s of silence at power-up, so a cold start is not a fanfare
    -- WITH NO HEIGHT DATUM the system is not blind, it is only deaf to height: modes 1, 3, 4
    -- and the decision-height callout each check for a datum and stand down on their own,
    -- while BANK ANGLE and AUTOPILOT DISCONNECT need no height at all and keep working. That
    -- is the whole of a long en-route leg over unsurveyed ground, and it is better company
    -- than silence.
    --
    -- Set this true to shut the system down entirely instead, banner and all, when there is
    -- no datum. It does not invent one either way.
    NO_DATUM      = false,

    -- Descending onto a pad is CLOSING ON TERRAIN on purpose, so the terrain modes stand
    -- down while the FMS is flying a landing (CFG.GPWS.MODE2/FLTA LANDING_INHIBIT). This is
    -- the speed that decides whether it is still a landing: arriving faster than the FMS
    -- would ever command is not a landing, it is an accident in progress, and it is
    -- precisely when the warning has to survive the inhibit.
    --
    -- apconf.lua's descent rates are the reference -- the FMS commands single figures all
    -- the way down and 0.8-1.5 m/s in the flare -- so 6 m/s is comfortably outside anything
    -- a normal arrival produces.
    LANDING_VS    = 6.0,
}

-- MODE 1 -- EXCESSIVE DESCENT RATE. The one mode that works on height and sink rate alone,
-- which is why it is the one that survives having no terrain data.
-- The envelope is a piecewise-linear pair of boundaries against RA. SINK is the caution,
-- PULL is the warning. Interpolated between the rows; held flat outside them.
CFG.GPWS.MODE1 = {
    ENABLE  = true,
    ARM_RA  = 750,    -- m: above this the mode is disarmed (nothing to hit that we know of)
    ENVELOPE = {
        --  ra    sink   pull
        { ra =   3, sink =  3.0, pull =  4.5 },   -- in the flare: FMS commands 0.8-1.5 here
        { ra =  10, sink =  4.0, pull =  6.0 },   -- FLARE_AGL; FMS commands up to 3.0
        { ra =  30, sink =  5.0, pull =  8.0 },   -- RWY_MIN_AGL; FMS commands up to 3.5
        { ra = 100, sink =  8.0, pull = 13.0 },
        { ra = 300, sink = 14.0, pull = 22.0 },
        { ra = 750, sink = 22.0, pull = 34.0 },
    },
    -- The caution takes SINK RATE and the warning WHOOP WHOOP PULL UP. See CFG.VOICE.CLIPS.
    HYSTERESIS = 0.5,   -- m/s the sink rate must recover by before the alert clears

    -- Which vertical rate this mode judges: the estimated trend, or the raw reported rate.
    --
    -- TRUE (the trend) is the default, and on this airframe it is not a close call. The VS
    -- swings -5/+5 m/s on a two-second period; judged raw, that peaks straight through the
    -- envelope and SINK RATE fires on every negative half-cycle -- five separate calls in
    -- one arrival, for an aeroplane that is not descending at all. An aircraft oscillating
    -- about zero is not sinking, and a mode that says otherwise is the kind of mode pilots
    -- switch off.
    --
    -- The cost is real and worth stating: the estimate reaches half of a genuine new sink
    -- rate in about 2.5 s (see VS_FILTER). A developing dive is therefore called a couple of
    -- seconds later than it would be off the raw rate. Set FALSE on an airframe whose
    -- vertical channel is steady, where that lag buys nothing.
    USE_FILTERED = true,
}

-- MODE 2 -- EXCESSIVE TERRAIN CLOSURE.
--
-- CLOSURE is the whole word. This mode is not "am I low", it is "is the gap shutting"; the
-- absolute-height question belongs to Mode 4. Flown level over flat ground at 40 m nothing
-- is closing, however uncomfortable the number looks, and Mode 2 must say nothing -- an
-- earlier build of this compared clearance against the thresholds alone and called TERRAIN
-- TERRAIN PULL UP for the whole of a low cruise, which is how a warning gets ignored.
--
-- The closure rate comes off the terrain profile: project the aircraft forward along its
-- ground track at its current vertical speed, find the least clearance it would have, and
-- divide the clearance lost by the time to get there. That one number covers both ways of
-- closing on the ground -- sinking towards it, and flying level into rising terrain -- and
-- with no ground speed it degrades cleanly to the descent rate alone, which is right for a
-- VTOL coming straight down.
-- MODE 2 asks ONE question: COULD I CLEAR IT IF I TRIED?
--
-- Not "is there terrain within N metres of me", which is what it used to ask and what made
-- it useless. Cruising at 200 m over ridges topping 175 m the clearance is 25 m for the
-- whole leg -- so a 30 m threshold is true at every range, at 30 s out as much as at 2 s,
-- and the mode annunciated continuously over ground the aeroplane was comfortably clearing.
-- Range never entered into it, and neither did whether the aircraft was in any danger.
--
-- The question it asks now is the climb rate that would be needed to cross the worst terrain
-- in the look-ahead with CLEAR_M to spare:
--
--     required = (terrain_top + CLEAR_M - projected_altitude) / seconds_until_reaching_it
--
-- and it alerts on THAT, in m/s. Everything falls out of it correctly:
--
--   * Level at 200 over a 175 m ridge: required is NEGATIVE at every range -- the aeroplane
--     is already clearing it. Silent, which is the reported complaint, fixed at the root.
--   * The same ridge 30 s away needs a fifth of a m/s; 3 s away it needs metres per second.
--     Range stops being something the mode has to be told about: it is in the divisor.
--   * Terrain 50 m ABOVE the aircraft 15 s out needs 4.7 m/s, more than CLIMB_MAX. That is
--     a genuine warning and it fires.
--
-- The two levels are climb rates, and meaningful ones: CAUTION is a rate a gentle correction
-- fixes, WARNING is a rate approaching everything the airframe has (CLIMB_MAX is 3.75 m/s on
-- this aeroplane), which is what "you must act now" actually means.
--
-- The projected altitude carries the vertical speed as far as VS_HORIZON, so descending
-- raises the required climb -- correctly, since a descent must be arrested before it can be
-- reversed. Arresting a descent onto flat ground is Mode 1's job, not this one's: this mode
-- is purely forward-looking now, and MIN_T is what keeps it that way.
CFG.GPWS.MODE2 = {
    ENABLE          = true,
    ARM_RA          = 600,   -- m of terrain clearance above which there is nothing to hit
    ESCAPE_CAUTION  = 1.5,   -- m/s of required climb -> TERRAIN
    ESCAPE_WARNING  = 3.0,   -- m/s -> TERRAIN TERRAIN PULL UP
    HYSTERESIS      = 0.5,   -- m/s the requirement must ease by before the call stops
    LANDING_INHIBIT = true,  -- stand down for a normal landing (see INHIBIT.LANDING_VS)
}

-- FORWARD-LOOKING TERRAIN AND OBSTACLE AVOIDANCE (FLTA) -- the advisory ahead of Mode 2.
--
-- The same escape calculation at a gentler rate: it speaks when a correction is merely
-- ADVISABLE, where Mode 2 speaks when one is required. That gives a ladder -- TERRAIN AHEAD,
-- then TERRAIN, then TERRAIN TERRAIN PULL UP -- driven by one number getting worse, instead
-- of three modes arguing about the same piece of ground.
--
-- "AHEAD" HAS TO MEAN SOMETHING, so both calls are disarmed below MIN_GS. A hovering VTOL is
-- not about to fly into anything, and it matters more on this fleet than most: a VTOL
-- routinely holds heading while barely moving, and the terrain client cuts its profile along
-- the NOSE at low speed, so "ahead" would otherwise mean "wherever we happen to be pointing".
CFG.GPWS.FLTA = {
    ENABLE          = true,
    MIN_GS          = 10.0,  -- m/s below which neither call is armed
    ESCAPE          = 1.0,   -- m/s of required climb -> TERRAIN AHEAD / OBSTACLE AHEAD
    HYSTERESIS      = 0.4,   -- m/s of easing before the call stops
    LANDING_INHIBIT = true,
}

-- MODE 3 -- ALTITUDE LOSS AFTER TAKE-OFF ("DON'T SINK").
-- Armed from lift-off until the aircraft is established, and trips when the height given
-- away since the post-lift-off peak exceeds a fraction of the height gained.
--
-- IT IS ARMED BY A TAKE-OFF AND BY NOTHING ELSE, and that latch is the whole mode: without
-- it, "height below the highest height reached recently" describes every descent ever flown,
-- and the system says DON'T SINK the whole way down every approach.
--
-- MEASURED GEOMETRICALLY, against the altitude the wheels left -- not in RA. This mode has
-- always claimed to be terrain-free, and its reference has always been the departure
-- elevation, but the implementation read RA -- and once terrain became a height datum, RA
-- stopped being that. RA now moves for reasons the aeroplane had nothing to do with: the
-- terrain database steps in 8 m cells, so crossing a chunk boundary can drop RA by 8 m in a
-- single tick -- twice LOSS_MIN -- and the datum switching from TERR to SITE on entering a
-- field's radius jumps it by the difference between the surveyed field elevation and the
-- charted ground under the aircraft. Neither is altitude anybody lost. Reading sublevel's
-- own y against the lift-off altitude is immune to both, and is what the comment above has
-- said all along.
--
-- LANDING_INHIBIT is the other half, and the reported fault. A short VTOL hop -- under
-- WINDOW seconds, never ARM_GAIN above the pad -- leaves this mode armed for the whole
-- flight, so the descent onto the destination pad at 3 m/s read as altitude loss after
-- take-off and rang continuously. A commanded arrival is not a mistake. Mode 4 has always
-- known that (see its APPROACH_* note); Mode 3 was simply never told.
CFG.GPWS.MODE3 = {
    ENABLE    = true,
    ARM_GAIN  = 150,   -- m climbed above the lift-off elevation that ends the take-off
    WINDOW    = 60,    -- s airborne after which it disarms anyway -- a low-level cruise that
                       -- never climbs ARM_GAIN must not leave it armed for the whole flight
    LOSS_FRAC = 0.10,  -- of the height gained since lift-off
    LOSS_MIN  = 4.0,   -- m: ...but never trip on less than this, however small the climb
    CLEAR_ON_CLIMB = 0.5,   -- m/s of climb that re-arms the peak and clears the alert
    LANDING_INHIBIT = true, -- a landing the FMS is flying, or an approach to a field whose
                            -- elevation we have, is not altitude loss after take-off
}

-- MODE 4 -- UNSAFE TERRAIN CLEARANCE ("TOO LOW").
-- The true Mode 4 compares radio altitude with terrain and with the landing configuration;
-- terrain form now has terrain behind it, but the CONFIGURATION half -- gear, flaps, a
-- landing selected -- this airframe does not have. What runs here is the datum part:
-- flying FAST and LOW over a field we have an elevation for, with no landing in progress.
-- The floor rises with ground speed -- 15 m is unremarkable in a hover and reckless at
-- 300 m/s. Interpolated between rows, held flat outside them.
CFG.GPWS.MODE4 = {
    ENABLE = true,
    -- TAKE-OFF INHIBIT, and it is not optional: every departure begins by flying low and
    -- accelerating, which is the exact shape of the condition below. The mode stays disarmed
    -- from lift-off until the aircraft has climbed through ARM_RA once, after which it
    -- protects the rest of the flight. Landing re-disarms it. This is what a real Mode 4
    -- does and it is why it does not shout at every take-off roll.
    ARM_RA = 100,   -- m the aircraft must reach after lift-off before the mode arms
    FLOOR = {
        --  gs (m/s)   ra (m)
        { gs =  15, ra =  15 },
        { gs =  40, ra =  25 },
        { gs = 120, ra =  60 },
        { gs = 320, ra = 120 },
    },
    MIN_GS      = 15,    -- m/s below which the mode does not apply: a low hover is a hover
    HYSTERESIS  = 5.0,   -- m of climb needed to clear the alert

    -- THE APPROACH INHIBIT. Being low over a field is what arriving looks like, so the mode
    -- has to be able to tell an arrival from a mistake. A real Mode 4 asks the gear and the
    -- flaps; this airframe has neither, so it asks the geometry: descending at a SANE rate
    -- toward a field whose elevation we actually have. That covers an FMS autoland and a
    -- hand-flown circuit alike -- which matters, because the autoland stream is only there
    -- for the first of those, and a mode that cries wolf on every manual landing is a mode
    -- the pilot switches off.
    --
    -- What it deliberately does NOT cover: fast and low with no descent (flying INTO the
    -- field rather than onto it), and a steep dive at low height. Both stay armed.
    APPROACH_VS = 4.0,   -- m/s: the steepest descent still read as an approach. Just above
                         -- apconf.lua's RWY_VS_MAX (3.5) and LG.DESCEND_FAST (3.0).
    APPROACH_MIN_VS = 0.3, -- m/s: ...and the shallowest. LEVEL FLIGHT IS NOT AN APPROACH --
                         -- without this floor, beating across a field at 60 m/s and 20 m
                         -- would inhibit itself simply by being near an aerodrome, which is
                         -- the one case the mode most needs to keep.
    APPROACH_NEAR = 400, -- m from the field inside which closure is not required -- a VTOL
                         -- over its pad is arriving even when it is going nowhere
}

-- MODE 6 -- CALLOUTS.
-- DECISION HEIGHT is the one the pack speaks. One shot per approach, on the way down;
-- re-armed by climbing back through DH + RESET.
-- The callout is ARMED BY CLIMBING THROUGH DH + RESET, and starts each flight disarmed.
-- Without that, a take-off that sags a few metres gets a DECISION HEIGHT call at 20 m, which
-- is both wrong and the sort of wrong that teaches a pilot to stop listening.
CFG.GPWS.MODE6 = {
    ENABLE   = true,
    DH       = 30,     -- m RA. Same number as apconf.lua's RWY_MIN_AGL, the approach minimum.
    RESET    = 15,     -- m above DH the aircraft must reach before the callout arms
    REQUIRE_DESCENT = true,  -- only call it while actually descending

    -- ...and only where a decision is actually being made. DECISION HEIGHT is a callout for
    -- the point on FINAL where the approach is continued or abandoned; anywhere else it is
    -- noise. This became load-bearing the moment terrain became a height datum: RA used to
    -- read dashes over open country, so the callout physically could not fire there, and now
    -- RA exists everywhere.
    --
    -- "ON FINAL" is taken from the FMS broadcasting a LANDING TARGET -- the same signal the
    -- HSI dispatches its landing display and glideslope pointer on. Two looser tests were
    -- tried first and both rang when they should not have:
    --
    --   "the autoland channel is live"  -- true for the whole of a GO-AROUND, which keeps
    --      the vertical channel while the route machinery flies the circuit.
    --   "near a published field"        -- true throughout an APPROACH PATTERN, whose fixes
    --      are ordinary route legs inside the 4 km arming radius.
    --
    -- Set false to go back to firing on any descent through DH with a datum, which is what
    -- a hand-flown approach would need -- there is no glideslope to be on the centreline of
    -- when nobody is flying one.
    REQUIRE_LANDING = true,

    -- BANK ANGLE. Datum-free, so it works everywhere -- including en route with no datum at
    -- all. The pack has no bank clip, so it is annunciated on the HUD only.
    BANK_ENABLE  = true,
    BANK_CAUTION = 55,   -- deg. The planner's DC.MAX_BANK_DEG is 45; leave it room.
    BANK_WARNING = 70,   -- deg
    BANK_HYST    = 5,    -- deg of recovery before it clears
}

-- MODE 7 -- WINDSHEAR. STUB: there is no wind model to detect a shear against.
CFG.GPWS.MODE7 = { ENABLE = false }

-- AUTOPILOT DISCONNECT. Not a GPWS mode at all -- it is the other thing this speaker is for.
-- Sounds once on either AP switch going from engaged to disengaged, and once if an engaged
-- autopilot's channel goes silent (a dead brain announces nothing, which looks identical to
-- a disengagement from here and is at least as worth knowing about).
CFG.GPWS.APDISC = {
    ENABLE      = true,
    ON_SILENCE  = true,   -- treat a stale APSWITCH channel as a disconnect
    LATCH       = 5.0,    -- s the banner stays up after the call
    INHIBIT_ON_GROUND = true,   -- switching the autopilot off on the pad is not an event
}

-- ============================================================================
-- VOICE PACK
--
-- The clips live on a floppy in the drive, not on this computer: the pack is shared fleet
-- equipment and swapping the disk swaps the voice. The drive is found by looking for
-- MARKER on it, not by peripheral name -- with more than one drive attached, the one with
-- the voice pack is the one with the manifest on it.
--
-- File names are LANG .. "_" .. stem .. ".bin", e.g. uk_whoop_whoop_pull_up.bin.
--
-- THE FILES ARE RAW PCM: mono, UNSIGNED 8-BIT, 48 kHz, no header. One byte is one sample,
-- so the program subtracts 128 and hands the result straight to speaker.playAudio. The pack
-- was DFPWM until 2026-08-26; the 1:8 saving bought nothing on a floppy with room to spare
-- and cost a Lua decoder running bit arithmetic per sample on the computer that must not be
-- busy when the ground comes up. Re-encode with ffmpeg:
--
--     ffmpeg -i word.wav -ac 1 -ar 48000 -f u8 uk_word.bin
--
-- AN ALERT'S VOICE IS A LIST OF STEMS, NOT ONE FILE. The pack's manifest is explicit that
-- some words are MODIFIERS rather than messages: "TOO LOW" is not something the system ever
-- says on its own -- it qualifies the word that follows, so TOO LOW--TERRAIN is
-- too_low.bin then terrain.bin, spoken back to back. The same composition is what builds
-- the real Mode 2 and FLTA warnings out of the words in the pack.
--
-- The clips of one alert are pushed into the speaker as a single stream, so they run
-- together as one phrase rather than as two announcements with a seam between them. An alert
-- is only speakable if EVERY stem in its list is on the disk -- half a phrase is a different
-- phrase, and "TOO LOW" with the qualifier missing is exactly the sentence the manifest says
-- the pack does not contain.
--
-- NOTE ON PACKS: the manifest lists 'bt' (Betty, Boeing) and 'gb' (Honeywell) beside 'uk'
-- (Rita, Antonov), and marks all three as copyrighted material that MUST NOT go into the
-- distribution package. Only LANG changes here to switch voice; nothing about the clips
-- themselves belongs in create-aeronautics-flight-system.
-- ============================================================================

CFG.VOICE = {
    ENABLE = true,
    LANG   = "uk",              -- two-letter voice-actor prefix. 'uk' is Rita.
    DIR    = nil,               -- force a directory (e.g. "/disk"); nil = find it by MARKER
    MARKER = "_voicepack.md",   -- the manifest file that identifies a voice-pack disk
    VOLUME = 3.0,               -- speaker.playAudio volume, 0.0 .. 3.0. A warning is loud.
    CHUNK  = 16 * 1024,         -- bytes read and pushed per go. The files are one byte per
                                -- sample, so this is also 16384 samples -- a third of a
                                -- second at RATE, comfortably inside playAudio's 128 Ki
                                -- ceiling, and the granularity at which a higher-priority
                                -- warning can cut this one off mid-word. See
                                -- https://tweaked.cc/guide/speaker_audio.html
    RATE   = 48000,             -- samples/s the speaker plays at. Used to time a clip's end.

    -- MINIMUM DWELL BEFORE THE SPEAKER IS TOUCHED AT ALL.
    --
    -- An envelope that trips and clears again inside a few ticks used to start a clip and
    -- stop it immediately, and CC does something worse than nothing with that: the clip is
    -- not audible, but it IS queued, and it then plays AHEAD of the next clip that is
    -- genuinely commanded. One flickering mode therefore made the NEXT warning say the
    -- wrong word -- SINK RATE arriving where PULL UP was ordered.
    --
    -- So an alert has to have held for this long before AUDIO.want() will look at it. It
    -- costs half a second on every terrain call, which CFG.TERRAIN.LOOK_AHEAD pays back.
    --
    -- LATCHED ONE-SHOTS ARE EXEMPT and speak at once: DECISION HEIGHT and AUTOPILOT
    -- DISCONNECTED are events, not conditions, and their banner latch has already committed
    -- them for seconds -- there is nothing to wait and see about.
    --
    -- Pairs with CFG.GPWS.MIN_SHOW, which keeps the banner up for the same half second so a
    -- gated call is at least seen. Keep the two equal.
    MIN_DWELL = 0.5,

    -- alert id -> the stems that make up its phrase, in the order they are spoken. An id
    -- with no entry here is annunciated on the HUD in silence, which is the correct
    -- behaviour when the pack has no word for it: a wrong word is worse than none. KEEP EACH
    -- LIST'S WORDING IN STEP WITH THAT ALERT'S `text` IN CFG.ALERTS -- the banner and the
    -- speaker saying different things is its own kind of failure.
    --
    -- THE PACK IS MOVING TO SELF-CONTAINED PHRASES: one alert, one file, as _voicepack.md
    -- now states. Composition was how TOO LOW TERRAIN used to be built, from a "too low"
    -- modifier and a "terrain" hazard spoken back to back; that pair is gone from the disk
    -- and the single uk_too_low_terrain replaces it. A phrase recorded whole is better
    -- anyway -- the join between two clips is audible, and the intonation across a real
    -- phrase is not the intonation of two words read separately.
    --
    -- The mechanism stays because TERRAIN TERRAIN PULL UP still needs it: the pack has
    -- `terrain_terrain` and `whoop_whoop_pull_up` but no file holding both. Record one and
    -- this list becomes single-stem throughout.
    CLIPS = {
        pull_up        = { "whoop_whoop_pull_up" },
        -- The full Mode 2 warning: two words and a command, not one word repeated. The only
        -- composed phrase left.
        terrain_hard   = { "terrain_terrain", "whoop_whoop_pull_up" },
        terrain        = { "terrain" },
        terrain_ahead  = { "terrain_ahead" },
        obstacle_ahead = { "obstacle_ahead" },
        -- MODE 4. One file: "too low" alone names no hazard and is never spoken alone.
        too_low        = { "too_low_terrain" },
        sink_rate      = { "sink_rate" },
        dont_sink      = { "dont_sink" },
        ap_disconnect  = { "ap_disconnected" },
        decision       = { "decision_height" },
    },
}

-- ============================================================================
-- THE ALERT TABLE -- priority, wording and repeat behaviour, in one place.
--
-- PRIO is the whole arbitration: the highest-priority ACTIVE alert is the one on the banner
-- and the one in the speaker, and a higher one interrupts a lower one mid-word. Alerts of
-- equal priority never fight because no two of them can be active at once.
--
-- LEVEL drives the colour: "warn" is red and reverse video, "caut" amber, "info" green.
-- LOOP alerts re-speak every GAP seconds for as long as the condition holds; one-shot
-- alerts speak once and latch the banner for LATCH seconds.
-- ============================================================================

CFG.ALERTS = {
    { id = "pull_up",        prio = 100, level = "warn", text = "PULL UP",
      loop = true,  gap = 0.10, mode = "1/2" },
    { id = "terrain_hard",   prio =  95, level = "warn", text = "TERRAIN TERRAIN PULL UP",
      loop = true,  gap = 0.40, mode = "2" },
    { id = "terrain",        prio =  90, level = "warn", text = "TERRAIN",
      loop = true,  gap = 0.80, mode = "2" },
    { id = "ap_disconnect",  prio =  85, level = "warn", text = "AUTOPILOT DISCONNECTED",
      loop = false, latch = 5.0, mode = "AP" },
    { id = "terrain_ahead",  prio =  70, level = "caut", text = "TERRAIN AHEAD",
      loop = true,  gap = 2.00, mode = "FLTA" },
    { id = "obstacle_ahead", prio =  65, level = "caut", text = "OBSTACLE AHEAD",
      loop = true,  gap = 2.00, mode = "FLTA" },
    -- "TOO LOW TERRAIN" is the standard Mode 4 wording and it is what the pack composes; it
    -- is a statement about CLEARANCE, not a claim that a terrain database exists. The height
    -- behind it is still the datum height -- see the DATUM block.
    { id = "too_low",        prio =  60, level = "caut", text = "TOO LOW TERRAIN",
      loop = true,  gap = 1.50, mode = "4" },
    { id = "dont_sink",      prio =  55, level = "caut", text = "DON'T SINK",
      loop = true,  gap = 1.50, mode = "3" },
    { id = "sink_rate",      prio =  50, level = "caut", text = "SINK RATE",
      loop = true,  gap = 1.50, mode = "1" },
    { id = "bank_angle",     prio =  45, level = "caut", text = "BANK ANGLE",
      loop = true,  gap = 2.00, mode = "6" },   -- no clip in the pack: HUD only
    { id = "decision",       prio =  40, level = "info", text = "DECISION HEIGHT",
      loop = false, latch = 4.0, mode = "6" },
}

-- ============================================================================
-- DEBUG -- which mode is shouting, and what it was just shouting about.
--
-- The banner can only ever show ONE alert: the highest-priority one showing. That is right
-- for flying and useless for diagnosis, because TERRAIN on the banner could be Mode 2
-- tripping on closure or FLTA tripping on clearance, and three modes are routinely up with
-- only the loudest visible.
--
-- With this on, the WARNING LINE ITSELF carries the rest -- not a second line somewhere
-- else, because the warning line is where the eye already is:
--
--     TERRAIN TERRAIN PULL UP  [2 FLTA 1]  clr-55  cls6.0  lk30s
--
-- The bracketed list is every mode currently showing, in priority order, written exactly as
-- the `mode` field of CFG.ALERTS writes it -- 2, 4, FLTA, AP, 1/2 -- with the first being
-- the mode behind the text on the banner.
--
-- AND WHEN IT GOES QUIET the same line keeps the answer, because a GPWS says its piece and
-- stops, and "what was that" is a question asked several seconds after the banner cleared:
--
--     [ TOGEKISS ]  last TERRAIN TERRAIN PULL UP [2] 7s  clr112  cls-0.4
--
-- The recall is drawn in the level colour of the alert it remembers. That cannot be
-- confused with a live warning: a live one fills the row in reverse video, the quiet rule is
-- a dashed line. An inhibited system names its inhibit here too, which answers the other
-- half of the question -- why a mode did NOT fire.
--
-- Segments shed from the RIGHT as the monitor narrows, so the alert text goes last and the
-- numbers go first.
--
-- ON_KEY toggles it from the computer's keyboard in flight. That matters more than it looks:
-- nothing in this tree can be tested in-game without flying it, and the alternative to a key
-- is editing this file and rebooting the computer mid-sortie to read one number.
-- ============================================================================

CFG.DEBUG = {
    ENABLE     = false,   -- state at boot
    WHEN_QUIET = true,    -- keep the recall/inhibit/numbers on the quiet rule as well
    ON_KEY     = "d",     -- runtime toggle; false for no key. SELFTEST has "t".
}

-- ============================================================================
-- THE HUD.
--
-- The monitor is mounted ABOVE THE PILOT'S EYE LINE, so the layout is upside down compared
-- with a panel display: the LOWEST rows are the ones nearest the natural line of sight and
-- they carry the most important symbology. Reading UP from the bottom edge --
--
--     alert banner  ->  speed/height/sink  ->  heading index + scale
--     ->  autopilot annunciators  ->  attitude field  ->  route
--
-- -- so a glance that barely leaves the windscreen gets the warning and the primary numbers.
--
-- THE BOTTOM ROW CARRIES NOTHING THAT MATTERS. A warning fills it edge to edge, so anything
-- informative parked there is information the pilot loses at exactly the moment things are
-- going wrong. When the system is quiet it is an ornamental frame rule and no more.
--
-- WHAT IS NOT ON THE MONITOR: the self-status line (peripherals, voice pack, nav-data
-- revision, uptime) and the GPWS armed/inhibited line. Both are on the COMPUTER'S OWN
-- SCREEN. Neither is flown by -- they are read on the ground, or after something has already
-- gone wrong -- and a head-up display is not the place to keep housekeeping.
--
-- Rows are dropped from the TOP as the monitor gets shorter; the banner and the primary strip
-- are never dropped, and the heading indicator is two rows or none (an index caret with no
-- scale under it points at nothing). A monitor of any size shows a correct HUD, just less
-- of one.
--
-- BACKGROUND COLOUR is spent on exactly two things: the heading indicator and the bank scale.
-- Both are rows of small ticks that vanish into a display which is already all small green
-- ticks, and a filled band is what makes each read as an instrument. The artificial horizon
-- deliberately gets none -- filling it would turn most of the screen into a lamp and put the
-- symbology inside it.
-- ============================================================================

CFG.HUD = {
    MONITOR_NAME = nil,    -- force one (e.g. "monitor_1"); nil = the first one found
    TEXT_SCALE   = 0.5,    -- Aer Togekiss's HUD measures 57 x 10 characters at this scale.
                           -- CHANGING IT CHANGES EVERY SCALE BELOW: the program reads
                           -- mon.getSize() and lays itself out to whatever it finds, so it
                           -- will not break, but FOV_DEG / LADDER_STEP / HDG_PER_COL were
                           -- chosen against those 57 columns and 10 rows and would want
                           -- re-picking for a bigger sheet of glass.
    REDRAW       = 0.05,   -- s between repaints. 20 Hz, the same rate the state arrives at.
    TERM_REDRAW  = 0.5,    -- s between repaints of the computer's own status page

    -- SCALES, sized for the monitor this aircraft actually carries: 57 x 10 characters.
    -- That is a SHORT, NARROW HUD -- after the banner, the primary strip, the two heading
    -- rows and the annunciators there are four rows left for the attitude field -- and every
    -- number below follows from it. Re-tune all four together if the glass ever changes.
    FOV_DEG      = 30,     -- deg of pitch across the full height of the attitude field. 40
                           -- over four rows put a 10-degree pitch change inside one row;
                           -- 30 is as much sky as this many rows can usefully show.
    LADDER_STEP  = 10,     -- deg between pitch-ladder rungs. At 5 the rungs landed on top of
                           -- each other -- see the auto-suppress in HUD.drawAttitude, which
                           -- drops the ladder entirely rather than draw an illegible one.
    HDG_PER_COL  = 1.0,    -- deg of heading per column. 57 columns therefore span +-28 deg,
                           -- and a graduation every 10 deg falls every 10 columns -- room for
                           -- a two-digit number with eight columns of air around it. At the
                           -- old 0.5 the span was 28 deg total and there was seldom more than
                           -- one number on the tape at a time.
    CELL_ASPECT  = 1.5,    -- a character is this many times taller than it is wide. Sets the
                           -- horizon's slope and the flight-path vector's lateral scale.

    -- Attitude signs. Flip if the horizon tilts or climbs the wrong way in the air. These
    -- are the values #7 altcom.lua's attitude indicator was flown with on this airframe --
    -- both displays read the same quaternion and must agree.
    PITCH_SIGN = -1,
    ROLL_SIGN  =  1,

    SHOW_FPV     = true,   -- the flight-path vector. On a VTOL it routinely sits well off to
                           -- the side of the nose -- that is the aircraft, not the symbol.
    SHOW_LADDER  = true,
    SHOW_HORIZON = true,

    -- A HUD is drawn in one colour on nothing, not painted like a panel. Only the cautions
    -- and warnings break out of green, which is what makes them impossible to miss.
    --
    -- The two BAND colours are the exception, and they are deliberately dark: they have to
    -- lift the heading indicator and the bank scale off the background without competing
    -- with the symbology drawn ON them. Green-black rather than neutral, so a band reads as
    -- part of the HUD and not as a window cut into it.
    PALETTE = {
        lime      = 0x33ff66,   -- primary symbology
        green     = 0x117a33,   -- ladder rungs, tape ticks, secondary structure
        lightGrey = 0x8ab89a,   -- labels and units
        grey      = 0x3a4a40,   -- inactive annunciators
        orange    = 0xffaa00,   -- caution
        red       = 0xff3333,   -- warning
        cyan      = 0x66ddff,   -- selected/commanded values (bugs, targets)
        white     = 0xffffff,
        blue      = 0x0a2618,   -- BAND: heading scale, bank scale
        brown     = 0x14472c,   -- BAND: the heading index row, a shade up from the scale
    },
}

-- Tapping the HUD (or pressing T at the computer) runs a SELF TEST: every clip in the pack
-- is played in priority order with its banner up, which is the only way to find out that a
-- disk is missing, a file is corrupt or the speaker has been unbolted -- before the ground
-- is the thing that tells you.
CFG.SELFTEST = {
    ON_TOUCH = true,
    ON_KEY   = "t",
    GAP      = 0.25,   -- s between clips
}

return CFG
