-- ============================================================================
-- REFERENCE ONLY -- the vector-nozzle (TRIANGULAR) allocator's section of a uacconf.lua.
--
-- This file is NOT loaded by anything. The attitude-controller's example
-- pid/uacconf.lua is a conventional (thruster-bank) airframe and carries none of the TRI_*
-- knobs -- absent is a valid answer and means "this aircraft has axis thruster banks".
-- To fly an airframe whose lift/pitch/roll/yaw/strafe come out of three vectoring engines,
-- paste the block below into THAT airframe's pid/uacconf.lua, add
--
--     CFG.TRI_PROTOCOL = 'aertogekiss_vector'   -- out: UAC -> all three engine computers
--
-- beside the other protocol names (it must match PROTOCOL in vecengine.lua), set
-- CFG.TRI_ENABLE = true, and then measure every number below on your own aircraft.
--
-- The values are the ones the author's three-nozzle testbed flew with before it was
-- converted back to thruster banks (2026-08-28), which is why TRI_ENABLE reads false. They
-- are a starting point and a record of the reasoning, not a calibration: the engine
-- positions, the per-step authorities and every sign are properties of one particular hull.
-- Computer numbers in the comments (#72, #77...) refer to that testbed.
-- ============================================================================

-- ============================================================================
-- TRIANGULAR THRUST ALLOCATION -- Scout's three vectoring nozzles
-- ============================================================================
-- SCOUT IS NOT BUILT LIKE THE REST OF THE FLEET. Every other aircraft here has a bank of
-- thrusters per axis: a pitch/roll pair, a yaw pair, a strafe pair, an up/down pair. Scout
-- has THREE down-firing engines on vectoring mounts, in a triangle -- front-centre,
-- back-left, back-right -- and those three make pitch, roll, yaw, strafe AND lift between
-- them. Only fore/aft is still a bank of its own (#79, on CFG.NAVCTRL_PROTOCOL).
--
-- Each engine's control computer takes three numbers: gimbal X (-15..15), gimbal Y
-- (-15..15) and thrust (0..15). So the UAC does not send five axis commands to five relays;
-- it sends three FORCE VECTORS to three engines. The maths that turns one into the other is
-- sendVector() in uac.lua -- read its header before touching anything below, because the
-- knobs here only make sense against the model it describes.
--
-- SETTING THIS UP, IN ORDER. Do not skip ahead; every step after the first is meaningless
-- until the one before it is right.
--   1. GEOMETRY (TRI_ENGINES). Measure it. Everything else is built on it, and a wrong
--      arm shows up as an axis that will not tune.
--   2. TRI_VEC_X_SIGN / TRI_VEC_Y_SIGN / TRI_VEC_SWAP, and TRI_VEC_MAX_ANGLE. Hover the
--      craft, command a pure strafe, and watch WHICH WAY THE NOZZLES POINT. Getting this
--      wrong makes the strafe and yaw channels run away; getting it wrong by 90 degrees
--      (SWAP) makes a strafe command yaw the craft instead.
--   3. TRI_PITCH_SIGN / TRI_ROLL_SIGN / TRI_YAW_SIGN / TRI_LAT_SIGN. On the ground, engines
--      live, command one axis at a time and watch WHICH ENGINE SPOOLS UP. A nose-up demand
--      that spools the FRONT engine up is backwards.
--   4. The *_PER_STEP authorities. These are the loop gains of the whole aircraft, and they
--      are the last thing to touch, not the first.
CFG.TRI_ENABLE = false

-- WHERE THE THREE ENGINES ARE, in metres FROM THE CENTRE OF MASS, in the body frame the
-- attitude loop already uses:  x = starboard (+right),  y = up,  z = nose (+forward).
--
-- MEASURE THESE. The numbers below are the layout as drawn, not as flown -- they set every
-- arm in the allocation, and an arm that is wrong by a factor of two is a gain that is wrong
-- by a factor of two on that axis alone (which reads as "pitch tunes fine but roll never
-- settles", the most confusing failure this file can produce).
--
--   key  the field name this engine's values travel under on TRI_PROTOCOL. Two characters,
--        because the display packs all three engines onto one 51-column line. The engine's
--        own control computer (#77 / #78 / #80) hardcodes the matching key, exactly as every
--        other relay on this fleet hardcodes its protocol name.
--   y    height of the mount relative to the CoM. Positive = ABOVE the centre of mass; for
--        down-firing engines slung under a hull it is NEGATIVE.
--
--        **THIS IS NOT A REFINEMENT. GET IT ROUGHLY RIGHT.** An earlier version of this note
--        said to leave it 0 unless measured, on the grounds that 0 makes the correction
--        "cleanly inert rather than approximately wrong". That was wrong, and it cost a flight
--        test. 0 is not inert: it tells the allocator the sideways thrust makes no moment, and
--        the sideways thrust makes a moment regardless. Measured on the bench with the mounts
--        1.5 m below the CoM and a full strafe demand -- no pitch, roll or yaw commanded:
--
--            declared y = 0.0   ->  lateral force 6.00,  ROLL MOMENT +9.00  (37.5% of the
--                                   craft's entire roll authority, uncommanded)
--            declared y = -1.5  ->  lateral force 6.00,  roll moment  +0.00
--
--        And it is worse than a one-off disturbance, because the sign closes a loop: a craft
--        leaning right is pushed right by its own lift, the tilt allocation answers with a push
--        to PORT, and a port push applied BELOW the centre of mass rolls it further right.
--        Positive feedback, round a slow path (translation, not attitude), so it shows up as a
--        10-20 s wallow that no attitude gain will touch -- raising or lowering RATE_KP/ATT_KP
--        does nothing to it because the loop does not run through them.
--
--        So a rough figure beats 0 by a mile, and the SIGN matters most. Count blocks from the
--        mount plane to mid-hull if you cannot find the CoM exactly; a uniform hull's CoM is
--        near its geometric centre. If an estimate leaves the craft still rolling INTO a hard
--        strafe, |y| is too small; if it now rolls out of one, too big.
--
-- The centre of mass must lie INSIDE the triangle these three make, or no set of down-firing
-- thrusts can hold a level hover. uac.lua refuses to boot if it does not, and names the
-- engine that would have to pull downwards -- if you see that error, the x/z figures are
-- measured from the hull rather than from the CoM, or a sign is inverted.
CFG.TRI_ENGINES = {
    { key = "fc", name = "FRONT-CENTRE", x =  0.0, y = 0.0, z =  6.0 },
    { key = "bl", name = "BACK-LEFT",    x = -4.0, y = 0.0, z = -4.0 },
    { key = "br", name = "BACK-RIGHT",   x =  4.0, y = 0.0, z = -4.0 },
}

-- HOVER BASELINE -- the thrust each engine sits at when the vertical cascade wants zero net
-- effort. Exactly what CFG.ALT_NEUTRAL is on an up/down airframe, and set to the same figure
-- for the same reason: it leaves 8 steps of headroom above and 7 below. It is a MEAN, not a
-- per-engine setting -- the allocator splits the total by the geometry, so a nose-heavy
-- craft's front engine sits above this figure and the back pair below it, automatically.
-- Too low and the craft sags until the velocity loop's integral learns the difference (it
-- will, and quickly); too high and it needs a standing negative altSteps to stay level,
-- which eats climb headroom. Trim it so the learnt hover trim sits near zero.
-- MEASURED 2026-08-22 and corrected 7.0 -> 4.2. The flight recorder gave 200 samples of
-- vertical acceleration against commanded lift; regressed at the measured 0.30 s transport lag
-- (r = 0.94) the craft accelerates at 0.889 m/s^2 per engine-unit and is weightless at a total
-- lift of 12.54 units, i.e. 4.18 per engine. At 7.0 the allocator was commanding 21 units for
-- a hover that needs 12.5, so the vertical loop sat at a STANDING -3 alt steps just to stay
-- level -- a third of its authority spent before it did any work, and that much closer to the
-- -8 clamp it was in fact hitting 8% of the time.
CFG.TRI_HOVER = 4.2
-- Engine-units of TOTAL lift per step of the vertical cascade's output. 3.0 = one step on
-- each of the three engines, which is the figure that makes ALT_SWING (8 steps) reach from
-- the hover baseline to all three engines at full: (3*15 - 3*7) / 8 = 3.0. Change TRI_HOVER
-- and this should follow, or the top of the vertical channel's travel stops short of -- or
-- runs past, and is clamped away at -- the engines' actual ceiling.
CFG.TRI_LIFT_PER_ALTSTEP = 3.0

-- AUTHORITIES: how much wrench one step of each axis command asks for. Torques are in
-- engine-units times metres, forces in engine-units (one unit = one redstone step on one
-- engine), so they are directly comparable with TRI_HOVER above -- which is the point, and
-- the way to sanity-check a value before flying it.
--
-- HOW THESE WERE SIZED (do the same arithmetic when you change the geometry):
--   PITCH  full stick (15) should move about 6 units of lift between the nose engine and the
--          back pair -- roughly a quarter of the 21 units a hover burns. The pitch arm is
--          6 + 4 = 10 m, so 6 * 10 / 15 = 4.0 per step.
--   ROLL   same 6 units, across the 4 m half-track: 6 * 4 / 15 = 1.6 per step.
--   YAW and LAT SHARE ONE BUDGET, and this is the part that is easy to get wrong. Every
--          sideways newton on this aircraft comes out of the same deflection cone: an engine
--          burning at the 7-unit hover baseline can make at most 7 * tan(30 deg) = 4 units of
--          horizontal force, so all three together have about 12 units in TOTAL to divide
--          between TURNING and TRANSLATING. These two are sized to take half each:
--   YAW    ~2 units of tangential push per engine at a ~5.8 m radius, at full stick:
--          3 * 2 * 5.8 / 15 = 2.3 per step.
--   LAT    ~6 units of sideways force in total at full strafe: 6 / 15 = 0.4 per step.
--          Sized that way, full yaw AND full strafe TOGETHER sit right at the cone, and
--          either alone is comfortably inside it. Raise one and the other loses out -- there
--          is no setting here that buys both, only a choice about which the aircraft should
--          be better at. If the CONE tag lights in ordinary flight, it is these two that are
--          over budget, not the attitude gains.
--          Swept offline, 12,000 ticks of random demand: at 3.5 / 0.4 the cone saturated on
--          43% of full-envelope ticks. At the 2.3 / 0.4 below it is 31% with EVERY axis at
--          full stick at once, 9% at 60% stick, and NOTHING saturates below about 40% stick
--          -- which is where ordinary flight lives. Full-stick-on-all-five-axes saturating is
--          not a fault; it is three engines being asked for more than three engines have.
--
-- THESE ARE OPENING BIDS. They are the gains of the whole aircraft and they were computed,
-- not flown. If an axis oscillates, come DOWN here before reaching for RATE_KP or
-- ROT_GAIN_* -- an authority set too high saturates the allocator, and a saturated allocator
-- rations attitude (watch for the S<n>% tag on the display), which no attitude gain can fix.
CFG.TRI_PITCH_PER_STEP = 4.0
CFG.TRI_ROLL_PER_STEP  = 1.6
CFG.TRI_YAW_PER_STEP   = 2.3
CFG.TRI_LAT_PER_STEP   = 0.4
-- FORE/AFT SHARE. 0 = the nozzles make none of it and the dedicated fore/aft bank (#79)
-- carries the whole thing, which is how Scout is built and why this starts at zero. Raise it
-- to let the nozzles lean into an acceleration alongside that bank -- it buys a much crisper
-- response, at the cost of spending deflection cone that yaw and strafe are also drawing on.
-- Note the fore/aft command still reaches #79 unchanged either way; this only ADDS.
-- WARNING BEFORE RAISING IT: at 0.0 the nozzles make no fore/aft force, which is the only
-- reason the PITCH axis does not have the same mount-height instability roll has (bench: 0.00
-- pitch moment from a full fore/aft demand, against 37.5% of roll authority from a full strafe
-- one). Declare the y figures in CFG.TRI_ENGINES before putting anything on this knob, or the
-- nodding gets a second axis to live on.
CFG.TRI_FWD_PER_STEP   = 0.0

-- SIGNS. Same rule as every other *_SIGN in this file: they are empirical, they are found by
-- flying, and no axis label is to be trusted over what the aeroplane actually did.
--
-- These are SEPARATE from CFG.PITCH_SIGN / ROLL_SIGN / YAW_SIGN / STRAFE_SIGN above, which are
-- applied upstream in attitude() and in the translation laws.
--
-- WHICH OF THE TWO TO FLIP, because there is a right answer and it is not obvious. On an
-- airframe that has BOTH thruster banks and nozzles, flip these -- the upstream signs are that
-- craft's axis-thruster calibration and must not be disturbed to fix a nozzle. On an airframe
-- that is ONLY nozzles (Scout: nothing listens to CFG.STAB_PROTOCOL), flip the UPSTREAM sign
-- and leave these at the identity, for two reasons:
--   * CFG.PITCH_SIGN is read twice -- once to make the command and once by the pitch Smith
--     predictor to read it back -- and only flipping it in both places keeps those agreeing;
--   * the allocator's polarity is not a free calibration. It falls out of CFG.TRI_ENGINES,
--     which is measured, so a wrong sign here means the geometry is being papered over rather
--     than corrected. If pitch AND roll are both inverted, suspect exactly that: a TRI_ENGINES
--     written nose-forward on a hull whose sublevel body frame has the nose at -z (this
--     airframe sets CFG.HEADING_OFFSET = pi, which says the logical nose is 180 deg off the
--     thrust-forward axis). Negating every x and z in the table is then the honest fix, and it
--     corrects the strafe and yaw polarity at the same time instead of leaving those to be
--     rediscovered one at a time.
-- So: these exist for the mixed case and for a wiring fault you do not want to fix upstream.
-- They are at the identity on Scout deliberately.
CFG.TRI_PITCH_SIGN = 1
CFG.TRI_ROLL_SIGN  = 1
CFG.TRI_YAW_SIGN   = 1
CFG.TRI_LAT_SIGN   = 1
CFG.TRI_FWD_SIGN   = 1

-- ENGINE OUTPUT BAND. TRI_T_MIN above 0 keeps every engine idling rather than letting one
-- shut down completely -- worth having if the engines are slow to relight or if a fully
-- closed nozzle takes time to come back on demand, at the cost of the craft never being able
-- to stop pushing. 0 is right unless the engines misbehave at zero.
CFG.TRI_T_MIN = 0
CFG.TRI_T_MAX = 15
-- Floor on the per-engine weight used to split the horizontal (strafe/yaw) demand between
-- the three nozzles. The weights are last tick's thrusts -- an engine burning hard can lend
-- more sideways push at the same nozzle angle than one barely alight -- and this stops a
-- shut-down engine from dividing by zero, or the whole split from collapsing onto one engine
-- during a cut. It only chooses BETWEEN exact answers, so it can never change the force the
-- craft is asked for; raise it toward TRI_HOVER to make the split more even, drop it toward
-- zero to lean harder on whichever engine is already hot.
CFG.TRI_W_FLOOR = 1.0

-- THE NOZZLES THEMSELVES.
-- TRI_VEC_MAX      the gimbal command at full deflection -- the 15 in "X-axis, -15 to 15".
-- TRI_VEC_MAX_ANGLE  what that 15 is actually WORTH, as an angle off straight down. This is
--                  the one number here that is a property of the mod, not of the airframe,
--                  and it matters twice over: it scales every nozzle command, AND it is the
--                  cone the allocator refuses to command past. Set it too large and the
--                  craft under-deflects and feels sluggish in strafe and yaw; too small and
--                  it commands angles the mount cannot reach, which reads as a strafe that
--                  saturates early and a yaw that will not settle. Measure it: command a
--                  known deflection and see how far the nozzle actually goes.
-- TRI_VEC_LAW      "angle" = the command is proportional to the deflection ANGLE (the usual
--                  case for a servo-driven mount). "tan" = it is proportional to the RATIO
--                  of sideways to downward force, which is what a mount that is really
--                  blending two thrust components wants. If strafe response is correct near
--                  centre but goes soft at large deflections, try the other one.
CFG.TRI_VEC_MAX       = 15
CFG.TRI_VEC_MAX_ANGLE = math.rad(30)
CFG.TRI_VEC_LAW       = "angle"
-- Which way a positive gimbal command actually tips the thrust. X is the body-lateral axis
-- (+ should push the craft to STARBOARD), Y is the body-longitudinal one (+ should push it
-- FORWARD). Both are guesses until flown. TRI_VEC_SWAP exchanges the two outputs wholesale,
-- for the case where the mount's "X" is the aircraft's fore/aft -- the symptom is a strafe
-- command that yaws or surges the craft instead of moving it sideways, which no amount of
-- sign-flipping will fix. (SWAP and LAW are the only two knobs in this block uac.lua does
-- not demand at boot: absent means false and "angle", both of which are valid answers.)
CFG.TRI_VEC_X_SIGN = 1
CFG.TRI_VEC_Y_SIGN = 1
CFG.TRI_VEC_SWAP   = false
