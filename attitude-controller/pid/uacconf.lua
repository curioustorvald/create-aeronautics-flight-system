-- AER TOGEKISS -- UNIFIED ATTITUDE CONTROLLER: CALIBRATION & TUNING
--
-- Every tuning value, sign calibration and protocol name the UAC uses. Split out of
-- uac.lua so the PROGRAM and the AIRFRAME's calibration are separate files: dropping in
-- a new uac.lua no longer clobbers weeks of in-game sign-flipping, and this file is a
-- manageable thing to open and edit in-game instead of scrolling a 1800-line program.
--
-- This is the SINGLE SOURCE OF TRUTH for these values -- uac.lua holds no fallback copy.
-- That is deliberate: a duplicated default would mean editing a knob here could silently
-- do nothing (or vice versa), which is exactly the kind of ghost that makes empirical
-- sign-calibration impossible to trust. If this file is missing or has a syntax error the
-- UAC refuses to boot with a clear message rather than flying on guessed numbers.
--
-- HOW IT LOADS: uac.lua does require("uacconf"), which resolves relative to the running
-- program's directory -- so this file must sit BESIDE uac.lua (i.e. in pid/), same as
-- pid.lua. It must return the CFG table.
--
-- RULES FOR EDITING:
--   * Add new knobs as CFG.NAME = value. NEVER as a bare `local` -- Cobalt (CC's Lua VM)
--     caps a function at 200 local variables and counts the main chunk's locals against
--     every nested function; the whole point of this table is that ~200 knobs cost 1 local.
--   * Values may reference earlier ones (e.g. CFG.AL_DESC_MAX = CFG.DESCEND_MAX) since
--     this builds top-to-bottom. Order therefore matters for those.
--   * The DERIVED flight-mode flags (STRAFE_ENABLED / ATTITUDE_BANKS / MODE_TAG) are NOT
--     here -- applyFlightMode() in uac.lua computes them from CFG.FLIGHT_MODE, because the
--     on-screen MODE button re-derives them at runtime. Set CFG.FLIGHT_MODE here; the
--     flags follow.
--   * CALIBRATION IS EMPIRICAL. The per-knob comments record what was actually observed
--     in-game and why a sign is what it is -- they are the most valuable thing in this
--     file. Keep them with the values, and add to them when you flip something.

local CFG = {}
-- PROTOCOLS (identical to control.lua + navcom.lua, so nothing else changes) ---
CFG.APNAV_PROTOCOL    = 'aertogekiss_navap'      -- in : FMS target + phase
CFG.ALTAP_PROTOCOL    = 'aertogekiss_altap'      -- in : ALT-AP panel setpoint
CFG.AUTOLAND_PROTOCOL = 'aertogekiss_autoland'   -- in : FMS vertical channel
CFG.APSWITCH_PROTOCOL = 'aertogekiss_apswitch'   -- in : pilot autopilot toggles (nav / altitude), #12
CFG.MANCON_PROTOCOL   = 'aertogekiss_mancon'     -- in : manual wanted climb rate (m/s), #6
CFG.MANNAV_PROTOCOL   = 'aertogekiss_mancon_nav' -- in : manual horizontal+yaw stick (m/s, rad/s), #6
                                                 --      (was the MIC #14 until 2026-08-28;
                                                 --      #6's mancon.lua owns both now)
CFG.KBDJOY_PROTOCOL   = 'aertogekiss_kbdjoy'     -- in : keyboard joystick -- pitch/roll TARGET (rad) + throttle, #21
-- OUTPUT PROTOCOLS CARRY FLOATS, NOT REDSTONE STEPS (2026-08-26). STAB, NAVCTRL and ALT
-- all changed shape when the sigma-delta moved into the relays: signed [-1, 1] per AXIS
-- (a fraction of one full redstone line), not split unsigned 0..15 per BANK.
--     STAB    { pitch, roll }                  -- was { front, back, left, right }
--     NAVCTRL { thrust, yaw, strafe }          -- was { thrust_forward, ... , strafe_right }
--     ALT     { up, down, cut }                -- up/down now [0,1]; cut is new and explicit
-- Which bank a signed demand lands on, and what whole step it rounds to, is the receiving
-- relay's answer to give. See uac.lua's OUTPUT header for why.
CFG.STAB_PROTOCOL     = 'aertogekiss_stab'       -- out: pitch/roll thrusters
CFG.NAVCTRL_PROTOCOL  = 'aertogekiss_navcontrol' -- out: yaw/thrust/strafe
                                             --      (additive; NAVCTRL above is unchanged)
CFG.ALT_PROTOCOL      = 'aertogekiss_alt'        -- out: up/down thruster

-- 'aertogekiss_controlf' was a half-built attempt at the same idea -- ONE message carrying
-- every axis as a float -- that never had a listener, and uac.lua's sendControlf() that fed
-- it was never called. The 2026-08-26 refactor did the job on the three protocols the relays
-- already subscribe to instead, so the knob is gone rather than left looking live. Nothing
-- broadcasts on that name.
CFG.THRUSTF_PROTOCOL = 'aertogekiss_thrustf'

-- MASTER ENABLES -- bring the craft up one channel at a time (it is still ONE
-- file; these just zero a channel's output so you can isolate a problem). These are
-- the source-level debug isolators; the pilot's runtime mode is the AP switches below.
CFG.ENABLE_ATTITUDE    = true
CFG.ENABLE_TRANSLATION = true
CFG.ENABLE_ALTITUDE    = true

-- PILOT AUTOPILOT MODE (the #12 toggle switches, over aertogekiss_apswitch) ----
-- Levelling/attitude is ALWAYS held. On top of that the two switches pick the mode:
--   nav ON  + alt ON  -> navigate + hold altitude + level          (full autopilot)
--   nav ON  + alt OFF -> navigate + level; altitude from the manual VSC (aertogekiss_mancon)
--   nav OFF + alt ON  -> hold altitude + level; horizontal/yaw flown by the pilot (MIC #14)
--   nav OFF + alt OFF -> level + manual horizontal/yaw (MIC #14); altitude manual via the VSC
-- "handed to the pilot" now means the nav MANCON stick (MIC #14) over MANNAV_PROTOCOL, not a
-- dead-stick coast: with no MIC packet the craft still just holds heading + zero translation.
-- An in-progress autoland overrides BOTH switches (it is a committed sequence with its
-- own abort path) and keeps flying the craft down. The DEFAULTS below apply only until
-- #12 is first heard; false/false = safe (hand-off + manual) so a craft with no switch
-- computer wired does not fly itself off.
CFG.AP_NAV_DEFAULT = false
CFG.AP_ALT_DEFAULT = false
CFG.MANCON_STALE_AFTER = 0.5   -- s; manual vertical older than this -> treat as neutral hover

-- SCRAMBLE -- the VS stick's TOP DETENT (level 15 on the VSC), and the one vertical command
-- in the whole system that is NOT a rate: "lift engine to full throttle, climb as fast as
-- this airframe can". It bypasses the inner rate loop entirely and drives the alt thruster
-- to CFG.ALT_SWING outright, routed by attitude like every other vertical command.
--
-- IT IS ALSO THE ONE VERTICAL COMMAND THAT DOES NOT DRIVE BOTH BANKS. Every other command
-- is a differential about the balanced hover (CFG.ALT_NEUTRAL +/- steps, so the down line is
-- still burning at 3 while the up line is at 11); a scramble switches the opposing bank OFF
-- and burns the helping one at CFG.ALT_NEUTRAL + |steps|. Without that the attitude routing
-- above quietly cancels the detent -- the down line only reaches CFG.ALT_MIN while the tilt
-- is inside acos(ALT_NEUTRAL / ALT_SWING) (~29 deg on this fleet), and past it the retro bank
-- burns against the scramble. See sendAlt's header in uac.lua for the arithmetic.
--
-- WHY IT NEEDED A CHANNEL OF ITS OWN. It used to be sent as a rate -- vs = 100 m/s off the
-- stick's top step -- and there were three independent layers between that and the thrusters,
-- any ONE of which was enough to defeat it:
--   1. the VSC's own command slew (VS_SLEW, 4 m/s per second) ramped toward the 100, so the
--      command took ~25 s to arrive -- and, worse, left the accumulator parked up there
--      afterwards, so the stick stayed disconnected from the aeroplane for another ~25 s
--      after the pilot let go of the detent;
--   2. this file clamped the wanted rate to CFG.CLIMB_MAX before the rate loop ever saw it,
--      so what actually survived was the SAME command a level-14 stick already gives;
--   3. with ALT-AP on, the mancon rate is not read at all -- the altitude cascade owns the
--      channel -- so in the aircraft's normal cruise state the detent was a plain no-op.
-- No value of `vs` could have cleared all three. A flag does, and it leaves the rate channel
-- and its envelope -- which the autoland also flies inside -- completely untouched.
--
-- FALSE disables the detent here: the VSC still shows SCRAMBLE on its own screen, the flag
-- simply goes unread, and level 15 flies the frozen rate travelling beside it (i.e. whatever
-- the proportional part of the stick last asked for).
CFG.MANCON_SCRAMBLE = true
-- Does a scramble outrank the ALTITUDE CASCADE -- ALT-AP holding a level, and an autoland
-- already flying the craft down?
--   true  -- yes. The detent is then an override the pilot can reach for without switching
--            anything off first, which is what makes it usable as a hand-flown go-around and
--            as a terrain escape: at the moment either is wanted, ALT-AP or the autoland is
--            precisely what is holding the craft where it must not stay. Note what it does
--            NOT do -- it neither cancels the autoland nor clears the FMS's target. It takes
--            the vertical for exactly as long as the detent is held and hands the channel
--            straight back on release, so announce the go-around on the FMS as usual if the
--            approach is actually being abandoned.
--   false -- the detent works only with the vertical already in manual (ALT-AP off and no
--            autoland running). Safer against a stray redstone 15 on the stick line, at the
--            cost of the two cases anyone would want a scramble for in the first place.
-- Either way it outranks the ON-THE-GEAR rules (CFG.AL_GROUND_CUT and CFG.AL_GROUND_VS):
-- scrambling off a pad is half the point of having one, and a cut engine cannot answer it.
CFG.MANCON_SCRAMBLE_OVERRIDE = true

-- MANUAL HORIZONTAL + YAW (the nav MANCON stick, via the MIC #14 over MANNAV_PROTOCOL).
-- Active only with AP-NAV OFF and a fresh MIC packet; then the pilot's wanted body-frame
-- forward/right speed (m/s) and yaw RATE (rad/s) fly the craft. A released stick commands
-- zero -> brake to a hover and hold heading. MANNAV_YAW_SIGN is a calibration GUESS like the
-- other *_SIGN (flip if yaw turns the wrong way); fwd/strafe reuse the calibrated THRUST_SIGN/
-- STRAFE_SIGN. Independent of the autopilot by design -- see the MIC header on #14.
CFG.MANNAV_ENABLE      = true
CFG.MANNAV_YAW_SIGN    = -1     -- +1: MIC yaw + (nose-right) advances the heading target CW
CFG.MANNAV_STALE_AFTER = 0.5   -- s; MIC packet older than this -> no manual horizontal/yaw
-- Yaw-rate BRAKE (DISABLED -- kept for reference). This added EXTRA proportional rate damping
-- (-BRAKE*wy) on release to stop the yaw harder. It turned out to be the CAUSE of the release
-- oscillation: RATE_KP_YAW=5 already puts the discrete rate loop at ~critical damping
-- (gain*dt = 5*0.05 = 0.25), and the ~1-tick actuator lag makes any EXTRA rate-feedback gain
-- RING -- with BRAKE=9 the effective gain 14 gives complex closed-loop roots (|z|~0.84), i.e.
-- the several-oscillations-then-settle the pilot saw (and why no MANNAV_YAW_SETTLE value fixed
-- it -- SETTLE changes when the HEADING is captured, not the loop gain). You cannot stop faster
-- than the critically-damped loop by adding gain: loop bandwidth is capped by the 50 ms tick +
-- lag, NOT by thrust (the extra yaw thrusters raised authority, not bandwidth). To stop quicker,
-- build LESS rate to begin with -- lower the MIC's YAW_MAX.
CFG.MANNAV_YAW_BRAKE   = 0     -- 0 = off. Leave off; >0 re-introduces the release ring (see above).
CFG.MANNAV_YAW_EPS     = 0.02  -- rad/s; |manual yaw command| below this = "not commanding yaw" -> brake on
CFG.MANNAV_YAW_SETTLE  = 0.05  -- rad/s; on release, don't CAPTURE the hold-heading until |yaw rate| is
                               -- below this. Freezing it earlier makes the craft drift back (opposite way)
                               -- to the release-instant heading momentum already carried it past.

-- ATTITUDE LAW MODE -----------------------------------------------------------
--  "PD"      : steps = KP*errVec - KD*angVel, straight to redstone. Simple,
--              tunes exactly like the old stabiliser (see the deg->rad note on the
--              gains). Get leveling + yaw flying in this mode first.
--  "INERTIA" : the draft's full loop -- attitude error -> desired angular VELOCITY
--              (outer P) -> desired angular ACCELERATION (inner PD) -> multiply by
--              the inertia tensor -> add gyroscopic term w x (I w) -> steps.
--              Switch to this once PD flies and probe.lua has confirmed the tensor
--              shape and the angular-velocity frame. Its gains are unitless-ish
--              (steps per N.m), so they need their own tune -- see ROT_GAIN_*.
CFG.ATT_MODE = "INERTIA"

-- SENSOR FRAME -- which frame getAngularVelocity()/getInertiaTensor() use. Sable
-- does not document it and its source is closed, so it is switchable. The draft's
-- own note that the inertia tensor is "NOT constant" points to WORLD frame (a
-- rigid body's tensor is constant under rotation in its BODY frame). The attitude
-- loop always works in the BODY frame; "world" means the angular velocity is rotated
-- into body and the tensor is applied via a body->world->body round-trip, "body"
-- means both are used as-is. The *_SIGN/SWAP output calibration is shared by PD and
-- INERTIA either way. Symptom of the wrong setting: levels fine near heading 0 but
-- wanders once yawed ~90 deg -- then flip this.
CFG.SENSOR_FRAME = "body"

-- Component convention (confirmed against control.lua/navcom's working use of
-- angVel): x = PITCH axis, y = YAW axis, z = ROLL axis. errVec and torque share it.

-- ---- PD-mode gains (default) ----
-- NOTE ON MAGNITUDE: the old gimbal law used KP=0.4 on DEGREES. The quaternion
-- error is in RADIANS, so the equivalent P gain is ~0.4*57.3 = ~23. That is why
-- these look large next to the old 0.4 -- same aggressiveness, different units.
-- If PD feels sluggish, raise these (the frame + shortest-path fixes let you push
-- them harder without the old wander). Pitch and roll gains are independent -- the
-- loop now runs per-axis in the body frame, with no cross-axis rotation.
CFG.PD_KP_PITCH = 23.0   -- steps per rad of pitch error
CFG.PD_KP_ROLL  = 23.0   -- steps per rad of roll error
CFG.PD_KP_YAW   = 6.0    -- steps per rad of yaw error
CFG.PD_KD_PITCH = 0.6    -- steps per rad/s (damps on measured pitch rate); old = 0.6
CFG.PD_KD_ROLL  = 0.6
CFG.PD_KD_YAW   = 1.4    -- old navcom hands-off yaw damping was 1.4

-- ---- INERTIA-mode gains ----
-- The cascade acts as an effective PD: P ~ ROT_GAIN*I*CFG.RATE_KP*CFG.ATT_KP on the angle
-- error, D ~ ROT_GAIN*I*CFG.RATE_KP on the rate. So LOOP BANDWIDTH (speed) scales with
-- CFG.RATE_KP (and ROT_GAIN), and the DAMPING RATIO is set by 1/CFG.ATT_KP. Underdamped +
-- sluggish (several slow oscillations before settling) = low bandwidth: RAISE CFG.RATE_KP
-- first (1 -> 3 -> 5) -- it speeds up AND damps, by pushing the loop crossover above
-- the fixed CFG.ATT_KP corner. If it still overshoots, LOWER CFG.ATT_KP (3 -> 2). If it buzzes
-- (fast oscillation) you have over-gained -- back CFG.RATE_KP/ROT_GAIN off.
CFG.ATT_KP      = 1.5    -- outer: desired angular velocity per rad of error (1/s)
-- ---------------------------------------------------------------------------------------
-- PER-AXIS OUTER GAINS (nil => fall back to CFG.ATT_KP; uac.lua reads them as
-- `CFG.ATT_KP_PITCH or CFG.ATT_KP`, so absent is a valid answer and this is a no-op on any
-- airframe that does not set them). PITCH and ROLL added 2026-08-20 alongside ATT_KP_YAW.
--
-- THE ONE THING TO UNDERSTAND BEFORE TOUCHING THESE. The rate loop's true gain is
--     G = ROT_GAIN * I * (rad/s^2 per step) * RATE_KP
-- and the accel per step is itself (torque per step)/I, so
--     G = ROT_GAIN * (torque per step) * RATE_KP        -- THE INERTIA CANCELS.
-- ROT_GAIN_* is therefore NOT an inertia-compensation knob -- the live tensor multiply in
-- uac.lua already does that job, which is why it adapts to cargo for free. ROT_GAIN_* is purely
-- the THRUSTER's torque-per-step calibration, and the loop gain does not care how heavy the
-- craft is. Two consequences that matter here:
--   * the SAME stability ceiling applies to all three axes, whatever their inertia. With the
--     ~0.5 s transport delay measured on this rig (see CFG.ATT_KP_YAW) that ceiling is
--     G = pi/(2T) = 3.14 /s, and 60 deg of phase margin wants G ~ 1.05 /s.
--   * the three axes still need three DIFFERENT ROT_GAIN and ATT_KP values, because they hang
--     off three different thruster sets with three different torque-per-step.
--
-- PITCH AND ROLL ARE THE UNTOUCHED TWIN OF THE YAW FAULT. They carry ROT_GAIN 5e-6 against the
-- shared ATT_KP 1.5 -- the exact pair that was just proven to sit ON the stability boundary on
-- yaw, which had to come down to 5.7e-7 to get off it. Nothing has ever been done to them: the
-- 2026-08-20 work fixed yaw, fore/aft and strafe and left these two exactly as they were. The
-- pilot's report of nose nodding in level flight, and of heavy roll+pitch swaying whenever the
-- craft is climbing or descending, is what that predicts.
--
-- MEASURED 2026-08-20: nose nodding with a 3-5 s period while STAB stayed at a FEW STEPS OF 15.
-- Those two facts together pin one cell of the table, and it is NOT the cell yaw was in.
--
-- Simulated pitch cascade, T = 0.50 s, 6 deg upset -- ring period / peak STAB step:
--
--     ATT_KP \ G      0.80        1.05        1.60        2.20        3.14
--     1.50            4.9s/0.4    4.1s/0.7    4.6s/15     5.0s/15     5.1s/15
--     0.60            7.6s/0.1    5.6s/0.1    3.2s/0.1    2.6s/0.4    2.4s/15
--     0.45            9.6s/0.1    5.7s/0.1    3.1s/0.0    2.6s/0.2    2.3s/15
--     0.33           15.9s/0.0    none/0.04   none/0.02   2.5s/0.1    2.2s/15
--     0.25            none/0.02   none/0.02   none/0.02   none/0.08   2.2s/15
--
-- Only the top row at G 0.80-1.05 produces a 3-5 s ring with a SMALL command; every G at or
-- above 1.60 saturates STAB, which the pilot did not see. So on this axis the RATE LOOP GAIN IS
-- ALREADY ABOUT RIGHT -- G ~ 1, which is exactly the 60-deg-phase-margin target -- and what is
-- ringing is the shared outer gain ATT_KP = 1.5 sitting on top of it.
--
-- THIS IS THE OPPOSITE OF THE YAW DIAGNOSIS, and the difference is real, not a contradiction.
-- G = ROT_GAIN * (torque per step) * RATE_KP, so with pitch/roll at ROT_GAIN 5e-6 and G ~ 1, a
-- stab step is worth ~4.2e4 N.m, against ~3.7e5 N.m for a yaw step at ROT_GAIN_YAW's measured
-- G ~ 3.14. A yaw step being ~9x a stab step is exactly what "~3x more yaw thrusters" on a
-- SUPERWIDE hull (long moment arm) should give. The two calibrations corroborate each other.
--
-- SO: ROT_GAIN_PITCH AND ROT_GAIN_ROLL ARE LEFT ALONE. Change one thing. The fix is the outer
-- gain, and only the outer gain.
--
-- PITCH 0.33: clean at the measured G, and it is also the analytic target -- zeta 0.9 wants
-- ATT_KP = G/(4*zeta^2) = 1.05/3.24 = 0.32.
-- ROLL 0.25, one notch more conservative, DELIBERATELY NOT THE SAME as pitch. Roll's G was
-- never measured (the pilot's trace was the nose), and it is the axis most likely to be higher:
-- inertia cancels out of G, so what is left is torque per step, and on a superwide hull the
-- left/right stab thrusters sit much further outboard than the front/back pair. 0.25 is clean
-- across G 0.8-2.2 where 0.33 starts ringing at 2.2. If roll now feels sluggish, raise it to
-- meet pitch at 0.33.
-- THE TRADE, stated plainly. A lower outer gain buys damping with STIFFNESS. Simulated step
-- disturbance -- the off-CoM moment the lift thrusters inject once a real climb is commanded --
-- peak excursion / still ringing / where it settles:
--
--     ATT_KP 1.50 (before)   1.38 deg / ringing at 4.2 s / 0.72 deg
--     ATT_KP 0.33 (pitch)    3.31 deg / no ring          / 3.31 deg
--     ATT_KP 0.25 (roll)     4.37 deg / no ring          / 4.37 deg
--
-- That is the whole "ascending/descending sways it" case: the ring is gone, at the cost of a
-- few degrees of standing tilt while the disturbance lasts. On PITCH that cost is temporary --
-- attIntegX is active and trims the standing part away. On ROLL it is NOT: attIntegZ is
-- commented out in uac.lua, so the roll axis has no integral trim and will simply SIT at its
-- offset for the length of the climb. A few degrees is tolerable (the strafe loop answers the
-- lateral drift it causes), but know which symptom is which: SWAYING is this gain's business,
-- SITTING at an angle is the missing integral's, and chasing the second with this knob only
-- brings the first one back.
--
-- If either axis instead starts SLAMMING STAB 0<->15, stop and re-read: that is a different
-- fault (an unbrakeable rate demand, #19 Citation's pitch nod) whose fix is MAX_RATE_* and
-- MAX_DECEL_* below, not these. Those are still 4.0/8.0 here, i.e. the profile is effectively
-- disabled -- but with ATT_KP at 0.33 the loop cannot demand more than 0.33*pi ~ 1.0 rad/s even
-- from a full inversion, so it stays clear of the profile and the profile stays a safety net.
CFG.ATT_KP_PITCH = 0.70  -- outer gain, PITCH axis only (1/s)
CFG.ATT_KP_ROLL  = 0.33  -- outer gain, ROLL  axis only (1/s)
-- YAW-ONLY outer gain (nil => fall back to CFG.ATT_KP; uac.lua reads
-- `CFG.ATT_KP_YAW or CFG.ATT_KP`, so this is a no-op on any airframe that does not set it).
-- Added 2026-08-20, the same hook #19 Citation added as CFG.ATT_KP_PITCH when it found that
-- "ON THIS AIRFRAME PITCH IS A YAW-CLASS AXIS AND WAS WEARING PITCH-CLASS SETTINGS". Yaw here
-- is the yaw-class axis and has always worn the shared, pitch/roll-tuned number.
-- ATT_KP sets the cascade's DAMPING RATIO, not its speed:
--     omega_n = sqrt(G*ATT_KP)      zeta = 0.5*sqrt(G/ATT_KP)
-- with G = ROT_GAIN_YAW * Iy * RATE_KP_YAW * (rad/s^2 per step) -- the rate loop's true gain.
-- 1.5 -> 0.33 (2026-08-20), from the pilot's measurement: yaw oscillating with a ~2 s period
-- while the yaw command stayed inside +-1 step of 15.
--
-- THE TWO NUMBERS TOGETHER RULE OUT THE OBVIOUS ANSWER. A command that never leaves +-1 of 15
-- is not saturating, so this is a LINEAR ring and not the unbrakeable-rate-demand mechanism
-- that #19's pitch nod turned out to be -- the motion profile cannot be the cause, because the
-- profile only bites when the loop is asking for more than it can deliver. And the period rules
-- out this outer loop as the source: with zeta = 0.5*sqrt(G/ATT_KP), any G high enough to ring
-- fast is also high enough to be overdamped, so the FASTEST ring this cascade can produce at
-- ATT_KP 1.5 is 4.19 s (at G = 3.0, zeta = 0.71). Observed 2 s is twice that. It is the INNER
-- rate loop ringing, and the outer loop is merely riding it.
--
-- WHAT 2 s PINS. The rate loop is an integrator (steps -> torque -> angular accel -> rate)
-- behind the transport delay of the whole chain: UAC tick -> rednet -> relay #9 -> redstone ->
-- thruster spool. A SUSTAINED oscillation means it sits on the stability boundary, and that is
-- two equations for the two unknowns:
--     angle  -90 - w*T = -180  ->  T = (pi/2)/w
--     gain   G/w = 1           ->  G = w
-- with w = 2*pi/2.0 = 3.14 rad/s, so T ~ 0.50 s and G ~ 3.14 /s. (A slight over-estimate of G:
-- the pilot's oscillation is bounded at +-1 step rather than growing, so the truth is just
-- under the boundary, not over it.) Half a second of dead time is entirely plausible for that
-- chain, and it is the real ceiling on every loop in this system -- see the note on G = NORM/tau
-- in the TRANSLATION section, which found the SAME 0.5 s and the same factor of ~3 on strafe.
--
-- THE FIX IS BOTH KNOBS, and that is why this hook had to exist. Phase margin at T = 0.5 s
-- depends only on G, so ROT_GAIN_YAW must come down 3x for PM 60 deg (G 3.14 -> 1.05); but at
-- G 1.05 with ATT_KP still 1.5 the simulated capture STILL crosses the target 42 times, ringing
-- at 4.1 s -- it just moves the oscillation from the inner loop to the outer one. Simulated
-- 10 deg capture, T = 0.50 s, crossings / ring period / settle:
--
--     G 3.14  ATT_KP 1.50  (today)      78 / 2.3 s / never     <- reproduces the report
--     G 1.05  ATT_KP 1.50               42 / 4.1 s / 8.7 s
--     G 1.05  ATT_KP 0.60               32 / 5.6 s / 4.0 s
--     G 1.05  ATT_KP 0.33                0 /  --  / 5.2 s      <- chosen
--     G 1.05  ATT_KP 0.20                0 /  --  / 9.6 s
--
-- 0.33 is also what the analytic target gives: zeta 0.9 needs ATT_KP = G/(4*zeta^2) = 0.32.
-- RATIO SETS DAMPING, PRODUCT SETS SPEED -- if the nose is now too lazy, raise this and
-- ROT_GAIN_YAW TOGETHER; if it swings again, lower this one alone.
CFG.ATT_KP_YAW  = 0.50   -- outer gain, YAW axis only (1/s)
CFG.RATE_KP     = 5.0    -- inner: desired angular accel per rad/s of rate error (pitch/roll)
CFG.RATE_KP_YAW = 8.0    -- inner gain for YAW alone -- raise for a crisper yaw WITHOUT
                           -- over-gaining pitch/roll (they keep CFG.RATE_KP)
CFG.RATE_KD     = 0.0    -- inner: derivative on the rate error (noisy; start 0)
-- INERTIA outputs a physical torque (I*alpha + gyro) in N.m. This craft's inertia
-- is huge (the on-screen `raw` runs into the 1e5-1e6 range), so steps-per-N.m is
-- tiny. Rule: ROT_GAIN ~= (steps wanted at a firm correction) / raw seen at that
-- correction, e.g. 7 / 1.3e6 ~= 5e-6. Pitch/roll (stab thrusters) and yaw (nav
-- thrusters) have different authority, so tune the three separately.
CFG.ROT_GAIN_PITCH = 0.000005
CFG.ROT_GAIN_ROLL  = 0.000005
CFG.ROT_GAIN_YAW   = 0.00000140   -- was 5e-6; /3 after ~3x more yaw thrusters (each step now ~3x
                              -- the torque, so the loop was 3x over-gained -> left/right oscillation).
                              -- "If it STILL hunts you added more than 3x -> keep dividing" -- and it
                              -- still hunted, so 1.7e-6 /3 again = 5.7e-7 (2026-08-20). This time the
                              -- factor is MEASURED, not guessed: the pilot's ~2 s ring puts the rate
                              -- loop's true gain at G ~ 3.14 /s behind ~0.5 s of transport delay, and
                              -- 60 deg of phase margin at that delay wants G ~ 1.05 -- exactly /3.
                              -- See CFG.ATT_KP_YAW for the derivation; the two must move together.
                              -- NOTE this REPLACES the earlier reading that the rate loop was already
                              -- near critical damping (from the MANNAV_YAW_BRAKE experiment ringing at
                              -- an effective gain of 14 but not at 5). That reasoning assumed NO
                              -- transport delay -- gain*dt with dt = 0.05 -- which is the wrong model
                              -- for a chain that takes half a second to answer. With the delay in it,
                              -- gain 5 is already past the boundary, which is why the brake experiment
                              -- rang at all and why the yaw has been hunting since.
                              -- So reach for the profile first, this second.

-- INERTIA cross-axis coupling. false (default): use ONLY the body-diagonal moments,
-- so each axis is independent -- a pitch demand makes pitch torque and nothing else
-- (fixes "tries to roll when it shouldn't"). The diagonal gyro term still decouples
-- cross-axis RATE coupling. true: the full tensor incl. products of inertia --
-- physically exact for a rigid body, but it deliberately asks the per-axis thrusters
-- for cross-axis torque, which on an asymmetric craft shows up as unwanted roll/yaw.
CFG.INERTIA_COUPLING = false

-- Outer-loop MOTION PROFILE (per axis). The desired angular velocity is capped at
-- sqrt(2*MAX_DECEL*|err|), itself capped at MAX_RATE, so a HIGH-INERTIA axis counter-
-- steers EARLY and cannot build momentum it can't shed before the target. This is what
-- stops YAW (enormous inertia here) from blasting past the heading, reversing, passing
-- 180 deg, and -- via the huge yaw term in the gyro coupling -- pitching the craft over
-- and plummeting. Set MAX_DECEL to the axis's REAL max angular deceleration (thruster
-- torque / axis inertia); LOWER = brakes sooner = less overshoot. Measure like navcom:
-- spin up to a steady rate (= MAX_RATE), then full-reverse and time to zero
-- (MAX_DECEL = MAX_RATE / that time). Caps set high effectively disable the profile
-- (pitch/roll here are high, so leveling is unchanged; only the slow yaw axis binds).
--
-- *** YAW 0.5/0.05 -> 0.30/0.02 (2026-08-20). NOT THE FIX FOR THE YAW OSCILLATION -- that is
-- CFG.ROT_GAIN_YAW and CFG.ATT_KP_YAW, see their notes. The pilot's report (command never
-- leaving +-1 step of 15) says the loop never saturates, and an unbrakeable rate demand is
-- exactly the saturating failure, so the profile was never what was ringing. These are lowered
-- anyway, as the SAFETY NET they are meant to be: with ATT_KP_YAW at 0.33 the linear branch now
-- governs everything inside 22 deg of heading error, so these two sit clear of the retuned loop
-- rather than defining it -- which is precisely where #19 left MAX_RATE_PITCH/MAX_DECEL_PITCH
-- after its own retune. Do not tune the swing with these. *** The tell that they were never
-- anyone's considered values is the same one that gave away CLIMB_AT_FULL = 5.0 and the whole
-- horizontal-gain block: 0.5/0.05 is what #38 Mantine and #27 SS Argo carry too. A superwide
-- VTOL with the largest yaw inertia in the fleet, a small winged VTOL and a sailless watership
-- cannot all decelerate in yaw at the same 0.05 rad/s^2 -- these were never measured on any of
-- them. Only #19 Citation ever measured its own, and doing so is what cured its nod.
--
-- WHY THIS DIRECTION, given the number is unknown. MAX_DECEL is a PROMISE the profile makes on
-- the thrusters' behalf: it commands the fastest closing rate it believes it can still stop
-- from. Promise more than the craft can deliver and it arrives at the heading too fast, sails
-- past, brakes, sails past the other way -- left/right oscillation. Promise less and the yaw
-- is merely lazy. The error is therefore ONE-SIDED, so with the true figure unknown the safe
-- move is to be pessimistic; #19 says the same ("the profile must be more pessimistic than the
-- thrusters, or it plans a stop it cannot make"). This craft is the fleet's heaviest in yaw,
-- so 0.05 is far likelier to be the optimistic end than the pessimistic one.
--
-- COST OF BEING WRONG-BUT-PESSIMISTIC: a slower nose, and nothing else. This is a VTOL -- the
-- track is flown by the strafe/translation loops, which are holonomic and heading-agnostic, so
-- yaw only points the nose. Manual yaw is untouched (the MIC names its rate directly and
-- bypasses profileRate), and CFG.HDG_SIM_MAX_RATE tracks MAX_RATE_YAW but only bites in
-- HYBRID/keyboard-joystick, neither of which exists here.
--
-- MEASURE IT, exactly as navcom and #19 describe: spin up to a steady yaw rate, full-reverse,
-- and time to zero -- MAX_DECEL = rate/time, then keep ~0.7x of it as margin. The steady rate
-- that full yaw reaches IS MAX_RATE_YAW. If the nose is now too lazy after measuring, raise
-- these; if it STILL swings past the heading, lower MAX_DECEL_YAW further before touching any
-- gain -- an unbrakeable rate demand looks exactly like an over-gained loop and is not one.
-- (0.30/0.02: the sqrt branch governs above 2*MAX_DECEL/ATT_KP_YAW^2 ~ 0.018 rad, i.e. the
-- profile shapes essentially the whole working range, and the rate cap only binds beyond
-- 129 deg of heading error.)
CFG.MAX_RATE_PITCH,  CFG.MAX_RATE_YAW,  CFG.MAX_RATE_ROLL  = 4.0, 0.30, 4.0   -- rad/s
CFG.MAX_DECEL_PITCH, CFG.MAX_DECEL_YAW, CFG.MAX_DECEL_ROLL = 8.0, 0.02, 8.0   -- rad/s^2

-- Gyroscopic feedforward, omega x (I omega). With this craft's enormous yaw inertia
-- and 20 Hz-noisy angular velocity, this term injects large spurious pitch/roll during
-- a fast yaw ("pitches down due to gyroscopic nonsense"); the feedback loop rejects the
-- real gyroscopic disturbance on its own. OFF by default -- enable only for a fast,
-- clean-spinning craft that visibly needs the decoupling.
CFG.GYRO_COMPENSATION = true

-- Gyro comp is a feedforward computed from THIS tick's omega but applied NEXT tick, so
-- it carries a ~50 ms lag. At low yaw rate the nutation is slow and the lag is harmless
-- (comp cancels the coupling -> helps). As yaw rate rises the nutation speeds up until
-- the lagged comp arrives out of phase and INJECTS energy -> exponential nutation on a
-- sustained burn. So fade the comp out with |yaw rate|: full below LO, zero above HI
-- (rad/s). Above HI the bounded major-axis nutation + feedback carry it instead.
CFG.GYRO_FADE_LO = 0.15
CFG.GYRO_FADE_HI = 0.35

-- LEVEL PRIORITY. A flip is catastrophic; a yaw error is not. When the craft is
-- tilting (pitch+roll error large -- e.g. off-CoM yaw thrust starting to roll it over,
-- or an intermediate-axis tumble), fade the yaw command out so the pitch/roll loop
-- gets full authority to recover, then restore yaw once level. This stops "excessive
-- yaw flips the craft" WITHOUT having to model the yaw->roll actuator coupling. Yaw is
-- at full authority below CFG.TILT_YAW_MIN of tilt, zero above CFG.TILT_YAW_MAX (rad). Widen
-- (raise MAX) if it gives up yaw too eagerly; tighten (lower MAX) if a fast yaw still
-- tips it. Pair with a lower CFG.MAX_RATE_YAW so the yaw is gentle enough to begin with.
CFG.TILT_YAW_MIN = 0.20   -- rad (~11 deg): full yaw at/below this tilt
CFG.TILT_YAW_MAX = 0.60   -- rad (~34 deg): zero yaw at/above this tilt

-- EXTREME-ATTITUDE FIXES. Both are NO-OPS near level -- they only change behaviour at
-- large bank/pitch/inversion, which is otherwise mishandled.
--  * YAW_TILT_DECOMP (Bug 1): split the attitude error into a TILT error (bring body-up to
--    world-up) + a YAW error (wrapped heading), instead of one combined quaternion error.
--    A ~180 deg heading change is a routine half-turn, but as a single quaternion rotation
--    it is ~180 deg total -> ill-defined axis -> tiny tilt noise amplified into a pitch/roll
--    kick ("pitches up while yawing"). Decomposed, a half-turn stays PURE yaw. If LEVELLING
--    misbehaves with this on, the tilt sign is flipped -- set false (combined error) & tell me.
--  * TILT_ALT_ALLOC (Bug 2): "hold altitude" is a WORLD-up goal but the alt thruster is
--    body-fixed. Route the vertical effort by the true orientation: the alt thruster takes
--    the body-up share of world-up (auto-flips inverted; fades toward 90 deg pitch), and the
--    fore/aft + strafe thrusters take the rest (so a nose-up craft holds altitude on its
--    fore/aft thruster). Calibrate VERT_XFER_* by pitching/rolling ~45 deg and checking
--    altitude HOLDS -- flip a sign if it drops faster; raise the magnitude if lift is much
--    stronger than the horizontal thrusters so altitude sags when tilted.
CFG.YAW_TILT_DECOMP = true
CFG.TILT_ALT_ALLOC  = true
-- VERT_XFER_FWD is NEGATIVE because the ship's logical forward (the quaternion's body-fwd
-- axis this uz is dotted against) is 180 deg from thrust-forward (HEADING_OFFSET = pi), and
-- the spill is added AFTER THRUST_SIGN so it carries its own sign. Calibrated in-game: nose
-- DOWN 90 deg + climb wanted must fire the BACKWARD thruster (not forward). Flip back to +1
-- only if that test inverts.
--
-- SECOND SUSPECT FOR "CLIMBING AND DESCENDING SWAY IT IN ROLL AND PITCH" (2026-08-20), and the
-- reason these two knobs are the A/B test for it. The spill is proportional to vertEffort, so
-- it is nearly absent in level cruise (the loop is holding a small hover trim) and at its
-- largest in a commanded climb or descent -- which is exactly when the pilot sees the swaying.
-- The path is a genuine feedback loop: vertEffort spills into fore/aft and strafe by the craft's
-- TILT, those thrusters are off-CoM so they inject a pitch/roll moment, and that moment changes
-- the tilt that set the spill. Its loop gain rises with vertEffort, so a climb can push an
-- attitude loop that is merely marginal in the cruise over the edge.
-- TO TEST, no code change needed: set BOTH of these to 0 (or CFG.TILT_ALT_ALLOC = false) and fly
-- the same climb. The cost is only that altitude sags a little while tilted -- the alt thruster
-- keeps the body-up share either way.
--   * sway GONE with them at 0      -> it is this coupling; the attitude loop is fine.
--   * sway UNCHANGED with them at 0 -> it is the pitch/roll loop itself, and the climb was only
--                                      ever the disturbance that excited it. See CFG.ATT_KP_PITCH.
-- That second outcome is the more likely one, because the nose also nods in LEVEL flight, where
-- vertEffort -- and therefore this whole term -- is close to zero.
CFG.VERT_XFER_FWD   = -1.0  -- fore/aft steps per unit vertical-effort * (world-up . body-fwd)
CFG.VERT_XFER_STR   = 1.0   -- strafe   steps per unit vertical-effort * (world-up . body-lat)

-- Yaw -> pitch/roll ACTUATOR-coupling feedforward. An off-CoM yaw thruster produces a
-- pitch/roll torque as a mechanical side effect, proportional to how hard yaw fires --
-- and CFG.GYRO_COMPENSATION does NOT cancel this (it cancels the inertial omega x I omega,
-- not thruster geometry), which is why a crisper yaw worsens pitch. These pre-apply an
-- opposing pitch/roll step proportional to the actual yaw command, so pitch/roll never
-- have to chase the disturbance. CALIBRATE: hold level, command a steady yaw, and nudge
-- each until the pitch/roll kick at yaw onset disappears (sign matters). 0 = off.
CFG.YAW_TO_PITCH_FF = -0.0   -- pitch steps per yaw step
CFG.YAW_TO_ROLL_FF  = -0.0   -- roll  steps per yaw step

-- Attitude INTEGRAL -- trims a steady disturbance (a CoM/mass offset, an off-CoM
-- translation thruster) that a pure P/PD cascade leaves as a small standing tilt
-- (the craft "sits slightly nose-up", and `raw` is non-zero at rest). This is what
-- control.lua's tiny STAB_KI did, per body axis. It injects a torque/step BIAS --
-- NOT a velocity setpoint, which would demand rotation -- so at equilibrium it
-- holds the trim with zero error. Per axis it freezes + bleeds while THAT axis's
-- error is large (a deliberate manoeuvre or a recovery), so it never winds up
-- mid-turn. If the standing tilt persists, raise CFG.ATT_KI/CFG.PD_KI; if it slowly hunts
-- up and down, lower it.
CFG.ATT_INTEGRAL    = true
CFG.ATT_KI          = 1.0    -- INERTIA: accel bias (rad/s^2) per rad.s of integ. error
CFG.PD_KI           = 10.0   -- PD: step bias per rad.s of integrated error
CFG.ATT_I_LIMIT     = 2.0    -- clamp on |integral| per axis (rad.s), anti-windup
CFG.ATT_I_BLEED_ERR = 0.30   -- rad (~17 deg); above this on an axis, bleed not accumulate
CFG.ATT_I_RATE_GATE = 0.05   -- rad/s; FREEZE the integral on any axis rotating faster
                               -- than this. Integrating through motion lags the error by
                               -- ~90 deg and PUMPS a slow oscillation until it diverges
                               -- (the "pitches up/down greatly, then spins" on a nudge).
                               -- So the integral learns the steady trim only, at near-rest.

-- IDLE / PARKED trim-freeze. The attitude integral above is a NEAR-REST learner, which makes
-- a craft sitting on the ground -- engine master OFF, or simply parked with this computer left
-- running -- the very condition under which it accumulates: it sees a small standing tilt (the
-- craft resting askew, or the thrusters disabled so it cannot level itself) at zero rate and
-- winds the trim to ATT_I_LIMIT chasing an error it has NO authority to null. That bogus trim
-- then disturbs the NEXT flight. So detect "idle" -- quasi-stationary AND not under active
-- guidance -- and while idle FREEZE the trims and BLEED the attitude integral back toward zero,
-- so any parked wind-up has decayed by takeoff. Idle needs the craft both still (all three
-- speeds under their thresholds) and unguided; a nav target, an autoland approach/descent, or
-- an ALT-AP climb keeps it "active", so genuine in-flight trim learning (incl. a commanded
-- hover/hold) is untouched and flight behaviour is byte-for-byte unchanged. "landed" (sitting on
-- the pad) counts as parked. Calibrate so a parked craft reads idle within IDLE_SETTLE s while
-- the smallest real manoeuvre clears it at once (watch the "PARK" flag on the TRIM line).
CFG.ATT_I_IDLE_BLEED = 0.95   -- per-tick (~0.05 s) decay applied to the attitude trim while idle
CFG.IDLE_SPEED   = 0.6    -- m/s; horizontal speed below this counts as still
CFG.IDLE_VSPEED  = 0.4    -- m/s; vertical speed below this counts as still
CFG.IDLE_RATE    = 0.06   -- rad/s; total angular rate below this counts as still
CFG.IDLE_ALT_TOL = 1.0    -- m; |TARGET_ALT - posY| above this = an ALT-AP climb/descent (active)
CFG.IDLE_SETTLE  = 1.5    -- s of continuous stillness-and-unguided before idle latches

-- Sign / channel calibration (see header). All default to +1 / false.
CFG.PITCH_SIGN = 1
CFG.ROLL_SIGN  = -1
CFG.YAW_SIGN   = -1
CFG.SWAP_PITCH_ROLL_CHANNELS = false  -- true if pitch cmd should drive left/right

-- Rate-feedback sign, per axis -- SEPARATE from the output *_SIGN above. *_SIGN makes
-- a static tilt CORRECT (leveling, the P term); these make the rate DAMPING oppose
-- rotation (the D term). They must be independent: the orientation quaternion and the
-- angular-velocity sensor are different data sources whose sign conventions need not
-- agree, and *_SIGN flips P and D together -- so P can be right while D is
-- ANTI-DAMPING. That reads as: stable at rest, but EXPLODES WHEN NUDGED (the nudge
-- injects a rate the D term then amplifies). If an axis blows up only when perturbed,
-- flip ITS rate sign.
CFG.PITCH_RATE_SIGN = 1
CFG.YAW_RATE_SIGN   = 1
CFG.ROLL_RATE_SIGN  = 1

-- Heading reference, reused verbatim from navcom so its whole calibrated chain
-- (and the FMS's copy of it) still holds. heading = CFG.HEADING_SIGN*yaw + CFG.HEADING_OFFSET.
CFG.HEADING_SIGN   = 1
CFG.HEADING_OFFSET = math.pi   -- ship's logical forward is 180 from thrust-forward

-- Channel limits (redstone steps). Full 0..15 range now that nothing downstream
-- reserves the top step.
CFG.STAB_MAX   = 15
CFG.YAW_MAX    = 15
CFG.THRUST_MAX = 15
CFG.STRAFE_MAX = 15

-- ============================================================================
-- FLIGHT MODE -- how the craft holds course and handles turns / altitude changes.
-- Set in source; there is no runtime switch (edit + rerun to change).
--   "VTOL"      : the original holonomic-hover behaviour. The body is held wings-
--                 LEVEL at all times; EVERY bit of horizontal course correction is
--                 done by STRAFING; yaw swings the nose but the craft never banks.
--                 The bank/pitch overlays below are OFF (exact prior behaviour).
--   "HYBRID"    : a VTOL flown to LOOK like -- and steer like -- an aeroplane. In-course
--                 lateral correction is flown by BANKING toward the track (the tilted
--                 alt-thruster lift pulls it back on line, see the cross-track bank),
--                 NOT by strafing. Strafe is reserved for aligning to the pad in the
--                 close-in phases (TERMINAL/ARRIVED/LANDING), plus a SPARING amount
--                 through the banking phases to damp bank oscillation. Banks into turns
--                 and (optionally) pitches for climb. (Bank overlays ON; strafe = pad
--                 alignment + sparing damping.)
--   "FIXEDWING" : strafe thrusters COMPLETELY disabled -- lateral course is corrected
--                 ONLY by banking (toward the track) + the yaw law pointing the nose,
--                 which drag the craft round via the tilted alt-thruster "lift" in every
--                 phase, landing included. Banks and pitches like HYBRID. (Bank overlays
--                 ON, strafe OFF -- so it banks even in the pad phases.)
-- Anything unrecognised is treated as VTOL (safe default).
CFG.FLIGHT_MODE = "VTOL"

-- Coordinated-bank overlay (HYBRID/FIXEDWING only; a no-op in VTOL and whenever the
-- craft is already tracking its heading, i.e. straight-and-level). Commands a roll
-- angle proportional to the current heading error, so the craft leans INTO a turn
-- and rolls wings-level again once pointed at the target. It is added to the
-- attitude SETPOINT, so the leveling loop holds the bank and the alt-thruster tilt-
-- allocation turns that bank into the lateral "lift" that pulls the craft round.
-- CALIBRATE like the other axes: if it banks the WRONG way into a turn, flip
-- CFG.BANK_SIGN. This is the MASTER bank sign -- it multiplies BOTH the coordinated-turn
-- bank and the cross-track bank (they share the "positive = go right" convention and the
-- same roll->lateral-lift physics, so they invert together). RE-FLIPPED TO -1 (2026-07-28):
-- the +1 note below was from an earlier session and is now SUPERSEDED -- with the HYBRID
-- coordinated-turn heading sim wired in (CFG.HDG_FROM_BANK_SIGN, which faithfully turns the
-- nose whichever way the craft is ACTUALLY banked), the pilot directly observed BANK_SIGN=1
-- banking the WRONG way (left while pointed off the bearing needing a right turn, with the
-- nose then also swinging left in agreement -- i.e. the bank itself, not the turn-from-bank
-- coupling, was inverted). Confirmed correct since: with the cross-track term faded out beyond
-- 90 deg of heading error, HYBRID settled STABLY at 90 rather than running away to 180, which
-- can only happen if this (the sole remaining term there) pushes the right way.
-- CAUTION, a reasoning error made here once already: because BANK_SIGN multiplies BOTH bank
-- sources, it is IRRELEVANT to how they relate to each other -- flipping it inverts both and
-- leaves their RELATIVE sign untouched. So a symptom of the two FIGHTING each other can never
-- be explained, caused, or fixed by this constant; that is purely CFG.BANK_XTK_SIGN's job. Do
-- not "re-audit" BANK_XTK_SIGN as a stale compensation for a BANK_SIGN change -- it cannot be
-- one. (Original note, now superseded -- kept for history: "Set +1 in-game: at -1 the craft
-- banked AWAY from the target... Flip back to -1 only if it banks away again.")
CFG.BANK_SIGN       = -1
-- RESCALED 2026-07-28 from a COSMETIC overlay to the PRIMARY heading control (was 0.02).
-- In HYBRID the yaw loop is driven entirely by bankYawRate (see CFG.HDG_FROM_BANK_SIGN) --
-- profileRate(ey) is bypassed -- so this gain is the ONLY thing that makes the nose seek the
-- target at all; the whole heading loop is hErr -> bank -> g*tan(bank)/v -> heading. At 0.02
-- the bank could never exceed 0.02*pi ~ 0.06 rad even at 180 deg of error, giving a loop time
-- constant of v/(g*K) ~ 270 s (and <0.5 deg/s of turn at cruise): effectively OPEN LOOP. The
-- craft then flew its track perfectly (translation is holonomic and heading-agnostic) while
-- the nose sat at whatever angle it happened to hold -- reported as "flies point-to-point well
-- but faces 180 deg backwards", which is the signature of an unconstrained heading, not of a
-- wrong direction. 0.5 saturates BANK_MAX at ~90 deg of error and gives tau = v/(g*K) ~ 36 s
-- at 200 m/s. Lower it if the nose hunts/overshoots the bearing; raise it if it still converges
-- too lazily. NOTE this gain only bites in HYBRID -- VTOL/FIXEDWING still yaw via profileRate.
CFG.BANK_PER_YAWERR = 0.5
CFG.BANK_HDG_KD     = 1.2     -- rad of counter-bank per rad/s of actual yaw rate -- damps the
                              -- heading-error bank the same way BANK_XTK_KD damps the cross-track
                              -- one. Raised 0.1 -> 1.0 alongside BANK_PER_YAWERR: at the old gain
                              -- it removed ~1% of the commanded bank (inert). At 1.0 a full-bank
                              -- turn's own yaw rate trims the bank back by a useful fraction, so
                              -- the bank eases off as the turn is already closing rather than only
                              -- once hErr itself shrinks. If it rings, raise; if sluggish/won't
                              -- bank in enough to initiate the turn, lower toward 0.
                              -- NOTE (2026-08-19): this gain is now applied to the WASHED rate,
                              -- not the raw one -- see CFG.BANK_HDG_KD_WASHOUT. Retune it there
                              -- first; the value here no longer sets any standing droop.
-- Washout (high-pass) time constant, in seconds, for the rate the damper above sees. The
-- damper subtracts (yaw rate - a slow low-pass of the yaw rate), i.e. the CHANGE in turn
-- rate, instead of the raw rate.
--
-- WHY THIS EXISTS. Damping on the raw rate is correct for a heading CAPTURE, whose steady
-- state is wings level at zero rate, and wrong for a curved PATH, whose steady state is a
-- SUSTAINED rate. On an arc the old law settled at
--     bank = BANK_PER_YAWERR*hErr - BANK_HDG_KD*GRAVITY*tan(bank)/v
-- so the damper never let go: a standing droop of BANK_HDG_KD*GRAVITY*tan(bank)/v rad, which
-- (dividing by v) GROWS as the FMS slows for a corner -- worst exactly where the corner needs
-- the most bank. 14% of BANK_MAX at 120 m/s cruise, but 33/47/74% at the 51/35/23 m/s Mantine's
-- FMS schedules for the QATON/QETAM/BAZRA corners of WAU-BZR. That was the "HYBRID will not
-- turn on DSWAE-QATON-BAZRA" report: it banked ~30 deg where the plan wanted 45, flew each of
-- those three corners 2-3x wider than planned, and walked out past DC.PASS_CORRIDOR.
--
-- CHOOSING IT. Long next to the roll-in (BANK_MAX/BANK_RATE ~ 2.2 s) so the damper is fully
-- alive through the roll and through any ring, short next to a corner (5-15 s) so it has
-- washed out before the arc settles. 2.0 s sits between them. Raise it toward infinity to
-- reproduce the OLD drooping behaviour exactly (useful as an A/B in flight); lower it if the
-- roll-in itself still overshoots -- but below ~1 s it starts washing out DURING the roll-in
-- and stops damping the thing it is for.
CFG.BANK_HDG_KD_WASHOUT = 2.0
CFG.BANK_HDG_FADE_NEAR = 60    -- m; below this distance to the current target, the heading-error
                               -- bank term is fully faded to 0 -- the cross-track term (which
                               -- stays aligned with the translation law) becomes the sole lateral
                               -- corrector, avoiding a fight with a noisy near-target bearing.
CFG.BANK_HDG_FADE_FAR  = 200   -- m; above this distance, full heading-error bank authority.
                               -- FRESH tunables: raise FAR / lower NEAR if it still turns away
                               -- approaching a target; lower FAR toward NEAR if the coordinated
                               -- lean-into-the-turn look cuts out too early/too far from the target.
-- Cap on commanded bank. RAISED 2026-07-28: 0.25 rad is 14.3 deg, NOT the "~29 deg" the old
-- comment claimed. That silently capped every turn at a third of the rate the FMS plans for --
-- the flight plan plots each corner's radius as R = V^2/(GRAVITY*tan(DC.MAX_BANK_DEG)) with
-- DC.MAX_BANK_DEG = 45, so a UAC that can only reach 14 deg must fly WIDE of every planned
-- transition. Matched to the FMS's own assumption so the planned radius is actually flyable;
-- if these two ever disagree again, the FMS is the one to follow. CAVEAT: at 45 deg of bank
-- TILT_ALT_ALLOC leaves the alt thruster only cos(45) = 0.71 of its vertical authority (the
-- rest becomes the lateral lift that turns the craft), so expect some altitude sag in a
-- sustained hard turn -- lower BOTH this and DC.MAX_BANK_DEG together if that is too much.
CFG.BANK_MAX        = 0.785   -- rad (45 deg), matches the FMS's DC.MAX_BANK_DEG
-- Roll rate. RAISED 0.1 -> 0.35 (~20 deg/s, a brisk but ordinary roll): at 0.1 rad/s reaching
-- the new BANK_MAX took 7.9 s, comparable to the heading loop's own time constant, so the roll
-- slew -- not the aircraft -- would have been the dominant lag and the loop would hunt. Keep
-- this several times faster than tau = v/(GRAVITY*BANK_PER_YAWERR) or expect oscillation.
CFG.BANK_RATE       = 0.35   -- rad/s slew on the bank command (smooth roll in/out)

-- Pitch-for-altitude overlay (HYBRID/FIXEDWING only; a no-op in VTOL and in level
-- cruise). Commands a pitch angle proportional to the desired climb/descent rate,
-- so the nose rises to climb and drops to descend. The alt thruster still does the
-- real vertical work and the tilt-allocation keeps altitude held regardless; this
-- makes the motion LOOK right and, in FIXEDWING, lets the nose-up attitude convert
-- forward thrust into climb. Suppressed during autoland/LANDING so the craft keeps
-- a level pad approach. Flip CFG.PITCH_ALT_SIGN if the nose drops when it should
-- rise to climb.
-- The FIXEDWING vertical-by-pitch block below supersedes this overlay whenever it is live.
-- These four stay at 0 (all four ways of switching this overlay off); it remains the
-- HYBRID/VTOL cosmetic lean only. Never run both: two terms fighting over one pitch setpoint
-- is the pitch version of the failure the BANK_XTK_SIGN history documents at length.
CFG.PITCH_ALT_SIGN  = 0 -- -1
CFG.PITCH_PER_MPS   = 0.0--0.04   -- rad of commanded pitch per m/s of desired vertical rate
CFG.PITCH_ALT_MAX   = 0.0--0.3    -- rad (~23 deg) cap on commanded pitch
CFG.PITCH_ALT_RATE  = 0.0--0.2    -- rad/s slew on the pitch command

-- ============================================================================
-- FIXEDWING VERTICAL-BY-PITCH -- the aeroplane climbs by pointing the nose up
-- ============================================================================
-- INERT ON THIS AIRFRAME. Everything here is gated on CFG.FLIGHT_MODE == "FIXEDWING" and this
-- craft is VTOL, so not one of these values is read in normal operation -- they are defined
-- because uac.lua is shared byte-for-byte across the fleet and reads them unconditionally at
-- boot (the canary list names them for exactly that reason). They also become live the moment
-- someone cycles the on-screen MODE button round to FW, which is worth knowing before doing it.
--
-- WHAT IT DOES, on the aeroplane it was written for: with no lift thruster the only way to
-- change altitude is to fly at an angle to the horizon, so the altitude cascade's desired climb
-- rate stops being a demand on a vertical thruster and becomes a commanded PITCH ANGLE that the
-- stabiliser holds like any other attitude setpoint. The law is
--   pitch = (FF*vsCmd + KP*(vsCmd - vsActual) + KI*integral) / max(speed, MIN_SPEED)
-- -- dividing by the LIVE horizontal speed because tan(gamma) = vs/v, so the angle that buys a
-- given climb rate is inversely proportional to speed and a fixed rad-per-m/s gain would be
-- wrong by nearly an order of magnitude across this fleet's speed range. See #19's uacconf for
-- the full commentary; the values below are that airframe's, carried over unchanged.
CFG.FW_PITCH_ENABLE = true
-- Inherits the in-flight-calibrated CFG.KJOY_PITCH_SIGN's convention ("+command = nose up"),
-- which is the same physical thing -- not a fresh guess. Flip both together if ever wrong.
CFG.FW_PITCH_SIGN   = 1
CFG.FW_PITCH_FF     = 0.0    -- 1.0 = the honest geometric flight path angle for the commanded rate
CFG.FW_PITCH_KP     = 3.0   -- dimensionless CORRECTION on top of FF, not a second FF
CFG.FW_PITCH_KI     = 0.06   -- 1/s; weak on purpose -- a strong one pegs the nose up
CFG.FW_PITCH_I_LIMIT = 100.0 -- clamp on |integral| (m); angle contribution is KI*LIMIT/speed
CFG.FW_PITCH_MAX    = 0.45   -- rad (11.5 deg) hard stop; == CFG.TILT_YAW_MIN, so yaw never fades
-- Fraction of FW_PITCH_MAX the FEEDFORWARD alone may use. Also the SPEED PROTECTION: the cap
-- it makes (HEADROOM*MAX*speed) limits the commanded climb rate using the RAW speed, so a slow
-- aeroplane asks for less climb instead of mushing nose-high. See #19's uacconf for the story.
CFG.FW_PITCH_HEADROOM = 0.27
CFG.FW_PITCH_RATE   = 0.15   -- rad/s slew; faster buys nothing against a multi-second loop
CFG.FW_PITCH_MIN_SPEED = 20.0 -- m/s floor on the divisor (cf. CFG.HDG_SIM_MIN_SPEED)


-- Cross-track (lateral-correction) BANK -- HYBRID/FIXEDWING only. In these modes the
-- lateral course correction that VTOL flies by STRAFING is flown by BANKING instead:
-- the craft rolls toward the track and the tilted alt-thruster's sideways lift pulls it
-- back on line (the TILT_ALT_ALLOC spill, left UNcancelled while steering by bank -- see
-- there). This term sets that bank from the SAME body-right demand strafe would use (last
-- tick's latDemand), redirected into roll, so it is the PRIMARY in-course lateral
-- corrector; the heading-error coordinated bank above just layers a turn-lean on top.
-- Active only where the mode steers by bank (bankSteer): HYBRID in the non-pad phases,
-- FIXEDWING in every phase; zero in VTOL and during autoland. CALIBRATE CFG.BANK_XTK_SIGN
-- in-game like every other *_SIGN -- it is a GUESS combining several already-calibrated
-- signs: if the craft banks the WRONG way and drifts FURTHER off track (diverges), flip it.
-- BACK TO -1 (2026-07-28, second flip -- and this time DERIVED, not guessed). The +1 above
-- was wrong, and so was the reasoning that produced it ("it must have been compensating for
-- the BANK_SIGN bug"): BANK_SIGN is a COMMON multiplier on BOTH bank sources, so flipping it
-- flips them TOGETHER and can never change their RELATIVE sign. Only BANK_XTK_SIGN sets
-- whether the cross-track bank agrees or fights the heading-error bank. Exact derivation
-- (self-contained, chains through no unverified prose -- cf. [[navcom-sign-convention-debugging]]):
--   * bearing = atan2(dx,dz), so heading h has forward f = (sin h, cos h).
--   * every strafe law here projects onto r-component = V.z*sin h - V.x*cos h (lines
--     "matches xtk handedness"), i.e. the axis r = (-cos h, sin h).
--   * a target at bearing h+D therefore has r-component -sin(D). So POSITIVE heading error
--     gives a NEGATIVE body-right demand: the r-axis points toward DECREASING bearing.
-- Hence hErr=+D drives hdgBank = +BANK_PER_YAWERR*D but latDemand<0, so with STRAFE_SIGN=+1
-- and BANK_XTK_SIGN=+1 the cross-track bank came out NEGATIVE -- the two bank sources pointed
-- OPPOSITE ways for the same geometry, always, in every phase, and latBank (up to
-- BANK_PER_LAT*STRAFE_MAX ~ 0.225 rad) outweighs hdgBank (~0.03 rad at 90 deg) by ~7x, so the
-- wrong one won. Two independent in-game confirmations: (1) HYBRID settled STABLY ~90 deg off
-- the target -- and it being stable (rather than running away to 180) is itself proof hdgBank
-- pushes the right way while latBank pushes the other; (2) FIXEDWING banked visibly the WRONG
-- way, which isolates this cleanly since FIXEDWING has no HDG_SIM coupling at all (its yaw law
-- turns the nose correctly on its own) so the bank there is essentially pure latBank.
CFG.BANK_XTK_SIGN = -1
CFG.BANK_PER_LAT  = 0.015   -- rad of commanded bank per redstone-step of body-right demand
-- Cap on the cross-track bank component alone. CUT 0.5 -> 0.2 (2026-07-28): 0.5 rad is 28.6
-- deg, well over half the whole 45-deg BANK_MAX budget -- far too much authority for what is
-- only a fine TRIM now that BANK_PER_YAWERR gives the heading loop real authority (and the
-- FMS's 400 m carrot is pure-pursuit, so the commanded BEARING already self-corrects
-- cross-track on its own). Keeping this well below BANK_MAX guarantees the heading loop always
-- dominates and that a misbehaving cross-track term can never produce a full-scale bank.
CFG.BANK_LAT_MAX  = 0.2    -- rad (~11 deg)
-- Cross-track-RATE damping on the bank. Banking-to-track is a slow 2nd-order loop (bank ->
-- lateral lift -> velocity -> position) that SWAYS if driven on position error alone -- and
-- the P demand above SATURATES (clamps at BANK_LAT_MAX / STRAFE_MAX), which throws away the
-- little rate-damping baked into the strafe law exactly when off-track and closing fast. So
-- add an explicit counter-bank against the cross-track closing rate that survives that
-- saturation. Built with the SAME handedness as the strafe law's D term (STRAFE_SIGN*xtkRate)
-- so it rides on BANK_XTK_SIGN and needs no separate calibration.
-- CUT 1.5 -> 0.04 (2026-07-28). 1.5 caused a full-scale +/-45 deg sway on a dead-straight leg
-- that was completely INSENSITIVE to BANK_PER_YAWERR/BANK_HDG_KD -- the tell that the heading
-- term was not driving it. Two independent ways to see the old value was ~40x too big:
--   * it dwarfed the SAME damping already baked into the P path above, which contributes
--     BANK_PER_LAT*STRAFE_KD = 0.015*2.0 = 0.03 rad per m/s of xtkRate. 1.5 was 50x that.
--   * bandwidth: a bank produces lateral accel a ~ GRAVITY*tan(bank) ~ 11 m/s^2 per rad, so
--     rate feedback -K*xtkRate closes at 1/(11*K) seconds. K=1.5 demands a 0.06 s time
--     constant from an actuator whose real lag is ~0.3 s (50 ms tick + BANK_RATE roll slew +
--     the attitude loop) -- ~5x beyond what it can track, so it slammed rail to rail. It also
--     saturated BANK_LAT_MAX at a mere 0.33 m/s of drift, so ordinary velocity noise alone
--     pinned the bank full over. 0.04 keeps the closure ~2 s, safely inside the lag.
-- THIS IS THE THIRD TIME this exact trap has bitten this file (cf. MANNAV_YAW_BRAKE, and the
-- manual-yaw counter-steer buzz): you CANNOT damp faster than the loop bandwidth by adding
-- rate gain -- past the actuator's lag more gain converts damping into a limit cycle. If it
-- still sways, LOWER this, do not raise it.
CFG.BANK_XTK_KD   = 0.04  -- rad of counter-bank per m/s of cross-track closing rate (leg only)
-- REMOVED (2026-07-28): CFG.BANK_XTK_FADE_LO/HI, an |hErr| fade that cut cross-track bank
-- authority to zero at 90 deg of heading error. It was added to stop latBank "overriding" the
-- heading correction -- but that fight was the BANK_XTK_SIGN error above, not a magnitude
-- problem, and the fade made it strictly WORSE: it turned a sloppy fight into a hard STABLE
-- attractor at exactly 90 deg (inside 90 the larger, wrong-signed latBank pushed the error
-- out; at/beyond 90 latBank was faded away so hdgBank alone pushed it back in), which is
-- precisely the "aligns 90 degrees off" symptom that followed. With the signs agreeing there
-- is nothing left to fade: latBank IS the primary steering term (it is literally "which way
-- must I accelerate to reach the target, in body frame"), it is already bounded by
-- BANK_LAT_MAX and BANK_MAX, and starving it at large heading error would leave big turns
-- driven only by hdgBank -- at most BANK_PER_YAWERR*pi ~ 0.06 rad of bank, a fraction of a
-- degree per second of turn. Do NOT re-add a fade here to fix a directional symptom.

-- Coordinated-turn HEADING SIMULATION -- HYBRID only. A real fixed-wing aircraft turns
-- "for free" when banked: the tilted lift curves the flight path, and (per the FIXEDWING
-- lateral note above) the yaw law chasing the live bearing follows that curving path
-- round, so the nose visibly swings as a side effect of the bank alone. HYBRID is a
-- thrust-vectored VTOL merely commanding a cosmetic bank OVERLAY -- nothing otherwise
-- couples roll to yaw -- so it can sit banked while the ordinary yaw motion profile
-- chases desiredHeading at its own, physically-unrelated rate: bank and turn visibly
-- disagree. This reproduces the real relationship instead: for a bank angle theta at
-- airspeed v, the coordinated turn rate is omega = g*tan(theta)/v (equivalently radius
-- R = v^2/(g*tan(theta))), so the simulated yaw rate is DERIVED from whatever bank is
-- currently commanded -- both the heading-error lean and the cross-track bank feed it,
-- since if the craft is banked at all, for any reason, a real one would be turning. The
-- stabiliser's pitch loop is untouched -- like a pilot holding the nose level (in pitch)
-- through the bank -- this only ever drives yaw. Feeds the SAME "we already know the
-- rate, skip the profile" slot manYawRate uses for the hand-flown MIC yaw (see
-- attitude()'s wdy/py) -- manual yaw still wins if the pilot is actively holding the
-- stick. Zero (a no-op, falling back to the ordinary heading-error profile) outside
-- HYBRID's banking phases: VTOL, FIXEDWING, autoland, and HYBRID's wings-level pad
-- phases, exactly like the bank overlay it derives from.
CFG.HDG_SIM_ENABLE     = true
CFG.GRAVITY            = 11.0   -- m/s^2, Minecraft's value (matches the FMS's DC.GRAVITY)
CFG.HDG_FROM_BANK_SIGN = 1      -- CALIBRATION GUESS, like every other *_SIGN: if the nose
                                -- turns the WRONG way relative to which way it is leaning
                                -- (diverges instead of curving the same way it's banked), flip it.
                                -- RE-FLIPPED 2026-07-28: with BANK_SIGN confirmed correct (banks
                                -- right when it should), -1 here still turned the wrong way, so an
                                -- algebraic derivation from MANNAV_YAW_SIGN's comment (itself a
                                -- pre-calibration guess, not a confirmed fact) was unreliable --
                                -- corrected empirically instead, the same way every other *_SIGN
                                -- in this file is calibrated.
CFG.HDG_SIM_MIN_SPEED  = 5.0    -- m/s; floors the omega=g*tan(bank)/v divisor so a HYBRID
                                -- craft banked while slow or hovering doesn't demand a runaway rate
CFG.HDG_SIM_MAX_RATE   = CFG.MAX_RATE_YAW   -- rad/s cap -- same ceiling the yaw axis is tuned to

-- SPARING strafe for HYBRID's banking phases. Banking is a slow 2nd-order lateral response
-- (bank -> sideways lift -> velocity -> position) and tends to OSCILLATE about the track.
-- So HYBRID keeps a SMALL strafe authority through the banking phases -- mostly its rate/D
-- damping -- to settle that oscillation without taking lateral control back off the bank
-- ("strafe sparingly, to minimise banking oscillation"). FIXEDWING keeps NONE (strafe fully
-- off). FRAC -> 0 flies HYBRID like FIXEDWING (pure banking); FRAC -> 1 is more VTOL-like
-- strafe; MAX hard-caps the steps the sparing strafe may ever command so it can't dominate.
CFG.HYBRID_STRAFE_FRAC = 0.25   -- fraction of the full course strafe kept while banking
CFG.HYBRID_STRAFE_MAX  = 4      -- redstone-step cap on that sparing strafe

-- EAGER STRAFE for SMALL corrections -- hands lateral control back from bank to strafe once
-- the remaining correction is small. Added 2026-08-01 for: "HYBRID and FIXEDWING oscillate
-- the yaw when they should be near stand-still; VTOL doesn't."
--
-- Why the two banking modes ring at rest and VTOL never does. Near a target the heading-error
-- bank is ALREADY faded out (BANK_HDG_FADE_NEAR = 60 m), so latBank is the SOLE bank source
-- there -- and latBank is fed by lastLatDemand, the translation law's body-right demand, which
-- at a standstill is chasing nothing but velocity NOISE. In HYBRID that noise then goes through
-- HDG_SIM with v floored at HDG_SIM_MIN_SPEED (5 m/s, its smallest legal divisor = its LARGEST
-- gain), so a few steps of jitter demand omega = GRAVITY*tan(BANK_PER_LAT*jitter)/5 -- easily
-- the full HDG_SIM_MAX_RATE. A metre of drift buys a full-rate yaw command. VTOL is immune
-- because it simply strafes the metre out and never involves the nose at all.
--
-- So: below NEAR the correction is a job for translation, not for turning -- take latBank to
-- zero and give the strafe channel its FULL authority back (undoing HYBRID_STRAFE_FRAC/MAX),
-- restoring VTOL's behaviour in the small. FIXEDWING has no strafe thruster, so there the
-- strafe half is a deliberate NO-OP and only the bank half bites: the aircraft simply stops
-- trying to correct a trivial offset, which is exactly right -- no mini-swaying on the ground.
-- Ramp, not a switch, so nothing chatters at the boundary. (This is NOT the BANK_XTK_FADE
-- mistake above: that faded ONE of two OPPOSED terms on heading error, creating an attractor
-- where the correction reversed. Here both channels push the SAME way and the fade only
-- chooses which ACTUATOR flies it, so the boundary has no equilibrium to sit in.)
--
-- The measured "correction" is whatever the lateral law is actually nulling this tick: cross-
-- track |xtk| on a plain ENROUTE leg, else distance to the target (which, with the FMS's
-- carrot ~400 m ahead, stays far outside FAR all cruise -- so cruise banking is untouched).
-- Set BOTH to 0 to disable the feature entirely. Raise NEAR if it still twitches at rest;
-- lower it if HYBRID now strafes flat through corrections it should have banked.
CFG.STRAFE_EAGER_NEAR = 8    -- m; at/below this correction, full strafe and NO cross-track bank
CFG.STRAFE_EAGER_FAR  = 25   -- m; at/above this, the normal mode policy (bank steers)

-- ============================================================================
-- KEYBOARD JOYSTICK (#21 keyboardjoystick.lua, over KBDJOY_PROTOCOL) -- the pilot
-- hand-flies the craft on a virtual stick + throttle quadrant read off a typewriter.
-- The stick sends a pitch/roll TARGET (rad, trim-style: it holds where the pilot left it)
-- and a 0..1 throttle lever; this block decides what the UAC does with them.
--
-- LIVE ONLY WHEN ALL THREE HOLD -- see kjLive in controlLoop:
--   * CFG.FLIGHT_MODE == "FIXEDWING" (the pilot's own requirement). VTOL has no bank
--     overlay at all and HYBRID keeps a strafe/pad-alignment policy the stick would fight;
--     FIXEDWING is the mode where roll IS the lateral control, so a stick means something.
--     Runtime-switchable: the on-screen MODE button re-derives it, so tapping to FW arms
--     the stick and tapping past it disarms it, live.
--   * AP-NAV OFF and no autoland (the same handoff gate the MIC uses) -- the FMS and a
--     hand-held stick must never both own the attitude setpoint.
--   * a packet fresher than KJOY_STALE_AFTER -- a crashed/unplugged #21 must not leave a
--     stale bank latched in, so silence hands the craft straight back.
-- When live the stick REPLACES the autopilot's bank/pitch targets rather than adding to
-- them. Deliberate: this file's whole BANK_XTK_SIGN history is what happens when two terms
-- share one bankTarget, and a pilot's stick is not a term to be blended with.
CFG.KJOY_ENABLE      = true
CFG.KJOY_STALE_AFTER = 0.5    -- s; joystick packet older than this -> no manual attitude
-- Hard stops on what the stick may command (rad). +-60 deg per the #21 spec; keep in step
-- with PITCH_MAX/ROLL_MAX over there, which clamps at source. NOTE these deliberately
-- EXCEED CFG.BANK_MAX (45 deg, the autopilot's planned-corner limit): a pilot may bank
-- harder than the FMS ever plans for. At 60 deg TILT_ALT_ALLOC leaves the alt thruster only
-- cos(60) = 0.5 of its vertical authority, so expect real altitude sag in a sustained hard
-- hand-flown bank -- that is the aerodynamics being honest, not a bug.
CFG.KJOY_PITCH_MAX   = math.rad(60)
CFG.KJOY_ROLL_MAX    = math.rad(60)
-- Sign for the stick's PITCH command. It needs its own knob because the existing
-- CFG.PITCH_ALT_SIGN -- the sign attitude() normally applies to pitchCmd -- is 0, i.e. the
-- climb-pitch overlay is switched OFF; routing the stick through it would multiply the
-- pilot's input by zero and silently do nothing. So controlLoop passes THIS as attitude()'s
-- pitchCmdSign override whenever the stick is live. CALIBRATION, not a guess any more:
-- FLIPPED -1 -> +1 (2026-07-31) after the pilot found the stick inverted in flight and
-- corrected it by negating pitch in #21's broadcast; that fix was moved here, which is
-- exactly equivalent at the thrusters (ex = ex - SIGN*cmd, so negating the command and
-- negating the sign are the same thing) and keeps #21's display honest -- with the negation
-- in the broadcast, the stick box drew the cursor down while the readout said +60.
-- If E (nose up) ever drops the nose again, flip this, NOT #21.
CFG.KJOY_PITCH_SIGN  = 1
-- Sign for the stick's ROLL command. Multiplies bankTarget, which then rides the calibrated
-- CFG.BANK_SIGN. Also FLIPPED +1 -> -1 (2026-07-31), same session and the same reason as
-- KJOY_PITCH_SIGN above: it carries the roll half of that in-flight correction.
-- CAUTION, per BANK_SIGN's own note: this is the stick's PRIVATE sign, so it is the right
-- knob when the STICK alone works backwards. If the autopilot banks wrongly too, the fault
-- is upstream in BANK_SIGN/BANK_XTK_SIGN and flipping this will only mask half of it.
CFG.KJOY_ROLL_SIGN   = -1
-- Slew rate on the commanded attitude WHILE THE STICK IS LIVE (rad/s), replacing BANK_RATE
-- and PITCH_ALT_RATE. Two reasons it must override them, not add to them:
--   * PITCH_ALT_RATE is 0.0 (the disabled climb-pitch overlay). slew() with a zero step
--     freezes pitchCmd at 0 forever -- the stick's pitch would be eaten in silence.
--   * #21's own STICK_RATE (45 deg/s) IS the pilot's roll rate. A second, slower slew here
--     would just add lag between the stick and the craft. Keep this comfortably faster than
--     STICK_RATE so the UAC tracks the demand instead of becoming the bottleneck.
CFG.KJOY_SLEW_RATE   = 1.5    -- rad/s (~86 deg/s)
-- Throttle. A throttle LEVER names engine power, not a speed, so 0..1 maps straight onto
-- forward thruster steps -- no PID, no commanded airspeed (use the MIC #14 if you want
-- velocity control instead). One-sided: idle is 0, there is no reverse.
CFG.KJOY_THRUST_MAX  = CFG.THRUST_MAX   -- steps at full throttle; lower to detune the engine
-- Bank -> turn for the HAND-FLOWN craft. FIXEDWING normally needs no bank/yaw coupling
-- because, as CFG.HDG_SIM_ENABLE's note explains, its yaw law chases the live BEARING and
-- so follows the banked craft's curving path round on its own. But that argument assumes
-- there IS a bearing to chase -- with AP-NAV off, desiredHeading is frozen at the current
-- heading, so the yaw loop would hold the nose rigidly straight while the banked lift slid
-- the craft bodily sideways. Deriving the turn rate from the commanded bank restores "bank
-- and the nose follows", which is the entire point of a stick.
--
-- Structurally this is SAFER than the HYBRID heading sim it borrows from, despite feeding
-- the same wdy slot: HYBRID closes a real loop (hErr -> bank -> yaw -> hErr) and that loop
-- is what needed BANK_HDG_KD, the fade, and three sign flips. Here there is no loop at all
-- -- the bank comes from the pilot's fingers, so the pilot IS the outer loop. It reuses the
-- same GRAVITY / HDG_SIM_MIN_SPEED / HDG_SIM_MAX_RATE / HDG_FROM_BANK_SIGN calibration.
-- Set false for a pure heading-hold stick (bank leans the craft, nose stays put).
-- If the nose turns the WRONG way for the bank, that is HDG_FROM_BANK_SIGN, shared with
-- HYBRID -- flip it there and re-check HYBRID too, do not add a private sign here.
--
-- INTERACTION WITH THE MIC (#14), worth knowing before it surprises you: this feeds the same
-- "rate is already known, skip the heading profile" slot manYawRate does, so while the stick
-- is live yaw is RATE-controlled off the bank and the MIC's release-the-rudder-and-hold-this-
-- heading behaviour no longer applies (at wings level the rate command is simply 0, i.e. a
-- rate-null, which holds the nose steady but does not correct slow drift back). That is
-- ordinary aeroplane behaviour and the intended feel. Actively holding the MIC rudder still
-- wins over this (manYawRate takes priority); KJOY_BANK_TURN = false restores the heading-hold.
CFG.KJOY_BANK_TURN   = true
-- HOW MUCH turn a bank commands ------------------------------------------------------------
-- Worth being precise about what is and is not simulated here, because it decides which knob
-- to reach for. The PATH does curve correctly on its own: the alt thruster points along body-
-- up, so at bank b the altitude loop must hold F*cos(b) = mg, leaving a sideways component
-- F*sin(b) = mg*tan(b) -- exactly the centripetal force a real wing gives, hence the honest
-- omega = g*tan(b)/v. What the game does NOT simulate is any WEATHERVANING: a real fin drags
-- the nose round to follow that curving path, and nothing here does, which is why the yaw
-- thrusters have to be commanded at all (that is what KJOY_BANK_TURN above is for).
--
-- The catch is the magnitude. At CRUISE_SPEED(280) the honest rate is only 2.3 deg/s at 45 deg
-- of bank and 3.9 deg/s at 60 (a 4.1 km radius, 46 s to reverse course). That is correct
-- aerodynamics for something this fast -- not a bug -- but it is glacial to hand-fly, and
-- there are TWO reasons not to feel bound by it here:
--   * at steep bank the alt thruster has to make mg/cos(b) -- DOUBLE hover thrust at 60 deg --
--     so it saturates, the real sideways force falls short of mg*tan(b), and the actual path
--     curves LESS than the formula claims. The honest rate is optimistic in practice.
--   * the "wing" is a thruster and the engine points along the body, so yawing the nose drags
--     the velocity vector round after it. A thrust-vectored craft turns by pointing, not by
--     lift alone -- so leading the path is a legitimate way to fly this, not a fault.
--
-- KJOY_TURN_GAIN scales the derived rate. >1 makes the turn deliberately UNCOORDINATED (nose
-- inside the flight path -- a skid, which on a real aircraft is a fault and here is the
-- point). Start around 2-3 if the honest rate feels dead. HDG_SIM_MAX_RATE still caps the
-- result, so this cannot run away however high it is set.
CFG.KJOY_TURN_GAIN = 1.0
-- Speed used in the divisor. nil = the craft's TRUE ground speed, i.e. honest physics: the
-- same bank turns briskly when slow and lazily at cruise. Set a NUMBER to pin it, and turn
-- rate then depends on BANK ALONE -- full stick always turns at the same rate whatever the
-- throttle is doing, which is far more predictable to hand-fly. ~60 makes 60 deg of bank worth
-- ~18 deg/s at any speed. Affects ONLY the joystick; HYBRID's autopilot sim always uses the
-- true speed, since it is flying a planned radius the FMS computed from real physics.
-- SET TO 60 (2026-07-31) at the pilot's choice, i.e. hand-flying is deliberately NOT
-- speed-coordinated: 45 deg of bank turns at 18 deg/s and 60 deg saturates MAX_RATE_YAW
-- (28.6 deg/s), whether the craft is doing 30 m/s or cruise. Raise it toward CRUISE_SPEED for
-- lazier, more realistic turns; lower it for a twitchier stick.
CFG.KJOY_TURN_REF_SPEED = 60


-- ============================================================================
-- TRANSLATION CONFIG -- ported from navcom.lua (yaw removed; it is attitude now)
-- ============================================================================
-- A CEILING on the speed the FMS may schedule, and the value navCruiseCap boots at.
-- 999.0 -> SPEED_AT_FULL (2026-08-20). 999 was a "no ceiling" placeholder, and a ceiling
-- above what the thrusters can actually buy is not a ceiling: it only lets the loop carry a
-- permanent, unclosable error, and it boots the craft asking for a speed it can never reach.
-- No practical change -- the FMS's own CFG.SPEED_FULL is 320 -- but it keeps the one number
-- that says "how fast is this aeroplane" in ONE place, which is the whole point of the block
-- below. Raise BOTH this and SPEED_AT_FULL if thrusters are added.
CFG.CRUISE_SPEED       = 320.0
CFG.APPROACH_KP        = 0.20
CFG.APPROACH_SPEED     = 30.0
CFG.TERMINAL_MAX_SPEED = 10.0
-- NOTE: the manoeuvre slowdowns (nose-alignment gate, cross-track bleed) now live in the FMS
-- (#8 dynamicSpeedTarget) so that ONE place owns the commanded speed and its HSI CMD readout is
-- truthful. Do not re-add them here -- two sources would double-cut the throttle.
CFG.TRANSLATE_MIN_CMD  = 5.0
-- FORE/AFT half of the breakaway floor, in steps. SPLIT OFF from TRANSLATE_MIN_CMD above on
-- 2026-08-21: that one number is in OUTPUT STEPS, but a step is not worth the same demand on
-- the two axes it was applied to. Fore/aft, one step is SPEED_AT_FULL/THRUST_MAX; lateral,
-- STRAFE_AT_FULL/STRAFE_MAX. On Aer Togekiss that is 21.3 m/s against 2.67 -- eightfold -- so a
-- flat 5.0 asked for a reasonable 13 m/s sideways and an absurd 107 m/s fore/aft, i.e. 15 of the
-- 45 A-units #69 resolves, slammed on and off as the closing rate crossed TRANSLATE_MIN_RATE.
-- That was the fore-aft rocking. The floor is blended along the demand's direction (see uac.lua),
-- so this value is what a PURELY fore/aft demand is floored to and TRANSLATE_MIN_CMD what a
-- purely lateral one is; set them equal to get exactly the old flat behaviour back.
--
-- CALIBRATE by what the floor should be WORTH, not by the step count: multiply by
-- SPEED_AT_FULL/THRUST_MAX to read it in m/s. Too low and the craft creeps at a fix instead of
-- breaking away; too high and it lunges and rocks again -- that is the failure this split fixes,
-- so come DOWN to it rather than up.
-- Aer Togekiss: 0.5 steps = 1.5 of #69's 45 A-units ~ 10.7 m/s of demand, which is
-- TERMINAL_MAX_SPEED (10.0) -- enough to always get moving, nowhere near a lunge.
CFG.TRANSLATE_MIN_CMD_FWD = 0.5
CFG.TRANSLATE_DEADZONE = 1.0
CFG.TRANSLATE_MIN_RATE = 0.35

-- ---------------------------------------------------------------------------
-- HORIZONTAL AUTHORITY -- the two measured numbers the whole translation loop scales from.
--
-- These are PHYSICAL PROPERTIES OF THIS AEROPLANE, exactly like CFG.CLIMB_AT_FULL in the
-- altitude section below: the steady speed that CFG.THRUST_MAX steps of fore/aft, and
-- CFG.STRAFE_MAX steps of strafe, actually buy. They are not preferences, and they change
-- every time thrusters are added.
--
-- WHY THIS BLOCK EXISTS (2026-08-20). The vertical channel was rescaled on 2026-08-20 because
-- its gains were written in absolute redstone steps with no reference to what a step buys, so
-- a thruster upgrade silently doubled the loop gain and it hunted. The HORIZONTAL channel had
-- the identical fault and worse: it had never been scaled to anything at all. The proof is
-- that #1 (superwide VTOL, 320 m/s), #38 Mantine (160), #27 SS Argo (100) and #19 Citation
-- (280) all carried the SAME BLOCK, byte for byte:
--
--     SPD_KP 1.5   SPD_KI 0.40   SPD_KD 1.0001   SPD_FF 0.0
--     STRAFE_KP 1.2   STRAFE_KD 2.0   TRANS_KP 0.75
--
-- Four airframes with nothing in common cannot share a steps-per-m/s gain, for the same
-- reason they could not share CLIMB_AT_FULL = 5.0. What that gain is worth here:
--
--     STEPS_PER_MPS_FWD = 15 / 320 = 0.047 steps per m/s
--     SPD_KP 1.5 = 32x that.  TRANS_KP 0.75 = 16x.  STRAFE_KP 1.2 = 3.2x.
--
-- A P gain 32x the plant's own scale is not a proportional controller, it is a RELAY: at
-- SPD_KP 1.5 the whole 15-step range is crossed in 10 m/s of speed error, so anything outside
-- +-10 m/s commands full thrust or full reverse.
--
-- BE PRECISE ABOUT WHICH OF THESE IS THE SURGE, because the first guess was wrong. Benched
-- (below), the 32x P gain on its own is NOT what oscillates -- with SPD_KD at navcom's 0.0 it
-- settles in one overshoot. SPD_KD = 1.0001 is what oscillates, and it does so violently. The
-- over-scaled gains are the reason the channel had no MARGIN left to absorb it, and the reason
-- nobody could tell which knob was at fault; they are worth fixing, but the airspeed surge
-- itself is the derivative. See CFG.SPD_KD.
--
-- MEASURE THEM. Both are one flight test each, and everything below rescales off them:
--
--   1. Calm air, plenty of altitude, wings level, AP-NAV OFF so the MIC stick (#14) owns the
--      horizontal (a released stick is a velocity-hold at zero, so centre it to stop).
--   2. Full FORWARD on the stick; hold until the ground speed (GS on the UAC display, or the
--      HSI) STOPS INCREASING. That steady speed is SPEED_AT_FULL. Repeat full aft; use the
--      smaller of the two.
--   3. Same again full LEFT and full RIGHT for STRAFE_AT_FULL.
--
-- SPEED_AT_FULL 320 is taken from the FMS's own CFG.SPEED_FULL (8/apconf.lua), which is the
-- fastest cruise this aeroplane is flown at -- a good figure, but chosen for chunk loading,
-- not measured at the thrusters, so confirm it. STRAFE_AT_FULL 40 IS A GUESS: nothing in the
-- tree records what this craft's strafe thrusters are worth, and the strafe channel could not
-- be flight-tested when this block was written. Measure it before trusting any strafe number
-- here -- every strafe gain below is proportional to it, so one measurement retunes them all.
CFG.SPEED_AT_FULL     = 320.0
CFG.STRAFE_AT_FULL    = 40.0
-- Inverse plant gain: the steps that buy one m/s in the steady state. This is the fore/aft and
-- lateral twin of CFG.STEPS_PER_MPS, and it is used for BOTH jobs the vertical channel uses
-- that number for -- as the FEEDFORWARD (below and in uac.lua's velocity-vector laws) and as
-- the SCALE every feedback gain in this section is expressed in.
CFG.STEPS_PER_MPS_FWD = CFG.THRUST_MAX / CFG.SPEED_AT_FULL   -- 0.047 steps per m/s
CFG.STEPS_PER_MPS_LAT = CFG.STRAFE_MAX / CFG.STRAFE_AT_FULL  -- 0.375 steps per m/s

-- ---- ENROUTE along-track loop (speedPID) ----
-- FEEDFORWARD, ON AT LAST. The code has multiplied CFG.SPD_FF by the commanded closing speed
-- since navcom, and every airframe in the fleet has carried it at 0.0 -- i.e. the feedback
-- alone had to manufacture the entire cruise thrust out of the tracking error. That is what
-- forced SPD_KP to 32x the plant scale in the first place, and it is the same "the feedforward
-- and the feedback must scale off the SAME authority" fault the vertical channel was fixed for.
-- At exactly STEPS_PER_MPS_FWD this is an inverse-plant feedforward: on its own it commands
-- precisely the steps that hold the commanded speed, so P and I are left with nothing to do
-- but trim mass changes, bank, and climb attitude -- which is why they can now be gentle.
CFG.SPD_FF = CFG.STEPS_PER_MPS_FWD
-- P and I in MULTIPLES OF THE PLANT SCALE, so they follow SPEED_AT_FULL automatically -- the
-- half of the vertical fix that mattered most (correcting the authority while leaving the
-- feedback a bare constant is half a fix, and the half that hunts).
-- BENCHED, not guessed (first-order thrust plant + 1-tick actuator lag, 20 Hz, 0 -> 150 m/s,
-- drag time constant swept 6/15/30 s). Worst tracking error after the first 5 s, number of
-- error sign reversals, and command chatter (output reversing direction tick to tick):
--
--     kp 1.5    ki 0.40  kd 1.0001  ff 0        16.04 m/s   2734 revs   1464 chatter
--     kp 1.5    ki 0.40  kd 0       ff 0         3.02 m/s      1 rev      47 chatter
--     KP_NORM 2   ki 0.023  kd 0    ff on       13.60 m/s     10 revs     15 chatter
--     KP_NORM 8   ki 0.023  kd 0    ff on        0.60 m/s      1 rev       0 chatter
--     KP_NORM 16  ki 0.023  kd 0    ff on        0.08 m/s      1 rev       0 chatter
--     KP_NORM 32  ki 0.023  kd 0    ff on        0.00 m/s      1 rev       0 chatter
--
-- Read it carefully: cutting the gain HARD (KP_NORM 2, the first cut tried here) is WORSE than
-- leaving it alone -- the feedforward then over-commands through the transient and the feedback
-- is too slow to catch it. The gain is not the fault; see the SPD_KD note below.
-- KP_NORM 16 = kp 0.75 = HALF the historic 1.5: bench-clean, and the halving is margin for what
-- the bench cannot model (relay hop, redstone granularity, thruster spool, quadratic drag).
-- KP_NORM 32 would restore 1.5 exactly if it turns out sluggish in the air.
-- KI_NORM 0.5 = ki 0.023 beat 1.0 / 2.0 / 4.0 (which reintroduced 100+ chatter), and still
-- closes the steady state exactly when the feedforward is 25% wrong (2.06 m/s worst, 1 chatter)
-- -- which matters, because SPEED_AT_FULL is not yet measured.
CFG.SPD_KP_NORM = 16.0
CFG.SPD_KI_NORM = 0.5
CFG.SPD_KP = CFG.SPD_KP_NORM * CFG.STEPS_PER_MPS_FWD
CFG.SPD_KI = CFG.SPD_KI_NORM * CFG.STEPS_PER_MPS_FWD
-- 1.0001 -> 0. THIS IS THE AIRSPEED OSCILLATION, and it was never meant to be here: the
-- ancestor 4/navcom.lua sets `SPD_KD = 0.0  -- leave 0; the derivative here is acceleration,
-- and noisy`, uac.lua carried 0.0 through v1.0, and v1.4 (2026-07-27) raised it to 1.0001 with
-- NO comment, in the same edit that put SPD_KP back to 1.5 -- i.e. the gain was restored and a
-- derivative bolted on to damp it, which is the wrong half of the fix twice over.
--
-- WHY IT OSCILLATES, not merely "is noisy". pid.lua differences the ERROR and does not filter
-- it (see CFG.VEL_KD, zeroed for the same reason), and on a SPEED loop the error's derivative
-- IS the craft's acceleration -- which is produced by this loop's own last output. So with A =
-- m/s^2 per step, the law reduces to u(n) = Kp*e + Ki*I - KD*A*u(n-1): a recurrence whose pole
-- is -KD*A. At KD ~ 1 it alternates sign every tick and DIVERGES as soon as one step is worth
-- more than 1 m/s^2, until the +-15 clamp turns it into a sustained limit cycle. It bites
-- hardest at low speed, where there is no drag opposing the thrust and A is at its largest --
-- which is exactly "especially noticeable on starting from full stop".
--
-- The bench above is unambiguous: the SAME gains with kd 1.0001 vs kd 0 give 2734 error sign
-- reversals against 1, and 1464 command reversals against 47. Everything else in this block is
-- scaling and margin; THIS line is the airspeed oscillation.
-- Leave it at 0. If the loop still hunts, lower SPD_KP_NORM; the damping is supposed to come
-- from the gain being right, not from differencing a noisy speed 20 times a second.
CFG.SPD_KD = 0.0
CFG.CRUISE_TRIM_INIT = 0

-- ---- Cross-track (strafe) loop ----
-- Read the PD as the cascade it actually is. With sp = 0 the law in uac.lua is
--     steps = -STRAFE_KP*xtk - STRAFE_KD*xtkRate  =  -STRAFE_KD * ( (KP/KD)*xtk + xtkRate )
-- i.e. an OUTER position gain KP/KD (1/s -- the lateral closing speed wanted per metre off
-- track) driving an INNER lateral-velocity gain KD (steps per m/s). Written that way it is the
-- same shape as the vertical cascade, and the two halves can be given their proper units:
--   * XTK_KP is the outer gain and is a pure 1/s -- it does NOT depend on thruster authority,
--     so it is an absolute number and is LEFT AT ITS HISTORIC VALUE (1.2/2.0 = 0.6). Path
--     tracking behaves as it always has.
--   * STRAFE_VEL_KP is the inner gain and is steps per m/s, so it MUST scale with
--     STEPS_PER_MPS_LAT, and it is the one number the strafe channel now derives from.
-- THE PILOT'S GUESS WAS RIGHT -- strafe does share the fault, and it shares the CAUSE with yaw.
-- Reported: strafe holding inside +-3 steps of 15 and oscillating on the same ~2 s period as the
-- yaw. Same period, same non-saturating command, same mechanism: an inner velocity loop sitting
-- on its stability boundary behind the ~0.5 s transport delay derived in CFG.ATT_KP_YAW.
--
-- THE ONE IDENTITY WORTH REMEMBERING, and the reason normalising the gains was worth doing at
-- all. The inner loop's true gain is
--     G = (steps per m/s) * (m/s^2 per step) = *_KP_NORM / tau
-- where tau is that axis's drag time constant. STRAFE_AT_FULL CANCELS OUT -- so the normalised
-- gain is exactly "loop gain times tau", the authority guess does not enter the stability
-- question at all, and every channel in this file can be read off the same scale. With T = 0.5 s
-- of delay the boundary is G = pi/(2T) = 3.14 and 60 deg of phase margin wants G ~ 1.05, i.e.
--     KP_NORM ~= tau       for every one of these loops.
-- Fore/aft has tau ~15 s and SPD_KP_NORM 16 -- which is why that channel is fine. Strafe's tau
-- is short (~1.7 s if it is exactly on the boundary today), and it was carrying NORM 5.33.
--
-- Simulated 30 m capture WITH the 0.5 s delay, lateral tau swept 1.0/1.7/4.0 s (crossings):
--
--     VEL_NORM 5.33  XTK_KP 0.60   (the historic 1.2/2.0)    150   138     0
--     VEL_NORM 2.00  XTK_KP 0.30                               0     0     0
--
-- 138-150 crossings is not a cross-track hold, it is a weave. NORM 2.0 is clean with zero
-- overshoot across the whole 4x range of lateral drag -- i.e. it stays clean however wrong the
-- STRAFE_AT_FULL guess turns out to be -- and recaptures 30 m in ~14 s.
--
-- CORRECTION, worth recording so it is not repeated: the first cut of this block bench-tuned
-- these two WITHOUT the transport delay in the model and concluded the opposite -- that the
-- outer/inner RATIO was at fault and the inner gain should be RAISED to VEL_NORM 10. With the
-- delay in, VEL_NORM 10 is three times past the stability boundary. A cascade bench with no
-- dead time cannot see the thing that limits every loop here; do not tune this channel with one.
CFG.XTK_KP             = 0.30   -- 1/s: lateral closing speed wanted per metre of cross-track
CFG.STRAFE_VEL_KP_NORM = 2.0
CFG.STRAFE_VEL_KP = CFG.STRAFE_VEL_KP_NORM * CFG.STEPS_PER_MPS_LAT   -- steps per m/s
CFG.STRAFE_KP = CFG.XTK_KP * CFG.STRAFE_VEL_KP                       -- steps per metre of xtk
CFG.STRAFE_KI = 0.0
-- The inner gain again, under the name uac.lua's cross-track law reads it by. It is the SAME
-- NUMBER as STRAFE_VEL_KP on purpose: both are "steps per m/s of body-right velocity", and
-- carrying two different values for one physical quantity (2.0 here, 1.2 in the velocity laws)
-- was the other half of the strafe channel's confusion. Unlike SPD_KD this derivative is taken
-- from the MEASURED cross-track rate, not from a differenced error, so it is real damping and
-- stays.
CFG.STRAFE_KD = CFG.STRAFE_VEL_KP
-- TRANS_KP: gain for the UNDAMPED direct-velocity-error laws -- align-to-point
-- (TERMINAL/ARRIVED/LANDING/APPROACH, the en-route CARROT, and direct-to with no leg) and the
-- manual MIC stick law. They compute thrust as a bare Kp*error with no I and no D, so unlike
-- speedPID they have nothing to damp a too-high gain. This is the number that was already cut
-- once for the 2x fore/aft thruster upgrade (1.5 -> 0.75, same story as ROT_GAIN_YAW).
--
-- LEFT AT 0.75 (TRANS_KP_NORM 16 reproduces it exactly), and what changed instead is that
-- uac.lua now gives this law the same inverse-plant feedforward the ENROUTE law gets --
-- STEPS_PER_MPS_FWD times the commanded body-frame velocity. Note WHICH law this is: with a
-- baked path the FMS flies a CARROT, so this -- not speedPID -- is the law that holds cruise,
-- and it is a bare P with no integral at all. Benched (0 -> 150 m/s, tau 15 s), that shows up
-- as a standing DROOP, because a pure P must hold an error to hold thrust:
--
--     TRANS_KP 0.75, no feedforward        steady error 8.82 m/s
--     TRANS_KP 0.75, feedforward on        steady error 0.04 m/s
--     TRANS_KP 0.75, feedforward 25% wrong steady error 2.21 m/s
--
-- 8.8 m/s slow, with the HSI reading the commanded number -- the FMS's speed schedule was being
-- quietly undershot at every carrot. Halving the gain on top of this was tried and reverted:
-- with the droop gone there is nothing left for a smaller gain to buy, and the close-in
-- align-to-point phases want the authority. Lower TRANS_KP_NORM if align-to-point or the manual
-- stick hunts; the middle column above is why measuring SPEED_AT_FULL is worth the flight.
-- Close-in station keeping is unaffected either way: below CFG.TRANSLATE_MIN_RATE,
-- CFG.TRANSLATE_MIN_CMD floors the command at 5 steps regardless of what this gain computes.
CFG.TRANS_KP_NORM = 16.0
CFG.TRANS_KP = CFG.TRANS_KP_NORM * CFG.STEPS_PER_MPS_FWD
CFG.THRUST_SIGN = 1
CFG.STRAFE_SIGN = 1
CFG.XTK_SIGN    = 1
CFG.WP_EPSILON  = 0.5

-- ============================================================================
-- ALTITUDE CONFIG -- ported from control.lua (the vertical cascade + autoland)
-- ============================================================================
-- THE ALTITUDE DATUM. How far the pose `sublevel` reports sits ABOVE THE BOTTOM OF THE HULL,
-- in metres. uac.lua subtracts it on the first line of every tick, so every altitude in this
-- system -- what the cascade holds, what the ALT-AP panel picks, what the HSI and the PFD
-- print, what the responder tells ATC, what the GPWS measures its clearance from -- is the
-- world Y of the part of the aeroplane that touches the ground.
--
-- WHY IT HAS TO BE SUBTRACTED. `sublevel` reports the assembly's pose, which is somewhere
-- inside the hull; the aeroplane's altitude is where its FEET are. Every number the aircraft
-- compares itself against is a piece of GROUND -- landing.db's pad and runway elevations are
-- surveyed surface Y, and so is the terrain server's profile -- so an uncorrected reading is
-- an aeroplane that believes it is 9 m higher than it is. Aer Togekiss parked on the
-- Y=63 airstrip read 73. What that cost, before this was measured:
--   * the autoland flew its descent to a pad elevation 9 m UNDERGROUND, and only the
--     squat switch stopped it there;
--   * the GPWS's radio altitude over-read by the same 9 m on every SITE and TERR
--     datum, so every terrain-closure envelope fired that much late -- and this is the one
--     that matters, because the whole point of that computer is to be early;
--   * ATC painted the fleet 9 m high, each hull by a different amount.
--
-- HOW TO MEASURE IT. Set it to 0, park the aeroplane on ground of a KNOWN Y (an aerodrome in
-- landing.db is ideal -- TMV/TMA are Y=64), and read the altitude off the PFD or the UAC's
-- own display. The offset is (what it read) - (the Y the hull is resting on). It is a
-- property of the BUILD, so re-measure it whenever the hull grows a deck underneath.
--
-- POSITIVE means the reported pose is above the bottom, which is the only case seen so far;
-- the arithmetic is a plain subtraction and would take a negative just as happily.
CFG.ALT_OFFSET = 9
-- Measured 2026-08-29: parked on the Y=63 airstrip, Aer Togekiss reported alt 73.

-- DOES THIS AIRFRAME HAVE LIFT THRUSTERS AT ALL? A physical fact about the aeroplane, NOT a
-- mode: FLIGHT_MODE says how the craft is flown, this says what it is built out of, and the
-- two are set independently. TRUE here -- this craft has up/down thrusters and everything
-- below runs exactly as it always has. It is FALSE on the Citation (#19), whose only vertical
-- control is the pitch angle; with it false uac.lua skips the inner climb-rate loop and the
-- tilt allocation entirely and CFG.FW_PITCH_* flies the vertical instead.
CFG.HAS_ALT_THRUSTER = true
-- The vertical channel now drives the up- and down-firing thrusters directly on
-- two separate redstone lines (see sendAlt) -- the old single 0..14 line + external
-- splitter box is gone. ALT_NEUTRAL is the balanced-hover level BOTH lines sit at
-- when the cascade wants zero net effort; +steps drives up, -steps drives down.
CFG.ALT_NEUTRAL = 7
CFG.ALT_MIN, CFG.ALT_MAX = 0, 15
-- math.max (not min): with neutral 7 in a 0..15 range the up side has 8 steps of
-- headroom, so full effort reaches level 15 and the down line reaches 0.
CFG.ALT_SWING   = math.max(CFG.ALT_NEUTRAL - CFG.ALT_MIN, CFG.ALT_MAX - CFG.ALT_NEUTRAL)
-- ---------------------------------------------------------------------------
-- VERTICAL AUTHORITY -- the one measured number the whole vertical loop scales from.
--
-- CLIMB_AT_FULL is a PHYSICAL PROPERTY OF THIS AEROPLANE: the climb rate ALT_SWING steps
-- of lift actually buys. It is not a preference, and it changes every time lift thrusters
-- are added.
--
-- WHY IT WAS WRONG, AND WHAT IT COST. It sat at 5.0 -- the number ported from control.tua
-- -- and 5.0 is still what #38 Mantine, #27 SS Argo and #19 Citation all carry. A massive
-- superwide VTOL, a small winged one, a sailless watership and a Cessna cannot all buy the
-- same climb rate per step. Meanwhile this airframe's OTHER channels were rescaled as it
-- was upgraded, and the comments beside them say so:
--
--     TRANS_KP    1.5  -> 0.75    "cut for the 2x fore/aft thruster upgrade"
--     ROT_GAIN_YAW 5e-6 -> 1.7e-6 "/3 after ~3x more yaw thrusters"
--
-- The vertical channel never got that treatment. More lift thrusters means more
-- acceleration per step, which raises the vertical loop gain by the same factor, and a loop
-- whose gain quietly went up by 2x is a loop that hunts. It hunted: VS swinging -5/+5 m/s
-- on a ~2 s period, in level cruise.
--
-- Note what the symptom says about the number. The loop was TRYING TO HOLD ZERO, using
-- about five of its eight steps to do it, and the aeroplane still reached 5 m/s. If
-- ALT_SWING really bought 5 m/s, five steps could not have. Full authority is therefore
-- well above 5, and 10 is the conservative reading of that -- which is also exactly the
-- "2x the thrusters, half the gain" precedent above.
--
-- 10.0 IS AN ESTIMATE FROM THE SYMPTOM, NOT A MEASUREMENT. Measure it and put the real
-- number here:
--
--   1. Calm air, plenty of altitude, AP-ALT OFF so the manual VSC (#6) owns the vertical.
--   2. Full up on the VS stick, hold it until the climb rate STOPS INCREASING.
--   3. That steady rate is CLIMB_AT_FULL. Repeat with full down; use the smaller of the two.
--
-- Everything below scales off it, so that one measurement retunes the whole channel.
CFG.CLIMB_AT_FULL = 10.0
CFG.STEPS_PER_MPS = CFG.ALT_SWING / CFG.CLIMB_AT_FULL
CFG.ALT_KP      = 0.3

-- The commanded-rate envelope is deliberately ABSOLUTE now, not 0.75 * CLIMB_AT_FULL.
-- Correcting the authority above must not silently change how fast the aeroplane is
-- ALLOWED to climb -- that is a handling decision, and it was 3.75 m/s before this edit and
-- is 3.75 m/s after it. Raise these on their own if a brisker vertical is wanted.
CFG.CLIMB_MAX   = 3.75
CFG.DESCEND_MAX = 3.75

-- ...and the MANUAL vertical's own envelope, which is deliberately NOT the two above.
-- ADDED 2026-09-08 with mancon.lua's boost key (#6's CFG.KEYS.BOOST, A). The stick has two
-- ceilings now -- CFG.VS.MAX_CMD 3.75 unboosted, CFG.VS.BOOST_MAX_CMD 7.5 held -- and the
-- boosted one has to get past the clamp uac.lua applies before its rate loop ever sees the
-- number. Raising CLIMB_MAX to buy that would have loosened the envelope the AUTOLAND and
-- ALT-AP fly inside as well (CFG.AL_DESC_MAX / CFG.AL_CLIMB_MAX are literally these two), so
-- the manual branch clamps here instead and the autopilots keep the 3.75 they were tuned at.
--
-- 7.5 = twice the unboosted stick, and still comfortably under CFG.CLIMB_AT_FULL (10.0), so
-- it is a rate the lift bank can actually buy rather than one it saturates against. THE TWO
-- FILES MUST AGREE: this is the ceiling, and 6/manconconf.lua's BOOST_MAX_CMD is what asks
-- for it -- set that higher than this and the extra travel is silently thrown away here.
-- If CLIMB_AT_FULL is ever measured properly (CLAUDE.TODO.md #1) this is one of the numbers
-- to revisit: it should stay below whatever that turns out to be.
CFG.MANCON_CLIMB_MAX   = 7.5
CFG.MANCON_DESCEND_MAX = 7.5

-- The inner climb-rate loop, in STEPS PER m/s OF ERROR.
--
-- Expressed as a multiple of STEPS_PER_MPS rather than as a bare number, so that it scales
-- with the airframe's authority automatically. That is the whole point of this edit: with
-- VEL_KP written as a constant, correcting CLIMB_AT_FULL fixed the FEEDFORWARD and left the
-- FEEDBACK at its old, now-doubled loop gain -- half a fix, and the half that hunts.
--
-- 0.625 * STEPS_PER_MPS reproduces the historical VEL_KP = 1.0 exactly at CLIMB_AT_FULL =
-- 5.0, so this expression changed nothing on its own; the authority correction above is
-- what halves it, to 0.5.
CFG.VEL_KP_NORM = 0.625
CFG.VEL_KP = CFG.VEL_KP_NORM * CFG.STEPS_PER_MPS
CFG.VEL_KI = 0.5
-- Still 0. pid.lua takes its derivative from the ERROR and does not filter it, so on a rate
-- loop at 20 Hz this is differenced acceleration with no smoothing -- noise, not damping.
-- If the loop still hunts after the authority above is measured properly, LOWER VEL_KP_NORM
-- rather than reaching for this.
CFG.VEL_KD = 0.0
CFG.ALT_TRIM_INIT = 0
CFG.MASS_STEP_THRESHOLD = 50.0
CFG.TRIM_BOOST_GAIN     = 4.0
CFG.TRIM_BOOST_TIME     = 3.0
CFG.AP_ALT_MIN, CFG.AP_ALT_MAX = 5, 999
CFG.AL_DESC_MAX    = CFG.DESCEND_MAX
CFG.AL_CLIMB_MAX   = CFG.CLIMB_MAX
CFG.AL_STALE_AFTER = 3.0
-- Standing down-command (m/s) held while the aircraft is on its gear with the squat contact
-- made -- see alGrounded in uac.lua. It is not a descent, it is a bias: it keeps the lift
-- thrusters from ever answering a settle by firing UP, which is what unloads the gear and
-- makes a craft skittish on the ground (and, on a runway rollout, stops the wheels braking).
-- The velocity loop's integral is frozen while it applies, so this can never wind up.
-- 0 disables the down-bias without re-enabling the altitude hold; raise it to press harder.
CFG.AL_GROUND_VS   = 0.5
-- CUT THE LIFT ENGINE DEAD once the FMS calls the landing (alState "landed" -- a pad touchdown
-- or the end of a runway rollout). Both thruster lines go to 0, not to the balanced hover
-- ALT_NEUTRAL means, so the aeroplane sits on its gear under its own weight instead of being
-- held there by thrust it is also fighting. This is the reach-over-and-flip-the-ALTITUDE-switch
-- the pilot has always had to do by hand at touchdown, done automatically and at the right
-- instant. AL_GROUND_VS above still applies until the cut takes over, and the velocity loop's
-- integral is frozen through it, so the learnt hover trim is waiting intact for the next
-- takeoff -- the cut lifts the moment the landing is cancelled/released.
-- SAFETY: this trusts the FMS's touchdown call completely, so it is only as good as that call.
-- It is safe today because the pad sequence no longer declares a touchdown from ALTITUDE ALONE
-- (see LG.PAD_PROBE_M in apconf.lua: a helipad that is only a point in the air is HELD, not
-- landed on) -- a landing is called by the squat contact or by the craft measurably stopping.
-- Set false to go back to the standing down-bias and leave the engine to the pilot.
CFG.AL_GROUND_CUT  = true

-- Link freshness
CFG.AP_STALE_AFTER = 5.0

-- OUTPUT DITHERING HAS MOVED (2026-08-26). The sigma-delta that recovers fractional
-- commands whole-step redstone would otherwise throw away now lives in the RELAYS, one
-- accumulator per redstone line, and each relay carries its own DITHER flag at the top of
-- its program. uac.lua broadcasts floats in [-1, 1] and quantises nothing.
--
-- CFG.DITHER IS DELETED RATHER THAN LEFT SET, deliberately: a knob still sitting here that
-- the program no longer reads is exactly the "an edit here silently does nothing" failure
-- this file's header refuses to allow. To turn dithering off on this airframe, set
-- DITHER = false at the top of the relay concerned (#3/#9/#10 and their equivalents).

-- LOOP RATE ------------------------------------------------------------------
-- Added 2026-08-28 with the loop-rate fork. Aer Togekiss and Scout run an optimised uac.lua;
-- the other three airframes run the file in ../../_goldmaster/ and do not have these knobs.
--
-- WHAT THE FORK IS ANSWERING. The control loop asked for 20 Hz and measured 4.4 Hz. Scout's
-- flight recorder says why, and the shape of the answer is the useful part: across 660 ticks
-- every single measured period was an exact multiple of one game tick (0.10 s 0.2%, 0.15 s
-- 6.7%, 0.20 s 41.7%, 0.25 s 41.4%, 0.30 s 10.2%) and nothing in between. Arithmetic running
-- long gives a CONTINUOUS spread; a spread with no fractional part is a loop that is BLOCKING
-- a whole tick at a time. What blocks is the calls the server has to serve on its main
-- thread, and the loop was making ten a tick. That is the thing being cut -- not the control
-- law, which is byte-for-byte the same law it was.

-- The period the control loop TARGETS. It sleeps only if the tick finished inside this, and
-- yields through the event queue if it did not -- so an overrunning tick no longer buys
-- itself another whole 50 ms of sleep on top of the overrun. 0.05 = the 20 Hz every gain in
-- this file is derived against; raising it deliberately slows the loop and nothing here
-- wants that.
CFG.LOOP_PERIOD = 0.05

-- How often to re-read the inertia tensor and the mass. These are properties of the ASSEMBLY,
-- not of its motion -- they change when a block is added or broken or cargo moves, not at
-- 20 Hz -- so reading them every tick spent two of the loop's ten main-thread round trips on
-- two numbers that were almost always last tick's.
-- WHAT IT COSTS, so nobody has to re-derive it: a mass change is noticed up to this late,
-- which delays the CFG.TRIM_BOOST_TIME re-learn by the same amount. TRIM_BOOST_TIME is in
-- seconds and the boost works over the seconds after a mass step, so 1 s of notice is
-- immaterial. Nothing else on this aircraft reads either value.
-- Lower it if you fly with rapidly changing cargo and see the hover trim lag a load change.
CFG.SLOW_POLL_PERIOD = 1.0

-- ============================================================================
-- FLIGHT DATA RECORDER
-- ============================================================================
-- THIS AIRFRAME HAD NO RECORDER BLOCK AT ALL until 2026-08-29, which is why every vertical
-- fault on it so far has been argued from a 20 Hz screen and a memory of what the screen
-- said. uac.lua's LOG is optional by design -- absent LOG_ENABLE means "no recorder" and the
-- boot canary only demands the rest of the block once it is set -- so #1 simply never had
-- one. It does now, off by default, and the whole block is here so switching it on is one
-- edit and not a copy from Scout.
--
-- HOW TO USE IT. Set LOG_ENABLE = true, REBOOT #1 (a CC computer keeps running the code it
-- booted with -- this is worth saying out loud, because "I changed the file" and "the
-- aeroplane is running the change" are not the same statement), fly the fault, watch for the
-- green LOG DONE banner on the top row of the artificial horizon, then read /uaclog.csv off
-- the world save. Set it back to false afterwards: it costs a string.format of 40 values per
-- tick and it overwrites the previous record on every boot.
--
-- FOR THE SCRAMBLE, the two columns that matter are the last two, and they are last because
-- they were appended rather than inserted (every earlier record still parses):
--   scram   the flag as THE CONTROLLER saw it. The MANCON's panel showing SCRM only proves
--           the stick sent it. If this column is 0 while the panel says SCRM, the aeroplane
--           is deaf and the fault is between #6 and here. If it is 1, the flag arrived and
--           nothing about the input path is worth looking at again.
--   velInt  the hover trim the loop has learnt, in alt steps. The detent commands
--           CFG.ALT_SWING (8) outright, so what it is WORTH over the hover is 8 - velInt.
--           At velInt 6-7 the up line is already at 13-14 of 15 just to stay level and the
--           scramble is worth one or two steps -- which would read as "it does nothing" no
--           matter what the code did, and would mean the answer is lift, not software.
-- Read them against altSteps (what was commanded), velY (what the aeroplane did) and tiltUp
-- (body-up . world-up, so a row can be told from one taken on its side).
CFG.LOG_ENABLE  = false
CFG.LOG_FILE    = "/uaclog.csv"
-- Seconds after boot before the file is opened. The first moments are peripherals attaching,
-- the first nav/altitude packets landing and the trim starting from zero -- a transient nobody
-- asked about, and it would be the loudest thing in the record.
CFG.LOG_DELAY   = 4.0
-- How long to record, then STOP and close the file. Bounded on purpose: a computer has a fixed
-- disk quota, and an unbounded log eventually fails a write mid-flight. 60 s at 20 Hz is 1200
-- rows -- and this loop does not reach 20 Hz (see CFG.LOOP_PERIOD), so expect nearer 300.
CFG.LOG_SECONDS = 60.0
-- Record every Nth tick. 1 = every tick. Raise it for a longer window at coarser resolution.
CFG.LOG_EVERY   = 1

-- Visualiser
CFG.AH_TOP    = 1     -- artificial-horizon top row
CFG.AH_HEIGHT = 9     -- rows the horizon occupies
CFG.PITCH_PIXELS_PER_RAD = 8   -- vertical sensitivity of the horizon to pitch
-- How often the instrument panel redraws. THIS AIRCRAFT HAD NO SUCH KNOB, so it was
-- redrawing at the fallback 0.05 -- the full artificial horizon plus ten text lines, 20 times
-- a second, on the same computer that was failing to run its control loop at 20 Hz. Scout has
-- been at 0.2 since it got the knob. The horizon itself is far cheaper than it was (drawAH
-- draws runs now, and skips rows that did not change), but a readout no eye can follow faster
-- than ~5 Hz has no business competing with the attitude loop for the tick budget either way.
CFG.DISPLAY_PERIOD = 0.2

return CFG
