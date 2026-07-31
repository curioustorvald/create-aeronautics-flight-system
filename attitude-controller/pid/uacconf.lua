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
CFG.MANCON_PROTOCOL   = 'aertogekiss_mancon'     -- in : manual wanted climb rate (m/s) from the VSC, #6
CFG.MANNAV_PROTOCOL   = 'aertogekiss_mancon_nav' -- in : manual horizontal+yaw stick (m/s, rad/s) from the MIC, #14
CFG.STAB_PROTOCOL     = 'aertogekiss_stab'       -- out: pitch/roll thrusters
CFG.NAVCTRL_PROTOCOL  = 'aertogekiss_navcontrol' -- out: yaw/thrust/strafe
CFG.ALT_PROTOCOL      = 'aertogekiss_alt'        -- out: up/down thruster

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
CFG.RATE_KP     = 5.0    -- inner: desired angular accel per rad/s of rate error (pitch/roll)
CFG.RATE_KP_YAW = 5.0    -- inner gain for YAW alone -- raise for a crisper yaw WITHOUT
                           -- over-gaining pitch/roll (they keep CFG.RATE_KP)
CFG.RATE_KD     = 0.0    -- inner: derivative on the rate error (noisy; start 0)
-- INERTIA outputs a physical torque (I*alpha + gyro) in N.m. This craft's inertia
-- is huge (the on-screen `raw` runs into the 1e5-1e6 range), so steps-per-N.m is
-- tiny. Rule: ROT_GAIN ~= (steps wanted at a firm correction) / raw seen at that
-- correction, e.g. 7 / 1.3e6 ~= 5e-6. Pitch/roll (stab thrusters) and yaw (nav
-- thrusters) have different authority, so tune the three separately.
CFG.ROT_GAIN_PITCH = 5e-6
CFG.ROT_GAIN_ROLL  = 5e-6
CFG.ROT_GAIN_YAW   = 1.7e-6   -- was 5e-6; /3 after ~3x more yaw thrusters (each step now ~3x
                              -- the torque, so the loop was 3x over-gained -> left/right oscillation).
                              -- If it STILL hunts you added more than 3x -> keep dividing; if it went
                              -- sluggish, raise back toward 5e-6. This is the yaw plant-gain calibration.

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
CFG.MAX_RATE_PITCH,  CFG.MAX_RATE_YAW,  CFG.MAX_RATE_ROLL  = 4.0, 0.5,  4.0   -- rad/s
CFG.MAX_DECEL_PITCH, CFG.MAX_DECEL_YAW, CFG.MAX_DECEL_ROLL = 8.0, 0.05, 8.0   -- rad/s^2

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
CFG.SWAP_PITCH_ROLL_CHANNELS = true  -- true if pitch cmd should drive left/right

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
CFG.PITCH_ALT_SIGN  = 0 -- -1
CFG.PITCH_PER_MPS   = 0.0--0.04   -- rad of commanded pitch per m/s of desired vertical rate
CFG.PITCH_ALT_MAX   = 0.0--0.3    -- rad (~23 deg) cap on commanded pitch
CFG.PITCH_ALT_RATE  = 0.0--0.2    -- rad/s slew on the pitch command

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


-- ============================================================================
-- TRANSLATION CONFIG -- ported from navcom.lua (yaw removed; it is attitude now)
-- ============================================================================
CFG.CRUISE_SPEED       = 280.0
CFG.APPROACH_KP        = 0.20
CFG.APPROACH_SPEED     = 30.0
CFG.TERMINAL_MAX_SPEED = 10.0
-- NOTE: the manoeuvre slowdowns (nose-alignment gate, cross-track bleed) now live in the FMS
-- (#8 dynamicSpeedTarget) so that ONE place owns the commanded speed and its HSI CMD readout is
-- truthful. Do not re-add them here -- two sources would double-cut the throttle.
CFG.TRANSLATE_MIN_CMD  = 5.0
CFG.TRANSLATE_DEADZONE = 1.0
CFG.TRANSLATE_MIN_RATE = 0.35
CFG.SPD_KP = 1.5    -- ENROUTE speedPID gain only (see TRANS_KP below for the undamped laws).
                    -- speedPID has its own KD (below), which damps the 2x-stronger fore/aft
                    -- thrust fine on its own -- restored to the pre-thruster-upgrade value
                    -- after cutting it project-wide caused ENROUTE cruise to undershoot its
                    -- setpoint (over-damped: SPD_KI alone was too slow to trim the rest of
                    -- the way). If ENROUTE cruise starts hunting again, this is the one to
                    -- lower, not TRANS_KP.
CFG.SPD_KI = 0.40
CFG.SPD_KD = 1.0001
CFG.SPD_FF = 0.0
CFG.CRUISE_TRIM_INIT = 0
CFG.STRAFE_KP = 1.2
CFG.STRAFE_KI = 0.0
CFG.STRAFE_KD = 2.0
-- TRANS_KP: gain for the UNDAMPED direct-velocity-error laws only -- align-to-point
-- (TERMINAL/ARRIVED/LANDING/APPROACH, and en-route direct-to with no leg) and the
-- manual MIC stick law. Both compute thrust as a bare Kp*error with no I or D term,
-- so unlike speedPID above they have nothing to damp a too-high gain -- this is the
-- number that was cut for the 2x fore/aft thruster upgrade (was 1.5, same story as
-- ROT_GAIN_YAW). Tune independently of SPD_KP: if align-to-point/manual-stick still
-- hunts, lower this; if ENROUTE cruise undershoots its setpoint, that is SPD_KP, not
-- this one.
CFG.TRANS_KP = 0.75
CFG.THRUST_SIGN = 1
CFG.STRAFE_SIGN = 1
CFG.XTK_SIGN    = 1
CFG.WP_EPSILON  = 0.5

-- ============================================================================
-- ALTITUDE CONFIG -- ported from control.lua (the vertical cascade + autoland)
-- ============================================================================
-- The vertical channel now drives the up- and down-firing thrusters directly on
-- two separate redstone lines (see sendAlt) -- the old single 0..14 line + external
-- splitter box is gone. ALT_NEUTRAL is the balanced-hover level BOTH lines sit at
-- when the cascade wants zero net effort; +steps drives up, -steps drives down.
CFG.ALT_NEUTRAL = 7
CFG.ALT_MIN, CFG.ALT_MAX = 0, 15
-- math.max (not min): with neutral 7 in a 0..15 range the up side has 8 steps of
-- headroom, so full effort reaches level 15 and the down line reaches 0.
CFG.ALT_SWING   = math.max(CFG.ALT_NEUTRAL - CFG.ALT_MIN, CFG.ALT_MAX - CFG.ALT_NEUTRAL)
CFG.CLIMB_AT_FULL = 5.0
CFG.STEPS_PER_MPS = CFG.ALT_SWING / CFG.CLIMB_AT_FULL
CFG.ALT_KP      = 0.3
CFG.CLIMB_MAX   = 0.75 * CFG.CLIMB_AT_FULL
CFG.DESCEND_MAX = 0.75 * CFG.CLIMB_AT_FULL
CFG.VEL_KP = 1.0
CFG.VEL_KI = 0.5
CFG.VEL_KD = 0.0
CFG.ALT_TRIM_INIT = 0
CFG.MASS_STEP_THRESHOLD = 50.0
CFG.TRIM_BOOST_GAIN     = 4.0
CFG.TRIM_BOOST_TIME     = 3.0
CFG.AP_ALT_MIN, CFG.AP_ALT_MAX = 5, 999
CFG.AL_DESC_MAX    = CFG.DESCEND_MAX
CFG.AL_CLIMB_MAX   = CFG.CLIMB_MAX
CFG.AL_STALE_AFTER = 3.0

-- Link freshness
CFG.AP_STALE_AFTER = 5.0

-- Output dithering (sigma-delta): recover fractional commands that whole-step
-- redstone would otherwise throw away, since the airframe is far slower than 20 Hz.
CFG.DITHER = true

-- Visualiser
CFG.AH_TOP    = 1     -- artificial-horizon top row
CFG.AH_HEIGHT = 9     -- rows the horizon occupies
CFG.PITCH_PIXELS_PER_RAD = 8   -- vertical sensitivity of the horizon to pitch

return CFG
