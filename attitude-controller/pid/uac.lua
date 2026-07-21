-- AER TOGEKISS -- UNIFIED ATTITUDE CONTROLLER (UAC), Quaternion Edition
--
-- Supersedes the old #1 gimbal stabiliser (control.lua) AND #4 navcom.lua. One
-- loop now owns EVERY thruster: pitch/roll (stab), yaw+fore/aft+strafe (navcontrol),
-- and up/down (alt). Attitude is derived ONLY from the orientation quaternion --
-- there is NO gimbal peripheral anymore. Owning all thrusters lets the attitude
-- loop treat the disturbances the off-CoM translation thrusters inject as exactly
-- that -- disturbances it rejects -- instead of two computers fighting each other.
--
-- WHY QUATERNION (vs the old Euler/gimbal stabiliser):
--   * no gimbal hardware, no gimbal lock, no +-pi wrap seams;
--   * a single rotation-vector error that recovers a fully INVERTED craft along the
--     shortest path (the old gimbal could not);
--   * true angular-velocity and inertia-tensor data (20 Hz) feed a proper
--     rate loop + gyroscopic decoupling, which a P-on-angle gimbal law cannot.
--
-- ============================================================================
-- CALIBRATION IS EMPIRICAL. Do it in this order, flipping the *_SIGN / channel
-- constants below -- do NOT trust axis labels (the draft's stab comments and the
-- old control.lua disagree on which of pitch/roll is front/back vs left/right).
-- Run pid/probe.lua FIRST to confirm the quaternion API, the inertia-tensor shape,
-- and the frame of getAngularVelocity(). A control loop that "blows up rather than
-- settling" is a SIGN/FRAME error, not a gain -- suspect that first.
--   1. Leveling  : tilt the craft by hand; it must right itself. If a nose-up
--                  disturbance is answered nose-UP (diverges), flip CFG.PITCH_SIGN;
--                  same for CFG.ROLL_SIGN. If pitch corrections move the craft in ROLL
--                  (wrong thrusters), set CFG.SWAP_PITCH_ROLL_CHANNELS = true.
--   2. Invert    : flip it upside down; it must roll/pitch back to level.
--   3. Yaw       : ENROUTE, command a target off to one side; the nose must swing
--                  TOWARD it and settle ON it. If it runs away, flip CFG.YAW_SIGN.
--   4. Translate : (ported navcom) flip CFG.STRAFE_SIGN/CFG.THRUST_SIGN per its own notes.
--   5. Altitude  : (ported control) learns its own hover trim; just confirm it
--                  climbs to a tapped ALT-AP altitude and holds.
-- ============================================================================

-- PROTOCOLS (identical to control.lua + navcom.lua, so nothing else changes) ---
local CFG = {}   -- tuning config, tabled to stay under Cobalt's 200-locals-per-function limit
CFG.APNAV_PROTOCOL    = 'aertogekiss_navap'      -- in : FMS target + phase
CFG.ALTAP_PROTOCOL    = 'aertogekiss_altap'      -- in : ALT-AP panel setpoint
CFG.AUTOLAND_PROTOCOL = 'aertogekiss_autoland'   -- in : FMS vertical channel
CFG.STAB_PROTOCOL     = 'aertogekiss_stab'       -- out: pitch/roll thrusters
CFG.NAVCTRL_PROTOCOL  = 'aertogekiss_navcontrol' -- out: yaw/thrust/strafe
CFG.ALT_PROTOCOL      = 'aertogekiss_alt'        -- out: up/down thruster

-- MASTER ENABLES -- bring the craft up one channel at a time (it is still ONE
-- file; these just zero a channel's output so you can isolate a problem).
CFG.ENABLE_ATTITUDE    = true
CFG.ENABLE_TRANSLATION = true
CFG.ENABLE_ALTITUDE    = true

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
CFG.ROT_GAIN_YAW   = 5e-6

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
CFG.VERT_XFER_FWD   = 1.0   -- fore/aft steps per unit vertical-effort * (world-up . body-fwd)
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

-- Channel limits (redstone steps)
CFG.STAB_MAX   = 14
CFG.YAW_MAX    = 14
CFG.THRUST_MAX = 14
CFG.STRAFE_MAX = 14

-- ============================================================================
-- TRANSLATION CONFIG -- ported from navcom.lua (yaw removed; it is attitude now)
-- ============================================================================
CFG.CRUISE_SPEED       = 70.0
CFG.APPROACH_KP        = 0.20
CFG.APPROACH_SPEED     = 30.0
CFG.TERMINAL_MAX_SPEED = 10.0
CFG.TRANSLATE_MIN_CMD  = 5.0
CFG.TRANSLATE_DEADZONE = 1.0
CFG.TRANSLATE_MIN_RATE = 0.35
CFG.SPD_KP = 1.5
CFG.SPD_KI = 0.40
CFG.SPD_KD = 0.0
CFG.SPD_FF = 0.0
CFG.CRUISE_TRIM_INIT = 0
CFG.STRAFE_KP = 1.2
CFG.STRAFE_KI = 0.0
CFG.STRAFE_KD = 2.0
CFG.THRUST_SIGN = 1
CFG.STRAFE_SIGN = 1
CFG.XTK_SIGN    = 1
CFG.WP_EPSILON  = 0.5

-- ============================================================================
-- ALTITUDE CONFIG -- ported from control.lua (the vertical cascade + autoland)
-- ============================================================================
CFG.ALT_NEUTRAL = 7
CFG.ALT_MIN, CFG.ALT_MAX = 0, 14
CFG.ALT_SWING   = math.min(CFG.ALT_NEUTRAL - CFG.ALT_MIN, CFG.ALT_MAX - CFG.ALT_NEUTRAL)
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

-- SETUP -----------------------------------------------------------------------
if not sublevel then error("No sublevel API - is the Sable mod present?") end

local pid = require("pid")

local modem = peripheral.find("modem")
if not modem then error("No modem attached") end
rednet.open(peripheral.getName(modem))

term.setPaletteColour(colours.green,  0x116611)
term.setPaletteColour(colours.brown,  0x553322)
term.setPaletteColour(colours.orange, 0xffbb00)
term.setPaletteColour(colours.cyan,   0x117766)
term.setPaletteColour(colours.blue,   0x114488)
term.setPaletteColour(colours.red,    0x995500)

-- HELPERS ---------------------------------------------------------------------
local function clamp(v, lo, hi) return math.max(lo, math.min(hi, v)) end
local function round(x) return math.floor(x + 0.5) end
local function wrapPi(a) return (a + math.pi) % (2 * math.pi) - math.pi end

local atan2 = math.atan2 or function(y, x)
    if x > 0 then return math.atan(y / x)
    elseif x < 0 then return math.atan(y / x) + (y >= 0 and math.pi or -math.pi)
    else return (y > 0 and math.pi / 2) or (y < 0 and -math.pi / 2) or 0 end
end

-- toEuler tolerates either return shape: (pitch,yaw,roll) numbers or one vector.
local function euler(orientation)
    local a, b, c = orientation:toEuler()
    if type(a) == "number" then return a, b, c end
    return a.x, a.y, a.z
end
local function getYaw(orientation)
    local _, y = euler(orientation)
    return y
end

-- Sigma-delta quantiser; one accumulator per channel, they must not share.
local function makeDither()
    local acc = 0
    return function(value, lo, hi)
        local want = value + acc
        local q = clamp(math.floor(want + 0.5), lo, hi)
        acc = clamp(want - q, -0.5, 0.5)
        return q
    end
end
local ditherPitch  = makeDither()
local ditherRoll   = makeDither()
local ditherYaw    = makeDither()
local ditherThrust = makeDither()
local ditherStrafe = makeDither()
local ditherAlt    = makeDither()

-- Resolve, once, how to BUILD a wings-level quaternion facing a given raw yaw
-- (a rotation about world-up Y only => zero pitch/roll). probe.lua tells you which
-- of these your build actually has; we auto-detect so this file runs regardless.
local makeYawQuat, quatCtorName
do
    local function testable(fn)
        local ok, q = pcall(fn)
        return ok and type(q) == "table" and q.toEuler ~= nil, q
    end
    if type(quaternion) == "table" then
        if testable(function() return quaternion.fromAxisAngle(vector.new(0, 1, 0), 0) end) then
            makeYawQuat = function(y) return quaternion.fromAxisAngle(vector.new(0, 1, 0), y) end
            quatCtorName = "fromAxisAngle"
        elseif testable(function() return quaternion.fromEuler(0, 0, 0) end) then
            makeYawQuat = function(y) return quaternion.fromEuler(0, y, 0) end
            quatCtorName = "fromEuler"
        elseif testable(function() return quaternion.new(vector.new(0, 0, 0), 1) end) then
            -- quaternion.new(vec, w): imaginary vector first, real scalar second.
            makeYawQuat = function(y) local h = y * 0.5; return quaternion.new(vector.new(0, math.sin(h), 0), math.cos(h)) end
            quatCtorName = "new(vec,w)"
        end
    end
    if not makeYawQuat then
        quatCtorName = "NONE - attitude disabled"
        CFG.ENABLE_ATTITUDE = false
    end
end

-- mat3x3 * vec3, tolerating the tensor's unknown shape (nested [i][j], flat 9,
-- or {m11=..}). Only used in INERTIA mode. Returns three numbers.
local function matVec(M, x, y, z)
    if type(M) ~= "table" then return x, y, z end
    local m = {}
    if type(M[1]) == "table" then                    -- nested M[i][j]
        m = { M[1][1], M[1][2], M[1][3], M[2][1], M[2][2], M[2][3], M[3][1], M[3][2], M[3][3] }
    elseif type(M[1]) == "number" then               -- flat row-major 9
        m = { M[1], M[2], M[3], M[4], M[5], M[6], M[7], M[8], M[9] }
    else                                             -- named fields m11.. or c0..
        m = { M.m11 or M[0] or 1, M.m12 or 0, M.m13 or 0,
              M.m21 or 0, M.m22 or 1, M.m23 or 0,
              M.m31 or 0, M.m32 or 0, M.m33 or 1 }
    end
    return m[1]*x + m[2]*y + m[3]*z,
           m[4]*x + m[5]*y + m[6]*z,
           m[7]*x + m[8]*y + m[9]*z
end

-- Rotate vector (x,y,z) by the UNIT quaternion q, PRESERVING its magnitude.
-- Done by hand (v + 2a(u x v) + 2(u x (u x v)), u = q.v) rather than the library's
-- q*vector operator: that operator normalises its intermediate quaternion product,
-- so it keeps only the DIRECTION and throws the length away -- which silently
-- collapsed million-N.m torque commands down to ~1 step.
local function rotateVec(q, x, y, z)
    local ux, uy, uz, a = q.v.x, q.v.y, q.v.z, q.a
    local tx = 2 * (uy*z - uz*y)
    local ty = 2 * (uz*x - ux*z)
    local tz = 2 * (ux*y - uy*x)
    return x + a*tx + (uy*tz - uz*ty),
           y + a*ty + (uz*tx - ux*tz),
           z + a*tz + (ux*ty - uy*tx)
end

-- One attitude-integral axis update. Accumulates ONLY when the axis is quasi-static
-- (small error AND low rate): a large error is a manoeuvre/recovery (bleed toward 0),
-- and any real rotation means the P/D loop is doing dynamic work the integral must
-- NOT join -- integrating through motion lags the error ~90 deg and pumps a slow,
-- growing oscillation. So the integral learns the steady trim only, near rest.
local function integStep(cur, e, rate, dt)
    if math.abs(e) > CFG.ATT_I_BLEED_ERR then return cur * 0.98 end
    if math.abs(rate) > CFG.ATT_I_RATE_GATE then return cur end
    return clamp(cur + e * dt, -CFG.ATT_I_LIMIT, CFG.ATT_I_LIMIT)
end

-- Motion-profile desired rate: linear kp*err near the target, but never faster than
-- can still be braked to zero by the time err reaches zero (sqrt(2*decel*|err|)),
-- and never above maxRate. Lets a high-inertia axis counter-steer before overshoot.
local function profileRate(err, kp, maxRate, maxDecel)
    local cap = math.min(maxRate, math.sqrt(2 * maxDecel * math.abs(err)))
    return clamp(kp * err, -cap, cap)
end

-- STATE (set by the receive loops; read by the control loop) ------------------
local navTarget = { nil, nil, nil, '' }   -- {X, Z, type, name}
local navPrev   = nil                     -- {X, Z} previous waypoint, or nil
local navPhase  = "ENROUTE"
local navHoldHeading = nil
local navCruiseCap   = CFG.CRUISE_SPEED
local apLastMsg = nil

local AP_ALTI    = nil
local altLastMsg = nil
local alState     = "normal"   -- normal | hold | descend | landed
local alDescendVS = 0
local alLastMsg   = nil
local latestPosY  = nil
local TARGET_ALT  = nil        -- set to boot altitude on first tick (hold-in-place)

-- PID instances (translation + altitude, exactly as their source files) --------
local speedPID = pid.new(0, CFG.SPD_KP, CFG.SPD_KI, CFG.SPD_KD)
speedPID:clampOutput(-CFG.THRUST_MAX, CFG.THRUST_MAX)
if CFG.SPD_KI > 0 then local l = CFG.THRUST_MAX / CFG.SPD_KI; speedPID:limitIntegral(-l, l); speedPID.integral = CFG.CRUISE_TRIM_INIT / CFG.SPD_KI end

local strafePID = pid.new(0, CFG.STRAFE_KP, CFG.STRAFE_KI, 0)
strafePID:clampOutput(-CFG.STRAFE_MAX, CFG.STRAFE_MAX)
if CFG.STRAFE_KI > 0 then local l = CFG.STRAFE_MAX / CFG.STRAFE_KI; strafePID:limitIntegral(-l, l) end

local altPID = pid.new(0, CFG.ALT_KP, 0, 0)
altPID:clampOutput(-CFG.DESCEND_MAX, CFG.CLIMB_MAX)
local VEL_I_LIMIT = (CFG.VEL_KI > 0) and (CFG.ALT_SWING / CFG.VEL_KI) or 1
local velPID = pid.new(0, CFG.VEL_KP, CFG.VEL_KI, CFG.VEL_KD)
velPID:limitIntegral(-VEL_I_LIMIT, VEL_I_LIMIT)
velPID.integral = (CFG.VEL_KI > 0) and (CFG.ALT_TRIM_INIT / CFG.VEL_KI) or 0

-- For the INERTIA-mode inner PD's derivative term.
local prevRateErrX, prevRateErrY, prevRateErrZ = 0, 0, 0
-- Attitude integral (sensor frame), per axis. Trims the standing tilt (CFG.ATT_INTEGRAL).
local attIntegX, attIntegY, attIntegZ = 0, 0, 0

-- Published for the display loop.
local ui = {}

-- Alter TARGET_ALT (the only door to the altitude setpoint).
local function setTarget(alt)
    if type(alt) ~= "number" or alt ~= alt then return false end
    alt = clamp(alt, CFG.AP_ALT_MIN, CFG.AP_ALT_MAX)
    TARGET_ALT = alt
    altPID.sp  = alt
    return true
end

-- OUTPUT ----------------------------------------------------------------------
local outStab   = { front = 0, back = 0, left = 0, right = 0 }
local outNav    = { thrust = 0, yaw = 0, strafe = 0 }
local outAlt    = CFG.ALT_NEUTRAL

-- pitch/roll are signed steps [-14,14]. front/back carry pitch, left/right roll
-- (control.lua's mapping; flip via CFG.SWAP_PITCH_ROLL_CHANNELS / *_SIGN if wrong).
local function sendStab(pitch, roll)
    if CFG.DITHER then
        pitch = ditherPitch(pitch, -CFG.STAB_MAX, CFG.STAB_MAX)
        roll  = ditherRoll(roll,  -CFG.STAB_MAX, CFG.STAB_MAX)
    else
        pitch = clamp(round(pitch), -CFG.STAB_MAX, CFG.STAB_MAX)
        roll  = clamp(round(roll),  -CFG.STAB_MAX, CFG.STAB_MAX)
    end
    local pCh, rCh = pitch, roll
    if CFG.SWAP_PITCH_ROLL_CHANNELS then pCh, rCh = roll, pitch end
    local front, back, left, right = 0, 0, 0, 0
    if pCh >= 0 then front = pCh else back = -pCh end
    if rCh >= 0 then right = rCh else left = -rCh end
    rednet.broadcast({ left = left, right = right, front = front, back = back }, CFG.STAB_PROTOCOL)
    outStab = { front = front, back = back, left = left, right = right }
end

-- thrust +fwd/-back, yaw +right/-left, strafe +right/-left; all [-14,14].
local function sendNav(thrust, yaw, strafe)
    if CFG.DITHER then
        thrust = ditherThrust(thrust, -CFG.THRUST_MAX, CFG.THRUST_MAX)
        yaw    = ditherYaw(yaw,       -CFG.YAW_MAX,    CFG.YAW_MAX)
        strafe = ditherStrafe(strafe, -CFG.STRAFE_MAX, CFG.STRAFE_MAX)
    else
        thrust = clamp(round(thrust), -CFG.THRUST_MAX, CFG.THRUST_MAX)
        yaw    = clamp(round(yaw),    -CFG.YAW_MAX,    CFG.YAW_MAX)
        strafe = clamp(round(strafe), -CFG.STRAFE_MAX, CFG.STRAFE_MAX)
    end
    local fwd, back = 0, 0;      if thrust >= 0 then fwd = thrust else back = -thrust end
    local yawL, yawR = 0, 0;     if yaw >= 0 then yawR = yaw else yawL = -yaw end
    local strL, strR = 0, 0;     if strafe >= 0 then strR = strafe else strL = -strafe end
    rednet.broadcast({
        thrust_forward = fwd, thrust_backward = back,
        yaw_left = yawL, yaw_right = yawR,
        strafe_left = strL, strafe_right = strR,
    }, CFG.NAVCTRL_PROTOCOL)
    outNav = { thrust = thrust, yaw = yaw, strafe = strafe }
end

local function sendAlt(steps)
    local want = CFG.ALT_NEUTRAL + steps
    local level = CFG.DITHER and ditherAlt(want, CFG.ALT_MIN, CFG.ALT_MAX) or clamp(round(want), CFG.ALT_MIN, CFG.ALT_MAX)
    rednet.broadcast({ alt = level }, CFG.ALT_PROTOCOL)
    outAlt = level
    return level
end

-- ATTITUDE --------------------------------------------------------------------
-- Returns pitchSteps, rollSteps, yawSteps (signed). Everything is computed in the
-- BODY frame -- the frame the per-axis thrusters live in: the attitude error is
-- q_cur^-1 * q_des directly, the angular velocity is rotated in from the sensor
-- frame if needed, and the inertia tensor is applied via inertiaMul (body vector ->
-- body vector, whatever frame the tensor itself is in). There is no final command
-- rotation, so a pure pitch demand stays a pure pitch command.
local function attitude(q_cur, w, heading, desiredHeading, I, dt)
    local q_inv = q_cur:inverse()
    -- Desired level attitude facing desiredHeading. Invert navcom's heading chain
    -- so the yaw we build lives in the raw-quaternion frame q_cur reads back in.
    local rawYawDes = (desiredHeading - CFG.HEADING_OFFSET) / CFG.HEADING_SIGN
    local q_des = makeYawQuat(rawYawDes)

    -- BODY-frame attitude error as a rotation vector (x=pitch, y=yaw, z=roll).
    local ex, ey, ez
    if CFG.YAW_TILT_DECOMP then
        -- DECOMPOSED (Bug 1): tilt error + yaw error, computed separately so a ~180 deg
        -- heading change stays PURE yaw and never hits the combined-quaternion 180 deg
        -- singularity. Tilt = the rotation bringing body-up back to world-up (from the up
        -- vectors -> smooth except at a full inversion); yaw = the wrapped heading error.
        local ubx, uby, ubz = rotateVec(q_cur, 0, 1, 0)   -- body-up expressed in world
        -- level-ing rotation: axis(world) = bodyUp x worldUp = (-ubz, 0, ubx), angle in [0,pi]
        local hs = math.sqrt(ubx*ubx + ubz*ubz)
        local twx, twy, twz = 0, 0, 0
        if hs > 1e-9 then
            local k = atan2(hs, uby) / hs
            twx, twy, twz = -ubz*k, 0, ubx*k              -- world-frame tilt rotation vector
        end
        local tbx, _, tbz = rotateVec(q_inv, twx, twy, twz)  -- -> body frame (pitch/roll)
        ex = tbx
        ey = wrapPi(desiredHeading - heading)             -- pure yaw about the up axis
        ez = tbz
    else
        -- COMBINED quaternion error (fallback): one shortest-path rotation from the
        -- components (a >= 0 = short way; atan2 finite through identity). Elegant, but a
        -- ~180 deg total rotation has an ill-defined axis -> the Bug-1 pitch kick.
        local eq = q_inv * q_des
        local ea, vx, vy, vz = eq.a, eq.v.x, eq.v.y, eq.v.z
        if ea < 0 then ea, vx, vy, vz = -ea, -vx, -vy, -vz end
        local vlen = math.sqrt(vx*vx + vy*vy + vz*vz)
        local ang  = 2 * atan2(vlen, ea)                     -- shortest angle, [0, pi]
        local es   = (vlen > 1e-9) and (ang / vlen) or 2.0
        ex, ey, ez = vx*es, vy*es, vz*es
    end

    -- Angular velocity in the BODY frame (rotate the world reading in if needed),
    -- then the per-axis rate-feedback sign (independent of the output *_SIGN).
    local wx, wy, wz
    if CFG.SENSOR_FRAME == "world" then wx, wy, wz = rotateVec(q_inv, w.x, w.y, w.z)
    else                            wx, wy, wz = w.x, w.y, w.z end
    wx, wy, wz = wx * CFG.PITCH_RATE_SIGN, wy * CFG.YAW_RATE_SIGN, wz * CFG.ROLL_RATE_SIGN

    -- Integral trim (body frame), rate-gated so it can't pump a manoeuvre (integStep).
    if CFG.ATT_INTEGRAL then
        attIntegX = integStep(attIntegX, ex, wx, dt)
        --attIntegY = integStep(attIntegY, ey, wy, dt)
        --attIntegZ = integStep(attIntegZ, ez, wz, dt)
    end
    local biasX = CFG.ATT_INTEGRAL and attIntegX or 0
    local biasY = CFG.ATT_INTEGRAL and attIntegY or 0
    local biasZ = CFG.ATT_INTEGRAL and attIntegZ or 0

    local px, py, pz   -- body-frame command (pitch, yaw, roll), pre-sign/clamp
    if CFG.ATT_MODE == "INERTIA" then
        -- I * (body vector) -> body vector, from whatever frame the tensor is in.
        local function inertiaMul(a, b, c)
            if CFG.SENSOR_FRAME == "world" then
                local wa, wb, wc = rotateVec(q_cur, a, b, c)   -- body -> world
                local ia, ib, ic = matVec(I, wa, wb, wc)       -- world-frame tensor
                return rotateVec(q_inv, ia, ib, ic)            -- world -> body
            end
            return matVec(I, a, b, c)
        end

        -- Desired body angular acceleration. Outer loop: a decel-limited motion
        -- profile turns error into a desired RATE that a high-inertia axis can still
        -- brake from (so yaw doesn't overshoot/spin); inner P(D) tracks that rate,
        -- plus the integral trim as an accel bias.
        local wdx = profileRate(ex, CFG.ATT_KP, CFG.MAX_RATE_PITCH, CFG.MAX_DECEL_PITCH)
        local wdy = profileRate(ey, CFG.ATT_KP, CFG.MAX_RATE_YAW,   CFG.MAX_DECEL_YAW)
        local wdz = profileRate(ez, CFG.ATT_KP, CFG.MAX_RATE_ROLL,  CFG.MAX_DECEL_ROLL)
        local rex, rey, rez = wdx - wx, wdy - wy, wdz - wz
        local ax = CFG.RATE_KP*rex     + CFG.ATT_KI*biasX
        local ay = CFG.RATE_KP_YAW*rey + CFG.ATT_KI*biasY
        local az = CFG.RATE_KP*rez     + CFG.ATT_KI*biasZ
        if CFG.RATE_KD ~= 0 then
            ax = ax + CFG.RATE_KD*(rex - prevRateErrX)/dt
            ay = ay + CFG.RATE_KD*(rey - prevRateErrY)/dt
            az = az + CFG.RATE_KD*(rez - prevRateErrZ)/dt
        end
        prevRateErrX, prevRateErrY, prevRateErrZ = rex, rey, rez

        -- Gyro-comp fade: full below GYRO_FADE_LO of |yaw rate|, zero above GYRO_FADE_HI,
        -- so the lagged feedforward stands down before it can pump the fast-yaw nutation.
        local gyroFade = CFG.GYRO_COMPENSATION
            and clamp((CFG.GYRO_FADE_HI - math.abs(wy)) / (CFG.GYRO_FADE_HI - CFG.GYRO_FADE_LO), 0, 1) or 0
        ui.gyroFade = gyroFade

        local tpx, tpy, tpz
        if CFG.INERTIA_COUPLING then
            -- Full tensor. Products of inertia => a pure pitch demand also asks for
            -- roll/yaw torque; on this asymmetric craft that is the unwanted roll.
            tpx, tpy, tpz = inertiaMul(ax, ay, az)
            if gyroFade > 0 then
                local iwx, iwy, iwz = inertiaMul(wx, wy, wz)      -- I*omega (body)
                tpx = tpx + gyroFade * (wy*iwz - wz*iwy)          -- + faded omega x (I omega)
                tpy = tpy + gyroFade * (wz*iwx - wx*iwz)
                tpz = tpz + gyroFade * (wx*iwy - wy*iwx)
            end
        else
            -- Decoupled: body-diagonal moments only, so each axis is independent
            -- (Ix/Iy/Iz are the tensor's quadratic form on each body unit axis).
            local Ix       = inertiaMul(1, 0, 0)   -- first component = I_body_xx
            local _,  Iy   = inertiaMul(0, 1, 0)   -- second        = I_body_yy
            local _, _, Iz = inertiaMul(0, 0, 1)   -- third         = I_body_zz
            ui.Ix, ui.Iy, ui.Iz = Ix, Iy, Iz       -- for display (intermediate-axis check)
            tpx = Ix*ax
            tpy = Iy*ay
            tpz = Iz*az
            if gyroFade > 0 then                        -- diagonal gyro decoupling, faded
                tpx = tpx + gyroFade * wy*wz*(Iz - Iy)
                tpy = tpy + gyroFade * wz*wx*(Ix - Iz)
                tpz = tpz + gyroFade * wx*wy*(Iy - Ix)
            end
        end
        px = CFG.ROT_GAIN_PITCH*tpx
        py = CFG.ROT_GAIN_YAW  *tpy
        pz = CFG.ROT_GAIN_ROLL *tpz
        ui.rotRaw = math.sqrt(tpx*tpx + tpy*tpy + tpz*tpz)
    else
        -- PD in the body frame, per axis (inherently decoupled), + integral step bias.
        px = CFG.PD_KP_PITCH*ex - CFG.PD_KD_PITCH*wx + CFG.PD_KI*biasX
        py = CFG.PD_KP_YAW  *ey - CFG.PD_KD_YAW  *wy + CFG.PD_KI*biasY
        pz = CFG.PD_KP_ROLL *ez - CFG.PD_KD_ROLL *wz + CFG.PD_KI*biasZ
        ui.rotRaw = math.sqrt(px*px + py*py + pz*pz)
    end

    -- Level priority: fade yaw out as pitch+roll error grows, so a yaw manoeuvre can
    -- never roll/pitch the craft over faster than the leveling loop can recover.
    local tilt = math.sqrt(ex*ex + ez*ez)   -- pitch+roll error magnitude (rad), excl. yaw
    local yawScale = clamp((CFG.TILT_YAW_MAX - tilt) / (CFG.TILT_YAW_MAX - CFG.TILT_YAW_MIN), 0, 1)

    ui.errPitch, ui.errYaw, ui.errRoll = ex, ey, ez
    ui.integP, ui.integR = attIntegX, attIntegZ
    ui.yawScale = yawScale
    -- Yaw first, then feedforward its coupling into pitch/roll (based on the ACTUAL
    -- yaw command, so it fades with yawScale exactly as the real disturbance does).
    local yawSteps   = clamp(CFG.YAW_SIGN * py, -CFG.YAW_MAX, CFG.YAW_MAX) * yawScale
    local pitchSteps = clamp(CFG.PITCH_SIGN * px + CFG.YAW_TO_PITCH_FF * yawSteps, -CFG.STAB_MAX, CFG.STAB_MAX)
    local rollSteps  = clamp(CFG.ROLL_SIGN  * pz + CFG.YAW_TO_ROLL_FF  * yawSteps, -CFG.STAB_MAX, CFG.STAB_MAX)
    return pitchSteps, rollSteps, yawSteps
end

-- CONTROL LOOP ----------------------------------------------------------------
local function controlLoop()
    local lastTime = os.clock()
    local prevVelY = 0
    local lastMass = nil
    local trimBoost = 0

    while true do
        local now = os.clock()
        local dt  = math.max(now - lastTime, 0.001)
        lastTime  = now

        local pose   = sublevel.getLogicalPose()
        local pos    = pose.position
        local q_cur  = pose.orientation
        local q_inv  = q_cur:inverse()   -- for the attitude-aware vertical allocation
        local vel    = sublevel.getLinearVelocity()
        local w      = sublevel.getAngularVelocity()
        local I      = (CFG.ATT_MODE == "INERTIA") and sublevel.getInertiaTensor() or nil
        local mass   = sublevel.getMass()

        latestPosY = pos.y
        if TARGET_ALT == nil then setTarget(pos.y) end   -- boot: hold where we are

        local heading = CFG.HEADING_SIGN * getYaw(q_cur) + CFG.HEADING_OFFSET

        -- ---- nav geometry (from the current leg) ----
        local tx, tz = navTarget[1], navTarget[2]
        local hasTarget = (type(tx) == "number" and type(tz) == "number")
        local dist, bearing, closing = 0, heading, 0
        local hasLeg, xtk, xtkRate, dtk = false, 0, 0, heading
        local dx, dz = 0, 0
        if hasTarget then
            dx, dz = tx - pos.x, tz - pos.z
            dist = math.sqrt(dx*dx + dz*dz)
            closing = (dist > 1e-6) and ((vel.x*dx + vel.z*dz) / dist) or 0
            bearing = atan2(dx, dz)
            if navPrev then
                local lx, lz = tx - navPrev[1], tz - navPrev[2]
                local legLen = math.sqrt(lx*lx + lz*lz)
                if legLen > 1e-3 then
                    hasLeg = true
                    dtk = atan2(lx, lz)
                    local ux, uz = lx / legLen, lz / legLen
                    local rx, rz = pos.x - navPrev[1], pos.z - navPrev[2]
                    xtk     = ux * rz - uz * rx
                    xtkRate = ux * vel.z - uz * vel.x
                end
            end
        end

        local translateOnly = (navPhase == "APPROACH" or navPhase == "TERMINAL"
            or navPhase == "ARRIVED" or navPhase == "LANDING")
        local headingHold = (navPhase == "LANDING" and type(navHoldHeading) == "number")

        -- ---- desired heading, per phase (the yaw "autopilot") ----
        --  ENROUTE  -> seek the bearing to the target
        --  LANDING  -> hold the commanded pad heading
        --  else / hands-off / no target -> current heading, so the attitude loop
        --                     only rate-damps yaw and the pilot keeps steering.
        local desiredHeading = heading
        if hasTarget and navPhase == "ENROUTE" then desiredHeading = bearing
        elseif headingHold then desiredHeading = navHoldHeading end

        -- ---- ATTITUDE ----
        local pitchSteps, rollSteps, yawSteps = 0, 0, 0
        if CFG.ENABLE_ATTITUDE then
            pitchSteps, rollSteps, yawSteps = attitude(q_cur, w, heading, desiredHeading, I, dt)
        end

        -- ---- HORIZONTAL TRANSLATION (ported navcom) ----
        local thrustCmd, strafeCmd = 0, 0
        if CFG.ENABLE_TRANSLATION and hasTarget then
            if translateOnly then
                local speedCap = (navPhase == "APPROACH") and CFG.APPROACH_SPEED or CFG.TERMINAL_MAX_SPEED
                local desiredSpeed = clamp(CFG.APPROACH_KP * dist, 0, speedCap)
                local ux, uz = 0, 0
                if dist > 1e-6 then ux, uz = dx / dist, dz / dist end
                local velErrX = desiredSpeed * ux - vel.x
                local velErrZ = desiredSpeed * uz - vel.z
                local sinH, cosH = math.sin(heading), math.cos(heading)
                local fwdErr   = velErrX * sinH + velErrZ * cosH
                local rightErr = velErrZ * sinH - velErrX * cosH   -- matches xtk handedness
                local fwdCmd   = CFG.SPD_KP * fwdErr
                local rightCmd = CFG.STRAFE_KP * rightErr
                local mag = math.sqrt(fwdCmd*fwdCmd + rightCmd*rightCmd)
                if dist > CFG.TRANSLATE_DEADZONE and closing < CFG.TRANSLATE_MIN_RATE
                        and mag > 1e-6 and mag < CFG.TRANSLATE_MIN_CMD then
                    local k = CFG.TRANSLATE_MIN_CMD / mag
                    fwdCmd, rightCmd = fwdCmd * k, rightCmd * k
                end
                thrustCmd = CFG.THRUST_SIGN * clamp(fwdCmd, -CFG.THRUST_MAX, CFG.THRUST_MAX)
                strafeCmd = CFG.STRAFE_SIGN * clamp(rightCmd, -CFG.STRAFE_MAX, CFG.STRAFE_MAX)
                speedPID.integral  = 0
                strafePID.integral = 0
            else
                -- ENROUTE along-track: outer P closing speed (gated by nose alignment),
                -- inner PI buys it. Cross-track strafe holds the leg.
                local cap = clamp(navCruiseCap, 0, CFG.CRUISE_SPEED)
                local headingErr = wrapPi(bearing - heading)
                local align = math.max(0, math.cos(headingErr))
                local desiredClosing = clamp(CFG.APPROACH_KP * dist, 0, cap) * align
                speedPID.sp = desiredClosing
                local ib = speedPID.integral
                local thrustRaw = CFG.SPD_FF * desiredClosing + speedPID:step(closing, dt)
                local thrustSat = clamp(thrustRaw, -CFG.THRUST_MAX, CFG.THRUST_MAX)
                if thrustSat ~= thrustRaw and (thrustRaw - thrustSat) * speedPID.integral > 0 then
                    speedPID.integral = ib
                end
                thrustCmd = CFG.THRUST_SIGN * thrustSat

                if hasLeg then
                    local xtkErr  = CFG.STRAFE_SIGN * xtk
                    local xtkErrD = CFG.STRAFE_SIGN * xtkRate
                    strafePID.sp = 0
                    local sib = strafePID.integral
                    local strafeRaw = strafePID:step(xtkErr, dt) - CFG.STRAFE_KD * xtkErrD
                    local strafeSat = clamp(strafeRaw, -CFG.STRAFE_MAX, CFG.STRAFE_MAX)
                    if strafeSat ~= strafeRaw and (strafeRaw - strafeSat) * strafePID.integral > 0 then
                        strafePID.integral = sib
                    end
                    strafeCmd = strafeSat
                else
                    -- Direct-to (no leg to cross-track-hold): this is a VTOL, so also STRAFE
                    -- toward the target's lateral offset rather than waiting on the (huge-
                    -- inertia, slow) yaw to swing the nose onto the bearing -- a yaw that
                    -- can't keep up otherwise flies a pursuit SPIRAL that never closes, with
                    -- the strafe thrusters never commanded. Body-right velocity error toward
                    -- the target, same handedness/sign as the leg cross-track and the
                    -- translate law; it fades to zero as the nose comes onto the bearing, so
                    -- an aligned cruise is unaffected (forward thrust does the work).
                    local ux2, uz2 = 0, 0
                    if dist > 1e-6 then ux2, uz2 = dx / dist, dz / dist end
                    local dSpeed = clamp(CFG.APPROACH_KP * dist, 0, cap)
                    local vErrX  = dSpeed * ux2 - vel.x
                    local vErrZ  = dSpeed * uz2 - vel.z
                    local rErr   = vErrZ * math.sin(heading) - vErrX * math.cos(heading)
                    strafeCmd = CFG.STRAFE_SIGN * clamp(CFG.STRAFE_KP * rErr, -CFG.STRAFE_MAX, CFG.STRAFE_MAX)
                    strafePID.integral = 0
                end
            end
        end

        -- ---- VERTICAL (ported control.lua cascade + autoland) ----
        local altLevel = CFG.ALT_NEUTRAL
        local desiredVelY, velY = 0, vel.y
        local vertEffort = 0   -- signed alt-thruster steps the cascade wants (+ = climb)
        if CFG.ENABLE_ALTITUDE then
            if alState == "descend" then
                desiredVelY = alDescendVS
                if alLastMsg and (now - alLastMsg) > CFG.AL_STALE_AFTER then desiredVelY = 0 end
            else
                desiredVelY = altPID:step(pos.y, dt)
            end
            if lastMass and math.abs(mass - lastMass) > CFG.MASS_STEP_THRESHOLD then trimBoost = CFG.TRIM_BOOST_TIME end
            lastMass = mass
            if trimBoost > 0 then trimBoost = trimBoost - dt end

            velPID.sp = desiredVelY
            local ib = velPID.integral
            local steps = CFG.STEPS_PER_MPS * desiredVelY + velPID:step(velY, dt)
            local stepsSat = clamp(steps, -CFG.ALT_SWING, CFG.ALT_SWING)
            if stepsSat ~= steps and (steps - stepsSat) * velPID.integral > 0 then
                velPID.integral = ib
            elseif trimBoost > 0 then
                velPID.integral = clamp(velPID.integral + (desiredVelY - velY) * dt * (CFG.TRIM_BOOST_GAIN - 1),
                    -VEL_I_LIMIT, VEL_I_LIMIT)
            end
            vertEffort = stepsSat
        end

        -- ---- ATTITUDE-AWARE VERTICAL ALLOCATION (Bug 2) ----
        -- "Hold altitude" is world-up, but the alt thruster is body-fixed. Route the
        -- vertical effort by the true orientation: the alt thruster gets the body-up share
        -- of world-up (flips sign inverted, fades toward 90 deg pitch), and fore/aft + strafe
        -- pick up the rest. No-op at level (world-up-in-body = (0,1,0) -> all to alt).
        local altSteps = vertEffort
        if CFG.ENABLE_ALTITUDE and CFG.TILT_ALT_ALLOC then
            local ux, uy, uz = rotateVec(q_inv, 0, 1, 0)   -- world-up expressed in body
            altSteps  = vertEffort * uy                     -- body-up axis  -> alt thruster
            thrustCmd = thrustCmd + vertEffort * uz * CFG.VERT_XFER_FWD   -- body-fwd -> fore/aft
            strafeCmd = strafeCmd + vertEffort * ux * CFG.VERT_XFER_STR   -- body-lat -> strafe
            ui.tiltUp = uy
        end
        altLevel = sendAlt(CFG.ENABLE_ALTITUDE and altSteps or 0)

        -- ---- OUTPUT ----
        if CFG.ENABLE_ATTITUDE then sendStab(pitchSteps, rollSteps) else sendStab(0, 0) end
        sendNav(thrustCmd, yawSteps, strafeCmd)

        -- ---- publish for display ----
        local age = apLastMsg and (now - apLastMsg)
        ui.link = (not age) and "no link" or (age > CFG.AP_STALE_AFTER and string.format("STALE %ds", math.floor(age)) or "ok")
        ui.posX, ui.posY, ui.posZ = pos.x, pos.y, pos.z
        ui.pitch, ui.roll = select(1, euler(q_cur)), select(3, euler(q_cur))
        ui.headingDeg = math.deg(wrapPi(heading))
        ui.desHeadingDeg = math.deg(wrapPi(desiredHeading))
        ui.yawRate = w.y
        ui.phase = hasTarget and navPhase or "NO TGT"
        ui.dist, ui.bearingDeg = dist, math.deg(wrapPi(bearing))
        ui.xtk = hasLeg and (CFG.XTK_SIGN * xtk) or nil
        ui.targetAlt, ui.velY, ui.desVelY = TARGET_ALT, velY, desiredVelY
        ui.altLevel = altLevel
        ui.pStep, ui.rStep, ui.yStep = outStab, outNav.yaw, nil
        ui.thr, ui.yaw, ui.str = outNav.thrust, outNav.yaw, outNav.str or outNav.strafe
        ui.alState = alState
        ui.tgtName = navTarget[4]

        sleep(0.05)
    end
end

-- RECEIVE LOOPS ---------------------------------------------------------------
local function navReceiveLoop()
    while true do
        local sender, msg = rednet.receive(CFG.APNAV_PROTOCOL, 1)
        if sender and type(msg) == "table" then
            if msg.cancel then
                navTarget = { nil, nil, nil, '' }
                navPrev, navPhase = nil, "ENROUTE"
                navHoldHeading, navCruiseCap = nil, CFG.CRUISE_SPEED
                apLastMsg = os.clock()
            else
                local x = tonumber(msg.x)
                local z = tonumber(msg.y)   -- protocol: msg.y is world Z
                if x and z then
                    local curX, curZ = navTarget[1], navTarget[2]
                    local isNew = type(curX) ~= "number" or type(curZ) ~= "number"
                        or math.abs(x - curX) > CFG.WP_EPSILON or math.abs(z - curZ) > CFG.WP_EPSILON
                    if isNew then
                        if type(curX) == "number" and type(curZ) == "number" then navPrev = { curX, curZ } end
                        navTarget = { x, z, tostring(msg.type or ""), tostring(msg.name or "") }
                    end
                    navPhase = tostring(msg.phase or "ENROUTE")
                    navHoldHeading = tonumber(msg.holdHeading)
                    navCruiseCap = tonumber(msg.cruiseCap) or CFG.CRUISE_SPEED
                    apLastMsg = os.clock()
                end
            end
        end
    end
end

local function altReceiveLoop()
    while true do
        local sender, msg = rednet.receive(CFG.ALTAP_PROTOCOL, 1)
        if sender and type(msg) == "table" then
            local a1 = tonumber(msg.alti)
            if a1 then AP_ALTI = clamp(a1, CFG.AP_ALT_MIN, CFG.AP_ALT_MAX) end
            -- Ignore ALT-AP while autoland owns the vertical channel.
            if alState == "normal" and a1 then setTarget(AP_ALTI) end
            if a1 then altLastMsg = os.clock() end
        end
    end
end

local function autolandReceiveLoop()
    while true do
        local sender, msg = rednet.receive(CFG.AUTOLAND_PROTOCOL, 1)
        if sender and type(msg) == "table" then
            local cmd = msg.cmd
            if cmd == "cancel" or cmd == "off" then
                -- cancel = resume/abort (AS restores its pick); off = AP OFF (AS stays
                -- off). Same for us either way: drop the landing latch, track ALT-AP again
                -- -- with no altap that just holds the current (ground) altitude.
                alState = "normal"
            elseif cmd == "release" then
                -- Pilot overrode the descent mid-landing: hand the VERTICAL channel back
                -- to ALT-AP (same as cancel for us) while the FMS keeps streaming the
                -- LANDING navap so position/heading alignment stays online.
                alState = "normal"
            elseif cmd == "hold" then
                if alState == "normal" and latestPosY then setTarget(latestPosY) end
                alState = "hold"
            elseif cmd == "descend" then
                local vs = tonumber(msg.vs)
                if vs then alDescendVS = clamp(vs, -CFG.AL_DESC_MAX, CFG.AL_CLIMB_MAX) end
                alState = "descend"
            elseif cmd == "landed" then
                if alState ~= "landed" and latestPosY then setTarget(latestPosY) end
                alState = "landed"
            end
            alLastMsg = os.clock()
        end
    end
end

-- DISPLAY: 2D artificial horizon + readouts ("numbers whizzing by is not
-- debuggable" -- draft). The horizon shows ACTUAL pitch/roll; the fixed centre
-- crosshair is the aircraft, so level = horizon through the middle.
local function drawAH(pitch, roll)
    local W = term.getSize()
    local top, h = CFG.AH_TOP, CFG.AH_HEIGHT
    local midY = top + math.floor(h / 2)
    local midX = math.floor(W / 2) + 1
    local slope = math.tan(clamp(roll, -1.2, 1.2))
    local pitchOff = clamp(pitch * CFG.PITCH_PIXELS_PER_RAD, -h, h)

    for col = 1, W do
        -- horizon row for this column: pitch shifts it, roll tilts it.
        local hy = midY + pitchOff - (col - midX) * slope
        local hrow = round(hy)
        for row = top, top + h - 1 do
            term.setCursorPos(col, row)
            if row < hrow then
                term.setBackgroundColour(colours.blue)      -- sky
            elseif row > hrow then
                term.setBackgroundColour(colours.brown)     -- ground
            else
                term.setBackgroundColour(colours.white)      -- horizon
            end
            term.write(" ")
        end
    end
    -- fixed aircraft reference (a small "-o-" at centre)
    term.setBackgroundColour(colours.black)
    term.setTextColour(colours.orange)
    term.setCursorPos(midX - 2, midY); term.write("-")
    term.setCursorPos(midX,     midY); term.write("o")
    term.setCursorPos(midX + 2, midY); term.write("-")
    term.setBackgroundColour(colours.black)
end

local function line(row, text)
    term.setCursorPos(1, row); term.clearLine(); term.write(text)
end

local function displayLoop()
    while true do
        drawAH(ui.pitch or 0, ui.roll or 0)
        term.setTextColour(colours.white)
        local r = CFG.AH_TOP + CFG.AH_HEIGHT
        line(r + 0, string.format("P%+5.1f R%+5.1f [%s/%s %s]",
            math.deg(ui.pitch or 0), math.deg(ui.roll or 0), CFG.ATT_MODE, CFG.SENSOR_FRAME, quatCtorName))
        line(r + 1, string.format("HDG %+6.1f -> %+6.1f  YR %+5.2f",
            ui.headingDeg or 0, ui.desHeadingDeg or 0, ui.yawRate or 0))
        line(r + 2, string.format("STAB F%d B%d L%d R%d Y%+3d raw%.0f",
            (ui.pStep or outStab).front, (ui.pStep or outStab).back,
            (ui.pStep or outStab).left, (ui.pStep or outStab).right, ui.yaw or 0, ui.rotRaw or 0))
        line(r + 7, string.format("TRIM iP%+.2f iR%+.2f yS%.2f gy%.2f", ui.integP or 0, ui.integR or 0, ui.yawScale or 1, ui.gyroFade or 0))
        line(r + 8, string.format("I(k) x%.0f y%.0f z%.0f", (ui.Ix or 0)/1000, (ui.Iy or 0)/1000, (ui.Iz or 0)/1000))
        term.setTextColour(colours.orange)
        line(r + 3, string.format("PHASE %-7s  DIST %6.1f", ui.phase or "--", ui.dist or 0))
        term.setTextColour(colours.white)
        if ui.xtk then
            line(r + 4, string.format("XTK %+5.1fm  THR %+3d  STR %+3d", ui.xtk, ui.thr or 0, ui.str or 0))
        else
            line(r + 4, string.format("XTK  ---     THR %+3d  STR %+3d", ui.thr or 0, ui.str or 0))
        end
        line(r + 5, string.format("ALT %6.1f -> %s  Vy%+5.2f o%2d up%+.2f",
            ui.posY or 0, ui.targetAlt and string.format("%.0f", ui.targetAlt) or "--",
            ui.velY or 0, ui.altLevel or CFG.ALT_NEUTRAL, ui.tiltUp or 1))
        local al = (ui.alState ~= "normal") and ("AUTOLAND " .. string.upper(ui.alState or "")) or ("LINK " .. (ui.link or "--"))
        term.setTextColour(ui.link == "ok" and colours.green or colours.orange)
        line(r + 6, al)
        term.setTextColour(colours.white)
        sleep(0.1)
    end
end

-- Kill outputs on exit; hold the learnt altitude trim rather than a blind neutral.
local function shutdown()
    pcall(function()
        rednet.broadcast({ left = 0, right = 0, front = 0, back = 0 }, CFG.STAB_PROTOCOL)
        rednet.broadcast({ thrust_forward = 0, thrust_backward = 0, yaw_left = 0,
            yaw_right = 0, strafe_left = 0, strafe_right = 0 }, CFG.NAVCTRL_PROTOCOL)
        rednet.broadcast({ alt = clamp(CFG.ALT_NEUTRAL + CFG.VEL_KI * velPID.integral, CFG.ALT_MIN, CFG.ALT_MAX) }, CFG.ALT_PROTOCOL)
    end)
end

term.setBackgroundColour(colours.black); term.clear()
local ok, err = pcall(parallel.waitForAny,
    controlLoop, navReceiveLoop, altReceiveLoop, autolandReceiveLoop, displayLoop)
shutdown()
term.setBackgroundColour(colours.black); term.setTextColour(colours.white)
term.setCursorPos(1, term.getSize() and select(2, term.getSize()) or 19)
if not ok then print("UAC stopped: " .. tostring(err)) end
