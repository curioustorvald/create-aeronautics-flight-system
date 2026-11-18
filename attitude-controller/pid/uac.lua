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

-- CONFIG ----------------------------------------------------------------------
-- All tuning/calibration lives in pid/uacconf.lua (see its header). Kept out of this
-- file so a program update never overwrites the airframe's calibration, and so the
-- knobs are editable without scrolling a 1800-line program. There is no fallback copy
-- of these values here on purpose -- a second set of defaults would let an edit here
-- silently do nothing, which would make empirical sign-calibration untrustworthy.
-- Everything but CFG itself is scoped inside the do-block so the load temporaries do not
-- occupy top-level registers: this file has already hit Cobalt's 200-locals cap once (see
-- the note at the CFG table's old home), and the cap counts the main chunk's LIVE locals
-- against every nested function. Net cost of this loader is therefore 1 local -- exactly
-- what `local CFG = {}` cost before.
local CFG
do
    local ok, res = pcall(require, "uacconf")
    if not ok then
        error("Cannot load pid/uacconf.lua (the UAC's calibration file): " .. tostring(res)
            .. "\nIt must sit beside uac.lua and end with 'return CFG'.", 0)
    end
    if type(res) ~= "table" then
        error("pid/uacconf.lua did not return the CFG table (got " .. type(res) .. ").", 0)
    end
    -- Canary check: catch a truncated or half-saved config at BOOT, naming the missing knob,
    -- rather than as a nil-arithmetic crash mid-flight. One representative key per section --
    -- not exhaustive, but a file with all of these is the real config, not a fragment.
    for _, k in ipairs({ "APNAV_PROTOCOL", "ATT_MODE", "ATT_KP", "RATE_KP", "PITCH_SIGN",
            "ROLL_SIGN", "YAW_SIGN", "ROT_GAIN_PITCH", "MAX_RATE_YAW", "FLIGHT_MODE",
            "BANK_SIGN", "BANK_MAX", "BANK_XTK_SIGN", "THRUST_SIGN", "STRAFE_SIGN",
            "CRUISE_SPEED", "STEPS_PER_MPS", "CLIMB_MAX", "ALT_NEUTRAL", "AP_STALE_AFTER",
            -- Added after the config split. Both are read UNCONDITIONALLY every tick, so an
            -- airframe whose uacconf predates them dies of nil-arithmetic in controlLoop
            -- rather than at boot -- exactly how #19 was left unbootable by STRAFE_EAGER_*.
            "HAS_ALT_THRUSTER", "FW_PITCH_ENABLE", "FW_PITCH_SIGN", "FW_PITCH_MIN_SPEED",
            "FW_PITCH_HEADROOM", "AL_GROUND_CUT",
            -- Same rule, same reason -- both are read UNCONDITIONALLY every tick (the heading
            -- bank washout in controlLoop, the float-thrust protocol in sendNav), so a uacconf
            -- predating them must fail HERE and not mid-turn.
            "BANK_HDG_KD_WASHOUT", "THRUSTF_PROTOCOL",
            -- Same rule again (2026-08-28, the loop-rate fork). Both are read UNCONDITIONALLY
            -- on every tick -- LOOP_PERIOD by the pacing at the bottom of controlLoop,
            -- SLOW_POLL_PERIOD by the inertia/mass poll at the top -- and a nil in either is a
            -- first-tick arithmetic crash, so a uacconf predating the fork must fail HERE.
            -- LOOP_PERIOD in particular must not be allowed to read nil and be treated as 0:
            -- that would turn the pacing into a yield-only busy loop with no sleep in it at
            -- all, which is the one failure mode of this change that would be hard to see.
            "LOOP_PERIOD", "SLOW_POLL_PERIOD",
            -- Same rule again (2026-08-20): the horizontal channel now scales off a measured
            -- authority the way the vertical one does, and these are read on every navigating
            -- tick. A uacconf predating them must fail HERE, not as a nil-arithmetic crash the
            -- first time the craft is asked to translate. CFG.ATT_KP_YAW is deliberately NOT
            -- listed -- uac.lua reads it as `or CFG.ATT_KP`, so absent is a valid answer.
            "STEPS_PER_MPS_FWD", "STEPS_PER_MPS_LAT", "STRAFE_VEL_KP",
            -- Same rule again (2026-08-21): the breakaway floor is per-axis now and this half
            -- is read on every translating tick, so a uacconf predating it must fail HERE.
            "TRANSLATE_MIN_CMD_FWD",
            -- Same rule again (2026-08-21), with one difference worth stating: this one is a
            -- BOOLEAN, so a uacconf predating it would NOT crash -- it would read nil, evaluate
            -- false, and leave the VS stick's top detent quietly doing nothing at all. That is
            -- precisely the failure this knob exists to end (the detent used to be sent as
            -- `vs = 100`, a command this file then silently clamped away to CLIMB_MAX), so a
            -- missing one has to fail LOUDLY at boot rather than fail invisibly in the air.
            "MANCON_SCRAMBLE",
            -- MANCON_CLIMB_MAX / MANCON_DESCEND_MAX (2026-09-08): the MANUAL vertical's own
            -- rate envelope, separate from CLIMB_MAX / DESCEND_MAX so that mancon.lua's boost
            -- key can reach past the autopilot's handling limit without loosening it for the
            -- autoland and ALT-AP as well. Read unconditionally in the manual branch below, so
            -- a uacconf predating them is a nil-arithmetic crash the first time ALT-AP is
            -- switched off in flight -- i.e. airborne, hand-flying. They fail HERE instead.
            "MANCON_CLIMB_MAX", "MANCON_DESCEND_MAX",
            -- Same rule again (2026-08-29), and it is the one on this list where a WRONG
            -- answer is worse than a missing one. CFG.ALT_OFFSET is how far the reported pose
            -- sits above the bottom of the hull; it is subtracted on the first line of every
            -- tick, so a uacconf predating it is a nil-arithmetic crash on tick one. It must
            -- fail HERE, and it must not be given a default of 0 anywhere: 0 is a legitimate
            -- value (the display bench), so a defaulted 0 would quietly restore the old
            -- over-reading altimetry on a real airframe and look exactly like a working one.
            "ALT_OFFSET" }) do
        if res[k] == nil then
            error("pid/uacconf.lua is missing CFG." .. k .. " -- the file looks incomplete.", 0)
        end
    end
    -- CONDITIONAL requirement, not a canary (2026-08-21, at the four-way merge). The Smith
    -- predictor's keys are deliberately NOT in the list above -- absent is a valid answer and
    -- means "no dead-time compensation", which is what every airframe but #19 wants. But once
    -- PITCH_SMITH_ENABLE is TRUE the other two are read on every attitude tick, and
    -- PITCH_SMITH_ACCEL_PER_STEP in particular is read inside attitude() where a nil would be a
    -- first-tick arithmetic crash. Now that all airframes share one uac.lua this flag is one
    -- edit away on any of them, so switching it on with a half-filled config must fail HERE.
    if res.PITCH_SMITH_ENABLE then
        for _, k in ipairs({ "PITCH_SMITH_TD", "PITCH_SMITH_ACCEL_PER_STEP" }) do
            if res[k] == nil then
                error("pid/uacconf.lua sets CFG.PITCH_SMITH_ENABLE but is missing CFG." .. k
                    .. " -- the pitch Smith predictor needs both.", 0)
            end
        end
    end
    -- CONDITIONAL requirement, same rule (2026-08-21, Scout #72). The triangular thrust
    -- allocator's knobs are deliberately NOT in the canary list above: absent is a valid
    -- answer and means "this airframe has axis thruster banks, not three vectoring nozzles",
    -- which is every aircraft on the fleet but Scout. But once TRI_ENABLE is true every one
    -- of these is read on EVERY tick inside sendVector, where a nil is a first-tick
    -- arithmetic crash, and TRI_ENGINES is read at boot to build the allocation itself. All
    -- airframes share one uac.lua, so this flag is one edit away on any of them -- switching
    -- it on with a half-filled config has to fail HERE and name the knob.
    -- Same conditional rule again (2026-08-22): absent means "no flight data recorder", which
    -- is every airframe but whichever one is being debugged. Present means all five are read
    -- on every tick inside LOG.tick.
    if res.LOG_ENABLE then
        for _, k in ipairs({ "LOG_FILE", "LOG_DELAY", "LOG_SECONDS", "LOG_EVERY" }) do
            if res[k] == nil then
                error("pid/uacconf.lua sets CFG.LOG_ENABLE but is missing CFG." .. k
                    .. " -- the flight data recorder needs the whole block.", 0)
            end
        end
    end
    if res.TRI_ENABLE then
        for _, k in ipairs({ "TRI_PROTOCOL", "TRI_ENGINES", "TRI_HOVER", "TRI_LIFT_PER_ALTSTEP",
                "TRI_PITCH_PER_STEP", "TRI_ROLL_PER_STEP", "TRI_YAW_PER_STEP",
                "TRI_LAT_PER_STEP", "TRI_FWD_PER_STEP", "TRI_PITCH_SIGN", "TRI_ROLL_SIGN",
                "TRI_YAW_SIGN", "TRI_LAT_SIGN", "TRI_FWD_SIGN", "TRI_T_MIN", "TRI_T_MAX",
                "TRI_W_FLOOR", "TRI_VEC_MAX", "TRI_VEC_MAX_ANGLE", "TRI_VEC_X_SIGN",
                "TRI_VEC_Y_SIGN" }) do
            if res[k] == nil then
                error("pid/uacconf.lua sets CFG.TRI_ENABLE but is missing CFG." .. k
                    .. " -- the triangular thrust allocator needs the whole block.", 0)
            end
        end
    end
    CFG = res
end

-- Set CFG.FLIGHT_MODE and derive its flags (unrecognised -> VTOL). A function, not an inline
-- block, so the on-screen MODE button (inputLoop) can switch modes at runtime and re-derive.
-- The flags stay in CFG (not new top-level locals) to respect Cobalt's 200-locals cap.
local function applyFlightMode(mode)
    if mode ~= "HYBRID" and mode ~= "FIXEDWING" then mode = "VTOL" end
    CFG.FLIGHT_MODE    = mode
    CFG.STRAFE_ENABLED = (mode ~= "FIXEDWING")
    CFG.ATTITUDE_BANKS = (mode == "HYBRID" or mode == "FIXEDWING")
    CFG.MODE_TAG       = (mode == "HYBRID" and "HYB") or (mode == "FIXEDWING" and "FW") or "VTOL"
end
applyFlightMode(CFG.FLIGHT_MODE)   -- derive from the source-configured default

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
-- getYaw() was here. It is gone (2026-08-28): its only caller was controlLoop's heading, which
-- now takes the yaw out of the one euler() the tick already does for the display. A wrapper
-- that decomposes a quaternion to keep one of three angles is exactly the call this fork
-- exists to stop making, and an unused top-level local still counts against Cobalt's 200.

-- THE REDSTONE SCALE. Not an authority limit and not a calibration -- it is what one
-- redstone line is worth in Minecraft, and it is the denominator every output in this file
-- normalises on. CFG.STAB_MAX and friends are the authority limits and are applied FIRST, as
-- a clamp in step units, so an airframe that grants less than full authority still gets less
-- than full authority at the far end (normalising on *_MAX instead would hand the limit
-- straight back when the relay multiplied by 15 again).
local RS_MAX = 15
-- PITCH SMITH PREDICTOR (2026-08-14) -- dead-time compensation for the pitch rate loop.
-- WHY IT IS NEEDED, measured not assumed: the actuator dead time is ~1 s (pilot timed a 60 deg
-- step), and the old +/-4 deg limit cycle ran at 4 s = 1.5 rad/s, where 1 s of delay is 86 deg of
-- phase -- that cycle WAS the dead-time instability, which is why ROT_GAIN 1.6e-4/ATT_KP 0.28 had
-- to be abandoned. Usable bandwidth without compensation is ~0.3-0.5/tau_d, and the loop sits at
-- omega_n = 0.25 rad/s, i.e. already at the wall. No gain buys pitch response past this point.
-- HOW: the plant is an INTEGRATOR (torque -> angular accel -> rate), so the rate the aeroplane
-- will have once the pipeline drains is the measured rate PLUS every acceleration already
-- commanded within the last tau_d but not yet felt. Keep those in a ring buffer and add them.
-- That is a Smith predictor specialised to an integrator plant, and it needs only tau_d and the
-- accel-per-step -- the latter cancels out here, because the buffer stores the DELIVERED accel
-- computed from the same ROT_GAIN_PITCH the output used.
-- **THIS IS THE MOST MODEL-SENSITIVE THING IN THIS FILE. It fails INVISIBLY -- a wrong tau_d
-- makes the loop trust a wrong prediction rather than doing anything visible on screen. If pitch
-- gets worse after enabling it, set CFG.PITCH_SMITH_ENABLE = false FIRST, before touching any
-- gain, and re-measure tau_d.** Display tag "SMI" on the TRIM line shows when it is active.
local smithBuf, smithIdx, smithSum = {}, 1, 0
local smithN = (CFG.PITCH_SMITH_ENABLE and CFG.PITCH_SMITH_TD and CFG.PITCH_SMITH_TD > 0)
    and math.max(1, math.floor(CFG.PITCH_SMITH_TD / 0.05 + 0.5)) or 0
for i = 1, smithN do smithBuf[i] = 0 end
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

-- SLOW-POLLED PHYSICAL PROPERTIES (2026-08-28, the loop-rate fork -- see the pacing note at
-- the bottom of controlLoop for the measurement this all comes from).
--
-- The inertia tensor and the mass are the two things this loop read at 20 Hz that DO NOT
-- CHANGE at 20 Hz. They are properties of the assembly, and they move only when the assembly
-- does -- a block added, a block broken, cargo loaded. Reading them every tick cost two
-- blocking round trips to the server's main thread, out of the ten this loop makes, for two
-- numbers that were the same as last tick's ~99.9% of the time.
--
-- So they are polled on CFG.SLOW_POLL_PERIOD instead and cached here. What that costs, stated
-- plainly so nobody has to re-derive it: a mass change is noticed up to SLOW_POLL_PERIOD late,
-- which delays the CFG.TRIM_BOOST_TIME kick by that much. CFG.TRIM_BOOST_TIME is measured in
-- seconds and the boost exists to re-learn a hover trim over the seconds AFTER a mass step, so
-- a fraction of a second of notice is immaterial. Nothing else reads either value.
--
-- ONE TABLE, not six top-level locals, for the reason the CFG table itself exists: Cobalt caps
-- a function at 200 locals and counts the main chunk's live locals against every nested
-- function, and this file has hit that ceiling once already.
local SLOW = {
    m11 = 1, m12 = 0, m13 = 0,      -- the tensor, unpacked ONCE per poll into nine numbers
    m21 = 0, m22 = 1, m23 = 0,
    m31 = 0, m32 = 0, m33 = 1,
    Ix = 1, Iy = 1, Iz = 1,         -- ...and its body-frame diagonal, which is all the
                                    -- DECOUPLED path ever wanted out of it
    mass = 0,
    next = -1,                      -- os.clock() of the next poll; -1 = poll on the first tick
}

-- Unpack the tensor's unknown shape (nested [i][j], flat 9, or {m11=..}) into SLOW's nine
-- fields. Called ONLY from the slow poll, so the shape sniffing and -- more to the point --
-- the two throwaway tables the old matVec built on EVERY call now happen once a second
-- instead of three times a tick. Also derives the body-frame diagonal: Ix/Iy/Iz are the
-- tensor's quadratic form on the three body unit axes, i.e. simply m11/m22/m33, which is what
-- the DECOUPLED branch was spending three matVec calls to rediscover every tick.
local function unpackTensor(M)
    if type(M) ~= "table" then
        SLOW.m11, SLOW.m12, SLOW.m13 = 1, 0, 0
        SLOW.m21, SLOW.m22, SLOW.m23 = 0, 1, 0
        SLOW.m31, SLOW.m32, SLOW.m33 = 0, 0, 1
    elseif type(M[1]) == "table" then                -- nested M[i][j]
        SLOW.m11, SLOW.m12, SLOW.m13 = M[1][1], M[1][2], M[1][3]
        SLOW.m21, SLOW.m22, SLOW.m23 = M[2][1], M[2][2], M[2][3]
        SLOW.m31, SLOW.m32, SLOW.m33 = M[3][1], M[3][2], M[3][3]
    elseif type(M[1]) == "number" then               -- flat row-major 9
        SLOW.m11, SLOW.m12, SLOW.m13 = M[1], M[2], M[3]
        SLOW.m21, SLOW.m22, SLOW.m23 = M[4], M[5], M[6]
        SLOW.m31, SLOW.m32, SLOW.m33 = M[7], M[8], M[9]
    else                                             -- named fields m11.. or c0..
        SLOW.m11, SLOW.m12, SLOW.m13 = M.m11 or M[0] or 1, M.m12 or 0, M.m13 or 0
        SLOW.m21, SLOW.m22, SLOW.m23 = M.m21 or 0, M.m22 or 1, M.m23 or 0
        SLOW.m31, SLOW.m32, SLOW.m33 = M.m31 or 0, M.m32 or 0, M.m33 or 1
    end
    SLOW.Ix, SLOW.Iy, SLOW.Iz = SLOW.m11, SLOW.m22, SLOW.m33
end

-- mat3x3 * vec3 against the cached tensor. Only used in INERTIA mode, and only on the
-- SENSOR_FRAME == "world" path now (the body path reads SLOW.Ix/Iy/Iz directly). Allocates
-- nothing: it reads the nine numbers unpackTensor already sniffed out.
local function matVec(x, y, z)
    return SLOW.m11*x + SLOW.m12*y + SLOW.m13*z,
           SLOW.m21*x + SLOW.m22*y + SLOW.m23*z,
           SLOW.m31*x + SLOW.m32*y + SLOW.m33*z
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
local function integStep(cur, e, rate, dt, idle)
    -- Parked / engine-off: never learn a trim from a craft that isn't flying (the tilt it
    -- reads is imposed by the ground, not a moment the thrusters could null) -- bleed any
    -- wind-up accrued while parked away so the next flight starts from a clean trim.
    if idle then return cur * CFG.ATT_I_IDLE_BLEED end
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

-- Slew-rate limiter: move `cur` toward `target` by at most `maxStep` this call.
-- Smooths the commanded bank/pitch so a turn or climb rolls/pitches in and out
-- gently instead of snapping the setpoint (which would jerk the attitude loop).
local function slew(cur, target, maxStep)
    local d = target - cur
    if d >  maxStep then d =  maxStep end
    if d < -maxStep then d = -maxStep end
    return cur + d
end

-- STATE (set by the receive loops; read by the control loop) ------------------
local navTarget = { nil, nil, nil, '' }   -- {X, Z, type, name}
local navPrev   = nil                     -- {X, Z} previous waypoint, or nil
local navPhase  = "ENROUTE"
local navHoldHeading = nil
local navCruiseCap   = CFG.CRUISE_SPEED
local navCruiseCapDynamic = false   -- FMS schedules the speed (schedSpeed): track cruiseCap as a
                                    -- speed target, not just a ceiling. false -> old per-fix decel.
local navCarrot = false             -- target is an en-route carrot on the FMS's smoothed path:
                                    -- pure-pursue it (velocity-vector, no cross-track leg hold).
local navGlide  = false             -- the FMS is flying a RUNWAY GLIDESLOPE (msg.glide): the
                                    -- vertical channel is in a commanded descent, but this is
                                    -- still a flying leg down a centreline, NOT the wings-level
                                    -- sink onto a pad every other autoland state means. Its only
                                    -- job here is to keep bankSteer alive -- without it a
                                    -- FIXEDWING approach would have no lateral control at all
                                    -- (no strafe, and banking switched off by alState). See the
                                    -- LG.RWY_* block in 8/autopilot.lua.
local navAlign  = false             -- the FMS is holding cruiseCap at 0 for its route-start
                                    -- ALIGN (msg.align, NAV.aligning): nose swinging onto a
                                    -- fresh bearing, most commonly after cancelling and
                                    -- re-committing to a new destination mid-flight. See
                                    -- `alignStop` below for why this routes through the
                                    -- velocity-vector law on VTOL/HYBRID but not FIXEDWING.
local apLastMsg = nil

-- Pilot autopilot mode (from #12) and manual vertical (from #6). Held between
-- messages: these are latching switches / a continuously-streamed stick, so a dropped
-- packet must not disengage anything -- we keep the last value and only flag staleness.
local AP_NAV = CFG.AP_NAV_DEFAULT
local AP_ALT = CFG.AP_ALT_DEFAULT
local apswLastMsg = nil
local manVS = 0            -- pilot's wanted climb rate (m/s) from the VSC #6
-- The VS stick's TOP DETENT (level 15 on the VSC), which is not a rate at all but a discrete
-- "lift engine to full throttle" -- see CFG.MANCON_SCRAMBLE. Latched from the same packet as
-- manVS, so manLastMsg ages both and neither can outlive the link.
local manScramble = false
local manLastMsg = nil
local manFwd, manStrafe, manYaw = 0, 0, 0  -- pilot's wanted body velocities (m/s, m/s, rad/s), MIC #14
local manNavActive  = false                -- MIC reports a key currently held (vs. sticks centred)
local manNavLastMsg = nil
-- Keyboard joystick (#21): the hand-flown pitch/roll TARGET (rad) and throttle lever (0..1).
-- ONE table, not five locals -- the 200-locals rule this file's CFG table exists for applies
-- to state as much as to knobs. Held between packets like every other manual channel; the
-- control loop gates on freshness, on FIXEDWING, and on AP-NAV (see CFG.KJOY_ENABLE).
local kj = { pitch = 0, roll = 0, thr = 0, active = false, last = nil }

-- FIXEDWING vertical-by-pitch (CFG.FW_PITCH_*): the integral of the vertical-SPEED error
-- that trims the commanded flight path angle. One table for the same 200-locals reason as
-- kj above, and kept at top level (not inside controlLoop) so it survives nothing -- it is
-- simply the one piece of state the PI needs between ticks.
local fw = { pitchI = 0 }

-- HYBRID/FIXEDWING heading-bank damper (CFG.BANK_HDG_KD): the WASHOUT reference -- a slow
-- low-pass of the measured yaw rate. Same table-not-a-local reason as kj/fw above. See the
-- CFG.BANK_HDG_KD_WASHOUT block for why the damper needs one at all.
local bnk = { yawWash = 0 }

local AP_ALTI    = nil
local altLastMsg = nil
local alState     = "normal"   -- normal | hold | descend | landed
local alGrounded  = false      -- ...and, while "landed", the gear's squat contact is MADE, so
                               -- we are down on GROUND TRUTH rather than on the altimeter's
                               -- opinion. Only ever true in the "landed" state.
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
-- THIS FILE NO LONGER QUANTISES ANYTHING. Every output leaves here as a FLOAT in [-1, 1] --
-- the demand as the control law computed it, expressed as a fraction of one full redstone
-- line -- and the relay at the far end is what turns it into a whole 0..15 level.
--
-- WHY THE SIGMA-DELTA MOVED (2026-08-26). It used to live here: seven accumulators, one per
-- channel, rounding to whole steps before broadcast. That was only ever right while the UAC
-- knew what its relay would do with the number, and it stopped being true the moment relays
-- started spending a demand on something other than one 0..15 line. #69 already does: it
-- splits fore/aft over TWO bundles at 45 A-units of resolution, and it had to be handed the
-- raw float on a protocol of its own (THRUSTF) to get it, because the NAVCTRL message had
-- already thrown 2/3 of the resolution away. Scout's nozzle computers are the same case and
-- said so outright in sendVector's header. Rounding at this end destroys resolution the
-- receiver may well have and can never get back; rounding at the far end costs nothing and
-- needs no second protocol. So the rule is now uniform across every output in this file:
-- the UAC states the demand, the relay decides how to spend it.
--
-- WHAT DID *NOT* MOVE: the authority limits. CFG.STAB_MAX / YAW_MAX / THRUST_MAX /
-- STRAFE_MAX / ALT_MIN / ALT_MAX are still applied HERE, as a clamp in step units before the
-- division by RS_MAX, because they are this airframe's calibration and belong with the rest
-- of it. A relay never widens what it is sent.
--
-- THE ONE BEHAVIOURAL CHANGE, and it is an improvement: the accumulator is now fed the
-- CLAMPED command, where before it was fed the raw one. Bench-diffed against the pre-refactor
-- build the two chains are step-for-step identical over 200 ticks on all five airframes and
-- over 60k fuzzed in-band commands -- they part company only when a command SATURATES. The
-- old sigma-delta went on integrating the part of the demand the actuator could never
-- deliver, and paid that phantom debt out over the frames after the command came back into
-- band (81 of 200 frames off by a step, on a fuzz that saturates then releases). That is
-- integrator wind-up, not sub-step recovery. Clamping first is the standard cure.
--
-- The out* tables below are the DISPLAY copies and now hold the same floats as the wire.
local outStab   = { pitch = 0, roll = 0 }
local outNav    = { thrust = 0, yaw = 0, strafe = 0, thrustF = 0 }
local outAlt    = { up = CFG.ALT_NEUTRAL / RS_MAX, down = CFG.ALT_NEUTRAL / RS_MAX }

-- pitch/roll arrive as signed STEPS and go out as signed FLOATS [-1,1]. Which physical
-- thruster bank each ends up on -- front/back for pitch, left/right for roll -- is the
-- relay's business now; CFG.SWAP_PITCH_ROLL_CHANNELS stays here because it is a statement
-- about how THIS airframe is wired, and the relay has no conf file to hold it in.
local function sendStab(pitch, roll)
    local p = clamp(pitch, -CFG.STAB_MAX, CFG.STAB_MAX) / RS_MAX
    local r = clamp(roll,  -CFG.STAB_MAX, CFG.STAB_MAX) / RS_MAX
    if CFG.SWAP_PITCH_ROLL_CHANNELS then p, r = r, p end
    rednet.broadcast({ pitch = p, roll = r }, CFG.STAB_PROTOCOL)
    outStab = { pitch = p, roll = r }
end

-- thrust +fwd/-back, yaw +right/-left, strafe +right/-left; all signed floats [-1,1].
local function sendNav(thrust, yaw, strafe)
    -- Fore/aft on its own protocol, and normalised on CFG.THRUST_MAX rather than on RS_MAX:
    -- THRUSTF's +1 means FULL FORWARD AUTHORITY, whatever redstone encoding the receiver
    -- happens to use, and #69 spends it on 45 A-units rather than on 15 steps. The two
    -- denominators coincide on every airframe in the fleet today (THRUST_MAX is 15
    -- everywhere) -- they are written differently because they MEAN differently.
    local thrustF = (CFG.THRUST_MAX > 0) and clamp(thrust / CFG.THRUST_MAX, -1, 1) or 0
    local t = clamp(thrust, -CFG.THRUST_MAX, CFG.THRUST_MAX) / RS_MAX
    local y = clamp(yaw,    -CFG.YAW_MAX,    CFG.YAW_MAX)    / RS_MAX
    local s = clamp(strafe, -CFG.STRAFE_MAX, CFG.STRAFE_MAX) / RS_MAX
    -- VECTOR-NOZZLE AIRFRAME (CFG.TRI_ENABLE): yaw and strafe are not thruster banks here --
    -- the three nozzles make both, and sendVector has already spent them. The NAVCTRL message
    -- still goes out because the fore/aft bank IS real and IS on it (#79), but those two axes
    -- are pinned to zero so a relay wired to them later cannot double-command an axis the
    -- nozzles are already flying. outNav below keeps the TRUE commands: they are what the
    -- display reads, and zeroing them there would hide the yaw the craft is actually making.
    local yw, sw = y, s
    if CFG.TRI_ENABLE then yw, sw = 0, 0 end
    rednet.broadcast({ thrust = t, yaw = yw, strafe = sw }, CFG.NAVCTRL_PROTOCOL)
    rednet.broadcast({ thrust = thrustF }, CFG.THRUSTF_PROTOCOL)
    outNav = { thrust = t, yaw = y, strafe = s, thrustF = thrustF }
end

-- Split the signed vertical command into independent up- and down-thruster levels
-- (replacing the external splitter box). steps 0 -> both at ALT_NEUTRAL (balanced
-- hover); +steps drives the up line and backs off the down line, -steps the reverse.
--
-- THE SPLIT STAYS HERE, THE QUANTISATION DOES NOT. CFG.ALT_NEUTRAL is the balanced-hover
-- burn this hull needs to sit still, which is airframe calibration of the first order and
-- has no business on a relay with no conf file; so this function still does NEUTRAL +/- steps
-- and still clamps to the ALT_MIN..ALT_MAX authority. What goes on the wire is the pair of
-- line levels as FRACTIONS of a full line, [0,1], and each relay line dithers its own.
--
-- `cut` is the one thing steps cannot say: BOTH LINES TO A FLAT ZERO, i.e. the lift engine
-- switched OFF. steps 0 is not that -- it is a balanced hover, still burning on both lines --
-- and on the ground that is exactly what has to stop (see CFG.AL_GROUND_CUT). It travels as
-- its own flag rather than as a pair of zeroes, so the relay can bypass its accumulators
-- instead of feeding them: they hold at most half a step, and nothing should be owed when
-- the engine comes back.
--
-- `openThrottle` IS THE SCRAMBLE (CFG.MANCON_SCRAMBLE), and it changes the SPLIT, not the
-- magnitude and not the sign. NEUTRAL +/- steps is the right split for a RATE command: the
-- two banks are a differential about a balanced hover, and the down line backing off is half
-- of what buys a climb. It is the wrong split for an OPEN THROTTLE, and that is the bug this
-- flag fixes -- "the scramble is nullified by the alt-down thrusters", 2026-08-29.
--
-- WHY IT WAS NULLIFIED. A scramble asks for CFG.ALT_SWING outright, but the tilt allocation
-- routes it by attitude before it gets here, so what arrives is ALT_SWING * (world-up .
-- body-up). The down line only reaches ALT_MIN while that product is >= ALT_NEUTRAL -- with
-- the fleet's 8 and 7 that is cos(tilt) >= 0.875, i.e. within ~29 deg of level. Past that the
-- RETRO bank lights up and burns against the scramble: at 60 deg it is up 11 / down 3, and
-- the net across the pair (up - down = 2*steps) is barely half of what the detent asked for.
-- And a scramble is precisely the command reached for when the craft is NOT level -- a dive
-- recovery, a terrain escape, a hand-flown go-around.
--
-- So on this flag the OPPOSING bank goes to ALT_MIN and the helping one carries
-- NEUTRAL + |steps|. The attitude routing is deliberately left alone: the SIGN of `steps`
-- still chooses which bank is the helping one, so an inverted craft still scrambles on the
-- down bank, and at level the pair is 15/0 -- exactly what it already was.
local function sendAlt(steps, cut, openThrottle)
    local up, down
    if cut then
        up, down = 0, 0
    elseif openThrottle then
        local burn = clamp(CFG.ALT_NEUTRAL + math.abs(steps), CFG.ALT_MIN, CFG.ALT_MAX) / RS_MAX
        local off  = CFG.ALT_MIN / RS_MAX
        if steps < 0 then up, down = off, burn else up, down = burn, off end
    else
        up   = clamp(CFG.ALT_NEUTRAL + steps, CFG.ALT_MIN, CFG.ALT_MAX) / RS_MAX
        down = clamp(CFG.ALT_NEUTRAL - steps, CFG.ALT_MIN, CFG.ALT_MAX) / RS_MAX
    end
    rednet.broadcast({ up = up, down = down, cut = cut or false }, CFG.ALT_PROTOCOL)
    outAlt = { up = up, down = down }
    return up, down
end

-- TRIANGULAR (VECTOR-NOZZLE) THRUST ALLOCATION -- CFG.TRI_ENABLE ---------------
-- Scout (#72), and any airframe built the same way. Instead of a separate thruster bank per
-- axis, the whole craft hangs on THREE down-firing engines on vectoring mounts, placed as a
-- triangle: front-centre, back-left, back-right. Each engine's control computer takes three
-- numbers -- gimbal X (-15..15), gimbal Y (-15..15), thrust (0..15) -- so what the UAC has
-- to send is not five axis commands but three FORCE VECTORS. Pitch, roll, yaw, strafe and
-- lift all come out of those three nozzles; only fore/aft still has a bank of its own
-- (CFG.NAVCTRL_PROTOCOL, #79), which is why sendNav is still called on this airframe.
--
-- WHY THIS LIVES IN THE UAC AND NOT IN A RELAY. Every other output on this fleet is one axis
-- translated into redstone by a relay that needs to know nothing else. This is not that: every
-- axis lands on every engine, and how a demand splits depends on what the OTHER axes are
-- already spending (an engine held at 15 for lift cannot also make yaw). A relay sees one
-- channel. Only the controller sees the whole wrench, so only the controller can divide it.
--
-- THE MODEL. The body frame is the one the attitude loop already works in: x = starboard
-- (pitch axis), y = up (yaw axis), z = nose (roll axis). Engine i sits at r_i = (x, y, z)
-- measured FROM THE CENTRE OF MASS and makes force f_i = (fx, fy, fz):
--
--     F = sum f_i                        M = sum r_i x f_i
--     M_x (pitch) = sum( y_i*fz_i - z_i*fy_i )
--     M_y (yaw)   = sum( z_i*fx_i - x_i*fz_i )
--     M_z (roll)  = sum( x_i*fy_i - y_i*fx_i )
--
-- Note what the yaw line does NOT contain: y_i. YAW IS MADE ENTIRELY BY THE HORIZONTAL
-- components and lift/pitch/roll entirely by the vertical ones -- except for the two y_i
-- terms, which are the pitch/roll moment a sideways push makes about a centre of mass that
-- is not level with the mounts. That splits the problem into two solves run in a fixed order:
--
--   1. HORIZONTAL -- 6 unknowns (fx, fz), 3 equations (strafe, the fore/aft share, yaw).
--      Three spare degrees of freedom, resolved as the least-effort answer WEIGHTED by how
--      much thrust each engine is already carrying: an engine burning at 12 lends more
--      sideways push at the same nozzle angle than one at 3, so it should be asked for more.
--      The weights choose BETWEEN exact answers and never change the wrench delivered, which
--      is exactly why last tick's thrusts are a good enough weight and no inner iteration is
--      needed here.
--   2. VERTICAL -- 3 unknowns (fy), 3 equations (lift, pitch, roll): exactly determined, once
--      the pitch/roll moment step 1 has already committed us to (the y_i terms) is subtracted
--      from the demand.
--
-- Step 2 is LINEAR in its three demands, so its three basis solutions are computed ONCE at
-- boot -- TRI.n (pure lift), TRI.p (pure pitch), TRI.r (pure roll) -- and each tick is three
-- multiply-adds. TRI.p and TRI.r each sum to ZERO by construction: they move lift between
-- engines without changing the total. That is what makes the saturation rule below possible.
--
-- SATURATION: ATTITUDE OUTRANKS LIFT. When the demands will not fit in 0..15 the differential
-- (pitch/roll) is kept and the COMMON lift is slid to make room -- a multirotor that loses
-- lift for a moment sags, and one that loses pitch/roll authority turns over. Only when no
-- common lift can fit the differential at all is the differential itself scaled, and then by
-- the exact largest factor that admits a feasible lift, not by bisection.
--
-- The nozzles have a finite deflection cone (CFG.TRI_VEC_MAX_ANGLE), so each engine's
-- horizontal share is finally limited against its own vertical share (tan(cone) * fy) and
-- what will not fit is DROPPED rather than commanded and quietly not delivered. The attitude
-- loop then sees the shortfall as a disturbance and answers it on the next tick, which is the
-- correct behaviour and the reason this code does not try to be cleverer about it.
--
-- WHAT GOES ON THE WIRE IS A FLOAT, AND IS NOT ROUNDED HERE. The three engine computers
-- drive their thrusters through an output allocation that is theirs to choose -- more than
-- one line per axis, peripheral-side relays, whatever the airframe turned out to need -- so
-- the grid the demand must be quantised ONTO is only known at the far end. Rounding here
-- would throw away resolution the receiver may well have, and it could never be got back.
-- So the sigma-delta lives in the engine computers (77/78/80 vecengine.lua) and this file
-- sends the demand as the allocation computed it.
--
-- That was once the exception; since 2026-08-26 it is the rule, and sendStab/sendNav/sendAlt
-- work the same way (see the OUTPUT header). This path differs only in its UNITS: the axis
-- outputs are normalised to [-1, 1] because a relay knows nothing of engine geometry,
-- whereas these stay in engine-units (0..CFG.TRI_T_MAX, +/-CFG.TRI_VEC_MAX) because the
-- allocation and the engine computer already agree on what an engine-unit is.
--
-- CALIBRATION IS EMPIRICAL, like everything else in this file. Do the TRI_*_SIGN knobs first,
-- one axis at a time on the ground, watching WHICH ENGINE SPOOLS UP; then the *_PER_STEP
-- authorities. See the CFG.TRI_ block in uacconf.lua for the order and what each knob means.
local TRI = { eng = {}, n = {}, p = {}, r = {}, a = {}, w = {}, fx = {}, fy = {}, fz = {},
              out = {}, sat = 1, cone = 0, hot = 0, tot = 0, cut = false, tanMax = 1 }
if CFG.TRI_ENABLE then
    -- Geometry -> the vertical channel's three basis solutions, ONCE. The working locals are
    -- inside this block so they cost the main chunk no registers (Cobalt counts the main
    -- chunk's live locals against every nested function -- see the CFG loader's note).
    local E = CFG.TRI_ENGINES
    if type(E) ~= "table" or #E ~= 3 then
        error("CFG.TRI_ENGINES must list exactly three engines (got "
            .. (type(E) == "table" and #E or type(E)) .. ").", 0)
    end
    for i = 1, 3 do
        local e = E[i]
        if type(e) ~= "table" or type(e.key) ~= "string"
            or type(e.x) ~= "number" or type(e.z) ~= "number" then
            error("CFG.TRI_ENGINES[" .. i .. "] needs key (string) plus x/z (numbers, metres "
                .. "from the centre of mass).", 0)
        end
        TRI.eng[i] = { key = e.key, name = e.name or e.key, x = e.x, y = e.y or 0, z = e.z }
        TRI.w[i], TRI.a[i] = CFG.TRI_HOVER, 0
        TRI.fx[i], TRI.fy[i], TRI.fz[i] = 0, 0, 0
        TRI.out[i] = { x = 0, y = 0, thrust = 0 }
    end
    TRI.tanMax = math.tan(CFG.TRI_VEC_MAX_ANGLE)
    -- A * fy = (lift, pitch, roll), i.e. row 1 = sum fy, row 2 = -sum z*fy, row 3 = sum x*fy.
    -- Gaussian elimination with partial pivoting rather than a hand-derived adjugate: it runs
    -- once at boot, so the cost is nothing and the derivation cannot be got wrong.
    local A = { {  1, 1, 1 },
                { -TRI.eng[1].z, -TRI.eng[2].z, -TRI.eng[3].z },
                {  TRI.eng[1].x,  TRI.eng[2].x,  TRI.eng[3].x } }
    local function solve3(b1, b2, b3)
        local m = { { A[1][1], A[1][2], A[1][3], b1 },
                    { A[2][1], A[2][2], A[2][3], b2 },
                    { A[3][1], A[3][2], A[3][3], b3 } }
        for c = 1, 3 do
            local piv, best = c, math.abs(m[c][c])
            for k = c + 1, 3 do
                if math.abs(m[k][c]) > best then piv, best = k, math.abs(m[k][c]) end
            end
            if best < 1e-9 then return nil end
            m[c], m[piv] = m[piv], m[c]
            for k = 1, 3 do
                if k ~= c then
                    local f = m[k][c] / m[c][c]
                    for cc = c, 4 do m[k][cc] = m[k][cc] - f * m[c][cc] end
                end
            end
        end
        return m[1][4] / m[1][1], m[2][4] / m[2][2], m[3][4] / m[3][3]
    end
    TRI.n = { solve3(1, 0, 0) }
    if TRI.n[1] == nil then
        error("CFG.TRI_ENGINES is degenerate -- the three mounts lie on one straight line "
            .. "through the centre of mass, so lift, pitch and roll cannot be told apart. "
            .. "Check the x/z figures: they are metres FROM THE CoM, not from the hull.", 0)
    end
    TRI.p, TRI.r = { solve3(0, 1, 0) }, { solve3(0, 0, 1) }
    -- Every engine must PUSH to hold a level hover. A non-positive share here means the CoM
    -- lies outside (or on an edge of) the triangle the mounts make, and no set of down-firing
    -- thrusts can balance the craft -- worth refusing at boot, because in the air it presents
    -- as an unkillable pitch or roll bias that no gain will tune out.
    for i = 1, 3 do
        if TRI.n[i] <= 0 then
            error("CFG.TRI_ENGINES: engine '" .. TRI.eng[i].name .. "' would have to pull DOWN "
                .. "(share " .. string.format("%.3f", TRI.n[i]) .. ") to hold a level hover -- "
                .. "the centre of mass is not inside the triangle of mounts. Re-measure x/z "
                .. "from the CoM and check their signs (x = starboard, z = nose).", 0)
        end
    end
end

-- One tick of the allocation. Takes the SAME step commands every other output takes -- so a
-- sign or gain tuned on the axis-thruster path means the same thing here -- and turns them
-- into three (gimbal X, gimbal Y, thrust) triplets on CFG.TRI_PROTOCOL. `cut` is the lift
-- engine switched off (CFG.AL_GROUND_CUT / touchdown): all three engines to a flat zero,
-- nozzles centred, dither accumulators BYPASSED rather than fed, so nothing is owed when the
-- engines come back (sendAlt's note applies verbatim).
local function sendVector(pitch, roll, yaw, strafe, fwd, altSteps, cut)
    local E = TRI.eng
    -- ---- the demanded wrench, in engine-units (1 unit = one redstone step on one engine) ----
    local Fx = CFG.TRI_LAT_SIGN   * strafe * CFG.TRI_LAT_PER_STEP
    local Fz = CFG.TRI_FWD_SIGN   * fwd    * CFG.TRI_FWD_PER_STEP
    local My = CFG.TRI_YAW_SIGN   * yaw    * CFG.TRI_YAW_PER_STEP
    local Mx = CFG.TRI_PITCH_SIGN * pitch  * CFG.TRI_PITCH_PER_STEP
    local Mz = CFG.TRI_ROLL_SIGN  * roll   * CFG.TRI_ROLL_PER_STEP
    local L  = 3 * CFG.TRI_HOVER  + altSteps * CFG.TRI_LIFT_PER_ALTSTEP
    if cut then Fx, Fz, My, Mx, Mz, L = 0, 0, 0, 0, 0, 0 end

    -- ---- 1. HORIZONTAL: least-effort (fx, fz) hitting strafe, fore/aft share and yaw ----
    -- Lagrange solution of min sum |f_h,i|^2 / w_i subject to the three constraints:
    --   fx_i = w_i*(L1 + L3*z_i)      fz_i = w_i*(L2 - L3*x_i)
    -- with the multipliers eliminated by hand down to one division. Sw* are the weighted
    -- moments of the mount positions; D is the weighted second moment about the weighted
    -- centroid, so it is positive for any triangle that is not a line (already refused above).
    local Sw, Swx, Swz, Sq = 0, 0, 0, 0
    for i = 1, 3 do
        local w = math.max(TRI.w[i], CFG.TRI_W_FLOOR)
        TRI.w[i] = w
        Sw, Swx, Swz = Sw + w, Swx + w * E[i].x, Swz + w * E[i].z
        Sq = Sq + w * (E[i].x * E[i].x + E[i].z * E[i].z)
    end
    local D  = Sq - (Swz * Swz + Swx * Swx) / Sw
    local L3 = (D > 1e-6) and ((My - (Swz * Fx - Swx * Fz) / Sw) / D) or 0
    local L1, L2 = (Fx - L3 * Swz) / Sw, (Fz + L3 * Swx) / Sw
    -- ...and the pitch/roll moment that sideways push has ALREADY committed us to, through
    -- the height of the mounts above/below the CoM. Zero when e.y is 0 (mounts level with the
    -- CoM), which is why leaving e.y unset is a safe default rather than an approximation.
    local dMx, dMz = 0, 0
    for i = 1, 3 do
        TRI.fx[i] = TRI.w[i] * (L1 + L3 * E[i].z)
        TRI.fz[i] = TRI.w[i] * (L2 - L3 * E[i].x)
        dMx = dMx + E[i].y * TRI.fz[i]
        dMz = dMz - E[i].y * TRI.fx[i]
    end

    -- ---- 2. VERTICAL: the pure differential first, then the common lift under it ----
    -- TRI.a is the attitude part and sums to zero (see TRI.p/TRI.r above): it moves lift
    -- between engines and adds none. TRI.n * L is the lift part and makes no moment. So the
    -- band each engine has left for lift is (lo - a_i)/n_i .. (hi - a_i)/n_i, and a common
    -- lift exists at all only while, for every ordered pair (i, j),
    --     s*(n_i*a_j - n_j*a_i) <= n_i*hi - n_j*lo.
    -- Take the tightest of those six as the attitude scale rather than bisecting for it --
    -- this is on the 20 Hz path, and a scale that is exact never chatters.
    local lo, hi, s = CFG.TRI_T_MIN, CFG.TRI_T_MAX, 1
    for i = 1, 3 do TRI.a[i] = (Mx - dMx) * TRI.p[i] + (Mz - dMz) * TRI.r[i] end
    for i = 1, 3 do
        for j = 1, 3 do
            local c = TRI.n[i] * TRI.a[j] - TRI.n[j] * TRI.a[i]
            if c > 1e-9 then s = math.min(s, (TRI.n[i] * hi - TRI.n[j] * lo) / c) end
        end
    end
    s = clamp(s, 0, 1)
    local Lmin, Lmax = -math.huge, math.huge
    for i = 1, 3 do
        TRI.a[i] = TRI.a[i] * s
        Lmin = math.max(Lmin, (lo - TRI.a[i]) / TRI.n[i])
        Lmax = math.min(Lmax, (hi - TRI.a[i]) / TRI.n[i])
    end
    local Lu = clamp(L, Lmin, math.max(Lmin, Lmax))
    for i = 1, 3 do TRI.fy[i] = clamp(Lu * TRI.n[i] + TRI.a[i], lo, hi) end

    -- ---- 3. cone limit, magnitude, nozzle angles, dither, wire ----
    local msg = { cut = cut or false }
    TRI.cone, TRI.hot, TRI.tot, TRI.cut, TRI.sat = 0, 0, 0, cut or false, s
    for i = 1, 3 do
        local fy, fx, fz = TRI.fy[i], TRI.fx[i], TRI.fz[i]
        -- Deflection cone: the nozzle cannot make more sideways force than tan(cone) times
        -- what it is making downwards. Scale fx and fz TOGETHER so the direction of the push
        -- survives and only its size is cut.
        local h, hmax = math.sqrt(fx * fx + fz * fz), fy * TRI.tanMax
        if h > hmax then
            local k = (h > 1e-9) and (hmax / h) or 0
            fx, fz = fx * k, fz * k
            TRI.cone = TRI.cone + 1
        end
        local T = math.sqrt(fx * fx + fy * fy + fz * fz)
        if T > hi then                       -- a tilted engine's MAGNITUDE can exceed the
            local k = hi / T                 -- ceiling even with fy inside it
            fx, fy, fz, T = fx * k, fy * k, fz * k, hi
            TRI.hot = TRI.hot + 1
        end
        TRI.fx[i], TRI.fy[i], TRI.fz[i] = fx, fy, fz
        -- Nozzle angles: the tilt of the thrust line away from straight down, resolved onto
        -- the two gimbal axes and measured AGAINST THE VERTICAL COMPONENT -- so an engine
        -- asked for a big sideways push while barely burning reads the large angle it really
        -- needs, instead of a small one scaled off full thrust. "tan" law: the command is
        -- proportional to the force RATIO instead of to the angle; which of the two the
        -- hardware wants is a calibration, see CFG.TRI_VEC_LAW.
        local gx, gz
        if CFG.TRI_VEC_LAW == "tan" then
            gx = ((fy > 1e-6) and (fx / fy) or 0) / TRI.tanMax * CFG.TRI_VEC_MAX
            gz = ((fy > 1e-6) and (fz / fy) or 0) / TRI.tanMax * CFG.TRI_VEC_MAX
        else
            gx = atan2(fx, fy) / CFG.TRI_VEC_MAX_ANGLE * CFG.TRI_VEC_MAX
            gz = atan2(fz, fy) / CFG.TRI_VEC_MAX_ANGLE * CFG.TRI_VEC_MAX
        end
        local cx, cy = CFG.TRI_VEC_X_SIGN * gx, CFG.TRI_VEC_Y_SIGN * gz
        if CFG.TRI_VEC_SWAP then cx, cy = cy, cx end
        -- CLAMPED but NOT ROUNDED (see the header). The band still has to be enforced here --
        -- a gimbal command outside +/-TRI_VEC_MAX is not a degraded flight, it is a number the
        -- engine computer has no legal output for -- but the fraction inside the band is the
        -- receiver's to spend, so it travels intact.
        local ox, oy, ot
        if cut then
            ox, oy, ot = 0, 0, 0
        else
            ox = clamp(cx, -CFG.TRI_VEC_MAX, CFG.TRI_VEC_MAX)
            oy = clamp(cy, -CFG.TRI_VEC_MAX, CFG.TRI_VEC_MAX)
            ot = clamp(T, 0, hi)
        end
        TRI.out[i].x, TRI.out[i].y, TRI.out[i].thrust = ox, oy, ot
        msg[E[i].key] = { x = ox, y = oy, thrust = ot }
        TRI.tot = TRI.tot + ot
        TRI.w[i] = fy                        -- weight for next tick's horizontal split
    end
    rednet.broadcast(msg, CFG.TRI_PROTOCOL)
end

-- FLIGHT DATA RECORDER (CFG.LOG_ENABLE) ---------------------------------------
-- Writes one CSV row per control tick to CFG.LOG_FILE, so a fault can be read off a
-- time-aligned record instead of off a 20 Hz screen. The screen shows the same numbers, but
-- not simultaneously and not at a rate anyone can transcribe -- and the question this exists
-- to answer ("does the kick appear first in the COMMAND, in the LIFT, or only in the ENGINE
-- OUTPUTS?") can only be answered by columns that share a timestamp.
--
-- It waits CFG.LOG_DELAY seconds after boot before opening the file, so the first moments --
-- peripherals attaching, the first nav/altitude packets arriving, the trim integral starting
-- from zero -- do not fill the record with a transient nobody asked about. It then records for
-- CFG.LOG_SECONDS and STOPS, closing the file, because an unbounded log on a computer with a
-- fixed disk quota eventually fails a write mid-flight and takes the controller with it.
--
-- Everything here is off unless CFG.LOG_ENABLE is set, and no other airframe's uacconf has it.
local LOG = { state = "off", t0 = nil, tRec = nil, n = 0, fh = nil, skip = 0, why = nil }
-- scram and velInt are APPENDED, not inserted, so every column index a previous record
-- was read with still points at the same field.
--   scram   ui.scramble as the CONTROLLER saw it, not as the stick sent it. This is the
--           column that separates "deaf" from "working against it": the MANCON's own panel
--           can show SCRM while this reads 0, and then the fault is between the two -- a
--           protocol name, a stale packet, or a gate in the scramble expression. If it reads
--           1 and the aeroplane does not climb, the flag arrived and the fault is downstream.
--   velInt  the vertical PID's integral -- the HOVER TRIM the loop has learnt, in alt steps.
--           This is what says whether a scramble had anything left to give: the detent
--           commands ALT_SWING, so what it ADDS over the hover is (ALT_SWING - velInt). A
--           hull sitting at velInt = 7 out of a swing of 8 is already at 14 of its 15 line
--           levels to stay up, and no software can make the detent worth more than one step.
LOG.HEAD = "t,pitch,roll,wx,wz,errP,errR,rotRaw,pStep,rStep,yStep,integP,integR,"
    .. "posY,velY,desVelY,vertEff,altSteps,tiltUp,thrCmd,strCmd,"
    .. "L,sat,cone,hot,fcx,fcy,fct,blx,bly,blt,brx,bry,brt,idle,apN,apA,phase,"
    .. "scram,velInt,"
    -- navCap, appended 2026-09-01 for "something is still making thrust in the align phase",
    -- and appended for the same reason the yaw and scramble blocks were: every earlier
    -- uaclog.csv still parses. It is the FMS's SCHEDULED SPEED as the controller received it
    -- (msg.cruiseCap when msg.schedSpeed is set, else -1 for "the FMS is not scheduling and
    -- this law draws its own"). It is the column that settles an align-phase argument without
    -- anybody having to watch the HSI: read it beside thrCmd and phase.
    --   navCap 0, thrCmd 0            the hold is being honoured -- this is the fix landing.
    --   navCap 0, thrCmd NONZERO      something downstream of the schedule is still spending
    --                                 authority. That was the breakaway floor (see the
    --                                 commanded-zero note in the velocity-vector law) and
    --                                 before it the ENROUTE speed integral; a third would be
    --                                 a new fault, so log which phase it appears in.
    --   navCap -1 in ENROUTE          the FMS is not scheduling at all -- DC.DYNAMIC_CRUISE
    --                                 is off on that airframe, and no cap of ANY value is
    --                                 reaching this controller. Fix that before reading on.
    .. "navCap,"
    -- ARRIVAL BLOCK, appended 2026-09-02 for "DIRECT TO does not slow down and circles the
    -- point". All three are already on `ui` for the status screen; the recorder simply could
    -- not see them, which is why the arrival profile has never once been checked against what
    -- the airframe actually does. Appended at the end, as every block here is.
    --   gs       horizontal ground speed. Its slope IS the achieved deceleration -- take
    --            d(gs)/dt over the braking run and compare it with DC.ARRIVE_DECEL in apconf,
    --            which is a CLAIM about the airframe's stopping power that nothing has ever
    --            measured. If the slope is shallower, the arrival sweep is allocating less
    --            distance than the craft needs and it will always arrive hot: lower the knob
    --            to the measured figure. On Scout expect it to be: the aft bank is wired
    --            across 15 units against forward's 45 (see 86/thrustcon.lua).
    --   closing  velocity component TOWARD the target. The overshoot signature is closing
    --            going NEGATIVE while gs stays high -- that is the craft flying away from a
    --            fix it has passed, and in the ENROUTE law it is what commanded full forward
    --            thrust to chase it round. In APPROACH/TERMINAL the velocity-vector law nulls
    --            the tangential component instead, so closing should come back through zero
    --            once and stay there.
    --   dist     range to the target the controller is steering to, for the x-axis of both.
    .. "gs,closing,dist"

-- Called at the END of every control tick, once ui is published and the commands are final.
-- Takes the four values that are controlLoop locals (and so are not on ui) as arguments
-- rather than having controlLoop publish them: this file is already close to Cobalt's
-- 200-locals ceiling and ui lookups in the hot path are not free either.
function LOG.tick(now, pStep, rStep, yStep, altSteps)
    if not CFG.LOG_ENABLE or LOG.state == "done" or LOG.state == "fail" then return end
    LOG.t0 = LOG.t0 or now
    if now - LOG.t0 < CFG.LOG_DELAY then
        LOG.state, LOG.wait = "wait", CFG.LOG_DELAY - (now - LOG.t0)
        return
    end
    if LOG.state ~= "rec" then                      -- first tick past the delay: open the file
        local ok, h = pcall(fs.open, CFG.LOG_FILE, "w")
        if not ok or not h then
            LOG.state, LOG.why = "fail", "cannot open " .. tostring(CFG.LOG_FILE)
            return
        end
        LOG.fh, LOG.state, LOG.tRec = h, "rec", now
        h.writeLine(LOG.HEAD)
    end
    -- Decimate BEFORE the row is built, not after: string.format on 38 values every tick is
    -- the expensive part, and CFG.LOG_EVERY exists to make it cheaper on a busy computer.
    LOG.skip = LOG.skip + 1
    if LOG.skip < CFG.LOG_EVERY then return end
    LOG.skip = 0
    local o1, o2, o3 = TRI.out[1], TRI.out[2], TRI.out[3]
    local okw = pcall(LOG.fh.writeLine, string.format(
        "%.2f,%.3f,%.3f,%.4f,%.4f,%.4f,%.4f,%.0f,%.3f,%.3f,%.3f,%.4f,%.4f,"
        .. "%.2f,%.3f,%.3f,%.3f,%.3f,%.3f,%.3f,%.3f,"
        .. "%.2f,%.3f,%d,%d,%.2f,%.2f,%.2f,%.2f,%.2f,%.2f,%.2f,%.2f,%.2f,%d,%d,%d,%s,%d,%.3f,%.1f,%.2f,%.2f,%.1f",
        now - LOG.tRec,
        math.deg(ui.pitch or 0), math.deg(ui.roll or 0), ui.wx or 0, ui.wz or 0,
        ui.errPitch or 0, ui.errRoll or 0, ui.rotRaw or 0,
        pStep or 0, rStep or 0, yStep or 0, ui.integP or 0, ui.integR or 0,
        ui.posY or 0, ui.velY or 0, ui.desVelY or 0, ui.vertEff or 0, altSteps or 0,
        ui.tiltUp or 1, ui.thr or 0, ui.str or 0,
        TRI.tot or 0, TRI.sat or 1, TRI.cone or 0, TRI.hot or 0,
        o1 and o1.x or 0, o1 and o1.y or 0, o1 and o1.thrust or 0,
        o2 and o2.x or 0, o2 and o2.y or 0, o2 and o2.thrust or 0,
        o3 and o3.x or 0, o3 and o3.y or 0, o3 and o3.thrust or 0,
        ui.idle and 1 or 0, ui.apNav and 1 or 0, ui.apAlt and 1 or 0,
        (tostring(ui.phase or "-"):gsub(",", ";")),
        ui.scramble and 1 or 0, ui.velInt or 0, ui.navCap or -1,
        ui.gs or 0, ui.closing or 0, ui.dist or 0))
    if not okw then
        -- A failed write is almost always the disk quota. Close what we have rather than
        -- retrying every tick: a partial log is still readable, and a controller that spends
        -- its tick budget on failing writes is a worse problem than a short record.
        LOG.state, LOG.why = "fail", "write failed at row " .. LOG.n .. " (disk full?)"
        pcall(LOG.fh.close)
        return
    end
    LOG.n = LOG.n + 1
    LOG.pct = (now - LOG.tRec) / CFG.LOG_SECONDS
    if now - LOG.tRec >= CFG.LOG_SECONDS then
        pcall(LOG.fh.close)
        LOG.state = "done"
    end
end

-- ATTITUDE --------------------------------------------------------------------
-- Returns pitchSteps, rollSteps, yawSteps (signed). Everything is computed in the
-- BODY frame -- the frame the per-axis thrusters live in: the attitude error is
-- q_cur^-1 * q_des directly, the angular velocity is rotated in from the sensor
-- frame if needed, and the inertia tensor is applied via inertiaMul (body vector ->
-- body vector, whatever frame the tensor itself is in). There is no final command
-- rotation, so a pure pitch demand stays a pure pitch command. manYawRate/bankYawRate
-- (either nil) both name a yaw RATE directly and skip the heading-error motion profile --
-- manYawRate for the hand-flown MIC stick, bankYawRate for the HYBRID coordinated-turn
-- sim AND the hand-flown keyboard joystick (see their CFG blocks); manYawRate takes
-- priority when both are live. pitchCmdSign (nil = use CFG.PITCH_ALT_SIGN) lets a caller
-- override the pitch overlay's sign -- needed because PITCH_ALT_SIGN is 0 (climb-pitch
-- overlay off), which would otherwise multiply the joystick's pitch by zero. See
-- CFG.KJOY_PITCH_SIGN.
-- q_inv IS PASSED IN, not recomputed (2026-08-28). controlLoop already needs it for the
-- attitude-aware vertical allocation, so the shipped build called q_cur:inverse() twice a
-- tick for the same value. The tensor argument is gone with it: it is slow-polled into SLOW
-- now (see unpackTensor), so there is nothing left for a caller to hand over.
local function attitude(q_cur, q_inv, w, heading, desiredHeading, dt, bankCmd, pitchCmd, idle, yawBrake, manYawRate, bankYawRate, pitchCmdSign)
    bankCmd  = bankCmd  or 0   -- commanded roll  overlay (rad), HYBRID/FIXEDWING; 0 = level
    pitchCmd = pitchCmd or 0   -- commanded pitch overlay (rad), HYBRID/FIXEDWING; 0 = level
    -- idle: craft parked/engine-off (see the IDLE block). Freezes + bleeds the trim integral.

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
        --
        -- q_des IS BUILT HERE, INSIDE THIS BRANCH (2026-08-28). It used to be built at the top
        -- of the function, unconditionally -- but the DECOMPOSED path above never reads it: it
        -- derives the tilt error from the up vectors and the yaw error from wrapPi, and touches
        -- no desired quaternion at all. Every airframe on the fleet sets YAW_TILT_DECOMP, so
        -- that was one quaternion and one vector constructed and thrown away, every tick,
        -- on every aircraft.
        -- Desired level attitude facing desiredHeading. Invert navcom's heading chain
        -- so the yaw we build lives in the raw-quaternion frame q_cur reads back in.
        local q_des = makeYawQuat((desiredHeading - CFG.HEADING_OFFSET) / CFG.HEADING_SIGN)
        local eq = q_inv * q_des
        local ea, vx, vy, vz = eq.a, eq.v.x, eq.v.y, eq.v.z
        if ea < 0 then ea, vx, vy, vz = -ea, -vx, -vy, -vz end
        local vlen = math.sqrt(vx*vx + vy*vy + vz*vz)
        local ang  = 2 * atan2(vlen, ea)                     -- shortest angle, [0, pi]
        local es   = (vlen > 1e-9) and (ang / vlen) or 2.0
        ex, ey, ez = vx*es, vy*es, vz*es
    end

    -- COMMANDED-ATTITUDE OVERLAY (HYBRID/FIXEDWING). Shift the pitch/roll SETPOINT
    -- off level by the coordinated-turn bank and the climb pitch. ex/ez are the
    -- drive-to-zero tilt errors, so SUBTRACTING the command moves the equilibrium to
    -- that attitude, and everything downstream -- the integral, the P/D law, and the
    -- tilt/yaw level-priority (which reads sqrt(ex^2+ez^2)) -- then works relative to
    -- the COMMANDED attitude. So a deliberate bank is NOT mistaken for an upset and
    -- does not fade the yaw during a coordinated turn, while a genuine flip (error
    -- far from the small commanded bank) still fades it. Both commands are zero in
    -- VTOL and in straight-and-level flight, making this an exact no-op there.
    -- Publish the MEASURED pitch attitude in the COMMAND's own convention, before the command is
    -- subtracted out. This is the one comparison that is sign-safe without any new calibration:
    -- at equilibrium the loop drives the error to zero, so ex must end up EQUAL to the applied
    -- command below. Displaying the two side by side therefore reads directly:
    --   they converge          -> the attitude loop is tracking; any fault is upstream
    --   equal and OPPOSITE     -> the pitch command's sign is inverted (CFG.FW_PITCH_SIGN)
    --   command large, measured stuck short -> not enough authority, or something else is
    --                                          holding the attitude (mass trim, attIntegX)
    ui.pitchMeas = ex
    ui.pitchApplied = (pitchCmdSign or CFG.PITCH_ALT_SIGN) * pitchCmd
    ex = ex - (pitchCmdSign or CFG.PITCH_ALT_SIGN) * pitchCmd
    ez = ez - CFG.BANK_SIGN                        * bankCmd

    -- Angular velocity in the BODY frame (rotate the world reading in if needed),
    -- then the per-axis rate-feedback sign (independent of the output *_SIGN).
    local wx, wy, wz
    if CFG.SENSOR_FRAME == "world" then wx, wy, wz = rotateVec(q_inv, w.x, w.y, w.z)
    else                            wx, wy, wz = w.x, w.y, w.z end
    wx, wy, wz = wx * CFG.PITCH_RATE_SIGN, wy * CFG.YAW_RATE_SIGN, wz * CFG.ROLL_RATE_SIGN

    -- Integral trim (body frame), rate-gated so it can't pump a manoeuvre (integStep).
    -- PITCH always. YAW and ROLL are OPT-IN and default OFF, which is what the two commented
    -- -out lines they replace amounted to -- absent knob = nil = false = the behaviour every
    -- airframe on this fleet has had. The reason they were off is not recorded anywhere, so
    -- they are made reachable rather than simply switched on.
    --
    -- WHY ROLL WANTS IT ON A MULTIROTOR (Scout, 2026-08-22): with no integral the axis can
    -- only answer a standing moment with a standing ERROR, and on a craft whose lift vector
    -- IS its attitude, a standing roll error is a standing sideways acceleration that nothing
    -- ever cancels. Measured off 201 samples of hover: errR held +0.0125 rad (+0.72 deg) with
    -- integR flat at exactly 0.0000, which is 11*sin(0.72deg) = 0.14 m/s^2 of permanent
    -- sideways push -- the aircraft crept, and no gain could have stopped it. Pitch, which
    -- does have the integral, sat at errP +0.0001 with integP holding a steady 0.0177 on the
    -- same flight: the mechanism works, roll just was not allowed to use it.
    if CFG.ATT_INTEGRAL then
        attIntegX = integStep(attIntegX, ex, wx, dt, idle)
        if CFG.ATT_INTEGRAL_YAW  then attIntegY = integStep(attIntegY, ey, wy, dt, idle) end
        if CFG.ATT_INTEGRAL_ROLL then attIntegZ = integStep(attIntegZ, ez, wz, dt, idle) end
    end
    local biasX = CFG.ATT_INTEGRAL and attIntegX or 0
    local biasY = CFG.ATT_INTEGRAL and attIntegY or 0
    local biasZ = CFG.ATT_INTEGRAL and attIntegZ or 0

    -- SMITH PREDICTION on the pitch rate only: add the acceleration already commanded within the
    -- last tau_d but not yet felt, so the rate loop sees where the aeroplane WILL be rather than
    -- where it was a dead time ago. smithSum is a running sum of buffered accels (rad/s^2), so
    -- smithSum*dt is the rate change still in the pipeline. Zero-length buffer = exact no-op.
    local wxUse = wx
    if smithN > 0 then wxUse = wx + smithSum * dt end
    ui.smith = (smithN > 0) and (wxUse - wx) or nil

    local px, py, pz   -- body-frame command (pitch, yaw, roll), pre-sign/clamp
    if CFG.ATT_MODE == "INERTIA" then
        -- I * (body vector) -> body vector, from whatever frame the tensor is in.
        -- Reads the SLOW-polled tensor; see unpackTensor for why it is not fetched here.
        local function inertiaMul(a, b, c)
            if CFG.SENSOR_FRAME == "world" then
                local wa, wb, wc = rotateVec(q_cur, a, b, c)   -- body -> world
                local ia, ib, ic = matVec(wa, wb, wc)          -- world-frame tensor
                return rotateVec(q_inv, ia, ib, ic)            -- world -> body
            end
            return matVec(a, b, c)
        end

        -- Desired body angular acceleration. Outer loop: a decel-limited motion
        -- profile turns error into a desired RATE that a high-inertia axis can still
        -- brake from (so yaw doesn't overshoot/spin); inner P(D) tracks that rate,
        -- plus the integral trim as an accel bias.
        -- PITCH and ROLL get their OWN outer gains where the config supplies them (nil =>
        -- CFG.ATT_KP, so this is a no-op on any airframe that does not set them). Same hook and
        -- same reason as CFG.ATT_KP_YAW below and #19's CFG.ATT_KP_PITCH: ATT_KP sets the
        -- cascade's DAMPING RATIO, zeta = 0.5*sqrt(G/ATT_KP), and the three axes hang off three
        -- different thruster sets with three different G. One shared number cannot damp them all.
        local wdx = profileRate(ex, CFG.ATT_KP_PITCH or CFG.ATT_KP, CFG.MAX_RATE_PITCH, CFG.MAX_DECEL_PITCH)
        -- Manual yaw held -> the pilot NAMES the yaw rate; feed it straight to the rate loop and
        -- skip the heading profile (no ramp to chase = no counter-steer overshoot). Else profile.
        -- YAW gets its OWN outer gain where the config supplies one (nil => CFG.ATT_KP, so
        -- this is a no-op on any airframe that does not set it). Same hook, same reason, as
        -- #19's CFG.ATT_KP_PITCH: ATT_KP sets the cascade's DAMPING RATIO
        -- (zeta = 0.5*sqrt(G/ATT_KP)), and a yaw axis with this much inertia has a completely
        -- different G from pitch and roll, which one shared number cannot express.
        local wdy = manYawRate or bankYawRate
            or profileRate(ey, CFG.ATT_KP_YAW or CFG.ATT_KP, CFG.MAX_RATE_YAW, CFG.MAX_DECEL_YAW)
        local wdz = profileRate(ez, CFG.ATT_KP_ROLL or CFG.ATT_KP, CFG.MAX_RATE_ROLL,  CFG.MAX_DECEL_ROLL)
        local rex, rey, rez = wdx - wxUse, wdy - wy, wdz - wz   -- pitch uses the predicted rate
        local ax = CFG.RATE_KP*rex     + CFG.ATT_KI*biasX
        local ay = CFG.RATE_KP_YAW*rey + CFG.ATT_KI*biasY - (yawBrake or 0)*wy
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
            --
            -- SENSOR_FRAME == "body" (every airframe on the fleet) reads them straight off
            -- the slow poll: the quadratic form on the body unit axes is just the tensor's
            -- own diagonal, it does not depend on the attitude, and unpackTensor has already
            -- taken it. The three inertiaMul calls this replaces were six throwaway tables a
            -- tick to read three numbers that had not changed since the aircraft was built.
            -- The "world" tensor DOES rotate with the attitude, so that path still recomputes
            -- per tick -- correctness first, and no fleet airframe pays for it.
            local Ix, Iy, Iz
            if CFG.SENSOR_FRAME == "world" then
                Ix              = inertiaMul(1, 0, 0)   -- first component = I_body_xx
                local _,  Iy2   = inertiaMul(0, 1, 0)   -- second        = I_body_yy
                local _, _, Iz2 = inertiaMul(0, 0, 1)   -- third         = I_body_zz
                Iy, Iz = Iy2, Iz2
            else
                Ix, Iy, Iz = SLOW.Ix, SLOW.Iy, SLOW.Iz
            end
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
        local yawRateCmd = manYawRate or bankYawRate
        if yawRateCmd then   -- manual, or HYBRID coordinated-turn sim: track the rate, not a heading
            py = CFG.PD_KD_YAW*(yawRateCmd - wy) + CFG.PD_KI*biasY
        else
            py = CFG.PD_KP_YAW  *ey - CFG.PD_KD_YAW  *wy + CFG.PD_KI*biasY - (yawBrake or 0)*wy
        end
        pz = CFG.PD_KP_ROLL *ez - CFG.PD_KD_ROLL *wz + CFG.PD_KI*biasZ
        ui.rotRaw = math.sqrt(px*px + py*py + pz*pz)
    end

    -- Level priority: fade yaw out as pitch+roll error grows, so a yaw manoeuvre can
    -- never roll/pitch the craft over faster than the leveling loop can recover.
    local tilt = math.sqrt(ex*ex + ez*ez)   -- pitch+roll error magnitude (rad), excl. yaw
    local yawScale = clamp((CFG.TILT_YAW_MAX - tilt) / (CFG.TILT_YAW_MAX - CFG.TILT_YAW_MIN), 0, 1)
    -- Manual yaw BRAKE gets full authority: level-priority normally fades yaw as tilt grows, but
    -- nulling a spin REDUCES flip risk (it can't tip the craft over faster than it's already
    -- going), so a brake must not be starved of thrust by the little nutation-tilt a hard yaw
    -- throws off. Keep full scale up to the hard flip guard (TILT_YAW_MAX ~34 deg).
    if (yawBrake or 0) > 0 and tilt < CFG.TILT_YAW_MAX then yawScale = 1 end

    ui.errPitch, ui.errYaw, ui.errRoll = ex, ey, ez
    ui.integP, ui.integR = attIntegX, attIntegZ
    ui.yawScale = yawScale
    -- Yaw first, then feedforward its coupling into pitch/roll (based on the ACTUAL
    -- yaw command, so it fades with yawScale exactly as the real disturbance does).
    local yawSteps   = clamp(CFG.YAW_SIGN * py, -CFG.YAW_MAX, CFG.YAW_MAX) * yawScale
    local pitchSteps = clamp(CFG.PITCH_SIGN * px + CFG.YAW_TO_PITCH_FF * yawSteps, -CFG.STAB_MAX, CFG.STAB_MAX)
    local rollSteps  = clamp(CFG.ROLL_SIGN  * pz + CFG.YAW_TO_ROLL_FF  * yawSteps, -CFG.STAB_MAX, CFG.STAB_MAX)
    -- Feed the predictor the accel this tick's output will ACTUALLY deliver, not the demanded
    -- one: pitchSteps is clamped to STAB_MAX, and buffering an unachievable demand would have
    -- the predictor expect a rate the aeroplane never reaches -- the loop would then back off
    -- while still saturated, which is precisely the windup pattern this file keeps rediscovering.
    -- The saturation ratio is taken against the pre-clamp command, so the accel-per-step
    -- calibration cancels and only ROT_GAIN_PITCH's own consistency matters.
    -- CRITICAL: the buffered accel must come from the OUTPUT STEPS times a MEASURED accel-per-
    -- step, NOT from axCmd. axCmd is the accel the loop DEMANDS, and ROT_GAIN_PITCH is a tuned
    -- LOOP GAIN, not an absolute calibration -- ROT_GAIN_PITCH*Ix = 12.88 steps per rad/s^2
    -- implies 0.078 rad/s^2 per step, while the limit-cycle measurement says the real figure is
    -- ~0.008. Feeding axCmd would therefore have the predictor expect **~10x** the rate the
    -- aeroplane actually develops, which is the one error mode that makes a Smith predictor
    -- worse than no predictor at all. Using steps x CFG.PITCH_SMITH_ACCEL_PER_STEP keeps the
    -- prediction on a physically measurable constant and drops ROT_GAIN_PITCH out entirely.
    -- Sign: pitchSteps is in the OUTPUT convention, so multiply back by CFG.PITCH_SIGN to get
    -- the accel in the loop's internal frame, the same frame wx lives in.
    -- Reading the OUTPUT also makes this correct in PD mode and through saturation for free:
    -- pitchSteps is already clamped to STAB_MAX, so an unachievable DEMAND is never buffered --
    -- which would otherwise have the predictor expect a rate the aeroplane cannot reach and back
    -- the loop off while still saturated, the windup pattern this file keeps rediscovering.
    if smithN > 0 then
        local deliv = CFG.PITCH_SIGN * pitchSteps * CFG.PITCH_SMITH_ACCEL_PER_STEP
        smithSum = smithSum - smithBuf[smithIdx] + deliv
        smithBuf[smithIdx] = deliv
        smithIdx = smithIdx % smithN + 1
    end
    return pitchSteps, rollSteps, yawSteps
end

-- CONTROL LOOP ----------------------------------------------------------------
local function controlLoop()
    local lastTime = os.clock()
    local lastMass = nil
    local trimBoost = 0
    local lastDesiredVelY = 0   -- previous tick's desired climb rate, feeds pitch overlay
    local bankCmd  = 0          -- slewed coordinated-bank command (rad), HYBRID/FIXEDWING
    local lastLatDemand = 0     -- previous tick's body-right course demand, feeds the cross-track bank
    local pitchCmd = 0          -- slewed pitch-for-altitude command (rad), HYBRID/FIXEDWING
    local manHeading = nil      -- integrated manual-yaw heading target (MIC #14); nil until first tick
    local stillFor = 0          -- s the craft has been still-and-unguided, for the idle gate
    local altCascadePrev = false -- edge-detect entry into altitude-hold, to seize current alt

    while true do
        local now = os.clock()
        local dt  = math.max(now - lastTime, 0.001)
        lastTime  = now

        -- THE THREE READS THAT GENUINELY CHANGE AT 20 Hz. Pose, linear velocity and angular
        -- velocity are the loop's sensors and every one of them is fetched every tick.
        -- The inertia tensor and the mass are NOT here any more -- they are properties of the
        -- assembly, not of its motion, and they are slow-polled below. See SLOW/unpackTensor.
        local pose   = sublevel.getLogicalPose()
        local pos    = pose.position
        local q_cur  = pose.orientation
        local q_inv  = q_cur:inverse()   -- for the attitude-aware vertical allocation, and
                                         -- handed to attitude(), which used to take its own
        local vel    = sublevel.getLinearVelocity()
        local w      = sublevel.getAngularVelocity()

        -- ---- slow poll: the assembly's physical properties ----
        -- Two fewer blocking main-thread round trips per tick, out of ten. The measurement
        -- that says round trips (and not arithmetic) are what this loop is spending its time
        -- on is written up at the bottom of the loop, beside the pacing.
        if now >= SLOW.next then
            SLOW.next = now + CFG.SLOW_POLL_PERIOD
            if CFG.ATT_MODE == "INERTIA" then unpackTensor(sublevel.getInertiaTensor()) end
            SLOW.mass = sublevel.getMass()
        end
        local mass = SLOW.mass

        -- ALTITUDE DATUM (2026-08-29). `pos.y` is the ASSEMBLY's reported pose, and that
        -- sits CFG.ALT_OFFSET metres above the bottom of the hull -- 9 m on Aer Togekiss,
        -- which is why it read 73 parked on a Y=63 airstrip. latestPosY is the Y of the
        -- BOTTOM, and from here down it is the only altitude this file is allowed to see:
        -- every altitude the aircraft compares itself against (landing.db's pad and runway
        -- elevations, the terrain survey, an ALT-AP pick) is the world Y of a piece of
        -- GROUND, so the aeroplane's own number has to be the world Y of the part that
        -- touches it. See CFG.ALT_OFFSET in uacconf for how to measure it.
        latestPosY = pos.y - CFG.ALT_OFFSET
        if TARGET_ALT == nil then setTarget(latestPosY) end   -- boot: hold where we are

        -- ---- pilot autopilot mode (from the #12 toggle switches) ----
        -- Attitude/levelling always runs (below). navActive gates navigation; else
        -- positional control is handed to the pilot. useAltCascade runs the altitude-hold
        -- cascade; else the vertical effort comes from the manual VSC (mancon). An
        -- in-progress autoland overrides both -- it owns the craft until it aborts/finishes.
        local autolandActive = (alState ~= "normal")
        local navActive      = AP_NAV or autolandActive
        local useAltCascade  = AP_ALT or autolandActive
        -- Entering altitude-hold from manual: seize the current altitude so it holds here
        -- rather than snapping to a stale target (a live ALT-AP panel pick overrides within 1 s).
        if useAltCascade and not altCascadePrev and latestPosY then setTarget(latestPosY) end
        altCascadePrev = useAltCascade

        -- ONE toEuler PER TICK (2026-08-28). The shipped build called it three times: once
        -- through getYaw for the heading, and twice more in the publish block --
        -- `select(1, euler(q_cur))` and `select(3, euler(q_cur))` -- each of which decomposed
        -- the whole quaternion and then threw two of the three angles away. All three wanted
        -- the same decomposition of the same q_cur. Landed on `ui` rather than three new
        -- locals for the usual Cobalt 200-locals reason, and the publish block downstream now
        -- reads these instead of recomputing them. Same tick, same q_cur, so the display
        -- snapshot stays exactly as coherent as it was.
        ui.pitch, ui.eulerYaw, ui.roll = euler(q_cur)
        local heading = CFG.HEADING_SIGN * ui.eulerYaw + CFG.HEADING_OFFSET

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

        -- APPROACH is deliberately treated like ENROUTE here (cruise speed + along-track +
        -- leg cross-track hold), NOT as a slow position-hold: the FMS emits APPROACH for the
        -- whole penultimate leg and the final leg while still outside the final point's pass
        -- radius, and we want to keep FLYING (and seeking heading) through all that, slowing
        -- and handing off heading only once actually AT the final point (TERMINAL/ARRIVED,
        -- inside its pass radius -- the autoland then rotates the nose via the LANDING phase).
        local translateOnly = (navPhase == "TERMINAL"
            or navPhase == "ARRIVED" or navPhase == "LANDING")
        -- APPROACH also flies the VELOCITY VECTOR toward the fix (via the translateOnly law below),
        -- not the scalar closing speed: the closing loop only sees velocity TOWARD the target, so an
        -- overshoot/orbit (closing goes NEGATIVE while ground speed stays high) is never braked -- it
        -- even commands forward thrust to chase the fix it just passed. Velocity-vector control nulls
        -- the tangential component too. APPROACH still seeks the bearing (desiredHeading, above).
        local approachVec = (navPhase == "APPROACH")
        local headingHold = (navPhase == "LANDING" and type(navHoldHeading) == "number")
        -- ALIGN HOLD, VTOL/HYBRID ONLY (2026-09-04). navCruiseCap is already forced to 0 for
        -- the FMS's route-start ALIGN (msg.align, NAV.aligning -- nose swinging onto a fresh
        -- bearing, most commonly right after cancelling and re-committing to a new destination
        -- mid-flight), but on its own that only zeroes the ENROUTE law's ALONG-TRACK term
        -- (closing, the velocity component TOWARD the target). It says nothing about any OTHER
        -- component of whatever the aircraft was doing a moment ago: that is flown as a
        -- cross-track POSITION hold against navPrev, which right after a fresh commit is the
        -- UAC's own bogus leg from the old target to the new one (see the FMS's isApproachLeg
        -- note) -- not a velocity brake, so the inherited momentum is never actually killed.
        -- A FIXEDWING cannot come to a hover, so flying the ordinary forward-only ENROUTE law
        -- through the hold -- bank onto the new bearing, thrust governed by closing rate -- is
        -- exactly correct for it and must stay untouched (CFG.STRAFE_ENABLED gates it out).
        -- A VTOL/HYBRID CAN hover, and the whole point of the hold is to stop before committing
        -- to the new bearing, so route it through the velocity-vector law instead -- the same
        -- law approachVec/navCarrot use below, which nulls the FULL velocity vector (toward
        -- AND across the bogus leg alike), not just its along-track component.
        local alignStop = navAlign and CFG.STRAFE_ENABLED

        -- Lateral-control policy for HYBRID/FIXEDWING. Pad phases = the close-in, precise
        -- ones where strafe does the pad alignment. bankSteer = "this tick the mode corrects
        -- lateral course by BANKING": HYBRID banks in the non-pad phases and hands lateral
        -- control to strafe (wings-level) in the pad phases; FIXEDWING has no strafe at all so
        -- it banks in EVERY phase. Autoland (alState ~= normal, incl. the LANDING phase) forces
        -- wings-level in BOTH modes for a level pad approach -- excluded here via alState.
        -- ...UNLESS it is a runway glideslope (navGlide), which is an ordinary flying leg that
        -- happens to be descending: there the mode's normal lateral control has to stay on, or
        -- FIXEDWING would be left with no way at all to hold the centreline.
        local padPhase  = (navPhase == "TERMINAL" or navPhase == "ARRIVED" or navPhase == "LANDING")
        local bankSteer = CFG.ATTITUDE_BANKS and (alState == "normal" or navGlide)
            and not (CFG.FLIGHT_MODE == "HYBRID" and padPhase)
        -- EAGER STRAFE (see its CFG block): 1 = the remaining correction is small enough that
        -- translation should fly it and bank should not. Measure whatever the lateral law is
        -- actually nulling this tick -- cross-track on a plain ENROUTE leg, else range to the
        -- target (the carrot keeps that ~400 m in cruise, so cruise banking is unaffected). No
        -- target at all (parked, AP off) => there is no correction => fully eager, i.e. wings
        -- level: this is the "no mini-swaying on the ground" case.
        local latErr = (hasLeg and not (translateOnly or approachVec or navCarrot or alignStop))
            and math.abs(xtk) or (hasTarget and dist or 0)
        local strEager = 1 - clamp((latErr - CFG.STRAFE_EAGER_NEAR)
            / math.max(CFG.STRAFE_EAGER_FAR - CFG.STRAFE_EAGER_NEAR, 1e-6), 0, 1)
        ui.strEager = strEager

        -- FIXEDWING: PITCH *IS* THE VERTICAL CONTROL (see the CFG.FW_PITCH_* block).
        -- On an aeroplane with no lift thruster the altitude cascade's desired climb rate
        -- cannot be bought with vertical thrust -- the only way to go up is to point the nose
        -- up and let the forward thrust carry the craft along the new flight path. So in this
        -- mode the cascade's desiredVelY stops being a thruster demand and becomes the input to
        -- a flight-path-angle command, which the attitude loop then holds like any other pitch
        -- setpoint. Unlike the old cosmetic climb-pitch overlay this is NOT gated on bankSteer:
        -- the vertical channel has to keep working through autoland, the pad phases and a
        -- glideslope, i.e. exactly the states bankSteer switches off.
        -- This flag means "the FIXEDWING pitch channel OWNS the pitch axis on this airframe",
        -- not "it is commanding something right now" -- deliberately, because it also selects
        -- the slew rate and the sign, and those must stay valid while the command is being
        -- unwound back to level (see the slew note below). Being on the gear is handled INSIDE
        -- the block by commanding zero, not by dropping ownership.
        local fwPitch = CFG.FW_PITCH_ENABLE and CFG.FLIGHT_MODE == "FIXEDWING"
            and CFG.ENABLE_ALTITUDE

        -- ---- desired heading, per phase (the yaw "autopilot") ----
        --  ENROUTE/APPROACH -> seek the bearing to the target (keep pointing the nose all the
        --                      way in; APPROACH is still an active flying leg -- see translateOnly)
        --  LANDING  -> hold the commanded pad heading
        --  else (TERMINAL/ARRIVED at the final point) / hands-off / no target / AP_NAV off ->
        --                      current heading: stop seeking, the attitude loop only rate-damps
        --                      yaw and the pilot / autoland sequence takes the nose from here.
        -- ---- manual horizontal + yaw stick (MIC #14) ----
        -- Live only with AP-NAV OFF and a fresh MIC packet. Manual yaw is a RATE command:
        --   STICK HELD -> feed the wanted yaw rate straight to the inner rate loop (crisp,
        --     constant rotation). Do NOT ramp a heading target and chase it -- the position
        --     cascade's decel-limited profile is built to slew to a FIXED heading and stop,
        --     so tracking a moving ramp makes it overshoot/reverse (the counter-steer buzz).
        --   STICK CENTRED -> the stable inner rate loop nulls the yaw to zero (wdy=0), then the
        --     heading is frozen where it stops and held -- the MIC's "cancel unwanted yawing".
        -- manHeading tracks the live heading whenever we're NOT holding (commanding, or not
        -- manual), so the freeze-on-release grabs the current heading with no snap.
        local manNavFresh = manNavLastMsg and (now - manNavLastMsg) <= CFG.MANNAV_STALE_AFTER
        local manNavLive  = CFG.MANNAV_ENABLE and (not navActive) and manNavFresh and true or false
        -- ---- MANCON / keyboard joystick (#21): AUTHORITY IS PER-AXIS, NOT ALL-OR-NOTHING ----
        -- THE BUG THIS REPLACES (found 2026-08-11): the old single kjLive flag gated the WHOLE
        -- stick -- pitch, roll and throttle -- on `not navActive`, i.e. on AP-NAV alone. It never
        -- consulted AP-ALT. So with AP-NAV off and ALT-AP ON, the stick stayed fully live and its
        -- pitch OVERWROTE the altitude channel's commanded pitch a few lines below. A centred
        -- stick means pitch 0, so the aeroplane flew resolutely level and simply sat at whatever
        -- altitude the physics trims it to, while the vertical law upstream went on computing a
        -- perfectly good climb/descent command that was thrown away every tick. That is the
        -- "commanded Vy large, achieved ~0", the "won't leave ~320", and the sub-one-step
        -- elevator -- and it is why tuning the vertical law never moved the needle.
        --
        -- The rule, per the pilot: each autopilot owns its own axes and MANCON gets the REST.
        --   AP-ALT owns PITCH (it is the vertical channel).
        --   AP-NAV owns ROLL and THROTTLE (they are the navigation channel).
        --   both on  -> MANCON is completely overshadowed
        --   ALT only -> MANCON keeps throttle and roll
        --   NAV only -> MANCON keeps pitch
        --   neither  -> MANCON has full authority
        -- useAltCascade and navActive already mean exactly "ALT-AP (or autoland) owns the
        -- vertical" and "AP-NAV (or autoland) owns navigation", so the split reuses them rather
        -- than inventing new conditions -- and autoland, which owns everything, correctly
        -- silences the whole stick through both.
        -- Common gates unchanged: FIXEDWING only (the mode where roll is the lateral control),
        -- and a fresh packet, so silence hands the craft straight back instead of latching a
        -- stale bank. Reads CFG.FLIGHT_MODE live, so the MODE button arms/disarms in flight.
        local kjFresh = (CFG.KJOY_ENABLE and CFG.FLIGHT_MODE == "FIXEDWING"
            and kj.last and (now - kj.last) <= CFG.KJOY_STALE_AFTER) and true or false
        local kjPitch = kjFresh and not useAltCascade   -- ALT-AP owns pitch
        local kjRoll  = kjFresh and not navActive       -- AP-NAV owns roll
        local kjThr   = kjFresh and not navActive       -- ...and the throttle
        if manHeading == nil then manHeading = heading end
        local manYawCmd  = manNavLive and (math.abs(manYaw) >= CFG.MANNAV_YAW_EPS)   -- stick held
        local manYawRate = nil
        if manYawCmd then
            manYawRate = clamp(CFG.MANNAV_YAW_SIGN * manYaw, -CFG.MAX_RATE_YAW, CFG.MAX_RATE_YAW)
            manHeading = heading      -- track live heading while steering
        elseif manNavLive then
            -- Stick centred: with manYawRate nil and the heading error held at ~0 (below), the
            -- attitude loop runs wdy=0 -> the STABLE, critically-damped inner rate loop nulls wy
            -- (RATE_KP_YAW alone -- no extra brake gain, so no ring). Keep manHeading tracking the
            -- live heading while the craft is still rotating so there's no positional pull (pure
            -- rate-null), and CAPTURE/freeze the hold-heading only once the rate has settled -- so
            -- it holds where it actually stops, not the release-instant heading (momentum carries
            -- past that, and the heading-hold would then drag it back the opposite way).
            if math.abs(w.y) > CFG.MANNAV_YAW_SETTLE then
                manHeading = heading  -- still rotating: track -> no heading-hold pull, just the rate-null
            end                        -- settled: manHeading FROZEN at the stop heading -> hold it
        else
            manHeading = heading      -- not flying manually: stay synced (no snap on handoff)
        end

        local desiredHeading = heading
        if navActive then
            if hasTarget and (navPhase == "ENROUTE" or navPhase == "APPROACH") then desiredHeading = bearing
            elseif headingHold then desiredHeading = navHoldHeading end
        elseif manNavLive then
            desiredHeading = manHeading   -- used only when centred; a held stick overrides via manYawRate
        end

        -- ---- IDLE / PARKED trim-freeze gate ----
        -- Freeze (and bleed) the learnt trims when the craft sits still and is not being
        -- actively guided -- parked, or engine master OFF -- so a near-rest integral can't wind
        -- up against a tilt the thrusters have no authority to correct and poison the next
        -- flight. Any active guidance (a nav target, an autoland approach/descent, or an ALT-AP
        -- climb/descent) or the least motion clears it at once. "landed" (on the pad) is parked.
        -- MOVED ABOVE THE BANK/PITCH BLOCK (2026-08-10): the FIXEDWING vertical-by-pitch PI
        -- below is a second near-rest learner and needs the same freeze, so `idle` has to
        -- exist by the time the pitch command is built. Nothing it reads is computed in
        -- between, so this is a pure relocation; speedH is still defined before its other
        -- readers (the coordinated-turn sim and the ui.gs readout).
        local speedH    = math.sqrt(vel.x*vel.x + vel.z*vel.z)
        local wmag      = math.sqrt(w.x*w.x + w.y*w.y + w.z*w.z)
        local altActive = (TARGET_ALT ~= nil) and (math.abs(TARGET_ALT - latestPosY) > CFG.IDLE_ALT_TOL)
        local guiding   = (navActive and hasTarget) or (useAltCascade and altActive)
            or alState == "hold" or alState == "descend"
        local still     = (not guiding) and speedH < CFG.IDLE_SPEED
            and math.abs(vel.y) < CFG.IDLE_VSPEED and wmag < CFG.IDLE_RATE
        stillFor = still and (stillFor + dt) or 0
        local idle = stillFor >= CFG.IDLE_SETTLE
        ui.idle = idle

        -- ---- coordinated + cross-track bank + pitch-for-altitude (HYBRID/FIXEDWING) ----
        -- Only when the mode is steering by bank this tick (bankSteer -- see its note): zero
        -- in VTOL, during autoland, and in HYBRID's wings-level pad phases. Two bank sources:
        --   * coordinated-turn bank -- leans INTO a heading change (from the heading error),
        --     so it fades to zero once tracking; purely how a turn LOOKS.
        --   * cross-track bank -- the PRIMARY in-course lateral corrector: banks toward the
        --     track using the same body-right demand strafe would apply (last tick's
        --     latDemand), so the tilted alt-thruster lift pulls the craft back on line
        --     instead of strafing. See CFG.BANK_XTK_SIGN (a calibration GUESS).
        -- pitch tracks the desired climb rate. All are slewed for a smooth roll/pitch in/out.
        local bankTarget, pitchTarget = 0, 0
        -- WASHOUT REFERENCE for the heading-error bank's rate damping, stepped EVERY tick --
        -- not inside the bankSteer block -- so it never goes stale across a pad phase, an
        -- autoland, or a MODE button press and then dumps a fake "transient" into the first
        -- banking tick after it. See CFG.BANK_HDG_KD_WASHOUT.
        bnk.yawWash = bnk.yawWash + (CFG.YAW_RATE_SIGN * w.y - bnk.yawWash)
            * clamp(dt / math.max(CFG.BANK_HDG_KD_WASHOUT, dt), 0, 1)
        if bankSteer then
            -- Heading-error bank: zero on a tracked course. For a turn-RATE bank instead
            -- (banks during ANY yaw, incl. pilot yaw), use CFG.BANK_PER_YAWERR * w.y here.
            -- Rate-damped (2026-07-28): pure P here was harmless while bank was cosmetic, but
            -- now that CFG.HDG_FROM_BANK_SIGN actually turns the craft from this bank, the
            -- undamped P law rings (banks in, overshoots the heading, reverses, overshoots
            -- back -- the SAME class of "slow 2nd-order loop SWAYS if driven on position/angle
            -- error alone" the cross-track term below already solves with BANK_XTK_KD). Same
            -- fix here: subtract a term proportional to the ACTUAL yaw rate (in the SAME sign
            -- convention attitude() itself uses to compare a rate against ey/wdy, i.e.
            -- YAW_RATE_SIGN*w.y) so bank backs off as the heading is already closing, instead
            -- of only easing once hErr itself shrinks.
            -- Fade the heading-error term out near the target (2026-07-28): bearing
            -- (desiredHeading's source in ENROUTE/APPROACH) is a raw line-of-sight angle to a
            -- POINT, which -- exactly like the translation law's old per-fix closing PID before
            -- the APPROACH orbit fix (see the FMS history) -- gets noisy/whips around fast at
            -- short range (a small position wobble swings a big angle when dist is small, and
            -- once nearly abeam or past the point it can even point BACKWARD). Since that heading
            -- error now drives a REAL bank -> REAL turn (not a cosmetic lean), a whipping/reversing
            -- bearing fights the translation law, which stays smooth close-in because it already
            -- controls on VELOCITY error, not raw bearing -- "one wants to fly toward, the other
            -- away" was this exact fight, worst close to any target. The cross-track term below is
            -- NOT faded -- it reuses the translation law's own lastLatDemand, so it stays aligned
            -- with (not fighting) the correct, close-in-stable translation.
            local hdgBankFade = hasTarget
                and clamp((dist - CFG.BANK_HDG_FADE_NEAR) / (CFG.BANK_HDG_FADE_FAR - CFG.BANK_HDG_FADE_NEAR), 0, 1)
                or 1
            local hErr  = wrapPi(desiredHeading - heading)
            -- DAMP THE RATE TRANSIENT, NOT THE STEADY TURN RATE (2026-08-19). The term below
            -- used to subtract the RAW yaw rate, which is right for a heading CAPTURE (whose
            -- steady state is wings level, rate 0) and wrong for a curved PATH (whose steady
            -- state is a sustained rate). On an arc the loop settles where
            --   bank = BANK_PER_YAWERR*hErr - BANK_HDG_KD * GRAVITY*tan(bank)/v
            -- so the damper never stops pulling: it stands a permanent droop of
            -- BANK_HDG_KD*GRAVITY*tan(bank)/v rad off the commanded bank, and because it
            -- divides by SPEED that droop GROWS as the FMS slows for a corner -- i.e. it bites
            -- hardest exactly where the most bank is needed. At 120 m/s cruise it costs 0.11 of
            -- 0.785 rad (14%, which is what it was tuned against and why it looked healthy);
            -- at the 51/35/23 m/s Mantine's FMS schedules for the QATON/QETAM/BAZRA corners of
            -- WAU-BZR it costs 0.26/0.37/0.58 rad -- 33%/47%/74% of the ENTIRE bank authority.
            -- Holding the planned 45 deg at 23 m/s would have needed hErr = 134 deg, which pure
            -- pursuit to a 400 m carrot can never present, so the craft simply banked ~30 deg,
            -- flew every corner 2-3x wide, and read to the pilot as "it will not turn".
            -- Fix: high-pass (wash out) the rate before damping with it. bnk.yawWash is a slow
            -- low-pass of the same rate, so (rate - yawWash) is the CHANGE in turn rate:
            --   * settled on an arc -> yawWash has caught up -> term ~0 -> no droop, full
            --     bank authority, the corner radius the FMS planned is reachable.
            --   * rolling in, overshooting the heading, or ringing -> the rate is moving faster
            --     than the filter -> the term is live and damps exactly as before.
            --   * heading capture -> the rate decays to 0 and so does yawWash, so the end state
            --     is identical to the old law. This is the standard yaw-damper washout, and it
            --     is why the FIX IS THE FILTER, not a smaller BANK_HDG_KD: lowering the gain
            --     would trade the droop straight back for the ringing it was raised to cure.
            -- Bench: 97% of the damping survives a 0.4 Hz ring; a sustained turn's droop is
            -- gone within ~5 s. VTOL never sets bankSteer, so this whole block is inert there
            -- -- carried anyway so the four UACs stay one file apart from their conf.
            -- CFG.BANK_HDG_KD_WASHOUT -> huge reproduces the old (drooping) behaviour exactly.
            bankTarget  = clamp((CFG.BANK_PER_YAWERR * hErr
                    - CFG.BANK_HDG_KD * (CFG.YAW_RATE_SIGN * w.y - bnk.yawWash)) * hdgBankFade,
                -CFG.BANK_MAX, CFG.BANK_MAX)
            local latBank = CFG.BANK_XTK_SIGN * CFG.BANK_PER_LAT * lastLatDemand
            if hasLeg then   -- rate damping to settle the banking sway; survives P saturation
                latBank = latBank - CFG.BANK_XTK_SIGN * CFG.STRAFE_SIGN * CFG.BANK_XTK_KD * xtkRate
            end
            latBank = clamp(latBank, -CFG.BANK_LAT_MAX, CFG.BANK_LAT_MAX)
            -- Hand the SMALL corrections to strafe instead (CFG.STRAFE_EAGER_NEAR/FAR): close in,
            -- latBank is the only bank source left (the heading term is already faded by then) and
            -- it is riding pure velocity noise, which HDG_SIM's floored divisor turns into
            -- full-rate yaw commands at a standstill. Note this is a DISTANCE fade choosing which
            -- actuator flies the correction, not the removed heading-error fade below.
            latBank = latBank * (1 - strEager)
            -- No alignment fade here -- see the CFG.BANK_XTK_SIGN / removed BANK_XTK_FADE notes.
            -- latBank and the heading-error term above now agree in sign for every geometry, so
            -- they simply ADD; latBank is the strong primary steering term and hdgBank the lean/trim.
            bankTarget  = clamp(bankTarget + latBank, -CFG.BANK_MAX, CFG.BANK_MAX)
            -- Pitch from LAST tick's desired climb rate (computed after attitude, below);
            -- one 50 ms tick of lag on a cosmetic pitch command is immaterial.
            pitchTarget = clamp(CFG.PITCH_PER_MPS * lastDesiredVelY, -CFG.PITCH_ALT_MAX, CFG.PITCH_ALT_MAX)
        end
        -- ---- FIXEDWING: desired climb rate -> commanded FLIGHT PATH ANGLE ----
        -- REPLACES the cosmetic overlay above (never adds to it -- same one-owner-per-setpoint
        -- rule the whole BANK_XTK_SIGN history exists to enforce), and is in turn replaced by
        -- the pilot's stick below.
        --
        -- IT IS A PI ON VERTICAL SPEED, AND THE STANDING ANGLE IS *LEARNED*, NOT ASSUMED.
        -- That is a deliberate retreat from the first two versions, which computed the angle
        -- from geometry (gamma = vs/V) and treated the feedback as a trim on top. Geometry is
        -- only right if pitch sets the FLIGHT PATH ANGLE -- true of an aeroplane held up by its
        -- wings, false of one held up by thrust, where pitch sets the vertical ACCELERATION and
        -- a constant commanded angle is a RAMP GENERATOR: vertical speed never settles, it just
        -- keeps growing until something saturates. That is the "commanded -3.75, reached -10,
        -- reversed, repeated" the pilot measured, and a bench reproduces it (-10 to -11 and a
        -- dozen-plus reversals on an integrating plant, against -4.4 and none on a lifting one).
        -- An integrator that LEARNS whatever standing angle holds the wanted rate is correct on
        -- BOTH plants, so the law no longer depends on knowing which one this airframe is.
        -- CFG.FW_PITCH_FF keeps the geometric term as an option; it is 0 on this aeroplane.
        --
        -- All terms are divided by the live speed, so they are angles: KP is dimensionless and
        -- KI is 1/s. That division is also a gain schedule that keeps one set of numbers working
        -- from a 20 m/s approach to a 280 m/s cruise, where the old fixed rad-per-m/s
        -- CFG.PITCH_PER_MPS is wrong by an order of magnitude at one end or the other.
        --
        -- USE THE TOTAL SPEED, NOT THE HORIZONTAL SPEED: speedH is V*cos(gamma), so pitching up
        -- SHRINKS the divisor, which RAISES the gain, which pitches up further -- a small
        -- positive feedback loop (x1.035 at 15 deg) sitting inside the negative one. |V| barely
        -- changes as the velocity vector rotates.
        --
        -- The integral is not optional: the outer altitude loop is P-ONLY
        -- (altPID = pid.new(0, CFG.ALT_KP, 0, 0)), and with no feedforward it is now the ONLY
        -- thing supplying the steady-state angle. It must still stay SLOW next to KP -- this is
        -- a PI wrapped around a plant that may itself integrate, and too much I against too
        -- little P is precisely what makes such a loop hunt (bench: 33 sign reversals at
        -- KP 0.25 / KI 0.02, down to 2 at KP 3.0 / KI 0.06).
        --
        -- Speed floor (CFG.FW_PITCH_MIN_SPEED) for the same reason CFG.HDG_SIM_MIN_SPEED has
        -- one: the smallest legal divisor is the largest gain, and a near-stationary craft
        -- asking for 3 m/s of climb would otherwise demand a vertical nose.
        if fwPitch and not alGrounded then
            local spd3  = math.sqrt(vel.x*vel.x + vel.y*vel.y + vel.z*vel.z)
            local vRef  = math.max(spd3, CFG.FW_PITCH_MIN_SPEED)
            -- SPEED PROTECTION -- NEVER ASK FOR A CLIMB THE AEROPLANE IS TOO SLOW TO FLY.
            -- This is the guard against the failure the first cut actually showed in flight:
            -- pitching up does not MAKE lift here, it only aims the forward thrust upward, so a
            -- climb the thrust cannot sustain is simply never delivered -- the rate error never
            -- closes, the trim winds until the command pegs, the nose-up bleeds off the very
            -- speed the climb depends on, and the aeroplane mushes nose-high and eventually goes
            -- over the top. Both symptoms the pilot reported are that one loop.
            -- The cap uses the RAW speed, deliberately NOT the floored vRef: as the craft slows,
            -- the commanded climb rate shrinks with it and reaches zero on its own, so the nose
            -- comes DOWN, the aeroplane accelerates, and the climb resumes -- which is exactly
            -- what a real aeroplane (and a real autopilot) does when it runs out of energy.
            -- HEADROOM < 1 also means the feedforward alone can never reach FW_PITCH_MAX, so the
            -- whole remaining range stays available to the feedback as correction margin.
            local vsMax = CFG.FW_PITCH_HEADROOM * CFG.FW_PITCH_MAX * spd3
            local vsCmd = clamp(lastDesiredVelY, -vsMax, vsMax)
            ui.fwVsCap = (vsCmd ~= lastDesiredVelY)   -- display: speed protection is limiting us
            local vsErr = vsCmd - vel.y
            -- THE TRIM TERM HAS A DIFFERENT SPEED LAW FROM THE OTHER TWO (pilot, 2026-08-14):
            -- "at faster airspeed the airframe only needs a small nose-up to stay level, at
            -- slower speed it requires greater nose-up". Correct, and the law had it wrong.
            -- FF*vsCmd and KP*vsErr are FLIGHT PATH ANGLES -- vs/V -- so dividing by vRef once
            -- is exactly right for them. KI*fw.pitchI is an ANGLE-OF-ATTACK TRIM, and for
            -- constant lift C_L ~ 1/V^2, so trim AoA falls as 1/V^2, NOT 1/V. All three were
            -- sharing one divisor because the law was written when pitch was believed to buy
            -- nothing but flight path angle -- the same disproven assumption behind the
            -- geometric feedforward and the thrust-borne-plant detour.
            -- Scaled by TRIM_VNOM so the law is IDENTICAL at that speed and the already-learnt
            -- integral carries over unchanged; only the off-cruise behaviour differs (at 20 m/s
            -- the trim is now 6.25x its cruise value instead of 2.5x). The `or vRef` fallback
            -- reproduces the OLD law exactly when the key is absent, so this is a no-op on any
            -- airframe whose config does not define it.
            local trimAng = CFG.FW_PITCH_KI * fw.pitchI
                * (CFG.FW_PITCH_TRIM_VNOM or vRef) / (vRef * vRef)
            local gamma = (CFG.FW_PITCH_FF * vsCmd
                + CFG.FW_PITCH_KP * vsErr) / vRef + trimAng
            ui.fwTrimDeg = math.deg(trimAng)   -- published here: trimAng only exists in this branch
            pitchTarget = clamp(gamma, -CFG.FW_PITCH_MAX, CFG.FW_PITCH_MAX)
            -- Integrate only when the command is inside its clamp, or when the error is pushing
            -- it back toward the clamp -- the same conditional-integration anti-windup the speed
            -- and strafe loops use. Parked: bleed, never learn (see integStep's note -- a craft
            -- on the ground is exactly the accumulate condition and exactly the wrong one).
            -- ALSO frozen while the speed protection above is limiting the demand: the error is
            -- then measured against a rate we have deliberately chosen not to ask for, and
            -- learning from it is precisely how the trim used to wind the nose up and keep it
            -- there. This is the windup case the clamp test alone does NOT catch -- the command
            -- can sit well inside FW_PITCH_MAX while the demand is still unachievable.
            if idle then
                fw.pitchI = fw.pitchI * CFG.ATT_I_IDLE_BLEED
            elseif vsCmd ~= lastDesiredVelY then
                -- SPEED PROTECTION ACTIVE: FREEZE, DO NOT BLEED (fixed 2026-08-14).
                -- This branch used to decay the trim at ATT_I_IDLE_BLEED (0.95/tick, a ~1 s time
                -- constant). It fires whenever the cap bites, i.e. for ANY altitude error over
                -- ~122 m at cruise (vsMax = 0.27*0.45*50 = 6.1 m/s against desiredVelY =
                -- ALT_KP*error), so EVERY large altitude change DESTROYED the vertical trim and
                -- held it at zero for the whole approach -- leaving only FF+KP with nothing
                -- compensating the airframe's trim angle of attack. Reported as "climb/descend
                -- rate is inadequate": it was climbing on geometry alone with the learnt trim
                -- deleted underneath it.
                -- The reason NOT to integrate here is that the error is measured against a rate
                -- we deliberately chose not to ask for -- that is an argument for FREEZING, not
                -- for discarding a trim that is still true. Same distinction the vertical
                -- cascade already makes for velPID ("Freeze, don't bleed: the hover lift is a
                -- real value wanted at the next takeoff"). Windup is still prevented, because
                -- freezing stops accumulation just as effectively.
                --
                -- ...BUT A PLAIN FREEZE LOCKS IN AN ALREADY-WOUND-UP TRIM, which is worse than
                -- the bleed it replaced (found 2026-08-14, same day). Failure it caused: with a
                -- target ABOVE the aeroplane's lift ceiling, vsErr stays positive indefinitely,
                -- fw.pitchI winds to FW_PITCH_I_LIMIT (0.05*300/50 = 0.30 rad = 17 deg of
                -- standing nose-up), and the clamp-based anti-windup below never fires because
                -- 17 deg sits well inside FW_PITCH_MAX (26 deg) -- it can only see the COMMAND
                -- saturating, never the AEROPLANE saturating. Then a descent demand of a few
                -- degrees cannot out-vote 17 deg of trim, so it keeps climbing whatever you
                -- select: the "pins at 300..320 regardless of the set altitude" symptom.
                -- ONE-DIRECTIONAL INTEGRATION is the correct anti-windup: never let it GROW
                -- while the demand is capped, but ALWAYS let it unwind toward zero, so a trim
                -- learnt against an unachievable demand can always be given back.
                local d = vsErr * dt
                if d * fw.pitchI < 0 then     -- moving toward zero: allow it
                    fw.pitchI = clamp(fw.pitchI + d, -CFG.FW_PITCH_I_LIMIT, CFG.FW_PITCH_I_LIMIT)
                end

            elseif pitchTarget == gamma or (gamma - pitchTarget) * vsErr < 0 then
                fw.pitchI = clamp(fw.pitchI + vsErr * dt, -CFG.FW_PITCH_I_LIMIT, CFG.FW_PITCH_I_LIMIT)
            end
        else
            -- ON THE GEAR, or not flying the vertical by pitch at all (VTOL/HYBRID, mode button
            -- just moved). Command LEVEL and bleed the trim away so coming back does not start
            -- from a value learnt under conditions that no longer apply.
            -- The gear case is why this commands zero rather than simply not writing
            -- pitchTarget: a rollout's standing -AL_GROUND_VS demand would otherwise hold a
            -- permanent nose-down and dig the aeroplane into the runway. It is also why fwPitch
            -- stays TRUE here -- ownership of the axis must outlive the command, or the slew
            -- below has no non-zero step with which to unwind it.
            if fwPitch then pitchTarget = 0 end
            fw.pitchI = fw.pitchI * CFG.ATT_I_IDLE_BLEED
        end
        ui.fwPitch, ui.fwPitchI = (fwPitch and not alGrounded), fw.pitchI
        ui.fwTrimPct = 100 * fw.pitchI / CFG.FW_PITCH_I_LIMIT   -- ui.fwTrimDeg set where trimAng lives
        -- ---- KEYBOARD JOYSTICK (#21): the stick OWNS the attitude setpoint ----
        -- REPLACES the coordinated/cross-track bank and climb pitch above, never adds to
        -- them. Blending would recreate exactly the two-terms-fighting-over-one-bankTarget
        -- failure the BANK_XTK_SIGN comment block documents at length -- and a pilot's stick
        -- is not a term to be trimmed by the autopilot. KJOY_*_MAX (60 deg) deliberately
        -- exceeds BANK_MAX; see its CFG note on the altitude sag that buys.
        -- PER-AXIS now (see the kjPitch/kjRoll block above): the stick may own the roll while
        -- ALT-AP keeps the pitch, or the pitch while AP-NAV keeps the roll. Taking these as one
        -- decision is precisely the bug that let a centred stick veto the altitude channel.
        if kjRoll then
            bankTarget  = clamp(CFG.KJOY_ROLL_SIGN * kj.roll, -CFG.KJOY_ROLL_MAX,  CFG.KJOY_ROLL_MAX)
        end
        if kjPitch then
            pitchTarget = clamp(kj.pitch,                     -CFG.KJOY_PITCH_MAX, CFG.KJOY_PITCH_MAX)
        end
        -- While hand-flying, #21's own STICK_RATE is the pilot's roll rate, so slew fast
        -- enough to just track it (KJOY_SLEW_RATE) instead of adding a second lag. This also
        -- has to bypass PITCH_ALT_RATE, which is 0 for the disabled climb-pitch overlay: a
        -- zero step would freeze pitchCmd at 0 and eat the stick's pitch silently.
        -- ...and FIXEDWING's vertical-by-pitch needs its own step size for the very same reason
        -- the stick did: CFG.PITCH_ALT_RATE is 0.0, and slew() with a zero step can never move
        -- pitchCmd off 0, so the command would be discarded in silence before attitude() ever
        -- saw it. Priority stick > FIXEDWING > the old overlay, matching who owns pitchTarget.
        bankCmd = slew(bankCmd, bankTarget, (kjRoll and CFG.KJOY_SLEW_RATE or CFG.BANK_RATE) * dt)
        local pitchRate = (kjPitch and CFG.KJOY_SLEW_RATE)
            or (fwPitch and CFG.FW_PITCH_RATE) or CFG.PITCH_ALT_RATE
        -- A ZERO STEP MUST NEVER BE ABLE TO LATCH A COMMAND NOBODY OWNS ANY MORE. The zeroed
        -- CFG.PITCH_ALT_RATE is the fallback here, so the instant an owner stands down --
        -- the joystick going stale, the MODE button leaving FW -- pitchTarget correctly drops
        -- to 0 but the step to get there is 0 too, and pitchCmd stays wherever that owner left
        -- it, for the rest of the flight. That is a real latch in the SHIPPED joystick path,
        -- not just the new one: "silence hands the craft straight back" was true of roll (which
        -- rides the live CFG.BANK_RATE) and quietly false of pitch. One guard fixes both.
        if pitchRate <= 0 and pitchCmd ~= 0 then pitchRate = CFG.FW_PITCH_RATE end
        pitchCmd = slew(pitchCmd, pitchTarget, pitchRate * dt)

        -- ---- ATTITUDE ----
        -- Manual yaw brake: engage the extra yaw rate-null only when flying manually with the
        -- yaw stick released (command ~0). Drives residual angular velocity to zero hard so a
        -- hand-flown yaw settles crisply; a no-op for the autopilot and while actively yawing.
        local yawBrake = (manNavLive and math.abs(manYaw) < CFG.MANNAV_YAW_EPS)
            and CFG.MANNAV_YAW_BRAKE or 0
        ui.yawBrake = (yawBrake > 0)

        -- ---- coordinated-turn heading simulation (HYBRID only, see its CFG block) ----
        -- Bank -> real turn rate, whatever produced that bank (heading-error lean and/or
        -- cross-track correction both feed bankCmd already); floor v so a slow/hovering
        -- HYBRID craft that's still banked can't demand a runaway rate.
        -- Two callers, ONE law -- whatever produced the bank, a real aircraft banked at that
        -- angle turns at omega = g*tan(bank)/v, so they share the formula and the whole
        -- HDG_FROM_BANK_SIGN / MIN_SPEED / MAX_RATE calibration:
        --   * HYBRID's coordinated-turn sim (CFG.HDG_SIM_ENABLE), where the bank is the
        --     autopilot's own heading-error lean + cross-track correction.
        --   * the hand-flown keyboard joystick (CFG.KJOY_BANK_TURN), where it is the pilot's
        --     stick. FIXEDWING normally needs no such coupling -- its yaw law chases the live
        --     BEARING and so follows the curving path round on its own -- but with AP-NAV off
        --     there IS no bearing: desiredHeading is frozen at the current heading, so the
        --     nose would stay rigidly put while the banked lift slid the craft sideways.
        --     Note this second case closes NO control loop (the bank comes from the pilot's
        --     fingers, not from a heading error), which is why it needs none of the damping
        --     and fading HYBRID's closed hErr->bank->yaw->hErr loop had to grow.
        -- The joystick may scale the derived rate (KJOY_TURN_GAIN) and pin the speed in the
        -- divisor (KJOY_TURN_REF_SPEED) -- see their CFG block for why the honest coordinated
        -- rate is too lazy to hand-fly at this airframe's speeds. The HYBRID autopilot sim
        -- takes neither: it flies a radius the FMS planned from real physics.
        local bankYawRate = nil
        local kjTurn = kjRoll and CFG.KJOY_BANK_TURN
        if (CFG.HDG_SIM_ENABLE and bankSteer and CFG.FLIGHT_MODE == "HYBRID") or kjTurn then
            local v = (kjTurn and CFG.KJOY_TURN_REF_SPEED)
                or math.max(speedH, CFG.HDG_SIM_MIN_SPEED)
            bankYawRate = clamp((kjTurn and CFG.KJOY_TURN_GAIN or 1)
                    * CFG.HDG_FROM_BANK_SIGN * CFG.GRAVITY * math.tan(bankCmd) / v,
                -CFG.HDG_SIM_MAX_RATE, CFG.HDG_SIM_MAX_RATE)
        end
        ui.bankYawRate = bankYawRate

        local pitchSteps, rollSteps, yawSteps = 0, 0, 0
        if CFG.ENABLE_ATTITUDE then
            -- The joystick's pitch needs its own sign: attitude()'s default CFG.PITCH_ALT_SIGN
            -- is 0 (climb-pitch overlay off) and would multiply the stick by zero. FIXEDWING's
            -- vertical-by-pitch carries its own sign for exactly the same reason -- and both
            -- describe the same physical thing ("+cmd = nose up"), so CFG.FW_PITCH_SIGN should
            -- track the in-flight-calibrated CFG.KJOY_PITCH_SIGN rather than be re-guessed.
            pitchSteps, rollSteps, yawSteps = attitude(q_cur, q_inv, w, heading, desiredHeading, dt, bankCmd, pitchCmd, idle, yawBrake, manYawRate, bankYawRate,
                (kjPitch and CFG.KJOY_PITCH_SIGN) or (fwPitch and CFG.FW_PITCH_SIGN) or nil)
        end

        -- ---- HORIZONTAL TRANSLATION (ported navcom) ----
        -- navActive off => positional control handed to the pilot: command no translation
        -- (the tilt-allocation vertical spill below may still add some -- that is executing
        -- the vertical command, not navigating).
        local thrustCmd, strafeCmd = 0, 0
        if CFG.ENABLE_TRANSLATION and navActive and hasTarget then
            if translateOnly or approachVec or navCarrot or alignStop then
                -- Velocity-vector control toward the target. APPROACH and the en-route CARROT both
                -- track the FMS's scheduled speed directly (single source of truth on the HSI); the
                -- carrot is a moving point on the smoothed path, so pure-pursuing it gives smooth
                -- bearing AND self-corrects cross-track (no separate leg hold). TERMINAL/ARRIVED/
                -- LANDING draw their own decelerate-to-zero position hold to stop AT the point.
                -- alignStop (VTOL/HYBRID route-start ALIGN) is the same law for the same reason:
                -- navCruiseCap is pinned at 0 for the hold, so desiredSpeed below comes out 0 and
                -- this is a full velocity-vector brake to a hover, not a pursuit of anything.
                local desiredSpeed
                if (approachVec or navCarrot or alignStop) and navCruiseCapDynamic then
                    desiredSpeed = clamp(navCruiseCap, 0, CFG.CRUISE_SPEED)
                else
                    local speedCap = (navPhase == "APPROACH") and CFG.APPROACH_SPEED or CFG.TERMINAL_MAX_SPEED
                    desiredSpeed = clamp(CFG.APPROACH_KP * dist, 0, speedCap)
                end
                local ux, uz = 0, 0
                if dist > 1e-6 then ux, uz = dx / dist, dz / dist end
                local velErrX = desiredSpeed * ux - vel.x
                local velErrZ = desiredSpeed * uz - vel.z
                local sinH, cosH = math.sin(heading), math.cos(heading)
                local fwdErr   = velErrX * sinH + velErrZ * cosH
                local rightErr = velErrZ * sinH - velErrX * cosH   -- matches xtk handedness
                -- INVERSE-PLANT FEEDFORWARD + trim, the shape the vertical cascade uses
                -- (CFG.STEPS_PER_MPS * desiredVelY + velPID:step(...)). The first term is the
                -- commanded velocity vector projected into the body frame and multiplied by the
                -- steps that buy one m/s, so on its own it already commands very nearly the
                -- right thrust; TRANS_KP/STRAFE_VEL_KP are left only trimming what it misses.
                -- Without it this law -- which is what flies the FMS's en-route CARROT, i.e.
                -- the normal cruise law -- was a bare P with no integral, and had to stand a
                -- 20 m/s speed error just to hold full thrust. See CFG.SPEED_AT_FULL.
                local fwdCmd   = CFG.STEPS_PER_MPS_FWD * (desiredSpeed * (ux * sinH + uz * cosH))
                    + CFG.TRANS_KP * fwdErr
                local rightCmd = CFG.STEPS_PER_MPS_LAT * (desiredSpeed * (uz * sinH - ux * cosH))
                    + CFG.STRAFE_VEL_KP * rightErr
                local mag = math.sqrt(fwdCmd*fwdCmd + rightCmd*rightCmd)
                -- BREAKAWAY FLOOR (CFG.TRANSLATE_MIN_CMD / _FWD). Two changes, 2026-08-21,
                -- both from "Aer Togekiss rocks fore-aft when it should be making minuscule
                -- adjustments", traced to this block commanding +/-15 of #69's 45 A-units.
                --
                -- (1) PER-AXIS. One floor in STEP units cannot mean the same thing on both
                -- axes: a step is SPEED_AT_FULL/THRUST_MAX of fore/aft demand but only
                -- STRAFE_AT_FULL/STRAFE_MAX of lateral, and on Aer Togekiss those differ
                -- EIGHTFOLD (21.3 vs 2.67 m/s per step). The old flat 5.0 was therefore a
                -- sane 13 m/s nudge sideways and a 107 m/s LUNGE fore/aft. The floor is now
                -- blended along the demand's OWN direction, so it still scales the magnitude
                -- and preserves the direction (which is the point of a vector boost -- a
                -- per-axis clamp would steer the craft off the bearing it wants) while
                -- meaning the same physical nudge whichever way it points.
                --
                -- (2) FADE, NOT A SWITCH. `closing < MIN_RATE` was a relay: at 0.349 m/s the
                -- command was the full floor and at 0.351 whatever the PID said, with nothing
                -- in between. An equilibrium sitting on that switching surface cannot do
                -- anything BUT limit-cycle -- kick, accelerate through MIN_RATE, boost drops,
                -- coast, overshoot, closing goes negative, kick the other way. That is the
                -- rocking. Fading the boost out as the closing rate comes up to MIN_RATE keeps
                -- the breakaway at a standstill (need = 1 at closing <= 0, so drifting AWAY
                -- still gets the full floor) and reaches MIN_RATE continuously.
                local floorCmd = CFG.TRANSLATE_MIN_CMD
                if mag > 1e-6 then
                    local uf, ul = fwdCmd / mag, rightCmd / mag
                    floorCmd = math.sqrt((uf * CFG.TRANSLATE_MIN_CMD_FWD) ^ 2
                                       + (ul * CFG.TRANSLATE_MIN_CMD) ^ 2)
                end
                --
                -- (3) A COMMANDED ZERO IS A STOP, NOT A BREAKAWAY (2026-09-01). The floor is
                -- gated on `desiredSpeed > 0` -- on A MOVEMENT BEING ASKED FOR -- and not, as
                -- it was, on the range to the target and the closing rate alone. Neither of
                -- those says anything about whether any speed was commanded, and during the
                -- FMS's route-start ALIGN none is: it holds cruiseCap at 0 (NAV.aligning)
                -- while the nose swings onto the first bearing, so desiredSpeed here is 0, the
                -- feedforward vanishes, and the whole demand is the brake KP * (-vel).
                --
                -- A craft parked on a pad is never exactly still -- the hull sways, the lift
                -- banks dither -- so `mag` was a few hundredths of a step while `dist` was the
                -- 200 m carrot and `closing` ~ 0 (need = 1), and the floor multiplied that
                -- residue up to the FULL breakaway, bang-bang about zero velocity, with the
                -- HSI reading ALIGN / CMD 0. Nor does it fade with speed: the demand only
                -- reaches the floor honestly at |velErr| = MIN_CMD/KP, so the boost stays hard
                -- on for the whole of the hold. Per airframe, with KP = 0.75 on both axes
                -- everywhere (TRANS_KP, and STRAFE_VEL_KP = 2.0 * STEPS_PER_MPS_LAT):
                --   fore/aft  MIN_CMD_FWD 5.0 of THRUST_MAX 15 out to 6.7 m/s -- Scout,
                --             Citation, Mantine, SS Argo. A THIRD of thrust authority.
                --   fore/aft  MIN_CMD_FWD 0.5 of 15 out to 0.7 m/s -- Aer Togekiss alone,
                --             because (1) above already tuned it down for the rocking. That
                --             is why this shows up on Scout first and is mild on #1.
                --   lateral   MIN_CMD 5.0 of STRAFE_MAX 15 out to 6.7 m/s -- ALL FIVE. The
                --             strafe kick was never tuned down, so no airframe escaped it.
                --
                -- This is the same rule as the commanded-zero note in the ENROUTE law below,
                -- one law over: that one was an integral with no operating point to trim, this
                -- one is a floor with nothing to break away TO. A brake does not need a floor;
                -- it needs to fade to nothing as the craft stops, which is what the bare P
                -- term already does. Every use the floor exists for still has it -- the pad
                -- creep, the en-route carrot and the runway rollout all command a positive
                -- speed, and none of their behaviour moves by a bit.
                if desiredSpeed > 0
                        and dist > CFG.TRANSLATE_DEADZONE and mag > 1e-6 and mag < floorCmd then
                    local need = clamp((CFG.TRANSLATE_MIN_RATE - closing)
                        / math.max(CFG.TRANSLATE_MIN_RATE, 1e-6), 0, 1)
                    local k = 1 + (floorCmd / mag - 1) * need
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
                -- When the FMS is scheduling the speed (turn-aware look-ahead), TRACK its cap
                -- directly -- it already bleeds speed in for sharp corners and the destination and
                -- holds cruise through gentle fly-by fixes. Only fall back to the old decelerate-
                -- toward-THIS-fix law (which slowed at every waypoint) when no schedule is present.
                local desiredClosing
                if navCruiseCapDynamic then
                    -- Track the FMS speed EXACTLY. It already folds in the proximity/look-ahead
                    -- slowdown AND the nose-alignment / cross-track gates, so re-deriving any of
                    -- that here would double-cut it and make the FMS's displayed CMD a lie (the
                    -- craft coasted at ~30 while the HSI read 130). One source of speed truth.
                    desiredClosing = cap
                else
                    desiredClosing = clamp(CFG.APPROACH_KP * dist, 0, cap) * align
                end
                -- A COMMANDED ZERO IS A STOP, NOT A TRIM POINT (2026-08-29). This is the law that
                -- runs when the FMS sends NO carrot -- i.e. the plan is being joined from off the
                -- planned path, which is exactly what a mid-route RESUME does -- and it was the
                -- only translation law that carried its integral through a commanded zero.
                -- The FMS holds cruiseCap at 0 for its route-start ALIGN (NAV.aligning), so the
                -- error here is 0 - closing: at a standstill that is 0, and the loop's ENTIRE
                -- output is SPD_KI * integral. That integral is bounded at THRUST_MAX/SPD_KI, so
                -- a trim wound up on the previous cruise -- and nothing unwinds it, because
                -- AP-NAV off does not step this PID, it just stops stepping it -- commands up to
                -- FULL FORWARD THRUST while the HSI reads CMD 0 and ALIGN. It does not clear
                -- itself either: accelerating at 90 degrees to the target leaves closing ~ 0 and
                -- the error at 0, and pointing AWAY from it makes closing negative and winds the
                -- integral up further. That is "the autopilot accelerates during the align
                -- phase", and why it shows up on a mid-route join and not on a route flown from
                -- its first waypoint (which is on the path, takes the carrot law above, and that
                -- law already zeroes both integrals every tick).
                -- The P term still brakes a commanded zero, which is what is supposed to stop the
                -- craft; an integral has no operating point to trim when the demand is zero.
                if desiredClosing <= 0 then speedPID.integral = 0 end
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
                    -- Feedforward + trim, as in the velocity-vector law above; STRAFE_VEL_KP
                    -- (steps per m/s) not STRAFE_KP (steps per metre of cross-track) -- the two
                    -- were one overloaded constant until 2026-08-20.
                    strafeCmd = CFG.STRAFE_SIGN * clamp(
                        CFG.STEPS_PER_MPS_LAT * (dSpeed * (uz2 * math.sin(heading) - ux2 * math.cos(heading)))
                        + CFG.STRAFE_VEL_KP * rErr, -CFG.STRAFE_MAX, CFG.STRAFE_MAX)
                    strafePID.integral = 0
                end
            end
        elseif CFG.ENABLE_TRANSLATION and manNavLive then
            -- MANUAL body-frame velocity command from the MIC (#14). Proportional velocity
            -- control to the pilot's wanted forward/right speed -- the SAME body projection and
            -- gains/signs as the TERMINAL translate law above, but the target is pilot-named,
            -- not position-derived. A released stick sends 0 -> the craft brakes to a hover in
            -- place (velocity-hold at zero), which is how sideways drift gets nulled.
            local sinH, cosH = math.sin(heading), math.cos(heading)
            local fwdVel   = vel.x * sinH + vel.z * cosH        -- body forward speed
            local rightVel = vel.z * sinH - vel.x * cosH        -- body right speed (xtk handedness)
            -- Same feedforward + trim split as the autopilot's velocity-vector law: the stick
            -- names a body-frame speed, so the steps that buy that speed can be commanded
            -- outright and the feedback left to trim. A centred stick asks for 0, so the
            -- feedforward vanishes and this is the old pure velocity-hold-at-zero exactly.
            local fwdCmd   = CFG.STEPS_PER_MPS_FWD * manFwd    + CFG.TRANS_KP      * (manFwd    - fwdVel)
            local rightCmd = CFG.STEPS_PER_MPS_LAT * manStrafe + CFG.STRAFE_VEL_KP * (manStrafe - rightVel)
            thrustCmd = CFG.THRUST_SIGN * clamp(fwdCmd,   -CFG.THRUST_MAX, CFG.THRUST_MAX)
            strafeCmd = CFG.STRAFE_SIGN * clamp(rightCmd, -CFG.STRAFE_MAX, CFG.STRAFE_MAX)
            speedPID.integral  = 0
            strafePID.integral = 0
        else
            -- NOTHING drives translation this tick: AP-NAV off with no MANCON, AP-NAV on but no
            -- target yet, or translation disabled outright. Both integrals are FROZEN rather
            -- than stepped for as long as that lasts, so without this they survive the
            -- disengagement intact and are spent the instant navigation comes back -- before a
            -- single tick of error has been measured. Every branch above already zeroes them for
            -- exactly that reason; this is the case that had no branch. See the commanded-zero
            -- note in the ENROUTE law above for what a stale speed integral is worth.
            speedPID.integral  = 0
            strafePID.integral = 0
        end

        -- ---- KEYBOARD JOYSTICK THROTTLE (#21) ----
        -- A throttle LEVER names engine power, not a speed, so 0..1 maps straight onto
        -- forward thruster steps -- no PID to buy a velocity. It therefore OVERRIDES the
        -- velocity laws above (including the MIC's fwd demand, if a MIC is also live) rather
        -- than summing with them; the MIC's YAW is untouched, so stick+throttle here and
        -- rudder there is a sane split. Zero the speed integral so it cannot wind up while a
        -- fixed lever position owns the channel. One-sided: idle is 0, there is no reverse.
        if kjThr and CFG.ENABLE_TRANSLATION then
            thrustCmd = CFG.THRUST_SIGN * clamp(kj.thr * CFG.KJOY_THRUST_MAX, 0, CFG.THRUST_MAX)
            speedPID.integral = 0
        end

        -- Capture the body-right course-correction demand (pre-mode, pre-spill) so next
        -- tick's cross-track bank can fly it by BANKING instead of strafing.
        local latDemand = strafeCmd

        -- HYBRID banking phases: banking is the lateral corrector, so keep only a SPARING
        -- strafe -- just enough to damp the slow, 2nd-order bank oscillation (its D term
        -- opposes cross-track drift). FIXEDWING keeps NONE (zeroed at output). VTOL and the
        -- HYBRID pad phases keep full strafe.
        -- ...but blend back to FULL authority as the correction gets small (strEager), the other
        -- half of handing small corrections to translation -- the sparing fraction exists to stop
        -- strafe stealing the STEERING from bank, and below a few metres there is no steering
        -- left to steal. No-op in FIXEDWING (zeroed at output regardless), which is the point.
        if bankSteer and CFG.FLIGHT_MODE == "HYBRID" then
            local sf = CFG.HYBRID_STRAFE_FRAC + (1 - CFG.HYBRID_STRAFE_FRAC) * strEager
            local sm = CFG.HYBRID_STRAFE_MAX + (CFG.STRAFE_MAX - CFG.HYBRID_STRAFE_MAX) * strEager
            strafeCmd = clamp(strafeCmd * sf, -sm, sm)
        end

        -- ---- VERTICAL (ported control.lua cascade + autoland, OR manual VSC) ----
        -- LIFT ENGINE CUT ON TOUCHDOWN (CFG.AL_GROUND_CUT). Once the FMS has called the landing
        -- -- "landed" is sent from the pad sequence's touchdown and from the end of a runway
        -- rollout alike -- there is nothing left for the lift thrusters to buy. Holding them at
        -- a balanced hover, or on the standing down-bias below, leaves the engine burning
        -- against the ground: it keeps the craft skittish, unloads the gear the brakes need,
        -- and is the thing the pilot has always had to reach over and switch off by hand.
        -- So the whole channel goes to a flat zero on both lines. The altitude reference is NOT
        -- what does this -- alGrounded still decides whether there is one -- and the velocity
        -- loop's integral is frozen alongside, so the learnt hover trim is intact for the next
        -- takeoff (the cut lifts the moment the FMS cancels/releases the landing).
        -- On ui rather than a new local: controlLoop is one of the two functions this file
        -- has already had to keep off Cobalt's 200-locals cap, and the display wants it anyway.
        -- ---- SCRAMBLE: the VS stick's top detent, and the one vertical command that is NOT
        -- a rate. "Lift engine at full throttle, get us up NOW." See the CFG.MANCON_SCRAMBLE
        -- block in uacconf, and the VSC's own header on #6 for why it travels as a flag.
        --
        -- Decided HERE, above ui.altCut, because it outranks the ground cut: a scramble off the
        -- pad is half the reason to have one, and a cut engine cannot answer it. On `ui` rather
        -- than a local for the same 200-locals reason as ui.altCut itself (below).
        -- A stale or absent VSC reads false, so a dropped link can never latch full lift on.
        ui.scramble = (CFG.MANCON_SCRAMBLE and CFG.ENABLE_ALTITUDE and CFG.HAS_ALT_THRUSTER
            and manScramble and manLastMsg and (now - manLastMsg) <= CFG.MANCON_STALE_AFTER
            and (CFG.MANCON_SCRAMBLE_OVERRIDE or not useAltCascade)) or false
        ui.altCut = CFG.AL_GROUND_CUT and alState == "landed" and not ui.scramble
        local altUp, altDown = CFG.ALT_NEUTRAL / RS_MAX, CFG.ALT_NEUTRAL / RS_MAX
        local desiredVelY, velY = 0, vel.y
        local vertEffort = 0   -- signed alt-thruster steps wanted (+ = climb)
        if CFG.ENABLE_ALTITUDE then
            -- ---- desired climb rate: pick the source, then run ONE shared inner loop ----
            -- Three sources feed the same inner climb-rate loop below: the altitude-hold
            -- outer P, an autoland-commanded descent, or -- when altitude-hold is off -- the
            -- pilot's wanted rate straight off the VSC (#6) over mancon. The VSC no longer
            -- controls anything itself; it just names a rate, and this loop buys it.
            if ui.scramble then
                -- SCRAMBLE outranks all three sources below -- including an ALT-AP hold and an
                -- in-progress autoland, which is the point of CFG.MANCON_SCRAMBLE_OVERRIDE: a
                -- hand-flown go-around IS this detent. The rate named here is not what buys the
                -- climb (the outright full-lift command further down is), but it still has to be
                -- HONEST, because it is what the ALT line displays and what the FIXEDWING pitch
                -- overlay flies off next tick via lastDesiredVelY. CLIMB_AT_FULL is by definition
                -- the rate ALT_SWING steps of lift actually buys, so it is the truthful answer to
                -- "what is being asked for here" -- which the old 100 m/s placeholder was not.
                desiredVelY = CFG.CLIMB_AT_FULL
            elseif useAltCascade then
                if alState == "descend" then
                    desiredVelY = alDescendVS
                    if alLastMsg and (now - alLastMsg) > CFG.AL_STALE_AFTER then desiredVelY = 0 end
                elseif alGrounded then
                    -- ON THE GEAR, on ground truth. Hold the craft DOWN on it with a small
                    -- standing down-command and NO altitude reference at all. An altitude hold
                    -- here is actively wrong: the moment the craft settles a few centimetres --
                    -- gear compressing, a bump, the surface not being exactly where landing.db
                    -- says -- it answers by firing the lift thrusters UP, which keeps the
                    -- aircraft skittish, unloads the gear, and on a rollout stops the wheels
                    -- braking. With no squat contact to believe (alGrounded false) we fall
                    -- through to the altimeter below, which is the best we then have.
                    desiredVelY = -CFG.AL_GROUND_VS
                else
                    desiredVelY = altPID:step(latestPosY, dt)
                end
            else
                -- MANUAL: stale/absent stick -> hold (0 = hover). Clamp to the same rate
                -- limits the cascade obeys so a full stick can't ask for the impossible.
                -- ...clamped to the MANUAL envelope, which is deliberately not the autopilot
                -- one. CFG.CLIMB_MAX / DESCEND_MAX is the rate the ALT-AP hold and the autoland
                -- fly inside, and it is a handling decision made for THEM; the stick's boost key
                -- (mancon.lua's CFG.KEYS.BOOST) has to be able to ask past it, and buying that by
                -- raising CLIMB_MAX would have quietly steepened every automatic descent on the
                -- aircraft to pay for one keystroke. So the manual branch, and only the manual
                -- branch, clamps to CFG.MANCON_CLIMB_MAX / CFG.MANCON_DESCEND_MAX instead.
                -- These must be >= the stick's own CFG.VS.BOOST_MAX_CMD in manconconf.lua or
                -- the top of the boosted travel is thrown away here, and <= CLIMB_AT_FULL or it
                -- is thrown away by the aeroplane instead -- the rate loop can only ask for
                -- ALT_SWING steps, and CLIMB_AT_FULL is by definition what those buy.
                local manFresh = manLastMsg and (now - manLastMsg) <= CFG.MANCON_STALE_AFTER
                desiredVelY = manFresh
                    and clamp(manVS, -CFG.MANCON_DESCEND_MAX, CFG.MANCON_CLIMB_MAX) or 0
            end

            -- ---- inner climb-rate loop (shared): rate error -> alt-thruster steps ----
            -- SKIPPED ENTIRELY on an airframe with no lift thruster (CFG.HAS_ALT_THRUSTER).
            -- desiredVelY above is still computed -- it is what the FIXEDWING pitch law flies --
            -- but there is no vertical actuator to buy it with, so running this loop would only
            -- wind velPID's integral up against an output nobody receives (nothing on the fleet
            -- listens to CFG.ALT_PROTOCOL on such a craft) and then leak that phantom effort into
            -- the FORWARD thruster through the tilt allocation below, fighting the speed loop.
            if CFG.HAS_ALT_THRUSTER then
                if lastMass and math.abs(mass - lastMass) > CFG.MASS_STEP_THRESHOLD then trimBoost = CFG.TRIM_BOOST_TIME end
                lastMass = mass
                if trimBoost > 0 then trimBoost = trimBoost - dt end

                velPID.sp = desiredVelY
                local ib = velPID.integral
                local steps = CFG.STEPS_PER_MPS * desiredVelY + velPID:step(velY, dt)
                local stepsSat = clamp(steps, -CFG.ALT_SWING, CFG.ALT_SWING)
                if idle or alGrounded or ui.altCut or ui.scramble then
                    -- Parked/landed: HOLD the learnt hover trim (as shutdown() preserves it) rather
                    -- than pumping it against a grounded craft that can't respond -- undo this tick's
                    -- integration. Freeze, don't bleed: the hover lift is a real value wanted at the
                    -- next takeoff, unlike the attitude trim which bleeds toward zero.
                    -- alGrounded covers the case `idle` cannot: a ROLLOUT is moving fast, so it is
                    -- not idle, but the wheels are down and the standing down-command above is one
                    -- the loop can never satisfy -- left to integrate it would wind up hard against
                    -- the ground and poison the hover trim for the next takeoff.
                    -- ui.scramble is in this list for a THIRD reason: during a scramble this loop's
                    -- output is thrown away (see the override below), so it must not go on learning
                    -- from an error it is not being allowed to answer. Left running it would spend
                    -- the whole detent winding against a saturated output and hand back a poisoned
                    -- hover trim the moment the pilot let go -- a scramble would end in a sag.
                    velPID.integral = ib
                elseif stepsSat ~= steps and (steps - stepsSat) * velPID.integral > 0 then
                    velPID.integral = ib
                elseif trimBoost > 0 then
                    velPID.integral = clamp(velPID.integral + (desiredVelY - velY) * dt * (CFG.TRIM_BOOST_GAIN - 1),
                        -VEL_I_LIMIT, VEL_I_LIMIT)
                end
                vertEffort = stepsSat
                -- Belt and braces on the same rule: on the gear, nothing may command lift UP.
                if alGrounded then vertEffort = math.min(vertEffort, 0) end
                -- ...and with the engine cut, nothing may command it at all -- including the
                -- share the tilt allocation below would otherwise spill into fore/aft + strafe.
                if ui.altCut then vertEffort = 0 end
                -- SCRAMBLE HAS THE LAST WORD, over the rate loop and over both ground rules
                -- above. Stated outright rather than left for the rate loop to arrive at: the
                -- feedforward alone (STEPS_PER_MPS * CLIMB_AT_FULL) is exactly ALT_SWING, so in
                -- the ordinary case this line changes nothing -- but that is the ordinary case
                -- only while the craft is still SLOWER than CLIMB_AT_FULL. Once it is not (a
                -- dive recovery, a gust, or simply a CLIMB_AT_FULL set conservatively), the rate
                -- loop starts SUBTRACTING lift to hold the figure, which is the exact opposite
                -- of what the pilot asked for. A scramble is an open throttle, not a rate.
                -- The tilt allocation below still applies, and should: it routes this by the
                -- true attitude, so full lift means full lift ALONG WORLD-UP (and flips sign
                -- inverted) rather than full lift out of whichever way the roof happens to face.
                if ui.scramble then vertEffort = CFG.ALT_SWING end
            end
        end
        lastDesiredVelY = desiredVelY   -- feed next tick's pitch-for-altitude overlay

        -- ---- ATTITUDE-AWARE VERTICAL ALLOCATION (Bug 2) ----
        -- "Hold altitude" is world-up, but the alt thruster is body-fixed. Route the
        -- vertical effort by the true orientation: the alt thruster gets the body-up share
        -- of world-up (flips sign inverted, fades toward 90 deg pitch), and fore/aft + strafe
        -- pick up the rest. No-op at level (world-up-in-body = (0,1,0) -> all to alt).
        -- Also a no-op with no lift thruster: vertEffort is then permanently 0, so this whole
        -- block is stated explicitly rather than left to arrive at zero by arithmetic.
        local altSteps = vertEffort
        if CFG.ENABLE_ALTITUDE and CFG.TILT_ALT_ALLOC and CFG.HAS_ALT_THRUSTER then
            local ux, uy, uz = rotateVec(q_inv, 0, 1, 0)   -- world-up expressed in body
            altSteps  = vertEffort * uy                     -- body-up axis  -> alt thruster
            thrustCmd = thrustCmd + vertEffort * uz * CFG.VERT_XFER_FWD   -- body-fwd -> fore/aft
            -- Lateral spill: normally the strafe thruster picks up the banked alt thruster's
            -- sideways component so altitude AND position hold. But while STEERING by bank
            -- (bankSteer) that sideways component is exactly the lift turning the craft, so
            -- cancelling it would defeat the bank -- leave it UNcancelled (as FIXEDWING does
            -- by zeroing strafe entirely). Otherwise (VTOL, HYBRID pad phases) compensate.
            -- Follows the same eager-strafe handoff: where bank is no longer flying the lateral
            -- correction (strEager -> 1) it is no longer a bank to protect either, so the spill
            -- goes back to being cancelled -- otherwise the residual lean would keep pushing the
            -- craft sideways with nothing left commanded to oppose it.
            strafeCmd = strafeCmd
                + vertEffort * ux * CFG.VERT_XFER_STR * (bankSteer and strEager or 1)
            ui.tiltUp = uy
            -- A SCRAMBLE IS NOT SCALED BY TILT -- ONLY ITS SIGN IS ROUTED (2026-08-29).
            -- Everything else on this channel is a RATE, and a rate must be scaled by uy: to
            -- hold a world-up climb of 2 m/s from a banked craft you spend exactly the
            -- body-up share and hand the rest to fore/aft and strafe. A scramble is not a
            -- rate, it is the lift engine WIDE OPEN, and scaling it by uy quietly made it an
            -- attitude-dependent command: 0.87 of full at 30 deg, half at 60, and at 90 deg
            -- exactly ZERO -- both lines back at ALT_NEUTRAL, i.e. the detent doing nothing
            -- at all in the one attitude a pilot is most likely to be holding it down in.
            -- Inverted it did the arithmetic correctly and asked the DOWN bank for full, but
            -- everywhere in between it asked for less lift the more the craft needed it.
            --
            -- So the magnitude is pinned at full authority and only the SIGN of uy is kept:
            -- which bank is pointing away from the ground still decides which one burns, so
            -- an inverted craft still scrambles on the down bank rather than flying itself
            -- into the terrain. sendAlt's openThrottle then puts the opposing bank at
            -- ALT_MIN, so the pair is ALT_MAX / ALT_MIN at EVERY attitude.
            --
            -- Near uy = 0 (knife-edge) the sign is not meaningful and will flip. That is
            -- deliberately left alone rather than damped: at 90 deg of tilt BOTH banks fire
            -- horizontally, so neither choice buys or costs any climb, and the attitude loop
            -- is already rolling the craft out of it. Damping the flip would only add state.
            if ui.scramble then altSteps = (uy < 0) and -vertEffort or vertEffort end
        end
        if CFG.TRI_ENABLE then
            -- VECTOR-NOZZLE AIRFRAME: the three nozzles ARE the lift engine, so there is no
            -- up/down thruster pair for this channel to drive -- altSteps is spent below by
            -- sendVector, together with every other axis. Broadcasting CFG.ALT_PROTOCOL as
            -- well would put a confident-looking number on a protocol nothing on this craft
            -- listens to, which is exactly the sort of thing that gets believed a year later.
            -- The display's u/d pair is pinned at neutral and hidden (see displayLoop).
            altUp, altDown = CFG.ALT_NEUTRAL / RS_MAX, CFG.ALT_NEUTRAL / RS_MAX
        else
            altUp, altDown = sendAlt((CFG.ENABLE_ALTITUDE and CFG.HAS_ALT_THRUSTER) and altSteps or 0,
                ui.altCut, ui.scramble)
        end

        -- ---- OUTPUT ----
        -- FIXEDWING: strafe thrusters fully disabled -- drop course-correction AND the
        -- tilt-allocation lateral spill above, and hold the strafe integral at zero so
        -- it cannot wind up while its output is suppressed. Lateral course is then
        -- corrected only by turning (bank + yaw), and the alt thruster carries whatever
        -- vertical effort the banked craft can't route laterally (may sag in hard banks).
        if not CFG.STRAFE_ENABLED then strafeCmd = 0; strafePID.integral = 0 end
        -- ONE call carrying EVERY axis on a vector-nozzle airframe. It has to be one call:
        -- pitch, roll, yaw, strafe and lift all land on the same three engines, so they can
        -- only be divided between them together (see sendVector's header). It sits HERE, at
        -- the bottom of the output block rather than beside the vertical channel, because
        -- strafeCmd is not final until the FIXEDWING zeroing on the line above.
        if CFG.TRI_ENABLE then
            sendVector(CFG.ENABLE_ATTITUDE and pitchSteps or 0,
                CFG.ENABLE_ATTITUDE and rollSteps or 0,
                yawSteps, strafeCmd, thrustCmd,
                (CFG.ENABLE_ALTITUDE and CFG.HAS_ALT_THRUSTER) and altSteps or 0,
                ui.altCut)
        elseif CFG.ENABLE_ATTITUDE then sendStab(pitchSteps, rollSteps)
        else sendStab(0, 0) end
        sendNav(thrustCmd, yawSteps, strafeCmd)

        -- ---- publish for display ----
        local age = apLastMsg and (now - apLastMsg)
        ui.link = (not age) and "no link" or (age > CFG.AP_STALE_AFTER and string.format("STALE %ds", math.floor(age)) or "ok")
        ui.posX, ui.posY, ui.posZ = pos.x, latestPosY, pos.z
        -- ui.pitch / ui.roll were set at the top of the tick, from the single toEuler; see
        -- the note there. Two more toEuler calls used to live on this line.
        ui.headingDeg = math.deg(wrapPi(heading))
        ui.desHeadingDeg = math.deg(wrapPi(desiredHeading))
        ui.yawRate = w.y
        ui.phase = navActive and (hasTarget and navPhase or "NO TGT")
            or ((kjPitch or kjRoll) and (kj.active and "KJOY" or "KJOY-HLD"))
            or (manNavLive and (manNavActive and "MANUAL" or "MAN-HOLD") or "HANDOFF")
        -- The scheduled speed exactly as it arrived, for the recorder: -1 means the FMS sent
        -- no schedule this tick (no schedSpeed), which is a different thing from a scheduled
        -- zero and must not be allowed to look like one in the log.
        ui.navCap = navCruiseCapDynamic and navCruiseCap or -1
        ui.kj, ui.kjThr = (kjPitch or kjRoll), kj.thr
        ui.kjAxes = kjFresh and ((kjPitch and "P" or "-") .. (kjRoll and "R" or "-")
            .. (kjThr and "T" or "-")) or nil
        ui.apNav, ui.apAlt, ui.altManual = AP_NAV, AP_ALT, (not useAltCascade)
        ui.dist, ui.bearingDeg = dist, math.deg(wrapPi(bearing))
        -- Speed diagnostics: CMD = cap the FMS actually sent us, CLO = closing (velocity TOWARD the
        -- target -- what the along-track PID controls), GS = true horizontal ground speed. If CMD is
        -- 20-30 but the craft stays fast: CLO high + THR not going negative => PID/thruster (can't
        -- brake); CLO low but GS high => speed is tangential (loop happy, craft still fast).
        ui.cmdCap, ui.closing, ui.gs = navCruiseCap, closing, speedH
        ui.xtk = hasLeg and (CFG.XTK_SIGN * xtk) or nil
        ui.targetAlt, ui.velY, ui.desVelY = TARGET_ALT, velY, desiredVelY
        ui.altUp, ui.altDown = altUp, altDown
        ui.pStep, ui.rStep, ui.yStep = outStab, outNav.yaw, nil
        ui.wx, ui.wz, ui.vertEff = w.x, w.z, vertEffort   -- for the flight data recorder
        -- The learnt hover trim, in alt steps, published for the same recorder. It is the
        -- other half of reading a scramble: the detent commands CFG.ALT_SWING outright, so
        -- what it is WORTH over the hover the loop was already holding is ALT_SWING - this.
        ui.velInt = velPID.integral
        -- MEASURED loop period. sleep(0.05) asks for 20 Hz; what the computer actually
        -- achieves is another matter, and every gain in uacconf assumes it got what it asked
        -- for. A slow tick is dead time, and dead time is what destabilises these loops -- so
        -- this belongs on the screen permanently, not in a debug build. Rolling mean, on ui
        -- rather than a local (Cobalt's 200-locals ceiling; see the CFG loader's note).
        ui.dtAvg = (ui.dtAvg or dt) * 0.9 + dt * 0.1
        ui.thr, ui.yaw, ui.str = outNav.thrust, outNav.yaw, outNav.str or outNav.strafe
        ui.alState = alState
        ui.tgtName = navTarget[4]
        ui.bankCmd, ui.pitchCmd = bankCmd, pitchCmd
        ui.bankSteer = bankSteer
        lastLatDemand = latDemand   -- feed next tick's cross-track bank

        -- LAST thing in the tick: every command is final and ui is published, so the row is a
        -- coherent snapshot rather than a mix of this tick and the last.
        LOG.tick(now, pitchSteps, rollSteps, yawSteps, altSteps)

        -- ---- PACING: sleep to the DEADLINE, not for a fixed 0.05 on top of the work ----
        --
        -- THE MEASUREMENT THIS COMES FROM. Scout's flight recorder logged 660 consecutive
        -- ticks. Every single measured period was an exact multiple of one game tick:
        --     0.10 s 0.2% | 0.15 s 6.7% | 0.20 s 41.7% | 0.25 s 41.4% | 0.30 s 10.2%
        -- and nothing in between, on any of three separate flights. That distribution is the
        -- whole diagnosis. Lua arithmetic running long produces a CONTINUOUS spread of
        -- periods; a spread with no fractional part anywhere is a loop that is not compute
        -- bound at all but BLOCKING, a whole tick at a time. The things in here that can block
        -- are the calls the server has to serve on its main thread -- and the shipped build
        -- made ten per iteration: five `sublevel` reads, four `rednet.broadcast`es, and this
        -- sleep. (pid/uacprof.lua measures what each one is worth in ticks. Run it before
        -- reaching for any of this.)
        --
        -- Given that, an unconditional sleep(0.05) is a bug and not a pace. `sleep` cannot
        -- return sooner than the NEXT tick boundary, so a loop that has already spent four
        -- ticks blocking on its own calls asks for -- and gets -- a fifth. It was not waiting
        -- to hit 20 Hz; it was already five times past 20 Hz and adding to the overrun. The
        -- period slots at 0.20/0.25 above are that: four or five ticks of work, plus this.
        --
        -- So: sleep only if there is time left before the deadline, and if there is not,
        -- yield through the event queue instead. os.queueEvent + os.pullEvent is a real yield
        -- -- other coroutines run, the watchdog is satisfied -- but unlike a timer it can
        -- resume within the SAME tick, so an already-late iteration starts the next one at
        -- once instead of standing down for another 50 ms. When the loop is healthy and
        -- finishes inside its budget this branch never runs and the behaviour is the old
        -- behaviour exactly.
        --
        -- dt is measured, never assumed, everywhere in this file, so a period that varies
        -- with load stays correct -- and the whole point is that it varies DOWNWARD.
        local spent = os.clock() - now
        if spent < CFG.LOOP_PERIOD then
            sleep(CFG.LOOP_PERIOD - spent)
        else
            os.queueEvent("uac_tick")
            os.pullEvent("uac_tick")
        end
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
                navCruiseCapDynamic = false
                navCarrot, navGlide = false, false
                navAlign = false
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
                    navCruiseCapDynamic = msg.schedSpeed and true or false
                    navCarrot = msg.carrot and true or false
                    navGlide  = msg.glide  and true or false
                    navAlign  = msg.align  and true or false
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

-- Pilot AP toggle switches (#12). Latching switches streamed continuously; hold the last
-- value between packets so a dropped one cannot disengage the autopilot.

function switchApnav()
    while true do
        local sender, msg, __ = rednet.receive(CFG.APSWITCH_PROTOCOL..'_1', 1)
        if sender ~= nil then
            AP_NAV = msg.autopilot_nav
        end
    end
end

function switchApalt()
    while true do
        local sender, msg, __ = rednet.receive(CFG.APSWITCH_PROTOCOL..'_2', 1)
        if sender ~= nil then
            AP_ALT = msg.autopilot_altitude
        end
    end
end

-- Manual vertical from the VSC (#6): the pilot's wanted climb rate (m/s), plus the scramble
-- flag that rides beside it. Held between packets; staleness (and the rate clamp) handled by
-- the control loop.
local function manConReceiveLoop()
    while true do
        local sender, msg = rednet.receive(CFG.MANCON_PROTOCOL, 1)
        if sender and type(msg) == "table" then
            local v = tonumber(msg.vs)
            if v then
                manVS = v
                -- Latched INSIDE the same freshness gate as the rate, deliberately: a VSC that
                -- goes quiet mid-scramble -- crashed program, pulled modem, chunk unloaded --
                -- then ages out on MANCON_STALE_AFTER exactly as manVS does, instead of leaving
                -- the lift engine pinned wide open with nobody at the stick.
                manScramble = (msg.scramble == true)
                manLastMsg = os.clock()
            end
        end
    end
end

-- Manual horizontal + yaw from the MIC (#14): wanted body velocities (fwd/strafe m/s, yaw
-- rad/s) + an "active" flag. Held between packets; the control loop gates on freshness and
-- the AP-NAV switch. A distinct protocol from the vertical mancon so this can own a loop
-- (two rednet.receive loops cannot share one protocol filter).
local function manNavReceiveLoop()
    while true do
        local sender, msg = rednet.receive(CFG.MANNAV_PROTOCOL, 1)
        if sender and type(msg) == "table" then
            manFwd    = tonumber(msg.fwd)    or 0
            manStrafe = tonumber(msg.strafe) or 0
            manYaw    = tonumber(msg.yaw)    or 0
            manNavActive  = msg.active and true or false
            manNavLastMsg = os.clock()
        end
    end
end

-- Keyboard joystick (#21): the pilot's virtual stick + throttle quadrant. A TARGET
-- attitude, not a rate -- the stick is trim-style and holds where the pilot left it, so
-- these values are meant to persist between packets. Its own loop for the same reason every
-- other input has one: two rednet.receive loops cannot share a protocol filter.
local function kbdJoyReceiveLoop()
    while true do
        local sender, msg = rednet.receive(CFG.KBDJOY_PROTOCOL, 1)
        if sender and type(msg) == "table" then
            kj.pitch  = tonumber(msg.pitch) or 0
            kj.roll   = tonumber(msg.roll)  or 0
            kj.thr    = clamp(tonumber(msg.throttle) or 0, 0, 1)
            kj.active = msg.active and true or false
            kj.last   = os.clock()
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
                alState, alGrounded = "normal", false
            elseif cmd == "release" then
                -- Pilot overrode the descent mid-landing: hand the VERTICAL channel back
                -- to ALT-AP (same as cancel for us) while the FMS keeps streaming the
                -- LANDING navap so position/heading alignment stays online.
                alState, alGrounded = "normal", false
            elseif cmd == "hold" then
                -- "hold" means hold WHERE WE ARE NOW, so the altitude has to be seized on
                -- entry from ANY other state -- not just from "normal". Entering it from
                -- "descend" (a runway approach levelling off at its approach minimum, or a
                -- missed approach) used to leave TARGET_ALT at the stale ALT-AP selection, so
                -- the aircraft would stop descending and then climb all the way back to cruise.
                -- Guarded on ~= "hold" so repeated holds don't re-seize and ratchet on noise.
                -- msg.alt overrides all of that with an altitude to hold OUTRIGHT. The FMS sends
                -- it when it is station-keeping on a "helipad" that turned out to be a published
                -- point in open air (its HOVER phase): the number to sit on is landing.db's, not
                -- the metre of slop the descent happened to stop at, so it says so rather than
                -- letting us seize. Re-applied on every packet -- it is a constant the FMS
                -- repeats, so there is nothing here to ratchet either.
                local alt = tonumber(msg.alt)
                if alt then setTarget(alt)
                elseif alState ~= "hold" and latestPosY then setTarget(latestPosY) end
                alGrounded = false
                alState = "hold"
            elseif cmd == "descend" then
                local vs = tonumber(msg.vs)
                if vs then alDescendVS = clamp(vs, -CFG.AL_DESC_MAX, CFG.AL_CLIMB_MAX) end
                alGrounded = false
                alState = "descend"
            elseif cmd == "landed" then
                -- msg.squat = the FMS has the gear's weight-on-wheels contact and it is MADE.
                -- That is ground truth and it outranks the altimeter: see the vertical section,
                -- where it stops the lift thrusters ever being commanded UP on the ground.
                -- Without it we have only the altimeter, so hold the touchdown altitude.
                if alState ~= "landed" and latestPosY then setTarget(latestPosY) end
                alGrounded = msg.squat and true or false
                alState = "landed"
            end
            alLastMsg = os.clock()
        end
    end
end

-- DISPLAY: 2D artificial horizon + readouts ("numbers whizzing by is not
-- debuggable" -- draft). The horizon shows ACTUAL pitch/roll; the fixed centre
-- crosshair is the aircraft, so level = horizon through the middle.
-- THE HORIZON IS DRAWN IN RUNS, NOT IN CELLS (2026-08-28).
--
-- What it used to cost: one setCursorPos + one setBackgroundColour + one write PER CELL, so
-- width x AH_HEIGHT triples -- 51 x 9 = 459 cells, ~1400 terminal calls, every frame. On Aer
-- Togekiss, which had no CFG.DISPLAY_PERIOD and so ran the display at the full 20 Hz, that was
-- ~27,500 terminal calls a second for a picture no eye can follow faster than about five.
--
-- What it costs now: two calls per ROW. Each row of this picture is at most three horizontal
-- runs -- sky, the horizon cell(s) where the tilted line crosses, ground -- so it can be
-- built with string.rep and pushed out in a single blit. That is 18 terminal calls a frame
-- instead of 1400, and the arithmetic is unchanged (the same round(hy) per column, hoisted
-- into AH.hr so it is done once per frame rather than once per cell).
--
-- Rows identical to last frame are skipped outright, which is most of them most of the time:
-- straight and level, only the rows the horizon actually crosses change. Same technique, and
-- the same reason, as the PFD's row skipping in altcom.lua -- see the pseudo-bitmap section
-- of CLAUDE.md. This is a CELL-resolution picture, though, not a 2x3 subpixel one, so there
-- is no SW/complement business here: a cell is one colour and the runs are exact.
--
-- One table, not seven top-level locals (Cobalt's 200-locals cap, as everywhere else here).
local AH = { hr = {}, prev = {}, w = 0 }
do
    -- blit's colour digits are ordered BY BIT, so white is "0" and black is "f". Computed,
    -- not typed and not taken from colours.toBlit, so the mapping cannot drift with the
    -- installed CC version -- the house rule for every blit user in this tree.
    local function digit(c)
        local n, i = c, 0
        while n > 1 do n = n / 2; i = i + 1 end
        return ("0123456789abcdef"):sub(i + 1, i + 1)
    end
    AH.SKY, AH.GND, AH.HOR = digit(colours.blue), digit(colours.brown), digit(colours.white)
    AH.BLK, AH.OVL = digit(colours.black), digit(colours.orange)
end

local function drawAH(pitch, roll)
    local W = term.getSize()
    local top, h = CFG.AH_TOP, CFG.AH_HEIGHT
    local midY = top + math.floor(h / 2)
    local midX = math.floor(W / 2) + 1
    local slope = math.tan(clamp(roll, -1.2, 1.2))
    local pitchOff = clamp(pitch * CFG.PITCH_PIXELS_PER_RAD, -h, h)
    -- A terminal resize invalidates every cached row: the strings are the wrong length.
    if W ~= AH.w then AH.w, AH.prev = W, {} end

    -- Where the horizon sits in each column. Once per frame -- this is the arithmetic the
    -- old version repeated for every row of every column.
    local hr = AH.hr
    for col = 1, W do
        hr[col] = round(midY + pitchOff - (col - midX) * slope)
    end

    local text = (" "):rep(W)
    local fg   = AH.BLK:rep(W)
    for row = top, top + h - 1 do
        -- Walk the columns and emit a run wherever the band changes. At most three runs, so
        -- this concatenates at most three pieces; the loop itself is pure arithmetic.
        local parts, n, cls, start = {}, 0, nil, 1
        for col = 1, W + 1 do
            local c
            if col <= W then
                local hrow = hr[col]
                c = (row < hrow and AH.SKY) or (row > hrow and AH.GND) or AH.HOR
            end
            if c ~= cls then
                if cls then n = n + 1; parts[n] = cls:rep(col - start) end
                cls, start = c, col
            end
        end
        local bg = table.concat(parts)
        local tx, fgr = text, fg
        if row == midY then
            -- The fixed aircraft reference "-o-", patched into the row rather than written
            -- over it afterwards: three more terminal calls a frame otherwise, and patching
            -- keeps the cached string honest about what is actually on screen.
            local function put(x, ch)
                if x >= 1 and x <= W then
                    tx  = tx:sub(1, x - 1)  .. ch     .. tx:sub(x + 1)
                    fgr = fgr:sub(1, x - 1) .. AH.OVL .. fgr:sub(x + 1)
                    bg  = bg:sub(1, x - 1)  .. AH.BLK .. bg:sub(x + 1)
                end
            end
            put(midX - 2, "-"); put(midX, "o"); put(midX + 2, "-")
        end
        -- Skip the blit entirely if this row is byte-for-byte what is already on screen.
        local key = tx .. fgr .. bg
        if AH.prev[row] ~= key then
            AH.prev[row] = key
            term.setCursorPos(1, row)
            term.blit(tx, fgr, bg)
        end
    end
    term.setBackgroundColour(colours.black)
end

local function line(row, text, colour)
    term.setCursorPos(1, row); term.clearLine()
    if colour then term.setTextColour(colour) end
    term.write(text)
    if colour then term.setTextColour(colours.white) end
end

local function displayLoop2()
    while true do
        line(1, string.format("TICK %.0fms %.1fHz",
            (ui.dtAvg or 0.05) * 1000, 1 / (ui.dtAvg or 0.05)),
            ((ui.dtAvg or 0.05) > 0.125 and colours.red)
            or ((ui.dtAvg or 0.05) > 0.067 and colours.orange) or colours.white)
        sleep(0.05)
    end
end

local function displayLoop()
    while true do
        drawAH(ui.pitch or 0, ui.roll or 0)
        term.setTextColour(colours.white)
        local r = CFG.AH_TOP + CFG.AH_HEIGHT
        -- FLIGHT DATA RECORDER status, drawn over the TOP row of the artificial horizon (the
        -- least informative row of it) and AFTER drawAH, so it survives the redraw. Every other
        -- row on this screen is spoken for; this costs the horizon one line of sky and only
        -- while CFG.LOG_ENABLE is set.
        if CFG.LOG_ENABLE then
            -- This banner takes row CFG.AH_TOP away from the horizon, so drop that row from
            -- drawAH's cache: without this the horizon row would be skipped as "unchanged"
            -- while the banner is actually what is on screen, and the sky would never come
            -- back if the recorder ever stopped drawing. Costs one blit a frame, and only
            -- while CFG.LOG_ENABLE is set.
            AH.prev[CFG.AH_TOP] = nil
            if LOG.state == "done" then
                -- The whole point of the exercise: an unmissable, PERSISTENT "stop flying and
                -- go and read it". Inverse video because a coloured line among coloured lines
                -- is exactly what gets missed.
                term.setCursorPos(1, CFG.AH_TOP); term.clearLine()
                term.setBackgroundColour(colours.green); term.setTextColour(colours.black)
                term.write(string.format(" LOG DONE  %d rows  -> %s ", LOG.n, CFG.LOG_FILE))
                term.setBackgroundColour(colours.black); term.setTextColour(colours.white)
            elseif LOG.state == "fail" then
                term.setCursorPos(1, CFG.AH_TOP); term.clearLine()
                term.setBackgroundColour(colours.red); term.setTextColour(colours.white)
                term.write(" LOG FAILED: " .. tostring(LOG.why) .. " ")
                term.setBackgroundColour(colours.black); term.setTextColour(colours.white)
            elseif LOG.state == "wait" then
                line(CFG.AH_TOP, string.format("REC starts in %.1fs", LOG.wait or 0), colours.orange)
            elseif LOG.state == "rec" then
                line(CFG.AH_TOP, string.format("REC %3.0f%%  %d rows",
                    (LOG.pct or 0) * 100, LOG.n), colours.orange)
            end
        end
        line(r + 0, string.format("P%+5.1f R%+5.1f [%s/%s %s]",
            math.deg(ui.pitch or 0), math.deg(ui.roll or 0), CFG.ATT_MODE, CFG.SENSOR_FRAME, quatCtorName))
        line(r + 1, string.format("HDG %+6.1f -> %+6.1f  YR %+5.2f BY%+5.2f",
            ui.headingDeg or 0, ui.desHeadingDeg or 0, ui.yawRate or 0, ui.bankYawRate or 0))
        if CFG.TRI_ENABLE then
            -- <key><gimbal X><gimbal Y>/<thrust> per engine, in the order CFG.TRI_ENGINES
            -- lists them, then the allocator's own health -- the four things that can go
            -- wrong with it, in the order they matter:
            --   CUT  = lift engine off (touchdown cut), all three at a flat zero
            --   S<n> = the pitch/roll differential did NOT fit in 0..15 and was scaled to n%.
            --          This is the readout to watch: it means attitude authority is being
            --          rationed, and sitting below ~80% in ordinary flight says the
            --          TRI_*_PER_STEP authorities are asking for more than three engines can
            --          pay -- lower them rather than reaching for a gain.
            --   MAX  = an engine is hard against the 15 ceiling
            --   CONE = a nozzle wanted more deflection than TRI_VEC_MAX_ANGLE allows, so part
            --          of the commanded strafe/yaw was dropped rather than delivered
            local vt = (TRI.cut and "CUT")
                or (TRI.sat < 0.999 and string.format("S%d%%", math.floor(TRI.sat * 100)))
                or (TRI.hot > 0 and "MAX") or (TRI.cone > 0 and "CONE") or ""
            -- Y is the COMMANDED yaw, kept from the STAB line this replaces: the
            -- nozzle X columns show yaw indirectly (front one way, back pair the other) but
            -- not how much was asked for, which is the half needed to tune it.
            line(r + 2, string.format("%s%+3.0f%+3.0f/%2.0f %s%+3.0f%+3.0f/%2.0f %s%+3.0f%+3.0f/%2.0f Y%+.2f %s",
                TRI.eng[1].key, TRI.out[1].x, TRI.out[1].y, TRI.out[1].thrust,
                TRI.eng[2].key, TRI.out[2].x, TRI.out[2].y, TRI.out[2].thrust,
                TRI.eng[3].key, TRI.out[3].x, TRI.out[3].y, TRI.out[3].thrust,
                ui.yaw or 0, vt))
        else
            -- PITCH/ROLL/YAW as COMMANDED, in the units that now go on the wire: a fraction
            -- of one full redstone line. The old F/B/L/R quartet is gone from this screen
            -- because it is gone from this computer -- which bank a demand lands on, and what
            -- whole step it rounds to, is the relay's answer to give (see the OUTPUT header).
            line(r + 2, string.format("STAB P%+.2f R%+.2f Y%+.2f raw%.0f",
                (ui.pStep or outStab).pitch or 0, (ui.pStep or outStab).roll or 0,
                ui.yaw or 0, ui.rotRaw or 0))
        end
        line(r + 7, string.format("TRIM iP%+.2f iR%+.2f yS%.2f gy%.2f %s",
            ui.integP or 0, ui.integR or 0, ui.yawScale or 1, ui.gyroFade or 0,
            (ui.idle and "PARK " or "") .. (ui.yawBrake and "YBRK" or "")
            .. (ui.smith and string.format(" SMI%+.2f", ui.smith) or "")))
        -- TICK is the loop's MEASURED period and rate. It should read 50ms / 20.0Hz. Anything
        -- slower is dead time the tuning does not know about, and the number to fix before
        -- touching a gain. Amber below 15 Hz, red below 8 Hz.
        line(r + 8, string.format("I(k) x%.0f y%.0f z%.0f  TICK %.0fms %.1fHz",
            (ui.Ix or 0)/1000, (ui.Iy or 0)/1000, (ui.Iz or 0)/1000,
            (ui.dtAvg or 0.05) * 1000, 1 / (ui.dtAvg or 0.05)),
            ((ui.dtAvg or 0.05) > 0.125 and colours.red)
            or ((ui.dtAvg or 0.05) > 0.067 and colours.orange) or colours.white)
        -- FIXEDWING off; HYBRID banking=spar, pad=on. "eagr" = the eager-strafe handoff has
        -- taken the lateral correction off the bank (see CFG.STRAFE_EAGER_NEAR) -- it shows on
        -- FIXEDWING's "off" too, since there the handoff still silences the BANK half.
        local strTag = (ui.bankSteer and (ui.strEager or 0) > 0.5 and "eagr")
            or (not CFG.STRAFE_ENABLED and "off")
            or (ui.bankSteer and "spar") or "on"
        -- MODE line: the mode tag is a CLICKABLE BUTTON (highlighted) that cycles the flight
        -- mode -- inputLoop reads the bounds we publish here as ui.modeBtn. Tap to change.
        term.setCursorPos(1, r + 9); term.clearLine()
        term.setTextColour(colours.white); term.write("MODE ")
        local btnText = " " .. CFG.MODE_TAG .. " "
        local bx1 = 6                              -- first column of the button ("MODE " = 5)
        term.setBackgroundColour(colours.orange); term.setTextColour(colours.black)
        term.write(btnText)
        ui.modeBtn = { x1 = bx1, x2 = bx1 + #btnText - 1, y = r + 9 }
        term.setBackgroundColour(colours.black); term.setTextColour(colours.white)
        -- KJ<pct> = the keyboard joystick (#21) owns the attitude setpoint + throttle; the
        -- bank/pit figures beside it are then the STICK's commands, not the autopilot's.
        -- "FW" = FIXEDWING vertical-by-pitch owns the pitch setpoint. The pit pair is
        -- COMMANDED/MEASURED in degrees, both in the attitude loop's OWN convention (see
        -- ui.pitchMeas), so they must CONVERGE. Equal and opposite = the pitch sign is inverted;
        -- command large with measured stuck short = not enough authority to reach the attitude.
        term.write(string.format(" STR:%-4s bank%+.2f pit%+5.1f/%+5.1f%s%s", strTag, ui.bankCmd or 0,
            math.deg(ui.pitchApplied or 0), math.deg(ui.pitchMeas or 0),
            ui.fwPitch and "FW" or "",
            -- KJ<axes><throttle%>: which axes the stick actually holds right now, as P/R/T with
            -- a dash where an autopilot has taken that axis. "KJ--T" = ALT-AP has the pitch and
            -- AP-NAV is off; "KJ---" = both autopilots on, stick fully overshadowed. This is the
            -- readout that would have shown the pitch being vetoed immediately.
            ui.kjAxes and string.format(" KJ%s%d", ui.kjAxes, math.floor((ui.kjThr or 0) * 100 + 0.5)) or ""))
        term.setTextColour(colours.orange)
        line(r + 3, string.format("PH %-7s D%5.0f CMD%3.0f CLO%3.0f GS%3.0f",
            ui.phase or "--", ui.dist or 0, ui.cmdCap or 0, ui.closing or 0, ui.gs or 0))
        term.setTextColour(colours.white)
        if ui.xtk then
            line(r + 4, string.format("XTK %+5.1fm  THR%+.2f  STR%+.2f", ui.xtk, ui.thr or 0, ui.str or 0))
        else
            line(r + 4, string.format("XTK  ---     THR%+.2f  STR%+.2f", ui.thr or 0, ui.str or 0))
        end
        -- WHAT IS ACTUALLY CONTROLLING THE VERTICAL -- not just what TARGET_ALT happens to hold.
        -- The old version printed TARGET_ALT unconditionally, which is actively misleading in the
        -- two states where TARGET_ALT IS NOT IN CONTROL, and reads as "the UAC agrees with me"
        -- while the aeroplane ignores the setting:
        --   MAN  = altitude hold is OFF. The rate then comes from the vertical-speed mancon, and
        --          on an airframe with no VSC computer that means desiredVelY is PERMANENTLY 0 --
        --          i.e. "hold whatever altitude we are at", never the selected one.
        --   VS   = the FMS owns the rate channel (alState "descend"): TARGET_ALT is bypassed
        --          entirely and the aeroplane flies alDescendVS, whatever the panel says.
        --   GND  = on the gear, no altitude reference at all.
        --   CUT  = ...and the lift engine is off with it (CFG.AL_GROUND_CUT): both thruster
        --          lines at a flat 0, the aeroplane sitting on its own weight. Reads CUT for the
        --          whole of a landing, so seeing GND there means the cut is switched off.
        -- SCRM sits ahead of all of them: during a scramble none of the four below is what is
        -- flying the vertical, including the MAN the switches would otherwise show.
        local altTgt = ui.scramble and "SCRM"
            or ui.altManual and "MAN"
            or (ui.alState == "descend" and string.format("VS%+.1f", ui.desVelY or 0))
            or (ui.alState == "landed" and (ui.altCut and "CUT" or "GND"))
            or (ui.targetAlt and string.format("%.0f", ui.targetAlt) or "--")
        if CFG.TRI_ENABLE then
            -- No up/down pair to show here: the vertical IS the sum of the three nozzle
            -- thrusts, so L is that total (out of 45 on three 0..15 engines). t is still the
            -- body-up share of world-up, i.e. how much of the climb the tilt allocation is
            -- routing to lift rather than spilling into fore/aft and strafe.
            line(r + 5, string.format("ALT %6.1f -> %s  Vy%+5.2f L%2.0f t%+.2f",
                ui.posY or 0, altTgt, ui.velY or 0, TRI.tot, ui.tiltUp or 1))
        elseif CFG.HAS_ALT_THRUSTER then
            line(r + 5, string.format("ALT %6.1f -> %s  Vy%+5.2f u%.2f d%.2f t%+.2f",
                ui.posY or 0, altTgt, ui.velY or 0,
                ui.altUp or (CFG.ALT_NEUTRAL / RS_MAX), ui.altDown or (CFG.ALT_NEUTRAL / RS_MAX),
                ui.tiltUp or 1))
        else
            -- No lift thruster: the up/down levels are pinned at neutral and the tilt share is
            -- meaningless. Show what actually flies the vertical instead -- ACHIEVED vs COMMANDED
            -- climb rate, and the learnt flight-path-angle trim (iV). Tuning read: Vy not closing
            -- on the commanded figure at all = FW_PITCH_KP too low or FW_PITCH_MAX clipping;
            -- closing but hunting = KP too high; closing slowly with iV steadily growing = the
            -- integral is doing the feedforward's job, so FW_PITCH_FF is wrong.
            -- "SPD" = the speed protection is capping the commanded climb rate: the aeroplane
            -- is too slow to fly what the altitude loop asked for, so it is deliberately asking
            -- for less rather than raising the nose. Expected transiently in a hard climb; if it
            -- stays lit in level cruise the aeroplane is short of thrust, not short of gain.
            -- TRM is the trim in SCALE-INVARIANT units: <percent of FW_PITCH_I_LIMIT>/<degrees
            -- of commanded pitch it is contributing>. The old raw iV was unreadable because its
            -- units are whatever FW_PITCH_KI happens to be, and the same physical trim has read
            -- 0.7, 300 and 1000 across rescalings. Read it as:
            --   pct at +/-100        -> the integral is PEGGED; it is asking for something the
            --                           aeroplane is not delivering (windup, or a real ceiling)
            --   deg near FW_PITCH_MAX -> the TRIM is eating the whole command budget, so there is
            --                           no manoeuvring authority left -- raise FW_PITCH_MAX
            --   deg changing with SPEED is now CORRECT and expected: the trim is an angle of
            --                           attack and scales as 1/V^2 (see the gamma law).
            line(r + 5, string.format("ALT %6.1f -> %s  Vy%+5.2f/%+5.2f TRM%+4.0f%%%+5.1fd%s",
                ui.posY or 0, altTgt, ui.velY or 0, ui.desVelY or 0,
                ui.fwTrimPct or 0, ui.fwTrimDeg or 0,
                ui.fwVsCap and " SPD" or ""))
        end
        local apTag = string.format("AP nav:%s alt:%s  %s",
            ui.apNav and "on" or "off", ui.apAlt and "on" or "off", ui.link or "--")
        local al = (ui.alState ~= "normal") and ("AUTOLAND " .. string.upper(ui.alState or "")) or apTag
        term.setTextColour(ui.link == "ok" and colours.green or colours.orange)
        line(r + 6, al)
        term.setTextColour(colours.white)
        -- The instruments do NOT need 20 Hz, and on this fleet they cannot have it: every
        -- terminal write is main-thread work charged against the server-wide CC budget
        -- (max_main_global_time, 10 ms per tick shared by every computer), and a full redraw
        -- here is the artificial horizon plus ten lines -- around 20 operations, 400 a second.
        -- At 5 Hz that is 100 a second for a readout no eye can follow faster anyway.
        -- Optional knob: absent = the old 0.05, so this is a no-op on every other airframe.
        sleep(CFG.DISPLAY_PERIOD or 0.05)
    end
end

-- TERMINAL INPUT: the on-screen MODE button. A click (advanced-computer terminal) or a
-- monitor touch within the button bounds displayLoop published (ui.modeBtn) cycles the
-- flight mode VTOL -> HYBRID -> FIXEDWING -> VTOL and re-derives its flags. Levelling and
-- the slewed bank/strafe transitions absorb the switch, so it is safe to toggle in flight.
local function inputLoop()
    local order = { "VTOL", "HYBRID", "FIXEDWING" }
    while true do
        local ev, _, x, y = os.pullEvent()   -- mouse_click/monitor_touch both give (.,.,x,y)
        if ev == "mouse_click" or ev == "monitor_touch" then
            local b = ui.modeBtn
            if b and y == b.y and x >= b.x1 and x <= b.x2 then
                local idx = 1
                for i = 1, #order do if CFG.FLIGHT_MODE == order[i] then idx = i end end
                applyFlightMode(order[(idx % #order) + 1])
            end
        end
    end
end

-- Kill outputs on exit; hold the learnt altitude trim rather than a blind neutral.
local function shutdown()
    pcall(function()
        rednet.broadcast({ pitch = 0.0, roll = 0.0 }, CFG.STAB_PROTOCOL)
        rednet.broadcast({ thrust = 0.0, yaw = 0.0, strafe = 0.0 }, CFG.NAVCTRL_PROTOCOL)
        rednet.broadcast({ thrust = 0.0 }, CFG.THRUSTF_PROTOCOL)
        -- Vector-nozzle airframe: kill the three engines outright and centre the nozzles.
        -- There is no hover trim to preserve on this side -- unlike the up/down pair below,
        -- whose neutral is a balanced burn, a nozzle held at anything above zero is lift, and
        -- lift is the one thing a stopped controller must not leave running.
        if CFG.TRI_ENABLE then
            local m = { cut = true }
            for i = 1, #TRI.eng do m[TRI.eng[i].key] = { x = 0, y = 0, thrust = 0 } end
            rednet.broadcast(m, CFG.TRI_PROTOCOL)
        end
        -- ...and the up/down pair, held at the LEARNT hover trim rather than a blind neutral.
        -- Skipped entirely on a vector-nozzle airframe: the message above has already stopped
        -- the engines, and this one is a standing burn on a protocol nothing there listens to.
        if not CFG.TRI_ENABLE then
            local trimSteps = clamp(CFG.VEL_KI * velPID.integral, -CFG.ALT_SWING, CFG.ALT_SWING)
            -- ROUNDED HERE, and this is the one place in the file that rounds anything.
            -- A sigma-delta needs a STREAM to average back into the value asked for, and this
            -- is the last frame there will ever be: the relay holds whatever it is handed
            -- until something else commands it, so a fraction left mid-carry would park the
            -- craft a whole step off its learnt hover trim and leave it there. The UAC
            -- therefore picks the step itself and sends it as an exact multiple of one line,
            -- which the relay's quantiser passes through unchanged. It is still a float on
            -- the wire; it is just a float that lands on the grid.
            rednet.broadcast({
                up   = clamp(round(CFG.ALT_NEUTRAL + trimSteps), CFG.ALT_MIN, CFG.ALT_MAX) / RS_MAX,
                down = clamp(round(CFG.ALT_NEUTRAL - trimSteps), CFG.ALT_MIN, CFG.ALT_MAX) / RS_MAX,
                cut  = false,
            }, CFG.ALT_PROTOCOL)
        end
    end)
end

term.setBackgroundColour(colours.black); term.clear()
local ok, err = pcall(parallel.waitForAny,
    -- The note that used to sit here read "this loop alone drops tick from 20 Hz to 3 Hz
    -- (test env: all computers but this one has been shut down)". That observation was right
    -- and its implied cause -- the loop is too big -- was wrong, which is worth keeping on the
    -- record because it is the natural conclusion and it sends you the wrong way. Shutting the
    -- rest of the fleet down did not help because the cost was never contention for the
    -- server-wide budget: it was THIS loop's own ten blocking main-thread calls per iteration,
    -- each of which can only be answered on a tick boundary. See the pacing block at the
    -- bottom of controlLoop for the flight-recorder histogram that shows it, and
    -- pid/uacprof.lua for the per-call measurement.
    controlLoop,
    navReceiveLoop, altReceiveLoop, autolandReceiveLoop,
    switchApnav, switchApalt, manConReceiveLoop, manNavReceiveLoop, kbdJoyReceiveLoop,
    displayLoop,
    inputLoop
)
shutdown()
term.setBackgroundColour(colours.black); term.setTextColour(colours.white)
term.setCursorPos(1, term.getSize() and select(2, term.getSize()) or 19)
if not ok then print("UAC stopped: " .. tostring(err)) end
