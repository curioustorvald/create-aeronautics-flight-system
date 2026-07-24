-- MANCON INTENTION CONTROLLER
-- Converts pilot's manual control signal, treat is as 'intention',
-- and outputs PID-controlled fore/aft/strafe/yawing thruster output that best serves pilot's intention
-- (e.g. cancelling unwanted yawing, when yawing is not commanded)

-- Signal interpretation in layman's terms
-- fore/aft/strate ('aertogekiss_mancon_translational'):
--   key down: I want to move by this horizontal speed (the input is binary, internal accumulator slowly raises target)
--   key up: quickly fade the target to 0 m/s
-- yawing ('aertogekiss_mancon_rotational'):
--   key down: I want to move by this angular velocity (same integrator)
--   key up: quickly fade to 0 m/s

-- unlike altitude MANCON, nav MANCON (this thing) runs independently to the autopilot, meaning it can be used for both manual navigation and emergency course alteration

-- Computed output is "desired speed in m/s" that goes into the UAC via rednet

-- IMPLEMENTATION NOTES ------------------------------------------------------
-- This computer runs NO thruster control of its own. Like the vertical-speed
-- commander (#6), it only NAMES a wanted velocity; the UAC (#1) owns the PID that
-- turns that wanted velocity into attitude-aware thruster steps -- and, because a
-- released axis is commanded to exactly 0, the UAC actively holds heading / nulls
-- drift (that is the "cancel unwanted yawing when not commanded" above).
--
-- The two relays (#13 translation, #15 rotation) broadcast the pilot's raw key
-- state ~20x/s, but ONLY while manual control is live (master on, AP-NAV off); they
-- go silent otherwise. So a missing packet is read as "keys released": if a relay
-- has not been heard within STALE_AFTER, that axis's keys are treated as up and its
-- accumulator fades to 0. The last key state is otherwise held between packets.
--
-- OUTPUT protocol is DISTINCT from the vertical 'aertogekiss_mancon' (#6): the UAC
-- must be able to run a dedicated receive loop for this, and two loops cannot share
-- one protocol filter. Message: { fwd, strafe (m/s), yaw (rad/s), active }.

-- CONFIG --------------------------------------------------------------------
local TRANS_PROTOCOL = "aertogekiss_mancon_translational"  -- in  <- #13
local ROT_PROTOCOL   = "aertogekiss_mancon_rotational"     -- in  <- #15
local OUT_PROTOCOL   = "aertogekiss_mancon_nav"            -- out -> UAC (#1)

-- Full-stick target speeds (what an axis winds up to if a key is held long enough).
-- Body frame: fwd + = forward, strafe + = right, yaw + = nose-right (turn rate).
-- The UAC clamps/calibrates further (its own limits and *_SIGN), so these are just
-- the sensible manual ceilings; less than the autopilot cruise on purpose.
local V_FWD_MAX    = 50.0   -- m/s, full fore/aft target
local V_STRAFE_MAX = 20.0   -- m/s, full strafe target
local YAW_MAX      = 0.5    -- rad/s, full yaw-rate target (supposed to match the UAC's MAX_RATE_YAW, but 0.5 seems working well?)

-- Accumulator dynamics. "key down -> accumulator SLOWLY raises target" = RAMP; "key
-- up -> QUICKLY fade to 0" = FADE (much faster). A tap gives a small nudge; a long
-- hold builds up to the max. Rates are (target units) per second.
local RAMP_UP      = 15.0    -- m/s per s, fore/aft/strafe target growth while held
local FADE_DOWN    = 40.0   -- m/s per s, fore/aft/strafe fade to 0 on release
local YAW_RAMP_UP  = 0.8    -- rad/s per s, yaw target growth while held
local YAW_FADE     = 3.0    -- rad/s per s, yaw fade to 0 on release

local STALE_AFTER  = 0.3    -- s; a relay silent longer than this -> its keys read as up
local TICK         = 0.05   -- s between integrate/broadcast steps (matches the UAC tick)

-- SETUP ---------------------------------------------------------------------
-- Open whatever modem is attached (the relays and the UAC are all on rednet). The
-- terminal shows status; the modem carries the traffic.
local modem = peripheral.find("modem")
if modem then
    rednet.open(peripheral.getName(modem))
else
    printError("No modem attached; cannot relay " .. OUT_PROTOCOL)
end

local function displayLine(row, text)
    term.setCursorPos(1, row)
    term.clearLine()
    term.write(text)
end

local function clamp(val, lo, hi)
    return math.max(lo, math.min(hi, val))
end

local function f(pred, str)
    if pred then return str else return '---' end
end

-- INTENTION STATE (raw key state from the relays + when each was last heard) -----
-- Written by receiveLoop, read by commandLoop.
local inFore, inAft, inSleft, inSright = false, false, false, false
local inYleft, inYright = false, false
local transLastMsg, rotLastMsg = nil, nil

-- RECEIVE: one loop, no protocol filter, dispatch by the returned protocol. (Two
-- filtered loops would each pull -- and drop -- the other's messages.)
local function receiveLoop()
    while true do
        local sender, msg, proto = rednet.receive(nil, TICK)
        if sender and type(msg) == "table" then
            if proto == TRANS_PROTOCOL then
                inFore   = msg.fore   and true or false
                inAft    = msg.aft    and true or false
                inSleft  = msg.sleft  and true or false
                inSright = msg.sright and true or false
                transLastMsg = os.clock()
            elseif proto == ROT_PROTOCOL then
                inYleft  = msg.yleft  and true or false
                inYright = msg.yright and true or false
                rotLastMsg = os.clock()
            end
        end
    end
end

-- Move an accumulator toward its commanded goal. Holding a single direction drives
-- the goal to +/- maxVal at the (slow) ramp rate; anything else -- released, or both
-- opposing keys -- drives it to 0 at the (fast) fade rate.
local function integrate(cur, pos, neg, maxVal, rampRate, fadeRate, dt)
    local goal, rate
    if pos and not neg then
        goal, rate = maxVal, rampRate
    elseif neg and not pos then
        goal, rate = -maxVal, rampRate
    else
        goal, rate = 0, fadeRate
    end
    local step = rate * dt
    local d = goal - cur
    if d >  step then d =  step end
    if d < -step then d = -step end
    return cur + d
end

-- MAIN ----------------------------------------------------------------------
term.clear()
displayLine(1, "Aer Togekiss MANCON Intention Controller")

local function commandLoop()
    local lastTime = os.clock()
    local fwd, strafe, yaw = 0, 0, 0
    while true do
        local now = os.clock()
        local dt  = math.max(now - lastTime, 0.001)
        lastTime  = now

        -- Freshness gate: a silent relay means "no keys held" for that axis group, so
        -- a relay that drops out (AP engaged / master off) fades the craft to a stop
        -- instead of latching the last-seen keys.
        local transFresh = transLastMsg and (now - transLastMsg) <= STALE_AFTER
        local rotFresh   = rotLastMsg   and (now - rotLastMsg)   <= STALE_AFTER
        local fore   = (transFresh and inFore)   or false
        local aft    = (transFresh and inAft)    or false
        local sleft  = (transFresh and inSleft)  or false
        local sright = (transFresh and inSright) or false
        local yleft  = (rotFresh and inYleft)  or false
        local yright = (rotFresh and inYright) or false

        -- Integrate each pilot intention into a wanted velocity.
        fwd    = integrate(fwd,    fore,   aft,   V_FWD_MAX,    RAMP_UP,     FADE_DOWN, dt)
        strafe = integrate(strafe, sright, sleft, V_STRAFE_MAX, RAMP_UP,     FADE_DOWN, dt)
        yaw    = integrate(yaw,    yright, yleft, YAW_MAX,      YAW_RAMP_UP, YAW_FADE,  dt)

        -- Snap tiny residues to exactly 0 so a fully-faded axis commands a clean hold.
        if math.abs(fwd)    < 1e-3 then fwd    = 0 end
        if math.abs(strafe) < 1e-3 then strafe = 0 end
        if math.abs(yaw)    < 1e-4 then yaw    = 0 end

        local active = fore or aft or sleft or sright or yleft or yright

        rednet.broadcast({
            fwd    = fwd,      -- m/s,  body forward +, aft -
            strafe = strafe,   -- m/s,  body right +, left -
            yaw    = yaw,      -- rad/s, nose-right +, left -
            active = active,   -- any key currently held (helps the UAC hold vs. fly)
        }, OUT_PROTOCOL)

        displayLine(2, string.format('KEYS: %s %s %s %s | YAW %s %s',
            f(fore, 'FWD'), f(aft, 'AFT'), f(sleft, 'LFT'), f(sright, 'RGT'),
            f(yleft, 'L'), f(yright, 'R')))
        displayLine(3, string.format('CMD : fwd %+6.2f  str %+6.2f m/s', fwd, strafe))
        displayLine(4, string.format('      yaw %+5.2f rad/s   %s', yaw,
            active and '(commanding)' or (transFresh or rotFresh) and '(coasting)' or '(no signal)'))
        displayLine(5, '-> ' .. OUT_PROTOCOL)

        sleep(TICK)
    end
end

parallel.waitForAny(receiveLoop, commandLoop)
