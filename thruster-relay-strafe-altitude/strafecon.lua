if not sublevel then error("No sublevel API - is the Sable mod present?") end

local modem = peripheral.find("modem")
if not modem then error("No modem attached") end
rednet.open(peripheral.getName(modem))

-- WHAT ARRIVES ON THESE TWO PROTOCOLS IS FLOATS, NOT REDSTONE LEVELS.
--
--   NAVCTRL  `strafe`      signed -1..+1 -- the fraction of one full redstone line the UAC's
--                          control law asked for. (`thrust` and `yaw` on the same message
--                          belong to the yaw/thrust relay, not to this computer.)
--   ALT      `up`, `down`  each 0..1, ALREADY SPLIT. The UAC still owns that split, because
--                          the balanced-hover point it is measured from (CFG.ALT_NEUTRAL) is
--                          airframe calibration and belongs in uacconf.lua with the rest of
--                          it. What the UAC no longer does is round it.
--   ALT      `cut`         the lift engine switched OFF, which is not the same thing as
--                          up = down = neutral (a balanced hover still burns on both lines).
--
-- Turning all of that into whole 0..15 levels is THIS computer's job, and has been since
-- 2026-08-26; before that the UAC rounded and dithered before broadcasting. See uac.lua's
-- OUTPUT header for why it moved.
--
-- SIGMA-DELTA. What is left below a whole step is carried and paid out a frame or two later,
-- and the airframe (far slower than 20 Hz) integrates it back into the value asked for;
-- residual error is at most half a step.
--
-- STRAFE gets ONE accumulator for the AXIS, not one per line: its two lines are a single
-- push-pull actuator, and accumulating each separately would leave the line going quiet
-- owing a debt it could only repay by firing AGAINST its opposite number every time the
-- command crossed zero. UP and DOWN get one EACH, because they are not a push-pull pair --
-- they are two independent burns either side of a hover trim, which is exactly how the UAC
-- dithered them before the move.
STRAFE_POS_SIDE = 'right'   -- +strafe fires this line, -strafe fires STRAFE_NEG_SIDE
STRAFE_NEG_SIDE = 'left'
STEP_MAX = 15               -- redstone ceiling on one line
DITHER   = true             -- false = plain rounding, i.e. throw the fraction away again

local function clamp(v, lo, hi) return math.max(lo, math.min(hi, v)) end
local function num(v) return type(v) == "number" and v == v end   -- present, and not NaN

-- One accumulator, and a reset for it. The reset is called whenever the channel is cut, so
-- no stale fraction is paid into the first frame after it comes back: the accumulator holds
-- at most half a step and nothing should be owed across an engine switch.
local function makeDither()
    local acc = 0
    return function(value, lo, hi)
        local want = value + acc
        local q = clamp(math.floor(want + 0.5), lo, hi)
        acc = clamp(want - q, -0.5, 0.5)
        return q
    end, function() acc = 0 end
end
local ditherStrafe, resetStrafe = makeDither()
local ditherUp,     resetUp     = makeDither()
local ditherDown,   resetDown   = makeDither()

-- Signed float -> the two line levels that make it (positive side first).
local function axisLines(f, dither)
    f = clamp(f, -1, 1) * STEP_MAX
    local steps
    if DITHER then steps = dither(f, -STEP_MAX, STEP_MAX)
    else steps = clamp(math.floor(f + 0.5), -STEP_MAX, STEP_MAX) end
    if steps >= 0 then return steps, 0 else return 0, -steps end
end

-- Unipolar float -> one line level.
local function lineLevel(f, dither)
    f = clamp(f, 0, 1) * STEP_MAX
    if DITHER then return dither(f, 0, STEP_MAX) end
    return clamp(math.floor(f + 0.5), 0, STEP_MAX)
end

term.clear()

-- Altitude thrusters, one redstone line each (the external splitter box is gone).
-- Rewire these to the sides your up- and down-firing thrusters actually sit on.
local UP_SIDE   = "front"
local DOWN_SIDE = "back"
local ALT_NEUTRAL = 7   -- balanced-hover level held on both lines when the link drops

local function displayLine(row, text)
    term.setCursorPos(1, row)
    term.clearLine()
    term.write(text)
end

local alt, att, mst = false
-- Each handler OWNS ITS OWN `while true`, and parallel.waitForAny is called once, at the
-- bottom. It used to be the other way round -- one-shot handlers inside
-- `while true do parallel.waitForAny(...) end` -- which rebuilt three coroutines for every
-- single message and killed two of them mid-receive each time. #3, #13 and #15 were already
-- written this way; #9 and #10 were the two that were missed.
function switchEngine()
  while true do
    local sender, msg, __ = rednet.receive("aertogekiss_engineswitch", 1)
    if sender ~= nil then
        alt = msg.altitude
        att = msg.attitude
        mst = msg.master
        displayLine(4, 'MST: '..tostring(mst)..'  ALT: '..tostring(alt)..'  ATT: '..tostring(att))
    else
        displayLine(4, "No engine switch signal received")
    end
  end
end

--[[local apnav, apalt = false
function switchAutopilot()
    local sender, msg, __ = rednet.receive("aertogekiss_apswitch", 1)
    if sender ~= nil then
        apnav = msg.autopilot_nav
        apalt = msg.autopilot_altitude
        displayLine(5, 'AP-NAV: '..tostring(apnav)..' AP-ALT: '..tostring(apalt))
    else
        displayLine(5, "No AP switch signal received")
    end
end]]

function strafe()
  while true do
    local sender, msg, __ = rednet.receive("aertogekiss_navcontrol", 1)
    local active = mst
    if sender ~= nil and type(msg) == "table" and num(msg.strafe) then
        local sP, sN = 0, 0
        if active then sP, sN = axisLines(msg.strafe, ditherStrafe) else resetStrafe() end
        redstone.setAnalogOutput(STRAFE_POS_SIDE, sP)
        redstone.setAnalogOutput(STRAFE_NEG_SIDE, sN)
        displayLine(2, string.format("Strafe: %+.3f -> %2d %2d  active: %s", msg.strafe, sP, sN, active))
    elseif sender ~= nil and type(msg) == "table" and num(msg.strafe_left) then
        -- TRANSITIONAL, and delete it once every UAC on the fleet runs the float build: a UAC
        -- that has not been rebooted since 2026-08-26 still sends the old message, already
        -- split and already whole. Pass it straight through. This branch is what makes the
        -- deployment order RELAYS FIRST, UAC LAST safe -- the other way round feeds `nil` to
        -- setAnalogOutput on whichever side is not firing and kills the relay outright.
        local sl = num(msg.strafe_left)  and clamp(math.floor(msg.strafe_left + 0.5), 0, 15) or 0
        local sr = num(msg.strafe_right) and clamp(math.floor(msg.strafe_right + 0.5), 0, 15) or 0
        if not active then sl, sr = 0, 0 end
        redstone.setAnalogOutput("left",  sl)
        redstone.setAnalogOutput("right", sr)
        resetStrafe()
        displayLine(2, string.format("Strafe: LEGACY %2d %2d  active: %s", sl, sr, active))
    else
        redstone.setAnalogOutput(STRAFE_POS_SIDE, 0)
        redstone.setAnalogOutput(STRAFE_NEG_SIDE, 0)
        resetStrafe()
        displayLine(2, "No strafe signal received")
    end
  end
end

function alti()
  while true do
    local sender, msg, __ = rednet.receive("aertogekiss_alt", 1)
    if sender ~= nil and type(msg) == "table" and num(msg.up) and num(msg.down)
            and type(msg.cut) == "boolean" then
        local active = alt and mst and not msg.cut
        local up, down = 0, 0
        if active then
            up, down = lineLevel(msg.up, ditherUp), lineLevel(msg.down, ditherDown)
        else
            -- Cut, or the engine switched off: bypass the accumulators rather than feed them.
            -- They hold at most half a step and nothing should be owed when the lift comes back.
            resetUp() resetDown()
        end
        redstone.setAnalogOutput(UP_SIDE, up)
        redstone.setAnalogOutput(DOWN_SIDE, down)
        displayLine(3, string.format("Alti: %.3f/%.3f -> up %2d down %2d  active: %s",
            msg.up, msg.down, up, down, tostring(active)))
    elseif sender ~= nil and type(msg) == "table" and num(msg.up) and num(msg.down) then
        -- TRANSITIONAL, and delete it once every UAC on the fleet runs the float build.
        -- ALT is the one protocol whose field NAMES did not change: an old UAC sends `up`
        -- and `down` too, but as whole 0..15 levels and with no `cut` flag. The flag is the
        -- only thing that tells the two apart, and getting it wrong is not a subtlety -- a
        -- legacy `up = 7` read as a fraction clamps to 1.0 and puts the lift line hard
        -- against 15 on an aeroplane that asked for a balanced hover.
        local active = alt and mst
        local up   = clamp(math.floor(msg.up + 0.5), 0, 15)
        local down = clamp(math.floor(msg.down + 0.5), 0, 15)
        if not active then up, down = 0, 0 end
        redstone.setAnalogOutput(UP_SIDE, up)
        redstone.setAnalogOutput(DOWN_SIDE, down)
        resetUp() resetDown()
        displayLine(3, string.format("Alti: LEGACY up %2d down %2d  active: %s",
            up, down, tostring(active)))
    else
        -- Link lost: hold a balanced hover rather than plunging. Same figure the UAC's
        -- CFG.ALT_NEUTRAL holds -- keep the two in step if either is retuned.
        if alt and mst then
            redstone.setAnalogOutput(UP_SIDE, ALT_NEUTRAL)
            redstone.setAnalogOutput(DOWN_SIDE, ALT_NEUTRAL)
        else
            redstone.setAnalogOutput(UP_SIDE, 0)
            redstone.setAnalogOutput(DOWN_SIDE, 0)
        end
        resetUp() resetDown()
        displayLine(3, string.format("No alti signal, holding %d/%d", ALT_NEUTRAL, ALT_NEUTRAL))
    end
  end
end

displayLine(1, 'Aer Togekiss altitude and strafe controller')
parallel.waitForAny(strafe, alti, switchEngine)
