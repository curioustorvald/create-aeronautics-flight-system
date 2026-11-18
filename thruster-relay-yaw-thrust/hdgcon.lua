if not sublevel then error("No sublevel API - is the Sable mod present?") end

local modem = peripheral.find("modem")
if not modem then error("No modem attached") end
rednet.open(peripheral.getName(modem))

-- WHAT ARRIVES ON THE NAVCTRL PROTOCOL IS A SIGNED FLOAT PER AXIS, NOT REDSTONE LEVELS.
-- `thrust`, `yaw` and `strafe` are each -1..+1: the fraction of one full redstone line the
-- UAC's control law asked for. Turning that into a whole 0..15 level on the correct side is
-- THIS computer's job, and has been since 2026-08-26. Before that the UAC rounded and
-- dithered before broadcasting, which meant every relay was handed a number whose fraction
-- had already been thrown away -- fine while a relay was one line per axis, wrong the moment
-- one was not (#69 spends fore/aft over two bundles at three times the resolution, and had
-- to be given a private float protocol to get it). See uac.lua's OUTPUT header.
--
-- This computer reads THRUST and YAW; `strafe` on the same message is the strafe relay's.
--
-- SIGMA-DELTA, ONE ACCUMULATOR PER AXIS -- not per line. What is left below a whole step is
-- carried and paid out a frame or two later, and the airframe (far slower than 20 Hz)
-- integrates it back into the value that was asked for; residual error is at most half a
-- step. It has to be per AXIS, on the SIGNED value, because the two lines of an axis are one
-- push-pull actuator: accumulating each line on its own would leave the line that is going
-- quiet owing a debt it could only repay by firing AGAINST its opposite number, every time a
-- command crossed zero.
YAW_POS_SIDE    = 'right'   -- +yaw fires this line, -yaw fires YAW_NEG_SIDE
YAW_NEG_SIDE    = 'left'
THRUST_POS_SIDE = 'front'   -- +thrust (forward) fires this line, -thrust THRUST_NEG_SIDE
THRUST_NEG_SIDE = 'back'
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
local ditherYaw,    resetYaw    = makeDither()
local ditherThrust, resetThrust = makeDither()

-- Signed float -> the two line levels that make it (positive side first).
local function axisLines(f, dither)
    f = clamp(f, -1, 1) * STEP_MAX
    local steps
    if DITHER then steps = dither(f, -STEP_MAX, STEP_MAX)
    else steps = clamp(math.floor(f + 0.5), -STEP_MAX, STEP_MAX) end
    if steps >= 0 then return steps, 0 else return 0, -steps end
end

term.clear()

local function displayLine(row, text)
    term.setCursorPos(1, row)
    term.clearLine()
    term.write(text)
end

local alt, att, mst = false
-- Each handler OWNS ITS OWN `while true`, and parallel.waitForAny is called once, at the
-- bottom. It used to be the other way round -- one-shot handlers inside
-- `while true do parallel.waitForAny(...) end` -- which rebuilt both coroutines for every
-- message and killed the other mid-receive each time. #3, #13 and #15 were already written
-- this way; #9 and #10 were the two that were missed.
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

function headingThrust()
  while true do
    local sender, msg, __ = rednet.receive("aertogekiss_navcontrol", 1)
    local active = mst
    if sender ~= nil and type(msg) == "table" and num(msg.thrust) and num(msg.yaw) then
        local yP, yN, tP, tN = 0, 0, 0, 0
        if active then
            yP, yN = axisLines(msg.yaw,    ditherYaw)
            tP, tN = axisLines(msg.thrust, ditherThrust)
        else
            resetYaw() resetThrust()
        end
        redstone.setAnalogOutput(YAW_POS_SIDE,    yP)
        redstone.setAnalogOutput(YAW_NEG_SIDE,    yN)
        redstone.setAnalogOutput(THRUST_POS_SIDE, tP)
        redstone.setAnalogOutput(THRUST_NEG_SIDE, tN)
        displayLine(2, string.format("Yaw:    %+.3f -> %2d %2d  active: %s", msg.yaw, yP, yN, active))
        displayLine(3, string.format("Thrust: %+.3f -> %2d %2d  active: %s", msg.thrust, tP, tN, active))
    elseif sender ~= nil and type(msg) == "table" and num(msg.yaw_left) then
        -- TRANSITIONAL, and delete it once every UAC on the fleet runs the float build: a UAC
        -- that has not been rebooted since 2026-08-26 still sends the old message, already
        -- split and already whole. Pass it straight through. This branch is what makes the
        -- deployment order RELAYS FIRST, UAC LAST safe -- the other way round feeds `nil` to
        -- setAnalogOutput on whichever side is not firing and kills the relay outright.
        local yl = num(msg.yaw_left)        and clamp(math.floor(msg.yaw_left + 0.5), 0, 15) or 0
        local yr = num(msg.yaw_right)       and clamp(math.floor(msg.yaw_right + 0.5), 0, 15) or 0
        local tf = num(msg.thrust_forward)  and clamp(math.floor(msg.thrust_forward + 0.5), 0, 15) or 0
        local tb = num(msg.thrust_backward) and clamp(math.floor(msg.thrust_backward + 0.5), 0, 15) or 0
        if not active then yl, yr, tf, tb = 0, 0, 0, 0 end
        redstone.setAnalogOutput("left",  yl)
        redstone.setAnalogOutput("right", yr)
        redstone.setAnalogOutput("front", tf)
        redstone.setAnalogOutput("back",  tb)
        resetYaw() resetThrust()
        displayLine(2, string.format("Yaw:    LEGACY %2d %2d  active: %s", yl, yr, active))
        displayLine(3, string.format("Thrust: LEGACY %2d %2d  active: %s", tf, tb, active))
    else
        redstone.setAnalogOutput(YAW_POS_SIDE,    0)
        redstone.setAnalogOutput(YAW_NEG_SIDE,    0)
        redstone.setAnalogOutput(THRUST_POS_SIDE, 0)
        redstone.setAnalogOutput(THRUST_NEG_SIDE, 0)
        resetYaw() resetThrust()
        displayLine(2, "No nav signal received")
        displayLine(3, "")
    end
  end
end

displayLine(1, 'Aer Togekiss heading and thrust controller')
parallel.waitForAny(headingThrust, switchEngine)
