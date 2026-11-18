if not sublevel then error("No sublevel API - is the Sable mod present?") end

local modem = peripheral.find("modem")
if not modem then error("No modem attached") end
rednet.open(peripheral.getName(modem))


-- WHAT ARRIVES ON THE STAB PROTOCOL IS A PAIR OF SIGNED FLOATS, NOT REDSTONE LEVELS.
-- `pitch` and `roll` are each -1..+1: the fraction of one full redstone line the UAC's
-- control law asked for. Turning that into a whole 0..15 level on the correct side is THIS
-- computer's job, and has been since 2026-08-26. Before that the UAC rounded and dithered
-- before broadcasting, which meant every relay was handed a number that had already had its
-- fraction thrown away -- fine while a relay was one line per axis, wrong the moment one was
-- not (#69 spends fore/aft over two bundles at three times the resolution, and had to be
-- given a private float protocol to get it). See uac.lua's OUTPUT header.
--
-- SIGMA-DELTA, ONE ACCUMULATOR PER AXIS -- not per line. What is left below a whole step is
-- carried and paid out a frame or two later, and the airframe (far slower than 20 Hz)
-- integrates it back into the value that was asked for; residual error is at most half a
-- step. It has to be per AXIS, on the SIGNED value, because the two lines of an axis are one
-- push-pull actuator: accumulating each line on its own would leave the line that is going
-- quiet owing a debt it could only repay by firing AGAINST its opposite number, every time a
-- command crossed zero.
PITCH_POS_SIDE = 'back'   -- +pitch fires this line, -pitch fires PITCH_NEG_SIDE
PITCH_NEG_SIDE = 'front'
ROLL_POS_SIDE  = 'left'   -- +roll fires this line, -roll fires ROLL_NEG_SIDE
ROLL_NEG_SIDE  = 'right'
STEP_MAX = 15              -- redstone ceiling on one line
DITHER   = true            -- false = plain rounding, i.e. throw the fraction away again

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
local ditherPitch, resetPitch = makeDither()
local ditherRoll,  resetRoll  = makeDither()

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

local apnav, apalt = false
--[[function switchAutopilot()
    local sender, msg, __ = rednet.receive("aertogekiss_apswitch", 1)
    if sender ~= nil then
        apnav = msg.autopilot_nav
        apalt = msg.autopilot_altitude
        displayLine(5, 'AP-NAV: '..tostring(apnav)..' AP-ALT: '..tostring(apalt))
    else
        displayLine(5, "No AP switch signal received")
    end
end]]

function switchApnav()
    while true do
        local sender, msg, __ = rednet.receive("aertogekiss_apswitch_1", 1)
        if sender ~= nil then
            apnav = msg.autopilot_nav
            displayLine(5, 'AP-NAV: '..tostring(apnav))
        else
            displayLine(5, "No APNAV signal received")
        end
    end
end

function switchApalt()
    while true do
        local sender, msg, __ = rednet.receive("aertogekiss_apswitch_2", 1)
        if sender ~= nil then
            apalt = msg.autopilot_altitude
            displayLine(6, 'AP-ALT: '..tostring(apalt))
        else
            displayLine(6, "No APALT signal received")
        end
    end
end

function stabil()
    while true do
        local sender, msg, __ = rednet.receive("aertogekiss_stab", 1)
        local active = false
        if sender ~= nil and type(msg) == "table" and num(msg.pitch) and num(msg.roll) then
            active = att and mst
            local pP, pN, rP, rN = 0, 0, 0, 0
            if active then
                pP, pN = axisLines(msg.pitch, ditherPitch)
                rP, rN = axisLines(msg.roll,  ditherRoll)
            else
                resetPitch() resetRoll()
            end
            redstone.setAnalogOutput(PITCH_POS_SIDE, pP)
            redstone.setAnalogOutput(PITCH_NEG_SIDE, pN)
            redstone.setAnalogOutput(ROLL_POS_SIDE,  rP)
            redstone.setAnalogOutput(ROLL_NEG_SIDE,  rN)
            displayLine(2, string.format("Stabil: P%+.3f R%+.3f -> %s%d %s%d %s%d %s%d",
                msg.pitch, msg.roll,
                PITCH_POS_SIDE:sub(1, 1):upper(), pP, PITCH_NEG_SIDE:sub(1, 1):upper(), pN,
                ROLL_POS_SIDE:sub(1, 1):upper(),  rP, ROLL_NEG_SIDE:sub(1, 1):upper(),  rN))
        elseif sender ~= nil and type(msg) == "table" and num(msg.front) then
            -- TRANSITIONAL, and delete it once every UAC on the fleet runs the float build:
            -- a UAC that has not been rebooted since 2026-08-26 still sends the old message,
            -- already split and already whole. Pass it straight through. This branch is what
            -- makes the deployment order RELAYS FIRST, UAC LAST safe -- the other way round
            -- feeds `nil` to setAnalogOutput on whichever side is not firing and kills the
            -- relay outright.
            active = att and mst
            local l = num(msg.left)  and clamp(math.floor(msg.left  + 0.5), 0, 15) or 0
            local r = num(msg.right) and clamp(math.floor(msg.right + 0.5), 0, 15) or 0
            local f = num(msg.front) and clamp(math.floor(msg.front + 0.5), 0, 15) or 0
            local b = num(msg.back)  and clamp(math.floor(msg.back  + 0.5), 0, 15) or 0
            if not active then l, r, f, b = 0, 0, 0, 0 end
            redstone.setAnalogOutput("left",  l)
            redstone.setAnalogOutput("right", r)
            redstone.setAnalogOutput("front", f)
            redstone.setAnalogOutput("back",  b)
            resetPitch() resetRoll()
            displayLine(2, string.format("Stabil: LEGACY %2d %2d %2d %2d", l, r, f, b))
        else
            redstone.setAnalogOutput(PITCH_POS_SIDE, 0)
            redstone.setAnalogOutput(PITCH_NEG_SIDE, 0)
            redstone.setAnalogOutput(ROLL_POS_SIDE,  0)
            redstone.setAnalogOutput(ROLL_NEG_SIDE,  0)
            resetPitch() resetRoll()
            displayLine(2, "No stabil signal received")
        end
        
        displayLine(3, string.format('active: %s', active))
    end
end


displayLine(1, 'Aer Togekiss stabiliser controller')
parallel.waitForAny(stabil, switchEngine, switchApnav, switchApalt)
