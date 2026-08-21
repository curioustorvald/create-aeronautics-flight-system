-- SCOUT -- FORE/AFT THRUST CONTROLLER
--
-- The one axis on Scout that is NOT made by the three vectoring engines. Pitch, roll, yaw,
-- strafe and lift all come out of the nozzles (see 72/pid/uac.lua's sendVector); fore/aft has
-- a thruster bank of its own, and this computer drives it.
--
-- IT LISTENS ON THE FLOAT CHANNEL, NOT THE INTEGER ONE. The UAC broadcasts fore/aft twice:
-- as a whole redstone step inside the NAVCTRL message, and as a signed float -1..+1 on
-- CFG.THRUSTF_PROTOCOL -- the demand as the control law actually computed it, before anything
-- rounded it. This computer takes the float, for the same reason 77/78/80 take floats: the
-- output allocation below is this computer's business, so the grid the demand should be
-- quantised onto is known here and nowhere else. Whatever is finer than one step is carried
-- in a sigma-delta accumulator and paid out a frame or two later, which the airframe (far
-- slower than 20 Hz) integrates back into the value that was asked for.
--
-- WIRING NOTE: the fore/aft thrusters must be wired to THIS computer and nowhere else. The
-- UAC pins NAVCTRL's yaw and strafe lines to zero on a vector-nozzle airframe, but it still
-- sends thrust_forward / thrust_backward there. If any thruster is still on those lines it
-- gets two commands and the finer one is wasted.
--
-- The OUTPUT ALLOCATION block is shared, line for line, with 77/78/80 vecengine.lua. Fix a
-- bug in one and fix it in all four.

PROTOCOL               = 'scout_thrustf'        -- in: UAC -> signed float, -1..+1
PROTOCOL_ENGINE_SWITCH = 'scout_engineswitch'   -- in: master/altitude/attitude switch (#71)

THRUST_SIGN = 1        -- flip to -1 if +thrust drives the aircraft backwards

-- ============================================================================
-- WIRING -- THE OUTPUT ALLOCATION. **THIS IS THE PART THAT NEEDS FILLING IN.**
-- ============================================================================
-- Left empty deliberately: the program boots, receives, quantises and displays with nothing
-- connected, so the numbers can be watched before a single line is run.
--
-- TWO CHANNELS come out of this computer:
--     fwd    forward thrust, 0..SCALE
--     aft    backward thrust, 0..SCALE     (only one is ever non-zero)
--
-- Each channel is a LIST OF SINKS and the channel's integer level is spread across them,
-- highest weight first, so a channel is not limited to the 0..15 one redstone line carries.
-- A sink is one of:
--
--   { side = "back", max = 15 }
--       the computer's own redstone face.
--
--   { per = "left", side = "back", max = 15 }
--       a face of a wrapped peripheral that speaks setAnalogOutput -- a Redstone Relay, or
--       anything with the same method. It is exactly
--           peripheral.wrap("left").setAnalogOutput("back", level)
--       written as data. `per` is whatever peripheral.wrap() takes: a SIDE of this computer,
--       when the relay is the block bolted to that face, or a network name like
--       "redstone_relay_1" when it sits out on a wired modem. Channels naming the same `per`
--       share one wrap. Note { side = "back" } drives this computer's OWN back face, while
--       { per = "left", side = "back" } drives the back face of the relay on its left.
--
--   { per = "thruster_0", call = "setThrust", max = 15 }
--       a method call instead of a redstone level: put(level) becomes p.setThrust(level).
--
--   { ..., weight = 2 }
--       optional, default 1: what one level on this sink is worth in channel units. Two lines
--       at weights 2 and 1 make 0..45 out of two 0..15 faces -- exactly the two-bundle trick
--       69/finerthrust.lua uses on Aer Togekiss, and the reason SCALE below can exceed 15.
--       Put the weight-1 sink LAST, or some totals cannot be expressed and show up as ! on
--       the display.
--
-- SCALE is what |thrust| = 1.0 is worth in channel units -- what the quantiser aims at. 15
-- for one plain line; raise it to the real capacity when a channel is wired across several.
SCALE = 15

WIRING = {
    fwd = { { side = 'front', max = 15, weight = 1 }, { per = 'left', side = 'back', max = 15, weight = 2 } },   -- e.g. { { side = "back",  max = 15, weight = 2 }, { side = "right", max = 15 } }
    aft = { { per = 'left', side = 'left', max = 15 } },   -- e.g. { { side = "front", max = 15, weight = 2 }, { side = "left",  max = 15 } }
}

-- Link loss on a fore/aft bank is the easy case, unlike the lift engines: zero is the safe
-- command and the UAC still holds attitude with the nozzles, so a dead link costs the
-- aircraft its acceleration and nothing else. Cut, and do it without a grace period.
LINK_TIMEOUT = 1.0

DEADBAND = 0.0         -- |thrust| below this reads as zero; raise it to stop idle chatter
DITHER   = true        -- false = plain rounding, i.e. throw the sub-step away again

-- ============================================================================

local modem = peripheral.find("modem")
if not modem then error("No modem attached") end
rednet.open(peripheral.getName(modem))
term.clear()

local function clamp(v, lo, hi) return math.max(lo, math.min(hi, v)) end
local function round(v) return math.floor(v + 0.5) end
local function line(row, text, colour)
    term.setCursorPos(1, row); term.clearLine()
    if colour then term.setTextColour(colour) end
    term.write(text)
    if colour then term.setTextColour(colours.white) end
end

-- Sigma-delta quantiser; one accumulator, reset on a cut so no stale fraction is paid out
-- into the first frame after the engines come back.
local function makeDither()
    local acc = 0
    return function(value, lo, hi)
        local want = value + acc
        local q = clamp(math.floor(want + 0.5), lo, hi)
        acc = clamp(want - q, -0.5, 0.5)
        return q
    end,
    function() acc = 0 end,
    function() return acc end
end
local ditherT, resetT, carryT = makeDither()

-- ---- resolve WIRING into callable sinks, once ----
-- A channel that will not resolve is not a crash: it is reported on the LINES row and simply
-- produces no output, which is what makes this usable while the wiring is still being worked
-- out. Identical to the block in vecengine.lua -- keep the four copies in step.
local OUT = { ch = {}, faults = {}, armed = true }
do
    local wrapped = {}
    for _, name in ipairs({ "fwd", "aft" }) do
        local sinks, cap = {}, 0
        for i, s in ipairs(WIRING[name] or {}) do
            local put, why = nil, nil
            if s.per then
                if wrapped[s.per] == nil then wrapped[s.per] = peripheral.wrap(s.per) or false end
                local p = wrapped[s.per]
                if not p then
                    why = s.per .. " not attached"
                elseif s.side then
                    if type(p.setAnalogOutput) ~= "function" then
                        why = s.per .. " has no setAnalogOutput"
                    else
                        put = function(v) p.setAnalogOutput(s.side, v) end
                    end
                elseif s.call then
                    if type(p[s.call]) ~= "function" then
                        why = s.per .. " has no " .. s.call .. "()"
                    else
                        put = function(v) p[s.call](v) end
                    end
                else
                    why = "sink " .. i .. " has `per` but neither `side` nor `call`"
                end
            elseif s.side then
                put = function(v) redstone.setAnalogOutput(s.side, v) end
            else
                why = "sink " .. i .. " names neither `side` nor `per`"
            end
            if put then
                sinks[#sinks + 1] = { put = put, max = s.max or 15, weight = s.weight or 1 }
                cap = cap + (s.max or 15) * (s.weight or 1)
            else
                OUT.faults[#OUT.faults + 1] = name .. ": " .. why
                OUT.armed = false
            end
        end
        table.sort(sinks, function(a, b) return a.weight > b.weight end)
        OUT.ch[name] = { sinks = sinks, cap = cap, level = 0, err = 0 }
        if #sinks == 0 then OUT.armed = false end
    end
end

-- Spread an integer channel level across that channel's sinks, biggest weight first. The
-- leftover is 0 for any level the wiring can express, and shown as ! when it is not -- a
-- demand that cannot be made is a wiring fact worth seeing, not a rounding detail.
local function emit(name, level)
    local c = OUT.ch[name]
    c.level = level
    local rest = level
    for _, s in ipairs(c.sinks) do
        local n = clamp(math.floor(rest / s.weight), 0, s.max)
        s.put(n)
        rest = rest - n * s.weight
    end
    c.err = rest
end

local function emitAll(steps)
    emit("fwd", steps >= 0 and steps or 0)
    emit("aft", steps <  0 and -steps or 0)
end

local function cutEngines()
    emitAll(0)
    resetT()
end

-- ---- engine master switch ----
local mst = false
local function switchEngine()
    while true do
        local sender, msg = rednet.receive(PROTOCOL_ENGINE_SWITCH, 1)
        if sender ~= nil and type(msg) == "table" then
            -- Scout has ONE physical switch feeding master/altitude/attitude alike, because
            -- on this airframe they are the same bank of thrusters (see 71/gpws.lua).
            mst = msg.master and true or false
        end
        line(8, "MST: " .. tostring(mst), mst and colours.green or colours.orange)
    end
end

local function thrustLoop()
    while true do
        local sender, msg = rednet.receive(PROTOCOL, LINK_TIMEOUT)
        -- msg.thrust ~= msg.thrust rejects a NaN, which would otherwise walk straight through
        -- clamp() and out onto the engines as an undefined level.
        if sender ~= nil and type(msg) == "table" and type(msg.thrust) == "number"
            and msg.thrust == msg.thrust then
            local f = clamp(THRUST_SIGN * msg.thrust, -1, 1)
            if math.abs(f) < DEADBAND then f = 0 end
            local steps
            if DITHER then
                steps = ditherT(f * SCALE, -SCALE, SCALE)
            else
                steps = clamp(round(f * SCALE), -SCALE, SCALE)
            end
            if mst then emitAll(steps) else cutEngines() end
            line(3, string.format("DEMAND %+7.4f  ->  %+3d / %d  (%+6.1f%%)",
                f, steps, SCALE, 100 * steps / SCALE))
            line(4, string.format("carry  %+7.3f   active: %s", carryT(), tostring(mst)))
            line(6, string.format("LINES  fwd[%3d%s]  aft[%3d%s]",
                OUT.ch.fwd.level, OUT.ch.fwd.err ~= 0 and "!" or " ",
                OUT.ch.aft.level, OUT.ch.aft.err ~= 0 and "!" or " "))
            line(9, "LINK: ok", colours.green)
        else
            -- Link lost. Zero is the safe fore/aft command; the UAC still holds attitude.
            cutEngines()
            line(3, "DEMAND    ----      ->    0")
            line(4, string.format("carry   0.000   active: %s", tostring(mst)))
            line(6, "LINES  fwd[  0 ]  aft[  0 ]")
            line(9, "LINK: no thrust signal - engines cut", colours.orange)
        end
        line(7, string.format("scale %d   %s", SCALE,
            OUT.armed and "wiring: ARMED" or "wiring: INCOMPLETE"),
            OUT.armed and colours.green or colours.orange)
    end
end

line(1, "SCOUT fore/aft thrust controller")
for i, f in ipairs(OUT.faults) do
    if i <= 3 then line(12 + i, "WIRING FAULT " .. f, colours.red) end
end
local ok, err = pcall(parallel.waitForAny, thrustLoop, switchEngine)
pcall(cutEngines)
if not ok then error(err, 0) end
