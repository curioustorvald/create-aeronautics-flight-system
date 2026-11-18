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

PROTOCOL               = 'aertogekiss_thrustf'        -- in: UAC -> signed float, -1..+1
PROTOCOL_ENGINE_SWITCH = 'aertogekiss_engineswitch' -- in: master/altitude/attitude switch panel

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
-- SCALE is what |thrust| = 1.0 is worth in channel units -- what the quantiser aims at.
--
-- IT IS PER CHANNEL, AND `nil` MEANS "THIS CHANNEL'S OWN WIRED CAPACITY" (2026-08-28). It was
-- a single number 15 for both channels, and that was the bug that left the secondary forward
-- engine dead: `fwd` is wired across 45 units (a weight-2 line plus a weight-1 line, the
-- two-bundle resolution trick #69 uses), but the quantiser only ever aimed at 15 of them.
-- emit() fills highest weight first, so a demand that never exceeded 15 was spent almost
-- entirely on the weight-2 line -- which itself only ever reached 7 of its 15 -- and the
-- weight-1 line got `level mod 2`, i.e. 0 or 1, for the whole of the demand range. Forward
-- thrust was capped at a third of what is bolted to the aircraft.
--
-- The resolved capacity of every channel was ALREADY being computed at boot (OUT.ch[x].cap)
-- and then never read. It is what these default to now, so a correctly-declared WIRING needs
-- no SCALE at all and cannot fall out of step with itself when a line is added.
--
-- THE TWO CHANNELS ARE NOT THE SAME SIZE ON THIS AIRCRAFT: fwd 45, aft 15. That is a fact
-- about what is wired, not a setting -- so |thrust| = 1.0 astern commands a third of the force
-- it commands ahead, and the aeroplane accelerates three times harder than it brakes. The UAC
-- has no way to express that (it normalises both directions on CFG.THRUST_MAX), so its speed
-- loop sees a kink at zero and trims through it on the integral. TO FIX IT PROPERLY, wire the
-- aft bank across two lines the way fwd is and the asymmetry goes away on its own.
-- Setting these to a number overrides the wiring; keep them nil unless there is a reason.
SCALE = { fwd = nil, aft = nil }

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

-- Resolve each channel's scale: the declared override, else what the wiring can actually
-- express. A channel with no usable sink resolves to 0 and is simply never commanded.
for _, name in ipairs({ "fwd", "aft" }) do
    local c = OUT.ch[name]
    c.scale = SCALE[name] or c.cap
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
            -- The two channels can have different capacities, so the grid the demand is
            -- quantised onto depends on WHICH WAY it is pointing. Pick the scale first, then
            -- quantise. The sigma-delta accumulator is shared across the sign change and so
            -- carries at most half a unit into the other channel's units on the tick the
            -- demand crosses zero -- the same half-unit bound it always had, and far smaller
            -- than the step it is smoothing.
            local sc = (f >= 0) and OUT.ch.fwd.scale or OUT.ch.aft.scale
            local steps
            if DITHER then
                steps = ditherT(f * sc, -sc, sc)
            else
                steps = clamp(round(f * sc), -sc, sc)
            end
            if mst then emitAll(steps) else cutEngines() end
            line(3, string.format("DEMAND %+7.4f  ->  %+3d / %d  (%+6.1f%%)",
                f, steps, sc, (sc > 0) and (100 * steps / sc) or 0))
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
        line(7, string.format("scale fwd %d / aft %d   %s",
            OUT.ch.fwd.scale, OUT.ch.aft.scale,
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
