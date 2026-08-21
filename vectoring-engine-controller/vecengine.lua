-- SCOUT -- VECTORING ENGINE CONTROLLER
--
-- One of Scout's three down-firing vectoring engines. The UAC (#72) solves the whole
-- aircraft's thrust allocation in one place -- see sendVector's header in 72/pid/uac.lua --
-- and broadcasts three force vectors, one per engine. This computer owns exactly one of
-- them: it takes that engine's gimbal X, gimbal Y and thrust demand and turns them into
-- whatever this airframe's wiring actually wants.
--
-- THE SAME PROGRAM RUNS ON #77, #78 AND #80, and as of 2026-08-22 the three copies are
-- BYTE-IDENTICAL -- everything that differs per computer moved to vecconf.lua. So this file
-- can be copied between the three freely, which is the point: a fix made on one corner and not
-- the others is an aircraft that flies differently depending on which corner you look at, and
-- the allocation upstream assumes it does not. Keep them in step, exactly as 44/atc.lua and
-- 64/atc.lua are kept in step. **Never hand-edit vecconf.lua by copying it between computers.**
--
-- WHY THE DITHERING IS HERE AND NOT IN THE UAC. Every other relay on this fleet is handed
-- whole redstone steps, because the UAC knows exactly what the relay will do with them: one
-- line, 0..15. That is not true here. This engine's output allocation is not a single line
-- -- see WIRING below -- so the grid the demand has to be quantised ONTO is known only at
-- this end. The UAC therefore sends the demand as its control law computed it (a float,
-- clamped to the band but not rounded) and the sigma-delta quantiser lives here, where it
-- can be aimed at the real grid. Anything finer than one step is carried in an accumulator
-- and paid out a frame or two later; the airframe, far slower than 20 Hz, integrates it back
-- into the value that was asked for.

-- PER-COMPUTER CONFIGURATION lives in vecconf.lua beside this file: which engine this is, and
-- what its outputs are plugged into. It is a separate file for one reason -- **dropping in a
-- new vecengine.lua must never be able to touch this computer's wiring.** On 2026-08-22 the
-- wiring lived in this file, regenerating #78 and #80 from #77 overwrote both of their WIRING
-- tables with front-centre's, all three engines drove the same relay layout, and the aircraft
-- rolled itself over. Same split and same reason as pid/uacconf.lua against pid/uac.lua.
--
-- There is deliberately NO fallback copy of these values here: a second set of defaults would
-- let an edit there silently do nothing, and a half-configured engine must fail at BOOT with
-- the missing knob named, not fly on a guess.
local VC
do
    local ok, res = pcall(require, "vecconf")
    if not ok then
        error("Cannot load vecconf.lua (this engine's identity and wiring): " .. tostring(res)
            .. "\nIt must sit beside vecengine.lua and end with 'return VC'.", 0)
    end
    if type(res) ~= "table" then
        error("vecconf.lua did not return the VC table (got " .. type(res) .. ").", 0)
    end
    for _, k in ipairs({ "ENGINE", "ENGINE_NAME", "WIRING", "SCALE_X", "SCALE_Y", "SCALE_T" }) do
        if res[k] == nil then
            error("vecconf.lua is missing VC." .. k .. " -- the file looks incomplete.", 0)
        end
    end
    VC = res
end
ENGINE, ENGINE_NAME = VC.ENGINE, VC.ENGINE_NAME
WIRING              = VC.WIRING
SCALE_X, SCALE_Y, SCALE_T = VC.SCALE_X, VC.SCALE_Y, VC.SCALE_T

PROTOCOL               = 'scout_vector'         -- in: UAC -> all three engines, one message
PROTOCOL_ENGINE_SWITCH = 'scout_engineswitch'   -- in: master/altitude/attitude switch (#71)

-- The bands the UAC clamps its demand to. MUST MATCH CFG.TRI_VEC_MAX and CFG.TRI_T_MAX in
-- 72/pid/uacconf.lua -- they are the definition of "full deflection" and "full thrust", and
-- if the two ends disagree this engine quietly runs at the wrong scale while every other
-- number on both screens looks right.
VEC_MAX = 15
THR_MAX = 15

-- The five output channels (x_pos, x_neg, y_pos, y_neg, thrust), the sinks they are wired to
-- and the SCALE_* the quantiser aims at are all in vecconf.lua -- see its header.

-- WHAT TO DO WHEN THE UAC GOES QUIET. This is a real decision and it is worth making
-- deliberately, because unlike every other relay on this fleet these engines are the LIFT:
--   'hover'  hold a level hover -- nozzles centred, thrust at HOVER_THRUST -- after
--            GRACE seconds of silence. The craft stops manoeuvring and drifts, but it stays
--            up, which gives a pilot something to work with. THE DEFAULT.
--   'hold'   keep commanding whatever arrived last, forever. Rides out a hiccup perfectly
--            and turns a dead controller into a runaway; only sensible with a short GRACE.
--   'cut'    stop the engines. Safe on a fore/aft bank (69/finerthrust.lua does exactly
--            this, and says why) and NOT safe here: on the lift engines it is a crash.
-- Whichever is chosen, a hiccup shorter than GRACE always holds -- a dropped packet must not
-- twitch the engines.
LINK_LOST    = 'hover'
LINK_TIMEOUT = 0.5     -- s; a receive returning nothing this often is what starts the clock
GRACE        = 1.0     -- s of silence tolerated before LINK_LOST applies (hold until then)
HOVER_THRUST = 7.0     -- thrust units for 'hover'. Match CFG.TRI_HOVER in uacconf.

-- How fast the command eases to the fallback when the link is lost, in command units per
-- second. It is a RAMP and not a jump on purpose: snapping three engines to centred nozzles and
-- hover thrust the instant a packet goes missing is a violent kick, and a kick that arrives
-- only on dropped packets looks exactly like a control-loop instability while being nothing of
-- the sort. At 15/s a full-deflection nozzle takes a second to centre.
LOST_SLEW = 15.0

-- Redraw at most every N messages. The screen is NOT free: a full redraw is a dozen
-- string.format calls, a table.concat and six cleared lines, and at 20 messages a second on a
-- busy rednet that is enough work to make this computer fall behind its own event queue --
-- at which point it starts DROPPING broadcasts, which is what the fallback above then has to
-- paper over. 5 = redraw at 4 Hz, which is plenty to read and leaves the tick almost empty.
-- The control path itself is untouched by this: every message is still acted on in full.
DRAW_EVERY = 5

DEADBAND = 0.0         -- |gimbal| below this reads as centred; raise it to stop idle chatter
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

-- Sigma-delta quantiser; one accumulator per channel, they must NOT share (the same rule,
-- and the same code, as the UAC's makeDither). reset() is for a cut: the accumulator holds
-- at most half a step, so nothing is owed when the engine comes back, and paying out a stale
-- fraction into the first frame after a relight is a twitch nobody asked for.
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
-- Channel display labels, built ONCE. These used to be produced by two gsubs per channel per
-- message -- ten string allocations a tick to print five fixed strings.
local CHAN  = { "x_pos", "x_neg", "y_pos", "y_neg", "thrust" }
local CLABEL = { x_pos = "x+", x_neg = "x-", y_pos = "y+", y_neg = "y-", thrust = "th" }

local ditherX, resetX, carryX = makeDither()
local ditherY, resetY, carryY = makeDither()
local ditherT, resetT, carryT = makeDither()

-- ---- resolve WIRING into callable sinks, once ----
-- A channel that will not resolve is not a crash: it is reported UNWIRED/FAULT on the LINES
-- row and simply produces no output. That is what makes this a usable stub -- the rest of the
-- aircraft can be flown and watched while the wiring is still being worked out.
local OUT = { ch = {}, faults = {}, armed = true }
do
    local wrapped = {}
    for _, name in ipairs(CHAN) do
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
        -- Highest weight first: that is what makes the greedy spread below exact.
        table.sort(sinks, function(a, b) return a.weight > b.weight end)
        OUT.ch[name] = { sinks = sinks, cap = cap, level = 0, err = 0 }
        if #sinks == 0 then OUT.armed = false end
    end
end

-- Spread an integer channel level across that channel's sinks, biggest weight first. Returns
-- the leftover, which is 0 for any level the wiring can express and non-zero when it cannot
-- -- either the level is over capacity, or the weights have a gap in them (no weight-1 sink).
-- It is shown as ERR rather than silently absorbed, because a demand that cannot be made is
-- a wiring fact worth seeing, not a rounding detail.
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

local function emitAll(x, y, t)
    emit("x_pos", x >= 0 and x or 0)
    emit("x_neg", x <  0 and -x or 0)
    emit("y_pos", y >= 0 and y or 0)
    emit("y_neg", y <  0 and -y or 0)
    emit("thrust", t)
end

local function cutEngine()
    emitAll(0, 0, 0)
    resetX(); resetY(); resetT()
end

-- ---- engine master switch ----
local mst = false
local function switchEngine()
    while true do
        local sender, msg = rednet.receive(PROTOCOL_ENGINE_SWITCH, 1)
        if sender ~= nil and type(msg) == "table" then
            -- Scout has ONE physical switch feeding master/altitude/attitude alike, because
            -- on this airframe they are the same bank of thrusters (see 71/gpws.lua). Read
            -- master and let the other two follow it.
            mst = msg.master and true or false
        end
        line(10, string.format("MST: %-5s   ENGINE %s (%s)", tostring(mst), ENGINE, ENGINE_NAME),
            mst and colours.green or colours.orange)
    end
end

-- ---- the demand ----
local last = { x = 0, y = 0, t = 0 }   -- last GOOD demand, in UAC units
local quiet = 0                        -- seconds since the last good message
local drops, worstQuiet = 0, 0         -- link-health counters, for the display
local frame = 0                        -- display throttle counter (see DRAW_EVERY)

-- Ease `cur` toward `tgt` by at most `step`. Used only on the fallback path.
local function toward(cur, tgt, step)
    if cur < tgt then return math.min(tgt, cur + step) end
    return math.max(tgt, cur - step)
end

local function vectorLoop()
    while true do
        local sender, msg = rednet.receive(PROTOCOL, LINK_TIMEOUT)
        local m = (sender ~= nil and type(msg) == "table") and msg[ENGINE] or nil
        local cut, state = false, "ok"
        -- A message that arrived but carries no slot for us is not a dead link -- it is the
        -- ENGINE key above not matching any key in CFG.TRI_ENGINES, which is the single most
        -- likely mistake when cloning this program onto the next engine. Say so outright
        -- rather than letting it present as a mysterious loss of signal.
        if sender ~= nil and type(msg) == "table" and m == nil then
            line(6, "NO SLOT '" .. ENGINE .. "' IN MESSAGE - check ENGINE vs CFG.TRI_ENGINES",
                colours.red)
        else
            line(6, "")
        end
        if type(m) == "table" and type(m.x) == "number" and type(m.y) == "number"
            and type(m.thrust) == "number"
            -- v ~= v rejects a NaN, which would otherwise walk straight through clamp() and
            -- out onto the engine as an undefined level.
            and m.x == m.x and m.y == m.y and m.thrust == m.thrust then
            quiet = 0
            last.x = clamp(m.x, -VEC_MAX, VEC_MAX)
            last.y = clamp(m.y, -VEC_MAX, VEC_MAX)
            last.t = clamp(m.thrust, 0, THR_MAX)
            cut = msg.cut and true or false
            if cut then state = "CUT" end
        else
            if quiet == 0 then drops = drops + 1 end
            quiet = quiet + LINK_TIMEOUT
            if quiet > worstQuiet then worstQuiet = quiet end
            if quiet <= GRACE then
                state = "hold"                       -- a hiccup must not twitch the engine
            elseif LINK_LOST == 'cut' then
                cut, state = true, "LOST/cut"
            elseif LINK_LOST == 'hover' then
                -- RAMP, never snap -- see LOST_SLEW.
                local step = LOST_SLEW * LINK_TIMEOUT
                last.x = toward(last.x, 0, step)
                last.y = toward(last.y, 0, step)
                last.t = toward(last.t, HOVER_THRUST, step)
                state = "LOST/hover"
            else
                state = "LOST/hold"
            end
        end

        local xs, ys, ts = 0, 0, 0
        if cut or not mst then
            cutEngine()
        else
            local fx = last.x / VEC_MAX
            local fy = last.y / VEC_MAX
            if math.abs(fx) < DEADBAND then fx = 0 end
            if math.abs(fy) < DEADBAND then fy = 0 end
            local ft = last.t / THR_MAX
            if DITHER then
                xs = ditherX(fx * SCALE_X, -SCALE_X, SCALE_X)
                ys = ditherY(fy * SCALE_Y, -SCALE_Y, SCALE_Y)
                ts = ditherT(ft * SCALE_T, 0, SCALE_T)
            else
                xs = clamp(round(fx * SCALE_X), -SCALE_X, SCALE_X)
                ys = clamp(round(fy * SCALE_Y), -SCALE_Y, SCALE_Y)
                ts = clamp(round(ft * SCALE_T), 0, SCALE_T)
            end
            emitAll(xs, ys, ts)
        end

        -- ---- display, THROTTLED (see DRAW_EVERY). Everything above this line runs on every
        -- message; nothing below it affects a thruster.
        frame = frame + 1
        if frame >= DRAW_EVERY or state ~= "ok" then
            frame = 0
            line(3, string.format("DEMAND  x%+7.2f  y%+7.2f  thr%7.2f", last.x, last.y, last.t))
            line(4, string.format("OUTPUT  x%+7d  y%+7d  thr%7d", xs, ys, ts))
            line(5, string.format("carry   x%+7.2f  y%+7.2f  thr%+6.2f", carryX(), carryY(), carryT()))
            local ln = {}
            for i = 1, #CHAN do
                local c = OUT.ch[CHAN[i]]
                ln[i] = CLABEL[CHAN[i]] .. (#c.sinks == 0 and "[--]"
                    or string.format("[%2d%s]", c.level, c.err ~= 0 and "!" or ""))
            end
            line(7, "LINES " .. table.concat(ln, " "))
            line(8, string.format("scale x%d y%d t%d   %s", SCALE_X, SCALE_Y, SCALE_T,
                OUT.armed and "wiring: ARMED" or "wiring: INCOMPLETE"),
                OUT.armed and colours.green or colours.orange)
            line(11, string.format("LINK: %-11s quiet %.1fs  on-loss: %s", state, quiet, LINK_LOST),
                (state == "ok" or state == "CUT") and colours.green or colours.orange)
            -- THE LINE TO WATCH IF THE AIRCRAFT KICKS. drops = how many times the stream from
            -- the UAC has broken since boot; worst = the longest gap. On a healthy link both
            -- stay at zero. Anything else and this computer is missing broadcasts -- and every
            -- gap longer than GRACE is the fallback easing the engines toward hover and back,
            -- which the pilot feels as a lurch that no amount of UAC tuning will remove.
            line(12, string.format("drops %d   worst gap %.1fs", drops, worstQuiet),
                drops == 0 and colours.green or colours.red)
        end
    end
end

line(1, string.format("SCOUT vector engine  %s (%s)", ENGINE_NAME, ENGINE))
for i, f in ipairs(OUT.faults) do
    if i <= 3 then line(12 + i, "WIRING FAULT " .. f, colours.red) end
end
local ok, err = pcall(parallel.waitForAny, vectorLoop, switchEngine)
pcall(cutEngine)
if not ok then error(err, 0) end
