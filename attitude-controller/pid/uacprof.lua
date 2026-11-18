-- UAC LOOP PROFILER -- where does the control loop's tick actually go?
--
-- WHAT THIS IS FOR. uac.lua's controlLoop asks for 20 Hz (sleep(0.05)) and gets ~4.4 Hz.
-- Scout's flight recorder says something very specific about WHY, and it is not what
-- "the code is too big" would predict: across 660 logged ticks the measured period was
-- ALWAYS an exact multiple of one game tick --
--
--     0.10 s  0.2%   0.15 s  6.7%   0.20 s 41.7%   0.25 s 41.4%   0.30 s 10.2%
--
-- -- with nothing in between. Lua arithmetic taking too long produces a CONTINUOUS spread
-- of periods. A distribution with no fractional part at all is a loop that is not compute
-- bound at all: it is BLOCKING, a whole game tick at a time, and the only things in that
-- loop that can block are the calls that have to be served by the server's main thread.
-- controlLoop makes ten of them per iteration: five `sublevel` reads, four
-- `rednet.broadcast`es, and the trailing `sleep`.
--
-- So the question this program answers is not "how many milliseconds does X cost" -- it is
-- "HOW MANY GAME TICKS does X cost", which is the only unit the scheduler works in. Each
-- phase runs the SAME `sleep(0.05)` loop with one more kind of work in it, so the
-- difference between two phases is what that work is worth in ticks.
--
-- HOW TO READ THE RESULT
--   BARE ~= 0.05                     -- the scheduler is fine; every later phase is our own doing
--   BARE >> 0.05                     -- server-wide budget, nothing in this file can reach it;
--                                       go and read 18/tickprobe.lua instead
--   SUB5 - SUB3 large                -- caching the tensor and the mass is the win, as designed
--   RADIO4 - RADIO1 large            -- the four broadcasts are the wall, and the next move is a
--                                       single bus message the relays each pick a field out of
--   MATH ~= BARE                     -- arithmetic is free; stop reading the control law
--   AH_OLD >> AH_NEW                 -- the artificial horizon was the compute cost, and the
--                                       run-based redraw already fixed it
--
-- Nothing here commands a thruster: the radio phases broadcast on "uacprof_null", which no
-- computer on the fleet subscribes to. Safe to run on a parked, powered aircraft.
--
-- Results are written to /uacprof.csv as well as drawn, because reading twelve numbers off
-- a terminal by eye is how the 6x got mistaken for jitter in the first place.

local N       = 60                  -- iterations per phase (60 * ~0.25 s ~= 15 s each)
local PERIOD  = 0.05                -- what each phase's loop ASKS for
local OUTFILE = "/uacprof.csv"
local NULLPROTO = "uacprof_null"    -- deliberately nothing's protocol

local modem = peripheral.find("modem")
if modem then rednet.open(peripheral.getName(modem)) end
local hasSub = (sublevel ~= nil)

local W, H = term.getSize()

local function line(row, text, colour)
    term.setCursorPos(1, row); term.clearLine()
    term.setTextColour(colour or colours.white)
    term.write(text:sub(1, W))
    term.setTextColour(colours.white)
end

-- ---------------------------------------------------------------------------
-- The work each phase adds. Every one is written to be as close as possible to what
-- controlLoop actually does with that call, INCLUDING what it does with the result --
-- a getter whose return value is thrown away can be optimised away by nothing here, but
-- keeping the arithmetic makes the phases comparable to the real loop.
-- ---------------------------------------------------------------------------
local sink = 0   -- keeps results live so nothing looks unused

local function wPose()   local p = sublevel.getLogicalPose();   sink = sink + p.position.y end
local function wVel()    local v = sublevel.getLinearVelocity(); sink = sink + v.y end
local function wAngVel() local w = sublevel.getAngularVelocity(); sink = sink + w.y end
local function wInert()  local I = sublevel.getInertiaTensor();  sink = sink + (type(I) == "table" and 1 or 0) end
local function wMass()   sink = sink + sublevel.getMass() end

local function wSub3() wPose(); wVel(); wAngVel() end
local function wSub5() wPose(); wVel(); wAngVel(); wInert(); wMass() end

local function wRadio1()
    rednet.broadcast({ pitch = 0.0, roll = 0.0 }, NULLPROTO)
end
local function wRadio4()
    rednet.broadcast({ pitch = 0.0, roll = 0.0 }, NULLPROTO)
    rednet.broadcast({ thrust = 0.0, yaw = 0.0, strafe = 0.0 }, NULLPROTO)
    rednet.broadcast({ thrust = 0.0 }, NULLPROTO)
    rednet.broadcast({ up = 0.5, down = 0.5, cut = false }, NULLPROTO)
end

-- The Advanced-Math object calls the loop makes per tick: one inverse (twice, in the
-- shipped build), three toEuler, and one fromAxisAngle+vector.new for a q_des the
-- decomposed error path never reads.
local function wQuat()
    local q = sublevel.getLogicalPose().orientation
    local qi = q:inverse()
    local a = q:toEuler()
    sink = sink + (type(a) == "number" and a or a.x) + (qi and 0 or 1)
    if type(quaternion) == "table" and quaternion.fromAxisAngle then
        local qd = quaternion.fromAxisAngle(vector.new(0, 1, 0), 0.3)
        sink = sink + (qd and 0 or 1)
    end
end

-- A stand-in for the control law: no API calls at all, just the sort of arithmetic
-- attitude() and the translation laws do. If this phase is indistinguishable from BARE,
-- the size of the control law is not the problem and no amount of trimming it will help.
local function wMath()
    local acc = 0
    for i = 1, 400 do
        local a = math.sin(i * 0.017) * math.cos(i * 0.031)
        acc = acc + math.sqrt(a * a + 1e-6) + math.atan(a, 1.0)
    end
    sink = sink + acc
end

-- The artificial horizon, as the shipped drawAH draws it: one setCursorPos +
-- setBackgroundColour + write per CELL.
local AH_TOP, AH_H = 1, 9
local function wAHOld()
    local midY, midX = AH_TOP + math.floor(AH_H / 2), math.floor(W / 2) + 1
    local slope, pitchOff = 0.2, 1.0
    for col = 1, W do
        local hrow = math.floor(midY + pitchOff - (col - midX) * slope + 0.5)
        for row = AH_TOP, AH_TOP + AH_H - 1 do
            term.setCursorPos(col, row)
            if row < hrow then term.setBackgroundColour(colours.blue)
            elseif row > hrow then term.setBackgroundColour(colours.brown)
            else term.setBackgroundColour(colours.white) end
            term.write(" ")
        end
    end
    term.setBackgroundColour(colours.black)
end

-- ...and as the optimised drawAH draws it: one blit per ROW, of three runs.
local function wAHNew()
    local midY, midX = AH_TOP + math.floor(AH_H / 2), math.floor(W / 2) + 1
    local slope, pitchOff = 0.2, 1.0
    local A = midY + pitchOff
    local spaces = string.rep(" ", W)
    for row = AH_TOP, AH_TOP + AH_H - 1 do
        local lo = (A - row - 0.5) / slope + midX
        local hi = (A - row + 0.5) / slope + midX
        local c1 = math.max(1, math.min(W + 1, math.floor(lo) + 1))
        local c2 = math.max(0, math.min(W, math.floor(hi)))
        local bg = string.rep("b", c1 - 1) .. string.rep("0", math.max(0, c2 - c1 + 1))
        bg = bg .. string.rep("c", W - #bg)
        term.setCursorPos(1, row)
        term.blit(spaces, string.rep("0", W), bg)
    end
    term.setBackgroundColour(colours.black)
end

-- ---------------------------------------------------------------------------
-- The measurement. Both clocks on purpose, exactly as 18/tickprobe.lua does it: os.clock()
-- is what controlLoop times itself with, os.epoch("utc") is real wall time. If the two
-- disagree the slowdown is an artefact of the clock and every conclusion changes.
-- Per-iteration periods are kept so the TICK HISTOGRAM can be printed -- that histogram is
-- the whole diagnosis, and a mean alone hides it.
-- ---------------------------------------------------------------------------
local results = {}

local function phase(name, work, sleepy)
    local dts = {}
    local c0, e0 = os.clock(), os.epoch("utc")
    local prev = c0
    for _ = 1, N do
        work()
        if sleepy then sleep(PERIOD) else os.queueEvent("uacprof"); os.pullEvent("uacprof") end
        local t = os.clock()
        dts[#dts + 1] = t - prev
        prev = t
    end
    local c = (os.clock() - c0) / N
    local e = (os.epoch("utc") - e0) / N / 1000
    -- Histogram in whole game ticks. This is the number that matters.
    local hist = {}
    for _, d in ipairs(dts) do
        local k = math.floor(d / 0.05 + 0.5)
        hist[k] = (hist[k] or 0) + 1
    end
    results[#results + 1] = { name = name, clock = c, real = e, hist = hist }
    return c, e
end

-- ---------------------------------------------------------------------------
term.setBackgroundColour(colours.black); term.clear()
line(1, "UAC LOOP PROFILER -- computer #" .. os.getComputerID(), colours.yellow)
line(2, string.format("%d iterations/phase, asking sleep(%.2f) = 20.0 Hz", N, PERIOD))
line(3, hasSub and "sublevel: present" or "sublevel: ABSENT (sub phases skipped)",
    hasSub and colours.green or colours.red)

local row = 5
local function run(name, work, sleepy)
    if row > H - 1 then row = 5 end
    line(row, string.format("%-8s running...", name), colours.orange)
    local c = phase(name, work, sleepy ~= false)
    line(row, string.format("%-8s %6.1f ms  %5.2f Hz  %4.1f ticks  %4.1fx",
        name, c * 1000, 1 / c, c / 0.05, c / PERIOD),
        (c > 0.125 and colours.red) or (c > 0.067 and colours.orange) or colours.green)
    row = row + 1
end

local function noop() end

run("BARE",   noop)
run("MATH",   wMath)
if hasSub then
    run("POSE",   wPose)
    run("VEL",    wVel)
    run("ANGVEL", wAngVel)
    run("INERTIA", wInert)
    run("MASS",   wMass)
    run("SUB3",   wSub3)
    run("SUB5",   wSub5)
    run("QUAT",   wQuat)
end
if modem then
    run("RADIO1", wRadio1)
    run("RADIO4", wRadio4)
end
run("AH_OLD", wAHOld)
run("AH_NEW", wAHNew)
-- The same two heaviest phases WITHOUT the sleep, to separate "the call blocks" from "the
-- sleep is what paces us". If SUB5_NS is still multiple ticks, the calls themselves block
-- and removing the sleep buys nothing; if it collapses to ~1 tick, the sleep was the pacer.
if hasSub then run("SUB5_NS", wSub5, false) end
if modem then run("RAD4_NS", wRadio4, false) end

-- ---------------------------------------------------------------------------
local fh = fs.open(OUTFILE, "w")
if fh then
    fh.writeLine("phase,clock_ms,real_ms,hz,ticks,t1,t2,t3,t4,t5,t6,t7,t8")
    for _, r in ipairs(results) do
        local cols = {}
        for k = 1, 8 do cols[k] = tostring(r.hist[k] or 0) end
        fh.writeLine(string.format("%s,%.2f,%.2f,%.3f,%.2f,%s",
            r.name, r.clock * 1000, r.real * 1000, 1 / r.clock, r.clock / 0.05,
            table.concat(cols, ",")))
    end
    fh.close()
end

term.setBackgroundColour(colours.black); term.clear()
line(1, "UAC PROFILE -- #" .. os.getComputerID() .. "  -> " .. OUTFILE, colours.yellow)
line(2, "phase      ms    Hz  ticks   tick histogram 1..6")
row = 3
for _, r in ipairs(results) do
    if row <= H then
        local h = {}
        for k = 1, 6 do h[k] = string.format("%2d", r.hist[k] or 0) end
        line(row, string.format("%-8s %5.0f %5.1f %5.2f   %s",
            r.name, r.clock * 1000, 1 / r.clock, r.clock / 0.05, table.concat(h, " ")),
            (r.clock > 0.125 and colours.red) or (r.clock > 0.067 and colours.orange)
            or colours.green)
        row = row + 1
    end
end
term.setCursorPos(1, H)
