-- TICK PROBE -- what rate does a computer on THIS server actually get?
--
-- The UAC's control loop measured itself at 0.30 s per iteration against the 0.05 s its
-- sleep(0.05) asks for -- a dead-on 6x, with zero jitter across 200 samples. Zero jitter is
-- the interesting part: a program that is merely busy runs late by a varying amount. A fixed
-- ratio is what a BUDGET looks like.
--
-- This computer runs nothing else, so if it is also slow the cause is not in any flight
-- program and no amount of tuning will reach it. Three phases separate the kinds of work that
-- CC charges to the shared main-thread budget (config/computercraft-server.toml:
-- max_main_global_time, 10 ms per tick for EVERY computer on the server put together):
--
--   BARE    sleep only -- the floor. This is what the scheduler will give a computer that
--           asks for nothing.
--   RADIO   one rednet.broadcast per iteration. The flight system's main cost: the UAC alone
--           sends three per tick, and there are 62 auto-running computers.
--   SCREEN  ten lines of terminal writes per iteration, i.e. what a full instrument redraw
--           costs.
--
-- Both clocks are read on purpose. os.clock() is the computer's own uptime, which is what the
-- UAC times itself with; os.epoch("utc") is real wall time. If they disagree, the slowdown is
-- an artefact of the clock rather than of the scheduler, and that changes the answer entirely.

local N = 100

local function line(row, text, colour)
    term.setCursorPos(1, row); term.clearLine()
    if colour then term.setTextColour(colour) end
    term.write(text)
    term.setTextColour(colours.white)
end

local modem = peripheral.find("modem")
if modem then rednet.open(peripheral.getName(modem)) end

local function phase(work)
    local c0, e0 = os.clock(), os.epoch("utc")
    for _ = 1, N do
        work()
        sleep(0.05)
    end
    return (os.clock() - c0) / N, (os.epoch("utc") - e0) / N / 1000
end

term.clear()
line(1, "TICK PROBE -- computer #" .. os.getComputerID(), colours.yellow)
line(2, N .. " iterations per phase, sleep(0.05) each")
line(3, "ideal = 50 ms / 20.0 Hz")

local rows = {}
local function run(row, name, work)
    line(row, string.format("%-7s running...", name), colours.orange)
    local c, e = phase(work)
    rows[#rows + 1] = { name, c, e }
    line(row, string.format("%-7s clock %6.1f ms   real %6.1f ms   %5.2f Hz   %4.1fx slow",
        name, c * 1000, e * 1000, 1 / c, c / 0.05),
        (c > 0.125 and colours.red) or (c > 0.067 and colours.orange) or colours.green)
end

run(5, "BARE", function() end)
run(6, "RADIO", function()
    if modem then rednet.broadcast({ probe = 1 }, "tickprobe") end
end)
run(7, "SCREEN", function()
    for i = 10, 19 do
        term.setCursorPos(1, i); term.clearLine()
        term.write(string.format("redraw %d %.4f %.4f %s", i, os.clock(), math.pi * i, "xxxxxxxx"))
    end
end)

for i = 10, 19 do term.setCursorPos(1, i); term.clearLine() end
line(9, "----------------------------------------")
line(10, "BARE slow too  => server-wide budget, not the", colours.yellow)
line(11, "                 flight code. See max_main_global_time", colours.yellow)
line(12, "                 in computercraft-server.toml (10 ms for", colours.yellow)
line(13, "                 ALL 62 computers, per tick).", colours.yellow)
line(14, "BARE fine, RADIO/SCREEN slow => it is what the", colours.yellow)
line(15, "                 programs DO, and that can be cut.", colours.yellow)
line(16, "clock vs real disagreeing => os.clock is the liar,", colours.yellow)
line(17, "                 and the UAC's own timing is wrong.", colours.yellow)
term.setCursorPos(1, 19)
term.setBackgroundColour(colours.green); term.setTextColour(colours.black)
term.write(" PROBE DONE -- read the three rows above ")
term.setBackgroundColour(colours.black); term.setTextColour(colours.white)
