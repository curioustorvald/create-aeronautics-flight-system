-- SCRAMBLE TRACE -- a receive-only sniffer for the one command that has to work.
--
-- WHY IT EXISTS. "The scramble does not work" has four places it can break and no single
-- screen that shows all four at once: the MANCON may never put the flag on the wire (a key
-- bound to the wrong code, an unwired lever, no typewriter), the UAC may set it and have it
-- gated away, the ALT output may be right and the engine switch off, or the pair of line
-- levels may be right and the airframe simply have no lift left to give. This program prints
-- all four side by side and writes them to /scrambletrace.csv, so the answer comes off a file
-- instead of off four displays watched in sequence.
--
-- IT TRANSMITS NOTHING and it drives no output. Run it in a TAB beside the UAC:
--     bg pid/scrambletrace.lua          (or shell.openTab)
-- then hold the scramble -- key AND lever, one at a time -- and read the file back.
--
-- WHAT EACH COLUMN MEANS, and what a wrong one tells you:
--   stick_scram  the flag as the MANCON broadcast it. FALSE while you are holding the key
--                means the input never happened: wrong key code in manconconf's CFG.KEYS
--                (X is 88, V is 86, C is 67 -- get_keycodes.lua prints what a key really
--                sends), no typewriter found, or the lever on the wrong CFG.VS.SIDE.
--   stick_vs     the trim rate beside it. It FREEZES during a scramble by design, so an
--                unchanging value here is correct, not a stuck stick.
--   alt_up/dn    what the UAC put on the ALT protocol, as line FRACTIONS (0..1). During a
--                scramble this must be 1.000/0.000 -- or 0.000/1.000 inverted -- at EVERY
--                attitude. Anything between the two is the scramble not reaching the output.
--   cut          the lift engine commanded OFF. A scramble outranks the ground cut, so this
--                must be false while scrambling.
--   mst/alt_sw   the engine switches the relay gates on. The relay zeroes both lines unless
--                both are true, so a perfect alt_up/dn with mst=false is a switch, not code.
--   posy/vy      altitude (already datum-corrected by the sender) and climb rate, so the
--                file also answers "did the aeroplane actually go up".
--
-- Protocol names come from uacconf, so this file is airframe-independent -- copy it to any
-- UAC's pid/ directory and it traces that aircraft.

local CFG
do
    local ok, res = pcall(require, "uacconf")
    if not ok or type(res) ~= "table" then
        error("Cannot load pid/uacconf.lua: " .. tostring(res), 0)
    end
    CFG = res
end

local modem = peripheral.find("modem")
if not modem then error("No modem attached", 0) end
rednet.open(peripheral.getName(modem))

-- The engine switch has no knob in uacconf (the UAC does not listen to it; the relays do),
-- so it is derived from the family the other protocols share.
local FAMILY = (CFG.MANCON_PROTOCOL or ""):gsub("_mancon$", "")
local P_STICK  = CFG.MANCON_PROTOCOL
local P_ALT    = CFG.ALT_PROTOCOL
local P_ENGINE = FAMILY .. "_engineswitch"

local FILE = "/scrambletrace.csv"
local ROWS = 4000            -- ~3 min at 20 Hz, then it stops rather than filling the disk

-- Last-seen state, written by the receive loops and sampled by the writer. Held between
-- packets on purpose: a row is "what every channel was saying at this instant", and a
-- channel that has said nothing yet reads nil, which prints as an empty cell.
local S = { }

local function receiver(proto, fn)
    return function()
        while true do
            local sender, msg = rednet.receive(proto, 1)
            if sender and type(msg) == "table" then fn(msg) end
        end
    end
end

local rxStick = receiver(P_STICK, function(m)
    S.scram = (m.scramble == true)
    S.vs    = tonumber(m.vs)
    S.stickAt = os.clock()
end)
local rxAlt = receiver(P_ALT, function(m)
    S.up, S.down = tonumber(m.up), tonumber(m.down)
    S.cut = (m.cut == true)
    S.altAt = os.clock()
end)
local rxEngine = receiver(P_ENGINE, function(m)
    S.mst, S.altSw = m.master, m.altitude
end)

local function num(v, fmt) return v and string.format(fmt, v) or "" end
local function bool(v) if v == nil then return "" end return tostring(v) end

-- The same rotation uac.lua uses, copied verbatim rather than reached for through the
-- quaternion library: this file has to agree with the controller about which way is up to
-- the last digit, or a row that says "the craft was level" is worth nothing.
local function rotateVec(q, x, y, z)
    local ux, uy, uz, a = q.v.x, q.v.y, q.v.z, q.a
    local tx = 2 * (uy*z - uz*y)
    local ty = 2 * (uz*x - ux*z)
    local tz = 2 * (ux*y - uy*x)
    return x + a*tx + (uy*tz - uz*ty),
           y + a*ty + (uz*tx - ux*tz),
           z + a*tz + (ux*ty - uy*tx)
end

local function writerLoop()
    local f = fs.open(FILE, "w")
    if not f then error("Cannot open " .. FILE .. " for writing", 0) end
    f.writeLine("t,stick_scram,stick_vs,stick_age,alt_up,alt_dn,cut,mst,alt_sw,posy,vy,tilt_up")
    local t0 = os.clock()
    local n = 0
    term.clear()
    while n < ROWS do
        local now = os.clock()
        local posy, vy, tilt = nil, nil, nil
        local ok, pose = pcall(sublevel.getLogicalPose)
        if ok and pose and pose.position then
            posy = pose.position.y - CFG.ALT_OFFSET
            -- Body-up . world-up, the exact quantity the alt allocation routes on: 1 level,
            -- 0 knife-edge, -1 inverted. It is what tells a "the flag was set but the lines
            -- did not open" row apart from a "the craft was on its side" one.
            local okq, uy = pcall(function()
                local _, y = rotateVec(pose.orientation:inverse(), 0, 1, 0)
                return y
            end)
            if okq then tilt = uy end
        end
        local okv, v = pcall(sublevel.getLinearVelocity)
        if okv and v then vy = v.y end

        f.writeLine(table.concat({
            string.format("%.3f", now - t0),
            bool(S.scram), num(S.vs, "%.3f"),
            S.stickAt and string.format("%.2f", now - S.stickAt) or "",
            num(S.up, "%.4f"), num(S.down, "%.4f"), bool(S.cut),
            bool(S.mst), bool(S.altSw),
            num(posy, "%.2f"), num(vy, "%.2f"), num(tilt, "%.3f"),
        }, ","))
        n = n + 1
        if n % 20 == 0 then f.flush() end

        term.setCursorPos(1, 1) term.clearLine()
        term.write("SCRAMBLE TRACE -> " .. FILE .. "  row " .. n .. "/" .. ROWS)
        term.setCursorPos(1, 3) term.clearLine()
        term.write(string.format("stick  SCRAM %s   vs %s", bool(S.scram), num(S.vs, "%+.2f")))
        term.setCursorPos(1, 4) term.clearLine()
        term.write(string.format("alt    up %s  down %s  cut %s",
            num(S.up, "%.3f"), num(S.down, "%.3f"), bool(S.cut)))
        term.setCursorPos(1, 5) term.clearLine()
        term.write(string.format("engine mst %s  alt %s", bool(S.mst), bool(S.altSw)))
        term.setCursorPos(1, 6) term.clearLine()
        term.write(string.format("ship   y %s  vy %s  up.y %s",
            num(posy, "%.1f"), num(vy, "%+.1f"), num(tilt, "%+.2f")))

        sleep(0.05)
    end
    f.close()
    term.setCursorPos(1, 8)
    print("Done -- " .. ROWS .. " rows in " .. FILE)
end

print("Tracing " .. P_STICK .. " / " .. P_ALT .. " / " .. P_ENGINE)
parallel.waitForAny(writerLoop, rxStick, rxAlt, rxEngine)
