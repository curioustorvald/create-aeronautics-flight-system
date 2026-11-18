-- VERTICAL SPEED COMMANDER
-- Reads the pilot's vertical-speed stick and broadcasts the WANTED climb rate (m/s) on
-- aertogekiss_mancon. It runs NO control of its own any more: the UAC (#1) owns the
-- vertical channel and turns this wanted rate into thruster commands (attitude-aware),
-- through the same inner climb-rate loop it uses for altitude-hold and autoland. This
-- computer drives NO redstone -- it is a rednet commander only.
--
-- INPUT:  analog level on IN_SIDE, 0..15. 7 = hold, 14 = climb at VS_MAX_CMD, 0 =
--         descend at VS_MAX_CMD, linear in between. 15 = SCRAMBLE (see below).
-- OUTPUT: rednet { vs = <m/s>, scramble = true|nil } on aertogekiss_mancon. No thruster line.
--
-- CAUTION: an unpowered input line reads 0, which is a full-rate descent command. Nothing
-- here can tell a cut wire from a deliberate 0, so wire the stick so it fails to 7.

-- CONFIG ---------------------------------------------------------------------
local IN_SIDE  = "front"
local PROTOCOL = "aertogekiss_mancon"

-- Command input. CC hands back 0..15; 0..14 is the proportional rate travel and 15 is the
-- separate scramble step below, so the top step is taken OUT of the scaling rather than
-- being read as a 15th of extra climb the scaling does not know about.
local IN_NEUTRAL = 7
local IN_MIN, IN_MAX = 0, 14
local IN_SWING = math.min(IN_NEUTRAL - IN_MIN, IN_MAX - IN_NEUTRAL)

-- SCRAMBLE -- the top redstone step, and the one stick position that is NOT a rate.
-- "Get us up NOW": lift engine open at full throttle, climb as fast as the airframe can.
--
-- WHY IT IS A FLAG AND NOT A BIG NUMBER. This step used to be sent as `vs = 100.0`, and
-- that could never have worked, because a rate is exactly what the rest of the chain exists
-- to limit. The UAC clamps the wanted rate to CFG.CLIMB_MAX (3.75 m/s on this airframe)
-- before the rate loop ever sees it, so 100 m/s arrived as 3.75 m/s -- the same command a
-- level-14 stick already gives. The stick's top step did nothing but repeat its second-top
-- step. Raising CLIMB_MAX instead is not the fix either: that is the handling envelope for
-- the WHOLE proportional travel, and loosening it to buy one detent would loosen every
-- other rate the aircraft ever flies (including the autoland's).
--
-- So scramble leaves the rate channel alone entirely and travels as its own flag. The UAC
-- bypasses the rate loop for it and commands full lift outright -- see CFG.MANCON_SCRAMBLE
-- in pid/uacconf.lua. A receiver that predates the flag simply ignores it and keeps flying
-- the rate beside it, which is the safe way round: an old UAC degrades to "no scramble",
-- not to "100 m/s runaway".
local IN_SCRAMBLE = 15

-- What a full stick asks for, m/s. The UAC clamps the wanted rate to its own
-- CLIMB_MAX / DESCEND_MAX, so keep this at (or just below) that -- otherwise the top of
-- the stick travel simply saturates against the UAC's limit and loses resolution.
local VS_MAX_CMD      = 12.0
local MPS_PER_IN_STEP = VS_MAX_CMD / IN_SWING

-- Command slew, m/s per second. NOT feedback control -- it just shapes a yanked stick
-- into a ramp so the cargo stays settled; the UAC still flies whatever rate we send.
-- 0 disables (the UAC then chases each new stick position as hard as it can).
local VS_SLEW = 4.0

local TICK = 0.05   -- s between reads/broadcasts (matches the UAC control tick)

-- SETUP ----------------------------------------------------------------------
-- Open whatever modem is attached, on whichever side (the stick already occupies IN_SIDE,
-- so the modem cannot share it). rednet.broadcast needs an open side or it errors.
local modem = peripheral.find("modem")
if modem then
    rednet.open(peripheral.getName(modem))
else
    printError("No modem attached; cannot broadcast " .. PROTOCOL)
end

local function displayLine(row, text)
    term.setCursorPos(1, row)
    term.clearLine()
    term.write(text)
end

local function clamp(val, min, max)
    return math.max(min, math.min(max, val))
end

-- Read the stick. Level 7 is exactly zero and the input cannot drift off it by itself,
-- so there is nothing here for a deadband to do. Returns the raw level, the wanted RATE
-- (m/s) and whether this is a scramble; on a scramble the rate returned is the one the
-- proportional travel would give at full up-stick, so nothing downstream has to invent a
-- number -- but it is the FLAG, not that rate, that the UAC acts on.
local function readStick()
    local rawIn = redstone.getAnalogInput(IN_SIDE)
    if rawIn >= IN_SCRAMBLE then return rawIn, VS_MAX_CMD, true end
    local level = clamp(rawIn, IN_MIN, IN_MAX)
    return level, (level - IN_NEUTRAL) * MPS_PER_IN_STEP, false
end

-- MAIN -----------------------------------------------------------------------
term.clear()
displayLine(1, "Aer Togekiss vertical-speed commander")

local function commandLoop()
    local lastTime = os.clock()
    local cmdVelY  = 0
    while true do
        local now = os.clock()
        local dt  = math.max(now - lastTime, 0.001)
        lastTime  = now

        local stick, wantVelY, scramble = readStick()
        -- A SCRAMBLE FREEZES THE SLEW ACCUMULATOR RATHER THAN DRIVING IT. cmdVelY is a rate
        -- command being ramped at VS_SLEW; a scramble is not a rate, so there is nothing here
        -- for it to ramp toward -- and feeding it one hurts at both ends of the detent:
        --   * going in, the aircraft would spend seconds crawling up the ramp before the
        --     command even arrived, which is the opposite of what "as fast as possible" means;
        --   * coming out, the accumulator would be left parked at the scramble value and would
        --     take just as long to walk back down, so the stick would stay disconnected from
        --     the aeroplane long after the pilot had let go of it.
        -- Frozen, the accumulator still holds the rate the pilot last asked for, so dropping
        -- out of the detent resumes the proportional travel from exactly where it left off.
        if scramble then
            -- deliberately no-op: cmdVelY stays where the proportional stick left it
        elseif VS_SLEW > 0 then
            local maxStep = VS_SLEW * dt
            cmdVelY = cmdVelY + clamp(wantVelY - cmdVelY, -maxStep, maxStep)
        else
            cmdVelY = wantVelY
        end

        -- `or nil` so a non-scramble packet is byte-for-byte the shape it always was.
        rednet.broadcast({ vs = cmdVelY, scramble = scramble or nil }, PROTOCOL)

        -- Actual climb rate is the UAC's business now; show it for reference if the
        -- sublevel API happens to be wired here, otherwise skip it.
        local velY
        if sublevel then
            local ok, vel = pcall(sublevel.getLinearVelocity)
            if ok and vel then velY = vel.y end
        end

        displayLine(2, scramble
            and string.format("Stick: %2d  -> SCRAMBLE (full lift)", stick)
            or string.format("Stick: %2d  -> %+5.2f m/s", stick, wantVelY))
        displayLine(3, scramble
            and string.format("Cmd:  %+6.2f m/s  (held)", cmdVelY)
            or string.format("Cmd:  %+6.2f m/s%s", cmdVelY,
                math.abs(wantVelY - cmdVelY) > 0.01 and "  (slewing)" or ""))
        displayLine(4, velY and string.format("VelY: %+6.2f m/s (actual)", velY) or "VelY:   --")
        displayLine(5, "-> " .. PROTOCOL)

        sleep(TICK)
    end
end

parallel.waitForAny(commandLoop)
