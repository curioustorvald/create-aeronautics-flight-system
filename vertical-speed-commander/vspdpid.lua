-- VERTICAL SPEED COMMANDER
-- Reads the pilot's vertical-speed stick and broadcasts the WANTED climb rate (m/s) on
-- aertogekiss_mancon. It runs NO control of its own any more: the UAC (#1) owns the
-- vertical channel and turns this wanted rate into thruster commands (attitude-aware),
-- through the same inner climb-rate loop it uses for altitude-hold and autoland. This
-- computer drives NO redstone -- it is a rednet commander only.
--
-- INPUT:  analog level on IN_SIDE, 0..14. 7 = hold, 14 = climb at VS_MAX_CMD, 0 =
--         descend at VS_MAX_CMD, linear in between.
-- OUTPUT: rednet { vs = <m/s> } on aertogekiss_mancon. No thruster line.
--
-- CAUTION: an unpowered input line reads 0, which is a full-rate descent command. Nothing
-- here can tell a cut wire from a deliberate 0, so wire the stick so it fails to 7.

-- CONFIG ---------------------------------------------------------------------
local IN_SIDE  = "top"
local PROTOCOL = "aertogekiss_mancon"

-- Command input. CC hands back 0..15, so the top step is clamped away rather than being
-- read as a 15th of extra climb the scaling does not know about.
local IN_NEUTRAL = 7
local IN_MIN, IN_MAX = 0, 14
local IN_SWING = math.min(IN_NEUTRAL - IN_MIN, IN_MAX - IN_NEUTRAL)

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
-- so there is nothing here for a deadband to do.
local function readStick()
    local level = clamp(redstone.getAnalogInput(IN_SIDE), IN_MIN, IN_MAX)
    return level, (level - IN_NEUTRAL) * MPS_PER_IN_STEP
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

        local stick, wantVelY = readStick()
        if VS_SLEW > 0 then
            local maxStep = VS_SLEW * dt
            cmdVelY = cmdVelY + clamp(wantVelY - cmdVelY, -maxStep, maxStep)
        else
            cmdVelY = wantVelY
        end

        rednet.broadcast({ vs = cmdVelY }, PROTOCOL)

        -- Actual climb rate is the UAC's business now; show it for reference if the
        -- sublevel API happens to be wired here, otherwise skip it.
        local velY
        if sublevel then
            local ok, vel = pcall(sublevel.getLinearVelocity)
            if ok and vel then velY = vel.y end
        end

        displayLine(2, string.format("Stick: %2d  -> %+5.2f m/s", stick, wantVelY))
        displayLine(3, string.format("Cmd:  %+6.2f m/s%s", cmdVelY,
            math.abs(wantVelY - cmdVelY) > 0.01 and "  (slewing)" or ""))
        displayLine(4, velY and string.format("VelY: %+6.2f m/s (actual)", velY) or "VelY:   --")
        displayLine(5, "-> " .. PROTOCOL)

        sleep(TICK)
    end
end

parallel.waitForAny(commandLoop)
