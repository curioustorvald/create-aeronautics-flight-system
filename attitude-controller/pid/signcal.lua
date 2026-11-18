-- SIGN CALIBRATION RECORDER -- throwaway diagnostic, delete once #77/#80 are calibrated.
--
-- WHY THIS EXISTS. Scout went back to traditional control on 2026-08-28 (hand-written
-- pitch/roll relay on #77, yaw/strafe on #80, CFG.TRI_ENABLE false). Every sign calibration
-- in uacconf.lua -- PITCH_SIGN, ROLL_SIGN, YAW_SIGN, SWAP_PITCH_ROLL_CHANNELS -- was measured
-- against the NOZZLE allocator and says nothing about which way a fresh thruster bank fires.
-- Guessing them in the air is how you get the divergence that prompted this file: the
-- levelling loop answers a tilt with more tilt, so the craft wanders off one way and then
-- breaks up. This measures them on the ground instead.
--
-- IT COMMANDS NOTHING. Receive-only on the UAC's own output protocols, plus read-only
-- sublevel. Run it in a tab beside the UAC: on #72, `bg pid/signcal.lua` (or shell.openTab).
--
-- HOW TO USE IT -- three passes, ENGINE MASTER OFF for the first two so no thruster answers:
--
--   PASS 1, PITCH.  Tilt the craft nose-up and hold. Read the PITCH line: it shows the
--     measured tilt and the step the UAC is asking for. WRITE DOWN which line #77 lights
--     (its own screen prints `P<n> <side>` per frame). Nose-up must command the bank that
--     pushes the NOSE BACK DOWN. If it lights the one that pushes further nose-up, flip
--     CFG.PITCH_SIGN. Repeat nose-down and check it mirrors cleanly -- if it does not
--     mirror, the axis is tangled, not inverted, and the fix is SWAP_PITCH_ROLL_CHANNELS.
--
--   PASS 2, ROLL.  Same, rolled right. The LOW wing's bank must spool up. Flip
--     CFG.ROLL_SIGN if the high wing's does.
--
--   CROSS-CHECK.  If a PITCH tilt moves the ROLL command (or vice versa) more than it moves
--     its own, that is SWAP_PITCH_ROLL_CHANNELS, and no amount of sign-flipping fixes it.
--     The CORR column below is what to read: it is the running correlation between each
--     axis's measured tilt and its own commanded step.
--
--   PASS 3, YAW.  Engine master ON, hover clear of everything, and nudge the nose off the
--     held heading. Commanded yaw must oppose the error. If |HDG ERR| GROWS on its own,
--     that is the reported failure and CFG.YAW_SIGN is inverted -- land and flip it.
--
-- Everything is also appended to /signcal.csv (one row per UAC frame) so the passes can be
-- read back afterwards instead of off a moving screen. Delete the file between runs.

if not sublevel then error("No sublevel API - is the Sable mod present?") end
local modem = peripheral.find("modem")
if not modem then error("No modem attached") end
rednet.open(peripheral.getName(modem))

local CSV        = "/signcal.csv"
local STAB_PROTO = "aertogekiss_stab"
local NAV_PROTO  = "aertogekiss_navcontrol"
local RS_MAX     = 15      -- the wire carries fractions of a line; x15 puts them back in steps

local function wrapPi(a) return (a + math.pi) % (2 * math.pi) - math.pi end
local function euler(o)
    local a, b, c = o:toEuler()
    if type(a) == "number" then return a, b, c end
    return a.x, a.y, a.z
end

-- Running correlation between a measured tilt and its commanded step. Sign is the whole
-- point: NEGATIVE means the command opposes the tilt (a levelling loop, what we want after
-- the relay's own wiring is accounted for), POSITIVE means it reinforces it. Magnitude near
-- zero means the axis is not answering at all -- check the engine switch and the protocol.
local function makeCorr()
    local n, sx, sy, sxx, syy, sxy = 0, 0, 0, 0, 0, 0
    return function(x, y)
        n, sx, sy = n + 1, sx + x, sy + y
        sxx, syy, sxy = sxx + x*x, syy + y*y, sxy + x*y
        if n < 8 then return nil end
        local cx, cy = sxx - sx*sx/n, syy - sy*sy/n
        if cx <= 1e-9 or cy <= 1e-9 then return nil end
        return (sxy - sx*sy/n) / math.sqrt(cx*cy)
    end, function() n, sx, sy, sxx, syy, sxy = 0, 0, 0, 0, 0, 0 end
end
local corrPitch, resetPitch = makeCorr()
local corrRoll,  resetRoll  = makeCorr()

-- Latest commanded steps, filled by the listeners below.
local cmd = { pitch = 0, roll = 0, yaw = 0, strafe = 0, thrust = 0, seen = false }

local function listen(proto, apply)
    return function()
        while true do
            local sender, msg = rednet.receive(proto, 2)
            if sender and type(msg) == "table" then apply(msg); cmd.seen = true end
        end
    end
end

local stabListener = listen(STAB_PROTO, function(m)
    if type(m.pitch) == "number" then cmd.pitch = m.pitch * RS_MAX end
    if type(m.roll)  == "number" then cmd.roll  = m.roll  * RS_MAX end
end)
local navListener = listen(NAV_PROTO, function(m)
    if type(m.yaw)    == "number" then cmd.yaw    = m.yaw    * RS_MAX end
    if type(m.strafe) == "number" then cmd.strafe = m.strafe * RS_MAX end
    if type(m.thrust) == "number" then cmd.thrust = m.thrust * RS_MAX end
end)

local function row(t, text)
    term.setCursorPos(1, t); term.clearLine(); term.write(text)
end

local function sampler()
    local fresh = not fs.exists(CSV)
    local f = fs.open(CSV, fresh and "w" or "a")
    if fresh then f.writeLine("t,pitch_deg,roll_deg,hdg_deg,wx,wy,wz,cmd_pitch,cmd_roll,cmd_yaw") end
    f.flush()
    local t0 = os.clock()
    term.clear()
    row(1, "SIGN CAL -- receive-only. C=reset correlations, Q=quit")
    while true do
        local ok, pose = pcall(sublevel.getLogicalPose)
        if ok and pose then
            local p, y, r = euler(pose.orientation)
            local w = sublevel.getAngularVelocity()
            local t = os.clock() - t0
            local cp = corrPitch(p, cmd.pitch)
            local cr = corrRoll(r, cmd.roll)

            f.writeLine(string.format("%.2f,%.3f,%.3f,%.3f,%.4f,%.4f,%.4f,%.3f,%.3f,%.3f",
                t, math.deg(p), math.deg(r), math.deg(wrapPi(y)),
                w.x, w.y, w.z, cmd.pitch, cmd.roll, cmd.yaw))
            f.flush()

            row(3, cmd.seen and "link: ok" or "link: NO UAC OUTPUT HEARD -- is #72 running?")
            row(5, string.format("PITCH  tilt %+7.2f deg   cmd %+6.2f steps   CORR %s",
                math.deg(p), cmd.pitch, cp and string.format("%+.2f", cp) or " --"))
            row(6, string.format("ROLL   tilt %+7.2f deg   cmd %+6.2f steps   CORR %s",
                math.deg(r), cmd.roll, cr and string.format("%+.2f", cr) or " --"))
            row(7, string.format("YAW    hdg  %+7.2f deg   cmd %+6.2f steps   rate %+.3f",
                math.deg(wrapPi(y)), cmd.yaw, w.y))
            row(9, "CORR sign: - = command opposes tilt, + = reinforces it")
            row(10, string.format("rows written: %s", CSV))
        else
            row(3, "sublevel.getLogicalPose() failed -- is the craft assembled?")
        end
        sleep(0.05)
    end
end

-- NOT named `keys` -- that is CC's own keycode table, and a local of that name would shadow
-- it inside its own body, so `keys.c` would index the function and crash on the first press.
local function keyWatch()
    while true do
        local _, k = os.pullEvent("key")
        if k == keys.c then resetPitch(); resetRoll()
        elseif k == keys.q then return end
    end
end

parallel.waitForAny(sampler, keyWatch)
print("signcal stopped -- " .. CSV .. " kept.")
