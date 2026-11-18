-- Aer Togekiss MANCON x MST toggler
-- MANCON and MST -> MANCON Intention Controller
-- where MIC converts intention to PID-controlled fore/aft/strafe thruster output

if not sublevel then error("No sublevel API - is the Sable mod present?") end

local modem = peripheral.find("modem")
if not modem then error("No modem attached") end
rednet.open(peripheral.getName(modem))
term.clear()

local function displayLine(row, text)
    term.setCursorPos(1, row)
    term.clearLine()
    term.write(text)
end

-- YAW RELAY GATE. This computer has three jobs: the yaw buttons of the four-computer MANCON
-- stack (aertogekiss_mancon_rotational, integrated by the MIC), touchdown detection
-- (aertogekiss_touchdown_detection, which the FMS and the GPWS both read as ground truth for a
-- landing) and the EXTRA LIFT bank, which repeats the UAC's up-thruster command onto its own
-- thrusters when the engine-switch panel's extra_lift input is on.
--
-- On an aircraft that hand-flies through the ONE-computer MANCON (mancon/mancon.lua) instead,
-- yaw comes off that computer's typewriter and nobody listens to the rotational channel: set
-- this false there. Touchdown detection and EXTRA LIFT keep running either way, which is why
-- the computer stays installed even when the button panel is gone.
local YAW_RELAY_ENABLED = true

local alt, att, mst, extralift = false
function switchEngine()
    while true do
        local sender, msg, __ = rednet.receive("aertogekiss_engineswitch", 1)
        if sender ~= nil then
            alt = msg.altitude
            att = msg.attitude
            mst = msg.master
            extralift = msg.extra_lift
            displayLine(4, 'MST: '..tostring(mst)..'  ALT: '..tostring(alt)..'  ATT: '..tostring(att)..'  EXLFT: '..tostring(extralift))
        else
            displayLine(4, "No engine switch signal received")
        end
    end
end

local function f(pred, str)
    if pred then return str else return '---' end
end

local apnav, apalt = false
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

displayLine(1, 'Aer Togekiss Rotational MANCON')
local function relayLoop()
    if not YAW_RELAY_ENABLED then
        displayLine(2, 'MANCON yaw: OFF (YAW_RELAY_ENABLED = false)')
        while true do sleep(1) end
    end
    while true do
        local active = (not apnav) and mst
        
        if active then
            local yleft = redstone.getInput('left')
            local yright = redstone.getInput('right')
            
            rednet.broadcast({
                ['yleft'] = yleft,
                ['yright'] = yright
            }, 'aertogekiss_mancon_rotational')
            
            displayLine(2, string.format('MANCON: %s %s',f(yleft, 'LFT'), f(yright, 'RGT')))
        else
            displayLine(2, 'MANCON: disabled')

        end
        
        sleep(0.05)
    end
end


-- EXTRA LIFT reads the UAC's up-thruster command off the ALT protocol and repeats it onto
-- its own bank. Since 2026-08-26 that command arrives as a FLOAT, 0..1 -- the fraction of a
-- full redstone line -- with a `cut` flag beside it, so this computer has to quantise it the
-- same way the altitude relay does. Dropping a 0..1 fraction straight into
-- setAnalogOutput truncates it to 0 and the bank simply never fires.
--
-- One sigma-delta accumulator, on the command rate rather than on extraLiftLoop's 20 Hz
-- repeat: the fraction below a whole step is carried and paid out a frame or two later, and
-- the airframe integrates it back into the value asked for.
EXTRALIFT_STEP_MAX = 15
EXTRALIFT_DITHER   = true

local function elClamp(v, lo, hi) return math.max(lo, math.min(hi, v)) end
local function elNum(v) return type(v) == "number" and v == v end
local elAcc = 0
local function elQuantise(f)
    f = elClamp(f, 0, 1) * EXTRALIFT_STEP_MAX
    if not EXTRALIFT_DITHER then return elClamp(math.floor(f + 0.5), 0, EXTRALIFT_STEP_MAX) end
    local want = f + elAcc
    local q = elClamp(math.floor(want + 0.5), 0, EXTRALIFT_STEP_MAX)
    elAcc = elClamp(want - q, -0.5, 0.5)
    return q
end

local liftSig = 0
function alti2()
    while true do
        local sender, msg, __ = rednet.receive("aertogekiss_alt", 1)
        if sender ~= nil and type(msg) == "table" and elNum(msg.up) and elNum(msg.down)
                and type(msg.cut) == "boolean" then
            liftSig = msg.cut and 0 or elQuantise(msg.up)
            if msg.cut then elAcc = 0 end
            displayLine(8, string.format('ALTI: UP %.3f  DN %.3f  -> %d',
                msg.up, msg.down, liftSig))
        elseif sender ~= nil and type(msg) == "table" and elNum(msg.up) and elNum(msg.down) then
            -- TRANSITIONAL, and delete it once every UAC on the fleet runs the float build:
            -- an old UAC sends the same two field names as whole 0..15 levels and no `cut`.
            -- The flag is the only thing that tells the two apart.
            liftSig = elClamp(math.floor(msg.up + 0.5), 0, 15)
            elAcc = 0
            displayLine(8, string.format('ALTI: LEGACY UP %d  DN %d',
                liftSig, elClamp(math.floor(msg.down + 0.5), 0, 15)))
        else
            liftSig = 0
            elAcc = 0
            displayLine(8, string.format('ALTI: UP --  DN --'))
        end

    end
end

local function extraLiftLoop()
    while true do
        local active = extralift and mst

        if active then
            redstone.setAnalogOutput('front', liftSig)
            displayLine(7, string.format('EXTRALIFT: %d', liftSig))
        else
            redstone.setAnalogOutput('front', 0)
            displayLine(7, string.format('EXTRALIFT: %d', 0))
            displayLine(7, string.format('EXTRALIFT: --'))
        end

        sleep(0.05)
    end
end

local function touchDownDetectionLoop()
    while true do
        local td = redstone.getInput('back')
        rednet.broadcast({ ['touchedDown'] = td }, 'aertogekiss_touchdown_detection')
        displayLine(3, 'Touched down: '..tostring(td))
        
        sleep(0.1)
    end
end

parallel.waitForAny(relayLoop, touchDownDetectionLoop, switchEngine, switchApnav, switchApalt, extraLiftLoop, alti2)
