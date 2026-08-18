-- Aer Togekiss MANCON x MST toggler
-- MANCON and MST -> MANCON Intention Controller
-- where MIC converts intention to PID-controlled fore/aft/strafe thruster output

rednet.open("top")
term.clear()

local function displayLine(row, text)
    term.setCursorPos(1, row)
    term.clearLine()
    term.write(text)
end

local alt, att, mst = false
function switchEngine()
    while true do
        local sender, msg, __ = rednet.receive("aertogekiss_engineswitch", 1)
        if sender ~= nil then
            alt = msg.altitude
            att = msg.attitude
            mst = msg.master
            displayLine(4, 'MST: '..tostring(mst)..'  ALT: '..tostring(alt)..'  ATT: '..tostring(att))
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

local function touchDownDetectionLoop()
    while true do
        local td = redstone.getInput('back')
        rednet.broadcast({ ['touchedDown'] = td }, 'aertogekiss_touchdown_detection')
        displayLine(3, 'Touched down: '..tostring(td))
        
        sleep(0.1)
    end
end

parallel.waitForAny(relayLoop, touchDownDetectionLoop, switchEngine, switchApnav, switchApalt)
