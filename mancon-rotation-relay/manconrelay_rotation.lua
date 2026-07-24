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

local apnav, apalt = false
function switchAutopilot()
    while true do
        local sender, msg, __ = rednet.receive("aertogekiss_apswitch", 1)
        if sender ~= nil then
            apnav = msg.autopilot_nav
            apalt = msg.autopilot_altitude
            displayLine(5, 'AP-NAV: '..tostring(apnav)..' AP-ALT: '..tostring(apalt))
        else
            displayLine(5, "No AP switch signal received")
        end
    end
end

local function f(pred, str)
    if pred then return str else return '---' end
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

parallel.waitForAny(relayLoop, switchEngine, switchAutopilot)
