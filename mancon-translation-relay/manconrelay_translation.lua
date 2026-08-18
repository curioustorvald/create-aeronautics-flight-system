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

local function f(pred, str)
    if pred then return str else return '---' end
end

displayLine(1, 'Aer Togekiss Translational MANCON')
local function relayLoop()
    while true do
        local active = (not apnav) and mst
        
        if active then
            local aft = redstone.getInput('front')
            local fwd = redstone.getInput('back')
            local sleft = redstone.getInput('left')
            local sright = redstone.getInput('right')
            
            rednet.broadcast({
                ['aft'] = aft,
                ['fore'] = fwd,
                ['sleft'] = sleft,
                ['sright'] = sright
            }, 'aertogekiss_mancon_translational')
            
            displayLine(2, string.format('MANCON: %s %s %s %s', f(fwd, 'FWD'), f(aft, 'AFT'), f(sleft, 'LFT'), f(sright, 'RGT')))
        else
            displayLine(2, 'MANCON: disabled')

        end
        
        sleep(0.05)
    end
end

parallel.waitForAny(relayLoop, switchEngine, switchApnav, switchApalt)
