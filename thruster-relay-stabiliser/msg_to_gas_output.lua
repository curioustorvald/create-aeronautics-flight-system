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
--[[function switchAutopilot()
    local sender, msg, __ = rednet.receive("aertogekiss_apswitch", 1)
    if sender ~= nil then
        apnav = msg.autopilot_nav
        apalt = msg.autopilot_altitude
        displayLine(5, 'AP-NAV: '..tostring(apnav)..' AP-ALT: '..tostring(apalt))
    else
        displayLine(5, "No AP switch signal received")
    end
end]]

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

function stabil()
    while true do
        local sender, msg, __ = rednet.receive("aertogekiss_stab", 1)
        local active = false
        if sender ~= nil then
            active = att and mst
            if active then
                redstone.setAnalogOutput("left", msg.left)
                redstone.setAnalogOutput("right", msg.right)
                redstone.setAnalogOutput("front", msg.front)
                redstone.setAnalogOutput("back", msg.back)
            else
                redstone.setAnalogOutput("left", 0)
                redstone.setAnalogOutput("right", 0)
                redstone.setAnalogOutput("front", 0)
                redstone.setAnalogOutput("back", 0)
            end
            displayLine(2, string.format("Stabil: %2d  %2d  %2d  %2d", msg.left, msg.right, msg.front, msg.back))
        else
            displayLine(2, "No stabil signal received")
        end
        
        displayLine(3, string.format('active: %s', active))
    end
end


displayLine(1, 'Aer Togekiss stabiliser controller')
parallel.waitForAny(stabil, switchEngine, switchApnav, switchApalt)
