rednet.open("top")
term.clear()

local function displayLine(row, text)
    term.setCursorPos(1, row)
    term.clearLine()
    term.write(text)
end

local alt, att, mst = false
function switchEngine()
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

local apnav, apalt = false
function switchAutopilot()
    local sender, msg, __ = rednet.receive("aertogekiss_apswitch", 1)
    if sender ~= nil then
        apnav = msg.autopilot_nav
        apalt = msg.autopilot_altitude
        displayLine(5, 'AP-NAV: '..tostring(apnav)..' AP-ALT: '..tostring(apalt))
    else
        displayLine(5, "No AP switch signal received")
    end
end

function stabil()
    local sender, msg, __ = rednet.receive("aertogekiss_stab", 1)
    local active = false
    if sender ~= nil then
        active = att and mst
        if active then
            redstone.setAnalogOutput("left", msg.left)
            redstone.setAnalogOutput("right", msg.right)
            redstone.setAnalogOutput("front", msg.front)
            redstone.setAnalogOutput("back", msg.back)
        end
        displayLine(2, string.format("Stabil: %2d  %2d  %2d  %2d", msg.left, msg.right, msg.front, msg.back))
    else
        displayLine(2, "No stabil signal received")
    end
    
    displayLine(3, string.format('active: %s', active))
end


displayLine(1, 'Aer Togekiss stabiliser controller')
while true do
    parallel.waitForAny(stabil, switchEngine, switchAutopilot)
end
