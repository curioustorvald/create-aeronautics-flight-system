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

--[[local apnav, apalt = false
function switchAutopilot()
    local sender, msg, __ = rednet.receive("aertogekiss_apswitch", 1)
    if sender ~= nil then
        apnav = msg.autopilot_nav
        apalt = msg.autopilot_altitude
        displayLine(5, 'AP-NAV: '..tostring(apnav)..' AP-ALT: '..tostring(apalt))
    else
        displayLine(5, "No AP switch signal received")
    end
end]]

function headingThrust()
    local sender, msg, __ = rednet.receive("aertogekiss_navcontrol", 1)
    if sender ~= nil then
        local active = mst
        if active then
            redstone.setAnalogOutput("left", msg.yaw_left)
            redstone.setAnalogOutput("right", msg.yaw_right)
            redstone.setAnalogOutput("front", msg.thrust_forward)
            redstone.setAnalogOutput("back", msg.thrust_backward)
        end
        displayLine(2, string.format("Yaw: %2d  %2d  active: %s", msg.yaw_left, msg.yaw_right, active))
        displayLine(3, string.format("Thrust: %2d  %2d  active: %s", msg.thrust_forward, msg.thrust_backward, active))
    else
        displayLine(2, "No nav signal received")
        displayLine(3, "")
    end
end

displayLine(1, 'Aer Togekiss heading and thrust controller')
while true do
    parallel.waitForAny(headingThrust, switchEngine)
end
