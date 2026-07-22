rednet.open("top")
term.clear()

-- Altitude thrusters, one redstone line each (the external splitter box is gone).
-- Rewire these to the sides your up- and down-firing thrusters actually sit on.
local UP_SIDE   = "front"
local DOWN_SIDE = "back"
local ALT_NEUTRAL = 7   -- balanced-hover level held on both lines when the link drops

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

function strafe()
    local sender, msg, __ = rednet.receive("aertogekiss_navcontrol", 1)
    if sender ~= nil then
        local active = mst
        if active then
            redstone.setAnalogOutput("left", msg.strafe_left)
            redstone.setAnalogOutput("right", msg.strafe_right)
        else
            redstone.setAnalogOutput("left", 0)
            redstone.setAnalogOutput("right", 0)
        end
        displayLine(2, string.format("Strafe: %2d  %2d  active: %s", msg.strafe_left, msg.strafe_right, active))
    else
        displayLine(2, "No strafe signal received")
    end
end

local function clamp(val, min, max)
    return math.max(min, math.min(max, val))
end

function alti()
    local sender, msg, __ = rednet.receive("aertogekiss_alt", 1)
    if sender ~= nil and msg.up ~= nil and msg.down ~= nil then
        local up   = clamp(msg.up, 0, 15)
        local down = clamp(msg.down, 0, 15)
        local active = alt and mst
        if active then
            redstone.setAnalogOutput(UP_SIDE, up)
            redstone.setAnalogOutput(DOWN_SIDE, down)
        else
            redstone.setAnalogOutput(UP_SIDE, 0)
            redstone.setAnalogOutput(DOWN_SIDE, 0)
        end
        displayLine(3, string.format("Alti: up %2d  down %2d  active: %s", up, down, tostring(active)))
    else
        -- Link lost: hold a balanced hover rather than plunging.
        if alt and mst then
            redstone.setAnalogOutput(UP_SIDE, ALT_NEUTRAL)
            redstone.setAnalogOutput(DOWN_SIDE, ALT_NEUTRAL)
        else
            redstone.setAnalogOutput(UP_SIDE, 0)
            redstone.setAnalogOutput(DOWN_SIDE, 0)
        end
        displayLine(3, string.format("No alti signal, holding %d/%d", ALT_NEUTRAL, ALT_NEUTRAL))
    end
end

displayLine(1, 'Aer Togekiss altitude and strafe controller')
while true do
    parallel.waitForAny(strafe, alti, switchEngine)
end
