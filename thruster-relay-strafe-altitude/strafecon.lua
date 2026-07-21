rednet.open("top")
term.clear()

local function displayLine(row, text)
    term.setCursorPos(1, row)
    term.clearLine()
    term.write(text)
end

function strafe()
    local sender, msg, __ = rednet.receive("aertogekiss_navcontrol", 1)
    if sender ~= nil then
        redstone.setAnalogOutput("left", msg.strafe_left)
        redstone.setAnalogOutput("right", msg.strafe_right)
        displayLine(2, string.format("Strafe: %2d  %2d", msg.strafe_left, msg.strafe_right))
    else
        displayLine(2, "No strafe signal received")
    end
end

local function clamp(val, min, max)
    return math.max(min, math.min(max, val))
end

function alti()
    local sender, msg, __ = rednet.receive("aertogekiss_alt", 1)
    if sender ~= nil then
        redstone.setAnalogOutput("front", clamp(msg.alt, 1, 14))
        displayLine(3, string.format("Alti: %2d", clamp(msg.alt, 1, 14)))
    else
        redstone.setAnalogOutput("front", 1)
        displayLine(3, "No alti signal received, emitting 1")
    end
end

displayLine(1, 'Aer Togekiss altitude and strafe controller')
while true do
    parallel.waitForAny(strafe, alti)
end
