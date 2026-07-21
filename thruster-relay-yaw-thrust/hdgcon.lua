rednet.open("top")
term.clear()

local function displayLine(row, text)
    term.setCursorPos(1, row)
    term.clearLine()
    term.write(text)
end

function headingThrust()
    local sender, msg, __ = rednet.receive("aertogekiss_navcontrol", 1)
    if sender ~= nil then
        redstone.setAnalogOutput("left", msg.yaw_left)
        redstone.setAnalogOutput("right", msg.yaw_right)
        redstone.setAnalogOutput("front", msg.thrust_forward)
        redstone.setAnalogOutput("back", msg.thrust_backward)
        displayLine(2, string.format("Yaw: %2d  %2d", msg.yaw_left, msg.yaw_right))
        displayLine(3, string.format("Thrust: %2d  %2d", msg.thrust_forward, msg.thrust_backward))
    else
        displayLine(2, "No nav signal received")
        displayLine(3, "")
    end
end

displayLine(1, 'Aer Togekiss heading and thrust controller')
while true do
    parallel.waitForAny(headingThrust)
end
