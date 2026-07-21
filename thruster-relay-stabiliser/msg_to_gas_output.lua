rednet.open("top")
term.clear()

local function displayLine(row, text)
    term.setCursorPos(1, row)
    term.clearLine()
    term.write(text)
end

function stabil()
    local sender, msg, __ = rednet.receive("aertogekiss_stab", 1)
    if sender ~= nil then
        redstone.setAnalogOutput("left", msg.left)
        redstone.setAnalogOutput("right", msg.right)
        redstone.setAnalogOutput("front", msg.front)
        redstone.setAnalogOutput("back", msg.back)
        displayLine(2, string.format("Stabil: %2d  %2d  %2d  %2d", msg.left, msg.right, msg.front, msg.back))
    else
        displayLine(2, "No stabil signal received")
    end
end

function alti()
    local sender, msg, __ = rednet.receive("aertogekiss_alt", 1)
    if sender ~= nil then
        redstone.setAnalogOutput("bottom", msg.alt)
        displayLine(3, string.format("Alti: %2d", msg.alt))
    else
        displayLine(3, "No alti signal received")
    end
end

displayLine(1, 'Aer Togekiss stabiliser controller')
while true do
    parallel.waitForAny(stabil)
end
