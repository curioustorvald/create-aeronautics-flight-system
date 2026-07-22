local PROTOCOL_ENGINE_SWITCH = 'aertogekiss_engineswitch'

rednet.open('top')
term.clear()

local function displayLine(row, text)
    term.setCursorPos(1, row)
    term.clearLine()
    term.write(text)
end
local function relayLoop()
    while true do
        local mst = redstone.getInput('front')
        local alt = redstone.getInput('right')
        local att = redstone.getInput('left')
    
        rednet.broadcast({ ['master'] = mst, ['altitude'] = alt, ['attitude'] = att }, PROTOCOL_ENGINE_SWITCH)

        displayLine(1, "MASTER: "..tostring(mst))
        displayLine(2, "ALTITUDE: "..tostring(alt))
        displayLine(3, "ATTITUDE: "..tostring(att))
            
        sleep(0.1)
    end
end

parallel.waitForAny(relayLoop)
