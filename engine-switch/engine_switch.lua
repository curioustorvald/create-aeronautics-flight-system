local PROTOCOL_ENGINE_SWITCH = 'aertogekiss_engineswitch'

if not sublevel then error("No sublevel API - is the Sable mod present?") end

local modem = peripheral.find("modem")
if not modem then error("No modem attached") end
rednet.open(peripheral.getName(modem))
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
        local extralift = redstone.getInput('back')

        rednet.broadcast({ ['master'] = mst, ['altitude'] = alt, ['attitude'] = att, ['extra_lift'] = extralift }, PROTOCOL_ENGINE_SWITCH)

        displayLine(1, "MASTER: "..tostring(mst))
        displayLine(2, "ALTITUDE: "..tostring(alt))
        displayLine(3, "ATTITUDE: "..tostring(att))
        displayLine(4, "EXTRA LIFT: "..tostring(extralift))

        sleep(0.1)
    end
end

parallel.waitForAny(relayLoop)
