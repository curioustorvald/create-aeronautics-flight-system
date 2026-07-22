local PROTOCOL_AUTOPILOT_SWITCH = 'aertogekiss_apswitch'

rednet.open('top')
term.clear()

local function displayLine(row, text)
    term.setCursorPos(1, row)
    term.clearLine()
    term.write(text)
end
local function relayLoop()
    while true do
        local alt = redstone.getInput('right')
        local nav = redstone.getInput('left')
    
        rednet.broadcast({ ['autopilot_altitude'] = alt, ['autopilot_nav'] = nav }, PROTOCOL_AUTOPILOT_SWITCH)

        displayLine(1, "AP NAV: "..tostring(nav))
        displayLine(2, "AP ALT: "..tostring(alt))
            
        sleep(0.1)
    end
end

parallel.waitForAny(relayLoop)
