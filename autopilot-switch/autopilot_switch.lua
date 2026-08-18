local PROTOCOL_AUTOPILOT_SWITCH = 'aertogekiss_apswitch'

rednet.open('top')
term.clear()

local function displayLine(row, text)
    term.setCursorPos(1, row)
    term.clearLine()
    term.write(text)
end

local alt = false
local nav = false

local function relayLoop1()
    while true do
        local sender, msg = rednet.receive(PROTOCOL_AUTOPILOT_SWITCH..'_1', 1)
        if sender and type(msg) == "table" and msg.autopilot_nav ~= nil then
            nav = msg.autopilot_nav
        end
        
        sleep(0.1)
    end
end

local function relayLoop2()
    while true do
        local sender, msg = rednet.receive(PROTOCOL_AUTOPILOT_SWITCH..'_2', 1)
        if sender and type(msg) == "table" and msg.autopilot_altitude ~= nil then
            alt = msg.autopilot_altitude
        end
        
        sleep(0.1)
    end
end

local function relayLoop()
    while true do
        rednet.broadcast({ ['autopilot_altitude'] = alt, ['autopilot_nav'] = nav }, PROTOCOL_AUTOPILOT_SWITCH)

        displayLine(1, "AP NAV: "..tostring(nav))
        displayLine(2, "AP ALT: "..tostring(alt))
            
        sleep(0.1)
    end
end

parallel.waitForAny(relayLoop, relayLoop1, relayLoop2)
