a = peripheral.wrap('top')
local monitor = peripheral.find("monitor")
monitor.setCursorPos(1, 1)
monitor.clear()
monitor.setTextScale(0.5)
while true do
    t = a.getPressedKeyCodes()
    monitor.clear()
    for key, value in pairs(t) do -- simultaneous key presses will write that much characters
        monitor.setCursorPos(1, key)
        monitor.write(tostring(key)..': '..tostring(value))
    end
    sleep(0.05)
end
