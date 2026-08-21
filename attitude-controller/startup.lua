-- Unified Attitude Controller (quaternion). Supersedes the old gimbal stabiliser.
-- To fall back to the old controller, comment the line below and uncomment control.lua.
shell.run("pid/uac.lua")
--shell.run("pid/control.lua")
--[[
while true do
    local result = shell.run("pid/control.lua")
    if not result then
        print("Exit with error, retrying")
        sleep(1)
    else
        break
    end
end
print()
print("LX4 cya!")]]
