-- Create: Aeronautics Flight System -- vectoring engine controller (one per nozzle; optional)
-- Only for an airframe flown by the UAC's vector-nozzle allocator (CFG.TRI_ENABLE -- see
-- uacconf-tri-section.lua). Identity and wiring live in vecconf.lua, one per engine.
shell.run("vecengine.lua")
