-- Create: Aeronautics Flight System -- fore/aft thrust controller (optional)
-- Spends the UAC's raw signed-float thrust demand (aertogekiss_thrustf) across however many
-- redstone lines the fore/aft bank is wired to, for finer resolution than the yaw+thrust
-- relay's whole steps. Fill in WIRING at the top of thrustcon.lua.
shell.run("thrustcon.lua")
