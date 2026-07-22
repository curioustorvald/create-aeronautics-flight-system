-- Aer Togekiss Flight System -- Autopilot Switch panel (flight-mode toggles)
-- Reads two physical redstone switches (nav / altitude) and broadcasts them
-- (aertogekiss_apswitch). The UAC picks its flight mode from these.
shell.run("autopilot_switch.lua")
