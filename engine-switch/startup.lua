-- Aer Togekiss Flight System -- Engine Switch panel (hardware enables)
-- Reads three physical redstone switches (master / altitude / attitude) and broadcasts
-- them (aertogekiss_engineswitch). The relays gate their thruster output on these.
shell.run("engine_switch.lua")
