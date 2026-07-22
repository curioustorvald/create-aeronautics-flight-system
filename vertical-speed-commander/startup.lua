-- Aer Togekiss Flight System -- Vertical-Speed Commander (manual climb-rate stick)
-- Reads the pilot's VS stick and broadcasts the wanted climb rate (aertogekiss_mancon).
-- Drives no thrusters; the UAC executes the rate when altitude-hold is off.
shell.run("vspdpid.lua")
