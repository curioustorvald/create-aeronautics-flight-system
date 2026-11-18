-- Create: Aeronautics Flight System -- ATC/TCAS responder (transponder), one per aircraft
-- Broadcasts this aircraft's position, altitude, speeds, heading and callsign on the
-- fleet-wide atc_broadcast. Set CALLSIGN/CSG and ALT_OFFSET at the top of responder.lua.
shell.run("responder.lua")
