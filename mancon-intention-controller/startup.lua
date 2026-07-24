-- Aer Togekiss Flight System -- MANCON Intention Controller (MIC)
-- Integrates the pilot's fore/aft/strafe/yaw key "intentions" (from the two nav-stick relays)
-- into wanted body velocities (m/s, rad/s) and broadcasts them (aertogekiss_mancon_nav). The UAC
-- flies these when AP-NAV is off; a released axis commands 0, so the UAC holds heading / nulls drift.
-- Drives no thrusters -- like the VS-commander, it only NAMES a wanted velocity.
shell.run("mic.lua")
