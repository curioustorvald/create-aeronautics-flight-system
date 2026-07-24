-- Aer Togekiss Flight System -- Manual-Nav Rotation Relay (yaw stick)
-- Reads the pilot's yaw keys off redstone and broadcasts them
-- (aertogekiss_mancon_rotational) while manual control is live (master on, AP-NAV off).
-- The MIC integrates them into a wanted yaw rate; this computer drives no thrusters.
shell.run("manconrelay_rotation.lua")
