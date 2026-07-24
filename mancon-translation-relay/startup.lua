-- Aer Togekiss Flight System -- Manual-Nav Translation Relay (fore/aft/strafe stick)
-- Reads the pilot's translation keys off redstone and broadcasts them
-- (aertogekiss_mancon_translational) while manual control is live (master on, AP-NAV off).
-- The MIC integrates them into a wanted body velocity; this computer drives no thrusters.
shell.run("manconrelay_translation.lua")
