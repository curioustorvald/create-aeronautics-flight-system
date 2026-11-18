-- Create: Aeronautics Flight System -- HUD / GPWS
-- Head-up display and ground-proximity warning system. Receive-only: it controls nothing.
-- Terrain modes need the terrain data server (terrain-data-server/); the voice needs a
-- voice-pack floppy in an attached drive (see CFG.VOICE in gpwsconf.lua -- no clips ship here).
shell.run("gpws.lua")
