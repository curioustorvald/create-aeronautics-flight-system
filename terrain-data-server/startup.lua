-- Create: Aeronautics Flight System -- Terrain Data Server (ground station, one per world)
-- Serves the offline-built terrain survey (terrain/ on a disk in an attached drive) fleet-wide
-- on aer_terrain: a periodic beacon plus request/response corridor and map queries. Powers the
-- terrain-closure GPWS modes (gpws-hud/) and the FMS's terrain avoidance and HSI terrain map.
-- The survey is built OUTSIDE the game by terrainbuild.jar -- see TERRAIN.md.
shell.run("terrain_server.lua")
