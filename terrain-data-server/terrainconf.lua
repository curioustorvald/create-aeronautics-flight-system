-- TERRAIN DATA SERVER -- CONFIGURATION (computer #68)
-- =====================================================================================
-- Every protocol name, limit and geometry constant the terrain server uses. As with
-- uacconf.lua and apconf.lua, terrain_server.lua carries NO FALLBACK DEFAULTS: a knob
-- missing from this file is a named error at boot (see the canary list in the loader),
-- never a nil that turns into a wrong altitude over a mountain.
--
-- Loaded by require("terrainconf"), which resolves because startup.lua runs
-- shell.run("terrain_server.lua") and so the program directory is this one. Must end
-- with `return CFG`.
-- =====================================================================================

local CFG = {}

CFG.STATION = {
    NAME = "AER TERRAIN",
    -- Shown on the status screen and sent in the beacon, so a client can say WHICH server
    -- answered it. There is only one today; there is no reason there could not be two.
    ID   = "TDS-1",
}

-- =====================================================================================
-- LINKS
-- =====================================================================================
-- aer_terrain is FLEET-WIDE and is not to be renamed per aircraft, exactly like
-- aer_navdata and atc_broadcast. One database serves every airframe; a per-aircraft
-- protocol here would mean a per-aircraft database, which is the thing this exists to
-- avoid.
--
-- atc_broadcast is READ-ONLY here, as it is at the radar station. This computer never
-- transmits on it. It listens for one reason: to know where the fleet actually is, so
-- that terrainbuild.jar's incremental pass can be told to re-survey the ground the
-- aircraft are over rather than the whole 2596-region world.
CFG.LINKS = {
    TERRAIN  = "aer_terrain",
    ATC_CAST = "atc_broadcast",
}

-- =====================================================================================
-- THE DATABASE ON THE DISK
-- =====================================================================================
-- Built by terrainbuild.jar (beside the server launcher) from world/region/*.mca, and
-- read here. NOTHING IN THIS PROGRAM WRITES TO IT -- the builder owns those files and
-- swaps each one atomically, so a tile is either the old survey or the new one, never
-- half of each.
--
-- DRIVE may name a disk drive peripheral explicitly. Left nil, every drive is searched
-- for one whose mount holds DIR/META, which is what makes the disk portable between
-- drives without an edit here.
CFG.DB = {
    DRIVE = nil,
    DIR   = "terrain",
    META  = "meta.txt",
    -- HOW OFTEN THE SURVEY IS RE-READ, in seconds. terrainbuild.jar runs offline and swaps
    -- tiles under this program while it is running, and this program CACHES what it reads --
    -- including the misses, since most of the coordinate plane has no tile at all. Without a
    -- reload a region surveyed for the first time stayed "never surveyed" until the computer
    -- was rebooted, and the fleet drew black over ground the builder had already charted.
    -- Cheap: it drops the tile cache and re-reads meta.txt, and the next query re-reads only
    -- the tiles it actually needs, off a local disk. The cost is the first query after each
    -- reload paying for its own tiles again.
    -- 0 = never reload (a rebuilt survey then needs a reboot). Not defaulted anywhere: 0 is a
    -- legal value, and a defaulted one would look exactly like a working server quietly
    -- serving the survey it booted with.
    RELOAD = 30,
}

-- Geometry the program EXPECTS. meta.txt carries the geometry the database was actually
-- built with, and the two are compared at boot: a mismatch is a boot error naming the
-- field, because a database built at one Y resolution and read at another produces
-- confident, plausible and completely wrong heights.
CFG.GEO = {
    MIN_Y  = -64,   -- world floor; level 0 covers Y -64..-57
    CELL   = 8,     -- metres of Y per level -- the vertical resolution
    LEVELS = 48,    -- (320 - -64) / 8
    TILE   = 32,    -- chunks per tile edge; one tile == one .mca region
    HEADER = 32,    -- bytes of tile header before the records
    RECSZ  = 4,     -- bytes per chunk record: TERR, OTOP, OBAS, FLAG
}

-- Record flag bits, as terrainbuild.jar's ChunkAnalyzer writes them. Read with the
-- arithmetic helpers in the program rather than bit32, so nothing here depends on which
-- bit library the target VM ships.
CFG.FLAG = {
    MASS   = 3,     -- low two bits: 0 none, 1 thin, 2 medium, 3 large
    WATER  = 4,
    MULTI  = 8,
    TREES  = 16,
    SPIKE  = 32,
    VOID   = 64,
    NODATA = 128,
}

-- =====================================================================================
-- SERVING
-- =====================================================================================
-- The cache is the whole performance story. A corridor query 10 km long crosses about
-- twenty tiles; served cold that is twenty file reads, served warm it is twenty table
-- lookups. 96 tiles is roughly 400 kB of Lua string and covers several aircraft working
-- different parts of the map at once.
CFG.CACHE = { TILES = 96 }

-- Ceilings on what one query may ask for. A client asking for a 40 km corridor at 1 m
-- steps is a client with a bug, and answering it would stall this computer for everyone
-- else. Requests over the limit are CLAMPED, not refused, and the answer says what it
-- actually contains.
CFG.LIMITS = {
    SAMPLES  = 768,   -- samples in one corridor
    LATERAL  = 7,     -- chunks either side of the track that a corridor may sweep
    STEP_MIN = 8,     -- m between samples
    STEP_MAX = 256,

    -- ...and the same ceilings for the MAP query, which is a different shape of question:
    -- a north-up square of cells rather than a line of samples, asked for by the FMS so its
    -- HSI can paint terrain under the compass rose.
    --
    -- MAP_CELLS is the edge of the grid. The client picks the CELL SIZE from its own display
    -- scale, so a bigger grid buys RESOLUTION, not coverage.
    --
    -- It was 48 while a cell cost four bytes. The packed format (see Q.map) spends one byte
    -- a cell, so a 96 x 96 grid is now 9 kB -- less than the old 40 x 40 ever was -- and 128
    -- is the honest ceiling on what one rednet message should carry. That headroom is the
    -- whole point: a display drawing 2x3 subpixels per character needs roughly twice the
    -- grid an unsubdivided one does, or the extra pixels have nothing new to say.
    MAP_CELLS    = 128,
    MAP_STEP_MIN = 8,      -- m per grid cell; below a chunk the answer just repeats itself
    MAP_STEP_MAX = 2048,   -- ...and above this one cell spans a whole tile

    -- Each grid cell is reduced over a SUB x SUB lattice of chunks inside it rather than
    -- over every chunk it covers. At 256 m cells that is 16x16 = 256 chunk reads per cell
    -- and a quarter of a million for the grid, which is not a query, it is an outage. Three
    -- samples an axis keeps a peak from hiding in a large cell while keeping the cost flat
    -- in the cell size.
    MAP_SUB  = 3,
    -- The hard budget, in chunk reads, for one map. A request that would cost more has its
    -- SUB reduced until it fits -- CLAMPED, not refused, exactly like the corridor limits.
    MAP_WORK = 12000,
}

CFG.BEACON = { INTERVAL = 15 }   -- s between unsolicited "I am here" broadcasts

-- =====================================================================================
-- ACTIVITY -- what makes the incremental rebuild cheap.
-- =====================================================================================
-- Written to this COMPUTER's own filesystem (not the disk: the builder rewrites the
-- disk, and having both ends write the same mount is how accounting drifts). The jar
-- reads it with `update --near`, so a periodic re-survey touches the ground under the
-- fleet instead of all 2596 regions.
--
-- Two sources feed it: the position in every terrain query, and the responders on
-- atc_broadcast. The first covers aircraft actually using terrain data, the second
-- covers the rest of the fleet.
CFG.ACTIVITY = {
    FILE        = "activity.txt",
    WRITE_EVERY = 20,    -- s between rewrites
    KEEP        = 900,   -- s an aircraft stays on the roster after its last report
    MAX         = 32,    -- most aircraft listed; the freshest win
}

CFG.CLIENT_STALE = 60    -- s of silence before a client is shown as gone on the status page

CFG.TERM = { REDRAW = 0.5 }

return CFG
