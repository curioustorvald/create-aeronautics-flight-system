-- TERRAIN DATA SERVER -- computer #68
-- =====================================================================================
-- The fleet's ground-proximity database, and the only thing in this world that can tell an
-- aeroplane where the ground is.
--
-- WHY IT EXISTS. Create: Aeronautics gives an airframe its own pose off sublevel, but
-- nothing gives it the SHAPE OF THE WORLD. Every terrain-derived GPWS mode on #67 -- terrain
-- closure, the true clearance form of Mode 4, TERRAIN AHEAD, OBSTACLE AHEAD -- was therefore
-- wired but not armed, because a warning derived from a guess is worse than no warning at
-- all. This computer is the missing half.
--
-- WHERE THE DATA COMES FROM. Not from in-game scanning: the overworld here is 2596 regions
-- and 2.06 million chunks, and no CC computer is going to walk that. terrainbuild.jar, which
-- sits beside the server launcher, reads world/region/*.mca directly -- Anvil, NBT, palette
-- and all -- and writes one 4128-byte tile per region onto the disk in the drive. This
-- program only ever READS those files. The builder swaps each tile atomically, so a tile is
-- either last week's survey or today's, never half of each, and this program needs no
-- locking to be safe against a rebuild happening underneath it.
--
-- THE SHAPE OF AN ANSWER. One record per chunk (16 x 16 m) and one level per 8 m of Y:
--
--     TERR  the top of the GROUND, as a level. Rounded UP, so it errs high.
--     OTOP  the top of anything FLOATING above that ground, or none.
--     OBAS  the underside of the lowest floating thing, or none.
--     FLAG  mass class, water, multiple layers, canopy, spike, void, no-data.
--
-- Terrain and obstacle are separated in the BUILDER, not here -- see ChunkAnalyzer -- so
-- that the distinction is made once, against the real blocks, rather than re-guessed by
-- every client.
--
-- PROTOCOL "aer_terrain" -- fleet-wide, request/response, and this end is stateless.
--
--   srv -> all    {t="beacon", id, rev, tiles, geo}      every BEACON.INTERVAL
--   cli -> all    {t="hello", callsign}                  answered with a targeted beacon
--   cli -> srv    {t="corr", id, x, z, dx, dz, step, n, w, alt, callsign}
--   srv -> cli    {t="corr", id, x, z, dx, dz, step, n, w, prof}
--                 prof is 4 bytes per sample -- TERR, OTOP, OBAS, FLAG -- reduced across
--                 the corridor width, so the client gets the WORST ground in the swathe
--                 rather than a single line of it.
--   cli -> srv    {t="map", id, x, z, cell, nx, nz, sub, pack, alt, callsign}
--   srv -> cli    {t="map", id, x0, z0, cell, nx, nz, sub, grid}            (pack unset)
--   srv -> cli    {t="map", id, x0, z0, cell, nx, nz, sub, pack, grid, obs} (pack set)
--                 With pack, grid is ONE byte per cell -- 255 never surveyed, else the
--                 terrain level in bits 0-5 (63 = charted, no ground) and water in bit 6 --
--                 and obs is a sparse list of 5-byte records, {idxLo, idxHi, OTOP, OBAS,
--                 mass}, for the 1-2% of cells that carry a floating mass. A quarter of the
--                 width, which is what makes a grid fine enough for subpixel drawing
--                 affordable. Without pack, grid is 4 bytes per cell -- TERR, OTOP, OBAS,
--                 FLAG -- as it always was.
--                 Either way the block is NORTH-UP,
--                 axis-aligned nx by nz block of `cell`-metre squares whose min corner is
--                 (x0, z0), each cell reduced to the worst chunk under it. Row-major in +Z:
--                 cell (i, j) is at offset (j*nx + i) * 4. North-up rather than heading-up
--                 on purpose: the grid then stays valid through a turn, and the client
--                 rotates it into its own display frame for free.
--   cli -> srv    {t="tile", rx, rz}   ->  {t="tile", rx, rz, data}   raw tile, or nil
--   cli -> srv    {t="gnd",  x, z}     ->  {t="gnd",  x, z, rec}      one record, or nil
--
-- It answers a broadcast as readily as a direct send, so a client that has not yet learnt
-- this computer's id is not stuck waiting for a beacon.
--
-- IT CONTROLS NOTHING. Like the radar station and the HUD, every link here is receive-only
-- in the sense that matters: nothing this computer says can move a thruster. The worst a
-- wrong answer can do is make #67 speak, and #67 cannot fly the aeroplane either.
-- =====================================================================================

local CFG
local LINKS, GEO, FLAG, LIMITS, DBC, ACTC
do
    local ok, res = pcall(require, "terrainconf")
    if not ok then
        error("Cannot load terrainconf.lua (the terrain server configuration): " .. tostring(res)
            .. "\nIt must sit beside terrain_server.lua and end with 'return CFG'.", 0)
    end
    if type(res) ~= "table" then
        error("terrainconf.lua did not return the CFG table (got " .. type(res) .. ").", 0)
    end
    for _, k in ipairs({ "STATION", "LINKS", "DB", "GEO", "FLAG", "CACHE", "LIMITS",
            "BEACON", "ACTIVITY", "CLIENT_STALE", "TERM" }) do
        if res[k] == nil then
            error("terrainconf.lua is missing CFG." .. k .. " -- the file looks incomplete.", 0)
        end
    end
    for t, ks in pairs({
            STATION  = { "NAME", "ID" },
            LINKS    = { "TERRAIN", "ATC_CAST" },
            -- RELOAD added 2026-09-08 and read by DB.loop, which decides whether the server
            -- ever notices a rebuilt survey. 0 is a legal value (never reload), so a defaulted
            -- one would look exactly like a working server serving the survey it booted with.
            DB       = { "DIR", "META", "RELOAD" },
            GEO      = { "MIN_Y", "CELL", "LEVELS", "TILE", "HEADER", "RECSZ" },
            FLAG     = { "MASS", "WATER", "MULTI", "TREES", "SPIKE", "VOID", "NODATA" },
            CACHE    = { "TILES" },
            LIMITS   = { "SAMPLES", "LATERAL", "STEP_MIN", "STEP_MAX",
                         "MAP_CELLS", "MAP_STEP_MIN", "MAP_STEP_MAX", "MAP_SUB", "MAP_WORK" },
            BEACON   = { "INTERVAL" },
            ACTIVITY = { "FILE", "WRITE_EVERY", "KEEP", "MAX" },
            TERM     = { "REDRAW" },
        }) do
        if type(res[t]) ~= "table" then
            error("terrainconf.lua's CFG." .. t .. " is not a table.", 0)
        end
        for _, k in ipairs(ks) do
            if res[t][k] == nil then
                error("terrainconf.lua's CFG." .. t .. " is missing " .. k .. ".", 0)
            end
        end
    end
    CFG = res
    LINKS, GEO, FLAG, LIMITS = CFG.LINKS, CFG.GEO, CFG.FLAG, CFG.LIMITS
    DBC, ACTC = CFG.DB, CFG.ACTIVITY
end

-- =====================================================================================
-- SMALL HELPERS
-- =====================================================================================

local function clamp(v, lo, hi) return math.max(lo, math.min(hi, v)) end

-- Y of the TOP of a level. Every height this server reports is the top of its cell, which
-- rounds the ground UP by as much as CELL-1 metres. That is the safe direction and it is
-- deliberate: a GPWS that thinks the hill is 7 m taller than it is calls too early, one
-- that thinks it is 7 m shorter calls too late.
local function levelTop(L) return GEO.MIN_Y + GEO.CELL * L + (GEO.CELL - 1) end
local function levelBase(L) return GEO.MIN_Y + GEO.CELL * L end

-- Flag bits, tested by arithmetic rather than bit32, so this does not care which bit
-- library the VM ships with.
local function hasFlag(f, bit) return math.floor(f / bit) % 2 == 1 end

-- =====================================================================================
-- PERIPHERALS
-- =====================================================================================

local HW = { faults = {} }
do
    HW.modem = peripheral.find("modem", function(_, m) return m.isWireless and m.isWireless() end)
             or peripheral.find("modem")
    if not HW.modem then
        error("No modem attached -- the fleet could not reach this server.\n"
            .. "An ENDER modem is wanted: a plain wireless one reaches 384 m at best and the\n"
            .. "aircraft ask from tens of kilometres away.", 0)
    end
    rednet.open(peripheral.getName(HW.modem))
    HW.wireless = HW.modem.isWireless and HW.modem.isWireless() or false

    HW.mon = peripheral.find("monitor")
    if HW.mon then pcall(function() HW.mon.setTextScale(0.5) end) end
end

-- =====================================================================================
-- THE DATABASE
-- =====================================================================================

local DB = {
    dir = nil, meta = {}, cache = {}, seq = 0, held = 0,
    hits = 0, misses = 0, reads = 0, bytes = 0,
    reloads = 0, lastReload = 0, fault = nil, frozen = false,
}

-- Find the mount that actually holds a database, rather than trusting a hardcoded /disk.
-- The disk can be moved to another drive without an edit here, which matters because the
-- drive is a block someone can break.
function DB.locate()
    local tries = {}
    if DBC.DRIVE then
        local d = peripheral.wrap(DBC.DRIVE)
        if d and d.getMountPath then tries[#tries + 1] = d.getMountPath() end
    else
        for _, d in ipairs({ peripheral.find("drive") }) do
            if d.getMountPath then tries[#tries + 1] = d.getMountPath() end
        end
        -- Last resort: a disk already mounted whose drive we cannot see from here.
        for _, n in ipairs(fs.list("/")) do
            if fs.isDir("/" .. n) then tries[#tries + 1] = n end
        end
    end
    for _, m in ipairs(tries) do
        if m then
            local dir = "/" .. m .. "/" .. DBC.DIR
            if fs.exists(dir .. "/" .. DBC.META) then return dir end
        end
    end
    return nil
end

function DB.loadMeta()
    local h = fs.open(DB.dir .. "/" .. DBC.META, "r")
    if not h then return false, "cannot open " .. DBC.META end
    local m = {}
    while true do
        local line = h.readLine()
        if not line then break end
        line = line:match("^%s*(.-)%s*$")
        if line ~= "" and line:sub(1, 1) ~= ";" then
            local k, v = line:match("^([%w_]+)=(.*)$")
            if k then m[k] = tonumber(v) or v end
        end
    end
    h.close()
    DB.meta = m

    -- The database states the geometry it was built with. If it disagrees with what this
    -- program expects, every height it serves would be wrong by a believable amount, which
    -- is the worst kind of wrong. Refuse to start.
    local want = { minY = GEO.MIN_Y, cell = GEO.CELL, levels = GEO.LEVELS,
                   tile = GEO.TILE, header = GEO.HEADER, record = GEO.RECSZ }
    for k, v in pairs(want) do
        if m[k] ~= nil and m[k] ~= v then
            return false, string.format("%s says %s=%s but this program is built for %s"
                .. " -- rebuild the database or fix CFG.GEO", DBC.META, k, tostring(m[k]), tostring(v))
        end
    end
    return true
end

-- PICK UP A REBUILD WITHOUT A REBOOT. terrainbuild.jar runs offline and swaps tiles under us;
-- this program only ever READS them, and it caches what it reads -- including the MISSES, since
-- most of the coordinate plane has no tile (see DB.tile). So until this existed, a region
-- surveyed for the first time stayed "never surveyed" for as long as the computer was up, and
-- the fleet drew black over ground the builder had already charted. A survey is exactly the
-- thing that arrives while the server is running.
--
-- Dropping the cache is the whole mechanism -- there is nothing to re-read eagerly, because the
-- next query re-reads what it actually needs and the disk is local. The meta is re-read too:
-- it carries the survey's own `built` stamp and tile count, which go out on every beacon, so
-- that is how the fleet is told the map changed.
--
-- FAILURE IS NOT FATAL. Every path here keeps the last good database rather than throwing:
-- clients are flying on this data, and a server that exits because someone pulled the disk out
-- for a moment is worse than one serving a slightly old survey.
--   * disk gone / no meta -> keep dir, meta AND cache, raise a fault, try again next time.
--   * geometry changed    -> the tiles on that disk are in a geometry this program is not
--                            built for, and heights read out of them would be confident,
--                            plausible and completely wrong -- the same reason it is a BOOT
--                            error. Keep what is cached (it is in the right geometry), stop
--                            reading new tiles (DB.frozen) and say so. It clears itself if the
--                            disk is put back.
function DB.reload()
    local dir = DB.locate()
    if not dir then
        DB.fault = "database not found -- serving the cached survey"
        return false
    end
    local wasDir, wasMeta = DB.dir, DB.meta
    DB.dir = dir
    local ok, err = DB.loadMeta()
    if not ok then
        -- loadMeta ASSIGNS DB.meta before it validates the geometry, so put the old one back:
        -- the beacon advertises meta.built and meta.tiles to the whole fleet, and a survey we
        -- have refused to read must not be announced as the one being served.
        DB.dir, DB.meta = wasDir, wasMeta
        DB.frozen = true
        DB.fault = tostring(err)
        return false
    end
    DB.cache, DB.held, DB.seq = {}, 0, 0
    DB.frozen = false
    DB.fault = nil
    DB.reloads = DB.reloads + 1
    DB.lastReload = os.clock()
    return true
end

-- NOTE the shape: this is one of parallel.waitForAny's functions, and waitForAny finishes the
-- moment ANY of them RETURNS -- so a disabled reload has to park here forever, not return, or
-- switching it off would shut the whole server down at boot.
function DB.loop()
    while true do
        local period = DBC.RELOAD or 0
        if period <= 0 then
            sleep(60)
        else
            sleep(period)
            pcall(DB.reload)
        end
    end
end

function DB.put(key, data)
    if DB.cache[key] == nil then DB.held = DB.held + 1 end
    DB.seq = DB.seq + 1
    DB.cache[key] = { d = data, u = DB.seq }
    if DB.held > CFG.CACHE.TILES then
        local oldK, oldU
        for k, e in pairs(DB.cache) do
            if not oldU or e.u < oldU then oldK, oldU = k, e.u end
        end
        if oldK then DB.cache[oldK] = nil DB.held = DB.held - 1 end
    end
end

-- Returns the tile's raw bytes, or false when the region was never surveyed. A MISS IS
-- CACHED TOO: most of the coordinate plane has no tile (the world is 2596 regions inside a
-- span of forty thousand), and without negative caching every query over open nothing would
-- stat the filesystem once per chunk.
function DB.tile(rx, rz)
    local key = rx .. ":" .. rz
    local e = DB.cache[key]
    if e ~= nil then
        DB.hits = DB.hits + 1
        DB.seq = DB.seq + 1
        e.u = DB.seq
        return e.d
    end
    -- FROZEN: the database on the disk is not in the geometry this program reads (DB.reload).
    -- Answer "not surveyed" rather than reading it -- and do not cache that answer, or it
    -- would outlive the fault. Ahead of the miss counter: this is not a cache miss, it is the
    -- database being unavailable.
    if DB.frozen then return false end
    DB.misses = DB.misses + 1

    local data = false
    local path = DB.dir .. "/" .. rx .. "_" .. rz .. ".agt"
    if fs.exists(path) then
        local h = fs.open(path, "rb")
        if h then
            local s = h.readAll()
            h.close()
            DB.reads = DB.reads + 1
            if s and #s >= GEO.HEADER + GEO.TILE * GEO.TILE * GEO.RECSZ
                 and s:sub(1, 4) == "AGT1" then
                data = s
                DB.bytes = DB.bytes + #s
            end
        end
    end
    DB.put(key, data)
    return data
end

-- One chunk. Returns TERR, OTOP, OBAS, FLAG, or nil when that chunk is not charted.
function DB.rec(cx, cz)
    local rx = math.floor(cx / GEO.TILE)
    local rz = math.floor(cz / GEO.TILE)
    local t = DB.tile(rx, rz)
    if not t then return nil end
    local lx, lz = cx - rx * GEO.TILE, cz - rz * GEO.TILE
    local o = GEO.HEADER + (lz * GEO.TILE + lx) * GEO.RECSZ + 1
    local terr, otop, obas, f = t:byte(o, o + 3)
    if not f or hasFlag(f, FLAG.NODATA) then return nil end
    return terr, otop, obas, f
end

-- =====================================================================================
-- QUERIES
-- =====================================================================================

local Q = { corr = 0, tile = 0, gnd = 0, maps = 0, samples = 0, cells = 0 }

-- The corridor. For each sample along the track it sweeps LATERAL chunks either side and
-- keeps the WORST of them -- highest ground, highest obstacle top, lowest obstacle base --
-- so a client flying a 240 m wide swathe is told about the ridge beside its track, not just
-- the one it happens to be pointed at. Reducing here rather than on the aircraft is the
-- point of having a server: this end has the tiles in memory already.
function Q.corridor(msg)
    local x, z = tonumber(msg.x), tonumber(msg.z)
    local dx, dz = tonumber(msg.dx), tonumber(msg.dz)
    if not (x and z and dx and dz) then return nil end
    local mag = math.sqrt(dx * dx + dz * dz)
    if mag < 1e-6 then return nil end
    dx, dz = dx / mag, dz / mag

    local step = clamp(tonumber(msg.step) or 16, LIMITS.STEP_MIN, LIMITS.STEP_MAX)
    local n    = math.floor(clamp(tonumber(msg.n) or 64, 1, LIMITS.SAMPLES))
    local lat  = math.floor(clamp((tonumber(msg.w) or 0) / 16, 0, LIMITS.LATERAL))

    local px, pz = -dz, dx          -- unit normal to the track
    local out = {}
    for i = 0, n - 1 do
        local sx, sz = x + dx * step * i, z + dz * step * i
        local terr, otop, obas, mass = -1, -1, 256, 0
        local multi, trees, spike, any = false, false, false, false
        local seen, wet = 0, 0

        for k = -lat, lat do
            local qx, qz = sx + px * k * 16, sz + pz * k * 16
            local a, b, c, f = DB.rec(math.floor(qx / 16), math.floor(qz / 16))
            if a then
                any = true
                seen = seen + 1
                if a ~= 255 and a > terr then terr = a end
                if b ~= 255 then
                    if b > otop then otop = b end
                    if c < obas then obas = c end
                    local m = f % (FLAG.MASS + 1)
                    if m > mass then mass = m end
                    if hasFlag(f, FLAG.MULTI) then multi = true end
                end
                if hasFlag(f, FLAG.WATER) then wet = wet + 1 end
                if hasFlag(f, FLAG.TREES) then trees = true end
                if hasFlag(f, FLAG.SPIKE) then spike = true end
            end
        end
        -- WATER means ALL of it is water, not "some of it touched water" -- see the note on
        -- the same reduction in Q.map. Everything else here reduces to the WORST sample
        -- because everything else is a hazard; water is a terrain TYPE, and the honest
        -- reduction of a type over a mixed area is "mixed", which is to say: not water.
        local water = (seen > 0 and wet == seen)

        if not any then
            out[#out + 1] = string.char(255, 255, 255, FLAG.NODATA)
        else
            local f = mass
            if water then f = f + FLAG.WATER end
            if multi then f = f + FLAG.MULTI end
            if trees then f = f + FLAG.TREES end
            if spike then f = f + FLAG.SPIKE end
            out[#out + 1] = string.char(terr < 0 and 255 or terr,
                                        otop < 0 and 255 or otop,
                                        obas > 255 and 255 or obas, f)
        end
    end
    Q.samples = Q.samples + n
    return { t = "corr", srv = true, id = msg.id, x = x, z = z, dx = dx, dz = dz,
             step = step, n = n, w = lat * 16, prof = table.concat(out) }
end

-- The map. The corridor answers "what is along my track"; this answers "what is AROUND me",
-- which is the question a moving-map instrument asks. One north-up, axis-aligned block of
-- square cells centred on the client, each cell carrying the same four bytes a chunk record
-- does, reduced to the WORST chunk under it.
--
-- WHY NORTH-UP AND NOT HEADING-UP. The instrument at the other end is heading-up, so it
-- would be less work here to cut the grid already rotated. It would also be wrong: a
-- heading-up grid is invalidated by every degree of bank, and an aeroplane in a turn is
-- exactly when the display must not go blank. Axis-aligned, the same grid stays true through
-- the whole turn and the client rotates it as it paints, which costs it nothing it was not
-- already doing to place a symbol.
--
-- WHY THE WORST AND NOT THE MEAN. Same reason the corridor reduces the way it does, and the
-- same reason the builder reports the TOP of a level: a cell 128 m across holds sixty-four
-- chunks, and the one that matters is the tall one. Averaging a peak into its neighbours is
-- how a terrain display shows clear ground where there is a mountain.
--
-- SUB-SAMPLING. Reading every chunk under a large cell is quadratic in the cell size and
-- runs to a quarter of a million reads at the top of the range. The lattice is capped at
-- LIMITS.MAP_SUB per axis and then reduced further, here, until the whole grid fits inside
-- LIMITS.MAP_WORK reads. The client is told what it got (`sub` comes back in the reply) so a
-- coarse answer is visible as a coarse answer rather than as missing terrain.
function Q.map(msg)
    local x, z = tonumber(msg.x), tonumber(msg.z)
    if not (x and z) then return nil end
    local cell = clamp(tonumber(msg.cell) or 64, LIMITS.MAP_STEP_MIN, LIMITS.MAP_STEP_MAX)
    local nx   = math.floor(clamp(tonumber(msg.nx) or 32, 1, LIMITS.MAP_CELLS))
    local nz   = math.floor(clamp(tonumber(msg.nz) or 32, 1, LIMITS.MAP_CELLS))
    local sub  = math.floor(clamp(tonumber(msg.sub) or LIMITS.MAP_SUB, 1, LIMITS.MAP_SUB))
    -- Never sample a cell finer than the database is: a 16 m cell holds exactly one chunk,
    -- and a 3x3 lattice inside it reads that same record nine times for the same answer.
    -- This costs nothing to check and is what makes a close-in, high-resolution map cheap --
    -- which is exactly the map an aircraft asks for near the ground.
    sub = math.min(sub, math.max(1, math.ceil(cell / 16)))
    while sub > 1 and nx * nz * sub * sub > LIMITS.MAP_WORK do sub = sub - 1 end

    local x0, z0 = x - nx * cell / 2, z - nz * cell / 2

    -- The tile the last chunk came out of, held across the walk. The grid is read in raster
    -- order, so consecutive chunks are nearly always in the same 512 m tile: keeping it here
    -- turns the inner loop into arithmetic and one string.byte, instead of a key concat and
    -- a table lookup per chunk. It is the difference between a map query costing 30 ms and
    -- costing a third of a second.
    local lrx, lrz, lt = nil, nil, nil
    local T, H, R = GEO.TILE, GEO.HEADER, GEO.RECSZ

    -- Packed is opt-in so an older client keeps getting the four-byte records it expects.
    local pack = msg.pack and true or false
    local out, obs = {}, {}
    for j = 0, nz - 1 do
        for i = 0, nx - 1 do
            local terr, otop, obas, mass = -1, -1, 256, 0
            local multi, trees, spike, any = false, false, false, false
            local seen, wet = 0, 0

            for sj = 0, sub - 1 do
                local wz = z0 + (j + (sj + 0.5) / sub) * cell
                local cz = math.floor(wz / 16)
                for si = 0, sub - 1 do
                    local wx = x0 + (i + (si + 0.5) / sub) * cell
                    local cx = math.floor(wx / 16)
                    local rx, rz = math.floor(cx / T), math.floor(cz / T)
                    if rx ~= lrx or rz ~= lrz then lrx, lrz, lt = rx, rz, DB.tile(rx, rz) end
                    if lt then
                        local o = H + ((cz - rz * T) * T + (cx - rx * T)) * R + 1
                        local a, b, c, f = lt:byte(o, o + 3)
                        if f and not hasFlag(f, FLAG.NODATA) then
                            any = true
                            seen = seen + 1
                            if a ~= 255 and a > terr then terr = a end
                            if b ~= 255 then
                                if b > otop then otop = b end
                                if c < obas then obas = c end
                                local m = f % (FLAG.MASS + 1)
                                if m > mass then mass = m end
                                if hasFlag(f, FLAG.MULTI) then multi = true end
                            end
                            if hasFlag(f, FLAG.WATER) then wet = wet + 1 end
                            if hasFlag(f, FLAG.TREES) then trees = true end
                            if hasFlag(f, FLAG.SPIKE) then spike = true end
                        end
                    end
                end
            end

            -- WATER only when EVERY charted sample in the cell is water, where the rest of
            -- this reduction takes the worst sample. That asymmetry is deliberate and it is
            -- load-bearing: the HSI paints water as water at any altitude, suppressing the
            -- hazard ramp on it, so a cell that is nine parts cliff and one part lake must
            -- NOT come back flagged water -- that would hide the cliff behind a calm blue
            -- square. "Some of it is wet" is not a fact worth having; "all of it is wet" is.
            -- Mixed ground therefore falls back to the terrain ramp, which is the safe way
            -- round: a coastline reads as land one cell early, never as sea one cell late.
            local water = (seen > 0 and wet == seen)

            if pack then
                -- ONE BYTE for the ground, and the obstacles sent separately. A level is
                -- 0..47, so six bits carry it with room to spare: bit 6 is water, 63 is
                -- "charted but no ground under it" (a void), and 255 is "never surveyed".
                -- That is everything the heightmap needs, at a quarter of the width -- which
                -- is what pays for a grid fine enough to be worth drawing at subpixel
                -- resolution. Obstacles do not go in the dense array because they are on
                -- 1-2% of cells; a sparse list costs a twentieth of a byte per cell instead
                -- of three.
                if not any then
                    out[#out + 1] = "\255"
                else
                    local v = (terr < 0) and 63 or terr
                    if water then v = v + 64 end
                    out[#out + 1] = string.char(v)
                    if otop >= 0 then
                        local idx = j * nx + i
                        obs[#obs + 1] = string.char(idx % 256, math.floor(idx / 256),
                                                    otop, obas > 255 and 255 or obas, mass)
                    end
                end
            elseif not any then
                out[#out + 1] = string.char(255, 255, 255, FLAG.NODATA)
            else
                local f = mass
                if water then f = f + FLAG.WATER end
                if multi then f = f + FLAG.MULTI end
                if trees then f = f + FLAG.TREES end
                if spike then f = f + FLAG.SPIKE end
                out[#out + 1] = string.char(terr < 0 and 255 or terr,
                                            otop < 0 and 255 or otop,
                                            obas > 255 and 255 or obas, f)
            end
        end
    end
    Q.cells = Q.cells + nx * nz
    return { t = "map", srv = true, id = msg.id, x0 = x0, z0 = z0, cell = cell,
             nx = nx, nz = nz, sub = sub, pack = pack or nil,
             grid = table.concat(out), obs = pack and table.concat(obs) or nil }
end

-- =====================================================================================
-- ACTIVITY -- where the fleet is, for the incremental rebuild.
-- =====================================================================================

local ACT = { seen = {}, lastWrite = 0, writes = 0, fault = nil }

function ACT.note(who, x, y, z)
    x, y, z = tonumber(x), tonumber(y), tonumber(z)
    if not x or not z or not who then return end
    ACT.seen[who] = { x = x, y = y or 0, z = z, t = os.clock(), utc = os.epoch("utc") }
end

function ACT.roster()
    local now, list = os.clock(), {}
    for who, e in pairs(ACT.seen) do
        if now - e.t <= ACTC.KEEP then list[#list + 1] = { who = who, e = e }
        else ACT.seen[who] = nil end
    end
    table.sort(list, function(p, q) return p.e.t > q.e.t end)
    while #list > ACTC.MAX do table.remove(list) end
    return list
end

function ACT.write()
    local list = ACT.roster()
    local out = { "; aer terrain activity -- written by computer #68, read by terrainbuild.jar",
                  "; callsign  x  z  y  seen_utc_ms" }
    for _, r in ipairs(list) do
        out[#out + 1] = string.format("%s %.1f %.1f %.1f %d",
            r.who, r.e.x, r.e.z, r.e.y, r.e.utc)
    end
    local h = fs.open(ACTC.FILE, "w")
    if not h then ACT.fault = "cannot write " .. ACTC.FILE return end
    h.write(table.concat(out, "\n") .. "\n")
    h.close()
    ACT.fault = nil
    ACT.writes = ACT.writes + 1
    ACT.lastWrite = os.clock()
end

function ACT.loop()
    while true do
        pcall(ACT.write)
        sleep(ACTC.WRITE_EVERY)
    end
end

-- =====================================================================================
-- NETWORK
-- =====================================================================================

local NET = { clients = {}, served = 0, refused = 0 }

function NET.beaconMsg()
    -- srv=true marks this as coming FROM the server. Every message on this protocol is
    -- seen by every listener, so without it a client would mistake another aircraft's
    -- request for an answer and latch that aircraft as its server.
    return { t = "beacon", srv = true, id = CFG.STATION.ID, name = CFG.STATION.NAME,
             rev = DB.meta.built, tiles = DB.meta.tiles,
             geo = { minY = GEO.MIN_Y, cell = GEO.CELL, levels = GEO.LEVELS,
                     tile = GEO.TILE, header = GEO.HEADER, record = GEO.RECSZ } }
end

function NET.seen(sender, msg)
    -- tostring, not a bare field: a client is free to send whatever it likes as a
    -- callsign, and the status page calls :sub on this.
    local who = (msg and msg.callsign ~= nil) and tostring(msg.callsign) or ("#" .. tostring(sender))
    NET.clients[sender] = { who = who, t = os.clock(),
                            n = (NET.clients[sender] and NET.clients[sender].n or 0) + 1 }
    return who
end

function NET.handle(sender, msg)
    if type(msg) ~= "table" or type(msg.t) ~= "string" then return end
    if msg.srv then return end          -- another server's beacon, not a request
    local who = NET.seen(sender, msg)

    if msg.t == "hello" then
        rednet.send(sender, NET.beaconMsg(), LINKS.TERRAIN)
        ACT.note(who, msg.x, msg.y, msg.z)

    elseif msg.t == "corr" then
        Q.corr = Q.corr + 1
        ACT.note(who, msg.x, msg.alt, msg.z)
        local ok, reply = pcall(Q.corridor, msg)
        if ok and reply then rednet.send(sender, reply, LINKS.TERRAIN) NET.served = NET.served + 1
        else NET.refused = NET.refused + 1 end

    elseif msg.t == "map" then
        Q.maps = Q.maps + 1
        ACT.note(who, msg.x, msg.alt, msg.z)
        local ok, reply = pcall(Q.map, msg)
        if ok and reply then rednet.send(sender, reply, LINKS.TERRAIN) NET.served = NET.served + 1
        else NET.refused = NET.refused + 1 end

    elseif msg.t == "gnd" then
        Q.gnd = Q.gnd + 1
        local x, z = tonumber(msg.x), tonumber(msg.z)
        if x and z then
            local a, b, c, f = DB.rec(math.floor(x / 16), math.floor(z / 16))
            rednet.send(sender, { t = "gnd", srv = true, x = x, z = z,
                rec = a and string.char(a, b, c, f) or nil }, LINKS.TERRAIN)
            NET.served = NET.served + 1
        end

    elseif msg.t == "tile" then
        Q.tile = Q.tile + 1
        local rx, rz = tonumber(msg.rx), tonumber(msg.rz)
        if rx and rz then
            local d = DB.tile(math.floor(rx), math.floor(rz))
            rednet.send(sender, { t = "tile", srv = true, rx = rx, rz = rz,
                data = d or nil }, LINKS.TERRAIN)
            NET.served = NET.served + 1
        end
    end
end

function NET.loop()
    while true do
        local sender, msg, proto = rednet.receive(nil, 1)
        if sender then
            if proto == LINKS.TERRAIN then
                pcall(NET.handle, sender, msg)
            elseif proto == LINKS.ATC_CAST and type(msg) == "table" then
                -- Receive-only, exactly as at the radar station: this is a position
                -- report from a responder and nothing here ever answers it.
                ACT.note(msg.callsign or ("#" .. tostring(sender)), msg.x, msg.y, msg.z)
            end
        end
    end
end

function NET.beaconLoop()
    while true do
        rednet.broadcast(NET.beaconMsg(), LINKS.TERRAIN)
        sleep(CFG.BEACON.INTERVAL)
    end
end

-- =====================================================================================
-- STATUS PAGE
-- =====================================================================================

local TERM = {}

function TERM.page(t)
    local w, h = t.getSize()
    t.setBackgroundColour(colours.black)
    t.clear()
    local function line(row, s, c)
        if row > h then return end
        t.setCursorPos(1, row)
        t.setTextColour(c or colours.white)
        t.write(s:sub(1, w))
    end

    line(1, CFG.STATION.NAME .. " " .. CFG.STATION.ID .. "  TERRAIN DATA SERVER", colours.cyan)
    line(2, string.rep("-", w), colours.grey)
    line(3, string.format("db   %s", DB.dir or "NONE"), DB.dir and colours.lime or colours.red)
    line(4, string.format("     %s tiles, built %s", tostring(DB.meta.tiles or "?"),
        DB.meta.built and tostring(DB.meta.built) or "?"), colours.lightGrey)
    if DB.fault then
        line(6, "     " .. DB.fault:sub(1, w - 6), colours.red)
    elseif (DBC.RELOAD or 0) > 0 then
        line(6, string.format("     reload every %ds -- %d done, last %ds ago",
            DBC.RELOAD, DB.reloads,
            DB.reloads > 0 and math.floor(os.clock() - DB.lastReload) or 0), colours.grey)
    else
        line(6, "     reload OFF -- a rebuilt survey needs a reboot", colours.grey)
    end
    line(5, string.format("geo  Y %d..%d  step %d m  chunk %d m",
        GEO.MIN_Y, GEO.MIN_Y + GEO.CELL * GEO.LEVELS, GEO.CELL, 16), colours.lightGrey)

    line(7, string.format("cache %d/%d  hit %d  miss %d  read %d",
        DB.held, CFG.CACHE.TILES, DB.hits, DB.misses, DB.reads), colours.white)
    line(8, string.format("query corr %d  map %d  gnd %d  tile %d",
        Q.corr, Q.maps, Q.gnd, Q.tile), colours.white)
    line(9, string.format("      %d samples, %d cells, %d served, %d refused",
        Q.samples, Q.cells, NET.served, NET.refused), colours.lightGrey)

    local row = 11
    line(row, "CLIENTS", colours.yellow) row = row + 1
    local now, anyC = os.clock(), false
    for id, c in pairs(NET.clients) do
        if row < h - 1 then
            local age = now - c.t
            anyC = true
            line(row, string.format(" %-10s #%-4d %4d req  %3ds",
                c.who:sub(1, 10), id, c.n, math.floor(age)),
                age <= CFG.CLIENT_STALE and colours.lime or colours.grey)
            row = row + 1
        end
    end
    if not anyC then line(row, " (none)", colours.grey) row = row + 1 end

    row = row + 1
    line(row, "ACTIVITY -> " .. ACTC.FILE .. (ACT.fault and ("  " .. ACT.fault) or ""),
        ACT.fault and colours.red or colours.yellow)
    row = row + 1
    local list = ACT.roster()
    if #list == 0 then line(row, " (no aircraft heard)", colours.grey) end
    for _, r in ipairs(list) do
        if row < h then
            line(row, string.format(" %-10s %7.0f %7.0f  %3ds",
                r.who:sub(1, 10), r.e.x, r.e.z, math.floor(now - r.e.t)), colours.white)
            row = row + 1
        end
    end
end

function TERM.loop()
    while true do
        pcall(TERM.page, term.current())
        if HW.mon then pcall(TERM.page, HW.mon) end
        sleep(CFG.TERM.REDRAW)
    end
end

-- =====================================================================================

term.clear()
term.setCursorPos(1, 1)
print(CFG.STATION.NAME .. " terrain data server starting...")

DB.dir = DB.locate()
if not DB.dir then
    error("No terrain database found.\n"
        .. "Looked for <mount>/" .. DBC.DIR .. "/" .. DBC.META .. " on every disk drive.\n"
        .. "Put the disk in the drive, and build it with:\n"
        .. "  java -jar terrainbuild.jar build", 0)
end
local ok, err = DB.loadMeta()
if not ok then error("Terrain database at " .. DB.dir .. ": " .. tostring(err), 0) end

print("  database: " .. DB.dir)
print("  tiles   : " .. tostring(DB.meta.tiles) .. "   built " .. tostring(DB.meta.built))
print("  geometry: Y " .. GEO.MIN_Y .. ".." .. (GEO.MIN_Y + GEO.CELL * GEO.LEVELS)
    .. ", " .. GEO.CELL .. " m levels, " .. (16 * 1) .. " m chunks")
print("  modem   : " .. peripheral.getName(HW.modem) .. (HW.wireless and " (wireless)" or " (WIRED)"))
if not HW.wireless then
    print("  ** a wired modem cannot hear an aeroplane. An ENDER modem is wanted. **")
end
print("  serving : " .. LINKS.TERRAIN)
sleep(1)

parallel.waitForAny(NET.loop, NET.beaconLoop, ACT.loop, TERM.loop, DB.loop)
