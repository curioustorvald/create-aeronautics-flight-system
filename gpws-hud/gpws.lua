-- HEAD-UP DISPLAY + GROUND PROXIMITY WARNING SYSTEM
--
-- ONE PROGRAM, EVERY AIRFRAME. Nothing in this file names an aeroplane: the ship, its
-- protocols and its calibration all come from gpwsconf.lua beside it, which is the copy
-- that differs per aircraft. Deployed to #67 (Aer Togekiss), #70 (Mantine) and #71 (Scout);
-- keep them byte-identical, and put anything airframe-shaped in the conf where it belongs.
--
-- Two instruments in one computer, because they are one instrument: the HUD is what the
-- pilot reads and the GPWS is what interrupts them when reading is no longer enough.
--
-- IT CONTROLS NOTHING. Every rednet link is receive-only and every number on the screen is
-- either read off sublevel or heard on a channel the flight system was already broadcasting.
-- Nothing here can move a thruster. A warning system that can fly the aeroplane is a second
-- autopilot nobody asked for -- and one that fails silent is better than one that fails
-- flying. (The single transmission is a nav-data acknowledgement to #24, a roster ping.)
--
-- THE HUD is mounted ABOVE THE PILOT'S EYE LINE, so its layout is inverted with respect to a
-- panel display: the LOWEST rows sit nearest the natural line of sight and carry the most
-- important symbology. Reading UP from the bottom edge --
--
--     alert banner -> speed/height/sink -> heading index + scale -> autopilot annunciators
--     -> attitude field -> route   (self-status and the armed/inhibited line are on the
--     computer's own screen, not here -- see HUD.layout)
--
-- Rows are dropped from the top as the monitor gets shorter; the banner and the primary
-- strip are never dropped.
--
-- THE GPWS HAS TERRAIN DATA, and is careful about what that is worth. There is still no
-- radio altimeter on this airframe and no scanner: what there is, is a SURVEY -- computer
-- #68 on the ground serves a database built out of the world's own region files, one record
-- per chunk and one level per 8 m of Y. So:
--
--   * every terrain-derived mode (2, the true form of 4, and the forward-looking TERRAIN
--     AHEAD / OBSTACLE AHEAD calls) is ARMED, behind TERRAIN.available(). When the link to
--     #68 goes quiet they all fall silent again within CFG.TERRAIN.STALE seconds and this
--     computer behaves exactly as it did before the database existed. Modes 5 (glideslope)
--     and 7 (windshear) remain off: neither needs terrain, both need instruments this
--     airframe does not carry.
--
--   * having terrain is not the same as having something to say about it. Mode 2 needs the
--     gap to be SHUTTING, not merely small -- level flight over flat ground closes on
--     nothing however low it is. The forward-looking calls need the aircraft to be actually
--     going somewhere (FLTA.MIN_GS), because "ahead" is meaningless in a hover. Both stand
--     down for a landing the FMS is flying at a rate it would actually command, since
--     arriving at the ground is the point of one. And DECISION HEIGHT is an approach
--     callout, so it is gated on being in the terminal area at all -- without that, a
--     datum that now exists everywhere makes it fire on every descent in the world.
--
--   * the height it warns on ("RA") is a GEOMETRIC height above a DATUM it can justify --
--     the published elevation of a nearby landing.db field, the surveyed ground beneath us,
--     or the elevation the wheels last left -- and the HUD ALWAYS NAMES THE DATUM. With
--     none of them available it reads dashes and the height-dependent modes inhibit
--     themselves. See the DATUM block.
--
-- Surveyed ground is charted HIGH on purpose -- the top of an 8 m cell, and the highest
-- ground in a 16 m chunk. Clearance therefore reads low and calls come early. That bias is
-- the whole reason the number is safe to warn on, and it is also why SITE still outranks
-- TERR near a field, where the exact elevation of the pad is known and matters.
--
-- Everything tunable is in gpwsconf.lua beside this file. See its header.

-- ============================================================================
-- CONFIGURATION
-- ============================================================================
-- Scoped inside the do-block so the load temporaries do not sit in top-level registers,
-- the same discipline autopilot.lua, uac.lua and atc.lua keep (see apconf.lua's note on
-- Cobalt's 200-local ceiling).
local CFG
local SHIP, LINKS, STALE, GP, VOICE, HC
local POINTS_FILE, COURSE_FILE, LANDING_FILE
do
    local ok, res = pcall(require, "gpwsconf")
    if not ok then
        error("Cannot load gpwsconf.lua (the HUD/GPWS configuration): " .. tostring(res)
            .. "\nIt must sit beside gpws.lua and end with 'return CFG'.", 0)
    end
    if type(res) ~= "table" then
        error("gpwsconf.lua did not return the CFG table (got " .. type(res) .. ").", 0)
    end
    -- Canary check: catch a truncated or half-saved config at BOOT, naming what is missing,
    -- rather than as a nil-arithmetic crash on short final.
    -- SHIP.ALT_OFFSET is checked one level down, with the rest of SHIP: a missing one is a
    -- nil-arithmetic crash the first time a fix is read, and a DEFAULTED one would be worse
    -- -- 0 is a legal value, so a silent default puts the radio altitude back to over-reading
    -- by the height of the hull, which on this computer means every terrain call fires late.
    for _, k in ipairs({ "SHIP", "LINKS", "STALE", "POINTS_FILE", "COURSE_FILE",
            "LANDING_FILE", "NAVDATA", "TERRAIN", "DATUM", "GPWS", "VOICE", "ALERTS",
            "HUD", "SELFTEST", "DEBUG" }) do
        if res[k] == nil then
            error("gpwsconf.lua is missing CFG." .. k .. " -- the file looks incomplete.", 0)
        end
    end
    for t, ks in pairs({
            SHIP    = { "NAME", "CALLSIGN", "HEADING_SIGN", "HEADING_OFFSET", "ALT_OFFSET" },
            LINKS   = { "APSWITCH_NAV", "APSWITCH_ALT", "ALTAP", "AUTOLAND", "APNAV",
                        "ENGINE", "TOUCHDOWN", "MANCON", "NAVDATA", "TERRAIN" },
            STALE   = { "FIX", "APSWITCH", "ALTAP", "AUTOLAND", "APNAV", "ENGINE",
                        "TOUCHDOWN", "MANCON" },
            NAVDATA = { "ENABLE", "PULL_EVERY", "STALE_AFTER", "TMP" },
            TERRAIN = { "ENABLE", "LOOK_AHEAD", "CLEAR_M", "MIN_T", "DATUM",
                        "STALE", "PROF_STALE", "REQ_EVERY", "STEP", "WIDTH", "MIN_RANGE",
                        "RANGE_MARGIN", "MAX_SAMPLES", "MIN_GS", "TRACK_TOL", "BEHIND",
                        "MIN_MASS", "OBST_CLEAR", "VS_HORIZON", "LEG_MARGIN", "TILE_CACHE", "TILE_EVERY",
                        "FLAG" },
            DATUM   = { "PAD_HALF", "RWY_HALF_W", "RWY_OVERRUN", "HELI_RADIUS",
                        "RWY_RADIUS", "DEP_RADIUS", "SQUAT_TRUST" },
            GPWS    = { "INHIBIT", "MIN_SHOW", "MODE1", "MODE2", "MODE3", "MODE4", "MODE6",
                        "MODE7", "APDISC", "FLTA", "VS_FILTER" },
            VOICE   = { "ENABLE", "LANG", "MARKER", "VOLUME", "CHUNK", "RATE", "MIN_DWELL",
                        "CLIPS" },
            HUD     = { "TEXT_SCALE", "REDRAW", "TERM_REDRAW", "FOV_DEG", "LADDER_STEP",
                        "HDG_PER_COL", "CELL_ASPECT", "PITCH_SIGN", "ROLL_SIGN", "PALETTE" },
            SELFTEST = { "ON_TOUCH", "ON_KEY", "GAP" },
            DEBUG    = { "ENABLE", "WHEN_QUIET", "ON_KEY" },
        }) do
        if type(res[t]) ~= "table" then
            error("gpwsconf.lua's CFG." .. t .. " is not a table.", 0)
        end
        for _, k in ipairs(ks) do
            if res[t][k] == nil then
                error("gpwsconf.lua's CFG." .. t .. " is missing " .. k .. ".", 0)
            end
        end
    end
    if type(res.ALERTS) ~= "table" or #res.ALERTS == 0 then
        error("gpwsconf.lua's CFG.ALERTS is empty -- there would be nothing to annunciate.", 0)
    end
    CFG = res
    SHIP, LINKS, STALE = CFG.SHIP, CFG.LINKS, CFG.STALE
    GP, VOICE, HC = CFG.GPWS, CFG.VOICE, CFG.HUD
    POINTS_FILE, COURSE_FILE, LANDING_FILE =
        CFG.POINTS_FILE, CFG.COURSE_FILE, CFG.LANDING_FILE
end

if not sublevel then error("No sublevel API - is the Sable mod present?", 0) end

-- ============================================================================
-- FRAME, SIGNS AND SMALL MATHS
-- ============================================================================
-- Same frame, same conventions, same sign choices as autopilot.lua, responder.lua and
-- atc.lua -- deliberately. A heading on this HUD and a heading on the fleet's radar have to
-- be the same number, or the pilot and the controller are not talking about the same
-- aeroplane.

local atan2 = math.atan2
if not atan2 then
    atan2 = function(y, x)
        if x > 0 then return math.atan(y / x)
        elseif x < 0 then
            if y >= 0 then return math.atan(y / x) + math.pi
            else return math.atan(y / x) - math.pi end
        else
            if y > 0 then return math.pi / 2
            elseif y < 0 then return -math.pi / 2
            else return 0 end
        end
    end
end

local function clamp(v, lo, hi) return math.max(lo, math.min(hi, v)) end
-- Cobalt is 5.1-flavoured (global `unpack`); the syntax checker on this workstation is 5.4
-- (`table.unpack`). Bound once so neither VM has to be guessed at further down.
local unpk = table.unpack or unpack
-- Bearings are atan2(dx, dz) in the frame where North = pi; this maps one to a true
-- compass heading (deg, N=0 E=90 S=180 W=270). Identical to the FMS's toCompass.
local function toCompass(rad) return (180 - math.deg(rad)) % 360 end
local function bearingTo(dx, dz) return toCompass(atan2(dx, dz)) end
-- Signed difference between two compass headings, in (-180, 180].
local function wrap180(d) d = (d + 180) % 360 - 180; if d <= -180 then d = d + 360 end return d end
-- Snap a hair either side of zero to zero, so a level display reads "+0" and not "-0".
local function snap0(v) if math.abs(v) < 0.05 then return 0 end return v end
local function dist2d(ax, az, bx, bz)
    local dx, dz = ax - bx, az - bz
    return math.sqrt(dx * dx + dz * dz)
end

-- Rotate vector (x,y,z) by the UNIT quaternion q, PRESERVING its magnitude (q*vector
-- normalises the intermediate product and silently discards it -- see uac.lua's rotateVec
-- and altcom.lua's copy of it; this is the same routine again, for the same reason).
local function rotateVec(q, x, y, z)
    local ux, uy, uz, a = q.v.x, q.v.y, q.v.z, q.a
    local tx = 2 * (uy*z - uz*y)
    local ty = 2 * (uz*x - ux*z)
    local tz = 2 * (ux*y - uy*x)
    return x + a*tx + (uy*tz - uz*ty),
           y + a*ty + (uz*tx - ux*tz),
           z + a*tz + (ux*ty - uy*tx)
end

-- Piecewise-linear lookup over a list of rows sorted by `key`, held flat outside the ends.
-- Both GPWS envelopes are expressed this way so their shape is visible in gpwsconf.lua
-- rather than buried in an equation here.
local function envelope(rows, key, x, ...)
    local out = {}
    local a, b = rows[1], rows[#rows]
    for i = 1, #rows - 1 do
        if x >= rows[i][key] and x <= rows[i + 1][key] then a, b = rows[i], rows[i + 1] break end
    end
    if x <= rows[1][key] then a, b = rows[1], rows[1]
    elseif x >= rows[#rows][key] then a, b = rows[#rows], rows[#rows] end
    local span = b[key] - a[key]
    local f = (span > 1e-9) and clamp((x - a[key]) / span, 0, 1) or 0
    for i = 1, select("#", ...) do
        local k = select(i, ...)
        out[i] = a[k] + (b[k] - a[k]) * f
    end
    -- unpack, not `table.unpack and table.unpack(out) or unpack(out)`: an and/or chain
    -- truncates a multiple-return to its first value, which would have handed Mode 1 the
    -- caution boundary as its warning boundary and never fired a PULL UP.
    return unpk(out)
end

-- ============================================================================
-- PERIPHERALS
-- ============================================================================
-- Everything is optional except the modem: a HUD with no monitor still watches the
-- envelopes and still talks (the speaker is the half that cannot be missed), and a GPWS with
-- no speaker still annunciates. Each missing piece is named on the status page rather than
-- being a boot failure -- the aircraft should not be grounded by an unbolted monitor.

local HW = { faults = {} }
do
    -- Prefer a WIRELESS modem: this aircraft's fleet links are all broadcast, and a wired
    -- modem attached for peripherals would open a network with nothing on it.
    HW.modem = peripheral.find("modem", function(_, m) return m.isWireless and m.isWireless() end)
             or peripheral.find("modem")
    if not HW.modem then error("No modem attached -- nothing to listen to.", 0) end
    rednet.open(peripheral.getName(HW.modem))

    HW.mon = HC.MONITOR_NAME and peripheral.wrap(HC.MONITOR_NAME) or peripheral.find("monitor")
    if HW.mon then
        HW.monName = peripheral.getName(HW.mon)
        pcall(function() HW.mon.setTextScale(HC.TEXT_SCALE) end)
        for name, rgb in pairs(HC.PALETTE) do
            if colours[name] then pcall(HW.mon.setPaletteColour, colours[name], rgb) end
        end
    else
        HW.faults.mon = "no monitor"
    end

    HW.speaker = peripheral.find("speaker")
    if not HW.speaker then HW.faults.speaker = "no speaker" end
end

-- ============================================================================
-- DATA LAYER: points.db / landing.db
-- ============================================================================
-- READ ONLY, and for exactly one purpose: the published elevation of the fields this
-- aircraft flies into and out of. #24 owns these files and the FMS and the radar carry the
-- same two parsers; keeping all three character-for-character in step matters, because a
-- record this computer skips is a field it will not arm the GPWS over.

local DB = { POINTS = {}, LANDING = {}, SITES = {} }

function DB.loadPoints()
    DB.POINTS = {}
    local f = io.open(POINTS_FILE, "r")
    if not f then return end
    for line in f:lines() do
        local t = line:match("^%s*(.-)%s*$")
        if t ~= "" and t:sub(1, 1) ~= ";" then
            local name, xs, zs = t:match("^(%a+)=(%-?%d+),(%-?%d+)$")
            if name then
                DB.POINTS[name] = { name = name, x = tonumber(xs), z = tonumber(zs) }
            end
        end
    end
    f:close()
end

-- Only the SITE shape (NAME=type,dir,alt,len) is read. The goaround/appr shapes in the same
-- file are procedures, not places -- they carry no elevation of their own and the FMS's
-- isLandingSite excludes them for the same reason.
function DB.loadLanding()
    DB.LANDING = {}
    local f = io.open(LANDING_FILE, "r")
    if not f then return end
    for line in f:lines() do
        local t = line:match("^%s*(.-)%s*$")
        if t ~= "" and t:sub(1, 1) ~= ";" then
            local name, ty, dir, alt, rwy =
                t:match("^(%a+)=(%a+),([%a%.%d]+),(%-?%d+),(%-?%d+)$")
            if name and ty:lower():sub(1, 8) ~= "goaround" then
                DB.LANDING[name] = { type = ty:lower(), dir = dir:lower(),
                                     alt = tonumber(alt), rwyLen = tonumber(rwy) }
            end
        end
    end
    f:close()
end

-- The datum candidates: every landing site whose position points.db also knows. Built once
-- per database load rather than per tick -- this list is walked 20 times a second.
-- landing.db's direction as a world unit vector, in autopilot.lua's convention exactly
-- (its DIR_VECTORS): north is Z-, east is X+, and a numeric heading is
-- {sin(theta), -cos(theta)}. 'x' -- don't care -- has no axis, so such a runway is treated
-- as a pad-shaped footprint.
function DB.dirVec(dir)
    if not dir then return nil end
    dir = dir:lower()
    if dir == "n" then return 0, -1 end
    if dir == "e" then return 1, 0 end
    if dir == "s" then return 0, 1 end
    if dir == "w" then return -1, 0 end
    local d = tonumber(dir)
    if not d then return nil end
    local r = math.rad(d)
    return math.sin(r), -math.cos(r)
end

function DB.build()
    DB.SITES = {}
    for name, l in pairs(DB.LANDING) do
        local p = DB.POINTS[name]
        if p and l.alt then
            -- The axis is resolved ONCE here, not per tick: this list is walked 20 times a
            -- second and the footprint test projects onto it every time.
            local vx, vz = DB.dirVec(l.dir)
            DB.SITES[#DB.SITES + 1] = { name = name, x = p.x, z = p.z,
                                        alt = l.alt, type = l.type,
                                        len = l.rwyLen, vx = vx, vz = vz }
        end
    end
end

function DB.loadAll() DB.loadPoints() DB.loadLanding() DB.build() end
DB.loadAll()

-- ============================================================================
-- NAV DATA CLIENT -- an ordinary #24 subscriber, and the simplest one on the fleet: this
-- computer has no screen an update is unsafe on, so a refresh applies the moment it arrives.
-- What it keeps from the FMS's NDU is the part that matters -- stage beside the live file,
-- verify the length on disk, and only then swap -- because opening the live points.db for
-- writing truncates it before a failure is known, and a truncated database here silently
-- disarms the GPWS over every field in it.
-- ============================================================================

local NDU = { rev = nil, stamp = nil, serverId = nil, lastHeard = nil, applied = 0,
              fault = nil, nextPull = 0 }

function NDU.sane(m)
    if type(m.points) ~= "string" or type(m.courses) ~= "string" then return false end
    if type(m.landing) ~= "string" then return false end
    return m.points:find("%a+=%-?%d+,%-?%d+") ~= nil
end

function NDU.stage(path, body)
    local tmp = path .. CFG.NAVDATA.TMP
    pcall(fs.delete, tmp)
    local f = io.open(tmp, "w")
    if not f then return false end
    local ok = pcall(function() f:write(body) end)
    pcall(function() f:close() end)
    if ok and fs.exists(tmp) and fs.getSize(tmp) == #body then return true end
    pcall(fs.delete, tmp)
    return false
end

function NDU.swap(path)
    local tmp = path .. CFG.NAVDATA.TMP
    if not fs.exists(tmp) then return end
    pcall(fs.delete, path)
    pcall(fs.move, tmp, path)
end

function NDU.applyNow(m)
    -- All three staged, then all three swapped -- course.db included, though nothing here
    -- reads it. The file set is kept identical to every other client's so this computer can
    -- be copied onto another airframe without a surprise, and a part-applied refresh cannot
    -- leave the two files that DO matter disagreeing about which fields exist.
    local files = { { POINTS_FILE, m.points }, { COURSE_FILE, m.courses },
                    { LANDING_FILE, m.landing } }
    for _, fl in ipairs(files) do
        if not NDU.stage(fl[1], fl[2]) then
            for _, g in ipairs(files) do pcall(fs.delete, g[1] .. CFG.NAVDATA.TMP) end
            NDU.fault = "cannot write - disk full?"
            return false
        end
    end
    for _, fl in ipairs(files) do NDU.swap(fl[1]) end
    NDU.fault = nil
    DB.loadAll()
    NDU.rev, NDU.stamp = m.rev, m.stamp
    NDU.applied = NDU.applied + 1
    return true
end

-- Answer every broadcast, once, whatever the outcome -- that is what keeps this computer's
-- row on the server's roster alive.
function NDU.report()
    if not NDU.serverId then return end
    rednet.send(NDU.serverId, {
        t = "ack", rev = NDU.rev, stamp = NDU.stamp, fault = NDU.fault,
        who = os.getComputerLabel() or ("computer " .. os.getComputerID()),
    }, LINKS.NAVDATA)
end

function NDU.onSync(sender, msg)
    NDU.serverId, NDU.lastHeard = sender, os.clock()
    -- Stamp, not rev: content equality is the real question, and it survives the server
    -- being rebuilt with its revision counter lost.
    if msg.stamp ~= NDU.stamp and NDU.sane(msg) then pcall(NDU.applyNow, msg) end
    NDU.report()
end

function NDU.status()
    if not CFG.NAVDATA.ENABLE then return "NAV local", colours.lightGrey end
    if NDU.fault then return "NAV " .. NDU.fault, colours.red end
    if not NDU.lastHeard then return "NAV local, no server", colours.lightGrey end
    local age = os.clock() - NDU.lastHeard
    if age > CFG.NAVDATA.STALE_AFTER then
        return string.format("NAV rev %s srv %ds", tostring(NDU.rev or "?"), math.floor(age)),
            colours.orange
    end
    return string.format("NAV rev %s", tostring(NDU.rev or "?")), colours.lime
end

function NDU.pullLoop()
    if not CFG.NAVDATA.ENABLE then while true do sleep(10) end end
    while true do
        -- Only while we have never heard the server. Once synced, its own 30 s broadcast is
        -- the heartbeat and asking again would just be noise on a shared channel.
        if not NDU.lastHeard and os.clock() >= NDU.nextPull then
            rednet.broadcast({ t = "pull" }, LINKS.NAVDATA)
            NDU.nextPull = os.clock() + CFG.NAVDATA.PULL_EVERY
        end
        sleep(1)
    end
end

-- ============================================================================
-- LINKS -- what the rest of the aircraft is saying.
-- ============================================================================
-- Every channel is stored with the time it was last heard, and NOTHING reads a value
-- without a staleness bound. A frozen number on a head-up display is worse than no number,
-- because the pilot cannot tell it has stopped moving; a stale field here comes back nil and
-- is drawn as dashes.

-- Forward declaration. LINK.dispatch and DATUM.update both reach for the terrain service,
-- and both are defined ABOVE it -- without this the name would compile as a global and read
-- nil for ever. Declared here rather than moving the block, because the terrain service in
-- turn needs ST, which is defined between the two.
local TERRAIN

local LINK = { v = {}, t = {} }

function LINK.set(key, value) LINK.v[key] = value LINK.t[key] = os.clock() end
function LINK.age(key) local t = LINK.t[key] if not t then return math.huge end return os.clock() - t end
function LINK.get(key, maxAge)
    if LINK.age(key) > maxAge then return nil end
    return LINK.v[key]
end
function LINK.fresh(key, maxAge) return LINK.age(key) <= maxAge end

function LINK.dispatch(sender, msg, proto)
    if type(msg) ~= "table" then return end
    if proto == LINKS.APSWITCH_NAV then
        if msg.autopilot_nav ~= nil then LINK.set("apnav", msg.autopilot_nav and true or false) end
    elseif proto == LINKS.APSWITCH_ALT then
        if msg.autopilot_altitude ~= nil then LINK.set("apalt", msg.autopilot_altitude and true or false) end
    elseif proto == LINKS.ALTAP then
        LINK.set("altsel", tonumber(msg.alti))
    elseif proto == LINKS.AUTOLAND then
        LINK.set("autoland", msg)
    elseif proto == LINKS.APNAV then
        LINK.set("target", msg)
    elseif proto == LINKS.ENGINE then
        LINK.set("engine", msg)
    elseif proto == LINKS.TOUCHDOWN then
        LINK.set("squat", msg.touchedDown and true or false)
    elseif proto == LINKS.MANCON then
        LINK.set("cmdvs", tonumber(msg.vs))
    elseif proto == LINKS.NAVDATA then
        if msg.t == "sync" then NDU.onSync(sender, msg) end
    elseif proto == LINKS.TERRAIN then
        TERRAIN.onMsg(sender, msg)
    end
end

function LINK.loop()
    while true do
        local sender, msg, proto = rednet.receive(nil, 1)
        if sender then LINK.dispatch(sender, msg, proto) end
    end
end

-- ============================================================================
-- OWN-SHIP STATE -- read straight off sublevel, 20 Hz, exactly as the responder does.
-- ============================================================================

local ST = {
    ok = false, fixT = 0, boot = os.clock(),
    x = 0, y = 0, z = 0, vx = 0, vy = 0, vz = 0,
    gs = 0, vs = 0, hdg = 0, trk = 0,
    pitch = 0, bank = 0, ux = 0, uy = 1, uz = 0, inverted = false,
    ra = nil, datum = nil, datumSrc = nil, datumName = nil,
    siteNear = nil, siteNearD = nil,   -- the field being OPERATED AT, if any. Not a datum.
    onGround = false, airborneSince = nil, groundSince = os.clock(),
    vsf = 0,                 -- the ESTIMATED vertical rate. See ST.filterVS.
    vsEst = { y = nil, v = 0, t = nil },
}

-- VERTICAL RATE, ESTIMATED FROM ALTITUDE.
--
-- Driven by POSITION, not by the reported rate, and that is the whole point. This airframe
-- swings its VS -5 to +5 m/s on a two-second period; that same oscillation displaces it by
-- only +-1.6 m. Smoothing the velocity signal means fighting a +-5 m/s waveform and trading
-- lag against ripple; estimating from altitude means seeing a 1.6 m wobble on a flat line.
--
-- A critically-damped alpha-beta filter: the steady-state Kalman gains for a
-- constant-velocity model, in closed form, parameterised by the single time constant
-- GP.VS_FILTER.TAU. `now` is passed in rather than read here so the estimator can be driven
-- from a simulated clock in test.
function ST.filterVS(y, now)
    local e, F = ST.vsEst, GP.VS_FILTER
    if not e.y or not e.t then e.y, e.v, e.t = y, 0, now return 0 end
    local dt = now - e.t
    if dt <= 0 then return e.v end
    if dt > F.RESET then e.y, e.v, e.t = y, 0, now return 0 end
    e.t = now

    local yp = e.y + e.v * dt          -- where the constant-velocity model said we would be
    local r  = y - yp                  -- and what actually happened
    if math.abs(r) > F.JUMP then       -- that is not drift, that is a different place
        e.y, e.v = y, 0
        return 0
    end
    local q = math.exp(-dt / F.TAU)
    local a = 1 - q * q
    local b = (1 - q) * (1 - q)
    e.y = yp + a * r
    e.v = e.v + (b / dt) * r
    return e.v
end

function ST.resetVS() ST.vsEst.y, ST.vsEst.v, ST.vsEst.t = nil, 0, nil end

local function getYaw(orientation)
    local a, b = orientation:toEuler()
    if type(a) == "number" then return b end
    return a.y
end

function ST.read()
    local ok, pose = pcall(sublevel.getLogicalPose)
    if not ok or not pose then ST.ok = false ST.resetVS() return end
    local okv, v = pcall(sublevel.getLinearVelocity)
    if not okv or not v then ST.ok = false ST.resetVS() return end

    -- ALTITUDE DATUM (2026-08-29). The pose sits SHIP.ALT_OFFSET metres above the bottom of
    -- the hull; ST.y is the Y of that bottom. It has to be, because ST.datum is the world Y
    -- of a piece of GROUND every time -- landing.db's surveyed pad/runway elevation on a SITE
    -- datum, the terrain server's profile on a TERR one -- so ST.ra was over-reading by the
    -- height of the aeroplane on every approach, and an over-reading clearance is a GPWS that
    -- calls late. (The DEP datum was always self-consistent: it latches this same corrected
    -- Y, so the difference was right even when the number was not.)
    ST.x, ST.y, ST.z = pose.position.x, pose.position.y - SHIP.ALT_OFFSET, pose.position.z
    ST.vx, ST.vy, ST.vz = v.x, v.y, v.z
    ST.gs = math.sqrt(v.x * v.x + v.z * v.z)
    ST.vs = v.y
    ST.hdg = toCompass(SHIP.HEADING_SIGN * getYaw(pose.orientation) + SHIP.HEADING_OFFSET)
    ST.trk = (ST.gs > 0.5) and bearingTo(v.x, v.z) or ST.hdg

    -- World "up" expressed in the body frame. This one vector encodes pitch AND roll with no
    -- +-90 deg clamp and no discontinuity when inverted, which Euler pitch cannot do -- it is
    -- the vector projection, not an angle, that generalises through any attitude. Same
    -- routine, same two sign knobs and the same flown values as #7's attitude indicator:
    -- both displays read the same quaternion and must agree.
    local ux, uy, uz = rotateVec(pose.orientation:inverse(), 0, 1, 0)
    ST.ux, ST.uy, ST.uz = ux, uy, uz
    ST.pitch = math.deg(math.asin(clamp(HC.PITCH_SIGN * uz, -1, 1)))
    ST.bank  = math.deg(HC.ROLL_SIGN * atan2(ux, uy))
    ST.inverted = (uy < 0)

    ST.fixT = os.clock()
    ST.vsf = ST.filterVS(ST.y, ST.fixT)
    ST.ok = true
end

function ST.fixFresh() return ST.ok and (os.clock() - ST.fixT) <= STALE.FIX end

-- ============================================================================
-- HEIGHT DATUM -- what "RA" on this HUD actually means.
-- ============================================================================
-- There is no radio altimeter on this airframe. The height every GPWS mode is driven by is
-- therefore a GEOMETRIC height above a datum this computer can justify, and the HUD always
-- names which datum that is, so the pilot knows what the number is worth:
--
--   SITE -- the published elevation of a nearby landing.db field. Surveyed data about the
--           exact patch of ground being flown at, live on approach and departure, which is
--           where GPWS earns its keep. The runway radius matches the FMS's own approach arm
--           distance so the two arm together. It outranks TERR because it describes the
--           actual pad, not the highest block in a 16 m square containing it.
--   TERR -- the terrain database's ground directly beneath, from #68. Good to within the
--           8 m cell plus whatever relief a chunk hides, and biased HIGH, so RA derived
--           from it reads LOW and every envelope trips early. Available wherever the world
--           has been surveyed, which is most places the fleet flies -- this is what put an
--           RA on the HUD en route. Switched off with CFG.TERRAIN.DATUM.
--   DEP  -- the elevation the wheels last left, latched from the squat switch or the FMS's
--           own touchdown call. Trustworthy near where it was taken and nowhere else, so it
--           expires with DISTANCE, not with time. Below TERR because it describes ground
--           that may be a kilometre behind, where TERR describes the ground underneath.
--   nil  -- over ground the survey never covered, with no field and no latch. RA reads
--           dashes and every height-dependent mode inhibits itself. Still the honest
--           answer, and still what happens off the edge of the map.
--
-- Note what is NOT here: no "settled and still" test that would latch a datum from a steady
-- hover. A VTOL holding station at 200 m looks identical to one sitting on a pad, and a
-- datum invented there would put the ground 200 m too high in every envelope below.

local DATUM = { dep = nil }

function DATUM.groundTruth()
    if LINK.get("squat", CFG.DATUM.SQUAT_TRUST) == true then return true end
    local al = LINK.get("autoland", STALE.AUTOLAND)
    if al and al.cmd == "landed" then return true end
    return false
end

-- Is (x,z) ON this site -- over the pad square, or over the runway strip? This is the ONLY
-- place a published elevation is allowed to be the datum, because it is the only place that
-- elevation describes the ground.
function DATUM.onSite(s, x, z)
    local D = CFG.DATUM
    local dx, dz = x - s.x, z - s.z
    if s.type == "rwy" and s.vx and (s.len or 0) > 0 then
        -- Project onto the runway axis. The point is the THRESHOLD and the strip runs from
        -- there to threshold + length, in the landing direction.
        local along = dx * s.vx + dz * s.vz
        local lat   = -dx * s.vz + dz * s.vx
        return along >= -D.RWY_OVERRUN and along <= s.len + D.RWY_OVERRUN
           and math.abs(lat) <= D.RWY_HALF_W
    end
    return math.abs(dx) <= D.PAD_HALF and math.abs(dz) <= D.PAD_HALF
end

function DATUM.update()
    if DATUM.groundTruth() and ST.ok then
        DATUM.dep = { y = ST.y, x = ST.x, z = ST.z, t = os.clock() }
    end
    if not ST.ok then
        ST.ra, ST.datum, ST.datumSrc, ST.datumName = nil, nil, nil, nil
        ST.datumX, ST.datumZ = nil, nil
        ST.siteNear, ST.siteNearD = nil, nil
        return
    end

    -- TWO SEPARATE QUESTIONS, and conflating them is what put the datum 59 m out for four
    -- kilometres on every arrival:
    --   onSite  -- is this field's published elevation the ground HERE?  (the footprint)
    --   near    -- are we OPERATING at this field?                       (the arm radius)
    -- The first picks the datum. The second arms the approach logic and the decision-height
    -- callout, and makes no claim about height at all.
    local best, bestD, near, nearD
    for i = 1, #DB.SITES do
        local s = DB.SITES[i]
        local d = dist2d(ST.x, ST.z, s.x, s.z)
        if DATUM.onSite(s, ST.x, ST.z) and (not bestD or d < bestD) then best, bestD = s, d end
        local r = (s.type == "rwy") and CFG.DATUM.RWY_RADIUS or CFG.DATUM.HELI_RADIUS
        if d <= r and (not nearD or d < nearD) then near, nearD = s, d end
    end
    ST.siteNear, ST.siteNearD = near, nearD

    local terr = (CFG.TERRAIN.DATUM and TERRAIN.available())
        and TERRAIN.groundAt(ST.x, ST.z) or nil

    if best then
        ST.datum, ST.datumSrc, ST.datumName = best.alt, "SITE", best.name
        ST.datumX, ST.datumZ = best.x, best.z
    elseif terr then
        ST.datum, ST.datumSrc, ST.datumName = terr, "TERR", nil
        ST.datumX, ST.datumZ = ST.x, ST.z
    elseif DATUM.dep and dist2d(ST.x, ST.z, DATUM.dep.x, DATUM.dep.z) <= CFG.DATUM.DEP_RADIUS then
        ST.datum, ST.datumSrc, ST.datumName = DATUM.dep.y, "DEP", nil
        ST.datumX, ST.datumZ = DATUM.dep.x, DATUM.dep.z
    else
        ST.datum, ST.datumSrc, ST.datumName = nil, nil, nil
        ST.datumX, ST.datumZ = nil, nil
    end
    ST.ra = ST.datum and (ST.y - ST.datum) or nil
end

-- ============================================================================
-- TERRAIN SERVICE
-- ============================================================================
-- The three functions the rest of this program asks about the ground, and the client that
-- feeds them. Their contract is unchanged from the stub they replace:
--
--   groundAt(x, z)            -> world Y of the surface under (x,z), or nil
--   aheadClearance(secs)      -> least clearance (m) along the projected ground track over
--                                the next `secs`, or nil
--   obstacleAhead(secs)       -> true if a charted obstacle (not terrain) is on the track,
--                                false if the volume is known clear, nil if unknown
--
-- NIL STILL MEANS SILENCE. Every one of them returns nil the moment the answer would be a
-- guess -- no server, no geometry, a profile older than PROF_STALE, a turn off the profile's
-- track, or a chunk the survey never covered. The modes above check available() and give up,
-- exactly as they did when there was no data at all. That is not a fallback, it is the
-- design: a terrain warning derived from a stale profile teaches the pilot to ignore the
-- real one.
--
-- TWO SHAPES OF ANSWER, because the questions are different shapes:
--
--   the CORRIDOR  -- one request every REQ_EVERY seconds asks #68 for the worst ground in a
--                    swathe along the ground track, reduced server-side and returned as four
--                    bytes per 16 m sample. The 20 Hz core tick then re-reads that cached
--                    profile against LIVE altitude, so alerting runs at tick rate while the
--                    radio runs at half a hertz.
--   the TILE      -- groundAt() needs an arbitrary point, not a point on the track, so it is
--                    served from a small cache of whole 32x32-chunk tiles. This is what puts
--                    an RA on the HUD en route.
--
-- The profile is anchored where the aircraft WAS when it was cut. Every read projects the
-- current position back onto it (profIndex) rather than assuming sample 0 is underfoot -- at
-- 320 m/s a half-second-old profile is already 160 m behind.
-- ============================================================================

TERRAIN = {
    F = CFG.TERRAIN.FLAG,
    srv = nil, geo = nil, rev = nil, heard = nil, fault = nil,
    prof = nil, reqId = 0,
    tiles = {}, tileHeld = 0, tileSeq = 0, tileAt = 0,
    asked = 0, served = 0,
}

-- Flag bits by arithmetic, so this does not depend on which bit library the VM ships.
function TERRAIN.bit(f, b) return math.floor(f / b) % 2 == 1 end

-- The Y a level means. TOP of the cell for anything solid, BASE for the underside of a
-- floating mass: both round in the direction that makes the gap look smaller than it is.
function TERRAIN.levelTop(L) return TERRAIN.geo.minY + TERRAIN.geo.cell * L + (TERRAIN.geo.cell - 1) end
function TERRAIN.levelBase(L) return TERRAIN.geo.minY + TERRAIN.geo.cell * L end

function TERRAIN.available()
    return CFG.TERRAIN.ENABLE == true and TERRAIN.geo ~= nil and TERRAIN.heard ~= nil
       and (os.clock() - TERRAIN.heard) <= CFG.TERRAIN.STALE
end

-- One four-byte record -> ground Y, obstacle top Y, obstacle base Y, flag byte.
-- nil when that chunk was never surveyed.
function TERRAIN.decode(s, o)
    local terr, otop, obas, f = s:byte(o, o + 3)
    if not f or TERRAIN.bit(f, TERRAIN.F.NODATA) then return nil end
    local gy = (terr ~= 255) and TERRAIN.levelTop(terr) or nil
    local oy, ob
    if otop ~= 255 and obas ~= 255 then
        oy, ob = TERRAIN.levelTop(otop), TERRAIN.levelBase(obas)
    end
    return gy, oy, ob, f
end

-- ---- tiles, for groundAt --------------------------------------------------------------

function TERRAIN.putTile(rx, rz, d)
    if not rx or not rz then return end
    local k = math.floor(rx) .. ":" .. math.floor(rz)
    if TERRAIN.tiles[k] == nil then TERRAIN.tileHeld = TERRAIN.tileHeld + 1 end
    TERRAIN.tileSeq = TERRAIN.tileSeq + 1
    TERRAIN.tiles[k] = { d = d, u = TERRAIN.tileSeq }
    while TERRAIN.tileHeld > CFG.TERRAIN.TILE_CACHE do
        local evK, evU
        for kk, e in pairs(TERRAIN.tiles) do
            if not evU or e.u < evU then evK, evU = kk, e.u end
        end
        if not evK then break end
        TERRAIN.tiles[evK] = nil
        TERRAIN.tileHeld = TERRAIN.tileHeld - 1
    end
end

function TERRAIN.wantTile(rx, rz)
    if not TERRAIN.srv then return end
    local now = os.clock()
    if now - TERRAIN.tileAt < CFG.TERRAIN.TILE_EVERY then return end
    TERRAIN.tileAt = now
    TERRAIN.asked = TERRAIN.asked + 1
    rednet.send(TERRAIN.srv, { t = "tile", rx = rx, rz = rz, callsign = SHIP.CALLSIGN },
        LINKS.TERRAIN)
end

-- Ground under an arbitrary point. Returns the surface Y, and -- for callers that want them
-- -- the obstacle top, obstacle base and flag byte of the same chunk.
function TERRAIN.groundAt(x, z)
    if not TERRAIN.available() then return nil end
    local g = TERRAIN.geo
    local cx, cz = math.floor(x / 16), math.floor(z / 16)
    local rx, rz = math.floor(cx / g.tile), math.floor(cz / g.tile)
    local e = TERRAIN.tiles[rx .. ":" .. rz]
    if e == nil then TERRAIN.wantTile(rx, rz) return nil end
    if e.d == false then return nil end
    TERRAIN.tileSeq = TERRAIN.tileSeq + 1
    e.u = TERRAIN.tileSeq
    local lx, lz = cx - rx * g.tile, cz - rz * g.tile
    return TERRAIN.decode(e.d, g.header + (lz * g.tile + lx) * g.record + 1)
end

-- ---- the corridor ---------------------------------------------------------------------

-- Where we are on the cached profile, or nil if it no longer describes this flight path.
function TERRAIN.profIndex()
    local p = TERRAIN.prof
    if not p or not ST.ok then return nil end
    if os.clock() - p.t > CFG.TERRAIN.PROF_STALE then return nil end
    local ptrk = bearingTo(p.dx, p.dz)
    if math.abs(wrap180((ST.trk or ptrk) - ptrk)) > CFG.TERRAIN.TRACK_TOL then return nil end
    local d = (ST.x - p.x) * p.dx + (ST.z - p.z) * p.dz    -- along-track drift since the cut
    if d < -CFG.TERRAIN.BEHIND then return nil end
    local i0 = math.floor(math.max(0, d) / p.step)
    if i0 >= p.n then return nil end
    return i0, p
end

function TERRAIN.span(secs)
    local i0, p = TERRAIN.profIndex()
    if not i0 then return nil end
    local reach = math.max(0, (ST.gs or 0) * (secs or 0))
    return i0, math.min(p.n - 1, i0 + math.floor(reach / p.step)), p
end

-- The forward look, in ONE pass, as everything the terrain modes need:
--
--   here     clearance under the aircraft right now
--   worst    the LEAST clearance anywhere in the window, at the CURRENT altitude
--   closure  m/s the gap is shutting = (terrain rise along track) + (sink rate)
--   isObs    whether the worst thing in the window is a floating mass, not ground
--   obs      whether any charted obstacle in the window is one we will not clear
--
-- `worst` IS PROJECTED, BUT ONLY AS FAR AS VS_HORIZON. Clearance at a point d metres ahead
-- is read against the altitude the aircraft will have when it gets there -- which is the
-- only honest way to ask whether a descent is going to hit something. What is NOT honest is
-- believing that descent to the end of a 30 s window: a steady 3 m/s sink extrapolated that
-- far subtracts 90 m from everything, and every ordinary arrival reads as an impact. Past
-- VS_HORIZON the aircraft is assumed to have levelled.
--
-- ONE PASS, ONCE A TICK. Mode 2 and FLTA between them used to walk this profile three times
-- per tick; at 600 samples and 20 Hz that is thirty-six thousand string decodes a second,
-- which a CC computer notices. GPWS.lookOnce() calls this and both modes read the result.
function TERRAIN.lookAhead(secs)
    if not TERRAIN.available() then return nil end
    local i0, last, p = TERRAIN.span(secs)
    if not i0 then return nil end
    local rec = TERRAIN.geo.record
    -- The ESTIMATED rate, never the raw one: a raw VS oscillating +-5 m/s extrapolates
    -- to +-50 m over VS_HORIZON and annunciates on every negative half-cycle.
    local gs, vs, y = math.max(ST.gs or 0, 0), ST.vsf or 0, ST.y
    local oc, minMass = CFG.TERRAIN.OBST_CLEAR, CFG.TERRAIN.MIN_MASS
    local hz, clearM, minT = CFG.TERRAIN.VS_HORIZON, CFG.TERRAIN.CLEAR_M, CFG.TERRAIN.MIN_T

    local hereTop, worst, worstT, worstObs, seen, obs = nil, nil, 0, false, 0, false
    local req, reqT, reqObs = nil, 0, false

    for i = i0, last do
        local gy, oy, ob, f = TERRAIN.decode(p.d, i * rec + 1)
        if gy or oy then
            seen = seen + 1
            local top, isO = gy, false
            if oy and (not top or oy > top) then top, isO = oy, true end
            if i == i0 then hereTop = top end
            if top then
                local t = (gs > 0.5) and ((i - i0) * p.step / gs) or 0
                local c = (y + vs * math.min(t, hz)) - top
                if not worst or c < worst then worst, worstObs, worstT = c, isO, t end
                -- THE ESCAPE RATE, and the whole of what the terrain modes now alert on:
                -- the climb needed, from where the aeroplane will actually BE when it gets
                -- there, to cross this with CLEAR_M to spare.
                --
                -- Samples nearer than MIN_T are excluded. Terrain underfoot is not something
                -- to be climbed over -- it is already being flown past -- and a divisor
                -- approaching zero turns every overflight into an emergency.
                if t >= minT then
                    local r = (top + clearM - (y + vs * math.min(t, hz))) / t
                    if not req or r > req then req, reqT, reqObs = r, t, isO end
                end
            end
            -- In the way means we would pass THROUGH it: comfortably over the top, or
            -- comfortably under the base, and a floating mass is not our problem.
            if oy and ob and (f % (TERRAIN.F.MASS + 1)) >= minMass
                    and y < oy + oc and y > ob - oc then
                obs = true
            end
        end
    end
    if seen == 0 or not worst then return nil end

    local here = hereTop and (y - hereTop) or nil
    -- Closure, kept for the debug line: the rate the gap is shutting, from both causes at once -- ground rising
    -- along the track, and the aircraft sinking towards it. Differencing the projected
    -- clearance captures both, because `worst` already carries the vertical projection.
    --
    -- Two guards. With the worst point underfoot -- a VTOL descending vertically, or flat
    -- ground -- there is no horizon to difference against and the only honest closure is
    -- our own descent. And the divisor is floored at a second: the worst sample can be
    -- metres away, and a clearance difference over a fraction of a second reports hundreds
    -- of metres per second, which is arithmetic rather than information.
    local closure = -vs
    if here and worstT > 0.5 then closure = (here - worst) / math.max(worstT, 1.0) end
    return here, worst, closure, worstObs, obs, req, reqT, reqObs
end

function TERRAIN.aheadClearance(secs)
    if not TERRAIN.available() then return nil end
    local i0, last, p = TERRAIN.span(secs)
    if not i0 then return nil end
    local rec, worst, seen = TERRAIN.geo.record, nil, 0
    for i = i0, last do
        local gy, oy = TERRAIN.decode(p.d, i * rec + 1)
        if gy or oy then
            seen = seen + 1
            local top = gy
            if oy and (not top or oy > top) then top = oy end
            if top then
                local c = ST.y - top
                if not worst or c < worst then worst = c end
            end
        end
    end
    if seen == 0 then return nil end     -- the whole swathe is unsurveyed: say nothing
    return worst
end

function TERRAIN.obstacleAhead(secs)
    if not TERRAIN.available() then return nil end
    local _, _, _, _, obs = TERRAIN.lookAhead(secs)
    if obs == nil then return nil end
    return obs
end

-- ---- talking to #68 --------------------------------------------------------------------

function TERRAIN.request()
    if not (CFG.TERRAIN.ENABLE and ST.ok) then return end
    local dx, dz
    if (ST.gs or 0) > CFG.TERRAIN.MIN_GS then
        dx, dz = ST.vx, ST.vz
    else
        -- Too slow for the ground track to mean anything: look where the nose points.
        -- This is the inverse of toCompass, and it is why a hovering VTOL still gets a
        -- profile out in front of it.
        local r = math.rad(180 - (ST.hdg or 0))
        dx, dz = math.sin(r), math.cos(r)
    end

    local step  = CFG.TERRAIN.STEP
    local reach = math.max(CFG.TERRAIN.MIN_RANGE,
                           (ST.gs or 0) * CFG.TERRAIN.LOOK_AHEAD * CFG.TERRAIN.RANGE_MARGIN)
    local n = math.min(CFG.TERRAIN.MAX_SAMPLES, math.floor(reach / step) + 2)

    TERRAIN.reqId = TERRAIN.reqId + 1
    TERRAIN.asked = TERRAIN.asked + 1
    local m = { t = "corr", id = TERRAIN.reqId, x = ST.x, z = ST.z, dx = dx, dz = dz,
                step = step, n = n, w = CFG.TERRAIN.WIDTH, alt = ST.y,
                callsign = SHIP.CALLSIGN }
    if TERRAIN.srv then rednet.send(TERRAIN.srv, m, LINKS.TERRAIN)
    else rednet.broadcast(m, LINKS.TERRAIN) end
end

function TERRAIN.onMsg(sender, msg)
    -- srv marks a message as coming FROM the server. Without that test another aircraft's
    -- request, which lands here on the same broadcast protocol, would be latched as the
    -- server and every question would go to an aeroplane.
    if type(msg) ~= "table" or not msg.srv then return end
    TERRAIN.srv, TERRAIN.heard = sender, os.clock()

    if msg.t == "beacon" then
        local g = msg.geo
        if type(g) == "table" and g.minY and g.cell and g.levels and g.tile and g.header
                and g.record then
            TERRAIN.geo, TERRAIN.rev, TERRAIN.fault = g, msg.rev, nil
            TERRAIN.tilesN = msg.tiles
        else
            TERRAIN.fault = "server sent no geometry"
        end

    elseif msg.t == "corr" then
        if type(msg.prof) == "string" and TERRAIN.geo and tonumber(msg.n)
                and #msg.prof >= msg.n * TERRAIN.geo.record then
            TERRAIN.prof = { x = msg.x, z = msg.z, dx = msg.dx, dz = msg.dz,
                             step = msg.step, n = msg.n, d = msg.prof, t = os.clock() }
            TERRAIN.served = TERRAIN.served + 1
        end

    elseif msg.t == "tile" then
        -- A nil `data` is a real answer: that region was never generated. Cached as false
        -- so the same empty ocean is not asked about once per chunk.
        TERRAIN.putTile(msg.rx, msg.rz, msg.data or false)
    end
end

function TERRAIN.loop()
    while true do
        if CFG.TERRAIN.ENABLE then
            if not TERRAIN.srv or not TERRAIN.geo then
                rednet.broadcast({ t = "hello", callsign = SHIP.CALLSIGN,
                    x = ST.x, y = ST.y, z = ST.z }, LINKS.TERRAIN)
            end
            pcall(TERRAIN.request)
        end
        sleep(CFG.TERRAIN.REQ_EVERY)
    end
end

-- What the terrain service is doing, in words, for the HUD's system line.
function TERRAIN.why()
    if not CFG.TERRAIN.ENABLE then return "TERR OFF (disarmed)" end
    if TERRAIN.fault then return "TERR " .. TERRAIN.fault end
    if not TERRAIN.heard then return "TERR no server" end
    local age = os.clock() - TERRAIN.heard
    if age > CFG.TERRAIN.STALE then return string.format("TERR silent %ds", math.floor(age)) end
    if not TERRAIN.geo then return "TERR no geometry" end
    if not TERRAIN.prof then return "TERR srv ok, no profile" end
    return string.format("TERR ok rev %s", tostring(TERRAIN.rev or "?"))
end

-- ============================================================================
-- THE ALERT SET
-- ============================================================================
-- One table per alert, sorted once by priority. PRIO is the whole arbitration: the highest
-- active alert is the one on the banner and the one in the speaker, and a higher one
-- interrupts a lower one mid-word.

local ALERTS, ALERT = {}, {}
do
    for _, a in ipairs(CFG.ALERTS) do ALERTS[#ALERTS + 1] = a ALERT[a.id] = a end
    table.sort(ALERTS, function(p, q) return p.prio > q.prio end)
end

-- ============================================================================
-- GPWS -- the envelopes.
-- ============================================================================

local GPWS = { st = {}, inhibit = nil, armed = false }
for _, a in ipairs(ALERTS) do
    GPWS.st[a.id] = { active = false, seq = 0, since = 0, latchUntil = 0 }
end
-- Live state, not config: CFG.DEBUG.ENABLE is only where it STARTS. The key toggle moves
-- this one, so the pilot can ask the question in the air instead of on the ground.
GPWS.debug = CFG.DEBUG.ENABLE and true or false

-- Hold an alert on or off. The rising edge bumps `seq`, which is what tells the audio layer
-- this is a NEW call rather than the same one still running.
-- The last alert the BANNER SHOWED, kept so the quiet rule can still answer for it. A GPWS
-- says its piece and stops; "what was that" is a question asked about three seconds after
-- the banner has already cleared, and until now nothing on this aircraft could answer it.
--
-- Recorded from GPWS.top(), not from the rising edge of whichever mode happened to trip
-- last. Three modes routinely go up together and the one that rises last is an artefact of
-- the order tick() runs them in -- FLTA, as it happens. The pilot did not see FLTA's
-- TERRAIN AHEAD; they saw Mode 2's TERRAIN TERRAIN PULL UP, because that is what outranked
-- it, and that is what the recall has to name.
GPWS.last = nil
function GPWS.remember(id)
    local a = ALERT[id]
    if not a then return end
    GPWS.last = { id = id, text = a.text, mode = a.mode or "?", level = a.level, t = os.clock() }
end

function GPWS.set(id, on)
    local s = GPWS.st[id]
    if not s then return end
    if on and not s.active then s.seq, s.since = s.seq + 1, os.clock() end
    s.active = on and true or false
end

-- A one-shot: speak once, hold the banner for LATCH seconds.
function GPWS.fire(id)
    local s, a = GPWS.st[id], ALERT[id]
    if not s or not a then return end
    s.seq, s.since = s.seq + 1, os.clock()
    s.latchUntil = os.clock() + (a.latch or 3.0)
end

-- IS IT REALLY UP -- the condition still holds, or a one-shot's latch has not run out. This
-- is what the SPEAKER goes by, and it is the older, narrower of the two tests.
function GPWS.speakable(id)
    local s = GPWS.st[id]
    return s and (s.active or s.latchUntil > os.clock())
end

-- IS IT ON THE BANNER -- the same thing, plus a MINIMUM DISPLAY TIME measured from the
-- rising edge.
--
-- An envelope that trips for a single 20 Hz tick used to paint a banner for one frame, which
-- at HUD.REDRAW is a banner nobody sees. Worse, it is the visual half of the same problem the
-- voice has (see CFG.VOICE.MIN_DWELL): the speaker will no longer start a clip for a call
-- that short, so without this floor the pilot would get neither the word nor a readable
-- banner and the mode would trip in complete silence. GP.MIN_SHOW is that floor.
--
-- The `seq > 0` guard is not decoration: `since` starts at 0 and so does os.clock(), so
-- without it every alert in the table would light up for the computer's first half second.
function GPWS.showing(id)
    local s = GPWS.st[id]
    if not s then return false end
    local now = os.clock()
    return s.active or s.latchUntil > now or (s.seq > 0 and (now - s.since) < GP.MIN_SHOW)
end

-- The highest-priority alert currently up. `needClip` restricts it to ones this voice pack
-- can actually speak, which is how SINK RATE and BANK ANGLE stay visible without stealing
-- the speaker from a warning that has words. `live` asks the narrow question -- still really
-- up, not merely being held on the glass by MIN_SHOW -- which is the one the audio layer
-- wants: a blip that has already stopped must never reach the speaker.
function GPWS.top(needClip, clipTest, live)
    local up = live and GPWS.speakable or GPWS.showing
    for i = 1, #ALERTS do
        local a = ALERTS[i]
        if up(a.id) and (not needClip or clipTest(a.id)) then return a, GPWS.st[a.id] end
    end
end

function GPWS.clearAll()
    for _, a in ipairs(ALERTS) do GPWS.st[a.id].active = false end
end

-- Master inhibits, evaluated once a tick. Returns a reason string, or nil when the system
-- is armed. The reason is what the quiet banner shows, so "why is it not warning me" is
-- answered on the glass rather than in the manual.
function GPWS.inhibitReason()
    local I = GP.INHIBIT
    if (os.clock() - ST.boot) < I.AFTER_BOOT then return "SELF TEST" end
    if not ST.fixFresh() then return "NO FIX" end
    if I.MASTER_OFF then
        local e = LINK.get("engine", STALE.ENGINE)
        if e and not e.master then return "MASTER OFF" end
    end
    -- ST.onGround is the ONLY ground test. There is deliberately no second, unguarded
    -- "RA below 2 m" line here: that is the same test without the speed guards, and it
    -- would silence the system for an aircraft arriving below field elevation at speed.
    if I.ON_GROUND and ST.onGround then return "ON GROUND" end
    if I.NO_DATUM and not ST.ra then return "NO DATUM" end
    return nil
end

-- ONE forward look per tick, cached here for Mode 2 and FLTA to share. Walking the
-- profile is by a distance the most expensive thing this computer does; doing it once is
-- the difference between a GPWS and a computer too busy to annunciate one.
GPWS.lk = {}

-- How far ahead it is worth looking, in seconds. LOOK_AHEAD, cut short at the waypoint the
-- FMS is actually flying to: the profile is a straight line along the present track and the
-- route is not, so past the next fix it describes ground the aeroplane will turn away from.
function GPWS.lookSecs()
    local secs = CFG.TERRAIN.LOOK_AHEAD
    local tgt = LINK.get("target", STALE.APNAV)
    local gs = ST.gs or 0
    if tgt and tonumber(tgt.x) and tonumber(tgt.y) and gs > 1 then
        -- APNAV names the target's Z as `y` -- it is a ground position, not an altitude.
        local d = dist2d(ST.x, ST.z, tgt.x, tgt.y) + CFG.TERRAIN.LEG_MARGIN
        local t = d / gs
        if t < secs then secs = t end
    end
    return secs
end

function GPWS.lookOnce()
    local k = GPWS.lk
    k.secs = GPWS.lookSecs()
    k.here, k.worst, k.closure, k.isObs, k.obs, k.req, k.reqT, k.reqObs =
        TERRAIN.lookAhead(k.secs)
end

-- Is the FMS flying a landing right now? "cancel" is it saying it has STOPPED, so that is
-- not a landing; every other command it broadcasts is a live one.
function GPWS.landingCmd()
    local al = LINK.get("autoland", STALE.AUTOLAND)
    if not al or not al.cmd or al.cmd == "cancel" then return nil end
    return al.cmd
end

-- A landing being flown NORMALLY. Closing on the ground is the point of a landing, so the
-- terrain modes stand down for one -- but only while the descent is one the FMS would
-- actually command. Arriving at the pad at 20 m/s is not a landing, it is an accident in
-- progress, and it is exactly the case the inhibit must not swallow.
function GPWS.normalLanding()
    if not GPWS.landingCmd() then return false end
    -- Estimated, so that one bad half-cycle cannot drop the inhibit mid-landing and let
    -- a warning escape.
    return (ST.vsf or 0) > -GP.INHIBIT.LANDING_VS
end

-- THE TRUE LANDING PHASE: the FMS is flying a landing and broadcasting a LANDING TARGET.
-- That is exactly when the HSI switches to its landing display and draws the glideslope
-- pointer, and it is the only time a DECISION HEIGHT callout means anything.
--
-- IT IS NOT "the autoland channel is live". A GO-AROUND keeps the vertical channel while
-- LND.active is false -- autopilot.lua's broadcastVertical() streams cmd="descend" whenever
-- LND.ga is set, and says why: "the route machinery is flying the circuit, but its altitude
-- is ours to hold". So landingCmd() is true for the WHOLE go-around, and the callout rang
-- all the way round it.
--
-- NOR is it "near a field". An approach pattern's fixes are ordinary route legs flown well
-- inside the arming radius, so a descent through the decision height while manoeuvring onto
-- final counted as an arrival.
--
-- broadcastCurrent() sends type="rwy" (runway) or type="land" (pad) only under
-- `if LND.active`, and drawHSI dispatches on the same flag. One test, and it is the same
-- one the instrument uses. Note the glideslope POINTER is runway-only ("landing and
-- LND.rwy"); a pad arrival is still a true landing phase and still earns the callout.
function GPWS.onFinal()
    local t = LINK.get("target", STALE.APNAV)
    if type(t) ~= "table" then return false end
    return t.type == "rwy" or t.type == "land"
end

-- MODE 1 -- EXCESSIVE DESCENT RATE. Height and sink rate alone, which is exactly why it is
-- the mode that survives having no terrain data.
function GPWS.mode1()
    local M = GP.MODE1
    if not M.ENABLE or not ST.ra or ST.ra > M.ARM_RA then
        GPWS.set("sink_rate", false) GPWS.set("pull_up", false) return
    end
    local sink, pull = envelope(M.ENVELOPE, "ra", ST.ra, "sink", "pull")
    -- Positive = going down. The estimated trend by default: an airframe oscillating about
    -- zero is not sinking, whatever the instantaneous number says. See MODE1.USE_FILTERED.
    local rate = -((M.USE_FILTERED and ST.vsf or ST.vs) or 0)
    local h = M.HYSTERESIS
    GPWS.set("pull_up",   rate >= (GPWS.st.pull_up.active   and (pull - h) or pull))
    GPWS.set("sink_rate", rate >= (GPWS.st.sink_rate.active and (sink - h) or sink)
                          and not GPWS.st.pull_up.active)
end

-- MODE 2 -- EXCESSIVE TERRAIN CLOSURE.
-- Alerts on the CLIMB RATE THAT WOULD BE NEEDED to clear the worst terrain ahead, not on
-- how near that terrain happens to be. Proximity alone said the same thing at 30 s as at
-- 2 s and annunciated for whole legs over ground the aeroplane was clearing perfectly well.
function GPWS.mode2()
    local M, k = GP.MODE2, GPWS.lk
    if not M.ENABLE or not TERRAIN.available()
            or (M.LANDING_INHIBIT and GPWS.normalLanding()) or not k.req then
        GPWS.set("terrain", false) GPWS.set("terrain_hard", false) return
    end
    -- Arm on measured terrain clearance where we have it, falling back to the datum height.
    local arm = k.here or ST.ra
    if not arm or arm > M.ARM_RA then
        GPWS.set("terrain", false) GPWS.set("terrain_hard", false) return
    end
    -- Hysteresis both ways: an alert already up needs the requirement to genuinely ease
    -- before it stops, or the annunciator stutters as the profile refreshes.
    local h = M.HYSTERESIS
    local warn = GPWS.st.terrain_hard.active and (M.ESCAPE_WARNING - h) or M.ESCAPE_WARNING
    local caut = GPWS.st.terrain.active      and (M.ESCAPE_CAUTION - h) or M.ESCAPE_CAUTION
    GPWS.set("terrain_hard", k.req > warn)
    GPWS.set("terrain", k.req > caut and k.req <= warn)
end

-- FORWARD-LOOKING TERRAIN AND OBSTACLE AVOIDANCE -- the advisory below Mode 2.
-- The same escape rate at a gentler threshold, so the three calls form a ladder off one
-- number rather than three modes arguing about the same ground. Both need the aircraft to
-- be going somewhere (FLTA.MIN_GS): "ahead" is meaningless in a hover.
function GPWS.flta()
    local M, k = GP.FLTA, GPWS.lk
    if not M.ENABLE or not TERRAIN.available() or (ST.gs or 0) < M.MIN_GS
            or (M.LANDING_INHIBIT and GPWS.normalLanding()) or not k.req then
        GPWS.set("terrain_ahead", false) GPWS.set("obstacle_ahead", false) return
    end
    local lim = M.ESCAPE
    if GPWS.st.terrain_ahead.active or GPWS.st.obstacle_ahead.active then
        lim = lim - M.HYSTERESIS
    end
    local threat = k.req > lim

    -- The two calls are mutually exclusive: the wording follows whatever is actually the
    -- worst thing ahead. An obstacle we would pass through earns the call even when bare
    -- ground happens to be what the escape rate is measured against.
    local obstacle = threat and (k.reqObs or k.obs == true)
    GPWS.set("obstacle_ahead", obstacle)
    GPWS.set("terrain_ahead", threat and not obstacle)
end

-- MODE 3 -- ALTITUDE LOSS AFTER TAKE-OFF ("DON'T SINK"). Terrain-free: the reference is the
-- elevation the wheels just left, which is exactly what the DEP datum is.
-- MODE 3 -- ALTITUDE LOSS AFTER TAKE-OFF ("DON'T SINK").
-- Measured against the altitude the wheels left, in metres of sublevel's own y -- NOT in RA.
-- RA has not been a departure-relative height since terrain became a datum: it steps 8 m at
-- a chunk boundary and jumps again when the datum switches to a surveyed field, and neither
-- is altitude the aeroplane lost.
GPWS.peakY, GPWS.liftY = nil, nil
GPWS.m3Armed = false
function GPWS.mode3()
    local M = GP.MODE3
    local gained = (GPWS.peakY and GPWS.liftY) and (GPWS.peakY - GPWS.liftY) or 0
    -- Disarm for the rest of the flight on the first of: climbing ARM_GAIN above the pad, or
    -- simply having been airborne long enough that this is no longer a take-off. Re-armed by
    -- the next lift-off (see GPWS.tick).
    if GPWS.m3Armed and ST.airborneSince then
        if gained > M.ARM_GAIN or (os.clock() - ST.airborneSince) > M.WINDOW then
            GPWS.m3Armed = false
        end
    end
    if not M.ENABLE or not GPWS.m3Armed or not ST.airborneSince or not GPWS.liftY then
        GPWS.peakY = nil GPWS.set("dont_sink", false) return
    end
    -- A COMMANDED ARRIVAL IS NOT A MISTAKE. A short hop stays inside WINDOW and never climbs
    -- ARM_GAIN, so without this the descent onto the destination pad rings the whole way
    -- down -- which is exactly what it did.
    if M.LANDING_INHIBIT and (GPWS.landingCmd() or GPWS.approaching()) then
        GPWS.set("dont_sink", false) return
    end
    if not GPWS.peakY or ST.y > GPWS.peakY then GPWS.peakY = ST.y end
    -- The ESTIMATED rate: a raw VS oscillating about zero crosses CLEAR_ON_CLIMB twice a
    -- second and would keep resetting the peak, quietly disabling the mode.
    if (ST.vsf or ST.vs) >= M.CLEAR_ON_CLIMB then
        GPWS.peakY = ST.y                       -- climbing again: this is the new high point
        GPWS.set("dont_sink", false) return
    end
    local lost = GPWS.peakY - ST.y
    GPWS.set("dont_sink", lost >= math.max(M.LOSS_MIN, M.LOSS_FRAC * math.max(gained, 0)))
end

-- "This is an arrival, not a mistake." Descending at a sane rate toward a field whose
-- elevation we actually have -- which is what an approach looks like from here whether the
-- FMS is flying it or the pilot is. See CFG.GPWS.MODE4's APPROACH_* note for what this
-- deliberately does not cover.
function GPWS.approaching()
    local M = GP.MODE4
    local site = ST.siteNear
    if not site then return false end
    local vs = ST.vsf or ST.vs
    if vs > -M.APPROACH_MIN_VS or vs < -M.APPROACH_VS then return false end
    local dx, dz = site.x - ST.x, site.z - ST.z
    local d = math.sqrt(dx * dx + dz * dz)
    if d <= M.APPROACH_NEAR then return true end
    return (dx * ST.vx + dz * ST.vz) / d > 0        -- closing on the field
end

-- MODE 4 -- UNSAFE CLEARANCE ("TOO LOW"). The true Mode 4 compares radio altitude with
-- terrain and with the landing configuration; the terrain half has terrain behind it now,
-- but gear and flaps this airframe does not have. What runs here is the datum part: fast and low over a field we have an
-- elevation for, with no landing in progress. The floor rises with ground speed -- 15 m is
-- unremarkable in a hover and reckless at 300 m/s.
GPWS.m4Armed = false
function GPWS.mode4()
    local M = GP.MODE4
    if ST.ra and ST.ra > M.ARM_RA then GPWS.m4Armed = true end
    if not M.ENABLE or not GPWS.m4Armed or not ST.ra or ST.gs < M.MIN_GS then
        GPWS.set("too_low", false) return
    end
    if GPWS.landingCmd() then GPWS.set("too_low", false) return end  -- a landing IS meant to be low
    if GPWS.approaching() then GPWS.set("too_low", false) return end
    local floor = envelope(M.FLOOR, "gs", ST.gs, "ra")
    if GPWS.st.too_low.active then floor = floor + M.HYSTERESIS end
    GPWS.set("too_low", ST.ra < floor)
end

-- MODE 6 -- CALLOUTS. Decision height on the way down, one per approach; and the bank-angle
-- caution, which is datum-free and so works everywhere, including en route with no datum.
-- Starts DISARMED and is armed only by climbing through DH + RESET. Losing the datum does
-- NOT arm it: arriving from cruise, RA reappears as a large number and arms it on the way
-- past, which is the moment that actually means an approach has begun.
GPWS.dhArmed = false
function GPWS.mode6()
    local M = GP.MODE6
    if M.ENABLE and ST.ra then
        if ST.ra > M.DH + M.RESET then GPWS.dhArmed = true end
        if GPWS.dhArmed and ST.ra <= M.DH
                and (not M.REQUIRE_DESCENT or ST.vs < 0)
                and (not M.REQUIRE_LANDING or GPWS.onFinal()) then
            GPWS.dhArmed = false
            GPWS.fire("decision")
        end
    end

    if not M.BANK_ENABLE then GPWS.set("bank_angle", false) return end
    local b = math.abs(ST.bank)
    local limit = M.BANK_CAUTION
    if GPWS.st.bank_angle.active then limit = limit - M.BANK_HYST end
    GPWS.set("bank_angle", b >= limit)
end

-- AUTOPILOT DISCONNECT. Not a GPWS mode -- the other thing this speaker is for. Fires on
-- either switch going from engaged to disengaged, and (optionally) on an engaged channel
-- falling silent: a dead brain announces nothing, which from here looks identical to a
-- disengagement and is at least as worth knowing about.
GPWS.apPrev = { apnav = nil, apalt = nil }
function GPWS.apdisc()
    local A = GP.APDISC
    if not A.ENABLE then return end
    for _, key in ipairs({ "apnav", "apalt" }) do
        local now = LINK.get(key, STALE.APSWITCH)
        local was = GPWS.apPrev[key]
        local dropped = false
        if was == true and now == false then dropped = true end
        if was == true and now == nil and A.ON_SILENCE then dropped = true end
        if dropped and not (A.INHIBIT_ON_GROUND and ST.onGround) then GPWS.fire("ap_disconnect") end
        -- nil (silent) is remembered as nil, so a channel that comes back engaged re-arms.
        GPWS.apPrev[key] = now
    end
end

function GPWS.tick()
    -- Ground state first: it gates everything, and it is the one thing this computer knows
    -- from a real sensor rather than from an inference.
    local wasGround = ST.onGround
    ST.onGround = DATUM.groundTruth()
        or (ST.ra ~= nil and ST.ra < GP.INHIBIT.RA_BELOW
            and ST.gs < GP.INHIBIT.GROUND_GS and math.abs(ST.vs) < GP.INHIBIT.GROUND_VS)
    if ST.onGround and not wasGround then
        ST.groundSince, ST.airborneSince = os.clock(), nil
        GPWS.m4Armed = false          -- the next departure gets its take-off inhibit back
        GPWS.dhArmed = false          -- ...and the next approach has to earn its callout
    end
    if wasGround and not ST.onGround then
        ST.airborneSince = os.clock()
        -- A take-off is the only thing that arms "don't sink", and it is also what fixes the
        -- reference: the altitude the wheels just left.
        GPWS.m3Armed, GPWS.peakY, GPWS.liftY = true, ST.y, ST.y
    end                                          -- "don't sink"

    GPWS.apdisc()          -- outside the inhibit: the autopilot dropping out matters even
                           -- with no fix, no datum and nothing else to say about it.
    GPWS.inhibit = GPWS.inhibitReason()
    GPWS.armed = (GPWS.inhibit == nil)
    if not GPWS.armed then
        -- Drop every held condition. One-shots keep their latch: the pilot should still see
        -- the banner of the call that just went out, even if the aircraft has since landed.
        for _, a in ipairs(ALERTS) do
            if a.id ~= "ap_disconnect" then GPWS.st[a.id].active = false end
        end
        GPWS.peakY = nil
        return
    end
    GPWS.lookOnce()        -- one profile walk, for mode2 and flta both
    GPWS.mode1()
    GPWS.mode2()
    GPWS.flta()
    GPWS.mode3()
    GPWS.mode4()
    GPWS.mode6()
    -- ...and last, note what all of that is going to put on the banner, so the quiet rule
    -- can still name it once it has gone.
    local shown = GPWS.top(false)
    if shown then GPWS.remember(shown.id) end
end

-- ============================================================================
-- AUDIO -- the voice pack.
-- ============================================================================
-- The clips live on a floppy in the drive, not on this computer: the pack is shared fleet
-- equipment and swapping the disk swaps the voice. The drive is found by looking for the
-- MANIFEST on it, not by peripheral name -- with more than one drive attached, the one with
-- the voice pack is the one with _voicepack.md on it.
--
-- THE CLIPS ARE RAW PCM, NOT DFPWM. Every file on the disk is mono UNSIGNED 8-BIT at
-- 48 kHz -- one byte, one sample -- which is precisely the shape speaker.playAudio wants
-- once 128 is taken off each byte to centre it. There is no decoder any more and no
-- cc.audio.dfpwm dependency: a chunk is read, subtracted and pushed. DFPWM bought a 1:8
-- size saving on a floppy that has room to spare, and charged for it in Lua bit arithmetic
-- per sample, on the one computer in the aircraft that must not be busy when the ground
-- comes up.
--
-- Playback otherwise follows https://tweaked.cc/guide/speaker_audio.html: chunks are pushed
-- with speaker.playAudio, which returns false when the speaker's backlog is full and is
-- retried on speaker_audio_empty. CFG.VOICE.CHUNK is now bytes of PCM, i.e. samples, per
-- push -- 16 KiB is a third of a second and well inside playAudio's 128 Ki maximum.
--
-- PREEMPTION is the part that is not in the guide and is the part that matters: a warning
-- that has to wait for a callout to finish is a warning that arrives after the ground does.
-- Between every chunk, and while waiting out a clip's tail, the player asks whether a
-- higher-priority alert has come up; if one has, it stops the speaker mid-word and returns.
--
-- WHAT IS NOT STARTED IS NEVER CUT SHORT. A clip that is play()ed and then stop()ped almost
-- immediately is not merely inaudible: CC queues it anyway and plays it AHEAD of the next
-- clip that is genuinely commanded, so one flickering envelope turns the next real warning
-- into the wrong word. The fix is upstream of the speaker -- an alert has to have been up
-- for CFG.VOICE.MIN_DWELL before AUDIO.want() will look at it. See AUDIO.dwelt.

local AUDIO = {
    dir = nil, clips = 0, missing = {}, fault = nil,
    playingId = nil, playingPrio = -1, lastEnd = {}, spoken = {}, nextScan = 0,
    selftest = nil,
}

local sbyte = string.byte

local nullsamples = { 0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0, }

-- One chunk of file bytes -> the amplitude table playAudio takes. Unsigned 0..255 centred
-- on 0x80 becomes signed -128..127; playAudio range-checks and would refuse the raw byte.
--
-- string.byte is asked for BYTES_PER_CALL bytes at a time rather than the whole chunk in one
-- go: it returns one Lua value per byte, and a multi-return sixteen thousand long is how you
-- find out what Cobalt's stack is worth.
local BYTES_PER_CALL = 256
local function pcmSamples(s, out)
    local n, o, i = #s, 0, 1
    while i <= n do
        local j = i + BYTES_PER_CALL - 1
        if j > n then j = n end
        local b = { sbyte(s, i, j) }
        for k = 1, #b do out[o + k] = b[k] - 128 end
        o = o + #b
        i = j + 1
    end
    return o
end

local function speakerStop()
    HW.speaker.stop() HW.speaker.stop()
    -- play dummy sample
    HW.speaker.playAudio(nullsamples)
    HW.speaker.stop() HW.speaker.stop()
end

-- The files that make up one alert's phrase, in speaking order -- or nil if the alert has no
-- voice at all, or if ANY word of its phrase is missing from the disk. All-or-nothing is the
-- point: the pack's manifest says "TOO LOW" is a modifier that qualifies the word after it,
-- so a phrase with a word missing is not a shorter warning, it is a different one.
function AUDIO.pathsFor(id)
    if not AUDIO.dir then return nil end
    local stems = VOICE.CLIPS[id]
    if not stems then return nil end
    local out = {}
    for i = 1, #stems do
        local p = fs.combine(AUDIO.dir, VOICE.LANG .. "_" .. stems[i] .. ".bin")
        if not fs.exists(p) then return nil end
        out[i] = p
    end
    return out
end

-- Find the voice-pack disk and take stock of it. Re-run when the disk changes, so swapping
-- the pack in flight works and is reported.
function AUDIO.scan()
    AUDIO.nextScan = os.clock() + 5
    AUDIO.dir, AUDIO.clips, AUDIO.missing = nil, 0, {}
    if not VOICE.ENABLE then AUDIO.fault = "voice disabled" return end
    if VOICE.DIR then
        if fs.exists(VOICE.DIR) then AUDIO.dir = VOICE.DIR end
    else
        for _, name in ipairs(peripheral.getNames()) do
            if peripheral.getType(name) == "drive" then
                local ok, mp = pcall(peripheral.call, name, "getMountPath")
                if ok and mp and fs.exists(fs.combine(mp, VOICE.MARKER)) then
                    AUDIO.dir = mp
                    break
                end
            end
        end
    end
    if not AUDIO.dir then AUDIO.fault = "no voice pack disk" return end
    -- Counted in PHRASES the system can actually speak, not in files on the disk: a disk
    -- holding eight of the nine words TOO LOW TERRAIN needs is a disk that cannot say it.
    for id in pairs(VOICE.CLIPS) do
        if AUDIO.pathsFor(id) then AUDIO.clips = AUDIO.clips + 1
        else AUDIO.missing[#AUDIO.missing + 1] = id end
    end
    if AUDIO.clips == 0 then AUDIO.fault = "no '" .. VOICE.LANG .. "' phrases on the disk"
    elseif #AUDIO.missing > 0 then
        table.sort(AUDIO.missing)
        AUDIO.fault = "no voice for " .. table.concat(AUDIO.missing, ",")
    else AUDIO.fault = nil end
end
AUDIO.scan()

function AUDIO.available() return HW.speaker ~= nil and AUDIO.dir ~= nil end

-- HAS IT BEEN UP LONG ENOUGH TO BE WORTH SPEAKING? CFG.VOICE.MIN_DWELL, and the whole reason
-- it exists is in the AUDIO header: a clip started and stopped inside a few ticks is queued
-- by CC and then played AHEAD of the next commanded clip, so a mode that flickers does not
-- merely say nothing, it makes the following warning say the wrong thing.
--
-- A LATCHED ONE-SHOT SPEAKS AT ONCE. Its length is not in question -- GPWS.fire() has
-- already committed the banner for LATCH seconds and nothing in the world can shorten that
-- -- so waiting would just put DECISION HEIGHT and AUTOPILOT DISCONNECTED half a second
-- late for no gain. Only the envelope-driven alerts, whose future is genuinely unknown, have
-- to serve out the dwell.
--
-- The test is "latched at all", not "latched for at least MIN_DWELL", on purpose: the second
-- form would silently mute any alert someone gave a LATCH shorter than the dwell, because by
-- the time the dwell was served the latch would have run out and GPWS.speakable() would
-- already say no. A one-shot cannot suffer the queued-clip problem in any case -- speak() is
-- only ever cut short by a HIGHER alert, never by its own condition going away.
function AUDIO.dwelt(id)
    local s = GPWS.st[id]
    if not s then return false end
    local now = os.clock()
    if s.latchUntil > now then return true end
    return (now - s.since) >= VOICE.MIN_DWELL
end

-- Speakable AND settled: what AUDIO.want() screens on.
function AUDIO.ready(id)
    return AUDIO.dwelt(id) and AUDIO.pathsFor(id) ~= nil
end

-- The alert that should be sounding right now: highest priority, genuinely still up (not
-- merely held on the banner by MIN_SHOW), dwelt, and with a phrase on the disk.
function AUDIO.want()
    return (GPWS.top(true, AUDIO.ready, true))
end

-- Has something outranked what is playing? Checked between chunks and while waiting out a
-- tail, which is what makes a warning able to cut a callout off mid-word.
function AUDIO.outranked(prio)
    local a = AUDIO.want()
    return a ~= nil and a.prio > prio
end

-- Interruptible wait. Returns false if something outranked us.
function AUDIO.hold(untilClock, prio)
    while os.clock() < untilClock do
        if AUDIO.outranked(prio) then return false end
        sleep(0.05)
    end
    return true
end

-- Speak one alert's whole phrase.
--
-- The words are pushed into the speaker as ONE STREAM -- every chunk of every file, in order,
-- through the same backpressure loop -- so TERRAIN TERRAIN--PULL UP comes out as a phrase
-- rather than as two announcements with a seam between them. With raw PCM there is no
-- per-file decoder state to worry about at the join any more: the bytes of the second file
-- mean the same thing whatever preceded them.
function AUDIO.speak(a)
    local paths = AUDIO.pathsFor(a.id)
    if not paths or not HW.speaker then return end

    AUDIO.playingId, AUDIO.playingPrio = a.id, a.prio
    local samples, cut = 0, false
    local t0 = os.clock()
    for i = 1, #paths do
        if cut then break end
        local h = fs.open(paths[i], "rb")
        if not h then
            -- The disk went away mid-phrase (ejected, or never really there). Drop the scan
            -- so the next pass re-finds it, and stop -- half a phrase is not worth saying.
            AUDIO.fault = "cannot read " .. paths[i]
            AUDIO.dir, cut = nil, true
            break
        end
        while true do
            if AUDIO.outranked(a.prio) then cut = true break end
            local ok, chunk = pcall(h.read, VOICE.CHUNK)
            if not ok or not chunk or #chunk == 0 then break end
            local buf = {}
            samples = samples + pcmSamples(chunk, buf)
            while not HW.speaker.playAudio(buf, VOICE.VOLUME) do
                if AUDIO.outranked(a.prio) then cut = true break end
                -- Retry on speaker_audio_empty; the timer is only so the preemption test
                -- above gets a look in at a sane rate if that event is slow in coming.
                local timer = os.startTimer(0.1)
                while true do
                    local ev, p = os.pullEvent()
                    if ev == "speaker_audio_empty" then os.cancelTimer(timer) break end
                    if ev == "timer" and p == timer then break end
                end
            end
            if cut then break end
        end
        pcall(h.close)
    end

    if cut then
        pcall(speakerStop)
    else
        -- playAudio QUEUES; it does not block until the sound has finished. The clip's real
        -- length is what the repeat gap has to be measured from, or a looping warning
        -- stutters over itself. Every clip used to fit in one DFPWM chunk, so this hold was
        -- where a preemption almost always landed; a CHUNK of PCM is a third of a second, so
        -- a cut now falls between chunks about as often as it falls here.
        --
        -- Being outranked here means the words are still coming out of the speaker with
        -- nothing left to push, so the speaker has to be STOPPED explicitly. Without this,
        -- PULL UP starts on top of the tail of the callout it just interrupted and the two
        -- play over each other -- which is how the one alert that has to be understood
        -- becomes the one that cannot be.
        if not AUDIO.hold(t0 + samples / VOICE.RATE, a.prio) then pcall(speakerStop) end
    end
    AUDIO.lastEnd[a.id] = os.clock()
    AUDIO.spoken[a.id] = GPWS.st[a.id].seq
    AUDIO.playingId, AUDIO.playingPrio = nil, -1
end

function AUDIO.due(a)
    local s = GPWS.st[a.id]
    if AUDIO.spoken[a.id] ~= s.seq then return true end          -- a NEW call
    if not a.loop then return false end                          -- one-shot, already spoken
    return os.clock() >= (AUDIO.lastEnd[a.id] or 0) + (a.gap or 1.0)
end

function AUDIO.loop()
    while true do
        if not AUDIO.available() then
            if os.clock() >= AUDIO.nextScan then AUDIO.scan() end
            sleep(0.25)
        else
            local a = AUDIO.want()
            if a and AUDIO.due(a) then
                AUDIO.speak(a)
            elseif AUDIO.selftest then
                AUDIO.runSelfTest()
            else
                sleep(0.05)
            end
        end
    end
end

-- SELF TEST. The only way to find out that the disk is missing, a file is corrupt or the
-- speaker has been unbolted -- before the ground is the thing that tells you. Every clip in
-- priority order, banner and all. A real alert outranks it instantly and cancels the rest.
function AUDIO.runSelfTest()
    for i = 1, #ALERTS do
        local a = ALERTS[i]
        if not AUDIO.selftest then break end
        if AUDIO.want() then break end     -- a real call is ready to speak: abandon
        if AUDIO.pathsFor(a.id) then
            AUDIO.selftestId = a.id
            local fake = { id = a.id, prio = -1 }         -- prio -1: anything outranks it
            AUDIO.speak(fake)
            AUDIO.spoken[a.id] = nil                      -- do not count a test as spoken
            sleep(CFG.SELFTEST.GAP)
        end
    end
    AUDIO.selftest, AUDIO.selftestId = nil, nil
    AUDIO.scan()
end

-- ============================================================================
-- FRAMEBUFFER -- one blit per changed row.
-- ============================================================================
-- The HUD repaints 20 times a second on a monitor that may be 160 columns wide. Painting it
-- cell by cell would spend the whole tick in peripheral calls, so the frame is composed into
-- character/colour arrays and flushed a row at a time, and a row whose three strings are
-- identical to last frame's is not sent at all. The system and route lines therefore cost
-- nothing on a frame where nothing changed, which is most of them.

local BLIT = {}
for i = 0, 15 do BLIT[2 ^ i] = ("0123456789abcdef"):sub(i + 1, i + 1) end

local FB = { W = 0, H = 0, ch = {}, fg = {}, bg = {}, prev = {}, clipT = 1, clipB = 1e9 }

function FB.size(w, h)
    if FB.W == w and FB.H == h then return end
    FB.W, FB.H, FB.ch, FB.fg, FB.bg, FB.prev = w, h, {}, {}, {}, {}
    for r = 1, h do FB.ch[r], FB.fg[r], FB.bg[r] = {}, {}, {} end
end

-- A row window everything is clipped to. The attitude field needs it: a pitch ladder is a
-- set of long lines that only PART of belongs on the screen, and without a clip a rung with
-- one endpoint just outside the field scribbles across the system and route lines above it.
function FB.clip(top, bot) FB.clipT, FB.clipB = top or 1, bot or 1e9 end

function FB.clear(fgc, bgc)
    local f, b = BLIT[fgc or colours.lime], BLIT[bgc or colours.black]
    for r = 1, FB.H do
        local cr, fr, br = FB.ch[r], FB.fg[r], FB.bg[r]
        for c = 1, FB.W do cr[c], fr[c], br[c] = " ", f, b end
    end
end

function FB.put(c, r, ch, fgc, bgc)
    if c < 1 or c > FB.W or r < 1 or r > FB.H then return end
    if r < FB.clipT or r > FB.clipB then return end
    c, r = math.floor(c), math.floor(r)
    FB.ch[r][c] = ch
    if fgc then FB.fg[r][c] = BLIT[fgc] end
    if bgc then FB.bg[r][c] = BLIT[bgc] end
end

function FB.text(c, r, s, fgc, bgc)
    for i = 1, #s do FB.put(c + i - 1, r, s:sub(i, i), fgc, bgc) end
end

-- Right-anchored, for the readouts that hang off the end of a strip.
function FB.textR(c, r, s, fgc, bgc) FB.text(c - #s + 1, r, s, fgc, bgc) end

function FB.fill(r, fgc, bgc)
    for c = 1, FB.W do FB.put(c, r, " ", fgc, bgc) end
end

-- DDA over the longer axis, so a near-vertical horizon (which is what 80 degrees of bank
-- looks like) rasterises as well as a level one.
function FB.line(x0, y0, x1, y1, ch, fgc)
    local dx, dy = x1 - x0, y1 - y0
    local n = math.max(math.abs(dx), math.abs(dy))
    if n < 1 then FB.put(x0, y0, ch, fgc) return end
    n = math.min(n, 4 * (FB.W + FB.H))       -- a slope near infinity must not become a loop
    for i = 0, n do
        FB.put(math.floor(x0 + dx * i / n + 0.5), math.floor(y0 + dy * i / n + 0.5), ch, fgc)
    end
end

function FB.flush(mon)
    for r = 1, FB.H do
        local ch, fg, bg = table.concat(FB.ch[r]), table.concat(FB.fg[r]), table.concat(FB.bg[r])
        local line = ch .. "\1" .. fg .. "\1" .. bg
        if FB.prev[r] ~= line then
            FB.prev[r] = line
            mon.setCursorPos(1, r)
            mon.blit(ch, fg, bg)
        end
    end
end

-- ============================================================================
-- THE HUD
-- ============================================================================

local HUD = { L = nil }

-- Row budget, bottom-anchored, because the bottom of this monitor is the part of it nearest
-- the pilot's eye line. Reading UP from the bottom edge:
--
--   H    banner -- the warning, when there is one. NOTHING THAT MATTERS LIVES HERE, because
--        a warning covers the whole row: when the system is quiet this is an ornamental
--        frame rule, and losing it under a PULL UP costs nothing.
--   H-1  primary strip -- speed, height, RA, sink. Never covered, never dropped.
--   H-2  heading INDEX -- the fixed caret and the moving bugs
--   H-3  heading SCALE -- the tape it points at
--   H-4  autopilot / engine annunciators
--   ...  attitude field
--   1    route
--
-- The self-status line and the GPWS armed/inhibited line are NOT here -- they live on the
-- computer's own screen. Neither is flown by; both are read on the ground or when something
-- has already gone wrong, and a head-up display is not the place to keep housekeeping.
function HUD.layout(W, H)
    local L = { W = W, H = H, cx = math.floor(W / 2) + 1 }
    L.banner  = H
    L.primary = H - 1
    local used = 2
    -- The heading indicator is two rows or none: an index caret with no scale under it
    -- points at nothing.
    if H >= 5 then L.hdgIndex, L.hdgScale = H - 2, H - 3 used = 4 end
    if H >= 7 then L.annun = H - 4 used = 5 end
    -- Whatever is left goes to the attitude field, with the route line above it when there is
    -- room for both. Two rows is the field's minimum and it is still worth having -- a bank
    -- scale and a horizon line is an attitude display. Below that the route line takes the
    -- last row instead. Nothing is ever left blank.
    local avail = H - used
    if avail >= 3 then L.route = 1 L.attTop, L.attBot = 2, avail
    elseif avail == 2 then L.attTop, L.attBot = 1, 2
    elseif avail == 1 then L.route = 1 end
    return L
end

-- A point on the pitch ladder, in screen (column, row).
--   theta -- the ladder angle in degrees (0 = the horizon)
--   s     -- how far ALONG the ladder line, in the same degrees-of-arc units
-- The whole ladder is a rigid frame rotated about the aircraft symbol, which is what makes a
-- banked horizon and its rungs stay parallel. Work is done in isotropic units (a character
-- is CELL_ASPECT times taller than it is wide) and converted back at the end.
function HUD.ladderPoint(theta, s)
    local L = HUD.L
    local cx = L.cx
    local cy = (L.attTop + L.attBot) / 2
    local rowsPerDeg = (L.attBot - L.attTop + 1) / HC.FOV_DEG
    local phi = math.rad(ST.bank)
    local k = rowsPerDeg * HC.CELL_ASPECT                    -- isotropic units per degree
    local d = (ST.pitch - theta) * k                          -- perpendicular, + = below
    local tx, ty = math.cos(phi), math.sin(phi)               -- along the horizon
    local nx, ny = -ty, tx                                    -- perpendicular, + = down
    local px = d * nx + s * k * tx
    local py = d * ny + s * k * ty
    return cx + px, cy + py / HC.CELL_ASPECT
end

function HUD.drawAttitude()
    local L = HUD.L
    if not L.attTop then return end
    local top, bot = L.attTop, L.attBot
    local rowsPerDeg = (bot - top + 1) / HC.FOV_DEG
    -- The top row of the field belongs to the BANK SCALE alone. It is the only bank cue left
    -- when a steep climb or dive has pushed the horizon off the field entirely, so it must
    -- not have ladder rungs running through it.
    FB.clip(top + 1, bot)

    -- HORIZON. Drawn as the theta = 0 rung, from well off one edge to well off the other so
    -- it stays a full-width line at any bank; FB.put clips it.
    if HC.SHOW_HORIZON then
        local span = (L.W + (bot - top + 1) * HC.CELL_ASPECT) / (rowsPerDeg * HC.CELL_ASPECT)
        local x0, y0 = HUD.ladderPoint(0, -span)
        local x1, y1 = HUD.ladderPoint(0,  span)
        -- '=', not '-': the positive ladder rungs are drawn with '-', and on a steeply banked
        -- display the two run through each other. The horizon is the one line that has to be
        -- identifiable at a glance.
        FB.line(x0, y0, x1, y1, "=", colours.lime)
    end

    -- PITCH LADDER. Solid rungs above the horizon, dashed below -- the standard reading, and
    -- the only cue that says which side of the horizon a rung belongs to when the aircraft is
    -- pitched far enough that the horizon itself has left the field.
    -- ...and suppressed altogether when the field is too short to separate the rungs. On a
    -- 4-row field a 5-degree ladder puts consecutive rungs less than a row apart, so they
    -- quantise onto the same cells and the "ladder" becomes a smear that reads as a second
    -- horizon. A horizon and a bank scale with no ladder is a poorer instrument; a horizon
    -- and a bank scale with a lying ladder is a worse one.
    if HC.SHOW_LADDER and rowsPerDeg * HC.LADDER_STEP >= 1 then
        local step = HC.LADDER_STEP
        -- Capped at the zenith: ST.pitch comes from an asin and so saturates at +-90, and a
        -- rung past vertical is not a pitch angle any more.
        local reach = math.min(90,
            math.ceil((HC.FOV_DEG / 2 + math.abs(ST.pitch)) / step) * step)
        for theta = -reach, reach, step do
            if theta ~= 0 then
                local ch = (theta > 0) and "-" or "."
                local gap, len = 4, 8
                for _, sgn in ipairs({ -1, 1 }) do
                    local ax, ay = HUD.ladderPoint(theta, sgn * gap / (rowsPerDeg * HC.CELL_ASPECT))
                    local bx, by = HUD.ladderPoint(theta, sgn * (gap + len) / (rowsPerDeg * HC.CELL_ASPECT))
                    if (ay >= top - 1 and ay <= bot + 1) or (by >= top - 1 and by <= bot + 1) then
                        FB.line(ax, ay, bx, by, ch, colours.green)
                    end
                end
                local lx, ly = HUD.ladderPoint(theta, (gap + len + 3) / (rowsPerDeg * HC.CELL_ASPECT))
                if ly >= top and ly <= bot then
                    FB.text(math.floor(lx), math.floor(ly + 0.5),
                        string.format("%d", math.abs(theta)), colours.green)
                end
            end
        end
    end

    -- FLIGHT PATH VECTOR -- where the aircraft is actually going, as opposed to where it is
    -- pointing. On a VTOL those two routinely part company by 90 degrees (it holds a heading
    -- and translates sideways), so the symbol sitting well off to one side of the boresight
    -- is the aircraft telling the truth, not the display drifting.
    if HC.SHOW_FPV and ST.gs > 0.5 then
        local gamma = math.deg(atan2(ST.vy, ST.gs))
        local drift = wrap180(ST.trk - ST.hdg)
        local fx, fy = HUD.ladderPoint(gamma, drift)
        local caged = false
        if fx < 2 then fx, caged = 2, true elseif fx > L.W - 1 then fx, caged = L.W - 1, true end
        if fy < top + 1 then fy, caged = top + 1, true
        elseif fy > bot then fy, caged = bot, true end
        FB.put(fx, fy, caged and "x" or "o", caged and colours.orange or colours.lime)
        if not caged then
            FB.put(fx - 2, fy, "-", colours.lime) FB.put(fx - 1, fy, "-", colours.lime)
            FB.put(fx + 1, fy, "-", colours.lime) FB.put(fx + 2, fy, "-", colours.lime)
        end
    end

    -- BOresight -- the fixed aircraft reference. Everything above moves around this. Never on
    -- the field's top row, which belongs to the bank scale: on a two-row field the midpoint
    -- rounds onto it and the scale, drawn later, would paint the aircraft symbol out.
    local cy = math.max(top + 1, math.floor((top + bot) / 2))
    FB.text(L.cx - 5, cy, "---", colours.white)
    FB.put(L.cx - 1, cy, "\\", colours.white)
    FB.put(L.cx,     cy, "v",  colours.white)
    FB.put(L.cx + 1, cy, "/",  colours.white)
    FB.text(L.cx + 3, cy, "---", colours.white)

    -- BANK SCALE on the top edge of the field: a fixed arc of ticks with a pointer that
    -- reads bank ALONE. Pitch never moves it, so bank stays readable in a steep climb or
    -- dive when the horizon itself has slid off the field entirely.
    --
    -- ON A BAND, like the heading indicator and for the same reason: unfilled, it is a row of
    -- green ticks lost among all the other green ticks. The band stops at the ends of the
    -- scale rather than running the width of the display, so its edges ARE the +-75 deg
    -- stops. The artificial horizon deliberately gets no such treatment -- filling that would
    -- turn most of the screen into a lamp and put the symbology inside it.
    local scaleRow = top
    FB.clip(top, bot)          -- the scale's own row, which the ladder was kept out of
    -- Columns per degree of bank, sized so +-75 deg always fits the monitor being flown --
    -- a scale whose ends are off the screen is a scale with no ends.
    local bk = math.min(0.6, (L.W / 2 - 2) / 75)
    local half = math.floor(75 * bk + 0.5)
    for c = L.cx - half, L.cx + half do FB.put(c, scaleRow, " ", colours.green, colours.blue) end
    for _, d in ipairs({ -60, -45, -30, -20, -10, 10, 20, 30, 45, 60 }) do
        FB.put(L.cx + math.floor(d * bk + 0.5), scaleRow,
            (d % 30 == 0) and "|" or "'", colours.lime, colours.blue)
    end
    FB.put(L.cx, scaleRow, "^", colours.white, colours.blue)
    -- The pointer is a LIT BLOCK, not a character: it is the one thing on this row that
    -- moves, and it changes colour rather than shape as the bank passes the caution.
    local pc = L.cx + math.floor(clamp(ST.bank, -75, 75) * bk + 0.5)
    local b = math.abs(ST.bank)
    FB.put(pc, scaleRow, " ", colours.black,
        (b >= GP.MODE6.BANK_WARNING) and colours.red
        or (b >= GP.MODE6.BANK_CAUTION) and colours.orange or colours.lime)
    if ST.inverted then FB.text(L.cx - 1, scaleRow + 1, "INV", colours.orange) end
    FB.clip(nil, nil)
end

-- THE HEADING INDICATOR -- two rows, on a background band.
--
-- Two rows because one could not carry both the scale and the index without them fighting
-- over the same cells: the caret used to land on a label and turn 090 into "0^". Split, the
-- scale row is nothing but numbers and the index row is nothing but the caret and the bugs,
-- and each is legible on its own.
--
-- EVERY GRADUATION IS A NUMBER. There are no anonymous ticks: on a tape this narrow (this
-- airframe's HUD is 57 columns) an unlabelled tick is a mark the pilot has to count from the
-- nearest label to decode, and there is only ever one label in sight to count from. A number
-- every ten degrees is read, not counted. Thirties are drawn brighter so the eye still has a
-- coarse structure to land on.
--
-- On a BAND because a row of small marks on black is almost invisible against a HUD that is
-- already all small green marks on black. The band is what makes it read as an instrument
-- rather than as more symbology, and it is one of only two places on this display where a
-- filled background is worth its brightness -- unlike the artificial horizon, which would
-- become a lamp.
function HUD.drawHeading()
    local L = HUD.L
    if not L.hdgScale then return end
    local rs, ri = L.hdgScale, L.hdgIndex
    local per = HC.HDG_PER_COL
    FB.fill(rs, colours.green, colours.blue)     -- scale band
    FB.fill(ri, colours.green, colours.brown)    -- index band, a shade up from the scale

    for c = 1, L.W do
        local h = (ST.hdg + (c - L.cx) * per) % 360
        local prev = (ST.hdg + (c - 1 - L.cx) * per) % 360
        -- A graduation lands on the column where the tape crosses a multiple of 10 degrees.
        if math.floor(h / 10) ~= math.floor(prev / 10) then
            local t = math.floor(h / 10) * 10
            -- Centred on its own column, and drawn ONLY IF IT FITS WHOLE. A number sliced by
            -- the edge of the tape is the "0^" problem again in a different place: half a
            -- heading is not a smaller heading, it is a wrong one.
            if c - 1 >= 1 and c <= L.W then
                FB.text(c - 1, rs, string.format("%02d", (t / 10) % 36),
                    (t % 30 == 0) and colours.white or colours.lime, colours.blue)
            end
        end
    end

    -- The index row. Bugs are CAGED to the edge rather than allowed off it -- a bug that has
    -- vanished tells the pilot nothing, and which edge it is pinned to is exactly the useful
    -- part. Painted in increasing order of importance, the fixed caret last.
    local function bug(deg, ch, col)
        local c = L.cx + math.floor(wrap180(deg - ST.hdg) / per + 0.5)
        if c < 1 then FB.put(1, ri, "<", col, colours.brown)
        elseif c > L.W then FB.put(L.W, ri, ">", col, colours.brown)
        else FB.put(c, ri, ch, col, colours.brown) end
    end
    local tgt = LINK.get("target", STALE.APNAV)
    if tgt and tgt.x and tgt.y then
        -- the FMS names its target's Z as `y` -- this is the plan-view frame, not altitude
        bug(bearingTo(tgt.x - ST.x, tgt.y - ST.z), "W", colours.cyan)
    end
    if ST.gs > 0.5 then bug(ST.trk, "T", colours.lime) end
    -- The fixed index, drawn as a lit block so it is found without being read. It points UP
    -- at the scale immediately above it.
    FB.put(L.cx, ri, "^", colours.black, colours.white)
end

-- Build the primary strip as a list of segments and drop them off the right-hand end until
-- it fits: on a narrow monitor the pilot keeps speed, height and sink rate, and loses the
-- attitude digits, which the picture above is already showing.
function HUD.primarySegments()
    local segs = {}
    local function seg(text, col) segs[#segs + 1] = { text = text, col = col } end

    seg(ST.fixFresh() and string.format("GS %3.0f", ST.gs) or "GS ---",
        ST.fixFresh() and colours.lime or colours.orange)

    local sel = LINK.get("altsel", STALE.ALTAP)
    seg(string.format("ALT %4.0f%s", ST.y, sel and string.format(">%d", sel) or ""),
        colours.lime)

    if ST.ra then
        local src = (ST.datumSrc == "SITE") and (ST.datumName or "SITE") or ST.datumSrc
        seg(string.format("RA %4.0f %s", ST.ra, src),
            (ST.ra < GP.MODE6.DH) and colours.orange or colours.lime)
    else
        seg("RA ---- NODATUM", colours.grey)
    end

    -- The COMMANDED vertical speed is shown only when the VS stick is actually asking for
    -- something. At rest it is six columns of "c+0.0" -- and on a 57-column strip six columns
    -- is the difference between carrying the heading and not.
    local cmd = LINK.get("cmdvs", STALE.MANCON)
    if cmd and math.abs(cmd) < 0.1 then cmd = nil end
    seg(string.format("VS %+4.1f%s", snap0(ST.vs), cmd and string.format(" c%+.1f", cmd) or ""),
        (ST.vs <= -0.1) and colours.orange or colours.lime)

    -- The tail, in the order it may be given up. Heading first because it is the one number
    -- here the tape above cannot state exactly; bank next because it is an alerting
    -- parameter; then track and pitch, which the index row and the ladder already show.
    seg(string.format("HDG %03.0f", ST.hdg), colours.lime)
    -- R/L comes off the same signed number the bank pointer is drawn from, so the two can
    -- never disagree. If it reads backwards in the air, flip CFG.HUD.ROLL_SIGN -- which
    -- flips the horizon with it, and that is the point: they must agree.
    seg(string.format("BNK %2.0f%s", math.abs(ST.bank), (ST.bank >= 0) and "R" or "L"),
        (math.abs(ST.bank) >= GP.MODE6.BANK_CAUTION) and colours.orange or colours.lightGrey)
    seg(string.format("TRK %03.0f", ST.trk), colours.lightGrey)
    seg(string.format("PIT %+3.0f", snap0(ST.pitch)), colours.lightGrey)
    return segs
end

function HUD.drawPrimary()
    local L = HUD.L
    local segs = HUD.primarySegments()
    local sep = "  "
    while #segs > 1 do
        local w = 0
        for i, s in ipairs(segs) do w = w + #s.text + (i > 1 and #sep or 0) end
        if w <= L.W - 2 then break end
        table.remove(segs)
    end
    local w = 0
    for i, s in ipairs(segs) do w = w + #s.text + (i > 1 and #sep or 0) end
    local c = math.max(1, math.floor((L.W - w) / 2) + 1)
    for i, s in ipairs(segs) do
        if i > 1 then FB.text(c, L.primary, sep, colours.grey) c = c + #sep end
        FB.text(c, L.primary, s.text, s.col)
        c = c + #s.text
    end
end

function HUD.drawAnnunciators()
    local L = HUD.L
    if not L.annun then return end
    local r = L.annun
    local items = {}
    local function ann(label, on, onCol)
        items[#items + 1] = { t = label, c = on and (onCol or colours.lime) or colours.grey }
    end

    ann("AP-NAV", LINK.get("apnav", STALE.APSWITCH) == true)
    ann("AP-ALT", LINK.get("apalt", STALE.APSWITCH) == true)

    local al = LINK.get("autoland", STALE.AUTOLAND)
    if al and al.cmd then
        items[#items + 1] = { t = "LAND " .. tostring(al.cmd):upper(), c = colours.cyan }
    else
        items[#items + 1] = { t = "LAND ----", c = colours.grey }
    end

    local e = LINK.get("engine", STALE.ENGINE)
    if e then
        -- Named E-ALT / E-ATT, not ALT / ATT: these are the ENGINE switches on #11, and the
        -- two annunciators immediately to their left are the AUTOPILOT ones. Two of anything
        -- called "ALT" on one row is how a pilot reads the wrong one.
        ann("MSTR", e.master and true or false)
        ann("E-ALT", e.altitude and true or false)
        ann("E-ATT", e.attitude and true or false)
        ann("XLIFT", e.extra_lift and true or false, colours.cyan)
    else
        items[#items + 1] = { t = "ENG ?", c = colours.orange }
    end

    ann("WOW", ST.onGround, colours.cyan)

    local tgt = LINK.get("target", STALE.APNAV)
    if tgt and tgt.phase then
        items[#items + 1] = { t = tostring(tgt.phase):upper(), c = colours.cyan }
    end

    -- Trimmed from the right on a narrow monitor, same as the primary strip: the two
    -- autopilot annunciators and the landing phase are what a pilot needs at a glance, and a
    -- half-drawn "E-AT" at the edge is just noise pretending to be an indication.
    local function width()
        local w = 0
        for i, s in ipairs(items) do w = w + #s.t + (i > 1 and 2 or 0) end
        return w
    end
    while #items > 1 and width() > L.W do table.remove(items) end
    local c = math.max(1, math.floor((L.W - width()) / 2) + 1)
    for i, s in ipairs(items) do
        if i > 1 then c = c + 2 end
        FB.text(c, r, s.t, s.c)
        c = c + #s.t
    end
end

function HUD.drawRoute()
    local L = HUD.L
    if not L.route then return end
    local r = L.route
    local tgt = LINK.get("target", STALE.APNAV)
    if not tgt then
        FB.text(2, r, "NAV  no active target", colours.grey)
        return
    end
    local parts = { string.format("TO %s", tostring(tgt.name or tgt.type or "?")) }
    if tgt.x and tgt.y then
        local d = dist2d(ST.x, ST.z, tgt.x, tgt.y)
        local brg = bearingTo(tgt.x - ST.x, tgt.y - ST.z)
        parts[#parts + 1] = string.format("BRG %03.0f", brg)
        parts[#parts + 1] = (d >= 1000) and string.format("DST %.1fkm", d / 1000)
                                         or string.format("DST %.0fm", d)
        if ST.gs > 1 then
            local ete = d / ST.gs
            parts[#parts + 1] = string.format("ETE %d:%02d", math.floor(ete / 60), math.floor(ete % 60))
        end
    end
    if tgt.cruiseCap then parts[#parts + 1] = string.format("CMD %.0f", tgt.cruiseCap) end
    if tgt.phase then parts[#parts + 1] = tostring(tgt.phase) end
    -- Trimmed from the right like the other composed rows: on 57 columns a long distance
    -- ("DST 706.3km") is enough to push the phase off the end, and a row that runs off the
    -- glass loses whatever happens to be last rather than whatever matters least.
    while #parts > 1 and #table.concat(parts, "  ") > L.W - 2 do table.remove(parts) end
    FB.text(2, r, table.concat(parts, "  "), colours.lightGrey)
end

-- THE BANNER -- the bottom row, and the one the pilot's eye is already nearest.
--
-- When something is up it is the WHOLE row in reverse video, because the one thing this line
-- must never be is easy to miss.
--
-- When nothing is up it is an ORNAMENTAL FRAME RULE and nothing else. That is deliberate: a
-- warning covers this row completely, so anything informative parked here is information the
-- pilot loses at exactly the moment things are going wrong. The system's own status -- armed,
-- inhibited and why, the datum, the voice pack, the links -- is on the computer's screen,
-- where it can be read without a warning sitting on top of it.
-- THE BANNER -- the warning, and (with the debug flag up) everything about it worth knowing.
--
-- ONE ROW, because that is where the eye already is. A warning fills it; when the system is
-- quiet it is a rule with the callsign on it. With CFG.DEBUG this same row also carries:
--
--   * THE MODE NUMBERS behind the alerts currently showing, in priority order, written
--     exactly as CFG.ALERTS writes them -- 2, 4, FLTA, AP, 1/2. The FIRST is the mode behind
--     the text being displayed; the rest are modes that agreed and were outranked, and are
--     otherwise invisible. The banner shows one alert; three modes are often up.
--   * WHEN QUIET, WHAT WAS JUST SAID and how long ago, in the level colour of the alert it
--     was. The dashed rule is visually nothing like a filled warning bar, so a red recall
--     cannot be mistaken for a live one.
--   * the numbers the terrain modes were looking at, which shed off the right first.
--
-- Segments are composed and centred as one line so the row reads as a sentence. Segment one
-- -- the alert text, or the callsign -- is never shed.
function HUD.bannerNums()
    local k, out = GPWS.lk, {}
    -- req FIRST: it is the number the terrain modes actually alert on, so it is the one
    -- that explains the banner. clr and cls are context.
    if k.req then
        out[#out + 1] = string.format("req%+.1f@%ds", k.req, math.floor(k.reqT or 0))
    end
    if k.worst then
        out[#out + 1] = string.format("clr%d", math.floor(k.worst))
        out[#out + 1] = string.format("cls%.1f", k.closure or 0)
        out[#out + 1] = string.format("lk%ds", math.floor(k.secs or 0))
    end
    -- Which field is in play, and whether we are actually ON it (the * means the datum is
    -- that field's published elevation rather than the terrain database).
    if ST.siteNear then
        out[#out + 1] = ST.siteNear.name .. ((ST.datumSrc == "SITE") and "*" or "")
    end
    out[#out + 1] = (ST.datumSrc or "--") .. (ST.ra and string.format("%d", math.floor(ST.ra)) or "--")
    -- Raw and estimated vertical rate. When they disagree the airframe is oscillating.
    out[#out + 1] = string.format("vs%+.1f/%+.1f", ST.vs or 0, ST.vsf or 0)
    return out
end

-- Modes showing, priority order, in CFG.ALERTS' own notation and nothing added to it.
function HUD.bannerModes()
    local out = {}
    for _, a in ipairs(ALERTS) do
        if GPWS.showing(a.id) then out[#out + 1] = a.mode or "?" end
    end
    return out
end

function HUD.bannerLine(row, segs, fg, bg)
    local function w()
        local n = 0
        for i = 1, #segs do n = n + #segs[i].s + (i > 1 and 2 or 0) end
        return n
    end
    while #segs > 1 and w() > HUD.L.W do table.remove(segs) end
    local c = math.max(1, math.floor((HUD.L.W - w()) / 2) + 1)
    for i = 1, #segs do
        if i > 1 then c = c + 2 end
        FB.text(c, row, segs[i].s, segs[i].c or fg, bg)
        c = c + #segs[i].s
    end
end

function HUD.drawBanner()
    local L = HUD.L
    -- A REAL alert outranks the self test on the banner as well as in the speaker, and it is
    -- shown WITHOUT the self-test prefix. Labelling a genuine AUTOPILOT DISCONNECTED as a
    -- test merely because a test happened to be running is how a warning gets ignored.
    local a, isTest = GPWS.top(false), false
    if not a and AUDIO.selftest then a, isTest = ALERT[AUDIO.selftestId or ""], true end

    local segs = {}
    if a then
        local bg = (a.level == "warn") and colours.red
                or (a.level == "caut") and colours.orange or colours.lime
        local txt = a.text
        if isTest then txt = "SELF TEST -- " .. txt bg = colours.cyan end
        segs[1] = { s = txt }
        if GPWS.debug then
            local m = HUD.bannerModes()
            if #m > 0 then segs[#segs + 1] = { s = "[" .. table.concat(m, " ") .. "]" } end
            for _, n in ipairs(HUD.bannerNums()) do segs[#segs + 1] = { s = n } end
        end
        FB.fill(L.banner, colours.black, bg)
        HUD.bannerLine(L.banner, segs, colours.black, bg)
        return
    end

    -- The quiet rule. Dim, and carrying only the aircraft's own name -- which the pilot
    -- knows -- until the debug flag asks it for the rest.
    FB.fill(L.banner, colours.green, colours.black)
    for c = 1, L.W do FB.put(c, L.banner, "-", colours.green) end
    segs[1] = { s = "[ " .. SHIP.CALLSIGN .. " ]", c = colours.lightGrey }
    if GPWS.debug and CFG.DEBUG.WHEN_QUIET then
        -- The inhibit goes BEFORE the recall, because segments shed from the right and the
        -- two answer different questions. "Why is it not firing" beats "what did it say"
        -- every time -- an inhibited GPWS is the case where the pilot most needs telling.
        if GPWS.inhibit then
            segs[#segs + 1] = { s = "INH:" .. GPWS.inhibit, c = colours.grey }
        end
        if GPWS.last then
            segs[#segs + 1] = {
                s = string.format("last %s [%s] %ds", GPWS.last.text, GPWS.last.mode,
                    math.floor(os.clock() - GPWS.last.t)),
                c = (GPWS.last.level == "warn") and colours.red
                    or (GPWS.last.level == "caut") and colours.orange or colours.lime,
            }
        end
        for _, n in ipairs(HUD.bannerNums()) do
            segs[#segs + 1] = { s = n, c = colours.cyan }
        end
    end
    HUD.bannerLine(L.banner, segs, colours.lightGrey, colours.black)
end

function HUD.draw()
    local W, H = HW.mon.getSize()
    FB.size(W, H)
    HUD.L = HUD.layout(W, H)
    FB.clear(colours.lime, colours.black)
    if ST.fixFresh() then
        HUD.drawAttitude()
        HUD.drawHeading()
    elseif HUD.L.attTop then
        FB.text(math.max(1, HUD.L.cx - 8), math.floor((HUD.L.attTop + HUD.L.attBot) / 2),
            "NO SUBLEVEL FIX", colours.red)
    end
    HUD.drawRoute()
    HUD.drawAnnunciators()
    HUD.drawPrimary()
    HUD.drawBanner()
    FB.flush(HW.mon)
end

function HUD.loop()
    if not HW.mon then while true do sleep(1) end end
    HW.mon.setBackgroundColour(colours.black)
    HW.mon.clear()
    while true do
        local ok, err = pcall(HUD.draw)
        if not ok then HW.faults.hud = tostring(err) end
        sleep(HC.REDRAW)
    end
end

-- ============================================================================
-- THE COMPUTER'S OWN SCREEN -- a maintenance page, not an instrument. What is wired, what is
-- talking, what the GPWS thinks and what the voice pack looks like.
-- ============================================================================

local TERM = {}

function TERM.line(r, s, col)
    term.setCursorPos(1, r)
    term.setTextColour(col or colours.white)
    term.clearLine()
    term.write(s)
end

function TERM.draw()
    local W, H = term.getSize()
    term.setBackgroundColour(colours.black)
    TERM.line(1, SHIP.NAME .. " HUD/GPWS  #" .. os.getComputerID(), colours.yellow)
    TERM.line(2, string.format("MON %s  SPK %s  DISK %s",
        HW.monName or "none", HW.speaker and "ok" or "none", AUDIO.dir or "none"),
        (HW.mon and HW.speaker and AUDIO.dir) and colours.lime or colours.orange)
    TERM.line(3, string.format("VOICE %s  %d phrase(s)%s", VOICE.LANG, AUDIO.clips,
        AUDIO.fault and ("  " .. AUDIO.fault) or ""),
        AUDIO.fault and colours.orange or colours.lime)
    TERM.line(4, TERRAIN.why() .. "   " .. (NDU.status()), colours.lightGrey)
    if ST.fixFresh() then
        TERM.line(5, string.format("POS %.0f %.0f ALT %.0f  GS %.1f VS %+.1f",
            ST.x, ST.z, ST.y, ST.gs, snap0(ST.vs)), colours.white)
        TERM.line(6, string.format("HDG %03.0f TRK %03.0f  PIT %+.0f BNK %+.0f%s",
            ST.hdg, ST.trk, snap0(ST.pitch), snap0(ST.bank),
            ST.inverted and " INV" or ""), colours.white)
    else
        TERM.line(5, "NO SUBLEVEL FIX", colours.red)
        TERM.line(6, "", colours.white)
    end
    TERM.line(7, ST.ra and string.format("RA %.1f m  datum %s %s (%.0f)", ST.ra,
        ST.datumSrc, ST.datumName or "", ST.datum) or "RA unavailable - no datum",
        ST.ra and colours.lime or colours.grey)
    -- THE ARMED/INHIBITED LINE LIVES HERE, not on the HUD. It answers "why did it not warn
    -- me", which is a question asked on the ground or after the fact -- and on the HUD it sat
    -- on the bottom row, where a warning would have covered it precisely when it mattered.
    -- So it says the whole thing here instead: armed or not, which modes that leaves live,
    -- and what is being annunciated right now.
    local a = GPWS.top(false)
    local st
    if not GPWS.armed then
        st = "GPWS INHIBITED - " .. (GPWS.inhibit or "?")
    elseif ST.ra then
        st = "GPWS ARMED  modes 1/3/4/6" .. (GPWS.m4Armed and "" or " (4 disarmed: takeoff)")
    else
        st = "GPWS ARMED  bank angle + AP only, no height datum"
    end
    if a then st = st .. "   >> " .. a.text end
    TERM.line(8, st, a and colours.red or (GPWS.armed and colours.lime or colours.grey))
    TERM.line(9, string.format("WOW %s  APNAV %s  APALT %s  LAND %s",
        tostring(ST.onGround),
        tostring(LINK.get("apnav", STALE.APSWITCH)),
        tostring(LINK.get("apalt", STALE.APSWITCH)),
        (LINK.get("autoland", STALE.AUTOLAND) or {}).cmd or "-"), colours.lightGrey)
    if H >= 11 then
        TERM.line(H, "[" .. CFG.SELFTEST.ON_KEY:upper() .. "] self test"
            .. (AUDIO.selftest and "  RUNNING" or ""), colours.cyan)
    end
end

function TERM.loop()
    term.clear()
    while true do
        pcall(TERM.draw)
        sleep(HC.TERM_REDRAW)
    end
end

-- ============================================================================
-- INPUT -- the self-test button, and nothing else. There is no control on this computer
-- because there is nothing here to control.
-- ============================================================================

local function inputLoop()
    while true do
        local ev, a = os.pullEvent()
        if ev == "key" and CFG.SELFTEST.ON_KEY
                and a == keys[CFG.SELFTEST.ON_KEY] then
            AUDIO.selftest = true
        elseif ev == "key" and CFG.DEBUG.ON_KEY and a == keys[CFG.DEBUG.ON_KEY] then
            GPWS.debug = not GPWS.debug
        elseif (ev == "monitor_touch" or ev == "mouse_click") and CFG.SELFTEST.ON_TOUCH then
            AUDIO.selftest = true
        elseif ev == "disk" or ev == "disk_eject" then
            AUDIO.scan()
        elseif ev == "peripheral" or ev == "peripheral_detach" then
            HW.speaker = peripheral.find("speaker")
            AUDIO.scan()
        end
    end
end

-- ============================================================================
-- THE CORE TICK -- state, datum, envelopes. 20 Hz, the rate the flight system runs at.
-- ============================================================================

local function coreLoop()
    while true do
        ST.read()
        DATUM.update()
        local ok, err = pcall(GPWS.tick)
        if not ok then HW.faults.gpws = tostring(err) end
        sleep(0.05)
    end
end

-- ============================================================================

term.clear()
term.setCursorPos(1, 1)
print(SHIP.NAME .. " HUD/GPWS starting...")
print("  monitor: " .. (HW.monName or "NONE"))
print("  speaker: " .. (HW.speaker and "ok" or "NONE"))
print("  voice  : " .. (AUDIO.dir and (AUDIO.dir .. " (" .. VOICE.LANG .. ", "
    .. AUDIO.clips .. " phrases)") or ("NONE - " .. tostring(AUDIO.fault))))
print("  terrain: " .. TERRAIN.why())

parallel.waitForAny(coreLoop, LINK.loop, NDU.pullLoop, TERRAIN.loop, AUDIO.loop,
    HUD.loop, TERM.loop, inputLoop)
