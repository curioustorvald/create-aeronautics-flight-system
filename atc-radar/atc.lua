-- TMU ATC -- RADAR DISPLAY
--
-- A surveillance scope for the fleet. It watches `atc_broadcast`, the one-way channel every
-- aircraft's responder.lua transmits its position on at 20 Hz, and paints what it hears onto
-- a monitor: a plan view with the aerodromes, runways, helipads and airways of the shared
-- navigation database underneath, and one traffic strip per contact down the right-hand side.
--
-- IT RUNS IN TWO MODES, set by CFG.STATION.MODE, and they are one program because they are one
-- picture drawn about a different centre:
--
--   * STATIC  -- a tower. The scope is centred on the coordinates in atcconf.lua and oriented
--                to the field (#44, TMU).
--   * DYNAMIC -- airborne. The scope is centred on THIS aircraft, read off sublevel every
--                frame, and normally turns with it so the nose is up (#64, Aer Togekiss's
--                AWACS). Own-ship sits at the middle as a nose-up blip, and its own responder
--                is filtered out of the traffic -- see CFG.STATION.OWN_CALLSIGN, without which
--                the aircraft would plot itself as a contact permanently on top of itself.
--
-- IT DOES NOT CONTROL ANYTHING. There is no clearance, no vectoring, no separation logic and
-- nothing is ever transmitted to an aircraft -- the station is receive-only on the traffic
-- channel and the only thing it ever sends is a nav-data acknowledgement to #24. Pilots fly
-- themselves; this tells whoever is in the tower what is out there.
--
-- The map underneath comes from the same points.db / course.db / landing.db an aircraft
-- carries, kept current by subscribing to the nav data server (#24) exactly as an FMS does.
-- So a runway added to the fleet database appears on this scope within a broadcast interval,
-- with no copying by hand.
--
-- Everything tunable is in atcconf.lua beside this file. See its header.

-- ============================================================================
-- CONFIGURATION
-- ============================================================================
-- Scoped inside the do-block so the load temporaries do not sit in top-level registers,
-- the same discipline autopilot.lua and uac.lua keep (see apconf.lua's note on Cobalt's
-- 200-local ceiling). This file is nowhere near that limit, but the house style is what
-- makes a later addition safe.
local CFG
local ST, LINKS, TRACKCFG, MAPCFG, PANELCFG
local POINTS_FILE, COURSE_FILE, LANDING_FILE
do
    local ok, res = pcall(require, "atcconf")
    if not ok then
        error("Cannot load atcconf.lua (the station configuration): " .. tostring(res)
            .. "\nIt must sit beside atc.lua and end with 'return CFG'.", 0)
    end
    if type(res) ~= "table" then
        error("atcconf.lua did not return the CFG table (got " .. type(res) .. ").", 0)
    end
    -- Canary check: catch a truncated or half-saved config at BOOT, naming what is missing,
    -- rather than as a nil-arithmetic crash once traffic shows up.
    for _, k in ipairs({ "STATION", "LINKS", "POINTS_FILE", "COURSE_FILE", "LANDING_FILE",
            "NAVDATA", "TRACK", "MAP", "PANEL", "REDRAW", "TERM_REDRAW", "TEXT_SCALE",
            "PALETTE" }) do
        if res[k] == nil then
            error("atcconf.lua is missing CFG." .. k .. " -- the file looks incomplete.", 0)
        end
    end
    -- STATION is checked separately below: which of its knobs are required depends on the
    -- mode, and demanding a tower's coordinates from an aeroplane would be nonsense.
    for t, ks in pairs({ STATION = {"NAME", "MODE"},
            LINKS = {"ATC", "NAVDATA"}, NAVDATA = {"ENABLE", "STALE_AFTER", "TMP"},
            TRACK = {"COAST_AFTER", "DROP_AFTER", "LEADER_SEC"},
            MAP = {"RANGES", "DEFAULT_RANGE", "CELL_ASPECT"}, PANEL = {"WIDTH"},
            PALETTE = {"MON"} }) do
        if type(res[t]) ~= "table" then
            error("atcconf.lua's CFG." .. t .. " is not a table.", 0)
        end
        for _, k in ipairs(ks) do
            if res[t][k] == nil then
                error("atcconf.lua's CFG." .. t .. " is missing " .. k .. ".", 0)
            end
        end
    end
    CFG = res
    ST, LINKS, TRACKCFG   = CFG.STATION, CFG.LINKS, CFG.TRACK
    if ST.MODE ~= "static" and ST.MODE ~= "dynamic" then
        error("atcconf.lua's CFG.STATION.MODE is '" .. tostring(ST.MODE) .. "'. It must be"
            .. " 'static' (a tower, fixed at X/Z) or 'dynamic' (airborne, position read off"
            .. " sublevel every frame).", 0)
    end
    -- Mode-dependent canary. Same principle as the rest: what this station actually uses must
    -- be present and named if it is not, and what it cannot use must not be sitting in the
    -- file looking as though it does something.
    for _, k in ipairs((ST.MODE == "static")
            and { "X", "Z", "Y", "ORIENTATION" }
            or  { "ORIENTATION", "HEADING_SIGN", "HEADING_OFFSET", "OWN_CALLSIGN",
                  "ALT_OFFSET" }) do
        if ST[k] == nil then
            error("atcconf.lua's CFG.STATION is missing " .. k .. ", which MODE '"
                .. ST.MODE .. "' requires.", 0)
        end
    end
    if ST.MODE == "static" and (ST.ORIENTATION == "heading" or ST.ORIENTATION == "track") then
        error("atcconf.lua sets CFG.STATION.ORIENTATION = '" .. ST.ORIENTATION
            .. "', which follows the aircraft -- a static station has neither.", 0)
    end
    MAPCFG, PANELCFG      = CFG.MAP, CFG.PANEL
    POINTS_FILE, COURSE_FILE, LANDING_FILE =
        CFG.POINTS_FILE, CFG.COURSE_FILE, CFG.LANDING_FILE
end

-- ============================================================================
-- GEOMETRY HELPERS
-- ============================================================================
-- Same frame, same conventions, same sign choices as autopilot.lua -- deliberately. A
-- bearing here and a bearing on the aircraft's HSI have to be the same number or the
-- controller and the pilot are not talking about the same piece of sky.

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

-- Bearings are atan2(dx, dz) in the frame where North = pi; this maps one to a true
-- compass heading (deg, N=0 E=90 S=180 W=270). Identical to the FMS's toCompass.
local function toCompass(rad) return (180 - math.deg(rad)) % 360 end
local function bearingTo(dx, dz) return toCompass(atan2(dx, dz)) end

-- landing.db's n/e/s/w/x (or a raw heading) as a world unit vector. Copied in behaviour
-- from autopilot.lua's resolveDirVec so a runway is drawn along exactly the axis the FMS
-- will land on. 'x' (don't care) yields nil.
local DIR_VECTORS = { n = {0, -1}, e = {1, 0}, s = {0, 1}, w = {-1, 0} }
local function resolveDirVec(query)
    if type(query) ~= "string" then return nil end
    if query == 'x' or query == 'X' then return nil end
    local v = DIR_VECTORS[query:lower()]
    if v then return v end
    if tonumber(query) then
        local r = math.rad(tonumber(query))
        return {math.sin(r), -math.cos(r)}
    end
    return nil
end

-- Pull the yaw out of an orientation quaternion, tolerating either return shape from
-- toEuler -- three numbers or one vector. Copied from responder.lua, which is the program
-- whose heading this one has to agree with: when the AWACS and the responder on the SAME
-- aircraft disagree about which way it is pointing, one of them is lying to the fleet.
local function euler(orientation)
    local a, b, c = orientation:toEuler()
    if type(a) == "number" then return a, b, c end
    return a.x, a.y, a.z
end
local function getYaw(orientation)
    local _, y = euler(orientation)
    return y
end

-- The compass heading drawn at the top of the scope. Same vocabulary as landing.db's
-- direction field, so CFG.STATION.ORIENTATION accepts "north" or "n" or "13.5" alike.
-- "heading" and "track" are not directions, they are instructions to follow one; they
-- resolve per frame in scopeUp() and are worth 0 here.
local FOLLOW = { heading = true, track = true }
local function orientationDeg(o)
    local named = { north = 0, east = 90, south = 180, west = 270, n = 0, e = 90, s = 180, w = 270 }
    if type(o) == "number" then return o % 360 end
    if type(o) == "string" then
        if FOLLOW[o:lower()] then return 0 end
        local n = named[o:lower()]
        if n then return n end
        if tonumber(o) then return tonumber(o) % 360 end
    end
    return 0
end

local function fmtDist(m)
    if m >= 10000 then return string.format("%.0fk", m / 1000)
    elseif m >= 1000 then return string.format("%.1fk", m / 1000)
    else return string.format("%d", math.floor(m + 0.5)) end
end

-- ============================================================================
-- DATA LAYER: points.db / course.db / landing.db
-- ============================================================================
-- READ ONLY. The station never edits navigation data -- #24 owns it and the aircraft carry
-- it; this is a third reader of the same three files, in the same format, with the same
-- parsers. Keeping the parsers character-for-character in step with the FMS's matters: a
-- record the scope shows but the FMS skips is a runway on the screen that nothing can land
-- on, which is a worse lie than not drawing it.

local POINTS, POINT_ORDER = {}, {}      -- name -> {name,x,z,kind}; load order
local LANDING = {}                      -- name -> {type,dir,alt,rwyLen} (goaround shapes too)
local SITES = {}                        -- the render list: everything the scope draws
local AIRWAYS = {}                      -- deduplicated {x1,z1,x2,z2} segments from course.db

local function classify(name) if #name == 5 then return "vor" end return "poi" end

local function loadPoints()
    POINTS, POINT_ORDER = {}, {}
    local f = io.open(POINTS_FILE, "r")
    if not f then return end
    for line in f:lines() do
        local t = line:match("^%s*(.-)%s*$")
        if t ~= "" and t:sub(1, 1) ~= ";" then
            local name, xs, zs = t:match("^(%a+)=(%-?%d+),(%-?%d+)$")
            if name then
                POINTS[name] = { name = name, x = tonumber(xs), z = tonumber(zs),
                                 kind = classify(name) }
                table.insert(POINT_ORDER, name)
            end
        end
    end
    f:close()
end

-- Only the SITE shape (NAME=type,dir,alt,len) is read here. The goaround/appr shapes in the
-- same file are procedures, not places -- they have no point of their own to draw at, and
-- the FMS's isLandingSite excludes them for the same reason. Matching that prefix test here
-- keeps a third record shape from quietly becoming an aerodrome on the scope.
local function loadLanding()
    LANDING = {}
    local f = io.open(LANDING_FILE, "r")
    if not f then return end
    for line in f:lines() do
        local t = line:match("^%s*(.-)%s*$")
        if t ~= "" and t:sub(1, 1) ~= ";" then
            local name, ty, dir, alt, rwy =
                t:match("^(%a+)=(%a+),([%a%.%d]+),(%-?%d+),(%-?%d+)$")
            if name and ty:lower():sub(1, 8) ~= "goaround" then
                LANDING[name] = { type = ty:lower(), dir = dir:lower(),
                                  alt = tonumber(alt), rwyLen = tonumber(rwy) }
            end
        end
    end
    f:close()
end

-- Airways: every consecutive pair in every route, deduplicated so the trunk legs that a
-- dozen routes share are drawn once rather than a dozen times. Direction is dropped -- an
-- airway on a scope is a line, and TMU-WAX and WAX-TMU are the same line.
local function loadAirways()
    AIRWAYS = {}
    local seen = {}
    local f = io.open(COURSE_FILE, "r")
    if not f then return end
    for line in f:lines() do
        local t = line:match("^%s*(.-)%s*$")
        if t ~= "" and t:sub(1, 1) ~= ";" then
            local _, rest = t:match("^([%w%-]+)=(.+)$")
            if rest then
                local prev = nil
                for p in rest:gmatch("[^,]+") do
                    local cur = POINTS[p]
                    if prev and cur then
                        local a, b = prev.name, cur.name
                        local key = (a < b) and (a .. "|" .. b) or (b .. "|" .. a)
                        if not seen[key] then
                            seen[key] = true
                            table.insert(AIRWAYS, { x1 = prev.x, z1 = prev.z,
                                                    x2 = cur.x,  z2 = cur.z })
                        end
                    end
                    if cur then prev = cur end
                end
            end
        end
    end
    f:close()
end

-- One flat list of things to paint, built once per database load rather than per frame:
-- the scope repaints four times a second and there are several hundred points.
local function buildSites()
    SITES = {}
    for _, name in ipairs(POINT_ORDER) do
        local p = POINTS[name]
        local l = LANDING[name]
        local s = { name = name, x = p.x, z = p.z, kind = p.kind }
        if l then
            s.alt = l.alt
            if l.type == "rwy" then
                local v = resolveDirVec(l.dir)
                -- The point in points.db is the THRESHOLD and the strip runs from it along
                -- the landing direction for rwyLen metres (autopilot.lua's LND.rwyPrep reads
                -- it the same way). A runway with dir 'x' has no axis to draw, so it falls
                -- back to a pad symbol.
                if v then
                    s.kind, s.ux, s.uz, s.len = "rwy", v[1], v[2], l.rwyLen or 0
                else
                    s.kind = "heli"
                end
            elseif l.type == "heli" then
                s.kind = "heli"
            end
        end
        table.insert(SITES, s)
    end
end

local function loadAll()
    loadPoints(); loadLanding(); loadAirways(); buildSites()
end

-- ============================================================================
-- THE STATION FIX
-- ============================================================================
-- Where the scope is centred, and -- airborne -- which way it is pointing and how fast it is
-- going. A tower knows because it was told; an aircraft asks sublevel every frame.

local DYN = (ST.MODE == "dynamic")
local FIX = {
    x = 0, z = 0, y = 0,
    have = false,           -- has there EVER been a fix
    at = 0,                 -- os.clock() of the last good one
    hdg = 0, trk = 0, gs = 0, vs = 0, haveHdg = false,
}
if not DYN then
    FIX.x, FIX.y, FIX.z, FIX.have = ST.X, ST.Y, ST.Z, true
end

-- A DROPPED fix is not the same as never having had one, and the difference is exactly the
-- difference between "the aeroplane is somewhere" and "the aeroplane is at 0,0". One bad
-- sublevel call must not blank the map: the last known position keeps the picture, the header
-- says how old it is, and only a station that has never had a fix at all draws nothing.
local function updateFix()
    if not DYN then return end
    if not sublevel then return end
    local ok, pose = pcall(sublevel.getLogicalPose)
    if not (ok and pose and pose.position) then return end
    -- ALTITUDE DATUM (2026-08-29). The pose sits ST.ALT_OFFSET metres above the bottom of
    -- the hull, so subtract it here and the scope's own ALT readout is the same number this
    -- airframe's responder puts on atc_broadcast, the same number its PFD shows the pilot,
    -- and the same datum the traffic around it is reporting in. A static station has no hull
    -- and so has no offset -- ST.Y above is a surveyed tower elevation already.
    FIX.x, FIX.y, FIX.z = pose.position.x, pose.position.y - ST.ALT_OFFSET, pose.position.z
    FIX.have, FIX.at = true, os.clock()
    -- The SAME sign and offset convention as this airframe's responder and FMS. It is a knob
    -- and not a constant because "the ship's logical forward" is not the same as "the way the
    -- thrusters point" on every hull -- Aer Togekiss's is 180 degrees round.
    local okh, hdg = pcall(function()
        return toCompass(ST.HEADING_SIGN * getYaw(pose.orientation) + ST.HEADING_OFFSET)
    end)
    if okh and hdg then FIX.hdg, FIX.haveHdg = hdg, true end
    local okv, v = pcall(sublevel.getLinearVelocity)
    if okv and v then
        FIX.gs, FIX.vs = math.sqrt(v.x * v.x + v.z * v.z), v.y
        -- Track, not heading, when the aircraft is actually going somewhere. On a VTOL these
        -- part company routinely -- it holds a heading and translates sideways -- and a scope
        -- set to track-up that quietly used heading would be lying about where "ahead" is.
        FIX.trk = (FIX.gs > 0.5) and bearingTo(v.x, v.z) or FIX.hdg
    end
end

-- Seconds since the last good fix; 0 for a tower, which cannot lose one.
local function fixAge() if not DYN then return 0 end return os.clock() - FIX.at end

-- ============================================================================
-- TARGETS
-- ============================================================================
-- One entry per responder heard from, keyed by the sending computer's id rather than by
-- callsign: the id is what actually identifies a transmitter, and two aircraft cloned from
-- the same responder.lua with the callsign line left unedited is a real and easy mistake --
-- keying on callsign would silently merge them into one blip that teleports.

local TGT = {
    list = {},      -- id -> target
    sel  = nil,     -- id of the selected target, or nil
}

function TGT.note(id, m)
    local t = TGT.list[id]
    if not t then
        t = { id = id, trail = {}, nextTrail = 0, first = os.clock() }
        TGT.list[id] = t
    end
    -- Everything below the position is taken defensively. atc_broadcast is a shared,
    -- unauthenticated channel that anything on the network can transmit on, and one report
    -- carrying a string where a number belongs would otherwise throw inside the draw and
    -- leave the scope stuck on an error message -- with the aeroplanes still out there.
    local function num(v, d) return (type(v) == "number") and v or d end
    local function str(v, d) return (type(v) == "string") and v or d end
    t.at   = os.clock()
    t.x, t.y, t.z = m.x, m.y, m.z
    t.gs   = num(m.gs, 0)
    t.vs   = num(m.vs, 0)
    t.ys   = num(m.ys, 0)
    t.hdg  = num(m.hdg, 0) % 360
    -- Ground TRACK, not heading. A responder that sends its velocity vector gets a leader
    -- line along the path the aircraft is actually taking; one that does not falls back to
    -- where the nose points. On a VTOL those are routinely 90 degrees apart -- it will hold
    -- a heading and translate sideways -- so the distinction is not academic here.
    if type(m.vx) == "number" and type(m.vz) == "number" then
        t.gs = math.sqrt(m.vx * m.vx + m.vz * m.vz)
        t.trk = (t.gs > 0.5) and bearingTo(m.vx, m.vz) or t.hdg
        t.haveTrk = true
    else
        t.trk, t.haveTrk = t.hdg, false
    end
    t.callsign = str(m.callsign, "#" .. id)
    t.csg      = str(m.callsign_short, t.callsign:sub(1, 3))
    t.ifr      = m.is_ifr and true or false
    t.tcas     = str(m.tcas_status, nil)
    if os.clock() >= t.nextTrail then
        t.nextTrail = os.clock() + TRACKCFG.TRAIL_EVERY
        table.insert(t.trail, { x = m.x, z = m.z })
        while #t.trail > TRACKCFG.TRAIL_LEN do table.remove(t.trail, 1) end
    end
end

-- Age a target off the scope. Coasting first (dimmed, flagged, still drawn where it was
-- last seen) and only then dropped, because a responder that misses a second of reports has
-- not gone anywhere -- and a blip that vanishes and reappears is far harder to read than
-- one that goes grey.
function TGT.sweep()
    local now = os.clock()
    for id, t in pairs(TGT.list) do
        if now - t.at > TRACKCFG.DROP_AFTER then
            TGT.list[id] = nil
            if TGT.sel == id then TGT.sel = nil end
        end
    end
end

-- Sorted nearest-first: the traffic strips read down the panel in the order a controller
-- cares about them, and the nearest is the one that survives if the panel runs out of rows.
function TGT.sorted()
    local out = {}
    for _, t in pairs(TGT.list) do
        local dx, dz = t.x - FIX.x, t.z - FIX.z
        t.dist = math.sqrt(dx * dx + dz * dz)
        t.brg  = bearingTo(dx, dz)
        t.age  = os.clock() - t.at
        t.coast = t.age > TRACKCFG.COAST_AFTER
        table.insert(out, t)
    end
    table.sort(out, function(a, b) return a.dist < b.dist end)
    return out
end

function TGT.count()
    local n = 0
    for _ in pairs(TGT.list) do n = n + 1 end
    return n
end

local function rxLoop()
    while true do
        local sender, msg = rednet.receive(LINKS.ATC)
        -- Anything can broadcast on a shared protocol. A report without a position is not a
        -- target, it is noise, and plotting it would put a blip on the station symbol.
        if sender and type(msg) == "table"
                and type(msg.x) == "number" and type(msg.z) == "number"
                and type(msg.y) == "number"
                -- Own-ship's own responder, dropped here rather than drawn. Aer Togekiss's
                -- #45 transmits TOGEKISS on this very channel and the AWACS is bolted to the
                -- same hull, so without this the aircraft plots itself as a permanent contact
                -- sitting exactly under its own symbol -- and, worse, auto-range would count
                -- it as traffic at zero metres and never zoom out.
                and not (ST.OWN_CALLSIGN and msg.callsign == ST.OWN_CALLSIGN) then
            TGT.note(sender, msg)
        end
    end
end

-- ============================================================================
-- NAV DATA SUBSCRIPTION
-- ============================================================================
-- The station is an ordinary client of #24, and a much simpler one than an aircraft: it has
-- no screen an update is unsafe on, so a refresh applies the moment it arrives. What it
-- keeps from the FMS's NDU is the part that matters -- stage beside the live file, verify
-- the length on disk, and only then swap. A CC computer has a hard space limit, and opening
-- the live points.db for writing truncates it before the failure is known; on an aircraft
-- that costs the navigation database mid-flight, and here it would blank the map.

local NDU = {
    PROTOCOL = LINKS.NAVDATA,
    rev = nil, stamp = nil, serverId = nil, lastHeard = nil,
    applied = 0, fault = nil,
}

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
    -- All three staged, then all three swapped. A part-applied refresh would leave airways
    -- naming waypoints that are no longer in points.db, and those legs would simply stop
    -- being drawn with nothing on the screen to say why.
    local files = { {POINTS_FILE, m.points}, {COURSE_FILE, m.courses}, {LANDING_FILE, m.landing} }
    for _, fl in ipairs(files) do
        if not NDU.stage(fl[1], fl[2]) then
            for _, g in ipairs(files) do pcall(fs.delete, g[1] .. CFG.NAVDATA.TMP) end
            NDU.fault = "cannot write - disk full?"
            return false
        end
    end
    for _, fl in ipairs(files) do NDU.swap(fl[1]) end
    NDU.fault = nil
    loadAll()
    NDU.rev, NDU.stamp = m.rev, m.stamp
    NDU.applied = NDU.applied + 1
    return true
end

-- Answer every broadcast, once, whatever the outcome. That is what keeps the station's row
-- on the server's roster alive -- acknowledging only on a change would let a perfectly
-- healthy station age out to SILENT between database edits.
function NDU.report()
    if not NDU.serverId then return end
    rednet.send(NDU.serverId, {
        t = "ack", rev = NDU.rev, stamp = NDU.stamp, fault = NDU.fault,
        who = os.getComputerLabel() or ("computer " .. os.getComputerID()),
    }, NDU.PROTOCOL)
end

function NDU.status()
    if not CFG.NAVDATA.ENABLE then return "NAV local", colours.lightGrey end
    if NDU.fault then return "NAV " .. NDU.fault, colours.red end
    if not NDU.lastHeard then return "NAV local, no server", colours.lightGrey end
    local age = os.clock() - NDU.lastHeard
    if age > CFG.NAVDATA.STALE_AFTER then
        return string.format("NAV rev %s, srv %ds", tostring(NDU.rev or "?"), math.floor(age)),
            colours.orange
    end
    return string.format("NAV rev %s", tostring(NDU.rev or "?")), colours.lime
end

function NDU.loop()
    if not CFG.NAVDATA.ENABLE then while true do sleep(10) end end
    local nextPull = 0
    while true do
        local sender, msg = rednet.receive(NDU.PROTOCOL, 1)
        if sender and type(msg) == "table" and msg.t == "sync" then
            NDU.serverId, NDU.lastHeard = sender, os.clock()
            -- Stamp, not rev: content equality is the real question, and it survives the
            -- server being rebuilt with its revision counter lost.
            if msg.stamp ~= NDU.stamp and NDU.sane(msg) then pcall(NDU.applyNow, msg) end
            NDU.report()
        end
        if not NDU.lastHeard and os.clock() >= nextPull then
            rednet.broadcast({ t = "pull" }, NDU.PROTOCOL)
            nextPull = os.clock() + CFG.NAVDATA.PULL_EVERY
        end
    end
end

-- ============================================================================
-- DISPLAY
-- ============================================================================
-- The scope lives on an attached monitor. Without one it takes the computer's own terminal
-- instead, which is how it can be looked at at all when the tower has not been built yet --
-- the same picture, just fewer cells, and the touch handling falls back to mouse clicks.

local mon, monName, onTerm
do
    if CFG.MONITOR_NAME then
        mon = peripheral.wrap(CFG.MONITOR_NAME)
        if not mon then
            error("atcconf.lua names monitor '" .. CFG.MONITOR_NAME .. "' - not attached.", 0)
        end
        monName = CFG.MONITOR_NAME
    else
        mon = peripheral.find("monitor")
        if mon then monName = peripheral.getName(mon) end
    end
    onTerm = (mon == nil)
    if onTerm then mon = term end
    pcall(function() mon.setTextScale(CFG.TEXT_SCALE) end)
    pcall(function()
        for name, rgb in pairs(CFG.PALETTE.MON) do
            if colours[name] then mon.setPaletteColour(colours[name], rgb) end
        end
    end)
end

-- Every primitive clips to CLIP rather than to the screen, because the map has the traffic
-- strips beside it: a runway two hundred kilometres off scale must not scribble a line
-- through the panel on its way past.
local CLIP = { x1 = 1, y1 = 1, x2 = 1, y2 = 1 }
local function setClip(x1, y1, x2, y2) CLIP.x1, CLIP.y1, CLIP.x2, CLIP.y2 = x1, y1, x2, y2 end

-- ---- the back buffer -------------------------------------------------------
-- EVERY primitive draws into this, and the whole frame reaches the monitor as one blit per
-- row. That is not an optimisation, it is the difference between a scope that runs and one
-- that does not: painting the airways alone touches thousands of cells, and a peripheral
-- call per cell is thousands of round trips per frame -- which a CC computer answers by
-- falling minutes behind and eventually being killed for running too long in a tick. Rows
-- flushed: about thirty. Cells drawn: as many as the picture needs.
local BUF = { w = 0, h = 0, ch = {}, fg = {}, bg = {} }
local BLITCH = {}
do
    local d = "0123456789abcdef"
    local v = 1
    for i = 1, 16 do BLITCH[v] = d:sub(i, i); v = v * 2 end
end
local BLACK, WHITE = BLITCH[colours.black], BLITCH[colours.white]

local function bufResize(w, h)
    if BUF.w == w and BUF.h == h then return end
    BUF.w, BUF.h, BUF.ch, BUF.fg, BUF.bg = w, h, {}, {}, {}
    for y = 1, h do
        BUF.ch[y], BUF.fg[y], BUF.bg[y] = {}, {}, {}
    end
end

local function bufClear()
    for y = 1, BUF.h do
        local c, f, b = BUF.ch[y], BUF.fg[y], BUF.bg[y]
        for x = 1, BUF.w do c[x], f[x], b[x] = " ", WHITE, BLACK end
    end
end

local function bufFlush()
    for y = 1, BUF.h do
        mon.setCursorPos(1, y)
        mon.blit(table.concat(BUF.ch[y]), table.concat(BUF.fg[y]), table.concat(BUF.bg[y]))
    end
end

local function cell(x, y, ch, fg, bg)
    if x < CLIP.x1 or x > CLIP.x2 or y < CLIP.y1 or y > CLIP.y2 then return end
    if x < 1 or x > BUF.w or y < 1 or y > BUF.h then return end
    BUF.ch[y][x] = ch
    if fg then BUF.fg[y][x] = BLITCH[fg] or WHITE end
    if bg then BUF.bg[y][x] = BLITCH[bg] or BLACK end
end

local function px(x, y, col)                    -- one solid colour cell
    cell(math.floor(x + 0.5), math.floor(y + 0.5), " ", nil, col)
end

-- Text is OPAQUE: it lays down its own background, so a label stays readable where it
-- crosses an airway or a range ring. Anything that should let the map show through it is a
-- glyph() instead -- which is the whole difference between a name and a mark.
local function txt(x, y, s, fg, bg)             -- text, clipped character by character
    x, y = math.floor(x + 0.5), math.floor(y + 0.5)
    if y < CLIP.y1 or y > CLIP.y2 then return end
    bg = bg or colours.black
    for i = 1, #s do cell(x + i - 1, y, s:sub(i, i), fg or colours.white, bg) end
end

local function glyph(x, y, ch, fg)              -- a mark; whatever is under it shows through
    cell(math.floor(x + 0.5), math.floor(y + 0.5), ch, fg, nil)
end

local function fillRect(x1, y1, x2, y2, col)
    for y = y1, y2 do
        for x = x1, x2 do cell(x, y, " ", nil, col) end
    end
end

-- Liang-Barsky, and it is not decoration. An airway leg can be thirty kilometres long while
-- the scope is set to 250 m, and rasterising that unclipped is a Bresenham loop of a hundred
-- thousand iterations per leg per frame -- the computer simply stops. Clip first, draw second.
-- Parametric rather than Cohen-Sutherland because Cobalt is Lua 5.1 and has no bitwise ops.
local function clipSeg(x0, y0, x1, y1)
    local dx, dy = x1 - x0, y1 - y0
    local t0, t1 = 0, 1
    local p = { -dx, dx, -dy, dy }
    local q = { x0 - CLIP.x1, CLIP.x2 - x0, y0 - CLIP.y1, CLIP.y2 - y0 }
    for i = 1, 4 do
        if p[i] == 0 then
            if q[i] < 0 then return nil end
        else
            local r = q[i] / p[i]
            if p[i] < 0 then
                if r > t1 then return nil end
                if r > t0 then t0 = r end
            else
                if r < t0 then return nil end
                if r < t1 then t1 = r end
            end
        end
    end
    return x0 + t0 * dx, y0 + t0 * dy, x0 + t1 * dx, y0 + t1 * dy
end

-- Bresenham. `ch` nil draws solid cells (rings, runways); a character draws that instead,
-- which is how the faint things stay faint -- a glyph covers a fraction of the cell where a
-- background colour covers all of it.
local function line(x0, y0, x1, y1, col, ch)
    local a, b, c, d = clipSeg(x0, y0, x1, y1)
    if not a then return end
    x0, y0, x1, y1 = math.floor(a + 0.5), math.floor(b + 0.5),
                     math.floor(c + 0.5), math.floor(d + 0.5)
    local ddx, ddy = math.abs(x1 - x0), -math.abs(y1 - y0)
    local sx, sy = (x0 < x1) and 1 or -1, (y0 < y1) and 1 or -1
    local err = ddx + ddy
    while true do
        if ch then cell(x0, y0, ch, col, nil) else cell(x0, y0, " ", nil, col) end
        if x0 == x1 and y0 == y1 then break end
        local e2 = 2 * err
        if e2 >= ddy then err = err + ddy; x0 = x0 + sx end
        if e2 <= ddx then err = err + ddx; y0 = y0 + sy end
    end
end

-- ============================================================================
-- SCOPE STATE + LAYOUT
-- ============================================================================

local SCOPE = {
    range = MAPCFG.DEFAULT_RANGE,
    auto  = MAPCFG.AUTO_RANGE and true or false,
    wpt   = MAPCFG.SHOW_WAYPOINT and true or false,
    rte   = MAPCFG.SHOW_ROUTE and true or false,
    trl   = MAPCFG.SHOW_TRAIL and true or false,
    blk   = MAPCFG.FULL_BLOCK and true or false,
    nup   = false,                                  -- [NUP]: force north-up, whatever the conf says
    up    = 0,                                      -- resolved per frame by scopeUp()
    fixedUp  = orientationDeg(ST.ORIENTATION),      -- a tower's facing, or a fixed airborne one
    follows  = FOLLOW[tostring(ST.ORIENTATION):lower()] or false,
}

-- Which way is up, this frame. A tower's answer never changes; an aircraft's changes every
-- time it turns, which is the entire point of a heading-up scope -- what is drawn at the top
-- of the screen is what is in front of the windscreen.
local function scopeUp()
    if SCOPE.nup or not SCOPE.follows then
        return SCOPE.nup and 0 or SCOPE.fixedUp
    end
    if not FIX.haveHdg then return 0 end
    return (tostring(ST.ORIENTATION):lower() == "track") and FIX.trk or FIX.hdg
end

local LAY = {}      -- recomputed every frame; the monitor can be resized under us
local HITS = {}     -- touch targets, rebuilt every frame

local function addHit(x1, y1, x2, y2, id)
    table.insert(HITS, { x1 = x1, y1 = y1, x2 = x2, y2 = y2, id = id })
end
local function hitTest(x, y)
    for i = #HITS, 1, -1 do
        local h = HITS[i]
        if x >= h.x1 and x <= h.x2 and y >= h.y1 and y <= h.y2 then return h.id end
    end
end

local function layout()
    local W, H = mon.getSize()
    LAY.W, LAY.H = W, H
    SCOPE.up = scopeUp()
    -- Row 1 is the header, row H the button bar; everything between is the working area.
    local top, bot = 2, H - 1
    -- The strips never take more than a share of the width, and only get the panel at all if
    -- what is left is still a usable map. On a monitor too narrow for both, the map wins --
    -- a scope with no strips is still a scope, where strips with no scope are a text file.
    local pw = math.min(PANELCFG.WIDTH, math.floor(W * 0.4))
    if W - pw - 1 < (PANELCFG.MIN_MAP_W or 24) then pw = 0 end
    LAY.panelW = pw
    LAY.mapX1, LAY.mapY1 = 1, top
    LAY.mapX2, LAY.mapY2 = W - pw - ((pw > 0) and 1 or 0), bot
    LAY.panX1 = W - pw + 1
    local mw, mh = LAY.mapX2 - LAY.mapX1 + 1, LAY.mapY2 - LAY.mapY1 + 1
    LAY.cx = LAY.mapX1 + (mw - 1) / 2
    LAY.cy = LAY.mapY1 + (mh - 1) / 2
    -- The outer range ring is sized to fit BOTH axes, so it stays a circle. The corners of
    -- the box reach past it and are still drawn -- extra ground, simply outside the scale.
    LAY.ry = math.max(1, math.min((mh - 1) / 2, ((mw - 1) / 2) / MAPCFG.CELL_ASPECT))
    LAY.rx = LAY.ry * MAPCFG.CELL_ASPECT
    LAY.kz = LAY.ry / SCOPE.range
    LAY.kx = LAY.kz * MAPCFG.CELL_ASPECT
    local u = math.rad(SCOPE.up)
    LAY.cosUp, LAY.sinUp = math.cos(u), math.sin(u)
end

-- World (x, z) to screen cell. Rotating by the station's orientation puts CFG.STATION's
-- facing at the top of the picture; with ORIENTATION = north this is the identity, which is
-- exactly the check to make after changing it.
local function project(wx, wz)
    local dx, dz = wx - FIX.x, wz - FIX.z
    local ex =  dx * LAY.cosUp + dz * LAY.sinUp
    local ey = -dx * LAY.sinUp + dz * LAY.cosUp
    return LAY.cx + ex * LAY.kx, LAY.cy + ey * LAY.kz
end

-- A point at range r on scope-relative bearing b (deg from the top of the screen).
local function polar(bdeg, r)
    local a = math.rad(bdeg)
    return LAY.cx + r * LAY.rx * math.sin(a), LAY.cy - r * LAY.ry * math.cos(a)
end

local function pickRange()
    if not SCOPE.auto then return end
    local far = 0
    for _, t in pairs(TGT.list) do
        local dx, dz = t.x - FIX.x, t.z - FIX.z
        local d = math.sqrt(dx * dx + dz * dz)
        if d > far then far = d end
    end
    if far <= 0 then return end
    far = far * (MAPCFG.AUTO_MARGIN or 1.15)
    local pickTo = MAPCFG.RANGES[#MAPCFG.RANGES]
    for _, r in ipairs(MAPCFG.RANGES) do if far <= r then pickTo = r; break end end
    SCOPE.range = pickTo
end

local function stepRange(dir)
    SCOPE.auto = false
    local idx = 1
    for i, r in ipairs(MAPCFG.RANGES) do
        if r == SCOPE.range then idx = i; break end
        if r < SCOPE.range then idx = i + 1 end
    end
    SCOPE.range = MAPCFG.RANGES[clamp(idx + dir, 1, #MAPCFG.RANGES)]
end

-- ============================================================================
-- THE SCOPE PICTURE
-- ============================================================================

local function vsArrow(vs)
    if vs > 0.5 then return "^" elseif vs < -0.5 then return "v" else return "-" end
end

-- Selected first, then the alert, then the age. A target reporting a TCAS resolution is red
-- whatever else is true of it -- that is the one thing on the screen that cannot wait.
local function tgtColour(t)
    if TGT.sel == t.id then return colours.white end
    if t.tcas and t.tcas ~= "CLEAR" then return colours.red end
    if t.coast then return colours.lightGrey end
    if t.ifr then return colours.cyan end
    return colours.lime
end

-- Eight-way blip, pointing along the target's ground track in SCREEN terms -- so on a scope
-- oriented to the field rather than to north, the arrows turn with the picture.
local BLIP = { "^", "/", ">", "\\", "v", "/", "<", "\\" }
local function blipGlyph(dirdeg)
    return BLIP[math.floor((((dirdeg - SCOPE.up) % 360) + 22.5) / 45) % 8 + 1]
end

local function drawRings()
    local rings = MAPCFG.RINGS or 3
    for i = 1, rings do
        local f = i / rings
        -- One dot roughly every three cells of circumference: dense enough to read as a
        -- ring, cheap enough to redraw four times a second.
        local steps = clamp(math.floor(2 * math.pi * LAY.rx * f / 3), 12, 180)
        for k = 0, steps - 1 do
            local x, y = polar(k * 360 / steps, f)
            px(x, y, colours.grey)
        end
    end
    -- Compass card, turned with the picture: the 30s ticked ON the outer ring, the four
    -- cardinals named just inside it. Inside matters -- the ring is sized to touch the top
    -- and bottom rows of the map, so anything drawn outside it is off the box and lost.
    local card = { [0] = "N", [90] = "E", [180] = "S", [270] = "W" }
    for b = 0, 330, 30 do
        local rel = b - SCOPE.up
        if card[b] then
            local x, y = polar(rel, 0.93)
            txt(x, y, card[b], colours.lightGrey)
        else
            local x, y = polar(rel, 1.0)
            px(x, y, colours.grey)
        end
    end
end

local function drawAirways()
    if not SCOPE.rte then return end
    for _, a in ipairs(AIRWAYS) do
        local x1, y1 = project(a.x1, a.z1)
        local x2, y2 = project(a.x2, a.z2)
        line(x1, y1, x2, y2, colours.green)
    end
end

local function drawSites()
    local inRing = SCOPE.range      -- labels are decluttered to the scale, glyphs are not

    -- The strips first, all of them, before any threshold or label goes down. A two-way
    -- runway is TWO records -- TMV and TMA are the two ends of one 295 m strip -- and each
    -- draws the whole thing with the OTHER end marked as its far end. Drawn in one pass,
    -- whichever record came second would paint brown over the first one's threshold; in two,
    -- every published threshold survives.
    for _, s in ipairs(SITES) do
        if s.kind == "rwy" then
            local sx, sy = project(s.x, s.z)
            local ex, ey = project(s.x + s.ux * s.len, s.z + s.uz * s.len)
            if math.abs(ex - sx) + math.abs(ey - sy) >= 1 then
                line(sx, sy, ex, ey, colours.orange)
                px(ex, ey, colours.brown)
            end
        end
    end

    for _, s in ipairs(SITES) do
        local show = (s.kind ~= "vor") or SCOPE.wpt
        if show then
            local sx, sy = project(s.x, s.z)
            local dx, dz = s.x - FIX.x, s.z - FIX.z
            local named  = (dx * dx + dz * dz) <= inRing * inRing
            if s.kind == "rwy" then
                -- The threshold, which is the point points.db actually holds, and the name
                -- against it. A strip too short to draw at this scale still gets a mark.
                local ex, ey = project(s.x + s.ux * s.len, s.z + s.uz * s.len)
                if math.abs(ex - sx) + math.abs(ey - sy) < 1 then
                    txt(sx, sy, "=", colours.orange)
                end
                px(sx, sy, colours.orange)
                if named then txt(sx + 2, sy, s.name, colours.orange) end
            elseif s.kind == "heli" then
                txt(sx, sy, "H", colours.orange)
                if named then txt(sx + 2, sy, s.name, colours.orange) end
            elseif s.kind == "vor" then
                txt(sx, sy, "+", colours.lightGrey)
                if named then txt(sx + 2, sy, s.name, colours.lightGrey) end
            else
                txt(sx, sy, "o", colours.lightGrey)
                if named then txt(sx + 2, sy, s.name, colours.lightGrey) end
            end
        end
    end
end

-- The centre of the picture: a tower, or ourselves. Airborne it is drawn as a blip like any
-- other target, which means that on a heading-up scope it is always the nose-up "^" and on a
-- north-up one it swings round -- so the operator can see at a glance which the scope is in.
local function drawOwnship()
    if not FIX.have then return end
    local col = DYN and colours.white or colours.orange
    if DYN and FIX.haveHdg then
        txt(LAY.cx, LAY.cy, blipGlyph(FIX.hdg), col)
    else
        txt(LAY.cx, LAY.cy, "*", col)
    end
    -- A blank NAME means "no tag": own-ship is already named in the header, and the middle of
    -- an airborne scope is the busiest part of it.
    local nm = tostring(ST.NAME or ""):match("^%s*(.-)%s*$")
    if #nm > 0 then txt(LAY.cx + 2, LAY.cy, nm, col) end
end

-- Where each target has been and where it is going. Drawn UNDER the aerodromes: an
-- aircraft on final is by definition over the runway it is landing on, and a leader line
-- that erased the strip would blank the one piece of the map that approach is about.
local function drawTargetTracks(list)
    for i = #list, 1, -1 do
        local t   = list[i]
        local col = tgtColour(t)
        local sx, sy = project(t.x, t.z)

        if SCOPE.trl then
            for _, h in ipairs(t.trail) do
                local hx, hy = project(h.x, h.z)
                glyph(hx, hy, ".", col)
            end
        end

        -- The velocity leader: where this target will be in CFG.TRACK.LEADER_SEC seconds if
        -- it does nothing. Length IS the speed -- that is the whole point of drawing it --
        -- so a fast aircraft trails a long line and a hovering one none at all.
        if t.gs > 1 and not t.coast then
            -- Capped to a fraction of the range. A one-minute leader is the airway-chart
            -- convention and it assumes an airway-chart scale: at 320 m/s it is 19 km, which
            -- on a 1 km approach scope is a line straight off the screen telling you nothing.
            -- Capped, the leader still reads as "faster = longer" everywhere it fits, and
            -- degrades to a direction arrow instead of a wall where it does not.
            local L = math.min(t.gs * TRACKCFG.LEADER_SEC,
                SCOPE.range * (TRACKCFG.LEADER_MAX_FRAC or 0.35))
            local a = math.rad(t.trk)
            local lx, ly = project(t.x + math.sin(a) * L, t.z - math.cos(a) * L)
            line(sx, sy, lx, ly, col, ".")
        end
    end
end

-- ...and the targets themselves, last of all, over everything.
local function drawTargetBlips(list)
    for i = #list, 1, -1 do             -- nearest last, so it paints over the far ones
        local t   = list[i]
        local col = tgtColour(t)
        local sx, sy = project(t.x, t.z)

        -- Data block, offset clear of the blip, flipped to the other side rather than
        -- letting it run off the edge of the map.
        local l1 = t.csg .. (t.coast and " C" or "")
        local l2 = string.format("%d%s %d", math.floor(t.y + 0.5), vsArrow(t.vs),
            math.floor(t.gs + 0.5))
        local bw = math.max(#l1, SCOPE.blk and #l2 or 0)
        local bx = sx + 2
        if bx + bw - 1 > LAY.mapX2 then bx = sx - 1 - bw end
        if SCOPE.blk then
            txt(bx, sy - 1, l1, col)
            txt(bx, sy, l2, col)
        else
            txt(bx, sy, l1, col)
        end

        if TGT.sel == t.id then
            txt(sx, sy, blipGlyph(t.trk), colours.white, colours.blue)
        else
            txt(sx, sy, blipGlyph(t.trk), col)
        end
        -- Selecting by touch means knowing where the blip landed; the box is generous
        -- because a target is one cell and a finger is not. Clamped to the map, and skipped
        -- entirely for a blip off the edge of it -- an off-scale contact must not leave an
        -- invisible hitbox sitting over the traffic strips or the button bar.
        local r = MAPCFG.PICK_RADIUS or 2
        if sx >= LAY.mapX1 and sx <= LAY.mapX2 and sy >= LAY.mapY1 and sy <= LAY.mapY2 then
            addHit(math.max(math.floor(sx - r), LAY.mapX1), math.max(math.floor(sy - r), LAY.mapY1),
                   math.min(math.floor(sx + r), LAY.mapX2), math.min(math.floor(sy + r), LAY.mapY2),
                   "sel:" .. t.id)
        end
    end
end

-- ============================================================================
-- TRAFFIC STRIPS
-- ============================================================================
-- One block per contact down the right-hand side, nearest first. Everything the map cannot
-- say in two lines beside a blip goes here: the full callsign, the flight rule, the bearing
-- and range from this station, and how stale the report is.

local function drawStrip(t, y, x1, W)
    local col  = tgtColour(t)
    local sel  = (TGT.sel == t.id)
    local bg   = sel and colours.blue or colours.black
    local alert = (t.tcas ~= nil and t.tcas ~= "CLEAR")
    local rows = 3 + (alert and 1 or 0)
    if sel then fillRect(x1, y, W, y + rows - 1, colours.blue) end

    -- A coasting target says how long it has been quiet INSTEAD of its flight rule: the rule
    -- is what it was doing when last heard, and the seconds are the thing worth reading.
    local tag = t.coast and string.format("C%ds", math.floor(t.age))
        or (t.ifr and "IFR" or "VFR")
    txt(x1, y, (t.csg .. " " .. t.callsign):sub(1, W - x1 - #tag), col, bg)
    txt(W - #tag + 1, y, tag, t.coast and colours.lightGrey or col, bg)

    local a = string.format("ALT%4d %s%.1f", math.floor(t.y + 0.5), vsArrow(t.vs), math.abs(t.vs))
    local b = string.format("GS%d", math.floor(t.gs + 0.5))
    txt(x1, y + 1, a, colours.white, bg)
    txt(W - #b + 1, y + 1, b, colours.white, bg)

    a = string.format("BRG%03d D%s", math.floor(t.brg + 0.5) % 360, fmtDist(t.dist))
    b = string.format("TRK%03d", math.floor(t.trk + 0.5) % 360)
    txt(x1, y + 2, a, colours.lightGrey, bg)
    txt(W - #b + 1, y + 2, b, colours.lightGrey, bg)

    if alert then txt(x1, y + 3, ("TCAS " .. tostring(t.tcas)):sub(1, W - x1 + 1), colours.red, bg) end

    addHit(x1, y, W, y + rows - 1, "sel:" .. t.id)
    return rows
end

local function drawPanel(list)
    if LAY.panelW <= 0 then return end
    local x1, W = LAY.panX1, LAY.W
    setClip(x1 - 1, LAY.mapY1, W, LAY.mapY2)
    for y = LAY.mapY1, LAY.mapY2 do px(x1 - 1, y, colours.blue) end

    local y = LAY.mapY1
    txt(x1, y, "TRAFFIC", colours.white)
    local n = tostring(#list)
    txt(W - #n + 1, y, n, colours.white)
    y = y + 1
    txt(x1, y, string.rep("-", W - x1 + 1), colours.grey)
    y = y + 1

    if #list == 0 then
        txt(x1, y + 1, "NO CONTACTS", colours.lightGrey)
        return
    end
    for i, t in ipairs(list) do
        local need = 3 + ((t.tcas ~= nil and t.tcas ~= "CLEAR") and 1 or 0)
        if y + need - 1 > LAY.mapY2 then
            txt(x1, LAY.mapY2, "+" .. (#list - i + 1) .. " more", colours.lightGrey)
            break
        end
        y = y + drawStrip(t, y, x1, W) + 1
    end
end

-- ============================================================================
-- HEADER AND BUTTON BAR
-- ============================================================================

local function drawHeader(n)
    local W = LAY.W
    setClip(1, 1, W, 1)
    fillRect(1, 1, W, 1, colours.blue)
    -- A tower says where it stands, once. An aircraft says what it is doing, because on an
    -- airborne scope every bearing and range on the screen is measured from a number that is
    -- changing -- and a controller reading a range off a moving origin needs to know it.
    -- Two forms of each, because a narrow monitor has to give something up: the SHORT form
    -- keeps whichever half is not already obvious, which for a tower is its name and for an
    -- aircraft is its state -- nobody needs reminding which aeroplane they are standing in.
    local leftFull, leftShort
    if DYN then
        leftShort = string.format("ALT%d%s GS%d %s", math.floor(FIX.y + 0.5), vsArrow(FIX.vs),
            math.floor(FIX.gs + 0.5),
            FIX.haveHdg and string.format("HDG%03d", math.floor(FIX.hdg + 0.5) % 360) or "HDG---")
        leftFull = (ST.LABEL or ST.NAME) .. "  " .. leftShort
    else
        leftFull = string.format("%s  %d,%d", ST.LABEL or ST.NAME,
            math.floor(FIX.x + 0.5), math.floor(FIX.z + 0.5))
        leftShort = ST.NAME
    end
    -- What each ring is worth belongs here and not on the map: at a terminal-area scale the
    -- centre of the scope is exactly where the aerodrome labels already are, and a number
    -- printed across them costs more than it tells.
    local mid   = string.format("RNG %s  RING %s%s", fmtDist(SCOPE.range),
        fmtDist(SCOPE.range / (MAPCFG.RINGS or 3)), SCOPE.auto and "  AUTO" or "")
    local right = string.format("TFC %d  %s", n, textutils.formatTime(os.time(), true))
    local left  = leftFull
    -- A stale or missing fix outranks everything else in the bar. Every range and bearing on
    -- the screen is measured from it, so if it is old the whole picture is old -- and saying
    -- so is the one thing the header must never be too narrow to do.
    local warn = nil
    if DYN and not FIX.have then warn = "NO FIX"
    elseif DYN and fixAge() > (TRACKCFG.FIX_STALE_AFTER or 2) then
        warn = string.format("FIX %ds OLD", math.floor(fixAge()))
    end
    if warn then left, right = (ST.LABEL or ST.NAME), warn end
    -- Give up detail in order, narrowest monitor last. Text is opaque, so anything that does
    -- not fit does not crowd -- it overwrites, and a header reading "7RNG 1.0k" is worse than
    -- one that simply left the station's coordinates out.
    -- The warning is never what gets dropped. Everything else goes first, the range last of
    -- all, and on a monitor too narrow even for that the bar says "NO FIX" and nothing else --
    -- which is still the only thing worth saying when the scope has no origin.
    local function fits() return #left + #mid + #right + 4 <= W end
    if not fits() and not warn then
        left, right = leftShort, string.format("TFC %d", n)
    end
    if not fits() then
        mid = string.format("RNG %s%s", fmtDist(SCOPE.range), SCOPE.auto and " A" or "")
    end
    if not fits() then left = "" end
    if not fits() and not warn then right = "" end
    if not fits() then mid = "" end
    if not fits() then right = right:sub(1, math.max(0, W - 2)) end

    -- Left flush left, right flush right, the range centred in whatever is left between them.
    -- Centring the range FIRST and fitting the others around it is what used to drop a long
    -- label that would have fitted perfectly well beside it.
    local lx = 2
    local rx = (#right > 0) and (W - #right) or (W + 1)
    local lo, hi = lx + #left + 1, rx - 1 - #mid
    local mx = clamp(math.floor((lo + hi) / 2), lo, math.max(lo, hi))
    if #left > 0 then txt(lx, 1, left, colours.white, colours.blue) end
    if #mid > 0 then txt(mx, 1, mid, colours.orange, colours.blue) end
    if #right > 0 then
        txt(rx, 1, right, warn and colours.red or colours.white, colours.blue)
    end
end

-- [-] and [+] are ZOOM, not range: [+] moves in to a SMALLER radius and [-] pulls out to a
-- larger one, which is the direction every map anyone has ever used works in. (The ids say
-- zoom for the same reason -- "rng+" meaning "less range" is exactly the sort of thing that
-- gets swapped back by accident.) Either one also drops out of auto: reaching for the zoom is
-- the moment a controller has stopped wanting the scope to choose for them. The rest are
-- toggles, lit when on.
local BTNS = {
    { id = "zoomout", label = "-" },
    { id = "zoomin",  label = "+" },
    { id = "auto", label = "AUTO", key = "auto" },
    { id = "wpt",  label = "WPT",  key = "wpt"  },
    { id = "rte",  label = "RTE",  key = "rte"  },
    { id = "trl",  label = "TRL",  key = "trl"  },
    { id = "blk",  label = "BLK",  key = "blk"  },
    { id = "nup",  label = "NUP",  key = "nup"  },
}

local function drawFooter()
    local W, H = LAY.W, LAY.H
    setClip(1, H, W, H)
    fillRect(1, H, W, H, colours.black)
    local x = 2
    for _, b in ipairs(BTNS) do
        local w = #b.label + 2
        if x + w - 1 <= W - 2 then
            local col = b.key and (SCOPE[b.key] and colours.lime or colours.grey) or colours.white
            txt(x, H, "[", colours.lightGrey)
            txt(x + 1, H, b.label, col)
            txt(x + w - 1, H, "]", colours.lightGrey)
            addHit(x, H, x + w - 1, H, b.id)
            x = x + w + 1
        end
    end
    local st, sc = NDU.status()
    if W - #st > x then txt(W - #st, H, st, sc) end
end

-- ============================================================================
-- FRAME
-- ============================================================================

local function drawFrame()
    updateFix()
    TGT.sweep()
    local list = TGT.sorted()
    pickRange()
    layout()
    HITS = {}
    bufResize(LAY.W, LAY.H)
    setClip(1, 1, LAY.W, LAY.H)
    bufClear()
    drawHeader(#list)
    setClip(LAY.mapX1, LAY.mapY1, LAY.mapX2, LAY.mapY2)
    -- Strictly back to front, and the order is the whole argument about what matters most
    -- when two things want the same cell: airways lose to the range card, the range card
    -- loses to a target's track, the track loses to an aerodrome, and nothing beats a blip.
    if not FIX.have then
        -- Nothing on this map means anything without a centre to hang it on, so none of it is
        -- drawn: an aerodrome plotted relative to a position we do not have is not a partial
        -- picture, it is a wrong one.
        drawRings()
        local m = "NO FIX - WAITING FOR SUBLEVEL"
        txt(LAY.cx - #m / 2, LAY.cy, m, colours.red)
        drawPanel(list)
        drawFooter()
        bufFlush()
        return
    end
    drawAirways()
    drawRings()
    drawTargetTracks(list)
    drawSites()
    drawOwnship()
    drawTargetBlips(list)
    drawPanel(list)
    drawFooter()
    bufFlush()
end

local function drawLoop()
    while true do
        local ok, err = pcall(drawFrame)
        if not ok then
            -- A bad frame must not take the receiver down with it: the station keeps
            -- listening, and the error stays on the screen until it is fixed.
            pcall(function()
                mon.setBackgroundColour(colours.black); mon.setTextColour(colours.red)
                mon.setCursorPos(1, 1); mon.clearLine(); mon.write("DRAW: " .. tostring(err))
            end)
        end
        sleep(CFG.REDRAW)
    end
end

-- ============================================================================
-- INPUT
-- ============================================================================
-- Touch on the monitor, mouse when the scope has fallen back to the terminal. Touching a
-- blip or its strip selects that target (and touching it again lets it go); touching bare
-- map clears the selection, which is the gesture people try first.

local function touchLoop()
    while true do
        local ev = { os.pullEvent() }
        local x, y
        if ev[1] == "monitor_touch" and (not monName or ev[2] == monName) then
            x, y = ev[3], ev[4]
        elseif ev[1] == "mouse_click" and onTerm then
            x, y = ev[3], ev[4]
        end
        if x then
            local id = hitTest(x, y)
            if id == "zoomin" then stepRange(-1)
            elseif id == "zoomout" then stepRange(1)
            elseif id == "auto" then SCOPE.auto = not SCOPE.auto
            elseif id == "wpt"  then SCOPE.wpt  = not SCOPE.wpt
            elseif id == "rte"  then SCOPE.rte  = not SCOPE.rte
            elseif id == "trl"  then SCOPE.trl  = not SCOPE.trl
            elseif id == "blk"  then SCOPE.blk  = not SCOPE.blk
            elseif id == "nup"  then SCOPE.nup  = not SCOPE.nup
            elseif id and id:sub(1, 4) == "sel:" then
                local n = tonumber(id:sub(5))
                TGT.sel = (TGT.sel == n) and nil or n
            elseif not id and x >= LAY.mapX1 and x <= LAY.mapX2
                    and y >= LAY.mapY1 and y <= LAY.mapY2 then
                -- Bare map clears the selection. Bare anything ELSE does not: missing a
                -- button by one column, or the gap between two strips, must not silently
                -- throw away the target the controller was watching.
                TGT.sel = nil
            end
            pcall(drawFrame)        -- answer the touch now, not at the next repaint
        end
    end
end

-- ============================================================================
-- THE COMPUTER'S OWN SCREEN
-- ============================================================================
-- Not a second scope -- a maintenance readout. It answers the two questions asked when the
-- monitor shows nothing: is the radio open, and is anything transmitting.

local function termLoop()
    if onTerm then while true do sleep(10) end end
    while true do
        term.setBackgroundColour(colours.black); term.setTextColour(colours.white)
        term.clear(); term.setCursorPos(1, 1)
        print("ATC RADAR - " .. (ST.LABEL or ST.NAME))
        print("scope on " .. tostring(monName))
        print("listening " .. LINKS.ATC)
        local st = NDU.status()
        print(st)
        local list = TGT.sorted()
        print("contacts " .. #list)
        local _, h = term.getSize()
        for i, t in ipairs(list) do
            if 6 + i > h then break end
            print(string.format(" %-3s %-8s %4dm %3dm/s %03d", t.csg, t.callsign:sub(1, 8),
                math.floor(t.y + 0.5), math.floor(t.gs + 0.5), math.floor(t.brg + 0.5) % 360))
        end
        sleep(CFG.TERM_REDRAW)
    end
end

-- ============================================================================
-- MAIN
-- ============================================================================

-- Wireless by preference: a tower big enough to want a monitor wall is likely to have a
-- WIRED modem for it, and rednet.open on that one would leave the station deaf to every
-- aircraft while looking perfectly healthy.
local modem = peripheral.find("modem", function(_, m) return m.isWireless() end)
    or peripheral.find("modem")
if not modem then error("No modem attached - the station cannot hear anything.", 0) end
rednet.open(peripheral.getName(modem))

loadAll()
updateFix()

parallel.waitForAny(rxLoop, drawLoop, touchLoop, NDU.loop, termLoop)
