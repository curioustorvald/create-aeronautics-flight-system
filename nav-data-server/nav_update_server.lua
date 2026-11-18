-- NAV DATA UPDATE SERVER -- computer #24
-- =====================================================================================
-- The master copy of the fleet's navigation database, a keyboard+mouse editor for it, and
-- the broadcaster that keeps every autopilot's own copy in step.
--
-- WHY A SERVER AT ALL. Every FMS (#8 Aer Togekiss, #16 Citation) carries its OWN
-- points.db / course.db / landing.db and reads them at boot -- that stays true, and is
-- deliberate: an aircraft out of radio contact still has a complete database and flies
-- normally. This computer does not replace those files, it REFRESHES them: it owns the
-- editable master and broadcasts the whole set every BROADCAST_INTERVAL seconds, so a
-- listening FMS can notice its copy is out of date and reload.
--
-- THE WIRE FORMAT IS THE FILE ITSELF. The sync message carries the three .db files as
-- plain text, exactly as they sit on this computer's disk. An FMS writes them straight to
-- its own disk and then calls its EXISTING loadPoints()/loadCourses()/loadLanding(). That
-- is the whole point: there is no second parser and no second schema anywhere in the
-- fleet, so this program cannot drift out of format agreement with the aircraft. The
-- parsers and validators below therefore mirror the FMS's patterns EXACTLY -- a line this
-- refuses is a line the aircraft would silently skip, which is far worse than an error
-- message here.
--
-- PROTOCOL "aer_navdata"
--   server -> all   {t="sync", rev, stamp, points=<text>, courses=<text>, landing=<text>,
--                    sites=<text>}
--                   broadcast every BROADCAST_INTERVAL s, and at once after any edit;
--                   the disk is re-read immediately before every one of them.
--                   `sites` is sites.db, which only the ATC controller (#87) reads -- every
--                   other client ignores the field, as an extra field is ignored anywhere on
--                   this network. It is carried here rather than kept on #87 so that the
--                   fleet's navigation data has ONE master, and it is hand-edited: this
--                   program has no editor for it, only the re-read below.
--   client -> all   {t="pull"}                 "I just booted, send me the set now"
--                   answered with a targeted sync, so a fresh FMS need not wait 30 s.
--   client -> srv   {t="ack", rev, stamp, who} "applied it" -- drives the CLIENTS roster
--                   on the dashboard, which is how you confirm an edit actually landed.
--
-- THE DISK IS THE MASTER, NOT THIS PROGRAM'S COPY OF IT. Every send re-reads the three
-- files first (refreshFromDisk, below), so a broadcast can never be older than the files it
-- claims to be carrying. The files here are edited from outside this program as often as
-- through its own editor -- in the world save, with the server left running -- and before
-- this the fleet went on being told the boot-time contents until someone pressed R.
--
--   rev   -- monotonic counter, persisted in navdata.rev, bumped on every save. For
--            display and ordering only.
--   stamp -- checksum of the three payloads. This, not rev, is what a client compares:
--            content equality is the real question, and it survives this computer being
--            rebuilt with its rev file lost.
--
-- WHAT A REFRESH MAY NOT DO. An aircraft in flight must not have the course it is
-- currently flying altered underneath it. That guarantee lives on the FMS side and it is
-- structural, not a rule this program has to remember: NAV.points is a snapshot copied at
-- plan-set time, never a live reference into the point table (autopilot.lua, "NAV ENGINE"),
-- and NAV.landSite likewise. So a mid-flight reload re-arms every FUTURE plan without
-- touching the active one. The FMS additionally defers applying while the pilot is on any
-- screen that lists or edits data -- see its NDU block.
--
-- Editing here is authoritative: a sync overwrites whatever an FMS holds locally,
-- including edits made on the aircraft's own EDIT DATA screens. Those still work, they are
-- just transient now -- changes meant to last belong on this computer.
-- =====================================================================================

local NAVDATA_PROTOCOL   = "aer_navdata"
local BROADCAST_INTERVAL = 30    -- s between unsolicited full broadcasts
local CLIENT_STALE       = 90    -- s of silence before a client is shown as silent
local TOAST_TIME         = 5     -- s a status message stays on the bottom line

local POINTS_FILE  = "points.db"
local COURSE_FILE  = "course.db"
local LANDING_FILE = "landing.db"
local SITES_FILE   = "sites.db"     -- ATC sites (#87); hand-edited only -- see the header
local REV_FILE     = "navdata.rev"

-- Reproduced verbatim whenever landing.db is rewritten -- it is the only documentation of
-- the field order and of Minecraft's compass/axis convention, and it belongs in the file
-- rather than only in this program.
local LANDING_HEADER = [[; autolanding info
; aerodrome/poi = type (heli,rwy,goaround,appr), direction (n, w, s, e; x for don't care; integer/decimal number for heading), altitude, runway length in meters (ignored for heli)
; IF TYPE IS goaround (goaround pattern), the format changes:
; random id = "goaround", starting point X, starting point Y, ending point X, ending point Y, altitude
; random id = "goaround-wp", minimum altitude, waypoints (always starts AND ends at the runway it belongs to)
; an FMS should be able to circle into the goaround route, fly straight, circle back to runway line, realign, and try landing again.
; IF TYPE IS appr (approach pattern), the format AND the interpretation changes:
; interpretation, on preparation to the landing, the FMS MUST choose the most suitable approach pattern according to its current position and heading
; random id = "appr", minimum altitude, waypoints
; BOTH GOAROUND AND APPROACH PATTERN CAN REFERENCE EACH OTHERS PATTERN
;   appr naming another appr  = continue as that pattern; its fixes are spliced in where the id stands
;   appr naming a goaround    = that circuit is this approach's MISSED APPROACH, and names the runway it ends at
; ALL AEROPLANES MUST ABIDE BY THE GOAROUND AND APPROACH PATTERNS
; Minecraft coord convention:
; North: Z-
; East: X+
; South: Z+
; West: X-
; Altitude: Y axis
]]

local POINTS_HEADER_POI = "; aerodromes and/or point-of-interests (always three letters)"
local POINTS_HEADER_VOR = "; waypoints (VORs are always five letters long)"
-- Reproduced whenever course.db is rewritten, for the same reason LANDING_HEADER is: a route
-- entry that is not a point is a REFERENCE, and the file should say so where it is read.
local COURSE_HEADER     = [[; routes
; ROUTE-NAME=N1,N2,...,Nn  -- at least two, name charset A-Z 0-9 -
; a token may name another ROUTE instead of a point: its own waypoints are spliced in there
; a token may name an "appr" pattern from landing.db: that published approach is flown in
;   there, and the route then lands by it -- minimum altitude and missed approach included
; a goaround id is NOT a route and is skipped: it is how you leave, not how you arrive
; unknown names are skipped, and the route is silently shortened around them]]

-- =====================================================================================
-- DATA LAYER
-- =====================================================================================
-- body.* is the exact text last read from / written to disk, and is what goes on the wire.
-- Keeping it beside the parsed tables means "what the dashboard shows", "what is on disk"
-- and "what the aircraft receive" can never become three different things.

local DB = {
    points = {}, pointOrder = {},   -- name -> {name, x, z, kind}
    courses = {}, courseOrder = {}, -- name -> {name, seq={names}, gapBefore}
    landing = {}, landOrder = {},   -- name -> a landing site {name, type, dir, alt, rwyLen},
                                    -- a go-around leg {name, type, x1, z1, x2, z2, alt} or a
                                    -- go-around circuit {name, type, alt, seq={names}}
    body = {points = "", courses = "", landing = "", sites = ""},
    siteCount = 0,              -- SITE= lines in sites.db, for the dashboard
    rev = 0,
    stamp = "",
    adopts = 0, lastAdopt = 0,  -- disk changes picked up by refreshFromDisk
    diskFault = nil,            -- why the last re-read was refused, for the dashboard
}

local function readFile(path)
    local f = io.open(path, "r")
    if not f then return nil end
    local s = f:read("*a")
    f:close()
    return s
end

local function writeFile(path, body)
    local f = io.open(path, "w")
    if not f then return false end
    f:write(body)
    f:close()
    return true
end

local function trim(s) return (s:gsub("^%s*(.-)%s*$", "%1")) end

-- djb2 over the payloads. There is no md5 in CC and none is needed -- this only has
-- to detect "the data changed", against an editor that changes it deliberately, not
-- against an adversary. Formatted in 16-bit halves because Lua 5.1's %x on a value above
-- 2^31 is not something to rely on.
--
-- sites.db is folded in only when it has content, so a server with no sites stamps exactly
-- what it always did -- and with some, a sites-only edit still changes the stamp, which is
-- the only thing that tells #87 to reload. (Every other client re-applies its three
-- unchanged files once when that happens: harmless, and the price of one stamp.)
local function stampOf(a, b, c, d)
    local s = (a or "") .. "\30" .. (b or "") .. "\30" .. (c or "")
    if d and d ~= "" then s = s .. "\30" .. d end
    local h = 5381
    for i = 1, #s do
        h = (h * 33 + s:byte(i)) % 4294967296
    end
    return string.format("%04x%04x.%d", math.floor(h / 65536), h % 65536, #s)
end

-- The three parsers apply line-for-line the same tests the FMS applies.
local function parsePoints(body)
    local pts, order = {}, {}
    for line in (body or ""):gmatch("[^\r\n]+") do
        local t = trim(line)
        if t ~= "" and t:sub(1, 1) ~= ";" then
            local name, xs, zs = t:match("^(%a+)=(%-?%d+),(%-?%d+)$")
            if name then
                if not pts[name] then table.insert(order, name) end
                pts[name] = {
                    name = name, x = tonumber(xs), z = tonumber(zs),
                    kind = (#name == 5) and "vor" or "poi",
                }
            end
        end
    end
    return pts, order
end

-- gapBefore preserves the blank-line grouping the hand-maintained course.db uses to keep
-- related routes together; without it the first save from this editor would flatten the
-- whole file into one block.
local function parseCourses(body)
    local crs, order, gap = {}, {}, false
    for line in (body or ""):gmatch("[^\r\n]*") do
        local t = trim(line)
        if t == "" then
            gap = true
        elseif t:sub(1, 1) ~= ";" then
            local name, rest = t:match("^([%w%-]+)=(.+)$")
            if name then
                local seq = {}
                for p in rest:gmatch("[^,]+") do table.insert(seq, trim(p)) end
                if not crs[name] then table.insert(order, name) end
                crs[name] = {name = name, seq = seq, gapBefore = gap}
                gap = false
            end
        end
    end
    return crs, order
end

local function parseLanding(body)
    local lnd, order = {}, {}
    for line in (body or ""):gmatch("[^\r\n]+") do
        local t = trim(line)
        if t ~= "" and t:sub(1, 1) ~= ";" then
            local name, ty, dir, alt, rwy = t:match("^(%a+)=(%a+),([%a%.%d]+),(%-?%d+),(%-?%d+)$")
            -- GO-AROUND routes are a SECOND record shape in the same file (see the header):
            -- an arbitrary id, then the two ends of a straight leg and the altitude to fly it
            -- at. They are not landing SITES -- they are not tied to a point and nothing here
            -- may treat them as one -- but they live in landing.db because they belong to the
            -- runway they serve. Parsed and re-serialised so an edit made on this screen can
            -- never quietly drop them (the old single pattern did exactly that: the ids carry
            -- an underscore and there are five numbers, so no goaround line matched, and
            -- bodyLanding() then rewrote the file from what HAD matched).
            local gid, gx1, gz1, gx2, gz2, galt =
                t:match("^([%w_]+)=goaround,(%-?%d+),(%-?%d+),(%-?%d+),(%-?%d+),(%-?%d+)$")
            -- ...and a THIRD shape, the preferred one: the same circuit published as a list of
            -- waypoints rather than one leg, with the runway it serves named at both ends of
            -- that list. Same rule as above and for the same reason -- it is parsed here only
            -- so that it survives being re-serialised.
            local wid, walt, wseq = t:match("^([%w_]+)=goaround%-wp,(%-?%d+),(.+)$")
            -- ...and a FOURTH: a published APPROACH -- how an aircraft ARRIVES, where the two
            -- above are how it leaves again. Same list shape as the circuit, same reason for
            -- parsing it here (so that it survives being re-serialised), and the same trap
            -- avoided: an appr line matches none of the site patterns, so a version of this
            -- program that did not know about it would rewrite the file without them.
            local aid, aalt, aseq = t:match("^([%w_]+)=appr,(%-?%d+),(.+)$")
            if gid then
                if not lnd[gid] then table.insert(order, gid) end
                lnd[gid] = {
                    name = gid, type = "goaround",
                    x1 = tonumber(gx1), z1 = tonumber(gz1),
                    x2 = tonumber(gx2), z2 = tonumber(gz2), alt = tonumber(galt),
                }
            elseif wid or aid then
                local id, ty, altS, body =
                    wid or aid, wid and "goaround-wp" or "appr", walt or aalt, wseq or aseq
                local seq = {}
                for p in body:gmatch("[^,]+") do table.insert(seq, trim(p)) end
                if not lnd[id] then table.insert(order, id) end
                lnd[id] = {
                    name = id, type = ty, alt = tonumber(altS), seq = seq,
                }
            elseif name then
                if not lnd[name] then table.insert(order, name) end
                lnd[name] = {
                    name = name, type = ty:lower(), dir = dir:lower(),
                    alt = tonumber(alt), rwyLen = tonumber(rwy),
                }
            end
        end
    end
    return lnd, order
end

-- sites.db is not parsed here beyond counting its SITE= lines for the dashboard: the ATC
-- controller (#87) is its only reader and the authority on what a line means, and it shows
-- every line it cannot use on its own board. Same line shape it accepts.
local function countSites(body)
    local n = 0
    for line in (body or ""):gmatch("[^\r\n]+") do
        local t = trim(line)
        if t ~= "" and t:sub(1, 1) ~= ";" and t:match("^[%w_%-]+=.+$") then n = n + 1 end
    end
    return n
end

-- Serialisers. bodyPoints matches the FMS's own savePoints() layout exactly (POI block,
-- VOR block, each under its comment) so a file written here and one written by an aircraft
-- look the same.
local function bodyPoints()
    local pois, vors = {}, {}
    for _, name in ipairs(DB.pointOrder) do
        local p = DB.points[name]
        if p then table.insert(p.kind == "vor" and vors or pois, p) end
    end
    local out = {POINTS_HEADER_POI}
    for _, p in ipairs(pois) do
        table.insert(out, string.format("%s=%d,%d", p.name, p.x, p.z))
    end
    table.insert(out, POINTS_HEADER_VOR)
    for _, p in ipairs(vors) do
        table.insert(out, string.format("%s=%d,%d", p.name, p.x, p.z))
    end
    table.insert(out, "")
    return table.concat(out, "\n") .. "\n"
end

local function bodyCourses()
    local out = {COURSE_HEADER}
    for _, name in ipairs(DB.courseOrder) do
        local c = DB.courses[name]
        if c then
            if c.gapBefore then table.insert(out, "") end
            table.insert(out, name .. "=" .. table.concat(c.seq, ","))
        end
    end
    return table.concat(out, "\n") .. "\n"
end

local function bodyLanding()
    local out = {}
    for _, name in ipairs(DB.landOrder) do
        local l = DB.landing[name]
        if l and l.type == "goaround" then
            table.insert(out, string.format("%s=goaround,%d,%d,%d,%d,%d",
                l.name, l.x1, l.z1, l.x2, l.z2, l.alt))
        elseif l and (l.type == "goaround-wp" or l.type == "appr") then
            -- Both list shapes write out identically bar the keyword: id, altitude, fixes.
            table.insert(out, string.format("%s=%s,%d,%s",
                l.name, l.type, l.alt, table.concat(l.seq, ",")))
        elseif l then
            table.insert(out, string.format("%s=%s,%s,%d,%d", l.name, l.type, l.dir, l.alt, l.rwyLen))
        end
    end
    return LANDING_HEADER .. table.concat(out, "\n") .. "\n"
end

-- Read whatever is on disk and adopt it as the master. The RAW file text becomes the
-- payload (not a re-serialisation of it), so a hand-edited file with its own comments and
-- spacing reaches the aircraft byte-for-byte until the first edit made here.
local function loadAll()
    local bp = readFile(POINTS_FILE)  or ""
    local bc = readFile(COURSE_FILE)  or ""
    local bl = readFile(LANDING_FILE) or ""
    local bs = readFile(SITES_FILE)   or ""
    DB.points,  DB.pointOrder  = parsePoints(bp)
    DB.courses, DB.courseOrder = parseCourses(bc)
    DB.landing, DB.landOrder   = parseLanding(bl)
    DB.body.points, DB.body.courses, DB.body.landing = bp, bc, bl
    DB.body.sites, DB.siteCount = bs, countSites(bs)
    DB.stamp = stampOf(bp, bc, bl, bs)
    DB.rev = tonumber(trim(readFile(REV_FILE) or "")) or 1
end

-- Adopt whatever is on disk, if it differs from what is held in memory. NET.send calls this
-- before every sync, so a file edited outside this program -- which is the normal way these
-- three are edited, in the world save with the server left running -- reaches the fleet on
-- the next broadcast instead of waiting for a reboot or for someone to press R.
--
-- Three small file reads every BROADCAST_INTERVAL cost nothing on a computer with no control
-- loop, and the stamp comparison means the re-PARSE only happens when the content actually
-- changed. It also makes the post-edit send from applied() a no-op by construction: commit()
-- has just written exactly what is in memory, so disk and stamp agree.
--
-- IT REFUSES A PARTIAL DISK, WHERE loadAll() DOES NOT. loadAll reads a missing file as "",
-- because at boot that is honestly "no database yet"; here the same coercion would broadcast
-- an empty points.db over every aircraft's working copy the moment a file went unreadable or
-- was caught mid-write. So a file that has gone missing, or that reads back empty while we
-- hold a non-empty copy, keeps the copy already being served and raises a fault on the
-- dashboard instead. (Clients would refuse an empty points payload anyway -- 01-protocols
-- §7, the sanity rule -- but they would refuse the courses and landing that came with it
-- too, and the server would sit there insisting.) R is the deliberate override: doReload
-- goes through loadAll and adopts the disk whatever it says.
--
-- The revision is NOT bumped for an outside edit. rev counts saves made here and is for
-- display and ordering; what a client compares is the stamp, and that does change.
local function refreshFromDisk()
    local bp, bc, bl = readFile(POINTS_FILE), readFile(COURSE_FILE), readFile(LANDING_FILE)
    -- sites.db is OPTIONAL -- a fleet with no controlled sites has no file -- so its absence is
    -- only a fault when a non-empty copy is being served, by the same rule as below: an
    -- unreadable file mid-edit must not become "no sites" and drop every clearance.
    local bs = readFile(SITES_FILE)
    local bad = (not bp and POINTS_FILE) or (not bc and COURSE_FILE) or (not bl and LANDING_FILE)
             or (not bs and DB.body.sites ~= "" and SITES_FILE)
    if bad then
        DB.diskFault = bad .. " unreadable - serving the last good copy"
        return false
    end
    bs = bs or ""
    bad = (bp == "" and DB.body.points  ~= "" and POINTS_FILE)
       or (bc == "" and DB.body.courses ~= "" and COURSE_FILE)
       or (bl == "" and DB.body.landing ~= "" and LANDING_FILE)
       or (bs == "" and DB.body.sites   ~= "" and SITES_FILE)
    if bad then
        DB.diskFault = bad .. " read back empty - serving the last good copy"
        return false
    end
    DB.diskFault = nil

    local stamp = stampOf(bp, bc, bl, bs)
    if stamp == DB.stamp then return false end

    DB.points,  DB.pointOrder  = parsePoints(bp)
    DB.courses, DB.courseOrder = parseCourses(bc)
    DB.landing, DB.landOrder   = parseLanding(bl)
    DB.body.points, DB.body.courses, DB.body.landing = bp, bc, bl
    DB.body.sites, DB.siteCount = bs, countSites(bs)
    DB.stamp     = stamp
    DB.adopts    = DB.adopts + 1
    DB.lastAdopt = os.clock()
    return true
end

-- Re-serialise, write to disk, bump the revision, re-stamp. Every mutation funnels through
-- here, so there is exactly one place where the master can change.
local function commit()
    DB.body.points  = bodyPoints()
    DB.body.courses = bodyCourses()
    DB.body.landing = bodyLanding()
    writeFile(POINTS_FILE,  DB.body.points)
    writeFile(COURSE_FILE,  DB.body.courses)
    writeFile(LANDING_FILE, DB.body.landing)
    DB.rev = DB.rev + 1
    DB.stamp = stampOf(DB.body.points, DB.body.courses, DB.body.landing, DB.body.sites)
    writeFile(REV_FILE, tostring(DB.rev))
end

-- ---------------------------------------------------------------------------- queries

-- Which courses name this point. Deleting a point still used by a route would leave the
-- aircraft with a course it cannot fly, so this gates deletion (the FMS's pointInUseBy
-- rule, enforced at the master instead).
local function pointUsedBy(name)
    local used = {}
    for _, cname in ipairs(DB.courseOrder) do
        local c = DB.courses[cname]
        if c then
            for _, p in ipairs(c.seq) do
                if p == name then table.insert(used, cname); break end
            end
        end
    end
    return used
end

-- Waypoints a course names that do not exist. The FMS skips unknown names when it builds a
-- plan, so this is a silently-shortened route in the making -- flagged in the course list
-- and blocked at save time.
-- ...and which PATTERNS name it. Same question as pointUsedBy, asked of the other file: a
-- `goaround-wp` circuit and an `appr` approach are both waypoint lists, so deleting or renaming
-- a point out from under one silently shortens a procedure an aircraft is required to fly -- and
-- unlike a course, nobody is looking at it until the day it is needed. Tested on the presence of
-- a fix list rather than on the keyword, so a fifth list-shaped record is covered the day it is
-- added rather than the day someone remembers this function exists.
local function pointUsedByGa(name)
    local used = {}
    for _, lname in ipairs(DB.landOrder) do
        local l = DB.landing[lname]
        if l and l.seq then
            for _, p in ipairs(l.seq) do
                if p == name then table.insert(used, lname); break end
            end
        end
    end
    return used
end

-- What a name in a course's waypoint list is, if it is not a point: course.db lets a route
-- continue as another route, and lets it name a published APPROACH from landing.db -- see
-- autopilot.lua's expandCourseSeq, which actually flies both.
--   "course" -- `WAX-TMU=WAX,WAU-TMU` is "WAX, then fly WAU-TMU"
--   "appr"   -- `AGS-TMV=AGS,...,TMV_APPR4` is "...then arrive by TMV_APPR4"
-- A go-around id is deliberately NOT one of them: a missed-approach procedure is not a route,
-- and splicing one into an arrival would fly a lap of the aerodrome on the way in.
-- nil for anything else, which is a genuinely broken reference.
local function courseRefKind(name)
    if DB.courses[name] then return "course" end
    local l = DB.landing[name]
    if l and l.type == "appr" then return "appr" end
    return nil
end

-- Waypoints a course names that are neither a point nor one of the references above. Those
-- references are references, not broken waypoints, so flagging them BAD (which a points-only
-- check would) would paint every route built out of another red on this screen. Same shape as
-- patternUnknowns below, for landing.db's pattern-to-pattern references.
local function courseUnknowns(seq)
    local bad = {}
    for _, p in ipairs(seq) do
        if not DB.points[p] and not courseRefKind(p) then table.insert(bad, p) end
    end
    return bad
end

-- The same question for a PATTERN's fix list, where a name may legitimately be another
-- pattern's id: landing.db lets an approach continue as another approach, and name a go-around
-- as its missed approach, by writing that pattern's id in the list. Those are references, not
-- broken fixes, so flagging them BAD (which courseUnknowns would) would paint every published
-- approach red on this screen. A reference to something that exists as NEITHER is still bad.
local function patternUnknowns(seq)
    local bad = {}
    for _, p in ipairs(seq) do
        if not DB.points[p] and not DB.landing[p] then table.insert(bad, p) end
    end
    return bad
end

-- ---------------------------------------------------------------------------- mutators
-- Each returns ok, err and commits on success.

local function validatePointName(name)
    if name == "" then return false, "Name required" end
    if not name:match("^%u+$") then return false, "Letters A-Z only" end
    if #name ~= 3 and #name ~= 5 then return false, "3 letters = POI, 5 = VOR" end
    return true
end

local function savePoint(oldName, name, xs, zs)
    local ok, err = validatePointName(name)
    if not ok then return false, err end
    -- Integers only: the FMS's point pattern is %-?%d+ , so "12.5" would not parse there
    -- and the point would vanish from the aircraft's database without a word.
    local x, z = tonumber(xs), tonumber(zs)
    if not x or not z then return false, "X and Z must be numbers" end
    if x ~= math.floor(x) or z ~= math.floor(z) then return false, "X and Z must be whole" end
    if name ~= oldName and DB.points[name] then return false, name .. " already exists" end

    if oldName and name ~= oldName then
        -- A rename has to follow the point through every route, or those routes break.
        for i, n in ipairs(DB.pointOrder) do
            if n == oldName then DB.pointOrder[i] = name; break end
        end
        DB.points[oldName] = nil
        for _, cname in ipairs(DB.courseOrder) do
            local c = DB.courses[cname]
            for i, p in ipairs(c.seq) do
                if p == oldName then c.seq[i] = name end
            end
        end
        -- ... and through any PATTERN that names it -- go-around circuit or published approach --
        -- which includes the runway ids at the ends of one: that is how a procedure is matched to
        -- the runway it serves, so a rename that skipped them would leave the aerodrome with a
        -- go-around, or an arrival, belonging to nothing.
        for _, lname in ipairs(DB.landOrder) do
            local l = DB.landing[lname]
            if l and l.seq then
                for i, p in ipairs(l.seq) do
                    if p == oldName then l.seq[i] = name end
                end
            end
        end
        -- ... and through its landing entry, which is keyed by point name.
        local l = DB.landing[oldName]
        if l then
            DB.landing[oldName] = nil
            l.name = name
            DB.landing[name] = l
            for i, n in ipairs(DB.landOrder) do
                if n == oldName then DB.landOrder[i] = name; break end
            end
        end
    elseif not oldName then
        table.insert(DB.pointOrder, name)
    end

    DB.points[name] = {name = name, x = x, z = z, kind = (#name == 5) and "vor" or "poi"}
    commit()
    return true
end

local function deletePoint(name)
    local used = pointUsedBy(name)
    if #used > 0 then
        return false, "In use by " .. #used .. " course(s): " .. table.concat(used, ", ")
    end
    local usedGa = pointUsedByGa(name)
    if #usedGa > 0 then
        return false, "In use by pattern " .. table.concat(usedGa, ", ")
    end
    if DB.landing[name] then return false, "Delete its LANDING entry first" end
    DB.points[name] = nil
    for i, n in ipairs(DB.pointOrder) do
        if n == name then table.remove(DB.pointOrder, i); break end
    end
    commit()
    return true
end

local function saveCourse(oldName, name, seq)
    if name == "" then return false, "Name required" end
    if not name:match("^[%u%d%-]+$") then return false, "Use A-Z, 0-9, - only" end
    if #seq < 2 then return false, "Need at least 2 waypoints" end
    if name ~= oldName and DB.courses[name] then return false, name .. " already exists" end
    local bad = courseUnknowns(seq)
    if #bad > 0 then return false, "Unknown waypoint: " .. table.concat(bad, ", ") end

    local gap = false
    if oldName then
        gap = DB.courses[oldName] and DB.courses[oldName].gapBefore or false
        if name ~= oldName then
            DB.courses[oldName] = nil
            for i, n in ipairs(DB.courseOrder) do
                if n == oldName then DB.courseOrder[i] = name; break end
            end
        end
    else
        table.insert(DB.courseOrder, name)
    end
    DB.courses[name] = {name = name, seq = seq, gapBefore = gap}
    commit()
    return true
end

local function deleteCourse(name)
    DB.courses[name] = nil
    for i, n in ipairs(DB.courseOrder) do
        if n == name then table.remove(DB.courseOrder, i); break end
    end
    commit()
    return true
end

local LAND_TYPES = {"heli", "rwy"}
local LAND_DIRS  = {"n", "e", "s", "w", "x"}

local function saveLanding(oldName, name, ty, dir, alts, rwys)
    if name == "" then return false, "Site required" end
    -- A landing entry for a point that does not exist can never be reached: the FMS only
    -- consults landing.db for a point it is already flying to.
    if not DB.points[name] then return false, name .. " is not a point" end
    if name ~= oldName and DB.landing[name] then return false, name .. " already has an entry" end
    local alt, rwy = tonumber(alts), tonumber(rwys)
    if not alt or not rwy then return false, "ALT and RWY must be numbers" end
    if alt ~= math.floor(alt) or rwy ~= math.floor(rwy) then return false, "ALT and RWY must be whole" end

    if oldName and name ~= oldName then
        DB.landing[oldName] = nil
        for i, n in ipairs(DB.landOrder) do
            if n == oldName then DB.landOrder[i] = name; break end
        end
    elseif not oldName then
        table.insert(DB.landOrder, name)
    end
    DB.landing[name] = {name = name, type = ty, dir = dir, alt = alt, rwyLen = rwy}
    commit()
    return true
end

local function deleteLanding(name)
    DB.landing[name] = nil
    for i, n in ipairs(DB.landOrder) do
        if n == name then table.remove(DB.landOrder, i); break end
    end
    commit()
    return true
end

-- =====================================================================================
-- NETWORK
-- =====================================================================================

local NET = {
    nextTx  = 0,
    lastTx  = nil,
    txCount = 0,
    clients = {},   -- id -> {id, label, rev, stamp, at, kind}
}

function NET.message()
    return {
        t       = "sync",
        rev     = DB.rev,
        stamp   = DB.stamp,
        points  = DB.body.points,
        courses = DB.body.courses,
        landing = DB.body.landing,
        sites   = DB.body.sites,
        server  = os.getComputerLabel() or ("computer " .. os.getComputerID()),
    }
end

-- target=nil broadcasts and re-arms the periodic timer; a targeted reply to a pull does
-- not, so one aircraft rebooting cannot drag the fleet-wide cadence around.
--
-- The disk is re-read FIRST, on every send, pull replies included: what goes on the wire is
-- what is on this computer's disk at the moment it is sent, never a copy of it from earlier.
function NET.send(target)
    refreshFromDisk()
    local m = NET.message()
    if target then
        rednet.send(target, m, NAVDATA_PROTOCOL)
    else
        rednet.broadcast(m, NAVDATA_PROTOCOL)
        NET.nextTx = os.clock() + BROADCAST_INTERVAL
    end
    NET.lastTx  = os.clock()
    NET.txCount = NET.txCount + 1
end

function NET.note(id, msg, kind)
    local c = NET.clients[id] or {id = id}
    c.at   = os.clock()
    c.kind = kind
    if msg.who then c.label = msg.who end
    if kind == "ack" then
        -- An ack is a COMPLETE statement of what that aircraft holds, so take it whole: a
        -- field the client leaves out means it has none, not "keep the last value". A pull
        -- carries none of this and must only refresh the age.
        c.rev, c.stamp, c.held, c.screen = msg.rev, msg.stamp, msg.held, msg.screen
        c.fault = msg.fault
    end
    NET.clients[id] = c
end

-- True once this client has acknowledged the CURRENT content. Stamp, not rev: the stamp is
-- what the client actually compared before deciding to reload.
function NET.inSync(c) return c.stamp == DB.stamp end

function NET.roster()
    local t = {}
    for _, c in pairs(NET.clients) do table.insert(t, c) end
    table.sort(t, function(a, b) return a.id < b.id end)
    return t
end

function NET.loop()
    NET.send()
    while true do
        if os.clock() >= NET.nextTx then NET.send() end
        -- The 1 s timeout doubles as the dashboard's clock: the timer event it posts wakes
        -- the UI loop too, which is what makes the countdown and the client ages tick.
        local sender, msg = rednet.receive(NAVDATA_PROTOCOL, 1)
        if sender and type(msg) == "table" then
            if msg.t == "pull" then
                NET.note(sender, msg, "pull")
                NET.send(sender)
            elseif msg.t == "ack" then
                NET.note(sender, msg, "ack")
            end
        end
    end
end

-- =====================================================================================
-- UI PRIMITIVES
-- =====================================================================================
-- Keyboard first, mouse everywhere: every command on the help line has a key, and every
-- key has a clickable equivalent.
--
-- ESC IS DELIBERATELY UNUSED. In Minecraft, Escape closes the computer's GUI and the key
-- event never reaches the program, so "back" is BACKSPACE or the on-screen button, and a
-- form is cancelled by tabbing to CANCEL.
--
-- CHARACTER COMMANDS LIVE IN onChar, SPECIAL KEYS IN onKey. A letter press posts BOTH a
-- key and a char event; handling a letter in onKey and then landing on a screen with a
-- text field would type that same letter into the field. Keep the split.

local UI = {
    screen = "home", msg = nil, msgUntil = 0, msgCol = colours.lime,
    hits = {}, quit = false,
}

local function toast(text, col)
    UI.msg, UI.msgCol, UI.msgUntil = text, col or colours.lime, os.clock() + TOAST_TIME
end

local function clearHits() UI.hits = {} end

local function addHit(x1, y1, x2, y2, id)
    table.insert(UI.hits, {x1 = x1, y1 = y1, x2 = x2, y2 = y2, id = id})
end

local function hitTest(x, y)
    for i = #UI.hits, 1, -1 do
        local h = UI.hits[i]
        if x >= h.x1 and x <= h.x2 and y >= h.y1 and y <= h.y2 then return h.id end
    end
    return nil
end

local function put(x, y, text, fg, bg)
    term.setCursorPos(x, y)
    term.setTextColour(fg or colours.white)
    term.setBackgroundColour(bg or colours.black)
    term.write(tostring(text))
end

local function fillRow(y, w, bg)
    term.setBackgroundColour(bg)
    term.setCursorPos(1, y)
    term.write(string.rep(" ", w))
end

local function rule(y, w)
    put(1, y, string.rep("-", w), colours.brown, colours.black)
end

-- A labelled shortcut: "[B] BROADCAST", the bracketed character being the key, the whole
-- run being the hitbox. Returns the x to start the next one at.
local function keyBtn(x, y, key, label, id, col)
    put(x, y, "[", colours.brown)
    put(x + 1, y, key, col or colours.orange)
    put(x + 2, y, "]" .. label, colours.white)
    local wid = 3 + #label
    addHit(x, y, x + wid - 1, y, id)
    return x + wid + 1
end

local function clampScroll(sel, top, rows, n)
    if n == 0 then return 1, 1 end
    sel = math.max(1, math.min(n, sel))
    if sel < top then top = sel end
    if sel > top + rows - 1 then top = sel - rows + 1 end
    top = math.max(1, math.min(math.max(1, n - rows + 1), top))
    return sel, top
end

-- Modal yes/no. Runs its own event loop; NET.loop is a separate coroutine, so broadcasts
-- and pulls keep being served while a dialog is open.
local function confirm(title, message)
    while true do
        local w, h = term.getSize()
        clearHits()
        term.setBackgroundColour(colours.black)
        term.clear()
        put(2, 2, title, colours.orange)
        put(2, 4, message:sub(1, w - 2), colours.white)
        local bx = keyBtn(2, 6, "Y", " YES", "YES", colours.lime)
        keyBtn(bx, 6, "N", " NO", "NO", colours.orange)
        put(2, h - 1, "ENTER or Y = yes,  N or BACKSPACE = no", colours.brown)
        local ev = {os.pullEvent()}
        if ev[1] == "char" then
            if ev[2]:lower() == "y" then return true end
            if ev[2]:lower() == "n" then return false end
        elseif ev[1] == "key" then
            if ev[2] == keys.enter or ev[2] == keys.numPadEnter then return true end
            if ev[2] == keys.backspace then return false end
        elseif ev[1] == "mouse_click" then
            local id = hitTest(ev[3], ev[4])
            if id == "YES" then return true elseif id == "NO" then return false end
        end
    end
end

-- ---------------------------------------------------------------------------- fields
-- A text field carries its own caret, so a coordinate can be corrected in the middle
-- instead of retyped. charset is matched per typed character, so a field can only ever
-- hold characters its .db line is allowed to contain.

local function newField(label, value, max, charset, upper, hint)
    return {
        label = label, value = value or "", cur = #(value or "") + 1,
        max = max, charset = charset, upper = upper, hint = hint, kind = "text",
    }
end

local function newChoice(label, value, choices, hint)
    return {label = label, value = value, choices = choices, hint = hint, kind = "choice"}
end

local function fieldKey(f, k)
    if f.kind == "choice" then
        if k == keys.left or k == keys.right or k == keys.space then
            local i = 1
            for j, c in ipairs(f.choices) do if c == f.value then i = j end end
            if k == keys.left then i = (i - 2) % #f.choices + 1 else i = i % #f.choices + 1 end
            f.value = f.choices[i]
        end
        return
    end
    if k == keys.backspace then
        if f.cur > 1 then
            f.value = f.value:sub(1, f.cur - 2) .. f.value:sub(f.cur)
            f.cur = f.cur - 1
        end
    elseif k == keys.delete then
        f.value = f.value:sub(1, f.cur - 1) .. f.value:sub(f.cur + 1)
    elseif k == keys.left then
        f.cur = math.max(1, f.cur - 1)
    elseif k == keys.right then
        f.cur = math.min(#f.value + 1, f.cur + 1)
    elseif k == keys.home then
        f.cur = 1
    elseif k == keys["end"] then
        f.cur = #f.value + 1
    end
end

local function fieldChar(f, ch)
    if f.kind == "choice" then
        for _, c in ipairs(f.choices) do
            if c:sub(1, 1):lower() == ch:lower() then f.value = c end
        end
        return
    end
    if f.upper then ch = ch:upper() end
    if f.charset and not ch:match(f.charset) then return end
    if #f.value >= f.max then return end
    f.value = f.value:sub(1, f.cur - 1) .. ch .. f.value:sub(f.cur)
    f.cur = f.cur + 1
end

local function drawField(x, y, w, f, focused, id)
    put(x, y, f.label, focused and colours.orange or colours.white)
    local bx = x + 6
    local bg = focused and colours.grey or colours.black
    term.setBackgroundColour(bg)
    term.setCursorPos(bx, y)
    term.write(string.rep(" ", w))
    local shown = f.value
    if type(shown) == 'number' then shown = tostring(shown) end
    if #shown > w - 1 then shown = shown:sub(#shown - w + 2) end
    put(bx, y, shown, colours.lime, bg)
    if focused then
        local cx = bx + math.min(f.cur - 1, #shown)
        put(cx, y, (f.kind == "choice") and "\17" or "_", colours.white, bg)
    end
    addHit(bx, y, bx + w - 1, y, id)
    if f.hint then put(bx + w + 2, y, f.hint, colours.brown) end
end

-- =====================================================================================
-- SCREEN STATE
-- =====================================================================================

-- The one list browser, reused for all three kinds. Position survives a trip into an
-- editor and back; changing kind resets it.
local BR = {kind = nil, sel = 1, top = 1, filter = "", filtering = false}

-- The one form, used for points and landing sites. focus runs over the fields and then
-- over the actions, so SAVE/DELETE/CANCEL are reachable by TAB alone.
local FORM = {kind = nil, editing = nil, fields = {}, focus = 1, actions = {}}

-- The course editor: a name field plus a waypoint-sequence pane.
local CRS = {editing = nil, name = nil, seq = {}, pane = "name", sel = 1, top = 1}

-- The waypoint picker launched from the course editor.
local PICK = {sel = 1, top = 1, filter = "", filtering = false, mode = "insert"}

local function goHome() UI.screen = "home" end

local function openBrowse(kind)
    if BR.kind ~= kind then
        BR.kind, BR.sel, BR.top, BR.filter, BR.filtering = kind, 1, 1, "", false
    end
    UI.screen = "browse"
end

-- The list a browser is showing, after its filter. Shown in FILE order (POIs before VORs,
-- routes in the order they were written) -- the same order the FMS lists them in, so the
-- two screens read alike.
local function browseItems()
    local out = {}
    local f = BR.filter:upper()
    local order = (BR.kind == "points" and DB.pointOrder)
        or (BR.kind == "courses" and DB.courseOrder)
        or DB.landOrder
    for _, name in ipairs(order) do
        if f == "" or name:upper():find(f, 1, true) then table.insert(out, name) end
    end
    return out
end

local function openPointForm(name)
    local p = name and DB.points[name]
    FORM.kind, FORM.editing, FORM.focus = "point", name, 1
    FORM.fields = {
        newField("NAME", p and p.name or "", 5, "^%a$", true, "3=POI 5=VOR"),
        newField("X",    p and tostring(p.x) or "", 8, "^[%d%-]$", false, "whole"),
        newField("Z",    p and tostring(p.z) or "", 8, "^[%d%-]$", false, "whole"),
    }
    FORM.actions = name and {"SAVE", "DELETE", "CANCEL"} or {"SAVE", "CANCEL"}
    UI.screen = "form"
end

local function openLandForm(name)
    local l = name and DB.landing[name]
    -- A PATTERN -- a go-around leg, a go-around circuit or a published approach -- has no
    -- direction/length to edit and its own record shape; opening it in this form and saving
    -- would rewrite it as an ordinary site and lose the procedure. They are hand-maintained in
    -- the file. Keyword-tested, because that is the only thing all three share.
    if l and (l.type == "goaround" or l.type == "goaround-wp" or l.type == "appr") then
        toast("Patterns are edited in landing.db, not on this screen", colours.orange)
        return   -- UI.screen unchanged, so the browser stays put
    end
    FORM.kind, FORM.editing, FORM.focus = "landing", name, 1
    FORM.fields = {
        newField("SITE", l and l.name or "", 5, "^%a$", true, "existing point"),
        newChoice("TYPE", l and l.type or "heli", LAND_TYPES, "SPACE cycles"),
        newChoice("DIR",  l and l.dir or "n", LAND_DIRS, "nose points"),
        newField("ALT",  l and tostring(l.alt) or "", 6, "^[%d%-]$", false, "pad Y"),
        newField("RWY",  l and tostring(l.rwyLen) or "0", 6, "^[%d%-]$", false, "m, rwy only"),
    }
    FORM.actions = name and {"SAVE", "DELETE", "CANCEL"} or {"SAVE", "CANCEL"}
    UI.screen = "form"
end

local function openCourse(name)
    local c = name and DB.courses[name]
    CRS.editing = name
    CRS.name = newField("NAME", name or "", 16, "^[%w%-]$", true, nil)
    CRS.seq = {}
    if c then for _, p in ipairs(c.seq) do table.insert(CRS.seq, p) end end
    CRS.pane, CRS.sel, CRS.top = c and "seq" or "name", 1, 1
    UI.screen = "course"
end

-- Points first, then the REFERENCES a course entry may also be (courseRefKind): the other
-- courses, then the published approaches. Points lead because that is what almost every entry
-- is; the references are at the bottom of an unfiltered list and one keystroke of FIND away.
-- Offering them here at all is what makes `TMU-TMV=...,TMV_APPR4` authorable on the master
-- rather than only by hand-editing course.db and rebooting this computer.
local function pickItems()
    local out = {}
    local f = PICK.filter:upper()
    local function offer(name)
        if f == "" or name:upper():find(f, 1, true) then table.insert(out, name) end
    end
    for _, name in ipairs(DB.pointOrder) do offer(name) end
    for _, name in ipairs(DB.courseOrder) do
        -- ...but never the course being edited: a course referencing itself expands to nothing
        -- (the FMS skips a name already on its reference stack), so it is a silent way to
        -- shorten your own route.
        if name ~= CRS.editing then offer(name) end
    end
    for _, name in ipairs(DB.landOrder) do
        if DB.landing[name].type == "appr" then offer(name) end
    end
    return out
end

-- =====================================================================================
-- SCREENS -- draw
-- =====================================================================================

local function drawChrome(title, right)
    local w, h = term.getSize()
    term.setBackgroundColour(colours.black)
    term.clear()
    put(2, 1, title, colours.orange)
    if right then put(math.max(2, w - #right), 1, right, colours.brown) end
    rule(2, w)
    rule(h - 3, w)
    if UI.msg and os.clock() < UI.msgUntil then
        put(2, h, UI.msg:sub(1, w - 2), UI.msgCol)
    end
    return w, h
end

local function drawHome()
    local w, h = drawChrome("NAV DATA SERVER", "rev " .. DB.rev .. "  #" .. os.getComputerID())

    put(2, 4, string.format("POINTS %3d", #DB.pointOrder))
    put(16, 4, string.format("COURSES %3d", #DB.courseOrder))
    put(32, 4, string.format("LANDING %3d", #DB.landOrder))
    put(45, 4, string.format("SITES%2d", DB.siteCount))

    local due = math.max(0, math.ceil(NET.nextTx - os.clock()))
    put(2, 5, string.format("broadcast in %2ds  (every %ds, %d sent)",
        due, BROADCAST_INTERVAL, NET.txCount), colours.brown)

    -- Whether the disk is being picked up matters more than how often: a fault here means
    -- the fleet is being served a copy older than the files sitting next to this program.
    if DB.diskFault then
        put(2, 6, DB.diskFault:sub(1, w - 2), colours.red)
    else
        put(2, 6, string.format("disk re-read each broadcast  (%d outside edit%s adopted)",
            DB.adopts, DB.adopts == 1 and "" or "s"), colours.brown)
    end

    put(2, 7, "CLIENTS", colours.orange)
    local roster = NET.roster()
    if #roster == 0 then
        put(2, 8, "none heard yet - waiting for an autopilot", colours.brown)
    end
    for i, c in ipairs(roster) do
        local y = 7 + i
        if y <= h - 5 then
            local age = os.clock() - c.at
            local state, col
            if age > CLIENT_STALE then
                state, col = "SILENT", colours.brown
            elseif c.fault then
                -- It is receiving fine and cannot store what it receives. Almost always a
                -- CC computer over computer_space_limit (1 MB): clear old program versions
                -- off that computer. It keeps flying its last good database meanwhile.
                state, col = tostring(c.fault), colours.red
            elseif NET.inSync(c) then
                state, col = "IN SYNC", colours.lime
            elseif c.held then
                -- It HAS the update and is refusing to load it yet, because the pilot is on
                -- a screen that lists or edits nav data. Naming that screen is the whole
                -- point: otherwise this is indistinguishable from an aircraft that is not
                -- receiving at all, and the fix (back out to HOME) is not obvious.
                state, col = "HELD " .. tostring(c.screen or ""), colours.orange
            else
                state, col = "UPDATING", colours.orange
            end
            put(2, y, "#" .. c.id)
            put(6, y, tostring(c.label or "?"):sub(1, 12))
            put(19, y, "rev " .. tostring(c.rev or "?"), colours.brown)
            put(27, y, string.format("%3ds", math.floor(age)), colours.brown)
            put(32, y, state:sub(1, w - 32), col)
        end
    end

    local x = keyBtn(2, h - 2, "P", " POINTS", "P")
    x = keyBtn(x, h - 2, "C", " COURSES", "C")
    keyBtn(x, h - 2, "L", " LANDING", "L")
    x = keyBtn(2, h - 1, "B", " BROADCAST", "B", colours.lime)
    x = keyBtn(x, h - 1, "R", " RELOAD DISK", "R")
    keyBtn(x, h - 1, "Q", " QUIT", "Q")
end

-- One row of the browser, per kind. Kept in one function so the three lists line up in the
-- same columns and can be scanned the same way.
local function drawBrowseRow(y, name, selected)
    local w = term.getSize()
    local bg = selected and colours.grey or colours.black
    fillRow(y, w, bg)
    put(2, y, selected and ">" or " ", colours.orange, bg)
    put(4, y, name, colours.white, bg)

    if BR.kind == "points" then
        local p = DB.points[name]
        if p then
            put(11, y, string.format("%6d,%6d", p.x, p.z), colours.lime, bg)
            put(26, y, p.kind == "vor" and "VOR" or "POI", colours.brown, bg)
            local l = DB.landing[name]
            if l then put(31, y, l.type:upper() .. " " .. l.dir:upper(), colours.orange, bg) end
            local n = #pointUsedBy(name)
            put(41, y, n > 0 and (n .. " crs") or "-", colours.brown, bg)
        end
    elseif BR.kind == "courses" then
        local c = DB.courses[name]
        if c then
            put(21, y, string.format("%2d wpt", #c.seq), colours.lime, bg)
            put(29, y, (c.seq[1] or "?") .. ">" .. (c.seq[#c.seq] or "?"), colours.white, bg)
            if #courseUnknowns(c.seq) > 0 then put(43, y, "BAD REF", colours.red, bg) end
        end
    else
        local l = DB.landing[name]
        if l then
            if l.type == "goaround" then
                put(11, y, l.type, colours.cyan, bg)
                put(20, y, string.format("%d,%d > %d,%d", l.x1, l.z1, l.x2, l.z2), colours.white, bg)
                put(43, y, "alt " .. l.alt, colours.white, bg)
            elseif l.type == "goaround-wp" or l.type == "appr" then
                -- The site a procedure serves is the LAST name in its list -- a circuit starts
                -- and ends at its runway, an approach ends at the site it arrives into -- and
                -- that is the field worth showing: it is what an FMS matches on. An approach is
                -- drawn in the arrival colour so the two are not read as the same thing at a
                -- glance; the last name of an approach may itself be a reference, in which case
                -- the site is whatever that pattern ends at and only the FMS resolves it.
                local isAppr = (l.type == "appr")
                put(11, y, l.type, isAppr and colours.lime or colours.cyan, bg)
                put(23, y, string.format("%2d wpt", #l.seq), colours.lime, bg)
                put(30, y, (isAppr and "to  " or "rwy ") .. tostring(l.seq[#l.seq]),
                    colours.white, bg)
                put(41, y, "min " .. l.alt, colours.white, bg)
                if #patternUnknowns(l.seq) > 0 then put(47, y, "BAD", colours.red, bg) end
            else
                put(11, y, l.type, colours.lime, bg)
                if tonumber(l.dir) then
                    put(18, y, l.dir, colours.white, bg)
                else
                    put(18, y, "dir " .. l.dir, colours.white, bg)
                end
                put(27, y, "alt " .. l.alt, colours.white, bg)
                if l.type == "rwy" then put(36, y, l.rwyLen .. "m", colours.white, bg) end
                if not DB.points[name] then put(43, y, "NO PT", colours.red, bg) end
            end
        end
    end
    addHit(1, y, w, y, "ROW:" .. name)
end

local function drawBrowse()
    local title = (BR.kind == "points" and "POINTS")
        or (BR.kind == "courses" and "COURSES") or "LANDING SITES"
    local items = browseItems()
    local w, h = drawChrome(title, "rev " .. DB.rev)

    local rows = h - 7
    BR.sel, BR.top = clampScroll(BR.sel, BR.top, rows, #items)

    put(2, 3, "FIND", BR.filtering and colours.orange or colours.brown)
    if BR.filtering then
        put(7, 3, BR.filter .. "_", colours.lime)
    else
        put(7, 3, BR.filter ~= "" and BR.filter or "(press / to search)",
            BR.filter ~= "" and colours.lime or colours.brown)
    end
    addHit(2, 3, w - 12, 3, "FILTER")
    put(w - 9, 3, string.format("%3d shown", #items), colours.brown)

    for i = 0, rows - 1 do
        local idx = BR.top + i
        if items[idx] then drawBrowseRow(4 + i, items[idx], idx == BR.sel) end
    end
    if #items == 0 then
        put(4, 4, BR.filter ~= "" and "nothing matches that" or "empty", colours.brown)
    end

    local x = keyBtn(2, h - 2, "E", " EDIT", "EDIT", colours.lime)
    x = keyBtn(x, h - 2, "N", " NEW", "NEW", colours.lime)
    x = keyBtn(x, h - 2, "D", " DELETE", "DELETE")
    keyBtn(x, h - 2, "/", " FIND", "FILTER")
    put(2, h - 1, "\24\25 select   ENTER edit   BACKSPACE back", colours.brown)
    addHit(2, h - 1, 20, h - 1, "NOP")
end

local function drawForm()
    local title = (FORM.kind == "point")
        and (FORM.editing and ("EDIT POINT " .. FORM.editing) or "NEW POINT")
        or (FORM.editing and ("EDIT LANDING " .. FORM.editing) or "NEW LANDING SITE")
    local w, h = drawChrome(title, "rev " .. DB.rev)

    for i, f in ipairs(FORM.fields) do
        drawField(3, 3 + (i - 1) * 2, 14, f, FORM.focus == i, "F:" .. i)
    end

    -- Consequences of this edit, shown while it is being made rather than after the fact.
    local y = 3 + #FORM.fields * 2
    if FORM.kind == "point" and FORM.editing then
        local used = pointUsedBy(FORM.editing)
        if #used > 0 then
            put(3, y, ("in " .. #used .. " course(s): " .. table.concat(used, ", ")):sub(1, w - 4),
                colours.brown)
            put(3, y + 1, "renaming updates them all", colours.brown)
        else
            put(3, y, "not used by any course", colours.brown)
        end
        if DB.landing[FORM.editing] then
            put(3, y + 2, "has a landing entry - it follows a rename", colours.orange)
        end
        local ga = pointUsedByGa(FORM.editing)
        if #ga > 0 then
            put(3, y + 3, ("in go-around " .. table.concat(ga, ", ")):sub(1, w - 4), colours.orange)
        end
    elseif FORM.kind == "landing" then
        local n = FORM.fields[1].value
        if n ~= "" and not DB.points[n] then
            put(3, y, n .. " is not a point - add it first", colours.red)
        elseif FORM.fields[3].value == "x" then
            put(3, y, "dir x = touch down on any heading", colours.brown)
        else
            put(3, y, "nose points " .. FORM.fields[3].value:upper() .. " on touchdown", colours.brown)
        end
    end

    local bx = 3
    for i, a in ipairs(FORM.actions) do
        local fi = #FORM.fields + i
        local col = (a == "SAVE" and colours.lime) or (a == "DELETE" and colours.red) or colours.brown
        local lbl = " " .. a .. " "
        term.setBackgroundColour(FORM.focus == fi and col or colours.black)
        term.setTextColour(FORM.focus == fi and colours.black or col)
        term.setCursorPos(bx, h - 5)
        term.write(lbl)
        addHit(bx, h - 5, bx + #lbl - 1, h - 5, "A:" .. a)
        bx = bx + #lbl + 2
    end

    put(2, h - 2, "TAB / \24\25 field    type to edit    \27\26 caret", colours.brown)
    put(2, h - 1, "ENTER on a field = save,  or TAB to a button", colours.brown)
end

local function drawCourse()
    local w, h = drawChrome(CRS.editing and ("EDIT COURSE " .. CRS.editing) or "NEW COURSE",
        "rev " .. DB.rev)

    drawField(3, 3, 18, CRS.name, CRS.pane == "name", "F:NAME")
    put(30, 3, string.format("%d waypoints", #CRS.seq), colours.brown)

    local rows = h - 8
    if CRS.pane == "seq" then
        CRS.sel, CRS.top = clampScroll(CRS.sel, CRS.top, rows, #CRS.seq)
    end
    for i = 0, rows - 1 do
        local idx = CRS.top + i
        local name = CRS.seq[idx]
        local y = 5 + i
        if name then
            local selected = (CRS.pane == "seq" and idx == CRS.sel)
            local bg = selected and colours.grey or colours.black
            fillRow(y, w, bg)
            put(2, y, selected and ">" or " ", colours.orange, bg)
            put(4, y, string.format("%2d", idx), colours.brown, bg)
            local known = DB.points[name]
            -- ...or a REFERENCE, which is a legal entry and not a broken one (courseRefKind).
            -- Painting one "unknown point" in red is how a hand-written `WAU-TMU` or
            -- `TMV_APPR4` gets "fixed" out of a route by someone tidying this screen up.
            local ref = not known and courseRefKind(name)
            put(8, y, name, (known and colours.white) or (ref and colours.orange) or colours.red, bg)
            if known then
                put(15, y, string.format("%6d,%6d", known.x, known.z), colours.lime, bg)
                put(30, y, known.kind == "vor" and "VOR" or "POI", colours.brown, bg)
                if DB.landing[name] then
                    put(35, y, "<" .. DB.landing[name].type .. ">", colours.orange, bg)
                end
            elseif ref == "course" then
                local n = #(DB.courses[name].seq or {})
                put(15, y, string.format("course, %d fixes", n), colours.orange, bg)
            elseif ref == "appr" then
                local l = DB.landing[name]
                put(15, y, string.format("approach, min %d", l.alt or 0), colours.orange, bg)
            else
                put(15, y, "unknown point", colours.red, bg)
            end
            addHit(1, y, w, y, "SEQ:" .. idx)
        end
    end
    if #CRS.seq == 0 then put(4, 5, "empty - press A to append a waypoint", colours.brown) end

    if CRS.pane == "name" then
        put(2, h - 2, "type the name    TAB to the waypoint list", colours.brown)
        put(2, h - 1, "ENTER = save     BACKSPACE = back", colours.brown)
    else
        local x = keyBtn(2, h - 2, "A", " APPEND", "APPEND", colours.lime)
        x = keyBtn(x, h - 2, "I", " INSERT", "INSERT", colours.lime)
        x = keyBtn(x, h - 2, "X", " REMOVE", "REMOVE")
        keyBtn(x, h - 2, "V", " REVERSE", "REVERSE")
        x = keyBtn(2, h - 1, "S", " SAVE", "SAVE", colours.lime)
        x = keyBtn(x, h - 1, "T", " NAME", "TABNAME")
        put(x, h - 1, "< > move   BACKSPACE back", colours.brown)
    end
end

local function drawPick()
    local items = pickItems()
    local w, h = drawChrome("ADD WAYPOINT " .. (PICK.mode == "insert" and "(insert)" or "(append)"),
        (CRS.name and CRS.name.value ~= "" and CRS.name.value) or "new course")

    local rows = h - 7
    PICK.sel, PICK.top = clampScroll(PICK.sel, PICK.top, rows, #items)

    put(2, 3, "FIND", colours.orange)
    put(7, 3, PICK.filter .. "_", colours.lime)
    put(w - 9, 3, string.format("%3d shown", #items), colours.brown)

    for i = 0, rows - 1 do
        local idx = PICK.top + i
        local name = items[idx]
        local y = 4 + i
        if name then
            local p = DB.points[name]
            local selected = (idx == PICK.sel)
            local bg = selected and colours.grey or colours.black
            fillRow(y, w, bg)
            put(2, y, selected and ">" or " ", colours.orange, bg)
            put(4, y, name, p and colours.white or colours.orange, bg)
            if p then
                put(14, y, string.format("%6d,%6d", p.x, p.z), colours.lime, bg)
                put(29, y, p.kind == "vor" and "VOR" or "POI", colours.brown, bg)
                if DB.landing[name] then put(34, y, "<" .. DB.landing[name].type .. ">", colours.orange, bg) end
            elseif DB.courses[name] then
                put(14, y, string.format("course, %d fixes", #(DB.courses[name].seq or {})),
                    colours.brown, bg)
            else
                put(14, y, string.format("approach, min %d", DB.landing[name].alt or 0),
                    colours.brown, bg)
            end
            addHit(1, y, w, y, "ROW:" .. name)
        end
    end
    if #items == 0 then put(4, 4, "nothing matches that", colours.brown) end

    put(2, h - 2, "just type to narrow    \24\25 select", colours.brown)
    put(2, h - 1, "ENTER = add it         BACKSPACE = back", colours.brown)
    put(30, h - 1, "orange = a reference, not a fix", colours.brown)
end

local drawFns = {
    home = drawHome, browse = drawBrowse, form = drawForm,
    course = drawCourse, pick = drawPick,
}

-- =====================================================================================
-- SCREENS -- actions
-- =====================================================================================

-- Every mutator returns ok,err; on success the change goes out at once rather than waiting
-- up to BROADCAST_INTERVAL, which is what makes an edit feel live.
local function applied(ok, err, what)
    if ok then
        NET.send()
        toast(what .. " saved - rev " .. DB.rev .. " broadcast")
        return true
    end
    toast(err or "failed", colours.red)
    return false
end

local function doBroadcast()
    NET.send()
    toast("broadcast rev " .. DB.rev .. " to all aircraft")
end

local function doReload()
    loadAll()
    NET.send()
    toast("reloaded from disk, broadcast rev " .. DB.rev)
end

local function homeInput(id)
    if id == "P" then openBrowse("points")
    elseif id == "C" then openBrowse("courses")
    elseif id == "L" then openBrowse("landing")
    elseif id == "B" then doBroadcast()
    elseif id == "R" then doReload()
    elseif id == "Q" then
        if confirm("QUIT", "Stop the server? Aircraft keep their last copy.") then
            UI.quit = true
        end
    end
end

local function browseOpen(name)
    if BR.kind == "points" then openPointForm(name)
    elseif BR.kind == "courses" then openCourse(name)
    else openLandForm(name) end
end

local function browseNew()
    if BR.kind == "points" then openPointForm(nil)
    elseif BR.kind == "courses" then openCourse(nil)
    else openLandForm(nil) end
end

local function browseDelete(name)
    if not name then return end
    if BR.kind == "points" then
        if confirm("DELETE POINT", "Remove " .. name .. " from the database?") then
            local ok, err = deletePoint(name)
            applied(ok, err, "point " .. name)
        end
    elseif BR.kind == "courses" then
        if confirm("DELETE COURSE", "Remove route " .. name .. "?") then
            local ok, err = deleteCourse(name)
            applied(ok, err, "course " .. name)
        end
    else
        if confirm("DELETE LANDING", name .. " will no longer auto-land.") then
            local ok, err = deleteLanding(name)
            applied(ok, err, "landing " .. name)
        end
    end
end

local function browseInput(id, items)
    local name = items[BR.sel]
    if id == "EDIT" then
        if name then browseOpen(name) end
    elseif id == "NEW" then browseNew()
    elseif id == "DELETE" then browseDelete(name)
    elseif id == "FILTER" then BR.filtering, BR.filter = true, ""
    elseif id == "BACK" then goHome()
    end
end

local function formSave()
    local f = FORM.fields
    if FORM.kind == "point" then
        local ok, err = savePoint(FORM.editing, f[1].value, f[2].value, f[3].value)
        if applied(ok, err, "point " .. f[1].value) then openBrowse("points") end
    else
        local ok, err = saveLanding(FORM.editing, f[1].value, f[2].value, f[3].value,
            f[4].value, f[5].value)
        if applied(ok, err, "landing " .. f[1].value) then openBrowse("landing") end
    end
end

local function formDelete()
    if not FORM.editing then return end
    if FORM.kind == "point" then
        if confirm("DELETE POINT", "Remove " .. FORM.editing .. " from the database?") then
            local ok, err = deletePoint(FORM.editing)
            if applied(ok, err, "point " .. FORM.editing) then openBrowse("points") end
        end
    else
        if confirm("DELETE LANDING", FORM.editing .. " will no longer auto-land.") then
            local ok, err = deleteLanding(FORM.editing)
            if applied(ok, err, "landing " .. FORM.editing) then openBrowse("landing") end
        end
    end
end

local function formInput(id)
    if id == "A:SAVE" then formSave()
    elseif id == "A:DELETE" then formDelete()
    elseif id == "A:CANCEL" then openBrowse(FORM.kind == "point" and "points" or "landing")
    elseif id and id:sub(1, 2) == "F:" then
        FORM.focus = tonumber(id:sub(3)) or FORM.focus
    end
end

local function courseSave()
    local seq = {}
    for _, p in ipairs(CRS.seq) do table.insert(seq, p) end
    local ok, err = saveCourse(CRS.editing, CRS.name.value, seq)
    if applied(ok, err, "course " .. CRS.name.value) then openBrowse("courses") end
end

local function courseInput(id)
    if id == "SAVE" then courseSave()
    elseif id == "BACK" then openBrowse("courses")
    elseif id == "TABNAME" or id == "F:NAME" then CRS.pane = "name"
    elseif id == "APPEND" or id == "INSERT" then
        PICK.mode = (id == "APPEND") and "append" or "insert"
        PICK.sel, PICK.top, PICK.filter, PICK.filtering = 1, 1, "", true
        UI.screen = "pick"
    elseif id == "REMOVE" then
        if CRS.seq[CRS.sel] then table.remove(CRS.seq, CRS.sel) end
    elseif id == "REVERSE" then
        local r = {}
        for i = #CRS.seq, 1, -1 do table.insert(r, CRS.seq[i]) end
        CRS.seq = r
        toast("sequence reversed - give it the reverse name too")
    elseif id and id:sub(1, 4) == "SEQ:" then
        CRS.pane, CRS.sel = "seq", tonumber(id:sub(5)) or CRS.sel
    end
end

local function pickTake(name)
    if not name then return end
    if PICK.mode == "append" then
        table.insert(CRS.seq, name)
        CRS.sel = #CRS.seq
    else
        table.insert(CRS.seq, CRS.sel, name)
    end
    CRS.pane = "seq"
    UI.screen = "course"
end

-- =====================================================================================
-- EVENT LOOP
-- =====================================================================================
-- List navigation (arrows / page / home / end / wheel) is shared, so all three browsers
-- and the course sequence behave identically.

local function listNav(st, k, n, rows)
    if k == keys.up then st.sel = st.sel - 1
    elseif k == keys.down then st.sel = st.sel + 1
    elseif k == keys.pageUp then st.sel = st.sel - rows
    elseif k == keys.pageDown then st.sel = st.sel + rows
    elseif k == keys.home then st.sel = 1
    elseif k == keys["end"] then st.sel = n
    else return false end
    st.sel = math.max(1, math.min(math.max(1, n), st.sel))
    return true
end

local function onKey(k)
    local _, h = term.getSize()
    local rows = h - 7
    local s = UI.screen

    if s == "browse" then
        local items = browseItems()
        if listNav(BR, k, #items, rows) then return end
        if k == keys.enter or k == keys.numPadEnter then
            if BR.filtering then BR.filtering = false else browseInput("EDIT", items) end
        elseif k == keys.backspace then
            if BR.filtering then
                BR.filter = BR.filter:sub(1, -2)
                if BR.filter == "" then BR.filtering = false end
            else
                goHome()
            end
        elseif k == keys.delete then
            browseInput("DELETE", items)
        end

    elseif s == "form" then
        local n = #FORM.fields + #FORM.actions
        local f = FORM.fields[FORM.focus]
        if k == keys.tab or k == keys.down then
            FORM.focus = FORM.focus % n + 1
        elseif k == keys.up then
            FORM.focus = (FORM.focus - 2) % n + 1
        elseif k == keys.enter or k == keys.numPadEnter then
            local a = FORM.actions[FORM.focus - #FORM.fields]
            if a then formInput("A:" .. a) else formSave() end
        elseif f then
            fieldKey(f, k)
        elseif k == keys.backspace then
            formInput("A:CANCEL")
        end

    elseif s == "course" then
        if CRS.pane == "name" then
            if k == keys.tab or k == keys.down then CRS.pane = "seq"
            elseif k == keys.enter or k == keys.numPadEnter then courseSave()
            elseif k == keys.backspace and CRS.name.value == "" then courseInput("BACK")
            else fieldKey(CRS.name, k) end
        else
            if listNav(CRS, k, #CRS.seq, h - 8) then return end
            if k == keys.tab then CRS.pane = "name"
            elseif k == keys.enter or k == keys.numPadEnter then courseSave()
            elseif k == keys.backspace then courseInput("BACK")
            elseif k == keys.delete then courseInput("REMOVE") end
        end

    elseif s == "pick" then
        local items = pickItems()
        if listNav(PICK, k, #items, rows) then return end
        if k == keys.enter or k == keys.numPadEnter then
            pickTake(items[PICK.sel])
        elseif k == keys.backspace then
            if PICK.filter ~= "" then
                PICK.filter = PICK.filter:sub(1, -2)
                PICK.sel, PICK.top = 1, 1
            else
                UI.screen = "course"
            end
        end
    end
end

local function onChar(ch)
    local s = UI.screen

    if s == "home" then
        homeInput(ch:upper())

    elseif s == "browse" then
        if BR.filtering then
            BR.filter = BR.filter .. ch:upper()
            BR.sel, BR.top = 1, 1
        elseif ch == "/" then
            BR.filtering, BR.filter = true, ""
        else
            local c = ch:upper()
            if c == "E" or c == "N" or c == "D" then
                browseInput(c == "E" and "EDIT" or c == "N" and "NEW" or "DELETE", browseItems())
            elseif c == "Q" then
                goHome()
            end
        end

    elseif s == "form" then
        local f = FORM.fields[FORM.focus]
        if f then fieldChar(f, ch) end

    elseif s == "course" then
        if CRS.pane == "name" then
            fieldChar(CRS.name, ch)
        else
            local c = ch:upper()
            if c == "A" then courseInput("APPEND")
            elseif c == "I" then courseInput("INSERT")
            elseif c == "X" then courseInput("REMOVE")
            elseif c == "S" then courseInput("SAVE")
            elseif c == "V" then courseInput("REVERSE")
            elseif c == "T" then courseInput("TABNAME")
            elseif ch == "<" or ch == "," then
                -- Move the selected waypoint one place earlier; the cursor follows it, so
                -- repeating the key walks a waypoint up the route.
                if CRS.sel > 1 then
                    CRS.seq[CRS.sel], CRS.seq[CRS.sel - 1] = CRS.seq[CRS.sel - 1], CRS.seq[CRS.sel]
                    CRS.sel = CRS.sel - 1
                end
            elseif ch == ">" or ch == "." then
                if CRS.sel < #CRS.seq then
                    CRS.seq[CRS.sel], CRS.seq[CRS.sel + 1] = CRS.seq[CRS.sel + 1], CRS.seq[CRS.sel]
                    CRS.sel = CRS.sel + 1
                end
            end
        end

    elseif s == "pick" then
        -- Typing always narrows here: the picker exists to find one point fast.
        PICK.filter = PICK.filter .. ch:upper()
        PICK.sel, PICK.top = 1, 1
    end
end

local function onClick(x, y)
    local id = hitTest(x, y)
    local s = UI.screen

    if s == "home" then
        if id then homeInput(id) end

    elseif s == "browse" then
        local items = browseItems()
        if id and id:sub(1, 4) == "ROW:" then
            local name = id:sub(5)
            -- First click selects, a second click on the same row opens it. CC has no
            -- double-click event, and this is the closest thing that still lets you browse
            -- with the mouse without opening things by accident.
            if items[BR.sel] == name then
                browseOpen(name)
            else
                for i, n in ipairs(items) do if n == name then BR.sel = i end end
            end
        elseif id then
            browseInput(id, items)
        end

    elseif s == "form" then
        if id then formInput(id) end

    elseif s == "course" then
        if id then courseInput(id) end

    elseif s == "pick" then
        local items = pickItems()
        if id and id:sub(1, 4) == "ROW:" then
            local name = id:sub(5)
            if items[PICK.sel] == name then
                pickTake(name)
            else
                for i, n in ipairs(items) do if n == name then PICK.sel = i end end
            end
        end
    end
end

local function onScroll(dir)
    local st = (UI.screen == "browse" and BR)
        or (UI.screen == "pick" and PICK)
        or (UI.screen == "course" and CRS.pane == "seq" and CRS)
    if st then
        st.sel = st.sel + dir
        st.top = math.max(1, st.top + dir)
    end
end

local function uiLoop()
    while not UI.quit do
        clearHits()
        local ok, err = pcall(drawFns[UI.screen])
        if not ok then
            term.setBackgroundColour(colours.black)
            term.clear()
            put(2, 2, "draw error: " .. tostring(err), colours.red)
        end
        local ev = {os.pullEvent()}
        if ev[1] == "key" then onKey(ev[2])
        elseif ev[1] == "char" then onChar(ev[2])
        elseif ev[1] == "mouse_click" then onClick(ev[3], ev[4])
        elseif ev[1] == "mouse_scroll" then onScroll(ev[2])
        end
    end
end

-- =====================================================================================
-- MAIN
-- =====================================================================================

local modem = peripheral.find("modem")
if not modem then error("No modem attached - the server has nothing to broadcast on") end
if not rednet.isOpen(peripheral.getName(modem)) then
    rednet.open(peripheral.getName(modem))
end

loadAll()

if term.isColour and term.isColour() then
    term.setPaletteColour(colours.orange, 0xffbb00)
    term.setPaletteColour(colours.brown, 0x887766)
    term.setPaletteColour(colours.green, 0x116611)
    term.setPaletteColour(colours.grey, 0x333333)
end

toast("loaded rev " .. DB.rev .. " - " .. #DB.pointOrder .. " points, "
    .. #DB.courseOrder .. " courses, " .. #DB.landOrder .. " sites")

local ok, err = pcall(function()
    parallel.waitForAny(uiLoop, NET.loop)
end)

term.setBackgroundColour(colours.black)
term.setTextColour(colours.white)
term.clear()
term.setCursorPos(1, 1)
if not ok then
    print("nav update server error: " .. tostring(err))
else
    print("nav update server stopped at rev " .. DB.rev)
end
