-- ATC CONTROLLER -- computer #87
-- =====================================================================================
-- A crude automated approach controller. It knows which landing facilities belong together
-- (sites.db: a SITE is an aerodrome, its FACILITIES the pads and runways on it), watches who
-- is sitting on each one (the responders' own reports on atc_broadcast), and answers every
-- aircraft inbound to a facility with one of four words:
--
--   clr     -- cleared to land where you asked
--   divert  -- that one is taken; land on THIS one instead (another facility of the same site)
--   hold    -- nothing on the site is free for you; stop where you are and keep asking
--   none    -- that facility is in no site: uncontrolled, land as you like
--
-- WHAT IT DOES NOT DO. It never names a heading, an altitude, a route or a speed. A divert is
-- a facility NAME, and the FMS plans its own way there with the machinery a pilot's DIRECT-TO
-- LAND already uses (published approach and all); a hold is a single word, and how an airframe
-- holds -- stop and hover, or orbit -- is that airframe's apconf.lua. Separation in the air is
-- not attempted: this decides who may use which piece of GROUND, and nothing else.
--
-- FAIL SILENT. Every FMS lands exactly as it did before this computer existed unless it has
-- been told otherwise, and an FMS whose controller goes quiet changes nothing it was doing --
-- so the worst this station can do by being switched off is nothing at all.
--
-- PROTOCOL "aer_atc" (fleet-wide, CFG.LINKS.CTL)
--   FMS -> here   { t="req", cs, fac, x, y, z, wide, rwyonly, stage }   once a second, inbound
--   here -> FMS   { t="clr"|"divert"|"hold"|"none", cs, fac, to, why, q }   the answer, sent
--                 straight back to the asking computer. `fac` echoes what was asked, so an
--                 answer that arrives after the aircraft has already changed its mind is
--                 recognisably about something else; `to` is the facility assigned; `why` is
--                 one line for the pilot; `q` is the place in the hold queue.
--
-- HOW A FACILITY IS ALLOCATED (CTL.allocate, run on every request and twice a second):
--   * standing clearances are kept first, so nobody earlier in the queue can take a runway
--     out from under an aircraft already cleared onto it -- unless somebody else is now
--     SITTING on it, which no clearance survives;
--   * then everyone else, first come first served: the facility asked for if it is usable,
--     otherwise the first usable one of the same site in sites.db's order, otherwise hold.
--   "Usable" = not refused by a remark (narrow vs wide), a runway if the aircraft can land on
--   nothing else (rwyonly -- a FIXEDWING must never be diverted onto a pad), nobody but the
--   asker on it or on any
--   facility whose footprint overlaps it (see CFG.OCC), and not cleared to anybody else.
--
-- Everything tunable is in atcctlconf.lua beside this file. See its header.
-- =====================================================================================

-- ============================================================================
-- CONFIGURATION
-- ============================================================================
-- Scoped inside the do-block so the load temporaries do not sit in top-level registers --
-- the house style (see apconf.lua's note on Cobalt's 200-local ceiling).
local CFG
do
    local ok, res = pcall(require, "atcctlconf")
    if not ok then
        error("Cannot load atcctlconf.lua (the controller configuration): " .. tostring(res)
            .. "\nIt must sit beside atcctl.lua and end with 'return CFG'.", 0)
    end
    if type(res) ~= "table" then
        error("atcctlconf.lua did not return the CFG table (got " .. type(res) .. ").", 0)
    end
    for _, k in ipairs({ "LINKS", "POINTS_FILE", "LANDING_FILE", "SITES_FILE", "PARK_FILE",
            "NAVDATA", "REQ", "TRACK", "OCC", "REMARKS", "REDRAW", "TEXT_SCALE" }) do
        if res[k] == nil then
            error("atcctlconf.lua is missing CFG." .. k .. " -- the file looks incomplete.", 0)
        end
    end
    for t, ks in pairs({ LINKS = {"ATC", "NAVDATA", "CTL"},
            NAVDATA = {"ENABLE", "PULL_EVERY", "STALE_AFTER", "TMP"},
            REQ = {"STALE"}, TRACK = {"STALE", "DROP_AFTER"},
            OCC = {"PAD_R", "RWY_HALF_W", "RWY_END_M", "AGL_MAX", "BELOW_MAX", "FREE_AFTER",
                   "PARK_GS", "PARK_AFTER"},
            REMARKS = {} }) do
        if type(res[t]) ~= "table" then
            error("atcctlconf.lua's CFG." .. t .. " is not a table.", 0)
        end
        for _, k in ipairs(ks) do
            if res[t][k] == nil then
                error("atcctlconf.lua's CFG." .. t .. " is missing " .. k .. ".", 0)
            end
        end
    end
    CFG = res
end
local LINKS, OCCCFG = CFG.LINKS, CFG.OCC

-- ============================================================================
-- HELPERS
-- ============================================================================

local function trim(s) return (s:gsub("^%s*(.-)%s*$", "%1")) end

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

-- Landing directions exactly as autopilot.lua resolves them (resolveDirVec there), so the
-- strip this station calls occupied is the strip the aircraft lands on: the points.db
-- position is the THRESHOLD, and the runway runs rwyLen metres from it along this vector.
-- 'x' has no axis, and the FMS flies such a site as a pad -- so does this.
local DIR_VECTORS = { n = {0, -1}, e = {1, 0}, s = {0, 1}, w = {-1, 0} }
local function resolveDirVec(query)
    if query == 'x' or query == 'X' then return nil end
    local v = DIR_VECTORS[query:lower()]
    if v then return v end
    if tonumber(query) then
        local radians = math.rad(tonumber(query))
        return {math.sin(radians), -math.cos(radians)}
    end
    return nil
end

-- Squared distance from a point to a segment.
local function segPt2(px, pz, ax, az, bx, bz)
    local vx, vz = bx - ax, bz - az
    local l2 = vx * vx + vz * vz
    local t = 0
    if l2 > 1e-9 then
        t = math.max(0, math.min(1, ((px - ax) * vx + (pz - az) * vz) / l2))
    end
    local dx, dz = px - (ax + t * vx), pz - (az + t * vz)
    return dx * dx + dz * dz
end

local function cross(ax, az, bx, bz, cx, cz)
    return (bx - ax) * (cz - az) - (bz - az) * (cx - ax)
end

-- Do two capsules overlap? Segment-to-segment distance against the sum of the radii. A proper
-- crossing is distance zero; otherwise the closest approach is always at one of the four
-- endpoints -- which also covers two collinear strips lying on top of each other (TMV/TMA).
local function overlaps(a, b)
    local d1 = cross(b.ax, b.az, b.bx, b.bz, a.ax, a.az)
    local d2 = cross(b.ax, b.az, b.bx, b.bz, a.bx, a.bz)
    local d3 = cross(a.ax, a.az, a.bx, a.bz, b.ax, b.az)
    local d4 = cross(a.ax, a.az, a.bx, a.bz, b.bx, b.bz)
    if ((d1 > 0 and d2 < 0) or (d1 < 0 and d2 > 0))
            and ((d3 > 0 and d4 < 0) or (d3 < 0 and d4 > 0)) then
        return true
    end
    local d = math.min(segPt2(a.ax, a.az, b.ax, b.az, b.bx, b.bz),
                       segPt2(a.bx, a.bz, b.ax, b.az, b.bx, b.bz),
                       segPt2(b.ax, b.az, a.ax, a.az, a.bx, a.bz),
                       segPt2(b.bx, b.bz, a.ax, a.az, a.bx, a.bz))
    return d <= (a.r + b.r) ^ 2
end

local function fmtDist(m)
    if m >= 10000 then return string.format("%.0fkm", m / 1000) end
    if m >= 1000 then return string.format("%.1fkm", m / 1000) end
    return string.format("%.0fm", m)
end

local function fmtAge(s)
    if s >= 3600 then return string.format("%dh", math.floor(s / 3600)) end
    if s >= 60 then return string.format("%dm", math.floor(s / 60)) end
    return string.format("%ds", math.floor(s))
end

-- ============================================================================
-- THE DATABASE -- points and landing sites (mirroring the FMS's parsers line for line, so a
-- line the aircraft skips is a line this skips), plus sites.db, which only this reads.
-- ============================================================================

local DB = {
    points = {}, landing = {},
    sites = {}, siteOrder = {},     -- name -> { name, facs = { {name, remarks={...}}, ... } }
    fac = {}, facOrder = {},        -- CONTROLLED facilities: name -> capsule + state
    faults = {},                    -- sites.db problems, for the board (red)
    advis = {},                     -- uncontrolled facilities overlapping controlled ones (yellow)
}

local function parsePoints(body)
    local pts = {}
    for line in body:gmatch("[^\r\n]+") do
        local t = trim(line)
        if t ~= "" and t:sub(1, 1) ~= ";" then
            local name, xs, zs = t:match("^(%a+)=(%-?%d+),(%-?%d+)$")
            if name then pts[name] = { name = name, x = tonumber(xs), z = tonumber(zs) } end
        end
    end
    return pts
end

-- Landing SITES only. The pattern shapes (goaround, goaround-wp, appr) are procedures, not
-- places; they are named here so that a fifth record shape cannot quietly become a facility.
local PATTERNS = { goaround = true, ["goaround-wp"] = true, appr = true }
local function parseLanding(body)
    local land = {}
    for line in body:gmatch("[^\r\n]+") do
        local t = trim(line)
        if t ~= "" and t:sub(1, 1) ~= ";" then
            local name, ty, dir, alt, rwy =
                t:match("^(%a+)=(%a+),([%a%.%d]+),(%-?%d+),(%-?%d+)$")
            if name and not PATTERNS[ty:lower()] then
                land[name] = { type = ty:lower(), dir = dir:lower(), alt = tonumber(alt),
                               rwyLen = tonumber(rwy) }
            end
        end
    end
    return land
end

-- Split on commas that are not inside brackets, so a remark list may use commas too.
local function splitTop(s)
    local out, depth, cur = {}, 0, {}
    for ch in s:gmatch(".") do
        if ch == "(" then depth = depth + 1 elseif ch == ")" then depth = math.max(0, depth - 1) end
        if ch == "," and depth == 0 then
            table.insert(out, table.concat(cur))
            cur = {}
        else
            table.insert(cur, ch)
        end
    end
    table.insert(out, table.concat(cur))
    return out
end

-- SITE=FAC,FAC(remark),FAC(remark remark). The ORDER is the order alternatives are offered in.
local function parseSites(body)
    local sites, order, faults, owner = {}, {}, {}, {}
    for line in body:gmatch("[^\r\n]+") do
        local t = trim(line)
        if t ~= "" and t:sub(1, 1) ~= ";" then
            local sname, rest = t:match("^([%w_%-]+)=(.+)$")
            if not sname then
                table.insert(faults, "sites.db: cannot read '" .. t:sub(1, 24) .. "'")
            elseif sites[sname] then
                table.insert(faults, "sites.db: site " .. sname .. " defined twice - first kept")
            else
                local s = { name = sname, facs = {} }
                for _, tok in ipairs(splitTop(rest)) do
                    tok = trim(tok)
                    local fname, rem = tok:match("^(%a+)%s*(%b())$")
                    if not fname then fname = tok:match("^(%a+)$") end
                    if not fname then
                        if tok ~= "" then
                            table.insert(faults, sname .. ": cannot read '" .. tok:sub(1, 16) .. "'")
                        end
                    elseif owner[fname] then
                        -- One facility, one site: a divert offers the rest of THE site, and a
                        -- facility in two would make "the rest" mean two different things.
                        table.insert(faults, fname .. " is in " .. owner[fname] .. " and "
                            .. sname .. " - kept in " .. owner[fname])
                    else
                        local rs = {}
                        if rem then
                            for r in rem:sub(2, -2):gmatch("[^%s,/]+") do
                                table.insert(rs, r:lower())
                            end
                        end
                        table.insert(s.facs, { name = fname, remarks = rs })
                        owner[fname] = sname
                    end
                end
                sites[sname] = s
                table.insert(order, sname)
            end
        end
    end
    return sites, order, faults
end

-- A facility's footprint, as a capsule (see CFG.OCC's header).
local function capsule(name, p, l)
    local c = { name = name, x = p.x, z = p.z, alt = l.alt, type = l.type }
    local v = (l.type == "rwy" and (l.rwyLen or 0) > 0) and resolveDirVec(l.dir) or nil
    c.rwy = (v ~= nil)      -- landable AS A RUNWAY; a strip with no axis is flown as a pad
    if v then
        local e, len = OCCCFG.RWY_END_M, l.rwyLen
        c.ax, c.az = p.x - v[1] * e, p.z - v[2] * e
        c.bx, c.bz = p.x + v[1] * (len + e), p.z + v[2] * (len + e)
        c.r = OCCCFG.RWY_HALF_W
    else
        c.ax, c.az, c.bx, c.bz, c.r = p.x, p.z, p.x, p.z, OCCCFG.PAD_R
    end
    return c
end

local function loadAll()
    DB.points  = parsePoints(readFile(CFG.POINTS_FILE) or "")
    DB.landing = parseLanding(readFile(CFG.LANDING_FILE) or "")
    local sites, order, faults = parseSites(readFile(CFG.SITES_FILE) or "")
    local fac, facOrder, advis = {}, {}, {}
    for _, sname in ipairs(order) do
        for _, fe in ipairs(sites[sname].facs) do
            local p, l = DB.points[fe.name], DB.landing[fe.name]
            local f
            if p and l then
                f = capsule(fe.name, p, l)
                f.geo = true
            else
                f = { name = fe.name, alt = 0, bad = "not a landing site" }
                table.insert(faults, sname .. ": " .. fe.name .. " is not a landing site")
            end
            f.site, f.remarks, f.conf = sname, fe.remarks, {}
            for _, r in ipairs(fe.remarks) do
                if CFG.REMARKS[r] == nil then
                    f.bad = "unknown remark (" .. r .. ")"
                    table.insert(faults, fe.name .. ": unknown remark '" .. r .. "' - closed")
                end
            end
            -- A resync must not forget who is sitting where: carry the occupancy over.
            f.occ = (DB.fac[fe.name] and DB.fac[fe.name].occ) or {}
            fac[fe.name] = f
            table.insert(facOrder, fe.name)
        end
    end
    -- Conflicts: two facilities whose footprints overlap are one piece of ground.
    for i = 1, #facOrder do
        local a = fac[facOrder[i]]
        for j = i + 1, #facOrder do
            local b = fac[facOrder[j]]
            if a.geo and b.geo and overlaps(a, b) then
                a.conf[b.name], b.conf[a.name] = true, true
            end
        end
    end
    -- ...and a landing site in NO site that overlaps a controlled one is a hole in the
    -- control: an aircraft asking for it is told "uncontrolled" and may land head-on into a
    -- clearance given for the other end of the same strip. Said on the board rather than
    -- silently patched, because which site it belongs to is sites.db's author's call.
    for name, l in pairs(DB.landing) do
        local p = DB.points[name]
        if p and not fac[name] then
            local c = capsule(name, p, l)
            for _, fname in ipairs(facOrder) do
                local f = fac[fname]
                if f.geo and overlaps(c, f) then
                    table.insert(advis, name .. " overlaps " .. fname .. " (" .. f.site
                        .. ") but is in no site")
                    break
                end
            end
        end
    end
    table.sort(advis)
    DB.sites, DB.siteOrder, DB.fac, DB.facOrder = sites, order, fac, facOrder
    DB.faults, DB.advis = faults, advis
end

-- ============================================================================
-- TRACKS -- responder reports, keyed by sending computer id (not callsign: two responders
-- cloned with the callsign line unedited must not merge into one teleporting aircraft). The
-- callsign is still what identity is compared on, because that is what the FMS says it is.
-- ============================================================================

local TRK = { list = {} }

function TRK.note(id, m)
    local t = TRK.list[id]
    if not t then t = {}; TRK.list[id] = t end
    t.cs = m.callsign
    t.x, t.y, t.z = m.x, m.y, m.z
    if type(m.vx) == "number" and type(m.vz) == "number" then
        t.gs = math.sqrt(m.vx * m.vx + m.vz * m.vz)
    else
        t.gs = (type(m.gs) == "number") and m.gs or 0
    end
    t.at = os.clock()
end

function TRK.loop()
    while true do
        local sender, m = rednet.receive(LINKS.ATC, 5)
        if sender and type(m) == "table" and type(m.x) == "number" and type(m.y) == "number"
                and type(m.z) == "number" and type(m.callsign) == "string" then
            TRK.note(sender, m)
        end
        local now = os.clock()
        for id, t in pairs(TRK.list) do
            if now - t.at > CFG.TRACK.DROP_AFTER then TRK.list[id] = nil end
        end
    end
end

-- ============================================================================
-- OCCUPANCY -- who is on which facility, live or remembered. See CFG.OCC.
-- ============================================================================

local PARK = { list = {} }   -- callsign -> { fac = <facility>, at = <epoch ms> }

function PARK.load()
    local s = readFile(CFG.PARK_FILE)
    local t = s and textutils.unserialize(s)
    PARK.list = {}
    if type(t) == "table" then
        for cs, p in pairs(t) do
            if type(cs) == "string" and type(p) == "table" and type(p.fac) == "string" then
                PARK.list[cs] = { fac = p.fac, at = tonumber(p.at) or os.epoch("utc") }
            end
        end
    end
end

function PARK.save() writeFile(CFG.PARK_FILE, textutils.serialize(PARK.list)) end

-- Forget every aircraft remembered on a facility: the board's touch action, for a record
-- belonging to an aircraft that is never going to transmit again.
function PARK.forget(facName)
    local changed = false
    for cs, p in pairs(PARK.list) do
        if p.fac == facName then
            PARK.list[cs] = nil
            local f = DB.fac[facName]
            if f then f.occ[cs] = nil end
            changed = true
        end
    end
    if changed then PARK.save() end
    return changed
end

local OCC = {}

function OCC.inside(f, t)
    if not (f and f.geo) then return false end
    if t.y > f.alt + OCCCFG.AGL_MAX or t.y < f.alt - OCCCFG.BELOW_MAX then return false end
    return segPt2(t.x, t.z, f.ax, f.az, f.bx, f.bz) <= f.r * f.r
end

-- Refresh every facility's `occ` (callsign -> last seen on it). An entry lives FREE_AFTER
-- seconds past its last sighting, which is what makes a facility stay busy for a moment
-- after an aircraft leaves it.
function OCC.update(now)
    local fresh, dirty = {}, false
    for _, t in pairs(TRK.list) do
        if now - t.at <= CFG.TRACK.STALE then
            fresh[t.cs] = true
            local on = nil
            for _, name in ipairs(DB.facOrder) do
                local f = DB.fac[name]
                if OCC.inside(f, t) then
                    f.occ[t.cs] = now
                    on = on or name
                end
            end
            -- Remember an aircraft that has come to rest on a facility; forget it the moment
            -- it is heard anywhere off that facility.
            local p = PARK.list[t.cs]
            if on and t.gs < OCCCFG.PARK_GS then
                t.still = t.still or now
                if now - t.still >= OCCCFG.PARK_AFTER and not (p and OCC.inside(DB.fac[p.fac], t)) then
                    PARK.list[t.cs] = { fac = on, at = os.epoch("utc") }
                    dirty = true
                end
            else
                t.still = nil
            end
            p = PARK.list[t.cs]
            if p and not OCC.inside(DB.fac[p.fac], t) then
                PARK.list[t.cs] = nil
                dirty = true
            end
        end
    end
    -- A remembered aircraft that is not being heard is still sitting there.
    for cs, p in pairs(PARK.list) do
        local f = DB.fac[p.fac]
        if f and not fresh[cs] then f.occ[cs] = now end
    end
    for _, name in ipairs(DB.facOrder) do
        local f = DB.fac[name]
        for cs, at in pairs(f.occ) do
            if now - at > OCCCFG.FREE_AFTER then f.occ[cs] = nil end
        end
    end
    if dirty then PARK.save() end
    OCC.fresh = fresh
end

-- ============================================================================
-- REQUESTS AND ALLOCATION
-- ============================================================================

local REQ = { list = {} }    -- callsign -> request
local CTL = { served = 0, lastReq = nil }

function REQ.valid(m)
    return type(m) == "table" and m.t == "req" and type(m.cs) == "string" and m.cs ~= ""
       and type(m.fac) == "string" and m.fac ~= ""
end

function REQ.note(id, m, now)
    local r = REQ.list[m.cs]
    -- `first` is the place in the queue. It survives the aircraft switching facility (a
    -- divert, a go-around) -- it has been waiting for this site all along -- and is lost only
    -- when the aircraft stops asking altogether.
    if not r then r = { cs = m.cs, first = now }; REQ.list[m.cs] = r end
    r.id, r.fac, r.at = id, m.fac, now
    r.x = (type(m.x) == "number") and m.x or nil
    r.z = (type(m.z) == "number") and m.z or nil
    r.stage = (type(m.stage) == "string") and m.stage or "?"
    r.rwyOnly = m.rwyonly and true or false
    -- The flags a remark may refuse (CFG.REMARKS' values), read as booleans. A flag the
    -- aircraft does not send is false: an FMS predating a remark is not refused by it.
    r.flags = {}
    for _, flag in pairs(CFG.REMARKS) do r.flags[flag] = m[flag] and true or false end
end

local function busyBy(f, cs)
    for who in pairs(f.occ) do
        if who ~= cs then return who end
    end
    return nil
end

-- May request r be put on facility f, given the facilities already handed out this pass?
-- Returns false plus one line saying why, for the pilot.
function CTL.usable(f, r, taken)
    if not f then return false, "not controlled" end
    if f.bad then return false, f.name .. " closed: " .. f.bad end
    if r.rwyOnly and not f.rwy then return false, f.name .. " is not a runway" end
    for _, rm in ipairs(f.remarks) do
        local flag = CFG.REMARKS[rm]
        if flag and r.flags[flag] then return false, f.name .. " is " .. rm end
    end
    local who = busyBy(f, r.cs)
    if not who then
        for g in pairs(f.conf) do
            who = busyBy(DB.fac[g], r.cs)
            if who then break end
        end
    end
    if who then return false, f.name .. " occupied: " .. who end
    for g, cs in pairs(taken) do
        if cs ~= r.cs and (g == f.name or f.conf[g]) then
            return false, f.name .. " cleared to " .. cs
        end
    end
    return true
end

function CTL.allocate()
    local now = os.clock()
    for cs, r in pairs(REQ.list) do
        if now - r.at > CFG.REQ.STALE then REQ.list[cs] = nil end
    end
    OCC.update(now)

    local order = {}
    for _, r in pairs(REQ.list) do table.insert(order, r) end
    table.sort(order, function(a, b)
        if a.first ~= b.first then return a.first < b.first end
        return a.cs < b.cs
    end)
    CTL.order = order

    -- Pass 1: standing assignments, in queue order. Kept only while the aircraft is still
    -- asking for the same thing it was assigned against (a pilot who re-plans elsewhere is
    -- asking a new question), and only while the facility is still usable -- an assignment
    -- does not survive somebody else sitting down on it.
    local taken = {}
    for _, r in ipairs(order) do
        local f, rf = r.assign and DB.fac[r.assign], DB.fac[r.fac]
        if f and rf and f.site == rf.site and (r.assign == r.fac or r.assignFor == r.fac)
                and CTL.usable(f, r, taken) then
            taken[f.name] = r.cs
        else
            r.assign, r.assignFor = nil, nil
        end
    end

    -- Pass 2: everyone else, first come first served.
    local queued = {}
    for _, r in ipairs(order) do
        local rf = DB.fac[r.fac]
        r.why, r.q = nil, nil
        if not rf then
            r.status, r.assign = "none", nil
        else
            local ok, why = CTL.usable(rf, r, taken)
            if not r.assign then
                if ok then
                    r.assign = r.fac
                else
                    for _, fe in ipairs(DB.sites[rf.site].facs) do
                        if fe.name ~= r.fac and CTL.usable(DB.fac[fe.name], r, taken) then
                            r.assign = fe.name
                            break
                        end
                    end
                end
                if r.assign then
                    taken[r.assign] = r.cs
                    r.assignFor = r.fac
                end
            end
            if r.assign == r.fac then
                r.status = "clr"
            elseif r.assign then
                r.status, r.why = "divert", why
            else
                r.status, r.why = "hold", why
                queued[rf.site] = (queued[rf.site] or 0) + 1
                r.q = queued[rf.site]
            end
        end
    end
end

function CTL.reply(r)
    rednet.send(r.id, {
        t = r.status, cs = r.cs, fac = r.fac, to = r.assign, why = r.why, q = r.q,
    }, LINKS.CTL)
end

function CTL.loop()
    while true do
        local sender, m = rednet.receive(LINKS.CTL, 0.5)
        if sender and REQ.valid(m) then
            REQ.note(sender, m, os.clock())
            CTL.allocate()
            local r = REQ.list[m.cs]
            if r then
                CTL.reply(r)
                CTL.served, CTL.lastReq = CTL.served + 1, os.clock()
            end
        else
            -- Nobody asking: still re-evaluate, so the board and the parked-aircraft memory
            -- stay current between arrivals.
            CTL.allocate()
        end
    end
end

-- ============================================================================
-- NAV DATA SUBSCRIPTION -- the scope's client (atc.lua's NDU), plus sites.db. Stage beside the
-- live file, verify the length on disk, then swap: a CC computer has a hard space limit, and
-- opening the live file for writing truncates it before the failure is known.
-- ============================================================================

local NDU = { rev = nil, stamp = nil, serverId = nil, lastHeard = nil, applied = 0, fault = nil }

function NDU.sane(m)
    if type(m.points) ~= "string" or type(m.landing) ~= "string" then return false end
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
    local files = { {CFG.POINTS_FILE, m.points}, {CFG.LANDING_FILE, m.landing} }
    -- A server predating sites.db sends none; keep the copy on this disk rather than
    -- treating its absence as "no sites" and dropping all control.
    if type(m.sites) == "string" then table.insert(files, {CFG.SITES_FILE, m.sites}) end
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

function NDU.report()
    if not NDU.serverId then return end
    rednet.send(NDU.serverId, {
        t = "ack", rev = NDU.rev, stamp = NDU.stamp, fault = NDU.fault,
        who = os.getComputerLabel() or ("ATC controller #" .. os.getComputerID()),
    }, LINKS.NAVDATA)
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
        local sender, msg = rednet.receive(LINKS.NAVDATA, 1)
        if sender and type(msg) == "table" and msg.t == "sync" then
            NDU.serverId, NDU.lastHeard = sender, os.clock()
            if msg.stamp ~= NDU.stamp and NDU.sane(msg) then pcall(NDU.applyNow, msg) end
            NDU.report()
        end
        if not NDU.lastHeard and os.clock() >= nextPull then
            rednet.broadcast({ t = "pull" }, LINKS.NAVDATA)
            nextPull = os.clock() + CFG.NAVDATA.PULL_EVERY
        end
    end
end

-- ============================================================================
-- THE BOARD -- one screen: every controlled facility and its state, then every aircraft on
-- frequency and what it has been told. Drawn on the monitor if there is one AND on the
-- terminal, so the station can be read from either. Touching a facility row forgets any
-- silent aircraft remembered on it.
-- ============================================================================

local UI = { hits = {} }    -- device -> { [row] = facility }

local mon, monName
do
    if CFG.MONITOR_NAME then
        mon = peripheral.wrap(CFG.MONITOR_NAME)
        if not mon then
            error("atcctlconf.lua names monitor '" .. CFG.MONITOR_NAME .. "' - not attached.", 0)
        end
        monName = CFG.MONITOR_NAME
    else
        mon = peripheral.find("monitor")
        if mon then monName = peripheral.getName(mon) end
    end
    if mon then pcall(function() mon.setTextScale(CFG.TEXT_SCALE) end) end
end

-- One word and a colour for a facility.
function UI.facState(f)
    if f.bad then return "CLOSED " .. f.bad, colours.red end
    for cs in pairs(f.occ) do
        local p = PARK.list[cs]
        if p and p.fac == f.name and not (OCC.fresh and OCC.fresh[cs]) then
            return "OCC " .. cs .. " silent, parked " .. fmtAge((os.epoch("utc") - p.at) / 1000),
                colours.red
        end
        return "OCC " .. cs, colours.orange
    end
    for _, r in ipairs(CTL.order or {}) do
        if r.assign == f.name then return "CLR " .. r.cs, colours.lime end
    end
    for g in pairs(f.conf) do
        local gf = DB.fac[g]
        if gf and next(gf.occ) then return "BLOCKED by " .. g, colours.orange end
        for _, r in ipairs(CTL.order or {}) do
            if r.assign == g then return "BLOCKED by " .. g, colours.lightGrey end
        end
    end
    return "FREE", colours.green
end

function UI.draw(dev)
    local w, h = dev.getSize()
    local hits = {}
    dev.setBackgroundColour(colours.black)
    dev.clear()
    local function put(x, y, s, fg)
        if y < 1 or y > h or x > w then return end
        dev.setCursorPos(x, y)
        dev.setTextColour(fg or colours.white)
        dev.write(s:sub(1, w - x + 1))
    end

    put(1, 1, "ATC CONTROLLER #" .. os.getComputerID(), colours.orange)
    local ns, nc = NDU.status()
    put(math.max(1, w - #ns + 1), 1, ns, nc)

    local y = 2
    put(1, y, "SITE FAC  TYPE REMARKS   STATE", colours.lightGrey)
    y = y + 1
    if #DB.siteOrder == 0 then
        put(1, y, "no sites defined - nothing is controlled", colours.orange)
        y = y + 1
    end
    for _, sname in ipairs(DB.siteOrder) do
        for i, fe in ipairs(DB.sites[sname].facs) do
            local f = DB.fac[fe.name]
            if i == 1 then put(1, y, sname:sub(1, 4), colours.orange) end
            put(6, y, fe.name, colours.white)
            put(11, y, f.type or "?", colours.lightGrey)
            put(16, y, table.concat(fe.remarks, " "):sub(1, 9), colours.lightGrey)
            local st, col = UI.facState(f)
            put(26, y, st, col)
            hits[y] = fe.name
            y = y + 1
        end
    end

    y = y + 1
    put(1, y, "ON FREQUENCY", colours.orange)
    put(14, y, string.format("%d answered", CTL.served), colours.lightGrey)
    y = y + 1
    local now = os.clock()
    local order = CTL.order or {}
    if #order == 0 then
        put(1, y, "no aircraft calling", colours.lightGrey)
        y = y + 1
    end
    for _, r in ipairs(order) do
        local what, col
        if r.status == "clr" then what, col = "CLEARED", colours.lime
        elseif r.status == "divert" then what, col = "DIVERT " .. tostring(r.assign), colours.yellow
        elseif r.status == "hold" then what, col = "HOLD #" .. tostring(r.q or "?"), colours.orange
        else what, col = "UNCONTROLLED", colours.lightGrey end
        local f = DB.fac[r.fac] or DB.points[r.fac]
        -- A facility closed as "not a landing site" has no position to measure to.
        local d = (f and f.x and r.x and r.z)
            and fmtDist(math.sqrt((f.x - r.x) ^ 2 + (f.z - r.z) ^ 2)) or "?"
        put(1, y, r.cs:sub(1, 9), colours.white)
        put(11, y, r.fac:sub(1, 4), colours.white)
        put(16, y, what, col)
        put(32, y, d, colours.lightGrey)
        put(40, y, fmtAge(now - r.first), colours.lightGrey)
        y = y + 1
        if r.why and y <= h - 1 then
            put(3, y, r.why, colours.lightGrey)
            y = y + 1
        end
    end

    -- Faults and advisories last: they matter, but they are about the file, not the traffic.
    y = y + 1
    for _, s in ipairs(DB.faults) do
        if y > h - 1 then break end
        put(1, y, s, colours.red)
        y = y + 1
    end
    for _, s in ipairs(DB.advis) do
        if y > h - 1 then break end
        put(1, y, s, colours.yellow)
        y = y + 1
    end
    put(1, h, "touch a facility to forget a silent aircraft on it", colours.grey)
    return hits
end

function UI.drawAll()
    local ok, err = pcall(function()
        UI.hits.term = UI.draw(term)
        if mon then UI.hits.mon = UI.draw(mon) end
    end)
    if not ok then
        term.setCursorPos(1, 1)
        term.setTextColour(colours.red)
        term.write("draw error: " .. tostring(err))
    end
end

function UI.touch(hits, y)
    local name = hits and hits[y]
    if name then PARK.forget(name) end
end

function UI.loop()
    UI.drawAll()
    local timer = os.startTimer(CFG.REDRAW)
    while true do
        -- Repaint on the timer and on a touch, never on every event: this computer hears every
        -- responder in the fleet at 20 Hz, and a repaint per packet would be all it ever did.
        local ev = { os.pullEvent() }
        if ev[1] == "timer" and ev[2] == timer then
            UI.drawAll()
            timer = os.startTimer(CFG.REDRAW)
        elseif ev[1] == "monitor_touch" and ev[2] == monName then
            UI.touch(UI.hits.mon, ev[4])
            UI.drawAll()
        elseif ev[1] == "mouse_click" then
            UI.touch(UI.hits.term, ev[4])
            UI.drawAll()
        end
    end
end

-- ============================================================================
-- MAIN
-- ============================================================================

local modem = peripheral.find("modem")
if not modem then error("No modem attached - the controller has nobody to talk to") end
if not rednet.isOpen(peripheral.getName(modem)) then
    rednet.open(peripheral.getName(modem))
end

loadAll()
PARK.load()

local ok, err = pcall(function()
    parallel.waitForAny(TRK.loop, CTL.loop, NDU.loop, UI.loop)
end)

term.setBackgroundColour(colours.black)
term.setTextColour(colours.white)
term.clear()
term.setCursorPos(1, 1)
if not ok then print("atc controller error: " .. tostring(err)) end
