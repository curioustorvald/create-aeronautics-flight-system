-- Aer Togekiss Autopilot -- touch-oriented navigator / flight management page
-- a
-- This is the "external navigator" navcom.lua's own header talks about: navcom
-- just flies whatever (x, z, type, name) it is last told and holds it when the
-- link goes quiet; deciding WHAT to fly next, and telling the pilot about it on
-- a touchscreen, is this program's job. Nothing here needed to change in
-- navcom.lua -- its rednet handler already (a) keys x/z fields exactly as used
-- below, (b) treats a same-coordinate rebroadcast as a keep-alive rather than a
-- new leg (WP_EPSILON), and (c) refreshes its own "link ok" timer on every one
-- of those rebroadcasts. That is precisely the once-a-second ping this program
-- sends, so navcom needed no extension.
--
-- Data lives in two flat text files at the root next to this script:
--   points.db  NAME=x,z          3-letter name = aerodrome/POI, 5-letter = VOR
--   course.db  ROUTE-NAME=N1,N2,...,Nn   an ordered list of point names
-- Both are hand-editable but this program can also add/edit/delete entries
-- from the touchscreen, rewriting the files in the same layout.
--
-- Touch input: prefers an attached "monitor" peripheral (redirects the whole
-- display there and listens for monitor_touch); falls back to this computer's
-- own terminal and mouse_click if no monitor is present, so it still works
-- for testing directly on the computer.
--
-- Palette (matches navcom.lua's accent scheme):
--   colours.orange  highlight 1 (primary / active)
--   colours.lime    highlight 2 (secondary / editable / confirm)
--   colours.brown   inactive 1  (dimmed orange)
--   colours.green   inactive 2  (dimmed lime / "done")
--   colours.white   body text, on colours.black background

-- CONFIG ----------------------------------------------------------------------
-- Every tuning value, sign convention, protocol name and filename lives in apconf.lua
-- beside this program (see its header). This block loads it and binds the names the rest of
-- the file already used, so a knob is still just `DC.CURVE` or `LG.RWY_GS_DEG` down there.
--
-- There is deliberately NO fallback copy of any of it here: a second set of defaults would
-- let an edit in apconf.lua silently do nothing, which is precisely what would make an
-- empirically-calibrated sign untrustworthy. A missing or truncated file is a hard BOOT
-- error naming the missing knob, not a nil-arithmetic crash halfway through a flight.
--
-- Everything but CFG itself is scoped inside the do-block so the load temporaries do not
-- occupy top-level registers (see the 200-locals note in apconf.lua's DC block). Net cost
-- of the loader is therefore ONE local, CFG -- which the rarely-read knobs use directly
-- (CFG.TOUCHDOWN / NAVDATA / APSWITCH / PALETTE); the hot ones are bound to their old names.
local CFG
local APNAV_PROTOCOL, AUTOLAND_PROTOCOL, ALTOVERRIDE_PROTOCOL
local POINTS_FILE, COURSE_FILE, LANDING_FILE
local PING_INTERVAL, TICK_INTERVAL, HEADING_SIGN, HEADING_OFFSET
local SPEED_FULL, SPEED_SLOW, NEXT_CRUISE, RESUME_BACKTRACK_WEIGHT
local RAD, DC, LG, HSICFG
do
    local ok, res = pcall(require, "apconf")
    if not ok then
        error("Cannot load apconf.lua (the FMS configuration): " .. tostring(res)
            .. "\nIt must sit beside autopilot.lua and end with 'return CFG'.", 0)
    end
    if type(res) ~= "table" then
        error("apconf.lua did not return the CFG table (got " .. type(res) .. ").", 0)
    end
    -- Canary check: catch a truncated or half-saved config at BOOT, naming what is missing,
    -- rather than as a nil-arithmetic crash mid-flight. One representative key per section --
    -- not exhaustive, but a file carrying all of these is the real configuration, not a
    -- fragment. The second pass goes one level down, where a half-finished edit tends to land.
    for _, k in ipairs({ "APNAV_PROTOCOL", "AUTOLAND_PROTOCOL", "ALTOVERRIDE_PROTOCOL",
            "TOUCHDOWN", "NAVDATA", "APSWITCH", "POINTS_FILE", "COURSE_FILE", "LANDING_FILE",
            "PING_INTERVAL", "TICK_INTERVAL", "HEADING_SIGN", "HEADING_OFFSET", "RAD",
            "SPEED_FULL", "SPEED_SLOW", "NEXT_CRUISE", "RESUME_BACKTRACK_WEIGHT",
            "DC", "LG", "HSI", "PALETTE", "HSI_TEXT_SCALE" }) do
        if res[k] == nil then
            error("apconf.lua is missing CFG." .. k .. " -- the file looks incomplete.", 0)
        end
    end
    for t, k in pairs({ RAD = "VOR_RADIUS", DC = "CURVE", LG = "RWY_GS_DEG",
            HSI = "CDI_FULLSCALE_M", TOUCHDOWN = "PROTOCOL", NAVDATA = "PROTOCOL",
            APSWITCH = "SIDE", PALETTE = "HSI", NEXT_CRUISE = "full" }) do
        if type(res[t]) ~= "table" or res[t][k] == nil then
            error("apconf.lua's CFG." .. t .. " is missing " .. k .. ".", 0)
        end
    end
    CFG = res
    APNAV_PROTOCOL, AUTOLAND_PROTOCOL = CFG.APNAV_PROTOCOL, CFG.AUTOLAND_PROTOCOL
    ALTOVERRIDE_PROTOCOL                   = CFG.ALTOVERRIDE_PROTOCOL
    POINTS_FILE, COURSE_FILE, LANDING_FILE = CFG.POINTS_FILE, CFG.COURSE_FILE, CFG.LANDING_FILE
    PING_INTERVAL, TICK_INTERVAL           = CFG.PING_INTERVAL, CFG.TICK_INTERVAL
    HEADING_SIGN, HEADING_OFFSET           = CFG.HEADING_SIGN, CFG.HEADING_OFFSET
    SPEED_FULL, SPEED_SLOW, NEXT_CRUISE    = CFG.SPEED_FULL, CFG.SPEED_SLOW, CFG.NEXT_CRUISE
    RESUME_BACKTRACK_WEIGHT                = CFG.RESUME_BACKTRACK_WEIGHT
    RAD, DC, LG, HSICFG                    = CFG.RAD, CFG.DC, CFG.LG, CFG.HSI
end

-- AUTO-LANDING ---------------------------------------------------------------
-- The FMS runs the landing state machine itself: it reads position, heading,
-- altitude and vertical speed straight off the sublevel API and already decides
-- flight phase, so deciding when to align / translate / descend / flare / cut is
-- the same job one step further. It drives NAVCOM horizontally (position-hold over
-- the pad plus hold the landing heading, via a new "LANDING" phase) and the
-- Stabiliser+Altitude control vertically (a commanded descent rate, via
-- AUTOLAND_PROTOCOL). Landing sites and their approach heading/altitude come from
-- landing.db. The one thing it does NOT work out for itself is when the wheels are
-- actually down: that arrives from the gear's squat switch on TD.PROTOCOL (see the TD
-- table), with the AGL test kept as the fallback for when that sensor is silent.

-- GEOMETRY / HEADING HELPERS --------------------------------------------------

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

local function clamp(v, lo, hi)
    return math.max(lo, math.min(hi, v))
end

-- Wrap an angle to (-pi, pi]; keeps a heading error from exploding across the seam.
local function wrapPi(a)
    return (a + math.pi) % (2 * math.pi) - math.pi
end

-- Pull the yaw out of the orientation quaternion, tolerating either return shape --
-- copied verbatim from navcom.lua so the two measure heading identically.
local function getYaw(orientation)
    local a, b = orientation:toEuler()
    if type(a) == "number" then return b end
    return a.y
end

-- Landing directions (landing.db's n/e/s/w/x) as world unit vectors, mapped to a
-- heading in the SAME frame navcom uses for bearing (bearing = atan2(dx, dz)). So
-- "face north" resolves to the heading whose nose points along Z-, and NAVCOM holds
-- exactly that when handed this number. 'x' (don't care) yields nil -> no constraint.
local DIR_VECTORS = { n = {0, -1}, e = {1, 0}, s = {0, 1}, w = {-1, 0} }
local function headingForDir(dir)
    local v = DIR_VECTORS[dir]
    if not v then return nil end
    return atan2(v[1], v[2])
end

-- DISPLAY / TOUCH SETUP -------------------------------------------------------

local monName = nil -- set once we know whether a monitor is in use
local hsiMonName = nil -- the VOR/HSI instrument monitor; display-only, its touches must
                       -- NOT be read as UI input (the UI lives on this computer's screen)

local function pointerXY(ev)
    if ev[1] == "monitor_touch" and ev[2] ~= hsiMonName and (not monName or ev[2] == monName) then
        return ev[3], ev[4]
    elseif ev[1] == "mouse_click" then
        return ev[3], ev[4]
    end
    return nil
end

-- UI PRIMITIVES ----------------------------------------------------------------

local hitboxes = {}

local function clearHitboxes()
    hitboxes = {}
end

local function addHitbox(x1, y1, x2, y2, id)
    table.insert(hitboxes, {x1 = x1, y1 = y1, x2 = x2, y2 = y2, id = id})
end

local function hitTest(x, y)
    for i = #hitboxes, 1, -1 do
        local h = hitboxes[i]
        if x >= h.x1 and x <= h.x2 and y >= h.y1 and y <= h.y2 then
            return h.id
        end
    end
    return nil
end

local function fillRect(x, y, w, h, bg)
    term.setBackgroundColour(bg)
    for row = y, y + h - 1 do
        term.setCursorPos(x, row)
        term.write(string.rep(" ", w))
    end
end

local function centreText(x, y, w, text, fg, bg)
    text = tostring(text)
    if #text > w then text = text:sub(1, w) end
    term.setBackgroundColour(bg)
    term.setTextColour(fg)
    local pad = math.floor((w - #text) / 2)
    term.setCursorPos(x + pad, y)
    term.write(text)
end

-- Draws a button and (only if enabled) registers its hitbox. Disabled buttons
-- fall back to the matching inactive colour (orange -> brown, lime -> green)
-- so a greyed-out control still communicates state instead of disappearing.
local function button(x, y, w, h, id, label, enabled, bg)
    if enabled == nil then enabled = true end
    bg = bg or colours.orange
    local realBg = bg
    if not enabled then
        if bg == colours.lime then
            realBg = colours.green
        else
            realBg = colours.brown
        end
    end
    local fg = enabled and colours.black or colours.white
    fillRect(x, y, w, h, realBg)
    centreText(x, y + math.floor((h - 1) / 2), w, label, fg, realBg)
    if enabled then
        addHitbox(x, y, x + w - 1, y + h - 1, id)
    end
end

local function alert(title, message)
    while true do
        clearHitboxes()
        term.setBackgroundColour(colours.black)
        term.clear()
        term.setTextColour(colours.orange)
        term.setCursorPos(2, 1); term.write(title)
        term.setTextColour(colours.white)
        term.setCursorPos(2, 3); term.write(message)
        local w, h = term.getSize()
        button(math.floor(w / 2) - 5, h - 1, 10, 1, "OK", "OK", true, colours.orange)
        local ev = {os.pullEvent()}
        local x, y = pointerXY(ev)
        if x then
            local id = hitTest(x, y)
            if id == "OK" then return end
        end
    end
end

local function confirm(title, message)
    while true do
        clearHitboxes()
        term.setBackgroundColour(colours.black)
        term.clear()
        term.setTextColour(colours.orange)
        term.setCursorPos(2, 1); term.write(title)
        term.setTextColour(colours.white)
        term.setCursorPos(2, 3); term.write(message)
        local w, h = term.getSize()
        button(2, h - 1, 10, 1, "YES", "YES", true, colours.orange)
        button(w - 11, h - 1, 10, 1, "NO", "NO", true, colours.brown)
        local ev = {os.pullEvent()}
        local x, y = pointerXY(ev)
        if x then
            local id = hitTest(x, y)
            if id == "YES" then return true
            elseif id == "NO" then return false end
        end
    end
end

-- On-screen keyboards for the three kinds of text this program ever needs to
-- collect: a point code, a course name, or a signed coordinate.
local KEYSETS = {
    point = {
        chars = {"A","B","C","D","E","F","G","H","I","J","K","L","M",
                 "N","O","P","Q","R","S","T","U","V","W","X","Y","Z"},
        maxlen = 5, perRow = 9, keyW = 4,
    },
    route = {
        chars = {"A","B","C","D","E","F","G","H","I","J","K","L","M",
                 "N","O","P","Q","R","S","T","U","V","W","X","Y","Z",
                 "0","1","2","3","4","5","6","7","8","9","-"},
        maxlen = 16, perRow = 9, keyW = 4,
    },
    numeric = {
        chars = {"1","2","3","4","5","6","7","8","9","0","-"},
        maxlen = 8, perRow = 3, keyW = 5,
    },
}

local function promptKeyboard(title, initial, charsetName)
    local ks = KEYSETS[charsetName]
    local buf = initial or ""
    while true do
        clearHitboxes()
        term.setBackgroundColour(colours.black)
        term.clear()
        term.setTextColour(colours.orange)
        term.setCursorPos(2, 1); term.write(title)
        term.setTextColour(colours.white)
        term.setCursorPos(2, 2); term.write("> " .. buf .. "_")

        local startX, startY = 2, 4
        for i, ch in ipairs(ks.chars) do
            local col = (i - 1) % ks.perRow
            local row = math.floor((i - 1) / ks.perRow)
            local x = startX + col * (ks.keyW + 1)
            local y = startY + row * 2
            button(x, y, ks.keyW, 1, "K:" .. ch, ch, true, colours.lime)
        end

        local w, h = term.getSize()
        local by = h - 1
        button(2, by, 3, 1, "K:BKSP", "<-", true, colours.orange)
        button(7, by, 5, 1, "K:CLR", "CLR", true, colours.brown)
        button(w - 14, by, 6, 1, "K:OK", "OK", true, colours.orange)
        button(w - 7, by, 6, 1, "K:CANCEL", "X", true, colours.brown)

        local ev = {os.pullEvent()}
        local x, y = pointerXY(ev)
        if x then
            local id = hitTest(x, y)
            if id == "K:OK" then
                return buf
            elseif id == "K:CANCEL" then
                return nil
            elseif id == "K:BKSP" then
                buf = buf:sub(1, -2)
            elseif id == "K:CLR" then
                buf = ""
            elseif id and id:sub(1, 2) == "K:" then
                local ch = id:sub(3)
                if ch == "-" then
                    if buf:sub(1, 1) == "-" then
                        buf = buf:sub(2)
                    else
                        buf = "-" .. buf
                    end
                elseif #buf < ks.maxlen then
                    buf = buf .. ch
                end
            end
        end
    end
end

-- DATA LAYER: points.db / course.db --------------------------------------------

local POINTS = {}       -- name -> {name, x, z, kind}
local POINT_ORDER = {}  -- array of names, load/insertion order
local COURSES = {}      -- name -> {name, seq = {point names...}}
local COURSE_ORDER = {}
local LANDING = {}      -- name -> {type="heli"|"rwy", dir=n/e/s/w/x, alt, rwyLen}

local function classify(name)
    if #name == 5 then return "vor" end
    return "poi"
end

-- landing.db is hand-edited (see its own header); this program only reads it. A line
-- is NAME=type,dir,altitude,runwayLength. A point named here is a landing site the
-- autopilot can auto-land at; anything else is just a waypoint.
local function loadLanding()
    LANDING = {}
    local f = io.open(LANDING_FILE, "r")
    if not f then return end
    for line in f:lines() do
        local t = line:match("^%s*(.-)%s*$")
        if t ~= "" and t:sub(1, 1) ~= ";" then
            local name, ty, dir, alt, rwy = t:match("^(%a+)=(%a+),(%a+),(%-?%d+),(%-?%d+)$")
            -- GO-AROUND routes are a SECOND record shape in the same file: an arbitrary id,
            -- then the two ends of a straight leg and the altitude to fly it at. They are NOT
            -- landing sites (see isLandingSite) -- nothing flies TO one -- but they belong to
            -- the runway they serve, so they live here. Which route serves which runway is
            -- decided by geometry, not by the id: see LND.gaFor.
            local gid, gx1, gz1, gx2, gz2, galt =
                t:match("^([%w_]+)=goaround,(%-?%d+),(%-?%d+),(%-?%d+),(%-?%d+),(%-?%d+)$")
            -- ...and a THIRD, which is the one to publish: the same circuit written as a list
            -- of ordinary points instead of one leg -- an id, the minimum altitude to fly the
            -- pattern at, then the waypoint names, starting AND ending at the runway served.
            -- Naming the runway is the point: this shape says which runway it belongs to
            -- outright, where the straight-leg one has to be matched by geometry. When a
            -- runway publishes both, this is the one flown (see LND.gaFor).
            local wid, walt, wseq =
                t:match("^([%w_]+)=goaround%-wp,(%-?%d+),(.+)$")
            if gid then
                LANDING[gid] = {
                    type = "goaround",
                    x1 = tonumber(gx1), z1 = tonumber(gz1),
                    x2 = tonumber(gx2), z2 = tonumber(gz2), alt = tonumber(galt),
                }
            elseif wid then
                local seq = {}
                for p in wseq:gmatch("[^,]+") do
                    table.insert(seq, (p:match("^%s*(.-)%s*$")))
                end
                LANDING[wid] = { type = "goaround-wp", alt = tonumber(walt), seq = seq }
            elseif name then
                LANDING[name] = {
                    type   = ty:lower(),
                    dir    = dir:lower(),
                    alt    = tonumber(alt),
                    rwyLen = tonumber(rwy),
                }
            end
        end
    end
    f:close()
end

-- A place the autopilot can be told to LAND at. Go-around routes share landing.db but are not
-- one: they have no point of their own, and nothing may offer one as a destination. Both
-- published shapes are excluded by the one prefix test, so a third one cannot be added to the
-- file and quietly become a destination.
local function isLandingSite(name)
    local l = LANDING[name]
    return l ~= nil and l.type:sub(1, 8) ~= "goaround"
end

local function loadPoints()
    POINTS, POINT_ORDER = {}, {}
    local f = io.open(POINTS_FILE, "r")
    if not f then return end
    for line in f:lines() do
        local t = line:match("^%s*(.-)%s*$")
        if t ~= "" and t:sub(1, 1) ~= ";" then
            local name, xs, zs = t:match("^(%a+)=(%-?%d+),(%-?%d+)$")
            if name then
                POINTS[name] = {name = name, x = tonumber(xs), z = tonumber(zs), kind = classify(name)}
                table.insert(POINT_ORDER, name)
            end
        end
    end
    f:close()
end

local function savePoints()
    local pois, vors = {}, {}
    for _, name in ipairs(POINT_ORDER) do
        local p = POINTS[name]
        if p then
            if p.kind == "vor" then
                table.insert(vors, p)
            else
                table.insert(pois, p)
            end
        end
    end
    local f = io.open(POINTS_FILE, "w")
    f:write("; aerodromes and/or point-of-interests (always three letters)\n")
    for _, p in ipairs(pois) do
        f:write(string.format("%s=%d,%d\n", p.name, math.floor(p.x + 0.5), math.floor(p.z + 0.5)))
    end
    f:write("; waypoints (VORs are always five letters long)\n")
    for _, p in ipairs(vors) do
        f:write(string.format("%s=%d,%d\n", p.name, math.floor(p.x + 0.5), math.floor(p.z + 0.5)))
    end
    f:write("\n")
    f:close()
end

local function loadCourses()
    COURSES, COURSE_ORDER = {}, {}
    local f = io.open(COURSE_FILE, "r")
    if not f then return end
    for line in f:lines() do
        local t = line:match("^%s*(.-)%s*$")
        if t ~= "" and t:sub(1, 1) ~= ";" then
            local name, rest = t:match("^([%w%-]+)=(.+)$")
            if name then
                local seq = {}
                for p in rest:gmatch("[^,]+") do table.insert(seq, p) end
                COURSES[name] = {name = name, seq = seq}
                table.insert(COURSE_ORDER, name)
            end
        end
    end
    f:close()
end

local function saveCourses()
    local f = io.open(COURSE_FILE, "w")
    f:write("; routes\n")
    for _, name in ipairs(COURSE_ORDER) do
        local c = COURSES[name]
        if c then
            f:write(name .. "=" .. table.concat(c.seq, ",") .. "\n")
        end
    end
    f:close()
end

local function pointInUseBy(name)
    local used = {}
    for _, cname in ipairs(COURSE_ORDER) do
        local c = COURSES[cname]
        if c then
            for _, p in ipairs(c.seq) do
                if p == name then
                    table.insert(used, cname)
                    break
                end
            end
        end
    end
    return used
end

local function validatePointName(name)
    if not name or name == "" then return false, "Name required" end
    if not name:match("^%u+$") then return false, "Letters only" end
    if #name ~= 3 and #name ~= 5 then
        return false, "Name must be 3 (POI) or 5 (VOR) letters"
    end
    return true
end

local function addPoint(name, x, z)
    local ok, err = validatePointName(name)
    if not ok then return false, err end
    if POINTS[name] then return false, "Point " .. name .. " already exists" end
    POINTS[name] = {name = name, x = x, z = z, kind = classify(name)}
    table.insert(POINT_ORDER, name)
    savePoints()
    return true
end

local function editPoint(oldName, newName, x, z)
    local ok, err = validatePointName(newName)
    if not ok then return false, err end
    if newName ~= oldName and POINTS[newName] then
        return false, "Point " .. newName .. " already exists"
    end
    POINTS[oldName] = nil
    for i, n in ipairs(POINT_ORDER) do
        if n == oldName then
            POINT_ORDER[i] = newName
            break
        end
    end
    POINTS[newName] = {name = newName, x = x, z = z, kind = classify(newName)}
    if newName ~= oldName then
        for _, cname in ipairs(COURSE_ORDER) do
            local c = COURSES[cname]
            if c then
                for i, p in ipairs(c.seq) do
                    if p == oldName then c.seq[i] = newName end
                end
            end
        end
        saveCourses()
    end
    savePoints()
    return true
end

local function deletePoint(name)
    local used = pointInUseBy(name)
    if #used > 0 then
        return false, "In use by: " .. table.concat(used, ", ")
    end
    POINTS[name] = nil
    for i, n in ipairs(POINT_ORDER) do
        if n == name then
            table.remove(POINT_ORDER, i)
            break
        end
    end
    savePoints()
    return true
end

local function validateCourseName(name)
    if not name or name == "" then return false, "Name required" end
    if not name:match("^[%u%d%-]+$") then return false, "Use A-Z, 0-9, - only" end
    return true
end

local function addCourse(name, seq)
    local ok, err = validateCourseName(name)
    if not ok then return false, err end
    if COURSES[name] then return false, "Course " .. name .. " already exists" end
    if #seq < 2 then return false, "Need at least 2 waypoints" end
    COURSES[name] = {name = name, seq = seq}
    table.insert(COURSE_ORDER, name)
    saveCourses()
    return true
end

local function editCourse(oldName, newName, seq)
    local ok, err = validateCourseName(newName)
    if not ok then return false, err end
    if #seq < 2 then return false, "Need at least 2 waypoints" end
    if newName ~= oldName and COURSES[newName] then
        return false, "Course " .. newName .. " already exists"
    end
    COURSES[oldName] = nil
    for i, n in ipairs(COURSE_ORDER) do
        if n == oldName then
            COURSE_ORDER[i] = newName
            break
        end
    end
    COURSES[newName] = {name = newName, seq = seq}
    saveCourses()
    return true
end

local function deleteCourse(name)
    COURSES[name] = nil
    for i, n in ipairs(COURSE_ORDER) do
        if n == name then
            table.remove(COURSE_ORDER, i)
            break
        end
    end
    saveCourses()
    return true
end

-- NAV ENGINE ---------------------------------------------------------------
-- NAV.points is always a snapshot (copied x/y/kind at the moment the plan was
-- set), never a live reference into POINTS -- so editing the point database
-- mid-flight cannot retroactively move a target already being flown.

local NAV = {
    active = false,
    planName = nil,
    points = {},   -- array of {name, x, z, kind}
    index = 1,
    posX = 0, posZ = 0,
    haveFix = false,
    phase = "ENROUTE",  -- ENROUTE | APPROACH | TERMINAL | ARRIVED -- see phaseFor() below;
                        -- this is the one thing navcom now takes on faith rather than
                        -- guessing from its own distance thresholds.
    -- Pilot cruise-speed selection: "full" | "slow" | "coast" (see SPEED_* / cruiseCap).
    cruiseMode = "full",
    -- Landing:
    landAtEnd = false,  -- the final point is a landing site to auto-land at on arrival
    landSite  = nil,    -- snapshot {name, dir, alt, type} taken when the plan is set, so
                        -- editing landing.db mid-flight cannot move a landing in progress
    -- Extra pose, read every tick for the landing state machine.
    posY = 0, velY = 0, velX = 0, velZ = 0, heading = 0, haveHeading = false, groundSpeed = 0,
    cmdSpeed = 0,   -- rise-rate-limited commanded speed (dynamicSpeedTarget through DC.ACCEL_MAX);
                    -- the value actually sent as cruiseCap and shown as CMD, updated once per navTick.
    -- Route start: hold throttle at 0 until the nose is within LG.ALIGN_HDG_DEG of the first target.
    aligning = false,
    aimX = nil, aimZ = nil,   -- the point broadcastCurrent last steered to (carrot or waypoint)
    -- "This plan must be flown FROM ITS FIRST WAYPOINT, not joined at the nearest point."
    -- Normal plans are the opposite -- joining wherever the aircraft already is, is the whole
    -- point of RESUME -- but a go-around circuit starts BEHIND the aeroplane and beside the
    -- runway the circuit ends at, so locating ourselves by projection onto it would report the
    -- entire circuit as already flown. Cleared the moment the first waypoint is genuinely
    -- reached; until then the plan is navigated waypoint-to-waypoint (see PLAN.onPath).
    fromStart = false,
}

-- Auto-landing sequence state. Distinct from NAV: the plan can be flown (NAV) long
-- before the landing (LND) actually engages on arrival, and the landing outlives the
-- plan by a moment at touchdown (NAV is cancelled, LND lingers on the LANDED screen).
-- It also carries the runway-approach machinery itself (LND.rwy* below the nav engine):
-- functions on the state table rather than a dozen more top-level locals, the same shape TD
-- uses -- see the 200-locals note in the DC block.
local LND = {
    active = false,
    phase  = "OFF",     -- pad:    OFF | ALIGN | TRANSLATE | DESCEND | FLARE | LANDED
                        -- runway: OFF | INTERCEPT | GLIDE | FLARE | ROLLOUT | MISSED | LANDED
    paused = false,     -- pilot overrode the descent (AS override): hand VERTICAL to
                        -- ALT-AP while position/heading alignment keeps running. Set by
                        -- overrideLoop, cleared by RESUME LANDING / any teardown.
    x = 0, z = 0, name = "",
    holdHeading = nil,  -- radians, or nil when the site's direction is 'x' (any)
    siteAlt = 0, siteType = "heli",
    vs = 0,             -- current commanded vertical speed (m/s, negative = down)
    agl = 0, posErr = 0, hdgErrDeg = 0,
    settle = 0,         -- consecutive "not sinking" ticks, for the touchdown backup
    tdBy = nil,         -- which test called the touchdown ("squat" | "AGL" | "settle"), for the HUD
    vertHold = false,   -- this tick the vertical channel is told "hold", not "descend"

    -- ---- runway approach (LG.RWY_*; all zero/false for a pad landing) ----
    rwy   = false,      -- this landing is a straight-in runway approach, not a pad descent
    ux = 0, uz = 0,     -- unit vector along the runway, pointing the way we land
    gsTan = 0,          -- tan(glideslope)
    aimX = 0, aimZ = 0, -- touchdown aim point (a little way past the threshold)
    stopX = 0, stopZ = 0,   -- where the rollout is meant to finish (short of the far end)
    endX = 0, endZ = 0,     -- the far end itself -- for the map, not for guidance
    rwyLen = 0,             -- ...and its distance from the threshold, likewise
    tgtX = 0, tgtZ = 0,     -- the point currently broadcast to the UAC (carrot, or stop point)
    holdX = 0, holdZ = 0,   -- where a missed approach levels off and stops
    -- ---- go-around (LG.RWY_GA_*; see LND.goAround) ----
    ga    = false,      -- a published go-around circuit is being flown. LND.active is FALSE:
                        -- the ROUTE machinery has the horizontal, and all this flag means is
                        -- that the FMS still owns the vertical channel (holding LND.gaAlt).
    gaWp  = false,      -- ...and it is the PUBLISHED-WAYPOINT kind, not the straight-leg kind.
                        -- The horizontal is the same either way (a plan is a plan); this picks
                        -- the vertical profile, which is not (see LND.gaTick).
    gaAlt = 0,          -- ...that altitude, from the route's landing.db entry -- a minimum for
                        -- a waypoint circuit, the height to fly for a straight-leg one
    gaExitAlt = 0,      -- ...and the glidepath height to be at where the approach is rejoined
                        -- (straight-leg only: a waypoint circuit works this out as it goes)
    gaCount = 0,        -- attempts used on THIS landing; reset only when a new plan is set
    navPhase = "APPROACH",  -- ...and the phase it is broadcast with
    navHold = nil,          -- ...and the heading, once holding one matters (the rollout)
    glide = false,      -- tells the UAC this is a FLYING descent: it may keep steering by
                        -- bank even though the vertical channel is commanded down
    appSpeed = 0,       -- approach speed the glideslope and the runway length allow
    cmdSpeed = 0,       -- ...slewed, and what actually goes out as cruiseCap
    along = 0,          -- m still to run to the aim point, along the runway axis
    xtk = 0,            -- m off the extended centreline (+ = right of it)
    gsDev = 0,          -- m above (+) / below (-) the nominal glidepath
}

-- SQUAT SWITCH ---------------------------------------------------------------
-- The gear now has a real weight-on-wheels contact: computer #15 (manconrelay_rotation.lua)
-- reads it off a redstone input and broadcasts {touchedDown = <bool>} on TD.PROTOCOL. That is
-- ground TRUTH, where AGL is only an inference -- landing.db's pad altitude can be a metre or
-- two out, and a pad can be rebuilt higher. So landingTick takes touchdown from EITHER this
-- signal or the AGL test, whichever trips first.
--
-- Two guards, because a false touchdown at height cuts the navigation and drops the craft:
--   * FRESH -- the signal only counts while #15 is actually being heard from. Without this the
--     last value would latch forever: #15 off, out of range or crashed with a stale "true" from
--     the previous landing would cut the NEXT descent at altitude.
--   * HOLD  -- it has to stay on for TD.HOLD seconds, so one bounce of the contact (or a
--     redstone flicker as the gear brushes something) is not a landing.
-- With no sender at all, TD.confirmed() is simply always false and the landing falls back to
-- the AGL test plus the settle backup -- exactly how this program behaved before.
local TD = {
    PROTOCOL = CFG.TOUCHDOWN.PROTOCOL,   -- link + guards: see CFG.TOUCHDOWN in apconf.lua
    HOLD     = CFG.TOUCHDOWN.HOLD,
    STALE    = CFG.TOUCHDOWN.STALE,

    signal    = false, -- last value received
    since     = nil,   -- os.clock() when the signal last went true (nil while it reads false)
    lastHeard = nil,   -- os.clock() of the last message, whatever its value
}

-- Is the sensor alive? Also drives the HUD line, since "no signal" is the difference between
-- a landing that can end on the gear contact and one that is back to AGL alone.
function TD.fresh()
    return TD.lastHeard ~= nil and (os.clock() - TD.lastHeard) <= TD.STALE
end

-- The debounced, freshness-gated "we are on the ground" that the landing machine trusts.
function TD.confirmed()
    return TD.fresh() and TD.signal and TD.since ~= nil
        and (os.clock() - TD.since) >= TD.HOLD
end

-- Its own listener rather than a poll inside navTick: the sender broadcasts on its own
-- cadence, and the guards above are written against os.clock() stamps, not tick counts.
function TD.loop()
    while true do
        local sender, msg = rednet.receive(TD.PROTOCOL, 1)
        if sender and type(msg) == "table" and msg.touchedDown ~= nil then
            -- redstone.getInput hands back a boolean; tolerate 0/1 rather than read a 1 as
            -- "airborne" if the sender is ever rewritten to send numbers.
            local down = (msg.touchedDown ~= false and msg.touchedDown ~= 0)
            if down and not TD.signal then TD.since = os.clock() end   -- rising edge starts the hold
            if not down then TD.since = nil end
            TD.signal, TD.lastHeard = down, os.clock()
        end
    end
end

-- True on the penultimate or final leg of a multi-point plan (never on a
-- single-point direct-to, which has no penultimate leg to speak of). Shared
-- by radiusFor and phaseFor so the two stay in step: a VOR sitting in that
-- tier gets the same tighter tolerance that also decides when APPROACH phase
-- kicks in.
-- True only on the FINAL leg (the actual approach to the destination). Previously this also
-- covered the PENULTIMATE leg, but that made a fly-THROUGH second-to-last waypoint act like an
-- arrival -- dropping curve-following and, if the scheduled-speed flag lapsed, capping at the
-- UAC's APPROACH_SPEED (30). A mid-route fix should stay ENROUTE (curve + cruise); only the very
-- last leg approaches the destination, and the curvature speed profile already brakes for the
-- endpoint on its own (endpoint = FINAL_ARRIVE_SPEED) whether the phase says APPROACH or not.
local function isApproachLeg()
    local total = #NAV.points
    return total >= 2 and NAV.index >= total
end

-- RAD.VOR_RADIUS is sized for a cruise-speed fly-by, but a VOR that lands on the
-- penultimate or final leg is being approached slowly, not overflown at
-- speed -- holding it to that wide a tolerance there just delays the
-- slowdown/position-hold that phase is already supposed to be triggering. So
-- in that tier a VOR uses the same tighter radius a mid-route POI would.
local function radiusFor(pt, isFinal)
    if pt.kind == "vor" then
        if isApproachLeg() then return RAD.POI_ENROUTE_RADIUS end
        return RAD.VOR_RADIUS
    end
    if isFinal then return RAD.POI_FINAL_RADIUS end
    return RAD.POI_ENROUTE_RADIUS
end

local function legLength(a, b)
    local dx, dz = b.x - a.x, b.z - a.z
    return math.sqrt(dx * dx + dz * dz)
end

-- Distance-to-go for every waypoint from index onward: aircraft -> current
-- target, then leg by leg out to each following fix. Passed waypoints (before
-- index) are left out of the table; the UI shows those as PASSED instead.
local function remainingDistances()
    local res = {}
    if not NAV.active or #NAV.points == 0 then return res end
    local dx = NAV.points[NAV.index].x - NAV.posX
    local dz = NAV.points[NAV.index].z - NAV.posZ
    local acc = math.sqrt(dx * dx + dz * dz)
    res[NAV.index] = acc
    for i = NAV.index + 1, #NAV.points do
        acc = acc + legLength(NAV.points[i - 1], NAV.points[i])
        res[i] = acc
    end
    return res
end

-- Distance from the aircraft's last known fix to the current target. Used for
-- phase, not for the pass/advance check (which needs the fix taken at the
-- same instant as the advance decision -- see navTick).
local function currentDist()
    local tgt = NAV.points[NAV.index]
    if not tgt then return math.huge end
    local dx, dz = tgt.x - NAV.posX, tgt.z - NAV.posZ
    return math.sqrt(dx * dx + dz * dz)
end

-- This is the one piece of judgement navcom used to make for itself, from
-- fixed distance thresholds that had no idea whether the current target was a
-- VOR or a POI, final or not -- so they were always wrong for someone. This
-- program already knows the plan (which leg is penultimate, which is final)
-- and the same contextual pass radius used to decide when a waypoint counts
-- as "reached" (see radiusFor), so phase is derived from that, once, here --
-- navcom just receives the answer and flies accordingly.
--   ENROUTE  - normal cruise leg
--   APPROACH - penultimate leg of a multi-point plan, or its final leg while
--              still outside the final point's own pass radius: already slow,
--              not close yet
--   TERMINAL - inside the final point's pass radius: navcom switches from
--              along-track-plus-line-hold to direct position-hold
--   ARRIVED  - inside RAD.ARRIVED_FRACTION of that same radius
-- A single-point direct-to has no penultimate leg, so it stays ENROUTE (full
-- cruise) until it is genuinely close, rather than crawling the whole way.
local function phaseFor(dist)
    local total = #NAV.points
    if total == 0 then return "ENROUTE" end
    local tgt = NAV.points[NAV.index]
    local isFinal = (NAV.index == total)
    if isFinal then
        local radius = radiusFor(tgt, true)
        if dist <= radius * RAD.ARRIVED_FRACTION then
            return "ARRIVED"
        elseif dist <= radius then
            return "TERMINAL"
        elseif isApproachLeg() then
            return "APPROACH"
        else
            return "ENROUTE"
        end
    elseif isApproachLeg() then
        return "APPROACH"
    end
    return "ENROUTE"
end

local function updatePhase()
    NAV.phase = phaseFor(currentDist())
end

-- The numeric cruise cap (m/s) for the current pilot selection, and a short label for it.
local function cruiseCapValue()
    if NAV.cruiseMode == "slow"  then return SPEED_SLOW end
    if NAV.cruiseMode == "coast" then return 0 end
    return SPEED_FULL
end

local function cruiseModeLabel()
    if NAV.cruiseMode == "slow"  then return "SLOW " .. SPEED_SLOW end
    if NAV.cruiseMode == "coast" then return "COAST 0" end
    return "FULL " .. SPEED_FULL
end

-- Compass-frame direction of the leg a->b, in navcom's bearing frame (atan2(dx, dz)).
local function legBearing(ax, az, bx, bz)
    return atan2(bx - ax, bz - az)
end

-- Turn angle (radians, 0..pi) the plan makes AT waypoint i: the course change between the leg
-- arriving at i and the leg leaving it. The arriving leg of the current target starts from the
-- aircraft (no flown-from fix yet); later waypoints use the previous fix. The final point (no
-- outgoing leg) is 0.
local function turnAngleAt(i)
    local p = NAV.points
    local total = #p
    if i < 1 or i >= total then return 0 end            -- final / out of range: no outgoing leg
    local ax, az
    if i - 1 >= 1 then ax, az = p[i-1].x, p[i-1].z
    else ax, az = NAV.posX, NAV.posZ end                 -- first leg: arrive from the aircraft
    local inB  = legBearing(ax, az, p[i].x, p[i].z)
    local outB = legBearing(p[i].x, p[i].z, p[i+1].x, p[i+1].z)
    return math.abs(wrapPi(outB - inB))
end

-- PLAN: the pre-flight-computed path -- replaces the old per-tick Catmull-Rom CURVE table.
-- Built ONCE by PLAN.build(), called when the pilot confirms a plan (setDirectTo / setRoute /
-- RTR.setSolvedRoute), not re-evaluated every tick. Each interior waypoint gets an explicit
-- LineSegment -> ClothoidSegment -> CircularArc -> ClothoidSegment -> LineSegment fly-by
-- transition sized from a real turn radius (R = V^2/(GRAVITY*tan(bank))); curvature ramps
-- linearly through each clothoid instead of jumping at a spline knot, so lateral accel -- and
-- its rate of change, jerk -- is continuous through a turn. The whole route (straights +
-- transitions) is walked once into a dense arc-length-sampled polyline with the speed profile
-- baked in, so carrot/speed/course/passed lookups at runtime are near-O(1), not a per-tick walk.
local PLAN = {
    samples  = {},  -- arc-length ordered: {x, z, hdg, kappa, s, speed}
    totalLen = 0,
    wpS      = {},  -- wpS[i] = arc length of waypoint i's "abeam" point (its transition
                     -- midpoint, or the waypoint itself where it isn't rounded; 0 and totalLen
                     -- for the two ENDS) -- what PLAN.passed compares against to advance NAV.index
    cursor   = 1,   -- last-known nearest-sample index; advances monotonically at runtime
    lastX = nil, lastZ = nil, lastS = 0,       -- cache for updateCursor (see below)
    lastD2 = math.huge,                        -- ...and how far off the path that left us (m^2)
    SEARCH_WINDOW = 120,                        -- samples either side of cursor to search each tick
    RESCAN_DIST2  = 1000 * 1000,                 -- if the window search is still this far off (m^2),
                                                 -- do one full O(n) rescan (route resume, teleport, etc.)
}

-- Exact heading (rad, relative to the transition's own start heading) at distance l along a
-- symmetric entry-clothoid / arc / exit-clothoid profile: curvature ramps 0->1/R over the
-- first Ls, holds at 1/R for the middle La, then ramps back to 0 over the last Ls. sigma =
-- +1 (right turn) / -1 (left turn). Exact for any l in [0, 2*Ls+La]: the curvature profile is
-- piecewise-LINEAR in l, so heading (its integral) is piecewise-quadratic -- no numeric
-- integration needed for heading, only for position (PLAN.marchTransition, below).
function PLAN.transHdg(l, Ls, R, La, sigma)
    if l <= Ls then
        return sigma * l * l / (2 * Ls * R)
    elseif l <= Ls + La then
        return sigma * (Ls / (2 * R) + (l - Ls) / R)
    else
        local m = 2 * Ls + La - l
        return sigma * (Ls / R + La / R - m * m / (2 * Ls * R))
    end
end

-- Canonical (right-turn, start heading 0) spiral end point in a LOCAL frame: used only to
-- solve a corner's exact tangent length (PLAN.solveCorner's p/k/T), not for placing samples
-- (PLAN.marchTransition does that, in world coordinates, with the corner's real R/Ls/La/sigma).
-- Numeric (fine sub-stepped) integration of the exact heading above -- a one-time/offline cost,
-- so there's no need for the truncated-series shift/tangent approximations highway design
-- normally uses; plain integration is exact to any tolerance we sample at.
function PLAN.spiralEnd(Ls, R)
    if Ls <= 1e-6 then return 0, 0, 0 end
    local n = math.max(8, math.floor(Ls / 2))
    local h, sub = Ls / n, 4
    local x, z = 0, 0
    for i = 1, n do
        local lPrev, subH = (i - 1) * h, h / 4
        for j = 1, sub do
            local l = lPrev + (j - 0.5) * subH
            local hdg = PLAN.transHdg(l, Ls, R, 0, 1)
            x, z = x + math.sin(hdg) * subH, z + math.cos(hdg) * subH
        end
    end
    return x, z, PLAN.transHdg(Ls, Ls, R, 0, 1)
end

-- March a straight run from (x,z) to the exact target (tx,tz) at heading hdg, appending
-- samples to `out` at roughly `spacing` intervals (snapped exactly onto the target on the
-- last sample, so straight runs never drift from floating-point accumulation).
function PLAN.marchStraightInto(x, z, hdg, tx, tz, s, spacing, out)
    local len = math.sqrt((tx - x) * (tx - x) + (tz - z) * (tz - z))
    if len <= 1e-6 then return s end
    local n = math.max(1, math.floor(len / spacing + 0.5))
    for i = 1, n do
        local f = i / n
        s = s + len / n
        table.insert(out, {x = x + (tx - x) * f, z = z + (tz - z) * f, hdg = hdg, kappa = 0, s = s})
    end
    return s
end

-- March the actual clothoid-arc-clothoid transition in world coordinates, from a start pose
-- (x0,z0,hdg0) -- the incoming leg's tangent point -- through to the outgoing tangent point.
-- Position is numerically integrated (fine sub-stepped) from the exact heading formula above;
-- appends samples to `out` roughly every `spacing`. Returns the end (s,x,z,hdg) and the
-- transition's midpoint arc length (PLAN.build records this as the waypoint's abeam point).
function PLAN.marchTransition(x0, z0, hdg0, Ls, R, La, sigma, s0, spacing, out)
    local Ltot = 2 * Ls + La
    if Ltot <= 1e-6 then return s0, x0, z0, hdg0, s0 end
    local n = math.max(2, math.floor(Ltot / spacing + 0.5))
    local h, sub = Ltot / n, 4
    local x, z, s, sMid = x0, z0, s0, nil
    for i = 1, n do
        local lPrev, subH = (i - 1) * h, h / 4
        for j = 1, sub do
            local lMid = lPrev + (j - 0.5) * subH
            local hdg = hdg0 + PLAN.transHdg(lMid, Ls, R, La, sigma)
            x, z = x + math.sin(hdg) * subH, z + math.cos(hdg) * subH
        end
        s = s + h
        local lNow = i * h
        local hdgNow = hdg0 + PLAN.transHdg(lNow, Ls, R, La, sigma)
        local kfrac = (lNow <= Ls) and (lNow / Ls) or ((lNow <= Ls + La) and 1 or (Ltot - lNow) / Ls)
        table.insert(out, {x = x, z = z, hdg = hdgNow, kappa = sigma * kfrac / R, s = s})
        if not sMid and lNow >= Ls + La / 2 then sMid = s end
    end
    return s, x, z, hdg0 + PLAN.transHdg(Ltot, Ls, R, La, sigma), sMid or s
end

-- Binary search an arc-length-sorted sample array for the index whose s is closest to (and
-- not past) the given arc length. Shared by the offline solver (corner speed feedback) and
-- the runtime carrot/speed/course lookups.
function PLAN.sampleIndexAtS(samples, s)
    local lo, hi = 1, #samples
    if hi == 0 then return 0 end
    if s <= samples[1].s then return 1 end
    if s >= samples[hi].s then return hi end
    while hi - lo > 1 do
        local mid = math.floor((lo + hi) / 2)
        if samples[mid].s <= s then lo = mid else hi = mid end
    end
    return lo
end

-- Solve one corner's clothoid-arc-clothoid transition for a planned speed V and deflection
-- Delta (rad, unsigned; sigma = +1 right / -1 left). Radius is clamped to [MIN_RADIUS,
-- MAX_RADIUS] and, by shrinking, to whatever fits within LEG_FIT_FRACTION of the SHORTER
-- adjacent leg -- the "maximum radius for this corner" constraint, so neighbouring corners on
-- a short leg can never overlap. Returns nil for a near-zero deflection (straight through).
function PLAN.solveCorner(Delta, sigma, V, legIn, legOut)
    if Delta < 1e-4 then return nil end
    local turnAccel = DC.GRAVITY * math.tan(math.rad(DC.MAX_BANK_DEG))
    local maxT = DC.LEG_FIT_FRACTION * math.min(legIn, legOut)

    local function forRadius(R)
        local Ls = (V * V * V) / (R * DC.JERK_MAX)
        local thetaS = Ls / (2 * R)
        if thetaS > Delta / 2 then
            thetaS = Delta / 2
            Ls = thetaS * 2 * R
        end
        local arcLen = math.max(0, (Delta - 2 * thetaS) * R)
        local lx, lz, ths = PLAN.spiralEnd(Ls, R)
        local p = lx - R * (1 - math.cos(ths))
        local k = lz - R * math.sin(ths)
        local T = k + (R + p) * math.tan(Delta / 2)
        return Ls, arcLen, T
    end

    local R = clamp(V * V / turnAccel, DC.MIN_RADIUS, DC.MAX_RADIUS)
    local Ls, arcLen, T = forRadius(R)
    if T > maxT then
        -- Shrink the radius until the transition fits on the shorter adjacent leg. A plain
        -- decay search (not bisection) because T(R) isn't guaranteed perfectly monotonic once
        -- the jerk-limited clothoid length Ls also grows as R shrinks -- this is a one-time,
        -- offline computation, so a few dozen extra steps cost nothing.
        for _ = 1, 80 do
            R = math.max(DC.MIN_RADIUS, R * 0.95)
            Ls, arcLen, T = forRadius(R)
            if T <= maxT or R <= DC.MIN_RADIUS then break end
        end
    end
    return { R = R, Ls = Ls, arcLen = arcLen, T = T, sigma = sigma, Delta = Delta }
end

-- Walk the whole route once: a straight run into each interior vertex's transition start, the
-- transition itself, a straight run to the next vertex (or straight through, for a
-- near-zero-deflection vertex with no `corners[i]`), ... a final straight run into the last
-- waypoint. Appends world-frame samples to `out`; fills wpS[i] with each interior waypoint's
-- abeam arc length (transition midpoint, or the waypoint itself if unrounded).
function PLAN.buildGeometry(pts, corners, out, wpS)
    local n = #pts
    local x, z, s = pts[1].x, pts[1].z, 0
    table.insert(out, {x = x, z = z, hdg = legBearing(x, z, pts[2].x, pts[2].z), kappa = 0, s = 0})
    -- The FIRST waypoint's abeam point is the start of the path. It has no transition, so the
    -- interior loop below never sets it -- and a nil here used to make PLAN.passed(1) return
    -- false forever, which is what stranded NAV.index on waypoint 1 whenever the aircraft
    -- joined a plan without physically flying through that waypoint's pass radius.
    wpS[1] = 0
    for i = 2, n - 1 do
        local c = corners[i]
        local bIn = legBearing(pts[i - 1].x, pts[i - 1].z, pts[i].x, pts[i].z)
        if c then
            local tsX, tsZ = pts[i].x - c.T * math.sin(bIn), pts[i].z - c.T * math.cos(bIn)
            s = PLAN.marchStraightInto(x, z, bIn, tsX, tsZ, s, DC.SAMPLE_SPACING, out)
            local sMid
            s, x, z, _, sMid = PLAN.marchTransition(tsX, tsZ, bIn, c.Ls, c.R, c.arcLen, c.sigma, s, DC.SAMPLE_SPACING, out)
            wpS[i] = sMid
        else
            s = PLAN.marchStraightInto(x, z, bIn, pts[i].x, pts[i].z, s, DC.SAMPLE_SPACING, out)
            x, z = pts[i].x, pts[i].z
            wpS[i] = s
        end
    end
    local bInLast = legBearing(pts[n - 1].x, pts[n - 1].z, pts[n].x, pts[n].z)
    s = PLAN.marchStraightInto(x, z, bInLast, pts[n].x, pts[n].z, s, DC.SAMPLE_SPACING, out)
    wpS[n] = s   -- ...and the last one's is the end of it. PLAN.passed is never asked about the
                 -- final waypoint (arrival is the pass-radius test's job), but leaving a hole
                 -- at either end of this table is exactly the trap that produced the bug above.
    return s
end

-- Two-pass velocity profile, the standard way to turn per-point speed LIMITS into an
-- achievable speed PROFILE: a backward braking pass from the route end, then a forward
-- acceleration pass. Replaces the old per-tick single forward-braking scan (CURVE.speed) --
-- this runs once, over the WHOLE route, so there's no look-ahead distance limit either.
-- Mutates sample.speed in place.
function PLAN.bakeSpeedProfile(samples, limits, brakeDecel, accelMax, endSpeed)
    local n = #samples
    if n == 0 then return end
    samples[n].speed = math.min(limits[n] or endSpeed, endSpeed)
    for i = n - 1, 1, -1 do
        local ds = samples[i + 1].s - samples[i].s
        samples[i].speed = math.min(limits[i], math.sqrt(samples[i + 1].speed ^ 2 + 2 * brakeDecel * ds))
    end
    for i = 2, n do
        local ds = samples[i].s - samples[i - 1].s
        samples[i].speed = math.min(samples[i].speed, math.sqrt(samples[i - 1].speed ^ 2 + 2 * accelMax * ds))
    end
end

-- A SEPARATE, steeper backward-only sweep from the route end (DC.ARRIVE_DECEL, not
-- BRAKE_DECEL), min'd into the already-baked profile. bakeSpeedProfile's corner braking is
-- tuned for comfortable cornering and can legitimately leave the craft at full cruise right
-- up to the last waypoint on a gentle final approach -- this guarantees the destination
-- stop itself is never out-run by that, regardless of how short the final leg turns out to
-- be, by capping every sample to what ARRIVE_DECEL can actually stop from in the remaining
-- distance.
function PLAN.applyArriveBrake(samples, decel, endSpeed)
    local n = #samples
    if n == 0 then return end
    local v = endSpeed
    samples[n].speed = math.min(samples[n].speed, v)
    for i = n - 1, 1, -1 do
        local ds = samples[i + 1].s - samples[i].s
        v = math.sqrt(v * v + 2 * decel * ds)
        samples[i].speed = math.min(samples[i].speed, v)
    end
end

-- THE offline planning pipeline: waypoints -> leg definitions -> turn anticipation -> fly-by
-- transitions -> iterate speed<->geometry to convergence -> bake the sampled polyline that
-- PLAN.carrot/speedAt/course/passed serve at runtime. Called once when the pilot confirms a
-- plan (setDirectTo / setRoute / RTR.setSolvedRoute).
function PLAN.build(pts)
    PLAN.samples, PLAN.wpS, PLAN.cursor = {}, {}, 1
    PLAN.lastX, PLAN.lastZ, PLAN.lastS, PLAN.lastD2 = nil, nil, 0, math.huge
    local n = #pts
    if n < 3 then
        -- Nothing to round -- just the straight run between however many points exist. A lone
        -- point stays at SPEED_FULL (there's no path to bake a brake into) -- the runtime
        -- destination-arrival brake in dynamicSpeedTarget handles the actual slowdown, same as
        -- the old CURVE.speed's n<2 passthrough did.
        local samples, s = {}, 0
        if n == 1 then
            table.insert(samples, {x = pts[1].x, z = pts[1].z, hdg = 0, kappa = 0, s = 0, speed = SPEED_FULL})
        elseif n == 2 then
            local bIn = legBearing(pts[1].x, pts[1].z, pts[2].x, pts[2].z)
            table.insert(samples, {x = pts[1].x, z = pts[1].z, hdg = bIn, kappa = 0, s = 0})
            s = PLAN.marchStraightInto(pts[1].x, pts[1].z, bIn, pts[2].x, pts[2].z, 0, DC.SAMPLE_SPACING, samples)
            for _, sm in ipairs(samples) do sm.speed = SPEED_FULL end
            PLAN.applyArriveBrake(samples, DC.ARRIVE_DECEL, DC.FINAL_ARRIVE_SPEED)
            PLAN.wpS[1], PLAN.wpS[2] = 0, s   -- same endpoints as buildGeometry fills in
        end
        PLAN.samples, PLAN.totalLen = samples, s
        return
    end

    local turnAccel = DC.GRAVITY * math.tan(math.rad(DC.MAX_BANK_DEG))
    local legs = {}
    for i = 1, n - 1 do legs[i] = legLength(pts[i], pts[i + 1]) end

    -- Turn anticipation: a rough first guess, sharper angle -> slower -- just a seed for the
    -- iteration below, doesn't need to be accurate.
    local speedGuess, delta, sigma = {}, {}, {}
    for i = 2, n - 1 do
        local bIn  = legBearing(pts[i - 1].x, pts[i - 1].z, pts[i].x, pts[i].z)
        local bOut = legBearing(pts[i].x, pts[i].z, pts[i + 1].x, pts[i + 1].z)
        local signedTurn = wrapPi(bOut - bIn)
        delta[i] = math.abs(signedTurn)
        sigma[i] = (signedTurn >= 0) and 1 or -1
        speedGuess[i] = SPEED_FULL / (1 + delta[i])
    end

    local corners, prevR = {}, {}
    for iter = 1, DC.PLAN_MAX_ITERS do
        local maxDelta = 0
        for i = 2, n - 1 do
            if delta[i] > 1e-4 then
                local c = PLAN.solveCorner(delta[i], sigma[i], speedGuess[i], legs[i - 1], legs[i])
                corners[i] = c
                local r = c and c.R or 0
                local dr = math.abs(r - (prevR[i] or -1))
                if dr > maxDelta then maxDelta = dr end
                prevR[i] = r
            end
        end

        local samples, wpS = {}, {}
        PLAN.buildGeometry(pts, corners, samples, wpS)
        local limits = {}
        for k, sm in ipairs(samples) do
            limits[k] = (math.abs(sm.kappa) > 1e-9) and math.sqrt(turnAccel / math.abs(sm.kappa)) or SPEED_FULL
        end
        PLAN.bakeSpeedProfile(samples, limits, DC.BRAKE_DECEL, DC.ACCEL_MAX, DC.FINAL_ARRIVE_SPEED)
        PLAN.applyArriveBrake(samples, DC.ARRIVE_DECEL, DC.FINAL_ARRIVE_SPEED)

        if iter == DC.PLAN_MAX_ITERS or maxDelta < DC.PLAN_CONV_EPS then
            PLAN.samples, PLAN.wpS, PLAN.totalLen = samples, wpS, samples[#samples].s
            break
        end

        -- Feed each corner its own achievable speed (read off the baked profile at its
        -- transition midpoint) back in for the next geometry pass -- a corner that got
        -- leg-fit-clamped can't actually be taken at the speed it was guessed at.
        for i = 2, n - 1 do
            if wpS[i] then
                speedGuess[i] = samples[PLAN.sampleIndexAtS(samples, wpS[i])].speed
            end
        end
    end
end

-- Called once per navTick (and defensively by carrot/speedAt/course/passed below, in case
-- they run from a different loop/redraw cadence): find the sample nearest the aircraft's
-- position. Searches a small window around the cached cursor (the craft moves forward
-- continuously, so this is cheap and never needs a full scan) except after a big jump (route
-- resume, teleport), when one full O(n) rescan relocates it. Cached against repeated
-- same-position calls within one tick.
function PLAN.updateCursor(x, z)
    if x == PLAN.lastX and z == PLAN.lastZ then return PLAN.lastS end
    local samples = PLAN.samples
    local n = #samples
    if n == 0 then
        PLAN.lastX, PLAN.lastZ, PLAN.lastS, PLAN.lastD2 = x, z, 0, math.huge
        return 0
    end
    local lo = math.max(1, PLAN.cursor - PLAN.SEARCH_WINDOW)
    local hi = math.min(n, PLAN.cursor + PLAN.SEARCH_WINDOW)
    local bestI, bestD = PLAN.cursor, math.huge
    for i = lo, hi do
        local dx, dz = samples[i].x - x, samples[i].z - z
        local d = dx * dx + dz * dz
        if d < bestD then bestD, bestI = d, i end
    end
    if bestD > PLAN.RESCAN_DIST2 then
        for i = 1, n do
            local dx, dz = samples[i].x - x, samples[i].z - z
            local d = dx * dx + dz * dz
            if d < bestD then bestD, bestI = d, i end
        end
    end
    PLAN.cursor = bestI
    PLAN.lastX, PLAN.lastZ, PLAN.lastS = x, z, samples[bestI].s
    PLAN.lastD2 = bestD   -- how far off the path we actually are; PLAN.passed gates on it
    return PLAN.lastS
end

-- Carrot: `lookahead` m ahead of the aircraft along the planned path (guidance / bearing).
function PLAN.carrot(lookahead)
    PLAN.updateCursor(NAV.posX, NAV.posZ)
    local samples = PLAN.samples
    if #samples == 0 then return NAV.posX, NAV.posZ end
    local i = PLAN.sampleIndexAtS(samples, PLAN.lastS + lookahead)
    return samples[i].x, samples[i].z
end

-- Commanded speed at the aircraft's current position on the planned path: the baked profile,
-- linearly interpolated between the two bracketing samples. Replaces CURVE.speed's per-tick
-- forward-braking scan -- it's already baked into PLAN.samples by PLAN.build.
function PLAN.speedAt()
    PLAN.updateCursor(NAV.posX, NAV.posZ)
    local samples = PLAN.samples
    local n = #samples
    if n == 0 then return SPEED_FULL end
    local i = PLAN.sampleIndexAtS(samples, PLAN.lastS)
    if i < n then
        local a, b = samples[i], samples[i + 1]
        local span = b.s - a.s
        local f = (span > 1e-6) and clamp((PLAN.lastS - a.s) / span, 0, 1) or 0
        return a.speed + (b.speed - a.speed) * f
    end
    return samples[i].speed
end

-- HSI course from the planned path at the aircraft's position: the sample heading (DTK) and
-- the signed perpendicular deviation (+=right of course, same sign as the leg CDI). nil if
-- there's no plan yet.
function PLAN.course()
    PLAN.updateCursor(NAV.posX, NAV.posZ)
    local samples = PLAN.samples
    if #samples == 0 then return nil end
    local sm = samples[PLAN.sampleIndexAtS(samples, PLAN.lastS)]
    local dx, dz = NAV.posX - sm.x, NAV.posZ - sm.z
    return sm.hdg, math.sin(sm.hdg) * dz - math.cos(sm.hdg) * dx
end

-- Has the aircraft's arc-length position passed interior waypoint i's abeam point (its
-- transition midpoint, or the waypoint itself if it wasn't rounded)? Replaces the old
-- tangent-plane checkpointPassed with a direct arc-length compare -- the plan already knows
-- exactly where each waypoint's abeam point is.
-- Is the aircraft actually ON the planned path? EVERYTHING the plan can tell us -- the
-- carrot, the baked speed, the course, which waypoint is behind us -- is read by projecting
-- the aircraft onto the polyline, and that projection only means anything while it is near it.
-- Off the path (a plan just set from somewhere else, a go-around circuit whose start lies
-- BEHIND us, a diversion, a teleport) the honest answer is "navigate straight to the active
-- waypoint until you have joined", which is what every caller below falls back to.
-- Without this a go-around projected onto its own new circuit at the far END of the path --
-- the aircraft being beside the runway, which the circuit finishes at -- and so declared the
-- whole circuit already flown, re-arming the approach in the same tick it was abandoned.
function PLAN.onPath()
    if #PLAN.samples == 0 or NAV.fromStart then return false end
    PLAN.updateCursor(NAV.posX, NAV.posZ)
    return PLAN.lastD2 <= DC.PASS_CORRIDOR * DC.PASS_CORRIDOR
end

function PLAN.passed(i)
    local wp = PLAN.wpS[i]
    if not wp then return false end
    -- The corridor matters most at i == 1, whose abeam point is s = 0: every projection is at
    -- or past it, so without it a craft anywhere off the route would "pass" the first waypoint
    -- from anywhere at all.
    if not PLAN.onPath() then return false end
    -- STRICTLY past, not "at": the aircraft parked exactly on the route start projects to
    -- s = 0 = wpS[1], and it has not passed anything yet. (For interior waypoints the two
    -- readings differ by at most one sample spacing, so this is immaterial there.)
    return PLAN.lastS > wp
end

-- GO-AROUND PATH ---------------------------------------------------------------
-- A go-around is NOT a list of waypoints. The en-route planner rounds a corner by cutting it
-- (a fly-by transition), which is right for a route and completely wrong here: the published
-- leg is an ORIENTED LINE SEGMENT that must be entered at its start, ON its heading, and flown
-- end to end -- and it may point anywhere at all relative to where the missed approach left us,
-- including straight back at it. Handed to the fly-by planner that produces the nonsense the
-- first attempt flew.
--
-- So the circuit is built as an explicit CURVATURE-LIMITED path instead: the shortest
-- turn-straight-turn (Dubins) join from where we are, pointing where we point, to the leg's
-- start pointing along the leg. That is one construction that covers every case -- already
-- aligned (it degenerates to a straight), a right-angle join, a full course reversal (it comes
-- out as the teardrop you would fly by hand) -- with no special cases and no assumption that
-- the leg is parallel to the runway or any particular length. The turn radius comes from the
-- speed we will actually be doing, which is why the circuit is flown at RWY_GA_SPEED and not
-- at cruise: you cannot spiral a 320 m/s aeroplane onto a 200 m leg.
local GA = {}

-- Unit vector along a heading, and the vector 90 degrees to its LEFT. Turning left increases
-- the heading in this frame (u(h+e) = u(h) + e*left(h)), which is what fixes every sign below.
function GA.dir(h)  return math.sin(h), math.cos(h) end
function GA.left(h) return math.cos(h), -math.sin(h) end

-- Centre of the turning circle for a craft at (x,z) heading h turning s (+1 left, -1 right).
function GA.centre(x, z, h, s, R)
    local lx, lz = GA.left(h)
    return x + s * R * lx, z + s * R * lz
end

-- Shortest turn-straight-turn path from (x0,z0,h0) to (x1,z1,h1) at radius R. Returns
-- {s1, d1, ls, hs, s2, d2, len} -- turn direction and arc ANGLE for each end, the straight's
-- length and heading, and the total. nil if no such path exists (the two circles overlap and
-- the geometry needs a turn-turn-turn solution; the caller flies straight on and re-tries,
-- which opens the geometry up).
function GA.dubins(x0, z0, h0, x1, z1, h1, R)
    local best
    for _, s1 in ipairs({1, -1}) do
        for _, s2 in ipairs({1, -1}) do
            local c1x, c1z = GA.centre(x0, z0, h0, s1, R)
            local c2x, c2z = GA.centre(x1, z1, h1, s2, R)
            local vx, vz = c2x - c1x, c2z - c1z
            local d = math.sqrt(vx * vx + vz * vz)
            local hs, ls
            if s1 == s2 then
                -- Same-handed turns: the connecting straight is simply parallel to the line
                -- joining the two centres.
                if d > 1e-6 then hs, ls = atan2(vx, vz), d end
            elseif d >= 2 * R then
                -- Opposite-handed: an inner tangent, which only exists once the circles are
                -- clear of each other.
                local th = atan2(vx, vz)
                hs = th - math.asin(R * (s2 - s1) / d)
                ls = math.sqrt(d * d - 4 * R * R)
            end
            if hs then
                local d1 = (s1 * (hs - h0)) % (2 * math.pi)
                local d2 = (s2 * (h1 - hs)) % (2 * math.pi)
                local len = R * (d1 + d2) + ls
                if not best or len < best.len then
                    best = {s1 = s1, d1 = d1, ls = ls, hs = hs, s2 = s2, d2 = d2, len = len}
                end
            end
        end
    end
    return best
end

-- Walk a straight into `out`, returning the arc length reached. Shares the sample shape the
-- en-route planner bakes (x, z, hdg, kappa, s, speed) so every runtime lookup -- carrot,
-- speed, course, which waypoint is behind us -- works on a go-around exactly as it does on a
-- route, with no code downstream knowing the difference.
function GA.straight(x, z, h, len, s0, out)
    local ux, uz = GA.dir(h)
    local n = math.max(1, math.ceil(len / DC.SAMPLE_SPACING))
    for i = 1, n do
        local f = (i / n) * len
        table.insert(out, {x = x + ux * f, z = z + uz * f, hdg = h, kappa = 0,
                           s = s0 + f, speed = LG.RWY_GA_SPEED})
    end
    return s0 + len, x + ux * len, z + uz * len
end

-- ...and an arc, turning s through `ang` radians at radius R.
function GA.arc(x, z, h, s, ang, R, s0, out)
    local cx, cz = GA.centre(x, z, h, s, R)
    local n = math.max(1, math.ceil(ang * R / DC.SAMPLE_SPACING))
    local ex, ez, eh = x, z, h
    for i = 1, n do
        local a = h + s * ang * (i / n)
        local lx, lz = GA.left(a)
        ex, ez, eh = cx - s * R * lx, cz - s * R * lz, a
        table.insert(out, {x = ex, z = ez, hdg = a, kappa = s / R,
                           s = s0 + ang * R * (i / n), speed = LG.RWY_GA_SPEED})
    end
    return s0 + ang * R, ex, ez, eh
end

-- HOW FAR TO FLY STRAIGHT FIRST. "Commence go-around, fly straight, THEN turn in" is not just
-- for separation: the join geometry needs room. Turning onto a leg that runs the SAME way as
-- you are already going, a few hundred metres to one side, is the hard case -- a curvature-
-- limited path has to arrive at an exact point on an exact heading, and from close abeam the
-- only way to do that is a full 360. Run on a little first and the same join becomes a gentle
-- S. So rather than guess a distance, search it: try running on 0, 50, 100 ... m and keep
-- whichever gives the shortest TOTAL path. One cheap one-dimensional search, done once, and it
-- needs no case analysis -- it produces the run-and-turn for a parallel leg and the teardrop
-- for a reversed one out of the same code.
function GA.bestRun(x, z, h, tx, tz, th, R, lo, hi)
    local ux, uz = GA.dir(h)
    local bestL, bestLen
    local step = math.max((hi - lo) / 40, 10)
    local L = lo
    while L <= hi do
        local p = GA.dubins(x + ux * L, z + uz * L, h, tx, tz, th, R)
        if p and (not bestLen or L + p.len < bestLen) then bestL, bestLen = L, L + p.len end
        L = L + step
    end
    return bestL
end

-- Fly a straight run-out and then a whole Dubins join into `out`. Returns the arc length
-- reached, or nil if no turn-straight-turn solution exists at any run-out length.
function GA.join(x, z, h, tx, tz, th, R, s0, out, lo, hi)
    local run = GA.bestRun(x, z, h, tx, tz, th, R, lo, hi)
    if not run then return nil end
    local s, cx, cz, ch = s0, x, z, h
    if run > 1e-6 then s, cx, cz = GA.straight(cx, cz, h, run, s, out) end
    local p = GA.dubins(cx, cz, ch, tx, tz, th, R)
    if not p then return nil end
    s, cx, cz, ch = GA.arc(cx, cz, ch, p.s1, p.d1, R, s, out)
    if p.ls > 1e-6 then s, cx, cz = GA.straight(cx, cz, p.hs, p.ls, s, out) ; ch = p.hs end
    s = GA.arc(cx, cz, ch, p.s2, p.d2, R, s, out)
    return s
end

-- Build the whole circuit and install it as the flight plan. The shape is exactly the
-- procedure: fly straight out from the miss, join the published leg at its start on its
-- heading, run its FULL length, and only then turn back onto the approach line.
--   1. a straight run at the missed-approach heading (RWY_GA_STRAIGHT_M) -- clears the runway,
--      settles the speed, and opens up the geometry so a join exists at all;
--   2. a curvature-limited join to the leg's START, arriving ON the leg heading;
--   3. the leg itself, start to end, at its own heading and its own length;
--   4. a second join from the leg's END to the approach line, arriving on the RUNWAY heading
--      at a point far enough out to fly the circuit altitude off (see dApp) -- this is the
--      "only after passing the goaround-end" turn;
--   5. the run in from there to the threshold, where the approach re-arms as usual.
-- Returns false if no join exists (circles overlapping at this radius); the caller then holds.
function GA.build(ga, site, thrX, thrZ)
    local R = math.max(LG.RWY_GA_SPEED ^ 2 / (DC.GRAVITY * math.tan(math.rad(DC.MAX_BANK_DEG))),
        DC.MIN_RADIUS)
    local legH = atan2(ga.x2 - ga.x1, ga.z2 - ga.z1)
    local legL = math.sqrt((ga.x2 - ga.x1) ^ 2 + (ga.z2 - ga.z1) ^ 2)
    if legL < 1e-3 then return false end
    local v    = DIR_VECTORS[site.dir]
    local rwyH = headingForDir(site.dir)

    -- Where the approach is rejoined. Rejoin AT the leg's end if the leg finishes on the
    -- approach side of the threshold -- that is the tight circuit the published geometry is
    -- obviously drawn for -- and otherwise (a leg that finishes beyond the runway, i.e. an
    -- escape leg) at the shortest final worth flying. Either way the height to arrive at is
    -- then whatever the glidepath is at that distance, and the circuit DESCENDS to it: holding
    -- the circuit altitude to the rejoin is what left the first versionpermanently high.
    local dEnd = (thrX - ga.x2) * v[1] + (thrZ - ga.z2) * v[2]   -- +ve = before the threshold
    local dApp = math.max(LG.RWY_FINAL_MIN, dEnd)
    local appX, appZ = thrX - v[1] * dApp, thrZ - v[2] * dApp
    LND.gaExitAlt = site.alt + dApp * math.tan(math.rad(LG.RWY_GS_DEG))

    local out, wpS = {}, {}
    table.insert(out, {x = NAV.posX, z = NAV.posZ, hdg = NAV.heading, kappa = 0, s = 0,
                       speed = LG.RWY_GA_SPEED})
    -- RWY_GA_STRAIGHT_M is the MINIMUM run-out (clear the runway, settle the speed); the
    -- search above picks the actual one.
    local s = GA.join(NAV.posX, NAV.posZ, NAV.heading, ga.x1, ga.z1, legH, R, 0, out,
        LG.RWY_GA_STRAIGHT_M, LG.RWY_GA_STRAIGHT_M + LG.RWY_GA_RUNOUT_M)
    if not s then return false end
    wpS[1] = s                                            -- the leg's start
    s = GA.straight(ga.x1, ga.z1, legH, legL, s, out)
    wpS[2] = s                                            -- ...and its end
    -- The turn back onto the approach happens only AFTER the leg's end, so its run-out starts
    -- at zero -- but it may run on past the end if that makes the reversal shorter.
    s = GA.join(ga.x2, ga.z2, legH, appX, appZ, rwyH, R, s, out, 0, LG.RWY_GA_RUNOUT_M)
    if not s then return false end
    wpS[3] = s                                            -- back on the approach line
    s = GA.straight(appX, appZ, rwyH, dApp, s, out)
    wpS[4] = s                                            -- the threshold

    PLAN.samples, PLAN.wpS, PLAN.totalLen = out, wpS, s
    PLAN.cursor, PLAN.lastX, PLAN.lastZ, PLAN.lastS, PLAN.lastD2 = 1, nil, nil, 0, math.huge
    NAV.points = {
        {name = "GAENT", x = ga.x1, z = ga.z1, kind = "poi"},
        {name = "GAEND", x = ga.x2, z = ga.z2, kind = "poi"},
        {name = "FINAL", x = appX,  z = appZ,  kind = "poi"},
        {name = site.name, x = thrX, z = thrZ, kind = "poi"},
    }
    return true
end

-- ...and the WAYPOINT form of the same thing, which needs almost none of the above. Everything
-- the Dubins construction is for -- entering an oriented leg at its start, on its heading, from
-- wherever the miss left us -- exists because that record publishes a LINE and leaves the way
-- onto it to the aeroplane. A `goaround-wp` record publishes the whole pattern instead, fix by
-- fix, so there is no join to solve: it is a route, and the en-route planner already flies
-- routes -- with the same jerk-limited corner rounding, the same baked speed profile, and
-- LND.rwyPrep's straight-in final at the end of it. So this builds an ordinary plan and gets
-- out of the way.
--
-- The published list starts and ends at the runway it serves. The leading entry is dropped: it
-- is the threshold we have just missed and are sitting over, and flying TO it would turn the
-- circuit inside out. The trailing one is replaced by (thrX, thrZ) -- the landing fix the
-- approach was already flying to, snapshotted at engagement -- so the circuit ends at exactly
-- the point the retry starts from, whatever points.db says today.
--
-- Returns false when nothing in the middle of the list resolves against points.db: there is no
-- circuit there to fly, and the caller falls back rather than flying the empty shape.
function GA.buildWp(ga, site, thrX, thrZ)
    local seq = ga.seq
    local first, last = 1, #seq
    if seq[first] == site.name then first = first + 1 end
    if seq[last]  == site.name then last  = last  - 1 end
    local pts = {}
    for i = first, last do
        local p = POINTS[seq[i]]
        -- An unknown name is skipped, not fatal -- exactly what setRoute does with a course
        -- naming a point that has since been deleted.
        if p then table.insert(pts, {name = p.name, x = p.x, z = p.z, kind = p.kind}) end
    end
    if #pts == 0 then return false end
    table.insert(pts, {name = site.name, x = thrX, z = thrZ, kind = "poi"})
    LND.rwyPrep(pts)          -- MUTATES pts: must come before PLAN.build, as everywhere else
    NAV.points = pts
    PLAN.build(NAV.points)
    return true
end

-- Heading error (rad) from the nose to the CURRENT target, or nil if we lack a fix/heading.
local function targetHeadingErr()
    if not (NAV.active and NAV.haveHeading) then return nil end
    -- Error to the point we ACTUALLY steer toward -- the carrot on the curve (stored by
    -- broadcastCurrent) -- NOT the raw waypoint. Otherwise the align gate never clears while the
    -- craft correctly chases a carrot whose bearing differs from the waypoint's: alignment sticks.
    local ax, az = NAV.aimX, NAV.aimZ
    if not ax then
        local tgt = NAV.points[NAV.index]
        if not tgt then return nil end
        ax, az = tgt.x, tgt.z
    end
    return wrapPi(atan2(ax - NAV.posX, az - NAV.posZ) - NAV.heading)
end

-- Cross-track offset (m) of the active leg (prev -> current target); + = right of course.
-- 0 when there is no previous fix to form a leg (a direct-to has no line to hold).
local function currentXtk()
    local tgt, prev = NAV.points[NAV.index], NAV.points[NAV.index - 1]
    if not (tgt and prev) then return 0 end
    local lx, lz = tgt.x - prev.x, tgt.z - prev.z
    local ll = math.sqrt(lx * lx + lz * lz)
    if ll <= 1e-3 then return 0 end
    return (lx / ll) * (NAV.posZ - prev.z) - (lz / ll) * (NAV.posX - prev.x)
end

-- THE commanded along-track speed = the baked-in speed profile of the planned clothoid path
-- (PLAN.speedAt: slow through tight bends, cruise on straights, already bled off IN ADVANCE by
-- the offline backward/forward braking pass), plus the destination arrival brake on the final
-- leg, trimmed by the cross-track gate. Because it reads the SAME planned path the bearing
-- follows, the speed and the path are one trajectory -- the speed slows exactly as much as the
-- rounded corner needs, not for the raw waypoint angle. Zero while aligning; bounded by the
-- pilot's cap. (MAX_BANK_DEG/GRAVITY = how fast through a bend, baked in at plan time;
-- BRAKE_DECEL = how early the runtime destination brake starts.)
local function dynamicSpeedTarget()
    local capBase = cruiseCapValue()
    -- A go-around circuit is flown at approach speed, not at cruise. Both shapes, for their own
    -- reasons: the straight-leg one is BUILT at a turn radius computed from RWY_GA_SPEED, so
    -- flying it faster would not fit the path at all; and either one ends by descending onto
    -- the glidepath and handing straight back to the approach, which the airframe's descent
    -- authority caps at about this speed anyway (RWY_VS_MAX * RWY_GS_MARGIN / tan(GS)). A
    -- published waypoint circuit is kilometres long and this makes it a slow few minutes --
    -- which is what a go-around is.
    if LND.ga then capBase = math.min(capBase, LG.RWY_GA_SPEED) end
    if not DC.DYNAMIC_CRUISE or capBase <= 0 then return capBase end
    local total = #NAV.points
    if total == 0 then return capBase end
    if NAV.aligning then return 0 end          -- route start: yaw onto the bearing before throttling

    -- Baked speed profile (bends + the endpoint) -- but only while we are ON the path to read
    -- it from; joining one from elsewhere is flown at the plain cruise cap.
    local best = capBase
    if PLAN.onPath() then best = PLAN.speedAt() end

    -- Destination arrival on the FINAL leg (also covers a single-point direct-to, where the
    -- plan has nothing to walk): brake straight-line to FINAL_ARRIVE_SPEED at the last fix,
    -- at DC.ARRIVE_DECEL (steeper than BRAKE_DECEL -- see its comment) so this live distance
    -- check can't be out-run any more than the baked profile's own arrive-brake sweep can.
    if NAV.index == total then
        local last = NAV.points[total]
        local dx, dz = last.x - NAV.posX, last.z - NAV.posZ
        local dd = math.sqrt(dx * dx + dz * dz)
        best = math.min(best, math.sqrt(DC.FINAL_ARRIVE_SPEED * DC.FINAL_ARRIVE_SPEED + 2 * DC.ARRIVE_DECEL * dd))
    end

    -- Cross-track: well off COURSE -> slow to fly IN and intercept. When curve-following, "course"
    -- is the PLANNED PATH, not the straight leg: a craft correctly on a rounded corner is off the
    -- LEG by design, which would FALSELY trip this. Measure deviation from the path there.
    local xtk
    if DC.CURVE and NAV.phase == "ENROUTE" and NAV.index >= 2 and NAV.index ~= total
            and PLAN.onPath() then
        local _, cxtk = PLAN.course()
        xtk = math.abs(cxtk or 0)
    else
        xtk = math.abs(currentXtk())
    end
    if xtk > DC.XTK_SLOW_LO then
        best = best * clamp(1 - (xtk - DC.XTK_SLOW_LO) / (DC.XTK_SLOW_HI - DC.XTK_SLOW_LO), DC.XTK_SLOW_MIN, 1)
    end
    return clamp(best, 0, capBase)
end

-- Pass radius for the ADVANCE decision, turn-aware (see ADVANCE_*_MULT). Gentle turn -> larger
-- (cut the corner early / fly-by); sharp -> tighter. The destination stays at the base radius.
-- This is the FALLBACK test (used when DC.CURVE is off); while curve-following, PLAN.passed
-- (the baked transition-midpoint arc length) is the primary corner-cut signal -- see navTick.
local function advanceRadiusFor(i, tgt, isFinal)
    local base = radiusFor(tgt, isFinal)
    if isFinal then return base end
    local t = clamp((turnAngleAt(i) - DC.TURN_GENTLE) / (DC.TURN_SHARP - DC.TURN_GENTLE), 0, 1)
    return base * (DC.ADVANCE_GENTLE_MULT + (DC.ADVANCE_SHARP_MULT - DC.ADVANCE_GENTLE_MULT) * t)
end

local function broadcastCurrent()
    if LND.active then
        -- Once touched down, NAVCOM has already been cancelled and should stay coasting
        -- -- do not re-broadcast a LANDING target, or we would re-engage its position
        -- hold on the ground.
        if LND.phase == "LANDED" then return end
        if LND.rwy then
            -- RUNWAY: a moving point on the centreline (or, once rolling, the far end of the
            -- runway) rather than a pad to hover over. schedSpeed hands the UAC the approach
            -- speed the glideslope and the runway length allowed, exactly as en route, so the
            -- HSI's CMD stays the one true commanded speed; carrot makes it pure-pursue that
            -- point; glide tells it this is still a FLYING leg, so HYBRID/FIXEDWING may bank
            -- to hold the centreline even though the vertical channel is descending. On the
            -- rollout it becomes an ordinary LANDING position hold on the stop point, whose
            -- decelerate-to-zero law is the brake.
            rednet.broadcast({
                x = LND.tgtX, y = LND.tgtZ, type = "rwy", name = LND.name,
                phase = LND.navPhase, holdHeading = LND.navHold,
                cruiseCap = LND.cmdSpeed, schedSpeed = true,
                carrot = LND.glide or nil, glide = LND.glide or nil,
            }, APNAV_PROTOCOL)
            return
        end
        -- LANDING: hold the pad position and the commanded landing heading. Same target
        -- coords the plan's final point had (a rebroadcast to navcom, not a new leg), but
        -- with phase LANDING + a heading to hold.
        rednet.broadcast({
            x = LND.x, y = LND.z, type = "land", name = LND.name,
            phase = "LANDING", holdHeading = LND.holdHeading,
        }, APNAV_PROTOCOL)
        return
    end
    local tgt = NAV.points[NAV.index]
    if not tgt then return end
    local isFinal = (NAV.index == #NAV.points)
    -- En-route we hand the UAC a carrot on the planned clothoid path (smooth bearing, jerk-
    -- controlled rounded corners). The precise capture phases (APPROACH/TERMINAL/ARRIVED) and
    -- the final fix keep aiming at the real waypoint, unchanged -- a different, precise job.
    local tx, tz, ptype, carrot
    if DC.CURVE and NAV.phase == "ENROUTE" and not isFinal and PLAN.onPath() then
        tx, tz = PLAN.carrot(DC.CURVE_LOOKAHEAD)
        ptype, carrot = "carrot", true
    else
        tx, tz = tgt.x, tgt.z
        ptype = isFinal and "end" or tgt.kind
    end
    NAV.aimX, NAV.aimZ = tx, tz    -- the point we're actually steering to; read by the align gate + HSI
    rednet.broadcast({
        x = tx, y = tz, type = ptype, name = tgt.name,
        phase = NAV.phase or "ENROUTE",
        cruiseCap = NAV.cmdSpeed,  -- the rise-rate-limited commanded along-track speed (set in navTick)
        schedSpeed = DC.DYNAMIC_CRUISE,       -- tell the UAC this cap is a speed to TRACK, not just a
                                           -- ceiling, so it drops its own decelerate-at-every-fix law
        carrot = carrot,           -- UAC: pure-pursue this moving point (velocity-vector, no cross-track)
        -- On a go-around the vertical channel is in a commanded CLIMB, which otherwise makes
        -- the UAC treat this as a pad manoeuvre and fly wings-level -- leaving a FIXEDWING
        -- with no lateral control at all. Same flag, same reason, as the glideslope.
        glide = LND.ga or nil,
    }, APNAV_PROTOCOL)
end

-- Vertical command to the Stabiliser+Altitude control during a landing. Streamed every
-- tick (not just on the 1 s ping) so the flare is smooth as LND.vs eases with altitude.
local function broadcastVertical()
    -- A go-around keeps the vertical channel even though LND.active is false -- the route
    -- machinery is flying the circuit, but its altitude is ours to hold, not ALT-AP's.
    if LND.ga then
        rednet.broadcast({ cmd = "descend", vs = LND.vs }, AUTOLAND_PROTOCOL)
        return
    end
    if not LND.active then return end
    if LND.paused then
        -- Pilot override: hand the vertical channel back to ALT-AP. The UAC treats
        -- "release" like a cancel of the vertical (tracks altap again) while we keep
        -- streaming the LANDING navap (broadcastCurrent) so alignment stays online.
        rednet.broadcast({ cmd = "release" }, AUTOLAND_PROTOCOL)
        return
    end
    local p = LND.phase
    -- ROLLOUT is "down" as far as the vertical channel is concerned -- the wheels are on the
    -- runway and only the brakes are still working. LND.vertHold is the runway machine's way
    -- of saying "level off" (lining up, or a missed approach); the pad phases name themselves.
    if p == "LANDED" or p == "ROLLOUT" then
        -- `squat` tells the vertical channel WHICH REFERENCE it is down on. With the gear
        -- contact alive and made, that is ground truth: the UAC holds the craft down on it and
        -- must never fire the lift thrusters UP, which is what an altitude hold does every time
        -- the craft settles a few centimetres (and what keeps it skittish and stops the gear
        -- loading). With no gear contact to believe, the altimeter is all there is, so it falls
        -- back to holding the altitude it touched down at. Latched through the rollout on
        -- LND.tdBy so one bounce of the contact cannot flip it back to an altitude hold.
        rednet.broadcast({ cmd = "landed",
            squat = TD.fresh() and (TD.signal or LND.tdBy == "squat") or nil,
        }, AUTOLAND_PROTOCOL)
    elseif LND.vertHold or p == "ALIGN" or p == "TRANSLATE" then
        -- Hold altitude and take ALT-AP offline while we line up.
        rednet.broadcast({ cmd = "hold" }, AUTOLAND_PROTOCOL)
    elseif p == "DESCEND" or p == "FLARE" or p == "GLIDE" or p == "INTERCEPT" then
        -- INTERCEPT belongs here too: a runway approach descends toward the glidepath while it
        -- lines up, and only levels off (via vertHold, above) at the approach minimum.
        rednet.broadcast({ cmd = "descend", vs = LND.vs }, AUTOLAND_PROTOCOL)
    end
end

-- Descent-rate schedule: quick well above the pad, easing linearly to LG.DESCEND_SLOW as
-- the flare height is reached, and to zero at touchdown height. Returns a rate <= 0.
local function descentRateFor(agl)
    if agl <= LG.TOUCHDOWN_AGL then return 0 end
    if agl >= LG.FLARE_AGL then return -LG.DESCEND_FAST end
    local f = (agl - LG.TOUCHDOWN_AGL) / (LG.FLARE_AGL - LG.TOUCHDOWN_AGL)   -- 0 at pad .. 1 at flare top
    return -(LG.DESCEND_SLOW + (LG.DESCEND_FAST - LG.DESCEND_SLOW) * f)
end

-- The three touchdown tests, factored out so both landing machines ask the same question:
--   1. the squat switch -- the gear is loaded, so we are down at whatever height landing.db
--      thinks the surface is. Believed at any AGL (TD.confirmed already debounces it and
--      gates it on freshness), which is the whole point of having the sensor.
--   2. AGL against the site altitude -- the original test, and the one that still works with
--      the sensor offline.
--   3. "commanding descent but no longer sinking" near the surface -- the backup for a site
--      altitude that is a metre or two off and a sensor that is not there.
-- Returns which test tripped ("squat"/"AGL"/"settle"), or nil. Owns LND.settle.
function LND.touched(agl)
    if TD.confirmed() then return "squat" end
    if agl <= LG.TOUCHDOWN_AGL then return "AGL" end
    if LND.vs < 0 and agl <= LG.TD_SETTLE_AGL and math.abs(NAV.velY) < LG.TD_SETTLE_VEL then
        LND.settle = LND.settle + 1
        if LND.settle >= LG.TD_SETTLE_TICKS then return "settle" end
    elseif LND.vs < 0 and math.abs(NAV.velY) >= LG.TD_SETTLE_VEL then
        LND.settle = 0
    end
    return nil
end

-- RUNWAY APPROACH ------------------------------------------------------------
-- See the LG.RWY_* block for what this is and why it lives entirely in the FMS.

-- Can this site actually be flown as a runway? It needs a length to touch down on and a
-- direction to land in; 'x' (any direction) is meaningless for a strip. Anything that fails
-- here quietly falls back to the pad descent, which works for any site.
function LND.rwyUsable(site)
    return site ~= nil and site.type == "rwy" and (site.rwyLen or 0) > 0
        and DIR_VECTORS[site.dir] ~= nil
end

-- PLAN TIME: give a runway destination a straight-in final leg. The plan's last point is the
-- THRESHOLD; this inserts a FINAL fix RWY_FINAL_M back down the extended centreline just
-- before it, so the en-route planner flies its own jerk-limited turn onto the runway heading
-- and hands the approach over already lined up -- the landing machine's INTERCEPT is then a
-- safety net rather than the normal way in. When the point before the runway is nearer than
-- the full leg wants, the leg is SHORTENED to fit rather than abandoned (a short aligned
-- final still beats none), and only dropped when even RWY_FINAL_MIN will not fit.
-- MUTATES pts, which is why every caller runs it after setLandFinal but BEFORE PLAN.build
-- and before NAV.index is chosen -- the planner and the waypoint index must both see it.
function LND.rwyPrep(pts)
    if not (LG.RWY_FINAL_FIX and NAV.landAtEnd and LND.rwyUsable(NAV.landSite)) then return end
    local n   = #pts
    local thr = pts[n]
    local d   = LG.RWY_FINAL_M
    local px, pz
    if n >= 2 then px, pz = pts[n - 1].x, pts[n - 1].z
    elseif NAV.haveFix then px, pz = NAV.posX, NAV.posZ end
    if px then
        local dx, dz = thr.x - px, thr.z - pz
        -- Keep the final leg well under the run in from the point before it, so the planner
        -- has room to turn onto it (the same reason DC.LEG_FIT_FRACTION exists).
        d = math.min(d, math.sqrt(dx * dx + dz * dz) / 1.2)
    end
    if d < LG.RWY_FINAL_MIN then return end
    local v = DIR_VECTORS[NAV.landSite.dir]
    table.insert(pts, n, { name = "FINAL", kind = "poi",
        x = thr.x - v[1] * d, z = thr.z - v[2] * d })
end

-- Everything the approach needs from the current fix, in RUNWAY coordinates: how far there
-- is still to run to the aim point along the centreline, how far off that centreline we are,
-- and how far above/below the nominal glidepath. The steering point is a carrot on the
-- centreline RWY_CARROT_M ahead of our own projection onto it -- pure pursuit, exactly as en
-- route, so joining the centreline and staying on it are one law needing no separate mode.
-- Deliberately NOT clamped at the runway end: through the flare the carrot must stay ahead of
-- the aircraft, or the nose would swing round to chase a point already behind it.
function LND.rwyGeom()
    local ux, uz = LND.ux, LND.uz
    local dx, dz = NAV.posX - LND.x, NAV.posZ - LND.z          -- from the THRESHOLD
    LND.s     = dx * ux + dz * uz                               -- + = past the threshold
    LND.xtk   = ux * dz - uz * dx                               -- + = right of the centreline
    LND.xtkRate = ux * NAV.velZ - uz * NAV.velX                 -- ...and how fast that is changing
    LND.along = (LND.aimX - NAV.posX) * ux + (LND.aimZ - NAV.posZ) * uz
    LND.gsDev = (NAV.posY - LND.siteAlt) - math.max(LND.along, 0) * LND.gsTan

    -- THE LOCALIZER. The steering point is deliberately NOT just "a point on the centreline
    -- RWY_CARROT_M ahead": pure-pursuing that commands an intercept angle of only
    -- atan(xtk / lookahead), which at a 400 m lookahead is 3.6 deg for a 25 m offset. That is
    -- far too soft to actually centre the aircraft -- it settles onto a track PARALLEL to the
    -- centreline and stays there, which is exactly what a VTOL makes obvious (it crabs
    -- sideways instead of turning, so the offset is plain to see).
    -- Instead: command a lateral closure RATE proportional to the cross-track error, damped by
    -- the rate at which it is already closing, capped at a maximum intercept angle. That is a
    -- real localizer, with a gain that means something. It is expressed as a carrot placed in
    -- the commanded DIRECTION, because the direction to the target is precisely what the UAC's
    -- velocity-vector law flies -- so this drives all three flight modes without either of us
    -- knowing which one is active.
    local lat = -(LG.RWY_LOC_KP * LND.xtk + LG.RWY_LOC_KD * LND.xtkRate)
    local cap = LND.appSpeed * math.sin(math.rad(LG.RWY_LOC_MAX_DEG))
    lat = clamp(lat, -cap, cap)
    local fwd = math.sqrt(math.max(LND.appSpeed * LND.appSpeed - lat * lat, 1))
    -- "right of the runway" is (-uz, ux) -- moving that way raises xtk by 1 m per metre, which
    -- is the sign convention xtk itself is written in, so a positive `lat` means "go right".
    local vx, vz = ux * fwd - uz * lat, uz * fwd + ux * lat
    local n = math.sqrt(vx * vx + vz * vz)
    LND.tgtX = NAV.posX + vx / n * LG.RWY_CARROT_M
    LND.tgtZ = NAV.posZ + vz / n * LG.RWY_CARROT_M
end

-- GO-AROUND -------------------------------------------------------------------
-- Which go-around route serves this runway? A field may publish either shape, and the WAYPOINT
-- one wins wherever both exist: it names its runway outright and draws the whole pattern, so
-- flying it is carrying out a published procedure. The straight-leg form names nothing and
-- draws one line -- which runway it serves is inferred, and most of the circuit is invented by
-- the FMS around it. A drawn procedure beats a deduced one every time, so it is only reached
-- when there is no waypoint circuit for this runway at all.
function LND.gaFor(site, thrX, thrZ)
    if not LG.RWY_GA_ENABLE then return nil end
    return LND.gaWpFor(site) or LND.gaPathFor(site, thrX, thrZ)
end

-- The waypoint form, matched BY NAME: its list starts and ends at the runway it belongs to, so
-- there is nothing to infer. A circuit has to END at the runway (otherwise it is a departure,
-- not a go-around); one written the other way round -- only its first entry naming the runway
-- -- is flown backwards rather than refused, which is also what makes the two ends of a single
-- strip pick out their own circuit and not each other's.
function LND.gaWpFor(site)
    if not (site and site.name) then return nil end
    for _, e in pairs(LANDING) do
        local seq = (e.type == "goaround-wp") and e.seq or nil
        if seq and #seq >= 2 then
            if seq[#seq] == site.name then return e end
            if seq[1] == site.name then
                local rev = {}
                for i = #seq, 1, -1 do table.insert(rev, seq[i]) end
                return {type = e.type, alt = e.alt, seq = rev}
            end
        end
    end
    return nil
end

-- The straight-leg form, matched GEOMETRICALLY: the id in landing.db is explicitly arbitrary,
-- so the route whose leg runs the same way as the landing direction is the one that CONTINUES
-- a missed approach rather than opposing it. Ties (a field with several parallel routes) go to
-- whichever starts nearest the threshold.
function LND.gaPathFor(site, thrX, thrZ)
    local v = DIR_VECTORS[site and site.dir]
    if not v then return nil end
    local best, bestD
    for _, e in pairs(LANDING) do
        if e.type == "goaround" then
            local lx, lz = e.x2 - e.x1, e.z2 - e.z1
            local ll = math.sqrt(lx * lx + lz * lz)
            if ll > 1e-6 and (lx / ll * v[1] + lz / ll * v[2]) >= LG.RWY_GA_ALIGN then
                local dx, dz = e.x1 - thrX, e.z1 - thrZ
                local d = dx * dx + dz * dz
                if not bestD or d < bestD then best, bestD = e, d end
            end
        end
    end
    return best
end

-- Fly the missed approach as an ordinary ROUTE. Everything the circuit needs already exists:
-- the path planner turns with jerk-limited clothoids, so "circle into the route" and "circle
-- back to the runway line" come free; LND.rwyPrep re-inserts the straight-in final leg if
-- there is room for one; and arriving at the last point re-arms the approach exactly as any
-- other plan does. So the go-around is not a new state machine, it is a new PLAN.
-- The one thing that does NOT go back to normal is the VERTICAL channel: handing it to ALT-AP
-- would climb to whatever the pilot last selected, which is not where the route lives, so the
-- FMS keeps it (LND.ga) and holds the route's own altitude until the approach re-arms.
function LND.goAround(ga)
    local name, thrX, thrZ = LND.name, LND.x, LND.z
    LND.gaCount = LND.gaCount + 1
    LND.ga, LND.gaAlt = true, ga.alt
    LND.gaWp = (ga.type == "goaround-wp")
    LND.active, LND.rwy, LND.paused, LND.phase = false, false, false, "OFF"
    -- NAV.landAtEnd / NAV.landSite are still the ones set when the pilot committed to this
    -- landing, so the circuit lands at the same runway without re-deciding anything.
    local built
    if LND.gaWp then
        built = GA.buildWp(ga, NAV.landSite, thrX, thrZ)
        if not built then
            -- Its waypoints are not in points.db, so the published circuit is not flyable on
            -- this aircraft's data. A straight-leg route for the same runway is still a far
            -- better answer than levelling off, so try that before giving up.
            local alt = LND.gaPathFor(NAV.landSite, thrX, thrZ)
            if alt then
                LND.gaWp, LND.gaAlt = false, alt.alt
                built = GA.build(alt, NAV.landSite, thrX, thrZ)
            end
        end
    else
        built = GA.build(ga, NAV.landSite, thrX, thrZ)
    end
    if not built then
        -- Nothing flyable came out of either shape -- an unresolvable waypoint list, or no
        -- curvature-limited join from here (the turn circles overlap). Level off and hold
        -- rather than fly something arbitrary; the pilot has ABORT.
        LND.ga = false
        LND.active, LND.rwy, LND.phase = true, true, "MISSED"
        LND.vs, LND.vertHold = 0, true
        LND.holdX, LND.holdZ = NAV.posX + LND.ux * LG.RWY_CARROT_M, NAV.posZ + LND.uz * LG.RWY_CARROT_M
        return
    end
    NAV.index    = 1
    NAV.active   = true
    NAV.aligning = false   -- already flying: do NOT hold the throttle at 0 to swing the nose
    NAV.fromStart = true   -- ...but DO fly the circuit from its start. Either shape ENDS at the
                           -- runway we are sitting over, so projecting ourselves onto it would
                           -- report the whole circuit as already flown (see PLAN.onPath)
    NAV.planName = "GO-AROUND " .. name
    NAV.phase    = "ENROUTE"
    updatePhase()
    broadcastCurrent()
    LND.gaTick()        -- set the CLIMB before announcing it: LND.vs still holds the approach's
    broadcastVertical() -- last sink rate, and broadcasting that would dig in for one more tick
end

-- How much of the circuit is still to fly: to the active waypoint, then along every leg after
-- it. Not PLAN.lastS, deliberately -- that only means anything while we are ON the path, and
-- the vertical channel needs an answer during the join at the start, which is exactly when we
-- are not (see PLAN.onPath / NAV.fromStart).
function LND.gaRemaining()
    local pts = NAV.points
    local n = #pts
    if n == 0 then return 0 end
    local i = math.min(math.max(NAV.index, 1), n)
    local dx, dz = pts[i].x - NAV.posX, pts[i].z - NAV.posZ
    local d = math.sqrt(dx * dx + dz * dz)
    for k = i, n - 1 do d = d + legLength(pts[k], pts[k + 1]) end
    return d
end

-- The vertical channel during a go-around: an altitude capture flown on the same commanded-rate
-- channel the approach uses. Runs from navTick while the route machinery flies the horizontal.
function LND.gaTick()
    LND.agl = NAV.posY - LND.siteAlt
    local want, ff = LND.gaAlt, 0
    if LND.gaWp then
        -- A WAYPOINT circuit has no single "leg end" to start down after: it is a whole
        -- pattern, and the place the descent belongs is wherever the glidepath -- extended back
        -- along however much of the route is still to fly -- comes down through the published
        -- minimum. So: hold the minimum until the glidepath sinks below it, then ride the
        -- glidepath in. That lands the aircraft ON the path at the threshold whatever the
        -- pattern's shape or length, and needs no geometry the aerodrome has to get right --
        -- the rule the straight-leg form imposes ("end the leg far enough out to fly the
        -- altitude off") is answered here by construction instead.
        -- The published figure is a MINIMUM, and nothing above puts us under it: the pattern is
        -- flown AT it, and the only thing that ever commands lower is the glidepath itself.
        -- Measured to the THRESHOLD, so the arrival is a shade high (the aim point is a little
        -- way past it) -- which is the side to be on, since the approach can catch a glidepath
        -- from above and cannot from below (RWY_GS_MARGIN).
        local gsTan = math.tan(math.rad(LG.RWY_GS_DEG))
        local path  = LND.siteAlt + LND.gaRemaining() * gsTan
        if path < want then
            -- FEED-FORWARD. Once the glidepath binds, `want` is a RAMP falling at
            -- groundSpeed*gsTan every second, and a plain proportional law tracks a ramp with a
            -- standing error of rate/KP -- nearly 30 m high, all the way down, at these
            -- numbers. Command the ramp's own rate and let the gain work on the deviation
            -- about it instead.
            want, ff = path, -NAV.groundSpeed * gsTan
        end
    else
        -- Circuit altitude round the pattern; once past the leg's end and turning back onto the
        -- approach, the height the glidepath will want at the rejoin. Descending on the way
        -- round is what a circuit is for -- arriving at the runway line still at pattern
        -- altitude just means going around again.
        if NAV.index >= 3 then want = LND.gaExitAlt end
    end
    LND.vs = clamp(ff + (want - NAV.posY) * LG.RWY_GA_ALT_KP,
        -LG.RWY_VS_MAX, LG.RWY_GA_CLIMB_VS)
end

-- Lay out the runway and size the approach, once, at engagement.
function LND.rwySetup(site, fp)
    local v    = DIR_VECTORS[site.dir]
    local aim  = math.min(site.rwyLen * LG.RWY_AIM_FRAC, LG.RWY_AIM_MAX)
    local stop = math.max(site.rwyLen - LG.RWY_STOP_MARGIN, aim + 1)
    LND.rwy    = true
    LND.phase  = "INTERCEPT"
    LND.rwyLen = site.rwyLen
    LND.ux, LND.uz     = v[1], v[2]
    LND.gsTan          = math.tan(math.rad(LG.RWY_GS_DEG))
    LND.aimX,  LND.aimZ  = fp.x + v[1] * aim,          fp.z + v[2] * aim
    LND.stopX, LND.stopZ = fp.x + v[1] * stop,         fp.z + v[2] * stop
    LND.endX,  LND.endZ  = fp.x + v[1] * site.rwyLen,  fp.z + v[2] * site.rwyLen
    LND.stopS = stop
    -- APPROACH SPEED. Three caps: what we asked for; what the glideslope can be flown at with
    -- the vertical channel's descent authority, keeping RWY_GS_MARGIN in hand to rejoin the
    -- path from above; and what the rollout can stop from in the runway that is genuinely left.
    --
    -- "Genuinely left" is the bit that was wrong: the FLARE FLOATS the aircraft a long way down
    -- the strip before the wheels arrive, and that float was not being budgeted. It is easily
    -- the biggest consumer of a short runway -- easing from the glideslope sink to RWY_TD_VS
    -- through RWY_FLARE_AGL takes 2*FLARE_AGL/(V*gsTan + TD_VS) seconds, which at 37 m/s is
    -- ~90 m of runway. Sizing the speed off (stop - aim) alone therefore promised a touchdown
    -- at the aim point and delivered one ~90 m later, so the same approach randomly ran out of
    -- room. Solved by iteration, since the float depends on the very speed it is limiting; the
    -- speed only ever decreases, so four passes converge.
    -- RWY_ROLLOUT_FRAC then reserves part of what remains as deliberate go-around slack, so the
    -- planned touchdown sits comfortably before the point of no return rather than on it.
    local v = math.min(LG.RWY_APP_SPEED, LG.RWY_VS_MAX * LG.RWY_GS_MARGIN / LND.gsTan)
    local float = 0
    for _ = 1, 4 do
        float = 2 * LG.RWY_FLARE_AGL * v / (v * LND.gsTan + LG.RWY_TD_VS)
        local usable = math.max((stop - aim - float) * LG.RWY_ROLLOUT_FRAC, 5)
        v = math.min(v, math.sqrt(2 * LG.RWY_BRAKE_DECEL * usable))
    end
    LND.appSpeed = v
    LND.tdS      = aim + float    -- where the wheels are actually expected to arrive
    LND.cmdSpeed = NAV.cmdSpeed   -- pick the speed up where the en-route profile left it
    LND.rwyGeom()                 -- so the broadcast that follows engagement is already right
end

-- THE RUNWAY APPROACH, one step per navTick (readouts and geometry are refreshed by
-- landingTick before this runs). It names a point, a speed and a vertical speed -- never an
-- attitude -- so the profile is identical whatever the UAC is flying as.
function LND.rwyTick()
    local agl = LND.agl
    local p   = LND.phase
    -- Default posture: a flying approach chasing the centreline carrot at the approach speed.
    LND.navPhase, LND.navHold, LND.glide, LND.vertHold = "APPROACH", nil, true, false
    local want = LND.appSpeed

    if p == "INTERCEPT" or p == "GLIDE" or p == "FLARE" then
        -- ESTABLISHED on the centreline? Captured inside RWY_CAPT_XTK, and not given up again
        -- until RWY_LOSE_XTK.
        -- (written out rather than as `cond and a or b`: with a FALSE capture test that idiom
        -- falls straight through to the wider keep test, i.e. it would capture at RWY_LOSE_XTK)
        local est
        if p == "INTERCEPT" then est = math.abs(LND.xtk) <= LG.RWY_CAPT_XTK
        else                     est = math.abs(LND.xtk) <= LG.RWY_LOSE_XTK end
        -- The glideslope: sink at the rate the geometry needs AT THE SPEED WE ARE ACTUALLY
        -- FLYING (so the path angle is right whatever the craft is doing), plus a proportional
        -- pull back onto the path. Arriving HIGH is the normal case -- we join at whatever
        -- altitude ALT-AP was holding -- so this saturates at RWY_VS_MAX and descends at the
        -- maximum until the path is caught. It can only catch it at all because RWY_GS_MARGIN
        -- kept the nominal sink rate below that limit; see the approach-speed cap.
        local vs = -(NAV.groundSpeed * LND.gsTan) - LG.RWY_GS_KP * LND.gsDev
        if est and agl <= LG.RWY_FLARE_AGL then
            -- Flare: blend the glideslope rate into a gentle constant touchdown sink.
            local f = math.max(agl, 0) / LG.RWY_FLARE_AGL
            vs = f * vs - (1 - f) * LG.RWY_TD_VS
        end
        -- BELOW the glidepath the correction is a CLIMB, and it has to be allowed or the
        -- approach can only ever level off and wait for the path to descend onto it -- which is
        -- not glidepath capture, it is just a vertical-speed command that happens to be zero.
        -- Never inside the flare, though: down there the only way is down.
        LND.vs = clamp(vs, -LG.RWY_VS_MAX,
            (agl > LG.RWY_FLARE_AGL) and LG.RWY_VS_UP_MAX or 0)

        if not est then
            -- Descending while still lining up is fine, and is what the length of the final
            -- leg is FOR -- but never below the approach minimum until the centreline is
            -- captured, or we would end up low and wide of the runway.
            LND.phase = "INTERCEPT"
            if agl <= LG.RWY_MIN_AGL then LND.vs, LND.vertHold = 0, true end
        else
            LND.phase = (agl <= LG.RWY_FLARE_AGL) and "FLARE" or "GLIDE"
        end

        -- TOUCHDOWN IS GROUND TRUTH, WHEREVER IT HAPPENS. Tested in every airborne phase, not
        -- only once established: the gear being loaded means the aircraft is DOWN, and ignoring
        -- that because it is a few metres wide of the centreline is how a landing ends up
        -- flying along at the approach minimum with its wheels already on the runway.
        local td = LND.touched(agl)
        if td then
            LND.tdBy, LND.phase, LND.settle = td, "ROLLOUT", 0
        elseif LND.s > 0 and
               LND.stopS - LND.s < NAV.groundSpeed * NAV.groundSpeed / (2 * LG.RWY_BRAKE_DECEL) then
            -- (only once past the THRESHOLD -- before it the whole strip is still ahead, and
            -- the aircraft may legitimately still be shedding cruise speed, which would make
            -- the stopping distance below read as thousands of metres.)
            -- MISSED APPROACH -- and the test is "there is no longer enough runway left to stop
            -- in from the speed we are doing", NOT "we drifted past the aiming point". The old
            -- test fired RWY_STOP_MARGIN past the aim point, which on a 295 m strip meant
            -- giving up with 200 m of perfectly good runway still ahead -- and since a normal
            -- flare floats most of the way to that line anyway, the same approach would land or
            -- go around more or less at random. This one uses the whole strip, and agrees with
            -- the sizing above by construction (both are RWY_BRAKE_DECEL).
            -- ...and if this runway has a go-around route in landing.db, FLY IT: climb away on
            -- the published circuit and come back for another attempt. RWY_GA_MAX bounds that,
            -- so a runway that cannot be landed on ends up parked in the air rather than
            -- circling until the fuel or the patience runs out.
            local ga = nil
            if LND.gaCount < LG.RWY_GA_MAX then ga = LND.gaFor(NAV.landSite, LND.x, LND.z) end
            if ga then LND.goAround(ga); return end
            -- Nothing published (or out of attempts): level off and stop, wings level on the
            -- runway heading. No blind climb-away -- without a route there is no altitude the
            -- FMS can be sure is clear -- so the pilot gets a stable aircraft and ABORT.
            LND.phase, LND.vs, LND.vertHold = "MISSED", 0, true
            LND.holdX = NAV.posX + LND.ux * LG.RWY_CARROT_M
            LND.holdZ = NAV.posZ + LND.uz * LG.RWY_CARROT_M
        end

    elseif p == "ROLLOUT" then
        -- On the runway: hand the UAC the stop point as an ordinary LANDING position hold --
        -- its decelerate-to-zero law IS the brake -- with the runway heading held so we track
        -- the centreline while slowing. RWY_BRAKE_DECEL, which sized the approach speed, is
        -- the assumption this is meant to satisfy.
        LND.navPhase, LND.navHold, LND.glide = "LANDING", LND.holdHeading, false
        LND.tgtX, LND.tgtZ = LND.stopX, LND.stopZ
        LND.vs, want = 0, 0
        if NAV.groundSpeed <= LG.RWY_STOP_SPEED then
            -- Stopped. Same teardown as a pad touchdown: NAVCOM coasts (control handed back),
            -- the vertical channel is told "landed" so it holds the ground and keeps ignoring
            -- ALT-AP, and the LANDED screen offers AP OFF / RESUME ALT-AP.
            LND.phase = "LANDED"
            rednet.broadcast({ cancel = true }, APNAV_PROTOCOL)
            NAV.active, NAV.landAtEnd = false, false
            broadcastVertical()
        end

    elseif p == "MISSED" then
        -- Levelled off past the runway. Hand the UAC the frozen point ahead as an ordinary
        -- LANDING position hold, so the aircraft decelerates and STOPS there on the runway
        -- heading instead of flying away down the extended centreline while nobody is
        -- watching. ABORT is the go-around; RESUME LND retries from here.
        LND.navPhase, LND.navHold, LND.glide = "LANDING", LND.holdHeading, false
        LND.tgtX, LND.tgtZ = LND.holdX, LND.holdZ
        LND.vs, LND.vertHold, want = 0, true, 0

    elseif p == "LANDED" then
        LND.vs, want = 0, 0
    end

    -- Same slew as the en-route commanded speed, and for the same reason: cruiseCap is a live
    -- command now, so it has to be a curve rather than a staircase.
    if want > LND.cmdSpeed then
        LND.cmdSpeed = math.min(want, LND.cmdSpeed + DC.ACCEL_MAX * TICK_INTERVAL)
    else
        LND.cmdSpeed = math.max(want, LND.cmdSpeed - DC.DECEL_MAX * TICK_INTERVAL)
    end
end

-- Arm the landing sequence. Called once, when a landAtEnd plan has genuinely arrived at
-- its final (landing) point. Everything it needs was snapshotted into NAV.landSite when
-- the plan was set, so a mid-flight landing.db edit cannot move the landing under us.
local function engageAutoland()
    local site = NAV.landSite
    local fp   = NAV.points[#NAV.points]
    LND.active      = true
    LND.phase       = "ALIGN"
    LND.paused      = false
    LND.rwy         = false
    LND.ga          = false   -- (LND.gaCount deliberately NOT reset -- it bounds the retries)
    LND.gaWp        = false
    LND.vertHold    = false
    LND.x, LND.z    = fp.x, fp.z
    LND.name        = site.name
    LND.holdHeading = headingForDir(site.dir)
    LND.siteAlt     = site.alt
    LND.siteType    = site.type
    LND.vs          = 0
    LND.settle      = 0
    LND.tdBy        = nil
    -- A usable runway is flown straight in on a glideslope instead of hovered onto; anything
    -- else (a helipad, or a runway entry missing a length or a direction) keeps the pad
    -- descent below. LND.rwySetup replaces the phase and lays the strip out.
    if LND.rwyUsable(site) then LND.rwySetup(site, fp) end
end

-- The landing state machine, one step per navTick. Reads the pose NAV published this
-- tick, decides the phase and the commanded vertical speed, and (at touchdown) stops
-- the navigation. NAVCOM is kept holding the pad position + landing heading throughout
-- by broadcastCurrent/broadcastVertical; this function only decides WHEN to descend and
-- WHEN we are down.
local function landingTick()
    local dx, dz  = LND.x - NAV.posX, LND.z - NAV.posZ
    local posErr  = math.sqrt(dx * dx + dz * dz)
    local hdgErr  = LND.holdHeading and wrapPi(LND.holdHeading - NAV.heading) or 0
    local agl     = NAV.posY - LND.siteAlt

    LND.posErr    = posErr
    LND.hdgErrDeg = math.deg(hdgErr)
    LND.agl       = agl

    -- A runway approach keeps its errors in runway coordinates instead: how far off the
    -- centreline, not how far from the point (which we are deliberately flying past).
    if LND.rwy then
        LND.rwyGeom()
        LND.posErr = math.abs(LND.xtk)
        if LND.paused then return end
        LND.rwyTick()
        return
    end

    -- Paused by a pilot override: ALT-AP owns the vertical, position/heading alignment
    -- keeps running via broadcastCurrent. Freeze the descent/touchdown state machine (the
    -- craft may be climbing away on ALT-AP); RESUME LANDING clears LND.paused and it picks
    -- up from the same phase.
    if LND.paused then return end

    local alignedCoarse = (not LND.holdHeading) or (math.abs(hdgErr) <= math.rad(LG.ALIGN_HDG_DEG))
    local alignedFine   = (posErr <= LG.POS_TOL_M)
        and ((not LND.holdHeading) or (math.abs(hdgErr) <= math.rad(LG.FINE_HDG_DEG)))
    local drifted       = (posErr > LG.RECAPTURE_M)
        or (LND.holdHeading and math.abs(hdgErr) > math.rad(LG.RECAPTURE_HDG_DEG))

    if LND.phase == "ALIGN" then
        -- Step 1: NAVCOM holds the pad and swings to the landing heading; hold altitude.
        LND.vs = 0
        if alignedCoarse then LND.phase = "TRANSLATE" end

    elseif LND.phase == "TRANSLATE" then
        -- Step 2: close the last few metres precisely while keeping heading. Hold altitude
        -- until BOTH position and heading are inside tolerance.
        LND.vs = 0
        if alignedFine then LND.phase = "DESCEND" end

    elseif LND.phase == "DESCEND" or LND.phase == "FLARE" then
        -- Steps 3-4: descend while holding position/heading. If the descent nod has shoved
        -- us off the pad, stop sinking and let NAVCOM re-capture before continuing.
        if drifted then
            LND.vs = 0
            LND.settle = 0
        else
            LND.vs = descentRateFor(agl)
        end
        LND.phase = (agl <= LG.FLARE_AGL) and "FLARE" or "DESCEND"

        -- Touchdown, from whichever of the three tests trips first (see LND.touched).
        local tdBy = LND.touched(agl)
        if tdBy then
            LND.tdBy = tdBy
            -- Step 5: stop the navigation. NAVCOM coasts (hands control back); the vertical
            -- channel is told "landed" so it holds the ground and keeps ignoring ALT-AP.
            LND.phase  = "LANDED"
            LND.vs     = 0
            rednet.broadcast({ cancel = true }, APNAV_PROTOCOL)
            NAV.active    = false
            NAV.landAtEnd = false
            broadcastVertical()   -- immediate "landed" so control.lua latches at once
        end

    elseif LND.phase == "LANDED" then
        LND.vs = 0
    end
end

-- Snapshot the landing site for the plan's final point, if it is one and we mean to
-- land there. Taken once, at plan-set time, so editing landing.db mid-flight cannot
-- retarget a landing already under way -- the same reason NAV.points is a snapshot.
local function setLandFinal(finalName, wantLand)
    if wantLand and isLandingSite(finalName) then
        local li = LANDING[finalName]
        NAV.landAtEnd = true
        NAV.landSite  = { name = finalName, dir = li.dir, alt = li.alt, type = li.type,
                          rwyLen = li.rwyLen }
    else
        NAV.landAtEnd = false
        NAV.landSite  = nil
    end
end

-- landAtEnd: engage the landing sequence automatically once we arrive (used by the LAND
-- menu, and by a route whose final point is a landing site). A plain DIRECT-to leaves it
-- false -- go there and hold, do not land, even if the point happens to be a pad.
local function setDirectTo(name, landAtEnd)
    local p = POINTS[name]
    if not p then return false end
    LND.active = false
    LND.phase  = "OFF"
    LND.rwy    = false
    LND.ga     = false
    LND.gaCount = 0   -- a plan the pilot just committed to gets its full allowance of attempts
    NAV.cruiseMode = "full"   -- a fresh plan starts at full cruise, never a stale COAST
    NAV.aligning   = true     -- ...and points the nose at the first target before throttling up
    NAV.fromStart  = false    -- ...and joins the plan wherever it already is
    NAV.planName = landAtEnd and "LAND" or "DIRECT"
    local pts = {{name = p.name, x = p.x, z = p.z, kind = p.kind}}
    -- The landing site is snapshotted BEFORE the plan is built, because a runway destination
    -- adds a straight-in final leg to the point list that the path planner and the waypoint
    -- index must both be built over (LND.rwyPrep).
    setLandFinal(name, landAtEnd)
    LND.rwyPrep(pts)
    NAV.points = pts
    NAV.index = 1
    NAV.active = true
    PLAN.build(NAV.points)
    updatePhase() -- against the last known fix, so the first broadcast isn't stale
    broadcastCurrent()
    return true
end

-- Which waypoint of a route to start from when resuming mid-flight, given the aircraft's
-- current fix and heading. Normally the nearest waypoint, but one BEHIND us is penalised: a
-- U-turn costs distance we would rather not fly, so its straight-line range is inflated by
-- RESUME_BACKTRACK_WEIGHT times its backward component before the cheapest is chosen. Result:
-- when the nearest point is behind but the next is ahead, we press on to the one ahead --
-- unless the one behind is close enough that turning back is genuinely cheaper. Evaluated over
-- ALL waypoints (not just the two nearest), so a third point ahead can win too. Falls back to
-- plain nearest with no heading fix, and to index 1 with no position fix at all.
local function bestResumeIndex(pts)
    if #pts == 0 then return 1 end
    if not NAV.haveFix then return 1 end
    -- Forward unit vector in the same (x,z) frame as bearing = atan2(dx,dz): heading th maps
    -- to (sin th, cos th). nil when we have no heading -> no backtrack penalty (plain nearest).
    local fwdX, fwdZ
    if NAV.haveHeading then
        fwdX, fwdZ = math.sin(NAV.heading), math.cos(NAV.heading)
    end
    local bestI, bestCost = 1, math.huge
    for i, p in ipairs(pts) do
        local dx, dz = p.x - NAV.posX, p.z - NAV.posZ
        local d = math.sqrt(dx * dx + dz * dz)
        local penalty = 0
        if fwdX and d > 1e-6 then
            local align = (fwdX * dx + fwdZ * dz) / d   -- cos(angle to wp): >0 ahead, <0 behind
            if align < 0 then penalty = RESUME_BACKTRACK_WEIGHT * (-align) end
        end
        local cost = d * (1 + penalty)
        if cost < bestCost then
            bestCost, bestI = cost, i
        end
    end
    return bestI
end

-- resume: join the plan at bestResumeIndex(pts) instead of the first point (mid-route
-- resume). Everything downstream -- phase, pass radii, the PASSED/remaining status list --
-- keys off NAV.index, so a resumed start at index i is identical to having advanced there.
local function setRoute(name, resume)
    local c = COURSES[name]
    if not c then return false end
    local pts = {}
    for _, pname in ipairs(c.seq) do
        local p = POINTS[pname]
        if p then
            table.insert(pts, {name = p.name, x = p.x, z = p.z, kind = p.kind})
        end
    end
    if #pts == 0 then return false end
    LND.active = false
    LND.phase  = "OFF"
    LND.rwy    = false
    LND.ga     = false
    LND.gaCount = 0   -- a plan the pilot just committed to gets its full allowance of attempts
    NAV.cruiseMode = "full"   -- a fresh plan starts at full cruise, never a stale COAST
    NAV.aligning   = true     -- ...and points the nose at the first target before throttling up
    NAV.fromStart  = false    -- ...and joins the plan wherever it already is (see RESUME)
    NAV.planName = name
    -- A route ending at a landing site auto-lands there on arrival (fully automatic). Both
    -- calls come before PLAN.build / NAV.index: a runway destination gains a final leg here.
    setLandFinal(pts[#pts].name, true)
    LND.rwyPrep(pts)
    NAV.points = pts
    NAV.index = resume and bestResumeIndex(pts) or 1
    NAV.active = true
    PLAN.build(NAV.points)
    updatePhase()
    broadcastCurrent()
    return true
end

-- AUTO-ROUTER (destination-driven multi-course planner) -------------------------
-- "FLY ROUTE > TO DESTINATION": rather than the pilot naming one course, they name a
-- destination hub and the FMS solves a flight plan to it from the aircraft's current
-- position, stitching published courses together through the waypoints they share. There is
-- no direct KST-TMU course, but KST-WAU and WAU-TMU exist and meet at WAU, so KST->TMU solves
-- as KST..WAU..TMU. The result is one ordered point list, flown (and joined mid-route) exactly
-- like any other plan -- so everything downstream (phase, pass radii, resume, auto-land) is
-- reused unchanged; only the point list is chosen by search instead of by name.

-- Everything the auto-router owns lives on this one table rather than a dozen separate
-- top-level locals -- the main chunk is close to Lua's 200-local-per-function ceiling, and a
-- table costs one slot while holding the whole feature (functions + its UI state). Fields are
-- resolved at call time, so definition order among them does not matter.
local RTR = { destPage = 1, dest = nil }   -- dest = the hub currently being solved/previewed

-- The interchange hubs: the first and last point of every course. These are the only points
-- offered as destinations (all are landing sites) and the ones named in the route summary.
function RTR.routeHubNames()
    local set = {}
    for _, cname in ipairs(COURSE_ORDER) do
        local c = COURSES[cname]
        if c and #c.seq >= 1 then
            local a, b = c.seq[1], c.seq[#c.seq]
            if POINTS[a] then set[a] = true end
            if POINTS[b] then set[b] = true end
        end
    end
    return set
end

-- The route graph: nodes are points used by some course, edges are consecutive waypoint pairs
-- weighted by straight-line leg distance. Edges are UNDIRECTED -- a leg can be flown either way
-- (and reverse courses exist for every route anyway), so a waypoint that two courses both pass
-- through becomes a shared interchange node and a path can cut from one course to another
-- wherever they meet, not only at hub aerodromes. adj is name -> { neighbourName -> legDist }.
function RTR.buildRouteGraph()
    local adj = {}
    local function link(a, b)
        local pa, pb = POINTS[a], POINTS[b]
        if not pa or not pb or a == b then return end
        local dx, dz = pb.x - pa.x, pb.z - pa.z
        local w = math.sqrt(dx * dx + dz * dz)
        adj[a] = adj[a] or {}
        adj[b] = adj[b] or {}
        -- a leg shared by several courses collapses to one edge (keep the shortest, though the
        -- geometric length is identical whichever course it came from)
        if not adj[a][b] or w < adj[a][b] then adj[a][b] = w end
        if not adj[b][a] or w < adj[b][a] then adj[b][a] = w end
    end
    for _, cname in ipairs(COURSE_ORDER) do
        local c = COURSES[cname]
        if c then
            for i = 1, #c.seq - 1 do link(c.seq[i], c.seq[i + 1]) end
        end
    end
    return adj
end

-- Where the aircraft joins the network: the nearest node to the current fix. The first leg
-- flown is a direct hop to it; from there the plan follows published legs. nil without a fix.
function RTR.nearestGraphNode(adj)
    if not NAV.haveFix then return nil end
    local best, bestD = nil, math.huge
    for name in pairs(adj) do
        local p = POINTS[name]
        if p then
            local dx, dz = p.x - NAV.posX, p.z - NAV.posZ
            local d = dx * dx + dz * dz   -- squared: only the argmin matters
            if d < bestD then bestD, best = d, name end
        end
    end
    return best
end

-- Dijkstra over the route graph (a few dozen nodes -- a linear-scan settle is plenty, no heap
-- needed). Returns the point-name array from startName to goalName inclusive plus the total
-- path distance, or nil if the goal is unreachable (the UI errors out on that, per spec).
function RTR.solvePath(adj, startName, goalName)
    if not adj[startName] or not adj[goalName] then return nil end
    if startName == goalName then return {startName}, 0 end
    local dist, prev, done = {}, {}, {}
    for name in pairs(adj) do dist[name] = math.huge end
    dist[startName] = 0
    while true do
        -- settle the unvisited node with the smallest tentative distance
        local u, uD = nil, math.huge
        for name, d in pairs(dist) do
            if not done[name] and d < uD then uD, u = d, name end
        end
        if not u or uD == math.huge then break end   -- nothing left reachable
        if u == goalName then break end               -- shortest to the goal is now settled
        done[u] = true
        for v, w in pairs(adj[u]) do
            if not done[v] then
                local nd = uD + w
                if nd < dist[v] then dist[v] = nd; prev[v] = u end
            end
        end
    end
    if dist[goalName] == math.huge then return nil end
    local path, cur = {}, goalName
    while cur do
        table.insert(path, 1, cur)
        cur = prev[cur]
    end
    if path[1] ~= startName then return nil end       -- guard: must trace back to the start
    return path, dist[goalName]
end

-- Solve a plan from the current position to destName. Returns the point-name path, the total
-- path distance, and the join node -- or nil plus an error message. Pure (no NAV mutation), so
-- the preview screen can call it live on every redraw.
function RTR.solveRoute(destName)
    if not destName or not POINTS[destName] then return nil, nil, nil, "Unknown destination" end
    if not NAV.haveFix then return nil, nil, nil, "No position fix" end
    local adj = RTR.buildRouteGraph()
    local start = RTR.nearestGraphNode(adj)
    if not start then return nil, nil, nil, "No route network" end
    local path, total = RTR.solvePath(adj, start, destName)
    if not path then return nil, nil, nil, "No route to " .. destName end
    return path, total, start, nil
end

-- Commit a solved destination as the active plan: fly it (joining mid-route via
-- bestResumeIndex, the same heading-aware logic RESUME uses) and auto-land at the destination
-- hub on arrival, exactly like setRoute. Re-solves from the live fix so a moment's drift
-- between preview and confirm can't leave a stale plan. Returns true, or false + an error.
function RTR.setSolvedRoute(destName)
    local path, _, _, err = RTR.solveRoute(destName)
    if not path then return false, err or "No route" end
    local pts = {}
    for _, pname in ipairs(path) do
        local p = POINTS[pname]
        if p then table.insert(pts, {name = p.name, x = p.x, z = p.z, kind = p.kind}) end
    end
    if #pts == 0 then return false, "No route to " .. destName end
    LND.active = false
    LND.phase  = "OFF"
    LND.rwy    = false
    LND.ga     = false
    LND.gaCount = 0   -- a plan the pilot just committed to gets its full allowance of attempts
    NAV.cruiseMode = "full"    -- a fresh plan starts at full cruise, never a stale COAST
    NAV.aligning   = true      -- ...and points the nose at the first target before throttling up
    NAV.fromStart  = false     -- ...and joins the plan wherever it already is
    NAV.planName = "TO " .. destName
    -- The destination is a hub landing site: auto-land on arrival, same as a route to it --
    -- and, if it is a runway, gain a straight-in final leg before the plan is built.
    setLandFinal(pts[#pts].name, true)
    LND.rwyPrep(pts)
    NAV.points = pts
    NAV.index = bestResumeIndex(pts)     -- heading-aware mid-route join, same as RESUME
    NAV.active = true
    PLAN.build(NAV.points)
    updatePhase()
    broadcastCurrent()
    return true
end

-- A real disengage, not the old trick of commanding a HOLD target at the
-- current position: navcom drops the target entirely on msg.cancel and coasts
-- (zero thrust/yaw/strafe) rather than station-keeping at wherever the
-- aircraft happened to be -- autopilot off should hand control back, not pin
-- the aircraft to a spot it now has to fight to leave.
-- Serves both a normal cancel and the landing controls: ABORT (mid-descent) and
-- RESUME ALT-AP (after touchdown) both land here. Whenever a landing is/was in force
-- the vertical channel is handed back to ALT-AP -- so an abort climbs away as a
-- go-around, and RESUME lets the craft leave the pad on the ALT-AP cruise altitude.
-- vertCmd (default "cancel") is the autoland command sent to end the vertical channel:
-- "cancel" hands it back to ALT-AP (the AS restores its selection -> a go-around / leave
-- the pad on cruise), "off" is AP OFF (the AS stays deselected -> the UAC just holds the
-- ground altitude). Both drop the landing latch on the UAC identically.
local function cancelNav(vertCmd)
    if NAV.active or LND.active then
        rednet.broadcast({cancel = true}, APNAV_PROTOCOL)
    end
    if LND.active or LND.ga then
        rednet.broadcast({cmd = vertCmd or "cancel"}, AUTOLAND_PROTOCOL)
    end
    LND.active = false
    LND.phase  = "OFF"
    LND.paused = false
    LND.rwy    = false
    LND.ga     = false
    LND.gaCount = 0
    NAV.active = false
    NAV.aligning = false
    NAV.cmdSpeed = 0
    NAV.landAtEnd = false
    NAV.landSite  = nil
    NAV.planName = nil
    NAV.points = {}
    NAV.index = 1
    NAV.phase = "ENROUTE"
    PLAN.samples, PLAN.wpS, PLAN.totalLen, PLAN.cursor = {}, {}, 0, 1
    PLAN.lastX, PLAN.lastZ, PLAN.lastS, PLAN.lastD2 = nil, nil, 0, math.huge
end

local function navTick()
    local pose = sublevel.getLogicalPose()
    local pos = pose.position
    NAV.posX, NAV.posZ, NAV.posY, NAV.haveFix = pos.x, pos.z, pos.y, true
    -- Heading in navcom's exact frame, and vertical speed -- both needed by the landing
    -- state machine (alignment gate and touchdown detection).
    NAV.heading = HEADING_SIGN * getYaw(pose.orientation) + HEADING_OFFSET
    NAV.haveHeading = true
    local vel = sublevel.getLinearVelocity()
    NAV.velY = vel.y
    NAV.velX, NAV.velZ = vel.x, vel.z   -- horizontal velocity VECTOR: the runway localizer needs
                                        -- the rate its cross-track error is closing at, not just
                                        -- the speed (see LND.rwyGeom)
    NAV.groundSpeed = math.sqrt(vel.x * vel.x + vel.z * vel.z)   -- horizontal speed, for the HSI readout

    if LND.active then
        -- The landing owns the aircraft now: run its state machine and stream the vertical
        -- command; skip normal waypoint advance entirely.
        landingTick()
        broadcastVertical()
        return
    end
    -- A go-around is the other way round: the plan below flies the circuit, and all the
    -- landing machinery still does is hold the route's altitude.
    if LND.ga then
        LND.gaTick()
        broadcastVertical()
    end

    -- Route start: hold throttle at 0 (dynamicSpeedTarget) until the nose swings onto the first
    -- target, then release it for good -- so we never launch off on the old heading and arc round.
    if NAV.aligning then
        local herr = targetHeadingErr()
        if herr and math.abs(herr) <= math.rad(LG.ALIGN_HDG_DEG) then NAV.aligning = false end
    end

    if NAV.active and NAV.points[NAV.index] then
        -- Commanded speed, slew-rate-limited BOTH ways so it is a smooth curve, not a staircase:
        -- rise capped at DC.ACCEL_MAX (don't slam to cruise while still yawing onto a new leg),
        -- fall capped at DC.DECEL_MAX (any step in the raw envelope -- a turn dropping out of the
        -- lookahead, a kink where the binding fix switches -- eases down instead of snapping).
        -- DC.DECEL_MAX >= the envelope's own decel (the steeper of BRAKE_DECEL/ARRIVE_DECEL) so
        -- genuine slowdowns aren't delayed -- it only rounds off the sudden steps. This is the
        -- single CMD value.
        local rawSpeed = dynamicSpeedTarget()
        if rawSpeed > NAV.cmdSpeed then
            NAV.cmdSpeed = math.min(rawSpeed, NAV.cmdSpeed + DC.ACCEL_MAX * TICK_INTERVAL)
        else
            NAV.cmdSpeed = math.max(rawSpeed, NAV.cmdSpeed - DC.DECEL_MAX * TICK_INTERVAL)
        end

        local tgt = NAV.points[NAV.index]
        local dx, dz = tgt.x - pos.x, tgt.z - pos.z
        local dist = math.sqrt(dx * dx + dz * dz)
        local isFinal = (NAV.index == #NAV.points)
        -- Waypoint counts as passed either by flying inside its pass radius, OR (curve-following,
        -- en-route, not the destination) by the planned-path arc-length cursor crossing the
        -- waypoint's baked-in abeam point -- see PLAN.passed. Either is sufficient; radius alone
        -- can't detect a corner-cut pass.
        local passed = dist <= advanceRadiusFor(NAV.index, tgt, isFinal)
            or (not isFinal and DC.CURVE and NAV.phase == "ENROUTE" and PLAN.passed(NAV.index))
        if passed and not isFinal then
            NAV.index = NAV.index + 1
            NAV.fromStart = false   -- first waypoint genuinely reached: the plan is joined
            updatePhase() -- against the point we just switched to, not the one we passed
            broadcastCurrent() -- resend immediately on advance, not just on the 1s tick
        else
            updatePhase()
            -- Arrived at a landing-capable final point: engage the landing sequence
            -- automatically (fully automatic, per config).
            -- A RUNWAY arms far earlier -- ARRIVED means "stopped over the point", which for a
            -- strip is already the far side of the touchdown zone. It takes the aircraft at
            -- RWY_ARM_M out (i.e. as the planned FINAL fix goes by) and flies the approach in
            -- itself, so the en-route arrival brake never gets to slow it to a hover first.
            if isFinal and NAV.landAtEnd and (NAV.phase == "ARRIVED"
                    or (LND.rwyUsable(NAV.landSite) and dist <= LG.RWY_ARM_M)) then
                engageAutoland()
                broadcastCurrent()  -- switch NAVCOM into LANDING at once
                broadcastVertical() -- and take ALT-AP offline / hold altitude at once
            end
        end
    else
        NAV.phase = "ENROUTE"
    end
end

local function navLoop()
    local nextPing = 0
    while true do
        navTick()
        local now = os.clock()
        -- The 1 s keep-alive to NAVCOM (position/heading target). The vertical channel is
        -- streamed every tick from navTick instead, since its rate changes with altitude.
        if (NAV.active or LND.active) and now >= nextPing then
            broadcastCurrent()
            nextPing = now + PING_INTERVAL
        end
        sleep(TICK_INTERVAL)
    end
end

-- UI STATE -------------------------------------------------------------------

local screen = "home"
local pointsPage, coursesPage = 1, 1
local pickDirectPage, pickRoutePage = 1, 1
local pickLandPage = 1
local coursePickPage = 1
-- NOTE: the auto-router's page/preview state lives on the RTR table (RTR.destPage / RTR.dest),
-- not on separate locals here, to keep the main chunk under Lua's 200-local ceiling.

-- DIRECT-TO picker: sort mode + a 1 Hz cache of the sorted, distance-tagged list.
local pickDirectSort  = "nearest"  -- "nearest" | "az" | "za"
local pickDirectItems = {}
local pickDirectStamp = 0          -- os.clock() of the last recompute (0 = force one)

-- En-route (status) waypoint list scroll. nil = auto-scroll (keep the active waypoint
-- centred); a number pins the top waypoint index (manual browse) until RECENTRE.
local statusManualTop   = nil
local statusRenderedTop = 1        -- top index actually drawn, for the up/down buttons

-- Course-editor sequence scroll: index of the top waypoint shown.
local courseScroll = 1

local editingPointName = nil
local pointForm = {nameStr = "", xStr = "", zStr = ""}

local editingCourseName = nil
local courseForm = {nameStr = "", seq = {}}

-- NAV DATA UPDATE (NDU) --------------------------------------------------------
-- Keeps this aircraft's points.db / course.db / landing.db in step with the fleet's
-- master copy on the nav data update server (computer #24) without ever disturbing a
-- course being flown.
--
-- The aircraft still OWNS its files and still reads them at boot, so an aeroplane out of
-- radio contact has a complete database and flies normally -- the server only refreshes
-- them. The sync message carries the three .db files as plain text, so applying an update
-- is: write the files, then call the very same loadPoints()/loadCourses()/loadLanding()
-- used at boot. One parser in the fleet, and no way for the server's idea of the format to
-- drift from this program's.
--
-- WHY THIS CANNOT MOVE AN ACTIVE COURSE. NAV.points is a snapshot copied at plan-set time
-- and NAV.landSite likewise (see the NAV ENGINE header), so the flown plan holds no live
-- reference into POINTS/COURSES/LANDING. Swapping those tables therefore re-arms every
-- FUTURE plan and leaves the current one exactly as it was -- the same property that
-- already let the pilot edit a point mid-flight on the EDIT DATA screens.
--
-- What is NOT safe is swapping the database under a screen that is listing or editing it:
-- a picker would renumber under the pilot's finger between the draw and the tap, and the
-- point editor could save a record that no longer exists. So an update is held until the
-- FMS is back on HOME or STATUS -- see NDU.SAFE. Held updates are not lost; the server
-- re-broadcasts every 30 s and NDU.tryApply runs once a second regardless.
--
-- Data flows one way. Edits made here on EDIT DATA still work but are transient: the next
-- update from #24 overwrites them. Changes meant to last belong on the server.
local NDU = {
    PROTOCOL    = CFG.NAVDATA.PROTOCOL,     -- link + timings: see CFG.NAVDATA in apconf.lua
    PULL_EVERY  = CFG.NAVDATA.PULL_EVERY,
    STALE_AFTER = CFG.NAVDATA.STALE_AFTER,

    TMP         = CFG.NAVDATA.TMP,          -- suffix of the staging file each .db goes through

    rev = nil, stamp = nil,   -- the revision/checksum of what we currently hold
    serverId  = nil,
    lastHeard = nil,          -- os.clock() of the last message from the server
    lastApply = nil,
    pending   = nil,          -- a newer payload, waiting for a safe screen
    applied   = 0,            -- refreshes this session, for the status line
    fault     = nil,          -- why the last refresh could not be written, if it could not
}

-- The only two screens that neither list nor edit navigation data.
NDU.SAFE = {home = true, status = true}

-- A payload has to still look like a points file before it is allowed to replace one --
-- an empty or unparseable points.db arriving over the air would otherwise wipe the
-- aircraft's navigation database. courses/landing are accepted as sent (both can be
-- legitimately empty; points cannot, since everything else references them).
function NDU.sane(m)
    if type(m.points) ~= "string" or type(m.courses) ~= "string" then return false end
    if type(m.landing) ~= "string" then return false end
    return m.points:find("%a+=%-?%d+,%-?%d+") ~= nil
end

-- Write one file BESIDE the live one, never over it.
--
-- Opening a file for writing truncates it immediately, so the obvious version -- open the
-- real file and write -- destroys the aircraft's database the instant the write itself
-- fails. It does fail: a CC computer has a hard space limit (computer_space_limit, 1 MB by
-- default) and a computer that has accumulated old program versions can sit over it, at
-- which point every write throws "Out of space" and points.db is left at zero bytes with
-- the error swallowed by tryApply's pcall. Staging first makes a full disk harmless -- the
-- temp write fails, the live file is never opened, and the aeroplane keeps flying on the
-- database it already had.
--
-- The length actually on disk is checked rather than the return of write(): a partial
-- write is the exact failure this has to catch, and it is not always an error.
function NDU.stage(path, body)
    local tmp = path .. NDU.TMP
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
    local tmp = path .. NDU.TMP
    if not fs.exists(tmp) then return end
    pcall(fs.delete, path)      -- frees the old file's space; the move is only a rename
    pcall(fs.move, tmp, path)
end

function NDU.applyNow(m)
    -- Stage ALL THREE, then swap all three. A part-applied refresh is worse than none:
    -- new points beside old courses is a database whose routes name waypoints that are
    -- not there, and the FMS silently shortens a route around a missing waypoint.
    local files = {
        {POINTS_FILE, m.points}, {COURSE_FILE, m.courses}, {LANDING_FILE, m.landing},
    }
    for _, fl in ipairs(files) do
        if not NDU.stage(fl[1], fl[2]) then
            for _, g in ipairs(files) do pcall(fs.delete, g[1] .. NDU.TMP) end
            -- Recorded, not reported here: applyNow returning false leaves NDU.applied
            -- unchanged, so the loop's own once-per-broadcast report goes out carrying
            -- this fault. Reporting here as well would send it twice.
            NDU.fault = "cannot write - disk full?"
            return false
        end
    end
    for _, fl in ipairs(files) do NDU.swap(fl[1]) end
    NDU.fault = nil
    loadPoints()
    loadCourses()
    loadLanding()
    -- Everything derived from the old tables goes: a list page can now point past the end
    -- of its list, and the DIRECT-TO picker holds a 1 Hz cache of the sorted point list.
    -- (The route graph needs no invalidation -- RTR rebuilds it from scratch per solve.)
    pointsPage, coursesPage = 1, 1
    pickDirectPage, pickRoutePage, pickLandPage, coursePickPage = 1, 1, 1, 1
    pickDirectStamp = 0
    NDU.rev, NDU.stamp = m.rev, m.stamp
    NDU.lastApply = os.clock()
    NDU.applied = NDU.applied + 1
    NDU.report()
    return true
end

-- What this aircraft holds, and what it is sitting on and cannot load yet. Sent after
-- every apply AND after a broadcast that had to be deferred: without the second case a
-- client parked on a data screen looks exactly like one that never heard the server at
-- all -- same "?" name, same "?" revision, same never-in-sync -- which is the one thing
-- the dashboard exists to tell apart.
function NDU.report()
    if not NDU.serverId then return end
    rednet.send(NDU.serverId, {
        t      = "ack",
        rev    = NDU.rev,
        stamp  = NDU.stamp,
        held   = NDU.pending and NDU.pending.rev or nil,   -- the revision we are holding
        screen = NDU.pending and screen or nil,            -- ...and the screen blocking it
        fault  = NDU.fault,                                -- ...or why it could not be stored
        who    = os.getComputerLabel() or ("computer " .. os.getComputerID()),
    }, NDU.PROTOCOL)
end

-- A corner marker for the screens that block an update but have no room for the full
-- status line (the point/course lists, the pickers, the two detail editors). HOME and
-- EDIT DATA carry NDU.status() itself. Nobody should be able to sit on a blocking screen
-- without being told that is what they are doing.
function NDU.mark()
    if not NDU.pending then return end
    local w = term.getSize()
    term.setTextColour(colours.orange)
    term.setCursorPos(w - 8, 1)
    term.write("UPD HELD")
end

function NDU.tryApply()
    if not NDU.pending then return end
    if not NDU.SAFE[screen] then return end
    local m = NDU.pending
    NDU.pending = nil
    -- If the write half fails (read-only disk, no space) the stamp is never adopted, so
    -- the next 30 s broadcast simply offers the same update again.
    pcall(NDU.applyNow, m)
end

-- One line for the pilot: where the database came from, and whether anything is waiting.
function NDU.status()
    -- A refresh that cannot be written is the one state the pilot must not have to guess
    -- at: the aeroplane is flying valid but possibly outdated data and will keep failing
    -- every 30 s until the disk is cleared.
    if NDU.fault then
        return "NAV DATA " .. NDU.fault .. " (rev " .. tostring(NDU.rev or "local") .. " in use)",
            colours.red
    end
    if NDU.pending then
        return "NAV DATA rev " .. tostring(NDU.pending.rev) .. " held - back out to load",
            colours.orange
    end
    if not NDU.lastHeard then
        return "NAV DATA local copy, no server heard", colours.green
    end
    local age = os.clock() - NDU.lastHeard
    if age > NDU.STALE_AFTER then
        return string.format("NAV DATA rev %s, server silent %ds",
            tostring(NDU.rev or "?"), math.floor(age)), colours.brown
    end
    return string.format("NAV DATA rev %s from server #%s",
        tostring(NDU.rev or "?"), tostring(NDU.serverId or "?")), colours.green
end

function NDU.loop()
    local nextPull = 0
    while true do
        local sender, msg = rednet.receive(NDU.PROTOCOL, 1)
        if sender and type(msg) == "table" and msg.t == "sync" then
            NDU.serverId, NDU.lastHeard = sender, os.clock()
            -- Stamp, not rev: content equality is the real question, and it survives the
            -- server being rebuilt with its revision counter lost. A boot always applies
            -- once (our stamp starts nil) even if the content matches -- harmless, and it
            -- is what puts this aircraft on the server's roster.
            if msg.stamp ~= NDU.stamp and NDU.sane(msg) then NDU.pending = msg end
            -- Answer every broadcast, once, whatever the outcome -- applied, already had
            -- it, or holding it. That is what keeps this aircraft's row on the server's
            -- roster alive; reporting only on a change would let a perfectly healthy
            -- aeroplane age out to SILENT 90 s after its last edit. (If tryApply did
            -- apply, applyNow has already reported, so do not send twice.)
            local was = NDU.applied
            NDU.tryApply()
            if NDU.applied == was then NDU.report() end
        end
        -- The idle path: a held update lands within a second of the pilot leaving the
        -- screen, without waiting for the next broadcast. It reports through applyNow.
        NDU.tryApply()
        -- Ask, until the server has been heard from AT ALL. The condition is "never heard",
        -- not "never applied": an aircraft holding an update it cannot apply yet already
        -- has the data, and pulling for it again every 5 s would spam the server with full
        -- payloads for as long as the pilot stayed on a data screen.
        if not NDU.lastHeard and os.clock() >= nextPull then
            rednet.broadcast({t = "pull"}, NDU.PROTOCOL)
            nextPull = os.clock() + NDU.PULL_EVERY
        end
    end
end

-- LIST HELPERS -----------------------------------------------------------------

local function itemsFromPoints()
    local items = {}
    for _, name in ipairs(POINT_ORDER) do
        local p = POINTS[name]
        if p then
            -- Mark landing sites so a POI that can be landed at is visible in every list.
            local mark = LANDING[name] and (" <" .. LANDING[name].type:sub(1, 1):upper() .. ">") or ""
            table.insert(items, {
                key = name,
                label = string.format("%-6s %d,%d [%s]%s", p.name, p.x, p.z, p.kind:upper(), mark),
            })
        end
    end
    table.sort(items, function(a, b) return a.key < b.key end)
    return items
end

-- Only landing-capable sites, with their approach heading / altitude, for the LAND menu.
local function itemsFromLandingSites()
    local items = {}
    for name, li in pairs(LANDING) do
        local p = POINTS[name]
        if p then
            local extra = (li.type == "rwy") and string.format(" %dm", li.rwyLen) or ""
            table.insert(items, {
                key = name,
                label = string.format("%-6s %d,%d  %s %s%s  alt %d",
                    p.name, p.x, p.z, li.type:upper(), li.dir:upper(), extra, li.alt),
            })
        end
    end
    table.sort(items, function(a, b) return a.key < b.key end)
    return items
end

-- Destination hubs for the auto-router (FLY ROUTE > TO DESTINATION): the course endpoints,
-- distance-tagged from the current fix. These are the only points a solved route may end at.
function RTR.itemsFromHubs()
    local set = RTR.routeHubNames()
    local items = {}
    for name in pairs(set) do
        local p = POINTS[name]
        if p then
            local mark = LANDING[name] and (" <" .. LANDING[name].type:sub(1, 1):upper() .. ">") or ""
            local dstr = ""
            if NAV.haveFix then
                local dx, dz = p.x - NAV.posX, p.z - NAV.posZ
                dstr = string.format("  %.0fm", math.sqrt(dx * dx + dz * dz))
            end
            table.insert(items, {
                key = name,
                label = string.format("%-6s %d,%d [HUB]%s%s", p.name, p.x, p.z, mark, dstr),
            })
        end
    end
    table.sort(items, function(a, b) return a.key < b.key end)
    return items
end

-- Ordered for departure: nearest DEPARTURE (the course's first fix) first, and among courses
-- leaving the same place the SHORTEST flight first -- so the plans you could actually set off on
-- from here float to the top. Distance is course length (sum of its legs), not straight-line.
-- Courses sharing a departure tie exactly on dep, so the length key orders them; name breaks
-- any remaining tie for a stable list. No fix yet (or a dangling first point) -> dep = huge, so
-- those sort last on length alone rather than jumping around.
local function itemsFromCourses()
    local items = {}
    for _, name in ipairs(COURSE_ORDER) do
        local c = COURSES[name]
        if c then
            local dep, len = math.huge, 0
            local first = POINTS[c.seq[1]]
            if first and NAV.haveFix then
                local dx, dz = first.x - NAV.posX, first.z - NAV.posZ
                dep = math.sqrt(dx * dx + dz * dz)
            end
            for i = 2, #c.seq do
                local a, b = POINTS[c.seq[i - 1]], POINTS[c.seq[i]]
                if a and b then len = len + legLength(a, b) end
            end
            table.insert(items, {key = name, dep = dep, len = len,
                label = name .. " : " .. table.concat(c.seq, ">")})
        end
    end
    table.sort(items, function(a, b)
        if a.dep ~= b.dep then return a.dep < b.dep end
        if a.len ~= b.len then return a.len < b.len end
        return a.key < b.key
    end)
    return items
end

-- DIRECT-TO list, recomputed at most once a second (see refreshPickDirect): distance to
-- each point from the aircraft's current fix, sorted by the active mode. Caching it keeps
-- the generic ~5 Hz redraw from re-sorting (and visibly reordering) the list every frame
-- -- distances and order refresh together on the same 1 s cadence, which is the "live,
-- every second" the picker asks for.
local function computePickDirectItems()
    local items = {}
    for _, name in ipairs(POINT_ORDER) do
        local p = POINTS[name]
        if p then
            local dx, dz = p.x - NAV.posX, p.z - NAV.posZ
            items[#items + 1] = {
                key  = name,
                p    = p,
                dist = math.sqrt(dx * dx + dz * dz),
                mark = LANDING[name] and (" <" .. LANDING[name].type:sub(1, 1):upper() .. ">") or "",
            }
        end
    end
    if pickDirectSort == "az" then
        table.sort(items, function(a, b) return a.key < b.key end)
    elseif pickDirectSort == "za" then
        table.sort(items, function(a, b) return a.key > b.key end)
    else -- nearest; ties broken alphabetically so equidistant points hold a stable order
        table.sort(items, function(a, b)
            if a.dist ~= b.dist then return a.dist < b.dist end
            return a.key < b.key
        end)
    end
    pickDirectItems = items
    pickDirectStamp = os.clock()
end

-- Recompute when forced (mode change / screen entry, via stamp = 0) or the cache expired.
local function refreshPickDirect(force)
    if force or pickDirectStamp == 0 or (os.clock() - pickDirectStamp) >= 1.0 then
        computePickDirectItems()
    end
end

local function rowsAvailable()
    local w, h = term.getSize()
    return h - 5
end

local function renderList(title, items, page, showAdd, addLabel)
    local w, h = term.getSize()
    term.setBackgroundColour(colours.black)
    term.clear()
    term.setTextColour(colours.orange)
    term.setCursorPos(2, 1)
    term.write(title)
    NDU.mark()   -- every list screen is one that defers an update

    local rows = rowsAvailable()
    local first = (page - 1) * rows + 1
    for i = 1, rows do
        local idx = first + i - 1
        local it = items[idx]
        local y = 2 + i
        if it then
            term.setBackgroundColour(colours.black)
            term.setTextColour(colours.white)
            term.setCursorPos(2, y)
            local label = it.label
            if #label > w - 2 then label = label:sub(1, w - 2) end
            term.write(label)
            addHitbox(1, y, w, y, "ITEM:" .. it.key)
        end
    end

    local by = h - 1
    local bx = 2
    if showAdd then
        button(bx, by, 10, 1, "ADD", addLabel or "+ ADD", true, colours.lime)
        bx = bx + 11
    end
    if first > 1 then
        button(bx, by, 6, 1, "PREV", "<<", true, colours.orange)
        bx = bx + 7
    end
    if first + rows - 1 < #items then
        button(bx, by, 6, 1, "NEXT", ">>", true, colours.orange)
    end
    button(w - 9, by, 8, 1, "BACK", "BACK", true, colours.brown)
end

-- SCREENS ----------------------------------------------------------------------

local function drawHome()
    local w, h = term.getSize()
    term.setBackgroundColour(colours.black)
    term.clear()
    term.setTextColour(colours.orange)
    term.setCursorPos(2, 1)
    term.write("AER TOGEKISS  AUTOPILOT")

    term.setTextColour(colours.white)
    term.setCursorPos(2, 3)
    if NAV.haveFix then
        term.write(string.format("POS  X %.1f  Z %.1f", NAV.posX, NAV.posZ))
    else
        term.write("POS  no fix")
    end

    term.setCursorPos(2, 4)
    if LND.active then
        term.setTextColour(colours.orange)
        term.write(string.format("LANDING %s  %s", LND.name, LND.phase))
    elseif NAV.active then
        local tgt = NAV.points[NAV.index]
        local dx, dz = tgt.x - NAV.posX, tgt.z - NAV.posZ
        local dist = math.sqrt(dx * dx + dz * dz)
        term.setTextColour(colours.lime)
        term.write(string.format("%s -> %s  (%.0fm)", NAV.planName, tgt.name, dist))
    else
        term.setTextColour(colours.green)
        term.write("IDLE - no course set")
    end

    -- Where the navigation database came from. Worth a line on the home page: a plan is
    -- only as current as the points it was built from.
    local ndText, ndCol = NDU.status()
    term.setCursorPos(2, 5)
    term.setTextColour(ndCol)
    term.write(ndText:sub(1, w - 2))

    button(2, 7, 20, 3, "DIRECT", "DIRECT TO", true, colours.orange)
    button(2, 11, 20, 3, "ROUTE", "FLY ROUTE", true, colours.orange)
    button(2, 15, 20, 3, "EDIT", "EDIT DATA", true, colours.orange)
    button(w - 17, 7, 15, 3, "LAND", "LAND", true, colours.lime)
    button(w - 17, 11, 15, 3, "STATUS", "STATUS", NAV.active or LND.active, colours.lime)
end

local function touchHome(id)
    if id == "DIRECT" then
        pickDirectPage = 1
        pickDirectStamp = 0   -- force a fresh distance sort against the current fix
        screen = "pickDirect"
    elseif id == "ROUTE" then
        screen = "routeMenu"
    elseif id == "LAND" then
        pickLandPage = 1
        screen = "pickLand"
    elseif id == "EDIT" then
        screen = "dataMenu"
    elseif id == "STATUS" then
        screen = "status"
    end
end

local function drawPickLand()
    renderList("LAND AT - select site", itemsFromLandingSites(), pickLandPage, false)
end

local function touchPickLand(id)
    if id == "PREV" then
        pickLandPage = math.max(1, pickLandPage - 1)
    elseif id == "NEXT" then
        pickLandPage = pickLandPage + 1
    elseif id == "BACK" then
        screen = "home"
    elseif id and id:sub(1, 5) == "ITEM:" then
        local name = id:sub(6)
        statusManualTop = nil   -- fresh plan -> auto-scroll the en-route list
        -- Direct-to the site with land-at-end armed: fly there normally, then the landing
        -- sequence engages automatically on arrival.
        if setDirectTo(name, true) then screen = "status" end
    end
end

-- DIRECT TO picker: distance-tagged, sortable (nearest / A-Z / Z-A), refreshing the
-- distances and order once a second. Custom-rendered rather than via renderList because
-- it carries a right-hand distance column and a sort-mode button row.
local function drawPickDirect()
    refreshPickDirect(false)
    local w, h = term.getSize()
    term.setBackgroundColour(colours.black)
    term.clear()
    term.setTextColour(colours.orange)
    term.setCursorPos(2, 1)
    term.write("DIRECT TO - select point")

    local items = pickDirectItems
    local rows  = h - 5                 -- two button rows (sort + paging) below the list
    local first = (pickDirectPage - 1) * rows + 1
    for i = 1, rows do
        local it = items[first + i - 1]
        local y  = 2 + i
        if it then
            term.setBackgroundColour(colours.black)
            term.setTextColour(colours.white)
            local diststr = NAV.haveFix and string.format("%.0fm", it.dist) or "--"
            local label = string.format("%-6s %d,%d [%s]%s",
                it.p.name, it.p.x, it.p.z, it.p.kind:upper(), it.mark)
            local room = w - #diststr - 3
            if #label > room then label = label:sub(1, room) end
            term.setCursorPos(2, y); term.write(label)
            term.setCursorPos(math.max(2, w - #diststr - 1), y); term.write(diststr)
            addHitbox(1, y, w, y, "ITEM:" .. it.key)
        end
    end

    -- Sort-mode row: the active mode is lime, the others orange.
    local sy = h - 2
    local function sortBtn(x, id, label, mode)
        button(x, sy, 7, 1, id, label, true,
            (pickDirectSort == mode) and colours.lime or colours.orange)
    end
    sortBtn(2,  "SORT_NEAR", "NEAR", "nearest")
    sortBtn(10, "SORT_AZ",   "A-Z",  "az")
    sortBtn(18, "SORT_ZA",   "Z-A",  "za")

    -- Paging + back row.
    local by = h - 1
    local bx = 2
    if first > 1 then
        button(bx, by, 6, 1, "PREV", "<<", true, colours.orange); bx = bx + 7
    end
    if first + rows - 1 < #items then
        button(bx, by, 6, 1, "NEXT", ">>", true, colours.orange)
    end
    button(w - 9, by, 8, 1, "BACK", "BACK", true, colours.brown)
end

local function touchPickDirect(id)
    if id == "PREV" then
        pickDirectPage = math.max(1, pickDirectPage - 1)
    elseif id == "NEXT" then
        pickDirectPage = pickDirectPage + 1
    elseif id == "SORT_NEAR" then
        pickDirectSort = "nearest"; pickDirectPage = 1; refreshPickDirect(true)
    elseif id == "SORT_AZ" then
        pickDirectSort = "az"; pickDirectPage = 1; refreshPickDirect(true)
    elseif id == "SORT_ZA" then
        pickDirectSort = "za"; pickDirectPage = 1; refreshPickDirect(true)
    elseif id == "BACK" then
        screen = "home"
    elseif id and id:sub(1, 5) == "ITEM:" then
        local name = id:sub(6)
        statusManualTop = nil   -- fresh plan -> auto-scroll the en-route list
        if setDirectTo(name) then screen = "status" end
    end
end

local function drawPickRoute()
    renderList("FLY ROUTE - select course", itemsFromCourses(), pickRoutePage, false)
end

-- After a route is picked, ask whether to fly it from the start or resume mid-route (join
-- at the best waypoint for our current position/heading). Blocking modal, same self-contained
-- pattern as confirm(): it owns the screen until answered and is re-woken ~5x/s by navLoop's
-- timer events, so the RESUME preview (join waypoint + range) stays live. Returns "start",
-- "resume", or nil (cancelled).
local function chooseRouteStart(name)
    local c = COURSES[name]
    if not c then return nil end
    -- Same point list setRoute will build, so the preview matches what RESUME actually flies.
    local pts = {}
    for _, pname in ipairs(c.seq) do
        local p = POINTS[pname]
        if p then table.insert(pts, {name = p.name, x = p.x, z = p.z, kind = p.kind}) end
    end
    while true do
        clearHitboxes()
        term.setBackgroundColour(colours.black)
        term.clear()
        local w, h = term.getSize()
        term.setTextColour(colours.orange)
        term.setCursorPos(2, 1); term.write("FLY ROUTE  " .. name)

        term.setTextColour(colours.white)
        term.setCursorPos(2, 3); term.write("Start this plan from:")

        local ri = bestResumeIndex(pts)
        term.setTextColour(colours.green)
        term.setCursorPos(2, 5)
        term.write("Start:  " .. (pts[1] and pts[1].name or "?"))
        term.setCursorPos(2, 6)
        if not NAV.haveFix then
            term.write("Resume: no fix")
        elseif pts[ri] then
            local dx, dz = pts[ri].x - NAV.posX, pts[ri].z - NAV.posZ
            term.write(string.format("Resume: %s  (%.0fm)", pts[ri].name, math.sqrt(dx*dx + dz*dz)))
        else
            term.write("Resume: --")
        end

        button(2, 9, 22, 3, "START", "FROM START", true, colours.orange)
        button(2, 13, 22, 3, "RESUME", "RESUME MID-ROUTE", NAV.haveFix, colours.lime)
        button(w - 9, h - 1, 8, 1, "CANCEL", "CANCEL", true, colours.brown)

        local ev = {os.pullEvent()}
        local x, y = pointerXY(ev)
        if x then
            local id = hitTest(x, y)
            if id == "START" then return "start"
            elseif id == "RESUME" then return "resume"
            elseif id == "CANCEL" then return nil end
        end
    end
end

local function touchPickRoute(id)
    if id == "PREV" then
        pickRoutePage = math.max(1, pickRoutePage - 1)
    elseif id == "NEXT" then
        pickRoutePage = pickRoutePage + 1
    elseif id == "BACK" then
        screen = "routeMenu"
    elseif id and id:sub(1, 5) == "ITEM:" then
        local name = id:sub(6)
        local choice = chooseRouteStart(name)   -- FROM START / RESUME MID-ROUTE / cancel
        if choice then
            statusManualTop = nil   -- fresh plan -> auto-scroll the en-route list
            if setRoute(name, choice == "resume") then screen = "status" end
        end
    end
end

-- FLY ROUTE sub-menu: pick a published course by name (existing resume-mid-route flow), or let
-- the FMS solve a plan to a destination hub from the current position (the auto-router).
function RTR.drawRouteMenu()
    local w, h = term.getSize()
    term.setBackgroundColour(colours.black)
    term.clear()
    term.setTextColour(colours.orange)
    term.setCursorPos(2, 1); term.write("FLY ROUTE")
    term.setTextColour(colours.white)
    term.setCursorPos(2, 3); term.write("Plan a route by:")

    button(2, 5, 26, 3, "BYCOURSE", "BY COURSE", true, colours.orange)
    term.setTextColour(colours.white)
    term.setCursorPos(2, 8); term.write("fly one named course")

    button(2, 11, 26, 3, "TODEST", "TO DESTINATION", true, colours.lime)
    term.setTextColour(colours.white)
    term.setCursorPos(2, 14); term.write("auto-solve via courses + waypoints")

    button(2, h - 1, 10, 1, "BACK", "BACK", true, colours.brown)
end

function RTR.touchRouteMenu(id)
    if id == "BYCOURSE" then
        pickRoutePage = 1
        screen = "pickRoute"
    elseif id == "TODEST" then
        RTR.destPage = 1
        screen = "pickDest"
    elseif id == "BACK" then
        screen = "home"
    end
end

-- Destination picker for the auto-router: the network hubs only (course endpoints). Selecting
-- one solves a plan immediately; a solvable one goes to the preview, an unsolvable one errors
-- out (per requirement) rather than silently doing nothing.
function RTR.drawPickDest()
    renderList("TO DESTINATION - select hub", RTR.itemsFromHubs(), RTR.destPage, false)
end

function RTR.touchPickDest(id)
    if id == "PREV" then
        RTR.destPage = math.max(1, RTR.destPage - 1)
    elseif id == "NEXT" then
        RTR.destPage = RTR.destPage + 1
    elseif id == "BACK" then
        screen = "routeMenu"
    elseif id and id:sub(1, 5) == "ITEM:" then
        local name = id:sub(6)
        local path, _, _, err = RTR.solveRoute(name)
        if path then
            RTR.dest = name
            screen = "planPreview"
        else
            alert("NO ROUTE", err or ("No path to " .. name))
        end
    end
end

-- One-line summary of a solved path: its origin node, every interchange hub it crosses, and
-- the destination, joined with '>' (the full waypoint list is on STATUS once committed).
function RTR.routeSummary(path)
    local hubs = RTR.routeHubNames()
    local parts, n = {}, #path
    for i, name in ipairs(path) do
        if i == 1 or i == n or hubs[name] then
            if parts[#parts] ~= name then table.insert(parts, name) end   -- no dupe if endpoint is a hub
        end
    end
    return table.concat(parts, ">")
end

-- Preview of the solved plan before committing: the via-summary, waypoint count / total range,
-- the mid-route join point, and the auto-land destination. Re-solved live each redraw (like
-- chooseRouteStart) so the join range stays current; FLY commits it, BACK returns to the picker.
function RTR.drawPlanPreview()
    local w, h = term.getSize()
    term.setBackgroundColour(colours.black)
    term.clear()
    term.setTextColour(colours.orange)
    term.setCursorPos(2, 1); term.write("ROUTE TO " .. (RTR.dest or "?"))

    local path, total, _, err = RTR.solveRoute(RTR.dest)
    if not path then
        -- Solvable when picked, not now (e.g. the fix dropped): show why, offer only BACK.
        term.setTextColour(colours.white)
        term.setCursorPos(2, 3); term.write(err or "No route")
        button(2, h - 1, 10, 1, "BACK", "BACK", true, colours.brown)
        return
    end

    -- Snapshot points so bestResumeIndex can preview the same join the commit will make.
    local pts = {}
    for _, pname in ipairs(path) do
        local p = POINTS[pname]
        if p then table.insert(pts, {name = p.name, x = p.x, z = p.z, kind = p.kind}) end
    end
    local ri = bestResumeIndex(pts)

    term.setTextColour(colours.white)
    term.setCursorPos(2, 3); term.write("VIA")

    -- Wrap the '>'-joined summary to the screen width.
    term.setTextColour(colours.lime)
    local yy, line = 4, ""
    for tok in (RTR.routeSummary(path)):gmatch("[^>]+") do
        local piece = (line == "") and tok or (line .. ">" .. tok)
        if #piece > w - 3 and line ~= "" then
            term.setCursorPos(2, yy); term.write(line)
            yy, line = yy + 1, tok
        else
            line = piece
        end
    end
    if line ~= "" then term.setCursorPos(2, yy); term.write(line); yy = yy + 1 end

    local infoY = yy + 1
    term.setTextColour(colours.white)
    term.setCursorPos(2, infoY)
    term.write(string.format("%d waypoints  %.0fm total", #pts, total or 0))
    term.setCursorPos(2, infoY + 1)
    if pts[ri] then
        local dx, dz = pts[ri].x - NAV.posX, pts[ri].z - NAV.posZ
        term.write(string.format("Join at %s  (%.0fm)", pts[ri].name, math.sqrt(dx * dx + dz * dz)))
    end
    -- Most hubs are landing sites and auto-land on arrival; a hub without a landing.db entry
    -- (e.g. VLP) is flown to and held instead -- say which, so the preview never over-promises.
    local dest = pts[#pts] and pts[#pts].name or "?"
    term.setCursorPos(2, infoY + 2)
    if isLandingSite(dest) then
        term.setTextColour(colours.green)
        term.write("Auto-lands at " .. dest)
    else
        term.setTextColour(colours.orange)
        term.write("Holds at " .. dest .. " (no landing site)")
    end

    button(2, h - 1, 12, 1, "FLY", "FLY ROUTE", true, colours.lime)
    button(w - 9, h - 1, 8, 1, "BACK", "BACK", true, colours.brown)
end

function RTR.touchPlanPreview(id)
    if id == "BACK" then
        screen = "pickDest"
    elseif id == "FLY" then
        local ok, err = RTR.setSolvedRoute(RTR.dest)
        if ok then
            statusManualTop = nil   -- fresh plan -> auto-scroll the en-route list
            screen = "status"
        else
            alert("NO ROUTE", err or "No route")
        end
    end
end

-- The landing HUD, shown on the status screen while a landing is engaged: the live
-- gate numbers (position/heading error, AGL, commanded VS) and the phase, plus ABORT
-- (mid-air) or RESUME ALT-AP (after touchdown).
local function drawLandingStatus()
    local w, h = term.getSize()
    term.setBackgroundColour(colours.black)
    term.clear()

    term.setTextColour(colours.orange)
    term.setCursorPos(2, 1)
    -- A runway approach names the two numbers it sized itself with: the approach speed the
    -- glideslope and the runway length allowed, and the glideslope being flown.
    term.write("AUTOLAND " .. LND.name .. (LND.rwy
        and string.format(" RWY %.0fm/s %sdeg", LND.appSpeed, tostring(LG.RWY_GS_DEG)) or ""))
    term.setCursorPos(math.max(2, w - #LND.phase), 1); term.write(LND.phase)

    term.setTextColour(colours.white)
    term.setCursorPos(2, 3)
    term.write(string.format("POS  X %.1f  Z %.1f", NAV.posX, NAV.posZ))
    term.setCursorPos(2, 4)
    if LND.rwy then
        -- Runway: the errors that matter are along/across the strip, not range to a point.
        term.write(string.format("XTK %+6.1f m  (<=%d)", LND.xtk, LG.RWY_CAPT_XTK))
    else
        term.write(string.format("POS ERR  %5.1f m  (<=%d)", LND.posErr, LG.POS_TOL_M))
    end
    term.setCursorPos(2, 5)
    if LND.holdHeading then
        term.write(string.format("HDG ERR %+5.1f deg", LND.hdgErrDeg))
    else
        term.write("HDG ERR   -- (any)")
    end
    term.setCursorPos(2, 6)
    if LND.rwy then
        term.write(string.format("AGL %6.1f m   GS %+6.1f m", LND.agl, LND.gsDev))
    else
        term.write(string.format("AGL  %6.1f m", LND.agl))
    end
    term.setCursorPos(2, 7)
    if LND.rwy then
        term.write(string.format("VS  %+5.2f m/s  TO RUN %5.0f m  CMD %3.0f",
            LND.vs, LND.along, LND.cmdSpeed))
    else
        term.write(string.format("VS   %+5.2f m/s", LND.vs))
    end

    -- Squat switch. Always shown, including "no signal": whether the gear contact is being
    -- heard decides whether this landing can end on it or is back to the altimeter alone.
    local sq, sqCol
    if not TD.fresh()  then sq, sqCol = "no signal (AGL only)", colours.brown
    elseif TD.signal   then sq, sqCol = "DOWN", colours.lime
    else                    sq, sqCol = "airborne", colours.white end
    if LND.phase == "LANDED" then sq = sq .. " - by " .. (LND.tdBy or "?") end
    term.setTextColour(sqCol)
    term.setCursorPos(2, 8); term.write("SQUAT  " .. sq)
    term.setTextColour(colours.white)

    local hint = {
        ALIGN     = "aligning heading...",
        TRANSLATE = "closing on the pad...",
        DESCEND   = "descending",
        FLARE     = "flare - easing for touchdown",
        LANDED    = "LANDED - engines on ground hold. STOP engine NOW",
        -- runway approach
        INTERCEPT = "lining up on the centreline (level)",
        GLIDE     = "on the glideslope",
        ROLLOUT   = "rolling out - braking on the runway",
        MISSED    = "PAST THE TOUCHDOWN ZONE - level. ABORT to go around",
    }
    local hintText = LND.paused and "PILOT ALT-AP -- descent PAUSED" or (hint[LND.phase] or "")
    term.setTextColour((LND.phase == "MISSED" and colours.red)
        or ((LND.phase == "LANDED" or LND.paused) and colours.lime) or colours.orange)
    term.setCursorPos(2, 9); term.write(hintText)

    -- Same layout convention as the en-route screen: HOME on the left, the destructive /
    -- action button on the right edge so it is not tapped by accident.
    local by = h - 1
    button(2, by, 8, 1, "HOME", "HOME", true, colours.brown)
    if LND.paused then
        -- Pilot has ALT-AP: resume the descent, or abort the landing entirely.
        button(w - 27, by, 12, 1, "RESUMELND", "RESUME LND", true, colours.lime)
        button(w - 11, by, 10, 1, "CANCEL", "ABORT", true, colours.orange)
    elseif LND.phase == "LANDED" then
        -- Two options after touchdown: park (AP OFF, hold ground) or wake ALT-AP (leave
        -- the pad on cruise). Tapping the selector directly also wakes ALT-AP (overrideLoop).
        button(w - 28, by, 11, 1, "APOFF", "AP OFF", true, colours.orange)
        button(w - 15, by, 14, 1, "RESUME", "RESUME ALT-AP", true, colours.lime)
    else
        button(w - 11, by, 10, 1, "CANCEL", "ABORT", true, colours.orange)
    end
end

local function drawStatus()
    if LND.active then
        drawLandingStatus()
        return
    end
    local w, h = term.getSize()
    term.setBackgroundColour(colours.black)
    term.clear()
    term.setTextColour(colours.orange)
    term.setCursorPos(2, 1)
    term.write("ROUTE: " .. (NAV.planName or "--"))
    term.setCursorPos(math.max(2, w - 12), 1)
    term.write(NAV.phase or "--")

    term.setTextColour(colours.white)
    term.setCursorPos(2, 2)
    if NAV.haveFix then
        term.write(string.format("WE ARE HERE  X %.1f  Z %.1f", NAV.posX, NAV.posZ))
    else
        term.write("NO POSITION FIX")
    end

    local remain = remainingDistances()
    local total  = #NAV.points

    -- Scroll: if the whole route fits, show it all; otherwise reserve a row for the scroll
    -- controls and window the list. Auto-scroll keeps the active waypoint centred
    -- (statusManualTop == nil); the up/down buttons pin the window until RECENTRE.
    local rowsNoScroll   = (h - 2) - 3 + 1     -- items on rows 3..h-2
    local rowsWithScroll = (h - 3) - 3 + 1     -- items on rows 3..h-3, scroll row at h-2
    local scrollNeeded   = total > rowsNoScroll
    local rows   = scrollNeeded and rowsWithScroll or rowsNoScroll
    local maxTop = math.max(1, total - rows + 1)

    local top
    if statusManualTop then
        top = clamp(statusManualTop, 1, maxTop)
    else
        top = clamp(NAV.index - math.floor((rows - 1) / 2), 1, maxTop)
    end
    statusRenderedTop = top

    for disp = 0, rows - 1 do
        local i  = top + disp
        local pt = NAV.points[i]
        if pt then
            local y = 3 + disp
            local fg, tag
            if i < NAV.index then
                fg, tag = colours.green, "PASSED"
            elseif i == NAV.index then
                fg, tag = colours.orange, string.format("%.0fm", remain[i] or 0)
            else
                fg, tag = colours.white, string.format("%.0fm togo", remain[i] or 0)
            end
            local isFinal = (i == total)
            local kindLbl = isFinal and "END" or pt.kind:upper()
            term.setTextColour(fg)
            term.setCursorPos(2, y)
            term.write(string.format("%d. %-6s [%s]", i, pt.name, kindLbl))
            term.setCursorPos(math.max(2, w - 16), y)
            term.write(tag)
        end
    end

    if scrollNeeded then
        local sy = h - 2
        button(2,  sy, 5, 1, "SCRL_UP",  "UP",       top > 1,               colours.orange)
        button(8,  sy, 5, 1, "SCRL_DN",  "DN",       top < maxTop,          colours.orange)
        button(14, sy, 9, 1, "SCRL_CTR", "RECENTRE", statusManualTop ~= nil, colours.lime)
        term.setTextColour(colours.white)
        local info = string.format("%d-%d/%d %s", top, math.min(total, top + rows - 1),
            total, statusManualTop and "MAN" or "AUTO")
        term.setCursorPos(math.max(24, w - #info - 1), sy)
        term.write(info)
    end

    -- Speed selector: enabled (tappable) only while cruising, since its cap only bites in
    -- ENROUTE. Greyed the rest of the time, but still showing the current selection.
    -- HOME sits on the left (under the scroll buttons, where stray taps land); CANCEL is
    -- exiled to the right edge so it is not hit by accident while scrolling.
    local cruising = NAV.active and NAV.phase == "ENROUTE"
    button(2, h - 1, 8, 1, "HOME", "HOME", true, colours.brown)
    button(14, h - 1, 14, 1, "SPD", "SPD " .. cruiseModeLabel(), cruising, colours.lime)
    button(w - 11, h - 1, 10, 1, "CANCEL", "CANCEL", true, colours.orange)
end

local function touchStatus(id)
    if id == "CANCEL" or id == "RESUME" then
        -- ABORT (mid-descent) and RESUME ALT-AP (after touchdown) are the same teardown:
        -- stop NAVCOM and hand the vertical channel back to ALT-AP (see cancelNav).
        cancelNav()
        screen = "home"
    elseif id == "RESUMELND" then
        -- Leave the pilot-override pause and retake the descent. Streaming "descend" again
        -- re-overshadows the altitude selector on its own.
        LND.paused = false
        broadcastVertical()
    elseif id == "APOFF" then
        -- AP OFF (after touchdown): drop the landing latch but leave the selector OFF, so
        -- the UAC just holds the ground altitude. The pilot re-arms ALT-AP later by tapping
        -- a panel. Same teardown as RESUME, only the autoland cmd differs.
        cancelNav("off")
        screen = "home"
    elseif id == "HOME" then
        screen = "home"
    elseif id == "SCRL_UP" then
        -- Any manual scroll leaves auto-centre; draw clamps the value into range.
        statusManualTop = math.max(1, statusRenderedTop - 1)
    elseif id == "SCRL_DN" then
        statusManualTop = statusRenderedTop + 1
    elseif id == "SCRL_CTR" then
        statusManualTop = nil   -- back to auto-scroll (centre on the active waypoint)
    elseif id == "SPD" then
        -- Cycle FULL -> SLOW -> COAST -> FULL, and push the new cap to NAVCOM at once
        -- rather than waiting for the next 1 s ping.
        NAV.cruiseMode = NEXT_CRUISE[NAV.cruiseMode] or "full"
        broadcastCurrent()
    end
end

local function drawDataMenu()
    local w, h = term.getSize()
    term.setBackgroundColour(colours.black)
    term.clear()
    term.setTextColour(colours.orange)
    term.setCursorPos(2, 1)
    term.write("EDIT DATA")
    button(2, 4, 20, 3, "POINTS", "POINTS", true, colours.orange)
    button(2, 8, 20, 3, "COURSES", "COURSES", true, colours.orange)

    -- An update that arrived while the pilot is in here is held rather than applied (see
    -- NDU.SAFE), so say so, and say plainly that this editor is no longer the master copy
    -- once a server is on the air.
    local ndText, ndCol = NDU.status()
    term.setTextColour(ndCol)
    term.setCursorPos(2, 12); term.write(ndText:sub(1, w - 2))
    if NDU.lastHeard then
        term.setTextColour(colours.brown)
        term.setCursorPos(2, 13); term.write("edits here last only until the next")
        term.setCursorPos(2, 14); term.write("update - edit on the data server")
    end

    button(2, h - 2, 10, 1, "BACK", "BACK", true, colours.brown)
end

local function touchDataMenu(id)
    if id == "POINTS" then
        pointsPage = 1
        screen = "pointsList"
    elseif id == "COURSES" then
        coursesPage = 1
        screen = "coursesList"
    elseif id == "BACK" then
        screen = "home"
    end
end

local function drawPointsList()
    renderList("POINTS", itemsFromPoints(), pointsPage, true, "+ ADD")
end

local function touchPointsList(id)
    if id == "PREV" then
        pointsPage = math.max(1, pointsPage - 1)
    elseif id == "NEXT" then
        pointsPage = pointsPage + 1
    elseif id == "BACK" then
        screen = "dataMenu"
    elseif id == "ADD" then
        editingPointName = nil
        pointForm = {nameStr = "", xStr = "", zStr = ""}
        screen = "pointDetail"
    elseif id and id:sub(1, 5) == "ITEM:" then
        local name = id:sub(6)
        local p = POINTS[name]
        if p then
            editingPointName = name
            pointForm = {nameStr = p.name, xStr = tostring(p.x), zStr = tostring(p.z)}
            screen = "pointDetail"
        end
    end
end

local function drawPointDetail()
    local w, h = term.getSize()
    term.setBackgroundColour(colours.black)
    term.clear()
    term.setTextColour(colours.orange)
    term.setCursorPos(2, 1)
    term.write(editingPointName and ("EDIT POINT " .. editingPointName) or "NEW POINT")
    NDU.mark()

    local kindLbl = "?"
    if #pointForm.nameStr == 3 then kindLbl = "POI"
    elseif #pointForm.nameStr == 5 then kindLbl = "VOR" end

    term.setTextColour(colours.white)
    term.setCursorPos(2, 3); term.write("NAME")
    button(9, 3, 10, 1, "F:NAME", pointForm.nameStr ~= "" and pointForm.nameStr or "----", true, colours.lime)
    term.setCursorPos(21, 3); term.write("[" .. kindLbl .. "]")

    term.setCursorPos(2, 5); term.write("X")
    button(9, 5, 10, 1, "F:X", pointForm.xStr ~= "" and pointForm.xStr or "----", true, colours.lime)

    term.setCursorPos(2, 7); term.write("Z")
    button(9, 7, 10, 1, "F:Z", pointForm.zStr ~= "" and pointForm.zStr or "----", true, colours.lime)

    local by = h - 1
    button(2, by, 8, 1, "SAVE", "SAVE", true, colours.orange)
    button(11, by, 10, 1, "DELETE", "DELETE", editingPointName ~= nil, colours.orange)
    button(22, by, 10, 1, "CANCEL", "CANCEL", true, colours.brown)
end

local function touchPointDetail(id)
    if id == "F:NAME" then
        local v = promptKeyboard("POINT NAME", pointForm.nameStr, "point")
        if v then pointForm.nameStr = v end
    elseif id == "F:X" then
        local v = promptKeyboard("X COORD", pointForm.xStr, "numeric")
        if v then pointForm.xStr = v end
    elseif id == "F:Z" then
        local v = promptKeyboard("Z COORD", pointForm.zStr, "numeric")
        if v then pointForm.zStr = v end
    elseif id == "SAVE" then
        local x, z = tonumber(pointForm.xStr), tonumber(pointForm.zStr)
        if not x or not z then
            alert("ERROR", "X and Z must be numbers")
        else
            local ok, err
            if editingPointName then
                ok, err = editPoint(editingPointName, pointForm.nameStr, x, z)
            else
                ok, err = addPoint(pointForm.nameStr, x, z)
            end
            if ok then
                screen = "pointsList"
            else
                alert("ERROR", err)
            end
        end
    elseif id == "DELETE" then
        if confirm("DELETE POINT", "Delete " .. editingPointName .. "?") then
            local ok, err = deletePoint(editingPointName)
            if ok then
                screen = "pointsList"
            else
                alert("ERROR", err)
            end
        end
    elseif id == "CANCEL" then
        screen = "pointsList"
    end
end

local function drawCoursesList()
    renderList("COURSES", itemsFromCourses(), coursesPage, true, "+ ADD")
end

local function touchCoursesList(id)
    if id == "PREV" then
        coursesPage = math.max(1, coursesPage - 1)
    elseif id == "NEXT" then
        coursesPage = coursesPage + 1
    elseif id == "BACK" then
        screen = "dataMenu"
    elseif id == "ADD" then
        editingCourseName = nil
        courseForm = {nameStr = "", seq = {}}
        courseScroll = 1
        screen = "courseDetail"
    elseif id and id:sub(1, 5) == "ITEM:" then
        local name = id:sub(6)
        local c = COURSES[name]
        if c then
            editingCourseName = name
            local seq = {}
            for _, p in ipairs(c.seq) do table.insert(seq, p) end
            courseForm = {nameStr = name, seq = seq}
            courseScroll = 1
            screen = "courseDetail"
        end
    end
end

local function drawCourseDetail()
    local w, h = term.getSize()
    term.setBackgroundColour(colours.black)
    term.clear()
    term.setTextColour(colours.orange)
    term.setCursorPos(2, 1)
    term.write(editingCourseName and ("EDIT COURSE " .. editingCourseName) or "NEW COURSE")
    NDU.mark()

    term.setTextColour(colours.white)
    term.setCursorPos(2, 3); term.write("NAME")
    button(9, 3, 16, 1, "F:NAME", courseForm.nameStr ~= "" and courseForm.nameStr or "--------", true, colours.lime)

    term.setCursorPos(2, 5)
    term.write("SEQUENCE (tap to remove):")

    local seq   = courseForm.seq
    local total = #seq
    -- Items start on row 6. Reserve row h-3 for the scroll controls when the list
    -- overflows; the action buttons sit on h-1 either way.
    local rowsNoScroll   = (h - 3) - 6 + 1     -- rows 6..h-3
    local rowsWithScroll = (h - 4) - 6 + 1     -- rows 6..h-4, scroll row at h-3
    local scrollNeeded   = total > rowsNoScroll
    local rows   = scrollNeeded and rowsWithScroll or rowsNoScroll
    local maxTop = math.max(1, total - rows + 1)
    courseScroll = clamp(courseScroll, 1, maxTop)

    for disp = 0, rows - 1 do
        local i     = courseScroll + disp
        local pname = seq[i]
        if pname then
            local y = 6 + disp
            local p = POINTS[pname]
            local coordStr = p and string.format("(%d,%d)", p.x, p.z) or "(missing)"
            term.setTextColour(colours.white)
            term.setCursorPos(2, y)
            term.write(string.format("%d. %-6s %s", i, pname, coordStr))
            addHitbox(1, y, w, y, "SEQ:" .. i)
        end
    end

    if scrollNeeded then
        local sy = h - 3
        button(2, sy, 5, 1, "CSEQ_UP", "UP", courseScroll > 1,      colours.orange)
        button(8, sy, 5, 1, "CSEQ_DN", "DN", courseScroll < maxTop, colours.orange)
        term.setTextColour(colours.white)
        local info = string.format("%d-%d/%d", courseScroll,
            math.min(total, courseScroll + rows - 1), total)
        term.setCursorPos(14, sy); term.write(info)
    end

    local by = h - 1
    button(2, by, 12, 1, "ADDWP", "+ WAYPOINT", true, colours.lime)
    button(15, by, 7, 1, "SAVE", "SAVE", true, colours.orange)
    button(23, by, 9, 1, "DELETE", "DELETE", editingCourseName ~= nil, colours.orange)
    button(33, by, 9, 1, "CANCEL", "CANCEL", true, colours.brown)
end

local function touchCourseDetail(id)
    if id == "F:NAME" then
        local v = promptKeyboard("COURSE NAME", courseForm.nameStr, "route")
        if v then courseForm.nameStr = v end
    elseif id == "ADDWP" then
        coursePickPage = 1
        screen = "coursePointPicker"
    elseif id == "SAVE" then
        local ok, err
        if editingCourseName then
            ok, err = editCourse(editingCourseName, courseForm.nameStr, courseForm.seq)
        else
            ok, err = addCourse(courseForm.nameStr, courseForm.seq)
        end
        if ok then
            screen = "coursesList"
        else
            alert("ERROR", err)
        end
    elseif id == "DELETE" then
        if confirm("DELETE COURSE", "Delete " .. editingCourseName .. "?") then
            deleteCourse(editingCourseName)
            screen = "coursesList"
        end
    elseif id == "CANCEL" then
        screen = "coursesList"
    elseif id == "CSEQ_UP" then
        courseScroll = math.max(1, courseScroll - 1)
    elseif id == "CSEQ_DN" then
        courseScroll = courseScroll + 1   -- draw clamps to the last page
    elseif id and id:sub(1, 4) == "SEQ:" then
        local idx = tonumber(id:sub(5))
        if idx then table.remove(courseForm.seq, idx) end
    end
end

local function drawCoursePointPicker()
    renderList("ADD WAYPOINT", itemsFromPoints(), coursePickPage, false)
end

local function touchCoursePointPicker(id)
    if id == "PREV" then
        coursePickPage = math.max(1, coursePickPage - 1)
    elseif id == "NEXT" then
        coursePickPage = coursePickPage + 1
    elseif id == "BACK" then
        screen = "courseDetail"
    elseif id and id:sub(1, 5) == "ITEM:" then
        local name = id:sub(6)
        table.insert(courseForm.seq, name)
        courseScroll = #courseForm.seq   -- scroll so the just-added waypoint is visible
        screen = "courseDetail"
    end
end

local drawFns = {
    home = drawHome,
    pickDirect = drawPickDirect,
    routeMenu = RTR.drawRouteMenu,
    pickRoute = drawPickRoute,
    pickDest = RTR.drawPickDest,
    planPreview = RTR.drawPlanPreview,
    pickLand = drawPickLand,
    status = drawStatus,
    dataMenu = drawDataMenu,
    pointsList = drawPointsList,
    pointDetail = drawPointDetail,
    coursesList = drawCoursesList,
    courseDetail = drawCourseDetail,
    coursePointPicker = drawCoursePointPicker,
}

local touchFns = {
    home = touchHome,
    pickDirect = touchPickDirect,
    routeMenu = RTR.touchRouteMenu,
    pickRoute = touchPickRoute,
    pickDest = RTR.touchPickDest,
    planPreview = RTR.touchPlanPreview,
    pickLand = touchPickLand,
    status = touchStatus,
    dataMenu = touchDataMenu,
    pointsList = touchPointsList,
    pointDetail = touchPointDetail,
    coursesList = touchCoursesList,
    courseDetail = touchCourseDetail,
    coursePointPicker = touchCoursePointPicker,
}

-- Redraws on every event, not just touches: navLoop's own sleep() ticks (every
-- TICK_INTERVAL) surface here too since parallel.waitForAny broadcasts every
-- event to both branches, which is what keeps POS/DIST live on screen without
-- a second timer of our own.
local function uiLoop()
    while true do
        clearHitboxes()
        drawFns[screen]()
        local ev = {os.pullEvent()}
        local x, y = pointerXY(ev)
        if x then
            local id = hitTest(x, y)
            if id and touchFns[screen] then
                touchFns[screen](id)
            end
        end
    end
end

-- AS -> FMS override. The altitude selector pings ALTOVERRIDE_PROTOCOL when the pilot
-- taps a panel while a landing has it overshadowed. Mid-descent this PAUSES the descent
-- (ALT-AP takes the vertical, position/heading alignment stays online via broadcastCurrent);
-- after touchdown it is the same as RESUME ALT-AP (hand back and go home). Any other time
-- (not landing, or already paused) it is a stray -- ignore it.
local function overrideLoop()
    while true do
        local sender = rednet.receive(ALTOVERRIDE_PROTOCOL, 1)
        if sender and LND.ga then
            -- Pilot wants the altitude back mid-circuit: hand the vertical channel to ALT-AP
            -- and let the route carry on flying itself. The approach will still re-arm at the
            -- end of the plan; it simply arrives at whatever altitude the pilot chose.
            LND.ga = false
            rednet.broadcast({ cmd = "cancel" }, AUTOLAND_PROTOCOL)
        elseif sender and LND.active then
            local p = LND.phase
            -- Every airborne phase of either machine can be paused. ROLLOUT is deliberately
            -- not one of them: the aircraft is braking on a finite runway, and handing the
            -- vertical channel back mid-rollout would achieve nothing but confusion.
            if p == "ALIGN" or p == "TRANSLATE" or p == "DESCEND" or p == "FLARE"
                    or p == "INTERCEPT" or p == "GLIDE" or p == "MISSED" then
                if not LND.paused then
                    LND.paused = true
                    broadcastVertical()   -- send "release" at once so the UAC/AS react now
                end
            elseif p == "LANDED" then
                cancelNav()
                screen = "home"
            end
        end
    end
end

-- ============================================================================
-- EXTERNAL MONITOR -- HSI, in TWO modes (drawHSI dispatches on LND.active):
--   * COURSE (drawCourseHSI): the classic heading-up VOR/HSI rose with a course-deviation
--     needle for the active leg -- shown while navigating a course / direct-to.
--   * LANDING (drawLandingHSI): a heading-up top-down moving map (aircraft fixed nose-up at
--     centre, the pad/target rotating under it, a heading-error bar on top, numbers on the
--     bottom) -- shown once an autoland/landing engages, for flying it down by hand or
--     watching the autoland.
-- Display-only; reads NAV/LND (filled every tick). Idles if no monitor is attached.
-- ============================================================================
-- HSI display constants, kept on one table (like RTR) so the whole instrument section costs a
-- single main-chunk local instead of six -- the file is close to Lua's 200-local ceiling.
local HSICFG = {
    -- COURSE mode (CDI):
    CDI_FULLSCALE_M = 80,   -- metres of cross-track at full needle deflection
    CDI_ONCOURSE_M  = 10,  -- inside this the needle turns green
    -- TO/FROM: the course arrow is drawn as a compass needle, the tip that points at the
    -- waypoint lit and the other dark. Within this many degrees of abeam the question has no
    -- meaningful answer (the classic VOR ambiguity zone) and both tips go grey rather than
    -- flickering; the number is |cos| of the angle between the course and the bearing, so
    -- 0.09 ~ 5 deg either side of 90.
    TOFROM_DEAD     = 0.09,
    -- LANDING mode (map):
    HDG_BAR_FS_DEG  = 45,   -- heading error (deg) at full-width bar deflection
    HDG_ALIGN_DEG   = 5,    -- within this the bar marker turns green
    -- Runway approach: the glideslope pointer up the right-hand edge. It marks WHERE THE
    -- GLIDEPATH IS (fly toward it), same convention as the CDI needle.
    GS_FULLSCALE_M  = 40,   -- metres off the glidepath at full-scale deflection
    GS_ONPATH_M     = 5,    -- inside this the pointer turns green
    -- Map range steps (metres to the ring edge): the smallest that still frames the target
    -- wins, so the target creeps inward as you close and the scale only jumps occasionally.
    RANGES          = {5, 10, 25, 50, 100, 250, 500, 1000, 2500, 5000},
    DEFAULT_RANGE   = 50,   -- ring scale when there is no target to frame
}

local hsiMon = peripheral.find("monitor")
if hsiMon then
    hsiMonName = peripheral.getName(hsiMon)
    hsiMon.setTextScale(CFG.HSI_TEXT_SCALE)
    pcall(function()
        for name, rgb in pairs(CFG.PALETTE.HSI) do
            hsiMon.setPaletteColour(colours[name], rgb)
        end
    end)
end

local function hFill(x, y, col)                 -- one colour cell (instrument graphics)
    local W, H = hsiMon.getSize()
    x, y = math.floor(x + 0.5), math.floor(y + 0.5)
    if x < 1 or x > W or y < 1 or y > H then return end
    hsiMon.setCursorPos(x, y); hsiMon.setBackgroundColour(col); hsiMon.write(" ")
end

local function hText(x, y, s, fg)
    hsiMon.setCursorPos(math.floor(x + 0.5), math.floor(y + 0.5))
    hsiMon.setTextColour(fg or colours.white); hsiMon.setBackgroundColour(colours.black)
    hsiMon.write(s)
end

local function hLine(x0, y0, x1, y1, col)       -- Bresenham colour-cell line
    x0,y0,x1,y1 = math.floor(x0+0.5),math.floor(y0+0.5),math.floor(x1+0.5),math.floor(y1+0.5)
    local dx, dy = math.abs(x1-x0), -math.abs(y1-y0)
    local sx, sy = (x0<x1) and 1 or -1, (y0<y1) and 1 or -1
    local err = dx + dy
    while true do
        hFill(x0, y0, col)
        if x0 == x1 and y0 == y1 then break end
        local e2 = 2*err
        if e2 >= dy then err = err+dy; x0 = x0+sx end
        if e2 <= dx then err = err+dx; y0 = y0+sy end
    end
end

-- NAV.heading and the atan2(dx,dz) bearings live in a frame where North = 180; this maps
-- any such angle (radians) to a true compass heading (deg, N=0 E=90 S=180 W=270).
local function toCompass(rad) return (180 - math.deg(rad)) % 360 end

-- COURSE mode: the classic heading-up compass rose + course-deviation needle for the
-- active leg. Shown while a course/direct-to is being flown (no landing engaged).
local function drawCourseHSI()
    local W, H = hsiMon.getSize()
    hsiMon.setBackgroundColour(colours.black); hsiMon.clear()

    local heading = NAV.haveHeading and NAV.heading or 0

    -- course + cross-track for the active leg (prev -> current waypoint)
    local hasCourse, dtk, xtk, dist, tgtName = false, heading, 0, 0, nil
    local toFrom = 0        -- cos(bearing to the waypoint - course): >0 = TO, <0 = FROM
    if NAV.active and NAV.points[NAV.index] then
        local tgt = NAV.points[NAV.index]
        tgtName = tgt.name
        local dx, dz = tgt.x - NAV.posX, tgt.z - NAV.posZ
        dist, dtk, hasCourse = math.sqrt(dx*dx + dz*dz), atan2(dx, dz), true
        local prev = NAV.points[NAV.index - 1]
        if prev then
            local lx, lz = tgt.x - prev.x, tgt.z - prev.z
            local ll = math.sqrt(lx*lx + lz*lz)
            if ll > 1e-3 then
                dtk = atan2(lx, lz)
                xtk = (lx/ll)*(NAV.posZ - prev.z) - (lz/ll)*(NAV.posX - prev.x)  -- +=right of course
            end
        end
        -- Curve-following: show the PLANNED PATH as the course, not the straight leg, so the dial
        -- reads what the aircraft actually flies (and on-course when it's on the path). DTK = the
        -- path's heading at our projection; XTK/CDI = signed perpendicular deviation FROM the path
        -- (same +=right-of-course sign as the leg version). DIST stays to the real waypoint.
        if DC.CURVE and NAV.phase == "ENROUTE" and NAV.index >= 2 and NAV.index ~= #NAV.points
                and PLAN.onPath() then
            local cdtk, cxtk = PLAN.course()   -- path heading (DTK) + signed deviation FROM the path
            if cdtk then dtk, xtk = cdtk, cxtk end
        end
        -- TO/FROM -- the same question a VOR's flag answers: is the waypoint ahead of us along
        -- the course, or behind? Measured against whichever course is actually displayed (leg
        -- or planned path), so it flips exactly as the fix goes abeam.
        toFrom = math.cos(atan2(dx, dz) - dtk)
    end

    local midX, midY = math.floor(W/2) + 1, math.floor(H/2) + 1
    local Rr = math.max(4, math.min(midY - 3, math.floor((midX - 3)/1.5)))
    local Rc = math.floor(Rr * 1.5)
    -- screen point for a COMPASS-relative angle (deg, 0 = top = current heading, clockwise),
    -- with the char-cell aspect baked into Rc:Rr so the ring reads round.
    local function sp(deg, f)
        local a = math.rad(deg)
        return midX + f*Rc*math.sin(a), midY - f*Rr*math.cos(a)
    end

    -- NAV.heading/dtk live in the atan2(dx,dz) frame (North = 180); convert to a true
    -- compass (N=0 E=90 S=180 W=270) so the number and the N/E/S/W letters read correctly.
    local chdg = toCompass(heading)

    for a = 0, 357, 2 do local x, y = sp(a, 1.0); hFill(x, y, colours.grey) end     -- ring (bezel)

    local card = {[0]="N", [90]="E", [180]="S", [270]="W"}                          -- heading-up card
    for b = 0, 330, 30 do
        local ang = b - chdg                                                        -- bearing b -> screen
        if card[b] then local x, y = sp(ang, 0.80); hText(x, y, card[b], colours.white)
        else            local x, y = sp(ang, 0.90); hFill(x, y, colours.grey) end
    end

    local cdtk
    if hasCourse then
        cdtk = toCompass(dtk)
        local cAng = cdtk - chdg
        -- The course arrow, drawn as a two-tone compass needle: the tip that points at the
        -- waypoint is LIT, the tip pointing the opposite way is DARK -- so TO/FROM is read at
        -- a glance from the instrument itself, without hunting for a flag. Both tips grey in
        -- the abeam ambiguity zone (see HSICFG.TOFROM_DEAD).
        local amb  = math.abs(toFrom) < HSICFG.TOFROM_DEAD
        local lit  = (toFrom >= 0) and cAng or (cAng + 180)   -- the half the waypoint lies on
        local a1x, a1y = sp(lit + 180, 0.95)
        local a2x, a2y = sp(lit, 0.95)
        hLine(midX, midY, a1x, a1y, amb and colours.grey or colours.blue)   -- dark (away) tip
        hLine(midX, midY, a2x, a2y, amb and colours.grey or colours.cyan)   -- lit (toward) tip
        if not amb then                       -- ...and an arrowhead on the lit tip
            local bx, by = sp(lit - 14, 0.78); hFill(bx, by, colours.cyan)
            bx, by = sp(lit + 14, 0.78);       hFill(bx, by, colours.cyan)
        end
        local pr = math.rad(cAng + 90)                                             -- perpendicular
        for _, d in ipairs({-1, -0.5, 0.5, 1}) do                                   -- deviation dots
            hFill(midX + d*(Rc*0.6)*math.sin(pr), midY - d*(Rr*0.6)*math.cos(pr), colours.grey)
        end
        -- The needle marks WHERE THE COURSE IS (fly toward it): right-of-course puts it
        -- left. If it deflects the wrong way in flight, flip this sign.
        local defl = math.max(-1, math.min(1, -xtk / HSICFG.CDI_FULLSCALE_M))
        local ox, oy = defl*(Rc*0.6)*math.sin(pr), -defl*(Rr*0.6)*math.cos(pr)
        local ncol = (math.abs(xtk) <= HSICFG.CDI_ONCOURSE_M) and colours.lime or colours.yellow
        local n1x, n1y = sp(cAng, 0.5)
        local n2x, n2y = sp(cAng + 180, 0.5)
        hLine(n1x+ox, n1y+oy, n2x+ox, n2y+oy, ncol)                                 -- CDI needle
    end

    hText(midX, midY - Rr - 1, "v", colours.orange)   -- fixed lubber index (top = heading)
    hText(midX, midY, "A", colours.white)             -- aircraft symbol (nose up)

    local phdg = math.floor(chdg + 0.5) % 360
    if phdg == 0 then phdg = 360 end
    
    hText(midX - 3, 1, string.format("HDG %03d", phdg), colours.orange)
    -- Commanded (scheduled) speed vs actual ground speed -- realtime speed-law debugging.
    if NAV.active then
        if NAV.aligning then
            hText(2, 1, "ALIGN  ", colours.yellow)     -- yawing onto the bearing, throttle held at 0
        else
            hText(2, 1, string.format("CMD%3.0f", NAV.cmdSpeed), colours.lime)
        end
        hText(W - 7, 1, string.format("GS%4.0f", NAV.groundSpeed or 0), colours.cyan)
    end
    if hasCourse then
        hText(2, H, string.format("%-6s DTK%03d %s", (tgtName or ""):sub(1,6),
            math.floor(cdtk + 0.5) % 360,
            (math.abs(toFrom) < HSICFG.TOFROM_DEAD) and "--" or (toFrom >= 0 and "TO" or "FR")),
            colours.cyan)
        hText(W - 16, H, string.format("XTK%4.1f%s D%4.0f", math.abs(xtk), xtk >= 0 and "R" or "L", dist), colours.white)
    else
        hText(2, H, "NO ACTIVE LEG", colours.grey)
    end
end

-- LANDING mode: a heading-up top-down moving map, shown once a landing engages.
local function drawLandingHSI()
    local W, H = hsiMon.getSize()
    hsiMon.setBackgroundColour(colours.black); hsiMon.clear()

    local haveHdg = NAV.haveHeading
    local chdg    = haveHdg and toCompass(NAV.heading) or 0

    -- Pick the target and the heading we WANT: the landing pad + its pad heading while
    -- landing, otherwise the active waypoint + the bearing to it.
    local landing = LND.active
    local tgtX, tgtZ, tgtName, tgtGlyph, tgtCol, desired
    if landing then
        tgtX, tgtZ = LND.x, LND.z
        tgtName    = LND.name
        tgtGlyph   = (LND.siteType == "heli") and "H" or "+"
        tgtCol     = colours.orange
        if LND.holdHeading then desired = toCompass(LND.holdHeading) end
    elseif NAV.active and NAV.points[NAV.index] then
        local p = NAV.points[NAV.index]
        tgtX, tgtZ, tgtName = p.x, p.z, p.name
        tgtGlyph, tgtCol    = "o", colours.cyan
        desired = toCompass(atan2(p.x - NAV.posX, p.z - NAV.posZ))
    end

    local haveTgt, dist, brg = (tgtX ~= nil) and NAV.haveFix, 0, 0
    if haveTgt then
        local dx, dz = tgtX - NAV.posX, tgtZ - NAV.posZ
        dist = math.sqrt(dx*dx + dz*dz)
        brg  = toCompass(atan2(dx, dz))
    end

    -- ---- top-down map (HEADING-UP, like the waypoint rose): aircraft fixed at centre and
    -- always nose-up; the card, cardinals and target rotate by -heading so dead ahead is
    -- the top row ----
    local mapTop, mapBot = 2, H - 1
    local cx = math.floor(W/2) + 1
    local cy = math.floor((mapTop + mapBot) / 2 + 0.5)
    local Ry = (mapBot - mapTop) / 2
    local Rx = math.min((W - 1) / 2, Ry * 1.5)     -- char cells are ~1.5:1 -> keep it round

    -- screen point for a HEADING-RELATIVE angle (deg, 0 = top = the nose, clockwise)
    local function sp(deg, f)
        local a = math.rad(deg)
        return cx + f*Rx*math.sin(a), cy - f*Ry*math.cos(a)
    end

    local range = HSICFG.DEFAULT_RANGE
    if haveTgt then
        range = HSICFG.RANGES[#HSICFG.RANGES]
        for _, r in ipairs(HSICFG.RANGES) do if dist <= r then range = r; break end end
    end

    for a = 0, 345, 15 do local x, y = sp(a, 1.0); hFill(x, y, colours.grey) end   -- ring

    local card = {[0]="N", [90]="E", [180]="S", [270]="W"}       -- rotating heading-up card
    for b = 0, 330, 30 do
        local x, y = sp(b - chdg, card[b] and 0.82 or 0.92)
        if card[b] then hText(x, y, card[b], colours.white) else hFill(x, y, colours.grey) end
    end
    hText(cx, cy - Ry, "v", colours.orange)                      -- fixed lubber: the nose points here
    if haveTgt then
        local labelTxt = "R " .. math.floor(range) .. "m"
        hText(W - #labelTxt + 1, mapTop, labelTxt, colours.grey)
    end

    if haveTgt then
        local sx, sy = sp(brg - chdg, math.min(dist / range, 1))
        if landing and LND.rwy then
            -- The STRIP itself, drawn from the threshold along the landing direction, so the
            -- map shows the thing being lined up on instead of a point to hover over. Only as
            -- much of it as the current range step can show (the geometry stays exact; the
            -- length is what gets cut), which also bounds the line to a couple of radii.
            local rl = math.min(LND.rwyLen, range * 1.2)
            local ex, ez = (LND.x + LND.ux*rl) - NAV.posX, (LND.z + LND.uz*rl) - NAV.posZ
            local px, py = sp(toCompass(atan2(ex, ez)) - chdg,
                math.sqrt(ex*ex + ez*ez) / range)
            hLine(sx, sy, px, py, colours.orange)
            if rl >= LND.rwyLen then hFill(px, py, colours.brown) end   -- the far end
            -- ...and the touchdown aim point we are actually flying at.
            ex, ez = LND.aimX - NAV.posX, LND.aimZ - NAV.posZ
            px, py = sp(toCompass(atan2(ex, ez)) - chdg, math.sqrt(ex*ex + ez*ez) / range)
            hFill(px, py, colours.lime)
        elseif landing and desired then                          -- pad's approach axis (landing heading)
            local ax = desired - chdg
            local ex, ey = Rx*0.16*math.sin(math.rad(ax)), -Ry*0.16*math.cos(math.rad(ax))
            hLine(sx - ex, sy - ey, sx + ex, sy + ey, colours.orange)
        end
        hText(sx, sy, tgtGlyph, tgtCol)
    end
    hText(cx, cy, "^", colours.white)                            -- aircraft, always nose-up

    -- ---- glideslope pointer (right-hand edge), runway approach only ----
    -- Marks WHERE THE GLIDEPATH IS -- fly toward it -- the same convention as the CDI needle,
    -- so being high on the approach puts the pointer below the centre tick.
    if landing and LND.rwy then
        local half = math.max(1, Ry - 1)
        for y = mapTop + 1, mapBot do hFill(W, y, colours.grey) end
        hText(W, cy, "-", colours.white)                          -- on-path index
        local gy = cy + clamp((LND.gsDev or 0) / HSICFG.GS_FULLSCALE_M, -1, 1) * half
        hFill(W, gy, (math.abs(LND.gsDev or 0) <= HSICFG.GS_ONPATH_M)
            and colours.lime or colours.orange)
    end

    -- ---- heading-error bar (top row): where the wanted heading sits vs the nose ----
    for x = 1, W do hText(x, 1, "-", colours.grey) end
    hText(cx, 1, "|", colours.white)               -- on-heading centre index
    if haveHdg and desired then
        local err  = ((desired - chdg + 540) % 360) - 180        -- +ve = wanted heading is to the right
        local half = math.max(1, math.floor(W/2) - 1)
        local mx   = cx + math.max(-1, math.min(1, err / HSICFG.HDG_BAR_FS_DEG)) * half
        hFill(mx, 1, (math.abs(err) <= HSICFG.HDG_ALIGN_DEG) and colours.lime or colours.orange)
    end

    -- ---- numbers (bottom row) ----
    local bottom, bcol
    if landing and LND.rwy then
        -- Runway numbers: how far off the glidepath and the centreline, and how much runway
        -- there is still to run to the aim point (negative = past it).
        bcol   = colours.orange
        bottom = string.format("%-8s AGL%d GS%+.0f XT%+.0f R%.0f V%+.1f%s",
            (LND.phase or ""):sub(1, 8), math.floor((LND.agl or 0) + 0.5), LND.gsDev or 0,
            LND.xtk or 0, LND.along or 0, LND.vs or 0, LND.paused and " PSE" or "")
    elseif landing then
        bcol   = colours.orange
        bottom = string.format("%-8s AGL%d VS%+.1f PE%.1f HE%+d%s",
            (LND.phase or ""):sub(1, 8), math.floor((LND.agl or 0) + 0.5), LND.vs or 0,
            LND.posErr or 0, math.floor((LND.hdgErrDeg or 0) + 0.5), LND.paused and " PSE" or "")
    elseif haveTgt then
        bcol   = tgtCol
        bottom = string.format("%-6s D%d BRG%03d HDG%03d", (tgtName or ""):sub(1, 6),
            math.floor(dist + 0.5), math.floor(brg + 0.5) % 360, math.floor(chdg + 0.5) % 360)
    elseif NAV.haveFix then
        bcol, bottom = colours.grey, string.format("HDG%03d  NO TARGET", math.floor(chdg + 0.5) % 360)
    else
        bcol, bottom = colours.grey, "NO FIX"
    end
    hText(1, H, bottom:sub(1, W), bcol)
end

-- The landing map only earns the screen once a landing is actually engaged; the rest of
-- the time (flying a course/direct-to, or idle) it's the familiar CDI rose.
local function drawHSI()
    if LND.active then drawLandingHSI() else drawCourseHSI() end
end

local function hsiLoop()
    if not hsiMon then while true do sleep(1) end end
    while true do pcall(drawHSI); sleep(HSICFG.REDRAW) end
end

local function apAnnounceLoop()
    while true do
        local nav = redstone.getInput(CFG.APSWITCH.SIDE)
        rednet.broadcast({ ['autopilot_nav'] = nav }, CFG.APSWITCH.PROTOCOL)
        sleep(CFG.APSWITCH.INTERVAL)
    end
end

-- MAIN -------------------------------------------------------------------------

if not sublevel then error("No sublevel API - is the Sable mod present?") end

local modem = peripheral.find("modem")
if not modem then error("No modem attached") end
if not rednet.isOpen(peripheral.getName(modem)) then
    rednet.open(peripheral.getName(modem))
end

loadPoints()
loadCourses()
loadLanding()

local nativeTerm = term.current()

if term.isColour and term.isColour() then
    for name, rgb in pairs(CFG.PALETTE.TERM) do term.setPaletteColour(colours[name], rgb) end
end

local ok, err = pcall(function()
    parallel.waitForAny(navLoop, uiLoop, overrideLoop, hsiLoop, NDU.loop, TD.loop, apAnnounceLoop)
end)

term.redirect(nativeTerm)
if not ok then
    term.setTextColour(colours.white)
    print("autopilot error: " .. tostring(err))
end
