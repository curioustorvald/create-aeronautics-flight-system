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
local SPEED_FULL, SPEED_SLOW, NEXT_CRUISE, RESUME_BACKTRACK_WEIGHT, COURSE_MAX_REF
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
            -- ALT_OFFSET (2026-08-29) is on this list for a reason the others are not: a
            -- missing one is a nil-arithmetic crash on the first navTick, and a DEFAULTED one
            -- would be worse than that -- 0 is a legal value (the display bench), so a
            -- silently defaulted 0 restores the old over-reading altimetry on a real hull and
            -- looks exactly like a working aeroplane right up to the flare.
            "ALT_OFFSET",
            "SPEED_FULL", "SPEED_SLOW", "NEXT_CRUISE", "RESUME_BACKTRACK_WEIGHT",
            "COURSE_MAX_REF",
            "DC", "LG", "HSI", "TERR", "TAVOID", "PALETTE", "HSI_TEXT_SCALE",
            -- ATC added 2026-09-28 (the automated controller's client). Read by navTick on
            -- every tick, and ENABLE is a BOOLEAN: a missing one would read false and the
            -- aircraft would quietly land on an occupied pad with no error anywhere.
            "ATC",
            -- DEP added 2026-10-15 (the runway departure). Read by every plan commit, and
            -- ENABLE is a BOOLEAN: a missing one would read false and the aircraft would go
            -- back to rising straight off the strip with nothing to say why.
            "DEP" }) do
        if res[k] == nil then
            error("apconf.lua is missing CFG." .. k .. " -- the file looks incomplete.", 0)
        end
    end
    -- CURVE_LOOKAHEAD_T added 2026-08-20 and read UNCONDITIONALLY on every en-route broadcast,
    -- so an apconf predating it must fail HERE rather than as a nil-arithmetic crash the first
    -- time a carrot is computed -- i.e. airborne, mid-leg.
    for t, ks in pairs({ RAD = {"VOR_RADIUS"}, DC = {"CURVE", "CURVE_LOOKAHEAD_T"},
            -- APPR_JOIN_MAX_DEG added 2026-08-26 and read by APR.prep on EVERY plan that
            -- lands, so an apconf predating it must fail HERE and not as a nil-arithmetic
            -- crash the first time the pilot commits a route to a field with a published
            -- approach -- i.e. airborne, with the plan half built.
            -- RWY_TCH / RWY_AIM_MAX_FRAC added 2026-09-08 and read by LND.rwyGlide on every
            -- runway landing, every go-around circuit and every published approach. A missing
            -- one is a nil-arithmetic crash at the moment the approach engages; a DEFAULTED one
            -- would silently restore the too-low glidepath that made the aircraft flare short
            -- of the threshold, which looks like a working approach until the wheels arrive.
            LG = {"RWY_GS_DEG", "RWY_TCH", "RWY_AIM_MAX_FRAC", "PAD_PROBE_M", "APPR_ENABLE",
                  "APPR_JOIN_MAX_DEG"},
            -- REDRAW / RANGES / PLAN added to this list 2026-09-16, when the program's own
            -- duplicate copy of CFG.HSI was deleted. Until then this table was SHADOWED by a
            -- second `local HSICFG` in the instrument section and nothing here was read at
            -- all -- which is how RANGES came to differ between airframes without anyone
            -- noticing. Naming them here is what stops that recurring silently.
            HSI = {"CDI_FULLSCALE_M", "REDRAW", "RANGES", "PLAN"},
            TOUCHDOWN = {"PROTOCOL"}, NAVDATA = {"PROTOCOL"},
            APSWITCH = {"SIDE"}, PALETTE = {"HSI"}, NEXT_CRUISE = {"full"},
            TERR = {"PROTOCOL", "WARN_M", "CAUTION_M", "CONTEXT_M", "TINT_LO_Y", "TINT_HI_Y",
                    "DITHER", "PACK", "SUBPIXEL", "COL", "DENSITY", "FLAG"},
            -- TAVOID added 2026-09-04 (terrain avoidance). Read unconditionally by navTick /
            -- broadcastVertical / overrideLoop on every tick once the aircraft is airborne, so
            -- an apconf predating it must fail HERE, not as a nil-arithmetic crash the first
            -- time the aircraft leaves the pad.
            TAVOID = {"ENABLE", "CLEAR_M", "RELEASE_MARGIN", "STALE", "PROF_STALE", "REQ_EVERY",
                    "STEP", "WIDTH", "MIN_RANGE", "RANGE_MARGIN", "LOOK_AHEAD_TIME",
                    "MAX_SAMPLES", "MIN_GS", "TRACK_COS_MIN", "BEHIND", "LEG_MARGIN",
                    "MIN_MASS", "OVERRIDE_COOLDOWN", "FLAG"},
            ATC = {"ENABLE", "PROTOCOL", "WIDE", "RUNWAY_ONLY", "CONTACT_M", "INTERVAL",
                   "STALE", "HOLD"},
            DEP = {"ENABLE", "STRIP_HALFW", "STRIP_MARGIN", "GROUND_AGL", "MAX_GS", "VR",
                   "ROLL_ACCEL", "XTK_TOL", "CENTRE_T", "BACKTRACK_WEIGHT", "SPEED",
                   "CLIMB_VS", "CLIMB_AGL", "STRAIGHT_M", "RUNOUT_M"} }) do
        if type(res[t]) ~= "table" then
            error("apconf.lua's CFG." .. t .. " is not a table.", 0)
        end
        for _, k in ipairs(ks) do
            if res[t][k] == nil then
                error("apconf.lua's CFG." .. t .. " is missing " .. k .. ".", 0)
            end
        end
    end
    -- CFG.HSI.PLAN added 2026-09-16 (the route/PLAN view on the instrument). Two levels down,
    -- so it needs its own pass -- and it needs one rather than being left to default, for the
    -- usual reason: ENABLE is a BOOLEAN, and a defaulted false would take the second view off
    -- the instrument with no error anywhere to say why the touch stopped working.
    if type(res.HSI.PLAN) ~= "table" then
        error("apconf.lua's CFG.HSI.PLAN is not a table.", 0)
    end
    for _, k in ipairs({ "ENABLE", "REDRAW", "LOOK_T", "CORNER_K", "END_K", "MIN_RANGE",
            "MAX_RANGE", "HYST", "SUBPIXEL", "MAX_POINTS", "LABELS",
            -- TERRAIN / THIN_LVL / TRAFFIC added 2026-09-17 with the hazard and traffic
            -- layers. Both feature switches are BOOLEANS and so must be named here rather
            -- than left to default: a nil reads false, and the view would then quietly draw
            -- a route over a blank screen -- which on THIS view means "nothing in the way".
            "TERRAIN", "THIN_LVL", "TRAFFIC" }) do
        if res.HSI.PLAN[k] == nil then
            error("apconf.lua's CFG.HSI.PLAN is missing " .. k .. ".", 0)
        end
    end
    -- How this airframe holds when told to. Checked by VALUE, not just presence: "hover" is
    -- a speed-zero stop on the path, which a FIXEDWING cannot fly at all, so a typo here must
    -- not fall through to it -- and an orbit needs its two numbers, which "hover" never reads.
    if res.ATC.HOLD ~= "hover" and res.ATC.HOLD ~= "orbit" then
        error("apconf.lua's CFG.ATC.HOLD is '" .. tostring(res.ATC.HOLD) .. "'. It must be"
            .. " 'hover' (stop on the path -- VTOL/HYBRID) or 'orbit' (circle -- FIXEDWING).", 0)
    end
    if res.ATC.HOLD == "orbit" then
        for _, k in ipairs({ "ORBIT_R", "ORBIT_SPEED" }) do
            if type(res.ATC[k]) ~= "number" or res.ATC[k] <= 0 then
                error("apconf.lua's CFG.ATC.HOLD is 'orbit' but CFG.ATC." .. k
                    .. " is not a positive number.", 0)
            end
        end
    end
    -- The two the departure DIVIDES by, or builds a turn radius from: a zero is not a slow
    -- aeroplane, it is a division by zero at the moment the pilot commits a plan off a runway.
    for _, k in ipairs({ "ROLL_ACCEL", "SPEED" }) do
        if type(res.DEP[k]) ~= "number" or res.DEP[k] <= 0 then
            error("apconf.lua's CFG.DEP." .. k .. " is not a positive number.", 0)
        end
    end
    if type(res.HSI.PLAN.TRAFFIC) ~= "table" then
        error("apconf.lua's CFG.HSI.PLAN.TRAFFIC is not a table.", 0)
    end
    for _, k in ipairs({ "ENABLE", "PROTOCOL", "COAST_AFTER", "DROP_AFTER", "ALT_FILTER_M",
            "PROX_M", "PROX_V", "MAX" }) do
        if res.HSI.PLAN.TRAFFIC[k] == nil then
            error("apconf.lua's CFG.HSI.PLAN.TRAFFIC is missing " .. k .. ".", 0)
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
    COURSE_MAX_REF                         = CFG.COURSE_MAX_REF
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
local function resolveDirVec(query)
    if query == 'x' or query == 'X' then return nil end -- don't care -> nil
    local v = DIR_VECTORS[query:lower()] -- named direction?
    if v then return v end
    if tonumber(query) then
        local radians = math.rad(tonumber(query))
        return {math.sin(radians), -math.cos(radians)}
    end
    return nil
end
local function headingForDir(dir)
    local v = resolveDirVec(dir)
    if not v then return nil end
    return atan2(v[1], v[2])
end


-- DISPLAY / TOUCH SETUP -------------------------------------------------------

local monName = nil -- set once we know whether a monitor is in use
local hsiMonName = nil -- the VOR/HSI instrument monitor; its touches are NOT read as UI input
                       -- (the UI lives on this computer's screen) -- they are the instrument's
                       -- own view toggle, see CRS below and uiLoop.

-- The HSI's VIEW state, and -- further down -- the PLAN view's cached raster. The table is
-- declared HERE, up with the touch plumbing, because uiLoop is the only loop that pulls
-- events and so is the only place the toggle can live; everything else about it (the
-- autozoom, the subpixel raster, the drawing) is defined down in the HSI section next to the
-- code that uses it. Same "table early, methods where their dependencies come into scope"
-- split this file already uses for LND and TAVOID.
local CRS = {
    -- "hsi"  the heading-up rose + course-deviation needle, over the terrain overlay
    -- "plan" the route map: the baked path, its waypoints, and a one-row CDI
    -- LANDING is deliberately NOT one of these. It is forced by LND.active and the pilot
    -- cannot switch out of it -- an instrument the aeroplane is being landed on is the FMS's,
    -- not the pilot's (see drawHSI).
    view = "hsi",
    rng  = nil,        -- the autozoom's currently selected range step, kept across frames so
                       -- the hysteresis has a previous value to be hysteretic about
}

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
local LANDING = {}      -- name -> {type="heli"|"rwy", dir=n/e/s/w/x/rawdeg, alt, rwyLen}
                        -- ...or one of the PATTERN shapes below, which are not sites at all

-- PUBLISHED APPROACH PATTERNS. The whole feature lives on this one table (the functions are
-- down in the APPROACH PATTERNS section, which needs POINTS and the geometry helpers first);
-- declared up here because isLandingSite already has to know which landing.db record shapes
-- are patterns rather than places. One table rather than a dozen top-level locals: the main
-- chunk is close to Lua's 200-local ceiling -- see the note in apconf.lua's DC block.
local APR = {
    -- The record shapes in landing.db that are PATTERNS: a go-around leg, a go-around circuit
    -- and a published approach. None is tied to a point of its own and none may ever be offered
    -- as a destination. Listed by name rather than tested as "anything that is not heli/rwy" so
    -- that a hand-added SITE type still lands, while a fourth PATTERN shape has to be named
    -- here before anything treats it as one -- the safe way round for both mistakes.
    PATTERNS = { goaround = true, ["goaround-wp"] = true, appr = true },
}

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
            local name, ty, dir, alt, rwy = t:match("^(%a+)=(%a+),([%a%.%d]+),(%-?%d+),(%-?%d+)$")
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
            -- ...and a FOURTH: a published APPROACH. Same shape on the wire as the circuit
            -- above -- an id, a minimum altitude, a list of fixes -- and a different meaning:
            -- this is how the aircraft ARRIVES. A field publishes one per direction it can be
            -- arrived from and the FMS picks between them (see the APPROACH PATTERNS section).
            local aid, aalt, aseq =
                t:match("^([%w_]+)=appr,(%-?%d+),(.+)$")
            -- Both list shapes split their tail the same way; an unknown name in one is left in
            -- and skipped later, exactly as a course naming a deleted point is.
            local function seqOf(s)
                local seq = {}
                for p in s:gmatch("[^,]+") do
                    table.insert(seq, (p:match("^%s*(.-)%s*$")))
                end
                return seq
            end
            if gid then
                LANDING[gid] = {
                    type = "goaround",
                    x1 = tonumber(gx1), z1 = tonumber(gz1),
                    x2 = tonumber(gx2), z2 = tonumber(gz2), alt = tonumber(galt),
                }
            elseif wid then
                LANDING[wid] = { type = "goaround-wp", alt = tonumber(walt), seq = seqOf(wseq) }
            elseif aid then
                LANDING[aid] = { type = "appr", alt = tonumber(aalt), seq = seqOf(aseq) }
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

-- A place the autopilot can be told to LAND at. Go-around circuits and published approaches
-- share landing.db but are not one: they have no point of their own, and nothing may offer one
-- as a destination. The test is the PATTERN whitelist rather than a prefix, so adding a fifth
-- record shape to the file cannot let it quietly become a destination by failing to match.
local function isLandingSite(name)
    local l = LANDING[name]
    return l ~= nil and not APR.PATTERNS[l.type]
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

-- Expand one course's waypoint list, letting a token name a PATTERN instead of a point:
--   * another COURSE -- that course's own (already-expanded) sequence is spliced in where the
--     reference stood, so `WAX-TMU=WAX,WAU-TMU` flies "WAX, then whatever WAU-TMU flies"
--     without restating its fixes.
--   * a published APPROACH from landing.db -- `AGS-TMV=AGS,...,TMV_APPR4` is "...then arrive by
--     TMV_APPR4", i.e. a route that names the arrival it is meant to be flown with instead of
--     leaving the FMS to pick one on its own at landing preparation. The approach's fixes are
--     spliced in through APR.expand, so a reference chain inside landing.db (an appr naming
--     another appr, an appr naming its missed approach) resolves exactly as it does when the
--     FMS chooses the same procedure itself, and the expansion ends at the landing site --
--     which is what makes such a course auto-land and what APR.prep then recognises as
--     "already flying one" (APR.established), picking up the published minimum and the missed
--     approach circuit with it.
-- A GO-AROUND id names no route: it is a missed-approach procedure, and splicing one into an
-- arrival would fly a lap of the aerodrome on the way in -- the same reason APR.expand refuses
-- to splice one. It is skipped here like any other unusable name.
-- Cycles and runaway nesting are bounded rather than trusted -- a name already on the
-- reference stack is skipped, and COURSE_MAX_REF caps the depth -- same shape and reasoning as
-- APR.expand further down, for landing.db's pattern-to-pattern references.
-- A reference is written where the route reaches its join fix, and the thing referenced very
-- often STARTS at that same fix (`TMU,TAIMU,PDRNT,TMV_APPR4` where the approach's own entry is
-- PDRNT). Two identical fixes in a row are a zero-length leg, which the corner solver has no
-- meaningful answer for, so the seam is collapsed as it is written. Only CONSECUTIVE repeats
-- go: a procedure that visits one fix twice with something in between is a procedure turn and
-- entirely deliberate (TMV_APPR1 does exactly that).
local function pushFix(out, nm)
    if out[#out] ~= nm then table.insert(out, nm) end
end

local function expandCourseSeq(seq, seen, depth)
    seen, depth = seen or {}, depth or 0
    local out = {}
    for _, nm in ipairs(seq) do
        local land = LANDING[nm]
        if POINTS[nm] then
            pushFix(out, nm)                          -- an ordinary fix
        elseif COURSES[nm] then
            if not seen[nm] and depth < COURSE_MAX_REF then
                seen[nm] = true
                local sub = expandCourseSeq(COURSES[nm].seq, seen, depth + 1)
                seen[nm] = nil
                for _, s in ipairs(sub) do pushFix(out, s) end
            end
        elseif land and land.type == "appr" then
            -- landing.db's own reference stack, not this one: the two namespaces are separate
            -- (a course cannot be a fix of an approach), so APR.expand gets a fresh stack
            -- seeded with this id and bounded by its own APPR_MAX_REF, exactly as APR.forSite
            -- calls it.
            for _, s in ipairs(APR.expand(land, {[nm] = true}, 1)) do pushFix(out, s) end
        end
        -- Anything else is a name neither points.db, course.db nor an approach in landing.db
        -- has -- skipped, exactly what setRoute already did with a course naming a point that
        -- has since been deleted.
    end
    return out
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
    -- The published approach chosen for that landing, decided once with the rest of the plan
    -- (APR.prep) and snapshotted here for the same reason landSite is. All four are nil when
    -- the destination publishes no approach, or when the plan already arrives straight in.
    apprName = nil,     -- the landing.db id of the pattern being flown, for the pilot
    apprFrom = nil,     -- the NAV.points index its first fix was spliced in at
    apprAlt  = nil,     -- its published MINIMUM altitude, held until the glidepath sinks below
    apprGa   = nil,     -- the go-around id the pattern named as its missed approach, if any --
                        -- kept across a go-around so a second attempt flies the same circuit
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
    phase  = "OFF",     -- pad:    OFF | ALIGN | TRANSLATE | DESCEND | FLARE | PROBE
                        --          | HOVER | LANDED
                        -- runway: OFF | INTERCEPT | GLIDE | FLARE | ROLLOUT | MISSED | LANDED
                        -- PROBE/HOVER are the "is there actually a pad here?" pair: see
                        -- LG.PAD_PROBE_M and LND.padDown.
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
    ga    = false,      -- a published PATTERN is being flown -- a go-around circuit, or (with
                        -- .appr below) an approach. LND.active is FALSE: the ROUTE machinery has
                        -- the horizontal, and all this flag means is that the FMS still owns the
                        -- vertical channel (holding LND.gaAlt). Both patterns need exactly that,
                        -- which is why one flag carries both rather than the vertical channel
                        -- being taught the difference.
    gaWp  = false,      -- ...and it is the PUBLISHED-WAYPOINT kind, not the straight-leg kind.
                        -- The horizontal is the same either way (a plan is a plan); this picks
                        -- the vertical profile, which is not (see LND.gaTick).
    gaAlt = 0,          -- ...that altitude, from the route's landing.db entry -- a minimum for
                        -- a waypoint circuit, the height to fly for a straight-leg one
    gaExitAlt = 0,      -- ...and the glidepath height to be at where the approach is rejoined
                        -- (straight-leg only: a waypoint circuit works this out as it goes)
    gaCount = 0,        -- attempts used on THIS landing; reset only when a new plan is set
    -- ---- published approach (LG.APPR_*; see APR.arm) ----
    appr  = false,      -- the pattern under .ga is an APPROACH, not a missed approach. Same
                        -- vertical profile, a different speed cap and a different word on the
                        -- screen -- and it must not touch .gaCount, which bounds the RETRIES.
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

-- TERRAIN AVOIDANCE state -- see apconf.lua's CFG.TAVOID header for what this is and why it
-- lives in the FMS rather than on the GPWS. Declared here (early, beside LND) so
-- broadcastVertical, navTick and overrideLoop below can read and set it; its METHODS are
-- defined later, next to the TERR module they depend on (TERR.geo / TERR.srv) -- the same
-- "table early, methods where their dependencies come into scope" split this file already
-- uses for LND (compare LND.gaTick, defined hundreds of lines after this table is).
local TAVOID = {
    prof = nil,        -- cached corridor profile: { x, z, dx, dz, step, n, d, t }
    reqAt = 0, reqId = 0, asked = 0, served = 0,
    active = false,    -- currently holding an automatic terrain-avoidance altitude
    holdAlt = 0,       -- the altitude being held while active -- only ever ratchets UP; see
                        -- apconf.lua's CFG.TAVOID header for why a release is the only way down
    overrideUntil = 0, -- os.clock() until which a pilot's ALT-AP override suppresses re-engagement
}

-- AUTOMATED ATC CLIENT state -- see apconf.lua's CFG.ATC header and the methods in the ATC
-- section below navTick's helpers. Declared here, early, because dynamicSpeedTarget,
-- broadcastCurrent and the plan-commit functions all read or reset it, and they come long
-- before the section that needs setDirectTo in scope. Same split as TAVOID above.
local ATC = {
    st = nil,          -- nil (not in contact) | "ASK" | "CLR" | "HOLD" | "UNCTL" | "REL"
    fac = nil,         -- the facility this aircraft is asking about
    reply = nil,       -- the controller's last answer, as received -- its .fac says what about
    heard = nil,       -- os.clock() of that answer
    srv = nil,         -- the controller's computer id, once heard from
    holding = false,   -- a HOLD is in force: no forward speed, and the landing may not engage
    holdAt = 0,        -- os.clock() the hold began
    ocx = nil, ocz = nil,  -- the orbit's centre, when this airframe holds by orbiting
    released = false,  -- the pilot has taken this plan off ATC's hands; cleared by a new plan
    note = nil,        -- what ATC last did to the plan ("TMU>TMA"), for the pilot
}

-- RUNWAY DEPARTURE state -- see apconf.lua's CFG.DEP header and the RUNWAY DEPARTURE section
-- beside the plan-commit functions, where the methods live. Declared here, early, for the
-- reason ATC is: broadcastCurrent, broadcastVertical, dynamicSpeedTarget, cancelNav, navTick
-- and the overrideLoop all read it, and they come long before the section that needs GA, PLAN
-- and bestResumeIndex in scope.
local DEP = {
    phase = "OFF",      -- OFF | READY | ALIGN | LINEUP | ROLL | CLIMB -- see DEP.tick
    -- The runway being departed, SNAPSHOTTED when the plan is committed (invariant I6, as
    -- NAV.landSite is), so a nav-data refresh mid-roll cannot move the strip under the wheels.
    name = "", thrX = 0, thrZ = 0, ux = 0, uz = 0, hdg = 0, len = 0, alt = 0,
    luS = 0, luX = 0, luZ = 0,  -- the line-up point: metres past the threshold, and where
    rotS = 0,           -- the furthest the roll may run before it rotates, whatever its speed
    back = 0,           -- metres of runway backtracked to reach the line-up point
    holdX = 0, holdZ = 0,   -- where ALIGN stands still while the nose comes round
    tgtX = 0, tgtZ = 0,     -- the point being broadcast to the UAC
    cmd = 0,            -- the roll's commanded speed, ramped at CFG.DEP.ROLL_ACCEL
    s = 0, xtk = 0, xtkRate = 0, agl = 0, hdgErr = 0,   -- live, in RUNWAY coordinates
    centreAt = nil,     -- os.clock() LINEUP was lined up and short only of the centreline
    GROUND = { READY = true, ALIGN = true, LINEUP = true, ROLL = true },  -- owns navap
    OWNS   = { ALIGN = true, LINEUP = true, ROLL = true, CLIMB = true },  -- owns the vertical
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

-- True only on the FINAL leg -- the actual approach to the destination. It once also covered
-- the PENULTIMATE leg, but that made a fly-THROUGH second-to-last waypoint act like an arrival
-- (dropping curve-following and, if the scheduled-speed flag lapsed, capping at the UAC's
-- APPROACH_SPEED of 30). A mid-route fix should stay ENROUTE (curve + cruise); only the very
-- last leg approaches the destination, and the curvature speed profile already brakes for the
-- endpoint on its own (endpoint = FINAL_ARRIVE_SPEED) whether the phase says APPROACH or not.
-- Shared by radiusFor and phaseFor so the two stay in step.
--
-- A SINGLE-POINT DIRECT-TO IS A FINAL LEG (2026-09-02, `total >= 1`, was `>= 2`). It used to be
-- excluded -- "it has no penultimate leg to speak of" -- from back when this covered the
-- penultimate leg and being an approach leg meant crawling. Neither is true any more, and the
-- exclusion was the whole of "DIRECT TO does not slow down, flies past the point, turns back
-- and circles it":
--   * A direct-to is the ONE plan shape that never reached APPROACH. It went ENROUTE straight
--     to TERMINAL at RAD.POI_FINAL_RADIUS, so it was flown by the UAC's ENROUTE law -- the
--     scalar CLOSING-RATE loop, which sees only the velocity component TOWARD the target.
--     Overshoot it and `closing` goes NEGATIVE while the ground speed stays high, so the error
--     (cap - closing) grows and the loop answers with FULL FORWARD THRUST to chase the fix it
--     has just passed, while desiredHeading (= bearing) yaws the nose back round onto it. That
--     is an orbit, and it is drawn exactly as such in uac.lua's own note beside `approachVec`:
--     APPROACH exists to fly the VELOCITY VECTOR instead, which nulls the tangential component
--     and therefore brakes an overshoot rather than pursuing it.
--   * It is also not a leg. `navPrev` in the UAC is set to whatever the PREVIOUS target was
--     (a new plan is broadcast without a cancel), so a direct-to arrived carrying a bogus
--     leg from the old plan's last fix and ran a cross-track hold against it. The
--     velocity-vector law ignores `hasLeg` entirely, so that goes with it.
-- APPROACH does NOT mean slow: with schedSpeed set the UAC tracks the FMS's cruiseCap exactly
-- (APPROACH_SPEED is only its no-schedule fallback), and nothing in dynamicSpeedTarget reads
-- the phase, so a long direct-to still cruises the whole way and brakes on the arrival sweep.
-- The one other reader is radiusFor: a VOR flown direct-to now gets POI_ENROUTE_RADIUS rather
-- than the wider fly-by VOR_RADIUS, which is what that function's own comment already argues
-- for -- a direct-to a VOR is an arrival AT it, not an overflight.
local function isApproachLeg()
    local total = #NAV.points
    return total >= 1 and NAV.index >= total
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
--   APPROACH - the final leg, while still outside the final point's own pass
--              radius: closing on the destination, but not close yet. NOT a
--              slow phase -- the UAC flies it at the scheduled speed; what it
--              buys is velocity-vector control instead of a closing-rate loop.
--   TERMINAL - inside the final point's pass radius: navcom switches from
--              along-track-plus-line-hold to direct position-hold
--   ARRIVED  - inside RAD.ARRIVED_FRACTION of that same radius
-- A single-point direct-to is ALL final leg, so it is APPROACH from the moment
-- it is committed until it reaches that radius -- see isApproachLeg for why it
-- must be, and why that costs it nothing in cruise speed.
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

-- NO POINT LIST MAY CONTAIN A REVERSAL. This is a GUARANTEE the path planner depends on, not a
-- preference -- 2026-09-17, after CC's watchdog killed the FMS in flight.
--
-- A fly-by transition is a circle tangent to both legs. At 180 degrees the two legs are
-- parallel and NO such circle exists at any finite distance: `T = k + (R+p)*tan(Delta/2)` comes
-- back at ~1.1e18 m whatever the radius (tan(pi/2) is 1.63e16 in doubles -- not an infinity,
-- not an error, just a number), PLAN.marchStraightInto is then asked for 2.2e17 samples, and
-- the watchdog takes the WHOLE PROGRAM with it -- which in the air also stops navap and stales
-- the UAC's link. See _docs/02 section 4.
--
-- So the apex is dropped, here, before PLAN.build is ever called. It is safe in a way dropping
-- a fix generally is not: the three fixes are within REVERSAL_DEG of collinear, so the direct
-- leg between the apex's neighbours lies within leg*sin(5 deg) of the track the out-and-back
-- already flew -- 64 m on the longest leg this database contains, well inside DC.PASS_CORRIDOR.
-- Nothing new is overflown; only the pointless run out and back is removed.
--
-- IT RUNS ON THE FINAL LIST, NOT ON THE SOLVED ROUTE, and that distinction is the whole reason
-- this is not in RTR: the auto-router is only one of the sources. APR.prep splices a published
-- approach in AFTER the route is solved, and a procedure that enters from beyond the field is a
-- reversal of its own -- NUN_APPR1 enters at BLTAG, 1 465 m north of NUN, so a plan arriving
-- from the south turns exactly 180 degrees onto it. course.db routes and goaround-wp circuits
-- are two more sources. One pass over the assembled list catches all of them; a pass over the
-- solved route caught one case in three.
--
-- The cost is real and is the lesser evil: dropping a PROCEDURE fix means not flying the
-- published arrival as printed. The alternative is the FMS dying on the approach. The proper
-- answer is a procedure turn (a teardrop, which is what a chart means by that shape) -- that is
-- planner work, and it is in CLAUDE.TODO.md.
-- REVERSAL_DEG is a geometric degeneracy, the same kind of thing as PLAN.solveCorner's own
-- `Delta < 1e-4`, and not an airframe calibration -- which is why it lives here and not in
-- apconf.lua. The database has a clean gap either side of it: the largest turn left alone is
-- 172.41 deg, the smallest collapsed is 175.56 deg.
local REVERSAL_DEG = 175
local function collapseReversals(pts)
    -- Bounded by the ORIGINAL length, evaluated once: each pass removes exactly one fix, so
    -- this cannot run away -- and it must not be `while n < #pts`, which compares a growing
    -- counter against a shrinking list and would stop with a reversal still in it.
    for _ = 1, #pts do
        local cut
        for i = 2, #pts - 1 do
            local a, b, c = pts[i - 1], pts[i], pts[i + 1]
            local turn = math.abs(wrapPi(legBearing(b.x, b.z, c.x, c.z)
                                       - legBearing(a.x, a.z, b.x, b.z)))
            if math.deg(turn) >= REVERSAL_DEG then
                cut = i
                break
            end
        end
        if not cut then break end
        table.remove(pts, cut)
        -- Keep the spliced approach's own index pointing at the same PATTERN, not the same
        -- slot: a cut BEFORE it shifts it down, and a cut AT it (the apex was the pattern's
        -- entry fix, which is exactly the NUN_APPR1 case) leaves it pointing at what follows.
        if NAV.apprFrom and cut < NAV.apprFrom then NAV.apprFrom = NAV.apprFrom - 1 end
    end
    -- ...and if that ate the WHOLE pattern, say so rather than flying a procedure by name only.
    -- NUN_APPR1 is a procedure turn -- north past the field to BLTAG, then back south through
    -- BORTA -- so both of its fixes are reversals and both go. What is left is a direct arrival,
    -- and it must not carry the published minimum and the missed-approach circuit of a
    -- procedure that is no longer in the plan: APR.due would arm at the destination itself.
    if NAV.apprFrom and NAV.apprFrom > #pts - 1 then
        NAV.apprName, NAV.apprFrom, NAV.apprAlt, NAV.apprGa = nil, nil, nil, nil
    end
    return pts
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

-- An arc's sweep, in [0, 2*pi): `a % (2*pi)` -- except that a turn of nothing must come out as
-- nothing. A point dead ahead (or a leg already on our heading) leaves `a` at 0 give or take
-- the last bit of a double, and -1e-16 % 2*pi is 2*pi: a full circle, laid into the plan
-- where a straight was meant. Found 2026-10-15 on the TMV departure, whose climb-out runs
-- straight at TAIMW along x = 609 -- every candidate came back as a 360 deg loop. No shortest
-- path ever wants a sweep within a micro-radian of a full turn, so that IS the rounding.
function GA.sweep(a)
    local d = a % (2 * math.pi)
    if d > 2 * math.pi - 1e-6 then return 0 end
    return d
end

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
                local d1 = GA.sweep(s1 * (hs - h0))
                local d2 = GA.sweep(s2 * (h1 - hs))
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
    local v    = resolveDirVec(site.dir)
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
    LND.gaExitAlt = site.alt + dApp * LND.gsTanFor(site)   -- the slope this runway is flown at

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
    collapseReversals(pts)    -- ...and so must this; a published circuit can reverse
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
    -- THE ROUTE-START ALIGN HOLD COMES FIRST, ahead of every cap and every early return below
    -- (2026-09-01). It used to sit three returns further down, past `not DC.DYNAMIC_CRUISE`,
    -- which handed back the flat pilot cruise cap instead -- so on an airframe with the
    -- dynamic profile switched off the hold silently did not exist at all. That is also the
    -- one configuration where the UAC has nothing to track it with (schedSpeed rides on the
    -- same flag, and without it the UAC draws its own decelerate-at-every-fix law), so
    -- NOTHING anywhere would have held the throttle while the nose swung. Every airframe has
    -- DYNAMIC_CRUISE true today, which is the only reason it never bit. An empty plan
    -- (`total == 0`, below) is likewise not a reason to release the hold.
    if NAV.aligning then return 0 end          -- route start: yaw onto the bearing before throttling
    -- An ATC HOLD comes next, for the same reason: it is a stop, not a cap on some other speed.
    -- Hovering airframes are handed zero (the UAC brakes to a standstill on the path, exactly
    -- as for the ALIGN hold -- see ATC.beginHold); an orbiting one is flown round its circle at
    -- the orbit speed, which is ATC's number rather than the pilot's cruise selection -- a
    -- COAST left selected must not stall a FIXEDWING in the pattern.
    if ATC.holding then return ATC.ocx and CFG.ATC.ORBIT_SPEED or 0 end
    local capBase = cruiseCapValue()
    -- A go-around circuit is flown at approach speed, not at cruise. Both shapes, for their own
    -- reasons: the straight-leg one is BUILT at a turn radius computed from RWY_GA_SPEED, so
    -- flying it faster would not fit the path at all; and either one ends by descending onto
    -- the glidepath and handing straight back to the approach, which the airframe's descent
    -- authority caps at about this speed anyway (RWY_VS_MAX * RWY_GS_MARGIN / tan(GS)). A
    -- published waypoint circuit is kilometres long and this makes it a slow few minutes --
    -- which is what a go-around is.
    -- ...and a published APPROACH pattern likewise, at its own cap: it is a different procedure
    -- flown for a different reason, but sized by the same argument (see LG.APPR_SPEED).
    if LND.appr then capBase = math.min(capBase, LG.APPR_SPEED)
    elseif LND.ga then capBase = math.min(capBase, LG.RWY_GA_SPEED) end
    -- ...and a departure's climb-out at its own, for the go-around's first reason: the turn off
    -- the runway was BUILT at the radius CFG.DEP.SPEED flies (DEP.candidate), so it is flown at
    -- that until the FMS hands the vertical back. The baked profile brakes for the turn anyway;
    -- this keeps the climb-out from accelerating away at cruise in the few seconds before it.
    if DEP.phase == "CLIMB" then capBase = math.min(capBase, CFG.DEP.SPEED) end
    if not DC.DYNAMIC_CRUISE or capBase <= 0 then return capBase end
    local total = #NAV.points
    if total == 0 then return capBase end
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
    if DEP.ground() then
        -- RUNWAY DEPARTURE, on the ground: the rollout's message run backwards (see the RUNWAY
        -- DEPARTURE section). LANDING + holdHeading is what makes a VTOL/HYBRID hold the runway
        -- heading and STRAFE onto the target, and a FIXEDWING hold it and roll straight; the
        -- target is a fixed point to taxi to until the ROLL, and from then a carrot on the
        -- centreline with a commanded speed that rises. type "dep", NOT "rwy"/"land": the GPWS
        -- reads those two as "on final" (gpws.lua's onFinal), which a take-off is not.
        if DEP.phase == "READY" then
            -- Armed, waiting for AP-NAV: name nothing to fly. Not merely ignored -- a UAC still
            -- latched "landed" from the landing before counts itself as navigating (autoland
            -- active), and would taxi to whatever it was sent.
            rednet.broadcast({ cancel = true }, APNAV_PROTOCOL)
            return
        end
        NAV.aimX, NAV.aimZ = DEP.tgtX, DEP.tgtZ
        rednet.broadcast({
            x = DEP.tgtX, y = DEP.tgtZ, type = "dep", name = DEP.name,
            phase = "LANDING", holdHeading = DEP.hdg,
            cruiseCap = DEP.cmd, schedSpeed = true,
            carrot = (DEP.phase == "ROLL") or nil,
        }, APNAV_PROTOCOL)
        return
    end
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
            -- `carrot` and `glide` are NOT the same flag, and the rollout is where that stops
            -- being an academic distinction. In the UAC, `glide` means "this is still a FLYING
            -- leg, you may bank to steer"; `carrot` is what gates the branch that TRACKS
            -- cruiseCap instead of drawing its own decelerate-to-zero position hold. Airborne
            -- they are both true and it never mattered. On the rollout the aircraft is on its
            -- wheels and must not bank -- but the brake is a schedule the FMS owns (see
            -- LND.rwyTick), so the speed must still be tracked, or the UAC substitutes
            -- clamp(APPROACH_KP * dist, 0, TERMINAL_MAX_SPEED) for it. That substitution is
            -- what the touchdown jolt WAS: on Aer Togekiss the commanded speed fell from the
            -- 28.7 m/s the wheels arrived at to a flat 10.0 in a single 0.05 s tick -- a step,
            -- not a deceleration -- and the velocity-vector law answered it with full reverse
            -- thrust. Sent apart, the rollout brake is the FMS's own ramp and the UAC flies it.
            rednet.broadcast({
                x = LND.tgtX, y = LND.tgtZ, type = "rwy", name = LND.name,
                phase = LND.navPhase, holdHeading = LND.navHold,
                cruiseCap = LND.cmdSpeed, schedSpeed = true,
                carrot = (LND.glide or LND.phase == "ROLLOUT") or nil,
                glide = LND.glide or nil,
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
    local phase = NAV.phase or "ENROUTE"
    if ATC.holding and ATC.ocx then
        -- ATC HOLD by ORBITING (CFG.ATC.HOLD = "orbit"): pure-pursue a carrot running round the
        -- circle ahead of the aircraft, clockwise seen from above -- right-hand turns, the way a
        -- holding pattern is flown. Sent as ENROUTE whatever phase the plan is in, because
        -- TERMINAL/ARRIVED would make the UAC draw a position hold on the point, which is the
        -- one thing an airframe that holds by orbiting cannot fly. Same lookahead law as the
        -- en-route carrot below, as an angle `lead` round the circle.
        -- The carrot rides a circle of R / cos(lead), not R: pure pursuit of a point leading
        -- by `lead` settles where the velocity is square to the radius, i.e. at the carrot's
        -- radius times cos(lead) -- on R itself that was 237 m for a 300 m orbit, measured in a
        -- kinematic run. Lead capped at 1 rad so the carrot never sits more than ~2R out.
        local R = CFG.ATC.ORBIT_R
        local phi = atan2(NAV.posX - ATC.ocx, NAV.posZ - ATC.ocz)
        local lead = math.min(1.0,
            math.max(DC.CURVE_LOOKAHEAD, DC.CURVE_LOOKAHEAD_T * (NAV.cmdSpeed or 0)) / R)
        local Rc = R / math.cos(lead)
        tx, tz = ATC.ocx + Rc * math.sin(phi - lead), ATC.ocz + Rc * math.cos(phi - lead)
        ptype, carrot, phase = "carrot", true, "ENROUTE"
    elseif DC.CURVE and NAV.phase == "ENROUTE" and not isFinal and PLAN.onPath() then
        -- LOOKAHEAD IS A TIME, FLOORED BY A DISTANCE (see DC.CURVE_LOOKAHEAD_T). Pure pursuit's
        -- guidance gain is V/L, so a FIXED carrot distance means the gain rises with speed and
        -- the loop goes unstable at the top of the speed range while looking fine at the bottom.
        -- Off NAV.cmdSpeed rather than the measured ground speed: it is already rise-rate
        -- limited, so the carrot cannot jitter, and it pulls IN as the profile slows for a
        -- corner -- which tightens tracking exactly where the corner needs it.
        tx, tz = PLAN.carrot(math.max(DC.CURVE_LOOKAHEAD,
            DC.CURVE_LOOKAHEAD_T * (NAV.cmdSpeed or 0)))
        ptype, carrot = "carrot", true
    else
        tx, tz = tgt.x, tgt.z
        ptype = isFinal and "end" or tgt.kind
    end
    NAV.aimX, NAV.aimZ = tx, tz    -- the point we're actually steering to; read by the align gate + HSI
    rednet.broadcast({
        x = tx, y = tz, type = ptype, name = tgt.name,
        phase = phase,
        cruiseCap = NAV.cmdSpeed,  -- the rise-rate-limited commanded along-track speed (set in navTick)
        schedSpeed = DC.DYNAMIC_CRUISE,       -- tell the UAC this cap is a speed to TRACK, not just a
                                           -- ceiling, so it drops its own decelerate-at-every-fix law
        carrot = carrot,           -- UAC: pure-pursue this moving point (velocity-vector, no cross-track)
        -- On a go-around the vertical channel is in a commanded CLIMB, which otherwise makes
        -- the UAC treat this as a pad manoeuvre and fly wings-level -- leaving a FIXEDWING
        -- with no lateral control at all. Same flag, same reason, as the glideslope.
        -- A departure's climb-out is the same case: the climb is on the autoland channel until
        -- CLIMB_AGL, and the turn off the runway falls inside that for a FIXEDWING.
        glide = (LND.ga or DEP.phase == "CLIMB") or nil,
        -- ROUTE-START ALIGN HOLD (2026-09-04). cruiseCap is already forced to 0 while
        -- NAV.aligning (dynamicSpeedTarget), but that alone only zeroes the UAC's ENROUTE
        -- along-track term -- the velocity component TOWARD the target. It says nothing about
        -- any OTHER component the aircraft carried into the align (inherited momentum from
        -- whatever it was doing a moment ago, most commonly cancelling and re-committing to a
        -- new destination mid-flight): that is flown as a cross-track POSITION hold against
        -- navPrev, which during a fresh commit is the UAC's own bogus leg from the old target
        -- to the new one (see isApproachLeg's note), not a velocity brake. This flag tells the
        -- UAC a hold is in effect so it can route translation through the velocity-vector law
        -- instead -- the same law APPROACH/TERMINAL/the carrot use, which nulls the FULL
        -- velocity vector, not just its along-track component. See uac.lua's `alignStop`.
        -- An ATC HOLD by hovering is the same request for the same reason -- cruiseCap is
        -- pinned at 0 (dynamicSpeedTarget) and the whole velocity vector must be braked, not
        -- only its along-track part -- so it rides the same flag. It even works from TERMINAL
        -- and ARRIVED, where the UAC would otherwise creep on towards the point it is holding
        -- short of (APPROACH_KP * dist): with the flag set it tracks the commanded zero.
        align = (NAV.aligning or (ATC.holding and not ATC.ocx)) or nil,
    }, APNAV_PROTOCOL)
end

-- Vertical command to the Stabiliser+Altitude control during a landing. Streamed every
-- tick (not just on the 1 s ping) so the flare is smooth as LND.vs eases with altitude.
local function broadcastVertical()
    -- A RUNWAY DEPARTURE holds the vertical from the moment it starts lining up: on the ground
    -- it is `landed` (hold down on the gear, lift cut -- what the rollout sends, for the same
    -- reason) and in the climb-out it is a commanded climb, until CLIMB_AGL hands it to ALT-AP.
    -- READY owns nothing. `dep` tells a receiver this is a take-off and not a landing; nothing
    -- reads it yet (01 §9: extra fields are free).
    if DEP.phase ~= "OFF" then
        if DEP.phase == "CLIMB" then
            rednet.broadcast({ cmd = "descend", vs = CFG.DEP.CLIMB_VS, dep = true },
                AUTOLAND_PROTOCOL)
        elseif DEP.owns() then
            rednet.broadcast({ cmd = "landed", squat = (TD.fresh() and TD.signal) or nil,
                dep = true }, AUTOLAND_PROTOCOL)
        end
        return
    end
    -- A go-around keeps the vertical channel even though LND.active is false -- the route
    -- machinery is flying the circuit, but its altitude is ours to hold, not ALT-AP's.
    if LND.ga then
        rednet.broadcast({ cmd = "descend", vs = LND.vs }, AUTOLAND_PROTOCOL)
        return
    end
    if not LND.active then
        -- Automatic terrain avoidance (apconf.lua's CFG.TAVOID): the same "hold THIS altitude
        -- outright" command the HOVER phase uses below, streamed every tick like any other
        -- landing command so it cannot ratchet on a repeat. Silent when not engaged -- exactly
        -- the behaviour this branch always had before TAVOID existed.
        if TAVOID.active then
            rednet.broadcast({ cmd = "hold", alt = TAVOID.holdAlt }, AUTOLAND_PROTOCOL)
        end
        return
    end
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
    elseif p == "HOVER" then
        -- Station-keeping on a pad that turned out to be a point in open air. A NAMED altitude,
        -- not the plain "hold" below: plain hold means "hold wherever we are now", which would
        -- freeze the metre of slop the descent stopped at and then let it drift; the published
        -- site altitude is the number the aircraft is actually meant to be on, and holding it
        -- explicitly is the tightest the vertical channel gets. Streamed every tick like any
        -- other landing command -- it is a constant, so re-sending it cannot ratchet.
        rednet.broadcast({ cmd = "hold", alt = LND.siteAlt }, AUTOLAND_PROTOCOL)
    elseif LND.vertHold or p == "ALIGN" or p == "TRANSLATE" then
        -- Hold altitude and take ALT-AP offline while we line up.
        rednet.broadcast({ cmd = "hold" }, AUTOLAND_PROTOCOL)
    elseif p == "DESCEND" or p == "FLARE" or p == "PROBE"
            or p == "GLIDE" or p == "INTERCEPT" then
        -- INTERCEPT belongs here too: a runway approach descends toward the glidepath while it
        -- lines up, and only levels off (via vertHold, above) at the approach minimum. PROBE is
        -- the pad descent continued below the published altitude, so it is an ordinary
        -- commanded descent as far as the vertical channel is concerned.
        rednet.broadcast({ cmd = "descend", vs = LND.vs }, AUTOLAND_PROTOCOL)
    end
end

-- Descent-rate schedule: quick well above the pad, easing linearly to LG.DESCEND_SLOW as
-- the flare height is reached. Returns a rate <= 0.
-- It no longer eases to ZERO at touchdown height, because touchdown height is not a surface --
-- it is only where landing.db SAYS the pad is. Below it the schedule holds DESCEND_SLOW so the
-- craft keeps pressing down through LG.PAD_PROBE_M looking for something solid; landingTick is
-- what ends that probe, on a real touchdown or by giving up into HOVER. (Continuous across
-- TOUCHDOWN_AGL: the ramp already reaches exactly -DESCEND_SLOW there.)
local function descentRateFor(agl)
    if agl <= LG.TOUCHDOWN_AGL then return -LG.DESCEND_SLOW end
    if agl >= LG.FLARE_AGL then return -LG.DESCEND_FAST end
    local f = (agl - LG.TOUCHDOWN_AGL) / (LG.FLARE_AGL - LG.TOUCHDOWN_AGL)   -- 0 at pad .. 1 at flare top
    return -(LG.DESCEND_SLOW + (LG.DESCEND_FAST - LG.DESCEND_SLOW) * f)
end

-- The three touchdown tests, factored out so both landing machines ask the same question:
--   1. the squat switch -- the gear is loaded, so we are down at whatever height landing.db
--      thinks the surface is. Believed at any AGL (TD.confirmed already debounces it and
--      gates it on freshness), which is the whole point of having the sensor.
--   2. AGL against the site altitude -- the original test, and the one that still works with
--      the sensor offline. OPT-IN (useAGL), because it is an ASSERTION, not a measurement:
--      it says "landing.db claims a surface at this height, so we must be on it". NOT true of a
--      helipad, which is just a published 3D point and may have nothing but air under it (WCL),
--      so the pad machine leaves it out and probes for the surface instead (LG.PAD_PROBE_M) --
--      and true of a runway only WHERE THE RUNWAY IS, which is why the runway machine passes
--      `LND.s >= 0` rather than a bare `true`: short of the threshold the ground underneath is
--      not the strip, and asserting a touchdown over it is how a landing rolls out into the
--      undershoot.
--   3. "commanding descent but no longer sinking" near the surface -- the backup for a site
--      altitude that is a metre or two off and a sensor that is not there. This one IS a
--      measurement, which is why it is the pad machine's fallback once (2) is gone: the probe
--      keeps a descent commanded below the pad altitude precisely so this test can still speak.
-- Returns which test tripped ("squat"/"AGL"/"settle"), or nil. Owns LND.settle.
function LND.touched(agl, useAGL)
    if TD.confirmed() then return "squat" end
    if useAGL and agl <= LG.TOUCHDOWN_AGL then return "AGL" end
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
        and resolveDirVec(site.dir) ~= nil
end

-- WHERE THE GLIDEPATH MEETS THE STRIP, AND HOW STEEPLY -- the one place the approach's vertical
-- geometry is decided, so the final approach, the go-around circuit's descent and a published
-- approach's descent cannot disagree about it. Returns the aim point (m past the threshold) and
-- tan(glideslope).
--
-- THE GLIDEPATH IS ZERO AGL AT THE AIM POINT (LND.rwyGeom measures gsDev to it, and the wheels
-- arrive `float` metres further on), so the height the aircraft crosses the THRESHOLD at is
-- aim * tan(gs) and nothing else -- it is not a knob, it falls out of the two that are. With
-- the published 4 deg and an aim point of 0.15 of the strip capped at 60 m, that crossing
-- height came out at 0.7 m (NUS/NUN) to 3.1 m (TMV/TMA) on every runway in landing.db: the
-- aircraft crossed the threshold at knee height, having begun its FLARE (RWY_FLARE_AGL, 5 m)
-- 27-61 m BEFORE the threshold and being entitled to assert touchdown (TOUCHDOWN_AGL, 1 m)
-- before it too. That is "autolanding on a runway always approaches too low", and -- once the
-- ALT_OFFSET datum fix of 2026-08-29 stopped the altimetry over-reading by a hull height and
-- the aircraft actually flew where this geometry says -- it is also "it flares and rolls out
-- well before the runway and sinks in short of it". The masking, not the geometry, was what
-- changed; the geometry was always this.
--
-- So the crossing height is made the INPUT (RWY_TCH) and the two that used to be inputs give
-- way to it in the order that costs least:
--   1. move the aim point out to where the PUBLISHED slope crosses at RWY_TCH. Free, except
--      that it is runway spent, so it is capped at RWY_AIM_MAX_FRAC of the strip.
--   2. only if that is not enough, STEEPEN. A 70 m strip cannot be approached at 4 deg with any
--      sane crossing height -- the aim point alone would be past the far end -- and the honest
--      answer to a short runway is a steep approach, which is what a VTOL would fly onto it by
--      hand. The approach-speed sizing in LND.rwySetup pays for it automatically: its first cap
--      is RWY_VS_MAX * RWY_GS_MARGIN / tan(gs), so a steeper strip is simply flown slower.
-- The crossing height is never allowed below RWY_FLARE_AGL whatever RWY_TCH says, because a
-- flare that begins before the threshold is the entire fault this exists to fix.
function LND.rwyGlide(site)
    local pub = math.tan(math.rad(LG.RWY_GS_DEG))
    if not LND.rwyUsable(site) then return 0, pub end
    local tch = math.max(LG.RWY_TCH, LG.RWY_FLARE_AGL)
    local aim = math.min(site.rwyLen * LG.RWY_AIM_FRAC, LG.RWY_AIM_MAX)
    aim = math.max(aim, tch / pub)                            -- (1) out to what the slope needs
    aim = math.min(aim, site.rwyLen * LG.RWY_AIM_MAX_FRAC)    -- ...but not more strip than this
    return aim, math.max(pub, tch / aim)                      -- (2) steepen for whatever is left
end

-- ...for the two callers that only want the slope: the go-around circuit and the published
-- approach both descend onto THIS path, and reading it from anywhere else is how they would
-- deliver the aircraft onto a glidepath the approach is not flying.
function LND.gsTanFor(site)
    local _, t = LND.rwyGlide(site)
    return t
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
    local v = resolveDirVec(NAV.landSite.dir)
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
    -- ...and ahead of both, the circuit the APPROACH ITSELF named. An `appr` record that
    -- references a go-around id is the approach chart printing its own missed-approach
    -- procedure at the bottom, and a published pairing beats matching by name (which cannot
    -- tell two circuits at one field apart) and beats inferring by geometry outright.
    -- NAV.apprGa survives a go-around on purpose, so the second attempt flies the same circuit.
    local named = NAV.apprGa and LANDING[NAV.apprGa]
    if named then
        local w = LND.gaWpOne(named, site and site.name)
        if w then return w end
        if named.type == "goaround" then return named end
    end
    return LND.gaWpFor(site) or LND.gaPathFor(site, thrX, thrZ)
end

-- One circuit, oriented for one runway. A `goaround-wp` list has to END at the runway it serves
-- (otherwise it is a departure, not a go-around); one written the other way round -- only its
-- FIRST entry naming the runway -- is flown backwards rather than refused, which is also what
-- makes the two ends of a single strip pick out their own circuit and not each other's.
-- nil when the record names this runway at neither end, or is not a circuit at all.
function LND.gaWpOne(e, siteName)
    local seq = (e and e.type == "goaround-wp") and e.seq or nil
    if not (seq and #seq >= 2 and siteName) then return nil end
    if seq[#seq] == siteName then return e end
    if seq[1] == siteName then
        local rev = {}
        for i = #seq, 1, -1 do table.insert(rev, seq[i]) end
        return {type = e.type, alt = e.alt, seq = rev}
    end
    return nil
end

-- The waypoint form, matched BY NAME: its list starts and ends at the runway it belongs to, so
-- there is nothing to infer. Orientation (and the whole "which end of the strip is this?"
-- question) is LND.gaWpOne's; this only walks the file looking for one that answers.
function LND.gaWpFor(site)
    if not (site and site.name) then return nil end
    for _, e in pairs(LANDING) do
        local w = LND.gaWpOne(e, site.name)
        if w then return w end
    end
    return nil
end

-- The straight-leg form, matched GEOMETRICALLY: the id in landing.db is explicitly arbitrary,
-- so the route whose leg runs the same way as the landing direction is the one that CONTINUES
-- a missed approach rather than opposing it. Ties (a field with several parallel routes) go to
-- whichever starts nearest the threshold.
function LND.gaPathFor(site, thrX, thrZ)
    local v = resolveDirVec(site and site.dir)
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
    -- The plan about to be built IS the circuit, so the approach that was spliced into the old
    -- one is gone with it -- but NOT NAV.apprGa: that is the pairing the approach published, and
    -- a second miss must fly the same circuit as the first.
    LND.appr = false
    NAV.apprName, NAV.apprFrom, NAV.apprAlt = nil, nil, nil
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
        -- THE SAME glidepath the approach itself will fly (LND.rwyGlide), not the published
        -- angle: a circuit that descends onto a shallower path than the approach flies hands
        -- the aircraft over BELOW the glidepath, which is the one side the approach cannot
        -- recover from (RWY_GS_MARGIN buys authority to catch it from above only).
        local gsTan = LND.gsTanFor(NAV.landSite)
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

-- APPROACH PATTERNS ------------------------------------------------------------
-- landing.db's FOURTH record shape (the pilot's version of all this is in apconf.lua's
-- LG.APPR_* block):
--
--   `<id>=appr,minimumAltitude,<fixes...>`
--
-- A PUBLISHED ARRIVAL: the fixes to fly, in order, ending at the landing site it serves --
-- which is also how it is matched to that site, by NAME, with nothing inferred, exactly as a
-- `goaround-wp` circuit is. A field publishes one per direction it can be arrived from, and the
-- FMS picks ONE when the plan is prepared. That choice is the feature: landing.db's own words
-- are "on preparation to the landing, the FMS MUST choose the most suitable approach pattern
-- according to its current position and heading", and APR.cost is what "most suitable" means.
--
-- Everything after the choice is an ordinary route. The chosen fixes are spliced into the plan
-- ahead of the landing site (APR.prep -- the counterpart of LND.rwyPrep, and called beside it)
-- and the en-route planner flies them with the same jerk-limited corners, the same baked speed
-- profile and the same straight-in final as any other plan. There is no approach state machine
-- here, and deliberately so: the same reason GA.buildWp is eight lines long.
--
-- What is NOT ordinary is the VERTICAL channel. A published approach comes with a minimum
-- altitude because these procedures put the last fix a few hundred metres from the threshold,
-- and cruise height cannot be flown off in that distance -- the approach would arrive high,
-- fail to catch the glidepath and go around for ever. So entering the pattern hands the
-- vertical to the FMS (APR.arm), which flies it exactly the way a published go-around circuit
-- is already flown: hold the published minimum until the glidepath -- extended back along
-- whatever is left of the plan -- sinks below it, then ride it down to the threshold. That is
-- LND.gaTick's waypoint branch, unchanged and now doing both jobs.
--
-- REFERENCES. landing.db says a pattern may name another pattern's id in its fix list, and the
-- two directions of that mean different things -- as they do on a real approach chart:
--   * naming another APPROACH continues as that approach, its fixes spliced in where the
--     reference stood. It is how a family of arrivals shares one common tail:
--     `TMV_APPR3=appr,150,NTARI,TMV_APPR2` is "NTARI, then fly TMV_APPR2".
--   * naming a GO-AROUND contributes only the runway that circuit serves, and records the
--     circuit as THIS approach's missed-approach procedure (LND.gaFor honours it):
--     `TMA_APPR1=appr,150,KUTAC,UKRAN,TMA_GAWP` is "KUTAC, UKRAN, land at TMA -- and if you
--     miss, fly TMA_GAWP". Splicing the circuit's own fixes in here instead would fly a whole
--     lap of the aerodrome on ARRIVAL, which is not what a missed approach printed at the
--     bottom of an approach chart means.

-- Expand one pattern's fix list per those rules. Returns the flat list of names and the id of
-- the missed-approach circuit it named (nil if it named none).
-- Cycles and runaway nesting are bounded rather than trusted -- an id already on the stack is
-- skipped, and APPR_MAX_REF caps the depth -- because this runs on data that arrived over
-- rednet from the nav data server and that nobody has flown yet.
function APR.expand(rec, seen, depth)
    local out, ga = {}, nil
    for _, nm in ipairs(rec.seq or {}) do
        local ref = LANDING[nm]
        if POINTS[nm] then
            table.insert(out, nm)                  -- an ordinary fix (or the site at the end)
        elseif ref and ref.type == "appr" then
            if not seen[nm] and depth < LG.APPR_MAX_REF then
                seen[nm] = true
                local sub, subGa = APR.expand(ref, seen, depth + 1)
                seen[nm] = nil
                for _, s in ipairs(sub) do table.insert(out, s) end
                ga = ga or subGa
            end
        elseif ref and APR.PATTERNS[ref.type] then
            -- A go-around: this approach's missed-approach procedure, plus -- for the waypoint
            -- shape, which starts and ends at the runway it serves -- the name of that runway,
            -- which is the site this approach is FOR. The straight-leg shape names no runway,
            -- so it contributes the pairing and nothing else.
            ga = ga or nm
            local rwy = ref.seq and ref.seq[#ref.seq]
            if rwy and POINTS[rwy] then table.insert(out, rwy) end
        end
        -- Anything else is a name this aircraft's points.db does not have, and is skipped --
        -- exactly what setRoute does with a course naming a point that has since been deleted.
    end
    return out, ga
end

-- Every published approach that serves this site, as flyable fixes. An approach QUALIFIES by
-- ending at the site's name, and is never reversed to make it fit: a go-around circuit may be
-- flown either way round (both ends of one strip share it), but an approach runs one way only,
-- into the field. The trailing site entry is dropped from the fixes -- the plan already ends
-- with it, and the pattern's job is only to say how to get there.
-- Ordered by id, because pairs() order is not stable across boots and two approaches costing
-- the same must not give the aerodrome a different shape every time the computer reboots.
function APR.forSite(siteName)
    local out = {}
    for id, e in pairs(LANDING) do
        if e.type == "appr" then
            local seq, ga = APR.expand(e, {[id] = true}, 1)
            if #seq >= 2 and seq[#seq] == siteName then
                local fixes = {}
                for i = 1, #seq - 1 do
                    local p = POINTS[seq[i]]
                    if p then
                        table.insert(fixes, {name = p.name, x = p.x, z = p.z, kind = p.kind})
                    end
                end
                if #fixes >= 1 then
                    table.insert(out, {id = id, alt = e.alt, ga = ga, fixes = fixes})
                end
            end
        end
    end
    table.sort(out, function(a, b) return a.id < b.id end)
    return out
end

-- What this approach COSTS from where the plan arrives, in METRES OF TRACK: the run-in to the
-- fix it is JOINED at, the rest of the pattern from there, and a charge for the turning that
-- join demands -- a radian of turn costing a turn radius' worth of track, which is roughly what
-- a radian of turn really costs. The radius is the same R = V^2/(g*tan(bank)) the path planner
-- and the go-around builder use, evaluated at APPR_SPEED because that is the speed the pattern
-- is actually flown at.
-- TWO turns are charged: swinging onto the run-in from the heading the plan arrives on, and the
-- corner at the join fix itself. Together they are what makes a pattern BEHIND the aircraft
-- lose to a longer one ahead of it -- i.e. what makes this a choice by position AND heading and
-- not by position alone. APPR_TURN_WEIGHT is how much that matters; at 0 it is pure distance.
--
-- `from` is the fix the procedure is ENTERED at: 1 is its published entry (fly the whole thing),
-- a higher index is a "direct to <fix>" join onto the middle of it. The two turn angles come
-- back alongside the cost because the callers must also TEST them against APPR_JOIN_MAX_DEG,
-- and working them out a second time is how the two copies would drift apart.
function APR.cost(cand, site, px, pz, hdg, from)
    local f = cand.fixes
    from = from or 1
    local e = f[from]
    local dx, dz = e.x - px, e.z - pz
    local dIn = math.sqrt(dx * dx + dz * dz)
    local len = 0
    for i = from, #f - 1 do len = len + legLength(f[i], f[i + 1]) end
    len = len + legLength(f[#f], site)
    local t1, t2 = 0, 0
    if dIn > 1e-3 then
        local inH = atan2(dx, dz)
        if hdg then t1 = math.abs(wrapPi(inH - hdg)) end
        local nxt = f[from + 1] or site
        local nx, nz = nxt.x - e.x, nxt.z - e.z
        if nx * nx + nz * nz > 1e-6 then
            t2 = math.abs(wrapPi(atan2(nx, nz) - inH))
        end
    end
    local R = LG.APPR_SPEED * LG.APPR_SPEED / (DC.GRAVITY * math.tan(math.rad(DC.MAX_BANK_DEG)))
    return dIn + len + LG.APPR_TURN_WEIGHT * R * (t1 + t2), t1, t2
end

-- Where the plan ARRIVES, and pointing where: the position and bearing an approach is chosen
-- against. `b` is the last index of pts the aircraft flies before the pattern -- normally #pts-1,
-- the last en-route fix, and 0 when there is nothing in the plan before the destination at all
-- (a DIRECT-TO, or a plan whose entire tail was the pattern and has just been trimmed off it),
-- in which case the aircraft's own fix and heading are the arrival. Returns nil when there is
-- neither. hdg stays nil when there is no leg to take a bearing from, and APR.cost then charges
-- only the corner at the join fix.
--
-- These are the position and heading the plan ARRIVES on, not the ones the aircraft happens to
-- be sitting at while the pilot presses the button. Choosing off the parked position instead
-- would pick the approach for the wrong side of the aerodrome on every plan that turns on the
-- way. For a direct-to the two are the same thing -- there is nothing in the plan before the
-- destination, so the run-in starts here and now.
function APR.arrival(pts, b)
    if b >= 1 then
        local bx, bz, h
        if b >= 2 then bx, bz = pts[b - 1].x, pts[b - 1].z
        elseif NAV.haveFix then bx, bz = NAV.posX, NAV.posZ end
        if bx and ((pts[b].x - bx) ^ 2 + (pts[b].z - bz) ^ 2) > 1e-6 then
            h = atan2(pts[b].x - bx, pts[b].z - bz)
        end
        return pts[b].x, pts[b].z, h
    end
    if NAV.haveFix then
        return NAV.posX, NAV.posZ, (NAV.haveHeading and NAV.heading or nil)
    end
    return nil
end

-- How much of this pattern the plan ALREADY ENDS WITH: walking back from the last en-route fix,
-- how many of them are the procedure's own fixes, in the procedure's own order, finishing at its
-- last one. Returns that count and the pattern index the run starts at (0, 0 when the plan's
-- tail is not a tail of the procedure -- a course that merely brushes one of its fixes on the
-- way past has flown none of it).
--
-- Being ON the procedure's fixes is not the same as FLYING the procedure, and missing that
-- distinction was the whole of the WAX bug. The auto-router's solved route to WAX ends
-- ..., DSWAE, KOBUL, WAUZA, WAX -- WAX_APPR1's last two fixes, in the right order -- which the
-- old "join at the last occurrence of the plan's final fix" rule read as "established, nothing
-- left to splice". It is not established: the leg in from DSWAE meets the procedure 71 deg
-- across its own track and the aircraft then had to reverse 156 deg at KOBUL to follow it. So
-- the run is only half the test; HOW the plan reaches it is the other half, and that is
-- APPR_JOIN_MAX_DEG, applied by both callers below.
function APR.flownRun(cand, pts, n)
    local f, run = cand.fixes, 0
    local j = #f
    while j >= 1 and n - 1 - run >= 1 and pts[n - 1 - run].name == f[j].name do
        run, j = run + 1, j - 1
    end
    if run == 0 then return 0, 0 end
    return run, j + 1
end

-- Does the plan's last en-route fix belong to one of this site's published patterns? Asked only
-- after APR.established has said the plan is NOT flying that pattern -- and the two answers
-- together are what disqualifies the straight-in test below.
--
-- The straight-in test is an ANGLE, and an angle alone was the second half of the WAX bug.
-- WAX_APPR1's last leg, WAUZA -> WAX, is 514 m straight up the centreline, so the solved route
-- that reversed 156 deg at KOBUL to reach WAUZA nonetheless arrived "lined up" and the whole
-- procedure was skipped -- published minimum, missed approach and all. But LENGTHENING the test
-- (demand N metres of aligned run) would break the case it exists for from the other side: a
-- pilot pressing DIRECT WAX from 800 m out on the centreline is lined up and must be left alone.
-- The honest distinction is not how long the aligned run is, it is whether the plan is SITTING
-- ON a published procedure it has already been found not to be flying. If it is, the alignment
-- is an accident of the procedure's own geometry rather than a final approach, and it does not
-- get to veto the splice.
function APR.onPattern(pts, n, cands)
    if n < 2 then return false end
    for _, c in ipairs(cands) do
        if APR.hasFix(c, pts[n - 1].name) then return true end
    end
    return false
end

-- APR.prep case 1: is the plan ALREADY FLYING one of these patterns? Returns the pattern and how
-- many of the plan's trailing fixes belong to it, or nil.
-- The run has to be ENTERED cleanly to count: at the pattern's published entry fix (nothing to
-- prove -- that is where the procedure starts), or, for a join further down it, within
-- APPR_JOIN_MAX_DEG on both the swing onto the run-in and the corner at the fix itself.
-- Cheapest of whatever qualifies, so a fix two of the field's procedures share does not resolve
-- by pairs() order.
function APR.established(pts, n, site, cands, cap)
    local best, bestCost, bestRun
    for _, c in ipairs(cands) do
        local run, j = APR.flownRun(c, pts, n)
        if run > 0 then
            local ax, az, ah = APR.arrival(pts, n - 1 - run)
            if ax then
                local cost, t1, t2 = APR.cost(c, site, ax, az, ah, j)
                if (j == 1 or (t1 <= cap and t2 <= cap))
                        and (not bestCost or cost < bestCost) then
                    best, bestCost, bestRun = c, cost, run
                end
            end
        end
    end
    return best, bestRun
end

-- APR.prep case 3: CHOOSE a pattern and the fix to join it at. EVERY fix of every candidate is a
-- candidate join now, not just the published entry and not the old match-the-plan's-last-name
-- rule: index 1 is always allowed, any other only when the turn onto it stays inside
-- APPR_JOIN_MAX_DEG. That is what a real "direct to <fix>" clearance means, and it is both
-- halves of the answer at WAX -- arriving from the north the only clean join is the published
-- entry WAYCL (KOBUL wants 156 deg, WAUZA 150 deg, WAUZU 102 deg), so the whole procedure is
-- flown; arriving from ZTP in the west the run down from DOBUL meets KOBUL 13 deg off the
-- procedure's own track, so the aircraft joins there instead of flying 3 km back out to WAYCL
-- to do it properly. If no join is clean the published entry wins by default, which is the
-- conservative answer: fly the procedure as printed.
--
-- Each candidate is costed from where the plan arrives once ITS OWN trailing fixes are trimmed
-- off (APR.prep does the trimming; this only measures), which is why the arrival is recomputed
-- per candidate instead of being passed in. The trim here is deliberately broader than
-- APR.flownRun's -- ANY trailing fix belonging to the pattern goes, in order or not -- because
-- case 1 has already ruled that whatever is there is not being flown as the procedure, and
-- leaving it in would fly it twice.
-- Returns the pattern, the join index and the trim, or nil when the site publishes nothing.
function APR.choose(pts, n, site, cands, cap)
    local best, bestCost, bestFrom, bestRun
    for _, c in ipairs(cands) do
        local run = 0
        while n - 1 - run >= 1 and APR.hasFix(c, pts[n - 1 - run].name) do run = run + 1 end
        local ax, az, ah = APR.arrival(pts, n - 1 - run)
        if ax then
            for j = 1, #c.fixes do
                if j == 1 or APR.inTrim(pts, n, run, c.fixes[j].name) then
                    local cost, t1, t2 = APR.cost(c, site, ax, az, ah, j)
                    if (j == 1 or (t1 <= cap and t2 <= cap))
                            and (not bestCost or cost < bestCost) then
                        best, bestCost, bestFrom, bestRun = c, cost, j, run
                    end
                end
            end
        end
    end
    return best, bestFrom, bestRun
end

-- Is that name one of the fixes APR.choose just trimmed off the plan -- pts[n-run] .. pts[n-1]?
-- This is what keeps a mid-procedure join HONEST. A join anywhere but the published entry may
-- only be made at a fix the plan was ALREADY ROUTED THROUGH: it is then not a shortcut at all,
-- just the recognition that the network took the aircraft onto the procedure and there is no
-- sense flying back out to the entry to start again. Without the restriction the cheapest join
-- is always the LAST fix of the pattern, and every direct-to would truncate the procedure to its
-- final leg -- measured on the first cut of this change, DIRECT TMV from 12 km west collapsed
-- TMV_APPR1 from PDRNT,TAIMU,NATIO,KOCNI,TAIMU to a bare TAIMU. A direct-to has no plan to be
-- routed through anything, so it trims nothing and flies the procedure whole, as it always did.
function APR.inTrim(pts, n, run, name)
    for i = n - run, n - 1 do
        if pts[i].name == name then return true end
    end
    return false
end

-- Does this pattern contain a fix of that name? (A pattern is a handful of fixes, so a scan is
-- the whole of it; a name-keyed set per candidate would cost a table per plan for nothing.)
function APR.hasFix(cand, name)
    for _, f in ipairs(cand.fixes) do
        if f.name == name then return true end
    end
    return false
end

-- PLAN TIME: give a landing destination its published approach. Mirrors LND.rwyPrep exactly --
-- same contract, same call sites, same reasons: it MUTATES pts, so it runs AFTER setLandFinal
-- (it has to know we mean to land, and where) and BEFORE PLAN.build and the choice of waypoint
-- index, both of which must be built over the fixes it inserts. It runs before rwyPrep too, so
-- the straight-in FINAL fix ends up on the approach's last leg and not the other way round.
-- One more ordering rule, and the one that is easy to break by tidying: it must run AFTER the
-- caller has assigned NAV.fromStart, because splicing a pattern in OVERRIDES it (see below).
--
-- Three cases, tested in this order, and the order IS the fix for the WAX bug:
--
--   1. ALREADY FLYING ONE (APR.established). The plan's tail is a tail of a published pattern
--      AND it arrives along the pattern rather than merely at a fix of it. Courses are built
--      out of the same fixes the procedures are, so this is ordinary rather than exceptional --
--      TMU-WAX and BZR-WAX both end with WAX_APPR1 spelled out. Nothing is spliced, but the
--      pattern is RECORDED: the published minimum and the missed approach are properties of the
--      procedure, not of whoever assembled the point list, and until now the straight-in test
--      threw them away along with the splice, so those two courses flew the procedure at cruise
--      height with no missed approach attached.
--   2. ALREADY STRAIGHT IN (APPR_STRAIGHT_IN_DEG). Not on a pattern, but arriving down the
--      centreline. Leave it alone: a plan lined up on the runway must not be turned away from
--      it. Case 1 running FIRST is what makes this safe -- see APR.onPattern.
--   3. OTHERWISE, CHOOSE ONE (APR.choose). Trim off whatever tail of the chosen pattern the plan
--      had blundered onto -- case 1 has already found that tail is not actually being flown as
--      the procedure -- and splice the pattern in from the fix it is joined at.
function APR.prep(pts)
    NAV.apprName, NAV.apprFrom, NAV.apprAlt, NAV.apprGa = nil, nil, nil, nil
    if not (LG.APPR_ENABLE and NAV.landAtEnd and NAV.landSite) then return end
    local n, site = #pts, pts[#pts]
    local cands = APR.forSite(site.name)
    if #cands == 0 then return end
    local cap = math.rad(LG.APPR_JOIN_MAX_DEG)

    -- (1) Already flying one: record it and change nothing.
    local best, run = APR.established(pts, n, site, cands, cap)
    if best then
        NAV.apprName, NAV.apprFrom = best.id, n - run
        NAV.apprAlt, NAV.apprGa    = best.alt, best.ga
        return
    end

    -- (2) Already lined up -- and not merely sitting on a fix of a pattern case 1 has just
    -- rejected (APR.onPattern). Then this plan IS the approach; leave it alone.
    local v = resolveDirVec(NAV.landSite.dir)
    if v and not APR.onPattern(pts, n, cands) then
        local px, pz = APR.arrival(pts, n - 1)
        local lx, lz = site.x - (px or site.x), site.z - (pz or site.z)
        local ll = math.sqrt(lx * lx + lz * lz)
        if px and ll > 1e-6 and (lx / ll * v[1] + lz / ll * v[2])
                >= math.cos(math.rad(LG.APPR_STRAIGHT_IN_DEG)) then
            return
        end
    end

    -- (3) Choose a pattern and where to join it, then trim and splice.
    local from
    best, from, run = APR.choose(pts, n, site, cands, cap)
    if not best then return end
    for _ = 1, run do
        table.remove(pts, n - 1)
        n = n - 1
    end
    -- ...and then peel back any en-route fix left standing BEHIND the join. Everything before
    -- the pattern was routed to the DESTINATION, so the plan's last fixes can exist only to
    -- reach the aerodrome from a side the procedure does not use, and once the procedure goes in
    -- ahead of them they are an out-and-back. Measured on the auto-router's TMU -> WAX: the
    -- graph reaches WAX through QATON, DSWAE, KOBUL, and splicing WAX_APPR1 in at WAYCL turns
    -- the DSWAE that only ever existed to get to KOBUL into a 131 deg reversal -- the very
    -- hairpin this change removes from KOBUL, moved one fix up the plan. So a fix whose corner
    -- into the join is sharper than a join is allowed to be goes, one at a time, re-testing as
    -- the plan shortens: TMU -> WAX ends ..., DSRET, QATON, WAYCL, ... on the published leg the
    -- courses already use, and WAU -> WAX keeps its DSWAE, which turns only 48 deg.
    local jx, jz = best.fixes[from].x, best.fixes[from].z
    while n >= 2 do
        local ax, az, ah = APR.arrival(pts, n - 1)
        if not ah then break end
        local ox, oz = jx - ax, jz - az
        if ox * ox + oz * oz <= 1e-6
                or math.abs(wrapPi(atan2(ox, oz) - ah)) <= cap then break end
        table.remove(pts, n - 1)
        n = n - 1
    end
    local k = 0
    for j = from, #best.fixes do
        table.insert(pts, n + k, best.fixes[j])
        k = k + 1
    end
    -- A spliced pattern must be flown FROM ITS JOIN FIX, never located on by projection. It has
    -- the same shape as a go-around circuit -- it starts behind the aeroplane and wraps round the
    -- aerodrome the aircraft is already near -- so finding ourselves on it by nearest-sample
    -- would put us at the END of it and report the whole published procedure as already flown.
    -- (Measured: a LAND TMA commanded from over TMU sits ~300 m off the TMA_APPR2 legs, inside
    -- DC.PASS_CORRIDOR, and without this the FMS skipped the procedure and engaged the approach
    -- 186 m above the glidepath.) Cleared again the moment the first fix is genuinely reached,
    -- exactly as for a go-around -- see NAV.fromStart and PLAN.onPath.
    if k > 0 then NAV.fromStart = true end
    NAV.apprName, NAV.apprFrom = best.id, n
    NAV.apprAlt, NAV.apprGa    = best.alt, best.ga
end

-- ENTER the pattern. From here the FMS owns the vertical channel and flies the published
-- minimum, in exactly the way a go-around circuit is flown -- LND.ga is that flag, and
-- LND.gaTick's waypoint branch is that profile (hold the minimum, then ride the glidepath in).
-- Armed by the DISTANCE STILL TO FLY and not merely by reaching the pattern's first fix,
-- because a DIRECT-TO's plan is nothing BUT the pattern: without the distance test such a
-- landing would be flown at the published minimum, at approach speed, from wherever in the
-- world it was commanded. See LG.APPR_ARM_M.
function APR.arm()
    LND.appr, LND.ga, LND.gaWp = true, true, true
    LND.gaAlt    = NAV.apprAlt
    LND.siteAlt  = NAV.landSite.alt   -- gaTick measures the glidepath from the FIELD. On a real
                                      -- go-around engageAutoland had already set this; entering
                                      -- an approach happens BEFORE any landing engages, so it
                                      -- has to be set here or the glidepath is drawn off zero.
    LND.vertHold = false
end

-- Should it arm this tick? Kept beside APR.arm rather than inline in navTick so the whole
-- decision reads in one place. Never while a landing or a go-around already owns the vertical.
function APR.due()
    return LG.APPR_ENABLE and NAV.active and NAV.apprFrom ~= nil and NAV.landSite ~= nil
        and not (LND.appr or LND.ga or LND.active)
        and NAV.index >= NAV.apprFrom
        and LND.gaRemaining() <= LG.APPR_ARM_M
end

-- Lay out the runway and size the approach, once, at engagement.
function LND.rwySetup(site, fp)
    local v    = resolveDirVec(site.dir)
    local aim, gsTan = LND.rwyGlide(site)
    local stop = math.max(site.rwyLen - LG.RWY_STOP_MARGIN, aim + 1)
    LND.rwy    = true
    LND.phase  = "INTERCEPT"
    LND.rwyLen = site.rwyLen
    LND.ux, LND.uz     = v[1], v[2]
    LND.gsTan          = gsTan
    LND.aimX,  LND.aimZ  = fp.x + v[1] * aim,          fp.z + v[2] * aim
    LND.stopX, LND.stopZ = fp.x + v[1] * stop,         fp.z + v[2] * stop
    LND.endX,  LND.endZ  = fp.x + v[1] * site.rwyLen,  fp.z + v[2] * site.rwyLen
    LND.stopS = stop
    -- APPROACH SPEED. Three caps: what we asked for; what the glideslope can be flown at with
    -- the vertical channel's descent authority, keeping RWY_GS_MARGIN in hand to rejoin the
    -- path from above; and what the rollout can stop from in the runway that is genuinely left.
    --
    -- "Genuinely left" has to budget the FLARE, which FLOATS the aircraft a long way down the
    -- strip before the wheels arrive. It is easily the biggest consumer of a short runway --
    -- easing from the glideslope sink to RWY_TD_VS through RWY_FLARE_AGL takes
    -- 2*FLARE_AGL/(V*gsTan + TD_VS) seconds, which at 37 m/s is ~90 m of runway. Sizing the
    -- speed off (stop - aim) alone promised a touchdown at the aim point and delivered one
    -- ~90 m later, so the same approach randomly ran out of room.
    -- RWY_ROLLOUT_FRAC then reserves part of what remains as deliberate go-around slack, so the
    -- planned touchdown sits comfortably before the point of no return rather than on it.
    --
    -- The float depends on the very speed it is limiting, so the answer is a FIXED POINT:
    --   V = min(cap, sqrt(2*BRAKE_DECEL*ROLLOUT_FRAC*(stop - aim - float(V))))
    -- and it is solved here by BISECTION, not by the old "iterate four times and keep whichever
    -- is smaller" loop. That loop LATCHED. The right-hand side FALLS as V rises, so the first
    -- pass -- taken at the cap -- charges the float of a speed the aircraft was never going to
    -- fly, and `v = math.min(v, ...)` then made that one pessimistic pass permanent: the three
    -- passes after it agree the answer is faster and are forbidden from saying so.
    -- Measured at WAX, 166 m of runway (aim 24.9 m, stop 136 m): pass 1 charged the 91 m float
    -- of the 37.5 m/s cap and dropped V to 10.6 m/s; pass 2 found 10.6 m/s only floats 47 m and
    -- that 18.9 m/s would fit inside what is left -- and could not raise it. Flown, that is the
    -- 11 m/s crawl down the WAX centreline. The true root is 16.5 m/s.
    -- Bisection cannot latch: the residual V - min(cap, sqrt(...)) is strictly increasing in V,
    -- so the bracket always holds the single root and closes on it from both sides. The cap is
    -- tested first, so a runway long enough to want the full RWY_APP_SPEED still gets exactly
    -- that and never pays for the search.
    -- The braking term is 2*a*s, i.e. v^2 = 2*a*s solved for v -- as the comment beside
    -- RWY_BRAKE_DECEL in apconf and the autobrake at the bottom of LND.rwyTick both already
    -- said. It was written a*a*s here (and v^2/a^2 in the missed-approach test below), which is
    -- not a stopping distance at all: at RWY_BRAKE_DECEL = 20 it claimed the aircraft could
    -- stop in a TENTH of the room it really needs, so the rollout cap almost never bound and a
    -- short strip was flown at the full descent-limited speed. Fixed 2026-09-08 with the
    -- glidepath geometry above, since the two together decide where the wheels arrive.
    local v = math.min(LG.RWY_APP_SPEED, LG.RWY_VS_MAX * LG.RWY_GS_MARGIN / LND.gsTan)
    local float = 2 * LG.RWY_FLARE_AGL * v / (v * LND.gsTan + LG.RWY_TD_VS)
    if v * v > 2 * LG.RWY_BRAKE_DECEL
            * math.max((stop - aim - float) * LG.RWY_ROLLOUT_FRAC, 5) then
        local lo, hi = 0, v
        for _ = 1, 32 do
            v = (lo + hi) * 0.5
            float = 2 * LG.RWY_FLARE_AGL * v / (v * LND.gsTan + LG.RWY_TD_VS)
            if v * v <= 2 * LG.RWY_BRAKE_DECEL
                    * math.max((stop - aim - float) * LG.RWY_ROLLOUT_FRAC, 5) then
                lo = v
            else
                hi = v
            end
        end
        v     = lo
        float = 2 * LG.RWY_FLARE_AGL * v / (v * LND.gsTan + LG.RWY_TD_VS)
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
    local decel = DC.DECEL_MAX      -- how fast `want` may be closed on; see the slew at the end

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
        -- ...but the AGL assertion is only true OVER THE STRIP. "landing.db claims a surface at
        -- this height, so we must be on it" is a statement about the runway, and short of the
        -- threshold the ground underneath is not the runway -- asserting a touchdown there is
        -- how a landing rolls out into the undershoot. Same gate, and the same reasoning, as
        -- the missed-approach test below. The squat switch and the settle test are NOT gated:
        -- both are measurements of what is actually happening to the aircraft, and if the gear
        -- is loaded short of the runway the FMS needs to know that wherever it happened.
        local td = LND.touched(agl, LND.s >= 0)
        if td then
            LND.tdBy, LND.phase, LND.settle = td, "ROLLOUT", 0
            -- THE ROLLOUT BRAKE STARTS FROM THE SPEED THE WHEELS ACTUALLY ARRIVED AT, and it
            -- starts on THIS tick. Both halves matter, because the commanded speed has to be
            -- CONTINUOUS across the touchdown or the aircraft is jolted at the exact moment it
            -- is least able to take it. Seeding from the measured ground speed rather than from
            -- LND.appSpeed covers the aircraft that floated and arrived slow, or came in hot:
            -- either way the first rollout command equals what the craft is already doing, so
            -- the translation law sees zero velocity error and nothing to correct.
            LND.cmdSpeed, want, decel = NAV.groundSpeed, NAV.groundSpeed, LG.RWY_BRAKE_DECEL
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
        -- AUTOBRAKE. Normally RWY_BRAKE_DECEL, the rate the knob names and the rate the strip
        -- was sized on -- but never less than the rate that actually stops the aircraft in the
        -- runway it has left, v^2/2s. The floor is a continuous function of a distance that
        -- itself varies continuously, so it firms the brake up without ever putting a step back
        -- into the command: on a nominal touchdown it never binds at all (WAX with 109 m to run
        -- wants 6.5 m/s^2, well under the 20 it is given), and it only takes over when the
        -- landing floated long -- which is exactly when the sized rate is no longer enough.
        -- Note this is the ONE place the FMS assumes what RWY_BRAKE_DECEL physically means. If
        -- the craft cannot actually deliver the commanded rate the UAC just saturates in
        -- reverse, which is all anything could do by then; the honest guard against that is
        -- upstream, in the missed-approach test.
        decel = math.max(LG.RWY_BRAKE_DECEL,
            LND.cmdSpeed * LND.cmdSpeed / (2 * math.max(LND.stopS - LND.s, 1)))
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
    -- The ROLLOUT names its own rate. DC.DECEL_MAX is an EN-ROUTE knob -- how fast the commanded
    -- speed may fall, sized only to round the steps out of the speed envelope -- and on this
    -- airframe it is 32 m/s^2, which as a braking law is over three g. RWY_BRAKE_DECEL is the
    -- one whose whole definition is "m/s^2 the rollout is ASSUMED to brake at" (apconf), and
    -- until now nothing ever actually braked at it: it only ever sized the approach speed. Now
    -- the rollout is flown at it, so the assumption and the landing are the same number.
    if want > LND.cmdSpeed then
        LND.cmdSpeed = math.min(want, LND.cmdSpeed + DC.ACCEL_MAX * TICK_INTERVAL)
    else
        LND.cmdSpeed = math.max(want, LND.cmdSpeed - decel * TICK_INTERVAL)
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
    LND.appr        = false   -- the published approach has delivered us here; from the arm point
                              -- the runway approach owns the vertical (NAV.apprGa is KEPT, so a
                              -- missed approach still flies the circuit the pattern named)
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

-- Down on a pad. Step 5 of the pad sequence, factored out because two phases can reach it
-- (the descent/probe, and a squat contact made while station-keeping in HOVER) and both must
-- tear down identically: stop the navigation -- NAVCOM coasts, handing control back -- and tell
-- the vertical channel "landed", which holds the ground, keeps ignoring ALT-AP, and (with
-- CFG.AL_GROUND_CUT set on the UAC) cuts the lift thrusters dead so the craft sits on its gear
-- instead of hovering on them. On the LND table rather than as another top-level local: see the
-- 200-locals note in the DC block.
function LND.padDown(tdBy)
    LND.tdBy   = tdBy
    LND.phase  = "LANDED"
    LND.vs     = 0
    LND.settle = 0
    rednet.broadcast({ cancel = true }, APNAV_PROTOCOL)
    NAV.active    = false
    NAV.landAtEnd = false
    broadcastVertical()   -- immediate "landed" so the vertical channel latches at once
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

    elseif LND.phase == "DESCEND" or LND.phase == "FLARE" or LND.phase == "PROBE" then
        -- Steps 3-4: descend while holding position/heading. If the descent nod has shoved
        -- us off the pad, stop sinking and let NAVCOM re-capture before continuing.
        if drifted then
            LND.vs = 0
            LND.settle = 0
        else
            LND.vs = descentRateFor(agl)
        end
        -- PROBE is the same descent continued BELOW the published pad altitude (see
        -- LG.PAD_PROBE_M): a real pad stops the craft here and one of the touchdown tests
        -- speaks, so getting all the way through the probe is itself the answer -- there is
        -- nothing under this site.
        LND.phase = (agl <= LG.TOUCHDOWN_AGL) and "PROBE"
            or ((agl <= LG.FLARE_AGL) and "FLARE" or "DESCEND")

        -- Touchdown -- squat contact or the settle test, but NOT AGL: on a pad, reaching the
        -- site altitude is a claim, not a measurement (see LND.touched).
        local tdBy = LND.touched(agl, false)
        if tdBy then
            LND.padDown(tdBy)                       -- step 5
        elseif agl <= -LG.PAD_PROBE_M then
            -- Probed the whole way below the pad and found nothing solid: this "helipad" is a
            -- 3D point in open air. Stop looking and station-keep ON it -- the couple of metres
            -- the probe spent are flown straight back by the altitude hold below.
            LND.phase, LND.vs, LND.settle = "HOVER", 0, 0
        end

    elseif LND.phase == "HOVER" then
        -- Arrived, but there is no surface here: hold the published point as tightly as the
        -- aircraft can. The vertical channel is handed LND.siteAlt as an explicit altitude to
        -- hold (broadcastVertical) rather than a zero descent rate, which would only stop the
        -- sink and let the craft drift off the number; the horizontal is unchanged from the way
        -- down -- NAVCOM is still holding x/z and the landing heading via broadcastCurrent.
        -- The landing stays ENGAGED: ABORT hands it back, the selector pauses it, and the squat
        -- contact -- the one test that can still speak with nothing commanded down -- still ends
        -- it if something solid does turn up under the gear.
        LND.vs = 0
        if TD.confirmed() then LND.padDown("squat") end

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
    local owned = DEP.reset()   -- BEFORE the teardown: was the vertical held? (DEP.commit)
    ATC.reset()       -- a new plan is a new question for the controller (and ends any hold)
    LND.active = false
    LND.phase  = "OFF"
    LND.rwy    = false
    LND.ga     = false
    LND.gaCount = 0   -- a plan the pilot just committed to gets its full allowance of attempts
    NAV.cruiseMode = "full"   -- a fresh plan starts at full cruise, never a stale COAST
    NAV.aligning   = true     -- ...and points the nose at the first target before throttling up
    -- ...starting from the speed the aircraft IS DOING, never the one the last plan was
    -- commanding. cmdSpeed is SLEW-limited (DC.DECEL_MAX), not assigned, so a stale cruise cap
    -- left behind by the previous plan needs SPEED_FULL/DECEL_MAX = 5 s to bleed off -- five
    -- seconds of the ALIGN hold during which the UAC is still being handed several hundred m/s
    -- of cruiseCap and quite correctly accelerates on the old heading. It goes stale exactly
    -- where that hurts: navTick returns early for the whole of a landing, and it keeps being
    -- updated while AP-NAV is switched off, so "fly a route, take the stick, stop, commit a new
    -- plan" leaves it at cruise with the aircraft at a standstill. Ground speed is the honest
    -- start: zero when parked, and no step at all when the plan is changed in the cruise.
    NAV.cmdSpeed   = math.min(NAV.cmdSpeed, NAV.groundSpeed)
    NAV.fromStart  = false    -- ...and joins the plan wherever it already is
    NAV.planName = landAtEnd and "LAND" or "DIRECT"
    local pts = {{name = p.name, x = p.x, z = p.z, kind = p.kind}}
    -- The landing site is snapshotted BEFORE the plan is built, because a runway destination
    -- adds a straight-in final leg to the point list that the path planner and the waypoint
    -- index must both be built over (LND.rwyPrep) -- and, ahead of that, a published approach
    -- adds its own fixes (APR.prep). A DIRECT-TO is the one case where the approach really is
    -- chosen off the aircraft's own position and heading: there is no plan before the pad.
    setLandFinal(name, landAtEnd)
    APR.prep(pts)
    LND.rwyPrep(pts)
    collapseReversals(pts)             -- last, after every mutation; see its header
    NAV.points = pts
    NAV.index = 1
    NAV.active = true
    DEP.commit(owned, false)           -- off a runway if we stand on one, else PLAN.build
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
-- (px, pz, ph) judge it from another pose than the aircraft's own: a runway departure asks
-- from the far end of the strip on the runway heading, where the plan is actually joined.
local function bestResumeIndex(pts, px, pz, ph)
    if #pts == 0 then return 1 end
    if not (px or NAV.haveFix) then return 1 end
    px, pz = px or NAV.posX, pz or NAV.posZ
    if not ph and NAV.haveHeading then ph = NAV.heading end
    -- Forward unit vector in the same (x,z) frame as bearing = atan2(dx,dz): heading th maps
    -- to (sin th, cos th). nil when we have no heading -> no backtrack penalty (plain nearest).
    local fwdX, fwdZ
    if ph then
        fwdX, fwdZ = math.sin(ph), math.cos(ph)
    end
    local bestI, bestCost = 1, math.huge
    for i, p in ipairs(pts) do
        local dx, dz = p.x - px, p.z - pz
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

-- RUNWAY DEPARTURE (2026-10-15) ---------------------------------------------------------
-- A plan committed with the aircraft standing on a runway is FLOWN OFF THE RUNWAY, whatever the
-- flight mode: nose onto the runway heading, onto the centreline (VTOL/HYBRID by strafing), run
-- the strip, rotate, climb out straight ahead and only then turn for the route. Before this a
-- VTOL simply rose off the strip under ALT-AP and set off for the first fix from wherever it
-- stood -- which, on the way to a fix beside or behind the runway, meant turning across it.
--
-- TWO HALVES, and they are the two halves a landing has.
--   * THE GROUND RUN is a state machine (READY > ALIGN > LINEUP > ROLL) that owns the aircraft
--     the way LND.active does: navTick returns after it, it names its own point and speed, and
--     it holds the vertical at `landed`. It speaks the ROLLOUT's vocabulary run backwards --
--     phase LANDING, holdHeading = the runway heading, a carrot on the centreline and a
--     commanded speed that RISES -- so the UAC needed nothing new: a VTOL/HYBRID holds the
--     heading and strafes onto the centreline, a FIXEDWING (no strafe) holds the heading and
--     rolls straight, exactly as each of them already flies a rollout. The FMS never learns
--     which it is talking to.
--   * THE CLIMB-OUT is a ROUTE, as a go-around circuit is: the runway, a straight run past its
--     end, and a curvature-limited turn for the first fix beyond the aerodrome, baked into the
--     plan's own samples ahead of the rest of the route (DEP.install). From rotation the route
--     machinery flies it with nothing special, and all CLIMB means is that the FMS still holds
--     the vertical -- a commanded climb, until CFG.DEP.CLIMB_AGL -- which is LND.ga's
--     arrangement for LND.ga's reason: ALT-AP's selection is not a height anybody has checked
--     clears the end of this runway, and with nothing selected it would hold the strip's own.
--
-- WHY THE TURN IS BUILT HERE AND NOT LEFT TO THE FLY-BY PLANNER. The planner rounds a corner by
-- CUTTING it, and the corner here would be the end of the runway: a fix beside or behind the
-- strip -- WAX-TMA, where WAU lies well left of the WAX departure, 270 m from its far end --
-- would start the turn over the runway, before the wheels were off it, and the leg-fit shrink
-- would make a near-reversal into a corner nothing can fly. So the turn off the runway is laid
-- out explicitly, from (runway end + STRAIGHT_M, on the runway heading) at the radius
-- CFG.DEP.SPEED flies, with GA's arcs and straights, in one of two shapes (DEP.candidate):
-- TURN TOWARDS the fix and hand the planner a plan starting at the end of that turn, so it
-- rounds the fix at its own speed like any other; or, where the fix is too close and the route
-- turns too sharply there for any corner to fit, a turn-straight-turn join ARRIVING ON the
-- outbound leg -- which covers beside (an arc) and behind (a teardrop) with no special case,
-- and is how GA.join already flies a go-around onto its leg.
--
-- WHEN. Only at a plan COMMIT, and only from a near-standstill on the ground of a published
-- runway (DEP.plan); every other commit, airborne or on a pad, is exactly what it was. AP-NAV
-- gates it like everything else the FMS flies: committed with AP-NAV off it waits in READY
-- owning nothing, and AP-NAV switched off once it has started ends it (DEP.abandon).

function DEP.ground() return DEP.GROUND[DEP.phase] or false end
function DEP.owns()   return DEP.OWNS[DEP.phase] or false end

-- Is (x,z) on this strip -- within STRIP_HALFW of the centreline, and between the ends or within
-- STRIP_MARGIN of one? The test for the aircraft standing on it, and for the fixes at the head
-- of a plan that ARE the aerodrome being departed (WAX-TMA begins at WAX's own threshold), which
-- nothing flies to. Also returns the along-runway distance, for the caller that wants it.
function DEP.onStrip(x, z, thrX, thrZ, ux, uz, len)
    local dx, dz = x - thrX, z - thrZ
    local s = dx * ux + dz * uz
    return s >= -CFG.DEP.STRIP_MARGIN and s <= len + CFG.DEP.STRIP_MARGIN
        and math.abs(ux * dz - uz * dx) <= CFG.DEP.STRIP_HALFW, s
end

-- Turn, then straight AT a point, arriving on whatever heading that takes. The climb-out for a
-- plan whose first fix past the aerodrome is also its LAST, which has no outbound leg to arrive
-- along (GA.dubins needs one). GA's conventions: s = +1 left, d1 the arc angle, then t metres
-- straight on heading hs. nil if the point lies inside both turning circles.
-- The straight leaves the circle where it is tangent to it AND passes through the point, so
-- (point - centre) = t along hs and R across it, away from the turn: its heading is hs plus
-- atan2(-s*R, t), which is what is solved for below.
function DEP.cs(x, z, h, px, pz, R)
    local best
    for _, s in ipairs({1, -1}) do
        local cx, cz = GA.centre(x, z, h, s, R)
        local dx, dz = px - cx, pz - cz
        local d2 = dx * dx + dz * dz
        if d2 > R * R + 1e-6 then
            local t  = math.sqrt(d2 - R * R)
            local hs = atan2(dx, dz) + atan2(s * R, t)
            local d1 = GA.sweep(s * (hs - h))
            local len = R * d1 + t
            if not best or len < best.len then
                best = {s = s, d1 = d1, hs = hs, t = t, len = len}
            end
        end
    end
    return best
end

-- ...with the straight run-out first searched exactly as GA.bestRun searches it. `outH`, if
-- given, is the heading of the leg leaving the point, and a candidate is kept only if the
-- en-route planner can ROUND the point onto it -- see DEP.candidate for why that matters.
function DEP.bestCs(x, z, h, px, pz, R, lo, hi, outH, outLen)
    local ux, uz = GA.dir(h)
    local step = math.max((hi - lo) / 40, 10)
    local bestL, bestP
    local L = lo
    while L <= hi do
        local p = DEP.cs(x + ux * L, z + uz * L, h, px, pz, R)
        if p and outH then
            -- The corner the planner will be handed at the point, and the SMALLEST tangent
            -- length it could round it with (MIN_RADIUS, no clothoid): if even that does not fit
            -- the planner's LEG_FIT_FRACTION of the legs either side, the corner it builds would
            -- start behind the end of the turn or overrun the next fix -- and a reversal it
            -- cannot build at all (collapseReversals' header).
            local d = math.abs(wrapPi(outH - p.hs))
            if math.deg(d) >= REVERSAL_DEG or DC.MIN_RADIUS * math.tan(d / 2)
                    > DC.LEG_FIT_FRACTION * math.min(p.t, outLen) then
                p = nil
            end
        end
        if p and (not bestP or L + p.len < bestL + bestP.len) then bestL, bestP = L, p end
        L = L + step
    end
    return bestL, bestP
end

-- ONE WAY OF LEAVING ONE RUNWAY, costed. nil if the aircraft is not standing on this strip, or
-- if the plan has nothing beyond the aerodrome to depart TO (a DIRECT to the field we are on).
-- Departures are flown only in a direction landing.db PUBLISHES: a record is the statement that
-- this strip is used this way, and a strip used both ways publishes both ends (TMA/TMV,
-- NUS/NUN). Both ends are candidates wherever the aircraft stands on a strip published twice.
function DEP.candidate(name, li, thr, resume)
    local D = CFG.DEP
    local v = resolveDirVec(li.dir)
    local ux, uz, len = v[1], v[2], li.rwyLen
    local on, s0 = DEP.onStrip(NAV.posX, NAV.posZ, thr.x, thr.z, ux, uz, len)
    if not on or math.abs(NAV.posY - li.alt) > D.GROUND_AGL then return nil end
    local hdg = atan2(ux, uz)
    -- WHERE TO LINE UP. Where it stands, if there is room ahead to reach VR short of the last
    -- RWY_STOP_MARGIN metres; otherwise BACKTRACK to the threshold and take the whole strip.
    -- An aircraft that LANDED here stands exactly at that margin (it is the rollout's stop
    -- point), so departing the way it landed always backtracks -- and on a strip published both
    -- ways the other end is usually cheaper, which the cost below is left to decide.
    local rotS = math.max(len - LG.RWY_STOP_MARGIN, 0)
    local luS, back = math.max(s0, 0), 0
    if luS + D.VR * D.VR / (2 * D.ROLL_ACCEL) > rotS then luS, back = 0, math.max(s0, 0) end
    -- The climb-out starts from the far end on the runway heading, and the first fix is chosen
    -- FROM THERE: the RESUME choice is heading-aware, and the heading that matters is the one
    -- the aircraft will be flying, not the one it happened to park on.
    -- A fix ON the strip is never a candidate: the choice is made among the others, not made
    -- first and then stepped past -- stepping on from a join at the aerodrome's own threshold
    -- lands on whatever follows it in the list, which can be the far side of the field.
    local ex, ez = thr.x + ux * len, thr.z + uz * len
    local pts = NAV.points
    local off, map = {}, {}
    for i, p in ipairs(pts) do
        if not DEP.onStrip(p.x, p.z, thr.x, thr.z, ux, uz, len) then
            off[#off + 1], map[#off + 1] = p, i
        end
    end
    if #off == 0 then return nil end
    local k
    if resume then
        k = map[bestResumeIndex(off, ex, ez, hdg)]
    else
        k = 1
        while DEP.onStrip(pts[k].x, pts[k].z, thr.x, thr.z, ux, uz, len) do k = k + 1 end
    end
    local P = pts[k]
    local R = math.max(D.SPEED * D.SPEED / (DC.GRAVITY * math.tan(math.rad(DC.MAX_BANK_DEG))),
        DC.MIN_RADIUS)
    local c = { name = name, thrX = thr.x, thrZ = thr.z, ux = ux, uz = uz, hdg = hdg,
                len = len, alt = li.alt, luS = luS, back = back, rotS = rotS, k = k, R = R }
    -- THE TURN OFF THE RUNWAY, two ways, the first preferred:
    --  (1) TURN TOWARDS THE FIX AND FLY AT IT, and let the en-route planner round the fix
    --      itself, at the speed it is reached at. A first fix kilometres out is a fix like any
    --      other, and must not be met at the departure's slow radius: that would brake a 320 m/s
    --      cruise to SPEED for a turn that has no business being tight.
    --  (2) when the planner could not round it -- a fix so close, and an outbound leg turning
    --      so sharply, that no corner fits (WAX-TMA: WAU is 360 m behind the climb-out and the
    --      route turns 136 deg there onto a 157 m leg) -- ARRIVE ON THE OUTBOUND LEG instead,
    --      by a turn-straight-turn join at SPEED's radius. That is a loop out and back round, and
    --      it is the shortest thing that can be flown there; it is also GA's construction, which
    --      handles a reversal as the teardrop it is.
    local lo, hi, join = D.STRAIGHT_M, D.STRAIGHT_M + D.RUNOUT_M, nil
    local Pn = pts[k + 1]
    local outH = Pn and legBearing(P.x, P.z, Pn.x, Pn.z)
    c.run, c.cs = DEP.bestCs(ex, ez, hdg, P.x, P.z, R, lo, hi, outH, Pn and legLength(P, Pn))
    if c.cs then
        join = c.cs.len
    elseif Pn then
        c.th  = outH
        c.run = GA.bestRun(ex, ez, hdg, P.x, P.z, c.th, R, lo, hi)
        if not c.run then return nil end
        local p = GA.dubins(ex + ux * c.run, ez + uz * c.run, hdg, P.x, P.z, c.th, R)
        if not p then return nil end
        join = p.len
    else
        return nil
    end
    -- In metres: the backtrack (weighted -- taxiing is slow), the runway still to run, the
    -- climb-out, and the rest of the route from the join, so that two ends joining the plan at
    -- different fixes are compared over the same whole flight.
    local rest = 0
    for i = k, #pts - 1 do rest = rest + legLength(pts[i], pts[i + 1]) end
    c.cost = back * D.BACKTRACK_WEIGHT + (len - luS) + c.run + join + rest
    return c
end

-- BAKE THE CHOSEN DEPARTURE INTO THE PLAN: the runway from the line-up point, the straight run
-- past its end, the join -- and then the en-route plan from the first fix past the aerodrome,
-- built by PLAN.build exactly as any plan is and appended, so that from rotation on every runtime
-- lookup (carrot, speed, course, which fix is behind us, the PLAN view) is served off one path.
-- False, with nothing touched, if the join cannot be laid (the caller then plans as before).
function DEP.install(c)
    local pts = NAV.points
    local P   = pts[c.k]
    local luX, luZ = c.thrX + c.ux * c.luS, c.thrZ + c.uz * c.luS
    local out = {{x = luX, z = luZ, hdg = c.hdg, kappa = 0, s = 0}}
    local s, x, z = GA.straight(luX, luZ, c.hdg, c.len - c.luS, 0, out)   -- the strip
    s, x, z = GA.straight(x, z, c.hdg, c.run, s, out)                     -- straight out past it
    -- The fix where the turn starts. A real waypoint of the plan, not decoration: it is the
    -- target while the climb-out is flown, and its abeam point is where the turn begins.
    local lead, wpS = {{name = "CLIMB", x = x, z = z, kind = "poi"}}, {s}
    local sub = {}
    if c.cs then
        -- (1) the turn, and then the PLAN from where it ends: the straight at the fix and the
        -- corner round it are the en-route planner's, built like any other. The end of the turn
        -- is a fix of its own, the plan's first -- which is also what keeps a DIRECT-TO's turn
        -- flown, since a plan's LAST leg is flown at the raw point and not along the path
        -- (broadcastCurrent): without it the whole turn would be cut.
        local p = c.cs
        s, x, z = GA.arc(x, z, c.hdg, p.s, p.d1, c.R, s, out)
        sub[1] = {name = "TURN", x = x, z = z, kind = "poi"}
    else
        -- (2) all the way onto the outbound leg, at the fix itself.
        s = GA.join(x, z, c.hdg, P.x, P.z, c.th, c.R, s, out, 0, 0)
        if not s then return false end
    end
    local nSyn = #lead + #sub        -- the fixes ahead of pts[c.k] in the new list
    for i = c.k, #pts do sub[#sub + 1] = pts[i] end
    PLAN.build(sub)
    for i = 2, #PLAN.samples do          -- [1] is the join point itself, already the last of out
        local sm = PLAN.samples[i]
        sm.s = sm.s + s
        out[#out + 1] = sm
    end
    for j = 1, #sub do wpS[#wpS + 1] = s + (PLAN.wpS[j] or 0) end
    -- ONE speed profile over the whole of it, from a standstill at the line-up point; the
    -- en-route part's own was baked from wherever ITS first sample began. Curvature limits are
    -- read off the samples exactly as PLAN.build reads them (|kappa| only, so GA's sign
    -- convention does not matter). A corner the sub-plan solved for a faster entry is simply
    -- flown slower, which it can be.
    local turnAccel = DC.GRAVITY * math.tan(math.rad(DC.MAX_BANK_DEG))
    local limits = {}
    for i, sm in ipairs(out) do
        limits[i] = (math.abs(sm.kappa) > 1e-9) and math.sqrt(turnAccel / math.abs(sm.kappa))
            or SPEED_FULL
    end
    limits[1] = 0
    PLAN.bakeSpeedProfile(out, limits, DC.BRAKE_DECEL, DC.ACCEL_MAX, DC.FINAL_ARRIVE_SPEED)
    PLAN.applyArriveBrake(out, DC.ARRIVE_DECEL, DC.FINAL_ARRIVE_SPEED)
    PLAN.samples, PLAN.wpS, PLAN.totalLen = out, wpS, out[#out].s
    PLAN.cursor, PLAN.lastX, PLAN.lastZ, PLAN.lastS, PLAN.lastD2 = 1, nil, nil, 0, math.huge

    local np = {}
    for _, p in ipairs(lead) do np[#np + 1] = p end
    for _, p in ipairs(sub) do np[#np + 1] = p end
    -- The spliced approach's index follows its PATTERN into the new list (see collapseReversals
    -- for why it must): shifted by the fixes dropped ahead of the join and the ones added.
    if NAV.apprFrom then NAV.apprFrom = nSyn + math.max(NAV.apprFrom - c.k, 0) + 1 end
    NAV.points, NAV.index = np, 1
    NAV.aligning, NAV.fromStart = false, false   -- the departure lines the nose up itself

    DEP.name, DEP.thrX, DEP.thrZ, DEP.ux, DEP.uz = c.name, c.thrX, c.thrZ, c.ux, c.uz
    DEP.hdg, DEP.len, DEP.alt, DEP.rotS, DEP.back = c.hdg, c.len, c.alt, c.rotS, c.back
    DEP.luS, DEP.luX, DEP.luZ = c.luS, luX, luZ
    DEP.tgtX, DEP.tgtZ, DEP.holdX, DEP.holdZ = NAV.posX, NAV.posZ, NAV.posX, NAV.posZ
    DEP.cmd, DEP.centreAt = 0, nil
    DEP.phase = "READY"    -- the first navTick starts it, if AP-NAV is on
    DEP.geom()
    return true
end

-- AT A PLAN COMMIT: is this a runway departure? Standing on a runway means a near-standstill
-- (MAX_GS), within GROUND_AGL of a published strip's surveyed elevation, on the strip, and not
-- reported AIRBORNE by a live squat switch (a hover a metre over the runway is not a take-off).
-- Every published direction of every strip the aircraft stands on is costed and the cheapest
-- installed.
function DEP.plan(resume)
    local D = CFG.DEP
    if not (D.ENABLE and NAV.haveFix and NAV.haveHeading) then return false end
    if NAV.groundSpeed > D.MAX_GS then return false end
    if TD.fresh() and not TD.signal then return false end
    local best
    for name, li in pairs(LANDING) do
        local thr = POINTS[name]
        if thr and LND.rwyUsable(li) then
            local c = DEP.candidate(name, li, thr, resume)
            if c and (not best or c.cost < best.cost) then best = c end
        end
    end
    return best ~= nil and DEP.install(best)
end

-- The HEAD of every plan commit (setDirectTo, setRoute, RTR.setSolvedRoute): was the vertical
-- channel held by something the new plan is about to tear down -- a landing, a published
-- pattern, a departure? Asked BEFORE the teardown clears the flags it reads. Ends any departure.
function DEP.reset()
    local owned = LND.active or LND.ga or DEP.owns()
    DEP.phase = "OFF"
    return owned
end

-- ...and the TAIL, with NAV.points and NAV.index chosen: depart off the runway if we stand on
-- one, else build the plan as always. Then hand back a vertical channel the old plan was
-- holding, with `cancel`. That hand-back is NEW, and it was a bug before it: the commits tore
-- LND down without a word to the UAC, which latches `landed`/`descend`/`hold` until told
-- otherwise -- so a plan committed from the LANDED screen was flown along the ground with the
-- lift cut, and one committed mid-approach went on ignoring ALT-AP for good. (ATC.divert had
-- found this for itself and sent its own cancel; it now relies on this one.) A departure takes
-- the channel straight over instead: a cancel would only release it for a tick -- ALT-AP
-- restored, the lift opening -- before `landed` pinned it down again.
function DEP.commit(owned, resume)
    if DEP.plan(resume) then return end
    PLAN.build(NAV.points)
    if owned then rednet.broadcast({ cmd = "cancel" }, AUTOLAND_PROTOCOL) end
end

-- Live runway coordinates, the same quantities LND.rwyGeom keeps for an approach.
function DEP.geom()
    local dx, dz = NAV.posX - DEP.thrX, NAV.posZ - DEP.thrZ
    DEP.s       = dx * DEP.ux + dz * DEP.uz                 -- + = past the threshold
    DEP.xtk     = DEP.ux * dz - DEP.uz * dx                 -- + = right of the centreline
    DEP.xtkRate = DEP.ux * NAV.velZ - DEP.uz * NAV.velX
    DEP.agl     = NAV.posY - DEP.alt
    DEP.hdgErr  = wrapPi(DEP.hdg - NAV.heading)
end

-- End the departure. `cmd` hands the vertical back on the autoland channel -- `cancel`
-- restores the pilot's ALT-AP pick, `release` is the pilot-override verb (01 §2) -- and is nil
-- when whatever takes over (a published approach arming, a landing engaging) sends its own
-- command this same tick.
function DEP.finish(cmd)
    if cmd and DEP.owns() then rednet.broadcast({ cmd = cmd }, AUTOLAND_PROTOCOL) end
    DEP.phase = "OFF"
end

-- Give the ground run up and leave the plan an ordinary one: AP-NAV switched off, or the pilot
-- tapped ALT-AP. The route machinery has it from the next tick, under the ALIGN hold a fresh
-- commit gets, so nothing launches down the old heading when AP-NAV comes back -- it flies the
-- same path from wherever it then is, off the ground under ALT-AP.
function DEP.abandon(cmd)
    DEP.finish(cmd)
    NAV.aligning = true
    NAV.cmdSpeed = math.min(NAV.cmdSpeed, NAV.groundSpeed)
    broadcastCurrent()
end

-- THE GROUND RUN, once per navTick (which returns after it, as it does after landingTick).
--   READY   armed, owning nothing; waits for AP-NAV
--   ALIGN   stand still and bring the nose onto the runway heading (to ALIGN_HDG_DEG)
--   LINEUP  taxi to the line-up point on the centreline -- forwards, BACKWARDS for a
--           backtrack, sideways for a VTOL -- finishing the heading to FINE_HDG_DEG
--   ROLL    a commanded speed rising at ROLL_ACCEL along a localizer carrot; rotate at VR, or
--           at rotS whatever the speed, which is the most runway the roll may use
-- A FIXEDWING cannot strafe, so the centreline may never come: CENTRE_T after it is otherwise
-- lined up it rolls from where it is, parallel to the centreline, which is what it would do by
-- hand. A VTOL that is merely slow to centre gets the same allowance.
function DEP.tick()
    DEP.geom()
    local D, p = CFG.DEP, DEP.phase
    -- STILL ON THE GROUND? Asked every tick, because everything below ends in `landed` -- which
    -- CUTS THE LIFT (AL_GROUND_CUT) -- and the aircraft can leave the strip without the
    -- departure's say: READY owns nothing, so an engaged ALT-AP lifts a VTOL off while AP-NAV is
    -- still off, and a scramble overrides the autoland channel outright. Streaming `landed` into
    -- either is a lift cut in the air. Off the ground, the departure gives up and hands back
    -- what it held (DEP.abandon), and the plan flies on as an ordinary one.
    if DEP.agl > D.GROUND_AGL then DEP.abandon("cancel"); return end
    local apnav = redstone.getInput(CFG.APSWITCH.SIDE)
    if p == "READY" then
        NAV.cmdSpeed = 0
        if apnav then
            DEP.phase, DEP.holdX, DEP.holdZ = "ALIGN", NAV.posX, NAV.posZ
            DEP.tgtX, DEP.tgtZ = DEP.holdX, DEP.holdZ
            broadcastCurrent()
        end
        return
    end
    if not apnav then DEP.abandon("cancel"); return end
    if p == "ALIGN" then
        DEP.tgtX, DEP.tgtZ = DEP.holdX, DEP.holdZ
        if math.abs(DEP.hdgErr) <= math.rad(LG.ALIGN_HDG_DEG) then
            DEP.phase, DEP.centreAt = "LINEUP", nil
        end
    elseif p == "LINEUP" then
        DEP.tgtX, DEP.tgtZ = DEP.luX, DEP.luZ
        local ready = math.abs(DEP.hdgErr) <= math.rad(LG.FINE_HDG_DEG)
            and math.abs(DEP.s - DEP.luS) <= LG.POS_TOL_M
        if not ready then DEP.centreAt = nil
        elseif not DEP.centreAt then DEP.centreAt = os.clock() end
        if ready and (math.abs(DEP.xtk) <= D.XTK_TOL
                or os.clock() - DEP.centreAt >= D.CENTRE_T) then
            DEP.phase, DEP.cmd = "ROLL", 0
        end
    elseif p == "ROLL" then
        DEP.cmd = math.min(DEP.cmd + D.ROLL_ACCEL * TICK_INTERVAL, D.SPEED)
        -- THE APPROACH'S LOCALIZER (LND.rwyGeom), same gains: a lateral closure rate on the
        -- cross-track error, capped at an intercept angle, laid out as a carrot in the
        -- commanded direction. It only ever has a little to do -- LINEUP centred the aircraft
        -- -- and nothing at all on a FIXEDWING, whose strafe is off; its nose is held on the
        -- runway heading whatever the carrot says.
        local ux, uz = DEP.ux, DEP.uz
        local spd = math.max(DEP.cmd, NAV.groundSpeed)
        local cap = spd * math.sin(math.rad(LG.RWY_LOC_MAX_DEG))
        local lat = clamp(-(LG.RWY_LOC_KP * DEP.xtk + LG.RWY_LOC_KD * DEP.xtkRate), -cap, cap)
        local fwd = math.sqrt(math.max(spd * spd - lat * lat, 1))
        local vx, vz = ux * fwd - uz * lat, uz * fwd + ux * lat
        local n = math.sqrt(vx * vx + vz * vz)
        DEP.tgtX = NAV.posX + vx / n * LG.RWY_CARROT_M
        DEP.tgtZ = NAV.posZ + vz / n * LG.RWY_CARROT_M
        if NAV.groundSpeed >= D.VR or DEP.s >= DEP.rotS then
            -- ROTATE. The route machinery has the horizontal from here (the plan's path runs on
            -- down the runway and out past its end), picking the speed up where the roll left
            -- it; the vertical becomes a commanded climb on this tick's broadcastVertical.
            DEP.phase = "CLIMB"
            NAV.cmdSpeed = DEP.cmd
            updatePhase()
            broadcastCurrent()
            return
        end
    end
    NAV.cmdSpeed = DEP.cmd     -- so the HSI's CMD reads the roll
    if DEP.phase ~= p then broadcastCurrent() end   -- a phase change goes out at once
end

-- THE CLIMB-OUT's vertical, once per navTick while the route machinery flies the path: hold the
-- commanded climb until CLIMB_AGL above the strip, then give the vertical to ALT-AP. AP-NAV off
-- ends it at once, as it ends the ground run -- the aircraft is the pilot's, and while the
-- autoland channel is live the UAC counts the navigation as the autopilot's, MANCON and all.
function DEP.climbTick()
    DEP.agl = NAV.posY - DEP.alt
    if not redstone.getInput(CFG.APSWITCH.SIDE) or DEP.agl >= CFG.DEP.CLIMB_AGL then
        DEP.finish("cancel")
    end
end

-- The status screen's line for a departure, and its colour; nil when there is none.
function DEP.line()
    local p = DEP.phase
    if p == "OFF" then return nil end
    local t = string.format("DEP %s %03d ", DEP.name,
        math.floor((180 - math.deg(DEP.hdg)) % 360 + 0.5) % 360)
    if p == "READY" then return t .. "ARMED - AP-NAV off", colours.brown end
    if p == "ALIGN" then
        return t .. string.format("ALIGN %+.0fdeg", math.deg(DEP.hdgErr)), colours.yellow
    end
    if p == "LINEUP" then
        return t .. string.format("LINE UP %.0fm XTK %+.1f", DEP.luS - DEP.s, DEP.xtk),
            colours.yellow
    end
    if p == "ROLL" then
        return t .. string.format("ROLL %.0f/%.0f m/s", NAV.groundSpeed, CFG.DEP.VR),
            colours.lime
    end
    return t .. string.format("CLIMB %.0f/%.0f m", DEP.agl, CFG.DEP.CLIMB_AGL), colours.lime
end

-- resume: join the plan at bestResumeIndex(pts) instead of the first point (mid-route
-- resume). Everything downstream -- phase, pass radii, the PASSED/remaining status list --
-- keys off NAV.index, so a resumed start at index i is identical to having advanced there.
local function setRoute(name, resume)
    local c = COURSES[name]
    if not c then return false end
    local pts = {}
    for _, pname in ipairs(expandCourseSeq(c.seq)) do
        local p = POINTS[pname]
        if p then
            table.insert(pts, {name = p.name, x = p.x, z = p.z, kind = p.kind})
        end
    end
    if #pts == 0 then return false end
    local owned = DEP.reset()   -- BEFORE the teardown: was the vertical held? (DEP.commit)
    ATC.reset()       -- a new plan is a new question for the controller (and ends any hold)
    LND.active = false
    LND.phase  = "OFF"
    LND.rwy    = false
    LND.ga     = false
    LND.gaCount = 0   -- a plan the pilot just committed to gets its full allowance of attempts
    NAV.cruiseMode = "full"   -- a fresh plan starts at full cruise, never a stale COAST
    NAV.aligning   = true     -- ...and points the nose at the first target before throttling up
    -- ...from the speed the aircraft IS DOING, not the one the last plan was commanding: the
    -- ALIGN hold is worth nothing while a stale cap bleeds off at DC.DECEL_MAX (see setDirectTo)
    NAV.cmdSpeed   = math.min(NAV.cmdSpeed, NAV.groundSpeed)
    NAV.fromStart  = false    -- ...and joins the plan wherever it already is (see RESUME)
    NAV.planName = name
    -- A route ending at a landing site auto-lands there on arrival (fully automatic). All three
    -- calls come before PLAN.build / NAV.index: the destination gains its published approach and
    -- then, if it is a runway, a straight-in final leg -- both of them extra fixes in this list.
    setLandFinal(pts[#pts].name, true)
    APR.prep(pts)
    LND.rwyPrep(pts)
    collapseReversals(pts)             -- last, after every mutation; see its header
    NAV.points = pts
    NAV.index = resume and bestResumeIndex(pts) or 1
    NAV.active = true
    DEP.commit(owned, resume)          -- off a runway if we stand on one, else PLAN.build
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
            local names = expandCourseSeq(c.seq)
            local a, b = names[1], names[#names]
            if a and POINTS[a] then set[a] = true end
            if b and POINTS[b] then set[b] = true end
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
            local names = expandCourseSeq(c.seq)
            for i = 1, #names - 1 do link(names[i], names[i + 1]) end
        end
    end
    return adj
end

-- SEARCHING THE ROUTE GRAPH -------------------------------------------------------------
-- Examined 2026-09-17, after the FMS was killed by CC's watchdog ("Too long without yielding")
-- while committing a plan from the TMA/TMV strip to FGY. The solved path was
-- `TMV, TAIMW, TMA, UKRAN, ...` -- out 733 m north past the far threshold and straight back
-- down its own track -- and because TMV, TAIMW and TMA share an x, that is a corner of exactly
-- 180 degrees, which the path planner cannot solve at ALL (_docs/02 section 4).
--
-- **The search and the join are UNCHANGED, and that is a result rather than an omission.**
-- Both obvious repairs were tried and both were wrong: choosing the join by total track
-- degenerates (below), and rooting the search at the destination to price every node at once
-- buys nothing once the join stays nearest -- it only re-orders the settle, which silently
-- changed 20 EQUAL-LENGTH routes to a different tie-break. A reversal is dealt with instead
-- where every source of one passes: `collapseReversals`, up beside PLAN, on the assembled
-- point list.

-- WHERE THE AIRCRAFT JOINS THE NETWORK: the nearest node to the current fix. The first leg
-- flown is a direct hop to it; from there the plan follows published legs. nil without a fix.
--
-- IT IS STILL "NEAREST", AND THAT IS DELIBERATE -- 2026-09-17, having tried the alternative.
-- The join that hung the FMS was a nearest one (parked on the TMA/TMV strip, TMV at 145 m beat
-- TMA at 150 m, and TMV's only way north runs 733 m out to TAIMW and straight back), so the
-- obvious repair is to choose the join by TOTAL TRACK instead: the hop out to the node plus
-- what the network then costs from it. **That degenerates.** The destination is a node like any
-- other and its own cost is zero, so "hop straight to the destination" is always a candidate,
-- and by the triangle inequality it always wins -- the solver returns a single direct leg and
-- the published network is never flown at all. Measured: TMV -> FGY came back as one 19 145 m
-- direct hop instead of the 15-fix airway route. Charging off-airway track more than on-airway
-- would fix that, but only by inventing a weight nobody has flown.
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
    local nodes = 0
    for name in pairs(adj) do dist[name], nodes = math.huge, nodes + 1 end
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
    -- The walk back down prev[] is bounded by the node count rather than trusted: a cycle in
    -- prev would otherwise spin here for ever, and this runs on a graph built from files that
    -- arrive over rednet. (2026-09-17: the bound is new, the walk is not.)
    local path, cur = {}, goalName
    while cur do
        table.insert(path, 1, cur)
        if #path > nodes then return nil end
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

-- THE PREVIEW's solve, cached. drawPlanPreview is called from uiLoop, which wakes on every
-- event the computer sees -- so this used to rebuild the whole route graph (expanding ~90
-- courses) and run a Dijkstra ONCE PER EVENT, tens of times a second, for a screen whose
-- numbers move at walking pace.
--
-- SOLVE_TTL is how stale the preview may be. The reason the preview re-solves at all is that
-- the join range is measured from the live fix, so it must not simply be solved once when the
-- screen opens; half a second of staleness is invisible on a range that changes by metres.
--
-- THE COMMIT DOES NOT USE THIS. RTR.setSolvedRoute calls RTR.solveRoute directly and always
-- re-solves from the live fix, because a plan committed from a cached solve is exactly the
-- "drift between preview and confirm" the commit path exists to rule out.
RTR.SOLVE_TTL = 0.5
RTR.solveCache = { dest = nil, t = -1 }
function RTR.solveCached(destName)
    local c = RTR.solveCache
    if c.dest ~= destName or (os.clock() - c.t) >= RTR.SOLVE_TTL then
        c.dest, c.t = destName, os.clock()
        c.path, c.total, c.join, c.err = RTR.solveRoute(destName)
    end
    return c.path, c.total, c.join, c.err
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
    local owned = DEP.reset()   -- BEFORE the teardown: was the vertical held? (DEP.commit)
    ATC.reset()       -- a new plan is a new question for the controller (and ends any hold)
    LND.active = false
    LND.phase  = "OFF"
    LND.rwy    = false
    LND.ga     = false
    LND.gaCount = 0   -- a plan the pilot just committed to gets its full allowance of attempts
    NAV.cruiseMode = "full"    -- a fresh plan starts at full cruise, never a stale COAST
    NAV.aligning   = true      -- ...and points the nose at the first target before throttling up
    -- ...from the speed the aircraft IS DOING, not the one the last plan was commanding: the
    -- ALIGN hold is worth nothing while a stale cap bleeds off at DC.DECEL_MAX (see setDirectTo)
    NAV.cmdSpeed   = math.min(NAV.cmdSpeed, NAV.groundSpeed)
    NAV.fromStart  = false     -- ...and joins the plan wherever it already is
    NAV.planName = "TO " .. destName
    -- The destination is a hub landing site: auto-land on arrival, same as a route to it --
    -- gaining its published approach and, if it is a runway, a straight-in final leg before the
    -- plan is built. The solved path ends at the hub, so the approach is chosen off the leg the
    -- search decided to arrive on.
    setLandFinal(pts[#pts].name, true)
    APR.prep(pts)
    LND.rwyPrep(pts)
    collapseReversals(pts)               -- last, after every mutation; see its header
    NAV.points = pts
    NAV.index = bestResumeIndex(pts)     -- heading-aware mid-route join, same as RESUME
    NAV.active = true
    DEP.commit(owned, true)              -- off a runway if we stand on one, else PLAN.build
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
    if LND.active or LND.ga or DEP.owns() then
        rednet.broadcast({cmd = vertCmd or "cancel"}, AUTOLAND_PROTOCOL)
    end
    ATC.reset()
    DEP.phase  = "OFF"
    LND.active = false
    LND.phase  = "OFF"
    LND.paused = false
    LND.rwy    = false
    LND.ga     = false
    LND.appr   = false
    LND.gaCount = 0
    NAV.active = false
    NAV.aligning = false
    NAV.cmdSpeed = 0
    NAV.landAtEnd = false
    NAV.landSite  = nil
    NAV.apprName, NAV.apprFrom, NAV.apprAlt, NAV.apprGa = nil, nil, nil, nil
    NAV.planName = nil
    NAV.points = {}
    NAV.index = 1
    NAV.phase = "ENROUTE"
    PLAN.samples, PLAN.wpS, PLAN.totalLen, PLAN.cursor = {}, {}, 0, 1
    PLAN.lastX, PLAN.lastZ, PLAN.lastS, PLAN.lastD2 = nil, nil, 0, math.huge
end

-- AUTOMATED ATC CLIENT -----------------------------------------------------------
-- The aircraft's half of the automated controller (computer #87, atcctl.lua). Inbound to a
-- landing facility, the FMS asks the controller once a second whether it may land there, and
-- does what it is told:
--
--   clr     -- carry on. Nothing changes; the landing engages exactly as it always has.
--   divert  -- land on the named facility instead. The plan is REPLACED by a DIRECT-TO-LAND to
--              it -- the same call the pilot's LAND menu makes -- so it gets its own published
--              approach, runway final and go-around exactly as if the pilot had chosen it.
--   hold    -- stop and wait (CFG.ATC.HOLD: brake to a hover on the path, or orbit), keep
--              asking, and carry on from where the plan stopped once cleared.
--   none    -- the facility is in no site: uncontrolled. Nothing changes.
--
-- IT NEVER OVERRIDES THE PILOT AND NEVER INVENTS AN ORDER. It asks only while AP-NAV is on
-- (read off this computer's own redstone input, as TAVOID does) and only about the plan the
-- pilot committed; RELEASE ATC on the status screen takes the rest of that plan off its hands,
-- and any new plan is a fresh question. A controller that has never answered changes nothing,
-- and one that goes quiet changes nothing it was doing (invariant I8) -- a hold in force stays
-- in force, because "the controller stopped talking" is not "the pad is now free".
--
-- COMMITTED. Once a landing is below the flare (or rolling out) it is finished whatever the
-- controller now says: aborting a flare is how an aircraft lands on its tail. Before that, a
-- hold or divert arriving mid-descent ABORTS the landing -- the vertical goes back to ALT-AP
-- with `cancel` (which restores the pilot's selection, i.e. climbs back up) -- because a
-- facility turning occupied under a descending aircraft is exactly the case this exists for:
-- a parked aircraft at a remote field is silent until its chunk loads, which is when the
-- arriving aircraft is already close.
ATC.COMMITTED = { FLARE = true, PROBE = true, HOVER = true, ROLLOUT = true, LANDED = true }

-- ...and so is a departure still on the runway: an answer about the DESTINATION is no reason to
-- reject a take-off, and a divert there would re-plan with the aircraft rolling. It is acted
-- on from rotation, in the climb-out, like any other answer en route.
function ATC.committed()
    return (LND.active and ATC.COMMITTED[LND.phase]) or DEP.ground() or false
end

-- Forget everything about the current question. Called by every plan commit and by
-- cancelNav, so a pilot's new plan always starts un-held and un-released.
function ATC.reset()
    ATC.holding, ATC.ocx, ATC.ocz = false, nil, nil
    ATC.st, ATC.fac, ATC.reply = nil, nil, nil
    ATC.released, ATC.note = false, nil
end

-- The facility this aircraft is inbound to, and where it is -- or nil. A landing in progress
-- names its own (LND); otherwise the plan's snapshot of its landing site does. Asked right up
-- to LANDED, so the reservation outlasts the rollout; from LANDED on, this aircraft's own
-- responder is what holds the facility (the controller watches it sitting there).
function ATC.wanted()
    if LND.active then
        if LND.phase == "LANDED" or LND.phase == "OFF" then return nil end
        return LND.name, LND.x, LND.z
    end
    if NAV.active and NAV.landAtEnd and NAV.landSite then
        local fp = NAV.points[#NAV.points]
        if fp then return NAV.landSite.name, fp.x, fp.z end
    end
    return nil
end

-- Take the landing machine down WITHOUT ending the plan: engageAutoland never stops NAV, so
-- from the next tick the route machinery flies again and the landing re-engages once the
-- hold ends (navTick's engage test is gated on ATC.holding). The vertical goes back with
-- `cancel`, not `release` -- the altitude selector restores the pilot's pick (01 §2).
function ATC.abortLanding()
    rednet.broadcast({ cmd = "cancel" }, AUTOLAND_PROTOCOL)
    LND.active, LND.phase, LND.paused = false, "OFF", false
    LND.rwy, LND.vertHold = false, false
end

function ATC.beginHold()
    if LND.active then ATC.abortLanding() end
    ATC.holding, ATC.holdAt = true, os.clock()
    ATC.ocx, ATC.ocz = nil, nil
    if CFG.ATC.HOLD == "orbit" then
        -- Centre the circle ORBIT_R to the RIGHT of the ground track (the nose, when barely
        -- moving), so the aircraft is already on it, heading round it. Right of heading h in
        -- this frame is (-cos h, sin h): see broadcastCurrent for the matching carrot.
        local h = (NAV.groundSpeed > 1) and atan2(NAV.velX, NAV.velZ) or NAV.heading
        local R = CFG.ATC.ORBIT_R
        ATC.ocx, ATC.ocz = NAV.posX - R * math.cos(h), NAV.posZ + R * math.sin(h)
    end
    broadcastCurrent()
end

function ATC.endHold()
    ATC.holding, ATC.ocx, ATC.ocz = false, nil, nil
end

-- Re-plan to another facility of the same site. False if this aircraft cannot: the name is not
-- a landing site in OUR database (the controller's and ours can differ for a sync interval).
function ATC.divert(to)
    if not (POINTS[to] and isLandingSite(to)) then return false end
    local from, mode = ATC.fac, NAV.cruiseMode
    -- A landing or a published pattern holds the vertical channel, and it has to be handed
    -- back before the new plan or the altitude selector sits in OVER until its 3 s timeout.
    -- setDirectTo does that itself now (DEP.commit -- every commit had the same hole; this was
    -- the one place that had found it), so it is not sent twice here. It clears LND.active/ga
    -- but not LND.appr, which would stop the new plan's own published approach from ever
    -- arming (APR.due) -- hence the line below.
    LND.paused, LND.appr, LND.vertHold = false, false, false
    if not setDirectTo(to, true) then return false end
    NAV.cruiseMode = mode             -- the pilot's speed selection survives a divert
    NAV.planName   = "ATC " .. to
    ATC.note = from .. ">" .. to      -- after setDirectTo, whose ATC.reset() clears it
    return true
end

-- Once per navTick, before the landing machine: act on the latest answer.
function ATC.tick()
    if not CFG.ATC.ENABLE or ATC.released then return end
    local fac, fx, fz = ATC.wanted()
    -- AP-NAV off: the pilot has the aeroplane, and nothing this module does would reach it.
    if fac and not redstone.getInput(CFG.APSWITCH.SIDE) then fac = nil end
    if fac ~= ATC.fac then
        -- A different facility (or none): whatever was said was about something else.
        if ATC.holding then ATC.endHold() end
        ATC.fac, ATC.st, ATC.reply = fac, nil, nil
    end
    if not fac then return end
    if not ATC.st then
        -- Call up when the field is within CONTACT_M plus what it takes to STOP from the
        -- current speed, so a hold always has room to be flown short of the facility.
        -- Stays in contact from then on, for as long as this is the facility.
        local dx, dz = fx - NAV.posX, fz - NAV.posZ
        local reach = CFG.ATC.CONTACT_M + NAV.groundSpeed ^ 2 / (2 * DC.DECEL_MAX)
        if dx * dx + dz * dz > reach * reach then return end
        ATC.st = "ASK"
    end
    local r = ATC.reply
    if not (r and r.fac == ATC.fac) then return end
    if os.clock() - ATC.heard > CFG.ATC.STALE then return end   -- silent: change nothing
    if ATC.committed() then return end
    if r.t == "clr" then
        if ATC.holding then ATC.endHold() end
        ATC.st = "CLR"
    elseif r.t == "hold" then
        if not ATC.holding then ATC.beginHold() end
        ATC.st = "HOLD"
    elseif r.t == "divert" and type(r.to) == "string" and r.to ~= ATC.fac then
        if not ATC.divert(r.to) then
            if not ATC.holding then ATC.beginHold() end
            ATC.st, ATC.note = "HOLD", "cannot divert to " .. r.to
        end
    elseif r.t == "none" then
        if ATC.holding then ATC.endHold() end
        ATC.st = "UNCTL"
    end
end

-- The pilot's RELEASE ATC: this plan is flown as if there were no controller.
function ATC.release()
    if ATC.holding then ATC.endHold() end
    ATC.released, ATC.st = true, "REL"
end

function ATC.request()
    if not (ATC.st and ATC.fac) or ATC.released then return end
    local msg = {
        t = "req", cs = CFG.TERR.CALLSIGN, fac = ATC.fac,
        x = NAV.posX, y = NAV.posY, z = NAV.posZ,
        wide = CFG.ATC.WIDE, rwyonly = CFG.ATC.RUNWAY_ONLY,
        stage = ATC.holding and "hold" or (LND.active and "landing" or "enroute"),
    }
    -- Straight to the controller once it has answered; broadcast until then, and again if it
    -- goes quiet (it may have rebooted onto a different id).
    if ATC.srv and ATC.heard and os.clock() - ATC.heard <= CFG.ATC.STALE then
        rednet.send(ATC.srv, msg, CFG.ATC.PROTOCOL)
    else
        rednet.broadcast(msg, CFG.ATC.PROTOCOL)
    end
end

-- The radio: send the question on the INTERVAL, keep the latest answer. Acting on it is left
-- to ATC.tick inside navTick, so a divert or a hold changes the plan at a tick boundary like
-- every other plan change does. Replies are addressed by the CALLSIGN this hull goes by
-- (CFG.TERR.CALLSIGN -- the same name its responder transmits, which is how the controller
-- tells this aircraft's own report from somebody else parked on the facility).
function ATC.loop()
    if not CFG.ATC.ENABLE then while true do sleep(5) end end
    local nextReq = 0
    while true do
        local sender, msg = rednet.receive(CFG.ATC.PROTOCOL, math.max(0.05, nextReq - os.clock()))
        if sender and type(msg) == "table" and msg.t ~= "req"
                and msg.cs == CFG.TERR.CALLSIGN and type(msg.fac) == "string" then
            ATC.srv, ATC.heard, ATC.reply = sender, os.clock(), msg
        end
        if os.clock() >= nextReq then
            ATC.request()
            nextReq = os.clock() + CFG.ATC.INTERVAL
        end
    end
end

-- One line for the pilot, or nil when ATC has nothing to say.
function ATC.label()
    if not (CFG.ATC.ENABLE and ATC.st) then return nil end
    if ATC.st == "REL" then return "ATC released - landing unmanaged", colours.brown end
    local silent = not ATC.heard or (os.clock() - ATC.heard > CFG.ATC.STALE)
    if ATC.holding then
        local s = string.format("ATC HOLD %ds ", math.floor(os.clock() - ATC.holdAt))
        if silent then return s .. "- controller silent", colours.red end
        local r = ATC.reply or {}
        return s .. (r.q and ("#" .. r.q .. " ") or "") .. (r.why or ATC.note or ""),
            colours.orange
    end
    if silent then
        return (ATC.heard and "ATC silent - " or "ATC no answer - ") .. ATC.fac, colours.brown
    end
    if ATC.st == "CLR" then
        return "ATC cleared " .. ATC.fac .. (ATC.note and ("  (" .. ATC.note .. ")") or ""),
            colours.lime
    end
    if ATC.st == "UNCTL" then return "ATC " .. ATC.fac .. " uncontrolled", colours.white end
    return "ATC calling for " .. ATC.fac, colours.white
end

local function navTick()
    local pose = sublevel.getLogicalPose()
    local pos = pose.position
    -- ALTITUDE DATUM (2026-08-29). `pos.y` is the ASSEMBLY's pose, which sits
    -- CFG.ALT_OFFSET metres above the bottom of the hull; NAV.posY is the Y of that bottom,
    -- and every altitude in this file is measured against a piece of GROUND -- landing.db's
    -- surveyed pad and runway elevations (LND.siteAlt, and so LND.agl and the glideslope
    -- deviation), the terrain server's profile (TERR.band), the go-around level. Uncorrected,
    -- this aeroplane flew its descent to a pad elevation ALT_OFFSET metres underground and
    -- was stopped only by the squat switch. See CFG.ALT_OFFSET in apconf.
    NAV.posX, NAV.posZ, NAV.posY, NAV.haveFix = pos.x, pos.z, pos.y - CFG.ALT_OFFSET, true
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

    -- The automated controller's answer, acted on BEFORE the landing machine runs: a hold or a
    -- divert may have to take down a landing that is under way (see ATC.tick).
    ATC.tick()

    -- A RUNWAY DEPARTURE still on the ground owns the aircraft exactly as a landing does: it
    -- runs its own machine and streams the vertical, and the waypoint machinery waits for the
    -- rotation (see the RUNWAY DEPARTURE section). No terrain avoidance on the runway.
    if DEP.ground() then
        TAVOID.active = false
        DEP.tick()
        broadcastVertical()
        return
    end

    if LND.active then
        -- The landing owns the aircraft now: run its state machine and stream the vertical
        -- command; skip normal waypoint advance entirely. A terrain-avoidance hold does not
        -- survive a landing engaging -- the landing's own vertical command takes the channel
        -- on the very next broadcastVertical() regardless, so this is bookkeeping only (see
        -- apconf.lua's CFG.TAVOID header).
        TAVOID.active = false
        landingTick()
        broadcastVertical()
        return
    end
    -- Reached the published approach? Take the vertical channel and fly its minimum from here
    -- (see APR.arm/APR.due). Tested before the go-around block below so the pattern's first
    -- tick already commands the descent rather than one tick of stale altitude hold.
    -- (A departure still climbing out stops here: the pattern's own vertical takes over at
    -- once, so there is nothing to hand back.)
    if APR.due() then DEP.finish(); APR.arm() end
    -- A go-around is the other way round: the plan below flies the circuit, and all the
    -- landing machinery still does is hold the route's altitude. A published APPROACH is flown
    -- on exactly this path -- same flag, same vertical profile -- see the APPROACH PATTERNS
    -- section for why the two share it.
    --
    -- Terrain avoidance (CFG.TAVOID) runs in the ELSE: it is an en-route backstop, not a
    -- landing/go-around one -- those vertical profiles are already authored and hand-verified
    -- against the ground (see the header in apconf.lua), and LND.gaTick's own climb already
    -- owns the channel here.
    if LND.ga then
        TAVOID.active = false
        LND.gaTick()
        broadcastVertical()
    elseif DEP.phase == "CLIMB" then
        -- ...and so is a departure's climb-out: the FMS holds the vertical to CLIMB_AGL, and a
        -- terrain floor has no business inside a profile that is already a climb.
        TAVOID.active = false
        DEP.climbTick()
        broadcastVertical()
    else
        TAVOID.tick()
        broadcastVertical()
    end

    -- Route start: hold throttle at 0 (dynamicSpeedTarget) until the nose swings onto the first
    -- target, then release it for good -- so we never launch off on the old heading and arc round.
    if NAV.aligning then
        local herr = targetHeadingErr()
        if herr and math.abs(herr) <= math.rad(LG.ALIGN_HDG_DEG) then NAV.aligning = false end
        -- ...AND DROP IT OUTRIGHT IN A PHASE WHERE NOTHING IS SWINGING THE NOSE (2026-09-01).
        -- The UAC only steers to the bearing in ENROUTE and APPROACH; at TERMINAL/ARRIVED it
        -- sets desiredHeading = heading and draws a position hold instead. So a plan committed
        -- with its final point ALREADY inside POI_FINAL_RADIUS starts in TERMINAL, the nose
        -- never swings, the heading test above can never come true -- and the hold latched for
        -- good. The aircraft still arrived (the UAC's own APPROACH_KP * dist law flew it in,
        -- which is what a commanded zero cannot stop at TERMINAL), so the only visible symptom
        -- was the HSI reading ALIGN / CMD 0 the whole way in while the engines worked: the
        -- displayed command was simply not the one being flown. Releasing the hold here makes
        -- CMD honest again; it cannot let the craft launch off on the old heading, because at
        -- TERMINAL there is no leg to launch down -- the target is already within 60 m.
        if NAV.phase == "TERMINAL" or NAV.phase == "ARRIVED" then NAV.aligning = false end
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
            -- ...never while an ATC hold is in force: the facility is somebody else's.
            if isFinal and NAV.landAtEnd and not ATC.holding and (NAV.phase == "ARRIVED"
                    or (LND.rwyUsable(NAV.landSite) and dist <= LG.RWY_ARM_M)) then
                DEP.finish()   -- a hop short enough to arrive mid-climb-out: the landing has it
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
            local names = expandCourseSeq(c.seq)
            local dep, len = math.huge, 0
            local first = POINTS[names[1]]
            if first and NAV.haveFix then
                local dx, dz = first.x - NAV.posX, first.z - NAV.posZ
                dep = math.sqrt(dx * dx + dz * dz)
            end
            for i = 2, #names do
                local a, b = POINTS[names[i - 1]], POINTS[names[i]]
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
        -- Flying the published approach is a different thing from being en route -- the FMS has
        -- the vertical channel and the speed is capped -- so it says so, in the landing colour.
        term.setTextColour(LND.appr and colours.orange or colours.lime)
        term.write(string.format("%s%s -> %s  (%.0fm)",
            LND.appr and "APPR " or "", NAV.planName, tgt.name, dist))
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
    for _, pname in ipairs(expandCourseSeq(c.seq)) do
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
-- the mid-route join point, and the auto-land destination. Re-solved as the pilot watches (so
-- the join range stays current) but through RTR.solveCached, at most every SOLVE_TTL -- NOT on
-- every redraw, which is every event this computer sees. FLY commits it, and the commit solves
-- again from the live fix. BACK returns to the picker.
function RTR.drawPlanPreview()
    local w, h = term.getSize()
    term.setBackgroundColour(colours.black)
    term.clear()
    term.setTextColour(colours.orange)
    term.setCursorPos(2, 1); term.write("ROUTE TO " .. (RTR.dest or "?"))

    local path, total, _, err = RTR.solveCached(RTR.dest)
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
    -- ...the slope actually being flown, which on a short strip is steeper than RWY_GS_DEG
    -- (LND.rwyGlide); printing the published figure would hide exactly the number that changed.
    term.write("AUTOLAND " .. LND.name .. (LND.rwy
        and string.format(" RWY %.0fm/s %.1fdeg", LND.appSpeed,
            math.deg(math.atan(LND.gsTan))) or ""))
    term.setCursorPos(math.max(2, w - #LND.phase), 1); term.write(LND.phase)

    -- What the controller has said about this facility, on the row that was free.
    local atcL, atcC = ATC.label()
    if atcL then
        term.setTextColour(atcC)
        term.setCursorPos(2, 2); term.write(atcL:sub(1, w - 2))
    end

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
        PROBE     = "below pad altitude - probing for the surface",
        HOVER     = "NO SURFACE HERE - station keeping on the point",
        LANDED    = "LANDED - lift cut, on the gear. STOP engine NOW",
        -- runway approach
        INTERCEPT = "lining up on the centreline (level)",
        GLIDE     = "on the glideslope",
        ROLLOUT   = "rolling out - braking on the runway",
        MISSED    = "PAST THE TOUCHDOWN ZONE - level. ABORT to go around",
    }
    local hintText = LND.paused and "PILOT ALT-AP -- descent PAUSED" or (hint[LND.phase] or "")
    term.setTextColour((LND.phase == "MISSED" and colours.red)
        or ((LND.phase == "LANDED" or LND.phase == "HOVER" or LND.paused) and colours.lime)
        or colours.orange)
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
    -- Which of a field's published approaches is being flown belongs on this line: it is part of
    -- the plan the pilot committed to and is not derivable from the fix list on its own -- two
    -- approaches to one runway can share most of their fixes.
    local hdr = "ROUTE: " .. (NAV.planName or "--")
    if NAV.apprName then hdr = hdr .. "  VIA " .. NAV.apprName end
    term.write(hdr:sub(1, math.max(1, w - 14)))
    term.setCursorPos(math.max(2, w - 12), 1)
    term.write((DEP.phase ~= "OFF" and DEP.phase) or NAV.phase or "--")

    term.setTextColour(colours.white)
    term.setCursorPos(2, 2)
    -- The controller's word takes this row while it has one: inbound to a field, whether the
    -- aircraft may land there matters more than its coordinates, which the HSI also carries.
    -- A departure in progress outranks it -- the controller only ever talks about the far end.
    local depL, depC = DEP.line()
    local atcL, atcC = ATC.label()
    if depL then
        term.setTextColour(depC)
        term.write(depL:sub(1, w - 2))
    elseif atcL then
        term.setTextColour(atcC)
        term.write(atcL:sub(1, w - 2))
    elseif NAV.haveFix then
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
            -- Fixes belonging to the published approach are labelled as such rather than by
            -- their POI/VOR kind: where the arrival starts is the one thing about the pattern
            -- worth reading off the plan.
            local kindLbl = isFinal and "END"
                or ((NAV.apprFrom and i >= NAV.apprFrom) and "APPR" or pt.kind:upper())
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

    -- Speed selector: enabled (tappable) whenever a plan is being flown. It used to demand
    -- NAV.phase == "ENROUTE" on the grounds that "its cap only bites there", which is not so --
    -- dynamicSpeedTarget applies cruiseCapValue() as `capBase` on every tick and reads the
    -- phase nowhere (see the note on isApproachLeg: APPROACH is a control law, not a speed).
    -- What that gate really did was take the speed selector away from the two plan shapes that
    -- never REACH ENROUTE: a DIRECT-TO is a single-point plan, so isApproachLeg() -- which is
    -- `NAV.index >= total` -- is true from the moment it is committed, and so is a DIRECT-TO-LAND
    -- at a helipad. Both cruise the whole way at SPEED_FULL with the SPD button greyed out.
    -- (A landing owns the speed itself, but that cannot reach here: drawStatus hands over to
    -- drawLandingStatus while LND.active, and that screen has no SPD button at all.)
    local cruising = NAV.active
    button(2, h - 1, 8, 1, "HOME", "HOME", true, colours.brown)
    if ATC.holding then
        -- During an ATC hold the speed selector means nothing (the hold is a stop), so its
        -- place goes to the one decision the pilot may want to take instead: fly this plan as
        -- if there were no controller. Deliberately not CANCEL -- the plan keeps flying.
        button(14, h - 1, 14, 1, "ATCREL", "RELEASE ATC", true, colours.orange)
    else
        button(14, h - 1, 14, 1, "SPD", "SPD " .. cruiseModeLabel(), cruising, colours.lime)
    end
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
    elseif id == "ATCREL" then
        -- Take this plan off the controller's hands: the hold ends and the landing engages
        -- where the plan says, occupied or not. A new plan asks again.
        ATC.release()
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
            -- A name that is not a point may still be a legal REFERENCE -- another course, or a
            -- published approach out of landing.db (see expandCourseSeq). Say which, so a
            -- hand-written `WAU-TMU` or `TMV_APPR4` does not read as a broken waypoint here and
            -- get tapped away.
            local coordStr = (p and string.format("(%d,%d)", p.x, p.z))
                or (COURSES[pname] and "(course)")
                or (LANDING[pname] and LANDING[pname].type == "appr" and "(approach)")
                or "(missing)"
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

-- Redraws on a GATE, not on every event -- 2026-09-17, for "the display flickers a lot".
--
-- It used to redraw on every event, and the comment here said that was what kept POS/DIST live
-- without a timer of our own. It is, and it is also how the screen came to be cleared and
-- repainted ~100 times a second: `parallel.waitForAny` hands every event to every branch, so
-- this loop woke on all of it -- navLoop's 20 Hz sleep, hsiLoop's 20 Hz sleep,
-- apAnnounceLoop's 10 Hz, four `rednet.receive` timeouts, AND every rednet packet this modem
-- hears from anywhere on the fleet. Each wake-up is a full `term.clear()` and repaint, and on
-- the ROUTE PREVIEW screen it was a whole route solve as well (RTR.solveCached, now).
--
-- So: repaint when something actually changed (`dirty`) and otherwise at most every
-- UI_REDRAW_MIN. The ambient event rate is what advances it -- navLoop's own sleep guarantees
-- 20 events a second whatever else is quiet, so a gate that has expired is serviced within a
-- tick and no timer of our own is needed here either.
--
-- `dirty` is what keeps the gate honest about INPUT. A touch may change `screen` outright, and
-- alert/confirm/promptKeyboard draw over us and return, so both must repaint at once rather
-- than up to UI_REDRAW_MIN later -- a button that looks dead for a tenth of a second gets
-- pressed twice.
--
-- The hitboxes deliberately survive a skipped repaint: they are only cleared when the screen is
-- actually redrawn, so the ones being hit-tested always belong to what is on the glass.
local UI_REDRAW_MIN = 0.1
local function uiLoop()
    local lastDraw, dirty = -1, true
    while true do
        if dirty or (os.clock() - lastDraw) >= UI_REDRAW_MIN then
            clearHitboxes()
            drawFns[screen]()
            lastDraw, dirty = os.clock(), false
        end
        local ev = {os.pullEvent()}
        if ev[1] == "term_resize" or ev[1] == "monitor_resize" then dirty = true end
        -- A touch on the HSI MONITOR is not UI input -- pointerXY rejects it -- it is the
        -- instrument's own view toggle. It is handled here because this is the only loop that
        -- pulls events: hsiLoop sleeps, and a second os.pullEvent would be a second consumer
        -- of the same queue. Ignored outright while a landing is engaged, rather than merely
        -- having no visible effect, so that the stored view cannot change underneath the
        -- landing map and surprise the pilot the moment the landing ends.
        if hsiMonName and ev[1] == "monitor_touch" and ev[2] == hsiMonName
                and HSICFG.PLAN.ENABLE and not LND.active then
            CRS.view = (CRS.view == "plan") and "hsi" or "plan"
        end
        local x, y = pointerXY(ev)
        if x then
            local id = hitTest(x, y)
            if id and touchFns[screen] then
                touchFns[screen](id)
                dirty = true     -- it may have changed `screen`, or drawn a dialog over us
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
                    or p == "PROBE" or p == "HOVER"
                    or p == "INTERCEPT" or p == "GLIDE" or p == "MISSED" then
                if not LND.paused then
                    LND.paused = true
                    broadcastVertical()   -- send "release" at once so the UAC/AS react now
                end
            elseif p == "LANDED" then
                cancelNav()
                screen = "home"
            end
        elseif sender and DEP.owns() then
            -- Pilot tapped ALT-AP during a departure: the vertical is theirs at once, with
            -- `release` (it keeps the pick the tap just made), as for a terrain hold. In the
            -- climb-out the route flies on; on the runway the run is given up as well, and the
            -- aircraft leaves under ALT-AP and flies the same path from where it is.
            if DEP.phase == "CLIMB" then DEP.finish("release") else DEP.abandon("release") end
        elseif sender and TAVOID.active then
            -- Pilot tapped ALT-AP during an automatic terrain-avoidance hold: hand the
            -- vertical channel back at once, exactly as a landing override does, and stand
            -- the system down for CFG.TAVOID.OVERRIDE_COOLDOWN even if the corridor still
            -- says the same ground is unsafe -- an override that gets fought at once is not
            -- an override. The MANCON scramble detent and, where fitted, the GPWS remain
            -- armed underneath (see apconf.lua's CFG.TAVOID header).
            TAVOID.active = false
            TAVOID.overrideUntil = os.clock() + CFG.TAVOID.OVERRIDE_COOLDOWN
            rednet.broadcast({ cmd = "release" }, AUTOLAND_PROTOCOL)
        end
    end
end

-- ============================================================================
-- EXTERNAL MONITOR -- HSI, in THREE views (drawHSI dispatches):
--   * COURSE (drawCourseHSI): the classic heading-up VOR/HSI rose with a course-deviation
--     needle for the active leg, over the terrain overlay.
--   * PLAN (drawPlanHSI): the route map -- the baked path itself, the waypoints on it, an
--     autozoom, and the same course deviation collapsed onto one row. No terrain.
--   * LANDING (drawLandingHSI): a heading-up top-down moving map (aircraft fixed nose-up at
--     centre, the pad/target rotating under it, a heading-error bar on top, numbers on the
--     bottom) -- shown once an autoland/landing engages, for flying it down by hand or
--     watching the autoland.
--
-- WHICH ONE IS UP is two decisions, not one, and they are not peers. COURSE and PLAN are the
-- PILOT'S choice, toggled by touching the monitor (see uiLoop). LANDING is the FMS's
-- OVERRIDE: while LND.active it is the only thing this instrument will show, and the touch
-- is ignored rather than deferred. Same shape as the altitude selector's NORMAL/OVER machine.
--
-- Display-only; reads NAV/LND (filled every tick). Idles if no monitor is attached.
-- ============================================================================
-- HSICFG IS CFG.HSI, bound at the top of this file with the rest of the configuration.
--
-- There used to be a second `local HSICFG = {...}` right here, left behind when these
-- constants were moved out to apconf.lua, and it SHADOWED the one the config file fills in
-- -- so for as long as it existed apconf's CFG.HSI was dead and every edit to it silently
-- did nothing. It was found (2026-09-16) by RANGES having quietly drifted apart between
-- airframes' apconf.lua with no effect on any of them. Deleted; do not reintroduce a copy of
-- a conf table into the program, for exactly the reason the loader block up top gives.

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

-- ============================================================================
-- TERRAIN OVERLAY -- the ground, painted underneath both HSI modes
-- ============================================================================
-- The client for the #68 terrain data server, and the raster it paints inside the HSI's
-- ring. Both modes use it: the course rose gets a ground picture out to the range it can
-- reach in CFG.TERR.COURSE_LOOK seconds, the landing map gets one at its own map range.
--
-- IT IS A DISPLAY AND ONLY A DISPLAY. Nothing here reads back into the navigation tick, the
-- speed law or the landing state machine: the FMS flies exactly the same path with the
-- overlay on, off, or with the server unplugged. The alerting half of this database is the
-- GPWS's (#67) job and is deliberately somewhere else -- an instrument that both paints the
-- ground and decides what to do about it is one program with two reasons to be wrong.
--
-- WHAT IT ASKS FOR. One "map" query a second: a north-up, axis-aligned grid of cells around
-- the aircraft, each cell reduced server-side to the worst chunk under it. North-up rather
-- than heading-up is what lets one grid survive a whole turn -- the rotation into the
-- instrument's heading-up frame happens here, per repaint, and costs two multiply-adds a
-- cell. The reply is re-read against LIVE altitude on every rebuild, so a descent recolours
-- the ground immediately instead of waiting for the next radio answer.
--
-- COLOUR IS PURELY ALTITUDE, and it is a RAMP. See apconf.lua's CFG.TERR header for the
-- bands and the palette. The short of it: the five slots from dark green up to red are one
-- continuous scale of "how much room is there under us", hottest nearest -- and because this
-- world's obstacles FLOAT, magenta and purple are off that scale entirely, for a mass we are
-- passing UNDERNEATH, where "how far below its belly are we" is the question and colouring
-- it red would be a lie.
--
-- HOW IT IS DRAWN. Every painted cell is a space with a background colour, so a whole row is
-- one monitor.blit of three strings instead of forty setCursorPos/write pairs. The rows are
-- rebuilt at CFG.TERR.REDRAW and re-blitted at the instrument's own 20 Hz; rebuilding the
-- classification every frame would be tens of thousands of chunk decodes a second for a
-- picture that cannot meaningfully change in 50 ms. The thinned bands are thinned by
-- SKIPPING cells (a checkerboard), not by using a pattern glyph, which is what keeps the
-- character and foreground rows constant strings.
local TERR = {
    F = CFG.TERR.FLAG,
    srv = nil, geo = nil, rev = nil, heard = nil, fault = nil,
    grid = nil,              -- { d, x0, z0, cell, nx, nz, rec, t }
    reqId = 0, asked = 0, got = 0, reqAt = 0, reqRange = nil,
    range = nil,             -- the range the instrument last asked the overlay to cover,
                             -- nil when the overlay is off -- which is also what stops the
                             -- radio traffic when there is no monitor to paint.
    rows = nil, rowTop = 0, rowBot = -1, builtAt = 0, dirty = true,
    w = nil, chdg = 0,       -- the frame TERR.build works in. These sit here rather than in
                             -- its argument list purely to keep that function's local count
                             -- down -- see the ceiling note on TERR.at.
    hx = 0, hz = 0, vx = 0, vz = 0,  -- world vectors to half a subpixel across and down,
                             -- set once per build; the six samples are the cell centre plus
                             -- and minus these.
    sub = false,             -- whether this build is drawing 6 samples a cell or 1
    memo = nil,              -- grid index -> band*8+lvl, or -1 for "nothing here". One pass
                             -- per build: six subpixels of a screen cell very often land in
                             -- the same grid cell, and classifying it once is the difference
                             -- between six times the work and about one and a half.
    sb = {}, sl = {},        -- the six samples, reused rather than reallocated per cell
    BIT = { 1, 2, 4, 8, 16, 32 },
    SX = nil, SW = nil,      -- glyph for an arbitrary 6-bit subpixel mask, and whether it
                             -- swaps its two colours
    PC = nil, PS = nil,      -- the subpixel glyphs, and whether each swaps its two colours.
                             -- All three blit rows vary now that cells are dithered; the
                             -- constant blank/fgrow rows this used to keep are gone.
    worst = -1,              -- 3 warning / 2 caution / 1 context / 0 all clear / -1 nothing
                             -- charted at all. Sorting those last two apart is the whole
                             -- reason band() answers 0 rather than nil for clear ground.
    C = {}, B = {}, D = {},  -- band -> colour, blit char, fill density
    canBlit = false,
}
-- CC's blit digit for a colour: the hex of its bit position. Computed rather than taken from
-- colours.toBlit so this does not depend on which CC version is installed -- and given a name
-- of its own because the mapping is a TRAP. The digits are ordered by bit, so "0" is WHITE
-- and BLACK is "f". The first cut of this instrument defaulted an unpainted cell to "0" and
-- painted the sky white; TERR.blk exists so that number is written down exactly once.
function TERR.blitChar(c)
    local n, v = 0, 1
    while v < c and n < 15 do v, n = v * 2, n + 1 end
    return ("0123456789abcdef"):sub(n + 1, n + 1)
end

do
    -- Band indices, used everywhere below and ordered worst-first: 1 WARN, 2 CAUTION,
    -- then the heightmap from its highest step down to its lowest (3 TINT4 .. 6 TINT1),
    -- 7 WATER, 8 OVER, 9 OVER_LO. The tint runs high-to-low so that "band N dithers toward
    -- band N-1" walks UP the heightmap, and so that the tint's top step (3) has the caution
    -- band above it in the numbering without ever blending into it. Resolved once, by NAME,
    -- exactly as CFG.PALETTE is -- so the config file never has to know `colours` exists.
    for i, k in ipairs({ "WARN", "CAUTION", "TINT4", "TINT3", "TINT2", "TINT1",
            "WATER", "OVER", "OVER_LO" }) do
        local c = colours[CFG.TERR.COL[k]]
        if c == nil then
            error("apconf.lua's CFG.TERR.COL." .. k .. " is not a colour name: "
                .. tostring(CFG.TERR.COL[k]), 0)
        end
        if CFG.TERR.DENSITY[k] == nil then
            error("apconf.lua's CFG.TERR.DENSITY is missing " .. k .. ".", 0)
        end
        TERR.C[i], TERR.D[i] = c, CFG.TERR.DENSITY[k]
        TERR.B[i] = TERR.blitChar(c)
    end
    TERR.blk = TERR.blitChar(colours.black)
    TERR.canBlit = hsiMon ~= nil and type(hsiMon.blit) == "function"

    -- The 2x3 subpixel glyphs, indexed by how many of a cell's six pixels carry the HOTTER
    -- of the two colours being blended. CC packs the thirty-two combinations of five pixels
    -- into characters 128..159 and leaves the sixth (bottom right) always background, so a
    -- density above half is drawn as its complement with the two colours SWAPPED -- which is
    -- what PS records. Bit order is TL, TR, ML, MR, BL: \129 is one pixel (TL), \137 two on
    -- a diagonal (TL+MR), \153 three spread corner to corner (TL+MR+BL). Spread rather than
    -- clustered, so a half-tone reads as a tone and not as a little picture.
    TERR.PC = { "\129", "\137", "\153", "\137", "\129", " " }
    TERR.PS = { false,   false,   false,   true,    true,    true }

    -- ...and the same glyphs addressed as a BITMAP rather than as a tone: SX[m] draws the
    -- 6-bit mask m, where bit 1 is the top-left subpixel and bit 32 the bottom-right:
    --
    --      +-----+
    --      | 1 2 |     bits 1 and 2
    --      | 3 4 |     bits 4 and 8
    --      | 5 6 |     bits 16 and 32
    --      +-----+
    --
    -- CC gives 31 characters, 129..159, for the 31 non-empty masks that leave the BOTTOM
    -- RIGHT subpixel clear. Anything with that pixel set is drawn as its complement with the
    -- two colours swapped, which is what SW records -- and 128 and 160 are unrelated glyphs,
    -- so the empty and full masks are a plain space instead.
    TERR.SX, TERR.SW = {}, {}
    for m = 0, 63 do
        if m == 0 then TERR.SX[m], TERR.SW[m] = " ", false
        elseif m == 63 then TERR.SX[m], TERR.SW[m] = " ", true
        elseif m < 32 then TERR.SX[m], TERR.SW[m] = string.char(128 + m), false
        else TERR.SX[m], TERR.SW[m] = string.char(128 + (63 - m)), true end
    end
end

-- Where in its band a clearance sits, as a count of subpixels (0..6) to give the NEXT HOTTER
-- colour. 0 leaves the cell a flat block of its own band -- which is what CFG.TERR.DITHER = 0
-- forces everywhere, restoring the undithered display exactly.
function TERR.mix(v, lo, hi)
    local d = CFG.TERR.DITHER
    if d <= 0 or hi <= lo then return 0 end
    local f = (hi - v) / (hi - lo)
    if f < 0 then f = 0 elseif f > 1 then f = 1 end
    -- Quantise to d steps first, then scale to the glyphs' six, so a lower DITHER gives
    -- genuinely coarser shading rather than the same shading computed differently.
    return math.floor(math.floor(f * d + 0.5) * 6 / d + 0.5)
end

function TERR.available()
    return CFG.TERR.ENABLE == true and TERR.geo ~= nil and TERR.heard ~= nil
       and (os.clock() - TERR.heard) <= CFG.TERR.STALE
end

-- A terrain picture that is merely OLD is worse than none at all, because it looks exactly
-- like a current one. Past GRID_STALE it comes off the screen and the annunciator says so.
function TERR.haveGrid()
    return TERR.grid ~= nil and (os.clock() - TERR.grid.t) <= CFG.TERR.GRID_STALE
end

-- One four-byte record -> which colour band it falls in against altitude `alt`, or nil for
-- "paint nothing here" (never surveyed, or nothing within CONTEXT_M of us).
--
-- The order of the tests IS the logic. A floating mass whose vertical span we are inside --
-- or within WARN_M of either face of -- is settled first and is always red: at that point it
-- is not a clearance question, it is the aeroplane and the island trying to occupy the same
-- air, and which side of us it started on has stopped mattering. Only after that test can a
-- mass be called unambiguously below us (where it is terrain, and coloured as terrain) or
-- unambiguously above us (where the question is the gap under its belly).
--
-- With both a floor and a ceiling in the same chunk -- a valley under an island, which this
-- world genuinely has -- the SMALLER gap wins and its family picks the colour. So threading
-- such a gap paints yellow while the ground is the tighter squeeze and magenta once the
-- ceiling is, which is the honest reading of where the aeroplane's room actually is.
function TERR.band(a, b, c, f, alt)
    if f == nil then return nil end
    local F, g, K = TERR.F, TERR.geo, CFG.TERR
    if math.floor(f / F.NODATA) % 2 == 1 then return nil end

    -- Solid tops are read at the TOP of their level and undersides at the BASE of theirs --
    -- the same rounding the server and the builder use, and in both cases the direction that
    -- makes the gap look smaller than it is.
    local top = (a ~= 255) and (g.minY + g.cell * a + g.cell - 1) or nil
    local oy, ob
    if b ~= 255 and c ~= 255 and (f % (F.MASS + 1)) >= K.MIN_MASS then
        oy, ob = g.minY + g.cell * b + g.cell - 1, g.minY + g.cell * c
    end
    -- Inside a floating mass, or within WARN_M of either of its faces. Not a clearance
    -- question at all: it is the aeroplane and the island trying to occupy the same air, and
    -- which side of us it started on has stopped mattering.
    if oy and alt <= oy + K.WARN_M and alt >= ob - K.WARN_M then return 1, 0 end

    local water = (math.floor(f / F.WATER) % 2 == 1) and top ~= nil
    if oy and oy < alt and (not top or oy > top) then top, water = oy, false end
    local cb = top and (alt - top) or nil                 -- gap down to the nearest floor
    local ca = (ob and ob > alt) and (ob - alt) or nil     -- ...and up to the nearest ceiling
    -- Water is never a terrain hazard, at any height -- see the note in apconf.lua. Taking it
    -- out of cb here is what keeps the sea navy at thirty metres as well as at three hundred.
    if water then cb = nil end

    -- ---- the hazard layer, RELATIVE to us, which wins wherever it applies ----------------
    if cb and cb <= K.WARN_M then return 1, 0 end
    if (cb and cb <= K.CAUTION_M) or (ca and ca <= K.CAUTION_M) then
        -- The tighter of the two gaps decides, and because a floor and a ceiling are not the
        -- same kind of thing, it also decides which colour family says so.
        if cb and (not ca or cb <= ca) then return 2, TERR.mix(cb, K.WARN_M, K.CAUTION_M) end
        return 8, 0
    end
    -- A ceiling that is there but is not a factor: the same magenta, thinned toward black by
    -- how far overhead it is, until at CONTEXT_M it stops being drawn at all. That fade is
    -- what let OVER and OVER_LO share one palette slot, which is what paid for a tint step.
    if ca and ca <= K.CONTEXT_M then
        return 9, TERR.mix(K.CONTEXT_M - ca, 0, K.CONTEXT_M)
    end
    if water then return 7, 0 end

    -- ---- the heightmap, ABSOLUTE, which paints everything else ---------------------------
    -- No clearance test here on purpose: all charted ground is tinted, however far below it
    -- is, so black on this display means "never surveyed" and nothing else. What the colour
    -- says is what the LAND is, not where we are.
    if not top then return 0, 0 end
    local s = (top - K.TINT_LO_Y) / (K.TINT_HI_Y - K.TINT_LO_Y)
    if s < 0 then s = 0 elseif s > 1 then s = 1 end
    s = s * 3                                    -- 0..3, across the four tint steps
    local i = math.floor(s)
    if i >= 3 then return 3, 0 end               -- the top step; nothing above it to blend to
    -- 1 - frac, because mix() counts UP as its argument falls toward `lo`, and here the blend
    -- must grow as the ground rises through its step.
    return 6 - i, TERR.mix(1 - (s - i), 0, 1)
end

-- One GRID CELL, classified and remembered for the rest of this build. The memo is what
-- makes subpixel drawing affordable: six subpixels of a character very often fall inside one
-- grid cell, and the classification is the expensive half.
--
-- It also hides the two wire formats from everything above it. Packed, a cell is one byte --
-- 255 never surveyed, level in bits 0-5 (63 = charted but no ground), water in bit 6 -- with
-- floating masses in a sparse side table; unpacked, it is the original four records. Either
-- way TERR.band sees the same four numbers it always has.
function TERR.at(gi)
    local v = TERR.memo[gi]
    if v then
        if v < 0 then return nil end
        return math.floor(v / 8), v % 8
    end
    local g, band, lvl = TERR.grid, nil, nil
    if g.pack then
        local byte = g.d:byte(gi + 1)
        if byte == nil or byte == 255 then TERR.memo[gi] = -1 return nil end
        local lev, wet = byte % 64, (byte >= 64) and TERR.F.WATER or 0
        local o = g.ob[gi]
        if o then
            band, lvl = TERR.band(lev == 63 and 255 or lev, o % 256,
                math.floor(o / 256) % 256, math.floor(o / 65536) + wet, NAV.posY)
        else
            band, lvl = TERR.band(lev == 63 and 255 or lev, 255, 255, wet, NAV.posY)
        end
    else
        local o = gi * g.rec + 1
        local ra, rb, rc, rf = g.d:byte(o, o + 3)
        band, lvl = TERR.band(ra, rb, rc, rf, NAV.posY)
    end
    if not band then TERR.memo[gi] = -1 return nil end
    TERR.memo[gi] = band * 8 + lvl
    return band, lvl
end

-- ...and one WORLD POINT, via the grid it falls in. nil when it falls outside the grid.
function TERR.pt(wx, wz)
    local g = TERR.grid
    local i = math.floor((wx - g.x0) / g.cell)
    if i < 0 or i >= g.nx then return nil end
    local j = math.floor((wz - g.z0) / g.cell)
    if j < 0 or j >= g.nz then return nil end
    return TERR.at(j * g.nx + i)
end

-- One SCREEN CELL: six terrain samples in, one character with two colours out.
--
-- THE WHOLE PROBLEM IS THAT A CELL HOLDS TWO COLOURS. Six samples have to be quantised down
-- to two, and which two is a safety decision, not an aesthetic one:
--
--   A is the MOST SEVERE band present -- the lowest index, red before amber before ground.
--     A hazard subpixel is never the one that gets quantised away.
--   B is the least severe present, so the pair brackets the range and everything between
--     maps to whichever it is nearer, ties going to A.
--   Unsurveyed ground, if any of the six is unsurveyed, takes B outright and everything else
--     takes A. Painting "no data" as terrain would be inventing ground, and painting terrain
--     as "no data" would be hiding it; keeping them separate costs one subpixel of nuance.
--
-- WHERE ALL SIX AGREE there is no boundary to draw, so the six pixels are spent on DITHER
-- instead -- shading the cell by where it sits inside its band. Detail where the ground
-- varies, gradation where it does not, and the two never compete for the same pixels.
function TERR.cell(wx, wz, x, y)
    local S, L = TERR.sb, TERR.sl
    if TERR.sub then
        S[1], L[1] = TERR.pt(wx - TERR.hx + TERR.vx, wz - TERR.hz + TERR.vz)
        S[2], L[2] = TERR.pt(wx + TERR.hx + TERR.vx, wz + TERR.hz + TERR.vz)
        S[3], L[3] = TERR.pt(wx - TERR.hx,           wz - TERR.hz)
        S[4], L[4] = TERR.pt(wx + TERR.hx,           wz + TERR.hz)
        S[5], L[5] = TERR.pt(wx - TERR.hx - TERR.vx, wz - TERR.hz - TERR.vz)
        S[6], L[6] = TERR.pt(wx + TERR.hx - TERR.vx, wz + TERR.hz - TERR.vz)
    else
        local b, l = TERR.pt(wx, wz)
        for k = 1, 6 do S[k], L[k] = b, l end
    end

    local a, b0, bg, same = nil, nil, false, true
    for k = 1, 6 do
        local c = S[k]
        if c and c > 0 then
            if not a or c < a then a = c end
            if not b0 or c > b0 then b0 = c end
            local sev = (c == 1) and 3 or ((c == 2 or c == 8) and 2 or 1)
            if sev > TERR.worst then TERR.worst = sev end
        else
            bg = true
            if TERR.worst < 0 and c == 0 then TERR.worst = 0 end
        end
        if S[k] ~= S[1] then same = false end
    end
    if not a then return " ", TERR.blk, TERR.blk end          -- nothing to paint at all

    -- Thinning, if any band asks for it. Applied to the whole character, since a cell has
    -- one background and cannot be half-thinned.
    local d = TERR.D[a]
    if not (d == 1 or (d == 2 and (x + y) % 2 == 0)
                   or (d == 3 and x % 2 == 0 and y % 2 == 0)) then
        return " ", TERR.blk, TERR.blk
    end

    if same then
        -- All six agree: spend the pixels on tone instead of on detail.
        local lvl = L[1]
        if lvl > 0 and a > 1 then
            local hot = (a == 9) and TERR.blk or TERR.B[a - 1]
            if TERR.PS[lvl] then return TERR.PC[lvl], TERR.B[a], hot end
            return TERR.PC[lvl], hot, TERR.B[a]
        end
        return " ", TERR.blk, TERR.B[a]
    end

    if bg then b0 = 0 end
    local m = 0
    for k = 1, 6 do
        local c = S[k] or 0
        local toA
        if bg then toA = (c > 0)
        elseif c == a then toA = true
        else toA = (c - a) <= (b0 - c) end
        if toA then m = m + TERR.BIT[k] end
    end
    local ca = TERR.B[a]
    local cb = (b0 == 0) and TERR.blk or TERR.B[b0]
    if TERR.SW[m] then return TERR.SX[m], cb, ca end
    return TERR.SX[m], ca, cb
end

-- Rebuild the raster: one background-colour string per screen row of the map area, ready to
-- blit. Everything the inner loop needs is hoisted out of it first -- this walks about
-- fourteen hundred cells and is the only expensive thing the instrument does.
-- It is kept deliberately spare on locals -- everything per-cell is in TERR.cell and the cold
-- half of the frame (width, row span, heading) is on TERR -- because of the same Cobalt
-- ceiling described there. Measured 2026-08-21: this compiles with the main chunk's locals
-- up to 175 and fails at 176, so with the chunk at 164 there are ELEVEN in hand -- and that
-- is the budget for new top-level locals in this WHOLE FILE, not just for anything added
-- here. DO NOT hoist TERR.B / TERR.D / NAV.posX
-- into locals here "for speed", and do not put the frame back in the argument list: either
-- change costs the margin, and the failure is not a slow instrument, it is the whole FMS
-- refusing to boot.
function TERR.build(cx, cy, Rx, Ry, range)
    -- Screen up is the nose. One metre up the screen, and one metre across it to the right,
    -- as world vectors: the compass-to-world map is the inverse of toCompass, and "right" is
    -- just the nose turned ninety degrees clockwise, so it costs no second pair of trig.
    local fx, fz, rx, rz
    do
        local phi = math.rad(180 - TERR.chdg)
        fx, fz = math.sin(phi), math.cos(phi)
        rx, rz = -fz, fx
    end
    -- Half a subpixel across and down, as world vectors. A character is 1/Rx of the ring
    -- wide and 1/Ry tall, and it holds 2x3 subpixels, so the sample offsets are a QUARTER of
    -- a cell horizontally and a THIRD vertically -- which lands each sample in the middle of
    -- its own subpixel rather than on the seam between two.
    TERR.sub = CFG.TERR.SUBPIXEL == true
    if TERR.sub then
        local du, dv = (0.25 / Rx) * range, (1 / 3 / Ry) * range
        TERR.hx, TERR.hz = rx * du, rz * du
        TERR.vx, TERR.vz = fx * dv, fz * dv
    end
    -- -1 is "not one charted cell on the screen", 0 is "all charted and all of it clear".
    -- TERR.cell raises it as it goes.
    local rows = {}
    TERR.worst = -1
    TERR.memo = {}

    for y = TERR.rowTop, TERR.rowBot do
        local v = (cy - y) / Ry
        local bx = NAV.posX + fx * v * range           -- this row's centre, in the world
        local bz = NAV.posZ + fz * v * range
        local tc, tf, tb = {}, {}, {}
        for x = 1, TERR.w do
            local u = (x - cx) / Rx
            if u * u + v * v <= 1 then                 -- inside the ring, and only there
                tc[x], tf[x], tb[x] =
                    TERR.cell(bx + rx * u * range, bz + rz * u * range, x, y)
            else
                tc[x], tf[x], tb[x] = " ", TERR.blk, TERR.blk
            end
        end
        rows[y] = { table.concat(tc), table.concat(tf), table.concat(tb) }
    end

    TERR.rows = rows
    TERR.builtAt, TERR.dirty = os.clock(), false
end

-- Paint the overlay for one instrument frame, and tell the radio loop what range to ask
-- about. Called FIRST by both draw modes, straight after the clear, so everything else is
-- drawn on top of the ground rather than under it.
function TERR.paint(W, cx, cy, Rx, Ry, top, bot, range, chdg)
    TERR.range = nil
    if not (CFG.TERR.ENABLE and TERR.canBlit) then return end
    -- No fix or no heading and the picture would be a rotation of nothing about nowhere.
    if not (NAV.haveFix and NAV.haveHeading) then return end
    -- Below MIN_RANGE the map is framing a landing pad, not a landscape: at that scale the
    -- overlay would be a solid red screen saying only that we are about to touch the ground,
    -- which is the one thing the pilot already knows.
    if range < CFG.TERR.MIN_RANGE then return end
    TERR.range = range

    if not (TERR.available() and TERR.haveGrid()) then
        TERR.rows, TERR.worst = nil, -1
        return
    end
    if TERR.dirty or TERR.rows == nil or TERR.w ~= W or TERR.rowTop ~= top
            or TERR.rowBot ~= bot or (os.clock() - TERR.builtAt) >= CFG.TERR.REDRAW then
        TERR.w, TERR.rowTop, TERR.rowBot, TERR.chdg = W, top, bot, chdg
        TERR.build(cx, cy, Rx, Ry, range)
    end
    for y = TERR.rowTop, TERR.rowBot do
        local r = TERR.rows[y]
        if r then
            hsiMon.setCursorPos(1, y)
            hsiMon.blit(r[1], r[2], r[3])
        end
    end
end

-- The annunciator. A terrain display that has quietly stopped being a terrain display looks
-- identical to one over flat ground, so this says which it is -- and, when there IS data,
-- takes the colour of the worst thing on the screen, so the pilot's eye is pulled to the
-- picture rather than having to audit it.
function TERR.label()
    if not CFG.TERR.ENABLE then return nil end
    if not TERR.canBlit then return "TERR N/A", colours.grey end
    if TERR.range == nil then return nil end            -- deliberately off, not broken
    if not TERR.heard then return "TERR NO SRV", colours.grey end
    if not TERR.available() then return "TERR STALE", colours.orange end
    if not TERR.haveGrid() then return "TERR WAIT", colours.grey end
    if TERR.worst < 0 then return "TERR NODATA", colours.grey end
    -- The word takes the colour of the worst thing on the screen, off the ramp itself:
    -- band 1 for a warning, band 3 (the amber caution step) for a caution.
    if TERR.worst >= 3 then return "TERR", TERR.C[1] end
    if TERR.worst == 2 then return "TERR", TERR.C[3] end
    return "TERR", colours.lime
end

-- ---- talking to #68 --------------------------------------------------------------------

function TERR.request()
    local range = TERR.range
    if not (range and NAV.haveFix) then return end
    -- Cover the ring, plus MARGIN for the ground the aircraft will have flown over by the
    -- time the answer is being painted. The cell size falls out of that coverage and the
    -- grid size: there is no point asking for cells finer than the database's own chunks.
    local cell = clamp((2 * range * CFG.TERR.MARGIN) / CFG.TERR.GRID,
                       CFG.TERR.CELL_MIN, CFG.TERR.CELL_MAX)
    TERR.reqId = TERR.reqId + 1
    TERR.asked = TERR.asked + 1
    TERR.reqRange = range
    local m = { t = "map", id = TERR.reqId, x = NAV.posX, z = NAV.posZ,
                cell = math.floor(cell + 0.5), nx = CFG.TERR.GRID, nz = CFG.TERR.GRID,
                sub = CFG.TERR.SUB, pack = CFG.TERR.PACK and 1 or nil,
                alt = NAV.posY, callsign = CFG.TERR.CALLSIGN }
    if TERR.srv then rednet.send(TERR.srv, m, CFG.TERR.PROTOCOL)
    else rednet.broadcast(m, CFG.TERR.PROTOCOL) end
end

function TERR.onMsg(sender, msg)
    -- srv marks a message as coming FROM the server. Without that test another aircraft's
    -- request, which lands here on the same broadcast protocol, would be latched as the
    -- server and every question would go to an aeroplane.
    if type(msg) ~= "table" or not msg.srv then return end
    TERR.srv, TERR.heard = sender, os.clock()

    if msg.t == "beacon" then
        local g = msg.geo
        if type(g) == "table" and g.minY and g.cell and g.levels and g.record then
            TERR.geo, TERR.rev, TERR.fault = g, msg.rev, nil
        else
            TERR.fault = "server sent no geometry"
        end

    elseif msg.t == "map" then
        -- Only the answer to the question we are actually asking. A rescale sends a fresh
        -- query at once (see TERR.loop), and the previous grid -- cut for a different ring --
        -- may still be in the air behind it; painting that one would put the ground at the
        -- wrong scale for a second, which is worse than painting nothing.
        local rec = msg.pack and 1 or (TERR.geo and TERR.geo.record)
        if msg.id ~= TERR.reqId then return end
        if type(msg.grid) == "string" and rec and tonumber(msg.nx) and tonumber(msg.nz)
                and tonumber(msg.x0) and tonumber(msg.z0)
                and tonumber(msg.cell) and msg.cell > 0
                and #msg.grid >= msg.nx * msg.nz * rec then
            local G = { d = msg.grid, x0 = msg.x0, z0 = msg.z0, cell = msg.cell,
                        nx = msg.nx, nz = msg.nz, rec = rec, pack = msg.pack and true or false,
                        ob = nil, t = os.clock() }
            if G.pack then
                -- The sparse obstacle list, unpacked into a lookup once per grid rather than
                -- re-scanned per cell: five bytes each, and only the 1-2% of cells that
                -- carry a floating mass appear at all. Packed three-into-one so the table
                -- holds numbers instead of tables -- a few hundred entries, no garbage.
                G.ob = {}
                local s = msg.obs
                if type(s) == "string" then
                    for k = 1, #s - 4, 5 do
                        local i1, i2, ot, ob, ms = s:byte(k, k + 4)
                        G.ob[i1 + i2 * 256] = ot + ob * 256 + ms * 65536
                    end
                end
            end
            TERR.grid = G
            TERR.got, TERR.dirty = TERR.got + 1, true
        end
    end
end

-- ============================================================================
-- TERRAIN AVOIDANCE -- the client and the decision, kept apart from TERR above on purpose.
-- TERR's own header says it is a display and "nothing else"; this is the one place that reads
-- the terrain database and acts on it (see apconf.lua's CFG.TAVOID for the full design note).
-- It shares TERR.srv/TERR.geo -- the same server, the same beacon, no reason to negotiate
-- twice -- but keeps its own cached corridor and its own state: nothing here ever writes to
-- TERR.grid or anything the instrument reads, and nothing in TERR ever reads TAVOID.
-- ============================================================================

function TAVOID.available()
    return CFG.TAVOID.ENABLE == true and TERR.geo ~= nil and TERR.heard ~= nil
       and (os.clock() - TERR.heard) <= CFG.TAVOID.STALE
end

-- Ask #68 for a corridor along the current ground track (or the nose, below MIN_GS). Cut at
-- the FMS's own next waypoint plus LEG_MARGIN so a straight-line corridor never claims to
-- know about ground the plan turns away from before reaching -- see apconf.lua's header.
function TAVOID.request()
    if not (CFG.TAVOID.ENABLE and NAV.haveFix) then return end
    local dx, dz
    if (NAV.groundSpeed or 0) > CFG.TAVOID.MIN_GS then
        local mag = math.sqrt(NAV.velX * NAV.velX + NAV.velZ * NAV.velZ)
        dx, dz = NAV.velX / mag, NAV.velZ / mag
    else
        -- Too slow for the ground track to mean anything: look where the nose points -- the
        -- same sin/cos pair LND.goAround already uses to turn NAV.heading into a world vector.
        dx, dz = math.sin(NAV.heading), math.cos(NAV.heading)
    end

    local step  = CFG.TAVOID.STEP
    local reach = math.max(CFG.TAVOID.MIN_RANGE,
        (NAV.groundSpeed or 0) * CFG.TAVOID.LOOK_AHEAD_TIME * CFG.TAVOID.RANGE_MARGIN)
    local tgt = NAV.active and NAV.points[NAV.index]
    if tgt then
        local ldx, ldz = tgt.x - NAV.posX, tgt.z - NAV.posZ
        local along = ldx * dx + ldz * dz    -- distance to the target, PROJECTED onto the track
        if along > 0 then reach = math.min(reach, along + CFG.TAVOID.LEG_MARGIN) end
    end
    local n = math.min(CFG.TAVOID.MAX_SAMPLES, math.floor(reach / step) + 2)

    TAVOID.reqId = TAVOID.reqId + 1
    TAVOID.asked = TAVOID.asked + 1
    local m = { t = "corr", id = TAVOID.reqId, x = NAV.posX, z = NAV.posZ, dx = dx, dz = dz,
                step = step, n = n, w = CFG.TAVOID.WIDTH, alt = NAV.posY, callsign = CFG.TERR.CALLSIGN }
    if TERR.srv then rednet.send(TERR.srv, m, CFG.TERR.PROTOCOL)
    else rednet.broadcast(m, CFG.TERR.PROTOCOL) end
end

-- The corridor reply. Called on EVERY message TERR.loop hears (see there for why: two
-- independent rednet.receive calls on the same protocol would race each other for the same
-- messages) and ignores everything that is not a fresh corridor answer.
function TAVOID.onMsg(sender, msg)
    if type(msg) ~= "table" or not msg.srv or msg.t ~= "corr" then return end
    if type(msg.prof) == "string" and TERR.geo and tonumber(msg.n)
            and #msg.prof >= msg.n * TERR.geo.record then
        TAVOID.prof = { x = msg.x, z = msg.z, dx = msg.dx, dz = msg.dz,
                        step = msg.step, n = msg.n, d = msg.prof, t = os.clock() }
        TAVOID.served = TAVOID.served + 1
    end
end

-- The one question this module exists to answer: the highest (charted top + CLEAR_M) of
-- anything in the cached corridor window, or nil if the window is charted and clear. The
-- FIRST return, `ok`, is what makes the silence honest -- see apconf.lua's CFG.TAVOID header
-- on why "don't know" must change nothing: false means the corridor is missing, stale, off
-- the current track, or covers nothing charted at all, and TAVOID.tick() must not act on a
-- false `ok` either way, whether that leaves it disengaged or holding where it already was.
function TAVOID.requiredAlt()
    if not TAVOID.available() then return false end
    local p = TAVOID.prof
    if not p then return false end
    if os.clock() - p.t > CFG.TAVOID.PROF_STALE then return false end

    -- Track alignment by cosine, not by an angle-wrap this file has no helper for: how far the
    -- current track has swung since the corridor was cut, against CFG.TAVOID.TRACK_COS_MIN.
    local tvx, tvz
    if (NAV.groundSpeed or 0) > CFG.TAVOID.MIN_GS then tvx, tvz = NAV.velX, NAV.velZ
    else tvx, tvz = math.sin(NAV.heading), math.cos(NAV.heading) end
    local tm = math.sqrt(tvx * tvx + tvz * tvz)
    if tm > 1e-6 then
        local cosErr = (tvx * p.dx + tvz * p.dz) / tm
        if cosErr < CFG.TAVOID.TRACK_COS_MIN then return false end
    end

    local d = (NAV.posX - p.x) * p.dx + (NAV.posZ - p.z) * p.dz
    if d < -CFG.TAVOID.BEHIND then return false end
    local i0 = math.floor(math.max(0, d) / p.step)
    if i0 >= p.n then return false end

    -- Cut at the FMS's own next waypoint plus LEG_MARGIN, same reasoning as TAVOID.request --
    -- re-derived here in case the target changed since the cached profile was cut.
    local last = p.n - 1
    local tgt = NAV.active and NAV.points[NAV.index]
    if tgt then
        local ldx, ldz = tgt.x - p.x, tgt.z - p.z
        local along = ldx * p.dx + ldz * p.dz
        local cap = math.floor((along + CFG.TAVOID.LEG_MARGIN) / p.step)
        if cap < last then last = cap end
    end
    if last < i0 then return false end

    local rec, F = TERR.geo.record, CFG.TAVOID.FLAG
    local need, seen = nil, 0
    for i = i0, last do
        local o = i * rec + 1
        local terr, otop, obas, f = p.d:byte(o, o + 3)
        if f and math.floor(f / F.NODATA) % 2 == 0 then
            seen = seen + 1
            local top = nil
            if terr ~= 255 then top = TERR.geo.minY + TERR.geo.cell * terr + (TERR.geo.cell - 1) end
            if otop ~= 255 then
                local mass = f % (F.MASS + 1)
                if mass >= CFG.TAVOID.MIN_MASS then
                    local oy = TERR.geo.minY + TERR.geo.cell * otop + (TERR.geo.cell - 1)
                    if not top or oy > top then top = oy end
                end
            end
            if top then
                local want = top + CFG.TAVOID.CLEAR_M
                if not need or want > need then need = want end
            end
        end
    end
    if seen == 0 then return false end   -- nothing charted anywhere in the window: don't know
    return true, need
end

-- Called once a navTick, only while not landing and not flying a go-around (see navTick). See
-- apconf.lua's CFG.TAVOID header for the engage/ratchet/release rule implemented here.
function TAVOID.tick()
    if not CFG.TAVOID.ENABLE then return end
    -- AP-NAV GATE (see apconf.lua's CFG.TAVOID header): full manual is full manual, with no
    -- exception for a hold already in progress. Read directly off THIS computer's own
    -- redstone input -- the same one apAnnounceLoop re-announces as apswitch_1 -- ahead of
    -- every other check below, so a stale/unknown corridor reading at the exact moment the
    -- switch goes off cannot leave a hold running past it.
    if not redstone.getInput(CFG.APSWITCH.SIDE) then
        if TAVOID.active then
            TAVOID.active, TAVOID.holdAlt = false, 0
            rednet.broadcast({ cmd = "cancel" }, AUTOLAND_PROTOCOL)   -- see the note below
        end
        return
    end
    if os.clock() < TAVOID.overrideUntil then return end
    local ok, need = TAVOID.requiredAlt()
    if not ok then return end   -- don't know: change nothing, whichever state that leaves us in
    if TAVOID.active then
        if need and need > TAVOID.holdAlt then TAVOID.holdAlt = need end
        if (not need) or NAV.posY >= need + CFG.TAVOID.RELEASE_MARGIN then
            TAVOID.active, TAVOID.holdAlt = false, 0
            -- "cancel", not "release": this is a PLAIN TEARDOWN (the hazard cleared on its
            -- own), not a pilot override. altcom.lua's toOver() stashed whatever the pilot
            -- had selected when this engaged (and cleared the live pick, panel all green);
            -- "release" hands the panel to toPilot(), which assumes the tap that triggered it
            -- already re-selected something and so restores nothing -- exactly the bug this
            -- comment is here to stop someone reintroducing. "cancel" drives toNormal(false),
            -- which restores the stashed pick and resumes broadcasting it. Same verb LND uses
            -- on every one of ITS plain teardowns (see cancelNav and the RESUME LANDING path).
            rednet.broadcast({ cmd = "cancel" }, AUTOLAND_PROTOCOL)
        end
    elseif need and NAV.posY < need then
        TAVOID.active, TAVOID.holdAlt = true, need
    end
end

function TERR.loop()
    while true do
        local sender, msg = rednet.receive(CFG.TERR.PROTOCOL, 0.25)
        if sender then
            pcall(TERR.onMsg, sender, msg)
            -- Shares this receive rather than opening a second rednet.receive on the same
            -- protocol, which would race TERR's for the same messages (see the TAVOID header
            -- above).
            pcall(TAVOID.onMsg, sender, msg)
        end
        -- TERR.range is set by the instrument and is nil whenever the overlay is not on the
        -- screen -- no monitor, no fix, scaled in too close, switched off. Asking is gated on
        -- it, so an aircraft that is not showing terrain does not put the question on the air.
        if CFG.TERR.ENABLE and TERR.range and NAV.haveFix then
            local now = os.clock()
            -- ...and ask again at once, not on the next tick, when the instrument has
            -- changed scale under us: the grid we hold covers the wrong ground now.
            local rescaled = TERR.reqRange == nil or TERR.range > TERR.reqRange * 1.3
                          or TERR.range < TERR.reqRange * 0.7
            if rescaled or (now - TERR.reqAt) >= CFG.TERR.REQ_EVERY then
                if not (TERR.srv and TERR.geo) then
                    rednet.broadcast({ t = "hello", callsign = CFG.TERR.CALLSIGN,
                        x = NAV.posX, y = NAV.posY, z = NAV.posZ }, CFG.TERR.PROTOCOL)
                end
                pcall(TERR.request)
                TERR.reqAt = now
            end
        end
        -- TAVOID asks independently of the display (it must keep working with no monitor
        -- attached and CFG.TERR.ENABLE false), so it sends its own "hello" too when neither
        -- client has heard the server yet -- otherwise it would wait out CFG.TERR's own gate
        -- above, which may never open at all.
        if CFG.TAVOID.ENABLE and NAV.haveFix and not LND.active and not LND.ga then
            local now = os.clock()
            if (now - TAVOID.reqAt) >= CFG.TAVOID.REQ_EVERY then
                if not (TERR.srv and TERR.geo) then
                    rednet.broadcast({ t = "hello", callsign = CFG.TERR.CALLSIGN,
                        x = NAV.posX, y = NAV.posY, z = NAV.posZ }, CFG.TERR.PROTOCOL)
                end
                pcall(TAVOID.request)
                TAVOID.reqAt = now
            end
        end
    end
end

-- The range the COURSE rose paints terrain to. The rose has no range of its own -- it is a
-- CDI, not a map -- so this takes the ground the aircraft can reach in COURSE_LOOK seconds
-- and snaps it up to the HSI's own range ladder, which makes the scale jump in the same
-- familiar steps the landing map uses instead of breathing with every gust. One dial then
-- reads correctly from a hover to 320 m/s with nothing for the pilot to select.
function TERR.courseRange()
    local r = clamp((NAV.groundSpeed or 0) * CFG.TERR.COURSE_LOOK,
                    CFG.TERR.COURSE_MIN, CFG.TERR.COURSE_MAX)
    for _, v in ipairs(HSICFG.RANGES) do if r <= v then return v end end
    return CFG.TERR.COURSE_MAX
end

-- ============================================================================
-- TRAFFIC -- the other aeroplanes, off atc_broadcast
-- ============================================================================
-- A RECEIVE-ONLY client of the fleet-wide channel every responder already transmits on: the
-- same reports #44 (the tower) and #64 (the AWACS) paint, read here so the PLAN view can put
-- them on the route map. Nothing is transmitted. That asymmetry is the whole reason this is
-- safe to add to an aeroplane: a second BROADCASTER on a protocol is what the one-broadcaster
-- rule forbids (two senders alternate at 20 Hz and the last to speak wins); a second listener
-- costs the channel nothing and cannot be noticed by anyone.
--
-- It runs whatever view is up. It has to: contacts age out in seconds, so a roster that only
-- filled while the PLAN view was selected would be empty for the first few seconds after
-- every switch -- which is exactly the moment the pilot switched in order to look at it.
--
-- IT IS NOT A TCAS and must not be mistaken for the start of one. Nothing in this fleet
-- computes a resolution advisory: every responder hardcodes tcas_status = 'CLEAR'. What is
-- kept below is the field, not a decision made from it.
local TFC = {
    list  = {},      -- sending computer id -> { x, y, z, gs, trk, cs, vs, ifr, tcas, at }
    heard = nil,     -- os.clock() of the last report from anyone, for the annunciator
    n     = 0,       -- ...and how many are on the roster, kept rather than counted per frame
}

-- One report, keyed by SENDING COMPUTER ID and not by callsign -- the same choice #44 and #64
-- make, for the same reason: two aircraft cloned from one responder.lua with the callsign line
-- left unedited is a real and easy mistake, and keying on the callsign would silently merge
-- them into a single blip that teleports between the two. The callsign is still what own-ship
-- is filtered on (TFC.accept), because that is a question about identity rather than about
-- which transmitter a report came from.
function TFC.note(id, msg)
    local t = TFC.list[id]
    if not t then
        t = {}
        TFC.list[id] = t
        TFC.n = TFC.n + 1
    end
    local cs = msg.callsign
    t.at = os.clock()
    t.x, t.y, t.z = msg.x, msg.y, msg.z
    t.cs   = (type(msg.callsign_short) == "string") and msg.callsign_short or cs:sub(1, 3)
    t.vs   = tonumber(msg.vs) or 0
    t.ifr  = msg.is_ifr and true or false
    -- 'CLEAR' is not a resolution and must not read as one. Anything else is kept verbatim.
    t.tcas = (type(msg.tcas_status) == "string" and msg.tcas_status ~= "CLEAR")
             and msg.tcas_status or nil
    -- Ground TRACK where the responder sends its velocity vector, the nose where it does not.
    -- On a VTOL those are routinely ninety degrees apart -- it holds a heading and translates
    -- sideways -- so the distinction is not academic. Same rule #64 uses.
    if type(msg.vx) == "number" and type(msg.vz) == "number" then
        t.gs = math.sqrt(msg.vx * msg.vx + msg.vz * msg.vz)
        t.trk = (t.gs > 0.5) and toCompass(atan2(msg.vx, msg.vz)) or (tonumber(msg.hdg) or 0)
    else
        t.gs, t.trk = tonumber(msg.gs) or 0, tonumber(msg.hdg) or 0
    end
end

function TFC.sweep()
    local now, drop = os.clock(), CFG.HSI.PLAN.TRAFFIC.DROP_AFTER
    for id, t in pairs(TFC.list) do
        if now - t.at > drop then TFC.list[id] = nil; TFC.n = TFC.n - 1 end
    end
end

-- Is this report a contact? Anything at all can broadcast on a shared protocol, and a report
-- without a position is not a target but noise -- plotting it would put a blip on the
-- aircraft's own symbol. Its own function rather than a condition inside the loop so that the
-- rule can be exercised without a radio.
function TFC.accept(msg)
    return type(msg) == "table"
       and type(msg.x) == "number" and type(msg.y) == "number"
       and type(msg.z) == "number" and type(msg.callsign) == "string"
       -- Own-ship's own responder, dropped here rather than drawn. It transmits on this very
       -- channel from the same hull, so without this the aeroplane plots itself as a permanent
       -- contact sitting exactly under its own symbol. The name comes from CFG.TERR.CALLSIGN
       -- -- one hull, one name; see apconf.
       and msg.callsign ~= CFG.TERR.CALLSIGN
end

function TFC.loop()
    local T = CFG.HSI.PLAN.TRAFFIC
    if not (T.ENABLE and CFG.HSI.PLAN.ENABLE) then while true do sleep(5) end end
    while true do
        local sender, msg = rednet.receive(T.PROTOCOL, 1)
        if sender and TFC.accept(msg) then
            TFC.heard = os.clock()
            pcall(TFC.note, sender, msg)
        end
        TFC.sweep()
    end
end

-- ============================================================================
-- PLAN VIEW -- the route map, and the leg maths both normal-flight views share
-- ============================================================================
-- The instrument's second pilot-selectable view. Where the rose answers "where am I relative
-- to the course I am on", this answers "what does the rest of the flight look like": the
-- baked path itself (PLAN.samples, the clothoid-arc-clothoid geometry the aircraft actually
-- flies -- not the straight legs between the waypoints), the waypoints on it, and the same
-- cross-track reading collapsed onto a one-row gauge along the bottom so that switching views
-- costs the pilot nothing he was reading before.
--
-- IT IS A DISPLAY AND ONLY A DISPLAY, exactly as TERR is: nothing here is read back by
-- navTick, the speed law or the landing machine, and the aircraft flies the same path
-- whichever view is up. What it does change is the RADIO -- the terrain overlay is off here,
-- and saying so on TERR.range takes the map query off the air (see TERR.loop), so the second
-- view costs #68 nothing.
--
-- HEADING-UP, aircraft fixed at the centre, same as the other two views. A pilot switching
-- views should not have to re-orient, and this instrument is never the chart: a north-up
-- overview of the database is what the FMS's own EDIT DATA screen is for.
--
-- WHAT ELSE IS ON IT. The route is drawn over two other layers, and all three have to share
-- a character cell's two colours -- which is what decides how each is drawn:
--
--   TERRAIN, as the cells' BACKGROUND, sampled one per character cell. Only the bands that
--   mean something is in the way: warning, caution, and a floating mass overhead. Not the
--   rose's heightmap -- see apconf.lua's CFG.HSI.PLAN header for why a landscape is the wrong
--   picture under a route. A hazard is a BLOB and loses nothing at cell resolution; that is
--   what leaves the subpixels free for the path, which is a LINE and needs them.
--
--   TRAFFIC, as glyphs over the top, from TFC. hText paints its own black background, so a
--   contact stays legible over any hazard colour underneath it -- the same trick, and the
--   same one-cell cost, the landing map pays for the far end of a runway.
--
-- HOW THE PATH IS DRAWN, and why not with hLine. The polyline is CFG.DC.SAMPLE_SPACING (5 m)
-- dense -- thousands of points on a real route -- and hFill costs three monitor calls per
-- cell. So the path is rasterised into a 2x3 SUBPIXEL bitmap and blitted a row at a time,
-- which is both cheaper and six times the samples (see 11-conventions-and-traps.md, "the
-- pseudo-bitmap"). It shares TERR.SX / TERR.SW rather than building its own glyph tables:
-- one bitmap convention on this instrument, not two.
--
-- WHAT IS CACHED, and why the glyphs are cached too. The raster is rebuilt at PLAN.REDRAW and
-- re-blitted at the instrument's own rate, the bargain TERR strikes. The compass card and the
-- waypoint glyphs are cheap enough to recompute every frame -- and are deliberately NOT,
-- because they are spatially tied to the path: a glyph that moved at 20 Hz against a path
-- that moves at 10 would swim, and would read as a fault rather than as a saving.
CRS.DOTS = { -1, -0.5, 0.5, 1 }                             -- CDI gauge dot positions
CRS.CARD = { [0] = "N", [90] = "E", [180] = "S", [270] = "W" }
-- The two tones the path is drawn in, as blit digits (see TERR.blitChar -- the digits are
-- ordered BY BIT, so "0" is white and black is "f"). Flown path dim, path still to fly lit;
-- the same cyan/blue pair, and the same brighter-is-the-live-one reading, as the course
-- needle's lit and dark tips.
CRS.AHEAD, CRS.BEHIND = TERR.blitChar(colours.cyan), TERR.blitChar(colours.blue)
CRS.m, CRS.c   = {}, {}      -- cell index -> 6-bit subpixel mask / "holds a to-fly subpixel"
CRS.bg         = {}          -- ...and its terrain background, as a blit digit (nil = black)
CRS.lo, CRS.hi = {}, {}      -- ...and the first and last column touched in each row, so
                             -- rasterise() only walks the span that has anything in it
CRS.gl      = {}             -- cached waypoint marks: { x, y, ch, col, label, lx }
CRS.tfc     = {}             -- ...and traffic marks, same shape, built in the same frame.
                             -- NOT `tf`: that is CRS.leg's TO/FROM cosine, and the two
                             -- collided the first time this was written.
CRS.rows    = nil            -- the blitted raster, one { chars, fg, bg } per map row
CRS.rowN, CRS.sw, CRS.sh = 0, 0, 0
CRS.builtAt = 0

-- The active leg, as BOTH normal-flight views need it: desired track, signed cross-track,
-- distance, name and TO/FROM. Lifted out of drawCourseHSI when the PLAN view arrived, because
-- the conventions in here are subtle enough that two copies of them would drift -- the sign
-- (+xtk = RIGHT of course), the curve-following override that makes the dial read the PLANNED
-- PATH rather than the straight leg, and TO/FROM being measured against whichever of the two
-- is actually displayed so that it flips exactly as the fix goes abeam.
--
-- Answers on the table rather than as six returns: both callers are large functions and this
-- file's nested functions are charged the main chunk's locals (see apconf.lua's DC block).
function CRS.leg(heading)
    CRS.has, CRS.dtk, CRS.xtk, CRS.dist, CRS.tgt, CRS.tf = false, heading, 0, 0, nil, 0
    local tgt = NAV.active and NAV.points[NAV.index]
    if not tgt then return end
    local dx, dz = tgt.x - NAV.posX, tgt.z - NAV.posZ
    CRS.has, CRS.tgt = true, tgt.name
    CRS.dist, CRS.dtk = math.sqrt(dx*dx + dz*dz), atan2(dx, dz)
    local prev = NAV.points[NAV.index - 1]
    if prev then
        local lx, lz = tgt.x - prev.x, tgt.z - prev.z
        local ll = math.sqrt(lx*lx + lz*lz)
        if ll > 1e-3 then
            CRS.dtk = atan2(lx, lz)
            CRS.xtk = (lx/ll)*(NAV.posZ - prev.z) - (lz/ll)*(NAV.posX - prev.x)  -- += right
        end
    end
    -- Curve-following: show the PLANNED PATH as the course, not the straight leg, so the dial
    -- reads what the aircraft actually flies (and on-course when it's on the path). DTK = the
    -- path's heading at our projection; XTK/CDI = signed perpendicular deviation FROM the path
    -- (same += right-of-course sign as the leg version). DIST stays to the real waypoint.
    if DC.CURVE and NAV.phase == "ENROUTE" and NAV.index >= 2 and NAV.index ~= #NAV.points
            and PLAN.onPath() then
        local cdtk, cxtk = PLAN.course()
        if cdtk then CRS.dtk, CRS.xtk = cdtk, cxtk end
    end
    -- TO/FROM -- the same question a VOR's flag answers: is the waypoint ahead of us along
    -- the course, or behind?
    CRS.tf = math.cos(atan2(dx, dz) - CRS.dtk)
end

-- ---- autozoom ---------------------------------------------------------------------------
-- There is no range selector on this instrument and deliberately so: one dial that reads
-- correctly from a hover to 320 m/s beats a knob nobody touches. Three things ask for a
-- range, the SMALLEST wins, and the answer is snapped up HSICFG.RANGES. See apconf.lua's
-- CFG.HSI.PLAN header for what the three are and why.
function CRS.autoRange()
    local P = HSICFG.PLAN
    local want = clamp((NAV.groundSpeed or 0) * P.LOOK_T, P.MIN_RANGE, P.MAX_RANGE)
    local S, n = PLAN.samples, #PLAN.samples
    if n > 0 and NAV.haveFix then
        -- The corners ahead. A corner of radius R asks for CORNER_K * R of range -- BUT ONLY
        -- ONCE IT IS INSIDE THAT RANGE, which is the whole difference between "zoom in on the
        -- interesting bit" and a dial that zooms in on something it then cannot show. Without
        -- the second half, a 400 m corner three kilometres away would pull the map straight
        -- down to a one-kilometre ring and hide the corner it zoomed in for.
        --
        -- It also makes the horizon self-consistent: a corner further ahead than `want` asks
        -- for more range than `want` and so cannot constrain anything, which is exactly why
        -- the scan can stop there. Walked with a stride that bounds it at about two hundred
        -- samples whatever the range -- curvature is piecewise-LINEAR in arc length
        -- (PLAN.transHdg), so coarse sampling cannot miss a corner, only clip the tip of one.
        PLAN.updateCursor(NAV.posX, NAV.posZ)
        local st = math.max(1, math.floor(want / DC.SAMPLE_SPACING / 200))
        local sEnd = PLAN.lastS + want
        local i = PLAN.sampleIndexAtS(S, PLAN.lastS)
        while i <= n and S[i].s <= sEnd do
            local k = math.abs(S[i].kappa)
            if k > 1e-9 then
                local rc = P.CORNER_K / k
                if (S[i].s - PLAN.lastS) <= rc and rc < want then want = rc end
            end
            i = i + st
        end
    end
    -- ...and the LAST waypoint, which is what makes an arrival close in on itself. A distant
    -- one constrains nothing, which is the point.
    local last = NAV.active and NAV.points[#NAV.points]
    if last and NAV.haveFix then
        local dx, dz = last.x - NAV.posX, last.z - NAV.posZ
        want = math.min(want, math.sqrt(dx*dx + dz*dz) * P.END_K)
    end
    return clamp(want, P.MIN_RANGE, P.MAX_RANGE)
end

-- Snap a wanted range up the ladder, with hysteresis. Zooming OUT is immediate -- you always
-- want to see far enough -- and zooming IN waits until the wanted range has fallen well
-- inside the step below, because a value sitting on a ladder boundary would otherwise flap
-- between two scales at the rebuild rate, which is the one thing a moving map must not do.
--
-- THE DEADBAND IS ON THE BOUNDARY BEING CROSSED -- the step immediately below the one we are
-- on -- and not on the step being moved to. Measured against the destination it locks the map
-- out of a legitimate several-step zoom-in: a corner appearing on a long leg wants 880 m off
-- a 5000 m ring, lands on the 1000 m step, and is 12% inside it, so a 25% deadband read that
-- way refuses the whole move and the map never leaves 5000. Found in a stub harness before it
-- ever flew, which is the only reason it is written down here rather than discovered in the
-- air: zoom hysteresis looks identical to "the autozoom does not work".
function CRS.snap(want)
    local R, r = HSICFG.RANGES, nil
    for _, v in ipairs(R) do if want <= v then r = v break end end
    r = r or R[#R]
    if CRS.rng and r < CRS.rng then
        local below = nil
        for i = #R, 1, -1 do if R[i] < CRS.rng then below = R[i] break end end
        if below and want > below * HSICFG.PLAN.HYST then r = CRS.rng end
    end
    CRS.rng = r
    return r
end

-- ---- the raster -------------------------------------------------------------------------

-- World point -> continuous SUBPIXEL coordinates in the map rectangle, heading-up: x across
-- (0 .. 2*width), y down (0 .. 3*rows). Fractional on purpose -- the DDA in CRS.seg steps
-- between two of these.
function CRS.proj(wx, wz)
    local dx, dz = wx - NAV.posX, wz - NAV.posZ
    local f = dx * CRS.fx + dz * CRS.fz      -- up the screen (the nose)
    local r = dx * CRS.rx + dz * CRS.rz      -- across it, to the right
    return (CRS.cx + (r / CRS.range) * CRS.Rx - 0.5) * CRS.spx,
           (CRS.cy - (f / CRS.range) * CRS.Ry - CRS.top + 0.5) * CRS.spy
end

-- Light one subpixel. `ahead` is what decides the cell's COLOUR: a cell holds two colours and
-- no more, so a cell carrying any still-to-fly subpixel is drawn lit even if most of it is
-- flown path. The two only ever meet in the one cell the aircraft is projected into.
function CRS.plot(sx, sy, ahead)
    if sx < 0 or sx >= CRS.sw or sy < 0 or sy >= CRS.sh then return end
    local px, py = math.floor(sx), math.floor(sy)
    local r = (py - py % CRS.spy) / CRS.spy + 1
    local c = (px - px % CRS.spx) / CRS.spx + 1
    local i = (r - 1) * CRS.w + c
    if CRS.spx == 1 then
        CRS.m[i] = 63                        -- whole-cell mode: light every subpixel there is
    else
        -- Bit order is the one TERR.SX documents: 1,2 top row; 4,8 middle; 16,32 bottom. Set
        -- WITHOUT a bitwise or -- Cobalt is 5.1-flavoured and has none.
        local b = TERR.BIT[(py % 3) * 2 + (px % 2) + 1]
        local m = CRS.m[i] or 0
        if m % (b + b) < b then CRS.m[i] = m + b end
    end
    if ahead then CRS.c[i] = true end
    if not CRS.lo[r] or c < CRS.lo[r] then CRS.lo[r] = c end
    if not CRS.hi[r] or c > CRS.hi[r] then CRS.hi[r] = c end
end

-- One chord of the path. CEIL on the step count, not floor: floor makes the step marginally
-- longer than a subpixel and every so often skips one, which reads as a path with holes
-- punched along it (11-conventions-and-traps.md again).
function CRS.seg(x0, y0, x1, y1, ahead)
    local dx, dy = x1 - x0, y1 - y0
    local n = math.ceil(math.max(math.abs(dx), math.abs(dy)))
    for k = 1, n do CRS.plot(x0 + dx * k / n, y0 + dy * k / n, ahead) end
end

-- The baked path. Walked with a stride that puts consecutive plotted points about one
-- subpixel apart at the current scale, and never more than PLAN.MAX_POINTS of them however
-- long the route is; CRS.seg fills whatever gap the stride leaves.
--
-- The in-box test is PADDED BY A WHOLE CHORD, and that padding is the point of it: a chord
-- with one end off the screen still has to be drawn or the path would stop short of the edge
-- by up to a chord's length. Both ends outside the padded box and there is nothing to draw,
-- which is also what bounds the DDA -- otherwise a route leg passing a thousand screens away
-- would be stepped a thousand screens' worth of subpixels.
function CRS.path()
    local S, n = PLAN.samples, #PLAN.samples
    if n == 0 then return end
    local P = HSICFG.PLAN
    local sub = math.min(CRS.range / (CRS.spx * CRS.Rx), CRS.range / (CRS.spy * CRS.Ry))
    local st  = math.max(1, math.floor(sub / DC.SAMPLE_SPACING), math.ceil(n / P.MAX_POINTS))
    local pad = st * DC.SAMPLE_SPACING / sub + 2
    -- Everything behind the arc-length cursor is flown. Off the path there is no cursor worth
    -- believing (PLAN.onPath), and the honest answer is that none of it is flown yet.
    local s0 = PLAN.onPath() and PLAN.lastS or -1
    local px, py, pin, i = 0, 0, false, 1
    while true do
        local ax, ay = CRS.proj(S[i].x, S[i].z)
        local inb = ax >= -pad and ax < CRS.sw + pad and ay >= -pad and ay < CRS.sh + pad
        if inb then
            local ahead = S[i].s >= s0
            CRS.plot(ax, ay, ahead)
            if pin then CRS.seg(px, py, ax, ay, ahead) end
        end
        px, py, pin = ax, ay, inb
        if i >= n then break end
        i = math.min(i + st, n)
    end
end

-- ---- the terrain layer ------------------------------------------------------------------

-- Mark a cell's background, and widen the row's touched span so rasterise() reaches it. The
-- span is shared with CRS.plot: a row is skipped only when NEITHER layer put anything in it.
function CRS.paintCell(r, c, digit)
    CRS.bg[(r - 1) * CRS.w + c] = digit
    if not CRS.lo[r] or c < CRS.lo[r] then CRS.lo[r] = c end
    if not CRS.hi[r] or c > CRS.hi[r] then CRS.hi[r] = c end
end

-- Hazard and obstacles, one sample per character cell, straight off TERR's own classifier so
-- that red means here exactly what it means on the rose. Four of the nine bands are kept:
-- 1 WARN, 2 CAUTION, 8 a mass within CAUTION_M overhead, 9 one further off. The heightmap
-- steps and water are dropped -- this layer says what is IN THE WAY, not what the land is.
--
-- The gradient is dropped with them, down to one coarse step: a CAUTION cell further from the
-- warning band than THIN_LVL, and an overhead mass further away than THIN_LVL, are drawn as a
-- CHECKERBOARD instead of solid. Both of those bands are wide, and without the thinning every
-- cell under an island is a solid magenta wash with the route lost inside it. `lvl` counts UP
-- toward the hotter neighbour for CAUTION (closer to a warning) and UP toward black for an
-- overhead mass (further overhead), which is why the two comparisons face opposite ways.
--
-- It also sets TERR.range, which is what puts the map query on the air (TERR.loop gates on
-- it) and what TERR.label reads to decide whether the overlay is off or broken. The gates
-- below are TERR.paint's, for TERR.paint's reasons -- in particular the MIN_RANGE floor,
-- which matters MORE here: at that scale the rose paints a solid red screen, and this view
-- would instead paint nothing at all, which reads as "nothing in the way".
function CRS.terrain()
    TERR.range = nil
    if not (HSICFG.PLAN.TERRAIN and CFG.TERR.ENABLE and TERR.canBlit) then return end
    if not (NAV.haveFix and NAV.haveHeading) then return end
    if CRS.range < CFG.TERR.MIN_RANGE then return end
    TERR.range = CRS.range
    if not (TERR.available() and TERR.haveGrid()) then TERR.worst = -1 return end

    -- TERR.at memoises by GRID index against the altitude of the build it was made in, so the
    -- memo has to be dropped here for the same reason TERR.build drops it: a descent since the
    -- last one would otherwise be painted at the old aircraft's clearances.
    TERR.memo, TERR.worst = {}, -1
    local K, thin = CFG.TERR, HSICFG.PLAN.THIN_LVL
    for r = 1, CRS.rowN do
        local v = (CRS.cy - (CRS.top + r - 1)) / CRS.Ry
        local bx = NAV.posX + CRS.fx * v * CRS.range      -- this row's centre, in the world
        local bz = NAV.posZ + CRS.fz * v * CRS.range
        for c = 1, CRS.w do
            local u = (c - CRS.cx) / CRS.Rx
            local band, lvl = TERR.pt(bx + CRS.rx * u * CRS.range, bz + CRS.rz * u * CRS.range)
            if band and band > 0 then
                -- The annunciator's severity, on TERR.cell's own scale and split the same way
                -- -- 0 is "charted and clear", -1 is "nothing charted at all", and TERR.label
                -- turns that pair into the difference between a working display and a dead
                -- one. It is shared with the rose, so it must be computed identically here.
                local sev = (band == 1) and 3 or ((band == 2 or band == 8) and 2 or 1)
                if sev > TERR.worst then TERR.worst = sev end
                if band == 1 then
                    CRS.paintCell(r, c, TERR.B[1])
                elseif band == 2 or band == 8 or band == 9 then
                    local dim = (band == 2) and (lvl < thin) or (band ~= 2 and lvl >= thin)
                    if not (dim and (r + c) % 2 == 0) then
                        CRS.paintCell(r, c, TERR.B[band])
                    end
                end
            elseif band == 0 and TERR.worst < 0 then
                TERR.worst = 0                           -- charted, and nothing on it
            end
        end
    end
end

-- ---- traffic ----------------------------------------------------------------------------

-- The contacts TFC is holding, projected and classified once per rebuild alongside the
-- waypoints -- same frame, same reason (see the header): a symbol recomputed every frame
-- would swim against a path that is not.
--
-- Two filters before anything is drawn. The VERTICAL one is the ND's ABV/NRM/BLW in one
-- fixed step, sized for a 384 m world rather than for an aeroplane's 2 700 ft. The LATERAL
-- one is the screen itself: a contact off the map is simply not drawn, and is not clamped to
-- the edge, because a symbol at the wrong position is worse than no symbol -- the count on
-- the second row is what says there is traffic you cannot see.
function CRS.traffic()
    local T = HSICFG.PLAN.TRAFFIC
    CRS.tfcWorst = 0                      -- 0 none | 1 shown | 2 proximate | 3 resolution
    if not (T.ENABLE and NAV.haveFix and NAV.haveHeading) then return end
    local now, n = os.clock(), 0
    for _, t in pairs(TFC.list) do
        if n >= T.MAX then break end
        local dy = t.y - NAV.posY
        if dy <= T.ALT_FILTER_M and dy >= -T.ALT_FILTER_M then
            local sx, sy = CRS.proj(t.x, t.z)
            local x, y = math.floor(sx / CRS.spx) + 1, CRS.top + math.floor(sy / CRS.spy)
            if x >= 1 and x <= CRS.w and y >= CRS.top and y <= CRS.bot then
                local dx, dz = t.x - NAV.posX, t.z - NAV.posZ
                local near = (dx * dx + dz * dz) <= T.PROX_M * T.PROX_M
                             and math.abs(dy) <= T.PROX_V
                local e = { x = x, y = y, ch = "*", col = colours.white }
                if (now - t.at) > T.COAST_AFTER then
                    e.col = colours.grey               -- coasting: last seen here, not now
                elseif t.tcas then
                    e.ch, e.col = "!", colours.yellow  -- a responder that sent a resolution
                    CRS.tfcWorst = 3
                elseif near then
                    e.col = colours.yellow
                    if CRS.tfcWorst < 2 then CRS.tfcWorst = 2 end
                elseif CRS.tfcWorst < 1 then
                    CRS.tfcWorst = 1
                end
                CRS.tfc[#CRS.tfc + 1] = e
                CRS.occ[y * 4096 + x] = true
                -- Relative altitude, the ND's own tag, in metres rather than hundreds of feet
                -- because this world is 384 m tall. Clamped to two digits: past 99 m apart the
                -- exact number has stopped mattering and the tag is only saying "well clear".
                CRS.label(e, string.format("%+03d", math.floor(clamp(dy, -99, 99) + 0.5)))
                n = n + 1
            end
        end
    end
end

-- Put a short label beside a mark -- a waypoint's name, or a contact's relative altitude --
-- if it fits and if nothing is already there. Right of the mark by preference, left when that
-- would run off the edge; dropped rather than overprinted, because an unreadable map is worse
-- than a sparsely labelled one.
--
-- CRS.occ is ONE map shared by every layer that claims screen cells, and the order the layers
-- claim in is the priority: waypoints before traffic, so a contact's altitude tag gives way to
-- a fix's name rather than the other way round. (The SYMBOLS are drawn in the opposite order,
-- so where the two want the same single cell, traffic wins the cell and the fix keeps its
-- name. That is deliberate: a fix is where the name says it is, an aeroplane is not.)
function CRS.label(e, name)
    local s = (name or ""):sub(1, 5)
    if s == "" then return false end
    local x = e.x + 2
    if x + #s - 1 > CRS.w then x = e.x - 1 - #s end
    if x < 1 then return false end
    for k = x, x + #s - 1 do if CRS.occ[e.y * 4096 + k] then return false end end
    for k = x, x + #s - 1 do CRS.occ[e.y * 4096 + k] = true end
    e.label, e.lx = s, x
    return true
end

-- The waypoints, in four tones: flown (dim), still to fly (lit), THE ONE BEING FLOWN TO
-- (lime, the same "this is the live one" green as the on-course needle and the landing map's
-- aim point), and the destination when it is a landing site (orange, the same colour and the
-- same H/+ glyph the landing map draws it with, so the two views agree about what it is).
function CRS.marks()
    if not NAV.active then return end
    local pts, at = NAV.points, {}
    for i = 1, #pts do
        local sx, sy = CRS.proj(pts[i].x, pts[i].z)
        local x, y = math.floor(sx / CRS.spx) + 1, CRS.top + math.floor(sy / CRS.spy)
        if x >= 1 and x <= CRS.w and y >= CRS.top and y <= CRS.bot then
            local e = { x = x, y = y, ch = "o", col = colours.cyan }
            if i < NAV.index then e.col = colours.blue end
            if NAV.landAtEnd and i == #pts then
                e.ch  = (NAV.landSite and NAV.landSite.type == "heli") and "H" or "+"
                e.col = colours.orange
            elseif i == NAV.index then
                e.col = colours.lime
            end
            CRS.gl[#CRS.gl + 1] = e
            at[i] = e
            CRS.occ[y * 4096 + x] = true
        end
    end
    -- Names, most-wanted first and only as many as PLAN.LABELS: the one being flown to, then
    -- the destination, then the rest in route order.
    local ord, budget = { NAV.index, #pts }, HSICFG.PLAN.LABELS
    for k = 1, #pts do ord[#ord + 1] = k end
    for _, i in ipairs(ord) do
        if budget <= 0 then break end
        local e = at[i]
        if e and not e.label and CRS.label(e, pts[i].name) then budget = budget - 1 end
    end
end

-- Masks and colours -> the three blit strings per row. Rows with nothing in them are the
-- shared blank triple rather than W concatenations of a space, and a row that does have
-- something is built as run + span + run (CRS.lo/hi) rather than cell by cell -- the same
-- reason the PFD's horizon is built from runs: a whole-screen inner loop per rebuild is the
-- cost this instrument cannot afford, not the arithmetic inside it.
function CRS.rasterise()
    if CRS.blankW ~= CRS.w then
        CRS.blank = { string.rep(" ", CRS.w), string.rep(TERR.blk, CRS.w),
                      string.rep(TERR.blk, CRS.w) }
        CRS.blankW = CRS.w
    end
    local rows, blank, ch, fg, bg = {}, CRS.blank, {}, {}, {}
    for r = 1, CRS.rowN do
        local lo, hi = CRS.lo[r], CRS.hi[r]
        if not lo then
            rows[r] = blank
        else
            local base, n = (r - 1) * CRS.w, hi - lo + 1
            for c = 1, n do
                local i = base + lo + c - 1
                local m = CRS.m[i]
                -- The cell's two colours, spent exactly once each: the terrain band (or
                -- black) as the BACKGROUND, the path as the ink over it. That is the whole
                -- reason the terrain layer is sampled per cell and not per subpixel -- six
                -- terrain samples and a path in one cell is three colours, and there are two.
                local tb = CRS.bg[i] or TERR.blk
                if m then
                    local hot = CRS.c[i] and CRS.AHEAD or CRS.BEHIND
                    ch[c] = TERR.SX[m]
                    -- A mask with the bottom-right subpixel set is drawn as its COMPLEMENT
                    -- with the two colours swapped; TERR.SW is what records that.
                    if TERR.SW[m] then fg[c], bg[c] = tb, hot
                    else               fg[c], bg[c] = hot, tb end
                else
                    ch[c], fg[c], bg[c] = " ", TERR.blk, tb
                end
            end
            rows[r] = {
                blank[1]:sub(1, lo - 1) .. table.concat(ch, "", 1, n) .. blank[1]:sub(1, CRS.w - hi),
                blank[2]:sub(1, lo - 1) .. table.concat(fg, "", 1, n) .. blank[2]:sub(1, CRS.w - hi),
                blank[3]:sub(1, lo - 1) .. table.concat(bg, "", 1, n) .. blank[3]:sub(1, CRS.w - hi),
            }
        end
    end
    CRS.rows = rows
end

-- Rebuild everything that is cached together, in ONE frame of reference: the world vectors
-- the projection uses, the path raster, the waypoint marks and the heading the compass card
-- is drawn from. They are rebuilt together because they are read together -- see the header.
function CRS.build()
    CRS.m, CRS.c, CRS.lo, CRS.hi = {}, {}, {}, {}
    CRS.bg, CRS.gl, CRS.tfc, CRS.occ = {}, {}, {}, {}
    CRS.rowN = CRS.bot - CRS.top + 1
    -- Subpixels per character cell, and the raster's size in them. 2x3 is the block glyphs'
    -- grid; 1x1 is CFG.HSI.PLAN.SUBPIXEL = false, where a "subpixel" is the whole cell.
    CRS.spx, CRS.spy = (HSICFG.PLAN.SUBPIXEL and 2 or 1), (HSICFG.PLAN.SUBPIXEL and 3 or 1)
    CRS.sw, CRS.sh = CRS.w * CRS.spx, CRS.rowN * CRS.spy
    -- Screen up is the nose: the compass-to-world map is the inverse of toCompass, and
    -- "right" is the nose turned ninety degrees clockwise, which costs no second pair of
    -- trig. Identical to TERR.build's, and deliberately derived from the same cached chdg so
    -- the card, the ground and the path can never disagree about north.
    local phi = math.rad(180 - CRS.chdg)
    CRS.fx, CRS.fz = math.sin(phi), math.cos(phi)
    CRS.rx, CRS.rz = -CRS.fz, CRS.fx
    -- Bottom to top, and the order is the priority: the ground, the route over it, the fixes
    -- on the route, and the traffic last because it is the only one of the four that moves on
    -- its own. CRS.terrain is outside the fix test because it has its own gates (and has to
    -- run to clear TERR.range when it declines).
    CRS.terrain()
    if NAV.haveFix and NAV.haveHeading then
        CRS.path()
        CRS.marks()
        CRS.traffic()
    end
    CRS.rasterise()
    CRS.builtAt = os.clock()
end

-- Work out this frame's geometry and range, and rebuild the cache if it has gone stale or if
-- anything it was built against has moved under it.
function CRS.frame(W, H)
    local top, bot = 3, math.max(3, H - 2)
    if CRS.rows and CRS.w == W and CRS.top == top and CRS.bot == bot
            and (os.clock() - CRS.builtAt) < HSICFG.PLAN.REDRAW then
        return
    end
    -- The range is re-chosen only on a rebuild, not on every frame: the autozoom walks the
    -- path to find its tightest corner, and a rescale has to be accompanied by a rebuild in
    -- any case, so paying for it at 20 Hz would buy nothing at all.
    CRS.w, CRS.top, CRS.bot = W, top, bot
    CRS.range = CRS.snap(CRS.autoRange())
    CRS.cx, CRS.cy = math.floor(W / 2) + 1, math.floor((top + bot) / 2 + 0.5)
    CRS.Ry = math.max(1, (bot - top) / 2)
    CRS.Rx = math.max(1, math.min((W - 1) / 2, CRS.Ry * 1.5))  -- cells are ~1.5:1: keep it round
    CRS.chdg = NAV.haveHeading and toCompass(NAV.heading) or 0
    CRS.build()
end

function CRS.paint()
    if not (CRS.rows and TERR.canBlit) then return end
    for r = 1, CRS.rowN do
        hsiMon.setCursorPos(1, CRS.top + r - 1)
        hsiMon.blit(CRS.rows[r][1], CRS.rows[r][2], CRS.rows[r][3])
    end
end

-- The range ring and the rotating card, drawn from the RASTER's heading rather than the live
-- one so that north and the path never disagree by a rebuild's worth of turn.
function CRS.ring()
    for a = 0, 345, 15 do
        hFill(CRS.cx + CRS.Rx * math.sin(math.rad(a)),
              CRS.cy - CRS.Ry * math.cos(math.rad(a)), colours.grey)
    end
    for b = 0, 270, 90 do
        local a = math.rad(b - CRS.chdg)
        hText(CRS.cx + CRS.Rx * 0.85 * math.sin(a), CRS.cy - CRS.Ry * 0.85 * math.cos(a),
              CRS.CARD[b], colours.white)
    end
end

-- Waypoints first, then traffic over them: where the two want one cell, the aeroplane wins
-- it. A fix is exactly where its name says it is and can be found again; a contact cannot.
function CRS.glyphs()
    for i = 1, #CRS.gl do
        local e = CRS.gl[i]
        hText(e.x, e.y, e.ch, e.col)
        if e.label then hText(e.lx, e.y, e.label, e.col) end
    end
    for i = 1, #CRS.tfc do
        local e = CRS.tfc[i]
        hText(e.x, e.y, e.ch, e.col)
        if e.label then hText(e.lx, e.y, e.label, e.col) end
    end
end

-- The traffic annunciator: how many contacts are on the roster, coloured by the worst thing
-- ON THE SCREEN. The count is the ROSTER's, not the screen's, and deliberately so -- a
-- contact filtered out by range or by altitude is the one case where the pilot needs telling
-- that there is something the map is not showing him. Nothing at all and it says nothing,
-- because a permanent "TFC 0" is a line nobody reads.
function CRS.tfcLabel()
    if not (HSICFG.PLAN.TRAFFIC.ENABLE and TFC.n > 0) then return nil end
    local c = colours.grey
    if CRS.tfcWorst and CRS.tfcWorst >= 2 then c = colours.yellow
    elseif CRS.tfcWorst == 1 then c = colours.white end
    return "TFC " .. TFC.n, c
end

-- The one-row course-deviation gauge along the bottom. SAME SCALE, same sign and the same two
-- colours as the rose's CDI needle (HSICFG.CDI_FULLSCALE_M / CDI_ONCOURSE_M) -- one scale,
-- two presentations, so retuning the rose retunes this and the pilot never has to hold two
-- full-scale values in his head. The needle marks WHERE THE COURSE IS: right of course puts
-- it left of centre and you fly toward it.
function CRS.gauge(W, y)
    local half = math.max(1, math.floor(W / 2) - 2)
    for _, d in ipairs(CRS.DOTS) do hFill(CRS.cx + d * half, y, colours.grey) end
    hText(CRS.cx, y, "|", colours.white)
    if not CRS.has then return end
    hFill(CRS.cx - clamp(CRS.xtk / HSICFG.CDI_FULLSCALE_M, -1, 1) * half, y,
        (math.abs(CRS.xtk) <= HSICFG.CDI_ONCOURSE_M) and colours.lime or colours.yellow)
end

-- PLAN view: the route map. Shown instead of the rose while the pilot has selected it and no
-- landing is engaged.
local function drawPlanHSI()
    local W, H = hsiMon.getSize()
    hsiMon.setBackgroundColour(colours.black); hsiMon.clear()

    CRS.leg(NAV.haveHeading and NAV.heading or 0)
    CRS.frame(W, H)
    CRS.paint()
    CRS.ring()
    CRS.glyphs()
    hText(CRS.cx, CRS.cy, "^", colours.white)             -- aircraft, always nose-up

    -- Top line, identical to the rose's so the two views read the same at a glance. The
    -- heading NUMBER is the live one even though the card is the raster's: a readout that
    -- lags is the kind of thing that gets reported as a fault, and the card cannot lag the
    -- path it is drawn over.
    local phdg = math.floor((NAV.haveHeading and toCompass(NAV.heading) or 0) + 0.5) % 360
    if phdg == 0 then phdg = 360 end
    hText(CRS.cx - 3, 1, string.format("HDG %03d", phdg), colours.orange)
    if NAV.active then
        if NAV.aligning then
            hText(2, 1, "ALIGN  ", colours.yellow)
        elseif DEP.ground() and DEP.phase ~= "ROLL" then
            hText(2, 1, "LINE UP", colours.yellow)   -- a departure, short of its take-off roll
        elseif ATC.holding then
            hText(2, 1, "HOLD   ", colours.orange)
        else
            hText(2, 1, string.format("CMD%3.0f", NAV.cmdSpeed), colours.lime)
        end
        hText(W - 7, 1, string.format("GS%4.0f", NAV.groundSpeed or 0), colours.cyan)
    end

    -- Second row: which view this is and what is being flown on the left, then the two
    -- annunciators, then the scale on the right. TERR earns its place here for the reason it
    -- earns it on the rose -- a terrain layer that has quietly stopped being one looks exactly
    -- like clear ground -- and on THIS view it earns it more, because black here means "your
    -- route is clear" and it must never be able to mean "the server died" as well.
    local rt = "R" .. math.floor(CRS.range) .. "m"
    hText(W - #rt + 1, 2, rt, colours.grey)
    if NAV.active then
        hText(2, 2, ("PLAN " .. (NAV.planName or "")):sub(1, 12), colours.cyan)
    else
        hText(2, 2, "PLAN NOPLAN ", colours.grey)
    end
    do
        local tl, tc = TERR.label()
        if tl then hText(15, 2, tl:sub(1, 11), tc) end
        local fl, fc = CRS.tfcLabel()
        if fl then hText(27, 2, fl:sub(1, 8), fc) end
    end

    CRS.gauge(W, H - 1)

    -- ...and the numbers under it, the same line the rose carries.
    if CRS.has then
        hText(2, H, string.format("%-6s DTK%03d %s", (CRS.tgt or ""):sub(1, 6),
            math.floor(toCompass(CRS.dtk) + 0.5) % 360,
            (math.abs(CRS.tf) < HSICFG.TOFROM_DEAD) and "--" or (CRS.tf >= 0 and "TO" or "FR")),
            colours.cyan)
        hText(W - 16, H, string.format("XTK%4.1f%s D%4.0f", math.abs(CRS.xtk),
            CRS.xtk >= 0 and "R" or "L", CRS.dist), colours.white)
    else
        hText(2, H, "NO ACTIVE LEG", colours.grey)
    end
end

-- COURSE mode: the classic heading-up compass rose + course-deviation needle for the
-- active leg. Shown while a course/direct-to is being flown (no landing engaged).
local function drawCourseHSI()
    local W, H = hsiMon.getSize()
    hsiMon.setBackgroundColour(colours.black); hsiMon.clear()

    local heading = NAV.haveHeading and NAV.heading or 0

    -- Course, cross-track, distance, name and TO/FROM for the active leg. Shared with the
    -- PLAN view -- CRS.leg is where all of that, and the sign conventions in it, are written
    -- down; the answers come back on CRS.has / .dtk / .xtk / .dist / .tgt / .tf.
    CRS.leg(heading)

    local midX, midY = math.floor(W/2) + 1, math.floor(H/2) + 1
    local Rr = math.max(4, math.min(midY - 3, math.floor((midX - 3)/1.5)))
    local Rc = math.floor(Rr * 1.5)
    -- screen point for a COMPASS-relative angle (deg, 0 = top = current heading, clockwise),
    -- with the char-cell aspect baked into Rc:Rr so the ring reads round.
    local function sp(deg, f)
        local a = math.rad(deg)
        return midX + f*Rc*math.sin(a), midY - f*Rr*math.cos(a)
    end

    -- NAV.heading and CRS.dtk live in the atan2(dx,dz) frame (North = 180); convert to a
    -- true compass (N=0 E=90 S=180 W=270) so the number and the letters read correctly.
    local chdg = toCompass(heading)

    -- The ground goes down FIRST, so the bezel, the card and the needles are drawn on top of
    -- it rather than being buried under it. Its ring is the rose's own, and its range comes
    -- from the ground speed (see TERR.courseRange) since a CDI has no scale of its own.
    local trange = TERR.courseRange()
    TERR.paint(W, midX, midY, Rc, Rr, math.max(2, midY - Rr), math.min(H - 1, midY + Rr),
        trange, chdg)

    for a = 0, 357, 2 do local x, y = sp(a, 1.0); hFill(x, y, colours.grey) end     -- ring (bezel)

    local card = {[0]="N", [90]="E", [180]="S", [270]="W"}                          -- heading-up card
    for b = 0, 330, 30 do
        local ang = b - chdg                                                        -- bearing b -> screen
        if card[b] then local x, y = sp(ang, 0.80); hText(x, y, card[b], colours.white)
        else            local x, y = sp(ang, 0.90); hFill(x, y, colours.grey) end
    end

    local cdtk
    if CRS.has then
        cdtk = toCompass(CRS.dtk)
        local cAng = cdtk - chdg
        -- The course arrow, drawn as a two-tone compass needle: the tip that points at the
        -- waypoint is LIT, the tip pointing the opposite way is DARK -- so TO/FROM is read at
        -- a glance from the instrument itself, without hunting for a flag. Both tips grey in
        -- the abeam ambiguity zone (see HSICFG.TOFROM_DEAD).
        local amb  = math.abs(CRS.tf) < HSICFG.TOFROM_DEAD
        local lit  = (CRS.tf >= 0) and cAng or (cAng + 180)   -- the half the waypoint lies on
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
        local defl = math.max(-1, math.min(1, -CRS.xtk / HSICFG.CDI_FULLSCALE_M))
        local ox, oy = defl*(Rc*0.6)*math.sin(pr), -defl*(Rr*0.6)*math.cos(pr)
        local ncol = (math.abs(CRS.xtk) <= HSICFG.CDI_ONCOURSE_M) and colours.lime or colours.yellow
        local n1x, n1y = sp(cAng, 0.5)
        local n2x, n2y = sp(cAng + 180, 0.5)
        hLine(n1x+ox, n1y+oy, n2x+ox, n2y+oy, ncol)                                 -- CDI needle
    end

    hText(midX, midY - Rr - 1, "v", colours.orange)   -- fixed lubber index (top = heading)
    hText(midX, midY, "A", colours.white)             -- aircraft symbol (nose up)

    -- Terrain annunciator and the scale it is painted at, on the row under the top line and
    -- outside the rose. The range matters as much as the colours do: a ground picture with
    -- no scale beside it cannot be read.
    do
        local tl, tc = TERR.label()
        if tl then hText(2, 2, tl, tc) end
        if TERR.range then
            local rt = "R" .. math.floor(TERR.range) .. "m"
            hText(W - #rt + 1, 2, rt, colours.grey)
        end
    end

    local phdg = math.floor(chdg + 0.5) % 360
    if phdg == 0 then phdg = 360 end
    
    hText(midX - 3, 1, string.format("HDG %03d", phdg), colours.orange)
    -- Commanded (scheduled) speed vs actual ground speed -- realtime speed-law debugging.
    if NAV.active then
        if NAV.aligning then
            hText(2, 1, "ALIGN  ", colours.yellow)     -- yawing onto the bearing, throttle held at 0
        elseif DEP.ground() and DEP.phase ~= "ROLL" then
            hText(2, 1, "LINE UP", colours.yellow)     -- a departure, short of its take-off roll
        elseif ATC.holding then
            hText(2, 1, "HOLD   ", colours.orange)     -- ATC hold: stopped (or orbiting) short of the field
        else
            hText(2, 1, string.format("CMD%3.0f", NAV.cmdSpeed), colours.lime)
        end
        hText(W - 7, 1, string.format("GS%4.0f", NAV.groundSpeed or 0), colours.cyan)
    end
    if CRS.has then
        hText(2, H, string.format("%-6s DTK%03d %s", (CRS.tgt or ""):sub(1,6),
            math.floor(cdtk + 0.5) % 360,
            (math.abs(CRS.tf) < HSICFG.TOFROM_DEAD) and "--" or (CRS.tf >= 0 and "TO" or "FR")),
            colours.cyan)
        hText(W - 16, H, string.format("XTK%4.1f%s D%4.0f", math.abs(CRS.xtk), CRS.xtk >= 0 and "R" or "L", CRS.dist), colours.white)
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

    -- The ground first, at the map's own range so the two agree about what the ring means.
    -- It takes itself off below CFG.TERR.MIN_RANGE, which is where this map stops being a
    -- landscape and becomes a close-up of a pad.
    TERR.paint(W, cx, cy, Rx, Ry, mapTop, mapBot, range, chdg)

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
    do
        local tl, tc = TERR.label()
        if tl then hText(2, mapTop, tl, tc) end
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
            -- The far end, drawn as a GLYPH rather than a coloured cell. hText paints its
            -- own black background, so this one mark stays legible over any tint the
            -- heightmap puts under it -- which a plain fill, in any colour, would not.
            if rl >= LND.rwyLen then hText(px, py, "x", colours.orange) end
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

-- Which view the instrument shows, and the two decisions are NOT peers.
--
-- The landing map only earns the screen once a landing is actually engaged -- and once it
-- has, it is the only thing this instrument will show: LND.active is the FMS OVERRIDING the
-- pilot's choice, exactly as the altitude selector's OVER state overrides his altitude, and
-- like that one it is not a preference that can be argued with. uiLoop ignores the toggle
-- outright while it holds, so the stored view cannot change underneath it either.
--
-- The rest of the time the choice is the pilot's: the familiar CDI rose over the terrain, or
-- the route map. CRS.view remembers which, across landings and across go-arounds.
local function drawHSI()
    if LND.active then drawLandingHSI()
    elseif CRS.view == "plan" and HSICFG.PLAN.ENABLE then drawPlanHSI()
    else drawCourseHSI() end
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
    parallel.waitForAny(navLoop, uiLoop, overrideLoop, hsiLoop, NDU.loop, TD.loop,
        apAnnounceLoop, TERR.loop, TFC.loop, ATC.loop)
end)

term.redirect(nativeTerm)
if not ok then
    term.setTextColour(colours.white)
    print("autopilot error: " .. tostring(err))
end
