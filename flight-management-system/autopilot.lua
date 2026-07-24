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

local APNAV_PROTOCOL = "aertogekiss_navap" -- must match navcom.lua's APNAV_PROTOCOL
-- Vertical channel to the Stabiliser+Altitude control during an automatic landing.
-- Must match control.lua's AUTOLAND_PROTOCOL.
local AUTOLAND_PROTOCOL = "aertogekiss_autoland"
-- AS -> FMS: the altitude selector pings this when the pilot taps a panel while it is
-- overshadowed by a landing, asking for ALT-AP back (see overrideLoop / LND.paused).
local ALTOVERRIDE_PROTOCOL = "aertogekiss_altoverride"

local POINTS_FILE = "points.db"
local COURSE_FILE = "course.db"
local LANDING_FILE = "landing.db"

-- Pass radii: how close counts as "at" a waypoint before we advance to the
-- next one. Contextual per navigation-planning convention: VORs are flown
-- past at a distance (fly-by fix), aerodromes/POIs mid-route get a moderate
-- radius, and the very last point of a plan is drawn in tight since that is
-- an actual arrival.
-- ARRIVED_FRACTION: inside this fraction of the final point's own pass radius, the phase sent
-- to navcom becomes ARRIVED rather than TERMINAL. Scales with the same contextual radius as
-- everything else here, so a VOR final gets a looser "arrived" than a POI final does.
-- (Tabled, not one local each -- see the 200-locals note at the top of the tuning section.)
local RAD = {
    VOR_RADIUS         = 240,
    POI_ENROUTE_RADIUS = 120,
    POI_FINAL_RADIUS   = 60,
    ARRIVED_FRACTION   = 0.25,
}

local PING_INTERVAL = 0.2  -- s; how often the current target is re-broadcast. Matches TICK_INTERVAL
                           -- because cruiseCap now carries a LIVE commanded speed (with the nose-
                           -- alignment / cross-track gates folded in), not just a slow keep-alive --
                           -- at 1 s the craft would accelerate for a whole second before a gate bit.
local TICK_INTERVAL = 0.1  -- s; how often position vs target is checked

-- AUTO-LANDING ---------------------------------------------------------------
-- The FMS runs the landing state machine itself: it reads position, heading,
-- altitude and vertical speed straight off the sublevel API and already decides
-- flight phase, so deciding when to align / translate / descend / flare / cut is
-- the same job one step further. It drives NAVCOM horizontally (position-hold over
-- the pad plus hold the landing heading, via a new "LANDING" phase) and the
-- Stabiliser+Altitude control vertically (a commanded descent rate, via
-- AUTOLAND_PROTOCOL). Landing sites and their approach heading/altitude come from
-- landing.db.

-- Landing gates/profile, tabled rather than one local each (see the 200-locals note below).
--   ALIGN/FINE_HDG_DEG, POS_TOL_M : step 1 coarse heading, then step 3's tight heading+position
--                                   that begins the descent.
--   RECAPTURE_*  : once descending, allow a little more slop before we call it "drifted" and
--                  pause to re-capture the pad -- the descent burn nods the craft and the
--                  vertical thrusters have no vectoring, so X/Z gets shoved.
--   DESCEND_*/FLARE_AGL/TOUCHDOWN_AGL : descent profile, quick up high easing to a gentle rate.
--                  Keep DESCEND_FAST within what the vertical channel can fly (its DESCEND_MAX).
--   TD_SETTLE_*  : touchdown backup for when the pad altitude is a touch off -- if we are
--                  commanding descent but the craft stopped sinking near the pad, that is a landing.
local LG = {
    ALIGN_HDG_DEG = 15, FINE_HDG_DEG = 2, POS_TOL_M = 3,
    RECAPTURE_M = 4, RECAPTURE_HDG_DEG = 8,
    DESCEND_FAST = 3.0, DESCEND_SLOW = 0.8, FLARE_AGL = 10.0, TOUCHDOWN_AGL = 1.0,
    TD_SETTLE_VEL = 0.15, TD_SETTLE_AGL = 2.0, TD_SETTLE_TICKS = 3,
}

-- Heading measurement -- MUST match navcom.lua's HEADING_SIGN / HEADING_OFFSET,
-- since this is the same aircraft and the same sublevel orientation. If navcom's
-- convention is ever recalibrated, change it here too: NAVCOM would still fly the
-- craft to the right heading (it owns those constants), but this alignment gate
-- would open at the wrong moment.
local HEADING_SIGN   = 1
local HEADING_OFFSET = math.pi

-- CRUISE SPEED ---------------------------------------------------------------
-- Pilot-selectable cruise-speed cap, sent to NAVCOM as msg.cruiseCap and applied only
-- by its ENROUTE law (so the status-screen button "only works in cruising mode"). Three
-- presets; COAST is a plain 0.
local SPEED_FULL = 110   -- m/s "FULL" cruise (kept below navcom's CRUISE_SPEED ceiling) Choose fastest speed that does not outrun chunk loading
local SPEED_SLOW = 40   -- m/s "SLOW"; mirror of navcom.lua's APPROACH_SPEED
-- Cycle order when the button is tapped.
local NEXT_CRUISE = { full = "slow", slow = "coast", coast = "full" }

-- DYNAMIC CRUISE (kinematic braking envelope) --------------------------------
-- SPEED_FULL / SPEED_SLOW are a soft max/min band. Each upcoming fix has a `need` speed (a turn's
-- corner speed, or DC.FINAL_ARRIVE_SPEED at the destination; a gentle fly-by needs full cruise and
-- never slows). dynamicSpeedTarget commands the fastest speed from which the craft can still bleed
-- to every upcoming need in time, braking at DC.BRAKE_DECEL -- envelope sqrt(need^2 + 2*DECEL*dist),
-- min over fixes scanned by DISTANCE (so a sharp turn several fixes ahead is caught early). Braking
-- distance therefore scales with the speed to be shed (~v^2), which a flat proximity distance could
-- not -- it braked too late up high and too early down low.
--
-- This function is the SINGLE source of commanded speed: the manoeuvre gates (nose alignment,
-- cross-track) are applied HERE too, not in the UAC, so the CMD on the HSI is exactly what the
-- aircraft is flown to. The UAC just tracks it as cruiseCap (schedSpeed=true).
-- ROUTE START: point the nose at the first target BEFORE applying any throttle. While the
-- heading error to the target exceeds LG.ALIGN_HDG_DEG the commanded speed is 0, so the craft
-- simply yaws onto the bearing instead of accelerating off on the old heading and arcing round.
-- Cleared for good once aligned (per plan, not per leg).
--
-- ADVANCE_*_MULT -- turn-aware pass radius (corner cutting): advance to the next leg early on a
-- gentle turn so the craft flies a smooth fly-by without having to reach the fix, and hold in
-- tighter on a sharp turn so it does not overshoot. 1.0 / 1.0 disables (plain radiusFor).
-- Intermediate waypoints only; the destination is always drawn in tight.
--
-- NOTE (200-locals): CC's Lua VM caps a function at 200 locals and counts a nested function's
-- own locals ON TOP of the enclosing scope's, so late/large functions (drawLandingHSI etc.) blow
-- up once the file has too many TOP-LEVEL locals. Keep tuning knobs in tables like this one --
-- add new ones as DC.X, never `local X`. (luac -p does NOT model this; it will pass regardless.)
local DC = {
    DYNAMIC_CRUISE = true,         -- false -> fall back to the flat pilot cruise cap
    CURVE          = true,         -- en-route: steer to a carrot on a Catmull-Rom curve through the
                                   -- waypoints (smooth continuous bearing, rounded corners) instead of
                                   -- straight at the fix. false -> aim straight at the waypoint.
    CURVE_LOOKAHEAD = 400,         -- m: how far ahead on the curve the carrot sits. Larger = smoother /
                                   -- cuts corners more; smaller = tracks the waypoints tighter.
    CURVE_DEV_MAX  = 120,          -- m: soft cap on how far the curve may bow OFF the straight leg. The
                                   -- Catmull-Rom bow scales with leg length (~14 m on a 200 m leg, ~270 m
                                   -- on a 4 km leg), so this reins in km-apart waypoints without touching
                                   -- finely-placed ones (whose bow is already << this). Smooth, endpoint-
                                   -- exact, pulls only toward the chord. Raise to allow rounder corners.
    A_LAT_MAX      = 20,           -- m/s^2: max lateral accel used for the CURVATURE speed limit. Speed
                                   -- through a bend of radius R is sqrt(A_LAT_MAX*R). Higher = faster
                                   -- through bends (needs more strafe/bank authority); lower = slower.
    CURVE_V_MIN    = 10,           -- m/s: floor on the curvature limit, so a Catmull-Rom curvature spike
                                   -- at a sharp/close waypoint can't command a near-stop mid-route.
    BRAKE_DECEL    = 8,            -- m/s^2: deceleration the speed profile plans with. Braking distance
                                   -- to a slow point = (v^2 - v_target^2)/(2*DECEL): at 3 it was ~2000 m
                                   -- to a stop from 110 (half of a 4000 m leg spent slowing!); 8 -> ~750 m.
                                   -- The profile ALREADY cruises the middle of a long leg -- this only
                                   -- sets how LATE/short the braking into a turn or the destination is.
                                   -- Set it to what the craft can REALLY brake at: too high and it
                                   -- overshoots the turn/pad (arrives hot); too low and it crawls in early.
    TURN_GENTLE    = math.rad(12), -- turn <= this: full advance radius (fly-by); see ADVANCE_*_MULT.
    TURN_SHARP     = math.rad(90), -- turn >= this: tight advance radius. (Speed is curvature-based now.)
    FINAL_ARRIVE_SPEED = 12,       -- commanded speed AT the destination; TERMINAL takes over
    ACCEL_MAX      = 8,            -- m/s^2: max rate the COMMANDED speed may RISE (NAV.cmdSpeed slew).
                                   -- Eases the throttle up OUT of a turn instead of jumping to cruise
                                   -- while still yawing onto the new leg. Lower = gentler spool-up.
    DECEL_MAX      = 12,           -- m/s^2: max rate the COMMANDED speed may FALL. Rounds off sudden
                                   -- steps in the raw envelope into a smooth curve. Keep >= BRAKE_DECEL
                                   -- so real slowdowns aren't delayed; lower = smoother but slows later.
    XTK_SLOW_LO    = 20,           -- m off the leg: below this, no cross-track slowdown
    XTK_SLOW_HI    = 80,           -- m off the leg: at/above this, the full XTK_SLOW_MIN cut
    XTK_SLOW_MIN   = 0.30,         -- floor speed fraction when far off the leg (fly in to intercept)
    ADVANCE_GENTLE_MULT = 1.3333,  -- gentle turn: advance this * base pass radius (cut the corner)
    ADVANCE_SHARP_MULT  = 0.7,     -- sharp turn: hold in to this * base radius
    TURN_GAMMA = 1.732 -- the nonlinearity for slowdown per angle. 1=linear, <1=harder break on small turns, >1=lax breaking on small turns
}

-- RESUME MID-ROUTE -----------------------------------------------------------
-- FLY ROUTE can join a plan at the nearest sensible waypoint instead of always flying it
-- from the first point (see bestResumeIndex). "Nearest" is heading-aware: a waypoint behind
-- the aircraft costs a U-turn, so its straight-line distance is inflated -- by this weight
-- times the backward component of the displacement -- before the cheapest waypoint is
-- chosen. So the penalty in metres = weight * (how far behind us the waypoint is): a
-- waypoint dead astern at distance d is judged as costing d*(1+weight); one abeam or ahead
-- pays nothing. At 1.0 a waypoint dead behind must be nearer than HALF the distance of one
-- dead ahead to still win -- i.e. turning back has to be substantially cheaper. Raise it to
-- lean harder toward "press on"; drop toward 0 to just pick the geometrically nearest.
local RESUME_BACKTRACK_WEIGHT = 1.0

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
            if name then
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

local function isLandingSite(name)
    return LANDING[name] ~= nil
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
    posY = 0, velY = 0, heading = 0, haveHeading = false, groundSpeed = 0,
    cmdSpeed = 0,   -- rise-rate-limited commanded speed (dynamicSpeedTarget through DC.ACCEL_MAX);
                    -- the value actually sent as cruiseCap and shown as CMD, updated once per navTick.
    -- Route start: hold throttle at 0 until the nose is within LG.ALIGN_HDG_DEG of the first target.
    aligning = false,
    aimX = nil, aimZ = nil,   -- the point broadcastCurrent last steered to (carrot or waypoint)
}

-- Auto-landing sequence state. Distinct from NAV: the plan can be flown (NAV) long
-- before the landing (LND) actually engages on arrival, and the landing outlives the
-- plan by a moment at touchdown (NAV is cancelled, LND lingers on the LANDED screen).
local LND = {
    active = false,
    phase  = "OFF",     -- OFF | ALIGN | TRANSLATE | DESCEND | FLARE | LANDED
    paused = false,     -- pilot overrode the descent (AS override): hand VERTICAL to
                        -- ALT-AP while position/heading alignment keeps running. Set by
                        -- overrideLoop, cleared by RESUME LANDING / any teardown.
    x = 0, z = 0, name = "",
    holdHeading = nil,  -- radians, or nil when the site's direction is 'x' (any)
    siteAlt = 0, siteType = "heli",
    vs = 0,             -- current commanded vertical speed (m/s, negative = down)
    agl = 0, posErr = 0, hdgErrDeg = 0,
    settle = 0,         -- consecutive "not sinking" ticks, for the touchdown backup
}

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

-- CURVE: the Catmull-Rom trajectory through the waypoints, shared by the carrot (bearing) guidance
-- and the curvature-based speed profile -- so the path the craft flies and the speed it flies are
-- ONE computer-drawn trajectory. Global parameter g: floor = segment pts[seg]->pts[seg+1], frac =
-- position within it; neighbour points clamped at the route ends.
local CURVE = {}

-- CENTRIPETAL (alpha=0.5) Catmull-Rom, Barry-Goldman evaluation. Unlike the uniform (alpha=0)
-- form, it never makes cusps or self-intersecting loops on unevenly-spaced or sharp waypoints
-- (confirmed on real paths). Knot spacing = chord^0.5; the point is a pyramid of lerps over knots.
function CURVE.at(g)
    local pts, n = NAV.points, #NAV.points
    local seg = math.floor(g)
    if seg < 1 then seg = 1 elseif seg > n - 1 then seg = n - 1 end
    local u = math.max(0, math.min(1, g - seg))
    local p0, p1, p2, p3 = pts[math.max(1, seg-1)], pts[seg], pts[seg+1], pts[math.min(n, seg+2)]
    local function knot(a, b)
        local dx, dz = b.x - a.x, b.z - a.z
        local k = math.sqrt(math.sqrt(dx*dx + dz*dz))    -- |b-a|^0.5 (centripetal)
        return (k > 1e-6) and k or 1e-6                   -- keep knots distinct on duplicate points
    end
    local t0 = 0
    local t1 = t0 + knot(p0, p1)
    local t2 = t1 + knot(p1, p2)
    local t3 = t2 + knot(p2, p3)
    local t  = t1 + u * (t2 - t1)
    local function lp(ta, tb, a, b) return (a*(tb - t) + b*(t - ta)) / (tb - ta) end
    local a1 = lp(t0,t1, p0.x, p1.x); local a2 = lp(t1,t2, p1.x, p2.x); local a3 = lp(t2,t3, p2.x, p3.x)
    local x  = lp(t1,t2, lp(t0,t2, a1, a2), lp(t1,t3, a2, a3))
    a1 = lp(t0,t1, p0.z, p1.z); a2 = lp(t1,t2, p1.z, p2.z); a3 = lp(t2,t3, p2.z, p3.z)
    local z  = lp(t1,t2, lp(t0,t2, a1, a2), lp(t1,t3, a2, a3))
    -- Soft-cap the off-course BOW: limit the point's deviation from the straight leg to ~CURVE_DEV_MAX
    -- so a km-long segment can't swing wide, while short segments (bow << cap) pass through unchanged.
    -- Smooth, pulls only toward the chord (no loops), and exactly 0 at the waypoints (still passes through).
    local lx, lz = p1.x + u*(p2.x - p1.x), p1.z + u*(p2.z - p1.z)
    local dx, dz = x - lx, z - lz
    local dev = math.sqrt(dx*dx + dz*dz)
    if dev > 1e-6 then
        local sc = 1 / math.sqrt(1 + (dev / DC.CURVE_DEV_MAX)^2)   -- ~1 for small dev, ->cap/dev for large
        x, z = lx + dx*sc, lz + dz*sc
    end
    return x, z
end

-- The aircraft's parameter on the curve (projection onto the current straight leg).
function CURVE.craftG()
    local pts = NAV.points
    local seg = math.max(1, NAV.index - 1)
    local a, b = pts[seg], pts[seg + 1]
    if not (a and b) then return seg end
    local dx, dz = b.x - a.x, b.z - a.z
    local ll = dx * dx + dz * dz
    local u0 = (ll > 1e-9) and math.max(0, math.min(1, ((NAV.posX - a.x)*dx + (NAV.posZ - a.z)*dz) / ll)) or 0
    return seg + u0
end

-- HSI course from the curve at the aircraft's position: the tangent bearing (DTK) and the signed
-- perpendicular deviation (+=right of course, same sign as the leg CDI), or nil if unusable. So
-- the dial shows the smoothed path as the course, reading on-course when the craft is on the curve.
function CURVE.course()
    if #NAV.points < 2 then return nil end
    local g = CURVE.craftG()
    local cx, cz = CURVE.at(g)
    local fx, fz = CURVE.at(math.min(g + 0.01, #NAV.points))
    fx, fz = fx - cx, fz - cz
    local tl = math.sqrt(fx*fx + fz*fz)
    if tl < 1e-6 then return nil end
    return atan2(fx, fz), (fx*(NAV.posZ - cz) - fz*(NAV.posX - cx)) / tl
end

-- Menger curvature (1/radius, 1/m) of three points -- 0 if degenerate/collinear.
function CURVE.curvature(x1, z1, x2, z2, x3, z3)
    local cross = math.abs((x2-x1)*(z3-z1) - (x3-x1)*(z2-z1))          -- 2 * triangle area
    local a = math.sqrt((x3-x2)*(x3-x2) + (z3-z2)*(z3-z2))
    local b = math.sqrt((x3-x1)*(x3-x1) + (z3-z1)*(z3-z1))
    local c = math.sqrt((x2-x1)*(x2-x1) + (z2-z1)*(z2-z1))
    local d = a * b * c
    return (d > 1e-9) and (2 * cross / d) or 0
end

-- Carrot: `lookahead` m ahead of the aircraft along the curve (guidance / bearing). Straight to
-- the first fix at route start (no curve behind it yet).
function CURVE.carrot(lookahead)
    local pts, n = NAV.points, #NAV.points
    if NAV.index < 2 or n < 2 then
        local p = pts[math.max(1, math.min(n, NAV.index))]
        return p.x, p.z
    end
    local g = CURVE.craftG()
    local px, pz = CURVE.at(g)
    local acc = 0
    while acc < lookahead and g < n do
        g = math.min(g + 0.02, n)
        local nx, nz = CURVE.at(g)
        acc = acc + math.sqrt((nx-px)*(nx-px) + (nz-pz)*(nz-pz))
        px, pz = nx, nz
    end
    return px, pz
end

-- Curvature-limited commanded speed: walk the curve forward from the aircraft and run a FORWARD
-- BRAKING PASS. The local limit at each point is sqrt(A_LAT_MAX / kappa) (a tight bend is slow),
-- floored at CURVE_V_MIN so a curvature spike can't stall us; the route END limits to
-- FINAL_ARRIVE_SPEED. The fastest we can be NOW is min over the scan of sqrt(vlim^2 + 2*DECEL*s),
-- so speed bleeds off BEFORE a bend and holds cruise on straights -- the speed curve OF the path.
function CURVE.speed(capBase)
    local n = #NAV.points
    if n < 2 then return capBase end
    local scan = capBase * capBase / (2 * DC.BRAKE_DECEL)          -- full-cruise braking distance
    local best = capBase
    local g = CURVE.craftG()
    local x1, z1 = CURVE.at(g)
    -- seed s with the craft -> curve-start distance: ~0 mid-route (we sit on the curve), but real
    -- at route start where we're still far from the first fix -- so a bend just past it isn't
    -- treated as right on top of us.
    local s = math.sqrt((NAV.posX-x1)*(NAV.posX-x1) + (NAV.posZ-z1)*(NAV.posZ-z1))
    g = math.min(g + 0.02, n)
    local x2, z2 = CURVE.at(g)
    s = s + math.sqrt((x2-x1)*(x2-x1) + (z2-z1)*(z2-z1))           -- arc length to x2 (the sample point)
    while s < scan and g < n do
        g = math.min(g + 0.02, n)
        local x3, z3 = CURVE.at(g)
        local vlim
        if g >= n then
            vlim = DC.FINAL_ARRIVE_SPEED                           -- the route end point
        else
            local k = CURVE.curvature(x1, z1, x2, z2, x3, z3)
            vlim = (k > 1e-6) and math.max(DC.CURVE_V_MIN, math.sqrt(DC.A_LAT_MAX / k)) or capBase
        end
        local allow = math.sqrt(vlim * vlim + 2 * DC.BRAKE_DECEL * s)
        if allow < best then best = allow end
        s = s + math.sqrt((x3-x2)*(x3-x2) + (z3-z2)*(z3-z2))
        x1, z1, x2, z2 = x2, z2, x3, z3
    end
    return clamp(best, 0, capBase)
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

-- THE commanded along-track speed = the CURVATURE speed profile of the smoothed path (CURVE.speed:
-- slow through tight bends, cruise on straights, bled off IN ADVANCE by a forward braking pass),
-- plus the destination arrival brake on the final leg, trimmed by the cross-track gate. Because it
-- reads the SAME Catmull-Rom curve the bearing follows, the speed and the path are one trajectory --
-- the speed slows exactly as much as the rounded curve needs, not for the raw waypoint angle. Zero
-- while aligning; bounded by the pilot's cap. (A_LAT_MAX = how fast through a bend; BRAKE_DECEL =
-- how early it starts; CURVE_V_MIN = floor at a curvature spike.)
local function dynamicSpeedTarget()
    local capBase = cruiseCapValue()
    if not DC.DYNAMIC_CRUISE or capBase <= 0 then return capBase end
    local total = #NAV.points
    if total == 0 then return capBase end
    if NAV.aligning then return 0 end          -- route start: yaw onto the bearing before throttling

    local best = CURVE.speed(capBase)          -- curvature forward braking pass (bends + the endpoint)

    -- Destination arrival on the FINAL leg (also covers a single-point direct-to, where the curve
    -- has nothing to walk): brake straight-line to FINAL_ARRIVE_SPEED at the last fix.
    if NAV.index == total then
        local last = NAV.points[total]
        local dx, dz = last.x - NAV.posX, last.z - NAV.posZ
        local dd = math.sqrt(dx * dx + dz * dz)
        best = math.min(best, math.sqrt(DC.FINAL_ARRIVE_SPEED * DC.FINAL_ARRIVE_SPEED + 2 * DC.BRAKE_DECEL * dd))
    end

    -- Cross-track: well off COURSE -> slow to fly IN and intercept. When curve-following, "course"
    -- is the CURVE, not the straight leg: a craft correctly ON the curve is off the LEG by the bow
    -- (up to CURVE_DEV_MAX), which would FALSELY trip this. Measure deviation from the curve there.
    local xtk
    if DC.CURVE and NAV.phase == "ENROUTE" and NAV.index >= 2 and NAV.index ~= total then
        local _, cxtk = CURVE.course()
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
    -- En-route we hand the UAC a carrot on the smoothed curve (smooth bearing, rounded corners).
    -- The precise capture phases (APPROACH/TERMINAL/ARRIVED) and the final fix keep aiming at the
    -- real waypoint, unchanged -- that's a different, precise job and it works.
    local tx, tz, ptype, carrot
    if DC.CURVE and NAV.phase == "ENROUTE" and not isFinal then
        tx, tz = CURVE.carrot(DC.CURVE_LOOKAHEAD)
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
    }, APNAV_PROTOCOL)
end

-- Vertical command to the Stabiliser+Altitude control during a landing. Streamed every
-- tick (not just on the 1 s ping) so the flare is smooth as LND.vs eases with altitude.
local function broadcastVertical()
    if not LND.active then return end
    if LND.paused then
        -- Pilot override: hand the vertical channel back to ALT-AP. The UAC treats
        -- "release" like a cancel of the vertical (tracks altap again) while we keep
        -- streaming the LANDING navap (broadcastCurrent) so alignment stays online.
        rednet.broadcast({ cmd = "release" }, AUTOLAND_PROTOCOL)
        return
    end
    local p = LND.phase
    if p == "ALIGN" or p == "TRANSLATE" then
        -- Hold altitude and take ALT-AP offline while we line up over the pad.
        rednet.broadcast({ cmd = "hold" }, AUTOLAND_PROTOCOL)
    elseif p == "DESCEND" or p == "FLARE" then
        rednet.broadcast({ cmd = "descend", vs = LND.vs }, AUTOLAND_PROTOCOL)
    elseif p == "LANDED" then
        rednet.broadcast({ cmd = "landed" }, AUTOLAND_PROTOCOL)
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

-- Arm the landing sequence. Called once, when a landAtEnd plan has genuinely arrived at
-- its final (landing) point. Everything it needs was snapshotted into NAV.landSite when
-- the plan was set, so a mid-flight landing.db edit cannot move the landing under us.
local function engageAutoland()
    local site = NAV.landSite
    local fp   = NAV.points[#NAV.points]
    LND.active      = true
    LND.phase       = "ALIGN"
    LND.paused      = false
    LND.x, LND.z    = fp.x, fp.z
    LND.name        = site.name
    LND.holdHeading = headingForDir(site.dir)
    LND.siteAlt     = site.alt
    LND.siteType    = site.type
    LND.vs          = 0
    LND.settle      = 0
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

        -- Touchdown: primarily from AGL (pad altitude is known); backed up by "commanding
        -- descent but no longer sinking" for when the configured pad altitude is a bit off.
        local touchdown = (agl <= LG.TOUCHDOWN_AGL)
        if not touchdown and LND.vs < 0 and agl <= LG.TD_SETTLE_AGL
                and math.abs(NAV.velY) < LG.TD_SETTLE_VEL then
            LND.settle = LND.settle + 1
            if LND.settle >= LG.TD_SETTLE_TICKS then touchdown = true end
        elseif LND.vs < 0 and math.abs(NAV.velY) >= LG.TD_SETTLE_VEL then
            LND.settle = 0
        end

        if touchdown then
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
        NAV.landSite  = { name = finalName, dir = li.dir, alt = li.alt, type = li.type }
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
    NAV.cruiseMode = "full"   -- a fresh plan starts at full cruise, never a stale COAST
    NAV.aligning   = true     -- ...and points the nose at the first target before throttling up
    NAV.planName = landAtEnd and "LAND" or "DIRECT"
    NAV.points = {{name = p.name, x = p.x, z = p.z, kind = p.kind}}
    NAV.index = 1
    NAV.active = true
    setLandFinal(name, landAtEnd)
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
    NAV.cruiseMode = "full"   -- a fresh plan starts at full cruise, never a stale COAST
    NAV.aligning   = true     -- ...and points the nose at the first target before throttling up
    NAV.planName = name
    NAV.points = pts
    NAV.index = resume and bestResumeIndex(pts) or 1
    NAV.active = true
    -- A route ending at a landing site auto-lands there on arrival (fully automatic).
    setLandFinal(pts[#pts].name, true)
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
    NAV.cruiseMode = "full"    -- a fresh plan starts at full cruise, never a stale COAST
    NAV.aligning   = true      -- ...and points the nose at the first target before throttling up
    NAV.planName = "TO " .. destName
    NAV.points = pts
    NAV.index = bestResumeIndex(pts)     -- heading-aware mid-route join, same as RESUME
    NAV.active = true
    -- The destination is a hub landing site: auto-land on arrival, same as a route to it.
    setLandFinal(pts[#pts].name, true)
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
    if LND.active then
        rednet.broadcast({cmd = vertCmd or "cancel"}, AUTOLAND_PROTOCOL)
    end
    LND.active = false
    LND.phase  = "OFF"
    LND.paused = false
    NAV.active = false
    NAV.aligning = false
    NAV.cmdSpeed = 0
    NAV.landAtEnd = false
    NAV.landSite  = nil
    NAV.planName = nil
    NAV.points = {}
    NAV.index = 1
    NAV.phase = "ENROUTE"
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
    NAV.groundSpeed = math.sqrt(vel.x * vel.x + vel.z * vel.z)   -- horizontal speed, for the HSI readout

    if LND.active then
        -- The landing owns the aircraft now: run its state machine and stream the vertical
        -- command; skip normal waypoint advance entirely.
        landingTick()
        broadcastVertical()
        return
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
        -- DC.DECEL_MAX >= the envelope's own decel (~BRAKE_DECEL) so genuine slowdowns aren't
        -- delayed -- it only rounds off the sudden steps. This is the single CMD value.
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
        if dist <= advanceRadiusFor(NAV.index, tgt, isFinal) and not isFinal then
            NAV.index = NAV.index + 1
            updatePhase() -- against the point we just switched to, not the one we passed
            broadcastCurrent() -- resend immediately on advance, not just on the 1s tick
        else
            updatePhase()
            -- Arrived at a landing-capable final point: engage the landing sequence
            -- automatically (fully automatic, per config).
            if isFinal and NAV.landAtEnd and NAV.phase == "ARRIVED" then
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
    term.setCursorPos(2, 1); term.write("AUTOLAND " .. LND.name)
    term.setCursorPos(math.max(2, w - #LND.phase), 1); term.write(LND.phase)

    term.setTextColour(colours.white)
    term.setCursorPos(2, 3)
    term.write(string.format("POS  X %.1f  Z %.1f", NAV.posX, NAV.posZ))
    term.setCursorPos(2, 4)
    term.write(string.format("POS ERR  %5.1f m  (<=%d)", LND.posErr, LG.POS_TOL_M))
    term.setCursorPos(2, 5)
    if LND.holdHeading then
        term.write(string.format("HDG ERR %+5.1f deg", LND.hdgErrDeg))
    else
        term.write("HDG ERR   -- (any)")
    end
    term.setCursorPos(2, 6)
    term.write(string.format("AGL  %6.1f m", LND.agl))
    term.setCursorPos(2, 7)
    term.write(string.format("VS   %+5.2f m/s", LND.vs))

    local hint = {
        ALIGN     = "aligning heading...",
        TRANSLATE = "closing on the pad...",
        DESCEND   = "descending",
        FLARE     = "flare - easing for touchdown",
        LANDED    = "LANDED - engines on ground hold. Disengage AP NOW",
    }
    local hintText = LND.paused and "PILOT ALT-AP -- descent PAUSED" or (hint[LND.phase] or "")
    term.setTextColour((LND.phase == "LANDED" or LND.paused) and colours.lime or colours.orange)
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
        if sender and LND.active then
            local p = LND.phase
            if p == "ALIGN" or p == "TRANSLATE" or p == "DESCEND" or p == "FLARE" then
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
    CDI_FULLSCALE_M = 15,   -- metres of cross-track at full needle deflection
    CDI_ONCOURSE_M  = 1.5,  -- inside this the needle turns green
    -- LANDING mode (map):
    HDG_BAR_FS_DEG  = 45,   -- heading error (deg) at full-width bar deflection
    HDG_ALIGN_DEG   = 5,    -- within this the bar marker turns green
    -- Map range steps (metres to the ring edge): the smallest that still frames the target
    -- wins, so the target creeps inward as you close and the scale only jumps occasionally.
    RANGES          = {5, 10, 25, 50, 100, 250, 500, 1000, 2500, 5000},
    DEFAULT_RANGE   = 50,   -- ring scale when there is no target to frame
}

local hsiMon = peripheral.find("monitor")
if hsiMon then
    hsiMonName = peripheral.getName(hsiMon)
    hsiMon.setTextScale(0.5)
    pcall(function()
        hsiMon.setPaletteColour(colours.cyan,   0x33bbdd)
        hsiMon.setPaletteColour(colours.orange, 0xffbb00)
        hsiMon.setPaletteColour(colours.grey,   0x555555)
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
        -- Curve-following: show the CURVE as the course, not the straight leg, so the dial reads
        -- what the aircraft actually flies (and on-course when it's on the curve). DTK = the curve
        -- TANGENT at our projection; XTK/CDI = signed perpendicular deviation FROM the curve (same
        -- +=right-of-course sign as the leg version). DIST stays to the real waypoint.
        if DC.CURVE and NAV.phase == "ENROUTE" and NAV.index >= 2 and NAV.index ~= #NAV.points then
            local cdtk, cxtk = CURVE.course()   -- curve tangent (DTK) + signed deviation FROM the curve
            if cdtk then dtk, xtk = cdtk, cxtk end
        end
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
        local a1x, a1y = sp(cAng + 180, 0.95)
        local a2x, a2y = sp(cAng, 0.95)
        hLine(a1x, a1y, a2x, a2y, colours.cyan)                                     -- course line
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
    hText(midX, midY, "^", colours.white)             -- aircraft symbol (nose up)

    hText(midX - 3, 1, string.format("HDG %03d", math.floor(chdg + 0.5) % 360), colours.orange)
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
        hText(2, H, string.format("%-6s DTK%03d", (tgtName or ""):sub(1,6), math.floor(cdtk + 0.5) % 360), colours.cyan)
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
        if landing and desired then                              -- pad's approach axis (landing heading)
            local ax = desired - chdg
            local ex, ey = Rx*0.16*math.sin(math.rad(ax)), -Ry*0.16*math.cos(math.rad(ax))
            hLine(sx - ex, sy - ey, sx + ex, sy + ey, colours.orange)
        end
        hText(sx, sy, tgtGlyph, tgtCol)
    end
    hText(cx, cy, "^", colours.white)                            -- aircraft, always nose-up

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
    if landing then
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
    while true do pcall(drawHSI); sleep(0.2) end
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
    term.setPaletteColour(colours.orange, 0xffbb00)
    term.setPaletteColour(colours.brown, 0x553322)
    term.setPaletteColour(colours.green, 0x116611)
end

local ok, err = pcall(function()
    parallel.waitForAny(navLoop, uiLoop, overrideLoop, hsiLoop)
end)

term.redirect(nativeTerm)
if not ok then
    term.setTextColour(colours.white)
    print("autopilot error: " .. tostring(err))
end
