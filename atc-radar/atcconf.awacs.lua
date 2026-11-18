-- AER TOGEKISS AWACS -- CONFIGURATION
--
-- The same radar program as the TMU tower (#44) runs here, centred on this aircraft instead
-- of on a patch of ground. Everything that differs between a tower and an airborne early
-- warning console is in this file: MODE, the heading convention, which callsign is us, and a
-- set of ranges chosen for a 320 m/s aeroplane rather than a circuit.
--
-- Same split, and for the same reasons, as the FMS's apconf.lua and the UAC's pid/uacconf.lua:
-- a program update never overwrites this station's setup, and the knobs are editable without
-- scrolling the program. THERE ARE DELIBERATELY NO FALLBACK COPIES OF THESE VALUES IN atc.lua
-- -- a missing knob is a named BOOT error, and which knobs are required depends on MODE.
--
-- Loaded by `require("atcconf")`, which resolves because startup.lua runs shell.run("atc.lua")
-- and so the program directory is this one. Must end with `return CFG`.

local CFG = {}

-- ============================================================================
-- THE STATION -- which is an aeroplane. There are no coordinates here on purpose: a
-- dynamic station reads its position, heading and velocity off sublevel every frame, and
-- atc.lua does not look at X/Z/Y in this mode at all.
-- ============================================================================

CFG.STATION = {
    NAME  = "",                       -- tag drawn beside the centre symbol. Blank: own-ship is
                                      -- named in the header already and the middle of an
                                      -- airborne scope is the busiest part of the screen.
    LABEL = "AER TOGEKISS AWACS",     -- shown in the header bar
    MODE  = "dynamic",

    -- Which compass direction is drawn at the TOP of the scope:
    --   "heading" -- where the nose points. What is at the top of the screen is what is out
    --                of the windscreen, which is the whole reason an airborne scope turns.
    --   "track"   -- where the aircraft is actually GOING. On a VTOL these two part company
    --                routinely: Aer Togekiss will hold a heading and translate sideways.
    --   n/e/s/w or a number -- a fixed orientation, ignoring the aircraft entirely.
    -- Heading, because this is a lookout console: the operator is matching what is on the
    -- scope to what is out of the window, and the window follows the nose. The [NUP] button
    -- forces north-up at any time without editing this.
    ORIENTATION = "heading",

    -- Heading measurement. MUST MATCH this airframe's responder.lua and apconf.lua -- when
    -- the AWACS and the responder bolted to the same hull disagree about which way it is
    -- pointing, one of them is lying to the fleet. Aer Togekiss's logical forward is 180
    -- degrees from its thrust-forward, hence the offset.
    HEADING_SIGN   = 1,
    HEADING_OFFSET = math.pi,

    -- OURSELVES, as our own responder (#45) announces us on atc_broadcast. Reports carrying
    -- this callsign are dropped instead of plotted. Without it the aircraft paints itself as
    -- a contact permanently underneath its own symbol -- and auto-range, seeing traffic at
    -- zero metres, would sit at the closest scale for ever.
    OWN_CALLSIGN = "TOGEKISS",

    -- THE ALTITUDE DATUM -- how far the pose `sublevel` reports sits ABOVE THE BOTTOM OF THE
    -- HULL, in metres. Dynamic mode only: a static station's CFG.STATION.Y is a surveyed
    -- tower elevation and needs no correction. MUST MATCH this airframe's responder.lua,
    -- apconf.lua, gpwsconf.lua and pid/uacconf.lua -- the scope's own ALT readout and the
    -- altitudes of the traffic around it have to be the same datum, or the vertical
    -- separation an operator reads off this screen is wrong by the height of this aeroplane.
    ALT_OFFSET   = 9,   -- measured 2026-08-29: parked on the Y=63 airstrip, it read alt 73
}

-- ============================================================================
-- LINKS -- who this computer listens to. Both names are FLEET-WIDE and must not be
-- renamed per aircraft, so they are the same two the tower uses.
-- ============================================================================

CFG.LINKS = {
    ATC     = "atc_broadcast",  -- responders -> here. Must match responder.lua's ATC_CAST.
    NAVDATA = "aer_navdata",    -- nav data server -> here. Must match nav_update_server.lua.
}

-- ============================================================================
-- NAVIGATION DATABASE -- read only, and kept current by subscribing to #24 exactly as an
-- FMS does. That matters more airborne than it does in a tower: this aircraft flies out of
-- range of the fleet's home field, and the map under the contacts has to be the same one
-- the aircraft around it are navigating by.
-- ============================================================================

CFG.POINTS_FILE  = "points.db"
CFG.COURSE_FILE  = "course.db"
CFG.LANDING_FILE = "landing.db"

CFG.NAVDATA = {
    ENABLE      = true,
    PULL_EVERY  = 5,
    STALE_AFTER = 90,
    TMP         = ".new",
}

-- ============================================================================
-- TARGETS.
-- ============================================================================

CFG.TRACK = {
    COAST_AFTER = 3,
    DROP_AFTER  = 20,
    -- Longer than the tower's, and it can afford to be: this scope works in tens of
    -- kilometres, so a 45 s leader is a readable fraction of the picture rather than a line
    -- off the edge of it. The fraction cap below still governs at close range.
    LEADER_SEC  = 45,
    LEADER_MAX_FRAC = 0.35,
    TRAIL_LEN   = 8,
    TRAIL_EVERY = 2.0,      -- twice the tower's interval, so a trail covers twice the ground
    FIX_STALE_AFTER = 2,    -- s without a sublevel fix before the header says the picture is
                            -- old. This one is real here: lose sublevel and every range and
                            -- bearing on the screen is being measured from a stale origin.
}

-- ============================================================================
-- THE SCOPE. Wider than the tower's throughout -- this is a lookout, not an approach aid.
-- ============================================================================

CFG.MAP = {
    RANGES        = {500, 1000, 2500, 5000, 10000, 25000, 50000, 100000},
    DEFAULT_RANGE = 10000,
    RINGS         = 3,
    AUTO_RANGE    = true,
    AUTO_MARGIN   = 1.15,
    SHOW_WAYPOINT = false,
    SHOW_ROUTE    = true,   -- at these ranges the airways ARE the picture: they are where the
                            -- traffic will be, and a contact off them is the interesting one
    SHOW_TRAIL    = true,
    FULL_BLOCK    = true,
    CELL_ASPECT   = 1.5,
    PICK_RADIUS   = 2,
}

CFG.PANEL = {
    WIDTH     = 26,
    MIN_MAP_W = 24,
}

-- ============================================================================
-- PRESENTATION.
-- ============================================================================

CFG.REDRAW      = 0.25      -- s between repaints. On a heading-up scope this is also how
                            -- often the whole picture rotates, so faster is not better --
                            -- 4 Hz reads as smooth turning, 20 Hz reads as a spinning map.
CFG.TERM_REDRAW = 1.0
CFG.TEXT_SCALE  = 0.5

CFG.MONITOR_NAME = nil

CFG.PALETTE = {
    MON = {
        grey      = 0x555555,   -- range rings, cardinal ticks
        green     = 0x28402e,   -- airways, drawn faint and underneath everything
        lightGrey = 0x8a8a8a,   -- waypoint names, secondary text
        cyan      = 0x33bbdd,   -- IFR traffic
        lime      = 0x55dd55,   -- VFR traffic
        orange    = 0xffbb00,   -- aerodromes, runways
        red       = 0xdd4444,   -- a target reporting a TCAS resolution
        blue      = 0x0e4657,   -- selected strip background, panel rule
        brown     = 0x553322,   -- the far end of a runway
    },
}

return CFG
