-- TMU ATC -- CONFIGURATION
--
-- Everything the radar station (#44 atc.lua) can be REPOINTED or RESCALED with: where the
-- station stands, which protocols it listens on, the database filenames, the scope's ranges
-- and colours, and how long a silent target is kept on the screen.
--
-- Same split, and for the same reasons, as the FMS's apconf.lua and the UAC's pid/uacconf.lua:
-- a program update never overwrites this station's setup, and the knobs are editable without
-- scrolling the program. THERE ARE DELIBERATELY NO FALLBACK COPIES OF THESE VALUES IN atc.lua
-- -- a missing knob is a named BOOT error (see the canary list in atc.lua's loader), not a nil
-- arithmetic crash halfway through a shift.
--
-- Loaded by `require("atcconf")`, which resolves because startup.lua runs shell.run("atc.lua")
-- and so the program directory is this one. Must end with `return CFG`.
--
-- Supersedes the old five-line stub (kept as atcconf.prestation.bak); the four values it
-- carried -- static, 705, 647, north -- are the four in CFG.STATION below, unchanged.

local CFG = {}

-- ============================================================================
-- THE STATION -- where the scope is centred, and which way is up.
-- ============================================================================

CFG.STATION = {
    NAME  = "TMU",          -- shown on the scope at the station symbol
    LABEL = "TMU TOWER",    -- shown in the header bar
    -- "static":  the station stands at X/Z/Y below -- a tower, which is what #44 is.
    -- "dynamic": airborne. Position, heading and velocity come off sublevel every frame and
    --            X/Z/Y are not read at all; the scope normally turns with the aircraft. See
    --            #64, Aer Togekiss's AWACS, for the full set of knobs that mode needs.
    -- Which knobs are REQUIRED depends on this: atc.lua checks the ones the chosen mode uses
    -- and names the missing one at boot, so a half-converted config cannot boot half-working.
    MODE  = "static",
    X = 705,                -- Minecraft world coordinates. East is X+, SOUTH is Z+.
    Z = 647,
    Y = 64,                 -- station elevation. The scope reports every target's ABSOLUTE
                            -- altitude, so this is only the third component of the fix -- it
                            -- matters when MODE is "mobile" and nothing reads it when static.
    -- Which compass direction is drawn at the TOP of the scope. A name (n/e/s/w) or a
    -- heading in degrees. Real approach scopes are oriented to the field, not to north;
    -- TMU's is oriented to the tower's own facing, which happens to be north.
    -- ("heading" and "track" follow the aircraft and are rejected here -- a tower has neither.)
    ORIENTATION = "north",
}

-- ============================================================================
-- LINKS -- who this computer listens to. Both names are FLEET-WIDE and must not be
-- renamed per aircraft: every responder.lua transmits on the first, and the nav data
-- server (#24) broadcasts on the second.
-- ============================================================================

CFG.LINKS = {
    ATC     = "atc_broadcast",  -- responders -> here. Must match responder.lua's ATC_CAST.
    NAVDATA = "aer_navdata",    -- nav data server -> here. Must match nav_update_server.lua.
}

-- ============================================================================
-- NAVIGATION DATABASE -- the same three files, in the same format, as an aircraft
-- carries. The station only READS them; it never edits them. They are kept current by
-- subscribing to the nav data server exactly as an FMS does, which is why the station
-- shows up on the server's client roster.
-- ============================================================================

CFG.POINTS_FILE  = "points.db"
CFG.COURSE_FILE  = "course.db"
CFG.LANDING_FILE = "landing.db"

CFG.NAVDATA = {
    ENABLE      = true,     -- false: fly on whatever is on this disk, ignore the server
    PULL_EVERY  = 5,        -- s between "send me the set" calls, until the server answers
    STALE_AFTER = 90,       -- s of silence before the header calls the server stale
    TMP         = ".new",   -- suffix of the staging file each .db is written through
}

-- ============================================================================
-- TARGETS -- how a contact ages off the scope. The responders transmit at 20 Hz, so a
-- gap of even a second means something went quiet.
-- ============================================================================

CFG.TRACK = {
    COAST_AFTER = 3,        -- s without a report before the target is dimmed and flagged C
    DROP_AFTER  = 20,       -- s without a report before it leaves the scope entirely
    LEADER_SEC  = 30,       -- the velocity leader line is where the target will be in this
                            -- many seconds -- shorter than the chart-standard minute because
                            -- these aircraft cruise at 320 m/s and this scope works in km
    LEADER_MAX_FRAC = 0.35, -- ...and never longer than this fraction of the current range,
                            -- so a fast target at close range points rather than smears
    TRAIL_LEN   = 8,        -- how many history dots a target drags behind it
    TRAIL_EVERY = 1.0,      -- s between history dots
    FIX_STALE_AFTER = 2,    -- s without a sublevel fix before the header says the picture is
                            -- old. MODE "dynamic" only -- a tower cannot lose its position.
}

-- ============================================================================
-- THE SCOPE.
-- ============================================================================

CFG.MAP = {
    -- Range steps, in metres from the centre to the OUTER RING. [-] and [+] walk this
    -- list; AUTO picks the smallest step that still frames every target on frequency.
    -- The corners of a rectangular monitor reach further than the ring does -- that
    -- extra ground is drawn, it is simply outside the scale.
    RANGES        = {250, 500, 1000, 2500, 5000, 10000, 25000, 50000},
    DEFAULT_RANGE = 2500,
    RINGS         = 3,      -- range rings drawn, evenly spaced out to RANGES' current step
    AUTO_RANGE    = true,   -- start with auto-ranging on
    AUTO_MARGIN   = 1.15,   -- auto-range leaves this much slack around the furthest target
    SHOW_WAYPOINT = false,  -- five-letter VORs. Off by default: en route fixes swamp a
                            -- terminal-area scope, and they are one button away.
    SHOW_ROUTE    = true,   -- the airways from course.db, as faint lines
    SHOW_TRAIL    = true,   -- target history dots
    FULL_BLOCK    = true,   -- full two-line data block per target, vs the callsign alone
    CELL_ASPECT   = 1.5,    -- a monitor character is 6x9 px, so a metre is 1.5x as many
                            -- cells across as it is down. Keeps a circle circular.
    PICK_RADIUS   = 2,      -- cells: how close a touch must land to select a target
}

CFG.PANEL = {
    WIDTH     = 26,         -- columns reserved for the traffic strips on the right
    MIN_MAP_W = 24,         -- if the map would end up narrower than this, drop the panel
}

-- ============================================================================
-- PRESENTATION.
-- ============================================================================

CFG.REDRAW      = 0.25      -- s between scope repaints. Display only -- the receive loop
                            -- runs independently and never misses a report because of this.
CFG.TERM_REDRAW = 1.0       -- s between repaints of the computer's own status screen
CFG.TEXT_SCALE  = 0.5       -- the monitor's text scale (smaller = more cells)

CFG.MONITOR_NAME = nil      -- pin a specific monitor by peripheral name, or nil for the
                            -- first one found. Set this if the station ever grows a second.

CFG.PALETTE = {
    -- Redefined on the monitor at boot. The scope keeps the fleet's instrument look
    -- (see apconf.lua's HSI palette) rather than the traditional phosphor green, so a
    -- controller and a pilot are reading the same colours for the same things.
    MON = {
        grey      = 0x555555,   -- range rings, cardinal ticks, ring labels
        green     = 0x28402e,   -- airways, drawn faint and underneath everything
        lightGrey = 0x8a8a8a,   -- waypoint names, secondary text
        cyan      = 0x33bbdd,   -- IFR traffic
        lime      = 0x55dd55,   -- VFR traffic
        orange    = 0xffbb00,   -- aerodromes, runways, the station itself
        red       = 0xdd4444,   -- a target reporting a TCAS resolution
        blue      = 0x0e4657,   -- selected strip background, panel rule
        brown     = 0x553322,   -- the far end of a runway
    },
}

return CFG
