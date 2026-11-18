-- ATC CONTROLLER -- CONFIGURATION (#87)
--
-- Everything the automated controller (atcctl.lua) can be repointed or retuned with: which
-- protocols it speaks on, the database filenames, how big a facility is for the purpose of
-- "is somebody on it", how long a silent aircraft is remembered, and what the facility
-- remarks in sites.db mean.
--
-- Same split, and for the same reasons, as every other *conf.lua in this tree: a program
-- update never overwrites this station's setup, and THERE ARE DELIBERATELY NO FALLBACK COPIES
-- OF THESE VALUES IN atcctl.lua -- a missing knob is a named BOOT error (see the canary list in
-- atcctl.lua's loader), not a nil-arithmetic crash in the middle of sequencing two arrivals.
--
-- Loaded by `require("atcctlconf")`, which resolves because startup.lua runs
-- shell.run("atcctl.lua") from this directory. Must end with `return CFG`.

local CFG = {}

-- ============================================================================
-- LINKS -- all three are FLEET-WIDE and must not be renamed per station.
-- ============================================================================

CFG.LINKS = {
    ATC     = "atc_broadcast",  -- responders -> here. WHO IS WHERE: the only thing occupancy is
                                -- ever decided from. Must match responder.lua's ATC_CAST.
    NAVDATA = "aer_navdata",    -- nav data server (#24) -> here: points/landing/sites.
    CTL     = "aer_atc",        -- FMS <-> here: requests in, clearances out. Must match every
                                -- apconf.lua's CFG.ATC.PROTOCOL. THIS STATION IS ITS ONLY
                                -- ANSWERER -- a second controller on the same protocol would
                                -- hand an aircraft two different clearances a second apart.
}

-- ============================================================================
-- NAVIGATION DATABASE -- read only. Kept current by subscribing to #24 exactly as the scope
-- does; course.db is not needed here and is not stored.
-- ============================================================================

CFG.POINTS_FILE  = "points.db"
CFG.LANDING_FILE = "landing.db"
CFG.SITES_FILE   = "sites.db"
CFG.PARK_FILE    = "parked.db"    -- the silent-aircraft memory, persisted across a reboot

CFG.NAVDATA = {
    ENABLE      = true,     -- false: control on whatever is on this disk, ignore the server
    PULL_EVERY  = 5,        -- s between "send me the set" calls, until the server answers
    STALE_AFTER = 90,       -- s of silence before the header calls the server stale
    TMP         = ".new",   -- suffix of the staging file each .db is written through
}

-- ============================================================================
-- REQUESTS -- the FMS asks once a second (apconf.lua's CFG.ATC.INTERVAL) for as long as it is
-- inbound to a facility. A request not renewed within STALE seconds is a plan that ended: the
-- aircraft landed, the pilot cancelled, AP-NAV went off, or it released ATC. Its reservation
-- goes with it -- and if it landed, its own responder is what occupies the facility from then.
-- Must comfortably exceed the FMS's INTERVAL, or one dropped packet frees a reservation.
-- ============================================================================

CFG.REQ = {
    STALE = 5,
}

-- ============================================================================
-- TRACKS -- responder reports. The responders transmit at 20 Hz, so STALE is generous.
-- ============================================================================

CFG.TRACK = {
    STALE      = 3,     -- s: a report older than this does not put anyone on a facility.
                        -- From here an aircraft that was PARKED is held by the memory
                        -- below instead; one that was merely passing is simply gone.
    DROP_AFTER = 60,    -- s: forgotten entirely (the board stops listing it)
}

-- ============================================================================
-- OCCUPANCY -- what "somebody is on that facility" means, geometrically.
--
-- Every facility is a CAPSULE: a line segment with a radius. A pad is a segment of zero
-- length (a circle); a runway is its strip, threshold to far end, stretched RWY_END_M past
-- both ends, with RWY_HALF_W either side. One shape for both means one inside-test and one
-- overlap-test, and two facilities whose capsules overlap CONFLICT -- which is what keeps a
-- clearance to TMV from being handed out while another aircraft is cleared onto TMA, the
-- same strip flown the other way. Nothing in sites.db has to say so.
--
-- Sized for the fleet's LARGEST airframe, because the question is "would a landing here hit
-- whatever is sitting there", and the position a responder reports is its centre: Aer
-- Togekiss is described in CLAUDE.md as "a massive, superwide VTOL", and its centre can sit a
-- long way from the pad marker it is parked across. Err wide -- a facility held busy a little
-- too eagerly costs a hold; one released too eagerly costs a collision.
-- ============================================================================

CFG.OCC = {
    PAD_R      = 30,    -- m: radius of a helipad's footprint
    RWY_HALF_W = 20,    -- m: half-width of a runway's footprint
    RWY_END_M  = 30,    -- m: how far past each end of the strip the footprint runs
    AGL_MAX    = 20,    -- m above the facility's surveyed altitude a report still counts as
                        -- "on it" -- low enough that an overflight does not close the field,
                        -- high enough that an aircraft lifting off or settling onto it does
    BELOW_MAX  = 10,    -- m BELOW it. A pad can be a platform in the air (WCL sits at 226 over
                        -- open ground) and whoever flies underneath is not on it.
    FREE_AFTER = 5,     -- s a facility stays busy after the last occupant is seen leaving.
                        -- A departure needs a moment to clear the approach path, and it stops
                        -- a report that flickers in and out of the footprint flapping the
                        -- clearance between CLEARED and HOLD.

    -- THE SILENT-AIRCRAFT MEMORY. A parked aircraft goes silent the moment nobody is near
    -- enough to keep its chunk loaded -- and is still physically there. Without this, a
    -- remote field reads FREE until the arriving aircraft is a couple of hundred metres out
    -- and the parked one's responder boots, i.e. until the arrival is already descending on
    -- top of it. So an aircraft seen sitting still on a facility is REMEMBERED there, on disk,
    -- until it is heard somewhere else. Its own responder being heard anywhere off that
    -- facility is what clears the record; a record for an aircraft that will never transmit
    -- again (scrapped, renamed) is cleared by touching that facility on the board.
    PARK_GS    = 1.0,   -- m/s: slower than this counts as sitting still
    PARK_AFTER = 3,     -- s of sitting still on a facility before it is remembered there
}

-- ============================================================================
-- REMARKS -- the vocabulary sites.db may put in brackets after a facility name, and which
-- request flag each one turns away. The FMS sends the flags (apconf.lua's CFG.ATC); this maps
-- a remark to the flag that it excludes:
--
--   narrow = "wide"   -- a facility marked (narrow) is refused to any aircraft whose request
--                        says wide = true
--
-- A remark NOT listed here makes its facility unusable to everyone and is shown as a fault on
-- the board: an unknown restriction the controller silently ignored would be a clearance onto
-- a facility the person who wrote sites.db said was not safe for somebody.
-- ============================================================================

CFG.REMARKS = {
    narrow = "wide",
}

-- ============================================================================
-- PRESENTATION
-- ============================================================================

CFG.REDRAW       = 0.5      -- s between repaints of the board. Display only -- the request and
                            -- traffic loops run independently of it.
CFG.TEXT_SCALE   = 0.5      -- the monitor's text scale, if one is attached
CFG.MONITOR_NAME = nil      -- pin a specific monitor by peripheral name, or nil for the first
                            -- one found. Without any, the board is drawn on the terminal.

return CFG
