-- AER TOGEKISS -- MANCON: PER-COMPUTER CONFIGURATION
--
-- Every protocol name, key code, redstone face, limit and rate mancon.lua uses. Split out of
-- the program because mancon.lua is deployed to TWO computers -- #6 here and #76 on Scout --
-- and those two differ in every one of these values. Same reasoning, and the same rules, as
-- vecconf.lua beside vecengine.lua and gpwsconf.lua beside gpws.lua:
--
--   * This is the SINGLE SOURCE OF TRUTH. mancon.lua holds no fallback copy, deliberately:
--     a duplicated default would mean editing a knob here could silently do nothing. A
--     missing knob is a NAMED BOOT ERROR instead.
--   * NEVER regenerate this file by copying Scout's. That is the mistake that rolled Scout
--     over on 2026-08-22 (see 78/vecconf.lua) -- these are wiring facts and flown
--     calibration, and the comments beside them are the record of why.
--   * It must sit BESIDE mancon.lua (require resolves relative to the running program) and
--     must return the CFG table.
--
-- WHAT THIS COMPUTER REPLACED (2026-08-28). #6 used to be the vertical-speed commander alone
-- (vspdpid.lua, an analog lever -> aertogekiss_mancon). It now carries the whole hand-flying
-- stick, so it also owns aertogekiss_mancon_nav -- which the MIC (#14) used to broadcast.
-- TWO COMPUTERS CANNOT SHARE ONE PROTOCOL: whichever spoke last would win, alternating at
-- 20 Hz, and the button panel and the keyboard would fight over one setpoint. #13 and #14
-- are therefore retired (startup.lua prints a notice, as #4's does), and #15 keeps its
-- touchdown detection and EXTRA LIFT bank with only its yaw-relay loop switched off. See
-- CLAUDE.md's Aer Togekiss section for the revert.

local CFG = {}

-- PROTOCOLS ------------------------------------------------------------------
-- Out: the two the UAC (#1) listens on -- CFG.MANCON_PROTOCOL and CFG.MANNAV_PROTOCOL in
-- 1/pid/uacconf.lua. OUT_NAV was #14's; the GPWS (#67) also reads OUT_VERT for its
-- commanded-VS readout. In: the AP-NAV toggle the FMS (#8) re-announces, and #11's engine
-- switch. Note IN_APNAV is the UNMERGED `_1`, not the plain merged name #12 produces -- the
-- merged one carries no autopilot_nav field.
CFG.OUT_VERT  = "aertogekiss_mancon"
CFG.OUT_NAV   = "aertogekiss_mancon_nav"
CFG.IN_APNAV  = "aertogekiss_apswitch_1"
CFG.IN_ENGINE = "aertogekiss_engineswitch"

-- IN_SCRAMBLE -- a REMOTE scramble request from another computer on this aircraft, ORed in
-- parallel with the key and the lever inside commandLoop. false = this hull has no such
-- source, which is a real answer and canaried as one.
--
-- WHY IT COMES IN HERE AND NOT STRAIGHT OUT ON OUT_VERT. An automatic pull-up has to be
-- indistinguishable from the pilot's own detent to everything downstream -- the UAC, the
-- GPWS, the relays -- so it is folded into `scram` BEFORE the broadcast and OUT_VERT keeps
-- exactly the shape it has always had. Nothing downstream learns a new field, gets a second
-- sender, or can tell the two apart, and the pull-up inherits every property of a scramble
-- for free: it outranks ALT-AP and an autoland, and it ages out on the UAC's own staleness
-- gate. (It used to inherit a third: freezing the vertical TRIM. There is no trim to freeze
-- since 2026-09-08 -- the axis is spring-loaded -- so the vertical simply follows the keys
-- throughout, which the UAC ignores for as long as the flag is up.) ONE PROTOCOL, ONE BROADCASTER -- the rule
-- #13 and #14 are retired to keep.
-- FALSE HERE: nothing on Aer Togekiss can call a scramble remotely. #15 is the nearest
-- equivalent to Scout's #78 -- it does the touchdown detection and the EXTRA LIFT bank --
-- but it has no pull-up lever and asks for no scramble. If one is ever fitted, give it its
-- own protocol and name it here; do NOT let it broadcast on OUT_VERT.
CFG.IN_SCRAMBLE = false
CFG.IN_SCRAMBLE_STALE = 0.5   -- unused while IN_SCRAMBLE is false; kept so the pair travels together

-- PERIPHERALS ----------------------------------------------------------------
-- The monitor's face, named rather than searched for with peripheral.find("monitor"):
-- find would also answer with a monitor on some other face. false = draw on the computer's
-- own terminal, which is what this is set to until a monitor is actually bolted on -- #6
-- carried no monitor as the vertical-speed commander, because a five-line text readout fitted
-- on the terminal. Set this to the side once one is fitted; the panel needs no other change.
CFG.MONITOR_SIDE = false

-- KEY CODES ------------------------------------------------------------------
-- GLFW codes as the typewriter reports them; sample new ones with get_keycodes.lua beside
-- mancon.lua. Each is a LIST so an action can carry several keys.
--
-- The same on both airframes, so one pilot flies both with the same fingers. The horizontal
-- six are the two rows of the WER/SDF cluster, laid out the way the aircraft moves; the
-- vertical group sits off that cluster:
--       W strafe left    E fore     R strafe right          SPACE climb    A boost
--       S yaw left       D aft      F yaw right             Z     descend  V scramble
--
-- REBOUND 2026-09-08 with the vertical's change of feel. It used to be SPACE climb / A
-- descend / C centre, which followed 21/keyboardjoystick.lua's throttle -- but that layout
-- was a TRIM's layout: a trim that holds needs a centre key, and a spring-loaded axis does
-- not, because letting go IS centring it. So:
--   * descend moves off A onto Z, keeping it beside the key it opposes on the bottom row;
--   * A becomes BOOST, a modifier held with anything else (see NAV.BOOST_* / VS.BOOST_MAX_CMD);
--   * C is RETIRED and deliberately not left here as a dead knob -- mancon.lua no longer reads
--     it, and a key code sitting in a conf file that nothing consumes is exactly the silent
--     lie the no-fallback-defaults rule exists to prevent. If a centre key is ever wanted
--     again, it comes back with the code that reads it.
-- The Citation's #21 keeps space/A/C: it flies a PITCH STICK, which is a different thing that
-- still holds and still needs centring.
CFG.KEYS = {
    FORE         = { 69 },   -- E
    AFT          = { 68 },   -- D
    YAW_LEFT     = { 83 },   -- S
    YAW_RIGHT    = { 70 },   -- F
    STRAFE_LEFT  = { 87 },   -- W
    STRAFE_RIGHT = { 82 },   -- R
    VS_UP        = { 32 },   -- space
    VS_DN        = { 90 },   -- Z
    BOOST        = { 65 },   -- A
    -- SCRAMBLE IS V, AND V ALONE (2026-09-08). X was dropped because it collides with an
    -- in-game shortcut: the typewriter is not the only thing listening when the pilot is sat
    -- at it, and a key that also does something to the world is the wrong key for the one
    -- command that exists for when everything has gone wrong.
    --
    -- The history, because it is the reason this is a LIST and the reason V is the survivor:
    -- the 2026-08-28 rebind documented X everywhere -- the layout above, the panel, CLAUDE.md
    -- -- while what this file actually carried was 86 alone, which is V. So the documented key
    -- produced no packet and the only working scramble on this hull was the redstone lever on
    -- VS.SIDE; that is the whole of "the scramble does not work, regardless of AP", found
    -- 2026-08-29. Both codes were then kept so an emergency command had more than one finger
    -- on it. X is now the one that goes, which leaves the key that has actually been flown.
    --
    -- It is still a list: the scramble is the one command that deserves a second binding, and
    -- adding one here is all it takes. THE REDSTONE LEVER ON VS.SIDE IS THE REAL SECOND PATH
    -- and is untouched -- every key on this stick comes off one peripheral, so a dead
    -- typewriter must not cost the pilot this.
    SCRAMBLE     = { 86 },   -- V
}

-- VERTICAL CHANNEL -----------------------------------------------------------
CFG.VS = {
    -- The scramble lever's redstone face -- a single ON/OFF switch, not a level. This is
    -- the face the old analog vertical lever ran into (vspdpid.lua's IN_SIDE = "front"), so
    -- the wire is already there; what changes in-game is the lever at the far end of it,
    -- from an analog source to a plain toggle. An unpowered face reads false, which is "no
    -- scramble", so an unwired or cut lever fails to the safe state by itself -- which the
    -- analog build could not manage, since its unpowered 0 was a full-rate DESCENT.
    SIDE     = "front",

    -- What full stick travel asks for, m/s. THE UAC'S OWN CLAMP IS THE CEILING HERE: this
    -- airframe's CFG.CLIMB_MAX / CFG.DESCEND_MAX are both 3.75 m/s, applied before the rate
    -- loop ever sees the number.
    --
    -- CHANGED FROM vspdpid.lua's 12.0, and this is a correctness fix rather than a retune.
    -- Against a 3.75 clamp, 12.0 made the old lever's levels 10-14 all mean the same thing
    -- -- wasteful but harmless, because a lever springs back to where the pilot puts it. A
    -- TRIM has to be wound back by hand, so the same 12.0 would mean holding the climb key
    -- for 2.3 s past the point anything changes and then holding descend for 2.3 s before
    -- the aircraft responded at all. Dead travel a lever can carry, a trim cannot.
    -- STILL RIGHT NOW THE AXIS IS A SPRING (2026-09-08), for the second half of that reason:
    -- a spring does fall back to zero on its own, but travel above the UAC's clamp is a
    -- second of held key during which the aeroplane does not respond at all, and this is now
    -- also the number the BOOST is measured against -- see BOOST_MAX_CMD below.
    MAX_CMD  = 3.75,

    -- Ramp rate, m/s per second of commanded rate while a key is held. Full travel from
    -- level therefore takes MAX_CMD / RATE = 0.94 s -- which is what vspdpid.lua's own
    -- 4 m/s/s command slew took to cover the same ground, so the CLIMB feel is unchanged
    -- from what this aircraft has always flown. What changed on 2026-09-08 is the RELEASE.
    RATE     = 4.0,

    -- Rate the command falls back to zero at when no vertical key is held -- the SPRING.
    -- This used to be reached only by the centre key or by holding both trim keys, because
    -- the axis was a trim and release meant HOLD. It is now what release itself does, so it
    -- is the number that decides how promptly letting go stops the climb: ~0.31 s from full
    -- travel. Still faster than the ramp (as the MIC's fade is ~2.7x its own), because
    -- stopping is a correction and starting is an input.
    FADE     = 12.0,

    -- BOOSTED ceiling, m/s, while CFG.KEYS.BOOST is held: twice the unboosted 3.75, and
    -- comfortably inside this airframe's CFG.CLIMB_AT_FULL (10.0 -- itself still an estimate;
    -- see CLAUDE.TODO.md #1), so it is a rate the lift bank can actually buy rather than one
    -- it saturates against.
    --
    -- THIS ONE IS NOT FREE, unlike the horizontal boosts below: the UAC clamps the wanted
    -- rate before its rate loop ever sees it, and that clamp is CFG.CLIMB_MAX/DESCEND_MAX --
    -- the HANDLING envelope, which the autoland and ALT-AP fly inside too. So a matching
    -- CFG.MANCON_CLIMB_MAX / CFG.MANCON_DESCEND_MAX was added to 1/pid/uacconf.lua beside them,
    -- applied to the manual branch ALONE. Raising CLIMB_MAX itself would have bought the
    -- boost by loosening every autopilot descent as well. THE TWO FILES MUST AGREE: set this
    -- above the UAC's manual clamp and the extra travel is simply thrown away up there.
    BOOST_MAX_CMD = 7.5,
}

-- HORIZONTAL / YAW CHANNEL ---------------------------------------------------
-- Full-key target speeds and the accumulator dynamics. Body frame: fwd + = forward,
-- strafe + = right, yaw + = nose-right (a turn RATE). The UAC clamps and sign-calibrates
-- further (CFG.THRUST_SIGN / CFG.STRAFE_SIGN / CFG.MANNAV_YAW_SIGN and CFG.MAX_RATE_YAW),
-- so these are the manual ceilings only.
--
-- CARRIED OVER FROM 14/mic.lua VERBATIM. These are this airframe's flown numbers and the
-- port does not second-guess them -- the keyboard replaces the button relays, not the feel.
--
-- One observation, recorded rather than acted on: YAW_MAX 0.5 sits above this airframe's
-- CFG.MAX_RATE_YAW of 0.30, so the UAC clamps it and the top ~40% of the yaw ramp is dead
-- travel. #14's own comment already knew ("supposed to match the UAC's MAX_RATE_YAW, but 0.5
-- seems working well?"). It costs nothing but the last part of the ramp on a spring-loaded
-- axis -- which since 2026-09-08 is every axis on this computer, the vertical included.
-- Left alone; change it only from a flight.
CFG.NAV = {
    FWD_MAX    = 80.0,   -- m/s,   full fore/aft target
    STRAFE_MAX = 25.0,   -- m/s,   full strafe target
    YAW_MAX    = 0.5,    -- rad/s, full yaw-rate target (UAC clamps to 0.30 -- see above)
    -- BOOSTED ceilings while CFG.KEYS.BOOST is held. Free, unlike the vertical one: nothing
    -- downstream clamps fore/aft or strafe to a fixed number, so raising the target here is
    -- the whole change. Both are 2x the unboosted figure -- hand-flying was capped at a
    -- fraction of what this aircraft cruises at under the autopilot, and a ferry flight to a
    -- flight-test area should not have to be flown at a quarter speed.
    -- The RAMP is deliberately NOT boosted with them, so the first part of the travel feels
    -- exactly as it always did and the boost is extra RUNWAY rather than a step. It also
    -- means releasing the boost while still holding the key walks back down at RAMP (the
    -- slow rate) rather than FADE -- a gentle deceleration, which is the right way round.
    -- YAW IS NOT BOOSTED and deliberately has no entry: YAW_MAX is already above the UAC's
    -- own CFG.MAX_RATE_YAW, so a boosted yaw target would be clamped away without ever
    -- reaching a thruster. A knob that provably cannot do anything is worse than no knob.
    BOOST_FWD_MAX    = 160.0,
    BOOST_STRAFE_MAX = 50.0,
    RAMP       = 15.0,   -- m/s per s, fore/aft/strafe target growth while held
    FADE       = 40.0,   -- m/s per s, fade to 0 on release
    YAW_RAMP   = 0.8,    -- rad/s per s, yaw target growth while held
    YAW_FADE   = 3.0,    -- rad/s per s, yaw fade to 0 on release
}

-- LOCKOUT --------------------------------------------------------------------
-- When the pilot's HORIZONTAL keys are read as released even though they are down. This is
-- the `(not apnav) and mst` gate #13 and #15 used to apply before they broadcast at all,
-- moved into this computer along with everything else they did. Not a safety interlock --
-- the UAC has its own AP-NAV gate and the engines are cut when the master is off -- but it
-- stops a key held while the autopilot flies from winding the accumulator up behind the
-- pilot's back.
--
-- The VERTICAL is gated by neither, exactly as vspdpid.lua was gated by nothing: AP-NAV does
-- not own the vertical channel (ALT-AP does, and that is the UAC's own gate).
--
-- UNLIKE SCOUT, the master gate really bites here: #11 exists and broadcasts, so master-off
-- reads the keys as released. That is the behaviour #13/#15 had and it is deliberate.
CFG.LOCK_ON_APNAV  = true
CFG.LOCK_ON_MASTER = true

-- RATE -----------------------------------------------------------------------
CFG.TICK = 0.05   -- s between read/integrate/broadcast steps (matches the UAC control tick)

return CFG
