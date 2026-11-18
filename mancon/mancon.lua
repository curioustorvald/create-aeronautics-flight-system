-- SCOUT -- MANCON (#76)
--
-- ALL of Scout's hand-flying, on one computer. Every other airframe spreads this across
-- four: a vertical-speed commander (#6), two key relays (#13 translation / #15 rotation)
-- and the intention controller that integrates them (#14). Scout does not have the
-- computers to spare, so this file is all four at once -- but the two CHANNELS stay
-- completely independent of each other, exactly as they are on Aer Togekiss:
--
--   VERTICAL    THE KEYBOARD: two keys ramp a wanted climb rate up and down and release
--               falls back to a hover, and a third calls a scramble -- as does a single
--               digital lever on VS.SIDE, wired in parallel with that key. Same packet as
--               6/vspdpid.lua's analog lever, driven differently.     -> scout_mancon
--   HORIZONTAL  the same TYPEWRITER: six keys for fore/aft, strafe and yaw, integrated
--               into wanted body-frame velocities. The same accumulator dynamics and the
--               same packet 14/mic.lua sends, with the keyboard standing in for the two
--               button relays.                                       -> scout_mancon_nav
--
-- ALL TEN KEYS COME OFF ONE PERIPHERAL, so a dead typewriter now costs the pilot the
-- vertical as well as the horizontal. That is why the scramble ALSO has a redstone lever:
-- it is the one command that exists for the moment everything else has gone wrong, so it
-- keeps a path to the UAC that does not run through the keyboard.
--
-- The two are deliberately NOT merged into one packet: the UAC runs a dedicated
-- rednet.receive loop per protocol (two loops cannot share one protocol filter), and it
-- gates the two channels on different switches -- ALT-AP overshadows the vertical, AP-NAV
-- overshadows the horizontal. See CFG.MANCON_PROTOCOL / CFG.MANNAV_PROTOCOL in
-- 72/pid/uacconf.lua.
--
-- THIS COMPUTER DRIVES NO REDSTONE OUT and runs no control loop of its own. It only NAMES
-- a wanted rate and a wanted body velocity; the UAC (#72) owns every loop that buys them,
-- and on this airframe spends them through the three vectoring nozzles (#77/#78/#80) and
-- the fore/aft bank (#79). A released key commands exactly 0, which is an active
-- brake-to-a-hover and heading-hold, not a coast -- that is the MIC's "cancel unwanted
-- yawing when yawing is not commanded".
--
-- WHY THE PACKETS GO OUT EVERY TICK, even when nothing is being commanded: the UAC gates
-- both channels on FRESHNESS (CFG.MANCON_STALE_AFTER / CFG.MANNAV_STALE_AFTER, 0.5 s
-- each). Silence is read as "no stick attached" and the craft is handed back to the
-- autopilot's hold, so a crashed or unplugged #76 cannot leave a stale velocity latched
-- in. It also means the horizontal packet is broadcast as ZEROS while the pilot is locked
-- out (below) rather than going silent as the Togekiss relays do -- the UAC then starts
-- nulling drift on the first tick after AP-NAV drops instead of waiting for a first packet.
--
-- DEPLOYED TO TWO COMPUTERS -- #76 (Scout) and #6 (Aer Togekiss) -- and this file is
-- BYTE-IDENTICAL on both. Everything that differs between them (protocol names, key codes,
-- redstone faces, limits, rates) lives in manconconf.lua beside it. Fix a bug here and copy
-- this file to the other computer; NEVER copy manconconf.lua between them.
--
-- Main chunk: 35 locals, and every function here still compiles with 160 injected on top of
-- that, so there is plenty of room under Cobalt's 200 -- unlike the two brains, a new
-- top-level local costs nothing here. That is why the CFG values below are aliased into
-- locals rather than read through CFG.* everywhere: it keeps the body of the program
-- reading exactly as it did before the split, at a cost this file can easily afford.
--
-- KEYS (GLFW codes, sampled with get_keycodes.lua beside this file, and BOUND IN
-- manconconf.lua -- the codes below are what both hulls carry today, not a hardcoding).
-- The horizontal six are the two rows of the WER/SDF cluster, laid out the way the aircraft
-- moves:
--       W (87) strafe left    E (69) fore      R (82) strafe right
--       S (83) yaw left       D (68) aft       F (70) yaw right
-- The vertical four sit off that cluster:
--       SPACE climb      Z descend      A boost                 V scramble
-- Opposing keys held together cancel, as they do on the MIC.
--
-- ONE STICK FEEL, EVERYWHERE (2026-09-08). Every axis on this computer is SPRING-LOADED:
-- hold a key and the command ramps toward that axis's ceiling, let go and it falls back to
-- zero -- which for the horizontal six means brake to a hover and hold heading, and for the
-- vertical now means stop climbing and hover. All seven run through the SAME integrate()
-- below, differing only in their limits.
--
-- WHAT THIS REPLACED, AND WHY THE CENTRE KEY WENT WITH IT. The vertical two used to be
-- TRIM-STYLE: they wound a commanded rate up and down and RELEASE HELD IT, because that is
-- what the analog lever they replaced did -- park it and the craft goes on climbing. A trim
-- that holds has to be given a way back to level that is not "wind it back by hand", which
-- is what C was for; a spring-loaded axis centres itself the moment the pilot lets go, so
-- there is nothing left for C to do and it is retired rather than kept as a key that
-- duplicates letting go. The two feels also had to be learnt separately, which is the real
-- cost the change removes: one rule now covers the whole keyboard.
--
-- The thing that made C worth having survives, because it was never really about C: the
-- vertical comes back to zero on VS.FADE (~0.3 s from full travel) rather than SNAPPING
-- there. A snap is a step input, and shaping a yanked vertical command into a ramp is
-- exactly what vspdpid.lua's VS_SLEW existed for ("so the cargo stays settled").
--
-- A: BOOST. Held with anything else, it raises the CEILINGS -- fore/aft, strafe and the
-- vertical -- to NAV.BOOST_FWD_MAX / NAV.BOOST_STRAFE_MAX / VS.BOOST_MAX_CMD. It is not a
-- separate command and it does nothing on its own; it is a modifier on how far the travel
-- the pilot is already asking for is allowed to go. The RAMP RATES are deliberately NOT
-- boosted with the ceilings, so the boost is extra RUNWAY rather than a step: pressing it at
-- full deflection accelerates on from there instead of jumping. Yaw is not boosted, because
-- the UAC's own CFG.MAX_RATE_YAW already clamps below the unboosted YAW_MAX.
--
-- THE VERTICAL BOOST IS THE ONE THAT NEEDED THE UAC'S CONSENT. That end clamps the wanted
-- rate to CFG.CLIMB_MAX/DESCEND_MAX -- the handling envelope the autoland and ALT-AP fly
-- inside as well -- so it now clamps the MANUAL branch alone to CFG.MANCON_CLIMB_MAX /
-- CFG.MANCON_DESCEND_MAX instead. The autopilots' envelope is untouched.

-- CONFIGURATION -------------------------------------------------------------
-- Loaded from manconconf.lua beside this file. No fallback defaults live here: a missing
-- knob is a named boot error, so an incomplete conf file cannot fly on guessed numbers.
local CFG
do
    local ok, res = pcall(require, "manconconf")
    if not ok then
        error("Cannot load manconconf.lua (this computer's protocols, keys and wiring): "
            .. tostring(res) .. "\nIt must sit beside mancon.lua and return the CFG table.", 0)
    end
    if type(res) ~= "table" then
        error("manconconf.lua did not return the CFG table (got " .. type(res) .. ").", 0)
    end
    for _, k in ipairs({ "OUT_VERT", "OUT_NAV", "IN_APNAV", "IN_ENGINE", "KEYS", "VS", "NAV",
                         "LOCK_ON_APNAV", "LOCK_ON_MASTER", "TICK" }) do
        if res[k] == nil then
            error("manconconf.lua is missing CFG." .. k .. " -- the file looks incomplete.", 0)
        end
    end
    -- MONITOR_SIDE is checked separately: `false` is a legitimate value (draw on the
    -- terminal), and `res[k] == nil` above cannot tell that from an absent knob.
    if res.MONITOR_SIDE == nil then
        error("manconconf.lua is missing CFG.MONITOR_SIDE (a side name, or false for the "
            .. "computer's own terminal).", 0)
    end
    -- IN_SCRAMBLE the same way, and for the same reason: `false` means "this hull has no
    -- remote scramble source", which is a real answer and not a missing knob.
    if res.IN_SCRAMBLE == nil then
        error("manconconf.lua is missing CFG.IN_SCRAMBLE (a protocol name, or false if "
            .. "nothing on this aircraft can call a scramble remotely).", 0)
    end
    if res.IN_SCRAMBLE and res.IN_SCRAMBLE_STALE == nil then
        error("manconconf.lua sets CFG.IN_SCRAMBLE but is missing CFG.IN_SCRAMBLE_STALE -- "
            .. "a remote scramble MUST be able to age out.", 0)
    end
    for _, t in ipairs({ "KEYS", "VS", "NAV" }) do
        if type(res[t]) ~= "table" then
            error("manconconf.lua's CFG." .. t .. " is not a table.", 0)
        end
    end
    -- CENTRE is gone from this list as of 2026-09-08 and BOOST has taken its place; a conf
    -- that still carries CENTRE boots fine and that key simply does nothing, which is the
    -- right way round -- the old binding was C and the new one is A, so a half-updated conf
    -- fails on the MISSING knob it needs rather than on the spare one it does not.
    for _, k in ipairs({ "FORE", "AFT", "YAW_LEFT", "YAW_RIGHT", "STRAFE_LEFT",
                         "STRAFE_RIGHT", "VS_UP", "VS_DN", "BOOST", "SCRAMBLE" }) do
        if type(res.KEYS[k]) ~= "table" or #res.KEYS[k] == 0 then
            error("manconconf.lua's CFG.KEYS." .. k .. " must be a non-empty list of key "
                .. "codes (get_keycodes.lua prints them).", 0)
        end
    end
    -- BOOST_MAX_CMD / BOOST_FWD_MAX / BOOST_STRAFE_MAX added 2026-09-08 and read on EVERY
    -- tick (the ceiling is picked per tick from the boost key), so a conf predating them must
    -- fail HERE rather than as a nil-arithmetic crash the first time the pilot touches a key.
    for _, k in ipairs({ "SIDE", "MAX_CMD", "RATE", "FADE", "BOOST_MAX_CMD" }) do
        if res.VS[k] == nil then error("manconconf.lua's CFG.VS is missing " .. k .. ".", 0) end
    end
    for _, k in ipairs({ "FWD_MAX", "STRAFE_MAX", "YAW_MAX", "RAMP", "FADE",
                         "YAW_RAMP", "YAW_FADE", "BOOST_FWD_MAX", "BOOST_STRAFE_MAX" }) do
        if res.NAV[k] == nil then error("manconconf.lua's CFG.NAV is missing " .. k .. ".", 0) end
    end
    CFG = res
end

-- Aliases, so the body of the program reads as it did before the split. Read-only: they
-- point at the conf's own tables, they are not copies of them.
local OUT_VERT, OUT_NAV = CFG.OUT_VERT, CFG.OUT_NAV
local IN_APNAV, IN_ENGINE = CFG.IN_APNAV, CFG.IN_ENGINE
local IN_SCRAMBLE, IN_SCRAMBLE_STALE = CFG.IN_SCRAMBLE, CFG.IN_SCRAMBLE_STALE
local TICK = CFG.TICK
local VS, NAV = CFG.VS, CFG.NAV
local K_FORE, K_AFT               = CFG.KEYS.FORE, CFG.KEYS.AFT
local K_YAW_LEFT, K_YAW_RIGHT     = CFG.KEYS.YAW_LEFT, CFG.KEYS.YAW_RIGHT
local K_STRAFE_LEFT, K_STRAFE_RIGHT = CFG.KEYS.STRAFE_LEFT, CFG.KEYS.STRAFE_RIGHT
local K_VS_UP, K_VS_DN = CFG.KEYS.VS_UP, CFG.KEYS.VS_DN
local K_BOOST, K_SCRAMBLE = CFG.KEYS.BOOST, CFG.KEYS.SCRAMBLE
local LOCK_ON_APNAV, LOCK_ON_MASTER = CFG.LOCK_ON_APNAV, CFG.LOCK_ON_MASTER

-- SETUP ----------------------------------------------------------------------
local modem = peripheral.find("modem")
if modem then
    rednet.open(peripheral.getName(modem))
else
    printError("No modem attached; the UAC will hear neither " .. OUT_VERT .. " nor " .. OUT_NAV)
end

-- Find the keyboard by CAPABILITY rather than by peripheral type or a hardcoded side: the
-- only thing actually required of it is getPressedKeyCodes(), and get_keycodes.lua's
-- peripheral.wrap('top') would break the moment the typewriter is moved.
--
-- A MISSING TYPEWRITER IS NOT FATAL HERE, unlike on #21. This computer owns two independent
-- channels and the vertical one is the more important of the two: losing the keyboard must
-- cost the pilot fore/aft/strafe/yaw and nothing else, so it is reported on the panel and
-- the horizontal accumulators simply read every key as up.
local function findKeyboard()
    for _, name in ipairs(peripheral.getNames()) do
        local p = peripheral.wrap(name)
        if type(p) == "table" and type(p.getPressedKeyCodes) == "function" then
            return p, name
        end
    end
end
local keyboard = findKeyboard()

-- Draw on the monitor named by CFG.MONITOR_SIDE (the panel wants the room), else on the
-- computer's own terminal. Redirect rather than mirror so there is exactly one draw path.
--
-- The face is named rather than searched for, because peripheral.find("monitor") would also
-- answer with a monitor on some other face -- and these computers' other faces are spoken
-- for. The capability check is the cost of naming it: if that face ever holds something
-- that is not a monitor, term.redirect would raise at BOOT and take BOTH channels with it,
-- which is a far worse failure than drawing on the terminal instead.
local monitor = CFG.MONITOR_SIDE and peripheral.wrap(CFG.MONITOR_SIDE)
if monitor and (type(monitor.setTextScale) ~= "function"
        or type(monitor.getSize) ~= "function" or type(monitor.blit) ~= "function") then
    monitor = nil
end
if monitor then
    monitor.setTextScale(0.5)
    term.clear(); term.setCursorPos(1, 1)
    print("Scout MANCON: panel on the monitor.")
    term.redirect(monitor)
end

local COLOUR = term.isColour and term.isColour() or false

-- Instrument palette. Every draw call names a ROLE, never a raw colour, so the panel
-- restyles from here -- and so a basic (non-advanced) computer, which only has
-- white/lightGrey/grey/black, gets a legible monochrome version instead of a crash.
local PAL
if COLOUR then
    PAL = {
        panel = colours.black,  face  = colours.blue,   grid  = colours.grey,
        sight = colours.cyan,   stick = colours.orange, yawc  = colours.orange,
        climb = colours.green,  desc  = colours.yellow, scram = colours.red,
        text  = colours.white,  ok    = colours.green,  warn  = colours.red,
        idle  = colours.grey,
    }
    -- Repaint for an instrument look, as the FMS does for its HSI and altcom for its
    -- horizon. pcall: setPaletteColour is advanced-only and a monitor that refuses must
    -- not be fatal.
    pcall(function()
        term.setPaletteColour(colours.black,  0x070910)   -- surround, not quite black
        term.setPaletteColour(colours.blue,   0x16233a)   -- instrument face
        term.setPaletteColour(colours.grey,   0x39485f)   -- graticule
        term.setPaletteColour(colours.cyan,   0x2c6f80)   -- sight lines
        term.setPaletteColour(colours.orange, 0xffb000)   -- the stick itself (amber)
        term.setPaletteColour(colours.green,  0x3fb950)
        term.setPaletteColour(colours.yellow, 0xf0c000)
        term.setPaletteColour(colours.red,    0xe05038)
    end)
else
    PAL = {
        panel = colours.black,  face  = colours.black,      grid  = colours.grey,
        sight = colours.grey,   stick = colours.white,      yawc  = colours.white,
        climb = colours.lightGrey, desc = colours.grey,     scram = colours.white,
        text  = colours.white,  ok    = colours.lightGrey,  warn  = colours.white,
        idle  = colours.grey,
    }
end

term.setBackgroundColour(PAL.panel)
term.setTextColour(PAL.text)
term.clear()

-- HELPERS --------------------------------------------------------------------
local function clamp(v, lo, hi) return math.max(lo, math.min(hi, v)) end
local function round(x) return math.floor(x + 0.5) end

-- Collapse whatever shape getPressedKeyCodes() returns into a set of codes. The typewriter
-- hands back a LIST of the codes currently down ({ [1]=69, [2]=87 } -- which is why
-- get_keycodes.lua notes "simultaneous key presses will write that much characters"), but
-- accept a set ({ [69]=true }) too, so a mod update to either shape cannot silently give a
-- permanently-centred stick. The second return says whether the peripheral answered at all.
local function pressedSet()
    if not keyboard then return {}, false end
    local ok, t = pcall(keyboard.getPressedKeyCodes)
    local set = {}
    if not ok or type(t) ~= "table" then return set, false end
    for k, v in pairs(t) do
        if type(v) == "number" then set[v] = true
        elseif v and type(k) == "number" then set[k] = true end
    end
    return set, true
end

local function held(set, codes)
    for i = 1, #codes do if set[codes[i]] then return true end end
    return false
end

-- Move an accumulator toward its commanded goal. Holding a single direction drives the goal
-- to +/- maxVal at the (slow) ramp rate; anything else -- released, or both opposing keys --
-- drives it to 0 at the (fast) fade rate. Lifted from 14/mic.lua unchanged.
local function integrate(cur, pos, neg, maxVal, rampRate, fadeRate, dt)
    local goal, rate
    if pos and not neg then goal, rate = maxVal, rampRate
    elseif neg and not pos then goal, rate = -maxVal, rampRate
    else goal, rate = 0, fadeRate end
    local step = rate * dt
    local d = clamp(goal - cur, -step, step)
    return cur + d
end

-- INPUT STATE (written by the receive loops, read by the command loop) --------
local apnav, mst, mstHeard = false, false, false
-- The REMOTE scramble: another computer on this aircraft asking for one. On Scout that is
-- #78's automatic pull-up. Held between packets and aged out in commandLoop, exactly as the
-- UAC ages this computer's own packets -- a sender that dies mid-pull-up must not leave the
-- lift engine latched wide open with nobody at the stick.
local remoteScram, remoteAt = false, nil

-- The scramble's redstone half: a plain ON/OFF lever, wired in parallel with K_SCRAMBLE.
-- Either source alone calls it, which is the point -- see the header on why the one command
-- that exists for when everything has gone wrong does not run solely through the typewriter.
local function leverScramble()
    return redstone.getInput(VS.SIDE) and true or false
end

-- The vertical had a trim() of its own here until 2026-09-08 -- it HELD on release instead
-- of fading, so it could not be written in terms of integrate(): the goal for "no key" was
-- the current value rather than zero. Spring-loaded, that difference disappears entirely and
-- the vertical is just another call to integrate() with VS.RATE / VS.FADE as its two rates,
-- which is precisely what "just like the fore/aft/strafe controls" means. Deleted rather
-- than kept beside the new path: two ways to move the same accumulator is how the two ends
-- drift apart.

-- RECEIVE: one loop, no protocol filter, dispatched on the returned protocol. Two filtered
-- loops would each pull -- and drop -- the other's messages.
local function receiveLoop()
    while true do
        local sender, msg, proto = rednet.receive(nil, TICK)
        if sender and type(msg) == "table" then
            if proto == IN_APNAV then
                apnav = msg.autopilot_nav and true or false
            elseif proto == IN_ENGINE then
                -- Scout has ONE physical switch feeding master/altitude/attitude alike,
                -- because on this airframe they are the same engines (see 79/thrustcon.lua).
                mst, mstHeard = (msg.master and true or false), true
            elseif IN_SCRAMBLE and proto == IN_SCRAMBLE then
                remoteScram, remoteAt = (msg.scramble == true), os.clock()
            end
        end
    end
end

-- DISPLAY --------------------------------------------------------------------
-- Everything is drawn as COLOURED CHARACTER CELLS -- a space written on a coloured
-- background -- the same idiom #21's virtual stick uses, and the one drawing primitive
-- every CC monitor is guaranteed to support. No subpixel glyphs: nothing here moves fast
-- enough to need the resolution, and a cell is the smallest thing that exists.
--
-- The layout adapts to whatever size it is given:
--     cols 1..boxW      the translation box   -- strafe across, fore/aft up the screen
--     row  yawRow       the yaw strip         -- nose-left/right, under the box
--     col  boxW+1       gap
--     cols vsX..SW      the VS column         -- climb above the neutral rule, descent below
--     bottom rows       the readouts
local SW, SH = 0, 0

local function line(row, text, colour)
    if row < 1 or row > SH then return end
    term.setTextColour(colour or PAL.text)
    term.setBackgroundColour(PAL.panel)
    term.setCursorPos(1, row); term.clearLine(); term.write(string.sub(text, 1, SW))
end

-- A run of colour cells on one row -- the workhorse. One write per run rather than per cell.
local function bar(x1, x2, y, col)
    if y < 1 or y > SH then return end
    x1, x2 = math.max(x1, 1), math.min(x2, SW)
    if x2 < x1 then return end
    term.setCursorPos(x1, y); term.setBackgroundColour(col)
    term.write(string.rep(" ", x2 - x1 + 1))
end
local function cell(x, y, col) bar(x, x, y, col) end

-- One glyph per key for the "keys down" echo. The horizontal six keep their own letters, in
-- the physical cluster order, so the field reads like the keyboard: W E R over S D F. The
-- vertical four get SYMBOLS instead of letters -- `^` `v` `+` `!` -- because the point of
-- the field is to separate a dead typewriter from a dead radio at a glance, and a group that
-- does not look like the other group is quicker to read than SPACE/Z/A/X spelled out. A key
-- not down shows as a dot, so the field is fixed-width and never reshuffles while the pilot
-- is watching it. NAVCAPS and VSCAPS are drawn as two runs with a gap between them.
local NAVCAPS = {
    { "W", K_STRAFE_LEFT }, { "E", K_FORE },  { "R", K_STRAFE_RIGHT },
    { "S", K_YAW_LEFT },    { "D", K_AFT },   { "F", K_YAW_RIGHT },
}
local VSCAPS = {
    { "^", K_VS_UP }, { "v", K_VS_DN }, { "+", K_BOOST }, { "!", K_SCRAMBLE },
}
local function caps(set, tbl)
    local s = ""
    for _, k in ipairs(tbl) do s = s .. (held(set, k[2]) and k[1] or ".") end
    return s
end

-- `boost` is passed in rather than re-read off `set` here: the panel must show the ceilings
-- THIS tick's command was integrated against, and re-sampling the keyboard inside the draw
-- would let the two disagree by a frame at exactly the moment the pilot is watching.
local function draw(cmdVS, lever, scram, fwd, strafe, yaw, set, keyOk, locked, remote, boost)
    SW, SH = term.getSize()
    term.setBackgroundColour(PAL.panel)

    -- Rows reserved for the readouts, and how wide the VS column gets. Both shrink before
    -- the translation box does: the box is the instrument, the numbers are the backup.
    local textRows = (SH >= 9) and 3 or ((SH >= 6) and 2 or ((SH >= 4) and 1 or 0))
    local gh       = SH - textRows
    local vsW      = (SW >= 14) and 3 or 2
    local boxW     = SW - vsW - 1

    if boxW < 5 or gh < 3 then
        -- Too small to draw anything meaningful: numbers only, one axis per row, most
        -- important first. The stick still flies. `line` drops anything past the bottom,
        -- so this degrades a row at a time rather than needing a case per size.
        term.clear()
        -- "SCRM", not "SCRAMBLE": line() truncates, and a 7-column panel would render the
        -- long word as "SCRAMBL", which reads as a display fault rather than as a command.
        -- It is also the token row 1 of the full panel uses, so the two agree.
        line(1, scram and "SCRM" or string.format("V%+5.2f", cmdVS),
            scram and PAL.scram or PAL.text)
        line(2, string.format("F%+5.1f", fwd), PAL.stick)
        line(3, string.format("S%+5.1f", strafe), PAL.stick)
        line(4, string.format("Y%+5.2f", yaw), PAL.stick)
        return
    end

    -- The yaw strip takes the bottom graphics row once there is one to spare.
    local yawRow = (gh >= 4) and gh or nil
    local boxH   = yawRow and (gh - 1) or gh
    local vsX    = SW - vsW + 1

    -- ---- translation box ----
    -- Strafe -> column, fore/aft -> row, as functions, so the GRATICULE is placed by asking
    -- where zero lands rather than by halving the box separately. Rounded independently the
    -- two disagree on an even-sized box, and a "centred" stick that does not sit on its own
    -- centre cross looks broken. Forward draws UP (the view from above, nose at the top).
    -- EVERY SCALE ON THIS PANEL IS THE BOOSTED ONE, held or not. A gauge whose scale changes
    -- under the pilot's eyes is the classic instrument sin -- full deflection would mean two
    -- different speeds depending on a key he may not remember pressing. Drawn against the
    -- boosted ceiling the unboosted stick simply reaches half travel, and the extra range is
    -- visible as room the boost can still reach into.
    local function colFor(s) return 1 + round((clamp(s / NAV.BOOST_STRAFE_MAX, -1, 1) + 1) / 2 * (boxW - 1)) end
    local function rowFor(f) return 1 + round((1 - (clamp(f / NAV.BOOST_FWD_MAX, -1, 1) + 1) / 2) * (boxH - 1)) end

    for y = 1, boxH do bar(1, boxW, y, PAL.face) end
    local mx, my = colFor(0), rowFor(0)
    bar(1, boxW, my, PAL.grid)
    for y = 1, boxH do cell(mx, y, PAL.grid) end

    local cx, cy = colFor(strafe), rowFor(fwd)
    bar(1, boxW, cy, PAL.sight)
    for y = 1, boxH do cell(cx, y, PAL.sight) end
    cell(cx, cy, locked and PAL.idle or PAL.stick)

    -- ---- yaw strip ----
    -- Yaw is a RATE, not a position, so it gets a bar growing out of the centre rather than
    -- a point: how hard the nose is being asked round, and which way.
    if yawRow then
        bar(1, boxW, yawRow, PAL.face)
        local yc = 1 + round((boxW - 1) / 2)
        cell(yc, yawRow, PAL.grid)
        -- Only paint the bar when the nose is actually being asked round: an amber cell
        -- sitting on the centre mark at rest reads as a standing yaw command that is not
        -- there, and the whole value of this strip is that it is empty when it should be.
        if yaw ~= 0 then
            local yx = 1 + round((clamp(yaw / NAV.YAW_MAX, -1, 1) + 1) / 2 * (boxW - 1))
            bar(math.min(yc, yx), math.max(yc, yx), yawRow, locked and PAL.idle or PAL.yawc)
        end
    end

    -- ---- VS column ----
    -- Climb fills upward from the neutral rule, descent downward, so the rule itself is the
    -- hover a released stick falls back to. A scramble is not a rate and does not belong on
    -- the same scale: it floods the whole climb half instead.
    for y = 1, gh do bar(vsX, SW, y, PAL.face) end
    local function vsRow(v)   -- boosted scale, same reasoning as the translation box above
        return 1 + round((1 - (clamp(v / VS.BOOST_MAX_CMD, -1, 1) + 1) / 2) * (gh - 1))
    end
    local ny = vsRow(0)
    bar(vsX, SW, ny, PAL.grid)
    if scram then
        for y = 1, ny do bar(vsX, SW, y, PAL.scram) end
    else
        local vy = vsRow(cmdVS)
        local col = (cmdVS > 0.005 and PAL.climb) or (cmdVS < -0.005 and PAL.desc) or PAL.grid
        for y = math.min(ny, vy), math.max(ny, vy) do bar(vsX, SW, y, col) end
    end

    -- ---- readouts ----
    local function rightAt(row, s, colour)
        if row < 1 or row > SH then return end
        term.setCursorPos(math.max(1, SW - #s + 1), row)
        term.setTextColour(colour); term.setBackgroundColour(PAL.panel)
        term.write(string.sub(s, 1, SW))
    end

    -- Pick the widest of `cands` that still leaves room for the right-aligned field beside
    -- it, or nothing. A left field is DROPPED, never truncated: `S+2` is a number the pilot
    -- can read and believe, and a half-written key echo says nothing at all -- whereas an
    -- empty gap is honestly empty, and the box and the columns are still drawing the same
    -- quantities graphically. It is also why the right-hand field is the one that always
    -- survives: the climb rate and the link state have no graphical twin.
    local function fit(cands, right)
        for _, c in ipairs(cands) do
            if #c + 1 + #right <= SW then return c end
        end
        return ""
    end

    local r0 = SH - textRows + 1
    -- Row 1: the commanded body velocities (amber, matching the stick cell) and the
    -- commanded climb rate right-aligned so it lands under the column it belongs to.
    if textRows >= 1 then
        local rs = scram and "SCRM" or string.format("%+5.2f", cmdVS)
        line(r0, fit({ string.format("F%+5.1f S%+5.1f", fwd, strafe),
                       string.format("F%+3.0f S%+3.0f", fwd, strafe),
                       string.format("F%+5.1f", fwd),
                       string.format("F%+3.0f", fwd) }, rs),
            locked and PAL.idle or PAL.stick)
        rightAt(r0, rs, scram and PAL.scram or PAL.text)
    end
    -- Row 2: which keys are down right now -- the quickest way to tell a dead typewriter
    -- from a dead radio -- and the link/lockout state.
    --   !M no modem   !K typewriter not answering   AP  AP-NAV has the horizontal
    --   MST engine master off   BST boost held   TX  transmitting, the pilot has it
    if textRows >= 2 then
        local kn, kv = caps(set, NAVCAPS), caps(set, VSCAPS)
        -- BST is deliberately BELOW the fault and lockout tags and above plain TX: a boosted
        -- stick that is being ignored is a lockout to report, not a boost to celebrate, and
        -- the `+` in the key echo beside it says the key is down either way.
        local tag = (not modem and "!M") or (not keyOk and "!K")
            or (LOCK_ON_APNAV and apnav and "AP")
            or (LOCK_ON_MASTER and mstHeard and not mst and "MST")
            or (boost and "BST") or "TX"
        -- The vertical group is the half that survives a squeeze: the horizontal six already
        -- have the box and the yaw strip drawing them, and these keys have nothing else.
        line(r0 + 1, fit({ string.format("Y%+5.2f %s %s", yaw, kn, kv),
                           string.format("Y%+5.2f %s", yaw, kv),
                           string.format("Y%+5.2f", yaw),
                           string.format("Y%+4.1f", yaw) }, tag),
            locked and PAL.idle or PAL.stick)
        rightAt(r0 + 1, tag, (tag == "TX" and PAL.ok) or (tag == "BST" and PAL.climb)
            or PAL.warn)
    end
    -- Row 3: the scramble LEVER, which is the one input on this computer with no other
    -- feedback anywhere on the panel -- the vertical keys have the VS column and the key
    -- echo, the horizontal six have the box, and the boost has BST. A wrong or unwired face then reads as a switch that
    -- never moves, rather than as an aeroplane that will not scramble.
    -- A REMOTE scramble is shown here too, and only here. It is invisible to the rest of the
    -- aircraft by design (see the OR in commandLoop), but the pilot of THIS computer must be
    -- able to see that the aeroplane is pulling up on its own and that he did not ask for it
    -- -- an unannounced full-lift command is how a pilot ends up fighting his own aircraft.
    -- AUTO outranks the lever in the text because it is the surprising one.
    if textRows >= 3 then
        local src = remote and "AUTO" or (lever and "ON" or "off")
        line(r0 + 2, fit({ string.format("SCRM lever %s", src),
                           string.format("SCRM lvr %s", src),
                           string.format("SCRM %s", remote and "AUTO" or (lever and "ON" or "--")) }, ""),
            (lever or remote) and PAL.scram or PAL.text)
    end
    term.setBackgroundColour(PAL.panel); term.setTextColour(PAL.text)
end

-- MAIN -----------------------------------------------------------------------
local function commandLoop()
    local lastTime = os.clock()
    local cmdVS = 0                     -- slewed vertical command, m/s
    local fwd, strafe, yaw = 0, 0, 0    -- integrated body-frame command
    local shown, since = nil, 0         -- last drawn state / frames since, for the redraw gate

    while true do
        local now = os.clock()
        local dt  = math.max(now - lastTime, 0.001)
        lastTime  = now

        -- ---- the keyboard, read ONCE for both channels ----
        -- One getPressedKeyCodes() per tick, not two: the second call would be a second
        -- sample of a moving thing, and the two channels would disagree about which keys
        -- were down on the same frame.
        local set, keyOk = pressedSet()

        -- ---- vertical ----
        -- ANY scramble source alone is enough; they are wired in parallel, not ANDed. There
        -- are three: the key, the redstone lever, and a REMOTE request from another computer
        -- on this aircraft (CFG.IN_SCRAMBLE -- Scout's #78 automatic pull-up).
        --
        -- THE REMOTE ONE JOINS HERE, AND NOWHERE ELSE, AND THAT IS THE WHOLE POINT. An
        -- automatic pull-up has to be indistinguishable from the pilot's own detent to
        -- everything downstream -- the UAC, the GPWS, the relays -- so it is folded into
        -- `scram` BEFORE the broadcast and the packet on OUT_VERT is byte-for-byte the shape
        -- it has always been. Nothing downstream learns a new field, gets a second sender, or
        -- can tell the two apart; the pull-up is completely invisible until it engages, and
        -- when it does it IS a scramble. It also inherits every property of one for free: it
        -- outranks ALT-AP and an autoland (CFG.MANCON_SCRAMBLE_OVERRIDE), it freezes rather
        -- than drives the trim, and it ages out on the UAC's own staleness gate.
        --
        -- #78 used to broadcast on OUT_VERT itself, which is what this replaces. Two senders
        -- on one protocol alternate at 20 Hz and the last to speak wins, so the pilot's own
        -- scramble flickered off on every other packet and his trim was overwritten with #78's
        -- `vs = 0.0` -- the fault "the scramble does not work" turned out to be, 2026-08-30.
        -- ONE PROTOCOL, ONE BROADCASTER. It is why #13 and #14 are retired.
        local lever = leverScramble()
        local remote = (IN_SCRAMBLE and remoteScram and remoteAt
            and (now - remoteAt) <= IN_SCRAMBLE_STALE) or false
        local scram = lever or held(set, K_SCRAMBLE) or remote

        -- BOOST: a modifier, not a command. It picks which set of ceilings this tick's
        -- integrations run against and does nothing else -- no separate packet field, no
        -- state of its own, nothing downstream that has to learn about it. Reading it here,
        -- once, is what keeps the three axes agreeing about whether it is held.
        -- NOT gated by the lockout below: it does not need to be. A locked-out horizontal
        -- reads every key as released, so its accumulators fade to zero whatever ceiling
        -- they are nominally allowed -- and the vertical is not locked out at all, by design.
        local boost      = held(set, K_BOOST)
        local vsMax      = boost and VS.BOOST_MAX_CMD or VS.MAX_CMD
        local fwdMax     = boost and NAV.BOOST_FWD_MAX or NAV.FWD_MAX
        local strafeMax  = boost and NAV.BOOST_STRAFE_MAX or NAV.STRAFE_MAX
        -- ---- vertical ----
        -- The same integrator the horizontal six use, on VS.RATE / VS.FADE: hold to ramp
        -- toward the ceiling, let go and it falls back to zero. Opposing keys cancel, which
        -- is the fade branch, which is also what the retired C key used to reach by hand.
        --
        -- A SCRAMBLE NO LONGER FREEZES THIS, and the reason the freeze existed went with the
        -- trim. cmdVS was a PARKED SETTING then, so a scramble had to preserve it or the
        -- pilot would come out of the pull-up with his trim overwritten. It is a spring now,
        -- and a spring's honest reading of "the vertical keys are not being obeyed right now"
        -- is that the command decays to zero -- so coming out of a scramble with no key held
        -- leaves the aircraft in a hover rather than resuming a climb (or, far worse, a
        -- descent) nobody is asking for. Running the integrator unconditionally also removes
        -- the special case: the UAC ignores `vs` outright for as long as `scramble` is set
        -- (see the ui.scramble branch in uac.lua's vertical cascade), so what this computer
        -- puts in the field during a scramble cannot reach a thruster either way.
        --
        -- The vertical is NOT gated by the lockout below, exactly as #6 was gated by nothing:
        -- AP-NAV does not own the vertical channel (ALT-AP does, and that is the UAC's own
        -- gate), so the stick stays live under AP-NAV and answers the moment ALT-AP drops.
        cmdVS = integrate(cmdVS, held(set, K_VS_UP), held(set, K_VS_DN),
            vsMax, VS.RATE, VS.FADE, dt)
        if math.abs(cmdVS) < 1e-3 then cmdVS = 0 end

        -- ---- horizontal / yaw ----
        local locked = (LOCK_ON_APNAV and apnav)
            or (LOCK_ON_MASTER and mstHeard and not mst) or false
        local fore   = (not locked) and held(set, K_FORE)         or false
        local aft    = (not locked) and held(set, K_AFT)          or false
        local sleft  = (not locked) and held(set, K_STRAFE_LEFT)  or false
        local sright = (not locked) and held(set, K_STRAFE_RIGHT) or false
        local yleft  = (not locked) and held(set, K_YAW_LEFT)     or false
        local yright = (not locked) and held(set, K_YAW_RIGHT)    or false

        -- Boosted ceilings, unboosted RATES: releasing the boost while still holding the key
        -- therefore walks the command back down at NAV.RAMP rather than NAV.FADE, which is a
        -- gentler deceleration than letting go of the key altogether -- the right way round.
        fwd    = integrate(fwd,    fore,   aft,   fwdMax,      NAV.RAMP,     NAV.FADE,     dt)
        strafe = integrate(strafe, sright, sleft, strafeMax,   NAV.RAMP,     NAV.FADE,     dt)
        yaw    = integrate(yaw,    yright, yleft, NAV.YAW_MAX, NAV.YAW_RAMP, NAV.YAW_FADE, dt)

        -- Snap tiny residues to exactly 0 so a fully-faded axis commands a clean hold (the
        -- UAC's null-the-drift / hold-this-heading path is an exact no-op only at 0).
        if math.abs(fwd)    < 1e-3 then fwd    = 0 end
        if math.abs(strafe) < 1e-3 then strafe = 0 end
        if math.abs(yaw)    < 1e-4 then yaw    = 0 end

        -- ---- broadcast ----
        -- Both every tick; see the header on why silence is not an option. `scramble = ... or
        -- nil` so a non-scramble packet is byte-for-byte the shape it has always been.
        if modem then
            rednet.broadcast({ vs = cmdVS, scramble = scram or nil }, OUT_VERT)
            rednet.broadcast({
                fwd    = fwd,      -- m/s,   body forward +, aft -
                strafe = strafe,   -- m/s,   body right +, left -
                yaw    = yaw,      -- rad/s, nose-right +, left -
                active = (fore or aft or sleft or sright or yleft or yright),
            }, OUT_NAV)
        end

        -- Redraw only when something actually moved (plus a periodic refresh so the status
        -- line stays honest). The broadcasts above must be every tick -- the UAC reads
        -- silence as an unplugged stick -- but repainting the whole panel 20x/s while the
        -- pilot's hands are off the keys is pure waste.
        local state = string.format("%.2f|%s|%s|%s|%.2f|%.2f|%.3f|%s|%s|%s%s%s%s%s",
            cmdVS, tostring(lever), tostring(remote), tostring(scram), fwd, strafe, yaw,
            tostring(keyOk), tostring(locked),
            table.concat({ fore and 1 or 0, aft and 1 or 0, sleft and 1 or 0,
                sright and 1 or 0, yleft and 1 or 0, yright and 1 or 0 }),
            held(set, K_VS_UP) and 1 or 0, held(set, K_VS_DN) and 1 or 0,
            boost and 1 or 0, held(set, K_SCRAMBLE) and 1 or 0)
        since = since + 1
        if state ~= shown or since >= 20 then
            draw(cmdVS, lever, scram, fwd, strafe, yaw, set, keyOk, locked, remote, boost)
            shown, since = state, 0
        end

        sleep(TICK)
    end
end

local ok, err = pcall(function() parallel.waitForAny(receiveLoop, commandLoop) end)

-- Hand the display back the way we found it: this panel repaints the palette, and leaving a
-- monitor (or this terminal) recoloured would confuse whatever runs next.
if COLOUR and term.nativePaletteColour then
    pcall(function()
        for _, c in ipairs({ colours.black, colours.blue, colours.grey, colours.cyan,
                colours.orange, colours.yellow, colours.green, colours.red }) do
            term.setPaletteColour(c, term.nativePaletteColour(c))
        end
    end)
end
-- Blank whatever we were drawing to FIRST (that is the monitor, if redirected -- leaving the
-- last frame frozen on it would read as a still-live stick), then come back to the
-- computer's own terminal to report.
term.setBackgroundColour(colours.black); term.setTextColour(colours.white)
term.clear(); term.setCursorPos(1, 1)
if monitor then term.redirect(term.native()) end
if not ok then print("Scout MANCON stopped: " .. tostring(err)) end
