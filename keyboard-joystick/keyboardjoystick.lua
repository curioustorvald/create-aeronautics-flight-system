-- AER TOGEKISS -- KEYBOARD JOYSTICK (#21)
--
-- Keyboard to pitch/roll TARGET and throttle. Holding a key increments/decrements the
-- commanded attitude (max +-60 deg on each axis) and the throttle lever; the UAC (#1)
-- flies it. Draws a 2D virtual stick (pitch/roll target) and a virtual throttle stick.
--
-- The keypresses are read from a TYPEWRITER peripheral's getPressedKeyCodes() (see
-- get_keycodes.lua beside this file, which is how the codes below were found -- they are
-- GLFW codes, i.e. the ESDF cluster + space + A/Z):
--     E (69) nose up     D (68) nose down     S (83) roll left    F (70) roll right
--     SPACE (32) throttle up          A (65) / Z (90) throttle down
--     C (67) centre the stick (pitch and roll straight back to 0)
--
-- STICK BEHAVIOUR -- this is a TRIM-style stick, not a spring-loaded one: a key HELD ramps
-- the target at STICK_RATE; a key RELEASED leaves the target exactly where it was. That is
-- what "increment/decrement the target" means and it is the only thing a binary keyboard
-- can do smoothly -- but it also means the craft will sit at whatever bank you last
-- commanded, so C (centre) is the way back to level, not letting go. Opposing keys held
-- together (E+D / S+F) cancel and hold. The throttle behaves the same way, as a real
-- throttle lever does.
--
-- IMPLEMENTATION NOTES -------------------------------------------------------
-- Like the MIC (#14) and the VSC (#6), this computer runs NO thruster control of its own:
-- it only NAMES a wanted attitude and engine power, and the UAC owns every loop that buys
-- it. The packet is broadcast EVERY tick (not just on change) because the UAC gates on
-- freshness -- a silent joystick is read as "no joystick" and the autopilot/MIC take the
-- craft back, so a crashed or unplugged #21 cannot leave a stale bank latched in.
--
-- The UAC acts on this ONLY in FIXEDWING mode, and only with AP-NAV off (the same handoff
-- gate every other manual channel uses -- the FMS and a hand-held stick must never fight
-- over one attitude setpoint). In VTOL/HYBRID, or with the autopilot flying, these packets
-- are received and ignored. Everything past this file -- limits, sign calibration, how
-- bank becomes a turn -- lives in the UAC's CFG.KJOY_* block in pid/uacconf.lua; the values
-- here are just the stick's own feel and its own hard +-60 deg stop.

-- CONFIG --------------------------------------------------------------------
local OUT_PROTOCOL = "aertogekiss_kbdjoy"   -- out -> UAC (#1), must match CFG.KBDJOY_PROTOCOL

-- Key codes, as reported by the typewriter (get_keycodes.lua prints them). Each entry is a
-- LIST so an action can have several keys -- the spec gives throttle-down as "65 or 90".
local K_PITCH_UP   = { 69 }       -- E
local K_PITCH_DOWN = { 68 }       -- D
local K_ROLL_LEFT  = { 83 }       -- S
local K_ROLL_RIGHT = { 70 }       -- F
local K_THR_UP     = { 32 }       -- space
local K_THR_DOWN   = { 65, 90 }   -- A or Z
local K_CENTRE     = { 67 }       -- C

-- Hard stops on the commanded attitude (deg). The UAC clamps again (CFG.KJOY_PITCH_MAX /
-- CFG.KJOY_ROLL_MAX) -- keep the two in step, and note both deliberately exceed the
-- autopilot's own CFG.BANK_MAX (45 deg): a hand-flown aircraft may bank harder than the
-- FMS ever plans for.
local PITCH_MAX = 60.0
local ROLL_MAX  = 60.0

-- Stick feel. STICK_RATE is the pilot's roll/pitch rate: full deflection from centre takes
-- PITCH_MAX/STICK_RATE seconds (60/45 = 1.3 s). This -- not the UAC -- is what should set
-- how fast the commanded attitude moves, which is why CFG.KJOY_SLEW_RATE over there is set
-- fast enough to just track this demand rather than adding a second lag on top of it.
local STICK_RATE  = 45.0    -- deg/s of target change while a pitch/roll key is held
local THR_RATE    = 50.0    -- %/s of throttle change while a throttle key is held
local CENTRE_SNAP = true    -- C snaps to 0 instantly; false = ramp back at STICK_RATE

local TICK = 0.05           -- s between read/integrate/broadcast steps (matches the UAC tick)

-- SETUP ---------------------------------------------------------------------
-- Find the keyboard by CAPABILITY, not by peripheral type or by a hardcoded side: the only
-- thing we actually require is getPressedKeyCodes(), and get_keycodes.lua's peripheral.wrap
-- ('front') would break the moment the typewriter is moved.
local function findKeyboard()
    for _, name in ipairs(peripheral.getNames()) do
        local p = peripheral.wrap(name)
        if type(p) == "table" and type(p.getPressedKeyCodes) == "function" then
            return p, name
        end
    end
end

local keyboard, keyboardName = findKeyboard()
if not keyboard then
    error("No typewriter attached (nothing exposes getPressedKeyCodes). "
        .. "Place one against this computer and rerun.", 0)
end

local modem = peripheral.find("modem")
if modem then
    rednet.open(peripheral.getName(modem))
else
    printError("No modem attached; the UAC will not hear " .. OUT_PROTOCOL)
end

-- Draw on a monitor if one is attached (the stick view wants the room), else on the
-- computer's own terminal. Redirect rather than mirror so there is exactly one draw path.
local monitor = peripheral.find("monitor")
if monitor then
    monitor.setTextScale(0.5)
    term.clear()
    term.setCursorPos(1, 1)
    print("Keyboard joystick: display on the monitor.")
    term.redirect(monitor)
end

local COLOUR = term.isColour and term.isColour() or false

-- Instrument palette. Every drawing call names a ROLE, never a raw colour, so the whole
-- panel restyles from here -- and so a basic (non-advanced) computer, which only has
-- white/lightGrey/grey/black, still gets a legible monochrome version instead of a
-- "Colour not supported" crash.
local PAL
if COLOUR then
    PAL = {
        panel = colours.black,  face  = colours.blue,    grid  = colours.grey,
        sight = colours.cyan,   stick = colours.orange,
        thrLo = colours.green,  thrMid = colours.yellow, thrHi = colours.red,
        text  = colours.white,  ok     = colours.green,
        warn  = colours.red,
    }
    -- Repaint the palette for an instrument look, exactly as the FMS does for its HSI
    -- (autopilot.lua) and uac.lua for its horizon. pcall: setPaletteColour is advanced-only,
    -- and a monitor that refuses must not be fatal.
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
    -- Basic computers have only white/lightGrey/grey/black. Face collapses into the panel.
    PAL = {
        panel = colours.black,  face  = colours.black,      grid  = colours.grey,
        sight = colours.grey,   stick = colours.white,
        thrLo = colours.grey,   thrMid = colours.lightGrey, thrHi = colours.white,
        text  = colours.white,  ok     = colours.lightGrey,
        warn  = colours.white,
    }
end

term.setBackgroundColour(PAL.panel)
term.setTextColour(PAL.text)
term.clear()

-- HELPERS -------------------------------------------------------------------
local function clamp(v, lo, hi) return math.max(lo, math.min(hi, v)) end
local function round(x) return math.floor(x + 0.5) end

-- Collapse whatever shape getPressedKeyCodes() returns into a set of codes. The typewriter
-- hands back a LIST of the codes currently down ({ [1]=69, [2]=32 } -- which is why
-- get_keycodes.lua notes "simultaneous key presses will write that much characters"), but
-- accept a set ({ [69]=true }) too so a mod update to either shape cannot silently give a
-- permanently-centred stick.
local function pressedSet()
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

-- One axis of the stick: hold a direction and the target ramps that way at `rate`; release
-- (or hold both directions) and it FREEZES where it is. No self-centring -- see the header.
local function ramp(cur, pos, neg, maxVal, rate, dt)
    local d = 0
    if pos and not neg then d =  rate * dt
    elseif neg and not pos then d = -rate * dt end
    return clamp(cur + d, -maxVal, maxVal)
end

-- DISPLAY -------------------------------------------------------------------
-- THE PANEL IS 15 COLUMNS x 10 ROWS. That is the monitor actually fitted to this computer,
-- and it is the whole design constraint: with 150 cells there is no room for bezels, labels
-- or decoration, so every cell has to carry information.
--
-- Everything is drawn as COLOURED CHARACTER CELLS -- a space written on a coloured background
-- -- which is the same idiom the FMS's HSI uses (autopilot.lua's hFill/hLine/hText) and the
-- one drawing primitive every CC:Tweaked terminal and monitor is guaranteed to support. No
-- sub-character/glyph tricks: a cell is the smallest thing that exists here.
--
-- The layout adapts to whatever size it is given (a bigger monitor simply gets a bigger box),
-- but it is TUNED for 15x10:
--     cols 1-11  the stick box      rows 1-8   (roll across, pitch up the screen)
--     col  12    gap
--     cols 13-15 the throttle       rows 1-8   (fills from the bottom)
--     row  9     P/R readout        row 10     throttle % + link state
local SW, SH = 0, 0   -- display size, refreshed by draw()

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

-- One letter per key for the bottom row's "keys down" echo. Fixed order, so the field never
-- reshuffles while the pilot is watching it.
local KEYCAPS = {
    { "E", K_PITCH_UP }, { "D", K_PITCH_DOWN }, { "S", K_ROLL_LEFT }, { "F", K_ROLL_RIGHT },
    { "^", K_THR_UP },   { "v", K_THR_DOWN },   { "C", K_CENTRE },
}

-- The instrument. Left: the stick box -- horizontal axis is the ROLL target, vertical axis the
-- PITCH target with nose-up at the TOP (the view from behind the aircraft). The commanded
-- point is a bright cell, and its row and column are tinted so the position is readable even
-- when the cursor sits on the graticule. Right: the throttle, filling from the bottom.
local function draw(pitch, roll, thr, set, keyOk)
    SW, SH = term.getSize()
    term.setBackgroundColour(PAL.panel)

    -- Rows reserved for the readouts, and how wide the throttle gets. Both shrink before the
    -- stick box does: the box is the instrument, the numbers are the backup.
    local textRows = (SH >= 6) and 2 or ((SH >= 4) and 1 or 0)
    local gh       = SH - textRows                      -- graphics rows
    local thrW     = (SW >= 12) and 3 or 2
    local boxW     = SW - thrW - 1                      -- -1 for the gap column

    if boxW < 5 or gh < 3 then
        -- Too small to draw anything meaningful: numbers only. The stick still flies.
        term.clear()
        if SH >= 1 then line(1, string.format("P%+3.0f R%+3.0f", pitch, roll), PAL.stick) end
        if SH >= 2 then line(2, string.format("T%3.0f%%", thr), PAL.text) end
        return
    end
    local thrX = SW - thrW + 1

    -- ---- stick box ----
    -- Roll -> column, pitch -> row, as functions, so the GRATICULE is placed by asking where
    -- level lands rather than by halving the box separately. Rounded independently the two
    -- disagree on an even-sized box, and a "centred" stick that does not sit on its own
    -- centre cross looks broken. Pitch is inverted for the screen: nose-UP must draw UP.
    local function colFor(r) return 1 + round((r / ROLL_MAX + 1) / 2 * (boxW - 1)) end
    local function rowFor(p) return 1 + round((1 - (p / PITCH_MAX + 1) / 2) * (gh - 1)) end

    for y = 1, gh do bar(1, boxW, y, PAL.face) end                       -- lit face
    local mx, my = colFor(0), rowFor(0)                                  -- centre = level
    bar(1, boxW, my, PAL.grid)
    for y = 1, gh do cell(mx, y, PAL.grid) end

    local cx, cy = colFor(roll), rowFor(pitch)
    bar(1, boxW, cy, PAL.sight)                                          -- sight lines
    for y = 1, gh do cell(cx, y, PAL.sight) end
    cell(cx, cy, PAL.stick)                                              -- the stick itself

    -- ---- throttle ----
    for y = 1, gh do bar(thrX, SW, y, PAL.face) end
    local fill = round(thr / 100 * gh)
    for i = 1, fill do
        local frac = i / gh
        bar(thrX, SW, gh - i + 1,
            (frac > 0.85 and PAL.thrHi) or (frac > 0.60 and PAL.thrMid) or PAL.thrLo)
    end

    -- ---- readouts ----
    -- Right-align a short field on `row`, so a number sits under the thing it describes.
    local function rightAt(row, s, colour)
        term.setCursorPos(math.max(1, SW - #s + 1), row)
        term.setTextColour(colour); term.setBackgroundColour(PAL.panel)
        term.write(string.sub(s, 1, SW))
    end

    -- Row 1 of 2: commanded attitude under the box (amber, matching the stick cell), and the
    -- throttle % right-aligned so it lands under the throttle bar rather than floating.
    if textRows >= 1 then
        local r = SH - textRows + 1
        line(r, string.format("P%+3.0f R%+3.0f", pitch, roll), PAL.stick)
        rightAt(r, string.format("%3.0f%%", thr), PAL.text)
    end
    -- Row 2 of 2: which keys are down right now (the quickest way to tell a dead typewriter
    -- from a dead radio), then the link state -- TX transmitting, !M no modem, !K the
    -- typewriter stopped answering. Fixed order so the field never jumps about.
    if textRows >= 2 then
        local ks = ""
        for _, k in ipairs(KEYCAPS) do if held(set, k[2]) then ks = ks .. k[1] end end
        line(SH, ks, PAL.stick)
        local tag = (not keyOk and "!K") or (not modem and "!M") or "TX"
        rightAt(SH, tag, (tag == "TX") and PAL.ok or PAL.warn)
    end
    term.setBackgroundColour(PAL.panel); term.setTextColour(PAL.text)
end

-- MAIN ----------------------------------------------------------------------
local function stickLoop()
    local lastTime = os.clock()
    local pitch, roll, thr = 0, 0, 0
    local shown, since = nil, 0   -- last drawn state / frames since, for the redraw gate

    while true do
        local now = os.clock()
        local dt  = math.max(now - lastTime, 0.001)
        lastTime  = now

        local set, keyOk = pressedSet()

        if held(set, K_CENTRE) then
            if CENTRE_SNAP then
                pitch, roll = 0, 0
            else
                pitch = ramp(pitch, pitch < 0, pitch > 0, math.abs(pitch), STICK_RATE, dt)
                roll  = ramp(roll,  roll  < 0, roll  > 0, math.abs(roll),  STICK_RATE, dt)
            end
        else
            pitch = ramp(pitch, held(set, K_PITCH_UP),   held(set, K_PITCH_DOWN),
                PITCH_MAX, STICK_RATE, dt)
            roll  = ramp(roll,  held(set, K_ROLL_RIGHT), held(set, K_ROLL_LEFT),
                ROLL_MAX,  STICK_RATE, dt)
        end
        -- Throttle is one-sided (0..100%): a lever, never reverse thrust.
        thr = clamp(thr + ((held(set, K_THR_UP) and 1 or 0) - (held(set, K_THR_DOWN) and 1 or 0))
            * THR_RATE * dt, 0, 100)

        -- Snap tiny residues so a centred stick commands EXACTLY level (the UAC's
        -- straight-and-level path is an exact no-op only at 0).
        if math.abs(pitch) < 1e-3 then pitch = 0 end
        if math.abs(roll)  < 1e-3 then roll  = 0 end

        if modem then
            rednet.broadcast({
                pitch    = math.rad(pitch),   -- rad, + = nose UP    (the UAC works in radians)
                roll     = math.rad(roll),    -- rad, + = roll RIGHT
                throttle = thr / 100,         -- 0..1 lever position
                pitchDeg = pitch,             -- degrees too, purely for readouts/debugging
                rollDeg  = roll,
                active   = (held(set, K_PITCH_UP) or held(set, K_PITCH_DOWN)
                    or held(set, K_ROLL_LEFT) or held(set, K_ROLL_RIGHT)
                    or held(set, K_THR_UP) or held(set, K_THR_DOWN)
                    or held(set, K_CENTRE)) and true or false,
            }, OUT_PROTOCOL)
        end

        -- Redraw only when something actually moved (plus a periodic refresh so the status
        -- line stays honest). The broadcast above must be every tick -- the UAC reads silence
        -- as an unplugged joystick -- but repainting the whole panel 20x/s while the
        -- pilot's hands are off the keys is pure waste.
        local state = string.format("%.2f|%.2f|%.1f|%s|%s", pitch, roll, thr,
            tostring(keyOk), table.concat({
                held(set, K_PITCH_UP) and 1 or 0, held(set, K_PITCH_DOWN) and 1 or 0,
                held(set, K_ROLL_LEFT) and 1 or 0, held(set, K_ROLL_RIGHT) and 1 or 0,
                held(set, K_THR_UP) and 1 or 0, held(set, K_THR_DOWN) and 1 or 0,
                held(set, K_CENTRE) and 1 or 0 }))
        since = since + 1
        if state ~= shown or since >= 20 then
            draw(pitch, roll, thr, set, keyOk)
            shown, since = state, 0
        end
        sleep(TICK)
    end
end

local ok, err = pcall(stickLoop)
-- Hand the display back the way we found it: the instrument repaints the palette, and
-- leaving a monitor (or this terminal) recoloured would confuse whatever runs next.
if COLOUR and term.nativePaletteColour then
    pcall(function()
        for _, c in ipairs({ colours.black, colours.grey, colours.lightGrey, colours.cyan,
                colours.orange, colours.yellow, colours.green, colours.red }) do
            term.setPaletteColour(c, term.nativePaletteColour(c))
        end
    end)
end
-- Blank whatever we were drawing to FIRST (that is the monitor, if redirected -- leaving the
-- last frame frozen on it would read as a still-live joystick), then come back to the
-- computer's own terminal to report.
term.setBackgroundColour(colours.black); term.setTextColour(colours.white)
term.clear(); term.setCursorPos(1, 1)
if monitor then term.redirect(term.native()) end
if not ok then print("Keyboard joystick stopped: " .. tostring(err)) end
