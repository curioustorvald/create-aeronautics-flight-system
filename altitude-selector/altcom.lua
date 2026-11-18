--Aer Togekiss Autopilot Altitude Computer

-- NOTE: setCursorPos, drawFilledBox use one-based index. 
-- drawFilledBox arguments: x-start, y-start, x-end-inclusive, y-end-inclusive, colour

local PANELS = 4
local PANEL_WIDTH = 12
local PANEL_STRIDE = PANEL_WIDTH + 1

local Y_INPUTWINDOW = 2 -- one-based
local Y_KEYPAD = 4
local Y_TOGGLEBTN = 17

term.setPaletteColour(colours.green, 0x116611)
term.setPaletteColour(colours.brown, 0x553322)
term.setPaletteColour(colours.orange, 0xffbb00)
term.setPaletteColour(colours.cyan, 0x117766)
term.setPaletteColour(colours.blue, 0x114488)
term.setPaletteColour(colours.red, 0x995500)

local oldAltSelection = nil
local altSelection = nil
local defaultAlti = {'160', '240', '320', '400'}

-- FMS OVERRIDE, three modes (learned from the aertogekiss_autoland stream the FMS
-- already sends the UAC):
--   NORMAL - ordinary ALT-AP: the pilot's selection (if any) broadcasts.
--   OVER   - the FMS's autoland owns the vertical channel and the UAC ignores our
--            altap, so the panel must not look live: all four GREEN, no broadcast, ALT
--            taps disabled. Entered on cmd hold/descend/landed.
--   PILOT  - the pilot tapped a panel DURING a landing (AS override): ALT-AP takes the
--            vertical back (the FMS pauses the descent but keeps position/heading
--            alignment), so we broadcast again and the selected panel is live. Entered
--            when the pilot taps while OVER; confirmed by the FMS's cmd 'release'.
-- Transitions: hold/descend/landed -> OVER (stashing the current pick); release -> PILOT
-- (keep the pilot's pick); cancel -> NORMAL (restore the stashed pick on a plain
-- teardown, or keep the pilot's pick if we were PILOT). A staleness timeout recovers
-- OVER if the FMS goes quiet (missed cancel / reboot). A short suppress window after an
-- override tap ignores the FMS's last few in-flight descend msgs so they don't yank us
-- straight back to OVER before it processes the override.
local AUTOLAND_PROTOCOL   = 'aertogekiss_autoland'
local ALTOVERRIDE_PROTOCOL = 'aertogekiss_altoverride'  -- AS -> FMS: "pilot wants ALT-AP"
local AUTOLAND_STALE      = 3.0   -- s without an 'active' autoland msg -> leave OVER
local mode                = 'NORMAL'   -- 'NORMAL' | 'OVER' | 'PILOT'
local savedSelection      = nil
local lastAutolandActive  = 0
local suppressUntil       = 0     -- os.clock() until which OVER re-entry is suppressed

local function newGuiStatus(index) -- one-based
    return {
        ['inputBuffer'] = '',
        ['currentAlti'] = defaultAlti[index],
        ['selected'] = false,  -- boot: nothing selected -> altSelection nil -> broadcast nothing
        ['index'] = index
    }
end

local guiStatus = { newGuiStatus(1), newGuiStatus(2), newGuiStatus(3), newGuiStatus(4) }

local function drawPanel(index) -- one-based
    local x = (index-1) * PANEL_STRIDE + 3
    local sta = guiStatus[index]
    
    -- draw input panel
    paintutils.drawFilledBox(x + 0, Y_INPUTWINDOW, x + PANEL_WIDTH - 4, Y_INPUTWINDOW, colours.brown)
    
    -- draw keypad
    paintutils.drawFilledBox(x, Y_KEYPAD, x + 8, Y_KEYPAD + 11, colours.blue)
    -- keypad checkerboard arrangements
    paintutils.drawFilledBox(x + 0, Y_KEYPAD + 0, x + 2, Y_KEYPAD + 2, colours.cyan)
    paintutils.drawFilledBox(x + 6, Y_KEYPAD + 0, x + 8, Y_KEYPAD + 2, colours.cyan)
    
    paintutils.drawFilledBox(x + 3, Y_KEYPAD + 3, x + 5, Y_KEYPAD + 5, colours.cyan)

    paintutils.drawFilledBox(x + 0, Y_KEYPAD + 6, x + 2, Y_KEYPAD + 8, colours.cyan)
    paintutils.drawFilledBox(x + 6, Y_KEYPAD + 6, x + 8, Y_KEYPAD + 8, colours.cyan)
    
    paintutils.drawFilledBox(x + 3, Y_KEYPAD + 9, x + 5, Y_KEYPAD + 11, colours.cyan)
    
    -- draw numbers
    term.setCursorPos(x, Y_KEYPAD + 1)  term.blit(' 1  2  3 ', '000000000', '999bbb999')
    term.setCursorPos(x, Y_KEYPAD + 4)  term.blit(' 4  5  6 ', '000000000', 'bbb999bbb')
    term.setCursorPos(x, Y_KEYPAD + 7)  term.blit(' 7  8  9 ', '000000000', '999bbb999')
    term.setCursorPos(x, Y_KEYPAD + 10) term.blit('Ent 0 Clr', '000000000', 'bbb999bbb')
    
    -- draw buttons
    paintutils.drawFilledBox(x, Y_TOGGLEBTN, x + 8, Y_TOGGLEBTN + 1, sta.selected and colours.red or colours.green)
    term.setCursorPos(x + 3, Y_TOGGLEBTN)
    term.blit(string.format('%3d', sta.currentAlti), '111', sta.selected and 'eee' or 'ddd')
    term.setCursorPos(x + 2, Y_TOGGLEBTN + 1)
    term.blit('ALT '..index, '11111', sta.selected and 'eeeee' or 'ddddd')
end


local function broadcastAlti()
    if altSelection and mode ~= 'OVER' then   -- NORMAL and PILOT broadcast; OVER stays silent
        local a = guiStatus[altSelection].currentAlti
        rednet.broadcast({ ['alti'] = a }, 'aertogekiss_altap')
    end
end

local function broadcastAltiLoop()
    while true do
        broadcastAlti()
        sleep(1)
    end
end

local function updateGUI(index)
    local x = (index-1) * PANEL_STRIDE + 3
    local sta = guiStatus[index]
    
    -- draw input panel
    term.setCursorPos(x + 3, Y_INPUTWINDOW)
    term.blit(string.format('%3s', sta.inputBuffer), '111', 'ccc')
    
    -- draw buttons
    paintutils.drawFilledBox(x, Y_TOGGLEBTN, x + 8, Y_TOGGLEBTN + 1, sta.selected and colours.red or colours.green)
    term.setCursorPos(x + 3, Y_TOGGLEBTN)
    term.blit(string.format('%3d', sta.currentAlti), '111', sta.selected and 'eee' or 'ddd')
    term.setCursorPos(x + 2, Y_TOGGLEBTN + 1)
    term.blit('ALT '..index, '11111', sta.selected and 'eeeee' or 'ddddd')
end

local function repaintButtons()
    for i = 1, PANELS do
        guiStatus[i].selected = (i == altSelection)
        updateGUI(i)
    end
end

-- Mode transitions (see the FMS OVERRIDE note above).
local function toOver()          -- FMS took the vertical channel
    if mode == 'OVER' then return end
    savedSelection = altSelection     -- stash the current pick (pilot's, or a PILOT pick)
    altSelection = nil
    mode = 'OVER'
    repaintButtons()                  -- all green
end

local function toPilot()         -- pilot has ALT-AP during the landing; keep the pick
    if mode == 'PILOT' then return end
    mode = 'PILOT'
    repaintButtons()
    broadcastAlti()
end

local function toNormal(keep)    -- landing over: keep the pilot's pick, or restore stash
    if not keep then altSelection = savedSelection end
    savedSelection = nil
    mode = 'NORMAL'
    repaintButtons()
    broadcastAlti()
end

local function toOff()           -- AP OFF: fully disengaged, nothing selected, silent
    altSelection = nil
    savedSelection = nil
    mode = 'NORMAL'
    repaintButtons()             -- all green; the pilot re-arms ALT-AP by tapping a panel
end

-- Listen to the FMS's autoland stream and mirror it into the mode above.
local function autolandLoop()
    while true do
        local sender, msg = rednet.receive(AUTOLAND_PROTOCOL, 0.5)
        if sender and type(msg) == 'table' and msg.cmd then
            local c = msg.cmd
            if c == 'hold' or c == 'descend' or c == 'landed' then
                lastAutolandActive = os.clock()
                if os.clock() >= suppressUntil then toOver() end   -- RESUME LANDING lands here too
            elseif c == 'release' then
                lastAutolandActive = os.clock()
                suppressUntil = 0
                toPilot()
            elseif c == 'cancel' then
                suppressUntil = 0
                toNormal(mode == 'PILOT')   -- keep the pilot's pick, else restore the stash
            elseif c == 'off' then
                suppressUntil = 0
                toOff()                     -- AP OFF: deselect and stay silent
            end
        end
        -- Safety net: the FMS streams autoland continuously while it owns the vertical, so
        -- a long silence in OVER means it is done (or gone) -- hand the panel back.
        if mode == 'OVER' and (os.clock() - lastAutolandActive) > AUTOLAND_STALE then
            toNormal(false)
        end
    end
end

local function updateInput()
    local w = term.getSize() -- 51 on an advanced computer/monitor

    while true do
        local _, button, x, y = os.pullEvent("mouse_click")

        -- panel: slice the screen width into PANELS equal columns
        local panel = math.floor((x - 1) * PANELS / w) + 1
        local px = (panel - 1) * PANEL_STRIDE + 3 -- that panel's left edge
        local lx = x - px                          -- x relative to the panel

        -- column 0..2 across the 9-wide keypad; -1 if the click misses it
        local column = -1
        if lx >= 0 and lx <= 8 then
            column = math.floor(lx / 3)
        end

        -- row 0..3 keypad rows, 4 = ALT button; -1 otherwise
        local row = -1
        if y >= Y_KEYPAD and y <= Y_KEYPAD + 11 then
            row = math.floor((y - Y_KEYPAD) / 3)
        elseif y >= Y_TOGGLEBTN and y <= Y_TOGGLEBTN + 1 then
            row = 4
        end

        if button == 1 and column ~= -1 and row ~= -1 then
            local sta = guiStatus[panel]

            if row == 4 then
                -- ALT button. In NORMAL/PILOT this is an ordinary selection. In OVER it is
                -- a pilot OVERRIDE: take ALT-AP back mid-landing -- select the panel, resume
                -- broadcasting, tell the FMS (which pauses the descent but keeps position/
                -- heading alignment), and suppress OVER re-entry briefly so the FMS's last
                -- in-flight descend msgs don't yank us back before it processes the override.
                altSelection = panel
                if mode == 'OVER' then
                    suppressUntil = os.clock() + 1.5
                    mode = 'PILOT'
                    rednet.broadcast({}, ALTOVERRIDE_PROTOCOL)
                end
                for i = 1, PANELS do
                    guiStatus[i].selected = (i == panel)
                    updateGUI(i)
                end
                broadcastAlti()

            elseif row <= 2 then
                -- digits 1..9
                local digit = row * 3 + column + 1
                sta.inputBuffer = (sta.inputBuffer .. digit):sub(-3)
                updateGUI(panel)

            else -- row == 3: Ent | 0 | Clr
                if column == 0 then          -- Enter
                    if sta.inputBuffer ~= '' then
                        sta.currentAlti = tostring(tonumber(sta.inputBuffer))
                        sta.inputBuffer = ''
                        updateGUI(panel)
                        if panel == altSelection then broadcastAlti() end
                    end
                elseif column == 1 then      -- 0
                    sta.inputBuffer = (sta.inputBuffer .. '0'):sub(-3)
                    updateGUI(panel)
                else                          -- Clr
                    sta.inputBuffer = ''
                    updateGUI(panel)
                end
            end
        end
    end
end

-- ============================================================================
-- EXTERNAL MONITOR -- PRIMARY FLIGHT DISPLAY
--
-- An artificial horizon with a pitch ladder, a bank scale and a flight path marker, between
-- a ground speed tape on the left and an altitude tape and vertical speed scale on the right,
-- carrying the altitude this panel has selected as a figure and a bug. The craft state is
-- read straight off sublevel (this computer is on the aircraft). Display-only; the keypad on
-- the computer's own screen (mouse_click) is untouched, and if no monitor is attached the
-- loop just idles.
--
-- DRAWN AS A PSEUDO-BITMAP, the same way autopilot.lua's terrain map is (TERR.SX / TERR.SW
-- there, SX / SW here). CC's font carries 32 "teletext" glyphs, 128..159, that divide one
-- character cell into a 2-wide by 3-tall grid of subpixels: character 128+n lights the
-- subpixels whose bits are set in n in the FOREGROUND colour, over a background of all the
-- rest. Bit order, with the values that address them:
--
--      +-----+
--      | 1 2 |   bits 1 and 2
--      | 3 4 |   bits 4 and 8
--      | 5 6 |   bits 16 and 32
--      +-----+
--
-- Only 31 masks are drawable -- the bottom-right subpixel is always background -- so a mask
-- with that pixel set is drawn as its COMPLEMENT with the two colours SWAPPED, which is what
-- SW records; the empty and the full mask are a plain space. A cell still holds only TWO
-- colours. What the trick buys is three times the vertical and twice the horizontal
-- resolution within them, and that is the whole reason this display can carry degree marks at
-- all: on a monitor twenty-odd characters tall a five-degree ladder puts consecutive rungs
-- less than a character apart, which quantises into a smear that reads as a second horizon --
-- but a subpixel and a half apart, which reads as a ladder.
--
-- WHAT IS ON IT
--   horizon + ladder    one rigid frame, rotated about the boresight by the bank angle and
--                       slid along its own perpendicular by the pitch -- which is what keeps
--                       every rung parallel to the horizon at any bank. Rungs every
--                       HZN_LADDER degrees, solid above the horizon and DASHED below (the
--                       standard reading, and the only cue that says which side of the
--                       horizon a rung belongs to once the horizon itself has left the
--                       screen); every second rung is longer and carries its angle in figures.
--   bank scale          a FIXED arc of ticks about the boresight with a moving pointer, so
--                       bank reads the same in a steep climb or dive when the horizon has
--                       slid off the screen entirely. It replaced the black bezel this
--                       display used to carry round all four edges for the same purpose,
--                       which cost two columns each side and could say nothing about the
--                       bank but where one character sat.
--   flight path marker  where the aircraft is actually GOING, as against where it is
--                       pointing. On a VTOL those two part company by ninety degrees
--                       routinely (it holds a heading and translates sideways), so the marker
--                       sitting well off to one side is the aircraft telling the truth, not
--                       the display drifting. Caged to the edge, and amber, when it would
--                       fall off the screen.
--   the three tapes      ground speed scrolling on the left, altitude on the right and
--                       vertical speed beside it, each with its value in a box that does not
--                       move -- so the FIGURE is read in one fixed place and the RATE is read
--                       off how fast the scale behind it is going by. The altitude tape also
--                       carries the ALT-AP selection twice over, as a figure at its head and
--                       as a magenta bug on the scale, which is the one thing the old corner
--                       readouts could not show: how far away the selected altitude is, and
--                       whether the aeroplane is closing on it. See THE TAPES below.
--
-- PITCH AND BANK COME OFF THE UP-VECTOR, not off Euler angles: world "up" expressed in the
-- body frame is two degrees of freedom, and asin/atan2 of it recover pitch and bank exactly
-- and reversibly. The +-90 clamp on asin is not a loss here -- inverted flight lands in the
-- BANK term, past +-90, and the whole frame rotates with it, which is what a real ADI does.
-- The only degenerate point is the zenith, where bank stops meaning anything at all.
--
-- HOW IT IS PAINTED. One blit per row, and a row is built out of a handful of native string
-- operations rather than a loop over its cells: the horizon is a run of sky, a short band of
-- mixed cells and a run of ground, and everything drawn on top of it is a patch on a named
-- column. Rows that come out identical to the frame before are not blitted at all. That
-- matters because a CC computer's Lua shares one thread with every other computer on the
-- server and this redraws at 20 Hz. Benched off the game on Lua 5.4, 200 frames of a rolling
-- and pitching aircraft, milliseconds per frame:
--
--       monitor   row-at-a-time (what this replaced)   this   cell-at-a-time subpixel
--        29x12                0.07                     0.16           0.20
--        51x26                0.24                     0.40           0.52
--        81x41                0.58                     0.77           1.08
--       121x61                1.16                     1.40           2.12
--
-- so three times the resolution and everything listed above costs well under twice the old
-- display, and the gap CLOSES as the monitor grows -- where drawing the same picture a cell
-- at a time costs half as much again and grows faster than the display does. Cobalt is a good
-- deal slower than 5.4, so take the ratios as the finding and the absolute figures as a floor.
--
-- THE TAPES cost a third more again, benched the same way on 2026-09-10 (a different machine,
-- so these four numbers are comparable with each other and NOT with the four above):
--
--       monitor   horizon alone   with the three tapes
--        29x12        0.109              0.114        (too narrow: no tapes drawn)
--        51x26        0.237              0.291
--        81x41        0.359              0.428
--       121x61        0.521              0.614
--
-- (measured 2026-09-10 at SPD_STEP/ALT_STEP = 10 and TAPE_GAP = 3. What a tape costs tracks
-- the number of FIGURES on it, and those two knobs set that between them: the same tapes at
-- a one-row gap, which is twice the figures, measured 0.33 / 0.48 / 0.69.)
--
-- and it is a third rather than the two-thirds it first measured because a tape BODY is a run
-- exactly as the sky and the ground are (AI.layout builds the two of them as strings, AI.row
-- concatenates them, and the columns behind a tape are never masked or classified at all).
-- Filling those cells one at a time instead -- which is the obvious way to write it, and was
-- the first way -- cost 0.85 ms at 121x61 for a picture pixel for pixel identical.
-- ============================================================================
local mon = peripheral.find("monitor")

-- colour -> blit char. The digits are ordered by BIT, so "0" is white and black is "f" --
-- the mapping is a trap, and this is the one place it is written down.
local BLIT = {}
for i = 0, 15 do BLIT[2 ^ i] = ("0123456789abcdef"):sub(i + 1, i + 1) end

-- The subpixel glyphs (see the header): SX[m] draws the 6-bit mask m, SW[m] says whether the
-- caller must hand blit its two colours the other way round.
local SX, SW = {}, {}
for m = 0, 63 do
    if m == 0 then SX[m], SW[m] = " ", false
    elseif m == 63 then SX[m], SW[m] = " ", true
    elseif m < 32 then SX[m], SW[m] = string.char(128 + m), false
    else SX[m], SW[m] = string.char(128 + (63 - m)), true end
end

-- Population count, and "set subpixel k (1..6) of mask m". Cobalt has no bitwise operators
-- and both of these sit on the hot path, so both are tables rather than arithmetic.
local POP, SETB = {}, {}
for m = 0, 63 do
    local n, v = 0, m
    for _ = 1, 6 do n = n + v % 2 v = math.floor(v / 2) end
    POP[m] = n
    SETB[m] = {}
    for k = 1, 6 do
        local b = 2 ^ (k - 1)
        SETB[m][k] = (math.floor(m / b) % 2 == 1) and m or (m + b)
    end
end

-- Rotate vector (x,y,z) by the UNIT quaternion q, PRESERVING its magnitude (q*vector
-- normalises the intermediate product and silently discards it -- see uac.lua's rotateVec,
-- copied here byte-for-byte).
local function rotateVec(q, x, y, z)
    local ux, uy, uz, a = q.v.x, q.v.y, q.v.z, q.a
    local tx = 2 * (uy*z - uz*y)
    local ty = 2 * (uz*x - ux*z)
    local tz = 2 * (ux*y - uy*x)
    return x + a*tx + (uy*tz - uz*ty),
           y + a*ty + (uz*tx - ux*tz),
           z + a*tz + (ux*ty - uy*tx)
end

-- One switch each, same pattern as every other sign in this project: flip if the display
-- moves the wrong way in-game. The first two are the signs the row-at-a-time horizon this
-- replaced was flown with, and they mean the same thing here -- PITCH_SIGN makes nose-up a
-- positive pitch, ROLL_SIGN makes a right bank tilt the horizon the way it does out of the
-- window and sends the bank pointer to the right.
local HZN_PITCH_SIGN = -1
local HZN_ROLL_SIGN  = 1
local HZN_DRIFT_SIGN = 1   -- flight path marker must slide toward the wing it is drifting over

-- THE ALTITUDE DATUM -- how far the pose `sublevel` reports sits ABOVE THE BOTTOM OF THE
-- HULL, in metres, subtracted from the altitude this PFD prints. MUST MATCH this airframe's
-- pid/uacconf.lua, apconf.lua, gpwsconf.lua and responder.lua: the number on this screen is
-- what the pilot flies the aeroplane on, and the UAC holds the ALT-AP pick made from it, so
-- the two have to be the same datum or the selected altitude is not the one flown.
-- Every altitude the aeroplane is measured against is the world Y of a piece of GROUND
-- (landing.db's pad and runway elevations, the terrain survey), so this reads the world Y of
-- the part of the aeroplane that touches it. Measure by parking on ground of a known Y with
-- this at 0 and taking (what it read) - (the Y the hull rests on); it is a build dimension.
local ALT_OFFSET = 9   -- measured 2026-08-29: on the Y=63 airstrip it read 73

-- Vertical field of view, top to bottom of the monitor, in degrees. This is the ONE number
-- that sets the pitch scale: the ladder is LINEAR in degrees, as a real ADI's is, so a rung
-- sits (3*H / HZN_FOV) subpixel rows per degree from the horizon. 80 keeps a five-degree
-- ladder about a character and a half apart on the monitors in this fleet, which is the
-- closest spacing that still reads as separate rungs.
--   (The row-at-a-time display this replaced projected the horizon through a pinhole,
--   f*tan(pitch), with f = H/1.6 rows. The two agree to within a rung out to about 25
--   degrees and only part company where tan starts to run away.)
local HZN_FOV    = 80
-- Degrees between rungs, coarsest first: the display takes the FINEST of these whose rungs
-- still land HZN_SEP subpixels apart, and draws no ladder at all if even the coarsest does
-- not. A horizon and a bank scale with no ladder is a poorer instrument; a horizon and a bank
-- scale with a ladder whose rungs have quantised on top of each other is a worse one -- the
-- smear reads as a second horizon. Every second rung of whichever step wins is a labelled
-- major, so the figures stay ten or twenty degrees apart either way.
local HZN_LADDER = { 20, 10, 5 }
local HZN_SEP    = 3
local HZN_FPV_GS = 5      -- m/s of speed below which the flight path marker is meaningless

-- ============================================================================
-- THE TAPES -- speed down the left edge, altitude and vertical speed down the right.
--
-- A tape is the pitch ladder's idea again: a scale that SCROLLS past a fixed readout, so the
-- number is always read in one place and the RATE is read off the movement. The readout box
-- IS the pointer -- it never moves and its digits are the live value -- so nothing here has
-- to interpolate a number into a character row, and the scale may quantise as coarsely as
-- the monitor forces without the figure the pilot flies on ever being wrong.
--
-- The tapes are drawn LAST and are OPAQUE: the horizon, the ladder, the bank scale and the
-- flight path marker all pass behind them. What they cost is width, so the attitude picture
-- is re-centred on what is LEFT of the monitor (AI.fc0/AI.fc1, and AI.hw where the ladder
-- and the boresight used to read AI.W) rather than on the monitor itself -- and on a display
-- too narrow to give up twelve columns and still hold an attitude picture worth looking at,
-- the tapes are not drawn at all and the corner readouts they replaced come back.
--
-- SPEED IS GROUND SPEED, and the tape says so at its foot. There is no air in this game and
-- so no airspeed: the FMS's speed law, the responder's contact reports and this tape are all
-- the HORIZONTAL component of the velocity sublevel reports, and the vertical component is
-- the VS scale's business alone -- which is also why the two never disagree here the way an
-- ASI and a groundspeed readout do in an aeroplane.
-- ============================================================================
-- The GRANULARITY of a tape: the units between its FIGURES, and the only knob either tape
-- has. The scale FOLLOWS from it, rather than the other way round -- the figures are put
-- TAPE_GAP+1 cells apart by construction, so a tape covers (H * step / (TAPE_GAP + 1)) units
-- top to bottom, and a taller monitor buys MORE SCALE at the same readability instead of the
-- same scale in steps too fine to read.
--
-- It was the span that was the knob until 2026-09-10, with the step derived from it by
-- AI.tapeStep. That had the granularity depend on the monitor -- one 160 m/s span figured
-- every 20 on a 26-row display and every 50 on a 14-row one -- which is the wrong way round:
-- how finely a figure can be read is a property of the reader, not of the aeroplane.
--
-- Both are FLEET-WIDE, where SPD_SPAN was per airframe, for the same reason: ten units
-- between figures reads the same on the Citation as on Scout, and that Scout's tape then
-- scrolls ten times faster at cruise is the tape telling the truth about the difference.
local SPD_STEP = 10       -- m/s between figures on the speed tape
local ALT_STEP = 10       -- m between figures on the altitude tape
-- Blank rows BETWEEN a tape's figures. They are placed TAPE_GAP+1 cells apart, so this and
-- the step together fix the scale outright:
--
--     k = 3 * (TAPE_GAP + 1) / step        subpixel rows per unit
--     the tape covers H * step / (TAPE_GAP + 1) units, top to bottom
--
-- which means widening the gap SPENDS SCALE to buy air around the figures -- at a step of 10
-- on a 26-row monitor, one row of gap shows 130 units of the world and three rows show 65.
-- One row is as tight as the glyphs allow and reads as a wall of numbers; three is
-- comfortable on the monitors in this fleet. Minor ticks sit halfway between the figures
-- whatever the gap, so they follow it and need no knob of their own.
local TAPE_GAP = 3        -- rows between figures on both scrolling tapes
-- Full deflection of the VS scale: the aircraft's own vertical envelope with a little over.
-- uacconf's CFG.CLIMB_MAX is 3.75 here and CFG.MANCON_CLIMB_MAX (the boost key) reaches 7.5,
-- so 8 puts the boost at the end of the scale and everything the autopilot commands inside
-- half of it.
local VS_FULL  = 8        -- m/s at the ends of the scale

if mon then
    mon.setTextScale(0.5)
    pcall(function()
        mon.setPaletteColour(colours.blue,   0x2277cc)   -- sky
        mon.setPaletteColour(colours.brown,  0x8a5a2b)   -- ground
        mon.setPaletteColour(colours.orange, 0xffbb00)   -- boresight, bank caution
        mon.setPaletteColour(colours.lime,   0x33dd66)   -- flight path marker
        mon.setPaletteColour(colours.grey,   0x333333)   -- the tapes' own ground
    end)
end

-- ---------------------------------------------------------------------------
-- The display itself. Everything hangs off one table rather than off top-level locals, the
-- same reason autopilot.lua's TERR does: the main chunk's local count is charged against
-- every function nested in it, and Cobalt caps that at 200.
-- ---------------------------------------------------------------------------
local AI = {
    W = 0, H = 0,
    prev = {},                       -- last row blitted, so a still display costs no blits
    ink = {}, inkc = {},             -- subpixel overlay: 6-bit mask and colour, per cell
    inkb = {},                       -- ...and a per-cell background, for the readout boxes
    colbg = {},                      -- which columns a tape owns, and what it paints them
    lrc = "", lrb = "", rrc = "", rrb = "",   -- the tape bodies, prebuilt as run strings
    ontape = false,                  -- ...drawing INTO a tape rather than behind one
    tch = {}, tfg = {}, tbg = {},    -- character overlay (labels, readouts), per cell
    pr = {},                         -- pr[row] = the columns of that row carrying an overlay
    mark = {}, frame = 0,            -- which frame last touched a cell, so pr gets no dupes
    SC = {}, SF = {}, SB = {},       -- row segments, reused rather than reallocated per frame
    OC = {}, OF = {}, OB = {},       -- ...and the same again for the patched row
    ox = 0, oy = 0, k = 1,           -- boresight in subpixel coords, subpixel rows per degree
    R = 0, rx = 0, ry = 0,           -- the bank scale's radius, and the attitude window
    -- The tapes eat the outer columns, so the attitude picture gets fc0..fc1 and hw is ITS
    -- half-width in subpixels -- which, as everywhere in this file, is its width in cells.
    tapes = false, fc0 = 1, fc1 = 1, hw = 0, lay = "",
    SPD_W = 5, ALT_W = 4, VS_W = 3,  -- tape widths in cells: figures + one column of ticks
    TAPE_FREE = 20,                  -- ...and the cells of attitude they must leave behind
    TAPE_MIN_H = 10,
    tapebg = nil,                    -- (filled in below, once BLIT exists)
    -- Figure steps a tape may use, FINEST FIRST: AI.tapeStep takes the first that fits, which
    -- is HZN_LADDER's rule with the list the other way up.
    NUM = { 1, 2, 5, 10, 20, 50, 100, 200, 500, 1000, 2000, 5000 },
    nx = 0, ny = 1, B = 0,           -- the horizon half-plane: sky is g(i,j) = nx*i+ny*j+B < 0
    pitch = 0, bank = 0, q = nil, v = nil,
    sky = BLIT[colours.blue], gnd = BLIT[colours.brown],
    -- The bank scale, as degree and tick-length pairs. Hoisted because rebuilding it inside
    -- the frame would allocate a dozen tables twenty times a second.
    -- degree, tick length, and the smallest arc radius that has room for it: ten degrees of a
    -- radius under about eighteen subpixels is less than three subpixels of arc, and the fine
    -- ticks close up into a band.
    BT = { {-60,4,0}, {-45,4,0}, {-30,3,0}, {-20,2,18}, {-10,2,18},
           {10,2,18}, {20,2,18}, {30,3,0}, {45,4,0}, {60,4,0} },
    -- ...and the flight path marker's ring, for the same reason.
    RING = { {2,0}, {-2,0}, {0,2}, {0,-2}, {1,1}, {1,-1}, {-1,1}, {-1,-1} },
}
AI.tapebg = BLIT[colours.grey]

function AI.size(w, h)
    if AI.W == w and AI.H == h then return end
    AI.W, AI.H = w, h
    AI.prev, AI.ink, AI.inkc, AI.inkb, AI.tch, AI.tfg, AI.tbg, AI.mark, AI.pr =
        {}, {}, {}, {}, {}, {}, {}, {}, {}
    for r = 1, h do AI.pr[r] = {} end
end

-- The tape columns, worked out once per layout rather than once per frame. A tape body is a
-- RUN of its own ground exactly as the sky and the ground are -- the whole point of AI.row --
-- so the two of them are built here as strings and concatenated onto every row, and a tape
-- cell costs nothing at all unless something is actually drawn in it.
function AI.layout()
    local key = AI.W .. "x" .. AI.H .. (AI.tapes and "t" or "-")
    if AI.lay == key then return end
    AI.lay = key
    for c = 1, AI.W do AI.colbg[c] = nil end
    if AI.tapes then
        for c = 1, AI.fc0 - 1 do AI.colbg[c] = AI.tapebg end
        for c = AI.fc1 + 1, AI.W do AI.colbg[c] = AI.tapebg end
    end
    AI.lrc, AI.lrb = string.rep(" ", AI.fc0 - 1), string.rep(AI.tapebg, AI.fc0 - 1)
    AI.rrc, AI.rrb = string.rep(" ", AI.W - AI.fc1), string.rep(AI.tapebg, AI.W - AI.fc1)
end

-- Mark cell (c,r) as carrying an overlay this frame and return its index. The row builder
-- walks pr, so a cell has to land in it exactly once however many times it is drawn on.
function AI.touch(c, r)
    local idx = (r - 1) * AI.W + c
    if AI.mark[idx] ~= AI.frame then
        AI.mark[idx] = AI.frame
        AI.ink[idx], AI.inkc[idx], AI.tch[idx], AI.inkb[idx] = 0, nil, nil, nil
        local l = AI.pr[r]
        l[#l + 1] = c
    end
    return idx
end

-- One SUBPIXEL of ink. sx/sy are 1-based subpixel coordinates: 2*W across, 3*H down.
-- A cell holds ONE ink colour, so where two overlays cross the later one takes the cell --
-- keeping the bits it inherits, so the crossing does not punch a hole in what is underneath.
-- Draw order is therefore importance order: horizon, ladder, bank scale, boresight, marker.
function AI.plot(sx, sy, col)
    sx, sy = math.floor(sx), math.floor(sy)
    if sx < 1 or sy < 1 or sx > 2 * AI.W or sy > 3 * AI.H then return end
    local c = math.floor((sx - 1) / 2) + 1
    -- Behind an opaque tape, so it is not drawn rather than drawn and covered: the horizon
    -- runs the full width and the flight path marker's wings reach six subpixels either side.
    if AI.colbg[c] and not AI.ontape then return end
    local idx = AI.touch(c, math.floor((sy - 1) / 3) + 1)
    AI.inkc[idx] = col
    AI.ink[idx] = SETB[AI.ink[idx]][((sy - 1) % 3) * 2 + ((sx - 1) % 2) + 1]
end

-- DDA over the longer axis, so a near-vertical horizon (which is what eighty degrees of bank
-- looks like) rasterises as evenly as a level one. `dash` draws that many subpixels on and
-- the same off, which is how a nose-DOWN rung says which side of the horizon it is on.
-- `clip` holds the line inside the attitude WINDOW (AI.inWin), which is what keeps the pitch
-- ladder from running out through the bank scale on the rim. The window has to be a shape
-- about the boresight rather than a row span, because at ninety degrees of bank the rungs are
-- vertical and it is the SIDES of the display they would run out through.
function AI.line(x0, y0, x1, y1, col, dash, clip)
    local dx, dy = x1 - x0, y1 - y0
    -- CEIL, not floor: floor makes the step marginally longer than a subpixel, and every so
    -- often two consecutive steps land two subpixels apart -- which reads as a horizon with
    -- holes punched along it.
    local n = math.ceil(math.max(math.abs(dx), math.abs(dy)))
    if n < 1 then AI.plot(x0, y0, col) return end
    n = math.min(n, 4 * (AI.W + AI.H))          -- a slope near infinity must not become a loop
    for i = 0, n do
        if not dash or math.floor(i / dash) % 2 == 0 then
            local x, y = x0 + dx * i / n + 0.5, y0 + dy * i / n + 0.5
            if not clip or AI.inWin(x, y) then AI.plot(x, y, col) end
        end
    end
end

-- The attitude WINDOW: an ellipse about the boresight, as tall as the bank scale's arc will
-- allow and as wide as the monitor will. It is an ellipse and not the arc's own circle
-- because the monitors in this fleet are a good deal wider than they are tall, and a circle
-- inscribed in one throws away exactly the width the ladder wants for its figures.
function AI.inWin(x, y)
    local dx, dy = (x - AI.ox) / AI.rx, (y - AI.oy) / AI.ry
    return dx * dx + dy * dy < 1
end

-- Characters go on top of the ink: a rung label has to be readable, and it is written where
-- the rung it belongs to has already been drawn. bg nil means "whatever was already here" --
-- the tape's own ground inside a tape, and the horizon outside one, which is what stops a
-- label sitting in the sky from carrying a black box around.
function AI.text(c, r, s, fg, bg)
    if r < 1 or r > AI.H then return end
    for i = 1, #s do
        local x = c + i - 1
        if x >= 1 and x <= AI.W and (AI.ontape or not AI.colbg[x]) then
            local idx = AI.touch(x, r)
            AI.tch[idx], AI.tfg[idx], AI.tbg[idx] = s:sub(i, i), BLIT[fg], bg and BLIT[bg]
        end
    end
end

-- The sky/ground mask of one cell -- bit set means that subpixel is ABOVE the horizon. This
-- is the whole horizon rendering, at three times the vertical resolution of a row-at-a-time
-- fill, and it costs six compares because g() is linear and its row terms are walked, not
-- recomputed.
function AI.mask(c, r)
    local aL = AI.nx * (2 * c - 1) + AI.B
    local aR = aL + AI.nx
    local y, m = AI.ny * (3 * r - 2), 0
    if aL + y < 0 then m = m + 1 end
    if aR + y < 0 then m = m + 2 end
    y = y + AI.ny
    if aL + y < 0 then m = m + 4 end
    if aR + y < 0 then m = m + 8 end
    y = y + AI.ny
    if aL + y < 0 then m = m + 16 end
    if aR + y < 0 then m = m + 32 end
    return m
end

-- A point on the ladder, in subpixel coordinates.
--   theta -- the rung's angle in degrees, 0 being the horizon
--   s     -- how far ALONG that rung, in the same degrees
function AI.lp(theta, s)
    local d = (AI.pitch - theta) * AI.k     -- perpendicular, + = below the boresight
    local u = s * AI.k                      -- along the horizon, + = to the right
    return AI.ox + d * AI.nx + u * AI.ny,
           AI.oy + d * AI.ny - u * AI.nx
end

-- One overlaid cell, resolved against whatever is underneath it -- a TAPE's own ground if the
-- cell is inside one, and the horizon if it is not.
function AI.cell(idx, c, r)
    local t, m, col = AI.tch[idx], AI.ink[idx], AI.inkc[idx]
    local bg = AI.inkb[idx] or AI.colbg[c]
    if bg then
        -- Inside a tape. The horizon is not visible here at all, so it is not even sampled:
        -- twelve columns of mask() a row, twenty times a second, for a picture the tape is
        -- about to cover.
        if t then return t, AI.tfg[idx], AI.tbg[idx] or bg end
        if m == 0 or not col then return " ", bg, bg end
        col = BLIT[col]
        if SW[m] then return SX[m], bg, col end
        return SX[m], col, bg
    end
    local fm = AI.mask(c, r)
    -- What the horizon painted here, quantised to the one colour a two-colour cell can spare
    -- for a background. Six subpixels of terrain get this same treatment in autopilot.lua and
    -- for the same reason: the overlay is the thing that has to be legible.
    local dom = (POP[fm] >= 3) and AI.sky or AI.gnd
    if t then return t, AI.tfg[idx], AI.tbg[idx] or dom end
    if m == 0 or not col then
        if SW[fm] then return SX[fm], AI.gnd, AI.sky end
        return SX[fm], AI.sky, AI.gnd
    end
    col = BLIT[col]
    if SW[m] then return SX[m], dom, col end
    return SX[m], col, dom
end

-- Build one row and blit it. The horizon is a run of one colour, a short band of cells the
-- boundary actually crosses, and a run of the other -- so the row costs a couple of
-- string.reps and one concat, however tall the monitor is. Only the cells in the band and the
-- cells carrying an overlay are ever looked at one at a time.
function AI.row(r)
    local W, SC, SF, SB, ns = AI.W, AI.SC, AI.SF, AI.SB, 0
    -- The horizon is only built where it can be SEEN: fc0..fc1, with a tape body run dropped
    -- either side of it. A tape hides the horizon completely, so those columns are never
    -- masked, never classified and never looked at one at a time.
    local f0, f1 = AI.fc0, AI.fc1
    if AI.tapes then ns = 1 SC[1], SF[1], SB[1] = AI.lrc, AI.lrb, AI.lrb end
    -- Which cells of this row can straddle the horizon: solve g(i, y) = 0 for the row's top
    -- and bottom subrows, then widen by a cell each way so no rounding can lose a boundary.
    local m0, m1 = 1, 0
    if AI.nx > 1e-6 or AI.nx < -1e-6 then
        local a = (-AI.ny * (3 * r - 2) - AI.B) / AI.nx
        local b = (-AI.ny * (3 * r) - AI.B) / AI.nx
        if a > b then a, b = b, a end
        m0 = math.max(f0, math.floor(a / 2) - 1)
        m1 = math.min(f1, math.floor(b / 2) + 2)
        if m0 > m1 then m0, m1 = 1, 0 end       -- band off the screen: the row is one colour
    end
    -- ...and with the horizon exactly level on the screen (nx == 0) every cell of the row is
    -- identical -- which is not always all-sky or all-ground, it can be a partial mask. That
    -- is why the runs below repeat a MASK rather than a colour.
    local lend   = (m1 >= m0) and (m0 - 1) or f1
    local rstart = (m1 >= m0) and (m1 + 1) or (f1 + 1)

    if lend >= f0 then
        local m, n = AI.mask(f0, r), lend - f0 + 1
        ns = ns + 1
        SC[ns] = string.rep(SX[m], n)
        SF[ns] = string.rep(SW[m] and AI.gnd or AI.sky, n)
        SB[ns] = string.rep(SW[m] and AI.sky or AI.gnd, n)
    end
    for c = m0, m1 do
        local m = AI.mask(c, r)
        ns = ns + 1
        SC[ns] = SX[m]
        SF[ns] = SW[m] and AI.gnd or AI.sky
        SB[ns] = SW[m] and AI.sky or AI.gnd
    end
    if rstart <= f1 then
        local m, n = AI.mask(f1, r), f1 - rstart + 1
        ns = ns + 1
        SC[ns] = string.rep(SX[m], n)
        SF[ns] = string.rep(SW[m] and AI.gnd or AI.sky, n)
        SB[ns] = string.rep(SW[m] and AI.sky or AI.gnd, n)
    end
    if AI.tapes then ns = ns + 1 SC[ns], SF[ns], SB[ns] = AI.rrc, AI.rrb, AI.rrb end

    local bch = table.concat(SC, "", 1, ns)
    local bfg = table.concat(SF, "", 1, ns)
    local bbg = table.concat(SB, "", 1, ns)

    local pl = AI.pr[r]
    if #pl > 0 then
        table.sort(pl)
        local OC, OF, OB, no, prev = AI.OC, AI.OF, AI.OB, 0, 0
        for i = 1, #pl do
            local c = pl[i]
            if c > prev then                     -- a column drawn on twice is one patch
                if c > prev + 1 then
                    no = no + 1
                    OC[no] = bch:sub(prev + 1, c - 1)
                    OF[no] = bfg:sub(prev + 1, c - 1)
                    OB[no] = bbg:sub(prev + 1, c - 1)
                end
                no = no + 1
                OC[no], OF[no], OB[no] = AI.cell((r - 1) * W + c, c, r)
                prev = c
            end
        end
        if prev < W then
            no = no + 1
            OC[no], OF[no], OB[no] = bch:sub(prev + 1), bfg:sub(prev + 1), bbg:sub(prev + 1)
        end
        bch = table.concat(OC, "", 1, no)
        bfg = table.concat(OF, "", 1, no)
        bbg = table.concat(OB, "", 1, no)
    end

    local line = bch .. "\1" .. bfg .. "\1" .. bbg
    if AI.prev[r] ~= line then
        AI.prev[r] = line
        mon.setCursorPos(1, r)
        mon.blit(bch, bfg, bbg)
    end
end

-- HORIZON and PITCH LADDER.
--
-- The horizon runs the full width -- it is the one line that must never look clipped -- and
-- `rad`, the distance to the furthest corner of the screen, is how far it is drawn. The
-- LADDER lives inside the attitude window instead, both so the rungs do not run out through
-- the bank scale and because a ladder with an edge to it is what says which part of the
-- picture is being read as attitude.
function AI.ladder(rad)
    local hw = AI.hw                                 -- half the ATTITUDE picture, in subpixels
    local gap, maj, mnr = 0.24 * hw, 0.22 * hw, 0.11 * hw
    local win = math.max(AI.rx, AI.ry)
    local x0, y0 = AI.lp(0, -rad / AI.k)
    local x1, y1 = AI.lp(0,  rad / AI.k)
    AI.line(x0, y0, x1, y1, colours.white)
    -- The finest ladder this monitor can hold apart, or none at all -- see HZN_LADDER.
    local step = nil
    for i = 1, #HZN_LADDER do
        if HZN_LADDER[i] * AI.k >= HZN_SEP then step = HZN_LADDER[i] end
    end
    if not step then return end
    -- A rung whose perpendicular distance from the boresight is past the window cannot enter
    -- it at any bank, so there is no point rasterising it at all.
    local reach = math.min(90, math.ceil((math.abs(AI.pitch) + win / AI.k) / step) * step)
    for theta = -reach, reach, step do
        local d = (AI.pitch - theta) * AI.k
        if theta ~= 0 and d > -win and d < win then
            local major = theta % (2 * step) == 0
            local len   = major and maj or mnr
            local dash  = (theta < 0) and 2 or nil
            for si = 1, 2 do
                local sg = si * 2 - 3                 -- -1 then +1: the two ends of the rung
                local ax, ay = AI.lp(theta, sg * gap / AI.k)
                local bx, by = AI.lp(theta, sg * (gap + len) / AI.k)
                AI.line(ax, ay, bx, by, colours.white, dash, true)
                if major then
                    local s = tostring(math.abs(theta))
                    local lx, ly = AI.lp(theta, sg * (gap + len + 0.04 * hw) / AI.k)
                    if AI.inWin(lx, ly) then
                        local cc = math.floor((lx - 1) / 2) + 1
                        AI.text(sg < 0 and cc - #s + 1 or cc,
                                math.floor((ly - 1) / 3) + 1, s, colours.white)
                    end
                end
            end
        end
    end
end

-- BANK SCALE -- a fixed arc of ticks about the boresight, with a pointer that reads bank
-- ALONE. Pitch never moves it, so bank stays readable in a steep climb or dive when the
-- horizon has slid off the screen entirely. Zero is straight up and positive bank is to the
-- right, which is the convention the bezel this replaced used.
function AI.bankScale()
    local R = AI.R
    if R < 10 then return end                        -- too small a monitor to carry a scale
    for i = 1, #AI.BT do
        local d, L, rmin = AI.BT[i][1], AI.BT[i][2], AI.BT[i][3]
        if R >= rmin then
            local s, c = math.sin(math.rad(d)), math.cos(math.rad(d))
            AI.line(AI.ox + (R - L) * s, AI.oy - (R - L) * c,
                    AI.ox + R * s, AI.oy - R * c, colours.lightGrey)
        end
    end
    local bx = math.floor(AI.ox)                     -- the fixed zero index, outside the arc
    AI.plot(bx - 1, AI.oy - R - 2, colours.white)
    AI.plot(bx,     AI.oy - R - 2, colours.white)
    AI.plot(bx + 1, AI.oy - R - 2, colours.white)
    AI.plot(bx,     AI.oy - R - 1, colours.white)
    -- The pointer changes COLOUR rather than shape as the bank passes caution and warning,
    -- and parks at the end of the scale rather than wrapping round the back of it.
    local b = math.abs(AI.bank)
    local col = (b >= 45) and colours.red or (b >= 30) and colours.orange or colours.white
    local t = math.rad(AI.bank < -60 and -60 or (AI.bank > 60 and 60 or AI.bank))
    local s, c = math.sin(t), math.cos(t)
    local px, py = AI.ox + (R - 2) * s, AI.oy - (R - 2) * c        -- apex, pointing outward
    local qx, qy = AI.ox + (R - 6) * s, AI.oy - (R - 6) * c        -- centre of its base
    AI.line(px, py, qx + 2 * c, qy + 2 * s, col)
    AI.line(px, py, qx - 2 * c, qy - 2 * s, col)
    AI.line(qx - 2 * c, qy - 2 * s, qx + 2 * c, qy + 2 * s, col)
end

-- FLIGHT PATH MARKER. Placed on the ladder at (drift, flight path angle), so it reads
-- against the same degree marks everything else does. The drift is taken as a 2-D cross and
-- dot of the nose against the ground track, which means it needs no heading convention of
-- its own -- only which way round positive is, and that is HZN_DRIFT_SIGN.
function AI.fpv()
    local v = AI.v
    local gs = math.sqrt(v.x * v.x + v.z * v.z)
    local sp = math.sqrt(gs * gs + v.y * v.y)
    if sp < HZN_FPV_GS or gs < 1e-3 then return end
    local fx, _, fz = rotateVec(AI.q, 0, 0, HZN_PITCH_SIGN)          -- the nose, in the world
    local fl = math.sqrt(fx * fx + fz * fz)
    if fl < 1e-6 then return end                                     -- pointing at the zenith
    fx, fz = fx / fl, fz / fl
    local drift = HZN_DRIFT_SIGN *
        math.deg(math.atan2(fx * v.z - fz * v.x, fx * v.x + fz * v.z))
    local x, y = AI.lp(math.deg(math.asin(v.y / sp)), drift)
    -- Caged to the ATTITUDE picture, not to the monitor: a marker parked under a tape is a
    -- marker the pilot cannot see, and the tapes are drawn over it.
    local caged = false
    local xl, xr = 2 * AI.fc0 + 1, 2 * AI.fc1 - 2
    if x < xl then x, caged = xl, true elseif x > xr then x, caged = xr, true end
    if y < 3 then y, caged = 3, true elseif y > 3 * AI.H - 2 then y, caged = 3 * AI.H - 2, true end
    local col = caged and colours.orange or colours.lime
    x, y = math.floor(x), math.floor(y)
    for i = 1, #AI.RING do
        local o = AI.RING[i]
        AI.plot(x + o[1], y + o[2], col)                             -- the ring
    end
    if not caged then                                                -- ...and its wings and fin
        AI.line(x - 6, y, x - 3, y, col)
        AI.line(x + 3, y, x + 6, y, col)
        AI.line(x, y - 4, x, y - 3, col)
    end
end

-- A block of cells in a flat colour of its own, DISCARDING whatever was drawn there. The
-- tape BODIES are runs built into the row (AI.layout); this is for the boxes drawn on top of
-- one -- the readout, the selection, the VS figures -- which have to be opaque against the
-- tape's ticks the way the tape is opaque against the horizon.
function AI.fillBox(c0, c1, r0, r1, col)
    local b = BLIT[col]
    for r = math.max(1, r0), math.min(AI.H, r1) do
        for c = math.max(1, c0), math.min(AI.W, c1) do
            local idx = AI.touch(c, r)
            AI.ink[idx], AI.inkc[idx], AI.tch[idx], AI.inkb[idx] = 0, nil, nil, b
        end
    end
end

-- The tick step for a FIXED scale, given k subpixel rows per unit: the finest nice number
-- whose ticks still land `sep` cells apart, or nil if even the coarsest does not. The pitch
-- ladder's rule (HZN_LADDER / HZN_SEP) and the same reason -- marks that have quantised onto
-- one row read as one mark. Only the VS scale needs it: the two scrolling tapes are given
-- their step and derive their scale from it instead.
function AI.tapeStep(k, sep)
    for i = 1, #AI.NUM do
        if AI.NUM[i] * k >= 3 * sep then return AI.NUM[i] end
    end
end

-- ONE SCROLLING TAPE. c0..c1 are its cells; `val` sits at the boresight ROW and the scale
-- moves past it.
--   step  the units between FIGURES. The scale follows: they land TAPE_GAP+1 cells apart by
--         construction, so the tape covers H*step/(TAPE_GAP+1) units top to bottom
--   side  -1: ticks on the tape's RIGHT edge, figures to their left  (speed, screen left)
--         +1: ticks on the tape's LEFT edge,  figures to their right (altitude, screen right)
--   vmin  a value the scale stops at, or nil for a scale that runs both ways. Speed has a
--         floor and altitude does not: a tape marked -50 m/s says the aircraft can fly
--         backwards, while a world Y below zero is just the underside of the map.
--   bug   a value to mark on the scale, parked at the end of the tape when it is off scale,
--         or nil -- the ALT-AP selection
function AI.tape(c0, c1, val, step, side, vmin, bug, bugcol)
    local H3 = 3 * AI.H
    -- Subpixel rows per unit, BY CONSTRUCTION rather than by measurement: this one line is
    -- what makes the granularity independent of how tall the monitor is, and it is why there
    -- is no "no figures at all" case here the way there is in the pitch ladder.
    local k  = 3 * (TAPE_GAP + 1) / step
    AI.ontape = true                                 -- ...these columns are the tape's own
    -- Minor ticks halfway between the figures, so they widen with TAPE_GAP -- unless the gap
    -- is closed right up, where even those would land on top of each other and the figured
    -- ticks become the only ticks.
    local sub = (step / 2 * k >= 3) and (step / 2) or step
    -- Walked as an INTEGER count of minor ticks: stepping a float by step/2 and testing
    -- v % step picks up a rounding dust that turns majors into minors at random.
    local n0 = math.ceil((val - (H3 - AI.oy) / k) / sub)
    local n1 = math.floor((val + (AI.oy - 1) / k) / sub)
    if vmin then n0 = math.max(n0, math.ceil(vmin / sub)) end
    if n1 - n0 > H3 then n1 = n0 + H3 end            -- a nonsense step must not become a loop
    for n = n0, n1 do
        local v     = n * sub
        local major = (sub == step) or (n % 2 == 0)
        local y     = AI.oy + (val - v) * k
        -- Two subpixels for a figured tick and one for a minor, both INSIDE the tick column:
        -- a longer major would reach into the figure column, where the figure it belongs to
        -- is about to cover the half that made it a major.
        local len   = major and 2 or 1
        if side < 0 then AI.line(2 * c1 - len + 1, y, 2 * c1, y, colours.white)
        else             AI.line(2 * c0 - 1, y, 2 * c0 + len - 2, y, colours.white) end
        if major then
            local s = string.format("%d", v)
            AI.text(side < 0 and (c1 - #s) or (c0 + 1),
                    math.floor((y - 1) / 3) + 1, s, colours.white)
        end
    end
    -- The ALT-AP bug, in the TICK column: a bracket that shares its cell with whatever tick
    -- it lands on and takes the colour, because a bug and a tick are both marks on the scale
    -- and the bug is the one that matters. It cannot go in the figure columns -- a character
    -- beats ink in AI.cell, so the bug would vanish behind the very figure it is pointing at,
    -- which is exactly the figure it is most often next to.
    if bug then
        -- Parked a row in from each end when it is off scale, so it never disappears under
        -- the selection box at the head of the tape or the figures at its foot.
        local y  = AI.oy + (val - bug) * k
        if y < 5 then y = 5 elseif y > H3 - 4 then y = H3 - 4 end
        local x0 = (side < 0) and (2 * c1 - 1) or (2 * c0 - 1)
        -- The legs sit on the subpixel the ticks grow FROM -- the tape's inner edge, next
        -- to the attitude picture -- so the bracket opens away from the scale it marks.
        local xl = x0 + ((side < 0) and 1 or 0)
        AI.plot(x0, y, bugcol) AI.plot(x0 + 1, y, bugcol)
        AI.plot(xl, y - 1, bugcol) AI.plot(xl, y + 1, bugcol)
    end
    -- THE READOUT BOX, fixed at the boresight row and overhanging one cell INTO the attitude
    -- picture. It is the pointer as well as the number, which is why the tape behind it may
    -- be as coarse as the monitor forces: the value is never read off the scale.
    local br     = math.floor((AI.oy - 1) / 3) + 1
    local b0, b1 = (side < 0) and c0 or (c0 - 1), (side < 0) and (c1 + 1) or c1
    AI.fillBox(b0, b1, br, br, colours.black)
    local s = string.format("%.0f", val)
    AI.text((side < 0) and (b1 - #s + 1) or b0, br, s, colours.white, colours.black)
    AI.ontape = false
end

-- VERTICAL SPEED -- a fixed scale with a moving needle, not a scrolling tape. The quantity
-- being read IS a rate, so there is nothing for a tape to scroll: what matters is how far off
-- zero the needle is and which side of it, and both are read from the picture before the
-- digits are read at all. The digits sit above the scale in a climb and below it in a
-- descent, so the number is on the side the aeroplane is going.
function AI.vsTape(c0, c1, vs)
    -- Symmetric, and four subpixels short of the digit rows at each end so full deflection
    -- does not park the needle underneath its own readout.
    local kv = math.min(AI.oy - 4, 3 * AI.H - AI.oy - 4) / VS_FULL
    AI.ontape = true
    if kv > 0 then
        local sub = AI.tapeStep(kv, 1)               -- ticks, not figures: one cell apart will do
        if sub then
            for n = -math.floor(VS_FULL / sub), math.floor(VS_FULL / sub) do
                local len = (n % 2 == 0) and 4 or 2
                AI.line(2 * c0 - 1, AI.oy - n * sub * kv,
                        2 * c0 + len - 2, AI.oy - n * sub * kv, colours.lightGrey)
            end
        end
        local v = (vs > VS_FULL) and VS_FULL or ((vs < -VS_FULL) and -VS_FULL or vs)
        local col = (vs >= 0.1) and colours.lime
                 or (vs <= -0.1) and colours.orange or colours.white
        AI.line(2 * c0 - 1, AI.oy - v * kv, 2 * c1, AI.oy - v * kv, col)
        -- One decimal while it matters -- a hover is trimmed in tenths -- and whole metres
        -- per second once the rate is large enough that the tenth is noise. NO SIGN: the
        -- figure already sits ABOVE the scale in a climb and below it in a descent, and
        -- already carries the needle's colour, so a leading + or - would be the third time
        -- the display said the same thing -- and it would cost a column the digits need.
        -- The 9.95 rather than 10 is so a rate that rounds UP to ten metres never formats as
        -- four characters into three cells.
        local a = math.abs(vs)
        local s = (a < 9.95) and string.format("%3.1f", a) or string.format("%3.0f", a)
        local r = (vs >= 0) and 1 or AI.H
        AI.fillBox(c0, c1, r, r, colours.black)
        AI.text(c0, r, s, col, colours.black)
    end
    AI.ontape = false
end

-- What THIS panel is asking for, as a string, or nil. OVER is the FMS flying the vertical
-- channel: the panel is silent, so saying "SEL" would be a lie about who is holding the
-- altitude. In NORMAL and PILOT the same figure is also the altitude tape's bug.
function AI.sel()
    if mode == 'OVER' then return 'LAND' end
    if altSelection then return guiStatus[altSelection].currentAlti end
end

-- BORESIGHT and READOUTS. The boresight is the fixed aircraft reference everything else moves
-- around; drawn in subpixels it can be a wing one pixel thick instead of one character thick.
function AI.chrome(alt, gs, vs)
    local hw, W, H = AI.hw, AI.W, AI.H         -- the ATTITUDE picture's half-width, and cells
    local by, inw, out = math.floor(AI.oy), math.floor(0.06 * hw), math.floor(0.20 * hw)
    local bx = math.floor(AI.ox)
    AI.line(bx - out, by, bx - inw, by, colours.orange)
    AI.line(bx + inw, by, bx + out, by, colours.orange)
    for k = 1, 2 do
        AI.plot(bx - inw, by + k, colours.orange)
        AI.plot(bx + inw, by + k, colours.orange)
    end
    AI.plot(bx, by, colours.orange)

    -- The corner readouts are what the display carries when it is too narrow for the tapes;
    -- where there ARE tapes every one of these three numbers is in one of them, in a box that
    -- is also its pointer, and repeating them in the corners would only cost legibility.
    -- Black boxes, because these sit on top of whatever the horizon happens to be doing.
    if not AI.tapes then
        AI.text(2, H - 1, string.format("GS %3.0f", gs), colours.white, colours.black)
        AI.text(W - 7, H - 1, string.format("VS%+5.1f", vs),
            (vs >= 0.1) and colours.lime or (vs <= -0.1) and colours.orange or colours.white,
            colours.black)
        AI.text(W - 8, 2, string.format("ALT%5.0f", alt), colours.white, colours.black)
        local s = AI.sel()      -- ...in the colour a PFD reserves for a selection
        if s then
            AI.text(2, 2, (s == 'LAND') and s or string.format("SEL%4s", s),
                colours.magenta, colours.black)
        end
    end
end

function AI.draw()
    local W, H = mon.getSize()
    AI.size(W, H)
    if not sublevel then
        mon.setBackgroundColour(colours.black) mon.clear()
        mon.setTextColour(colours.red) mon.setCursorPos(2, 2) mon.write("NO SUBLEVEL API")
        AI.prev = {}
        return
    end
    AI.frame = AI.frame + 1
    for r = 1, H do
        local l = AI.pr[r]
        for i = #l, 1, -1 do l[i] = nil end
    end

    local pose = sublevel.getLogicalPose()
    AI.q, AI.v = pose.orientation, sublevel.getLinearVelocity()
    -- World "up" in the body frame. One vector, two degrees of freedom, and the pitch and
    -- bank below are exactly that vector again in polar form -- see the header.
    local ux, uy, uz = rotateVec(AI.q:inverse(), 0, 1, 0)
    ux, uz = HZN_ROLL_SIGN * ux, HZN_PITCH_SIGN * uz
    AI.pitch = math.deg(math.asin(uz < -1 and -1 or (uz > 1 and 1 or uz)))
    AI.bank  = math.deg(math.atan2(ux, uy))

    AI.k  = 3 * H / HZN_FOV                       -- subpixel rows per degree
    -- The tapes take the outer columns and the attitude picture is centred on what is left.
    -- Below that width the picture left over is not worth looking at, so there are no tapes
    -- and the whole monitor is attitude again, with the corner readouts back (AI.chrome).
    AI.tapes = (W >= AI.SPD_W + AI.ALT_W + AI.VS_W + AI.TAPE_FREE) and (H >= AI.TAPE_MIN_H)
    AI.fc0 = AI.tapes and (AI.SPD_W + 1) or 1
    AI.fc1 = AI.tapes and (W - AI.ALT_W - AI.VS_W) or W
    AI.hw = AI.fc1 - AI.fc0 + 1                   -- half the attitude picture, in subpixels
    AI.layout()
    AI.ox = 2 * (math.floor((AI.fc0 + AI.fc1 - 1) / 2) + 1) - 0.5   -- boresight, in subpixels
    AI.oy = 3 * math.floor(H / 2) - 1
    local t = math.rad(AI.bank)
    AI.nx, AI.ny = -math.sin(t), math.cos(t)      -- across the horizon, + = below it
    AI.B = -(AI.ox * AI.nx + AI.oy * AI.ny + AI.pitch * AI.k)

    -- The attitude window: as big a circle about the boresight as the monitor will take,
    -- leaving the corners for the readouts. The bank scale sits on its rim and the pitch
    -- ladder inside it.
    AI.R = math.min(AI.oy - 4, (AI.hw - 2) / 0.87)
    AI.rx, AI.ry = AI.hw - 4, AI.R - 5
    -- The HORIZON still runs the full width -- it passes behind the tapes -- so it is drawn
    -- to the furthest corner from a boresight that is no longer the middle of the monitor.
    local dx = math.max(AI.ox, 2 * W - AI.ox)
    local dy = math.max(AI.oy, 3 * H - AI.oy)
    local alt = pose.position.y - ALT_OFFSET
    local gs  = math.sqrt(AI.v.x * AI.v.x + AI.v.z * AI.v.z)
    AI.ladder(math.sqrt(dx * dx + dy * dy) + 2)
    AI.bankScale()
    AI.chrome(alt, gs, AI.v.y)
    AI.fpv()
    -- LAST, because they are opaque: everything above passes behind them.
    if AI.tapes then
        local bug = (altSelection and mode ~= 'OVER')
            and tonumber(guiStatus[altSelection].currentAlti) or nil
        AI.tape(1, AI.SPD_W, gs, SPD_STEP, -1, 0)
        -- The legend goes in the SPARE half of the speed tape's readout box, labelling the
        -- number it belongs to. At the foot of the tape, where it was, it shares a row with
        -- whatever figure the scale has reached -- and a "100" with its first two columns
        -- painted over reads as a speed of nought.
        AI.ontape = true
        AI.text(1, math.floor((AI.oy - 1) / 3) + 1, "GS",   -- no airspeed in this game
            colours.lightGrey, colours.black)
        AI.tape(AI.fc1 + 1, AI.fc1 + AI.ALT_W, alt, ALT_STEP, 1, nil, bug, colours.magenta)
        AI.vsTape(W - AI.VS_W + 1, W, AI.v.y)
        -- The selection over the head of the altitude tape, where the bug it matches is.
        AI.ontape = true
        local s = AI.sel()
        if s then
            AI.fillBox(AI.fc1 + 1, AI.fc1 + AI.ALT_W, 1, 1, colours.black)
            -- LEFT-aligned, so the spare column falls between this figure and the VS
            -- figure beside it: both are digits in a black box, and right-aligned they read
            -- as one number ("240" and "0.8" as 2400.8).
            AI.text(AI.fc1 + 1, 1, string.format("%-4s", s), colours.magenta, colours.black)
        end
        AI.ontape = false
    end
    for r = 1, H do AI.row(r) end
end

local function aiLoop()
    if not mon then while true do sleep(1) end end
    mon.setBackgroundColour(colours.black) mon.clear()
    while true do
        pcall(AI.draw)
        sleep(0.05)
    end
end

local function apAnnounceLoop()
    while true do
        local alt = redstone.getInput('front')
        rednet.broadcast({ ['autopilot_altitude'] = alt }, 'aertogekiss_apswitch_2')
        sleep(0.1)
    end
end

term.clear()
rednet.open('back')

drawPanel(1) drawPanel(2) drawPanel(3) drawPanel(4)

parallel.waitForAny(updateInput, broadcastAltiLoop, autolandLoop, aiLoop, apAnnounceLoop)
