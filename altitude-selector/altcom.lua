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
local defaultAlti = {'120', '150', '180', '250'}

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
-- EXTERNAL MONITOR -- Attitude Indicator (PFD): artificial horizon + ground speed,
-- altitude and vertical speed. Reads the craft state straight off sublevel (this
-- computer is on the aircraft). Display-only; the keypad (mouse_click) is untouched.
-- If no monitor is attached the loop just idles.
-- ============================================================================
local mon = peripheral.find("monitor")

-- colour -> blit char, for fast per-row horizon fills via mon.blit
local BLIT = {}
for i = 0, 15 do BLIT[2 ^ i] = ("0123456789abcdef"):sub(i + 1, i + 1) end

local function aiEuler(o)
    local a, b, c = o:toEuler()
    if type(a) == "number" then return a, b, c end
    return a.x, a.y, a.z
end

if mon then
    mon.setTextScale(0.5)
    pcall(function()
        mon.setPaletteColour(colours.blue,  0x2277cc)   -- sky
        mon.setPaletteColour(colours.brown, 0x8a5a2b)   -- ground
        mon.setPaletteColour(colours.orange, 0xffbb00)
    end)
end

local function box(x, y, text, col)
    mon.setBackgroundColour(colours.black)
    mon.setTextColour(col or colours.white)
    mon.setCursorPos(x, y); mon.write(text)
end

local function drawAI()
    local W, H = mon.getSize()
    if not sublevel then
        mon.setBackgroundColour(colours.black); mon.clear()
        box(2, 2, "NO SUBLEVEL API", colours.red)
        return
    end
    local pose = sublevel.getLogicalPose()
    local pitch = select(1, aiEuler(pose.orientation))
    local roll  = select(3, aiEuler(pose.orientation))
    local alt   = pose.position.y
    local v     = sublevel.getLinearVelocity()
    local gs    = math.sqrt(v.x * v.x + v.z * v.z)
    local vs    = v.y

    local midX = math.floor(W / 2) + 1
    local midY = math.floor(H / 2) + 0
    local pxPerRad = H / 1.6                                  -- rows per rad of pitch
    local slope    = math.tan(math.max(-1.3, math.min(1.3, -roll))) * (2 / 3)  -- char aspect
    local pitchOff = pitch * pxPerRad                          -- +pitch (nose up) -> horizon down

    -- horizon, one blit per row (sky above, ground below, white line at the boundary)
    local sky, gnd, wht = BLIT[colours.blue], BLIT[colours.brown], BLIT[colours.white]
    local spaces = string.rep(" ", W)
    local fg = string.rep("0", W)
    for row = 1, H do
        local bg = {}
        for col = 1, W do
            local hrow = math.floor(midY + pitchOff - (col - midX) * slope + 0.5)
            bg[col] = (row < hrow) and sky or (row > hrow and gnd or wht)
        end
        mon.setCursorPos(1, row); mon.blit(spaces, fg, table.concat(bg))
    end

    -- ---- outer rim: a bank (roll) indicator that ignores pitch --------------
    -- A thin bezel painted over the horizon edges -- 2 cols each side, 1 row top & bottom
    -- -- carrying a roll pointer that reads bank ALONE. Pitch never moves it, so the bank
    -- stays readable even in a steep climb/dive when the horizon has slid off-screen, just
    -- like a real AI's bank pointer against its fixed bezel scale.
    mon.setBackgroundColour(colours.black)
    for row = 1, H do
        mon.setCursorPos(1, row);     mon.write("  ")          -- left rim (2 wide)
        mon.setCursorPos(W - 1, row); mon.write("  ")          -- right rim (2 wide)
    end
    mon.setCursorPos(1, 1); mon.write(string.rep(" ", W))      -- top rim (1 tall)
    mon.setCursorPos(1, H); mon.write(string.rep(" ", W))      -- bottom rim (1 tall)

    -- A ray from centre at the bank angle lands on the rim: on the top row for |bank| up
    -- to ~60 deg, then onto the side rims. 0 = straight up; +bank goes to the right.
    local bcx, bcy = (1 + W) / 2, (1 + H) / 2
    local function rimXY(theta)
        local s, c = math.sin(theta), math.cos(theta)
        local t = math.huge
        if c >  1e-6 then t = math.min(t, (bcy - 1) / c) end   -- top edge
        if s >  1e-6 then t = math.min(t, (W - bcx) / s) end   -- right edge
        if s < -1e-6 then t = math.min(t, (1 - bcx) / s) end   -- left edge
        if c < -1e-6 then t = math.min(t, (H - bcy) / c) end   -- bottom edge
        return math.floor(bcx + t*s + 0.5), math.floor(bcy - t*c + 0.5)
    end
    for _, d in ipairs({-60, -30, 30, 60}) do                  -- fixed bank-scale ticks
        local x, y = rimXY(math.rad(d)); box(x, y, "'", colours.grey)
    end
    do local x, y = rimXY(0); box(x, y, "|", colours.white) end -- fixed 0 index
    -- Moving roll pointer. Flip the sign if it banks the wrong way (this file's one switch).
    local px, py = rimXY(math.max(-1.4, math.min(1.4, roll)))
    mon.setCursorPos(px, py); mon.setBackgroundColour(colours.orange); mon.write(" ")

    -- fixed aircraft reference wings at centre
    mon.setBackgroundColour(colours.black); mon.setTextColour(colours.orange)
    mon.setCursorPos(midX - 4, midY); mon.write("---")
    mon.setCursorPos(midX + 2, midY); mon.write("---")
    mon.setCursorPos(midX,     midY); mon.write("O")

    -- readouts (kept inside the rim)
    box(3,        H - 1, string.format("GS %3.0f", gs))
    box(midX - 3, 2,     string.format("ALT%4.0f", alt))
    box(W - 8,    H - 1, string.format("VS%+4.1f", vs), vs >= 0.1 and colours.lime or (vs <= -0.1 and colours.orange or colours.white))
end

local function aiLoop()
    if not mon then while true do sleep(1) end end
    mon.setBackgroundColour(colours.black); mon.clear()
    while true do
        pcall(drawAI)
        sleep(0.15)
    end
end

term.clear()
rednet.open('back')

drawPanel(1) drawPanel(2) drawPanel(3) drawPanel(4)

parallel.waitForAny(updateInput, broadcastAltiLoop, autolandLoop, aiLoop)
