local CALLSIGN = 'TOGEKISS'
local CSG = 'TGK'
local ATC_CAST = 'atc_broadcast'
local tcas_status = 'CLEAR' -- CLEAR, Vx (vertical adj; x: signed VS), nil if TCAS incapable
local HEADING_SIGN   = 1
local HEADING_OFFSET = math.pi   -- ship's logical forward is 180 from thrust-forward
-- THE ALTITUDE DATUM -- how far the pose `sublevel` reports sits ABOVE THE BOTTOM OF THE
-- HULL, in metres. Subtracted from the altitude this computer puts on atc_broadcast, so what
-- the fleet is painted at is the world Y of the part of each aeroplane that touches the
-- ground -- the same datum landing.db's pad and runway elevations are surveyed in, and the
-- same one the ATC scope's own ALT readout uses. MUST MATCH this airframe's pid/uacconf.lua,
-- apconf.lua, gpwsconf.lua and altcom.lua for the same reason HEADING_* above must match
-- them: it is one hull, and vertical separation read off a scope is only as good as the
-- worst-calibrated aeroplane on it. Measure by parking on ground of a known Y with this at 0
-- and taking (what it read) - (the Y the hull rests on); it is a build dimension.
local ALT_OFFSET = 9   -- measured 2026-08-29: on the Y=63 airstrip it read 73

if not sublevel then error("No sublevel API - is the Sable mod present?") end

local modem = peripheral.find("modem")
if not modem then error("No modem attached") end
rednet.open(peripheral.getName(modem))
term.clear()

local function displayLine(row, text)
    term.setCursorPos(1, row)
    term.clearLine()
    term.write(text)
end
local function toCompass(rad) return (180 - math.deg(rad)) % 360 end


-- toEuler tolerates either return shape: (pitch,yaw,roll) numbers or one vector.
local function euler(orientation)
    local a, b, c = orientation:toEuler()
    if type(a) == "number" then return a, b, c end
    return a.x, a.y, a.z
end
local function getYaw(orientation)
    local _, y = euler(orientation)
    return y
end

local apnav, apalt = false
function switchApnav()
    while true do
        local sender, msg, __ = rednet.receive("aertogekiss_apswitch_1", 1)
        if sender ~= nil then
            apnav = msg.autopilot_nav
        end
    end
end

displayLine(1, 'RESPONDER - Callsign '..CALLSIGN..'/'..CSG)

function broadcastLoop()
    while true do
        local pose   = sublevel.getLogicalPose()
        local pos    = pose.position
        local q_cur  = pose.orientation
        local vel    = sublevel.getLinearVelocity()
        local w      = sublevel.getAngularVelocity()
        local v      = sublevel.getLinearVelocity()
        local gs     = math.sqrt(v.x * v.x + v.z * v.z)
        local vs     = v.y
        local hdgf   = HEADING_SIGN * getYaw(q_cur) + HEADING_OFFSET
        local chdg   = toCompass(hdgf)

        rednet.broadcast({
            ['x'] = pos.x,
            ['y'] = pos.y - ALT_OFFSET,
            ['z'] = pos.z,
            ['gs'] = gs,
            -- The velocity VECTOR as well as its magnitude: ATC draws the leader line along
            -- the ground track, and on a VTOL the track and the heading are routinely 90
            -- degrees apart (it holds a heading and translates sideways). Extra fields only
            -- -- a receiver that does not know about them is unaffected.
            ['vx'] = v.x,
            ['vz'] = v.z,
            ['vs'] = vs,
            ['ys'] = w.y,
            ['hdg'] = chdg,
            ['callsign'] = CALLSIGN,
            ['callsign_short'] = CSG,
            ['is_ifr'] = apnav,
            ['tcas_status'] = tcas_status
        }, ATC_CAST)

        displayLine(3, string.format('POS %.1f %.1f   ALT %.1f', pos.x, pos.z, pos.y - ALT_OFFSET))
        displayLine(4, string.format('HDG %03.1f', chdg))
        displayLine(5, string.format('GS %+6.1f   VS %+6.1f   YS %+6.1f', gs, vs, w.y))
        displayLine(6, apnav and 'IFR' or 'VFR')
        displayLine(7, 'TCAS '..tcas_status)

        sleep(0.05)
    end
end

parallel.waitForAny(broadcastLoop, switchApnav)
