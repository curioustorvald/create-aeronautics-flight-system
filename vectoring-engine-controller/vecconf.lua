-- SCOUT -- VECTORING ENGINE CONTROLLER: PER-COMPUTER CONFIGURATION
--
-- Which engine this computer is, and what its outputs are plugged into. Split out of
-- vecengine.lua so the PROGRAM and this COMPUTER's wiring are separate files -- the same split,
-- for exactly the same reason, as pid/uacconf.lua against pid/uac.lua:
--
--   **dropping in a new vecengine.lua must never be able to touch what is in here.**
--
-- That is not a theoretical concern. On 2026-08-22 this file did not exist, the wiring lived in
-- vecengine.lua, and regenerating #78 and #80 from #77 (to keep the three program copies in
-- step, which is right) silently overwrote both of their WIRING tables with #77's -- so all
-- three engines drove front-centre's relay layout and the aircraft rolled itself over. The
-- wiring for #78 and #80 was not recoverable. Hence this file.
--
-- HOW IT LOADS: vecengine.lua does require("vecconf"), which resolves relative to the running
-- program's directory -- so this file must sit BESIDE vecengine.lua. It must return the table.
-- A missing or incomplete one is a named boot error, not a silent half-configured engine.

local VC = {}

-- WHICH ENGINE THIS IS. Must match a key in CFG.TRI_ENGINES (72/pid/uacconf.lua): it is the
-- slot this computer reads out of the UAC's message.
VC.ENGINE      = 'fc'
VC.ENGINE_NAME = 'FRONT-CENTRE'


-- ============================================================================
-- WIRING -- THE OUTPUT ALLOCATION
-- ============================================================================
-- FIVE CHANNELS come out of this computer:
--     x_pos  x_neg    gimbal X, split into two lines because redstone has no sign
--     y_pos  y_neg    gimbal Y, same
--     thrust           0..full, no sign
--
-- Each channel is a LIST OF SINKS. The channel's integer level is spread across its sinks,
-- highest weight first, so a channel is not limited to the 0..15 one redstone line can carry.
-- A sink is one of:
--
--   { side = "left", max = 15 }
--       this computer's own redstone face.
--
--   { per = "left", side = "back", max = 15 }
--       a face of a wrapped peripheral that speaks setAnalogOutput -- a Redstone Relay, or
--       anything else with the same method. It is exactly
--           peripheral.wrap("left").setAnalogOutput("back", level)
--       written as data. `per` is whatever peripheral.wrap() takes: a SIDE of this computer,
--       when the relay is the block bolted to that face, or a network name like
--       "redstone_relay_3" when it sits out on a wired modem. Channels naming the same `per`
--       share one wrap. Note { side = "left" } drives this computer's OWN left face, whereas
--       { per = "left", side = ... } drives the relay ON that face.
--
--   { per = "vector_thruster_0", call = "setAngleX", max = 15 }
--       a method call instead of a redstone level: put(level) becomes p.setAngleX(level).
--
--   { ..., weight = 3 }
--       optional, default 1: what one level on this sink is worth in channel units. Two lines
--       at weights 3 and 1 make 0..60 out of two 0..15 faces. Put the weight-1 sink LAST or
--       some totals cannot be expressed, and the leftover shows as ! on the display.
--
-- SCALE_* is what "full deflection" and "full thrust" are worth in channel units -- what the
-- quantiser aims at. 15 for one plain line per channel; raise to the real capacity if a
-- channel is wired across several.
VC.SCALE_X = 15
VC.SCALE_Y = 15
VC.SCALE_T = 15

-- TEMPLATE -- fill in for THIS computer. Empty means "not wired": the program boots, reports
-- "wiring: INCOMPLETE" and drives nothing, which is the safe state to install into. An engine
-- with the WRONG wiring rolls the aircraft over; an engine with none simply does not push.
--
-- A worked example -- one Redstone Relay bolted to this computer's left face, carrying all
-- five channels on its five free sides (one wrap, five entries):
--   x_pos  = { { per = "left", side = "front",  max = 15 } },
--   x_neg  = { { per = "left", side = "back",   max = 15 } },
--   y_pos  = { { per = "left", side = "left",   max = 15 } },
--   y_neg  = { { per = "left", side = "right",  max = 15 } },
--   thrust = { { per = "left", side = "bottom", max = 15 } },
VC.WIRING = {
    x_pos  = {},
    x_neg  = {},
    y_pos  = {},
    y_neg  = {},
    thrust = {},
}

return VC
