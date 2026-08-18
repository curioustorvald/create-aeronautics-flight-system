-- Aer Togekiss Flight System -- Nav Data Update Server
-- The fleet's MASTER navigation database (points.db / course.db / landing.db), a keyboard+mouse
-- editor for it, and a broadcaster: every aircraft's FMS picks edits up over the air on
-- aer_navdata, so waypoints are maintained in ONE place instead of per aeroplane.
-- The three .db files shipped here are this server's copies -- they are the masters, and the
-- ones each FMS will be overwritten with. Edit them here (or on screen), not on the aircraft.
-- The roster on screen is how you confirm an edit actually reached an aeroplane.
shell.run("nav_update_server.lua")
