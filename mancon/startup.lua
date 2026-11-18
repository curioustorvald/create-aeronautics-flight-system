-- Create: Aeronautics Flight System -- MANCON (the whole hand-flying stick on ONE computer)
-- A typewriter peripheral drives both manual channels: WER/SDF for fore/aft, strafe and yaw
-- (aertogekiss_mancon_nav) and SPACE/Z climb/descend, A boost, V scramble (aertogekiss_mancon).
-- A digital redstone lever is a second, keyboard-independent path to the scramble.
-- Replaces the four-computer stack (vertical-speed-commander + the two mancon-*-relay
-- computers + mancon-intention-controller). Do NOT run both: they broadcast the same
-- protocols, and two senders on one protocol fight at 20 Hz.
-- All wiring, keys and rates are in manconconf.lua. get_keycodes.lua prints a key's code.
shell.run("mancon.lua")
