-- Aer Togekiss Flight System -- Keyboard Joystick (hand-flying stick + throttle quadrant)
-- Reads a typewriter peripheral's held keys (ESDF cluster: E/D pitch, S/F roll, SPACE/A throttle,
-- C centres) and broadcasts the wanted attitude + throttle (aertogekiss_kbdjoy). Drives no
-- thrusters -- like the VS-commander and the MIC, it only NAMES a wanted attitude; the UAC flies
-- it, and only in FIXEDWING with AP-NAV off. Sign conventions live in the UAC's uacconf.lua
-- (CFG.KJOY_PITCH_SIGN / KJOY_ROLL_SIGN) -- this program always sends raw stick position.
-- get_keycodes.lua is a scratch tool: run it to print the GLFW code of any key you press.
shell.run("keyboardjoystick.lua")
