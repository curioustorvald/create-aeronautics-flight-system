-- Aer Togekiss Flight System -- Unified Attitude Controller (UAC)
-- Boots the controller. All calibration/tuning lives in pid/uacconf.lua (see the manual);
-- pid/uac.lua is the program and can be replaced without touching your calibration.
shell.run("pid/uac.lua")
