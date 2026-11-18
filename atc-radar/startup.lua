-- Create: Aeronautics Flight System -- ATC radar scope. Receive-only; it controls nothing.
-- atcconf.lua as shipped is a ground TOWER (STATION.MODE = "static"). For an airborne AWACS
-- scope centred on its own aircraft, use atcconf.awacs.lua as atcconf.lua instead.
shell.run("atc.lua")
