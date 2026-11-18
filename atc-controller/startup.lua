-- Create: Aeronautics Flight System -- automated ATC controller (ground station, one per world)
-- Allocates landing facilities per sites.db (which arrives from the nav data server over
-- aer_navdata): an FMS inbound to a facility asks on aer_atc and is told cleared / divert /
-- hold / uncontrolled. Allocates ground only -- no headings, altitudes or air separation.
shell.run("atcctl.lua")
