# Aer Togekiss Flight System

A full **autopilot + fly-by-wire flight control system** for [Create: Aeronautics](https://createsimulated.com/) aircraft, written in Lua for [CC: Tweaked](https://tweaked.cc) computers.

It flies the aircraft for you: it holds attitude and altitude, navigates a route of waypoints along a **smooth computed trajectory**, takes off from a runway by running it, flies published arrivals, lands itself onto a pad or a runway — and draws a glass cockpit (PFD, HSI with a terrain map, a head-up display with ground-proximity warnings) on monitors. You stay in command through a cockpit of physical switches and sticks: engine cut-offs, autopilot nav/altitude toggles, and — with the autopilot off — full hand-flying from a keyboard stick or a button panel.

The centrepiece is the **Unified Attitude Controller (UAC)** — a quaternion-based attitude controller that reads the ship's true orientation, angular velocity and inertia tensor and drives every thruster through one loop. No gimbal peripheral, no gimbal lock, and it can right the craft from fully inverted along the shortest path.

Around the aircraft, optional **ground stations** give a fleet a shared navigation database, a terrain survey, radar scopes and an automated landing-facility controller.

**What's new in this release: see [`CHANGELOG.md`](CHANGELOG.md)** — including the upgrade notes if you are running the August build.

---

## Features

- **Quaternion attitude control** with an inner rate loop, inertia-tensor scaling and gyroscopic decoupling — stable through the whole envelope, including aggressive turns and upset recovery. Deadline-paced and tuned for what a CC computer actually gets on a busy server.
- **Trajectory autopilot**: fly direct-to a point, or a multi-waypoint route flown as one **smooth curve** — every interior waypoint gets a `straight → clothoid → arc → clothoid → straight` fly-by transition sized from `R = V²/(g·tan(bank))`, and the aircraft pure-pursues an aim-point along the baked path.
- **Planned speed profile**: a two-pass (brake-backward, accelerate-forward) profile over the whole route, so it holds cruise on the straights and arrives at each bend already slowed.
- **Three flight modes**: **VTOL** (strafes onto the track), **FIXEDWING** (banks only, coordinated turns) and **HYBRID** (banks en route, strafes for precise pad work). One flight plan flies identically in all three.
- **Runway departures**: commit a plan while standing on a published runway and it lines up, runs the strip, rotates, climbs straight out and only then turns for the route — in any flight mode. *(New; not yet flight-tested — see the changelog.)*
- **Automatic landing, two kinds**: onto a **pad** (align → translate → descend → flare → touchdown) or onto a **runway** (straight-in on a glideslope, touching down in the touchdown zone and braking along the strip — the runway's *length* caps the approach speed). Missed approaches fly a **published go-around circuit** automatically and try again.
- **Published approaches**: a field publishes its arrivals in `landing.db`, and the FMS picks the one that suits the aircraft's position and heading, with its minimum altitude and missed-approach circuit. Routes may name an approach (or another route) directly.
- **Terrain avoidance**: with a terrain server in the world, the FMS keeps an automatic altitude floor along the track ahead — it only ever climbs, never descends you.
- **HUD + GPWS**: a head-up display and a ground-proximity warning system — sink rate, terrain closure, terrain/obstacle ahead, altitude loss after take-off, too low terrain, decision-height callouts, autopilot disconnect — with spoken callouts from a voice-pack floppy.
- **Automated ATC**: transponders on every aircraft, radar scopes (tower or airborne AWACS), and a controller that clears, holds or diverts aircraft between the landing facilities of an aerodrome so two never land on one pad.
- **Full manual fly-by-wire**: with both autopilot switches off, hand-fly the whole aircraft — off a single computer with a **typewriter** (WER/SDF to translate and yaw, SPACE/Z to climb and descend, A to boost, V to scramble) or off the original button panel and vertical-speed lever. The UAC keeps it level and holds heading between inputs. A **scramble** detent gives full lift instantly, overriding ALT-AP and the autoland.
- **Cockpit switches**: two **autopilot toggles** (navigation, altitude) select the flight mode live; four **engine switches** (master / altitude / attitude / extra lift) are hardware cut-offs for the thruster banks.
- **Attitude-aware thrust allocation**: "hold altitude" works even inverted or pitched 90° — the vertical effort is routed to whichever thrusters are actually pointing up.
- **Glass cockpit**: a **PFD** (artificial horizon, pitch ladder, bank scale and flight-path marker at sub-character resolution, with speed and altitude tapes and a VS scale) and an **HSI** with three faces — a VOR/HSI following the flown curve, a **PLAN view** of the course over a terrain-hazard map with traffic, and a top-down landing map with glideslope pointer.
- **One touch-screen FMS** to build/edit waypoints and routes and to fly them — or run a **nav data server** and maintain the whole fleet's database in one place, over the air.
- **Configuration is separate from program** for every program with more than one copy: every sign, protocol name, speed and gain is in a `*conf.lua`, and no program carries a fallback default — a missing knob is a boot error that names it, never a silent surprise in the air.

## Requirements

| Mod | Why |
|-----|-----|
| **CC: Tweaked** | the computers everything runs on |
| **Create** + **Create: Aeronautics** | the airframe and its redstone-throttled gas thrusters (0–15) |
| **CC-Sable** (TechTastic) | provides the `sublevel` / `aero` API — the ship's live pose, velocity and inertia tensor |
| **Advanced-Math** (TechTastic) | provides the `quaternion`, `vector`, `matrix` globals and the `pid` module the UAC is built on |

The terrain builder (`terrain-data-server/terrainbuild/`) is a separate Java 17+ program run on the server machine, outside the game.

## The computers at a glance

**The core aircraft** is three "brains" that read sensors and decide, three "relays" that copy rednet numbers onto redstone, an engine-switch panel, and a hand-flying stick. Everything else is optional.

| Role | Folder | Needs |
|------|--------|-------|
| **Unified Attitude Controller** (the brain) | `attitude-controller/` | modem, on the aircraft (Sable) |
| **Flight Management System** (nav + touch UI + HSI) | `flight-management-system/` | modem, touch monitor (+ a second monitor for the HSI), on the aircraft; the **AP-NAV toggle** switch in |
| **Altitude Selector** (ALT-AP + PFD) | `altitude-selector/` | modem on `back`, monitor, on the aircraft; the **AP-ALT toggle** switch in |
| **Stabiliser relay** | `thruster-relay-stabiliser/` | modem, redstone to pitch/roll thrusters |
| **Yaw + fore/aft relay** | `thruster-relay-yaw-thrust/` | modem, redstone to yaw + fore/aft thrusters |
| **Strafe + altitude relay** | `thruster-relay-strafe-altitude/` | modem, redstone to strafe + up/down lift thrusters |
| **Engine switches** (master/altitude/attitude/extra lift) | `engine-switch/` | modem, 4 redstone switches in |
| **Autopilot-switch merger** *(optional)* | `autopilot-switch/` | modem on `top`; merges the two toggles' announcements for the relays' status lines |
| **Hand-flying stick — one computer** | `mancon/` | modem, **typewriter**, optional monitor and scramble lever |
| *…or* the **four-computer stick**: | | |
| ↳ vertical-speed commander | `vertical-speed-commander/` | modem, analog redstone lever in |
| ↳ MANCON translation relay | `mancon-translation-relay/` | modem, 4 redstone buttons in |
| ↳ MANCON rotation relay (+ squat switch, extra lift) | `mancon-rotation-relay/` | modem, 2 buttons + gear contact in, extra-lift bank out |
| ↳ MANCON intention controller (MIC) | `mancon-intention-controller/` | modem |
| **Keyboard joystick** *(optional, FIXEDWING)* | `keyboard-joystick/` | modem, **typewriter**, monitor (1×1 is enough) |
| **HUD / GPWS** *(optional)* | `gpws-hud/` | modem, monitor, speaker; a disk drive for the voice pack |
| **ATC responder** *(optional)* | `atc-responder/` | modem |
| **Fore/aft thrust controller** *(optional)* | `fore-aft-thrust-controller/` | modem, redstone to a multi-line fore/aft bank |
| **Vectoring engine controller** *(optional, ×3)* | `vectoring-engine-controller/` | only for a three-nozzle airframe — see its `uacconf-tri-section.lua` |
| **Ground stations** *(optional, off-aircraft, one per world)*: | | |
| ↳ nav data server | `nav-data-server/` | modem, monitor, keyboard |
| ↳ terrain data server | `terrain-data-server/` | **ender** modem, disk drive with the survey, optional monitor |
| ↳ ATC radar (tower) | `atc-radar/` | modem, monitor (also flies as an AWACS scope on an aircraft) |
| ↳ ATC controller | `atc-controller/` | modem, monitor |

**Hand-flying comes in two shapes.** `mancon/` does on one computer what the four-computer stack does on four, from a typewriter. Install one or the other — both broadcast the same protocols, and two senders on one protocol fight. If you use `mancon/`, still consider the `mancon-rotation-relay/` computer for its other two jobs (the landing gear's squat switch, which the FMS and GPWS read as ground truth for a touchdown, and the extra-lift bank), with `YAW_RELAY_ENABLED = false`.

## Install

Each folder above is dropped, contents-and-all, onto one CC computer. **[Full instructions, wiring, calibration and usage are in `MANUAL.md`.](MANUAL.md)** Do read the calibration section before you fly — every airframe needs its signs set once, and its **altitude datum** (`ALT_OFFSET`) measured.

Deep reference — every protocol, every state machine, every config section and the known traps — is in [`docs/`](docs/README.md).

## Layout

```
create-aeronautics-flight-system/
├─ README.md, MANUAL.md, CHANGELOG.md
├─ docs/                           (the reference manual: protocols, internals, traps)
├─ attitude-controller/            -> the UAC computer
│  ├─ startup.lua
│  └─ pid/  uac.lua, uacconf.lua (ALL its calibration), pid.lua
│           probe.lua, signcal.lua, uacprof.lua, scrambletrace.lua   (diagnostics)
├─ flight-management-system/       -> the FMS computer
│  ├─ startup.lua, autopilot.lua, apconf.lua   (apconf = ALL its tuning/signs/protocols)
│  └─ points.db, course.db, landing.db   (example nav data)
├─ altitude-selector/              -> ALT-AP + PFD          (altcom.lua)
├─ thruster-relay-stabiliser/      -> pitch/roll relay      (msg_to_gas_output.lua)
├─ thruster-relay-yaw-thrust/      -> yaw+fore/aft relay    (hdgcon.lua)
├─ thruster-relay-strafe-altitude/ -> strafe+lift relay     (strafecon.lua)
├─ engine-switch/                  -> engine cut-offs       (engine_switch.lua)
├─ autopilot-switch/               -> AP toggle merger      (autopilot_switch.lua)
├─ mancon/                         -> one-computer stick    (mancon.lua, manconconf.lua, get_keycodes.lua)
├─ vertical-speed-commander/       -> 4-computer stick: VS lever          (vspdpid.lua)
├─ mancon-translation-relay/       -> 4-computer stick: fore/aft/strafe   (manconrelay_translation.lua)
├─ mancon-rotation-relay/          -> 4-computer stick: yaw + squat switch + extra lift
├─ mancon-intention-controller/    -> 4-computer stick: the MIC           (mic.lua)
├─ keyboard-joystick/              -> FIXEDWING stick       (keyboardjoystick.lua, get_keycodes.lua)
├─ gpws-hud/                       -> HUD + GPWS            (gpws.lua, gpwsconf.lua)
├─ atc-responder/                  -> transponder           (responder.lua)
├─ fore-aft-thrust-controller/     -> finer fore/aft output (thrustcon.lua)
├─ vectoring-engine-controller/    -> one per nozzle        (vecengine.lua, vecconf.lua, uacconf-tri-section.lua)
├─ nav-data-server/                -> ground: the MASTER points/course/landing/sites.db
├─ terrain-data-server/            -> ground: terrain_server.lua, terrainconf.lua, TERRAIN.md
│  └─ terrainbuild/                   the offline survey builder (Java source, build.sh, terrain-update.sh)
├─ atc-radar/                      -> ground/air: atc.lua, atcconf.lua (tower), atcconf.awacs.lua
├─ atc-controller/                 -> ground: atcctl.lua, atcctlconf.lua
└─ tools/tickprobe.lua             (what loop rate a computer really gets on your server)
```

**Your calibration lives in the `*conf.lua` files, and nothing else does**: `uacconf.lua`, `apconf.lua`, `manconconf.lua`, `gpwsconf.lua`, `atcconf.lua`, `atcctlconf.lua`, `terrainconf.lua`, `vecconf.lua`. Keep them when you update a program; no program carries a fallback copy of any value (a missing or truncated one is a boot error that names the missing knob, rather than a crash in the air). The relays and panels have no conf file — their protocol names and redstone faces are written at the top of each program.

The shipped conf files are the *Aer Togekiss*'s flown calibration — a VTOL with an altitude datum of 9 and a 320 m/s cruise. They are a starting point for your airframe, not a calibration of it.

If you fly more than one aeroplane, give each its own protocol prefix (`aertogekiss_` throughout) in every conf file and every relay/panel. Four protocols are fleet-wide and stay the same on every aircraft: `aer_navdata`, `atc_broadcast`, `aer_terrain`, `aer_atc`.

## Credits

Built for the *Aer Togekiss*, with *Mantine*, *Citation*, *SS Argo* and *Scout* as the other development airframes. Quaternion/PID math from TechTastic's **Advanced-Math**; ship telemetry from **CC-Sable**.
