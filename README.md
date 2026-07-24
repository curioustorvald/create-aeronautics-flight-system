# Aer Togekiss Flight System

A full **autopilot + fly-by-wire flight control system** for a [Create: Aeronautics](https://createsimulated.com/) VTOL aircraft, written in Lua for [CC: Tweaked](https://tweaked.cc) computers.

It flies the aircraft for you: it holds attitude and altitude, navigates a route of waypoints along a **smooth computed trajectory**, and lands itself — with a glass-cockpit set of instruments (artificial horizon, top-down HSI map) on the side. You stay in command through a cockpit of physical switches and sticks: master/altitude/attitude engine cut-offs, autopilot nav/altitude toggles, and — with the autopilot off — full hand-flying on a **vertical-speed stick** (climbs/descents) and a **nav stick** (fore/aft, strafe, yaw).

The centrepiece is the **Unified Attitude Controller (UAC)** — a quaternion-based attitude controller that reads the ship's true orientation, angular velocity and inertia tensor at 20 Hz and drives every thruster through one loop. No gimbal peripheral, no gimbal lock, and it can right the craft from fully inverted along the shortest path.

---

## Features

- **Quaternion attitude control** with an inner rate loop, inertia-tensor scaling and gyroscopic decoupling — stable through the whole envelope, including aggressive turns and upset recovery.
- **Trajectory autopilot**: fly direct-to a point, or a multi-waypoint route flown as one **smooth curve** — the FMS fits a centripetal Catmull-Rom spline through the waypoints, rounds the corners, and pure-pursues an aim-point along it, so the bearing changes continuously with no snap at a waypoint.
- **Curvature-based speed**: the FMS draws a *speed* curve to match the *path* curve — it holds cruise on the straights and slows through tight bends (and for the destination), braking **in advance**, all smoothly rate-limited.
- **Automatic landing**: align → translate over the pad → descend → flare → touchdown, with a pilot-override option to pause the descent and hand the vertical channel back to ALT-AP mid-approach.
- **Full manual fly-by-wire**: with both autopilot switches off, hand-fly the whole aircraft — the **nav stick** commands fore/aft/strafe/yaw and the **vertical-speed stick** commands climb/descent, while the UAC keeps it level and holds heading between inputs. Independent of the autopilot, so it doubles as an emergency course-alteration override.
- **Cockpit switches**: two **autopilot toggles** (navigation, altitude) select the flight mode live — full auto, navigate-only, altitude-only, or just-keep-level; three **engine switches** (master / altitude / attitude) are hardware cut-offs for the thruster banks.
- **Holonomic navigation**: it's a VTOL, so it strafes sideways to close on a target instead of relying on the (slow, heavy) yaw axis.
- **Attitude-aware thrust allocation**: "hold altitude" works even inverted or pitched 90° — the vertical effort is routed to whichever thruster is actually pointing up.
- **Glass cockpit**: an Attitude Indicator (artificial horizon with a real-AI bank pointer on the bezel) and an HSI that switches modes — a heading-up VOR/HSI whose course needle follows the flown **curve** while navigating, and a top-down landing map (aircraft, pad, heading-error bar) once a landing engages — on external monitors.
- **One touch-screen FMS** to build/edit waypoints and routes and to fly them.

## Requirements

| Mod | Why |
|-----|-----|
| **CC: Tweaked** | the computers everything runs on |
| **Create** + **Create: Aeronautics** | the airframe and its redstone-throttled gas thrusters (0–15) |
| **CC-Sable** (TechTastic) | provides the `sublevel` / `aero` API — the ship's live pose, velocity and inertia tensor |
| **Advanced-Math** (TechTastic) | provides the `quaternion`, `vector`, `matrix` globals and the `pid` module the UAC is built on |

## The computers at a glance

Twelve computers: three "brains" that read sensors and decide, three "relays" that copy rednet numbers onto redstone, and six cockpit "panels/sticks" that broadcast the pilot's switches and hand-flying inputs.

| Role | Folder | Needs |
|------|--------|-------|
| **Unified Attitude Controller** (the brain) | `attitude-controller/` | modem, on the aircraft (Sable) |
| **Flight Management System** (nav + touch UI) | `flight-management-system/` | modem, monitor, on the aircraft |
| **Altitude Selector** (ALT-AP + attitude indicator) | `altitude-selector/` | modem, monitor, on the aircraft |
| **Stabiliser relay** | `thruster-relay-stabiliser/` | modem, redstone to pitch/roll thrusters |
| **Yaw + fore/aft relay** | `thruster-relay-yaw-thrust/` | modem, redstone to yaw + fore/aft thrusters |
| **Strafe + altitude relay** | `thruster-relay-strafe-altitude/` | modem, redstone to strafe + up/down lift thrusters |
| **Vertical-speed commander** (manual VS stick) | `vertical-speed-commander/` | modem, analog redstone lever in |
| **Engine switches** (master/altitude/attitude) | `engine-switch/` | modem, 3 redstone switches in |
| **Autopilot switches** (nav/altitude) | `autopilot-switch/` | modem, 2 redstone switches in |
| **MANCON translation relay** (nav stick, fore/aft/strafe) | `mancon-translation-relay/` | modem, 4 redstone buttons in |
| **MANCON rotation relay** (nav stick, yaw) | `mancon-rotation-relay/` | modem, 2 redstone buttons in |
| **MANCON intention controller** (MIC) | `mancon-intention-controller/` | modem |

The last three are the **nav stick**: two relays read the pilot's fore/aft/strafe/yaw buttons, and the MIC turns those key-presses into wanted body velocities the UAC flies when the autopilot nav switch is off. (It mirrors the vertical-speed stick, but for the horizontal + yaw axes.)

## Install

Each folder above is dropped, contents-and-all, onto one CC computer. **[Full instructions, wiring, calibration and usage are in `MANUAL.md`.](MANUAL.md)** Do read the calibration section before you fly — every airframe needs its signs set once. The nav stick (`mancon-*`) is optional; leave it out if you only want autopilot flight.

## Layout

```
create-aeronautics-flight-system/
├─ README.md                       (this file)
├─ MANUAL.md                       (install, wiring, calibration, usage, troubleshooting)
├─ attitude-controller/            -> the UAC computer
│  ├─ startup.lua
│  └─ pid/  (uac.lua, pid.lua, probe.lua)
├─ flight-management-system/       -> the FMS computer
│  ├─ startup.lua, autopilot.lua
│  └─ points.db, course.db, landing.db   (example nav data)
├─ altitude-selector/              -> the ALT-AP computer  (startup.lua, altcom.lua)
├─ thruster-relay-stabiliser/      -> pitch/roll relay     (startup.lua, msg_to_gas_output.lua)
├─ thruster-relay-yaw-thrust/      -> yaw+fore/aft relay   (startup.lua, hdgcon.lua)
├─ thruster-relay-strafe-altitude/ -> strafe+lift relay    (startup.lua, strafecon.lua)
├─ vertical-speed-commander/       -> manual VS stick      (startup.lua, vspdpid.lua)
├─ engine-switch/                  -> engine cut-offs      (startup.lua, engine_switch.lua)
├─ autopilot-switch/               -> AP mode toggles      (startup.lua, autopilot_switch.lua)
├─ mancon-translation-relay/       -> nav stick fore/aft/strafe (startup.lua, manconrelay_translation.lua)
├─ mancon-rotation-relay/          -> nav stick yaw        (startup.lua, manconrelay_rotation.lua)
└─ mancon-intention-controller/    -> nav stick brain (MIC) (startup.lua, mic.lua)
```

## Credits

Built for the *Aer Togekiss*. Quaternion/PID math from TechTastic's **Advanced-Math**; ship telemetry from **CC-Sable**.
