# Aer Togekiss Flight System

A full **autopilot + fly-by-wire flight control system** for a [Create: Aeronautics](https://createsimulated.com/) VTOL aircraft, written in Lua for [CC: Tweaked](https://tweaked.cc) computers.

It flies the aircraft for you: it holds attitude and altitude, navigates a route of waypoints, and lands itself — with a glass-cockpit set of instruments (artificial horizon, VOR/HSI) on the side.

The centrepiece is the **Unified Attitude Controller (UAC)** — a quaternion-based attitude controller that reads the ship's true orientation, angular velocity and inertia tensor at 20 Hz and drives every thruster through one loop. No gimbal peripheral, no gimbal lock, and it can right the craft from fully inverted along the shortest path.

---

## Features

- **Quaternion attitude control** with an inner rate loop, inertia-tensor scaling and gyroscopic decoupling — stable through the whole envelope, including aggressive turns and upset recovery.
- **Full autopilot**: fly direct-to a point, or a multi-waypoint route with VOR/POI fixes and cross-track guidance.
- **Automatic landing**: align → translate over the pad → descend → flare → touchdown, with a pilot-override option to pause the descent and hand the vertical channel back to ALT-AP mid-approach.
- **Holonomic navigation**: it's a VTOL, so it strafes sideways to close on a target instead of relying on the (slow, heavy) yaw axis.
- **Attitude-aware thrust allocation**: "hold altitude" works even inverted or pitched 90° — the vertical effort is routed to whichever thruster is actually pointing up.
- **Glass cockpit**: an Attitude Indicator (PFD) and a VOR/HSI with course-deviation needle, on external monitors.
- **One touch-screen FMS** to build/edit waypoints and routes and to fly them.

## Requirements

| Mod | Why |
|-----|-----|
| **CC: Tweaked** | the computers everything runs on |
| **Create** + **Create: Aeronautics** | the airframe and its redstone-throttled gas thrusters |
| **CC-Sable** (TechTastic) | provides the `sublevel` / `aero` API — the ship's live pose, velocity and inertia tensor |
| **Advanced-Math** (TechTastic) | provides the `quaternion`, `vector`, `matrix` globals and the `pid` module the UAC is built on |

## The computers at a glance

| Role | Folder | Needs |
|------|--------|-------|
| **Unified Attitude Controller** (the brain) | `attitude-controller/` | modem, on the aircraft (Sable) |
| **Flight Management System** (nav + touch UI) | `flight-management-system/` | modem, monitor, on the aircraft |
| **Altitude Selector** (ALT-AP + attitude indicator) | `altitude-selector/` | modem, monitor, on the aircraft |
| **Stabiliser relay** | `thruster-relay-stabiliser/` | modem, redstone to pitch/roll thrusters |
| **Yaw + fore/aft relay** | `thruster-relay-yaw-thrust/` | modem, redstone to yaw + fore/aft thrusters |
| **Strafe + altitude relay** | `thruster-relay-strafe-altitude/` | modem, redstone to strafe + lift thrusters |

## Install

Each folder above is dropped, contents-and-all, onto one CC computer. **[Full instructions, wiring, calibration and usage are in `MANUAL.md`.](MANUAL.md)** Do read the calibration section before you fly — every airframe needs its signs set once.

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
└─ thruster-relay-strafe-altitude/ -> strafe+lift relay    (startup.lua, strafecon.lua)
```

## Credits

Built for the *Aer Togekiss*. Quaternion/PID math from TechTastic's **Advanced-Math**; ship telemetry from **CC-Sable**.
