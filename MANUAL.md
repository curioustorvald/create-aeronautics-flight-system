# Aer Togekiss Flight System — User's Manual

> A quaternion fly-by-wire autopilot for a Create: Aeronautics VTOL, on CC: Tweaked computers.

---

## 0. Introduction — what this is

This is a complete **autopilot and flight-control stack** for a Create: Aeronautics VTOL aircraft. Once installed and calibrated, it will:

- keep the aircraft **level and pointed** where you want it, automatically, at all times;
- **hold a selected altitude**;
- **navigate** to a single point (direct-to) or fly a **route** of waypoints, holding the course line;
- **land itself** on a marked pad — line up, translate over the pad, descend, flare, touch down;
- show you a **glass cockpit** — an artificial horizon and a VOR/HSI course indicator.

**The heart of it is the Unified Attitude Controller (UAC).** A Create: Aeronautics craft is flown by lots of little gas thrusters — some for pitch/roll, some for yaw, some for fore/aft and strafe, some for lift. Older setups stabilised these with a *gimbal peripheral* and Euler angles, which can't cleanly recover an inverted craft and needs extra hardware. The UAC throws that away: it reads the ship's **true orientation quaternion, angular velocity and inertia tensor** straight off the physics engine (via CC-Sable) at 20 Hz, and runs **one control loop that owns every thruster**. Because it works in quaternions it has no gimbal lock, no ±180° wrap seams, and it recovers from *any* attitude — including fully upside-down — along the shortest path.

On top of that core it does a lot of things a real flight computer does:

- an **inner rate loop with inertia-tensor scaling and gyroscopic decoupling**, so a heavy, high-inertia airframe is controlled properly rather than just damped;
- a **motion-profiled yaw** that starts braking a turn *early* so the nose doesn't overshoot the heading;
- **holonomic navigation** — since a VTOL can slide sideways, it *strafes* toward a target rather than waiting for the slow yaw axis to swing the nose around (which otherwise causes a lazy spiral that never closes);
- **attitude-aware vertical thrust** — "hold altitude" is a *world-up* goal, so if the craft is banked, pitched or inverted, the vertical effort is re-routed to whichever thrusters are actually pointing up;
- an **automatic-landing state machine**, with a **pilot override** to pause the descent mid-approach and hand the vertical channel back to you (keeping the alignment), plus **AP OFF / RESUME ALT-AP** options after touchdown.

Everything talks over `rednet`, so the airframe is just: a few "brain" computers that read sensors and decide, and a few "dumb" relay computers that turn rednet messages into redstone for the thrusters.

---

## 1. Requirements

**Mods**

- **CC: Tweaked** — the computers.
- **Create** and **Create: Aeronautics** — the aircraft and its gas thrusters (throttled by a redstone signal 0–15).
- **CC-Sable** (TechTastic) — provides the `sublevel` and `aero` APIs: the ship's live position, orientation, velocity and inertia tensor. The "brain" computers must be *part of the flying contraption* for this to work.
- **Advanced-Math** (TechTastic) — provides the `quaternion`, `vector` and `matrix` globals and the `pid` module the UAC math is built on.

**Blocks you'll place on the aircraft**

- 6 computers (Advanced Computers recommended — the UIs use colour and touch).
- 2 Advanced Monitors (for the FMS's VOR/HSI and the Altitude Selector's attitude indicator). Any size ≥ ~3×3 blocks reads well.
- A **wireless (or ender) modem** on every computer — the whole system is one rednet network.
- Redstone links from the three relay computers to your thruster banks.

---

## 2. How it's put together

Six computers. Three of them read sensors and think ("masters"); three just copy rednet numbers onto redstone ("relays").

```
                 ┌───────────────────────┐
   sensors  ───► │  UNIFIED ATTITUDE      │ ──aertogekiss_stab──────►  Stabiliser relay ─► pitch/roll thrusters
  (sublevel)     │  CONTROLLER  (UAC)     │ ──aertogekiss_navcontrol─► Yaw+Fore/aft relay ─► yaw + fore/aft
                 │  = the brain           │ ──aertogekiss_navcontrol─► Strafe+Alt relay  ─► strafe thrusters
                 └────▲───────▲───────▲───┘ ──aertogekiss_alt────────► Strafe+Alt relay  ─► lift thruster
                      │       │       │
        aertogekiss_navap  altap  autoland
                      │       │       │
      ┌───────────────┴─┐   ┌─┴───────┴──────────┐
      │ FLIGHT MGMT SYS │   │ ALTITUDE SELECTOR  │
      │ (FMS) routing + │   │ (ALT-AP) + attitude│
      │ VOR/HSI monitor │◄──┤ indicator monitor  │
      └─────────────────┘ altoverride            │
```

**rednet protocols** (all prefixed `aertogekiss_`):

| Protocol | From → To | Carries |
|----------|-----------|---------|
| `navap` | FMS → UAC | target X/Z, flight phase, hold-heading, cruise cap, or `{cancel}` |
| `altap` | ALT-AP → UAC | selected altitude `{alti}` |
| `autoland` | FMS → UAC (+ ALT-AP listens) | vertical cmd: `hold` / `descend`+vs / `landed` / `release` / `cancel` / `off` |
| `altoverride` | ALT-AP → FMS | "pilot tapped a panel mid-landing, wants ALT-AP" |
| `stab` | UAC → relay | `{left,right,front,back}` — pitch/roll |
| `navcontrol` | UAC → relays | `{thrust_forward,thrust_backward,yaw_left,yaw_right,strafe_left,strafe_right}` |
| `alt` | UAC → relay | `{alt}` — lift, 0..14, 7 = neutral |

All thruster values are **0–14** redstone steps. Altitude is centred on **7** (7 = hold, 14 = full up, 0 = full down); the other channels are one-directional pairs.

---

## 3. Installation

For each of the six folders in this package, put its contents on one computer (its own `startup.lua` auto-launches it on boot). It doesn't matter what computer *IDs* Minecraft gives them — only that they all share one rednet network and have the right peripherals.

The quickest way to copy files onto a CC computer is `wget`/`pastebin`, or edit them in-game, or (on a server) drop them into the computer's folder under `world/computercraft/computer/<id>/`.

| Put this folder's contents … | … on a computer that has |
|---|---|
| `attitude-controller/` (→ `startup.lua`, `pid/`) | modem; is part of the aircraft (Sable). No monitor needed. |
| `flight-management-system/` | modem; a monitor; part of the aircraft; keep `points.db`/`course.db`/`landing.db` alongside `autopilot.lua`. |
| `altitude-selector/` | modem; a monitor; part of the aircraft. |
| `thruster-relay-stabiliser/` | modem on **top**; redstone to the pitch/roll thruster bank. |
| `thruster-relay-yaw-thrust/` | modem on **top**; redstone to the yaw + fore/aft banks. |
| `thruster-relay-strafe-altitude/` | modem on **top**; redstone to the strafe + lift banks. |

Then **open the modem** on each machine (the code does `rednet.open(...)` on the side shown — the relays expect the modem on `top`; the masters auto-find any modem). Reboot each computer; the relays should print "…controller" and the masters their UIs.

> The `.db` files are **example** navigation data. You can edit waypoints/routes live from the FMS touch screen (Section 6), so a fresh install can start from these or you can clear them.

---

## 4. Wiring the thrusters

The three relay computers each `rednet.open("top")` and copy message fields onto their **own** redstone sides with `redstone.setAnalogOutput`. Wire those sides to the redstone inputs of the matching Create: Aeronautics thruster banks.

**Stabiliser relay** — listens `aertogekiss_stab`:

| message field | redstone side | drives |
|---|---|---|
| `left`  | left  | one pitch-or-roll direction |
| `right` | right | the opposite |
| `front` | front | the other axis, one direction |
| `back`  | back  | the opposite |

**Yaw + fore/aft relay** — listens `aertogekiss_navcontrol`:

| field | side | drives |
|---|---|---|
| `yaw_left`        | left  | yaw |
| `yaw_right`       | right | yaw |
| `thrust_forward`  | front | fore/aft |
| `thrust_backward` | back  | fore/aft |

**Strafe + altitude relay** — listens `aertogekiss_navcontrol` **and** `aertogekiss_alt`:

| field | side | drives |
|---|---|---|
| `strafe_left`  | left  | strafe |
| `strafe_right` | right | strafe |
| `alt`          | front | lift (0..14, 7 = hover) |

You do **not** need to get the pitch-vs-roll or the forward-vs-backward assignment "right" physically — the calibration step (next) has sign/swap switches that sort out any mirrored wiring in software.

---

## 5. First boot & calibration (do this once, on the ground / tethered)

Every airframe is different, so a handful of **sign** switches must be set once. They all live in the `CFG` table at the top of **`attitude-controller/pid/uac.lua`**. Edit, save, reboot the UAC, test. The golden rule: **if an axis diverges/blows up rather than oscillating, it's a sign or frame error — not a gain.**

There is a throwaway helper, **`pid/probe.lua`**, you can `probe` (run) on the UAC computer first: it prints the quaternion API, the inertia-tensor shape, and lets you hand-rotate the craft to see the angular-velocity frame.

Recommended order (all switches are in `CFG`):

1. **Sensor frame** — `CFG.SENSOR_FRAME`. Leave `"body"` (correct for this airframe family). Symptom of wrong: levels fine near heading 0 but wanders once yawed ~90°.
2. **Levelling** — hand-tilt the craft; it must right itself. If a nose-up disturbance is answered nose-*up* (it diverges), flip `CFG.PITCH_SIGN`; same for `CFG.ROLL_SIGN`. If a *pitch* command makes the craft *roll* (channels crossed), set `CFG.SWAP_PITCH_ROLL_CHANNELS = true`.
3. **Rate signs** — `CFG.PITCH_RATE_SIGN` / `YAW_RATE_SIGN` / `ROLL_RATE_SIGN`. If an axis is calm at rest but **explodes the moment you nudge it**, its rate feedback is anti-damping — flip that axis's rate sign. (These are independent of step 2; the orientation and the angular-velocity sensor are different sources.)
4. **Invert test** — flip it upside-down; it must roll/pitch back to level. (Quaternion shortest-path recovery.)
5. **Output scale** — `CFG.ROT_GAIN_PITCH/ROLL/YAW`. INERTIA mode outputs a physical torque, and this craft's inertia is huge, so these are tiny (~`5e-6`). On the UAC screen read `raw` at a firm correction and set `ROT_GAIN ≈ 7 / raw`.
6. **Yaw** — command a heading off to one side (via the FMS): the nose must swing *toward* it and settle *on* it. If it runs away, flip `CFG.YAW_SIGN`. If it *overshoots* the heading, lower `CFG.MAX_DECEL_YAW` (brakes earlier).
7. **Translation** — fly toward a point: it must *close* the range. If it runs off a course line, flip `CFG.STRAFE_SIGN`; if it flies away from the target, flip `CFG.THRUST_SIGN`.
8. **Altitude** — the lift channel *learns its own hover trim*; just tap an altitude on the ALT-AP panel and confirm it climbs to it and holds.

That's it — after this the aircraft flies. The tuning knobs (speed, damping, etc.) in Section 7 are optional polish.

---

## 6. Using the system

### 6.1 Altitude Selector (ALT-AP) — the four-panel keypad

Four altitude presets. **Tap the ALT button** under a panel to select that altitude — it goes red and the aircraft climbs/descends to it and holds. Type a new number on a panel's keypad and press **Ent** to change it. On boot **nothing is selected** (all green) and the craft simply **holds its current altitude** until you pick one.

The external **monitor** on this computer is the **Attitude Indicator**: an artificial horizon (blue sky / brown ground, tilting with roll, shifting with pitch), with **GS** (ground speed), **ALT** (altitude) and **VS** (vertical speed) read out in the corners.

### 6.2 Flight Management System (FMS) — the touch screen

This is your navigator. From its screen you can:

- **Edit data**: create **POINTS** (waypoints — POIs and VORs, with X/Z coordinates) and **COURSES** (named sequences of points). A point marked as a landing site shows a `<H>`/etc. tag.
- **Fly DIRECT** to any point.
- **Fly a ROUTE** (course): it sequences the waypoints, holds the straight leg between each pair with cross-track guidance, and decides the flight phase (ENROUTE → APPROACH → TERMINAL → ARRIVED) from the plan itself.
- **SPD** button: cycle the cruise-speed cap **FULL → SLOW → COAST**.
- **HOME / CANCEL**: stop the autopilot and hand control back.

The external **monitor** on this computer is the **VOR / HSI**: a heading-up compass rose (the "squarised circle"), a **course line** for the active leg, a green/yellow **course-deviation needle** (how far you are off the course, left/right), and the **heading number** — plus DTK (desired track), XTK (cross-track metres, L/R) and distance-to-go.

### 6.3 Automatic landing

Landing sites live in the landing data (a POINT marked as a pad, with an approach direction and pad altitude). Two ways in:

- Fly a **route that ends at a landing site** — it auto-lands on arrival.
- Use the **LAND** menu to go direct-to a site and land at the end.

Then the FMS runs the sequence automatically: **ALIGN** (swing to the pad heading, holding position) → **TRANSLATE** (close the last few metres precisely) → **DESCEND** → **FLARE** → **LANDED**. Throughout, the UAC holds the craft over the pad and on the landing heading while flying the commanded descent rate. The FMS landing screen shows POS/HDG error, AGL and VS.

While it's flying itself down, the **Altitude Selector shows all-green** (it's "overshadowed" — ALT-AP is off while autoland owns the vertical channel).

### 6.4 Pilot control during & after landing

- **ABORT** (button, mid-descent): stops the landing, climbs away on ALT-AP as a go-around.
- **Pilot override — pause the descent**: while it's descending, **tap any panel on the Altitude Selector**. The FMS **pauses the descent** and hands the vertical channel back to ALT-AP (the craft holds/flies your selected altitude) **while keeping position and heading alignment over the pad**. The FMS shows **RESUME LND** (resume the descent — the panel goes all-green again) and **ABORT**.
- **After touchdown** the FMS offers two options:
  - **RESUME ALT-AP** — leave the pad and climb to the ALT-AP cruise altitude.
  - **AP OFF** — park on the ground, autopilot off (the craft just holds the ground). Re-arm later by tapping a panel on the Altitude Selector.

### 6.5 The UAC's own screen

The Unified Attitude Controller's computer screen is a live **artificial horizon + telemetry**: pitch/roll, heading vs. desired, the per-channel thruster outputs, the `raw` torque magnitude (for tuning), altitude and vertical speed, the trim integrators, and the body-diagonal inertia moments. Handy while calibrating.

---

## 7. Tuning reference (`CFG` in `pid/uac.lua`)

All optional once the signs (Section 5) are right. Numbers are the shipped defaults for the *Aer Togekiss*.

| Knob | What it does |
|---|---|
| `ATT_MODE` | `"INERTIA"` (full inertia/gyro loop) or `"PD"` (simple, tunes like an old stabiliser). |
| `ATT_KP` / `RATE_KP` | outer (error→rate) and inner (rate→accel) gains. **Underdamped + sluggish → raise `RATE_KP`** (it speeds up *and* damps). Overshoots → lower `ATT_KP`. Buzzes → over-gained, back off. |
| `ROT_GAIN_*` | steps per N·m per axis (INERTIA). `≈ 7 / raw`. Pitch/roll and yaw use different thrusters, tune separately. |
| `MAX_RATE_YAW` / `MAX_DECEL_YAW` | yaw motion profile. High-inertia yaw → keep the rate low; if it still overshoots, lower `MAX_DECEL_YAW` (brakes sooner). |
| `GYRO_COMPENSATION` + `GYRO_FADE_LO/HI` | axis decoupling. Helps at gentle yaw, so it's faded out above a yaw rate where its 50 ms lag would pump a nutation. |
| `RATE_KP_YAW` | yaw-only inner gain — crisper yaw without over-gaining pitch/roll. |
| `YAW_TO_PITCH_FF` / `YAW_TO_ROLL_FF` | feed-forward cancel of off-CoM yaw-thrust coupling into pitch/roll (calibrate against a steady yaw). |
| `ATT_KI` / `ATT_I_*` | integral trim (removes a standing tilt from a CoM offset); rate-gated so it can't pump a manoeuvre. |
| `TILT_YAW_MIN/MAX` | **level priority** — fades yaw out as the craft tilts, so a yaw manoeuvre can never roll it over faster than levelling recovers. |
| `YAW_TILT_DECOMP` | split attitude error into tilt + yaw (avoids a pitch kick on ~180° turns). |
| `TILT_ALT_ALLOC` + `VERT_XFER_*` | attitude-aware vertical thrust (hold altitude inverted / at 90° pitch). |
| `ENABLE_ATTITUDE/TRANSLATION/ALTITUDE` | bring channels up one at a time during first setup. |

---

## 8. Troubleshooting

| Symptom | Likely cause / fix |
|---|---|
| Craft **diverges/flips** instead of oscillating | a **sign or frame** error, not a gain. Re-check Section 5 (levelling signs, then rate signs). |
| Stable at rest but **explodes the instant it's nudged** | a **rate sign** is anti-damping — flip that axis's `*_RATE_SIGN`. |
| Levels near heading 0 but **wanders once yawed** | wrong `CFG.SENSOR_FRAME`. |
| **Pitches up during a ~180° turn** | set `CFG.YAW_TILT_DECOMP = true`. |
| **Rolls when it shouldn't** during a manoeuvre | off-diagonal inertia coupling — keep `CFG.INERTIA_COUPLING = false` (diagonal, decoupled). |
| Yaw **overshoots** the heading | lower `CFG.MAX_DECEL_YAW`. |
| Nutation/wobble that **grows on a sustained hard yaw** | the gyro feed-forward lag — lower `CFG.GYRO_FADE_HI` and/or `CFG.MAX_RATE_YAW`. |
| Loses altitude when **banked/pitched hard or inverted** | ensure `CFG.TILT_ALT_ALLOC = true`; calibrate `VERT_XFER_FWD/STR` signs. |
| **Lazy ~200 m spiral** on approach that never closes | (fixed in this build) — the direct-to path now strafes toward the target instead of relying on yaw. |
| UAC screen shows the whole loop dead / `raw` in the millions but output ~1 | check the modem is open and the relays are running; the vector-rotation/scale bugs are fixed in this build. |
| `function ... has more than 200 local variables` on load | you added a top-level `local`; put new tuning constants in the `CFG` table (`CFG.NAME = ...`), not as a bare `local`. |
| Attitude Indicator shows **"NO SUBLEVEL API"** | that computer isn't part of the Sable-tracked contraption — it must be built into the aircraft. |
| VOR/HSI heading reads wrong (rotated/mirrored) | see the one compass-conversion line in `drawHSI` (autopilot.lua); the CDI deflection sign is a one-character flip if it points the wrong way. |
| Two computers fight over the thrusters | only **one** controller may own `aertogekiss_navcontrol`/`stab`/`alt`. Don't run the retired `navcom`/`control` alongside the UAC. |

---

*Fly safe. — Aer Togekiss Flight System*
