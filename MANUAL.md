# Aer Togekiss Flight System — User's Manual

> A quaternion fly-by-wire autopilot for a Create: Aeronautics VTOL, on CC: Tweaked computers.

---

## 0. Introduction — what this is

This is a complete **autopilot and flight-control stack** for a Create: Aeronautics VTOL aircraft. Once installed and calibrated, it will:

- keep the aircraft **level and pointed** where you want it, automatically, at all times;
- **hold a selected altitude**, or let you fly the vertical by hand on a **vertical-speed stick**;
- **navigate** to a single point (direct-to) or fly a **route** of waypoints as one **smooth computed curve** — rounding the corners and setting its own speed for each bend;
- with the autopilot off, **hand-fly the horizontal and yaw** on a **nav stick** (fore/aft, strafe, yaw) — together with the vertical stick that's full manual fly-by-wire, and being independent of the autopilot it doubles as an emergency course-alteration override;
- **land itself** on a marked pad — line up, translate over the pad, descend, flare, touch down;
- give you a **cockpit of physical switches** — engine cut-offs (master / altitude / attitude) and autopilot toggles (nav / altitude) that pick the flight mode live;
- show you a **glass cockpit** — an artificial horizon and a VOR/HSI (that becomes a top-down map on landing).

**The heart of it is the Unified Attitude Controller (UAC).** A Create: Aeronautics craft is flown by lots of little gas thrusters — some for pitch/roll, some for yaw, some for fore/aft and strafe, some for lift. Older setups stabilised these with a *gimbal peripheral* and Euler angles, which can't cleanly recover an inverted craft and needs extra hardware. The UAC throws that away: it reads the ship's **true orientation quaternion, angular velocity and inertia tensor** straight off the physics engine (via CC-Sable) at 20 Hz, and runs **one control loop that owns every thruster**. Because it works in quaternions it has no gimbal lock, no ±180° wrap seams, and it recovers from *any* attitude — including fully upside-down — along the shortest path.

On top of that core it does a lot of things a real flight computer does:

- an **inner rate loop with inertia-tensor scaling and gyroscopic decoupling**, so a heavy, high-inertia airframe is controlled properly rather than just damped;
- a **motion-profiled yaw** that starts braking a turn *early* so the nose doesn't overshoot the heading;
- a **trajectory-planning FMS** — rather than chase waypoints one at a time and slow at each, it fits a **smooth curve** through the whole route, flies an aim-point along it (pure pursuit), and derives its **speed from the curve's curvature** so it holds cruise on the straights and eases through the bends, braking *in advance*;
- **holonomic navigation** — since a VTOL can slide sideways, it *strafes* toward a target rather than waiting for the slow yaw axis to swing the nose around (which otherwise causes a lazy spiral that never closes);
- **attitude-aware vertical thrust** — "hold altitude" is a *world-up* goal, so if the craft is banked, pitched or inverted, the vertical effort is re-routed to whichever thrusters are actually pointing up;
- an **automatic-landing state machine**, with a **pilot override** to pause the descent mid-approach and hand the vertical channel back to you (keeping the alignment), plus **AP OFF / RESUME ALT-AP** options after touchdown;
- a **four-mode flight-mode layer** driven by the two autopilot toggle switches — *levelling is always on*, and on top of it you choose per axis: **navigate** or hand the position to yourself, **hold altitude** or fly the vertical stick by hand (see §6.6).

Everything talks over `rednet`, so the airframe is just: a few "brain" computers that read sensors and decide, a few "dumb" relay computers that turn rednet messages into redstone for the thrusters, and a few cockpit panels that broadcast the pilot's switches and stick.

---

## 1. Requirements

**Mods**

- **CC: Tweaked** — the computers.
- **Create** and **Create: Aeronautics** — the aircraft and its gas thrusters (throttled by a redstone signal 0–15).
- **CC-Sable** (TechTastic) — provides the `sublevel` and `aero` APIs: the ship's live position, orientation, velocity and inertia tensor. The "brain" computers must be *part of the flying contraption* for this to work.
- **Advanced-Math** (TechTastic) — provides the `quaternion`, `vector` and `matrix` globals and the `pid` module the UAC math is built on.

**Blocks you'll place on the aircraft**

- 12 computers (Advanced Computers recommended — the UIs use colour and touch): 3 brains, 3 relays, 6 cockpit panels/sticks. *(The last 3 — the nav stick — are optional if you only want autopilot flight.)*
- 2 Advanced Monitors (for the FMS's HSI map and the Altitude Selector's attitude indicator). Any size ≥ ~3×3 blocks reads well.
- A **wireless (NOT ender) modem** on every computer — the whole system is one rednet network.
- Redstone links from the three relay computers to your thruster banks.
- Cockpit redstone in:
  - **5 switches/levers** — 3 engine cut-offs → engine-switch computer, 2 autopilot toggles → autopilot-switch computer.
  - **1 analog lever** — the vertical-speed stick → the VS-commander computer (an Analog Lever / redstone-integrator gives its 0–14 range; a plain lever is fine for the on/off switches).
  - **6 momentary buttons** for the nav stick — 4 (fore/aft/strafe-left/strafe-right) → the MANCON translation relay, 2 (yaw-left/right) → the MANCON rotation relay. Buttons/keys, not latching levers: hold to move, release to stop.

---

## 2. How it's put together

Twelve computers: three read sensors and think ("brains"), three copy rednet numbers onto redstone ("relays"), and six read the cockpit and broadcast it ("panels/sticks", the last three being the nav stick).

```
  COCKPIT PANELS / STICKS (read switches & keys, broadcast state)
    engine-switch     ─ engineswitch {master,altitude,attitude} ─►  the 3 RELAYS  (hardware enable)
    autopilot-switch  ─ apswitch {nav, altitude} ────────────────►  the UAC       (flight mode)
    VS-commander      ─ mancon {vs}  (wanted climb rate, m/s) ─────►  the UAC       (manual altitude)
    NAV STICK: 2 relays ─ mancon_translational / mancon_rotational ─► MIC
                 MIC ─ mancon_nav {fwd, strafe (m/s), yaw (rad/s)} ──►  the UAC     (manual position + heading)

  BRAIN → RELAYS → THRUSTERS
                 ┌───────────────────────┐
   sensors  ───► │  UNIFIED ATTITUDE      │ ──stab──────────►  Stabiliser relay  ─► pitch/roll thrusters
  (sublevel)     │  CONTROLLER  (UAC)     │ ──navcontrol────►  Yaw+Fore/aft relay ─► yaw + fore/aft
                 │  = the brain           │ ──navcontrol────►  Strafe+Alt relay   ─► strafe thrusters
                 └───▲──────▲──────▲──────┘ ──alt {up,down}─►  Strafe+Alt relay   ─► up/down lift thrusters
                     │      │      │
                  navap   altap  autoland
                     │      │      │
      ┌──────────────┴─┐  ┌─┴──────┴───────────┐
      │ FLIGHT MGMT    │  │ ALTITUDE SELECTOR  │
      │ SYS (FMS) +    │  │ (ALT-AP) + attitude│
      │ VOR/HSI monitor│◄─┤ indicator monitor  │
      └────────────────┘ altoverride           │
```

**rednet protocols** (all prefixed `aertogekiss_`):

| Protocol | From → To | Carries |
|----------|-----------|---------|
| `navap` | FMS → UAC | target X/Z, flight phase, hold-heading, cruise cap, or `{cancel}` |
| `altap` | ALT-AP → UAC | selected altitude `{alti}` |
| `autoland` | FMS → UAC (+ ALT-AP listens) | vertical cmd: `hold` / `descend`+vs / `landed` / `release` / `cancel` / `off` |
| `altoverride` | ALT-AP → FMS | "pilot tapped a panel mid-landing, wants ALT-AP" |
| `apswitch` | AP panel → UAC | `{autopilot_nav, autopilot_altitude}` booleans — the flight mode (§6.6) |
| `engineswitch` | Engine panel → relays | `{master, altitude, attitude}` booleans — hardware thruster enables |
| `mancon` | VS-commander → UAC | `{vs}` — pilot's wanted climb rate (m/s), flown when altitude-hold is off |
| `mancon_translational` | translation relay → MIC | `{fore, aft, sleft, sright}` booleans — the pilot's fore/aft/strafe keys |
| `mancon_rotational` | rotation relay → MIC | `{yleft, yright}` booleans — the pilot's yaw keys |
| `mancon_nav` | MIC → UAC | `{fwd, strafe (m/s), yaw (rad/s), active}` — wanted body velocity, flown when nav is off |
| `stab` | UAC → relay | `{left,right,front,back}` — pitch/roll |
| `navcontrol` | UAC → relays | `{thrust_forward,thrust_backward,yaw_left,yaw_right,strafe_left,strafe_right}` |
| `alt` | UAC → relay | `{up, down}` — up/down lift thrusters, 0..15, 7/7 = balanced hover |

All thruster values are **0–15** redstone steps. The lift channel is now **two lines** — an up-firing and a down-firing thruster group, balanced at **7/7** for hover (climb raises `up` and backs off `down`, descent the reverse); every other channel is a one-directional pair (one side or the other, 0 at rest). *(An earlier build sent a single 0..14 `alt` value that an external redstone contraption split into up/down — that box is gone; the UAC drives the two lines directly.)*

**Two layers of switch — don't confuse them.** The **engine** switches are hardware cut-offs read by the *relays*: with `master` off nothing moves at all; `attitude` powers the pitch/roll bank; `altitude` powers the lift bank. The **autopilot** switches are read by the *UAC* and choose how it flies (navigate vs. hand-off; hold-altitude vs. manual stick). For normal autopilot flight you want the engine switches **on** (thrusters live) *and* the autopilot switches set to the mode you want.

---

## 3. Installation

For each of the twelve folders in this package, put its contents on one computer (its own `startup.lua` auto-launches it on boot). It doesn't matter what computer *IDs* Minecraft gives them — only that they all share one rednet network and have the right peripherals. *(The three `mancon-*` folders are the optional nav stick; skip them if you only want autopilot flight.)*

The quickest way to copy files onto a CC computer is `wget`/`pastebin`, or edit them in-game, or (on a server) drop them into the computer's folder under `world/computercraft/computer/<id>/`.

| Put this folder's contents … | … on a computer that has |
|---|---|
| `attitude-controller/` (→ `startup.lua`, `pid/`) | modem; is part of the aircraft (Sable). No monitor needed. |
| `flight-management-system/` | modem; a monitor; part of the aircraft; keep `points.db`/`course.db`/`landing.db` alongside `autopilot.lua`. |
| `altitude-selector/` | modem; a monitor; part of the aircraft. |
| `thruster-relay-stabiliser/` | modem on **top**; redstone to the pitch/roll thruster bank. |
| `thruster-relay-yaw-thrust/` | modem on **top**; redstone to the yaw + fore/aft banks. |
| `thruster-relay-strafe-altitude/` | modem on **top**; redstone to the strafe + up/down lift banks. |
| `vertical-speed-commander/` | modem (any side, **not** `top` — the stick uses `top`); analog lever into `top`. |
| `engine-switch/` | modem on **top**; 3 redstone switches into `front`/`right`/`left`. |
| `autopilot-switch/` | modem on **top**; 2 redstone switches into `right`/`left`. |
| `mancon-translation-relay/` | modem on **top**; 4 momentary buttons into `back`/`front`/`left`/`right` (fore/aft/strafe-L/strafe-R). |
| `mancon-rotation-relay/` | modem on **top**; 2 momentary buttons into `left`/`right` (yaw-L/yaw-R). |
| `mancon-intention-controller/` | modem (any side). No redstone, no monitor — it just listens and re-broadcasts. |

Then **open the modem** on each machine (the code does `rednet.open(...)` on the side shown — the relays and switch panels expect the modem on `top`; the UAC, FMS and VS-commander auto-find any modem). Reboot each computer; the relays and panels print their titles/state and the brains their UIs.

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
| `up`           | front | **up-firing** lift thrusters (0..15) |
| `down`         | back  | **down-firing** lift thrusters (0..15) |

The lift is two lines now, not one — `up` and `down` are `UP_SIDE`/`DOWN_SIDE` constants at the top of `strafecon.lua`; rewire there if your banks sit elsewhere. At hover both read ~7; a climb drives `up` toward 15 and `down` toward 0. (If you're migrating from the old single-line lift, delete the external splitter contraption — the relay does the split now.)

You do **not** need to get the pitch-vs-roll or the forward-vs-backward assignment "right" physically — the calibration step (next) has sign/swap switches that sort out any mirrored wiring in software.

### 4.1 Cockpit inputs (switches & the vertical-speed stick)

Three more computers read redstone *in* from the cockpit and broadcast it. These drive nothing directly.

**Engine switches** (`engine-switch/`, modem on `top`) — three on/off levers, broadcast on `aertogekiss_engineswitch`. These are **hardware cut-offs the relays obey**: a relay only energises its thrusters when its switch (and the master) is on.

| input side | switch | effect when off |
|---|---|---|
| `front` | **master** | all relays hold output — the whole craft is dead-stick |
| `right` | **altitude** | the lift (up/down) relay output is cut |
| `left`  | **attitude** | the stabiliser (pitch/roll) relay output is cut |

**Autopilot switches** (`autopilot-switch/`, modem on `top`) — two on/off levers, broadcast on `aertogekiss_apswitch`. These pick the **UAC's flight mode** (§6.6); they do not gate hardware.

| input side | switch | on = |
|---|---|---|
| `right` | **altitude** | autopilot holds a selected altitude; off = fly the vertical stick by hand |
| `left`  | **nav** | autopilot navigates to the target; off = position handed to the pilot |

**Vertical-speed stick** (`vertical-speed-commander/`, modem on any side *except* `top`) — one **analog** input on `top` (0–14, 7 = hold), broadcast as a wanted climb rate on `aertogekiss_mancon`. Use an Analog Lever or a redstone integrator so you get the full 0–14 swing. It is only obeyed when the autopilot **altitude** switch is off. **Wire the stick so a cut/absent signal reads 7** (0 would command a full-rate descent).

**Nav stick** (`mancon-translation-relay/` + `mancon-rotation-relay/` + `mancon-intention-controller/`) — the horizontal/yaw equivalent of the VS stick, built from three computers. The two relays each `rednet.open("top")` and read **momentary buttons** (hold to move, release to stop):

| relay | input side | key |
|---|---|---|
| translation | `back`  | fore (forward) |
| translation | `front` | aft (backward) |
| translation | `left`  | strafe left |
| translation | `right` | strafe right |
| rotation | `left`  | yaw left |
| rotation | `right` | yaw right |

The **MIC** (intention controller) listens to both relays, integrates the key-presses into a wanted body velocity — holding a key ramps that axis up, releasing fades it to 0 — and broadcasts it on `aertogekiss_mancon_nav`. The UAC flies it **whenever the autopilot nav switch is off**: with nav off you steer the aircraft directly, and with a key released the UAC holds heading and nulls drift. The two relays only transmit while the engine **master** is on *and* the autopilot **nav** switch is off, so the stick is silent (and the autopilot has full control) during automatic flight.

---

## 5. First boot & calibration (do this once, on the ground / tethered)

Every airframe is different, so a handful of **sign** switches must be set once. They all live in the `CFG` table at the top of **`attitude-controller/pid/uac.lua`**. Edit, save, reboot the UAC, test. The golden rule: **if an axis diverges/blows up rather than oscillating, it's a sign or frame error — not a gain.**

> **Before anything responds:** the engine **master** switch (plus the relevant **attitude** / **altitude** engine switch) must be **on**, or the relays hold their output and the craft won't move no matter what the UAC computes. Levelling is always active regardless of the autopilot switches, so the attitude steps below work with the AP switches in any position.

There is a throwaway helper, **`pid/probe.lua`**, you can `probe` (run) on the UAC computer first: it prints the quaternion API, the inertia-tensor shape, and lets you hand-rotate the craft to see the angular-velocity frame.

Recommended order (all switches are in `CFG`):

1. **Sensor frame** — `CFG.SENSOR_FRAME`. Leave `"body"` (correct for this airframe family). Symptom of wrong: levels fine near heading 0 but wanders once yawed ~90°.
2. **Levelling** — hand-tilt the craft; it must right itself. If a nose-up disturbance is answered nose-*up* (it diverges), flip `CFG.PITCH_SIGN`; same for `CFG.ROLL_SIGN`. If a *pitch* command makes the craft *roll* (channels crossed), set `CFG.SWAP_PITCH_ROLL_CHANNELS = true`.
3. **Rate signs** — `CFG.PITCH_RATE_SIGN` / `YAW_RATE_SIGN` / `ROLL_RATE_SIGN`. If an axis is calm at rest but **explodes the moment you nudge it**, its rate feedback is anti-damping — flip that axis's rate sign. (These are independent of step 2; the orientation and the angular-velocity sensor are different sources.)
4. **Invert test** — flip it upside-down; it must roll/pitch back to level. (Quaternion shortest-path recovery.)
5. **Output scale** — `CFG.ROT_GAIN_PITCH/ROLL/YAW`. INERTIA mode outputs a physical torque, and this craft's inertia is huge, so these are tiny (~`5e-6`). On the UAC screen read `raw` at a firm correction and set `ROT_GAIN ≈ 7 / raw`.
6. **Yaw** — command a heading off to one side (via the FMS): the nose must swing *toward* it and settle *on* it. If it runs away, flip `CFG.YAW_SIGN`. If it *overshoots* the heading, lower `CFG.MAX_DECEL_YAW` (brakes earlier).
7. **Translation** — fly toward a point: it must *close* the range. If it runs off a course line, flip `CFG.STRAFE_SIGN`; if it flies away from the target, flip `CFG.THRUST_SIGN`.
8. **Altitude** — turn on the engine **master** + **altitude** switches and the autopilot **altitude** switch. The lift channel *learns its own hover trim*, so just tap an altitude on the ALT-AP panel and confirm it climbs to it and holds. If it drives the *wrong* way (sinks when it should climb), your up/down lift banks are swapped — swap the wiring, or the `UP_SIDE`/`DOWN_SIDE` constants in `strafecon.lua`. Then flip the autopilot **altitude** switch *off* and check the **vertical-speed stick**: up = climb, centre (7) = hold, down = descend.

That's it — after this the aircraft flies. The tuning knobs (speed, damping, etc.) in Section 7 are optional polish.

---

## 6. Using the system

### 6.1 Altitude Selector (ALT-AP) — the four-panel keypad

Four altitude presets. **Tap the ALT button** under a panel to select that altitude — it goes red and the aircraft climbs/descends to it and holds. Type a new number on a panel's keypad and press **Ent** to change it. On boot **nothing is selected** (all green) and the craft simply **holds its current altitude** until you pick one.

The external **monitor** on this computer is the **Attitude Indicator**: an artificial horizon (blue sky / brown ground, tilting with roll, shifting with pitch), with **GS** (ground speed), **ALT** (altitude) and **VS** (vertical speed) read out in the corners. Around it is a thin **bezel with a bank pointer** (orange) that rides a fixed scale (ticks at 0 / ±30 / ±60°) — like a real AI, it reads *roll only* and never moves with pitch, so bank stays readable even in a steep climb or dive when the horizon has slid off-screen.

### 6.2 Flight Management System (FMS) — the touch screen

This is your navigator. From its screen you can:

- **Edit data**: create **POINTS** (waypoints — POIs and VORs, with X/Z coordinates) and **COURSES** (named sequences of points). A point marked as a landing site shows a `<H>`/etc. tag.
- **Fly DIRECT** to any point.
- **Fly a ROUTE** (course): it flies the waypoints as one **smooth curve** — a spline fitted through the fixes (corners rounded, no bearing snap) that it pure-pursues via a moving aim-point ("carrot") — and it sets its **speed from the curve's shape**: cruise on the straights, easing through tight bends and down to the destination, braking *in advance*. It still decides the flight phase (ENROUTE → APPROACH → TERMINAL → ARRIVED) from the plan, handing to a precise point-capture only for the final arrival/landing.
- **SPD** button: cycle the cruise-speed **ceiling** **FULL → SLOW → COAST** (the curve speed profile always stays under whatever you pick).
- **HOME / CANCEL**: stop the autopilot and hand control back.

The external **monitor** on this computer is the **HSI**, and it has **two modes**:

- **While navigating a course / direct-to** it's the classic **VOR / HSI**: a heading-up compass rose (the "squarised circle"), a **course line**, a green/yellow **course-deviation needle**, and the **heading number** — plus DTK (desired track), XTK (cross-track metres, L/R) and distance-to-go. On a curved route the course line and needle follow the **flown curve** — DTK is the curve's *tangent* and XTK your deviation *from the curve* — so the needle centres when you're tracking the curve, not the straight leg. Top-left **CMD** reads the commanded speed and top-right **GS** your ground speed.
- **Once a landing engages** (auto or manual) it switches to a **heading-up top-down map** for flying the approach or watching the autoland. Your aircraft sits fixed at the centre as a nose-up `^` while the compass card and the **pad** (`H`, with a short line marking its approach heading) rotate under it (dead ahead is the top). The scale auto-ranges (shown as `R__m`) so the pad creeps in as you close. The **top row is a heading-error bar** — a bright marker shows how far the pad's landing heading is off your nose, green when aligned. The **bottom row** shows the landing numbers: phase / AGL / VS / position error / heading error.

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

The Unified Attitude Controller's computer screen is a live **artificial horizon + telemetry**: pitch/roll, heading vs. desired, the per-channel thruster outputs, the `raw` torque magnitude (for tuning), altitude and vertical speed (the `ALT` line shows the two lift levels `u`/`d`), the trim integrators, and the body-diagonal inertia moments. The mode line reads `AP nav:on/off alt:on/off`; the phase shows **HANDOFF** when nav is handed to the pilot, and the altitude target shows **MAN** when you're flying the vertical stick. Handy while calibrating.

### 6.6 Cockpit switches & the vertical-speed stick

Two banks of physical switches, plus one stick, sit between you and the autopilot.

**Engine switches (master / altitude / attitude)** — the hardware cut-offs. `master` off = the whole craft is dead-stick (no thruster moves). `attitude` off = pitch/roll stabiliser off. `altitude` off = lift thrusters off. Leave all three **on** for normal flight; they're your kill switches and your way to isolate a bank.

**Autopilot switches (nav / altitude)** — these pick *how* the UAC flies. **Levelling is always on**; the two switches choose the rest, giving four modes:

| nav | altitude | The craft… |
|:---:|:---:|---|
| **on**  | **on**  | full autopilot — navigates the route/target **and** holds altitude, level throughout |
| **on**  | off | navigates + stays level, but **you** fly the vertical on the VS stick |
| off | **on**  | holds altitude + stays level, but **you** fly the position/heading on the **nav stick** |
| off | off | keeps the craft **level**; **you** fly everything — position/heading on the nav stick, vertical on the VS stick (full manual fly-by-wire) |

Flipping the **altitude** switch on grabs the *current* altitude to hold (a live ALT-AP panel selection then takes over within a second); flipping it off hands the vertical to the VS stick. Flipping the **nav** switch off hands the horizontal position and heading to the **nav stick** — with no key pressed the UAC just holds the current heading and station. An **automatic landing in progress overrides both switches** — it owns the craft until it finishes or you abort.

**Vertical-speed stick** — active whenever the autopilot **altitude** switch is off. Centre (7) holds level; push up to climb, down to descend, proportionally, up to `VS_MAX_CMD`. The UAC executes the rate through the *same* inner loop it uses for altitude-hold, so it's attitude-aware (it still lifts you straight up even nose-high) and it learns the hover trim as you fly. The stick only *asks* for a rate; the UAC clamps it to what the airframe can actually do (`CLIMB_MAX`/`DESCEND_MAX`).

**Nav stick** — active whenever the autopilot **nav** switch is off. The fore/aft/strafe/yaw keys command a *body velocity*: hold a key and the aircraft accelerates that way (up to the MIC's per-axis limit), release and it fades to a stop; with everything released the UAC holds heading and nulls drift, so it station-keeps. Because the keys are fed as a velocity into the UAC's own loops it flies steady and attitude-aware like the autopilot, and since it runs independently it's also your quick manual course-change override any time nav is off.

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
| `CLIMB_AT_FULL` / `CLIMB_MAX` / `DESCEND_MAX` | vertical-rate model. `CLIMB_AT_FULL` is the measured full-thrust climb rate; the two caps (autopilot *and* manual stick obey them) default to 0.75× it. The lift channel learns its own hover trim (`VEL_KI`), so leave that be. |
| `AP_NAV_DEFAULT` / `AP_ALT_DEFAULT` | assumed autopilot mode until the switch panel is first heard. Both `false` (safe: hand-off + manual) so a craft with no switch panel wired won't fly itself off. |
| `MANCON_STALE_AFTER` | manual-stick timeout — if the VS-commander goes quiet, the vertical falls back to a neutral hover. |
| `MANNAV_*` | the manual **nav** stick: `MANNAV_ENABLE`; `MANNAV_YAW_SIGN` (flip if a manual yaw command turns the wrong way); `MANNAV_STALE_AFTER` (nav-stick timeout → hold heading/station). |
| `ENABLE_ATTITUDE/TRANSLATION/ALTITUDE` | bring channels up one at a time during first setup (source-level debug isolators, independent of the AP switches). |

> The vertical-speed stick's full-scale rate is `VS_MAX_CMD` in **`vertical-speed-commander/vspdpid.lua`** (in m/s). Keep it at or just below the UAC's `CLIMB_MAX`, or the top of the stick travel just saturates against the UAC's cap.
>
> The **nav stick's** full-scale speeds are `V_FWD_MAX` / `V_STRAFE_MAX` (m/s) and `YAW_MAX` (rad/s) in **`mancon-intention-controller/mic.lua`**, with `RAMP_UP` / `YAW_RAMP_UP` setting how fast a held key builds up to them.

### 7.1 Trajectory tuning (`DC` table in `flight-management-system/autopilot.lua`)

How the FMS draws its **path** curve and **speed** curve. All optional; the defaults suit the *Aer Togekiss*.

| Knob | What it does |
|---|---|
| `CURVE` | `true` = fly the smoothed curve; `false` = aim straight at each waypoint (no rounding). |
| `CURVE_LOOKAHEAD` | how far ahead on the curve the aim-point ("carrot") sits, m. Larger = smoother / cuts corners more; smaller = tracks the fixes tighter. |
| `CURVE_DEV_MAX` | soft cap on how far the curve may bow off the straight leg, m — reins in **km-apart** waypoints without touching finely-placed ones (whose bow is already small). |
| `A_LAT_MAX` | cornering lateral accel for the curvature speed limit — speed through a bend of radius R is √(`A_LAT_MAX`·R). Higher = faster through bends (needs the strafe/bank authority to match). |
| `BRAKE_DECEL` | how hard the speed profile plans to brake, m/s². Sets how EARLY/short the slowdown into a bend or the destination is (braking distance ≈ (v²−v_target²)/(2·`BRAKE_DECEL`)). **Set it to what the craft can really brake** — too high and it arrives hot; too low and it crawls in from far out. |
| `ACCEL_MAX` / `DECEL_MAX` | rate limits on how fast the commanded speed may rise / fall, so it reads as a smooth curve, not a staircase. |
| `FINAL_ARRIVE_SPEED` | the speed the profile aims for AT the destination (the point-capture then stops it). Raise it to fly *through* a course without slowing at the last fix. |
| `SPEED_FULL` / `SPEED_SLOW` | the FULL / SLOW cruise-cap presets the **SPD** button cycles (the ceiling the curve speed stays under). |

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
| **Nothing moves at all**, UAC screen looks alive | engine **master** switch is off — the relays hold their output. Check the engine-switch panel (its screen shows `MST/ALT/ATT`). |
| **Pitch/roll dead** but yaw & lift work | engine **attitude** switch off. / **Lift dead** but the rest works | engine **altitude** switch off. |
| Autopilot **won't navigate** (phase shows `HANDOFF`) or **won't hold altitude** (target shows `MAN`) | the matching autopilot switch is off, or the autopilot-switch panel isn't broadcasting — check its screen (`AP NAV/AP ALT`) and modem. |
| **Vertical stick does nothing** | the autopilot **altitude** switch is on, so the stick is ignored (it's manual-only). Flip it off. If still nothing, the VS-commander has no modem, or the stick reads 0 (wire it to fail to 7). |
| **Nav stick does nothing** | the autopilot **nav** switch is on (stick ignored — it's manual-only), or the engine **master** is off (the relays only transmit while master is on and nav off). If still nothing, the MIC or a relay is missing its modem. |
| Manual **yaw turns the wrong way** on the nav stick | flip `CFG.MANNAV_YAW_SIGN` in `uac.lua`. (Fore/aft & strafe reuse the calibrated `THRUST_SIGN`/`STRAFE_SIGN`, so they follow from Section 5.) |
| Lift **drives the wrong way** (sinks on climb) | up/down banks swapped — swap the wiring or the `UP_SIDE`/`DOWN_SIDE` constants in `strafecon.lua`. |
| Autopilot **stays slow for much of a long straight leg** | `DC.BRAKE_DECEL` too low → the braking distance to the destination/bend is huge. Raise it (see §7.1). |
| Autopilot **arrives at a bend/destination too fast** | `DC.BRAKE_DECEL` higher than the craft can actually brake. Lower it so it starts slowing sooner. |
| The curve **swings noticeably off-course** between far-apart waypoints | lower `DC.CURVE_DEV_MAX` (bounds the bow) and/or `DC.CURVE_LOOKAHEAD`. |
| Craft **won't throttle up at route start** (sits at 0) | it's aligning — pointing the nose at the first fix before accelerating. It releases once within `ALIGN_HDG_DEG`; if it's stuck, the align target and the flown target disagree (check the FMS/UAC are the synced build). |
| **Lazy ~200 m spiral** on approach that never closes | (fixed in this build) — the direct-to path now strafes toward the target instead of relying on yaw. |
| UAC screen shows the whole loop dead / `raw` in the millions but output ~1 | check the modem is open and the relays are running; the vector-rotation/scale bugs are fixed in this build. |
| `function ... has more than 200 local variables` on load | you added a top-level `local`; put new tuning constants in a table — `CFG.NAME` in `uac.lua`, or the `DC` / `LG` / `RAD` tables in `autopilot.lua` — not as a bare `local`. (`luac -p` won't catch this; the VM counts a nested function's locals on top of the file's top-level ones.) |
| Attitude Indicator shows **"NO SUBLEVEL API"** | that computer isn't part of the Sable-tracked contraption — it must be built into the aircraft. |
| HSI heading/map reads rotated or mirrored | the compass conversion is the one `toCompass()` in autopilot.lua (shared by both HSI modes). Course mode: the **CDI needle** deflection is a one-character sign flip in `drawCourseHSI`. Landing mode: the heading-bar turn direction is the sign on `err` in `drawLandingHSI`. |
| AI bank pointer moves the **wrong way** | flip the roll sign on the pointer in `drawAI` (altcom.lua) — it's the one marked bank switch. |
| Two computers fight over the thrusters | only **one** controller may own `aertogekiss_navcontrol`/`stab`/`alt`. Don't run the retired `navcom`/`control` alongside the UAC. |

---

*Fly safe. — Aer Togekiss Flight System*
