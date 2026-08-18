# Aer Togekiss Flight System — User's Manual

> A quaternion fly-by-wire autopilot for a Create: Aeronautics VTOL, on CC: Tweaked computers.

---

## 0. Introduction — what this is

This is a complete **autopilot and flight-control stack** for a Create: Aeronautics VTOL aircraft. Once installed and calibrated, it will:

- keep the aircraft **level and pointed** where you want it, automatically, at all times;
- **hold a selected altitude**, or let you fly the vertical by hand on a **vertical-speed stick**;
- **navigate** to a single point (direct-to) or fly a **route** of waypoints as one **smooth computed curve** — rounding the corners and setting its own speed for each bend;
- with the autopilot off, **hand-fly the horizontal and yaw** on a **nav stick** (fore/aft, strafe, yaw) — together with the vertical stick that's full manual fly-by-wire, and being independent of the autopilot it doubles as an emergency course-alteration override; there is also an optional **keyboard joystick** (stick + throttle quadrant) for hand-flying FIXEDWING;
- **land itself** — onto a marked pad (line up, translate over it, descend, flare, touch down) or onto a **runway**, flying a straight-in approach down the extended centreline on a glideslope and braking along the strip, at a speed the runway's own length decides;
- give you a **cockpit of physical switches** — engine cut-offs (master / altitude / attitude) and autopilot toggles (nav / altitude) that pick the flight mode live;
- show you a **glass cockpit** — an artificial horizon and a VOR/HSI (that becomes a top-down approach map on landing);
- optionally take its waypoints from a **nav data server** on the ground, so a whole fleet's navigation database is maintained in one place instead of per aeroplane.

**The heart of it is the Unified Attitude Controller (UAC).** A Create: Aeronautics craft is flown by lots of little gas thrusters — some for pitch/roll, some for yaw, some for fore/aft and strafe, some for lift. Older setups stabilised these with a *gimbal peripheral* and Euler angles, which can't cleanly recover an inverted craft and needs extra hardware. The UAC throws that away: it reads the ship's **true orientation quaternion, angular velocity and inertia tensor** straight off the physics engine (via CC-Sable) at 20 Hz, and runs **one control loop that owns every thruster**. Because it works in quaternions it has no gimbal lock, no ±180° wrap seams, and it recovers from *any* attitude — including fully upside-down — along the shortest path.

On top of that core it does a lot of things a real flight computer does:

- an **inner rate loop with inertia-tensor scaling and gyroscopic decoupling**, so a heavy, high-inertia airframe is controlled properly rather than just damped;
- a **motion-profiled yaw** that starts braking a turn *early* so the nose doesn't overshoot the heading;
- a **trajectory-planning FMS** — rather than chase waypoints one at a time and slow at each, it fits a **smooth curve** through the whole route, flies an aim-point along it (pure pursuit), and derives its **speed from the curve's curvature** so it holds cruise on the straights and eases through the bends, braking *in advance*;
- **holonomic navigation** — since a VTOL can slide sideways, it *strafes* toward a target rather than waiting for the slow yaw axis to swing the nose around (which otherwise causes a lazy spiral that never closes);
- **attitude-aware vertical thrust** — "hold altitude" is a *world-up* goal, so if the craft is banked, pitched or inverted, the vertical effort is re-routed to whichever thrusters are actually pointing up;
- **two automatic-landing state machines** — a vertical descent onto a pad, and a glideslope approach onto a runway — with a **pilot override** to pause the descent mid-approach and hand the vertical channel back to you (keeping the alignment), plus **AP OFF / RESUME ALT-AP** options after touchdown. Both are written so they name only a point, a speed and a vertical speed, never an attitude, so the same approach flies identically in all three flight modes;
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

Twelve computers on the aircraft: three read sensors and think ("brains"), three copy rednet numbers onto redstone ("relays"), and six read the cockpit and broadcast it ("panels/sticks", three of those being the nav stick). Two optional extras sit outside that core: a **keyboard joystick** (another way to hand-fly) and a **nav data server**, which is not on the aircraft at all but on the ground, feeding waypoints to a whole fleet.

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
| `kbdjoy` | keyboard joystick → UAC | `{pitch, roll (rad), throttle 0..1, active}` — the hand-flying stick (§6.8) |
| `touchdown_detection` | rotation relay → FMS | `{touchedDown}` — the gear's squat switch, ground truth for a landing |
| `apswitch_1` | FMS → AP panel | the FMS re-announces its own AP-NAV switch input, so the panel and the UAC agree |
| `stab` | UAC → relay | `{left,right,front,back}` — pitch/roll |
| `navcontrol` | UAC → relays | `{thrust_forward,thrust_backward,yaw_left,yaw_right,strafe_left,strafe_right}` |
| `alt` | UAC → relay | `{up, down}` — up/down lift thrusters, 0..15, 7/7 = balanced hover |

One protocol is deliberately **not** prefixed, because it is shared by a whole fleet rather than
owned by one aeroplane: **`aer_navdata`**, the nav data server's broadcast (§6.9).

> **Flying more than one aeroplane?** The prefix is what keeps them apart. Change it in
> `pid/uacconf.lua` and `flight-management-system/apconf.lua` (and in the relay/panel programs,
> where the names are written out) so each airframe has its own — otherwise one FMS commands
> both aircraft. `aer_navdata` is the exception and should stay the same on all of them.

All thruster values are **0–15** redstone steps. The lift channel is now **two lines** — an up-firing and a down-firing thruster group, balanced at **7/7** for hover (climb raises `up` and backs off `down`, descent the reverse); every other channel is a one-directional pair (one side or the other, 0 at rest). *(An earlier build sent a single 0..14 `alt` value that an external redstone contraption split into up/down — that box is gone; the UAC drives the two lines directly.)*

**Two layers of switch — don't confuse them.** The **engine** switches are hardware cut-offs read by the *relays*: with `master` off nothing moves at all; `attitude` powers the pitch/roll bank; `altitude` powers the lift bank. The **autopilot** switches are read by the *UAC* and choose how it flies (navigate vs. hand-off; hold-altitude vs. manual stick). For normal autopilot flight you want the engine switches **on** (thrusters live) *and* the autopilot switches set to the mode you want.

---

## 3. Installation

For each of the folders in this package, put its contents on one computer (its own `startup.lua` auto-launches it on boot). It doesn't matter what computer *IDs* Minecraft gives them — only that they all share one rednet network and have the right peripherals. *(The three `mancon-*` folders are the optional nav stick; `keyboard-joystick/` and `nav-data-server/` are optional too. Skip any of them if you only want autopilot flight on a single aeroplane.)*

The quickest way to copy files onto a CC computer is `wget`/`pastebin`, or edit them in-game, or (on a server) drop them into the computer's folder under `world/computercraft/computer/<id>/`.

| Put this folder's contents … | … on a computer that has |
|---|---|
| `attitude-controller/` (→ `startup.lua`, `pid/`) | modem; is part of the aircraft (Sable). No monitor needed. |
| `flight-management-system/` | modem; a monitor; part of the aircraft; keep `apconf.lua` **and** `points.db`/`course.db`/`landing.db` alongside `autopilot.lua`. A second monitor is the HSI. |
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
| `keyboard-joystick/` *(optional)* | modem (any side); a **typewriter** peripheral; a monitor if you want the stick display (a single 1×1 block is enough — the panel is drawn for 15×10 cells). |
| `nav-data-server/` *(optional)* | modem, monitor, keyboard — **on the ground, one per fleet, NOT on an aircraft.** Keep its `points.db`/`course.db`/`landing.db`: these are the masters. |

Then **open the modem** on each machine (the code does `rednet.open(...)` on the side shown — the relays and switch panels expect the modem on `top`; the UAC, FMS and VS-commander auto-find any modem). Reboot each computer; the relays and panels print their titles/state and the brains their UIs.

> The `.db` files are **example** navigation data. You can edit waypoints/routes live from the FMS touch screen (Section 6), so a fresh install can start from these or you can clear them.

> **The two configuration files are yours, the programs are not.** Every sign, protocol name,
> speed, gain and profile lives in **`pid/uacconf.lua`** (attitude controller) and
> **`flight-management-system/apconf.lua`** (FMS). `uac.lua` and `autopilot.lua` hold only logic
> and carry **no fallback copies** of those values — deliberately, so that an edit to a config
> file can never be silently overridden by a stale default. Two consequences: **keep your two
> config files when you update a program**, and if one is missing or half-saved the computer
> stops at boot with a message naming the missing knob, instead of crashing in the air.

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

Every airframe is different, so a handful of **sign** switches must be set once. Almost all of them live in **`attitude-controller/pid/uacconf.lua`** — the calibration file, separate from the program so updating `uac.lua` never overwrites them. (The FMS has the same arrangement in **`flight-management-system/apconf.lua`**; the only calibration in it is `CFG.HEADING_SIGN` / `CFG.HEADING_OFFSET`, which **must match the UAC's** — same aircraft, same sensor — plus the protocol names, which must match at both ends.) Edit, save, reboot the UAC, test. The golden rule: **if an axis diverges/blows up rather than oscillating, it's a sign or frame error — not a gain.**

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

9. **Heading frame, FMS side** — confirm `CFG.HEADING_SIGN` / `CFG.HEADING_OFFSET` in `flight-management-system/apconf.lua` match the UAC's. They are the same aircraft and the same sensor, so they must agree: the UAC would still fly to the right heading (it owns its own copy), but the FMS's alignment gates and landing heading checks would open at the wrong moment.

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
- **Fly a ROUTE** (course): it flies the waypoints as one **planned path**. When you confirm the plan the FMS lays out each interior corner as a `straight → clothoid → arc → clothoid → straight` fly-by transition — the turn radius from a real bank angle, the spiral entry jerk-limited so lateral acceleration ramps in rather than stepping — then bakes a **speed profile** along it (a backward braking pass from the route end, then a forward acceleration pass), and pure-pursues a moving aim-point ("carrot") along the result. So it cruises the straights, arrives at each bend already slowed, and brakes for the destination *in advance*, with no bearing snap at a waypoint. It still decides the flight phase (ENROUTE → APPROACH → TERMINAL → ARRIVED) from the plan, handing to a precise point-capture only for the final arrival/landing.
- **SPD** button: cycle the cruise-speed **ceiling** **FULL → SLOW → COAST** (the curve speed profile always stays under whatever you pick).
- **HOME / CANCEL**: stop the autopilot and hand control back.

Everything the FMS can be retuned or rewired with — protocol names, database filenames, the heading sign convention, cruise speeds, the path planner's physics, the landing profiles, the instrument scales — lives in **`apconf.lua`** beside `autopilot.lua`, not in the program (§7.1).

The external **monitor** on this computer is the **HSI**, and it has **two modes**:

- **While navigating a course / direct-to** it's the classic **VOR / HSI**: a heading-up compass rose (the "squarised circle"), a **course line**, a green/yellow **course-deviation needle**, and the **heading number** — plus DTK (desired track), XTK (cross-track metres, L/R) and distance-to-go. The course line is drawn as a **two-tone compass needle**: the tip pointing at the waypoint is lit, the tip pointing the opposite way is dark, so you can read **TO** or **FROM** off the instrument the way you would a physical compass. It flips as the fix goes abeam, and within a few degrees of abeam — where the question has no meaningful answer — both tips go grey. The same reading is spelled out as `TO` / `FR` / `--` beside the DTK number. On a curved route the course line and needle follow the **flown curve** — DTK is the curve's *tangent* and XTK your deviation *from the curve* — so the needle centres when you're tracking the curve, not the straight leg. Top-left **CMD** reads the commanded speed and top-right **GS** your ground speed.
- **Once a landing engages** (auto or manual) it switches to a **heading-up top-down map** for flying the approach or watching the autoland. Your aircraft sits fixed at the centre as a nose-up `^` while the compass card and the **landing site** rotate under it (dead ahead is the top). A pad is an `H` with a short line marking its approach heading; a **runway** is drawn as the strip itself — threshold `+`, a lime cell at the touchdown aim point, and a line running down to the far end. The scale auto-ranges (shown as `R__m`) so the pad creeps in as you close. The **top row is a heading-error bar** — a bright marker shows how far the pad's landing heading is off your nose, green when aligned. The **bottom row** shows the landing numbers: phase / AGL / VS / position error / heading error — or, on a runway approach, phase / AGL / **glidepath deviation** / cross-track / runway still to run. A runway approach also gets a **glideslope pointer** up the right-hand edge: like the course needle it marks *where the glidepath is*, so fly toward it — high on the approach puts it below the centre tick.

### 6.3 Automatic landing

Landing sites live in `landing.db` — one line per site, naming a point that already exists in `points.db`:

```
; NAME = type , direction , altitude , runway length (m; ignored for a pad)
TMU=heli,n,69,0        <- a helipad: land facing north, pad surface at y=69
TMV=rwy,s,69,295       <- a runway THRESHOLD at TMV: land heading south, 295 m of strip
TMA=rwy,n,69,295       <- ...the same strip flown the other way, its own threshold point
```

Two ways in, whichever the site type:

- Fly a **route that ends at a landing site** — it auto-lands on arrival.
- Use the **LAND** menu to go direct-to a site and land at the end.

**A pad (`heli`)** is flown straight down onto the point: **ALIGN** (swing to the pad heading, holding position) → **TRANSLATE** (close the last few metres precisely) → **DESCEND** → **FLARE** → **LANDED**. Throughout, the UAC holds the craft over the pad and on the landing heading while flying the commanded descent rate.

**A runway (`rwy`)** is flown as a straight-in approach instead. The point is the **threshold**, and the strip runs from it for `runway length` metres in the landing direction (so `TMV=rwy,s,…,295` at `609,598` ends at `609,893`). When you confirm a plan that ends at a runway the FMS quietly adds a **FINAL** fix on the extended centreline (`DC.RWY_FINAL_M`, 3 km by default) so the path planner flies its own turn onto the runway heading; passing that fix arms the approach:

| Phase | What it is doing |
|---|---|
| **INTERCEPT** | flying onto the centreline, descending toward the glidepath — but never below the approach minimum (`RWY_MIN_AGL`) until established |
| **GLIDE** | established, on the glideslope |
| **FLARE** | last few metres, easing the sink rate into a gentle touchdown |
| **ROLLOUT** | on the strip, braking toward the far end |
| **LANDED** | stopped |
| **MISSED** | no longer enough runway left to touch down *and stop* in — flies the published go-around circuit if the runway has one, otherwise levels off and stops, wings level on the runway heading, for you to go around from |

**The runway's length is an input, not decoration.** The approach speed is the *slowest* of what you asked for, what the glideslope can be flown at with the vertical channel's descent authority, and **what the rollout can stop from in the strip that is genuinely left** — so a short runway is flown slower without anyone deciding to. The FMS landing screen names the two numbers it sized itself with (`RWY 28m/s 4deg`).

"Genuinely left" includes the **flare float**, which on a short strip is the single biggest consumer of runway: easing from the glideslope sink rate to `RWY_TD_VS` through `RWY_FLARE_AGL` takes `2·FLARE_AGL / (V·tan GS + TD_VS)` seconds, which even at 30 m/s is ~80 m of tarmac before the wheels arrive. The speed and the float are solved together (each limits the other), and `RWY_ROLLOUT_FRAC` then holds part of the remaining rollout back as deliberate go-around slack — so the planned touchdown point sits comfortably *before* the point of no return rather than exactly on it.

**Staying on the centreline** is a real localizer, not a lazy pursuit of a point ahead: the approach commands a lateral closure *rate* proportional to how far off the line it is, damped by how fast that error is already closing, and capped at `RWY_LOC_MAX_DEG` of intercept. Raise `RWY_LOC_KP` for a crisper capture; lower it if the aircraft weaves. This is also why the approach flies the same in all three modes — it names a direction and a speed, and each mode achieves that with whatever it has (a VTOL strafes onto the line, a FIXEDWING banks onto it).

**Automatic go-around.** A runway may publish a circuit in `landing.db` as its own kind of record — `<id>=goaround,startX,startZ,endX,endZ,altitude`. On a missed approach the FMS flies it and then tries the landing again, up to `LG.RWY_GA_MAX` times. Publish nothing and a missed approach levels off and holds, as before.

The published leg is an **oriented line segment**, not a pair of waypoints, and it is flown as one:

1. **run straight on** at the missed-approach heading, climbing to the circuit altitude;
2. **turn in** and arrive at the leg's **start**, on the **leg's own heading**;
3. **fly the whole leg**, start to end, dead straight;
4. only **after passing the end**, turn back onto the runway line, descending as you come round;
5. **rejoin the approach** on the runway heading and land.

Steps 2 and 4 are built as shortest **curvature-limited** (turn–straight–turn) paths at the turn radius the circuit speed allows, so the leg may point in *any* direction relative to the missed approach — including straight back at it — and may be any length. A leg running the same way you already are comes out as a gentle S; one running the opposite way comes out as the teardrop you would fly by hand. Nothing assumes the leg is parallel to the runway. **How far the aircraft runs on before turning in is searched, not fixed**, because turning onto a parallel leg from close abeam has no curvature-limited solution but a full 360° loop, whereas the same turn from a little further back is an easy S.

The id is arbitrary; a circuit is matched to a runway by its **leg direction**, so the two ends of one strip each get their own record (the same segment, published both ways round).

> **The circuit descends.** It holds the published altitude outbound and along the leg, then comes down to whatever the glidepath wants at the point it rejoins — holding pattern altitude all the way to the runway line just means arriving high and going around again. The rejoin is placed at the leg's end if the leg finishes on the approach side of the threshold (the tight circuit most published geometry is drawn for), otherwise at the shortest final worth flying (`RWY_FINAL_MIN`).

**The glidepath is captured, not merely descended toward.** The vertical command is the sink the geometry needs at the speed actually being flown, plus a correction proportional to how far off the path we are — which means it will **climb** back up onto the path (bounded by `RWY_VS_UP_MAX`) if it ends up low, and never inside the flare.

**One thing you must do yourself: arrive at a sensible height.** The FMS only takes the vertical channel once the approach arms, so ALT-AP still owns your cruise altitude up to the FINAL fix. Be no more than about `RWY_VS_MAX × RWY_FINAL_M ÷ approach speed` above **field** elevation as that fix goes by — roughly **280 m** with the shipped numbers. Higher than that simply cannot be flown down in the distance left, and the approach will run through to MISSED rather than dive at the runway.

The whole approach is identical in **VTOL, HYBRID and FIXEDWING** — it only ever names a point to fly to, a speed and a vertical speed, never an attitude, so each mode executes it with whatever actuators it has.

While it's flying itself down, the **Altitude Selector shows all-green** (it's "overshadowed" — ALT-AP is off while autoland owns the vertical channel).

### 6.4 Pilot control during & after landing

- **ABORT** (button, mid-descent): stops the landing, climbs away on ALT-AP as a go-around.
- **Pilot override — pause the descent**: at any point in the air (pad or runway approach alike), **tap any panel on the Altitude Selector**. The FMS **pauses the descent** and hands the vertical channel back to ALT-AP (the craft holds/flies your selected altitude) **while keeping position and heading alignment over the pad**. The FMS shows **RESUME LND** (resume the descent — the panel goes all-green again) and **ABORT**.
- **After touchdown** the FMS offers two options:
  - **RESUME ALT-AP** — leave the pad and climb to the ALT-AP cruise altitude.
  - **AP OFF** — park on the ground, autopilot off (the craft just holds the ground). Re-arm later by tapping a panel on the Altitude Selector.

  *(A rollout is the one airborne-ish phase that cannot be paused: the aeroplane is braking on a finite strip, and handing the vertical channel back mid-rollout would achieve nothing.)*

**On the ground, the gear's squat switch outranks the altimeter.** Once the weight-on-wheels contact is made, the vertical channel stops holding an altitude entirely and holds the craft *down* on its gear with a small standing down-command (`CFG.AL_GROUND_VS`). This matters: an altitude hold on the ground answers every settle — gear compressing, a bump, the surface not being exactly where `landing.db` says — by firing the lift thrusters **up**, which keeps the aircraft skittish, unloads the gear and on a rollout stops the wheels braking. With no squat contact fitted or heard, there is nothing but the altimeter, so it falls back to holding the altitude it touched down at.

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

### 6.7 Flight modes — VTOL / HYBRID / FIXEDWING

`CFG.FLIGHT_MODE` in `pid/uacconf.lua` sets **how the aircraft corrects sideways error**. Levelling, altitude and the flight plan are identical in all three; only the lateral technique changes. An unrecognised value falls back safely to VTOL.

| Mode | How it flies |
|---|---|
| `"VTOL"` *(default)* | Holonomic. Strafe thrusters push the craft sideways onto the track; the nose is free to point wherever. Nothing banks. This is the original behaviour and is byte-for-byte unchanged by the other two modes existing. |
| `"HYBRID"` | Banks like an aeroplane while en route — lateral error is flown by rolling toward the track and letting the tilted lift pull it round — but keeps a *sparing* amount of strafe for damping, and reverts to full wings-level strafe for the close-in pad phases (TERMINAL/ARRIVED/LANDING) and throughout an automatic landing. |
| `"FIXEDWING"` | Strafe thrusters disabled entirely, in every phase. All lateral correction is by banking and turning. |

**The coordinated turn.** In HYBRID the yaw axis is driven *from the bank*, reproducing the real relationship `ω = g·tan(bank)/v` — so the nose swings at the rate the aircraft's lean actually implies, rather than at some unrelated rate of its own. That means **in HYBRID the bank is what steers**: heading error commands a bank, and the bank turns the aircraft. FIXEDWING doesn't need this (banking is already its only lateral mechanism, so its ordinary yaw law follows the curving path naturally).

**Key tuning knobs** (all in `CFG`, see the sign-calibration notes at each):

| Knob | What it does |
|---|---|
| `BANK_MAX` | hard cap on commanded bank, **radians**. Keep it equal to the FMS's `DC.MAX_BANK_DEG` or the aircraft can't fly the corners it plans. Note a hard bank costs vertical authority (`cos(bank)` of the lift is left pointing up), so expect some altitude sag in a sustained steep turn. |
| `BANK_PER_YAWERR` | rad of bank per rad of heading error — in HYBRID this is the **entire** heading loop, since yaw follows the bank. Too low and the aircraft tracks its route perfectly while the nose drifts to any angle at all. |
| `BANK_HDG_KD` | damps that heading loop against the measured yaw rate. |
| `BANK_RATE` | roll rate, rad/s. Keep it several times faster than the heading loop's own time constant (`v / (GRAVITY·BANK_PER_YAWERR)`) or the roll slew becomes the dominant lag and the loop hunts. |
| `BANK_PER_LAT` / `BANK_LAT_MAX` / `BANK_XTK_KD` | the cross-track *trim* bank. Keep `BANK_LAT_MAX` well under `BANK_MAX` — this is a trim, not the primary control. **`BANK_XTK_KD` is a rate gain: if it sways, lower it.** |
| `BANK_SIGN` / `BANK_XTK_SIGN` / `HDG_FROM_BANK_SIGN` | direction calibration, like every other `*_SIGN` — see Section 8. |
| `HYBRID_STRAFE_FRAC` / `HYBRID_STRAFE_MAX` | how much strafe HYBRID keeps while banking. `0` flies it like FIXEDWING; `1` like a VTOL. |


### 6.8 Keyboard joystick — hand-flying FIXEDWING

*Optional, `keyboard-joystick/`.* A trim-style stick and throttle quadrant, read off a **typewriter** peripheral. It flies nothing itself — like the VS-commander and the MIC it only *names* a wanted attitude, which the UAC then flies.

| Key | Does |
|---|---|
| **E / D** | nose up / nose down |
| **S / F** | roll left / roll right |
| **SPACE / A** (or **Z**) | throttle up / down |
| **C** | centre the stick (pitch and roll back to level; the throttle is **not** touched) |

**It is a trim, not a spring.** Holding a key ramps the *target* at a steady rate and releasing it leaves the target where it got to — so the aircraft sits at whatever bank you last commanded. `C` is how you get back to level; there is no other way but counting keypresses back.

It is live only when **`CFG.KJOY_ENABLE`**, the UAC is in **FIXEDWING**, **AP-NAV is off**, and packets are arriving (silence hands the aircraft straight back rather than latching a stale bank). When live it *replaces* the autopilot's bank and pitch targets and drives the throttle directly — a lever names engine power, not a speed, so there is no PID between your hand and the thrusters. Yaw is left to the nav stick's rudder, so stick-and-throttle here plus rudder there is the split.

Its display is drawn for a **single 1×1 monitor** (15×10 cells): a stick box, a throttle bar, the commanded angles, and a link tag (`TX`, or `!M` / `!K` if the modem or typewriter is missing).

**Signs live in `uacconf.lua`, not here** (`CFG.KJOY_PITCH_SIGN`, `CFG.KJOY_ROLL_SIGN`) — this program always sends raw stick position. If the stick is inverted in flight, flip it there; flipping it in two places is how you end up with a display that disagrees with the aeroplane. `CFG.KJOY_TURN_GAIN` / `KJOY_TURN_REF_SPEED` set how briskly a given bank turns you (the honest coordinated-turn rate is realistic but far too lazy to hand-fly at these speeds — see their notes in the config).

`get_keycodes.lua` is a scratch tool: run it, press a key, and it prints that key's GLFW code, in case you want to remap the cluster.

### 6.9 Nav data server — one waypoint database for a fleet

*Optional, `nav-data-server/`, and **not** part of an aircraft.* Put it on the ground with a monitor and a keyboard. It holds the master `points.db` / `course.db` / `landing.db`, gives you a proper mouse-and-keyboard editor for them, and broadcasts them on `aer_navdata` every 30 s and immediately after any edit. Every FMS in range applies the update and acknowledges it.

- **The server is master and the flow is one way.** An FMS's own EDIT DATA screens still work, but they are transient — the next broadcast overwrites them. Changes meant to last belong on the server.
- **Its roster is the point of it.** Each aircraft appears with its label, the revision it holds and its state: `IN SYNC`, `HELD <screen>` (the pilot is on a screen where swapping the database under them would be unsafe — it will apply as soon as they leave it), a fault, `UPDATING`, or `SILENT`. This is how you confirm an edit actually reached an aeroplane rather than assuming it.
- **An update can never move a flight already under way.** The flown plan is a snapshot taken when it was confirmed, so a refresh that moves, renames or deletes a waypoint you are currently flying to changes nothing until the *next* plan.
- The server refuses to send a points file that has stopped looking like one, so a corrupt master cannot wipe the fleet's databases from the air. Coordinates must be **whole numbers**, or the aircraft will silently drop the point.

---

## 7. Tuning reference (`CFG` in `pid/uacconf.lua`)

All optional once the signs (Section 5) are right. Numbers are the shipped defaults for the *Aer Togekiss*.

| Knob | What it does |
|---|---|
| `FLIGHT_MODE` | `"VTOL"` / `"HYBRID"` / `"FIXEDWING"` — how lateral error is corrected (strafe vs. bank). See **§6.7** for the modes and their `BANK_*` knobs. |
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

### 7.1 FMS configuration (`flight-management-system/apconf.lua`)

`apconf.lua` is to the FMS what `uacconf.lua` is to the attitude controller: **all** of its configuration, and none of its logic. It returns one `CFG` table holding the rednet protocol names, the `.db` filenames, the heading sign/offset (which **must** match the UAC's), the FULL/SLOW cruise presets, the waypoint pass radii (`RAD`), the path planner (`DC`), the landing profiles (`LG`), the instrument scales and both colour palettes. `autopilot.lua` keeps no fallback copy of any of it, so a missing or half-saved file stops the computer at boot naming the missing knob.

The two tables you are most likely to touch are `DC` (below) and `LG` (§7.2).

**`DC` — trajectory.** How the FMS plans its **path** and its **speed**. The path is built **once**, when you confirm a plan — not re-evaluated every tick. Each interior waypoint gets an explicit `straight → clothoid → arc → clothoid → straight` fly-by transition, and a solver iterates geometry against speed until both agree. All optional; the defaults suit the *Aer Togekiss*.

**Path geometry**

| Knob | What it does |
|---|---|
| `CURVE` | `true` = fly the planned path; `false` = aim straight at each waypoint (no rounding). |
| `CURVE_LOOKAHEAD` | how far ahead on the path the aim-point ("carrot") sits, m. Larger = smoother / cuts corners more; smaller = tracks the waypoints tighter. |
| `MAX_BANK_DEG` | **the main cornering knob.** Every corner's radius comes from `R = V²/(GRAVITY·tan(MAX_BANK_DEG))`. Higher = tighter, faster corners at a steeper bank. One *physical* number that means the same thing in all three flight modes — retune this, not a per-mode acceleration figure, if cornering feels wrong. **Keep it equal to the UAC's `CFG.BANK_MAX`** (in radians) or the aircraft can't fly the corner it planned. |
| `JERK_MAX` | m/s³ — caps how fast lateral acceleration may ramp through the spiral entry, giving the clothoid its length (`Ls = V³/(R·JERK_MAX)`). Lower = longer, gentler roll-in; higher = shorter spiral, closer to a plain circular corner. |
| `MIN_RADIUS` / `MAX_RADIUS` | floor and ceiling on a corner's radius, m. |
| `LEG_FIT_FRACTION` | max fraction of *either* adjacent leg a transition may consume. Kept under 0.5 so two corners sharing one leg can never overlap. |
| `SAMPLE_SPACING` | m between samples of the baked path polyline. Smaller = smoother carrot, more memory. |
| `PASS_CORRIDOR` | m — how far off the planned path the aircraft may be and still have its *position along* that path believed when deciding which waypoint is next. Beyond this only the pass-radius test advances the plan. Comfortably wider than any legitimate cross-track; there is no reason to touch it. |
| `PLAN_MAX_ITERS` / `PLAN_CONV_EPS` | how hard the geometry↔speed solver tries before accepting the plan (iteration cap; convergence tolerance in m). |

**Speed profile**

| Knob | What it does |
|---|---|
| `BRAKE_DECEL` | m/s² the **corner** speed profile plans to brake with. Sets how early/short the slowdown into a bend is (braking distance ≈ (v²−v_target²)/(2·`BRAKE_DECEL`)). **Set it to what the craft can really brake** — too high and it arrives hot; too low and it crawls in from far out. |
| `ARRIVE_DECEL` | m/s² reserved for the **final destination stop only**, independent of `BRAKE_DECEL` so it can be much steeper. A gentle final approach can legitimately carry near-cruise almost to the last waypoint, so this is what actually stops it. Raise it if the craft still arrives hot. |
| `ACCEL_MAX` / `DECEL_MAX` | rate limits on how fast the *commanded* speed may rise / fall, so it reads as a smooth curve, not a staircase. Keep `DECEL_MAX ≥ ARRIVE_DECEL`, or this slew — not the planned physics — becomes the bottleneck. |
| `FINAL_ARRIVE_SPEED` | the speed the profile aims for AT the destination (the point-capture then stops it). Raise it to fly *through* a course without slowing at the last fix. |
| `XTK_SLOW_LO/HI/MIN` | slow down while well off the leg, so the craft flies *in* to intercept the line instead of arcing across it. |
| `SPEED_FULL` / `SPEED_SLOW` | the FULL / SLOW cruise-cap presets the **SPD** button cycles (the ceiling the planned speed stays under). *(These two live at the top level of `apconf.lua`, not inside `DC`.)* |

### 7.2 Landing tuning (`LG` table in `apconf.lua`)

**Pads.** `ALIGN_HDG_DEG` / `FINE_HDG_DEG` / `POS_TOL_M` are the coarse-then-fine gates that must be met before the descent starts; `RECAPTURE_M` / `RECAPTURE_HDG_DEG` are how far it may drift mid-descent before it stops sinking and re-captures the pad; `DESCEND_FAST` / `DESCEND_SLOW` / `FLARE_AGL` / `TOUCHDOWN_AGL` are the descent profile (keep `DESCEND_FAST` within the vertical channel's `CFG.AL_DESC_MAX`); `TD_SETTLE_*` are the "commanding descent but no longer sinking" backup for a pad altitude that is a metre or two out.

**Runways** (`RWY_*`). The one number everything else is sized around is **`RWY_VS_MAX`** — the steepest descent the approach may command, which must stay at or under the UAC's `CFG.AL_DESC_MAX`. **This airframe's descent authority, not its engines, is what limits the approach.**

| Knob | What it does |
|---|---|
| `RWY_GS_DEG` | the glideslope. Steeper = shorter final, but a faster sink rate for a given speed. |
| `RWY_VS_MAX` | steepest commanded descent, m/s. Keep ≤ the UAC's `CFG.AL_DESC_MAX` or that will clamp it and silently flatten the glideslope. |
| `RWY_GS_MARGIN` | fraction of that descent authority the *nominal* glideslope may use; the rest is what is left to **catch** the glidepath when you join above it. **Do not set this to 1** — the steepest possible descent would then be exactly the glideslope, so a high join could only ever fly parallel to the path and never rejoin it. |
| `RWY_APP_SPEED` | the approach speed you *want*; the two caps above and below may lower it. |
| `RWY_BRAKE_DECEL` | m/s² the rollout is **assumed** to brake at. This is what makes runway length bite: approach speed ≤ `sqrt(2·a·rollout)`. Set it to what the craft really stops at — too high and it runs off the far end. |
| `RWY_AIM_FRAC` / `RWY_AIM_MAX` | where in the touchdown zone to aim, as a fraction of runway length past the threshold, capped in metres. |
| `RWY_STOP_MARGIN` | metres of strip deliberately left unused at the far end. |
| `RWY_FINAL_FIX` / `RWY_FINAL_M` / `RWY_FINAL_MIN` | the planned straight-in final leg: whether to add the FINAL fix at all, how far back down the centreline it goes (and so how much height can be lost on it), and the shortest leg still worth flying. |
| `RWY_ARM_M` | distance from the threshold at which the approach takes the aircraft over. Must exceed `RWY_FINAL_M`. |
| `RWY_CARROT_M` | how far ahead the steering point is placed. It sits in the localizer's commanded *direction*, not on the centreline, so this is only how far ahead the UAC is asked to aim. |
| `RWY_LOC_KP` / `RWY_LOC_KD` / `RWY_LOC_MAX_DEG` | **the localizer.** `KP` is metres per second of lateral closure per metre off the line (so 1/`KP` ≈ its time constant, ~5 s at 0.20); `KD` damps against the rate the error is already closing; `MAX_DEG` caps the intercept angle however far off you are. Raise `KP` for a crisper capture, lower it if it weaves. |
| `RWY_VS_UP_MAX` | the most the approach may **climb** to rejoin the glidepath from below. Without it the vertical law can only level off and wait for the path to come down, which is not a capture. Never applied inside the flare. |
| `RWY_ROLLOUT_FRAC` | fraction of the usable rollout the approach speed is sized to need; the rest is deliberate go-around slack. **1.0 puts the planned touchdown exactly on the point of no return**, so any float at all triggers a missed approach — lower it for a slower, more forgiving approach onto a short strip. |
| `RWY_CAPT_XTK` / `RWY_LOSE_XTK` | how close to the centreline counts as established, and how far off gives it up again (the gap between them is deliberate hysteresis). |
| `RWY_MIN_AGL` | the approach minimum: never descend below this until established on the centreline. |
| `RWY_FLARE_AGL` / `RWY_TD_VS` | where the flare starts, and the sink rate flown onto the surface. **These two set the float**, and the float is what eats a short runway — a higher/earlier flare and a gentler touchdown are kinder to the gear and much longer. The approach speed is sized around whichever you pick, so softening them buys comfort at the price of speed. |
| `RWY_GS_KP` | how hard it pulls back onto the glidepath per metre of deviation. |
| `RWY_STOP_SPEED` | ground speed under which the rollout is finished. |
| `RWY_GA_ENABLE` / `RWY_GA_MAX` | whether a missed approach flies the published circuit, and how many attempts before it gives up and holds instead. The count resets only when you commit to a new plan. |
| `RWY_GA_ALIGN` | how closely a circuit's leg must run with the landing direction to be considered that runway's (cos of the angle; 0.5 = within 60°). |
| `RWY_GA_SPEED` / `RWY_GA_CLIMB_VS` / `RWY_GA_ALT_KP` | the speed the circuit is flown at, the most it may climb while joining, and how hard it captures the circuit altitude. The speed also sets the **turn radius** the whole circuit is built at (`V²/(g·tan(MAX_BANK_DEG))`), which is why a circuit is not flown at cruise. |
| `RWY_GA_STRAIGHT_M` | the **minimum** straight run before any turning. **Keep it short** — it is a floor on the search, and setting it long throws away the room the turn-in needs (a 125 m sidestep onto a parallel leg is a gentle S from 150 m out and a full 360° loop from 200 m out). |
| `RWY_GA_RUNOUT_M` | how much further than that minimum the FMS may run on, searched for the shortest total circuit. This is what lets it run *past* a leg and turn back onto it instead of looping. |

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
| FMS or UAC **stops at boot** naming a missing knob | its config file (`apconf.lua` / `pid/uacconf.lua`) is missing, truncated or half-saved. That is deliberate — neither program carries fallback values. Restore the file; the message names what it could not find. |
| Runway approach ends in **MISSED** every time | you are arriving too high to be flown down in the distance left. Descend on ALT-AP before the FINAL fix (§6.3), or lengthen `LG.RWY_FINAL_M` / steepen `LG.RWY_GS_DEG`. |
| Same approach lands sometimes and **goes around other times** | the touchdown is landing near the go-around point, so a little float decides it. Lower `LG.RWY_ROLLOUT_FRAC` (slower approach, more slack), or shorten the float with a lower `RWY_FLARE_AGL` / higher `RWY_TD_VS`. |
| Flies a track **parallel to the centreline** and lands wide | the localizer gain is too low for the airframe — raise `LG.RWY_LOC_KP`. (Very visible on a VTOL, which crabs sideways rather than turning.) |
| **Weaves** along the centreline | the opposite: lower `LG.RWY_LOC_KP`, or raise `LG.RWY_LOC_KD` to damp it. |
| Descends, then **climbs back to cruise altitude** mid-approach | the UAC is a generation behind: its `hold` handler must seize the *current* altitude when entering hold from a descent, or it reverts to the stale ALT-AP selection. |
| **Fires the lift thrusters up** while on the ground | the vertical channel is holding an altitude instead of holding the craft down. Check the squat switch is being heard (the FMS landing screen's SQUAT line) — without it there is only the altimeter. See §6.4. |
| Runway approach **flies parallel to the glidepath** and never rejoins it | `LG.RWY_GS_MARGIN` is at (or too near) 1, so the steepest possible descent equals the glideslope itself. Lower it — 0.75 leaves a third in hand. |
| **Runs off the far end** of the runway | `LG.RWY_BRAKE_DECEL` claims more braking than the craft has, so the approach speed was sized too high. Lower it to what the aeroplane really stops at. |
| Glideslope is **shallower than `RWY_GS_DEG`** says | `LG.RWY_VS_MAX` exceeds the UAC's `CFG.AL_DESC_MAX`, which clamps it. Lower `RWY_VS_MAX` (or raise the vertical channel's authority). |
| **FIXEDWING drifts off the centreline** on a runway approach | the UAC is not being told this is a *flying* descent — check the FMS and UAC are the same generation (the approach sends a `glide` flag that keeps banking alive while the vertical channel descends). |
| Hand-flying stick is **inverted** | flip `CFG.KJOY_PITCH_SIGN` / `CFG.KJOY_ROLL_SIGN` in `uacconf.lua` — **not** in the joystick program, or the display and the aeroplane will disagree. |
| Stick does nothing | it is FIXEDWING-only and needs **AP-NAV off** and fresh packets. Check the mode tag on the UAC screen and the link tag on the stick's own display. |
| An edit on the nav data server **never reaches an aircraft** | look at the server's roster: `HELD` means the pilot is on a listing/editing screen (it applies when they leave), a fault names why the write failed, `SILENT` means it is not being heard at all. |
| One FMS **commands both aeroplanes** | they share a protocol prefix. Give each airframe its own in `apconf.lua` *and* `uacconf.lua` (and in the relay/panel programs). Leave `aer_navdata` shared. |
| **Pitch/roll dead** but yaw & lift work | engine **attitude** switch off. / **Lift dead** but the rest works | engine **altitude** switch off. |
| Autopilot **won't navigate** (phase shows `HANDOFF`) or **won't hold altitude** (target shows `MAN`) | the matching autopilot switch is off, or the autopilot-switch panel isn't broadcasting — check its screen (`AP NAV/AP ALT`) and modem. |
| **Vertical stick does nothing** | the autopilot **altitude** switch is on, so the stick is ignored (it's manual-only). Flip it off. If still nothing, the VS-commander has no modem, or the stick reads 0 (wire it to fail to 7). |
| **Nav stick does nothing** | the autopilot **nav** switch is on (stick ignored — it's manual-only), or the engine **master** is off (the relays only transmit while master is on and nav off). If still nothing, the MIC or a relay is missing its modem. |
| Manual **yaw turns the wrong way** on the nav stick | flip `CFG.MANNAV_YAW_SIGN` in `uacconf.lua`. (Fore/aft & strafe reuse the calibrated `THRUST_SIGN`/`STRAFE_SIGN`, so they follow from Section 5.) |
| Lift **drives the wrong way** (sinks on climb) | up/down banks swapped — swap the wiring or the `UP_SIDE`/`DOWN_SIDE` constants in `strafecon.lua`. |
| Autopilot **stays slow for much of a long straight leg** | `DC.BRAKE_DECEL` too low → the braking distance to the destination/bend is huge. Raise it (see §7.1). |
| Autopilot **arrives at a bend/destination too fast** | `DC.BRAKE_DECEL` higher than the craft can actually brake. Lower it so it starts slowing sooner. |
| The path **swings noticeably off-course** between far-apart waypoints | lower `DC.CURVE_LOOKAHEAD` (the carrot is cutting the corner) and/or `DC.MAX_RADIUS`. |
| Craft **flies wide of every planned corner** | `DC.MAX_BANK_DEG` and the UAC's `CFG.BANK_MAX` disagree — the plan assumes a bank the aircraft is capped below. Match them (`BANK_MAX` is in radians). |
| In HYBRID/FIXEDWING it **banks the wrong way** | flip `CFG.BANK_XTK_SIGN` if the *cross-track* correction is inverted, `CFG.BANK_SIGN` if the whole bank is. Note `BANK_SIGN` multiplies **both** bank sources, so it can never fix the two *disagreeing* with each other — that's always `BANK_XTK_SIGN`. |
| In HYBRID it **flies the track correctly but the nose points anywhere** (even backwards) | heading authority, not a sign — translation is holonomic and works at any nose angle, so nothing else constrains heading. Raise `CFG.BANK_PER_YAWERR`. |
| Banks **rail-to-rail on a straight leg** | too much rate gain for the loop's bandwidth: **lower** `CFG.BANK_XTK_KD` (and check `CFG.BANK_LAT_MAX` is well under `BANK_MAX`). Raising a D gain past the actuator lag turns damping into a limit cycle — the same trap as `MANNAV_YAW_BRAKE`. |
| Craft **won't throttle up at route start** (sits at 0) | it's aligning — pointing the nose at the first fix before accelerating. It releases once within `ALIGN_HDG_DEG`; if it's stuck, the align target and the flown target disagree (check the FMS/UAC are the synced build). |
| **Lazy ~200 m spiral** on approach that never closes | (fixed in this build) — the direct-to path now strafes toward the target instead of relying on yaw. |
| UAC screen shows the whole loop dead / `raw` in the millions but output ~1 | check the modem is open and the relays are running; the vector-rotation/scale bugs are fixed in this build. |
| `function ... has more than 200 local variables` on load | you added a top-level `local`; put new tuning constants in a table — `CFG.NAME` in `uacconf.lua`, or the `DC` / `LG` / `RAD` tables in `autopilot.lua` — not as a bare `local`. (`luac -p` won't catch this; the VM counts a nested function's locals on top of the file's top-level ones.) |
| Attitude Indicator shows **"NO SUBLEVEL API"** | that computer isn't part of the Sable-tracked contraption — it must be built into the aircraft. |
| HSI heading/map reads rotated or mirrored | the compass conversion is the one `toCompass()` in autopilot.lua (shared by both HSI modes). Course mode: the **CDI needle** deflection is a one-character sign flip in `drawCourseHSI`. Landing mode: the heading-bar turn direction is the sign on `err` in `drawLandingHSI`. |
| AI bank pointer moves the **wrong way** | flip the roll sign on the pointer in `drawAI` (altcom.lua) — it's the one marked bank switch. |
| Two computers fight over the thrusters | only **one** controller may own `aertogekiss_navcontrol`/`stab`/`alt`. Don't run the retired `navcom`/`control` alongside the UAC. |

---

*Fly safe. — Aer Togekiss Flight System*
