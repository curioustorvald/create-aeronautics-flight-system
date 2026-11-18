# Changelog

## 2026-11-18 — sync with the development fleet

The first release since 2026-08-22. Every program is mirrored from the author's development
aircraft (*Aer Togekiss* for every role it has; the vector-engine controller and the fore/aft
thrust controller come from *Scout*) and is the build those computers are running today. The
example configuration files are Aer Togekiss's flown calibration.

> **Upgrading an existing install? Read "Upgrading" at the bottom first.** Several knobs are new
> and *required*: every program refuses to boot without its knobs and names the missing one,
> so an old config file stops the computer on the ground rather than misbehaving in the air.

### New computers (all optional)

| Folder | What it is |
|---|---|
| `mancon/` | **The whole hand-flying stick on one computer.** A typewriter drives fore/aft, strafe, yaw (WER/SDF), climb/descend (SPACE/Z), a boost key (A) and the scramble (V); a redstone lever is a second path to the scramble. Replaces the four-computer stack (`vertical-speed-commander/` + both `mancon-*-relay/` + `mancon-intention-controller/`), which still ships and still works — run one or the other, never both. |
| `gpws-hud/` | **Head-up display + ground-proximity warning system.** Receive-only. Excessive sink rate, terrain closure, forward-looking terrain/obstacle, altitude loss after take-off, too-low-terrain, decision-height callouts, AUTOPILOT DISCONNECTED. Speaks through a speaker from a voice-pack floppy (not supplied). |
| `atc-responder/` | **Transponder**, one per aircraft: position, altitude, speeds, velocity vector, heading, callsign and IFR flag on the fleet-wide `atc_broadcast`. |
| `atc-radar/` | **Radar scope.** A ground tower (`static`) or an airborne AWACS scope centred on its own aircraft (`dynamic`, heading-up or track-up). Paints the nav database underneath the traffic. Receive-only. |
| `atc-controller/` | **Automated ATC (landing facilities).** Reads `sites.db` and answers FMS requests on `aer_atc`: cleared / divert to another facility of the same site / hold / uncontrolled. Allocates ground only — no headings, altitudes or air separation. *Exercised in the author's world but not yet through a full flight test.* |
| `terrain-data-server/` | **Terrain database server** on `aer_terrain`, fed by `terrainbuild` — an offline Java tool (source included) that surveys the world's region files. Powers the GPWS terrain modes, the FMS's terrain avoidance and the HSI's terrain map. |

### Flight Management System

- **Runway departures.** A plan committed while standing on a published runway is flown off it in
  any flight mode: line up (backtracking if needed), take-off roll, rotate, climb straight out,
  then turn for the route. Configured in `apconf.lua`'s `CFG.DEP`. *Every `DEP` number is an
  estimate — this has been exercised under a simulator, not yet flown.*
- **Published approaches** (`appr` records in `landing.db`): the FMS chooses the most suitable
  arrival for the aircraft's position and heading, and flies its published minimum and
  missed-approach circuit. **Waypoint go-around circuits** (`goaround-wp`) are now the preferred
  go-around shape.
- **`course.db` references**: a route may name another route, or a published approach, as one of
  its tokens.
- **Terrain avoidance** (`CFG.TAVOID`): an automatic altitude floor along the track ahead, using
  the terrain server. It only ever climbs; ALT-AP or the pilot descend.
- **Automated ATC client** (`CFG.ATC`): asks the controller for the facility it is inbound to,
  and holds, orbits (FIXEDWING) or diverts as told. **RELEASE ATC** flies on unmanaged.
- **HSI**: a terrain overlay (`CFG.TERR`) and a second normal-flight view, the **PLAN view** —
  the flight course drawn in subpixels over a terrain-hazard layer, with obstacles and traffic.
- An exact 180° reversal in a route no longer hangs the path planner (the reversal apex is
  dropped before planning; a published procedure turn is therefore flown direct for now).
- DIRECT-TO no longer overshoots and orbits its target; VTOL/HYBRID stop properly for the
  route-start ALIGN hold; the runway glidepath no longer crosses the threshold at knee height;
  the display repaints only when something changed.

### Attitude controller (UAC)

- **Loop-rate optimised build.** The control loop was measured blocking a whole game tick at a
  time on main-thread calls; the inertia tensor and mass are now slow-polled
  (`CFG.SLOW_POLL_PERIOD`), duplicated quaternion work is gone, the loop is deadline-paced
  (`CFG.LOOP_PERIOD`) instead of sleeping unconditionally, and the artificial horizon draws in
  runs (~1400 terminal calls per frame → ~46). No gain, sign or control law changed.
- **The scramble is a flag, not a big number.** The stick's scramble detent opens the lift
  outright, bypassing the rate loop, and overrides ALT-AP and the autoland.
- **Manual climb envelope** `CFG.MANCON_CLIMB_MAX` / `MANCON_DESCEND_MAX`, separate from the
  autopilot's `CLIMB_MAX` / `DESCEND_MAX`, so the MANCON boost key can reach past the autopilot's
  handling envelope without loosening it.
- **Altitude datum** `CFG.ALT_OFFSET` (see below).
- **Flight data recorder** (`CFG.LOG_*`) — one CSV row per tick, for reading a fault off a record.
- The relays now quantise the UAC's float demand themselves (sigma-delta per line, each relay's
  own `DITHER` flag), so `CFG.DITHER` is gone from `uacconf.lua`.
- New diagnostics beside `uac.lua`: `uacprof.lua` (what each call class costs in game ticks),
  `scrambletrace.lua` (stick flag → altitude lines → climb, to one CSV), `signcal.lua` (records
  pitch/roll/yaw sign calibration on the ground, commanding nothing).

### Altitude selector

- A full **PFD**: the artificial horizon, pitch ladder, bank scale and flight-path marker are drawn
  at 2×3 subpixel resolution, with a **speed tape**, an **altitude tape** and a **VS scale**.

### Relays and panels

- Every relay and panel finds its modem itself (`peripheral.find`) instead of assuming `top` —
  except `autopilot-switch` (still `top`) and `altitude-selector` (`back`).
- The thruster relays take the UAC's **float** demand and run their own sigma-delta per redstone
  line; face assignments are named constants at the top of each relay.
- The yaw+thrust and strafe+altitude relays no longer rebuild their coroutines on every message.
- **Engine switch**: a fourth input, **`extra_lift`** on `back`.
- **MANCON rotation relay**: also the **squat switch** (`touchdown_detection`, input `back`) the FMS
  and GPWS read as ground truth for a touchdown, and the **EXTRA LIFT** bank (output `front`),
  which repeats the UAC's up-thrust command when `extra_lift` is on. `YAW_RELAY_ENABLED` turns
  just its yaw buttons off for aircraft flown with the one-computer `mancon/`.
- **Vertical-speed commander**: redstone level 15 is the **scramble** detent.
- **Fore/aft thrust controller**: per-channel `SCALE`, defaulting to each channel's wired
  capacity (it used to aim a 45-unit bank at 15 and leave the secondary line dead).

### Nav data server

- Re-reads its `.db` files from disk before every send, so an edit made outside the program goes
  out on the next broadcast.
- Carries **`sites.db`** (for the ATC controller) in the sync, and understands `appr`,
  `goaround-wp` and course references.

### Upgrading

Keep your own `uacconf.lua` / `apconf.lua` and add the new knobs to them rather than copying the
shipped files over yours — the shipped ones are another aircraft's calibration.

**`pid/uacconf.lua` — new required knobs** (copy each block, with its comments, from the
shipped file and then set it for your airframe):

| Knob | What to put |
|---|---|
| `CFG.ALT_OFFSET` | see "Altitude datum" below |
| `CFG.LOOP_PERIOD` | `0.05` |
| `CFG.SLOW_POLL_PERIOD` | `1.0` is fine; it only paces the inertia-tensor and mass reads |
| `CFG.MANCON_CLIMB_MAX` / `CFG.MANCON_DESCEND_MAX` | start equal to your `CLIMB_MAX` / `DESCEND_MAX` |
| `CFG.DISPLAY_PERIOD` | `0.2` (s between UAC screen redraws) |
| `CFG.LOG_ENABLE` (+ `LOG_FILE`, `LOG_DELAY`, `LOG_SECONDS`, `LOG_EVERY` if true) | `false` |

Delete `CFG.DITHER`; each relay carries its own `DITHER` now.

**`apconf.lua` — new required tables:** `CFG.ALT_OFFSET`, `CFG.DEP`, `CFG.TAVOID`, `CFG.TERR`,
`CFG.ATC`, `CFG.COURSE_MAX_REF`. Copy them from the shipped file. To leave a feature off, set
its `ENABLE = false` (terrain avoidance and ATC need the matching ground station anyway).

**Altitude datum — `ALT_OFFSET`.** Altitude everywhere is now the world Y of the **bottom of the
hull**, because every altitude it is compared with (`landing.db` elevations, the terrain survey)
is the world Y of a piece of ground. Sable reports the assembly's pose, which sits somewhere up
inside the aeroplane, so each computer subtracts `ALT_OFFSET` from it. **Every computer on one
hull must carry the same number**: `uacconf.lua`, `apconf.lua`, `gpwsconf.lua` (`SHIP`),
`atcconf.lua` (dynamic mode only), and the two programs with no conf file, `altcom.lua` and
`responder.lua`, which have it at the top. To measure it: set it to 0 everywhere, park on ground
of a known Y, and take (the altitude it reads) − (that Y). The shipped value, 9, is Aer
Togekiss's. **0 is a legal value**, so no program defaults it — a missing one is a boot error.

**Relays**: copy the new program and re-apply your face assignments in the named constants at
the top (`PITCH_POS_SIDE`, `YAW_POS_SIDE`, `STRAFE_POS_SIDE`, `UP_SIDE`, …). **Update the relays
before the UAC**: each relay accepts both the old and the new message shape, but an old relay
cannot read the new one.

**Protocol names** are still `aertogekiss_*` throughout, plus four fleet-wide names that must
**not** be renamed per aircraft: `aer_navdata`, `atc_broadcast`, `aer_terrain` and `aer_atc`.
