# 10 — The fleet: computer maps and calibration

*Which computer is which, on which aircraft, running what. Verified against the tree on
2026-09-05. Where this disagrees with `CLAUDE.md`, the tree is the authority — see §9.*

---

## 1. The fleet at a glance

| Aircraft | Protocol family | `FLIGHT_MODE` (boot) | FMS `SPEED_FULL` | `ALT_OFFSET` | Callsign | UAC build |
|---|---|---|---|---|---|---|
| **Aer Togekiss** | `aertogekiss` | VTOL | 320 m/s | **9** | TOGEKISS / TGK | forked |
| **Mantine** | `mantine` | VTOL *(see §9)* | 120 m/s | **5** | MANTINE / MTI | pre-fork |
| **Citation** | `citation` | FIXEDWING | 80 m/s | *0 — unmeasured* | — | pre-fork |
| **SS Argo** | `diginav` | VTOL | 80 m/s | **8** | ARGO / RGO | pre-fork |
| **Scout** | `scout` | VTOL | **800 m/s** | **4** | SCOUT / SCU | forked |
| *(airbus, in build)* | `airbus` | — | — | *unmeasured* | AERBUS / RBS | — |
| display bench | `demounit` | — | 320 m/s | **0** (no hull) | DEMO | — |

**`ALT_OFFSET` must be identical across every computer on a hull.** The files that carry it:

* `pid/uacconf.lua` (UAC)
* `apconf.lua` (FMS)
* `gpwsconf.lua` → `CFG.SHIP.ALT_OFFSET`
* `atcconf.lua` → `CFG.STATION.ALT_OFFSET` (**dynamic mode only** — a tower's `STATION.Y` is a
  surveyed elevation and takes no offset)
* `altcom.lua` — **hardcoded at the top**, no conf file
* `responder.lua` — **hardcoded at the top**, no conf file

**To measure:** set it to 0, park on ground of a known Y, read the PFD, take
`(what it read) − (that Y)`. It is a property of the **build**.

### The ATC client (`apconf.lua`'s `CFG.ATC`, 2026-09-28)

| Aircraft | `ENABLE` | `WIDE` | `RUNWAY_ONLY` | `HOLD` |
|---|---|---|---|---|
| **Aer Togekiss** | true | **true** — `sites.db`'s own example of a wide airframe | false | hover |
| **Mantine** | true | false — lands `narrow` strips safely (confirmed by the pilot) | false | hover |
| **Citation** | **false** — no responder, and the orbit is unflown | false | **true** | orbit, 300 m at 40 m/s |
| **SS Argo** | true | false — lands `narrow` strips safely (confirmed by the pilot) | false | hover |
| **Scout** | true | false | false | hover |
| display bench | false | false | false | hover |

Aer Togekiss is the only wide hull. For a new airframe whose span is unknown, set `WIDE = true`
until someone has looked: a wrong `true` costs a hold or a divert; a wrong `false` puts a wide
hull on a strip marked `narrow`.

The controller tells an aircraft's own responder from somebody else parked on its facility **by
callsign**, so `apconf.lua`'s `TERR.CALLSIGN` must equal `responder.lua`'s `CALLSIGN`. It does
on all four hulls that carry a responder.

### The runway departure (`apconf.lua`'s `CFG.DEP`, 2026-10-15)

**All unflown** — estimates from each airframe's other knobs. `VR`/`ROLL_ACCEL` set the roll
(`VR²/2a`), `SPEED` the radius of the turn off the runway (`SPEED²/(11·tan 45°)`, floored at 40 m).

| Aircraft | `ENABLE` | `VR` | `ROLL_ACCEL` | roll | `SPEED` | radius | `GROUND_AGL` | `CENTRE_T` |
|---|---|---|---|---|---|---|---|---|
| **Aer Togekiss** | true | 30 | 6 | 75 m | 40 | 145 m | 3 | 10 |
| **Mantine** | true | 30 | 5 | 90 m | 40 | 145 m | 3 | 10 |
| **Citation** | true | 30 | 4 | 112 m | 40 | 145 m | **6** — `ALT_OFFSET` unmeasured | **3** — cannot strafe, nothing to wait for |
| **SS Argo** | true | 20 | 4 | 50 m | 30 | 82 m | 3 | 10 |
| **Scout** | true | 30 | 10 | 45 m | 40 | 145 m | 3 | 10 |
| display bench | **false** — nothing flies; it would sit in ALIGN | | | | | | | |

Citation's `VR` is tied to its FIXEDWING speed protection: the UAC allows
`FW_PITCH_HEADROOM·FW_PITCH_MAX·V` of climb, 3.7 m/s at 30 m/s, just over `CLIMB_VS` (3.0).

---

## 2. Aer Togekiss — `aertogekiss`, VTOL, primary development platform

A massive, superwide VTOL. The most complete installation on the fleet, and the reference for
what a full fit looks like.

| # | Role | Program |
|---|---|---|
| **1** | **UAC** | `pid/uac.lua` + `pid/uacconf.lua` — the **loop-rate-optimised fork**. Also `pid/uacprof.lua` (tick profiler), `pid/scrambletrace.lua`, `pid/probe.lua`, `pid/obsolete/control.lua` |
| **3** | stabiliser relay (pitch/roll) | `msg_to_gas_output.lua` |
| **4** | **RETIRED** legacy nav controller | `navcom.lua` — startup prints a notice. Kept for A/B testing |
| **6** | **MANCON** — the whole hand-flying stick | `mancon.lua` + `manconconf.lua`. `vspdpid.lua` kept beside it |
| **7** | altitude selector / ALT-AP + PFD | `altcom.lua` |
| **8** | **FMS** | `autopilot.lua` + `apconf.lua` + the three `.db` files |
| **9** | yaw relay (fore/aft unused — #69 has it) | `hdgcon.lua` |
| **10** | strafe + lift relay | `strafecon.lua` |
| **11** | engine switches (master / altitude / attitude / **extra lift**) | `engine_switch.lua` |
| **12** | autopilot-switch merger | `autopilot_switch.lua` — **only this aircraft has one** |
| **13** | **RETIRED** MANCON translation relay | `manconrelay_translation.lua` |
| **14** | **RETIRED** MANCON intention controller | `mic.lua` |
| **15** | touchdown detection + EXTRA LIFT (yaw relay gated off) | `manconrelay_rotation.lua` |
| **45** | ATC/TCAS responder | `responder.lua` |
| **64** | **AWACS** | `atc.lua` + `atcconf.lua`, `MODE = "dynamic"` |
| **67** | HUD / GPWS | `gpws.lua` + `gpwsconf.lua` |
| **69** | finer fore/aft thrust controller | `finerthrust.lua` |

### The 2026-08-28 MANCON collapse

Four computers became one. The forcing constraint is invariant I1: `aertogekiss_mancon_nav` can
have exactly one broadcaster. #6 owns it now, so **#14** (which used to) and **#13** (which fed
#14) are retired the way #4 is — `startup.lua` prints a notice, nothing is deleted.

**#15 is *not* retired**, because its other two jobs are load-bearing: the squat switch the FMS
and the GPWS read as ground truth, and the EXTRA LIFT bank. Only its yaw relay is gated off.

**To put the button panel back**, in this order:

1. `YAW_RELAY_ENABLED = true` in `15/manconrelay_rotation.lua`
2. uncomment the `shell.run` in `13/startup.lua` and `14/startup.lua`
3. point `6/startup.lua` back at `vspdpid.lua`

**#6 must not run both** — they broadcast the same `aertogekiss_mancon`.

### Calibration notes

* `CFG.CLIMB_AT_FULL = 10.0` is **an estimate, not a measurement**, and both `STEPS_PER_MPS` and
  `VEL_KP` derive from it. See `CLAUDE.TODO.md` #1 for how to measure it (hold the scramble
  detent; the steady `velY` reached while `scram` is 1 **is** `CLIMB_AT_FULL` by definition).
  `CFG.TAVOID`'s `MIN_RANGE`/`LOOK_AHEAD_TIME` are sized generously because of this.
* `TRANSLATE_MIN_CMD_FWD = 0.5` — tuned **down** from the fleet's 5.0 for the fore/aft rocking.
  This aircraft is the only one with it low, which is why the commanded-zero breakaway bug showed
  up on Scout first and was mild here.
* `CURVE_LOOKAHEAD_T = 3.4 s` — the fix for the 320 m/s nose sway.
* `CFG.TAVOID.ENABLE = true` — flown here first, as the primary VTOL platform. `WIDTH = 60` m
  (wider than the GPWS's 48) because the corridor should be at least as wide as this hull.

---

## 3. Mantine — `mantine`

The four-computer MANCON stack, and a full relay fit.

| # | Role |
|---|---|
| **38** | UAC (pre-fork build) |
| **39** | FMS |
| **40** | vertical-speed commander (`vspdpid.lua`) |
| **41** | altitude selector / PFD |
| **42** | MANCON intention controller (`mic.lua`) |
| **43** | strafe + lift relay |
| **48** | MANCON rotation relay |
| **49** | engine switches |
| **50** | yaw + thrust relay |
| **51** | stabiliser relay |
| **52** | MANCON translation relay |
| **53** | ATC/TCAS responder |
| **70** | HUD / GPWS |

---

## 4. Citation — `citation`, FIXEDWING

A Minecraft implementation of Cessna's *Citation* jet.

| # | Role |
|---|---|
| **16** | FMS |
| **17** | altitude selector / PFD |
| **19** | UAC (pre-fork build; the only airframe with `PITCH_SMITH_ENABLE`) |
| **20** | stabiliser relay |
| **21** | keyboard joystick (typewriter; FIXEDWING hand-flying) |
| **22** | engine switches |
| **23** | yaw + thrust relay |

**No strafe relay and no MANCON stack** — FIXEDWING disables strafe outright, and hand-flying
goes through the keyboard joystick instead.

**`HAS_ALT_THRUSTER = false`** — the only airframe with no lift engine at all. The vertical is
flown by pitch (`FW_PITCH_*`), and the inner climb-rate loop is skipped entirely so it cannot
wind up against an output nobody receives.

`ALT_OFFSET = 0` and **unmeasured**. Measure it before trusting any landing here.

---

## 5. SS Argo — `diginav`, VTOL

**A floating watership with no sail.** It flies as a VTOL, **deliberately calibrated to sway**.

> Its gains are **not** a reference for an aircraft. Loose, swaying behaviour is the intended
> result there, so do not "fix" its tuning against Aer Togekiss's.

The protocol family is `diginav`, which does not resemble the ship's name — **go by the family,
not by the label.**

| # | Role |
|---|---|
| **26** | vertical-speed commander |
| **27** | UAC (pre-fork build) |
| **28** | FMS |
| **29** | altitude selector / PFD |
| **30** | stabiliser relay |
| **31** | engine switches |
| **32** | yaw + thrust relay |
| **33** | strafe + lift relay |
| **34** | MANCON translation relay |
| **35** | MANCON rotation relay |
| **36** | MANCON intention controller |
| **82** | ATC/TCAS responder |

No GPWS.

---

## 6. Scout — `scout`, VTOL

A small, nimble VTOL. Built as a testbed for the UAC's **vector-nozzle** thrust allocator, and
**reverted to conventional relay control on 2026-08-28**: `CFG.TRI_ENABLE = false`, so
`sendVector()` is inert and the airframe drives directional thruster banks like everything else.

**The revert renumbered the engine-bay computers: #77, #79 and #80 no longer exist** and their
roles moved to **#84, #85, #86**.

| # | Role | Program |
|---|---|---|
| **71** | HUD / GPWS **+ engine switches** | `gpws.lua` + `gpwsconf.lua`; `startup.lua` also opens `engineswitch.lua` in a tab — one lever on `bottom` driving master/altitude/attitude alike |
| **72** | **UAC** | the **forked** build. `pid/uacprof.lua`, `pid/scrambletrace.lua`, `pid/signcal.lua`, `pid/probe.lua`. Its `uaclog*.csv` are the flight records the loop-rate diagnosis came off |
| **73** | ATC/TCAS responder | `responder.lua` |
| **74** | **FMS** | `autopilot.lua` + `apconf.lua` + `.db` |
| **75** | altitude selector / PFD | `altcom.lua` |
| **76** | **MANCON** | `mancon.lua` + `manconconf.lua`; typewriter, a lever on `front`, **and** #78's remote pull-up on `CFG.IN_SCRAMBLE`. Monitor on `bottom` |
| **78** | touchdown detection + AUTO PULL-UP | `scout_touchdown.lua` — **not dead**; a `startup.lua` was added 2026-08-28. Its old `vecengine.lua` + `vecconf.lua` sit beside it, unrun |
| **84** | stabiliser (pitch/roll) **+ lift (`up`)** | `stabil_scout.lua` |
| **85** | yaw + strafe relay **+ lift (`down`)** | `scout_yaw_strafe.lua` |
| **86** | fore/aft thrust controller | `thrustcon.lua` — reads `scout_thrustf`, spends it across two lines at weights 2:1 |

`84` and `85` are **hand-written for this airframe**, not the fleet's shared relays. See
[04-relays-and-panels.md](04-relays-and-panels.md) §4 — **their face and tap assignments are
correct as written and are not to be "fixed"**.

### Calibration facts worth knowing before touching this airframe

Found by flight recorder over 2026-08-28; the reasoning is in `72/pid/uacconf.lua` beside each
knob, **and that file is the authority.**

* **Strafe rolls it.** Full strafe is worth ~8.4 of the 15 roll steps — over half the roll bank —
  reproducible to 1.5 % across two flights and confirmed causal by an AP toggle.
  `CFG.STRAFE_TO_ROLL_FF = -8.4` cancels it. **The cause is still unexplained:** the strafe bank
  is mounted on the centre of mass, and a lateral force through the CoM cannot make a roll
  moment. Suspect the assembly rotates about its **anchor block** rather than its true CoM.
* **Without that feedforward the aircraft is unflyable with AP-NAV off**: a bank makes sideslip,
  the manual lateral hold answers with strafe, the strafe deepens the bank. AP-NAV *with no
  target* runs no translation law at all, which is why engaging it used to recover the aircraft
  and disengaging returned it to the same bank.
* **`CFG.ATT_INTEGRAL_ROLL` must stay true.** With it off, roll has no trim memory and drifts
  away, while pitch (whose integral is unconditional) holds level under the same disturbance.
  See [03-uac-attitude.md](03-uac-attitude.md) §5 for the measurement.
* **`CFG.VERT_XFER_FWD` and `CFG.VERT_XFER_STR` are 0**, disabling the tilt-allocation spill after
  it drove a **3.6 km uncommanded climb**. Restoring them needs the sign settled, and that is tied
  to `CFG.HEADING_OFFSET = math.pi`, which has never been verified against this build.
* **`SPEED_FULL` is 800 m/s** and `CRUISE_SPEED` 800 — by far the fastest setting on the fleet.
* **Fore/aft is asymmetric:** `fwd` is wired across 45 A-units and `aft` across 15, so the
  aircraft **accelerates three times harder than it brakes** until the aft bank is wired the same
  way.
* `CFG.TAVOID.ENABLE = true` here as well.

### Scout's lift is a PAIR SPLIT ACROSS TWO COMPUTERS

`sendAlt` broadcasts `up` and `down` either side of `CFG.ALT_NEUTRAL`; **#84 drives its bank off
`msg.up` and #85's own `alti()` drives a second bank off `msg.down`.** `84`'s in-code comment
*"IF AN UP-FIRING BANK IS EVER BUILT ON THIS HULL…"* is **stale** — it was built, on the other
computer, and the note never moved.

Consequence: the two lines always sum to `2 · ALT_NEUTRAL`. **A command changes the SPLIT between
the banks and never the total burn.**

### The nozzle era

`vecengine.lua` + `vecconf.lua` remain on #78/#84/#85, unrun. **Keep the split.** Before
`vecconf.lua` existed, regenerating the sibling copies from one computer's — to keep the three
programs in step — silently overwrote their wiring tables with the front-centre layout, the
aircraft rolled itself over, and one computer's original wiring was **unrecoverable and had to be
re-derived by hand**.

The vector allocator in `pid/uac.lua` is still the **only** implementation of it, so keep it
working; nothing on the fleet exercises it today.

---

## 7. Ground stations and the bench

| # | Role |
|---|---|
| **18** | tick probe (`tickprobe.lua`) — runs nothing else |
| **24** | nav data server |
| **44** | TMU ATC — a **radar display**, not a controller (`static`, x 705, z 647, facing north) |
| **87** | **ATC controller** (`atcctl.lua`) — allocates landing facilities by `sites.db`. Folder seeded 2026-09-28 **before the computer was placed** (see §8) |
| **64** | AWACS (on Aer Togekiss) |
| **68** | terrain data server |
| **46**, **47** | `demounit` display bench (FMS = HSI demo, altitude selector = horizon demo) |
| **83** | airbus responder — the only computer that airframe has |

---

## 8. Unused ids

**`2` and `5` are empty.**

**`25`, `37`, `54`–`63`, `65`, `66`, `77`, `79`, `80` and `81` do not exist.**

`../ids.json` records the last allocated computer id (**86**), so the next id the world will hand
out is **87** — and **#87's folder already holds the ATC controller** (seeded 2026-09-28), so the
next computer placed in the world *becomes* the controller. Place that one first, with a modem.
Once it exists, the next free id is **88**; use that for any other new computer.

---

## 9. Known drift between `CLAUDE.md` and the tree

Recorded here rather than silently reconciled, because the tree is the authority but the
discrepancy itself may be information.

| `CLAUDE.md` says | The tree says | Note |
|---|---|---|
| Mantine's mode is **HYBRID** | `38/pid/uacconf.lua` has `CFG.FLIGHT_MODE = "VTOL"`, and its whole `FW_PITCH_*` block is annotated "INERT ON THIS AIRFRAME … this craft is VTOL" | The mode is **runtime-switchable** from the UAC's MODE button, so a boot default and a development role can legitimately differ. **Verify before assuming**, and note that switching to HYBRID in flight also arms `HDG_SIM` and the bank overlays |
| Scout `SPEED_FULL` **320** | `74/apconf.lua` has **800** (and `72/pid/uacconf.lua` `CRUISE_SPEED = 800`) | |
| `autopilot.lua` is ~4.5k lines | **6 145** | |
| `uac.lua` is ~2.6k lines | **3 005** (#1) / **3 045** (#72); pre-fork **2 752** | |
| #19/#38/#27 run `_goldmaster/uac.lua` **byte-for-byte** | they now differ from it — they have gained `ALT_OFFSET`, `navAlign`/`alignStop` and `sendAlt`'s `openThrottle` | The goldmaster is still the correct **fork ancestor**; it is no longer a byte-identical description of what those three run |
| Citation `SPEED_FULL` 80, Mantine 120, Argo 80 | confirmed | |
| `autopilot.lua` is deployed **byte-identical** to #8/#16/#28/#39/#74 | true again, and **#46 joined them on 2026-09-17** — six identical copies | #74's debug fork (the route tracer) was folded back the same day; it is kept unrun as `74/autopilot.rtrace.lua`. #46 gained the `align` field it had been one change behind on, which is inert there because nothing listens to the bench |

The `SPEED_FULL` figures in `apconf.lua` and `CRUISE_SPEED` in `uacconf.lua` are **different
knobs**: the FMS's is the pilot-selectable cap it schedules to, the UAC's is a hard ceiling it
clamps any received cap against. Keep the UAC's ≥ the FMS's.

---

## 10. Adding a new airframe

1. Pick a **protocol family** and use it consistently. Do **not** rename `aer_navdata`,
   `atc_broadcast`, `aer_terrain` or `aer_atc`.
2. Copy the **programs** from the nearest airframe, and write the **conf files by hand**. Never
   regenerate a conf from another computer's copy.
3. Give every computer its **marker file** `_<aircraft>_<role>.lua` (zero/one byte; it exists so
   the computer identifies itself when you `ls` it).
4. `startup.lua` runs **exactly one** program (`shell.run`/`shell.execute`). It is how you tell
   what a computer is for. (The one exception in the tree is Scout's #71, which opens a tab.)
5. Edit the hardcoded **protocol name at the top of every relay and panel** — the usual bug is an
   old copy still broadcasting on the donor aircraft's protocol.
6. Measure `ALT_OFFSET` and put the **same** figure in all six places (§1).
7. Calibrate in the order in [03-uac-attitude.md](03-uac-attitude.md) §14 — signs and frames
   first, gains last.
8. Add the aircraft to `points.db`/`landing.db` only if it needs its own pad; add its callsign to
   its `responder.lua` **and** to `gpwsconf.lua`'s `SHIP.CALLSIGN` and `apconf.lua`'s
   `TERR.CALLSIGN`, so the terrain server's activity roster names it — and so the ATC controller
   recognises the aircraft's own report on the facility it has just landed on.
9. Write its `CFG.ATC` by hand (§1): `WIDE` for a hull too wide for a strip marked `narrow`,
   `RUNWAY_ONLY` and `HOLD = "orbit"` for a FIXEDWING.
