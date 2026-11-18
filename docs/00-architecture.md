# 00 — Architecture

*What the whole system is, how the pieces connect, and which invariants everything else
depends on. Read this first; every other file in `_docs/` assumes it.*

---

## 1. What this system is

A distributed **autopilot + fly-by-wire flight control system** for *Create: Aeronautics*
airframes, written in Lua for CC: Tweaked. It is not one program. It is **10–14 CC computers
per aircraft**, plus five shared ground stations, talking over **one-way rednet broadcasts on
named protocols**. There is no bus master, no handshake and no acknowledgement anywhere in
the flight path — the only request/response traffic in the whole system is to the two ground
databases (#24 nav data, #68 terrain), neither of which can move a thruster, and to the
automated ATC controller (#87), which can change **which facility** an aircraft lands on and
**whether it waits** — never how it flies (§7).

### The two brains

Everything on an aircraft is subordinate to exactly two programs:

| | **FMS** (`autopilot.lua`) | **UAC** (`pid/uac.lua`) |
|---|---|---|
| Owns | *where to go* | *how to fly* |
| Frame | world X/Z/Y, metres | body frame, quaternions |
| Rate | 20 Hz nav tick | 20 Hz control loop (blocks; see §6) |
| Reads | `sublevel` pose/velocity, the .db files, terrain server, squat switch | `sublevel` pose/velocity/ω/inertia/mass, everything the panels say |
| Emits | a **point to fly to**, a **speed to fly at**, a **vertical rate/altitude** | **signed floats per thruster axis** |
| Never does | attitude, thrusters, lateral avoidance | waypoints, `landing.db`, quantising to redstone |

The dividing line is worth stating in one sentence, because almost every bug in this tree's
history came from blurring it:

> **The FMS names a point, a speed and a vertical rate. It never names an attitude.**
> **The UAC names a per-axis demand. It never names a redstone level.**

That is why the *identical* landing profile flies on a VTOL, a HYBRID and a FIXEDWING, and
why a relay can spend a fore/aft demand over two bundles at 3× resolution without the UAC
knowing (see [04-relays-and-panels.md](04-relays-and-panels.md)).

---

## 2. The signal chain

```
 GROUND STATIONS (shared, fleet-wide)
 ┌──────────────────┐  aer_navdata  ┌───────────────────┐  aer_terrain
 │ #24 nav data     │──────────────▶│ #68 terrain data  │◀────────────┐
 │ points/course/   │   (also →#44) │ region-file survey│             │
 │ landing .db      │               └───────────────────┘             │
 └────────┬─────────┘                        │  corridor / map        │
          │ sync                             ▼  replies               │
          ▼                                                           │
 ══════════════════════ ONE AIRCRAFT ══════════════════════════════════╪═══════
                                                                       │
  touch monitor ─┐                                                     │
                 ▼                                                     │
        ┌────────────────────┐   navap    ┌──────────────────┐         │
        │  FMS  autopilot.lua│───────────▶│  UAC  uac.lua    │         │
        │  #8/16/28/39/74/46 │  autoland  │  #1/19/27/38/72  │         │
        │                    │───────────▶│                  │         │
        │  HSI monitor  ◀────┤            │                  │         │
        │  terrain client ───┼────────────────────────────────┘        │
        └───┬────────────┬───┘            └──┬───┬───┬───┬────┐        │
   apswitch_1│            │altoverride◀──┐   │   │   │   │    │        │
            ▼            ▼               │  stab│navctrl│ alt│ tri     │
   ┌──────────────┐  ┌──────────────┐    │   │   │   │   │    │        │
   │ UAC, relays, │  │ AS altcom.lua│────┘   │   │   │thrustf │        │
   │ #12, GPWS    │  │ #7/17/29/41/ │        │   │   │   │    │        │
   └──────────────┘  │ 47/75        │        ▼   ▼   ▼   ▼    ▼        │
                     │  altap ──────┼──▶UAC  ┌─────────────────────┐   │
                     │  apswitch_2 ─┼──▶     │  RELAYS (§3)        │   │
                     │  PFD monitor │        │ stabiliser / yaw+   │   │
                     └──────────────┘        │ thrust / strafe+lift│   │
                                             │ / finer thrust      │   │
  MANCON #6/#76 ──mancon──────▶UAC           └──────────┬──────────┘   │
   (typewriter)  ──mancon_nav──▶UAC                     ▼              │
                                              redstone 0–15 → thrusters│
  engine switch #11/22/31/49/71 ──engineswitch──▶ every relay          │
  squat switch  #15/78          ──touchdown_detection──▶ FMS, GPWS     │
                                                                       │
  GPWS/HUD #67/70/71  ── receive-only, listens to all of the above ────┘
  responder #45/53/73/82/83 ──atc_broadcast──▶ #44 TMU / #64 AWACS / every FMS (PLAN view)
                                             └▶ #87 ATC controller ◀──aer_atc──▶ every FMS
                                                (who is parked where)   (may I land? clr /
                                                                         divert / hold)
```

### One-line summary of each participant

| Role | Program | What it contributes |
|---|---|---|
| **FMS** | `autopilot.lua` + `apconf.lua` | route editing, path baking, speed profile, landing, go-around, published approaches, terrain avoidance, HSI |
| **UAC** | `pid/uac.lua` + `pid/uacconf.lua` | quaternion attitude, translation, vertical cascade, manual channels, output allocation |
| **AS** (altitude selector) | `altcom.lua` | four-panel ALT-AP selector, AP-ALT switch announce, PFD (artificial horizon) |
| **MANCON** | `mancon.lua` + `manconconf.lua` (or the 4-computer stack) | hand-flying: the vertical, six horizontal axes, the boost, the scramble |
| **relays** | `msg_to_gas_output` / `hdgcon` / `strafecon` / `finerthrust` / Scout's own | float → whole redstone level, sigma-delta dither, engine-switch gate, deadman |
| **panels** | `engine_switch`, `autopilot_switch`, `manconrelay_rotation` | physical redstone → rednet |
| **GPWS/HUD** | `gpws.lua` + `gpwsconf.lua` | head-up display + ground-proximity alerting. **Controls nothing** |
| **responder** | `responder.lua` | 20 Hz position report on `atc_broadcast` |
| **ATC / AWACS** | `atc.lua` + `atcconf.lua` | radar scope, static (tower) or dynamic (airborne) |
| **nav data server** | `nav_update_server.lua` | master `.db` set, editor, fleet broadcast + ack roster |
| **terrain server** | `terrain_server.lua` + `terrainconf.lua` | region-file ground/obstacle survey, corridor + map queries |
| **ATC controller** | `atcctl.lua` + `atcctlconf.lua` | allocates landing facilities by `sites.db` and the responders' positions: clear, divert within the site, or hold |

---

## 3. The invariants

These are the rules the whole tree is built on. Breaking one of them has, historically,
produced a specific in-flight failure, and each is written up where it bit.

### I1 — One protocol, one broadcaster

Two senders on one rednet protocol alternate at 20 Hz and the last to speak wins. This is
not a race that resolves; it is a 50 % duty cycle on both commands.

*Cost of breaking it:* Scout's #78 broadcast on `scout_mancon` beside the MANCON. The
pilot's scramble flickered off every other packet and #78's `vs = 0.0` overwrote the climb
trim twenty times a second — "the scramble does not work", 2026-08-30. The fix was to route
#78's request in through `CFG.IN_SCRAMBLE` so the MANCON ORs it and remains sole broadcaster.

This is also why #13 and #14 are retired on Aer Togekiss (#6 took `aertogekiss_mancon_nav`),
and why #4 is retired (the UAC took `navcontrol`).

### I2 — Configuration is split from program, and there are no fallback defaults

Any program deployed to more than one computer carries its per-airframe values in a
`*conf.lua` beside it: `uacconf.lua`, `apconf.lua`, `gpwsconf.lua`, `atcconf.lua`,
`manconconf.lua`, `terrainconf.lua`, `vecconf.lua`. The program holds **no** copy of any
value. A missing knob is a **named boot error** (the "canary" list at the top of every
loader), never a nil-arithmetic crash mid-flight and never a silent default.

*Why no defaults:* a second set of values would let an edit in the conf silently do nothing,
which is exactly what makes an empirically-calibrated sign untrustworthy. And for
`ALT_OFFSET` specifically, `0` is a **legal** value (the display bench), so a defaulted 0
would silently restore over-reading altimetry on a real hull and look like a working
aeroplane right up to the flare.

*Never regenerate one conf from another computer's copy.* See
[11-conventions-and-traps.md](11-conventions-and-traps.md) §"The vecconf cautionary tale".

### I3 — The UAC states a demand; the relay decides how to spend it

Since 2026-08-26 nothing in `uac.lua` quantises. Every output leaves as a float in `[-1,1]`
(or `[0,1]` for the split lift lines), and the relay at the far end turns it into a whole
0–15 level with its own sigma-delta accumulator.

*Why:* rounding at the UAC destroys resolution the receiver may have and can never get back.
#69 spends fore/aft over two bundles at 45 A-units; Scout's `thrustcon` does the same. The
authority **limits** (`STAB_MAX`, `YAW_MAX`, `THRUST_MAX`, `STRAFE_MAX`, `ALT_MIN/MAX`) stay
in the UAC — they are airframe calibration, and a relay never widens what it is sent.

### I4 — Message rate *is* dither rate, and silence *is* a deadman

The UAC deliberately does **not** suppress unchanged broadcasts. Relays advance their
sigma-delta accumulator once per *received* message, so the message rate is the dither rate;
and they zero their outputs on a ~1 s link timeout, so silence is a trip, not a saving.

Every manual channel is likewise broadcast every tick even when nothing is commanded, because
the UAC gates on **freshness** — a silent stick is read as "no stick attached" and the craft
is handed back to the autopilot rather than latching a stale velocity.

### I5 — Altitude is the world Y of the **bottom of the hull**, everywhere

`sublevel` reports the assembly's pose, which sits some way up inside the aeroplane. Every
computer on a hull subtracts that hull's `ALT_OFFSET` **the moment it reads the pose**, and
nothing downstream of that subtraction ever sees the raw number.

It has to be that datum because every altitude the aircraft is compared against is the world
Y of a piece of **ground**: `landing.db`'s pad and runway elevations are surveyed surfaces,
and so is the terrain server's profile. Uncorrected, Aer Togekiss flew its descent to a pad
elevation 9 m underground and was stopped only by the squat switch.

**All of a hull's computers must carry the same figure** — see
[10-aircraft.md](10-aircraft.md) for the table and the list of files.

### I6 — Snapshots, not live references

`NAV.points` and `NAV.landSite` are copies taken when the pilot commits a plan, never live
references into `POINTS`/`LANDING`. That is what makes a mid-flight nav-database refresh from
#24 safe: swapping the tables re-arms every *future* plan and leaves the current one exactly
as it was.

### I7 — The alerting half and the display half are separate programs

The FMS's HSI paints terrain and **only** paints it — nothing in `TERR` reads back into the
navigation tick, the speed law or the landing machine. The alerting half of the same database
is the GPWS's job on a different computer. *An instrument that both paints the ground and
decides what to do about it is one program with two reasons to be wrong.*

The single exception is deliberate, isolated and documented: `TAVOID` in the FMS (terrain
avoidance) reads the same server through the same beacon but keeps its own cached corridor and
its own state, and never touches anything `TERR` reads.

### I8 — Fail silent, in both directions

"Don't know" must change nothing. A missing terrain server, a stale corridor, a profile off
the current track, or a window with nothing charted: terrain avoidance neither engages
(never invents a climb from a guess) nor releases (never hands back into ground it cannot
see). Only a fresh, honest, in-range reading may raise the floor or take it away.

---

## 4. Authority: who owns which axis, when

This is the single most useful table in the system, and it is *not* all-or-nothing — it is
per-axis, and getting that wrong is a real historical bug (a centred keyboard stick used to
veto the whole altitude channel).

| Condition | Pitch/roll setpoint | Yaw | Fore/aft + strafe | Vertical |
|---|---|---|---|---|
| **Autoland active** (`alState ≠ normal`) | levelling + FMS's implied attitude | FMS `holdHeading`, or bearing | FMS target | FMS `autoland` |
| **Runway departure, on the ground** (02 §7b) — it *is* an autoland as far as the UAC knows | levelling | FMS `holdHeading` (the runway) | FMS target: line-up point, then a carrot down the centreline | FMS `landed` |
| **Runway departure, climbing out** | autopilot, banking allowed (`glide`) | autopilot | autopilot (the route) | FMS climb (`descend`, `vs > 0`) to `CLIMB_AGL`, then ALT-AP |
| **AP-NAV on, AP-ALT on** | autopilot | autopilot | autopilot | ALT-AP |
| **AP-NAV on, AP-ALT off** | autopilot (roll) | autopilot | autopilot | MANCON vertical stick |
| **AP-NAV off, AP-ALT on** | ALT-AP owns pitch (FIXEDWING); levelling otherwise | manual (MANCON yaw) | MANCON | ALT-AP |
| **both off** | MANCON / keyboard joystick | MANCON | MANCON | MANCON |
| **SCRAMBLE held** | unchanged | unchanged | unchanged | **overrides everything, including an autoland** |

Notes:

* Attitude **stabilisation always runs**. AP-NAV/AP-ALT gate *navigation* and *the altitude
  cascade*, never the levelling loop.
* `AP-NAV` reaches the UAC on `<family>_apswitch_1`, announced by the **FMS** off its own
  redstone input; `AP-ALT` on `_2`, announced by the **altitude selector**. Only Aer Togekiss
  has #12, which merges both onto plain `<family>_apswitch` for the older relays.
* The keyboard joystick (#21, FIXEDWING only) splits per axis: AP-ALT owns pitch, AP-NAV owns
  roll *and* throttle.
* The scramble's override of ALT-AP and the autoland is `CFG.MANCON_SCRAMBLE_OVERRIDE`, and
  it is what makes the detent usable as a hand-flown go-around and a terrain escape.
* **A departure is gated on AP-NAV, unlike a landing.** Because it rides the autoland channel the
  UAC would count it as navigating with AP-NAV off — MANCON horizontal locked out — so the FMS
  ends it itself the moment its own AP-NAV input drops, and waits (owning nothing) if the plan
  was committed with AP-NAV off.

---

## 5. Two things that look like layering violations and are not

**The FMS reads its own AP-NAV redstone input directly** (`CFG.APSWITCH.SIDE`) in
`TAVOID.tick`, rather than trusting the protocol it itself announces. That is deliberate: it
is the same signal, read one step earlier, so a stale corridor reading at the exact moment the
switch goes off cannot leave an automatic climb running past it.

**The FMS owns the vertical channel during a published approach and a go-around** even though
`LND.active` is false. `LND.ga` is that flag. The horizontal is an ordinary route; only the
altitude is the FMS's, because handing it to ALT-AP would climb to whatever the pilot last
selected, which is not where the route lives.

---

## 6. The loop-rate fact everything is calibrated against

`uac.lua`'s control loop asks for 20 Hz and, on the unforked build, got about **4.4 Hz**. The
size of the file is **not** the cause and trimming the control law does not help.

Scout's flight recorder settled it on 2026-08-28: across 660 logged ticks every measured
period was an **exact multiple of one game tick** — 0.10 s 0.2 % | 0.15 s 6.7 % | 0.20 s
41.7 % | 0.25 s 41.4 % | 0.30 s 10.2 % — with nothing in between, reproduced on two further
flights. Lua running long gives a *continuous* spread. A spread with no fractional part is a
loop that is not compute-bound at all but **blocking, a whole tick at a time**, on the calls
the server must serve on its main thread. The loop made **ten per iteration**: five `sublevel`
reads, four `rednet.broadcast`es and the trailing `sleep`.

**So the unit to optimise in is main-thread round trips per tick, not lines or milliseconds.**

Consequences you must know before touching any gain:

* Every attitude gain in `uacconf.lua` is derived from that dead time. `#18` (`tickprobe.lua`)
  measures the server's own scheduling budget (`max_main_global_time`, 10 ms per tick shared
  by every computer on the server). **Read #18 before blaming a control loop for being slow.**
* `pid/uacprof.lua` on #1 and #72 measures what each call class is worth *in game ticks*.
* #1 and #72 run the **forked, optimised** `uac.lua`; #19, #27 and #38 run the pre-fork build
  (see [03-uac-attitude.md](03-uac-attitude.md) §fork). The fork changed **no gain, sign or
  control law** — both builds were captured tick by tick under a stub harness and every
  command on every protocol is identical to 17 significant figures.

---

## 7. What the system deliberately does **not** do

* **No lateral terrain avoidance.** A climb is one axis and well understood; a reroute would
  need the baked clothoid path re-planned around an obstacle mid-flight.
* **No air traffic control in the air.** `atc_broadcast` is still one-way; every listener —
  the scopes, each FMS's PLAN view, and the controller — paints or reads what it hears and
  never answers on it. The automated controller (#87, since 2026-09-28) allocates **ground**:
  it tells an aircraft inbound to a facility whether it may land there, which other facility
  of the same site to use instead, or to hold. It never names a heading, an altitude, a route
  or a speed, and there is still **no vectoring and no separation logic** between aircraft in
  flight. It is fail-silent: with no controller on the air every FMS lands as it always did.
* **No GPWS actuation.** Every GPWS link is receive-only. *A warning system that can fly the
  aeroplane is a second autopilot nobody asked for.*
* **No terrain avoidance during a landing or a go-around.** Those vertical profiles are
  authored and hand-verified against the ground; an automatic floor has no business in them.
* **No suppression of unchanged broadcasts, and no folding of the UAC's four output messages
  into one bus message.** Both would work; both remove a per-channel deadman. The second is a
  change to make on a `uacprof.lua` measurement, not on a guess.
* **No test harness.** Nothing here runs outside Minecraft. `luac -p` is a syntax check and
  nothing more (Lua 5.4 locally, Cobalt 5.1-flavoured in-game).

---

## 8. Where to go next

| Question | File |
|---|---|
| What is on the wire, and in what shape? | [01-protocols.md](01-protocols.md) |
| How does the route/landing/approach logic work? | [02-fms-autopilot.md](02-fms-autopilot.md) |
| How does the attitude/translation/vertical control work? | [03-uac-attitude.md](03-uac-attitude.md) |
| Which redstone face does what? | [04-relays-and-panels.md](04-relays-and-panels.md) |
| How does hand-flying work? | [05-mancon-manual-flight.md](05-mancon-manual-flight.md) |
| What is on the two instrument monitors? | [06-altitude-selector-pfd.md](06-altitude-selector-pfd.md), [02](02-fms-autopilot.md) §HSI |
| What does the GPWS say and when? | [07-gpws-hud.md](07-gpws-hud.md) |
| What do the ground stations serve? | [08-ground-stations.md](08-ground-stations.md) |
| What is the `.db` syntax? | [09-navigation-database.md](09-navigation-database.md) |
| Which computer is which aircraft? | [10-aircraft.md](10-aircraft.md) |
| Signs, frames, Cobalt limits, drawing, traps | [11-conventions-and-traps.md](11-conventions-and-traps.md) |
| How do I get a number out of the game? | [12-diagnostics-and-tools.md](12-diagnostics-and-tools.md) |
