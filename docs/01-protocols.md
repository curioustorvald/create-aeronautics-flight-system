# 01 — Protocols: the complete wire reference

*Every rednet protocol in the system: who broadcasts it, who listens, what shape the message
is, how often it goes out, and what happens when it stops. If you are about to add a field,
rename a channel, or debug "why is nothing moving", start here.*

---

## 0. Naming

Per-aircraft protocols are **`<family>_<channel>`**. The family is *not* the aircraft's name:

| Aircraft | family |
|---|---|
| *Aer Togekiss* | `aertogekiss` |
| *Mantine* | `mantine` |
| *Citation* | `citation` |
| *SS Argo* | **`diginav`** ← does not resemble the ship's name; go by the family |
| *Scout* | `scout` |
| *(airbus, in build)* | `airbus` |
| display bench | `demounit` |

Four protocols are **fleet-wide and must never be renamed per aircraft**:
`aer_navdata`, `atc_broadcast`, `aer_terrain`, `aer_atc`.

Relays and panels **hardcode their protocol name at the top of their program** — they have no
conf file. That line is what to edit when cloning a computer onto another airframe, and the
usual bug is an old copy still broadcasting on the donor aircraft's protocol.

---

## 1. Quick index

| Protocol | From | To | Rate | On silence |
|---|---|---|---|---|
| `<f>_navap` | FMS | UAC | 0.2 s + on change | UAC holds last target; display shows STALE after 5 s |
| `<f>_autoland` | FMS | UAC, altitude selector, GPWS | every 0.05 s tick while owning the vertical | UAC keeps `alState` (but a `descend` ages out at 3 s → vs 0); AS leaves OVER after 3 s |
| `<f>_altap` | altitude selector | UAC, GPWS | 1 s, only while a panel is selected | UAC keeps last `TARGET_ALT` |
| `<f>_apswitch_1` | **FMS** | UAC, relays, MANCON, GPWS, #12 | 0.1 s | latched (a dropped packet must not disengage the autopilot) |
| `<f>_apswitch_2` | **altitude selector** | UAC, relays, GPWS, #12 | 0.1 s | latched |
| `<f>_apswitch` (merged) | #12 **Aer Togekiss only** | older relays | 0.1 s | latched |
| `<f>_stab` | UAC | stabiliser relay | every UAC tick | relay zeroes outputs after ~1 s |
| `<f>_navcontrol` | UAC | yaw+thrust relay, strafe relay | every UAC tick | relay zeroes outputs after ~1 s |
| `<f>_alt` | UAC | strafe+lift relay, extra-lift bank, Scout's #84/#85 | every UAC tick | relay holds `ALT_NEUTRAL` on both lines (not zero) |
| `<f>_thrustf` | UAC | finer thrust controller (#69), Scout `thrustcon` (#86) | every UAC tick | engines cut after `LINK_TIMEOUT` (1 s) |
| `<f>_mancon` | MANCON / VSC | UAC, GPWS | 0.05 s, always | UAC: stale > 0.5 s → hover (vs 0), scramble false |
| `<f>_mancon_nav` | MANCON / MIC | UAC | 0.05 s, always | UAC: stale > 0.5 s → no manual horizontal |
| `<f>_mancon_translational` | #13 (retired on Togekiss) | MIC #14 | 0.05 s while manual live | MIC reads keys as up after 0.3 s |
| `<f>_mancon_rotational` | #15 (gated off) / #35 | MIC | 0.05 s while manual live | as above |
| `<f>_engineswitch` | engine-switch panel | every relay | 0.1 s | relay's `mst`/`alt`/`att` stay latched; display says "no signal" |
| `<f>_touchdown_detection` | squat-switch computer | FMS, GPWS | 0.1 s (Scout 0.05 s) | FMS: `TD.fresh()` false after 3 s → AGL fallback |
| `<f>_altoverride` | altitude selector | FMS | on pilot tap only | n/a (an event, not a stream) |
| `<f>_kbdjoy` | keyboard joystick #21 | UAC | 0.05 s | UAC: stale > 0.5 s → stick disarmed |
| `scout_pullup` | #78 | MANCON #76 | 0.05 s, true **or** false | MANCON ages it out at `IN_SCRAMBLE_STALE` |
| `<f>_tri` | UAC | vector engines (Scout, dormant) | every UAC tick | engine computer's own timeout |
| `atc_broadcast` | responders | #44 TMU, #64 AWACS, **every FMS** | 0.05 s | scope coasts, then drops the track |
| `aer_navdata` | #24 ↔ FMSs, ATC, GPWS | | 30 s broadcast + pull/ack | client keeps its local `.db` and flies normally |
| `aer_terrain` | #68 ↔ FMS, GPWS | | 15 s beacon + request/response | every terrain mode disarms; TAVOID changes nothing |
| `aer_atc` | FMS ↔ #87 ATC controller | | 1 s request/response, only while inbound to a facility | FMS: nothing it was told is undone (a hold stays held); an FMS never answered lands as planned. #87: a request not renewed in 5 s is dropped with its reservation |

---

## 2. FMS → UAC

### `<family>_navap` — the navigation target

**The trap:** `msg.y` carries **world Z**, not altitude. It has always been that way (it is
inherited from `navcom.lua`) and both ends agree, but it reads wrong every single time.

```lua
{
  x           = <world X>,       -- number
  y           = <world Z>,       -- number  ** NOT altitude **
  type        = "poi"|"vor"|"end"|"carrot"|"land"|"rwy"|"dep",
  name        = "<waypoint or site name>",
  phase       = "ENROUTE"|"APPROACH"|"TERMINAL"|"ARRIVED"|"LANDING",
  holdHeading = <rad>|nil,       -- LANDING only: the heading to hold
  cruiseCap   = <m/s>,           -- the rise-rate-limited commanded speed (NAV.cmdSpeed)
  schedSpeed  = true|nil,        -- "cruiseCap is a speed to TRACK, not a ceiling"
  carrot      = true|nil,        -- pure-pursue this moving point (velocity-vector law)
  glide       = true|nil,        -- a FLYING descent: you may still bank to steer
  align       = true|nil,        -- route-start ALIGN hold is in effect
}
-- ...or, to disengage entirely:
{ cancel = true }
```

Field-by-field, and why each exists:

* **`phase`** is the one piece of judgement the UAC takes on faith rather than deriving. The
  FMS knows the plan (which leg is final, whether the target is a VOR or a POI) and the
  contextual pass radius, so it decides once. `APPROACH` **does not mean slow** — with
  `schedSpeed` set the UAC tracks `cruiseCap` exactly; what `APPROACH` buys is
  **velocity-vector control instead of a closing-rate loop**.
* **`schedSpeed`** rides on `DC.DYNAMIC_CRUISE`. Without it the UAC falls back to drawing its
  own decelerate-at-every-fix law, and the FMS's displayed CMD becomes a lie.
* **`carrot`** says the point is a moving carrot on the FMS's baked path, so pure-pursue it —
  no separate cross-track leg hold is needed, since pursuing a point *on* the path corrects
  cross-track for free.
* **`glide`** and **`carrot`** are **not the same flag**, and the rollout is where that stops
  being academic. `glide` = "still a flying leg, you may bank"; `carrot` = "track `cruiseCap`
  instead of drawing your own decelerate-to-zero hold". On the rollout the wheels are down
  (no bank — `glide` false) but the brake is a schedule the FMS owns (`carrot` true). Sent
  together, the touchdown jolt was a commanded-speed **step** from 28.7 m/s to a flat 10.0 in
  one tick, answered with full reverse thrust.
* **`align`** routes VTOL/HYBRID translation through the velocity-vector law during the
  route-start hold, so *inherited momentum* is actually braked rather than flown as a
  cross-track hold against a bogus leg. FIXEDWING ignores it (it cannot hover).
* **`type`** is read by nobody in the UAC (it only names the target on its screen); the **GPWS**
  reads `rwy`/`land` as "on final" (07 §`onFinal`), which is why a runway **departure**'s ground
  run says `dep` (02 §7b). The departure otherwise sends exactly the rollout's message —
  LANDING + `holdHeading` + `schedSpeed`, with `carrot` once it rolls — and the UAC flies it
  exactly as it flies a rollout, only with a `cruiseCap` that rises.

**`cancel` is a real disengage**, not a hold-here trick: the UAC drops the target entirely and
coasts (zero thrust/yaw/strafe). Autopilot off should hand control back, not pin the aircraft
to a spot it now has to fight to leave.

**Legs:** the UAC forms `navPrev` from the *previous* target whenever a new one arrives more
than `CFG.WP_EPSILON` away. A same-coordinate rebroadcast is a keep-alive, not a new leg. A
direct-to therefore arrives carrying a **bogus leg** from the old plan's last fix — which is
why the velocity-vector law (which ignores `hasLeg`) is the right one for it.

### `<family>_autoland` — the vertical channel

```lua
{ cmd = "hold",    alt = <world Y>|nil }  -- hold THIS altitude, or "hold where we are now"
{ cmd = "descend", vs  = <m/s, -down> }
{ cmd = "landed",  squat = true|nil }     -- squat: gear contact made → hold DOWN, never lift
{ cmd = "release" }                        -- pilot override: hand the vertical to ALT-AP
{ cmd = "cancel" }                         -- plain teardown: ALT-AP resumes its own pick
{ cmd = "off" }                            -- AP OFF: selector stays deselected
```

Three receivers, three different readings of the same stream:

| | UAC | altitude selector | GPWS |
|---|---|---|---|
| `hold`/`descend`/`landed` | `alState` ← that state; ALT-AP ignored | → **OVER** (all panels green, stops broadcasting, stashes the pick) | "a landing is in progress" |
| `release` | back to `normal`, tracks ALT-AP again | → **PILOT** (keeps the pilot's pick, broadcasts again) | ditto |
| `cancel` | back to `normal` | → **NORMAL**, *restoring the stashed pick* | ditto |
| `off` | back to `normal` | → all deselected and silent | ditto |

> **`cancel` vs `release` is load-bearing and easy to get wrong.** `release` assumes the tap
> that triggered it has already re-selected something, so it restores nothing. `cancel`
> restores the stash. Terrain avoidance's *plain* teardown (hazard cleared on its own) must
> therefore send `cancel`; only the pilot-override path sends `release`. Sending `release`
> there is the bug that dropped the pilot's ALT-AP selection on the floor.

`hold` **seizes the current altitude on entry from any other state**, guarded on
`alState ~= "hold"` so repeated holds cannot ratchet on noise. `msg.alt` overrides that with
an altitude to hold outright — used by the pad `HOVER` phase, where the number to sit on is
`landing.db`'s, not the metre of slop the descent stopped at.

**A runway departure rides this channel too** (02 §7b): `landed` (with `squat`) through the
line-up and the take-off roll, `descend` with a **positive** `vs` (`CFG.DEP.CLIMB_VS`) from
rotation, and `cancel` at `CLIMB_AGL` — so the selector restores its pick for the cruise climb.
Both live messages carry **`dep = true`**; no receiver reads it yet (§9: extra fields are free),
so to the selector and the GPWS a take-off still looks like "a landing in progress" (the GPWS's
terrain modes stand down for the ~10 s of climb-out, as they already do for a go-around circuit).

**A commit that supersedes a plan holding this channel sends `cancel`** (since 2026-10-15). Every
plan commit used to tear the landing down without a word here, and the UAC latches
`landed`/`descend`/`hold` until told otherwise — a plan committed from the LANDED screen was flown
along the ground with the lift cut.

### `<family>_altoverride` — altitude selector → FMS

A bare ping (any message). Sent when the pilot taps a panel while the selector is
overshadowed. The FMS's `overrideLoop` interprets it by context:

| FMS state | Effect |
|---|---|
| flying a go-around/approach (`LND.ga`) | drop `LND.ga`, send `cancel`; the route keeps flying itself at the pilot's altitude |
| landing airborne (ALIGN…MISSED, not ROLLOUT) | `LND.paused = true`, send `release`; alignment keeps running |
| landed | full teardown, back to HOME |
| runway departure, on the ground (ALIGN/LINEUP/ROLL) | send `release`; the take-off run is given up and the plan flown as an ordinary one from where it is |
| runway departure, climbing out (CLIMB) | send `release`; the route flies on at the pilot's altitude |
| terrain-avoidance hold active | release the hold + stand down for `TAVOID.OVERRIDE_COOLDOWN` (30 s) |
| anything else | ignored as a stray |

---

## 3. UAC → thrusters

All four go out **every UAC tick**, unconditionally. See invariant I4.

| Protocol | Message | Units | Read by |
|---|---|---|---|
| `<f>_stab` | `{ pitch, roll }` | signed float `[-1,1]` = fraction of one full redstone line | stabiliser relay |
| `<f>_navcontrol` | `{ thrust, yaw, strafe }` | signed float `[-1,1]` | yaw+thrust relay (thrust, yaw), strafe relay (strafe) |
| `<f>_alt` | `{ up, down, cut }` | `up`/`down` each `[0,1]`, **already split**; `cut` boolean | strafe+lift relay, extra-lift bank, Scout #84 (`up`) and #85 (`down`) |
| `<f>_thrustf` | `{ thrust }` | signed float `[-1,1]`, normalised on `CFG.THRUST_MAX` | #69 finer thrust, #86 Scout thrustcon |

Points worth knowing:

* **`thrustf` and `navctrl.thrust` are the same demand at two resolutions.** `thrustf`'s
  `+1` means *full forward authority whatever redstone encoding the receiver uses*;
  `navctrl.thrust`'s `+1` means *one full redstone line*. The two denominators coincide on
  every airframe today (`THRUST_MAX` is 15 everywhere) but they mean differently. **Wire the
  fore/aft thrusters to the finer controller and nowhere else**, or they get two commands and
  the finer one is wasted.
* **`up`/`down` are not a push-pull pair.** They are two independent burns either side of
  `CFG.ALT_NEUTRAL` (the balanced-hover trim), which is why the split stays in the UAC
  (airframe calibration) and why each line gets its own dither accumulator, while an *axis*
  gets one accumulator for both its lines.
* **`cut` is not `up = down = 0`.** Steps 0 is a balanced hover, still burning. `cut` says the
  lift engine is OFF, so the relay bypasses its accumulators rather than feeding them.
* **On a vector-nozzle airframe** (`CFG.TRI_ENABLE`) `navctrl`'s yaw and strafe are pinned to
  zero — the nozzles already made them — while `thrust` stays real. `alt` is not broadcast at
  all. Everything goes on `<f>_tri` as three `{key = {x, y, thrust}}` triplets. Currently
  dormant fleet-wide.
* **On shutdown** the UAC sends zeros on stab/navctrl/thrustf but parks the lift pair at the
  **learnt hover trim**, rounded — the one place in `uac.lua` that rounds anything, because a
  sigma-delta needs a stream and this is the last frame there will ever be.

### Legacy message shapes

Every relay still accepts the **pre-2026-08-26 split-and-whole** shapes
(`front/back/left/right`, `yaw_left/yaw_right/thrust_forward/thrust_backward`,
`strafe_left/strafe_right`, and `up`/`down` as whole levels with no `cut`). This is what
makes the deployment order **RELAYS FIRST, UAC LAST** safe. The other way round feeds `nil`
to `setAnalogOutput` and kills the relay outright. Delete those branches once every UAC on the
fleet runs the float build.

> `alt` is the one protocol whose **field names did not change**. The `cut` flag is the only
> thing telling the two apart, and getting it wrong is not subtle: a legacy `up = 7` read as
> a fraction clamps to 1.0 and puts the lift line hard against 15.

---

## 4. Panels and switches

### `<family>_engineswitch`

```lua
{ master = bool, altitude = bool, attitude = bool, extra_lift = bool }
```

Broadcast at 10 Hz off four redstone inputs (Aer Togekiss #11: `front`/`right`/`left`/`back`).
`extra_lift` is Aer Togekiss's addition; other airframes' panels send three fields. **Scout's
#71 drives all three from one lever** — on that airframe they are the same engines.

Which relay honours which flag:

| Relay | gate |
|---|---|
| stabiliser | `attitude and master` |
| yaw + thrust | `master` |
| strafe | `master` |
| lift (`alti`) | `altitude and master and not cut` |
| extra lift (#15) | `extra_lift and master` |

### `<family>_apswitch_1` / `_2` / (merged)

```lua
{ autopilot_nav = bool }        -- _1, from the FMS's own redstone input
{ autopilot_altitude = bool }   -- _2, from the altitude selector's ('front')
{ autopilot_nav = bool, autopilot_altitude = bool }   -- merged, #12 only
```

The two AP toggles are **physical redstone switches wired to the two brains**, not one panel.
The UAC subscribes to `_1`/`_2` directly. The older relays (`hdgcon`, `strafecon`,
`msg_to_gas_output`) still read the merged name — so on any aircraft without a #12, anything
depending on the merged protocol is dead. (In practice those relay handlers only paint a
status line, so nothing flies worse for it.)

### `<family>_touchdown_detection`

```lua
{ touchedDown = bool }
```

Weight-on-wheels. **Ground truth**, where AGL is only an inference. The FMS guards it twice —
freshness (`TD.STALE` 3 s: without it a stale `true` would cut the *next* descent at altitude)
and a hold time (`TD.HOLD` 0.25 s: one bounce of the contact is not a landing). With no sender
at all `TD.confirmed()` is simply always false and the landing falls back to the AGL test plus
the settle backup.

---

## 5. Manual flight

### `<family>_mancon` — the vertical stick

```lua
{ vs = <m/s wanted climb rate>, scramble = true|nil }
```

`scramble` is a **flag, not a big number**. It used to be sent as `vs = 100`, which could
never work: the UAC clamps the wanted rate *before* the rate loop sees it (today
`CFG.MANCON_CLIMB_MAX`, 7.5 m/s on Aer Togekiss; `CFG.CLIMB_MAX`, 3.75, when this was written),
so the top detent did nothing but repeat the second-top step. A receiver that predates the flag
ignores it and keeps flying the rate beside it — degrading to "no scramble", not to "100 m/s
runaway".

`vs` is a **spring** since 2026-09-08, not a trim: it decays to 0 when the pilot's vertical keys
are up, including throughout a scramble (which the UAC ignores `vs` for anyway). A receiver that
latched the last value it saw would now read a hover as a held climb — nothing does, but it is
the assumption that changed.

`scramble` is latched **inside the same freshness gate** as `vs`, so a stick that goes quiet
mid-scramble ages out rather than leaving the lift engine pinned open.

### `<family>_mancon_nav` — the horizontal stick

```lua
{ fwd = <m/s, +forward>, strafe = <m/s, +right>, yaw = <rad/s, +nose-right>, active = bool }
```

Broadcast **every tick, zeros included**, so the UAC starts nulling drift on the first tick
after AP-NAV drops rather than waiting for a first packet. `active` says a key is currently
held (vs. sticks centred).

### `<family>_kbdjoy` — the keyboard joystick (#21, FIXEDWING)

```lua
{ pitch = <rad target>, roll = <rad target>, throttle = <0..1>, active = bool }
```

A **target attitude, not a rate** — trim-style, holds where the pilot left it. Acted on only
in FIXEDWING and only for the axes the autopilots do not own.

### `scout_pullup` — a remote scramble request

```lua
{ scramble = bool }
```

Sent **every tick, true or false alike**, so silence is how a crashed sender releases the
pull-up. The MANCON ORs it with its own key and lever and remains the sole broadcaster of
`scout_mancon`. Set `CFG.IN_SCRAMBLE = false` on a hull with no such source (that is a real
answer and is canaried as one).

---

## 6. `atc_broadcast` — fleet-wide, one-way

```lua
{
  x, y, z,                  -- y is ALTITUDE here (pose.y - ALT_OFFSET), unlike navap
  gs, vs, ys,               -- ground speed, vertical speed, yaw rate
  vx, vz,                   -- horizontal velocity VECTOR (added for ATC's ground-track leader)
  hdg,                      -- compass degrees
  callsign, callsign_short,
  is_ifr,                   -- = AP-NAV on
  tcas_status,              -- "CLEAR" | "Vx" | nil
}
```

Broadcast at 20 Hz by each `responder.lua`. **Receive-only at EVERY other end** — #44 paints
what it hears and never answers, and since 2026-09-17 every FMS listens too (`TFC`, for the
HSI's PLAN view) and likewise never answers. Since 2026-09-28 the ATC controller (#87) listens
as well: this channel is the **only** thing it decides occupancy from, and it answers on its
own protocol (`aer_atc`, §8a), never on this one. That asymmetry is what makes a new listener free
to add: a second *broadcaster* is what the one-broadcaster rule forbids, and there is still
exactly one per aeroplane.

`vx`/`vz` are extra fields; older receivers ignore them. They matter because the scope draws
the leader line along the **ground track**, and on a VTOL the track and the heading are
routinely 90° apart.

Every receiver keys tracks by **sending computer id**, not by callsign: two aircraft cloned
from one `responder.lua` with the callsign line unedited is a real and easy mistake, and keying
on callsign would silently merge them into one blip that teleports. Reports are read
**defensively** (a string where a number belongs would otherwise throw inside the draw and
leave the scope stuck on an error message with the aeroplanes still out there).

Own-ship is filtered out **by callsign** — identity is the question, not which transmitter a
report came from. `CFG.STATION.OWN_CALLSIGN` does it on the AWACS; on an FMS it is
`CFG.TERR.CALLSIGN`, reused rather than copied, because a hull has one name and a second copy
of it is a second thing to get wrong. Without the filter the aircraft plots itself as a
permanent contact under its own symbol — and on the AWACS auto-range counts it as traffic at
zero metres and never zooms out.

---

## 7. `aer_navdata` — the fleet navigation database

```lua
srv -> all  { t="sync", rev, stamp, points=<text>, courses=<text>, landing=<text>, sites=<text> }
cli -> all  { t="pull" }                                  -- "I just booted, send me the set"
cli -> srv  { t="ack", rev, stamp, held, screen, fault, who }
```

* **The wire format is the file itself.** The three `.db` files travel as plain text and a
  client writes them to disk and calls its own existing loader. One parser in the fleet; no
  way for the server's idea of the format to drift from a client's.
* Clients compare **`stamp`** (a checksum of the payloads), not `rev`. Content equality is the
  real question and it survives the server being rebuilt with its `rev` file lost.
* `held` + `screen` say "I have a newer revision but I am parked on a screen that lists or
  edits data". `fault` says "I cannot write it" (a full disk). Both are reported so a client
  parked on a data screen does not look identical to one that never heard the server.
* Clients: every FMS, the ATC scope (#44), the GPWS (#67/#70/#71), the ATC controller (#87).
  The scope subscribing is why the radar map is the same database the aircraft fly.
* **`sites`** (2026-09-28) is `sites.db`, read only by #87; every other client ignores the
  field. It is folded into `stamp` **only when non-empty**, so a server with no sites stamps
  exactly what it always did — and a sites-only edit still changes the stamp, which is the
  only thing that tells #87 to reload (every other client then re-applies its three unchanged
  files once, harmlessly). #87 treats a sync **without** the field (an older server) as "keep
  the `sites.db` on this disk", never as "no sites".
* **The server re-reads its own three files before every send**, periodic or in reply to a
  `pull`, so a sync can never be older than #24's disk — a file hand-edited under the running
  server goes out on the next broadcast. It refuses to adopt a missing or emptied file
  ([08](08-ground-stations.md) §1).

Client-side safety (identical in `autopilot.lua`, `atc.lua` and `gpws.lua`):

1. **Sanity** — a payload must still look like a points file (`%a+=%-?%d+,%-?%d+`) before it
   may replace one. Courses and landing may legitimately be empty; points cannot.
2. **Stage then swap, all three together.** Opening a file for writing truncates it
   immediately, and a CC computer has a hard 1 MB space limit — so the obvious version
   destroys the database the instant the write fails. Each file is written to `<name>.new`,
   the length on disk is checked (not the return of `write`), and only then are all three
   renamed. A part-applied refresh is worse than none.
3. **Deferred while unsafe.** The FMS applies only on HOME or STATUS; a picker would renumber
   under the pilot's finger between the draw and the tap.

**Data flows one way.** Edits made on an aircraft's own EDIT DATA screens still work but are
transient: the next sync overwrites them. Changes meant to last belong on #24.

---

## 8. `aer_terrain` — the terrain database

```lua
srv -> all  { t="beacon", id, rev, tiles, geo }              -- every BEACON.INTERVAL (15 s)
cli -> all  { t="hello", callsign, x, y, z }                 -- answered with a targeted beacon
cli -> srv  { t="corr", id, x, z, dx, dz, step, n, w, alt, callsign }
srv -> cli  { t="corr", id, x, z, dx, dz, step, n, w, prof }
cli -> srv  { t="map",  id, x, z, cell, nx, nz, sub, pack, alt, callsign }
srv -> cli  { t="map",  id, x0, z0, cell, nx, nz, sub, grid [, pack, obs] }
cli -> srv  { t="tile", rx, rz }  ->  { t="tile", rx, rz, data }
cli -> srv  { t="gnd",  x, z }    ->  { t="gnd",  x, z, rec }
```

The server is **stateless** and answers a broadcast as readily as a direct send, so a client
that has not yet learnt its id is not stuck waiting for a beacon.

**Record format** — one record per chunk (16 × 16 m), one level per 8 m of Y:

| byte | meaning |
|---|---|
| `TERR` | top of the **ground** as a level; `255` = never surveyed. **Rounded UP** |
| `OTOP` | top of anything **floating** above that ground, or `255` |
| `OBAS` | underside of the lowest floating thing, or `255` |
| `FLAG` | `MASS=3` (2-bit class), `WATER=4`, `MULTI=8`, `TREES=16`, `SPIKE=32`, `VOID=64`, `NODATA=128` |

Every height reported is the **top of its cell**, so the ground rounds up by as much as 7 m.
That is the safe direction and it is deliberate: a GPWS that thinks the hill is 7 m taller
calls early; one that thinks it is 7 m shorter calls late.

`corr` (corridor) reduces across the **corridor width**, so the client gets the worst ground
in the swathe, not one line of it. `map` returns a **north-up, axis-aligned** block —
north-up on purpose, so one grid survives a whole turn and the client rotates it into its own
heading-up frame for two multiply-adds a cell.

`pack` cuts the map reply to **one byte per cell** (255 never surveyed; terrain level in bits
0–5, 63 = charted but no ground; water in bit 6) plus a sparse 5-byte-per-entry obstacle list
for the 1–2 % of cells carrying a floating mass. A quarter of the width, which is what makes a
grid fine enough for subpixel drawing affordable.

**The flag bits are the wire format** and are mirrored (not shared) in `apconf.lua`'s
`CFG.TERR.FLAG`, `CFG.TAVOID.FLAG`, `gpwsconf.lua`'s `CFG.TERRAIN.FLAG` and
`terrainconf.lua`'s `CFG.FLAG`, plus `terrainbuild.jar`. **All of them must agree.**

Clients: the FMS (twice — `TERR` for the HSI overlay, `TAVOID` for the altitude floor, sharing
one `rednet.receive` because two receives on one protocol would race for the same messages)
and the GPWS (Mode 2, Mode 4, FLTA, and the `TERR` height datum).

---

## 8a. `aer_atc` — landing clearances (the automated controller)

```lua
fms -> #87  { t="req", cs, fac, x, y, z, wide, rwyonly, stage }     -- 1 s, while inbound
#87 -> fms  { t="clr"|"divert"|"hold"|"none", cs, fac, to, why, q } -- rednet.send, to the asker
```

* **Who asks.** An FMS with `CFG.ATC.ENABLE`, AP-NAV on, flying a plan that lands at `fac`,
  once the facility is within `CONTACT_M` plus its own stopping distance — and from then on
  every `INTERVAL` until the landing is `LANDED`, the plan ends, or the pilot releases ATC. It
  broadcasts until the controller has answered, then sends to it directly.
* **`cs`** is `apconf.lua`'s `TERR.CALLSIGN`. It is how the controller tells the asker's **own**
  responder report from somebody else parked on the facility, so it must equal that hull's
  `responder.lua` `CALLSIGN` — otherwise an aircraft that has landed is told to hold for itself.
* **`wide`**, **`rwyonly`** are the flags `sites.db`'s remarks may refuse (`atcctlconf.lua`'s
  `CFG.REMARKS`: `narrow` refuses `wide`) and "never a pad" (a FIXEDWING). A flag an FMS does
  not send reads false.
* **The answer** echoes `fac` (the question), so an answer that arrives after the aircraft has
  already switched facility is recognisably about something else and ignored. `to` is the
  facility assigned; `why` one line for the pilot; `q` the place in the site's hold queue.

| `t` | FMS does |
|---|---|
| `clr` | nothing — it lands as planned; ends a hold in force |
| `divert` | replaces the plan with a DIRECT-TO-LAND `to` (`setDirectTo`, so its own published approach and runway final come with it) |
| `hold` | stops on the path (`HOLD = "hover"`: `cruiseCap` 0 + `align`) or orbits (`"orbit"`), and may not engage the landing |
| `none` | nothing — `fac` is in no site, uncontrolled |

**One answerer.** Exactly one controller may run on this protocol: two would hand an aircraft
two different clearances a second apart, which is invariant I1 with a different failure mode.

**Committed.** A `hold`/`divert` that arrives while the landing is below the flare
(`FLARE`/`PROBE`/`HOVER`/`ROLLOUT`/`LANDED`) is ignored. Before that it **aborts** the landing
with `{cmd = "cancel"}` on `<f>_autoland` — the AS restores the pilot's selection, i.e. the
aircraft climbs back up — because a facility turning occupied under a descending aircraft is
the case this exists for (a parked aircraft at a remote field is silent until its chunk loads).

---

## 9. Adding a field, safely

1. **Extra fields are free.** A receiver that does not know about them is unaffected — that is
   how `vx`/`vz` reached `atc_broadcast` and how `scramble` reached `mancon`.
2. **Never add a second broadcaster.** If another computer needs to influence a channel, give
   it its own protocol and have the existing broadcaster OR it in (the `scout_pullup` pattern).
3. **If the receiver must read it every tick, canary it.** Add it to the conf's required list
   so an out-of-date conf fails at **boot** with a name, not mid-flight with a nil.
4. **Deploy receivers first.** Then senders. Keep the legacy branch until every sender is
   converted.
5. **A boolean added to a conf will not crash if it is missing** — it reads nil, evaluates
   false, and the feature quietly does nothing. That is worse than a crash, so booleans go in
   the canary list too (`CFG.MANCON_SCRAMBLE` is there for exactly this reason).
