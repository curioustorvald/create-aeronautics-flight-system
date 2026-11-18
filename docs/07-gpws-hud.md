# 07 — HUD and GPWS (`gpws.lua`)

*#67 Aer Togekiss · #70 Mantine · #71 Scout. One program, byte-identical on all three;
everything airframe-shaped is in `gpwsconf.lua` beside it. On Scout it runs with
`engineswitch.lua` open in a tab.*

---

## 1. The two rules

**IT CONTROLS NOTHING.** Every rednet link is receive-only and every number on the screen is
either read off `sublevel` or heard on a channel the flight system was already broadcasting.
Nothing here can move a thruster. *A warning system that can fly the aeroplane is a second
autopilot nobody asked for — and one that fails silent is better than one that fails flying.*
The single transmission is a nav-data acknowledgement to #24, a roster ping.

**THE HUD IS MOUNTED ABOVE THE PILOT'S EYE LINE**, so its layout is **inverted** with respect to
a panel display: the *lowest* rows sit nearest the natural line of sight and carry the most
important symbology. Reading **up** from the bottom edge:

```
   route                       ← dropped first as the monitor shrinks
   attitude field  (horizon, pitch ladder, bank scale, FPV)
   annunciators    (AP-NAV / AP-ALT / engine / squat / mode)
   heading scale   ┐ two rows or none — an index caret with no scale
   heading index   ┘  under it points at nothing
   speed / height / sink
   ALERT BANNER                ← never dropped
```

Self-status and the armed/inhibited line are on the **computer's own screen**, not the HUD.

---

## 2. What it has, and what it is worth

There is **no radio altimeter** on any of these airframes and no scanner. What there is, is a
**survey**: #68 serves a database built from the world's own region files, one record per chunk
and one level per 8 m of Y.

So every terrain-derived mode — Mode 2, the true form of Mode 4, and the forward-looking
TERRAIN AHEAD / OBSTACLE AHEAD calls — is **armed behind `TERRAIN.available()`**. When the link
to #68 goes quiet they all fall silent within `CFG.TERRAIN.STALE` and this computer behaves
exactly as it did before the database existed.

Modes 5 (glideslope) and 7 (windshear) remain **off**: neither needs terrain, both need
instruments the airframe does not carry.

**Having terrain is not the same as having something to say about it:**

* Mode 2 needs the gap to be **shutting**, not merely small — level flight over flat ground
  closes on nothing however low it is.
* The forward-looking calls need the aircraft to be actually **going somewhere**
  (`FLTA.MIN_GS`), because "ahead" is meaningless in a hover.
* Both stand down for a landing the FMS is flying **at a rate it would actually command**, since
  arriving at the ground is the point of one.
* DECISION HEIGHT is an approach callout, so it is gated on being in the terminal area at all —
  without that, a datum that now exists everywhere makes it fire on every descent in the world.

**Surveyed ground is charted HIGH on purpose** — the top of an 8 m cell, and the highest ground
in a 16 m chunk. Clearance therefore reads **low** and calls come **early**. That bias is the
whole reason the number is safe to warn on.

---

## 3. The height datum — what "RA" on this HUD actually means

A geometric height above a datum this computer can **justify**, and **the HUD always names
which**, so the pilot knows what the number is worth:

| datum | source | why it ranks where it does |
|---|---|---|
| **SITE** | the published elevation of a nearby `landing.db` field | surveyed data about the exact patch of ground being flown at, live on approach and departure — where GPWS earns its keep. Outranks TERR because it describes the actual pad, not the highest block in a 16 m square containing it |
| **TERR** | the terrain database's ground directly beneath | good to within the 8 m cell plus whatever relief a chunk hides, and biased high. Available wherever the world has been surveyed — this is what put an RA on the HUD *en route* |
| **DEP** | the elevation the wheels last left | trustworthy near where it was taken and nowhere else, so it expires with **distance**, not with time. Below TERR because it describes ground that may be a kilometre behind |
| **nil** | — | RA reads dashes and every height-dependent mode inhibits itself. Still the honest answer, and still what happens off the edge of the map |

`DATUM.onSite` is the only place a published elevation may become the datum, because it is the
only place that elevation describes the ground: over the pad square (`PAD_HALF`), or over the
runway strip projected onto its axis (`RWY_HALF_W`, plus `RWY_OVERRUN` at each end for the flare
and the rollout). The runway arming radius matches the FMS's own approach-arm distance so the
two arm together.

> **Note what is deliberately NOT here: no "settled and still" test that would latch a datum from
> a steady hover.** A VTOL holding station at 200 m looks identical to one sitting on a pad, and
> a datum invented there would put the ground 200 m too high in every envelope below.

---

## 4. The alert set

Sorted once by priority. **PRIO is the whole arbitration**: the highest active alert is the one
on the banner *and* the one in the speaker, and a higher one interrupts a lower one mid-word.

| prio | id | text | level | mode |
|---|---|---|---|---|
| 100 | `pull_up` | PULL UP | warn | 1 |
| 95 | `terrain_hard` | TERRAIN TERRAIN PULL UP | warn | 2 |
| 90 | `terrain` | TERRAIN | warn | 2 |
| 85 | `ap_disconnect` | AUTOPILOT DISCONNECTED | warn | — |
| 70 | `terrain_ahead` | TERRAIN AHEAD | caut | FLTA |
| 65 | `obstacle_ahead` | OBSTACLE AHEAD | caut | FLTA |
| 60 | `too_low` | TOO LOW TERRAIN | caut | 4 |
| 55 | `dont_sink` | DON'T SINK | caut | 3 |
| 50 | `sink_rate` | SINK RATE | caut | 1 |
| 45 | `bank_angle` | BANK ANGLE | caut | 6 |
| 40 | `decision` | DECISION HEIGHT | info | 6 |

`GPWS.last` records what the **banner showed** (from `GPWS.top()`), not the rising edge of
whichever mode tripped last — three modes routinely go up together and the one that rises last is
an artefact of the order `tick()` runs them in. *The pilot did not see FLTA's TERRAIN AHEAD; they
saw Mode 2's TERRAIN TERRAIN PULL UP, because that is what outranked it, and that is what the
recall has to name.* This is what lets the display answer "what was that?" about three seconds
after the banner has cleared.

---

## 5. The modes

### Mode 1 — excessive descent rate

Height and sink rate alone, **which is exactly why it is the mode that survives having no terrain
data.** A two-level envelope table (`sink` → SINK RATE, `pull` → PULL UP) interpolated on RA, with
hysteresis.

Uses the **filtered** vertical speed by default (`MODE1.USE_FILTERED`): an airframe oscillating
about zero is not sinking, whatever the instantaneous number says.

### Mode 2 — excessive terrain closure

Alerts on **the climb rate that would be needed to clear the worst terrain ahead**, not on how
near that terrain happens to be. Proximity alone said the same thing at 30 s as at 2 s and
annunciated for whole legs over ground the aeroplane was clearing perfectly well.

`ESCAPE_CAUTION` 1.5 m/s → TERRAIN; `ESCAPE_WARNING` 3.0 m/s → TERRAIN TERRAIN PULL UP.
Hysteresis both ways, or the annunciator stutters as the profile refreshes.

### FLTA — forward-looking terrain and obstacle avoidance

The **same escape rate at a gentler threshold** (`ESCAPE` 1.0 m/s), so the three calls form a
ladder off one number rather than three modes arguing about the same ground. Needs
`gs ≥ FLTA.MIN_GS` (10 m/s).

`OBSTACLE AHEAD` is the same test where the threat is a floating mass rather than ground.

### Mode 3 — altitude loss after take-off

Arms at lift-off; **disarms for the rest of the flight** on the first of: climbing `ARM_GAIN`
(150 m) above the pad, or simply having been airborne longer than `WINDOW` (60 s) — a low-level
cruise is not a departure. Re-armed by the next lift-off.

Trips on losing `max(LOSS_MIN, LOSS_FRAC · gained)` from the peak.

> **A commanded arrival is not a mistake.** A short hop stays inside `WINDOW` and never climbs
> `ARM_GAIN`, so without `LANDING_INHIBIT` the descent onto the destination pad rings the whole
> way down — which is exactly what it did.

The peak is reset on the **estimated** climb rate: a raw VS oscillating about zero crosses
`CLEAR_ON_CLIMB` twice a second and would keep resetting the peak, quietly disabling the mode.

### Mode 4 — unsafe terrain clearance ("TOO LOW TERRAIN")

The real Mode 4 compares radio altitude with terrain **and with the landing configuration**;
gear and flaps do not exist here. What runs is the datum part: **fast and low** over a field we
have an elevation for, with no landing in progress. The floor **rises with ground speed** — 15 m
is unremarkable in a hover and reckless at 300 m/s.

Starts disarmed; armed by climbing above `ARM_RA`, disarmed again on touchdown.

### `GPWS.approaching()` — "this is an arrival, not a mistake"

Descending at a **sane** rate toward a field whose elevation we actually have, and either within
`APPROACH_NEAR` or **closing** on it. That is what an approach looks like from here whether the
FMS is flying it or the pilot is. `APPROACH_MIN_VS` matters: **level flight is not an approach.**

### Mode 6 — callouts

* **DECISION HEIGHT** at `DH` (30 m — the same number as `apconf.lua`'s `RWY_MIN_AGL`). One per
  approach. **Starts disarmed** and is armed only by climbing through `DH + RESET`; losing the
  datum does not arm it, because arriving from cruise makes RA reappear as a large number and
  arms it on the way past — which is the moment that actually means an approach has begun.
* **BANK ANGLE** at `BANK_CAUTION` (55°) — datum-free, so it works everywhere including en route
  with no datum. The planner's `DC.MAX_BANK_DEG` is 45°; this leaves it room.

### `GPWS.onFinal()` — the true landing phase

**Not** "the autoland channel is live", because a **go-around keeps the vertical channel** while
`LND.active` is false — so `landingCmd()` is true for the whole circuit and the DH callout rang
all the way round it. **Nor** "near a field", because an approach pattern's fixes are ordinary
route legs flown well inside the arming radius.

The test is `target.type == "rwy" or "land"`, which the FMS sends **only** under `if LND.active` —
the same flag `drawHSI` dispatches on. One test, and it is the instrument's own. A runway
**departure** (02 §7b) sends the rollout's message on the ground but with `type = "dep"`, precisely
so that it does not pass this test.

**A departure does look like a landing to `landingCmd()`**, though: it streams `landed` through the
take-off roll and a positive-rate `descend` through the climb-out, to `CLIMB_AGL` (30 m). So the
terrain modes stand down for those ~10 s, exactly as they do round a go-around circuit — and the
HUD's autoland line reads `LAND LANDED` / `LAND DESCEND`. Both messages carry `dep = true`; teaching
`landingCmd()` and the HUD line to read it is the change to make if either matters.

### AUTOPILOT DISCONNECTED

Not a GPWS mode; the other thing this speaker is for. Fires on either switch going engaged →
disengaged, and (optionally) **on an engaged channel falling silent**: a dead brain announces
nothing, which from here looks identical to a disengagement and is at least as worth knowing
about. Silence is remembered as `nil`, so a channel that comes back engaged re-arms.

Inhibited on the ground — switching the autopilot off on the pad is not an event.

---

## 6. The look-ahead is cut at the FMS's own next waypoint

`GPWS.lookSecs()` clamps `CFG.TERRAIN.LOOK_AHEAD` to the time to the active target plus
`LEG_MARGIN`. A corridor is a straight line and a route rarely is, so ground the plan turns away
from before reaching is not this aircraft's problem.

*(And note the same trap as everywhere: **`target.y` is the target's world Z**, not an
altitude.)*

`GPWS.lookOnce()` runs the whole look **once per tick** and every mode reads the cached
`GPWS.lk` — otherwise three modes would each decode the same profile.

---

## 7. The voice

* A **raw mono unsigned 8-bit 48 kHz `.bin`** pack on a floppy, found by a marker file
  (`_voicepack.md`). One byte per sample, minus 128. DFPWM and its decoder were removed;
  `CFG.VOICE.CHUNK` is bytes of PCM per push (16 KiB ≈ ⅓ s), which also made preemption three
  times finer.
* **`MIN_DWELL` (0.5 s) gates the speaker.** A clip that is `play()`'d and `stop()`'d very
  quickly is *not played but IS queued*, and then plays ahead of the next commanded clip. So
  `AUDIO.want()` will not look at an alert until it has been continuously up for that long.
* **Latched one-shots are exempt** (DECISION HEIGHT, AUTOPILOT DISCONNECTED) and speak on the
  tick they fire — `GPWS.fire()` has already committed the banner, and `AUDIO.speak()` is only
  ever cut short by a *higher* alert, never by its own condition clearing.
* `GPWS.showing()` and `GPWS.speakable()` split two questions the arbitration used to answer with
  one test: the **banner** holds an alert for at least `MIN_SHOW` (0.5 s) from its rising edge,
  while the **speaker** only ever sees one whose condition still holds. **A one-tick blip is
  therefore seen and never heard.**
* `CFG.TERRAIN.LOOK_AHEAD` carries an extra second over what the geometry needs, to pay the
  half-second dwell back with margin.

---

## 8. `gpwsconf.lua`

| Section | Contents |
|---|---|
| `SHIP` | `NAME`, `CALLSIGN`, `HEADING_SIGN/OFFSET`, **`ALT_OFFSET`** |
| `LINKS` | ten protocols it listens to, plus `NAVDATA` and `TERRAIN` |
| `STALE` | a freshness budget **per link**, each sized to that link's own broadcast rate |
| `NAVDATA` | the #24 subscription |
| `TERRAIN` | the #68 subscription, the corridor request, and the tile cache |
| `DATUM` | the four datum sources' geometry and radii |
| `GPWS` | `INHIBIT` + one table per mode + `VS_FILTER` |
| `VOICE` | pack location, volume, chunking, `MIN_DWELL`, the clip map |
| `ALERTS` | the priority table above |
| `HUD` | scales, signs, palette, redraw rates |
| `SELFTEST`, `DEBUG` | a touch/key-triggered clip run; a runtime debug toggle |

`CFG.DEBUG.ENABLE` is only where the debug flag **starts** — the key toggle moves the live one,
so the pilot can ask the question in the air instead of on the ground.

`SHIP.ALT_OFFSET` is canaried one level down with the rest of `SHIP`, and **must not be
defaulted**: 0 is a legal value, so a silent default puts the radio altitude back to
over-reading by the height of the hull, which on this computer means **every terrain call fires
late**.

---

## 9. Reading the HUD when something is wrong

1. **RA reads dashes** → no datum. Off the surveyed map, no field near, no departure latch. Every
   height-dependent mode has inhibited itself. This is correct behaviour, not a fault.
2. **The datum tag says DEP a long way from anywhere** → it expires with distance
   (`DEP_RADIUS`), so it should drop to nil, not to a wrong number.
3. **Terrain modes silent** → check `TERRAIN.why()` on the computer's own screen; it names the
   reason (disabled, no server, stale, no profile, off-track, nothing charted).
4. **A call that will not stop** → check the hysteresis; every mode has one, and every one of
   them requires the condition to genuinely ease, not merely to flicker.
5. **A call during a landing** → `normalLanding()` only inhibits while the descent is one the FMS
   *would actually command* (`INHIBIT.LANDING_VS`). Arriving at the pad at 20 m/s is not a
   landing, it is an accident in progress, and it is exactly the case the inhibit must not
   swallow.
