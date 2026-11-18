# 02 — The FMS (`autopilot.lua`)

*The flight management system: `autopilot.lua` (~8 100 lines) + `apconf.lua` (~1 380 lines).
Deployed **byte-identical** to #8 (Aer Togekiss), #16 (Citation), #28 (SS Argo), #39 (Mantine),
#74 (Scout) and #46 (the display bench) — all six, as of 2026-10-15 (the runway departure). Everything that differs per
aircraft is in `apconf.lua`.*

---

## 1. What it does and does not do

**Does:** own the navigation database on the aircraft; edit points/courses on a touch monitor;
solve a route to a destination; **bake a flyable path** with jerk-limited corners and a speed
profile; decide the flight phase; **fly a take-off off a runway** (§7b); fly a pad landing or a
runway approach; fly a published approach and a published go-around; hold an automatic
terrain-avoidance altitude; draw the HSI/landing map; announce AP-NAV; keep its `.db` files in
step with #24.

**Does not:** command an attitude, a bank, or a thruster. Decide when the wheels are down
(that is the squat switch; AGL is the fallback). Avoid terrain laterally. Do anything at all
while AP-NAV is off, except keep its own state current and keep drawing.

**Its whole output is three things:** a point, a speed, and a vertical rate or altitude.

---

## 2. Loops

`parallel.waitForAny(navLoop, uiLoop, overrideLoop, hsiLoop, NDU.loop, TD.loop, apAnnounceLoop, TERR.loop, TFC.loop, ATC.loop)`

| Loop | Rate | Job |
|---|---|---|
| `navLoop` → `navTick` | `CFG.TICK_INTERVAL` = 0.05 s | pose, phase, waypoint advance, landing machine, vertical broadcast |
| ...plus the ping | `CFG.PING_INTERVAL` = 0.2 s | re-broadcast `navap` as a keep-alive |
| `uiLoop` | event-driven, **repaint gated at `UI_REDRAW_MIN` = 0.1 s** | the touch UI, **and the HSI's view toggle** (§11). It still *wakes* on every event — `parallel` hands every event to every branch — but it repaints at most 10×/s unless something changed (see §10) |
| `overrideLoop` | 1 s receive | the altitude selector's override ping |
| `hsiLoop` | `CFG.HSI.REDRAW` = 0.05 s | the external monitor |
| `NDU.loop` | 1 s receive | nav-data sync |
| `TD.loop` | 1 s receive | squat switch (its own loop because the guards are `os.clock()` stamps, not tick counts) |
| `apAnnounceLoop` | `CFG.APSWITCH.INTERVAL` = 0.1 s | read own redstone → `apswitch_1` |
| `TERR.loop` | 0.25 s receive | terrain server: **shared** by `TERR` and `TAVOID` (two receives on one protocol would race) |
| `TFC.loop` | 1 s receive | traffic: `atc_broadcast`, **receive-only**, for the PLAN view. Runs whatever view is up — contacts age out in seconds, so a roster that only filled while PLAN was selected would be empty at exactly the moment the pilot switched to look at it |
| `ATC.loop` | `CFG.ATC.INTERVAL` = 1 s | the automated controller (§8a): sends the question, keeps the latest answer. **Acting** on it is `ATC.tick`, inside `navTick`, so a divert or a hold changes the plan at a tick boundary |

`broadcastCurrent()` is called on the 0.2 s ping **and immediately** on any waypoint advance,
landing engagement or speed-mode change — the ping is a keep-alive, not the control rate.
`broadcastVertical()` is called **every tick** from `navTick`, because the flare's rate changes
with altitude.

---

## 3. State: three tables, three lifetimes

| Table | Lifetime | Holds |
|---|---|---|
| `NAV` | a plan | points (a **snapshot**), index, phase, cruise mode, live pose, `cmdSpeed`, `aligning`, `fromStart`, the chosen approach |
| `LND` | a landing | phase, pad geometry **or** runway geometry, commanded `vs`, go-around/approach flags, gate readouts |
| `TAVOID` | a hazard | cached corridor profile, `active`, `holdAlt`, override cooldown |
| `DEP` | a take-off | phase, the runway being departed (a **snapshot**), line-up point, roll command, live runway coordinates — see §7b |

They are distinct because a plan can be flown long before its landing engages, and the landing
outlives the plan by a moment at touchdown (`NAV` is cancelled, `LND` lingers on the LANDED
screen). `DEP` is the mirror image: it exists only for the first minute of a plan, from the
commit to `CLIMB_AGL`.

`NAV.points`, `NAV.landSite` and `DEP`'s runway are **copies**, so editing
`points.db`/`landing.db` mid-flight (by hand, or via a #24 sync) cannot retarget a landing or a
take-off already under way. This is invariant I6.

---

## 4. The path planner (`PLAN`)

Built **once**, when the pilot commits a plan (`setDirectTo` / `setRoute` /
`RTR.setSolvedRoute`). It replaces a per-tick Catmull-Rom spline.

### The corner

Each interior waypoint gets an explicit

```
straight → clothoid (curvature 0 → 1/R) → circular arc → clothoid (1/R → 0) → straight
```

fly-by transition. Curvature ramps *linearly* through each clothoid instead of jumping at a
spline knot, so lateral acceleration **and its rate of change (jerk)** are continuous through
a turn.

* Radius: `R = V² / (g·tan(bank))` with `DC.GRAVITY = 11` (Minecraft's) and
  `DC.MAX_BANK_DEG = 45`. **One physical knob that means the same thing whatever flight mode
  the UAC is presently in** — corner geometry works across VTOL/HYBRID/FIXEDWING without the
  FMS knowing which.
* Clothoid length: `Ls = V³ / (R · DC.JERK_MAX)`.
* Clamped to `[MIN_RADIUS, MAX_RADIUS]`, then **shrunk to fit** `DC.LEG_FIT_FRACTION` (0.45) of
  the *shorter* adjacent leg. Kept well under 0.5 so two corners sharing one leg, fed from each
  end, can never overlap even if both want their full share.
* Solved by a decay search, not bisection: `T(R)` is not guaranteed monotonic once `Ls` also
  grows as `R` shrinks. This is offline, so a few dozen extra steps cost nothing.

> **KNOWN FAULT (2026-09-17, open): an exact 180° reversal hangs the build.** The tangent length
> is `T = k + (R+p)·tan(Δ/2)`, and at Δ = π that `tan` is 1.63e16 — not an infinity, not an
> error, just a number. `T` comes back at ~1.1e18 m whatever the radius (the shrink search runs
> all 80 steps and cannot help, because the `tan` is the whole of it), `PLAN.buildGeometry`
> places the transition start that far back along the incoming leg, and
> `PLAN.marchStraightInto` is then asked for **2.2e17 samples** — a `for i = 1, n` with a
> `table.insert` in it. The FMS never returns.
>
> It is reachable from ordinary data: the auto-router's graph is **undirected**, so a plan may
> legitimately run out to a fix and straight back down its own track, and fixes that share an x
> or a z make the two bearings differ by *exactly* π. Measured case — aircraft at TMA, FLY ROUTE
> → AGC, solved as `TMA TAIMW TMV MDARO …`, where TMA (609,893), TAIMW (609,1331) and TMV
> (609,598) share x = 609. `74/autopilot.lua`'s route tracer is what reads this off:
> [12-diagnostics-and-tools.md](12-diagnostics-and-tools.md) §10.
>
> **Confirmed in the aircraft 2026-09-17** (Scout #74, FLY ROUTE → FGY from the TMA/TMV strip):
> CC's watchdog killed the FMS at `autopilot.lua:1388`, the `table.insert` in that loop. The
> trace read `PLAN/TURN TAIMW 180`, `CORNER/EXIT … 80 shrink steps … T = 1.11406e18, fits=false`,
> `MARCH/STRAIGHT … 2.22813e17`.
>
> **The planner is still exactly this fragile.** What changed is that nothing hands it a
> reversal any more — `collapseReversals` drops the apex from the assembled point list (§4a) —
> so the reachable cause is gone while the mechanism is not. Anything that reaches `PLAN.build`
> without passing that pre-pass will still kill the FMS, and the watchdog takes the *whole
> program* with it: in the air that also stops `navap` and stales the UAC's link.
> The planner-side candidates are still open and still in `CLAUDE.TODO.md`: a clamp on Δ, a cap
> on `T` against `maxT` in the solver, and a sanity limit on the sample count in the marches.

`PLAN.transHdg` is **exact** (curvature is piecewise-linear in arc length, so heading is
piecewise-quadratic); only *position* needs numeric integration (`PLAN.marchTransition`).

### 4a. No point list may contain a reversal (`collapseReversals`)

A **precondition of the planner**, enforced as a pre-pass on the assembled point list —
`collapseReversals(pts, tag)`, called by `setDirectTo`, `setRoute`, `RTR.setSolvedRoute` and
`GA.buildWp` as the **last** thing before `PLAN.build`, after every other mutation.

It drops the apex of any turn ≥ `REVERSAL_DEG` (175°), re-testing after each cut. Safe in a way
dropping a fix generally is not: the three fixes are within 5° of collinear, so the direct leg
between the apex's neighbours lies within `leg·sin 5°` of the track the out-and-back already
flew — 64 m on the longest leg in this database, well inside `DC.PASS_CORRIDOR`.

**Why it is not in the auto-router**, which is where it was first written: the router is only one
source of a reversal. `APR.prep` splices a published approach in *after* the route is solved, and
a procedure that enters from beyond the field is a reversal of its own — `NUN_APPR1` enters at
BLTAG, 1 465 m north of NUN, so a plan arriving from the south turns exactly 180° onto it.
`course.db` routes and `goaround-wp` circuits are two more sources. One pass over the assembled
list catches all of them; a pass over the solved route catches one in three.

Two consequences worth knowing:

* `NAV.apprFrom` is **kept pointing at the pattern, not at the slot**: a cut before it shifts it
  down, a cut *at* it leaves it pointing at what followed.
* If the collapse eats **every** fix of a spliced pattern — which is what happens to `NUN_APPR1`,
  a procedure turn whose fixes are both reversals — the approach record is **cleared**
  (`NAV.apprName/apprFrom/apprAlt/apprGa`), so the arrival does not carry the published minimum
  and the missed-approach circuit of a procedure that is no longer in the plan. The `PLAN`
  stage's `COLLAPSE` and `APPRGONE` rows in a route trace say when this happened.

**The cost is real and is the lesser evil.** Dropping a procedure fix means not flying the
published arrival as printed; the alternative is the FMS dying on the approach. The proper answer
is a procedure turn — a teardrop, which is what a chart means by that shape — and that is planner
work, in `CLAUDE.TODO.md`.

Measured over the whole database (every point as a start × every hub as a destination, 3 925
routes): the angle data has a clean gap either side of the threshold — **the largest turn left
alone is 172.41°, the smallest collapsed is 175.56°**, and 105 of the collapses are at exactly
180.0000°.

### The speed profile

1. Per-sample **curvature limit** `v = sqrt(turnAccel / |κ|)`.
2. **Two-pass** profile (`PLAN.bakeSpeedProfile`): backward braking sweep from the route end at
   `DC.BRAKE_DECEL`, then forward acceleration at `DC.ACCEL_MAX`. This runs once over the whole
   route, so there is **no look-ahead distance limit**.
3. A **separate, steeper backward-only sweep** (`PLAN.applyArriveBrake`, `DC.ARRIVE_DECEL`)
   min'd in. Corner braking is tuned for comfortable cornering and can legitimately leave the
   craft at full cruise right up to the last waypoint on a gentle final; this guarantees the
   destination stop is never out-run however short the final leg is.
4. The whole thing **iterates** (`DC.PLAN_MAX_ITERS`): each corner is re-solved at the speed
   the baked profile says it can actually be taken at, because a leg-fit-clamped corner cannot
   be flown at the speed it was guessed at. Converges on `DC.PLAN_CONV_EPS` metres of radius
   change.

### Runtime lookups

Everything is served off the arc-length-sampled polyline (`DC.SAMPLE_SPACING` = 5 m):

* `PLAN.updateCursor(x,z)` — nearest sample, searched in a ±120-sample window around the
  cached cursor, with one full O(n) rescan if that leaves us > 1000 m off (route resume,
  teleport). Cached against repeated same-position calls in one tick.
* `PLAN.carrot(L)` — the point `L` metres ahead along the path.
* `PLAN.speedAt()` — the baked profile, linearly interpolated.
* `PLAN.course()` — DTK and signed cross-track from the path (for the HSI).
* `PLAN.passed(i)` — has the arc-length cursor crossed waypoint `i`'s baked "abeam" point?

### `PLAN.onPath` — the gate on all of it

Everything the plan can tell you is read by **projecting the aircraft onto the polyline**, and
that projection only means anything while the aircraft is near it. Off the path — a plan just
committed from elsewhere, a go-around circuit starting *behind* the aeroplane, a diversion, a
teleport — the honest answer is "navigate straight to the active waypoint until you have
joined", which is what every caller falls back to.

`onPath` = within `DC.PASS_CORRIDOR` (400 m) **and not** `NAV.fromStart`.

> Without it, a go-around projected onto its own new circuit at the far **end** of the path —
> the aircraft being beside the runway, which the circuit finishes at — and declared the whole
> circuit already flown, re-arming the approach in the same tick it was abandoned.

`NAV.fromStart` is the explicit "this plan must be flown from its first waypoint, not joined at
the nearest point" flag. Set by a go-around and by a spliced-in approach; cleared the moment
the first waypoint is genuinely reached.

### The carrot lookahead is a TIME, floored by a distance

`DC.CURVE_LOOKAHEAD_T` (3.4 s) with `DC.CURVE_LOOKAHEAD` (200 m) as a floor, evaluated against
`NAV.cmdSpeed` (already rate-limited, so the carrot cannot jitter — and it pulls **in** as the
profile slows for a corner, tightening tracking exactly where the corner needs it).

**Why a fixed distance was a bug:** pure pursuit's outer-loop gain is `V/L`, so a fixed carrot
distance makes the guidance gain rise in direct proportion to speed. 200 m is 2.5 s for a
Citation at 80 m/s and **0.62 s** for Aer Togekiss at 320 — which puts the *outer* loop faster
than the inner lateral-velocity loop it commands (~1.18 /s), and is barely more than the ~0.5 s
transport delay. Simulated at 320 m/s with a 20 m lateral upset:

| lookahead | result |
|---|---|
| 200 m (0.62 s) | weave 3.3 s, ζ ≈ 0.01, ±5.7° of nose sway ← the reported fault |
| 400 m (1.25 s) | marginal, ±2.9° |
| 800 m (2.50 s) | no weave, ±1.4° |
| **1100 m (3.44 s)** | no weave, ±1.0° ← chosen |

Note the weave got **slower**, not better, when the UAC's inner loops were fixed (2.8 s →
3.3 s). Damping the inner loop makes it track the guidance demand faithfully, which *exposes*
an outer-loop problem sloppy tracking had been masking. **A longer period after an inner fix is
the signature of an outer-loop fault.**

---

## 5. Phase, radii and waypoint advance

```
ENROUTE ──(final leg, outside pass radius)──▶ APPROACH ──▶ TERMINAL ──▶ ARRIVED
                                                (inside radius)  (inside radius·ARRIVED_FRACTION)
```

Pass radii (`CFG.RAD`) are contextual: `VOR_RADIUS` 240 m (a fly-by fix at cruise),
`POI_ENROUTE_RADIUS` 120 m, `POI_FINAL_RADIUS` 60 m. A VOR that lands on the **final** leg gets
the tighter POI radius — it is being approached slowly, not overflown at speed.

**A waypoint counts as passed** by *either* flying inside `advanceRadiusFor()` (a turn-aware
multiple of the base radius: gentle turn → wider, cut the corner; sharp → tighter) **or** by
`PLAN.passed()` crossing its baked abeam point. Either is sufficient; radius alone cannot
detect a corner-cut pass.

### A single-point DIRECT-TO is a final leg

`isApproachLeg()` is `total >= 1 and NAV.index >= total`. It used to be `>= 2`, and that
exclusion was the entirety of *"DIRECT TO does not slow down, flies past the point, turns back
and circles it"*:

* A direct-to was the one plan shape that never reached `APPROACH`. It went `ENROUTE` straight
  to `TERMINAL`, so it was flown by the UAC's **scalar closing-rate** law, which sees only the
  velocity component *toward* the target. Overshoot it and `closing` goes negative while ground
  speed stays high, so the error grows and the loop answers with **full forward thrust** to
  chase the fix it has just passed, while `desiredHeading` yaws the nose back onto it. That is
  an orbit. `APPROACH` exists to fly the **velocity vector** instead, which nulls the tangential
  component and therefore brakes an overshoot rather than pursuing it.
* It is also not a leg. `navPrev` in the UAC is whatever the previous target was, so a direct-to
  arrived carrying a bogus leg and ran a cross-track hold against it. The velocity-vector law
  ignores `hasLeg` entirely.

**`APPROACH` does not mean slow.** With `schedSpeed` set the UAC tracks `cruiseCap` exactly and
nothing in `dynamicSpeedTarget` reads the phase, so a long direct-to still cruises the whole way
and brakes on the arrival sweep.

---

## 6. Commanded speed — one source of truth

`dynamicSpeedTarget()` is the **only** place commanded along-track speed is decided, and the
UAC just tracks it as `cruiseCap`. That is why the HSI's CMD is exactly what the aircraft is
flown to. Order of application:

1. **The route-start ALIGN hold returns 0 first**, ahead of every cap and every early return.
   It used to sit three returns further down, past `not DC.DYNAMIC_CRUISE` — so on an airframe
   with the dynamic profile switched off the hold silently did not exist, and that is also the
   one configuration where the UAC has nothing to track it with.
   **An ATC hold comes next** (§8a): 0 for a hovering airframe, `CFG.ATC.ORBIT_SPEED` for an
   orbiting one — ATC's number, not the pilot's, so a COAST left selected cannot stall a
   FIXEDWING in the pattern.
2. Pilot cruise cap (`FULL` / `SLOW` / `COAST` — `CFG.SPEED_FULL`, `SPEED_SLOW`, 0).
3. Capped further to `LG.APPR_SPEED` while flying a published approach, or `LG.RWY_GA_SPEED`
   while flying a go-around circuit — and to `CFG.DEP.SPEED` through a departure's climb-out
   (§7b), the speed its turn off the runway was built for. (The take-off **roll** is not decided
   here at all: the departure owns the speed on the ground, as a landing owns the rollout.)
4. The **baked profile** at the aircraft's position — but only while `PLAN.onPath()`; joining a
   plan from elsewhere is flown at the plain cruise cap.
5. The **live destination arrival brake** on the final leg:
   `sqrt(FINAL_ARRIVE_SPEED² + 2·ARRIVE_DECEL·d)`.
6. The **cross-track gate**: beyond `XTK_SLOW_LO` (50 m) the speed is scaled down toward
   `XTK_SLOW_MIN` (0.30) at `XTK_SLOW_HI` (180 m), to fly in and intercept. When curve-following
   the deviation is measured from the **planned path**, not the straight leg — a craft correctly
   on a rounded corner is off the leg *by design*, which would falsely trip it.

Then `NAV.cmdSpeed` is **slew-rate limited both ways** (`DC.ACCEL_MAX` up, `DC.DECEL_MAX` down)
so it is a smooth curve, not a staircase.

### The ALIGN hold, and how it ends

`NAV.aligning` is set by every plan commit. It holds `cmdSpeed` at 0 while the nose swings onto
the first bearing, released when the heading error to the **carrot** (`NAV.aimX/aimZ`, not the
raw waypoint) is within `LG.ALIGN_HDG_DEG`.

Two additions that are each a fixed bug:

* **It is also dropped outright at `TERMINAL`/`ARRIVED`.** The UAC only steers to the bearing
  in ENROUTE/APPROACH; closer in it sets `desiredHeading = heading` and draws a position hold.
  A plan committed with its final point already inside `POI_FINAL_RADIUS` therefore starts in
  TERMINAL, the nose never swings, the heading test can never come true — and the hold latched
  for good. The aircraft still arrived (the UAC's own `APPROACH_KP · dist` law flew it in), so
  the only symptom was the HSI reading ALIGN / CMD 0 all the way in while the engines worked.
* **`NAV.cmdSpeed` is seeded from `min(cmdSpeed, groundSpeed)` on commit**, not left as-is.
  `cmdSpeed` is slew-limited, so a stale cruise cap needs `SPEED_FULL/DECEL_MAX` = 5 s to bleed
  off — five seconds of the hold during which the UAC is still handed hundreds of m/s and quite
  correctly accelerates on the old heading.

---

## 7. Landing

### `engageAutoland()` decides which machine

A site is flown as a **runway** if `LND.rwyUsable()`: `type == "rwy"`, a positive `rwyLen`, and
a resolvable direction (`x` = "any" is meaningless for a strip). Anything else gets the **pad**
descent, which works for any site.

### Pad descent

```
ALIGN → TRANSLATE → DESCEND → FLARE → PROBE → (LANDED | HOVER)
```

| Phase | Vertical | Exit |
|---|---|---|
| `ALIGN` | hold | heading within `ALIGN_HDG_DEG` (15°) |
| `TRANSLATE` | hold | position within `POS_TOL_M` (3 m) **and** heading within `FINE_HDG_DEG` (2°) |
| `DESCEND`/`FLARE` | `descentRateFor(agl)` — `DESCEND_FAST` 3.0 m/s easing linearly to `DESCEND_SLOW` 0.8 m/s at `FLARE_AGL` 10 m | AGL |
| `PROBE` | still `DESCEND_SLOW` | a touchdown test speaks, or `PAD_PROBE_M` below the pad |
| `HOVER` | `cmd = "hold", alt = siteAlt` | a squat contact |
| `LANDED` | `cmd = "landed"` + `squat` | — |

If the descent nod shoves the craft off the pad (`RECAPTURE_M` 4 m / `RECAPTURE_HDG_DEG` 8°) the
sink stops and the position hold re-captures before continuing.

**Why PROBE and HOVER exist:** *a helipad in `landing.db` is only a 3D point.* Nothing promises
there is anything solid under it — WCL sits at Y 226 with open air below and is meant to be
station-kept, not landed on. So reaching the pad altitude is **not a touchdown by itself**; the
descent presses on `PAD_PROBE_M` (2 m) below the site looking for a surface, which a real pad
simply blocks. Finding none, it climbs back and holds the published point rather than cutting
the engines in mid-air. Keep `TD_SETTLE_AGL >= PAD_PROBE_M`: the settle test is the only
touchdown test left to an airframe with no squat contact, and it is deaf outside its own AGL
window.

### The three touchdown tests (`LND.touched`)

1. **Squat switch** — believed at any AGL (already debounced and freshness-gated). That is the
   whole point of having the sensor.
2. **AGL vs the site altitude** — an **assertion**, not a measurement: "`landing.db` claims a
   surface at this height, so we must be on it". **Not** true of a helipad, and true of a runway
   only *where the runway is* — so `useAGL` is opt-in and the runway machine passes `LND.s >= 0`
   rather than a bare `true`. Short of the threshold the ground underneath is not the strip, and
   asserting a touchdown over it is how a landing rolls out into the undershoot.
3. **"Commanding descent but no longer sinking"** near the surface (`TD_SETTLE_*`). A real
   measurement, and the pad machine's fallback — the probe keeps a descent commanded below the
   pad altitude precisely so this test can still speak.

### Runway approach

```
INTERCEPT ⇄ GLIDE → FLARE → ROLLOUT → LANDED
                 ↘ MISSED  (or → go-around circuit)
```

All FMS-side, deliberately: the approach names a point, a speed and a vertical speed and
**never an attitude**, so all three flight modes fly the identical profile.

**Geometry** (`LND.rwyGlide`, `LND.rwySetup`): `landing.db`'s point is the **threshold**; the
strip runs `rwyLen` metres in the landing direction. Stop point at `rwyLen − RWY_STOP_MARGIN`.

The glidepath is **zero AGL at the aim point** (`gsDev` is measured to it; the wheels arrive
`float` metres further on), so the height the aircraft crosses the threshold at is
`aim · tan(gs)` — not a knob, a consequence of two others. With the published 4° and an aim
point of `min(rwyLen·RWY_AIM_FRAC, RWY_AIM_MAX)` that came out at **0.7 m (NUS/NUN) to 3.1 m
(TMV/TMA) on every runway in `landing.db`**: the aircraft crossed the threshold at knee height,
having begun its flare (`RWY_FLARE_AGL`, 5 m) **27–61 m before the threshold** and entitled to
assert touchdown (`TOUCHDOWN_AGL`, 1 m) before it too.

So **the crossing height is the input** (`RWY_TCH`), and the two that used to be inputs give way
to it in the order that costs least:

1. move the aim point out to where the **published** slope crosses at `RWY_TCH` — free, except
   that it is runway spent, hence the `RWY_AIM_MAX_FRAC` cap;
2. only for whatever is left, **steepen**. A 70 m strip cannot be approached at 4° with any sane
   crossing height — the aim point alone would be past the far end — and a steep approach is what
   you would fly onto it by hand. The approach-speed sizing pays for it automatically: its first
   cap is `RWY_VS_MAX·RWY_GS_MARGIN/tan(gs)`, so a steeper strip is flown slower.

The crossing height is never effective below `RWY_FLARE_AGL` whatever `RWY_TCH` says, because a
flare beginning before the threshold is the fault this exists to fix. Today: TMV/TMA 4.0°,
TMN 4.9°, WAX 5.9°, AGC 6.5°, NUS/NUN 13.8°, and the flare starts 4–14 m **inside** every strip.

`LND.gsTanFor` is the single reader for the go-around circuit's descent and a published
approach's descent, so neither can deliver the aircraft onto a **shallower** path than the
approach flies — which is the one side `RWY_GS_MARGIN` cannot recover from.

> **This is what "the runway landing flares and rolls out before the runway" was**, and the
> `ALT_OFFSET` datum fix (2026-08-29) is why it turned up when it did: before it the altimetry
> over-read by a hull height, so the aircraft flew a hull height above this geometry and the
> shortfall was masked. The masking changed; the geometry was always this.

**The localizer** is a real one, not pure pursuit. Pure-pursuing a point on the centreline
commands an intercept angle of only `atan(xtk / lookahead)` — 3.6° for a 25 m offset at a 400 m
lookahead — far too soft to centre: the aircraft settles onto a track *parallel* to the
centreline and lands wide (obvious on a VTOL, which crabs sideways instead of turning). So:

```
lat = −(RWY_LOC_KP · xtk + RWY_LOC_KD · xtkRate),  capped at appSpeed·sin(RWY_LOC_MAX_DEG)
```

expressed as a carrot placed in the **commanded direction** — because direction-to-target is
precisely what the UAC's velocity-vector law flies, so this drives all three flight modes
without either end knowing which is active.

**The glideslope:** `vs = −(groundSpeed · tan(GS)) − RWY_GS_KP · gsDev`, clamped to
`[−RWY_VS_MAX, RWY_VS_UP_MAX]` (0 inside the flare). Arriving **high** is the normal case — the
approach joins at whatever altitude ALT-AP was holding — so it saturates at `RWY_VS_MAX` until
the path is caught. It can catch it at all only because `RWY_GS_MARGIN` (0.75) kept the *nominal*
sink below that limit. **Do not raise `RWY_GS_MARGIN` to 1**: the steepest possible descent would
then be exactly the glideslope, so a high join could only fly parallel to the path forever.

**Approach speed is a fixed point solved by bisection.** Three caps: what was asked
(`RWY_APP_SPEED`); what the descent authority allows (`RWY_VS_MAX · RWY_GS_MARGIN / tan(GS)`,
with the **per-runway** GS above); and what the rollout can stop from in the runway genuinely
left. "Genuinely left" must budget the **flare float**, which is the biggest consumer of a short
strip:

```
float = 2·RWY_FLARE_AGL·V / (V·tan(GS) + RWY_TD_VS)
V     = min(cap, sqrt(2·BRAKE_DECEL·ROLLOUT_FRAC·(stop − aim − float(V))))
```

The float depends on the very speed it limits, hence a fixed point. It is solved by **bisection
because the old "iterate four times and keep the smaller" loop latched**: the right-hand side
*falls* as V rises, so the first pass — taken at the cap — charges the float of a speed the
aircraft was never going to fly, and `v = min(v, …)` made that pessimistic pass permanent.
Measured at WAX (166 m of runway): pass 1 charged the 91 m float of the 37.5 m/s cap and dropped
V to 10.6 m/s; pass 2 found 18.9 m/s would fit and was forbidden from saying so. Flown, that is
the 11 m/s crawl down the WAX centreline. **The true root is 16.5 m/s.**

**`RWY_ARM_M` (3600 m) is where the approach takes over**, and must exceed `RWY_FINAL_M` (3000 m)
so passing the planned FINAL fix arms it. ARRIVED would be far too late — for a strip that is
already the far side of the touchdown zone.

**The braking term is `2·a·s`.** Both the speed sizing and the missed-approach test wrote
`a·a·s` (and `v²/a²`) until 2026-09-08, which is not a stopping distance at all: at
`RWY_BRAKE_DECEL = 20` it claimed the aircraft could stop in a **tenth** of the room it needs, so
the rollout cap almost never bound and a short strip was flown at the full descent-limited speed.
The autobrake at the bottom of `LND.rwyTick` always used `v²/2s`, and so did the knob's own
comment. **The consequence of fixing it is that `RWY_BRAKE_DECEL` bites for the first time** —
if a landing now crawls down the centreline, that is the knob.

**The missed-approach test is "there is no longer enough runway left to stop in from the speed
we are doing"**, not "we drifted past the aiming point". The old test fired `RWY_STOP_MARGIN`
past the aim point, which on a 295 m strip meant giving up with 200 m of perfectly good runway
ahead — and since a normal flare floats most of the way to that line, the same approach would
land or go around more or less at random. It is only armed once past the threshold (`LND.s > 0`),
because before it the whole strip is still ahead and the aircraft may legitimately still be
shedding cruise speed.

**The rollout brake starts from the speed the wheels actually arrived at, on the touchdown
tick.** Both halves matter: seeding from measured ground speed covers the aircraft that floated
and arrived slow or came in hot, so the first rollout command equals what the craft is already
doing and the translation law sees zero velocity error. The **autobrake** is
`max(RWY_BRAKE_DECEL, v²/2s)` — a continuous function of a continuously-varying distance, so it
firms up without a step, never binds on a nominal touchdown, and only takes over when the
landing floated long.

`ROLLOUT` sends `navPhase = "LANDING"` with `carrot = true` and `glide = false`. See
[01-protocols.md](01-protocols.md) §navap for why those two must be sent separately.

### Go-around

Two published shapes, both in `landing.db`, and the **waypoint circuit always wins** where a
runway publishes both (a *drawn* procedure beats a *deduced* one). Selection order in
`LND.gaFor`:

1. the circuit the **approach itself named** (`NAV.apprGa`) — an approach chart printing its own
   missed-approach procedure. Survives a go-around on purpose, so a second miss flies the same
   circuit.
2. `goaround-wp` matched **by name** (a circuit starts *and* ends at the runway it serves; one
   written the other way round is flown backwards rather than refused, which is also how the two
   ends of one strip pick out their own circuit).
3. `goaround` (straight leg) matched **geometrically** — the leg running the same way as the
   landing direction (`RWY_GA_ALIGN` ≥ 0.5, i.e. within 60°), ties to the nearest start.

**A go-around is not a new state machine; it is a new PLAN.** `LND.active` goes false, the route
machinery flies the horizontal, and all `LND.ga` means is that the FMS keeps the vertical
channel — because handing it to ALT-AP would climb to whatever the pilot last selected.

For the **waypoint** shape (`GA.buildWp`) that is eight lines: drop the leading entry (the
threshold we are sitting over — flying *to* it would turn the circuit inside out), replace the
trailing one with the snapshotted landing fix, run `LND.rwyPrep`, `PLAN.build`. Done.

For the **straight-leg** shape (`GA.build`) the FMS has to invent the circuit, and this is where
the `GA` module earns its keep. The published record is an **oriented line segment** that must be
entered at its start, on its heading, and flown end to end — and it may point anywhere relative
to where the miss left us. Handed to the fly-by planner (which *cuts* corners) it produces
nonsense. So:

* **Dubins** (turn–straight–turn) joins from where we are, pointing where we point, to the leg's
  start pointing along the leg. One construction covers every case — already aligned
  (degenerates to a straight), a right-angle join, a full course reversal (comes out as the
  teardrop you would fly by hand) — with no special cases.
* The turn radius comes from `RWY_GA_SPEED`, which is why the circuit is flown at approach speed
  and not at cruise: *you cannot spiral a 320 m/s aeroplane onto a 200 m leg.*
* **How far to run straight first is searched, not guessed** (`GA.bestRun`): try 0, 50, 100 … m
  and keep whichever gives the shortest *total* path. Turning onto a leg that runs the same way
  a few hundred metres to one side is the hard case — from close abeam the only curvature-limited
  answer is a full 360°; run on a little and the same join becomes a gentle S. This is why
  `RWY_GA_STRAIGHT_M` must stay **short** (150 m): it is a floor on the search, and setting it
  long throws away the room the join needs.
* The rejoin point is at the leg's end if that finishes on the approach side of the threshold,
  else at `RWY_FINAL_MIN`; and the circuit **descends to the glidepath height at that point** —
  holding circuit altitude to the rejoin is what left the first version permanently high.

**The vertical during a circuit** (`LND.gaTick`):

* waypoint shape: hold the published minimum until the glidepath — extended back along
  `LND.gaRemaining()` — sinks below it, then **ride the glidepath in**, with a feed-forward of
  `−groundSpeed·tan(GS)`. Without the feed-forward a proportional law tracks a ramp with a
  standing error of `rate/KP` — nearly 30 m high, all the way down.
* straight-leg shape: circuit altitude around the pattern, then `gaExitAlt` past the leg's end.

`gaRemaining()` deliberately walks the waypoint list rather than using `PLAN.lastS`, because
`lastS` only means anything while on the path — and the vertical channel needs an answer during
the join at the start, which is exactly when we are not.

`RWY_GA_MAX` (2) bounds the attempts per landing, reset only when the pilot commits a new plan.
Out of attempts, or nothing flyable: level off and hold on the runway heading — **no blind
climb-away**, because without a route there is no altitude the FMS can be sure is clear.

### Published approaches (`APR`)

`landing.db`'s fourth record shape. A field publishes one arrival per direction it can be
arrived from, and the FMS picks **one** when the plan is prepared. Everything after the choice
is an ordinary route — same corners, same speed profile, same straight-in final. There is
deliberately no approach state machine.

What is *not* ordinary is the vertical: these procedures put the last fix a few hundred metres
from the threshold, and cruise height cannot be flown off in that distance. Entering the pattern
(`APR.arm`) hands the vertical to the FMS and flies **exactly `LND.gaTick`'s waypoint branch**.

`APR.prep(pts)` — three cases, **and the order is the fix for the WAX bug**:

1. **Already flying one** (`APR.established`). The plan's tail is a tail of a published pattern
   *and* it arrives **along** the pattern rather than merely at a fix of it. Courses are built out
   of the same fixes procedures are, so this is ordinary rather than exceptional. Nothing is
   spliced, but the pattern is **recorded** — the published minimum and the missed approach are
   properties of the procedure, not of whoever assembled the point list.
2. **Already straight in** (`APPR_STRAIGHT_IN_DEG`, 20°). Leave it alone: a plan lined up on the
   runway must not be turned away from it. Case 1 running first is what makes this safe, via
   `APR.onPattern`.
3. **Otherwise choose one** (`APR.choose`), trim whatever tail of it the plan had blundered onto,
   peel back any en-route fix left standing behind the join, and splice.

`APR.cost` is "most suitable" made concrete: **metres of track** — the run-in to the join fix,
the rest of the pattern, and `APPR_TURN_WEIGHT · R · (t1 + t2)` for the two turns the join
demands. That turn charge is what makes a pattern *behind* the aircraft lose to a longer one
ahead of it, i.e. what makes this a choice by position **and** heading.

It is costed from where the **plan arrives**, not where the aircraft is parked (`APR.arrival`) —
choosing off the parked position would pick the wrong side of the aerodrome on every plan that
turns on the way. A direct-to is the one case where the two coincide.

**`APPR_JOIN_MAX_DEG` (90°) is the whole of the WAX fix.** Measured around WAX: clean joins are
0–13°, bad ones 99–156°. The auto-router's route to WAX ends `…, DSWAE, KOBUL, WAUZA, WAX` —
WAX_APPR1's last two fixes in the right order — which the old rule read as "established, nothing
to splice". It is not: the leg in from DSWAE meets the procedure 71° across its own track and
the aircraft then had to reverse 156° at KOBUL. And an *angle alone* was the second half of the
bug: WAX_APPR1's last leg is 514 m straight up the centreline, so that same route arrived
"lined up" and the whole procedure was skipped — minimum, missed approach and all. The honest
distinction is not how long the aligned run is, it is whether the plan is **sitting on** a
procedure it has already been found not to be flying.

**`APR.inTrim` keeps a mid-procedure join honest:** a join anywhere but the published entry may
only be made at a fix the plan was *already routed through*. Without it the cheapest join is
always the pattern's **last** fix and every direct-to truncates the procedure to its final leg —
measured, DIRECT TMV from 12 km west collapsed a five-fix procedure to a bare TAIMU.

**`APPR_ARM_M` (12 000 m) is distance still to fly, not "reached the first fix"** — because a
direct-to's plan is *nothing but* the pattern, and without it such a landing would be flown at
the published minimum, at approach speed, from the far side of the world.

---

## 7b. Runway departure (`DEP`, 2026-10-15)

A plan committed while the aircraft **stands on a runway** is flown off it, **whatever the flight
mode**: nose onto the runway heading, onto the centreline, a take-off roll, rotation, a climb
straight out past the far end, and only then a turn for the route. Before this a VTOL simply rose
off the strip under ALT-AP and set off for the first fix from wherever it stood — on the way to a
fix beside or behind the runway, that meant turning across it. Configured in `apconf.lua`'s
`CFG.DEP`; **every number there is unflown** (see `CLAUDE.TODO.md`).

### When it engages

Only at a **plan commit** (`setDirectTo`, `setRoute`, `RTR.setSolvedRoute` → `DEP.commit`), and
only if `DEP.plan` finds the aircraft:

* at a near-standstill (`MAX_GS`, 3 m/s — a low pass is not a take-off),
* within `GROUND_AGL` of a published `rwy` record's surveyed elevation, inside its strip
  (`STRIP_HALFW` either side of the centreline, `STRIP_MARGIN` past either end),
* and not reported **airborne** by a live squat switch (a hover a metre up is not a take-off).

Every published direction of every strip the aircraft stands on is a candidate (`DEP.candidate`),
and the cheapest wins. **Departures are flown only in a direction `landing.db` publishes**: a
record is the statement that the strip is used that way, and a strip used both ways publishes
both ends (TMA/TMV, NUS/NUN). WAX publishes only 056, so WAX is always left on 056.

Everything else — airborne, on a pad, a DIRECT to the field it stands on (nothing left to depart
*to* once the aerodrome's own fixes are dropped) — commits exactly as before.

### The ground run (owns the aircraft, like `LND.active`)

```
READY ──(AP-NAV on)──▶ ALIGN ──▶ LINEUP ──▶ ROLL ──(VR, or rotS)──▶ CLIMB ──(CLIMB_AGL)──▶ OFF
```

| Phase | `navap` | vertical | exit |
|---|---|---|---|
| `READY` | `{cancel}` | **nothing** | AP-NAV on |
| `ALIGN` | LANDING, `holdHeading` = runway, position hold where it stands | `landed` | heading within `LG.ALIGN_HDG_DEG` |
| `LINEUP` | LANDING, `holdHeading`, position hold on the **line-up point** | `landed` | heading within `FINE_HDG_DEG`, along-track within `POS_TOL_M`, and \|xtk\| ≤ `XTK_TOL` **or** `CENTRE_T` elapsed |
| `ROLL` | LANDING, `holdHeading`, **carrot** on the centreline, `cruiseCap` rising at `ROLL_ACCEL` | `landed` | ground speed ≥ `VR`, or past `rotS` |
| `CLIMB` | the route's own carrot, `glide` | `descend`, `vs = +CLIMB_VS` | `CLIMB_AGL` above the strip → `cancel` |

**It speaks the rollout's vocabulary run backwards, so the UAC needed nothing new.** LANDING +
`holdHeading` + carrot + `schedSpeed` is exactly what the rollout sends; only the commanded speed
rises instead of falls. A VTOL/HYBRID therefore holds the runway heading and **strafes** onto the
centreline (HYBRID's pad-phase policy), and a FIXEDWING — no strafe — holds the heading and rolls
straight. The FMS never learns which it is talking to. `type = "dep"`, deliberately **not**
`rwy`/`land`: the GPWS reads those two as "on final" (07 §`onFinal`).

* **The line-up point.** Where the aircraft stands, if there is room ahead to reach `VR` short of
  the last `LG.RWY_STOP_MARGIN`; otherwise **backtrack** to the threshold and use the whole strip.
  An aircraft that landed here stands exactly at that margin (the rollout's stop point), so leaving
  the way it landed always backtracks — 136 m at WAX, flown in reverse on the runway heading by the
  same position hold. On a two-ended strip the other end usually wins instead: the cost is
  `BACKTRACK_WEIGHT × backtrack + runway to run + climb-out + route from the join`.
* **A FIXEDWING cannot centre.** It waits out `CENTRE_T` (3 s on Citation) once otherwise lined up
  and rolls parallel to the centreline, which is what it would do by hand.
* **The roll's steering is the approach's localizer** (`LG.RWY_LOC_*`, §7) — it has little to do
  after LINEUP, and nothing on a FIXEDWING.
* **Rotation** at `VR`, or at `rotS` (`RWY_STOP_MARGIN` short of the far end) whatever the speed.
  A FIXEDWING commanded to climb below its speed simply does not (the UAC's speed protection,
  03 §10); `VR` must be reachable in the strip for it to leave the ground on the runway.

### The climb-out (a ROUTE, like a go-around circuit)

`DEP.install` bakes the whole departure into the plan's own samples ahead of the route: the strip
from the line-up point, `STRAIGHT_M` past its far end (searched up to `RUNOUT_M` further, as
`GA.bestRun` does), then a turn at the radius `CFG.DEP.SPEED` flies, then the ordinary `PLAN.build`
output from the first fix beyond the aerodrome. One speed profile is re-baked over the lot, from a
standstill. From rotation the route machinery flies it with nothing special; all `CLIMB` means is
that the FMS still holds the vertical — `LND.ga`'s arrangement, for its reason: ALT-AP's selection
is not a height anybody has checked clears the end of this runway.

**Why the turn is built here and not left to the fly-by planner.** The planner rounds a corner by
cutting it, and the corner would be the end of the runway: a fix beside or behind the strip would
start the turn over the runway, before the wheels were off it. Two shapes, the first preferred:

1. **Turn towards the fix and fly at it** (`DEP.cs`, a turn-then-straight), with the end of the
   turn a fix of its own (`TURN`) and the **first point of the plan handed to `PLAN.build`** — so
   the planner rounds the first fix at its own speed like any other. A first fix kilometres out
   must not be met at the departure's slow radius: that would brake a 320 m/s cruise to 40 m/s for
   a turn with no business being tight.
2. **Arrive on the outbound leg** (`GA.join`, turn-straight-turn) when the planner could not round
   the fix — a reversal, or a corner whose tangent at `MIN_RADIUS` will not fit
   `LEG_FIT_FRACTION` of the legs. **WAX-TMA is this case**: WAU lies ~360 m behind-left of the
   climb-out and the route turns 136° there onto a 157 m leg. At Aer Togekiss's 145 m radius the
   shortest flyable join is a ~240° right-hand swing that rolls out heading north-west onto WAU and
   turns north there; a lower `SPEED` tightens it.

The fixes at the head of the plan that **are** the aerodrome (WAX-TMA starts at WAX's own
threshold) are dropped, not flown to. With RESUME semantics (FLY ROUTE → RESUME, and every
auto-routed plan) the join is chosen by `bestResumeIndex` **from the climb-out's far end on the
runway heading**, and only among fixes off the strip — choosing first and then stepping past an
on-strip fix landed on whatever followed it in the list (TMV → TAIMW, 733 m behind).

### What ends it

| Event | Effect |
|---|---|
| `CLIMB_AGL` reached | `cancel` — ALT-AP gets the vertical back and the selector **restores its pick** |
| AP-NAV switched off (any phase from ALIGN) | `DEP.abandon("cancel")`: vertical back, plan flown as an ordinary one under the ALIGN hold |
| **off the ground** during the ground run (\|AGL\| > `GROUND_AGL`) | the same. Every ground tick would otherwise send `landed`, which **cuts the lift** — and READY owns nothing, so an engaged ALT-AP can lift a VTOL while AP-NAV is still off, and a scramble overrides the autoland channel outright |
| pilot taps ALT-AP | `release` (the tap's pick stands); on the runway the run is given up too |
| a published approach arms / a landing engages | ends silently — their own vertical command goes out the same tick |
| ABORT / any new commit | `cancelNav` / `DEP.reset` |

ATC answers are **not acted on during the ground run** (`ATC.committed`): an answer about the
destination is no reason to reject a take-off, and a divert would re-plan with the aircraft rolling.

**The pilot's workflow.** Commit the plan straight from the LANDED screen (HOME → FLY ROUTE), or
after AP OFF. **RESUME ALT-AP first lifts the aircraft** under ALT-AP, and a departure needs it on
the ground — so that plan is flown the old way, up and away.

### The commit's hand-back, which was a bug before it

`DEP.reset` (head of every commit) and `DEP.commit` (tail) also close a hole every commit had: they
tore `LND` down **without a word to the UAC**, which latches `landed`/`descend`/`hold` until told
otherwise. A plan committed from the LANDED screen was flown along the ground with the lift cut,
and one committed mid-approach went on ignoring ALT-AP for good. Now a commit that supersedes a
landing, a published pattern or a departure sends `cancel` — unless a new departure takes the
channel straight over, which a cancel would only release for a tick. `ATC.divert` had found this
for itself; it now relies on the commit's.

---

## 8. Terrain avoidance (`TAVOID`)

An **automatic altitude floor**, and the one place in the FMS allowed to read the terrain
database and *act*. Configured in `apconf.lua`'s `CFG.TAVOID`; per-aircraft `ENABLE` is the
flight-test gate (true on Aer Togekiss and Scout, false elsewhere today).

**Mechanism:** it takes the vertical channel exactly as an autoland does, on the same
`AUTOLAND_PROTOCOL`, with the same `{cmd="hold", alt=…}` the pad HOVER phase uses.

**Asks for:** a straight corridor along the current ground track (or the nose, below `MIN_GS`),
cut at the FMS's own next waypoint plus `LEG_MARGIN` — a corridor is a straight line and a route
rarely is, so ground the plan turns away from before reaching is not this aircraft's problem.

**Required altitude:** the highest `(charted top + CLEAR_M)` in the corridor window — terrain or
an obstacle heavy enough to clear `MIN_MASS`. Floor *and* ceiling are climbed over, never
threaded underneath (deliberately the simple, conservative half of a real TAWS escape).

**The engage / ratchet / release rule:**

| Situation | Action |
|---|---|
| below required, not engaged | engage, hold the required altitude |
| engaged, a taller hazard slides into the window | **ratchet the hold UP** |
| engaged, requirement drops | the hold **never sinks** |
| engaged, actually above `required + RELEASE_MARGIN`, fresh + on-track + charted | release (`cmd = "cancel"`) |
| any "don't know" | **change nothing** — see invariant I8 |

*Nothing here ever descends the aircraft.* Only ALT-AP or the pilot do that, same as before it
existed.

**Gates:** AP-NAV must be on — read **directly off this computer's redstone input**, ahead of
every other check, so a stale corridor at the moment the switch drops cannot leave a hold
running past it. It never runs during a landing or a go-around. A pilot ALT-AP tap releases it
and stands it down for `OVERRIDE_COOLDOWN` (30 s) — *an override that gets fought at once is not
an override* — while the MANCON scramble and the GPWS stay armed underneath.

**The disengage verb is `cancel`, not `release`.** See [01-protocols.md](01-protocols.md) §autoland.

---

## 8a. The automated ATC client (`ATC`)

The aircraft's half of the controller on #87 ([08](08-ground-stations.md) §7; the wire is
[01](01-protocols.md) §8a). Configured in `apconf.lua`'s `CFG.ATC`; `ENABLE` is the per-airframe
gate (true on the four VTOL hulls, false on Citation and the bench — see
[10](10-aircraft.md) §1).

**When it asks.** `ATC.wanted()` names the facility: `LND.name` while a landing is engaged
(until `LANDED`), else the plan's `NAV.landSite`. Only with AP-NAV on — read off this
computer's own redstone input, as `TAVOID` does. Contact starts once the facility is within
`CONTACT_M + v²/(2·DC.DECEL_MAX)` — the stop term is what gives a hold room to be flown short of
the field from 320 m/s (Aer Togekiss) or 800 m/s (Scout) — and then lasts as long as that
facility does.

**What each answer does** (`ATC.tick`, first thing in `navTick` after the pose):

| answer | effect |
|---|---|
| `clr` | nothing; ends a hold |
| `hold` | `ATC.holding`: `dynamicSpeedTarget` returns 0 (or the orbit speed), `broadcastCurrent` sets `align` (hover — the UAC brakes the **whole** velocity vector, the ALIGN hold's mechanism) or flies an orbit carrot, and **navTick may not engage the landing** |
| `divert` | `ATC.divert(to)`: `cancel` the vertical if a landing or a published pattern holds it, clear `LND.appr` (or the new plan's approach never arms — `APR.due`), `setDirectTo(to, true)`, restore the pilot's cruise mode, name the plan `ATC <to>` |
| `none` | nothing; the facility is uncontrolled |

An answer is acted on only if it is about the facility currently asked for (`reply.fac`), was
heard within `STALE`, and the landing is not **committed** (`FLARE`/`PROBE`/`HOVER`/`ROLLOUT`/
`LANDED`). A hold or divert that arrives earlier in a landing **aborts** it
(`ATC.abortLanding`): `cancel` on the autoland channel, `LND` down, `NAV` left alone — so the
route machinery flies again from the next tick and the landing re-engages when the hold ends.

**Fail silent, both ways (I8).** A controller that has never answered changes nothing; one that
goes quiet undoes nothing — a hold in force stays in force, because "the controller stopped
talking" is not "the pad is free". The pilot's way out is **RELEASE ATC**, which replaces the
SPD button on the status screen during a hold and flies the rest of the plan unmanaged; any new
plan (every commit calls `ATC.reset()`), `cancelNav`, or AP-NAV off ends it too.

**The orbit** (`HOLD = "orbit"`, FIXEDWING): centre `ORBIT_R` to the right of the track,
clockwise from above. The carrot rides a circle of `R / cos(lead)`, not `R`: pure pursuit of a
point leading by `lead` settles at the carrot's radius × `cos(lead)`, which on `R` itself flew a
300 m orbit at 237 m in a kinematic check. Sent as `ENROUTE` whatever the plan's phase, because
TERMINAL/ARRIVED would draw a position hold. **Unflown**: Citation, the one airframe that needs
it, has `ENABLE = false`.

**Displays.** Row 2 of the status screen (and of the landing screen, where it was free) carries
`ATC.label()`; the HSI shows `HOLD` in the CMD slot. A hold's reason and queue place come from
the controller's `why`/`q`.

---

## 9. The auto-router (`RTR`)

"FLY ROUTE → TO DESTINATION": the pilot names a destination hub and the FMS solves a plan from
the current position, stitching published courses through the waypoints they share. There is no
direct KST–TMU course, but KST-WAU and WAU-TMU exist and meet at WAU.

* **Hubs** = the first and last point of every course. Only these are offered as destinations
  (all are landing sites).
* **Graph** = nodes are points used by some course; edges are consecutive waypoint pairs weighted
  by leg distance. **Undirected** — a leg can be flown either way, so a waypoint two courses share
  becomes an interchange node and a path can cut between courses *wherever they meet*, not only
  at hub aerodromes.
* **Join** = the nearest graph node to the current fix; the first leg is a direct hop to it.
* **Search** = Dijkstra with a linear-scan settle (a few dozen nodes; no heap needed), **rooted
  at the destination** and settling the whole graph — `RTR.costsTo` — with `RTR.pathFrom`
  walking `prev` forwards from the join. One search prices every node.
* **Collapse** — not part of `RTR` at all, and that is the point: see §4a. The solved path is
  handed on as it is.

> **Why the join is still "nearest"**, having tried the obvious alternative: choosing it by
> total track (hop out + network cost onward) **degenerates**, because the destination is a node
> whose own cost is zero and the triangle inequality then makes "fly straight there" the winner
> every time — measured, TMV → FGY came back as one 19 145 m direct leg instead of the 15-fix
> airway route. Charging off-airway track more than on-airway would fix it, but only by
> inventing a weight nobody has flown.

`RTR.solvePath` is gone; `RTR.costsTo` + `RTR.pathFrom` + `RTR.pathLength` replace it.
* **Commit** re-solves from the live fix, so drift between preview and confirm cannot leave a
  stale plan. It joins mid-route via `bestResumeIndex` and auto-lands at the hub.

`bestResumeIndex` is heading-aware: a waypoint *behind* the aircraft has its straight-line range
inflated by `RESUME_BACKTRACK_WEIGHT` × its backward component. At 1.0, a waypoint dead astern
must be nearer than **half** the distance of one dead ahead to still win. Evaluated over all
waypoints, so a third point ahead can win too.

---

## 10. The UI

Touch monitor if one is attached (the whole display redirects there), else the computer's own
terminal and `mouse_click`. **The HSI monitor's touches are explicitly not read as UI input** —
`hsiMonName` is excluded in `pointerXY`. They are not discarded, though: `uiLoop` reads them, one
branch earlier, as the **HSI's view toggle** (§11).

### The repaint gate, and the preview's cached solve (2026-09-17)

Every draw function is a `term.clear()` and a full repaint — there is no `window` buffer
anywhere in the program — and `uiLoop` used to do one **per event**. `parallel.waitForAny` hands
every event to every branch, so that meant `navLoop`'s 20 Hz sleep, `hsiLoop`'s 20 Hz sleep,
`apAnnounceLoop`'s 10 Hz, four `rednet.receive` timeouts **and every rednet packet this modem
hears from anywhere on the fleet**: ~58 repaints a second before a single packet arrives, and
well past 100 with the fleet talking. That is the flicker.

Two changes, and they are one idea — stop doing the work:

* **`uiLoop` repaints on a gate.** `dirty` or `UI_REDRAW_MIN` (0.1 s) elapsed, whichever comes
  first. `dirty` is set by a touch that hits a hitbox (it may change `screen` outright, and
  `alert`/`confirm`/`promptKeyboard` draw over the screen and return) and by a resize, so input
  never waits on the gate — a button that looks dead for a tenth of a second gets pressed twice.
  Nothing needs a timer of its own: `navLoop`'s sleep guarantees 20 events a second whatever
  else is quiet, so an expired gate is serviced within a tick. **The hitboxes deliberately
  survive a skipped repaint** — they are cleared only when the screen is actually redrawn, so
  what is hit-tested always belongs to what is on the glass.
* **The ROUTE PREVIEW's solve is cached** (`RTR.solveCached`, `RTR.SOLVE_TTL` = 0.5 s). It was
  rebuilding the route graph — expanding ~90 courses — and running a Dijkstra *per event*, for a
  screen whose numbers move at walking pace. It still re-solves as the pilot watches, because
  the join range is measured from the live fix; half a second of staleness is invisible on a
  range that changes by metres. **The commit does not use the cache**: `RTR.setSolvedRoute`
  calls `RTR.solveRoute` directly and always re-solves from the live fix, because a plan
  committed from a cached solve is exactly the preview/confirm drift that path exists to rule
  out (§9).

If 10 Hz of clear-and-repaint still shows on a big monitor, the remaining fix is double
buffering — `window.create`, draw with `setVisible(false)`, flip with `setVisible(true)` — which
removes the artefact rather than reducing its rate. Not done; nothing depends on it.

```
HOME ─┬─ DIRECT TO ──▶ picker (nearest / A-Z / Z-A, distances refreshed 1 Hz)
      ├─ FLY ROUTE ──┬─ by course ──▶ START or RESUME (live join preview)
      │              └─ TO DESTINATION ──▶ hub picker ──▶ plan preview ──▶ FLY
      ├─ LAND ──────▶ landing-site picker (auto-lands on arrival)
      ├─ EDIT DATA ─┬─ points list ──▶ point detail
      │             └─ courses list ─▶ course detail ─▶ waypoint picker
      └─ STATUS ────▶ waypoint list + PASSED/remaining, SPD selector,
                      or the LANDING HUD (gate numbers + ABORT / RESUME LND /
                      RESUME ALT-AP / AP OFF)
```

* **SPD** cycles FULL → SLOW → COAST and is tappable whenever a plan is being flown. It pushes
  the new cap immediately rather than waiting for the ping. It used to demand `ENROUTE` on the
  grounds that the cap only bites there — which is not so (`dynamicSpeedTarget` applies
  `cruiseCapValue()` as `capBase` every tick and reads the phase nowhere; see §5, *`APPROACH`
  does not mean slow*). What that gate really did was take the speed selector away from the two
  plan shapes that never **reach** `ENROUTE`: a **direct-to** is a single-point plan, so
  `isApproachLeg()` (`NAV.index >= total`) is true from the moment it is committed, and so is a
  **direct-to-land at a helipad**. Both cruised the whole way at `SPEED_FULL` with SPD greyed.
  A landing owns the speed itself, but that cannot reach here — `drawStatus` hands over to the
  landing HUD while `LND.active`, and that screen has no SPD button.
* **ABORT** and **RESUME ALT-AP** are the *same* teardown (`cancelNav()`): stop the navigation and
  hand the vertical back to ALT-AP — so an abort climbs away as a go-around and RESUME lets the
  craft leave the pad on the cruise altitude. **AP OFF** is `cancelNav("off")`: same teardown, but
  the selector stays deselected and the UAC just holds the ground altitude.
* Nav-data updates are held while on any screen that lists or edits data; every such screen paints
  a corner `UPD HELD` marker, and HOME/EDIT DATA carry the full status line.
* **A departure** (§7b) takes row 2 of the status screen while it runs — `DEP WAX 056 LINE UP 34m
  XTK +1.2`, `ROLL 23/30 m/s`, `CLIMB 12/30 m` — ahead of the ATC line, which only ever talks about
  the far end; and its phase replaces `NAV.phase` top right. Its synthetic fixes `CLIMB` (where the
  turn off the runway starts) and `TURN` (where it ends) are ordinary entries in the waypoint list.

---

## 11. The HSI (external monitor)

**Three views, and two different kinds of decision.** COURSE and PLAN are the *pilot's* choice,
toggled by **touching the HSI monitor**; LANDING is the FMS **overriding** that choice while
`LND.active`, and the touch is then ignored outright rather than deferred — so the stored view
cannot change underneath the landing map either. Same shape as the altitude selector's
NORMAL/OVER machine (06 §2).

* **COURSE** (`drawCourseHSI`) — heading-up compass rose, course-deviation needle for the active
  leg, TO/FROM tips (grey inside `TOFROM_DEAD` — the classic VOR ambiguity zone), over the
  terrain overlay. The top-left CMD slot reads `ALIGN` during the route-start hold, `LINE UP`
  while a departure is short of its take-off roll (COURSE and PLAN alike), `HOLD` in an ATC hold.
* **PLAN** (`drawPlanHSI`, the `CRS` table) — the route map. No terrain. See below.
* **LANDING** (`drawLandingHSI`) — heading-up moving map, aircraft fixed nose-up at centre, target
  rotating under it, heading-error bar on top, numbers on the bottom; plus a glideslope pointer up
  the right edge for a runway approach.

The touch is handled in **`uiLoop`**, not in `hsiLoop`: `hsiLoop` sleeps, and `uiLoop` is the only
loop that pulls events. `pointerXY` still rejects HSI-monitor touches as UI input, exactly as it
did — the toggle is a separate branch ahead of it.

Map range snaps to the smallest of `CFG.HSI.RANGES` that still frames the target, so the target
creeps inward as you close and the scale only jumps occasionally.

### The PLAN view (`CRS`)

Heading-up and aircraft-centred like the other two — a pilot switching views should not have to
re-orient, and this instrument is never the chart; EDIT DATA is.

* **What is drawn, bottom to top:** a **terrain hazard layer**, the **baked path**
  (`PLAN.samples` — the clothoid-arc-clothoid geometry the aircraft actually flies, not the
  straight legs), the waypoints on it, **traffic**, a range ring with a rotating N/E/S/W card,
  and a **one-row course-deviation gauge** along the bottom row above the numbers. The gauge
  uses `CFG.HSI.CDI_FULLSCALE_M` / `CDI_ONCOURSE_M` — *the rose's own scale*, so there is one
  full-scale value and not two, and the same "the needle marks where the course is" convention.

#### The terrain layer — hazards and obstacles, not a landscape

Only four of `TERR.band`'s nine bands: **1 WARN**, **2 CAUTION**, **8** a mass within
`CAUTION_M` overhead and **9** one further off. The heightmap tint and water are dropped. The
rose says what the land **is**; this view says what is **in the way**, and black here means
"nothing to hit" rather than "not surveyed" — which is why `TERR.label` is on this view's second
row too, and why it matters more here than on the rose.

* **Sampled per character cell, painted as the cell's BACKGROUND**, with the path drawn over it
  in subpixels. That is not a saving, it is what makes a cell's two colours reach: a hazard is a
  blob and loses nothing at cell resolution, a path is a line and needs the subpixels.
* **The gradient is dropped to one coarse step.** A CAUTION cell further from the warning band
  than `THIN_LVL`, and a mass further overhead than `THIN_LVL`, are drawn as a **checkerboard**
  instead of solid. Without it every cell under an island is a solid magenta wash with the route
  lost inside it. `lvl` counts up toward the hotter neighbour for CAUTION and up toward *black*
  for an overhead mass, so the two comparisons face opposite ways.
* **An island BELOW the aircraft is not drawn** — it is not a hazard while you are above it, and
  it becomes one the moment a descent brings it inside `WARN_M`. The rose is where you look at
  the island itself.
* **`CFG.HSI.PLAN.MIN_RANGE` must equal `CFG.TERR.MIN_RANGE`.** Below that floor the rose takes
  its overlay off; this view would instead draw a route over a blank where the hazard layer was,
  which reads as "nothing in the way" — a lie. The code enforces the floor as well, so a
  mismatch gives a blank rather than a wrong picture.

#### Traffic (`TFC`)

A **receive-only** client of `atc_broadcast` — the same reports #44 and #64 paint (01 §6).
Nothing is transmitted; that is what makes a new listener free to add.

* **Keyed by sending computer id**, own-ship filtered **by callsign** using `CFG.TERR.CALLSIGN`
  — reused, not copied, because a hull has one name. Both rules are 01 §6's.
* **`*` white** other traffic, **`*` yellow** proximate (inside `PROX_M` *and* `PROX_V`),
  **`!` yellow** a contact that reported a TCAS resolution, **grey** coasting (older than
  `COAST_AFTER`, drawn where it was last seen). Beside each, its relative altitude in metres,
  `%+03d`, clamped to ±99.
* **Not red, amber or magenta** — those three now mean terrain on this view, and a traffic
  symbol that could be mistaken for a hazard cell is worse than no symbol.
* **`ALT_FILTER_M` is 120 m, not an ND's ±2700 ft.** 2700 ft is 820 m, more than twice the height
  of this world, i.e. no filter at all.
* **It is not a TCAS.** Every responder in the fleet hardcodes `tcas_status = 'CLEAR'`; nothing
  computes a resolution. `PROX_M`/`PROX_V` decide what turns a symbol amber and alert nobody, and
  the `!` glyph is reserved for the day a responder sends something else.
* Off-screen and altitude-filtered contacts are **not** clamped to the edge — a symbol in the
  wrong place is worse than none. The `TFC n` count on the second row is the roster's, not the
  screen's, so it is the one thing that says there is traffic the map is not showing.
* **Four tones, all already in the palette:** path and waypoints still to fly `cyan`, flown
  `blue` (the course needle's lit/dark pair), the waypoint being flown to `lime`, a destination
  that is a landing site `orange` with the same `H`/`+` glyph the landing map gives it.
* **How:** a 2×3 subpixel raster sharing `TERR.SX`/`TERR.SW`, blitted a row at a time — a 5 m
  polyline at cruise scale is thousands of points and `hFill` is three monitor calls a cell.
  Rebuilt at `CFG.HSI.PLAN.REDRAW`, re-blitted at the instrument's own rate. The compass card and
  the waypoint glyphs are cached in the **same** rebuild although they are cheap, because they are
  spatially tied to the path: a glyph moving at 20 Hz against a path moving at 10 swims, and reads
  as a fault.
* **It keeps the terrain query on the air**, at its own range — `CRS.terrain` sets `TERR.range`,
  which is what `TERR.loop` gates the map query on. (It set it to `nil` when the view was first
  built and drew no terrain; painting hazards reversed that.) `TAVOID` asks independently and was
  never affected either way.
* **Autozoom**, because there is no range selector. Three things ask for a range and the smallest
  wins, then it is snapped up `CFG.HSI.RANGES`: ground speed × `LOOK_T` (the boring default — a
  long straight leg is this and nothing else); the tightest **corner** ahead, at `CORNER_K × R`,
  **but only once that corner is inside the range it asks for** (without the second half a corner
  three kilometres away pulls the map down to a ring that cannot show it); and the distance to the
  **last waypoint** × `END_K`, which is what makes an arrival close in on itself.
* **The band is narrower than the ladder** — `MIN_RANGE` 250 m to `MAX_RANGE` 2500 m, four steps,
  and that is what "moderate autozoom" means here. The view carries a route, a terrain picture and
  traffic at once, and both ends of the full ladder are where at least one of the three stops
  being readable. The 5 km step still exists; the rose's overlay uses it.
* **Hysteresis:** zooming out is immediate — you always want to see far enough — and zooming in
  waits until the wanted range has fallen to `HYST` of **the step below the one currently
  selected**. Measuring that deadband against the step being moved *to* instead locks the map out
  of a legitimate several-step zoom-in; see 11 §10 trap 23. `HYST` is **0.9**, and that is a
  consequence of the ladder rather than timidity: the steps are 2.5× apart, so a deadband of *x*
  on the lower step is a lockout of *x*/2.5 from the upper one. At 0.75 a 400 m corner — which
  asks for 880 m — could never move the map off the 2500 m step at all.

### The terrain overlay (`TERR`)

**A display and only a display.** The FMS flies the same path with the overlay on, off, or with
the server unplugged.

* One `map` query a second, north-up, each cell reduced server-side to the worst chunk under it.
  The reply is re-read against **live altitude** on every rebuild, so a descent recolours the
  ground immediately while the radio runs at 1 Hz.
* Rebuilt at `CFG.TERR.REDRAW` (0.2 s) and **re-blitted** at 20 Hz; rebuilding the classification
  every frame would be tens of thousands of chunk decodes a second for a picture that cannot
  change in 50 ms.
* Colour is two schemes layered: **hazard bands** (relative — red within `WARN_M` below/at/inside
  a mass, amber within `CAUTION_M`) win wherever they apply; the **heightmap** (absolute, tinted
  between `TINT_LO_Y` and `TINT_HI_Y`) paints everything else. Magenta is a mass *above* us —
  deliberately off both scales, because this world's obstacles float and colouring "flying under
  an island" red would be a lie. Navy is water. **Black means exactly one thing: nobody has
  surveyed that chunk.**
* Bands are sized for a **384 m tall world**, not for a real EGPWS's thousands of feet. At
  `CAUTION_M = 150` ordinary cruise paints the whole screen yellow, and every cell under an
  island becomes a caution so "there is something above you, not a factor" is unreachable.
  `CONTEXT_M = 400` is the faithful translation of "blank terrain 600 m below": 600 m is more than
  this world is tall, so the rule becomes *never blank surveyed ground*.
* In COURSE mode the range comes from ground speed (`COURSE_LOOK` 30 s) snapped to the HSI's own
  ladder — one dial reads correctly from a hover to 320 m/s with nothing to select. 30 s, not 60,
  because 60 pins **every** airframe to the top step.
* Below `MIN_RANGE` (250 m) the overlay takes itself off the screen: at touchdown a terrain
  picture is meaningless and would be solid red.

Drawing technique: 2×3 subpixel glyphs — see
[11-conventions-and-traps.md](11-conventions-and-traps.md) §"The pseudo-bitmap".

**All sixteen palette slots are spoken for**, and the way they are spent is a **luminance
budget**: every instrument mark is drawn *over* the heightmap, so the tint is confined to the
dark third (luminance 0.03–0.19) and every mark clears its ceiling by at least 1.5×. That is why
the greens are deep and the sand is dark khaki. White is deliberately not a terrain colour.

---

## 12. `apconf.lua` — the sections

| Section | Contains |
|---|---|
| **LINKS** | `APNAV_PROTOCOL`, `AUTOLAND_PROTOCOL`, `ALTOVERRIDE_PROTOCOL`, `TOUCHDOWN`, `NAVDATA`, `APSWITCH`, the three filenames |
| **RATES** | `PING_INTERVAL` 0.2, `TICK_INTERVAL` 0.05 |
| **FRAME / SIGNS** | `HEADING_SIGN`, `HEADING_OFFSET`, **`ALT_OFFSET`** |
| **`CFG.RAD`** | the four pass radii |
| **cruise** | `SPEED_FULL`, `SPEED_SLOW`, `NEXT_CRUISE` |
| **`CFG.DC`** | the whole path planner + speed law |
| **`CFG.LG`** | pad gates, descent profile, the entire `RWY_*` approach, `RWY_GA_*`, `APPR_*` |
| **`CFG.HSI`** | instrument scales, the range ladder, and `CFG.HSI.PLAN` (the PLAN view) |
| **`CFG.TERR`** | the display overlay |
| **`CFG.TAVOID`** | the altitude floor |
| **`CFG.ATC`** | the automated-controller client: `ENABLE`, `WIDE`, `RUNWAY_ONLY`, `CONTACT_M`, `HOLD` (+ `ORBIT_R`/`ORBIT_SPEED` for `"orbit"`) |
| **`CFG.DEP`** | the runway departure (§7b): `ENABLE`; recognising the strip (`STRIP_HALFW`, `STRIP_MARGIN`, `GROUND_AGL`, `MAX_GS`); the roll (`VR`, `ROLL_ACCEL`, `XTK_TOL`, `CENTRE_T`, `BACKTRACK_WEIGHT`); the climb-out (`SPEED`, `CLIMB_VS`, `CLIMB_AGL`, `STRAIGHT_M`, `RUNOUT_M`). It also reads `LG.ALIGN_HDG_DEG`/`FINE_HDG_DEG`/`POS_TOL_M`, `RWY_STOP_MARGIN` and the `RWY_LOC_*` localizer. **Unflown estimates**; `false` on the bench |
| **`CFG.PALETTE`**, `HSI_TEXT_SCALE` | colours |

**The comments in `apconf.lua` are the configuration.** Most of those numbers were arrived at in
the air; keep each comment with the value it explains. Add knobs as `CFG.X` inside a table, never
as a bare top-level `local` — see the 200-locals note in
[11-conventions-and-traps.md](11-conventions-and-traps.md).

---

## 13. Quirks and gotchas, collected

* `msg.y` on `navap` is **world Z**.
* `APPROACH` is a *control law*, not a speed.
* `carrot` and `glide` are different flags; the rollout needs one without the other.
* The FMS reads its **own** AP-NAV redstone directly in `TAVOID.tick` — not a layering violation.
* `LND.ga` is true during a **published approach** too. One flag, two procedures, because both
  need exactly "the FMS holds the vertical".
* `LND.active` is **false** during a go-around and during a published approach.
* `LND.gaCount` is *not* reset by `engageAutoland` — it bounds the retries. It is reset only when
  the pilot commits a new plan.
* `NAV.apprGa` **survives** a go-around on purpose.
* `NAV.fromStart` overrides `PLAN.onPath`; `APR.prep` sets it *after* the caller has assigned it,
  which is an ordering rule that is easy to break by tidying.
* `LND.rwyPrep` and `APR.prep` both **mutate** `pts`, and must run after `setLandFinal` and
  before `PLAN.build` and before `NAV.index` is chosen. `APR.prep` runs before `LND.rwyPrep`.
* `CFG.HSI` was **dead** until 2026-09-16 — `autopilot.lua` carried a duplicate `local HSICFG`
  that shadowed it (11 §10 trap 22). Numbers in that section of an old `apconf.lua` may therefore
  never have been the ones flown; `RANGES` in particular had drifted apart between airframes.
* On the **display bench (#46)** nothing listens: no UAC, no relay, no thruster. It exists to
  demonstrate the HSI.
* The ATC client identifies itself by **`CFG.TERR.CALLSIGN`**, the same name the own-ship filter
  uses. It must equal the hull's `responder.lua` `CALLSIGN`, or the controller reads the landed
  aircraft as somebody else sitting on its facility.
* An **ATC hold is not COAST**, though both are a zero `cruiseCap`: the hold also sets `align`
  (so the UAC brakes the whole velocity vector), and it blocks the landing from engaging.
* A departure's ground run sends `navap` **type `"dep"`**, never `rwy`/`land` — those two are the
  GPWS's "on final" test. And its autoland messages carry **`dep = true`**, which nothing reads
  yet: until `gpws.lua` learns it, the HUD's autoland line reads `LAND LANDED` on the runway and
  `LAND DESCEND` through the climb-out.
* `DEP.reset()` must be called **before** a commit's teardown (it reads `LND.active`/`LND.ga` to
  learn whether the vertical was held) and `DEP.commit()` **instead of** `PLAN.build` at its end.
  A fourth commit path that skips either reopens the latched-vertical bug §7b closes.
* **An arc's sweep goes through `GA.sweep`**, never a bare `% (2*math.pi)`: a turn of nothing comes
  back from rounding as `-1e-16`, and that modulo is a full circle (11 §10 trap 28). `GA.dubins`
  had this too; a go-around already aligned with its leg could have flown a 360.
