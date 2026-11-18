# 09 — The navigation database (`points.db`, `course.db`, `landing.db`)

*Line-based text with `;` comments, hand-editable, shared in format between every aircraft, the
ATC scope, the GPWS and the nav data server. The master copy lives on #24; every other copy is a
refresh of it.*

> **Both parsers must stay in agreement.** The server refuses to write anything the FMS would
> silently skip — a line the server accepts and the FMS drops is far worse than an error message
> at the keyboard.

---

## 1. `points.db` — aerodromes, POIs and waypoints

```
; aerodromes and/or point-of-interests (always three letters)
TMU=666,588
WAX=-2321,22260
; waypoints (VORs are always five letters long)
TAIMU=609,160
KOBUL=-2496,22516
```

```
NAME=x,z
```

* `NAME` is **letters only**, and its **length is its type**: **3 letters = POI/aerodrome**,
  **5 letters = VOR/waypoint**. Nothing else is accepted (`validatePointName`).
* `x,z` are Minecraft **world** coordinates, integers. There is no altitude here — a point is a
  ground position. Altitude, where a point has one, lives in `landing.db`.
* The two comment headers are **regenerated** on save, with POIs first and VORs second. Ordering
  within each group is load/insertion order.

### Why length is the type

`classify(name)` is `#name == 5 and "vor" or "poi"`, and that single fact drives the contextual
pass radii ([02-fms-autopilot.md](02-fms-autopilot.md) §5), the HSI symbology and whether the ATC
scope draws it at all. **Renaming a point from 3 to 5 letters silently changes how it is flown.**

### Coordinate convention (from `landing.db`'s own header)

```
North: Z-      East: X+      South: Z+      West: X-      Altitude: Y
```

---

## 2. `course.db` — named routes

```
; routes
TMU-WAX=TMU,UKRAN,UHAKF,NOIDU,RNBUL,ZNTRA,DAOFU,DOUAN,DSRET,QATON,WAYCL,WAUZU,KOBUL,WAUZA,WAX
WAX-TMU=WAX,WAU-TMU
```

```
ROUTE-NAME=N1,N2,...,Nn
```

* Name charset is `A-Z 0-9 -`. At least two waypoints.
* **A token may name another course instead of a point**, and that course's own (already
  expanded) sequence is spliced in where the reference stood. `WAX-TMU=WAX,WAU-TMU` is *"WAX,
  then whatever WAU-TMU flies"* without restating its fixes.
* **A token may also name a published approach** (`appr`) out of `landing.db`, and its fixes —
  expanded by `APR.expand`, so a chain inside `landing.db` resolves the same way it does when
  the FMS picks the procedure itself — are spliced in. `TMU-TMV=TMU,TAIMU,PDRNT,TMV_APPR4` is
  *"…then arrive by TMV_APPR4"*: a route that **names the arrival it is meant to be flown with**
  instead of leaving the FMS to choose one at landing preparation. See §2.1.
* **A go-around id is not a route** and is skipped like any other unusable name. A missed
  approach spliced into an arrival would fly a lap of the aerodrome on the way in — the same
  reason `APR.expand` contributes only the runway when an `appr` names one.
* **Cycles and runaway nesting are bounded rather than trusted**: a name already on the reference
  stack is skipped, and `CFG.COURSE_MAX_REF` (4) caps the depth. So a cycle in `course.db` cannot
  hang the FMS; the depth limit only bounds a silly chain. An approach reference is bounded by
  `landing.db`'s own `LG.APPR_MAX_REF` instead — the two namespaces are separate (a course can
  never be a fix of an approach), so `APR.expand` gets a fresh stack.
* **An unknown name is skipped, not fatal** — the same behaviour a course naming a
  since-deleted point has always had. The route is silently shortened around it.
* **Two identical fixes in a row are collapsed** as the list is written (`pushFix`), because a
  reference is normally written *at* the join fix and the thing referenced often starts there
  too — `…,PDRNT,TMV_APPR4` where the approach's own entry is PDRNT. A zero-length leg is
  something the corner solver has no meaningful answer for. Only **consecutive** repeats go: a
  procedure visiting one fix twice with something in between is a procedure turn and deliberate
  (`TMV_APPR1` does exactly that).
* Blank lines are preserved as visual grouping by the server's writer.

### 2.1 What a course-with-an-approach buys

Nothing downstream is special-cased. The expansion ends at the landing site, so such a route
**auto-lands** like any other route that ends at one; and because the plan's tail then *is* a
published pattern, flown from its published entry, `APR.prep` case 1 (`APR.established`) matches
it and **records** it — the published minimum altitude and the missed-approach circuit come with
it, exactly as if the FMS had chosen that approach itself. See
[02-fms-autopilot.md](02-fms-autopilot.md) §7.

The difference from letting `APR.choose` decide is that the choice is **published rather than
computed**: an aerodrome whose best arrival from a particular route is not the one `APR.cost`
would pick can now say so in `course.db`.

Courses are also the **auto-router's graph**: nodes are points used by some course, edges are
consecutive pairs, **undirected**. The first and last point of every course is an
**interchange hub**, and those are the only destinations the auto-router offers.

**Reverse courses exist for every route** by convention (`TMU-WAX` and `WAX-TMU`), which is
belt-and-braces given the graph is undirected, but it also makes both directions selectable by
name in the FLY ROUTE picker.

---

## 3. `landing.db` — four record shapes in one file

The header block at the top of the file **is the specification** and is reproduced verbatim
whenever #24 rewrites the file. Do not delete it.

### (a) A landing SITE

```
NAME=type,direction,altitude,runwayLength
TMU=heli,n,64,0
TMV=rwy,s,64,295
WAX=rwy,56.30993,116,166
```

| field | meaning |
|---|---|
| `type` | `heli` or `rwy` |
| `direction` | `n` / `e` / `s` / `w`, `x` for *don't care*, or a decimal **heading in degrees** |
| `altitude` | the **surveyed surface Y** of the pad or threshold. TMV/TMA are 64, i.e. standing on the block at Y = 63 |
| `runwayLength` | metres; ignored for `heli` |

* The point must also exist in `points.db`; the `landing.db` entry only adds the landing data.
* For a runway, the `points.db` position is the **threshold**, and the strip runs `rwyLen` metres
  in the landing direction. `TMV=609,598 dir s, 295 m` therefore ends at `609,893` — and `TMA` is
  **the same strip flown the other way** (`609,893`, dir n).
* `direction = x` makes a site unusable as a runway (`LND.rwyUsable`) and it falls back to a pad
  descent, which works for any site.

> **A `heli` record is only a 3D point.** Nothing promises there is anything solid under it. WCL
> sits at Y 226 with open air below and is meant to be **station-kept**, not landed on — which is
> exactly why the pad machine probes rather than trusting the altitude. See
> [02-fms-autopilot.md](02-fms-autopilot.md) §7.

### (b) A go-around LEG (`goaround`)

```
<id>=goaround,startX,startZ,endX,endZ,altitude
```

One **straight, oriented** line segment; the FMS invents the rest of the circuit around it (climb
away, curvature-limited join to the start, down the leg, back onto the runway line).

* **The id is arbitrary.** Which runway it serves is inferred from its **leg direction** vs. the
  landing direction (`RWY_GA_ALIGN` ≥ 0.5, i.e. within 60°), ties going to the nearest start.
* **Geometry you have to get right:** the leg must **end far enough out** that the altitude can be
  flown off before the aim point — roughly
  `(altitude − field elevation) / tan(RWY_GS_DEG)` metres, plus the aim distance. End it closer
  and the aircraft arrives above the glidepath, cannot get down, and goes around again until
  `RWY_GA_MAX` stops it.

### (c) A go-around CIRCUIT (`goaround-wp`) — **preferred**

```
<id>=goaround-wp,minimumAltitude,<waypoints...>
TMA_GAWP=goaround-wp,150,TMA,TAIMU,CECIN,NATIO,VAROI,GOGET,NAROD,VOTRE,SLAVA,TAIMW,TMA
```

The whole pattern, drawn fix by fix, **starting AND ending at the runway it serves** — which is
also how it is matched to that runway, **by name**, with nothing inferred.

* The FMS flies it exactly as it flies any published course (jerk-limited corners, straight-in
  final at the end) and descends by extending the glidepath back along whatever is left of the
  pattern: hold the minimum until the glidepath sinks below it, then ride it in. **So there is no
  geometry to get right** — draw the circuit you want flown and pick a minimum altitude that
  clears the terrain.
* Written the other way round (only the *first* entry naming the runway) it is **flown backwards
  rather than refused**, which is also what makes the two ends of a single strip pick out their
  own circuit and not each other's.
* Waypoints must exist in `points.db`. If none of the middle ones resolve, the FMS falls back to a
  straight-leg record for the same runway before giving up.

**When a runway publishes both shapes, the waypoint circuit is always the one flown.** It is a
*drawn* procedure rather than a *deduced* one; the straight-leg record is then dead weight kept
for aircraft on older software.

### (d) A published APPROACH (`appr`)

```
<id>=appr,minimumAltitude,<fixes...>
TMV_APPR1=appr,150,PDRNT,TAIMU,NATIO,KOCNI,TAIMU,TMV
WAX_APPR1=appr,180,WAYCL,WAUZU,KOBUL,WAUZA,WAX
```

**How the aircraft ARRIVES.** A field publishes one per direction it can be arrived from, and the
FMS chooses **one** when the plan is prepared:

> *"on preparation to the landing, the FMS MUST choose the most suitable approach pattern
> according to its current position and heading"*

An approach **qualifies for a site by ending at that site's name**, and is **never reversed** to
make it fit — a go-around circuit may be flown either way round, but an approach runs one way
only, into the field.

Note `TMV_APPR1` visits `TAIMU` **twice**; that is legal and deliberate (a procedure turn).

---

## 4. References between patterns

Both go-around and approach records may name another pattern's id in their fix list, and **the
two directions mean different things — as they do on a real approach chart:**

| reference | meaning |
|---|---|
| `appr` naming another **`appr`** | *continue as that approach*; its fixes are spliced in where the id stood. How a family of arrivals shares one common tail: `TMV_APPR3=appr,150,NTARI,TMV_APPR2` is "NTARI, then fly TMV_APPR2" |
| `appr` naming a **go-around** | that circuit is **this approach's missed-approach procedure**, and it contributes the **runway** that circuit serves. `TMA_APPR1=appr,150,KUTAC,UKRAN,TMA_GAWP` is "KUTAC, UKRAN, land at TMA — and if you miss, fly TMA_GAWP" |

**Splicing the circuit's own fixes in would fly a whole lap of the aerodrome on ARRIVAL**, which
is not what a missed approach printed at the bottom of an approach chart means.

A published pairing **beats** matching by name (which cannot tell two circuits at one field
apart) and beats inferring by geometry outright. `NAV.apprGa` survives a go-around on purpose, so
a second attempt flies the same circuit as the first.

Nesting is bounded by `LG.APPR_MAX_REF` (4), and an id already on the stack is skipped — because
this runs on data that arrived over rednet and that nobody has flown yet.

---

## 5. Which record shapes are PATTERNS, and why the test is a whitelist

```lua
APR.PATTERNS = { goaround = true, ["goaround-wp"] = true, appr = true }
```

`isLandingSite(name)` is `LANDING[name] ~= nil and not APR.PATTERNS[type]`.

Listed **by name** rather than tested as "anything that is not `heli`/`rwy`" so that:

* a hand-added *site* type still lands, and
* a fourth *pattern* shape has to be named here before anything treats it as one.

That is the safe way round for both mistakes. Patterns have no point of their own and **none may
ever be offered as a destination**.

---

## 6. Editing

Three places, with different durability:

| where | durability |
|---|---|
| **#24's editor** (keyboard + mouse) | **authoritative**; broadcast to the fleet at once |
| **#24's files by hand** | authoritative once reloaded |
| an aircraft's **EDIT DATA** screens | **transient** — the next sync from #24 overwrites them |

`sites.db` (§9) has no editor anywhere: it is edited by hand on #24 and goes out on the next
broadcast.

A refresh cannot move a course being flown: `NAV.points` and `NAV.landSite` are snapshots taken
at plan-set time (invariant I6). It *is* held while the pilot is on any screen that lists or edits
data, because a picker would renumber under their finger between the draw and the tap.

---

## 7. Where the copies live

| computer | why it has one |
|---|---|
| #8, #16, #28, #39, #74, #46 | the FMS flies it |
| #44, #64 | the scope draws the map from it |
| #67, #70, #71 | the GPWS resolves the SITE datum and the approach test from it |
| #87 | the ATC controller: `points.db`, `landing.db` and **`sites.db`** (it keeps no `course.db`) |
| **#24** | the master — of `sites.db` too |

All are kept in step by `aer_navdata`. If one is out of date, check its ack on #24's roster
before assuming a parse problem.

---

## 8. Checklist for adding a new aerodrome

1. `points.db` — a **3-letter** name and its `x,z`.
2. `landing.db` — a site record: `heli` or `rwy`, the direction, the **surveyed surface Y** (stand
   on it and read the block's Y, then add 1), and the length for a runway.
3. For a runway, decide whether the **other end** is a separate site (it usually is — `TMV`/`TMA`)
   and add its point and record too.
4. If it should be reachable by the auto-router, it must be the **first or last point of some
   course** — that is what makes it a hub.
5. Publish a `goaround-wp` circuit if you want automatic missed approaches, and an `appr` for each
   direction the field can be arrived from. Reference the circuit from each approach so the
   pairing is explicit.
6. Edit on **#24**, not on an aircraft, and confirm on the CLIENTS roster that the aircraft have
   acked the new `stamp`.
7. If the field is somewhere the fleet has not flown, the terrain survey may not cover it — check
   #68's coverage (`activity.txt` drives the incremental rebuild).
8. If the automated controller should manage it, add it to a site in `sites.db` (§9) — **and
   both ends of a strip**, or the end left out is "uncontrolled" and can be cleared head-on into
   the other (#87's board flags exactly this).

---

## 9. `sites.db` — the ATC controller's sites

```
; comments as elsewhere
TMU=TMU,TMA(narrow),TMN(narrow)
NUK=NUK,NUS,NUN
```

```
SITE=FACILITY,FACILITY(remark),FACILITY(remark remark)
```

* A **site** is an aerodrome; its **facilities** are landing sites from `landing.db` (pads or
  runways) that may stand in for one another. When the one an aircraft asked for is taken, the
  controller (#87) offers the others **in the order written here**.
* The site name is `[%w_-]+`; a facility name is a `landing.db` site (`%a+`). A facility may
  belong to **one** site; a second mention is a fault on #87's board and is ignored.
* **Remarks** go in brackets after the facility, several separated by spaces (or commas, which
  are allowed inside the brackets). The vocabulary is `atcctlconf.lua`'s `CFG.REMARKS`:

  | remark | refuses |
  |---|---|
  | `narrow` | an aircraft whose `apconf.lua` says `CFG.ATC.WIDE = true` |

  **An unknown remark closes the facility** and says so on the board: a restriction the
  controller silently ignored would be a clearance onto a facility the author said was not safe
  for somebody.
* A landing site in **no** site is uncontrolled. Two ends of one runway (TMV/TMA, NUS/NUN) are
  one piece of ground to the controller whatever this file says — their footprints overlap — but
  an end that is in no site is not asked about, so list both.
* Only #87 reads it. It lives on #24 (the master) and travels in the sync's `sites` field
  ([01](01-protocols.md) §7).
