# 08 — Ground stations and shared services

*Five computers that are not on any aircraft, plus the responders that feed two of them.
Everything here is a database, a display, or (#87) an allocator of landing facilities;
**none of it can move a thruster.***

---

## 1. #24 — the nav data update server

`nav_update_server.lua` + the master `points.db` / `course.db` / `landing.db` (plus
`navdata.rev`), and — since 2026-09-28 — the master `sites.db` for the ATC controller (§7).

`sites.db` is **carried, not edited**: there is no editor for it here, only the same re-read
before every send, so it is changed by hand in the world save. It goes out as the sync's
`sites` field, which every client but #87 ignores; the dashboard counts its lines (`SITES n`).
It is optional — no file is "no sites" — but a copy that vanishes or empties while a non-empty
one is being served is refused like the other three, so a file caught mid-edit cannot drop
every clearance ([01](01-protocols.md) §7).

### Why a server at all

Every FMS **carries its own** `.db` files and reads them at boot — that stays true, and is
deliberate: an aircraft out of radio contact still has a complete database and flies normally.
This computer does not replace those files, it **refreshes** them.

### The design rule that makes it safe

**The wire format is the file itself.** The sync message carries the three `.db` files as plain
text, exactly as they sit on this computer's disk. A client writes them straight to its own disk
and then calls its **existing** `loadPoints()` / `loadCourses()` / `loadLanding()`.

There is therefore **no second parser and no second schema anywhere in the fleet**, so this
program cannot drift out of format agreement with the aircraft. Its own parsers and validators
mirror the FMS's patterns **exactly** — *a line this refuses is a line the aircraft would
silently skip, which is far worse than an error message here.*

### The disk is the master, not the program's copy of it

**`NET.send` re-reads the three files before every sync** — the periodic broadcast, the reply
to a `pull`, and the immediate send after an edit alike (`refreshFromDisk`, added 2026-09-10).
These files are edited from outside the program at least as often as through its editor — in
the world save, with the server left running — and before this the fleet went on being served
the boot-time contents until someone pressed **R**.

The stamp comparison makes it free in the common case: only a *changed* file is re-parsed, and
the send after an edit is a no-op by construction, because `commit()` has just written exactly
what is in memory.

It **refuses a partial disk**, where `loadAll()` does not: a file that has gone missing, or
that reads back empty while a non-empty copy is held, keeps the copy already being served and
raises a fault on the dashboard (line 6, in red) instead of broadcasting an empty `points.db`
over every aircraft's working copy. **R is the deliberate override** — `doReload` goes through
`loadAll` and adopts the disk whatever it says.

An outside edit does **not** bump `rev` (that counts saves made here); the `stamp` changes,
which is what a client compares. It is also not deferred while someone is on a form here — the
form buffer is separate, and saving it re-serialises the merged database.

### Protocol

See [01-protocols.md](01-protocols.md) §7 for the message shapes and the client-side
stage-then-swap discipline (which is identical in `autopilot.lua`, `atc.lua` and `gpws.lua`).

* Broadcasts every 30 s, and **at once after any edit** — **re-reading the disk first, always**.
* Answers a `pull` with a targeted sync, so a fresh FMS need not wait 30 s.
* `rev` is a monotonic counter, persisted, for display and ordering only. **`stamp`** — a
  checksum of the three payloads — is what a client actually compares.
* The **CLIENTS roster** on the dashboard is how you confirm an edit actually landed. A client
  that is holding an update (parked on a data screen) or that cannot write one (full disk) says
  so in its ack, so it does not look identical to one that never heard the server.
* `CLIENT_STALE` 90 s — longer than three broadcast intervals — before a client is shown silent.

### Editing

A full keyboard+mouse editor: points, courses and landing records, with validation that mirrors
the FMS's (`%u+` 3 or 5 letters for a point; `[%u%d%-]+` for a course; the four landing record
shapes). It refuses to delete a point still referenced by a course or a pattern.

A course entry that is not a point may still be a legal **reference** — another course, or a
published `appr` out of `landing.db` ([09](09-navigation-database.md) §2). `courseRefKind` is the
one place that decides which, and both the save check (`courseUnknowns`, which would otherwise
refuse the save) and the course editor's row (which would otherwise paint it red as an "unknown
point", inviting someone to tidy a working route away) ask it. The **waypoint picker offers them
too** — points first, then the other courses, then the published approaches, drawn in orange so
a reference never reads as a fix — which is what makes such a route authorable here rather than
only by hand-editing `course.db` and rebooting this computer. The course being edited is left
out of its own picker: a self-reference expands to nothing.

`LANDING_HEADER` is reproduced verbatim whenever `landing.db` is rewritten — **it is the only
documentation of the field order and of Minecraft's compass/axis convention, and it belongs in
the file rather than only in this program.**

### The one-way rule

**Editing here is authoritative.** A sync overwrites whatever an FMS holds locally, including
edits made on the aircraft's own EDIT DATA screens. Those still work; they are just transient.
**Changes meant to last belong on this computer.**

### Clients

Every FMS (#8, #16, #28, #39, #74, #46), the ATC scope (#44), the AWACS (#64), all three
GPWS computers (#67, #70, #71), and the ATC controller (#87). *The scope subscribing is why the
radar map is the same database the aircraft fly.*

---

## 2. #44 and #64 — the radar scope (`atc.lua`)

**One program deployed to two computers with different `CFG.STATION.MODE`.** Treat
`44/atc.lua` and `64/atc.lua` as one file and keep them in step.

| | **#44 TMU** | **#64 AWACS** |
|---|---|---|
| `MODE` | `static` | `dynamic` |
| centre | `CFG.STATION.X/Z` (705, 647) | this aircraft, read off `sublevel` every frame |
| `ORIENTATION` | `"north"` | `"heading"` (or `"track"`); `[NUP]` forces north-up any time |
| also needs | `Y` (a **surveyed** elevation — takes no `ALT_OFFSET`) | `HEADING_SIGN`, `HEADING_OFFSET`, `OWN_CALLSIGN`, **`ALT_OFFSET`** |

**Which knobs are required depends on the mode**, and the loader checks the set that mode uses
and names the missing one at boot — so a tower config cannot be half-converted into an airborne
one.

`heading` vs `track` matters here: on a VTOL the two part company routinely — Aer Togekiss will
hold a heading and translate sideways. Heading is chosen for a **lookout console**, because the
operator is matching what is on the scope to what is out of the window, and the window follows
the nose.

### It does not control anything

There is **no clearance, no vectoring, no separation logic**, and nothing is ever transmitted to
an aircraft. (Landing clearances exist since 2026-09-28, but in a **separate program on a separate
computer** — #87, §7 — so that the scope stays a display with one reason to be wrong.) The station is receive-only on the traffic channel; the only thing it ever sends is
a nav-data acknowledgement to #24. *Pilots fly themselves; this tells whoever is in the tower
what is out there.*

### Tracks

Keyed by **sending computer id**, not callsign — see [01-protocols.md](01-protocols.md) §6 for
why, and for the defensive field reading.

```
COAST_AFTER  3 s   → dimmed and flagged C, still drawn where it was last seen
DROP_AFTER  20 s   → leaves the scope
```

*A responder that misses a second of reports has not gone anywhere, and a blip that vanishes and
reappears is far harder to read than one that goes grey.*

Targets sort **nearest first**, so the traffic strips read down the panel in the order a
controller cares about and the nearest survives if the panel runs out of rows.

The **leader line** is along the ground **track** (from `vx`/`vz` where the responder sends
them), showing where the target will be in `LEADER_SEC` (30 s), capped at `LEADER_MAX_FRAC` of
the current range.

### The map underneath

The same `points.db` / `course.db` / `landing.db` an aircraft carries, kept current by
subscribing to #24 exactly as an FMS does. So a runway added to the fleet database appears on
this scope within a broadcast interval, with **no copying by hand**.

`SHOW_WAYPOINT` (five-letter VORs) is **off by default**: en-route fixes swamp a wide-range
picture. Airways from `course.db` are drawn as faint lines.

### Drawing

A **double-buffered** cell grid with its own clip rectangle, flushed once per frame — not
`setCursorPos`/`write` per mark.

---

## 3. The responders (#45, #53, #73, #82, #83)

`responder.lua`, one per aircraft, **no conf file**: callsign, protocol, `HEADING_*` and
`ALT_OFFSET` are hardcoded at the top.

| computer | aircraft | callsign |
|---|---|---|
| #45 | Aer Togekiss | TOGEKISS / TGK |
| #53 | Mantine | MANTINE / MTI |
| #73 | Scout | SCOUT / SCU |
| #82 | SS Argo | ARGO / RGO |
| #83 | *(airbus, in build)* | AERBUS / RBS |

It reads the pose and velocity at 20 Hz and broadcasts on `atc_broadcast`. `is_ifr` is simply
whether AP-NAV is on, learnt from `<family>_apswitch_1`.

**#83 is the only computer the airbus has so far** — it appears in the fleet table for its
protocol family and callsign and nowhere else.

> `ALT_OFFSET` here must match the rest of the hull: **vertical separation read off a scope is
> only as good as the worst-calibrated aeroplane on it.**

---

## 4. #68 — the terrain data server

`terrain_server.lua` + `terrainconf.lua` + a disk of tiles + `activity.txt`.

### Why it exists

*Create: Aeronautics* gives an airframe its own pose off `sublevel`, but **nothing gives it the
shape of the world**. Every terrain-derived GPWS mode was wired and unarmed, because a warning
derived from a guess is worse than no warning. This computer is the missing half.

### Where the data comes from

**Not from in-game scanning:** the overworld here is 2 596 regions and 2.06 million chunks, and
no CC computer is going to walk that. **`terrainbuild.jar`** (beside the server launcher) reads
`world/region/*.mca` directly — Anvil, NBT, palette and all — and writes **one 4 128-byte tile
per region** onto the disk in the drive.

This program only ever **reads** those files. The builder swaps each tile atomically, so a tile
is either last week's survey or today's, never half of each, and the server needs no locking to
be safe against a rebuild happening underneath it.

### Picking a rebuild up (`CFG.DB.RELOAD`, 30 s)

Tiles are **cached, misses included** — most of the coordinate plane has no tile, so without
negative caching every query over open nothing would stat the filesystem once per chunk. That
means a rebuild landing while the server is up was invisible to it: a region surveyed for the
first time stayed "never surveyed" until the computer was rebooted, and the fleet drew black over
ground the builder had already charted.

`DB.loop` therefore drops the cache and re-reads `meta.txt` every `CFG.DB.RELOAD` seconds.
Dropping the cache **is** the mechanism — there is nothing to re-read eagerly, since the next
query re-reads only what it needs off a local disk; the cost is the first query after each reload
paying for its own tiles again. Re-reading the meta is how the fleet is told: `built` and `tiles`
go out on every beacon. `0` disables it (a rebuilt survey then needs a reboot), which is why it
is a **required** knob — a defaulted 0 would look exactly like a working server quietly serving
the survey it booted with.

**Failure keeps the last good database rather than throwing** — clients are flying on this data:

| what happened | what the server does |
|---|---|
| disk gone / no `meta.txt` | keeps the directory, the meta **and** the cache; raises a fault; tries again next time |
| `meta.txt` now states a **different geometry** | keeps the old meta (the beacon must not advertise a survey we have refused to read), stops reading new tiles (`DB.frozen`), says so. Heights out of a database built at another Y resolution would be confident, plausible and completely wrong — the same reason it is a boot error. It thaws by itself once the geometry matches again |

`DB.loop` is one of `parallel.waitForAny`'s functions, so a disabled reload **parks** rather than
returning — `waitForAny` finishes the moment any of them returns, and switching the reload off
would otherwise shut the server down at boot.

### The record

One record per chunk (16 × 16 m), one level per 8 m of Y, 4 bytes:

```
TERR  top of the GROUND, as a level. Rounded UP, so it errs high. 255 = never surveyed
OTOP  top of anything FLOATING above that ground, or 255
OBAS  the underside of the lowest floating thing, or 255
FLAG  MASS(2 bits) | WATER | MULTI | TREES | SPIKE | VOID | NODATA
```

**Terrain and obstacle are separated in the BUILDER, not here** (see `ChunkAnalyzer`), so the
distinction is made once against the real blocks rather than re-guessed by every client.

`GEO`: `MIN_Y = -64`, `CELL = 8`, `LEVELS = 48`, `TILE = 32` chunks per edge (= one `.mca`
region), `HEADER = 32` bytes, `RECSZ = 4`.

### Queries

| query | answer |
|---|---|
| `corr` | a corridor along a track: `n` samples `step` metres apart, each **reduced across the corridor width `w`** — the worst ground in the swathe, not one line of it |
| `map` | a **north-up, axis-aligned** `nx × nz` block of `cell`-metre squares, each reduced to the worst chunk under it. Row-major in +Z |
| `tile` | one raw tile |
| `gnd` | one record |

North-up rather than heading-up is deliberate: the grid then stays valid **through a turn**, and
the client rotates it into its own display frame for two multiply-adds per cell.

`pack` on a map reply cuts it to **one byte per cell** plus a sparse obstacle list — a quarter of
the wire for exactly the same picture. That is what makes a grid fine enough for subpixel drawing
affordable (96 × 96 packed ≈ 9 kB, *less* than the old 40 × 40 × 4).

Limits (`CFG.LIMITS`) bound what a client may ask for: `SAMPLES` 768 per corridor, `LATERAL` 7
chunks either side, `MAP_CELLS` 128, and `MAP_WORK` 12 000 — the server **reduces `sub` itself**
if a grid would cost too much rather than refusing it.

The server is **stateless** and answers a broadcast as readily as a direct send, so a client that
has not yet learnt its id is not stuck waiting for a beacon.

### `activity.txt` — the feedback loop to the offline builder

Every client request carries a `callsign` and a position. The server keeps a roster of who has
been seen where (`KEEP` seconds, capped at `MAX`) and rewrites `activity.txt` every 20 s:

```
; aer terrain activity -- written by computer #68, read by terrainbuild.jar
; callsign  x  z  y  seen_utc_ms
TOGEKISS 666.2 588.0 63.1 1788547697077
ARGO 778.9 724.2 61.2 1788547696976
```

**That is what tells the offline builder where to re-survey.** The fleet's own flying decides
which regions get rebuilt.

### Height bias, restated

Every height reported is **the top of its cell**, which rounds the ground up by as much as
`CELL − 1` metres. That is the safe direction and it is deliberate: *a GPWS that thinks the hill
is 7 m taller than it is calls too early; one that thinks it is 7 m shorter calls too late.*

---

## 5. #18 — the tick probe

`tickprobe.lua`, and **nothing else runs on this computer** — so what it measures is the
server's own scheduling, not any flight program's.

Three phases, each the same `sleep(0.05)` loop with one more kind of work in it, so the
difference between two phases is what that work costs:

| phase | work |
|---|---|
| `BARE` | sleep only — the floor; what the scheduler gives a computer that asks for nothing |
| `RADIO` | one `rednet.broadcast` per iteration |
| `SCREEN` | ten lines of terminal writes — a full instrument redraw |

Both clocks are read on purpose: `os.clock()` is the computer's own uptime (what the UAC times
itself with) and `os.epoch("utc")` is real wall time. **If they disagree, the slowdown is an
artefact of the clock rather than of the scheduler, and that changes the answer entirely.**

It found a computer getting **0.30 s per iteration against the 0.05 s its `sleep(0.05)` asks
for** — a dead-on 6×, with **zero jitter** across 200 samples. Zero jitter is the tell: a merely
busy program runs late by a *varying* amount, while a fixed ratio is what a **budget** looks like
(`max_main_global_time`, 10 ms per tick shared by every computer on the server).

**That figure is the dead time every attitude gain in `uacconf.lua` is derived from. Read #18
before blaming a control loop for being slow.**

---

## 6. #46 / #47 — the `demounit` display bench

**Not a vehicle.** #46 runs the FMS purely to demonstrate the HSI; #47 runs the altitude selector
purely to demonstrate the artificial horizon. Nothing flies, so there is no UAC, no relay and no
thruster on the other end of its protocols.

`ALT_OFFSET = 0` on both — there is no hull. *That is precisely why a defaulted 0 elsewhere would
be so dangerous: it is a legal value here.*

Treat both as instrument showcases and keep them running whatever the current instrument code is.
#46 ran an `autopilot.lua` one change behind the fleet until 2026-09-17 (it lacked the
`align` field on `navap`); it is now on the same byte-identical build as the five
aircraft. The field is inert here in any case — nothing listens to the bench.

---

## 7. #87 — the automated ATC controller

`atcctl.lua` + `atcctlconf.lua` + `sites.db` (from #24) + `parked.db` (its own memory).
**The folder was seeded before the computer existed in the world** (2026-09-28): the next
computer placed is given id 87 and boots straight into it. It needs a modem that reaches the
whole fleet (as #24 and #68 do) and a chunk that stays loaded; a monitor is optional — the board
is drawn on the terminal as well.

### What it decides, and what it does not

It allocates **ground**. An aircraft inbound to a facility asks, once a second, whether it may
land there ([01](01-protocols.md) §8a), and gets one of: **cleared**; **divert** to another
facility of the same site; **hold**; or **uncontrolled** (the facility is in no site). It never
names a heading, an altitude, a route or a speed — a divert is a facility *name*, and the FMS
flies there with its own DIRECT-TO-LAND machinery ([02](02-fms-autopilot.md) §8a). Nothing here
separates aircraft in the air.

**Fail silent.** Switched off, it changes nothing: every FMS lands where its plan says, as before
it existed.

### Occupancy — from the responders only

Every facility is a **capsule**: a pad is a circle of `PAD_R`; a runway is its strip, threshold
to far end (laid out exactly as the FMS lays it out), extended `RWY_END_M` past both ends, with
`RWY_HALF_W` either side. A responder report inside a capsule, between `BELOW_MAX` under and
`AGL_MAX` over the facility's surveyed altitude, **occupies** it; the facility then stays busy
`FREE_AFTER` seconds after the last such report.

Two facilities whose capsules **overlap** are one piece of ground and are never cleared at the
same time — which is what stops a clearance onto TMV while another aircraft is cleared onto TMA
(the same strip the other way), whatever `sites.db` says. The board flags a landing site in
**no** site that overlaps a controlled one, because an aircraft asking for it is told
"uncontrolled". Sized for the largest airframe, the capsules do more than that: AGS's pad sits
29 m off AGC's centreline, so those two conflict and the AGS site takes one aircraft at a time.

**The silent-aircraft memory.** A parked aircraft goes silent when nobody is near enough to keep
its chunk loaded, and it is still there. So an aircraft seen sitting still (`PARK_GS`) on a
facility for `PARK_AFTER` seconds is remembered on it — on disk, in `parked.db` — until its own
responder is heard **anywhere off that facility**. A record for an aircraft that will never
transmit again is cleared by **touching that facility's row** on the board.

Identity is the **callsign**: a requester is never blocked by its own responder, which is why an
FMS's `TERR.CALLSIGN` must equal its hull's `responder.lua` `CALLSIGN`. A hull with no responder
(Citation today) is invisible: once it has landed, its facility reads free.

### Allocation (`CTL.allocate`, on every request and twice a second)

1. **Standing assignments first**, in queue order — kept while the aircraft is still asking the
   question it was assigned against and the facility is still usable. Nobody earlier in the
   queue can take a runway from under an aircraft already cleared onto it; nobody's clearance
   survives someone else **sitting down** on it.
2. **Then first come, first served**: the facility asked for if usable, else the first usable
   one of the same site **in `sites.db` order**, else hold (with a place in the site's queue).
   The queue place is the time of the first request and survives a divert or a go-around.

*Usable* = not closed by a fault, not refused by a remark (`narrow` refuses `wide`), a runway if
the aircraft asked `rwyonly` (a FIXEDWING is never diverted onto a pad), nobody but the asker on
it or on anything overlapping it, and not assigned to anybody else.

A request not renewed within `REQ.STALE` (5 s) is dropped with its reservation: the aircraft
landed, cancelled, went AP-NAV off or released ATC. If it landed, its responder holds the
facility from then on.

### The board

Every controlled facility with its state — `FREE`, `CLR <who>`, `OCC <who>`,
`OCC <who> silent, parked <age>`, `BLOCKED by <facility>`, `CLOSED <why>` — then every aircraft
on frequency with what it was told and why, then `sites.db` faults (red) and overlap advisories
(yellow). Repainted on a timer, never per packet: it hears every responder at 20 Hz.
