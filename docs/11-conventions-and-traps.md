# 11 — Conventions, frames and traps

*The things that are true everywhere in this tree, and the specific ways each has bitten. If
you are about to edit anything, read §1–§4 first.*

---

## 1. Frames, signs and angles

### Coordinates

```
North: Z-      East: X+      South: Z+      West: X-      Altitude: Y
```

### Bearing

Every bearing in the flight system is `atan2(dx, dz)` — **x first**, which is the frame
`navcom.lua` established and every program inherits. In that frame **north is π**, not 0.

```lua
local function toCompass(rad) return (180 - math.deg(rad)) % 360 end
```

`toCompass` is duplicated verbatim in the FMS, the GPWS and the ATC scope. That is deliberate: it
is three lines and a shared library would be a fourth file to keep in step.

### Heading

```lua
heading = CFG.HEADING_SIGN * getYaw(pose.orientation) + CFG.HEADING_OFFSET
```

`HEADING_OFFSET = math.pi` on every airframe: *the ship's logical forward is 180° from its
thrust-forward.*

**`HEADING_SIGN` and `HEADING_OFFSET` must match across every computer on a hull** —
`uacconf.lua`, `apconf.lua`, `gpwsconf.lua`, `atcconf.lua` (dynamic), `responder.lua`. If one is
recalibrated, change them all. The UAC would still fly the craft to the right heading (it owns
those constants), but the FMS's alignment gate would open at the wrong moment, and *when the
AWACS and the responder bolted to the same hull disagree about which way it is pointing, one of
them is lying to the fleet.*

### Direction letters

`landing.db`'s `n/e/s/w` resolve to world unit vectors, `x` means "don't care" → `nil` → no
constraint, and a **number** is a decimal heading in degrees:

```lua
DIR_VECTORS = { n = {0,-1}, e = {1,0}, s = {0,1}, w = {-1,0} }
-- numeric: { sin(rad), -cos(rad) }
```

### Handedness

Body-right is `(-uz, ux)` for a track `(ux, uz)`. That is the convention **cross-track** is
written in, so a positive lateral demand means "go right" and a positive `xtk` means "right of
course". Everything — the FMS's localizer, the UAC's `rightErr`, the bank cross-track term —
uses that same handedness. Check it before adding a term.

### Every `*_SIGN` is a calibration guess until flown

`PITCH_SIGN`, `ROLL_SIGN`, `YAW_SIGN`, `*_RATE_SIGN`, `THRUST_SIGN`, `STRAFE_SIGN`, `XTK_SIGN`,
`BANK_SIGN`, `BANK_XTK_SIGN`, `HDG_FROM_BANK_SIGN`, `KJOY_*_SIGN`, `FW_PITCH_SIGN`,
`TRI_*_SIGN`, `HZN_*_SIGN`, `MANNAV_YAW_SIGN`… **flip, do not scale.**

> **A control loop that "blows up rather than settling" is a SIGN/FRAME error, not a gain.
> Suspect that first.**

`CFG.FW_PITCH_SIGN` and `CFG.KJOY_PITCH_SIGN` describe the same physical thing ("+cmd = nose
up"), so the former should **track** the in-flight-calibrated latter rather than be re-guessed.

---

## 2. The altitude datum (invariant I5)

**Altitude is the world Y of the BOTTOM OF THE HULL, everywhere.**

`sublevel` reports the assembly's pose, which sits some way *up inside* the aeroplane. Every
computer on a hull subtracts `ALT_OFFSET` **the moment it reads the pose**, and nothing downstream
of that subtraction ever sees the raw number.

It has to be that datum because **every altitude the aircraft is compared against is the world Y
of a piece of ground**: `landing.db`'s pad and runway elevations are surveyed surfaces, and so is
the terrain server's profile.

| | Togekiss | Mantine | Citation | Argo | Scout | airbus | demounit |
|---|---|---|---|---|---|---|---|
| `ALT_OFFSET` | 9 | 5 | *unmeasured* | 8 | 4 | *unmeasured* | 0 (no hull) |

Six files per hull (see [10-aircraft.md](10-aircraft.md) §1). It is a **required** knob in all
four conf files (boot canary) **with no default anywhere**:

> **0 is a legal value** (the display bench), so a defaulted 0 would silently restore the old
> over-reading altimetry on a real airframe and look exactly like a working aeroplane — right up
> to the flare. Uncorrected, Aer Togekiss flew its descent to a pad elevation 9 m underground and
> was stopped only by the squat switch.

**To measure:** set it to 0, park on ground of a known Y, and take
`(what it read) − (that Y)`.

---

## 3. Cobalt's 200-local limit

CC's Lua VM caps a function at **200 locals**, *and counts the main chunk's live locals against
every nested function.* `autopilot.lua` and `uac.lua` are already near that ceiling — hence
`autopilot.pre200fix.bak`.

**The rule: add knobs and state as `CFG.NAME` / `TABLE.name`, never as a bare top-level
`local`.** That is why this tree hoists things into tables that look like they should be
separate variables:

| table | in | holds |
|---|---|---|
| `CFG` | everywhere | every knob |
| `NAV`, `LND`, `TAVOID`, `ATC`, `PLAN`, `GA`, `APR`, `RTR`, `TERR`, `CRS`, `TFC`, `NDU`, `TD`, `HSICFG` | `autopilot.lua` | the whole feature, functions included |
| `SLOW`, `TRI`, `LOG`, `AH`, `ui`, `kj`, `fw`, `bnk` | `uac.lua` | slow-polled physics, the allocator, the recorder, the horizon, published state, three PI/filter states |
| `AI` | `altcom.lua` | the entire PFD |
| `DB`, `LINK`, `ST`, `DATUM`, `TERRAIN`, `GPWS`, `AUDIO`, `FB`, `HUD` | `gpws.lua` | one per subsystem |

Config loaders are wrapped in a `do ... end` block so the load temporaries do not occupy
top-level registers — **net cost of a loader is one local, `CFG`.**

**`luac -p` does not model this.** It will pass a file that Cobalt refuses. And Lua 5.4 accepting
a file proves neither runtime semantics nor the local limit — the target VM is Cobalt, which is
5.1-flavoured.

The failure mode is that a *late, large* function (`drawLandingHSI`, `controlLoop`) blows up once
the file has too many top-level locals — so adding one `local` at the top can break a function
600 lines away.

---

## 4. The config split (invariant I2)

**Configuration is split from program wherever a program has more than one deployed copy.**

| program | conf | deployed to |
|---|---|---|
| `pid/uac.lua` | `pid/uacconf.lua` | #1, #19, #27, #38, #72 |
| `autopilot.lua` | `apconf.lua` | #8, #16, #28, #39, #74, #46 |
| `gpws.lua` | `gpwsconf.lua` | #67, #70, #71 |
| `atc.lua` | `atcconf.lua` | #44, #64 |
| `mancon.lua` | `manconconf.lua` | #6, #76 |
| `terrain_server.lua` | `terrainconf.lua` | #68 |
| `vecengine.lua` | `vecconf.lua` | #78, #84, #85 (dormant) |

Rules:

1. **The conf is the single source of truth.** The program holds **no fallback copy**,
   deliberately: a duplicated default would let an edit in the conf silently do nothing.
2. **A missing knob is a named boot error** — that is what the canary lists are for. Add every
   knob that is read *unconditionally every tick* to the canary. **Booleans especially**: a
   missing boolean does not crash, it reads nil, evaluates false, and the feature quietly does
   nothing, which is worse.
3. **Never overwrite a `*conf.lua` when updating a program.** It is that airframe's
   empirically-flown calibration, and **the comments beside each value are the record of why a
   sign is what it is.**
4. **Relays and panels hardcode their protocol name and faces at the top** — they have no conf.
   That line is what to edit when cloning a computer onto another airframe.

### The `vecconf.lua` cautionary tale

Before `vecconf.lua` existed, the three vector-engine computers held their wiring inline.
Regenerating the sibling copies from **one** computer's, to keep the three programs in step,
silently overwrote their wiring tables with the front-centre layout. **The aircraft rolled itself
over**, and one computer's original wiring was **unrecoverable and had to be re-derived by hand.**

`78/vecconf.lua` and `80/vecconf.lua` carry the note. The same reasoning applies to every conf
file in the table above, and `_goldmaster/` deliberately holds **no** `uacconf.lua` for exactly
this reason:

> *A copy of one airframe's conf sitting in a folder called "gold master" is an invitation to
> regenerate another airframe's from it. **There is no gold master for a conf file. There cannot
> be one.***

---

## 5. The pseudo-bitmap (2×3 subpixel drawing)

Every graphical instrument in this tree draws at **2×3 subpixel resolution inside a character
cell**, not at cell resolution. CC's font carries 32 "teletext" glyphs at codepoints 128–159 that
split a cell into a 2-wide by 3-tall grid: character `128+n` lights the subpixels whose bits are
set in `n` in the **foreground** colour, over a background of all the rest.

```
+-----+
| 1 2 |   bits 1 and 2      mask  = sum of the bits you want in the foreground
| 3 4 |   bits 4 and 8      glyph = string.char(128 + mask)
| 5 6 |   bits 16 and 32
+-----+
```

**Three rules, and all three are traps:**

1. **Only 31 masks are drawable.** The bottom-right subpixel is always background, so codes
   128–159 cover only masks 0–31. A mask with bit 32 set is drawn as its **complement**
   (`63 - mask`) with the two colours **swapped**. That is what the `SW` table records everywhere
   it appears — its consumers must hand `blit` its colours the other way round.
2. **128 and 160 are unrelated glyphs**, so the empty and the full mask are a plain **space**
   (with the swap flag distinguishing them), never `char(128)` or `char(160)`.
3. **A cell still holds only TWO colours.** Six subpixels must quantise to a foreground and a
   background, and *which* two is a design decision that has to be made explicitly and written
   down. The terrain map picks the **most severe** band present and the least severe, so a hazard
   subpixel is never the one quantised away; the PFD picks the overlay's colour and whichever of
   sky/ground holds the majority of the cell.

### `blit`'s colour digits are ordered BY BIT

So **`"0"` is white and black is `"f"`** — not the other way round.

Each user **computes** them rather than relying on `colours.toBlit`, so the mapping does not
depend on the installed CC version:

```lua
local function digit(c) local n,i = c,0 while n>1 do n=n/2 i=i+1 end
    return ("0123456789abcdef"):sub(i+1,i+1) end
```

> The first cut of the HSI's terrain overlay defaulted an unpainted cell to `"0"` and **painted
> the sky white**. `TERR.blk` exists so that number is written down exactly once.

### Where it is used

| File | Tables | Draws |
|---|---|---|
| `autopilot.lua` | `TERR.SX`/`TERR.SW`, `TERR.PC`/`TERR.PS` | the HSI's terrain overlay: `SX`/`SW` address the glyphs as a **bitmap** (a boundary between two bands), `PC`/`PS` as a **tone** (six spread pixels shading a cell by where it sits inside one band) |
| `autopilot.lua` | `TERR.SX`/`TERR.SW` again, from `CRS` | the HSI's **PLAN view**: the baked path as a bitmap, in subpixels, over a hazard layer sampled per **whole cell**. That split is what makes a cell's two colours reach — six terrain samples *and* a path in one cell is three colours, and there are two |
| `altcom.lua` | `SX`/`SW` | the PFD's artificial horizon, pitch ladder, bank scale, flight path marker, and the ticks and bug of the speed, altitude and VS tapes |

`gpws.lua`'s HUD and `uac.lua`'s artificial horizon are **cell**-resolution, not subpixel — so
there is no complement/swap business in either.

### Two consequences, already solved — copy the solution rather than rediscovering it

* **Cost.** A subpixel picture is six times the samples of a cell picture. The FMS rebuilds its
  terrain raster only at `CFG.TERR.REDRAW` and re-blits the cached rows at 20 Hz; the PFD, whose
  horizon moves every frame, instead builds each row from **runs** (`string.rep` of a uniform
  stretch, individual cells only where the boundary crosses and where an overlay patches) and
  skips blitting rows identical to the previous frame. Neither loops over width × height of cells
  per frame, deliberately.
* **Line rasterising.** Use `ceil`, **not `floor`**, on a DDA's step count: `floor` makes the step
  marginally longer than a subpixel and every so often skips one, which reads as a horizon with
  holes punched along it.

### Cost measured, not guessed

A raster build goes from 0.65 ms to 3.1 ms on desktop Lua with `SUBPIXEL` on — call it five times
the work, not six, because the six subpixels of a character usually land in the same grid cell and
it is classified once (`TERR.at`'s memo). **Cobalt is an order of magnitude slower again.**

Two levers if an instrument feels heavy, **in order**: raise its `REDRAW` (costs nothing but how
fast the picture recolours during a climb); or set `SUBPIXEL = false`, which keeps the picture
correct and makes it blockier.

The HSI's PLAN view has the same two knobs (`CFG.HSI.PLAN.REDRAW` / `.SUBPIXEL`) and a third,
`MAX_POINTS`, which is the one that actually bounds it: the path is a 5 m-sampled polyline, so a
30 km route is 6 000 points and only a ceiling stops the rebuild scaling with route length.
Skipping samples costs **chord error `c²/8R`**, which at the ceiling is about one subpixel on the
tightest corner the planner will build — so the ceiling is close to free and the accuracy argument
for raising it is weak.

---

## 6. Palettes are a luminance budget, not a list of favourite colours

All sixteen slots on the HSI are spoken for. Every instrument line is drawn **over** the
heightmap, so the two cannot share a brightness range: the tint is confined to the dark third
(luminance 0.03–0.19) and every instrument mark clears its ceiling by at least 1.5×.

* **White is deliberately not a terrain colour**, though technically free. Spending it would buy a
  fifth tint step and cost the legibility of every cardinal, the aircraft symbol and every number.
* **Water used to share `blue` with the course needle's away tip.** Measured, the two were a
  1.00 : 1 contrast ratio — *the same colour* — so the away tip vanished over the sea.
* **Saturation does the rest of the work:** the two hazard colours are the only saturated ones, so
  amber reads as a warning and khaki reads as a hillside even where they are close in hue.
* **The dither's resolution is bounded by the DATABASE, not by the glyphs.** Eleven distinct tones
  measured over the real window, not the nineteen four steps blended six ways would give: the
  vertical resolution is 8 m per level, so a 73 m window holds only about nine distinct ground
  heights to colour. *Widening `DITHER` past that buys nothing; the way to more relief is a
  narrower window, not a finer blend.*

Every palette repaint is in a `pcall` — `setPaletteColour` is advanced-only and a monitor that
refuses must not be fatal.

---

## 7. Versioning and file discipline

**Versioning WAS by filename; this tree has git now.** Both conventions coexist:

| pattern | meaning |
|---|---|
| `foo.vX.Y.lua` | a frozen release snapshot. **Never edited again** |
| `foo.<reason>.bak` | the copy taken immediately before a risky change (`.pre200fix`, `.prefwpitch`, `.precurvereplace`, `.prevtolmodes`, `.prealtdatum`, `.preopt`, …) |
| `foo.lua` | the live file, and the only one the computer runs |

* **Take a `.bak` before a change you may want to walk back.**
* **Do not "tidy" the old snapshots away.** The `.bak` name is the change's own record.
* `_goldmaster/uac.lua` is a snapshot in the same sense — never edited, never run, never copied
  onto a computer.

**`startup.lua` runs exactly one program** (`shell.run` / `shell.execute`). It is how you tell
what a computer is for. A retired computer's `startup.lua` **prints a notice** rather than being
deleted (#4, #13, #14) — the program stays for A/B testing. The one exception is Scout's #71,
which opens `engineswitch.lua` in a tab beside `gpws.lua`.

**Every computer carries a marker file** `_<aircraft>_<role>.lua` (zero or one byte). It is not
code — it exists so the computer identifies itself when you `ls` it:

```sh
grep -rl "mantine_" --include='*.lua' .    # every computer on Mantine
ls */_*.lua                                 # the marker files, one per computer
```

Keep the convention when adding a computer. (Scout's `84`/`85` still carry stale
`_scout_vector_*` markers from before the revert.)

---

## 8. Deployment order

**RELAYS FIRST, UAC LAST.** Every relay accepts both the current float message and the legacy
split-and-whole one, so the aircraft keeps flying in between. The other way round feeds `nil` to
`setAnalogOutput` on whichever side is not firing and **kills the relay outright**.

For a shared program (`uac.lua`, `autopilot.lua`, `gpws.lua`, `atc.lua`, `mancon.lua`): the
program file is **byte-identical** across its computers. Fix a bug once and copy it to every
computer that runs it. **Never copy the conf.**

**To publish a change:** mirror the computer's file into the matching **role** folder under
`~/Documents/create-aeronautics-flight-system/` and commit there. That repo is the release; this
tree is the workshop.

**But the computers here are ahead of the package** — they are the development copies and carry
features the release does not. **When package and computer disagree, the computer wins.**

**Last full sync: 2026-11-18** (package branch `sync-2026-11-18`). Every role was mirrored from
Aer Togekiss's computer where it has one (UAC #1 with its conf, FMS #8 with its conf, the relays
#3/#9/#10/#11/#12/#13/#14/#15, MANCON #6, GPWS #67, responder #45, radar #44 + #64's conf as the
AWACS example, ATC controller #87, terrain server #68, nav data #24 with its `.db` files), from
Scout's #86/#78 for the fore/aft and vector-engine controllers, and `_docs/` itself became the
package's `docs/`. Package-only edits, so a later sync does not undo them: `YAW_RELAY_ENABLED =
true` in the rotation relay, the `scout_` → `aertogekiss_` protocol lines in `thrustcon.lua`,
`vecengine.lua` and `signcal.lua`, and `terrainbuild`'s `DEF_WORLD` (relative to the jar instead
of the author's server path). **#72's `uac.lua` was not shipped** — it is a superset of #1's
(`THRUST_TO_PITCH_FF`, `STRAFE_TO_ROLL_FF`, the yaw log columns) and the two forks should be
merged before the next sync.

---

## 9. Working on this machine

* **Nothing here runs outside Minecraft.** `sublevel`, `quaternion`, `vector`, `matrix`, `pid`,
  `rednet`, `redstone` and `peripheral` are all in-game globals. **There is no test harness — the
  only real verification is flying it.**
* Only **Lua 5.4** is installed (`lua`, `luac`). `luac -p <file>` is a syntax check and nothing
  more; the target VM is **Cobalt** (5.1-flavoured), so 5.4 accepting a file proves neither
  runtime semantics nor the 200-local limit.
* `autopilot.lua` (~6.1k lines) and `uac.lua` (~3k) are large. **Make targeted edits; do not
  rewrite or reformat wholesale** — the comments carry flight-test history that cannot be
  re-derived from the code.
* **When in-game numbers are required** (e.g. a flight test), do not ask the user to read values
  off a screen: write a small snippet that samples the wanted number down to disk, and read from
  that. See [12-diagnostics-and-tools.md](12-diagnostics-and-tools.md).
* Required mods: **CC: Tweaked**, **Create: Aeronautics**, **CC-Sable** (`sublevel` — pose,
  velocity, inertia tensor), **Advanced-Math** (`quaternion`, `vector`, `matrix`, `pid`).

---

## 10. The trap list

Collected, in rough order of how often each has cost someone an afternoon.

1. **`msg.y` on `<family>_navap` is world Z, not altitude.** Same on the GPWS's cached target.
2. **`blit`'s digits are ordered by bit: `"0"` is white, black is `"f"`.**
3. **Adding a top-level `local` can break a function 600 lines away** (Cobalt's 200-local cap,
   charged against nested functions).
4. **A missing boolean in a conf does not crash** — it reads nil, evaluates false, and the feature
   silently does nothing. Canary it.
5. **`ALT_OFFSET` must never be defaulted**, because 0 is legal.
6. **Two broadcasters on one protocol alternate at 20 Hz.** Never add a second; OR it in at the
   existing broadcaster.
7. **`cancel` ≠ `release`** on the autoland channel. `release` restores nothing.
8. **`carrot` ≠ `glide`** on the navap channel. The rollout needs one without the other.
9. **A relay's timeout is a deadman, not a saving.** Do not suppress unchanged broadcasts.
10. **The lift channel's `up`/`down` are not a push-pull pair**, and `cut` is not `up = down = 0`.
    A legacy `up = 7` read as a fraction pins the lift line at 15.
11. **Scout's `84`/`85` face and tap assignments are correct as written.** So is `85`'s `alti()`
    using different taps on its two branches.
12. **Never regenerate one conf file from another computer's.**
13. **`luac -p` passing means nothing** beyond syntax.
14. **`peripheral.find("monitor")` will answer with a monitor on the wrong face.** Name the face
    where the other faces are spoken for (`mancon.lua` does; and it capability-checks the result
    so a wrong block does not take the whole program down at boot).
15. **`q * vector` in Advanced-Math normalises its intermediate product** and silently discards
    the magnitude. Use the hand-written `rotateVec` — it is copied byte-for-byte into `uac.lua`
    and `altcom.lua` for this reason.
16. **`sleep(0.05)` cannot return before the next tick boundary**, so an already-late loop asks for
    and gets a whole extra tick. Use deadline pacing.
17. **A rate command must be scaled by tilt; an open throttle must not.**
18. **A commanded zero is a stop, not a trim point** — clear the integral, and do not apply a
    breakaway floor.
19. **`floor`, not `ceil`, on a DDA step count punches holes in a horizon line.**
20. **`os.clock()` is the computer's uptime, not wall time.** Cross-computer logs align on
    `os.epoch("utc")`.
21. **The runway glidepath is zero AGL at the AIM POINT**, so the threshold crossing height is
    `aim · tan(gs)` and nothing else. Move the aim point or change the slope and you have
    changed how high the aircraft crosses the threshold — and if that drops below
    `RWY_FLARE_AGL`, the flare begins *before* the strip. `RWY_TCH` is the input now
    (`LND.rwyGlide`); the aim point and the slope are what give way to it.
22. **A conf table copied into the program SHADOWS the conf file, silently.** `autopilot.lua`
    carried a second `local HSICFG = {...}` for as long as `CFG.HSI` had existed in
    `apconf.lua`, so every edit to `CFG.HSI` did nothing at all — which is how `RANGES` came to
    differ between airframes' `apconf.lua` with no effect on any of them. Found 2026-09-16 and
    deleted. When moving constants out to a conf file, **delete the original and grep for the
    name**; and canary the knobs, because a shadow gives no error anywhere.
23. **Zoom hysteresis measured against the step you are moving TO locks out a multi-step
    zoom-in.** The PLAN view's autozoom wanted 880 m off a 5 000 m ring, landed on the 1 000 m
    step, was 12 % inside it, and a 25 % deadband read that way refused the whole move — so the
    map never left 5 000 and looked exactly like an autozoom that does not work. The deadband
    belongs on the **boundary being crossed**, i.e. the step immediately *below the one you are
    on* (`CRS.snap`).
24. **A stopping distance is `v²/2a`, not `v²/a²`.** Both the runway speed sizing and the
    missed-approach test carried the second form until 2026-09-08 and were an order of magnitude
    optimistic. The autobrake in the same function always had it right — when two expressions in
    one file disagree about a formula, check the units before choosing a side.
25. **An FMS's `TERR.CALLSIGN` that differs from its responder's `CALLSIGN` makes the aircraft
    an obstacle to itself.** The ATC controller excludes the asker's own report from its
    facility by callsign. With a mismatch, the aircraft's own responder entering the footprint on
    the way down (below `AGL_MAX`, 20 m) reads as somebody else on the pad, the controller orders
    a hold, and the FMS — not yet committed above the flare — aborts its own landing. The
    own-ship filters on the PLAN view and the AWACS miss it too.
26. **A facility left out of `sites.db` is uncontrolled even if it is the other end of a
    controlled runway.** TMV and TMA are one strip; with only TMA in a site, an aircraft asking
    for TMV is told "uncontrolled" and may be flown head-on into a clearance for TMA. #87's board
    flags the overlap; list both ends.
27. **Tearing down a plan that holds the vertical channel is a message, not a flag.** The UAC
    latches `landed`/`descend`/`hold` until it hears `cancel`/`release`/`off`; clearing `LND.active`
    in the FMS tells it nothing. Every plan commit did exactly that until 2026-10-15, so a plan
    committed from the LANDED screen was flown along the ground with the lift cut. Commits now go
    through `DEP.reset()`/`DEP.commit()` (02 §7b), which send the `cancel`; a new commit path that
    skips them reopens it.
28. **`a % (2*math.pi)` turns a sweep of nothing into a full circle.** A turn that should be zero
    comes back from rounding as `-1e-16`, and the modulo of that is `2*pi` — a 360° loop laid into a
    path where a straight was meant. Found on the TMV departure (its climb-out runs straight at
    TAIMW along x = 609). Arc sweeps go through `GA.sweep`, which snaps the last micro-radian to 0.
29. **Streaming `landed` cuts the lift, wherever the aircraft is.** `AL_GROUND_CUT` zeroes both lift
    lines on `landed`, and nothing in the UAC checks that the aircraft is actually down. Anything
    that sends it every tick must re-check every tick — the departure gives up the moment it is
    `GROUND_AGL` off the strip, because an engaged ALT-AP or a scramble can lift it without asking.
