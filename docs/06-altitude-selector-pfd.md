# 06 — The altitude selector and the PFD (`altcom.lua`)

*#7 Aer Togekiss · #17 Citation · #29 SS Argo · #41 Mantine · #75 Scout · #47 display bench.
Three jobs on one computer: the four-panel ALT-AP selector (the pilot's altitude input), the
AP-ALT switch announcer, and the primary flight display — artificial horizon, speed tape,
altitude tape and VS scale — on a monitor.*

> `altcom.lua` has **no conf file**. Protocol names, `ALT_OFFSET` and the display signs are
> hardcoded at the top and differ per computer. This is one of the two programs (with
> `responder.lua`) that carry `ALT_OFFSET` inline rather than in a conf.

---

## 1. The selector

Four panels across the screen, each a numeric keypad plus a toggle button:

```
┌─ input window ─┐   type digits ──▶ Ent commits ──▶ Clr clears
│  1  2  3       │
│  4  5  6       │   the toggle button under each panel shows the panel's altitude
│  7  8  9       │   and lights RED when it is the selected one, GREEN when it is not
│ Ent 0 Clr      │
└────────────────┘
```

Defaults `160 / 240 / 320 / 400`. At boot **nothing is selected**, so `altSelection` is nil and
the computer broadcasts nothing — the UAC then holds whatever altitude it booted at.

The selected panel's altitude goes out on `<family>_altap` as `{alti = <value>}` **once a
second**, and only while a panel is selected. The UAC clamps it to `AP_ALT_MIN`/`AP_ALT_MAX` and
ignores it entirely while an autoland owns the vertical.

The AP-ALT switch is a **redstone input on `front`**, re-announced on `<family>_apswitch_2` at
10 Hz. The FMS does the same for AP-NAV on `_1`. There is no single AP panel — the two toggles
are physical switches wired to the two brains.

---

## 2. The three modes — the FMS override state machine

The selector **learns what the FMS is doing** by listening to the `<family>_autoland` stream the
FMS already sends the UAC. It does not need a channel of its own to be told.

| mode | meaning | panels | broadcasting? |
|---|---|---|---|
| `NORMAL` | ordinary ALT-AP | the pilot's pick is lit | yes |
| `OVER` | an autoland (or a terrain hold) owns the vertical; the UAC is ignoring `altap` | **all green** | **no** |
| `PILOT` | the pilot tapped a panel *during* a landing — ALT-AP has the vertical back | the pilot's pick is lit | yes |

Transitions:

```
 hold | descend | landed ──▶ OVER      (stashes the current pick, clears the live one)
 release                 ──▶ PILOT     (keeps the pilot's pick)
 cancel                  ──▶ NORMAL    (restores the stash — unless we were PILOT)
 off                     ──▶ all deselected, silent
 3 s of silence in OVER  ──▶ NORMAL    (safety net: missed cancel, or the FMS rebooted)
```

Plus a **suppress window** after an override tap, so the FMS's last few in-flight `descend`
messages do not yank the panel straight back to OVER before the FMS has processed the override.

**When the pilot taps a panel while `OVER`**, the selector sends a ping on
`<family>_altoverride` — the FMS's `overrideLoop` decides what that means from *its* context
(pause the descent, drop the go-around's vertical, release a terrain hold, or finish a landing).
See [01-protocols.md](01-protocols.md) §altoverride.

> **This is why `cancel` and `release` are not interchangeable.** `toPilot()` (from `release`)
> assumes the tap that triggered it has already re-selected something, so it restores nothing.
> `toNormal(false)` (from `cancel`) restores the stash. A plain teardown that sends `release`
> silently drops the pilot's ALT-AP selection on the floor.

---

## 3. The PFD (artificial horizon and the three tapes)

Drawn on an attached monitor at 20 Hz, at **2×3 subpixel resolution** — see
[11-conventions-and-traps.md](11-conventions-and-traps.md) §"The pseudo-bitmap" for the glyph
mechanics and the traps.

### What is on it

| element | notes |
|---|---|
| **horizon** | full width — the one line that must never look clipped. It passes *behind* the tapes |
| **pitch ladder** | linear in degrees, as a real ADI's is. Confined to the attitude window, so rungs do not run out through the bank scale, and so the picture has an edge that says which part is being read as attitude |
| **bank scale** | an arc of degree ticks with a pointer |
| **flight path marker** | a small ring, offset by the actual drift; hidden below `HZN_FPV_GS` (5 m/s), where it is meaningless. Caged to the **attitude picture**, not to the monitor — a marker parked under a tape is one the pilot cannot see |
| **speed tape** | ground speed, scrolling, down the left edge (5 cells) |
| **altitude tape** | altitude, scrolling, down the right (4 cells), carrying the ALT-AP bug |
| **VS scale** | vertical speed, a fixed scale with a moving needle, at the right edge (3 cells) |
| **chrome** | the boresight — and, *only where the monitor is too narrow for tapes*, the old corner readouts |

### The one number that sets the scale

`HZN_FOV = 80` — the vertical field of view, top to bottom, in degrees. A rung sits
`3·H / HZN_FOV` subpixel rows per degree from the horizon.

*(The row-at-a-time display this replaced projected through a pinhole, `f·tan(pitch)` with
`f = H/1.6` rows. The two agree to within a rung out to about 25° and only part company where
`tan` starts to run away.)*

### The ladder picks its own step

`HZN_LADDER = {20, 10, 5}` — the **finest** of these whose rungs still land `HZN_SEP` (3)
subpixels apart, and **no ladder at all** if even the coarsest does not.

> A horizon and a bank scale with no ladder is a poorer instrument; one with a ladder whose
> rungs have quantised on top of each other is a **worse** one — the smear reads as a second
> horizon.

Every second rung of whichever step wins is a labelled major, so the figures stay 10° or 20°
apart either way.

The bank scale's fine ticks likewise carry a minimum arc radius (`BT`'s third column): 10° of an
arc under ~18 subpixels is less than three subpixels of arc, and the fine ticks close up into a
band.

### How it is drawn fast enough

The horizon moves every frame, so caching a raster is useless. Instead each row is built from
**runs**:

1. Solve the horizon half-plane `g(i,j) = nx·i + ny·j + B < 0` for the row's top and bottom
   subrows to find **which cells can straddle the boundary** — then widen by one cell each way so
   no rounding can lose it.
2. Everything left of that band is one run; everything right of it is another; only the band
   itself is built cell by cell.
3. The runs `string.rep` a **mask**, not a colour — with the horizon exactly level (`nx == 0`)
   every cell of the row is identical, and that identical cell is not necessarily all-sky or
   all-ground; it can be a partial mask.
4. Overlays (ladder, bank scale, FPV, text) are collected per row in `AI.pr[r]` and **patched**
   into the finished strings, so an overlay costs only the columns it touches. A column drawn on
   twice is one patch.
5. The assembled `chars\1fg\1bg` line is compared with the previous frame's and **skipped
   entirely if unchanged**. Straight and level, only the rows the horizon crosses change.

Every table (`SC/SF/SB`, `OC/OF/OB`, `ink/inkc/tch/tfg/tbg/pr/mark`) is **reused**, not
reallocated per frame, and lives on the single `AI` table rather than as top-level locals — the
main chunk's local count is charged against every nested function and Cobalt caps that at 200.

### The tapes

Added 2026-09-10. A tape is the pitch ladder's idea again: **a scale that scrolls past a
readout box that never moves**, so the figure is read in one fixed place and the rate is read
off how fast the scale behind it is going by. The box *is* the pointer, which is why the
scale behind it may quantise as coarsely as the monitor forces without the number ever being
wrong — nothing interpolates a value into a character row.

**Speed is GROUND speed**, and a lightGrey `GS` legend in the spare half of its readout box
says so. There is no air in this game and so no airspeed: the FMS's speed law, the responder's
contact reports and this tape are all the horizontal component of `sublevel`'s velocity.

### The knobs

Three, hardcoded at the top of `altcom.lua` beside `ALT_OFFSET` (this program has no conf
file), and **each is the only knob its scale has**:

| | | value |
|---|---|---|
| `SPD_STEP` | m/s between figures on the speed tape | **10**, fleet-wide |
| `ALT_STEP` | m between figures on the altitude tape | **10**, fleet-wide |
| `TAPE_GAP` | blank rows between figures, both tapes | **3**, fleet-wide |
| `VS_FULL` | m/s at the ends of the VS scale | 8 Togekiss · 12 Citation · 6 Argo · 6 Mantine · 8 bench · 6 Scout |

> **The step and the gap are the knobs, and the SCALE follows from them** — figures are placed
> `TAPE_GAP + 1` cells apart by construction, so
>
> ```
> k = 3 · (TAPE_GAP + 1) / step        subpixel rows per unit
> tape covers H · step / (TAPE_GAP + 1) units, top to bottom
> ```
>
> A taller monitor therefore buys *more scale at the same readability*, not the same scale in
> steps too fine to read; and **widening the gap spends scale to buy air around the figures** —
> at step 10 on a 26-row monitor, a one-row gap shows 130 units of the world and a three-row
> gap 65. Minor ticks sit halfway between the figures whatever the gap, so they follow it and
> need no knob of their own.
>
> It was the other way round until 2026-09-10: `SPD_SPAN`/`ALT_SPAN` were the knobs and
> `AI.tapeStep` derived the step. That made the granularity a property of the **monitor** —
> one 160 m/s span figured every 20 on a 26-row display and every 50 on a 14-row one — which
> is backwards, because how finely a figure can be read is a property of the reader. The
> per-airframe `SPD_SPAN` went with it: ten units between figures reads the same on the
> Citation as on Scout, and that Scout's tape then scrolls ten times faster at cruise is the
> tape telling the truth about the difference.

* `VS_FULL` is the airframe's own vertical envelope with a little over: `uacconf`'s
  `CFG.CLIMB_MAX`, or `CFG.MANCON_CLIMB_MAX` where the boost key can reach past it. The VS
  scale keeps a derived step (`AI.tapeStep`) because it is a *fixed* scale — its ends are the
  knob, so its ticks have to fit whatever height is left.
* The speed tape is floored at zero (`vmin`); the altitude tape is not, because a world Y
  below zero is just the underside of the map.
* **The VS figure carries no sign.** It sits above the scale in a climb and below it in a
  descent and it is drawn in the needle's colour, so a leading `+`/`−` would be the third
  time the display said the same thing — and it would cost one of the three columns the
  digits need.

**The ALT-AP selection appears twice**, which is the one thing the old corner readouts could
not show — how far away the selected altitude is and whether the aeroplane is closing on it:
as a **figure at the head of the altitude tape** (magenta, `LAND` instead while `OVER`), and
as a **magenta bug on the scale**, parked a row in from either end when it is off scale. The
bug lives in the tape's **tick column**, not among the figures: a character beats ink in
`AI.cell`, so a bug drawn in a figure column would vanish behind the very figure it is
pointing at.

**The tapes are opaque and drawn last.** Two consequences, both load-bearing:

* The attitude picture is re-centred on what is *left* of the monitor. `AI.fc0`/`AI.fc1` are
  its column range and **`AI.hw` is its half-width in subpixels** — the ladder, the bank scale
  radius, the boresight and the attitude window all read `AI.hw` where they used to read
  `AI.W`.
* `AI.plot` and `AI.text` **drop anything landing in a tape column** unless `AI.ontape` is
  set. That is not tidiness: the horizon runs the full width and the flight path marker's
  wings reach six subpixels either side, so without it they would show *through* the tape.

**Below a monitor 32 columns wide or 10 rows tall there are no tapes at all**, and the corner
readouts (`GS`, `ALT`, `VS`, `SEL`) come back in `AI.chrome`. Twelve columns of tape (5 speed,
4 altitude, 3 VS) on a narrow display leaves an attitude picture not worth looking at.

**Cost.** A tape *body* is a run exactly as the sky and the ground are — `AI.layout` builds
the two of them as strings once per layout, `AI.row` concatenates them, and the columns behind
a tape are never masked or classified at all. That is worth a third of a frame: filling those
cells one at a time instead (the obvious way, and the first way it was written) cost 0.85 ms
at 121×61 against 0.61 for a picture pixel for pixel identical. The figures are in the file's
own header.

### Signs

Three switches, same pattern as every sign in this project — flip if the display moves the wrong
way in-game:

| | |
|---|---|
| `HZN_PITCH_SIGN = -1` | nose-up is a positive pitch |
| `HZN_ROLL_SIGN = 1` | a right bank tilts the horizon the way it does out of the window, and sends the bank pointer right |
| `HZN_DRIFT_SIGN = 1` | the flight path marker slides toward the wing it is drifting over |

`rotateVec` is copied **byte-for-byte** from `uac.lua`: the library's `q * vector` operator
normalises its intermediate product and silently discards the magnitude.

---

## 4. `ALT_OFFSET` here specifically

```lua
local ALT_OFFSET = 9   -- measured 2026-08-29: on the Y=63 airstrip it read 73
```

**The number on this screen is what the pilot flies the aeroplane on, and the UAC holds the
ALT-AP pick made from it** — so the two must be the same datum or the selected altitude is not
the one flown. It must match this airframe's `pid/uacconf.lua`, `apconf.lua`, `gpwsconf.lua`,
`atcconf.lua` (dynamic mode) and `responder.lua`.

To measure: set it to 0, park on ground of a known Y, read the PFD, take
`(what it read) − (the Y the hull rests on)`. It is a property of the **build**.

---

## 5. The display bench (#47)

`demounit` is **not a vehicle**. #47 runs `altcom.lua` purely to demonstrate the artificial
horizon; nothing listens to what it broadcasts, there is no UAC on the other end of
`demounit_altap`, and `ALT_OFFSET = 0` because there is no hull. Keep it running whatever the
current instrument code is.

---

## 6. Quirks

* **`rednet.open('back')` is hardcoded** in `altcom.lua`, not `peripheral.find("modem")` — the
  older idiom, kept here (and documented in the release package's MANUAL).
* `msg.alti` goes out as whatever `currentAlti` holds, which starts as a **string** from the
  defaults table. Every receiver does `tonumber`.
* The selector redraws only the cells it changes (`updateGUI`), not the whole panel — but
  `repaintButtons()` on a mode change repaints all four.
* The palette is repainted at boot (`green`, `brown`, `orange`, `cyan`, `blue`, `red` on the
  terminal; `blue`/`brown`/`orange`/`lime`/`grey` on the monitor — `grey` is the tapes' own
  ground — in a `pcall` because a non-colour monitor must keep its own).
* Boot selects nothing on purpose. Do not "fix" that: a selector that armed itself at power-up
  would command a climb the pilot never asked for.
