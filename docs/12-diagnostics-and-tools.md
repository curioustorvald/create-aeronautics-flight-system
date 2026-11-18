# 12 — Diagnostics and tools

*How to get a number out of the game. **Do not ask the pilot to read values off a 20 Hz screen** —
write the number to disk and read the file. Every tool here does exactly that, and every one of
them is receive-only: none commands a thruster.*

---

## 1. The rule

> When in-game numbers are required (a flight test, a sign check, a timing measurement), write a
> small snippet that **samples the wanted number down to disk**, and read from that.

Reasons, all of them learned the hard way:

* The question is almost always about **several numbers at the same instant** — "does the kick
  appear first in the COMMAND, in the LIFT, or only in the ENGINE OUTPUTS?" can only be answered
  by columns that share a timestamp.
* A 20 Hz screen shows the same numbers but **not simultaneously and not at a rate anyone can
  transcribe**.
* Reading twelve numbers off a terminal by eye is **how the loop-rate 6× got mistaken for
  jitter** in the first place.

Run diagnostics in a **tab** beside the live program:

```
bg pid/scrambletrace.lua          -- or:
shell.openTab("pid/uacprof.lua")
```

Cross-computer logs align on **`os.epoch("utc")`** (world time, shared) — never on `os.clock()`,
which is each computer's own uptime.

---

## 2. The flight data recorder (`CFG.LOG_*` in `uacconf.lua`)

Built into `uac.lua`. One CSV row per control tick to `CFG.LOG_FILE`.

```lua
CFG.LOG_ENABLE  = false      -- off on every airframe but whichever is being debugged
CFG.LOG_FILE    = "/uaclog.csv"
CFG.LOG_DELAY   = 4.0        -- s after boot before the file is opened
CFG.LOG_SECONDS = 60.0       -- ...and how long it records before closing
CFG.LOG_EVERY   = 1          -- every Nth tick
```

* **`LOG_DELAY`** exists so the first moments — peripherals attaching, the first nav/altitude
  packets arriving, the trim integral starting from zero — do not fill the record with a transient
  nobody asked about.
* **`LOG_SECONDS` is a hard stop.** An unbounded log on a computer with a fixed disk quota
  eventually fails a write mid-flight and takes the controller with it.
* The row is written **last** in the tick, after every command is final and `ui` is published, so
  it is a coherent snapshot rather than a mix of this tick and the last.
* Enabling it demands the whole block (conditional canary).

### Columns

```
t,pitch,roll,wx,wz,errP,errR,rotRaw,pStep,rStep,yStep,integP,integR,
posY,velY,desVelY,vertEff,altSteps,tiltUp,thrCmd,strCmd,
L,sat,cone,hot,fcx,fcy,fct,blx,bly,blt,brx,bry,brt,idle,apN,apA,phase,
scram,velInt,navCap,<arrival block>
```

**Columns are APPENDED, never inserted**, so every earlier `uaclog.csv` still parses with the
same column indices.

The ones that answer specific questions:

| column | question it settles |
|---|---|
| `scram` | `ui.scramble` **as the controller saw it**, not as the stick sent it. The MANCON's own panel can show SCRM while this reads 0 — then the fault is *between* the two (a protocol name, a stale packet, a gate). If it reads 1 and the aeroplane does not climb, the flag arrived and the fault is **downstream** |
| `velInt` | the vertical PID's integral — **the hover trim the loop has learnt**, in alt steps. The detent commands `ALT_SWING`, so what it *adds* over the hover is `ALT_SWING − velInt`. A hull sitting at `velInt = 7` out of a swing of 8 is already at 14 of its 15 line levels just to stay up, and **no software can make the detent worth more than one step** |
| `navCap` | the FMS's **scheduled** speed as received: `cruiseCap` when `schedSpeed` is set, else **`-1`** for "the FMS is not scheduling". `-1` is deliberately distinguishable from a scheduled zero |

Reading `navCap` beside `thrCmd` and `phase` settles an align-phase argument without anyone
watching the HSI:

| `navCap` | `thrCmd` | verdict |
|---|---|---|
| 0 | 0 | the hold is being honoured |
| 0 | **nonzero** | something downstream of the schedule is still spending authority — that was the breakaway floor, and before it the ENROUTE speed integral. A third would be a new fault, so log **which phase** it appears in |
| **−1** in ENROUTE | — | the FMS is not scheduling at all (`DC.DYNAMIC_CRUISE` off, or no cap reaching this controller). **Fix that before reading on** |

---

## 3. `pid/uacprof.lua` — where the tick actually goes (#1, #72)

**Answers "how many GAME TICKS does X cost", not "how many milliseconds"** — because ticks are
the only unit the scheduler works in.

Each phase runs the **same** `sleep(0.05)` loop with one more kind of work in it, so the
difference between two phases is what that work is worth. Every phase's work is written to be as
close as possible to what `controlLoop` really does with that call **including what it does with
the result** (a `sink` accumulator keeps the values live).

Phases: `BARE`, `MATH`, `SUB1..SUB5` (pose / velocity / angular velocity / inertia / mass),
`RADIO1..RADIO4`, `AH_OLD`, `AH_NEW`.

### How to read the result

| observation | conclusion |
|---|---|
| `BARE ≈ 0.05` | the scheduler is fine; every later phase is our own doing |
| `BARE ≫ 0.05` | a server-wide budget. **Nothing in `uac.lua` can reach it — go and read #18** |
| `SUB5 − SUB3` large | caching the tensor and the mass is the win, as designed |
| `RADIO4 − RADIO1` large | the four broadcasts are the wall, and the next move is a single bus message the relays each pick a field out of |
| `MATH ≈ BARE` | arithmetic is free; **stop reading the control law** |
| `AH_OLD ≫ AH_NEW` | the artificial horizon was the compute cost, and the run-based redraw already fixed it |

The radio phases broadcast on `"uacprof_null"`, which **no computer on the fleet subscribes to**.
Safe to run on a parked, powered aircraft. Results go to `/uacprof.csv` as well as the screen.

**Run this before changing anything about loop performance.**

---

## 4. `18/tickprobe.lua` — is it us or the server?

A whole computer that runs nothing else, so what it measures is the **server's own scheduling**.
See [08-ground-stations.md](08-ground-stations.md) §5.

Three phases (`BARE` / `RADIO` / `SCREEN`) and **both clocks** — `os.clock()` (the computer's
uptime, what the UAC times itself with) and `os.epoch("utc")` (real wall time). *If they disagree,
the slowdown is an artefact of the clock rather than of the scheduler, and that changes the answer
entirely.*

**Zero jitter is the tell.** A merely busy program runs late by a *varying* amount; a fixed ratio
is what a **budget** looks like (`max_main_global_time`, 10 ms per tick shared by every computer
on the server).

---

## 5. `pid/scrambletrace.lua` — the four-way sniffer (#1, #72)

"The scramble does not work" has **four** places it can break and no single screen that shows all
four at once. This prints them side by side and writes `/scrambletrace.csv`.

| column | what a wrong one tells you |
|---|---|
| `stick_scram` | the flag **as the MANCON broadcast it**. FALSE while holding the key = the input never happened: wrong key code in `manconconf`'s `CFG.KEYS` (X is 88, V is 86, C is 67 — `get_keycodes.lua` prints what a key really sends), no typewriter found, or the lever on the wrong `CFG.VS.SIDE` |
| `stick_vs` | the trim rate beside it. **It freezes during a scramble by design** — an unchanging value is correct, not a stuck stick |
| `alt_up` / `alt_dn` | what the UAC put on the ALT protocol, as line **fractions** (0..1). During a scramble this must be `1.000/0.000` — or `0.000/1.000` inverted — **at every attitude**. Anything between is the scramble not reaching the output |
| `cut` | the lift engine commanded off. **A scramble outranks the ground cut**, so this must be false while scrambling |
| `mst` / `alt_sw` | the engine switches the relay gates on. A perfect `alt_up/dn` with `mst = false` is a **switch**, not code |
| `posy` / `vy` | altitude (already datum-corrected by the sender) and climb rate — so the file also answers *"did the aeroplane actually go up"* |

Protocol names come from `uacconf`, so the file is **airframe-independent**: copy it to any UAC's
`pid/` directory and it traces that aircraft.

Hold the scramble **key and lever one at a time** and read the file back.

---

## 6. `pid/probe.lua` — the API and frame probe

Run this **first** on any new airframe or after a mod update. It pins down three things that
cannot be read off the mod's docs with certainty:

1. **The quaternion namespace** — which constructor exists (`fromAxisAngle` / `fromEuler` /
   `new`), its argument order, and which methods an orientation quaternion exposes.
2. **The mat3×3 shape** `sublevel.getInertiaTensor()` returns — nested table, flat 9, or an
   object — so `unpackTensor` can read it.
3. **The FRAME of `sublevel.getAngularVelocity()`** — world axes or body axes. Hand-rotate the
   craft and watch which components move. This sets `CFG.SENSOR_FRAME`.

Nothing here commands a thruster.

---

## 7. `pid/signcal.lua` — ground sign calibration (#72)

Written for Scout's revert, when every sign in `uacconf.lua` had been measured against the
**nozzle** allocator and said nothing about which way a fresh thruster bank fires.

> Guessing them in the air is how you get divergence: the levelling loop answers a tilt with more
> tilt, so the craft wanders off one way and then breaks up. **This measures them on the ground
> instead.**

Three passes, **engine master OFF for the first two** so no thruster answers:

1. **PITCH** — tilt nose-up and hold. Write down which line the relay lights (its screen prints
   `P<n> <side>` per frame). Nose-up must command the bank that pushes the **nose back down**;
   if it lights the one that pushes further nose-up, flip `CFG.PITCH_SIGN`. Repeat nose-down and
   check it **mirrors cleanly** — *if it does not mirror, the axis is tangled, not inverted, and
   the fix is `SWAP_PITCH_ROLL_CHANNELS`.*
2. **ROLL** — rolled right, the **low** wing's bank must spool up. Flip `CFG.ROLL_SIGN` if the
   high wing's does.
3. **YAW / translation** — with the master on.

Receive-only on the UAC's own output protocols, plus read-only `sublevel`.

---

## 8. `facemon.lua` — "are two computers driving the same redstone line?" (#84, #85, #86)

`peripheral.wrap('left')` resolves to whatever block is on **that computer's** left, so two
computers sitting near one relay can be driving the **same** redstone output. If they are, the
last program to write each tick wins — **and from the air that is indistinguishable from a
control-law cross-coupling.**

**The test is passive.** No axis-wiggling, nobody watching a screen: *if two computers are
wrapping the same relay, then whatever one writes the other reads back, sample for sample.*

So: run it on each suspect computer, fly (or just let the UAC hold attitude on the ground) for
half a minute, and compare the logs offline. **Faces that are physically the same carry identical
values at identical timestamps; faces that are independent do not.**

It samples all six faces of the computer itself plus all six of the `left` and `right` wraps,
every 0.1 s for 150 s, into `/facelog.csv`:

```
# computer 84  label nil
epoch,own.top,...,own.back,left.top,...,left.back,right.top,...,right.back
```

Timestamps are `os.epoch("utc")` — world time, shared by every computer — **so the logs align
without any clock sync.** Read-only; writes no redstone.

*(Its header still names the pre-revert computers #77/#79/#80; the technique is unchanged.)*

---

## 9. `get_keycodes.lua` (#6, #21, #76)

Prints the GLFW code a key actually sends. Use it before writing anything into `CFG.KEYS` or
#21's key tables — **simultaneous presses write that many characters**, which is also why
`pressedSet()` accepts both a list and a set.

---

## 10. The FMS route tracer — where a plan build is stuck

*Added 2026-09-17 for "the FMS reliably hangs while calculating a path from TMA > TMV > Dest",
and it found it. **Not running now:** the fix is in and all six FMS copies are byte-identical
again, so the tracer is kept unrun as `74/autopilot.rtrace.lua`. To re-arm it:*

```sh
cp 74/autopilot.rtrace.lua 74/autopilot.lua     # then reboot #74
cp 74/autopilot.prereversal.bak 74/autopilot.lua   # ...and back to the fleet build
```

*The snapshot carries the reversal fix and the display fixes as well, so re-arming it costs
nothing but the trace itself. Copy it onto another FMS only after checking that computer's own
`autopilot.lua` is the same build (`md5sum */autopilot.lua`) — it is a whole program, not a
patch.*

It writes one CSV row per **step of the routing chain** to `/rtrace.csv`, and it exists because
the failure it was built for is a **hang**: nothing can be read off the screen afterwards, the
computer has to be killed, and anything left in a buffer dies with it.

Two properties are the whole design, and both matter if it is ever copied:

* **Every row is flushed as it is written** (`RT.FLUSH`).
* **Every row that precedes a loop is written before that loop is entered**, carrying the
  numbers the loop was entered with — notably `MARCH/STRAIGHT`'s sample count.

So **the last line in the file is the answer.** It names the stage, the waypoint and the
numbers.

```lua
local RT = {                      -- at the top of 74/autopilot.lua, NOT in apconf.lua
    ENABLE   = true,              -- the whole instrument, one switch
    FILE     = "/rtrace.csv",
    MAX_ROWS = 60000,             -- hard stop (the quota here is 100 MB; this is about
                                  -- readability, not space)
    FLUSH    = true,
    VERBOSE  = true,              -- per-reference / per-settle / per-shrink-step rows too
    HUGE_N   = 100000,            -- a march or spiral asking for more steps than this gets
                                  -- its own WARN row
}
```

It is **deliberately not in `apconf.lua`**: it is an instrument, not a feature, so no other
airframe's config has to gain a knob for it and an FMS's `apconf.lua` can still be overwritten
from another aircraft's without taking the trace with it. The cost of that choice is that
switching it on is a whole-file swap rather than one boolean — which is the right way round for
something that should not be running most of the time. (If it ever wants to be permanently
available, the move is `CFG.RTRACE` in all six `apconf.lua` files plus a conditional canary, the
way `CFG.LOG_ENABLE` works for the UAC's flight recorder in §2.)

### Armed windows

Rows are written only between `RT.arm()` and `RT.finish()`, which bracket the six entry points
that build a path: `setDirectTo`, `setRoute`, `RTR.setSolvedRoute`, the route **preview**,
`GA.build` and `GA.buildWp`. Two reasons — `uiLoop` redraws the preview on *every* event
(including `navLoop`'s sleep ticks), so an always-on trace would write the same Dijkstra twenty
times a second; and a file whose every row belongs to one plan is readable. The preview arms
**once per destination picked**, which is what a pilot does anyway. Nested arms are ignored, so
one commit is one bracketed block.

### The columns

```
row,t,tag,stage,event,key,i,j,a,b,c,d,note
```

`stage` is one of `EXPAND APPR GRAPH JOIN DIJKSTRA ROUTE PREP RWYPREP RESUME PLAN CORNER GEOM
MARCH SPIRAL SPEED GA SESSION`; `a`–`d` are four numbers whose meaning is per (stage, event) and
is written out beside each call site. **The four that matter:**

| row | read | means |
|---|---|---|
| `PLAN/TURN` | `a` = the turn in **degrees** | one per interior waypoint, before anything is solved. **180 is a reversal** |
| `CORNER/ENTER` | `a` = degrees, `note` = `maxT=` and `tan=` | `T = k + (R+p)·tan(Δ/2)`, so `tan` is the multiplier the tangent length is about to get |
| `CORNER/EXIT` | `d` = `T` against `c` = `maxT` | the tangent length the corner returns, and the most that fits on the shorter leg |
| `MARCH/STRAIGHT` | `b` = **samples about to be appended** | `for i = 1, n` with a `table.insert` in it. A `b` in the billions **is** the hang |

`SESSION/END` carries the totals: `spiralEnd` calls, its sub-steps, and samples marched.

### What it found on the first flight (2026-09-17)

In the aircraft, Scout #74, FLY ROUTE → FGY from the TMA/TMV strip. CC's watchdog terminated the
FMS ("Too long without yielding") at `autopilot.lua:1388`, the `table.insert`; these are the last
rows of `/rtrace.csv`:

```
JOIN,NEAREST,TMV,,,144.698,603.793,742.604,,nearest graph node to the current fix
ROUTE,EXIT,FGY,,15,20988.3,,,,TMV TAIMW TMA UKRAN UHAKF ... FGY
PLAN,TURN,TAIMW,2,15,180,-1,733,438                      <- exactly 180 degrees
CORNER,ENTER,TAIMW,2,,180,193.162,733,438,maxT=197.1 tan=1.63312e+16
CORNER,EXIT,TAIMW,2,80,49.5461,155.654,197.1,1.11406e+18,arc=0 fits=false
GEOM,WP,TAIMW,2,15,609,1331,0,1.11406e+18,rounded
MARCH,STRAIGHT,,,,1.11406e+18,2.22813e+17,609,-1.11406e+18
MARCH,WARN,huge-straight,...,n samples: this march will not finish
```

Three things the trace settled that could not have been assumed:

* **the preview is innocent** — `SESSION/END PREVIEW FGY … solved`. The preview solves the route
  only; the plan is built at commit, which is why only FLY hangs;
* **the approach is innocent** — `PREP/CASE FGY_APPR1 … case 1, nothing spliced`. The reversal is
  the router's own path;
* **the aircraft was never going to fly that leg** — `RESUME/PICK TMA, 3`. The heading-aware
  resume had already chosen to join at TMA, skipping TAIMW. `PLAN.build` bakes the whole point
  list regardless of the join index, so it died on a corner nobody would have flown.

The same case under a stub harness beforehand, aircraft at TMA, FLY ROUTE → AGC — the plan
reported as `TMA>TMV>AGC`:

```
ROUTE,EXIT,AGC,,10,13661.6,,,,TMA TAIMW TMV MDARO NDARF MIATU MTFRA VOLGA DNFRU AGC
PLAN,TURN,TAIMW,2,13,180,-1,438,733                      <- exactly 180 degrees
CORNER,ENTER,TAIMW,2,,180,193.162,438,733,maxT=197.1 tan=1.63312e+16
MARCH,STRAIGHT,,,,1.11406e+18,2.22813e+17,609,-1.11406e+18
MARCH,WARN,huge-straight,...,n samples: this march will not finish
```

TMA (609,893), TAIMW (609,1331) and TMV (609,598) **share an x**, so the two bearings are
exactly 0 and π and `wrapPi` returns exactly −π. `tan(π/2)` is 1.63e16 in doubles — not an
infinity, not an error, just a number large enough that the straight run into the transition is
1.1e18 m long and `PLAN.marchStraightInto` is asked for 2.2e17 samples. See
[02-fms-autopilot.md](02-fms-autopilot.md) §4 for the mechanism and §4a for what was done about
it — a reversal is now collapsed out of the assembled point list before the planner sees it, and
`PLAN/COLLAPSE` (with `PLAN/APPRGONE` when it ate a whole published pattern) is the row that
says so.

---

## 11. What each program shows on its own screen

You do not always need a tool.

| program | its screen carries |
|---|---|
| **UAC** | artificial horizon, attitude errors, integrals, phase, link age, commanded vs achieved speed (`CMD`/`CLO`/`GS`), target altitude, alt line levels, AP flags, the MODE button, and **`ui.dtAvg` — the measured loop period**, on screen permanently |
| **FMS** | HOME: position, plan, nav-data status. STATUS: the waypoint list with PASSED/remaining, the SPD selector, or the landing HUD (position/heading error, AGL, commanded VS, phase, squat state) |
| **relays** | the received float, the levels produced, the face names, `active:`, and the engine-switch row |
| **MANCON** | the translation box, yaw strip, VS column, the **keys-down echo** (`WER`/`SDF` and `^ v o !`), and whether the typewriter answered at all |
| **GPWS** | the HUD on the monitor; **self-status, the armed/inhibited line and `TERRAIN.why()` on the computer's own terminal** |
| **#24** | the CLIENTS roster — **how you confirm an edit actually landed**, and which client is holding or faulting |
| **#68** | tiles loaded, clients seen, the activity roster |
| **ATC** | the scope, the traffic strips, and a header with the nav-data and fix status |

### The `CMD` / `CLO` / `GS` triangle on the UAC

* `CMD` high but the craft stays fast, `CLO` high and `THR` not going negative → the PID or the
  thrusters cannot brake.
* `CLO` low but `GS` high → the speed is **tangential**; the loop is happy and the craft is still
  fast. That is the overshoot/orbit signature.

---

## 12. Interpreting a diagnosis: two patterns worth memorising

**A distribution with no fractional part is a blocking loop, not a slow one.** Lua running long
gives a *continuous* spread of periods. Exact multiples of one game tick mean main-thread round
trips. See [00-architecture.md](00-architecture.md) §6.

**A longer oscillation period after fixing an inner loop is the signature of an OUTER-loop
fault.** Damping the inner loop makes it track the guidance demand faithfully, which *exposes* an
outer-loop problem that sloppy tracking had been masking. The carrot-lookahead weave got slower —
2.8 s → 3.3 s — when the UAC's inner loops were fixed, and that is what pointed at the guidance
gain rather than at attitude. See [02-fms-autopilot.md](02-fms-autopilot.md) §4.

---

## 13. Offline checks

```sh
luac -p <file>                    # SYNTAX ONLY. Says nothing about Cobalt or the 200-local cap
grep -rl "mantine_" --include='*.lua' .    # every computer on one aircraft
ls */_*.lua                                 # the marker files, one per computer
md5sum */autopilot.lua                      # confirm a shared program really is byte-identical
diff _goldmaster/uac.lua 19/pid/uac.lua     # what a pre-fork airframe has diverged by
```

There is **no test harness** and there cannot be a useful one: `sublevel`, `quaternion`,
`vector`, `matrix`, `pid`, `rednet`, `redstone` and `peripheral` are all in-game globals. Past
work has used **stub harnesses** for specific questions (the fork was verified tick-by-tick that
way, and the dither refactor was bench-diffed over 60 k fuzzed commands), but they are written for
one question and thrown away.

**The only real verification is flying it.**
