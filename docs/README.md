# `docs/` — the flight system, documented

> **About these documents.** They are the author's development reference, shipped as they are
> written for the development fleet. Read them with three things in mind:
>
> * **Computer numbers (`#1`, `#8`, `#67`…) and aircraft names refer to the author's world.**
>   [10-aircraft.md](10-aircraft.md) maps them; in this package each *role* is a folder instead
>   (`#1` the UAC → `attitude-controller/`, `#8` the FMS → `flight-management-system/`, and so
>   on — the README's computer table names every role).
> * Mentions of `CLAUDE.md`, `CLAUDE.TODO.md` and `_goldmaster/` refer to the development tree's
>   own notes, which are not part of this package.
> * For installing, wiring and flying, start with [`../MANUAL.md`](../MANUAL.md). These pages are
>   for when you need to know *why* something behaves as it does, or what a field on the wire
>   means.

Reference documentation for the *Create: Aeronautics* + CC: Tweaked avionics.
Written so that a task can be started **without re-reading 6 000 lines of `autopilot.lua` to
work out what talks to what**.

Verified against the development tree on **2026-09-05** and extended since. Where a doc and the
code disagree, **the code is the authority** — but say so, because a stale doc is worse than none.

---

## Read this first

**[00-architecture.md](00-architecture.md)** — the whole system on one page: the two brains, the
signal chain, the eight invariants everything else depends on, the authority table (who owns
which axis, when), and the loop-rate fact every gain is calibrated against.

Almost every other file assumes it.

---

## The files

| | |
|---|---|
| **[00-architecture.md](00-architecture.md)** | System overview, invariants I1–I8, authority table, what the system deliberately does not do |
| **[01-protocols.md](01-protocols.md)** | Every rednet protocol: sender, receivers, message shape, rate, staleness behaviour. **The wire reference** |
| **[02-fms-autopilot.md](02-fms-autopilot.md)** | `autopilot.lua`: path planner, speed law, phases, pad + runway landings, go-arounds, published approaches, terrain avoidance, the ATC client, auto-router, UI, HSI |
| **[03-uac-attitude.md](03-uac-attitude.md)** | `pid/uac.lua`: quaternion attitude, the four translation laws, the vertical cascade, tilt allocation, bank/modes, FIXEDWING vertical-by-pitch, the vector allocator, the loop-rate fork |
| **[04-relays-and-panels.md](04-relays-and-panels.md)** | Float → redstone: the sigma-delta, the engine-switch gates, the deadman, every face assignment, Scout's hand-written relays |
| **[05-mancon-manual-flight.md](05-mancon-manual-flight.md)** | Hand-flying: `mancon.lua`, the four-computer stack, the VS lever, the MIC, the keyboard joystick, and the scramble |
| **[06-altitude-selector-pfd.md](06-altitude-selector-pfd.md)** | `altcom.lua`: the ALT-AP selector, its NORMAL/OVER/PILOT override machine, and the PFD |
| **[07-gpws-hud.md](07-gpws-hud.md)** | `gpws.lua`: the HUD layout, the height datum, every alert mode and its inhibits, the voice pack |
| **[08-ground-stations.md](08-ground-stations.md)** | #24 nav data, #44/#64 radar, the responders, #68 terrain, #18 tick probe, the display bench, **#87 the automated ATC controller** |
| **[09-navigation-database.md](09-navigation-database.md)** | `points.db` / `course.db` / `landing.db` / `sites.db`: syntax, the four landing record shapes, pattern references, ATC sites, and how to add an aerodrome |
| **[10-aircraft.md](10-aircraft.md)** | Per-airframe computer maps, calibration figures, per-aircraft quirks, and the known drift from `CLAUDE.md` |
| **[11-conventions-and-traps.md](11-conventions-and-traps.md)** | Frames and signs, the altitude datum, Cobalt's 200-local cap, the config-split rules, subpixel drawing, versioning, deployment order, **and the trap list** |
| **[12-diagnostics-and-tools.md](12-diagnostics-and-tools.md)** | How to get a number out of the game: the flight recorder, `uacprof`, `tickprobe`, `scrambletrace`, `probe`, `signcal`, `facemon`, the FMS route tracer |

---

## Router: "I want to…"

| Task | Read | Then |
|---|---|---|
| **understand the system at all** | 00 | — |
| **add or change a rednet field** | 01 §9 | the sender's and every receiver's file |
| **debug "nothing is moving"** | 04 §6 | 01 §quick index for the staleness behaviour |
| **change how a route is flown** (corners, speed, carrot) | 02 §4–§6 | `apconf.lua`'s `CFG.DC` |
| **change a landing** (pad, runway, flare, brake) | 02 §7 | `apconf.lua`'s `CFG.LG` |
| **change a take-off** (line-up, backtrack, roll, rotation, the turn off the runway) | 02 §7b | `apconf.lua`'s `CFG.DEP` |
| **change a go-around or an approach** | 02 §7 + 09 §3–§4 | `landing.db`, then `CFG.LG.RWY_GA_*` / `APPR_*` |
| **change terrain avoidance** | 02 §8 | `apconf.lua`'s `CFG.TAVOID` header |
| **retune attitude** | 03 §5 + §14 | that airframe's `pid/uacconf.lua`, **and run `probe.lua` first** |
| **retune translation or the vertical** | 03 §6–§7 | `uacconf.lua`'s translation / altitude sections |
| **fix a sign** | 11 §1 + 03 §14 | `signcal.lua` (12 §7) |
| **understand a bank/turn problem** | 03 §9 | especially `BANK_HDG_KD_WASHOUT` |
| **work on hand-flying or the scramble** | 05 | `manconconf.lua`; `scrambletrace.lua` (12 §5) |
| **work on the ALT-AP panel** | 06 §2 | note `cancel` ≠ `release` (01 §2) |
| **work on the GPWS** | 07 | `gpwsconf.lua` |
| **work on an instrument's drawing** | 11 §5–§6 | the existing solution in `altcom.lua` / `autopilot.lua`'s `TERR` |
| **add an aerodrome, route or approach** | 09 §8 | edit on **#24**, not on an aircraft |
| **add a computer or an airframe** | 10 §10 | 11 §4 (never regenerate a conf) |
| **make the loop faster** | 00 §6 + 03 §12 | `uacprof.lua` **before** changing anything (12 §3) |
| **find out why an aircraft did something** | 12 §2 (the flight recorder) | 12 §12 for the two patterns |
| **debug a hang while a plan is being built** | 12 §10 (the route tracer, #74) | 02 §4 for the corner solver |
| **work on the automated ATC** (sites, clearances, holds, diverts) | 08 §7 + 02 §8a | 01 §8a for the wire; `sites.db` is 09 §9; `atcctlconf.lua`, `apconf.lua`'s `CFG.ATC` |
| **know which computer is which** | 10 | `ls */_*.lua` |

---

## Ten things that are true everywhere

The short version of [11-conventions-and-traps.md](11-conventions-and-traps.md); the full trap
list is §10 there.

1. **`msg.y` on `<family>_navap` is world Z, not altitude.**
2. **Altitude is the world Y of the bottom of the hull.** Every computer subtracts `ALT_OFFSET`
   the moment it reads the pose, and **0 is a legal value**, so it must never be defaulted.
3. **One protocol, one broadcaster.** Two senders alternate at 20 Hz; OR the second source in at
   the existing broadcaster instead.
4. **The UAC states a demand as a float; the relay decides how to spend it.** The authority
   limits stay in the UAC.
5. **Message rate is dither rate, and silence is a deadman.** Never suppress unchanged
   broadcasts.
6. **No fallback defaults in any program.** A missing knob is a named boot error — including
   booleans, which would otherwise fail silently.
7. **Never regenerate one `*conf.lua` from another computer's copy.** It is flown calibration and
   the comments are the record of why.
8. **Add knobs as `CFG.NAME`, never as a bare top-level `local`** — Cobalt caps a function at 200
   locals and charges the main chunk's against every nested function.
9. **`blit`'s colour digits are ordered by bit: `"0"` is white, black is `"f"`.**
10. **A loop that "blows up rather than settling" is a sign/frame error, not a gain.**

---

## Keeping these docs honest

* **The code is the authority.** `apconf.lua`, `uacconf.lua` and their siblings carry the *why*
  beside each value, and those comments are flight-test history that cannot be re-derived. These
  docs summarise and connect them; they do not replace them.
* **When you change behaviour, update the doc in the same commit** — the file the router points
  at for that task.
* **When you find a doc wrong, fix it and say so**, rather than working around it.
* Known drift between these docs, `CLAUDE.md` and the tree is recorded in
  [10-aircraft.md](10-aircraft.md) §9 rather than silently reconciled, because the discrepancy
  itself is sometimes the information.
