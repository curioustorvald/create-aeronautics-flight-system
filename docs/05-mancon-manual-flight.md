# 05 — MANCON and hand-flying

*Everything the pilot flies with when the autopilots are not flying: the vertical stick, the
six horizontal axes, the yaw, the scramble detent, and the FIXEDWING keyboard joystick.*

---

## 1. The two shapes of MANCON

There are two hand-flying architectures in this tree, both live.

### (a) The original four-computer stack

Still what **SS Argo** flies, and what Mantine flies:

```
 vertical-speed lever ──▶ VSC (vspdpid.lua)  ──<f>_mancon──────────────▶ UAC
                                #26 / #40

 six buttons ─┬─▶ #34/#52 manconrelay_translation ──<f>_mancon_translational──┐
              └─▶ #35/#48 manconrelay_rotation    ──<f>_mancon_rotational─────┤
                                                                              ▼
                                                        MIC (mic.lua) #36 / #42
                                                                              │
                                                            ──<f>_mancon_nav──▶ UAC
```

The two button relays broadcast raw key state at 20 Hz **only while manual control is live**
(master on, AP-NAV off) and go silent otherwise. The MIC therefore reads a missing packet as
"keys released": an axis whose relay has not been heard within `STALE_AFTER` (0.3 s) fades to 0.

### (b) The one-computer `mancon.lua`

**Scout #76** and **Aer Togekiss #6**. One computer doing all four jobs, driven off a
**typewriter** peripheral. The file is **byte-identical** on both computers; everything that
differs (protocols, key codes, faces, limits, rates) is in `manconconf.lua` beside it.

Aer Togekiss was collapsed onto this on 2026-08-28, porting Scout's file. #13 and #14 are
retired (`startup.lua` prints a notice; nothing is deleted); **#15 is not**, because its
touchdown detection and extra-lift bank are load-bearing — only its yaw relay is gated off.

The forcing constraint is invariant I1: `aertogekiss_mancon_nav` can have exactly one
broadcaster.

---

## 2. `mancon.lua` — the one-computer stick

### Keys (GLFW codes, sampled with `get_keycodes.lua` beside the program)

```
    W (87) strafe left    E (69) fore      R (82) strafe right
    S (83) yaw left       D (68) aft       F (70) yaw right

    SPACE (32) climb   Z (90) descend   A (65) boost   V (86) scramble
```

The horizontal six are the two rows of the WER/SDF cluster, laid out **the way the aircraft
moves**. The vertical four sit off that cluster.

Opposing keys held together cancel.

### One stick feel, everywhere (2026-09-08)

**Every axis is spring-loaded**: hold to ramp toward that axis's ceiling, let go and the command
falls back to zero — brake to a hover and hold heading for the horizontal six, stop climbing and
hover for the vertical. All seven run through the **same** `integrate()`, differing only in their
limits.

It used not to be. The vertical two were **trim-style** — release *held*, because that is what
the analog lever they replaced did — and the two feels had to be learnt separately. That is the
real cost the change removed; one rule now covers the whole keyboard.

**The centre key went with the trim.** `C` existed because a trim that holds has to be given a
way back to level that is not "wind it back by hand"; a spring centres itself the moment the
pilot lets go, so there is nothing left for `C` to do and it is **retired** rather than kept as a
key that duplicates letting go. It is gone from `manconconf.lua` too, not left behind as a dead
knob (#21 keeps space/A/C — it flies a *pitch stick*, which still holds and still needs
centring).

What made `C` worth having survives, because it was never really about `C`: the vertical comes
back to zero **on `VS.FADE`** (~0.3 s from full travel) rather than snapping there. A snap is a
step input, and shaping a yanked vertical command into a ramp is exactly what `vspdpid.lua`'s
slew existed for ("so the cargo stays settled"). Holding both vertical keys reaches the same
branch, which is what `C` used to reach by hand.

### The boost (`A`)

A **modifier**, not a command: held with anything else it raises the **ceilings** —
`NAV.BOOST_FWD_MAX`, `NAV.BOOST_STRAFE_MAX`, `VS.BOOST_MAX_CMD` — and does nothing on its own.
There is no new packet field and nothing downstream learns about it.

* **The ramp rates are deliberately not boosted with the ceilings**, so the boost is extra
  *runway* rather than a step: pressing it at full deflection accelerates on from there. It also
  means releasing the boost while still holding the key walks back down at `RAMP` (the slow rate)
  rather than `FADE` — a gentler deceleration than letting go altogether, which is the right way
  round.
* **Yaw is not boosted and deliberately has no knob.** `YAW_MAX` is already above the UAC's own
  `MAX_RATE_YAW`, so a boosted yaw target would be clamped away without reaching a thruster.
* **The vertical boost is the one that needed the UAC's consent.** That end clamps the wanted rate
  before its rate loop sees it, and that clamp is the *handling* envelope ALT-AP and the autoland
  fly inside too — so the manual branch now clamps to `CFG.MANCON_CLIMB_MAX`/`MANCON_DESCEND_MAX`
  instead ([03](03-uac-attitude.md) §7). **The two files must agree**: a `BOOST_MAX_CMD` above the
  UAC's manual clamp is thrown away up there, and one above `CLIMB_AT_FULL` is thrown away by the
  aeroplane. Today: Togekiss 3.75 → 7.5 (of 10.0 available), Scout 3.75 → 5.0 (of 5.0 — its
  handling limit already spends three quarters of the authority, so 1.33× is all there is).
* **Every scale on the panel is the boosted one**, held or not. A gauge whose scale changes under
  the pilot's eyes is the classic instrument sin; drawn against the boosted ceiling the unboosted
  stick simply reaches half travel. `BST` on the status line and `+` in the key echo say it is
  held — `BST` ranks *below* the fault and lockout tags, because a boosted stick that is being
  ignored is a lockout to report.

### The integrator

```lua
integrate(cur, pos, neg, maxVal, rampRate, fadeRate, dt)
```

Holding one direction drives the goal to `±maxVal` at the **slow** ramp rate; anything else —
released, or both opposing keys — drives it to 0 at the **fast** fade rate. Lifted from
`mic.lua` unchanged. So a tap gives a small nudge and a long hold builds to the max. Since
2026-09-08 the vertical is one more call to it (`VS.RATE`/`VS.FADE`); the `trim()` it used to
have was deleted rather than kept beside it, because two ways to move one accumulator is how the
two ends drift apart.

Residues below 1e-3 (1e-4 for yaw) are snapped to **exactly 0**, because the UAC's
null-the-drift / hold-this-heading path is an exact no-op only at 0.

### The scramble: three sources, ORed here

```lua
local lever  = redstone.getInput(VS.SIDE)
local remote = IN_SCRAMBLE and remoteScram and fresh(remoteAt)
local scram  = lever or held(set, K_SCRAMBLE) or remote
```

1. the typewriter key (`V`). **`V`, not `X`** — X was dropped on 2026-09-08 because it
   collides with an in-game shortcut, and a key that also does something to the world is
   the wrong key for the one command that exists for when everything has gone wrong.
   `CFG.KEYS.SCRAMBLE` is still a **list**, because this is the command that deserves a
   second binding; V is the survivor because it is the code the file has actually carried
   and flown all along (see the note in `manconconf.lua` — the documented X once did
   nothing at all, which was the whole of "the scramble does not work").
2. a **digital redstone lever** on `VS.SIDE` (`front`) — a second, keyboard-independent path,
   because all nine keys come off one peripheral and a dead typewriter would otherwise cost the
   pilot the one command that exists for when everything else has gone wrong
3. an optional **remote request** on `CFG.IN_SCRAMBLE` — Scout's #78 auto pull-up on
   `scout_pullup`. `false` on Aer Togekiss: nothing there can call a scramble remotely, and that
   is a real answer, canaried as one.

> **The remote one joins here and nowhere else, and that is the whole point.** An automatic
> pull-up must be **indistinguishable from the pilot's own detent** to everything downstream —
> the UAC, the GPWS, the relays — so it is folded into `scram` *before* the broadcast and the
> packet on `<f>_mancon` is byte-for-byte the shape it has always had. Nothing downstream learns
> a new field, gets a second sender, or can tell the two apart. The pull-up is invisible until
> it engages, and when it does it **is** a scramble — inheriting the override over ALT-AP and
> the autoland, the trim freeze, and the staleness gate for free.

**A scramble no longer freezes the vertical, and the reason the freeze existed went with the
trim.** `cmdVS` was a *parked setting* then, so a scramble had to preserve it or the pilot would
come out of the pull-up with the trim overwritten. It is a spring now, and a spring's honest
reading of "the vertical keys are not being obeyed right now" is that the command decays to zero
— so coming out of a scramble with no key held leaves the aircraft in a hover rather than
resuming a climb, or worse a descent, nobody is asking for.

Running the integrator unconditionally also removes the special case: **the UAC ignores `vs`
outright for as long as `scramble` is set** (`ui.scramble` wins the vertical cascade before any
source is read), so what this computer puts in that field during a scramble cannot reach a
thruster either way.

### The lockout

```lua
locked = (LOCK_ON_APNAV and apnav) or (LOCK_ON_MASTER and mstHeard and not mst)
```

Locks the **horizontal six only**. The vertical is deliberately not gated: AP-NAV does not own
the vertical channel (ALT-AP does, and that is the UAC's own gate), so the stick stays live under
AP-NAV and answers the moment ALT-AP drops. The boost key is not gated either and does not need
to be — a locked-out horizontal reads every key as released, so its accumulators fade to zero
whatever ceiling they are nominally allowed.

`mstHeard` matters: a MANCON that has never heard the engine switch must not lock itself out.

### Why both packets go out every tick

The UAC gates both channels on **freshness** (`MANCON_STALE_AFTER` / `MANNAV_STALE_AFTER`,
0.5 s). Silence is read as "no stick attached" and the craft is handed back to the autopilot's
hold, so a crashed or unplugged MANCON cannot leave a stale velocity latched in.

It also means the horizontal packet is broadcast as **zeros** while the pilot is locked out
rather than going silent as the Togekiss relays do — so the UAC starts nulling drift on the first
tick after AP-NAV drops instead of waiting for a first packet.

### Peripherals

* The **keyboard is found by capability**, not by type or a hardcoded side: anything with
  `getPressedKeyCodes()`. A missing typewriter is **not fatal** — this computer owns two
  independent channels and the vertical is the more important, so losing the keyboard costs
  fore/aft/strafe/yaw and nothing else.
* `pressedSet()` accepts **either** shape the peripheral might return (a list of codes, or a set),
  so a mod update to either cannot silently give a permanently-centred stick. The second return
  says whether the peripheral answered at all.
* The **monitor face is named** (`CFG.MONITOR_SIDE`), not searched for, because
  `peripheral.find("monitor")` would answer with a monitor on some other face and these
  computers' other faces are spoken for. It is capability-checked so a wrong block on that face
  draws on the terminal instead of raising at boot and taking both channels down.
  `false` = draw on the computer's own terminal (Aer Togekiss's #6 today).
* Receive is **one loop with no protocol filter**, dispatched on the returned protocol — two
  filtered loops would each pull, and drop, the other's messages.

### The panel

Coloured character cells only — no subpixel glyphs; nothing here moves fast enough to need the
resolution. The layout adapts to whatever size it is given and degrades a row at a time:
translation box (strafe across, fore/aft up, forward drawing **up** — the view from above), yaw
strip, VS column, readouts. Below a minimum size it collapses to numbers only, most important
first, and the stick still flies.

Redraw is gated on **state change** plus a 1 Hz refresh; the broadcasts are not. Repainting the
whole panel 20×/s while the pilot's hands are off the keys is pure waste.

On exit it restores the palette and blanks whatever it was drawing to **first** (leaving the last
frame frozen on a monitor would read as a still-live stick), then returns to the terminal to
report.

---

## 3. `vspdpid.lua` — the analog vertical lever (#26, #40; kept beside #6's `mancon.lua`)

Reads an analog 0–15 level on `IN_SIDE` and broadcasts a wanted climb rate.

```
0 ────────── 7 ────────── 14      15
 full descent  hold  full climb   SCRAMBLE
```

The top step is taken **out** of the scaling rather than read as a 15th of extra climb the
scaling does not know about.

> **CAUTION:** an unpowered input line reads 0, which is a **full-rate descent command**. Nothing
> here can tell a cut wire from a deliberate 0. **Wire the stick so it fails to 7.**

It runs **no control of its own** and drives **no redstone** — it names a rate, and the UAC owns
the loop that buys it.

`VS_MAX_CMD` should sit at or just below the UAC's `CLIMB_MAX`/`DESCEND_MAX`, or the top of the
stick travel simply saturates against the UAC's limit and loses resolution.

### Why the scramble is a flag and not a big number

It used to be `vs = 100.0`, and that could never have worked: **a rate is exactly what the rest
of the chain exists to limit.** The UAC clamps the wanted rate to `CFG.CLIMB_MAX` (3.75 m/s on
Aer Togekiss) *before* the rate loop sees it, so 100 m/s arrived as 3.75 — the same command a
level-14 stick already gives. The top step did nothing but repeat the second-top step.

Raising `CLIMB_MAX` is not the fix either: that is the handling envelope for the **whole**
proportional travel, and loosening it to buy one detent would loosen every other rate the
aircraft ever flies, including the autoland's.

So the scramble leaves the rate channel alone and travels as its own flag. A receiver that
predates the flag ignores it and keeps flying the rate beside it — degrading to "no scramble",
not to "100 m/s runaway".

---

## 4. `mic.lua` — the MANCON Intention Controller (#14 retired, #36, #42)

Converts binary key state into **wanted body velocities**. It runs no thruster control: like the
VSC it only names a velocity, and the UAC owns the loop — and because a released axis is
commanded to **exactly 0**, the UAC actively holds heading and nulls drift. That is the MIC's
stated purpose: *"cancel unwanted yawing when yawing is not commanded."*

```lua
{ fwd = <m/s>, strafe = <m/s>, yaw = <rad/s>, active = <bool> }
```

Full-stick targets are deliberately **less than the autopilot cruise**: `V_FWD_MAX` 50 m/s,
`V_STRAFE_MAX` 20 m/s, `YAW_MAX` 0.5 rad/s. The UAC clamps and calibrates further.

Its output protocol is **distinct** from the vertical `<f>_mancon` because the UAC must be able
to run a dedicated receive loop for it.

---

## 5. How the UAC treats a manual yaw

This is the part that is easy to get wrong, and the reasoning is in `controlLoop`:

* **Stick held** → feed the wanted yaw **rate** straight to the inner rate loop and skip the
  heading-error motion profile. Do **not** ramp a heading target and chase it: the position
  cascade's decel-limited profile is built to slew to a *fixed* heading and stop, so tracking a
  moving ramp makes it overshoot and reverse — the counter-steer buzz.
* **Stick centred** → `manYawRate` is nil and the heading error is held at ~0, so the stable,
  critically-damped inner rate loop nulls the yaw rate (`RATE_KP_YAW` alone — no extra brake
  gain, so no ring).
* **`manHeading` tracks the live heading while still rotating**, and is **frozen only once the
  rate has settled** (`MANNAV_YAW_SETTLE`). So the hold captures where the craft actually stops,
  not the release-instant heading — momentum carries past that, and a heading hold set there
  would drag the nose back the opposite way.
* **`MANNAV_YAW_BRAKE` is 0 on Aer Togekiss and should stay there.** With `BRAKE = 9` the
  effective gain gives complex closed-loop roots (|z| ≈ 0.84) — a ring on release. Level priority
  is bypassed for a brake (§03 §5) precisely so that when a brake *is* wanted it is not starved,
  but the gain itself is off.

---

## 6. `keyboardjoystick.lua` — #21, Citation (FIXEDWING)

```
    E nose up      D nose down     S roll left    F roll right
    SPACE throttle up    A / Z throttle down    C centre (pitch and roll to 0)
```

A **trim-style stick, not spring-loaded**: a key held ramps the target at `STICK_RATE`
(45 °/s); a key released leaves the target exactly where it was. That is the only thing a binary
keyboard can do smoothly — but it means the craft sits at whatever bank you last commanded, so
**C is the way back to level, not letting go**. The throttle behaves the same way, as a real
throttle lever does.

Broadcasts `{pitch, roll, throttle, active}` **every tick**, because the UAC gates on freshness.

Hard stops of ±60° here, deliberately **exceeding** the autopilot's `BANK_MAX` (45°): a
hand-flown aircraft may bank harder than the FMS ever plans for. The UAC clamps again at
`KJOY_PITCH_MAX`/`KJOY_ROLL_MAX`; keep the two in step.

### Per-axis authority — and the bug that made it necessary

The old single `kjLive` flag gated the **whole** stick — pitch, roll and throttle — on
`not navActive`, i.e. on AP-NAV alone. It never consulted AP-ALT. So with **AP-NAV off and ALT-AP
on**, the stick stayed fully live and its pitch **overwrote the altitude channel's commanded
pitch**. A centred stick means pitch 0, so the aeroplane flew resolutely level and sat at
whatever altitude the physics trims it to, while the vertical law upstream went on computing a
perfectly good climb command that was thrown away every tick.

That is the "commanded Vy large, achieved ~0", the "won't leave ~320", and the sub-one-step
elevator — **and it is why tuning the vertical law never moved the needle.**

The rule now: *each autopilot owns its own axes and MANCON gets the rest.*

| | pitch | roll | throttle |
|---|---|---|---|
| AP-ALT + AP-NAV | AP-ALT | AP-NAV | AP-NAV |
| AP-ALT only | AP-ALT | **stick** | **stick** |
| AP-NAV only | **stick** | AP-NAV | AP-NAV |
| neither | **stick** | **stick** | **stick** |

Common gates unchanged: FIXEDWING only (the mode where roll *is* the lateral control), and a
fresh packet, so silence hands the craft straight back rather than latching a stale bank.
`CFG.FLIGHT_MODE` is read live, so the MODE button arms and disarms the stick in flight.

### Two more UAC-side details

* **The stick OWNS the setpoint; it never blends.** Blending would recreate the
  two-terms-fighting-over-one-`bankTarget` failure the `BANK_XTK_SIGN` history documents at
  length — and a pilot's stick is not a term to be trimmed by the autopilot.
* **The joystick's pitch needs its own sign.** `attitude()`'s default `PITCH_ALT_SIGN` is 0 (the
  climb-pitch overlay is disabled), which would multiply the stick by zero.
  `CFG.KJOY_PITCH_SIGN` overrides it; `CFG.FW_PITCH_SIGN` exists for the same reason, and since
  both describe the same physical thing ("+cmd = nose up"), `FW_PITCH_SIGN` should **track** the
  in-flight-calibrated `KJOY_PITCH_SIGN` rather than be re-guessed.
* **The throttle overrides the velocity laws** rather than summing with them (a lever names
  engine power, not a speed), and zeroes the speed integral so it cannot wind up while a fixed
  lever position owns the channel. The MIC's **yaw** is untouched, so stick+throttle here and
  rudder there is a sane split.

---

## 7. `manconconf.lua` — the per-computer config

| Knob | Meaning |
|---|---|
| `OUT_VERT`, `OUT_NAV` | the two protocols the UAC listens on |
| `IN_APNAV` | **the unmerged `_1`** — the merged name carries no `autopilot_nav` field |
| `IN_ENGINE` | the engine switch |
| `IN_SCRAMBLE` | a remote scramble protocol, or `false`. **Canaried**, because `false` is a real answer |
| `IN_SCRAMBLE_STALE` | required whenever `IN_SCRAMBLE` is set — a remote scramble *must* be able to age out |
| `MONITOR_SIDE` | a side name, or `false` for the terminal. **Canaried separately**, because `false` ≠ absent |
| `KEYS.*` | ten actions, each a **list** of GLFW codes |
| `VS.SIDE / MAX_CMD / RATE / FADE` | the scramble lever's face; the vertical trim's envelope and feel |
| `NAV.*_MAX / RAMP / FADE / YAW_RAMP / YAW_FADE` | the horizontal integrator |
| `LOCK_ON_APNAV`, `LOCK_ON_MASTER` | the lockout |
| `TICK` | loop period |

**NEVER regenerate this file by copying the other computer's.** These are wiring facts and flown
calibration, and the comments beside them are the record of why. See
[11-conventions-and-traps.md](11-conventions-and-traps.md) §"The vecconf cautionary tale".

---

## 8. A scramble has exactly one broadcaster, whatever asks for it

The rule, restated because it is the single most load-bearing thing in this file:

* `mancon.lua` ORs its three sources and puts **one** flag on `<family>_mancon`, in the packet
  shape that protocol has always had.
* An automatic pull-up is therefore indistinguishable from the pilot's own detent to the UAC, the
  GPWS and the relays alike.
* It inherits `MANCON_SCRAMBLE_OVERRIDE` (over ALT-AP and the autoland), the trim freeze, and the
  staleness gate **for free**.
* #78 used to broadcast on `scout_mancon` directly. Two senders on one protocol alternate at
  20 Hz with the last to speak winning: the pilot's scramble flickered off every other packet and
  #78's `vs = 0.0` overwrote his climb trim.

**That is the same rule #13 and #14 are retired to keep.**
