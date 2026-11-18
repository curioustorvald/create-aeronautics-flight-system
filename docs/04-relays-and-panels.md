# 04 — Relays and panels

*The last hop: rednet float → whole redstone level → thruster; and the first hop: physical
redstone switch → rednet. These programs have **no conf file** — they hardcode their protocol
name and their face assignments at the top. That line is what to edit when cloning a computer
onto another airframe, and the usual bug is an old copy still broadcasting on the donor
aircraft's protocol.*

---

## 1. The shape every relay shares

```lua
-- 1. protocol names + FACE ASSIGNMENTS at the top, as globals
YAW_POS_SIDE = 'right' ; YAW_NEG_SIDE = 'left'  ...
STEP_MAX = 15 ; DITHER = true

-- 2. one sigma-delta accumulator PER AXIS (not per line), with a reset
local function makeDither()
    local acc = 0
    return function(value, lo, hi)
        local want = value + acc
        local q = clamp(math.floor(want + 0.5), lo, hi)
        acc = clamp(want - q, -0.5, 0.5)
        return q
    end, function() acc = 0 end
end

-- 3. signed float -> the two line levels that make it
local function axisLines(f, dither)
    f = clamp(f, -1, 1) * STEP_MAX
    local steps = DITHER and dither(f, -STEP_MAX, STEP_MAX)
                          or clamp(math.floor(f + 0.5), -STEP_MAX, STEP_MAX)
    if steps >= 0 then return steps, 0 else return 0, -steps end
end

-- 4. handlers, each owning its own `while true`, and ONE parallel.waitForAny at the bottom
```

### Why the accumulator is per **axis** and not per **line**

The two lines of an axis are one push-pull actuator. Accumulating each line separately would
leave the line that is going quiet owing a debt it could only repay by **firing against its
opposite number**, every time a command crossed zero.

The exception is `up`/`down`, which are **not** a push-pull pair — they are two independent burns
either side of the hover trim, so each gets its own accumulator. This is exactly how the UAC
dithered them before the work moved out.

### Why dither at all

What is left below a whole step is carried and paid out a frame or two later, and the airframe
(far slower than 20 Hz) integrates it back into the value that was asked for. Residual error is
at most **half a step**. Setting `DITHER = false` is plain rounding, i.e. throwing the fraction
away again.

The accumulator is **reset**, not fed, whenever the channel is cut — it holds at most half a step
and nothing should be owed across an engine switch.

### The `while true` placement is load-bearing

Each handler owns its own `while true`, and `parallel.waitForAny` is called **once**, at the
bottom. It used to be the other way round — one-shot handlers inside
`while true do parallel.waitForAny(...) end` — which rebuilt every coroutine for every message
and killed the others mid-receive each time. #3, #13 and #15 were already correct; **#9 and #10
were the two that were missed** and have since been fixed.

### The deadman

Every relay's receive is `rednet.receive(proto, 1)`. On a 1 s timeout it **zeroes its outputs**
and resets its accumulators. That is why the UAC must not suppress unchanged broadcasts:
silence is a trip, not a saving (invariant I4).

The one exception is the **lift** channel: on link loss it holds `ALT_NEUTRAL` on both lines
rather than plunging — but only if the engine switch says the lift is on. Keep that constant in
step with the UAC's `CFG.ALT_NEUTRAL` if either is retuned.

### Legacy branches

Every relay still accepts the pre-2026-08-26 message shape (already split, already whole). That
is what makes the deployment order **RELAYS FIRST, UAC LAST** safe; the other way round feeds
`nil` to `setAnalogOutput` on whichever side is not firing and kills the relay outright. Delete
those branches once every UAC on the fleet runs the float build.

---

## 2. The fleet's shared relays

### `msg_to_gas_output.lua` — stabiliser (pitch/roll)

**#3** Aer Togekiss · **#20** Citation · **#30** SS Argo · **#51** Mantine

| in | `<f>_stab` `{pitch, roll}` |
|---|---|
| gate | `attitude and master` |
| out | four faces, two per axis |

Aer Togekiss faces: pitch `back`/`front`, roll `left`/`right`.

`CFG.SWAP_PITCH_ROLL_CHANNELS` stays in the **UAC**, not here: it is a statement about how the
airframe is wired and the relay has no conf file to hold it in.

### `hdgcon.lua` — yaw + fore/aft

**#9** Aer Togekiss · **#23** Citation · **#32** SS Argo · **#50** Mantine

| in | `<f>_navcontrol` — reads `thrust` and `yaw`; `strafe` on the same message belongs to the strafe relay |
|---|---|
| gate | `master` |
| out | yaw `right`/`left`, thrust `front`/`back` |

> On Aer Togekiss the fore/aft thrusters are **not** on this computer — #69 has them. #9 keeps
> yaw. If any thruster were still on `front`/`back` here it would get two commands and the finer
> one would be wasted.

### `strafecon.lua` — strafe + lift

**#10** Aer Togekiss · **#33** SS Argo · **#43** Mantine

| in | `<f>_navcontrol` (`strafe`) and `<f>_alt` (`up`, `down`, `cut`) |
|---|---|
| gate | strafe: `master`; lift: `altitude and master and not cut` |
| out | strafe `right`/`left`; lift `front` (up) / `back` (down) |

Three accumulators: one for the strafe **axis**, one each for **up** and **down**.

The external splitter box is gone — the UAC does the `NEUTRAL ± steps` split (that is airframe
calibration) and this relay quantises each line.

> **The legacy branch here is the dangerous one.** `alt` is the one protocol whose field *names*
> did not change; the `cut` flag is the only thing telling old from new. A legacy `up = 7` read
> as a fraction clamps to 1.0 and puts the lift line hard against 15 on an aeroplane that asked
> for a balanced hover.

### `finerthrust.lua` — finer fore/aft (**#69**, Aer Togekiss only)

| in | `aertogekiss_thrustf` `{thrust}`, signed float |
|---|---|
| gate | engine switch; plus its own `LINK_TIMEOUT` (1 s) |
| out | fore `back` + `right`, aft `left` + `front` |

The NAVCTRL message is a whole step on one line, so the smallest thrust change the aeroplane can
be asked for is 1/15 of full scale. THRUSTF carries the demand as the control law computed it,
and this computer spends it on **two bundles**:

```
T = a + BUNDLE_B_RATIO · b        a, b ∈ 0..15,  BUNDLE_B_RATIO = 2
```

= 0..**45** A-units off two 0..15 lines, and every integer in that range is reachable. Same
1-A-unit step, three times the span → **three times the resolution**. The sub-A-unit remainder
goes through the same sigma-delta; residual error is at most half an A-unit = 1/90 of full scale.

`SPLIT_MODE`:

* `balanced` (default) — both bundles ride at ~the same level (`b = T/3`), A carries the
  remainder, so a demand change spreads over the whole group and the bundles stay
  thrust-symmetric.
* `stage` — fill A first, bring B in only above 15. Low-thrust cruise runs on the small bundle
  alone; the big one stays cold until needed.

---

## 3. Panels

### `engine_switch.lua` — **#11** · #22 · #31 · #49 (and Scout's `71/engineswitch.lua`)

Four redstone inputs → `{master, altitude, attitude, extra_lift}` on `<f>_engineswitch` at 10 Hz.

Aer Togekiss #11 faces: `front` master, `right` altitude, `left` attitude, `back` extra lift.

**Scout's #71** is one lever on `bottom` driving `master`, `altitude` and `attitude` alike — on
that airframe they are the same engines. It runs as a **tab** alongside `gpws.lua`.

### `autopilot_switch.lua` — **#12**, Aer Togekiss only

Listens to `aertogekiss_apswitch_1` and `_2` and re-broadcasts both on the merged
`aertogekiss_apswitch` at 10 Hz. It **merges**; it reads no redstone of its own.

The merged name is only read by the older relays' status handlers. **Other aircraft have no #12,
so anything depending on the merged protocol is dead there.**

Note it opens `rednet.open('top')` rather than `peripheral.find("modem")` — the older idiom.

### `manconrelay_rotation.lua` — **#15** Aer Togekiss · #35 SS Argo · #48 Mantine

Three jobs on one computer. On Aer Togekiss only the first is switched off:

| job | state on #15 | protocol |
|---|---|---|
| **yaw button relay** | `YAW_RELAY_ENABLED = false` (the MANCON on #6 owns the stick) | `<f>_mancon_rotational` |
| **touchdown detection** | **live** — the FMS and the GPWS read it as ground truth | `<f>_touchdown_detection` |
| **EXTRA LIFT bank** | **live** — repeats the UAC's `up` command onto its own bank | reads `<f>_alt` |

This is why #15 is *not* retired the way #13 and #14 are.

The extra-lift bank has its **own** sigma-delta on the command rate (not on its 20 Hz repeat
loop): `msg.up` arrives as a 0..1 fraction, and dropping that straight into `setAnalogOutput`
truncates it to 0 — the bank would simply never fire.

Squat switch on `back`, yaw buttons on `left`/`right`.

**To put the button panel back**, in this order:

1. `YAW_RELAY_ENABLED = true` in `15/manconrelay_rotation.lua`
2. uncomment the `shell.run` in `13/startup.lua` and `14/startup.lua`
3. point `6/startup.lua` back at `vspdpid.lua`

**#6 must not run both** — they broadcast the same `aertogekiss_mancon`.

### `manconrelay_translation.lua` — **#13** (retired on Togekiss) · #34 SS Argo · #52 Mantine

Six key inputs → `<f>_mancon_translational`, gated on `master and not apnav`. Broadcasts **only
while manual control is live**, so silence is how the MIC learns the keys are up.

### `scout_touchdown.lua` — **#78**

Two redstone reads at 20 Hz:

| in | out |
|---|---|
| squat switch, `redstone:front` | `scout_touchdown_detection` `{touchedDown}` |
| pull-up lever, `REDTAP_RIGHT:back` | `scout_pullup` `{scramble}` |

`scramble = (not touchdown and pullup)` — on the ground the squat switch owns the aeroplane and a
pull-up lever is a lever someone leant on.

Sent **every tick, true or false alike**, because the MANCON ages this link out; a flag that only
appeared when set would make silence ambiguous.

> **It used to broadcast on `scout_mancon` directly**, and that is the two-broadcasters bug in
> §I1: the pilot's scramble flickered off every other packet and the `vs = 0.0` sent alongside it
> overwrote his climb trim twenty times a second. The UAC's freshness gate could not save it —
> the gate is `if tonumber(msg.vs) then`, and 0.0 is a number. `vs` is gone from this computer
> entirely now: it has no business naming a climb rate, it asks for a scramble, and a scramble is
> not a rate.

---

## 4. Scout's hand-written relays

`84/stabil_scout.lua` and `85/scout_yaw_strafe.lua` are **hand-written for this airframe**, not
the fleet's shared programs. They follow the same shape (float in, sigma-delta per axis,
engine-switch gate) but their face assignments are their own, written at the top of each file.

They also drive **wrapped Redstone Relay peripherals**, not just the computer's own faces:

```lua
REDTAP_LEFT  = peripheral.wrap('left')
REDTAP_RIGHT = peripheral.wrap('right')
```

so an output is `REDTAP_LEFT.setAnalogOutput('back', level)` — the back face of the relay bolted
to the computer's left — while `redstone.setAnalogOutput('front', level)` is the computer's own
front face. **The two are different physical lines even when the side name is the same.**

### > THE FACE AND TAP ASSIGNMENTS IN THESE TWO FILES ARE CORRECT AS WRITTEN <

They were flown that way. In particular:

* `84`: `PITCH_POS_SIDE = 'left'` (via `REDTAP_LEFT`), `PITCH_NEG_SIDE = 'front'` (own redstone),
  `ROLL_POS_SIDE = 'back'` (via `REDTAP_RIGHT`), `ROLL_NEG_SIDE = 'back'` (via `REDTAP_LEFT`) —
  the two roll branches naming the same *side name* on **different taps** is deliberate.
* `85`: `STRAFE_POS/NEG` are both `'back'` on different taps; `YAW_POS_SIDE = 'front'` (own),
  `YAW_NEG_SIDE = 'right'` (via `REDTAP_RIGHT`).
* `85`'s `alti()` uses **`REDTAP_LEFT`** on the live branch and **`REDTAP_RIGHT`** on the
  link-lost branch, both naming `UPFIRE_SIDE`. That asymmetry is the specific thing that reads
  as a bug and is not.

This reads as a copy-paste slip and is not one. It caught out a reader as recently as
2026-08-30. **The wiring in these two files is the airframe, and the airframe is the authority.**

`85`'s `alti()` is also gated on **`alt`, not `att`** — it is the *altitude* engine switch. The
two are the same boolean today only because #71's panel drives all three off one lever; the day
that panel grows its real three switches, cutting ATTITUDE would otherwise have cut the lift too.

`84` and `85` still carry stale `_scout_vector_*` marker files beside their real ones, left over
from before the revert.

### Scout's lift is a PAIR SPLIT ACROSS TWO COMPUTERS

`sendAlt` broadcasts `up` and `down` either side of `CFG.ALT_NEUTRAL`, and:

* **#84** drives its bank off **`msg.up`** (`REDTAP_RIGHT:top`, `DOWNFIRE_SIDE`)
* **#85**'s own `alti()` drives a second bank off **`msg.down`** (`UPFIRE_SIDE`)

`84`'s in-code comment *"IF AN UP-FIRING BANK IS EVER BUILT ON THIS HULL…"* is **stale**: it was
built, on the other computer, and the note never moved.

One consequence worth knowing before touching the vertical here: the two lines always sum to
`2 · ALT_NEUTRAL`, because they are `NEUTRAL ± steps`. **A command changes the SPLIT between the
banks and never the total burn.**

Both files also note that the ALT channel must go through `lineLevel`, **not** `axisLines`:
`up`/`down` arrive already split, each 0..1, as two independent burns — not one signed push-pull
command. Handing a 0..1 line level to a −1..+1 axis quantiser happens to agree on the positive
half (which is why it went unnoticed) but can never produce the negative side and makes the
accumulator's clamp asymmetric about the wrong point.

### `86/thrustcon.lua` — Scout's fore/aft

Reads the **float** protocol `scout_thrustf` for the same reason #69 does: the output allocation
is this computer's business, so the grid the demand should be quantised onto is known here and
nowhere else.

Its output block is a **generic sink allocator**, shared line for line with `vecengine.lua` — fix
a bug in one and fix it in all:

```lua
{ side = "back", max = 15 }                       -- this computer's own face
{ per = "left", side = "back", max = 15 }         -- a face of a wrapped peripheral
{ per = "thruster_0", call = "setThrust", max = 15 }  -- a method call instead of redstone
{ ..., weight = 2 }                                -- what one level here is worth in channel units
```

A channel is a **list of sinks** and its integer level is spread across them, highest weight
first — so a channel is not limited to the 0..15 one redstone line carries. Put the weight-1 sink
**last**, or some totals cannot be expressed (they show up as `!` on the display).

`SCALE` is what `|thrust| = 1.0` is worth in channel units, and **it is per channel**; `nil` means
"this channel's own wired capacity".

> **Known asymmetry:** Scout's two forward lines are **one bank at weights 2:1** (the resolution
> trick), not two engines — so `fwd` is 45 and `aft` is 15. The aircraft therefore **accelerates
> three times harder than it brakes** until the aft bank is wired across two lines the same way.

---

## 5. Face-assignment quick reference

Aer Togekiss (the fleet's normal layout):

| computer | face | drives |
|---|---|---|
| #3 | `back` / `front` | pitch + / − |
| #3 | `left` / `right` | roll + / − |
| #9 | `right` / `left` | yaw + / − |
| #9 | `front` / `back` | (fore/aft — unused, #69 has it) |
| #10 | `right` / `left` | strafe + / − |
| #10 | `front` / `back` | lift up / down |
| #11 | `front`,`right`,`left`,`back` (in) | master, altitude, attitude, extra lift |
| #15 | `back` (in) | squat switch |
| #15 | `front` (out) | extra-lift bank |
| #15 | `left`/`right` (in) | yaw buttons (retired) |
| #69 | `back` + `right` | fore bundles A + B |
| #69 | `left` + `front` | aft bundles A + B |
| #7 | `front` (in) | AP-ALT switch |
| #8 | `front` (in) | AP-NAV switch (`CFG.APSWITCH.SIDE`) |

Scout: see §4 and the files themselves — **do not infer them from this table.**

---

## 6. Debugging a dead thruster: the order to check

1. **Is the relay's status line moving?** Each relay paints the received float and the levels it
   produced. `No … signal received` = the protocol name, the modem, or the sender.
2. **`active:`** on the same line — that is the engine-switch gate. `MST/ALT/ATT` are on another
   row.
3. **Is the UAC actually commanding it?** Its display shows the same numbers. If the UAC says 0,
   the fault is upstream (authority clamp, a law that is not running, an autopilot that owns the
   axis).
4. **Is something else on the same line?** Two commanders on one redstone face is the classic
   (#9 vs #69; the NAVCTRL yaw/strafe lines on a vector-nozzle airframe).
5. **Is it the legacy branch?** A UAC that has not been rebooted since 2026-08-26 sends the old
   shape; the relay's status line says `LEGACY`.
6. **Only then** suspect the wiring.

`84`/`85`/`86` carry `facemon.lua` and a `facelog.csv` for exactly this: see
[12-diagnostics-and-tools.md](12-diagnostics-and-tools.md).
