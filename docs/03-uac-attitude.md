# 03 — The UAC (`pid/uac.lua`)

*The Unified Attitude Controller: `pid/uac.lua` (~3 000 lines) + `pid/uacconf.lua`
(1 000–1 830 lines per airframe). One loop owns **every** thruster on the aircraft.*

---

## 1. What it is, and what it replaced

The UAC supersedes both the old gimbal stabiliser (`control.lua`, kept in `1/pid/obsolete/`)
and the old external navigator (`navcom.lua`, kept on #4 with its startup disabled). One loop
now drives pitch/roll (stab), yaw + fore/aft + strafe (navcontrol), and up/down (alt).

**There is no gimbal peripheral.** Attitude is derived only from the orientation quaternion.

Why quaternion, over the old Euler/gimbal law:

* no gimbal hardware, no gimbal lock, no ±π wrap seams;
* a single rotation-vector error that recovers a **fully inverted** craft along the shortest
  path (the gimbal could not);
* true angular-velocity and inertia-tensor data at 20 Hz, feeding a proper rate loop and
  gyroscopic decoupling, which a P-on-angle law cannot use.

And the structural reason: owning **all** thrusters lets the attitude loop treat the
disturbances the off-CoM translation thrusters inject as exactly that — disturbances it
rejects — instead of two computers fighting each other.

---

## 2. Boot

```lua
require("uacconf")      -- must sit beside uac.lua and return CFG
```

Then a **canary check**: one representative key per section, plus every key that is read
*unconditionally* every tick. Missing → a named boot error, never a nil-arithmetic crash
mid-flight. Conditional blocks are canaried conditionally: setting `PITCH_SMITH_ENABLE`,
`LOG_ENABLE` or `TRI_ENABLE` demands the rest of that block.

`applyFlightMode(CFG.FLIGHT_MODE)` derives three flags, and is a **function** rather than an
inline block so the on-screen MODE button can switch modes at runtime:

| mode | `STRAFE_ENABLED` | `ATTITUDE_BANKS` | tag |
|---|---|---|---|
| `VTOL` | yes | no | VTOL |
| `HYBRID` | yes | yes | HYB |
| `FIXEDWING` | **no** | yes | FW |

Anything unrecognised is treated as VTOL (safe default).

`makeYawQuat` is **auto-detected** at boot against whichever quaternion constructor the
installed Advanced-Math build has (`fromAxisAngle`, `fromEuler`, or `new(vec,w)`); none found
disables attitude rather than crashing.

---

## 3. Loops

```lua
parallel.waitForAny(controlLoop,
  navReceiveLoop, altReceiveLoop, autolandReceiveLoop,
  switchApnav, switchApalt, manConReceiveLoop, manNavReceiveLoop, kbdJoyReceiveLoop,
  displayLoop, inputLoop)
```

Each input gets its **own** receive loop, because two `rednet.receive` calls cannot share one
protocol filter — they would race for the same messages. All of them **latch** their last value
between packets and let `controlLoop` decide on freshness; a dropped packet must never
disengage anything.

`shutdown()` runs on exit: zeros on stab/navctrl/thrustf, engines cut on `tri`, and the lift
pair parked at the **learnt hover trim** (rounded — see §7).

---

## 4. One tick of `controlLoop`

```
1  read pose, linear velocity, angular velocity          (3 blocking sublevel calls)
2  slow-poll inertia tensor + mass if due                 (0 or 2 more)
3  latestPosY = pos.y − CFG.ALT_OFFSET                     ← invariant I5, first thing
4  decide navActive / useAltCascade / autolandActive
5  one toEuler → ui.pitch, ui.eulerYaw, ui.roll; heading
6  nav geometry: dist, bearing, closing, hasLeg, xtk, xtkRate, dtk
7  law selectors: translateOnly / approachVec / navCarrot / alignStop / headingHold
                  padPhase / bankSteer / strEager / fwPitch
8  manual channel freshness + per-axis ownership (manNavLive, kjPitch/kjRoll/kjThr)
9  desiredHeading per phase
10 idle gate (stillFor ≥ IDLE_SETTLE)
11 bank/pitch setpoint overlays (HYBRID/FIXEDWING), or FW vertical-by-pitch, or the stick
12 attitude()  →  pitchSteps, rollSteps, yawSteps
13 horizontal translation  →  thrustCmd, strafeCmd
14 vertical cascade         →  vertEffort
15 attitude-aware vertical allocation (tilt) → altSteps, spill into thrust/strafe
16 output: sendStab / sendNav / sendAlt   (or one sendVector)
17 publish ui; LOG.tick
18 deadline pacing
```

---

## 5. `attitude()` — the quaternion law

Returns signed `pitchSteps, rollSteps, yawSteps`. **Everything is computed in the BODY frame** —
the frame the per-axis thrusters live in — and there is no final command rotation, so a pure
pitch demand stays a pure pitch command.

### The error

With `CFG.YAW_TILT_DECOMP = true` (every airframe), the error is **decomposed**:

* **tilt** = the rotation bringing body-up back to world-up, taken from the up vectors (smooth
  except at a full inversion), rotated into the body frame → `ex` (pitch), `ez` (roll);
* **yaw** = `wrapPi(desiredHeading − heading)` → `ey`, a pure rotation about the up axis.

That is Bug 1's fix: a ~180° heading change stays *pure yaw* and never hits the
combined-quaternion 180° singularity, whose ill-defined axis produced a pitch kick. The
fallback combined-quaternion path is kept but unused; `q_des` is built **inside** that branch
because the decomposed path never reads it.

### The commanded-attitude overlay (HYBRID/FIXEDWING)

`ex -= pitchCmdSign · pitchCmd`, `ez -= BANK_SIGN · bankCmd`. Subtracting the command moves the
**equilibrium** to that attitude, so everything downstream — the integral, the P/D law, and the
tilt/yaw level-priority that reads `sqrt(ex²+ez²)` — works relative to the *commanded* attitude.
A deliberate bank is therefore not mistaken for an upset and does not fade the yaw during a
coordinated turn, while a genuine flip still does. Both commands are zero in VTOL and in
straight-and-level flight, so this is an exact no-op there.

`ui.pitchMeas` / `ui.pitchApplied` publish the measured and applied pitch **in the same
convention**, which is a sign-safe diagnostic needing no new calibration: at equilibrium they
must converge; equal and opposite means the pitch command's sign is inverted; command large and
measured stuck short means not enough authority.

### The integral

`integStep` accumulates **only when the axis is quasi-static** (small error *and* low rate).
A large error is a manoeuvre (bleed toward 0 at ×0.98); any real rotation means the P/D loop is
doing dynamic work the integral must not join — integrating through motion lags the error ~90°
and pumps a slow, growing oscillation. So the integral learns the **steady trim only, near
rest**.

`idle` (parked / engine off) bleeds it at `ATT_I_IDLE_BLEED`: never learn a trim from a craft
that is not flying, since the tilt it reads is imposed by the ground, not by a moment the
thrusters could null.

Pitch always has it. **Yaw and roll are opt-in** (`ATT_INTEGRAL_YAW`, `ATT_INTEGRAL_ROLL`,
default off).

> **Why roll wants it on a multirotor** (Scout, measured over 201 hover samples): with no
> integral the axis can only answer a standing moment with a standing **error**, and on a craft
> whose lift vector *is* its attitude, a standing roll error is a standing sideways
> acceleration nothing cancels. `errR` held +0.0125 rad (+0.72°) with `integR` flat at exactly
> 0.0000 — that is `11·sin(0.72°) = 0.14 m/s²` of permanent sideways push. Pitch, which has the
> integral, sat at +0.0001 with `integP` at a steady 0.0177 on the same flight.
> **`CFG.ATT_INTEGRAL_ROLL` must stay true on Scout.**

### INERTIA mode (all airframes)

A cascade:

* **outer**: `profileRate(err, ATT_KP_<axis>, MAX_RATE_<axis>, MAX_DECEL_<axis>)` — a
  decel-limited motion profile that turns error into a desired **rate** a high-inertia axis can
  still brake from, so yaw does not overshoot or spin.
* **inner**: `RATE_KP · rateErr + ATT_KI · bias − yawBrake · wy`, optional `RATE_KD`.
* then `I · a` (full tensor if `INERTIA_COUPLING`, else the body diagonal), plus a **faded**
  gyroscopic term `ω × (Iω)`.
* then `ROT_GAIN_<axis> · torque` → steps.

**Each axis has its own outer gain** (`ATT_KP_PITCH/ROLL/YAW`, falling back to `ATT_KP`).
`ATT_KP` sets the cascade's **damping ratio**, `ζ = 0.5·sqrt(G/ATT_KP)`, and the three axes hang
off three different thruster sets with three different `G`. One shared number cannot damp them
all.

**Gyro fade** (`GYRO_FADE_LO`/`HI` on |yaw rate|): the lagged feedforward stands down before it
can pump the fast-yaw nutation.

### Level priority

`yawScale` fades yaw out as pitch+roll error grows (`TILT_YAW_MIN` 0.20 rad → `TILT_YAW_MAX`
0.60 rad), so a yaw manoeuvre can never roll the craft over faster than the levelling loop can
recover.

**A manual yaw brake gets full authority anyway** up to the hard flip guard: nulling a spin
*reduces* flip risk, and must not be starved by the little nutation-tilt a hard yaw throws off.

Yaw is computed **first**, then its coupling is fed forward into pitch/roll based on the
**actual** yaw command (so it fades with `yawScale` exactly as the real disturbance does).

### The pitch Smith predictor (`PITCH_SMITH_*`, #19 only today)

Dead-time compensation. Measured: actuator dead time ~1 s; the old ±4° limit cycle ran at 4 s =
1.5 rad/s, where 1 s of delay is 86° of phase — **that cycle *was* the dead-time instability**.
Usable bandwidth without compensation is ~0.3–0.5/τ_d and the loop already sits at ω_n = 0.25
rad/s, i.e. at the wall. *No gain buys pitch response past that point.*

The plant is an **integrator** (torque → angular accel → rate), so the rate the aeroplane will
have once the pipeline drains is the measured rate **plus every acceleration already commanded
within the last τ_d but not yet felt**. Keep those in a ring buffer and add them.

**Two things that make it worse than no predictor if got wrong:**

1. **Buffer the OUTPUT steps × a measured accel-per-step**, never `axCmd`. `ROT_GAIN_PITCH` is a
   tuned *loop gain*, not an absolute calibration: `ROT_GAIN_PITCH · Ix` implies 0.078 rad/s²
   per step while the limit-cycle measurement says the real figure is ~0.008. Feeding `axCmd`
   would have the predictor expect **~10×** the rate the aeroplane develops.
2. Reading the clamped output also makes it correct through saturation for free — an
   unachievable demand is never buffered, which would otherwise back the loop off while still
   saturated (the windup pattern this file keeps rediscovering).

**It fails invisibly.** If pitch gets worse after enabling it, set `PITCH_SMITH_ENABLE = false`
**first**, before touching any gain, and re-measure τ_d. Display tag `SMI`.

---

## 6. Translation

Four laws, selected by phase and flags:

### (a) Velocity-vector law — `translateOnly | approachVec | navCarrot | alignStop`

Used for `TERMINAL`/`ARRIVED`/`LANDING`, for `APPROACH`, for the en-route **carrot**, and for
the VTOL/HYBRID route-start ALIGN hold.

```
desiredSpeed = navCruiseCap                      (when schedSpeed and carrot/approach/align)
             = clamp(APPROACH_KP·dist, 0, cap)   (otherwise — the position hold)
velErr       = desiredSpeed·û − v
fwdCmd       = STEPS_PER_MPS_FWD·(desiredSpeed·û·body_fwd) + TRANS_KP·fwdErr
rightCmd     = STEPS_PER_MPS_LAT·(desiredSpeed·û·body_right) + STRAFE_VEL_KP·rightErr
```

The first term is an **inverse-plant feedforward**: the commanded velocity vector projected into
the body frame, times the steps that buy one m/s. On its own it already commands very nearly the
right thrust, leaving the gains only trimming what it misses. Without it this law — which is
what flies the normal cruise carrot — was a bare P with no integral, and had to stand a 20 m/s
speed error just to hold full thrust.

**The breakaway floor** (`TRANSLATE_MIN_CMD` / `_FWD`) exists so a tiny demand can still break
static friction, and it has three properties that were each a fixed bug:

1. **Per-axis.** One floor in *step* units cannot mean the same thing on both axes: a step is
   `SPEED_AT_FULL/THRUST_MAX` of fore/aft but `STRAFE_AT_FULL/STRAFE_MAX` of lateral, and on Aer
   Togekiss those differ **eightfold** (21.3 vs 2.67 m/s per step). The old flat 5.0 was a sane
   13 m/s nudge sideways and a 107 m/s **lunge** fore/aft. It is now blended along the demand's
   own direction, so it scales the magnitude and preserves the direction.
2. **A fade, not a switch.** `closing < MIN_RATE` was a relay: at 0.349 m/s full floor, at 0.351
   whatever the PID said, nothing in between. An equilibrium on that switching surface cannot do
   anything but limit-cycle — kick, accelerate through, boost drops, coast, overshoot, kick back.
   *That was the fore-aft rocking.*
3. **A commanded zero is a stop, not a breakaway.** Gated on `desiredSpeed > 0`. During the
   ALIGN hold none is commanded, so the whole demand is the brake `KP·(−v)`; a parked craft is
   never exactly still, so `mag` was a few hundredths of a step and the floor multiplied that
   residue up to the **full** breakaway, bang-bang about zero, with the HSI reading ALIGN /
   CMD 0.

Both PID integrals are **zeroed** every tick in this law.

### (b) ENROUTE along-track + cross-track hold

Outer P on closing speed (or, with `schedSpeed`, **track the FMS's cap exactly** — the FMS
already folds in the proximity, alignment and cross-track gates, and re-deriving any of it here
would double-cut the throttle and make the HSI's CMD a lie). Inner PI buys the speed; a separate
strafe PI(D) holds the leg's cross-track.

**A commanded zero clears the speed integral.** This is the only translation law that carried its
integral through a zero, and the consequence was "the autopilot accelerates during the align
phase": the integral is bounded at `THRUST_MAX/SPD_KI`, nothing unwinds it (AP-NAV off does not
*step* the PID, it just stops stepping it), and at a standstill the loop's entire output is
`SPD_KI · integral` — up to **full forward thrust** while the HSI reads CMD 0. It cannot clear
itself either: accelerating at 90° to the target leaves `closing ≈ 0`, and pointing away makes it
negative and winds up further.

With **no leg** (a direct-to) the fallback is a body-right velocity error toward the target, so a
slow high-inertia yaw does not fly a pursuit spiral with the strafe thrusters never commanded.

### (c) Manual (MIC / MANCON)

Same body projection, same gains and signs as (a), but the target is pilot-named. A released
stick sends 0, which is an **active brake to a hover and heading hold**, not a coast.

### (d) Keyboard throttle

A lever names engine **power**, not a speed, so `0..1` maps straight onto forward thruster steps
and **overrides** the velocity laws rather than summing with them. One-sided: no reverse.

### Nothing driving it

When AP-NAV is off with no MANCON, or on with no target: both integrals are **zeroed**, not
merely left unstepped. Otherwise they survive the disengagement intact and are spent the instant
navigation returns, before a single tick of error has been measured.

---

## 7. Vertical

### Sources of the desired climb rate, in precedence order

1. **SCRAMBLE** → `CLIMB_AT_FULL` (honest, because it is what the ALT line displays and what the
   FIXEDWING pitch overlay flies off next tick — the old `vs = 100` placeholder was not).
2. **autoland `descend`** → `alDescendVS`, zeroed if the FMS goes silent for `AL_STALE_AFTER`.
3. **on the gear** (`alGrounded`) → a small standing `−AL_GROUND_VS`, **with no altitude
   reference at all**. An altitude hold here is actively wrong: the moment the craft settles a few
   centimetres it answers by firing the lift thrusters **up**, which keeps the aircraft skittish,
   unloads the gear and on a rollout stops the wheels braking.
4. **ALT-AP / autoland `hold`** → `altPID:step(latestPosY)` (outer P only).
5. **manual** → the VS stick's rate, clamped to **`MANCON_CLIMB_MAX`/`MANCON_DESCEND_MAX`**;
   stale → 0. A separate envelope from the autopilot's `CLIMB_MAX`/`DESCEND_MAX` since
   2026-09-08, so `mancon.lua`'s boost key can ask past the handling limit without
   loosening the one ALT-AP and the autoland fly inside (`AL_CLIMB_MAX`/`AL_DESC_MAX` are
   those two). Keep it at or below `CLIMB_AT_FULL` — beyond that the rate loop saturates.
   **Fork only** (#1/#72), the one control change the optimised build carries.

### The inner climb-rate loop

`steps = STEPS_PER_MPS · desiredVelY + velPID:step(velY)` — feedforward + trim, the same shape
the horizontal law uses. Clamped to `±ALT_SWING`.

Skipped **entirely** on an airframe with no lift thruster (`HAS_ALT_THRUSTER = false`, i.e.
Citation): `desiredVelY` is still computed (the FIXEDWING pitch law flies it) but there is no
actuator, so running the loop would wind `velPID` against an output nobody receives and then leak
that phantom effort into the **forward** thruster through the tilt allocation, fighting the speed
loop.

The integral is **frozen, not bled**, while idle / on the gear / cut / scrambling — the hover lift
is a real value wanted at the next takeoff, unlike the attitude trim. `TRIM_BOOST_*` re-learns it
quickly after a mass step.

`AL_GROUND_CUT`: once the FMS calls "landed", both lift lines go to a flat zero (`cut`). Holding a
balanced hover leaves the engine burning against the ground.

### Scramble

`ui.scramble` is `MANCON_SCRAMBLE and ENABLE_ALTITUDE and HAS_ALT_THRUSTER and manScramble and
fresh and (MANCON_SCRAMBLE_OVERRIDE or not useAltCascade)`.

`vertEffort = CFG.ALT_SWING` **outright**, stated rather than left for the rate loop to arrive at.
The feedforward alone is exactly `ALT_SWING`, so in the ordinary case this changes nothing — but
only while the craft is *slower* than `CLIMB_AT_FULL`. Once it is not (a dive recovery, a gust, or
a conservatively-set `CLIMB_AT_FULL`), the rate loop starts **subtracting** lift to hold the
figure, which is the exact opposite of what the pilot asked for. *A scramble is an open throttle,
not a rate.*

---

## 8. Attitude-aware vertical allocation (Bug 2)

"Hold altitude" is world-up; the lift thruster is body-fixed. So route the vertical effort by the
true orientation:

```lua
ux, uy, uz = rotateVec(q_inv, 0, 1, 0)         -- world-up expressed in body
altSteps   = vertEffort * uy                   -- body-up  → the lift thruster
thrustCmd += vertEffort * uz * VERT_XFER_FWD   -- body-fwd → fore/aft
strafeCmd += vertEffort * ux * VERT_XFER_STR * (bankSteer and strEager or 1)
```

A no-op at level (`world-up-in-body = (0,1,0)`). It flips sign inverted and fades toward 90° of
pitch.

**The lateral spill is left uncancelled while steering by bank**, because that sideways component
*is* the lift turning the craft; cancelling it would defeat the bank.

**A scramble is not scaled by tilt — only its sign is routed.** Everything else on this channel is
a rate, and a rate *must* be scaled by `uy`. A scramble is not: scaling it made the detent
attitude-dependent — 0.87 of full at 30°, half at 60°, and **exactly zero at 90°**, i.e. doing
nothing at all in the one attitude a pilot is most likely to be holding it in. So the magnitude is
pinned at full and only `sign(uy)` is kept (an inverted craft still scrambles on the down bank),
and `sendAlt`'s `openThrottle` puts the opposing bank at `ALT_MIN` so the pair is `ALT_MAX/ALT_MIN`
at **every** attitude.

Near knife-edge the sign is not meaningful and will flip. Deliberately left alone: at 90° both
banks fire horizontally so neither choice buys or costs any climb, and the attitude loop is
already rolling out of it.

> **Aer Togekiss and Scout differ here.** Scout has `VERT_XFER_FWD = VERT_XFER_STR = 0`, disabling
> the spill after it drove a 3.6 km uncommanded climb. Restoring them needs the sign settled, and
> that is tied to `CFG.HEADING_OFFSET = math.pi`, which has never been verified against that build.

### `sendAlt`'s split

```lua
if cut          then up, down = 0, 0
elseif openThrottle then                         -- SCRAMBLE
    burn = clamp(ALT_NEUTRAL + |steps|, ALT_MIN, ALT_MAX)
    steps < 0 and (ALT_MIN, burn) or (burn, ALT_MIN)
else
    up   = clamp(ALT_NEUTRAL + steps, ALT_MIN, ALT_MAX)
    down = clamp(ALT_NEUTRAL − steps, ALT_MIN, ALT_MAX)
end
```

**Why `openThrottle` exists** ("the scramble is nullified by the alt-down thrusters", 2026-08-29):
`NEUTRAL ± steps` is the right split for a *rate* — the two banks are a differential about a
balanced hover, and the down line backing off is half of what buys a climb. It is the wrong split
for an *open throttle*. With `ALT_NEUTRAL = 7` and `ALT_SWING = 8`, the down line only reaches
`ALT_MIN` while `cos(tilt) ≥ 0.875`, i.e. within ~29° of level. Past that the retro bank lights up
and burns **against** the scramble: at 60° it is up 11 / down 3, and the net is barely half of what
the detent asked for. And a scramble is precisely the command reached for when the craft is *not*
level.

---

## 9. Bank, and the modes

`bankSteer` = "this tick the mode corrects lateral course by **banking**":

```lua
bankSteer = ATTITUDE_BANKS and (alState == "normal" or navGlide)
            and not (FLIGHT_MODE == "HYBRID" and padPhase)
```

* VTOL never banks (it strafes).
* HYBRID banks en route, hands lateral control to strafe (wings level) in the pad phases.
* FIXEDWING has no strafe at all, so it banks in **every** phase.
* An autoland forces wings-level in both — **unless** `navGlide`, a runway glideslope, which is
  an ordinary flying leg that happens to be descending. Without that exception a FIXEDWING
  approach would have no lateral control at all.

### The two bank sources

* **heading-error lean** — `BANK_PER_YAWERR · hErr`, faded near the target
  (`BANK_HDG_FADE_NEAR/FAR`), because bearing to a *point* whips around at short range exactly as
  the old per-fix closing PID did.
* **cross-track bank** — the **primary** in-course lateral corrector: `BANK_XTK_SIGN ·
  BANK_PER_LAT · lastLatDemand`, i.e. it banks toward the track using the same body-right demand
  strafe would have applied, so the tilted lift pulls the craft back on line instead of strafing.
  Rate-damped on `xtkRate`.

They **add** — they agree in sign for every geometry — with the cross-track term the strong
primary and the heading term the lean/trim.

### `BANK_HDG_KD_WASHOUT` — damp the transient, not the steady turn rate

The heading-bank damper used to subtract the **raw** yaw rate. That is right for a heading
*capture* (steady state: wings level, rate 0) and wrong for a curved **path** (steady state: a
sustained rate). On an arc the loop settles where

```
bank = BANK_PER_YAWERR·hErr − BANK_HDG_KD · g·tan(bank)/v
```

so the damper never stops pulling: it stands a permanent droop of `BANK_HDG_KD·g·tan(bank)/v`,
and because it **divides by speed** that droop *grows* as the FMS slows for a corner — it bites
hardest exactly where the most bank is needed. At 120 m/s it costs 0.11 of 0.785 rad (14 %, which
is why it looked healthy); at the 51/35/23 m/s Mantine's FMS schedules for the QATON/QETAM/BAZRA
corners it costs **33 %/47 %/74 % of the entire bank authority**. Holding 45° at 23 m/s would need
`hErr = 134°`, which pure pursuit to a 400 m carrot can never present — so the craft banked ~30°,
flew every corner 2–3× wide, and read to the pilot as *"it will not turn"*.

**The fix is the filter, not a smaller gain.** `bnk.yawWash` is a slow low-pass of the same rate,
so `(rate − yawWash)` is the **change** in turn rate: settled on an arc it is ~0 (no droop, full
authority, the planned radius is reachable); rolling in or ringing it is live and damps exactly as
before. Lowering `BANK_HDG_KD` would trade the droop straight back for the ringing it cured.
The washout is stepped **every** tick, outside the `bankSteer` block, so it never goes stale
across a pad phase or a MODE press and then dumps a fake transient into the first banking tick.

### `STRAFE_EAGER_NEAR/FAR`

Below `NEAR` (8 m) of remaining correction, hand it to **strafe** and switch the cross-track bank
off; above `FAR` (25 m), the mode's normal policy. Close in, `latBank` is the only bank source left
(the heading term is already faded) and it rides pure velocity noise, which the coordinated-turn
sim's floored divisor turns into full-rate yaw commands at a standstill. With no target at all
there is no correction, so it is fully eager — this is the "no mini-swaying on the ground" case.

### `HDG_SIM` — bank becomes a turn

`ω = g·tan(bank)/v`, floored at `HDG_SIM_MIN_SPEED`. Two callers, one law:

* **HYBRID's coordinated-turn sim** (`HDG_SIM_ENABLE`), where the bank is the autopilot's own.
  It takes no gain and no reference speed: it flies a radius the FMS planned from real physics.
* **the keyboard joystick** (`KJOY_BANK_TURN`), where the bank is the pilot's. FIXEDWING normally
  needs no such coupling — its yaw law chases the live bearing — but with AP-NAV off there **is**
  no bearing, so the nose would stay rigidly put while the banked lift slid the craft sideways.
  This case closes no control loop, which is why it needs none of the damping and fading HYBRID's
  closed loop had to grow, and why it may scale (`KJOY_TURN_GAIN`) and pin the divisor
  (`KJOY_TURN_REF_SPEED`).

---

## 10. FIXEDWING vertical-by-pitch (`FW_PITCH_*`)

On an aeroplane with no lift thruster, the only way up is to point the nose up and let forward
thrust carry the craft along the new flight path. So `desiredVelY` stops being a thruster demand
and becomes the input to a **flight-path-angle command** the attitude loop holds like any other
setpoint. Unlike the old cosmetic climb-pitch overlay this is **not** gated on `bankSteer` — the
vertical channel must keep working through autoland, the pad phases and a glideslope, i.e. exactly
the states `bankSteer` switches off.

**It is a PI on vertical speed, and the standing angle is *learned*, not assumed.** That is a
deliberate retreat from two earlier versions that computed the angle geometrically (`γ = vs/V`).
Geometry is only right if pitch sets the **flight path angle** — true of an aeroplane held up by
its wings, false of one held up by thrust, where pitch sets vertical **acceleration** and a
constant commanded angle is a **ramp generator**: vertical speed never settles, it just grows until
something saturates. That is the "commanded −3.75, reached −10, reversed, repeated" the pilot
measured, and a bench reproduces it (−10 to −11 with a dozen-plus reversals on an integrating
plant, against −4.4 and none on a lifting one). An integrator that learns whatever standing angle
holds the wanted rate is correct on **both** plants.

All terms are divided by the live speed, so they are angles — which is also a gain schedule that
keeps one set of numbers working from a 20 m/s approach to a 280 m/s cruise. Use the **total**
speed, not the horizontal: `speedH = V·cos γ`, so pitching up shrinks the divisor, raises the gain
and pitches up further — a small positive feedback loop inside the negative one.

**The trim term has a different speed law from the other two.** `FF·vsCmd` and `KP·vsErr` are
flight-path angles (`vs/V`), so one division by `V` is right. `KI·pitchI` is an **angle-of-attack
trim**, and for constant lift `C_L ∝ 1/V²`, so trim AoA falls as `1/V²`. Scaled by
`FW_PITCH_TRIM_VNOM` so the law is identical at that speed and an already-learnt integral carries
over unchanged.

**Speed protection** (`FW_PITCH_HEADROOM`): never ask for a climb the aeroplane is too slow to fly.
Pitching up does not *make* lift here, it only aims the thrust upward, so an unsustainable climb is
never delivered — the rate error never closes, the trim winds until the command pegs, the nose-up
bleeds the very speed the climb depends on, and the aeroplane mushes nose-high and goes over the
top. The cap uses the **raw** speed, deliberately not the floored `vRef`, so as the craft slows the
commanded rate shrinks to zero on its own, the nose comes down, it accelerates, and the climb
resumes — which is what a real aeroplane and a real autopilot do.

**Anti-windup, and the two bugs it had:**

* While the cap bites, **freeze, do not bleed**. The branch used to decay the trim at 0.95/tick,
  and it fires for *any* altitude error over ~122 m at cruise — so every large altitude change
  destroyed the vertical trim and held it at zero for the whole approach. Reported as "climb rate
  inadequate": it was climbing on geometry alone with the learnt trim deleted underneath it.
* But a **plain freeze locks in an already-wound-up trim**, which is worse. With a target above the
  lift ceiling, `vsErr` stays positive, `pitchI` winds to its limit (0.30 rad = 17° of standing
  nose-up), and the clamp-based anti-windup never fires because 17° sits well inside
  `FW_PITCH_MAX` (26°) — it can only see the **command** saturating, never the **aeroplane**. Then
  a few degrees of descent demand cannot out-vote 17° of trim: "pins at 300–320 regardless of the
  set altitude". The correct answer is **one-directional integration**: never let it grow while
  capped, always let it unwind toward zero.

**A zero slew step must never latch a command nobody owns any more.** `CFG.PITCH_ALT_RATE` is 0
(the old overlay is disabled), and it is the fallback rate — so when an owner stands down
(joystick stale, MODE leaves FW), `pitchTarget` correctly drops to 0 but the step to get there is
also 0, and `pitchCmd` stays where that owner left it for the rest of the flight. One guard fixes
it for both the joystick and the FW path.

---

## 11. Output

Since 2026-08-26, **`uac.lua` quantises nothing** (invariant I3). Every output leaves as a float;
the authority clamps (`STAB_MAX`, `YAW_MAX`, `THRUST_MAX`, `STRAFE_MAX`, `ALT_MIN/MAX`) are applied
**first**, in step units, before dividing by `RS_MAX = 15` — normalising on `*_MAX` instead would
hand the limit straight back when the relay multiplied by 15 again.

One behavioural change came with the move, and it is an improvement: **the accumulator is fed the
clamped command**, where before it was fed the raw one. Bench-diffed, the two chains are
step-for-step identical over 200 ticks on all five airframes and 60 k fuzzed in-band commands —
they part company only when a command **saturates**. The old sigma-delta went on integrating the
part of the demand the actuator could never deliver and paid that phantom debt out afterwards (81
of 200 frames off by a step). That is integrator windup; clamping first is the standard cure.

### The vector-nozzle allocator (`sendVector`, `TRI_*`)

Scout's original build: three down-firing engines on vectoring mounts (front-centre, back-left,
back-right), each taking gimbal X, gimbal Y and thrust. Pitch, roll, yaw, strafe and lift all come
out of those three nozzles; only fore/aft keeps a bank of its own.

**Why it lives in the UAC and not in a relay:** every other output is one axis a relay can
translate knowing nothing else. Here every axis lands on every engine, and how a demand splits
depends on what the *other* axes are already spending (an engine held at 15 for lift cannot also
make yaw). A relay sees one channel; only the controller sees the whole wrench.

The solve, in a fixed order:

1. **Horizontal** — 6 unknowns `(fx, fz)`, 3 equations (strafe, fore/aft share, yaw). Least-effort,
   **weighted by how much thrust each engine is already carrying** (an engine at 12 lends more
   sideways push at the same nozzle angle than one at 3). The weights choose *between* exact
   answers and never change the wrench delivered, which is why last tick's thrusts suffice and no
   inner iteration is needed.
2. **Vertical** — 3 unknowns `(fy)`, 3 equations (lift, pitch, roll): exactly determined once the
   pitch/roll moment step 1 committed to (the `y_i` terms) is subtracted. Linear, so its three
   basis solutions are computed **once at boot** (`TRI.n` pure lift, `TRI.p` pitch, `TRI.r` roll);
   `p` and `r` each sum to **zero** by construction.
3. **Saturation: attitude outranks lift.** When the demands will not fit, the differential is kept
   and the *common* lift slides to make room — a multirotor that loses lift sags; one that loses
   pitch/roll authority turns over. Only when no common lift fits the differential is the
   differential scaled, and then by the exact largest admissible factor, not by bisection (a scale
   that is exact never chatters).
4. **Cone limit** — a nozzle cannot make more sideways force than `tan(cone)·fy`; `fx` and `fz` are
   scaled **together** so the direction survives. What will not fit is **dropped** rather than
   commanded and quietly not delivered; the attitude loop sees the shortfall as a disturbance and
   answers next tick, which is correct.

Boot-time refusals: three engines exactly; non-degenerate geometry (three mounts on one line
through the CoM cannot tell lift, pitch and roll apart); and every engine must **push** to hold a
level hover (a non-positive share means the CoM is outside the triangle of mounts, which in the
air presents as an unkillable pitch or roll bias no gain will tune out).

**Currently inert fleet-wide** (`CFG.TRI_ENABLE = false` on Scout since 2026-08-28), but it is the
only implementation of this, so keep it working.

---

## 12. The loop-rate fork

**#1 and #72 run a forked, optimised `uac.lua`. #19, #27 and #38 run the pre-fork build.**
`_goldmaster/uac.lua` is the common ancestor snapshot (see its README).

The fork changed **no gain, sign or control law**. Both builds were captured tick by tick under a
stub harness; every command on every protocol is identical to 17 significant figures.

| Change | Why |
|---|---|
| inertia tensor + mass polled at `SLOW_POLL_PERIOD` | two fewer blocking `sublevel` round trips per tick |
| tensor unpacked once into nine numbers; `matVec` allocates nothing | was 2 throwaway tables per call, 3 calls a tick |
| body-frame diagonal `Ix/Iy/Iz` cached with it | was 3 closure calls a tick to read 3 unchanging numbers |
| `q_inv` computed once and passed into `attitude()` | was computed twice a tick |
| `toEuler()` once a tick | was three times, two of them discarding 2 of 3 results |
| `makeYawQuat` skipped under `YAW_TILT_DECOMP` | `q_des` is unused on that path — a wasted quaternion every tick |
| **deadline pacing** instead of unconditional `sleep(0.05)` | an already-late tick was buying itself another whole 50 ms |
| `drawAH` draws **runs** and skips unchanged rows | ~1400 terminal calls/frame → ~46 |

### The pacing, specifically

```lua
local spent = os.clock() - now
if spent < CFG.LOOP_PERIOD then sleep(CFG.LOOP_PERIOD - spent)
else os.queueEvent("uac_tick"); os.pullEvent("uac_tick") end
```

`sleep` cannot return sooner than the **next tick boundary**, so a loop that has already spent
four ticks blocking on its own calls asks for — and gets — a fifth. It was not waiting to hit
20 Hz; it was already five times past 20 Hz and adding to the overrun. `os.queueEvent` +
`os.pullEvent` is a real yield (other coroutines run, the watchdog is satisfied) that can resume
within the **same** tick. When the loop is healthy this branch never runs.

`CFG.LOOP_PERIOD` and `CFG.SLOW_POLL_PERIOD` are **required knobs** and are in the boot canary,
so porting the fork onto #19/#27/#38 means adding them to those conf files too. The release
package has shipped the fork (#1's build, with #1's conf) since **2026-11-18**; its CHANGELOG
lists the knobs an upgrading install has to add. `LOOP_PERIOD` in particular must never read nil and be treated as 0: that
turns the pacing into a yield-only busy loop with no sleep at all.

What was **deliberately not done** — see [00-architecture.md](00-architecture.md) §7.

---

## 13. `uacconf.lua` — the map

| Section | Knobs |
|---|---|
| **PROTOCOLS** | 6 in, 4 out (`APNAV`, `ALTAP`, `AUTOLAND`, `APSWITCH`, `MANCON`, `MANNAV`, `KBDJOY` → `STAB`, `NAVCTRL`, `ALT`, `THRUSTF`) |
| **MASTER ENABLES** | `ENABLE_ATTITUDE/TRANSLATION/ALTITUDE` — bring the craft up one channel at a time |
| **manual** | `MANCON_*`, `MANNAV_*`, `MANCON_SCRAMBLE`, `MANCON_SCRAMBLE_OVERRIDE` |
| **attitude law** | `ATT_MODE`, `SENSOR_FRAME`, `PD_*`, `ATT_KP[_PITCH/_ROLL/_YAW]`, `RATE_KP[_YAW]`, `RATE_KD`, `ROT_GAIN_*`, `INERTIA_COUPLING`, `GYRO_*`, `TILT_YAW_*`, `YAW_TILT_DECOMP`, `TILT_ALT_ALLOC`, `VERT_XFER_*`, `YAW_TO_*_FF`, `ATT_INTEGRAL*`, `IDLE_*` |
| **signs** | `PITCH/ROLL/YAW_SIGN`, `SWAP_PITCH_ROLL_CHANNELS`, `*_RATE_SIGN`, `HEADING_SIGN/OFFSET` |
| **authority** | `STAB_MAX`, `YAW_MAX`, `THRUST_MAX`, `STRAFE_MAX` |
| **flight mode** | `FLIGHT_MODE`, `BANK_*`, `PITCH_ALT_*`, `FW_PITCH_*`, `HDG_SIM_*`, `HYBRID_STRAFE_*`, `STRAFE_EAGER_*` |
| **joystick** | `KJOY_*` |
| **translation** | `CRUISE_SPEED`, `APPROACH_KP/SPEED`, `TERMINAL_MAX_SPEED`, `TRANSLATE_*`, `SPEED_AT_FULL`, `STRAFE_AT_FULL`, `STEPS_PER_MPS_FWD/LAT`, `SPD_*`, `XTK_KP`, `STRAFE_*`, `TRANS_KP`, `*_SIGN`, `WP_EPSILON` |
| **altitude** | **`ALT_OFFSET`**, `HAS_ALT_THRUSTER`, `ALT_NEUTRAL/MIN/MAX/SWING`, `CLIMB_AT_FULL`, `STEPS_PER_MPS`, `ALT_KP`, `CLIMB_MAX`, `DESCEND_MAX`, `MANCON_CLIMB_MAX`, `MANCON_DESCEND_MAX` (fork), `VEL_*`, `MASS_STEP_THRESHOLD`, `TRIM_BOOST_*`, `AL_*` |
| **loop rate** | `LOOP_PERIOD`, `SLOW_POLL_PERIOD` (forked builds only) |
| **recorder** | `LOG_*` |
| **display** | `AH_TOP/HEIGHT`, `PITCH_PIXELS_PER_RAD`, `DISPLAY_PERIOD` |

The gains are expressed as **normalised** numbers times a measured authority wherever possible —
`STEPS_PER_MPS_FWD = THRUST_MAX / SPEED_AT_FULL`, `SPD_KP = SPD_KP_NORM · STEPS_PER_MPS_FWD`,
`STEPS_PER_MPS = ALT_SWING / CLIMB_AT_FULL`. That is what makes one set of `*_NORM` numbers
portable across airframes with very different authority.

---

## 14. Calibration order (empirical, and it is a sign/frame hunt, not a gain hunt)

> *A control loop that "blows up rather than settling" is a **sign/frame** error, not a gain.
> Suspect that first.* Run `pid/probe.lua` before anything, to confirm the quaternion API, the
> inertia tensor's shape and the frame of `getAngularVelocity()`.

1. **Levelling** — tilt by hand; it must right itself. A nose-up disturbance answered nose-up
   → flip `PITCH_SIGN`. Pitch corrections that move the craft in **roll** → the wrong thrusters →
   `SWAP_PITCH_ROLL_CHANNELS = true`.
2. **Invert** — flip it upside down; it must roll/pitch back to level.
3. **Yaw** — ENROUTE, command a target off to one side; the nose must swing **toward** it and
   settle **on** it. Runs away → flip `YAW_SIGN`.
4. **Translate** — `STRAFE_SIGN` / `THRUST_SIGN`.
5. **Altitude** — it learns its own hover trim; just confirm it climbs to a tapped ALT-AP altitude
   and holds.

Do **not** trust axis labels — the old `control.lua` and the draft's stab comments disagree about
which of pitch/roll is front/back vs left/right.

---

## 15. Display

An artificial horizon plus readouts, on the computer's own terminal, at `CFG.DISPLAY_PERIOD`. The
horizon is drawn in **runs**, not cells: each row of the picture is at most three horizontal runs
(sky, the crossing cell(s), ground), so it is built with `string.rep` and pushed as one `blit` —
18 terminal calls a frame instead of ~1400. Rows identical to the last frame are skipped. This is
a **cell**-resolution picture, not a 2×3 subpixel one, so there is no complement/swap business.

The MODE button (tap on the terminal or a monitor) cycles VTOL → HYBRID → FIXEDWING live. Levelling
and the slewed bank/pitch transitions absorb the switch, so it is safe in flight — but note it also
arms/disarms the keyboard joystick and the FIXEDWING vertical-by-pitch.

`ui.dtAvg` is the **measured** loop period, on screen permanently rather than in a debug build:
every gain assumes the loop got the rate it asked for, and a slow tick is dead time.
