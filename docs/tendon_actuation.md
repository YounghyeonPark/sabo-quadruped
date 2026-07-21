# Sabo — Tendon / Cable-Driven Actuation (analysis)

An alternative to a motor-at-every-joint design: put the motors in the **torso**
and drive the leg joints remotely through **cables (tendons)** — exactly how a
real cat works (muscle mass proximal, thin tendons pulling the distal joints/paw).
Sabo's knee→hock mechanical coupling is already a small step in this direction (a
"reciprocal-apparatus tendon"); this is the full version.

## Why it fits a cat especially well
- **Ultra-light legs (the big win).** Motors move off the limbs into the body, so
  the legs are just structure + cables + pulleys. Low distal inertia →
  **faster, more agile, cat-like** motion, lower dynamic gait torque, and softer
  foot impacts (less impact clatter). Cats have light legs for exactly this reason.
- **Quieter — and it helps *our* noise problem.** The whining motors are
  **centralised in the torso**, where they can be enclosed in a padded/damped bay
  and muffled by the body shell + fur. The moving limbs themselves are near-silent
  (just cable + pulley). The noise source is one place, easy to soundproof —
  vs. motors on the legs radiating from resonant structures.
- **Biomimetic + compliant.** Cable + a little series spring gives natural
  compliance (absorbs pounces, safe for the cat), and it reads as organic motion.
- **Motors protected** from pounces/impacts (they're in the body, not exposed).

## The costs (honest)
- **Friction & backlash.** Bowden sheaths especially have stiction/hysteresis;
  cables stretch. This fights the precision the IK / RL gait wants — the hardest
  problem. Routing over **pulleys** (not sheaths) cuts friction but complicates layout.
- **Cables only pull.** Each rotational DOF needs either **antagonistic pairs**
  (2 cables → 2 motors or a differential; stiff, controllable) **or 1 cable +
  return spring** (1 motor, but weaker/compliant in the return direction). So you
  either add motors or give up stiffness in one direction — a real design cost for
  a weight-bearing leg.
- **Maintenance/reliability.** Cables fray, slip, need re-tensioning.
- **Control is nonlinear** (friction, stretch, routing) — harder than FOC/QDD.

## How it would restructure Sabo
- **Motors → torso** (in a damped, fur-covered bay), cables routed through the
  hip into the leg over pulleys to the knee (and via the existing coupling to the
  hock). Legs become light printed structure + guides.
- **Per joint:** either antagonistic 2-cable (best control, more motors) or
  1-cable + spring return (fewer motors, compliant). For a load-bearing hip/knee,
  actuate the weight-bearing (extension) pull; let flexion be spring/gravity.
- **HAL unchanged:** still position/torque setpoints; the torso motors + cable
  kinematics live below the HAL, like the gait engine does now.

## Recommended shape for Sabo — a hybrid
Don't go fully tendon everywhere (control risk). The sweet spot mirrors real cat
anatomy:

> **Motor at the hip (proximal), tendon-drive the distal joints (knee → hock),
> with a return spring + the existing knee→hock coupling.**

That makes the *lower* leg light and silent (where impact noise + inertia hurt
most), keeps the controllable actuator proximal, and is the most cat-authentic —
without full-body cable complexity.

## Tendon vs. the other actuator options
| Criterion | Coreless (v1) | Custom QDD | **Tendon (hybrid)** |
|---|---|---|---|
| Leg inertia / agility | med | **worst** (heavy joint) | **best** (light limb) |
| Joint/limb noise | med | low | **very low** (motors in body) |
| Motor noise dampable | hard (on legs) | hard | **easy** (centralised, enclosed) |
| Control precision | good | **best** (FOC) | **worst** (friction/backlash) |
| Compliance / cat-safe | via relax | **best** (torque ctrl) | good (cable+spring) |
| Mechanical complexity | low | med (electronics) | **high** (routing/tensioning) |
| Biomimetic | some (coupled hock) | some | **most** (real tendons) |

## Better than cables at the joint: RIGID-LINK transmission ⭐
Instead of a cable over pulleys, transmit the motor's motion with **rigid links**
(a four-bar / pushrod / parallel linkage). This keeps the motor proximal (light
distal limb — the whole point) but **removes the cable-friction problems**:

- **No capstan friction, no stretch, no hysteresis, no stick-slip.** Force goes
  through pin joints (tiny, ~constant bearing friction), not a cable wrapped on a
  drum. The transmission is deterministic.
- **Bidirectional — push AND pull.** A rigid rod drives the joint both ways, so
  **one motor per joint, no antagonist pair / return spring** (this was the cable's
  worst drawback — solved).
- **Known, invertible kinematics.** The linkage geometry gives an exact (if
  nonlinear) motor-angle → joint-angle map, so the IK/gait/RL know the real joint
  state without a joint-side sensor. Precision restored.
- **Efficient + quiet.** No friction heat loss, no cable squeak; motors still
  centralised/proximal so their whine is dampable.

This is exactly how serious quadrupeds drive the knee (e.g. MIT Cheetah-class:
**coaxial hip+knee actuators, a rigid pushrod/parallel linkage to the knee**, so
the shank is light). Sabo's existing **knee→hock coupling is already a rigid-link
coupling** — this just extends that philosophy to the knee drive.

**Costs (honest):**
- **Nonlinear transmission ratio** — the mechanical advantage varies over the
  joint range. It's a *known geometric* map (unlike cable friction), so you bake
  it into the IK; but it must be modelled.
- **Range-of-motion limits / singularities** — the linkage can lock or lose
  authority near singular poses; design the geometry so the leg's working range
  stays clear of them.
- **Pin backlash** — several pin joints stack clearance; mitigate with good
  bearings/preload. Still far tighter than a cable.
- **More parts / a little mass** — rods + extra pins/bearings (rods can be light
  CFRP/printed).

**For Sabo:** cluster the hip + knee motors proximally (coaxial at the hip, or in
the torso) and drive the knee via a rigid **pushrod/four-bar**, with the existing
coupling carrying the hock. Result: a **light lower leg, no cable friction,
bidirectional, precise** — arguably the best transmission for Sabo, and more
robust than tendons.

| | Cable/tendon | **Rigid link (4-bar/pushrod)** |
|---|---|---|
| Distal-limb mass | best | **best** (motor still proximal) |
| Friction/hysteresis | bad (capstan, stick-slip) | **minimal** (pin joints) |
| Push + pull | no (needs pair/spring) | **yes, 1 motor** |
| Kinematics known | no (friction) | **yes** (geometry) |
| Range of motion | large | limited (design around singularities) |
| Backlash | cable stretch | pin clearance (tighter) |

## Verdict / phasing
- **v1:** coreless direct-drive — proven, get it walking. (current default)
- **v2 quiet/agile upgrade:** keep motors proximal for a light, quiet lower leg —
  but prefer **RIGID-LINK transmission (pushrod/four-bar) over cables**: same
  light-limb benefit, none of the cable-friction risk, bidirectional with one
  motor, and deterministic kinematics. This is the recommended path (and how
  MIT-Cheetah-class legs work). Cables only if you specifically want the
  soft/compliant tendon feel.
- Prototype ONE leg either way and characterise its transmission (linkage
  backlash/ROM, or cable friction) before committing.

> Not modelled in the sim yet (cable friction/compliance would need a tendon model
> — MuJoCo *does* support tendons/`spatial` routing, so a future `sim/mjcf.py`
> could add cable tendons if we pursue this).
