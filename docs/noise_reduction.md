# Sabo — Motor / Servo Noise Reduction

Servo whine is the top behavioral risk (PLAN §10): a cat reads a high-pitched
mechanical whine as *wrong* and may avoid the robot for good. Sabo has 14 servos,
so quiet operation is a first-class design goal, not a polish item. Noise comes
from three places — **the motor/gears**, **holding buzz** (a digital servo hunting
while it holds a load), and **structure-borne vibration** (the shell acting as a
soundboard) — plus how *loud it feels* depends on the gait/behavior. Levers, by
impact:

## 1. Pick a quiet actuator (biggest lever)
- **Coreless / brushless motor**, not cheap cored (brushed): far less high-freq
  whine. Sabo's `cad/servo.py` default is already `quiet=True` coreless.
- **Nylon/POM gears** over metal where torque allows — metal gear-mesh whines;
  the trade is durability. Good grease on the gear train also drops mesh noise.
- **Digital vs analog:** digital holds precisely but *buzzes* under load (see §2);
  analog is quieter at rest but sloppier. Sabo uses digital + the §2 fix.
- Premium option: small **quasi-direct-drive BLDC** actuators are the quietest of
  all (robot-dog grade) — heavier/pricier, likely overkill at kitten scale.

## 2. Kill holding buzz — "go silent at rest" ⭐ (implemented)
A digital servo commanded to *hold* a loaded joint constantly micro-corrects →
audible buzz. For a companion that mostly **stands and watches**, this is the
noise the cat hears most — and it happens exactly when the cat is nearby.

Fixes, in order:
- **Relax when resting.** In grounded rest poses (sit / loaf / curl-sleep, and the
  low **watch** pose) the body weight sits on the frame/ground, so the leg servos
  don't need to hold torque — cut their PWM and they go **limp and silent**.
  Implemented: `Body.relax(on)` in the HAL → `HardwareBody.relax()` zeroes the 8
  leg channels' duty (unpowered = no buzz); the brain calls it in the SLEEPY/curl
  rest state, so Sabo is **fully silent while resting**. The low **watch** pose is
  only lightly loaded (already quiet); making watch a grounded **sit-and-watch**
  would extend full servo-off silence to the cat-detection moment too — the
  natural next step.
- **Low static load.** The validated design holds only ~17–27 % of stall at
  stance, and a lower/settled CoM (from the dimensional optimization) means less
  holding effort → less buzz even when it must hold.
- **Widen the deadband** on the position servo so it stops hunting tiny errors
  (trade a little precision for silence).

## 3. Smooth the motion (whine while moving) — done
- **Slew-rate limiting + soft-start** (already in `sim/mj_emulate.py`): targets
  ramp instead of step-changing, so no sudden current spikes / whine transients.
- **Move slowly, pause often.** The cat-like behavior (slow creep, watch, freeze)
  is *also* the quiet behavior. Servos are quietest at low speed.
- **Gentle gaits.** The low step-height, IMU-levelled walk avoids hard foot
  impacts (impact clatter) and structural resonance.

## 4. Damp structure-borne vibration
- **Soft-mount the servos:** seat each servo in the frame on **TPU / silicone
  grommets** (or a rubber gasket at the mounting tabs) so vibration doesn't couple
  into the shell. (CAD TODO: add compliant pockets at the leg-mount nodes.)
- **The fur/silicone over-skin** (PLAN §3.3) is a genuine acoustic muffler — a
  furred cat is much quieter than bare plastic; it absorbs the high frequencies
  ears (and cats) notice most.
- **Don't let the shell ring:** avoid large flat panels that radiate sound; the
  curved skin + a little foam in cavities kills panel resonance.
- **Reduce backlash:** loose gear trains rattle on direction changes; pick
  low-backlash servos / preload the joints.

## 5. Use behavior + sound to mask what's left
- **Silent watch/detect.** Because vision runs in the still watch pose (§2 relax),
  the camera sees the cat while the robot makes *no* motor sound.
- **Purr as masking.** The purr (vibration motor + low speaker, PLAN §5.1) is a
  sound the cat *expects* and it masks residual servo hum — a bug turned feature.
- **Introduce gradually** so the cat habituates (PLAN §10).

## 6. Electrical
- Clean, well-regulated servo rail (the separate 6 V buck + bulk cap) — brownout
  stutter adds noise. A stable supply keeps the motor drive smooth.

## What's implemented vs. mechanical/hardware TODO
- **Implemented (software):** slew-rate + soft-start (motion), and **`relax()`
  silent-rest** through the HAL → hardware backend + brain rest state.
- **Hardware/mechanical (build-time):** coreless servo choice (default set),
  TPU servo grommets (CAD), fur over-skin, gear grease, deadband tuning on the
  real servos.

> The sim can't measure acoustics, so noise is addressed at the *source*
> (actuator choice, relax-at-rest, smooth motion, damping) rather than verified by
> a dB meter. The single highest-value move for a cat companion is **§2: be silent
> whenever still** — done in software here, and it's exactly when the cat is close.
