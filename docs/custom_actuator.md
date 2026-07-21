# Sabo — Custom Actuator Design (quiet QDD)

Off-the-shelf servos can't hit **quiet + strong + light + backdrivable** at kitten
scale (trade study: `analysis/actuator_compare.py` — the quiet strong ones are too
heavy, the light ones too weak). A **custom quasi-direct-drive (QDD)** module,
sized for exactly this robot, is the only thing that lands all four — and it fits
the self-source/3D-print ethos because the gearbox is printed.

## Architecture
```
compact BLDC (gimbal/pancake) ─▶ 3D-printed cycloidal reducer (~12:1)
     ─▶ magnetic encoder (on-axis) ─▶ FOC driver ─▶ (CAN/UART bus)
```
- **BLDC + FOC** — sinusoidal commutation: no brushed/gear PWM whine, smooth.
- **Cycloidal reducer, single stage, low-ish ratio** — compact + high torque
  density + tolerant of printed tolerances; **printed on Sabo's own printer**.
- **Magnetic encoder** (AS5047/MT6701) — closes the FOC + position loop.
- **Current/torque control** — the big noise win: a *compliant, silent hold*
  (no digital-servo holding buzz, even standing) and a **backdrivable** joint that
  yields softly if the cat paws it (safe + quiet).

## Sizing (from `analysis/actuator_design.py`)
Sized against the sim's peak leg torque (1.2 N·m × 1.5 SF = 1.8 N·m needed):

| | value |
|---|---|
| Motor | compact gimbal BLDC, Kv ≈ 140 → Kt ≈ 0.068 N·m/A |
| Drive | FOC, 4 A phase, 3S (11.1 V) |
| Reducer | printed cycloidal **12:1**, η ≈ 0.85 |
| **Peak torque** | **2.8 N·m** (need ≥1.8 ✓) |
| No-load speed | 13.6 rad/s (need ≥8 ✓) |
| **Mass** | **63 g/joint** |
| Cost | ~$39/joint |

vs off-the-shelf: lighter than a Dynamixel XM430 (82 g) or a commercial QDD
(120 g), while quiet + backdrivable. In the trade table it's the **only** row that
is torque-OK **and** mass-OK **and** very-low-noise **and** backdrivable.
`cad/servo.py` → `PRESETS['custom_qdd']`.

## Per-actuator BOM (~$39)
| Item | ~$ |
|---|---|
| Compact gimbal BLDC (low-Kv) | 12 |
| Magnetic encoder (AS5047/MT6701) + diametric magnet | 6 |
| FOC driver (SimpleFOC ESP32/STM32 or a custom board) | 15 |
| Bearings + hardware | 5 |
| Printed cycloidal gearbox (filament) | ~1 |

## Control interface — direct torque & speed (the QDD payoff)
Unlike a PWM hobby servo (position command only; torque is just "whatever it takes
to hold"), a FOC driver lets you command **torque, speed, OR position directly**:

- **Torque = current, directly:** `τ = Kt · Iq` — set the q-axis current and you
  set the torque. FOC generates the 3-phase voltages (SVPWM) to hold that current.
- **Cascaded loops:** position → velocity → **current(torque)**. Command at any
  level (torque / velocity / position mode).
- **Signal form:** a **digital bus command (CAN/UART/SPI)** to the driver
  (analog torque command is possible but rare). moteus/ODrive/SimpleFOC accept a
  combined command: `pos_target, vel_target, torque_ff, Kp, Kd`.
- **Nuance:** at any instant only *one* of {torque, velocity, position} is the
  reference; the legged-robot standard is **impedance control** —
  `τ = Kp·(θd−θ) + Kd·(ωd−ω) + τ_ff` — "go to this angle, at this speed, with this
  much force/compliance," expressed as a single torque command computed in software.

**Implications for Sabo:** `Body.relax()` → literal **zero-torque** (silent +
backdrivable/cat-safe); silent compliant hold while standing; the RL policy can
output **torque/impedance** (common in quadruped RL) instead of just joint angles,
for softer contact/impacts. The HAL can stay position-based, or gain
`set_torque()` / `set_impedance()` verbs to use FOC fully.

## Electronics change (the cost of going custom)
QDD replaces the single **PCA9685 PWM** bus with **one FOC driver per joint** on a
**CAN/UART** bus (e.g. SimpleFOC, or a multi-axis FOC board). That's more wiring,
firmware, and power electronics than PWM servos. The **HAL doesn't change**:
`HardwareBody` maps the same verbs onto FOC current/position commands instead of
PCA9685 duty; `Body.relax()` becomes a true **zero-torque compliant** mode (even
better than cutting PWM).

## Mass tip — mix actuator classes
14× custom QDD ≈ 0.88 kg puts the robot at ~1.6 kg (top of target). Smarter: put
**QDD only on the 8 load/motion joints (hip+knee)** where torque + quiet + compliance
matter, and keep **tiny light servos on the 6 expression joints** (head ×3, ears,
tail, waist). That drops actuator mass to ~0.6 kg → robot ~1.35 kg, and only the
joints the cat feels/hears most are the premium quiet ones.

## Effort & risk — this is a real sub-project
Designing a custom actuator is months of R&D, not an afternoon: motor sourcing +
characterisation, cycloidal design → print → tolerance/backlash iteration, encoder
alignment, **per-motor FOC tuning**, driver-board bring-up, thermal, and CAN
plumbing ×(8–14). High reward (the ideal quiet, safe, compliant joint; printable)
but high effort and schedule risk.

## Recommendation / phasing
- **v1 (get it walking):** off-the-shelf **coreless-mini** (current default) +
  the software/mechanical noise fixes (relax-at-rest, soft mounts, fur, smooth
  motion). Lightest, cheapest, proven — don't block the first robot on actuator R&D.
- **v2 (quiet upgrade):** develop the **custom QDD** on the **hip+knee** joints.
  It's the endgame for a genuinely silent, cat-safe, backdrivable Sabo, and it's
  self-printable. Prototype ONE actuator first, characterise it, then commit.

To evaluate it in the full pipeline: set `cad/servo.py` `DEFAULT = PRESETS["custom_qdd"]`
and re-run `analysis/validate` + the gaits (mass/torque recompute automatically).
Default stays coreless for now.
