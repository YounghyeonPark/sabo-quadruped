"""
Custom actuator design — sizing a bespoke quiet QDD for Sabo.
=============================================================

    python -m analysis.actuator_design

Off-the-shelf servos can't hit quiet + strong + light + backdrivable at kitten
scale (see analysis/actuator_compare.py). A **custom quasi-direct-drive (QDD)**
module can, because we size it for exactly this niche:

    small BLDC (gimbal/pancake) → 3D-printed cycloidal reducer → magnetic encoder
    → FOC driver

Why quiet: FOC sinusoidal commutation (no cogging/PWM whine), a low-ish single-
stage cycloidal ratio, and — the big one — **current/torque control** gives a
compliant, *silent* hold (no digital-servo holding buzz) and a backdrivable,
cat-safe joint. The cycloidal gearbox is **printed** (fits self-source/print).

This script sizes the module from motor + gear numbers and checks it against the
joint requirement from the sim, then emits the `custom_qdd` spec used in
`cad/servo.py`.
"""

from __future__ import annotations

import math

# ---- requirement (from the sim gaits + a safety factor) ----------------------
REQ_PEAK_NM = 1.2         # peak leg-joint torque in walk/trot (tuned sim)
SAFETY = 1.5
NEED_TORQUE = REQ_PEAK_NM * SAFETY          # ≈1.8 N·m
NEED_SPEED_RADS = 8.0     # comfortable output speed for the gait (we use ~1–3)

# ---- design point (a compact gimbal BLDC + printed cycloidal) ----------------
DESIGN = dict(
    motor="compact gimbal BLDC (~Ø35)",
    Kv_rpm_per_V=140.0,   # low-Kv gimbal motor = high torque/amp
    I_peak_A=4.0,         # FOC phase current limit
    gear_ratio=12.0,      # single-stage 3D-printed cycloidal
    gear_eff=0.85,        # cycloidal efficiency
    supply_V=11.1,        # 3S LiPo
    # module mass budget (kg)
    m_motor=0.030, m_gearbox=0.016, m_encoder=0.004, m_bearing=0.005, m_driver=0.008,
    # per-actuator cost (USD)
    c_motor=12, c_gearbox_filament=1, c_encoder=6, c_bearings_magnets=5, c_driver=15,
)


def size(d=DESIGN) -> dict:
    Kt = 9.5493 / d["Kv_rpm_per_V"]                 # N·m/A  (Kt = 60/(2π·Kv))
    motor_torque = Kt * d["I_peak_A"]               # N·m at the motor
    out_torque = motor_torque * d["gear_ratio"] * d["gear_eff"]
    motor_noload_rpm = d["Kv_rpm_per_V"] * d["supply_V"]
    out_speed_rads = (motor_noload_rpm / d["gear_ratio"]) * 2 * math.pi / 60.0
    mass = d["m_motor"] + d["m_gearbox"] + d["m_encoder"] + d["m_bearing"] + d["m_driver"]
    cost = (d["c_motor"] + d["c_gearbox_filament"] + d["c_encoder"]
            + d["c_bearings_magnets"] + d["c_driver"])
    return dict(Kt=Kt, motor_torque=motor_torque, out_torque=out_torque,
                out_speed_rads=out_speed_rads, mass=mass, cost=cost)


def main() -> None:
    r = size()
    print("Custom QDD actuator — sizing\n" + "=" * 40)
    print(f"  motor            : {DESIGN['motor']}, Kv={DESIGN['Kv_rpm_per_V']:.0f} "
          f"→ Kt={r['Kt']:.4f} N·m/A")
    print(f"  drive            : FOC, I_peak={DESIGN['I_peak_A']:.1f} A, "
          f"{DESIGN['supply_V']:.1f} V (3S)")
    print(f"  reducer          : printed cycloidal {DESIGN['gear_ratio']:.0f}:1 "
          f"(η={DESIGN['gear_eff']:.2f})")
    print(f"  → peak torque    : {r['out_torque']:.2f} N·m   "
          f"(need ≥{NEED_TORQUE:.1f})  {'OK' if r['out_torque']>=NEED_TORQUE else 'SHORT'}")
    print(f"  → no-load speed  : {r['out_speed_rads']:.1f} rad/s "
          f"(need ≥{NEED_SPEED_RADS:.0f})  {'OK' if r['out_speed_rads']>=NEED_SPEED_RADS else 'SLOW'}")
    print(f"  → module mass    : {r['mass']*1000:.0f} g/joint  "
          f"(×14 = {r['mass']*14:.2f} kg of actuators)")
    print(f"  → cost           : ${r['cost']:.0f}/joint  (×14 = ${r['cost']*14:.0f})")
    print(f"\n  Robot total est. : ~{0.43 + 0.30 + r['mass']*14:.2f} kg "
          f"(plastic+electronics+actuators)")
    print("""
  Traits: FOC + cycloidal + CURRENT control → very-low noise, SILENT compliant
  hold (no holding buzz), backdrivable (cat-safe). Gearbox is 3D-printed.
  Electronics: replaces the PCA9685 PWM bus with a FOC driver per joint on a
  CAN/UART bus (e.g. SimpleFOC / a custom board). This is a real R&D sub-project
  (motor sourcing, gearbox print+tolerance, encoder alignment, per-motor FOC
  tuning) — see docs/custom_actuator.md. Emitted as cad/servo.py PRESETS['custom_qdd'].
  Sensitivity: ↑gear_ratio = ↑torque but ↓backdrivability + ↑gear noise; keep it
  as low as the torque requirement allows.""")


if __name__ == "__main__":
    main()
