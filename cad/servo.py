"""
Servo model — the self-sourced actuator the whole design is built around.
=========================================================================

Every leg/head/ear/tail joint is one hobby servo. Its physical envelope becomes
the servo *pocket* cut into the printed parts, and its **stall torque** is what
``analysis/validate.py`` checks each joint against. Because we self-source, this
is a single swappable data object — change ``DEFAULT`` (or pass another preset)
and the CAD pockets, mass budget, and torque checks all follow.

Torque is stored in SI (N·m) but presets are given in the kg·cm hobby servos are
sold in (1 kg·cm = 0.0980665 N·m).

Dimensions are millimetres (build123d convention).
"""

from __future__ import annotations

from dataclasses import dataclass

KGCM_TO_NM = 0.0980665


@dataclass(frozen=True)
class Servo:
    name: str
    body_l: float        # mm, along the output-shaft-perpendicular long axis
    body_w: float        # mm, thickness
    body_h: float        # mm, tall axis (to top of case, excl. horn)
    flange_l: float      # mm, total length across the mounting flanges
    flange_thk: float    # mm, mounting-tab thickness
    shaft_from_end: float  # mm, output-shaft axis offset from one body end
    horn_dia: float      # mm, servo horn disc diameter
    mass_kg: float       # kg, incl. horn + screws
    stall_kgcm: float    # kg·cm at nominal voltage
    speed_s_60: float    # s/60° (no-load)
    quiet: bool          # coreless/quiet? (servo-whine risk, PLAN §10)
    # noise-relevant traits (see docs/noise_reduction.md)
    control: str = "pwm"          # 'pwm' | 'serial' (current/pos) | 'foc'
    bus: str = "PWM (PCA9685)"    # how it's driven / wired
    backdrivable: bool = False    # torque/current control → compliant, silent hold, cat-safe
    noise: str = "med"            # subjective: 'very-low' | 'low' | 'med' | 'high'
    cost_usd: float = 15.0        # ~unit price

    @property
    def stall_nm(self) -> float:
        return self.stall_kgcm * KGCM_TO_NM

    @property
    def pocket(self) -> tuple[float, float, float]:
        """Envelope (l, w, h) to subtract from a part, with a small print clearance."""
        clr = 0.4
        return (self.body_l + 2 * clr, self.body_w + 2 * clr, self.body_h + clr)


# The coreless "mini" (Mini-Pupper-class) — kept as a preset (lightest/quietest-
# motion PWM option). It was the earlier default; superseded by the STS3215 below.
_CORELESS = Servo(
    name="coreless-20kg-mini", body_l=35.0, body_w=16.0, body_h=29.0,
    flange_l=48.0, flange_thk=2.6, shaft_from_end=10.0, horn_dia=20.0,
    mass_kg=0.035, stall_kgcm=20.0, speed_s_60=0.10, quiet=True,
    control="pwm", bus="PWM (PCA9685)", backdrivable=False, noise="low", cost_usd=15.0,
)

# Actuator candidates to weigh for noise (see analysis/actuator_compare.py).
# Noise fundamentally drops when the actuator is TORQUE/CURRENT-controllable
# (compliant, silent hold — no digital holding buzz) and coreless/FOC. The catch
# at kitten scale is torque-vs-mass: the quiet strong ones get heavy/big.
PRESETS = {
    "coreless-20kg-mini": _CORELESS,   # baseline: quiet coreless, PWM, light, cheap
    "mg996r": Servo("mg996r", 40.7, 19.7, 42.9, 54.0, 3.0, 11.0, 24.0,
                    0.055, 11.0, 0.17, quiet=False, control="pwm",
                    backdrivable=False, noise="high", cost_usd=6.0),  # cored, noisy
    # serial smart servos — current control → compliant, silent HOLD (fixes buzz
    # even while standing, not just grounded), daisy-chain (no PCA9685).
    "dynamixel_xl330": Servo("dynamixel_xl330", 20.5, 20.0, 34.0, 24.0, 2.0, 10.0, 14.0,
                             0.018, 5.3, 0.12, quiet=True, control="serial",
                             bus="TTL serial (daisy-chain)", backdrivable=True,
                             noise="low", cost_usd=25.0),   # ~0.52 N·m — light but weak
    "dynamixel_xm430": Servo("dynamixel_xm430", 28.5, 34.0, 46.5, 34.0, 3.0, 14.0, 24.0,
                             0.082, 41.0, 0.14, quiet=True, control="serial",
                             bus="TTL serial (daisy-chain)", backdrivable=True,
                             noise="low", cost_usd=50.0),   # ~4 N·m coreless — quiet but heavy
    "feetech_sts3215": Servo("feetech_sts3215", 45.0, 24.0, 36.0, 54.0, 3.0, 12.0, 20.0,
                             0.060, 30.0, 0.14, quiet=False, control="serial",
                             bus="TTL serial (daisy-chain)", backdrivable=True,
                             noise="med", cost_usd=15.0),   # ~2.9 N·m, cheap serial, metal gears
    # quasi-direct-drive BLDC + FOC — the quietest + fully backdrivable, but a
    # commercial QDD actuator is physically too big/heavy for a 1 kg kitten leg.
    "bldc_qdd": Servo("bldc_qdd", 50.0, 50.0, 30.0, 50.0, 4.0, 25.0, 40.0,
                      0.120, 61.0, 0.05, quiet=True, control="foc",
                      bus="CAN/UART (FOC driver)", backdrivable=True,
                      noise="very-low", cost_usd=90.0),
    # CUSTOM design (analysis/actuator_design.py): a compact gimbal BLDC + a
    # 3D-printed cycloidal 12:1 + magnetic encoder + FOC driver, sized for THIS
    # robot. ~63 g, ~2.8 N·m, very-low noise, backdrivable, printable gearbox —
    # the only option that hits quiet + strong + light + cat-safe at kitten scale.
    "custom_qdd": Servo("custom_qdd", 40.0, 40.0, 32.0, 40.0, 3.0, 20.0, 30.0,
                        0.063, 28.4, 0.077, quiet=True, control="foc",
                        bus="CAN/UART (SimpleFOC/custom)", backdrivable=True,
                        noise="very-low", cost_usd=39.0),
}

# FINALIZED actuator (2026-07-10): Feetech STS3215 serial bus servo. Chosen over the
# coreless mini because it (a) meets the SF-2 torque target — 2.9 N·m vs the ~1.16 N·m
# dynamic walk peak (SF 2.5); (b) is TORQUE/CURRENT-controlled → backdrivable, so it
# holds compliant + SILENT (no digital holding buzz) and is cat-safe on contact;
# (c) gives position FEEDBACK (useful for the four-bar knee transmission map + closed
# loop); (d) daisy-chains on one TTL serial bus → drops the PCA9685. Trade: +25 g/servo
# and 'med' gear noise while moving (the compliant silent HOLD is the bigger noise win
# for a companion pet). Swap back with `DEFAULT = PRESETS["coreless-20kg-mini"]`.
DEFAULT = PRESETS["feetech_sts3215"]
