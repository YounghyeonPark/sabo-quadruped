"""
Actuator trade study — which motor minimises noise for Sabo?
============================================================

    python -m analysis.actuator_compare

Noise drops most when the actuator is **torque/current-controllable** (compliant,
silent hold — no digital holding buzz) and coreless/FOC. But at kitten scale the
quiet strong options get heavy/big. This scores each candidate in `cad/servo.py`
on the three things that actually decide it: **torque feasibility, mass, noise**
(plus control mode, wiring, cost) so the trade-off is explicit.
"""

from __future__ import annotations

from cad import params as P
from cad.servo import DEFAULT, PRESETS

# Peak leg-joint torque the walk/trot need in sim (after the noise/gait tuning),
# and the safety factor we want over it.
REQ_PEAK_NM = 1.2
SAFETY = 1.5
NEED_STALL = REQ_PEAK_NM * SAFETY          # ≈1.8 N·m stall for margin

# non-servo mass (electronics + printed plastic) held constant across candidates
_NON_SERVO = P.component_mass_total() - P.N_SERVOS * DEFAULT.mass_kg
_PLASTIC = 0.43                             # frame ribcage + skin (kg), approx
MASS_LO, MASS_HI = P.MASS_TARGET


def main() -> None:
    print(f"Actuator trade study — {P.N_SERVOS} joints; need ≥{NEED_STALL:.1f} N·m "
          f"stall (peak {REQ_PEAK_NM} N·m × SF {SAFETY}); mass target "
          f"{MASS_LO:.1f}–{MASS_HI:.1f} kg\n")
    hdr = (f"{'actuator':20} {'stall':>6} {'torque':>7} {'14×g':>6} {'robot':>6} "
           f"{'mass':>5} {'noise':>8} {'control':>7} {'back':>5} {'$14':>5}")
    print(hdr); print("-" * len(hdr))
    for name, s in PRESETS.items():
        stall = s.stall_nm
        torque_ok = "OK" if stall >= NEED_STALL else "WEAK"
        servo14 = s.mass_kg * 14
        robot = _PLASTIC + _NON_SERVO + servo14
        mass_ok = "OK" if MASS_LO <= robot <= MASS_HI else ("HEAVY" if robot > MASS_HI else "lite")
        back = "yes" if s.backdrivable else "no"
        print(f"{name:20} {stall:5.2f}N {torque_ok:>7} {servo14*1000:5.0f} "
              f"{robot:5.2f}kg {mass_ok:>5} {s.noise:>8} {s.control:>7} {back:>5} "
              f"${s.cost_usd*14:>4.0f}")
    print("\nRead: 'torque' must be OK (stall ≥ margin); 'mass' must not be HEAVY; "
          "backdrivable + serial/foc control = compliant SILENT hold (kills holding buzz).")
    print("""
Verdict for a ~1 kg kitten:
  • coreless-20kg-mini (current default) — OK torque, lightest, quiet coreless,
    cheap. Holding buzz handled by relax-at-rest + soft mounts + fur. Best
    torque-to-weight at this scale → keep as the baseline.
  • feetech_sts3215 — torque headroom + CURRENT control (compliant, silent hold
    even while standing) + serial daisy-chain (no PCA9685) + cheap. Costs ~+0.35 kg
    and its metal gears are only 'med' noise. Pick this if silent standing-hold
    matters more than mass.
  • dynamixel_xm430 — quietest coreless + current control, but 14× is too HEAVY for
    a kitten (blows the mass target) and pricey.
  • dynamixel_xl330 — lovely + light + quiet, but too WEAK (~0.5 N·m) for the legs.
  • bldc_qdd — the truly silent, fully-backdrivable ideal, but a QDD actuator is
    physically too big/heavy for a 1 kg kitten leg.

Recommendation: keep the coreless-mini for the LEGS (torque-to-weight wins) and
lean on the software/mechanical noise fixes; OR, if you want compliant silent hold
while standing, switch to feetech_sts3215 (accept ~1.5 kg + a serial-bus rewire).
Swap by setting cad/servo.py DEFAULT, then re-run analysis/validate + the gaits.
""")


if __name__ == "__main__":
    main()
