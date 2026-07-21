"""
Bill of Materials + cost estimate for a self-sourced Sabo build.
================================================================

    python -m analysis.bom            # prints a markdown BOM + cost range
    python -m analysis.bom > docs/BOM.md

Prices are typical hobby/maker USD (≈2025–26), given as low–high because
individual parts swing a lot by vendor/region. Printed-part filament cost is
computed from the actual CAD mass (frame from ``cad/out/parts_manifest.json`` +
the cosmetic skin), so it tracks the current geometry. This is an ESTIMATE for
planning — not a live-priced cart. Excludes one-time tools you likely own
(3D printer, soldering iron) which are listed separately.
"""

from __future__ import annotations

import json
import os

from cad import params as P

FILAMENT_USD_PER_KG = (20.0, 28.0)     # PLA/PETG spool
PRINT_WASTE = 1.18                      # supports + purge + failed-print margin

# category, item, qty, unit_low, unit_high, note
COMPONENTS = [
    # ---- compute / edge-AI ----
    ("Compute", "Jetson Orin Nano Super Dev Kit (8 GB)", 1, 249, 249,
     "on-device AI brain; incl. carrier+cooling. (module+compact carrier ~similar)"),
    ("Compute", "NVMe SSD 256 GB (M.2)", 1, 22, 35, "OS + models"),
    ("Compute", "Wi-Fi/BT M.2 card (AX210)", 1, 15, 22, "app link / telemetry"),
    # ---- vision + sensors ----
    ("Sensors", "Wide-FOV CSI camera (IMX219, ~120°)", 2, 25, 35, "eyes — stereo pair"),
    ("Sensors", "BNO085 IMU (fusion)", 1, 15, 25, "inner ear — balance + gimbal + EIS"),
    ("Sensors", "VL53L1X ToF distance", 2, 10, 16, "nose (fwd) + chin (cliff)"),
    ("Sensors", "I2S MEMS mic (ICS-43434)", 2, 6, 9, "ears — stereo hearing"),
    ("Sensors", "BME688 gas/VOC e-nose", 1, 18, 28, "nose — scent classifier"),
    ("Sensors", "MAX98357A I2S amp", 1, 5, 8, "mouth — audio out"),
    ("Sensors", "Mini speaker 8Ω", 1, 2, 5, "mouth — meow/trill/TTS"),
    # ---- actuators ----
    ("Actuators", "Feetech STS3215 serial bus servo (30 kg·cm)", P.N_SERVOS, 14, 18,
     f"{P.N_SERVOS} joints: 8 leg + waist + head pan/pitch/tilt + ears + tail; "
     "TTL daisy-chain, position feedback, torque control"),
    ("Actuators", "TTL bus servo adapter (Waveshare / FE-URT-1)", 1, 5, 12,
     "UART↔half-duplex TTL bus for the STS3215 chain (replaces PCA9685)"),
    ("Actuators", "LED-eye driver (MOSFET + eye LEDs)", 1, 2, 6,
     "eyes off the servo bus → Jetson hardware-PWM pin + MOSFET"),
    # ---- power ----
    ("Power", "3S LiPo 5000 mAh", 1, 25, 40, "~1.5–2 h active"),
    ("Power", "Buck 5 V/5 A (Jetson rail)", 1, 8, 14, ""),
    ("Power", "Buck/BEC 7.4 V/≥15 A (STS3215 bus rail)", 1, 12, 22,
     "separate rail + bulk cap; sized for realistic simultaneous servo current"),
    ("Power", "Bulk cap + XT60 + wiring/connectors", 1, 15, 30, "power distribution"),
    # ---- mechanical / fasteners ----
    ("Mechanical", "M2/M3 screws + heat-set inserts", 1, 10, 18, "assembly"),
    ("Mechanical", "Servo horns / pins / small bearings", 1, 12, 25, "joint hardware"),
    ("Mechanical", "TPU for foot pads", 1, 5, 10, "grippy toe caps"),
    ("Mechanical", "Faux-fur / silicone skin (optional)", 1, 0, 25,
     "cosmetic over-skin, PLAN §3.3 (optional)"),
]

ONE_TIME_TOOLS = [
    ("LiPo balance charger", 20, 40),
    ("3D printer", 0, 0),        # assumed owned
    ("Soldering iron + supplies", 0, 0),
]


def _printed_grams() -> float:
    """Frame plastic from the CAD manifest + the cosmetic skin, with waste."""
    frame = 0.0
    mpath = os.path.join(os.path.dirname(__file__), "..", "cad", "out", "parts_manifest.json")
    try:
        frame = json.load(open(mpath))["totals"]["printed_plastic_g"]
    except Exception:
        frame = 275.0
    skin = 0.0
    try:
        from cad.parts.shell import (body_shell_aft, body_shell_fore, head_shell,
                                     waist_collar)
        for part, n in ((body_shell_fore(), 1), (body_shell_aft(), 1),
                        (head_shell(), 1), (waist_collar(), 1)):
            skin += part.volume * 1e-9 * P.EFFECTIVE_DENSITY * 1000.0
    except Exception:
        skin = 150.0
    return (frame + skin) * PRINT_WASTE


def build_rows() -> tuple[list, float]:
    """The full BOM rows (components + the CAD-mass-derived filament line) and the
    computed printed-filament grams. Shared by ``main`` and by ``analysis.platform_spec``
    so the spec sheet's unit-cost row can never drift from the printed BOM."""
    grams = _printed_grams()
    fil_low = grams / 1000.0 * FILAMENT_USD_PER_KG[0]
    fil_high = grams / 1000.0 * FILAMENT_USD_PER_KG[1]
    rows = list(COMPONENTS)
    rows.append(("Mechanical", f"3D-print filament (~{grams:.0f} g PLA/PETG)", 1,
                 round(fil_low, 1), round(fil_high, 1), "computed from CAD mass"))
    return rows, grams


def cost_totals() -> dict:
    """Whole-robot build cost (USD) low/mid/high, derived from ``build_rows``.
    The single source for the unit-cost figure quoted in the platform spec sheet."""
    rows, grams = build_rows()
    lo = sum(lo * qty for _, _, qty, lo, hi, _ in rows)
    hi = sum(hi * qty for _, _, qty, lo, hi, _ in rows)
    return {"low": lo, "mid": (lo + hi) / 2.0, "high": hi, "filament_g": grams}


def main() -> None:
    # Windows consoles default to cp1252, which can't encode the unicode in the
    # rows (e.g. the "8Ω" speaker) and crashes the print. Force UTF-8 stdout.
    import sys
    try:
        sys.stdout.reconfigure(encoding="utf-8")
    except (AttributeError, ValueError):
        pass

    rows, grams = build_rows()

    print("# Sabo — Bill of Materials & Cost Estimate\n")
    print("Self-sourced build. Prices = typical maker USD (~2025–26), low–high "
          "(vendor/region vary). Estimate for planning, not a live cart.\n")

    cats, lo_tot, hi_tot = {}, 0.0, 0.0
    for cat, item, qty, lo, hi, note in rows:
        cats.setdefault(cat, []).append((item, qty, lo, hi, note))
    order = ["Compute", "Sensors", "Actuators", "Power", "Mechanical"]
    for cat in order:
        print(f"## {cat}\n")
        print("| Item | Qty | Unit $ (lo–hi) | Subtotal $ | Note |")
        print("|---|--:|--:|--:|---|")
        clo = chi = 0.0
        for item, qty, lo, hi, note in cats[cat]:
            slo, shi = lo * qty, hi * qty
            clo += slo; chi += shi
            print(f"| {item} | {qty} | {lo:.0f}–{hi:.0f} | {slo:.0f}–{shi:.0f} | {note} |")
        lo_tot += clo; hi_tot += chi
        print(f"| **{cat} subtotal** | | | **{clo:.0f}–{chi:.0f}** | |\n")

    mid = (lo_tot + hi_tot) / 2
    print("## Total (one robot)\n")
    print(f"| | Low | Mid | High |")
    print(f"|---|--:|--:|--:|")
    print(f"| **Build cost (USD)** | **${lo_tot:.0f}** | **${mid:.0f}** | **${hi_tot:.0f}** |\n")

    tlo = sum(l for _, l, _ in ONE_TIME_TOOLS)
    thi = sum(h for _, _, h in ONE_TIME_TOOLS)
    print("### One-time tools (excluded from build cost)\n")
    for name, lo, hi in ONE_TIME_TOOLS:
        owned = " (assumed owned)" if hi == 0 else ""
        print(f"- {name}: ${lo:.0f}–{hi:.0f}{owned}")
    print(f"\n_Cost drivers: {P.N_SERVOS}× STS3215 servos and the Jetson dominate (~"
          f"{(P.N_SERVOS * 16 + 249) / mid * 100:.0f}% of a mid build). Cutting servo "
          f"count or grade, or a cheaper SBC, moves the total most._")


if __name__ == "__main__":
    main()
