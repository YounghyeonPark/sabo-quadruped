"""
Platform benchmark — the sim-measurable performance metrics + comparison scaffold.
==================================================================================

    python -m analysis.benchmark        # run gaits, aggregate, write docs/out/

Runs the existing MuJoCo gaits (``sim.mj_emulate.simulate``, ``sim.gait`` presets —
NOT reimplemented) and aggregates the metrics a paper can honestly report from
simulation alone:

  * per-gait (stand / walk / trot): stays-upright, torso roll/pitch p-p, travel,
    peak leg torque as % of servo stall (headroom), camera-shake;
  * mechanism: four-bar knee ROM + transmission-angle band + monotonicity, and the
    remote-axle hip lateral-inertia reduction (from ``platform_spec``);
  * a DOF-sharing count (dual-use joints — a design-efficiency metric);
  * a comparison-table scaffold vs named baselines (Mini Pupper / Mini-Cheetah /
    Solo) — Sabo's column is filled from our data; baseline cells are ``[cite]``
    placeholders (we do NOT invent competitor numbers);
  * the paper's hardware-measured claims (noise dB, backlash, backdrive torque,
    runtime, sim-to-real gap) as explicit ``TBD (hardware)`` slots — pending, not faked.

Deterministic + headless: renders are skipped here (metrics only); the 3-D gifs come
from ``sim.mj_emulate``. Writes ``docs/out/benchmark.md`` + ``benchmark.json``.
"""

from __future__ import annotations

import json
import os

import numpy as np

from cad.servo import DEFAULT as SERVO
from sim import gait
from sim.mj_emulate import simulate
from analysis import platform_spec as spec

OUT = os.path.join(os.path.dirname(__file__), "..", "docs", "out")

GAITS = ["stand", "walk", "trot"]
SECONDS = 6.0

# Baselines named in the platform paper. Cells left as [cite] on purpose — real
# numbers must come from each project's own publication, not be invented here.
CITE = "[cite]"
BASELINES = [
    ("Stanford / Mini Pupper", "cheap PWM/serial hobby-servo quadruped"),
    ("MIT Mini-Cheetah", "QDD BLDC, backdrivable, research-grade"),
    ("ODRI Solo (8/12)", "QDD BLDC open-source, backdrivable"),
]

# Hardware-measured axes — the paper's KEY empirical claims. Not derivable in sim,
# so they are explicit pending slots (never faked).
HARDWARE_TBD = [
    ("Acoustic noise — motion", "dB(A) @ 1 m during walk"),
    ("Acoustic noise — hold", "dB(A) @ 1 m holding stance (the 'quiet' claim)"),
    ("Joint backlash", "deg lost motion at the knee output"),
    ("Backdrive torque", "N·m to back-drive a held joint (the 'compliant' claim)"),
    ("Battery runtime", "min of active operation on the LiPo"),
    ("Sim-to-real tracking gap", "deg RMS joint-tracking error vs this sim"),
]


# --------------------------------------------------------------------- gaits
def run_gait(name: str, seconds: float = SECONDS) -> dict:
    """One gait via the real emulator (render off = fast + deterministic)."""
    _rig, log, fell_at, travel = simulate(name, seconds, render=False)
    peak_tau = max(log["tau"]) if log["tau"] else 0.0

    def pp(key):
        return (max(log[key]) - min(log[key])) if log[key] else 0.0

    cam_rms = 0.0
    if log["cam_p"]:
        cam_rms = float(np.sqrt(np.mean(np.square(log["cam_p"]) + np.square(log["cam_r"]))))
    return {
        "gait": name, "seconds": seconds,
        "stays_upright": fell_at is None,
        "fell_at_s": fell_at,
        "travel_cm": round(travel * 100.0, 1),
        "peak_leg_torque_nm": round(peak_tau, 3),
        "peak_torque_pct_stall": round(peak_tau / SERVO.stall_nm * 100.0, 1),
        "torque_headroom_pct": round(100.0 - peak_tau / SERVO.stall_nm * 100.0, 1),
        "torso_roll_pp_deg": round(pp("roll"), 1),
        "torso_pitch_pp_deg": round(pp("pitch"), 1),
        "camera_shake_pitch_pp_deg": round(pp("cam_p"), 1),
        "camera_shake_roll_pp_deg": round(pp("cam_r"), 1),
        "camera_shake_rms_deg": round(cam_rms, 1),
    }


# --------------------------------------------------------------------- DOF sharing
def dof_sharing() -> dict:
    """Dual-use DOF: joints the control code drives for BOTH stabilization/gait AND
    expression — a design-efficiency metric (one motor, two jobs). Grounded in the
    actual code paths: head_stabilize() (mj_emulate) drives head_tilt/head_pitch for
    the camera gimbal while cute_motion drives them for nod/tilt; gait.spine_wave
    drives the waist while cute_motion arches it."""
    shared = [
        {"joint": "torso_aft (waist)", "role_a": "gait spine undulation (gait.spine_wave)",
         "role_b": "expressive arch/loaf (cute_motion)"},
        {"joint": "head_tilt", "role_a": "camera roll stabilization (head_stabilize)",
         "role_b": "quizzical head-tilt (cute_motion)"},
        {"joint": "head_pitch", "role_a": "camera pitch stabilization (head_stabilize)",
         "role_b": "nod / head-up (cute_motion)"},
        {"joint": "head_pan", "role_a": "camera aim / look-at gimbal",
         "role_b": "idle look-around + anticipation wiggle (cute_motion)"},
    ]
    return {"count": len(shared), "shared": shared,
            "note": "one actuator serving two functions — reduces motor count vs a "
                    "dedicated gimbal + separate expression joints"}


# --------------------------------------------------------------------- comparison
def comparison_table(gaits: dict, mech: dict) -> dict:
    """Sabo's column from our data; baselines as [cite] placeholders."""
    c = spec.cost_summary()
    m = spec.mass_summary()
    act = spec.actuator_summary()
    walk = gaits.get("walk", {})
    sabo = {
        "cost_usd": f"~${c['usd_mid']:.0f} (build, self-sourced)",
        "mass_scale": f"{m['total_kg']*1000:.0f} g kitten-scale quadruped",
        "actuator_type": f"{act['model']} serial bus servo ({act['control']})",
        "quiet": "compliant silent-hold by design (backdrivable); dB TBD (hardware)",
        "backdrivable": str(act["backdrivable"]),
        "dof": f"{spec.dof_breakdown()['actuated_motors']} actuated",
    }
    axes = ["cost_usd", "mass_scale", "actuator_type", "quiet", "backdrivable", "dof"]
    return {"axes": axes, "sabo": sabo,
            "baselines": {name: {a: CITE for a in axes} | {"_desc": desc}
                          for name, desc in BASELINES}}


# --------------------------------------------------------------------- assemble
def build_benchmark() -> dict:
    gaits = {g: run_gait(g) for g in GAITS}
    tx = spec.transmission_summary()
    mech = {
        "four_bar_knee": {
            "knee_rom_rad": tx["four_bar_knee"]["knee_rom_rad"],
            "knee_rom_deg": tx["four_bar_knee"]["knee_rom_deg"],
            "transmission_angle_deg": tx["four_bar_knee"]["transmission_angle_deg"],
            "singularity_free": tx["four_bar_knee"]["singularity_free"],
            "monotonic_invertible": tx["four_bar_knee"]["monotonic_invertible"],
        },
        "remote_axle_hip": {
            "lateral_inertia_reduction_pct": tx["remote_axle_hip"]["inertia_reduction_pct"],
        },
    }
    dofs = dof_sharing()
    return {
        "servo": {"model": SERVO.name, "stall_nm": round(SERVO.stall_nm, 3)},
        "gaits": gaits,
        "mechanism": mech,
        "dof_sharing": dofs,
        "comparison": comparison_table(gaits, mech),
        "hardware_tbd": [{"axis": a, "measure": m, "value": "TBD (hardware)"}
                         for a, m in HARDWARE_TBD],
    }


# --------------------------------------------------------------------- render
def render_markdown(b: dict) -> str:
    L = []
    w = L.append
    w("# Sabo — Platform Benchmark\n")
    w("_Auto-generated by `python -m analysis.benchmark`. Sim metrics from the real "
      "`sim.mj_emulate` gaits + `sim.gait` presets under the servo torque limits. "
      "Deterministic, headless._\n")
    w(f"Servo: **{b['servo']['model']}**, stall **{b['servo']['stall_nm']:.2f} N·m**.\n")

    w("## Per-gait performance (sim-measured)\n")
    w("| Gait | Upright | Travel | Peak leg τ | % of stall | Headroom | "
      "Torso roll p-p | Torso pitch p-p | Camera shake (p-p / RMS) |")
    w("|---|:--:|--:|--:|--:|--:|--:|--:|--:|")
    for g in GAITS:
        r = b["gaits"][g]
        up = "PASS" if r["stays_upright"] else f"FELL@{r['fell_at_s']:.1f}s"
        cam = (f"{max(r['camera_shake_pitch_pp_deg'], r['camera_shake_roll_pp_deg']):.1f}° / "
               f"{r['camera_shake_rms_deg']:.1f}°")
        w(f"| {g} | {up} | {r['travel_cm']:.0f} cm | {r['peak_leg_torque_nm']:.2f} N·m | "
          f"{r['peak_torque_pct_stall']:.0f}% | {r['torque_headroom_pct']:.0f}% | "
          f"{r['torso_roll_pp_deg']:.1f}° | {r['torso_pitch_pp_deg']:.1f}° | {cam} |")
    w("")

    w("## Mechanism (design metrics)\n")
    fbk = b["mechanism"]["four_bar_knee"]
    hip = b["mechanism"]["remote_axle_hip"]
    w("| Metric | Value |")
    w("|---|---|")
    w(f"| Four-bar knee ROM | {fbk['knee_rom_deg']:.0f}° ({fbk['knee_rom_rad']:.2f} rad) |")
    w(f"| Four-bar transmission angle | {fbk['transmission_angle_deg'][0]:.0f}°.."
      f"{fbk['transmission_angle_deg'][1]:.0f}° "
      f"({'singularity-free' if fbk['singularity_free'] else 'RISK'}) |")
    w(f"| Four-bar monotonic / invertible | {fbk['monotonic_invertible']} |")
    w(f"| Remote-axle hip lateral-inertia reduction | "
      f"{hip['lateral_inertia_reduction_pct']:.0f}% |")
    w("")

    w("## DOF-sharing (design efficiency)\n")
    ds = b["dof_sharing"]
    w(f"**{ds['count']} dual-use DOF** — {ds['note']}.\n")
    w("| Joint | Function A | Function B |")
    w("|---|---|---|")
    for s in ds["shared"]:
        w(f"| {s['joint']} | {s['role_a']} | {s['role_b']} |")
    w("")

    w("## Platform comparison (scaffold)\n")
    cmp = b["comparison"]
    labels = {"cost_usd": "Cost", "mass_scale": "Mass / scale",
              "actuator_type": "Actuator", "quiet": "Quiet?",
              "backdrivable": "Backdrivable?", "dof": "DOF"}
    cols = ["Sabo (this work)"] + [name for name in cmp["baselines"]]
    w("| Axis | " + " | ".join(cols) + " |")
    w("|---|" + "|".join(["---"] * len(cols)) + "|")
    for a in cmp["axes"]:
        row = [cmp["sabo"][a]] + [cmp["baselines"][name][a] for name in cmp["baselines"]]
        w(f"| {labels[a]} | " + " | ".join(row) + " |")
    w("")
    w("_Baseline cells are `[cite]` placeholders — fill from each project's own "
      "publication; do not invent competitor numbers._\n")
    for name in cmp["baselines"]:
        w(f"- **{name}** — {cmp['baselines'][name]['_desc']}")
    w("")

    w("## Hardware-measured claims (pending)\n")
    w("_The paper's key empirical claims (\"quiet\", \"compliant\") are NOT derivable "
      "in sim. They are explicit pending slots — measured on the built robot._\n")
    w("| Axis | Measure | Value |")
    w("|---|---|---|")
    for h in b["hardware_tbd"]:
        w(f"| {h['axis']} | {h['measure']} | **{h['value']}** |")
    w("")
    return "\n".join(L)


def write(b: dict) -> tuple[str, str]:
    os.makedirs(OUT, exist_ok=True)
    md_path = os.path.abspath(os.path.join(OUT, "benchmark.md"))
    json_path = os.path.abspath(os.path.join(OUT, "benchmark.json"))
    with open(md_path, "w", encoding="utf-8") as f:
        f.write(render_markdown(b))
    with open(json_path, "w", encoding="utf-8") as f:
        json.dump(b, f, indent=2)
    return md_path, json_path


def main() -> None:
    b = build_benchmark()
    print(render_markdown(b))
    md_path, json_path = write(b)
    print(f"\nwrote: {md_path}")
    print(f"wrote: {json_path}")


if __name__ == "__main__":
    main()
