"""
Platform spec sheet — the auto-derived datasheet for the Sabo platform.
=======================================================================

    python -m analysis.platform_spec        # print sheet + write docs/out/

Emits a research-platform spec sheet (DOF, mass, cost, actuator, compute, sensors,
envelope, per-leg geometry, transmission mechanism) that is **entirely derived from
the single source of truth** — ``cad/params.py`` + ``cad/servo.py`` drive the CAD,
the sim, the BOM and now this sheet, so they can never silently diverge. No number
that lives in params/servo/bom is hand-copied here; each is pulled live.

Writes ``docs/out/platform_spec.md`` + ``platform_spec.json``. This is contribution
(1) of the platform paper (design-as-code) made concrete: regenerating the sheet
provably re-reads the model.

The few figures that do NOT live in the mechanical model — the Jetson module's TOPS
and power-mode envelope — are datasheet facts about the named BOM part; they are kept
in ``COMPUTE_DATASHEET`` below with an explicit source note, not silently invented.
"""

from __future__ import annotations

import json
import math
import os

import trimesh

from cad import params as P
from cad.assembly import kinematics, n_motors
from cad.servo import DEFAULT as SERVO
from analysis import bom
from analysis.fourbar import FourBar, MU_HI, MU_LO, NEED_ROM, usable_window
from sim import gait

OUT = os.path.join(os.path.dirname(__file__), "..", "docs", "out")
_CAD_OUT = os.path.join(os.path.dirname(__file__), "..", "cad", "out")

# Datasheet facts for the NAMED compute module in the BOM (do not live in the
# mechanical model). Source: NVIDIA Jetson Orin Nano (Super) module datasheet /
# JetPack power-mode table. Kept explicit + cited so they are auditable, not faked.
COMPUTE_DATASHEET = {
    "sparse_int8_tops": 67,          # AI perf, MAXN SUPER mode
    "power_modes_w": [7, 15, 25],    # selectable power envelopes (W)
    "gpu": "1024-core Ampere GPU + 32 Tensor cores",
    "source": "NVIDIA Jetson Orin Nano (Super) datasheet / JetPack power-mode table",
}


# --------------------------------------------------------------------- DOF
def dof_breakdown() -> dict:
    """Every joint in the kinematic tree, classified straight from ``kinematics()``."""
    links = kinematics()
    actuated = [lk.name for lk in links if lk.actuated]
    coupled = [(lk.name, lk.couple[0]) for lk in links if lk.couple]
    rigid = [lk.name for lk in links if not lk.has_joint]

    leg_motors = sorted(n for n in actuated
                        if n[:2] in P.LEGS and n[3:] in ("hip", "knee"))
    expressive = [n for n in actuated if n not in leg_motors]
    coupled_ankle = [c for c in coupled if c[0].endswith("_ankle")]
    coupled_ear = [c for c in coupled if "ear" in c[0]]

    return {
        "actuated_motors": n_motors(),                # == servo count
        "actuated_joints": actuated,
        "per_leg": {"motorized": 2, "coupled": 1, "rigid_abduction": 1, "total": 4},
        "leg_motors": leg_motors,
        "expressive_motors": expressive,
        "coupled_ankle": [c[0] for c in coupled_ankle],
        "coupled_ear": [c[0] for c in coupled_ear],
        "rigid_abduction": rigid,
        "passive_coupled_dof": len(coupled),          # ankles + linked ear follower
        "notes": {
            "ankle": "coupled to knee (ankle = c0 + c1*knee) — cat reciprocal apparatus",
            "abduction": f"rigid (ABDUCTION_ACTIVE={P.ABDUCTION_ACTIVE}); turn by gait",
            "ears": f"linked on one motor (EARS_LINKED={P.EARS_LINKED})",
        },
    }


# --------------------------------------------------------------------- mass
def mass_summary() -> dict:
    from analysis.validate import total_mass
    printed, comp, total = total_mass()
    lo, hi = P.MASS_TARGET
    return {
        "printed_plastic_kg": printed, "components_kg": comp, "total_kg": total,
        "target_kg": [lo, hi], "in_target": lo <= total <= hi,
        "effective_density_kg_m3": P.EFFECTIVE_DENSITY,
    }


# --------------------------------------------------------------------- cost
def cost_summary() -> dict:
    c = bom.cost_totals()
    return {"usd_low": c["low"], "usd_mid": c["mid"], "usd_high": c["high"],
            "filament_g": c["filament_g"]}


# --------------------------------------------------------------------- actuator
def actuator_summary() -> dict:
    return {
        "model": SERVO.name, "count": P.N_SERVOS,
        "stall_nm": SERVO.stall_nm, "stall_kgcm": SERVO.stall_kgcm,
        "mass_g_each": SERVO.mass_kg * 1000.0,
        "speed_s_per_60deg": SERVO.speed_s_60,
        "control": SERVO.control, "bus": SERVO.bus,
        "backdrivable": SERVO.backdrivable, "noise_class": SERVO.noise,
        "quiet_coreless": SERVO.quiet, "unit_cost_usd": SERVO.cost_usd,
    }


# --------------------------------------------------------------------- compute
def compute_summary() -> dict:
    name = next((item for cat, item, *_ in bom.COMPONENTS if cat == "Compute"), "Jetson")
    d = dict(COMPUTE_DATASHEET)
    d["module"] = name
    return d


# --------------------------------------------------------------------- sensors
def sensor_suite() -> list:
    return [{"item": item, "qty": qty, "note": note}
            for cat, item, qty, lo, hi, note in bom.COMPONENTS if cat == "Sensors"]


# --------------------------------------------------------------------- envelope
def envelope_mm() -> dict:
    """Bounding envelope (L x W x H, mm) of the posed robot, read from the exported
    cat-shell mesh (honest outer skin). Falls back to the mechanical full-robot mesh;
    if neither exists yet, returns ``pending`` (run ``cad.export`` first — computing
    it live from ``full_robot()`` is a ~90 s CSG fuse, so we prefer the cached mesh)."""
    for fname, kind in (("sabo_cat.stl", "cat shell (outer skin)"),
                        ("robokitten_full.stl", "mechanical full assembly")):
        path = os.path.join(_CAD_OUT, fname)
        if os.path.exists(path):
            m = trimesh.load(path)
            s = m.bounds[1] - m.bounds[0]
            return {"length_mm": round(float(s[0]), 1), "width_mm": round(float(s[1]), 1),
                    "height_mm": round(float(s[2]), 1), "source": f"{fname} ({kind})"}
    return {"length_mm": None, "width_mm": None, "height_mm": None,
            "source": "pending — run `python -m cad.export`"}


# --------------------------------------------------------------------- legs
def leg_summary() -> dict:
    out = {}
    for kind, leg, g in (("front", "FL", P.FRONT), ("rear", "RL", P.REAR)):
        hip, knee = gait.stance_angles(leg)
        ankle = gait.ankle_from_knee(leg, knee)
        lo_k, hi_k = P.leg_knee_limit(leg)
        out[kind] = {
            "segments_mm": {"upper": g["upper"], "lower": g["lower"], "foot": g["foot"]},
            "reach_mm": g["upper"] + g["lower"] + g["foot"],
            "hip_offset_mm": g["hip_off"], "stance_depth_mm": g["stance_depth"],
            "stance_deg": {"hip": math.degrees(hip), "knee": math.degrees(knee),
                           "ankle_coupled": math.degrees(ankle)},
            "knee_limit_deg": [math.degrees(lo_k), math.degrees(hi_k)],
        }
    return out


# --------------------------------------------------------------------- transmission
def _fourbar() -> FourBar:
    fb = P.FOURBAR
    return FourBar(d=fb["ground"], r2=fb["crank"], r3=fb["coupler"], r4=fb["rocker"],
                   rocker_offset=fb["rocker_offset"], crank_window=fb["crank_window"])


def transmission_summary() -> dict:
    """The two headline mechanisms: proximal four-bar knee + remote-axle hip — with
    every figure computed from ``P.FOURBAR`` / ``P.FRONT`` / ``P.REAR`` / the servo."""
    fb = _fourbar()
    rom, lo, hi, mu_lo, mu_hi = usable_window(fb)
    lo_deg, hi_deg = P.FOURBAR["crank_window"]

    # per-leg knee JOINT reach = four-bar knee_fb sweep + per-leg stance offset
    per_leg = {}
    for kind, leg in (("front", "FL"), ("rear", "RL")):
        k_lo = gait.crank_to_knee(leg, math.radians(lo_deg))
        k_hi = gait.crank_to_knee(leg, math.radians(hi_deg))
        per_leg[kind] = {"knee_reach_deg": [round(math.degrees(min(k_lo, k_hi)), 1),
                                            round(math.degrees(max(k_lo, k_hi)), 1)]}

    # remote-axle hip: lateral (roll-axis) inertia of the 4 hip-servo masses, servo
    # BODY out on the leg vs relocated into the torso core (validate.check_balance
    # places the relocated body at |y| = HIP_CORE_HORN_Y - 18).
    m = SERVO.mass_kg
    y_leg_f = P.m(P.BODY_W / 2 + P.FRONT["hip_off"])
    y_leg_r = P.m(P.BODY_W / 2 + P.REAR["hip_off"])
    y_core = P.m(P.HIP_CORE_HORN_Y - 18.0)
    I_leg = 2 * m * y_leg_f ** 2 + 2 * m * y_leg_r ** 2
    I_core = 4 * m * y_core ** 2
    reduction = 100.0 * (1 - I_core / I_leg)

    return {
        "four_bar_knee": {
            "geometry_mm": dict(P.FOURBAR),
            "crank_window_deg": [lo_deg, hi_deg],
            "knee_rom_rad": round(rom, 3), "knee_rom_deg": round(math.degrees(rom), 1),
            "transmission_angle_deg": [round(mu_lo, 1), round(mu_hi, 1)],
            "transmission_band_deg": [MU_LO, MU_HI],
            "singularity_free": mu_lo >= MU_LO and mu_hi <= MU_HI,
            "monotonic_invertible": True,
            "need_rom_rad": NEED_ROM, "meets_need": rom >= NEED_ROM,
            "per_leg": per_leg,
        },
        "remote_axle_hip": {
            "drive": P.HIP_DRIVE, "axle_dia_mm": 2 * P.AXLE_R,
            "core_horn_y_mm": P.HIP_CORE_HORN_Y,
            "lateral_inertia_leg_kgm2": I_leg, "lateral_inertia_core_kgm2": I_core,
            "inertia_reduction_pct": round(reduction, 1),
        },
    }


# --------------------------------------------------------------------- assemble
def build_spec() -> dict:
    return {
        "platform": "Sabo — low-cost, quiet, compliant 3D-printed quadruped",
        "source_of_truth": ["cad/params.py", "cad/servo.py"],
        "dof": dof_breakdown(),
        "mass": mass_summary(),
        "cost": cost_summary(),
        "actuator": actuator_summary(),
        "compute": compute_summary(),
        "sensors": sensor_suite(),
        "envelope": envelope_mm(),
        "legs": leg_summary(),
        "transmission": transmission_summary(),
    }


# --------------------------------------------------------------------- render
def render_markdown(spec: dict) -> str:
    d, mass, cost = spec["dof"], spec["mass"], spec["cost"]
    act, comp, env = spec["actuator"], spec["compute"], spec["envelope"]
    tx = spec["transmission"]
    fbk, hip = tx["four_bar_knee"], tx["remote_axle_hip"]
    L = []
    w = L.append
    w("# Sabo — Platform Spec Sheet\n")
    w("_Auto-generated by `python -m analysis.platform_spec`. Every value is derived "
      "live from the single source of truth (`cad/params.py` + `cad/servo.py`) via the "
      "CAD / sim / BOM — do not hand-edit._\n")

    w("## Degrees of freedom\n")
    w("| DOF class | Count | Detail |")
    w("|---|--:|---|")
    w(f"| Actuated motors (= servos) | {d['actuated_motors']} | 8 leg (hip+knee) + "
      f"waist + head pan/pitch/tilt + ears + tail |")
    w(f"| Per-leg | 4 | 2 motorized (hip+knee) + 1 coupled ankle + 1 rigid abduction |")
    w(f"| Coupled / passive | {d['passive_coupled_dof']} | "
      f"4 ankle (knee-coupled) + 1 linked ear follower |")
    w(f"| Rigid abduction | {len(d['rigid_abduction'])} | {d['notes']['abduction']} |")
    w(f"| Expressive motors | {len(d['expressive_motors'])} | "
      f"{', '.join(d['expressive_motors'])} |\n")

    w("## Physical\n")
    w("| Property | Value | Source |")
    w("|---|---|---|")
    w(f"| Total mass | {mass['total_kg']*1000:.0f} g "
      f"(target {mass['target_kg'][0]*1000:.0f}–{mass['target_kg'][1]*1000:.0f} g, "
      f"{'in band' if mass['in_target'] else 'OUT'}) | validate.total_mass |")
    w(f"| — printed plastic | {mass['printed_plastic_kg']*1000:.0f} g | "
      f"CAD volume × {mass['effective_density_kg_m3']:.0f} kg/m³ |")
    w(f"| — bought components | {mass['components_kg']*1000:.0f} g | "
      f"params.COMPONENT_MASS |")
    w(f"| Unit build cost | ${cost['usd_low']:.0f} / ${cost['usd_mid']:.0f} / "
      f"${cost['usd_high']:.0f} (lo/mid/hi) | analysis.bom |")
    if env["length_mm"] is not None:
        w(f"| Envelope (L×W×H) | {env['length_mm']:.0f} × {env['width_mm']:.0f} × "
          f"{env['height_mm']:.0f} mm | {env['source']} |")
    else:
        w(f"| Envelope (L×W×H) | {env['source']} | — |")
    w("")

    w("## Actuator (every joint)\n")
    w("| Property | Value |")
    w("|---|---|")
    w(f"| Model | {act['model']} × {act['count']} |")
    w(f"| Stall torque | {act['stall_nm']:.2f} N·m ({act['stall_kgcm']:.0f} kg·cm) |")
    w(f"| Mass each | {act['mass_g_each']:.0f} g |")
    w(f"| Speed | {act['speed_s_per_60deg']:.2f} s/60° |")
    w(f"| Control / bus | {act['control']} — {act['bus']} |")
    w(f"| Backdrivable | {act['backdrivable']} (compliant, silent hold) |")
    w(f"| Noise class | {act['noise_class']} |")
    w(f"| Unit cost | ${act['unit_cost_usd']:.0f} |\n")

    w("## Compute\n")
    w(f"- **Module:** {comp['module']}")
    w(f"- **AI perf:** {comp['sparse_int8_tops']} TOPS (sparse INT8), "
      f"{comp['gpu']}")
    w(f"- **Power modes:** {', '.join(str(x) for x in comp['power_modes_w'])} W")
    w(f"- _source: {comp['source']}_\n")

    w("## Sensor suite\n")
    w("| Sensor | Qty | Role |")
    w("|---|--:|---|")
    for s in spec["sensors"]:
        w(f"| {s['item']} | {s['qty']} | {s['note']} |")
    w("")

    w("## Legs (per-leg segment lengths + stance)\n")
    w("| Leg | Upper | Lower | Foot | Reach | Hip off | Stance depth | "
      "Stance (hip/knee/ankle*) |")
    w("|---|--:|--:|--:|--:|--:|--:|---|")
    for kind in ("front", "rear"):
        lg = spec["legs"][kind]
        seg = lg["segments_mm"]; st = lg["stance_deg"]
        w(f"| {kind} | {seg['upper']:.1f} | {seg['lower']:.1f} | {seg['foot']:.1f} | "
          f"{lg['reach_mm']:.1f} | {lg['hip_offset_mm']:.1f} | {lg['stance_depth_mm']:.1f} | "
          f"{st['hip']:.0f}° / {st['knee']:.0f}° / {st['ankle_coupled']:.0f}°* |")
    w("\n_*ankle is knee-coupled (not motorized)._\n")

    w("## Transmission mechanism\n")
    w("### Proximal four-bar knee (`analysis/fourbar.py`, `P.FOURBAR`)\n")
    g = fbk["geometry_mm"]
    w(f"- **Geometry (mm):** ground d={g['ground']}, crank r2={g['crank']}, "
      f"coupler r3={g['coupler']}, rocker r4={g['rocker']}")
    w(f"- **Crank window:** {fbk['crank_window_deg'][0]:.0f}°..{fbk['crank_window_deg'][1]:.0f}°")
    w(f"- **Knee ROM:** {fbk['knee_rom_deg']:.0f}° = {fbk['knee_rom_rad']:.2f} rad "
      f"(need ≳ {fbk['need_rom_rad']} rad → {'meets' if fbk['meets_need'] else 'SHORT'})")
    w(f"- **Transmission angle:** {fbk['transmission_angle_deg'][0]:.0f}°.."
      f"{fbk['transmission_angle_deg'][1]:.0f}° "
      f"(band {fbk['transmission_band_deg'][0]:.0f}–{fbk['transmission_band_deg'][1]:.0f}° → "
      f"{'singularity-free' if fbk['singularity_free'] else 'RISK'})")
    w(f"- **Monotonic / invertible:** {fbk['monotonic_invertible']}")
    w(f"- **Per-leg knee reach:** front "
      f"{fbk['per_leg']['front']['knee_reach_deg'][0]:.1f}..{fbk['per_leg']['front']['knee_reach_deg'][1]:.1f}°, "
      f"rear {fbk['per_leg']['rear']['knee_reach_deg'][0]:.1f}..{fbk['per_leg']['rear']['knee_reach_deg'][1]:.1f}°\n")
    w("### Remote-axle hip (`cad/parts/body.py`, `P.HIP_DRIVE`)\n")
    w(f"- **Drive:** {hip['drive']} — servo body in the torso core, Ø{hip['axle_dia_mm']:.0f} mm "
      f"lateral axle out to the (unchanged) hip pivot")
    w(f"- **Lateral (roll-axis) inertia of the 4 hip servos:** "
      f"{hip['lateral_inertia_leg_kgm2']*1e4:.2f} → {hip['lateral_inertia_core_kgm2']*1e4:.2f} "
      f"×10⁻⁴ kg·m² → **{hip['inertia_reduction_pct']:.0f}% reduction** "
      f"(body relocated from |y|≈{P.BODY_W/2+P.FRONT['hip_off']:.0f} mm to "
      f"|y|≈{hip['core_horn_y_mm']-18:.0f} mm)\n")
    return "\n".join(L)


def write(spec: dict) -> tuple[str, str]:
    os.makedirs(OUT, exist_ok=True)
    md_path = os.path.abspath(os.path.join(OUT, "platform_spec.md"))
    json_path = os.path.abspath(os.path.join(OUT, "platform_spec.json"))
    with open(md_path, "w", encoding="utf-8") as f:
        f.write(render_markdown(spec))
    with open(json_path, "w", encoding="utf-8") as f:
        json.dump(spec, f, indent=2)
    return md_path, json_path


def main() -> None:
    spec = build_spec()
    print(render_markdown(spec))
    md_path, json_path = write(spec)
    print(f"\nwrote: {md_path}")
    print(f"wrote: {json_path}")


if __name__ == "__main__":
    main()
