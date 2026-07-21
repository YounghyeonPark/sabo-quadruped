"""
Dimensional optimizer — shrink the walking waddle (roll) + peak torque.
=======================================================================

    python -m analysis.optimize                 # coordinate-descent + random search
    python -m analysis.optimize --evals 60
    python -m analysis.optimize --apply          # also write the winner into params.py

Searches the key body/leg dimensions in ``cad.params`` for the geometry that
minimises walking instability while staying cat-like and buildable.

Why a *fast* evaluator
----------------------
Rebuilding the real model per candidate is dominated by build123d (seconds each).
So this module builds a **primitive** MuJoCo model straight from the numbers in
``cad.params`` — box/capsule links whose dimensions mirror the real link lengths,
masses = length x cross-section x ``EFFECTIVE_DENSITY`` x a calibrated print-fill
plus the exact bought-component / servo masses, and box-formula inertia. The
kinematic tree, joint axes, servo ``kp``/torque limits, floor friction, the
knee->ankle coupling, and the IMU body-leveling + gait control are **identical**
to the real pipeline (we reuse ``sim.gait`` for IK and ``sim.mj_emulate`` for the
control loop), so the ranking transfers. The winner is then verified on the REAL
build123d pipeline.

Each eval mutates ``cad.params`` in place (so ``sim.gait`` IK stays the single
source of truth), rebuilds the primitive model, runs an 8 s walk (scored) and a
6 s trot (upright check).

Objective (minimise):  ``roll_pp + 0.05*torque_pct + 0.5*pitch_pp``
Hard constraints (rejected with a big penalty):
  * UPRIGHT on walk AND trot,
  * total mass in 0.8-1.6 kg,
  * walk travel >= 14 cm in 8 s,
  * every parameter inside its bound; REAR total leg length >= FRONT.
"""

from __future__ import annotations

import argparse
import math
import random

import mujoco
import numpy as np

from cad import params as P
from cad.servo import DEFAULT as SERVO
from sim import gait
from sim import mj_emulate as MJ

# ---------------------------------------------------------------- calibrated fills
# print-fill fractions chosen so the primitive plastic masses reproduce the real
# build123d parts at the baseline dims (torso ribcage is mostly air; struts are
# fenestrated). Only plastic (~230 g) scales with dims; the 708 g of components is
# exact, so total mass tracks the real model to within a few grams.
FILL_TORSO = 0.375
FILL_HEAD = 0.20
FILL_LEG = 0.55
FILL_TAIL = 0.15
FILL_MISC = 0.20
DENS = P.EFFECTIVE_DENSITY


# ---------------------------------------------------------------- search space
# name -> (getter, setter, low, high). Getters/setters read/write cad.params.
def _leg_get(side, key):
    return lambda: (P.FRONT if side == "F" else P.REAR)[key]


def _leg_set(side, key):
    def s(v):
        (P.FRONT if side == "F" else P.REAR)[key] = float(v)
    return s


def _attr_get(name):
    return lambda: getattr(P, name)


def _attr_set(name):
    return lambda v: setattr(P, name, float(v))


SPACE = {
    "BODY_W":      (_attr_get("BODY_W"), _attr_set("BODY_W"), 60.0, 95.0),
    "BODY_H":      (_attr_get("BODY_H"), _attr_set("BODY_H"), 40.0, 55.0),
    "F_hip_off":   (_leg_get("F", "hip_off"), _leg_set("F", "hip_off"), 22.0, 40.0),
    "R_hip_off":   (_leg_get("R", "hip_off"), _leg_set("R", "hip_off"), 22.0, 40.0),
    "F_upper":     (_leg_get("F", "upper"), _leg_set("F", "upper"), 36.4, 67.6),
    "F_lower":     (_leg_get("F", "lower"), _leg_set("F", "lower"), 36.4, 67.6),
    "F_foot":      (_leg_get("F", "foot"), _leg_set("F", "foot"), 18.2, 33.8),
    "R_upper":     (_leg_get("R", "upper"), _leg_set("R", "upper"), 43.4, 80.6),
    "R_lower":     (_leg_get("R", "lower"), _leg_set("R", "lower"), 43.4, 80.6),
    "R_foot":      (_leg_get("R", "foot"), _leg_set("R", "foot"), 29.4, 54.6),
    "F_stance":    (_leg_get("F", "stance_depth"), _leg_set("F", "stance_depth"), 60.0, 92.0),
    "R_stance":    (_leg_get("R", "stance_depth"), _leg_set("R", "stance_depth"), 60.0, 92.0),
}
NAMES = list(SPACE)


def snapshot() -> dict:
    return {n: SPACE[n][0]() for n in NAMES}


def apply_vec(vec: dict) -> None:
    for n, v in vec.items():
        SPACE[n][1](v)
    _refresh_derived()


def _refresh_derived() -> None:
    """Recompute the module-level values baked from the params at import time."""
    P.MOUNTS = {
        "FL": (P.FORE_LEN - P.MOUNT_INSET, +P.BODY_W / 2),
        "FR": (P.FORE_LEN - P.MOUNT_INSET, -P.BODY_W / 2),
        "RL": (-(P.AFT_LEN - P.MOUNT_INSET), +P.BODY_W / 2),
        "RR": (-(P.AFT_LEN - P.MOUNT_INSET), -P.BODY_W / 2),
    }
    P.STANCE_H = min(P.FRONT["stance_depth"], P.REAR["stance_depth"])


# ---------------------------------------------------------------- primitive MJCF
def _inertial(mass, center, size) -> str:
    a, b, c = size
    ix, iy, iz = (mass / 12 * (b*b + c*c), mass / 12 * (a*a + c*c), mass / 12 * (a*a + b*b))
    return (f'<inertial pos="{center[0]:.5f} {center[1]:.5f} {center[2]:.5f}" '
            f'mass="{max(mass, 1e-3):.4f}" '
            f'diaginertia="{ix:.6e} {iy:.6e} {iz:.6e}"/>')


def _box_geom(center, size, rgba="0.6 0.62 0.66 1") -> str:
    return (f'<geom type="box" pos="{center[0]:.5f} {center[1]:.5f} {center[2]:.5f}" '
            f'size="{size[0]/2:.5f} {size[1]/2:.5f} {size[2]/2:.5f}" '
            f'contype="0" conaffinity="0" rgba="{rgba}"/>')


def build_primitive_mjcf() -> str:
    """MuJoCo model from the current ``cad.params`` numbers only (no build123d).

    Mirrors ``cad.assembly.kinematics()`` link-for-link and emits bodies in the
    same nesting order, so index-based readouts in ``sim.mj_emulate`` line up."""
    m = P.m
    seg = P.LEG_SEG_W
    c = P.COMPONENT_MASS
    sv = SERVO.mass_kg

    def leg_plastic(L):
        return L * seg * seg * 1e-9 * DENS * FILL_LEG

    start_z = m(max(g["stance_depth"] + g["foot"] * 0.6
                    for g in (P.FRONT, P.REAR))) + 0.01

    def emit_leg(leg, indent):
        pad = "  " * indent
        mx, my = P.MOUNTS[leg]
        s = P.leg_plane_sign(leg)
        g = P.leg_geom(leg)
        c0, c1 = gait.ankle_couple_coef(leg)
        up, lo, ft = g["upper"], g["lower"], g["foot"]
        hoff = g["hip_off"]
        # abduction bracket (welded) — carries the hip servo
        br_mass = 20.0 * (hoff + 12) * 30.0 * 1e-9 * DENS * FILL_MISC + sv
        out = f'{pad}<body name="{leg}_abd" pos="{mx*0.001:.5f} {my*0.001:.5f} 0">\n'
        out += f'{pad}  {_inertial(br_mass, (0, s*hoff/2*0.001, 0), (m(20), m(hoff+12), m(30)))}\n'
        out += f'{pad}  {_box_geom((0, s*hoff/2*0.001, 0), (m(20), m(hoff+12), m(30)))}\n'
        # hip -> upper segment (+ hip servo)
        out += (f'{pad}  <body name="{leg}_hip" pos="0 {s*hoff*0.001:.5f} 0">\n'
                f'{pad}    <joint name="{leg}_hip" type="hinge" axis="0 -1 0" '
                f'range="{P.LIM_HIP[0]} {P.LIM_HIP[1]}"/>\n'
                f'{pad}    {_inertial(leg_plastic(up)+sv, (0,0,-m(up)/2), (m(seg),m(seg),m(up)))}\n'
                f'{pad}    {_box_geom((0,0,-m(up)/2), (m(seg),m(seg),m(up)))}\n')
        # knee -> lower segment
        out += (f'{pad}    <body name="{leg}_knee" pos="0 0 {-m(up):.5f}">\n'
                f'{pad}      <joint name="{leg}_knee" type="hinge" axis="0 -1 0" '
                f'range="{P.leg_knee_limit(leg)[0]} {P.leg_knee_limit(leg)[1]}"/>\n'
                f'{pad}      {_inertial(leg_plastic(lo), (0,0,-m(lo)/2), (m(seg),m(seg),m(lo)))}\n'
                f'{pad}      {_box_geom((0,0,-m(lo)/2), (m(seg),m(seg),m(lo)))}\n')
        # ankle -> foot segment (coupled to knee) + toe contact sphere
        out += (f'{pad}      <body name="{leg}_ankle" pos="0 0 {-m(lo):.5f}">\n'
                f'{pad}        <joint name="{leg}_ankle" type="hinge" axis="0 -1 0" '
                f'range="{P.LIM_ANKLE[0]} {P.LIM_ANKLE[1]}"/>\n'
                f'{pad}        {_inertial(leg_plastic(ft), (0,0,-m(ft)/2), (m(seg),m(seg),m(ft)))}\n'
                f'{pad}        {_box_geom((0,0,-m(ft)/2), (m(seg),m(seg),m(ft)))}\n'
                f'{pad}        <geom name="{leg}_ankle_toe" type="sphere" pos="0 0 {-m(ft):.5f}" '
                f'size="{m(P.TOE_R):.5f}" contype="1" conaffinity="1" '
                f'friction="1.3 .05 .05" rgba=".9 .5 .25 1"/>\n'
                f'{pad}      </body>\n{pad}    </body>\n{pad}  </body>\n{pad}</body>\n')
        return out

    # --- torso_aft subtree (RL, RR legs + tail) ---
    aft_plastic = P.AFT_LEN * P.BODY_W * P.BODY_H * 1e-9 * DENS * FILL_TORSO
    aft_extra = c["battery_2s"] + c["bus_adapter"] + c["speaker"] + sv
    aft_c = (-m(P.AFT_LEN)/2, 0, 0)
    aft_sz = (m(P.AFT_LEN), m(P.BODY_W), m(P.BODY_H))
    tail_mass = P.TAIL_L * 18 * 18 * 1e-9 * DENS * FILL_TAIL + sv

    aft = ('      <body name="torso_aft" pos="0 0 0">\n'
           '        <joint name="torso_aft" type="hinge" axis="0 -1 0" '
           f'range="{P.LIM_WAIST[0]} {P.LIM_WAIST[1]}"/>\n'
           f'        {_inertial(aft_plastic+aft_extra, aft_c, aft_sz)}\n'
           f'        {_box_geom(aft_c, aft_sz)}\n')
    for leg in ("RL", "RR"):
        aft += emit_leg(leg, 4)
    aft += (f'        <body name="tail" pos="{-m(P.AFT_LEN):.5f} 0 {m(P.BODY_H/4):.5f}">\n'
            '          <joint name="tail" type="hinge" axis="0 -1 0" range="-1.0 1.0"/>\n'
            f'          {_inertial(tail_mass, (-m(P.TAIL_L)/2,0,0), (m(P.TAIL_L),m(18),m(18)))}\n'
            f'          {_box_geom((-m(P.TAIL_L)/2,0,0), (m(P.TAIL_L),m(18),m(18)))}\n'
            '        </body>\n      </body>\n')

    # --- head subtree ---
    head_plastic = 4/3*math.pi*P.HEAD_R**3 * 1e-9 * DENS * FILL_HEAD
    neck_mass = 20*20*20 * 1e-9 * DENS * FILL_MISC + sv
    ear_mass = P.EAR_BASE*8*P.EAR_H * 1e-9 * DENS * FILL_MISC + sv
    head = (f'      <body name="head_pan" pos="{m(P.FORE_LEN-12):.5f} 0 {m(P.BODY_H/2-2):.5f}">\n'
            '        <joint name="head_pan" type="hinge" axis="0 0 1" range="-1.2 1.2"/>\n'
            f'        {_inertial(neck_mass, (0,0,0), (m(20),m(20),m(20)))}\n'
            f'        {_box_geom((0,0,0), (m(20),m(20),m(20)))}\n'
            f'        <body name="head_tilt" pos="{m(4):.5f} 0 0">\n'
            f'          <joint name="head_tilt" type="hinge" axis="1 0 0" '
            f'range="{P.LIM_HEAD_TILT[0]} {P.LIM_HEAD_TILT[1]}"/>\n'
            f'          {_inertial(head_plastic+c["camera"]+sv, (m(0.3*P.HEAD_R),0,0), (m(2*P.HEAD_R),m(2*P.HEAD_R),m(2*P.HEAD_R)))}\n'
            f'          <geom type="sphere" pos="{m(0.3*P.HEAD_R):.5f} 0 0" size="{m(P.HEAD_R):.5f}" '
            'contype="0" conaffinity="0" rgba="0.82 0.83 0.88 1"/>\n'
            f'          <body name="ear_L" pos="{m(P.HEAD_R*0.3):.5f} {m(P.EYE_SPACING/2):.5f} {m(P.HEAD_R*0.7):.5f}">\n'
            '            <joint name="ear_L" type="hinge" axis="0 1 0" range="-0.6 0.6"/>\n'
            f'            {_inertial(ear_mass, (0,0,0), (m(P.EAR_BASE),m(8),m(P.EAR_H)))}\n'
            f'          </body>\n'
            f'          <body name="ear_R" pos="{m(P.HEAD_R*0.3):.5f} {-m(P.EYE_SPACING/2):.5f} {m(P.HEAD_R*0.7):.5f}">\n'
            '            <joint name="ear_R" type="hinge" axis="0 1 0" range="-0.6 0.6"/>\n'
            f'            {_inertial(ear_mass-sv, (0,0,0), (m(P.EAR_BASE),m(8),m(P.EAR_H)))}\n'
            f'          </body>\n        </body>\n      </body>\n')

    # --- root (torso_fore) ---
    fore_plastic = P.FORE_LEN * P.BODY_W * P.BODY_H * 1e-9 * DENS * FILL_TORSO
    fore_mass = fore_plastic + c["pi"] + c["imu"] + c["wiring_misc"]
    fore_c = (m(P.FORE_LEN)/2, 0, 0)
    fore_sz = (m(P.FORE_LEN), m(P.BODY_W), m(P.BODY_H))

    xml = f'''<mujoco model="sabo_primitive">
  <compiler angle="radian" autolimits="true"/>
  <option gravity="0 0 -9.81" timestep="0.002" integrator="implicitfast"/>
  <default>
    <geom rgba="0.80 0.82 0.86 1"/>
    <joint damping="0.18" armature="0.012" frictionloss="0.002"/>
    <position kp="8" forcerange="{-SERVO.stall_nm:.3f} {SERVO.stall_nm:.3f}"/>
  </default>
  <worldbody>
    <light pos="0.3 -0.3 1.2" dir="-0.2 0.2 -1" diffuse="0.9 0.9 0.9"/>
    <camera name="cam" pos="0.02 -0.52 0.20" xyaxes="1 0 0 0 0.35 1" mode="trackcom"/>
    <geom name="floor" type="plane" size="2 2 0.1" rgba="0.18 0.20 0.25 1"
          contype="1" conaffinity="1" friction="1.3 .05 .05"/>
    <body name="torso_fore" pos="0 0 {start_z:.4f}">
      <freejoint name="root"/>
      {_inertial(fore_mass, fore_c, fore_sz)}
      <geom name="torso_fore_col" type="box" pos="{fore_c[0]:.5f} {fore_c[1]:.5f} {fore_c[2]:.5f}"
            size="{fore_sz[0]/2:.5f} {fore_sz[1]/2:.5f} {fore_sz[2]/2:.5f}"
            contype="1" conaffinity="1" rgba="0.6 0.62 0.66 1"/>
{aft}'''
    for leg in ("FL", "FR"):
        xml += emit_leg(leg, 3)
    xml += head
    xml += "    </body>\n  </worldbody>\n"

    # knee->ankle coupling + linked ears (equality, exactly as the real MJCF)
    xml += "  <equality>\n"
    for leg in P.LEGS:
        c0, c1 = gait.ankle_couple_coef(leg)
        xml += (f'    <joint joint1="{leg}_ankle" joint2="{leg}_knee" '
                f'polycoef="{c0:.5f} {c1:.5f} 0 0 0"/>\n')
    xml += ('    <joint joint1="ear_R" joint2="ear_L" polycoef="0 1 0 0 0"/>\n'
            "  </equality>\n")

    xml += "  <actuator>\n"
    for leg in P.LEGS:
        xml += (f'    <position name="{leg}_hip" joint="{leg}_hip" '
                f'ctrlrange="{P.LIM_HIP[0]} {P.LIM_HIP[1]}"/>\n'
                f'    <position name="{leg}_knee" joint="{leg}_knee" '
                f'ctrlrange="{P.leg_knee_limit(leg)[0]} {P.leg_knee_limit(leg)[1]}"/>\n')
    xml += ('    <position name="torso_aft" joint="torso_aft" '
            f'ctrlrange="{P.LIM_WAIST[0]} {P.LIM_WAIST[1]}"/>\n'
            '    <position name="head_pan" joint="head_pan" ctrlrange="-1.2 1.2"/>\n'
            f'    <position name="head_tilt" joint="head_tilt" '
            f'ctrlrange="{P.LIM_HEAD_TILT[0]} {P.LIM_HEAD_TILT[1]}"/>\n'
            '    <position name="ear_L" joint="ear_L" ctrlrange="-0.6 0.6"/>\n'
            '    <position name="tail" joint="tail" ctrlrange="-1.0 1.0"/>\n')
    xml += "  </actuator>\n</mujoco>\n"
    return xml


# ---------------------------------------------------------------- evaluation
BIG = 1e6


def _run(gait_name, seconds):
    rig = MJ.Rig(xml=build_primitive_mjcf())
    _, log, fell_at, travel = MJ.simulate(gait_name, seconds, rig=rig, render=False)
    roll_pp = (max(log["roll"]) - min(log["roll"])) if log["roll"] else 0.0
    pitch_pp = (max(log["pitch"]) - min(log["pitch"])) if log["pitch"] else 0.0
    peak = max(log["tau"]) if log["tau"] else 0.0
    total_mass = float(sum(rig.model.body_mass))
    return dict(fell=fell_at, travel=travel, roll_pp=roll_pp, pitch_pp=pitch_pp,
                torque_pct=peak / SERVO.stall_nm * 100.0, mass=total_mass)


def evaluate(vec: dict) -> tuple[float, dict]:
    """Set params, build primitive model, score. Returns (score, metrics)."""
    apply_vec(vec)
    # buildability / proportion constraints (cheap, checked first)
    f_len = P.FRONT["upper"] + P.FRONT["lower"] + P.FRONT["foot"]
    r_len = P.REAR["upper"] + P.REAR["lower"] + P.REAR["foot"]
    if r_len < f_len:                       # keep hind legs >= fore (digitigrade look)
        return BIG + (f_len - r_len), dict(reason="rear<front")
    w = _run("walk", 8.0)
    if w["fell"] is not None:
        return BIG + 500 - w["fell"] * 10, dict(reason="walk fell", **w)
    if w["mass"] < P.MASS_TARGET[0] or w["mass"] > P.MASS_TARGET[1]:
        return BIG + abs(w["mass"] - 1.0) * 100, dict(reason="mass", **w)
    if w["travel"] < 0.14:
        return BIG + (0.14 - w["travel"]) * 1000, dict(reason="travel", **w)
    tr = _run("trot", 6.0)
    if tr["fell"] is not None:
        return BIG + 400 - tr["fell"] * 10, dict(reason="trot fell", walk=w, **tr)
    score = w["roll_pp"] + 0.05 * w["torque_pct"] + 0.5 * w["pitch_pp"]
    metrics = dict(score=score, roll_pp=w["roll_pp"], pitch_pp=w["pitch_pp"],
                   torque_pct=w["torque_pct"], travel=w["travel"], mass=w["mass"],
                   trot_roll=tr["roll_pp"], trot_torque=tr["torque_pct"])
    return score, metrics


# ---------------------------------------------------------------- search
def optimize(evals: int = 60, seed: int = 0, log_fn=print):
    random.seed(seed)
    base = snapshot()
    best_vec = dict(base)
    best_score, best_m = evaluate(best_vec)
    log_fn(f"[init] score={best_score:.3f}  {_fmt(best_m)}")
    n_used = 1

    # --- phase 1: coordinate descent (each param tried at a few levels) ---
    levels = [0.0, 0.25, 0.5, 0.75, 1.0]        # fractions across each bound
    order = ["BODY_W", "F_hip_off", "R_hip_off", "F_stance", "R_stance", "BODY_H",
             "R_upper", "R_lower", "F_upper", "F_lower", "R_foot", "F_foot"]
    for name in order:
        if n_used >= evals:
            break
        _, _, lo, hi = SPACE[name]
        cur = best_vec[name]
        for fr in levels:
            if n_used >= evals:
                break
            cand = dict(best_vec)
            cand[name] = lo + fr * (hi - lo)
            if abs(cand[name] - cur) < 1e-6:
                continue
            sc, m = evaluate(cand); n_used += 1
            tag = ""
            if sc < best_score:
                best_score, best_vec, best_m = sc, cand, m; tag = "  *BEST*"
            log_fn(f"[cd {n_used:2d}] {name}={cand[name]:6.1f}  score={sc:9.3f}"
                   f"  {_fmt(m)}{tag}")

    # --- phase 2: random neighbourhood search around the current best ---
    while n_used < evals:
        cand = dict(best_vec)
        for name in random.sample(NAMES, k=random.randint(1, 4)):
            _, _, lo, hi = SPACE[name]
            span = (hi - lo) * 0.18
            cand[name] = min(hi, max(lo, cand[name] + random.uniform(-span, span)))
        sc, m = evaluate(cand); n_used += 1
        tag = ""
        if sc < best_score:
            best_score, best_vec, best_m = sc, cand, m; tag = "  *BEST*"
        log_fn(f"[rs {n_used:2d}] score={sc:9.3f}  {_fmt(m)}{tag}")

    apply_vec(base)     # leave params untouched unless the caller applies the winner
    return best_vec, best_score, best_m


def _fmt(m: dict) -> str:
    if "reason" in m and "roll_pp" not in m:
        return f"REJECT({m['reason']})"
    parts = []
    if "roll_pp" in m:
        parts.append(f"roll={m['roll_pp']:.2f}")
    if "pitch_pp" in m:
        parts.append(f"pitch={m['pitch_pp']:.2f}")
    if "torque_pct" in m:
        parts.append(f"tau={m['torque_pct']:.0f}%")
    if "travel" in m:
        parts.append(f"trav={m['travel']*100:.0f}cm")
    if "mass" in m:
        parts.append(f"m={m['mass']*1000:.0f}g")
    if "reason" in m:
        parts.append(f"REJECT({m['reason']})")
    return " ".join(parts)


def main():
    ap = argparse.ArgumentParser()
    ap.add_argument("--evals", type=int, default=60)
    ap.add_argument("--seed", type=int, default=0)
    ap.add_argument("--apply", action="store_true",
                    help="write the winning dimensions into cad/params.py")
    args = ap.parse_args()

    base = snapshot()
    print("=" * 70)
    print(f"Sabo dimensional optimizer — {args.evals} evals (primitive fast model)")
    print("=" * 70)
    best_vec, best_score, best_m = optimize(args.evals, args.seed)
    print("-" * 70)
    print(f"BEST score={best_score:.3f}  {_fmt(best_m)}")
    print(f"{'param':12s}{'before':>10s}{'after':>10s}")
    for n in NAMES:
        print(f"{n:12s}{base[n]:10.1f}{best_vec[n]:10.1f}")

    if args.apply:
        path = write_params(best_vec)
        print(f"\n[apply] wrote winning dimensions into {path}")


# ---------------------------------------------------------------- write-back
def write_params(vec: dict, path: str | None = None) -> str:
    """Patch the optimized scalars/leg-dict fields in ``cad/params.py`` in place.

    Rewrites ``BODY_W`` / ``BODY_H`` and the ``hip_off``/``upper``/``lower``/``foot``/
    ``stance_depth`` fields of the ``FRONT`` and ``REAR`` dicts. Other lines (and the
    coupling coefficients) are untouched."""
    import os
    import re
    if path is None:
        path = os.path.join(os.path.dirname(__file__), "..", "cad", "params.py")
        path = os.path.abspath(path)
    with open(path, "r", encoding="utf-8") as f:
        src = f.read()

    def sub_scalar(text, name, val):
        return re.sub(rf"(?m)^{name}\s*=\s*[-\d.]+", f"{name} = {val:.1f}", text)

    src = sub_scalar(src, "BODY_W", vec["BODY_W"])
    src = sub_scalar(src, "BODY_H", vec["BODY_H"])

    def sub_field(text, dict_name, field, val):
        # match the field inside a FRONT/REAR = dict(...) block (spanning lines)
        pat = rf"({dict_name}\s*=\s*dict\((?:[^)]*?){field}=)[-\d.]+"
        return re.sub(pat, rf"\g<1>{val:.1f}", text, count=1)

    for side, dname in (("F", "FRONT"), ("R", "REAR")):
        for field in ("hip_off", "upper", "lower", "foot", "stance_depth"):
            key = {"hip_off": f"{side}_hip_off", "upper": f"{side}_upper",
                   "lower": f"{side}_lower", "foot": f"{side}_foot",
                   "stance_depth": f"{side}_stance"}[field]
            src = sub_field(src, dname, field, vec[key])

    with open(path, "w", encoding="utf-8") as f:
        f.write(src)
    return path


if __name__ == "__main__":
    main()
