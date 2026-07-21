"""
Gait + leg IK (2 motors/leg, coupled hock).
===========================================

Each leg has two motors — hip + knee — that place the **ankle** with a classic
2-link solve. The metatarsus/hock is mechanically **coupled to the knee**
(ankle = c0 + c1·knee), like a cat's reciprocal-apparatus tendon, so it follows
along for free and keeps the digitigrade toe contact.

Angle convention (matches analysis/validate.py): straight down = 0; +hip swings
the limb forward (+x). MJCF gives hip/knee axis (0,-1,0) to share this sign.
"""

from __future__ import annotations

import math

from cad import params as P

# gait timing (phase offset per leg in [0,1))
_DIAGONAL = {"FL": 0.0, "RR": 0.0, "FR": 0.5, "RL": 0.5}          # trot: diagonal pairs
_LATERAL = {"RL": 0.0, "FL": 0.25, "RR": 0.5, "FR": 0.75}        # walk: 4-beat, cat sequence


def leg_ik(leg: str, x: float, depth: float) -> tuple[float, float]:
    """(hip, knee) to place the ANKLE at (x fwd, depth down) for this leg."""
    g = P.leg_geom(leg)
    upper, lower = g["upper"], g["lower"]
    d = math.hypot(x, depth)
    d = min(d, upper + lower - 0.5)
    d = max(d, abs(upper - lower) + 0.5)
    ck = (upper*upper + lower*lower - d*d) / (2*upper*lower)
    knee = math.pi - math.acos(max(-1, min(1, ck)))
    cb = (upper*upper + d*d - lower*lower) / (2*upper*d)
    beta = math.acos(max(-1, min(1, cb)))
    hip = math.atan2(x, depth) - beta
    return hip, knee


def stance_angles(leg: str) -> tuple[float, float]:
    """Standing (hip, knee): ankle straight below the hip at the stance depth."""
    return leg_ik(leg, 0.0, P.leg_geom(leg)["stance_depth"])


def ankle_couple_coef(leg: str) -> tuple[float, float]:
    """(c0, c1) for the knee->ankle coupling: ankle = c0 + c1*knee."""
    g = P.leg_geom(leg)
    _, knee_st = stance_angles(leg)
    c1 = g["couple_c1"]
    c0 = g["couple_ankle"] - c1 * knee_st
    return c0, c1


def ankle_from_knee(leg: str, knee: float) -> float:
    c0, c1 = ankle_couple_coef(leg)
    return c0 + c1 * knee


def foot_target(leg: str, phase: float, preset: dict, depth: float) -> tuple[float, float]:
    """Cyclic ankle target (x_fwd_mm, depth_mm) for a leg at gait ``phase``.

    ``duty`` is the fraction of the cycle a foot is planted. A walk uses
    duty≈0.75 so 3 feet are always down (statically stable — minimal wobble); a
    trot uses 0.5 (diagonal pairs)."""
    stride, step_h, duty = preset["stride"], preset["step_h"], preset["duty"]
    ph = (phase + preset["phase"][leg]) % 1.0
    if ph < duty:                                    # stance
        s = ph / duty
        return stride / 2 - stride * s, depth
    s = (ph - duty) / (1 - duty)                     # swing
    return -stride / 2 + stride * s, depth - step_h * math.sin(math.pi * s)


PRESETS = {
    "stand": dict(stride=0.0, step_h=0.0, cycle=1.0, duty=1.0, phase=_DIAGONAL, settle=0.0),
    # watch: the cat-detection pose — still + settled low (lower CoM) so the
    # head-mounted camera is rock-steady. The brain holds this while looking.
    "watch": dict(stride=0.0, step_h=0.0, cycle=1.0, duty=1.0, phase=_DIAGONAL, settle=14.0),
    # walk: 4-beat lateral sequence. Higher duty (0.78) keeps 3+ feet down more of
    # the time (statically stable -> less passive roll), a slower cycle gives each
    # foot a gentle plant, and a small settle lowers the CoM so the torso is less
    # top-heavy and rolls less. step_h kept low so the foot lands gently
    # (drop-impact was pinning the servo). ``level`` = full authority for the IMU
    # body-leveling PD (mj_emulate), which trims out the residual waddle-roll.
    "walk":  dict(stride=38.0, step_h=10.0, cycle=1.45, duty=0.78, phase=_LATERAL, settle=4.0,
                  level=1.0, spine_amp=0.045, spine_off=0.5),
    # trot: diagonal pairs; gentle (no abduction to catch a lateral tip); low step_h
    # for a soft touchdown so peak leg torque stays well under the servo stall.
    # ``level`` fractional: the trot already runs near the servo stall, so it gets
    # only partial leveling authority — enough to roughly halve its roll without
    # the depth trim adding to peak torque.
    "trot":  dict(stride=34.0, step_h=12.0, cycle=0.8, duty=0.5, phase=_DIAGONAL, settle=0.0,
                  level=0.5),
}


def leg_depth(leg: str) -> float:
    return P.leg_geom(leg)["stance_depth"]


def spine_wave(phase: float, preset: dict) -> float:
    """Waist (torso_aft) setpoint (rad) for a subtle feline spine undulation, in
    phase with the gait. A walking cat's back gently flexes/extends twice per stride
    (once per lateral couplet); we drive the sagittal waist joint with a small 2×
    sinusoid so the spine reads alive without shifting the CoM enough to add roll.
    Amplitude ``spine_amp`` (rad, default 0) and ``spine_off`` (cycle-phase offset)
    come from the preset; a bias toward gentle flex (belly-down, -) keeps the
    top-line low and the CoM settled. This is a real servo setpoint (torso_aft),
    so it runs identically on hardware."""
    amp = preset.get("spine_amp", 0.0)
    if amp == 0.0:
        return 0.0
    off = preset.get("spine_off", 0.0)
    return amp * math.sin(2 * math.pi * (2.0 * phase + off))


# --------------------------------------------------------------- four-bar knee transmission
# The knee is physically driven by a thigh-mounted crank through a rigid four-bar
# (analysis/fourbar.py, geometry P.FOURBAR). The gait/IK above works in the KNEE
# JOINT angle (the sim treats the knee as a direct hinge — validated). On HARDWARE
# the knee *servo* commands the CRANK, so its command must be mapped through the
# linkage's nonlinear ratio: crank = knee_to_crank(leg, knee_joint).
#
# The map is affine-consistent with the as-built CAD: the rocker is welded (leg.py)
# so that at each leg's stance the linkage sits at the crank window's MID-POINT.
# Hence  knee_joint = knee_fb + OFFSET(leg),  OFFSET = stance_knee - knee_fb(mid),
# where knee_fb is the four-bar's internal knee variable. We precompute the crank↔
# knee_fb sweep ONCE (monotonic over the window) and linearly interpolate — O(1)
# per call, cheap enough for per-tick control.

from analysis.fourbar import FourBar as _FourBar   # noqa: E402


def _fb() -> _FourBar:
    fb = P.FOURBAR
    return _FourBar(d=fb["ground"], r2=fb["crank"], r3=fb["coupler"],
                    r4=fb["rocker"], rocker_offset=fb["rocker_offset"])


# crank↔knee_fb table, sorted ascending in knee_fb for interpolation
_LO_DEG, _HI_DEG = P.FOURBAR["crank_window"]
_LO, _HI = math.radians(_LO_DEG), math.radians(_HI_DEG)
_KNEE_FB_MID = _fb().solve(math.radians((_LO_DEG + _HI_DEG) / 2))[1]
_SWEEP = sorted(((kf, t2) for t2, kf, _mu in _fb().sweep(_LO, _HI, 400)),
                key=lambda r: r[0])
_KFB = [r[0] for r in _SWEEP]     # knee_fb (ascending)
_CRK = [r[1] for r in _SWEEP]     # matching crank angle


def _knee_offset(leg: str) -> float:
    """OFFSET so knee_joint = knee_fb + OFFSET (from the stance/mid-window anchor)."""
    return stance_angles(leg)[1] - _KNEE_FB_MID


def _interp(xs: list, ys: list, x: float) -> float:
    if x <= xs[0]:
        return ys[0]
    if x >= xs[-1]:
        return ys[-1]
    lo, hi = 0, len(xs) - 1
    while hi - lo > 1:                       # binary search the bracketing pair
        mid = (lo + hi) // 2
        if xs[mid] <= x:
            lo = mid
        else:
            hi = mid
    t = (x - xs[lo]) / (xs[hi] - xs[lo])
    return ys[lo] + t * (ys[hi] - ys[lo])


def knee_to_crank(leg: str, knee_joint: float) -> float:
    """Knee JOINT angle (what the gait/IK produces) → CRANK (knee-servo) angle for
    the four-bar drive. This is the transmission map the hardware knee servo uses;
    the sim keeps the knee as a direct hinge so it never calls this."""
    knee_fb = knee_joint - _knee_offset(leg)
    return _interp(_KFB, _CRK, knee_fb)


def crank_to_knee(leg: str, crank: float) -> float:
    """Inverse: CRANK (servo) angle → knee JOINT angle. (Feedback / telemetry.)"""
    order = sorted(zip(_CRK, _KFB))          # ascending in crank
    kf = _interp([c for c, _ in order], [k for _, k in order], crank)
    return kf + _knee_offset(leg)


if __name__ == "__main__":
    print("Four-bar knee transmission map (knee joint <-> crank servo)")
    print("=" * 60)
    print(f"  crank window {_LO_DEG:.0f}..{_HI_DEG:.0f} deg,  knee_fb(mid)={math.degrees(_KNEE_FB_MID):.1f} deg")
    for leg in ("FL", "RL"):
        _, kst = stance_angles(leg)
        off = _knee_offset(leg)
        cst = knee_to_crank(leg, kst)
        rt = crank_to_knee(leg, cst)
        lo_k, hi_k = P.leg_knee_limit(leg)
        cranks = [math.degrees(knee_to_crank(leg, math.radians(d)))
                  for d in range(int(math.degrees(lo_k)), int(math.degrees(hi_k)) + 1, 10)]
        mono = all(cranks[i] <= cranks[i+1] + 1e-6 for i in range(len(cranks)-1)) or \
               all(cranks[i] >= cranks[i+1] - 1e-6 for i in range(len(cranks)-1))
        print(f"  {leg}: stance knee={math.degrees(kst):.1f} deg -> crank={math.degrees(cst):.1f} deg "
              f"(round-trip {math.degrees(rt):.1f} deg), offset={math.degrees(off):.1f} deg")
        print(f"      crank over knee ROM {math.degrees(lo_k):.0f}..{math.degrees(hi_k):.0f} deg: "
              f"{min(cranks):.0f}..{max(cranks):.0f} deg, monotonic={mono}")
