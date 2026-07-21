"""
Engineering validation — is the cat-anatomy design buildable on these servos?
=============================================================================

    python -m analysis.validate

Runs off the parametric CAD (no physics), PASS/FAIL on:
  1. MASS     — total mass inside the target band?
  2. STANCE   — do the 4-DOF digitigrade stance angles sit inside servo travel?
  3. BALANCE  — centre of mass inside the four-toe support polygon?
  4. TORQUE   — worst-case static joint torque over the leg workspace vs servo stall,
                per front and rear legs (trot sizing case: W/2 per stance toe).
"""

from __future__ import annotations

import math

from cad import params as P
from cad.assembly import PRINTABLE, kinematics
from cad.export import COUNTS
from cad.servo import DEFAULT as SERVO
from sim import gait

G = 9.81
SAFETY = 2.0


def part_masses() -> dict[str, float]:
    return {n: p.volume * 1e-9 * P.EFFECTIVE_DENSITY for n, p in PRINTABLE.items()}


def total_mass():
    pm = part_masses()
    printed = sum(pm[n] * COUNTS.get(n, 1) for n in pm)
    comp = P.component_mass_total()
    return printed, comp, printed + comp


def leg_fk(leg, hip, knee, ankle):
    """Joint + toe positions (x fwd, z up), hip at origin, for one leg pose."""
    g = P.leg_geom(leg)
    hx, hz = 0.0, 0.0
    kx = hx + g["upper"] * math.sin(hip)
    kz = hz - g["upper"] * math.cos(hip)
    ax = kx + g["lower"] * math.sin(hip + knee)
    az = kz - g["lower"] * math.cos(hip + knee)
    tx = ax + g["foot"] * math.sin(hip + knee + ankle)
    tz = az - g["foot"] * math.cos(hip + knee + ankle)
    return {"hip": (hx, hz), "knee": (kx, kz), "ankle": (ax, az), "toe": (tx, tz)}


def check_mass():
    printed, comp, total = total_mass()
    lo, hi = P.MASS_TARGET
    ok = lo <= total <= hi
    return ok, (f"total {total*1000:.0f} g  (plastic {printed*1000:.0f} + "
                f"components {comp*1000:.0f}; target {lo*1000:.0f}-{hi*1000:.0f} g)")


def check_stance():
    bad = []
    for leg in P.LEGS:
        lims = {"hip": P.LIM_HIP, "knee": P.leg_knee_limit(leg), "ankle": P.LIM_ANKLE}
        hip, knee = gait.stance_angles(leg)
        ankle = gait.ankle_from_knee(leg, knee)
        for jn, v in (("hip", hip), ("knee", knee), ("ankle", ankle)):
            lo, hi = lims[jn]
            if not (lo <= v <= hi):
                bad.append(f"{leg}.{jn}")
    ok = not bad
    def s(leg):
        h, k = gait.stance_angles(leg)
        return [round(math.degrees(h)), round(math.degrees(k)),
                round(math.degrees(gait.ankle_from_knee(leg, k)))]
    return ok, (f"front(hip,knee,ankle*)={s('FL')}° rear={s('RL')}° "
                f"(*coupled) " + ("within limits" if ok else f"OUT: {bad}"))


def check_balance():
    pm = part_masses()
    _, _, total = total_mass()
    feet = [(P.MOUNTS[l][0], P.MOUNTS[l][1] + P.leg_plane_sign(l) * P.leg_geom(l)["hip_off"])
            for l in P.LEGS]
    xs = [f[0] for f in feet]; ys = [f[1] for f in feet]
    # each leg lost its hip servo to the torso core (remote-axle hip drive): the leg now
    # carries 2 servos (crank/knee on the thigh + the abduction-mount allowance) instead
    # of 3, and the 4 hip servos are added to the torso at the core (near the centreline).
    leg_mass = {l: (sum(pm[k] for k in _leg_parts_of(l)) + 2 * SERVO.mass_kg) for l in P.LEGS}
    contribs = [
        (P.FORE_LEN/2, 0, pm["torso_fore"] + P.COMPONENT_MASS["pi"] + P.COMPONENT_MASS["imu"]),
        (-P.AFT_LEN/2, 0, pm["torso_aft"] + P.COMPONENT_MASS["battery_2s"]
         + P.COMPONENT_MASS["bus_adapter"] + P.COMPONENT_MASS["speaker"]),
        (P.FORE_LEN + P.NECK_L, 0, pm["head"] + P.COMPONENT_MASS["camera"]),
        (-P.AFT_LEN - P.TAIL_L/3, 0, pm["tail"]),
    ]
    for l in P.LEGS:
        mx, my = P.MOUNTS[l]
        contribs.append((mx, my + P.leg_plane_sign(l)*P.leg_geom(l)["hip_off"]/2, leg_mass[l]))
        # hip servo body relocated into the torso core, on the hip axis (x=mx) near the
        # centreline (its horn face at |y|=HIP_CORE_HORN_Y, body reaching inboard).
        contribs.append((mx, P.leg_plane_sign(l) * (P.HIP_CORE_HORN_Y - 18.0), SERVO.mass_kg))
    M = sum(c[2] for c in contribs)
    cx = sum(c[0]*c[2] for c in contribs)/M
    cy = sum(c[1]*c[2] for c in contribs)/M
    inside = (min(xs) < cx < max(xs)) and (min(ys) < cy < max(ys))
    margin = min(cx - min(xs), max(xs) - cx)
    return inside, (f"CoM=({cx:+.0f},{cy:+.0f}) mm inside toe box "
                    f"x[{min(xs):.0f},{max(xs):.0f}] y[{min(ys):.0f},{max(ys):.0f}]; "
                    f"fore/aft margin {margin:.0f} mm")


def _leg_parts_of(leg):
    suf = "F" if P.is_front(leg) else "R"
    s = "L" if leg.endswith("L") else "R"
    return [f"upper_{suf}", f"lower_{suf}", f"foot_{suf}", f"hipbr_{suf}_{s}"]


def check_torque():
    """Static torque on the two motors/leg (hip, knee). The knee motor also holds
    the coupled hock through the linkage, so we size it on the full GRF lever."""
    _, _, total = total_mass()
    F = total * G / 2.0
    allow = SERVO.stall_nm / SAFETY
    peak = {}
    for kind, leg in (("front", "FL"), ("rear", "RL")):
        p = {"hip": 0.0, "knee": 0.0}
        g = P.leg_geom(leg)
        for xi in range(-45, 46, 5):
            for depth in range(int(g["stance_depth"])-10, int(g["stance_depth"])+14, 4):
                try:
                    hip, knee = gait.leg_ik(leg, float(xi), float(depth))
                except Exception:
                    continue
                ankle = gait.ankle_from_knee(leg, knee)
                fk = leg_fk(leg, hip, knee, ankle)
                tx = fk["toe"][0]
                p["hip"] = max(p["hip"], F*abs(tx - fk["hip"][0])/1000.0)
                p["knee"] = max(p["knee"], F*abs(tx - fk["knee"][0])/1000.0)
        peak[kind] = p
    worst = max(v for p in peak.values() for v in p.values())
    ok = worst <= allow
    lines = []
    for kind in ("front", "rear"):
        for j in ("hip", "knee"):
            t = peak[kind][j]
            lines.append(f"      {kind:5s} {j:5s}: {t:.2f} N·m ({t/SERVO.stall_nm*100:.0f}%)"
                         + ("  FAIL" if t > allow else ""))
    detail = (f"stall {SERVO.stall_nm:.2f} N·m, allow {allow:.2f} (SF {SAFETY:.0f}); "
              f"F/toe={F:.1f} N; 2 motors/leg (ankle coupled)\n" + "\n".join(lines))
    return ok, detail


def main():
    print("=" * 66)
    print(f"Sabo cat-anatomy validation — servo: {SERVO.name} "
          f"({SERVO.stall_kgcm:.0f} kg·cm, {'quiet' if SERVO.quiet else 'NOISY'})")
    print("=" * 66)
    all_ok = True
    for name, fn in [("MASS", check_mass), ("STANCE", check_stance),
                     ("BALANCE", check_balance), ("TORQUE", check_torque)]:
        ok, detail = fn(); all_ok &= ok
        print(f"[{'PASS' if ok else 'FAIL'}] {name:8s} {detail}")
    print("-" * 66)
    print("RESULT:", "ALL CHECKS PASS." if all_ok else "FAILURES — adjust before printing.")
    return all_ok


if __name__ == "__main__":
    import sys
    sys.exit(0 if main() else 1)
