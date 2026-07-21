"""
Assembly — cat body with a waist joint + head tilt, and the kinematic chain.
============================================================================

Tree (root = ``torso_fore``, a free body):

    torso_fore ─ FL,FR legs (hip+knee, coupled ankle)
               ├ head_pan ─ head_tilt ─ ear_L, ear_R
               └ waist ─▶ torso_aft ─ RL,RR legs
                                     └ tail

Products: ``kinematics()`` (joint tree for MJCF/validate), ``PRINTABLE`` (per-part
STLs), ``full_robot()`` (posed preview). Frame: x fwd, y left, z up; each torso
half's frame origin is at the waist.
"""

from __future__ import annotations

import math
from dataclasses import dataclass

from build123d import Box, Part, Pos, Rotation, Sphere, scale

from cad import params as P
from cad.parts.body import torso_aft, torso_fore
from cad.parts.ears import ear
from cad.parts.head import head
from cad.parts.leg import crank, leg_parts, pushrod, stance_linkage
from cad.parts.tail import tail
from cad.servo import DEFAULT as SERVO
from sim.gait import ankle_couple_coef, ankle_from_knee, stance_angles

ROOT = "torso_fore"
# short organic neck stub carrying the pan joint: a rounded, forward-leaning
# ellipsoid (not a box) — same envelope, just softened. Internal (hidden in sim).
_NECK = Pos(2, 0, 1) * scale(Sphere(1), (10.0, 10.5, 12.0))
_PIVOT = scale(Sphere(1), (8.0, 9.0, 9.0))   # small gimbal connector (pitch link)


@dataclass
class Link:
    name: str
    parent: str
    origin: tuple[float, float, float]
    axis: tuple[float, float, float]
    limits: tuple[float, float]
    part: Part
    extra_mass: float = 0.0
    has_joint: bool = True
    actuated: bool = True
    couple: tuple | None = None


def kinematics() -> list[Link]:
    links: list[Link] = []
    Y = (0, -1, 0)
    # waist: aft half hangs off the fore half via a sagittal spine joint.
    # extra_mass: battery + bus adapter + speaker + waist servo + the TWO REAR HIP servos,
    # whose BODIES now live in the aft torso core (remote-axle hip drive) instead of out
    # on the legs. (The two FRONT hip servos likewise move into the fore core = the root;
    # they are added to the root inertial in sim/mjcf.py.)
    links.append(Link("torso_aft", ROOT, (0, 0, 0), Y, P.LIM_WAIST, torso_aft(),
                      extra_mass=P.COMPONENT_MASS["battery_2s"] + P.COMPONENT_MASS["bus_adapter"]
                      + P.COMPONENT_MASS["speaker"] + SERVO.mass_kg          # waist servo
                      + 2 * SERVO.mass_kg))                                  # RL, RR hip servos (core)
    for leg in P.LEGS:
        mx, my = P.MOUNTS[leg]
        s = P.leg_plane_sign(leg)
        g = P.leg_geom(leg)
        parts = leg_parts(leg)
        c0, c1 = ankle_couple_coef(leg)
        parent = P.leg_parent(leg)
        # abduction link (rigid): carries the slim hip BEARING BRACKET. The hip servo
        # BODY that used to ride here (extra_mass) has moved into the torso core (its
        # mass is now on torso_aft / the root); only the lightweight bracket remains.
        links.append(Link(f"{leg}_abd", parent, (mx, my, 0), (1, 0, 0), P.LIM_ABD,
                          parts["hip_bracket"], extra_mass=0.0,
                          has_joint=False, actuated=False))
        links.append(Link(f"{leg}_hip", f"{leg}_abd", (0, s*g["hip_off"], 0), Y,
                          P.LIM_HIP, parts["upper"], extra_mass=SERVO.mass_kg))
        links.append(Link(f"{leg}_knee", f"{leg}_hip", (0, 0, -g["upper"]), Y,
                          P.leg_knee_limit(leg), parts["lower"], extra_mass=0.0))
        links.append(Link(f"{leg}_ankle", f"{leg}_knee", (0, 0, -g["lower"]), Y,
                          P.LIM_ANKLE, parts["foot"], actuated=False,
                          couple=(f"{leg}_knee", c0, c1)))

    # head: a 2-axis camera gimbal + nod/tilt expression.
    #   pan (yaw, z) -> pitch (nod, y) -> tilt (roll, x) -> head shell.
    # Nestled onto the front-top of the chest. The pan+pitch+tilt chain lets the
    # head hold the camera level in roll AND pitch while the body moves (gimbal),
    # and do the cute nod/head-tilt while still. Head stays at the same place: the
    # 4mm forward offset is split across the pitch + tilt links.
    links.append(Link("head_pan", ROOT, (P.FORE_LEN - 12, 0, P.BODY_H/2 - 2),
                      (0, 0, 1), (-1.2, 1.2), _NECK, extra_mass=SERVO.mass_kg))
    links.append(Link("head_pitch", "head_pan", (2, 0, 0), (0, 1, 0),
                      P.LIM_HEAD_PITCH, _PIVOT, extra_mass=SERVO.mass_kg))
    links.append(Link("head_tilt", "head_pitch", (2, 0, 0), (1, 0, 0),
                      P.LIM_HEAD_TILT, head(),
                      extra_mass=P.COMPONENT_MASS["camera"] + SERVO.mass_kg))
    links.append(Link("ear_L", "head_tilt", (P.HEAD_R*0.3, P.EYE_SPACING/2, P.HEAD_R*0.7),
                      (0, 1, 0), (-0.6, 0.6), ear(), extra_mass=SERVO.mass_kg))
    links.append(Link("ear_R", "head_tilt", (P.HEAD_R*0.3, -P.EYE_SPACING/2, P.HEAD_R*0.7),
                      (0, 1, 0), (-0.6, 0.6), ear(), extra_mass=0.0,
                      actuated=not P.EARS_LINKED,
                      couple=("ear_L", 0.0, 1.0) if P.EARS_LINKED else None))
    links.append(Link("tail", "torso_aft", (-P.AFT_LEN, 0, P.BODY_H/4), (0, -1, 0),
                      (-1.0, 1.0), tail(), extra_mass=SERVO.mass_kg))
    return links


def n_motors() -> int:
    return sum(1 for lk in kinematics() if lk.actuated)


PRINTABLE = {
    "torso_fore": torso_fore(), "torso_aft": torso_aft(),
    "head": head(), "ear": ear(), "tail": tail(),
    "hipbr_F_L": leg_parts("FL")["hip_bracket"], "hipbr_F_R": leg_parts("FR")["hip_bracket"],
    "hipbr_R_L": leg_parts("RL")["hip_bracket"], "hipbr_R_R": leg_parts("RR")["hip_bracket"],
    "upper_F": leg_parts("FL")["upper"], "lower_F": leg_parts("FL")["lower"],
    "foot_F": leg_parts("FL")["foot"],
    "upper_R": leg_parts("RL")["upper"], "lower_R": leg_parts("RL")["lower"],
    "foot_R": leg_parts("RL")["foot"],
    # four-bar knee linkage — crank + pushrod share FOURBAR, so F/R are identical
    # geometry, but listed per leg-type per the build convention.
    "crank_F": crank(), "crank_R": crank(),
    "pushrod_F": pushrod(), "pushrod_R": pushrod(),
}


def _leg_locations(leg, hip, knee, ankle):
    mx, my = P.MOUNTS[leg]; s = P.leg_plane_sign(leg); g = P.leg_geom(leg)
    d = lambda r: -math.degrees(r)
    T_abd = Pos(mx, my, 0) * Rotation(0, 0, 0)
    T_hip = T_abd * Pos(0, s*g["hip_off"], 0) * Rotation(0, d(hip), 0)
    T_knee = T_hip * Pos(0, 0, -g["upper"]) * Rotation(0, d(knee), 0)
    T_ankle = T_knee * Pos(0, 0, -g["lower"]) * Rotation(0, d(ankle), 0)
    return T_abd, T_hip, T_knee, T_ankle


def _linkage_transforms(leg):
    """(crank, pushrod) transforms in the UPPER-LEG local frame that pose the verified
    four-bar at the mid-window stance pose. The fourbar frame (knee at origin) maps
    identically into the thigh frame with the knee at (0,0,-upper)."""
    upper = P.leg_geom(leg)["upper"]
    t2, pts = stance_linkage()
    loc = lambda p: (p[0], 0.0, -upper + p[1])            # (x_fb, y_fb) -> (x, 0, z)
    O2, C, R = loc(pts["O2"]), loc(pts["C"]), loc(pts["R"])
    T_crank = Pos(*O2) * Rotation(0, -math.degrees(t2), 0)   # +x arm -> crank tip C
    psi = math.atan2(R[2] - C[2], R[0] - C[0])
    T_push = Pos(*C) * Rotation(0, -math.degrees(psi), 0)    # +x rod C -> rocker tip R
    return T_crank, T_push


def full_robot() -> Part:
    lift = max(g["stance_depth"] + g["foot"] * 0.6 for g in (P.FRONT, P.REAR))
    base = Pos(0, 0, lift)
    parts = [base * torso_fore(), base * torso_aft()]
    for leg in P.LEGS:
        hip, knee = stance_angles(leg)
        ankle = ankle_from_knee(leg, knee)
        Ta, Th, Tk, Tan = _leg_locations(leg, hip, knee, ankle)
        pl = leg_parts(leg)
        parts += [base*Ta*pl["hip_bracket"], base*Th*pl["upper"],
                  base*Tk*pl["lower"], base*Tan*pl["foot"]]
        # four-bar crank + pushrod live in the thigh frame (posed by T_hip)
        Tcr, Tpr = _linkage_transforms(leg)
        parts += [base*Th*Tcr*crank(), base*Th*Tpr*pushrod()]
    parts.append(Pos(P.FORE_LEN + P.NECK_L*0.4, 0, lift + P.BODY_H/2 + 8) * head())
    parts.append(Pos(-P.AFT_LEN - 4, 0, lift + P.BODY_H/4) * (Rotation(0, -35, 0) * tail()))
    for s in (+1, -1):
        parts.append(Pos(P.FORE_LEN + P.NECK_L*0.4, s*P.EYE_SPACING/2,
                         lift + P.BODY_H/2 + P.HEAD_R*0.7) * ear())
    fused = parts[0]
    for p in parts[1:]:
        fused += p
    return fused


if __name__ == "__main__":
    print(f"{len(kinematics())} links, {n_motors()} MOTORS")
    for leg in ("FL", "RL"):
        hip, knee = stance_angles(leg)
        print(f"  {leg}: hip={math.degrees(hip):.0f} knee={math.degrees(knee):.0f} "
              f"ankle*={math.degrees(ankle_from_knee(leg, knee)):.0f}")
    print("full robot volume mm^3:", round(full_robot().volume, 1))
