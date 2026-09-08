"""
Tail — a tapered, gently up-curving appendage on a REMOTE four-bar drive.

Local frame: base pivot at origin, tail extending -x (rearward) and curving up; the
resting curve is modelled here, articulated in sim.

Why the drive is remote (params.TAIL_DRIVE)
-------------------------------------------
The tail pivot sits at the rear extremity of the frame, where the ribcage has tapered to
almost nothing. ``analysis.actuator_fit`` measures that no STS3215 can be housed there —
44.7% of its boss falls outside the body, and no orientation helps. So the actuator moves
forward to the one station in the aft torso where it does fit and reaches the joint through
a crank-rocker four-bar, exactly the way the knee is driven from up the thigh.

    servo horn -> crank (P.TAIL_FOURBAR['crank'])
               -> pushrod (coupler)
               -> rocker welded on the tail base   [2.08 rad of travel, mu 42..140 deg]

Link lengths are verified against ``analysis.fourbar`` (the same tool the knee uses).
"""

from __future__ import annotations

import math

from build123d import (BuildPart, BuildSketch, Box, Circle, Locations, Part,
                       Plane, Pos, Rot, loft)

from cad import params as P
from cad.parts import fasteners as F
from cad.parts.leg import _cyl_y, _flat_arm, _pin_bore, LINK_W, PIN_R

ROCKER_HUB_R = LINK_W * 0.6
PIVOT_FORK_R = 9.0          # the tail's fork cheeks at the base pivot


# ----------------------------------------------------------------- four-bar placement
def _frame():
    """Map four-bar (u, v) coordinates into the AFT-TORSO (x, z) plane.

    ``analysis.fourbar`` works in its own frame with the rocker pivot O4 at the origin and
    the crank pivot O2 straight 'up' at (0, d). Here O4 is the tail pivot and O2 is the
    servo's output shaft, so the whole linkage is that frame rotated onto the line between
    them. The perpendicular is chosen so the rocker swings into free air BEHIND the body
    rather than up through the ribcage.
    """
    ox, oz = -P.AFT_LEN, P.BODY_H / 4          # O4, the tail pivot
    sx, sz = P.TAIL_SERVO                      # O2, the servo shaft
    d = math.hypot(sx - ox, sz - oz)
    ev = ((sx - ox) / d, (sz - oz) / d)
    eu = (ev[1], -ev[0])
    return (ox, oz), eu, ev


def to_aft(p: tuple[float, float]) -> tuple[float, float]:
    """A four-bar point -> (x, z) in the aft-torso frame."""
    (ox, oz), eu, ev = _frame()
    u, v = p
    return (ox + u * eu[0] + v * ev[0], oz + u * eu[1] + v * ev[1])


def tail_linkage():
    """The tail four-bar at its mid-window (neutral) crank pose.

    Returns ``(t2_mid_rad, pts_aft)`` where ``pts_aft`` maps O2/C/R/O4 into the aft-torso
    (x, z) frame — the tail's own local frame is the same, translated to O4.
    """
    from analysis.fourbar import FourBar
    fb = P.TAIL_FOURBAR
    bar = FourBar(d=P.tail_ground(), r2=fb["crank"], r3=fb["coupler"], r4=fb["rocker"])
    lo, hi = fb["crank_window"]
    t2 = math.radians((lo + hi) / 2)
    pts = bar.linkage_points(t2)
    return t2, {k: to_aft(v) for k, v in pts.items() if isinstance(v, tuple)}


def _rocker_angle_deg() -> float:
    """Weld angle of the rocker on the tail base, so its tip lands on the four-bar's R
    point when the tail sits at its neutral pose."""
    _, pts = tail_linkage()
    ox, oz = -P.AFT_LEN, P.BODY_H / 4
    rx, rz = pts["R"]
    return math.degrees(math.atan2(rz - oz, rx - ox))


# ----------------------------------------------------------------- printed parts
def tail_crank() -> Part:
    """Crank on the tail servo's horn: horn interface at the pivot (origin), the
    crank-coupler pin at (crank, 0, 0). One flat part; flip it for a mirrored build."""
    r = P.TAIL_FOURBAR["crank"]
    t = P.CLEVIS_TONGUE_T
    body = _flat_arm(r, r0=P.HORN_DIA / 2 + 2.0, r1=PIN_R + 3.0, t=t)
    body -= F.horn_holes(axis="y", length=t + 2)
    body -= Pos(0, -(t / 2 - P.HORN_DISC_T / 2), 0) * _cyl_y(
        P.HORN_DIA / 2 + P.HORN_SEAT_CLEAR, P.HORN_DISC_T)      # horn disc seat
    body -= Pos(r, 0, 0) * _pin_bore()
    return body


def tail_pushrod() -> Part:
    """Coupler between the crank tip and the rocker tip, in its own lane so it can share
    both pins with in-plane links."""
    L = P.TAIL_FOURBAR["coupler"]
    t = P.CLEVIS_ROD_T
    hub = PIN_R + 3.0
    body = _flat_arm(L, r0=hub, r1=hub, t=t)
    for x in (0.0, L):
        body -= Pos(x, 0, 0) * _pin_bore()
        for yf in (1, -1):                                      # bearing counterbores
            body -= Pos(x, yf * (t / 2 - 0.6), 0) * _cyl_y(PIN_R + 1.4, 1.4)
    return body


def _rocker() -> Part:
    """Rocker welded to the tail base, offset into the linkage lane. It sits directly on
    the tail's outer fork cheek, so it needs no web back to the centre plane."""
    _, link_y, _ = P.tail_lane_y()
    r = P.TAIL_FOURBAR["rocker"]
    arm = _flat_arm(r, r0=ROCKER_HUB_R, r1=PIN_R + 3.0, t=P.CLEVIS_TONGUE_T)
    arm -= Pos(r, 0, 0) * _pin_bore()
    return Pos(0, link_y, 0) * (Rot(0, -_rocker_angle_deg(), 0) * arm)


def tail() -> Part:
    """The tail itself: a clevis FORK at the base (straddling a tongue on the torso), the
    rocker welded on, and the tapered curl.

    The fork mouth opens FORWARD (+x), because that is the direction the torso's tongue
    arrives from — unlike the leg's joints, where the mating link is always distal. The
    base therefore sits wholly behind the pivot, so nothing of the tail reaches into the
    body as it swings."""
    lo, hi = P.clevis_slot()
    slot_w = hi - lo
    body_w = slot_w + 2 * P.CLEVIS_CHEEK_T
    base = Pos(-7.0, 0, 0) * Box(14.0, body_w, 12.0)
    n = 7
    with BuildPart() as p:
        for i in range(n + 1):
            t = i / n
            x = -P.TAIL_L * t
            z = P.TAIL_L * 0.20 * t * t          # gentle upward curl
            r = P.TAIL_BASE_R * (1 - t) + 2.0 * t  # taper to a fine tip
            with BuildSketch(Plane.YZ.offset(x)):
                with Locations((0.0, z)):
                    Circle(r)
        loft()
    body = base + p.part + _rocker()
    body += _cyl_y(PIVOT_FORK_R, body_w)                    # fork cheeks at the pivot
    # slot for the torso's tongue, open forward so the tongue can enter
    rr = PIVOT_FORK_R + P.CLEVIS_GAP
    void = _cyl_y(rr, slot_w)
    void += Pos(rr, 0, 0) * Box(2 * rr, slot_w, 2 * rr)
    body -= void
    body -= _pin_bore(length=body_w + 10)
    for face in (+1, -1):
        body -= F.pin_head_seat("y", face=face * body_w / 2)
    return body


if __name__ == "__main__":
    t2, pts = tail_linkage()
    print("tail four-bar at neutral (aft frame):",
          {k: (round(v[0], 1), round(v[1], 1)) for k, v in pts.items()})
    print("rocker weld angle: %.1f deg" % _rocker_angle_deg())
    print("tail volume mm^3:", round(tail().volume, 1))
    print("crank %.1f  pushrod %.1f" % (tail_crank().volume, tail_pushrod().volume))


def linkage_transforms():
    """(crank, pushrod) placements in the AFT-TORSO frame at the neutral pose.

    Both parts are drawn on y = 0 and moved into their lanes here, the same split the leg
    uses: geometry in the part, placement in the assembly.
    """
    _, link_y, rod_y = P.tail_lane_y()
    _, pts = tail_linkage()
    sx, sz = P.TAIL_SERVO
    cx, cz = pts["C"]
    rx, rz = pts["R"]
    phi = math.degrees(math.atan2(cz - sz, cx - sx))
    psi = math.degrees(math.atan2(rz - cz, rx - cx))
    t_crank = Pos(sx, link_y, sz) * Rot(0, -phi, 0)
    t_rod = Pos(cx, rod_y, cz) * Rot(0, -psi, 0)
    return t_crank, t_rod
