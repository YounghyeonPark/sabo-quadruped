"""
Limb fairings — the moving half of the skin.
===========================================

`shell.py` covers the torso. It cannot cover the legs, and the reason is worth stating
because it decides the whole shape of this module: the knee servo lies LATERALLY on the
thigh and reaches 46 mm inboard of the thigh plane, which is inside the torso envelope. It
is the same protrusion that forces body.py to scallop 12% out of the ribcage. As the hip
sweeps, that lump carves the entire flank open — a port 76 x 58 mm on the front — so there
is no static skin that both covers the flank and lets the leg move.

What CAN cover it is a piece that turns with the thigh. That is exactly what a cat's
scapula does, and why these are separate printed parts rather than more torso skin.

Each fairing is lofted from the segment it wraps — sliced into z-slabs, each slab's own
cross-section grown by clearance and wall — so it hugs the limb instead of boxing it, and
the thigh's mid-length servo bulge reads as the muscle it is standing in for. It is then
trimmed against the torso: every fairing is subtracted by the skin's outer surface plus
clearance, sampled across the hip's real working window, so it cannot foul the body at any
pose the robot can strike.
"""

from __future__ import annotations

import math

from build123d import (BuildPart, BuildSketch, Locations, Part, Plane, Pos,
                       RectangleRounded, Rot, loft)

from cad import params as P

_STATION_CACHE: dict[tuple, list] = {}
_FAIRING_CACHE: dict[tuple, Part] = {}

WRAP_SLABS = 7          # z-slabs a segment is measured in
HIP_SAMPLES = 7         # poses the torso trim is sampled at


def _seg_stations(leg: str, seg: str) -> list:
    """(z, cx, cy, ax, ay) ellipse stations wrapping one limb segment.

    Measured from the segment solid rather than declared, so a change to the leg cannot
    leave the fairing behind — which is the failure this whole module exists to fix.
    """
    key = (leg, seg)
    hit = _STATION_CACHE.get(key)
    if hit is not None:
        return hit

    from build123d import Box
    from cad.parts.leg import leg_parts

    part = leg_parts(leg)[seg]
    bb = part.bounding_box()
    h = (bb.max.Z - bb.min.Z) / WRAP_SLABS
    gap = P.FAIRING_CLEAR + P.FAIRING_T
    out = []
    for i in range(WRAP_SLABS):
        a, b = bb.min.Z + i * h, bb.min.Z + (i + 1) * h
        z = (a + b) / 2
        slab = part & (Pos(0, 0, z) * Box(400, 400, b - a))
        if slab.volume < 1.0:
            continue
        s = slab.bounding_box()
        # ROUNDED RECTANGLES, not ellipses. The sections being wrapped are boxy -- the
        # knee servo most of all -- and an ellipse has to circumscribe the box's corners
        # to contain it, which blew the thigh fairing out to 80 x 90 mm. A rounded rect
        # hugs the same section at its true half-extents.
        out.append((z, s.center().X, s.center().Y,
                    s.size.X / 2 + gap, s.size.Y / 2 + gap))
    # extend the end stations to the segment's real ends so nothing pokes out
    if out:
        z0, cx0, cy0, ax0, ay0 = out[0]
        out.insert(0, (bb.min.Z - P.FAIRING_CLEAR, cx0, cy0, ax0 * 0.55, ay0 * 0.55))
        z1, cx1, cy1, ax1, ay1 = out[-1]
        out.append((bb.max.Z + P.FAIRING_CLEAR, cx1, cy1, ax1 * 0.55, ay1 * 0.55))
    _STATION_CACHE[key] = out
    return out


CORNER_R = 2.5          # section corner radius, CONSTANT along the loft


def _loft_stations(stations, grow: float = 0.0) -> Part:
    """Loft rounded-rectangle sections stacked along z.

    The corner radius is a constant, not a fraction of each section. Scaling it per
    station means the outer and inner lofts round differently, and the surfaces that
    produced could not be subtracted from one another: the FL shank came back at 21 972
    mm3 -- its outer volume, the cut silently not applied -- while the RL shank came back
    at 0. A boolean that returns one operand unchanged, or nothing at all, has failed, so
    the sections are kept parallel and the shell subtracts cleanly.
    """
    with BuildPart() as p:
        for z, cx, cy, ax, ay in stations:
            hx, hy = max(ax + grow, CORNER_R + 0.3), max(ay + grow, CORNER_R + 0.3)
            with BuildSketch(Plane.XY.offset(z)):
                with Locations((cx, cy)):
                    RectangleRounded(2 * hx, 2 * hy, CORNER_R)
        loft()
    return p.part


def _limb_cavity(leg: str, seg: str) -> Part:
    """What the fairing has to be hollow AROUND: the limb itself, plus running clearance.

    Not an inward offset of the wrap. Shrinking the lofted section by the wall thickness
    is the obvious way to hollow a lofted shell and it fails here, because the thigh's
    profile steps from a 60 mm servo bulge to a 23 mm neck over 12 mm of length: on a
    taper that steep an in-plane inset is far larger than a perpendicular one, and the
    inner surface crosses back OUTSIDE the outer. That split the RL thigh fairing into
    two solids before it was ever trimmed.

    A stack of per-slab boxes cannot invert, and it is the more honest cavity anyway -- a
    cover's inside should follow the limb it clips over, not the styling of its outside.
    """
    from build123d import Box
    from cad.parts.leg import leg_parts

    part = leg_parts(leg)[seg]
    bb = part.bounding_box()
    h = (bb.max.Z - bb.min.Z) / WRAP_SLABS
    c = P.FAIRING_CLEAR
    out = None
    for i in range(WRAP_SLABS):
        a, b = bb.min.Z + i * h, bb.min.Z + (i + 1) * h
        slab = part & (Pos(0, 0, (a + b) / 2) * Box(400, 400, b - a))
        if slab.volume < 1.0:
            continue
        sb = slab.bounding_box()
        box = Pos(sb.center().X, sb.center().Y, (a + b) / 2) * Box(
            sb.size.X + 2 * c, sb.size.Y + 2 * c, (b - a) + 2 * c)
        out = box if out is None else out + box
    return out


def _torso_keepout(leg: str) -> Part:
    """The torso, as the fairing sees it, in the THIGH's local frame.

    The skin is re-lofted solid and grown by ``SKIN_CLEAR`` rather than offset — an
    offset on a loft is fragile, and the station table is right there. Sampling the hip
    window and unioning gives the region the fairing must stay out of at every pose,
    which is the same real-swept-volume rule the ribcage scallops and the leg ports use.
    """
    from cad.parts.body import hip_work_range
    from cad.parts.shell import AFT_STATIONS, FORE_STATIONS, _loft_body
    from sim.gait import stance_angles

    st = FORE_STATIONS if leg[0] == "F" else AFT_STATIONS
    body = _loft_body([(x, a + P.SKIN_CLEAR, b + P.SKIN_CLEAR) for (x, a, b) in st])

    mx, my = P.MOUNTS[leg]
    s = P.leg_plane_sign(leg)
    hip0 = stance_angles(leg)[0]
    lo, hi = hip_work_range(leg)
    out = None
    for i in range(HIP_SAMPLES):
        ang = hip0 + lo + (hi - lo) * i / (HIP_SAMPLES - 1)
        T = Pos(mx, my + s * P.leg_geom(leg)["hip_off"], 0) * Rot(0, -math.degrees(ang), 0)
        posed = T.inverse() * body
        out = posed if out is None else out + posed
    return out


def _screw_holes(part: Part, leg: str, seg: str) -> Part:
    """Clearance + head counterbore where the fairing bolts to its limb.

    ``fasteners.screw_clearance(head=True)`` always puts the counterbore at the +axis
    end, and the outboard direction is -y on the robot's right side, so the two pieces are
    placed by hand instead of flipping a cutter whose head would end up buried.
    """
    from cad.parts import fasteners as F
    from cad.parts.leg import fairing_mounts

    g = P.leg_geom(leg)
    length = g["upper"] if seg == "upper" else g["lower"]
    sign = P.leg_plane_sign(leg)
    spec = P.SCREW[P.FAIRING_SCREW]
    span = 2 * (P.FAIRING_CLEAR + P.FAIRING_T) + 4.0
    for z, y in fairing_mounts(length, seg, sign):
        out = 1.0 if y >= 0 else -1.0              # outboard direction for this side
        mid = y + out * span / 2.0
        part -= Pos(0, mid, z) * F.screw_clearance(P.FAIRING_SCREW, "y", span)
        # counterbore, sunk into the OUTER face so the head does not stand proud
        head_y = y + out * (P.FAIRING_CLEAR + P.FAIRING_T + spec["head_h"] / 2)
        part -= Pos(0, head_y, z) * F._cyl("y", spec["head_r"], spec["head_h"])
    return part


def _fairing(leg: str, seg: str, trim_torso: bool) -> Part:
    key = (leg, seg)
    hit = _FAIRING_CACHE.get(key)
    if hit is not None:
        return hit

    st = _seg_stations(leg, seg)
    outer = _loft_stations(st)
    part = outer - _limb_cavity(leg, seg)
    if trim_torso:
        part = part - _torso_keepout(leg)
    part = _screw_holes(part, leg, seg)
    _FAIRING_CACHE[key] = part
    return part


def thigh_fairing(leg: str) -> Part:
    """The scapula / haunch cover: wraps the thigh and the knee servo it carries."""
    return _fairing(leg, "upper", trim_torso=True)


def shank_fairing(leg: str) -> Part:
    """The slim lower-leg cover.

    NOT trimmed against the torso, deliberately. ``_torso_keepout`` is built in the
    THIGH's frame -- the hip is the only joint between it and the body -- and the shank
    hangs off a second joint, so applying that keepout here subtracts the body from the
    wrong place. It showed: the FL shank fairing came out at 21 972 mm3 in two pieces,
    twice the size of its mirror on the rear leg. The shank is slim and hangs clear, so
    the honest answer is not to trim it and to CHECK it instead -- which is what
    tests/test_geometry.py does, over the poses the robot can actually strike.
    """
    return _fairing(leg, "lower", trim_torso=False)


if __name__ == "__main__":
    for lg in ("FL", "RL"):
        for fn in (thigh_fairing, shank_fairing):
            pt = fn(lg)
            print("%-6s %-15s %8.0f mm3  %d solid(s)"
                  % (lg, fn.__name__, pt.volume, len(pt.solids())))
