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
    """(z, cx, cy, hx, hy, r) sections wrapping one limb segment, shaped as muscle.

    Measured from the segment solid rather than declared, so a change to the leg cannot
    leave the fairing behind — which is the failure this whole module exists to fix.

    But measured is not the same as traced. Following each slab's own half-extents gave a
    cover that was a faithful box around a box: the thigh's dominant feature is the
    rectangular knee servo halfway down it, so the fairing came out as a crate slung under
    the hip, slimmer above and below it. Correct, and not a cat.

    So the extents are run up into a MONOTONE envelope instead — every station widened to
    the largest section at or below it — which turns the mid-thigh bulge into a haunch,
    broadest at the hip and tapering to the knee, the way the muscle it stands in for
    actually runs. Nothing is given up for it: the envelope only ever grows, so it still
    contains every section it has to clear. The ends then dome over instead of stopping
    flat.
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

    keep_out = None if (COVER_KNEE_BOSS or seg != "upper") else _boss_box(leg)
    slabs = []
    for i in range(WRAP_SLABS):
        lo, hi = bb.min.Z + i * h, bb.min.Z + (i + 1) * h
        slab = part & (Pos(0, 0, (lo + hi) / 2) * Box(400, 400, hi - lo))
        if keep_out is not None:
            slab = slab - keep_out          # size to the BONE, not to the housing
        if slab.volume < 1.0:
            continue
        sb = slab.bounding_box()
        slabs.append([lo, hi, sb.min.X - gap, sb.max.X + gap,
                      sb.min.Y - gap, sb.max.Y + gap])
    if not slabs:
        return []

    # The envelope is carried as BOUNDS, not as a centre plus a half-width. Growing a
    # half-width while averaging the centre -- which is what this did first -- slides the
    # section off the limb: at mid-thigh the smoothed centre sat 2.9 mm inboard of the
    # servo it was covering, ate the 1.2 mm running clearance, and the cover shared
    # 1200 mm3 with the limb. Bounds cannot drift.
    #
    # They are NOT run up into a monotone envelope. That was tried, to turn the thigh's
    # mid-length servo bulge into a haunch, and it carried the servo's full 59 mm width
    # straight up to the hip: the cover came out a slab as tall as the torso with a flat
    # outboard face, hanging off the shoulder like a suitcase. A cat's FRONT shoulder is
    # slim; what is thick is the muscle belly halfway down, tapering to both joints. The
    # raw bounds already describe that spindle, so they are left alone.

    def station(z, sl):
        return (z, (sl[2] + sl[3]) / 2, (sl[4] + sl[5]) / 2,
                (sl[3] - sl[2]) / 2, (sl[5] - sl[4]) / 2)

    # Stations go on the slab BOUNDARIES, not the slab centres. A slab's extent is the
    # maximum over its whole height, but a station at its centre only reaches that at the
    # centre: between two centres the ruled loft interpolates and dips inside the limb.
    out = [station(slabs[0][0], slabs[0])]
    for i, sl in enumerate(slabs):
        nxt = slabs[min(i + 1, len(slabs) - 1)]
        wide = [0, 0, min(sl[2], nxt[2]), max(sl[3], nxt[3]),
                min(sl[4], nxt[4]), max(sl[5], nxt[5])]
        out.append(station(sl[1], wide))

    # Square off at the limb's real ends first, then dome BEYOND them. Doming from the
    # end slab's centre put the tapered cap stations back inside the limb.
    e0, e1 = bb.min.Z - P.FAIRING_CLEAR, bb.max.Z + P.FAIRING_CLEAR
    _z, cx0, cy0, hx0, hy0 = out[0]
    _z, cx1, cy1, hx1, hy1 = out[-1]
    out.insert(0, (e0, cx0, cy0, hx0, hy0))
    out.append((e1, cx1, cy1, hx1, hy1))
    for f in (0.86, 0.55):                      # CAP_STATIONS of them; d = R*sqrt(1-f^2)
        d = CAP_RISE * math.sqrt(max(1e-6, 1.0 - f * f))
        out.insert(0, (e0 - d, cx0, cy0, hx0 * f, hy0 * f))
        out.append((e1 + d, cx1, cy1, hx1 * f, hy1 * f))

    # One corner radius per station, taken from the INNER wall so the outer and inner
    # lofts round identically — letting them differ is what made the two surfaces
    # un-subtractable (see _loft_stations).
    # A rounded corner cuts the corner off whatever it covers, so the section has to be
    # padded until its arc clears. For an inner corner of radius p inside an outer of
    # radius r, offset by d, the condition is sqrt2(r - p - d) <= r - p, which gives
    # d >= (r - p)(1 - 1/sqrt2). The running clearance pays part of it.
    #
    # ``p`` matters, and it is not the same all the way down. Treating the limb as a SHARP
    # box everywhere charges the full r(1 - 1/sqrt2) and that was 3.8 mm per side on the
    # thigh -- 7.6 mm of fore-aft width, the second biggest term in the cover after the
    # servo case itself. Over the knee-servo housing the limb is not sharp: _servo_pack
    # rounds every boss by BOSS_CORNER_R, so the allowance there is honestly smaller.
    #
    # Everywhere else it IS sharp. Claiming the discount along the whole thigh put the
    # cover back inside the hip-horn mount and the knee fork, ~27 mm3 at each of four
    # corners, so the discount is applied only over the housing's own z band.
    from cad.parts.leg import BOSS_CORNER_R, knee_boss_envelope

    if seg == "upper":
        (_cx, _cy, bz), (_dx, _dy, bdz) = knee_boss_envelope(
            P.leg_geom(leg)["upper"], P.leg_plane_sign(leg))
        # widened by one slab: the stations sitting on the slab boundaries just outside
        # the housing still carry ITS bounds -- they take the maximum over the slab below
        # and the slab above -- so charging them the sharp-corner allowance put the full
        # 7.6 mm back on the widest part of the cover, which is the part being slimmed.
        boss_lo, boss_hi = bz - bdz / 2 - h, bz + bdz / 2 + h
    else:
        boss_lo = boss_hi = None

    sized = []
    for z, cx, cy, hx, hy in out:
        lim = max(0.6, min(hx, hy) - P.FAIRING_T) * 0.9
        r = min(CORNER_R, lim)
        rounded = boss_lo is not None and boss_lo <= z <= boss_hi
        p_in = min(BOSS_CORNER_R, min(hx, hy)) if rounded else 0.0
        pad = max(0.0, (r - p_in) * (1.0 - 1.0 / math.sqrt(2.0)) - P.FAIRING_CLEAR)
        sized.append((z, cx, cy, hx + pad, hy + pad, r))
    _STATION_CACHE[key] = sized
    return sized


CORNER_R = 17.0         # how round a section may get, capped per station
CAP_RISE = 9.0          # how far a domed end reaches past the limb
CAP_STATIONS = 2        # sections per dome, beyond the limb's own ends

# Whether the thigh cover wraps the knee-servo housing as well as the bone. It does not.
#
# The housing is what made the leg thick: the cover came to 62 x 70 mm, and 45 of the 62
# was the servo case. But 86% of that housing sits INBOARD of the skin line -- body |y|
# 33.3 to 75.6, against a flank at 61.8 -- so nearly all of what the cover was being sized
# for is already hidden inside the body. Sizing to the bone instead halves the leg,
# 62 x 70 -> 31 x 33 mm, and leaves the housing's outboard 13.8 mm bare.
#
# Bare is the right answer here rather than a compromise. The skin is translucent by
# intent and the frame reads through it everywhere else on the robot, so an exposed
# housing is consistent with the rest; and the proportion it buys -- a plump body on slim
# limbs -- is the cat proportion the whole shell exists to get.
COVER_KNEE_BOSS = False


def _boss_box(leg: str) -> Part:
    """The knee-servo housing plus running clearance, in the thigh's frame."""
    from build123d import Box
    from cad.parts.leg import knee_boss_envelope

    (cx, cy, cz), (dx, dy, dz) = knee_boss_envelope(
        P.leg_geom(leg)["upper"], P.leg_plane_sign(leg))
    c = P.FAIRING_CLEAR
    return Pos(cx, cy, cz) * Box(dx + 2 * c, dy + 2 * c, dz + 2 * c)


def _loft_stations(stations, grow: float = 0.0) -> Part:
    """Loft rounded-rectangle sections stacked along z.

    Each station carries its OWN corner radius and both the outer and the inner loft use
    it unchanged. Scaling the radius to each section instead means the two lofts round
    differently, and the surfaces that produces cannot be subtracted from one another:
    the FL shank came back at 21 972 mm3 -- its outer volume, the cut silently not
    applied -- while the RL shank came back at 0.

    RULED, not smooth. A limb's sections are not monotonic and a smooth loft through them
    wobbles enough to self-intersect. ``Shape.is_valid`` does not test for that, so it
    answers True and the damage only shows downstream: one fairing came back with a
    NEGATIVE volume of -6853 mm3. Straight spans between sections cannot wobble.
    """
    with BuildPart() as p:
        for z, cx, cy, ax, ay, r in stations:
            hx, hy = max(ax + grow, r + 0.3), max(ay + grow, r + 0.3)
            with BuildSketch(Plane.XY.offset(z)):
                with Locations((cx, cy)):
                    RectangleRounded(2 * hx, 2 * hy, r)
        loft(ruled=True)
    return p.part


def _torso_keepouts(leg: str) -> list:
    """The torso, posed into the THIGH's frame once per hip sample -- as a LIST.

    The skin is re-lofted solid and grown by ``SKIN_CLEAR`` rather than offset: an offset
    on a loft is fragile and the station table is right there. Sampling the hip window
    gives the region the fairing must stay out of at every pose, which is the same
    real-swept-volume rule the ribcage scallops and the leg ports use.

    Not fused, for the same reason ``_limb_boxes`` is not: a union of seven big
    overlapping lofts is a corrupt solid, and it lies. Fused, it reported the RL thigh
    fairing entirely OUTSIDE it -- intersection 0.0 mm3 -- while the posed fairing was in
    fact sharing 161 mm3 with the aft skin. Subtracted one pose at a time, each cut is a
    shell against a single clean loft.
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
    out = []
    for i in range(HIP_SAMPLES):
        ang = hip0 + lo + (hi - lo) * i / (HIP_SAMPLES - 1)
        T = Pos(mx, my + s * P.leg_geom(leg)["hip_off"], 0) * Rot(0, -math.degrees(ang), 0)
        out.append(T.inverse() * body)
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
    # The cavity is the outer wall stepped inward, which is only safe now that the
    # sections are monotone and share a corner radius per station. Before those two, the
    # inner surface crossed back OUTSIDE the outer on the thigh's steep taper and split
    # the fairing in two, so the limb's own per-slab boxes were cut out instead. Those
    # follow the LIMB, and once the outer wall was reshaped into a haunch that no longer
    # follows the limb, the webs of wall left standing between boxes of different widths
    # broke off -- the FL thigh came out in five pieces. A parallel wall has no webs.
    # The cavity stops at the limb's ends, so the domed caps beyond them come out SOLID.
    # Running the inner wall the full length instead leaves the fairing an open-ended
    # tube, and the proximal mouth points straight outboard: from the front the shoulders
    # read as two bolt-on drums with holes in them.
    inner = _loft_stations(st[CAP_STATIONS:-CAP_STATIONS], grow=-P.FAIRING_T)
    part = outer - inner
    if not COVER_KNEE_BOSS and seg == "upper":
        part = part - _boss_box(leg)        # the housing passes through the cover
    if trim_torso:
        for ko in _torso_keepouts(leg):
            part = part - ko
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
