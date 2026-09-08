"""
Does the chosen actuator physically fit at every joint it is supposed to drive?
==============================================================================

    python -m analysis.actuator_fit            # per-joint fit table
    python -m analysis.actuator_fit --json

``analysis/validate.py`` asks whether the servo is strong enough. This asks the other
question, which is just as able to stop a build: is there *room* for it. A joint can be
in the kinematics, carry the servo's mass in the budget and still have nowhere to put the
actuator -- which is exactly the state the five expression joints (head pan/pitch/tilt,
ears, tail) were in: modelled, massed, and unhoused.

Method. For each joint we place the servo's housing at the joint origin, in the parent
link's frame, trying every orientation the shaft axis allows (the shaft direction is fixed
by the joint, but the case can be rolled about it and fitted either way round, since the
output shaft sits 12 mm from one end of a 45 mm case). We then measure how much of that
housing falls OUTSIDE the cavity available to it -- the inside of the host structure, less
the space other parts already claim. Zero spill means the servo can be housed there; the
best orientation is reported, so the number is "no orientation works", not "the first one
did not".

This is a geometric feasibility check, not a mount design: it says whether an actuator of
this size can live at this joint, not that a good bracket has been drawn for it.
"""

from __future__ import annotations

import argparse
import json

from build123d import Box, Part, Pos, Rot, Sphere

from cad import params as P
from cad.servo import DEFAULT as SERVO

WALL = 3.0          # boss wall around the case, as cad/parts/leg.py uses
SPILL_OK = 1.0      # mm^3 of boolean noise to ignore
BOSS_FILLET = 5.0   # matches cad.parts.leg._rounded_box, which is what is built


def housing(long_axis: str = "x", shaft_axis: str = "y", servo=None) -> Part:
    """The servo's housing envelope, centred on its OUTPUT SHAFT.

    The shaft is not centred on the case (``shaft_offset``), so the body hangs to one
    side; that offset is what makes orientation matter. The mounting flanges are added as
    the thin slab they actually are rather than by widening the whole case, so the check
    is tight rather than merely conservative.
    """
    sv = servo or SERVO
    l, w, h = sv.pocket
    fl, ft = sv.flange_cut
    in_plane = [a for a in "xyz" if a != shaft_axis]
    long_in_plane = long_axis if long_axis in in_plane else in_plane[0]
    short = [a for a in in_plane if a != long_in_plane][0]

    def _b(along, across, depth, round_r: float = 0.0):
        d = {long_in_plane: along, short: across, shaft_axis: depth}
        box = Box(d["x"], d["y"], d["z"])
        if round_r > 0.0:
            # the real bosses are built with cad.parts.leg._rounded_box, so a sharp box
            # here would condemn a servo for corners its housing does not have
            from build123d import BuildPart, fillet
            r = min(round_r, 0.49 * min(d["x"], d["y"], d["z"]))
            with BuildPart() as bp:
                Box(d["x"], d["y"], d["z"])
                fillet(bp.edges(), radius=r)
            box = bp.part
        return box

    def _p(along, depth):
        o = {long_in_plane: along, short: 0.0, shaft_axis: depth}
        return Pos(o["x"], o["y"], o["z"])

    # case + its boss walls, output face on the joint plane
    body = _p(sv.shaft_offset, -(h + 2 * WALL) / 2.0 + WALL) * _b(
        l + 2 * WALL, w + 2 * WALL, h + 2 * WALL, round_r=BOSS_FILLET)
    # mounting-tab slab at the output end
    body += _p(sv.shaft_offset, -ft / 2.0) * _b(fl, w + 2 * WALL, ft)
    return body


def _orientations(shaft_axis: str):
    """Every way the case can sit on a fixed shaft axis: two in-plane long directions x
    two ways round (the shaft is nearer one end of the case)."""
    in_plane = [a for a in "xyz" if a != shaft_axis]
    for long_axis in in_plane:
        for flip_long in (+1, -1):
            for flip_shaft in (+1, -1):
                yield long_axis, flip_long, flip_shaft


def _rot180(axis: str) -> Rot:
    return Rot(**{axis.upper(): 180})


def _posed(shaft_axis: str, long_axis: str, flip_long: int, flip_shaft: int,
           servo=None) -> Part:
    """One concrete way of fitting the servo at a joint.

    Reversing a direction means turning 180 degrees about a PERPENDICULAR axis -- turning
    about an axis leaves that axis pointing the same way. So the case is swapped end for
    end about the shaft axis, and turned to face the other side of the joint plane about
    an in-plane axis.
    """
    h = housing(long_axis, shaft_axis, servo)
    in_plane = [a for a in "xyz" if a != shaft_axis]
    long_in_plane = long_axis if long_axis in in_plane else in_plane[0]
    short = [a for a in in_plane if a != long_in_plane][0]
    if flip_long < 0:                       # swap the case end for end
        h = _rot180(shaft_axis) * h
    if flip_shaft < 0:                      # mount from the other side of the joint plane
        h = _rot180(short) * h
    return h


# --------------------------------------------------------------- available cavities
def head_cavity() -> Part:
    """Inside of the head shell -- what the tilt and ear servos have to live in."""
    return Sphere(P.HEAD_R - P.SHELL_T)


def _torso_envelope(stations) -> Part:
    """Solid lofted through the ribcage's own station table.

    A box through the torso would be optimistic at the ends, which is exactly where the
    tail and neck joints sit -- the body tapers hard there. Using the same
    ``(x, half_width, half_height)`` table the ribs are drawn from keeps the cavity honest.
    """
    from build123d import BuildPart, BuildSketch, Ellipse, Plane, loft
    ordered = sorted(stations, key=lambda s: s[0])
    solid = None
    for (x0, a0, b0), (x1, a1, b1) in zip(ordered, ordered[1:]):
        with BuildPart() as p:
            for x, a, b in ((x0, a0, b0), (x1, a1, b1)):
                with BuildSketch(Plane.YZ.offset(x)):
                    Ellipse(a, b)
            loft()
        solid = p.part if solid is None else solid + p.part
    return solid


def chest_cavity() -> Part:
    """Room in the front torso that the head ball does NOT already claim.

    The head is a Dia92 ball whose centre sits inside the chest, and ``body._head_socket``
    carves the ribcage out for it, so the neck gimbal cannot simply be put where the neck
    is: that space is the head.
    """
    from cad.parts.body import FORE_STATIONS, _head_socket
    return _torso_envelope(FORE_STATIONS) - _head_socket()


def aft_cavity() -> Part:
    """Room in the rear torso, for the tail servo."""
    from cad.parts.body import AFT_STATIONS
    return _torso_envelope(AFT_STATIONS)


# --------------------------------------------------------------- joints under test
def joints() -> list[dict]:
    """The joints whose actuator has no housing in the CAD, with the cavity each one
    would have to be housed in and the joint's origin in that cavity's frame."""
    # Head-frame origins, read off cad/assembly.py: the head part hangs on head_tilt at
    # ITS OWN origin, and pitch/pan are stacked 2 mm apart just behind it -- so all three
    # gimbal axes pass within 4 mm of the head's centre. They therefore have to be housed
    # in the HEAD, not in the chest: the chest at that station is not free space, it is
    # where the ball already is (body._head_socket carves it out).
    out = [
        # The tail is driven REMOTELY (params.TAIL_DRIVE): this measured that its pivot
        # had no room, so the actuator moved forward to P.TAIL_SERVO and reaches the joint
        # through a four-bar. What is checked here is that station, not the pivot.
        dict(name="tail", shaft="y", cavity="aft",
             at=(P.TAIL_SERVO[0], 0.0, P.TAIL_SERVO[1]),
             note="rear torso, remote crank drive to the tail pivot"),
        dict(name="head_pan", shaft="z", cavity="head", at=(-4.0, 0.0, 0.0),
             note="head gimbal, yaw -- axis through the head centre"),
        dict(name="head_pitch", shaft="y", cavity="head", at=(-2.0, 0.0, 0.0),
             note="head gimbal, nod -- axis through the head centre"),
    ]
    if P.HEAD_ROLL_ACTUATED:
        out.append(dict(name="head_tilt", shaft="x", cavity="head", at=(0.0, 0.0, 0.0),
                        note="head gimbal, roll"))
    if P.EARS_ACTUATED:
        out.append(dict(name="ear_L", shaft="y", cavity="head",
                        at=(P.HEAD_R * 0.3, P.EYE_SPACING / 2, P.HEAD_R * 0.7),
                        note="inside the head; both ears linked to one motor"))
    return out


_CAVITIES = {"head": head_cavity, "chest": chest_cavity, "aft": aft_cavity}


def fit(joint: dict, servo=None, cavity: Part | None = None) -> dict:
    """Best-case spill for one joint: the least housing volume left outside the cavity,
    over every orientation the shaft axis allows."""
    cav = _CAVITIES[joint["cavity"]]() if cavity is None else cavity
    at = Pos(*joint["at"])
    best = None
    for long_axis, fl, fs in _orientations(joint["shaft"]):
        h = at * _posed(joint["shaft"], long_axis, fl, fs, servo)
        spill = (h - cav).volume
        cand = dict(spill=spill, long_axis=long_axis, flip_long=fl, flip_shaft=fs,
                    housing=h.volume)
        if best is None or spill < best["spill"]:
            best = cand
    best["fits"] = best["spill"] <= SPILL_OK
    best["spill_pct"] = 100.0 * best["spill"] / best["housing"]
    return {**joint, **best}


def report(servo=None) -> list[dict]:
    return [fit(j, servo) for j in joints()]


def largest_case(joint: dict, step_pct: float = 2.0):
    """The biggest servo CASE that fits at this joint, found by scaling the default
    actuator's proportions down until nothing spills.

    This is the sourcing requirement: an actuator larger than this cannot be housed here,
    whatever else is true of it."""
    import dataclasses

    from cad.servo import Servo
    base = SERVO
    cav = _CAVITIES[joint["cavity"]]()
    k = 1.0
    while k > 0.15:
        trial = dataclasses.replace(
            base, body_l=base.body_l * k, body_w=base.body_w * k, body_h=base.body_h * k,
            flange_l=base.flange_l * k, flange_thk=base.flange_thk * k,
            shaft_from_end=base.shaft_from_end * k)
        if fit(joint, trial, cav)["fits"]:
            return (trial.body_l, trial.body_w, trial.body_h)
        k -= step_pct / 100.0
    return None


def main() -> None:
    ap = argparse.ArgumentParser(description="Does the actuator fit at each joint?")
    ap.add_argument("--json", action="store_true")
    ap.add_argument("--servo", help="test a cad.servo preset instead of the default")
    ap.add_argument("--largest", action="store_true",
                    help="also report the biggest case that WOULD fit at each joint")
    args = ap.parse_args()

    sv = None
    if args.servo:
        from cad.servo import PRESETS
        sv = PRESETS[args.servo]
    rs = report(sv)
    if args.json:
        print(json.dumps(rs, indent=2, default=float))
        return

    act = sv or SERVO
    l, w, h = act.pocket
    print("=" * 78)
    print("ACTUATOR FIT -- %s, case %.0fx%.0fx%.0f, housing %.0fx%.0fx%.0f with %.0f mm walls"
          % (act.name, l, w, h, l + 2 * WALL, w + 2 * WALL, h + 2 * WALL, WALL))
    print("=" * 78)
    print("%-11s %-6s %-7s %8s %8s   %s" % ("joint", "shaft", "cavity", "spill", "of boss", "verdict"))
    for r in rs:
        verdict = "fits" if r["fits"] else "NO ROOM"
        print("%-11s %-6s %-7s %7.0f%s %7.1f%%   %s"
              % (r["name"], r["shaft"], r["cavity"], r["spill"], "mm3", r["spill_pct"], verdict))
    bad = [r for r in rs if not r["fits"]]
    print("-" * 78)
    if not bad:
        print("RESULT: every joint can house the actuator.")
    else:
        print("RESULT: %d of %d joints have NO orientation that fits." % (len(bad), len(rs)))
        for r in bad:
            print("  %-11s %s" % (r["name"], r["note"]))
        print("")
        print("These joints need either a smaller actuator or a remote drive.")
        if args.largest:
            print("")
            print("Largest case that fits (sourcing requirement, LxWxH mm):")
            for r in bad:
                c = largest_case(r)
                txt = ("%.0f x %.0f x %.0f" % c) if c else "nothing of this shape fits"
                print("  %-11s %s" % (r["name"], txt))


def _preset_size(name: str) -> str:
    from cad.servo import PRESETS
    s = PRESETS[name]
    return "%.0fx%.0fx%.0f mm, %.0f g" % (s.body_l, s.body_w, s.body_h, s.mass_kg * 1000)


if __name__ == "__main__":
    main()


# --------------------------------------------------------------- remote-drive placement
# A remote drive frees the actuator from its joint: it only has to fit SOMEWHERE in the
# host cavity and reach the joint through a linkage. That is a different question, and it
# is answered analytically here rather than with booleans -- the torso cavity is a loft
# through elliptical stations, so a box fits iff all eight corners lie inside the ellipse
# at both of its end stations. Milliseconds instead of minutes, and exact.
_HEAD_CENTRE = P.head_centre()      # cad/params.py is the one definition


def _station_ab(stations, x: float):
    ordered = sorted(stations, key=lambda s: s[0])
    if x < ordered[0][0] or x > ordered[-1][0]:
        return None
    for (x0, a0, b0), (x1, a1, b1) in zip(ordered, ordered[1:]):
        if x0 <= x <= x1:
            t = 0.0 if x1 == x0 else (x - x0) / (x1 - x0)
            return (a0 + (a1 - a0) * t, b0 + (b1 - b0) * t)
    return None


def _corners(centre, dims):
    import itertools
    h = [d / 2.0 for d in dims]
    return [tuple(centre[i] + s[i] * h[i] for i in range(3))
            for s in itertools.product((-1, 1), repeat=3)]


def fits_in_torso(stations, centre, dims, avoid_head: bool = False) -> bool:
    for p in _corners(centre, dims):
        ab = _station_ab(stations, p[0])
        if ab is None:
            return False
        a, b = ab
        if (p[1] / a) ** 2 + (p[2] / b) ** 2 > 1.0:
            return False
        if avoid_head:
            d2 = sum((p[i] - _HEAD_CENTRE[i]) ** 2 for i in range(3))
            if d2 < (P.HEAD_R + 2.0) ** 2:
                return False
    return True


def fits_in_head(centre, dims) -> bool:
    r = P.HEAD_R - P.SHELL_T
    return all(sum(v * v for v in p) <= r * r for p in _corners(centre, dims))


def housing_dims(servo=None) -> tuple[float, float, float]:
    sv = servo or SERVO
    return tuple(d + 2 * WALL for d in sv.pocket)


def _orient_dims(dims):
    l, w, h = dims
    return {"long-x": (l, w, h), "long-y": (w, l, h), "long-z": (w, h, l),
            "tall-x": (l, h, w), "tall-y": (h, l, w), "tall-z": (h, w, l)}


GROUP_GAP = 2.0     # between adjacent housings in a stacked group


def _group_dims(dims, n: int, axis: int, gap: float = GROUP_GAP):
    g = list(dims)
    g[axis] = n * dims[axis] + (n - 1) * gap
    return tuple(g)


def capacity(cavity: str, servo=None, step: float = 4.0, max_n: int = 6) -> dict:
    """How many actuators can LIVE in a cavity, and where.

    The count is CONSTRUCTIVE: it looks for ``n`` housings laid side by side as one
    centred group, in every orientation and stacking direction, and accepts a group only
    when all of its corners are inside the real cavity. So a reported count is an
    arrangement that exists, not an estimate — and it is a lower bound, since a cleverer
    packing (staggered rather than in a row) could do better.

    A group is used rather than a free packing search because a first-come greedy search
    is badly order-dependent: it will place one housing against the wall of the cavity and
    then find no room for a second, when two centred side by side would have fitted.
    """
    from cad.parts.body import AFT_STATIONS, FORE_STATIONS
    dims = housing_dims(servo)
    circum = 0.5 * sum(d * d for d in dims) ** 0.5

    if cavity == "head":
        ok = fits_in_head
        lim = P.HEAD_R
        rng = ((-lim, lim), (-lim, lim), (-lim, lim))
    else:
        stations = FORE_STATIONS if cavity == "chest" else AFT_STATIONS
        avoid = cavity == "chest"
        ok = lambda c, d: fits_in_torso(stations, c, d, avoid)
        xs = [x for x, _, _ in stations]
        rng = ((min(xs), max(xs)), (-50, 50), (-40, 40))

    best = {"count": 0, "at": None, "group": None, "axis": None, "orient": None}
    for n in range(1, max_n + 1):
        found = None
        for oname, d in _orient_dims(dims).items():
            for axis in range(3):
                g = _group_dims(d, n, axis)
                for c in _grid(*rng, step=step):
                    if ok(c, g):
                        found = dict(count=n, at=c, group=g, axis=axis, orient=oname)
                        break
                if found:
                    break
            if found:
                break
        if not found:
            break
        best = found
    return {"cavity": cavity, "housing": dims, "circumscribed_r": circum, **best}


def _axis_samples(lo: float, hi: float, step: float):
    """Samples across [lo, hi] that always INCLUDE 0 where the range spans it.

    Starting at ``lo`` and stepping misses the origin unless the range happens to be a
    multiple of the step — and the origin is exactly where a centred group of housings
    wants to sit, so a grid that skips it reports "no room" for arrangements that fit.
    """
    import math
    k0 = math.ceil(lo / step)
    k1 = math.floor(hi / step)
    return [k * step for k in range(int(k0), int(k1) + 1)]


def _grid(xr, yr, zr, step: float = 2.0):
    for x in _axis_samples(xr[0], xr[1], step):
        for y in _axis_samples(yr[0], yr[1], step):
            for z in _axis_samples(zr[0], zr[1], step):
                yield (x, y, z)


# --------------------------------------------------------------- gimbal layout
def gimbal_layout(servo=None, step: float = 2.0, max_off: float = 45.0) -> dict:
    """Can the head's gimbal servos be mounted on their OWN axes, without colliding?

    A gimbal's axes meet at a point, so its motors cannot all sit at that point: each one
    is displaced along its own axis and drives a yoke. That is the arrangement this looks
    for -- one housing on the pan (Z) axis, one on the pitch (Y) axis, each offset from
    the intersection, both inside the head shell and clear of each other.

    The search is over ALL the axes at once. Placing them one at a time is what a person
    would do and it is exactly wrong here: the first servo takes the middle, and then
    there is no room for the second even though offsetting both by a little would have
    fitted them.

    ``capacity('head')`` answers the looser question of how many housings fit ANYWHERE in
    the head; this answers whether they fit where a gimbal actually needs them.
    """
    import itertools

    dims = housing_dims(servo)
    axes = {"head_pan": 2, "head_pitch": 1}          # index into (x, y, z)
    if P.HEAD_ROLL_ACTUATED:
        axes["head_tilt"] = 0

    def _oriented(shaft_idx: int):
        """Housing extents with the servo's DEPTH on the shaft axis (both rolls of the
        case about that axis)."""
        l, w, h = dims
        out = []
        for a, b in ((l, w), (w, l)):
            d = [0.0, 0.0, 0.0]
            d[shaft_idx] = h
            rest = [i for i in range(3) if i != shaft_idx]
            d[rest[0]], d[rest[1]] = a, b
            out.append(tuple(d))
        return out

    def _candidates(idx):
        """Every (centre, dims) this servo could take on its own axis, nearest first."""
        out = []
        off = 0.0
        while off <= max_off:
            for sgn in ((+1,) if off == 0 else (+1, -1)):
                for d in _oriented(idx):
                    c = [0.0, 0.0, 0.0]
                    c[idx] = sgn * off
                    c = tuple(c)
                    if fits_in_head(c, d):
                        out.append((c, d, sgn * off))
            off += step
        return out

    names = list(axes)
    per_axis = [_candidates(axes[n]) for n in names]
    if any(not c for c in per_axis):
        return {"axes": names, "placed": {}, "ok": False}

    def _clear(a, b):
        (ca, da, _), (cb, db, _) = a, b
        return not all(
            ca[i] - da[i] / 2 < cb[i] + db[i] / 2 - 1e-9 and
            cb[i] - db[i] / 2 < ca[i] + da[i] / 2 - 1e-9 for i in range(3))

    for combo in itertools.product(*per_axis):
        if all(_clear(x, y) for x, y in itertools.combinations(combo, 2)):
            return {"axes": names, "ok": True,
                    "placed": {n: {"at": c, "dims": d, "offset": o}
                               for n, (c, d, o) in zip(names, combo)}}
    return {"axes": names, "placed": {}, "ok": False}
