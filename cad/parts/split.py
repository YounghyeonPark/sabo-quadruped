"""
Print SPLIT geometry — cut the big parts in halves + inject mating features.
============================================================================

The three largest printable parts don't fit a typical FDM bed (or need heroic
supports) as one piece, so ``cad/print_manifest.py`` flags them ``split``. This
module performs that cut at PRINT/EXPORT time and adds the bond features, WITHOUT
touching the whole-part definitions — ``cad.assembly`` / ``full_robot`` / the mass
model still import the intact ``torso_fore()`` / ``torso_aft()`` / ``head()``. The
glued-together halves reproduce the intact part (plus a little added bond material,
minus the dowel clearances).

Cut planes (see ``cad/params.py`` "print SPLIT"):
  * ``torso_fore`` / ``torso_aft`` → SAGITTAL on Y=0 into ``_L`` (+y) / ``_R`` (-y).
    The ribcage is an open lattice; each half lays its flat XZ cut face on the bed so
    every hoop arches UP off the bed with almost no support, and the two halves bond
    along the spine + keel stringers (which run on Y=0) plus the waist bulkhead.
  * ``head`` → EQUATORIAL "brow-line" cut on Z=HEAD_SPLIT_Z into a lower ``_A`` face
    bowl (both eye sockets + the camera bore + muzzle + neck stub, all intact and open
    to the top so you drop in / wire the camera + LED eyes before bonding) and an upper
    ``_B`` cranial cap. Each bowl prints cut-face-down as a self-supporting dome.

Mating features at every cut:
  1. Alignment: 2-3 SEPARATE Ø4 dowel RODS straddling the cut (a printed-in-place boss
     would point into the bed on a cut-face-down half). Each rod presses into one half
     (``fit="press"``) and slips in the other (``fit="clear"``) so the halves still part
     for gluing. Sockets live in added bond material (torso spine/keel pads; head ring
     flange) so they're deep enough to resist shear.
  2. Bond face: the flat cut plane is the glue land (CA / epoxy). A shallow glue-relief
     channel at each bond pad / a ring groove in the head flange gives squeeze-out room.
  3. Each resulting half is a valid, closed solid that prints flat on its cut face.

All operations are robust cylinder/box booleans (plane cut = intersect with a big
half-space box), avoiding the loft/boolean validity pitfalls.
"""

from __future__ import annotations

from build123d import Box, Cylinder, Part, Pos, Sphere

from cad import params as P
from cad.parts import fasteners as F
from cad.parts.body import AFT_STATIONS, FORE_STATIONS, torso_aft, torso_fore
from cad.parts.head import head

_BIG = 600.0   # half-space cutter cube edge (>> any part) — the plane-cut primitive


# ------------------------------------------------------------------ plane cut primitive
def _halfspace(axis: str, positive: bool, at: float = 0.0) -> Part:
    """A big solid box filling one side of the plane ``axis == at``. Intersect a part
    with it to keep the ``positive`` (or negative) half-space."""
    d = _BIG / 2.0
    off = at + d if positive else at - d
    pos = {"x": (off, 0, 0), "y": (0, off, 0), "z": (0, 0, off)}[axis]
    return Pos(*pos) * Box(_BIG, _BIG, _BIG)


def _interp(stations, x: float) -> tuple[float, float]:
    """Linear (a=half-width, b=half-height) of the body contour at station ``x``."""
    xs = [s[0] for s in stations]
    lo, hi = min(xs), max(xs)
    x = max(lo, min(hi, x))
    ordered = sorted(stations, key=lambda s: s[0])
    for (x0, a0, b0), (x1, a1, b1) in zip(ordered, ordered[1:]):
        if x0 <= x <= x1:
            t = 0.0 if x1 == x0 else (x - x0) / (x1 - x0)
            return a0 + (a1 - a0) * t, b0 + (b1 - b0) * t
    a, b = ordered[-1][1], ordered[-1][2]
    return a, b


# ------------------------------------------------------------------ torso: sagittal Y=0
def _torso_split(frame: Part, stations, specs):
    """Add spine/keel bond pads + dowels straddling Y=0, then cut into L (+y) / R (-y).

    ``specs`` = list of ``(x, kind)`` with ``kind in {"spine","keel"}``; each becomes a
    flat bond pad on the top (spine) or bottom (keel) stringer hosting one Ø4 dowel."""
    dowels = []          # (x, z) socket centres on the Y=0 cut plane
    px, py, pz = P.SPLIT_PAD
    for x, kind in specs:
        _, b = _interp(stations, x)
        # pad centred on the stringer but biased 2 mm inward so it barely protrudes past
        # the hoop line (skin clearance); it overlaps the 5 mm stringer -> stays fused.
        z = (b - 2.0) if kind == "spine" else -(b - 2.0)
        frame += Pos(x, 0, z) * Box(px, py, pz)
        dowels.append((x, z))

    left = frame & _halfspace("y", True)
    right = frame & _halfspace("y", False)
    for (x, z) in dowels:
        left -= Pos(x, 0, z) * F.dowel_socket("press", "y")
        right -= Pos(x, 0, z) * F.dowel_socket("clear", "y")
        # glue-relief channel: a shallow vertical slot on the L bond face beside the dowel
        left -= Pos(x + 5.0, P.SPLIT_GLUE_D / 2, z) * Box(P.SPLIT_GLUE_W, P.SPLIT_GLUE_D, pz - 2)
    return left, right


# fore/aft dowel layout: two spine pads (fore/aft-separated) + one keel pad -> a triangle
# that locks translation + all rotations; the long spine/keel glue faces do the rest.
_FORE_SPECS = [(24.0, "spine"), (70.0, "spine"), (46.0, "keel")]
_AFT_SPECS = [(-24.0, "spine"), (-70.0, "spine"), (-46.0, "keel")]


def torso_fore_halves():
    return _torso_split(torso_fore(), FORE_STATIONS, _FORE_SPECS)


def torso_aft_halves():
    return _torso_split(torso_aft(), AFT_STATIONS, _AFT_SPECS)


# ------------------------------------------------------------------ head: equatorial Z=zc
def _head_dowel_xy():
    """(u, v) of the three bond pads / dowels on the ``HEAD_DOWEL_R`` circle."""
    import math
    return [(P.HEAD_DOWEL_R * math.cos(2 * math.pi * i / 3 + math.pi / 6),
             P.HEAD_DOWEL_R * math.sin(2 * math.pi * i / 3 + math.pi / 6)) for i in range(3)]


def head_halves():
    """(lower face bowl A, upper cranial cap B).

    Bond material is THREE local pads at the dowel spots, each a chunk of the SOLID head
    sphere within a small column (so it hugs the shell wall + is flush with the outer
    surface — never bulges, always fused). The thin shell-wall annulus is the continuous
    glue seam; the pads add depth for the dowels + a flat glue land."""
    zc, h = P.HEAD_SPLIT_Z, P.HEAD_PAD_H
    # The pads are cut from the head's OWN outer form, and each half is built from pieces
    # that were cut separately and THEN joined. Both matter. The pads used to be chunks of
    # a HEAD_R sphere, which hugged the head when the head was that sphere; it is the
    # baby-schema face now, and a sphere-chunk's skin lies almost on the face's without
    # lying on it. Fusing those and then cutting the fused body in half returned an EMPTY
    # lower bowl -- the head alone cut cleanly to 49 906 mm3, the padded head to nothing.
    #
    # The pads also have to REACH the wall. The dowels sit at HEAD_DOWEL_R = 39 and a pad
    # HEAD_PAD_XY wide stops at r 46.5; on the old sphere that was close enough, but the
    # face's wall at the split height is at r 47-49, so all three pads floated in the hollow
    # and the third touched nothing at all. They are drawn twice as wide, out past the
    # wall, and then clipped to 0.3 mm inside the head's OUTER form: wide enough to bond,
    # and never sharing a face with the skin they bond to (a pad that shared the outer
    # skin exactly made the third union return the pad alone, 51 765 + 1 462 -> 1 462).
    from cad.parts.shell import _head_cavity

    fill = _head_cavity(0.3)
    hd = head()
    reach = 2 * P.HEAD_PAD_XY
    pads = [fill & (Pos(u, v, zc) * Box(reach, reach, h)) for u, v in _head_dowel_xy()]
    below, above = _halfspace("z", False, zc), _halfspace("z", True, zc)
    # Each half is reduced to its ONE body before anything is added to it: the plane
    # leaves a 455 mm3 sliver of wall loose beside the stereo eye recesses, and fusing a
    # pad onto a half that is still two solids returned a fragment (4 690 mm3 of a bowl
    # that is ~50 000).
    #
    # The pads go on in ONE fuse, not three. Each pad fuses onto the bowl correctly on its
    # own (bowl + the -y pad = 53 806 mm3, as it should), yet chained one after another the
    # third union returned that pad and nothing else -- 4 802 mm3 of a ~62 500 bowl, with
    # every input valid. OCCT's general fuse takes all its arguments together and gets it
    # right; a chain of binary fuses carries something forward that it does not.
    lower = _largest(hd & below, "head lower bowl")    # z < zc : eyes/cam/muzzle/stub
    upper = _largest(hd & above, "head cranial cap")   # z > zc : cranial cap
    lower = _as_part(lower.fuse(*[pad & below for pad in pads]))
    upper = _as_part(upper.fuse(*[pad & above for pad in pads]))

    # dowels press into the lower bowl and slip in the cap (so the cap lifts off for camera
    # work); a glue-relief ring groove in the bowl's rim gives squeeze-out room clear of
    # the dowels. All of a half's cuts go in ONE cut, for the same reason the pads go in one
    # fuse: each socket alone takes its 74 mm3, but subtracted one after another the
    # second or third left no solid at all.
    sockets = [Pos(u, v, zc) for u, v in _head_dowel_xy()]
    gi = P.HEAD_DOWEL_R - P.HEAD_PAD_XY / 2 - 1.0
    groove = Cylinder(gi + P.SPLIT_GLUE_W, P.SPLIT_GLUE_D) - Cylinder(gi, P.SPLIT_GLUE_D)
    lower = _as_part(lower.cut(*[at * F.dowel_socket("press", "z") for at in sockets],
                               Pos(0, 0, zc - P.SPLIT_GLUE_D / 2) * groove))
    upper = _as_part(upper.cut(*[at * F.dowel_socket("clear", "z") for at in sockets]))
    # the split plane passes just under the stereo eye recesses and leaves a sliver of
    # wall between them standing loose (455 mm3); it is an offcut, not part of the bowl
    return _largest(lower, "head lower bowl"), _largest(upper, "head cranial cap")


def _as_part(shape) -> Part:
    """A fuse can return a Compound carrying faces and edges beside the solid, and the
    next subtraction refuses it ("dimensions ... inconsistent"). Keep the solids only."""
    from build123d import Compound

    return Part(Compound(children=list(shape.solids())).wrapped)


def _largest(part: Part, tag: str, limit: float = 0.03) -> Part:
    """Keep the body of a split half; fail if the split really cut it in two."""
    from build123d import Compound

    solids = sorted(part.solids(), key=lambda x: -x.volume)
    if len(solids) <= 1:
        return part
    lost = sum(x.volume for x in solids[1:])
    frac = lost / (solids[0].volume + lost)
    if frac > limit:
        raise ValueError(f"{tag} came apart: {len(solids)} solids, {frac:.1%} outside the largest")
    return Part(Compound(children=[solids[0]]).wrapped)


# ------------------------------------------------------------------ registry (for export)
def split_parts() -> dict[str, Part]:
    """The six printable sub-parts keyed by export name."""
    fl, fr = torso_fore_halves()
    al, ar = torso_aft_halves()
    ha, hb = head_halves()
    return {
        "torso_fore_L": fl, "torso_fore_R": fr,
        "torso_aft_L": al, "torso_aft_R": ar,
        "head_A": ha, "head_B": hb,
    }


# which whole part each sub-part comes from + its cut (for manifest/report)
SPLIT_SOURCE = {
    "torso_fore_L": "torso_fore", "torso_fore_R": "torso_fore",
    "torso_aft_L": "torso_aft", "torso_aft_R": "torso_aft",
    "head_A": "head", "head_B": "head",
}


if __name__ == "__main__":
    for name, part in split_parts().items():
        bb = part.bounding_box()
        print(f"{name:<14} vol {part.volume:9.1f}  valid {bool(part.is_valid)!s:<5}  "
              f"bbox {bb.size.X:5.1f} x {bb.size.Y:5.1f} x {bb.size.Z:5.1f}")
