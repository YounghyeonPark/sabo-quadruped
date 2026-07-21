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
    solid = head()
    for u, v in _head_dowel_xy():
        solid += Sphere(P.HEAD_R) & (Pos(u, v, zc) * Box(P.HEAD_PAD_XY, P.HEAD_PAD_XY, h))
    lower = solid & _halfspace("z", False, zc)         # z < zc : eyes/cam/muzzle/stub
    upper = solid & _halfspace("z", True, zc)          # z > zc : cranial cap

    # dowels: press into the lower bowl, slip in the cap (so the cap lifts off for camera work)
    for u, v in _head_dowel_xy():
        at = Pos(u, v, zc)
        lower -= at * F.dowel_socket("press", "z")
        upper -= at * F.dowel_socket("clear", "z")
    # glue-relief ring groove in the lower bowl's rim (squeeze-out room, clear of the dowels)
    gi = P.HEAD_DOWEL_R - P.HEAD_PAD_XY / 2 - 1.0
    groove = Cylinder(gi + P.SPLIT_GLUE_W, P.SPLIT_GLUE_D) - Cylinder(gi, P.SPLIT_GLUE_D)
    lower -= Pos(0, 0, zc - P.SPLIT_GLUE_D / 2) * groove
    return lower, upper


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
