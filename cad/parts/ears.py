"""
Ears — two expressive appendages (the loudest feline signal). Each is a thin
tapered blade on a small servo-driven base. Local frame: base pivot at origin,
ear rising +z, facing +x.

v1 uses a simple tapered blade (robust primitives); the exact triangular cat-ear
silhouette is a cosmetic refinement for a later pass.

The ears are RIGID -- see ``cad/params.py :: EARS_ACTUATED`` for why the motor that used
to drive them went elsewhere.
"""

from __future__ import annotations

from build123d import Box, Cone, Part, Pos, Rot

from cad import params as P
from cad.parts import fasteners as F


def ear() -> Part:
    """A rigid ear, bolted to a pad on the head (``params.EARS_ACTUATED`` is False).

    Everything sits ABOVE the origin, because the origin is now the head's mounting
    surface rather than a pivot buried inside the skull — the old part hung its foot below
    the pin, which put the blade through the shell.
    """
    h = P.EAR_BASE_H
    # tapered blade: a flattened cone gives a rounded-triangular silhouette
    blade = Cone(bottom_radius=P.EAR_BASE / 2, top_radius=1.5, height=P.EAR_H)
    blade = Pos(0, 0, h + P.EAR_H / 2) * blade
    # flatten front-to-back into a thin ear by intersecting with a thin slab
    slab = Pos(0, 0, h + P.EAR_H / 2) * Box(P.EAR_BASE, 3.0, P.EAR_H + 4)
    blade &= slab
    base = Pos(0, 0, h / 2) * Box(P.EAR_BASE * 0.5, P.EAR_FOOT_W, h)
    ear = base + blade
    # 2x M2 clearance down through the foot, into heat-set inserts in the head's pad
    for dx in (+4, -4):
        ear -= Pos(dx, 0, h / 2) * F.screw_clearance("M2", "z", h + 8)
    return ear


if __name__ == "__main__":
    print("ear volume mm^3:", round(ear().volume, 1))
