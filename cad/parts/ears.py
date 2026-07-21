"""
Ears — two expressive appendages (the loudest feline signal). Each is a thin
tapered blade on a small servo-driven base. Local frame: base pivot at origin,
ear rising +z, facing +x.

v1 uses a simple tapered blade (robust primitives); the exact triangular cat-ear
silhouette is a cosmetic refinement for a later pass.
"""

from __future__ import annotations

from build123d import Box, Cone, Part, Pos, Rot

from cad import params as P
from cad.parts import fasteners as F


def ear() -> Part:
    # tapered blade: a flattened cone gives a rounded-triangular silhouette
    blade = Cone(bottom_radius=P.EAR_BASE / 2, top_radius=1.5, height=P.EAR_H)
    blade = Pos(0, 0, P.EAR_H / 2) * blade
    # flatten front-to-back into a thin ear by intersecting with a thin slab
    slab = Pos(0, 0, P.EAR_H / 2) * Box(P.EAR_BASE, 3.0, P.EAR_H + 4)
    blade &= slab
    base = Pos(0, 0, -3) * Box(P.EAR_BASE * 0.5, 8, 6)
    ear = base + blade
    # horn interface: the (linked) ear servo horn drives the base about Y — 2x M2
    # heat-set inserts through the base + the centre horn-shaft clearance hole.
    for dx in (+4, -4):
        ear -= Pos(dx, 0, -3) * F.heatset_hole("M2", "y", 9)
    ear -= Pos(0, 0, -3) * F.wire_hole("sensor", "y", 9)   # centre horn-shaft relief
    return ear


if __name__ == "__main__":
    print("ear volume mm^3:", round(ear().volume, 1))
