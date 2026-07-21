"""
Tail — a tapered, gently up-curving appendage on a servo base (up/down = the
loudest emotion channel). Local frame: base pivot at origin, tail extending -x
(rearward) and curving up; the resting curve is modelled here, articulated in sim.
"""

from __future__ import annotations

from build123d import (BuildPart, BuildSketch, Box, Circle, Locations, Part,
                       Plane, loft)

from cad import params as P
from cad.parts import fasteners as F


def tail() -> Part:
    base = Box(14, 14, 10)
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
    tail = base + p.part
    # horn interface: the tail servo horn drives the base about Y — M2 heat-set inserts
    # on a small bolt circle + the centre horn-shaft clearance hole.
    tail -= F.horn_holes(axis="y", length=16, bolt_circle=9.0)
    return tail


if __name__ == "__main__":
    print("tail volume mm^3:", round(tail().volume, 1))
