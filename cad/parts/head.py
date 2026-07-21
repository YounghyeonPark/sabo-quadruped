"""
Head — the baby-schema face (PLAN §2.1): oversized round head, big low-set LED
eyes, small muzzle. Local frame: head centre at origin, x forward.

Carries the two LED-eye sockets, a front camera bore, and a pan/tilt mount stub
underneath. Modelled as a hollow sphere (shell) so it's light and printable.
"""

from __future__ import annotations

from build123d import Box, Cylinder, Part, Pos, Rot, Sphere

from cad import params as P
from cad.parts import fasteners as F


def head() -> Part:
    R = P.HEAD_R
    shell = Sphere(R) - Sphere(R - P.SHELL_T)          # hollow
    # muzzle bump forward + slightly down
    shell += Pos(R * 0.7, 0, -6) * Sphere(R * 0.42)

    # big LED-eye sockets bored from the front (baby-schema: large, low-set)
    for s in (+1, -1):
        eye = Pos(R * 0.55, s * P.EYE_SPACING / 2, -4) * (
            Rot(0, 90, 0) * Cylinder(P.EYE_R, R))       # bore along x
        shell -= eye
    # front camera bore between/below the eyes
    shell -= Pos(R * 0.6, 0, -16) * (Rot(0, 90, 0) * Cylinder(P.CAM_R, R))

    # pan/tilt mount stub underneath — bolts to the head_tilt bracket
    shell += Pos(0, 0, -R + 4) * Box(22, 22, 12)
    # head↔neck join: 4x M2 heat-set inserts in the stub (bolted from below through
    # the tilt bracket), on a ~14 mm square.
    for u in (+1, -1):
        for v in (+1, -1):
            shell -= Pos(u * 7, v * 7, -R + 1) * F.heatset_hole("M2", "z", 7)
    # sensor-cable pass-through: routes the front-camera / mic wiring from the head
    # cavity down through the stub to the neck → Jetson bay.
    shell -= Pos(0, 0, -R + 4) * F.wire_hole("sensor", "z", 34)
    return shell


if __name__ == "__main__":
    print("head volume mm^3:", round(head().volume, 1))
