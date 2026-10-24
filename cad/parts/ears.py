"""
Ears — two expressive appendages (the loudest feline signal). Each is a thin
blade on a small foot. Local frame: foot on the head's pad at the origin, ear rising +z,
its flat face toward +x -- forward, the way a cat's ears face what it is listening to.

The blade is a rounded triangle: the tip is an arc of ``EAR_TIP_R``, tangent to the two
sides, so the ear keeps its full height and loses only the point. It used to be a cone cut
by a slab, which ran to a 1.5 mm needle and faced sideways.

The ears are RIGID -- see ``cad/params.py :: EARS_ACTUATED`` for why the motor that used
to drive them went elsewhere.
"""

from __future__ import annotations

from build123d import Axis, Box, Edge, Location, Part, Plane, Pos, extrude, fillet, make_hull

from cad import params as P
from cad.parts import fasteners as F


def ear() -> Part:
    """A rigid ear, bolted to a pad on the head (``params.EARS_ACTUATED`` is False).

    Everything sits ABOVE the origin, because the origin is now the head's mounting
    surface rather than a pivot buried inside the skull — the old part hung its foot below
    the pin, which put the blade through the shell.
    """
    h, H, B = P.EAR_BASE_H, P.EAR_H, P.EAR_BASE
    # profile in the head's YZ plane: a base line and the tip arc, wrapped by their hull
    tip = Edge.make_circle(P.EAR_TIP_R, Plane.XY.move(Location((0, H - P.EAR_TIP_R, 0))))
    prof = make_hull([tip, Edge.make_line((-B / 2, 0, 0), (B / 2, 0, 0))])
    prof = fillet(prof.vertices(), radius=1.5)                   # soften the base corners
    blade = extrude(Plane.YZ * prof, amount=P.EAR_T / 2, both=True)
    # round the rim front and back, so the edge reads soft rather than cut
    blade = fillet(blade.edges().filter_by(Axis.X, reverse=True), radius=1.0)
    blade = Pos(0, 0, h) * blade
    # the foot runs across the head, under the blade
    base = Pos(0, 0, h / 2) * Box(P.EAR_FOOT_W, B * 0.5, h)
    ear = base + blade
    # 2x M2 clearance down through the foot, into heat-set inserts in the head's pad
    for dy in (+4, -4):
        ear -= Pos(0, dy, h / 2) * F.screw_clearance("M2", "z", h + 8)
    return ear


if __name__ == "__main__":
    print("ear volume mm^3:", round(ear().volume, 1))
