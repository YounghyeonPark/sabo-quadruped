"""
Head — the baby-schema face (PLAN §2.1): oversized round head, big low-set LED
eyes, small muzzle. Local frame: head centre at origin, x forward.

Carries the two LED-eye sockets, a front camera bore, and a gimbal mount stub
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

    # gimbal mount stub underneath — bolts to the head_pitch yoke
    shell += Pos(0, 0, -R + 4) * Box(22, 22, 12)
    # head↔neck join: 4x M2 heat-set inserts in the stub (bolted from below through
    # the tilt bracket), on a ~14 mm square.
    for u in (+1, -1):
        for v in (+1, -1):
            shell -= Pos(u * 7, v * 7, -R + 1) * F.heatset_hole("M2", "z", 7)
    # ear pads: the ears are bolted on (params.EARS_ACTUATED is False), so the head has to
    # present a flat landing with the inserts their screws pull into.
    ins = P.HEATSET["M2"]
    for side in (+1, -1):
        px, py, pz = P.ear_station(side)
        shell += Pos(px, py, pz - 7.0) * Cylinder(10.0, 14.0)      # boss up to a flat top
        for dx in (+4, -4):
            shell -= Pos(px + dx, py, pz - ins["depth"] / 2) * \
                F.heatset_hole("M2", "z", ins["depth"])

    # PITCH actuator: the nod axis is driven directly, by a servo centred in the head with
    # its horn bolted to the neck column's yoke (params.PITCH_DRIVE). It has to be centred
    # -- a 42 mm case hung off one side of the axis reaches outside the shell -- so its horn
    # lands about 21 mm off the centre plane and the yoke straddles the head to meet it.
    from cad.parts.leg import _cyl_y, _servo_pack
    yoke_face = P.pitch_yoke_face()
    boss, cut = _servo_pack(0.0, 0.0, yoke_face, +1, long_axis="x")
    shell += boss
    # The skull is a hollow shell, so a boss at its centre touches nothing and prints as a
    # loose second piece. Tie it out to the wall.
    for dx, dz in ((1, 0), (-1, 0), (0, 1), (0, -1)):
        shell += Pos(dx * R / 2.0, 0, dz * R / 2.0) * Box(
            R if dx else 8.0, 10.0, R if dz else 8.0)
    # the far side turns on a plain Dia6 stub running in the yoke's other arm
    # The far side turns on a plain Ø6 journal running in the yoke's other arm. It grows a
    # COLLAR where it leaves the boss: the servo's own shaft relief bores a Ø6 hole through
    # that face, so a Ø6 journal meets it exactly tangentially and prints as a loose pin.
    # The collar stops just OUTBOARD of the boss face and just INBOARD of the yoke's pad —
    # it has to bridge the bored face without running into the arm the journal turns in.
    c_lo, c_hi = -(yoke_face + 0.2), -(yoke_face - 7.0)
    shell += Pos(0, (c_lo + c_hi) / 2, 0) * _cyl_y(5.0, c_hi - c_lo)
    j_lo, j_hi = -(P.pitch_yoke_y() + 1.0), -(yoke_face - 0.7)
    shell += Pos(0, (j_lo + j_hi) / 2, 0) * _cyl_y(P.AXLE_R, j_hi - j_lo)
    # mouth for the neck column: it enters from below and swings as the head nods
    shell -= Pos(0, 0, -R * 0.55) * Box(2 * P.HEAD_GIMBAL_STACK + P.NECK_COLUMN_W + 8,
                                        2 * P.pitch_yoke_y() + P.PITCH_YOKE_ARM + 4,
                                        R * 1.2)

    # sensor-cable pass-through: routes the front-camera / mic wiring from the head
    # cavity down through the stub to the neck → Jetson bay.
    shell -= Pos(0, 0, -R + 4) * F.wire_hole("sensor", "z", 34)
    return shell - cut


if __name__ == "__main__":
    print("head volume mm^3:", round(head().volume, 1))
