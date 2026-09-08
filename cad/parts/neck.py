"""
Neck column + the head gimbal's two remote drives.

Why remote (params.HEAD_DRIVE)
------------------------------
``analysis.actuator_fit.gimbal_layout`` measures that a gimbal's motors cannot sit on
their own axes inside this head: the axes meet at a point, so each motor has to step aside
along its axis, and two STS3215 housings do not both fit that way until roughly a Ø148
head. So both axes are driven the way the knee, the hip and the tail already are —
actuator where there is room, four-bar to the joint.

The topology is forced, not chosen. The head is the LAST link in the chain, so a servo
inside it can only reach the one joint immediately above it (pitch). The other axis must
therefore be driven from further up, and the only cavity there is the chest::

    chest servo --four-bar--> PAN   (neck yaw, vertical axis, linkage in a horizontal plane)
    head  servo --on the axis-> PITCH (nod) -- ONE housing does fit on its own axis, centred
                                              in the head; it is the second that cannot.

Frames
------
The column is the ``head_pan`` link. Its origin is the yaw BEARING, down in the chest
under the head ball — the same vertical axis as before, just the end of it the torso can
support. The column runs up and forward from there to the pitch axis at the head's centre.
"""

from __future__ import annotations

import math

from build123d import Box, Cylinder, Part, Pos, Rot

from cad import params as P
from cad.parts import fasteners as F
from cad.parts.leg import _cyl_y, _flat_arm, _pin_bore, LINK_W, PIN_R
from cad.servo import DEFAULT as SERVO

COLUMN_R = P.NECK_COLUMN_W / 2.0
YAW_BEARING = dict(bore_r=3.0, od_r=6.5, width=5.0)      # 686ZZ, as the hips use


def _cyl_z(radius: float, height: float) -> Part:
    """Cylinder on the local Z axis — the yaw axis."""
    return Cylinder(radius=radius, height=height)


# ------------------------------------------------------------------ placements
def pan_linkage():
    """The pan four-bar at its mid-window pose, in the FORE-TORSO horizontal plane.

    ``analysis.fourbar`` works with O4 at the origin and O2 straight 'up' at (0, d); here
    O4 is the yaw axis and O2 is the chest servo's shaft, both vertical, so the linkage
    lives in a horizontal (x, y) plane and the mapping is a rotation in that plane.
    """
    from analysis.fourbar import FourBar
    fb = P.PAN_FOURBAR
    bar = FourBar(d=P.pan_ground(), r2=fb["crank"], r3=fb["coupler"], r4=fb["rocker"])
    lo, hi = fb["crank_window"]
    t2 = math.radians((lo + hi) / 2)
    pts = bar.linkage_points(t2)
    ox = P.HEAD_MOUNT_X                       # O4, the yaw axis
    sx = P.PAN_SERVO[0]                       # O2, the chest servo
    ev = ((sx - ox) / abs(sx - ox), 0.0)      # ground direction, along -x
    eu = (0.0, ev[0])                         # perpendicular, in +y
    out = {}
    for k, v in pts.items():
        if not isinstance(v, tuple):
            continue
        u, w = v
        out[k] = (ox + u * eu[0] + w * ev[0], 0.0 + u * eu[1] + w * ev[1])
    return t2, out                            # (x, y) in the fore-torso frame




# ------------------------------------------------------------------ printed links
def pan_crank() -> Part:
    """Crank on the chest servo's (vertical) horn, sweeping a horizontal plane."""
    r = P.PAN_FOURBAR["crank"]
    t = P.CLEVIS_TONGUE_T
    body = Rot(90, 0, 0) * _flat_arm(r, r0=P.HORN_DIA / 2 + 2.0, r1=PIN_R + 3.0, t=t)
    body -= Rot(90, 0, 0) * F.horn_holes(axis="y", length=t + 2)
    body -= Pos(0, 0, -(t / 2 - P.HORN_DISC_T / 2)) * _cyl_z(
        P.HORN_DIA / 2 + P.HORN_SEAT_CLEAR, P.HORN_DISC_T)
    body -= Pos(r, 0, 0) * _cyl_z(P.PIN_R + P.PIN_CLEARANCE, t + 8)
    return body


def pan_pushrod() -> Part:
    """Coupler from the pan crank to the neck column's yaw rocker."""
    L = P.PAN_FOURBAR["coupler"]
    t = P.CLEVIS_ROD_T
    hub = PIN_R + 3.0
    body = Rot(90, 0, 0) * _flat_arm(L, r0=hub, r1=hub, t=t)
    for x in (0.0, L):
        body -= Pos(x, 0, 0) * _cyl_z(P.PIN_R + P.PIN_CLEARANCE, t + 8)
    # its own lane, above the crank and the rocker — it shares a pin with each of them
    return Pos(0, 0, P.pan_rod_dz()) * body






# ------------------------------------------------------------------ the column
def _yaw_rocker() -> Part:
    """Rocker on the yaw axis, in the horizontal pan-linkage plane."""
    _, pts = pan_linkage()
    ox = P.HEAD_MOUNT_X
    rx, ry = pts["R"]
    ang = math.degrees(math.atan2(ry - 0.0, rx - ox))
    r = P.PAN_FOURBAR["rocker"]
    arm = Rot(90, 0, 0) * _flat_arm(r, r0=LINK_W * 0.6, r1=PIN_R + 3.0,
                                    t=P.CLEVIS_TONGUE_T)
    arm -= Pos(r, 0, 0) * _cyl_z(P.PIN_R + P.PIN_CLEARANCE, P.CLEVIS_TONGUE_T + 8)
    return Pos(0, 0, P.PAN_LINK_Z) * (Rot(0, 0, ang) * arm)




def neck_column() -> Part:
    """The ``head_pan`` link: yaw bearing at the bottom, pitch clevis at the top.

    It is a single printed strut carrying three things — the yaw axle that runs in the
    chest's bearing, the yaw rocker the chest pushrod drives, and the horn pad at the top
    that the head's pitch servo bolts to.
    """
    px, _, pz = P.pitch_origin_local()
    # yaw axle: a Dia6 stub down into the chest's bearings
    part = Pos(0, 0, 0.0) * _cyl_z(P.AXLE_R, 14.0)   # runs in the chest's yaw bearing
    part += Pos(0, 0, P.PAN_LINK_Z) * _cyl_z(COLUMN_R, P.CLEVIS_TONGUE_T + 6.0)
    part += _yaw_rocker()
    # The column stops SHORT of the pitch axis. The axis is inside the head's pitch servo,
    # so a strut that runs all the way to it is inside the servo's case; only the yoke arms,
    # which pass outboard of the case, may reach the axis.
    boss_half = (SERVO.pocket[1] + 2 * P.HIP_BOSS_WALL) / 2.0
    clear = boss_half + P.PITCH_YOKE_ARM / 2.0 + 1.0
    z_top = pz - clear
    n = 6
    for i in range(n):
        t0, t1 = i / n, (i + 1) / n
        x0, z0 = px * t0, P.PAN_LINK_Z + (z_top - P.PAN_LINK_Z) * t0
        x1, z1 = px * t1, P.PAN_LINK_Z + (z_top - P.PAN_LINK_Z) * t1
        seg = math.dist((x0, z0), (x1, z1))
        ang = math.degrees(math.atan2(z1 - z0, x1 - x0))
        part += Pos((x0 + x1) / 2, 0, (z0 + z1) / 2) * (
            Rot(0, 90 - ang, 0) * _cyl_z(COLUMN_R, seg + 2.0))
    # Pitch end: a C-YOKE straddling the head. The pitch servo is centred inside the head
    # (a 42 mm case hung off one side of the axis reaches outside the shell), so its horn
    # lands ~21 mm off the centre plane -- the yoke puts a horn pad there and a plain
    # pivot on the far arm, so the head is carried on both sides.
    pad_r = P.HORN_BOLT_CIRCLE / 2 + P.HEATSET[P.HORN_SCREW]["boss_r"]
    y = P.pitch_yoke_y()
    a = P.PITCH_YOKE_ARM
    # the cross-arm, clear below the servo's case, and the two arms rising outboard of it
    part += Pos(px, 0, z_top) * Box(a, 2 * y, a)
    for sgn in (+1, -1):
        part += Pos(px, sgn * (y - a / 2), (z_top + pz) / 2) * Box(a, a, pz - z_top)
    # driven side: the horn pad
    part += Pos(px, y - P.CLEVIS_TONGUE_T / 2, pz) * _cyl_y(pad_r, P.CLEVIS_TONGUE_T)
    part -= Pos(px, y - P.CLEVIS_TONGUE_T / 2, pz) * F.horn_holes(
        axis="y", length=P.CLEVIS_TONGUE_T + 2)
    # idle side: a plain pivot the head turns on
    part += Pos(px, -(y - P.CLEVIS_TONGUE_T / 2), pz) * _cyl_y(pad_r * 0.7, P.CLEVIS_TONGUE_T)
    part -= Pos(px, -(y - P.CLEVIS_TONGUE_T / 2), pz) * _cyl_y(
        P.AXLE_R + P.PIN_CLEARANCE, P.CLEVIS_TONGUE_T + 6)
    return part


if __name__ == "__main__":
    t2, pan = pan_linkage()
    print("pan  four-bar (fore x,y):", {k: (round(v[0], 1), round(v[1], 1)) for k, v in pan.items()})
    c = neck_column()
    b = c.bounding_box()
    print("neck_column vol %.0f  bbox %.0f x %.0f x %.0f" % (c.volume, b.size.X, b.size.Y, b.size.Z))
    for n, f in (("pan_crank", pan_crank), ("pan_pushrod", pan_pushrod)):
        print("  %-14s %.0f mm^3" % (n, f().volume))


def _pose_pan(t2: float):
    """(crank, pushrod) placements in the fore-torso frame at crank angle ``t2``."""
    from analysis.fourbar import FourBar
    fb = P.PAN_FOURBAR
    bar = FourBar(d=P.pan_ground(), r2=fb["crank"], r3=fb["coupler"], r4=fb["rocker"])
    pts = bar.linkage_points(t2)
    ox, sx = P.HEAD_MOUNT_X, P.PAN_SERVO[0]
    ev = ((sx - ox) / abs(sx - ox), 0.0)
    eu = (0.0, ev[0])
    def to_fore(v):
        u, w = v
        return (ox + u * eu[0] + w * ev[0], u * eu[1] + w * ev[1])
    cx, cy = to_fore(pts["C"])
    rx, ry = to_fore(pts["R"])
    z = P.PAN_BEARING_Z + P.PAN_LINK_Z
    phi = math.degrees(math.atan2(cy - 0.0, cx - sx))
    psi = math.degrees(math.atan2(ry - cy, rx - cx))
    return (Pos(sx, 0, z) * Rot(0, 0, phi),
            Pos(cx, cy, z) * Rot(0, 0, psi))


def pan_sweep(n: int = 11) -> Part:
    """The volume the pan linkage actually moves through, over its whole crank window.

    The chest needs a cavity for it, and a bounding box is the wrong cavity: the crank's
    swept disc alone is Ø70, and clearing that plus a corridor to the neck cut the ribcage
    into fourteen loose pieces. The links only ever occupy a thin curved region inside that
    box, so this unions their real poses and the torso subtracts exactly it.
    """
    lo, hi = P.PAN_FOURBAR["crank_window"]
    crank, rod = pan_crank(), pan_pushrod()
    out = None
    for i in range(n + 1):
        t2 = math.radians(lo + (hi - lo) * i / n)
        tc, tr = _pose_pan(t2)
        step = (tc * crank) + (tr * rod)
        out = step if out is None else out + step
    return out


def neck_sweep(n: int = 8) -> Part:
    """The volume the neck column moves through as the head pans.

    Same reason as ``pan_sweep``: a disc sized to the column's furthest reach is far
    bigger than the column, and cutting it out of the chest severs the braces that hold
    the yaw bearing in place. The column only ever occupies a sector of it.

    ``n`` is EVEN so the sample set includes the neutral pose — with an odd count the
    middle of the range falls between samples and the column's rest position is the one
    place the chest is not cleared for.
    """
    lo, hi = P.LIM_HEAD_PAN
    col = neck_column()
    out = None
    for i in range(n + 1):
        a = math.degrees(lo + (hi - lo) * i / n)
        posed = Rot(0, 0, a) * col
        out = posed if out is None else out + posed
    return Pos(*P.pan_origin()) * out


def head_sweep(n_pan: int = 3, n_pitch: int = 3) -> Part:
    """The volume the HEAD moves through, in the fore-torso frame.

    The chest has to be hollowed for it, and a plain ball is the wrong shape: the head is
    not a ball — the muzzle reaches 56 mm from its centre against a 50 mm shell — and it
    turns about a yaw axis that is not its own centre. Sampling the real poses is both
    tighter where the ball was too big and correct where it was too small.
    """
    from cad.parts.head import head

    h = head()
    hx, _, hz = P.head_centre()
    ox, _, oz = P.pan_origin()
    lo_a, hi_a = P.LIM_HEAD_PAN
    lo_b, hi_b = P.LIM_HEAD_PITCH
    out = None
    for i in range(n_pan):
        a = math.degrees(lo_a + (hi_a - lo_a) * i / max(n_pan - 1, 1))
        for j in range(n_pitch):
            b = math.degrees(lo_b + (hi_b - lo_b) * j / max(n_pitch - 1, 1))
            posed = (Pos(ox, 0, oz) * Rot(0, 0, a)
                     * Pos(hx - ox, 0, hz - oz) * Rot(0, b, 0) * h)
            out = posed if out is None else out + posed
    return out
