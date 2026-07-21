"""
Torso frame — a curved **ribcage** skeleton (two halves at the waist).
======================================================================

The inner frame is built *additively* as a real skeleton, not a box or a solid
shell: elliptical **rib hoops** at stations along the body, tied together by four
longitudinal **stringers** (spine / keel / two flanks), with solid **leg-mount
nodes** (servo pockets) at the four hips. All unions of solid struts — robust,
and it reads as an organic curved ribcage inside the translucent skin.

``torso_fore`` (front legs + head) is ahead of the waist; ``torso_aft`` (rear
legs + tail) behind. Each half's local frame origin is at the waist (x=0). The
rib ellipses sit a few mm inside the skin cavity so the skin fits over them.

Stations: (x, half_width_y, half_height_z) mm — the body contour, shrunk from the
skin loft.
"""

from __future__ import annotations

from build123d import (Box, BuildPart, BuildSketch, Cylinder, Ellipse, Locations,
                       Mode, Part, Plane, Pos, Rectangle, Rot, extrude, loft)

from cad import params as P
from cad.parts import fasteners as F
from cad.servo import DEFAULT as SERVO

RIB_W = 5.0        # rib radial width (mm)
RIB_T = 4.0        # rib thickness along the body axis (mm)
STRINGER = 5.0     # stringer square cross-section (mm)

# Base contour (drawn at BODY_W_REF x BODY_H_REF); the y half-widths scale with
# BODY_W and the z half-heights with BODY_H so the ribcage tracks the (optimized)
# torso size and the leg-mount nodes stay fused to the cage.
_FORE_BASE = [(0, 25, 23), (30, 33, 29), (58, 36, 33), (80, 33, 32), (90, 19, 19)]
_AFT_BASE = [(0, 25, 23), (-40, 37, 33), (-72, 31, 29), (-90, 16, 16)]


def _scaled(stations):
    wy = P.BODY_W / P.BODY_W_REF
    wz = P.BODY_H / P.BODY_H_REF
    return [(x, a * wy, b * wz) for (x, a, b) in stations]


FORE_STATIONS = _scaled(_FORE_BASE)
AFT_STATIONS = _scaled(_AFT_BASE)


def _interp(stations, extra: int = 1):
    """Insert ``extra`` evenly-spaced ribs between each pair for a fuller cage."""
    out = []
    for i in range(len(stations) - 1):
        x0, a0, b0 = stations[i]
        x1, a1, b1 = stations[i + 1]
        for k in range(extra + 1):
            t = k / (extra + 1)
            out.append((x0 + (x1 - x0) * t, a0 + (a1 - a0) * t, b0 + (b1 - b0) * t))
    out.append(stations[-1])
    return out


def _rib(x, a, b) -> Part:
    """A flat elliptical hoop (rib) at station x, thickness RIB_T along x."""
    with BuildPart() as p:
        with BuildSketch(Plane.YZ.offset(x)):
            Ellipse(a, b)
            Ellipse(max(a - RIB_W, 0.5), max(b - RIB_W, 0.5), mode=Mode.SUBTRACT)
        extrude(amount=RIB_T / 2, both=True)
    return p.part


def _stringer(stations, oy, oz) -> Part:
    """Longitudinal beam through (oy·a, oz·b) of each rib — a curved bar following
    the body (oy,oz select spine/keel/flank via signs).

    Built PIECEWISE — a 2-section loft between each adjacent station pair, then
    unioned — rather than one multi-section loft through all stations. A single loft
    across the FORE half's sharp nose taper (80,33,32)->(90,19,19) produced OCCT
    ``UnorientableShape`` faces, leaving ``torso_fore`` an invalid solid (unreliable to
    slice/print). Pairwise lofts are geometrically equivalent but robust, so both
    halves come out valid + watertight."""
    seg = None
    for i in range(len(stations) - 1):
        with BuildPart() as p:
            for x, a, b in (stations[i], stations[i + 1]):
                with BuildSketch(Plane.YZ.offset(x)):
                    with Locations((oy * a, oz * b)):
                        Rectangle(STRINGER, STRINGER)
            loft()
        seg = p.part if seg is None else seg + p.part
    return seg


def _cyl_y(radius: float, height: float) -> Part:
    """Solid cylinder whose axis is the local Y (the hip / bearing / axle axis)."""
    return Rot(90, 0, 0) * Cylinder(radius=radius, height=height)


def _core_hip_drive(mx: float, s: int) -> Part:
    """HIP servo relocated INTO the torso core + the lateral drive-axle bearing at the
    core wall (remote-axle hip drive, params.HIP_DRIVE).

    The servo's output shaft stays on the (unchanged) Y hip axis at ``x=mx, z=0``; its
    body tucks toward the centreline (horn face at |y|=HIP_CORE_HORN_Y, the ~36 mm body
    reaching further inboard). The 45 mm servo length runs VERTICAL (Z) and its 24 mm
    width fore-aft (X) so the body stays inside FORE_LEN in X and clears the Jetson bay
    (which sits further aft, x < ~60). A Ø6 axle exits the core through a Ø13 bearing
    (#1) seated here in the core wall, then runs out to the hip pivot where bearing #2
    (in the slim hip bracket, leg.py) supports it and it couples to the upper leg."""
    l, w, h = SERVO.pocket                     # (45.8, 24.8, 36.4): shaft exits the h face
    wall = P.HIP_BOSS_WALL
    y_horn = s * P.HIP_CORE_HORN_Y
    cy = y_horn - s * (h / 2.0)                # body centre, inboard of the horn face
    bc = Pos(mx, cy, 0)
    #  X=w(24, fore-aft), Y=h(36, shaft depth), Z=l(45, vertical)
    part = bc * Box(w + 2 * wall, h + 2 * wall, l + 2 * wall)     # servo boss
    part -= bc * Box(w, h, l)                                     # servo pocket
    # STS3215 case-retention screws parallel to the shaft (Y) into the flange face
    part -= bc * F.servo_case_screws("y", (w, l), length=h + 2 * wall + 8)
    # axle / horn channel straight out along the Y hip axis (through the boss + wall)
    part -= Pos(mx, 0, 0) * _cyl_y(P.AXLE_R + 1.0, 2 * P.BODY_W)
    # inboard drive-axle bearing (#1) where the shaft exits the core wall (~BODY_W/2)
    hb = P.HIP_BEARING
    by = s * (P.BODY_W / 2)
    part += Pos(mx, by, 0) * _cyl_y(hb["od_r"] + wall, hb["width"] + 6)
    part -= Pos(mx, by, 0) * _cyl_y(hb["od_r"], hb["width"])                 # bearing seat
    part -= Pos(mx, by, 0) * _cyl_y(hb["bore_r"] + P.AXLE_CLEAR, hb["width"] + 24)  # axle bore
    return part


def _half(stations, legs) -> Part:
    ribs = _interp(stations, extra=1)
    frame = _rib(*ribs[0])
    for r in ribs[1:]:
        frame += _rib(*r)
    # four stringers: spine (top), keel (bottom), two flanks
    for oy, oz in ((0, 1), (0, -1), (1, 0), (-1, 0)):
        frame += _stringer(stations, oy, oz)
    # core-mounted hip servos + drive-axle wall bearings (remote-axle hip drive): the
    # ~45 mm servo body no longer floors at the hip joint line out in the shoulder; it
    # lives here in the core near the centreline, driving a lateral axle out to the hip.
    for leg in legs:
        mx, my = P.MOUNTS[leg]
        s = P.leg_plane_sign(leg)
        frame += _core_hip_drive(mx, s)
    return frame


# --------------------------------------------------------------- waist interface
# The two halves meet at the waist joint (sagittal spine, axis Y through the origin).
# It is a JOINT, not a rigid bolt: the waist STS3215 body mounts in the AFT half and
# its Ø20 horn drives the FORE half. Each half gets a compact keel-connected bulkhead
# at the waist plane carrying that hardware, so both are attached + printable. A bus
# pass-through lets the daisy-chain trunk cross the waist. (No joint origin / length
# change — this only adds the servo pocket + horn interface + fastener features.)
_WAIST_BH_T = 6.0          # bulkhead plate thickness along the body axis (X)


def _waist_bulkhead(x_center: float) -> Part:
    """A compact plate spanning the lower body at the waist plane, overlapping the
    keel + flank stringers so it prints attached to the ribcage."""
    wy = P.BODY_W / P.BODY_W_REF
    wz = P.BODY_H / P.BODY_H_REF
    plate = Box(_WAIST_BH_T, 2 * 22 * wy, 30 * wz)      # width ~ interior, low-center
    return Pos(x_center, 0, -4 * wz) * plate


def _waist_aft_features(frame: Part) -> Part:
    """AFT half: waist servo pocket (shaft along the waist axis Y) + case screws +
    shaft relief + the daisy-chain crossing hole."""
    l, w, h = SERVO.pocket
    cx = -(_WAIST_BH_T / 2 + 2)
    frame += _waist_bulkhead(cx)
    node = Pos(cx, 0, 0)
    frame -= node * (Rot(90, 0, 0) * Box(l, w, h))       # pocket, horn/shaft axis -> Y
    frame -= node * F.servo_case_screws("y", (l, w), length=h + 14)
    frame -= node * F.wire_hole("bus", "y", h + 20)      # shaft/output relief along Y
    frame -= Pos(0, 12, 8) * F.wire_hole("bus", "x", 60)  # daisy-chain crosses the waist
    return frame


def _waist_fore_features(frame: Part) -> Part:
    """FORE half: horn coupling pad driven by the waist servo's Ø20 horn on the waist
    axis (Y), + the matching daisy-chain crossing hole."""
    pad_r = P.HORN_BOLT_CIRCLE / 2 + P.HEATSET[P.HORN_SCREW]["boss_r"]
    cx = _WAIST_BH_T / 2 + 2
    frame += _waist_bulkhead(cx)
    frame += Pos(cx, 0, 0) * (Rot(90, 0, 0) * Cylinder(pad_r, _WAIST_BH_T))
    frame -= Pos(cx, 0, 0) * F.horn_holes(axis="y", length=_WAIST_BH_T + 4)
    frame -= Pos(0, 12, 8) * F.wire_hole("bus", "x", 60)  # daisy-chain crosses the waist
    return frame


def torso_fore() -> Part:
    return _waist_fore_features(_half(FORE_STATIONS, ["FL", "FR"]))


def torso_aft() -> Part:
    return _waist_aft_features(_half(AFT_STATIONS, ["RL", "RR"]))


if __name__ == "__main__":
    print("fore ribcage vol mm^3:", round(torso_fore().volume, 1))
    print("aft  ribcage vol mm^3:", round(torso_aft().volume, 1))
