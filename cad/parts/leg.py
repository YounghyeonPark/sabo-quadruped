"""
Leg links — 4 printed parts per leg (digitigrade cat limb).
===========================================================

Each part is modelled in the local frame of its *proximal* joint (origin), limb
pointing down -z at neutral, so the chain composes by joint transforms:

    hip_bracket : abduction output -> carries the hip/shoulder servo (spans hip_off)
    upper_leg   : hip/shoulder pivot (origin) -> knee/elbow at (0,0,-upper)
    lower_leg   : knee/elbow pivot (origin)   -> ankle/hock at (0,0,-lower)
    foot_seg    : ankle/hock pivot (origin)   -> toe pad at (0,0,-foot)  [digitigrade]

Front and rear legs use the same builders with different lengths (``cad.params``).

The segments are **organic tapered struts**, not boxes: each is a loft of elliptical
cross-sections down the local -z axis, gently waisted in the middle and slightly
bulbous at the joint ends so it reads like a bone/muscle strut inside the skin.
The proximal joint stays at the origin and the distal joint stays at (0,0,-L) — only
the cross-section changes, so kinematics/IK/MJCF are untouched. Lightening is done
with lateral fenestration holes (robust; no fragile thin-shell inner loft).

Four-bar knee (params.FOURBAR / analysis.fourbar)
-------------------------------------------------
The knee servo lives **up the thigh** at a crank pivot ``ground`` mm above the
knee (relocated off the knee, which is now a passive pin/bearing). A crank drives
a rigid pushrod (coupler) down to a rocker rigidly welded on the lower leg at the
knee. Link lengths come straight from ``P.FOURBAR`` so the CAD matches the verified
kinematics; the four pivots (crank pivot, crank–coupler, coupler–rocker, knee) all
get an M3-ish Ø3 pin bore / bearing seat. The knee joint itself is unchanged
(direct hinge in sim) — this is the *mechanical* realisation only.
"""

from __future__ import annotations

import math

from build123d import (BuildPart, BuildSketch, Box, Cylinder, Ellipse, Locations,
                       Part, Plane, Pos, Rot, Sphere, fillet, loft, scale)

from analysis.fourbar import FourBar
from cad import params as P
from cad.parts import fasteners as F
from cad.servo import DEFAULT as SERVO


def _servo_pocket() -> Part:
    l, w, h = SERVO.pocket
    return Box(l, w, h)


def _rounded_box(dx: float, dy: float, dz: float, r: float) -> Part:
    """A box with every edge filleted — a robust, mildly organic rounded block used
    as a servo boss. ``r`` must stay below half the smallest side."""
    r = min(r, 0.49 * min(dx, dy, dz))
    with BuildPart() as p:
        Box(dx, dy, dz)
        fillet(p.edges(), radius=r)
    return p.part


def _servo_boss(ext: tuple[float, float, float], wall: float = 3.0,
                r: float = 5.0) -> Part:
    """Rounded block sized to FULLY enclose an (already oriented) servo pocket whose
    axis-aligned extents are ``ext``, leaving ``wall`` mm of material on every face.
    Sizes off ``SERVO.pocket`` via the caller, so a future servo swap re-fits."""
    ex, ey, ez = ext
    return _rounded_box(ex + 2 * wall, ey + 2 * wall, ez + 2 * wall, r)


# ----------------------------------------------------- four-bar knee linkage parts
PIN_R = 1.5              # Ø3 pin at every pivot (M3-ish)
LINK_T = 7.0             # link thickness along the pin (local Y) axis
LINK_W = 11.0            # link cross width (local Z)
HORN_R = SERVO.horn_dia / 2.0


def _fourbar() -> FourBar:
    fb = P.FOURBAR
    return FourBar(d=fb["ground"], r2=fb["crank"], r3=fb["coupler"],
                   r4=fb["rocker"], rocker_offset=fb["rocker_offset"])


def stance_linkage():
    """The verified four-bar drawn at its **mid-window** (stance) crank pose, in the
    thigh frame with the knee at the fourbar origin. Returns ``(t2_mid_rad, pts)``
    where ``pts`` = O2/C/R/O4 (mm) with the exact FOURBAR lengths. Mid-window keeps
    stance well away from the transmission-angle extremes (symmetric ROM headroom).
    Used both to weld the rocker (leg.py) and to pose crank+pushrod (assembly.py)."""
    lo, hi = P.FOURBAR["crank_window"]
    t2 = math.radians((lo + hi) / 2)
    return t2, _fourbar().linkage_points(t2)


def _cyl_y(radius: float, height: float) -> Part:
    """Solid cylinder with its axis along local Y — the linkage pivot axis."""
    return Rot(90, 0, 0) * Cylinder(radius=radius, height=height)


def _pin_bore(length: float = LINK_T * 4, rotating: bool = True) -> Part:
    """Ø3 pin bore along the pivot (Y) axis, to subtract at a pivot. All four-bar +
    knee pivots ROTATE, so the bore carries P.PIN_CLEARANCE (loose fit); a fixed
    (pressed) pin passes ``rotating=False`` for a PRESS_INTERFERENCE bore."""
    return F.pin_bore(length, rotating=rotating, axis="y")


def _flat_arm(length: float, r0: float, r1: float, t: float = LINK_T) -> Part:
    """A flat link in the x-z plane from the proximal pivot (0,0,0) to the distal
    pivot (length,0,0); pin axis = Y. Rounded hubs at both pivots + a joining web."""
    hub0 = _cyl_y(r0, t)
    hub1 = Pos(length, 0, 0) * _cyl_y(r1, t)
    web = Pos(length / 2, 0, 0) * Box(length, t, min(r0, r1) * 1.7)
    return hub0 + hub1 + web


def crank() -> Part:
    """Knee-servo crank (FOURBAR['crank'] mm): mounts on the servo horn at the crank
    pivot (origin) and carries the crank–coupler pin at (crank,0,0)."""
    r = P.FOURBAR["crank"]
    body = _flat_arm(r, r0=HORN_R + 2.0, r1=PIN_R + 3.0)
    body -= F.horn_holes(axis="y", length=LINK_T + 2)                # horn bolt circle + centre screw
    body -= Pos(0, LINK_T / 2 - 1.1, 0) * _cyl_y(HORN_R + 0.4, 2.4)   # horn disc seat recess
    body -= Pos(r, 0, 0) * _pin_bore()                               # crank–coupler pin (rotating)
    body -= Pos(r, 0, 0) * F.pin_head_seat("y", face=LINK_T / 2)     # pin retention (e-clip / head)
    return body


def pushrod() -> Part:
    """Rigid pushrod / coupler (FOURBAR['coupler'] mm) with a pin bore + shallow
    bearing seats at each end (crank tip at origin, rocker tip at coupler)."""
    L = P.FOURBAR["coupler"]
    t = LINK_T * 0.85
    body = _flat_arm(L, r0=PIN_R + 3.0, r1=PIN_R + 3.0, t=t)
    for x in (0.0, L):
        body -= Pos(x, 0, 0) * _pin_bore()
        for yf in (1, -1):                                           # bearing counterbores
            body -= Pos(x, yf * (t / 2 - 0.6), 0) * _cyl_y(PIN_R + 1.4, 1.4)
    return body


def _rocker_arm(stance_knee: float) -> Part:
    """Rocker (FOURBAR['rocker'] mm) rigidly welded to the lower leg at the knee
    (origin). Its weld angle is set so that, when the shank is posed at this leg's
    stance knee, the rocker tip lands exactly on the four-bar's R point (loop
    closes with the verified 50 mm coupler)."""
    r = P.FOURBAR["rocker"]
    _, pts = stance_linkage()
    rx, rz = pts["R"]                         # fourbar (x_fb, y_fb) -> thigh (x, z) rel. knee
    k = stance_knee                           # the knee pose applies Rot_y(+k), so pre-weld
    lx = rx * math.cos(k) + rz * math.sin(k)  # the rocker by Rot_y(-k): local = Rot_y(-k).R
    lz = -rx * math.sin(k) + rz * math.cos(k)
    ang = math.degrees(math.atan2(lz, lx))
    arm = _flat_arm(r, r0=LINK_W * 0.6, r1=PIN_R + 3.0)
    arm -= Pos(r, 0, 0) * _pin_bore()         # coupler–rocker pin at the tip (rotating)
    arm -= Pos(r, 0, 0) * F.pin_head_seat("y", face=LINK_T / 2)   # pin retention seat
    return Rot(0, -ang, 0) * arm              # swing the +x arm onto the weld direction


def _servo_pack(px: float, pz: float, pivot_y: float, sign: int,
                long_axis: str = "x", wall: float = 3.0, r: float = 5.0,
                off: tuple[float, float] = (0.0, 0.0)):
    """A servo housing whose OUTPUT SHAFT lies on the local Y axis at ``(px, pivot_y,
    pz)`` (the joint) with the servo BODY tucked INBOARD — toward the centreline —
    so it protrudes minimally in +|Y|. This is the shoulder/hip anti-"box" trick:
    the shaft (and horn) stay exactly at the joint (axis + origin unchanged), only
    the ~36 mm-deep body is pushed to the low-|Y| side of the joint.

    The STS3215 shaft exits its ``body_h`` (tall) face, so ``body_h`` is the depth
    along the shaft (Y); the ``45x24`` cross-section lies in X-Z. ``long_axis`` picks
    which in-plane axis the 45 mm length runs along ('x' = fore-aft, 'z' = up/down the
    limb) so a leg's two servos can be staggered to clear one another. Returns
    ``(boss, cutters)``: union the boss into the strut, then subtract the cutters.

    The boss encloses the ``SERVO.pocket`` with ``wall`` mm on every face and its
    OUTER face sits on the joint plane, so the outer |Y| of the whole housing is just
    the joint offset + one wall (vs. the old box that straddled the joint ±half the
    servo LENGTH). ``sign`` is the leg side (+1 L / -1 R): inboard = toward y=0."""
    l, w, h = SERVO.pocket
    dy = h                                          # depth along the shaft (Y)
    dx, dz = (l, w) if long_axis == "x" else (w, l)
    dyb = dy + 2 * wall
    cy = pivot_y - sign * dyb / 2.0                 # boss centre: outer face on the joint plane
    # ``off`` shifts the BODY in the (X, Z) plane while the shaft stays on the joint
    # axis (this is the real servo's shaft_from_end offset) — used to slide one leg's
    # two inboard servos apart so they don't overlap.
    bc = Pos(px + off[0], cy, pz + off[1])
    boss = bc * _rounded_box(dx + 2 * wall, dyb, dz + 2 * wall, r)
    cut = bc * Box(dx, dy, dz)                                    # servo pocket
    cut = cut + Pos(px, cy, pz) * _cyl_y(3.0, dyb)               # shaft/horn relief on the axis
    # STS3215 case-retention screws (M2), parallel to the shaft, into the flange face.
    cut = cut + bc * F.servo_case_screws("y", (dx, dz), length=dyb + 14)
    return boss, cut


def _crank_pivot_boss(strut: Part, length: float, sign: int) -> Part:
    """Relocate the knee servo UP the thigh: a boss at ``ground`` mm above the knee
    housing the knee servo whose output IS the crank pivot (shaft on Y at the thigh
    centreline). The body is tucked INBOARD via ``_servo_pack`` so the crank boss no
    longer juts ~21 mm past the joint plane. Its 45 mm length runs FORE-AFT (not down
    the thigh), so its 24 mm width takes the vertical (Z) — that keeps the crank servo
    clear (in Z) of the hip servo above it, which is now also tucked inboard in the
    same Y band; both servos sharing the inboard band would otherwise collide."""
    z = -(length - P.FOURBAR["ground"])
    # shift the crank servo body ~6 mm DOWN the thigh (shaft stays on the crank pivot)
    # so its top clears the hip servo boss centred 29.5 mm above it.
    boss, cut = _servo_pack(0.0, z, 0.0, sign, long_axis="x", off=(0.0, -6.0))
    return strut + boss - cut


def _pivot_seat(strut: Part, z: float, boss_r: float = 6.0) -> Part:
    """Passive pivot (knee or ankle): a bearing boss + Ø3 rotating pin bore along Y at
    height ``z``, with a retention counterbore (e-clip / shoulder-screw head) on the
    +Y face. Used for the passive knee AND the passive foot↔lower-leg ankle pin."""
    hb = LINK_T + 5.0
    strut = strut + Pos(0, 0, z) * _cyl_y(boss_r, hb)
    strut -= Pos(0, 0, z) * _pin_bore(length=44)
    strut -= Pos(0, 0, z) * F.pin_head_seat("y", face=hb / 2)
    return strut


# back-compat alias (knee pivot is the original caller)
_knee_pivot_seat = _pivot_seat


def _bone_strut(length: float, rp: tuple[float, float], rd: tuple[float, float],
                waist: float = 0.86) -> Part:
    """Lofted elliptical strut from the proximal joint (z=0) to the distal joint
    (z=-length). ``rp``/``rd`` are the (x,y) half-widths at the ends; the middle is
    pinched by ``waist`` for a subtle bone silhouette."""
    rxp, ryp = rp
    rxd, ryd = rd
    stations = [
        (0.00, rxp, ryp),
        (0.18, rxp * waist, ryp * waist),
        (0.50, min(rxp, rxd) * waist, min(ryp, ryd) * waist),
        (0.82, rxd * waist, ryd * waist),
        (1.00, rxd, ryd),
    ]
    with BuildPart() as p:
        for t, rx, ry in stations:
            with BuildSketch(Plane.XY.offset(-t * length)):
                Ellipse(rx, ry)
        loft()
    return p.part


def _fenestrae(part: Part, length: float, ry: float) -> Part:
    """Lightening: two lateral (fore-aft) bores through the shaft — organic, robust."""
    r = max(min(ry * 0.42, 3.2), 2.0)
    for tz in (0.36, 0.62):
        bore = Rot(0, 90, 0) * Cylinder(radius=r, height=length)  # axis -> local x
        part -= Pos(0, 0, -length * tz) * bore
    return part


def hip_bracket(sign: int = 1, hip_off: float = 28.0, x_shift: float = 0.0) -> Part:
    """Slim shoulder BEARING BLOCK (remote-axle hip drive, params.HIP_DRIVE).

    The hip/shoulder servo BODY is GONE from here — it now lives in the torso core
    (body.py ``_core_hip_drive``). This part rides the (rigid) abduction output at the
    mount (local origin, world |y|=BODY_W/2) and, at the hip pivot (local
    ``y=sign*hip_off`` → world 79.5/86, UNCHANGED), carries the OUTBOARD drive-axle
    bearing (#2). The lateral axle spins in that bearing and couples to the upper leg
    just beyond; the block is otherwise a thin web tying the bearing back to the mount
    hub. So the shoulder band holds only a Ø(bearing) boss + slim web instead of the fat
    ~45 mm servo body → the skin can hug the body.

    ``x_shift`` is retained for call-signature compatibility but no longer needed (there
    is no servo body out here to slide fore-aft)."""
    hb = P.HIP_BEARING
    wall = P.HIP_BOSS_WALL
    yj = sign * hip_off
    blen = hb["width"] + 6
    # Bearing (#2) boss sits just INBOARD of the hip pivot plane (its outboard face ~1 mm
    # inboard of the joint), so the upper leg's horn hub couples to the axle at the joint
    # without clashing with this boss.
    yb = yj - sign * (blen / 2.0 + 1.0)
    part = Pos(0, yb, 0) * _cyl_y(hb["od_r"] + wall, blen)          # outboard drive-axle bearing
    # abduction hub at the mount (origin), tying the block to the torso mount node
    part += scale(Sphere(1), (SERVO.body_w * 0.45 + 3, 10.0, 12.0))
    # slim web bridging hub -> bearing along Y (the shoulder link; kept narrow in X/Z)
    part += Pos(0, yb / 2.0, 0) * Box(2 * (hb["od_r"] + wall), abs(yb), 2 * hb["od_r"])
    # axle channel straight through the whole block on the Y hip axis
    part -= _cyl_y(hb["bore_r"] + P.AXLE_CLEAR, 3 * abs(yj) + 40)
    # bearing seat (wider than the axle bore) recessed from the inboard face of the boss
    part -= Pos(0, yb, 0) * _cyl_y(hb["od_r"], hb["width"])
    part -= Pos(0, yb, 0) * F.pin_head_seat("y", face=-sign * (blen / 2.0))
    return part


def hip_axle(hip_off: float) -> Part:
    """Lateral hip DRIVE AXLE (remote-axle hip drive). A Ø(2*AXLE_R) shaft on the Y hip
    axis from the core servo horn (inboard) out to the upper-leg coupling (outboard),
    spinning in two bearings (core wall #1 + hip bracket #2). Modelled in its own frame:
    shaft along +Y, inboard horn-clamp hub at y=0, outboard horn-mimic flange at y=L.

      * inboard end  — a hub that clamps the core servo's Ø20 STS3215 horn (driven).
      * outboard end — a Ø20 disc carrying the SAME horn bolt-circle the UPPER LEG
        expects, so the thigh's ``_hip_horn_mount`` bolts to the axle exactly as it used
        to bolt to a servo horn (upper_leg is UNCHANGED).

    Rigid coupling → kinematically invisible (the hip is still one Y hinge at the same
    origin). Printed/ordered as a small part; steel/CF rod, so not in the plastic mass
    budget (like the four-bar pins + split dowels)."""
    L = (P.BODY_W / 2 + hip_off) - P.HIP_CORE_HORN_Y
    hub_r = P.HORN_BOLT_CIRCLE / 2 + P.HEATSET[P.HORN_SCREW]["boss_r"]
    part = Pos(0, L / 2.0, 0) * _cyl_y(P.AXLE_R, L)                    # shaft 0..L
    part += _cyl_y(hub_r, LINK_T)                                     # inboard horn-clamp hub
    part += Pos(0, L, 0) * _cyl_y(P.HORN_DIA / 2.0, 3.0)              # outboard Ø20 horn-mimic
    part -= F.horn_holes(axis="y", length=LINK_T + 2)                 # clamp to core servo horn
    part -= Pos(0, L, 0) * F.horn_holes(axis="y", length=3.0 + 2)     # upper-leg bolt circle
    return part


def _segment(length: float, knee_servo: bool, rp, rd, lighten: bool) -> Part:
    strut = _bone_strut(length, rp, rd)
    if lighten and length > 30:
        strut = _fenestrae(strut, length, rd[1])
    if knee_servo:                          # distal servo pocket (drives next joint)
        strut -= Pos(0, 0, -length) * (Rot(0, 0, 90) * _servo_pocket())
    return strut


def _hip_horn_mount(strut: Part) -> Part:
    """Horn coupling at the hip pivot (origin): a disc pad that clamps onto the hip
    servo's Ø20 horn, with the M2 horn bolt-circle inserts + centre screw. Symmetric
    about the pivot so the same part serves the mirrored L / R legs."""
    pad_r = P.HORN_BOLT_CIRCLE / 2 + P.HEATSET[P.HORN_SCREW]["boss_r"]
    strut = strut + _cyl_y(pad_r, LINK_T)
    strut -= F.horn_holes(axis="y", length=LINK_T + 2)
    return strut


def upper_leg(length: float, sign: int = 1) -> Part:
    # knee servo relocated UP the thigh to the crank pivot; the knee is now passive.
    # ``sign`` (leg side) tucks the crank servo body inboard, so the L/R thighs are
    # mirror images (print one flipped) — necessary to pull the knee servo off the
    # silhouette without moving the crank pivot.
    strut = _segment(length, knee_servo=False, rp=(7.6, 8.6), rd=(6.2, 7.6), lighten=True)
    strut = _crank_pivot_boss(strut, length, sign)
    strut = _knee_pivot_seat(strut, -length)          # passive pin/bearing at the knee
    return _hip_horn_mount(strut)                      # driven by the hip servo horn


def lower_leg(length: float, stance_knee: float) -> Part:
    # In the four-bar realisation the ONLY knee servo lives on the thigh crank boss
    # and the knee/ankle are passive pins — so the shank carries no servo pocket. The
    # distal pocket only belongs to a 'direct' knee build (servo at the joint); cutting
    # it in four-bar mode just lopped the bottom ~15 mm off the shank (blown-through).
    direct = (P.KNEE_DRIVE == "direct")
    strut = _segment(length, knee_servo=direct, rp=(7.2, 8.2), rd=(5.8, 7.2), lighten=True)
    strut = strut + _rocker_arm(stance_knee)          # rocker welded on the shank at the knee
    strut = _knee_pivot_seat(strut, 0.0)              # passive pin/bearing at the knee
    return _pivot_seat(strut, -length, boss_r=5.5)    # passive ankle pin (foot↔lower leg)


def foot_seg(length: float) -> Part:
    """Metatarsus + shaped paw pad. Proximal (ankle) at origin, toe at -z. The
    ankle (foot↔lower-leg join) is a passive Ø3 pin: a bearing seat at the origin
    mates the lower-leg's distal ankle seat."""
    strut = _bone_strut(length, rp=(6.0, 7.0), rd=(4.6, 5.6))
    strut = _pivot_seat(strut, 0.0, boss_r=5.5)       # passive ankle pin (foot↔lower leg)
    # flattened, slightly forward paw pad in place of a bare sphere
    pad = scale(Sphere(P.TOE_R), (1.28, 1.06, 0.82))
    paw = Pos(2.5, 0, -length) * pad
    return strut + paw


def leg_parts(leg: str) -> dict[str, Part]:
    from sim.gait import stance_angles
    g = P.leg_geom(leg)
    s = P.leg_plane_sign(leg)
    _, knee = stance_angles(leg)                      # weld the rocker for this leg's stance
    # slide the fore-aft hip servo body toward the waist (front legs: -x, rear: +x) so
    # it stays inside the torso length rather than poking past the nose / tail.
    x_shift = -16.0 if P.is_front(leg) else 16.0
    return {
        "hip_bracket": hip_bracket(s, g["hip_off"], x_shift),
        "axle": hip_axle(g["hip_off"]),
        "upper": upper_leg(g["upper"], s),
        "lower": lower_leg(g["lower"], knee),
        "foot": foot_seg(g["foot"]),
    }


if __name__ == "__main__":
    for leg in ("FL", "RL"):
        print(leg, {n: round(p.volume, 1) for n, p in leg_parts(leg).items()})
    print("crank vol", round(crank().volume, 1), "pushrod vol", round(pushrod().volume, 1))
