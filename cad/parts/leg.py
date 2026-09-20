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


# ----------------------------------------------------- clevis (double-shear) pivots
# Two links that pivot together CANNOT both be drawn on the leg's centre plane — they
# would occupy the same solid. Every pin joint here is therefore a clevis: one link ends
# in a two-cheek FORK, the mating link ends in a TONGUE that sits in the slot between the
# cheeks, and the pin passes through cheek-tongue-cheek (double shear, so the pin is not
# cantilevered). Lane widths come from ``cad.params.clevis_slot``.


def _clevis_void(r: float, open_dir: int, slot_w: float) -> Part:
    """The cavity a FORK must contain at a pivot: a disc so the tongue's hub can turn
    through the joint's range, plus a channel out of the fork mouth so the tongue's own
    shaft can swing. ``open_dir`` is the local ±z the mating link leaves towards."""
    rr = r + P.CLEVIS_GAP
    void = _cyl_y(rr, slot_w)
    void += Pos(0, 0, open_dir * rr / 2.0) * Box(2 * rr, slot_w, rr)
    return void


def _fork_seat(part: Part, z: float, r: float, open_dir: int, rod: bool = False) -> Part:
    """FORK half of a clevis at height ``z``: two cheeks straddling the slot, the Ø3 pin
    bore through both, and a retention seat (e-clip / screw head) on each outer face."""
    lo, hi = P.clevis_slot(rod)
    slot_w = hi - lo
    body_w = slot_w + 2 * P.CLEVIS_CHEEK_T
    part = part + Pos(0, 0, z) * _cyl_y(r, body_w)
    part -= Pos(0, 0, z) * _clevis_void(r, open_dir, slot_w)
    part -= Pos(0, 0, z) * _pin_bore(length=body_w + 10)
    for face in (+1, -1):
        part -= Pos(0, 0, z) * F.pin_head_seat("y", face=face * body_w / 2)
    return part


def _tongue_seat(part: Part, z: float, r: float, open_dir: int,
                 reach: float = 0.0) -> Part:
    """TONGUE half of a clevis: thin everything around the pivot down to the tongue lane
    so it enters the mating fork's slot, then add the hub and the pin bore.

    ``reach`` carries that thinning on PAST the pivot, down the link's own shaft. The
    shank needs it: its top does not merely enter the thigh's fork, it lives inside the
    thigh channel alongside the pushrod, so it has to stay a blade for as long as the
    rod runs beside it. Bending in the sagittal plane is unaffected (the x-section is
    untouched); only lateral stiffness is traded, which this planar linkage does not use."""
    t = P.CLEVIS_TONGUE_T
    rr = r + 1.5
    region = _cyl_y(rr, 80.0) + Pos(0, 0, open_dir * rr / 2.0) * Box(2 * rr, 80.0, rr)
    if reach > 0.0:
        region += Pos(0, 0, -open_dir * reach / 2.0) * Box(2 * rr, 80.0, reach)
    part -= Pos(0, 0, z) * (region - Box(400.0, t, 400.0))
    part = part + Pos(0, 0, z) * _cyl_y(r, t)
    part -= Pos(0, 0, z) * _pin_bore(length=40)
    return part


def _flat_arm(length: float, r0: float, r1: float, t: float = LINK_T) -> Part:
    """A flat link in the x-z plane from the proximal pivot (0,0,0) to the distal
    pivot (length,0,0); pin axis = Y. Rounded hubs at both pivots + a joining web."""
    hub0 = _cyl_y(r0, t)
    hub1 = Pos(length, 0, 0) * _cyl_y(r1, t)
    web = Pos(length / 2, 0, 0) * Box(length, t, min(r0, r1) * 1.7)
    return hub0 + hub1 + web


def crank(sign: int = 1) -> Part:
    """Knee-servo crank (FOURBAR['crank'] mm): mounts on the servo horn at the crank
    pivot (origin) and carries the crank–coupler pin at (crank,0,0).

    It lives in the thigh channel's IN-PLANE lane (y = 0, ``CLEVIS_TONGUE_T`` thick),
    where the forked pushrod straddles it. The horn recess is on the inboard face only,
    so the L and R cranks are the same flat part printed and simply flipped over —
    ``sign`` only tells the MODEL which way round it is fitted."""
    r = P.FOURBAR["crank"]
    t = P.CLEVIS_TONGUE_T
    body = _flat_arm(r, r0=HORN_R + 2.0, r1=PIN_R + 3.0, t=t)
    body -= F.horn_holes(axis="y", length=t + 2)                     # horn bolt circle + centre screw
    body -= Pos(0, -sign * (t / 2 - P.HORN_DISC_T / 2), 0) * _cyl_y(
        HORN_R + P.HORN_SEAT_CLEAR, P.HORN_DISC_T)                   # horn disc seat recess
    body -= Pos(r, 0, 0) * _pin_bore()                               # crank–coupler pin (rotating)
    return body


def pushrod(sign: int = 1) -> Part:
    """Rigid pushrod / coupler (FOURBAR['coupler'] mm), running in its own lane.

    It shares the C pin with the crank and the R pin with the rocker, and both of those
    are in-plane links, so the rod cannot be coplanar with them. It sits one lane
    outboard (``sign * P.FB_ROD_Y``) and both pins are shouldered; bearing counterbores
    at each end take the shoulder so the rod runs on the pin, not on the printed bore.
    Like the crank it is one flat printed part, flipped for the other side."""
    L = P.FOURBAR["coupler"]
    t = P.CLEVIS_ROD_T
    hub = PIN_R + 3.0
    ry = sign * P.FB_ROD_Y
    body = Pos(0, ry, 0) * _flat_arm(L, r0=hub, r1=hub, t=t)
    for x in (0.0, L):
        body -= Pos(x, ry, 0) * _pin_bore()
        for yf in (1, -1):                                           # bearing counterbores
            body -= Pos(x, ry + yf * (t / 2 - 0.6), 0) * _cyl_y(PIN_R + 1.4, 1.4)
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
    # The rocker sits in the SAME in-plane lane as the crank and the shank's knee tongue,
    # inside the thigh channel — so the forked pushrod can straddle its pin.
    arm = _flat_arm(r, r0=LINK_W * 0.6, r1=PIN_R + 3.0, t=P.CLEVIS_TONGUE_T)
    arm -= Pos(r, 0, 0) * _pin_bore()         # coupler–rocker pin at the tip (rotating)
    return Rot(0, -ang, 0) * arm              # swing the +x arm onto the weld direction


def _servo_pack(px: float, pz: float, pivot_y: float, sign: int,
                long_axis: str = "x", wall: float = 3.0, r: float = 5.0,
                off: tuple[float, float] = (0.0, 0.0), shaft_end: int = +1):
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
    fl, ft = SERVO.flange_cut
    hd, _ = SERVO.horn_seat
    dy = h                                          # depth along the shaft (Y)
    dx, dz = (l, w) if long_axis == "x" else (w, l)
    flx, flz = (fl, dz) if long_axis == "x" else (dx, fl)
    dyb = dy + 2 * wall
    cy = pivot_y - sign * dyb / 2.0                 # boss centre: outer face on the joint plane
    # The STS3215's output shaft is NOT centred on its case — it sits 12 mm from one end
    # of a 45 mm body. The BODY therefore has to be offset from the joint axis by
    # ``SERVO.shaft_offset`` or the pocket is 10.5 mm away from where the servo really is.
    # ``shaft_end`` picks which way round the servo is fitted; ``off`` is an extra manual
    # nudge used to stagger two servos that share a limb.
    ecc = SERVO.shaft_offset * shaft_end
    ex, ez = (ecc, 0.0) if long_axis == "x" else (0.0, ecc)
    bx, bz = px + off[0] + ex, pz + off[1] + ez
    bc = Pos(bx, cy, bz)
    boss = bc * _rounded_box(dx + 2 * wall, dyb, dz + 2 * wall, r)
    cut = bc * Box(dx, dy, dz)                                    # servo CASE pocket
    # Mounting-FLANGE relief at the output end: the case is 45 mm long but the tabs span
    # 54 mm. Without this the servo cannot be dropped into its own pocket.
    cut = cut + Pos(bx, pivot_y - sign * (wall + ft / 2.0), bz) * Box(flx, ft, flz)
    # Horn seat: the Ø20 disc is fitted from OUTSIDE after the servo is seated, so the
    # outer wall needs a counterbore for it — a Ø6 shaft hole is not enough.
    hdp = wall + 0.5                                              # break through the wall
    cut = cut + Pos(px, pivot_y - sign * hdp / 2.0, pz) * _cyl_y(hd / 2.0, hdp)
    cut = cut + Pos(px, cy, pz) * _cyl_y(3.0, dyb)               # shaft relief on the joint axis
    # STS3215 case-retention screws (M2), parallel to the shaft, into the flange face.
    cut = cut + bc * F.servo_case_screws("y", (dx, dz), length=dyb + 14)
    return boss, cut


# How the knee servo's housing is laid out. body.py has to carve the torso for the volume
# this boss sweeps, and it used to restate the geometry from SERVO.pocket itself -- so when
# the servo was turned to lie along the thigh, the ribcage relief kept clearing the old
# orientation and the thigh started sharing 262 mm3 with it. One source now.
# The 45 mm length runs FORE-AFT, and it is worth writing down why, because the reason
# the code used to give is obsolete and the right one is not obvious.
#
# The old reason was to stay clear in Z of the HIP servo above it. That servo moved into
# the torso core when the hip went remote-axle (params.HIP_DRIVE), so nothing has been
# above the crank boss for a while.
#
# The real reason is the sweep. The hip carries this housing through the flank, and what
# sets the width of the opening it needs is the housing's FURTHEST radius from the hip
# axis -- the arc at that radius, not the arc at the pivot. Lying fore-aft, the boss
# reaches r = 39 mm and sweeps a 37 mm arc. Turned to lie along the thigh it is narrower
# across the sweep, which looks like the win, and it is not: its far end then sits at
# r = 67 mm and sweeps 63 mm. Measured, that turn made the fore leg port BIGGER --
# 8490 mm3 to 10712, x-span 76 mm to 87.
KNEE_BOSS_LONG_AXIS = "x"      # fore-aft: keeps the housing's far corner close to the hip
KNEE_BOSS_SHAFT_END = +1


def knee_boss_envelope(length: float, sign: int) -> tuple[tuple, tuple]:
    """(centre, (dx, dy, dz)) of the knee-servo housing, in the thigh's own frame."""
    l, w, h = SERVO.pocket
    wall = P.HIP_BOSS_WALL
    dx, dz = ((l, w) if KNEE_BOSS_LONG_AXIS == "x" else (w, l))
    ecc = SERVO.shaft_offset * KNEE_BOSS_SHAFT_END
    ex, ez = (ecc, 0.0) if KNEE_BOSS_LONG_AXIS == "x" else (0.0, ecc)
    dy = h + 2 * wall
    z = -(length - P.FOURBAR["ground"])
    return ((ex, -sign * (P.FB_HORN_Y + dy / 2.0), z + ez),
            (dx + 2 * wall, dy, dz + 2 * wall))


def _crank_pivot_boss(strut: Part, length: float, sign: int) -> Part:
    """Relocate the knee servo UP the thigh: a boss at ``ground`` mm above the knee
    housing the knee servo whose output IS the crank pivot (shaft on Y at the thigh
    centreline). The body is tucked INBOARD via ``_servo_pack`` so the crank boss no
    longer juts ~21 mm past the joint plane.

    Its orientation is set by ``KNEE_BOSS_LONG_AXIS``; see the note there for why it
    lies fore-aft, which is not the reason the code used to give."""
    z = -(length - P.FOURBAR["ground"])
    # shift the crank servo body ~6 mm DOWN the thigh (shaft stays on the crank pivot)
    # so its top clears the hip servo boss centred 29.5 mm above it.
    # NOTE the servo can only be offset along its OWN long axis (``shaft_end`` picks which
    # way), because the shaft is centred across the case's 24 mm width — the old free
    # ``off=(0,-6)`` nudge in Z was not a motion a real servo can make. Its 42 mm depth
    # along the hip axis still reaches inboard past the torso flank, so the torso carries
    # a swept clearance for it (body.py :: _hip_sweep_relief) instead.
    boss, cut = _servo_pack(0.0, z, -sign * P.FB_HORN_Y, sign,
                            long_axis=KNEE_BOSS_LONG_AXIS, shaft_end=KNEE_BOSS_SHAFT_END)
    return strut + boss - cut


def _thigh_channel(strut: Part, length: float, sign: int = 1) -> Part:
    """Open the thigh into a forward-facing CHANNEL.

    This is what makes the four-bar buildable: the crank, the pushrod blades, the rocker
    welded to the shank and the shank's own knee tongue all run INSIDE this one slot
    instead of being drawn through the middle of the thigh solid. The slot only opens
    towards +x (the linkage never swings behind the thigh — the crank window is
    -20°..81°), so the back of the thigh stays a closed spine and keeps its bending
    stiffness."""
    lo, hi = P.clevis_slot(rod=True)
    w = hi - lo
    yc = sign * (lo + hi) / 2.0
    z0 = -(length - P.FOURBAR["ground"])              # crank pivot
    # the slot has to clear the crank over its WHOLE window, not just the stance pose:
    # at the top of the window the crank tip swings up to nearly the hip.
    _, hi_deg = P.FOURBAR["crank_window"]
    reach = P.FOURBAR["crank"] * math.sin(math.radians(hi_deg)) + PIN_R + 3.0 + 2.0
    top = min(0.0, z0 + reach)
    bot = -length - 14.0
    strut -= Pos(24.0, yc, (top + bot) / 2.0) * Box(48.0, w, top - bot)
    # the crank hub sweeps a full disc about the crank pivot — clear it out of the spine
    strut -= Pos(0, yc, z0) * _cyl_y(HORN_R + 3.0, w)
    return strut


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


def bracket_mount_screw_count() -> int:
    """How many screws hold one leg bracket to the torso. The leg's ONLY attachment to
    the body — a bracket without these has no defined way of being fitted."""
    return P.MOUNT_SCREWS


def servo_body_offset() -> float:
    """Distance the servo BODY must be offset from the joint axis, because the STS3215's
    output shaft is not centred on its case."""
    return SERVO.shaft_offset


def servo_cavity(long_axis: str = "x", sign: int = 1, wall: float = 3.0) -> Part:
    """The complete cavity one servo needs — case pocket, mounting-flange relief, horn
    counterbore, shaft relief and case screws — at the origin. Exposed so a test can
    check the servo actually fits what the CAD cuts for it."""
    _, cut = _servo_pack(0.0, 0.0, 0.0, sign, long_axis=long_axis, wall=wall)
    return cut


def hip_bracket(sign: int = 1, hip_off: float = 28.0, x_shift: float = 0.0) -> Part:
    """Slim shoulder BEARING BLOCK, bolted to the torso flank (remote-axle hip drive).

    The hip/shoulder servo BODY is in the torso core (body.py ``_core_hip_drive``). This
    part BOLTS to the flat mount pad on the torso flank at |y| = BODY_W/2 with
    ``P.MOUNT_SCREWS`` screws — that is the leg's whole attachment to the body — and, part
    way out along the hip axis, carries the OUTBOARD drive-axle bearing (#2). The lateral
    axle spins in that bearing and couples to the upper leg at the hip pivot.

    The bearing is placed far enough inboard of the hip pivot to clear the thigh channel's
    full width, so the bracket and the thigh never occupy the same space.

    ``x_shift`` is retained for call-signature compatibility (there is no servo body out
    here to slide fore-aft any more)."""
    hb = P.HIP_BEARING
    wall = P.HIP_BOSS_WALL
    yj = sign * hip_off                     # the hip pivot, in this part's frame
    blen = hb["width"] + 6
    # bearing station: inboard of the pivot far enough to clear the thigh's inboard cheek
    # AND the axle flange that seats against it
    lo, _ = P.clevis_slot(rod=True)
    clear = -(lo - P.CLEVIS_CHEEK_T) + 3.0 + 1.5
    yb = yj - sign * (clear + blen / 2.0)

    part = Pos(0, yb, 0) * _cyl_y(hb["od_r"] + wall, blen)      # outboard drive-axle bearing
    # bolted mounting foot, seated flat on the torso's flank pad
    pad_x, pad_z = P.MOUNT_PAD
    foot_t = 4.0
    yf = sign * foot_t / 2.0
    part += Pos(0, yf, 0) * Box(pad_x, foot_t, pad_z)
    # slim web bridging foot -> bearing (kept narrow in X/Z so the skin can hug the body)
    y0, y1 = sign * foot_t, yb
    part += Pos(0, (y0 + y1) / 2.0, 0) * Box(2 * (hb["od_r"] + wall), abs(y1 - y0),
                                             2 * hb["od_r"])
    # axle channel straight through the whole block on the Y hip axis
    part -= _cyl_y(hb["bore_r"] + P.AXLE_CLEAR, 3 * abs(yj) + 40)
    # bearing seat (wider than the axle bore), recessed from the inboard face of the boss
    part -= Pos(0, yb, 0) * _cyl_y(hb["od_r"], hb["width"])
    part -= Pos(0, yb, 0) * F.pin_head_seat("y", face=-sign * (blen / 2.0))
    # mount screws: clearance through the foot, into heat-set inserts in the torso pad
    for i in range(P.MOUNT_SCREWS):
        sx = (i - (P.MOUNT_SCREWS - 1) / 2.0) * P.MOUNT_BOLT_PITCH
        part -= Pos(sx, yf, 0) * F.screw_clearance(P.MOUNT_SCREW, "y", foot_t + 8)
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
    lo, _ = P.clevis_slot(rod=True)
    # the flange seats against the thigh's INBOARD cheek face, not on the hip pivot plane
    L = (P.BODY_W / 2 + hip_off) - P.HIP_CORE_HORN_Y + (lo - P.CLEVIS_CHEEK_T) - 1.5
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


# Clevis pivot radii. The FORK half is the larger of each pair so its cheeks fully
# enclose the mating tongue's hub through the joint's whole range of motion.
KNEE_FORK_R = 8.5
KNEE_TONGUE_R = 7.5
ANKLE_FORK_R = 7.5
ANKLE_TONGUE_R = 6.5
KNEE_BLADE_L = 30.0     # how far the shank stays a blade below the knee


def _hip_horn_mount(strut: Part, sign: int = 1) -> Part:
    """Coupling to the hip DRIVE AXLE, on the thigh's INBOARD CHEEK.

    The axle arrives from the torso, so it has to land on the inside face of the thigh
    channel — not on the hip pivot plane, which is the thigh's own mid-plane and is
    buried inside the part (that is where the old pad was, inside the bracket's bearing).
    A Ø20 bolt-circle pad on the cheek takes the axle's outboard flange; the thigh then
    hangs off the axle, which runs in the bracket's two bearings."""
    pad_r = P.HORN_BOLT_CIRCLE / 2 + P.HEATSET[P.HORN_SCREW]["boss_r"]
    lo, _ = P.clevis_slot(rod=True)
    t = P.CLEVIS_CHEEK_T
    yc = sign * (lo - t / 2.0)                        # mid-plane of the inboard cheek
    strut = strut + Pos(0, yc, 0) * _cyl_y(pad_r, t)
    strut -= Pos(0, yc, 0) * F.horn_holes(axis="y", length=t - 0.2)
    return strut


def hip_pad_y(sign: int = 1) -> float:
    """Inboard face of the thigh where the drive axle's flange lands (leg-local Y)."""
    lo, _ = P.clevis_slot(rod=True)
    return sign * (lo - P.CLEVIS_CHEEK_T)


# ------------------------------------------------------------------ fairing mounting
# ``_bone_strut`` pinches its middle by ``waist``, and its station table gives the exact
# half-widths at t = 0.18 and 0.82. Putting the fairing screws there means both this
# module (which cuts the inserts) and fairing.py (which cuts the matching clearance holes)
# can place them from NUMBERS, without either building the other's solid -- asking the
# built segment would recurse, since leg_parts() is what would be calling in.
STRUT_WAIST = 0.86
# Fractions of segment length, from the proximal joint, with the y half-width the strut
# table gives there. The shank's stations are BOTH below 0.6: its top ``KNEE_BLADE_L`` =
# 30 mm is squeezed to a 7 mm blade so it can drop into the thigh's channel, so the
# obvious 0.18 station put the bore at y = 7.05 where the material only reaches 3.5 --
# a heat-set insert floating in air.
FAIRING_MOUNT_T = {"upper": (0.18, 0.82), "lower": (0.68, 0.88)}
_STRUT_RY = {"lower": (6.19, 6.19)}     # 7.2 * STRUT_WAIST, the waisted mid-span half-width


def fairing_mounts(length: float, seg: str, sign: int = 1) -> list[tuple[float, float]]:
    """[(z, y), ...] in the segment's own frame where its fairing bolts on.

    Takes the segment LENGTH rather than a leg name: ``upper_leg``/``lower_leg`` are built
    per length and do not know whether they are a front or a rear leg, and handing them
    the wrong one would put the front leg's stations on the rear.

    ``y`` is the OUTBOARD face: the screw comes in from the visible side, through the
    fairing wall and into an insert in the limb. The thigh's servo pack is inboard of the
    strut, so a bore entering outboard meets the strut first even where it passes the
    bulge.
    """
    if seg == "upper":
        half, yoff = P.thigh_profile()
        ry, centre, waist = (half, half), yoff, STRUT_WAIST
    else:
        ry, centre, waist = _STRUT_RY["lower"], 0.0, 1.0
    return [(-t * length, sign * (centre + r * waist))
            for t, r in zip(FAIRING_MOUNT_T[seg], ry)]


def _fairing_inserts(part: Part, length: float, seg: str, sign: int = 1) -> Part:
    """Blind heat-set bores in the limb, for the fairing that clips over it."""
    depth = P.HEATSET[P.FAIRING_SCREW]["depth"] + 1.0
    for z, y in fairing_mounts(length, seg, sign):
        # bore inward from the outboard face, running toward the centreline
        off = y - (1.0 if y >= 0 else -1.0) * depth / 2.0
        part -= Pos(0, off, z) * F.heatset_hole(P.FAIRING_SCREW, "y", depth)
    return part


def upper_leg(length: float, sign: int = 1) -> Part:
    """Thigh — a forward-opening CHANNEL that houses the whole four-bar.

    The knee servo sits at the crank pivot up the thigh (the knee itself is a passive
    pin) and the thigh forks at the knee to take the shank's tongue. ``sign`` (leg side)
    tucks the crank servo body inboard, so the L/R thighs are mirror images — print one
    flipped."""
    half, yoff = P.thigh_profile()
    strut = Pos(0, sign * yoff, 0) * _segment(length, knee_servo=False,
                                              rp=(7.6, half), rd=(6.2, half), lighten=False)
    strut = _thigh_channel(strut, length, sign)
    strut = _crank_pivot_boss(strut, length, sign)
    strut = _fork_seat(strut, -length, r=KNEE_FORK_R, open_dir=-1)   # knee fork takes the shank
    strut = _hip_horn_mount(strut, sign)               # driven out from the hip drive axle
    return _fairing_inserts(strut, length, "upper", sign)


def lower_leg(length: float, stance_knee: float) -> Part:
    """Shank — a TONGUE at the knee (into the thigh's fork) and a FORK at the ankle.

    In the four-bar realisation the only knee servo lives on the thigh crank boss and the
    knee/ankle are passive pins, so the shank carries no servo pocket. The rocker is
    welded on at the knee, in the same in-plane lane as the tongue, so the whole shank
    top is one 7 mm blade that drops into the thigh channel."""
    strut = _segment(length, knee_servo=False, rp=(7.2, 8.2), rd=(5.8, 7.2), lighten=True)
    strut = strut + _rocker_arm(stance_knee)          # rocker welded on the shank at the knee
    # the blade has to stay narrow for as long as the pushrod runs beside it in the channel
    strut = _tongue_seat(strut, 0.0, r=KNEE_TONGUE_R, open_dir=+1, reach=KNEE_BLADE_L)
    strut = _fork_seat(strut, -length, r=ANKLE_FORK_R, open_dir=-1)  # ankle fork takes the foot
    return _fairing_inserts(strut, length, "lower")


def foot_seg(length: float) -> Part:
    """Metatarsus + shaped paw pad. Proximal (ankle) at origin, toe at -z. The
    ankle (foot↔lower-leg join) is a passive Ø3 pin: a bearing seat at the origin
    mates the lower-leg's distal ankle seat."""
    strut = _bone_strut(length, rp=(6.0, 7.0), rd=(4.6, 5.6))
    strut = _tongue_seat(strut, 0.0, r=ANKLE_TONGUE_R, open_dir=+1)   # ankle tongue
    # flattened, slightly forward paw pad in place of a bare sphere
    pad = scale(Sphere(P.TOE_R), (1.28, 1.06, 0.82))
    paw = Pos(2.5, 0, -length) * pad
    return strut + paw


_LEG_PARTS_CACHE: dict[str, dict] = {}


def leg_parts(leg: str) -> dict[str, Part]:
    """Every printed piece of one leg, built once per leg and cached.

    It is called from the printable table, the fairings, the assembly and the tests, and
    each caller usually wants a single segment. Rebuilding six solids to hand back one was
    cheap enough to ignore until the fairings started asking; the parts are immutable in
    practice (build123d booleans return new objects), so the cache is safe to share.
    """
    hit = _LEG_PARTS_CACHE.get(leg)
    if hit is not None:
        return hit
    from sim.gait import stance_angles
    g = P.leg_geom(leg)
    s = P.leg_plane_sign(leg)
    _, knee = stance_angles(leg)                      # weld the rocker for this leg's stance
    # slide the fore-aft hip servo body toward the waist (front legs: -x, rear: +x) so
    # it stays inside the torso length rather than poking past the nose / tail.
    x_shift = -16.0 if P.is_front(leg) else 16.0
    out = {
        "hip_bracket": hip_bracket(s, g["hip_off"], x_shift),
        "axle": hip_axle(g["hip_off"]),
        "upper": upper_leg(g["upper"], s),
        "crank": crank(s),
        "pushrod": pushrod(s),
        "lower": lower_leg(g["lower"], knee),
        "foot": foot_seg(g["foot"]),
    }
    _LEG_PARTS_CACHE[leg] = out
    return out


if __name__ == "__main__":
    for leg in ("FL", "RL"):
        print(leg, {n: round(p.volume, 1) for n, p in leg_parts(leg).items()})
    print("crank vol", round(crank().volume, 1), "pushrod vol", round(pushrod().volume, 1))
