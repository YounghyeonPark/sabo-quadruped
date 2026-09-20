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

import math

from build123d import (Box, BuildPart, BuildSketch, Cylinder, Ellipse, Locations,
                       Mode, Part, Plane, Pos, Rectangle, Rot, Sphere, extrude, loft)

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


def _ab_at(stations, x: float) -> tuple[float, float]:
    """The ribcage's (half-width, half-height) at station ``x`` — used to size a brace so
    it reaches the shell exactly, without poking through it."""
    o = sorted(stations)
    x = max(o[0][0], min(o[-1][0], x))
    for (x0, a0, b0), (x1, a1, b1) in zip(o, o[1:]):
        if x0 <= x <= x1:
            t = 0.0 if x1 == x0 else (x - x0) / (x1 - x0)
            return (a0 + (a1 - a0) * t, b0 + (b1 - b0) * t)
    return (o[-1][1], o[-1][2])


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


# How far either side of its stance angle the hip actually works. The joint LIMIT is
# +-2.6 rad, but nothing drives it there, and clearing the full limit would mean carving
# a Ø94 disc out of the ribcage at every hip -- which is exactly what an earlier version
# of this function did, and it cut the ribcage into loose pieces.
#
# This used to be a hand-set +-0.65 rad carrying the claim that "the gaits and the
# cat-motion poses stay inside this". They did not. The SIT pose folds the REAR hip 59.6
# deg back, well past 37.2, and the scallop cut for the smaller number left the rear
# thigh sharing 238 mm3 of solid with the aft ribcage -- a pose in the demo sequence that
# the model cannot actually strike. Nothing caught it because the interference suite only
# ever posed the legs at STANCE.
#
# A constant cannot track a motion library that keeps growing, so the range is READ from
# that library: every gait preset, every gesture, and every pounce keyframe (plus the
# eased interpolation between them) is run through the same IK the emulator uses. The
# window is per-leg and ASYMMETRIC because the motion is -- the front hip works
# -23/+19 deg and the rear -60/+15 -- so deriving it both fixes the rear and hands the
# front ribcage back the material a symmetric +-37 deg was needlessly cutting away.
HIP_SWEEP_STEPS = 9
HIP_SWEEP_MARGIN = math.radians(6.0)   # headroom past the furthest commanded pose


def hip_work_range(leg: str) -> tuple[float, float]:
    """(lo, hi) rad about stance that the hip is really driven through, + a margin."""
    cached = _HIP_RANGE_CACHE.get(leg)
    if cached is not None:
        return cached

    from sim import cute_motion as CM
    from sim.gait import (PRESETS, foot_target, leg_depth, leg_ik, stance_angles)

    hip0 = stance_angles(leg)[0]
    d0 = leg_depth(leg)
    key = "front" if leg[0] == "F" else "rear"
    lo = hi = 0.0

    def note(tgt):
        nonlocal lo, hi
        try:
            v = leg_ik(leg, tgt.get(f"{key}_tuck", 0.0),
                       d0 * tgt.get(f"{key}_depth", 1.0))[0] - hip0
        except Exception:      # a keyframe the IK cannot reach is not a pose we can hold
            return
        lo, hi = min(lo, v), max(hi, v)

    for fn, _dur in CM.GESTURES.values():
        for i in range(61):
            note(fn(i / 60.0))

    jc = CM.JumpController()
    keys = [jc._stand(), jc._crouch(), jc._extend(), jc._reach(), jc._absorb()]
    for a, b in zip(keys, keys[1:] + keys[:1]):
        for i in range(21):
            note(CM._lerp_targets(a, b, i / 20.0))

    for preset in PRESETS.values():
        for i in range(72):
            try:
                v = leg_ik(leg, *foot_target(leg, i / 72.0, preset, d0))[0] - hip0
            except Exception:
                continue
            lo, hi = min(lo, v), max(hi, v)

    out = (lo - HIP_SWEEP_MARGIN, hi + HIP_SWEEP_MARGIN)
    _HIP_RANGE_CACHE[leg] = out
    return out


_HIP_RANGE_CACHE: dict[str, tuple[float, float]] = {}


def _hip_sweep_relief(mx: float, s: int, leg: str) -> Part:
    """Swept clearance for the thigh's knee-servo boss as the hip rotates.

    The knee servo's shaft has to lie on the hip-parallel crank axis, and an STS3215 is
    36 mm deep along its shaft, so its housing unavoidably reaches inboard past the torso
    flank. The torso therefore carries the scallop the boss sweeps -- the same thing a real
    quadruped does at the shoulder.

    It is the boss's REAL swept volume, sampled over the hip's working range. A disc sized
    to the boss's furthest corner is far bigger, and severs the ribs and stringers it
    crosses; the boss only ever occupies a curved sliver of it.
    """
    from cad.parts.leg import _rounded_box
    from sim.gait import stance_angles

    # ASK the leg for the boss rather than restating it. This block used to rebuild the
    # housing from SERVO.pocket, and when the knee servo was turned to lie along the thigh
    # the relief kept clearing the old orientation -- the thigh then shared 262 mm3 with
    # the ribcage, in a part the interference suite had just passed.
    from cad.parts.leg import knee_boss_envelope

    (cx, cy, cz), (dx, dy, dz) = knee_boss_envelope(P.leg_geom(leg)["upper"], s)
    boss = Pos(cx, cy, cz) * _rounded_box(dx + 2.0, dy + 2.0, dz + 2.0, 5.0)

    hip0, _ = stance_angles(leg)
    lo, hi = hip_work_range(leg)
    void = None
    for i in range(HIP_SWEEP_STEPS):
        t = i / (HIP_SWEEP_STEPS - 1)
        ang = hip0 + lo + t * (hi - lo)
        posed = Rot(0, -math.degrees(ang), 0) * boss
        void = posed if void is None else void + posed
    # The thigh hangs off the MOUNT, at |y| = BODY_W/2, and the hip is hip_off further out
    # again — the scallop has to be cut where the boss actually is, not hip_off from the
    # centreline.
    _, my = P.MOUNTS[leg]
    return Pos(mx, my + s * P.leg_geom(leg)["hip_off"], 0) * void


def _head_socket() -> Part:
    """Cavity in the chest for the head, which pans and nods on the neck gimbal.

    Cut as a ball around the head's centre, NOT as the head's true swept volume.

    That is a deliberate approximation and it has a cost worth writing down. The yaw axis
    sits 18 mm behind the head's centre — it had to, so the pan linkage would have a plane
    to run in under the ball — so as the head turns its centre orbits, and its real sweep
    is a blob of radius ~68 about the yaw axis, not a Ø104 ball. Cutting that swept blob is
    correct and it hollows out essentially the whole front of the chest: measured, it
    leaves the ribcage in 41 loose pieces. A Ø100 head this close to a yaw axis cannot turn
    ±57° without sweeping the chest.

    Resolving it means giving ground somewhere — a smaller pan range, a smaller head, or a
    yaw axis back at the head's centre with the linkage re-routed. Until then this cuts the
    ball, which is right for the head's rear (the part that shares space with the chest)
    and wrong for its swing."""
    hx, _, hz = P.head_centre()
    r = P.HEAD_R + 2.0
    return Pos(hx, 0, hz) * (Sphere(r)
                             + Pos(0, 0, -P.NECK_L) * Cylinder(16.0, 2 * P.NECK_L))


def _leg_mount_relief(mx: float, s: int) -> Part:
    """Clearance outboard of the seating plane over the bracket's footprint.

    The rib hoops bulge a little past |y| = BODY_W/2, so without this the bolted-on leg
    bracket would still be inside the ribcage. Cutting it here means the mount really is
    a flat face that another part lands on."""
    pad_x, pad_z = P.MOUNT_PAD
    yf = s * (P.BODY_W / 2.0)
    return Pos(mx, yf + s * 20.0, 0) * Box(pad_x + 8.0, 40.0, pad_z + 8.0)


def _leg_mount_pad(mx: float, s: int) -> Part:
    """Flat, bolted landing for one leg's hip bracket on the torso flank.

    Without this the leg had NO defined attachment to the body — the bracket was simply
    drawn overlapping the ribcage. The pad grows INBOARD from the flank plane
    (|y| = BODY_W/2) so it never pushes the leg outboard, and carries the heat-set
    inserts the bracket's screws pull into."""
    pad_x, pad_z = P.MOUNT_PAD
    t = P.MOUNT_PAD_T
    yf = s * (P.BODY_W / 2.0)                       # the flank / seating plane
    pad = Pos(mx, yf - s * t / 2.0, 0) * Box(pad_x, t, pad_z)
    ins = P.HEATSET[P.MOUNT_SCREW]
    for i in range(P.MOUNT_SCREWS):
        sx = mx + (i - (P.MOUNT_SCREWS - 1) / 2.0) * P.MOUNT_BOLT_PITCH
        pad -= Pos(sx, yf - s * (ins["depth"] / 2.0), 0) *             F.heatset_hole(P.MOUNT_SCREW, "y", ins["depth"])
    return pad


def _half(stations, legs) -> Part:
    ribs = _interp(stations, extra=1)
    frame = _rib(*ribs[0])
    for r in ribs[1:]:
        frame += _rib(*r)
    # Eight stringers: spine (top), keel (bottom), two flanks, and four diagonals.
    #
    # The four cardinal ones are not enough once the cage has clearance cut into it. The
    # head socket takes the SPINE out over the chest and the hip scallops take the FLANK
    # out beside the shoulders, which between them leave a rib's upper arc attached to
    # nothing. The diagonals run where neither cut reaches (|y| just inside the scallop,
    # z well outside the socket) and tie those arcs back in.
    d = 0.70
    for oy, oz in ((0, 1), (0, -1), (1, 0), (-1, 0),
                   (d, d), (d, -d), (-d, d), (-d, -d)):
        frame += _stringer(stations, oy, oz)
    # core-mounted hip servos + drive-axle wall bearings (remote-axle hip drive): the
    # ~45 mm servo body no longer floors at the hip joint line out in the shoulder; it
    # lives here in the core near the centreline, driving a lateral axle out to the hip.
    for leg in legs:
        mx, my = P.MOUNTS[leg]
        s = P.leg_plane_sign(leg)
        frame += _core_hip_drive(mx, s)
        frame -= _leg_mount_relief(mx, s)
        frame += _leg_mount_pad(mx, s)
    return frame


def _hip_scallops(frame: Part, legs) -> Part:
    """Cut the knee-servo clearance out of a FINISHED half.

    It has to be the last thing done to the half. Everything that gets added afterwards —
    the mount pads on the flank, the yaw servo's boss (which the STS3215's off-centre shaft
    pushes 10 mm sideways, straight into the right thigh's path) — sits inside the band the
    boss sweeps, so a scallop cut any earlier is simply filled back in."""
    for leg in legs:
        mx, _my = P.MOUNTS[leg]
        frame -= _hip_sweep_relief(mx, P.leg_plane_sign(leg), leg)
    return frame


def _tail_drive(frame: Part) -> Part:
    """Tail actuator + joint, in the AFT half (params.TAIL_DRIVE = 'remote_crank').

    The tail pivot is at the rear extremity of the frame, where nothing can be housed, so
    the servo lives forward at ``P.TAIL_SERVO`` — the one aft station where
    ``analysis.actuator_fit`` finds room — and drives the joint through a four-bar. This
    adds three things: the servo housing, the TONGUE the tail's fork pivots on, and the
    corridor the pushrod needs through the ribcage.
    """
    from cad.parts.leg import PIN_R, _cyl_y, _pin_bore, _servo_pack
    from cad.parts.tail import tail_linkage

    horn_y, link_y, rod_y = P.tail_lane_y()
    sx, sz = P.TAIL_SERVO
    tx, tz = -P.AFT_LEN, P.BODY_H / 4

    # 1. the actuator: shaft on the tail axis (Y) at the crank pivot, case reaching forward
    boss, cut = _servo_pack(sx, sz, horn_y, +1, long_axis="x", shaft_end=+1)
    frame = frame + boss

    # 2. the pushrod corridor. The rod swings between the crank tip and the rocker tip, so
    #    it crosses the ribcage; the lattice gets a slot for it rather than the rod being
    #    drawn through solid ribs.
    _, pts = tail_linkage()
    cx, cz = pts["C"]
    rx, rz = pts["R"]
    x0, x1 = min(cx, rx) - P.TAIL_FOURBAR["crank"], max(cx, rx) + 8.0
    z0, z1 = min(cz, rz) - P.TAIL_FOURBAR["crank"], max(cz, rz) + 8.0
    slot_w = P.CLEVIS_ROD_T + 2 * P.CLEVIS_GAP + 1.0
    frame -= Pos((x0 + x1) / 2, rod_y, (z0 + z1) / 2) * Box(x1 - x0, slot_w, z1 - z0)
    #    ...and for the crank, which sweeps a full disc about the servo shaft
    crank_w = P.CLEVIS_TONGUE_T + 2 * P.CLEVIS_GAP
    frame -= Pos(sx, link_y, sz) * _cyl_y(
        P.TAIL_FOURBAR["crank"] + PIN_R + 3.0 + P.CLEVIS_GAP, crank_w)

    # 3. swing clearance. The tail's fork cheeks and its rocker rotate about the pivot, and
    #    at the pivot the ribcage is still there — so the lanes either side of the tongue
    #    are cleared over the radius those parts sweep. The tongue's own lane is untouched.
    sweep_r = P.TAIL_FOURBAR["rocker"] + PIN_R + 3.0 + 2.0
    lo, hi = P.clevis_slot()
    _, link_y, _ = P.tail_lane_y()
    cheek = P.CLEVIS_CHEEK_T + P.CLEVIS_GAP
    for y0, y1 in ((lo - cheek, lo), (hi, link_y + P.CLEVIS_TONGUE_T / 2 + P.CLEVIS_GAP)):
        frame -= Pos(tx, (y0 + y1) / 2, tz) * _cyl_y(sweep_r, y1 - y0)

    # 4. the joint itself: a TONGUE at the tail pivot for the tail's fork to straddle,
    #    carried on a short stem off the last rib.
    pivot_r = 8.0
    frame += Pos(tx + 6.0, 0, tz) * Box(14.0, P.CLEVIS_TONGUE_T, 2 * pivot_r)
    frame += Pos(tx, 0, tz) * _cyl_y(pivot_r, P.CLEVIS_TONGUE_T)
    frame -= Pos(tx, 0, tz) * _pin_bore(length=40)

    return frame - cut


def _pan_drive(frame: Part) -> Part:
    """The head's YAW actuator and bearing, in the fore torso (params.HEAD_DRIVE).

    The gimbal's motors cannot both sit on their own axes inside the head
    (``analysis.actuator_fit.gimbal_layout``), and the head is the last link in the chain,
    so the one that has to move out goes here — the only cavity upstream of the yaw joint.
    It reaches the axis through a four-bar in a horizontal plane that passes UNDER the head
    ball, which is why the yaw axis is pulled back from the head centre and the head sits
    8 mm proud of the shoulder line.
    """
    from cad.parts.leg import PIN_R, _servo_pack
    from cad.parts.neck import neck_sweep, pan_sweep

    sx, sz = P.PAN_SERVO
    ox = P.HEAD_MOUNT_X
    link_z = P.PAN_BEARING_Z + P.PAN_LINK_Z

    # 1. the actuator: shaft on the VERTICAL yaw-parallel axis, case reaching upward
    boss, cut = _servo_pack(0.0, 0.0, 0.0, -1, long_axis="z")
    place = Pos(sx, 0, sz) * Rot(90, 0, 0)
    frame = frame + place * boss
    # ...and tie it into the cage. The ribcage is a hollow lattice, so a boss sitting in
    # the middle of it touches nothing and prints as a loose second piece; these braces
    # reach from the boss out to the flanks and down to the keel at the same station.
    a, b = _ab_at(FORE_STATIONS, sx)
    brace = 6.0
    for sgn in (+1, -1):
        frame += Pos(sx, sgn * a / 2, sz + 10.0) * Box(brace, a, brace)
    frame += Pos(sx, 0, (sz - b) / 2) * Box(brace, brace, b + sz)

    # 2. the yaw bearing: the neck column's Dia6 axle runs in a 686 seated here
    hb = P.HIP_BEARING
    wall = P.HIP_BOSS_WALL
    frame += Pos(ox, 0, P.PAN_BEARING_Z) * Cylinder(hb["od_r"] + wall, hb["width"] + 8)
    # ...on a cross-brace out to the flanks. The keel stringer is cut away just here by the
    # front hips' sweep relief, so a bearing boss resting on it alone comes off as a loose
    # piece; this ties it sideways into the ribs instead.
    # The keel stringer carries it. That is only true now the hip scallops are cut where
    # the knee-servo boss actually is (|y| >= 33) instead of across the centreline — an
    # earlier version put them 47 mm too far inboard, took the keel out from under this
    # boss, and needed diagonal braces that then ran straight through the thigh's path.
    frame -= Pos(ox, 0, P.PAN_BEARING_Z) * Cylinder(hb["od_r"], hb["width"])
    frame -= Pos(ox, 0, P.PAN_BEARING_Z) * Cylinder(hb["bore_r"] + P.AXLE_CLEAR, 60.0)

    # 3. the cavity the linkage moves through — its REAL swept volume, not a bounding box.
    #    A box round the crank's disc plus a corridor to the neck cut the ribcage into
    #    fourteen loose pieces; the links only ever occupy a thin curved region inside it.
    frame -= pan_sweep()

    # 4. clearance for the column itself as it swings about the yaw axis — again its REAL
    #    swept volume. A disc sized to the column's forward lean is much bigger, and cuts
    #    straight through the braces that hold the yaw bearing.
    frame -= neck_sweep()

    return frame - place * cut


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


# The waist servo's shaft must lie ON the waist axis (Y through the origin), which means
# its 45x24 case straddles the waist plane — a 45 mm case with the shaft 12 mm from one
# end cannot sit wholly on one side of its own axis. So the servo is bolted into the AFT
# half and the FORE half is relieved around it. Its output face is one end of the case,
# 18 mm off the centreline, so the drive is one-sided: the horn pad sits there and a plain
# idler pin on the opposite side carries the other half of the joint.
_WAIST_DRIVE_S = +1        # which side of the centreline the servo's output face is on
_WAIST_IDLER_R = 4.0       # idler pivot pin radius (Ø8 stub, opposite the horn)


def _waist_shaft_y() -> float:
    """|y| of the waist servo's output face — where the horn, and so the fore half's
    coupling pad, has to be."""
    return SERVO.pocket[2] / 2.0


def _waist_aft_features(frame: Part) -> Part:
    """AFT half: the waist servo, shaft ON the waist axis, plus the idler pin boss on the
    far side and the daisy-chain crossing hole."""
    l, w, h = SERVO.pocket
    fl, ft = SERVO.flange_cut
    sy = _WAIST_DRIVE_S
    cx = -(_WAIST_BH_T / 2 + 2)
    frame += _waist_bulkhead(cx)
    # servo boss straddling the waist plane, its shaft on the axis (x=0, z=0)
    frame += Pos(0, 0, -(l / 2 - SERVO.shaft_from_end)) * Box(w + 6, h + 6, l + 6)
    frame -= Pos(0, 0, -(l / 2 - SERVO.shaft_from_end)) * Box(w, h, l)      # case pocket
    frame -= Pos(0, sy * (h / 2 - ft / 2), 0) * Box(w, ft, fl)              # flange relief
    frame -= Pos(0, 0, -(l / 2 - SERVO.shaft_from_end)) *         F.servo_case_screws("y", (w, l), length=h + 14)
    # Horn window: the Ø20 disc is fitted from outside and the FORE half's coupling pad
    # bolts onto it, so the boss wall at the output face has to open to the pad's
    # diameter — a Ø6 shaft hole leaves the pad inside this boss's solid.
    pad_r = P.HORN_BOLT_CIRCLE / 2 + P.HEATSET[P.HORN_SCREW]["boss_r"]
    frame -= Pos(0, sy * (h / 2 + 12.0), 0) * (Rot(90, 0, 0) * Cylinder(pad_r + 1.0, 26.0))
    frame -= F.wire_hole("bus", "y", h + 20)                                # shaft relief on the axis
    # idler pivot on the opposite side of the centreline, on the same axis
    frame += Pos(0, -sy * (_waist_shaft_y() - 4.0), 0) * (Rot(90, 0, 0) * Cylinder(_WAIST_IDLER_R + 3.5, 8.0))
    frame -= Pos(0, -sy * _waist_shaft_y(), 0) * (Rot(90, 0, 0) * Cylinder(_WAIST_IDLER_R + P.PIN_CLEARANCE, 40.0))
    frame -= Pos(0, 12, 8) * F.wire_hole("bus", "x", 60)                    # daisy-chain crosses the waist
    return frame


def _waist_fore_features(frame: Part) -> Part:
    """FORE half: the horn coupling pad out at the servo's output face, the matching idler
    stub on the far side, and a relief cavity around the servo body — which necessarily
    reaches across the waist plane."""
    l, w, h = SERVO.pocket
    sy = _WAIST_DRIVE_S
    pad_r = P.HORN_BOLT_CIRCLE / 2 + P.HEATSET[P.HORN_SCREW]["boss_r"]
    cx = _WAIST_BH_T / 2 + 2
    frame += _waist_bulkhead(cx)
    # clear the aft half's servo boss + its swing through the waist range
    frame -= Pos(0, 0, -(l / 2 - SERVO.shaft_from_end)) * Box(w + 12, h + 12, l + 12)
    # Horn coupling pad, on the waist axis at the servo's output face. It is made long
    # enough to reach OUTBOARD past the servo relief cut above, because that cut takes the
    # middle of this half's bulkhead with it -- a pad that stops at the cut line is a loose
    # disc floating in the waist.
    pad_t = _WAIST_BH_T + 8.0
    ppos = Pos(0, sy * (_waist_shaft_y() + pad_t / 2), 0)
    frame += ppos * (Rot(90, 0, 0) * Cylinder(pad_r, pad_t))
    frame -= ppos * F.horn_holes(axis="y", length=pad_t + 4)
    # idler stub, likewise carried out to material the relief did not take
    stub_t = 20.0
    frame += Pos(0, -sy * (_waist_shaft_y() + stub_t / 2 - 6.0), 0) *         (Rot(90, 0, 0) * Cylinder(_WAIST_IDLER_R, stub_t))
    frame -= Pos(0, 12, 8) * F.wire_hole("bus", "x", 60)
    return frame


def _waist_gap(frame: Part) -> Part:
    """Hold the half off the waist plane. The two halves ROTATE against each other, so
    their innermost ribs cannot both sit on x = 0 — as drawn they shared 3.5 cm³ of
    solid."""
    return frame - Box(2 * P.WAIST_CLEAR, 400.0, 400.0)


def torso_fore() -> Part:
    # The head socket is cut LAST. Everything the gimbal adds — the yaw servo boss, the
    # bearing and its braces — goes in first and is then trimmed back out of the head's
    # space; cutting the socket first just let those additions grow back into it.
    frame = _waist_fore_features(_pan_drive(
        _waist_gap(_half(FORE_STATIONS, ["FL", "FR"]))))
    return _hip_scallops(frame, ["FL", "FR"]) - _head_socket()


def torso_aft() -> Part:
    return _hip_scallops(
        _waist_aft_features(_tail_drive(_waist_gap(_half(AFT_STATIONS, ["RL", "RR"])))),
        ["RL", "RR"])


if __name__ == "__main__":
    print("fore ribcage vol mm^3:", round(torso_fore().volume, 1))
    print("aft  ribcage vol mm^3:", round(torso_aft().volume, 1))
