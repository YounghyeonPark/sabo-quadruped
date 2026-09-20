"""
Cosmetic cat shell — the organic outer *skin* (PLAN §3.3, hardware §8).
=======================================================================

The functional frame (`body.py`, `leg.py`) is boxy on purpose — it's the
skeleton. This module sculpts the **cat-shaped outer shell** that mounts over it:
a smooth lofted body (fuller chest + haunches, narrow waist), a baby-schema head
(big rounded skull, chubby cheeks, small muzzle, big low eyes), and cat ears.
Split at the waist (fore/aft) so the spine joint still flexes.

The shell is now a **printable skin** of wall thickness ``P.SHELL_T`` rather than
a solid blob: each piece is ``outer - inner`` where the inner cavity is a copy of
the lofted / ellipsoid geometry inset by one wall thickness. The hollow interior
houses the edge-AI hardware (`docs/edge_ai_hardware.md` §8), reached through:

  * torso_fore  — a Jetson+carrier access bay on the back + a front fan vent
  * torso_aft   — a battery (3S LiPo) bay in the belly
  * head        — anatomical sensor mounts: a stereo camera aperture behind each
                  eye, a MEMS mic port at each ear base, an e-nose intake grille at
                  the nose, a speaker grille at the mouth, + ToF ports (nose, chin)

Building the hollow as *solid minus an inset solid* (rather than build123d's
``offset``/shell, which is fragile on lofts) keeps a clean, near-uniform wall.
"""

from __future__ import annotations

import math

from build123d import (Box, BuildPart, BuildSketch, Cylinder, Ellipse, Part, Plane,
                       Pos, Rot, Sphere, loft, scale)

from cad import params as P
from cad.servo import DEFAULT as SERVO

# --------------------------------------------------------------------------- body
# Elliptical loft stations: (x_along_body, half_width_y, half_height_z). The y/z
# half-extents scale with BODY_W / BODY_H (drawn at BODY_W_REF x BODY_H_REF) so the
# skin still fully covers the (optimized, wider) frame + leg-mount nodes.
# FORE shoulder — DE-FLARED (remote-axle hip drive): the fore-shoulder band no longer
# has to bulge past the hip joint line to enclose a fat ~45 mm hip servo body. That body
# has moved into the torso core (body.py), so the shoulder now carries only a slim Ø6
# axle + Ø13 bearing boss. The fore stations at x=52/72 come back IN from ~75/81 mm
# half-width to ~63/62 mm (base 47/46 × wy) — enough to cover the core (ribs ~48, core
# hip servos reach |y|≈40) with a slim, hugging silhouette. The legs + axle + bearing
# bracket exit through the (UNCHANGED) hip line at |y|=79.5 (a leg hole in the skin).
#
# AFT haunch — kept WIDE. The rear hip servos also move to the aft core, but the rear
# four-bar CRANK/knee servo still rides on the thigh just below the hip (part of the
# validated four-bar), and the haunch reads as a cat's muscular rump anyway, so the aft
# stations stay broad to smoothly cover that hardware.
_FORE_BASE = [(0, 33, 30), (26, 45, 40), (52, 47, 43),
              (72, 46, 42), (85, 44, 37), (P.FORE_LEN, 27, 26)]
_AFT_BASE = [(0, 33, 30), (-32, 52, 43), (-58, 64, 45),
             (-74, 65, 42), (-86, 34, 27), (-P.AFT_LEN, 20, 20)]


def _scaled(stations):
    wy = P.BODY_W / P.BODY_W_REF
    wz = P.BODY_H / P.BODY_H_REF
    return [(x, a * wy, b * wz) for (x, a, b) in stations]


FORE_STATIONS = _scaled(_FORE_BASE)
AFT_STATIONS = _scaled(_AFT_BASE)


def _ellipsoid(a: float, b: float, c: float) -> Part:
    return scale(Sphere(1), by=(a, b, c))


def _loft_body(stations) -> Part:
    """Loft elliptical sections: stations = [(x, half_width_y, half_height_z), ...]."""
    with BuildPart() as p:
        for x, a, b in stations:
            with BuildSketch(Plane.YZ.offset(x)):
                Ellipse(a, b)
        loft()
    return p.part


def _inner_stations(stations, t: float, waist_extend: float) -> list:
    """Inset stations by wall ``t`` to form the hollow cavity.

    The waist end (first station, at x~0) is pushed *past* the rim by
    ``waist_extend`` so that end stays **open** (the fore/aft cavities meet and
    read as one smooth tube through the spine joint). The far end (neck / tail
    base) is pulled *in* by ``t`` so it gets a closed end cap.
    """
    ins = [(x, max(a - t, 0.5), max(b - t, 0.5)) for (x, a, b) in stations]
    x0, a0, b0 = ins[0]
    ins = [(x0 + waist_extend, a0, b0)] + ins          # open the waist end
    xf, af, bf = ins[-1]
    ins[-1] = (xf - (t if xf > 0 else -t), af, bf)      # cap the far end
    return ins


def _jetson_bay(t: float) -> Part:
    """Access bay + fan vent cutters for torso_fore (local frame)."""
    # Back access hatch over the JETSON (top skin removed). The Jetson now sits aft
    # at x∈[14,59] (centre ~37); the forward core (x≳61.6) holds the relocated hip
    # servos, so the hatch is centred at x=37 and shortened to end at ~60 — it opens
    # over the Jetson for install/wiring without exposing the hip drive.
    hatch = Pos(37, 0, 71) * _box(46, 50, 90)
    # Front fan vent: round Ø24 through the neck end-cap, on the body axis.
    vent = Pos(P.FORE_LEN, 0, 2) * (Rot(0, 90, 0) * Cylinder(12, 40))
    return hatch + vent


def _battery_bay() -> Part:
    """Belly bay cutter for torso_aft (local frame): 3S LiPo compartment."""
    return Pos(-40, 0, -60) * _box(66, 42, 80)


def _box(dx: float, dy: float, dz: float) -> Part:
    from build123d import Box
    return Box(dx, dy, dz)


def _hollow(outer_stations, t: float, waist_extend: float, cutters=None) -> Part:
    outer = _loft_body(outer_stations)
    inner = _loft_body(_inner_stations(outer_stations, t, waist_extend))
    shell = outer - inner
    if cutters is not None:
        shell -= cutters
    return shell


# --------------------------------------------------------------------- openings
# Everything that has to pass through the skin gets a real opening, and each one is cut
# from the REAL swept volume of the thing that passes through it -- the rule body.py
# already uses for its ribcage scallops, and for the same reason: a bounding disc either
# gapes or binds.
#
# Until now the only cutters here were the Jetson and battery bays. The legs, the neck,
# the tail and the waist had no opening at all, so they simply shared solid material with
# the skin -- the front thigh by 1086 mm3, the aft frame by 4608 -- while a comment at the
# top of this file claimed "a leg hole in the skin" that no code ever cut.

_PORT_CACHE: dict[str, Part] = {}


def _hip_frame(leg: str, hip: float):
    """The thigh's transform in the torso frame at hip angle ``hip`` (rad).

    Written out rather than imported from ``cad.assembly`` so the skin does not depend on
    the assembly module, which imports every part in the robot.
    """
    mx, my = P.MOUNTS[leg]
    s = P.leg_plane_sign(leg)
    return Pos(mx, my + s * P.leg_geom(leg)["hip_off"], 0) * Rot(0, -math.degrees(hip), 0)


def _thigh_proxy(leg: str) -> Part:
    """A slab-wise stand-in for the thigh, cheap enough to sweep.

    The thigh itself cannot be swept. Fusing nine copies of it -- fenestrae, servo pocket,
    channel and all -- exhausted memory outright, and when a smaller version of the same
    fuse did return, OCCT handed back an intersection of 413 mm3 against a skin that a
    SINGLE pose already shared 1086 mm3 with: a boolean reporting less overlap than one of
    its own inputs has failed silently. So the thigh is reduced first.

    One bounding box would be the obvious stand-in and it is a bad one -- 9x the thigh's
    real volume, because the thigh is an L of slim bone strut plus a fat lateral servo
    pack -- and it opens the fore flank by 18%. Slicing into z-slabs and taking each
    slab's own box halves that while keeping the port's outline following the leg.
    """
    from cad.parts.leg import leg_parts

    th = leg_parts(leg)["upper"]
    bb = th.bounding_box()
    h = (bb.max.Z - bb.min.Z) / P.SKIN_PORT_SLABS
    c = P.SKIN_CLEAR
    proxy = None
    for i in range(P.SKIN_PORT_SLABS):
        a, b = bb.min.Z + i * h, bb.min.Z + (i + 1) * h
        slab = th & (Pos(0, 0, (a + b) / 2) * Box(400, 400, b - a))
        if slab.volume < 1.0:
            continue
        sb = slab.bounding_box()
        box = Pos(sb.center().X, sb.center().Y, (a + b) / 2) * Box(
            sb.size.X + 2 * c, sb.size.Y + 2 * c, (b - a) + 2 * c)
        proxy = box if proxy is None else proxy + box
    return proxy


def _leg_port(leg: str) -> Part:
    """The opening the leg swings through, swept over the hip's real working window.

    The window is ``body.hip_work_range`` -- read from the motion library, asymmetric,
    and the same window the ribcage is scalloped to, so the skin and the frame cannot
    disagree about how far the leg goes.
    """
    hit = _PORT_CACHE.get(leg)
    if hit is not None:
        return hit

    from cad.parts.body import hip_work_range
    from sim.gait import stance_angles

    proxy = _thigh_proxy(leg)
    hip0 = stance_angles(leg)[0]
    lo, hi = hip_work_range(leg)
    n = P.SKIN_PORT_STEPS
    port = None
    for i in range(n):
        posed = _hip_frame(leg, hip0 + lo + (hi - lo) * i / (n - 1)) * proxy
        port = posed if port is None else port + posed
    _PORT_CACHE[leg] = port
    return port


NECK_MOUTH_X = 87.0 * P.SCALE      # the skin ends here and the neck comes out


def _neck_port() -> Part:
    """Where the neck column passes out through the front of the chest.

    The column already publishes its real swept volume for the ribcage to be hollowed by,
    so the skin reuses it rather than inventing a second, disagreeing estimate.

    ``head_sweep`` is deliberately NOT unioned in. The head is a separate shell that lives
    forward of and above this one, and cutting its 421 000 mm3 of sweep out of the chest
    took the whole front off and left the fore skin in 29 pieces. What the chest owes the
    head is a mouth, not a cavity.

    The sweep alone leaves a ragged rim -- a 9 mm3 chip of skin was surviving above the
    opening, unattached to anything -- so the taper ahead of ``NECK_MOUTH_X`` goes with it
    and the skin ends in a clean rim instead.
    """
    from cad.parts.neck import neck_sweep

    a = max(a for _x, a, _b in FORE_STATIONS)
    b = max(b for _x, _a, b in FORE_STATIONS)
    mouth = Pos(NECK_MOUTH_X + a, 0, 0) * Box(2 * a, 2 * (a + 10.0), 2 * (b + 10.0))
    return neck_sweep() + mouth


TAIL_MOUTH_X = 84.0 * P.SCALE      # the skin ends here and the tail comes out


def _tail_port() -> Part:
    """Where the tail exits the rump, swept over its joint range.

    The tail pivots at ``(-AFT_LEN, 0, BODY_H/4)`` about -y over LIM_TAIL (cad.assembly's
    link table is the authority on both), so the port is that arc, not a hole at neutral.
    """
    from cad.parts.tail import tail as tail_part

    piv = Pos(-P.AFT_LEN, 0, P.BODY_H / 4)
    t = tail_part()
    lo, hi = getattr(P, "LIM_TAIL", (-1.0, 1.0))
    n = 7
    port = None
    for i in range(n):
        ang = lo + (hi - lo) * i / (n - 1)
        posed = piv * (Rot(0, -math.degrees(ang), 0) * t)
        port = posed if port is None else port + posed

    # ...and the rump ends in a clean rim, for the same reason the neck does: the sweep
    # passes BEHIND the skin's last two stations rather than through them, so on its own
    # it left the tail cap standing as a separate 7012 mm3 solid with nothing holding it.
    a = max(a for _x, a, _b in AFT_STATIONS)
    b = max(b for _x, _a, b in AFT_STATIONS)
    return port + Pos(-(TAIL_MOUTH_X + a), 0, 0) * Box(
        2 * a, 2 * (a + 10.0), 2 * (b + 10.0))


WAIST_SEAM = 20.0 * P.SCALE        # half-width of the band the spine joint lives in
WAIST_BELLY_OPEN = 12.0 * P.SCALE  # how far ABOVE the belly line the notch reaches


def _waist_port(aft: bool) -> Part:
    """The belly opening at the spine seam.

    Two things need it. The waist servo's boss straddles the seam and hangs below the
    skin's belly line -- it was sharing 559 mm3 with the aft skin -- and the joint has to
    FOLD through LIM_WAIST, which it cannot do through a closed belly.

    Getting the cutter right took three tries and both failures are worth keeping. The
    whole frame is the obvious cutter and it is far too blunt: the ribs and stringers
    reach the skin's inner wall by design, so subtracting the frame punched the wall out
    everywhere they touch and left the fore skin in 29 pieces. Clipping that same
    subtraction to the seam band was better and still wrong -- it left chips of 154 and
    22 mm3 standing loose, which a printed part cannot have. A plain notch cuts cleanly,
    and it is sized from the servo pocket so it tracks the actuator rather than a number
    typed in once.
    """
    _l, w, _h = SERVO.pocket
    st = AFT_STATIONS if aft else FORE_STATIONS
    # the SEAM station's half-height, not the loft's deepest: the notch belongs at the
    # belly the joint actually folds through, and taking the maximum put it 18 mm lower
    # than the skin, where it cut a bite out of the chest and missed the boss entirely.
    b = st[0][2]
    dy = w + 6.0 + 2 * P.SKIN_CLEAR          # the boss straddling the seam, plus clearance
    dz = WAIST_BELLY_OPEN + 20.0
    return Pos(0, 0, -(b + 20.0 - dz / 2)) * Box(2 * WAIST_SEAM, dy + 20.0, dz)


def body_shell_fore(ports: bool = True) -> Part:
    """waist(0) -> chest(full) -> shoulder -> neck(taper). Hollow skin + Jetson bay.

    ``ports=False`` returns the bare loft, which is what the port cutters are measured
    against; nothing else should ask for it.
    """
    cut = _jetson_bay(P.SHELL_T)
    if ports:
        cut = cut + _leg_port("FL") + _leg_port("FR") + _neck_port() + _waist_port(False)
    return _hollow(FORE_STATIONS, P.SHELL_T, waist_extend=-8, cutters=cut)


def body_shell_aft(ports: bool = True) -> Part:
    """waist(0) -> haunch(fullest) -> hip -> tail base(taper). Hollow skin + battery bay."""
    cut = _battery_bay()
    if ports:
        cut = cut + _leg_port("RL") + _leg_port("RR") + _tail_port() + _waist_port(True)
    return _hollow(AFT_STATIONS, P.SHELL_T, waist_extend=+8, cutters=cut)


# --------------------------------------------------------------------------- head
def _head_outer() -> Part:
    R = P.HEAD_R
    skull = _ellipsoid(R * 0.98, R, R * 0.98)                 # big round skull
    skull += Pos(0, 0, R * 0.35) * _ellipsoid(R*0.7, R*0.8, R*0.5)  # tall forehead (baby schema)
    for s in (1, -1):                                          # chubby cheeks
        skull += Pos(R*0.25, s*R*0.55, -R*0.12) * _ellipsoid(R*0.5, R*0.42, R*0.5)
    skull += Pos(R*0.72, 0, -R*0.34) * _ellipsoid(R*0.46, R*0.52, R*0.36)  # small muzzle
    for s in (1, -1):                                          # big low-set eye recesses
        skull -= Pos(R*0.78, s*R*0.40, -R*0.02) * _ellipsoid(R*0.18, R*0.22, R*0.26)
    skull += Pos(R*1.05, 0, -R*0.30) * _ellipsoid(R*0.12, R*0.16, R*0.12)  # nose
    for s in (1, -1):                                          # cat ears
        ear = _ellipsoid(R*0.10, R*0.34, R*0.42)
        ear = Pos(-R*0.15, s*R*0.5, R*0.95) * (Rot(s*22, 0, 0) * ear)
        skull += ear
    return skull


def _head_cavity(t: float) -> Part:
    """Inner cavity: the big volumes inset by one wall thickness (ears/nose stay
    solid — they're too thin to hollow usefully)."""
    R = P.HEAD_R
    cav = _ellipsoid(R*0.98 - t, R - t, R*0.98 - t)
    cav += Pos(0, 0, R*0.35) * _ellipsoid(R*0.7 - t, R*0.8 - t, R*0.5 - t)
    for s in (1, -1):
        cav += Pos(R*0.25, s*R*0.55, -R*0.12) * _ellipsoid(R*0.5 - t, R*0.42 - t, R*0.5 - t)
    cav += Pos(R*0.72, 0, -R*0.34) * _ellipsoid(R*0.46 - t, R*0.52 - t, R*0.36 - t)
    return cav


def _face_ports() -> Part:
    """ToF depth-sensor ports (nose, chin) as forward-pointing bores. The main RGB
    camera is now a STEREO pair bored behind the eyes — see ``_sense_ports``."""
    R = P.HEAD_R
    x = lambda: Rot(0, 90, 0)                        # spin a Z-cylinder onto +X
    tof_nose = Pos(R*1.05, 0, -R*0.30) * (x() * Cylinder(4.0, 60))       # ToF at the nose
    tof_chin = Pos(R*0.72, 0, -R*0.62) * (x() * Cylinder(4.0, 60))       # ToF under the chin
    return tof_nose + tof_chin


def _sense_ports() -> Part:
    """Anatomical sensor mounts: a sense organ per hole, mirrored L/R.

    Frame: head centre at origin, +x forward, +z up. Positions track the organs
    sculpted in ``_head_outer`` (eye recesses, ear bases, nose, muzzle) so each
    sensor sits behind its feature. All bores over-run into the hollow cavity so
    they read as real through-holes for the sensor.
    """
    R = P.HEAD_R
    x = lambda: Rot(0, 90, 0)                        # spin a Z-cylinder onto +X
    ports = None

    def add(c: Part) -> Part:
        nonlocal ports
        ports = c if ports is None else ports + c
        return ports

    # Eyes -> stereo camera lens apertures (one behind each big low-set eye recess).
    for s in (1, -1):
        add(Pos(R*0.78, s*R*0.40, -R*0.02) * (x() * Cylinder(P.EYE_CAM_R, 60)))

    # Ears -> a MEMS mic port at each ear base (bored down into the skull from the
    # top, just inboard of where the ear lifts off).
    for s in (1, -1):
        add(Pos(0, s*R*0.50, R*0.72) * Cylinder(P.MIC_PORT_R, 60))

    # Nose -> e-nose gas intake grille: a short row of small forward vents just
    # below the nose so the sensor gets airflow (kept clear of the nose ToF bore).
    n = P.NOSE_VENT_N
    for i in range(n):
        dy = (i - (n - 1) / 2) * P.NOSE_VENT_DY
        add(Pos(R*0.99, dy, -R*0.44) * (x() * Cylinder(P.NOSE_VENT_R, 60)))

    # Mouth -> speaker grille: a row of small forward holes on the lower muzzle,
    # the outer holes lifted slightly to draw a subtle smile.
    m = P.SPKR_GRILLE_N
    half = (m - 1) / 2
    for i in range(m):
        off = i - half
        dy = off * P.SPKR_GRILLE_DY
        dz = (abs(off) / half if half else 0.0) * P.SPKR_GRILLE_SMILE  # smile curve
        add(Pos(R*0.98, dy, -R*0.56 + dz) * (x() * Cylinder(P.SPKR_GRILLE_R, 60)))

    return ports


def head_shell() -> Part:
    R = P.HEAD_R
    shell = _head_outer() - _head_cavity(P.SHELL_T)
    shell -= _face_ports()
    shell -= _sense_ports()
    return shell


def waist_collar() -> Part:
    """A short band bridging the fore↔aft skin seam (cosmetic; rides torso_fore).
    Slightly proud of the body waist so the seam reads as one continuous back."""
    return _loft_body([(-12, 34, 31), (0, 36, 33), (12, 34, 31)])


# --------------------------------------------------------------------------- assembly
def full_cat() -> Part:
    """Shells + posed legs + tail, fused — the cat silhouette for preview/print.

    A small waist collar bridges the fore/aft rims so the spine seam reads as one
    continuous, rounded body rather than two lofts butted together (cosmetic only
    — the printed halves stay separate so the waist joint still flexes)."""
    from cad.assembly import _leg_locations, leg_parts
    from cad.parts.tail import tail
    from sim.gait import ankle_from_knee, stance_angles

    lift = max(g["stance_depth"] + g["foot"] * 0.6 for g in (P.FRONT, P.REAR))
    base = Pos(0, 0, lift)
    parts = [base * body_shell_fore(), base * body_shell_aft()]

    # waist collar — a short blended band across the seam (cosmetic, preview only)
    collar = _loft_body([(-10, 32, 29), (0, 34, 31), (10, 32, 29)])
    parts.append(base * collar)

    for leg in P.LEGS:
        hip, knee = stance_angles(leg)
        ankle = ankle_from_knee(leg, knee)
        Ta, Th, Tk, Tan = _leg_locations(leg, hip, knee, ankle)
        pl = leg_parts(leg)
        parts += [base*Ta*pl["hip_bracket"], base*Th*pl["upper"],
                  base*Tk*pl["lower"], base*Tan*pl["foot"]]

    parts.append(Pos(P.FORE_LEN + P.NECK_L*0.3, 0, lift + P.BODY_H*0.35) * head_shell())
    parts.append(Pos(-P.AFT_LEN + 6, 0, lift + P.BODY_H*0.3) * (Rot(0, -40, 0) * tail()))

    fused = parts[0]
    for p in parts[1:]:
        fused += p
    return fused


if __name__ == "__main__":
    for name, fn in (("fore", body_shell_fore), ("aft", body_shell_aft),
                     ("head", head_shell)):
        vol = fn().volume
        mass = vol * 1e-9 * P.EFFECTIVE_DENSITY * 1000.0
        print(f"{name:>4} shell vol: {vol:10.0f} mm^3   plastic mass: {mass:6.1f} g")
    print("full cat vol :", round(full_cat().volume, 0))
