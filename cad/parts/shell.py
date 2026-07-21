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

from build123d import (BuildPart, BuildSketch, Cylinder, Ellipse, Part, Plane,
                       Pos, Rot, Sphere, loft, scale)

from cad import params as P

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


def body_shell_fore() -> Part:
    # waist(0) -> chest(full) -> shoulder -> neck(taper). Hollow skin + Jetson bay.
    return _hollow(FORE_STATIONS, P.SHELL_T, waist_extend=-8,
                   cutters=_jetson_bay(P.SHELL_T))


def body_shell_aft() -> Part:
    # waist(0) -> haunch(fullest) -> hip -> tail base(taper). Hollow skin + battery bay.
    return _hollow(AFT_STATIONS, P.SHELL_T, waist_extend=+8,
                   cutters=_battery_bay())


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
