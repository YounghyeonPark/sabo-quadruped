"""
Fastener + print-fit features — shared robust primitives (build123d).
=====================================================================

Every mating clearance and every fastener boss / hole in the printable parts comes
from HERE, so the whole robot uses ONE fit standard (``cad.params``). Change a
clearance or an insert size in params and every part re-fits.

All features are plain cylinder / box booleans so the parts stay watertight:
  * heat-set-insert BOSSES are ADDITIVE (a solid cylinder unioned into the part),
  * insert / screw / pin / wire HOLES are blind or through cylinders (subtracted).

Axis convention: ``"x"|"y"|"z"`` is the hole/boss axis in the part's local frame.
A cylinder is modelled along +z then rotated onto the requested axis.
"""

from __future__ import annotations

import math

from build123d import Cylinder, Part, Pos, Rot

from cad import params as P


def _cyl(axis: str, r: float, h: float) -> Part:
    """A solid cylinder of radius ``r``, length ``h``, centred at the origin, whose
    axis is the local ``axis`` (``x``/``y``/``z``)."""
    c = Cylinder(radius=r, height=h)
    if axis == "z":
        return c
    if axis == "y":
        return Rot(90, 0, 0) * c
    if axis == "x":
        return Rot(0, 90, 0) * c
    raise ValueError(f"axis must be x/y/z, got {axis!r}")


# ------------------------------------------------------------------ rotating / fixed pins
def pin_bore(length: float, rotating: bool = True, axis: str = "y") -> Part:
    """Bore for the Ø3 four-bar / knee pin. ``rotating`` → loose PIN_CLEARANCE fit
    (the pivot turns); else a press fit (bore smaller by PRESS_INTERFERENCE) for a
    pin anchored in that link."""
    r = P.PIN_R + (P.PIN_CLEARANCE if rotating else -P.PRESS_INTERFERENCE)
    return _cyl(axis, r, length)


def pin_head_seat(axis: str = "y", face: float = 0.0, depth: float = 1.4) -> Part:
    """Shallow counterbore at a pivot's outer face to retain the pin — seats an
    M3 shoulder-screw head, a washer, or an e-clip. ``face`` = signed offset of the
    outer face along ``axis``; the seat is cut inward from there."""
    r = P.SCREW["M3"]["head_r"]
    seat = _cyl(axis, r, depth)
    d = depth / 2
    pos = {"x": (face - math.copysign(d, face), 0, 0),
           "y": (0, face - math.copysign(d, face), 0),
           "z": (0, 0, face - math.copysign(d, face))}[axis]
    return Pos(*pos) * seat


# ------------------------------------------------------------------ heat-set inserts
def heatset_hole(spec: str = "M2", axis: str = "z", length: float | None = None) -> Part:
    """Blind cylinder the brass heat-set insert melts into. Default length = the
    insert depth + a short lead-in; pass ``length`` to over-run into a cavity."""
    h = P.HEATSET[spec]
    ln = length if length is not None else h["depth"] + 1.0
    return _cyl(axis, h["bore_r"], ln)


def heatset_boss(spec: str = "M2", axis: str = "z", height: float | None = None) -> Part:
    """Solid boss sized to carry a heat-set insert (OD = bore + 2*min_wall). Union
    it into the part where it overlaps solid material, then subtract ``heatset_hole``."""
    h = P.HEATSET[spec]
    ht = height if height is not None else h["depth"] + h["min_wall"]
    return _cyl(axis, h["boss_r"], ht)


# ------------------------------------------------------------------ screw clearance
def screw_clearance(spec: str = "M2", axis: str = "z", length: float = 20.0,
                    head: bool = False) -> Part:
    """Through clearance hole for a machine screw shank, optionally with a head
    counterbore at the +``axis`` end (socket-cap / countersink pocket)."""
    s = P.SCREW[spec]
    body = _cyl(axis, s["clr_r"], length)
    if head:
        hd = _cyl(axis, s["head_r"], s["head_h"])
        off = length / 2 - s["head_h"] / 2
        pos = {"x": (off, 0, 0), "y": (0, off, 0), "z": (0, 0, off)}[axis]
        body = body + Pos(*pos) * hd
    return body


# ------------------------------------------------------------------ servo horn bolt circle
def horn_holes(spec: str | None = None, axis: str = "y", length: float | None = None,
               n: int | None = None, bolt_circle: float | None = None,
               center: bool = True) -> Part:
    """Union of the horn-screw HEAT-SET holes on the Ø(bolt_circle) circle in the
    plane perpendicular to ``axis``, plus (optionally) the centre horn-shaft
    clearance hole. Subtract from the part face the servo horn drives."""
    spec = spec or P.HORN_SCREW
    n = n or P.HORN_SCREWS
    bc = (bolt_circle or P.HORN_BOLT_CIRCLE) / 2.0
    ln = length if length is not None else P.HEATSET[spec]["depth"] + 1.5
    # in-plane axes (the two axes that are not `axis`)
    plane = {"x": ("y", "z"), "y": ("x", "z"), "z": ("x", "y")}[axis]
    cutter = None
    for i in range(n):
        ang = 2 * math.pi * i / n + math.pi / n
        u, v = bc * math.cos(ang), bc * math.sin(ang)
        pos = {"x": 0.0, "y": 0.0, "z": 0.0}
        pos[plane[0]] = u
        pos[plane[1]] = v
        hole = Pos(pos["x"], pos["y"], pos["z"]) * heatset_hole(spec, axis, ln)
        cutter = hole if cutter is None else cutter + hole
    if center:
        cutter = cutter + _cyl(axis, P.HORN_CENTER_R, ln * 1.4)
    return cutter


# ------------------------------------------------------------------ servo case retention
def servo_case_screws(axis: str, span: tuple[float, float], length: float,
                      spec: str | None = None, inset: float | None = None) -> Part:
    """Four case-retention clearance holes along ``axis`` at the corners of the
    rectangle ``span`` (the servo mount face extents perpendicular to ``axis``),
    inset from each edge. Placed parallel to a boss's shaft-relief bore, so the
    screws enter the servo's flange face. Subtract at the servo location."""
    spec = spec or P.SERVO_MOUNT_SCREW
    inset = P.SERVO_MOUNT_INSET if inset is None else inset
    a, b = span[0] / 2 - inset, span[1] / 2 - inset
    plane = {"x": ("y", "z"), "y": ("x", "z"), "z": ("x", "y")}[axis]
    cutter = None
    for su in (+1, -1):
        for sv in (+1, -1):
            pos = {"x": 0.0, "y": 0.0, "z": 0.0}
            pos[plane[0]] = su * a
            pos[plane[1]] = sv * b
            hole = Pos(pos["x"], pos["y"], pos["z"]) * screw_clearance(spec, axis, length)
            cutter = hole if cutter is None else cutter + hole
    return cutter


# ------------------------------------------------------------------ split-part dowel socket
def dowel_socket(fit: str = "clear", axis: str = "y", length: float | None = None,
                 r: float | None = None) -> Part:
    """Blind bore for a SEPARATE alignment dowel rod that straddles a print-split cut
    (``cad/parts/split.py``). ``fit="press"`` → bore smaller than the rod by
    ``PRESS_INTERFERENCE`` (the rod jams in that half); ``fit="clear"`` → bore larger by
    ``PIN_CLEARANCE`` (the rod slips, so the halves still separate for gluing)."""
    r = P.SPLIT_DOWEL_R if r is None else r
    if fit == "press":
        rr = r - P.PRESS_INTERFERENCE
    elif fit == "clear":
        rr = r + P.PIN_CLEARANCE
    else:
        raise ValueError(f"fit must be press/clear, got {fit!r}")
    ln = 2 * P.SPLIT_DOWEL_DEPTH if length is None else length
    return _cyl(axis, rr, ln)


# ------------------------------------------------------------------ wire pass-throughs
def wire_hole(kind: str = "bus", axis: str = "z", length: float = 30.0) -> Part:
    """Cable pass-through: ``bus`` = the 3-wire STS3215 TTL daisy-chain trunk,
    ``sensor`` = a thinner sensor cable (camera / mic / IMU / speaker)."""
    r = P.BUS_WIRE_R if kind == "bus" else P.SENSOR_WIRE_R
    return _cyl(axis, r, length)
