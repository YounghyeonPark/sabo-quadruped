"""
Fastener + joint-hardware selection, counted from the CAD itself.
=================================================================

    python -m analysis.hardware_bom              # markdown table + totals
    python -m analysis.hardware_bom --json       # machine-readable

``analysis/bom.py`` prices the robot at the level of "M2/M3 screws + heat-set inserts,
$10-18". That is fine for a cost estimate and useless for actually ordering parts: it
does not say how many M2 inserts, how long the pins are, or which bearing fits the hip
axle. This module answers that, and it answers it from the geometry rather than from a
hand-kept list -- every insert, screw and servo-case fastener is COUNTED by instrumenting
``cad.parts.fasteners`` while each printable part is rebuilt, then multiplied by the
per-robot part counts in ``cad/export.py``. Change a bolt circle or add a bracket and the
order list moves with it.

Joint hardware that is a bought metal part rather than a hole (pivot pins, ball bearings,
the hip drive axles, the split-alignment dowels) is derived from ``cad/params.py`` in
``_metal_parts`` -- sizes come from the same constants the CAD bores to, so a pin and its
bore cannot disagree.

Prices are typical hobby/maker USD (2025-26) for the small quantities a single build
needs, given as low-high. Fasteners are normally bought in assortment packs; the
"order as" column says what to actually buy.
"""

from __future__ import annotations

import argparse
import json

from cad import params as P
from cad.parts import fasteners as F
from cad.servo import DEFAULT as SERVO

# ------------------------------------------------------------------ instrumentation
_TALLY: dict[str, int] = {}
_IN_CASE_SCREWS = False


def _bump(key: str, n: int = 1) -> None:
    _TALLY[key] = _TALLY.get(key, 0) + n


def _instrument():
    """Wrap the fastener primitives so building a part counts its features.

    ``servo_case_screws`` calls ``screw_clearance`` four times; the re-entrancy guard
    keeps those from being counted twice under two different names.
    """
    orig = {n: getattr(F, n) for n in
            ("heatset_hole", "screw_clearance", "horn_holes", "servo_case_screws",
             "dowel_socket")}

    def heatset_hole(spec="M2", *a, **kw):
        _bump("insert:" + spec)
        return orig["heatset_hole"](spec, *a, **kw)

    def screw_clearance(spec="M2", *a, **kw):
        if not _IN_CASE_SCREWS:
            _bump("screw:" + spec)
        return orig["screw_clearance"](spec, *a, **kw)

    def horn_holes(spec=None, *a, **kw):
        _bump("horn_iface:" + (spec or P.HORN_SCREW))
        return orig["horn_holes"](spec, *a, **kw)

    def servo_case_screws(axis, span, length, spec=None, inset=None):
        global _IN_CASE_SCREWS
        spec = spec or P.SERVO_MOUNT_SCREW
        _bump("case_screw:" + spec, P.HORN_SCREWS)
        _IN_CASE_SCREWS = True
        try:
            return orig["servo_case_screws"](axis, span, length, spec, inset)
        finally:
            _IN_CASE_SCREWS = False

    def dowel_socket(fit="clear", *a, **kw):
        _bump("dowel_socket:" + fit)
        return orig["dowel_socket"](fit, *a, **kw)

    for name, fn in (("heatset_hole", heatset_hole), ("screw_clearance", screw_clearance),
                     ("horn_holes", horn_holes), ("servo_case_screws", servo_case_screws),
                     ("dowel_socket", dowel_socket)):
        setattr(F, name, fn)
    return orig


def _restore(orig) -> None:
    for n, fn in orig.items():
        setattr(F, n, fn)


def count_features() -> dict[str, int]:
    """Per-robot feature counts, from rebuilding every printable part under the
    instrumented fastener module and scaling by ``cad.export.COUNTS``."""
    orig = _instrument()
    try:
        from cad.export import COUNTS
        total: dict[str, int] = {}
        for name, build in _builders().items():
            _TALLY.clear()
            build()
            for k, v in _TALLY.items():
                total[k] = total.get(k, 0) + v * COUNTS.get(name, 1)

        # split sub-parts add their own dowel sockets on top of the whole parts
        _TALLY.clear()
        from cad.parts.split import split_parts
        split_parts()
        for k, v in _TALLY.items():
            if k.startswith("dowel_socket:press"):
                total[k] = total.get(k, 0) + v
        return total
    finally:
        _restore(orig)


def _builders() -> dict:
    """Each printable part's OWN builder.

    ``leg_parts()`` builds a whole leg, so using it here would tally every part of the leg
    once per part of the leg — which it did, inflating the order list four-fold."""
    if True:
        from cad.parts.body import torso_aft, torso_fore
        from cad.parts.ears import ear
        from cad.parts.head import head
        from cad.parts.leg import (crank, foot_seg, hip_bracket, lower_leg, pushrod,
                                    upper_leg)
        from cad.parts.neck import neck_column, pan_crank, pan_pushrod
        from cad.parts.tail import tail, tail_crank, tail_pushrod
        from sim.gait import stance_angles

        # NOTE: call each part's OWN builder. ``leg_parts()`` builds a whole leg, so using
        # it here would tally every part of the leg once per part of the leg.
        def _lower(kind):
            leg = "FL" if kind == "F" else "RL"
            return lambda: lower_leg(P.leg_geom(leg)["lower"], stance_angles(leg)[1])

        builders = {
            "torso_fore": torso_fore, "torso_aft": torso_aft, "head": head,
            "ear": ear, "tail": tail,
            "hipbr_F_L": lambda: hip_bracket(+1, P.FRONT["hip_off"]),
            "hipbr_F_R": lambda: hip_bracket(-1, P.FRONT["hip_off"]),
            "hipbr_R_L": lambda: hip_bracket(+1, P.REAR["hip_off"]),
            "hipbr_R_R": lambda: hip_bracket(-1, P.REAR["hip_off"]),
            "upper_F": lambda: upper_leg(P.FRONT["upper"], +1),
            "upper_R": lambda: upper_leg(P.REAR["upper"], +1),
            "lower_F": _lower("F"), "lower_R": _lower("R"),
            "foot_F": lambda: foot_seg(P.FRONT["foot"]),
            "foot_R": lambda: foot_seg(P.REAR["foot"]),
            "crank_F": crank, "crank_R": crank,
            "pushrod_F": pushrod, "pushrod_R": pushrod,
            "tail_crank": tail_crank, "tail_pushrod": tail_pushrod,
            "neck_column": neck_column, "pan_crank": pan_crank,
            "pan_pushrod": pan_pushrod,
        }
        return builders


def per_part_inserts() -> dict[str, int]:
    """Heat-set inserts in ONE of each printable part, counted from its geometry.

    ``cad/print_manifest.py`` carries the same numbers by hand, for the build table. They
    are two statements of one fact, so the manifest checks itself against this rather than
    letting the two drift — which they had, by sixteen inserts.
    """
    orig = _instrument()
    try:
        out: dict[str, int] = {}
        for name, build in _builders().items():
            _TALLY.clear()
            build()
            out[name] = sum(v for k, v in _TALLY.items() if k.startswith("insert:"))
        return out
    finally:
        _restore(orig)


# ------------------------------------------------------------------ metal parts
def _pin_lengths() -> dict[str, float]:
    """Pivot pin lengths, from the stack each pin actually passes through."""
    lo, hi = P.clevis_slot()
    fork = (hi - lo) + 2 * P.CLEVIS_CHEEK_T          # cheek + tongue + cheek
    link = P.CLEVIS_TONGUE_T + P.CLEVIS_GAP + P.CLEVIS_ROD_T
    grooves = 4.0                                    # e-clip land at each end
    return {"clevis": fork + grooves, "linkage": link + grooves}


_BEARING_CODES = {(6, 13, 5): "686ZZ", (5, 13, 4): "695ZZ", (6, 15, 5): "696ZZ",
                  (8, 16, 5): "688ZZ", (4, 13, 5): "624ZZ"}


def _bearing_code(hb: dict) -> str:
    """Metric miniature deep-groove code for a (bore, OD, width) in mm."""
    key = (round(2 * hb["bore_r"]), round(2 * hb["od_r"]), round(hb["width"]))
    return _BEARING_CODES.get(key, "%dx%dx%d (no standard code)" % key)


def hip_axle_length() -> float:
    """Cut length of one hip drive axle -- core servo horn out to the thigh's cheek."""
    lo, _ = P.clevis_slot(rod=True)
    return ((P.BODY_W / 2 + P.FRONT["hip_off"]) - P.HIP_CORE_HORN_Y
            + (lo - P.CLEVIS_CHEEK_T) - 1.5)


def _metal_parts() -> list[dict]:
    """Bought metal joint hardware, sized from the constants the CAD bores to."""
    pl = _pin_lengths()
    n = len(P.LEGS)
    hb = P.HIP_BEARING
    d_pin = 2 * P.PIN_R
    return [
        dict(item="Dowel pin Dia%.0f m6 x %.0f mm, stainless" % (d_pin, pl["clevis"]),
             qty=2 * n, lo=0.30, hi=0.60,
             pack="Dia3 dowel pin assortment",
             note="knee + ankle clevis pivots, one per joint, double shear"),
        dict(item="Dowel pin Dia%.0f m6 x %.0f mm, stainless" % (d_pin, pl["linkage"]),
             qty=2 * n, lo=0.30, hi=0.60,
             pack="Dia3 dowel pin assortment",
             note="four-bar C (crank-rod) and R (rod-rocker) pivots"),
        dict(item="Dowel pin Dia%.0f m6 x %.0f mm, stainless (tail four-bar)" % (
                 d_pin, pl["linkage"]),
             qty=2, lo=0.30, hi=0.60,
             pack="Dia3 dowel pin assortment",
             note="tail remote drive: crank-rod and rod-rocker pivots"),
        dict(item="Dowel pin Dia%.0f m6 x %.0f mm, stainless (tail pivot)" % (
                 d_pin, pl["clevis"]),
             qty=1, lo=0.30, hi=0.60,
             pack="Dia3 dowel pin assortment",
             note="the tail's own clevis pivot on the torso tongue"),
        dict(item="E-clip DIN 6799 for Dia%.0f shaft" % d_pin,
             qty=8 * n + 10, lo=0.05, hi=0.12,
             pack="Dia3 E-clip pack (100)",
             note="retains both ends of every pivot pin (legs, tail, head yaw); seats in pin_head_seat"),
        dict(item="Dowel pin Dia%.0f m6 x %.0f mm, stainless (head yaw four-bar)" % (
                 d_pin, pl["linkage"]),
             qty=2, lo=0.30, hi=0.60,
             pack="Dia3 dowel pin assortment",
             note="head pan remote drive: crank-rod and rod-column pivots"),
        dict(item="Ball bearing %s (%.0fx%.0fx%.0f)" % (
                 _bearing_code(hb), 2 * hb["bore_r"], 2 * hb["od_r"], hb["width"]),
             qty=2 * n + 1, lo=1.20, hi=2.50, pack="10-pack",
             note="hip drive axles (2 per leg) + the neck's yaw bearing in the chest"),
        dict(item="Shaft Dia%.0f h6 x %.0f mm, hardened steel or CF rod" % (
                 2 * P.AXLE_R, hip_axle_length()),
             qty=n, lo=1.50, hi=3.50, pack="Dia6 rod, cut to length",
             note="hip drive axle: core servo horn out to the thigh inboard cheek"),
        dict(item="Dowel rod Dia%.0f x %.0f mm" % (
                 2 * P.SPLIT_DOWEL_R, 2 * P.SPLIT_DOWEL_DEPTH - 1),
             qty=None, lo=0.15, hi=0.35, pack="Dia4 rod, cut to length",
             key="dowel_socket:press",
             note="print-split alignment (torso halves + head); press one side, slip the other"),
    ]


# ------------------------------------------------------------------ threaded parts
_SCREW_PRICE = {"M2": (0.04, 0.09), "M3": (0.05, 0.10)}
_INSERT_PRICE = {"M2": (0.08, 0.18), "M3": (0.10, 0.22)}


def rows() -> list[dict]:
    c = count_features()
    out: list[dict] = []

    for spec in ("M2", "M3"):
        n = c.get("insert:" + spec, 0)
        if not n:
            continue
        h = P.HEATSET[spec]
        out.append(dict(
            item="Heat-set insert %s x %.1f mm (Dia%.1f melt hole)" % (
                spec, h["depth"], 2 * h["bore_r"]),
            qty=n, lo=_INSERT_PRICE[spec][0], hi=_INSERT_PRICE[spec][1],
            pack="%s insert pack (50-100)" % spec,
            note="melt-in brass; every threaded joint in the printed frame"))
        out.append(dict(
            item="Socket-head screw %s x %.0f mm, stainless" % (spec, h["depth"] + 4),
            qty=n, lo=_SCREW_PRICE[spec][0], hi=_SCREW_PRICE[spec][1],
            pack="%s screw assortment" % spec,
            note="one per %s insert" % spec))

    spec = P.SERVO_MOUNT_SCREW
    n = c.get("case_screw:" + spec, 0)
    if n:
        out.append(dict(
            item="Socket-head screw %s x 10 mm, stainless (servo case)" % spec,
            qty=n, lo=_SCREW_PRICE[spec][0], hi=_SCREW_PRICE[spec][1],
            pack="%s screw assortment" % spec,
            note="through the printed boss into the STS3215 own tapped flange, "
                 "4 per servo, no insert"))

    n = c.get("horn_iface:" + P.HORN_SCREW, 0)
    if n:
        out.append(dict(
            item="Servo horn centre screw (supplied with the STS3215)",
            qty=n, lo=0.0, hi=0.0, pack="in the servo box",
            note="one per horn interface; clamps the horn to the output spline"))

    for spec in ("M2", "M3"):
        n = c.get("screw:" + spec, 0)
        if n:
            out.append(dict(
                item="Socket-head screw %s x 16 mm, stainless (leg to torso)" % spec,
                qty=n, lo=_SCREW_PRICE[spec][0], hi=_SCREW_PRICE[spec][1],
                pack="%s screw assortment" % spec,
                note="clearance through the hip bracket foot into the torso pad inserts"))

    for m in _metal_parts():
        if m.get("key"):
            m = dict(m)
            m["qty"] = c.get(m.pop("key"), 0)
        out.append(m)
    return out


def housed_servos() -> int:
    """How many of the robot's servos actually have a modelled pocket + case screws in a
    printed part. Anything less than ``P.N_SERVOS`` means some actuator has nowhere to be
    bolted, which no amount of fastener counting will fix."""
    return count_features().get("case_screw:" + P.SERVO_MOUNT_SCREW, 0) // P.HORN_SCREWS


def totals(rs) -> tuple[float, float]:
    return (sum(r["qty"] * r["lo"] for r in rs), sum(r["qty"] * r["hi"] for r in rs))


def main() -> None:
    ap = argparse.ArgumentParser(description="Sabo joint + fastener hardware selection")
    ap.add_argument("--json", action="store_true", help="machine-readable output")
    args = ap.parse_args()

    rs = rows()
    if args.json:
        print(json.dumps({"rows": rs, "total_usd": totals(rs)}, indent=2))
        return

    lo, hi = totals(rs)
    print("# Sabo - joint & fastener hardware\n")
    print("_Auto-generated by `python -m analysis.hardware_bom`. Counts come from the CAD:")
    print("every insert, screw and case fastener is tallied while each printable part is")
    print("rebuilt, then scaled by the per-robot part counts. Sizes come from `cad/params.py`,")
    print("so a pin and the bore it goes in cannot disagree._\n")
    print("Actuator: **%s** x%d.\n" % (SERVO.name, P.N_SERVOS))
    print("| Part | Qty | Unit $ | Ext $ | Order as | Where it goes |")
    print("|---|--:|--:|--:|---|---|")
    for r in rs:
        u = "-" if r["hi"] == 0 else "%.2f-%.2f" % (r["lo"], r["hi"])
        e = "-" if r["hi"] == 0 else "%.2f-%.2f" % (r["qty"] * r["lo"], r["qty"] * r["hi"])
        print("| %s | %d | %s | %s | %s | %s |" % (r["item"], r["qty"], u, e, r["pack"], r["note"]))
    print("\n**Joint + fastener hardware total: $%.0f-$%.0f per robot.**\n" % (lo, hi))
    print("Notes")
    print("-----")
    print("* Buy the fasteners as assortment packs -- the per-piece prices above are what the")
    print("  quantities are worth, not what a vendor will sell you a dozen of.")
    print("* Every pivot is a **clevis in double shear**: the pin passes cheek-tongue-cheek")
    print("  and is retained by an E-clip at each end, so no pin is cantilevered.")
    print("* The STS3215 case screws thread into the servo's **own tapped flanges** -- they")
    print("  need no insert, but the printed boss must relieve the flanges (%.0f mm across,"
          % SERVO.flange_l)
    print("  vs a %.0f mm case)." % SERVO.body_l)
    print("* Verify the servo's flange hole pattern against your actual STS3215 before")
    print("  ordering screw lengths; the CAD pattern is indicative.")

    housed = sum(r["qty"] for r in rs
                 if "servo case" in r["item"]) // P.HORN_SCREWS
    if housed < P.N_SERVOS:
        print("")
        print("> **Incomplete: %d of %d servos have a modelled housing.**"
              % (housed, P.N_SERVOS))
        print("> `python -m analysis.actuator_fit` measures which joints have room and")
        print("> which do not; `capacity()` measures how many an enclosure can hold at all.")
    else:
        print("")
        print("> **All %d servos have a modelled housing.** Every actuator has a pocket,"
              % P.N_SERVOS)
        print("> case screws and a horn interface cut for it in a printed part, and")
        print("> `python -m analysis.actuator_fit` confirms each one has the room.")
        print(">")
        print("> Two of them could not be housed on their own joint and are driven")
        print("> through a four-bar from wherever there WAS room, the way the hip already")
        print("> was: the **tail** (its pivot is at the rear extremity of the frame) and")
        print("> the head's **yaw** (the head is the last link in the chain, so a servo")
        print("> inside it can only reach the joint immediately above — the other axis has")
        print("> to come from the chest). Head **roll** is not a joint at all: the head")
        print("> holds two housings and roll is the one axis a camera can correct in")
        print("> software, so it went to electronic stabilisation. The **ears** are rigid")
        print("> for the same arithmetic.")


if __name__ == "__main__":
    main()
