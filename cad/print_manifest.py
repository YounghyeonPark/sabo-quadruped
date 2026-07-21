"""
Print manifest — per-part FDM print plan for Sabo's printable parts.
====================================================================

    python -m cad.print_manifest

For every part in ``cad.assembly.PRINTABLE`` this records the engineering print
decisions that a slicer can't infer from geometry alone:

    * ``orient``      recommended build orientation (minimise supports; keep layer
                      lines OFF the main load path)
    * ``support``     does it need support material as oriented?  (y/n)
    * ``split``       must / should it be SPLIT to print without heroic supports?
    * ``material``    suggested filament (PLA cosmetic-light / PETG structural)
    * ``perimeters`` / ``infill``   slicer starting point
    * ``inserts``     M2/M3 brass heat-set insert count in ONE copy of the part

The judgments live in ``PRINT_META`` (hand-authored); the module cross-checks it
against ``PRINTABLE`` (fails loudly if a part is added/removed without updating
this), multiplies inserts by ``cad.export.COUNTS`` for robot totals, and emits
``cad/out/print_manifest.json`` + a printed table.
"""

from __future__ import annotations

import json
import os

from cad.assembly import PRINTABLE
from cad.export import COUNTS, OUT

# Heat-set inserts here are the load-bearing HORN interfaces + the head↔neck join;
# servo CASE screws (into the STS3215's own tapped flanges) and four-bar PINS
# (retained by e-clip / shoulder-screw-head seats) are clearance features, not
# inserts, and are called out in docs/assembly.md instead.
PRINT_META: dict[str, dict] = {
    # ---- torso frame (open ribcage halves) -----------------------------------
    "torso_fore": dict(
        orient="body axis (X) vertical, waist face on bed; Jetson-bay side out",
        support=True, split="recommended: sagittal L/R — hoops arch off the cut face",
        material="PETG", perimeters=3, infill=0.18, inserts=4,
        notes="waist horn pad (4x M2) + 4 leg-node abduction mounts + bus channels"),
    "torso_aft": dict(
        orient="body axis (X) vertical, waist face on bed; belly/battery bay out",
        support=True, split="recommended: sagittal L/R — hoops arch off the cut face",
        material="PETG", perimeters=3, infill=0.18, inserts=0,
        notes="waist servo pocket (M2 case screws) + 4 leg-node mounts + bus channels"),
    # ---- head / appendages ---------------------------------------------------
    "head": dict(
        orient="neck stub down; OR split at the equator into two bowls",
        support=True, split="recommended: horizontal equator split (2 bowls, bond)",
        material="PLA", perimeters=3, infill=0.10, inserts=4,
        notes="hollow skull; muzzle bump + eye/cam bores need support if not split"),
    "ear": dict(
        orient="lay the blade flat on the bed (largest face down)",
        support=False, split="no", material="PLA", perimeters=3, infill=0.20, inserts=2,
        notes="thin 3 mm blade prints flat with no support; 2x M2 horn inserts in base"),
    "tail": dict(
        orient="lay the tail axis along the bed, curl upward",
        support=False, split="no", material="PLA", perimeters=3, infill=0.20, inserts=4,
        notes="gentle up-curl self-supports; 4x M2 horn inserts + centre relief in base"),
    # ---- leg: hip brackets (abduction mount, rigid, load-bearing) ------------
    "hipbr_F_L": dict(orient="servo-pocket mouth up (avoid pocket support)",
                      support=True, split="no", material="PETG", perimeters=4,
                      infill=0.25, inserts=0, notes="rigid abduction root; M2 servo case screws"),
    "hipbr_F_R": dict(orient="servo-pocket mouth up (avoid pocket support)",
                      support=True, split="no", material="PETG", perimeters=4,
                      infill=0.25, inserts=0, notes="rigid abduction root; M2 servo case screws"),
    "hipbr_R_L": dict(orient="servo-pocket mouth up (avoid pocket support)",
                      support=True, split="no", material="PETG", perimeters=4,
                      infill=0.25, inserts=0, notes="rigid abduction root; M2 servo case screws"),
    "hipbr_R_R": dict(orient="servo-pocket mouth up (avoid pocket support)",
                      support=True, split="no", material="PETG", perimeters=4,
                      infill=0.25, inserts=0, notes="rigid abduction root; M2 servo case screws"),
    # ---- leg: bone struts ----------------------------------------------------
    "upper_F": dict(orient="strut axis flat on bed; crank-servo pocket up",
                    support=True, split="no", material="PETG", perimeters=4, infill=0.30,
                    inserts=4, notes="carries knee servo (crank boss); 4x M2 hip-horn inserts"),
    "upper_R": dict(orient="strut axis flat on bed; crank-servo pocket up",
                    support=True, split="no", material="PETG", perimeters=4, infill=0.30,
                    inserts=4, notes="carries knee servo (crank boss); 4x M2 hip-horn inserts"),
    "lower_F": dict(orient="strut axis flat on bed; rocker + pin seats sideways",
                    support=False, split="no", material="PETG", perimeters=4, infill=0.30,
                    inserts=0, notes="welded rocker; passive knee + ankle pins (e-clip seats)"),
    "lower_R": dict(orient="strut axis flat on bed; rocker + pin seats sideways",
                    support=False, split="no", material="PETG", perimeters=4, infill=0.30,
                    inserts=0, notes="welded rocker; passive knee + ankle pins (e-clip seats)"),
    "foot_F": dict(orient="lay on side; paw pad off the bed",
                   support=True, split="no", material="PETG", perimeters=3, infill=0.30,
                   inserts=0, notes="paw pad ideal in TPU (grip); passive ankle pin seat"),
    "foot_R": dict(orient="lay on side; paw pad off the bed",
                   support=True, split="no", material="PETG", perimeters=3, infill=0.30,
                   inserts=0, notes="paw pad ideal in TPU (grip); passive ankle pin seat"),
    # ---- four-bar knee linkage (flat links, high stress) ---------------------
    "crank_F": dict(orient="flat on bed (link plane down) — no support",
                    support=False, split="no", material="PETG", perimeters=4, infill=0.60,
                    inserts=4, notes="knee torque path; 4x M2 horn inserts + centre horn screw"),
    "crank_R": dict(orient="flat on bed (link plane down) — no support",
                    support=False, split="no", material="PETG", perimeters=4, infill=0.60,
                    inserts=4, notes="knee torque path; 4x M2 horn inserts + centre horn screw"),
    "pushrod_F": dict(orient="flat on bed (link plane down) — no support",
                      support=False, split="no", material="PETG", perimeters=4, infill=0.60,
                      inserts=0, notes="rigid coupler; Ø3 rotating pins + bearing counterbores"),
    "pushrod_R": dict(orient="flat on bed (link plane down) — no support",
                      support=False, split="no", material="PETG", perimeters=4, infill=0.60,
                      inserts=0, notes="rigid coupler; Ø3 rotating pins + bearing counterbores"),
}


# ---- SPLIT sub-parts (how the split-flagged whole parts are physically printed) --------
# These are emitted by ``cad.parts.split`` and exported alongside the whole parts. They are
# NOT in PRINTABLE (the mass/sim model uses the intact parts) so they live in their own
# meta table and their own manifest section — one row per printable HALF, oriented CUT-FACE
# DOWN so the flat glue plane is on the bed. Inserts that straddle a cut (the waist horn
# pad) are installed AFTER bonding the halves; see docs/assembly.md.
SPLIT_META: dict[str, dict] = {
    "torso_fore_L": dict(source="torso_fore", material="PETG", perimeters=3, infill=0.18,
                         support=False, inserts=2,
                         orient="sagittal cut face (XZ, Y=0) on bed; hoops arch up — near-zero support",
                         notes="left half; 2x M2 waist-horn inserts (of 4, set after bond); "
                               "2 spine + 1 keel Ø4 dowel PRESS sockets"),
    "torso_fore_R": dict(source="torso_fore", material="PETG", perimeters=3, infill=0.18,
                         support=False, inserts=2,
                         orient="sagittal cut face (XZ, Y=0) on bed; hoops arch up — near-zero support",
                         notes="right half; 2x M2 waist-horn inserts (of 4, set after bond); "
                               "matching Ø4 dowel CLEARANCE sockets + glue-relief channels"),
    "torso_aft_L": dict(source="torso_aft", material="PETG", perimeters=3, infill=0.18,
                        support=False, inserts=0,
                        orient="sagittal cut face (XZ, Y=0) on bed; hoops arch up — near-zero support",
                        notes="left half; waist servo pocket split (servo captured on bond); "
                              "2 spine + 1 keel Ø4 dowel PRESS sockets"),
    "torso_aft_R": dict(source="torso_aft", material="PETG", perimeters=3, infill=0.18,
                        support=False, inserts=0,
                        orient="sagittal cut face (XZ, Y=0) on bed; hoops arch up — near-zero support",
                        notes="right half; matching Ø4 dowel CLEARANCE sockets + glue-relief channels"),
    "head_A": dict(source="head", material="PLA", perimeters=3, infill=0.10,
                   support=True, inserts=4,
                   orient="cut face (equator, Z=8) down, dome up; eye/cam bores bridge — light support",
                   notes="lower FACE bowl: both eyes + camera bore + muzzle + neck stub (4x M2 "
                         "inserts), open top to seat/wire camera+LEDs; 3 Ø4 dowel PRESS sockets"),
    "head_B": dict(source="head", material="PLA", perimeters=3, infill=0.10,
                   support=False, inserts=0,
                   orient="cut face (equator, Z=8) down, dome up — support-free cap",
                   notes="upper cranial CAP (plain dome); 3 Ø4 dowel CLEARANCE sockets + rim glue groove"),
}


def build_manifest() -> dict:
    missing = set(PRINTABLE) - set(PRINT_META)
    extra = set(PRINT_META) - set(PRINTABLE)
    if missing or extra:
        raise KeyError(f"PRINT_META out of sync with PRINTABLE: missing={missing} extra={extra}")
    # SPLIT_META must exactly cover the sub-parts emitted by cad.parts.split
    from cad.parts.split import SPLIT_SOURCE
    s_missing = set(SPLIT_SOURCE) - set(SPLIT_META)
    s_extra = set(SPLIT_META) - set(SPLIT_SOURCE)
    if s_missing or s_extra:
        raise KeyError(f"SPLIT_META out of sync with split_parts: missing={s_missing} extra={s_extra}")

    parts = {}
    total_inserts = 0
    supported = split_flagged = 0
    for name, meta in PRINT_META.items():
        count = COUNTS.get(name, 1)
        robot_inserts = meta["inserts"] * count
        total_inserts += robot_inserts
        supported += int(bool(meta["support"]))
        split_flagged += int(not str(meta["split"]).startswith("no"))
        parts[name] = {**meta, "count": count, "inserts_per_robot": robot_inserts}

    split_parts = {name: {**meta, "count": 1} for name, meta in SPLIT_META.items()}

    return {
        "parts": parts,
        "split_parts": split_parts,
        "totals": {
            "distinct_parts": len(parts),
            "heat_set_inserts_total": total_inserts,
            "parts_needing_support": supported,
            "parts_split_flagged": split_flagged,
            "split_sub_parts": len(split_parts),
        },
        "insert_specs": {"M2": {"screw": "M2", **_hs("M2")}, "M3": {"screw": "M3", **_hs("M3")}},
    }


def _hs(spec: str) -> dict:
    from cad import params as P
    return P.HEATSET[spec]


def main() -> None:
    os.makedirs(OUT, exist_ok=True)
    m = build_manifest()
    with open(os.path.join(OUT, "print_manifest.json"), "w") as f:
        json.dump(m, f, indent=2)

    hdr = f"{'part':<12}{'x':>3}{'mat':>6}{'per':>4}{'inf':>6}{'sup':>5}{'ins':>5}  orient / split"
    print(hdr)
    print("-" * len(hdr))
    for name, p in m["parts"].items():
        sp = "" if str(p["split"]).startswith("no") else "  [SPLIT] " + p["split"]
        print(f"{name:<12}{p['count']:>3}{p['material']:>6}{p['perimeters']:>4}"
              f"{p['infill']*100:>5.0f}%{('Y' if p['support'] else 'n'):>5}"
              f"{p['inserts']:>5}  {p['orient']}{sp}")
    print("-" * len(hdr))
    print("SPLIT sub-parts (print the split-flagged wholes as these halves, then bond):")
    for name, p in m["split_parts"].items():
        print(f"{name:<14}{'1':>1}{p['material']:>6}{p['perimeters']:>4}"
              f"{p['infill']*100:>5.0f}%{('Y' if p['support'] else 'n'):>5}"
              f"{p['inserts']:>5}  {p['orient']}")
    t = m["totals"]
    print("-" * len(hdr))
    print(f"{t['distinct_parts']} distinct parts (+{t['split_sub_parts']} split halves) | "
          f"{t['heat_set_inserts_total']} heat-set inserts (per robot) | "
          f"{t['parts_needing_support']} need support | "
          f"{t['parts_split_flagged']} split-flagged")


if __name__ == "__main__":
    main()
