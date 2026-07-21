"""
Export — turn the parametric model into printable files + a mass manifest.
===========================================================================

    python -m cad.export

Writes to ``cad/out/``:
    <part>.stl / <part>.step   per-part printable geometry (slice these)
    robokitten_full.stl        the whole posed robot (preview)
    parts_manifest.json        per-part volume, printed mass, count, bbox
    preview_iso.png / preview_side.png   headless renders (matplotlib)

Printed-plastic mass = volume × EFFECTIVE_DENSITY; bought components are added
from params.COMPONENT_MASS for the full mass budget.
"""

from __future__ import annotations

import json
import os

import matplotlib
matplotlib.use("Agg")
import matplotlib.pyplot as plt
import numpy as np
import trimesh
from build123d import export_step, export_stl
from mpl_toolkits.mplot3d.art3d import Poly3DCollection

from cad import params as P
from cad.assembly import PRINTABLE, full_robot

OUT = os.path.join(os.path.dirname(__file__), "out")

# how many of each printed part the robot needs
COUNTS = {
    "torso_fore": 1, "torso_aft": 1, "head": 1, "tail": 1, "ear": 2,
    "hipbr_F_L": 1, "hipbr_F_R": 1, "hipbr_R_L": 1, "hipbr_R_R": 1,
    "upper_F": 2, "lower_F": 2, "foot_F": 2,
    "upper_R": 2, "lower_R": 2, "foot_R": 2,
    "crank_F": 2, "crank_R": 2, "pushrod_F": 2, "pushrod_R": 2,
}


def _mass_g(volume_mm3: float) -> float:
    return volume_mm3 * 1e-9 * P.EFFECTIVE_DENSITY * 1000.0  # mm^3 -> m^3 -> kg -> g


def _render(stl_path: str, png_path: str, elev: float, azim: float) -> None:
    mesh = trimesh.load(stl_path)
    fig = plt.figure(figsize=(7, 5))
    ax = fig.add_subplot(111, projection="3d")
    tris = mesh.vertices[mesh.faces]

    # Lambert shading so hollowing, wall thickness and apertures actually read
    # (a flat single-colour silhouette hides every recess/opening).
    e, a = np.radians(elev), np.radians(azim)
    light = np.array([np.cos(e) * np.cos(a), np.cos(e) * np.sin(a), np.sin(e)])
    light = light / (np.linalg.norm(light) + 1e-9)
    n = mesh.face_normals
    shade = 0.35 + 0.65 * np.clip(np.abs(n @ light), 0.0, 1.0)  # ambient + diffuse
    base = np.array([0.80, 0.82, 0.86])
    facecolors = np.clip(shade[:, None] * base[None, :], 0, 1)

    coll = Poly3DCollection(tris, alpha=1.0, linewidths=0.0)
    coll.set_facecolor(facecolors)
    coll.set_edgecolor(facecolors)
    ax.add_collection3d(coll)
    b = mesh.bounds
    ctr = mesh.centroid
    r = float(np.max(b[1] - b[0])) / 2
    for setlim, c in ((ax.set_xlim, ctr[0]), (ax.set_ylim, ctr[1]), (ax.set_zlim, ctr[2])):
        setlim(c - r, c + r)
    ax.view_init(elev=elev, azim=azim)
    ax.set_box_aspect((1, 1, 1))
    ax.set_axis_off()
    fig.tight_layout()
    fig.savefig(png_path, dpi=120)
    plt.close(fig)


def main() -> None:
    os.makedirs(OUT, exist_ok=True)
    manifest = {"parts": {}, "targets": {}, "totals": {}}

    printed_total_g = 0.0
    for name, part in PRINTABLE.items():
        stl = os.path.join(OUT, f"{name}.stl")
        step = os.path.join(OUT, f"{name}.step")
        export_stl(part, stl)
        export_step(part, step)
        vol = float(part.volume)
        count = COUNTS.get(name, 1)
        bb = part.bounding_box()
        each_g = _mass_g(vol)
        printed_total_g += each_g * count
        manifest["parts"][name] = {
            "volume_mm3": round(vol, 1),
            "mass_g_each": round(each_g, 1),
            "count": count,
            "bbox_mm": [round(bb.size.X, 1), round(bb.size.Y, 1), round(bb.size.Z, 1)],
        }

    # print-SPLIT sub-parts (cut + bond geometry for the oversized parts). These are
    # how the WHOLE torso_fore / torso_aft / head are physically printed (in halves, then
    # bonded); the mass budget above already counts the intact parts once, so the halves
    # are recorded here for slicing but NOT re-added to the mass total. The small volume
    # delta is the deliberate bond material (spine/keel pads, head pads) minus dowel gaps.
    from cad.parts.split import SPLIT_SOURCE, split_parts
    manifest["split_parts"] = {}
    split_vol_by_src: dict[str, float] = {}
    for name, part in split_parts().items():
        export_stl(part, os.path.join(OUT, f"{name}.stl"))
        export_step(part, os.path.join(OUT, f"{name}.step"))
        vol = float(part.volume)
        bb = part.bounding_box()
        src = SPLIT_SOURCE[name]
        split_vol_by_src[src] = split_vol_by_src.get(src, 0.0) + vol
        manifest["split_parts"][name] = {
            "source": src,
            "volume_mm3": round(vol, 1),
            "mass_g_each": round(_mass_g(vol), 1),
            "count": 1,
            "bbox_mm": [round(bb.size.X, 1), round(bb.size.Y, 1), round(bb.size.Z, 1)],
        }
    split_extra_g = _mass_g(sum(split_vol_by_src[s] - PRINTABLE[s].volume for s in split_vol_by_src))

    # whole robot preview
    robot = full_robot()
    full_stl = os.path.join(OUT, "robokitten_full.stl")
    export_stl(robot, full_stl)

    comp_g = P.component_mass_total() * 1000.0
    total_g = printed_total_g + comp_g
    lo, hi = P.MASS_TARGET
    manifest["totals"] = {
        "printed_plastic_g": round(printed_total_g, 1),
        "components_g": round(comp_g, 1),
        "total_g": round(total_g, 1),
    }
    manifest["targets"] = {
        "mass_kg": list(P.MASS_TARGET),
        "mass_in_target": lo <= total_g / 1000.0 <= hi,
        "servo": {"name": __import__("cad.servo", fromlist=["DEFAULT"]).DEFAULT.name},
    }
    with open(os.path.join(OUT, "parts_manifest.json"), "w") as f:
        json.dump(manifest, f, indent=2)

    # renders (functional frame)
    _render(full_stl, os.path.join(OUT, "preview_iso.png"), elev=22, azim=-58)
    _render(full_stl, os.path.join(OUT, "preview_side.png"), elev=6, azim=-90)

    # cosmetic cat shell (the organic outer shape) + cat renders
    from cad.parts.shell import body_shell_aft, body_shell_fore, full_cat, head_shell
    for name, part in (("shell_fore", body_shell_fore()), ("shell_aft", body_shell_aft()),
                       ("shell_head", head_shell())):
        export_stl(part, os.path.join(OUT, f"{name}.stl"))
    # hip drive AXLES (remote-axle hip drive): a lateral shaft per leg from the core
    # servo horn out to the hip pivot. Steel/CF rod + printed horn hubs, so — like the
    # four-bar pins + split dowels — NOT in the plastic mass budget; exported here for
    # slicing + OCCT validity only. Front (L=39.5) / rear (L=46.0) differ by hip_off.
    from cad.parts.leg import hip_axle
    for tag, ho in (("hip_axle_F", P.FRONT["hip_off"]), ("hip_axle_R", P.REAR["hip_off"])):
        export_stl(hip_axle(ho), os.path.join(OUT, f"{tag}.stl"))

    cat_stl = os.path.join(OUT, "sabo_cat.stl")
    export_stl(full_cat(), cat_stl)
    _render(cat_stl, os.path.join(OUT, "cat_iso.png"), elev=18, azim=-60)
    _render(cat_stl, os.path.join(OUT, "cat_side.png"), elev=6, azim=-90)

    # print plan (orientation / supports / material / heat-set inserts)
    from cad.print_manifest import main as print_manifest_main
    print()
    print_manifest_main()
    print()

    # report
    print(f"Exported {len(PRINTABLE)} frame parts + full robot to {OUT}")
    print(f"  split sub-parts : {len(manifest['split_parts'])} halves "
          f"(torso_fore/aft L+R, head A+B); +{split_extra_g:.1f} g bond material when split")
    print("  cat shell       : shell_fore/aft/head.stl + sabo_cat.stl")
    print("  cat previews    : cat_iso.png, cat_side.png")
    print(f"  printed plastic : {printed_total_g:7.1f} g")
    print(f"  components      : {comp_g:7.1f} g  ({P.N_SERVOS} servos etc.)")
    print(f"  TOTAL           : {total_g:7.1f} g   "
          f"(target {lo*1000:.0f}-{hi*1000:.0f} g -> "
          f"{'OK' if lo <= total_g/1000 <= hi else 'OUT OF RANGE'})")
    print("  previews        : preview_iso.png, preview_side.png")


if __name__ == "__main__":
    main()
