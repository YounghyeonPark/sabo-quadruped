"""
Mesh export — real geometry for the MuJoCo simulator.
=====================================================

Writes an STL per body (the **inner frame** links) plus the three **skin case**
pieces to ``sim/meshes/``, so the simulator can render the actual cat — inner
frame inside a translucent skin — instead of gray primitive boxes. The meshes are
visual only; MuJoCo collision stays on the simple primitives (stable + fast).

Each part is exported in its own local frame (proximal joint at the origin), which
is exactly the MuJoCo body frame, so a mesh geom sits at ``pos 0 0 0`` (with a
mm→m ``scale`` on the asset). Regenerate after geometry changes:

    python -m sim.meshes           # or ensure_meshes(force=True)
"""

from __future__ import annotations

import os

from build123d import export_stl

from cad import params as P
from cad.assembly import PRINTABLE, kinematics

MESH_DIR = os.path.join(os.path.dirname(__file__), "meshes")

# bodies that get a skin over the frame: link/root name -> skin mesh.
# The limbs are in here too. A static shell cannot close the flank -- the knee servo lies
# laterally and sweeps it open as the hip works -- so the thigh and shank carry their own
# covers (cad/parts/fairing.py), and the render only reads as a cat if they are drawn.
SKIN_OVER = {"torso_fore": "skin_fore", "torso_aft": "skin_aft", "head_pitch": "skin_head"}
SKIN_OVER.update({f"{leg}_hip": f"fair_{leg}_upper" for leg in P.LEGS})
SKIN_OVER.update({f"{leg}_knee": f"fair_{leg}_lower" for leg in P.LEGS})


# link/root name -> the PRINTABLE key of the skin bolted to it. Same mapping as
# SKIN_OVER, kept beside it so the mass model and the visual model cannot drift apart.
SKIN_PART = {"torso_fore": "shell_fore", "torso_aft": "shell_aft", "head_pitch": "shell_head"}
SKIN_PART.update({f"{leg}_hip": f"fair_upper_{leg[0]}" for leg in P.LEGS})
SKIN_PART.update({f"{leg}_knee": f"fair_lower_{leg[0]}" for leg in P.LEGS})

_SKIN_MASS_CACHE: dict[str, float] = {}


def skin_mass_kg(link: str) -> float:
    """Mass of the skin bolted to ``link``, in kg. 0.0 if it carries none.

    The physics model used to see the FRAME only, so the skin -- 124 g of shell and 76 g
    of limb fairings -- was missing from every gait run while ``analysis.validate``
    published the robot at 1514 g. The headline mass and the gait numbers would have come
    from two different robots.

    Read from the export manifest rather than by building the part: a sim run should not
    pay for the swept leg ports. The manifest is written by ``cad.export``, which
    ``analysis.platform_report`` runs before any sim stage.
    """
    key = SKIN_PART.get(link)
    if key is None:
        return 0.0
    if not _SKIN_MASS_CACHE:
        import json
        path = os.path.join(os.path.dirname(__file__), "..", "cad", "out",
                            "parts_manifest.json")
        try:
            parts = json.load(open(path))["parts"]
            _SKIN_MASS_CACHE.update({k: v["mass_g_each"] / 1000.0 for k, v in parts.items()})
        except Exception:
            from cad.assembly import PRINTABLE
            from cad import params as _P
            for k in set(SKIN_PART.values()):
                _SKIN_MASS_CACHE[k] = (PRINTABLE[k].volume * 1e-9
                                       * _P.EFFECTIVE_DENSITY)
    return _SKIN_MASS_CACHE.get(key, 0.0)


def _export(part, name: str, tolerance: float = 0.001) -> None:
    # ``tolerance`` (mm) sets the STL chord tolerance. The cosmetic skin lofts are
    # large curved surfaces whose booleans (Jetson vent/hatch, battery bay) can shed
    # sliver triangles; at the fine default they tip past MuJoCo's 200k-face-per-mesh
    # decoder limit. They are translucent VISUAL meshes only, so a coarser tolerance
    # (invisible on screen) keeps them well under the limit without touching print STLs.
    export_stl(part, os.path.join(MESH_DIR, f"{name}.stl"), tolerance=tolerance)


def frame_mesh_names() -> list[str]:
    return ["torso_fore"] + [lk.name for lk in kinematics()]


def all_mesh_names() -> list[str]:
    return frame_mesh_names() + list(SKIN_OVER.values()) + ["skin_collar"]


def ensure_meshes(force: bool = False) -> str:
    """Export all meshes if missing (or force). Returns the mesh directory."""
    os.makedirs(MESH_DIR, exist_ok=True)
    marker = os.path.join(MESH_DIR, ".done")
    if os.path.exists(marker) and not force:
        return MESH_DIR
    # inner frame: root + every link's printed part
    _export(PRINTABLE["torso_fore"], "torso_fore")
    for lk in kinematics():
        _export(lk.part, lk.name)
    # skin case
    from cad.parts.shell import (body_shell_aft, body_shell_fore, head_shell,
                                 waist_collar)
    _export(body_shell_fore(), "skin_fore", tolerance=0.05)
    _export(body_shell_aft(), "skin_aft", tolerance=0.05)
    _export(head_shell(), "skin_head", tolerance=0.05)
    _export(waist_collar(), "skin_collar", tolerance=0.05)
    from cad.parts.fairing import shank_fairing, thigh_fairing
    for leg in P.LEGS:
        _export(thigh_fairing(leg), f"fair_{leg}_upper", tolerance=0.05)
        _export(shank_fairing(leg), f"fair_{leg}_lower", tolerance=0.05)
    with open(marker, "w") as f:
        f.write("ok")
    return MESH_DIR


if __name__ == "__main__":
    d = ensure_meshes(force=True)
    print(f"Exported {len(all_mesh_names())} meshes to {d}")
