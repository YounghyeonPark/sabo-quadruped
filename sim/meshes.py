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

from cad.assembly import PRINTABLE, kinematics

MESH_DIR = os.path.join(os.path.dirname(__file__), "meshes")

# bodies that get a translucent skin over the frame: link/root name -> skin mesh
SKIN_OVER = {"torso_fore": "skin_fore", "torso_aft": "skin_aft", "head_tilt": "skin_head"}


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
    with open(marker, "w") as f:
        f.write("ok")
    return MESH_DIR


if __name__ == "__main__":
    d = ensure_meshes(force=True)
    print(f"Exported {len(all_mesh_names())} meshes to {d}")
