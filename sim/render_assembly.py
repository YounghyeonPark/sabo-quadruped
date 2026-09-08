"""Assembly stills via MuJoCo — the robot as its own kinematics actually pose it.

Steps the rig to a settled stance, then captures a set of views with a free camera
following the torso CoM. Saves to ``cad/out/assembly_<view>.png``.

Azimuth runs with the robot's nose at 175 deg, so the views are:

    hero    front 3/4 — the one the README leads with, so it has to show the FACE
    front   straight on
    side    profile, for the digitigrade stance
    iso     rear 3/4, which is what the old hero shot actually was
"""

from __future__ import annotations

import os

import mujoco
from PIL import Image

from sim.mj_emulate import Rig

OUT = os.path.join(os.path.dirname(os.path.dirname(__file__)), "cad", "out")


def _cam(rig, az, el, dist=0.62):
    com = rig.data.subtree_com[1]
    cam = mujoco.MjvCamera()
    mujoco.mjv_defaultCamera(cam)
    cam.lookat[0], cam.lookat[1], cam.lookat[2] = float(com[0]), float(com[1]), float(com[2])
    cam.distance = dist
    cam.azimuth = az
    cam.elevation = el
    return cam


def main():
    os.makedirs(OUT, exist_ok=True)
    rig = Rig()
    # Rig() resets to the validated stance (ctrl already holds stance targets); step ~300
    # to let it settle level on the floor before the still.
    for _ in range(300):
        mujoco.mj_step(rig.model, rig.data)
    renderer = mujoco.Renderer(rig.model, 750, 1000)
    for label, (az, el) in (("hero", (138, -14)), ("front", (175, -8)),
                            ("side", (95, -8)), ("iso", (55, -18))):
        renderer.update_scene(rig.data, camera=_cam(rig, az, el))
        p = os.path.join(OUT, f"assembly_{label}.png")
        Image.fromarray(renderer.render()).save(p)
        print("wrote", p)


if __name__ == "__main__":
    main()
