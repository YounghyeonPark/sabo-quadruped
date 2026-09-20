"""The README's image set, rendered from the model rather than made by hand.

    python -m sim.render_docs

Writes ``docs/img/{hero,front,side,loaf,sit}.png`` and ``docs/img/walk.gif``.

These used to be captured ad hoc and copied in, which is exactly why they went stale:
the skin, the limb covers and the mass all moved while the README kept showing a robot
from before any of it. Everything else in the project regenerates (``analysis.platform_report``),
so the pictures do too.
"""

from __future__ import annotations

import os
import shutil

import mujoco
from PIL import Image

from sim import cute_motion as CM
from sim.mj_emulate import Rig, render_front
from sim.render_assembly import _cam

IMG = os.path.join(os.path.dirname(os.path.dirname(__file__)), "docs", "img")

# the stills the README leads with; azimuth has the nose at 175 deg
STANCE_VIEWS = (("hero", (138, -14)), ("front", (175, -8)), ("side", (95, -8)))
POSES = ("loaf", "sit")     # gestures held for the gallery
SETTLE = 300                # steps to let the stance settle on the floor
POSE_AT = 0.55              # fraction of the gesture to stop at -- see below


def _shot(rig, renderer, az, el, path):
    renderer.update_scene(rig.data, camera=_cam(rig, az, el))
    Image.fromarray(renderer.render()).save(path)
    print("wrote", path)


def main() -> None:
    os.makedirs(IMG, exist_ok=True)
    # ONE rig and ONE renderer for every still. Building a fresh Rig (and therefore a
    # fresh Renderer, since a Renderer is bound to its model) per pose ran out of GL
    # contexts on the third one and wrote a black frame -- silently, since the render
    # itself does not fail. ``_reset_stance`` puts the same rig back to the start.
    rig = Rig()
    renderer = mujoco.Renderer(rig.model, 750, 1000)

    for _ in range(SETTLE):
        mujoco.mj_step(rig.model, rig.data)
    for label, (az, el) in STANCE_VIEWS:
        _shot(rig, renderer, az, el, os.path.join(IMG, f"{label}.png"))

    # Poses are driven by the gesture CONTROLLER over the gesture's own duration, not by
    # snapping the rig to the held target. Snapping is not a motion the robot can make:
    # the sit went over on its face, rump 27 mm BELOW the floor and the torso at 114 deg,
    # and it looked like the added mass had broken the pose. Ramped properly it settles
    # where its docstring says it does -- shoulders 108 mm, rump 75, torso 4 deg.
    for name in POSES:
        rig._reset_stance()
        ctl = CM.make_cute_control([name], hold=True)
        dur = CM.GESTURES[name][1]
        # Stop PART WAY through, not at the end. Every gesture is shaped by _pulse(u,
        # 0.3, 0.8), which ramps in, HOLDS through the middle, and releases again -- so
        # at u = 1.0 the pose is let go and the gallery was photographing the robot on
        # its way back to stance. 0.55 is inside the held plateau.
        for i in range(int(dur * POSE_AT / rig.model.opt.timestep)):
            if i % 5 == 0:                      # the controller's own 10 ms tick
                ctl(rig, i * rig.model.opt.timestep)
            mujoco.mj_step(rig.model, rig.data)
        _shot(rig, renderer, 120, -10, os.path.join(IMG, f"{name}.png"))

    # walk, front-facing and CoM-tracked, which is what the caption claims.
    # mj_emulate already owns that shot (it has the camera and writes the gif);
    # ``simulate`` on its own returns telemetry, not a path.
    gif, _plot = render_front("walk", seconds=6.0)
    if os.path.exists(gif):
        shutil.copyfile(gif, os.path.join(IMG, "walk.gif"))
        print("wrote", os.path.join(IMG, "walk.gif"))
    else:
        print("NOTE: no 3-D frames captured; walk.gif left as it was")


if __name__ == "__main__":
    main()
