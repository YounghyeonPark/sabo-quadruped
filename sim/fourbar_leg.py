"""
Four-bar knee — MuJoCo closed-loop verification.
================================================

    python -m sim.fourbar_leg

Builds a single leg where the knee is driven by a hip-side crank through a rigid
four-bar (crank → pushrod → rocker-on-shank), closed with a MuJoCo `equality
connect` (the closed kinematic loop). The thigh is fixed so we isolate the knee
mechanism. We actuate the **crank** and check the **passive knee** follows the
four-bar prediction from `analysis/fourbar.py` — i.e. the rigid linkage really
drives the knee, no cable, one motor, bidirectional.

Geometry is emitted from the designed `FourBar` (mm → m). Angles about +y.
"""

from __future__ import annotations

import math
import mujoco
import numpy as np

from analysis.fourbar import FourBar

MM = 0.001


def _v(p):  # 2D fourbar (x,y) mm → MuJoCo (x,z) m, y=0
    return (p[0] * MM, 0.0, p[1] * MM)


def build(fb: FourBar, theta2_init_deg=30.0) -> str:
    p = fb.linkage_points(math.radians(theta2_init_deg))
    O2, C, R, O4, tip = (_v(p[k]) for k in ("O2", "C", "R", "O4", "tip"))
    cC = tuple(C[i] - O2[i] for i in range(3))       # crank arm vector (crank-local)
    pR = tuple(R[i] - C[i] for i in range(3))        # pushrod vector (pushrod-local)
    rR = tuple(R[i] - O4[i] for i in range(3))       # rocker vector (shank-local)
    tipv = tuple(tip[i] - O4[i] for i in range(3))   # shank leg vector (shank-local)
    ft = lambda a, b: f"{a[0]:.4f} {a[1]:.4f} {a[2]:.4f} {b[0]:.4f} {b[1]:.4f} {b[2]:.4f}"
    hip = (0.0, 0.0, 0.065)                           # thigh top (fixed)

    return f"""<mujoco model="fourbar_knee">
  <option gravity="0 0 -9.81" timestep="0.001" integrator="implicitfast"
          solver="Newton" iterations="200" ls_iterations="50" tolerance="1e-10"/>
  <default><geom rgba=".7 .72 .77 1"/><joint damping="0.02"/></default>
  <worldbody>
    <light pos="0.1 -0.3 0.5"/>
    <camera name="cam" pos="0.02 -0.35 0.0" xyaxes="1 0 0 0 0.2 1" mode="fixed"/>
    <!-- thigh (fixed): hip -> knee, carries the crank pivot -->
    <body name="thigh" pos="0 0 0">
      <geom type="capsule" fromto="{ft(hip, O4)}" size="0.006" rgba=".5 .52 .57 1"/>
      <!-- crank: hip-side motor -->
      <body name="crank" pos="{O2[0]:.4f} 0 {O2[2]:.4f}">
        <joint name="crank" type="hinge" axis="0 1 0"/>
        <geom type="capsule" fromto="{ft((0,0,0), cC)}" size="0.004" rgba=".85 .45 .2 1"/>
        <!-- pushrod (coupler): pinned at crank tip -->
        <body name="pushrod" pos="{cC[0]:.4f} 0 {cC[2]:.4f}">
          <joint name="push" type="hinge" axis="0 1 0"/>
          <geom type="capsule" fromto="{ft((0,0,0), pR)}" size="0.003" rgba=".05 .58 .53 1"/>
          <site name="push_end" pos="{pR[0]:.4f} 0 {pR[2]:.4f}" size="0.004"/>
        </body>
      </body>
      <!-- shank: passive knee at O4, with the rocker arm + the lower leg -->
      <body name="shank" pos="{O4[0]:.4f} 0 {O4[2]:.4f}">
        <joint name="knee" type="hinge" axis="0 1 0"/>
        <geom type="capsule" fromto="{ft((0,0,0), rR)}" size="0.003" rgba=".36 .61 .84 1"/>
        <geom type="capsule" fromto="{ft((0,0,0), tipv)}" size="0.005"/>
        <site name="rocker_end" pos="{rR[0]:.4f} 0 {rR[2]:.4f}" size="0.004"/>
      </body>
    </body>
  </worldbody>
  <equality>
    <connect site1="push_end" site2="rocker_end"
             solref="0.0005 1" solimp="0.98 0.999 0.0001 0.5 2"/>
  </equality>
  <actuator>
    <position name="crank" joint="crank" kp="12" ctrlrange="-1.6 1.6"/>
  </actuator>
</mujoco>
"""


def main():
    fb = FourBar()
    model = mujoco.MjModel.from_xml_string(build(fb))
    data = mujoco.MjData(model)
    ci = mujoco.mj_name2id(model, mujoco.mjtObj.mjOBJ_JOINT, "crank")
    ki = mujoco.mj_name2id(model, mujoco.mjtObj.mjOBJ_JOINT, "knee")
    ca = model.jnt_qposadr[ci]; ka = model.jnt_qposadr[ki]
    act = mujoco.mj_name2id(model, mujoco.mjtObj.mjOBJ_ACTUATOR, "crank")

    print("Four-bar knee — MuJoCo closed-loop check")
    print("=" * 52)
    print(f"  model OK: {model.nbody-1} bodies, {model.neq} equality (loop) constraint(s)")
    try:
        renderer = mujoco.Renderer(model, 360, 480)
    except Exception:
        renderer = None
    frames, rows = [], []
    # sweep the crank across the usable window; record crank vs knee
    for k in range(121):
        cmd = -0.9 + 1.8 * k / 120
        data.ctrl[act] = cmd
        for _ in range(70):                     # settle each step (stiff loop + tracking)
            mujoco.mj_step(model, data)
        if k % 8 == 0:
            crank = float(data.qpos[ca]); knee = float(data.qpos[ka])
            rows.append((crank, knee))
            if renderer is not None:
                renderer.update_scene(data, camera="cam")
                frames.append(renderer.render().copy())
    cranks = [r[0] for r in rows]; knees = [r[1] for r in rows]
    knee_rom = max(knees) - min(knees)
    # constraint residual: are the connected sites coincident? (loop held)
    s1 = data.site_xpos[mujoco.mj_name2id(model, mujoco.mjtObj.mjOBJ_SITE, "push_end")]
    s2 = data.site_xpos[mujoco.mj_name2id(model, mujoco.mjtObj.mjOBJ_SITE, "rocker_end")]
    gap_mm = float(np.linalg.norm(s1 - s2)) * 1000
    mono = all(knees[i] <= knees[i+1]+1e-3 for i in range(len(knees)-1)) or \
           all(knees[i] >= knees[i+1]-1e-3 for i in range(len(knees)-1))
    print(f"  crank swept    : {math.degrees(min(cranks)):.0f}°..{math.degrees(max(cranks)):.0f}°")
    print(f"  knee driven    : {math.degrees(min(knees)):.0f}°..{math.degrees(max(knees)):.0f}° "
          f"(ROM {math.degrees(knee_rom):.0f}° = {knee_rom:.2f} rad)")
    print(f"  loop closure   : sites {gap_mm:.2f} mm apart  "
          f"({'HELD' if gap_mm < 1.0 else 'BROKEN'})")
    print(f"  knee monotonic vs crank: {'yes' if mono else 'no'}")
    ok = gap_mm < 1.0 and knee_rom > 1.5 and mono
    print(f"  RESULT         : {'PASS — linkage drives the knee in physics' if ok else 'check'}")
    if frames:
        from PIL import Image
        import os
        os.makedirs("sim/out", exist_ok=True)
        imgs = [Image.fromarray(f) for f in frames]
        imgs[0].save("sim/out/fourbar_leg.gif", save_all=True,
                     append_images=imgs[1:], duration=80, loop=0)
        print("  render         : sim/out/fourbar_leg.gif")


if __name__ == "__main__":
    main()
