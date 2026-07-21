"""
MJCF builder — turn the cat-anatomy CAD into a MuJoCo model.
============================================================

Walks the 4-DOF kinematic tree from ``cad.assembly.kinematics()`` and emits a
MuJoCo model where each link is a body with:
  * a hinge joint on the real axis (hip/knee/ankle share the IK sign convention),
    range-limited to the servo's travel;
  * an explicit inertial from the printed part's bbox × real mass (CAD-volume
    plastic + the bought parts riding on that link, from ``Link.extra_mass``);
  * collision only on the toe pads (on each ``*_ankle`` foot segment) + floor + torso.
Actuators are position servos clamped to the servo stall torque.
"""

from __future__ import annotations

import os

from cad import params as P
from cad.assembly import PRINTABLE, ROOT, kinematics
from cad.servo import DEFAULT as SERVO
from sim.meshes import MESH_DIR, SKIN_OVER, all_mesh_names, ensure_meshes

_DENS = P.EFFECTIVE_DENSITY
FRAME_RGBA = "0.55 0.57 0.62 1"      # inner frame — solid grey
SKIN_RGBA = "0.86 0.55 0.35 0.38"    # torso skin — translucent apricot (see frame inside)
HEAD_RGBA = "0.82 0.83 0.88 1"       # head — opaque light skin (the cute face)
HEAD_LINK = "head_tilt"
# ears live on the head skin; the neck/gimbal stubs are internal — don't render them
SKIP_VISUAL = {"ear_L", "ear_R", "head_pan", "head_pitch"}


def _fore_electronics() -> float:
    c = P.COMPONENT_MASS
    # front half: Jetson + IMU + wiring, PLUS the two FRONT HIP servos whose bodies now
    # live in the fore torso core (remote-axle hip drive); battery/rear-hips ride aft.
    return c["pi"] + c["imu"] + c["wiring_misc"] + 2 * SERVO.mass_kg


def _bbox_m(part):
    bb = part.bounding_box()
    center = ((bb.min.X + bb.max.X) / 2000.0, (bb.min.Y + bb.max.Y) / 2000.0,
              (bb.min.Z + bb.max.Z) / 2000.0)
    size = (max(bb.size.X / 1000.0, 1e-3), max(bb.size.Y / 1000.0, 1e-3),
            max(bb.size.Z / 1000.0, 1e-3))
    return center, size


def _inertial(mass, center, size) -> str:
    a, b, c = size
    ix, iy, iz = (mass/12*(b*b+c*c), mass/12*(a*a+c*c), mass/12*(a*a+b*b))
    return (f'<inertial pos="{center[0]:.5f} {center[1]:.5f} {center[2]:.5f}" '
            f'mass="{max(mass,1e-3):.4f}" diaginertia="{ix:.6e} {iy:.6e} {iz:.6e}"/>')


def _link_mass(lk) -> float:
    return lk.part.volume * 1e-9 * _DENS + lk.extra_mass


def build_mjcf(start_z: float | None = None) -> str:
    links = kinematics()
    children: dict[str, list] = {}
    for lk in links:
        children.setdefault(lk.parent, []).append(lk)
    if start_z is None:
        start_z = P.m(max(g["stance_depth"] + g["foot"] * 0.6
                          for g in (P.FRONT, P.REAR))) + 0.01

    def emit_body(lk, indent) -> str:
        pad = "  " * indent
        x, y, z = (P.m(v) for v in lk.origin)
        ax = " ".join(str(a) for a in lk.axis)
        lo, hi = lk.limits
        center, size = _bbox_m(lk.part)
        s = f'{pad}<body name="{lk.name}" pos="{x:.5f} {y:.5f} {z:.5f}">\n'
        if lk.has_joint:            # rigid (welded) links get no joint
            s += f'{pad}  <joint name="{lk.name}" type="hinge" axis="{ax}" range="{lo} {hi}"/>\n'
        s += f'{pad}  {_inertial(_link_mass(lk), center, size)}\n'
        # visual geoms (collision stays on the primitives added below / in root)
        if lk.name == HEAD_LINK:
            # head = clean opaque skin (its ears are sculpted in); no grey frame sphere
            s += (f'{pad}  <geom type="mesh" mesh="{SKIN_OVER[lk.name]}" '
                  f'contype="0" conaffinity="0" rgba="{HEAD_RGBA}"/>\n')
        elif lk.name in SKIP_VISUAL:
            pass                                    # ears shown by the head skin
        else:
            s += (f'{pad}  <geom type="mesh" mesh="{lk.name}" contype="0" '
                  f'conaffinity="0" rgba="{FRAME_RGBA}"/>\n')
            if lk.name in SKIN_OVER:                # torso: frame + translucent skin
                s += (f'{pad}  <geom type="mesh" mesh="{SKIN_OVER[lk.name]}" '
                      f'contype="0" conaffinity="0" rgba="{SKIN_RGBA}"/>\n')
        if lk.name.endswith("_ankle"):                       # toe pad = contact
            leg = lk.name[:2]
            fz = -P.m(P.leg_geom(leg)["foot"])
            s += (f'{pad}  <geom name="{lk.name}_toe" type="sphere" pos="0 0 {fz:.5f}" '
                  f'size="{P.m(P.TOE_R):.5f}" contype="1" conaffinity="1" '
                  f'friction="1.3 .05 .05" rgba=".9 .5 .25 1"/>\n')
        for ch in children.get(lk.name, []):
            s += emit_body(ch, indent + 2)
        s += f'{pad}</body>\n'
        return s

    fore_plastic = PRINTABLE["torso_fore"].volume * 1e-9 * _DENS
    fore_mass = fore_plastic + _fore_electronics()
    fcenter, fsize = _bbox_m(PRINTABLE["torso_fore"])

    # real meshes (inner frame + skin case) for visual geoms; mm→m via scale.
    meshdir = ensure_meshes().replace(os.sep, "/")
    assets = "\n".join(
        f'    <mesh name="{n}" file="{n}.stl" scale="0.001 0.001 0.001"/>'
        for n in all_mesh_names())

    xml = f'''<mujoco model="sabo">
  <compiler angle="radian" autolimits="true" meshdir="{meshdir}"/>
  <option gravity="0 0 -9.81" timestep="0.002" integrator="implicitfast"/>
  <default>
    <geom rgba="0.80 0.82 0.86 1"/>
    <joint damping="0.18" armature="0.012" frictionloss="0.002"/>
    <position kp="8" forcerange="{-SERVO.stall_nm:.3f} {SERVO.stall_nm:.3f}"/>
  </default>
  <asset>
{assets}
  </asset>
  <visual><global offwidth="900" offheight="700"/></visual>
  <worldbody>
    <light pos="0.3 -0.3 1.2" dir="-0.2 0.2 -1" diffuse="0.9 0.9 0.9"/>
    <camera name="cam" pos="0.02 -0.52 0.20" xyaxes="1 0 0 0 0.35 1" mode="trackcom"/>
    <geom name="floor" type="plane" size="2 2 0.1" rgba="0.18 0.20 0.25 1"
          contype="1" conaffinity="1" friction="1.3 .05 .05"/>
    <body name="{ROOT}" pos="0 0 {start_z:.4f}">
      <freejoint name="root"/>
      {_inertial(fore_mass, fcenter, fsize)}
      <geom name="torso_fore_col" type="box" pos="{fcenter[0]:.5f} {fcenter[1]:.5f} {fcenter[2]:.5f}"
            size="{fsize[0]/2:.5f} {fsize[1]/2:.5f} {fsize[2]/2:.5f}"
            contype="1" conaffinity="1" rgba="0 0 0 0"/>
      <geom type="mesh" mesh="torso_fore" contype="0" conaffinity="0" rgba="{FRAME_RGBA}"/>
      <geom type="mesh" mesh="skin_fore" contype="0" conaffinity="0" rgba="{SKIN_RGBA}"/>
      <geom type="mesh" mesh="skin_collar" contype="0" conaffinity="0" rgba="{SKIN_RGBA}"/>
'''
    for lk in children.get(ROOT, []):
        xml += emit_body(lk, 3)
    xml += "    </body>\n  </worldbody>\n"

    # mechanical couplings (knee->ankle tendon, linked ears) as equality constraints
    coupled = [lk for lk in links if lk.couple]
    if coupled:
        xml += "  <equality>\n"
        for lk in coupled:
            other, c0, c1 = lk.couple
            xml += (f'    <joint joint1="{lk.name}" joint2="{other}" '
                    f'polycoef="{c0:.5f} {c1:.5f} 0 0 0"/>\n')
        xml += "  </equality>\n"

    xml += "  <actuator>\n"
    for lk in links:
        if lk.actuated:
            lo, hi = lk.limits
            xml += f'    <position name="{lk.name}" joint="{lk.name}" ctrlrange="{lo} {hi}"/>\n'
    xml += "  </actuator>\n</mujoco>\n"
    return xml


if __name__ == "__main__":
    import mujoco
    model = mujoco.MjModel.from_xml_string(build_mjcf())
    print(f"MJCF OK — {model.nbody-1} bodies, {model.njnt} joints, "
          f"{model.nu} actuators, total mass {sum(model.body_mass):.3f} kg")
