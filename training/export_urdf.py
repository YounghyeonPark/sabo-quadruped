"""
URDF export — Sabo's kinematics → a URDF Isaac Sim / Isaac Lab can import.
=========================================================================

    python -m training.export_urdf         # writes training/sabo.urdf

Built from the same `cad.assembly.kinematics()` the MJCF uses, so the trained
robot matches the validated one. Units: metres / radians. Per-link inertials come
from the printed-part bounding boxes × real mass (plastic + the bought parts on
that link), matching `sim/mjcf.py`.

Coupled joints (the knee→hock reciprocal tendon, linked ears) are emitted with
URDF ``<mimic>`` tags. Importers that honour mimic (Gazebo, some Isaac paths) get
the coupling for free; if the Isaac importer ignores mimic, the RL env enforces
the same relation on the coupled joints (see `training/isaac/`). Rigid abduction
is a ``fixed`` joint.
"""

from __future__ import annotations

import os

from cad import params as P
from cad.assembly import PRINTABLE, ROOT, kinematics
from cad.servo import DEFAULT as SERVO

_DENS = P.EFFECTIVE_DENSITY
_VEL = (3.14159 / 3) / max(SERVO.speed_s_60, 1e-3)   # rad/s from s/60°


def _fore_electronics() -> float:
    c = P.COMPONENT_MASS
    return c["pi"] + c["imu"] + c["wiring_misc"]


def _bbox_m(part):
    bb = part.bounding_box()
    center = ((bb.min.X + bb.max.X) / 2000.0, (bb.min.Y + bb.max.Y) / 2000.0,
              (bb.min.Z + bb.max.Z) / 2000.0)
    size = (max(bb.size.X / 1000.0, 1e-3), max(bb.size.Y / 1000.0, 1e-3),
            max(bb.size.Z / 1000.0, 1e-3))
    return center, size


def _inertia_xml(mass, size):
    a, b, c = size
    ix, iy, iz = (mass/12*(b*b+c*c), mass/12*(a*a+c*c), mass/12*(a*a+b*b))
    return (f'<inertia ixx="{ix:.6e}" ixy="0" ixz="0" '
            f'iyy="{iy:.6e}" iyz="0" izz="{iz:.6e}"/>')


def _link_xml(name, part, mass) -> str:
    (cx, cy, cz), size = _bbox_m(part)
    box = f'{size[0]:.5f} {size[1]:.5f} {size[2]:.5f}'
    geom = (f'<geometry><box size="{box}"/></geometry>'
            f'<origin xyz="{cx:.5f} {cy:.5f} {cz:.5f}"/>')
    foot = ""
    if name.endswith("_ankle"):
        leg = name[:2]
        fz = -P.m(P.leg_geom(leg)["foot"])
        r = P.m(P.TOE_R)
        foot = (f'\n    <collision><origin xyz="0 0 {fz:.5f}"/>'
                f'<geometry><sphere radius="{r:.5f}"/></geometry></collision>')
    return f'''  <link name="{name}">
    <inertial><origin xyz="{cx:.5f} {cy:.5f} {cz:.5f}"/>
      <mass value="{max(mass,1e-3):.4f}"/>{_inertia_xml(mass, size)}</inertial>
    <visual>{geom}</visual>
    <collision>{geom}</collision>{foot}
  </link>'''


def _joint_xml(lk) -> str:
    x, y, z = (P.m(v) for v in lk.origin)
    origin = f'<origin xyz="{x:.5f} {y:.5f} {z:.5f}" rpy="0 0 0"/>'
    if not lk.has_joint:
        return f'''  <joint name="{lk.name}_fix" type="fixed">
    <parent link="{lk.parent}"/><child link="{lk.name}"/>{origin}</joint>'''
    ax = " ".join(str(a) for a in lk.axis)
    lo, hi = lk.limits
    mimic = ""
    if lk.couple:
        other, c0, c1 = lk.couple
        mimic = f'\n    <mimic joint="{other}" multiplier="{c1}" offset="{c0}"/>'
    return f'''  <joint name="{lk.name}" type="revolute">
    <parent link="{lk.parent}"/><child link="{lk.name}"/>{origin}
    <axis xyz="{ax}"/>
    <limit lower="{lo}" upper="{hi}" effort="{SERVO.stall_nm:.3f}" velocity="{_VEL:.2f}"/>{mimic}
  </joint>'''


def build_urdf() -> str:
    links = kinematics()
    fore_mass = PRINTABLE["torso_fore"].volume * 1e-9 * _DENS + _fore_electronics()
    parts = ['<?xml version="1.0"?>', '<robot name="sabo">']
    parts.append(_link_xml(ROOT, PRINTABLE["torso_fore"], fore_mass))
    for lk in links:
        parts.append(_link_xml(lk.name, lk.part,
                               lk.part.volume * 1e-9 * _DENS + lk.extra_mass))
        parts.append(_joint_xml(lk))
    parts.append("</robot>")
    return "\n".join(parts)


def main():
    urdf = build_urdf()
    out = os.path.join(os.path.dirname(__file__), "sabo.urdf")
    with open(out, "w") as f:
        f.write(urdf)
    # sanity: parse it back + count
    import xml.etree.ElementTree as ET
    root = ET.fromstring(urdf)
    links = root.findall("link")
    joints = root.findall("joint")
    rev = [j for j in joints if j.get("type") == "revolute"]
    actuated = [j for j in rev if j.find("mimic") is None]
    print(f"Wrote {out}")
    print(f"  {len(links)} links, {len(joints)} joints "
          f"({len(rev)} revolute, {len(actuated)} actuated, "
          f"{len(rev)-len(actuated)} coupled/mimic, "
          f"{len(joints)-len(rev)} fixed)")


if __name__ == "__main__":
    main()
