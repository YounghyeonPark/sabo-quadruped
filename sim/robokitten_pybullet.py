"""
RoboKitten — PyBullet physics simulation (Phase-0 engineering model)
====================================================================

A kitten-scale 12-DOF quadruped (3 joints/leg: abduction, hip, knee) with
analytic leg inverse-kinematics and a phase-based trot/walk gait. The joint
layout and gait code are meant to transfer to Mini Pupper-class hardware:
the same foot-trajectory + IK you tune here becomes the servo targets on
the real robot (swap `set_joint()` for a PCA9685 servo write).

Run
---
    pip install pybullet numpy
    python robokitten_pybullet.py

A GUI window opens with debug sliders:
    mode        0=stand  1=walk  2=trot
    gait speed  cycle-time scale
    step height foot lift during swing
    body height stance height

Controls the same behavior vocabulary as the browser sim, but with real
physics — so you can see whether a gait actually keeps its balance.
"""

import math
import os
import tempfile
import time

import numpy as np
import pybullet as p
import pybullet_data

# ---------------------------------------------------------------- dimensions
# metres — roughly a 3-4 month kitten / Mini Pupper scale
BODY = (0.20, 0.11, 0.05)        # length (x), width (y), height (z)
BODY_MASS = 0.55
L_HIP = 0.035                    # abduction offset (hip -> leg plane)
L_THIGH = 0.06
L_CALF = 0.065
LEG_MASS = 0.04

# leg mount points on the body (x forward, y left), corner offsets
MOUNTS = {
    "FL": (+BODY[0] / 2 - 0.01, +BODY[1] / 2),
    "FR": (+BODY[0] / 2 - 0.01, -BODY[1] / 2),
    "RL": (-BODY[0] / 2 + 0.01, +BODY[1] / 2),
    "RR": (-BODY[0] / 2 + 0.01, -BODY[1] / 2),
}
LEGS = ["FL", "FR", "RL", "RR"]


# ------------------------------------------------------------------- URDF gen
def build_urdf() -> str:
    """Generate a 12-DOF quadruped URDF and return its file path."""

    def inertia(m, r=0.02):
        i = 0.4 * m * r * r
        return f'<inertia ixx="{i}" ixy="0" ixz="0" iyy="{i}" iyz="0" izz="{i}"/>'

    def box_link(name, sx, sy, sz, mass, rgba):
        return f'''
  <link name="{name}">
    <visual><geometry><box size="{sx} {sy} {sz}"/></geometry>
      <material name="{name}_m"><color rgba="{rgba}"/></material></visual>
    <collision><geometry><box size="{sx} {sy} {sz}"/></geometry></collision>
    <inertial><mass value="{mass}"/>{inertia(mass)}</inertial>
  </link>'''

    def cyl_link(name, length, mass, rgba):
        # leg segment drawn along its local z, shifted so joint sits at top
        return f'''
  <link name="{name}">
    <visual><origin xyz="0 0 {-length/2}"/>
      <geometry><cylinder radius="0.008" length="{length}"/></geometry>
      <material name="{name}_m"><color rgba="{rgba}"/></material></visual>
    <collision><origin xyz="0 0 {-length/2}"/>
      <geometry><cylinder radius="0.008" length="{length}"/></geometry></collision>
    <inertial><origin xyz="0 0 {-length/2}"/><mass value="{mass}"/>{inertia(mass)}</inertial>
  </link>'''

    def joint(name, parent, child, xyz, axis, lower=-2.6, upper=2.6):
        return f'''
  <joint name="{name}" type="revolute">
    <parent link="{parent}"/><child link="{child}"/>
    <origin xyz="{xyz}"/><axis xyz="{axis}"/>
    <limit lower="{lower}" upper="{upper}" effort="4" velocity="12"/>
  </joint>'''

    fur = "0.80 0.82 0.86 1"
    accent = "0.85 0.45 0.20 1"

    urdf = ['<?xml version="1.0"?>', '<robot name="robokitten">']
    urdf.append(box_link("base", *BODY, BODY_MASS, fur))

    for leg in LEGS:
        mx, my = MOUNTS[leg]
        sign = 1 if my > 0 else -1
        # abduction hub (rotates about x -> swings leg plane sideways)
        urdf.append(box_link(f"{leg}_hub", 0.02, 0.02, 0.02, LEG_MASS, accent))
        urdf.append(joint(f"{leg}_abd", "base", f"{leg}_hub",
                          f"{mx} {my} 0", "1 0 0", -0.9, 0.9))
        # thigh (rotates about y -> hip pitch)
        urdf.append(cyl_link(f"{leg}_thigh", L_THIGH, LEG_MASS, fur))
        urdf.append(joint(f"{leg}_hip", f"{leg}_hub", f"{leg}_thigh",
                          f"0 {sign*L_HIP} 0", "0 1 0"))
        # calf (rotates about y -> knee)
        urdf.append(cyl_link(f"{leg}_calf", L_CALF, LEG_MASS, fur))
        urdf.append(joint(f"{leg}_knee", f"{leg}_thigh", f"{leg}_calf",
                          f"0 0 {-L_THIGH}", "0 1 0", 0.0, 2.6))

    urdf.append("</robot>")
    path = os.path.join(tempfile.gettempdir(), "robokitten.urdf")
    with open(path, "w") as f:
        f.write("\n".join(urdf))
    return path


# ---------------------------------------------------------------------- IK
def leg_ik(x, y, z):
    """Foot target (m) in the leg's hip frame -> (abduction, hip, knee) rad.

    x: forward   y: outward (+ = away from body)   z: up (negative = down)
    Sagittal 2-link IK for hip+knee; abduction handles lateral offset.
    """
    # abduction about x from the y/z projection, accounting for hip offset
    dyz = math.hypot(y, z)
    dyz = max(dyz, L_HIP + 1e-4)
    inner = math.sqrt(max(dyz * dyz - L_HIP * L_HIP, 0.0))
    abd = math.atan2(y, -z) - math.atan2(L_HIP, inner)

    # remaining reach lies in the sagittal plane: forward x, downward `inner`
    L = math.hypot(x, inner)
    L = min(L, L_THIGH + L_CALF - 1e-4)
    L = max(L, abs(L_THIGH - L_CALF) + 1e-4)

    # knee interior angle (cosine rule); >0 keeps a consistent bend
    ck = (L_THIGH ** 2 + L_CALF ** 2 - L * L) / (2 * L_THIGH * L_CALF)
    knee_interior = math.acos(max(-1, min(1, ck)))
    knee = math.pi - knee_interior            # joint angle (0 = straight)

    # hip pitch = point-down-and-forward angle minus the thigh-to-line angle
    alpha = math.atan2(x, inner)              # forward lean of the leg line
    cb = (L_THIGH ** 2 + L * L - L_CALF ** 2) / (2 * L_THIGH * L)
    beta = math.acos(max(-1, min(1, cb)))
    hip = alpha - beta
    return abd, hip, knee


# ------------------------------------------------------------------- gait
# trot: diagonal pairs move together, half-cycle apart
PHASE = {"FL": 0.0, "RR": 0.0, "FR": 0.5, "RL": 0.5}
STANCE_X = {"FL": 0.0, "FR": 0.0, "RL": 0.0, "RR": 0.0}   # neutral fore/aft


def foot_target(leg, phase, stride, step_h, body_h):
    """Cyclic foot trajectory in the leg frame for a given gait phase."""
    ph = (phase + PHASE[leg]) % 1.0
    duty = 0.5
    if ph < duty:                                   # stance: planted, slides back
        s = ph / duty
        x = stride / 2 - stride * s
        z = -body_h
    else:                                           # swing: lift + return forward
        s = (ph - duty) / (1 - duty)
        x = -stride / 2 + stride * s
        z = -body_h + step_h * math.sin(math.pi * s)
    y = L_HIP + 0.005                               # slight outward stance
    return STANCE_X[leg] + x, y, z


# --------------------------------------------------------------------- main
def main():
    cid = p.connect(p.GUI)
    p.setAdditionalSearchPath(pybullet_data.getDataPath())
    p.setGravity(0, 0, -9.81)
    p.setTimeStep(1 / 240)
    p.configureDebugVisualizer(p.COV_ENABLE_GUI, 0)
    p.resetDebugVisualizerCamera(0.6, 50, -20, (0, 0, 0.05))

    p.loadURDF("plane.urdf")
    robot = p.loadURDF(build_urdf(), (0, 0, 0.14))

    # map joint names -> indices
    jidx = {}
    for i in range(p.getNumJoints(robot)):
        jidx[p.getJointInfo(robot, i)[1].decode()] = i

    def set_joint(name, angle):
        p.setJointMotorControl2(robot, jidx[name], p.POSITION_CONTROL,
                                targetPosition=angle, force=3.5, maxVelocity=12)

    # knee joint direction differs by build; flip here if legs bend wrong
    KNEE_SIGN, HIP_SIGN = 1.0, 1.0

    def command_leg(leg, x, y, z):
        # left/right mirror the outward (+y) direction
        ysign = 1 if leg[-1] == "L" else -1
        abd, hip, knee = leg_ik(x, y * ysign, z)
        set_joint(f"{leg}_abd", abd * ysign)
        set_joint(f"{leg}_hip", hip * HIP_SIGN)
        set_joint(f"{leg}_knee", knee * KNEE_SIGN)

    # debug sliders
    s_mode = p.addUserDebugParameter("mode 0stand 1walk 2trot", 0, 2, 2)
    s_spd = p.addUserDebugParameter("gait speed", 0.3, 2.0, 1.0)
    s_step = p.addUserDebugParameter("step height", 0.0, 0.05, 0.03)
    s_bh = p.addUserDebugParameter("body height", 0.06, 0.12, 0.095)

    # settle onto feet
    for leg in LEGS:
        command_leg(leg, 0, L_HIP, -0.095)
    for _ in range(120):
        p.stepSimulation()

    phase = 0.0
    dt = 1 / 240
    print("RoboKitten sim running — drag sliders, close window to quit.")
    while p.isConnected():
        mode = round(p.readUserDebugParameter(s_mode))
        spd = p.readUserDebugParameter(s_spd)
        step_h = p.readUserDebugParameter(s_step)
        body_h = p.readUserDebugParameter(s_bh)

        if mode == 0:                       # stand
            for leg in LEGS:
                command_leg(leg, 0, L_HIP, -body_h)
        else:
            stride = 0.05 if mode == 1 else 0.07
            cycle = (1.1 if mode == 1 else 0.6) / spd
            phase = (phase + dt / cycle) % 1.0
            for leg in LEGS:
                x, y, z = foot_target(leg, phase, stride, step_h, body_h)
                command_leg(leg, x, y, z)

        p.stepSimulation()
        time.sleep(dt)

    p.disconnect()


if __name__ == "__main__":
    main()
