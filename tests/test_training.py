"""RL training scaffold: URDF export + policy deploy stub."""

import xml.etree.ElementTree as ET

import numpy as np

from training.deploy_policy import (ACTION_JOINTS, LearnedGait, OBS_DIM,
                                    build_obs, default_pose)
from training.export_urdf import build_urdf
from sim.gait import ankle_from_knee, stance_angles


def test_urdf_wellformed_and_counts():
    root = ET.fromstring(build_urdf())
    assert root.tag == "robot"
    joints = root.findall("joint")
    rev = [j for j in joints if j.get("type") == "revolute"]
    fixed = [j for j in joints if j.get("type") == "fixed"]
    actuated = [j for j in rev if j.find("mimic") is None]
    mimic = [j for j in rev if j.find("mimic") is not None]
    assert len(actuated) == 14          # the 14 motors (incl. head pitch+tilt gimbal)
    assert len(fixed) == 4              # rigid abduction ×4
    assert len(mimic) == 5              # 4 hocks + linked ear_R
    # every revolute joint has finite limits + effort
    for j in rev:
        lim = j.find("limit")
        assert lim is not None and float(lim.get("effort")) > 0


def test_obs_dim_and_builder():
    obs = build_obs([0]*3, [0]*3, [0, 0, -1], [0.15, 0, 0],
                    [0]*9, [0]*9, [0]*9)
    assert obs.shape == (OBS_DIM,) == (39,)


def test_learned_gait_stub_holds_stance():
    g = LearnedGait(policy_path=None)      # no policy → safe stub
    assert not g.live
    targets = g.step(np.zeros(OBS_DIM, dtype=np.float32))
    # 9 actuated + 4 coupled ankles present
    assert len(targets) == len(ACTION_JOINTS) + 4
    # stub output == standing pose
    dp = default_pose()
    for leg in ("FL", "FR", "RL", "RR"):
        hip, knee = stance_angles(leg)
        assert abs(targets[f"{leg}_hip"] - hip) < 1e-6
        assert abs(targets[f"{leg}_knee"] - knee) < 1e-6
        # coupled hock follows the knee
        assert abs(targets[f"{leg}_ankle"] - ankle_from_knee(leg, knee)) < 1e-6
    assert abs(targets["torso_aft"]) < 1e-6
