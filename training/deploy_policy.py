"""
Policy deploy — run a trained locomotion policy as Sabo's gait engine.
======================================================================

A trained Isaac Lab policy is a small MLP: observation → 9 joint targets (8 leg +
waist). This wraps it as ``LearnedGait``, a **drop-in replacement for the
hand-authored gait** (`sim/gait.py`) that plugs in at the same seam:
  * in sim — as a ``control_fn`` for `sim/mj_emulate.py`;
  * on hardware — as the gait engine behind `HardwareBody.gait()`.

The ONNX policy is loaded via onnxruntime (guarded). With no policy / no
onnxruntime it runs a **safe stub** that outputs the standing pose — so this
module imports and runs on the dev box and the robot just stands rather than
flailing.

Observation layout MUST match the training env (`isaac/sabo_locomotion_env.py`):
    [ base_lin_vel(3), base_ang_vel(3), projected_gravity(3),
      velocity_command(3), joint_pos_rel(9), joint_vel(9), last_action(9) ]  = 39
"""

from __future__ import annotations

import numpy as np

from sim.gait import ankle_from_knee, stance_angles

# the 9 actuated DOF the policy controls, in a fixed order (== training order)
ACTION_JOINTS = [f"{leg}_{j}" for leg in ("FL", "FR", "RL", "RR")
                 for j in ("hip", "knee")] + ["torso_aft"]
ACTION_SCALE = 0.5          # matches JointPositionActionCfg.scale
OBS_DIM = 3 + 3 + 3 + 3 + 9 + 9 + 9


def default_pose() -> dict[str, float]:
    """Standing target for each actuated joint (policy offset / stub output)."""
    pose = {}
    for leg in ("FL", "FR", "RL", "RR"):
        hip, knee = stance_angles(leg)
        pose[f"{leg}_hip"], pose[f"{leg}_knee"] = hip, knee
    pose["torso_aft"] = 0.0
    return pose


def build_obs(base_lin_vel, base_ang_vel, projected_gravity, command,
              joint_pos_rel, joint_vel, last_action) -> np.ndarray:
    """Assemble the 39-d observation in the trained layout."""
    return np.concatenate([
        np.asarray(base_lin_vel, dtype=np.float32).reshape(3),
        np.asarray(base_ang_vel, dtype=np.float32).reshape(3),
        np.asarray(projected_gravity, dtype=np.float32).reshape(3),
        np.asarray(command, dtype=np.float32).reshape(3),
        np.asarray(joint_pos_rel, dtype=np.float32).reshape(9),
        np.asarray(joint_vel, dtype=np.float32).reshape(9),
        np.asarray(last_action, dtype=np.float32).reshape(9),
    ]).astype(np.float32)


class LearnedGait:
    """Loads an ONNX policy (if available) and maps obs → joint targets."""

    def __init__(self, policy_path: str | None = None):
        self._sess = None
        self._in = None
        self._default = default_pose()
        self._last_action = np.zeros(len(ACTION_JOINTS), dtype=np.float32)
        if policy_path:
            try:
                import onnxruntime as ort
                self._sess = ort.InferenceSession(
                    policy_path, providers=["CPUExecutionProvider"])
                self._in = self._sess.get_inputs()[0].name
            except Exception:
                self._sess = None      # no onnxruntime / bad file → stub

    @property
    def live(self) -> bool:
        return self._sess is not None

    def reset(self):
        self._last_action[:] = 0.0

    def _infer(self, obs: np.ndarray) -> np.ndarray:
        if self._sess is None:
            return np.zeros(len(ACTION_JOINTS), dtype=np.float32)   # stub → default pose
        out = self._sess.run(None, {self._in: obs.reshape(1, -1)})[0]
        return np.asarray(out, dtype=np.float32).reshape(-1)[:len(ACTION_JOINTS)]

    def step(self, obs: np.ndarray) -> dict[str, float]:
        """obs (39,) → {joint: target_rad} for the 9 actuated DOF + coupled ankles."""
        action = self._infer(obs)
        self._last_action = action
        targets: dict[str, float] = {}
        for i, j in enumerate(ACTION_JOINTS):
            targets[j] = self._default[j] + ACTION_SCALE * float(action[i])
        # coupled hock follows the commanded knee (reciprocal tendon)
        for leg in ("FL", "FR", "RL", "RR"):
            targets[f"{leg}_ankle"] = ankle_from_knee(leg, targets[f"{leg}_knee"])
        return targets

    @property
    def last_action(self) -> np.ndarray:
        return self._last_action.copy()


def mj_control(policy_path: str | None = None):
    """Return a control_fn(rig, t) for sim/mj_emulate that drives Sabo with the
    policy — validate a trained policy in MuJoCo before flashing the Jetson.

    Command is a gentle forward walk; obs is read from the MuJoCo state. With no
    policy the stub holds stance (robot stands), which is a useful smoke test."""
    gaiteng = LearnedGait(policy_path)
    command = np.array([0.15, 0.0, 0.0], dtype=np.float32)   # forward 0.15 m/s

    def control_fn(rig, t):
        d = rig.data
        R = d.xmat[rig.torso].reshape(3, 3)
        proj_g = R.T @ np.array([0.0, 0.0, -1.0])
        jp = np.array([_qpos(rig, j) for j in ACTION_JOINTS], dtype=np.float32)
        jp_rel = jp - np.array([gaiteng._default[j] for j in ACTION_JOINTS], np.float32)
        jv = np.array([_qvel(rig, j) for j in ACTION_JOINTS], dtype=np.float32)
        obs = build_obs(d.qvel[0:3], d.qvel[3:6], proj_g, command,
                        jp_rel, jv, gaiteng.last_action)
        for j, ang in gaiteng.step(obs).items():
            rig.set_target(j, ang)

    return control_fn


def _qpos(rig, joint):
    return float(rig.data.qpos[rig.qadr[joint]])


def _qvel(rig, joint):
    import mujoco
    jid = mujoco.mj_name2id(rig.model, mujoco.mjtObj.mjOBJ_JOINT, joint)
    return float(rig.data.qvel[rig.model.jnt_dofadr[jid]])
