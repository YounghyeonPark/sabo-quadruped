"""
Isaac Lab locomotion task for Sabo — velocity-tracking RL.
==========================================================

Reference/scaffold — this module imports ``isaaclab`` and is meant to run inside
an Isaac Lab install (Linux/WSL2 on the RTX 4090), NOT on the dev box. It defines
a manager-based RL env that trains a policy to track a commanded body velocity
(forward + yaw) on Sabo's underactuated body.

Sabo specifics baked in:
  * **Action = 8 leg joints** (FL/FR/RL/RR × hip,knee) + **1 waist** = 9 target
    angles. Ankles are NOT actions — they follow the knee via the reciprocal
    coupling; if the USD import drops the mimic, re-impose it in a pre-physics
    callback (see ``_enforce_coupling``). Abduction is fixed (no action).
  * No abduction ⇒ the policy must learn to **turn by gait** — hence a yaw
    command term and an ang-vel-tracking reward.
  * Servo limits (stall torque, speed) come from the URDF and cap the actuator.

Tested numbers/shapes are documented in ``training/README.md``. Reconcile class
paths with your installed Isaac Lab version (this follows the ~1.2/2.0 API).
"""

from __future__ import annotations

import math

import isaaclab.sim as sim_utils
from isaaclab.assets import ArticulationCfg
from isaaclab.envs import ManagerBasedRLEnvCfg
from isaaclab.managers import (EventTermCfg as EventTerm,
                               ObservationGroupCfg as ObsGroup,
                               ObservationTermCfg as ObsTerm,
                               RewardTermCfg as RewTerm,
                               SceneEntityCfg,
                               TerminationTermCfg as DoneTerm)
from isaaclab.scene import InteractiveSceneCfg
from isaaclab.terrains import TerrainImporterCfg
from isaaclab.utils import configclass
import isaaclab.envs.mdp as mdp

# The 9 actuated DOF the policy commands (ankles are coupled, abduction fixed).
ACTION_JOINTS = [f"{leg}_{j}" for leg in ("FL", "FR", "RL", "RR")
                 for j in ("hip", "knee")] + ["torso_aft"]   # torso_aft = waist
# Convert the exported URDF → USD once with Isaac's urdf importer; point here:
SABO_USD = "training/isaac/sabo.usd"


# --------------------------------------------------------------------- robot
SABO_CFG = ArticulationCfg(
    spawn=sim_utils.UsdFileCfg(
        usd_path=SABO_USD,
        rigid_props=sim_utils.RigidBodyPropertiesCfg(
            max_depenetration_velocity=1.0),
        articulation_props=sim_utils.ArticulationRootPropertiesCfg(
            enabled_self_collisions=False, solver_position_iteration_count=8),
    ),
    init_state=ArticulationCfg.InitialStateCfg(
        pos=(0.0, 0.0, 0.11),            # drop onto its toes; ~torso height
        joint_pos={".*_hip": -0.9, ".*_knee": 1.7, ".*_ankle": -0.2,
                   "torso_aft": 0.0, "head.*": 0.0, "ear.*": 0.0, "tail": 0.0},
    ),
    # position servos matching cad/servo.py (stall ≈1.96 N·m); PD tuned in sim-to-real.
    actuators={"servos": sim_utils.ImplicitActuatorCfg(
        joint_names_expr=[".*_hip", ".*_knee", "torso_aft"],
        effort_limit=1.96, velocity_limit=10.0, stiffness=8.0, damping=0.3)},
)


# --------------------------------------------------------------------- scene
@configclass
class SaboSceneCfg(InteractiveSceneCfg):
    terrain = TerrainImporterCfg(prim_path="/World/ground", terrain_type="plane")
    robot: ArticulationCfg = SABO_CFG.replace(prim_path="{ENV_REGEX_NS}/Robot")
    # contact sensor on the toes for foot-air-time reward + fall detection
    # (add ContactSensorCfg on .*_ankle in your version's sensors module)


# --------------------------------------------------------------------- commands
@configclass
class CommandsCfg:
    base_velocity = mdp.UniformVelocityCommandCfg(
        asset_name="robot", resampling_time_range=(6.0, 6.0),
        rel_standing_envs=0.15, heading_command=False,
        ranges=mdp.UniformVelocityCommandCfg.Ranges(
            lin_vel_x=(-0.1, 0.25), lin_vel_y=(0.0, 0.0),   # no strafe (no abduction)
            ang_vel_z=(-0.8, 0.8)))                          # learn to turn by gait


@configclass
class ActionsCfg:
    joint_pos = mdp.JointPositionActionCfg(
        asset_name="robot", joint_names=ACTION_JOINTS, scale=0.5, use_default_offset=True)


# --------------------------------------------------------------------- observations
@configclass
class ObservationsCfg:
    @configclass
    class PolicyCfg(ObsGroup):
        base_lin_vel = ObsTerm(func=mdp.base_lin_vel)
        base_ang_vel = ObsTerm(func=mdp.base_ang_vel)
        projected_gravity = ObsTerm(func=mdp.projected_gravity)   # "which way is up"
        velocity_commands = ObsTerm(func=mdp.generated_commands,
                                    params={"command_name": "base_velocity"})
        joint_pos = ObsTerm(func=mdp.joint_pos_rel)
        joint_vel = ObsTerm(func=mdp.joint_vel_rel)
        actions = ObsTerm(func=mdp.last_action)

        def __post_init__(self):
            self.enable_corruption = True     # sensor noise → sim-to-real robustness
            self.concatenate_terms = True

    policy: PolicyCfg = PolicyCfg()


# --------------------------------------------------------------------- rewards
@configclass
class RewardsCfg:
    # track the command (the point of the task)
    track_lin_vel_xy = RewTerm(func=mdp.track_lin_vel_xy_exp, weight=1.5,
                               params={"command_name": "base_velocity", "std": 0.25})
    track_ang_vel_z = RewTerm(func=mdp.track_ang_vel_z_exp, weight=0.75,
                              params={"command_name": "base_velocity", "std": 0.25})
    # stay a stable, upright, level cat
    flat_orientation = RewTerm(func=mdp.flat_orientation_l2, weight=-2.0)
    base_height = RewTerm(func=mdp.base_height_l2, weight=-1.0,
                          params={"target_height": 0.095})
    lin_vel_z = RewTerm(func=mdp.lin_vel_z_l2, weight=-1.5)       # no bouncing
    ang_vel_xy = RewTerm(func=mdp.ang_vel_xy_l2, weight=-0.05)    # no wobble/roll
    # smooth, cheap, quiet (servo life + cat-friendly)
    joint_torques = RewTerm(func=mdp.joint_torques_l2, weight=-2.0e-4)
    action_rate = RewTerm(func=mdp.action_rate_l2, weight=-0.01)
    dof_acc = RewTerm(func=mdp.joint_acc_l2, weight=-2.5e-7)
    alive = RewTerm(func=mdp.is_alive, weight=0.5)
    # (add feet_air_time with the toe contact sensor to encourage a real gait)


# --------------------------------------------------------------------- terminations
@configclass
class TerminationsCfg:
    time_out = DoneTerm(func=mdp.time_out, time_out=True)
    fell_over = DoneTerm(func=mdp.bad_orientation, params={"limit_angle": 1.0})


# --------------------------------------------------------------------- domain randomization
@configclass
class EventsCfg:
    physics_material = EventTerm(func=mdp.randomize_rigid_body_material, mode="startup",
        params={"asset_cfg": SceneEntityCfg("robot", body_names=".*_ankle"),
                "static_friction_range": (0.6, 1.4), "dynamic_friction_range": (0.5, 1.2),
                "restitution_range": (0.0, 0.1), "num_buckets": 64})
    add_base_mass = EventTerm(func=mdp.randomize_rigid_body_mass, mode="startup",
        params={"asset_cfg": SceneEntityCfg("robot", body_names="torso_fore"),
                "mass_distribution_params": (-0.1, 0.15), "operation": "add"})
    push_robot = EventTerm(func=mdp.push_by_setting_velocity, mode="interval",
        interval_range_s=(8.0, 12.0),
        params={"velocity_range": {"x": (-0.3, 0.3), "y": (-0.3, 0.3)}})


# --------------------------------------------------------------------- env
@configclass
class SaboLocomotionEnvCfg(ManagerBasedRLEnvCfg):
    scene: SaboSceneCfg = SaboSceneCfg(num_envs=4096, env_spacing=1.5)
    commands: CommandsCfg = CommandsCfg()
    actions: ActionsCfg = ActionsCfg()
    observations: ObservationsCfg = ObservationsCfg()
    rewards: RewardsCfg = RewardsCfg()
    terminations: TerminationsCfg = TerminationsCfg()
    events: EventsCfg = EventsCfg()

    def __post_init__(self):
        self.decimation = 4               # 50 Hz policy over 200 Hz sim
        self.sim.dt = 0.005
        self.episode_length_s = 20.0
        self.viewer.eye = (1.5, 1.5, 0.6)


def _enforce_coupling(env):
    """If the USD import dropped the URDF <mimic>, call this each pre-physics step
    to re-impose ankle = c0 + c1·knee (the reciprocal-apparatus tendon) and
    ear_R = ear_L. Use sim.gait.ankle_couple_coef for (c0, c1) per leg."""
    # from sim.gait import ankle_couple_coef  # (import on the training host)
    ...
