# Sabo — RL locomotion training (Isaac Lab)

Train a locomotion **policy** (small MLP: state → 9 joint targets) that makes
Sabo's underactuated body (2 motors/leg, coupled hock, no abduction) walk, trot,
and **turn by gait** robustly — then transfer it to the Jetson. The learned
policy is a drop-in replacement for the hand-authored gait in `sim/gait.py`.

## Pipeline

```
cad.assembly.kinematics ──▶ training/export_urdf.py ──▶ sabo.urdf
   ──(Isaac urdf importer)──▶ sabo.usd
   ──(Isaac Lab, RTX 4090, 4096 envs)──▶ policy.pt   [isaac/sabo_locomotion_env.py]
   ──(export)──▶ policy.onnx
   ──(validate in MuJoCo)──▶ python -m sim.mj_emulate ... (LearnedGait)   [deploy_policy.py]
   ──(build on Jetson)──▶ policy.engine  ──▶ HardwareBody gait engine
```

## Why RL here (not just for show)
Sabo is **underactuated** — no abduction joint, hock coupled to the knee. That's
exactly where hand-authored gaits struggle (our trot needed babying; turning is
gait-only). RL with domain randomization discovers robust gaits + turning for
awkward morphologies, and the reward penalizes roll/bounce — so it should also
cut the **body wobble** that shakes the camera.

## Setup (on the RTX 4090; Linux or WSL2 recommended)
1. Install Isaac Sim + **Isaac Lab** (follow NVIDIA's installer; needs the RTX GPU).
2. `python -m training.export_urdf` → `training/sabo.urdf`.
3. Import URDF → USD with Isaac's `urdf` importer; save as `training/isaac/sabo.usd`.
   Check the **mimic** joints survived (ankle↔knee); if not, `_enforce_coupling`
   in the env re-imposes ankle = c0 + c1·knee (coefs from `sim.gait.ankle_couple_coef`).
4. Register `SaboLocomotionEnvCfg` as a Gym task and train (rsl_rl / skrl PPO):
   ```bash
   python scripts/rsl_rl/train.py --task Sabo-Locomotion-v0 --num_envs 4096 --headless
   ```

## Task (`isaac/sabo_locomotion_env.py`)
- **Action (9):** FL/FR/RL/RR × (hip, knee) + waist. Ankles coupled; abduction fixed.
- **Command:** forward `lin_vel_x ∈ [-0.1, 0.25] m/s`, `ang_vel_z ∈ [-0.8, 0.8]`,
  no strafe (no abduction).
- **Observation (39):** base lin/ang vel, projected gravity, command, joint
  pos/vel (9), last action (9).

| Reward term | weight | why |
|---|---|---|
| track_lin_vel_xy / track_ang_vel_z | +1.5 / +0.75 | follow the command |
| flat_orientation / base_height | −2.0 / −1.0 | upright, ~95 mm tall |
| lin_vel_z / ang_vel_xy | −1.5 / −0.05 | no bounce / **no wobble** |
| joint_torques / action_rate / dof_acc | −2e-4 / −1e-2 / −2.5e-7 | smooth, quiet, servo-safe |
| alive | +0.5 | don't fall |

**Domain randomization:** toe friction, base mass (±), random pushes — the
sim-to-real bridge. Servo effort/velocity limits come from the URDF (cap = real
servo), so the policy can't learn torques the hardware can't deliver.

## Deploy (`deploy_policy.py`)
- **Validate in MuJoCo first:** `LearnedGait("policy.onnx")` → `mj_control()` gives a
  `control_fn` for `sim/mj_emulate.py`. Confirm it stays upright/tracks before hardware.
- **On the Jetson:** build `policy.engine` (TensorRT) and run `LearnedGait` as the
  gait engine behind `HardwareBody.gait()` (the current TODO there) — obs from
  `HardwareSenses` + IMU, output → the 8 leg + waist servos; hocks follow via coupling.
- No policy / no onnxruntime → **stub holds the standing pose** (safe default), so
  everything imports and runs on the dev box.

## Alternative
If the USD/PhysX round-trip or WSL2 setup is friction, **MJX** (MuJoCo-native,
JAX) trains in the exact model we validated — same `deploy_policy.py` output. See
the toolchain comparison in chat; this scaffold's URDF + task/reward/DR translate.
