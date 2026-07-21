"""
Run the RoboKitten brain against the hardware backend.
======================================================

On the Jetson this drives the real robot; on the dev machine it runs in stub
mode (logging intended actuator writes) so you can smoke-test the wiring of
brain → HAL → hardware without any devices attached.

    python -m hardware.run_on_hardware            # a few stub ticks + summary
    python -m hardware.run_on_hardware --verbose  # show every stub servo write

The loop mirrors sim/runner.py's order but with NO simulator: perception reads
real (or stubbed) sensors, the brain decides, and Body verbs hit the STS3215 bus.

AI actuator loop (all actuator control is AI-driven, through the HAL):
  * the **behavior AI** (perception → mood → behavior → Expression) drives the
    head/eyes/ears/tail and issues locomotion *intents* via ``Body.gait()``;
  * the **learned RL gait policy** (``training.deploy_policy.LearnedGait``) turns
    those intents into per-leg joint targets. Pass ``--policy PATH.onnx`` to run
    it as the locomotion engine; with no path (or no onnxruntime) it holds the
    standing pose, so this stays a safe smoke test on the dev box.
"""

from __future__ import annotations

import argparse
import logging
import sys

import numpy as np

from brain.hal import Event, EventSink
from brain.robokitten import RoboKitten
from hardware import servo_channel_map as scm
from hardware.jetson_backend import HardwareBody, HardwareSenses
from training.deploy_policy import ACTION_JOINTS, LearnedGait, build_obs

# LearnedGait joint name → STS3215 bus servo name (ankles are coupled/passive; the
# policy's torso DOF is the waist servo). Only mapped joints are actuated.
_POLICY_TO_SERVO = {j: j for j in scm.SERVOS}     # FL_hip, FL_knee, ... match 1:1
_POLICY_TO_SERVO["torso_aft"] = "waist"


class ConsoleEvents(EventSink):
    """Print narratable events to stdout (stands in for the dashboard feed)."""

    def emit(self, event: Event) -> None:
        line = f"  [{event.t:8.3f}] {event.kind:8s} {event.text}"
        # Windows consoles default to cp1252 and choke on the brain's unicode
        # (e.g. the "→" in mood events); degrade gracefully rather than crash.
        enc = getattr(sys.stdout, "encoding", None) or "utf-8"
        print(line.encode(enc, errors="replace").decode(enc))


def drive_locomotion(body: HardwareBody, senses: HardwareSenses,
                     gait: LearnedGait) -> None:
    """Learned RL gait policy → per-leg servo targets (the AI locomotion path).

    Builds the policy observation from the brain's velocity *intent*
    (``body.cmd_forward``/``cmd_yaw``) plus the live IMU (orientation → projected
    gravity), runs the policy, and writes the joint targets straight to the
    STS3215 bus through ``HardwareBody._write_servo`` (stub-logged off-hardware).
    With no ONNX policy the stub returns the standing pose, so the robot stands.
    """
    imu = senses.imu()
    # projected gravity in the body frame: level → straight down; tilt leans it.
    proj_g = [np.sin(imu.tilt), 0.0, -np.cos(imu.tilt)]
    command = [body.cmd_forward, 0.0, body.cmd_yaw]
    zeros9 = np.zeros(9, dtype=np.float32)
    obs = build_obs(base_lin_vel=[body.cmd_forward, 0.0, 0.0],
                    base_ang_vel=[0.0, 0.0, body.cmd_yaw],
                    projected_gravity=proj_g, command=command,
                    joint_pos_rel=zeros9, joint_vel=zeros9,
                    last_action=gait.last_action)
    for joint, angle in gait.step(obs).items():
        servo = _POLICY_TO_SERVO.get(joint)
        if servo is not None:                 # skip coupled ankles (no servo)
            body._write_servo(servo, angle)


def main() -> None:
    ap = argparse.ArgumentParser(description=__doc__)
    ap.add_argument("--ticks", type=int, default=10, help="number of brain ticks")
    ap.add_argument("--dt", type=float, default=0.1, help="seconds per tick")
    ap.add_argument("--policy", type=str, default=None,
                    help="ONNX RL gait policy to drive locomotion (LearnedGait)")
    ap.add_argument("--verbose", action="store_true",
                    help="log every stub servo/LED write")
    args = ap.parse_args()

    logging.basicConfig(
        level=logging.DEBUG if args.verbose else logging.INFO,
        format="%(levelname)-7s %(name)s: %(message)s",
    )

    body = HardwareBody()
    senses = HardwareSenses()
    events = ConsoleEvents()

    gait = LearnedGait(args.policy)
    mode = "LIVE hardware" if body.live else "STUB (no hardware present)"
    locomotion = ("learned RL policy" if gait.live
                  else "learned-gait stub (standing pose)")
    print(f"\nRunning RoboKitten on backend: {mode}")
    print(f"Locomotion engine: {locomotion}")
    print(f"Ticks: {args.ticks}  dt: {args.dt}s\n")

    kitten = RoboKitten(body, senses, events)
    for i in range(args.ticks):
        snap = kitten.tick(args.dt)
        # AI actuator loop: behavior AI set the gait *intent* on Body this tick;
        # the learned RL policy now turns it into leg-joint targets on the STS3215 bus.
        drive_locomotion(body, senses, gait)
        print(f"tick {i:2d}: mood={snap['mood']:7s} behavior={snap['behavior']}")

    print("\nFinal body state:")
    print(f"  gait={body.gait_mode.value} forward={body.cmd_forward:.3f} "
          f"yaw={body.cmd_yaw:.3f}")
    print(f"  posture front={body.front_height:.2f} rear={body.rear_height:.2f}  "
          f"head_pan={body.head_pan:.2f} head_tilt={body.head_tilt:.2f}")
    print(f"  ears={body.ears.value} tail={body.tail.value} "
          f"wag={body.tail_wag:.2f} eyes={body.eyes_open:.2f} purr={body.purring}")
    print("\nOK — brain ran cleanly against the hardware backend.")


if __name__ == "__main__":
    main()
