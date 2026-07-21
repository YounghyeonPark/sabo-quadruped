"""
Brain bridge — run the Phase-0 brain on the physical MuJoCo model.
==================================================================

Closes the loop the whole architecture was designed for: the *same*
``brain/`` package (perception → mood → behavior → expression) that drove the 2-D
toy now drives the physics-accurate printed robot, through a MuJoCo-backed HAL.

    MujocoBody   — turns the brain's high-level commands (gait intent, posture,
                   head/ears/tail) into leg IK + joint targets on the model.
    MujocoSenses — feeds the brain a scripted cat encounter so it has something to
                   react to (approach → engage → lunge), plus the real IMU.

Watch the physical kitten play-bow, trot after the "cat", and freeze — all decided
by the unchanged Phase-0 brain.
"""

from __future__ import annotations

import math

from brain.hal import (BlinkKind, Body, CatDetection, EarPose, Event, EventSink,
                       Gait, ImuReading, ProximityReading, Senses, TailPose)
from brain.robokitten import RoboKitten
from cad import params as P
from sim import gait as gaitmod

_EAR = {EarPose.FORWARD: 0.45, EarPose.NEUTRAL: 0.0, EarPose.FLAT: -0.5}
_TAIL = {TailPose.UP: 0.7, TailPose.MID: 0.0, TailPose.LOW: -0.5, TailPose.PUFFED: 0.9}


class _Clock:
    def __init__(self): self.t = 0.0
    def __call__(self): return self.t


class _Printer(EventSink):
    def emit(self, e: Event) -> None:
        print(f"    {e.t:5.1f}s [{e.kind}] {e.text}")


class MujocoBody(Body):
    """Captures the brain's commands; the controller reads them each tick."""
    def __init__(self):
        self.gait_mode = Gait.STAND
        self.forward = 0.0
        self.yaw = 0.0
        self.fh = 1.0
        self.rh = 1.0
        self.head = 0.0
        self.ears = EarPose.NEUTRAL
        self.tail = TailPose.MID

    def gait(self, mode, forward=0.0, yaw=0.0):
        self.gait_mode, self.forward, self.yaw = mode, forward, yaw
    def look_at(self, bearing, tilt=0.0): self.head = max(-1.2, min(1.2, bearing))
    def blink(self, kind): pass
    def set_eyes(self, openness): pass
    def set_ears(self, pose): self.ears = pose
    def set_tail(self, pose, wag=0.0): self.tail = pose
    def set_posture(self, front_height, rear_height): self.fh, self.rh = front_height, rear_height
    def purr(self, on): pass
    def speak(self, clip): pass


class MujocoSenses(Senses):
    """Scripted cat encounter + real IMU from the running model."""
    def __init__(self, rig, clock):
        self._rig = rig
        self._clock = clock

    def camera(self):
        t = self._clock()
        # timeline: far -> approach -> engaged -> lunge -> gone
        if t < 2:
            return CatDetection(present=False)
        if t < 6:                                   # approaching from the front
            d = max(0.4, 1.4 - 0.25 * (t - 2))
            return CatDetection(present=True, distance=d, bearing=0.15,
                                speed=0.2, approaching=True, ears=EarPose.FORWARD)
        if t < 9:                                   # close & engaged (play)
            return CatDetection(present=True, distance=0.35, bearing=0.05,
                                speed=0.05, approaching=True, ears=EarPose.FORWARD)
        if t < 11:                                  # LUNGE
            return CatDetection(present=True, distance=0.2, bearing=0.0,
                                speed=0.5, approaching=True, ears=EarPose.FLAT,
                                hissing=True)
        return CatDetection(present=False)

    def imu(self):
        rzz = float(self._rig.data.xmat[self._rig.torso].reshape(3, 3)[2, 2])
        tilt = math.acos(max(-1.0, min(1.0, rzz)))
        return ImuReading(tilt=tilt, accel=0.0)

    def proximity(self):
        return ProximityReading(ahead=1.0, edge_ahead=False)

    def now(self):
        return self._clock()


class BrainController:
    """Callable control_fn(rig, t) that steps the brain and drives the joints."""
    def __init__(self):
        self.clock = _Clock()
        self.body = MujocoBody()
        self._rig = None
        self._senses = None
        self._brain = None
        self._phase = 0.0
        self._last_t = 0.0

    def _lazy(self, rig):
        if self._brain is None:
            self._rig = rig
            self._senses = MujocoSenses(rig, self.clock)
            self._brain = RoboKitten(self.body, self._senses, _Printer())

    def __call__(self, rig, t):
        self._lazy(rig)
        self.clock.t = t
        dt = max(1e-3, t - self._last_t); self._last_t = t
        self._brain.tick(dt)

        # advance a gait clock when the brain wants to move
        moving = self.body.gait_mode in (Gait.WALK, Gait.TROT) and abs(self.body.forward) > 0.02
        preset = gaitmod.PRESETS["trot" if self.body.gait_mode == Gait.TROT else "walk"]
        direction = 1.0
        if moving:
            self._phase = (self._phase + dt / preset["cycle"]) % 1.0
            scale = max(0.3, min(1.0, abs(self.body.forward) / 0.2))
            preset = dict(preset, stride=preset["stride"] * scale)
            direction = 1.0 if self.body.forward >= 0 else -1.0

        for leg in P.LEGS:
            pscale = self.body.fh if leg in ("FL", "FR") else self.body.rh
            depth = gaitmod.leg_depth(leg) * pscale
            if moving:
                x, d = gaitmod.foot_target(leg, self._phase, preset, depth)
                hip, knee = gaitmod.leg_ik(leg, direction * x, d)
            else:
                hip, knee = gaitmod.leg_ik(leg, 0.0, depth)
            rig.set_target(f"{leg}_hip", hip)      # ankle follows via coupling
            rig.set_target(f"{leg}_knee", knee)

        rig.set_target("head_pan", self.body.head)
        rig.set_target("ear_L", _EAR[self.body.ears])   # ear_R follows via coupling
        rig.set_target("tail", _TAIL[self.body.tail])


def make_brain_control():
    return BrainController()
