"""
SimWorld — the Phase-0 stand-in for the kitten's body and its world.
====================================================================

A headless 2-D top-down "room" that implements both HAL interfaces:

    * as a ``Body`` it receives the brain's actuator commands and moves the
      kitten (kinematic integration of the gait velocity intents);
    * as ``Senses`` it reports what the kitten perceives — chiefly a
      ``CatDetection`` computed from the simulated cat's true position, plus a
      jostle IMU and a wall-distance proximity read.

Swapping this class for a hardware ``Body``/``Senses`` pair (PCA9685 + Pi camera
+ IMU) is the entire porting step for the ``brain/`` package.

Coordinates: metres, origin at room centre, +x = the kitten's initial forward,
+y = its left. Bearings returned to the brain are in the kitten's own frame.
"""

from __future__ import annotations

import math
import random

from brain.hal import (BlinkKind, Body, CatDetection, EarPose, Gait, ImuReading,
                       ProximityReading, Senses, TailPose)
from sim.cat_agent import Cat, CatSignals

TWO_PI = 2 * math.pi


def _wrap(a: float) -> float:
    """Normalize angle to (-pi, pi]."""
    return (a + math.pi) % TWO_PI - math.pi


class SimWorld(Body, Senses):
    # room half-extents (metres) with a small wall margin
    X0, X1, Y0, Y1 = -1.5, 1.5, -1.0, 1.0
    MARGIN = 0.06
    SIGHT_RANGE = 1.9
    FOV = 2.1  # radians, half-angle ~60° each side (plus close-range omni)

    def __init__(self, rng: random.Random | None = None):
        self._rng = rng or random.Random()
        self.t = 0.0

        # kitten pose
        self.kx, self.ky, self.kh = -0.8, 0.0, 0.0
        self.k_speed = 0.0

        # commanded actuator state
        self.gait_mode = Gait.STAND
        self.cmd_forward = 0.0
        self.cmd_yaw = 0.0
        self.head_bearing = 0.0
        self.ears = EarPose.NEUTRAL
        self.tail = TailPose.MID
        self.tail_wag = 0.0
        self.fh, self.rh = 1.0, 1.0
        self.eyes_open = 1.0
        self.purring = False
        self.last_blink_t = -10.0
        self.last_blink_kind = None
        self.last_clip = None

        # per-tick signal accumulators (consumed in step)
        self._blinked_slow = False
        self._spoke = None

        # the cat
        self.cat = Cat()
        self._prev_cat_dist = self._cat_dist()

        # toy (owner "toss toy")
        self.toy = None            # (x, y) or None
        self._toy_until = 0.0

        # jostle
        self._accel = 0.0

    # ================================================================= Body
    def gait(self, mode: Gait, forward: float = 0.0, yaw: float = 0.0) -> None:
        self.gait_mode = mode
        self.cmd_forward = forward
        self.cmd_yaw = yaw

    def look_at(self, bearing: float, tilt: float = 0.0) -> None:
        self.head_bearing = bearing

    def blink(self, kind: BlinkKind) -> None:
        self.last_blink_t = self.t
        self.last_blink_kind = kind
        if kind == BlinkKind.SLOW:
            self._blinked_slow = True

    def set_eyes(self, openness: float) -> None:
        self.eyes_open = max(0.0, min(1.2, openness))

    def set_ears(self, pose: EarPose) -> None:
        self.ears = pose

    def set_tail(self, pose: TailPose, wag: float = 0.0) -> None:
        self.tail = pose
        self.tail_wag = wag

    def set_posture(self, front_height: float, rear_height: float) -> None:
        self.fh, self.rh = front_height, rear_height

    def purr(self, on: bool) -> None:
        self.purring = on

    def speak(self, clip: str) -> None:
        self.last_clip = clip
        self._spoke = clip

    # =============================================================== Senses
    def now(self) -> float:
        return self.t

    def camera(self) -> CatDetection:
        dx, dy = self.cat.x - self.kx, self.cat.y - self.ky
        dist = math.hypot(dx, dy)
        bearing = _wrap(math.atan2(dy, dx) - self.kh)

        visible = dist <= self.SIGHT_RANGE and (abs(bearing) <= self.FOV
                                                or dist < 0.6)
        if not visible:
            return CatDetection(present=False)

        approaching = dist < self._prev_cat_dist - 1e-4
        return CatDetection(
            present=True, distance=dist, bearing=bearing,
            speed=self.cat.speed, approaching=approaching,
            ears=self.cat.observed_ears, hissing=self.cat.hissing,
        )

    def imu(self) -> ImuReading:
        return ImuReading(tilt=0.0, accel=self._accel)

    def proximity(self) -> ProximityReading:
        ahead = self._forward_wall_dist()
        return ProximityReading(ahead=ahead, edge_ahead=False)

    # ================================================================= step
    def step(self, dt: float) -> None:
        self.t += dt

        # -- integrate kitten motion from gait velocity intents ------
        self.kh = _wrap(self.kh + self.cmd_yaw * dt)
        wall_ahead = self._forward_wall_dist()
        fwd = self.cmd_forward
        # don't drive through a wall (allow backing away)
        if fwd > 0 and wall_ahead < 0.08:
            fwd = 0.0
        nx = self.kx + math.cos(self.kh) * fwd * dt
        ny = self.ky + math.sin(self.kh) * fwd * dt
        moved = math.hypot(nx - self.kx, ny - self.ky)
        self.kx = min(self.X1 - self.MARGIN, max(self.X0 + self.MARGIN, nx))
        self.ky = min(self.Y1 - self.MARGIN, max(self.Y0 + self.MARGIN, ny))
        self.k_speed = moved / dt if dt > 0 else 0.0

        # -- did the kitten charge the cat? --------------------------
        dx, dy = self.cat.x - self.kx, self.cat.y - self.ky
        dist = math.hypot(dx, dy)
        rel = abs(_wrap(math.atan2(dy, dx) - self.kh))
        charging = (self.cmd_forward > 0.14 and dist < 0.6 and rel < 0.7)

        # -- toy lifetime -------------------------------------------
        if self.toy is not None and self.t >= self._toy_until:
            self.toy = None

        # -- advance the cat -----------------------------------------
        sig = CatSignals(
            kitten_x=self.kx, kitten_y=self.ky, kitten_speed=self.k_speed,
            kitten_charging=charging,
            slow_blinked=self._blinked_slow,
            trilled=self._spoke in ("trill", "meow", "chirp"),
            hissed=self._spoke == "hiss",
            toy_x=self.toy[0] if self.toy else None,
            toy_y=self.toy[1] if self.toy else None,
        )
        self.cat.step(dt, sig, self._rng, (self.X0 + self.MARGIN, self.X1 - self.MARGIN,
                                           self.Y0 + self.MARGIN, self.Y1 - self.MARGIN))

        # -- jostle: cat pounced onto the kitten ---------------------
        self._accel *= math.exp(-dt / 0.25)
        if dist < 0.16 and self.cat.speed > 0.3:
            self._accel = 8.0

        # -- bookkeeping --------------------------------------------
        self._prev_cat_dist = self._cat_dist()
        self._blinked_slow = False
        self._spoke = None

    # -- owner actions handled by the world ------------------------
    def owner_action(self, name: str) -> bool:
        if name == "toss_toy":
            # land a toy in open floor between the two, biased toward the cat
            self.toy = (self._rng.uniform(-0.3, 1.0), self._rng.uniform(-0.6, 0.6))
            self._toy_until = self.t + 6.0
            return True
        return False

    # ================================================================ render
    def snapshot(self) -> dict:
        recent_blink = (self.t - self.last_blink_t) < 0.5
        return {
            "t": round(self.t, 2),
            "bounds": [self.X0, self.X1, self.Y0, self.Y1],
            "kitten": {
                "x": round(self.kx, 3), "y": round(self.ky, 3),
                "heading": round(self.kh, 3),
                "head_bearing": round(self.head_bearing, 3),
                "gait": self.gait_mode.value, "speed": round(self.k_speed, 3),
                "fh": round(self.fh, 3), "rh": round(self.rh, 3),
                "ears": self.ears.value, "tail": self.tail.value,
                "tail_wag": round(self.tail_wag, 2),
                "eyes": round(self.eyes_open, 2), "blink": recent_blink,
                "purr": self.purring,
            },
            "cat": {
                "x": round(self.cat.x, 3), "y": round(self.cat.y, 3),
                "heading": round(self.cat.heading, 3),
                "trust": round(self.cat.trust, 3),
                "startle": round(self.cat.startle, 3),
                "mode": self.cat.mode,
                "ears": self.cat.observed_ears.value,
                "hissing": self.cat.hissing,
            },
            "toy": ({"x": round(self.toy[0], 3), "y": round(self.toy[1], 3)}
                    if self.toy else None),
        }

    # -- internals --------------------------------------------------
    def _cat_dist(self) -> float:
        return math.hypot(self.cat.x - self.kx, self.cat.y - self.ky)

    def _forward_wall_dist(self) -> float:
        """Distance from the kitten to the room wall straight ahead."""
        cx, sy = math.cos(self.kh), math.sin(self.kh)
        best = float("inf")
        for t in (self._ray_t(cx, self.kx, self.X0),
                  self._ray_t(cx, self.kx, self.X1),
                  self._ray_t(sy, self.ky, self.Y0),
                  self._ray_t(sy, self.ky, self.Y1)):
            if t is not None and t < best:
                best = t
        return best

    @staticmethod
    def _ray_t(dir_comp: float, origin: float, wall: float):
        if abs(dir_comp) < 1e-9:
            return None
        t = (wall - origin) / dir_comp
        return t if t > 1e-6 else None
