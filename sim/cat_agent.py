"""
Simulated cat (Sami stand-in).
==============================

A tiny behavioral model of a real cat, so the RoboKitten brain has something to
react to in Phase 0. The cat carries a **trust** scalar and a **startle**
scalar; together with distance they decide whether it approaches, wanders, or
flees.

This is the whole point of the sim: the project's north-star metric is *"does
the resident cat choose to interact?"* — here that is literally observable as
``trust`` rising and the cat closing distance, or falling and the cat fleeing.

Trust rises when the kitten does the *right* things (slow-blink, trill, calm
slow movement) and falls when it does the *wrong* things (charging, hissing) —
the same feline social rules the kitten's expression layer is built around.
"""

from __future__ import annotations

import math
from dataclasses import dataclass, field

from brain.hal import EarPose


@dataclass
class CatSignals:
    """What the kitten did this tick, as the cat would perceive it."""
    kitten_x: float
    kitten_y: float
    kitten_speed: float          # m/s
    kitten_charging: bool        # moving fast toward the cat at close range
    slow_blinked: bool = False
    trilled: bool = False
    hissed: bool = False
    toy_x: float | None = None   # owner tossed a toy here
    toy_y: float | None = None


@dataclass
class Cat:
    x: float = 1.1
    y: float = 0.4
    heading: float = math.pi
    trust: float = 0.35          # 0 wary … 1 bonded
    startle: float = 0.0         # 0 calm … 1 spooked
    vx: float = 0.0
    vy: float = 0.0
    mode: str = "wander"
    _wander_dir: float = field(default=math.pi)
    _wander_timer: float = 0.0

    # tuning
    APPROACH_SPEED = 0.16
    FLEE_SPEED = 0.5
    WANDER_SPEED = 0.08
    TRUST_TO_APPROACH = 0.55
    STARTLE_TO_FLEE = 0.4

    def step(self, dt: float, sig: CatSignals, rng, bounds) -> None:
        # -- update trust & startle from the kitten's conduct --------
        if sig.slow_blinked:
            self.trust = min(1.0, self.trust + 0.06)
        if sig.trilled:
            self.trust = min(1.0, self.trust + 0.03)
        if sig.hissed:
            self.startle = 1.0
            self.trust = max(0.0, self.trust - 0.12)
        if sig.kitten_charging:
            # A trusted kitten barrelling in reads as *play*; a stranger doing
            # it reads as a threat. Scale the fright by how little we trust it.
            fright = 1.0 - self.trust
            self.startle = min(1.0, self.startle + 2.0 * dt * fright)
            self.trust = max(0.0, self.trust - 0.3 * dt * fright)

        dx, dy = sig.kitten_x - self.x, sig.kitten_y - self.y
        dist = math.hypot(dx, dy)

        # calm, gentle, nearby company slowly earns trust (a slow creep still
        # counts as gentle — only fast/charging motion is threatening)
        if dist < 0.7 and sig.kitten_speed < 0.08 and not sig.kitten_charging:
            self.trust = min(1.0, self.trust + 0.05 * dt)

        self.startle = max(0.0, self.startle - 0.5 * dt)   # ~2 s to shake off

        # -- decide mode --------------------------------------------
        if self.startle > self.STARTLE_TO_FLEE:
            self.mode = "flee"
        elif sig.toy_x is not None:
            self.mode = "chase_toy"
        elif self.trust >= self.TRUST_TO_APPROACH and dist > 0.3:
            self.mode = "approach"
        else:
            self.mode = "wander"

        # -- act ----------------------------------------------------
        if self.mode == "flee":
            self._go(-dx, -dy, self.FLEE_SPEED, dist)
        elif self.mode == "chase_toy":
            self._go(sig.toy_x - self.x, sig.toy_y - self.y,
                     self.APPROACH_SPEED * 1.3, None)
        elif self.mode == "approach":
            # curious but cautious: ease off right up close
            spd = self.APPROACH_SPEED * (0.4 if dist < 0.35 else 1.0)
            self._go(dx, dy, spd, dist)
        else:
            self._wander(dt, rng)

        self.x += self.vx * dt
        self.y += self.vy * dt
        self._clamp(bounds)

    # -- helpers ----------------------------------------------------
    def _go(self, dx, dy, speed, dist):
        d = math.hypot(dx, dy) or 1.0
        self.vx, self.vy = dx / d * speed, dy / d * speed
        self.heading = math.atan2(dy, dx)

    def _wander(self, dt, rng):
        self._wander_timer -= dt
        if self._wander_timer <= 0:
            self._wander_timer = rng.uniform(1.0, 2.5)
            if rng.random() < 0.4:            # sometimes just sit
                self._wander_dir = None
            else:
                self._wander_dir = rng.uniform(-math.pi, math.pi)
        if self._wander_dir is None:
            self.vx = self.vy = 0.0
        else:
            self.vx = math.cos(self._wander_dir) * self.WANDER_SPEED
            self.vy = math.sin(self._wander_dir) * self.WANDER_SPEED
            self.heading = self._wander_dir

    def _clamp(self, bounds):
        (x0, x1, y0, y1) = bounds
        if self.x < x0: self.x, self.vx = x0, abs(self.vx)
        if self.x > x1: self.x, self.vx = x1, -abs(self.vx)
        if self.y < y0: self.y, self.vy = y0, abs(self.vy)
        if self.y > y1: self.y, self.vy = y1, -abs(self.vy)

    # -- observable body language (what the kitten's camera sees) ---
    @property
    def observed_ears(self) -> EarPose:
        if self.startle > self.STARTLE_TO_FLEE:
            return EarPose.FLAT
        if self.mode in ("approach", "chase_toy"):
            return EarPose.FORWARD
        return EarPose.NEUTRAL

    @property
    def hissing(self) -> bool:
        return self.startle > 0.7

    @property
    def speed(self) -> float:
        return math.hypot(self.vx, self.vy)
