"""
RoboKitten behaviors — what each mood actually *does*.
======================================================

One behavior per mood. Each is a small stateful object with:

    * ``enter(wm)`` — called once when the behavior becomes active
    * ``step(dt, wm)`` — called every tick; emits ``Expression`` calls

Behaviors are deliberately **erratic, curious, and pausing** rather than smooth
and continuous — that is what reads as "kitten" to a real cat (PLAN §4.2). They
take an injected RNG so the sim/tests are reproducible.

Behaviors emit narratable ``Event``s for salient moments (slow-blink, play-bow,
pounce, freeze) via ``ctx.emit`` so the owner dashboard has a story to show.
"""

from __future__ import annotations

import math
import random
from dataclasses import dataclass
from typing import Callable

from brain.expression import Expression
from brain.hal import EarPose, TailPose
from brain.perception import WorldModel


@dataclass
class Ctx:
    """Shared services handed to every behavior."""
    expr: Expression
    emit: Callable[[str, str], None]   # (kind, text) -> narratable event
    rng: random.Random
    now: Callable[[], float]           # monotonic seconds


class Behavior:
    name = "behavior"

    def __init__(self, ctx: Ctx):
        self.ctx = ctx

    def enter(self, wm: WorldModel) -> None:  # noqa: B027 - optional hook
        pass

    def step(self, dt: float, wm: WorldModel) -> None:
        raise NotImplementedError

    # -- small helper for "do X every T-ish seconds" ---------------
    @staticmethod
    def _due(clock: float, next_t: float) -> bool:
        return clock >= next_t


# --------------------------------------------------------------------- SLEEPY
class RestBehavior(Behavior):
    name = "rest"

    def enter(self, wm):
        self.ctx.expr.curl_sleep()
        self.ctx.expr.purr(True)         # contented sleep-purr
        self.ctx.emit("behavior", "😴 curls up to rest")
        self._next_blink = self.ctx.now() + self.ctx.rng.uniform(6, 10)

    def step(self, dt, wm):
        self.ctx.expr.curl_sleep()
        now = self.ctx.now()
        if self._due(now, self._next_blink):
            self.ctx.expr.slow_blink()   # a sleepy half-blink
            self._next_blink = now + self.ctx.rng.uniform(6, 10)


# --------------------------------------------------------------------- CALM
class CalmBehavior(Behavior):
    """Awake but idle: mostly still, occasional aimless wander + ambient blink."""
    name = "calm"

    def enter(self, wm):
        self.ctx.expr.purr(False)
        self.ctx.expr.stand()
        self.ctx.expr.friendly_signal()
        self.ctx.emit("behavior", "🐾 pads around calmly")
        self._pick_next(first=True)

    def _pick_next(self, first=False):
        now = self.ctx.now()
        self._phase_end = now + self.ctx.rng.uniform(2.0, 4.5)
        self._wandering = (not first) and self.ctx.rng.random() < 0.5
        self._wander_yaw = self.ctx.rng.uniform(-0.6, 0.6)
        self._next_blink = now + self.ctx.rng.uniform(4, 8)

    def step(self, dt, wm):
        now = self.ctx.now()
        if self._wandering:
            self.ctx.expr.wander(self._wander_yaw)
        else:
            self.ctx.expr.stand()
            self.ctx.expr.friendly_signal()
        if self._due(now, self._next_blink):
            self.ctx.expr.slow_blink()
            self._next_blink = now + self.ctx.rng.uniform(4, 8)
        if self._due(now, self._phase_end):
            self._pick_next()


# --------------------------------------------------------------------- CURIOUS
class WatchBehavior(Behavior):
    """A cat is around but not yet in play range: watch it, creep closer,
    and slow-blink/trill to build trust (the moves that earn approach)."""
    name = "watch"

    def enter(self, wm):
        self.ctx.expr.purr(False)
        self.ctx.expr.friendly_signal()
        self.ctx.emit("behavior", "👀 watches the cat, keeping soft and low")
        now = self.ctx.now()
        self._next_blink = now + self.ctx.rng.uniform(2.5, 4.0)
        self._next_trill = now + self.ctx.rng.uniform(4.0, 7.0)

    def step(self, dt, wm):
        now = self.ctx.now()
        self.ctx.expr.ears(EarPose.FORWARD)
        # Creep slowly and low toward a visible cat to close the gap
        # ("move slowly and low on first approach", PLAN §2.2). Once we're
        # near, hold station and let trust-building signals do the work — let
        # the cat make the final approach itself.
        if wm.cat_distance > 0.55:
            self.ctx.expr.walk_toward(wm.cat_bearing, speed=0.05)
        else:
            self.ctx.expr.look(wm.cat_bearing)
            self.ctx.expr.halt()
            self.ctx.expr.tail(TailPose.UP, wag=0.3)
        # trust-building signals
        if self._due(now, self._next_blink):
            self.ctx.expr.slow_blink()
            self.ctx.emit("cat", "😌 slow-blinks at the cat (building trust)")
            self._next_blink = now + self.ctx.rng.uniform(2.5, 4.0)
        if self._due(now, self._next_trill):
            self.ctx.expr.trill()
            self._next_trill = now + self.ctx.rng.uniform(5.0, 9.0)


# --------------------------------------------------------------------- PLAYFUL
class PlayBehavior(Behavior):
    """Cat is close & engaged: cycle the play invitation — bow, wiggle, pounce,
    pause. Erratic timing on purpose."""
    name = "play"

    # micro-phases of a play cycle
    _BOW, _WIGGLE, _POUNCE, _PAUSE = range(4)

    def enter(self, wm):
        self.ctx.expr.purr(False)
        self.ctx.emit("behavior", "🤸 play mode — 'let's play!'")
        self._start_phase(self._BOW)

    def _start_phase(self, phase):
        self._phase = phase
        now = self.ctx.now()
        durations = {
            self._BOW: self.ctx.rng.uniform(0.8, 1.4),
            self._WIGGLE: self.ctx.rng.uniform(0.4, 0.8),
            self._POUNCE: self.ctx.rng.uniform(0.5, 0.9),
            self._PAUSE: self.ctx.rng.uniform(0.6, 1.6),
        }
        self._phase_end = now + durations[phase]
        if phase == self._BOW:
            self.ctx.expr.play_bow()
            self.ctx.emit("cat", "🙇 play-bows (invitation to chase)")
        elif phase == self._POUNCE:
            self.ctx.emit("cat", "💨 pounces toward the cat!")

    def step(self, dt, wm):
        now = self.ctx.now()
        self.ctx.expr.ears(EarPose.FORWARD)
        self.ctx.expr.tail(TailPose.UP, wag=1.0)

        if self._phase == self._BOW:
            self.ctx.expr.play_bow()
        elif self._phase == self._WIGGLE:
            # rear-end wiggle: tiny alternating yaw, staying put
            self.ctx.expr.wiggle_in_place(0.5 * math.sin(now * 18))
        elif self._phase == self._POUNCE:
            self.ctx.expr.scamper_toward(wm.cat_bearing, speed=0.22)
            if self.ctx.rng.random() < 0.04:
                self.ctx.expr.chirp()
        elif self._phase == self._PAUSE:
            self.ctx.expr.halt()
            self.ctx.expr.crouch_freeze()

        if self._due(now, self._phase_end):
            self._start_phase((self._phase + 1) % 4)


# --------------------------------------------------------------------- SCARED
class RetreatBehavior(Behavior):
    """Startle-freeze, then back away from the threat, ears flat. A single hiss
    if the threat is right on top of it."""
    name = "retreat"

    def enter(self, wm):
        self.ctx.expr.purr(False)
        self.ctx.expr.crouch_freeze()
        self.ctx.expr.wary_signal()
        self.ctx.expr.wide_eyes()
        self.ctx.emit("alert", "🙀 startled — freezing low")
        now = self.ctx.now()
        self._freeze_until = now + 0.7      # hold still first (startle-freeze)
        self._hissed = False

    def step(self, dt, wm):
        now = self.ctx.now()
        self.ctx.expr.ears(EarPose.FLAT)
        self.ctx.expr.tail(TailPose.PUFFED, wag=0.9)

        if now < self._freeze_until:
            self.ctx.expr.crouch_freeze()   # frozen
            if wm.cat_close and not self._hissed:
                self.ctx.expr.hiss()
                self._hissed = True
            return
        # then retreat from the cat if we know where it is
        if wm.cat_present:
            self.ctx.expr.back_away(wm.cat_bearing, speed=0.12)
        else:
            self.ctx.expr.crouch_freeze()


# mood name -> behavior class is wired in robokitten.py
