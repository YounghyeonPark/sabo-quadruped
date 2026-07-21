"""
RoboKitten — the brain loop.
============================

Ties the layers together into the sense → perceive → decide → act cycle
(PLAN §8), once per tick::

    Senses ──▶ Perception ──▶ MoodMachine ──▶ Behavior.step ──▶ Expression ──▶ Body

The loop is hardware-agnostic: it only holds a ``Body``, ``Senses`` and an
``EventSink``. Swap those three for hardware implementations and the exact same
decision-making runs on the real Mini Pupper.
"""

from __future__ import annotations

import random
from typing import Optional

from brain.behaviors import (Behavior, CalmBehavior, Ctx, PlayBehavior,
                             RestBehavior, RetreatBehavior, WatchBehavior)
from brain.expression import Expression
from brain.hal import Body, Event, EventSink, Senses
from brain.mood import Mood, MoodMachine
from brain.perception import Perception
from brain.voice import Voice


class RoboKitten:
    def __init__(self, body: Body, senses: Senses, events: EventSink,
                 rng: Optional[random.Random] = None):
        self._body = body
        self._senses = senses
        self._events = events
        self._clock = senses.now
        self._rng = rng or random.Random()

        self.perception = Perception(senses)
        self.voice = Voice(body, events, self._clock)
        self.expr = Expression(body, self.voice)
        self.mood_machine = MoodMachine(self._clock)

        ctx = Ctx(expr=self.expr, emit=self._emit, rng=self._rng, now=self._clock)
        self._behaviors: dict[Mood, Behavior] = {
            Mood.SLEEPY: RestBehavior(ctx),
            Mood.CALM: CalmBehavior(ctx),
            Mood.CURIOUS: WatchBehavior(ctx),
            Mood.PLAYFUL: PlayBehavior(ctx),
            Mood.SCARED: RetreatBehavior(ctx),
        }
        self._active_mood: Optional[Mood] = None
        self.last_wm = None
        self.last_behavior = "—"

    # -- one brain tick ---------------------------------------------
    def tick(self, dt: float) -> dict:
        wm = self.perception.update()
        mood = self.mood_machine.update(wm)

        behavior = self._behaviors[mood]
        if mood != self._active_mood:
            self._emit("mood", f"mood → {mood.value}")
            behavior.enter(wm)
            # Silent rest: only when SLEEPY (curled on the ground) are the legs
            # unloaded enough to cut servo power and go quiet (docs/noise_reduction).
            self.expr.relax(mood == Mood.SLEEPY)
            self._active_mood = mood
        behavior.step(dt, wm)

        self.last_wm = wm
        self.last_behavior = behavior.name
        return self.snapshot()

    # -- owner overrides (dashboard buttons) ------------------------
    def owner_action(self, name: str) -> bool:
        """Handle kitten-directed owner actions. Returns True if handled here."""
        if name == "wake":
            self.mood_machine.force(Mood.CALM)
            self._emit("owner", "🧑 wakes the kitten")
            return True
        if name == "nap":
            self.mood_machine.force(Mood.SLEEPY)
            self._emit("owner", "🧑 tells the kitten to nap")
            return True
        if name == "call":
            self.expr.meow()
            self.mood_machine.force(Mood.CURIOUS, hold=2.0)
            self._emit("owner", "🧑 calls the kitten")
            return True
        return False

    # -- state snapshot for the dashboard ---------------------------
    def snapshot(self) -> dict:
        wm = self.last_wm
        return {
            "mood": self.mood_machine.mood.value,
            "behavior": self.last_behavior,
            "cat": {
                "present": bool(wm and wm.cat_present),
                "distance": round(wm.cat_distance, 2) if wm and wm.cat_present else None,
                "bearing": round(wm.cat_bearing, 2) if wm else 0.0,
                "engaged": bool(wm and wm.cat_engaged),
                "threat": bool(wm and wm.cat_threat),
            },
            "quiet_time": round(wm.quiet_time, 1) if wm else 0.0,
        }

    # -- internals --------------------------------------------------
    def _emit(self, kind: str, text: str) -> None:
        self._events.emit(Event(t=self._clock(), kind=kind, text=text))
