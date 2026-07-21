"""
RoboKitten mood — the personality state machine.
=================================================

A small finite-state machine over the ``WorldModel``. Mood is what gives the
kitten *coherent* personality instead of random twitching (PLAN §8): each mood
selects one behavior, and moods change only on meaningful, sustained evidence.

States (PLAN §8)::

    SLEEPY  — long quiet, no cat around → rests/curls
    CALM    — awake, nothing going on → idle wander/sit
    CURIOUS — a cat is around but far/still → watch, approach, slow-blink
    PLAYFUL — cat is close & engaged & not threatened → invite play
    SCARED  — threat / lunge / big jostle → freeze & retreat

Anti-flip-flop design:
    * SCARED is entered instantly (safety) but only *leaves* after a calm dwell.
    * Other transitions require the trigger to persist for a short debounce, so
      one noisy frame can't yank the mood around.
"""

from __future__ import annotations

import enum
from dataclasses import dataclass

from brain.perception import QUIET_FOR_SLEEP, WorldModel


class Mood(enum.Enum):
    SLEEPY = "sleepy"
    CALM = "calm"
    CURIOUS = "curious"
    PLAYFUL = "playful"
    SCARED = "scared"


# how long a trigger must hold before a (non-safety) transition fires
DEBOUNCE = 0.4
# minimum calm dwell before leaving SCARED
SCARED_MIN_DWELL = 3.0
# a fresh scare re-arms this recovery window
SCARED_SETTLE = 1.5


@dataclass
class _Candidate:
    mood: Mood
    since: float  # when this candidate first became the target


class MoodMachine:
    """Owns the current mood + transition logic. Owner overrides can force a mood."""

    def __init__(self, clock, start: Mood = Mood.CALM):
        self._now = clock
        self.mood = start
        self._entered_at = clock()
        self._candidate: _Candidate | None = None
        self._last_threat_t: float | None = None
        self._forced: Mood | None = None      # owner "wake"/"nap" override
        self._forced_until: float = 0.0

    # -- owner overrides (dashboard buttons) ------------------------
    def force(self, mood: Mood, hold: float = 4.0) -> None:
        """Owner nudges the kitten (e.g. 'wake up', 'nap'). Decays after ``hold``."""
        self._forced = mood
        self._forced_until = self._now() + hold
        self._set(mood)

    # -- main update ------------------------------------------------
    def update(self, wm: WorldModel) -> Mood:
        now = self._now()
        if wm.cat_threat or wm.jostled or wm.self_tilt > 0.5:
            self._last_threat_t = now

        # honor a live owner override
        if self._forced is not None and now < self._forced_until:
            return self.mood
        self._forced = None

        target = self._desired(wm, now)

        # SCARED has priority + sticky exit; handle it explicitly
        if target == Mood.SCARED:
            self._set(Mood.SCARED)
            return self.mood
        if self.mood == Mood.SCARED and not self._recovered(now):
            return self.mood  # still settling; ignore other targets

        # debounce non-safety transitions
        if target == self.mood:
            self._candidate = None
        else:
            if self._candidate is None or self._candidate.mood != target:
                self._candidate = _Candidate(mood=target, since=now)
            elif now - self._candidate.since >= DEBOUNCE:
                self._set(target)
                self._candidate = None
        return self.mood

    # -- pure policy: what mood does the world call for right now? --
    def _desired(self, wm: WorldModel, now: float) -> Mood:
        if wm.cat_threat or wm.jostled or wm.self_tilt > 0.5:
            return Mood.SCARED
        if wm.cat_present:
            # Only play once the cat itself chooses to engage (approaching &
            # relaxed) — pouncing on a still-wary cat would scare it off
            # ("let the cat approach first", PLAN §2.2). Otherwise stay curious:
            # watch and slow-blink to keep earning trust.
            if wm.cat_engaged:
                return Mood.PLAYFUL
            return Mood.CURIOUS
        # no cat
        if wm.quiet_time >= QUIET_FOR_SLEEP:
            return Mood.SLEEPY
        return Mood.CALM

    # -- helpers ----------------------------------------------------
    def _recovered(self, now: float) -> bool:
        """Enough calm time has passed since the last scare to leave SCARED."""
        dwell_ok = now - self._entered_at >= SCARED_MIN_DWELL
        settled = (self._last_threat_t is None
                   or now - self._last_threat_t >= SCARED_SETTLE)
        return dwell_ok and settled

    def _set(self, mood: Mood) -> None:
        if mood != self.mood:
            self.mood = mood
            self._entered_at = self._now()
