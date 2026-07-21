"""
Simulation runner — wires the SimWorld to the RoboKitten brain.
===============================================================

One ``Simulation`` owns a world, a brain, and an event buffer, and advances them
together each tick in the correct order (sense → decide → act → integrate). Both
the dashboard server and the ``--headless`` mode drive it through this class, so
they observe identical behavior.
"""

from __future__ import annotations

import random
import threading
from collections import deque

from brain.hal import Event, EventSink
from brain.robokitten import RoboKitten
from sim.world import SimWorld


class EventBuffer(EventSink):
    """Thread-safe ring buffer of narratable events, each with a stable id."""

    def __init__(self, maxlen: int = 400):
        self._events: deque[dict] = deque(maxlen=maxlen)
        self._next_id = 0
        self._lock = threading.Lock()

    def emit(self, event: Event) -> None:
        with self._lock:
            self._events.append({
                "id": self._next_id, "t": round(event.t, 2),
                "kind": event.kind, "text": event.text,
            })
            self._next_id += 1

    def since(self, last_id: int, limit: int = 60) -> list[dict]:
        with self._lock:
            return [e for e in self._events if e["id"] > last_id][-limit:]


class Simulation:
    def __init__(self, seed: int | None = None):
        rng = random.Random(seed)
        self.world = SimWorld(rng)
        self.events = EventBuffer()
        self.brain = RoboKitten(self.world, self.world, self.events, rng)
        self._lock = threading.Lock()

    def tick(self, dt: float) -> None:
        with self._lock:
            self.brain.tick(dt)     # sense → decide → act (commands the body)
            self.world.step(dt)     # integrate physics + advance the cat

    def owner_action(self, name: str) -> bool:
        with self._lock:
            return self.world.owner_action(name) or self.brain.owner_action(name)

    def snapshot(self, since_id: int = -1) -> dict:
        with self._lock:
            snap = self.world.snapshot()
            snap["brain"] = self.brain.snapshot()
            snap["events"] = self.events.since(since_id)
            return snap
