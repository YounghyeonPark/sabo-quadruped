"""Make the project root importable so `import brain` / `import sim` work under pytest."""

import os
import sys

sys.path.insert(0, os.path.dirname(os.path.dirname(os.path.abspath(__file__))))


class Clock:
    """A hand-advanced monotonic clock for deterministic FSM/timer tests."""

    def __init__(self, t: float = 0.0):
        self.t = t

    def __call__(self) -> float:
        return self.t

    def advance(self, dt: float) -> float:
        self.t += dt
        return self.t
