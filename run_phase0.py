"""
RoboKitten — Phase 0 entrypoint.
================================

Runs the autonomous brain against the simulated cat.

    python run_phase0.py                 # live web dashboard at http://localhost:5000
    python run_phase0.py --headless      # print a mood/behavior/event timeline, no Flask
    python run_phase0.py --headless --duration 60 --speed 4

Options:
    --headless        no server; stream the event timeline to the console
    --duration SEC    headless run length (default 45 sim-seconds)
    --speed X         sim-time / wall-time factor (live mode; headless runs as fast as it can)
    --seed N          RNG seed for reproducible runs
    --port N          dashboard port (default 5000)
"""

from __future__ import annotations

import argparse
import sys
import threading
import time

from sim.runner import Simulation

# The kitten narrates with emoji; make sure a cp1252 Windows console won't crash.
try:
    sys.stdout.reconfigure(encoding="utf-8", errors="replace")
except (AttributeError, ValueError):
    pass

DT = 1.0 / 30.0   # sim tick


def run_headless(sim: Simulation, duration: float) -> None:
    print(f"RoboKitten headless — {duration:.0f}s @ {1/DT:.0f}Hz. "
          f"Watch mood shift as the cat reacts.\n")
    steps = int(duration / DT)
    last_id = -1
    last_mood = None
    for _ in range(steps):
        sim.tick(DT)
        snap = sim.snapshot(last_id)
        for e in snap["events"]:
            last_id = e["id"]
            mark = {"mood": "◆", "alert": "⚠", "owner": "●"}.get(e["kind"], "·")
            print(f"  {e['t']:6.1f}s {mark} {e['text']}")
        mood = snap["brain"]["mood"]
        if mood != last_mood:
            last_mood = mood
    c = sim.snapshot()["cat"]
    print(f"\nFinal: cat trust={c['trust']:.0%}  startle={c['startle']:.0%}  "
          f"mode={c['mode']}")


def run_live(sim: Simulation, port: int, speed: float) -> None:
    from dashboard.server import create_app

    def loop():
        # real-time pacing: advance `speed` sim-seconds per wall-second
        next_t = time.perf_counter()
        while True:
            sim.tick(DT)
            next_t += DT / speed
            delay = next_t - time.perf_counter()
            if delay > 0:
                time.sleep(delay)
            else:
                next_t = time.perf_counter()   # fell behind; resync

    threading.Thread(target=loop, daemon=True).start()
    app = create_app(sim)
    print(f"RoboKitten dashboard → http://localhost:{port}  (Ctrl+C to stop)")
    app.run(host="127.0.0.1", port=port, threaded=True, debug=False)


def main() -> None:
    ap = argparse.ArgumentParser(description="RoboKitten Phase-0 brain + sim")
    ap.add_argument("--headless", action="store_true")
    ap.add_argument("--duration", type=float, default=45.0)
    ap.add_argument("--speed", type=float, default=1.0)
    ap.add_argument("--seed", type=int, default=None)
    ap.add_argument("--port", type=int, default=5000)
    args = ap.parse_args()

    sim = Simulation(seed=args.seed)
    if args.headless:
        run_headless(sim, args.duration)
    else:
        run_live(sim, args.port, args.speed)


if __name__ == "__main__":
    main()
