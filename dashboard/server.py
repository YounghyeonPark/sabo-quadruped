"""
RoboKitten owner dashboard — Flask + Server-Sent Events.
========================================================

Serves a single live page and streams the simulation state to it ~20×/s over
SSE. This is the Phase-0 realization of the owner-facing dashboard from PLAN
§5.2: mood, current behavior, a live top-down view of the kitten and the cat,
the cat's trust, and a scrolling event feed — plus owner-action buttons.

The ``Simulation`` is stepped by a background thread started in ``run_phase0.py``;
this module only reads snapshots and forwards owner actions.
"""

from __future__ import annotations

import json
import os
import time

from flask import Flask, Response, jsonify, send_from_directory

from sim.runner import Simulation

_HERE = os.path.dirname(os.path.abspath(__file__))


def create_app(sim: Simulation) -> Flask:
    app = Flask(__name__, static_folder=None)

    @app.route("/")
    def index():
        return send_from_directory(os.path.join(_HERE, "static"), "index.html")

    @app.route("/events")
    def events():
        def stream():
            last_id = -1
            while True:
                snap = sim.snapshot(last_id)
                if snap["events"]:
                    last_id = snap["events"][-1]["id"]
                yield f"data: {json.dumps(snap)}\n\n"
                time.sleep(0.05)
        return Response(stream(), mimetype="text/event-stream",
                        headers={"Cache-Control": "no-cache",
                                 "X-Accel-Buffering": "no"})

    @app.route("/action/<name>", methods=["POST"])
    def action(name):
        return jsonify({"ok": sim.owner_action(name)})

    return app
