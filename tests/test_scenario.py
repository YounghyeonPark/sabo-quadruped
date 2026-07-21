"""End-to-end scenarios driving the full Simulation (brain + sim cat).

These assert the *emergent* loop the whole project is about: gentle conduct earns
the cat's trust and draws it in; a lunge scares the kitten into a freeze/retreat;
and the kitten recovers once the threat passes.
"""

import math

from brain.mood import Mood
from sim.runner import Simulation

DT = 1 / 30


def run(sim, seconds, before_tick=None):
    """Advance the sim, returning (moods seen, event texts seen)."""
    moods, texts = set(), []
    last_id = -1
    for _ in range(int(seconds / DT)):
        if before_tick:
            before_tick(sim.world)
        sim.tick(DT)
        snap = sim.snapshot(last_id)
        moods.add(snap["brain"]["mood"])
        for e in snap["events"]:
            last_id = e["id"]
            texts.append(e["text"])
    return moods, texts


def test_kitten_notices_and_greets_a_friendly_cat():
    sim = Simulation(seed=7)
    # a friendly cat, in view but far enough that the kitten works through the
    # curious/watch phase (creeping + slow-blinking) before any play
    sim.world.cat.x, sim.world.cat.y = 0.6, 0.0
    sim.world.cat.trust = 0.6
    moods, texts = run(sim, 14)
    # it should socially engage rather than stay idle...
    assert "curious" in moods, moods
    # ...and perform the trust-building slow-blink at the cat
    assert any("slow-blink" in t for t in texts), texts


def test_lunge_scares_the_kitten():
    sim = Simulation(seed=3)

    def lunge(world):
        # cat right in the kitten's face, spitting mad
        world.cat.x = world.kx + 0.25 * math.cos(world.kh)
        world.cat.y = world.ky + 0.25 * math.sin(world.kh)
        world.cat.startle = 1.0

    moods, texts = run(sim, 1.5, before_tick=lunge)
    assert "scared" in moods, moods
    assert any("startled" in t.lower() or "freez" in t.lower() for t in texts), texts


def test_kitten_recovers_after_the_threat_passes():
    sim = Simulation(seed=3)

    def lunge(world):
        world.cat.x = world.kx + 0.25
        world.cat.y = world.ky
        world.cat.startle = 1.0

    run(sim, 1.0, before_tick=lunge)
    assert sim.brain.mood_machine.mood == Mood.SCARED

    # threat leaves: send the cat to the far corner and let it calm down
    def retreat(world):
        world.cat.x, world.cat.y = 1.4, 0.9
        world.cat.startle = 0.0

    run(sim, 6.0, before_tick=retreat)
    assert sim.brain.mood_machine.mood != Mood.SCARED


def test_gentleness_builds_trust():
    sim = Simulation(seed=11)
    sim.world.cat.x, sim.world.cat.y = 0.0, 0.0
    sim.world.cat.trust = 0.4
    start = sim.world.cat.trust
    run(sim, 10)
    assert sim.world.cat.trust > start, (start, sim.world.cat.trust)
