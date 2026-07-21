"""Mood state-machine transitions (PLAN §8)."""

from brain.mood import DEBOUNCE, Mood, MoodMachine
from brain.perception import QUIET_FOR_SLEEP, WorldModel
from tests.conftest import Clock


def _settle(mm, wm, clock, seconds=DEBOUNCE + 0.1, step=0.05):
    """Hold a world model steady past the debounce window."""
    n = int(seconds / step)
    for _ in range(n):
        clock.advance(step)
        mm.update(wm)
    return mm.mood


def test_threat_triggers_scared_instantly():
    clock = Clock()
    mm = MoodMachine(clock, start=Mood.CALM)
    wm = WorldModel(cat_present=True, cat_near=True, cat_threat=True)
    assert mm.update(wm) == Mood.SCARED   # no debounce for safety


def test_jostle_triggers_scared():
    clock = Clock()
    mm = MoodMachine(clock)
    assert mm.update(WorldModel(jostled=True)) == Mood.SCARED


def test_scared_is_sticky_then_recovers():
    clock = Clock()
    mm = MoodMachine(clock)
    mm.update(WorldModel(cat_threat=True))
    assert mm.mood == Mood.SCARED
    # threat gone, but it should NOT immediately flip back
    clock.advance(0.5)
    assert mm.update(WorldModel(cat_present=False)) == Mood.SCARED
    # after the settle + min-dwell window it recovers toward CALM
    _settle(mm, WorldModel(cat_present=False), clock, seconds=5.0)
    assert mm.mood in (Mood.CALM, Mood.SLEEPY)


def test_engaged_cat_makes_it_playful():
    clock = Clock()
    mm = MoodMachine(clock, start=Mood.CURIOUS)
    wm = WorldModel(cat_present=True, cat_near=True, cat_close=True,
                    cat_approaching=True, cat_engaged=True)
    assert _settle(mm, wm, clock) == Mood.PLAYFUL


def test_present_but_wary_cat_stays_curious_not_playful():
    clock = Clock()
    mm = MoodMachine(clock, start=Mood.CALM)
    # cat is close but NOT engaged (not approaching) -> don't pounce on it
    wm = WorldModel(cat_present=True, cat_near=True, cat_close=True,
                    cat_approaching=False, cat_engaged=False)
    assert _settle(mm, wm, clock) == Mood.CURIOUS


def test_long_quiet_goes_sleepy():
    clock = Clock()
    mm = MoodMachine(clock, start=Mood.CALM)
    wm = WorldModel(cat_present=False, quiet_time=QUIET_FOR_SLEEP + 1)
    assert _settle(mm, wm, clock) == Mood.SLEEPY


def test_owner_override_forces_mood():
    clock = Clock()
    mm = MoodMachine(clock, start=Mood.SLEEPY)
    mm.force(Mood.CALM, hold=2.0)
    assert mm.mood == Mood.CALM
    # override holds even against a sleepy world for its duration
    clock.advance(1.0)
    assert mm.update(WorldModel(cat_present=False,
                                quiet_time=QUIET_FOR_SLEEP + 1)) == Mood.CALM
