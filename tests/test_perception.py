"""Perception: raw Senses -> WorldModel facts (brain/perception.py)."""

from brain.hal import (CatDetection, EarPose, ImuReading, ProximityReading,
                       Senses)
from brain.perception import CAT_MEMORY, LUNGE_SPEED, Perception
from tests.conftest import Clock


class FakeSenses(Senses):
    def __init__(self, clock):
        self._clock = clock
        self.cat = CatDetection(present=False)
        self._imu = ImuReading()
        self._prox = ProximityReading()

    def camera(self):
        return self.cat

    def imu(self):
        return self._imu

    def proximity(self):
        return self._prox

    def now(self):
        return self._clock()


def test_distance_flags():
    clock = Clock()
    s = FakeSenses(clock)
    p = Perception(s)
    s.cat = CatDetection(present=True, distance=0.3, approaching=True,
                         ears=EarPose.FORWARD)
    wm = p.update()
    assert wm.cat_present and wm.cat_close and wm.cat_near
    assert wm.cat_engaged and not wm.cat_threat


def test_flat_ears_and_lunge_are_threats():
    clock = Clock()
    p = Perception(FakeSenses(clock))
    s = p._senses
    s.cat = CatDetection(present=True, distance=0.5, ears=EarPose.FLAT)
    assert p.update().cat_threat
    s.cat = CatDetection(present=True, distance=0.5, approaching=True,
                         speed=LUNGE_SPEED + 0.1)
    assert p.update().cat_threat


def test_cat_persists_briefly_after_dropout():
    clock = Clock()
    s = FakeSenses(clock)
    p = Perception(s)
    s.cat = CatDetection(present=True, distance=0.6)
    assert p.update().cat_present
    # camera loses the cat for one frame — memory keeps it "present"
    s.cat = CatDetection(present=False)
    clock.advance(CAT_MEMORY * 0.5)
    assert p.update().cat_present
    # ...but not forever
    clock.advance(CAT_MEMORY)
    assert not p.update().cat_present


def test_jostle_from_acceleration():
    clock = Clock()
    s = FakeSenses(clock)
    p = Perception(s)
    s._imu = ImuReading(accel=9.0)
    assert p.update().jostled
