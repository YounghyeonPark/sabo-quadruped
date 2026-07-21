"""
RoboKitten perception — raw Senses -> a small world model of facts.
===================================================================

Perception is the layer that turns noisy sensor reads into the handful of
*facts* the mood machine and behaviors reason about. Keeping this separate means
the layers above never care whether a cat sighting came from a sim or a real
camera — they only see a ``WorldModel``.

It also tracks a little short-term memory: how long since we last saw the cat,
how long things have been quiet, and whether the cat *looks* threatened. That
memory is what lets the mood machine (§ mood.py) be steady instead of twitchy.
"""

from __future__ import annotations

from dataclasses import dataclass

from brain.hal import CatDetection, EarPose, Senses

# thresholds (metres / seconds) — tuned for the ~1.5 m sim "room"
CLOSE_DISTANCE = 0.45     # within this the cat is "close" (play range)
NEAR_DISTANCE = 0.9       # within this the cat is "near" (interest range)
LUNGE_SPEED = 0.35        # cat ground speed that reads as a lunge (m/s)
QUIET_FOR_SLEEP = 20.0    # seconds of no cat before we consider napping
CAT_MEMORY = 2.0          # keep believing the cat is around this long after last sight


@dataclass
class WorldModel:
    """The distilled facts. One instance rebuilt each tick (except memory carries)."""
    # cat
    cat_present: bool = False
    cat_distance: float = float("inf")
    cat_bearing: float = 0.0
    cat_speed: float = 0.0
    cat_close: bool = False
    cat_near: bool = False
    cat_approaching: bool = False
    cat_engaged: bool = False     # near + approaching + not threatened
    cat_threat: bool = False      # cat looks hostile: flat ears / hiss / lunge
    # self / environment
    self_tilt: float = 0.0
    jostled: bool = False         # big acceleration (got pounced/knocked)
    edge_ahead: bool = False
    obstacle_ahead: bool = False
    # hearing (ears = stereo mics)
    sound_present: bool = False
    sound_bearing: float = 0.0    # radians toward the dominant sound
    sound_level: float = 0.0      # 0..1
    heard_meow: bool = False
    # smell (nose = gas/VOC e-nose)
    scent: str = "none"           # 'cat' | 'food' | 'litter' | 'unknown' | 'none'
    scent_intensity: float = 0.0
    # memory
    time_since_cat: float = float("inf")
    quiet_time: float = 0.0       # seconds since anything "interesting" happened


class Perception:
    def __init__(self, senses: Senses):
        self._senses = senses
        self._last_seen_t: float | None = None
        self._last_event_t: float | None = None
        self._prev_t: float | None = None

    def update(self) -> WorldModel:
        now = self._senses.now()
        cam = self._senses.camera()
        imu = self._senses.imu()
        prox = self._senses.proximity()
        ears = self._senses.hearing()
        nose = self._senses.smell()

        wm = WorldModel()

        # -- cat facts, with short-term persistence so a 1-frame dropout
        #    doesn't make the kitten "forget" the cat and reset its mood --
        if cam.present:
            self._last_seen_t = now
        seen_recently = (self._last_seen_t is not None
                         and now - self._last_seen_t <= CAT_MEMORY)

        if cam.present or seen_recently:
            wm.cat_present = True
            wm.cat_distance = cam.distance
            wm.cat_bearing = cam.bearing
            wm.cat_speed = cam.speed
            wm.cat_close = cam.distance <= CLOSE_DISTANCE
            wm.cat_near = cam.distance <= NEAR_DISTANCE
            wm.cat_approaching = cam.approaching
            wm.cat_threat = _is_threat(cam)
            # "Engaged" = the cat is relaxed and has itself chosen to be here:
            # either actively approaching, or already close. The kitten only
            # creeps in to ~CLOSE_DISTANCE and then holds, so a cat nearer than
            # that closed the gap on its own — a genuine bid to interact.
            wm.cat_engaged = (wm.cat_near and not wm.cat_threat
                              and (cam.approaching or wm.cat_close))

        wm.time_since_cat = (float("inf") if self._last_seen_t is None
                             else now - self._last_seen_t)

        # -- self / environment --
        wm.self_tilt = imu.tilt
        wm.jostled = imu.accel > 6.0
        wm.edge_ahead = prox.edge_ahead
        wm.obstacle_ahead = prox.ahead < 0.12

        # -- hearing / smell --
        wm.sound_present = ears.present
        wm.sound_bearing = ears.bearing
        wm.sound_level = ears.level
        wm.heard_meow = ears.meow
        wm.scent = nose.scent
        wm.scent_intensity = nose.intensity

        # -- quiet timer: anything salient resets it --
        salient = (wm.cat_present or wm.jostled or wm.edge_ahead
                   or wm.self_tilt > 0.4 or wm.heard_meow or wm.sound_level > 0.4)
        if salient or self._last_event_t is None:
            self._last_event_t = now
        wm.quiet_time = now - self._last_event_t

        return wm


def _is_threat(cam: CatDetection) -> bool:
    """Does the observed cat look hostile / about to strike?"""
    return (cam.hissing
            or cam.ears == EarPose.FLAT
            or (cam.approaching and cam.speed >= LUNGE_SPEED))
