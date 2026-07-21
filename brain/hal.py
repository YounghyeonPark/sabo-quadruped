"""
RoboKitten HAL — Hardware Abstraction Layer
===========================================

This module is the *seam* of the whole project. Everything in ``brain/`` talks
to the robot only through the two interfaces defined here:

    * ``Body``   — actuator channels (what the kitten can *do*)
    * ``Senses`` — sensor reads     (what the kitten can *perceive*)

In Phase 0 these are implemented by the simulator (``sim/world.py``). On the
real Mini Pupper the *same* ``brain/`` code runs unchanged — you only write a
new ``Body``/``Senses`` pair backed by PCA9685 servo writes, a Pi-camera cat
detector, and an MPU6050 IMU. The pybullet sim's own note —
"swap ``set_joint()`` for a PCA9685 servo write" — is exactly this boundary.

Units & conventions
--------------------
* Angles in **radians**, bearings measured from the kitten's nose
  (0 = straight ahead, + = its left, following the right-hand rule about +z up).
* Distances in **metres**.
* ``Gait`` velocities are *intents* (m/s forward, rad/s yaw); the locomotion
  layer below the HAL turns them into leg motion. Phase 0 integrates them
  kinematically in the sim.
* Everything is a plain value object (``dataclass``/``Enum``) so snapshots can
  be serialized straight to the dashboard without adapters.
"""

from __future__ import annotations

import enum
from abc import ABC, abstractmethod
from dataclasses import dataclass, field


# --------------------------------------------------------------------- actuator enums
class Gait(enum.Enum):
    """Locomotion mode. Maps to the gaits proven in the existing sims."""
    STAND = "stand"
    WALK = "walk"
    TROT = "trot"


class EarPose(enum.Enum):
    """Feline ear signal (PLAN §5.1). The loudest 'back off' vs 'I'm game' cue."""
    FORWARD = "forward"   # interested / playful
    NEUTRAL = "neutral"
    FLAT = "flat"         # fear / warning


class TailPose(enum.Enum):
    """Feline tail signal (PLAN §5.1)."""
    UP = "up"             # friendly, confident
    MID = "mid"
    LOW = "low"           # uncertain / submissive
    PUFFED = "puffed"     # alarm — paired with fast motion


class BlinkKind(enum.Enum):
    SLOW = "slow"         # the feline "I love you" — trust builder
    QUICK = "quick"       # ambient / reflexive


# --------------------------------------------------------------------- sensor value objects
@dataclass
class CatDetection:
    """One detected cat, as perception would receive it from a camera.

    In sim this is filled from ground-truth geometry; on hardware a vision model
    fills the same fields. ``ears``/``tail`` model the *observed* body language of
    the real cat — how threatened/relaxed it looks — which the kitten reacts to.
    """
    present: bool = False
    distance: float = float("inf")   # metres, nose-to-cat
    bearing: float = 0.0             # radians, 0 = ahead, + = kitten's left
    speed: float = 0.0               # m/s, cat's ground speed
    approaching: bool = False        # closing distance this tick?
    ears: EarPose = EarPose.NEUTRAL  # observed cat ear posture
    hissing: bool = False


@dataclass
class ImuReading:
    tilt: float = 0.0     # radians off level (roll/pitch magnitude)
    accel: float = 0.0    # m/s^2 magnitude of non-gravity acceleration (jostle)


@dataclass
class ProximityReading:
    ahead: float = float("inf")  # metres to nearest obstacle in front
    edge_ahead: bool = False     # cliff/stair edge detected (drop-off)


@dataclass
class HearingReading:
    """Ears = two MEMS mics. Stereo → sound direction (interaural difference).
    On the Jetson a small audio model can also flag a cat meow / call."""
    level: float = 0.0           # 0..1 loudness of the dominant sound
    bearing: float = 0.0         # radians, 0 = ahead, + = kitten's left
    meow: bool = False           # a cat vocalisation was heard
    present: bool = False        # any salient sound above the noise floor


@dataclass
class SmellReading:
    """Nose = a gas/VOC 'e-nose' (e.g. BME688). A classifier maps the gas
    signature to a coarse scent; intensity is how strong it is."""
    scent: str = "none"          # 'cat' | 'food' | 'litter' | 'unknown' | 'none'
    intensity: float = 0.0       # 0..1
    present: bool = False


# --------------------------------------------------------------------- Body (actuators)
class Body(ABC):
    """Actuator channels. A backend maps these onto sim state or real hardware.

    Calls are *setpoints*: they express desired state each tick, they do not
    block. The Expression layer (``brain/expression.py``) composes cat-language
    moves out of these primitives, so those moves are identical on sim and
    hardware.
    """

    # -- locomotion --------------------------------------------------
    @abstractmethod
    def gait(self, mode: Gait, forward: float = 0.0, yaw: float = 0.0) -> None:
        """Set locomotion mode + velocity intent (m/s forward, rad/s yaw)."""

    # -- head / eyes -------------------------------------------------
    @abstractmethod
    def look_at(self, bearing: float, tilt: float = 0.0) -> None:
        """Aim the head (radians). bearing 0 = straight ahead, + = left."""

    @abstractmethod
    def blink(self, kind: BlinkKind) -> None:
        """Trigger an eye blink (LED-eye fade on hardware)."""

    @abstractmethod
    def set_eyes(self, openness: float) -> None:
        """Static eye openness 0..1 (sleepy=0, wide/aroused ~1)."""

    # -- expressive appendages --------------------------------------
    @abstractmethod
    def set_ears(self, pose: EarPose) -> None: ...

    @abstractmethod
    def set_tail(self, pose: TailPose, wag: float = 0.0) -> None:
        """Tail posture + wag intensity 0..1."""

    # -- body pose (kitten motion vocabulary, PLAN §4.2) ------------
    @abstractmethod
    def set_posture(self, front_height: float, rear_height: float) -> None:
        """Front/rear stance height 0..1 (drives play-bow, sit, crouch, sleep)."""

    # -- sound / haptics --------------------------------------------
    @abstractmethod
    def purr(self, on: bool) -> None:
        """Purr via vibration motor + low speaker (PLAN §5.1)."""

    @abstractmethod
    def speak(self, clip: str) -> None:
        """Emit a sound clip id ('trill', 'meow', 'hiss', or TTS text)."""

    # -- power / quiet ----------------------------------------------
    def relax(self, on: bool) -> None:
        """Relax the load-bearing (leg) servos when the body is resting on the
        ground/frame, so they stop *holding* and go limp + **silent** (kills the
        digital-servo holding buzz — the noise a nearby cat hears most). Only safe
        in grounded rest poses. Concrete no-op default so sim backends ignore it;
        the hardware backend cuts servo PWM. See docs/noise_reduction.md."""


# --------------------------------------------------------------------- Senses (sensors)
class Senses(ABC):
    """Sensor reads. A backend fills these from sim state or real drivers."""

    @abstractmethod
    def camera(self) -> CatDetection:
        """Best current cat detection (present=False if none)."""

    @abstractmethod
    def imu(self) -> ImuReading: ...

    @abstractmethod
    def proximity(self) -> ProximityReading: ...

    @abstractmethod
    def now(self) -> float:
        """Monotonic seconds. Injectable so sim/tests control the clock."""

    # Additional senses — concrete defaults (silence / no scent) so existing
    # backends and tests need not implement them; hardware/sim override as wired.
    def hearing(self) -> HearingReading:
        """Ears (stereo mics) → sound level + direction + meow flag."""
        return HearingReading()

    def smell(self) -> SmellReading:
        """Nose (gas/VOC e-nose) → coarse scent + intensity."""
        return SmellReading()


# --------------------------------------------------------------------- event sink
@dataclass
class Event:
    """A human-readable thing that happened, for the owner dashboard feed."""
    t: float
    kind: str          # 'mood', 'behavior', 'voice', 'cat', 'alert'
    text: str
    extra: dict = field(default_factory=dict)


class EventSink(ABC):
    """Where narratable events go (dashboard feed, console, log)."""

    @abstractmethod
    def emit(self, event: Event) -> None: ...
