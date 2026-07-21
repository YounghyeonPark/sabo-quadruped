"""
RoboKitten expression — speak *cat* (and a little human).
=========================================================

High-level, hardware-independent "verbs" composed from ``Body`` primitives +
``Voice``. Behaviors (``brain/behaviors.py``) call these; they never touch the
Body directly. Because these are built only on the HAL, a play-bow looks the
same in sim and on the real robot.

Posture constants mirror the ``BEH`` table in ``sim/robokitten_sim.html`` so the
kitten's shapes match the already-tuned visual sim (front/rear stance heights as
fractions of nominal). The cat-signal semantics mirror PLAN §5.1.
"""

from __future__ import annotations

from brain.hal import BlinkKind, Body, EarPose, Gait, TailPose
from brain.voice import Voice

# front/rear stance heights (0..1 of nominal), lifted from the sim BEH table
POSTURE = {
    "stand": (1.00, 1.00),
    "alert": (1.02, 0.98),
    "bow":   (0.46, 1.10),   # front down, rear up — "let's play!"
    "crouch": (0.55, 0.55),  # low, ready to spring or freeze
    "sit":   (1.02, 0.40),
    "sleep": (0.32, 0.32),
}


class Expression:
    """Cat-language + locomotion verbs. Stateless w.r.t. the world; pure output."""

    def __init__(self, body: Body, voice: Voice):
        self.body = body
        self.voice = voice

    # -- eyes --------------------------------------------------------
    def slow_blink(self) -> None:
        """The feline 'I love you' — the cheapest, highest-impact trust builder."""
        self.body.blink(BlinkKind.SLOW)

    def wide_eyes(self) -> None:
        self.body.set_eyes(1.0)

    def sleepy_eyes(self) -> None:
        self.body.set_eyes(0.15)

    # -- ears / tail (feline signaling) -----------------------------
    def ears(self, pose: EarPose) -> None:
        self.body.set_ears(pose)

    def tail(self, pose: TailPose, wag: float = 0.0) -> None:
        self.body.set_tail(pose, wag)

    def friendly_signal(self) -> None:
        """Tail up + ears forward — 'safe to approach'."""
        self.body.set_ears(EarPose.FORWARD)
        self.body.set_tail(TailPose.UP, wag=0.2)

    def wary_signal(self) -> None:
        """Ears flat + tail low/puffed — 'give me space'."""
        self.body.set_ears(EarPose.FLAT)
        self.body.set_tail(TailPose.PUFFED, wag=0.8)

    # -- posture / motion vocabulary (PLAN §4.2) --------------------
    def stand(self) -> None:
        self._posture("stand")
        self.body.gait(Gait.STAND)

    def play_bow(self) -> None:
        """Front down, rear up: universal 'let's play' invitation."""
        self._posture("bow")
        self.body.gait(Gait.STAND)
        self.body.set_ears(EarPose.FORWARD)
        self.body.set_tail(TailPose.UP, wag=0.9)

    def sit(self) -> None:
        self._posture("sit")
        self.body.gait(Gait.STAND)

    def curl_sleep(self) -> None:
        self._posture("sleep")
        self.body.gait(Gait.STAND)
        self.sleepy_eyes()
        self.body.set_ears(EarPose.NEUTRAL)
        self.body.set_tail(TailPose.LOW)

    def crouch_freeze(self) -> None:
        """Startle-freeze: low crouch, hold still (PLAN §4.2)."""
        self._posture("crouch")
        self.body.gait(Gait.STAND)

    # -- locomotion intents -----------------------------------------
    def walk_toward(self, bearing: float, speed: float = 0.08) -> None:
        """Approach something slowly — yaw toward it while creeping forward."""
        self._posture("alert")
        self.body.look_at(bearing)
        self.body.gait(Gait.WALK, forward=speed, yaw=_yaw_for(bearing))

    def scamper_toward(self, bearing: float, speed: float = 0.18) -> None:
        """Quick playful dart (trot) toward a target — the pounce approach."""
        self._posture("alert")
        self.body.look_at(bearing)
        self.body.gait(Gait.TROT, forward=speed, yaw=_yaw_for(bearing))

    def back_away(self, bearing: float, speed: float = 0.10) -> None:
        """Retreat from a threat at ``bearing`` (walk backward, keep facing it)."""
        self.body.look_at(bearing)
        self.body.gait(Gait.WALK, forward=-speed, yaw=_yaw_for(bearing) * 0.3)

    def wander(self, yaw: float, speed: float = 0.06) -> None:
        self._posture("alert")
        self.body.gait(Gait.WALK, forward=speed, yaw=yaw)

    def wiggle_in_place(self, yaw: float) -> None:
        """The pre-pounce rear-end shimmy: crouch and jitter yaw without moving."""
        self._posture("crouch")
        self.body.gait(Gait.STAND, forward=0.0, yaw=yaw)

    def look(self, bearing: float) -> None:
        """Turn the head toward ``bearing`` without moving the feet."""
        self.body.look_at(bearing)

    def halt(self) -> None:
        self.body.gait(Gait.STAND)

    # -- sound / haptics --------------------------------------------
    def purr(self, on: bool) -> None:
        self.body.purr(on)

    def relax(self, on: bool) -> None:
        """Go limp + silent in a grounded rest pose (no servo holding buzz)."""
        self.body.relax(on)

    def trill(self) -> None:
        self.voice.trill()

    def meow(self) -> None:
        self.voice.meow()

    def chirp(self) -> None:
        self.voice.chirp()

    def hiss(self) -> None:
        self.voice.hiss()

    # -- internals ---------------------------------------------------
    def _posture(self, name: str) -> None:
        fh, rh = POSTURE[name]
        self.body.set_posture(fh, rh)


def _yaw_for(bearing: float) -> float:
    """Proportional turn-toward: bearing (rad, + = left) -> yaw rate intent."""
    # gentle P-controller; clamp so the kitten never spins alarmingly fast
    k = 1.5
    y = k * bearing
    return max(-1.2, min(1.2, y))
