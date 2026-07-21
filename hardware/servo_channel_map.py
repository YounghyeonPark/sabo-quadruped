"""
Servo bus-ID map + per-servo calibration for the Feetech STS3215 serial bus.
============================================================================

Sabo drives **14 servos** on a single **TTL serial daisy-chain** (Feetech
STS3215 smart servos). Every servo has a unique **bus ID (1..14)**; they share
one half-duplex TTL line off the bus adapter (see ``jetson_backend.py``). The
LED eyes are **no longer** on the actuator bus — an STS3215 chain can't drive a
PWM LED — so they now live on a Jetson GPIO/PWM pin (see ``LED_EYE`` below).

This module is the single source of truth for:

    * which **serial bus ID** each named actuator answers to, and
    * how a joint **angle (radians)** maps to an STS3215 **position count**.

Nothing here imports a hardware library — it is pure data + math, so it loads
and is unit-testable on the dev machine. ``jetson_backend.py`` consumes it.

Conventions (match ``brain/hal.py`` and ``cad/params.py``)
----------------------------------------------------------
* Angles are **radians**. 0 rad = the joint's mechanical neutral.
* + rotation follows the CAD/HAL right-hand rule (about +z up): for the head,
  + bearing = the kitten's left; for the waist, + = arch up.
* Soft limits (``lo_rad``/``hi_rad``) come straight from ``cad/params.py`` joint
  limits so the brain can never command a position that fights a mechanical stop.

STS3215 position model (per servo)
----------------------------------
The STS3215 is a 12-bit, single-turn servo: position is an integer **count**
``0..4095`` spanning its full ~360° travel, centred at **2048** (~180°, the
joint's neutral). The map is linear::

    pos = center + sign * counts_per_rad * clamp(angle, lo_rad, hi_rad)

where ``sign = -1`` if ``invert`` (horn mounted mirror-imaged — the L/R legs and
ears are physically opposite). The result is clamped to the servo's safe
mechanical count window ``[pos_min, pos_max]`` as a last-resort guard.

Because the STS3215 reports position back over the same bus, the inverse
``pos_to_angle()`` turns a feedback read into a joint angle (rad) — used by
``jetson_backend`` to expose cheap joint feedback (e.g. knee/crank angle).

**These constants are first-guess and MUST be re-measured per physical servo**
(assign the ID, mount the horn, sweep position, record the angle). Edit the
numbers here; no other file changes.
"""

from __future__ import annotations

from dataclasses import dataclass

# --------------------------------------------------------------------- bus params
# STS3215 TTL serial daisy-chain. Feetech default is 1 Mbps, half-duplex.
BUS_BAUD = 1_000_000
# 12-bit single-turn position: 0..4095 counts over ~360° of travel, centre 2048.
STS3215_STEPS = 4096
STS3215_CENTER = 2048
STS3215_TRAVEL_RAD = 6.283185307179586          # 2π (full 360° single turn)
_COUNTS_PER_RAD = STS3215_STEPS / STS3215_TRAVEL_RAD   # ~651.9 counts/rad
POS_MIN = 0
POS_MAX = 4095
# Default motion profile written alongside each goal position (0 = servo max).
DEFAULT_SPEED = 0        # 0 = use servo's max speed
DEFAULT_ACC = 0          # 0 = use servo's max accel


@dataclass(frozen=True)
class ServoBusChannel:
    """Calibration for one STS3215 on the serial bus."""
    name: str
    servo_id: int           # unique TTL bus ID, 1..14
    lo_rad: float           # soft lower angle limit (from cad/params.py)
    hi_rad: float           # soft upper angle limit
    center: int = STS3215_CENTER
    counts_per_rad: float = _COUNTS_PER_RAD
    pos_min: int = POS_MIN
    pos_max: int = POS_MAX
    invert: bool = False

    def angle_to_pos(self, angle_rad: float) -> int:
        """Convert a joint angle (rad) to a clamped STS3215 position count."""
        a = min(self.hi_rad, max(self.lo_rad, angle_rad))
        sign = -1.0 if self.invert else 1.0
        pos = self.center + sign * self.counts_per_rad * a
        return int(round(min(self.pos_max, max(self.pos_min, pos))))

    def pos_to_angle(self, pos: int) -> float:
        """Inverse map: an STS3215 position feedback count → joint angle (rad)."""
        sign = -1.0 if self.invert else 1.0
        return sign * (pos - self.center) / self.counts_per_rad

    @property
    def limit_lo_pos(self) -> int:
        """Soft lower angle limit expressed in servo counts (mirrors lo_rad)."""
        return self.angle_to_pos(self.lo_rad)

    @property
    def limit_hi_pos(self) -> int:
        """Soft upper angle limit expressed in servo counts (mirrors hi_rad)."""
        return self.angle_to_pos(self.hi_rad)


@dataclass(frozen=True)
class LedEye:
    """LED eyes — NOT on the servo bus. Re-homed to a Jetson PWM/GPIO pin.

    An STS3215 daisy-chain has no free PWM output, so the eyes moved off the
    (former) PCA9685 channel onto the Jetson's own hardware PWM. The pin sources
    only logic-level current; a small MOSFET/constant-current LED driver buffers
    it to the actual eye LEDs. Duty is 0..1 (brightness / blink fade).
    """
    name: str
    board_pin: int          # Jetson 40-pin header pin number (hardware PWM)
    pwm_chip: int           # sysfs pwmchip index (JetPack pinmux) for the driver
    pwm_channel: int        # channel within that pwmchip


# --------------------------------------------------------------------- limits
# Mirror cad/params.py so a joint can never be driven past a mechanical stop.
LIM_HIP = (-2.6, 2.6)
LIM_KNEE = (0.0, 2.6)             # generic default
# Per-leg knee caps = each leg's real four-bar reach (see cad/params FRONT/REAR
# ['lim_knee']). The rear folds deeper than the front (cat hindlimb > forelimb).
# NOTE(four-bar): the STS3215 drives the knee *crank*; the four-bar linkage
# converts crank angle → knee angle, so the servo count here is the crank-side
# command. Re-measure the crank→knee ratio when calibrating the physical leg.
LIM_KNEE_FRONT = (0.0, 2.62)     # 150.1 deg
LIM_KNEE_REAR = (0.0, 2.79)      # 159.9 deg
LIM_WAIST = (-0.45, 0.65)
LIM_HEAD_PAN = (-1.4, 1.4)    # ±~80°; head-pan not in params.py, chosen for FOV
LIM_HEAD_PITCH = (-0.7, 0.7)  # nod + camera-pitch gimbal
LIM_HEAD_TILT = (-0.7, 0.7)   # roll: cute tilt + camera-roll gimbal
LIM_EAR = (-0.6, 0.6)         # forward(+) .. flat(-)
LIM_TAIL = (-1.2, 1.2)        # low(-) .. up(+); wag rides on top

# --------------------------------------------------------------------- the map
# 14 servos on the daisy-chain, bus IDs 1..14 (ID 0 is the Feetech broadcast /
# unconfigured default, so we start at 1). Physical chain order is chosen to keep
# the wiring trunk short: front legs → rear legs → spine/head → appendages.
#   8 leg (FL/FR/RL/RR × hip+knee) + waist + head_pan + head_pitch + head_tilt
#   + ear_L + tail.  Right-side legs invert (mirror horns).
SERVOS: dict[str, ServoBusChannel] = {
    # ---- legs (hip + knee per leg) --------------------------------------
    "FL_hip":  ServoBusChannel("FL_hip",  1, *LIM_HIP),
    "FL_knee": ServoBusChannel("FL_knee", 2, *LIM_KNEE_FRONT),
    "FR_hip":  ServoBusChannel("FR_hip",  3, *LIM_HIP, invert=True),
    "FR_knee": ServoBusChannel("FR_knee", 4, *LIM_KNEE_FRONT, invert=True),
    "RL_hip":  ServoBusChannel("RL_hip",  5, *LIM_HIP),
    "RL_knee": ServoBusChannel("RL_knee", 6, *LIM_KNEE_REAR),
    "RR_hip":  ServoBusChannel("RR_hip",  7, *LIM_HIP, invert=True),
    "RR_knee": ServoBusChannel("RR_knee", 8, *LIM_KNEE_REAR, invert=True),
    # ---- spine / head (pan + pitch + tilt = 2-axis camera gimbal) -------
    "waist":      ServoBusChannel("waist",      9,  *LIM_WAIST),
    "head_pan":   ServoBusChannel("head_pan",   10, *LIM_HEAD_PAN),
    "head_pitch": ServoBusChannel("head_pitch", 11, *LIM_HEAD_PITCH),
    "head_tilt":  ServoBusChannel("head_tilt",  12, *LIM_HEAD_TILT),
    # ---- expressive appendages -----------------------------------------
    # EARS_LINKED in cad/params.py: one motor drives ear_L; ear_R follows mechanically.
    "ear_L":      ServoBusChannel("ear_L", 13, *LIM_EAR),
    "tail":       ServoBusChannel("tail",  14, *LIM_TAIL),
}

# LED eyes: Jetson hardware PWM on 40-pin header pin 33 (pwmchip0 ch0 on Orin
# Nano's default pinmux) → MOSFET LED driver. blink = fade, set_eyes = duty.
LED_EYE = LedEye("led_eye", board_pin=33, pwm_chip=0, pwm_channel=0)

# Valid STS3215 configurable ID range (ID 0 = broadcast, 254 = broadcast-write).
ID_MIN, ID_MAX = 1, 253

# Named leg groups, handy for posture/gait code.
LEG_JOINTS = {
    "FL": ("FL_hip", "FL_knee"),
    "FR": ("FR_hip", "FR_knee"),
    "RL": ("RL_hip", "RL_knee"),
    "RR": ("RR_hip", "RR_knee"),
}
FRONT_LEGS = ("FL", "FR")
REAR_LEGS = ("RL", "RR")


def all_channels() -> dict[int, str]:
    """Serial bus ID → actuator name (the daisy-chain map). The LED eyes are
    not on the bus (Jetson PWM pin) and are intentionally excluded."""
    return {sc.servo_id: sc.name for sc in SERVOS.values()}


def assert_no_collisions() -> None:
    """Fail fast if two servos were assigned the same TTL bus ID, or an ID is
    outside the addressable STS3215 range."""
    used: dict[int, str] = {}
    for sc in SERVOS.values():
        if not (ID_MIN <= sc.servo_id <= ID_MAX):
            raise ValueError(
                f"servo {sc.name} has bus ID {sc.servo_id} outside "
                f"[{ID_MIN}, {ID_MAX}]"
            )
        if sc.servo_id in used:
            raise ValueError(
                f"STS3215 bus ID {sc.servo_id} used by both "
                f"{used[sc.servo_id]} and {sc.name}"
            )
        used[sc.servo_id] = sc.name


# validate the table at import time — a duplicate ID is a bus-collision bug.
assert_no_collisions()
