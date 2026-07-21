"""
Jetson hardware backend — real ``Body``/``Senses`` for Sabo.
============================================================

This is the Phase-2 hardware implementation of the two ``brain/hal.py``
interfaces. The **exact same** ``brain/`` code that runs against the Phase-0
``sim.world.SimWorld`` runs against these classes on the real robot — this file
is the only thing that changes when moving from sim to metal.

    * ``HardwareBody``   — every actuator verb → **Feetech STS3215** serial-bus
      writes via ``servo_channel_map`` (radians → 12-bit position count). The
      LED eyes moved off the servo bus onto a Jetson hardware-PWM pin.
    * ``HardwareSenses`` — BNO085 → ``ImuReading``, 2× VL53L1X →
      ``ProximityReading``, camera → ``CatDetection`` (TensorRT TODO),
      ``now()`` = ``time.monotonic``.

Dev-machine safety (IMPORTANT)
------------------------------
The physical buses/drivers are absent on the Windows dev box. **Every driver
import is guarded**; if a library or bus is missing the backend drops into
**stub mode**: it logs the intended action (position count, target duty, sensor
read) and returns safe neutral values. So::

    from hardware.jetson_backend import HardwareBody, HardwareSenses

imports and runs anywhere, with or without hardware. Check ``body.live`` /
``senses.live`` to see whether real hardware is attached.

Install the real drivers on the Jetson (see ``hardware/README.md``); the STS3215
chain rides a TTL serial bus adapter (USB → ``/dev/ttyUSB0`` or buffered UART →
``/dev/ttyTHS1``); the I2C sensors bind to the 40-pin header's I2C bus (bus 7 on
Orin Nano, ``board.I2C()``).
"""

from __future__ import annotations

import logging
import math
import os
import time

from brain.hal import (BlinkKind, Body, CatDetection, EarPose, Gait,
                       HearingReading, ImuReading, ProximityReading, Senses,
                       SmellReading, TailPose)
from hardware import servo_channel_map as scm
from vision.detector import load_detector
from vision.pipeline import VisionPipeline

log = logging.getLogger("sabo.hardware")

# the 8 load-bearing leg servos — the ones cut when relaxing to go silent at rest
_LEG_SERVOS = {j for leg in scm.FRONT_LEGS + scm.REAR_LEGS for j in scm.LEG_JOINTS[leg]}

_MODELS_DIR = os.path.join(os.path.dirname(__file__), "..", "vision", "models")

# --- Eyes: CSI camera(s) -------------------------------------------------
# Wide-FOV module (see docs/edge_ai_hardware.md). Two identical modules — one per
# eye socket — on the Jetson's 2× MIPI-CSI lanes give a STEREO pair (sensor-id
# 0 = left eye, 1 = right eye). The LEFT eye is the primary detector feed; the
# RIGHT eye, when present, enables stereo disparity to refine cat distance.
CAM_W, CAM_H, CAM_HFOV = 640, 480, 120.0
EYE_BASELINE_M = 0.060           # inter-ocular distance between the two eye cams
# TensorRT/ONNX/PT cat detector weights; if present the YOLO backend loads it,
# otherwise the pipeline runs the stub (returns "no cat").
CAT_WEIGHTS = os.path.join(_MODELS_DIR, "cat_yolov8n.engine")

# --- Ears: 2× MEMS microphones (I2S/PDM) → stereo audio ------------------
# Stereo capture → interaural level/time difference gives sound bearing; a small
# on-device classifier flags a cat meow/call. Left channel = left ear.
MIC_RATE = 16000                 # Hz, mono-per-channel sample rate
MIC_CHANNELS = 2                 # stereo: [left ear, right ear]
MIC_BLOCK = 2048                 # samples/read (~128 ms @ 16 kHz)
EAR_SPACING_M = 0.090            # acoustic baseline between the two ear mics
SPEED_OF_SOUND = 343.0           # m/s
SOUND_FLOOR = 0.02               # RMS below this = "no salient sound"
# Meow/call classifier (e.g. a fine-tuned YAMNet/panns head exported to ONNX).
MEOW_MODEL = os.path.join(_MODELS_DIR, "meow_audio.onnx")

# --- Nose: BME688 gas/VOC "e-nose" (I2C) → scent classifier --------------
# The BME688's gas-resistance signature over its heater profile is classified
# (BSEC or a small trained head) into a coarse scent label.
BME688_ADDR = 0x77               # BME688 default (0x76 if SDO tied low)
SCENT_LABELS = ("cat", "food", "litter", "unknown")


# --------------------------------------------------------------------- bus bring-up
# All hardware imports are guarded. On the dev machine these all fail and we run
# in stub mode; on the Jetson they succeed and we get live buses.

# TTL serial bus adapter for the STS3215 daisy-chain. On the Jetson this is a
# USB adapter (Waveshare Bus Servo Adapter / FE-URT-1 → /dev/ttyUSB0) or a
# buffered 40-pin UART (/dev/ttyTHS1). Overridable via the SABO_SERVO_PORT env.
SERVO_BUS_PORT = os.environ.get("SABO_SERVO_PORT", "/dev/ttyUSB0")


def _try_open_i2c():
    """Return a shared board I2C bus, or None if unavailable (stub mode)."""
    try:
        import board          # type: ignore
        import busio          # type: ignore
        return busio.I2C(board.SCL, board.SDA)
    except Exception as exc:  # ImportError on dev; RuntimeError if no bus present
        log.warning("I2C bus unavailable (%s) — sensors run in STUB mode.", exc)
        return None


class _FeetechBus:
    """Thin wrapper over the Feetech/Waveshare ``scservo_sdk`` (STServo SDK).

    Exposes just what ``HardwareBody`` needs: goal-position write, torque
    enable/disable (compliant silent hold), and position feedback read. Kept
    tiny so the SDK detail stays out of the HAL backend. Never raises to the
    caller — bus errors are logged and swallowed so one flaky servo can't crash
    the control loop.
    """

    def __init__(self, port_handler, packet_handler):
        self._port = port_handler
        self._pkt = packet_handler

    def write_pos(self, servo_id: int, pos: int,
                  speed: int = scm.DEFAULT_SPEED, acc: int = scm.DEFAULT_ACC) -> None:
        try:
            # SDK signature: WritePosEx(id, position, speed, acc)
            self._pkt.WritePosEx(servo_id, int(pos), int(speed), int(acc))
        except Exception as exc:
            log.warning("STS3215 id%d write_pos failed (%s).", servo_id, exc)

    def torque(self, servo_id: int, on: bool) -> None:
        """Enable/disable torque. OFF → the servo goes limp + truly silent
        (compliant hold), the STS3215 win over cutting a PWM line."""
        try:
            # scservo_sdk exposes torque via a control-table write; the STServo
            # helper is often named ``TorqueEnable`` / register ADDR_STS_TORQUE.
            self._pkt.TorqueEnable(servo_id, 1 if on else 0)
        except Exception as exc:
            log.warning("STS3215 id%d torque(%s) failed (%s).", servo_id, on, exc)

    def read_pos(self, servo_id: int):
        """Return the servo's present position count, or None on error."""
        try:
            pos, _comm, _err = self._pkt.ReadPos(servo_id)
            return int(pos)
        except Exception as exc:
            log.warning("STS3215 id%d read_pos failed (%s).", servo_id, exc)
            return None


def _try_open_feetech(port: str = SERVO_BUS_PORT):
    """Open the STS3215 serial bus, or None if unavailable (stub mode)."""
    try:
        from scservo_sdk import PortHandler, sms_sts  # type: ignore
        ph = PortHandler(port)
        if not ph.openPort():
            log.warning("STS3215 bus port %s did not open — servos in STUB mode.", port)
            return None
        ph.setBaudRate(scm.BUS_BAUD)
        return _FeetechBus(ph, sms_sts(ph))
    except Exception as exc:
        log.warning("STS3215 bus unavailable (%s) — servos in STUB mode.", exc)
        return None


def _try_open_led_pwm(led=scm.LED_EYE):
    """Open the Jetson hardware-PWM channel that drives the LED-eye MOSFET, or
    None (stub) on the dev box. Uses Jetson.GPIO's PWM; the sysfs pwmchip is
    picked up from the JetPack pinmux for the header pin."""
    try:
        import Jetson.GPIO as GPIO  # type: ignore
        GPIO.setmode(GPIO.BOARD)
        GPIO.setup(led.board_pin, GPIO.OUT)
        pwm = GPIO.PWM(led.board_pin, 1000)   # 1 kHz — flicker-free eye brightness
        pwm.start(0.0)
        return pwm
    except Exception as exc:
        log.warning("LED-eye PWM (pin %d) unavailable (%s) — eyes in STUB mode.",
                    led.board_pin, exc)
        return None


# ===================================================================== Body
class HardwareBody(Body):
    """Actuator backend: HAL verbs → Feetech STS3215 serial-bus writes.

    All 14 joints daisy-chain on one TTL bus (see ``servo_channel_map``); the
    LED eyes hang off a Jetson hardware-PWM pin (no PWM left on the servo bus).

    Locomotion note: ``gait()`` takes *velocity intents* (m/s, rad/s), not joint
    angles. Turning those into leg trajectories is the job of a gait engine
    (see ``sim/gait.py`` for the reference generator). Until that engine is wired
    in on hardware, ``gait()`` records the intent and holds the current posture;
    ``set_posture()`` is what actually plants the legs.
    """

    def __init__(self, bus=None, port: str = SERVO_BUS_PORT):
        self._bus = bus if bus is not None else _try_open_feetech(port)
        self._led_pwm = _try_open_led_pwm()
        self.live = self._bus is not None

        # commanded state mirror (for logging / gait engine / diagnostics)
        self.gait_mode = Gait.STAND
        self.cmd_forward = 0.0
        self.cmd_yaw = 0.0
        self.front_height = 1.0
        self.rear_height = 1.0
        self.head_pan = 0.0
        self.head_pitch = 0.0
        self.head_tilt = 0.0
        self.ears = EarPose.NEUTRAL
        self.tail = TailPose.MID
        self.tail_wag = 0.0
        self.eyes_open = 1.0
        self.purring = False
        self.relaxed = False           # legs unpowered (silent) while resting

        log.info("HardwareBody init: %s", "LIVE (STS3215 bus)" if self.live else "STUB")
        # Plant a safe neutral stance on boot.
        self.set_posture(1.0, 1.0)
        self.look_at(0.0, 0.0)
        self.set_ears(EarPose.NEUTRAL)
        self.set_tail(TailPose.MID)
        self.set_eyes(1.0)

    # -- low-level bus writes ---------------------------------------
    def _write_servo(self, name: str, angle_rad: float) -> None:
        """Send one servo to ``angle_rad`` (clamped + calibrated to a count)."""
        if self.relaxed and name in _LEG_SERVOS:
            return                                 # resting: torque off / silent
        cal = scm.SERVOS[name]
        pos = cal.angle_to_pos(angle_rad)
        if self.live:
            self._bus.write_pos(cal.servo_id, pos)
        else:
            log.debug("STUB servo %-9s id%-2d %+.3f rad → pos %4d",
                      name, cal.servo_id, angle_rad, pos)

    def read_joint_angle(self, name: str):
        """Position FEEDBACK: read one STS3215's present position → angle (rad),
        or None if the bus is stubbed/unreadable. Extra to the HAL (the brain
        never calls it); handy for the four-bar knee/crank map + diagnostics."""
        cal = scm.SERVOS[name]
        if not self.live:
            return None
        pos = self._bus.read_pos(cal.servo_id)
        return None if pos is None else cal.pos_to_angle(pos)

    def _write_led(self, duty: float) -> None:
        """Set the LED-eye brightness (duty 0..1) on the Jetson PWM pin."""
        duty = max(0.0, min(1.0, duty))
        if self._led_pwm is not None:
            self._led_pwm.ChangeDutyCycle(duty * 100.0)   # Jetson.GPIO PWM = 0..100 %
        else:
            log.debug("STUB led     pin%-2d duty %.2f", scm.LED_EYE.board_pin, duty)

    # -- posture → leg joint angles ---------------------------------
    def _plant_legs(self) -> None:
        """Map front/rear stance heights (0..1) to hip+knee angles per leg.

        First-order kinematic stand-in: at height 1.0 the leg is at its standing
        pose; at 0.0 it is fully crouched. Real per-leg IK lives in the gait
        engine — replace this with a call into it when locomotion is wired up.
        """
        # standing vs crouched joint targets (rad) — retune against the sim/IK.
        HIP_STAND, HIP_CROUCH = 0.35, 1.10
        KNEE_STAND, KNEE_CROUCH = 0.60, 1.80
        for leg in scm.FRONT_LEGS + scm.REAR_LEGS:
            h = self.front_height if leg in scm.FRONT_LEGS else self.rear_height
            h = max(0.0, min(1.0, h))
            hip = HIP_CROUCH + (HIP_STAND - HIP_CROUCH) * h
            knee = KNEE_CROUCH + (KNEE_STAND - KNEE_CROUCH) * h
            hip_name, knee_name = scm.LEG_JOINTS[leg]
            self._write_servo(hip_name, hip)
            self._write_servo(knee_name, knee)

    # -- locomotion --------------------------------------------------
    def gait(self, mode: Gait, forward: float = 0.0, yaw: float = 0.0) -> None:
        self.gait_mode = mode
        self.cmd_forward = forward
        self.cmd_yaw = yaw
        # TODO(gait): feed (mode, forward, yaw) to the gait engine (sim/gait.py
        # reference) to generate per-leg hip/knee trajectories each tick and call
        # self._write_servo(...) for the 8 leg joints. For now we hold posture.
        if not self.live:
            log.debug("STUB gait    %-5s forward=%.3f yaw=%.3f (gait engine TODO)",
                      mode.value, forward, yaw)

    # -- head / eyes -------------------------------------------------
    def look_at(self, bearing: float, tilt: float = 0.0) -> None:
        # bearing -> pan (yaw); tilt -> pitch (nod). Roll (head_tilt) is reserved
        # for the cute head-tilt + the IMU camera-roll gimbal (see gimbal()).
        self.head_pan = bearing
        self.head_pitch = tilt
        self._write_servo("head_pan", bearing)
        self._write_servo("head_pitch", tilt)

    def gimbal(self, roll: float, pitch: float) -> None:
        """2-axis camera stabilization: counter the body's roll/pitch (from the
        BNO085 IMU) on the head so the camera stays level while moving. Called by
        the on-robot stabilization loop (mirrors sim's head_stabilize)."""
        self.head_tilt = roll
        self.head_pitch = pitch
        self._write_servo("head_tilt", roll)
        self._write_servo("head_pitch", pitch)

    def blink(self, kind: BlinkKind) -> None:
        # A blink is a brightness dip-and-recover on the LED eyes. These are
        # non-blocking setpoints (HAL contract), so we issue the terminal state
        # (eyes returned to their openness) and log the blink kind; a fade timer
        # would live in a display/LED task on hardware.
        if not self.live:
            log.debug("STUB blink   %s", kind.value)
        # TODO(led): drive an actual fade (slow ~400ms, quick ~120ms) on the
        # LED-eye channel or a dedicated eye display; here we restore openness.
        self._write_led(self.eyes_open)

    def set_eyes(self, openness: float) -> None:
        self.eyes_open = max(0.0, min(1.0, openness))
        self._write_led(self.eyes_open)

    # -- expressive appendages --------------------------------------
    _EAR_ANGLE = {EarPose.FORWARD: 0.55, EarPose.NEUTRAL: 0.0, EarPose.FLAT: -0.55}
    _TAIL_ANGLE = {TailPose.UP: 1.0, TailPose.MID: 0.0, TailPose.LOW: -1.0,
                   TailPose.PUFFED: 0.9}

    def set_ears(self, pose: EarPose) -> None:
        self.ears = pose
        # EARS_LINKED: one motor; ear_R follows ear_L mechanically.
        self._write_servo("ear_L", self._EAR_ANGLE[pose])

    def set_tail(self, pose: TailPose, wag: float = 0.0) -> None:
        self.tail = pose
        self.tail_wag = max(0.0, min(1.0, wag))
        # Base posture angle; a wag would oscillate around this each tick. Since
        # HAL calls are per-tick setpoints, we bias the base by the wag amplitude
        # so the dashboard/hardware still shows motion intent.
        base = self._TAIL_ANGLE[pose]
        self._write_servo("tail", base + 0.2 * self.tail_wag)

    # -- body pose ---------------------------------------------------
    def set_posture(self, front_height: float, rear_height: float) -> None:
        self.front_height = front_height
        self.rear_height = rear_height
        self._plant_legs()

    # -- sound / haptics --------------------------------------------
    def relax(self, on: bool) -> None:
        """Torque-OFF the leg servos so they go limp + **silent** while resting on
        the ground. On the STS3215 this is a real torque-disable command → truly
        compliant, no holding current, no digital-servo buzz (strictly better than
        cutting a PWM line, which still leaves an analog servo hunting). Only call
        in a grounded rest pose. relax(False) re-enables torque and the next
        posture/gait write drives the legs again."""
        self.relaxed = on
        for name in _LEG_SERVOS:
            if self.live:
                self._bus.torque(scm.SERVOS[name].servo_id, not on)
        if not self.live:
            log.debug("STUB relax   %s (leg servos torque %s)",
                      on, "OFF/silent" if on else "ON")

    def purr(self, on: bool) -> None:
        self.purring = on
        # TODO(haptics): vibration motor on a spare Jetson GPIO/PWM pin, plus
        # a low-frequency rumble on the I2S speaker (MAX98357A).
        if not self.live:
            log.debug("STUB purr    %s", "on" if on else "off")

    def speak(self, clip: str) -> None:
        # TODO(audio): play a wav clip id ('trill'/'meow'/'chirp'/'hiss') or TTS
        # text through the I2S amp. Keep clip ids in sync with brain/voice.py.
        if not self.live:
            log.debug("STUB speak   %r", clip)


# ===================================================================== Senses
class HardwareSenses(Senses):
    """Sensor backend: BNO085 IMU + 2× VL53L1X ToF + camera detector.

    Each driver is opened in a guarded try/except; any that is missing leaves the
    corresponding read in stub mode returning a safe neutral value.
    """

    # down-angled ToF closer than this (m) means the floor dropped away = edge.
    EDGE_THRESHOLD_M = 0.12

    def __init__(self, i2c=None):
        self._i2c = i2c if i2c is not None else _try_open_i2c()
        self._imu = self._open_bno085(self._i2c)
        self._tof_fwd, self._tof_down = self._open_tofs(self._i2c)
        # Eyes: left (primary detector feed) + optional right (stereo depth).
        self._cam = self._open_camera(0)
        self._cam_right = self._open_camera(1)
        self.stereo = self._cam is not None and self._cam_right is not None
        weights = CAT_WEIGHTS if os.path.exists(CAT_WEIGHTS) else None
        self._vision = VisionPipeline(load_detector("auto", weights=weights),
                                      img_w=CAM_W, img_h=CAM_H, hfov_deg=CAM_HFOV,
                                      clock=time.monotonic)
        # Ears: stereo mic stream + meow classifier (both guarded).
        self._audio = self._open_audio()
        self._meow = self._open_meow_classifier()
        # Nose: BME688 e-nose + scent classifier (guarded).
        self._enose = self._open_bme688(self._i2c)
        self.live = any(x is not None
                        for x in (self._imu, self._tof_fwd, self._tof_down,
                                  self._cam, self._audio, self._enose))
        log.info("HardwareSenses init: imu=%s tof_fwd=%s tof_down=%s eyes=%s(stereo=%s) "
                 "detector=%s audio=%s meow=%s enose=%s",
                 bool(self._imu), bool(self._tof_fwd), bool(self._tof_down),
                 bool(self._cam), self.stereo, self._vision.detector.name,
                 bool(self._audio), bool(self._meow), bool(self._enose))

    @staticmethod
    def _open_camera(sensor_id: int = 0):
        """Open one eye camera via OpenCV, or None (stub) on the dev box.

        On the Jetson, ``sensor_id`` selects the MIPI-CSI lane (0 = left eye,
        1 = right eye); a GStreamer/``nvarguscamerasrc`` pipeline would replace
        the plain index for the real CSI modules. On the dev box a single USB
        webcam may answer index 0 and index 1 stays closed → mono / no stereo.
        """
        try:
            import cv2  # type: ignore
            if not hasattr(cv2, "VideoCapture"):
                return None
            cap = cv2.VideoCapture(sensor_id)
            return cap if cap.isOpened() else None
        except Exception as exc:
            log.warning("eye camera %d unavailable (%s) — STUB.", sensor_id, exc)
            return None

    def _grab_frame(self):
        if self._cam is None:
            return None
        ok, frame = self._cam.read()
        return frame if ok else None

    # -- driver bring-up (guarded) ----------------------------------
    @staticmethod
    def _open_bno085(i2c):
        if i2c is None:
            return None
        try:
            from adafruit_bno08x import (BNO_REPORT_ACCELEROMETER,  # type: ignore
                                         BNO_REPORT_ROTATION_VECTOR)
            from adafruit_bno08x.i2c import BNO08X_I2C  # type: ignore
            imu = BNO08X_I2C(i2c, address=0x4A)
            imu.enable_feature(BNO_REPORT_ROTATION_VECTOR)
            imu.enable_feature(BNO_REPORT_ACCELEROMETER)
            return imu
        except Exception as exc:
            log.warning("BNO085 unavailable (%s) — imu() in STUB mode.", exc)
            return None

    @staticmethod
    def _open_tofs(i2c):
        """Bring up two VL53L1X. They share address 0x29, so the second is moved
        to 0x30 after asserting XSHUT — see README. Returns (forward, down)."""
        if i2c is None:
            return None, None
        try:
            import adafruit_vl53l1x  # type: ignore
            # NOTE(wiring): with XSHUT tied high on both, only one is addressable.
            # On real hardware, hold one in reset at boot, re-address the other to
            # 0x30, then release. Here we optimistically open the two addresses.
            fwd = adafruit_vl53l1x.VL53L1X(i2c, address=0x29)
            fwd.start_ranging()
            try:
                down = adafruit_vl53l1x.VL53L1X(i2c, address=0x30)
                down.start_ranging()
            except Exception:
                down = None
            return fwd, down
        except Exception as exc:
            log.warning("VL53L1X unavailable (%s) — proximity() in STUB mode.", exc)
            return None, None

    @staticmethod
    def _open_audio():
        """Open the stereo (2× MEMS mic) input stream, or None (stub) on dev.

        On the Jetson the two I2S/PDM ear mics present as a 2-channel ALSA
        capture device; ``sounddevice`` (PortAudio) opens it. With no library /
        no device we return None and ``hearing()`` reports silence.
        """
        try:
            import sounddevice as sd  # type: ignore
            stream = sd.InputStream(channels=MIC_CHANNELS, samplerate=MIC_RATE,
                                    blocksize=MIC_BLOCK, dtype="float32")
            stream.start()
            return stream
        except Exception as exc:
            log.warning("stereo mics unavailable (%s) — hearing() in STUB mode.", exc)
            return None

    @staticmethod
    def _open_meow_classifier():
        """Load the meow/call ONNX classifier, or None (stub → meow=False)."""
        if not os.path.exists(MEOW_MODEL):
            return None
        try:
            import onnxruntime as ort  # type: ignore
            sess = ort.InferenceSession(MEOW_MODEL,
                                        providers=["CPUExecutionProvider"])
            return sess
        except Exception as exc:
            log.warning("meow classifier unavailable (%s) — meow=False.", exc)
            return None

    @staticmethod
    def _open_bme688(i2c):
        """Bring up the BME688 gas/VOC e-nose, or None (stub → scent 'none')."""
        if i2c is None:
            return None
        try:
            import adafruit_bme680  # type: ignore  (BME688 uses the 680 driver)
            sensor = adafruit_bme680.Adafruit_BME680_I2C(i2c, address=BME688_ADDR)
            sensor.set_gas_heater(320, 150)   # 320 °C for 150 ms — VOC profile
            return sensor
        except Exception as exc:
            log.warning("BME688 e-nose unavailable (%s) — smell() in STUB mode.", exc)
            return None

    @staticmethod
    def _open_scent_classifier():
        """Load the scent ONNX classifier if trained, else None (heuristic map)."""
        path = os.path.join(_MODELS_DIR, "scent.onnx")
        if not os.path.exists(path):
            return None
        try:
            import onnxruntime as ort  # type: ignore
            return ort.InferenceSession(path, providers=["CPUExecutionProvider"])
        except Exception:
            return None

    # -- Senses interface -------------------------------------------
    def camera(self) -> CatDetection:
        # Eyes → cat detection. Grab the LEFT-eye frame and run it through the
        # vision pipeline (detector → best box → bearing/distance/approach). On
        # the dev box there is no camera and the stub detector returns "no cat",
        # so the brain idles calmly; on the Jetson, drop a YOLO .engine at
        # CAT_WEIGHTS and it goes live. IMU-based EIS (docs/camera_stabilization.md)
        # would stabilise the frame before detection — TODO once frames are real.
        det = self._vision.update(self._grab_frame())
        # Stereo eyes: when the right-eye camera is present, block disparity on
        # the detected cat region refines det.distance (bbox-height range is
        # coarse). Needs rectified/calibrated pairs — TODO once frames are real;
        # until then the mono pipeline's estimate stands and the ToF backstops it.
        if self.stereo and det.present:
            self._cam_right.read()   # keep the right eye in lock-step with left
        return det

    def imu(self) -> ImuReading:
        if self._imu is None:
            return ImuReading()  # neutral: level, no jostle
        try:
            qi, qj, qk, qr = self._imu.quaternion
            # tilt = angle of the gravity/up axis away from vertical.
            # roll & pitch from the quaternion; tilt magnitude = hypot(roll,pitch).
            roll = math.atan2(2 * (qr * qi + qj * qk),
                              1 - 2 * (qi * qi + qj * qj))
            pitch = math.asin(max(-1.0, min(1.0, 2 * (qr * qj - qk * qi))))
            tilt = math.hypot(roll, pitch)
            ax, ay, az = self._imu.acceleration           # includes gravity
            accel = abs(math.sqrt(ax * ax + ay * ay + az * az) - 9.81)
            return ImuReading(tilt=tilt, accel=accel)
        except Exception as exc:
            log.warning("imu() read failed (%s) — returning neutral.", exc)
            return ImuReading()

    def proximity(self) -> ProximityReading:
        ahead = float("inf")
        edge_ahead = False
        if self._tof_fwd is not None:
            try:
                mm = self._tof_fwd.distance      # cm in library units → *10 = mm
                if mm is not None:
                    ahead = mm / 100.0           # library returns cm → metres
            except Exception as exc:
                log.warning("forward ToF read failed (%s).", exc)
        if self._tof_down is not None:
            try:
                mm = self._tof_down.distance
                if mm is not None and (mm / 100.0) > self.EDGE_THRESHOLD_M:
                    edge_ahead = True            # floor fell away → cliff/edge
            except Exception as exc:
                log.warning("down ToF read failed (%s).", exc)
        return ProximityReading(ahead=ahead, edge_ahead=edge_ahead)

    def hearing(self) -> HearingReading:
        """Ears (2× MEMS mics) → loudness + bearing + meow flag.

        Stub-safe: with no mic stream we report silence (HAL default). Live, we
        pull one stereo block, take the loudness (RMS) as ``level``, estimate
        the source ``bearing`` from the interaural time difference (GCC-PHAT
        cross-correlation of L vs R, delay → angle), and run the block through
        the meow classifier (if loaded) to set ``meow``.
        """
        if self._audio is None:
            return HearingReading()          # silence
        try:
            import numpy as np
            block, _ = self._audio.read(MIC_BLOCK)
            block = np.asarray(block, dtype=np.float32)
            if block.ndim == 1:              # mono fallback: no bearing possible
                left = right = block
            else:
                left, right = block[:, 0], block[:, 1]
            level = float(min(1.0, np.sqrt(np.mean((left + right) ** 2) / 2.0) * 4.0))
            present = level > SOUND_FLOOR
            bearing = self._bearing_from_stereo(left, right) if present else 0.0
            meow = self._classify_meow(left + right) if present else False
            return HearingReading(level=level, bearing=bearing,
                                  meow=meow, present=present)
        except Exception as exc:
            log.warning("hearing() read failed (%s) — returning silence.", exc)
            return HearingReading()

    @staticmethod
    def _bearing_from_stereo(left, right) -> float:
        """Interaural time difference → source bearing (rad, + = kitten's left).

        GCC-PHAT: the lag that maximises the phase-normalised cross-correlation
        of the two ear signals is the time difference of arrival; converted to an
        angle through the mic spacing and the speed of sound (clamped to ±90°)."""
        import numpy as np
        n = len(left)
        L = np.fft.rfft(left, 2 * n)
        R = np.fft.rfft(right, 2 * n)
        cross = L * np.conj(R)
        cross /= np.abs(cross) + 1e-9        # PHAT weighting
        corr = np.fft.irfft(cross, 2 * n)
        max_lag = int(EAR_SPACING_M / SPEED_OF_SOUND * MIC_RATE) + 1
        corr = np.concatenate((corr[-max_lag:], corr[:max_lag + 1]))
        lag = int(np.argmax(corr)) - max_lag
        tdoa = lag / float(MIC_RATE)
        # tdoa = d*sin(theta)/c ; +tdoa (L leads) → source toward the left ear.
        sin_theta = max(-1.0, min(1.0, tdoa * SPEED_OF_SOUND / EAR_SPACING_M))
        return float(math.asin(sin_theta))

    def _classify_meow(self, mono) -> bool:
        """Run the mono block through the meow classifier; False if none loaded."""
        if self._meow is None:
            return False
        try:
            import numpy as np
            x = np.asarray(mono, dtype=np.float32).reshape(1, -1)
            name = self._meow.get_inputs()[0].name
            out = np.asarray(self._meow.run(None, {name: x})[0]).reshape(-1)
            return bool(out[0] > 0.5)        # single meow-probability output
        except Exception as exc:
            log.warning("meow classify failed (%s).", exc)
            return False

    def smell(self) -> SmellReading:
        """Nose (BME688 e-nose) → coarse scent label + intensity.

        Stub-safe: with no sensor we report 'none' (HAL default). Live, we read
        gas resistance + humidity; a trained classifier (if present) maps the
        signature to a label, otherwise a simple gas-resistance heuristic gives
        intensity and leaves the label 'unknown' when a strong VOC is present.
        """
        if self._enose is None:
            return SmellReading()            # no scent
        try:
            gas = float(self._enose.gas)              # ohms; drops as VOCs rise
            # Normalise: clean air ~ high resistance (>150 kΩ); strong VOC << that.
            intensity = float(max(0.0, min(1.0, (150_000.0 - gas) / 150_000.0)))
            present = intensity > 0.1
            if not present:
                return SmellReading(scent="none", intensity=intensity, present=False)
            scent = self._classify_scent(gas, intensity)
            return SmellReading(scent=scent, intensity=intensity, present=True)
        except Exception as exc:
            log.warning("smell() read failed (%s) — returning no scent.", exc)
            return SmellReading()

    def _classify_scent(self, gas: float, intensity: float) -> str:
        """Map an e-nose reading to a coarse scent via the ONNX head, else
        return 'unknown' (a strong VOC we can sense but not yet name)."""
        clf = getattr(self, "_scent_clf", None)
        if clf is None:
            clf = self._open_scent_classifier()
            self._scent_clf = clf
        if clf is None:
            return "unknown"
        try:
            import numpy as np
            x = np.asarray([[gas, intensity]], dtype=np.float32)
            name = clf.get_inputs()[0].name
            out = np.asarray(clf.run(None, {name: x})[0]).reshape(-1)
            return SCENT_LABELS[int(np.argmax(out))]
        except Exception as exc:
            log.warning("scent classify failed (%s).", exc)
            return "unknown"

    def now(self) -> float:
        return time.monotonic()
