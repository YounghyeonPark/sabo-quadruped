# Sabo Hardware HAL Backend (Phase 2)

Real-hardware implementations of the `brain/hal.py` seam. The **unchanged**
`brain/` package runs against these classes on the NVIDIA Jetson Orin Nano Super
exactly as it runs against `sim.world.SimWorld` in Phase 0.

```
brain/  ──uses──▶  brain.hal.Body / brain.hal.Senses  ◀──implements──  hardware/
                          (the seam)
   sim.world.SimWorld  ← Phase 0            hardware.jetson_backend  ← Phase 2
```

## Files

| File | What it is |
|---|---|
| `servo_channel_map.py` | The 14 servos → **STS3215 serial-bus IDs**, with per-servo angle→position-count calibration (+ inverse for feedback). LED-eye now on a Jetson PWM pin. **Pure data/math, no hardware imports.** |
| `jetson_backend.py` | `HardwareBody` (HAL verbs → STS3215 serial-bus writes) and `HardwareSenses` (stereo eyes / BNO085 / VL53L1X ×2 / 2× ear mics → `hearing()` / BME688 e-nose → `smell()`). Every driver import is guarded → **stub mode** when hardware is absent. |
| `run_on_hardware.py` | Runs `RoboKitten` ticks against the backend (real or stub). |

## Stub mode (dev machine)

The physical buses/drivers are **not** present on the Windows dev box, so every
driver import is wrapped in `try/except`. If a library, the serial bus, or the
I2C bus is missing, the backend drops into **stub mode**: it logs intended
actuator writes (STS3215 position count, LED duty) and returns safe neutral
sensor values. The package imports and runs anywhere.

```bash
# Import smoke test (must run cleanly with NO hardware libs installed):
python -c "from hardware.jetson_backend import HardwareBody, HardwareSenses; b=HardwareBody(); s=HardwareSenses(); print('stub-mode OK', s.now())"

# Drive the brain against the stub backend (bonus):
python -m hardware.run_on_hardware            # 10 ticks + state summary
python -m hardware.run_on_hardware --verbose  # + every stub servo/LED write
```

Check `body.live` / `senses.live` to see whether real hardware is attached.

## Running on the real Jetson

1. **Flash JetPack 6** (Ubuntu 22.04); enable I2C on the 40-pin header (bus 7 on
   the Orin Nano; `board.I2C()`/Blinka picks it up) and hardware PWM on pin 33.
2. **Wire the STS3215 serial bus:** all 14 servos daisy-chain on one half-duplex
   TTL line, bus IDs 1..14 (`servo_channel_map.SERVOS`), 1 Mbps, fed by a
   **7.4 V buck/BEC** (STS3215 is 6–7.4 V — do **not** feed it 3S directly) with
   a bulk cap. Connect via a **bus servo adapter** (Waveshare / FE-URT-1 on USB →
   `/dev/ttyUSB0`, or a buffered UART → `/dev/ttyTHS1`). Override the port with
   `SABO_SERVO_PORT`. Assign each servo its ID once with the Feetech tool before
   chaining. `relax()` sends a **torque-OFF** so resting legs go limp + silent.
3. **Wire the I2C sensor bus** (all share SDA/SCL, 3.3 V logic — no PCA9685 now):
   - BNO085 @ `0x4A`
   - VL53L1X ×2 — both power up at `0x29`. Hold one in reset via **XSHUT** at
     boot, re-address the other to `0x30`, then release. `_open_tofs()` opens the
     forward sensor at `0x29` and the down-angled one at `0x30`.
   - BME688 e-nose @ `0x77`.
4. **LED eyes:** Jetson hardware-PWM **pin 33** → MOSFET/LED driver → eye LEDs.
5. **Install the drivers** (see below), then:
   ```bash
   sudo python3 -m hardware.run_on_hardware      # sudo: /dev/i2c + /dev/ttyUSB access
   ```
   `body.live` should now be `True` and servos will move.

### pip packages (real hardware only — do NOT install on the dev box)

```bash
pip install adafruit-blinka                      # board / busio (Jetson GPIO+I2C)
pip install scservo-sdk                          # Feetech/Waveshare STServo (STS3215) bus SDK
pip install Jetson.GPIO                          # LED-eye hardware PWM (pin 33)
pip install adafruit-circuitpython-bno08x        # BNO085 IMU (on-chip fusion)
pip install adafruit-circuitpython-vl53l1x       # VL53L1X ToF (×2)
pip install adafruit-circuitpython-bme680         # BME688 gas/VOC e-nose (nose)
pip install sounddevice                           # 2× I2S MEMS ear mics (hearing)
pip install onnxruntime                           # meow / scent classifier heads + RL gait
```

The eyes/camera path additionally needs the JetPack-provided TensorRT + a built
engine (`trtexec` from an ONNX export of the trained detector) — see
`docs/edge_ai_hardware.md` §5. The `hearing()`/`smell()` classifiers and the
learned RL gait (`training/deploy_policy.py`) load their ONNX from
`vision/models/` when present. None of these packages are needed for stub mode.

## Servo bus map

One TTL serial daisy-chain (STS3215, 1 Mbps), bus IDs 1..14. LED eyes are **off
the servo bus** (Jetson PWM pin 33 + MOSFET).

| ID | Actuator | Limits (rad) | Notes |
|---:|---|---|---|
| 1 | FL_hip | ±2.6 | |
| 2 | FL_knee | 0..2.62 | front four-bar reach cap (150°); ID drives the crank |
| 3 | FR_hip | ±2.6 | `invert` (mirror horn) |
| 4 | FR_knee | 0..2.62 | `invert` |
| 5 | RL_hip | ±2.6 | |
| 6 | RL_knee | 0..2.79 | rear four-bar reach cap (160°), folds deeper |
| 7 | RR_hip | ±2.6 | `invert` |
| 8 | RR_knee | 0..2.79 | `invert` |
| 9 | waist | -0.45..0.65 | spine flex/arch |
| 10 | head_pan | ±1.4 | bearing, + = kitten's left |
| 11 | head_pitch | ±0.7 | nod + camera-pitch gimbal |
| 12 | head_tilt | ±0.7 | quizzical roll + camera-roll gimbal |
| 13 | ear_L | ±0.6 | EARS_LINKED: ear_R follows mechanically |
| 14 | tail | ±1.2 | wag rides on the base angle |
| — | **led_eye** | — | Jetson PWM pin 33 → MOSFET (blink = fade, set_eyes = duty) |

### Calibration

`ServoBusChannel.angle_to_pos()` is linear:
`pos = center + sign·counts_per_rad·clamp(angle, lo, hi)`, then clamped to the
servo's safe `[pos_min, pos_max]` count window (`sign = -1` when `invert`).
Defaults are `center = 2048`, `counts_per_rad ≈ 651.9` (4096 counts / 360°),
`0..4095` guard — **first-guess values**. `pos_to_angle()` is the inverse, used
by `HardwareBody.read_joint_angle()` to turn a servo's **position feedback** into
a joint angle. **Re-measure each physical servo** (assign the ID, sweep position,
record the angle) and edit the constants in `servo_channel_map.py`; no other file
changes. For the knees, also re-check the crank→knee four-bar ratio.

## Known TODOs (marked in code)

- **`camera()`** — plug in the TensorRT cat detector (currently returns
  `present=False`). See `docs/edge_ai_hardware.md` §5 and `camera_stabilization.md`.
- **`gait()`** — feed velocity intents to a gait engine (reference:
  `sim/gait.py`) to generate per-leg trajectories. Today `set_posture()` plants
  the legs and `gait()` records the intent.
- **`purr()` / `speak()`** — vibration motor (spare Jetson GPIO/PWM) and I2S audio
  (MAX98357A) not yet wired.
- **LED blink fade** — `blink()` issues the terminal openness; a real fade timer
  belongs in an LED/display task.
