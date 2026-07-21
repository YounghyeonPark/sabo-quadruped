# Sabo — Head-Camera Stabilization

## The problem

Sabo's camera is mounted in the head. Because the 11-motor design has **rigid
(non-motorized) hip abduction**, the body can't actively shift its weight
sideways while walking, so the torso — and the head with it — rolls/pitches as
the support pattern changes. Measured in the MuJoCo emulation (`sim/mj_emulate.py`,
which reports a **CAMERA shake** line):

| Mode | Camera shake (peak-to-peak) | RMS |
|---|---|---|
| **watch** (still, looking) | **0.0° pitch / 0.0° roll** | 3.4°* |
| walk | 16.6° pitch / 25.1° roll | 11.6° |
| trot | 7.8° pitch / 20.9° roll | 8.6° |

\* the watch RMS is a small *constant* downward lean from the settled crouch — a
fixed offset, not shake; calibrate it out once.

Reproduce: `python -m sim.mj_emulate --gait watch|walk|trot --seconds 6`.

## Strategy: stabilize the camera, not the body

We deliberately do **not** spend motors forcing the body still. Three layers,
cheapest first — the first two cost **zero extra motors** and are the plan of
record:

### 1. Watch-mode (behavioral) — primary, 0 motors
Cat-detection does not need to happen while walking. The Phase-0 brain already
has **watch** / **freeze** behaviors (`brain/behaviors.py`): when a cat might be
present, Sabo stops and holds a low, settled pose (`sim/gait.py` `PRESETS["watch"]`,
CoM dropped ~14 mm for extra stability). In that pose the camera shake is **~0°**.
So detection runs on clean frames by construction: *look while still, move while
not looking.*

### 2. IMU-based electronic image stabilization (EIS) — for motion, 0 motors
For the frames captured while moving (tracking a fleeing cat), stabilize in
software using the IMU Sabo already carries for balance (MPU6050, PLAN §7):

```
each camera frame:
    read IMU orientation (roll, pitch, yaw rate), timestamped to the frame
    compute the inter-frame rotation of the head
    warp/rotate + crop the frame to cancel it  (keep a ~10-15% crop margin)
```

This is exactly how phones, drones, and action cams do EIS. It removes the
*rotational* shake (the dominant term here) with no moving parts. Notes for the
hardware build:
- **Sync**: hardware-timestamp IMU samples against frame capture (rolling-shutter
  aware); a few ms of skew smears the correction.
- **Sample fast**: run the IMU at ≥200 Hz and integrate gyro between frames.
- **Crop budget**: reserve margin so the stabilized frame never runs past the
  sensor edge at the 25° peaks seen while walking.
- **Good enough**: cat-detection (lightweight model / motion+blob, PLAN §8)
  tolerates residual shake far better than human viewing does.

### 3. Head-tilt gimbal (mechanical) — optional, +1–2 motors
Only if crisp video is needed *while trotting*. Add a head **pitch** (and
optionally **roll**) joint and counter-rotate it from the IMU to hold the camera
level — "chicken-head" stabilization. Costs 1–2 motors (11 → 12/13), so it's a
last resort, not the default. The head already has a pan joint, so a pitch axis
also buys extra expressiveness (nods).

## Recommendation

Ship **watch-mode + IMU EIS** (0 extra motors). Revisit the head gimbal only if
field testing shows Sabo must both move fast *and* see clearly at the same time —
unlikely for a companion that mostly watches, creeps, and pounces.
