# Sabo — MVP Physical-Build Spec

**Purpose.** Take Sabo from a validated CAD/sim model to a *working physical robot*
on which the **five platform-paper hardware measurements** (`docs/platform.md` §
"Hardware-measured") can be taken. This is the gating deliverable for the paper:
the "quiet" and "compliant/animal-safe" claims cannot be faked in sim.

This document is **actionable** — order the parts, print the frame, bolt it up,
bring up the firmware, and run the measurement protocol. Every quantity is traced
to a source file; nothing here is invented. Items that cannot be derived are marked
`[verify]`.

> Sources of truth cross-checked for this spec: `analysis/bom.py`,
> `hardware/servo_channel_map.py`, `hardware/jetson_backend.py`,
> `cad/params.py`, `cad/parts/{leg,body,split}.py`, `cad/out/print_manifest.json`,
> `docs/{edge_ai_hardware,wiring_pinmap,assembly,platform}.md`.
> BOM cost reconciled against `python -m analysis.bom` → **$711 / $838 / $965**
> (lo/mid/hi). `python -m pytest -q` → 27 passed (no code changed).

---

## 1. Procurement BOM (exact, orderable)

Prices are self-sourced maker USD ranges (~2025–26), reconciled line-for-line to
`analysis/bom.py :: COMPONENTS`. The total below **equals** the generator's
`$711 / $838 / $965`. The added transmission/fastener hardware (§1b) is already
folded into the two "Mechanical" lump lines + the computed filament line, so it
adds **no cost beyond $838 mid** — it is itemized here so you can order the exact
counts instead of guessing.

### 1a. Buy list (traced to `analysis.bom`)

| # | Item | Qty | Unit $ (lo–hi) | Subtotal $ | Source / trace |
|--:|------|:---:|:---:|:---:|----------------|
| **Compute** |
| 1 | Jetson Orin Nano Super Dev Kit (8 GB) — module + compact carrier (Seeed reComputer **J401**), incl. heatsink+fan | 1 | 249 | 249 | bom Compute; edge_ai §1, §7 |
| 2 | NVMe M.2 SSD 256 GB (OS + models/engines) | 1 | 22–35 | 22–35 | bom Compute |
| 3 | Wi-Fi/BT M.2 card (AX210) — app link / telemetry | 1 | 15–22 | 15–22 | bom Compute |
| **Sensors** |
| 4 | Wide-FOV MIPI-CSI camera (IMX219, ~120°) + 15-pin FFC — one per eye socket (stereo) | 2 | 25–35 | 50–70 | bom Sensors; edge_ai §2a CAM0/CAM1 |
| 5 | BNO085 IMU (on-chip fusion) — balance + camera EIS | 1 | 15–25 | 15–25 | bom; scm/jetson_backend `0x4A` |
| 6 | VL53L1X ToF — nose (fwd `0x29`) + chin (down `0x30`) | 2 | 10–16 | 20–32 | bom; jetson_backend `_open_tofs` |
| 7 | I²S MEMS mic (ICS-43434 / SPH0645) — one per ear | 2 | 6–9 | 12–18 | bom; jetson_backend `MIC_CHANNELS=2` |
| 8 | BME688 gas/VOC e-nose (`0x77`) | 1 | 18–28 | 18–28 | bom; jetson_backend `BME688_ADDR` |
| 9 | MAX98357A I²S amp | 1 | 5–8 | 5–8 | bom; edge_ai §2a mouth |
| 10 | Mini speaker 8 Ω | 1 | 2–5 | 2–5 | bom |
| **Actuators** |
| 11 | **Feetech STS3215** serial bus servo (30 kg·cm / 2.94 N·m, 60 g) | **14** | 14–18 | 196–252 | **`servo_channel_map.SERVOS` = 14** (`P.N_SERVOS`) |
| 12 | TTL bus servo adapter (**Waveshare Bus Servo Adapter** / FE-URT-1) → `/dev/ttyUSB0` | 1 | 5–12 | 5–12 | bom; jetson_backend `SERVO_BUS_PORT` |
| 13 | LED-eye driver (MOSFET + eye LEDs) — Jetson PWM **pin 33** | 1 | 2–6 | 2–6 | bom; scm `LED_EYE` |
| **Power** |
| 14 | 3S LiPo 5000 mAh (~55 Wh) | 1 | 25–40 | 25–40 | bom; edge_ai §3 |
| 15 | Buck **5 V / 5 A** (Jetson rail) | 1 | 8–14 | 8–14 | bom; wiring_pinmap power tree |
| 16 | Buck/BEC **7.4 V / ≥15 A** (STS3215 bus rail) | 1 | 12–22 | 12–22 | bom; edge_ai §3 current budget |
| 17 | Bulk cap + XT60 + fuse/switch + wiring/connectors | 1 | 15–30 | 15–30 | bom; edge_ai §3 |
| **Mechanical / fasteners / transmission** |
| 18 | M2/M3 screws + M2 heat-set inserts (assortment) | 1 set | 10–18 | 10–18 | bom; **48× M2 inserts** (§1b) |
| 19 | Servo horns / **Ø3 pins** / **686 bearings** / **Ø6 axles** / **Ø4 dowels** (joint hardware) | 1 set | 12–25 | 12–25 | bom; counts in §1b |
| 20 | TPU for foot pads (grippy toe caps) | 1 | 5–10 | 5–10 | bom; foot_* manifest |
| 21 | Faux-fur / silicone over-skin (optional cosmetic) | 1 | 0–25 | 0–25 | bom |
| 22 | 3D-print filament (~**667 g** PLA/PETG, computed from CAD mass) | 1 | 13.3–18.7 | 13.3–18.7 | `analysis.bom._printed_grams` |
| | **TOTAL (one robot)** | | | **$711 / $838 / $965** | matches `analysis.bom.cost_totals()` |

*One-time tools (excluded from build cost, per `analysis.bom.ONE_TIME_TOOLS`):*
LiPo balance charger $20–40; 3D printer + soldering iron (assumed owned). Add an
**M2 heat-set insert tip** for the soldering iron and a **calibrated SPL meter**
(§5) — the SPL meter is measurement equipment, not part of the robot.

### 1b. Added hardware — exact counts (derived from CAD, folded into lines 18–19, 22)

These are the quantities to actually order for the lump "Mechanical" lines. All
derived from the parametric CAD, not guessed:

| Item | Qty | Derivation |
|------|:---:|-----------|
| **M2 heat-set inserts** | **48** | `print_manifest.json :: totals.heat_set_inserts_total`; breakdown: torso_fore 4 + head 4 + tail 4 + ear 2×2 + upper 4×4 + crank 4×4 = 48 |
| **Ø6 hip drive axles** (hardened steel / CF rod) | **4** | one `hip_axle()` per leg (`leg.py :: leg_parts`), count = # hips = 4; `P.AXLE_R = 3.0` → Ø6 |
| **686-class ball bearings** (Ø6 bore / Ø13 OD / 5 mm) | **8** | **2 per hip**: core-wall #1 (`body.py :: _core_hip_drive`) + bracket #2 (`leg.py :: hip_bracket`) × 4 hips; `P.HIP_BEARING` |
| **Ø3 pivot pins** (dowel or M3 shoulder screw) + e-clips | **16** (+16 clips) | **4 per leg** × 4 legs: crank–coupler, coupler–rocker, passive knee, ankle (`leg.py` `_pin_bore`/`_pivot_seat`); `P.PIN_R = 1.5` → Ø3 |
| **Ø4 split alignment dowel rods** (steel / PLA) | **9** | 3 per split part × 3 parts: torso_fore (2 spine + 1 keel) + torso_aft (2 spine + 1 keel) + head (3 on Ø78 circle); `P.SPLIT_DOWEL_R = 2.0` (`split.py`, assembly §3) |
| **M2 machine screws** (into inserts) | ~48 | one per insert (horn bolt circles + waist pad + neck stub + ear/tail bases) |
| **M2 servo case screws** | ~40–56 | `fasteners.servo_case_screws` = 4/servo × 14; STS3215 flange pattern **`[verify]`** vs datasheet (assembly §7) |
| **TPU foot pads** | 4 | `foot_F`×2 + `foot_R`×2 paw pads (ideally TPU; PETG OK for first build) |

> Metal axles, bearings, pins and dowels are **not** in the plastic mass budget
> (they are steel/CF), matching `leg.py :: hip_axle` docstring ("not in the plastic
> mass budget, like the four-bar pins + split dowels"). Cost sits inside line 19
> ($12–25). If self-sourcing 8× 686 bearings + hardened Ø6 rod runs to the top of
> that range, line 19 is the one to watch — it does not change the $838 mid total.

---

## 2. Print plan

From `cad/out/print_manifest.json` — **19 distinct parts**, 6 split sub-parts,
**48× M2 inserts**, 11 parts need support, 3 flagged for splitting.

### 2a. Frame parts (print these)

| Part | Qty | Material | Perims / infill | Support | Orientation |
|------|:---:|:---:|:---:|:---:|-------------|
| torso_fore *(SPLIT)* | 1→2 | PETG | 3 / 18% | yes→no | sagittal L/R, cut face on bed, hoops arch up |
| torso_aft *(SPLIT)* | 1→2 | PETG | 3 / 18% | yes→no | sagittal L/R, cut face on bed |
| head *(SPLIT)* | 1→2 | PLA | 3 / 10% | light | equator Z=8, cut face down, dome up |
| ear | 2 | PLA | 3 / 20% | no | blade flat, largest face down |
| tail | 1 | PLA | 3 / 20% | no | axis along bed, curl up |
| hipbr_F_L / F_R / R_L / R_R | 1 ea (×4) | PETG | 4 / 25% | yes | servo-pocket mouth up |
| upper_F / upper_R | 2 ea (×4) | PETG | 4 / 30% | yes | strut flat, crank pocket up. **Chiral:** `sign` mirrors L/R → print right-leg thighs **mirrored** (`leg.py :: upper_leg` docstring) |
| lower_F / lower_R | 2 ea (×4) | PETG | 4 / 30% | no | strut flat, rocker sideways |
| foot_F / foot_R | 2 ea (×4) | PETG (pad→TPU) | 3 / 30% | yes | on side, pad off bed |
| crank_F / crank_R | 2 ea (×4) | PETG | 4 / 60% | no | flat, link plane down (high stress) |
| pushrod_F / pushrod_R | 2 ea (×4) | PETG | 4 / 60% | no | flat, link plane down |

**Material split:** structural frame (legs, four-bar, brackets, torso) in **PETG**;
cosmetic (head, ears, tail) in **PLA**; paw pads in **TPU**. Cosmetic over-skin
(`shell.py`) is a separate clip-on cover, printed last (PLA).

### 2b. The 3 split parts → print 6 halves + bond (`split.py`, assembly §3)

Each of the 3 big parts is cut at export time; you print **6 halves total** and
bond them into whole parts **first**, before any bolt-up:

- **head** → `head_A` (lower face bowl: both eyes + camera bore + muzzle + neck
  stub, 4× M2 inserts, open top) + `head_B` (plain cap). Cut at equator Z=8,
  **above the eye tops (z=+7)** so no eye/camera feature straddles the seam.
  **Seat and wire the camera + LED eyes into `head_A` before bonding the cap.**
- **torso_fore** → `_L` / `_R` (sagittal Y=0). 4× M2 waist-horn inserts straddle
  the cut — melt them in **after** bonding.
- **torso_aft** → `_L` / `_R` (sagittal Y=0). Waist servo pocket is split → the
  waist servo is **captured during bonding**.

Registration: **Ø4 dowel rods** (9 total), press-fit one half / slip the other.
Bond with thin CA (PLA head) or 5-min epoxy (PETG torso); relief channels take
squeeze-out; clamp until cured.

### 2c. Filament + time estimate

- **Filament to order: ~667 g** PLA/PETG (`analysis.bom._printed_grams`):
  frame **360.3 g** (`parts_manifest.totals.printed_plastic_g`) + cosmetic skin
  **~205 g** + **×1.18** supports/purge/failure waste. Buy **1 kg PETG + 1 kg PLA**
  spools (structural vs cosmetic) + a short length of TPU for the 4 paw pads.
  Exact per-spool PETG/PLA gram split from your slicer — `[verify]` at slice time.
- **Print time: ~40–55 h** total across all parts on a single printer (estimate at
  ~12–14 g/h effective for these small, high-perimeter, high-infill parts; the
  60%-infill four-bar links and 4× hip brackets dominate per-gram time). `[verify]`
  against your slicer's estimate — this is a planning figure, not sliced.

---

## 3. Assembly sequence

From `docs/assembly.md §5`. Fasteners per joint called out inline.

1. **Print + post-process.** Print §2; ream all Ø3 pivot bores + servo-pocket walls;
   a Ø3 pin should rotate freely (0.15 mm `PIN_CLEARANCE`).
2. **Bond the 6 split halves first** (§2b): dowel-rod + glue the torso L/R pairs and
   the two head bowls into whole parts. (Head: install camera + LED eyes into
   `head_A` before capping.)
3. **Heat-set inserts — 48× M2.** Melt into: 2 crank horns/leg (8), 2 upper-leg hip
   horns/leg (8), tail base (4), both ear bases (4), head neck stub (4), fore-half
   waist pad (4). Flush to each boss with the insert tip.
4. **Assign + zero servo IDs** — do this on the bench *before* mounting (see §4,
   CRITICAL). Centre each horn at 2048 counts.
5. **Remote-hip drive** (per hip, ×4): seat the hip servo in the **torso core**
   (`_core_hip_drive`); press **686 bearing #1** into the core wall and **#2** into
   the `hipbr_*` bracket; slide the **Ø6 axle** through both, clamp its inboard hub to
   the core servo horn (4× M2), bolt the outboard Ø20 horn-mimic to the `upper_leg`
   hip pad (4× M2 into inserts + centre screw).
6. **Waist joint:** waist servo body captured in `torso_aft` (M2 case screws), horn
   into the fore-half waist pad (4× M2 into inserts). Bolt fore + aft halves together.
7. **Four-bar leg build** (per leg): mount knee servo in the upper-leg crank boss
   (M2 case screws); bolt `crank` onto its horn (4× M2 + centre). Pin the passive
   **knee** (`upper`↔`lower`, Ø3 + e-clip), link the **pushrod** crank→rocker
   (2× Ø3 + e-clips), pin the **ankle** (`lower`↔`foot`, Ø3 + e-clip). Confirm the
   four-bar sweeps its crank window (−20…81°) without binding. → **4 Ø3 pins/leg**.
8. **Head gimbal:** assemble pan→pitch→tilt (servos 10/11/12); bolt head stub to the
   tilt bracket (4× M2 into neck-stub inserts). Bolt ears + tail onto their horns
   (ear 2× M2 ea; tail 4× M2).
9. **Skin:** clip the cosmetic `shell` halves + head shell over the finished frame.
10. **Electronics bay:** Jetson in the fore back-bay (airflow), 3S LiPo in the aft
    belly bay, bus adapter + speaker in the aft. Daisy-chain the STS3215 bus
    (order: legs 1–8 → waist 9 → head 10–12 → ear 13 → tail 14), route the head
    sensor loom down the neck pass-through.

---

## 4. Bring-up sequence

1. **Flash JetPack 6** (Ubuntu 22.04) to the NVMe; enable I²C bus 7, the 40-pin
   UART (if using `ttyTHS1`) or leave the USB bus adapter (`/dev/ttyUSB0`), and
   PWM on pin 33 (pwmchip0 ch0) via the pinmux.
2. **Install deps:** `scservo_sdk` / STServo SDK (STS3215 bus); `adafruit-blinka`
   + `adafruit-bno08x` + `adafruit-vl53l1x` + `adafruit-bme680` (BME688 uses the 680
   driver); `Jetson.GPIO`; `opencv`, `onnxruntime`/TensorRT, `sounddevice`, `numpy`.
   Confirm `hardware/jetson_backend.py` imports and reports `live=True` per bus.
3. **CRITICAL — re-ID every servo, one at a time.** Every STS3215 ships as **ID 1**.
   Connect **one servo at a time** to the bus adapter and write its target ID before
   chaining, per `servo_channel_map.all_channels()` (ID→name): FL_hip=1, FL_knee=2,
   FR_hip=3, FR_knee=4, RL_hip=5, RL_knee=6, RR_hip=7, RR_knee=8, waist=9,
   head_pan=10, head_pitch=11, head_tilt=12, ear_L=13, tail=14. Chaining two
   factory-default (ID 1) servos = a bus-ID collision → nothing addressable. After
   each ID assignment, set the servo center (2048) and write the soft limits from
   `scm.SERVOS[name].lo_rad/hi_rad`.
4. **Servo zero / center / limits.** Mount each horn at 2048 (mechanical neutral).
   The counts in `servo_channel_map.py` are **first-guess** — sweep each physical
   servo, record angle, and re-measure `counts_per_rad` / `center` / `invert` per
   servo. Right-side legs + ears are mirror-mounted → `invert=True` (IDs 3,4,7,8).
5. **ToF XSHUT re-address.** Both VL53L1X boot at `0x29`. Hold one in XSHUT reset at
   boot (spare GPIO), re-address the other to `0x30`, then release — so forward
   (nose, `0x29`) + down (chin, `0x30`) coexist on the one I²C bus
   (`jetson_backend._open_tofs`).
6. **Verify each sensor** in stub→real order: BNO085 quaternion (level reads
   tilt≈0); ToF forward/down distances; both CSI eyes (sensor-id 0/1); stereo mics
   (2-ch capture, GCC-PHAT bearing); BME688 gas resistance; speaker tone.
7. **Run `hardware/run_on_hardware.py`** — stub first (no policy) → real. Confirm the
   Phase-0 brain drives the HAL unchanged: `set_posture` plants legs, `look_at`
   moves the gimbal, `relax(True)` torque-OFFs the 8 leg servos (silent hold).
8. **Stand → walk.** `set_posture(1.0, 1.0)` to stand; then feed a walk gait
   (gait engine / `--policy PATH.onnx`).

**Gotchas:**
- **STS3215 is 6–7.4 V — never wire it to 3S (11.1 V) directly.** The dedicated
  7.4 V buck/BEC feeds the bus rail; abs-max is ~8.4 V.
- **Separate the servo (7.4 V/≥15 A) and Jetson (5 V/5 A) rails** with a bulk cap on
  the servo rail — servo inrush must not brown out the compute rail (reset risk).
  18 AWG for the servo-power trunk.
- **Four-bar knee = crank-side command.** The knee servo (IDs 2/4/6/8) drives the
  *crank*; the linkage converts crank→knee angle. Re-measure the crank→knee ratio
  per physical leg — the `LIM_KNEE_FRONT/REAR` counts are crank-side.
- **Left/right invert.** Mirror-mounted horns on the right legs and ears mean
  `invert=True`; a sign error drives the joint into its stop.
- **Front vs rear knee fold differs** (front cap 2.62 rad / rear 2.79 rad) — a cat's
  hindlimb folds deeper; do not force them symmetric.
- **LED eyes are NOT on the servo bus** — Jetson PWM pin 33 + MOSFET.

---

## 5. Measurement protocol — the 5 hardware axes (the paper's payload)

For each: **equipment · method · log · sim value to beat/compare**. Sim values from
`docs/platform.md` benchmark table (regenerate via `python -m analysis.platform_report`).

### 5.1 Acoustic noise (dB) — the core "quiet" claim
- **Equipment:** calibrated SPL meter (A-weighting, class-2), tripod at **0.5 m**
  from the robot, quiet room (target ambient ≤ ~35 dBA).
- **Method:** measure three conditions — **(a) ambient** (robot off),
  **(b) static hold** (standing, torque ON), **(c) walk gait**. 3 trials × ≥10 s
  each; report LAeq + peak. Keep the SPL meter off the floor to avoid structure-borne
  coupling.
- **Log:** dBA per condition (mean ± sd), peak dBA, ambient floor, distance, gait
  cadence. Optionally the on-robot I²S mics as a secondary channel.
- **Compare:** no sim dB exists (this is the un-fakeable claim). The result *is* the
  contribution — the walk-vs-ambient delta and absolute dBA position Sabo against the
  noisy-hobby-PWM baseline. Also compare **static hold vs `relax(True)` torque-OFF**
  to show the silent-compliant-hold advantage.

### 5.2 Backlash (°) — transmission quality
- **Equipment:** dial indicator (or laser) on the **output link**, reversing hand
  load, servo torque held.
- **Method:** apply a small load one way, read; reverse, read; the angular lost
  motion at the output = backlash. Do it at the **hip** (near-direct: servo horn →
  Ø6 axle) and the **knee** (four-bar: crank → pushrod → rocker) so you **isolate
  four-bar backlash vs servo/axle backlash**. Convert linear indicator reading to °
  via the measured lever radius.
- **Log:** backlash° at hip and knee, lever radius, load magnitude, per leg.
- **Compare:** the MuJoCo four-bar closed loop held to **0.16 mm** (`platform.md`
  Mechanism); the four-bar is invertible/monotonic (transmission angle 41–140°). A
  clean build should show knee backlash dominated by the printed pin fits
  (`PIN_CLEARANCE` 0.15 mm radial), not linkage slop.

### 5.3 Backdrive torque (N·m) — the "compliant / animal-safe" claim
- **Equipment:** lever arm of **known radius** on the joint output + a force gauge
  (or calibrated hanging weights), **servo torque OFF** (`relax`/torque-disable).
- **Method:** with torque disabled, apply force at the lever until the joint just
  begins to backdrive; τ = F × r. Repeat per joint (hip via axle, knee via four-bar).
  Report the threshold torque to move the joint by hand.
- **Log:** backdrive τ (N·m) per joint, lever radius, both directions.
- **Compare:** STS3215 stall = **2.94 N·m** (`platform.md`). The compliance claim
  holds if backdrive τ ≪ stall (a soft, hand-movable joint that yields under contact
  with a live cat). This is the axis that, with §5.1, defines the paper's corner
  ("cheap AND quiet AND backdrivable AND kitten-scale").

### 5.4 Battery runtime (min / mAh) — usability
- **Equipment:** fully-charged **3S 5000 mAh** pack, LiPo charger (for the recharge
  mAh readback), inline power/coulomb meter, a low-voltage-cutoff (LVC) alarm.
- **Method:** from full charge, **walk to LVC** (per-cell ~3.5 V). Log elapsed time
  and the mAh returned on recharge. Optionally repeat for static-idle and
  watch/idle to bracket the range.
- **Log:** runtime (min) to LVC, mAh consumed, mean pack current/power, condition
  (walk vs idle).
- **Compare:** power budget predicts ~25–35 W typical (Jetson ~12 W + servos
  ~10–25 W + sensors ~1 W) on 55 Wh → **~1.5–2 h active** (edge_ai §3). Measured
  walk runtime should land near the low end of that band.

### 5.5 Sim-to-real gap — validates design-as-code
- **Equipment:** on-robot **BNO085** (torso roll), a tape/travel measure or floor
  markers, a level test surface.
- **Method:** run the **same walk gait** used in sim on hardware. Log torso **roll
  (peak-to-peak)** from the IMU and **forward travel** over a fixed number of gait
  cycles. Same for trot if stable.
- **Log:** roll p-p (°), travel (cm) per gait, cadence, number of cycles.
- **Compare (sim, `platform.md` benchmark):** **walk → roll 2.5°, travel 11 cm**
  (stand 0.0°/0 cm; trot 3.1°/48 cm). A small measured-vs-sim gap validates the
  design-as-code model (one param file → CAD + physics + BOM). Camera-shake sim
  references (walk raw/stabilized **1.8° / 0.5°**) are a bonus check on the IMU-EIS
  head gimbal.

---

## 6. Cost + time + risk summary

- **Total build cost:** **$711 / $838 / $965** (lo / **mid** / hi), reconciled to
  `analysis.bom.cost_totals()`. Cost drivers: 14× STS3215 (~$196–252) + Jetson
  ($249) ≈ 55% of the mid build. Excludes one-time tools (charger $20–40; printer +
  iron assumed owned) and the SPL meter (measurement equipment).
- **Time (single builder, single printer):**
  - Print: **~40–55 h** (unattended; `[verify]` at slice).
  - Assembly: **~10–16 h** (48 inserts, 6 split-half bonds + cures, 4 remote-hip
    drives, 4 four-bar legs, gimbal, wiring).
  - Bring-up: **~6–10 h** (JetPack + deps, **14× one-at-a-time servo re-ID**, ToF
    re-address, per-servo center/limit calibration, per-leg crank→knee calibration,
    sensor verify, stand→walk).
- **Top 5 risks / gotchas:**
  1. **Servo-rail voltage / brown-out.** STS3215 is 6–7.4 V (not 3S); servo inrush
     browning the shared rail resets the Jetson. → dedicated 7.4 V/≥15 A BEC +
     bulk cap + 18 AWG trunk, rails kept separate.
  2. **Servo ID collisions.** Every STS3215 is ID 1 out of the box — re-ID **one at
     a time** before chaining, or the bus is unaddressable.
  3. **Four-bar knee calibration.** The knee command is crank-side; the crank→knee
     ratio must be measured per physical leg or the knee won't hit its target angle
     (and front/rear folds differ, 2.62 vs 2.79 rad).
  4. **Split-part bonding + capture.** The waist servo is captured inside
     `torso_aft` on bond and 4× waist-horn inserts straddle the `torso_fore` seam
     (set after bonding); camera + LED eyes must be seated in `head_A` before the
     cap goes on. Get the sequence wrong and you re-print or re-open a part.
  5. **STS3215 flange-hole pattern is indicative** in the CAD (`assembly.md §7`) —
     verify the servo-case-screw pattern against the datasheet before driving M2
     screws into metal. `[verify]`

### Open `[verify]` items
- Per-spool PETG vs PLA gram split (§2c) — from the slicer.
- Print-hour total (§2c, §6) — planning estimate, not sliced.
- Servo-case-screw count / STS3215 flange pattern (§1b, risk 5).
- All `servo_channel_map` counts/center/limits are first-guess — re-measure per
  physical servo at bring-up (§4.4).
