# Sabo — a low-cost, quiet, compliant 3D-printed quadruped

![Sabo — the inner frame under a translucent skin](docs/img/hero.png)
<p align="center"><em>Standing in the pose its own kinematics settle into — MuJoCo, not a posed CAD render.</em></p>

<p align="center">
  <img src="docs/img/front.png" alt="Front view — the face" width="46%" />
  <img src="docs/img/walk.gif" alt="Walking gait, front-facing (MuJoCo)" width="46%" />
</p>
<p align="center"><em>Left: the face — Ø100 head on a 180 mm body, baby-schema by intent and by
actuator budget (it holds two of the twelve servos). Right: walk gait, CoM-tracked — torso
roll ≈ 3.1° p-p.</em></p>

Sabo is a kitten-scale (**~1.31 kg**) quadruped platform: **fully 3D-printed**, driven by
cheap **serial-bus servos**, and designed to be **quiet** and **backdrivable** enough to
share close space with a live animal. It is built **design-as-code** — one parameter file
(`cad/params.py`) drives the CAD, the physics model, and the bill of materials, so the
*simulated* robot and the *printed* robot cannot drift apart.

It occupies a design point that is currently unserved: cheap-but-noisy hobby quadrupeds
(Pupper/Petoi class) are not backdrivable; quiet, backdrivable quadrupeds (QDD,
Mini-Cheetah/Solo class) are expensive and heavier. Sabo aims for **cheap + quiet +
backdrivable + sub-1 kg** at once.

> **Status — research platform, not yet built.** The mechanical design, physics
> validation, edge-AI hardware spec, and build plan are complete and *reproducible in
> simulation / CAD*. **No physical robot has been built yet.** All hardware-measured
> values (acoustic noise, backlash, backdrive torque, battery runtime, sim-to-real gap)
> are marked **TBD (hardware)** in the reports — they are the gating work, not claims.
> Component prices and servo calibration are first-guess estimates to be re-measured.

## Highlights

- **Cat-anatomical morphology** — digitigrade 4-DOF legs (front/rear differ), a sagittal
  spine (waist) joint, and a 2-axis head gimbal (pan + pitch; roll is handled by EIS), in
  a parametric build123d model.
- **Actuators placed where they fit, not where they are convenient** — four joints have no
  room for a servo on the joint itself, so they are driven through four-bars from wherever
  there is: the hips from the torso core, the knees from up the thigh, the tail and the
  head's yaw from the body. `analysis/actuator_fit.py` is what decides.
- **A limb architecture for cheap compliance** — a **proximal four-bar knee** (cable-free,
  light shank), a **remote-axle hip** (servos in the torso → −93 % hip lateral inertia),
  and **coupled underactuation** (2 motors/leg).
- **Buildable, not just drawable** — every pivot is a **clevis in double shear** and the
  thigh is a channel the whole four-bar runs inside, so no two printed parts share solid.
  A regression test ([`tests/test_geometry.py`](tests/test_geometry.py)) rebuilds the posed
  robot and fails on any interference, missing servo relief, or unfastened part.
- **Actuators that actually fit** — `analysis/actuator_fit.py` measures whether there is
  *room* for each servo, not just torque. It is why the tail is driven remotely (like the
  hip), why the head is Ø100 and sits at the torso's nose, and why the last expression
  joint is flagged rather than quietly drawn as if it fitted.
- **Design-as-code** — `python -m analysis.platform_report` regenerates the whole artifact
  set (CAD → MJCF → BOM → spec) from one parameter file, with no hand-authored URDF.
- **MuJoCo-validated motion** — stand / walk / trot gaits with IMU body-leveling, plus a
  cat-motion library (loaf, sit, pounce) — all stay upright with torque headroom.
- **Edge-AI + HAL brain** — a hardware-independent behavior stack (perception → mood →
  behavior → expression) behind a HAL, with a Jetson Orin Nano backend (stub-runnable on a
  dev box) and a serial-servo wiring pin-map.

## Key numbers

All derived from the model — regenerate with `python -m analysis.platform_report`.

| Platform | |
|---|---|
| Mass | **1314 g** (plastic 343 + components 971) — target 0.8–1.6 kg |
| BOM cost | $699 / **$834** / $968 (lo / mid / hi) |
| DOF | **12 actuated** — 2 motors/leg (hip+knee) + coupled ankle + rigid abduction; 4 expressive (waist, head pan/pitch, tail). The head holds two servo housings, so the ears are rigid and camera **roll** is corrected electronically instead of by a third gimbal axis |
| Actuator | Feetech STS3215 ×12 — 2.94 N·m stall, 60 g, TTL serial, **backdrivable** |
| Compute | Jetson Orin Nano Super (8 GB), 67 TOPS, 7–25 W |
| Four-bar knee | 128° ROM, 41–140° transmission angle (singularity-free) |
| Remote-axle hip | **−93 %** hip lateral inertia (motors relocated to the torso) |
| Envelope | 352 × 201 × 197 mm |
| Viable scale range | **k ≈ 0.7–1.5** (body 126–270 mm) — the fixed actuator sets the window |

Gait benchmark (MuJoCo):

| Gait | Upright | Travel | Peak torque (% of stall) | Torso roll p-p |
|---|:--:|--:|--:|--:|
| stand | ✓ | — | 12 % | 0.0° |
| walk | ✓ | 8 cm | 40 % | 3.1° |
| trot | ✓ | 39 cm | 34 % | 3.4° |

> Hardware-measured metrics (acoustic dB, backlash, backdrive torque, battery runtime,
> sim-to-real gap) are **TBD** — pending the physical build (see
> [`docs/build_mvp.md`](docs/build_mvp.md)).

## Gallery

<p align="center">
  <img src="docs/img/side.png" alt="Side view — digitigrade stance" width="31%" />
  <img src="docs/img/loaf.png" alt="Loaf pose" width="31%" />
  <img src="docs/img/sit.png" alt="Sit pose" width="31%" />
</p>
<p align="center"><em>Left → right: digitigrade stance (side), loaf, sit — current model, MuJoCo.
The ribcage, the channel thighs and the four-bar inside them read through the translucent skin.</em></p>

### Mechanism & analysis

<p align="center">
  <img src="docs/img/fourbar.png" alt="Four-bar knee kinematics" width="90%" />
</p>
<p align="center"><em>Proximal four-bar knee — 129° ROM, monotonic and invertible, with the
transmission angle held inside 40–140° (no singularity/lock-up). The same tool sizes the
tail and head-yaw drives, which are four-bars for the same reason: no room for a servo on
the joint itself.</em></p>

<p align="center">
  <img src="docs/img/scaling.png" alt="Design-as-code scaling study" width="74%" />
</p>
<p align="center"><em>Scaling study — the whole robot regenerated + re-validated from one
<code>SCALE</code> knob, and this figure is drawn by that same run. The <b>fixed</b> actuator pins the
viable window to <b>k ≈ 0.7–1.5</b>: scale up until mass and static torque run out, down until the
45 mm servo body no longer fits the thigh. Note the walk peak (green) barely moves — the limits are
the static budget and packaging, not gait stability.</em></p>

## Repository layout

| Path | What |
|---|---|
| `cad/` | parametric CAD (build123d): `params.py` (source of truth), `servo.py`, `parts/`, `assembly.py`, `export.py`, `print_manifest.py`, `parts/split.py` |
| `sim/` | MuJoCo physics: `mjcf.py`, `gait.py`, `mj_emulate.py`, `cute_motion.py`, `meshes.py`, `fourbar_leg.py`, `brain_bridge.py` |
| `analysis/` | engineering checks + reporting: `validate.py`, `optimize.py`, `bom.py`, `hardware_bom.py`, `fourbar.py`, `platform_spec.py`, `benchmark.py`, `platform_report.py`, `scaling_study.py` |
| `brain/` | hardware-independent behavior: HAL (`hal.py`), `perception.py`, mood/behaviors, expression, voice |
| `hardware/` | Jetson HAL backend (`jetson_backend.py`), serial servo bus map, `run_on_hardware.py` |
| `vision/` | camera geometry, pluggable detector, perception pipeline |
| `training/` | Isaac Lab RL locomotion scaffold, URDF export, policy deploy |
| `dashboard/` | Flask + SSE owner dashboard (Phase-0 behavior demo) |
| `docs/` | design docs — see the index below |
| `PLAN.md` | the original project plan |

Generated artifacts (`cad/out/`, `sim/out/`, `sim/meshes/`, `docs/out/`) are **git-ignored**;
they are regenerated by the toolchain (below).

## Quickstart

Requires **Python 3.14** (build123d + mujoco). `pybullet` is not used (it does not build on
3.14).

```bash
python -m venv .venv && . .venv/Scripts/activate   # Windows; use bin/activate on *nix
pip install -r requirements.txt

# engineering validation (mass / center-of-mass / joint torque vs servo stall)
python -m analysis.validate

# physics: gaits + cat-motion (headless renders written to sim/out/)
python -m sim.mj_emulate --gait walk
python -m sim.mj_emulate --gesture loaf      # also: sit, --pounce, --cute
python -m sim.mj_emulate --gait trot --view  # interactive 3-D viewer (if a display is present)

# CAD: printable STL/STEP + a mass manifest + renders (cad/out/)
python -m cad.export

# tests — includes the validation gate as a regression test (builds the CAD, ~25 s)
python -m pytest -q
python -m pytest -q -m "not slow"   # skip the CAD build
```

Windows consoles: prefix with `PYTHONIOENCODING=utf-8` if you hit a cp1252 encode error.

## Reproduce the platform characterization (design-as-code)

```bash
python -m analysis.platform_report     # cad.export → validate → meshes → spec → benchmark
python -m analysis.platform_spec       # spec sheet, derived live from the source of truth
python -m analysis.benchmark           # sim metrics + baseline scaffold + hardware-TBD slots
python -m analysis.scaling_study       # regenerate + re-validate the robot at k = 0.5–1.75
```

The **scaling study** shows the toolchain rescaling the whole robot from one `SCALE` knob
(env `SABO_SCALE`) with no drift, and finds the **viable build range k ≈ 0.7–1.5** — the
scale window is pinned by the *fixed* actuator/electronics, not the printed geometry. Its
cache is keyed on a fingerprint of the design, so a changed model invalidates it instead of
quietly re-serving the previous run's numbers.

## Documentation

- [`docs/platform.md`](docs/platform.md) — the platform contribution + evaluation protocol.
- [`docs/paper_outline_iros.md`](docs/paper_outline_iros.md) — technical paper outline.
- [`docs/build_mvp.md`](docs/build_mvp.md) — physical-build minimum spec (order → build → measure).
- [`docs/wiring_pinmap.md`](docs/wiring_pinmap.md) — full wiring / pin-map.
- [`docs/edge_ai_hardware.md`](docs/edge_ai_hardware.md) — compute / sensors / power.
- [`docs/assembly.md`](docs/assembly.md) — print + bolt-up guide (clevis joints, fits).
- [`docs/hardware_bom.md`](docs/hardware_bom.md) — orderable joint & fastener list, counted from the CAD.
- `python -m analysis.actuator_fit` — per-joint actuator fit + where a servo can be housed.
- Design notes: [`docs/noise_reduction.md`](docs/noise_reduction.md),
  [`docs/tendon_actuation.md`](docs/tendon_actuation.md),
  [`docs/custom_actuator.md`](docs/custom_actuator.md),
  [`docs/camera_stabilization.md`](docs/camera_stabilization.md),
  [`docs/BOM.md`](docs/BOM.md).

## Status & roadmap

- **Done:** parametric CAD, MuJoCo validation, four-bar knee + remote-axle hip, cat-motion
  library, edge-AI hardware spec, wiring map, print-readiness + part-splitting, design-as-code
  characterization + scaling study.
- **Next (gating):** the **physical build** — order parts (`docs/build_mvp.md`), print,
  assemble, and take the five hardware measurements that carry the "quiet" and "compliant"
  claims.

## Motivation

Sabo began as a companion robot to befriend the maker's cat — hence kitten scale, quiet
operation, and compliant, animal-safe motion. That use case drives the requirements; the
repository itself is a general low-cost quadruped platform.

## License & attribution

Licensed under the **Apache License 2.0** — see [`LICENSE`](LICENSE).

Third-party assets retain their own licenses (see [`NOTICE`](NOTICE)), including the OpenCV
frontal-cat-face Haar cascade in `vision/models/`. Built with
[build123d](https://github.com/gumyr/build123d), [MuJoCo](https://mujoco.org/), and
[trimesh](https://github.com/mikedh/trimesh).

> This is experimental, unbuilt hardware provided "as is" with no warranty. Building and
> operating it (LiPo batteries, actuators, mains-powered tools) is at your own risk.
