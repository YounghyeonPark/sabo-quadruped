# Sabo — a low-cost, quiet, compliant 3D-printed quadruped platform

**Paper backbone (Idea B).** This document frames Sabo as a *research platform*
contribution and defines the evaluation protocol. The spec/benchmark tables below
are **auto-generated from the single source of truth** — regenerate with
`python -m analysis.platform_report` (writes `docs/out/platform_spec.md` +
`docs/out/benchmark.md`). Numbers here are transcribed from that output; if they
drift, the generator is authoritative.

## Thesis

A **sub-1 kg, fully 3D-printed, serial-servo** quadruped that is **quiet** and
**backdrivable/compliant** — occupying the gap between cheap-but-noisy hobby-PWM
quadrupeds (Pupper / Petoi class) and quiet-compliant-but-expensive QDD quadrupeds
(Mini-Cheetah / Solo class). It is aimed at *close-contact companion use around a
live animal*, where quietness and compliance are functional requirements, not
niceties.

## Contributions

1. **A cost/quietness/compliance point that is currently unserved at kitten scale.**
   ~$838 mid BOM, 1451 g, backdrivable serial servos with torque-off silent hold.
2. **A limb architecture that keeps the leg light + quiet + the motors central:**
   a **proximal four-bar knee** (crank → pushrod → rocker, no cable friction) and a
   **remote-axle hip** (hip servos in the torso core, driven out to the pivot by a
   shaft) — cutting per-hip lateral inertia **93%** and giving a slim, cat-like
   silhouette while preserving a wide, stable foot base.
3. **Design-as-code reproducibility:** one parameter file drives CAD, the physics
   model, the BOM, *and* the spec sheet, so the simulated model and the printed
   model provably cannot drift (no hand-authored URDF). `analysis/platform_report.py`
   regenerates the entire artifact set in one command.

## Platform at a glance

*(key rows — full sheet: `docs/out/platform_spec.md`)*

| | |
|---|---|
| **Mass** | 1451 g (plastic 360 g + components 1091 g) — target 0.8–1.6 kg |
| **Cost (BOM)** | $711 / **$838** / $965 (lo / mid / hi) |
| **DOF** | 14 actuated; per leg = 2 motorized (hip, knee) + coupled ankle/hock + rigid abduction; 6 expressive (waist, head pan/pitch/tilt, ears, tail) |
| **Actuator** | Feetech STS3215 ×14 — 2.94 N·m stall, 60 g, TTL serial daisy-chain, **backdrivable** |
| **Compute** | Jetson Orin Nano Super (8 GB), 67 TOPS, 7–25 W (15 W default) |
| **Sensors** | stereo CSI eyes, BNO085 IMU, 2× VL53L1X ToF, 2× I²S mic, BME688 e-nose, I²S speaker |
| **Envelope** | 347 × 189 × 192 mm (posed) |
| **Legs** | front 67.5 / 52.0 / 26.0 mm, rear 64.0 / 80.5 / 31.5 mm (digitigrade, cat-anatomical) |
| **Fabrication** | fully 3D-printed frame (PLA/PETG), 48× M2 heat-set inserts; head + torso split for print |

## Mechanism (design contribution)

- **Four-bar knee** (`analysis/fourbar.py`, `P.FOURBAR`): knee ROM **128° (2.24 rad)**,
  transmission angle **41–140°** (singularity-free), monotonic/invertible. Per-leg
  reachable knee: front 23.7–152.3°, rear 35.4–164.0° (cat-correct fore < hind fold).
  Verified as a closed loop in MuJoCo (held to 0.16 mm) and as CAD parts on the leg.
- **Remote-axle hip**: hip servos relocated to the torso core, driven to the pivot by
  a Ø6 axle in bearings → per-hip lateral inertia **16.46 → 1.16 ×10⁻⁴ kg·m² (−93%)**;
  shoulder skin de-flared 81 → 62 mm half-width.
- **Coupled underactuation**: ankle/hock mechanically coupled to the knee (cat
  reciprocal apparatus) + rigid abduction → 2 motors/leg instead of 3–4, at the cost
  of turning-by-gait. A DOF/cost reduction to be benchmarked against a fully-actuated
  baseline.
- **Dual-use DOF** (design-efficiency metric): 4 joints serve two jobs — the head
  pan/pitch/tilt gimbal does camera stabilization **and** expression; the waist does
  gait spine-flex **and** the arch/loaf posture.

## Benchmark protocol

### Sim-measurable (done — `docs/out/benchmark.md`)

| Gait | Upright | Travel | Peak τ (% stall) | Headroom | Roll p-p | Cam shake |
|---|:--:|--:|--:|--:|--:|--:|
| stand | PASS | 0 cm | 12% | 88% | 0.0° | 0.0 / 0.1° |
| walk | PASS | 11 cm | 44% | 56% | 2.5° | 1.8 / 0.5° |
| trot | PASS | 48 cm | 35% | 65% | 3.1° | 3.8 / 1.3° |

Plus: four-bar ROM/transmission-angle/monotonicity, remote-hip inertia reduction,
DOF-sharing count — all from the model.

### Hardware-measured (required for the paper — `TBD (hardware)`)

These carry the "quiet" and "compliant" claims and **cannot be faked in sim**:

| Axis | How to measure | Why it matters |
|---|---|---|
| **Acoustic noise (dB)** | calibrated mic @ 0.5 m, walk vs static hold | the core "quiet" claim |
| **Backlash (°)** | dial indicator at each joint, load reversal | transmission quality (four-bar vs direct) |
| **Backdrive torque (N·m)** | force gauge to back-drive each joint, torque-off | the "compliant / animal-safe" claim |
| **Battery runtime** | walk-until-cutoff on the 3S pack | usability |
| **Sim-to-real gap** | compare measured gait roll/travel vs sim | validates the design-as-code model |

### Baselines (positioning — cite each project's own numbers)

| | cost | scale | actuator | quiet? | backdrivable? |
|---|---|---|---|:--:|:--:|
| **Sabo** | **$838** | **~1.5 kg** | **serial servo** | **claim (TBD dB)** | **yes** |
| Stanford / Mini Pupper | [cite] | ~1–2 kg | hobby servo | no | no |
| Petoi Bittle/Nybble | [cite] | <1 kg | PWM servo | no | no |
| MIT Mini-Cheetah | [cite] | ~9 kg | QDD | ~ | yes |
| ODRI Solo | [cite] | ~2.5 kg | QDD | ~ | yes |

The defensible corner: **cheap AND quiet AND backdrivable AND kitten-scale** — no
current baseline hits all four.

## Reproducibility (design-as-code proof)

```
python -m analysis.platform_report      # cad.export → validate → meshes → spec → benchmark
python -m analysis.platform_report --skip-cad   # reuse cached geometry (fast)
```

One command regenerates every artifact from `cad/params.py` + `cad/servo.py`; the
spec sheet is derived live, so it cannot disagree with the printed/simulated robot.

**Scaling study (C3 evidence):** `analysis/scaling_study.py` regenerates + re-validates
the whole robot at k = 0.5–1.75 from a single `SCALE` knob, in a fresh subprocess per
scale (`docs/out/scaling_study.md`). It confirms the pipeline rescales with no drift, and
maps the **viable build range k ≈ 0.7–1.25** (body 126–225 mm): below it the fixed-size
servo/electronics break the design three ways (walk→100 % stall & fall, servo won't fit
the thigh, CAD blows through); above it mass exits the 1.6 kg band (k=1.5) then static
torque crosses the 2× line (k=1.75). The scale window is pinned by the *fixed actuator*,
not the geometry — a finding only a design-as-code sweep can produce cheaply.

## Honest gap — what stands between this and an accepted platform paper

- **Build one.** Everything above is CAD + physics + design; the platform paper needs
  a **working physical robot** to fill the five hardware axes (esp. noise dB and
  backdrive torque — the claims that define the contribution).
- **Fill the baseline cells** from the competitors' own publications.
- **Show reusability**: the platform must be demonstrably rescalable/reproducible by
  someone else (release CAD/sim/firmware; the design-as-code pipeline is the argument,
  but it needs an external build to be credible).
- Optional strengthener: a small **sim-to-real** study closing the loop on the walk
  gait, which doubles as validation of the design-as-code claim.

**Status:** the platform skeleton, characterization pipeline, and evaluation protocol
are complete and reproducible. The remaining work is the **physical build + the five
hardware measurements**, after which this is a submittable low-cost-quadruped platform
paper (IROS/ICRA platform track or an open-hardware venue).
