# IROS paper outline — low-cost, quiet, compliant 3D-printed quadruped platform

**Framing:** a *technical systems/mechanism* paper (IROS/ICRA platform track), not an
HRI paper. The companion-for-an-animal use case appears only as a one-line motivation
(a robot in close, quiet contact with a live animal makes quietness + compliance
*functional requirements*, not comfort features). The technical core is the design
point, the limb architecture, the toolchain, and the empirical characterization.

## Candidate titles
- *"A sub-1 kg, quiet, backdrivable 3D-printed quadruped: closing the cost–compliance gap with a proximal-linkage limb"*
- *"Design-as-code for a low-cost compliant quadruped: consistent CAD, simulation, and bill-of-materials from one parameter file"*
- *"Cheap and compliant: a serial-servo quadruped platform with a proximal four-bar knee and remote-axle hip"*

## Thesis / gap
Low-cost quadrupeds (hobby-PWM servo, Pupper/Petoi class) are **noisy and stiff
(non-backdrivable)**; quiet, backdrivable quadrupeds (QDD, Mini-Cheetah/Solo class)
are **expensive and heavier**. No open platform occupies **cheap AND quiet AND
backdrivable AND sub-1 kg**. We present one, enabled by a limb architecture that keeps
the leg light and cable-free while centralizing the motors, and a design-as-code
toolchain that makes the platform reproducibly rescalable.

## Contributions (state exactly these; keep them honest)
- **C1 — An unoccupied design point, realized and characterized.** An open, ~$838,
  1.45 kg, fully 3D-printed quadruped that is simultaneously quiet and backdrivable;
  quantified against cheap-noisy and expensive-compliant baselines on cost, acoustic
  noise, backlash, backdrive torque, and locomotion.
- **C2 — A limb architecture for cheap compliance.** Proximal **four-bar knee**
  (cable-free, light shank), **remote-axle hip** (motors in the torso → −93% hip
  lateral inertia), and **coupled underactuation** (2 motors/leg), analyzed and
  measured in sim + hardware.
- **C3 — A design-as-code toolchain.** One parameter file provably drives CAD, the
  MuJoCo model, and the BOM (no hand-authored URDF drift), enabling reproducible
  rescaling; released open-source.

> **Honesty note (for us, not the paper):** none of the primitives (four-bar,
> remote drive, coupled joints, MJCF) are individually novel. The contribution is the
> *system* hitting the unoccupied design point + the reproducible toolchain + the
> quantitative characterization. That is a legitimate IROS platform-paper shape (cf.
> Pupper, Solo, Minitaur papers) — but the benchmark numbers must *show* the gap, or a
> reviewer will read it as "just engineering." The noise-dB and backdrive-torque
> results vs baselines are the crux of acceptance.

## Section structure (IEEE, ~6–8 pp)

**I. Introduction** — the cost/compliance/noise design space; the gap; the three
contributions as a bulleted list; a teaser figure.

**II. Related Work** — four buckets, position ourselves in each:
1. *Low-cost / open quadrupeds* — Stanford Pupper & Mini Pupper, Petoi Bittle/Nybble `[cite]`. (cheap, PWM/serial, stiff, noisy)
2. *Proprioceptive / QDD legged robots* — MIT Cheetah & Mini-Cheetah, ODRI Solo, Minitaur `[cite]`. (backdrivable, quiet-ish, expensive/heavy)
3. *Transmission mechanisms in legs* — cable/tendon drives vs rigid linkages; four-bar / linkage knees; series-elastic `[cite]`. (why we chose a rigid four-bar)
4. *CAD-to-sim / robot-description automation* — Onshape-to-robot, URDF/MJCF generation, parametric design `[cite]`. (design-as-code positioning)
> Action: pull exact references for each `[cite]` — do NOT ship placeholder cites.

**III. Platform Overview** — spec table (Table I, auto-generated); anatomy-driven
morphology in one paragraph (digitigrade, sagittal legs, waist, head gimbal);
actuator + compute + sensor choices with the rationale (STS3215: serial, feedback,
torque-controlled → backdrivable, $ vs QDD).

**IV. Limb Architecture (C2)** —
- *Four-bar knee:* kinematics (crank→coupler→rocker), the transmission-angle /
  singularity / monotonicity analysis, ROM (128°), per-leg reach (front 152° / rear
  164°, cat-correct asymmetry). Fig: linkage + transmission-angle plot.
- *Remote-axle hip:* motors→torso, axle drive; the −93% lateral-inertia result and why
  it matters (roll/yaw inertia, silhouette). Fig: hip cross-section.
- *Coupled underactuation:* knee→hock coupling (reciprocal apparatus), rigid abduction;
  2 motors/leg; the DOF/cost trade (turn-by-gait) and a DOF-sharing metric (4 dual-use
  joints).

**V. Design-as-Code Toolchain (C3)** — params → {CAD (build123d), MJCF (real
inertials), BOM} pipeline; the consistency guarantee (spec sheet derived live);
`analysis/platform_report.py` one-command regeneration. Fig: pipeline diagram.

**VI. Locomotion & Control** (brief) — gait generation (leg IK + foot trajectories),
IMU body-leveling, dual-use head gimbal for camera stabilization; enough to run the
benchmarks, not the paper's focus.

**VII. Experiments** — sim validation + the hardware characterization (below).

**VIII. Results & Discussion** — the baseline-comparison table (Table II) as the
headline; per-axis discussion; where the design point wins / costs.

**IX. Conclusion, Limitations, Future Work** — limitations stated up front (n=1 build,
turn-by-gait, serial-servo bandwidth); future = sim-to-real learned gaits (Paper 2),
scaling study via the toolchain.

**Open-source statement** — CAD/sim/firmware/BOM released; the toolchain is the
reproducibility argument.

## Figures & tables plan
- **Fig 1** hero — render + (built) photo.
- **Fig 2** four-bar knee kinematics + transmission-angle plot (from `analysis/fourbar.py`).
- **Fig 3** remote-axle hip cross-section + inertia-reduction bar.
- **Fig 4** design-as-code pipeline (params → CAD/MJCF/BOM).
- **Fig 5** gait strip / motion snapshots.
- **Fig 6** acoustic noise: Sabo vs baselines (walk / hold / ambient) — *headline result*.
- **Fig 7** backdrive torque + backlash bars vs baselines.
- **Table I** platform spec (auto — `docs/out/platform_spec.md`).
- **Table II** baseline comparison: cost / mass / actuator / quiet? / backdrivable? / DOF — *headline table*.
- **Table III** per-gait metrics: sim vs hardware (roll, travel, torque headroom).

## Experiment plan (maps to `docs/build_mvp.md` §5)
| # | Claim | Metric | Method | Baseline | Sim value |
|---|-------|--------|--------|----------|-----------|
| E1 | quiet | A-weighted dB @0.5 m | walk / static-hold / ambient | Pupper-class, QDD-class `[cite/measure]` | — (HW-only) |
| E2 | compliant/safe | backdrive torque (N·m) | lever+gauge, torque-OFF, per joint | QDD-class | τ ≪ 2.94 stall |
| E3 | transmission quality | backlash (°) | dial indicator, reverse load; hip-axle vs knee-four-bar | direct-drive servo | four-bar loop 0.16 mm |
| E4 | capable locomotion | upright, travel, torque headroom | walk/trot on HW | — | walk 44% / trot 35% stall |
| E5 | model fidelity (C3) | sim-to-real gap | torso roll + travel, HW vs sim | — | walk roll 2.5° / 11 cm |
| E6 | cost | $ BOM | itemized | Pupper (cheaper), QDD (dearer) | $838 |
| E7 | limb inertia (C2) | hip lateral inertia | CAD/measured | direct-hip layout | −93% |

## Reviewer-risk register (anticipate + defend)
- *"Novelty = integration only."* → Defend with the **quantified unoccupied design
  point** (Table II) — show no baseline is cheap+quiet+backdrivable+sub-1kg; frame as
  a platform/systems contribution, cite precedent (Pupper/Solo papers).
- *"Sim-only."* → **Hard requirement: build it.** E1–E5 must be real hardware. Without
  the noise-dB + backdrive numbers there is no paper.
- *"Backdrivability of a geared serial servo is weak vs true QDD."* → Be precise:
  claim is *torque-off compliant hold + current-limited compliance*, quantified in E2;
  do not overclaim proprioceptive force control.
- *"Design-as-code is a tooling detail."* → Tie C3 to a concrete result: reproducible
  rescaling (a scaling study) + zero sim/CAD drift, and release everything.

## What stands between here and submission
1. **Physical build** (`docs/build_mvp.md`) — the gating item; E1–E5 need hardware.
2. **Baseline numbers** — measure or cite Pupper-class & QDD-class on E1–E3, E6.
3. **Exact citations** for all four related-work buckets (no placeholders).
4. **Scaling study — DONE** (`analysis/scaling_study.py` → `docs/out/scaling_study.md`):
   the toolchain regenerated + re-validated k = 0.5–1.75 from the single `SCALE` knob (no
   drift) — strong C3 evidence. Result: **viable build range k ≈ 0.7–1.25** (body 126–225 mm),
   bounded *below* by a triple fixed-part limit (walk hits 100 % stall & falls at k=0.6, the
   45.8 mm servo stops fitting the thigh at k≤0.6, CAD blows through at k=0.5) and *above* by
   the mass band (1657 g > 1.6 kg at k=1.5) then the static-torque 2× line (53 % at k=1.75).
   Headline finding: the fixed actuator + electronics — not the printed geometry — pin the
   platform's scale window; widening it is a one-line servo swap the same pipeline re-validates.

**Status:** paper skeleton, spec/benchmark pipeline (`docs/platform.md`,
`analysis/{platform_spec,benchmark,platform_report}.py`), and this outline are done.
The remaining work is empirical (build + measure + baselines + citations).
