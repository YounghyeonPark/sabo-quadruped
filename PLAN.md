# RoboKitten — Project Plan

A from-scratch quadruped companion robot that **mimics a kitten** in shape,
movement, and communication, built to befriend a real cat at home.

Core feature set: **Moving · Playing · Home-keeping · Talking**

---

## 1. Vision & Goals

Build a small, soft, kitten-sized four-legged robot that a real cat reads as a
non-threatening playmate — not a machine. Success is measured less by
engineering polish and more by one thing: **does the resident cat choose to
interact with it?**

Design north stars:

1. **Believable as a kitten** — size, proportions, and motion trigger the cat's
   social instincts, not its fear/prey-panic response.
2. **Safe for the cat, safe from the cat** — no pinch points, no eye-level
   lasers, and a body that survives being pounced on.
3. **Expressive** — it communicates using signals cats *and* humans understand.
4. **Software portable** — behaviors/voice/vision developed independently of the
   legs, so the brain can be prototyped on a wheeled base first.

---

## 2. Design Principles (kitten mimicry)

### 2.1 The "baby schema" (why proportions matter)
Kittens — and human babies — share a set of proportions (large head, big
low-set eyes, short rounded limbs, small body) that trigger caregiving and
non-aggression responses. Adult cats are measurably more tolerant of kittens.
We deliberately exaggerate these:

- Oversized rounded head relative to body
- Large, forward "eyes" (the camera + LED eyes live here)
- Short legs, compact torso
- Soft, rounded silhouette — no sharp corners or exposed brackets

### 2.2 Non-threat behavior
- Move **slowly and low** on first approach; never charge.
- Let the cat approach first; retreat/freeze if the cat's ears go flat.
- Idle "resting" poses (sit, curl) so it isn't always in motion.

### 2.3 Safety (both directions)
- Soft exterior shell (silicone skin or short-pile fur over foam).
- No servo pinch points reachable by paw or tongue; recessed joints.
- Battery in a sealed, chew-proof compartment.
- Quiet actuators — servo whine reads as "wrong" to a cat (see Risks §10).

---

## 3. Physical Design — Shape

### 3.1 Target size & weight
| Property | Target | Rationale |
|---|---|---|
| Body length | 18–25 cm | Real 3–4 month kitten scale |
| Standing height | 12–18 cm | Low, non-looming |
| Weight | 0.8–1.5 kg | Light enough to be safe, heavy enough to stand |

### 3.2 Degrees of freedom (DOF) map
| Group | DOF | Notes |
|---|---|---|
| Legs | 12 (3 × 4) | hip-roll + hip-pitch + knee per leg → real walking & turning |
| Head | 2 | pan + tilt (camera "eyes" aim + expressive nods) |
| Ears | 2 | forward/back — key feline signal |
| Tail | 1–2 | up/down (+ optional side sweep) — the loudest emotion channel |
| **Total** | **17–18** | Legs are must-have; head/ears/tail are the "kitten personality" budget |

> Minimum viable: 12 leg DOF + 1 tail + 2 head. Ears can be a fast-follow.

### 3.3 Exterior / materials
- **Frame:** 3D-printed (PLA for prototype, PETG/nylon for durability).
- **Skin:** short-pile faux fur over a foam underlayer, or cast silicone.
  Removable/washable — it *will* get cat hair and slobber on it.
- **Eyes:** ring/matrix LEDs behind a diffuser for blink, pupil-dilate, and
  "sleepy" expressions.

---

## 4. Movement System

### 4.1 Locomotion
- Gait/IK stack borrowed from a proven open design (see §9) — we do **not**
  re-derive inverse kinematics from scratch.
- Core gaits: **stand, walk, turn-in-place, trot.**
- Balance via an **IMU** (accelerometer + gyro); reflexive "catch" when tilted.

### 4.2 Kitten motion vocabulary (this is what sells it)
Beyond locomotion, script kitten-specific micro-behaviors — these matter more to
the cat than smooth walking:

- **Play bow** — front down, rear up: universal "let's play" invitation.
- **Pounce** — crouch, wiggle rear, spring forward a short distance.
- **Side-arch / crab-hop** — the classic startled-kitten sideways bounce.
- **Stretch & sit** — idle/resting poses so it isn't robotically constant.
- **Startle-freeze** — sudden stop + low crouch when the cat lunges.

Kitten motion is **erratic, curious, and sudden** — build randomness and pauses
into the behavior loop rather than smooth continuous motion.

### 4.3 Sensing for movement
- IMU (balance/orientation)
- Front distance sensor (ToF or ultrasonic) — obstacle & edge/stair avoidance
- Camera (also feeds Playing + Home-keeping)

---

## 5. Communication System ("with friends")

Two audiences: **the cat** (feline signals) and **the owner/family** (app +
voice). Optional third: **robot-to-robot / robot-to-smart-toy**.

### 5.1 Cat-directed communication — speak *cat*
Cats communicate through posture, tail, ears, eyes, and sound. Mimic these:

| Channel | Signal | Meaning to a cat | Implementation |
|---|---|---|---|
| **Tail** | up / gentle sway | friendly, confident | tail servo |
| | puffed / thrash | alarm / back off | tail servo + fast motion |
| **Ears** | forward | interested, playful | ear servos |
| | flattened | fear / stop | ear servos |
| **Eyes** | **slow blink** | trust & affection (proven rapport-builder) | LED eyes fade |
| | wide / dilated | alert / play arousal | LED eyes |
| **Posture** | play bow, low crouch | invitation / non-threat | leg IK |
| **Sound** | **trill / chirp** | friendly greeting | speaker |
| | **meow** | attention-seeking | speaker |
| | **purr** | contentment / calming | **haptic vibration motor + low speaker** |
| | hiss | warning (use sparingly) | speaker |

> The **slow-blink** and **purr-via-vibration** are high-impact, low-cost wins —
> both are documented ways to build trust with real cats.

### 5.2 Owner-directed communication
- Live camera stream to phone (also = Home-keeping).
- **Talking:** local TTS (e.g. Piper) through the speaker; speak canned phrases
  or, later, two-way audio so you talk *through* the kitten remotely.
- Status app/dashboard: battery, mood/state, "cat detected" events.
- Push alerts (motion, cat interaction, low battery).

### 5.3 Robot-to-friends (optional / future)
- Multiple RoboKittens could sync over Wi-Fi/BLE for "play together" behavior.
- Or trigger existing smart toys/feeders as part of a play routine.

---

## 6. Feature → System Mapping

| Feature | Delivered by |
|---|---|
| **Moving** | §4 locomotion + gaits + obstacle/edge avoidance |
| **Playing** | §4.2 pounce/bow/dart behaviors + camera cat-tracking + tail/ear expression |
| **Home-keeping** | Camera stream + motion detection → push alerts (§5.2) |
| **Talking** | Speaker + TTS + two-way audio (§5.2); cat-sounds (§5.1) |

---

## 7. Electronics & Compute Architecture

```
                 ┌────────────────────────┐
                 │   Raspberry Pi (brain)  │  vision, voice, behavior, Wi-Fi
                 │   Pi 4 / Pi 5 / Pi Zero2│
                 └───────────┬─────────────┘
                             │ I²C / UART / GPIO
        ┌────────────────────┼───────────────────────┐
        │                    │                        │
 ┌──────┴──────┐     ┌───────┴────────┐        ┌──────┴───────┐
 │ PCA9685 x1-2│     │  IMU (MPU6050) │        │ Mic + Speaker│
 │ servo driver│     │  balance/orient│        │ audio I/O    │
 └──────┬──────┘     └────────────────┘        └──────────────┘
        │ 16 ch PWM each
   ┌────┴─────────────────────────────┐
   │ 12 leg servos + head/ears/tail    │
   └───────────────────────────────────┘

 Power: LiPo (2S/3S) → buck converters → { 5V logic (Pi), 6-7.4V servos }
        separate servo rail + big capacitor to survive current spikes
```

- **Brain:** Raspberry Pi (Pi 4/5 for on-board vision + TTS; Pi Zero 2 W if
  offloading heavy compute). Optional microcontroller co-processor (RP2040/ESP32)
  for real-time gait if the Pi is busy.
- **Servo driver:** one or two PCA9685 (16 channels each) over I²C.
- **Sensors:** IMU, front distance sensor, camera, mic.
- **Output:** speaker (sound), vibration motor (purr), LED eyes/ears.
- **Custom PCB (optional, your KiCad win):** a carrier/power-distribution board
  seating Pi + PCA9685 + buck + IMU + connectors — replaces the jumper-wire nest.

---

## 8. Software Architecture

Layered so the top layers are hardware-independent (prototype on wheels first):

```
┌───────────────────────────────────────────────┐
│  App / Dashboard  (Flask web UI + push alerts)  │  owner comms
├───────────────────────────────────────────────┤
│  Personality / Mood state machine               │  curious｜playful｜sleepy｜scared
├───────────────────────────────────────────────┤
│  Behaviors: wander · play · watch · greet · rest │  the "kitten" logic
├───────────────────────────────────────────────┤
│  Perception: cat-detect (camera) · motion · IMU  │
├───────────────────────────────────────────────┤
│  Expression API: gait() blink() tail() purr() say()│  ← hardware-independent
├───────────────────────────────────────────────┤
│  Hardware abstraction layer (HAL)                │  swap wheels↔legs here
└───────────────────────────────────────────────┘
```

- A **mood state machine** drives which behaviors/expressions fire, giving
  coherent "personality" instead of random twitching.
- **Cat detection** via camera (lightweight model or motion+blob to start).
- Behaviors run as concurrent loops; the mood layer arbitrates.

---

## 9. Reference Designs to Borrow From

Do **not** start from a blank sheet for gait/IK. Fork one of these:

| Design | Why | Reuse |
|---|---|---|
| ✅ **Mini Pupper 2** (MangDang) — *CHOSEN BASE* | Fully 3D-printable, kitten-scale (~20 cm), Pi + camera ready, quiet-ish coreless servos, proven IK/gait | printable frame, IK/gait, servo layout, electronics BOM |
| **SpotMicro / SpotMicroAI** | Fully open printable quadruped, but **puppy-scale (too big)** | IK reference only |
| **Petoi Nybble / OpenCat** | Genuinely cat-shaped & cat-scale, tough firmware, but frame is laser-cut wood + Petoi's own board (not print-from-scratch) | motion primitives, robustness ideas |

**Decision (locked):** base the **mechanics + electronics + IK/gait on Mini
Pupper 2** — it's the best-documented *fully 3D-printable* Pi quadruped at kitten
scale, and its coreless servos are quieter (helps the whine risk in §10).

Our contribution on top: a **custom kitten-proportioned printed shell** (baby-
schema head, big LED eyes) + **ears/tail** appendages, **cat-language
communication**, and the **personality/mood software** — printed on your own
printer over the proven Mini Pupper chassis.

---

## 10. Risks & Mitigations

| Risk | Impact | Mitigation |
|---|---|---|
| **Servo whine scares the cat** | cat avoids robot permanently | quiet/coreless servos; damping; introduce slowly |
| **Cat pounces, breaks a leg** | hardware damage | soft shell, tough filament, low center of mass, "startle-freeze" behavior |
| **Walking is hard / unstable** | project stalls | reuse proven IK+gait (§9); prototype software on wheels first |
| **Cat ignores it entirely** | mission fail | lean hard on baby-schema shape, slow-blink, trills, erratic prey-like motion |
| **Power/current spikes reset Pi** | crashes mid-walk | separate servo power rail, big capacitor, adequate buck converter |
| **Scope creep (18 DOF at once)** | never finishes | phase it — legs first, expression later (§11) |

---

## 11. Build Roadmap (phased)

**Phase 0 — Software on wheels (de-risk, ~1 weekend)**
- Build behavior/voice/vision stack on a cheap wheeled base or in simulation.
- Deliver: cat-detection, play/wander/watch behaviors, TTS, dashboard.
- *Everything above the HAL transfers directly to the quadruped.*

**Phase 1 — Legs that stand & walk (~2–4 weeks)**
- Print/assemble a reference quadruped frame (§9), 12 leg servos + PCA9685 + IMU.
- Get stand → walk → turn working via the borrowed IK/gait.

**Phase 2 — Kitten motion (~2 weeks)**
- Add pounce, play-bow, crab-hop, stretch, startle-freeze.
- Tune for erratic, curious, pausing motion.

**Phase 3 — Expression & communication (~2 weeks)**
- Add tail, ears, LED eyes (slow-blink), speaker sounds, purr vibration.
- Wire into the mood state machine.

**Phase 4 — Skin & shape (~1–2 weeks)**
- Fur/silicone exterior, baby-schema head, final safety pass.

**Phase 5 — Custom carrier PCB (optional, KiCad)**
- Consolidate wiring onto one board.

**Phase 6 — Live with the cat**
- Introduce slowly, observe, iterate behaviors based on the real cat's reactions.

---

## 12. Open Questions (to decide next)

1. **Compute:** all-on-Pi, or Pi + microcontroller co-processor for gait?
2. **Servo budget:** start at 12 DOF (legs only) or go straight to 15+ (with tail/head)?
5. **Purr haptics:** worth adding a vibration motor, or speaker-only?

**Resolved:**
- ✅ **3D printer:** available — plan assumes self-printing (PLA prototype → PETG/nylon for durable legs).
- ✅ **Scale:** kitten/cat scale (~18–20 cm body) — see §3.1.
- ✅ **Base design:** **Mini Pupper 2** chassis + custom kitten shell — see §9.

---

*Next step options: (A) turn Phase 0 into a concrete task list + starter code,
(B) pick a reference base design and draft its full BOM, or (C) detail the
electronics/wiring for Phase 1.*
