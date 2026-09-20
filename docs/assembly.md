# Sabo — Assembly Guide (print + bolt-up)

This is the build guide for the printable frame. It is generated *around* the
parametric CAD: dimensions live in `cad/params.py`, the per-part print plan in
`cad/print_manifest.py` (run `python -m cad.print_manifest`), and the mass /
fit numbers below come from `python -m cad.export` + `python -m analysis.validate`.

The **skin** (`cad/parts/shell.py`) goes on at the end, but it is not cosmetic and it
is not optional: it carries the openings the legs, the neck, the tail and the spine
joint pass through, and it is in the mass budget. The **limb fairings**
(`cad/parts/fairing.py`) are part of it — a static shell cannot close the flank,
because the knee servo lies laterally and sweeps it open as the hip works, so the
scapula and haunch covers bolt to the thighs and turn with them.

---

## 1. Bill of printed parts (frame)

19 distinct parts (`cad/assembly.py :: PRINTABLE`), quantities from
`cad/export.py :: COUNTS`:

| part | qty | material | perims | infill | supports | heat-set inserts (ea) |
|------|----:|----------|-------:|-------:|:--------:|----------------------:|
| torso_fore | 1 | PETG | 3 | 18% | yes | 4× M2 |
| torso_aft | 1 | PETG | 3 | 18% | yes | 0 |
| head | 1 | PLA | 3 | 10% | yes | 4× M2 |
| ear | 2 | PLA | 3 | 20% | no | 2× M2 |
| tail | 1 | PLA | 3 | 20% | no | 0 (clevis pin + welded rocker) |
| tail_crank / tail_pushrod | 1 ea | PETG | 4 | 60% | no | 4× M2 (crank horn) |
| hipbr_F_L / F_R / R_L / R_R | 1 ea | PETG | 4 | 25% | yes | 0 |
| upper_F / upper_R | 2 ea | PETG | 4 | 30% | yes | 4× M2 |
| lower_F / lower_R | 2 ea | PETG | 4 | 30% | no | 0 |
| foot_F / foot_R | 2 ea | PETG | 4 | 30% | yes | 0 (paw ideally TPU) |
| crank_F / crank_R | 2 ea | PETG | 4 | 60% | no | 4× M2 |
| pushrod_F / pushrod_R | 2 ea | PETG | 4 | 60% | no | 0 |

**Total heat-set inserts per robot: 72 (64 × M2 + 8 × M3)** — counted from the CAD by
`analysis/hardware_bom.py`, which `cad/print_manifest.py` now checks itself against. (Servo *case* screws thread into
the STS3215's own tapped flanges, and the four-bar *pins* are retained by e-clip /
shoulder-screw-head seats — neither uses an insert.)

Structural parts (legs, four-bar, brackets, torso) print in **PETG**; light
cosmetic parts (head, ears, tail) in **PLA**. The paw pads are best printed (or
over-moulded) in **TPU** for grip — split the `foot_*` pad off if your slicer
supports multi-material, otherwise PETG is fine for a first build.

---

## 2. Fits & fasteners (single source of truth: `cad/params.py`)

### Print fits
| class | value | where |
|-------|-------|-------|
| `FIT_CLEARANCE` | 0.20 mm | part-to-part locating / registration |
| `PIN_CLEARANCE` | 0.15 mm radial | rotating Ø3 pin-in-bore pivots (four-bar, knee, ankle) |
| `PRESS_INTERFERENCE` | 0.10 mm radial | a pin pressed/fixed into a link (tighter bore) |
| `CLEVIS_GAP` | 0.35 mm/face | running clearance of a tongue inside its fork slot |
| servo pocket | 0.40 mm/face | `cad/servo.py :: SERVO.pocket` (case), + `flange_cut`, + `horn_seat` |

### Joint architecture — every pivot is a clevis

Two links that turn on one pin cannot both be drawn on the leg's centre plane, so each
pivot is a **fork + tongue in double shear**: the pin passes cheek → tongue → cheek and is
retained by an E-clip at both ends, never cantilevered.

| joint | fork (2 cheeks) | tongue (1 blade) |
|---|---|---|
| hip | — (thigh bolts to the drive axle's flange on its inboard cheek) | — |
| knee | `upper_*` | `lower_*` |
| ankle | `lower_*` | `foot_*` |
| four-bar C | `pushrod_*` runs in its own lane beside the crank | `crank_*` |
| four-bar R | `pushrod_*` | rocker (moulded into `lower_*`) |
| tail | `tail` (mouth opens forward) | tongue on `torso_aft` |

The thigh is not a solid strut but a forward-opening **channel**, and the whole four-bar
lives inside it. Across the channel, inboard → outboard:

```
 knee servo body | crank / rocker / shank tongue | pushrod | outboard cheek
     y <= -3.85  |         -3.5 .. +3.5          | 3.85..9.85 |  10.2 .. 14.7   (mm, x leg side)
```

Lane widths come from `params.clevis_slot()`; change `CLEVIS_TONGUE_T` and every fork
slot, tongue and the thigh's own width re-fit together.

### Tail — a remote drive, for the same reason the hip has one

The tail pivot sits at the rear extremity of the frame, where the body has tapered to
almost nothing. `python -m analysis.actuator_fit` measures that **no STS3215 can be housed
there** — 45 % of its boss falls outside the ribcage, in every orientation. So the tail is
driven the way the hip already is: the actuator moves to where there *is* room and reaches
the joint through a linkage.

| | |
|---|---|
| Servo station | `params.TAIL_SERVO` = (−54, 0) in the aft-torso frame — the one station `actuator_fit.capacity('aft')` finds room at |
| Mechanism | crank-rocker four-bar, ground 38.5 mm (servo shaft → tail pivot) |
| Links | crank 21.5 / coupler 45 / rocker 18 mm (`params.TAIL_FOURBAR`) |
| Travel | **2.08 rad** against the ±1.0 rad the joint commands; transmission angle 42–140° |
| Parts | `tail_crank`, `tail_pushrod`, rocker moulded into `tail` |

Link lengths were chosen with the same `analysis/fourbar.py` the knee uses, and the torso
carries the pushrod's corridor, the crank's swept disc and the tail's swing clearance.

### Head size is an actuator constraint

The head is a hollow ball whose socket is carved out of the chest, so its diameter trades
against the room in front of the waist. `python -m analysis.actuator_fit` measures both:

| HEAD_R | head dia | housings in the head | housings in the chest |
|--:|--:|--:|--:|
| 46 | 92 | 1 | 1 |
| **50** | **100** | **2** | **1** |
| 52 | 104 | 2 | 0 |
| 62 | 124 | 3 | 0 |

`HEAD_R = 50` is the smallest radius that buys a second slot in the head and the largest
that still leaves one in the chest. The chest slot also needs the gimbal pushed forward to
`params.HEAD_MOUNT_X` — with the head centre inside the ribcage (where it used to be), the
socket takes the whole front of the chest and nothing can be housed there at all.

That gives **three** places an expression actuator can live forward of the waist, against
four joints that want one (head pan/pitch/tilt + ears). Mounting three gimbal servos on
their own axes needs about a Ø124 head — two thirds of the body's length.

### Head gimbal — pitch direct, yaw remote

`analysis.actuator_fit.gimbal_layout` measures that a gimbal's motors cannot BOTH sit on
their own axes inside this head: the axes meet at a point, so each has to step aside along
its own axis, and two STS3215 housings do not both fit that way until roughly a Ø148 head.
One does, centred — so:

| axis | drive | where the motor is |
|---|---|---|
| **pitch** (nod) | direct | inside the head, on the axis, horn bolted to the neck yoke |
| **yaw** (pan) | four-bar | in the chest at `PAN_SERVO`; ground 41 mm, crank 17 / coupler 47 / rocker 13.5, **2.03 rad** against ±1.0 commanded, μ 42–140° |

The topology is forced. The head is the last link in the chain, so a servo inside it can
only reach the one joint immediately above it; the other axis has to be driven from
further up, and the only cavity there is the chest.

Two placement consequences worth knowing before changing anything:

- The **yaw axis is 18 mm behind the head's centre** (`HEAD_GIMBAL_STACK`). It has to be:
  the head ball leaves 2 mm under it at its own centre and 15 mm at 18 mm back, and the
  pan linkage needs a horizontal plane to run in. The head itself does not move — only the
  axis — which gives it a slight sideways shift as it turns, the way a real neck does.
- The **head sits 8 mm above the shoulder line** (`HEAD_RISE`), which is what finally gave
  the pan rocker room to pass under the ball.
- The **crank is deliberately small** (17 mm). A longer one reaches the same range with a
  nicer transmission angle, but its swept disc runs back past the waist plane into the aft
  half.

### Leg → torso attachment

Each hip bracket **bolts** to a flat pad on the torso flank at |y| = `BODY_W/2` with
2× M3 screws into heat-set inserts (`params.MOUNT_*`). The ribcage is relieved outboard of
that pad so the bracket lands on a real face, and a swept scallop clears the knee servo's
housing as the hip turns.

### Fastener families
- **Heat-set inserts** (`params.HEATSET`): M2 (Ø3.2 melt hole, 4.0 mm deep) and
  M3 (Ø4.1, 5.7 mm). Install with a soldering iron + insert tip, flush to the boss.
- **Screws** (`params.SCREW`): M2 clearance Ø2.5 / head Ø4; M3 clearance Ø3.5.
- **Servo horn interface** (`params.HORN_*`): the STS3215 Ø20 metal horn drives a
  printed part through **4× M2** screws on a Ø15 bolt circle **into heat-set
  inserts**, plus the centre horn-shaft screw. Used on: `crank`, `upper` (hip),
  `tail`, `ear`, and the fore-half waist pad.
- **Servo case retention** (`params.SERVO_MOUNT_SCREW = M2`): the servo drops into
  its printed pocket and is held by **M2 clearance screws** through the boss into
  the STS3215's tapped flange holes. Verify the exact flange pattern against the
  datasheet before drilling metal — the CAD holes are the indicative pattern.
- **Four-bar / knee / ankle pins**: Ø3 dowels. Rotating bore = `PIN_R + PIN_CLEARANCE`;
  each pivot has an E-clip retention counterbore on **both** outer cheeks
  (`fasteners.pin_head_seat`).
- **Hip drive**: a Ø6 axle per leg running in two **686ZZ** bearings (6×13×5) — one in the
  torso core wall, one in the hip bracket — with a Ø20 flange bolted to the thigh's
  inboard cheek.

All fastener geometry is generated by `cad/parts/fasteners.py` — change a size in
`params.py` and every part re-fits.

**The orderable list** — every insert, screw, pin, E-clip, bearing, axle and dowel with
its size and quantity — is counted from this CAD by
[`docs/hardware_bom.md`](hardware_bom.md) (`python -m analysis.hardware_bom`). It also
flags what is *not* yet housed: the five expression servos (head pan/pitch/tilt, ears,
tail) are in the kinematics and the mass budget but have no pocket cut for them yet.

---

## 3. Print orientation & splitting

Run `python -m cad.print_manifest` for the live table. Key calls:

- **Legs (`upper/lower/foot`)** — lay the bone axis flat on the bed so layer lines
  run *along* the bending load path; orient the servo pocket / rocker mouth up so
  it doesn't need support.
- **Four-bar links (`crank/pushrod`)** — flat on the bed, link plane down: no
  support, and the layers take the transmission load in-plane. Print at 60% infill
  (small, high-stress).
- **Hip brackets** — servo-pocket mouth up; some support for the arm/hub overhang.
- **Ears / tail** — lay flat / along the bed; self-supporting.
- **Head** *(SPLIT)* — printed as two bowls, split at the equator (see below).
- **Torso halves** *(SPLIT)* — each printed as left/right sagittal halves (see below).

### Splitting & bonding the big parts (`cad/parts/split.py`)

`torso_fore`, `torso_aft` and `head` are too big / support-hungry to print whole, so
`cad/parts/split.py` cuts each at export time and injects the mating features. The
**whole** parts are untouched (the sim, kinematics, `full_robot()` and mass model still
see them intact); the split is purely a print-time op emitting extra STL/STEP files
(`torso_fore_L/R`, `torso_aft_L/R`, `head_A/B`) and its own manifest rows. Registration
uses **separate Ø4 mm dowel rods** (a printed-in-place boss would point into the bed on a
cut-face-down half) — each rod **presses** into one half (`fit="press"`, −0.10 mm) and
**slips** into the other (`fit="clear"`, +0.15 mm), so a rod locks to one side and the
halves still part for gluing. Each cut adds a little bond material to give a flat glue
land and host the dowel sockets, then a shallow **glue-relief channel** for squeeze-out.

| part | cut plane | why | dowels | bond face |
|------|-----------|-----|--------|-----------|
| `torso_fore` | **sagittal** `Y=0` → `_L`(+y)/`_R`(−y) | open lattice; each hoop arches off the flat XZ cut face with near-zero support | 2 spine + 1 keel Ø4 rods (triangle) | spine + keel stringer faces, waist bulkhead + 3 bond pads; glue channel per pad |
| `torso_aft` | **sagittal** `Y=0` → `_L`/`_R` | same | 2 spine + 1 keel Ø4 rods | same; waist servo pocket is split (servo captured on bonding) |
| `head` | **equatorial** `Z=8` (brow line) → `_A` face bowl / `_B` cap | keeps **both eyes, the camera bore, muzzle and neck stub intact** in the lower bowl, open at the top so you seat + wire the camera and LED eyes before bonding; upper cap is a plain support-free dome | 3 Ø4 rods on a Ø78 circle | shell-wall annulus + 3 wall pads; ring glue groove |

The cut plane is chosen **above** the eye tops (`z=+7`) so no eye/camera feature straddles
the head seam. Each half is OCCT-valid and closed (0 real open-boundary edges; the torso-aft
halves are non-manifold-but-closed where the keel pad meets the keel stringer — printable).

**Bond procedure:** dry-fit the two halves on the dowel rods (press side first); confirm the
flat cut faces meet with no rock. Apply thin CA (PLA head/ears) or 5-min epoxy (PETG torso)
to the mating faces — the relief channel/groove takes the squeeze-out — press home on the
rods, and clamp (tape or light spring clamps) until cured. Inserts that straddle a cut (the
4× M2 waist-horn inserts, split across `torso_fore_L/R`) are melted in **after** bonding, when
the hole is whole again; the head's 4× M2 neck-stub inserts sit entirely in `head_A`.

---

## 4. Wiring (STS3215 TTL daisy-chain, `hardware/servo_channel_map.py`)

12 servos share one 3-wire (GND / V+ / signal) half-duplex TTL bus, IDs 1–12.
Physical pass-throughs (holes only, no connectors) are provided:

- **Each leg-mount node** has a bus channel above the servo pocket so the leg
  servo's loom runs inboard toward the spine.
- **The waist** has a bus hole through both bulkheads so the trunk crosses the
  fore↔aft joint.
- **The head stub** has a sensor-cable pass-through down to the neck for the
  camera / mic wiring, on to the Jetson bay.
- Leg struts are open (fenestrae) + the hip/knee bosses have shaft reliefs — route
  spare loom through these.

Recommended chain order (shortest trunk): front legs (1–4) → rear legs (5–8) →
waist (9) → head pan/pitch (10–11) → tail (12). LED eyes are
**not** on the bus (Jetson PWM pin — see the channel map).

---

## 5. Assembly sequence

1. **Print & post-process.** Print per §1/§3 — including the split halves
   (`torso_fore_L/R`, `torso_aft_L/R`, `head_A/B`). **Bond the split halves first**
   (§3 "Splitting & bonding"): dowel-rod + glue the torso L/R pairs and the two head
   bowls into whole parts before anything else. Ream all Ø3 pivot bores and the
   servo-pocket walls; test-fit a Ø3 pin (should rotate freely — that's the
   0.15 mm clearance).
2. **Heat-set inserts (48× M2).** Melt inserts into: 2 crank horns, 2 upper-leg
   hip horns, tail base, both ear bases, the head neck stub, and the fore-half
   waist pad.
3. **Assign servo IDs.** Before mounting, set each STS3215's bus ID (1–14) on the
   bench per `servo_channel_map.py`, and centre each horn at 2048 counts.
4. **Frame → servos.**
   - Drop the 4 abduction/hip servos and the waist servo into their pockets;
     secure with M2 case screws. Bolt the fore + aft half at the waist: waist
     servo body in the aft bulkhead, its horn into the fore-half waist pad (M2).
   - Fit the two front-leg hip servos into the `hipbr_F_*` brackets, rear into
     `hipbr_R_*`.
5. **Legs + four-bar.** Per leg: bolt `upper` onto the hip servo horn (Ø15 M2
   circle). Mount the knee servo in the upper-leg crank boss (M2 case screws) and
   bolt the `crank` onto its horn. Pin the passive **knee** (`upper`↔`lower`,
   Ø3 + e-clip), link the `pushrod` crank→rocker (2× Ø3 pins + e-clips), then pin
   the **ankle** (`lower`↔`foot`, Ø3 + e-clip). Confirm the four-bar sweeps the
   crank window without binding.
6. **Head.** Assemble the pan→pitch→tilt gimbal; bolt the head stub to the tilt
   bracket (4× M2 into the stub inserts). Bolt the two ears + tail onto their
   servo horns.
7. **Electronics.** Seat the Jetson in the fore back-bay, battery in the aft belly
   bay, bus adapter + speaker in the aft. Route + daisy-chain the bus (§4), plug
   the head sensor loom down the neck.
8. **Skin.** Fit the `shell` halves + head shell over the frame last, then bolt the
   four limb fairings on: 2× M2 each into the inserts in the thigh and the shank
   (the shank's pair sits below the knee blade). Check the fairings swing clear of
   the body through a full hip sweep before the screws go in — they are cut to the
   hip's real working window, so a fairing that fouls means a limb is mis-seated.

---

## 6. Validation snapshot (`python -m analysis.validate`)

- MASS 1533 g (plastic 442 + components 1091; target 800–1600) — **PASS**
  (+15 g vs pre-print-features from the insert bosses + waist bulkheads).
- STANCE / BALANCE / TORQUE — **PASS** (unchanged; no joint/geometry edits).
- `python -m pytest -q` → 27 passed; `mj_emulate --gait walk` → upright.

---

## 7. Open items (deferred)

- STS3215 flange-hole pattern is indicative; confirm against the datasheet.
- TPU paw-pad as a separate over-moulded part.
- `torso_fore` reports an OCCT self-intersection in the *fore ribcage loft*
  (pre-existing, unrelated to these print features); it still exports as one solid
  and slices, but should be cleaned up in a body-geometry pass.
