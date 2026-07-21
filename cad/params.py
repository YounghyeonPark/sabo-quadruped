"""
Sabo mechanical parameters — the single source of truth (cat anatomy).
======================================================================

Sabo is now modelled on real feline limb anatomy rather than a generic symmetric
quadruped. Each leg has **4 DOF** (abduction + hip/shoulder + knee/elbow +
ankle/hock) and is **digitigrade** — it stands on its toes, so the limb shows the
cat "double-bend" Z-shape. Front and rear legs differ (cats' hind legs are longer
and more angulated than the forelegs).

Values are **millimetres** (build123d convention); use ``m()`` for metres (MuJoCo).
Baby-schema proportions (PLAN §2.1) are unchanged: oversized head, big low eyes.
"""

from __future__ import annotations

import math
import os

from cad.servo import DEFAULT as SERVO

MM_PER_M = 1000.0


def m(mm: float) -> float:
    return mm / MM_PER_M


# --------------------------------------------------------------- scale knob (design-as-code)
# ONE knob that regenerates a larger/smaller Sabo variant from this single source of
# truth. A fresh import with ``SABO_SCALE=k`` set gives a consistently-scaled model
# that CAD, the MuJoCo physics, the BOM and validation all follow (see
# ``analysis/scaling_study.py``). It multiplies the geometric **LENGTH** constants
# below (all mm) and NOTHING else — never angles/limits (rad), the knee coupling
# coefficients, the SERVO (a fixed real part), bought-component masses,
# EFFECTIVE_DENSITY, the servo/joint counts, the fixed BODY_*_REF station references,
# the MASS_TARGET/BODY_LEN_TARGET bands (kept fixed so the study can show a variant
# leaving the validated band), or the print-fit / fastener / insert / bearing features
# (like the servo, those are fixed COTS parts: M2/M3 screws, brass inserts, Ø6 axle).
# DEFAULT-IDENTITY: with SABO_SCALE unset (=1.0) every constant is byte-identical to
# the unscaled design (x*1.0 == x), so validation output and the tests are unchanged.
SCALE = float(os.environ.get("SABO_SCALE", "1.0"))


def _scale_lengths(d: dict, keys: tuple[str, ...]) -> dict:
    """Return ``d`` with the given LENGTH keys multiplied by ``SCALE`` (mm), leaving
    every other key (angles, coupling coeffs, degree/rad windows) untouched."""
    return {k: (v * SCALE if k in keys else v) for k, v in d.items()}


# --------------------------------------------------------------- torso (split at the waist)
# BODY_W / BODY_H were widened + deepened by analysis/optimize.py: a wider torso
# sets a wider lateral FOOT BASE (foot base half = BODY_W/2 + hip_off), the single
# biggest lever on walking roll. The ribcage (body.py) + skin (shell.py) station
# tables scale off these two so the whole cat stays proportioned + buildable.
BODY_W_REF = 70.0     # baseline the hardcoded ribcage/skin stations were drawn at (FIXED ref)
BODY_H_REF = 45.0
BODY_L = 180.0 * SCALE
BODY_W = 94.0 * SCALE
BODY_H = 54.0 * SCALE
SHELL_T = 2.4         # print wall thickness (nozzle-driven, FIXED — not scaled)
# The torso is two shells joined by a sagittal WAIST joint (the cat spine): a
# front half (carries front legs + head) and a rear half (rear legs + tail).
FORE_LEN = 90.0 * SCALE
AFT_LEN = 90.0 * SCALE
LIM_WAIST = (-0.45, 0.65)   # rad: - = flex belly-down, + = arch back up (halloween cat)
LIM_HEAD_TILT = (-0.7, 0.7)   # rad: head roll — the quizzical tilt + camera-roll gimbal
LIM_HEAD_PITCH = (-0.7, 0.7)  # rad: head pitch — nod + camera-pitch gimbal

# --------------------------------------------------------------- legs (4 DOF, digitigrade)
# Per-leg segment lengths (mm) and the metatarsus (foot) pitch that gives the
# digitigrade toe-stance. Rear limbs are longer + more angled than the fore.
LEG_SEG_W = 13.0 * SCALE
TOE_R = 8.0 * SCALE   # toe pad contact radius

# Each leg now has 2 *motorized* joints (hip + knee). The ankle/hock is
# mechanically **coupled to the knee** (ankle = c0 + couple_c1·knee), mimicking
# the cat "reciprocal apparatus" tendon — one motor drives knee + hock together.
# ``stance_depth`` is the ANKLE depth the 2-link (hip+knee) plants at; the coupled
# foot hangs below it to the toe. ``couple_ankle`` = desired ankle angle at stance.
# Segment lengths + hip_off + stance_depth optimized (analysis/optimize.py) to
# minimise walk roll_pp + peak torque: wider hip_off widens the foot base, longer
# distal segments + a slightly deeper stance lower/stabilise the CoM, and the rear
# limb stays longer overall than the fore (digitigrade look).
# ``lim_knee`` = per-leg knee JOINT soft limit (rad). It is set to each leg's REAL
# four-bar reach (analysis/fourbar over P.FOURBAR crank window -20..81 deg) minus a
# small safety margin off the transmission-angle edge, so the sim/hardware never
# commands a knee the crank can't reach. Reach (from analysis.fourbar): FRONT knee
# 23.7..152.3 deg, REAR 35.4..164.0 deg — both on the crank window's monotonic,
# singularity-free branch (transmission angle 41..140 deg throughout). Upper caps:
# FRONT 2.62 rad (150.1 deg, crank 77.7 deg, mu 138.8 deg — 2.2 deg margin off reach)
# REAR  2.79 rad (159.9 deg, crank 75.0 deg, mu 137.7 deg — 4.1 deg margin off reach).
# The rear cap is deeper than the front on purpose: a cat's hindlimb (knee) folds
# MORE than its forelimb (elbow) — do not force these symmetric. Lower bound stays
# the generic 0.0 (never approached: the deepest-fold gaits/poses use knee >= 80 deg,
# far above the four-bar floor), so raising the cap can only ADD range, never clip
# the validated stance/walk. See LIM_KNEE below for the generic default.
_LEG_LEN_KEYS = ("hip_off", "upper", "lower", "foot", "stance_depth")
FRONT = _scale_lengths(dict(hip_off=32.5, upper=67.5, lower=52.0, foot=26.0,
                            stance_depth=76.0, couple_c1=-0.55, couple_ankle=-0.30,
                            lim_knee=(0.0, 2.62)), _LEG_LEN_KEYS)
REAR = _scale_lengths(dict(hip_off=39.0, upper=64.0, lower=80.5, foot=31.5,
                           stance_depth=80.0, couple_c1=-0.55, couple_ankle=-0.10,
                           lim_knee=(0.0, 2.79)), _LEG_LEN_KEYS)

# expression joints
ABDUCTION_ACTIVE = False   # legs are sagittal; turn by gait (fewer motors)
EARS_LINKED = True         # both ears on one motor (ear_R follows ear_L)

# --------------------------------------------------------------- four-bar knee linkage
# The knee is driven by a proximal (thigh-mounted) crank through a rigid four-bar
# (analysis/fourbar.py — verified: 2.24 rad ROM, transmission 41-140°, invertible;
# MuJoCo closed loop held to 0.16 mm). Keeps the shank light + no cable friction.
# ``ground`` = crank-pivot to knee-pivot distance up the thigh (so the crank pivot
# sits ``ground`` mm above the knee). Lengths mm.
KNEE_DRIVE = "fourbar"     # 'direct' | 'fourbar' (mechanical realisation)
FOURBAR = _scale_lengths(dict(ground=38.0, crank=27.0, coupler=50.0, rocker=18.0,
                              rocker_offset=0.0, crank_window=(-20.0, 81.0)),
                         ("ground", "crank", "coupler", "rocker"))

# leg mounts — coordinates are in each half's OWN frame (fore/aft origin at the
# waist; fore geometry extends +x, aft extends -x).
MOUNT_INSET = 16.0 * SCALE
MOUNTS = {
    "FL": (FORE_LEN - MOUNT_INSET, +BODY_W / 2),
    "FR": (FORE_LEN - MOUNT_INSET, -BODY_W / 2),
    "RL": (-(AFT_LEN - MOUNT_INSET), +BODY_W / 2),
    "RR": (-(AFT_LEN - MOUNT_INSET), -BODY_W / 2),
}
LEGS = ["FL", "FR", "RL", "RR"]
FRONT_LEGS = {"FL", "FR"}


def leg_parent(leg: str) -> str:
    return "torso_fore" if leg in FRONT_LEGS else "torso_aft"

# joint limits (radians)
LIM_ABD = (-0.9, 0.9)
LIM_HIP = (-2.6, 2.6)
LIM_KNEE = (0.0, 2.6)   # generic default; per-leg caps live in FRONT/REAR['lim_knee']
LIM_ANKLE = (-2.2, 2.2)

# --------------------------------------------------------------- head / ears / tail
HEAD_R = 46.0 * SCALE
NECK_L = 34.0 * SCALE
EYE_R = 11.0 * SCALE
EYE_SPACING = 40.0 * SCALE
CAM_R = 6.0           # internal camera bore radius (not in the scaled-dims list)
EAR_H = 34.0 * SCALE
EAR_BASE = 26.0 * SCALE
TAIL_L = 120.0 * SCALE
TAIL_BASE_R = 9.0 * SCALE

# --------------------------------------------------------------- sensor mounts (anatomy)
# Each sense organ in the skin (shell.py) houses its sensor as a modest, symmetric
# bore/grille. Bores are cylinder subtractions; a "grille" is a row of small
# cylinders (robust — no thin-shell ops). Kept small so the baby-schema face stays cute.
EYE_CAM_R = 3.0 * SCALE   # stereo camera lens aperture bored behind each eye centre
MIC_PORT_R = 1.6 * SCALE  # MEMS mic port near each ear base
NOSE_VENT_R = 1.3 * SCALE  # e-nose (gas sensor) intake grille hole radius
NOSE_VENT_N = 3        # holes across the nose intake grille (COUNT — not scaled)
NOSE_VENT_DY = 4.0 * SCALE  # lateral pitch of the nose intake holes (mm)
SPKR_GRILLE_R = 1.5 * SCALE  # speaker grille hole radius (muzzle / mouth)
SPKR_GRILLE_N = 5      # holes across the mouth speaker grille (COUNT — not scaled)
SPKR_GRILLE_DY = 4.5 * SCALE  # lateral pitch of the mouth grille holes (mm)
SPKR_GRILLE_SMILE = 2.0 * SCALE  # z-rise of the outer grille holes -> gentle smile curve (mm)

# --------------------------------------------------------------- stance
GROUND_CLEAR = 6.0
# nominal torso underside height = shortest stance depth (used to lift the preview)
STANCE_H = min(FRONT["stance_depth"], REAR["stance_depth"])

# --------------------------------------------------------------- masses (kg)
EFFECTIVE_DENSITY = 650.0
COMPONENT_MASS = {
    "servo_each": SERVO.mass_kg,
    # STS3215 serial bus: the PCA9685 PWM driver is gone (dropped 2026-07-10);
    # the whole chain is addressed by one small TTL bus adapter instead.
    "pi": 0.046, "battery_2s": 0.110, "bus_adapter": 0.008,
    "imu": 0.003, "camera": 0.004, "speaker": 0.010, "wiring_misc": 0.070,
}
# 14 motors: legs 2×4 (hip+knee) + waist + head pan/pitch/tilt (2-axis gimbal) + ears(1) + tail
N_SERVOS = 8 + 1 + 3 + 1 + 1

# --------------------------------------------------------------- gait (trot: diagonal pairs)
GAIT_PHASE = {"FL": 0.0, "RR": 0.0, "FR": 0.5, "RL": 0.5}


def is_front(leg: str) -> bool:
    return leg in FRONT_LEGS


def leg_geom(leg: str) -> dict:
    return FRONT if is_front(leg) else REAR


def leg_knee_limit(leg: str) -> tuple[float, float]:
    """Per-leg knee JOINT soft limit (rad), from the leg's real four-bar reach.
    Falls back to the generic LIM_KNEE if a leg dict omits ``lim_knee``."""
    return leg_geom(leg).get("lim_knee", LIM_KNEE)


def leg_plane_sign(leg: str) -> int:
    return 1 if leg.endswith("L") else -1


def component_mass_total() -> float:
    c = COMPONENT_MASS
    return (c["servo_each"] * N_SERVOS + c["pi"] + c["battery_2s"]
            + c["bus_adapter"] + c["imu"] + c["camera"] + c["speaker"]
            + c["wiring_misc"])


MASS_TARGET = (0.8, 1.6)          # kg (raised ceiling: more servos for anatomy)
BODY_LEN_TARGET = (180.0, 250.0)

# =============================================================== print fits + fasteners (FDM)
# Single source of truth for every MATING clearance and every fastener feature, so a
# printer / nozzle / insert change is ONE edit here. All parts pull these through
# ``cad.parts.fasteners``. Radii unless noted (mm); "radial" = added to a bore radius.
# These ADD print/assembly features only — no joint origin, segment length, four-bar
# (FOURBAR) or servo choice is touched.
#
# Fit classes (apply per mating feature):
#   FIT_CLEARANCE      slip / registration fit for part-to-part locating spigots
#   PIN_CLEARANCE      radial gap for a ROTATING pin-in-bore pivot (loose, low friction)
#   PRESS_INTERFERENCE radial interference for a FIXED pressed pin / dowel (bore smaller)
# The servo-pocket per-face clearance stays in cad/servo.py (SERVO.pocket, 0.4 mm/face).
FIT_CLEARANCE = 0.20
PIN_CLEARANCE = 0.15
PRESS_INTERFERENCE = 0.10

# Four-bar / knee pivot pin — unchanged Ø3 nominal (do NOT move the pivots; this is
# only the pin stock the clearance/press fits are measured against).
PIN_R = 1.5                       # nominal pin radius (Ø3 dowel / M3 shoulder screw)

# Brass HEAT-SET inserts (melt-in) + matching machine screws. ``bore_r`` = the melt-in
# hole the brass seats into; boss OD must be >= bore + 2*min_wall so the wall doesn't
# split. Values are for common tapered M2 / M3 heat-set inserts (CNC-Kitchen class);
# verify against the actual insert before ordering screws.
HEATSET = {
    "M2": dict(bore_r=1.60, depth=4.0, boss_r=3.4, min_wall=1.8),
    "M3": dict(bore_r=2.05, depth=5.7, boss_r=4.3, min_wall=2.0),
}
# Screw clearance (through-hole radius) + head counterbore, by thread.
SCREW = {
    "M2": dict(clr_r=1.25, head_r=2.00, head_h=2.0),
    "M3": dict(clr_r=1.75, head_r=2.90, head_h=3.0),
}

# Servo horn interface — the metal STS3215 horn (Ø20 disc) drives a printed part via a
# small bolt circle of screws INTO heat-set inserts in the part (+ the centre horn screw).
HORN_DIA = SERVO.horn_dia          # 20 mm STS3215 output disc
HORN_BOLT_CIRCLE = 15.0            # dia of the horn screw circle
HORN_SCREWS = 4                    # screws around the horn
HORN_SCREW = "M2"                  # thread of the horn screws
HORN_CENTER_R = 1.6               # centre horn-shaft screw clearance (Ø3.2)

# Servo CASE retention — screws through the printed boss into the servo's own tapped
# flange holes (STS3215 uses M2). CLEARANCE holes in the plastic; placed parallel to
# the existing shaft-relief bore in each boss, on this rectangle around the shaft.
# Pattern verified indicative — check the STS3215 datasheet before drilling metal.
SERVO_MOUNT_SCREW = "M2"
SERVO_MOUNT_INSET = 6.0            # hole inset from the servo body edge on the mount face

# Wire pass-throughs for the STS3215 3-wire TTL daisy-chain + sensor cables. Just the
# physical holes (no connector bodies). Chain order: hardware/servo_channel_map.py.
BUS_WIRE_R = 3.0                   # daisy-chain trunk pass-through radius (3-wire loom)
SENSOR_WIRE_R = 2.0               # sensor cable pass-through radius (cam / mic / IMU / spkr)

# =============================================================== remote HIP drive (core servo + lateral axle)
# The hip/shoulder STS3215 no longer sits BESIDE the hip joint (its fat 45x24x36 body
# floored at the joint line |y|=79.5/86, forcing the skin to flare past it → broad
# shoulders). Instead the servo BODY moves INTO the torso CORE near the centreline, and
# a lateral drive AXLE carries the torque OUT along the (UNCHANGED) Y hip axis to the hip
# pivot, running in two bearings — one at the core wall, one at the slim hip bracket. The
# shoulder band (|y| 47→79.5) then carries only a Ø6 axle + a Ø13 bearing boss, so the
# skin can hug the body.  This is a servo LOCATION + transmission change ONLY: the hip is
# still ONE Y hinge at the same origin (MOUNTS + hip_off + FOURBAR + stance untouched);
# the axle is a rigid coupling, invisible to the kinematic model.
HIP_DRIVE = "remote_axle"          # 'inboard' (old tuck) | 'remote_axle' (servo in core)
AXLE_R = 3.0                       # hip drive shaft radius (Ø6 hardened steel / CF rod)
AXLE_CLEAR = PIN_CLEARANCE         # radial running clearance of the shaft in a plain bore
# hip drive bearing: a 686-class deep-groove ball bearing (Ø6 bore, Ø13 OD, 5 mm wide).
HIP_BEARING = dict(bore_r=3.0, od_r=6.5, width=5.0)
HIP_BOSS_WALL = 3.0                # wall of the printed bearing boss around the OD
# Where the core servo sits: its HORN face lands on the hip axis at this |y| (inboard of
# the joint), the ~36 mm body reaching further inboard toward the centreline. Picked so
# the two back-to-back L/R hip servos share the axis WITHOUT meeting at y=0 and stay
# inside the core half-width (BODY_W/2 = 47). Axle length = (BODY_W/2 + hip_off) - this.
HIP_CORE_HORN_Y = 40.0

# =============================================================== print SPLIT (cut + bond)
# The three biggest parts exceed a typical 220-250 mm bed and/or need heroic supports,
# so ``cad/parts/split.py`` cuts them at PRINT time and injects mating features. The WHOLE
# parts (torso_fore/torso_aft/head) are untouched — the sim/mass model still sees them
# intact; only the printed sub-parts carry the cut + bond geometry. Cut planes:
#   torso_fore / torso_aft -> SAGITTAL (Y=0): L (+y) / R (-y) halves; each hoop arches off
#                             the flat XZ cut face, bonds along the spine + keel stringers.
#   head                   -> EQUATORIAL brow-line (Z=HEAD_SPLIT_Z): a lower "face bowl"
#                             (eyes + camera + muzzle + neck stub, all intact + open to fit)
#                             and an upper cranial cap; each prints cut-face-down, dome up.
# Registration uses SEPARATE Ø4 dowel RODS (not printed-in-place bosses — a printed boss on
# a cut-face-down half would point into the bed). Each cut adds a little bond material
# (spine/keel pads on the torso, an internal ring flange on the head) to give a real flat
# glue face + host the dowel sockets; the sockets are press-fit on one half, PIN_CLEARANCE
# slip on the other, so a rod locks into one side and the halves still separate for gluing.
SPLIT_DOWEL_R = 2.0                # Ø4 alignment dowel rod (steel/PLA); press one side, slip other
SPLIT_DOWEL_DEPTH = 6.5            # blind socket depth into EACH half (rod ~= 2*depth - 1 mm)
SPLIT_PAD = (14.0, 16.0, 12.0)     # torso spine/keel bond pad (X, Y across cut, Z) — flat glue land
SPLIT_GLUE_W = 2.0                 # glue-relief squeeze-out channel width (mm)
SPLIT_GLUE_D = 0.6                 # glue-relief channel depth into the bond face (mm)
HEAD_SPLIT_Z = 8.0                 # head equatorial cut height (brow line; clears eye tops z=+7)
HEAD_PAD_H = 13.0                  # head bond-pad height, centred on the cut (>= 2*dowel depth)
HEAD_PAD_XY = 15.0                 # head bond-pad footprint (bounded by the sphere, no bulge)
HEAD_DOWEL_R = 39.0                # radius of the 3-dowel bond-pad circle in the head wall
