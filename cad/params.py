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
# Head ROLL is not a joint. The head cavity holds two servo housings, not three
# (analysis.actuator_fit.capacity), and a third gimbal axis would need a ~Dia124 head.
# Roll is the one axis a camera can fix in software -- it is a rotation in the image
# plane -- so it is handled by electronic stabilisation (docs/camera_stabilization.md)
# and the mechanism keeps pan + pitch, which actually have to aim.
HEAD_ROLL_ACTUATED = False
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
# Ears are FIXED (bolted to the head, not driven). This is not a styling choice: the body
# has three places an expression actuator can live forward of the waist (two in the head,
# one in the chest -- ``analysis.actuator_fit.capacity``) against four joints that wanted
# one. PLAN.md's own minimum viable set breaks the tie: "Ears can be a fast-follow".
# They stay in the kinematics as rigid links so the silhouette and mass are unchanged, and
# ``brain.hal.Body.set_ears`` stays in the interface -- a later revision with a smaller
# actuator can drive them without the brain changing.
EARS_ACTUATED = False
EARS_LINKED = True         # if ever actuated: both ears on one motor (ear_R follows ear_L)

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
# Head size is an ACTUATOR constraint as much as a styling one. ``analysis.actuator_fit``
# measures how many servo housings the head cavity admits when they are laid out as a
# centred group: 1 up to R=48.6, 2 from there, and not 3 until R=61 (a head 68% of the
# body's length, which is past baby-schema into caricature). R=50 buys the second slot
# with margin, at +5 mm of diameter and ~12 g of shell.
HEAD_R = 50.0 * SCALE
NECK_L = 34.0 * SCALE
# Where the neck gimbal's first joint (pan) sits on the chest. The head centre lands
# HEAD_GIMBAL_STACK mm ahead of it, and it matters where that is: with the head centre
# INSIDE the ribcage, ``body._head_socket`` carves out the whole front of the chest and no
# actuator can be housed there at all. Pushing the pan joint forward until the head centre
# clears the torso's nose opens exactly one chest slot (measured; further out adds nothing,
# the chest's own taper is then the limit).
# =============================================================== head gimbal (REMOTE)
# Neither gimbal axis can be driven directly. ``analysis.actuator_fit.gimbal_layout``
# measures it: a gimbal's axes meet at a point, so its motors have to step aside ALONG
# their own axes, and two STS3215 housings do not both fit that way until about a Dia148
# head. So both axes are driven the way the knee, the hip and the tail already are --
# actuator where there is room, four-bar to the joint.
#
# The topology is forced. The head is the last link in the chain, so a servo inside it can
# only reach the ONE joint immediately above it (pitch). The other axis (pan) therefore has
# to be driven from further up the chain, and the only cavity there is the chest.
#
#   chest servo --four-bar--> PAN axis (neck yaw)    remote  [PAN_SERVO, PAN_FOURBAR]
#   head servo  --on the axis-> PITCH axis (nod)       direct
#
# Only ONE of the two has to be remote. A single housing DOES fit on its own axis, centred
# in the head -- it is the second one that has nowhere to step aside to. So pitch is driven
# directly (servo in the head, horn bolted to the neck column) and only pan is linked out.
# That leaves the head's other housing slot for the cameras.
#
# The pan axis is pulled BACK from the head centre so the pan linkage has a horizontal
# plane to run in under the head ball: at the head centre the ball leaves 2 mm, at 18 mm
# behind it there is 15 mm. The head itself does not move -- only the yaw axis does, which
# gives the head a slight sideways shift as it turns, the way a real neck does.
HEAD_DRIVE = "remote_fourbar"
HEAD_GIMBAL_STACK = 18.0           # pan axis -> pitch axis (= the head centre)
HEAD_MOUNT_X = FORE_LEN - HEAD_GIMBAL_STACK
LIM_HEAD_PAN = (-1.0, 1.0)         # rad: what the pan four-bar delivers with a crank small
                                   # enough to stay clear of the waist (2.07 rad available)

# Pan: servo shaft (x, z) in the fore-torso frame, from actuator_fit.capacity('chest'),
# dropped to the underside so its crank sweeps below the head ball (which reaches
# down to z = -24 at the pan axis; the chest floor is at z = -39).
PAN_SERVO = (31.0, -23.0)
# The crank is deliberately SMALL. A longer one reaches the same range with a nicer
# transmission angle, but its swept disc runs back past the waist plane -- at crank 27
# it reaches x = -0.5, into the aft half. At 17 it stops at x = +9.5.
PAN_FOURBAR = dict(crank=17.0, coupler=47.0, rocker=13.5, crank_window=(-22.0, 87.0))

# Pitch: the servo sits ON the pitch axis at the head's centre, its horn bolted to the
# neck column, so the head nods against the column. No linkage.
PITCH_DRIVE = "direct"
# The pitch servo has to be CENTRED in the head — a 42 mm-deep case hung off one side of
# the pitch axis reaches outside the shell — so its horn lands ~21 mm off the head's centre
# plane. The neck column therefore ends in a C-YOKE that straddles the head: the horn pad
# on one arm, a plain pivot on the other, so the head is supported on both sides rather
# than cantilevered off the servo spline.
def pitch_yoke_face() -> float:
    """|y| of the pitch servo's horn face — half its housing depth, since the servo has to
    sit centred in the head."""
    from cad.servo import DEFAULT as _S
    return (_S.pocket[2] + 2 * HIP_BOSS_WALL) / 2.0


def pitch_yoke_y() -> float:
    """|y| of the neck yoke's arms.

    The arms carry PADS of ``CLEVIS_TONGUE_T``, and it is the pad's INNER face that has to
    clear the servo's boss — sizing to the pad's centre leaves half its thickness buried in
    the case, which is an interference, not a bearing."""
    return pitch_yoke_face() + 1.0 + CLEVIS_TONGUE_T


PITCH_YOKE_Y = 24.0                # legacy alias; prefer pitch_yoke_y()
PITCH_YOKE_ARM = 8.0               # yoke arm cross-section (mm)


PAN_BEARING_Z = -35.0              # the neck's yaw bearing, in the chest under the ball
NECK_COLUMN_W = 14.0               # column cross-section (mm)


PAN_LINK_Z = 8.5                   # pan linkage plane, in the neck column's own frame
                                   # (global z = PAN_BEARING_Z + this = -26.5: below the
                                   # head ball, which stops at -16, and above the chest
                                   # floor, which is at -34 under the servo)


def pan_ground() -> float:
    """O2->O4 for the pan four-bar: chest servo shaft to the neck yaw axis (mm)."""
    return HEAD_MOUNT_X - PAN_SERVO[0]


def pan_origin() -> tuple[float, float, float]:
    """The yaw joint, in the fore-torso frame. It sits at the BEARING, down in the chest
    under the head ball, not up at the head — the axis is the same vertical line either
    way, and this is the end of it the torso can actually support."""
    return (HEAD_MOUNT_X, 0.0, PAN_BEARING_Z)


def pitch_origin_local() -> tuple[float, float, float]:
    """The nod joint, in the neck column's own frame: forward and up from the yaw bearing
    to the head's centre."""
    return (HEAD_GIMBAL_STACK, 0.0, head_centre()[2] - PAN_BEARING_Z)


# The head sits above the shoulder line rather than level with it. That is how a cat is
# built, and here it is also what gives the pan linkage a plane to run in: the head ball
# reaches down to z = -24 at the yaw axis, and the pan rocker needs to pass UNDER it.
HEAD_RISE = 8.0


def head_centre() -> tuple[float, float, float]:
    """Head centre in the fore-torso frame — the one definition of where the head is."""
    return (HEAD_MOUNT_X + HEAD_GIMBAL_STACK, 0.0, BODY_H / 2 - 2 + HEAD_RISE)
EYE_R = 11.0 * SCALE
EYE_SPACING = 40.0 * SCALE
CAM_R = 6.0           # internal camera bore radius (not in the scaled-dims list)
EAR_BASE_H = 6.0           # flat foot of the ear, seated on the head's mounting pad
EAR_FOOT_W = 10.0          # foot width across the head (Y)
EAR_PAD_PROUD = 1.0        # how far the pad stands off the sphere, so the foot lands flat


def ear_station(side: int = 1) -> tuple[float, float, float]:
    """Where an ear bolts to the head, in the head's own frame.

    The ear used to be pinned at a point INSIDE the sphere, so its blade grew out through
    the shell. A bolted-on ear has to start at the surface -- and not at the surface under
    the middle of its foot, but above the HIGHEST point of the sphere anywhere under the
    foot, or the curvature lifts the shell through the inboard corner.
    """
    import math
    x = HEAD_R * 0.3
    y = side * EYE_SPACING / 2
    # the foot's corner nearest the head's axis is where the sphere rises highest
    cx = max(abs(x) - EAR_BASE * 0.25, 0.0)
    cy = max(abs(y) - EAR_FOOT_W / 2, 0.0)
    z = math.sqrt(max(HEAD_R ** 2 - cx * cx - cy * cy, 1.0))
    return (x, y, z + EAR_PAD_PROUD)


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
# 12 motors: legs 2x4 (hip+knee) + waist + head pan/pitch + tail. The ears are rigid
# (EARS_ACTUATED) and the head has no roll axis (HEAD_ROLL_ACTUATED), so neither costs one.
N_SERVOS = (8 + 1
            + (3 if HEAD_ROLL_ACTUATED else 2)
            + (1 if EARS_ACTUATED else 0)
            + 1)

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

# =============================================================== joint clevis (double shear)
# Every pin joint is a CLEVIS: one link ends in a two-cheek FORK, the mating link ends
# in a TONGUE that sits in the slot between the cheeks, and the pin passes through all
# three. This is what makes the leg assemblable (two hubs cannot occupy one space) and
# it puts every pivot in DOUBLE SHEAR instead of cantilevering the pin.
#
# The thigh is the extreme case: its fork does not stop at the knee but runs the whole
# way up to the crank pivot, so the thigh is a CHANNEL whose slot houses, in one plane,
# the four-bar crank, the rocker welded to the shank, and the shank's own knee tongue.
# The pushrod is offset sideways within that slot (it shares pins with both the crank
# and the rocker, so it cannot be coplanar with them).
#
#   thigh cheek | pushrod | crank / rocker / shank-tongue | thigh cheek
#   <-CHEEK_T-> <-ROD_T-> <---------TONGUE_T------------> <-CHEEK_T->
#
# Changing TONGUE_T re-fits every fork slot, every tongue and the thigh width at once.
CLEVIS_TONGUE_T = 7.0     # in-plane link thickness (crank, rocker, shank tongue)
CLEVIS_ROD_T = 6.0        # pushrod thickness — its own lane, outboard of the in-plane links
CLEVIS_GAP = 0.35         # per-face running clearance inside a fork slot
CLEVIS_CHEEK_T = 4.5      # each thigh/shank fork cheek — >= 4 perimeters at a 0.4 nozzle

# LANE STACK across the thigh channel, inboard -> outboard (leg-local y, times leg side):
#
#   knee servo body | crank / rocker / shank tongue | pushrod | outboard cheek
#     y <= -3.85    |        -3.5 .. +3.5           | 3.85..9.85 |  10.2 .. 14.7
#
# The knee servo drives the crank from inboard, so the crank has to be the lane next to
# it; the pushrod shares the crank's C pin and the rocker's R pin, so it takes the next
# lane out. (A pushrod forked around the crank is the textbook arrangement, but its
# inboard blade would have to pass between the crank and the servo, which there is no
# room for at this scale — so the rod is single-blade and both pins are shouldered.)
FB_ROD_Y = CLEVIS_TONGUE_T / 2 + CLEVIS_GAP + CLEVIS_ROD_T / 2          # 6.85 mm


def clevis_slot(rod: bool = False) -> tuple[float, float]:
    """(lo, hi) y-bounds of the slot a fork must leave, in the leg's local frame.

    ``rod=False`` — a plain pivot: just the symmetric tongue lane (knee, ankle).
    ``rod=True``  — the thigh channel, which must additionally pass the pushrod in its
    own outboard lane, so the slot is asymmetric.
    """
    hi = CLEVIS_TONGUE_T / 2 + CLEVIS_GAP
    lo = -hi
    if rod:
        hi = FB_ROD_Y + CLEVIS_ROD_T / 2 + CLEVIS_GAP
    return (lo, hi)


def pan_rod_dz() -> float:
    """Height of the pan pushrod's lane above the crank/rocker plane.

    The crank and the rocker are in-plane links that share the C and R pins with the rod,
    so the rod cannot be coplanar with them — the same lane discipline the leg and the tail
    linkages use, turned on its side because this linkage lies flat."""
    return CLEVIS_TONGUE_T / 2 + CLEVIS_GAP + CLEVIS_ROD_T / 2


def thigh_profile() -> tuple[float, float]:
    """(half-width, y-offset of the centre) of the thigh's outer section — sized so the
    asymmetric channel keeps a full cheek on both sides."""
    lo, hi = clevis_slot(rod=True)
    return ((hi - lo) / 2.0 + CLEVIS_CHEEK_T, (lo + hi) / 2.0)


# The knee-servo horn face lands on the inboard wall of the crank's lane.
FB_HORN_Y = CLEVIS_TONGUE_T / 2 + CLEVIS_GAP                     # 3.85 mm
HORN_SEAT_CLEAR = 0.6    # radial clearance around the Ø20 horn where it sits in a bore
HORN_DISC_T = 2.5        # thickness of the metal STS3215 output disc

# =============================================================== leg -> torso mount
# The hip bracket is a bolted part, not a solid fused into the ribcage: it lands on a
# flat pad on the torso flank at |y| = BODY_W/2 and is held by MOUNT_SCREWS screws into
# heat-set inserts in that pad.
MOUNT_SCREW = "M3"
MOUNT_SCREWS = 2                  # screws per leg bracket
MOUNT_PAD = (26.0, 14.0)          # torso mount pad footprint (X, Z), mm —
                                  # kept short in Z so the bolted foot stays clear of
                                  # the knee servo boss swinging past it
MOUNT_PAD_T = 5.0                 # pad thickness (grows inboard from the flank plane)
MOUNT_BOLT_PITCH = 16.0           # screw spacing along X on the pad
MOUNT_FACE_GAP = 0.15             # bracket-to-pad seating gap (a print-fit, not a joint)

# =============================================================== waist joint clearance
# The two torso halves ROTATE against each other about the waist axis, so their innermost
# ribs cannot both sit on the waist plane. Each half's waist-end station is pushed back
# by this much, leaving 2*WAIST_CLEAR between the two rib faces.
WAIST_CLEAR = 3.0

# =============================================================== tail REMOTE drive
# The tail pivot sits at the rear extremity of the frame, where the body has tapered to
# almost nothing -- ``analysis.actuator_fit`` measures that no STS3215 can be housed
# there (44.7% of its boss falls outside the ribcage). So the tail joint is driven the
# way the hip already is: the actuator moves to where there IS room and reaches the joint
# through a linkage. Here that is a crank-rocker four-bar, the same mechanism as the knee.
#
# ``TAIL_SERVO`` is the (x, z) of the servo's OUTPUT SHAFT in the aft-torso frame, chosen
# from ``analysis.actuator_fit.capacity('aft')`` -- the one station where the housing fits.
TAIL_DRIVE = "remote_crank"        # 'direct' (servo on the joint) | 'remote_crank'
TAIL_SERVO = (-54.0, 0.0)
# Link lengths from a search over analysis.fourbar with this ground distance, keeping the
# transmission angle inside the same 40-140 deg band the knee uses. Gives 2.14 rad of
# rocker travel against the +-1.0 rad the tail joint actually needs.
TAIL_FOURBAR = dict(crank=21.5, coupler=45.0, rocker=18.0, crank_window=(-28.5, 84.0))
LIM_TAIL = (-1.0, 1.0)             # rad, matches cad/assembly.py's tail link


def tail_ground() -> float:
    """O2->O4 distance for the tail four-bar: servo shaft to tail pivot (mm)."""
    import math
    sx, sz = TAIL_SERVO
    return math.hypot(-AFT_LEN - sx, BODY_H / 4 - sz)


def tail_lane_y() -> tuple[float, float, float]:
    """(horn face, crank/rocker lane centre, pushrod lane centre) for the tail drive.

    Same clevis discipline as the leg: the tail forks around a tongue on the torso, the
    rocker is welded straight onto the tail's OUTER cheek so no web is needed, and the
    pushrod runs in its own lane beside the in-plane links.
    """
    lo, _ = clevis_slot()
    horn = -lo + CLEVIS_CHEEK_T                      # outer face of the tail's outer cheek
    link = horn + CLEVIS_TONGUE_T / 2
    rod = link + CLEVIS_TONGUE_T / 2 + CLEVIS_GAP + CLEVIS_ROD_T / 2
    return (horn, link, rod)
