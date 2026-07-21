"""
MuJoCo emulation — does the printed design actually stand and walk?
===================================================================

    python -m sim.mj_emulate --gait trot                 # headless: report + renders
    python -m sim.mj_emulate --gait walk --seconds 8
    python -m sim.mj_emulate --gait trot --view          # interactive 3-D viewer
    python -m sim.mj_emulate --gait trot --brain         # Phase-0 brain drives the body

Drives the model from ``sim/gait.py`` (the same foot-trajectory + leg IK that
becomes servo setpoints on the real robot), under the servo torque limits baked
into the MJCF. Reports whether the torso stayed upright, how far it travelled,
and peak joint torque vs the servo's stall — then writes renders to ``sim/out/``.
"""

from __future__ import annotations

import argparse
import math
import os

import numpy as np
import mujoco

from cad import params as P
from cad.servo import DEFAULT as SERVO
from sim import gait
from sim.mjcf import build_mjcf

OUT = os.path.join(os.path.dirname(__file__), "out")
LEG_JOINTS = [f"{leg}_{j}" for leg in P.LEGS for j in ("hip", "knee")]   # motorized
ACTUATED = LEG_JOINTS + ["torso_aft", "head_pan", "head_pitch", "head_tilt", "ear_L", "tail"]
ALL_HINGES = ([f"{leg}_{j}" for leg in P.LEGS for j in ("hip", "knee", "ankle")]
              + ["torso_aft", "head_pan", "head_pitch", "head_tilt", "ear_L", "ear_R", "tail"])

# Max rate (rad/s) a motorized joint target may slew. Abrupt setpoint steps from
# the gait are what drove the position servo into its stall torque; ramping them
# keeps the tracking error (and thus kp*error torque) bounded.
SLEW_RATE = 6.0
# Soft-start: ease the gait targets in from the reset stance pose over this many
# seconds. At power-on the body drops onto its feet while the first gait command
# steps the legs away from stance; blending from stance -> full gait removes that
# coincident spike (which was pinning the servo at 100% of stall).
GAIT_RAMP_S = 0.6

# --- IMU body-leveling (active roll/pitch trim) -------------------------------
# Read torso roll/pitch (the BNO085 signal on hardware) and add a PD trim to each
# leg's commanded stance DEPTH so the low/sagging corner is pushed down (raised)
# and the high corner relaxed -> the torso is actively held level. This is the
# highest-impact waddle fix on a sagittal-only (no-abduction) leg set.
# Gains are in mm-of-depth per rad (P) and per rad/s (D). Kept modest so the
# trim assists gravity/gait rather than fighting the foot trajectory.
STAB_KP_ROLL = 52.0
STAB_KD_ROLL = 6.0
STAB_KP_PITCH = 16.0
STAB_KD_PITCH = 2.0
STAB_MAX = 18.0        # clamp on the per-leg depth trim (mm) — never over-drive a leg
STAB_RAMP_S = 0.6      # ease the trim in with the gait soft-start


# ------------------------------------------------------------------ model helpers
def _id(model, objtype, name):
    return mujoco.mj_name2id(model, objtype, name)


class Rig:
    """Model + data + name->index maps + initial stance pose."""

    def __init__(self, xml: str | None = None):
        self.model = mujoco.MjModel.from_xml_string(xml if xml is not None else build_mjcf())
        self.data = mujoco.MjData(self.model)
        self.act = {n: _id(self.model, mujoco.mjtObj.mjOBJ_ACTUATOR, n) for n in ACTUATED}
        self.qadr = {n: self.model.jnt_qposadr[_id(self.model, mujoco.mjtObj.mjOBJ_JOINT, n)]
                     for n in ALL_HINGES}
        self.torso = _id(self.model, mujoco.mjtObj.mjOBJ_BODY, "torso_fore")
        self.head = _id(self.model, mujoco.mjtObj.mjOBJ_BODY, "head_tilt")  # camera mount
        # slew-rate limiter: cap how fast a motorized target may move (rad/s) so
        # step-changes in the gait setpoint ramp in over several control ticks
        # instead of spiking the position servo against its stall torque.
        self.slew_rate = SLEW_RATE          # rad/s; None disables limiting
        self.ctrl_dt = 0.01                 # control period (matches ctrl_every below)
        self._cmd: dict[str, float] = {}    # last commanded (slew-limited) target
        self._stab_prev = (0.0, 0.0)        # last (roll, pitch) for the leveling D-term
        self._reset_stance()

    def _reset_stance(self):
        mujoco.mj_resetData(self.model, self.data)
        for leg in P.LEGS:
            hip, knee = gait.stance_angles(leg)
            self.set_target(f"{leg}_hip", hip, hard=True)
            self.set_target(f"{leg}_knee", knee, hard=True)
            self.set_target(f"{leg}_ankle", gait.ankle_from_knee(leg, knee), hard=True)
        for n in ("torso_aft", "head_pan", "head_pitch", "head_tilt", "ear_L", "tail", "ear_R"):
            self.set_target(n, 0.0, hard=True)
        mujoco.mj_forward(self.model, self.data)

    def set_target(self, joint, angle, hard=False):
        if joint in self.act:                       # only motorized joints take ctrl
            if hard or self.slew_rate is None:      # snap (stance reset) or no limiting
                cmd = angle
            else:                                   # ramp toward target at <= slew_rate
                prev = self._cmd.get(joint, float(self.data.ctrl[self.act[joint]]))
                step = self.slew_rate * self.ctrl_dt
                cmd = prev + max(-step, min(step, angle - prev))
            self.data.ctrl[self.act[joint]] = cmd
            self._cmd[joint] = cmd
        if hard and joint in self.qadr:             # coupled joints: set qpos only
            self.data.qpos[self.qadr[joint]] = angle

    # -- readouts --
    def torso_z(self):
        return float(self.data.xpos[self.torso][2])

    def torso_tilt(self):
        rzz = float(self.data.xmat[self.torso].reshape(3, 3)[2, 2])
        return math.acos(max(-1.0, min(1.0, rzz)))

    def torso_roll_pitch(self):
        """(roll about +x, pitch about +y) of the torso in radians. Same
        extraction as the camera; this is what the IMU (BNO085) reports and what
        the body-leveling PD trims against."""
        R = self.data.xmat[self.torso].reshape(3, 3)
        pitch = math.atan2(-R[2, 0], math.hypot(R[2, 1], R[2, 2]))
        roll = math.atan2(R[2, 1], R[2, 2])
        return roll, pitch

    def peak_leg_torque(self):
        return max(abs(float(self.data.actuator_force[self.act[n]])) for n in LEG_JOINTS)

    def camera_angles(self):
        """(pitch, roll) of the head-mounted camera in degrees — the shake that
        matters for the vision feed."""
        R = self.data.xmat[self.head].reshape(3, 3)
        pitch = math.degrees(math.atan2(-R[2, 0], math.hypot(R[2, 1], R[2, 2])))
        roll = math.degrees(math.atan2(R[2, 1], R[2, 2]))
        return pitch, roll

    def body_point(self, name):
        return self.data.xpos[_id(self.model, mujoco.mjtObj.mjOBJ_BODY, name)].copy()

    def foot_point(self, leg):
        gid = _id(self.model, mujoco.mjtObj.mjOBJ_GEOM, f"{leg}_ankle_toe")
        return self.data.geom_xpos[gid].copy()


# ------------------------------------------------------------------ body leveling
def level_trim(rig: Rig, t: float, scale: float = 1.0):
    """Per-leg stance-DEPTH trim (mm) that holds the torso level from the IMU.

    Reads torso roll/pitch (the BNO085 signal), runs a PD on each, and returns a
    ``{leg: depth_delta_mm}`` map: push the low corner's foot down (raise it) and
    relax the high corner. Sagittal legs can't shift weight sideways, so this
    per-corner height trim is what actually cancels the passive waddle-roll.

    ``scale`` (preset ``"level"``) tunes the leveling authority per gait: the walk
    gets full authority (1.0); the aggressive trot uses a fraction so the extra
    depth trim doesn't add to its already near-stall peak torque."""
    if scale <= 0.0:
        return {leg: 0.0 for leg in P.LEGS}
    roll, pitch = rig.torso_roll_pitch()
    p_roll, p_pitch = rig._stab_prev
    droll = (roll - p_roll) / rig.ctrl_dt
    dpitch = (pitch - p_pitch) / rig.ctrl_dt
    rig._stab_prev = (roll, pitch)
    ramp = min(1.0, t / STAB_RAMP_S) if STAB_RAMP_S > 0 else 1.0
    roll_cmd = scale * (STAB_KP_ROLL * roll + STAB_KD_ROLL * droll)
    pitch_cmd = scale * (STAB_KP_PITCH * pitch + STAB_KD_PITCH * dpitch)
    trims = {}
    for leg in P.LEGS:
        side = 1.0 if leg.endswith("L") else -1.0     # +1 left, -1 right
        front = 1.0 if leg in P.FRONT_LEGS else -1.0  # +1 front, -1 rear
        corr = -side * roll_cmd + front * pitch_cmd
        corr = max(-STAB_MAX, min(STAB_MAX, corr))
        trims[leg] = ramp * corr
    return trims


# ------------------------------------------------------------------ head gimbal (camera stab)
# The head carries the camera and has a roll joint (head_tilt, axis x) on the SAME
# axis as the torso's residual walking roll. Counter-rotating head_tilt by the IMU
# roll holds the camera level while the body moves — a 1-DOF active gimbal using an
# existing motor (no new hardware). Runs identically on the robot (BNO085 -> servo).
HEAD_STAB_KROLL = 1.0     # counter-roll gain -> head_tilt (proportional; >1 overshoots)
HEAD_STAB_KPITCH = 1.0    # counter-pitch gain -> head_pitch


def head_stabilize(rig: Rig, t: float):
    """2-axis camera gimbal: counter the torso's roll with head_tilt and its pitch
    with head_pitch, so the head-mounted camera stays level in both axes while the
    body moves. Uses the IMU (BNO085) signal; runs identically on hardware."""
    roll, pitch = rig.torso_roll_pitch()
    lo_r, hi_r = P.LIM_HEAD_TILT
    lo_p, hi_p = P.LIM_HEAD_PITCH
    rig.set_target("head_tilt", max(lo_r, min(hi_r, -HEAD_STAB_KROLL * roll)))
    rig.set_target("head_pitch", max(lo_p, min(hi_p, -HEAD_STAB_KPITCH * pitch)))


# ------------------------------------------------------------------ gait controller
def gait_control(rig: Rig, t: float, preset: dict):
    cycle = preset["cycle"]
    phase = (t / cycle) % 1.0
    depth0 = lambda leg: gait.leg_depth(leg) - preset.get("settle", 0.0)
    ramp = min(1.0, t / GAIT_RAMP_S) if GAIT_RAMP_S > 0 else 1.0
    trims = level_trim(rig, t, preset.get("level", 1.0))
    for leg in P.LEGS:
        x, depth = gait.foot_target(leg, phase, preset, depth0(leg))
        depth += trims[leg]                      # IMU body-leveling trim (mm)
        hip, knee = gait.leg_ik(leg, x, depth)   # ankle follows via coupling
        if ramp < 1.0:                           # soft-start: blend in from stance
            hip0, knee0 = gait.stance_angles(leg)
            hip = hip0 + ramp * (hip - hip0)
            knee = knee0 + ramp * (knee - knee0)
        rig.set_target(f"{leg}_hip", hip)
        rig.set_target(f"{leg}_knee", knee)
    rig.set_target("torso_aft", gait.spine_wave(phase, preset))  # subtle feline spine undulation
    head_stabilize(rig, t)                       # active camera-roll gimbal


# ------------------------------------------------------------------ run
def simulate(gait_name: str, seconds: float, control_fn=None, rig=None,
             render=True, camera="cam", cam_setup=None):
    """Run a gait. ``rig`` lets a caller (e.g. the optimizer) supply a
    pre-built model; ``render=False`` skips the (slow) 3-D Renderer for fast
    headless scoring. ``camera`` / ``cam_setup`` pick / configure the render view
    (``cam_setup(model, cam)`` mutates a free camera before each frame)."""
    if rig is None:
        rig = Rig()
    preset = gait.PRESETS[gait_name]
    steps = int(seconds / rig.model.opt.timestep)
    ctrl_every = max(1, int(0.01 / rig.model.opt.timestep))   # 100 Hz control

    start_xy = rig.body_point("torso")[:2].copy()
    log = {"t": [], "z": [], "tilt": [], "tau": [], "cam_p": [], "cam_r": [],
           "roll": [], "pitch": [], "roll_t": [], "frames": [], "rgb": []}
    fell_at = None
    frame_every = max(1, steps // 90)
    settle_t = 1.0   # ignore the initial drop-onto-feet when scoring camera shake
    renderer = None
    free_cam = None
    if render:
        try:                                # real 3-D frames if a GL context exists
            renderer = mujoco.Renderer(rig.model, 420, 560)
            if cam_setup is not None:
                free_cam = mujoco.MjvCamera()
                mujoco.mjv_defaultCamera(free_cam)
        except Exception:
            renderer = None

    for i in range(steps):
        t = i * rig.model.opt.timestep
        if i % ctrl_every == 0:
            (control_fn or (lambda r, tt: gait_control(r, tt, preset)))(rig, t)
        mujoco.mj_step(rig.model, rig.data)

        if t > settle_t:
            cp, cr = rig.camera_angles()
            log["cam_p"].append(cp); log["cam_r"].append(cr)
            roll, pitch = rig.torso_roll_pitch()
            log["roll"].append(math.degrees(roll)); log["pitch"].append(math.degrees(pitch))
            log["roll_t"].append(t)
        if i % frame_every == 0:
            log["t"].append(t)
            log["z"].append(rig.torso_z())
            log["tilt"].append(math.degrees(rig.torso_tilt()))
            log["tau"].append(rig.peak_leg_torque())
            log["frames"].append(_skeleton(rig))
            if renderer is not None:
                if free_cam is not None:
                    cam_setup(rig, free_cam)
                    renderer.update_scene(rig.data, camera=free_cam)
                else:
                    renderer.update_scene(rig.data, camera=camera)
                log["rgb"].append(renderer.render().copy())
        if fell_at is None and (rig.torso_z() < 0.045 or rig.torso_tilt() > 1.05):
            fell_at = t

    travel = float(np.linalg.norm(rig.body_point("torso")[:2] - start_xy))
    return rig, log, fell_at, travel


def _skeleton(rig: Rig):
    """World points for a stick-figure frame: torso + each leg chain."""
    pts = {"torso": rig.body_point("torso")}
    for leg in P.LEGS:
        for j in ("abd", "hip", "knee", "ankle"):
            pts[f"{leg}_{j}"] = rig.body_point(f"{leg}_{j}")
        pts[f"{leg}_foot"] = rig.foot_point(leg)
    return pts


# ------------------------------------------------------------------ rendering
def render_reports(gait_name, log, fell_at, travel):
    os.makedirs(OUT, exist_ok=True)
    import matplotlib
    matplotlib.use("Agg")
    import matplotlib.pyplot as plt
    from matplotlib.animation import FuncAnimation, PillowWriter

    # (1) height + tilt + torque over time
    fig, ax = plt.subplots(3, 1, figsize=(7, 6), sharex=True)
    ax[0].plot(log["t"], np.array(log["z"]) * 1000, color="#0e9488"); ax[0].set_ylabel("torso z (mm)")
    ax[0].axhline(P.STANCE_H, ls="--", c="#888", lw=1)
    ax[1].plot(log["t"], log["tilt"], color="#d9702f"); ax[1].set_ylabel("tilt (°)")
    ax[2].plot(log["t"], log["tau"], color="#5b9bd5"); ax[2].set_ylabel("peak τ (N·m)")
    ax[2].axhline(SERVO.stall_nm, ls="--", c="#d1483f", lw=1, label="servo stall")
    ax[2].set_xlabel("time (s)"); ax[2].legend(loc="upper right", fontsize=8)
    if fell_at: [a.axvline(fell_at, c="#d1483f", lw=1) for a in ax]
    fig.suptitle(f"RoboKitten — {gait_name}  (travel {travel*100:.0f} cm)")
    fig.tight_layout(); fig.savefig(os.path.join(OUT, f"{gait_name}_telemetry.png"), dpi=110)
    plt.close(fig)

    # (2) motion animation — real 3-D frames if we captured them, else skeleton
    if log.get("rgb"):
        from PIL import Image
        imgs = [Image.fromarray(f) for f in log["rgb"]]
        gif = os.path.join(OUT, f"{gait_name}_motion.gif")
        imgs[0].save(gif, save_all=True, append_images=imgs[1:],
                     duration=int(1000 / 20), loop=0)
        return gif

    frames = log["frames"]
    fig = plt.figure(figsize=(6, 4)); axp = fig.add_subplot(111, projection="3d")
    conns = [("torso", f"{l}_abd") for l in P.LEGS]
    conns += [(f"{l}_abd", f"{l}_hip") for l in P.LEGS]
    conns += [(f"{l}_hip", f"{l}_knee") for l in P.LEGS]
    conns += [(f"{l}_knee", f"{l}_ankle") for l in P.LEGS]
    conns += [(f"{l}_ankle", f"{l}_foot") for l in P.LEGS]

    def draw(k):
        axp.clear(); axp.set_axis_off()
        pts = frames[k]
        c = pts["torso"]
        for a, b in conns:
            pa, pb = pts[a], pts[b]
            axp.plot([pa[0], pb[0]], [pa[1], pb[1]], [pa[2], pb[2]],
                     color="#cdd2dc", lw=3)
        axp.scatter([pts[f"{l}_foot"][0] for l in P.LEGS],
                    [pts[f"{l}_foot"][1] for l in P.LEGS],
                    [pts[f"{l}_foot"][2] for l in P.LEGS], color="#e8823c", s=18)
        # floor
        axp.plot([c[0]-0.2, c[0]+0.2], [c[1], c[1]], [0, 0], color="#3fd0c4", lw=1)
        axp.set_xlim(c[0]-0.25, c[0]+0.25); axp.set_ylim(c[1]-0.25, c[1]+0.25)
        axp.set_zlim(0, 0.25); axp.view_init(elev=8, azim=-80)

    anim = FuncAnimation(fig, draw, frames=len(frames), interval=60)
    gif = os.path.join(OUT, f"{gait_name}_motion.gif")
    anim.save(gif, writer=PillowWriter(fps=15)); plt.close(fig)
    return gif


# ------------------------------------------------------------------ main
def main():
    ap = argparse.ArgumentParser()
    ap.add_argument("--gait", choices=list(gait.PRESETS), default="trot")
    ap.add_argument("--seconds", type=float, default=6.0)
    ap.add_argument("--view", action="store_true", help="interactive 3-D viewer")
    ap.add_argument("--brain", action="store_true", help="Phase-0 brain drives the body")
    ap.add_argument("--cute", action="store_true", help="play the cute-motion gesture demo")
    ap.add_argument("--gesture", help="play/render a single cute gesture on loop (e.g. loaf)")
    ap.add_argument("--pounce", action="store_true",
                    help="dynamic cat pounce/leap (time-based JumpController) + jump metrics")
    ap.add_argument("--front", action="store_true",
                    help="render a front-facing (head-on) capture + roll-vs-time plot")
    args = ap.parse_args()

    if args.pounce:
        return run_pounce(args.seconds if args.seconds != 6.0 else 3.0)
    if args.front:
        return render_front(args.gait, args.seconds)
    if args.view:
        return run_view(args.gait, args.seconds, args.brain, args.cute)

    control = None
    if args.gesture:
        from sim.cute_motion import GESTURES, make_cute_control
        if args.gesture not in GESTURES:
            ap.error(f"unknown gesture {args.gesture!r}; choose from {list(GESTURES)}")
        control = make_cute_control([args.gesture], hold=True)   # settle into the pose + hold
    elif args.cute:
        from sim.cute_motion import make_cute_control
        control = make_cute_control()
    elif args.brain:
        from sim.brain_bridge import make_brain_control
        control = make_brain_control()

    label = args.gesture if args.gesture else ("cute" if args.cute else args.gait)
    rig, log, fell_at, travel = simulate(args.gait, args.seconds, control)
    peak = max(log["tau"]) if log["tau"] else 0.0
    print("=" * 60)
    print(f"MuJoCo emulation — {label}  {args.seconds:.0f}s"
          + ("  [brain-driven]" if args.brain else ""))
    print("=" * 60)
    if fell_at is None:
        print(f"[PASS] stayed UPRIGHT the whole run")
    else:
        print(f"[FAIL] fell at t={fell_at:.2f}s")
    print(f"  final torso height : {log['z'][-1]*1000:.0f} mm (stance {P.STANCE_H:.0f})")
    print(f"  max tilt           : {max(log['tilt']):.0f}°")
    print(f"  distance travelled : {travel*100:.0f} cm")
    print(f"  peak leg torque    : {peak:.2f} N·m  "
          f"({peak/SERVO.stall_nm*100:.0f}% of {SERVO.stall_nm:.2f} stall)")
    if log["cam_p"]:
        import numpy as _np
        pp = max(log["cam_p"]) - min(log["cam_p"])
        rr = max(log["cam_r"]) - min(log["cam_r"])
        rms = float(_np.sqrt(_np.mean(_np.square(log["cam_p"]) + _np.square(log["cam_r"]))))
        print(f"  CAMERA shake       : {pp:.1f}° pitch / {rr:.1f}° roll p-p  "
              f"(RMS {rms:.1f}°)")
    if log["roll"]:
        roll_pp = max(log["roll"]) - min(log["roll"])
        pitch_pp = max(log["pitch"]) - min(log["pitch"])
        print(f"  TORSO roll/pitch   : {roll_pp:.1f}° roll / {pitch_pp:.1f}° pitch p-p")
    gif = render_reports(label, log, fell_at, travel)
    msg = f"  renders            : {os.path.basename(gif)}, {label}_telemetry.png"
    if args.gesture and control is not None:               # held pose: also grab side+iso stills
        stills = render_stills(label, control, at_t=2.2)
        if stills:
            msg += ", " + ", ".join(os.path.basename(s) for s in stills)
    print(msg)


def _iso_cam(rig, cam):
    """Free camera in a 3/4 iso view, following the whole-robot CoM."""
    com = rig.data.subtree_com[1]
    cam.lookat[0], cam.lookat[1], cam.lookat[2] = float(com[0]), float(com[1]), float(com[2])
    cam.distance = 0.58
    cam.azimuth = -125.0
    cam.elevation = -18.0


def _side_cam(rig, cam):
    """Free camera in a side (profile) view, following the CoM — shows the leg
    fold + body pitch of a held pose most clearly."""
    com = rig.data.subtree_com[1]
    cam.lookat[0], cam.lookat[1], cam.lookat[2] = float(com[0]), float(com[1]), float(com[2])
    cam.distance = 0.52
    cam.azimuth = 90.0
    cam.elevation = -6.0


def render_stills(label, control_fn, at_t=2.2):
    """Settle a pose/motion under ``control_fn`` for ``at_t`` s, then capture a side
    + iso PNG still (each following the CoM) to ``sim/out/{label}_still_{view}.png``.
    Reuses the CoM-following free-camera approach of the gait front renders."""
    os.makedirs(OUT, exist_ok=True)
    rig = Rig()
    steps = int(at_t / rig.model.opt.timestep)
    ctrl_every = max(1, int(0.01 / rig.model.opt.timestep))
    for i in range(steps):
        t = i * rig.model.opt.timestep
        if i % ctrl_every == 0:
            control_fn(rig, t)
        mujoco.mj_step(rig.model, rig.data)
    try:
        renderer = mujoco.Renderer(rig.model, 560, 720)
    except Exception:
        return []
    from PIL import Image
    out = []
    for view, setup in (("side", _side_cam), ("iso", _iso_cam)):
        cam = mujoco.MjvCamera(); mujoco.mjv_defaultCamera(cam)
        setup(rig, cam)
        renderer.update_scene(rig.data, camera=cam)
        p = os.path.join(OUT, f"{label}_still_{view}.png")
        Image.fromarray(renderer.render()).save(p)
        out.append(p)
    return out


def run_pounce(seconds=3.0):
    """Dynamic cat POUNCE/leap (time-based JumpController): report the honest jump
    metrics (CoM rise, forward lunge, air-time, peak torque, upright/fell) and write
    a CoM-following gif + side/iso stills (still captured at the launch apex)."""
    from sim.cute_motion import make_jump_control
    rig = Rig()
    ctrl = make_jump_control()
    dt = rig.model.opt.timestep
    ctrl_every = max(1, int(0.01 / dt))
    sx = rig.body_point("torso")[0]
    crouch_z = apex_z = peak_tau = airtime = maxtilt = 0.0
    apex_t = 0.0
    fell = None
    log = {"t": [], "z": [], "tilt": [], "tau": [], "rgb": [], "cam_p": [], "cam_r": [], "roll": [], "pitch": []}
    frame_every = max(1, int(seconds / dt) // 90)
    try:
        renderer = mujoco.Renderer(rig.model, 420, 560)
    except Exception:
        renderer = None
    for i in range(int(seconds / dt)):
        t = i * dt
        if i % ctrl_every == 0:
            ctrl(rig, t)
        mujoco.mj_step(rig.model, rig.data)
        z = rig.torso_z()
        if abs(t - 0.98) < dt:
            crouch_z = z
        if z > apex_z:
            apex_z, apex_t = z, t
        if min(rig.foot_point(l)[2] for l in P.LEGS) > 0.013:
            airtime += dt
        peak_tau = max(peak_tau, rig.peak_leg_torque())
        maxtilt = max(maxtilt, math.degrees(rig.torso_tilt()))
        if fell is None and (z < 0.045 or rig.torso_tilt() > 1.2):
            fell = t
        if i % frame_every == 0:
            log["t"].append(t); log["z"].append(z)
            log["tilt"].append(math.degrees(rig.torso_tilt())); log["tau"].append(rig.peak_leg_torque())
            if renderer is not None:
                renderer.update_scene(rig.data, camera="cam")
                log["rgb"].append(renderer.render().copy())
    lunge = (rig.body_point("torso")[0] - sx) * 1000

    print("=" * 60)
    print(f"MuJoCo emulation — pounce  {seconds:.0f}s  (time-based JumpController)")
    print("=" * 60)
    print(f"[{'PASS' if fell is None else 'FAIL'}] "
          + ("stayed UPRIGHT the whole leap" if fell is None else f"fell at t={fell:.2f}s"))
    print(f"  CoM rise (crouch->apex): {(apex_z - crouch_z)*1000:.0f} mm  "
          f"(crouch {crouch_z*1000:.0f} -> apex {apex_z*1000:.0f} mm)")
    print(f"  forward lunge          : {lunge:.0f} mm")
    print(f"  air-time (feet off)    : {airtime:.3f} s  "
          + ("(GROUNDED loaded-spring pounce)" if airtime < 0.02 else "(airborne)"))
    print(f"  max tilt during leap   : {maxtilt:.0f}°   final tilt {math.degrees(rig.torso_tilt()):.0f}°")
    print(f"  peak leg torque        : {peak_tau:.2f} N·m ({peak_tau/SERVO.stall_nm*100:.0f}% of stall; "
          f"the launch burst intentionally bypasses the slew limiter)")
    gif = render_reports("pounce", log, fell, abs(lunge)/1000.0)
    stills = render_stills("pounce", make_jump_control(), at_t=apex_t if apex_t > 0 else 1.15)
    print(f"  renders                : {os.path.basename(gif)}, pounce_telemetry.png, "
          + ", ".join(os.path.basename(s) for s in stills))
    return fell is None


def _front_cam(rig, cam):
    """Free camera looking at the robot HEAD-ON, following the CoM. This is the
    view that shows lateral waddle (torso roll) most clearly."""
    com = rig.data.subtree_com[1]           # whole-robot CoM (root subtree)
    cam.lookat[0], cam.lookat[1], cam.lookat[2] = float(com[0]), float(com[1]), float(com[2])
    cam.distance = 0.5
    cam.azimuth = 180.0
    cam.elevation = -5.0


def render_front(gait_name="walk", seconds=8.0):
    """Render the gait from a front-facing, CoM-following camera to
    ``walk_front.gif`` and plot torso roll vs time to ``walk_roll.png``."""
    os.makedirs(OUT, exist_ok=True)
    rig, log, fell_at, travel = simulate(gait_name, seconds, cam_setup=_front_cam)
    roll_pp = (max(log["roll"]) - min(log["roll"])) if log["roll"] else 0.0

    gif = os.path.join(OUT, f"{gait_name}_front.gif")
    if log.get("rgb"):
        from PIL import Image
        imgs = [Image.fromarray(f) for f in log["rgb"]]
        imgs[0].save(gif, save_all=True, append_images=imgs[1:],
                     duration=int(1000 / 20), loop=0)

    import matplotlib
    matplotlib.use("Agg")
    import matplotlib.pyplot as plt
    fig, ax = plt.subplots(figsize=(7, 3))
    ax.plot(log["roll_t"], log["roll"], color="#0e9488", lw=1.4, label="torso roll")
    ax.plot(log["roll_t"], log["pitch"], color="#d9702f", lw=1.0, alpha=0.7, label="pitch")
    ax.axhline(0, ls="--", c="#888", lw=0.8)
    ax.set_xlabel("time (s)"); ax.set_ylabel("angle (°)")
    ax.set_title(f"{gait_name} torso roll  (roll p-p = {roll_pp:.1f}°)")
    ax.legend(loc="upper right", fontsize=8)
    plot = os.path.join(OUT, f"{gait_name}_roll.png")
    fig.tight_layout(); fig.savefig(plot, dpi=120); plt.close(fig)

    print(f"[{'PASS' if fell_at is None else 'FAIL'}] {gait_name} front-cam "
          f"roll p-p {roll_pp:.1f}°, travel {travel*100:.0f} cm")
    print(f"  renders: {os.path.basename(gif)}, {os.path.basename(plot)}")
    return gif, plot


def run_view(gait_name, seconds, brain, cute=False):
    import mujoco.viewer
    rig = Rig()
    preset = gait.PRESETS[gait_name]
    control = None
    if cute:
        from sim.cute_motion import make_cute_control
        control = make_cute_control()
    elif brain:
        from sim.brain_bridge import make_brain_control
        control = make_brain_control()
    with mujoco.viewer.launch_passive(rig.model, rig.data) as v:
        import time
        t0 = time.perf_counter()
        while v.is_running() and (time.perf_counter() - t0) < seconds:
            t = time.perf_counter() - t0
            (control or (lambda r, tt: gait_control(r, tt, preset)))(rig, t)
            mujoco.mj_step(rig.model, rig.data)
            v.sync()


if __name__ == "__main__":
    main()
