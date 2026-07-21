"""
Cute-motion library — what makes Sabo read as an adorable cat, not a machine.
=============================================================================

Two layers, applied on top of the standing pose:

* **Idle liveliness** — always-on micro-motion (breathing, slow tail sway, ear
  twitch, tiny head drift). A cat is never perfectly still; a frozen robot reads
  as "off". This runs under everything.
* **Signature gestures** — timed, eased keyframe animations of the cat motion
  vocabulary (PLAN §4.2), using the waist + head-tilt joints we just added:
  head-tilt, big stretch, loaf, rear-wiggle pounce, halloween arch, sit-pretty.

Animation principle baked in: **anticipation → burst → settle** with easing and a
little overshoot, never constant-velocity.

A gesture is a function ``u∈[0,1] -> targets`` where ``targets`` may set any of:
    front_depth, rear_depth   (fraction of stance depth; <1 = that end lower)
    waist                      (rad; - belly-down/extend, + arch up)
    head_tilt, head_pan        (rad)
    ear                        (rad; + forward/perk, - back)
    tail                       (rad; + up)
Missing keys fall back to neutral. Targets are hardware-independent — the same
library will drive the real servos.
"""

from __future__ import annotations

import math

from cad import params as P
from sim import gait

NEUTRAL = dict(front_depth=1.0, rear_depth=1.0, waist=0.0,
               head_tilt=0.0, head_pan=0.0, head_pitch=0.0, ear=0.0, tail=0.15,
               front_tuck=0.0, rear_tuck=0.0)   # foot x-offset (mm; + = forward)


# ---------------------------------------------------------------- easing
def ease(u):                     # smoothstep
    u = max(0.0, min(1.0, u))
    return u * u * (3 - 2 * u)


def ease_back(u):                # overshoot-and-settle (springy)
    u = max(0.0, min(1.0, u))
    c = 1.9
    return 1 + (c + 1) * (u - 1) ** 3 + c * (u - 1) ** 2


def _pulse(u, up=0.3, down=0.7):
    """0 -> 1 (by ``up``) -> hold -> 0 (after ``down``) — go, hold, return."""
    if u < up:
        return ease(u / up)
    if u > down:
        return 1 - ease((u - down) / (1 - down))
    return 1.0


# ---------------------------------------------------------------- gestures
def g_head_tilt(u):
    a = _pulse(u, 0.35, 0.6)
    return dict(head_tilt=0.55 * a, ear=0.4 * a, head_pan=0.15 * a)


def g_stretch(u):
    """Big downward-dog stretch: front paws forward+low, haunches high, spine long."""
    a = _pulse(u, 0.4, 0.65)
    return dict(front_depth=1.0 - 0.38 * a, rear_depth=1.0 + 0.10 * a,
                waist=-0.18 * a, tail=0.15 + 0.5 * a, ear=0.2 * a)


def g_loaf(u):
    """Settle into a compact cat LOAF (식빵): body LOW + level, all four paws tucked
    IN under the belly, back gently rounded, tail wrapped, ears forward.

    Deepened now that the per-leg knee caps opened up (FRONT 150° / REAR 160°): the
    fold goes to front≈0.46 stance (knee ≈147°, 3° under the 150° cap) and rear≈0.42
    (knee ≈155°, 5° under the 160° cap). The rear folds a touch more than the front —
    cat-correct — and the two depths are chosen so the front/rear HIP heights match
    (≈56/58 mm) and the torso stays level. About 8 mm lower + flatter than the old
    loaf (which stopped at the 149° cap). Belly-flat-on-floor is still out of reach
    (would need knee past the cap), so this is the lowest level loaf the folds allow."""
    a = _pulse(u, 0.35, 0.8)
    return dict(front_depth=1.0 - 0.50 * a,       # ≈38 mm underside (knee ≈146°, 4° margin)
                rear_depth=1.0 - 0.58 * a,        # ≈34 mm (knee ≈155°) → body sits level
                front_tuck=-10.0 * a,             # front paws slide back under the chest
                rear_tuck=+10.0 * a,              # rear paws slide forward under the hips
                waist=0.12 * a,                   # gently rounded back
                tail=0.03, ear=0.18 * a, head_tilt=0.05 * a)


def g_wiggle_pounce(u):
    """Crouch, wiggle the rear (anticipation), then spring — the cutest thing a cat does."""
    if u < 0.55:                                   # crouch + rear wiggle
        a = ease(u / 0.55)
        wig = 0.12 * math.sin(u * 60) * a
        return dict(front_depth=1.0 - 0.35 * a, rear_depth=1.0 - 0.4 * a,
                    waist=-0.1 * a + wig, head_pan=wig * 1.5, ear=0.45,
                    tail=0.3 + 0.4 * a)
    b = ease_back((u - 0.55) / 0.45)               # spring + settle
    return dict(front_depth=1.0 + 0.06 * b, rear_depth=1.0 + 0.03 * b,
                waist=0.08 * b, ear=0.45, tail=0.6)


def g_arch(u):
    """Halloween arch: spine up, ears back, tail up, then relax."""
    a = _pulse(u, 0.35, 0.65)
    return dict(waist=0.32 * a, ear=-0.5 * a, tail=0.7 * a,
                rear_depth=1.0 - 0.04 * a)


def g_sit(u):
    """Proper cat SIT (앉기): haunches DOWN, chest UP, head UP — a stable feline sit.

    The key to not faceplanting (a forward-heavy, no-abduction quadruped tips over a
    deep fold otherwise): the rear paws plant slightly BEHIND the hip (rear_tuck < 0)
    so the hind legs form a rearward support strut, the FRONT legs extend TALL and
    plant forward to lift the shoulders, and a spine ARCH (waist) drops the rump. The
    head is pitched UP so its mass doesn't lever the body forward. Result (settled,
    verified): shoulders ≈109 mm vs rump ≈76 mm — a clear rump-down / chest-up sit —
    torso tilt ~3° (stable), rear knee within the 160° cap. Empirically swept for the
    deepest rump-drop that still holds without tipping."""
    a = _pulse(u, 0.3, 0.8)
    return dict(front_depth=1.0 + 0.18 * a,       # TALL front — lifts the shoulders/chest
                rear_depth=1.0 - 0.55 * a,        # deep haunch fold, rump drops
                front_tuck=+18.0 * a,             # front paws planted forward (hold CoM over base)
                rear_tuck=-20.0 * a,              # rear paws BEHIND the hip — rearward support strut
                waist=0.35 * a,                   # spine arch — sits the rump down
                head_pitch=-0.35 * a,             # head UP (don't let the heavy head tip it forward)
                head_tilt=0.06 * a, ear=0.42 * a, tail=0.10)


def g_sit_pretty(u):
    """Alias kept for the demo sequence — the proper cat sit (앉기)."""
    return g_sit(u)


GESTURES = {
    "head_tilt": (g_head_tilt, 2.2),
    "stretch": (g_stretch, 3.2),
    "loaf": (g_loaf, 2.8),
    "wiggle_pounce": (g_wiggle_pounce, 2.4),
    "arch": (g_arch, 2.6),
    "sit": (g_sit, 2.6),
    "sit_pretty": (g_sit_pretty, 2.6),
}
DEMO_SEQUENCE = ["head_tilt", "stretch", "loaf", "wiggle_pounce", "arch", "sit_pretty"]


# ---------------------------------------------------------------- idle liveliness
def idle(t):
    """Always-on micro-motion so Sabo reads as alive even when 'still'."""
    return dict(
        waist=0.025 * math.sin(2 * math.pi * t / 3.0),      # breathing
        rear_depth=1.0 + 0.015 * math.sin(2 * math.pi * t / 3.0),
        tail=0.15 + 0.13 * math.sin(2 * math.pi * t / 4.2),  # slow sway
        head_pan=0.10 * math.sin(2 * math.pi * t / 6.5),     # idle look-around
        ear=0.06 * math.sin(2 * math.pi * t / 5.0),          # ear micro-twitch
    )


def _blend(base, over):
    out = dict(base)
    for k, v in over.items():
        if k in ("front_depth", "rear_depth"):
            out[k] = out.get(k, 1.0) * v          # depths multiply
        else:
            out[k] = out.get(k, 0.0) + v          # angles add
    return out


class CuteController:
    """control_fn(rig, t): plays the gesture sequence with idle liveliness under it.

    ``hold=True`` plays a SINGLE pose gesture as a settle-and-hold (ramp in over
    ``hold_ramp`` s, then stay) instead of the sequence's pulse-in/pulse-out — so a
    pose like the loaf is *held* rather than repeatedly entered and released (which
    otherwise reads as bobbing/marching in place)."""

    def __init__(self, sequence=None, loop=True, hold=False, hold_ramp=1.2):
        self.seq = sequence or DEMO_SEQUENCE
        self.loop = loop
        self.hold = hold
        self.hold_ramp = hold_ramp
        self._total = sum(GESTURES[g][1] for g in self.seq)

    def current(self, t):
        if self.hold:                              # ramp into the pose and stay there
            name = self.seq[0]
            fn = GESTURES[name][0]
            frac = min(1.0, t / self.hold_ramp)
            return name, fn(0.5 * frac)            # u=0.5 lands on the _pulse plateau (full pose)
        tt = t % self._total if self.loop else min(t, self._total - 1e-3)
        for name in self.seq:
            fn, dur = GESTURES[name]
            if tt < dur:
                return name, fn(tt / dur)
            tt -= dur
        return self.seq[-1], GESTURES[self.seq[-1]][0](1.0)

    def __call__(self, rig, t):
        name, gt = self.current(t)
        tgt = _blend(_blend(dict(NEUTRAL), idle(t)), gt)
        apply_targets(rig, tgt)
        return name


def apply_targets(rig, tgt):
    """Drive the rig's motorized joints from a resolved targets dict (front/rear
    depth-fraction + foot x-tuck -> per-leg hip/knee via IK, plus waist/head/ear/
    tail). Shared by the gesture controller and the jump controller so both speak
    the same hardware-independent pose vocabulary."""
    for leg in P.LEGS:
        front = leg in P.FRONT_LEGS
        scale = tgt["front_depth"] if front else tgt["rear_depth"]
        tuck = tgt["front_tuck"] if front else tgt["rear_tuck"]
        hip, knee = gait.leg_ik(leg, tuck, gait.leg_depth(leg) * scale)
        rig.set_target(f"{leg}_hip", hip)
        rig.set_target(f"{leg}_knee", knee)
    rig.set_target("torso_aft", tgt["waist"])
    rig.set_target("head_tilt", tgt["head_tilt"])
    rig.set_target("head_pan", tgt["head_pan"])
    rig.set_target("head_pitch", tgt["head_pitch"])
    rig.set_target("ear_L", tgt["ear"])
    rig.set_target("tail", tgt["tail"])


def make_cute_control(sequence=None, hold=False):
    return CuteController(sequence, hold=hold)


# ---------------------------------------------------------------- jump / pounce
def _lerp_targets(a: dict, b: dict, s: float) -> dict:
    """Ease between two full targets dicts (s in [0,1]); depths/tucks/angles all
    interpolate scalar-wise."""
    s = ease(s)
    keys = set(a) | set(b)
    out = {}
    for k in keys:
        d = 1.0 if k in ("front_depth", "rear_depth") else 0.0
        out[k] = a.get(k, d) + s * (b.get(k, d) - a.get(k, d))
    return out


class JumpController:
    """Time-based cat POUNCE (점프) — a real dynamic motion, not a held pose.

    Four phases: (a) settle into a LOADED crouch (deep fold, all four, a tiny rear
    wiggle of anticipation, head/ears aimed forward); (b) an EXPLOSIVE all-leg
    extension that drives the feet down-and-back to launch the body up-and-forward;
    (c) flight, front legs reaching out to catch the landing, tail up for balance;
    (d) landing, re-flexing the legs to ABSORB the impact, then settling to stand.

    The Rig slew-rate limiter (which keeps the position servo off its stall during
    gaits) would blunt the launch, so this controller does a clean PER-MODE slew
    override: it bypasses the limiter for the launch burst (so the servos can snap
    to full extension = maximum push), raises it moderately through flight/land so
    the legs can reposition, then restores the default for the settle. This is the
    honest ceiling of what these servos can do — see mj_emulate for the measured
    hop height / air-time.

    ``forward`` biases the push rearward (feet drive back) so the launch projects
    the CoM FORWARD as well as up — cats pounce forward far more than they hop
    straight up, and forward is also the more energetic/robust direction here."""

    def __init__(self, forward: float = 1.0, base_slew: float | None = None):
        self.fwd = forward
        self.base_slew = base_slew   # None -> captured from the rig on first call
        # phase boundaries (s)
        self.t_crouch = 1.0       # settle into the loaded crouch by here
        self.t_launch = 1.15      # explosive extension ends here (~0.15 s burst)
        self.t_reach = 1.35       # flight: reach the front legs out to land
        self.t_land = 1.65        # begin absorbing (re-flex)
        self.t_settle = 2.6       # back to a relaxed stand

    # -- keyframe poses (targets dicts) --
    def _stand(self):
        return dict(NEUTRAL)

    def _crouch(self):
        return dict(NEUTRAL, front_depth=0.58, rear_depth=0.52,
                    front_tuck=-4.0, rear_tuck=+6.0, waist=-0.06,
                    head_tilt=0.0, ear=0.45, tail=0.30)

    def _extend(self):
        # explosive spring: front legs push HARDEST (near full stretch) to hold the
        # nose up, rear drives the feet BACK for a forward lunge. This front-biased
        # balance is what keeps the launch upright — a rear-biased or symmetric push
        # of the same magnitude leaves the ground but over-rotates and tumbles (there
        # is no in-air attitude control on this sagittal, coupled-ankle leg set).
        return dict(NEUTRAL, front_depth=1.45, rear_depth=1.25,
                    front_tuck=-20.0 * self.fwd, rear_tuck=-40.0 * self.fwd,
                    waist=0.10, ear=0.4, tail=0.55)

    def _reach(self):
        # apex: ease off the spring toward a symmetric, gently-flexed carriage — the
        # feet stay in ground contact through the leap, so this repositioning is safe.
        return dict(NEUTRAL, front_depth=1.12, rear_depth=1.08,
                    front_tuck=+6.0, rear_tuck=0.0, waist=0.0,
                    ear=0.4, tail=0.55)

    def _absorb(self):
        # land: deep flex to soak the impact (spring), all four planted
        return dict(NEUTRAL, front_depth=0.66, rear_depth=0.60,
                    front_tuck=0.0, rear_tuck=+4.0, waist=0.04,
                    ear=0.25, tail=0.25)

    def __call__(self, rig, t):
        if self.base_slew is None:                  # capture the rig's default once
            self.base_slew = rig.slew_rate
        if t < self.t_crouch:                       # (a) load the spring
            rig.slew_rate = self.base_slew
            wig = 0.10 * math.sin(t * 42) * ease(min(1.0, t / self.t_crouch))
            tgt = _lerp_targets(self._stand(), self._crouch(), t / self.t_crouch)
            tgt["head_pan"] = wig                   # anticipation wiggle
            phase = "crouch"
        elif t < self.t_launch:                     # (b) EXPLODE — bypass the slew limiter
            rig.slew_rate = None
            tgt = self._extend()
            phase = "launch"
        elif t < self.t_reach:                       # (c) flight — reach to land
            rig.slew_rate = 40.0                    # fast reposition, still bounded
            tgt = _lerp_targets(self._extend(), self._reach(),
                                (t - self.t_launch) / (self.t_reach - self.t_launch))
            phase = "flight"
        elif t < self.t_land:                        # (c') still airborne, holding the reach
            rig.slew_rate = 40.0
            tgt = self._reach()
            phase = "flight"
        elif t < self.t_settle:                      # (d) land + absorb, then stand
            rig.slew_rate = 25.0
            s = (t - self.t_land) / (self.t_settle - self.t_land)
            tgt = _lerp_targets(self._absorb(), self._stand(), max(0.0, (s - 0.35) / 0.65)) \
                if s > 0.35 else self._absorb()
            phase = "land"
        else:
            rig.slew_rate = self.base_slew
            tgt = self._stand()
            phase = "stand"
        apply_targets(rig, tgt)
        return phase


def make_jump_control(forward: float = 1.0):
    return JumpController(forward=forward)
