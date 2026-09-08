"""Servo bus map + STS3215 calibration math.

``hardware/servo_channel_map.py`` is pure data + math (no driver imports), and it is
the single source of truth for *which serial ID each joint answers to* and *how a
radian becomes a position count*. Both are unrecoverable-at-runtime kinds of wrong:
a duplicate bus ID is a silent chain collision, and a bad angle map drives a joint
into a mechanical stop. It also claims to mirror ``cad/params.py`` joint limits —
that claim is only true if something checks it.
"""

import math

import pytest

from cad import params as P
from hardware import servo_channel_map as scm


# --------------------------------------------------------------- the bus map
def test_every_servo_has_a_unique_addressable_bus_id():
    scm.assert_no_collisions()          # raises on a duplicate / out-of-range ID
    ids = [sc.servo_id for sc in scm.SERVOS.values()]
    assert len(set(ids)) == len(ids)
    assert all(scm.ID_MIN <= i <= scm.ID_MAX for i in ids)


def test_map_covers_exactly_the_designed_servo_count():
    """Every actuator in cad/params.py must be one address on the chain, and no more.

    The count is read from params rather than written here, so dropping or adding a joint
    cannot leave a phantom servo on the bus — which is what happened to the ears when they
    became rigid (`params.EARS_ACTUATED`)."""
    assert len(scm.SERVOS) == P.N_SERVOS
    assert len(scm.all_channels()) == P.N_SERVOS


def test_rigid_joints_have_no_servo_on_the_bus():
    """A joint the design does not drive must not be addressable."""
    if not P.EARS_ACTUATED:
        assert "ear_L" not in scm.SERVOS and "ear_R" not in scm.SERVOS


def test_a_duplicate_bus_id_is_rejected():
    dup = dict(scm.SERVOS)
    dup["clone"] = scm.ServoBusChannel("clone", scm.SERVOS["tail"].servo_id, -1.0, 1.0)
    with pytest.MonkeyPatch.context() as mp:
        mp.setattr(scm, "SERVOS", dup)
        with pytest.raises(ValueError, match="used by both"):
            scm.assert_no_collisions()


def test_leg_groups_reference_real_channels():
    for leg, joints in scm.LEG_JOINTS.items():
        assert leg in P.LEGS
        for j in joints:
            assert j in scm.SERVOS


# --------------------------------------------------------------- angle <-> count
@pytest.mark.parametrize("name", sorted(scm.SERVOS))
def test_angle_to_pos_round_trips_inside_the_soft_limits(name):
    sc = scm.SERVOS[name]
    for frac in (0.0, 0.25, 0.5, 0.75, 1.0):
        angle = sc.lo_rad + frac * (sc.hi_rad - sc.lo_rad)
        back = sc.pos_to_angle(sc.angle_to_pos(angle))
        # one count is the quantisation floor (~1/652 rad); allow two.
        assert back == pytest.approx(angle, abs=2 / sc.counts_per_rad)


@pytest.mark.parametrize("name", sorted(scm.SERVOS))
def test_neutral_is_the_servo_centre_count(name):
    sc = scm.SERVOS[name]
    if sc.lo_rad <= 0.0 <= sc.hi_rad:
        assert sc.angle_to_pos(0.0) == sc.center


@pytest.mark.parametrize("name", sorted(scm.SERVOS))
def test_commands_past_the_soft_limit_are_clamped_not_wrapped(name):
    """A runaway command must saturate at the soft limit — never fold past the
    mechanical stop or wrap around the 12-bit count."""
    sc = scm.SERVOS[name]
    assert sc.angle_to_pos(sc.hi_rad + 5.0) == sc.limit_hi_pos
    assert sc.angle_to_pos(sc.lo_rad - 5.0) == sc.limit_lo_pos
    for a in (-50.0, -1.0, 0.0, 1.0, 50.0, math.pi, -math.pi):
        assert scm.POS_MIN <= sc.angle_to_pos(a) <= scm.POS_MAX


def test_inverted_servos_mirror_their_twin():
    """The right-side horns are mounted mirror-imaged; the same joint angle must
    land the same distance from centre, on the opposite side."""
    for left, right in (("FL_hip", "FR_hip"), ("FL_knee", "FR_knee"),
                        ("RL_hip", "RR_hip"), ("RL_knee", "RR_knee")):
        l, r = scm.SERVOS[left], scm.SERVOS[right]
        assert not l.invert and r.invert
        a = 0.5 * (l.hi_rad - max(l.lo_rad, 0.0))
        assert (l.angle_to_pos(a) - l.center) == -(r.angle_to_pos(a) - r.center)


# --------------------------------------------------------------- params mirroring
@pytest.mark.parametrize("scm_lim, params_lim, label", [
    (scm.LIM_HIP, P.LIM_HIP, "hip"),
    (scm.LIM_KNEE, P.LIM_KNEE, "knee (generic)"),
    (scm.LIM_KNEE_FRONT, P.leg_knee_limit("FL"), "knee (front four-bar reach)"),
    (scm.LIM_KNEE_REAR, P.leg_knee_limit("RL"), "knee (rear four-bar reach)"),
    (scm.LIM_WAIST, P.LIM_WAIST, "waist"),
    (scm.LIM_HEAD_PITCH, P.LIM_HEAD_PITCH, "head pitch"),
])
def test_soft_limits_mirror_cad_params(scm_lim, params_lim, label):
    """servo_channel_map.py promises it mirrors cad/params.py. If these drift, the
    brain can command a joint the mechanism cannot reach."""
    assert tuple(scm_lim) == tuple(params_lim), (
        f"{label}: servo map {tuple(scm_lim)} != cad/params {tuple(params_lim)}"
    )


def test_per_leg_knee_channels_use_that_leg_s_four_bar_reach():
    for leg in P.LEGS:
        knee = scm.SERVOS[f"{leg}_knee"]
        lo, hi = P.leg_knee_limit(leg)
        assert (knee.lo_rad, knee.hi_rad) == (lo, hi), leg


def test_head_roll_has_no_servo_when_it_is_not_a_joint():
    """The gimbal is 2-axis (params.HEAD_ROLL_ACTUATED). Roll is corrected electronically,
    so there must be no roll servo left addressable on the chain."""
    if not P.HEAD_ROLL_ACTUATED:
        assert "head_tilt" not in scm.SERVOS
        assert not hasattr(P, "LIM_HEAD_TILT")


def test_led_eyes_are_not_on_the_servo_bus():
    """An STS3215 chain has no PWM output; the eyes live on a Jetson PWM pin."""
    assert scm.LED_EYE.name not in scm.SERVOS
    assert scm.LED_EYE.name not in scm.all_channels().values()
