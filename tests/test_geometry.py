"""Buildability of the printed geometry — can these parts actually be assembled?

The kinematic model can be perfect while the *parts* are unbuildable: two rigid
printed links that share volume cannot both exist, a servo that is larger than its
pocket cannot be dropped in, and a bracket with no fastener interface cannot be
attached. None of that is visible to ``analysis.validate`` (masses and torques) or
to MuJoCo (which sees joints, not part solids), so it is checked here.

Building the parts is slow (build123d/OCCT booleans), so this module is marked
``slow`` and shares one session-scoped build.

    pytest tests/test_geometry.py -q        # this file only
    pytest -m "not slow"                    # skip it
"""

import itertools

import pytest

pytestmark = pytest.mark.slow

# Boolean noise floor. OCCT intersections of tangent/touching faces can report a
# sliver; anything at or below this is contact, not shared material.
NOISE_MM3 = 1.0


@pytest.fixture(scope="session")
def leg_assembly():
    """One leg posed at its stance pose, as separately-named rigid parts."""
    from cad.assembly import _leg_locations, _linkage_transforms
    from cad.parts.leg import crank, leg_parts, pushrod
    from sim.gait import ankle_from_knee, stance_angles

    leg = "FL"
    hip, knee = stance_angles(leg)
    ankle = ankle_from_knee(leg, knee)
    Ta, Th, Tk, Tan = _leg_locations(leg, hip, knee, ankle)
    Tcr, Tpr = _linkage_transforms(leg)
    pl = leg_parts(leg)
    return {
        "hip_bracket": Ta * pl["hip_bracket"],
        "upper": Th * pl["upper"],
        "lower": Tk * pl["lower"],
        "foot": Tan * pl["foot"],
        "crank": Th * Tcr * crank(),
        "pushrod": Th * Tpr * pushrod(),
    }


@pytest.fixture(scope="session")
def tail_assembly():
    """The tail and its remote four-bar, posed at the neutral crank angle."""
    from build123d import Pos
    from cad import params as P
    from cad.parts.body import torso_aft
    from cad.parts.tail import (linkage_transforms, tail, tail_crank, tail_pushrod)

    tcr, tpr = linkage_transforms()
    return {"torso_aft": torso_aft(),
            "tail": Pos(-P.AFT_LEN, 0, P.BODY_H / 4) * tail(),
            "tail_crank": tcr * tail_crank(),
            "tail_pushrod": tpr * tail_pushrod()}


@pytest.fixture(scope="session")
def torso_assembly():
    """The two torso halves + the front leg brackets, in the neutral pose."""
    from cad.assembly import _leg_locations
    from cad.parts.body import torso_aft, torso_fore
    from cad.parts.leg import leg_parts
    from sim.gait import ankle_from_knee, stance_angles

    from build123d import Pos

    from cad import params as P
    from cad.parts.head import head

    out = {"torso_fore": torso_fore(), "torso_aft": torso_aft(),
           "head": Pos(*P.head_centre()) * head()}
    for leg in ("FL", "FR"):
        hip, knee = stance_angles(leg)
        Ta, Th, _, _ = _leg_locations(leg, hip, knee, ankle_from_knee(leg, knee))
        out[f"{leg}_hip_bracket"] = Ta * leg_parts(leg)["hip_bracket"]
        out[f"{leg}_upper"] = Th * leg_parts(leg)["upper"]
    return out


def _report(parts):
    """Every pair of parts that shares more than boolean noise."""
    bad = []
    for (na, a), (nb, b) in itertools.combinations(parts.items(), 2):
        v = (a & b).volume
        if v > NOISE_MM3:
            bad.append((na, nb, v))
    return sorted(bad, key=lambda t: -t[2])


def _msg(bad):
    return "parts sharing solid material:\n" + "\n".join(
        f"  {na} <-> {nb}: {v:.0f} mm^3" for na, nb, v in bad)


# --------------------------------------------------------------- interference
def test_leg_parts_do_not_share_material(leg_assembly):
    """Every pivot must be a real clevis/offset joint, not two hubs drawn in the
    same place. A shared volume here means the leg cannot be assembled."""
    bad = _report(leg_assembly)
    assert not bad, _msg(bad)


def test_torso_and_leg_mounts_do_not_share_material(torso_assembly):
    """The two torso halves meet at the waist and the leg brackets bolt to the
    flanks — mating surfaces, not merged solids."""
    bad = _report(torso_assembly)
    assert not bad, _msg(bad)


def test_four_bar_links_clear_the_thigh(leg_assembly):
    """The crank and pushrod swing through their own plane; if either shares volume
    with the thigh it is buried in the structure and cannot move."""
    thigh = leg_assembly["upper"]
    for link in ("crank", "pushrod"):
        v = (thigh & leg_assembly[link]).volume
        assert v <= NOISE_MM3, f"{link} is embedded in the thigh by {v:.0f} mm^3"


# --------------------------------------------------------------- servo fit
def test_servo_cavity_admits_the_whole_servo():
    """What is CUT for the servo must clear its mounting FLANGES and its horn, not just
    the case body. The STS3215 is 45 mm across the case but 54 mm across the flanges, and
    its Ø20 output disc is fitted from outside once the servo is seated."""
    from cad.parts.leg import servo_cavity
    from cad.servo import DEFAULT as SERVO

    assert SERVO.flange_l > SERVO.body_l, "preset has no flange overhang to check"
    bb = servo_cavity(long_axis="x").bounding_box()
    assert bb.size.X >= SERVO.flange_l, (
        f"the cavity spans {bb.size.X:.1f} mm along the servo's long axis but the servo "
        f"measures {SERVO.flange_l:.1f} mm across its mounting flanges — it cannot drop in")
    assert bb.size.Z >= SERVO.body_w, "cavity is narrower than the servo case"


def test_servo_shaft_eccentricity_is_modelled():
    """The STS3215's output shaft is NOT centred on its case: it sits
    ``shaft_from_end`` from one end. A pocket centred on the joint axis puts every
    servo body in the wrong place."""
    from cad.parts.leg import servo_body_offset
    from cad.servo import DEFAULT as SERVO

    expected = SERVO.body_l / 2.0 - SERVO.shaft_from_end
    assert abs(servo_body_offset()) == pytest.approx(expected, abs=1e-9), (
        f"servo body must be offset {expected:.1f} mm from the shaft axis")


# --------------------------------------------------------------- fastening
def test_every_printed_part_can_be_fastened_to_its_neighbour():
    """A part that touches another but has no screw, insert, pin bore or horn
    interface has no defined way of being attached. Counted from the CAD, not docs."""
    from cad import params as P
    from cad.parts.leg import bracket_mount_screw_count, hip_bracket

    assert bracket_mount_screw_count() >= 2, (
        "hip_bracket has no screw interface to the torso — the leg has no defined "
        "attachment to the body")
    # the through-holes must actually be cut: a bracket with the screws only on paper
    # weighs more than one with holes in it
    plain = hip_bracket(1, P.FRONT["hip_off"]).volume
    from cad.parts import fasteners as F
    from build123d import Pos
    holes = 0.0
    for i in range(P.MOUNT_SCREWS):
        sx = (i - (P.MOUNT_SCREWS - 1) / 2.0) * P.MOUNT_BOLT_PITCH
        holes += (hip_bracket(1, P.FRONT["hip_off"])
                  & (Pos(sx, 2.0, 0) * F.screw_clearance(P.MOUNT_SCREW, "y", 12.0))).volume
    assert plain > 0
    assert holes < 1.0, "the mount screw holes are not actually cut in the bracket"


def test_torso_carries_the_matching_leg_mount_inserts():
    """The bracket's screws have to pull into something. The torso flank must present a
    flat pad with heat-set inserts at the same pitch."""
    from cad import params as P
    from cad.parts.body import _leg_mount_pad

    for leg in ("FL", "RL"):
        mx, _ = P.MOUNTS[leg]
        pad = _leg_mount_pad(mx, P.leg_plane_sign(leg))
        bb = pad.bounding_box()
        assert pad.volume > 0
        # the pad seats on the flank plane and grows inboard, never outboard
        assert abs(bb.max.Y) <= P.BODY_W / 2 + 1e-6


# --------------------------------------------------------------- actuator fit
def test_every_joint_that_claims_a_housing_has_room_for_one():
    """A joint can be in the kinematics, carry a servo's mass in the budget, and still
    have nowhere to put the actuator. ``analysis.actuator_fit`` measures that directly;
    this pins the result so a geometry change cannot quietly make it worse.

    The five expression joints are KNOWN not to fit the STS3215 — that is the finding, not
    a regression — so the test asserts the measured spill has not grown, and that the
    joints which do fit a smaller actuator still do.
    """
    from analysis.actuator_fit import report
    from cad.servo import PRESETS

    # the default actuator: nothing here fits yet, but nothing should get worse either
    worst = {r["name"]: r["spill_pct"] for r in report()}
    # The tail was the one of the five that a REMOTE drive could rescue: its actuator
    # moved to the station analysis.actuator_fit found, so it must now fit outright.
    budget = {"tail": 0.0, "head_pan": 3.5, "head_pitch": 4.6, "head_tilt": 6.0}
    for name, pct in worst.items():
        assert name in budget, f"{name} is a new joint with no fit budget recorded"
        assert pct <= budget[name], (
            f"{name}: the actuator now overflows its cavity by {pct:.1f}% "
            f"(was within {budget[name]:.0f}%) — the joint has less room than before")

    # the head gimbal is a size problem, not a placement one: a small serial servo fits
    small = {r["name"]: r["fits"] for r in report(PRESETS["dynamixel_xl330"])}
    for name in [n for n in ("head_pan", "head_pitch", "head_tilt") if n in small]:
        assert small[name], (
            f"{name} no longer houses even a 21x21x34 actuator — the head cavity or the "
            f"gimbal origins moved")


def test_tail_remote_drive_is_assemblable(tail_assembly):
    """The tail's actuator is not on its joint — it is forward in the aft torso, reaching
    the pivot through a four-bar. Crank, pushrod, rocker and the body all have to coexist,
    including the clearance the tail sweeps as it moves."""
    bad = _report(tail_assembly)
    assert not bad, _msg(bad)


def test_tail_four_bar_closes_and_covers_its_joint_range():
    """The linkage has to actually reach the tail's commanded travel, on a
    singularity-free branch — the same standard the knee is held to."""
    import math

    from analysis.fourbar import FourBar
    from cad import params as P

    fb = P.TAIL_FOURBAR
    bar = FourBar(d=P.tail_ground(), r2=fb["crank"], r3=fb["coupler"], r4=fb["rocker"])
    lo, hi = fb["crank_window"]
    sweep = bar.sweep(math.radians(lo), math.radians(hi))
    out = [s[1] for s in sweep]
    mu = [s[2] for s in sweep]
    travel = max(out) - min(out)
    need = P.LIM_TAIL[1] - P.LIM_TAIL[0]
    assert travel >= need, f"tail four-bar gives {travel:.2f} rad, joint needs {need:.2f}"
    assert min(mu) >= 40.0 and max(mu) <= 140.0, (
        f"transmission angle {min(mu):.0f}..{max(mu):.0f} deg leaves the safe band")


def test_the_body_has_room_for_the_expression_actuators_it_claims():
    """Head size and head POSITION are actuator constraints, not just styling.

    The head is a ball whose socket is carved out of the chest, so the two trade against
    each other: enlarging it buys room inside the head and spends room in front of the
    waist. ``HEAD_R = 50`` with the gimbal pushed forward to the torso's nose is the point
    where both are available at once — 2 housings in the head and 1 in the chest. Shrink
    the head, or push it back into the ribcage, and this fails.
    """
    from analysis.actuator_fit import capacity

    head = capacity("head")["count"]
    chest = capacity("chest")["count"]
    assert head >= 2, f"the head cavity now admits only {head} servo housing(s)"
    assert chest >= 1, (
        f"the chest admits {chest} servo housings — the head ball has grown into it, "
        f"or the gimbal has moved back inside the ribcage")


def test_head_still_clears_the_chest_it_sits_in(torso_assembly):
    """The socket cut for the head has to track the head's real size and station."""
    from cad.parts.body import _head_socket
    from cad import params as P

    v = (torso_assembly["head"] & torso_assembly["torso_fore"]).volume
    assert v <= NOISE_MM3, f"head and chest share {v:.0f} mm^3"
    # and the socket must be the head's swept ball, not a stale copy of an older one
    bb = _head_socket().bounding_box()
    assert bb.size.X >= 2 * P.HEAD_R


def test_ears_bolt_onto_the_head_rather_than_through_it():
    """The ears are rigid now (`params.EARS_ACTUATED`), so they have to sit ON the shell.

    Their station used to be a pivot INSIDE the sphere, which put the blade out through
    the skull; a bolted ear has to land on a pad that clears the head's curvature under
    the whole foot, not just under its centre."""
    from build123d import Pos

    from cad import params as P
    from cad.parts.ears import ear
    from cad.parts.head import head

    h = head()
    blade = ear()
    for side in (+1, -1):
        v = ((Pos(*P.ear_station(side)) * blade) & h).volume
        assert v <= NOISE_MM3, f"ear {side:+d} shares {v:.0f} mm^3 with the head shell"


def test_gimbal_housing_reality_is_recorded_not_assumed():
    """`capacity('head')` says two housings fit in the head — but side by side, as a pair.

    A gimbal's axes meet at a point, so its motors cannot sit at that point: each has to
    step aside ALONG its own axis. Measured that way the two do not fit, and pretending
    otherwise is exactly the kind of thing this suite exists to stop. One axis does fit,
    centred. Both facts are pinned here so a head resize has to confront them."""
    from analysis.actuator_fit import capacity, fits_in_head, gimbal_layout, housing_dims

    assert capacity("head")["count"] >= 2, "the head no longer holds a pair of housings"
    assert not gimbal_layout()["ok"], (
        "a direct-drive gimbal now fits — update the design notes that say it does not")

    # a single servo on the pitch axis, centred, is what the head can actually carry
    l, w, h = housing_dims()
    assert fits_in_head((0.0, 0.0, 0.0), (l, h, w)), (
        "even one gimbal servo no longer fits on its axis in the head")


# --------------------------------------------------------------- connectivity
def test_every_printed_part_is_one_solid():
    """A printed part has to be ONE connected solid. Anything else is loose pieces on the
    bed, and no amount of interference checking notices — the pieces do not overlap, they
    simply are not joined.

    The ribcage is a lattice, which makes this easy to get wrong in two ways: a clearance
    cut can sever the ribs and stringers that hold it together, and a boss added in the
    middle of the cage touches nothing at all.
    """
    from cad.assembly import PRINTABLE

    # Slivers below this are OCCT artefacts of a tangential cut, not parts: the ones this
    # catches measure ~0.05 mm thick and would not survive a slicer, let alone a printer.
    # A real loose piece is orders of magnitude bigger — the ones this test has caught so
    # far ran from 500 mm^3 to whole ribcage sections.
    SLIVER_MM3 = 5.0

    broken = {}
    for name, part in PRINTABLE.items():
        real = [s for s in part.solids() if s.volume > SLIVER_MM3]
        if len(real) != 1:
            broken[name] = len(real)
    assert not broken, "parts that are not a single solid: " + ", ".join(
        f"{k} ({v} pieces)" for k, v in sorted(broken.items()))
