"""The engineering validation gate, as regression tests.

``python -m analysis.validate`` is the PASS/FAIL gate behind the README's headline
numbers (mass, stance, balance, torque). It runs inside the
``analysis.platform_report`` pipeline, but nothing in the *test suite* noticed when a
``cad/params.py`` edit pushed one of those numbers out of band — the whole point of
design-as-code is that a parameter change is propagated everywhere, so it has to be
caught everywhere too.

Importing ``analysis.validate`` builds the entire build123d model (~25 s), so it is
imported lazily in a session fixture and these tests are marked ``slow``::

    pytest -m "not slow"      # skip the CAD build
"""

import re
import os

import pytest

pytestmark = pytest.mark.slow

ROOT = os.path.dirname(os.path.dirname(os.path.abspath(__file__)))


@pytest.fixture(scope="session")
def validate():
    """The validation module (importing it builds the CAD — do it once)."""
    import analysis.validate as v
    return v


def test_mass_inside_target_band(validate):
    ok, detail = validate.check_mass()
    assert ok, detail


def test_stance_angles_inside_joint_limits(validate):
    ok, detail = validate.check_stance()
    assert ok, detail


def test_com_inside_support_polygon(validate):
    ok, detail = validate.check_balance()
    assert ok, detail


def test_peak_joint_torque_inside_servo_budget(validate):
    ok, detail = validate.check_torque()
    assert ok, "\n" + detail


def test_gate_passes_end_to_end(validate):
    """The exact gate ``analysis.platform_report`` runs as pipeline stage 2."""
    assert validate.main() is True


def test_published_mass_matches_the_model(validate):
    """Design-as-code: the mass printed in README.md must be the mass the model
    computes. If this fails the design changed — regenerate the docs
    (``python -m analysis.platform_report``) rather than editing the number here."""
    _, _, total_kg = validate.total_mass()
    computed_g = round(total_kg * 1000)

    readme = open(os.path.join(ROOT, "README.md"), encoding="utf-8").read()
    m = re.search(r"\|\s*Mass\s*\|\s*\*\*(\d+)\s*g\*\*", readme)
    assert m, "could not find the Mass row in README.md's key-numbers table"
    published_g = int(m.group(1))

    assert published_g == computed_g, (
        f"README.md publishes {published_g} g but the model computes {computed_g} g "
        f"— rerun `python -m analysis.platform_report` and update the docs"
    )
