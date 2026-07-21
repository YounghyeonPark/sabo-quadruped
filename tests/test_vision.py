"""Vision: bbox → CatDetection geometry + pipeline (vision/)."""

from brain.hal import EarPose
from vision.detector import StubDetector
from vision.geometry import (Box, CameraModel, best_box, box_to_detection)
from vision.pipeline import VisionPipeline
from tests.conftest import Clock

CAM = CameraModel(640, 480, hfov_deg=120.0)


def _centered(h=100):
    return Box(x=320 - 30, y=240 - h/2, w=60, h=h)


def test_bearing_sign_and_zero():
    # centered box → straight ahead
    assert abs(CAM.bearing(320)) < 1e-6
    # left-of-center → + (kitten's left); right-of-center → -
    assert CAM.bearing(160) > 0.3
    assert CAM.bearing(480) < -0.3


def test_distance_shrinks_with_apparent_size():
    near = CAM.distance(200)
    far = CAM.distance(80)
    assert near < far                      # bigger box = closer
    # pinhole sanity: doubling height ≈ halving distance
    assert abs(CAM.distance(200) - CAM.distance(100) / 2) < 1e-6


def test_none_box_is_absent():
    d = box_to_detection(None, CAM)
    assert not d.present


def test_present_box_fills_fields():
    d = box_to_detection(_centered(120), CAM)
    assert d.present and d.distance > 0 and abs(d.bearing) < 1e-6


def test_approaching_and_speed_from_prev():
    clock = Clock()
    prev = box_to_detection(_centered(80), CAM)           # farther
    now = box_to_detection(_centered(160), CAM, prev=prev, dt=0.1)  # bigger = closer
    assert now.approaching and now.speed > 0


def test_fast_close_reads_as_threat():
    prev = box_to_detection(_centered(60), CAM)
    lunge = box_to_detection(_centered(300), CAM, prev=prev, dt=0.1)
    assert lunge.ears == EarPose.FLAT     # lunge → threat so brain can go SCARED


def test_best_box_prefers_confidence_then_size():
    a = Box(0, 0, 10, 10, conf=0.9)
    b = Box(0, 0, 50, 50, conf=0.5)
    assert best_box([a, b]) is a
    assert best_box([]) is None


def test_pipeline_stub_reports_no_cat():
    vp = VisionPipeline(StubDetector(), clock=Clock())
    assert not vp.update(frame=None).present


def test_pipeline_scripted_detection():
    clock = Clock()
    vp = VisionPipeline(StubDetector([_centered(140)]), img_w=640, img_h=480, clock=clock)
    d = vp.update(frame="dummy")           # stub ignores the frame content
    assert d.present and d.distance > 0
