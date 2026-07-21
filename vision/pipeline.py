"""
VisionPipeline — frame in, `CatDetection` out.
==============================================

Ties a detector backend to the geometry + temporal state. This is what
`hardware.jetson_backend.HardwareSenses.camera()` calls each tick: it runs the
detector, picks the best cat box, and converts it (with the previous frame's
result and the elapsed time) into the `CatDetection` the brain consumes.

On the dev box the detector is a stub (returns "no cat"); on the Jetson it's a
TensorRT YOLO engine — the pipeline and everything above it are identical.
"""

from __future__ import annotations

from brain.hal import CatDetection
from vision.detector import Detector, StubDetector
from vision.geometry import CameraModel, best_box, box_to_detection


class VisionPipeline:
    def __init__(self, detector: Detector | None = None,
                 img_w: int = 640, img_h: int = 480, hfov_deg: float = 120.0,
                 clock=None):
        self.detector = detector or StubDetector()
        self.cam = CameraModel(img_w, img_h, hfov_deg)
        self._prev: CatDetection | None = None
        self._prev_t: float | None = None
        if clock is None:
            import time
            clock = time.monotonic
        self._now = clock

    def update(self, frame) -> CatDetection:
        """Run detection on one frame → CatDetection (None frame → not present)."""
        now = self._now()
        dt = 0.0 if self._prev_t is None else max(0.0, now - self._prev_t)
        self._prev_t = now

        boxes = self.detector.detect(frame) if frame is not None else []
        det = box_to_detection(best_box(boxes), self.cam, self._prev, dt)
        self._prev = det
        return det


def annotate(frame, boxes, det=None):
    """Draw boxes + a distance/bearing label on a frame (debug/demo). Needs cv2."""
    import cv2
    out = frame.copy()
    for b in boxes:
        cv2.rectangle(out, (int(b.x), int(b.y)), (int(b.x + b.w), int(b.y + b.h)),
                      (60, 200, 90), 2)
    if det is not None and det.present:
        cv2.putText(out, f"cat {det.distance:.2f}m {det.bearing:+.2f}rad",
                    (10, 24), cv2.FONT_HERSHEY_SIMPLEX, 0.7, (60, 200, 90), 2)
    return out
