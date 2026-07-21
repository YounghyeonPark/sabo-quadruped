"""
Detection geometry — turn a camera bounding box into a `CatDetection`.
======================================================================

This is the hardware-independent core: given a detected cat box in the image, a
pinhole camera model recovers **bearing** (which way the cat is) and **distance**
(how far), and temporal differencing gives **approaching** + **speed**. The
detector that produces the box is swappable; this math is not.

Conventions match `brain.hal.CatDetection`: bearing 0 = straight ahead, + = the
kitten's left (image-left), radians; distance in metres.

Distance uses apparent height (pinhole): a cat of real height `H` fills
`h_px = f_y * H / distance`, so `distance = f_y * H / h_px`. Height is more
stable than width for a cat (width swings with pose/tail); we use the box height
and a nominal standing cat height, so "distance" is an estimate good enough for
approach/withdraw decisions, not metrology.
"""

from __future__ import annotations

import math
from dataclasses import dataclass

from brain.hal import CatDetection, EarPose

# nominal real cat size for the pinhole distance estimate
CAT_HEIGHT_M = 0.25       # a sitting/standing cat's visible height
LUNGE_SPEED = 0.35        # m/s bearing-frame closing speed that reads as a lunge


@dataclass
class Box:
    """Axis-aligned detection box in pixels + confidence."""
    x: float          # left
    y: float          # top
    w: float
    h: float
    conf: float = 1.0

    @property
    def cx(self) -> float:
        return self.x + self.w / 2

    @property
    def cy(self) -> float:
        return self.y + self.h / 2


class CameraModel:
    """Pinhole model from horizontal FOV + image size (wide-FOV CSI cam)."""

    def __init__(self, img_w: int, img_h: int, hfov_deg: float = 120.0):
        self.w = img_w
        self.h = img_h
        self.fx = (img_w / 2) / math.tan(math.radians(hfov_deg) / 2)
        self.fy = self.fx      # square pixels

    def bearing(self, cx: float) -> float:
        """Image-column → bearing (rad, + = kitten's left)."""
        return math.atan2((self.w / 2) - cx, self.fx)

    def distance(self, box_h: float, cat_h_m: float = CAT_HEIGHT_M) -> float:
        if box_h <= 1:
            return float("inf")
        return self.fy * cat_h_m / box_h


def box_to_detection(box: Box | None, cam: CameraModel,
                     prev: CatDetection | None = None, dt: float = 0.0) -> CatDetection:
    """Convert the best box (or None) to a `CatDetection`, using prev for motion."""
    if box is None:
        return CatDetection(present=False)

    dist = cam.distance(box.h)
    bearing = cam.bearing(box.cx)

    approaching = False
    speed = 0.0
    if prev is not None and prev.present and dt > 0:
        closing = (prev.distance - dist) / dt          # + = getting closer
        approaching = closing > 0.01
        # lateral + radial speed estimate in the ground plane
        lateral = dist * (bearing - prev.bearing) / dt
        speed = math.hypot(closing, lateral)

    return CatDetection(
        present=True, distance=dist, bearing=bearing, speed=speed,
        approaching=approaching,
        # A plain detector can't read the cat's mood — ears/hiss need a 2nd-stage
        # pose/expression classifier (TODO). Default to neutral/non-threat; the
        # lunge case is inferred from closing speed here so SCARED can still trip.
        ears=EarPose.FLAT if (approaching and speed >= LUNGE_SPEED) else EarPose.NEUTRAL,
        hissing=False,
    )


def best_box(boxes: list[Box]) -> Box | None:
    """Pick the most confident (ties → largest) detection."""
    if not boxes:
        return None
    return max(boxes, key=lambda b: (b.conf, b.w * b.h))
