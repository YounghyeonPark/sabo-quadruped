"""
Detector backends — produce cat bounding boxes from a frame.
============================================================

Pluggable so the same pipeline runs everywhere:
  * **StubDetector** — no deps; returns nothing (or scripted boxes for tests).
    Default on the dev box and when no model is present.
  * **YoloDetector** — Ultralytics YOLO; loads `.pt` (dev/4090), `.onnx`, or
    **`.engine` (TensorRT on the Jetson)** — Ultralytics handles pre/post-proc for
    all three, so the deploy path is one class.
  * **HaarDetector** — OpenCV cat-face cascade; a zero-training baseline for
    environments that still ship `cv2.CascadeClassifier` (OpenCV ≤4).

All heavy imports are lazy + guarded, so `import vision.detector` always works.
"""

from __future__ import annotations

import os
from abc import ABC, abstractmethod

from vision.geometry import Box

COCO_CAT = 15   # 'cat' class id in COCO


class Detector(ABC):
    name = "detector"

    @abstractmethod
    def detect(self, frame) -> list[Box]:
        """frame = HxWx3 uint8 (BGR). Returns cat boxes (may be empty)."""

    @property
    def available(self) -> bool:
        return True


class StubDetector(Detector):
    """No model. Returns scripted boxes if given (tests), else nothing."""
    name = "stub"

    def __init__(self, scripted: list[Box] | None = None):
        self._scripted = scripted or []

    def detect(self, frame) -> list[Box]:
        return list(self._scripted)


class YoloDetector(Detector):
    """Ultralytics YOLO over .pt/.onnx/.engine. Recommended real backend."""
    name = "yolo"

    def __init__(self, weights: str, conf: float = 0.4, cat_class: int = COCO_CAT):
        self.conf = conf
        self.cat_class = cat_class
        self._model = None
        try:
            from ultralytics import YOLO
            if os.path.exists(weights):
                self._model = YOLO(weights)
        except Exception as e:            # ultralytics/torch absent, or bad weights
            self._err = str(e)

    @property
    def available(self) -> bool:
        return self._model is not None

    def detect(self, frame) -> list[Box]:
        if self._model is None:
            return []
        res = self._model.predict(frame, conf=self.conf, verbose=False)[0]
        boxes = []
        for b in res.boxes:
            if int(b.cls[0]) != self.cat_class:
                continue
            x1, y1, x2, y2 = (float(v) for v in b.xyxy[0])
            boxes.append(Box(x1, y1, x2 - x1, y2 - y1, float(b.conf[0])))
        return boxes


class HaarDetector(Detector):
    """OpenCV Haar cat-face cascade (baseline; needs cv2.CascadeClassifier)."""
    name = "haar"

    def __init__(self, cascade_path: str, scale=1.05, min_neighbors=3):
        self._c = None
        self.scale, self.min_neighbors = scale, min_neighbors
        try:
            import cv2
            if hasattr(cv2, "CascadeClassifier") and os.path.exists(cascade_path):
                c = cv2.CascadeClassifier(cascade_path)
                self._c = None if c.empty() else c
        except Exception:
            pass

    @property
    def available(self) -> bool:
        return self._c is not None

    def detect(self, frame) -> list[Box]:
        if self._c is None:
            return []
        import cv2
        gray = cv2.cvtColor(frame, cv2.COLOR_BGR2GRAY)
        rects = self._c.detectMultiScale(gray, self.scale, self.min_neighbors)
        return [Box(float(x), float(y), float(w), float(h), 1.0) for (x, y, w, h) in rects]


def load_detector(backend: str = "auto", weights: str | None = None,
                  **kw) -> Detector:
    """Pick a backend. 'auto' → best available (YOLO if weights load) else stub."""
    if backend == "stub":
        return StubDetector(**kw)
    if backend in ("yolo", "auto") and weights:
        d = YoloDetector(weights, **kw)
        if d.available:
            return d
    if backend == "haar" and weights:
        d = HaarDetector(weights, **kw)
        if d.available:
            return d
    return StubDetector()
