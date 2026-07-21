# Sabo vision — cat detection

Turns camera frames into the `brain.hal.CatDetection` the perception layer
consumes. The **geometry** (bbox → bearing/distance/approach) is hardware-
independent and tested; the **detector backend** is swapped per platform.

```
frame ─▶ Detector.detect() ─▶ best_box ─▶ box_to_detection() ─▶ CatDetection
         (stub | YOLO | Haar)   (geometry.py: pinhole bearing + distance)
```

## Backends (`vision/detector.py`)
| Backend | Use | Deps |
|---|---|---|
| **stub** | dev box / no model — returns "no cat" (or scripted boxes in tests) | none |
| **yolo** | real detection — loads `.pt` (dev/4090), `.onnx`, or **`.engine`** (TensorRT on Jetson); Ultralytics does pre/post-proc for all three | ultralytics |
| **haar** | zero-training baseline (frontal cat faces) | opencv (≤4, needs `CascadeClassifier`) |

`load_detector("auto", weights=...)` picks YOLO if the weights load, else stub —
so the code path is identical everywhere; only the weights file differs.

## Train on the RTX 4090 → deploy to the Jetson
1. **Dataset:** cat images. Sami's own photos in `sami_photos/` are the eval/
   fine-tune set (convert HEIC→JPG first). Or start from COCO (class `cat`, id 15).
2. **Train (4090, PyTorch/Ultralytics):**
   ```bash
   yolo detect train model=yolov8n.pt data=cats.yaml imgsz=640 epochs=100
   ```
3. **Export → ONNX, then build a TensorRT engine on the Jetson** (do the
   `.engine` build *on* the Jetson — engines are device-specific):
   ```bash
   # on the 4090:      yolo export model=best.pt format=onnx
   # copy best.onnx to the Jetson, then on JetPack:
   yolo export model=best.onnx format=engine half=True   # → best.engine
   ```
4. **Drop it in:** put the engine at `vision/models/cat_yolov8n.engine`.
   `hardware/jetson_backend.py` (`CAT_WEIGHTS`) auto-loads it and `camera()` goes
   live — no code change.

## Calibration (`vision/geometry.py`)
- `CameraModel(img_w, img_h, hfov_deg)` — set to your CSI module's resolution +
  horizontal FOV (default 120°).
- `CAT_HEIGHT_M` (0.25 m) — nominal cat height for the pinhole distance estimate.
  Distance is an *estimate* for approach/withdraw decisions, not metrology; tune
  it against a few known-distance frames of Sami.

## Known limits / TODO
- A plain detector gives location, not mood. `ears`/`hissing` default to neutral;
  a **lunge** is inferred from closing speed so SCARED still trips. A 2nd-stage
  pose/expression classifier could fill true ear/tail/hiss state.
- **EIS:** stabilise the frame with the IMU (`docs/camera_stabilization.md`)
  before detection when moving — wire in once frames are real.
