"""Sabo vision — camera → cat detection → brain.hal.CatDetection.

The detector backend is pluggable (stub on the dev box; YOLO/ONNX/TensorRT on the
Jetson). The bbox→CatDetection geometry is hardware-independent and tested.
"""
