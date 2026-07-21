"""Sabo RL training — URDF export, Isaac Lab locomotion task, policy deploy.

Train a locomotion policy for Sabo's underactuated body on the RTX 4090 (Isaac
Lab), export to ONNX, run it on the Jetson via TensorRT. The learned policy is a
drop-in replacement for the hand-authored gait engine (sim/gait.py).
"""
