"""
Sabo hardware HAL backend (Phase 2).
====================================

Real-hardware implementations of the ``brain/hal.py`` seam so the *unchanged*
``brain/`` code can drive the physical kitten on the NVIDIA Jetson Orin Nano.

    from hardware.jetson_backend import HardwareBody, HardwareSenses

Every driver import is guarded: with no hardware libraries / buses present (e.g.
the Windows dev machine) the backend runs in **stub mode**, logging intended
actuator writes and returning safe neutral sensor values. See ``README.md``.
"""

from __future__ import annotations

from hardware.jetson_backend import HardwareBody, HardwareSenses

__all__ = ["HardwareBody", "HardwareSenses"]
