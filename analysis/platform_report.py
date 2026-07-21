"""
Platform report — one-command, reproducible artifact regeneration.
==================================================================

    python -m analysis.platform_report            # full pipeline + manifest
    python -m analysis.platform_report --skip-cad # skip the slow CAD re-export

This IS the design-as-code proof: it regenerates the whole artifact set IN ORDER
from the single source of truth, so every downstream document (mass manifest, spec
sheet, benchmark) is provably re-derived from ``cad/params.py`` + ``cad/servo.py``
in one run — no hand-authored URDF/spec drift.

Pipeline (each stage a fresh subprocess for isolation + honest exit codes):

    1. cad.export            → cad/out/ (STLs, parts_manifest.json, previews)
    2. analysis.validate     → mass/stance/balance/torque PASS-FAIL gate
    3. sim.meshes            → sim/meshes/ (sim geometry, forced refresh)
    4. analysis.platform_spec→ docs/out/platform_spec.{md,json}
    5. analysis.benchmark    → docs/out/benchmark.{md,json}

Then it prints a manifest of what each stage wrote (path, size, mtime).
"""

from __future__ import annotations

import argparse
import os
import subprocess
import sys
import time

ROOT = os.path.abspath(os.path.join(os.path.dirname(__file__), ".."))
DOCS_OUT = os.path.join(ROOT, "docs", "out")
CAD_OUT = os.path.join(ROOT, "cad", "out")
MESH_DIR = os.path.join(ROOT, "sim", "meshes")

# (module, argv, human label, output dir watched for the manifest)
STAGES = [
    ("cad.export", [], "CAD export (STL + mass manifest + previews)", CAD_OUT),
    ("analysis.validate", [], "Engineering validation gate (mass/stance/balance/torque)", None),
    ("sim.meshes", [], "Sim mesh export (inner frame + skin)", MESH_DIR),
    ("analysis.platform_spec", [], "Platform spec sheet", DOCS_OUT),
    ("analysis.benchmark", [], "Platform benchmark", DOCS_OUT),
]


def _snapshot(path: str) -> dict:
    if path is None or not os.path.isdir(path):
        return {}
    return {f: os.path.getmtime(os.path.join(path, f))
            for f in os.listdir(path) if os.path.isfile(os.path.join(path, f))}


def _run(module: str, argv: list) -> tuple[int, float]:
    env = dict(os.environ, PYTHONIOENCODING="utf-8")
    t0 = time.time()
    proc = subprocess.run([sys.executable, "-m", module, *argv], cwd=ROOT, env=env)
    return proc.returncode, time.time() - t0


def main() -> int:
    ap = argparse.ArgumentParser()
    ap.add_argument("--skip-cad", action="store_true",
                    help="skip cad.export + sim.meshes (reuse cached geometry)")
    args = ap.parse_args()

    os.makedirs(DOCS_OUT, exist_ok=True)
    print("=" * 70)
    print("Sabo — platform artifact regeneration (design-as-code reproducibility)")
    print("=" * 70)

    written: dict[str, list[str]] = {}
    results = []
    for module, argv, label, watch in STAGES:
        if args.skip_cad and module in ("cad.export", "sim.meshes"):
            print(f"\n[SKIP] {module} — {label}")
            results.append((module, "skip", 0.0))
            continue
        before = _snapshot(watch)
        print(f"\n[RUN ] {module} — {label}")
        code, dt = _run(module, argv)
        after = _snapshot(watch)
        new_or_changed = sorted(f for f, mt in after.items() if before.get(f) != mt)
        if watch is not None:
            written[module] = [os.path.join(watch, f) for f in new_or_changed]
        status = "ok" if code == 0 else f"FAIL(exit {code})"
        results.append((module, status, dt))
        print(f"[{'DONE' if code == 0 else 'FAIL'}] {module} in {dt:.1f}s"
              + (f" — {len(new_or_changed)} files written" if watch is not None else ""))
        if code != 0:
            print(f"\nABORTED at {module} (exit {code}).")
            _summary(results)
            return code

    print("\n" + "=" * 70)
    print("MANIFEST — files written this run")
    print("=" * 70)
    for module, argv, label, watch in STAGES:
        if module not in written:
            continue
        files = written[module]
        print(f"\n{module}  ({len(files)} files)")
        for p in files:
            try:
                size = os.path.getsize(p)
            except OSError:
                size = 0
            print(f"    {os.path.relpath(p, ROOT).replace(os.sep, '/'):45s} {size:>9,d} B")

    # the headline documents (always list explicitly)
    print("\nKey documents:")
    for doc in ("platform_spec.md", "platform_spec.json", "benchmark.md", "benchmark.json"):
        p = os.path.join(DOCS_OUT, doc)
        mark = "OK" if os.path.exists(p) else "MISSING"
        print(f"    [{mark}] {os.path.relpath(p, ROOT).replace(os.sep, '/')}")

    _summary(results)
    return 0


def _summary(results: list) -> None:
    print("\n" + "-" * 70)
    print("Stage summary:")
    for module, status, dt in results:
        print(f"    {module:28s} {status:14s} {dt:6.1f}s")
    print("-" * 70)


if __name__ == "__main__":
    sys.exit(main())
