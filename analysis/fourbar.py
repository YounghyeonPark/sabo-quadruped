"""
Four-bar knee linkage — design + kinematic verification.
========================================================

    python -m analysis.fourbar

Drives the knee from a hip-mounted (proximal) motor through a rigid four-bar, so
the shank stays light and there's **no cable friction** (see docs/tendon_actuation).

Four-bar (planar), in the thigh frame with the knee at the origin:
    O4 = knee pivot (0,0)            — rocker is rigidly fixed to the shank here
    O2 = crank pivot (0, d)          — at the hip end of the thigh (ground = thigh)
    r2 = crank (driven by the knee motor)   r4 = rocker (on the shank)
    r3 = coupler / pushrod (connects crank tip → rocker tip)

Input = crank angle θ2 (motor). Output = rocker angle θ4 → knee joint angle.
We verify: the knee **range of motion**, that the **transmission angle** stays
away from 0/180° (no singularity/lockup), monotonic (so it's invertible), and we
provide the inverse (knee → crank) for the gait/IK.
"""

from __future__ import annotations

import math
from dataclasses import dataclass


# Designed geometry (from search(), below): a proximal-crank knee drive with a
# 2.24 rad monotonic knee sweep and a healthy 41–140° transmission angle.
@dataclass
class FourBar:
    d: float = 38.0       # ground (thigh): O2..O4 distance (mm)
    r2: float = 27.0      # crank (mm) — driven by the hip-mounted knee motor
    r3: float = 50.0      # coupler / pushrod (mm)
    r4: float = 18.0      # rocker on the shank (mm)
    rocker_offset: float = 0.0    # rad: rocker fixed angle vs shank long axis
    branch: int = -1      # coupler-rocker assembly solution
    crank_window: tuple = (-20.0, 81.0)   # usable crank range (deg)

    @property
    def O2(self):
        return (0.0, self.d)

    def solve(self, theta2: float):
        """Crank angle θ2 (rad) → (theta4, knee_angle, transmission_angle) or None
        if the linkage can't close at this input (out of range)."""
        o2x, o2y = self.O2
        cx = o2x + self.r2 * math.cos(theta2)      # crank tip
        cy = o2y + self.r2 * math.sin(theta2)
        # rocker tip R lies on circle(O4=origin, r4) ∩ circle(C, r3)
        dx, dy = cx, cy                             # O4 is origin
        L = math.hypot(dx, dy)
        if L < 1e-6 or L > self.r4 + self.r3 or L < abs(self.r4 - self.r3):
            return None                             # cannot assemble
        a = (self.r4 * self.r4 - self.r3 * self.r3 + L * L) / (2 * L)
        h2 = self.r4 * self.r4 - a * a
        if h2 < 0:
            return None
        h = math.sqrt(h2)
        mx, my = a * dx / L, a * dy / L             # foot of perpendicular
        rx = mx + self.branch * h * (-dy / L)       # rocker tip R
        ry = my + self.branch * h * (dx / L)
        theta4 = math.atan2(ry, rx)
        knee = theta4 - self.rocker_offset          # shank orientation → knee joint
        # transmission angle μ = angle between coupler (C→R) and rocker (O4→R)
        v_cr = (rx - cx, ry - cy)
        v_or = (rx, ry)
        dot = v_cr[0]*v_or[0] + v_cr[1]*v_or[1]
        mu = math.acos(max(-1, min(1, dot / (math.hypot(*v_cr)*math.hypot(*v_or) + 1e-9))))
        return theta4, knee, mu

    def linkage_points(self, theta2: float):
        """Return the drawable joints (O2, C, R, O4) + shank tip for a crank angle."""
        o2 = self.O2
        cx = o2[0] + self.r2 * math.cos(theta2)
        cy = o2[1] + self.r2 * math.sin(theta2)
        s = self.solve(theta2)
        if not s:
            return None
        theta4 = s[0]
        R = (self.r4 * math.cos(theta4), self.r4 * math.sin(theta4))
        knee = s[1]
        # shank: from knee O4 down the leg at (−90° − knee) so knee=0 → straight down
        shank_dir = -math.pi / 2 - knee
        tip = (65 * math.cos(shank_dir), 65 * math.sin(shank_dir))
        return dict(O2=o2, C=(cx, cy), R=R, O4=(0.0, 0.0), tip=tip, knee=knee)

    def sweep(self, t2_lo, t2_hi, n=140):
        out = []
        for i in range(n + 1):
            t2 = t2_lo + (t2_hi - t2_lo) * i / n
            s = self.solve(t2)
            if s:
                out.append((t2, s[1], math.degrees(s[2])))   # (crank, knee, mu_deg)
        return out

    def crank_for_knee(self, knee_target: float, t2_lo, t2_hi):
        """Inverse: crank angle that yields the desired knee angle (bisection on the
        monotonic sweep)."""
        data = self.sweep(t2_lo, t2_hi, 200)
        best = min(data, key=lambda r: abs(r[1] - knee_target))
        return best[0]


MU_LO, MU_HI = 40.0, 140.0      # healthy transmission-angle band (deg)
NEED_ROM = 1.75                 # rad of knee travel we want to cover


def usable_window(fb: FourBar):
    """Longest contiguous crank window (deg) where the linkage assembles, the
    transmission angle stays in band, and the knee moves monotonically.
    Returns (rom_rad, lo_deg, hi_deg, mu_min, mu_max) or None."""
    pts = []
    for deg in range(-180, 181):
        s = fb.solve(math.radians(deg))
        pts.append((deg, s[1], math.degrees(s[2])) if s else None)
    best = None
    i = 0
    n = len(pts)
    while i < n:
        if pts[i] is None or not (MU_LO <= pts[i][2] <= MU_HI):
            i += 1; continue
        j = i
        while j + 1 < n and pts[j+1] is not None and MU_LO <= pts[j+1][2] <= MU_HI:
            j += 1
        run = [p for p in pts[i:j+1]]
        knees = [r[1] for r in run]
        inc = all(knees[k] < knees[k+1] for k in range(len(knees)-1))
        dec = all(knees[k] > knees[k+1] for k in range(len(knees)-1))
        if (inc or dec) and len(run) > 3:
            rom = abs(knees[-1] - knees[0])
            mus = [r[2] for r in run]
            cand = (rom, run[0][0], run[-1][0], min(mus), max(mus))
            if best is None or cand[0] > best[0]:
                best = cand
        i = j + 1
    return best


def search():
    """Grid-search link geometry for the widest monotonic, singularity-free knee."""
    best = None
    for d in range(38, 66, 4):
        for r2 in range(12, 30, 3):
            for r4 in range(18, 40, 3):
                for r3 in range(34, 72, 4):
                    for branch in (+1, -1):
                        fb = FourBar(d, r2, r3, r4, 0.0, branch)
                        w = usable_window(fb)
                        if w and w[0] >= NEED_ROM:
                            # prefer wide ROM + μ centred near 90°
                            score = w[0] - 0.002 * abs((w[3] + w[4]) / 2 - 90)
                            if best is None or score > best[0]:
                                best = (score, fb, w)
    return best


def report(fb: FourBar, w) -> bool:
    rom, lo, hi, mu_lo, mu_hi = w
    print("Four-bar knee linkage — design + verification")
    print("=" * 52)
    print(f"  geometry (mm): ground d={fb.d}, crank r2={fb.r2}, coupler r3={fb.r3}, "
          f"rocker r4={fb.r4}, branch={fb.branch:+d}")
    print(f"  crank window   : {lo:.0f}°..{hi:.0f}°")
    print(f"  knee ROM       : {math.degrees(rom):.0f}° = {rom:.2f} rad  "
          f"(need ≳{NEED_ROM})")
    print(f"  transmission μ : {mu_lo:.0f}°..{mu_hi:.0f}°  "
          f"({'OK — no singularity' if mu_lo >= MU_LO and mu_hi <= MU_HI else 'RISK'})")
    print(f"  monotonic      : yes (invertible over the window)")
    ok = rom >= NEED_ROM and mu_lo >= MU_LO and mu_hi <= MU_HI
    print(f"  RESULT         : {'PASS — usable proximal knee drive' if ok else 'tune'}")
    return ok


def animate(fb: FourBar, path="sim/out/fourbar.gif"):
    """Animate the linkage across its crank window — visual sanity check."""
    import os
    import matplotlib
    matplotlib.use("Agg")
    import matplotlib.pyplot as plt
    from matplotlib.animation import FuncAnimation, PillowWriter
    os.makedirs(os.path.dirname(path), exist_ok=True)
    lo, hi = (math.radians(a) for a in fb.crank_window)
    frames = [fb.linkage_points(lo + (hi - lo) * i / 40) for i in range(41)]
    frames = [f for f in frames if f]
    fig, ax = plt.subplots(figsize=(4, 5))

    def draw(k):
        ax.clear(); ax.set_aspect("equal"); ax.set_axis_off()
        ax.set_xlim(-70, 60); ax.set_ylim(-75, 55)
        p = frames[k]
        # ground (thigh) O2–O4, crank O2–C, coupler C–R, rocker O4–R, shank O4–tip
        for a, b, c, w in [("O2", "O4", "#888", 2), ("O2", "C", "#d9702f", 4),
                           ("C", "R", "#0e9488", 4), ("O4", "R", "#5b9bd5", 4),
                           ("O4", "tip", "#cdd2dc", 6)]:
            ax.plot([p[a][0], p[b][0]], [p[a][1], p[b][1]], c=c, lw=w,
                    solid_capstyle="round")
        for name in ("O2", "C", "R", "O4"):
            ax.plot(*p[name], "ko", ms=4)
        ax.set_title(f"knee = {math.degrees(p['knee']):.0f}°", fontsize=10)

    anim = FuncAnimation(fig, draw, frames=len(frames), interval=60)
    anim.save(path, writer=PillowWriter(fps=15)); plt.close(fig)
    print(f"  animation      : {path}")


if __name__ == "__main__":
    fb = FourBar()
    w = usable_window(fb)
    report(fb, w)
    animate(fb)
