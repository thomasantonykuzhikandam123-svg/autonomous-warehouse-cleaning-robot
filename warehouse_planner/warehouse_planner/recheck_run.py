#!/usr/bin/env python3
"""
recheck_run.py -- re-measure a recorded run against the floor the robot can ACTUALLY reach.

    python3 recheck_run.py ~/runs/baseline_1            # numbers
    python3 recheck_run.py ~/runs/baseline_1 --plot     # + reachable_heatmap.png

Why this exists: in /map the inside of each rack is marked as free floor, not as an
obstacle (only the rack's outline is drawn). So every coverage % measured against
"mapped floor" -- the recorder's and coverage_nav2's alike -- counts tens of square
metres the robot can never touch, and caps the achievable number well below 100%.

This finds the real denominator:
  1. centre-legal cells  = where the robot CENTRE fits (>= robot_radius from anything
                           not-free, unknown included)
  2. reachable centres   = the centre-legal cells connected to the dock (flood fill),
                           which drops rack interiors and any other sealed pocket
  3. reachable floor     = free cells within brush_radius of a reachable centre

It only reads the files the recorder already wrote. Nothing is re-run, no recorded
data changes, and the recorder itself is left alone so the instrument stays identical
across every run of the experiment.
Written for the VM's Python 3.10 / numpy 1.21.
"""
import argparse
import csv
import json
import math
import os
from collections import deque

import numpy as np


def disc_offsets(radius_m, res):
    """Cell offsets of a disc, as (dy, dx) pairs."""
    n = int(math.ceil(radius_m / res))
    out = []
    for dy in range(-n, n + 1):
        for dx in range(-n, n + 1):
            if (dx * res) ** 2 + (dy * res) ** 2 <= radius_m * radius_m:
                out.append((dy, dx))
    return out


def dilate(mask, offsets):
    """Grow a boolean mask by a set of cell offsets (shift-OR, no scipy needed)."""
    h, w = mask.shape
    out = np.zeros_like(mask)
    for dy, dx in offsets:
        ys0, ys1 = max(0, dy), min(h, h + dy)
        xs0, xs1 = max(0, dx), min(w, w + dx)
        if ys0 >= ys1 or xs0 >= xs1:
            continue
        out[ys0:ys1, xs0:xs1] |= mask[ys0 - dy:ys1 - dy, xs0 - dx:xs1 - dx]
    return out


def flood(ok, seed):
    """Cells of `ok` connected to `seed` (4-neighbour flood fill)."""
    h, w = ok.shape
    seen = np.zeros_like(ok)
    sy, sx = seed
    if not ok[sy, sx]:                       # dock not centre-legal: start at the nearest cell that is
        ys, xs = np.nonzero(ok)
        if ys.size == 0:
            return seen
        i = np.argmin((ys - sy) ** 2 + (xs - sx) ** 2)
        sy, sx = int(ys[i]), int(xs[i])
    seen[sy, sx] = True
    q = deque([(sy, sx)])
    while q:
        y, x = q.popleft()
        for ny, nx in ((y - 1, x), (y + 1, x), (y, x - 1), (y, x + 1)):
            if 0 <= ny < h and 0 <= nx < w and ok[ny, nx] and not seen[ny, nx]:
                seen[ny, nx] = True
                q.append((ny, nx))
    return seen


# The rack footprints, MAP frame (x0, x1, y0, y1), from coverage_nav2.obstacles:
# 1.0 m wide at world x = 1.5, 5.5, 9.5, 13.5, spanning world y 5.5-17.5.
# They are solid in Gazebo, but /map draws only their OUTLINE, and those
# outlines have gaps wide enough for a flood fill to walk through. Marking them
# blocked here is what stops 23.6 m2 of rack interior being reported as floor
# the robot failed to sweep.
SOLID = [(cx - 0.5, cx + 0.5, 1.5, 13.5) for cx in (-2.5, 1.5, 5.5, 9.5)]


def analyse(occ, res, origin, seed_xy, robot_radius=0.30, brush_radius=0.32,
            solid=SOLID):
    ox, oy = origin
    free = occ == 0
    blocked = ~free                                  # occupied AND unknown block the robot
    h, w = occ.shape
    for x0, x1, y0, y1 in solid or []:
        i0, i1 = max(0, int((x0 - ox) / res)), min(w, int((x1 - ox) / res))
        j0, j1 = max(0, int((y0 - oy) / res)), min(h, int((y1 - oy) / res))
        blocked[j0:j1, i0:i1] = True
        free[j0:j1, i0:i1] = False
    centre_ok = free & ~dilate(blocked, disc_offsets(robot_radius, res))
    sx = int(math.floor((seed_xy[0] - ox) / res))
    sy = int(math.floor((seed_xy[1] - oy) / res))
    sy = min(max(sy, 0), occ.shape[0] - 1)
    sx = min(max(sx, 0), occ.shape[1] - 1)
    centres = flood(centre_ok, (sy, sx))
    reachable = dilate(centres, disc_offsets(brush_radius, res)) & free
    return {"free": free, "centre_ok": centre_ok, "centres": centres, "reachable": reachable}


def main():
    ap = argparse.ArgumentParser()
    ap.add_argument("run_dir")
    ap.add_argument("--robot-radius", type=float, default=0.30)
    ap.add_argument("--brush-radius", type=float, default=0.32)
    ap.add_argument("--plot", action="store_true")
    a = ap.parse_args()
    run = os.path.realpath(os.path.expanduser(a.run_dir))

    s = json.load(open(os.path.join(run, "summary.json")))
    occ = np.load(os.path.join(run, "map.npy"))
    passes = np.load(os.path.join(run, "passes.npy"))
    res = s["map"]["resolution"]
    origin = s["map"]["origin"]
    cell_m2 = res * res

    seed = (0.0, 0.0)                                 # the dock, unless the trail says otherwise
    trail = os.path.join(run, "trail.csv")
    if os.path.exists(trail):
        with open(trail) as f:
            rows = csv.reader(f)
            next(rows, None)
            first = next(rows, None)
            if first:
                seed = (float(first[2]), float(first[3]))

    m = analyse(occ, res, origin, seed, a.robot_radius, a.brush_radius)
    free_n = int(m["free"].sum())
    reach_n = int(m["reachable"].sum())
    lost = m["free"] & ~m["reachable"]
    painted = passes > 0
    painted_n = int(painted.sum())
    outside = int((painted & ~m["reachable"]).sum())

    print(f"\nrun            {run}")
    print(f"seed (dock)    map ({seed[0]:.2f}, {seed[1]:.2f})   robot r={a.robot_radius} m, brush r={a.brush_radius} m")
    print(f"\nmapped floor   {free_n:>7} cells  {free_n * cell_m2:7.1f} m2   <- what both % numbers use today")
    print(f"unreachable    {int(lost.sum()):>7} cells  {int(lost.sum()) * cell_m2:7.1f} m2   "
          f"({100.0 * lost.sum() / free_n:.1f}% of it: rack insides + sealed pockets + edges)")
    print(f"REACHABLE      {reach_n:>7} cells  {reach_n * cell_m2:7.1f} m2   <- the honest denominator")
    print(f"\npainted        {painted_n:>7} cells  {painted_n * cell_m2:7.1f} m2")
    print(f"  vs mapped floor     {100.0 * painted_n / free_n:5.1f}%")
    print(f"  vs REACHABLE floor  {100.0 * painted_n / reach_n:5.1f}%   <- quote this one")
    print(f"\nceiling        a perfect run can only reach {100.0 * reach_n / free_n:.1f}% "
          f"of \"mapped floor\" -- that is the plateau, not the planner")
    print(f"               (rack interiors are excluded from both: solid in "
          f"Gazebo, hollow in /map)")
    if outside:
        print(f"note           {outside} painted cells sit outside the reachable set "
              f"({outside * cell_m2:.2f} m2) -- brush overhang at edges, expected to be small")

    if a.plot:
        import matplotlib
        matplotlib.use("Agg")
        import matplotlib.pyplot as plt
        h, w = occ.shape
        ext = [origin[0], origin[0] + w * res, origin[1], origin[1] + h * res]
        img = np.ones((h, w, 3))
        img[occ != 0] = (0.85, 0.85, 0.85)
        img[occ >= 50] = (0.25, 0.25, 0.25)
        img[lost] = (1.0, 0.92, 0.6)                  # floor that was never reachable
        img[painted] = (0.17, 0.42, 1.0)
        img[passes >= 2] = (0.61, 0.24, 1.0)
        img[passes >= 3] = (1.0, 0.24, 0.5)
        img[passes >= 4] = (1.0, 0.16, 0.0)
        fig, ax = plt.subplots(figsize=(6, 7.5), dpi=150)
        ax.imshow(img, origin="lower", extent=ext, interpolation="nearest")
        ax.set_xlabel("map x (m)")
        ax.set_ylabel("map y (m)")
        ax.set_title(f"{os.path.basename(run)}: {100.0 * painted_n / reach_n:.1f}% of reachable floor\n"
                     f"yellow = floor that is unreachable (counted by today's %)", fontsize=9)
        out = os.path.join(run, "reachable_heatmap.png")
        fig.tight_layout()
        fig.savefig(out)
        print(f"\nwrote {out}")


if __name__ == "__main__":
    main()
