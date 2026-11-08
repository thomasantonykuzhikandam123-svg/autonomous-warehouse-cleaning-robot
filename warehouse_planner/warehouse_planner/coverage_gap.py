#!/usr/bin/env python3
"""
coverage_gap.py -- where did the unswept floor go?

    python3 coverage_gap.py ~/runs/full_1

recheck_run.py says how much reachable floor was missed. This says WHERE, split
into the regions of the warehouse, so you can tell "the planner skipped it" from
"the denominator is wrong".
"""
import argparse
import json
import os
import sys

import numpy as np

sys.path.insert(0, os.path.dirname(os.path.abspath(__file__)))
from recheck_run import analyse            # noqa: E402

RACKS = (-2.5, 1.5, 5.5, 9.5)              # rack centres, map x
RACK_Y = (1.5, 13.5)
AISLES = ((-1.94, 0.96), (2.06, 4.96), (6.06, 9.01))
AISLE_Y = (1.10, 13.90)
STRIPS = (("outer strip L", -4.0, -3.0), ("outer strip R", 10.0, 11.0))


def main():
    ap = argparse.ArgumentParser()
    ap.add_argument("run_dir")
    a = ap.parse_args()
    run = os.path.realpath(os.path.expanduser(a.run_dir))
    s = json.load(open(os.path.join(run, "summary.json")))
    occ = np.load(os.path.join(run, "map.npy"))
    passes = np.load(os.path.join(run, "passes.npy"))
    res = s["map"]["resolution"]
    ox, oy = s["map"]["origin"]
    cell = res * res

    m = analyse(occ, res, (ox, oy), (0.0, 0.0))
    reach = m["reachable"]
    painted = passes > 0
    h, w = occ.shape
    claimed = np.zeros((h, w), dtype=bool)

    def box(x0, x1, y0, y1):
        g = np.zeros((h, w), dtype=bool)
        i0, i1 = max(0, int((x0 - ox) / res)), min(w, int((x1 - ox) / res))
        j0, j1 = max(0, int((y0 - oy) / res)), min(h, int((y1 - oy) / res))
        g[j0:j1, i0:i1] = True
        return g

    rows = []

    def add(name, mask):
        mask = mask & ~claimed
        claimed[mask] = True
        r = int((reach & mask).sum())
        p = int((reach & mask & painted).sum())
        if r or p:
            rows.append((name, r, p))

    for i, cx in enumerate(RACKS, 1):
        add(f"rack {i} interior", box(cx - 0.5, cx + 0.5, *RACK_Y))
    for name, x0, x1 in STRIPS:
        add(name, box(x0, x1, *RACK_Y))
    for i, (x0, x1) in enumerate(AISLES, 1):
        add(f"aisle {i}", box(x0, x1, *AISLE_Y))
    add("front", box(-99, 99, -99, AISLE_Y[0]))
    add("back", box(-99, 99, AISLE_Y[1], 99))
    add("elsewhere", np.ones((h, w), dtype=bool))

    tot_r = int(reach.sum())
    tot_p = int((reach & painted).sum())
    print(f"\nrun   {run}")
    print(f"{'region':<18}{'reachable':>11}{'swept':>10}{'%':>7}{'unswept m2':>12}")
    print("-" * 58)
    for name, r, p in sorted(rows, key=lambda t: -(t[1] - t[2])):
        print(f"{name:<18}{r * cell:>9.1f} m2{p * cell:>8.1f} m2"
              f"{100.0 * p / r if r else 0:>6.0f}%{(r - p) * cell:>11.1f}")
    print("-" * 58)
    print(f"{'TOTAL':<18}{tot_r * cell:>9.1f} m2{tot_p * cell:>8.1f} m2"
          f"{100.0 * tot_p / tot_r if tot_r else 0:>6.1f}%{(tot_r - tot_p) * cell:>11.1f}")

    leak = sum(r for n, r, _ in rows if n.startswith("rack"))
    if leak:
        print(f"\nNOTE  {leak * cell:.1f} m2 of rack interior is being counted as "
              f"reachable.\n      That is a leak in the flood fill, not floor the "
              f"robot skipped:\n      excluding it, the run swept "
              f"{100.0 * tot_p / max(1, tot_r - leak):.1f}% of reachable floor.")


if __name__ == "__main__":
    main()
