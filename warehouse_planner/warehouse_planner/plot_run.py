#!/usr/bin/env python3
"""
plot_run.py -- pass-count heatmap for one recorded run (the before/after thesis figure).

    python3 plot_run.py ~/runs/latest
    python3 plot_run.py ~/runs/2026-09-22_183000 --title "Baseline run 1"

Writes heatmap.png into the run folder. Needs matplotlib
(sudo apt install python3-matplotlib, or copy the run folder to the Mac).
"""
import argparse
import json
import os

import matplotlib
matplotlib.use("Agg")
import matplotlib.pyplot as plt
import numpy as np
from matplotlib.colors import BoundaryNorm, ListedColormap

PASS_COLOURS = ["#2b6cff", "#9b3dff", "#ff3d7f", "#ff2a00"]   # 1, 2, 3, 4+ passes


def plot(occ, passes, resolution, origin, title, out_path, driving=None, coverage=None):
    h, w = occ.shape
    ext = [origin[0], origin[0] + w * resolution, origin[1], origin[1] + h * resolution]
    base = np.ones((h, w, 3))
    base[occ != 0] = (0.85, 0.85, 0.85)                 # unknown: light grey
    base[occ >= 50] = (0.25, 0.25, 0.25)                # walls and racks: dark grey

    fig, ax = plt.subplots(figsize=(6, 7.5), dpi=150)
    ax.imshow(base, origin="lower", extent=ext, interpolation="nearest")
    shown = np.ma.masked_equal(np.minimum(passes, 4), 0)
    cmap = ListedColormap(PASS_COLOURS)
    norm = BoundaryNorm([0.5, 1.5, 2.5, 3.5, 4.5], cmap.N)
    im = ax.imshow(shown, origin="lower", extent=ext, cmap=cmap, norm=norm,
                   interpolation="nearest")
    cb = fig.colorbar(im, ax=ax, ticks=[1, 2, 3, 4], fraction=0.04, pad=0.02)
    cb.ax.set_yticklabels(["1 pass", "2", "3", "4+"])
    ax.set_xlabel("map x (m)")
    ax.set_ylabel("map y (m)")
    lines = [title]
    if coverage and driving:
        lines.append(f"overlap {coverage['overlap_ratio']:.2f}  |  "
                     f"sweep efficiency {driving['sweep_efficiency']:.2f}  |  "
                     f"unproductive driving {driving['unproductive_fraction']:.0%}")
    ax.set_title("\n".join(lines), fontsize=9)
    fig.tight_layout()
    fig.savefig(out_path)
    plt.close(fig)


def main():
    ap = argparse.ArgumentParser()
    ap.add_argument("run_dir")
    ap.add_argument("--title", default=None)
    ap.add_argument("--out", default=None)
    a = ap.parse_args()
    run = os.path.realpath(os.path.expanduser(a.run_dir))
    with open(os.path.join(run, "summary.json")) as f:
        s = json.load(f)
    occ = np.load(os.path.join(run, "map.npy"))
    passes = np.load(os.path.join(run, "passes.npy"))
    out = a.out or os.path.join(run, "heatmap.png")
    plot(occ, passes, s["map"]["resolution"], s["map"]["origin"],
         a.title or os.path.basename(run), out, s["driving"], s["coverage"])
    print(f"wrote {out}")


if __name__ == "__main__":
    main()