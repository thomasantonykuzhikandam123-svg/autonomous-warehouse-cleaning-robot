"""
coverage_meter.py -- the measuring half of coverage_recorder, with no ROS import.

PassGrid   counts how many separate times each floor cell has been under the brush.
DriveMeter turns the pose stream into distance, time moving, and how much of that
           driving swept floor that was already clean.

Kept ROS-free (same idea as mission_command.py) so the tests run anywhere:
    python3 test_coverage_meter.py
Written for the VM's Python 3.10 / numpy 1.21.
"""
import math

import numpy as np


class PassGrid:
    """Pass counts on the same grid as /map.

    A cell gets a new pass when it comes under the brush after being out from under
    it for more than `pass_gap` seconds. Sitting still, wobbling, or the brush
    overlapping itself in a turn is one pass. Driving back over the cell later is
    another.
    """

    def __init__(self, width, height, resolution, origin_x, origin_y, occupancy,
                 clean_width=0.52, pass_gap=3.0):
        self.w = int(width)
        self.h = int(height)
        self.res = float(resolution)
        self.ox = float(origin_x)
        self.oy = float(origin_y)
        self.clean_width = float(clean_width)
        self.pass_gap = float(pass_gap)

        occ = np.asarray(occupancy, dtype=np.int16).reshape(self.h, self.w)  # [row=y, col=x]
        self.floor = occ == 0            # free cells only: walls, racks, unknown never count
        self.floor_cells = int(np.count_nonzero(self.floor))

        self.passes = np.zeros((self.h, self.w), dtype=np.uint16)
        self.last_t = np.full((self.h, self.w), -np.inf)

        # Brush footprint: a disc of diameter clean_width, stored as cell offsets.
        r = self.clean_width / 2.0
        n = int(math.ceil(r / self.res))
        dy, dx = np.mgrid[-n:n + 1, -n:n + 1]
        inside = (dx * self.res) ** 2 + (dy * self.res) ** 2 <= r * r
        self._dx = dx[inside]
        self._dy = dy[inside]
        self.footprint_cells = int(self._dx.size)
        # New cells a fresh straight run sweeps per metre (brush width in cells / cell size).
        self.cells_per_metre = int(self._dx.max() - self._dx.min() + 1) / self.res

    def cell_of(self, x, y):
        """Map-frame metres -> (col, row) cell index."""
        return (int(math.floor((x - self.ox) / self.res)),
                int(math.floor((y - self.oy) / self.res)))

    def paint(self, t, x, y):
        """Brush centred at map (x, y) at time t. Returns how many never-swept cells it hit."""
        cx, cy = self.cell_of(x, y)
        xs = cx + self._dx
        ys = cy + self._dy
        ok = (xs >= 0) & (xs < self.w) & (ys >= 0) & (ys < self.h)
        xs, ys = xs[ok], ys[ok]
        ok = self.floor[ys, xs]
        xs, ys = xs[ok], ys[ok]
        if xs.size == 0:
            return 0
        first_ever = int(np.count_nonzero(self.passes[ys, xs] == 0))
        new_pass = (t - self.last_t[ys, xs]) > self.pass_gap
        self.passes[ys[new_pass], xs[new_pass]] += 1
        self.last_t[ys, xs] = t
        return first_ever

    def stats(self):
        painted = int(np.count_nonzero(self.passes))
        total = int(self.passes.sum(dtype=np.int64))
        return {
            "floor_cells": self.floor_cells,
            "painted_cells": painted,
            "painted_pct_of_floor": round(100.0 * painted / self.floor_cells, 2) if self.floor_cells else 0.0,
            "total_passes": total,
            "overlap_ratio": round(total / painted, 3) if painted else 0.0,
            "cells_2plus_passes": int(np.count_nonzero(self.passes >= 2)),
            "cells_3plus_passes": int(np.count_nonzero(self.passes >= 3)),
            "max_passes": int(self.passes.max()),
        }

    def heat_values(self):
        """int8 values for an OccupancyGrid drawn with RViz's costmap colour scheme.

        0 = never swept (transparent), 1 pass = 30 (blue), 2 = 60, 3 = 90, 4+ = 98 (red).
        Capped at 98 because that palette paints 99 cyan and 100 purple.
        """
        return np.minimum(self.passes.astype(np.int32) * 30, 98).astype(np.int8)


class DriveMeter:
    """Distance, time moving, and how much of that driving found no new floor.

    Driving is judged in 1 s bins. A bin counts as unproductive when the brush found
    less than `productive_frac` of the new floor a fresh straight run would have found
    over the same distance, so a few-cm sliver along an old lane isn't counted as work.

    sweep_efficiency = new floor found / floor the brush passed over
                       (1.0 = every metre driven cleaned fresh floor).
    """

    def __init__(self, cells_per_metre, footprint_cells=0, moving_speed=0.05,
                 bin_s=1.0, productive_frac=0.10):
        self.cells_per_metre = float(cells_per_metre)
        self.footprint_cells = int(footprint_cells)
        self.moving_speed = float(moving_speed)
        self.bin_s = float(bin_s)
        self.productive_frac = float(productive_frac)
        self.last = None               # (t, x, y)
        self.distance_m = 0.0
        self.moving_s = 0.0
        self.unproductive_s = 0.0
        self.new_cells = 0
        self._bin_start = None
        self._bin_moving = 0.0
        self._bin_dist = 0.0
        self._bin_new = 0

    def update(self, t, x, y, new_cells):
        if self.last is None:
            self._bin_start = t
        else:
            lt, lx, ly = self.last
            dt = t - lt
            if 0.0 < dt <= 1.0:        # longer gaps = missing data: count nothing
                d = math.hypot(x - lx, y - ly)
                if d / dt >= self.moving_speed:
                    self.distance_m += d
                    self._bin_moving += dt
                    self._bin_dist += d
        self.new_cells += int(new_cells)
        self._bin_new += int(new_cells)
        self.last = (t, x, y)
        if t - self._bin_start >= self.bin_s:
            self._close_bin(t)

    def _close_bin(self, t):
        self.moving_s += self._bin_moving
        expected = self._bin_dist * self.cells_per_metre
        if self._bin_moving > 0.0 and self._bin_new < self.productive_frac * expected:
            self.unproductive_s += self._bin_moving
        self._bin_start = t
        self._bin_moving = 0.0
        self._bin_dist = 0.0
        self._bin_new = 0

    def flush(self):
        """Close the last partial bin (call once at the end of a run)."""
        if self.last is not None and (self._bin_moving > 0.0 or self._bin_new):
            self._close_bin(self.last[0])

    def stats(self):
        swept = self.distance_m * self.cells_per_metre + self.footprint_cells
        return {
            "distance_m": round(self.distance_m, 2),
            "moving_s": round(self.moving_s, 1),
            "unproductive_moving_s": round(self.unproductive_s, 1),
            "unproductive_fraction": round(self.unproductive_s / self.moving_s, 3) if self.moving_s else 0.0,
            "sweep_efficiency": round(self.new_cells / swept, 3) if self.distance_m > 0 else 0.0,
        }