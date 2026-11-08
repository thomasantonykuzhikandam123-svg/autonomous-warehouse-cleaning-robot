#!/usr/bin/env python3
"""
Tests for coverage_meter.py. No ROS needed:  python3 test_coverage_meter.py

The last test replays the 22 Sep session using the waypoints from the T6 log.
It shows the recorder flags what we found by hand: lane x=-1.52 swept four times,
and a big share of the driving painting nothing new.
"""
import math
import sys

import numpy as np

try:
    from warehouse_planner.coverage_meter import DriveMeter, PassGrid
except ImportError:
    from coverage_meter import DriveMeter, PassGrid

RES = 0.05
W, H = 299, 398
OX, OY = -4.0, -4.0            # map (0,0) = world (4,4); close enough for a synthetic grid


def warehouse_map():
    """Synthetic occupancy grid shaped like the real one: walls + 4 racks, 1 m wide."""
    occ = np.zeros((H, W), dtype=np.int8)
    occ[0, :] = occ[-1, :] = 100
    occ[:, 0] = occ[:, -1] = 100
    y0, y1 = int((1.0 - OY) / RES), int((14.0 - OY) / RES)      # racks span map y 1..14
    for wx in (1.5, 5.5, 9.5, 13.5):                             # rack centres, world x
        x0, x1 = int((wx - 0.5) / RES), int((wx + 0.5) / RES)    # map x - OX == world x
        occ[y0:y1, x0:x1] = 100
    return occ.ravel()


def fresh(clean_width=0.52, pass_gap=3.0):
    g = PassGrid(W, H, RES, OX, OY, warehouse_map(), clean_width, pass_gap)
    return g, DriveMeter(g.cells_per_metre, g.footprint_cells)


def drive(g, m, t, points, speed=0.5, hz=10.0):
    """Straight segments through `points` at `speed`, sampled at `hz`. Returns end time."""
    dt = 1.0 / hz
    if m.last is None:
        m.update(t, points[0][0], points[0][1], g.paint(t, *points[0]))
    for (x0, y0), (x1, y1) in zip(points, points[1:]):
        steps = max(1, int(round(math.hypot(x1 - x0, y1 - y0) / (speed * dt))))
        for i in range(1, steps + 1):
            t += dt
            x = x0 + (x1 - x0) * i / steps
            y = y0 + (y1 - y0) * i / steps
            m.update(t, x, y, g.paint(t, x, y))
    return t


def wait(g, m, t, x, y, seconds, hz=10.0):
    for _ in range(int(seconds * hz)):
        t += 1.0 / hz
        m.update(t, x, y, g.paint(t, x, y))
    return t


def test_fresh_lane_is_one_pass():
    g, m = fresh()
    drive(g, m, 0.0, [(-1.52, 1.10), (-1.52, 13.90)])
    m.flush()
    s, d = g.stats(), m.stats()
    assert s["max_passes"] == 1, s
    assert s["overlap_ratio"] == 1.0, s
    assert d["unproductive_fraction"] == 0.0, d
    assert 0.95 <= d["sweep_efficiency"] <= 1.05, d


def test_same_lane_twice_counts_two_passes_and_half_the_driving_is_waste():
    g, m = fresh()
    t = drive(g, m, 0.0, [(-1.52, 1.10), (-1.52, 13.90)])
    drive(g, m, t, [(-1.52, 13.90), (-1.52, 1.10)])
    m.flush()
    s, d = g.stats(), m.stats()
    assert s["max_passes"] == 2, s
    assert 1.8 <= s["overlap_ratio"] <= 2.0, s
    assert 0.45 <= d["unproductive_fraction"] <= 0.55, d
    assert 0.45 <= d["sweep_efficiency"] <= 0.55, d


def test_parked_and_wobbling_is_still_one_pass():
    g, m = fresh()
    t = wait(g, m, 0.0, 0.0, 0.0, 120.0)             # parked for 2 minutes
    assert m.stats()["moving_s"] == 0.0
    for i in range(200):                              # 2 cm wobble for 20 s
        t += 0.1
        x = 0.02 * math.sin(i)
        m.update(t, x, 0.0, g.paint(t, x, 0.0))
    assert g.stats()["max_passes"] == 1, g.stats()


def test_coming_back_after_the_gap_is_a_new_pass():
    g, m = fresh(pass_gap=3.0)
    t = drive(g, m, 0.0, [(0.0, 0.0), (2.0, 0.0)])   # leaves (0,0) behind
    t = wait(g, m, t, 2.0, 0.0, 5.0)                 # > pass_gap away
    drive(g, m, t, [(2.0, 0.0), (0.0, 0.0)])
    cx, cy = g.cell_of(0.0, 0.0)
    assert g.passes[cy, cx] == 2, g.passes[cy, cx]


def test_racks_and_walls_never_count():
    g, m = fresh()
    drive(g, m, 0.0, [(-2.5, 5.0), (-2.5, 8.0)])     # straight down the middle of rack 1
    assert g.stats()["painted_cells"] == 0


def test_heat_values_for_rviz():
    g, _ = fresh()
    g.passes[0, :6] = [0, 1, 2, 3, 4, 9]
    heat = g.heat_values()
    assert heat.dtype == np.int8
    assert heat[0, :6].tolist() == [0, 30, 60, 90, 98, 98]


def replay_22_sep():
    """Waypoints straight from the T6 log; motion between them idealised at 0.5 m/s."""
    g, m = fresh()
    t = 0.0
    # Mission 1: clean aisle_1, aisle_3 -> stopped halfway down lane 2
    t = drive(g, m, t, [(0.0, 0.0), (-1.52, 0.0), (-1.52, 1.10), (-1.52, 13.90),
                        (-1.00, 13.90), (-1.00, 8.63)])
    t = wait(g, m, t, -1.00, 8.63, 100.0)            # paused ~100 s
    # Mission 2: "so can you start cleaning" -> aisle_1 restarted from lane 1
    t = drive(g, m, t, [(-1.00, 8.63), (-1.05, 14.80), (-1.52, 14.80), (-1.52, 13.90),
                        (-1.52, 1.10), (-1.00, 1.10), (-1.00, 13.90), (-0.49, 13.90),
                        (-0.49, 2.45)])
    # Mission 3: "clean the nearest dirty aisle" -> aisle_1 restarted from lane 1
    t = drive(g, m, t, [(-0.49, 2.45), (-0.44, 0.0), (-1.52, 0.0), (-1.52, 1.10),
                        (-1.52, 13.90), (-1.00, 13.90), (-1.00, 1.10), (-0.49, 1.10),
                        (-0.49, 13.90), (0.03, 13.90), (0.03, 12.50)])
    # Mission 4: "start from that point" -> aisle_1 restarted from lane 1; Ctrl-C at y=4.33
    drive(g, m, t, [(0.03, 12.50), (0.01, 14.80), (-1.52, 14.80), (-1.52, 13.90),
                    (-1.52, 4.33)])
    m.flush()
    return g, m


def test_replay_22_sep_session():
    g, m = replay_22_sep()
    cx, cy = g.cell_of(-1.52, 8.0)
    assert g.passes[cy, cx] == 4, g.passes[cy, cx]    # lane x=-1.52: four sweeps
    d = m.stats()
    assert d["unproductive_fraction"] > 0.4, d
    print("    replay:", g.stats())
    print("    replay:", d)


if __name__ == "__main__":
    failed = 0
    for name, fn in list(globals().items()):
        if name.startswith("test_") and callable(fn):
            try:
                fn()
                print(f"PASS {name}")
            except AssertionError as e:
                failed += 1
                print(f"FAIL {name}: {e}")
    print(f"\n{'ALL GREEN' if not failed else f'{failed} FAILED'}")
    sys.exit(1 if failed else 0)