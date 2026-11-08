#!/usr/bin/env python3
"""The world<->map conversion patch 3 puts into build_mission, tested on its own.

This is the bit most likely to be wrong: build_mission speaks WORLD, visited
speaks MAP (map = world - spawn).
"""
import math, sys
from coverage_lanes import lane_plan

SPAWN_X = SPAWN_Y = 4.0
RACK_Y_BOT, RACK_Y_TOP, END_MARGIN = 5.5, 17.5, 0.40
AISLE_1 = (2.06, 4.96)                  # world x bounds
WALL_MARGIN, COLUMN_SPACING = 0.42, 0.55


def aisle_lanes(x_left, x_right):       # copy of coverage_nav2.aisle_lanes
    lo, hi = x_left + WALL_MARGIN, x_right - WALL_MARGIN
    span = hi - lo
    n = max(2, int(math.ceil(span / COLUMN_SPACING)) + 1)
    return [round(lo + span * i / (n - 1), 3) for i in range(n)]


def branch(visited, here, enter_bottom=True):
    """The patched aisle branch, returning WORLD waypoints."""
    x_left, x_right = AISLE_1
    lanes = aisle_lanes(x_left, x_right)
    m_lanes = [lx - SPAWN_X for lx in lanes]
    m_lo = (RACK_Y_BOT - END_MARGIN) - SPAWN_Y
    m_hi = (RACK_Y_TOP + END_MARGIN) - SPAWN_Y
    in_this_aisle = here is not None and x_left <= here[0] + SPAWN_X <= x_right
    m_sweep, ended = lane_plan(m_lanes, m_lo, m_hi, visited, enter_bottom,
                               robot_y=(here[1] if in_this_aisle else None))
    if not m_sweep:
        return [], ended, in_this_aisle
    return ([(mx + SPAWN_X, my + SPAWN_Y) for mx, my in m_sweep],
            ended, in_this_aisle)


def painted(map_x, y0, y1, cell=0.25):
    if y1 < y0:
        y0, y1 = y1, y0
    i = int(math.floor(map_x / cell))
    return {(i, j) for j in range(int(math.floor(y0 / cell)),
                                  int(math.floor(y1 / cell)) + 1)}


def test_fresh_aisle_is_todays_mission():
    wp, ended, _ = branch(set(), here=(0.0, 0.0))
    # exactly the world waypoints the current sweep_lanes emits
    assert wp[0] == (2.48, 5.10) and wp[1] == (2.48, 17.90), wp[:2]
    assert wp[2] == (2.995, 17.90) and wp[3] == (2.995, 5.10), wp[2:4]
    assert len(wp) == 10, len(wp)
    assert ended is False, ended            # 5 lanes: the snake ends at the top


def test_map_cells_line_up_with_world_lanes():
    """A lane painted in MAP coords must silence the matching WORLD lane."""
    lanes = aisle_lanes(*AISLE_1)
    v = painted(lanes[0] - SPAWN_X, 1.10, 13.90)     # world 2.48 -> map -1.52
    wp, _, _ = branch(v, here=(0.0, 0.0))
    assert all(abs(x - 2.48) > 1e-6 for x, _ in wp), wp[:2]
    assert wp[0][0] == 2.995, wp[0]                  # next lane, in world


def test_run1_pause_continues_in_world_coords():
    lanes = aisle_lanes(*AISLE_1)                    # 2.48, 2.995, ...
    v = (painted(lanes[0] - SPAWN_X, 1.10, 13.90)
         | painted(lanes[1] - SPAWN_X, 8.63, 13.90))
    wp, _, inside = branch(v, here=(lanes[1] - SPAWN_X, 8.63))
    assert inside is True
    assert abs(wp[0][0] - 3.0) < 0.01, wp[0]         # world x of map -1.00
    assert 12.5 < wp[0][1] < 12.9, wp[0]             # world y of map ~8.7
    assert wp[1] == (2.995, 5.10), wp[1]             # sweeps DOWN to the bottom


def test_robot_outside_the_aisle_gets_no_robot_y():
    v = painted(aisle_lanes(*AISLE_1)[0] - SPAWN_X, 1.10, 13.90)
    wp, _, inside = branch(v, here=(7.50, 13.00))    # over in aisle 3
    assert inside is False
    assert wp[0] == (2.995, 5.10), wp[0]             # enter_bottom decides


def test_finished_aisle_gives_nothing():
    v = set()
    for lx in aisle_lanes(*AISLE_1):
        v |= painted(lx - SPAWN_X, 1.10, 13.90)
    wp, _, _ = branch(v, here=(0.0, 0.0))
    assert wp == [], wp


if __name__ == "__main__":
    bad = 0
    for n, f in sorted(globals().items()):
        if n.startswith("test_") and callable(f):
            try:
                f(); print("PASS", n)
            except AssertionError as e:
                bad += 1; print("FAIL", n, "::", e)
    print("\n" + ("ALL GREEN" if not bad else f"{bad} FAILED"))
    sys.exit(1 if bad else 0)
