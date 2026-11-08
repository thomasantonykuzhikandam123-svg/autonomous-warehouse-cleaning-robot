#!/usr/bin/env python3
"""
Tests for coverage_lanes.py. No ROS needed:  python3 test_coverage_lanes.py

The important one is test_fresh_floor_matches_todays_sweep: with nothing
painted, the filter must produce exactly the waypoints coverage_nav2 produces
today, so patch 3 cannot change a fresh mission.
"""
import math
import sys

try:
    from warehouse_planner.coverage_lanes import (CELL, lane_plan,
                                                  remaining_segments,
                                                  zone_progress)
except ImportError:
    from coverage_lanes import CELL, lane_plan, remaining_segments, zone_progress

# aisle 1 as coverage_nav2 builds it, in the MAP frame
LANES = [-1.52, -1.00, -0.49, 0.03, 0.54]
Y_LO, Y_HI = 1.10, 13.90


def sweep_lanes_today(lanes, start_at_bottom, y_bot=Y_LO, y_top=Y_HI):
    """A copy of coverage_nav2.sweep_lanes, to compare against."""
    wp, going_up = [], start_at_bottom
    for x in lanes:
        if going_up:
            wp.append((x, y_bot))
            wp.append((x, y_top))
        else:
            wp.append((x, y_top))
            wp.append((x, y_bot))
        going_up = not going_up
    return wp, abs(wp[-1][1] - y_bot) < 1e-6


def painted(x, y0, y1, cell=CELL):
    """The cells a sweep along lane x from y0 to y1 would mark."""
    if y1 < y0:
        y0, y1 = y1, y0
    i = int(math.floor(x / cell))
    return {(i, j) for j in range(int(math.floor(y0 / cell)),
                                  int(math.floor(y1 / cell)) + 1)}


def test_fresh_floor_matches_todays_sweep():
    for start_at_bottom in (True, False):
        mine, ended = lane_plan(LANES, Y_LO, Y_HI, set(), start_at_bottom)
        theirs, ended_t = sweep_lanes_today(LANES, start_at_bottom)
        assert mine == theirs, (start_at_bottom, mine[:4], theirs[:4])
        assert ended == ended_t, (start_at_bottom, ended, ended_t)


def test_a_finished_lane_is_skipped():
    v = painted(-1.52, Y_LO, Y_HI)
    wp, _ = lane_plan(LANES, Y_LO, Y_HI, v, True)
    assert all(abs(x - (-1.52)) > 1e-6 for x, _ in wp), wp[:4]
    assert wp[0] == (-1.00, Y_LO), wp[0]        # starts at the next lane


def test_run1_pause_leaves_only_the_unswept_part():
    """Lane 2 was swept 13.90 -> 8.63 before the stop."""
    v = painted(-1.00, 8.63, Y_HI)
    segs = remaining_segments(-1.00, Y_LO, Y_HI, v)
    assert len(segs) == 1, segs
    y0, y1 = segs[0]
    assert abs(y0 - Y_LO) < 0.01, segs
    assert 8.6 <= y1 <= 8.9, segs                # unswept part + 0.20 m overlap


def test_a_painted_middle_leaves_two_segments():
    v = painted(-0.49, 6.0, 9.0)
    segs = remaining_segments(-0.49, Y_LO, Y_HI, v)
    assert len(segs) == 2, segs
    assert segs[0][0] == Y_LO and abs(segs[1][1] - Y_HI) < 0.01, segs
    assert segs[0][1] < segs[1][0], segs


def test_a_sliver_is_not_worth_a_lane():
    v = painted(0.03, Y_LO, 13.60)               # 0.30 m left, under MIN_LEN
    assert remaining_segments(0.03, Y_LO, Y_HI, v) == []


def test_everything_done_returns_nothing():
    v = set()
    for x in LANES:
        v |= painted(x, Y_LO, Y_HI)
    wp, _ = lane_plan(LANES, Y_LO, Y_HI, v, True)
    assert wp == [], wp
    swept, total = zone_progress(LANES, Y_LO, Y_HI, v)
    assert swept == total, (swept, total)


def test_robot_position_picks_the_entry_end():
    v = painted(-1.52, Y_LO, Y_HI)               # lane 1 done, robot up top
    wp, _ = lane_plan(LANES, Y_LO, Y_HI, v, True, robot_y=13.5)
    assert wp[0] == (-1.00, Y_HI), wp[0]         # enters at the top, not 1.10
    wp, _ = lane_plan(LANES, Y_LO, Y_HI, v, True, robot_y=1.5)
    assert wp[0] == (-1.00, Y_LO), wp[0]


def test_skipping_a_lane_does_not_break_the_snake():
    v = painted(-1.00, Y_LO, Y_HI)               # middle lane of three done
    wp, _ = lane_plan(LANES[:3], Y_LO, Y_HI, v, True)
    assert wp == [(-1.52, Y_LO), (-1.52, Y_HI),
                  (-0.49, Y_HI), (-0.49, Y_LO)], wp


def test_partial_progress_is_measurable():
    v = painted(-1.52, Y_LO, Y_HI) | painted(-1.00, 8.63, Y_HI)
    swept, total = zone_progress(LANES, Y_LO, Y_HI, v)
    pct = 100.0 * swept / total
    assert 26 < pct < 34, pct                    # ~1.4 lanes of 5


if __name__ == "__main__":
    bad = 0
    for n, f in sorted(globals().items()):
        if n.startswith("test_") and callable(f):
            try:
                f()
                print("PASS", n)
            except AssertionError as e:
                bad += 1
                print("FAIL", n, "::", e)
    print("\n" + ("ALL GREEN" if not bad else f"{bad} FAILED"))
    sys.exit(1 if bad else 0)
