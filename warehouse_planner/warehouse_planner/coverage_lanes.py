"""
coverage_lanes.py -- the "what is still dirty in this aisle" maths, with no ROS.

coverage_nav2 already knows exactly which floor it has painted: self.visited, a
set of 0.25 m cells in the MAP frame. build_mission never reads it, so every
`clean aisle_1` rebuilds all five lanes end to end, whatever was swept a minute
ago. That is the restart, and it is most of the wasted driving in every run so
far.

This module answers one question and nothing else:

    given a lane and the painted cells, which parts of it still need sweeping?

Everything here is map-frame metres and ROS-free, so it is testable on its own:
    python3 test_coverage_lanes.py

Written for the VM's Python 3.10.
"""
import math

CELL = 0.25          # must match coverage_nav2.cov_cell
GROW = 0.20          # overlap back into painted floor, so joins leave no sliver
MIN_LEN = 0.50       # a shorter remnant costs more in turns than it cleans


def _cell_of(v, cell=CELL):
    return int(math.floor(v / cell))


def remaining_segments(mx, y_lo, y_hi, visited, cell=CELL, grow=GROW,
                       min_len=MIN_LEN):
    """Unswept spans of the lane at map x=mx, between y_lo and y_hi.

    The lane's CENTRE cells are the signal. Lanes are 0.515 m apart and the
    brush reaches 0.32 m, so a neighbouring lane can never paint this lane's
    centre line: only this lane, or a transit that drove along it, can.

    Returns [(y_start, y_end), ...] ascending, already grown and merged.
    """
    if y_hi < y_lo:
        y_lo, y_hi = y_hi, y_lo
    i = _cell_of(mx, cell)
    j0, j1 = _cell_of(y_lo, cell), _cell_of(y_hi, cell)

    runs, start = [], None
    for j in range(j0, j1 + 1):
        if (i, j) in visited:
            if start is not None:
                runs.append((start, j - 1))
                start = None
        elif start is None:
            start = j
    if start is not None:
        runs.append((start, j1))

    segs = []
    for a, b in runs:
        y0 = max(y_lo, a * cell) - grow
        y1 = min(y_hi, (b + 1) * cell) + grow
        y0, y1 = max(y_lo, y0), min(y_hi, y1)
        if y1 - y0 >= min_len:
            segs.append((round(y0, 3), round(y1, 3)))

    merged = []
    for s in segs:                       # growth can make neighbours overlap
        if merged and s[0] <= merged[-1][1]:
            merged[-1] = (merged[-1][0], max(merged[-1][1], s[1]))
        else:
            merged.append(s)
    return merged


def lane_plan(lanes, y_lo, y_hi, visited, start_at_bottom, robot_y=None,
              cell=CELL, grow=GROW, min_len=MIN_LEN):
    """Boustrophedon waypoints over the parts of `lanes` that are still dirty.

    Same shape as coverage_nav2.sweep_lanes, and with an empty `visited` it
    returns exactly what that function returns today -- a fresh mission is
    unchanged. Lanes with nothing left are skipped, and the snake only flips
    direction on lanes it actually sweeps, so skipping never adds travel.

    robot_y, when given, chooses the entry end of the FIRST swept lane: the
    end closer to the robot.

    Returns (waypoints, ended_at_bottom). An empty list means the zone is
    already clean -- say so instead of sending an empty mission.
    """
    work = [(x, remaining_segments(x, y_lo, y_hi, visited, cell, grow, min_len))
            for x in lanes]
    work = [(x, segs) for x, segs in work if segs]
    if not work:
        return [], start_at_bottom

    going_up = start_at_bottom
    if robot_y is not None:
        first = work[0][1]
        lo_end, hi_end = first[0][0], first[-1][1]
        going_up = abs(robot_y - lo_end) <= abs(robot_y - hi_end)

    wp = []
    for x, segs in work:
        ordered = segs if going_up else list(reversed(segs))
        for y0, y1 in ordered:
            a, b = (y0, y1) if going_up else (y1, y0)
            wp.append((x, a))
            wp.append((x, b))
        going_up = not going_up          # flip only when a lane was swept
    return wp, abs(wp[-1][1] - y_lo) < abs(wp[-1][1] - y_hi)


def segment_runs(p0, p1, visited, cell=CELL, grow=GROW, min_len=MIN_LEN, step=None):
    """v10.4: the still-dirty stretches of the straight drive p0 -> p1 (map frame).

    coverage_nav2 puts a waypoint off when someone stands in its lane, and comes
    back for it before going home. By then part of that drive may be painted
    (the robot got half-way, or a later pass crossed it), so only what is still
    dirty along the centre line is driven again: [(a, b), ...] point pairs in the
    direction p0 -> p1, grown by `grow` into painted floor, merged, and without
    stretches shorter than min_len (a turn costs more than they clean). The
    same centre-line rule as remaining_segments, for any direction. A drive of
    (almost) no length is one point: [(p0, p0)] if its cell is dirty, else [].
    """
    x0, y0 = float(p0[0]), float(p0[1])
    x1, y1 = float(p1[0]), float(p1[1])
    L = math.hypot(x1 - x0, y1 - y0)
    if L < 1e-6:
        return [] if (_cell_of(x0, cell), _cell_of(y0, cell)) in visited else [((x0, y0), (x0, y0))]
    step = step or cell / 4.0
    n = max(1, int(math.ceil(L / step)))
    ss = [L * k / n for k in range(n + 1)]
    dirty = [(_cell_of(x0 + (x1 - x0) * s / L, cell), _cell_of(y0 + (y1 - y0) * s / L, cell))
             not in visited for s in ss]
    runs, start = [], None
    for s, d in zip(ss, dirty):
        if d and start is None:
            start = s
        elif not d and start is not None:
            runs.append([start, s])
            start = None
    if start is not None:
        runs.append([start, L])
    grown = []
    for a, b in runs:
        a, b = max(0.0, a - grow), min(L, b + grow)
        if grown and a <= grown[-1][1]:
            grown[-1][1] = max(grown[-1][1], b)
        else:
            grown.append([a, b])
    out = []
    for a, b in grown:
        if b - a >= min_len:
            out.append(((round(x0 + (x1 - x0) * a / L, 3), round(y0 + (y1 - y0) * a / L, 3)),
                        (round(x0 + (x1 - x0) * b / L, 3), round(y0 + (y1 - y0) * b / L, 3))))
    return out


def zone_progress(lanes, y_lo, y_hi, visited, cell=CELL):
    """(swept_cells, total_cells) along the lane centres -- for honest replies."""
    total = swept = 0
    for x in lanes:
        i = _cell_of(x, cell)
        for j in range(_cell_of(y_lo, cell), _cell_of(y_hi, cell) + 1):
            total += 1
            if (i, j) in visited:
                swept += 1
    return swept, max(1, total)
