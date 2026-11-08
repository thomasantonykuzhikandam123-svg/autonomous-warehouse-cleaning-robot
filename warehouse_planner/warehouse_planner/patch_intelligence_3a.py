#!/usr/bin/env python3
"""
patch_intelligence_3a.py -- fixes the AttributeError patch 3 introduced.

    cd ~/ros2_ws/src/warehouse_planner/warehouse_planner
    python3 ~/patch_intelligence_3a.py

__init__ calls build_mission() at line ~152, but self.visited was created
further down, with the coverage marker. Before patch 3 that was fine, because
build_mission never read it. Now it does, so the node died on startup:

    AttributeError: 'CoverageNav2' object has no attribute 'visited'

The grid moves above the first build_mission() call. Nothing else changes: at
startup it is empty either way, so the opening 65-waypoint mission is the same.

(Note for later, not fixed here: self.tf_buffer is also created after that
first build_mission() call, so the _map_pose() inside it throws and is
swallowed, and the opening mission is built from spawn rather than from the
robot's pose. At startup they are the same point, so it has never mattered.)
"""
import sys

CN = 'coverage_nav2.py'
s = open(CN).read()

if 'created above, before the first build_mission' in s:
    sys.exit('already patched -- nothing changed')


def once(text, old, new):
    c = text.count(old)
    if c != 1:
        sys.exit(f'ABORT: anchor found {c} times, expected 1. Nothing written.'
                 f'\n---\n{old}\n---')
    return text.replace(old, new)


s = once(s, """        # ----- build the mission -----
        self.goals, self.goal_zones = self.build_mission()""",
         """        # The painted grid has to exist before the first mission is built:
        # since patch 3, build_mission reads it to work out what is still
        # dirty. It used to be created further down, after this call.
        self.visited = set()             # (cx, cy) map cells already swept
        # ----- build the mission -----
        self.goals, self.goal_zones = self.build_mission()""")

s = once(s, """        self.visited = set()             # set of (cx, cy) integer cell indices
        self._paint_last = None          # last painted pose -> paint the gap between""",
         """        # self.visited is created above, before the first build_mission()
        self._paint_last = None          # last painted pose -> paint the gap between""")

open(CN, 'w').write(s)
print('patched coverage_nav2.py (2 edits)')
print()
print('now run:  python3 -m py_compile coverage_nav2.py')
print('          ros2 run warehouse_planner coverage_nav2 --ros-args -p auto_start:=false')
print('  expect: COVERAGE v7-measured: 65 waypoints planned')
