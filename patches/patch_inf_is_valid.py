#!/usr/bin/env python3
"""patch_inf_is_valid.py - let 'no return' lidar rays clear the costmaps.

Evidence (2026-09-25, worker_test crossing aisle 1):
  * From spawn, 47 of 360 /scan rays are +inf. That matches the geometry:
    about 22 rays up aisle 1 (back wall 16 m away) and 25 along the front
    corridor (right wall 11 m away). The lidar's range_max is 8 m.
  * Nav2's obstacle layer drops +inf rays by default (inf_is_valid: false),
    so those rays clear nothing. The middle of aisle 1, seen from spawn or
    from the goal, lies on exactly those rays: once the worker walks off,
    her marks there are never cleared. RPP stops for ghosts until a
    recovery wipes the costmap, and the planner routes around them.
  * Baseline in ~/runs/people_trials.csv: 14 recoveries in 6 passes, and
    2 of the 3 return legs detoured through aisle 2.

Change (nothing else):
  global_costmap ... obstacle_layer.scan.inf_is_valid: true
  local_costmap  ... obstacle_layer.scan.inf_is_valid: true

Why these rays can only clear, never mark:
  Nav2 turns +inf into range_max - 0.1 mm = 7.9999 m. Marking stops at
  obstacle_max_range (6.0 global, 3.5 local), so a point at 7.9999 m is
  never marked. -inf (closer than range_min) and NaN are still dropped.

Safety: each anchor must match exactly once or nothing is written. The YAML
is parsed before and after; the only allowed difference is the two new keys.
Backup: nav2_params.before_inf_is_valid.yaml, next to this script.
Undo:   cp that backup back over config/nav2_params.yaml
"""
import copy
import difflib
import re
import sys
from pathlib import Path

import yaml

TARGET = Path.home() / 'ros2_ws/src/warehouse_cleaning_robot/config/nav2_params.yaml'
BACKUP = Path(__file__).resolve().parent / 'nav2_params.before_inf_is_valid.yaml'

GLOBAL_NOTE = [
    '# inf_is_valid: a ray that sees nothing within range_max (8 m) arrives',
    '# as +inf, and by default Nav2 drops it, so it clears nothing. Along an',
    "# aisle most rays are inf, so a worker's marks there were never cleared",
    '# after she walked off (ghosts, 2026-09-25). true turns inf into a free',
    '# ray to 7.9999 m: it clears, and it can never mark, because',
    '# obstacle_max_range 6.0 < 8.0.',
]
LOCAL_NOTE = [
    '# inf_is_valid: see global_costmap. Clears only: 3.5 < 8.0, never marks.',
]

# (costmap, the obstacle_max_range value that identifies its scan block, comment)
EDITS = [
    ('global_costmap', '6.0', GLOBAL_NOTE),
    ('local_costmap', '3.5', LOCAL_NOTE),
]


def fail(msg):
    print(f'ABORT: {msg}\nNothing was written.')
    sys.exit(1)


def scan_block(data, costmap):
    try:
        block = data[costmap][costmap]['ros__parameters']['obstacle_layer']['scan']
    except (KeyError, TypeError):
        fail(f'{costmap} -> obstacle_layer -> scan not found in {TARGET}')
    if not isinstance(block, dict):
        fail(f'{costmap} -> obstacle_layer -> scan is not a block')
    return block


def main():
    if not TARGET.is_file():
        fail(f'{TARGET} not found')
    old = TARGET.read_text()
    try:
        before = yaml.safe_load(old)
    except yaml.YAMLError as e:
        fail(f'the file does not parse as YAML before patching: {e}')

    have = {c: scan_block(before, c).get('inf_is_valid') for c, _, _ in EDITS}
    if all(v is True for v in have.values()):
        print('Already applied: both scan blocks have inf_is_valid: true. Nothing to do.')
        return
    if any(v is not None for v in have.values()):
        fail(f'inf_is_valid is already partly set ({have}). Check the file by hand.')

    new = old
    for costmap, max_range, note in EDITS:
        # obstacle_max_range: <value>, then obstacle_min_range: 0.0 on the next
        # line at the same indentation. Trailing spaces are tolerated.
        pattern = re.compile(
            r'^([ \t]*)obstacle_max_range:[ \t]*' + re.escape(max_range) + r'[ \t]*\n'
            r'\1obstacle_min_range:[ \t]*0\.0[ \t]*$', re.M)
        hits = list(pattern.finditer(new))
        if len(hits) != 1:
            fail(f'{costmap} anchor (obstacle_max_range: {max_range}) matched '
                 f'{len(hits)} times, need exactly 1')
        m = hits[0]
        indent = m.group(1)
        insert = ''.join(f'\n{indent}{line}' for line in note)
        insert += f'\n{indent}inf_is_valid: true'
        new = new[:m.end()] + insert + new[m.end():]

    try:
        after = yaml.safe_load(new)
    except yaml.YAMLError as e:
        fail(f'the patched text would not parse: {e}')
    expected = copy.deepcopy(before)
    for costmap, _, _ in EDITS:
        scan_block(expected, costmap)['inf_is_valid'] = True
    if after != expected:
        fail('the patched YAML differs from the original by more than the two new keys')

    BACKUP.write_text(old)
    TARGET.write_text(new)

    sys.stdout.writelines(difflib.unified_diff(
        old.splitlines(keepends=True), new.splitlines(keepends=True),
        fromfile='nav2_params.yaml (before)', tofile='nav2_params.yaml (after)', n=2))
    print(f'\nOK: inf_is_valid: true added to both costmaps. Backup: {BACKUP}')
    print('Restart T4 to load it. No rebuild: the installed file links to src.')


if __name__ == '__main__':
    main()
