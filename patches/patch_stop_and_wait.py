#!/usr/bin/env python3
"""patch_stop_and_wait.py - when a person blocks the path: stop, wait, go on.

Evidence (inf_clean run, 2026-09-25, 12 passes past the crossing worker):
  * The robot stopped for her ("collision ahead") in 8 of 12 passes. That part
    is right.
  * But Nav2 only lets the controller be blocked for failure_tolerance = 0.3 s.
    After that the goal is aborted and the behaviour tree's recovery moves run,
    in turn: clear the costmaps, spin 90 deg, wait 5 s, back up 0.3 m. This
    happened in 5 of 12 passes: a spin in pass 3 and a reverse in pass 7, both
    right next to her.
  * A forklift driver never does that. WorkSafe Victoria's traffic management
    guidance lists "eliminating the need to reverse"; ISO 3691-4 asks the truck
    to stop before contact with a person. So: stop and wait.

Change (nothing else):
  controller_server  failure_tolerance 0.3 -> 5.0
      blocked for up to 5 s: the robot holds still and carries on by itself as
      soon as the way is clear (she needs 1-2 s to cross its lane)
  bt_navigator       default_nav_to_pose_bt_xml / default_nav_through_poses_bt_xml
      Nav2's own default trees with ONE change: the recovery moves are "wait
      2 s, clear the local costmap" instead of clear / spin / wait / back up.
      Local only: wiping the global costmap cannot free a stuck robot and
      blinds the planner for an update cycle (the coverage rescue-routine
      lesson). After 6 tries the goal fails and the mission decides.

The two tree files are written next to nav2_params.yaml, in config/behavior_trees/.
Safety: every anchor must match exactly once or nothing is written. The YAML is
parsed before and after; the only allowed differences are the three values.
Backup: nav2_params.before_stop_and_wait.yaml, next to this script.
Undo:   cp that backup back over config/nav2_params.yaml, restart T4.
"""
import copy
import difflib
import re
import sys
from pathlib import Path

import yaml

PKG = Path.home() / 'ros2_ws/src/warehouse_cleaning_robot'
TARGET = PKG / 'config/nav2_params.yaml'
BT_DIR = PKG / 'config/behavior_trees'
BACKUP = Path(__file__).resolve().parent / 'nav2_params.before_stop_and_wait.yaml'
TOLERANCE = 5.0

HOLD_AND_CLEAR = '''      <ReactiveFallback name="RecoveryFallback">
        <GoalUpdated/>
        <Sequence name="HoldAndClear">
          <Wait wait_duration="2"/>
          <ClearEntireCostmap name="ClearLocalCostmap-Subtree" service_name="local_costmap/clear_entirely_local_costmap"/>
        </Sequence>
      </ReactiveFallback>'''

HEADER = '''<!--
  {name}  (warehouse_cleaning_robot)
  Nav2 Humble's {orig}
  with ONE change, the recovery moves. Default: clear the costmaps, spin 90 deg,
  wait 5 s, back up 0.3 m, in turn. Next to a person, spinning and reversing
  are what a driver must not do. Here the robot holds still for 2 s, clears
  its local costmap and tries again (not the global one: wiping it only
  blinds the planner). After 6 tries the goal fails and the mission layer
  decides (skip the aisle, come back later).
-->'''

TREES = {
    'stop_and_wait_to_pose.xml': HEADER.format(
        name='stop_and_wait_to_pose.xml',
        orig='navigate_to_pose_w_replanning_and_recovery.xml') + '''
<root main_tree_to_execute="MainTree">
  <BehaviorTree ID="MainTree">
    <RecoveryNode number_of_retries="6" name="NavigateRecovery">
      <PipelineSequence name="NavigateWithReplanning">
        <RateController hz="1.0">
          <RecoveryNode number_of_retries="1" name="ComputePathToPose">
            <ComputePathToPose goal="{goal}" path="{path}" planner_id="GridBased"/>
            <ClearEntireCostmap name="ClearGlobalCostmap-Context" service_name="global_costmap/clear_entirely_global_costmap"/>
          </RecoveryNode>
        </RateController>
        <RecoveryNode number_of_retries="1" name="FollowPath">
          <FollowPath path="{path}" controller_id="FollowPath"/>
          <ClearEntireCostmap name="ClearLocalCostmap-Context" service_name="local_costmap/clear_entirely_local_costmap"/>
        </RecoveryNode>
      </PipelineSequence>
''' + HOLD_AND_CLEAR + '''
    </RecoveryNode>
  </BehaviorTree>
</root>
''',
    'stop_and_wait_through_poses.xml': HEADER.format(
        name='stop_and_wait_through_poses.xml',
        orig='navigate_through_poses_w_replanning_and_recovery.xml') + '''
<root main_tree_to_execute="MainTree">
  <BehaviorTree ID="MainTree">
    <RecoveryNode number_of_retries="6" name="NavigateRecovery">
      <PipelineSequence name="NavigateWithReplanning">
        <RateController hz="0.333">
          <RecoveryNode number_of_retries="1" name="ComputePathThroughPoses">
            <ReactiveSequence>
              <RemovePassedGoals input_goals="{goals}" output_goals="{goals}" radius="0.7"/>
              <ComputePathThroughPoses goals="{goals}" path="{path}" planner_id="GridBased"/>
            </ReactiveSequence>
            <ClearEntireCostmap name="ClearGlobalCostmap-Context" service_name="global_costmap/clear_entirely_global_costmap"/>
          </RecoveryNode>
        </RateController>
        <RecoveryNode number_of_retries="1" name="FollowPath">
          <FollowPath path="{path}" controller_id="FollowPath"/>
          <ClearEntireCostmap name="ClearLocalCostmap-Context" service_name="local_costmap/clear_entirely_local_costmap"/>
        </RecoveryNode>
      </PipelineSequence>
''' + HOLD_AND_CLEAR + '''
    </RecoveryNode>
  </BehaviorTree>
</root>
''',
}
BT_KEYS = {'default_nav_to_pose_bt_xml': 'stop_and_wait_to_pose.xml',
           'default_nav_through_poses_bt_xml': 'stop_and_wait_through_poses.xml'}
# BT plugin libraries the two trees need, if the params list them explicitly
NEEDED_LIBS = ['nav2_compute_path_to_pose_action_bt_node',
               'nav2_compute_path_through_poses_action_bt_node',
               'nav2_follow_path_action_bt_node', 'nav2_wait_action_bt_node',
               'nav2_clear_costmap_service_bt_node', 'nav2_goal_updated_condition_bt_node',
               'nav2_rate_controller_bt_node', 'nav2_recovery_node_bt_node',
               'nav2_pipeline_sequence_bt_node', 'nav2_remove_passed_goals_action_bt_node']


def fail(msg):
    print(f'ABORT: {msg}\nNothing was written.')
    sys.exit(1)


def params(data, node):
    try:
        block = data[node]['ros__parameters']
    except (KeyError, TypeError):
        fail(f'{node} -> ros__parameters not found in {TARGET}')
    if not isinstance(block, dict):
        fail(f'{node} -> ros__parameters is not a block')
    return block


def block_span(lines, node):
    """(first, end) line indexes of '<node>:' at column 0 up to the next top-level key."""
    starts = [i for i, l in enumerate(lines) if re.match(rf'^{re.escape(node)}:\s*(#.*)?$', l)]
    if len(starts) != 1:
        fail(f'"{node}:" found {len(starts)} times at the start of a line, need exactly 1')
    first = starts[0]
    end = len(lines)
    for i in range(first + 1, len(lines)):
        if re.match(r'^[^\s#]', lines[i]):
            end = i
            break
    return first, end


def main():
    if not TARGET.is_file():
        fail(f'{TARGET} not found')
    old = TARGET.read_text()
    try:
        before = yaml.safe_load(old)
    except yaml.YAMLError as e:
        fail(f'the file does not parse as YAML before patching: {e}')

    ctrl, bt = params(before, 'controller_server'), params(before, 'bt_navigator')
    want_bt = {k: str(BT_DIR / f) for k, f in BT_KEYS.items()}
    trees_ok = all((BT_DIR / f).is_file() and (BT_DIR / f).read_text() == TREES[f]
                   for f in TREES)
    if (ctrl.get('failure_tolerance') == TOLERANCE
            and all(bt.get(k) == v for k, v in want_bt.items()) and trees_ok):
        print('Already applied: tolerance 5.0 and both stop-and-wait trees. Nothing to do.')
        return
    if not isinstance(ctrl.get('failure_tolerance'), (int, float)):
        fail('controller_server has no numeric failure_tolerance to change')
    libs = bt.get('plugin_lib_names')
    if libs is not None:
        missing = [n for n in NEEDED_LIBS if n not in libs]
        if missing:
            fail(f'bt_navigator plugin_lib_names lacks {missing}; the trees need them')

    lines = old.splitlines(keepends=True)

    # 1. failure_tolerance inside controller_server
    first, end = block_span(lines, 'controller_server')
    hits = [i for i in range(first, end)
            if re.match(r'^\s+failure_tolerance:\s*[-+0-9.eE]+\s*(#.*)?$', lines[i])]
    if len(hits) != 1:
        fail(f'failure_tolerance matched {len(hits)} times in controller_server, need 1')
    i = hits[0]
    indent = re.match(r'^(\s+)', lines[i]).group(1)
    lines[i] = (f'{indent}failure_tolerance: {TOLERANCE}  # was '
                f'{ctrl["failure_tolerance"]}: hold up to 5 s for a person (stop-and-wait)\n')

    # 2. behaviour tree paths inside bt_navigator
    first, end = block_span(lines, 'bt_navigator')
    rp = [i for i in range(first, end) if re.match(r'^\s+ros__parameters:\s*(#.*)?$', lines[i])]
    if len(rp) != 1:
        fail(f'bt_navigator -> ros__parameters matched {len(rp)} times, need 1')
    key_lines = [i for i in range(rp[0] + 1, end) if re.match(r'^\s+[A-Za-z_]\w*:', lines[i])]
    if not key_lines:
        fail('bt_navigator -> ros__parameters has no keys')
    key_indent = re.match(r'^(\s+)', lines[key_lines[0]]).group(1)
    inserts = []
    for key, path in want_bt.items():
        hits = [i for i in range(rp[0] + 1, end)
                if re.match(rf'^{key_indent}{key}:', lines[i])]
        new_line = f'{key_indent}{key}: "{path}"\n'
        if len(hits) > 1:
            fail(f'{key} appears {len(hits)} times in bt_navigator')
        if hits:
            i = hits[0]
            simple = re.match(
                rf'^{key_indent}{key}:\s*("[^"]*"|\'[^\']*\'|[^\s#>|&*!\[{{][^\s#]*)?\s*(#.*)?$',
                lines[i])
            nxt = next((l for l in lines[i + 1:end] if l.strip()), '')
            continued = len(nxt) - len(nxt.lstrip()) > len(key_indent)
            if not simple or continued:
                fail(f'{key} is not a simple one-line value; edit it by hand')
            lines[i] = new_line
        else:
            inserts.append(new_line)
    if inserts:
        note = (f'{key_indent}# stop-and-wait trees (patch_stop_and_wait.py): Nav2 defaults '
                f'with recovery = wait 2 s + clear local costmap, no spin, no reversing\n')
        lines[rp[0] + 1:rp[0] + 1] = [note] + inserts
    new = ''.join(lines)

    try:
        after = yaml.safe_load(new)
    except yaml.YAMLError as e:
        fail(f'the patched text would not parse: {e}')
    expected = copy.deepcopy(before)
    params(expected, 'controller_server')['failure_tolerance'] = TOLERANCE
    params(expected, 'bt_navigator').update(want_bt)
    if after != expected:
        fail('the patched YAML differs from the original by more than the three values')

    BT_DIR.mkdir(parents=True, exist_ok=True)
    for f, text in TREES.items():
        (BT_DIR / f).write_text(text)
    if not BACKUP.exists():            # keep the oldest copy: it is the true "before"
        BACKUP.write_text(old)
    TARGET.write_text(new)

    sys.stdout.writelines(difflib.unified_diff(
        old.splitlines(keepends=True), new.splitlines(keepends=True),
        fromfile='nav2_params.yaml (before)', tofile='nav2_params.yaml (after)', n=1))
    print(f'\nWrote {", ".join(TREES)} in {BT_DIR}')
    print(f'OK: stop-and-wait applied. Backup: {BACKUP}')
    print('Restart T4 (stack.sh stop, stack.sh start) to load it. No rebuild needed.')


if __name__ == '__main__':
    main()
