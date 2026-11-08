#!/usr/bin/env python3
"""check_wiring.py (v7) - is the running system wired the way the guard needs?

Expected (nav2_people.launch.py v7):
  /cmd_vel_raw      published by velocity_smoother (it no longer publishes /cmd_vel)
  /cmd_vel_safe_in  published by guard_gate only (the fail-safe gate)
  /cmd_vel          published by collision_monitor
  /scan_nav         read by both costmaps (local_costmap, global_costmap)
  /scan             read by collision_monitor and guard_gate, and by neither costmap
  /speed_limit      read by controller_server (the guard's speed limit near people)
behavior_server may also publish /cmd_vel directly: in Nav2 Humble its recovery
moves (spin, back up) bypass the smoother. The stop-and-wait trees never use
them (they only wait), so that is fine.
Prints OK or what is wrong; exit code 0 / 1. stack.sh runs it after 'start ... guard'.
"""
import sys
import time

import rclpy
from rclpy.node import Node

ALLOWED_EXTRA = {'behavior_server'}
COSTMAPS = {'local_costmap', 'global_costmap'}


def verdict(out, gated, raw, scan_subs, nav_subs, limit_subs):
    """(ok, message) from node names: publishers of /cmd_vel, /cmd_vel_safe_in and
    /cmd_vel_raw, subscribers of /scan, /scan_nav and /speed_limit."""
    problems, notes = [], []
    if not ('velocity_smoother' in raw and 'velocity_smoother' not in out
            and 'velocity_smoother' not in gated):
        problems.append(f'velocity_smoother should publish only /cmd_vel_raw; /cmd_vel_raw publishers '
                        f'{sorted(raw) or "none"}, /cmd_vel_safe_in {sorted(gated) or "none"}')
    if gated != {'guard_gate'}:
        problems.append(f'/cmd_vel_safe_in publishers {sorted(gated) or "none"} '
                        f'(only guard_gate should publish it)')
    if 'collision_monitor' not in out:
        problems.append(f'/cmd_vel publishers {sorted(out) or "none"} (collision_monitor missing)')
    extra = sorted(out - {'collision_monitor', 'velocity_smoother'} - ALLOWED_EXTRA)
    if extra:
        notes.append(f'{", ".join(extra)} also publish /cmd_vel directly (bypassing the monitor)')
    if not COSTMAPS <= nav_subs or scan_subs & COSTMAPS:
        problems.append(f'costmaps reading /scan_nav: {sorted(nav_subs & COSTMAPS) or "none"}, '
                        f'still reading raw /scan: {sorted(scan_subs & COSTMAPS) or "none"}')
    if 'collision_monitor' not in scan_subs:
        problems.append('collision_monitor is not reading the raw /scan')
    if 'guard_gate' not in scan_subs:
        problems.append('guard_gate is not reading /scan (it must see the lidar is alive)')
    if 'controller_server' not in limit_subs:
        problems.append(f'controller_server is not reading /speed_limit (readers: '
                        f'{sorted(limit_subs) or "none"}), so the guard could not slow it near people')
    if problems:
        return False, ('  WIRING WRONG: ' + '\n                '.join(problems) +
                       '\n  The guard cannot work like this. Send Claude these lines '
                       'and nav2_slam.launch.py.')
    msg = ('  wiring OK: velocity_smoother -> guard_gate -> collision_monitor -> /cmd_vel\n'
           '  wiring OK: costmaps read /scan_nav; collision_monitor and guard_gate read /scan\n'
           '  wiring OK: controller_server reads /speed_limit')
    for n in notes:
        msg += f'\n  note: {n}'
    return True, msg


def main():
    rclpy.init()
    node = Node('check_wiring')
    ok, msg = False, ''
    deadline = time.monotonic() + 8.0
    while time.monotonic() < deadline:
        rclpy.spin_once(node, timeout_sec=0.2)

        def pubs(topic):
            return {i.node_name for i in node.get_publishers_info_by_topic(topic)}

        def subs(topic):
            return {i.node_name for i in node.get_subscriptions_info_by_topic(topic)}
        ok, msg = verdict(pubs('/cmd_vel'), pubs('/cmd_vel_safe_in'), pubs('/cmd_vel_raw'),
                          subs('/scan'), subs('/scan_nav'), subs('/speed_limit'))
        if ok:
            break
    node.destroy_node()
    rclpy.shutdown()
    print(msg)
    return 0 if ok else 1


if __name__ == '__main__':
    sys.exit(main())
