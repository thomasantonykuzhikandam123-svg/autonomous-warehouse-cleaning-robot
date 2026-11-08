#!/usr/bin/env python3
"""
patch_intelligence_3.py -- the continue primitive.

    cd ~/ros2_ws/src/warehouse_planner/warehouse_planner
    python3 ~/patch_intelligence_3.py

Every anchor must appear EXACTLY ONCE or nothing is written.

coverage_nav2.py
  1. The aisle branch of build_mission sweeps only the parts of each lane that
     are still dirty, using coverage_lanes.lane_plan over self.visited. With
     nothing painted it produces exactly today's waypoints (proved by
     test_coverage_lanes.test_fresh_floor_matches_todays_sweep), so a fresh
     mission is unchanged.
  2. If the robot is already inside the aisle it is being asked to clean, the
     two transit waypoints are skipped and the entry end is chosen by the
     robot's own y -- no more driving out to the corridor and back in.
  3. A zone with nothing left is skipped and named in self._already_clean; a
     clean command for only such zones leaves the current mission alone
     instead of quietly building a drive home.

robot_agent.py
  4. Replies say "continuing" for an aisle and keep "restarting" for the front
     and back zones, which still rebuild in full.

FRAMES, the thing most likely to go wrong here: build_mission works in WORLD
coordinates and converts to map at the very end, while self.visited holds MAP
cells (map = world - spawn). The conversion happens immediately around the
lane_plan call and nowhere else.

Deferred on purpose: `partial` in /mission_state is still counted in waypoints,
not floor, so the percentage the robot quotes stays waypoint-based for now.
"""
import sys

CN = 'coverage_nav2.py'
RA = 'robot_agent.py'


class Patch:
    def __init__(self, path):
        self.path = path
        self.text = open(path).read()
        self.n = 0

    def once(self, old, new):
        c = self.text.count(old)
        if c != 1:
            sys.exit(f'ABORT in {self.path}: anchor found {c} times, expected 1. '
                     f'Nothing was written.\n---\n{old}\n---')
        self.text = self.text.replace(old, new)
        self.n += 1

    def save(self):
        open(self.path, 'w').write(self.text)
        print(f'patched {self.path} ({self.n} edits)')


# =====================================================================  nav2
cn = Patch(CN)
if 'lane_plan' in cn.text:
    sys.exit('already patched -- nothing changed')

cn.once("""from tf2_ros import Buffer, TransformListener""",
        """from tf2_ros import Buffer, TransformListener

try:
    from warehouse_planner.coverage_lanes import lane_plan
except ImportError:                      # running straight from the source folder
    from coverage_lanes import lane_plan""")

cn.once("""        self._zone_ok = set()            # zones with a success this mission""",
        """        self._zone_ok = set()            # zones with a success this mission
        self._already_clean = []         # zones build_mission found nothing left in""")

# 1 + 2. sweep what is still dirty, and don't leave the aisle to re-enter it
cn.once("""            idx = int(z.split('_')[1]) - 1
            if not (0 <= idx < len(self.aisles)):
                continue
            x_left, x_right = self.aisles[idx]
            lanes = self.aisle_lanes(x_left, x_right)
            transit_y = self.CORRIDOR_Y if enter_bottom else self.BACKSTRIP_Y
            world_wp.append((prev_x, transit_y, z))
            world_wp.append((lanes[0], transit_y, z))
            sweep, ended_bottom = self.sweep_lanes(lanes, enter_bottom)
            for p in sweep:
                world_wp.append((p[0], p[1], z))
            prev_x = sweep[-1][0]
            enter_bottom = ended_bottom""",
        """            idx = int(z.split('_')[1]) - 1
            if not (0 <= idx < len(self.aisles)):
                continue
            x_left, x_right = self.aisles[idx]
            lanes = self.aisle_lanes(x_left, x_right)

            # Only the dirty parts. lanes and sweeps are WORLD coordinates;
            # self.visited holds MAP cells, so convert into map for the filter
            # and straight back out. map = world - spawn.
            m_lanes = [lx - self.spawn_x for lx in lanes]
            m_lo = (self.rack_y_bot - self.END_MARGIN) - self.spawn_y
            m_hi = (self.rack_y_top + self.END_MARGIN) - self.spawn_y
            in_this_aisle = (here is not None
                             and x_left <= here[0] + self.spawn_x <= x_right)
            m_sweep, ended_bottom = lane_plan(
                m_lanes, m_lo, m_hi, self.visited, enter_bottom,
                robot_y=(here[1] if in_this_aisle else None))
            if not m_sweep:
                self._already_clean.append(z)
                self.get_logger().info(
                    f'{z}: already swept, nothing left to do')
                continue
            sweep = [(mx + self.spawn_x, my + self.spawn_y) for mx, my in m_sweep]

            # Standing in the aisle already? Then the corridor transit is pure
            # travel: go straight to the first thing that still needs sweeping.
            if not in_this_aisle:
                transit_y = self.CORRIDOR_Y if enter_bottom else self.BACKSTRIP_Y
                world_wp.append((prev_x, transit_y, z))
                world_wp.append((sweep[0][0], transit_y, z))
            for p in sweep:
                world_wp.append((p[0], p[1], z))
            prev_x = sweep[-1][0]
            enter_bottom = ended_bottom""")

# build_mission reports what it skipped, per call
cn.once("""        if zones is None:
            seq = list(ZONES)""",
        """        self._already_clean = []
        if zones is None:
            seq = list(ZONES)""")

# 3. a clean command for zones that are already done changes nothing
cn.once("""        goals, zlist = self.build_mission(list(zones))
        if not goals:
            self.get_logger().error(f'no waypoints for zones {zones}; ignoring')
            return""",
        """        goals, zlist = self.build_mission(list(zones))
        if not goals or not any(z in zones for z in zlist):
            done = [z for z in zones if z in self._already_clean]
            if done:
                self.get_logger().info(
                    f"{', '.join(done)} already swept -> nothing to do. The "
                    f"current mission is untouched.")
            else:
                self.get_logger().error(
                    f'no waypoints for zones {zones}; ignoring')
            return""")

# ====================================================================  agent
ra = Patch(RA)

# 4. an aisle is continued now; front and back are still rebuilt in full
ra.once('''            bits = []
            for z in zs:
                p = part.get(z)
                bits.append(nm(z) + (f" (restarting from the beginning; it was "
                                     f"about {p} percent done)" if p else ""))''',
        '''            bits = []
            for z in zs:
                p = part.get(z)
                if not p:
                    bits.append(nm(z))
                elif z.startswith("aisle_"):
                    bits.append(f"{nm(z)} (carrying on from where I stopped; "
                                f"about {p} percent was done)")
                else:
                    bits.append(f"{nm(z)} (restarting from the beginning; it "
                                f"was about {p} percent done)")''')

ra.once('''    part = (robot.get("partial") or {}).get(pick)
    extra = (f" Restarting it from the beginning; it was about {part} percent "
             f"done." if part else "")''',
        '''    part = (robot.get("partial") or {}).get(pick)
    if not part:
        extra = ""
    elif pick.startswith("aisle_"):
        extra = (f" Carrying on from where I stopped; about {part} percent was "
                 f"already done.")
    else:
        extra = (f" Restarting it from the beginning; it was about {part} "
                 f"percent done.")''')

cn.save()
ra.save()
print()
print('now run:  python3 -m py_compile coverage_nav2.py robot_agent.py')
print('          python3 -m pyflakes coverage_nav2.py robot_agent.py')
print('          python3 test_coverage_lanes.py')
print('          python3 agent_logic_test.py')

