#!/usr/bin/env python3
"""
patch_intelligence_2.py -- fixes the two things patch 1 broke, plus the stall
detector that was fighting Nav2's own recovery.

    cd ~/ros2_ws/src/warehouse_planner/warehouse_planner
    python3 ~/patch_intelligence_2.py

Every anchor must appear EXACTLY ONCE or nothing is written.

coverage_nav2.py
  1. The stall detector now watches rotation as well as translation, and waits
     15 s instead of 10. Evidence: behavior_server ran spin (5 s) then wait
     (5 s) from 1790132065 to 075, and the detector fired at 075.6 -- it had
     timed Nav2's own recovery and cancelled a goal Nav2 was already fixing.
     A spin is movement; treating it as a wedge put three recovery systems
     (Nav2's, the stall detector's, the rescue ladder's) on the same event.
  2. Per-zone percentages no longer persist. Patch 1 made the loaders work for
     the first time and they promptly restored a stale .pct: the robot said
     "aisle 3 at about 100 percent" in a run that never entered aisle 3.
     Finished zones still persist; progress inside a run does not.

robot_agent.py
  3. _zone_gap asks the robot which zone it is in before doing any geometry.
     /mission_state already publishes it. My box check alone put the lane-end
     turn at y=14.04 outside aisle_1's box (which ended at 13.90), so the
     reply said "aisle 1 (6.5 m)" while the robot was in aisle 1. The boxes
     stay as the fallback and now run y 0.80-14.40.
  4. The nearest/farthest reply says "I'm in it already" instead of "0.0 m",
     and states the restart: "Restarting it from the beginning; it was about
     80 percent done."
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
if 'STALL_TURN' in cn.text:
    sys.exit('already patched -- nothing changed')

# 1a. a turning robot is a moving robot
cn.once("""        self.STALL_SECONDS = 10.0
        self.STALL_DIST = 0.12""",
        """        self.STALL_SECONDS = 15.0    # a full Nav2 recovery (backup + spin +
        #   wait) runs ~12 s; 10 s fired in the middle of one
        self.STALL_DIST = 0.12
        self.STALL_TURN = 0.25       # rad. Nav2's spin recovery turns 1.57 on
        #   the spot: translation alone reads that as a wedged robot""")

# 1b. read yaw alongside the position
cn.once("""            x = t.transform.translation.x
            y = t.transform.translation.y
        except Exception:
            return""",
        """            x = t.transform.translation.x
            y = t.transform.translation.y
            _q = t.transform.rotation
            yaw = math.atan2(2.0 * (_q.w * _q.z + _q.x * _q.y),
                             1.0 - 2.0 * (_q.y * _q.y + _q.z * _q.z))
        except Exception:
            return""")

# 1c. and use it
cn.once("""            if self._stall_ref is None:
                self._stall_ref = (x, y, now_s)
            rx, ry, rt = self._stall_ref
            if math.hypot(x - rx, y - ry) > self.STALL_DIST:
                self._stall_ref = (x, y, now_s)
            elif now_s - rt > self.STALL_SECONDS:""",
        """            if self._stall_ref is None or len(self._stall_ref) != 4:
                self._stall_ref = (x, y, yaw, now_s)
            rx, ry, rw, rt = self._stall_ref
            turned = abs((yaw - rw + math.pi) % (2.0 * math.pi) - math.pi)
            if (math.hypot(x - rx, y - ry) > self.STALL_DIST
                    or turned > self.STALL_TURN):
                self._stall_ref = (x, y, yaw, now_s)
            elif now_s - rt > self.STALL_SECONDS:""")

# 2. percentages describe one run's floor: never load them, never save them
cn.once("""    def _load_pct(self):
        try:
            with open(self.CLEANED_FILE + '.pct') as f:
                v = json.load(f)
            return {k: int(x) for k, x in v.items()
                    if k in ('front', 'aisle_1', 'aisle_2', 'aisle_3', 'back')}
        except Exception:
            return {}""",
        '''    def _load_pct(self):
        """Always empty, and the stale file is removed.

        Per-zone percentages are a statement about the floor RIGHT NOW. Patch 1
        fixed the loader and it immediately restored a .pct from an older
        session, so the robot reported aisle 3 at 100 percent in a run that
        never entered aisle 3. Finished zones persist; progress does not.
        """
        try:
            os.remove(self.CLEANED_FILE + '.pct')
        except OSError:
            pass
        return {}''')

cn.once("""            with open(self.CLEANED_FILE, 'w') as f:
                json.dump(self.cleaned, f)
            with open(self.CLEANED_FILE + '.pct', 'w') as f:
                json.dump(self.zone_pct, f)""",
        """            with open(self.CLEANED_FILE, 'w') as f:
                json.dump(self.cleaned, f)""")

# ====================================================================  agent
ra = Patch(RA)

# 3. ask the robot which zone it is in, before any geometry
ra.once('''def _zone_gap(z, x, y):
    """Distance from (x, y) to zone z. ZERO when the robot is standing in it.

    Centre distance alone told the operator "aisle 3 is 4.8 m away" while the
    robot was driving down aisle 3. Outside a zone the old centre metric is
    kept on purpose: box distance would call the next aisle 1 m away when it
    is a 13 m drive around the rack. Travel distance comes with the roadmap.
    """
    b = ZONE_BOX.get(z)
    if b is not None and b[0] <= x <= b[1] and b[2] <= y <= b[3]:
        return 0.0
    cx, cy = ZONE_CENTRE[z]
    return math.hypot(cx - x, cy - y)''',
        '''def _zone_gap(z, x, y, here=None):
    """Distance from (x, y) to zone z. ZERO when the robot is standing in it.

    `here` is robot["zone"] from /mission_state -- the robot's own answer to
    "which zone am I working in", which beats re-deriving it from coordinates.
    The box is only the fallback: with boxes alone, the lane-end turn at
    y=14.04 fell outside aisle_1 (1.10-13.90) and the reply said "aisle 1
    (6.5 m)" while the robot was in aisle 1.

    Outside a zone the old centre metric is kept on purpose: box distance
    would call the next aisle 1 m away when it is a 13 m drive around the
    rack. Travel distance comes with the roadmap.
    """
    if here and z == here:
        return 0.0
    b = ZONE_BOX.get(z)
    if b is not None and b[0] <= x <= b[1] and b[2] <= y <= b[3]:
        return 0.0
    cx, cy = ZONE_CENTRE[z]
    return math.hypot(cx - x, cy - y)''')

# 3b. the aisle boxes cover the turn points at each lane end
ra.once('''    "aisle_1": (-1.94,  0.96,  1.10, 13.90),
    "aisle_2": (2.06,   4.96,  1.10, 13.90),
    "aisle_3": (6.06,   9.01,  1.10, 13.90),''',
        '''    "aisle_1": (-1.94,  0.96,  0.80, 14.40),   # 0.80/14.40, not 1.10/13.90:
    "aisle_2": (2.06,   4.96,  0.80, 14.40),   #   the sweep turns past the
    "aisle_3": (6.06,   9.01,  0.80, 14.40),   #   lane ends, out to y~14.1''')

# 3c. every caller passes it
ra.once("""    cleaned = set(robot.get("cleaned") or [])
    blocked = set(robot.get("blocked") or [])
    out = []
    for z in ZONE_CENTRE:""",
        """    cleaned = set(robot.get("cleaned") or [])
    blocked = set(robot.get("blocked") or [])
    here = robot.get("zone")
    out = []
    for z in ZONE_CENTRE:""")
ra.once("""        out.append((_zone_gap(z, x, y), z))
    return sorted(out)""",
        """        out.append((_zone_gap(z, x, y, here), z))
    return sorted(out)""")
ra.once("""        c = sorted((_zone_gap(z, x, y), z) for z in named if z not in blk)""",
        """        c = sorted((_zone_gap(z, x, y, robot.get("zone")), z)
                   for z in named if z not in blk)""")
ra.once("""            r = sorted((_zone_gap(c, x, y), c) for c in cands)""",
        """            r = sorted((_zone_gap(c, x, y, (robot or {}).get("zone")), c)
                       for c in cands)""")
ra.once("""        d = sorted(((_zone_gap(z, x, y), z) for z in ZONE_CENTRE),
                   key=lambda t: t[0])""",
        """        d = sorted(((_zone_gap(z, x, y, robot.get("zone")), z)
                    for z in ZONE_CENTRE), key=lambda t: t[0])""")

# 4. no more "0.0 m", and say the restart out loud
ra.once('''    dist, pick = r[0] if near else r[-1]
    if not re.match(r"^(clean|do|go|start|move|head)\\b", t, re.I):
        return True, None, (f"The {'nearest' if near else 'farthest'} "
                            f"{'uncleaned ' if dirty else ''}"
                            f"{'aisle' if aisles_only else 'zone'} is {name(pick)}, "
                            f"about {dist:.1f} m away.")
    return True, {"action": "clean", "zones": [pick],
                  "reason": f"{'nearest' if near else 'farthest'} "
                            f"{'uncleaned ' if dirty else ''}"
                            f"{'aisle' if aisles_only else 'zone'}, {dist:.1f} m"}, \\
        f"The {'nearest' if near else 'farthest'} {'uncleaned ' if dirty else ''}" \\
        f"{'aisle' if aisles_only else 'zone'} is {name(pick)} ({dist:.1f} m). Going there."''',
        '''    dist, pick = r[0] if near else r[-1]
    kind = (f"{'nearest' if near else 'farthest'} "
            f"{'uncleaned ' if dirty else ''}"
            f"{'aisle' if aisles_only else 'zone'}")
    where = "I'm in it already" if dist == 0.0 else f"about {dist:.1f} m away"
    if not re.match(r"^(clean|do|go|start|move|head)\\b", t, re.I):
        return True, None, f"The {kind} is {name(pick)}, {where}."
    part = (robot.get("partial") or {}).get(pick)
    extra = (f" Restarting it from the beginning; it was about {part} percent "
             f"done." if part else "")
    return True, {"action": "clean", "zones": [pick],
                  "reason": f"{kind}, {dist:.1f} m"}, \\
        f"The {kind} is {name(pick)} ({where}). Going there.{extra}"''')

cn.save()
ra.save()
print()
print('now run:  python3 -m py_compile coverage_nav2.py robot_agent.py')
print('          python3 -m pyflakes coverage_nav2.py robot_agent.py')
print('          python3 agent_logic_test.py')
