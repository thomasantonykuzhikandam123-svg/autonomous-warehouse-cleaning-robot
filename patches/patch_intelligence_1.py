#!/usr/bin/env python3
"""
patch_intelligence_1.py -- first intelligence patch, from the baseline-run logs.

Run it from the package folder:
    cd ~/ros2_ws/src/warehouse_planner/warehouse_planner
    python3 /tmp/patch_intelligence_1.py

Every anchor must appear EXACTLY ONCE in the file or nothing is written --
both files are written only after all anchors have matched.

coverage_nav2.py
  1. stop clears goal_deadline, and the watchdog ignores a paused robot
     (the goal index used to advance ~103 s into any pause).
  2. CLEANED_FILE is set BEFORE _load_cleaned()/_load_pct() run; it was set
     after, so both loads threw AttributeError into a bare except and the
     "memory survives a restart" feature never actually restored anything.
  3. a goal rejected by Nav2 no longer advances a mission that has since been
     replaced (it silently skipped the new mission's first waypoint).
  4. the operator log keeps the resolver's reason, so the robot can explain
     WHY it picked a zone ("nearest uncleaned aisle, 4.8 m").
  5. a zone counts as cleaned only when ~all of its waypoints succeeded, not
     after a single one.

robot_agent.py
  6. paused + "carry on"/"continue"/"start" with no zone named -> resume,
     decided in code. This is run 1's worst turn.
  7. "what have you cleaned" is answered from state, not by the model.
  8. a zone the robot is standing in is 0 m away, everywhere distances are
     used (ranking, the fact sheet, nearest_of references).
  9. after a command, the robot's reply is built from the VALIDATED steps.
     The model's sentence is kept in the notes, so /dialogue records both and
     you can measure how often they disagreed.
 10. the planner prompt learns that resume exists.
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
if 'goal_deadline = None        # a paused robot' in cn.text:
    sys.exit('already patched -- nothing changed')

# 1a. stop clears the per-goal deadline
cn.once("""            self._paused = True
            self._epoch += 1                       # orphan any in-flight goal
            self._cancel_current()
            self.get_logger().warn('PAUSED. Send resume to continue.')""",
        """            self._paused = True
            self._epoch += 1                       # orphan any in-flight goal
            self._cancel_current()
            self.goal_deadline = None        # a paused robot has no deadline;
            #   leaving it set let the watchdog fire mid-pause, log
            #   'TIMEOUT (no active goal) -> advancing' and step gi forward,
            #   so a resume silently skipped the rest of the lane.
            self.get_logger().warn('PAUSED. Send resume to continue.')""")

# 1b. and the watchdog never runs while paused
cn.once("""        if self.goal_deadline is None or self._done or self._rescue_phase:
            return""",
        """        if (self.goal_deadline is None or self._done or self._paused
                or self._rescue_phase):
            return""")

# 2. the cleaned-zone memory can actually load
cn.once("""        self.cleaned = self._load_cleaned()   # zones finished, in order""",
        """        # Set BEFORE the two loads below. It used to be assigned ~30 lines
        # further down, so _load_cleaned() and _load_pct() hit AttributeError,
        # returned the empty default, and nothing was ever restored.
        self.CLEANED_FILE = os.path.expanduser('~/.ros/warehouse_cleaned.json')
        self.cleaned = self._load_cleaned()   # zones finished, in order""")
cn.once("""        # cleaned zones survive a restart, so "clean whatever is left" still
        # means something after the node is bounced
        self.CLEANED_FILE = os.path.expanduser('~/.ros/warehouse_cleaned.json')""",
        """        # cleaned zones survive a restart, so "clean whatever is left" still
        # means something after the node is bounced (CLEANED_FILE is set above,
        # before the loads that need it)""")

# 3. a stale rejection must not advance the current mission
cn.once("""    def goal_response(self, fut, epoch=0):
        gh = fut.result()
        if not gh.accepted:""",
        """    def goal_response(self, fut, epoch=0):
        gh = fut.result()
        if epoch != self._epoch:
            return          # superseded mission: its rejection used to call
            #   advance() below and skip the NEW mission's first waypoint
        if not gh.accepted:""")

# 4. keep the reason with the command, so the robot can explain itself
cn.once("""            self.command_log.append(
                f"{action} {', '.join(zones)}".strip())""",
        """            self.command_log.append(
                (f"{action} {', '.join(zones)}".strip()
                 + (f" [{reason}]" if reason else "")))""")

# 5. "cleaned" means finished, not "one waypoint worked"
cn.once("""            for z in self.goal_zones:
                if z in self._zone_ok and z not in self.cleaned:""",
        """            for z in self.goal_zones:
                _done = self._zone_done.get(z, 0)
                _tot = self._zone_total.get(z, 0)
                if (z in self._zone_ok and z not in self.cleaned
                        and _tot and _done * 10 >= _tot * 9):""")

# ====================================================================  agent
ra = Patch(RA)

# 8a. zone boxes: standing inside a zone is zero distance
ra.once("""ALL_ZONES = ["front", "aisle_1", "aisle_2", "aisle_3", "back"]""",
        '''ALL_ZONES = ["front", "aisle_1", "aisle_2", "aisle_3", "back"]

ZONE_BOX = {                       # map frame (x0, x1, y0, y1): each zone's floor
    "front":   (-3.60, 10.60, -3.20,  1.00),
    "aisle_1": (-1.94,  0.96,  1.10, 13.90),
    "aisle_2": (2.06,   4.96,  1.10, 13.90),
    "aisle_3": (6.06,   9.01,  1.10, 13.90),
    "back":    (-3.60, 10.60, 14.00, 15.30),
}


def _zone_gap(z, x, y):
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
    return math.hypot(cx - x, cy - y)


# "carry on" while paused is a resume, not a new mission. Kept deliberately
# narrow: a named zone, "again"/"over", or a robot that is not paused all fall
# through to the planner.
_CONTINUE = re.compile(
    r"^(carry on|continue|keep going|keep cleaning|keep at it|go on|go ahead|"
    r"proceed|resume|get going|start cleaning|start|finish it|finish up)\\b", re.I)
_RESTART = re.compile(r"\\b(over|from the start|from scratch|again)\\b", re.I)
_WHAT_DONE = re.compile(
    r"^(what|how much|how many)\\b.*\\b(clean|cleaned|done|finished|swept|covered)\\b",
    re.I)
_ABOUT_LEFT = re.compile(r"\\b(left|remaining|still|to go|next|yet to)\\b", re.I)''')

# 8b. ranking uses it
ra.once("""    out = []
    for z, (cx, cy) in ZONE_CENTRE.items():
        if aisles_only and not z.startswith("aisle_"):
            continue
        if dirty_only and z in cleaned:
            continue
        if z in blocked:
            continue
        out.append((math.hypot(cx - x, cy - y), z))
    return sorted(out)""",
        """    out = []
    for z in ZONE_CENTRE:
        if aisles_only and not z.startswith("aisle_"):
            continue
        if dirty_only and z in cleaned:
            continue
        if z in blocked:
            continue
        out.append((_zone_gap(z, x, y), z))
    return sorted(out)""")

# 8c. "front or back, whichever is nearer" uses it
ra.once("""        c = sorted((math.hypot(ZONE_CENTRE[z][0] - x, ZONE_CENTRE[z][1] - y), z)
                   for z in named if z not in blk)""",
        """        c = sorted((_zone_gap(z, x, y), z) for z in named if z not in blk)""")

# 8d. the planner's nearest_of:/farthest_of: references use it
ra.once("""            r = sorted((math.hypot(ZONE_CENTRE[c][0] - x,
                                   ZONE_CENTRE[c][1] - y), c) for c in cands)""",
        """            r = sorted((_zone_gap(c, x, y), c) for c in cands)""")

# 8e. and so does the fact sheet the model reads
ra.once("""        d = sorted(((math.hypot(cx - x, cy - y), z)
                    for z, (cx, cy) in ZONE_CENTRE.items()), key=lambda t: t[0])
        L.append("Straight-line distance from the robot: \"""",
        """        d = sorted(((_zone_gap(z, x, y), z) for z in ZONE_CENTRE),
                   key=lambda t: t[0])
        L.append("Distance from the robot, where 0 m means it is standing in "
                 "that zone: \"""")

# 7b. and the fact sheet stops inviting "nothing yet"
ra.once("""         f"Cleaned so far: {', '.join(name(z) for z in cleaned) if cleaned else 'none yet'}.",""",
        """         f"Zones finished end to end: "
         f"{', '.join(name(z) for z in cleaned) if cleaned else 'none yet -- but read the part-cleaned line before saying nothing has been done'}.",""")

# 6 + 7. the two deterministic answers, at the very top of resolve_with_state
ra.once('''    t = strip_fillers(text or "")
    robot = robot or {}
    name = lambda z: ZONE_NAMES.get(z, z)

    # ---- conditional on the blocked set ----------------------------------''',
        '''    t = strip_fillers(text or "")
    robot = robot or {}
    name = lambda z: ZONE_NAMES.get(z, z)

    # ---- "carry on" while paused is a RESUME, never a new mission ---------
    # Run 1: paused mid aisle 1, "ok carry on" went to the planner, which
    # planned clean [aisle_3] and said "Continuing with the back area" -- a
    # different zone from the one it named, and the queued aisle_1 was lost.
    if (robot.get("paused") and _CONTINUE.match(t) and not _zones_in(t)
            and not _RESTART.search(t)):
        zone = name(robot.get("zone") or "")
        g, tot = robot.get("goal"), robot.get("total")
        where = f" ({g} of {tot}, in {zone})" if g and tot and zone else ""
        return True, {"action": "resume", "zones": [],
                      "reason": "told to continue while paused"}, \\
            f"Resuming from where I stopped{where}."

    # ---- "what have you cleaned" is a state lookup, not a language problem -
    # It answered "Nothing yet" while 3.3% of the floor was swept and aisle 1
    # was 36% done, because the model read the cleaned-zone line alone.
    if _WHAT_DONE.match(t) and not _ABOUT_LEFT.search(t):
        fin = [z for z in (robot.get("cleaned") or []) if z in ALL_ZONES]
        part = {z: p for z, p in (robot.get("partial") or {}).items()
                if z in ALL_ZONES and z not in fin}
        bits = ["Finished: " + ", ".join(name(z) for z in fin) + "."
                if fin else "Nothing finished end to end yet."]
        if part:
            bits.append("Part-way through " + ", ".join(
                f"{name(z)} at about {p} percent" for z, p in part.items()) + ".")
        pct = robot.get("coverage_pct")
        if pct is not None:
            bits.append(f"{pct} percent of the floor overall.")
        return True, None, " ".join(bits)

    # ---- conditional on the blocked set ----------------------------------''')

# 9. the robot's reply after a command is built from the validated steps
ra.once("""def build_graph(llm: Callable[[str, str], str]):""",
        '''def confirm(steps, robot):
    """What the robot says after a command, built from the VALIDATED steps.

    The model's own sentence is never used here. In run 1 it wrote "Continuing
    with the back area" over an accepted clean [aisle_3]: the plan was valid,
    so every existing check passed, and only the words were wrong. Those words
    are kept in the notes instead, which puts both versions in /dialogue and
    makes disagreement measurable.
    """
    nm = lambda z: ZONE_NAMES.get(z, z)
    part = (robot or {}).get("partial") or {}
    out = []
    for st in steps:
        a, zs = st.get("action"), st.get("zones") or []
        if a == "clean":
            bits = []
            for z in zs:
                p = part.get(z)
                bits.append(nm(z) + (f" (restarting from the beginning; it was "
                                     f"about {p} percent done)" if p else ""))
            out.append("cleaning " + ", then ".join(bits))
        elif a == "skip":
            out.append("skipping " + ", ".join(nm(z) for z in zs))
        elif a == "stop":
            out.append("stopping")
        elif a == "resume":
            out.append("resuming from where I stopped")
        elif a == "home":
            out.append("going back to the dock")
        elif a == "reset":
            out.append("forgetting which zones I've cleaned")
        else:
            out.append(str(a))
    s = ", then ".join(out)
    return s[0].upper() + s[1:] + "." if s else "Nothing to do."


def build_graph(llm: Callable[[str, str], str]):''')

ra.once("""        return {"command": {"steps": steps} if len(steps) > 1 else steps[0],
                "reply": say, "notes": notes + [f"planned {len(steps)} step(s)"]}""",
        """        return {"command": {"steps": steps} if len(steps) > 1 else steps[0],
                "reply": confirm(steps, robot),
                "notes": notes + [f"planned {len(steps)} step(s)",
                                  f'model said: "{say}"']}""")

# 10. teach the prompt that resume exists
ra.once("""  - Never work out distances yourself. Use a nearest_/farthest_ reference.""",
        """  - Never work out distances yourself. Use a nearest_/farthest_ reference.
  - If the FACTS say the robot is PAUSED and the operator says carry on,
    continue, keep going or start, with no zone named, the action is resume.
    Never turn that into a clean: it would restart a zone and drop the queue.""")

ra.once("""operator: carry on with aisle 3 from where you left off""",
        """operator: ok carry on          (the FACTS say the robot is PAUSED)
{"plan":[{"action":"resume","zones":[]}],"say":"Picking up where I stopped."}

operator: carry on with aisle 3 from where you left off""")

cn.save()
ra.save()
print()
print('now run:  python3 -m py_compile coverage_nav2.py robot_agent.py')
print('          python3 -m pyflakes coverage_nav2.py robot_agent.py')
print('          python3 test_mission_command.py')
