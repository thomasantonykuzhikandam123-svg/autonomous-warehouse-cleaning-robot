# Debugging log — warehouse cleaning robot

Every fault found, the evidence that proved it, the cause, and the fix. Written
so the reasoning survives outside the chat it happened in, and so the thesis has
something to cite.

Format: **symptom** -> evidence -> cause -> fix (commit).

Baseline for everything below is tag `baseline-v7`.

---

## A. Faults the robot showed in the logs

### A1. `clean <zone>` always restarted the zone
**Symptom.** Asked to clean an aisle it was already 70-80% through, the robot
swept it again from lane 1.
**Evidence.** Session of 22 Sep: painted cells sat at exactly 431 for 90 s
(t=...461 -> ...551) while the robot drove. Lane x=-1.52 of aisle 1 was swept
four times in one session. Across the run, roughly half of ~5.7 min of driving
added no new coverage.
**Cause.** `build_mission` built every lane from `aisle_lanes()` and never read
`self.visited`, the grid of painted cells the same node maintains.
**Fix.** `coverage_lanes.py` — given a lane and the painted grid, return the
spans still dirty, trimmed and merged, skipping lanes already done. Wired into
the aisle branch of `build_mission`. With an empty grid it reproduces the old
waypoints exactly, so a fresh mission is unchanged (`c7f8614`, `19f519c`).
**Result.** Overlap 1.86 -> 1.27 (theoretical floor 1.24), unproductive driving
31% -> 0%.

### A2. The robot described actions it was not taking
**Symptom.** "I'll resume cleaning where I left off in aisle 1", followed by a
restart from lane 1. Later: "Continuing with the back area" while the command
sent was `clean [aisle_3]` — a different zone from the one named.
**Evidence.** T7 reply vs T6 `NEW MISSION` line, same timestamp, twice in two
sessions.
**Cause.** `planner()` returned the model's own `say` string verbatim as the
robot's reply. The plan was validated; the words were not. Three safety layers
all guard the action channel and nothing guarded the narration.
**Fix.** `confirm()` builds the reply from the validated steps. The model's
sentence is kept in `notes` as `model said: "..."`, so `/dialogue` records both
and disagreement is measurable (`b73d7fb`).

### A3. "carry on" while paused started a new mission
**Symptom.** Paused mid aisle 1 with aisle 3 queued, "ok carry on" produced
`clean [aisle_3]`: aisle 1 abandoned at 3.3%, the queue silently dropped.
**Evidence.** T6 `COMMAND: clean ['aisle_3'] (planned)` right after `PAUSED`;
the later `skip aisle_3` reported "nothing to drop".
**Cause.** Three things at once. `_COMMAND_HINTS` contains "carry on" but is
never referenced anywhere in the file — dead code. `PLANNER_PROMPT` had no
`resume` example and a rule stating cleaning always restarts a zone. So the
route fell through to the model, which followed its instructions.
**Fix.** Deterministic rule in `resolve_with_state`: paused + a continuation
word + no zone named -> `resume`, decided in code before any model call. The
prompt also learned that `resume` exists (`b73d7fb`).

### A4. The goal index advanced while the robot was paused
**Symptom.** Paused at goal 5/24; 93 s later the state read 6/24, still paused.
A plain `resume` would have skipped the rest of the lane.
**Evidence.** `TIMEOUT (no active goal) -> advancing` at t=...329, 93 s into a
pause that began at ...236.
**Cause.** `on_command('stop')` cancelled the goal but left `goal_deadline` set,
and `watchdog()` did not check `self._paused`. The per-goal timeout
(`18 + dist/0.15` ~ 103 s for a 12.8 m lane) fired mid-pause, found no active
goal, and advanced.
**Fix.** `stop` clears `goal_deadline`; the watchdog returns early while paused
(`b73d7fb`).

### A5. Distances measured to a zone's centre point
**Symptom.** "The nearest uncleaned aisle is aisle 3 (4.8 m). Going there" —
while driving down aisle 3.
**Evidence.** `ZONE_CENTRE["aisle_3"] = (7.53, 7.50)`, robot at (7.54, 2.70):
exactly 4.80 m.
**Cause.** `_ranked` ranked zones by straight-line distance to a hardcoded
centre point, with no notion of being inside one.
**Fix.** `_zone_gap` returns 0 when the robot is in the zone, using
`/mission_state`'s own `zone` field first and a bounding box as fallback.
Outside a zone the centre metric is kept deliberately: box distance would call
the next aisle 1 m away when it is a 13 m drive around the rack (`b73d7fb`,
`4ecda33`).

### A6. "Nothing yet" while 4.1% of the floor was swept
**Symptom.** Asked what it had cleaned, the robot said "Nothing yet", then a
moment later said aisle 1 was 36% done.
**Cause.** The fact sheet opened with "Cleaned so far: none yet" (finished
zones), and the model answered from that line alone.
**Fix.** The question is answered in code from state — finished zones, per-zone
partials, overall percentage — with no model call (`b73d7fb`).

---

## B. Faults found by reading the code, before they bit

### B1. The cleaned-zone memory never loaded
`self.cleaned = self._load_cleaned()` ran ~30 lines before `self.CLEANED_FILE`
was assigned, so both loaders raised `AttributeError` into a bare `except` and
returned empty. Saving worked; restoring never had, despite being listed as a
working feature. Fixed by setting the path first (`b73d7fb`).

### B2. One waypoint marked a whole zone cleaned
`_zone_ok` gained a zone after a single successful waypoint, and mission
completion promoted it to `cleaned` at 100%. A zone mostly skipped still read as
finished, and that fed `uncleaned` and the fact sheet. Now requires ~90% of the
zone's waypoints (`b73d7fb`).

### B3. A stale rejection advanced the new mission
`goal_response` called `advance()` before its epoch check, so a goal rejected
from a superseded mission stepped the *new* mission's index, silently skipping
its first waypoint. Epoch check moved to the top (`b73d7fb`).

### B4. Still open, documented not fixed
- `ANSWER_PROMPT` is defined and never used. Questions are answered with
  `PLANNER_PROMPT`, so the stricter honesty rules already written in the file
  ("never state a distance, percentage or count not in the FACTS") apply to
  nothing.
- `_COMMAND_HINTS` is dead; `SYSTEM_PROMPT` and `parse_llm_output` are imported
  into `robot_agent` and unused.
- `partial` in `/mission_state` counts waypoints, not floor, so the percentage
  the robot quotes is a waypoint fraction.
- `self.tf_buffer` is created after the first `build_mission()` call, so
  `_map_pose()` inside it throws and is swallowed. Harmless at startup because
  the robot is at the dock; it would matter if the node restarted mid-aisle.
- `test_mission_command.py` (the 37-case validator suite) and `test_mission.py`
  are not in the repository and were not found on the machine.

---

## C. Measurement faults — the numbers themselves were wrong

### C1. The coverage denominator counted the insides of the racks
**Symptom.** The project claimed ~86-89% coverage and never got higher.
**Evidence.** The map is 299x398 at 0.05 m = 297.5 m2, of which 288.0 m2 is
marked free — leaving only 9.5 m2 not free. Four solid racks would be 44-52 m2
on their own; thin outlines plus walls are ~8.7 m2. So the rack interiors are
marked free floor, and /map draws only their outlines.
**Consequence.** ~43 m2 of "floor" sits where the robot can never go. A perfect
run could only ever reach ~85% of "mapped floor" — which *is* the 86-89%
plateau. It was never the planner leaving floor dirty.
**Fix.** `recheck_run.py` computes the honest denominator: cells where the robot
centre legally fits, flood-filled from the dock, dilated by the brush radius
(`f1cd01a`).

### C2. The recheck's own flood fill leaked into two racks
**Symptom.** Only two of four rack interiors showed as unreachable on the
heatmap.
**Cause.** Gaps in racks 1 and 4's outlines in /map let the fill walk inside,
counting 23.6 m2 of rack interior as floor the robot failed to clean.
**Fix.** Rack footprints marked solid explicitly, from the same geometry
`coverage_nav2` uses. 63.4% -> **69.8% of reachable floor** (`f1cd01a`).

### C3. Two instruments appeared to agree, and did not
An early claim that the recorder (5 cm grid) and `coverage_nav2` (25 cm grid)
cross-checked each other was wrong: they agreed because they shared the same
inflated denominator. They also disagree on painted area by ~18%, because a
0.25 m cell counts as painted only when its centre falls under a 0.64 m brush.
The 5 cm figure is the one to quote.

---

## D. Nav2 / platform faults

### D1. `default_server_timeout: 20` caused phantom recovery cascades
**Symptom.** 6 recovery behaviours in a 7-minute run; historic logs with 149
and 150.
**Evidence.** bt_navigator at t=...029: `Begin navigating`, then `Timed out
while waiting for action server to acknowledge goal request for
compute_path_to_pose`, then the same for the costmap clear, spin, wait and
backup — all within 170 ms. planner_server logged no planning failure.
**Cause.** The BT waits 20 ms (the stock default) for a server to *acknowledge*
a goal. On a loaded 8 GB VM that is not enough, so a slow acknowledgement reads
as a planner failure and the whole recovery branch runs.
**Fix.** `default_server_timeout: 20 -> 250` in nav2_params (`14f3b4a`).
**Result.** Recoveries 6 -> 0 on the next run; missed control loops 5 -> 1.

### D2. The stall detector timed Nav2's own recovery
**Symptom.** `STALL: no movement for 10s -> canceling goal early`, cancelling a
goal Nav2 was already recovering.
**Evidence.** behavior_server ran spin (5 s) then wait (5 s) from t=...065 to
...075; the stall fired at ...075.6, exactly at its 10 s mark.
**Cause.** The detector watched translation only. A spin recovery turns on the
spot, which read as a wedged robot, putting three recovery systems on one event.
**Fix.** Rotation counts as movement (0.25 rad threshold) and the window is 15 s,
longer than a full backup+spin+wait (`4ecda33`).

### D3. Per-zone percentages persisted across runs
After B1 was fixed, the loaders worked for the first time and immediately
restored a stale `.pct`: the robot reported "aisle 3 at about 100 percent" in a
run that never entered aisle 3. Percentages describe one run's floor, so they no
longer load or save; finished zones still persist (`4ecda33`).

---

## E. Faults introduced by the fixes themselves

Recorded because they are the cost of changing things, and because the pattern
matters more than the individual bugs.

- **`_zone_gap` box ended at the lane ends (13.90).** The sweep turns past them,
  out to y~14.04, so the robot fell outside its own aisle's box and the reply
  said "aisle 1 (6.5 m)" while in aisle 1. Boxes widened to 0.80-14.40, and the
  robot's own reported zone now wins over geometry (`4ecda33`).
- **`self.visited` used before it existed.** `build_mission()` is called at
  line ~152; the painted grid was created further down. Patch 3 made the mission
  builder read it, so the node died on startup with `AttributeError`. Same
  ordering class as B1 (`3a35e6f`).
- **The already-clean guard rejected `clean everything`.** It compared the
  requested tokens against the zone labels `build_mission` produced, but `all`
  and `uncleaned` are aliases that expand into real zone names, so the literal
  token never appeared. Every alias command was refused. The guard now asks
  whether any waypoint is real work (`b87c0ae`).
- **Tooling.** A `sed` that was not idempotent produced a duplicate console
  script entry, which setuptools rejects as a duplicate entry point — surfacing
  as the unhelpful `'distutils.core.setup()' was never called`. The two package
  folders were also committed as embedded git repositories (mode 160000) rather
  than files.

---

## F. Diagnosed, documented, deliberately not fixed

### F1. Sub-metre gaps are impassable for this robot
The gaps wall->rack 1 and rack 4->wall are 0.85 m. With `robot_radius` 0.30 the
legal band for the robot's centre is 0.25 m wide, inside a costmap inflated to
0.40 m. Three stuck points in the full run were all at **0.30-0.35 m clearance**
and all escaped at 0.50-0.60 m. 22.4 m2 of floor lives there. No lane is planned
for it, and none should be.

### F2. The front zone sweeps 59%, by design
The five lowest rows (world y 0.80-2.55) are restricted to x 6.2-9.5 — 3.3 m of
14.7. The model free map inflates the packing station, pallet and office by
0.5 m (`inflate_cells = 5`), which swallows the 0.5 m gap between packing
station and pallet and the 0.85 m strips at each wall. The 0.50 m standoff was
chosen deliberately after two wedge events at 0.35 m. Lowering it would reopen
exactly the condition F1 measured.

### F3. The reliability/coverage trade, priced
On the full run: **61 of 61 waypoints reached, zero rejections, zero aborts,
zero stalls, zero skips**, at **69.8% of reachable floor**. The unswept 71.8 m2
is 22.4 m2 of impassable gaps, 28 m2 of inflation band around the front
obstacles, and ~21 m2 of lane-overlap limits in the aisles and back. That is the
price of the margin that makes the first number possible.
