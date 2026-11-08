#!/usr/bin/env python3
"""people_guard.py (v7) - see people in the lidar, predict them, and cross their
path the way a considerate forklift driver crosses a pedestrian walkway: ease
off, stop well back at the line, cross in one go when there is room to spare,
set off gently, and never stop in the walkway.

  1. SEE    /scan points that are not on the map (racks, walls removed),
            grouped into blobs (two legs = one blob), followed scan to scan.
            A blob that has moved is a person. The LINE someone walks along is
            fitted through the longest stretch of their last 6 s that is still
            one straight line, and their speed along it over the last 0.6 s, so
            the sideways wobble of the legs in the lidar can't tip their
            predicted path towards the robot. Someone who drops out of view for
            a moment is remembered and recognised again.
  2. JUDGE  predict each person 4 s ahead along their line, turning back at
            racks and walls, from where they were last seen; nobody is ever
            predicted inside a rack. A walker out of view for up to 0.5 s walks
            on; paused, turning, or out of view for longer, they may stand or
            walk off either way along their line. Nobody reverses or sets off
            instantly: from standing they speed up at PERSON_ACCEL (2 m/s2).
            Each person walking ACROSS the robot's path has a crossing zone on
            it: the part of the path within SAFE of where they have walked in
            the last 8 s (their WALKWAY) or may walk in the next 4 s. The STOP
            LINE is SAFE + 0.3 m before it. Crossing people are taken out of the
            scan Nav2 sees (/scan_nav), so Nav2 doesn't swerve into their path.
            People walking ALONG the path stay visible to Nav2, which steers
            round them.
  3. DECIDE - cross only if the whole crossing keeps SAFE (0.76 m centre to
              centre) from everyone AND is comfortable for them, on scans under
              0.15 s old, for 0.2 s in a row. Comfortable means: at least
              COMFORT_FRONT (1.2 m, about 0.65 m between the robot's corner and
              the person) wherever the robot is in front of someone or they are
              standing, and COMFORT_BEHIND (0.95 m) where it passes behind the
              back of someone walking away; in between it blends with the angle.
              Personal space is larger in front: what a person can see coming is
              what makes them flinch. After COMFORT_WAIT s of waiting SAFE alone
              will do, and WAIT_MAX s after that SAFE_PATIENT, so it never waits
              for ever. A crossing never starts just before Nav2's progress checker
              would give up on the goal (it mirrors it: 0.5 m in 10 s), because
              Nav2's recovery would then stop the robot in the middle of it;
            - otherwise drive up to the stop line, easing off on the way, and
              stop there, never in a walkway. After COMFORT_WAIT s there without a
              comfortable gap, the line moves 0.2 m closer (v6's line) and the
              robot edges up to it slowly, once: from further back a slow robot
              may never get a gap from someone pacing to and fro;
            - once it can't stop before the walkway any more, it is COMMITTED
              and finishes the crossing (only an imminent contact, predicted in
              2 scans in a row, stops it);
            - stopped inside a walkway anyway, it leaves unless leaving is worse;
            - from a stop it never creeps: it moves only to cross, to a stop
              line at least 1 m ahead, or that one edge-up.
  4. MOVE GENTLY near people (anyone within NEAR_R): the guard sends Nav2 a
            speed limit on /speed_limit (nav2_msgs/SpeedLimit, the topic Nav2's
            own Speed Filter uses). Approaching a stop line it eases off at
            D_EASE and arrives at about V_CREEP; setting off it steps to V_START
            and speeds up at A_SOFT instead of the robot's ~2.5 m/s2. Inside a
            crossing zone the limit is lifted, so it never lingers in someone's
            walkway. Its predictions use the same gentle robot. With nobody near,
            no limit.
  The Collision Monitor brakes for anything right in front of the robot (like a
  car's emergency braking). guard_gate.py stops the robot if this node or the
  lidar goes quiet: /people_guard/hold is published after every scan this node
  has fully processed, and that is the heartbeat the gate watches.

Output
  /scan_nav           /scan with the people walking across the robot's path
                      removed (NaN). Nav2's costmaps read this (nav2_people.launch.py).
  /people_guard/hold  PointCloud2 in base_link: points inside the robot's body
                      while it must wait (the Collision Monitor stops for them);
                      empty otherwise. Sent after every processed scan (heartbeat).
  /speed_limit        nav2_msgs/SpeedLimit (m/s; 0 = no limit) near people.
  /people_guard/state v10.4: std_msgs/String, JSON, after every processed scan: state, why,
                      'say' (one plain sentence), v_limit and its source, nearest person and
                      vehicle (m, centre to nearest point), counts. For coverage_nav2 and the
                      robot agent; nothing in the safety chain reads it.
  /people_markers     RViz, map frame: people (orange crossing, blue along, red
                      waited for, grey not seen just now), each crossing person's
                      walkway (orange band), their crossing zone on the robot's
                      path (amber), the STOP LINE (white; red while waiting), the
                      robot's path (green) and its decision.
  ~/runs/guard/<label>.csv  one row per scan: what it saw and decided, and why,
                      the speed limit, and the Collision Monitor's input and
                      output speed (so its emergency braking can be located).
The terminal prints every GO / WAIT change with the sim time and the reason.

Run:  python3 ~/ros2_ws/src/tools/people_guard.py [label]

v5 (1 Oct): /scan_nav; hold point; hysteresis; lost people remembered; markers.
v5.1 (1 Oct): realistic slow robot; compare go/stop in someone's path; hold
    points inside the robot; speed from /odom.
v6 (1 Oct): stop line + commit instead of 'approach' and 'in_way' for crossing
    people; nobody predicted inside a rack; walking line fitted over up to 6 s;
    walking/paused with hysteresis; robot model from the measured start-up; GO
    only on scans under 0.15 s old; walkway, crossing zone and stop line in RViz.
v10 (5 Oct): VEHICLES. An off-map blob at least VEH_MIN_LEN long, or VEH_SOLID_W wide with a
    continuous face (legs leave gaps), is a vehicle candidate; one that moves faster than
    VEH_MOVING - with evidence in free space: compared with the scan VEH_EVIDENCE_DT s ago, where
    it was is free now or where it is was free then (a pallet stack the robot drives past only
    shows new faces) - is a moving vehicle (tugger, forklift, pallet truck). Lowered forks are below the
    lidar's plane, so a vehicle is padded VEH_LEAD m at both ends along the way it moves (forks
    lead when it drives forwards, trail when it reverses). A moving one is predicted straight on
    at its speed for VEH_HORIZON s and the robot gives way (WAIT) whenever going on would bring it
    within VEH_SAFE of any part of it, closer than it is now - unless it is already in the
    vehicle's way and stopping there is worse, then it clears it. One that has moved and now stands is kept clear of: for
    its first VEH_WORK s (picking, lifting, turning: forks can point anywhere) by VEH_KEEP all
    round, after that - or at once if it stopped with the robot just ahead of it, i.e. it is
    waiting for the robot - by VEH_BAND from the band its forks lie in, along the way it last
    moved (so the robot can clear its way rather than both waiting). Nav2 is shown that ground
    (/scan_nav), so it plans round the forks it cannot see; the robot waits rather than drive
    further into it. Out of view (behind a rack) it is remembered where it stood for VEH_MEMORY
    s, until the lidar sees through that spot. A vehicle behind the robot
    coming the same way gets no WAIT and no speed cap (stopping in front of it is worse; keeping
    back is its driver's job). Within VEH_NEAR of any other moving or standing vehicle the robot
    moves at V_VEHICLE at most. Vehicles crossing the robot's path are hidden from Nav2, like
    crossing people, but only while the guard is giving way to that vehicle: Nav2 is never left
    blind to one no layer is handling. A blob is never left to neither layer either: it leaves the
    people layer only while a vehicle the vehicle layer is actually handling owns it. People: as in v9.
    GUARD_VEHICLES=0 switches the vehicle layer off (v9 behaviour).
v10.3 (6 Oct, after tour_v101t: the first run with the vehicles visible; laps 148 s against 92,
    two contacts with the robot setting off into someone, one at 0.8 m/s 6 cm from a picker):
    the speed cap near vehicles only while one matters - a moving vehicle's predicted path comes
    within VEH_CAP_GAP of the robot's, or any vehicle is within VEH_CAP_NEAR, or the path ahead
    passes within VEH_CAP_KEEP of a standing one's fork ground (it was everything within 5 m).
    From a stop, no GO that would bring the robot's body within START_GAP of a person's and
    closer than staying would (the 'clearing' rule allowed 0.05 m worse, on centre distances);
    standing in GO (Nav2 or the Collision Monitor holding it) the same check turns GO into WAIT
    'start'. Speed and
    separation: the robot's speed towards anyone not behind it (also people the lidar saw walk who
    now stand still) is capped by the body gap, V_PROX at PROX_GAP0 rising to cruise at PROX_GAP1,
    also inside a walkway and when committed (where v7 lifted every limit); moving away or
    alongside is not capped (except within PROX_GAP0: crawl), so it still clears a walkway at speed. Body gaps use the robot's 0.6 x 0.4 m rectangle and a 0.25 m person.
v10.6 (11 Oct, after shift_v105a): a standing person's personal space is never less than her own
    size (PERSONAL_MIN): in a narrow spot it came out negative and she was left out of the scan Nav2
    sees. Decisions (GO / WAIT, speed limits) are unchanged: the radius only shapes /scan_nav.
v10.11 (19 Oct, after shift_v108a t 98, replayed): one rule, nothing else changed. Head-on at
    touching distance the robot HOLDS (WAIT 'touch', touch_hold): someone within PROX_GAP0 ahead of
    its middle while driving on closes in on them (closing > TOUCH_CLOSING). Before, prox_limit let
    it crawl at V_PROX there, and in shift_v108a it pushed on at 0.03-0.15 m/s into picker_a as she
    crossed its front bumper ('committed' in her walkway; crossers are hidden from the Collision
    Monitor, so nothing below caught it). It holds until her gap opens past PROX_GAP0 + TOUCH_HYST,
    she is no longer ahead of its middle, or going on no longer closes in. Beside or behind its
    middle the crawl stays (driving on is what clears her way).
v10.8 (13 Oct, after shift_v106a and shift_v105a, replayed): two rules, nothing else changed.
    (1) PATIENT_STEADY: in patience (COMFORT_WAIT + WAIT_MAX s at a stop) one scan that keeps
    SAFE_PATIENT + GO_MARGIN starts a crossing; before, it needed two in a row (GO_STEADY). The world's
    worker paces across aisle 1 on a 6.0 s loop and the lidar samples every 0.1 s, so a robot waiting
    at the middle lane sees the same numbers every cycle: in shift_v106a 0.737 then 0.707 m against
    0.71 (one scan), nine cycles running, 64 s, until coverage_nav2 gave the lane up; shift_v105a got
    0.730 then 0.716 and went. (2) d_exit: committed in a walkway, the emergency stop compares
    stopping now with going on to the end of that walkway and stopping there (Look.d_exit), not only
    with going on: a conflict further on (someone at the next walkway, 3 m away) is met by stopping
    after this walkway, not inside it. shift_v106a t 140.5: stopped in the worker's walkway for the
    visitor at the aisle mouth, then had to clear it as the worker came back: 0.03 m at 0.8 m/s.
    A contact the crossing itself would bring still stops it at once (that part of the way is the
    same either way).
v10.4 (11 Oct, after shift_v103a: the cleaning took every guard hold for a wedged robot -
    9 stall cancels, 4 reverses, 2 lanes skipped): decisions unchanged. New output
    /people_guard/state (std_msgs/String, JSON, every scan): GO/WAIT and why, one plain sentence
    for the operator, the speed limit and where it comes from, and how far the nearest person
    and vehicle are. coverage_nav2 reads it to tell traffic from a wedged robot; the robot agent
    reads it to answer "why did you stop?".
v9 (4 Oct, after guard8_eye: the camera saw nothing new because the worker never
    stops walking): the camera's people (eye.py -> /eye/persons) are matched to the lidar's
    blobs. A confirmed person who is STANDING (not walked for STAND_T s) - whom the lidar
    alone can't tell from a box, because a blob that never moved is not a person to it -
    gets personal space: Nav2 is shown her as a cylinder of PERSONAL_R (never so big that
    the aisle closes), and the robot passes at V_PERSON at most within PASS_R of her. A
    post or box the camera doesn't call a person is passed as before. Walking people are
    handled exactly as in v7.1. GUARD_CAMERA=0 ignores the camera (the baseline). The log
    also gets the wall clock and Nav2's plan length (a detour shows as a long plan).
v7.1 (4 Oct, after guard7_lanes: 0 contacts, 8 of 12 crossings comfortable): one
    edge-up per stop for real (the wait timer no longer resets after it); patience
    only 8 s after comfort is waived, never sooner than v6.
v7 (2 Oct, after guard6_lanes: 0 contacts, but crossings 0.98-1.16 m from her,
    full speed almost at once from a stop, and a hard stop at the line): comfort
    distances for crossings (1.2 m in front of someone, 0.95 m behind their back);
    stop line 0.2 m further back (edging up to v6's line after 6 s of waiting);
    speed limit near people (ease off, gentle start); people set off at PERSON_ACCEL, not instantly; no crossing just
    before Nav2's progress checker fires; heartbeat for guard_gate.py; Collision
    Monitor input/output speed in the log.
"""
import array
import collections
import math
import os
import sys
import time
import traceback

import numpy as np

VERSION = 'v10.11'
STATE_TOPIC = '/people_guard/state'   # v10.4: what the guard is doing, as JSON (status())
# --- seeing
LINK = 0.45            # m, points closer than this are one blob (two legs)
BLOB_MAX = 1.0         # m, bigger blobs are not people
GATE = 0.8             # m, how far a blob may be from a track to update it
TIMEOUT = 0.6          # s without a matching blob before a track counts as lost
MIN_HITS = 3           # scans before a track is trusted
MOVING = 0.35          # m/s: once a track goes faster than this ...
TRAVEL = 0.5           # m: ... and has travelled this far from where it appeared, it is a person
WALKING = 0.5          # m/s along their line: walking from here on ...
WALK_EXIT = 0.3        # m/s: ... until slower than this (paused or turning); no flicker
ALPHA, BETA = 0.6, 0.3  # position / velocity filter gains (only used to match blobs to tracks)
GAP_RESET = 0.3        # s: seen again after a longer gap, the filter restarts at the measurement
MAP_CLEAR = 0.2        # m around occupied map cells whose lidar points are ignored
PATH_CLEAR = 0.1       # m around occupied cells where a predicted walk turns back
LOST_KEEP = 5.0        # s a person who dropped out of view is remembered, to be recognised again
LOST_AVOID = 2.0       # s ... and still avoided around where they were last seen
REACQUIRE = 1.2        # m: a new blob this close to a remembered person is that person
HIDE_R = 0.45          # m: off-map lidar points this close to a handled person are hidden from Nav2
CROSSING = 0.7         # |cos| of the angle between someone's walking line and the robot's path:
                       # below this they walk across it (hidden from Nav2, crossing zone);
                       # above, they walk along it (Nav2 keeps seeing them)
LINE_WINDOWS = (6.0, 4.0, 2.0)   # s of measured positions the walking line is fitted through:
                       # the longest that is still one straight line (a picker pacing an aisle
                       # gives a rock-steady line; someone turning a corner falls back to 2 s)
LINE_RMS = 0.15        # m: sideways scatter (sway) allowed for a window to count as one line
LINE_RMS_REL = 0.35    #    ... or this x the spread along it, whichever is larger
LINE_ELONG = 2.5       # a fit must be at least this many times longer than wide to be a line
LINE_SPREAD = 0.25     # m: spread along the fit needed before a (new) line is trusted
LINE_OFF = 0.35        # m: further than this from their line, the line is dropped
SPEED_WINDOW = 0.6     # s: walking speed = least-squares slope along the line over this
STALE = 0.15           # s: not seen for longer, no new GO is given (data too old)
TURN_TIME = 0.5        # s: out of view longer than this, a walker may have turned round
PERSON_ACCEL = 2.0     # m/s^2 a person sets off with, from standing or after turning back
                       # (nobody reverses instantly; they are assumed to stop instantly, though)
# --- judging
SAFE = 0.56 + 0.20     # m, robot-person centre distance to keep (contact 0.56 + margin)
SAFE_PATIENT = 0.56 + 0.10   # m, the same after WAIT_MAX s of waiting (patience)
HORIZON = 4.0          # s looked ahead
DT = 0.1               # s prediction step
REMEMBER = 5.0         # s a paused person may still walk off along their last line;
                       # paused longer, they are left to Nav2 like a pallet
SPEED_MARGIN = 1.05    # predictions use this x their measured walking speed
ZONE_PAD = 0.30        # m: the stop line is SAFE + this from a crossing person's zone ...
ZONE_PAD_WAITED = 0.10 # m: ... and only this (v6's line) after COMFORT_WAIT s there without a
                       #    comfortable gap: the robot edges up to it slowly, once. From further
                       #    back a slow robot (0.5 m/s) may never get a gap from someone pacing
EDGE_MIN = 0.15        # m: the shortest edge-up worth making (once per stop: never creeping)
                       # (v6: 0.10; the robot now waits about 0.2 m further back)
ZONE_MEMORY = 8.0      # s of a crossing person's past positions that are part of their zone
ZONE_LOOK = 7.0        # m of the robot's path checked for crossing zones
CROSS_NEAR = 3.0       # m: a stop line this close makes a GO a 'crossing' (wording only)
FRESH_R = 4.0          # m: everyone this close must be seen < STALE s ago before a GO from WAIT
EMERGENCY = 0.50       # m: committed, but going on would pass this close ...
EMERGENCY_GAIN = 0.20  # m: ... and stopping keeps this much more: stop anyway
EMERGENCY_SCANS = 2    # scans in a row that must say so (one odd prediction is not enough;
                       # anything right in front is the Collision Monitor's job anyway)
# --- comfort (v7): how close and how abruptly the robot moves near people
COMFORT_FRONT = 1.20   # m centre distance a crossing keeps when the robot is in front of
                       # someone (in their walking direction) or they stand (~0.65 m clear)
COMFORT_BEHIND = 0.95  # m ... when it passes behind the back of someone walking away
                       # (in between: blended by the angle). SAFE stays the hard floor
COMFORT_WAIT = 6.0     # s waited at a stop without a comfortable gap before SAFE is accepted
NEAR_R = 3.0           # m: with someone this close the speed limit below applies
V_START = 0.25         # m/s the robot steps to when it sets off near someone ...
A_SOFT = 0.8           # m/s^2 ... then speeds up this gently (the robot on its own: ~2.5)
D_EASE = 0.4           # m/s^2 it eases off with on the way to a stop line ...
V_CREEP = 0.25         # m/s ... arriving at about this speed ...
CREEP_LEN = 0.20       # m ... this far before the stop line (so the final stop is gentle)
LIFT_INSIDE = True     # inside a crossing zone the limit is lifted: clear the walkway at the
                       # robot's own pace instead of lingering in someone's path
SPEED_LIMIT_TOPIC = '/speed_limit'   # nav2_msgs/SpeedLimit to Nav2's controller_server
# --- Nav2's progress checker, mirrored (the node reads the real values from controller_server)
PROG_RADIUS = 0.5      # m: a goal that hasn't moved this far ...
PROG_TIME = 10.0       # s ... in this long fails ('Failed to make progress'), and the
PROG_RECOVERY = 2.5    # s    recovery (stop-and-wait: wait 2 s, clear the costmap) holds the robot
PROG_MARGIN = 0.5      # s: a crossing only starts if it gets PROG_RADIUS done this long before
# --- the robot, as the guard models it
CRUISE_FACTOR = 1.0    # nominal speed = this x FollowPath.desired_linear_vel
CRUISE_DEFAULT = 0.5   # m/s if that parameter can't be read
# (x cruise, m/s^2, s before it moves when setting off from a stop). guard51_lanes, 15
# starts: 0.63 / 1.42 / 2.22 m after 1 / 2 / 3 s (cruise 0.80). SLOW makes 0.50 / 1.26 /
# 2.02 m, FAST 0.71 / 1.55 / 2.39 m: the real robot is in between, and both are checked.
# (x cruise, m/s^2, s before it moves from a stop, m/s it steps to when it moves)
SLOW = (0.95, 4.0, 0.25, 0.0)
FAST = (1.05, 8.0, 0.10, 0.0)
NOMINAL = (1.0, 6.0, 0.20, 0.0)     # the robot as expected, to compare going and stopping


def soft_profiles():
    """(slow, fast, nominal) robot near people, where the guard's own speed limit shapes
    the start (v7): it steps to about V_START and speeds up at about A_SOFT."""
    return ((0.95, 0.85 * A_SOFT, 0.30, 0.8 * V_START),
            (1.05, 1.3 * A_SOFT, 0.10, 1.2 * V_START),
            (1.0, A_SOFT, 0.20, V_START))
BRAKE = 2.0            # m/s^2 assumed when the robot is told to stop
LATENCY = 0.15         # s from a scan to the wheels reacting (this node + Collision Monitor)
STOP_CHECKS = (0.3, 0.6, 1.0, 1.5, 2.0, 2.5, 3.0)   # s: "keep going this long, then stop"
PLAN_AGE = 4.0         # s: an older /plan is not used; the heading line is used instead
PLAN_NEAR = 1.0        # m: the plan is used from its point nearest the robot if that is this close
# --- deciding (hysteresis, so it doesn't flip every scan)
GO_MARGIN = 0.05       # m: from WAIT, the crossing must keep SAFE + this ...
GO_STEADY = 0.2        # s: ... for this long in a row before the robot sets off
PATIENT_STEADY = 0.0   # v10.8: ... but in patience (after COMFORT_WAIT + WAIT_MAX s) one scan will do
KEEP_TOL = 0.08        # m: once crossing, it carries on unless that gets this much worse than SAFE
START_MOVE_DIST = 1.0  # m: from WAIT, drive up to the stop line only if it is at least this far
START_MOVE = 1.0       # s: (people walking along the path) creep only if it can keep going this long
KEEP_MOVE = 0.3        # s: (people along the path) keep going while it can still stop this late
IN_WAY_KEEP = 0.05     # m: (people along the path) moving, keep going if that keeps this much more room
CLEAR_GAIN = 0.10      # m: (people along the path) stopped in their way, go if that keeps this much more
LEAVE_TOL = 0.05       # m: stopped inside a crossing zone, leave it unless leaving is this much worse
WAIT_MAX = 8.0         # s of waiting for a SAFE gap (after COMFORT_WAIT) before patience (SAFE_PATIENT)
PATIENCE_AFTER_COMFORT = True   # v7.1: those WAIT_MAX s start when comfort is waived, so v7 never
                                #   accepts SAFE_PATIENT sooner than v6 did (guard7_lanes: 3 of 12)
# --- output
# --- the camera (v9): people the camera has confirmed (eye.py -> /eye/persons, map frame)
USE_CAMERA = os.environ.get('GUARD_CAMERA', '1') != '0'
CAM_MATCH = 0.5        # m: a camera person this close to a lidar blob's track is that track
CAM_KEEP = 20.0        # s a track stays a confirmed person after the camera last saw it, while
                       #   the lidar keeps tracking it (passing her, the camera loses her long before)
CAM_ONLY_KEEP = 4.0    # s a camera person with no lidar track (e.g. right at a rack face, where
                       #   rack points are ignored) is kept, standing where the camera last put her
CAM_MAX_AGE = 1.5      # s: camera reports older than this (sim time) are ignored
STAND_T = 4.0          # s a confirmed person must have stayed put to count as standing (a walker
STAND_MOVE = 0.30      #   turning round at the end of their walk, or a slow walker, is not): within
                       #   this many m of where they are now for all of the last STAND_T s
STAND_NEW = 1.0        # s, for someone never seen walking (already standing when the lidar first saw
                       #   them, like a picker at a shelf): the robot shouldn't wait 4 s to give room
CAM_ONLY_STAND = 2.0   # s a camera-only person must stay put (no lidar blob to check it against)
PERSONAL_R = 0.90      # m: personal space Nav2 is shown around a standing person, from her centre:
                       #   with Nav2's 0.30 m robot circle the robot's centre keeps 1.2 m = COMFORT_FRONT
PERSONAL_MIN = 0.25    # m: never less than her own size
AISLE_KEEP = 0.90      # m of floor always left free beside the personal space (the robot is 0.6 m
                       #   wide), so Nav2 still has a way past and never detours round the building
ROBOT_KEEP = 0.45      # m: the personal space never reaches within this of the robot's centre
                       #   (an obstacle inside the robot would freeze Nav2's controller)
PASS_R = 1.5           # m: within this of a standing person (centre to centre) the robot moves at most
V_PERSON = 0.30        # m/s ... this; it eases off at D_EASE on the way in, speeds up at A_SOFT after
# --- vehicles (v10): tuggers, forklifts, pallet trucks - big or solid movers the robot gives way to
USE_VEHICLES = os.environ.get('GUARD_VEHICLES', '1') != '0'
VEH_MIN_LEN = 0.9      # m: an off-map blob at least this long is a vehicle candidate, or one ...
VEH_SOLID_W = 0.6      # m: ... at least this wide whose face is continuous (two legs leave a gap;
                       #    a walker's legs, seen at a slant, can look like a solid 0.5 m face):
VEH_SOLID_FILL = 0.75  #     rays that hit it / rays across its angular width
VEH_MAX_LEN = 4.5      # m: longer than this it is a wall or rack the map doesn't have
VEH_MIN_PTS = 4        # lidar points a candidate needs
VEH_MIN_HITS = 5       # scans a vehicle must have been seen in before it counts (moves, owns people)
VEH_GATE = 1.5         # m: how far a candidate may be from a vehicle track to update it
VEH_TIMEOUT = 1.0      # s without a candidate before a vehicle track is dropped
VEH_WINDOW = 1.0       # s of centres its velocity is fitted over
VEH_MOVING = 0.3       # m/s: faster than this it is a moving vehicle (until slower than VEH_STOPPED)
VEH_STOPPED = 0.15     # m/s
VEH_EVIDENCE_DT = 0.8  # s: a vehicle's points are compared with the scan this long ago, and it is
VEH_CHANGE = 0.25      #    moving only if this share of them changed (left free / was free)
VEH_CHANGE_KEEP = 1.0  # s without such a change after which it is not moving
VEH_RECENT = 30.0      # s a vehicle that has moved still counts as one when it stands (a parked forklift)
VEH_SAFE = 1.0         # m from the robot's centre to any part of a moving vehicle (people: 0.76)
VEH_LEAD = 1.2         # m of unseen forks or load assumed at both ends of a vehicle, along its motion
                       #   (lowered forks sit below the lidar's 0.28 m plane)
VEH_HORIZON = 4.0      # s a vehicle is predicted ahead, straight on at its speed
VEH_CLEAR = 1.0        # s the way must have stayed clear before the robot moves after giving way
VEH_NEAR = 5.0         # m: within this of a moving (or recently moving) vehicle the robot moves at most
V_VEHICLE = 0.40       # m/s
VEH_HIDE_R = 8.0       # m: moving vehicles crossing the path this close are hidden from Nav2
VEH_OWN_R = 0.5        # m: a people track this close to a vehicle's points belongs to the vehicle
VEH_KEEP = 1.5         # m: a vehicle that has just stopped may have its forks anywhere this close to
                       #    what the lidar sees of it (forks 1.1 m + the robot's half-diagonal 0.36)
VEH_WORK = 8.0         # s: after standing this long its forks lie along the way it last moved
VEH_BAND = 0.36        # m: the robot's centre keeps this far from that band (VEH_LEAD at both ends):
                       #    its half-diagonal, so no part of it enters the band
NAV_INSCRIBED = 0.20   # m: Nav2 keeps the robot's centre at least this far from what it is shown
                       #    (inscribed radius of the 0.6 x 0.4 m footprint)
VEH_ZONE_R = VEH_KEEP - NAV_INSCRIBED       # m: Nav2 is shown a vehicle that just stopped this much
                                            #    bigger all round, so its plans keep VEH_KEEP
VEH_ZONE_PAD = VEH_BAND - NAV_INSCRIBED     # m: ... and a band this much bigger, to keep VEH_BAND
VEH_WIDTH = 1.0        # m: a vehicle is taken to be at least this wide (the lidar may see one side)
VEH_YIELD_D = 4.0      # m: stopped with the robot this close ahead of it: it is waiting for the robot
VEH_KEEP_LOOK = 3.0    # m of the robot's path checked against standing vehicles
VEH_MEMORY = 15.0      # s a standing vehicle out of view is remembered where it stood
VEH_SEEN_THROUGH = 0.5 # ... unless the lidar sees through this share of the points it had there
# v10.3
VEH_CAP_GAP = 2.0      # m: the V_VEHICLE cap applies while a moving vehicle (padded) is predicted this
                       #    close to the robot going on, ...
VEH_CAP_NEAR = 1.5     # m: ... or any vehicle it follows is this close now, ...
VEH_CAP_KEEP = 0.5     # m: ... or the path ahead passes within this of a standing vehicle's keep-out
ROBOT_LEN, ROBOT_WID = 0.6, 0.4    # m: the robot's body, for body-to-body gaps
PERSON_BODY = 0.25     # m: a person's radius for body gaps (the tour's measure)
START_GAP = 0.30       # m: from a stop, no GO that brings the robot's body this close to a person's
                       #    (and closer than staying would) within START_LOOK s
START_LOOK = (0.3, 0.6, 1.0)       # s checked after a GO (the robot starts at V_START after ~0.2 s)
START_WORSE = 0.03     # m: ... and closer than staying put would by more than this
START_STILL = 0.05     # m/s: slower than this the robot is standing; the start floor applies in GO
                       #    too (Nav2 or the Collision Monitor holding it: tour_v101t #1 set off
                       #    from a 'go' standstill into picker_a walking at it)
PROX_GAP0 = 0.10       # m: speed and separation: at this body gap to someone not behind the robot ...
PROX_GAP1 = 0.90       # m: ... rising linearly to cruise at this gap
V_PROX = 0.15          # m/s: ... the robot moves towards them at most this
TOUCH_CLOSING = 0.5    # v10.11: within PROX_GAP0 ahead and closing faster than this per metre driven:
                       # head-on at touching distance -> hold (WAIT), not the V_PROX crawl
TOUCH_HYST = 0.15      # m: holding for someone, it holds until the gap opens past PROX_GAP0 + this
PROX_STEP = 0.10       # m: how far ahead the closing rate is measured
HOLD_X = 0.0           # m: hold points sit inside the robot's body (the monitor's zone is there too)
Z = 0.3                # m, height of published points
BODY_R = 0.25          # m, person radius drawn in RViz


# ---------------------------------------------------------------- seeing
class StaticMap:
    """Occupancy grid, dilated, for point-in-obstacle checks (map frame)."""

    def __init__(self, width, height, resolution, origin_x, origin_y, data, threshold=50):
        self.w, self.h, self.res = width, height, resolution
        self.ox, self.oy = origin_x, origin_y
        occ = np.asarray(data, dtype=np.int16).reshape(height, width) > threshold
        self.scan_mask = self._dilate(occ, MAP_CLEAR)
        self.path_mask = self._dilate(occ, PATH_CLEAR)

    def _dilate(self, occ, radius):
        k = int(math.ceil(radius / self.res))
        out = occ.copy()
        for dy in range(-k, k + 1):
            for dx in range(-k, k + 1):
                if dx * dx + dy * dy > k * k:
                    continue
                shifted = np.zeros_like(occ)
                ys = slice(max(dy, 0), self.h + min(dy, 0))
                yd = slice(max(-dy, 0), self.h + min(-dy, 0))
                xs = slice(max(dx, 0), self.w + min(dx, 0))
                xd = slice(max(-dx, 0), self.w + min(-dx, 0))
                shifted[ys, xs] = occ[yd, xd]
                out |= shifted
        return out

    def blocked(self, x, y, mask='scan'):
        """Vectorised: True where (x, y) is on/near an obstacle or off the map."""
        m = self.scan_mask if mask == 'scan' else self.path_mask
        x, y = np.asarray(x, dtype=float), np.asarray(y, dtype=float)
        i = np.floor((x - self.ox) / self.res).astype(int)
        j = np.floor((y - self.oy) / self.res).astype(int)
        inside = (i >= 0) & (i < self.w) & (j >= 0) & (j < self.h)
        out = np.ones(x.shape, dtype=bool)
        out[inside] = m[j[inside], i[inside]]
        return out


def scan_points(ranges, angle_min, angle_inc, range_min, range_max, pose):
    """LaserScan -> ((N, 2) map-frame points, (N,) ray indices); pose = laser (x, y, yaw)."""
    r = np.asarray(ranges, dtype=float)
    a = angle_min + angle_inc * np.arange(r.size)
    idx = np.nonzero(np.isfinite(r) & (r > range_min) & (r < range_max))[0]
    r, a = r[idx], a[idx] + pose[2]
    return np.column_stack((pose[0] + r * np.cos(a), pose[1] + r * np.sin(a))), idx


def blobs(points):
    """Single-link clustering (LINK). Returns a list of index arrays into points."""
    n = len(points)
    if n == 0:
        return []
    label = -np.ones(n, dtype=int)
    k = 0
    for s in range(n):
        if label[s] >= 0:
            continue
        label[s] = k
        stack = [s]
        while stack:
            i = stack.pop()
            d = np.hypot(points[:, 0] - points[i, 0], points[:, 1] - points[i, 1])
            for j in np.nonzero((d < LINK) & (label < 0))[0]:
                label[j] = k
                stack.append(j)
        k += 1
    return [np.nonzero(label == c)[0] for c in range(k)]


class Track:
    _next = 1

    def __init__(self, x, y, t):
        self.id = Track._next
        Track._next += 1
        self.x, self.y, self.vx, self.vy = float(x), float(y), 0.0, 0.0   # filter (matching only)
        self.x0, self.y0 = self.x, self.y          # where it appeared
        self.t = t                                 # last seen
        self.hits = 1
        self.moved = False
        self.lost = False                          # dropped out of view, remembered
        self.raw = collections.deque([(t, self.x, self.y)])   # measured centres, ZONE_MEMORY s
        self.line = None          # (cx, cy, ux, uy): a point on the line they walk along, its direction
        self.v_along = 0.0        # m/s along (ux, uy), signed
        self.px, self.py = self.x, self.y          # where they are, on their line (sway removed)
        self.walk_dir = None      # unit vector of their last steady walk
        self.walk_t = None        # when they last walked steadily
        self.speeds = collections.deque()          # (t, speed) while walking, last 3 s
        self.walking = False      # walking (with hysteresis: WALKING / WALK_EXIT)
        self.cam_t = None         # v9: when the camera last confirmed this blob is a person

    @property
    def speed(self):
        """Walking speed along their line (least squares over SPEED_WINDOW)."""
        return abs(self.v_along)

    @property
    def walk_speed(self):
        """Their usual walking speed: median of the last 3 s of walking."""
        if not self.speeds:
            return 0.0
        return float(np.median([s for _, s in self.speeds]))

    @property
    def heading(self):
        """Unit vector of where they are walking now (None if not walking)."""
        if self.line is None or not self.walking:
            return None
        sg = 1.0 if self.v_along > 0 else -1.0
        return sg * self.line[2], sg * self.line[3]

    @property
    def wvx(self):
        return 0.0 if self.line is None else self.v_along * self.line[2]

    @property
    def wvy(self):
        return 0.0 if self.line is None else self.v_along * self.line[3]

    def update(self, x, y, t):
        x, y = float(x), float(y)
        dt = max(t - self.t, 1e-3)
        if dt > GAP_RESET:                  # back after a gap: the filter's guess has run off
            self.x, self.y, self.vx, self.vy = x, y, 0.0, 0.0
        else:
            px, py = self.x + self.vx * dt, self.y + self.vy * dt
            rx, ry = x - px, y - py
            self.x, self.y = px + ALPHA * rx, py + ALPHA * ry
            self.vx += BETA * rx / dt
            self.vy += BETA * ry / dt
        self.t = t
        self.hits += 1
        self.raw.append((t, x, y))
        while self.raw and t - self.raw[0][0] > ZONE_MEMORY:
            self.raw.popleft()
        self.estimate(t)
        if self.speed >= WALKING:
            self.walking = True
        elif self.speed < WALK_EXIT:
            self.walking = False
        if (self.hits >= MIN_HITS and self.speed > MOVING
                and math.hypot(x - self.x0, y - self.y0) > TRAVEL):
            self.moved = True        # jitter at a rack corner never travels 0.5 m
        if self.hits >= MIN_HITS and self.speed >= WALKING and self.line is not None:
            self.speeds.append((t, self.speed))
            self.walk_dir = self.heading
            self.walk_t = t
        while self.speeds and t - self.speeds[0][0] > 3.0:
            self.speeds.popleft()

    def estimate(self, t):
        """Fit their walking line through the longest window (LINE_WINDOWS) of measured
        positions that is still one straight line; speed along it = least-squares
        slope over the last SPEED_WINDOW s."""
        allp = list(self.raw)
        for win in LINE_WINDOWS:
            sel = [p for p in allp if t - p[0] <= win + 1e-9]
            if len(sel) < 4:
                continue
            Pw = np.array([(p[1], p[2]) for p in sel])
            c = Pw.mean(axis=0)
            Q = Pw - c
            w, V = np.linalg.eigh(Q.T.dot(Q) / len(Pw))
            along, side = math.sqrt(max(w[1], 0.0)), math.sqrt(max(w[0], 0.0))
            if along < LINE_SPREAD or along < LINE_ELONG * side:
                continue                         # too short, or a blob: not a line
            if side > max(LINE_RMS, LINE_RMS_REL * along):
                continue                         # not one straight line over this long: shorter
            ux, uy = float(V[0, 1]), float(V[1, 1])
            if self.line is not None and ux * self.line[2] + uy * self.line[3] < 0:
                ux, uy = -ux, -uy                # keep the direction's sign stable
            self.line = (float(c[0]), float(c[1]), ux, uy)
            break
        pts = [p for p in allp if t - p[0] <= LINE_WINDOWS[-1] + 1e-9]
        T = np.array([p[0] for p in pts])
        P = np.array([(p[1], p[2]) for p in pts])
        if self.line is not None:
            cx, cy, ux, uy = self.line
            off = np.abs(-(P[:, 0] - cx) * uy + (P[:, 1] - cy) * ux)
            if float(np.median(off[-5:])) > LINE_OFF:
                self.line = None                 # walked off that line: start again
        if self.line is None:
            self.v_along = 0.0
            self.px, self.py = float(P[-1, 0]), float(P[-1, 1])
            return
        cx, cy, ux, uy = self.line
        s = (P[:, 0] - cx) * ux + (P[:, 1] - cy) * uy
        rec = T >= t - SPEED_WINDOW - 1e-9
        if rec.sum() >= 3 and np.ptp(T[rec]) >= 0.25:
            tt = T[rec] - T[rec].mean()
            ss = s[rec] - s[rec].mean()
            self.v_along = float((tt * ss).sum() / (tt * tt).sum())
            s_now = float(s[rec].mean() + self.v_along * (T[rec][-1] - T[rec].mean()))
        else:
            self.v_along = 0.0
            s_now = float(s[-1])
        self.px, self.py = cx + ux * s_now, cy + uy * s_now

    def adopt(self, old):
        """This new track is a person seen before: take over who they are."""
        self.id, self.moved = old.id, True
        self.x0, self.y0 = old.x0, old.y0
        self.raw = collections.deque(list(old.raw) + list(self.raw))
        self.line, self.walk_dir, self.walk_t = old.line, old.walk_dir, old.walk_t
        self.speeds = collections.deque(old.speeds)
        self.hits = max(self.hits, MIN_HITS)
        if old.cam_t is not None:
            self.cam_t = max(old.cam_t, self.cam_t or old.cam_t)
        self.estimate(self.t)


def track_dist(tr, x, y, ts):
    """Distance from (x, y) to where this track was at time ts (its measured centre nearest in
    time, within 0.5 s; else where it is now): camera reports arrive a few tenths late."""
    best = None
    for tt, rx, ry in reversed(tr.raw):
        if abs(tt - ts) <= 0.5 and (best is None or abs(tt - ts) < best[0]):
            best = (abs(tt - ts), rx, ry)
        if tt < ts - 0.5:
            break
    if best is None:
        return math.hypot(tr.x - x, tr.y - y)
    return math.hypot(best[1] - x, best[2] - y)


def is_standing(tr, t):
    """Standing: seen for the last STAND_T s (most of it) and never more than STAND_MOVE from
    where it is now. Slow walkers and someone turning round at the end of their walk are not."""
    if tr.walking:
        return False
    window = STAND_T if tr.moved else STAND_NEW
    pts = [(tt, x, y) for tt, x, y in tr.raw if t - tt <= window]
    if len(pts) < 5 or pts[-1][0] - pts[0][0] < 0.8 * window:
        return False
    lx, ly = pts[-1][1], pts[-1][2]
    return max(math.hypot(x - lx, y - ly) for _, x, y in pts) < STAND_MOVE


def personal_radius(x, y, robot_xy, robot_yaw, static_map, plan=None):
    """Personal space for someone standing at (x, y): PERSONAL_R, but always leaving AISLE_KEEP
    of floor between it and the rack or wall across the robot's path, and never covering the
    robot. Returns (radius, room across the aisle from her centre)."""
    ux, uy = path_dir_near(x, y, robot_xy, robot_yaw, plan)
    nx, ny = -uy, ux                                  # across the path
    ref = None
    if plan is not None and len(plan) >= 2:
        p = np.asarray(plan, dtype=float)
        i = int(np.argmin(np.hypot(p[:, 0] - x, p[:, 1] - y)))
        ref = p[i]
    if ref is None or math.hypot(ref[0] - x, ref[1] - y) < 0.05:
        ref = robot_xy
    if (ref[0] - x) * nx + (ref[1] - y) * ny < 0:
        nx, ny = -nx, -ny                             # from her towards the robot's path
    room = free_runs(x, y, nx, ny, static_map)[0] + PATH_CLEAR
    r = min(PERSONAL_R, room - AISLE_KEEP)
    # v10.6: never less than her own size, as PERSONAL_MIN says. Standing in a narrow spot (the
    # stepper beside rack_2's end in shift_v105a: room 0.40 m) this came out at -0.50 m, which
    # left her out of Nav2's view altogether. Only ROBOT_KEEP may still take her out (an
    # obstacle inside the robot would freeze Nav2's controller); the lane check in coverage_nav2
    # v10.6 keeps the robot short of her then.
    r = max(r, PERSONAL_MIN)
    r = min(r, math.hypot(robot_xy[0] - x, robot_xy[1] - y) - ROBOT_KEEP)
    return r, room


def shape_ranges(ranges, angle_min, angle_inc, range_min, range_max, laser, circles):
    """Show Nav2 each standing person as a cylinder of their personal-space radius: a beam
    that would pass through that circle ends where it enters it. Hidden (NaN) rays stay
    hidden; the scan Nav2 gets is otherwise unchanged."""
    r = np.array(ranges, dtype=np.float32, copy=True)
    if not circles or not r.size:
        return r
    a = angle_min + angle_inc * np.arange(r.size) + laser[2]
    dx, dy = np.cos(a), np.sin(a)
    meas = np.where(np.isfinite(r), r, np.inf)
    for cx, cy, R in circles:
        fx, fy = laser[0] - cx, laser[1] - cy
        b = fx * dx + fy * dy
        c = fx * fx + fy * fy - R * R
        if c <= 0.0:
            continue                                  # the lidar is inside it: leave the scan alone
        disc = b * b - c
        t1 = -b - np.sqrt(np.maximum(disc, 0.0))
        cut = (disc > 0.0) & (t1 > range_min) & (t1 < min(range_max, 8.0)) & (t1 < meas) & ~np.isnan(r)
        r[cut] = t1[cut]
        meas = np.where(cut, t1, meas)
    return r


def shape_zones(ranges, angle_min, angle_inc, range_min, range_max, laser, zones):
    """Show Nav2 standing vehicles' fork ground: zones are ('box', u, n, a0, a1, b0, b1) (a
    rectangle in a frame along u / across n) or ('circle', x, y, R). A beam that would pass
    through one ends where it enters it - never closer than ROBOT_KEEP to the lidar, and not
    at all if the lidar is inside it. Hidden (NaN) rays stay hidden."""
    r = np.array(ranges, dtype=np.float32, copy=True)
    if not zones or not r.size:
        return r
    a = angle_min + angle_inc * np.arange(r.size) + laser[2]
    dx, dy = np.cos(a), np.sin(a)
    meas = np.where(np.isfinite(r), r, np.inf)
    lim = min(range_max, 8.0)
    for z in zones:
        if z[0] == 'circle':
            _, cx, cy, R = z
            fx, fy = laser[0] - cx, laser[1] - cy
            b = fx * dx + fy * dy
            c = fx * fx + fy * fy - R * R
            if c <= 0.0:
                continue
            disc = b * b - c
            t1 = -b - np.sqrt(np.maximum(disc, 0.0))
            ok = disc > 0.0
        else:
            _, u, n, a0, a1, b0, b1 = z
            oa, ob = laser[0] * u[0] + laser[1] * u[1], laser[0] * n[0] + laser[1] * n[1]
            if a0 <= oa <= a1 and b0 <= ob <= b1:
                continue
            da, db = dx * u[0] + dy * u[1], dx * n[0] + dy * n[1]
            with np.errstate(divide='ignore', invalid='ignore'):
                ta0, ta1 = (a0 - oa) / da, (a1 - oa) / da
                tb0, tb1 = (b0 - ob) / db, (b1 - ob) / db
                lo = np.maximum(np.minimum(ta0, ta1), np.minimum(tb0, tb1))
                hi = np.minimum(np.maximum(ta0, ta1), np.maximum(tb0, tb1))
            lo = np.where(np.abs(da) < 1e-9, np.where((a0 <= oa) & (oa <= a1), np.minimum(tb0, tb1), np.inf), lo)
            lo = np.where(np.abs(db) < 1e-9, np.where((b0 <= ob) & (ob <= b1), np.minimum(ta0, ta1), np.inf), lo)
            hi = np.where(np.abs(da) < 1e-9, np.where((a0 <= oa) & (oa <= a1), np.maximum(tb0, tb1), -np.inf), hi)
            hi = np.where(np.abs(db) < 1e-9, np.where((b0 <= ob) & (ob <= b1), np.maximum(ta0, ta1), -np.inf), hi)
            t1 = lo
            ok = hi >= lo
        cut = ok & (t1 > max(range_min, ROBOT_KEEP)) & (t1 < lim) & (t1 < meas) & ~np.isnan(r)
        r[cut] = t1[cut]
        meas = np.where(cut, t1, meas)
    return r


def recent(tr, t):
    return tr.walk_t is not None and t - tr.walk_t <= REMEMBER


def handled(tr, t):
    """Is this a person the guard handles (predicts, and hides from Nav2)?"""
    if getattr(tr, 'vehicle', False):
        return False                                  # v10: part of a vehicle
    if not tr.moved:
        return False
    if tr.lost:
        return t - tr.t <= LOST_AVOID and recent(tr, t)
    return tr.walking or recent(tr, t)


class Tracker:
    def __init__(self):
        self.tracks = []
        self.lost = []

    def update(self, centroids, t):
        """Match blob centroids to tracks. Returns {centroid index: track}."""
        pairs = sorted((math.hypot(c[0] - tr.x - tr.vx * min(t - tr.t, GAP_RESET),
                                   c[1] - tr.y - tr.vy * min(t - tr.t, GAP_RESET)), ti, ci)
                       for ti, tr in enumerate(self.tracks)
                       for ci, c in enumerate(centroids))
        used_t, match = set(), {}
        for d, ti, ci in pairs:                      # nearest pairs first
            if d > GATE or ti in used_t or ci in match:
                continue
            self.tracks[ti].update(centroids[ci][0], centroids[ci][1], t)
            used_t.add(ti)
            match[ci] = self.tracks[ti]
        for ci, c in enumerate(centroids):
            if ci not in match:
                tr = Track(c[0], c[1], t)
                self.recognise(tr, t)
                self.tracks.append(tr)
                match[ci] = tr
        keep = []
        for tr in self.tracks:
            if t - tr.t <= TIMEOUT:
                keep.append(tr)
            elif tr.moved:
                tr.lost = True
                self.lost.append(tr)
        self.tracks = keep
        self.lost = [tr for tr in self.lost if t - tr.t <= LOST_KEEP]
        return match

    def recognise(self, new, t):
        """A new blob near where a remembered person was (or would have walked to)."""
        best, best_d = None, REACQUIRE
        for old in self.lost:
            lag = min(t - old.t, 1.0)
            d = min(math.hypot(new.x - old.px, new.y - old.py),
                    math.hypot(new.x - old.px - old.wvx * lag, new.y - old.py - old.wvy * lag))
            if d < best_d:
                best, best_d = old, d
        if best is not None:
            self.lost.remove(best)
            new.adopt(best)

    def people(self, t):
        """Everyone the guard handles: people in view, and those just lost from view."""
        return [tr for tr in self.tracks + self.lost if handled(tr, t)]


def see(tracker, static_map, ranges, angle_min, angle_inc, range_min, range_max, pose, t,
        hide_who=None):
    """One scan through filtering, clustering and tracking. hide_who(track) picks
    whom to hide from Nav2 (default: everyone the guard handles).
    Returns (people, ray indices to hide from Nav2)."""
    pts, idx = scan_points(ranges, angle_min, angle_inc, range_min, range_max, pose)
    if static_map is not None and len(pts):
        off = ~static_map.blocked(pts[:, 0], pts[:, 1], 'scan')
        pts, idx = pts[off], idx[off]
    groups, centroids = [], []
    for g in blobs(pts):
        b = pts[g]
        if len(b) >= 2 and np.hypot(*(b.max(axis=0) - b.min(axis=0))) <= BLOB_MAX:
            groups.append(g)
            centroids.append(tuple(b.mean(axis=0)))
    match = tracker.update(centroids, t)
    people = tracker.people(t)
    hidden = [tr for tr in people if hide_who is None or hide_who(tr)]
    hide = np.zeros(len(pts), dtype=bool)
    ids = {tr.id for tr in hidden}
    for ci, g in enumerate(groups):
        if match[ci].id in ids:
            hide[g] = True
    for tr in hidden:
        if len(pts):
            hide |= np.hypot(pts[:, 0] - tr.x, pts[:, 1] - tr.y) < HIDE_R
    return people, idx[hide]


# ---------------------------------------------------------------- judging
def free_runs(x0, y0, ux, uy, static_map):
    """m someone can walk from (x0, y0) along (ux, uy) and against it before a rack or wall."""
    steps = np.arange(1, 161) * 0.05                   # up to 8 m each way

    def run(sign):
        if static_map is None:
            return 8.0
        hit = static_map.blocked(x0 + sign * ux * steps, y0 + sign * uy * steps, 'path')
        k = int(np.argmax(hit)) if hit.any() else len(steps)
        return 0.0 if k == 0 else float(steps[k - 1])
    return run(1.0), run(-1.0)


def turn_walk(x0, y0, ux, uy, sp, fwd, back, n, t0=0.0, v0=None):
    """n positions, DT apart starting t0 s after (x0, y0): walking along (ux, uy) at sp
    (setting off at v0, default sp), turning back after fwd m and from then on between
    the two ends (back m behind the start). At each end they stop (at once) and set off
    again at PERSON_ACCEL: nobody reverses instantly. Never inside an obstacle: with no
    room either way they stay where they are."""
    span = fwd + back
    u0 = sp if v0 is None else min(max(v0, 0.0), sp)
    out = np.empty((n, 2))
    for k in range(n):
        t = t0 + DT * k
        s, d, leg, u = 0.0, 1.0, fwd, u0
        while sp > 1e-6 and span > 1e-6:
            t_acc = (sp - u) / PERSON_ACCEL
            d_acc = 0.5 * (u + sp) * t_acc
            if leg <= d_acc:
                t_leg = (-u + math.sqrt(u * u + 2.0 * PERSON_ACCEL * leg)) / PERSON_ACCEL
            else:
                t_leg = t_acc + (leg - d_acc) / sp
            if t <= t_leg:
                x = u * t + 0.5 * PERSON_ACCEL * t * t if t < t_acc else d_acc + sp * (t - t_acc)
                s += d * min(x, leg)
                break
            t -= t_leg
            s += d * leg
            d, leg, u = -d, span, 0.0
        out[k] = (x0 + ux * s, y0 + uy * s)
    return out


def walk(x0, y0, ux, uy, sp, static_map, n, t0=0.0, v0=None):
    """n positions, DT apart starting t0 s after (x0, y0), walking along (ux, uy) at sp
    and turning back at racks and walls (see turn_walk)."""
    fwd, back = free_runs(x0, y0, ux, uy, static_map)
    return turn_walk(x0, y0, ux, uy, sp, fwd, back, n, t0=t0, v0=v0)


def heading_of(tr):
    """Unit vector of the line someone walks along (now, or their last steady walk)."""
    if not tr.lost and tr.heading is not None:
        return tr.heading
    return tr.walk_dir


def path_dir_near(x, y, robot_xy, robot_yaw, plan=None):
    """Direction of the robot's path where it passes closest to (x, y)."""
    if plan is not None and len(plan) >= 2:
        p = np.asarray(plan, dtype=float)
        i = int(np.argmin(np.hypot(p[:, 0] - x, p[:, 1] - y)))
        a, b = (i, i + 1) if i + 1 < len(p) else (i - 1, i)
        while b < len(p) - 1 and math.hypot(p[b, 0] - p[a, 0], p[b, 1] - p[a, 1]) < 0.3:
            b += 1                                   # plans come in 5 cm steps
        dx, dy = p[b, 0] - p[a, 0], p[b, 1] - p[a, 1]
        L = math.hypot(dx, dy)
        if L > 1e-6:
            return dx / L, dy / L
    return math.cos(robot_yaw), math.sin(robot_yaw)


def crossing(tr, robot_xy, robot_yaw, plan=None):
    """Does this person walk across the robot's path (rather than along it)?"""
    h = heading_of(tr)
    if h is None:
        return False
    ux, uy = path_dir_near(tr.px, tr.py, robot_xy, robot_yaw, plan)
    return abs(h[0] * ux + h[1] * uy) < CROSSING


def futures(tr, static_map, t_now, n):
    """Possible futures ((n, 2) arrays) for one person; [] = not ours to handle."""
    if not handled(tr, t_now):
        return []
    age = max(t_now - tr.t, 0.0)
    x0, y0 = tr.px, tr.py                      # where they were last seen (on their line)
    h = tr.heading
    if not tr.lost and age <= TURN_TIME and h is not None:
        # walking, seen in the last TURN_TIME s: on along their line, turning back at
        # racks. A scan or two without them changes nothing: they can't have turned
        # round in that time (and no new GO is given on data older than STALE).
        sp = SPEED_MARGIN * max(tr.speed, tr.walk_speed)
        return [walk(x0, y0, h[0], h[1], sp, static_map, n, t0=age)]
    # paused, turning, or out of view long enough to have turned: stand, or walk off
    # either way along their line
    stand = np.tile([x0, y0], (n, 1))
    d = tr.walk_dir if tr.walk_dir is not None else (None if tr.line is None else tr.line[2:])
    if d is None:
        return [stand]
    sp = SPEED_MARGIN * max(tr.walk_speed, WALKING)
    t0 = min(age, LOST_AVOID)
    # setting off from a standstill (at PERSON_ACCEL) since they were last seen
    return [stand,
            walk(x0, y0, d[0], d[1], sp, static_map, n, t0=t0, v0=0.0),
            walk(x0, y0, -d[0], -d[1], sp, static_map, n, t0=t0, v0=0.0)]


def walkway(tr, t):
    """Where a crossing person has walked in the last ZONE_MEMORY s."""
    past = [(x, y) for tt, x, y in tr.raw if t - tt <= ZONE_MEMORY][::2]
    return np.asarray(past, dtype=float).reshape(-1, 2)


def path_ahead(robot_xy, robot_yaw, plan=None):
    """(cumulative distance, x, y) of the line the robot will drive: the plan from
    its point nearest the robot, or the heading line if there is no usable plan."""
    if plan is not None and len(plan) >= 2:
        p = np.asarray(plan, dtype=float)
        d = np.hypot(p[:, 0] - robot_xy[0], p[:, 1] - robot_xy[1])
        i0 = int(np.argmin(d))
        if d[i0] <= PLAN_NEAR:
            pts = np.vstack(([robot_xy[0], robot_xy[1]], p[i0 + 1:]))
            if len(pts) >= 2:
                seg = np.hypot(np.diff(pts[:, 0]), np.diff(pts[:, 1]))
                return np.concatenate(([0.0], np.cumsum(seg))), pts[:, 0], pts[:, 1]
    hx, hy = math.cos(robot_yaw), math.sin(robot_yaw)
    return (np.array([0.0, 20.0]), np.array([robot_xy[0], robot_xy[0] + 20 * hx]),
            np.array([robot_xy[1], robot_xy[1] + 20 * hy]))


def travel(v0, v_top, accel, n, stop_after=None, delay=0.0, step=0.0):
    """Distance driven at each step: (from a stop, after `delay` s, stepping to
    `step` m/s) speeding up to v_top, or, with stop_after, braking (after
    LATENCY) from that time on."""
    s, v, out = 0.0, max(v0, 0.0), np.empty(n)
    wait = delay if v < 0.05 else 0.0
    for k in range(n):
        out[k] = s
        tk = k * DT
        if stop_after is not None and tk >= stop_after + LATENCY:
            v2 = max(0.0, v - BRAKE * DT)
        elif tk < wait:
            v2 = v
        elif v < v_top:
            v = max(v, min(step, v_top))
            v2 = min(v_top, v + accel * DT)
        else:
            v2 = v_top
        s += 0.5 * (v + v2) * DT
        v = v2
    return out


def stopping_distance(v):
    """How far the robot rolls after a WAIT from speed v (latency, then braking)."""
    return v * LATENCY + v * v / (2 * BRAKE)


class Zone:
    """A crossing person's zone on the robot's path (distances along the path):
    s_hold = the stop line, s_band = where their walkway (within SAFE of where they
    have walked) begins, s_out = where the zone ends."""

    def __init__(self, s_hold, s_band, s_out, who):
        self.s_hold, self.s_band, self.s_out, self.who = s_hold, s_band, s_out, who


class Look:
    """What the guard sees ahead this scan (all distances are robot-person centres)."""

    def __init__(self):
        self.d_go = self.d_stop = 9.0     # going on / stopping now: closest pass to anyone
        self.d_go_nom = 9.0               # going on at normal speed (to compare with stopping)
        self.d_exit = 9.0                 # v10.8: going on to the end of the zone it is in, then stopping
        self.who = None                   # the person who sets d_go
        self.who_stop = None
        self.can_go_for = HORIZON         # (people along the path) s it can keep going and still stop SAFE
        self.d_go_al = self.d_stop_al = self.d_go_nom_al = 9.0   # the same, people along the path only
        self.zone = None                  # first crossing zone ahead (Zone) or None
        self.inside = False               # robot centre already within SAFE of someone's walkway
        self.fresh = True                 # everyone within FRESH_R seen < STALE s ago
        self.room = 99.0                  # m it can still drive and stop before the stop line
        self.go_path = None               # (n, 2) where the robot would drive
        self.path_xy = None               # (M, 2) the path ahead, 5 cm steps (for markers)
        self.futs = {}                    # person id -> [futures]
        self.crossers = set()             # ids of people walking across the path
        self.need = SAFE                  # clearance a crossing needs now (SAFE + margin)
        self.d_comf = 9.0                 # comfort: COMFORT_FRONT x the worst distance / required
        self.comfy = True                 # comfortable enough to start a crossing now
        self.soft = False                 # predictions made with the gentle robot (speed limit)
        self.prog_left = 99.0             # s before Nav2's progress checker would give up
        self.edge = False                 # waited COMFORT_WAIT s: the stop line has moved up


def look_ahead(robot_xy, robot_yaw, robot_v, cruise, people, static_map, t, safe=SAFE, plan=None,
               crossers=frozenset(), soft=False, pad=ZONE_PAD):
    n = int(round(HORIZON / DT)) + 1
    lk = Look()
    lk.crossers = set(crossers)
    lk.soft = soft
    slow, fast, nominal = soft_profiles() if soft else (SLOW, FAST, NOMINAL)
    futs, futs_al = [], []
    for tr in people:
        f = futures(tr, static_map, t, n)
        if f:
            lk.futs[tr.id] = f
            futs += [(tr.id, x) for x in f]
            if tr.id not in crossers:
                futs_al += [(tr.id, x) for x in f]
        if (t - tr.t > STALE and math.hypot(tr.px - robot_xy[0], tr.py - robot_xy[1]) < FRESH_R):
            lk.fresh = False
    s_line, x_line, y_line = path_ahead(robot_xy, robot_yaw, plan)

    def where(s):
        return np.column_stack((np.interp(s, s_line, x_line), np.interp(s, s_line, y_line)))

    def closest(s, which):
        pos = where(s)
        best, who = 9.0, None
        for pid, f in which:
            d = float(np.min(np.hypot(f[:, 0] - pos[:, 0], f[:, 1] - pos[:, 1])))
            if d < best:
                best, who = d, pid
        return best, who

    def comfort(s, which):
        """COMFORT_FRONT x the smallest (distance / distance required) over all futures:
        >= COMFORT_FRONT means every moment keeps the comfort distance for where the robot
        is relative to the person (in front: COMFORT_FRONT, behind: COMFORT_BEHIND)."""
        pos = where(s)
        best = 9.0
        for pid, f in which:
            rel = pos - f
            d = np.hypot(rel[:, 0], rel[:, 1])
            h = np.diff(f, axis=0, append=f[-1:])                 # where they walk next
            hn = np.hypot(h[:, 0], h[:, 1])
            moving = hn > 0.3 * DT                                # walking (> 0.3 m/s)
            cos = np.where(moving, (rel[:, 0] * h[:, 0] + rel[:, 1] * h[:, 1])
                           / np.maximum(d * hn, 1e-9), 1.0)       # standing: as if in front
            req = COMFORT_BEHIND + (COMFORT_FRONT - COMFORT_BEHIND) * (1.0 + cos) / 2.0
            best = min(best, float(np.min(COMFORT_FRONT * d / req)))
        return best

    def drive(profile, stop_after=None):
        k, a, delay, step = profile
        return travel(robot_v, k * cruise, a, n, stop_after=stop_after, delay=delay, step=step)

    lk.go_path = where(drive(nominal))
    S = np.arange(0.0, min(ZONE_LOOK, float(s_line[-1])) + 1e-9, 0.05)
    lk.path_xy = where(S)
    if not futs:
        return lk
    for prof in (slow, fast):
        path_s = drive(prof)
        d, who = closest(path_s, futs)
        if d < lk.d_go:
            lk.d_go, lk.who = d, who
        lk.d_comf = min(lk.d_comf, comfort(path_s, futs))
    lk.d_go_nom = closest(drive(nominal), futs)[0]
    lk.d_stop, lk.who_stop = closest(drive(nominal, stop_after=0.0), futs)
    # people walking along the path: v5.1's rules (Nav2 sees them and steers round)
    if futs_al:
        lk.d_go_al = min(closest(drive(slow), futs_al)[0], closest(drive(fast), futs_al)[0])
        lk.d_go_nom_al = closest(drive(nominal), futs_al)[0]
        lk.d_stop_al = closest(drive(nominal, stop_after=0.0), futs_al)[0]
        if lk.d_go_al < safe:
            lk.can_go_for = None
            for tau in STOP_CHECKS:
                if closest(drive(fast, stop_after=tau), futs_al)[0] < safe:
                    break
                lk.can_go_for = tau
    # people walking across: their crossing zone on the path (where they have walked and
    # may walk: the stop line is SAFE + ZONE_PAD before it) and their walkway (where they
    # have walked: the robot never stops in it, and once it can't stop before it, it commits)
    hp, hi, fp, fi = [], [], [], []
    for tr in people:
        if tr.id in crossers and tr.id in lk.futs:
            w = walkway(tr, t)
            hp.append(w)
            hi += [tr.id] * len(w)
            for f in lk.futs[tr.id]:
                fp.append(f)
                fi += [tr.id] * len(f)
    if (hp or fp) and len(S):
        P = lk.path_xy

        def dist_to(pts):
            if not len(pts):
                return np.full(len(S), 99.0), np.zeros(len(S), dtype=int)
            D2 = (P[:, None, 0] - pts[None, :, 0]) ** 2 + (P[:, None, 1] - pts[None, :, 1]) ** 2
            k = np.argmin(D2, axis=1)
            return np.sqrt(D2[np.arange(len(S)), k]), k
        Hp = np.vstack(hp) if hp else np.zeros((0, 2))
        Fp = np.vstack(fp) if fp else np.zeros((0, 2))
        Dh, kh = dist_to(Hp)
        Df, kf = dist_to(Fp)
        D = np.minimum(Dh, Df)
        hold = D < SAFE + pad
        band = Dh < SAFE
        lk.inside = bool(band[0])
        if hold.any():
            i = int(np.argmax(hold))
            j = i
            while j + 1 < len(S) and hold[j + 1:j + 5].any():   # bridge gaps under 0.2 m
                j += 1
            b = np.nonzero(band[i:j + 1])[0]
            who = int(hi[kh[i]]) if Dh[i] <= Df[i] else int(fi[kf[i]])
            lk.zone = Zone(float(S[i]), float(S[i + b[0]]) if len(b) else None, float(S[j]), who)
    # v10.8: going on at normal speed to the end of the first zone, then stopping. An emergency
    # stop is for a contact the crossing itself would bring; a conflict further on (someone at the
    # next walkway) is met by stopping after this one, not inside it (shift_v106a t 140: stopped in
    # the worker's walkway for the visitor 3 m on, then had to clear it as she came back: 0.03 m)
    lk.d_exit = lk.d_go_nom
    if lk.zone is not None:
        s_nom = drive(nominal)
        hit = np.nonzero(s_nom >= lk.zone.s_out)[0]
        if len(hit):
            lk.d_exit = closest(drive(nominal, stop_after=float(hit[0]) * DT), futs)[0]
    return lk


class Vehicle:
    """A tugger or forklift as the lidar sees it: the points of its visible faces, followed by
    their centre; velocity = least-squares slope of the centre over VEH_WINDOW s."""
    _next = 1

    def __init__(self, pts, idx, t):
        self.id = Vehicle._next
        Vehicle._next += 1
        self.hist = collections.deque()
        self.moving = False
        self.moved_t = None          # last time it was moving
        self.max_len = 0.0
        self.hits = 0
        self.pts_hist = collections.deque()   # (t, points) over the last 2 s
        self.change_t = None         # last time its points changed in free space
        self.vx = self.vy = 0.0
        self.axis = None             # unit vector of the way it last moved
        self.stop_t = None           # when it last stopped
        self.yielding = False        # it stopped with the robot just ahead of it
        self.update(pts, idx, t)

    @property
    def speed(self):
        return math.hypot(self.vx, self.vy)

    def pts_at(self, t_old):
        """Its points as seen closest to t_old (within 0.3 s), or None."""
        best = min(self.pts_hist, key=lambda h: abs(h[0] - t_old), default=None)
        return None if best is None or abs(best[0] - t_old) > 0.3 else best[1]

    def update(self, pts, idx, t, change=0.0):
        self.pts, self.idx, self.t = pts, idx, t
        self.hits += 1
        self.pts_hist.append((t, pts))
        while self.pts_hist and t - self.pts_hist[0][0] > 2.0:
            self.pts_hist.popleft()
        if change >= VEH_CHANGE:
            self.change_t = t
        changing = self.change_t is not None and t - self.change_t <= VEH_CHANGE_KEEP
        self.cx, self.cy = float(pts[:, 0].mean()), float(pts[:, 1].mean())
        self.max_len = max(self.max_len, float(np.hypot(*np.ptp(pts, axis=0))))
        self.hist.append((t, self.cx, self.cy))
        while self.hist and t - self.hist[0][0] > VEH_WINDOW:
            self.hist.popleft()
        if len(self.hist) >= 4 and self.hist[-1][0] - self.hist[0][0] >= 0.4:
            T = np.array([h[0] for h in self.hist])
            T = T - T.mean()
            X = np.array([h[1] for h in self.hist])
            Y = np.array([h[2] for h in self.hist])
            den = float((T * T).sum())
            self.vx = float((T * (X - X.mean())).sum() / den)
            self.vy = float((T * (Y - Y.mean())).sum() / den)
        was = self.moving
        if self.speed > VEH_MOVING and self.hits >= VEH_MIN_HITS and changing:
            self.moving = True
        elif self.speed < VEH_STOPPED or not changing:
            self.moving = False
        if self.moving:
            self.moved_t = t
            self.axis = (self.vx / self.speed, self.vy / self.speed)
            self.stop_t, self.yielding = None, False
        elif was:
            self.stop_t = t

    def recently_moved(self, t):
        return self.moved_t is not None and t - self.moved_t <= VEH_RECENT

    def outline(self, n=16):
        """Up to n of its points, plus the same points VEH_LEAD ahead of and behind it along the
        way it moves (forks lead when it drives forwards and trail when it reverses)."""
        p = self.pts[np.linspace(0, len(self.pts) - 1, min(n, len(self.pts))).astype(int)]
        if self.speed > VEH_STOPPED:
            u = np.array([self.vx, self.vy]) / self.speed
            p = np.vstack((p, p + u * VEH_LEAD, p - u * VEH_LEAD))
        return p

    def standing(self, t):
        """Has moved, stands now: picking, lifting, turning or waiting."""
        return not self.moving and self.recently_moved(t)

    def banded(self, t):
        """Its forks are taken to lie along the way it last moved (else: anywhere round it)."""
        return self.axis is not None and (self.yielding or self.stop_t is None or
                                          t - self.stop_t >= VEH_WORK)

    def band(self, robot_xy):
        """(u, n, a0, a1, b0, b1): the band its forks may cover, in its own frame (u along the
        way it last moved, n across): what the lidar sees of it, at least VEH_WIDTH wide (the
        unseen side is away from the robot), VEH_LEAD longer at both ends."""
        u = np.array(self.axis)
        n = np.array((-u[1], u[0]))
        a, b = self.pts @ u, self.pts @ n
        a0, a1, b0, b1 = float(a.min()) - VEH_LEAD, float(a.max()) + VEH_LEAD, float(b.min()), float(b.max())
        if b1 - b0 < VEH_WIDTH:
            if float(np.dot(robot_xy, n)) <= 0.5 * (b0 + b1):
                b1 = b0 + VEH_WIDTH
            else:
                b0 = b1 - VEH_WIDTH
        return u, n, a0, a1, b0, b1

    def keep_dist(self, q, t, robot_xy):
        """How far points q (k, 2) are inside (< 0) or outside its keep-out."""
        if self.banded(t):
            u, n, a0, a1, b0, b1 = self.band(robot_xy)
            qa, qb = q @ u, q @ n
            da = np.maximum(np.maximum(a0 - qa, qa - a1), 0.0)
            db = np.maximum(np.maximum(b0 - qb, qb - b1), 0.0)
            return np.hypot(da, db) - VEH_BAND
        d = np.min(np.hypot(self.pts[None, :, 0] - q[:, None, 0], self.pts[None, :, 1] - q[:, None, 1]), axis=1)
        return d - VEH_KEEP


def robot_ahead(v, robot_xy):
    """The robot within VEH_YIELD_D ahead of a vehicle along the way it was moving, and not
    far to the side of it: what a driver stops for."""
    if v.axis is None:
        return False
    u = np.array(v.axis)
    n = np.array((-u[1], u[0]))
    r = np.asarray(robot_xy, dtype=float)
    a, b = v.pts @ u, v.pts @ n
    ra, rb = float(r @ u), float(r @ n)
    return 0.0 < ra - float(a.max()) < VEH_YIELD_D and float(b.min()) - 0.8 < rb < float(b.max()) + 0.8


def body_gap(robot_xy, robot_yaw, px, py, ahead=0.0):
    """v10.3: (gap, along) between the robot's body (ROBOT_LEN x ROBOT_WID, moved `ahead` m along
    its heading) and a person's (PERSON_BODY) at (px, py); gap < 0 overlaps; along: how far ahead
    of the robot's centre the person is."""
    c, s = math.cos(robot_yaw), math.sin(robot_yaw)
    dx, dy = px - robot_xy[0], py - robot_xy[1]
    lx = c * dx + s * dy
    ly = -s * dx + c * dy
    ex = max(abs(lx - ahead) - ROBOT_LEN / 2, 0.0)
    ey = max(abs(ly) - ROBOT_WID / 2, 0.0)
    return math.hypot(ex, ey) - PERSON_BODY, lx


def start_blocked(robot_xy, robot_yaw, people):
    """v10.3: would setting off now bring the robot's body within START_GAP of someone's in the
    next START_LOOK s, and closer than staying put would (people predicted along their walking
    line)? Staying in someone's way is no better (v6: never stop in a person's path), so a GO that
    gets the robot out of their way is not held back. -> (track id, body gap if it stays, smallest
    body gap if it sets off) or None"""
    for tr in people:
        g_now, _ = body_gap(robot_xy, robot_yaw, tr.px, tr.py)
        if g_now > START_GAP + V_START * START_LOOK[-1] + 1.5 * START_LOOK[-1]:
            continue                                  # too far to matter in START_LOOK s
        g_go = g_stay = g_now
        for tau in START_LOOK:
            adv = V_START * max(tau - 0.2, 0.0)
            px, py = tr.px + tr.wvx * tau, tr.py + tr.wvy * tau
            g_go = min(g_go, body_gap(robot_xy, robot_yaw, px, py, adv)[0])
            g_stay = min(g_stay, body_gap(robot_xy, robot_yaw, px, py)[0])
        if g_go < START_GAP and g_go < g_stay - START_WORSE:
            return tr.id, g_stay, g_go
    return None


def prox_limit(robot_xy, robot_yaw, people, standing, still, cruise):
    """v10.3 speed and separation: going on must not close in on anyone fast. For everyone not
    behind the robot - people it predicts (now and 0.5 s ahead), standing people the camera
    confirmed, and people the lidar saw walk who now stand still - the body gap g and how fast it
    shrinks per metre the robot drives on (closing: 1 straight at them, 0 passing alongside, < 0
    moving away). The robot's speed towards them is held to V_PROX at PROX_GAP0 rising to cruise
    at PROX_GAP1: speed <= v(g) / closing. Moving away or alongside is not capped, so the robot
    can still clear a walkway, except within PROX_GAP0 (touching distance) of someone ahead of the
    robot's middle, where it crawls at V_PROX unless it is clearly moving away (beside or behind
    its middle, driving on is what gets it out of their way). -> the cap (m/s) or None."""
    pts = [(tr.px, tr.py) for tr in people] + [(tr.px + 0.5 * tr.wvx, tr.py + 0.5 * tr.wvy) for tr in people]
    pts += [(x, y) for x, y, _, _ in standing]
    pts += [(tr.x, tr.y) for tr in still]
    cap = None
    for px, py in pts:
        g, along = body_gap(robot_xy, robot_yaw, px, py)
        if along < -(ROBOT_LEN / 2 + PERSON_BODY) or g >= PROX_GAP1:
            continue                                  # behind (the robot moves away), or far
        g2, _ = body_gap(robot_xy, robot_yaw, px, py, PROX_STEP)
        closing = (g - g2) / PROX_STEP
        if g < PROX_GAP0 and along > 0.0 and closing > -0.3:
            v = V_PROX                                # touching distance, ahead of its middle: crawl
        elif closing <= 0.05:
            continue                                  # going on doesn't bring it closer
        else:
            f = min(max((g - PROX_GAP0) / (PROX_GAP1 - PROX_GAP0), 0.0), 1.0)
            v = (V_PROX + (cruise - V_PROX) * f) / min(closing, 1.0)
        if v < cruise - 1e-3:
            cap = v if cap is None else min(cap, v)
    return cap


def touch_hold(robot_xy, robot_yaw, people, standing, still, held=None):
    """v10.11 (shift_v108a, t 98.6-99.4): someone within PROX_GAP0 (touching distance) ahead of the
    robot's middle, and driving on would close in on them head-on (closing > TOUCH_CLOSING): HOLD -
    a WAIT, like the vehicle hold - instead of prox_limit's crawl at V_PROX, which pushed on into
    picker_a crossing its front bumper (a v_limit of 0 would not do: Nav2's SpeedLimit reads 0 as
    no limit). Holding for someone (held = their id), it holds until their body gap opens past
    PROX_GAP0 + TOUCH_HYST, they are no longer ahead of its middle, or going on no longer closes
    in (closing <= 0.3). Beside or behind its middle nothing changes: driving on is what clears
    their way. -> (who, body gap, closing) or None"""
    cands = [(tr.px, tr.py, tr.id) for tr in people]
    cands += [(x, y, sid) for x, y, _, sid in standing]
    cands += [(tr.x, tr.y, tr.id) for tr in still]
    best = None
    for px, py, who in cands:
        keep = held is not None and who == held
        g, along = body_gap(robot_xy, robot_yaw, px, py)
        if along <= 0.0 or g >= PROX_GAP0 + (TOUCH_HYST if keep else 0.0):
            continue
        g2, _ = body_gap(robot_xy, robot_yaw, px, py, PROX_STEP)
        closing = (g - g2) / PROX_STEP
        if closing > (0.3 if keep else TOUCH_CLOSING) and (best is None or g < best[1]):
            best = (who, g, closing)
    return best


def behind_following(v, robot_xy, robot_yaw):
    """A vehicle wholly behind the robot, coming the same way (within 45 deg of its heading):
    stopping in front of it would not help (keeping back is its driver's job), so it gets no
    WAIT and no speed cap."""
    hx, hy = math.cos(robot_yaw), math.sin(robot_yaw)
    ahead = float(np.max((v.pts[:, 0] - robot_xy[0]) * hx + (v.pts[:, 1] - robot_xy[1]) * hy))
    return ahead < -0.3 and v.vx * hx + v.vy * hy > 0.7 * v.speed


def seen_through(pts, ranges, angle_min, angle_inc, range_max, laser_pose, margin=0.3, spread=0):
    """Share of these map points a scan sees past (its beam goes more than margin further):
    nothing is there. With spread, the beams that many either side must go past too (a beam
    grazing along a face can slip past it). Points out of the scan's reach don't count."""
    lx, ly, lyaw = laser_pose
    d = np.hypot(pts[:, 0] - lx, pts[:, 1] - ly)
    a = np.arctan2(pts[:, 1] - ly, pts[:, 0] - lx) - lyaw - angle_min
    n = len(ranges)
    i = np.mod(np.round(a / angle_inc).astype(int), n)
    rr = np.asarray(ranges, dtype=float)
    rr = np.where(np.isfinite(rr), rr, np.inf)
    r = rr[i]
    for k in range(1, spread + 1):
        r = np.minimum(r, np.minimum(rr[np.mod(i - k, n)], rr[np.mod(i + k, n)]))
    ok = d < range_max - 0.5
    if not np.any(ok):
        return 0.0
    past = r > d + margin
    return float(np.mean(past[ok]))


def vehicle_candidates(pts, idx, n_rays):
    """Off-map points -> [(points, ray indices)] of blobs that look like vehicles: long, or wide
    with a continuous face. Legs are two small arcs with a gap between them."""
    out = []
    for g in blobs(pts):
        if len(g) < VEH_MIN_PTS:
            continue
        b = pts[g]
        length = float(np.hypot(*np.ptp(b, axis=0)))
        if length > VEH_MAX_LEN:
            continue
        rays = np.sort(idx[g])
        gaps = np.diff(rays)
        if len(gaps) and gaps.max() > n_rays / 2:      # the blob straddles the scan's seam
            rays = np.sort((rays + n_rays // 2) % n_rays)
        span = int(rays[-1] - rays[0]) + 1
        fill = len(set(rays.tolist())) / max(span, 1)
        if length >= VEH_MIN_LEN or (length >= VEH_SOLID_W and fill >= VEH_SOLID_FILL):
            out.append((b, idx[g]))
    return out


class VehicleLook:
    """The vehicle layer's view this scan."""

    def __init__(self):
        self.d_go = self.d_stop = 9.0     # going on / stopping now: closest pass to any part of a vehicle
        self.who = None
        self.near = 9.0                   # closest moving (or recently moving) vehicle now
        self.near_v = 0.0                 # its speed
        self.d_now = 9.0                  # closest a moving vehicle (padded) is now
        self.conflict = False             # going on would come within VEH_SAFE, closer than now
        self.clear_it = False             # ... but the robot is in its way and stopping is worse
        self.keep_out = False             # the path ahead goes closer than VEH_KEEP to a standing vehicle
        self.keep_who = None
        self.keep_d = 9.0                 # how far the path ahead stays out of a standing vehicle's
                                          # keep-out (< 0: it goes in)


def vehicle_look(robot_xy, robot_yaw, robot_v, cruise, vehicles, t, plan=None, soft=False):
    vl = VehicleLook()
    behind = {v.id for v in vehicles if v.moving and behind_following(v, robot_xy, robot_yaw)}
    moving = [v for v in vehicles if v.moving and v.id not in behind]
    for v in vehicles:
        if (v.moving or v.recently_moved(t)) and v.id not in behind:
            d = float(np.min(np.hypot(v.pts[:, 0] - robot_xy[0], v.pts[:, 1] - robot_xy[1])))
            if d < vl.near:
                vl.near, vl.near_v = d, v.speed
    s_line, x_line, y_line = path_ahead(robot_xy, robot_yaw, plan)
    standing = [v for v in vehicles if v.standing(t)]
    if standing:
        sk = np.arange(0.0, VEH_KEEP_LOOK + 1e-9, 0.1)
        pk = np.column_stack((np.interp(sk, s_line, x_line), np.interp(sk, s_line, y_line)))
        r0 = np.array([robot_xy], dtype=float)
        for v in standing:
            k_now = float(v.keep_dist(r0, t, robot_xy)[0])
            k_path = float(np.min(v.keep_dist(pk, t, robot_xy)))
            if k_path < vl.keep_d:
                vl.keep_d = k_path
            if k_path < 0.0 and k_path < k_now - 0.05:
                vl.keep_out, vl.keep_who = True, v.id   # driving on would take it further in: wait
    if not moving:
        return vl
    n = int(round(VEH_HORIZON / DT)) + 1
    tt = np.arange(n) * DT

    def where(s):
        return np.column_stack((np.interp(s, s_line, x_line), np.interp(s, s_line, y_line)))

    def drive(profile, stop_after=None):
        k, a, delay, step = profile
        return travel(robot_v, k * cruise, a, n, stop_after=stop_after, delay=delay, step=step)

    slow, fast, nominal = soft_profiles() if soft else (SLOW, FAST, NOMINAL)
    futs = []
    for v in moving:
        o = v.outline()                                            # (m, 2)
        f = o[None, :, :] + tt[:, None, None] * np.array([v.vx, v.vy])[None, None, :]
        futs.append((v.id, f))                                     # (n, m, 2)

    def closest(s):
        pos = where(s)
        best, who = 9.0, None
        for vid, f in futs:
            d = float(np.min(np.hypot(f[:, :, 0] - pos[:, None, 0], f[:, :, 1] - pos[:, None, 1])))
            if d < best:
                best, who = d, vid
        return best, who

    for prof in (slow, fast):
        d, who = closest(drive(prof))
        if d < vl.d_go:
            vl.d_go, vl.who = d, who
    vl.d_stop = closest(drive(nominal, stop_after=0.0))[0]
    for vid, f in futs:
        vl.d_now = min(vl.d_now, float(np.min(np.hypot(f[0, :, 0] - robot_xy[0], f[0, :, 1] - robot_xy[1]))))
    vl.conflict = vl.d_go < VEH_SAFE and vl.d_go < vl.d_now - 0.05
    vl.clear_it = vl.conflict and robot_v > 0.1 and vl.d_stop < vl.d_go - 0.2
    return vl


class Guard:
    """GO / WAIT: stop line and commit for people crossing the path, v5.1's rules for
    people walking along it, hysteresis, a comfort target for crossings, patience
    (held without 0.5 m of progress: after COMFORT_WAIT s SAFE is accepted instead of
    comfort, WAIT_MAX s later SAFE_PATIENT instead of SAFE), and the speed limit
    near people (v_limit, m/s; None = no limit)."""

    def __init__(self):
        self.anchor = None
        self.held = 0.0
        self.last_t = None
        self.state = 'go'
        self.steady = 0.0        # s that going has looked fine while waiting
        self.why = 'clear'
        self.emergency = 0       # scans in a row that called for an emergency stop
        self.v_limit = None      # speed limit sent to Nav2 (m/s), None = no limit
        self.prog_xy = None      # mirror of Nav2's progress checker: where and when the robot
        self.prog_t = None       # last made PROG_RADIUS of progress
        self.prog_radius, self.prog_time = PROG_RADIUS, PROG_TIME   # the node sets Nav2's values
        self.edged = False       # has edged up at this stop already (only once)
        self.start_block = False  # v10.3: a GO was held back by the start floor this scan

    def step(self, robot_xy, robot_yaw, robot_v, cruise, people, static_map, t, plan=None,
             crossers=frozenset()):
        dt = 0.0 if self.last_t is None else min(max(t - self.last_t, 0.0), 0.5)
        self.last_t = t
        self.start_block = False
        if self.anchor is None or math.hypot(robot_xy[0] - self.anchor[0],
                                             robot_xy[1] - self.anchor[1]) > 0.5:
            self.anchor, self.held, self.edged = tuple(robot_xy), 0.0, False
        elif self.state == 'wait':
            self.held += dt
            if robot_v < 0.05:
                # v7.1: measure from where it actually stopped (on the way in the anchor
                # trails by up to 0.5 m, and the 0.2 m edge-up then reset the wait:
                # guard7_lanes edged up twice at one stop and waited 6 s more)
                self.anchor = tuple(robot_xy)
        patient = self.held < WAIT_MAX + (COMFORT_WAIT if PATIENCE_AFTER_COMFORT else 0.0)
        safe = SAFE if patient else SAFE_PATIENT
        near = any(math.hypot(tr.px - robot_xy[0], tr.py - robot_xy[1]) < NEAR_R for tr in people)
        edge = self.held >= COMFORT_WAIT          # waited long enough: the stop line moves up
        lk = look_ahead(robot_xy, robot_yaw, robot_v, cruise, people, static_map, t,
                        safe=safe, plan=plan, crossers=crossers,
                        soft=near or self.v_limit is not None,
                        pad=ZONE_PAD_WAITED if edge else ZONE_PAD)
        lk.edge = edge
        # a crossing must keep SAFE + margin, and be comfortable (until COMFORT_WAIT s at a stop)
        lk.need = safe + GO_MARGIN
        lk.comfy = self.held >= COMFORT_WAIT or lk.d_comf >= COMFORT_FRONT
        # Nav2's progress checker: don't start moving just before it gives up on the goal
        if self.prog_xy is None or math.hypot(robot_xy[0] - self.prog_xy[0],
                                              robot_xy[1] - self.prog_xy[1]) > self.prog_radius:
            self.prog_xy, self.prog_t = tuple(robot_xy), t
        deadline = self.prog_t + self.prog_time
        if t >= deadline + PROG_RECOVERY:          # it has given up and started again by now
            self.prog_xy, self.prog_t = tuple(robot_xy), deadline + PROG_RECOVERY
            deadline = self.prog_t + self.prog_time
        lk.prog_left = deadline - t
        prog_ok = lk.prog_left >= self.time_to(self.prog_radius, cruise) + PROG_MARGIN
        z = lk.zone
        stop_d = stopping_distance(robot_v)
        lk.room = 99.0 if z is None else z.s_hold - stop_d
        band_room = 99.0 if (z is None or z.s_band is None) else z.s_band - stop_d
        can = lk.can_go_for
        al_keep = lk.d_go_al >= safe or (can is not None and can >= KEEP_MOVE)
        al_start = lk.d_go_al >= safe + GO_MARGIN or (can is not None and can >= START_MOVE)
        near_zone = (z is not None and z.s_hold < CROSS_NEAR) or lk.inside
        lk.start = None
        if self.state == 'go' and robot_v < START_STILL:
            lk.start = start_blocked(robot_xy, robot_yaw, people)   # v10.3: standing in GO
            if lk.start is not None:
                self.start_block = True
                self.state, self.why, self.steady = 'wait', 'start', 0.0
        if self.state == 'go':
            crossing_now = self.why in ('crossing', 'committed', 'clearing')
            tol = KEEP_TOL if crossing_now else 0.0
            if not (lk.d_go < safe - tol and (lk.inside or band_room <= 0.0)):
                self.emergency = 0
            # carrying on with a crossing needs SAFE; starting one also needs comfort
            if (lk.d_go >= safe - tol if crossing_now else lk.d_go >= lk.need and lk.comfy):
                self.why = 'crossing' if near_zone else 'clear'
            elif lk.inside or band_room <= 0.0:
                # can't stop before the zone any more: finish the crossing
                if lk.d_go_nom < EMERGENCY and lk.d_stop > max(lk.d_go_nom, lk.d_exit) + EMERGENCY_GAIN:
                    self.emergency += 1
                else:
                    self.emergency = 0
                if self.emergency >= EMERGENCY_SCANS:
                    self.state, self.why, self.steady = 'wait', 'emergency', 0.0
                else:
                    self.why = 'committed'
            elif al_keep and (lk.room >= robot_v * DT if robot_v >= 0.1 else
                              lk.room >= (EDGE_MIN if edge else START_MOVE_DIST)):
                self.why = 'approach'                     # on to the stop line (no creeping)
            elif (lk.d_stop_al < safe and lk.d_go_nom_al > lk.d_stop_al + IN_WAY_KEEP
                  and lk.room > 0.5):
                self.why = 'in_way'                       # along: stopping here would be worse
            else:
                self.state, self.why, self.steady = 'wait', 'yield', 0.0
        else:
            want = None
            if lk.start is None:
                lk.start = start_blocked(robot_xy, robot_yaw, people)   # v10.3
            self.start_block = lk.start is not None
            if self.start_block:
                pass                                      # no GO of any kind: it would close in on someone
            elif lk.inside:
                if lk.d_go_nom >= lk.d_stop - LEAVE_TOL and lk.fresh:
                    want = 'clearing'                     # don't stand in a walkway
            elif lk.d_go >= lk.need and lk.comfy and lk.fresh and prog_ok:
                want = 'crossing' if near_zone else 'clear'
            elif (lk.room >= START_MOVE_DIST or edge and not self.edged and lk.room >= EDGE_MIN) \
                    and al_start and prog_ok:
                want = 'approach'                         # (or edge up once to the moved-up line)
            elif (lk.d_stop_al < safe and lk.d_go_nom_al > lk.d_stop_al + CLEAR_GAIN
                  and lk.room > 0.5):
                self.state, self.why = 'go', 'in_way'     # along: standing in their way, clear it
            if self.state == 'wait':
                if want is not None:
                    self.steady += dt
                    # v10.8: in patience, one scan that keeps SAFE_PATIENT + margin is enough for a
                    # crossing (a pacing walker leaves the same short gap every cycle: two scans in
                    # a row may never come, shift_v106a waited 64 s for 3 mm)
                    steady = PATIENT_STEADY if (not patient and want in ('crossing', 'clear')) else GO_STEADY
                    if self.steady >= steady - 1e-6:
                        self.state, self.why = 'go', want
                        if want == 'approach' and lk.room < START_MOVE_DIST:
                            self.edged = True             # the one edge-up at this stop
                else:
                    self.steady = 0.0
        self.limit(dt, robot_v, cruise, lk, near)
        return self.state, lk, patient

    def time_to(self, dist, cruise):
        """s the gentle, slow robot needs from a stop to cover dist."""
        k, a, delay, step = soft_profiles()[0]
        s = travel(0.0, k * cruise, a, int(round(HORIZON / DT)) + 1, delay=delay, step=step)
        i = int(np.argmax(s >= dist)) if np.any(s >= dist) else len(s) - 1
        return i * DT

    def limit(self, dt, robot_v, cruise, lk, near):
        """The speed limit for Nav2 after this decision: V_START while waiting, easing
        off towards a stop line, otherwise cruise; it falls at once but rises at most
        A_SOFT m/s per second, and only while the robot keeps up with it (after a GO the
        robot needs ~0.2 s to start, and the limit must not run ahead of it). With
        nobody within NEAR_R and the limit back at cruise, no limit at all."""
        if LIFT_INSIDE and self.state == 'go' and lk.inside:
            self.v_limit = None                         # in a walkway: get out of it
            return None
        if self.state == 'wait':
            target = V_START
        elif self.why == 'approach' and lk.zone is not None:
            target = math.sqrt(V_CREEP ** 2 + 2.0 * D_EASE * max(lk.zone.s_hold - CREEP_LEN, 0.0))
            if lk.edge:
                target = min(target, V_CREEP)           # edging up after a long wait: creep
        else:
            target = cruise
        target = min(target, cruise)
        if self.v_limit is None:
            if not near or target >= cruise - 1e-6 and robot_v >= cruise - 0.05:
                return None
            self.v_limit = max(min(robot_v, cruise), V_START)
        if target < self.v_limit:
            self.v_limit = target                       # slows at once
        elif robot_v >= self.v_limit - 0.1:             # rises only while the robot keeps up
            self.v_limit = min(target, self.v_limit + A_SOFT * dt)
        if not near and self.v_limit >= cruise - 1e-6:
            self.v_limit = None                         # nobody near, ramp done
        return self.v_limit


class GuardCore:
    """Everything except ROS: one call per scan."""

    def __init__(self, static_map=None):
        self.map = static_map
        self.tracker = Tracker()
        self.guard = Guard()
        self.people = []          # everyone the guard predicts
        self.along = set()        # ids of those walking along the robot's path (Nav2 sees them)
        self.crossers = set()     # ids of those walking across it (crossing zone, hidden from Nav2)
        self.cam_only = []        # v9: camera people the lidar tracker has no blob for (dicts)
        self.standing = []        # v9: (x, y, radius, track id or 0) standing people the camera confirmed
        self.cap = None           # v9: speed cap for passing them (m/s), None = none
        self.look = None
        self.vehicles = []        # v10: vehicle tracks
        self.vlook = VehicleLook()
        self.veh_hold = False     # v10: giving way to a vehicle
        self.veh_clear_t = None   # v10: since when the way has been clear
        self.veh_cap = None       # v10: speed cap near moving vehicles
        self.prox = None          # v10.3: speed-and-separation cap from the nearest body gap
        self.touch = None         # v10.11: (who, gap, closing) while holding at touching distance, head-on
        self.touch_wait = False   # v10.11: ... and that hold is what makes it WAIT (else another reason does)
        self.veh_zones = []       # v10: standing vehicles' fork ground, shown to Nav2
        self.scan_hist = collections.deque()   # v10: (t, ranges, laser, angle_min, angle_inc, range_max)

    def scan(self, ranges, angle_min, angle_inc, range_min, range_max, laser_pose, t,
             robot_pose=None, plan=None):
        """Track people in one scan. Returns the ray indices to hide from Nav2."""
        robot = robot_pose if robot_pose is not None else laser_pose

        def across(tr):
            return crossing(tr, robot[:2], robot[2], plan)
        veh_hide = self.see_vehicles(ranges, angle_min, angle_inc, range_min, range_max,
                                     laser_pose, t, robot, plan) if USE_VEHICLES else []
        self.people, hide = see(self.tracker, self.map, ranges, angle_min, angle_inc,
                                range_min, range_max, laser_pose, t, across)
        self.crossers = {tr.id for tr in self.people if across(tr)}
        self.along = {tr.id for tr in self.people} - self.crossers
        if len(veh_hide):
            hide = np.union1d(np.asarray(hide, dtype=int), np.asarray(veh_hide, dtype=int))
        return hide

    def see_vehicles(self, ranges, angle_min, angle_inc, range_min, range_max, laser_pose, t,
                     robot, plan):
        """v10: vehicle candidates -> tracks; people tracks on a vehicle become part of it;
        returns the rays of moving vehicles crossing the path near the robot (hidden from Nav2)."""
        self.scan_hist.append((t, np.asarray(ranges, dtype=np.float32).copy(), tuple(laser_pose),
                               angle_min, angle_inc, range_max))
        while self.scan_hist and t - self.scan_hist[0][0] > 2.0:
            self.scan_hist.popleft()
        old = None                                      # the scan VEH_EVIDENCE_DT ago
        for h in self.scan_hist:
            if h[0] <= t - VEH_EVIDENCE_DT:
                old = h
        if old is not None and t - old[0] > VEH_EVIDENCE_DT + 0.5:
            old = None
        pts, idx = scan_points(ranges, angle_min, angle_inc, range_min, range_max, laser_pose)
        if self.map is not None and len(pts):
            off = ~self.map.blocked(pts[:, 0], pts[:, 1], 'scan')
            pts, idx = pts[off], idx[off]
        cands = vehicle_candidates(pts, idx, len(ranges)) if len(pts) else []
        used, keep = set(), []
        for v in self.vehicles:
            best, best_d = None, VEH_GATE
            for k, (b, bi) in enumerate(cands):
                if k in used:
                    continue
                d = math.hypot(float(b[:, 0].mean()) - v.cx - v.vx * (t - v.t),
                               float(b[:, 1].mean()) - v.cy - v.vy * (t - v.t))
                if d < best_d:
                    best, best_d = k, d
            if best is not None:
                used.add(best)
                b = cands[best][0]
                change = 0.0
                if old is not None:                     # evidence of motion in free space
                    prev = v.pts_at(old[0])
                    gone = 0.0 if prev is None else seen_through(prev, ranges, angle_min, angle_inc,
                                                                 range_max, laser_pose, 0.15, 1)
                    came = seen_through(b, old[1], old[3], old[4], old[5], old[2], 0.15, 1)
                    change = max(gone, came)
                v.update(b, cands[best][1], t, change)
            if t - v.t <= VEH_TIMEOUT:
                keep.append(v)
            elif v.standing(t) and t - v.t <= VEH_MEMORY and seen_through(
                    v.pts, ranges, angle_min, angle_inc, range_max, laser_pose) < VEH_SEEN_THROUGH:
                keep.append(v)                          # out of view where it stood: remembered
        for k, (b, bi) in enumerate(cands):
            if k not in used:
                keep.append(Vehicle(b, bi, t))
        self.vehicles = keep
        seen = [v for v in self.vehicles if v.t == t]
        for v in seen:
            if v.stop_t == t:                           # just stopped: for the robot?
                v.yielding = robot_ahead(v, robot[:2])
        for tr in self.tracker.tracks + self.tracker.lost:
            # Only a vehicle the vehicle layer is actually handling (moving, or moved in the last
            # VEH_RECENT s) may take a blob away from the people layer. A candidate that never
            # moves is just an obstacle, and its blob stays the people layer's business, so no
            # blob is left to neither layer.
            tr.vehicle = any(float(np.min(np.hypot(v.pts[:, 0] - tr.x, v.pts[:, 1] - tr.y))) < VEH_OWN_R
                             for v in seen if v.hits >= VEH_MIN_HITS
                             and (v.moving or v.recently_moved(t)))
        hide = []
        for v in seen:
            if not v.moving or math.hypot(v.cx - robot[0], v.cy - robot[1]) > VEH_HIDE_R:
                continue
            # Only while the guard is actually giving way to THIS vehicle. Otherwise Nav2 keeps
            # seeing it: it is the only layer under the robot when the vehicle layer loses track,
            # and it avoids vehicles perfectly well on its own (tour_v9 reached every goal).
            if not (self.veh_hold and self.vlook.who == v.id):
                continue
            ux, uy = path_dir_near(v.cx, v.cy, robot[:2], robot[2], plan)
            if abs(v.vx * ux + v.vy * uy) < CROSSING * v.speed:
                hide.extend(v.idx.tolist())             # crossing the path: the guard gives way
        return hide

    def vehicle_decision(self, robot_xy, robot_yaw, robot_v, cruise, t, plan, soft):
        """v10: give way to vehicles. Returns True while the robot must wait for one."""
        self.vlook = vl = vehicle_look(robot_xy, robot_yaw, robot_v, cruise, self.vehicles, t, plan, soft)
        if (vl.conflict and not vl.clear_it) or vl.keep_out:
            self.veh_hold, self.veh_clear_t = True, None
        elif self.veh_hold:
            if self.veh_clear_t is None:
                self.veh_clear_t = t
            if t - self.veh_clear_t >= VEH_CLEAR:
                self.veh_hold, self.veh_clear_t = False, None
        # v10.3: only while a vehicle matters (v10: within VEH_NEAR of any, which slowed whole laps)
        relevant = vl.d_go < VEH_CAP_GAP or vl.near < VEH_CAP_NEAR or vl.keep_d < VEH_CAP_KEEP
        self.veh_cap = V_VEHICLE if relevant and vl.near < VEH_NEAR else None
        self.veh_zones = []
        for v in self.vehicles:
            if not v.standing(t):
                continue
            if v.banded(t):
                u, n, a0, a1, b0, b1 = v.band(robot_xy)
                p = VEH_ZONE_PAD
                self.veh_zones.append(('box', u, n, a0 - p, a1 + p, b0 - p, b1 + p))
            else:
                k = np.linspace(0, len(v.pts) - 1, min(8, len(v.pts))).astype(int)
                self.veh_zones += [('circle', float(x), float(y), VEH_ZONE_R) for x, y in v.pts[k]]
        return self.veh_hold

    def camera(self, persons, t):
        """v9: people the camera saw, [(x, y, stamp)] in the map frame (camera + lidar fused).
        Each confirms the lidar track it sits on; with no track there it is kept on its own."""
        if not USE_CAMERA:
            return
        for x, y, ts in persons:
            if t - ts > CAM_MAX_AGE:
                continue
            best, best_d = None, CAM_MATCH
            for tr in self.tracker.tracks:
                d = track_dist(tr, x, y, ts)          # where the blob was when the frame was taken
                if d < best_d:
                    best, best_d = tr, d
            if best is not None:
                best.cam_t = ts if best.cam_t is None else max(best.cam_t, ts)
                continue
            # no blob there: someone the lidar follows (or just lost) is not a new, standing person
            if any(math.hypot(p.px - x, p.py - y) <= REACQUIRE for p in self.tracker.people(t)) or                     any(math.hypot(p.x - x, p.y - y) <= REACQUIRE for p in self.tracker.lost):
                continue
            old = next((c for c in self.cam_only if math.hypot(c['x'] - x, c['y'] - y) <= CAM_MATCH), None)
            if old is None:
                self.cam_only.append(dict(x=x, y=y, t=ts, t0=ts, x0=x, y0=y, frames=1))
                continue
            old.update(x=x, y=y, t=max(old['t'], ts), frames=old['frames'] + 1)
            if math.hypot(x - old['x0'], y - old['y0']) > STAND_MOVE:
                old.update(t0=ts, x0=x, y0=y)             # moved: standing starts again from here

    def standing_people(self, robot_xy, robot_yaw, t, plan=None):
        """v9: the standing people the camera confirmed: (x, y, personal radius, track id or 0)."""
        out = []
        if not USE_CAMERA:
            return out
        for tr in self.tracker.tracks:
            if tr.cam_t is None or t - tr.cam_t > CAM_KEEP or t - tr.t > TIMEOUT:
                continue
            if not is_standing(tr, t):
                continue                              # walking people: v7.1's rules
            r, _ = personal_radius(tr.x, tr.y, robot_xy, robot_yaw, self.map, plan)
            out.append((tr.x, tr.y, r, tr.id))
        self.cam_only = [c for c in self.cam_only if t - c['t'] <= CAM_ONLY_KEEP]
        for c in self.cam_only:
            x, y = c['x'], c['y']
            if c['frames'] < 3 or c['t'] - c['t0'] < CAM_ONLY_STAND or                     any(math.hypot(x - o[0], y - o[1]) <= CAM_MATCH for o in out):
                continue                              # standing as long as a lidar-tracked person
            r, _ = personal_radius(x, y, robot_xy, robot_yaw, self.map, plan)
            out.append((x, y, r, 0))
        return out

    def person_cap(self, robot_xy, robot_yaw, cruise):
        """v9: speed limit for passing standing people: V_PERSON within PASS_R, easing off at
        D_EASE on the way in; nothing once they are behind the robot."""
        cap = None
        hx, hy = math.cos(robot_yaw), math.sin(robot_yaw)
        for x, y, _, _ in self.standing:
            d = math.hypot(x - robot_xy[0], y - robot_xy[1])
            if (x - robot_xy[0]) * hx + (y - robot_xy[1]) * hy < -0.3:
                continue                              # behind: passed
            v = math.sqrt(V_PERSON ** 2 + 2.0 * D_EASE * max(d - PASS_R, 0.0))
            if v < cruise:
                cap = v if cap is None else min(cap, v)
        return cap

    def decide(self, robot_xy, robot_yaw, robot_v, cruise, t, plan=None):
        state, self.look, patient = self.guard.step(robot_xy, robot_yaw, robot_v, cruise,
                                                    self.people, self.map, t, plan, self.crossers)
        self.standing = self.standing_people(robot_xy, robot_yaw, t, plan)
        g, lk = self.guard, self.look
        if USE_VEHICLES and self.vehicle_decision(robot_xy, robot_yaw, robot_v, cruise, t, plan,
                                                  g.v_limit is not None):
            state = 'wait'                            # giving way to a vehicle
        if USE_VEHICLES and self.veh_cap is not None and not lk.inside:
            g.v_limit = self.veh_cap if g.v_limit is None else min(g.v_limit, self.veh_cap)
        self.cap = self.person_cap(robot_xy, robot_yaw, cruise)
        z = lk.zone
        if lk.inside or (g.why in ('crossing', 'committed', 'clearing') and z is not None
                         and z.s_hold < CROSS_NEAR):
            self.cap = None                           # in or about to cross a walkway: get across it first
        if self.cap is not None:
            g.v_limit = self.cap if g.v_limit is None else min(g.v_limit, self.cap)
        # v10.3 speed and separation: after everything else, also inside a walkway or committed
        ids = {tr.id for tr in self.people}
        still = [tr for tr in self.tracker.tracks if tr.moved and tr.id not in ids
                 and not getattr(tr, 'vehicle', False) and t - tr.t <= TIMEOUT]
        self.prox = prox_limit(robot_xy, robot_yaw, self.people, self.standing, still, cruise)
        if self.prox is not None:
            g.v_limit = self.prox if g.v_limit is None else min(g.v_limit, self.prox)
        # v10.11: head-on at touching distance: hold, never crawl into someone
        self.touch = touch_hold(robot_xy, robot_yaw, self.people, self.standing, still,
                                self.touch[0] if self.touch is not None else None)
        lk.touch = self.touch
        self.touch_wait = self.touch is not None and state != 'wait'   # the hold is what stops it
        if self.touch is not None:
            state = 'wait'
        return state, self.look, patient

    def nav_ranges(self, ranges, angle_min, angle_inc, range_min, range_max, laser):
        """v9: the scan for Nav2 with standing people shown at their personal-space size;
        v10: and standing vehicles with the ground their forks may cover."""
        r = shape_ranges(ranges, angle_min, angle_inc, range_min, range_max, laser,
                         [(x, y, r) for x, y, r, _ in self.standing if r >= PERSONAL_MIN])
        if USE_VEHICLES and self.veh_zones:
            r = shape_zones(r, angle_min, angle_inc, range_min, range_max, laser, self.veh_zones)
        return r


def describe(state, why, lk, patient):
    """One line for the terminal when GO / WAIT changes."""
    tail = '' if patient else f'  (patience: accepting {SAFE_PATIENT:.2f} m)'
    z = lk.zone
    if state == 'wait':
        if why == 'touch' and getattr(lk, 'touch', None) is not None:
            who, gap, closing = lk.touch
            return (f'WAIT  person {who} right in front: {max(gap, 0.0):.2f} m body to body, head-on '
                    f'(closing {closing:.1f}): holding, not crawling')
        if why == 'emergency':
            return (f'STOP  emergency: going on would pass {lk.d_go_nom:.2f} m from person '
                    f'{lk.who}, stopping keeps {lk.d_stop:.2f} m{tail}')
        if why == 'start' and getattr(lk, 'start', None) is not None:
            who, g_stay, g_go = lk.start
            return (f'WAIT  setting off would close in on person {who}: {g_go:.2f} m body to body '
                    f'(staying: {g_stay:.2f} m){tail}')
        where = (f'at the stop line ({max(z.s_hold, 0.0):.1f} m ahead), ' if z is not None else '')
        if lk.d_go >= lk.need and not lk.comfy:
            short = (f'would be safe ({lk.d_go:.2f} m) but too close for comfort '
                     f'({lk.d_comf / COMFORT_FRONT:.0%} of the distance it keeps)')
        elif lk.d_go >= lk.need:
            short = f'would keep {lk.d_go:.2f} m, but Nav2 is about to restart the goal'
        else:
            short = f'would pass {lk.d_go:.2f} m from them'
        return (f'WAIT  {where}for person {lk.who}: crossing now {short}'
                f'{"" if lk.fresh else " (not seen just now)"}{tail}')
    if why == 'approach':
        if z is not None and lk.edge:
            return (f'GO    edging up to the stop line ({z.s_hold:.1f} m ahead): no comfortable gap '
                    f'for {COMFORT_WAIT:.0f} s{tail}')
        return f'GO    up to the stop line ({z.s_hold:.1f} m ahead){tail}' if z is not None else \
            f'GO    up to the hold point{tail}'
    if why == 'in_way':
        return f'GO    in person {lk.who_stop}\'s way ({lk.d_stop:.2f} m if it stays), clearing it{tail}'
    if why == 'clearing':
        return f'GO    out of a crossing zone (staying would keep {lk.d_stop:.2f} m){tail}'
    if why == 'committed':
        return f'GO    committed: finishing the crossing{tail}'
    if lk.d_go > 8.0:
        return f'GO    no one near the path{tail}'
    if why == 'crossing':
        comfy = '' if lk.d_comf >= COMFORT_FRONT else '  (comfort waived after waiting)'
        return f'GO    crossing: keeps {lk.d_go:.2f} m from everyone{comfy}{tail}'
    return f'GO    clear: going keeps {lk.d_go:.2f} m from everyone{tail}'


def limit_source(core, lim):
    """v10.4: which rule set the speed limit lim (the lowest of them wins)."""
    if lim is None:
        return None
    for val, what in ((getattr(core, 'prox', None), 'a person is close'),
                      (core.cap, 'passing a person standing nearby'),
                      (core.veh_cap, 'a vehicle is near')):
        if val is not None and lim >= val - 1e-6:
            return what
    return 'easing off near people'


def status(core, state, why, lk, robot, cruise, t):
    """v10.4: what the guard is doing, as a dict for /people_guard/state. Plain facts only; the
    'say' phrase is written so an operator (or the robot agent, in the first person) can repeat it.
    person_m / vehicle_m: from the robot's centre to the nearest person (walking, standing, or seen
    walking and now still) and to the nearest point of a vehicle that is moving or moved in the last
    VEH_RECENT s; None = none followed."""
    rx, ry = float(robot[0]), float(robot[1])
    person_m, person_id = None, None
    pts = [(tr.px, tr.py, tr.id) for tr in core.people]
    pts += [(x, y, sid) for x, y, _, sid in core.standing]
    ids = {tr.id for tr in core.people}
    pts += [(tr.x, tr.y, tr.id) for tr in core.tracker.tracks
            if getattr(tr, 'moved', False) and tr.id not in ids and not getattr(tr, 'vehicle', False)
            and t - tr.t <= TIMEOUT]
    for x, y, pid in pts:
        d = math.hypot(x - rx, y - ry)
        if person_m is None or d < person_m:
            person_m, person_id = d, pid
    vehicle_m, vehicle_moving, n_veh = None, False, 0
    for v in core.vehicles:
        if not (v.moving or v.recently_moved(t)):
            continue
        n_veh += 1
        d = float(np.min(np.hypot(v.pts[:, 0] - rx, v.pts[:, 1] - ry)))
        if vehicle_m is None or d < vehicle_m:
            vehicle_m, vehicle_moving = d, bool(v.moving)
    g = core.guard
    lim = g.v_limit
    slow = lim is not None and lim < cruise - 0.02
    src = limit_source(core, lim) if slow else None
    z = lk.zone
    if state == 'wait':
        if why == 'touch':
            say = 'stopped: a person is right in front of me, almost touching; I wait for them to move away'
        elif why == 'emergency':
            say = (f'stopped: going on would pass {lk.d_go_nom:.1f} m from a person, standing still '
                   f'keeps {lk.d_stop:.1f} m')
        elif why == 'start':
            say = 'waiting: setting off now would close in on a person beside me'
        elif why == 'vehicle':
            vl = core.vlook
            if vl.keep_out:
                say = 'waiting: a vehicle is standing in my way and its forks may be out'
            elif vl.d_go >= 9.0:
                say = 'giving way to a vehicle: it has just passed, waiting a moment to be sure'
            elif vl.d_go >= VEH_SAFE:
                say = (f'giving way to a vehicle: waiting a moment to be sure the way stays clear '
                       f'(going on now would pass {vl.d_go:.1f} m from it)')
            else:
                say = f'giving way to a vehicle: going on would pass {vl.d_go:.1f} m from it'
        elif z is not None:
            say = f'waiting at a walkway for a person to cross (going now would pass {lk.d_go:.1f} m from them)'
        else:
            say = f'waiting for a person to pass (going now would pass {lk.d_go:.1f} m from them)'
    else:
        what = {'crossing': 'crossing a walkway', 'committed': 'finishing a walkway crossing',
                'clearing': 'getting out of a walkway', 'in_way': "moving out of a person's way",
                'approach': 'driving up to a walkway, ready to stop'}.get(why)
        if slow:
            say = (f'{what}, ' if what else '') + f'slowed to {lim:.1f} m/s: {src}'
        else:
            say = what or 'clear'
    return dict(t=round(float(t), 2), state=state, why=why, say=say,
                v_limit=None if lim is None else round(float(lim), 2), limit=src,
                person_m=None if person_m is None else round(person_m, 2),
                person_id=None if person_id is None else int(person_id),
                vehicle_m=None if vehicle_m is None else round(vehicle_m, 2),
                vehicle_moving=vehicle_moving, people=len(pts), vehicles=n_veh,
                standing=len(core.standing), version=VERSION)


LOG_FIELDS = ['t', 'state', 'why', 'd_go', 'd_stop', 'can_go_for', 'who', 'robot_x', 'robot_y',
              'robot_yaw', 'robot_v', 'plan_used', 'people', 'hidden_rays', 'p_id', 'p_x', 'p_y',
              'p_vx', 'p_vy', 'p_speed', 'p_kind', 'patient', 'd_go_nom', 'who_stop', 'cruise',
              's_hold', 's_band', 's_out', 'inside', 'fresh', 'p_age', 'room',
              'need', 'v_limit', 'cm_in', 'cm_out', 'd_comf', 'comfy', 'prog_left', 'edge',
              'standing', 'stand_d', 'stand_r', 'cap', 'wall', 'plan_len', 'plan_end',
              'vehicles', 'veh_d', 'veh_v', 'veh_go', 'veh_hold', 'veh_keep',
              'veh_cap', 'prox', 'start_block']


def log_row(t, state, why, lk, robot, robot_v, plan_used, people, hidden, patient, cruise=0.0,
            v_limit=None, cm_in=None, cm_out=None, standing=(), cap=None, wall=None, plan=None,
            core=None):
    """One CSV row (list of strings) describing this scan's decision. cm_in / cm_out:
    the Collision Monitor's input and output forward speed (None: not heard)."""
    pid = lk.who if lk.who is not None else (lk.zone.who if lk.zone is not None else lk.who_stop)
    p = next((tr for tr in people if tr.id == pid), None)
    if p is None and people:
        p = min(people, key=lambda tr: math.hypot(tr.px - robot[0], tr.py - robot[1]))
    kind = '' if p is None else ('lost' if p.lost else ('walking' if p.walking else 'paused'))
    f = (lambda v: f'{v:.3f}')
    z = lk.zone
    return [f'{t:.2f}', state, why, f(lk.d_go), f(lk.d_stop),
            '' if lk.can_go_for is None else f'{lk.can_go_for:.1f}',
            '' if lk.who is None else str(lk.who), f(robot[0]), f(robot[1]), f(robot[2]),
            f(robot_v), str(int(plan_used)), str(len(people)), str(hidden),
            '' if p is None else str(p.id), '' if p is None else f(p.px), '' if p is None else f(p.py),
            '' if p is None else f(p.wvx), '' if p is None else f(p.wvy),
            '' if p is None else f(p.speed), kind, str(int(patient)), f(lk.d_go_nom),
            '' if lk.who_stop is None else str(lk.who_stop), f(cruise),
            '' if z is None else f(z.s_hold), '' if (z is None or z.s_band is None) else f(z.s_band),
            '' if z is None else f(z.s_out), str(int(lk.inside)), str(int(lk.fresh)),
            '' if p is None else f(t - p.t), '' if z is None else f(lk.room),
            f(lk.need), '' if v_limit is None else f(v_limit),
            '' if cm_in is None else f(cm_in), '' if cm_out is None else f(cm_out),
            f(lk.d_comf), str(int(lk.comfy)), f(min(lk.prog_left, 99.0)), str(int(lk.edge))] + \
        standing_cols(robot, standing, cap, wall, plan) + vehicle_cols(core)


def vehicle_cols(core):
    """v10 log columns: vehicles followed, the nearest moving one (distance, speed), the closest
    going on would come to one, whether the robot is giving way, and how far its path ahead stays
    out of a standing vehicle's keep-out (negative: it goes in)."""
    if core is None:
        return ['', '', '', '', '', '', '', '', '']
    vl = core.vlook
    return [str(sum(1 for v in core.vehicles if v.moving or v.recently_moved(core.guard.last_t or 0.0))),
            '' if vl.near >= 9.0 else f'{vl.near:.3f}', '' if vl.near >= 9.0 else f'{vl.near_v:.2f}',
            '' if vl.d_go >= 9.0 else f'{vl.d_go:.3f}', str(int(core.veh_hold)),
            '' if vl.keep_d >= 9.0 else f'{vl.keep_d:.3f}',
            '' if core.veh_cap is None else f'{core.veh_cap:.2f}',
            '' if getattr(core, 'prox', None) is None else f'{core.prox:.2f}',
            str(int(getattr(core.guard, 'start_block', False)))]


def standing_cols(robot, standing, cap, wall, plan):
    """v9 log columns: standing people, the nearest one's distance and personal radius, the speed
    cap for passing them, the wall clock, and how long Nav2's plan is from the robot to its end
    compared with the straight line (a detour round the racks shows as plan_len >> plan_end)."""
    near = min(standing, key=lambda s: math.hypot(s[0] - robot[0], s[1] - robot[1]), default=None)
    plan_len = plan_end = ''
    if plan is not None and len(plan) >= 2:
        p = np.asarray(plan, dtype=float)
        i0 = int(np.argmin(np.hypot(p[:, 0] - robot[0], p[:, 1] - robot[1])))
        seg = p[i0:]
        plan_len = f'{float(np.sum(np.hypot(np.diff(seg[:, 0]), np.diff(seg[:, 1])))) if len(seg) > 1 else 0.0:.2f}'
        plan_end = f'{math.hypot(p[-1, 0] - robot[0], p[-1, 1] - robot[1]):.2f}'
    return [str(len(standing)),
            '' if near is None else f'{math.hypot(near[0] - robot[0], near[1] - robot[1]):.3f}',
            '' if near is None else f'{near[2]:.2f}',
            '' if cap is None else f'{cap:.3f}',
            '' if wall is None else f'{wall:.3f}', plan_len, plan_end]


# ---------------------------------------------------------------- ROS node
def main():
    import csv
    import rclpy
    from rclpy.node import Node
    from rclpy.parameter import Parameter
    from rclpy.qos import (QoSProfile, DurabilityPolicy, ReliabilityPolicy,
                           qos_profile_sensor_data)
    from rclpy.time import Time
    from builtin_interfaces.msg import Time as TimeMsg
    import json
    from geometry_msgs.msg import Point, PoseArray, Twist
    from nav_msgs.msg import OccupancyGrid, Odometry, Path
    from rcl_interfaces.srv import GetParameters
    from sensor_msgs.msg import LaserScan, PointCloud2, PointField
    from std_msgs.msg import String
    from visualization_msgs.msg import Marker, MarkerArray
    from tf2_ros import Buffer, TransformListener
    try:
        from rclpy.executors import ExternalShutdownException
    except ImportError:
        ExternalShutdownException = KeyboardInterrupt
    try:
        from nav2_msgs.msg import SpeedLimit
    except ImportError:
        SpeedLimit = None

    label = sys.argv[1] if len(sys.argv) > 1 and sys.argv[1] else 'guard_last'

    def yaw_of(q):
        return math.atan2(2 * (q.w * q.z + q.x * q.y), 1 - 2 * (q.y ** 2 + q.z ** 2))

    class PeopleGuard(Node):
        def __init__(self):
            super().__init__('people_guard', parameter_overrides=[
                Parameter('use_sim_time', Parameter.Type.BOOL, True)])
            self.buf = Buffer()
            self.tfl = TransformListener(self.buf, self)
            self.core = GuardCore()
            self.plan, self.plan_t = None, None
            self.cruise = None
            self.prev_robot = None
            self.robot_v = 0.0
            self.odom_v, self.odom_t = 0.0, None     # robot speed from the wheels (/odom)
            latched = QoSProfile(depth=1, durability=DurabilityPolicy.TRANSIENT_LOCAL,
                                 reliability=ReliabilityPolicy.RELIABLE)
            self.create_subscription(OccupancyGrid, '/map', self.on_map, latched)
            self.create_subscription(LaserScan, '/scan', self.on_scan, qos_profile_sensor_data)
            self.create_subscription(Path, '/plan', self.on_plan, 1)
            self.create_subscription(Odometry, '/odom', self.on_odom, qos_profile_sensor_data)
            self.create_subscription(PoseArray, '/eye/persons', self.on_persons, 5)
            self.laser = None              # v9: laser pose of the scan being processed
            self.veh_said = False          # v10: last give-way state printed
            self.scan_pub = self.create_publisher(LaserScan, '/scan_nav', QoSProfile(depth=5))
            self.hold_pub = self.create_publisher(PointCloud2, '/people_guard/hold', 5)
            self.marker_pub = self.create_publisher(MarkerArray, '/people_markers', 5)
            self.state_pub = self.create_publisher(String, STATE_TOPIC, 10)   # v10.4
            self.limit_pub = (self.create_publisher(SpeedLimit, SPEED_LIMIT_TOPIC, 10)
                              if SpeedLimit is not None else None)
            self.cm_in = self.cm_out = None          # Collision Monitor in / out (m/s, sim time)
            self.create_subscription(Twist, '/cmd_vel_safe_in', self.on_cm_in, 10)
            self.create_subscription(Twist, '/cmd_vel', self.on_cm_out, 10)
            self.param_cli = self.create_client(GetParameters, '/controller_server/get_parameters')
            self.param_future = None
            self.said = set()
            self.last_state = None
            out_dir = os.path.expanduser('~/runs/guard')
            os.makedirs(out_dir, exist_ok=True)
            self.log_path = os.path.join(out_dir, f'{label}.csv')
            self.log_file = open(self.log_path, 'w', newline='')
            self.log = csv.writer(self.log_file)
            self.log.writerow(LOG_FIELDS)
            self.rows = 0
            print(f'people_guard {VERSION}: waiting for /map, /scan and TF ... '
                  f'(decisions are logged to {self.log_path})', flush=True)
            if self.limit_pub is None:
                print('people_guard: nav2_msgs not found, so no speed limit near people '
                      '(the robot will not ease off or start gently)', flush=True)
            print('people_guard: vehicles ' + (f'ON: gives way at {VEH_SAFE:.1f} m (+{VEH_LEAD:.1f} m at both ends '
                  f'for forks), keeps {VEH_KEEP:.1f} m from ones that just stopped and out of their fork band '
                  f'after {VEH_WORK:.0f} s, {V_VEHICLE:.1f} m/s within {VEH_NEAR:.0f} m' if USE_VEHICLES else
                  'OFF (GUARD_VEHICLES=0)'), flush=True)
            print('people_guard: camera ' + ('ON: standing people the camera confirms (/eye/persons) get '
                  f'{PERSONAL_R:.1f} m of personal space and a {V_PERSON:.1f} m/s pass' if USE_CAMERA else
                  'OFF (GUARD_CAMERA=0): lidar only, like v7.1'), flush=True)

        def once(self, key, text):
            if key not in self.said:
                self.said.add(key)
                print(text, flush=True)

        # -- inputs
        def on_map(self, msg):
            i = msg.info
            self.core.map = StaticMap(i.width, i.height, i.resolution,
                                      i.origin.position.x, i.origin.position.y, msg.data)
            print(f'people_guard: map {i.width} x {i.height} cells loaded.', flush=True)

        def on_cm_in(self, msg):
            self.cm_in = (msg.linear.x, self.get_clock().now().nanoseconds * 1e-9)

        def on_cm_out(self, msg):
            self.cm_out = (msg.linear.x, self.get_clock().now().nanoseconds * 1e-9)

        def cm_now(self, which, now):
            """Last forward speed into / out of the Collision Monitor, if under 0.3 s old."""
            return None if which is None or now - which[1] > 0.3 else which[0]

        def on_odom(self, msg):
            self.odom_v = abs(msg.twist.twist.linear.x)
            self.odom_t = self.get_clock().now().nanoseconds * 1e-9

        def on_persons(self, msg):
            """v9: people eye.py located with camera + lidar (map frame, stamped with the image)."""
            ts = msg.header.stamp.sec + msg.header.stamp.nanosec * 1e-9
            t = self.core.guard.last_t if self.core.guard.last_t is not None else ts
            self.core.camera([(p.position.x, p.position.y, ts) for p in msg.poses], t)
            if msg.poses:
                self.once('cam', f'[{ts:8.1f}] people_guard: first person from the camera at '
                                 f'({msg.poses[0].position.x:.2f}, {msg.poses[0].position.y:.2f})')

        def on_plan(self, msg):
            self.plan = [(p.pose.position.x, p.pose.position.y) for p in msg.poses]
            self.plan_t = self.get_clock().now().nanoseconds * 1e-9      # sim time it arrived

        def get_cruise(self):
            """FollowPath.desired_linear_vel from the controller, asked once."""
            if self.cruise is not None:
                return self.cruise
            if self.param_future is None and self.param_cli.service_is_ready():
                req = GetParameters.Request()
                req.names = ['FollowPath.desired_linear_vel',
                             'progress_checker.required_movement_radius',
                             'progress_checker.movement_time_allowance']
                self.param_future = self.param_cli.call_async(req)
            if self.param_future is not None and self.param_future.done():
                res = self.param_future.result()
                vals = [None, None, None]
                for k, pv in enumerate(res.values if res is not None else []):
                    if k < 3 and pv.type == 3:
                        vals[k] = pv.double_value
                v = vals[0]
                self.cruise = CRUISE_FACTOR * v if v else CRUISE_DEFAULT
                g = self.core.guard
                g.prog_radius = vals[1] if vals[1] else PROG_RADIUS
                g.prog_time = vals[2] if vals[2] else PROG_TIME
                print(f'people_guard: robot cruise speed for decisions {self.cruise:.2f} m/s '
                      f'({"from desired_linear_vel " + format(v, ".2f") if v else "default"}); '
                      f'Nav2 progress checker {g.prog_radius:.2f} m in {g.prog_time:.0f} s'
                      f'{"" if vals[2] else " (default)"}.', flush=True)
                return self.cruise
            return CRUISE_DEFAULT

        def pose_of(self, frame, stamp):
            for when in (stamp, Time()):
                try:
                    tf = self.buf.lookup_transform('map', frame, when)
                    return (tf.transform.translation.x, tf.transform.translation.y,
                            yaw_of(tf.transform.rotation))
                except Exception:
                    continue
            return None

        def on_scan(self, scan):
            self.sent = False
            try:
                self.process(scan)
            except Exception:           # never let a bug here blind Nav2 or leave people hidden
                self.once('error', 'people_guard: ERROR, passing /scan to Nav2 unchanged:\n'
                          + traceback.format_exc())
            if not self.sent:
                self.publish_scan_nav(scan, None)

        def process(self, scan):
            """Track, decide, then publish /scan_nav, hold, markers and the log row."""
            if self.core.map is None:
                self.once('map', 'people_guard: no /map yet (is T3 running?)')
                return
            stamp = Time.from_msg(scan.header.stamp)
            laser = self.pose_of(scan.header.frame_id, stamp)
            robot = self.pose_of('base_link', stamp)
            if laser is None or robot is None:
                self.once('tf', f'people_guard: no TF map -> {scan.header.frame_id} / base_link yet')
                return
            t = scan.header.stamp.sec + scan.header.stamp.nanosec * 1e-9
            plan = self.plan if self.plan_t is not None and t - self.plan_t <= PLAN_AGE else None
            hide = self.core.scan(scan.ranges, scan.angle_min, scan.angle_increment,
                                  scan.range_min, scan.range_max, laser, t, robot, plan)
            now = self.get_clock().now().nanoseconds * 1e-9
            if self.odom_t is not None and now - self.odom_t < 0.5:
                self.robot_v = min(self.odom_v, 1.2 * self.get_cruise())   # wheels: no TF jitter
            elif self.prev_robot is not None and t > self.prev_robot[0]:
                v = math.hypot(robot[0] - self.prev_robot[1], robot[1] - self.prev_robot[2]) \
                    / (t - self.prev_robot[0])
                self.robot_v = min(0.5 * self.robot_v + 0.5 * v, 1.2 * self.get_cruise())
            self.prev_robot = (t, robot[0], robot[1])
            state, lk, patient = self.core.decide(robot[:2], robot[2], self.robot_v,
                                                  self.get_cruise(), t, plan)
            # people are hidden from Nav2 only once the guard has decided about them
            self.laser = laser
            self.publish_scan_nav(scan, hide)
            v_limit = self.core.guard.v_limit
            self.publish_limit(scan.header.stamp, v_limit)
            self.publish_hold(scan.header.stamp, state == 'wait')      # also the gate's heartbeat
            why = ('touch' if self.core.touch_wait else
                   'vehicle' if self.core.veh_hold else self.core.guard.why)
            self.publish_markers(robot, state, why, lk)
            try:                                                        # v10.4: never in the way
                self.state_pub.publish(String(data=json.dumps(
                    status(self.core, state, why, lk, robot, self.get_cruise(), t))))
            except Exception:
                self.once('state_err', 'people_guard: could not publish /people_guard/state:\n'
                          + traceback.format_exc())
            self.log.writerow(log_row(t, state, why, lk, robot, self.robot_v, plan is not None,
                                      self.core.people, len(hide), patient, self.get_cruise(),
                                      v_limit, self.cm_now(self.cm_in, now),
                                      self.cm_now(self.cm_out, now), self.core.standing,
                                      self.core.cap, time.time(), plan, self.core))
            if self.core.veh_hold != self.veh_said:
                self.veh_said = self.core.veh_hold
                vl = self.core.vlook
                print(f'[{t:8.1f}] ' + ((f'WAIT  vehicle {vl.keep_who} standing (forks may be out): the path '
                                         f'goes {-vl.keep_d:.2f} m into the ground they may cover' if vl.keep_out else
                                         f'WAIT  giving way to vehicle {vl.who}: going on would pass '
                                         f'{vl.d_go:.2f} m from it') if self.core.veh_hold else
                                        'GO    vehicle gone: the way has been clear for '
                                        f'{VEH_CLEAR:.0f} s'), flush=True)
            if self.core.standing:
                s = min(self.core.standing, key=lambda p: math.hypot(p[0] - robot[0], p[1] - robot[1]))
                self.once(f'stand{s[3]}', f'[{t:8.1f}] people_guard: person standing at ({s[0]:.2f}, '
                          f'{s[1]:.2f}) (camera): personal space {s[2]:.2f} m, passing at '
                          f'{V_PERSON:.1f} m/s within {PASS_R:.1f} m')
            self.rows += 1
            if self.rows % 20 == 0:
                self.log_file.flush()
            if state != self.last_state:
                if why != 'vehicle' and (state == 'wait' or self.last_state is not None):     # v10.11: 'touch' too
                    print(f'[{t:8.1f}] {describe(state, why, lk, patient)}', flush=True)
                self.last_state = state

        # -- outputs
        def publish_scan_nav(self, scan, hide):
            out = LaserScan()
            out.header = scan.header
            out.angle_min, out.angle_max = scan.angle_min, scan.angle_max
            out.angle_increment, out.time_increment = scan.angle_increment, scan.time_increment
            out.scan_time = scan.scan_time
            out.range_min, out.range_max = scan.range_min, scan.range_max
            r = np.array(scan.ranges, dtype=np.float32)
            if hide is not None and len(hide):
                r[np.asarray(hide, dtype=int)] = np.nan
            if hide is not None and (self.core.standing or self.core.veh_zones) and self.laser is not None:
                r = self.core.nav_ranges(r, scan.angle_min, scan.angle_increment, scan.range_min,
                                         scan.range_max, self.laser)
            out.ranges = array.array('f', r.tobytes())
            out.intensities = scan.intensities
            self.scan_pub.publish(out)
            self.sent = True

        def publish_limit(self, stamp, v_limit):
            """Speed limit for Nav2's controller (absolute m/s; 0 means none)."""
            if self.limit_pub is None:
                return
            msg = SpeedLimit()
            msg.header.stamp = stamp
            msg.percentage = False
            msg.speed_limit = 0.0 if v_limit is None else float(v_limit)
            self.limit_pub.publish(msg)

        def publish_hold(self, stamp, hold):
            pts = [(HOLD_X, y, Z) for y in (-0.1, -0.05, 0.0, 0.05, 0.1)] if hold else []
            msg = PointCloud2()
            msg.header.frame_id, msg.header.stamp = 'base_link', stamp
            msg.height, msg.width = 1, len(pts)
            msg.fields = [PointField(name=n, offset=4 * k, datatype=PointField.FLOAT32, count=1)
                          for k, n in enumerate('xyz')]
            msg.is_bigendian, msg.point_step = False, 12
            msg.row_step, msg.is_dense = 12 * len(pts), True
            msg.data = array.array('B', np.array(pts, dtype=np.float32).reshape(-1, 3).tobytes())
            self.hold_pub.publish(msg)

        def publish_markers(self, robot, state, why, lk):
            """Everything in the map frame with a zero stamp: RViz draws it with
            the latest TF instead of waiting for TF at the scan time (which made
            the display blink Error / OK)."""
            ma = MarkerArray()
            clear = Marker()
            clear.action = Marker.DELETEALL
            ma.markers.append(clear)

            def marker(ns, mid, mtype):
                m = Marker()
                m.header.frame_id, m.header.stamp = 'map', TimeMsg()
                m.ns, m.id, m.type, m.action = ns, mid, mtype, Marker.ADD
                m.pose.orientation.w = 1.0
                return m

            def line(ns, mid, xy, rgba, width=0.05, z=0.05):
                m = marker(ns, mid, Marker.LINE_STRIP)
                m.scale.x = width
                m.color.r, m.color.g, m.color.b, m.color.a = rgba
                m.points = [Point(x=float(x), y=float(y), z=z) for x, y in xy]
                return m
            t_now = self.core.guard.last_t or 0.0
            waited_for = lk.who if state == 'wait' else None
            for tr in self.core.people:
                stale = tr.lost or (t_now - tr.t) > STALE
                body = marker('person', tr.id, Marker.CYLINDER)
                body.pose.position.x, body.pose.position.y, body.pose.position.z = tr.px, tr.py, 0.85
                body.scale.x = body.scale.y = 2 * BODY_R
                body.scale.z = 1.7
                body.color.r, body.color.g, body.color.b, body.color.a = (
                    (0.9, 0.1, 0.1, 0.7) if tr.id == waited_for else
                    (0.6, 0.6, 0.6, 0.4) if stale else
                    (0.2, 0.5, 1.0, 0.5) if tr.id in self.core.along else (1.0, 0.55, 0.1, 0.5))
                ma.markers.append(body)
                for k, f in enumerate(lk.futs.get(tr.id, [])):
                    if len(f) > 1 and np.ptp(f[:26, 0]) + np.ptp(f[:26, 1]) > 0.05:
                        ma.markers.append(line('predicted_path', tr.id * 10 + k, f[:26],
                                               (1.0, 0.25, 0.25, 0.9), 0.05))
                if tr.id in self.core.crossers and tr.line is not None and len(tr.raw) > 2:
                    cx, cy, ux, uy = tr.line
                    s = [(x - cx) * ux + (y - cy) * uy for _, x, y in tr.raw]
                    a, b = min(s), max(s)
                    ma.markers.append(line('walkway', tr.id, [(cx + ux * a, cy + uy * a),
                                                             (cx + ux * b, cy + uy * b)],
                                           (1.0, 0.55, 0.1, 0.35), 0.5, 0.01))
                text = marker('label', tr.id, Marker.TEXT_VIEW_FACING)
                text.pose.position.x, text.pose.position.y, text.pose.position.z = tr.px, tr.py, 2.0
                text.scale.z = 0.28
                text.color.r = text.color.g = text.color.b = text.color.a = 1.0
                text.text = (f'person {tr.id}  {tr.speed:.1f} m/s'
                             + ('  (along the path)' if tr.id in self.core.along else '')
                             + ('  (not seen just now)' if stale else '')
                             + ('  (robot waits)' if tr.id == waited_for else ''))
                ma.markers.append(text)
            z = lk.zone
            if z is not None and lk.path_xy is not None and len(lk.path_xy) > 1:
                P = lk.path_xy
                i0 = int(round(z.s_hold / 0.05))
                i1 = min(int(round(z.s_out / 0.05)), len(P) - 1)
                if i1 > i0:
                    ma.markers.append(line('crossing_zone', 0, P[i0:i1 + 1], (1.0, 0.75, 0.0, 0.25),
                                           0.7, 0.02))
                k = min(i0, len(P) - 2)
                dx, dy = P[k + 1, 0] - P[k, 0], P[k + 1, 1] - P[k, 1]
                L = math.hypot(dx, dy) or 1.0
                nx, ny = -dy / L * 0.45, dx / L * 0.45
                ma.markers.append(line('stop_line', 0, [(P[k, 0] - nx, P[k, 1] - ny),
                                                        (P[k, 0] + nx, P[k, 1] + ny)],
                                       (1.0, 0.15, 0.15, 1.0) if state == 'wait' else (1.0, 1.0, 1.0, 0.9),
                                       0.08, 0.03))
            if lk.go_path is not None:
                ma.markers.append(line('robot_path', 0, lk.go_path[:31], (0.2, 0.9, 0.3, 0.8), 0.04))
            for v in self.core.vehicles:
                if not (v.moving or v.recently_moved(t_now)):
                    continue
                (x0, y0), (x1, y1) = v.pts.min(axis=0), v.pts.max(axis=0)
                box = marker('vehicle', v.id, Marker.CUBE)
                box.pose.position.x, box.pose.position.y, box.pose.position.z = (x0 + x1) / 2, (y0 + y1) / 2, 0.6
                box.scale.x, box.scale.y, box.scale.z = max(x1 - x0, 0.2), max(y1 - y0, 0.2), 1.2
                hold = self.core.veh_hold and v.id in (self.core.vlook.who, self.core.vlook.keep_who)
                if v.standing(t_now) and v.banded(t_now):      # the band its forks may lie in
                    u, n, a0, a1, b0, b1 = v.band(robot[:2])
                    cs = [a0 * u + b0 * n, a1 * u + b0 * n, a1 * u + b1 * n, a0 * u + b1 * n, a0 * u + b0 * n]
                    ma.markers.append(line('vehicle_keep', v.id, np.array(cs), (1.0, 0.45, 0.0, 0.9), 0.05, 0.05))
                elif v.standing(t_now):                          # just stopped: forks anywhere
                    cx, cy = float(v.pts[:, 0].mean()), float(v.pts[:, 1].mean())
                    rr = float(np.max(np.hypot(v.pts[:, 0] - cx, v.pts[:, 1] - cy))) + VEH_KEEP
                    ang = np.linspace(0, 2 * math.pi, 25)
                    ma.markers.append(line('vehicle_keep', v.id, np.column_stack((cx + rr * np.cos(ang),
                                                                                  cy + rr * np.sin(ang))),
                                           (1.0, 0.45, 0.0, 0.9), 0.05, 0.05))
                box.color.r, box.color.g, box.color.b, box.color.a = (1.0, 0.2, 0.1, 0.55) if hold else (1.0, 0.6, 0.0, 0.45)
                ma.markers.append(box)
                if v.speed > VEH_STOPPED:
                    u = np.array([v.vx, v.vy]) / v.speed
                    ma.markers.append(line('vehicle_path', v.id, [(v.cx, v.cy),
                                           (v.cx + v.vx * VEH_HORIZON + u[0] * VEH_LEAD,
                                            v.cy + v.vy * VEH_HORIZON + u[1] * VEH_LEAD)],
                                           (1.0, 0.45, 0.0, 0.9), 0.06, 0.1))
                vt = marker('vehicle_label', v.id, Marker.TEXT_VIEW_FACING)
                vt.pose.position.x, vt.pose.position.y, vt.pose.position.z = v.cx, v.cy, 1.8
                vt.scale.z = 0.28
                vt.color.r = vt.color.g = vt.color.b = vt.color.a = 1.0
                vt.text = (f'VEHICLE {v.speed:.1f} m/s' if v.moving else 'VEHICLE standing (forks?)') + \
                    ('  (giving way)' if hold else '')
                ma.markers.append(vt)
            for k, (sx, sy, sr, sid) in enumerate(self.core.standing):
                ring = marker('personal_space', k, Marker.CYLINDER)
                ring.pose.position.x, ring.pose.position.y, ring.pose.position.z = sx, sy, 0.02
                ring.scale.x = ring.scale.y = 2 * max(sr, PERSONAL_MIN)
                ring.scale.z = 0.03
                ring.color.r, ring.color.g, ring.color.b, ring.color.a = 0.15, 0.85, 0.35, 0.35
                ma.markers.append(ring)
                body = marker('standing', k, Marker.CYLINDER)
                body.pose.position.x, body.pose.position.y, body.pose.position.z = sx, sy, 0.85
                body.scale.x = body.scale.y = 2 * BODY_R
                body.scale.z = 1.7
                body.color.r, body.color.g, body.color.b, body.color.a = 0.15, 0.85, 0.35, 0.6
                ma.markers.append(body)
                text = marker('standing_label', k, Marker.TEXT_VIEW_FACING)
                text.pose.position.x, text.pose.position.y, text.pose.position.z = sx, sy, 2.2
                text.scale.z = 0.26
                text.color.r = text.color.g = text.color.b = text.color.a = 1.0
                text.text = f'PERSON standing (camera)  space {sr:.1f} m'
                ma.markers.append(text)
            status = marker('robot_decision', 0, Marker.TEXT_VIEW_FACING)
            status.pose.position.x, status.pose.position.y = robot[0], robot[1]
            status.pose.position.z = 1.3
            status.scale.z = 0.35
            if state == 'wait':
                status.color.r, status.color.g, status.color.b = 1.0, 0.2, 0.2
                status.text = ('WAIT: someone right in front' if why == 'touch' else
                               'EMERGENCY STOP' if why == 'emergency' else
                               'WAIT: setting off would close in on someone' if why == 'start' else
                               ('WAIT: vehicle standing ahead' if self.core.vlook.keep_out else
                                'WAIT: giving way to a vehicle') if self.core.veh_hold else
                               f'WAIT at stop line (person {lk.who})' if z is not None else
                               f'WAIT for person {lk.who}')
            else:
                status.color.r, status.color.g, status.color.b = 0.2, 1.0, 0.3
                status.text = {'approach': 'GO (to stop line)', 'in_way': 'GO (clearing)',
                               'clearing': 'GO (leaving walkway)', 'crossing': 'CROSSING',
                               'committed': 'CROSSING (committed)'}.get(why, 'GO')
                v_lim = self.core.guard.v_limit
                if v_lim is not None and v_lim < self.get_cruise() - 0.02:
                    status.text += f'  {v_lim:.1f} m/s'
                if self.core.cap is not None and v_lim is not None and v_lim <= self.core.cap + 1e-6:
                    status.text += '  (passing a person)'
            status.color.a = 1.0
            ma.markers.append(status)
            self.marker_pub.publish(ma)

        def close(self):
            try:
                self.publish_limit(TimeMsg(), None)      # leave Nav2 without a limit
            except Exception:
                pass
            try:
                self.log_file.close()
            except Exception:
                pass

    rclpy.init()
    node = PeopleGuard()
    try:
        rclpy.spin(node)
    except (KeyboardInterrupt, ExternalShutdownException):
        pass
    finally:
        node.close()
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()


if __name__ == '__main__':
    sys.exit(main())
