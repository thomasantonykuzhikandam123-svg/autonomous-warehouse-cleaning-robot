#!/usr/bin/env python3
"""
Full-coverage cleaning mission, executed via Nav2 (NavigateToPose).

COVERAGE UPDATE (this version):
  * aisle bounds are the MEASURED rack walls (map [-1.42,0.31] / [2.34,4.14] /
    [6.35,8.29]) -> idealized 3 m aisles caused lanes to land inside the racks.
  * WALL_MARGIN 0.35 + column_spacing 0.40 + evenly-distributed lanes push the
    sweep out toward both rack faces, so the strips next to the racks get
    cleaned instead of leaving a fat uncleaned margin on each side.
  -> ~97% of each aisle width is now painted, lanes still stay 0.35 m off the
     walls (0.05 m clear of the 0.30 m costmap inscribed radius -> no wedging).

Earlier fixes kept from previous versions:
1. WALL_MARGIN  -> lanes pulled away from rack SIDES so the body never grazes.
2. END_MARGIN   -> turns happen in OPEN space past the rack ends.
3. Corridor transit between aisles instead of cutting across racks.
4. Nearest-first ordering so the first goal is short and stable.
5. Strict boustrophedon (snake) per aisle, each lane visited once.
6. Robust execution: rejected goals skipped, hung goals abandoned.

Marker note: the cleaned-trail marker is drawn at the robot's TRUE map pose
via TF (map -> base_link), so it sits on the robot and never drifts.

TRAFFIC-AWARE (v10.4, 11 Oct, after shift_v103a):
  shift_v103a ran this node behind the people guard with traffic. All 9 STALL
  cancels and all 3 TIMEOUTs happened while the guard held the robot for a
  person or a vehicle, or Nav2 would not drive past one standing in the lane
  (traffic in 100 % of every window): 4 reverses, 2 lanes skipped, aisle 2
  left at 83 %. Two causes:
  1. The node ran on the wall clock (no use_sim_time) while Gazebo ran at
     0.48 x real time, so STALL_SECONDS 15 was 7 s of robot time, shorter than
     an ordinary wait for a forklift. stack.sh v10.4 starts it with
     use_sim_time:=true.
  2. It could not tell traffic from a wedged robot. It now reads the guard's
     /people_guard/state (people_guard.py v10.4). While the guard holds the
     robot, or a person is within TRAFFIC_NEAR_PERSON or a vehicle within
     TRAFFIC_NEAR_VEHICLE (and TRAFFIC_LINGER s after), the stall clock and the
     goal deadline stand still; a goal Nav2 gives up on then is sent again
     after TRAFFIC_RETRY_GAP s and is not a failure; the robot never reverses
     with someone within RESCUE_CLEAR; a waypoint blocked for TRAFFIC_PATIENCE s
     of standing without getting TRAFFIC_PROGRESS m further, one that takes
     TRAFFIC_HARD_FACTOR x its normal time in all, or one that failed after the
     rescue, is PUT OFF (deferred), not
     skipped: before going home the robot comes back for the floor it left
     (only the stretches the painted grid says are still dirty). A revisit that
     fails again is skipped for good. Without the guard running, nothing
     changes. /mission_state says what traffic is doing (for the robot agent).

LANES (v10.5, 11 Oct, after shift_v104a):
  shift_v104a reached all 23 waypoints and called both aisles cleaned, yet the
  painted floor between the rack ends was 75 % of aisle 1 and 49 % of aisle 2
  (driven exactly, the lanes paint 93 %). NavigateToPose only cares about
  reaching the goal: around a picker standing on a lane, Nav2 planned a path
  that cut diagonally back to the lane END, leaving metres of lane unswept
  (lane x -1.52: 37 % swept; 0.03: 69 %; 2.48: 56 %), and both east lanes of
  aisle 2 (x 4.03, 4.54) were "reached" by driving round through AISLE 3
  (4 % swept). So:
  1. A lane (the second waypoint of every sweep pair) is driven with the
     controller's FollowPath on the straight lane itself, not NavigateToPose:
     the robot cannot wander off it. Blocked, it waits (traffic rules above);
     resent, it carries on from where it is on the lane. Turns, transits and
     the way onto a lane stay NavigateToPose. The lane is put on /plan while
     it is driven, for the guard and RViz.
  2. Someone standing in the lane (BYPASS_AFTER_GO s with the guard saying GO,
     or BYPASS_AFTER s in any traffic), or a stall: the robot goes round them
     (NavigateToPose to a point on the lane past them; Nav2's plan is checked
     and a way through another aisle is refused: the rest of the lane is put
     off and revisited) and carries on along the lane. Never round a vehicle.
     What it went round stays dirty for 3.
  3. Before going home, a PATCH pass: every lane of every aisle in the job is
     checked against the painted grid and its still-dirty stretches are swept
     (lane_plan, the same maths as "clean only what is left").
  4. An aisle counts as cleaned when at least 90 % of its lanes' centre line is
     painted, not when 90 % of its waypoints were reached.

SAFER LANES, FEWER WAITS (v10.6, 11 Oct, after shift_v105a: both aisles 100 % swept, but the
  job took 613 s instead of 427 and the robot touched a standing person):
  1. LANE CLEARANCE. Driving a lane, nothing replans round someone standing in it: the guard
     only caps the speed to a 0.15 m/s crawl near a standing person (her personal space had
     come out negative, see people_guard v10.6), so the robot crept into the stepper at rack 2's
     end three times (t 280-285). Now the lane ahead is read from /scan_nav (what Nav2 sees)
     and the robot stops LANE_STOP short of anything in it, then waits / goes round as before.
     It does not need the camera or the guard to know it is a person.
  2. A VEHICLE AT A LANE'S END OR START. The forklift works along x 2.6 from the yard into
     aisle 2's mouth and waits for a robot in its way, while the guard keeps the robot out of
     its fork ground: mutual waits of 14-42 s. Held VEH_SKIP_AFTER s for a vehicle within
     VEH_SKIP_LEN of a lane's end, the lane is done there; on the way to a lane's start, the
     lane is started VEH_SKIP_LEN in. The bit left is swept by the patch pass.
  3. FAIL-SAFE. If the guard was running and goes quiet for GUARD_LOST s (guard or lidar
     down), the robot holds still until it is back (it used to carry on as if no guard).
  4. A frozen simulation (shift_v105a: 22 min of real time with no sim time) is logged as
     SIM PAUSED, so a stall of the computer is not mistaken for the robot waiting.
  5. Waiting for Nav2 at start-up says so; Ctrl+C / stack.sh stop no longer print a traceback.
  6. A SECOND PATCH ROUND. An aisle still under CLEAN_PCT after the patch pass (someone stood on
     the same stretch both times: a picker working at a rack face) gets its dirty stretches tried
     once more before home (PATCH_ROUNDS; stress test: 2 of 40 runs had ended at 87-89 %).
  7. The forklift, continued: held at a lane's end, the next lane (starting beside it, in the same
     vehicle's ground) is started VEH_SKIP_LEN in at once, not after another 10 s held there
     (shift_v105a t 335-393: 42 s, then 14 s). In the patch pass a vehicle holding the robot
     VEH_SKIP_AFTER puts that stretch after the others (once), instead of up to 60 s of waiting.
  8. NAV2 DOWN. Goals rejected (its lifecycle manager deactivates Nav2 when a node misses its
     heartbeat, as a slow VM can) or no answer with no server: v10.5 skipped the waypoint, and
     then the next, through the whole job in seconds. Now the robot holds at the waypoint, says
     NAV2:, retries every NAV2_RETRY s and carries on when Nav2 takes goals again.

UNSTUCK AND IN ORDER (v10.7, 12 Oct, after shift_v106a: 724 s and only 1.4 aisles, 279 s of it
  wedged at a rack corner):
  1. NOTHING MOVES BEFORE A COMMAND. With auto_start false the node waits for /mission_command
     (_awaiting). v10.6's safety hold, released 0.4 s after start-up, called send_next() and
     drove the built-in 65-waypoint mission before any command (t 148-173).
  2. ZONES IN ORDER. build_mission gave the command-time pose to EVERY aisle: the robot stood in
     the front corridor below aisle 2 (x inside aisle 2's span), so aisle 2 was planned from the
     bottom with no transit although aisle 1 ends at the top: 138 s driving down aisle 2. Only
     the first zone of a job starts from where the robot is; the next starts where the last ends.
  3. ALIGN. RPP turns on the spot only beyond rotate_to_heading_min_angle (1.80 rad); below it,
     it drives forward on an arc. At the start of lane 2.48, facing west, that arc took the nose
     into rack 2's corner and RPP saw a collision ahead for good. Before a lane, next to a rack or
     wall (static map within ALIGN_NEAR) and facing more than ALIGN_MAX off it, the robot now
     turns on the spot first (on /cmd_vel, which stack.sh remaps to /cmd_vel_raw: through the
     gate and the Collision Monitor, like the rescue).
  4. UNSTICK. A goal active, the robot standing WEDGE_S with the guard saying GO, nobody within
     WEDGE_PERSON (vehicle WEDGE_VEHICLE), and a rack or wall in front (static map: up to
     WEDGE_FRONT ahead within WEDGE_HALF of the heading line, as far as RPP projects its arc; or
     WEDGE_SIDE anywhere): Nav2 cannot drive on from there, whatever the traffic. v10.6 counted
     the people passing as traffic and waited 60 s, four times. Now: cancel, back off BACK_DIST
     (if it is within BACK_IF and the scan shows the way behind clear), turn to face the lane (or
     Nav2's path), try again; UNSTICK_MAX per waypoint, then the usual rules. A FollowPath that
     RPP aborts there (after failure_tolerance) is handled the same way.

FRONT AND BACK THROUGH AN AISLE (v10.10, 14 Oct, after the demo rehearsal: 'clean the back area'
  from the front area's east end sent the robot straight up the 1.9 m corridor along the east wall,
  where the static map has no wall (so no inflation for the planner); it wedged 0.45 m from the
  wall, inside the keep-out, and every later goal stalled there):
  between the front and the back, the robot now goes through the middle of the nearest aisle
  (_via_aisle): building a mission (the back from below the racks, the front from above them) and
  sending the dock goal from above the racks (two 'via' goals inserted before HOME).
"""

import json
import math
import os
import time
from collections import deque
import rclpy
from rclpy.node import Node
from rclpy.action import ActionClient
from nav2_msgs.action import NavigateToPose
try:
    from nav2_msgs.action import FollowPath     # v10.5: lanes driven exactly
except ImportError:                              # no FollowPath: lanes as before
    FollowPath = None
from nav2_msgs.srv import ClearEntireCostmap
from rcl_interfaces.msg import SetParametersResult
from nav_msgs.msg import OccupancyGrid, Path
from rclpy.qos import (QoSProfile, QoSDurabilityPolicy,
                       QoSReliabilityPolicy, QoSHistoryPolicy)
from std_msgs.msg import String
from sensor_msgs.msg import LaserScan
from visualization_msgs.msg import Marker, MarkerArray
from geometry_msgs.msg import Point, Twist, PoseStamped
from tf2_ros import Buffer, TransformListener

try:
    from warehouse_planner.coverage_lanes import lane_plan, segment_runs, zone_progress
except ImportError:                      # running straight from the source folder
    from coverage_lanes import lane_plan, segment_runs, zone_progress


class CoverageNav2(Node):
    def __init__(self):
        super().__init__('coverage_nav2')

        # ----- world / map config (must match your map + spawn) -----
        self.resolution = 0.1            # grid resolution for the free-map (m)
        self.spawn_x = 4.0               # robot spawn in world X
        self.spawn_y = 4.0               # robot spawn in world Y
        self.world_w = 15.0              # world width  (X)
        self.world_h = 20.0              # world height (Y)
        self.width_cells = int(self.world_w / self.resolution)
        self.height_cells = int(self.world_h / self.resolution)

        # ----- coverage tuning knobs -----
        self.WALL_MARGIN = 0.42          # lateral clearance from rack sides (m).
                                         #   0.35 = robot_radius (0.30) + 0.05 m, so
                                         #   the lanes sit as close to BOTH rack faces
                                         #   as the robot can safely go -> it enters
                                         #   near the left wall and the sweep spans
                                         #   ~96% of the aisle width (max a 0.30 m
                                         #   robot can reach). Pairs with costmap
                                         #   inflation_radius 0.30 so these near-wall
                                         #   lanes carry ZERO cost -> no RPP weaving.
                                         #   (Only safe because GT localization gives
                                         #   an exact pose; for SLAM raise to 0.42.)
        self.END_MARGIN = 0.40           # turn point just past each rack end. Small
                                         #   because RPP pivots IN PLACE (no wide arc),
                                         #   so it won't clip the corner -- and this
                                         #   keeps the turn inside mapped space and lets
                                         #   the robot reach the true aisle ends.
        self.column_spacing = 0.55       # gap between sweep lanes (m). Robot body
                                         #   ~0.4 m wide, so 0.40 = touching strips;
                                         #   the 0.32 m footprint paint overlaps ->
                                         #   continuous clean band, no stripes.
        self.CORRIDOR_Y = 4.0            # low transit strip, below racks (mapped: the
                                         #   robot spawns/drives here)
        self.BACKSTRIP_Y = 18.8          # high transit strip, just above rack tops
        self.home = (0.0, 0.0)           # return-home point in MAP frame

        # ----- rack geometry (world coords): (center_x, half_width,
        #        y_bottom, y_top) for each vertical rack -----
        self.racks = [
            (1.5,  0.5,  5.5, 17.5),
            (5.5,  0.5,  5.5, 17.5),
            (9.5,  0.5,  5.5, 17.5),
            (13.5, 0.5,  5.5, 17.5),
        ]
        # rack vertical extent (shared) used for turn offsets
        self.rack_y_bot = 5.5
        self.rack_y_top = 17.5

        # aisles = free vertical corridors BETWEEN consecutive racks, given as the
        # MEASURED rack-wall x positions (world coords). These are the locked
        # measured bounds (map [-1.42,0.31] / [2.34,4.14] / [6.35,8.29]) + spawn 4.
        self.aisles = [
            # MEASURED from slam_map.pgm, not assumed. The SLAM map contains
            # rack FACES at world x = 2.0, 5.0, 6.0, 9.0, 10.0, 13.0, which
            # makes every rack 1.0 m wide and every aisle 3.0 m -- exactly what
            # self.obstacles below has always said.
            #
            # The previous values (2.58,4.31) (6.34,8.14) (10.35,12.29) were
            # 1.7-1.9 m wide, so the robot swept the middle of each aisle and
            # left ~0.65 m of floor untouched against BOTH rack faces. That is
            # the grey margin either side of every rack in RViz, and it is why
            # coverage sat at 65%.
            # These are the INNER edges of the rack faces (where free floor
            # actually begins), straight out of measure_map.py. Do not round
            # them to 2.0/5.0: the faces are ~0.10 m thick in the map, so
            # rounding to the face CENTRE moves every lane 0.06 m closer to the
            # rack and leaves only 0.06 m between the lane and lethal cost --
            # the same margin that wedged the robot at the pallet and the wall.
            (2.06, 4.96),    # aisle 1  -> map [-1.94, 0.96]  width 2.90 m
            (6.06, 8.96),    # aisle 2  -> map [ 2.06, 4.96]  width 2.90 m
            (10.06, 13.01),  # aisle 3  -> map [ 6.06, 9.01]  width 2.95 m
        ]

        # ----- ALL obstacles (world coords): (center_x, center_y, size_x, size_y).
        #        Used to build a free-space map so the open-floor (back/front)
        #        sweeps automatically avoid them and cover everything else. -----
        self.obstacles = [
            # outer walls
            (7.5, 20.0, 15.0, 0.3), (7.5, 0.0, 15.0, 0.3),
            (0.0, 10.0, 0.3, 20.0), (15.0, 10.0, 0.3, 20.0),
            # storage racks
            (1.5, 11.5, 1.0, 12.0), (5.5, 11.5, 1.0, 12.0),
            (9.5, 11.5, 1.0, 12.0), (13.5, 11.5, 1.0, 12.0),
            # front-area obstacles
            (2.5, 1.5, 3.0, 2.0),    # packing station
            (5.0, 1.5, 1.0, 2.0),    # pallet
            (12.0, 1.5, 4.0, 2.0),   # office
            # side pallets
            (0.6, 9.0, 0.8, 1.2), (14.4, 9.0, 0.8, 1.2),
        ]
        self.inflate_cells = 5           # ~0.5m keep-out around obstacles for the
                                         #   open-floor sweeps (robot radius + buffer)
        self.row_spacing = 0.55          # spacing between horizontal passes in open zones
        self.cov_cell = 0.25             # coverage grid cell size (m) -- needed by
                                         #   count_floor_cells(), so define it here
        self.foot_radius = 0.32          # robot footprint radius painted each tick
        self.free = self.build_free_map()
        self.total_floor = self.count_floor_cells()   # for coverage %

        # The painted grid has to exist before the first mission is built:
        # since patch 3, build_mission reads it to work out what is still
        # dirty. It used to be created further down, after this call.
        self.visited = set()             # (cx, cy) map cells already swept
        # ----- build the mission -----
        self.goals, self.goal_zones = self.build_mission()
        self.goals.append((self.home[0], self.home[1], 'HOME'))
        self.goal_zones.append('home')
        self.gi = 0
        self.sweeps = dict(self._last_sweeps)   # v10.5: lane end goal -> lane start

        # ----- coverage drawn as a GRID of swept cells (CUBE_LIST). Small
        #       localisation steps just fill neighbouring cells, so the result
        #       reads as one solid clean band instead of stair-step cubes. -----
        self.tf_buffer = Buffer()
        self.tf_listener = TransformListener(self.tf_buffer, self)
        # self.visited is created above, before the first build_mission()
        self._paint_last = None          # last painted pose -> paint the gap between
        self.mk = self.create_publisher(MarkerArray, '/coverage_markers', 10)
        self.st = self.create_publisher(String, '/mission_status', 10)
        self.create_timer(0.2, self.tick_marker)

        # ----- SELF-RESCUE infrastructure -----
        # If a goal fails twice, the robot is usually wedged in an obstacle's
        # lethal zone (pose-in-lethal makes EVERY plan fail -> cascade). The
        # rescue: reverse off the obstacle, clear both costmaps, retry once
        # more. This breaks the wedge-cascade automatically instead of
        # machine-gunning through all remaining goals.
        self.cmd_pub = self.create_publisher(Twist, '/cmd_vel', 10)
        self.clear_global = self.create_client(
            ClearEntireCostmap, '/global_costmap/clear_entirely_global_costmap')
        self.clear_local = self.create_client(
            ClearEntireCostmap, '/local_costmap/clear_entirely_local_costmap')
        self._consec_fails = 0           # consecutive SKIPPED waypoints

        # ----- REAL MAP as the coverage denominator -----
        # The hardcoded self.obstacles rack widths disagree with the measured
        # aisle bounds, so a model-based denominator cannot be trusted. The
        # occupancy grid from map_server is what the robot actually navigates,
        # so measure coverage against that instead. Falls back to the model if
        # /map never arrives.
        self._map = None
        self._map_floor = None
        self._map_coverable = None
        self.ROBOT_RADIUS = 0.30     # must match nav2_params robot_radius
        self.create_subscription(
            OccupancyGrid, '/map', self.on_map,
            QoSProfile(depth=1, history=QoSHistoryPolicy.KEEP_LAST,
                       reliability=QoSReliabilityPolicy.RELIABLE,
                       durability=QoSDurabilityPolicy.TRANSIENT_LOCAL))

        # ----- STALL DETECTOR -----
        # A wedged robot still holds an active goal, so the only thing that
        # ended it before was the full goal timeout (~30 s of standing still).
        # Watching actual displacement catches it in STALL_SECONDS instead.
        self.STALL_SECONDS = 15.0    # a full Nav2 recovery (backup + spin +
        #   wait) runs ~12 s; 10 s fired in the middle of one
        self.STALL_DIST = 0.12
        self.STALL_TURN = 0.25       # rad. Nav2's spin recovery turns 1.57 on
        #   the spot: translation alone reads that as a wedged robot
        self._stall_ref = None

        # ----- TRAFFIC (v10.4): the people guard's /people_guard/state -----
        # Waiting for people and vehicles is not being stuck. See the header.
        self.TRAFFIC_NEAR_PERSON = 2.0   # m, robot centre to person: traffic
        self.TRAFFIC_NEAR_VEHICLE = 3.0  # m, to the nearest point of a vehicle
        self.TRAFFIC_LINGER = 3.0        # s it still counts after it clears
        self.TRAFFIC_STALE = 1.0         # s without a guard message: no guard
        self.TRAFFIC_PATIENCE = 60.0     # s standing in traffic without getting
        #   TRAFFIC_PROGRESS m further -> put it off till the end (pickers
        #   stand 5-9 s, the stepper 16 s, the forklift works 6-10 s at a time)
        self.TRAFFIC_PROGRESS = 0.5      # m
        self._block_ref = None
        self.TRAFFIC_RETRY_GAP = 2.0     # s before resending a goal Nav2 gave up on
        self.TRAFFIC_MAX_RETRIES = 12    # such resends per waypoint, then the ladder
        self.TRAFFIC_HARD_FACTOR = 3.0   # a waypoint may take at most this x its
        #   normal deadline in all, traffic or not, before it is put off
        self.RESCUE_CLEAR = 1.5          # m: never reverse with a person this close
        self.TRAFFIC_LOG_AFTER = 3.0     # s standing in traffic before it is logged
        self._traffic = None             # last /people_guard/state (dict)
        self._traffic_t = None           # node time it arrived
        self._traffic_last = -1e9        # node time traffic was last present
        self._guard_waiting = False
        self._hold_started = None
        self.holds = 0                   # guard WAIT episodes this mission
        self.held_s = 0.0                # their total length (s)
        self.last_hold = None            # (say, started, ended) of the last one
        self._tick_t = None              # previous tick_marker time (for dt)
        self._tick_xy = None
        self._blocked_s = 0.0            # standing in traffic on this waypoint (s)
        self._standing_s = 0.0           # this stand in traffic so far (s)
        self._stand_logged = False
        self._traffic_retries = 0
        self._defer_pending = False      # cancel issued to put the waypoint off
        self._defer_why = ''
        self._goal_from = None           # where the robot was when this waypoint
        self._goal_from_gi = None        #   was first sent (the start of its stretch)
        self._hard_deadline = None
        self.deferred = []               # waypoints put off till before home
        self.skipped = []                # waypoints given up for good
        self.revisited = 0
        self.create_subscription(String, '/people_guard/state', self.on_guard, 10)

        # ----- LANES (v10.5): see the header -----
        self.declare_parameter('follow_lanes', True)
        self.LANES_FOLLOW = bool(self.get_parameter('follow_lanes').value)
        # switchable at run time, from the next lane on, without a restart:
        #   ros2 param set /coverage_nav2 follow_lanes false
        self.add_on_set_parameters_callback(self._on_params)
        self.FOLLOW_CONTROLLER = 'FollowPath'   # controller_server plugin name
        self.ENTER_TOL = 0.45            # m off the lane: drive onto it first
        self.LANE_END_TOL = 0.35         # m left of a lane: it is done
        self.BYPASS_AFTER_GO = 15.0      # s standing in a lane while the guard
        #   says GO (Nav2 will not drive into someone standing in it)
        self.BYPASS_AFTER = 40.0         # s standing in a lane in any traffic
        self.BYPASS_SKIP = 2.0           # m: go round to at least this far on
        self.BYPASS_CLEAR = 1.6          # m past the person (personal space 0.9)
        self.BYPASS_MAX = 3              # per lane, then the patience rules
        self.DETOUR_MAX = 1.8            # m a way round may stray from a front/back
        #   row (in an aisle the test is exact: no way through another aisle)
        self.ENTER_MAX = 4               # times got onto a lane and still not on it: put off
        self.AT_GOAL = 0.25              # m: a goal this close is reached (goal checker xy)
        self.PATCH = True                # sweep what is still dirty before home
        self.PATCH_MIN = 0.75            # m: a shorter dirty stretch is not worth a trip
        self.CLEAN_PCT = 90.0            # % of an aisle's lane centre line painted
        self.PATCH_ROUNDS = 2            # v10.6: a 2nd round only for an aisle under CLEAN_PCT
        self._fp = (ActionClient(self, FollowPath, 'follow_path')
                    if FollowPath is not None else None)
        self.plan_pub = self.create_publisher(Path, '/plan', 1)
        self.create_subscription(Path, '/plan', self.on_plan, 1)
        self._sweep = None               # the lane being driven (dict)
        self._lane_path = None           # its path while FollowPath drives it
        self._plan_pub_t = -1e9
        self._own_plans = deque(maxlen=8)    # stamps of the lanes we put on /plan
        self._bypass_why = ''
        self._bypass_pending = False
        self._detour_pending = False
        self._send_id = 0                # every goal sent; stale results ignored
        self._patched = False
        self._patch_rounds = 0           # v10.6: patch rounds run this mission
        self.patches = 0                 # dirty stretches swept in the patch pass
        self.bypasses = 0
        self.swept_pct = {}              # aisle -> % of its lanes' centre painted
        self._swept_t = -1e9

        # ----- v10.6: see the header -----
        self.LANE_STOP = 0.35            # m kept between the robot's front and anything in its lane
        self.LANE_RESUME = 0.65          # m clear ahead before it sets off again after such a stop
        self.LANE_HALF = 0.33            # m: half the lane corridor checked (robot 0.30 + 0.03; a
        #   rack face is 0.42 m from the edge lanes, outside it)
        self.LANE_LOOK = 1.5             # m ahead of the robot's centre checked (never past the lane end)
        self.SCAN_STALE = 1.0            # s: an older /scan_nav is not used
        self.STOP_REACT = 0.4            # s from seeing it to the robot braking (scan, cancel, smoother)
        self.STOP_DECEL = 1.5            # m/s^2 the robot is assumed to brake at
        self._speed = 0.0                # m/s, from the last ticks
        self.VEH_SKIP_AFTER = 10.0       # s held for a vehicle at a lane's end or start: skip that bit
        #   (the forklift lifts for 6 s and waits 10 s for a robot in its way, then backs off)
        self.VEH_SKIP_LEN = 4.0          # m: how much of a lane counts as its end (or start)
        self.GUARD_LOST = 2.0            # s without the guard's state, once it was running: hold still
        self.SIM_PAUSE_WALL = 10.0       # s of real time with no sim time: the simulation froze
        self._scan = None                # last /scan_nav and when it came (sim s)
        self._scan_t = -1e9
        self._ahead = None               # m clear in the lane ahead (None: not known)
        self._ahead_hold = False         # the robot was stopped short of something in its lane
        self._veh_hold_s = 0.0           # s held for a vehicle, without a break
        self._veh_skip_pending = None    # 'end' | 'start' while that cancel is on its way
        self._enter_hint = {}            # lane end goal -> s to start that lane at
        self._guard_seen = False         # the guard reported during this node's life
        self._safety_hold = False        # holding still: the guard went quiet
        self._wall_last = None             # set at the first tick (start-up may take long)
        self.sim_pauses, self.sim_paused_s = 0, 0.0
        self.vehicle_skips, self.clearance_stops = 0, 0
        self.NAV2_REJECTS = 3            # goals rejected in a row: Nav2 is down (not this goal)
        self.NAV2_RETRY = 5.0            # s between tries while Nav2 is down
        self._rejects = 0
        self._nav2_down = False
        self._nav2_gone_since = None
        self._unready = 0                # dispatches that found no Nav2 server
        self._no_answer = 0              # goals never answered (timed out with no handle)
        self.nav2_downs = 0

        # ----- v10.7: turning on the spot, getting unstuck (see the header) -----
        self.ALIGN_MAX = 0.6             # rad (34 deg) off the lane: turn first, if near a rack
        self.ALIGN_NEAR = 0.85           # m: a rack or wall (static map) this close to the centre
        self.ALIGN_TOL = 0.12            # rad: lined up
        self.TURN_RATE = 0.6             # rad/s at most when turning on the spot
        self.TURN_MIN = 0.3              # rad/s at least (slower and the wheels barely turn)
        self.MAN_SETTLE = 1.2            # s after a goal ends before a manoeuvre drives (the
        #   velocity smoother sends its last zeros for up to its velocity_timeout, 1 s)
        self.MAN_BUDGET = 10.0           # s a manoeuvre may drive (time held by the guard not counted)
        self.MAN_MAX = 30.0              # s a manoeuvre may last in all, held or not
        self.WEDGE_S = 6.0               # s standing with a goal, the guard saying GO, nobody near
        self.WEDGE_FRONT = 0.90          # m ahead of the centre: RPP projects its arc up to the carrot
        #   (0.3-0.6 m) or 1 s, plus the 0.30 m radius, so a rack this far ahead can stop it
        self.WEDGE_HALF = 0.35           # m either side of the heading line (radius 0.30 + 5 cm): a rack
        #   face beside an edge lane (0.42-0.48 m off) is not 'in front'
        self.WEDGE_SIDE = 0.33           # m: this close anywhere (robot radius 0.30 + 3 cm)
        self.BACK_IF = 0.60              # m: something this close in front: back off before turning
        self.WEDGE_PERSON = 1.0          # m: nobody this close ...
        self.WEDGE_VEHICLE = 1.5         # m: ... and no vehicle this close
        self.UNSTICK_MAX = 2             # per waypoint, then the usual rules
        self.BACK_DIST = 0.20            # m backed off a rack or wall in front
        self.BACK_SPEED = 0.10           # m/s
        self.BACK_CLEAR = 0.35           # m clear behind the rear edge (/scan_nav) to back off
        self.TURN_CLEAR = 0.40           # m: nothing this close to the centre to turn (corners 0.36)
        self._awaiting = False           # set from auto_start below: no command yet
        self._man = None                 # the manoeuvre in progress (dict), or None
        self._wedge_s = 0.0              # s standing as above, this goal
        self._wedge_next = 0.0           # when to look at the geometry again
        self._unstick_pending = False    # cancel issued to get unstuck
        self._unstick_why = ''
        self._unsticks_gi = None         # the waypoint the count below is for
        self._unsticks_wp = 0
        self.unsticks = 0                # this mission
        self.aligns = 0
        self._last_plan = None           # (time, [(x, y), ...]) Nav2's last /plan
        self.create_subscription(
            LaserScan, '/scan_nav', self.on_scan,
            QoSProfile(depth=1, history=QoSHistoryPolicy.KEEP_LAST,
                       reliability=QoSReliabilityPolicy.BEST_EFFORT))

        # ----- COMMAND BUS -----
        # A validated command arrives here as JSON on /mission_command. This
        # node re-checks it rather than trusting the sender: the LLM layer is
        # upstream, and anything upstream of the robot is untrusted by default.
        # No pydantic here on purpose -- the robot node gains no dependency.
        self.declare_parameter('auto_start', True)
        self.auto_start = self.get_parameter('auto_start').value
        # v10.7: no command yet -> no goal is ever sent (not even by the safety hold's release)
        self._awaiting = not bool(self.auto_start)
        self._rescue_phase = None        # None | 'reversing' | 'settling'
        self._rescue_until = 0.0
        self._rescue_then = None
        self._goal_started = 0.0         # when the current goal was sent
        self.MIN_GOAL_GAP = 0.6          # s between a result and the next send
        self._last_result_at = 0.0
        self._send_at = 0.0
        self._send_pending = False
        # Set BEFORE the two loads below. It used to be assigned ~30 lines
        # further down, so _load_cleaned() and _load_pct() hit AttributeError,
        # returned the empty default, and nothing was ever restored.
        self.CLEANED_FILE = os.path.expanduser('~/.ros/warehouse_cleaned.json')
        self.cleaned = self._load_cleaned()   # zones finished, in order
        self.blocked = []                # zones reported blocked this session
        self.command_log = []            # what the operator asked for, in order
        self._zone_ok = set()            # zones with a success this mission
        self._already_clean = []         # zones build_mission found nothing left in
        self._zone_done = {}             # zone -> waypoints done THIS mission
        self._zone_total = {}            # zone -> waypoints in THIS mission
        # Session-level progress. _zone_done is wiped whenever a new mission
        # starts, so a zone left half-finished used to read as untouched --
        # the robot denied work it had actually done. This keeps the high-water
        # mark per zone for the whole session.
        self.zone_pct = self._load_pct()
        self.create_timer(0.1, self.tick_fast)
        self._epoch = 0                  # bumped by every accepted command
        self._paused = False
        self.ZONES = {'front', 'aisle_1', 'aisle_2', 'aisle_3', 'back', 'all',
                      'uncleaned'}
        self.ZONE_ORDER = ['front', 'aisle_1', 'aisle_2', 'aisle_3', 'back']
        # cleaned zones survive a restart, so "clean whatever is left" still
        # means something after the node is bounced (CLEANED_FILE is set above,
        # before the loads that need it)
        self.ACTIONS = {'clean', 'stop', 'resume', 'skip', 'noop',
                        'home', 'reset'}
        self.create_subscription(String, '/mission_command', self.on_command, 10)
        self.state_pub = self.create_publisher(String, '/mission_state', 10)
        self._done = False               # guards against late-callback re-fire

        # ----- per-goal watchdog (abandon a goal that hangs) -----
        self.goal_deadline = None
        # Distance-aware watchdog: a long aisle (~13m) at cruising speed takes
        # ~30-50s, so a FIXED timeout wrongly abandons it partway (the "doesn't
        # reach the aisle end" bug). Allow base + travel time at a conservative
        # effective speed (accounts for turns + regulated slowing).
        self.GOAL_TIMEOUT_BASE = 18.0    # seconds of slack per goal
        self.GOAL_EFF_SPEED = 0.15       # m/s worst-case effective speed
        self._goal_handle = None
        self._retries = 0                # per-goal retry counter
        self.MAX_RETRIES = 2             # retry an aborted goal before skipping
        self.create_timer(1.0, self.watchdog)

        # ----- Nav2 action client -----
        self._c = ActionClient(self, NavigateToPose, 'navigate_to_pose')
        self.get_logger().info(
            f'COVERAGE v7-measured: {len(self.goals)} waypoints planned '
            f'(bottom-wall + rack-base + top-wall strips included)')
        while not self._c.wait_for_server(timeout_sec=10.0):    # v10.6: say what it waits for
            self.get_logger().warn('still waiting for Nav2 (navigate_to_pose): is the Nav2 tab up?')
        if not self.auto_start:
            self.get_logger().info(
                'Nav2 ready. auto_start=false -> waiting for a command on '
                '/mission_command (run llm_commander, or publish one by hand).')
            return
        self.get_logger().info('Nav2 ready -> starting coverage.')
        self.send_next()

    # ============================================================= #
    #  MISSION BUILDER
    # ============================================================= #
    def world_to_map(self, wx, wy):
        """World -> MAP frame. Verified consistent with map origin [-4,-4]."""
        return (wx - self.spawn_x, wy - self.spawn_y)

    def aisle_lanes(self, x_left, x_right):
        """Lane X positions inside one aisle, EVENLY distributed across the
        drivable band [x_left+WALL_MARGIN, x_right-WALL_MARGIN].

        Even spacing guarantees BOTH near-wall strips get a lane. The old
        'start at lo, step by column_spacing, maybe add hi' logic bunched the
        lanes toward the middle and left the rack-side strips of each aisle
        uncleaned (the robot looked like it was sweeping only the centre)."""
        lo = x_left + self.WALL_MARGIN
        hi = x_right - self.WALL_MARGIN
        if hi <= lo:
            return [round((x_left + x_right) / 2.0, 3)]   # too narrow -> centre lane
        span = hi - lo
        # number of lanes so the gap between them is <= column_spacing
        n = max(2, int(math.ceil(span / self.column_spacing)) + 1)
        return [round(lo + span * i / (n - 1), 3) for i in range(n)]

    def sweep_lanes(self, lanes, start_at_bottom):
        """Boustrophedon sweep over a pre-ordered list of lane X positions.
        Turn points sit in OPEN space (END_MARGIN beyond the rack ends), so the
        robot never turns next to a rack corner. Returns (waypoints, ended_at_bottom)."""
        y_bot = self.rack_y_bot - self.END_MARGIN     # turn below the rack
        y_top = self.rack_y_top + self.END_MARGIN     # turn above the rack
        wp = []
        going_up = start_at_bottom
        for x in lanes:
            if going_up:
                wp.append((x, y_bot))
                wp.append((x, y_top))
            else:
                wp.append((x, y_top))
                wp.append((x, y_bot))
            going_up = not going_up
        ended_at_bottom = (abs(wp[-1][1] - y_bot) < 1e-6)
        return wp, ended_at_bottom

    # ---------- free-space map (for the open-floor area sweeps) ----------
    def _mark(self, occ, xc, yc, lx, ly):
        gx0 = int((xc - lx / 2) / self.resolution)
        gx1 = int((xc + lx / 2) / self.resolution)
        gy0 = int((yc - ly / 2) / self.resolution)
        gy1 = int((yc + ly / 2) / self.resolution)
        for gy in range(max(0, gy0), min(self.height_cells, gy1 + 1)):
            for gx in range(max(0, gx0), min(self.width_cells, gx1 + 1)):
                occ[gy][gx] = 1

    def build_free_map(self):
        """Occupancy grid of known obstacles, inflated by the robot radius.
        free[gy][gx] == True means the robot centre can safely be there."""
        occ = [[0] * self.width_cells for _ in range(self.height_cells)]
        for (cx, cy, lx, ly) in self.obstacles:
            self._mark(occ, cx, cy, lx, ly)
        self._occ_raw = occ              # un-inflated obstacles (for coverage %)
        r = self.inflate_cells
        free = [[True] * self.width_cells for _ in range(self.height_cells)]
        for gy in range(self.height_cells):
            for gx in range(self.width_cells):
                if occ[gy][gx]:
                    for dy in range(-r, r + 1):
                        for dx in range(-r, r + 1):
                            ny, nx = gy + dy, gx + dx
                            if 0 <= ny < self.height_cells and 0 <= nx < self.width_cells:
                                free[ny][nx] = False
        return free

    def is_floor_world(self, wx, wy):
        """True if (wx,wy) is open floor (NOT inside an obstacle). Un-inflated,
        used to count how much real floor exists / has been cleaned."""
        gx = int(wx / self.resolution)
        gy = int(wy / self.resolution)
        if 0 <= gx < self.width_cells and 0 <= gy < self.height_cells:
            return self._occ_raw[gy][gx] == 0
        return False

    def count_floor_cells(self):
        """Total cleanable floor cells at the coverage-grid resolution (once)."""
        cs = self.cov_cell
        total = 0
        nx = int(self.world_w / cs)
        ny = int(self.world_h / cs)
        for iy in range(ny):
            for ix in range(nx):
                if self.is_floor_world(ix * cs + cs / 2, iy * cs + cs / 2):
                    total += 1
        return max(1, total)

    def is_free_world(self, wx, wy):
        gx = int(wx / self.resolution)
        gy = int(wy / self.resolution)
        if 0 <= gx < self.width_cells and 0 <= gy < self.height_cells:
            return self.free[gy][gx]
        return False

    def free_x_segments(self, wy, x_lo, x_hi, min_len=0.5):
        """Free (x_start, x_end) intervals along a horizontal row at height wy,
        skipping obstacle columns. This is what makes the open sweeps wrap
        around the packing/pallet/office automatically."""
        segs = []
        step = self.resolution
        in_seg = False
        seg_start = None
        x = x_lo
        while x <= x_hi + 1e-6:
            if self.is_free_world(x, wy):
                if not in_seg:
                    in_seg = True
                    seg_start = x
            else:
                if in_seg:
                    in_seg = False
                    if x - step - seg_start >= min_len:
                        segs.append((round(seg_start, 3), round(x - step, 3)))
            x += step
        if in_seg and x_hi - seg_start >= min_len:
            segs.append((round(seg_start, 3), round(x_hi, 3)))
        return segs

    def area_sweep(self, y_lo, y_hi, x_lo, x_hi, start_left=True):
        """Horizontal boustrophedon filling a rectangular band, covering ONLY
        free cells (auto-avoids obstacles, reaches the outer walls). Used for
        the back zone and the whole front zone."""
        wp = []
        go_right = start_left
        n_rows = max(1, int(round((y_hi - y_lo) / self.row_spacing)) + 1)
        for i in range(n_rows):
            yy = round(y_lo + i * self.row_spacing, 3)
            segs = self.free_x_segments(yy, x_lo, x_hi)
            if not segs:
                continue
            if not go_right:
                segs = segs[::-1]
            for (sx, ex) in segs:
                a, b = (sx, ex) if go_right else (ex, sx)
                wp.append((a, yy))
                wp.append((b, yy))
            go_right = not go_right
        return wp

    def _via_aisle(self, wx, wy, up):
        """v10.10: WORLD-frame waypoints through the middle of the aisle nearest to x = wx, from
        the front corridor up to the back strip (up) or down. Going down, a robot already inside
        that aisle's span goes straight down it. Without them Nav2 took the shortest way round the
        racks: from the east end, the corridor along the east wall (see the docstring)."""
        l, r = min(self.aisles, key=lambda a: abs((a[0] + a[1]) / 2.0 - wx))
        cx = (l + r) / 2.0
        if up:
            return [(cx, self.CORRIDOR_Y), (cx, self.BACKSTRIP_Y)]
        if l <= wx <= r:
            return [(cx, self.CORRIDOR_Y)]
        return [(cx, self.BACKSTRIP_Y), (cx, self.CORRIDOR_Y)]

    def build_mission(self, zones=None):
        """Assemble the full coverage path in MAP-frame waypoints.

        Order is chosen to MINIMISE re-driving already-cleaned floor (a real
        cleaner shouldn't re-clean):
          1. FRONT zone first -- the robot spawns here, so it cleans on the way
             out instead of saving it for last and back-tracking to it.
          2. AISLES left-to-right -- each entered near its left wall, swept
             left->right, exiting near the right wall (= next aisle's left wall).
             The front zone we just cleaned is the aisle mouths, so we flow
             straight up into the aisles with no extra travel.
          3. BACK zone last -- after the aisles the robot is already at the top,
             so the back sweep starts right there (no long trip up).
          4. HOME.
        This removes the old big back->front traversal that crossed every aisle.
        The only unavoidable re-cross is the final trip HOME (return to dock)."""
        ZONES = ['front', 'aisle_1', 'aisle_2', 'aisle_3', 'back']

        # ORDER MATTERS. zones is a LIST, and the mission follows it exactly,
        # so "clean aisle 3 then aisle 1 then the back" does those three in
        # that order. The old version put zones in a set and always swept
        # left-to-right, silently ignoring what was asked for.
        self._already_clean = []
        # v10.5: the sweep pairs (start, end), MAP frame: every end becomes a
        # lane goal driven with FollowPath along the straight line from its start
        self._last_sweeps = {}
        pairs = []

        def as_pairs(seg_world):
            return [((seg_world[k][0] - self.spawn_x, seg_world[k][1] - self.spawn_y),
                     (seg_world[k + 1][0] - self.spawn_x, seg_world[k + 1][1] - self.spawn_y))
                    for k in range(0, len(seg_world) - 1, 2)]
        if zones is None:
            seq = list(ZONES)
        else:
            seq = []
            for z in zones:
                if z == 'all':
                    seq = list(ZONES)
                    break
                if z in ZONES and z not in seq:
                    seq.append(z)
        if not seq:
            return [], []

        # Start from where the robot ACTUALLY is, not from the dock. Building
        # every mission from spawn is why re-tasking mid-run sent the robot
        # back across the warehouse before starting the new zone.
        here = self._map_pose()
        if here is not None:
            sx, sy = here[0] + self.spawn_x, here[1] + self.spawn_y
        else:
            sx, sy = self.spawn_x, 4.0

        world_wp = [(sx, sy, 'start')]
        prev_x = sx
        # Enter the aisles from whichever END the robot is closer to. This was
        # hard-coded True, so a robot parked at the TOP of an aisle drove the
        # full length down to the bottom corridor, across, and back up again.
        mid_y = (self.rack_y_bot + self.rack_y_top) / 2.0
        enter_bottom = (sy <= mid_y)
        # v10.7: only the FIRST zone of the job starts from where the robot is; every later one
        # starts where the zone before it ends (shift_v106a: the robot stood below aisle 2 when
        # 'clean aisle 1, aisle 2' came, so aisle 2 was planned from the bottom with no transit,
        # although aisle 1 ends at the top: 138 s driving down aisle 2 before it swept anything)
        first_zone = True

        for z in seq:
            if z == 'front':
                if world_wp[-1][1] > self.rack_y_top:          # v10.10: from the back: via an aisle
                    for vx, vy in self._via_aisle(world_wp[-1][0], world_wp[-1][1], up=False):
                        world_wp.append((vx, vy, 'front'))
                for seg in (self.area_sweep(0.80, 0.80, 0.4, 14.6, start_left=True),
                            self.area_sweep(0.9, 4.9, 0.4, 14.6, start_left=False),
                            self.area_sweep(4.95, 4.95, 0.4, 14.6, start_left=True)):
                    pairs += as_pairs(seg)
                    for p in seg:
                        world_wp.append((p[0], p[1], 'front'))
                prev_x = world_wp[-1][0]
                enter_bottom = True
                first_zone = False
                continue

            if z == 'back':
                if world_wp[-1][1] < self.rack_y_bot:          # v10.10: from the front: via an aisle
                    for vx, vy in self._via_aisle(world_wp[-1][0], world_wp[-1][1], up=True):
                        world_wp.append((vx, vy, 'back'))
                    prev_x = world_wp[-1][0]
                world_wp.append((prev_x, self.BACKSTRIP_Y, 'back'))
                for seg in (self.area_sweep(18.1, 19.1, 0.4, 14.6, start_left=(prev_x < 7.5)),
                            self.area_sweep(19.25, 19.25, 0.4, 14.6, start_left=True)):
                    pairs += as_pairs(seg)
                    for p in seg:
                        world_wp.append((p[0], p[1], 'back'))
                prev_x = world_wp[-1][0]
                enter_bottom = False
                first_zone = False
                continue

            idx = int(z.split('_')[1]) - 1
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
            in_this_aisle = (first_zone and here is not None
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
            pairs += [(m_sweep[k], m_sweep[k + 1]) for k in range(0, len(m_sweep) - 1, 2)]

            # Standing in the aisle already? Then the corridor transit is pure
            # travel: go straight to the first thing that still needs sweeping.
            if not in_this_aisle:
                transit_y = self.CORRIDOR_Y if enter_bottom else self.BACKSTRIP_Y
                world_wp.append((prev_x, transit_y, z))
                world_wp.append((sweep[0][0], transit_y, z))
            for p in sweep:
                world_wp.append((p[0], p[1], z))
            prev_x = sweep[-1][0]
            enter_bottom = ended_bottom
            first_zone = False

        # drop consecutive points closer than 0.25 m (removes the brief
        # "waypoint right next to the last one" retrace), then label
        labelled, zone_of = [], []
        last = None
        n = 0
        for (wx, wy, zn) in world_wp:
            if last is not None and math.hypot(wx - last[0], wy - last[1]) < 0.35:
                continue
            mx, my = self.world_to_map(wx, wy)
            n += 1
            labelled.append((round(mx, 3), round(my, 3), f'wp{n}'))
            zone_of.append(zn)
            last = (wx, wy)
        # v10.5: which goals end a sweep (lane or row), and where it starts.
        # The 0.35 m merge above may drop a sweep's start; the goal before it
        # is then within 0.4 m of it, which still counts.
        for i in range(1, len(labelled)):
            bx, by, _ = labelled[i]
            for (a, b) in pairs:
                if abs(b[0] - bx) < 0.01 and abs(b[1] - by) < 0.01:
                    px, py, _ = labelled[i - 1]
                    if math.hypot(px - a[0], py - a[1]) < 0.4 and \
                            math.hypot(b[0] - a[0], b[1] - a[1]) >= 0.5:
                        self._last_sweeps[labelled[i]] = (round(a[0], 3), round(a[1], 3))
                    break
        return labelled, zone_of

    # ============================================================= #
    #  GOAL EXECUTION
    # ============================================================= #
    def send_next(self, delay=None):
        """Ask for the current goal to be sent. Does NOT send immediately.

        Nav2's BT navigator halts the previous behaviour tree when a goal
        ends. A goal fired into that window is accepted and then aborted
        within ~100 ms. Every resend in the failing logs went out 1 ms after
        the previous result and died; the ones that went out ~300 ms later ran
        fine. The old blocking rescue hid this by forcing a 5 s gap.

        So sends are paced: MIN_GOAL_GAP after the last result, and never
        while a rescue is in flight.
        """
        if self._done or self._paused or self._awaiting:     # v10.7: no command yet: nothing
            return
        now = self.get_clock().now().nanoseconds / 1e9
        gap = self.MIN_GOAL_GAP if delay is None else delay
        self._send_at = max(now + gap, self._last_result_at + self.MIN_GOAL_GAP)
        self._send_pending = True

    def _dispatch(self):
        if self._done or self._paused or self._awaiting:
            return
        if self._safety_hold:                   # v10.6: no goal while the guard is silent
            return
        if not self._c.server_is_ready():
            # v10.6: no navigate_to_pose server (Nav2 stopped or crashed): send nothing into
            # the void (an unanswered goal timed out and was skipped)
            self._unready += 1
            if self._unready >= 3 or self._nav2_down:
                self._nav2_lost('no Nav2 server (navigate_to_pose): it stopped or crashed')
            else:
                self.send_next(delay=1.0)
            return
        self._unready = 0
        self._before_home()
        if self.gi >= len(self.goals):
            self._finish_mission()
            return
        x, y, lbl = self.goals[self.gi]
        if lbl == 'HOME' and self.goal_zones[self.gi] == 'home':     # v10.10: down an aisle first
            here = self._map_pose()
            if here is not None and here[1] + self.spawn_y > self.rack_y_top:
                vias = self._via_aisle(here[0] + self.spawn_x, here[1] + self.spawn_y, up=False)
                new_g = []
                for k, (vx, vy) in enumerate(vias):
                    mx, my = self.world_to_map(vx, vy)
                    new_g.append((round(mx, 3), round(my, 3), f'via{k + 1}'))
                self.goals[self.gi:self.gi] = new_g
                self.goal_zones[self.gi:self.gi] = ['home'] * len(new_g)
                self.get_logger().info(f'HOME from above the racks: down the middle of the nearest aisle first '
                                       f'({", ".join(f"({g[0]:.2f},{g[1]:.2f})" for g in new_g)})')
                x, y, lbl = self.goals[self.gi]
        nxt = self._lane_start_of_next()
        if nxt is not None and self._enter_hint.get(nxt, 0.0) > 0.0:
            # v10.6: this lane is started further in (a vehicle at its start): its start is not needed
            self.get_logger().info(
                f'{self.gi + 1}/{len(self.goals)}: {lbl} not needed: the lane is started '
                f'{self._enter_hint[nxt]:.1f} m in')
            self._goal_succeeded()
            return
        a = self.sweeps.get((x, y, lbl)) if self._lanes_ok() else None
        if a is not None and self._dispatch_sweep(a, (x, y), lbl):
            return
        # v10.5: a goal the robot already stands on (a patch or revisit stretch
        # starting where the last one ended) is reached: Nav2 is not asked to
        # drive nowhere (it sat still until the stall watch fired)
        here = self._map_pose() if a is None else None
        if here is not None and math.hypot(x - here[0], y - here[1]) < self.AT_GOAL:
            self.get_logger().info(f'{self.gi + 1}/{len(self.goals)}: {lbl} already there')
            self._goal_succeeded()
            return
        self._send_nav(x, y, lbl)

    def _before_home(self):
        """v10.4: before going home, come back for what was put off; v10.5: then
        sweep whatever the painted grid says is still dirty. Run as soon as the
        next goal is the dock (advance), so /mission_state never says 'home' first."""
        if 0 <= self.gi < len(self.goals) and self.goal_zones[self.gi] == 'home':
            if self.deferred:
                self._insert_revisits()
            if (self.PATCH and not self._patched and 0 <= self.gi < len(self.goals)
                    and self.goal_zones[self.gi] == 'home'):
                self._insert_patches()
            elif (self.PATCH and self._patched and 0 < self._patch_rounds < self.PATCH_ROUNDS
                  and 0 <= self.gi < len(self.goals) and self.goal_zones[self.gi] == 'home'):
                self._update_swept(force=True)          # v10.6: a second round if needed
                low = [z for z, p in self.swept_pct.items() if p < self.CLEAN_PCT]
                if low:
                    self._insert_patches(only=low)
                else:
                    self._patch_rounds = self.PATCH_ROUNDS

    def _finish_mission(self):
        """Everything done: which zones are clean, and the summary lines."""
        self._done = True
        if self._hold_started is not None:              # close an open hold
            self.held_s += max(0.0, self._now() - self._hold_started)
            self._hold_started = None
        self._update_swept(force=True)
        for z in dict.fromkeys(self.goal_zones):
            sw = self.swept_pct.get(z)
            if sw is not None:
                # v10.5: an aisle is clean when its lanes are painted, not when
                # its waypoints were reached (shift_v104a: "cleaned" at 49 %)
                ok = sw >= self.CLEAN_PCT
            else:
                _done = self._zone_done.get(z, 0)
                _tot = self._zone_total.get(z, 0)
                ok = z in self._zone_ok and _tot and _done * 10 >= _tot * 9
            if ok and z not in self.cleaned:
                self.cleaned.append(z)         # in the order they were done
                self.zone_pct[z] = 100
                if z in self.blocked:
                    self.blocked.remove(z)
        self._save_cleaned()
        self._zone_ok = set()
        done_list = ', '.join(self.cleaned) if self.cleaned else 'nothing yet'
        self.get_logger().info(
            f'=== COVERAGE COMPLETE === cleaned so far: {done_list}')
        if self.swept_pct:
            self.get_logger().info(
                '    lanes swept (centre line painted): '
                + ', '.join(f'{z} {p:.0f} %' for z, p in self.swept_pct.items())
                + f'; patch pass {self.patches} stretch(es); went round '
                  f'{self.bypasses} blockage(s); left {self.vehicle_skips} lane end(s) to a '
                  f'vehicle; stopped short of something in a lane {self.clearance_stops}x; '
                  f'turned on the spot before a lane {self.aligns}x; got unstuck {self.unsticks}x')
        if self.nav2_downs:
            self.get_logger().info(f'    Nav2 stopped taking goals {self.nav2_downs}x (the robot held, '
                                   'nothing was skipped for it)')
        if self.sim_pauses:
            self.get_logger().info(
                f'    the simulation froze {self.sim_pauses}x, {self.sim_paused_s / 60:.1f} min '
                f'of real time in all (not counted as the robot\'s time)')
        self.get_logger().info(
            f'    traffic: held by the guard {self.holds}x, {self.held_s:.0f} s; '
            f'put off and came back for {self.revisited} stretch(es); '
            f'skipped for good {len(self.skipped)}'
            + (': ' + ', '.join(f"{s['label']} ({s['zone']})" for s in self.skipped)
               if self.skipped else ''))

    def _start_clock(self, x, y, hard_dist=None):
        """Deadline, stall watch and the waypoint's first-send bookkeeping.
        hard_dist (v10.5): what the whole waypoint is worth, for the hard
        deadline, when its first send is only a part of it (a lane entered first)."""
        dist = self._dist_to_goal(x, y)
        timeout = self.GOAL_TIMEOUT_BASE + dist / self.GOAL_EFF_SPEED
        self.goal_deadline = self.get_clock().now().nanoseconds / 1e9 + timeout
        if self._goal_from_gi != self.gi:   # v10.4: first send of this waypoint
            here = self._map_pose()
            if here is None and self.gi > 0:
                here = tuple(self.goals[self.gi - 1][:2])
            self._goal_from, self._goal_from_gi = here, self.gi
            whole = (timeout if hard_dist is None
                     else self.GOAL_TIMEOUT_BASE + hard_dist / self.GOAL_EFF_SPEED)
            self._hard_deadline = self._now() + self.TRAFFIC_HARD_FACTOR * whole
        self._stall_ref = None          # fresh goal -> restart stall watch
        self._wedge_s, self._wedge_next = 0.0, 0.0      # v10.7
        self._goal_started = self.get_clock().now().nanoseconds / 1e9

    def _send_nav(self, x, y, lbl, yaw=None, note='', hard_dist=None):
        """NavigateToPose to (x, y): Nav2 plans the way (turns, transits, and
        in v10.5 the way onto a lane or round someone standing in it)."""
        g = NavigateToPose.Goal()
        g.pose.header.frame_id = 'map'
        g.pose.header.stamp = self.get_clock().now().to_msg()
        g.pose.pose.position.x = float(x)
        g.pose.pose.position.y = float(y)
        # ----- yaw fix: point at the NEXT waypoint -----
        # Previously every goal had yaw=0 (East). On a North-South aisle leg
        # the robot would reach xy_tolerance heading North, then have to spin
        # 90 deg to satisfy yaw_goal_tolerance before the action could complete
        # -> rotate-then-stop at every goal, which is the wait-turn-then-next
        # stutter. Setting the goal yaw to point at the NEXT waypoint means
        # the robot arrives already facing where it needs to go.
        if yaw is None:
            if self.gi + 1 < len(self.goals):
                nx, ny, _ = self.goals[self.gi + 1]
                yaw = math.atan2(ny - y, nx - x)
            else:
                yaw = 0.0
        g.pose.pose.orientation.z = math.sin(yaw / 2.0)
        g.pose.pose.orientation.w = math.cos(yaw / 2.0)
        self.get_logger().info(
            f'{self.gi + 1}/{len(self.goals)}: {lbl} ({x:.2f},{y:.2f})'
            + (f' {note}' if note else ''))
        self._start_clock(x, y, hard_dist)
        self._lane_path = None
        self._send_goal(self._c, g)

    def _send_goal(self, client, goal):
        """Send a NavigateToPose or FollowPath goal; its results come back to
        goal_response / goal_done tagged, so a stale one is never mistaken for
        the current goal's (v10.5: two action types)."""
        self._send_id += 1
        epoch, sid = self._epoch, self._send_id
        client.send_goal_async(goal).add_done_callback(
            lambda f: self.goal_response(f, epoch, sid))

    # ---------------------------------------------------------------- lanes
    def _on_params(self, params):
        for p in params:
            if p.name == 'follow_lanes':
                self.LANES_FOLLOW = bool(p.value)
                self.get_logger().warn(
                    f'follow_lanes = {self.LANES_FOLLOW}: from the next lane on, lanes are driven '
                    + ('along the lane (FollowPath)' if self.LANES_FOLLOW
                       else 'as in v10.4 (NavigateToPose to the lane end)'))
        return SetParametersResult(successful=True)

    def _lanes_ok(self):
        return (self.LANES_FOLLOW and self._fp is not None
                and self._fp.server_is_ready())

    def _lane_pos(self, sw, p):
        """(s, off): how far along the lane p is (clamped to the lane) and how
        far p is from that point of the lane."""
        dx, dy = p[0] - sw['a'][0], p[1] - sw['a'][1]
        s = min(max(dx * sw['ux'] + dy * sw['uy'], 0.0), sw['L'])
        q = self._lane_point(sw, s)
        return s, math.hypot(p[0] - q[0], p[1] - q[1])

    def _lane_side(self, sw, p):
        """Distance of p from the lane's (infinite) line."""
        return abs(-(p[0] - sw['a'][0]) * sw['uy'] + (p[1] - sw['a'][1]) * sw['ux'])

    def _lane_point(self, sw, s):
        return (sw['a'][0] + sw['ux'] * s, sw['a'][1] + sw['uy'] * s)

    def _cur_sweep(self):
        sw = self._sweep
        return sw if (sw is not None and sw['gi'] == self.gi) else None

    def _dispatch_sweep(self, a, b, lbl):
        """Drive the lane a -> b: onto it first if the robot is off it (or
        round someone standing in it: the bypass target), then FollowPath along
        it from where the robot is. False: could not (no pose): use Nav2."""
        here = self._map_pose()
        L = math.hypot(b[0] - a[0], b[1] - a[1])
        if here is None or L < 0.3:
            return False
        key = (b[0], b[1], lbl)
        sw = self._cur_sweep()
        if sw is None or sw['key'] != key:
            sw = self._sweep = dict(key=key, gi=self.gi, a=a, b=b, L=L,
                                    ux=(b[0] - a[0]) / L, uy=(b[1] - a[1]) / L,
                                    target=None, bypasses=0, phase=None, entered=0,
                                    stand=0.0, stand_go=0.0, ref_s=None, s0=0.0,
                                    box=(self._aisle_box(a[0]) if abs(a[0] - b[0]) < 0.05
                                         else None),
                                    # v10.6: a vehicle was at its start: begin this far in
                                    s_min=min(self._enter_hint.pop(key, 0.0),
                                              max(0.0, L - self.LANE_END_TOL - 0.1)))
        s, off = self._lane_pos(sw, here)
        if sw['target'] is not None and s >= sw['target'] - 0.3 and off <= self.ENTER_TOL:
            sw['target'] = None                     # gone round: back on the lane
        if sw['target'] is None and off <= self.ENTER_TOL:
            if L - s < self.LANE_END_TOL:
                self.get_logger().info(
                    f'{self.gi + 1}/{len(self.goals)}: {lbl} lane done')
                self._goal_succeeded()
                return True
            if self._bypass_due(sw):                # stood behind someone long enough
                r = self._plan_bypass(sw, self._stand_why(sw))
                if r == 'done':
                    self._goal_succeeded()
                    return True
        if sw['target'] is not None or off > self.ENTER_TOL or s < sw['s_min'] - 0.3:
            if sw['entered'] >= self.ENTER_MAX:     # Nav2 got there, yet not on the lane
                self.get_logger().warn(
                    f'{lbl}: not on the lane after getting onto it {self.ENTER_MAX} times')
                self._defer_current('could not get onto the lane')
                self.advance()
                return True
            s_go = sw['target'] if sw['target'] is not None else max(s, sw['s_min'])
            px, py = self._lane_point(sw, s_go)
            sw['phase'] = 'enter'
            self._send_nav(px, py, lbl, yaw=math.atan2(sw['uy'], sw['ux']),
                           note=('going round, back onto the lane' if sw['target'] is not None else
                                 f'onto the lane {s_go:.1f} m in (a vehicle at its start)'
                                 if s_go > s + 0.3 else 'onto the lane first'),
                           hard_dist=off + (L - s))
            return True
        # v10.6: never set off towards something standing in the lane ahead
        ahead = self._clear_ahead(sw, here)
        if ahead is None and (self._ahead_hold or self._guard_seen):
            # ... nor blind: no fresh /scan_nav (the guard is late, or stopping: then the safety
            # hold takes over). Without a guard at all, lanes are driven as before.
            sw['phase'] = 'follow'
            if not sw.get('blind_said'):
                sw['blind_said'] = True
                self.get_logger().info(f'CLEARANCE: {lbl}: no fresh scan from the guard -> waiting '
                                       'before setting off along the lane')
            self.send_next(delay=1.0)
            return True
        if ahead is not None and ahead < (self.LANE_RESUME if self._ahead_hold else self.LANE_STOP + 0.05):
            sw['phase'] = 'follow'
            if sw['stand'] > self.TRAFFIC_PATIENCE:
                self._ahead_hold = False
                self._defer_current(f'something stood in the lane, {ahead:.2f} m ahead, for '
                                    f'{sw["stand"]:.0f} s')
                self.advance()
                return True
            if not self._ahead_hold:
                self._ahead_hold = True
                self.clearance_stops += 1
                self.get_logger().info(
                    f'CLEARANCE: {lbl}: something stands {ahead:.2f} m ahead in the lane -> '
                    f'waiting short of it')
            self.send_next(delay=1.0)               # look again in a second (counts as standing)
            return True
        if self._ahead_hold:
            self.get_logger().info(f'CLEARANCE: {lbl}: the lane ahead is clear again')
        self._ahead_hold = False
        if self._align_first(sw, lbl):             # v10.7: turn on the spot first, next to a rack
            return True
        sw['phase'] = 'follow'
        sw['s0'] = s
        if sw['ref_s'] is None:
            sw['ref_s'] = s
        x0, y0 = self._lane_point(sw, s)
        self.get_logger().info(
            f'{self.gi + 1}/{len(self.goals)}: {lbl} ({b[0]:.2f},{b[1]:.2f}) sweeping the lane '
            f'from ({x0:.2f},{y0:.2f}), {L - s:.1f} m')
        g = FollowPath.Goal()
        g.path = self._lane_msg(sw, s)
        g.controller_id = self.FOLLOW_CONTROLLER
        self._start_clock(b[0], b[1], hard_dist=L - s)
        self._lane_path = g.path
        self._publish_lane_plan()
        self._send_goal(self._fp, g)
        return True

    def on_scan(self, msg):
        """v10.6: /scan_nav, the scan Nav2 sees (standing people at their personal space,
        vehicles with their fork ground, everything else as the lidar saw it). While a lane is
        driven, every scan is checked: the robot stops LANE_STOP short of anything in the lane,
        braking early enough for its speed."""
        self._scan, self._scan_t = msg, self._now()
        sw = self._cur_sweep()
        if (sw is None or sw['phase'] != 'follow' or self._lane_path is None or self._goal_handle is None
                or self._done or self._paused or self._ahead_hold or self._bypass_pending
                or self._defer_pending or self._detour_pending or self._veh_skip_pending is not None):
            return
        ahead = self._clear_ahead(sw)
        self._ahead = ahead
        if ahead is None or ahead >= self._stop_gap():
            return
        self._ahead_hold = True
        self.clearance_stops += 1
        self.get_logger().info(
            f'CLEARANCE: {sw["key"][2]}: something stands {ahead:.2f} m ahead in the lane '
            f'(at {self._speed:.2f} m/s) -> stopping short of it')
        self.goal_deadline = None
        try:
            self._goal_handle.cancel_goal_async()
        except Exception:
            pass

    def _stop_gap(self):
        """Gap ahead at which to stop: LANE_STOP plus reaction and braking at the current speed."""
        v = max(0.0, self._speed)
        return self.LANE_STOP + v * self.STOP_REACT + v * v / (2.0 * self.STOP_DECEL)

    def _clear_ahead(self, sw, here=None):
        """m between the robot's front and the nearest point in its lane ahead (LANE_HALF either
        side of the lane line, up to LANE_LOOK ahead and never past the lane's end); inf if
        nothing is there; None without a fresh scan or TF. Sideways it is measured from the lane
        line, not from the robot: just onto a lane, 0.12 m off it, the robot's own line grazed the
        corner of rack 2 (shift_v105a's scans replayed: the only stop for nothing in the lane)."""
        msg = self._scan
        if msg is None or self._now() - self._scan_t > self.SCAN_STALE:
            return None
        here = here or self._map_pose()
        if here is None:
            return None
        try:
            tf = self.tf_buffer.lookup_transform('map', msg.header.frame_id, rclpy.time.Time())
        except Exception:
            return None
        lx, ly = tf.transform.translation.x, tf.transform.translation.y
        q = tf.transform.rotation
        lyaw = math.atan2(2.0 * (q.w * q.z + q.x * q.y), 1.0 - 2.0 * (q.y * q.y + q.z * q.z))
        ux, uy = sw['ux'], sw['uy']
        ax, ay = sw['a']
        s_here = self._lane_pos(sw, here)[0]
        look = min(self.LANE_LOOK, sw['L'] - s_here + self.ROBOT_RADIUS)
        best = None
        a0, inc = msg.angle_min + lyaw, msg.angle_increment
        for k, r in enumerate(msg.ranges):
            if not (msg.range_min < r < msg.range_max):     # also drops NaN and inf
                continue
            ang = a0 + k * inc
            hx, hy = lx + r * math.cos(ang), ly + r * math.sin(ang)
            along = (hx - here[0]) * ux + (hy - here[1]) * uy
            if along <= 0.0 or along > look or abs(-(hx - ax) * uy + (hy - ay) * ux) > self.LANE_HALF:
                continue
            gap = along - self.ROBOT_RADIUS
            if best is None or gap < best:
                best = gap
        return max(0.0, best) if best is not None else float('inf')

    def _lane_start_of_next(self):
        """The next goal, if the current one is the start of its lane (a sweep pair's a)."""
        if not (0 <= self.gi < len(self.goals) - 1):
            return None
        x, y, _ = self.goals[self.gi]
        nxt = self.goals[self.gi + 1]
        a = self.sweeps.get(nxt)
        return nxt if (a is not None and math.hypot(a[0] - x, a[1] - y) < 0.4) else None

    def _veh_skip(self, kind):
        """v10.6: held for a vehicle at a lane's end (kind 'end') or on the way to its start
        ('start'): leave that bit for the patch pass and carry on. In the patch pass ('later'):
        that stretch after the others."""
        self._veh_skip_pending = None
        self._veh_hold_s = 0.0
        self.vehicle_skips += 1
        lbl = str(self.goals[self.gi][2]) if 0 <= self.gi < len(self.goals) else '?'
        if kind == 'later':
            self._veh_later(lbl)
            return
        if kind == 'end':
            sw = self._cur_sweep()
            here = self._map_pose()
            left = (sw['L'] - self._lane_pos(sw, here)[0]) if (sw and here) else 0.0
            # the next lane, if it starts beside this end, is in the same vehicle's ground
            beside = ''
            if self.gi + 2 < len(self.goals) and sw is not None:
                a2, b2 = self.goals[self.gi + 1], self.goals[self.gi + 2]
                s2 = self.sweeps.get(b2)
                if (s2 is not None and math.hypot(s2[0] - a2[0], s2[1] - a2[1]) < 0.4
                        and math.hypot(a2[0] - sw['b'][0], a2[1] - sw['b'][1]) <= 1.2
                        and not str(b2[2]).lower().startswith(('pt', 'rv'))):
                    self._enter_hint[b2] = self.VEH_SKIP_LEN
                    beside = (f'; the next lane starts beside it: started {self.VEH_SKIP_LEN:.1f} m in, '
                              f'its first {self.VEH_SKIP_LEN:.1f} m left too')
            self.get_logger().warn(
                f'VEHICLE: held {self.VEH_SKIP_AFTER:.0f} s for a vehicle at the end of lane {lbl} -> '
                f'the lane is done here, the last {left:.1f} m are left for the patch pass{beside}')
        else:
            nxt = self._lane_start_of_next()
            if nxt is not None:
                self._enter_hint[nxt] = self.VEH_SKIP_LEN
            self.get_logger().warn(
                f'VEHICLE: held {self.VEH_SKIP_AFTER:.0f} s for a vehicle at the start of the next lane '
                f'-> starting it {self.VEH_SKIP_LEN:.1f} m in, the first {self.VEH_SKIP_LEN:.1f} m are '
                f'left for the patch pass')
        self._goal_succeeded()

    def _veh_later(self, lbl):
        """v10.6: held VEH_SKIP_AFTER for a vehicle in the patch pass: this stretch is put off
        till the other stretches are done (a revisit, which keeps the normal patience)."""
        why = f'a vehicle held it {self.VEH_SKIP_AFTER:.0f} s'
        nxt = self._lane_start_of_next()
        if self._cur_sweep() is None and nxt is not None:     # on the way to the stretch's start
            a = self.sweeps.get(nxt)
            z = self.goal_zones[self.gi + 1] if self.gi + 1 < len(self.goal_zones) else ''
            self.deferred.append({'zone': z, 'from': (float(a[0]), float(a[1])),
                                  'to': (float(nxt[0]), float(nxt[1])), 'label': nxt[2], 'why': why})
            self.get_logger().warn(
                f'VEHICLE: {why} on the way to patch stretch {nxt[2]} -> the other stretches first, '
                f'then this one ({len(self.deferred)} put off)')
            self.gi += 1                            # its start goal is not needed now
        else:
            # not _defer_current: a patch stretch counts as a second go there and would be
            # skipped for good; this is a put-off, once (the revisit is the last go)
            sw = self._cur_sweep()
            here = self._map_pose()
            x, y, _ = self.goals[self.gi]
            if sw is not None:
                s = self._lane_pos(sw, here)[0] if here is not None else 0.0
                a, b = self._lane_point(sw, max(0.0, s - 0.3)), sw['b']
            else:
                a, b = (here or (x, y)), (x, y)
            self.deferred.append({'zone': self.goal_zones[self.gi], 'from': (float(a[0]), float(a[1])),
                                  'to': (float(b[0]), float(b[1])), 'label': lbl, 'why': why})
            self.get_logger().warn(f'VEHICLE: {why} on patch stretch {lbl} -> the other stretches '
                                   f'first, then the rest of this one ({len(self.deferred)} put off)')
        self._retries = 0
        self.advance()

    def _lane_msg(self, sw, s, step=0.05):
        """nav_msgs/Path along the lane from s to its end (map frame)."""
        path = Path()
        path.header.frame_id = 'map'
        path.header.stamp = self.get_clock().now().to_msg()
        yaw = math.atan2(sw['uy'], sw['ux'])
        n = max(2, int(math.ceil((sw['L'] - s) / step)) + 1)
        for k in range(n):
            sk = s + (sw['L'] - s) * k / (n - 1)
            ps = PoseStamped()
            ps.header = path.header
            ps.pose.position.x, ps.pose.position.y = self._lane_point(sw, sk)
            ps.pose.orientation.z = math.sin(yaw / 2.0)
            ps.pose.orientation.w = math.cos(yaw / 2.0)
            path.poses.append(ps)
        return path

    def _publish_lane_plan(self):
        """While FollowPath drives a lane, the rest of it goes on /plan, where
        the planner would put its path: the guard predicts with it, RViz shows it."""
        sw = self._cur_sweep()
        if self._lane_path is None or sw is None or sw['phase'] != 'follow':
            return
        here = self._map_pose()
        s = self._lane_pos(sw, here)[0] if here is not None else sw['s0']
        msg = self._lane_msg(sw, s, step=0.1)
        self._own_plans.append((msg.header.stamp.sec, msg.header.stamp.nanosec))
        self.plan_pub.publish(msg)
        self._plan_pub_t = self._now()

    def _aisle_box(self, mx):
        """(x_left, x_right, y_lo, y_hi), map frame, of the aisle holding map x
        mx, between the rack ends (0.3 m in from them), or None."""
        ylo = self.rack_y_bot - self.spawn_y + 0.3
        yhi = self.rack_y_top - self.spawn_y - 0.3
        for (wl, wr) in self.aisles:
            xl, xr = wl - self.spawn_x, wr - self.spawn_x
            if xl - 0.05 <= mx <= xr + 0.05:
                return (xl, xr, ylo, yhi)
        return None

    def on_plan(self, msg):
        """The planner's path while the robot gets onto a lane or round someone
        standing in it. In an aisle: a way through another aisle (shift_v104a:
        aisle 2's east lanes "reached" through aisle 3) is refused; round the
        rack ends is fine. Elsewhere (front and back rows): a way straying more
        than DETOUR_MAX from the row is refused."""
        if (msg.header.stamp.sec, msg.header.stamp.nanosec) in self._own_plans:
            return                                  # our own lane, from _publish_lane_plan
        if msg.poses:                               # v10.7: where Nav2 means to go (UNSTICK turns to it)
            self._last_plan = (self._now(), [(p.pose.position.x, p.pose.position.y)
                                             for p in msg.poses[::2]])
        sw = self._cur_sweep()
        if (sw is None or sw['phase'] != 'enter' or self._goal_handle is None
                or self._detour_pending or self._defer_pending or not msg.poses):
            return
        here = self._map_pose()
        worst, box = 0.0, sw.get('box')
        if box is not None:
            xl, xr, ylo, yhi = box
            mine = None                     # the aisle the robot is leaving, if another one
            if here is not None and ylo < here[1] < yhi:
                mine = self._aisle_box(here[0])
                mine = None if mine == box else mine
            for p in msg.poses[::3]:
                px, py = p.pose.position.x, p.pose.position.y
                if not (ylo < py < yhi) or xl - 0.1 <= px <= xr + 0.1:
                    continue                # in this aisle, or round a rack end
                if mine is not None and mine[0] - 0.1 <= px <= mine[1] + 0.1:
                    continue                # still on the way out of the robot's aisle
                worst = max(worst, self._lane_side(sw, (px, py)))
            stray = worst > 0.0
            what = 'goes through another aisle'
        else:
            lat0 = self._lane_side(sw, here) if here is not None else 0.0
            for p in msg.poses[::5]:
                worst = max(worst, self._lane_side(sw, (p.pose.position.x, p.pose.position.y)))
            stray = worst > max(self.DETOUR_MAX, lat0 + 1.0)
            what = 'strays from the row'
        if stray:
            self._detour_pending = True
            self.goal_deadline = None
            self.get_logger().warn(
                f'DETOUR: Nav2\'s way to lane {sw["key"][2]} {what} ({worst:.1f} m from the '
                f'lane) -> not taking it')
            try:
                self._goal_handle.cancel_goal_async()
            except Exception:
                pass

    def _vehicle_near(self):
        g = self._traffic if self._guard_fresh(self._now()) else None
        vm = g.get('vehicle_m') if g else None
        return isinstance(vm, (int, float)) and vm < self.TRAFFIC_NEAR_VEHICLE

    def _bypass_due(self, sw):
        """Time to go round what stands in the lane. Never round a vehicle: it
        moves on (or the patience rules put the stretch off)."""
        return (sw is not None and sw['phase'] == 'follow' and sw['bypasses'] < self.BYPASS_MAX
                and (sw['stand_go'] >= self.BYPASS_AFTER_GO or sw['stand'] >= self.BYPASS_AFTER)
                and not self._vehicle_near())

    def _stand_why(self, sw):
        if sw['stand_go'] >= self.BYPASS_AFTER_GO:
            return (f'standing {sw["stand_go"]:.0f} s in the lane with nobody crossing: '
                    f'someone (or something) is standing in it')
        return f'waiting {sw["stand"]:.0f} s in the lane'

    def _plan_bypass(self, sw, why):
        """Someone (or something) stands in the lane: the target becomes the lane
        point past them, and the robot carries on along the lane from there; what
        it goes round stays dirty for the patch pass. 'target' (go round),
        'done' (the blockage is at the lane's end: the lane is done here) or None."""
        here = self._map_pose()
        if here is None or sw is None:
            return None
        s = self._lane_pos(sw, here)[0]
        g = self._traffic if self._guard_fresh(self._now()) else None
        ahead = g.get('person_m') if g else None
        ahead = ahead if isinstance(ahead, (int, float)) and 0.5 < ahead < 4.0 else 0.6
        target = min(sw['L'], s + max(self.BYPASS_SKIP, ahead + self.BYPASS_CLEAR))
        sw['stand'] = sw['stand_go'] = 0.0
        sw['ref_s'] = None
        self._traffic_retries = 0
        self._retries = 0
        if sw['L'] - target < self.LANE_END_TOL:
            self.get_logger().warn(
                f'BYPASS {sw["key"][2]}: {why}; that is at the end of the lane -> the lane '
                f'is done here, the last {sw["L"] - s:.1f} m are left for the patch pass')
            return 'done'
        sw['target'] = target
        sw['bypasses'] += 1
        sw['entered'] = 0
        self.bypasses += 1
        tx, ty = self._lane_point(sw, target)
        self.get_logger().warn(
            f'BYPASS {sw["key"][2]}: {why} -> going round to ({tx:.2f},{ty:.2f}) on the lane '
            f'({target - s:.1f} m on), then on along it; the stretch gone round is left '
            f'for the patch pass ({sw["bypasses"]}/{self.BYPASS_MAX})')
        return 'target'

    def _go_round(self, sw, why):
        """_plan_bypass, then carry on. True if it took over."""
        r = self._plan_bypass(sw, why)
        if r == 'done':
            self._goal_succeeded()
            return True
        if r == 'target':
            self.send_next()
            return True
        return False

    def _goal_succeeded(self):
        """The current waypoint is done: credit it and go on."""
        self._retries = 0
        self._consec_fails = 0
        self._defer_pending = False
        if 0 <= self.gi < len(self.goal_zones):
            # v10.4: a revisit stretch counts once, at its end (label RVn),
            # standing in for the waypoint that was put off; v10.5: a patch
            # (ptN) never counts: it is floor the zone already had
            if not str(self.goals[self.gi][2]).startswith(('rv', 'pt')):
                self._credit(self.goal_zones[self.gi])
        self.advance()

    def _aisle_geom(self, z):
        """(lanes in the map frame, y_lo, y_hi, x_left, x_right) of aisle z, or None."""
        if not str(z).startswith('aisle_'):
            return None
        idx = int(z.split('_')[1]) - 1
        if not (0 <= idx < len(self.aisles)):
            return None
        x_left, x_right = self.aisles[idx]
        m_lanes = [lx - self.spawn_x for lx in self.aisle_lanes(x_left, x_right)]
        m_lo = (self.rack_y_bot - self.END_MARGIN) - self.spawn_y
        m_hi = (self.rack_y_top + self.END_MARGIN) - self.spawn_y
        return m_lanes, m_lo, m_hi, x_left - self.spawn_x, x_right - self.spawn_x

    def _update_swept(self, force=False):
        """% of each aisle's lane centre line painted, for this mission's aisles."""
        now = self._now()
        if not force and now - self._swept_t < 2.0:
            return
        self._swept_t = now
        for z in dict.fromkeys(self.goal_zones):
            geo = self._aisle_geom(z)
            if geo is None:
                continue
            done, total = zone_progress(geo[0], geo[1], geo[2], self.visited, cell=self.cov_cell)
            self.swept_pct[z] = round(100.0 * done / total, 1)
            # what /mission_state reports as the aisle's progress (it was the share
            # of waypoints reached: shift_v104a said 100 % for 49 % painted)
            self.zone_pct[z] = max(self.zone_pct.get(z, 0), int(self.swept_pct[z]))

    def _insert_patches(self, only=None):
        """Before going home: every lane of every aisle in this job, against
        the painted grid; its dirty stretches (lane_plan: the dirty runs of the
        lane centre line, grown 0.2 m, at least PATCH_MIN long) are swept now,
        nearest aisle first. Once per mission; v10.6: a second round for the
        aisles in only (still under CLEAN_PCT after the first)."""
        self._patched = True
        self._patch_rounds += 1
        again = only is not None
        zones = [z for z in dict.fromkeys(self.goal_zones) if self._aisle_geom(z) is not None
                 and (only is None or z in only)]
        if not zones:
            return
        here = self._map_pose()
        n0 = self.patches
        new_g, new_z, new_sweeps, n, where = [], [], {}, n0, []
        for z in reversed(zones):                   # the robot is in the last one
            m_lanes, m_lo, m_hi, xl, xr = self._aisle_geom(z)
            if here is not None and abs(here[0] - m_lanes[-1]) < abs(here[0] - m_lanes[0]):
                m_lanes = list(reversed(m_lanes))   # start from the nearer side
            in_z = here is not None and xl <= here[0] <= xr
            up = here is None or here[1] <= (m_lo + m_hi) / 2.0
            wp, _ = lane_plan(m_lanes, m_lo, m_hi, self.visited, up,
                              robot_y=(here[1] if in_z else None), cell=self.cov_cell,
                              min_len=self.PATCH_MIN)
            k0 = n
            for k in range(0, len(wp) - 1, 2):
                n += 1
                a = (round(wp[k][0], 3), round(wp[k][1], 3))
                b = (round(wp[k + 1][0], 3), round(wp[k + 1][1], 3))
                ga, gb = (a[0], a[1], f'pt{n}a'), (b[0], b[1], f'pt{n}b')
                new_g += [ga, gb]
                new_z += [z, z]
                new_sweeps[gb] = a
            if n > k0:
                where.append(f'{z} {n - k0}')
            if wp:
                here = wp[-1]
        if new_g:
            self.goals[self.gi:self.gi] = new_g
            self.goal_zones[self.gi:self.gi] = new_z
            self.sweeps.update(new_sweeps)
            self.patches = n
            if again:
                self.get_logger().warn(
                    f'PATCH round 2: {", ".join(f"{z} {self.swept_pct.get(z, 0):.0f} %" for z in zones)} '
                    f'is still under {self.CLEAN_PCT:.0f} % -> {n - n0} stretch(es) once more '
                    f'({", ".join(where)}) before going home')
            else:
                self.get_logger().warn(
                    f'PATCH: {n - n0} stretch(es) of lane still dirty ({", ".join(where)}) -> '
                    f'sweeping them before going home')
        elif again:
            self.get_logger().info('PATCH round 2: nothing long enough left to be worth a trip')
        else:
            self.get_logger().info('PATCH: every lane is swept, nothing to patch')

    def _dist_to_goal(self, gx, gy):
        """Straight-line distance from the robot (or previous goal) to (gx,gy)."""
        try:
            t = self.tf_buffer.lookup_transform('map', 'base_link', rclpy.time.Time())
            rx, ry = t.transform.translation.x, t.transform.translation.y
        except Exception:
            if self.gi > 0:
                rx, ry, _ = self.goals[self.gi - 1]
            else:
                rx, ry = gx, gy
        return math.hypot(gx - rx, gy - ry)

    def goal_response(self, fut, epoch=0, sid=None):
        gh = fut.result()
        if epoch != self._epoch or (sid is not None and sid != self._send_id):
            # superseded (a new command, or a newer send): its rejection used to
            # call advance() below and skip the NEW mission's first waypoint.
            # v10.5: an accepted one is cancelled, or it would drive on by itself
            # (or, two action types now, fight the current goal for the controller)
            if gh is not None and gh.accepted:
                try:
                    gh.cancel_goal_async()
                except Exception:
                    pass
            return
        if not gh.accepted:
            # v10.6: Nav2 rejects goals when it is not active (its lifecycle manager shut it
            # down): skipping meant skipping the whole job in seconds. Retry; then hold.
            self._rejects += 1
            self.goal_deadline = None
            if self._rejects >= self.NAV2_REJECTS:
                self._nav2_lost(f'{self._rejects} goals rejected in a row: Nav2 is not taking goals '
                                '(not active: its lifecycle manager may have shut it down)')
            else:
                self.get_logger().warn(f'wp{self.gi + 1} REJECTED by Nav2 -> the same goal again in 2 s')
                self.send_next(delay=2.0)
            return
        self._no_answer = 0
        if self._rejects or self._nav2_down:
            if self._nav2_down:
                self.get_logger().warn('NAV2: taking goals again -> carrying on')
                self._hard_deadline = None          # the time it was down is not the waypoint's
                self._retries = 0                   # nor the goal Nav2 aborted when it went down
            self._rejects, self._nav2_down = 0, False
        self._goal_handle = gh
        gh.get_result_async().add_done_callback(
            lambda f: self.goal_done(f, epoch, sid))

    def goal_done(self, fut, epoch=0, sid=None):
        # A result from a superseded mission must not drive the failure ladder,
        # or cancelling "clean all" to run "clean aisle 3" would make the old
        # goal retry itself on top of the new mission.
        if epoch != self._epoch or (sid is not None and sid != self._send_id):
            return
        # GoalStatus: 4 = SUCCEEDED, 5 = CANCELED, 6 = ABORTED
        status = fut.result().status
        self._goal_handle = None
        self._lane_path = None              # v10.5: FollowPath is not driving now
        self._last_result_at = self.get_clock().now().nanoseconds / 1e9
        if self._done:
            return
        sw = self._cur_sweep()
        phase = sw['phase'] if sw is not None else None
        detour, self._detour_pending = self._detour_pending, False
        bypass, self._bypass_pending = self._bypass_pending, False
        unstick, self._unstick_pending = self._unstick_pending, False      # v10.7
        if self._safety_hold:                   # v10.6: holding still; resumes when the guard is back
            return
        if self._veh_skip_pending is not None and status != 4:
            self._veh_skip(self._veh_skip_pending)
            return
        if self._ahead_hold and status != 4 and phase == 'follow':
            self.send_next(delay=1.0)           # v10.6: stopped short on purpose: look again
            return
        if status == 4:
            if phase == 'enter':
                # v10.5: on the lane (or past what stood in it): now sweep it
                sw['entered'] += 1
                self._retries = 0
                self._traffic_retries = 0
                self.send_next()
                return
            self._goal_succeeded()
            return
        # ---- v10.4: put off on purpose (blocked by traffic for too long) ----
        now = self._now()
        if self._defer_pending:
            self._defer_pending = False
            self._retries = 0
            self._defer_current(self._defer_why)
            self.advance()
            return
        # ---- v10.5: the only way round was out of the aisle: the rest of the
        # lane is put off (revisit, then the patch pass), never driven that way
        if detour:
            self._retries = 0
            self._defer_current("the only way round was out of the aisle")
            self.advance()
            return
        # ---- v10.7: wedged against a rack or wall, not traffic: back off, turn, the same goal
        # again. RPP aborts a lane it sees no way along after failure_tolerance (5 s), before
        # the tick's WEDGE_S, so an abort there is looked at here too.
        if unstick:
            self._start_unstick(self._unstick_why)
            return
        if status == 6 and self._unsticks_left():
            w = self._wedged_now()
            if w is not None:
                self._start_unstick(
                    f'Nav2 gave up (status 6) with the guard saying go, nobody within '
                    f'{self.WEDGE_PERSON:.1f} m and a rack or wall '
                    + (f'{w[0]:.2f} m ahead' if w[0] <= self.WEDGE_FRONT else f'{w[1]:.2f} m away'))
                return
        # ---- v10.5: someone stood in the lane long enough: go round them
        if bypass and self._go_round(sw, self._bypass_why):
            return
        in_traffic = self._in_traffic(now)
        if phase == 'follow' and self._bypass_due(sw) and self._go_round(sw, self._stand_why(sw)):
            return
        # ---- v10.4: ended while people or vehicles were in the way: not a
        # failure of the robot. Same goal again shortly (bounded by
        # TRAFFIC_MAX_RETRIES here and by TRAFFIC_PATIENCE in tick_marker).
        if in_traffic and self._traffic_retries < self.TRAFFIC_MAX_RETRIES:
            self._traffic_retries += 1
            self.goal_deadline = None
            cause = self._traffic_cause(self._traffic) or 'it has only just cleared'
            self.get_logger().warn(
                f'wp{self.gi + 1} ended status={status} in traffic ({cause}) -> '
                f'not a failure: same goal again in {self.TRAFFIC_RETRY_GAP:.0f} s '
                f'({self._traffic_retries}/{self.TRAFFIC_MAX_RETRIES})')
            self.send_next(delay=self.TRAFFIC_RETRY_GAP)
            return
        # ---- v10.5: a lane the robot cannot get further along (a stall, a
        # timeout without progress, or Nav2 giving up again and again in
        # traffic): go round whatever is in it before the retry/rescue ladder
        if phase == 'follow' and sw['bypasses'] < self.BYPASS_MAX and not self._vehicle_near():
            here = self._map_pose()
            moved = (self._lane_pos(sw, here)[0] - sw['s0']) if here is not None else 0.0
            if in_traffic or moved < 0.3:
                why = (f'Nav2 gave up {self._traffic_retries} times in traffic' if in_traffic
                       else f'not getting along the lane (status={status})')
                if self._go_round(sw, why):
                    return
        # ---- failure ladder: retry -> RESCUE + retry -> put off (v10.4) ----
        if self._retries == 0:
            self._retries = 1
            self.get_logger().warn(
                f'wp{self.gi + 1} ended status={status} (6=ABORTED,5=CANCELED)'
                f' -> retry 1')
            self.goal_deadline = None
            self.send_next()                 # resend SAME goal
            return
        if self._retries == 1:
            self._retries = 2
            self.goal_deadline = None
            # Only rescue if the robot actually TRIED and failed to move. A
            # goal that aborted in under 3 s never drove anywhere, so there is
            # nothing to reverse off -- reversing and clearing the costmap
            # there just creates the next failure.
            ran = (self.get_clock().now().nanoseconds / 1e9) - self._goal_started
            if ran < 3.0:
                self.get_logger().warn(
                    f'wp{self.gi + 1} failed twice in {ran:.1f}s (never moved) '
                    f'-> straight retry, no rescue')
                self.send_next()
                return
            self.get_logger().warn(
                f'wp{self.gi + 1} failed twice -> AUTO-RESCUE, then final try')
            self.rescue(then=self.send_next)   # async; resumes when done
            return
        # third strike: log WHERE we are and WHY, then put it off till the end
        # (v10.4; a revisit, the dock or the start is skipped for good)
        self._consec_fails += 1
        self._retries = 0
        self._defer_current('failed after the rescue')
        if self._consec_fails >= 3:
            self._consec_fails = 0
            self.get_logger().error(
                'CASCADE: 3 consecutive waypoints skipped -> rescuing, then '
                'continuing (robot is likely wedged).')
            # MUST wait for the rescue to finish. The previous version called
            # advance() one millisecond after starting it, so the rescue drove
            # the robot backwards while Nav2 drove it forwards, and the costmap
            # clear landed in the middle of an active goal.
            self.rescue(then=self.advance)
            return
        self.advance()

    def _nav2_lost(self, why):
        """v10.6: Nav2 is down: hold at this waypoint (skip nothing), try again every NAV2_RETRY s."""
        if not self._nav2_down:
            self._nav2_down = True
            self.nav2_downs += 1
            self.get_logger().error(
                f'NAV2: {why} -> holding at wp{self.gi + 1}, trying again every '
                f'{self.NAV2_RETRY:.0f} s; nothing is skipped. If it does not come back: stack.sh stop, '
                f'then start again')
        self.goal_deadline = None
        self.send_next(delay=self.NAV2_RETRY)

    def _tick_nav2(self):
        """v10.6: a goal out to a Nav2 that is gone (crashed) never answers, and nor does its
        cancel (v10.5: STALL, cancel, STALL... for ever). 3 s with no server: drop it and hold."""
        if self._goal_handle is None or self._done or not self._nav2_gone():
            self._nav2_gone_since = None
            return
        now = self._now()
        if self._nav2_gone_since is None:
            self._nav2_gone_since = now
            return
        if now - self._nav2_gone_since < 3.0:
            return
        self._nav2_gone_since = None
        self._send_id += 1                          # anything late from it is stale
        self._goal_handle = None
        self._lane_path = None
        self._defer_pending = self._bypass_pending = self._detour_pending = False
        self._veh_skip_pending = None
        self._nav2_lost('the Nav2 server went away while the robot was driving (it stopped or crashed)')

    def _nav2_gone(self):
        """v10.6: no server for the action the current goal went to."""
        try:
            if not self._c.server_is_ready():
                return True
            return (self._lane_path is not None and self._fp is not None
                    and not self._fp.server_is_ready())
        except Exception:
            return False

    def _pose_state_str(self):
        """Robot's live map pose + whether the planner's own free-map thinks
        that spot is inside a keep-out. THE diagnostic line for any failure."""
        try:
            t = self.tf_buffer.lookup_transform('map', 'base_link',
                                                rclpy.time.Time())
            mx = t.transform.translation.x
            my = t.transform.translation.y
            free = self.is_free_world(mx + self.spawn_x, my + self.spawn_y)
            return (f'[robot map pose ({mx:.2f},{my:.2f}) | '
                    f'inside keep-out: {not free}]')
        except Exception:
            return '[robot pose unavailable]'

    # ============================================================= #
    #  TRAFFIC (v10.4) -- the people guard's view, so that waiting for
    #  people and vehicles is never mistaken for a wedged robot
    # ============================================================= #
    def _now(self):
        return self.get_clock().now().nanoseconds / 1e9

    def on_guard(self, msg):
        """/people_guard/state: the guard's decision after every scan (JSON)."""
        try:
            s = json.loads(msg.data)
        except Exception:
            return
        if not isinstance(s, dict):
            return
        now = self._now()
        self._traffic, self._traffic_t = s, now
        self._guard_seen = True                 # v10.6: from now on, silence means trouble
        if self._traffic_cause(s) is not None:
            self._traffic_last = now
        waiting = s.get('state') == 'wait'
        active = not self._done and not self._paused
        if waiting and not self._guard_waiting and active:
            self.holds += 1
            self._hold_started = now
            self.last_hold = [str(s.get('say') or 'waiting'), now, None]
        elif not waiting and self._guard_waiting and self._hold_started is not None:
            self.held_s += max(0.0, now - self._hold_started)
            self._hold_started = None
            if self.last_hold is not None:
                self.last_hold[2] = now
        if waiting and self.last_hold is not None and self._hold_started is not None:
            self.last_hold[0] = str(s.get('say') or self.last_hold[0])
        self._guard_waiting = waiting

    def _traffic_cause(self, s):
        """Why traffic may be holding the robot (a phrase), or None."""
        if s is None:
            return None
        if s.get('state') == 'wait':
            return str(s.get('say') or 'the people guard is holding the robot')
        pm, vm = s.get('person_m'), s.get('vehicle_m')
        if isinstance(pm, (int, float)) and pm < self.TRAFFIC_NEAR_PERSON:
            return f'a person {pm:.1f} m away'
        if isinstance(vm, (int, float)) and vm < self.TRAFFIC_NEAR_VEHICLE:
            return f'a vehicle {vm:.1f} m away'
        return None

    def _guard_fresh(self, now):
        return self._traffic_t is not None and now - self._traffic_t <= self.TRAFFIC_STALE

    def _in_traffic(self, now):
        """Traffic now, or within TRAFFIC_LINGER s (Nav2 needs a moment to
        replan once the way clears). False with no guard: old behaviour."""
        return self._guard_fresh(now) and now - self._traffic_last <= self.TRAFFIC_LINGER

    def _someone_near(self, now, r_person):
        """Is anyone too close to reverse blind (or the guard holding)?"""
        if not self._guard_fresh(now):
            return False
        s = self._traffic
        if s.get('state') == 'wait':
            return True
        pm, vm = s.get('person_m'), s.get('vehicle_m')
        return ((isinstance(pm, (int, float)) and pm < r_person)
                or (isinstance(vm, (int, float)) and vm < r_person + 0.5))

    def _is_revisit(self, i=None):
        """A second go at floor (revisit rv/RV, v10.5 patch pt): it is
        skipped for good if it fails, never put off again."""
        i = self.gi if i is None else i
        return (0 <= i < len(self.goals)
                and str(self.goals[i][2]).lower().startswith(('rv', 'pt')))

    def _defer_current(self, why):
        """Put the current waypoint off till before going home (once). A
        revisit, the dock or the start is skipped instead. True if deferred."""
        if not (0 <= self.gi < len(self.goals)):
            return False
        x, y, lbl = self.goals[self.gi]
        z = self.goal_zones[self.gi] if self.gi < len(self.goal_zones) else ''
        if self._is_revisit() or z in ('home', 'start'):
            self.skipped.append({'zone': z, 'label': lbl, 'x': round(x, 2),
                                 'y': round(y, 2), 'why': why})
            self.get_logger().error(
                f'SKIP wp{self.gi + 1} ({lbl}, {z}) for good: {why}. '
                + self._pose_state_str())
            return False
        p0 = self._goal_from if self._goal_from is not None else (x, y)
        self.deferred.append({'zone': z, 'from': (float(p0[0]), float(p0[1])),
                              'to': (float(x), float(y)), 'label': lbl, 'why': why})
        self.get_logger().warn(
            f'DEFER wp{self.gi + 1} ({lbl}, {z}): {why} -> coming back for it '
            f'before going home ({len(self.deferred)} put off). '
            + self._pose_state_str())
        return True

    def _credit(self, z):
        """A waypoint of zone z counts as done (as a success in goal_done)."""
        if z in ('start', 'home', '') or z is None:
            return
        self._zone_ok.add(z)
        self._zone_done[z] = self._zone_done.get(z, 0) + 1
        tot = self._zone_total.get(z, 0)
        if tot and self._aisle_geom(z) is None:   # v10.5: aisles: painted lanes (_update_swept)
            pct = min(100, int(100 * self._zone_done[z] / tot))
            if pct > self.zone_pct.get(z, 0):
                self.zone_pct[z] = pct

    def _insert_revisits(self):
        """Before going home: drive the still-dirty stretches of every waypoint
        that was put off. The painted grid decides what is still dirty, so floor
        swept since (or on the way) is not driven again."""
        todo, self.deferred = self.deferred, []
        new_g, new_z, n = [], [], 0
        for d in todo:
            runs = segment_runs(d['from'], d['to'], self.visited, cell=self.cov_cell)
            if not runs:
                self._credit(d['zone'])
                self.get_logger().info(
                    f"REVISIT {d['label']} ({d['zone']}): swept since, nothing left")
                continue
            n += 1
            for k, (a, b) in enumerate(runs):
                ga = (round(a[0], 3), round(a[1], 3), f'rv{n}a')
                gb = (round(b[0], 3), round(b[1], 3),
                      f'RV{n}' if k == len(runs) - 1 else f'rv{n}b')
                new_g += [ga, gb]
                new_z += [d['zone'], d['zone']]
                box = self._aisle_box(ga[0])
                if (box is not None and abs(gb[0] - ga[0]) < 0.05 and box[0] <= ga[0] <= box[1]
                        and math.hypot(gb[0] - ga[0], gb[1] - ga[1]) >= 0.5):
                    self.sweeps[gb] = ga[:2]        # v10.5: a lane stretch: driven along it
        if new_g:
            self.goals[self.gi:self.gi] = new_g
            self.goal_zones[self.gi:self.gi] = new_z
            self.revisited += n
            self.get_logger().warn(
                f'REVISIT: {n} stretch(es) put off earlier are still dirty -> '
                f'{len(new_g)} goals before going home')

    def rescue(self, then=None):
        """Reverse off whatever we are wedged against, then clear the LOCAL
        costmap. Runs on a timer -- it does NOT block the executor.

        The blocking version was the single worst bug in this file. It did:

            cli.call_async(ClearEntireCostmap.Request())   # needs the executor
            time.sleep(1.0)                                # blocks the executor

        call_async only transmits while the executor spins, so the clear did
        not happen during the sleep -- it landed immediately AFTER the callback
        returned, which is exactly when the next goal was sent. The global
        costmap was therefore wiped at the moment the planner needed it, and
        with track_unknown_space: true an empty costmap makes every plan fail
        instantly. One rescue then caused the next goal to abort, which caused
        another rescue: a self-feeding cascade of ~150 ms aborts.

        The global costmap is also no longer cleared at all. It is the static
        map plus the lidar layer; wiping it cannot free a wedged robot and only
        blinds the planner for a full update cycle. Clearing the LOCAL costmap
        is what actually clears a phantom obstacle.
        """
        # v10.4: never reverse blind with a person or vehicle close behind or
        # beside: clear the costmap only (reversing phase of zero length).
        reverse = not self._someone_near(self._now(), self.RESCUE_CLEAR)
        if reverse:
            self.get_logger().warn('RESCUE: reversing, then clearing the local '
                                   'costmap. ' + self._pose_state_str())
        else:
            self.get_logger().warn(
                'RESCUE without reversing (someone within '
                f'{self.RESCUE_CLEAR:.1f} m, or the guard is holding): clearing the '
                'local costmap only. ' + self._pose_state_str())
        self._rescue_then = then
        self._rescue_phase = 'reversing'
        self._rescue_until = self._now() + (2.5 if reverse else 0.0)

    def tick_fast(self):
        """10 Hz: runs the rescue machine and (v10.7) a manoeuvre, then the paced goal dispatch."""
        self.tick_rescue()
        self.tick_manoeuvre()
        if not self._send_pending or self._rescue_phase is not None or self._man is not None:
            return                      # never dispatch mid-rescue or mid-manoeuvre
        if self.get_clock().now().nanoseconds / 1e9 < self._send_at:
            return
        self._send_pending = False
        self._dispatch()

    def tick_rescue(self):
        """Drives the rescue without ever blocking. 10 Hz."""
        if self._rescue_phase is None:
            return
        now = self.get_clock().now().nanoseconds / 1e9

        if self._rescue_phase == 'reversing':
            if now < self._rescue_until:
                if self._someone_near(now, self.RESCUE_CLEAR):
                    self._rescue_until = now      # v10.4: someone came close: stop reversing
                else:
                    tw = Twist()
                    tw.linear.x = -0.15
                    self.cmd_pub.publish(tw)
                    return
            self.cmd_pub.publish(Twist())               # full stop
            try:
                if self.clear_local.service_is_ready():
                    self.clear_local.call_async(ClearEntireCostmap.Request())
                    self.get_logger().info('  cleared local costmap')
                else:
                    self.get_logger().warn('  local clear service not ready')
            except Exception as e:
                self.get_logger().warn(f'  local costmap clear failed: {e}')
            self._rescue_phase = 'settling'
            self._rescue_until = now + 1.2       # executor free -> clear lands
            return

        if self._rescue_phase == 'settling':
            if now < self._rescue_until:
                return
            self._rescue_phase = None
            cb, self._rescue_then = self._rescue_then, None
            if cb is not None:
                cb()

    # ============================================================= #
    #  v10.7: TURNING ON THE SPOT AND GETTING UNSTUCK (see the header)
    #  Driven on /cmd_vel like the rescue; stack.sh remaps it to /cmd_vel_raw, so it goes through
    #  guard_gate (the guard alive, the emergency stop) and the Collision Monitor (the guard's
    #  hold stops it; its Brake zone keeps it 1 s from anything the lidar sees; turning on the
    #  spot is never limited there). Only with the guard running (that chain is there).
    # ============================================================= #
    @staticmethod
    def _wrap(a):
        return (a + math.pi) % (2.0 * math.pi) - math.pi

    def _map_pose_yaw(self):
        try:
            t = self.tf_buffer.lookup_transform('map', 'base_link', rclpy.time.Time())
        except Exception:
            return None
        q = t.transform.rotation
        yaw = math.atan2(2.0 * (q.w * q.z + q.x * q.y), 1.0 - 2.0 * (q.y * q.y + q.z * q.z))
        return t.transform.translation.x, t.transform.translation.y, yaw

    def _static_near(self, x, y, yaw, r=0.95):
        """(nearest rack/wall cell of the static map in front: up to WEDGE_FRONT ahead and within
        WEDGE_HALF of the heading line; nearest one anywhere), m from the robot's centre, inf if
        none within r; None without a map."""
        m = self._map
        if m is None:
            return None
        res = m.info.resolution
        ox, oy = m.info.origin.position.x, m.info.origin.position.y
        w, h = m.info.width, m.info.height
        gx0, gy0 = int((x - ox) / res), int((y - oy) / res)
        k = int(math.ceil(r / res))
        c, s = math.cos(yaw), math.sin(yaw)
        front = anyd = float('inf')
        data = m.data
        for gy in range(max(0, gy0 - k), min(h, gy0 + k + 1)):
            row = gy * w
            cy = oy + (gy + 0.5) * res - y
            for gx in range(max(0, gx0 - k), min(w, gx0 + k + 1)):
                if data[row + gx] < 65:             # free (0) or unknown (-1)
                    continue
                cx = ox + (gx + 0.5) * res - x
                d = math.hypot(cx, cy)
                if d > r:
                    continue
                anyd = min(anyd, d)
                ahead, side = cx * c + cy * s, -cx * s + cy * c
                if d < front and 0.0 < ahead <= self.WEDGE_FRONT and abs(side) <= self.WEDGE_HALF:
                    front = d
        return front, anyd

    def _go_clear(self, now):
        """The guard says GO and nobody is close: standing now is not traffic."""
        if not self._guard_fresh(now):
            return False
        g = self._traffic
        if g.get('state') == 'wait':
            return False
        pm, vm = g.get('person_m'), g.get('vehicle_m')
        return not ((isinstance(pm, (int, float)) and pm < self.WEDGE_PERSON)
                    or (isinstance(vm, (int, float)) and vm < self.WEDGE_VEHICLE))

    def _held_now(self, now):
        return self._guard_fresh(now) and self._traffic.get('state') == 'wait'

    def _person_within(self, now, r):
        """A person within r (a vehicle within r + 0.5) of the robot's centre, by the guard."""
        if not self._guard_fresh(now):
            return False
        pm, vm = self._traffic.get('person_m'), self._traffic.get('vehicle_m')
        return ((isinstance(pm, (int, float)) and pm < r)
                or (isinstance(vm, (int, float)) and vm < r + 0.5))

    def _wedged_now(self):
        """(front, anywhere) distances if the robot is wedged against a rack or wall: the guard
        says GO, nobody is near, and the static map has something right in front (or touching
        close anywhere). None otherwise (traffic, no guard, no map, nothing there)."""
        if not self._guard_seen or not self._go_clear(self._now()):
            return None
        pose = self._map_pose_yaw()
        if pose is None:
            return None
        near = self._static_near(*pose)
        if near is None:
            return None
        return near if (near[0] <= self.WEDGE_FRONT or near[1] <= self.WEDGE_SIDE) else None

    def _unsticks_left(self):
        if self._unsticks_gi != self.gi:
            self._unsticks_gi, self._unsticks_wp = self.gi, 0
        return self._unsticks_wp < self.UNSTICK_MAX

    def _scan_robot_pts(self):
        """The last fresh /scan_nav as points in the robot's frame (x ahead, y left), or None."""
        msg = self._scan
        if msg is None or self._now() - self._scan_t > self.SCAN_STALE:
            return None
        pose = self._map_pose_yaw()
        if pose is None:
            return None
        try:
            tf = self.tf_buffer.lookup_transform('map', msg.header.frame_id, rclpy.time.Time())
        except Exception:
            return None
        x, y, yaw = pose
        lx, ly = tf.transform.translation.x, tf.transform.translation.y
        q = tf.transform.rotation
        lyaw = math.atan2(2.0 * (q.w * q.z + q.x * q.y), 1.0 - 2.0 * (q.y * q.y + q.z * q.z))
        c, s = math.cos(-yaw), math.sin(-yaw)
        pts = []
        a0, inc = msg.angle_min + lyaw, msg.angle_increment
        for k, r in enumerate(msg.ranges):
            if not (msg.range_min < r < msg.range_max):
                continue
            ang = a0 + k * inc
            dx, dy = lx + r * math.cos(ang) - x, ly + r * math.sin(ang) - y
            pts.append((dx * c - dy * s, dx * s + dy * c))
        return pts

    def _rear_clear(self):
        """Nothing in /scan_nav within BACK_CLEAR behind the rear edge, in the robot's width."""
        pts = self._scan_robot_pts()
        if pts is None:
            return False
        return not any(-(0.30 + self.BACK_CLEAR) <= px <= -0.25 and abs(py) <= 0.27 for px, py in pts)

    def _turn_clear(self):
        """Nothing in /scan_nav within TURN_CLEAR of the centre (a turn sweeps 0.36 m)."""
        pts = self._scan_robot_pts()
        if pts is None:
            return False
        return not any(math.hypot(px, py) < self.TURN_CLEAR for px, py in pts)

    def _path_heading(self, x, y):
        """Where the robot should face to get going: along the lane being driven; else along
        Nav2's last path (0.6 m on); else at the goal."""
        sw = self._cur_sweep()
        if sw is not None and sw['phase'] == 'follow' and sw['target'] is None:
            return math.atan2(sw['uy'], sw['ux'])
        lp = self._last_plan
        if lp is not None and self._now() - lp[0] <= 5.0 and lp[1]:
            pts = lp[1]
            i0 = min(range(len(pts)), key=lambda i: (pts[i][0] - x) ** 2 + (pts[i][1] - y) ** 2)
            for px, py in pts[i0:]:
                if math.hypot(px - x, py - y) >= 0.6:
                    return math.atan2(py - y, px - x)
        if 0 <= self.gi < len(self.goals):
            gx, gy, _ = self.goals[self.gi]
            if math.hypot(gx - x, gy - y) > 0.05:
                return math.atan2(gy - y, gx - x)
        return None

    def _align_first(self, sw, lbl):
        """ALIGN: before driving a lane, next to a rack or wall and facing more than ALIGN_MAX off
        it, turn on the spot first (RPP would drive forward on an arc). True if it started."""
        if not self._guard_seen or sw.get('aligns', 0) >= 2:
            return False
        pose = self._map_pose_yaw()
        if pose is None:
            return False
        want = math.atan2(sw['uy'], sw['ux'])
        err = self._wrap(want - pose[2])
        if abs(err) <= self.ALIGN_MAX:
            return False
        near = self._static_near(*pose)
        if near is None or near[1] > self.ALIGN_NEAR:   # no map (cannot tell) or open floor: RPP turns
            return False
        anyd = near[1]
        sw['aligns'] = sw.get('aligns', 0) + 1
        self.aligns += 1
        self.get_logger().info(
            f'ALIGN: {lbl}: facing {math.degrees(abs(err)):.0f} deg off the lane with a rack or wall '
            f'{anyd:.2f} m away -> turning on the spot before driving it')
        self._man_start('align', want, back=False, then=self.send_next, sw=sw)
        return True

    def _start_unstick(self, why):
        """UNSTICK: back off what is in front (if the way behind is clear), turn to the path,
        then the same goal again."""
        pose = self._map_pose_yaw()
        if pose is None:
            self.send_next()
            return
        near = self._static_near(*pose) or (float('inf'), float('inf'))
        back = near[0] <= self.BACK_IF
        want = self._path_heading(pose[0], pose[1])
        self._unsticks_left()
        self._unsticks_wp += 1
        self.unsticks += 1
        self._wedge_s = 0.0
        sw = self._cur_sweep()
        if sw is not None:
            sw.update(stand=0.0, stand_go=0.0, ref_s=None)
        self.get_logger().warn(
            f'UNSTICK wp{self.gi + 1}: {why} -> '
            + (f'backing off {self.BACK_DIST:.2f} m, ' if back else '')
            + ('turning to face ' + ('the lane' if (sw is not None and sw['phase'] == 'follow')
                                    else 'the way on') + ', ' if want is not None else '')
            + f'then the same goal again ({self._unsticks_wp}/{self.UNSTICK_MAX}). '
            + self._pose_state_str())
        self._man_start('unstick', want, back=back, then=self.send_next)

    def _man_start(self, kind, yaw_to, back, then, sw=None):
        now = self._now()
        self._man = dict(kind=kind, phase='settle', yaw_to=yaw_to, back=back, then=then,
                         t0=now, last=now, used=0.0, phase_used=0.0, start=None, note=[], sw=sw)

    def _man_abort(self, why):
        """Stop a manoeuvre now (a command, a pause): zero velocity, nothing called after."""
        if self._man is not None:
            self.cmd_pub.publish(Twist())
            self.get_logger().info(f'{self._man["kind"].upper()}: stopped ({why})')
            self._man = None

    def _man_end(self, note=None):
        m, self._man = self._man, None
        self.cmd_pub.publish(Twist())
        if m is None:
            return
        if note:
            m['note'].append(note)
        if m['note']:
            self.get_logger().info(f'{m["kind"].upper()}: ' + '; '.join(m['note']))
        if m['then'] is not None:
            m['then']()

    def tick_manoeuvre(self):
        """10 Hz: settle (the smoother's last zeros), back off, turn, stop. Time held by the
        guard does not count (the Collision Monitor stops the robot meanwhile)."""
        m = self._man
        if m is None:
            return
        now = self._now()
        dt = min(max(now - m['last'], 0.0), 0.5)
        m['last'] = now
        if self._done or self._paused or self._safety_hold:
            self._man_abort('the job stopped' if not self._safety_hold else 'safety hold')
            return
        if now - m['t0'] > self.MAN_MAX:
            self._man_end(f'gave up after {self.MAN_MAX:.0f} s (held most of it)')
            return
        if self._held_now(now):
            # the guard is holding (the Collision Monitor zeroes whatever we send): wait, in place,
            # and do not count it; every check below is made again once it lets go
            if m['phase'] in ('back', 'turn'):
                self.cmd_pub.publish(Twist())
            return
        m['used'] += dt
        m['phase_used'] += dt
        if m['used'] > self.MAN_BUDGET:
            self._man_end(f'took over {self.MAN_BUDGET:.0f} s')
            return
        pose = self._map_pose_yaw()
        if pose is None:
            self._man_end('no pose')
            return
        x, y, yaw = pose
        if m['phase'] == 'settle':
            if now < self._last_result_at + self.MAN_SETTLE or now < m['t0'] + 0.3:
                return
            m['phase'], m['phase_used'] = ('back' if m['back'] else 'turn'), 0.0
            if m['phase'] == 'back':
                if self._person_within(now, self.RESCUE_CLEAR):
                    # as the rescue: never back up with a person within RESCUE_CLEAR (crossing people
                    # are not in /scan_nav, and the guard predicts with the path, not this move)
                    m['note'].append(f'not backing off: someone within {self.RESCUE_CLEAR:.1f} m')
                    m['phase'] = 'turn'
                elif self._rear_clear():
                    m['start'] = (x, y)
                else:
                    m['note'].append('not backing off: something behind')
                    m['phase'] = 'turn'
        if m['phase'] == 'back':
            sx, sy = m['start']
            gone = math.hypot(x - sx, y - sy)
            behind = (gone < self.BACK_DIST and m['phase_used'] <= 4.0
                      and (not self._rear_clear() or self._person_within(now, self.RESCUE_CLEAR)))
            if gone >= self.BACK_DIST or m['phase_used'] > 4.0 or behind:
                self.cmd_pub.publish(Twist())
                m['note'].append(f'backed off {gone:.2f} m'
                                 + (' (stopped: something behind, or someone close)' if behind else ''))
                m['phase'], m['phase_used'] = 'turn', 0.0
            else:
                tw = Twist()
                tw.linear.x = -self.BACK_SPEED
                self.cmd_pub.publish(tw)
                return
        if m['phase'] == 'turn':
            if m['yaw_to'] is None:
                m['phase'] = 'stop'
            else:
                err = self._wrap(m['yaw_to'] - yaw)
                if abs(err) <= self.ALIGN_TOL or m['phase_used'] > 8.0:
                    if abs(err) > self.ALIGN_TOL:
                        m['note'].append(f'turned, still {math.degrees(abs(err)):.0f} deg off')
                    else:
                        m['note'].append('lined up')
                    m['phase'] = 'stop'
                elif not m.get('turn_ok'):
                    if self._turn_clear():
                        m['turn_ok'] = True
                    else:
                        m['note'].append('not turning: something within '
                                         f'{self.TURN_CLEAR:.2f} m')
                        m['phase'] = 'stop'
                        if m['sw'] is not None:
                            m['sw']['aligns'] = 99     # no second try at this lane: RPP drives it
                if m['phase'] == 'turn':
                    w = min(self.TURN_RATE, max(self.TURN_MIN, 1.5 * abs(err)))
                    tw = Twist()
                    tw.angular.z = math.copysign(w, err)
                    self.cmd_pub.publish(tw)
                    return
        if m['phase'] == 'stop':
            self.cmd_pub.publish(Twist())
            m['phase'], m['stop_at'] = 'stopped', now
            return
        if m['phase'] == 'stopped' and now - m['stop_at'] >= 0.3:
            self._man_end()

    def advance(self):
        self.gi += 1
        self._no_answer = 0                 # v10.6
        self.goal_deadline = None
        self._retries = 0
        self._sweep = None                  # v10.5: the lane (if any) is over
        self._lane_path = None
        self._bypass_pending = self._detour_pending = False
        self._ahead_hold = False            # v10.6
        self._veh_skip_pending = None
        if not self._done and not self._paused:
            self._before_home()
        self._reset_waypoint_traffic()
        self.send_next()

    def _reset_waypoint_traffic(self):
        """v10.4: per-waypoint traffic bookkeeping starts again."""
        self._goal_from_gi = None
        self._hard_deadline = None
        self._blocked_s = 0.0
        self._block_ref = None
        self._standing_s = 0.0
        self._stand_logged = False
        self._traffic_retries = 0
        self._defer_pending = False

    def _lanes_reset(self, patched=False):
        """v10.5: lane, patch and bypass bookkeeping start again (a new mission
        or going home; patched=True: no patch pass)."""
        self.sweeps = {}
        self._sweep = None
        self._lane_path = None
        self._bypass_pending = self._detour_pending = False
        self._patched = patched
        self._patch_rounds = self.PATCH_ROUNDS if patched else 0      # v10.6
        self.patches = 0
        self.bypasses = 0
        self.swept_pct = {}
        self._swept_t = -1e9
        self._ahead_hold = False                  # v10.6
        self._veh_skip_pending = None
        self._veh_hold_s = 0.0
        self._enter_hint = {}
        self.vehicle_skips = self.clearance_stops = 0
        self.unsticks = self.aligns = 0           # v10.7

    def watchdog(self):
        """Cancel a goal that takes too long. IMPORTANT: cancel ONLY -- the
        CANCELED result then arrives in goal_done, which runs the normal
        failure ladder (retry -> rescue -> skip). The old version also called
        advance() here, so the late CANCELED callback advanced AGAIN -- that
        race skipped waypoints silently and printed COVERAGE COMPLETE twice."""
        if self._done or self._paused or self._rescue_phase or self._man is not None:
            return
        now = self.get_clock().now().nanoseconds / 1e9
        # v10.4: however much traffic, no waypoint takes more than
        # TRAFFIC_HARD_FACTOR x its normal deadline: then it is put off
        if (self._hard_deadline is not None and now > self._hard_deadline
                and self._goal_handle is not None and not self._defer_pending):
            self._hard_deadline = None
            self._defer_pending = True
            self._defer_why = (f'took {self.TRAFFIC_HARD_FACTOR:.0f}x its normal time '
                               '(traffic most of it)')
            self.goal_deadline = None
            self.get_logger().warn(f'wp{self.gi + 1}: {self._defer_why} -> putting it off')
            try:
                self._goal_handle.cancel_goal_async()
            except Exception:
                pass
            return
        if self.goal_deadline is None:
            return
        if now > self.goal_deadline:
            self.goal_deadline = None
            if self._nav2_gone():
                # v10.6: Nav2 itself is gone: its goal will never answer (nor a cancel)
                self._send_id += 1                  # anything late from it is stale
                self._goal_handle = None
                self._lane_path = None
                self._nav2_lost('no answer, and no Nav2 server (it stopped or crashed)')
                return
            if self._goal_handle is not None:
                self.get_logger().warn('TIMEOUT -> canceling this goal')
                try:
                    self._goal_handle.cancel_goal_async()
                except Exception:
                    self._goal_handle = None
                    self.advance()
            elif self._no_answer < 1:
                # v10.6: never answered (lost on the way, or Nav2 went and came back): that is
                # not this waypoint's fault: send it once more before moving on
                self._no_answer += 1
                self.get_logger().warn('TIMEOUT: Nav2 never answered this goal -> sending it again')
                self.send_next()
            else:
                # goal never got accepted; nothing to cancel -> move on
                self._no_answer = 0
                self.get_logger().warn('TIMEOUT (no active goal) -> advancing')
                self.advance()

    # ============================================================= #
    #  CLEANED-AREA MARKER -- grid of swept cells (CUBE_LIST)
    # ============================================================= #
    def _load_cleaned(self):
        try:
            with open(self.CLEANED_FILE) as f:
                v = json.load(f)
            return [z for z in v if z in ('front', 'aisle_1', 'aisle_2',
                                          'aisle_3', 'back')]
        except Exception:
            return []

    def _load_pct(self):
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
        return {}

    def _save_cleaned(self):
        try:
            os.makedirs(os.path.dirname(self.CLEANED_FILE), exist_ok=True)
            with open(self.CLEANED_FILE, 'w') as f:
                json.dump(self.cleaned, f)
        except Exception as e:
            self.get_logger().warn(f'could not save cleaned zones: {e}')

    def _expand(self, zones):
        """Resolve 'uncleaned' against what this robot remembers finishing."""
        out = []
        for z in zones:
            if z == 'uncleaned':
                todo = [k for k in self.ZONE_ORDER if k not in self.cleaned]
                if not todo:
                    self.get_logger().info(
                        'every zone is already cleaned this session')
                out.extend(k for k in todo if k not in out)
            elif z not in out:
                out.append(z)
        return out

    def on_command(self, msg):
        """A mission command arrives as JSON: {"action":..,"zones":[..]}.

        Re-validated here even though llm_commander already validated it. The
        robot is the last line, and it trusts nothing upstream of itself.
        """
        try:
            raw = json.loads(msg.data)
            if not isinstance(raw, dict):
                raise ValueError('not a JSON object')
            # a chained instruction ("stop and clean aisle 1") arrives as steps
            if isinstance(raw.get('steps'), list) and raw['steps']:
                for i, st in enumerate(raw['steps'], 1):
                    self.get_logger().info(
                        f'  chain step {i}/{len(raw["steps"])}')
                    m2 = String()
                    m2.data = json.dumps(st)
                    self.on_command(m2)
                return
            c = raw
            action = str(c.get('action', '')).lower()
            zones = c.get('zones', []) or []
            if not isinstance(zones, list):
                raise ValueError('zones is not a list')
            zones = self._expand([str(z).lower() for z in zones])
            if action not in self.ACTIONS:
                raise ValueError(f"unknown action '{action}'")
            bad = [z for z in zones if z not in self.ZONES]
            if bad:
                raise ValueError(f"unknown zone(s) {bad}")
            if action in ('clean', 'skip') and not zones:
                raise ValueError(f"'{action}' needs at least one zone")
        except Exception as e:
            self.get_logger().error(f'COMMAND REJECTED: {e} | raw={msg.data[:120]}')
            return

        reason = str(c.get('reason', ''))[:120]
        if action != 'noop':
            self.command_log.append(
                (f"{action} {', '.join(zones)}".strip()
                 + (f" [{reason}]" if reason else "")))
            self.command_log = self.command_log[-12:]
        self.get_logger().info(
            f"COMMAND: {action} {zones or ''} {('(' + reason + ')') if reason else ''}")

        if action == 'noop':
            return

        if action == 'home':
            self._epoch += 1
            self._man_abort('going home')          # v10.7
            self._awaiting = False
            self._cancel_current()
            self.goals = [(self.home[0], self.home[1], 'HOME')]
            self.goal_zones = ['home']
            self.gi = 0
            self._retries = 0
            self.deferred = []                   # v10.4: going home drops what was put off
            self._reset_waypoint_traffic()
            self._lanes_reset(patched=True)      # v10.5: and no patch pass on the way
            self._done = False
            self._paused = False
            self.get_logger().info('RETURNING TO DOCK')
            self.send_next()
            return

        if action == 'reset':
            self.cleaned = []
            self.zone_pct = {}
            self._zone_ok = set()
            self._save_cleaned()
            self.get_logger().warn('cleaned-zone memory cleared')
            return

        if action == 'stop':
            self._paused = True
            self._epoch += 1                       # orphan any in-flight goal
            self._man_abort('paused')              # v10.7: a turn or back-off stops at once
            self._cancel_current()
            self._reset_waypoint_traffic()   # v10.4: no hard deadline while paused
            self._lane_path = None           # v10.5: the lane (_sweep) is kept for resume
            self._bypass_pending = self._detour_pending = False
            self.goal_deadline = None        # a paused robot has no deadline;
            #   leaving it set let the watchdog fire mid-pause, log
            #   'TIMEOUT (no active goal) -> advancing' and step gi forward,
            #   so a resume silently skipped the rest of the lane.
            self.get_logger().warn('PAUSED. Send resume to continue.')
            return

        if action == 'resume':
            if not self._paused:
                self.get_logger().info('not paused; nothing to resume')
                return
            self._paused = False
            self._epoch += 1
            if self._sweep is not None:      # v10.5: waiting starts again
                self._sweep.update(stand=0.0, stand_go=0.0, ref_s=None, entered=0)
            self.get_logger().info(f'RESUMING at {self.gi + 1}/{len(self.goals)}')
            self.send_next()
            return

        if action == 'skip':
            drop = set(zones)
            for z in zones:
                if z in self.ZONE_ORDER and z not in self.blocked:
                    self.blocked.append(z)
            keep_g, keep_z = [], []
            removed = 0
            for i, (g, z) in enumerate(zip(self.goals, self.goal_zones)):
                if i > self.gi and z in drop:
                    removed += 1
                    continue
                keep_g.append(g)
                keep_z.append(z)
            self.goals, self.goal_zones = keep_g, keep_z
            self.deferred = [d for d in self.deferred if d['zone'] not in drop]   # v10.4
            if removed:
                self.get_logger().warn(
                    f'SKIP {sorted(drop)} -> dropped {removed} waypoints; '
                    f'{len(self.goals) - self.gi - 1} still to go')
            else:
                self.get_logger().info(
                    f'SKIP {sorted(drop)} -> nothing to drop; that zone is not '
                    f'in the current mission')
            return

        # ---- clean ----
        # a LIST, never set(): a set has no order, so "clean aisle 1 then
        # aisle 3" arrived here as {aisle_3, aisle_1} and aisle 3 ran first.
        # build_mission honoured order all along -- this call site discarded it.
        goals, zlist = self.build_mission(list(zones))
        # Any waypoint that is not start or home is real work. Testing the
        # requested tokens instead refused every 'all' / 'uncleaned' command,
        # because build_mission expands those aliases into real zone names.
        if not goals or not any(z not in ('start', 'home') for z in zlist):
            done = list(self._already_clean)
            if done:
                self.get_logger().info(
                    f"{', '.join(done)} already swept -> nothing to do. The "
                    f"current mission is untouched.")
            else:
                self.get_logger().error(
                    f'no waypoints for zones {zones}; ignoring')
            return
        goals.append((self.home[0], self.home[1], 'HOME'))
        zlist.append('home')

        # A zone-only mission still begins at spawn, so if the robot is already
        # standing there the first goal asks it to travel nowhere. Nav2 does not
        # complete that cleanly -- the robot sits still, the stall detector
        # fires after 10 s, and the cancel kicks off the whole failure ladder
        # before the mission has even started. Drop any leading goal the robot
        # has effectively already reached.
        here = self._map_pose()
        if here is not None:
            dropped = 0
            while len(goals) > 1 and math.hypot(goals[0][0] - here[0],
                                                goals[0][1] - here[1]) < 0.40:
                goals.pop(0)
                zlist.pop(0)
                dropped += 1
            if dropped:
                self.get_logger().info(
                    f'  (skipped {dropped} opening waypoint(s) the robot is '
                    f'already at)')

        self._epoch += 1                           # orphan the old mission
        self._man_abort('a new job')               # v10.7
        self._awaiting = False                     # v10.7: a job: from now on goals go out
        self._cancel_current()
        self.goals, self.goal_zones = goals, zlist
        self.gi = 0
        self._retries = 0
        self._consec_fails = 0
        self._zone_ok = set()
        # Per-mission waypoint totals, so progress can be expressed as a
        # fraction. This was never populated: every progress update hit
        # `if tot:` with tot == 0, so partial progress was never recorded and
        # the robot reported "I haven't cleaned anything" mid-aisle.
        self._zone_done = {}
        self._zone_total = {}
        for z in zlist:
            if z not in ('start', 'home'):
                self._zone_total[z] = self._zone_total.get(z, 0) + 1
        self._done = False
        self._paused = False
        self.goal_deadline = None
        self._stall_ref = None
        # v10.4: traffic bookkeeping is per mission
        self.deferred, self.skipped, self.revisited = [], [], 0
        self.holds, self.held_s, self.last_hold = 0, 0.0, None
        self._hold_started = None
        self._reset_waypoint_traffic()
        self._lanes_reset()                        # v10.5
        self.sweeps = dict(self._last_sweeps)      # this mission's lanes
        self.get_logger().info(
            f'NEW MISSION: {sorted(zones)} -> {len(self.goals)} waypoints, '
            f'{len(self.sweeps)} of them lanes driven along')
        self.send_next()

    def _map_pose(self):
        try:
            t = self.tf_buffer.lookup_transform('map', 'base_link',
                                                rclpy.time.Time())
            return t.transform.translation.x, t.transform.translation.y
        except Exception:
            return None

    def _cancel_current(self):
        if self._goal_handle is not None:
            try:
                self._goal_handle.cancel_goal_async()
            except Exception:
                pass
        self._goal_handle = None

    def publish_state(self, pct, x=None, y=None):
        """What the robot is doing right now, for an agent to reason over."""
        zone = (self.goal_zones[self.gi]
                if 0 <= self.gi < len(self.goal_zones) else 'done')
        idle = self._awaiting           # v10.7: no job yet: idle at the dock, nothing queued
        s = String()
        s.data = json.dumps({
            'goal': 0 if idle else min(self.gi + 1, len(self.goals)),
            'total': 0 if idle else len(self.goals),
            'zone': 'start' if idle else zone, 'coverage_pct': round(pct, 1),
            'paused': self._paused, 'done': self._done,
            'cleaned': self.cleaned,
            'queue': [] if idle else [z for z in dict.fromkeys(self.goal_zones[self.gi:])
                                      if z not in ('start', 'home')],
            'partial': {z: p for z, p in self.zone_pct.items()
                        if z not in self.cleaned and p},
            'rescuing': (self._rescue_phase is not None
                         or (self._man is not None and self._man['kind'] == 'unstick')),
            'blocked': self.blocked,
            'asked': self.command_log,
            'x': None if x is None else round(x, 2),
            'y': None if y is None else round(y, 2),
            **self._traffic_state(),
            **self._lane_state(x, y),
        })
        self.state_pub.publish(s)

    def _traffic_state(self):
        """v10.4: traffic for /mission_state (the robot agent explains stops
        from it). Times are seconds of robot (sim) time."""
        now = self._now()
        tr = None
        if self._guard_fresh(now):
            g = self._traffic
            tr = {'state': g.get('state'), 'why': g.get('why'), 'say': g.get('say'),
                  'v_limit': g.get('v_limit'), 'limit': g.get('limit'),
                  'person_m': g.get('person_m'), 'vehicle_m': g.get('vehicle_m'),
                  'for_s': (round(now - self._hold_started, 1)
                            if self._hold_started is not None else 0.0)}
        held = self.held_s + (now - self._hold_started if self._hold_started is not None else 0.0)
        last = None
        if self.last_hold is not None:
            say, t0, t1 = self.last_hold
            last = {'say': say, 'for_s': round((t1 if t1 is not None else now) - t0, 1),
                    'ago_s': 0.0 if t1 is None else round(now - t1, 1)}
        return {'traffic': tr, 'guard': tr is not None, 'holds': self.holds,
                'held_s': round(held, 1), 'last_hold': last,
                'waiting_s': round(self._standing_s, 1),
                'deferred': [{'zone': d['zone'], 'x': round(d['to'][0], 2),
                              'y': round(d['to'][1], 2), 'why': d['why']}
                             for d in self.deferred],
                'revisiting': self._is_revisit(), 'revisited': self.revisited,
                'skipped': [dict(s) for s in self.skipped]}

    def _lane_state(self, x=None, y=None):
        """v10.5: lanes for /mission_state: how much of each aisle's lanes is
        painted, the lane being driven, the patch pass, the blockages gone round."""
        lbl = str(self.goals[self.gi][2]) if 0 <= self.gi < len(self.goals) else ''
        sw = self._cur_sweep()
        lane = None
        if sw is not None:
            s = self._lane_pos(sw, (x, y))[0] if x is not None else None
            lane = {'phase': sw['phase'], 'going_round': sw['target'] is not None,
                    'left_m': None if s is None else round(sw['L'] - s, 1),
                    'stood_s': round(sw['stand'], 1), 'bypasses': sw['bypasses']}
        if lane is not None:
            lane['blocked_ahead'] = bool(self._ahead_hold)
            lane['clear_ahead_m'] = (None if self._ahead is None or self._ahead == float('inf')
                                     else round(self._ahead, 2))
        return {'swept': dict(self.swept_pct), 'clean_at': self.CLEAN_PCT,
                'patching': lbl.lower().startswith('pt'), 'patches': self.patches,
                'patch_round': self._patch_rounds,        # v10.6
                'bypasses': self.bypasses, 'lane': lane,
                # v10.6
                'safety_hold': self._safety_hold, 'vehicle_skips': self.vehicle_skips,
                'vehicle_held_s': round(self._veh_hold_s, 1),
                'clearance_stops': self.clearance_stops, 'sim_pauses': self.sim_pauses,
                'nav2_down': self._nav2_down,
                # v10.7
                'awaiting_command': self._awaiting, 'unsticks': self.unsticks,
                'aligns': self.aligns,
                'manoeuvre': None if self._man is None else self._man['kind']}

    def on_map(self, msg):
        """Occupancy grid from map_server. Builds TWO denominators:

          mapped floor    - every cell the map says is free.
          REACHABLE floor - the subset a 0.64 m robot can actually paint, i.e.
                            cells within one footprint radius of a pose where
                            the robot centre legally fits (>= robot_radius from
                            everything).

        The gap between them is pure geometry: floor tight against a wall or a
        rack face that no circular robot of this size can ever touch. Reporting
        against mapped floor makes a physically complete run look like a
        failure, so both are published and the reachable one is the headline.
        """
        self._map = msg
        cs = self.cov_cell
        inf = msg.info
        cx0 = int(math.floor(inf.origin.position.x / cs))
        cx1 = int(math.floor((inf.origin.position.x + inf.width * inf.resolution) / cs))
        cy0 = int(math.floor(inf.origin.position.y / cs))
        cy1 = int(math.floor((inf.origin.position.y + inf.height * inf.resolution) / cs))

        floor, navigable = set(), set()
        for cx in range(cx0, cx1 + 1):
            for cy in range(cy0, cy1 + 1):
                mx = cx * cs + cs / 2.0
                my = cy * cs + cs / 2.0
                if not self.map_is_floor(mx, my):
                    continue
                floor.add((cx, cy))
                if self._clear_disc(mx, my, self.ROBOT_RADIUS):
                    navigable.add((cx, cy))

        reach = int(math.ceil(self.foot_radius / cs))
        offs = [(dx, dy)
                for dx in range(-reach, reach + 1)
                for dy in range(-reach, reach + 1)
                if math.hypot(dx * cs, dy * cs) <= self.foot_radius]
        coverable = set()
        for (cx, cy) in navigable:
            for dx, dy in offs:
                c = (cx + dx, cy + dy)
                if c in floor:
                    coverable.add(c)

        self._map_floor = max(1, len(floor))
        self._map_coverable = max(1, len(coverable))
        self.get_logger().info(
            f'MAP received ({inf.width}x{inf.height} @ {inf.resolution} m). '
            f'Mapped floor {self._map_floor} cells; REACHABLE floor '
            f'{self._map_coverable} cells. The {self._map_floor - self._map_coverable} '
            f'cell difference is floor closer to a surface than the robot can '
            f'physically reach. Coverage is now measured against the real map.')

    def _clear_disc(self, mx, my, r):
        """True if the robot centre can legally sit at (mx,my): the map is free
        at the centre and at 12 points on the circle of radius r."""
        if not self.map_is_floor(mx, my):
            return False
        for k in range(12):
            a = 2.0 * math.pi * k / 12.0
            if not self.map_is_floor(mx + r * math.cos(a), my + r * math.sin(a)):
                return False
        return True

    def map_is_floor(self, mx, my):
        """True if (mx,my) in the MAP frame is known-free in the occupancy grid.
        `visited` cells are indexed in the map frame too, so no offset here."""
        m = self._map
        if m is None:
            return False
        gx = int((mx - m.info.origin.position.x) / m.info.resolution)
        gy = int((my - m.info.origin.position.y) / m.info.resolution)
        if 0 <= gx < m.info.width and 0 <= gy < m.info.height:
            return m.data[gy * m.info.width + gx] == 0
        return False

    def _tick_lane(self, x, y, now_s, dt, step):
        """v10.5, every tick: standing in the lane (-> go round), the lane on
        /plan once a second, and the aisles' painted lanes every 2 s."""
        sw = self._cur_sweep()
        if (sw is not None and sw['phase'] == 'follow' and not self._done
                and not self._paused and self._rescue_phase is None
                and self._man is None):             # v10.7: not while turning or backing off
            s_here = self._lane_pos(sw, (x, y))[0]
            if sw['ref_s'] is None or s_here - sw['ref_s'] >= self.TRAFFIC_PROGRESS:
                if sw['ref_s'] is not None:
                    self._traffic_retries = 0   # getting along: resends count again
                sw['ref_s'], sw['stand'], sw['stand_go'] = s_here, 0.0, 0.0
            elif dt > 0.0 and step < 0.05 * dt + 0.005:
                sw['stand'] += dt
                g = self._traffic if self._guard_fresh(now_s) else None
                if g is None or g.get('state') != 'wait':
                    sw['stand_go'] += dt        # not held: something stands in the lane
            if (self._goal_handle is not None and not self._bypass_pending
                    and not self._defer_pending and not self._detour_pending
                    and self._bypass_due(sw)):
                self._bypass_pending = True
                self._bypass_why = self._stand_why(sw)
                self.goal_deadline = None
                try:
                    self._goal_handle.cancel_goal_async()
                except Exception:
                    pass
            # v10.6: stop short of anything standing in the lane ahead
            self._ahead = self._clear_ahead(sw, (x, y))
            if (self._ahead is not None and self._ahead < self._stop_gap()
                    and self._goal_handle is not None and self._lane_path is not None
                    and not self._ahead_hold and not self._bypass_pending
                    and not self._defer_pending and not self._detour_pending
                    and self._veh_skip_pending is None):
                self._ahead_hold = True
                self.clearance_stops += 1
                lbl = sw['key'][2]
                self.get_logger().info(
                    f'CLEARANCE: {lbl}: something stands {self._ahead:.2f} m ahead in the lane -> '
                    f'stopping short of it')
                self.goal_deadline = None
                try:
                    self._goal_handle.cancel_goal_async()
                except Exception:
                    pass
            if self._lane_path is not None and now_s - self._plan_pub_t >= 1.0:
                self._publish_lane_plan()
        else:
            self._ahead = None
        self._tick_vehicle(x, y, now_s, dt, step)
        self._update_swept()

    def _tick_vehicle(self, x, y, now_s, dt, step):
        """v10.6: time held for a vehicle without a break; at a lane's end, or on the way to a
        lane's start, VEH_SKIP_AFTER of it skips that bit (not for patches or revisits: those
        are the last chance and keep the normal patience)."""
        g = self._traffic if self._guard_fresh(now_s) else None
        held = (g is not None and g.get('state') == 'wait' and g.get('why') == 'vehicle'
                and step < 0.05 * max(dt, 1e-3) + 0.005)
        if (not held or self._done or self._paused or self._rescue_phase is not None
                or self._man is not None):          # v10.7
            self._veh_hold_s = 0.0
            return
        self._veh_hold_s += dt
        if (self._veh_hold_s < self.VEH_SKIP_AFTER or self._veh_skip_pending is not None
                or self._bypass_pending or self._defer_pending or self._detour_pending
                or not (0 <= self.gi < len(self.goals))
                or str(self.goals[self.gi][2]).lower().startswith('rv')):
            return
        kind = None
        sw = self._cur_sweep()
        if str(self.goals[self.gi][2]).lower().startswith('pt'):
            # v10.6: in the patch pass: this stretch after the others (once; the revisit waits
            # with the normal patience)
            if sw is not None or self._lane_start_of_next() is not None:
                kind = 'later'
        elif sw is not None and sw['phase'] == 'follow':
            if sw['L'] - self._lane_pos(sw, (x, y))[0] <= self.VEH_SKIP_LEN:
                kind = 'end'
        elif sw is None and self._lane_start_of_next() is not None:
            kind = 'start'
        if kind is None:
            return
        if self._goal_handle is None:
            self._veh_skip(kind)                # between sends: act now
            return
        self._veh_skip_pending = kind
        self.goal_deadline = None
        try:
            self._goal_handle.cancel_goal_async()
        except Exception:
            pass

    def _tick_health(self):
        """v10.6, every tick: a frozen simulation (real time passing, sim time not) is logged
        when it resumes; the guard falling silent (once it was running) holds the robot still."""
        wall = time.monotonic()
        gap = 0.0 if self._wall_last is None else wall - self._wall_last
        self._wall_last = wall
        if gap > self.SIM_PAUSE_WALL:
            self.sim_pauses += 1
            self.sim_paused_s += gap
            self.get_logger().warn(
                f'SIM PAUSED: the simulation stood still for {gap:.0f} s of real time (Gazebo '
                f'paused, or the computer was busy or asleep). Not the robot: it carries on.')
        now = self._now()
        lost = (self._guard_seen and (self._traffic_t is None
                                      or now - self._traffic_t > self.GUARD_LOST))
        if (lost and not self._safety_hold and not self._done and not self._paused
                and not self._awaiting):        # v10.7: no job yet: nothing to hold (nor to resume)
            self._safety_hold = True
            self.get_logger().error(
                f'SAFETY: no word from the people guard for {self.GUARD_LOST:.0f} s (the guard or '
                f'the lidar has stopped) -> holding still until it is back')
            self.goal_deadline = None
            self._cancel_current()
        elif self._safety_hold and not lost:
            self._safety_hold = False
            self.get_logger().warn('SAFETY: the people guard is back -> carrying on')
            self._reset_waypoint_traffic()
            self.send_next()

    def tick_marker(self):
        self._tick_health()
        self._tick_nav2()
        try:
            t = self.tf_buffer.lookup_transform('map', 'base_link', rclpy.time.Time())
            x = t.transform.translation.x
            y = t.transform.translation.y
            _q = t.transform.rotation
            yaw = math.atan2(2.0 * (_q.w * _q.z + _q.x * _q.y),
                             1.0 - 2.0 * (_q.y * _q.y + _q.z * _q.z))
        except Exception:
            return
        # paint every grid cell within the robot footprint as "cleaned".
        # We paint the whole SEGMENT from the last pose to the current one, so
        # the band stays continuous even at speed or if a tick is missed.
        cs = self.cov_cell
        r = self.foot_radius
        if self._paint_last is None:
            self._paint_last = (x, y)

        # ---- STALL DETECTOR: a wedged robot holds its goal until the full
        # timeout expires. Watching displacement ends it in ~10 s instead. The
        # cancel lands in goal_done, which runs the normal retry/rescue ladder.
        now_s = self.get_clock().now().nanoseconds / 1e9
        dt = 0.0 if self._tick_t is None else min(max(now_s - self._tick_t, 0.0), 1.0)
        step = 0.0 if self._tick_xy is None else math.hypot(x - self._tick_xy[0],
                                                            y - self._tick_xy[1])
        self._tick_t, self._tick_xy = now_s, (x, y)
        if dt > 0.0:
            self._speed = 0.5 * self._speed + 0.5 * (step / dt)     # v10.6: for the braking distance
        if (not self._done) and self._goal_handle is not None \
                and self._rescue_phase is None:
            if self._stall_ref is None or len(self._stall_ref) != 4:
                self._stall_ref = (x, y, yaw, now_s)
            rx, ry, rw, rt = self._stall_ref
            turned = abs((yaw - rw + math.pi) % (2.0 * math.pi) - math.pi)
            # TRAFFIC_PATIENCE counts standing while BLOCKED: getting
            # TRAFFIC_PROGRESS m further between holds starts it again (a busy
            # aisle mouth is not a blocked lane; the hard deadline caps the total)
            if self._block_ref is None:
                self._block_ref = (x, y)
            elif math.hypot(x - self._block_ref[0], y - self._block_ref[1]) > self.TRAFFIC_PROGRESS:
                self._block_ref, self._blocked_s = (x, y), 0.0
            # v10.4: time spent in traffic is not a stall. The stall clock and
            # the goal deadline stand still; standing in it counts towards
            # TRAFFIC_PATIENCE, after which the waypoint is put off.
            if self._in_traffic(now_s) and dt > 0.0:
                self._stall_ref = (rx, ry, rw, rt + dt)
                rt += dt
                if self.goal_deadline is not None:
                    self.goal_deadline += dt
                standing = step < 0.05 * dt + 0.005
                if standing:
                    self._blocked_s += dt
                    self._standing_s += dt
                    if not self._stand_logged and self._standing_s >= self.TRAFFIC_LOG_AFTER:
                        self._stand_logged = True
                        cause = (self._traffic_cause(self._traffic)
                                 or 'the way has only just cleared')
                        self.get_logger().info(
                            f'TRAFFIC: wp{self.gi + 1} waiting ({cause}); '
                            f'{self._blocked_s:.0f} s of {self.TRAFFIC_PATIENCE:.0f} s')
                if (self._blocked_s > self.TRAFFIC_PATIENCE
                        and not self._defer_pending):
                    self._defer_pending = True
                    self._defer_why = (f'blocked by traffic for '
                                       f'{self.TRAFFIC_PATIENCE:.0f} s')
                    self.goal_deadline = None
                    self.get_logger().warn(
                        f'wp{self.gi + 1}: {self._defer_why} -> putting it off')
                    try:
                        self._goal_handle.cancel_goal_async()
                    except Exception:
                        pass
            if step >= 0.05 * dt + 0.005 or not self._in_traffic(now_s):
                if self._stand_logged:
                    self.get_logger().info(
                        f'TRAFFIC: wp{self.gi + 1} moving again after '
                        f'{self._standing_s:.0f} s')
                self._standing_s, self._stand_logged = 0.0, False
            # v10.7: standing with the guard saying GO and nobody near is not traffic: if a rack
            # or wall is right in front, Nav2 cannot drive on from here (shift_v106a: 279 s)
            if dt > 0.0 and step < 0.05 * dt + 0.005 and self._guard_seen and self._go_clear(now_s):
                self._wedge_s += dt
            if (self._wedge_s >= self.WEDGE_S and now_s >= self._wedge_next
                    and self._man is None and not self._unstick_pending
                    and not self._defer_pending and not self._bypass_pending
                    and not self._detour_pending and self._veh_skip_pending is None
                    and not self._ahead_hold):
                self._wedge_next = now_s + 2.0          # the map is looked at every 2 s at most
                w = self._wedged_now()
                if w is not None and self._unsticks_left():
                    self._unstick_pending = True
                    self._unstick_why = (
                        f'standing {self._wedge_s:.0f} s with the guard saying go, nobody within '
                        f'{self.WEDGE_PERSON:.1f} m and a rack or wall '
                        + (f'{w[0]:.2f} m ahead' if w[0] <= self.WEDGE_FRONT else f'{w[1]:.2f} m away')
                        + ': Nav2 cannot drive on from here')
                    self.goal_deadline = None
                    try:
                        self._goal_handle.cancel_goal_async()
                    except Exception:
                        pass
            if (math.hypot(x - rx, y - ry) > self.STALL_DIST
                    or turned > self.STALL_TURN):
                self._stall_ref = (x, y, yaw, now_s)
                self._wedge_s = 0.0                     # v10.7: it moved
            elif self._defer_pending or self._unstick_pending:
                pass                        # cancel already on its way
            elif now_s - rt > self.STALL_SECONDS:
                self.get_logger().warn(
                    f'STALL: no movement for {self.STALL_SECONDS:.0f}s -> '
                    f'canceling goal early. ' + self._pose_state_str())
                self._stall_ref = None
                self.goal_deadline = None
                try:
                    self._goal_handle.cancel_goal_async()
                except Exception:
                    pass
        else:
            self._stall_ref = None
        self._tick_lane(x, y, now_s, dt, step)
        lx, ly = self._paint_last
        seg = math.hypot(x - lx, y - ly)
        steps = max(1, int(seg / (cs * 0.5)))
        for s in range(steps + 1):
            f = s / steps
            px = lx + (x - lx) * f
            py = ly + (y - ly) * f
            cx0 = int(math.floor((px - r) / cs))
            cx1 = int(math.floor((px + r) / cs))
            cy0 = int(math.floor((py - r) / cs))
            cy1 = int(math.floor((py + r) / cs))
            for cx in range(cx0, cx1 + 1):
                for cy in range(cy0, cy1 + 1):
                    ccx = cx * cs + cs / 2.0
                    ccy = cy * cs + cs / 2.0
                    if math.hypot(ccx - px, ccy - py) <= r:
                        self.visited.add((cx, cy))
        self._paint_last = (x, y)

        # one CUBE_LIST marker holding every cleaned cell (efficient)
        m = Marker()
        m.header.frame_id = 'map'
        m.header.stamp = self.get_clock().now().to_msg()
        m.ns = 'clean'
        m.id = 0
        m.type = Marker.CUBE_LIST
        m.action = Marker.ADD
        m.scale.x = cs
        m.scale.y = cs
        m.scale.z = 0.02
        m.color.g = 0.90
        m.color.b = 0.15
        m.color.a = 0.85
        m.pose.orientation.w = 1.0
        for (cx, cy) in self.visited:
            p = Point()
            p.x = cx * cs + cs / 2.0
            p.y = cy * cs + cs / 2.0
            # z = 0.06, NOT 0.01. The OccupancyGrid map renders at z=0 and a
            # 0.02-tall cube sitting at z=0.01 has its bottom face exactly on
            # the map plane -> z-fighting, so the paint rendered faint and
            # patchy and looked like it was "not following" the robot.
            p.z = 0.06
            m.points.append(p)
        ma = MarkerArray()
        ma.markers.append(m)
        self.mk.publish(ma)

        # coverage % = cleaned floor cells / total floor cells
        # coverage % = cleaned floor cells / total floor cells
        # FRAME BUG FIX: `visited` cells are indexed from the MAP-frame robot
        # pose (map = world - spawn), but is_floor_world()/count_floor_cells()
        # work in WORLD coordinates. Testing a map coordinate against the world
        # floor model checks a point 4 m away in both axes, so most genuinely
        # cleaned cells failed the floor test and the reported percentage was
        # far below the truth. Convert back to world before testing.
        if self._map_floor is not None:
            # real occupancy grid: visited cells and the grid are both map-frame
            covered = sum(1 for (cx, cy) in self.visited
                          if self.map_is_floor(cx * cs + cs / 2.0,
                                               cy * cs + cs / 2.0))
            denom = self._map_coverable
            src = 'reachable'
        else:
            covered = sum(1 for (cx, cy) in self.visited
                          if self.is_floor_world(cx * cs + cs / 2.0 + self.spawn_x,
                                                 cy * cs + cs / 2.0 + self.spawn_y))
            denom = self.total_floor
            src = 'model'
        pct = min(100.0, 100.0 * covered / denom)

        s = String()
        prog = 'DONE' if self.gi >= len(self.goals) else f'{self.gi + 1}/{len(self.goals)}'
        s.data = f'{prog} | coverage {pct:.0f}%'
        self.st.publish(s)
        self.publish_state(pct, x, y)

        # heartbeat every 10 s so the terminal proves the paint is live.
        # If 'painted' stops rising while the robot is moving, TF is the
        # problem, not the coverage logic.
        self._tick = getattr(self, '_tick', 0) + 1
        if self._tick % 50 == 0:
            self.get_logger().info(
                f'COVERAGE {pct:.1f}% [{src}] | painted {len(self.visited)} cells '
                f'| robot map ({x:.2f},{y:.2f}) | goal {prog}')


def main(args=None):
    rclpy.init(args=args)
    node = None
    try:
        node = CoverageNav2()
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    except Exception as e:          # v10.6: Ctrl+C / stack.sh stop end rclpy.spin this way
        if type(e).__name__ != 'ExternalShutdownException':
            raise
    finally:
        if node is not None:
            try:
                node.destroy_node()
            except Exception:
                pass
        if rclpy.ok():
            rclpy.shutdown()


if __name__ == '__main__':
    main()