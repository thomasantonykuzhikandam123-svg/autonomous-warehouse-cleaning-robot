#!/usr/bin/env python3
"""lanes.py - keep-left traffic lanes for the warehouse, and a lane router.

Why: real warehouses do not let vehicles drive just anywhere. Tuggers and AGVs
follow a network of one-way lanes, like a road map, and the fleet manager sends
each one a route through it (VDA 5050 orders are graphs of nodes and edges;
Nav2's Route Server plans on a lane graph the same way). This file is that road
map for our warehouse. route() turns a start and a goal into the lane points the
robot must pass (via-points). Nav2 plans the short free-space pieces between
them and still does all the obstacle handling.

Rules (Australia keeps left):
  aisles, 3 m, two-way     northbound in the west half, southbound in the east half
  front and back corridor  eastbound in the north half, westbound in the south half
  entering an aisle        line up on the lane 0.75 m before the rack ends,
                           then drive in straight, like a tugger squaring up
  joining a lane           only in its direction of travel (or square to it)
  turning round            allowed (the robot turns on the spot), costs extra

Usage (map frame in and out; map = world - (4, 4)):
  python3 lanes.py                    list the lanes
  python3 lanes.py 0 0 -1.25 8        route from map (0, 0) to map (-1.25, 8)
  python3 lanes.py show               draw the lanes in RViz (MarkerArray /lanes)
"""
import heapq
import math
import sys

MAP_OFFSET = (4.0, 4.0)                 # map = world - MAP_OFFSET
ROBOT_R = 0.30
CLEARANCE = ROBOT_R + 0.05              # line-of-sight margin around obstacles

# warehouse.sdf, world frame, (xmin, ymin, xmax, ymax)
OBSTACLES = {
    'rack_1': (1.0, 5.5, 2.0, 17.5),
    'rack_2': (5.0, 5.5, 6.0, 17.5),
    'rack_3': (9.0, 5.5, 10.0, 17.5),
    'rack_4': (13.0, 5.5, 14.0, 17.5),
    'packing_area': (1.0, 0.5, 4.0, 2.5),
    'pallet_area': (4.5, 0.5, 5.5, 2.5),
    'office': (10.0, 0.5, 14.0, 2.5),
    'pallet_1': (0.2, 8.4, 1.0, 9.6),
    'pallet_2': (14.0, 8.4, 14.8, 9.6),
    'front_wall': (0.0, -0.1, 15.0, 0.1),
    'back_wall': (0.0, 19.9, 15.0, 20.1),
    'left_wall': (-0.1, 0.0, 0.1, 20.0),
    'right_wall': (14.9, 0.0, 15.1, 20.0),
}
AISLES = [(2.0, 5.0), (6.0, 9.0), (10.0, 13.0)]   # (west rack face, east rack face)
RACK_Y = (5.5, 17.5)                               # rack front ends, back ends
LANE_IN = 0.75           # lane centre from the rack face: quarter points of 3 m
FRONT = {'EB': 4.75, 'WB': 3.25}                   # front corridor, y 2.5-5.5
BACK = {'EB': 19.3, 'WB': 18.1}                    # back corridor, y 17.5-19.9
CORRIDOR_X = (1.0, 14.0)
STEP = 1.0               # spacing of lane points along a lane
UTURN_EXTRA = 1.0        # metres charged extra for turning round at a corridor end
OFF_LANE = 1.5           # cost factor for driving off the lanes to join or leave
JOIN_R = 2.6             # a start or goal joins lane points up to this far away
DIRECT_R = 1.5           # start and goal this close, same area: no lanes needed
KEEP_TURN_DEG = 10.0     # a route point becomes a via-point if the route turns there

EAST, WEST, NORTH, SOUTH = (1.0, 0.0), (-1.0, 0.0), (0.0, 1.0), (0.0, -1.0)


def frange(a, b, step):
    n = int(math.floor((b - a) / step + 1e-9))
    return [round(a + i * step, 3) for i in range(n + 1)]


def to_world(p):
    return (p[0] + MAP_OFFSET[0], p[1] + MAP_OFFSET[1])


def to_map(p):
    return (p[0] - MAP_OFFSET[0], p[1] - MAP_OFFSET[1])


def zone_of(p):
    """'front', 'back', 'aisle1'..'aisle3' or 'none' (world frame point)."""
    x, y = p
    if 0.1 < x < 14.9 and 0.1 < y <= RACK_Y[0]:
        return 'front'
    if 0.1 < x < 14.9 and RACK_Y[1] <= y < 19.9:
        return 'back'
    for k, (xw, xe) in enumerate(AISLES, 1):
        if xw < x < xe and RACK_Y[0] < y < RACK_Y[1]:
            return f'aisle{k}'
    return 'none'


def _segment_hits_box(p, q, box, margin):
    """True if the segment p-q passes through box grown by margin (slab test)."""
    x0, y0, x1, y1 = box[0] - margin, box[1] - margin, box[2] + margin, box[3] + margin
    t0, t1 = 0.0, 1.0
    for p_i, d_i, lo, hi in ((p[0], q[0] - p[0], x0, x1), (p[1], q[1] - p[1], y0, y1)):
        if abs(d_i) < 1e-12:
            if p_i < lo or p_i > hi:
                return False
            continue
        a, b = (lo - p_i) / d_i, (hi - p_i) / d_i
        if a > b:
            a, b = b, a
        t0, t1 = max(t0, a), min(t1, b)
        if t0 > t1:
            return False
    return True


def line_of_sight(p, q, margin=CLEARANCE):
    """World-frame points p, q: can the robot drive straight from p to q?"""
    return not any(_segment_hits_box(p, q, box, margin) for box in OBSTACLES.values())


def _wrap(a):
    return (a + math.pi) % (2 * math.pi) - math.pi


class LaneMap:
    """Directed graph of lane points, world frame."""

    def __init__(self):
        self.xy, self.zone, self.dir, self.mouth = [], [], [], []
        self.out = []
        self.lanes = []          # (name, [node ids in travel order]) for display
        self.links = []          # (from, to) turn / junction edges, for display
        self._ids = {}
        self._build()

    def _node(self, x, y, direction, mouth=False):
        key = (round(x, 3), round(y, 3))
        if key in self._ids:
            i = self._ids[key]
            self.mouth[i] = self.mouth[i] or mouth
            return i
        i = len(self.xy)
        self._ids[key] = i
        self.xy.append(key)
        self.zone.append(zone_of(key))
        self.dir.append(direction)
        self.mouth.append(mouth)
        self.out.append([])
        return i

    def _edge(self, i, j, extra=0.0):
        d = math.dist(self.xy[i], self.xy[j])
        self.out[i].append((j, d + extra))

    def _lane(self, name, points, direction, mouths=()):
        ids = [self._node(x, y, direction, (x, y) in mouths) for x, y in points]
        for a, b in zip(ids, ids[1:]):
            self._edge(a, b)
        self.lanes.append((name, ids))
        return ids

    def _link(self, a, b, extra=0.0):
        self._edge(a, b, extra)
        self.links.append((a, b))

    def _build(self):
        xs = set(frange(CORRIDOR_X[0], CORRIDOR_X[1], STEP))
        lane_x = []
        for xw, xe in AISLES:
            lane_x.append((round(xw + LANE_IN, 3), round(xe - LANE_IN, 3)))
            xs.update(lane_x[-1])
        xs = sorted(xs)
        # front corridor: the eastbound points in line with an aisle lane are the
        # line-up points (mouths) where the robot squares up before entering
        mouths = {(x, FRONT['EB']) for pair in lane_x for x in pair}
        f_eb = self._lane('front EB', [(x, FRONT['EB']) for x in xs], EAST, mouths)
        f_wb = self._lane('front WB', [(x, FRONT['WB']) for x in reversed(xs)], WEST)
        b_eb = self._lane('back EB', [(x, BACK['EB']) for x in xs], EAST)
        b_wb = self._lane('back WB', [(x, BACK['WB']) for x in reversed(xs)], WEST)
        at = {}
        for lane in (f_eb, f_wb, b_eb, b_wb):
            for i in lane:
                at[self.xy[i]] = i
        # turning round at the corridor ends
        for eb, wb, y_eb, y_wb in ((f_eb, f_wb, FRONT['EB'], FRONT['WB']),
                                   (b_eb, b_wb, BACK['EB'], BACK['WB'])):
            self._link(at[(xs[-1], y_eb)], at[(xs[-1], y_wb)], UTURN_EXTRA)
            self._link(at[(xs[0], y_wb)], at[(xs[0], y_eb)], UTURN_EXTRA)
        ys = frange(RACK_Y[0] + 0.5, RACK_Y[1] - 0.5, STEP)       # 6.0 .. 17.0
        for k, (xn, xs_) in enumerate(lane_x, 1):
            nb = self._lane(f'aisle{k} NB', [(xn, y) for y in ys], NORTH)
            sb = self._lane(f'aisle{k} SB', [(xs_, y) for y in reversed(ys)], SOUTH)
            # in from the front: left turn off EB (via the mouth), right turn off WB
            self._link(at[(xn, FRONT['EB'])], nb[0])
            self._link(at[(xn, FRONT['WB'])], nb[0])
            # out to the front: left turn onto EB, right turn onto WB
            self._link(sb[-1], at[(xs_, FRONT['EB'])])
            self._link(sb[-1], at[(xs_, FRONT['WB'])])
            # out at the back: left turn onto WB, right turn onto EB
            self._link(nb[-1], at[(xn, BACK['WB'])])
            self._link(nb[-1], at[(xn, BACK['EB'])])
            # in from the back: left turn off WB, right turn off EB
            self._link(at[(xs_, BACK['WB'])], sb[0])
            self._link(at[(xs_, BACK['EB'])], sb[0])

    def _joins(self, p, leaving):
        """[(node, cost)] lane points a free point p can join (leaving=False)
        or be reached from (leaving=True)."""
        zp = zone_of(p)
        found = []
        for i, q in enumerate(self.xy):
            if zp != 'none' and self.zone[i] != zp:
                continue
            d = math.dist(p, q)
            if d > JOIN_R:
                continue
            if d > 1e-6:
                if not line_of_sight(p, q):
                    continue
                if not self.mouth[i]:
                    move = (p[0] - q[0], p[1] - q[1]) if leaving else (q[0] - p[0], q[1] - p[1])
                    if move[0] * self.dir[i][0] + move[1] * self.dir[i][1] < -1e-9:
                        continue          # never against the lane's direction
            found.append((i, d * OFF_LANE))
        return found

    def route_world(self, start, goal):
        """World-frame start, goal -> list of world points [start, ..., goal]."""
        n = len(self.xy)
        s_id, g_id = n, n + 1
        pts = self.xy + [tuple(start), tuple(goal)]
        extra = {s_id: self._joins(start, leaving=False)}
        into_goal = {i: c for i, c in self._joins(goal, leaving=True)}
        zs, zg = zone_of(start), zone_of(goal)
        if (zs == zg and zs != 'none' and math.dist(start, goal) <= DIRECT_R
                and line_of_sight(start, goal)):
            extra[s_id].append((g_id, math.dist(start, goal)))
        best = {s_id: 0.0}
        prev = {}
        heap = [(0.0, s_id)]
        while heap:
            c, i = heapq.heappop(heap)
            if i == g_id:
                break
            if c > best.get(i, math.inf):
                continue
            edges = list(extra.get(i, [])) if i >= n else list(self.out[i])
            if i in into_goal:
                edges.append((g_id, into_goal[i]))
            for j, w in edges:
                if c + w < best.get(j, math.inf) - 1e-12:
                    best[j] = c + w
                    prev[j] = i
                    heapq.heappush(heap, (c + w, j))
        if g_id not in best:
            raise ValueError(f'no lane route from {start} to {goal} (world)')
        path, i = [g_id], g_id
        while i != s_id:
            i = prev[i]
            path.append(i)
        return [pts[i] for i in reversed(path)]


def simplify(points):
    """Drop the start; keep the points where the route turns, and the goal."""
    pts = [points[0]]
    for p in points[1:]:
        if math.dist(p, pts[-1]) > 1e-6:      # a goal on a lane point repeats it
            pts.append(p)
    points = pts
    keep = []
    for a, b, c in zip(points, points[1:], points[2:]):
        h1 = math.atan2(b[1] - a[1], b[0] - a[0])
        h2 = math.atan2(c[1] - b[1], c[0] - b[0])
        if abs(_wrap(h2 - h1)) > math.radians(KEEP_TURN_DEG):
            keep.append(b)
    keep.append(points[-1])
    return keep


_MAP = None


def lane_map():
    global _MAP
    if _MAP is None:
        _MAP = LaneMap()
    return _MAP


def route(start, goal, goal_yaw=None):
    """Map-frame start (x, y) and goal (x, y) -> [(x, y, yaw), ...] ending at
    the goal. Every point but the last is a via-point to drive through."""
    pts = simplify(lane_map().route_world(to_world(start), to_world(goal)))
    out, prev = [], to_world(start)
    for k, p in enumerate(pts):
        yaw = math.atan2(p[1] - prev[1], p[0] - prev[0])
        if k == len(pts) - 1 and goal_yaw is not None:
            yaw = goal_yaw
        m = to_map(p)
        out.append((round(m[0], 3), round(m[1], 3), yaw))
        prev = p
    return out


def show():
    """Publish the lanes as arrows for RViz: add a MarkerArray display on /lanes."""
    import rclpy
    from rclpy.node import Node
    from geometry_msgs.msg import Point
    from visualization_msgs.msg import Marker, MarkerArray

    lm = lane_map()
    colours = {EAST: (0.1, 0.8, 0.9), NORTH: (0.1, 0.8, 0.9),     # north / east: cyan
               WEST: (1.0, 0.6, 0.1), SOUTH: (1.0, 0.6, 0.1)}     # south / west: amber
    msg = MarkerArray()

    def arrow(i, a, b, rgb, alpha, ns):
        m = Marker()
        m.header.frame_id = 'map'
        m.ns, m.id, m.type, m.action = ns, i, Marker.ARROW, Marker.ADD
        m.scale.x, m.scale.y, m.scale.z = 0.05, 0.14, 0.18
        m.color.r, m.color.g, m.color.b = rgb
        m.color.a = alpha
        for p in (a, b):
            q = to_map(p)
            m.points.append(Point(x=q[0], y=q[1], z=0.05))
        m.pose.orientation.w = 1.0
        return m

    k = 0
    for _, ids in lm.lanes:
        for a, b in zip(ids, ids[1:]):
            pa, pb = lm.xy[a], lm.xy[b]
            # shorten each arrow a little so the direction reads clearly
            f = 0.15 / max(math.dist(pa, pb), 1e-6)
            pa2 = (pa[0] + (pb[0] - pa[0]) * f, pa[1] + (pb[1] - pa[1]) * f)
            pb2 = (pb[0] - (pb[0] - pa[0]) * f, pb[1] - (pb[1] - pa[1]) * f)
            msg.markers.append(arrow(k, pa2, pb2, colours[lm.dir[a]], 0.9, 'lanes'))
            k += 1
    for a, b in lm.links:
        msg.markers.append(arrow(k, lm.xy[a], lm.xy[b], (0.7, 0.7, 0.7), 0.5, 'turns'))
        k += 1

    try:
        from rclpy.executors import ExternalShutdownException
    except ImportError:
        ExternalShutdownException = KeyboardInterrupt
    rclpy.init()
    node = Node('lanes')
    pub = node.create_publisher(MarkerArray, 'lanes', 1)
    node.create_timer(2.0, lambda: pub.publish(msg))
    print(f'Publishing {len(msg.markers)} lane arrows on /lanes every 2 s. '
          f'In RViz: Add > By topic > /lanes > MarkerArray. Ctrl+C to stop.')
    try:
        rclpy.spin(node)
    except (KeyboardInterrupt, ExternalShutdownException):
        pass     # Ctrl+C, or stack.sh stop


def main(argv):
    if len(argv) == 2 and argv[1] == 'show':
        show()
        return
    if len(argv) == 5:
        s, g = (float(argv[1]), float(argv[2])), (float(argv[3]), float(argv[4]))
        for x, y, yaw in route(s, g):
            print(f'  via  ({x:6.2f}, {y:6.2f})  heading {math.degrees(yaw):6.1f} deg')
        return
    lm = lane_map()
    for name, ids in lm.lanes:
        a, b = to_map(lm.xy[ids[0]]), to_map(lm.xy[ids[-1]])
        print(f'  {name:10s} map ({a[0]:6.2f}, {a[1]:6.2f}) -> ({b[0]:6.2f}, {b[1]:6.2f})  '
              f'{len(ids)} points')
    print(f'  {len(lm.links)} turn links. Route: python3 lanes.py sx sy gx gy (map frame)')


if __name__ == '__main__':
    main(sys.argv)
