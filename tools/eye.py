#!/usr/bin/env python3
"""eye.py (v8.2) - the robot's eye: people found in the camera image, located with the lidar.

    python3 ~/ros2_ws/src/tools/eye.py [label]
    (stack.sh start <label> guard camera runs it for you, with a window showing what it sees)

1. LOOK   /camera/image_raw from Gazebo (640 x 480, 5 Hz). The newest frame goes, as a JPEG, to
          eye_server.py on the Mac (the VM's default gateway, port 8765), which runs YOLOv8n on the Mac's GPU
          and answers with boxes. Older frames are dropped: it never falls behind.
2. FUSE   the lidar scan closest in time is projected into the image (each lidar point through
          the camera's lens model; TF says where both sensors are). For every person box, the
          lidar points inside the box's middle columns are grouped by range, and the nearest
          group of at least 2 points (her legs) that is as far away as the box's feet stand on
          the floor is the person: WHERE comes from the lidar (to a few cm), WHAT from the
          camera. A person box with no such group is camera-only: the
          lidar can't see that person (too far, hidden low down, or not on the scan plane).
3. SHOW   /eye/image_annotated  the image with the boxes (green: camera + lidar, yellow: camera
                                only, grey: other objects), confidence and range, and the lidar
                                points drawn where the scan plane meets the scene
          /eye/detections       vision_msgs/Detection2DArray, every detection
          /eye/people           RViz markers (map frame): a green figure where both sensors see a
                                person, a yellow line towards a person only the camera sees
          /eye/persons          geometry_msgs/PoseArray, map frame, stamped with the image: every
                                person both sensors agree on. people_guard.py v9 reads it.
          ~/runs/eye/<label>.csv one row per frame for eye_report.py, with where the scripted
                                worker really was, and (v8.2) whether the camera found the
                                scene's standing picker and called the look-alike post a person
If eye_server.py isn't running, the image still goes out, with a red DETECTOR OFFLINE banner,
and it keeps retrying (and says why it can't reach the Mac).

v8 (4 Oct): first version.
v8.1 (4 Oct): finds the Mac by the VM's default gateway instead of assuming 10.0.2.2
    (UTM's shared network puts the Mac at 192.168.64.1).
v8.2 (4 Oct): /eye/persons for the guard; the scene (scene.py: a standing picker and a post in
    aisle 2) is scored per frame; the offline message says what the error means.
"""
import collections
import csv
import json
import math
import os
import sys
import threading
import time
import urllib.error
import urllib.request

import numpy as np

VERSION = 'v8.2'


def default_server(route_file='/proc/net/route', port=8765):
    """The Mac as the VM sees it: its default gateway (UTM's shared network: 192.168.64.1;
    QEMU's emulated network: 10.0.2.2)."""
    try:
        import socket
        import struct
        for line in open(route_file).read().splitlines()[1:]:
            f = line.split()
            if len(f) > 2 and f[1] == '00000000' and f[2] != '00000000':
                return f'http://{socket.inet_ntoa(struct.pack("<L", int(f[2], 16)))}:{port}'
    except (OSError, ValueError):
        pass
    return f'http://10.0.2.2:{port}'


SERVER = os.environ.get('EYE_SERVER') or default_server()
RATE = float(os.environ.get('EYE_SEND_RATE', '5'))   # frames per second sent to the detector at most
TIMEOUT = 1.0          # s for one detector request
JPEG_QUALITY = 80
PERSON = 'person'
MIN_CONF = 0.35        # a person box counts from this confidence
OTHER_CONF = 0.50      # other objects are drawn from this confidence
HFOV = 1.5708          # rad, used until /camera/camera_info arrives
SCAN_SYNC = 0.25       # s: the scan used must be this close in time to the image
SHRINK = 0.15          # of a box's width, ignored on each side (the background shows there)
GROUP_DEPTH = 0.45     # m: lidar points this far behind the nearest one are the same person (legs)
MIN_POINTS = 2         # lidar points needed to locate a person
FEET_TOL = 0.25        # the lidar group must be within this fraction (or FEET_MIN m) of the depth
FEET_MIN = 0.6         #   where the box's bottom edge meets the floor (else it's the wall behind)
CAM_FRAME = 'camera_optical_link'
STATUS_EVERY = 10.0    # s (wall) between status lines
RETRY_OFFLINE = 2.0    # s between tries while the detector is down

# the scripted worker (worker_test actor), world frame (t, x, y), 6 s loop; map = world - (4, 4).
# Same as worker_clearance.py: used only to score the eye, never to drive.
WAYPOINTS = [(0.0, 2.4, 8.0), (2.2, 4.6, 8.0), (3.0, 4.6, 8.0), (5.2, 2.4, 8.0), (6.0, 2.4, 8.0)]
PERIOD = 6.0
MAP_OFFSET = (4.0, 4.0)

SCENE_FILE = os.path.expanduser('~/runs/scene.json')   # written by scene.py spawn
HIT_PERSON = 0.8       # m: a fused person this close to the scene's picker found her
HIT_BEARING = 0.10     # rad: ... or a camera-only person box this close in bearing
FALSE_POST = 0.6       # m: a fused person this close to the post is a false person


def load_scene(path=SCENE_FILE):
    """[{name, kind ('person' or 'object'), x, y}] in the map frame, [] without a scene."""
    try:
        return [o for o in json.load(open(path)).get('objects', []) if {'name', 'kind', 'x', 'y'} <= set(o)]
    except (OSError, ValueError, AttributeError):
        return []


def scene_cols(scene, robot, cam_offset, cam, people):
    """Per scene object: in the camera's view (0/1), range from the camera (m), and hit:
    for a person, did the camera find her; for an object, did it call it a person."""
    out = []
    for o in scene:
        if robot is None or cam is None:
            out += ['', '', '']
            continue
        cxm = robot[0] + cam_offset[0] * math.cos(robot[2]) - cam_offset[1] * math.sin(robot[2])
        cym = robot[1] + cam_offset[0] * math.sin(robot[2]) + cam_offset[1] * math.cos(robot[2])
        dx, dy = o['x'] - cxm, o['y'] - cym
        rng = math.hypot(dx, dy)
        b = math.atan2(math.sin(math.atan2(dy, dx) - robot[2]), math.cos(math.atan2(dy, dx) - robot[2]))
        half = math.atan2(cam[2], cam[0])
        view = abs(b) < half - 0.09 and 0.8 < rng < 8.0
        if o['kind'] == 'person':
            hit = any(p['fused'] and p['xy_map'] is not None
                      and math.hypot(p['xy_map'][0] - o['x'], p['xy_map'][1] - o['y']) < HIT_PERSON
                      for p in people) or any(
                not p['fused'] and abs(camera_bearing((p['box'][0] + p['box'][2]) / 2.0, cam) - b) < HIT_BEARING
                for p in people)
        else:
            hit = any(p['fused'] and p['xy_map'] is not None
                      and math.hypot(p['xy_map'][0] - o['x'], p['xy_map'][1] - o['y']) < FALSE_POST
                      for p in people)
        out += [str(int(view)), f'{rng:.2f}', str(int(hit))]
    return out


LOG_FIELDS = ['t', 'wall', 'ok', 'det_ms', 'trip_ms', 'dets', 'persons', 'fused', 'cam_only',
              'p_x', 'p_y', 'p_range', 'p_bearing', 'p_conf', 'p_points', 'p_feet',
              'robot_x', 'robot_y', 'robot_yaw', 'truth_x', 'truth_y', 'truth_bearing', 'truth_range',
              'truth_in_view', 'truth_lidar', 'err', 'bearing_err', 'scan_dt', 'others']


# ---------------------------------------------------------------- geometry
def quat_matrix(x, y, z, w):
    return np.array([[1 - 2 * (y * y + z * z), 2 * (x * y - z * w), 2 * (x * z + y * w)],
                     [2 * (x * y + z * w), 1 - 2 * (x * x + z * z), 2 * (y * z - x * w)],
                     [2 * (x * z - y * w), 2 * (y * z + x * w), 1 - 2 * (x * x + y * y)]])


def transform_matrix(t, q):
    """4x4 from a geometry_msgs Transform-like (translation, rotation)."""
    m = np.eye(4)
    m[:3, :3] = quat_matrix(q.x, q.y, q.z, q.w)
    m[:3, 3] = (t.x, t.y, t.z)
    return m


def intrinsics(k, width, height, hfov=HFOV):
    """(fx, fy, cx, cy) from a CameraInfo K, or from the field of view."""
    if k is not None and len(k) == 9 and k[0] > 0:
        return float(k[0]), float(k[4]), float(k[2]), float(k[5])
    fx = (width / 2.0) / math.tan(hfov / 2.0)
    return fx, fx, width / 2.0, height / 2.0


def project_scan(ranges, angle_min, angle_inc, range_min, range_max, cam_from_lidar, cam, width, height):
    """The scan's valid points in the image. Returns dict of arrays: u, v (pixels), depth (m in
    front of the camera), xy (N x 2, lidar frame), rng (m)."""
    fx, fy, cx, cy = cam
    r = np.asarray(ranges, dtype=float)
    a = angle_min + angle_inc * np.arange(len(r))
    ok = np.isfinite(r) & (r > range_min) & (r < range_max)
    r, a = r[ok], a[ok]
    pts = np.column_stack((r * np.cos(a), r * np.sin(a), np.zeros(len(r)), np.ones(len(r))))
    pc = (cam_from_lidar @ pts.T).T
    front = pc[:, 2] > 0.2
    pc, pts, r = pc[front], pts[front], r[front]
    u = fx * pc[:, 0] / pc[:, 2] + cx
    v = fy * pc[:, 1] / pc[:, 2] + cy
    inside = (u >= 0) & (u < width) & (v >= 0) & (v < height)
    return dict(u=u[inside], v=v[inside], depth=pc[inside, 2], xy=pts[inside, :2], rng=r[inside])


def feet_depth(box, cam, cam_height, img_h):
    """Depth (m in front of the camera) where the box's bottom edge meets the floor, or None
    when the bottom is cut off by the image edge or too close to the horizon to tell."""
    fx, fy, cx, cy = cam
    y2 = box[3]
    if y2 >= img_h - 3 or y2 <= cy + 5:
        return None
    return fy * cam_height / (y2 - cy)


def locate(box, proj, cam=None, cam_height=None, img_h=None, shrink=SHRINK, group=GROUP_DEPTH,
           min_points=MIN_POINTS):
    """The person in a box, from the lidar: (x, y) in the lidar frame and the number of points,
    or None. The nearest group of points in the box's middle columns (legs before background)
    whose depth agrees with where the box's feet meet the floor, so the wall or rack behind
    someone the lidar can't see is never taken for them."""
    x1, _, x2, _ = box
    w = x2 - x1
    sel = (proj['u'] >= x1 + shrink * w) & (proj['u'] <= x2 - shrink * w)
    if not np.any(sel):
        return None
    rng, xy, depth = proj['rng'][sel], proj['xy'][sel], proj['depth'][sel]
    order = np.argsort(rng)
    rng, xy, depth = rng[order], xy[order], depth[order]
    zf = feet_depth(box, cam, cam_height, img_h) if cam is not None and cam_height else None
    i = 0
    while i < len(rng):                          # nearest group first; skip stray single points
        g = (rng >= rng[i]) & (rng <= rng[i] + group)
        n = int(np.sum(g))
        if n >= min_points:
            z = float(np.median(depth[g]))
            if zf is None or abs(z - zf) <= max(FEET_MIN, FEET_TOL * zf):
                c = xy[g].mean(axis=0)
                return float(c[0]), float(c[1]), n
            i = int(np.nonzero(g)[0][-1]) + 1    # not her: try the group behind it
        else:
            i += 1
    return None


def feet_row(depth, cam_height, cam):
    """Image row where the floor is at this depth in front of a level camera."""
    fx, fy, cx, cy = cam
    return fy * cam_height / max(depth, 0.05) + cy


def camera_bearing(u, cam):
    """Horizontal angle of image column u from the optical axis, + to the left (rad)."""
    fx, fy, cx, cy = cam
    return math.atan2(cx - u, fx)


def worker_truth(t):
    """Scripted worker position at sim time t, map frame."""
    t %= PERIOD
    for (t0, x0, y0), (t1, x1, y1) in zip(WAYPOINTS, WAYPOINTS[1:]):
        if t0 <= t <= t1:
            f = (t - t0) / (t1 - t0) if t1 > t0 else 0.0
            return x0 + f * (x1 - x0) - MAP_OFFSET[0], y0 + f * (y1 - y0) - MAP_OFFSET[1]
    return WAYPOINTS[0][1] - MAP_OFFSET[0], WAYPOINTS[0][2] - MAP_OFFSET[1]


def analyse(dets, proj, cam, cam_height, map_from_lidar=None, img_h=480):
    """People from detections (+ lidar). Returns (people, others): people are dicts with box,
    conf, fused, xy_lidar, xy_map, range, points, feet (box bottom / expected floor row)."""
    people, others = [], []
    for d in dets:
        if d.get('name') == PERSON and d.get('conf', 0) >= MIN_CONF:
            p = dict(box=d['box'], conf=d['conf'], fused=False, xy_lidar=None, xy_map=None,
                     range=None, points=0, feet=None)
            hit = locate(d['box'], proj, cam, cam_height, img_h) if proj is not None else None
            if hit is not None:
                x, y, n = hit
                p.update(fused=True, xy_lidar=(x, y), range=math.hypot(x, y), points=n)
                if map_from_lidar is not None:
                    q = map_from_lidar @ np.array([x, y, 0.0, 1.0])
                    p['xy_map'] = (float(q[0]), float(q[1]))
                sel = (np.abs(proj['xy'][:, 0] - x) < 0.5) & (np.abs(proj['xy'][:, 1] - y) < 0.5)
                depth = float(np.median(proj['depth'][sel])) if np.any(sel) else None
                if depth:
                    p['feet'] = d['box'][3] / feet_row(depth, cam_height, cam)
            people.append(p)
        elif d.get('conf', 0) >= OTHER_CONF:
            others.append(d)
    people.sort(key=lambda p: (not p['fused'], -p['conf']))
    return people, others


# ---------------------------------------------------------------- drawing
def range_colour(r):
    """BGR: red near (0.5 m) to blue far (8 m)."""
    f = min(max((r - 0.5) / 7.5, 0.0), 1.0)
    return (int(255 * f), int(80 + 100 * (1 - abs(2 * f - 1))), int(255 * (1 - f)))


def annotate(bgr, proj, people, others, status, offline):
    import cv2
    out = bgr.copy()
    h, w = out.shape[:2]
    if proj is not None:
        for u, v, r in zip(proj['u'], proj['v'], proj['rng']):
            cv2.circle(out, (int(u), int(v)), 2, range_colour(r), -1)
    for d in others:
        x1, y1, x2, y2 = [int(v) for v in d['box']]
        cv2.rectangle(out, (x1, y1), (x2, y2), (150, 150, 150), 1)
        cv2.putText(out, f'{d["name"]} {d["conf"]:.2f}', (x1 + 2, max(y1 - 4, 10)),
                    cv2.FONT_HERSHEY_SIMPLEX, 0.4, (200, 200, 200), 1, cv2.LINE_AA)
    for p in people:
        x1, y1, x2, y2 = [int(v) for v in p['box']]
        col = (60, 200, 60) if p['fused'] else (0, 210, 255)
        cv2.rectangle(out, (x1, y1), (x2, y2), col, 2)
        text = (f'PERSON {p["conf"]:.2f}  {p["range"]:.1f} m' if p['fused'] else
                f'person {p["conf"]:.2f}  camera only')
        (tw, th), _ = cv2.getTextSize(text, cv2.FONT_HERSHEY_SIMPLEX, 0.5, 1)
        cv2.rectangle(out, (x1, max(y1 - th - 6, 0)), (x1 + tw + 4, max(y1, th + 6)), col, -1)
        cv2.putText(out, text, (x1 + 2, max(y1 - 4, th + 2)), cv2.FONT_HERSHEY_SIMPLEX, 0.5,
                    (0, 0, 0), 1, cv2.LINE_AA)
    if offline:
        cv2.rectangle(out, (0, 0), (w, 30), (0, 0, 200), -1)
        cv2.putText(out, f'DETECTOR OFFLINE  ({SERVER})  - start eye_server.py on the Mac', (8, 21),
                    cv2.FONT_HERSHEY_SIMPLEX, 0.5, (255, 255, 255), 1, cv2.LINE_AA)
    else:
        cv2.rectangle(out, (0, h - 22), (w, h), (30, 30, 30), -1)
        cv2.putText(out, status, (8, h - 7), cv2.FONT_HERSHEY_SIMPLEX, 0.45, (230, 230, 230), 1,
                    cv2.LINE_AA)
    return out


# ---------------------------------------------------------------- detector client
def why_offline(err, server=SERVER):
    """What a failed request most likely means, in plain words."""
    e = str(err).lower()
    host = server.split('//')[-1].split(':')[0]
    if 'refused' in e:
        return f'{host} answered but nothing listens on the port: start eye_server.py on the Mac'
    if 'timed out' in e or 'timeout' in e:
        return (f'{host} did not answer: the Mac\'s firewall is blocking Python, or {host} is not '
                f'the Mac (check: ip route | grep default)')
    if 'no route' in e or 'unreachable' in e:
        return f'no network path to {host}: is the VM on UTM\'s shared network?'
    return 'see the error above'


def detect(jpeg, server=SERVER, timeout=TIMEOUT):
    """(reply dict, round-trip ms). Raises on any failure."""
    t0 = time.perf_counter()
    req = urllib.request.Request(server.rstrip('/') + '/detect', data=jpeg,
                                 headers={'Content-Type': 'image/jpeg'})
    with urllib.request.urlopen(req, timeout=timeout) as r:
        out = json.loads(r.read())
    return out, 1000.0 * (time.perf_counter() - t0)


def image_to_bgr(msg):
    """sensor_msgs/Image (rgb8 / bgr8) -> H x W x 3 BGR array, or None."""
    enc = msg.encoding.lower()
    if enc not in ('rgb8', 'bgr8'):
        return None
    a = np.frombuffer(bytes(msg.data), dtype=np.uint8).reshape(msg.height, msg.step)[:, :msg.width * 3]
    a = a.reshape(msg.height, msg.width, 3)
    return a[:, :, ::-1].copy() if enc == 'rgb8' else a.copy()


# ---------------------------------------------------------------- ROS node
def main():
    import cv2
    import rclpy
    from rclpy.node import Node
    from rclpy.parameter import Parameter
    from rclpy.qos import qos_profile_sensor_data
    from rclpy.time import Time
    from builtin_interfaces.msg import Time as TimeMsg
    from geometry_msgs.msg import Point, Pose, PoseArray
    from sensor_msgs.msg import CameraInfo, Image, LaserScan
    from visualization_msgs.msg import Marker, MarkerArray
    from tf2_ros import Buffer, TransformListener
    try:
        from rclpy.executors import ExternalShutdownException
    except ImportError:
        ExternalShutdownException = KeyboardInterrupt
    try:
        from vision_msgs.msg import Detection2D, Detection2DArray, ObjectHypothesisWithPose
    except ImportError:
        Detection2D = Detection2DArray = ObjectHypothesisWithPose = None

    label = sys.argv[1] if len(sys.argv) > 1 and sys.argv[1] else 'eye_last'

    class Eye(Node):
        def __init__(self):
            super().__init__('eye', parameter_overrides=[
                Parameter('use_sim_time', Parameter.Type.BOOL, True)])
            self.buf = Buffer()
            self.tfl = TransformListener(self.buf, self)
            self.k = None
            self.scans = collections.deque(maxlen=20)
            self.latest, self.new = None, threading.Event()
            self.cam_from_lidar = None
            self.cam_height, self.cam_offset = None, (0.25, 0.0)   # camera above / ahead of base_link
            self.online, self.offline_since_try = None, 0.0
            self.said = set()
            self.stats = collections.deque(maxlen=200)     # (wall, sim, trip_ms, persons, fused)
            self.last_status = time.monotonic()
            self.create_subscription(Image, '/camera/image_raw', self.on_image, qos_profile_sensor_data)
            self.create_subscription(CameraInfo, '/camera/camera_info', self.on_info, qos_profile_sensor_data)
            self.create_subscription(LaserScan, '/scan', self.on_scan, qos_profile_sensor_data)
            self.pub_img = self.create_publisher(Image, '/eye/image_annotated', 2)
            self.pub_people = self.create_publisher(MarkerArray, '/eye/people', 2)
            self.pub_persons = self.create_publisher(PoseArray, '/eye/persons', 5)
            self.scene = load_scene()
            self.pub_det = (self.create_publisher(Detection2DArray, '/eye/detections', 5)
                            if Detection2DArray is not None else None)
            out_dir = os.path.expanduser('~/runs/eye')
            os.makedirs(out_dir, exist_ok=True)
            self.log_path = os.path.join(out_dir, f'{label}.csv')
            self.log_file = open(self.log_path, 'w', newline='')
            self.log = csv.writer(self.log_file)
            self.log.writerow(LOG_FIELDS + [f'{o["name"]}_{c}' for o in self.scene
                                            for c in ('view', 'range', 'hit')])
            self.rows = 0
            self.stop = False
            self.worker = threading.Thread(target=self.loop, daemon=True)
            self.worker.start()
            print(f'eye {VERSION}: camera -> {SERVER} (YOLO on the Mac) -> fused with /scan; '
                  f'log {self.log_path}', flush=True)
            if self.scene:
                print('eye: scoring the scene: ' + ', '.join(f'{o["name"]} ({o["kind"]}) at ({o["x"]:.2f}, '
                                                          f'{o["y"]:.2f})' for o in self.scene), flush=True)

        def once(self, key, text):
            if key not in self.said:
                self.said.add(key)
                print(text, flush=True)

        def on_image(self, msg):
            self.latest = msg
            self.new.set()

        def on_info(self, msg):
            self.k = list(msg.k)

        def on_scan(self, msg):
            t = msg.header.stamp.sec + msg.header.stamp.nanosec * 1e-9
            self.scans.append((t, msg))

        def tf(self, target, source, stamp=None):
            for when in ((stamp,) if stamp is not None else ()) + (Time(),):
                try:
                    tr = self.buf.lookup_transform(target, source, when)
                    return transform_matrix(tr.transform.translation, tr.transform.rotation)
                except Exception:
                    continue
            return None

        # -- the worker thread: newest frame -> detector -> fuse -> publish
        def loop(self):
            last_send = 0.0
            while not self.stop:
                if not self.new.wait(timeout=0.5):
                    continue
                wait = 1.0 / RATE - (time.monotonic() - last_send)
                if wait > 0:
                    time.sleep(wait)
                self.new.clear()
                msg = self.latest
                if msg is None:
                    continue
                last_send = time.monotonic()
                try:
                    self.process(msg)
                except Exception as e:          # never die: report once, keep going
                    self.once(f'err-{type(e).__name__}', f'eye: ERROR {type(e).__name__}: {e}')

        def process(self, msg):
            t = msg.header.stamp.sec + msg.header.stamp.nanosec * 1e-9
            bgr = image_to_bgr(msg)
            if bgr is None:
                self.once('enc', f'eye: camera encoding {msg.encoding} not supported (rgb8/bgr8 expected)')
                return
            h, w = bgr.shape[:2]
            cam = intrinsics(self.k, w, h)
            ok_jpg, jpg = cv2.imencode('.jpg', bgr, [cv2.IMWRITE_JPEG_QUALITY, JPEG_QUALITY])
            reply, trip, offline = None, None, False
            if self.online is False and time.monotonic() - self.offline_since_try < RETRY_OFFLINE:
                offline = True
            else:
                try:
                    reply, trip = detect(jpg.tobytes())
                    if self.online is not True:
                        print(f'[{t:8.1f}] eye: detector online ({SERVER}, {reply.get("ms", 0):.0f} ms)',
                              flush=True)
                    self.online = True
                except (urllib.error.URLError, OSError, ValueError) as e:
                    if self.online is not False:
                        print(f'[{t:8.1f}] eye: detector OFFLINE ({SERVER}: {e}): {why_offline(e)}. '
                              f'Retrying every {RETRY_OFFLINE:.0f} s.', flush=True)
                    self.online, offline = False, True
                    self.offline_since_try = time.monotonic()
            # lidar: the scan closest in time, projected into the image
            proj, scan_dt, scan = None, None, None
            scans = list(self.scans)               # a copy: the ROS thread keeps appending
            if scans:
                st, scan = min(scans, key=lambda s: abs(s[0] - t))
                scan_dt = st - t
                if abs(scan_dt) > SCAN_SYNC:
                    scan = None
            if scan is not None:
                if self.cam_from_lidar is None:
                    self.cam_from_lidar = self.tf(msg.header.frame_id or CAM_FRAME, scan.header.frame_id)
                    floor = self.tf('base_link', msg.header.frame_id or CAM_FRAME)
                    if floor is not None:
                        self.cam_height = float(floor[2, 3])
                        self.cam_offset = (float(floor[0, 3]), float(floor[1, 3]))
                if self.cam_from_lidar is not None:
                    proj = project_scan(scan.ranges, scan.angle_min, scan.angle_increment,
                                        scan.range_min, scan.range_max, self.cam_from_lidar, cam, w, h)
                else:
                    self.once('tf', f'eye: no TF {msg.header.frame_id} <- {scan.header.frame_id} yet '
                                    f'(is robot_state_publisher running the camera URDF?)')
            map_from_lidar = robot = None
            if scan is not None:
                stamp = Time(seconds=scan.header.stamp.sec, nanoseconds=scan.header.stamp.nanosec)
                map_from_lidar = self.tf('map', scan.header.frame_id, stamp)
                base = self.tf('map', 'base_link', stamp)
                if base is not None:
                    robot = (float(base[0, 3]), float(base[1, 3]), math.atan2(base[1, 0], base[0, 0]))
            dets = reply.get('dets', []) if reply else []
            people, others = analyse(dets, proj, cam, self.cam_height or 0.6, map_from_lidar, h)
            fused = [p for p in people if p['fused']]
            # status line on the image
            now = time.monotonic()
            self.stats.append((now, t, trip, len(people), len(fused)))
            fps, rtf = self.rates()
            status = (f'eye {VERSION}  YOLO {reply.get("ms", 0):.0f} ms on Mac, {fps:.1f} fps  |  '
                      f'people {len(people)} ({len(fused)} with lidar)  |  sim speed {rtf:.2f}x'
                      if reply else '')
            out = annotate(bgr, proj, people, others, status, offline)
            self.publish_image(msg, out)
            self.publish_detections(msg, dets, cam)
            self.publish_people(people, robot, map_from_lidar, cam)
            self.publish_persons(msg, people)
            self.write_row(t, reply, trip, dets, people, robot, map_from_lidar, cam, scan, scan_dt, others)
            if now - self.last_status >= STATUS_EVERY:
                self.last_status = now
                trips = [s[2] for s in self.stats if s[2] is not None]
                print(f'[{t:8.1f}] eye: {fps:.1f} frames/s, detector round trip '
                      f'{np.median(trips) if trips else float("nan"):.0f} ms, people seen in '
                      f'{100 * np.mean([s[3] > 0 for s in self.stats]):.0f} % of frames, '
                      f'{sum(s[4] for s in self.stats)} of {sum(s[3] for s in self.stats)} located by the lidar, '
                      f'sim speed {rtf:.2f}x', flush=True)

        def rates(self):
            s = [x for x in self.stats if x[0] > self.stats[-1][0] - 20.0]
            if len(s) < 2 or s[-1][0] - s[0][0] < 1e-6:
                return 0.0, 0.0
            dw = s[-1][0] - s[0][0]
            return (len(s) - 1) / dw, (s[-1][1] - s[0][1]) / dw

        # -- outputs
        def publish_image(self, src, bgr):
            m = Image()
            m.header = src.header
            m.height, m.width = bgr.shape[0], bgr.shape[1]
            m.encoding, m.is_bigendian, m.step = 'rgb8', 0, bgr.shape[1] * 3
            m.data = bgr[:, :, ::-1].tobytes()
            self.pub_img.publish(m)

        def publish_detections(self, src, dets, cam):
            if self.pub_det is None:
                self.once('vm', 'eye: vision_msgs not found, /eye/detections is not published')
                return
            arr = Detection2DArray()
            arr.header = src.header
            for d in dets:
                x1, y1, x2, y2 = d['box']
                det = Detection2D()
                det.header = src.header
                c = det.bbox.center
                if hasattr(c, 'position'):           # vision_msgs 4 (Humble)
                    c.position.x, c.position.y = (x1 + x2) / 2.0, (y1 + y2) / 2.0
                else:                                # vision_msgs 3
                    c.x, c.y = (x1 + x2) / 2.0, (y1 + y2) / 2.0
                det.bbox.size_x, det.bbox.size_y = float(x2 - x1), float(y2 - y1)
                hyp = ObjectHypothesisWithPose()
                if hasattr(hyp, 'hypothesis'):
                    hyp.hypothesis.class_id, hyp.hypothesis.score = d['name'], float(d['conf'])
                else:
                    hyp.id, hyp.score = d['name'], float(d['conf'])
                det.results.append(hyp)
                arr.detections.append(det)
            self.pub_det.publish(arr)

        def publish_people(self, people, robot, map_from_lidar, cam):
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
            for k, p in enumerate(people):
                if p['fused'] and p['xy_map'] is not None:
                    x, y = p['xy_map']
                    body = marker('eye_person', k, Marker.CYLINDER)
                    body.pose.position.x, body.pose.position.y, body.pose.position.z = x, y, 0.9
                    body.scale.x = body.scale.y = 0.55
                    body.scale.z = 1.8
                    body.color.r, body.color.g, body.color.b, body.color.a = 0.25, 0.85, 0.3, 0.35
                    ma.markers.append(body)
                    text = marker('eye_label', k, Marker.TEXT_VIEW_FACING)
                    text.pose.position.x, text.pose.position.y, text.pose.position.z = x, y, 2.25
                    text.scale.z = 0.25
                    text.color.r, text.color.g, text.color.b, text.color.a = 0.3, 1.0, 0.4, 1.0
                    text.text = f'PERSON {p["conf"]:.2f} (camera + lidar)'
                    ma.markers.append(text)
                elif robot is not None:
                    # camera only: a line from the robot towards the person, 4 m long
                    b = camera_bearing((p['box'][0] + p['box'][2]) / 2.0, cam) + robot[2]
                    ray = marker('eye_ray', k, Marker.LINE_STRIP)
                    ray.scale.x = 0.04
                    ray.color.r, ray.color.g, ray.color.b, ray.color.a = 1.0, 0.8, 0.0, 0.9
                    ray.points = [Point(x=robot[0], y=robot[1], z=0.6),
                                  Point(x=robot[0] + 4.0 * math.cos(b), y=robot[1] + 4.0 * math.sin(b), z=0.6)]
                    ma.markers.append(ray)
            self.pub_people.publish(ma)

        def publish_persons(self, src, people):
            """Every person both sensors agree on, map frame, stamped with the image."""
            arr = PoseArray()
            arr.header.frame_id, arr.header.stamp = 'map', src.header.stamp
            for p in people:
                if p['fused'] and p['xy_map'] is not None:
                    q = Pose()
                    q.position.x, q.position.y = float(p['xy_map'][0]), float(p['xy_map'][1])
                    q.orientation.w = 1.0
                    arr.poses.append(q)
            self.pub_persons.publish(arr)

        def write_row(self, t, reply, trip, dets, people, robot, map_from_lidar, cam, scan, scan_dt, others):
            f = (lambda v: '' if v is None else f'{v:.3f}')
            best = people[0] if people else None
            tx, ty = worker_truth(t)
            truth_b = truth_r = None
            in_view = lidar_sees = err = berr = ''
            if robot is not None:
                # bearing and range from the camera (ahead of the robot's centre), in its heading
                ox, oy = self.cam_offset
                cxm = robot[0] + ox * math.cos(robot[2]) - oy * math.sin(robot[2])
                cym = robot[1] + ox * math.sin(robot[2]) + oy * math.cos(robot[2])
                dx, dy = tx - cxm, ty - cym
                truth_r = math.hypot(dx, dy)
                truth_b = math.atan2(math.sin(math.atan2(dy, dx) - robot[2]), math.cos(math.atan2(dy, dx) - robot[2]))
                half = math.atan2(cam[2], cam[0])            # half the horizontal field of view
                in_view = str(int(abs(truth_b) < half - 0.09 and 0.8 < truth_r < 8.0))
                if scan is not None and map_from_lidar is not None:
                    r = np.asarray(scan.ranges, dtype=float)
                    a = scan.angle_min + scan.angle_increment * np.arange(len(r))
                    ok = np.isfinite(r) & (r > scan.range_min) & (r < scan.range_max)
                    pts = np.column_stack((r[ok] * np.cos(a[ok]), r[ok] * np.sin(a[ok]),
                                           np.zeros(int(ok.sum())), np.ones(int(ok.sum()))))
                    pm = (map_from_lidar @ pts.T).T if len(pts) else np.zeros((0, 4))
                    lidar_sees = str(int(bool(len(pm)) and float(np.min(np.hypot(pm[:, 0] - tx, pm[:, 1] - ty))) < 0.35))
                if best is not None and best['xy_map'] is not None:
                    err = f'{math.hypot(best["xy_map"][0] - tx, best["xy_map"][1] - ty):.3f}'
                if best is not None:
                    b = camera_bearing((best['box'][0] + best['box'][2]) / 2.0, cam)
                    berr = f'{math.degrees(abs(math.atan2(math.sin(b - truth_b), math.cos(b - truth_b)))):.2f}'
            px = py = pr = pb = None
            if best is not None and best['fused']:
                px, py = best['xy_map'] if best['xy_map'] is not None else (None, None)
                pr = best['range']
                pb = math.degrees(math.atan2(best['xy_lidar'][1], best['xy_lidar'][0]))
            self.log.writerow([
                f'{t:.2f}', f'{time.time():.3f}', str(int(reply is not None)),
                f(reply.get('ms') if reply else None), f(trip), str(len(dets)), str(len(people)),
                str(sum(p['fused'] for p in people)), str(sum(not p['fused'] for p in people)),
                f(px), f(py), f(pr), f(pb), f(best['conf'] if best else None),
                str(best['points'] if best else ''), f(best['feet'] if best else None),
                f(robot[0] if robot else None), f(robot[1] if robot else None), f(robot[2] if robot else None),
                f(tx), f(ty), f(math.degrees(truth_b) if truth_b is not None else None), f(truth_r),
                in_view, lidar_sees, err, berr, f(scan_dt),
                ';'.join(f'{d["name"]}:{d["conf"]:.2f}' for d in others)]
                + scene_cols(self.scene, robot, self.cam_offset, cam, people if reply else []))
            self.rows += 1
            if self.rows % 20 == 0:
                self.log_file.flush()

        def close(self):
            self.stop = True
            try:
                self.log_file.close()
            except Exception:
                pass

    rclpy.init()
    node = Eye()
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
