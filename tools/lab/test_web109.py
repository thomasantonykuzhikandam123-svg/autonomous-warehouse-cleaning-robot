#!/usr/bin/env python3
"""test_web109.py <robot_web.py> [runs_dir label] - the Mission Control page (robot_web.py v10.9).

    python3 ~/ros2_ws/src/tools/lab/test_web109.py ~/ros2_ws/src/tools/robot_web.py [~/runs shift_v108a]

No ROS, no network, nothing on port 8080 (it uses a free port):
  A  the pure parts: words heard by a microphone, markers -> shapes (every type the guard and
     the traffic tab draw, DELETE / DELETEALL), the painted floor, the one-line status.
  B  the live side with a stand-in for rclpy / tf2 / the messages: every callback is fed a
     message shaped like the real one (the map, scan and pose from your recording when there is
     one), then the page's state is checked, and a typed command, a question and STOP go through
     the robot agent to /mission_command and /dialogue (the talk tab's turns show, the page's own
     are not shown twice).
  C  the web server: the page, the state, the map, the floor, the Mac helper with this VM's
     address in it, talk and STOP over HTTP; v10.12: the 3D view (/twin) and its three.js; v10.14: Rewind
     (live and replayed stops, /api/stops, /api/stop, the questions); v10.13: the
     wet floor (state, /api/wet, 'where is the floor wet?').
  D  a recorded shift (if runs_dir/rec/<label>.npz is there): replayed through the real people
     guard at 40x for a few seconds: shapes, the guard's decision and the job appear.
Ends with ALL OK, or says which check failed.
"""
import importlib.util
import json
import math
import os
import sys
import threading
import time
import types
import urllib.request
from types import SimpleNamespace as NS

import numpy as np

FAILS = []


def check(cond, what):
    print(('  ok   ' if cond else '  FAIL ') + what, flush=True)
    if not cond:
        FAILS.append(what)


def load(path):
    spec = importlib.util.spec_from_file_location('robot_web_t', path)
    m = importlib.util.module_from_spec(spec)
    spec.loader.exec_module(m)
    return m


# ---------------------------------------------------------------- message stand-ins
def P(x=0.0, y=0.0, z=0.0):
    return NS(x=x, y=y, z=z)


def Q(yaw=0.0):
    return NS(x=0.0, y=0.0, z=math.sin(yaw / 2), w=math.cos(yaw / 2))


def C(r, g, b, a):
    return NS(r=r, g=g, b=b, a=a)


def marker(ns, mid, mtype, x=0.0, y=0.0, yaw=0.0, sx=0.1, sy=0.1, sz=0.1, rgba=(1, 1, 1, 1), pts=(), text='',
           action=0, frame='map'):
    return NS(header=NS(frame_id=frame), ns=ns, id=mid, type=mtype, action=action,
              pose=NS(position=P(x, y, 0.5), orientation=Q(yaw)), scale=NS(x=sx, y=sy, z=sz),
              color=C(*rgba), points=[P(a, b, 0.05) for a, b in pts], colors=[], text=text)


def fake_ros():
    """Just enough of rclpy, tf2_ros and the message packages for robot_web's run_ros."""
    st = NS(nodes=[], pose={}, sim=[100.0], spun=[])

    class Node:
        def __init__(self, name, parameter_overrides=None):
            self.name, self.subs, self.pubs, self.timers = name, {}, {}, []
            st.nodes.append(self)

        def create_subscription(self, typ, topic, cb, qos):
            self.subs[topic] = cb

        def create_publisher(self, typ, topic, qos):
            pub = NS(topic=topic, sent=[])
            pub.publish = lambda m, pub=pub: pub.sent.append(m)
            self.pubs[topic] = pub
            return pub

        def create_timer(self, period, cb):
            self.timers.append(cb)

        def get_clock(self):
            return NS(now=lambda: NS(nanoseconds=int(st.sim[0] * 1e9)))

        def count_subscribers(self, topic):
            return 2 if topic == '/emergency_stop' else 1       # the gate, and this node itself

        def destroy_node(self):
            pass

    class Parameter:
        Type = NS(BOOL=1)

        def __init__(self, *a):
            pass

    class Buffer:
        def lookup_transform(self, target, frame, when):
            if frame not in st.pose:
                raise RuntimeError(f'no {frame}')
            x, y, yaw = st.pose[frame]
            return NS(transform=NS(translation=P(x, y, 0.0), rotation=Q(yaw)))

    class String:
        def __init__(self, data=''):
            self.data = data

    class Bool:
        def __init__(self, data=False):
            self.data = data

    def mod(name, **kw):
        m = types.ModuleType(name)
        m.__dict__.update(kw)
        sys.modules[name] = m
        return m
    mod('rclpy', init=lambda *a, **k: None, spin=lambda n: st.spun.append(n), ok=lambda: True,
        shutdown=lambda: None)
    mod('rclpy.node', Node=Node)
    mod('rclpy.parameter', Parameter=Parameter)
    mod('rclpy.qos', DurabilityPolicy=NS(TRANSIENT_LOCAL=1), ReliabilityPolicy=NS(RELIABLE=1),
        QoSProfile=lambda **k: k, qos_profile_sensor_data='sensor')
    mod('rclpy.time', Time=lambda *a, **k: 0)
    mod('nav_msgs')
    mod('nav_msgs.msg', OccupancyGrid=object, Path=object)
    mod('sensor_msgs')
    mod('sensor_msgs.msg', LaserScan=object, Image=object)
    mod('std_msgs')
    mod('std_msgs.msg', String=String, Bool=Bool)
    mod('tf2_ros', Buffer=Buffer, TransformListener=lambda buf, node: None)
    mod('visualization_msgs')
    mod('visualization_msgs.msg', MarkerArray=object)
    return st


def http(port, path, body=None):
    req = urllib.request.Request(f'http://127.0.0.1:{port}{path}',
                                 data=None if body is None else json.dumps(body).encode(),
                                 headers={'Content-Type': 'application/json', 'Host': '10.11.12.13:8080'})
    with urllib.request.urlopen(req, timeout=10) as r:
        return r.status, r.read(), r.headers


def http_any(port, path):
    """GET that returns (code, body, headers) for any status, 404 included."""
    import urllib.error
    try:
        return http(port, path)
    except urllib.error.HTTPError as e:
        return e.code, e.read(), e.headers


def wait_for(cond, timeout=15.0):
    t = time.time()
    while time.time() - t < timeout:
        if cond():
            return True
        time.sleep(0.05)
    return False


def main(argv):
    if not argv:
        print(__doc__)
        return 2
    path = os.path.abspath(os.path.expanduser(argv[0]))
    runs = os.path.expanduser(argv[1]) if len(argv) > 1 else os.path.expanduser('~/runs')
    label = argv[2] if len(argv) > 2 else 'shift_v108a'
    rec = os.path.join(runs, 'rec', f'{label}.npz')
    have_rec = os.path.exists(rec)
    W = load(path)
    print(f'test_web109: {path} ({W.VERSION}); recording: {rec if have_rec else "none (D skipped)"}')

    # ------------------------------------------------------------ A: pure parts
    print('A  the pure parts')
    vf = W.voice_fix
    check(vf("clean I'll two") == 'clean aisle 2' and vf('clean isle 3') == 'clean aisle 3'
          and vf('clean aisle to') == 'clean aisle 2' and vf('Clean aisles one') == 'Clean aisle 1',
          "speech mishearings of 'aisle N' are put right")
    check(vf("I'll be back") == "I'll be back" and vf('why did you stop?') == 'why did you stop?'
          and vf('go to the front') == 'go to the front', 'and nothing else is touched')
    check(vf('Clean Hazel three') == 'Clean aisle 3' and vf('Please L1') == 'clean aisle 1'
          and vf('Clean Al two') == 'Clean aisle 2' and vf('Clean bacteria after this') == 'Clean back area after this'
          and vf('Veetu') == 'go home', "v10.10: the rehearsal's mishearings ('Hazel three', 'Please L1', 'bacteria')")
    check(W.pick_heard(['Clean Hazel tree', 'clean aisle three'], lambda t: 1 if 'aisle 3' in t else None)
          == 'Clean aisle 3' and W.pick_heard(['hello there', 'clean aisle 2'], lambda t: 1 if 'aisle 2' in t else None)
          == 'clean aisle 2', "of the recogniser's guesses, the first that is a command wins")
    day = ['Please Swan', 'Please Awan', 'Clean Awan', 'Play Selvan', 'Play Awan', 'Please iron']
    check(all(vf(t) == 'clean aisle 1' for t in day),
          f"v10.10b: the demo morning's 'clean aisle one' as Safari wrote it: {[vf(t) for t in day]}")
    check(vf('Clean Altu').lower() == 'clean aisle 2' and vf('Please Eltree') == 'clean aisle 3' and vf('first aisle')
          == 'clean aisle 1' and vf("third I'll") == 'clean aisle 3' and vf('aisle 2') == 'clean aisle 2'
          and vf('clean row two') == 'clean aisle 2', "and aisles 2 and 3 alike; 'first aisle', 'aisle 2' alone")
    keep = ['please explain', 'please carry on', 'go on', 'please stop', 'thank you', 'clean aisle 2 and 3',
            'please do', 'what do you see?', 'how long left?', 'introduce yourself', 'Clean Hazel', 'Swan',
            'one second', 'Hello', 'go home', 'clean the front area']
    check(all(vf(t) == t for t in keep), f"and nothing else is touched: {[(t, vf(t)) for t in keep if vf(t) != t]}")
    vp = W.view_polygon([float('inf'), 2.0, 0.05, float('nan')], -math.pi, math.pi / 2, 0.3, 8.0, 1.0, 2.0, 0.0, step=1)
    d = [math.hypot(x - 1.0, y - 2.0) for x, y in vp['pts']]
    check(vp['o'] == [1.0, 2.0] and abs(d[0] - 8.0) < 0.02 and abs(d[1] - 2.0) < 0.02 and abs(d[2] - 0.3) < 0.02
          and abs(d[3] - 8.0) < 0.02, "v10.11: the lidar's view: no return = clear to 8 m, under range_min = blocked at once")
    zq, za = W.zone_question, W.zone_answer
    check(zq('Clean Hazel') == (W.ASK_ZONE, None) and zq('Clean') == (W.ASK_ZONE, None)
          and zq('Swan')[1] == 'clean aisle 1' and 'first aisle' in zq('Swan')[0]
          and all(zq(t) is None for t in ('please explain', 'carry on', 'why did you stop', 'hello', 'please do',
                                          'go home', 'thank you', 'what do you see')),
          "v10.10b: an order with its zone lost ('Clean Hazel') is asked about; 'Swan' alone: the first aisle?")
    check(za('the second') == 'clean aisle 2' and za('yes', 'clean aisle 1') == 'clean aisle 1' and za('yes') is None
          and za('front') == 'clean the front area' and za('Swan') == 'clean aisle 1' and za('third aisle')
          == 'clean aisle 3' and za('stop') is None and za('why') is None, 'and the answer is taken: the second, yes, front, Swan')
    check(W._JSONISH.search('{"plan":[{"action":"stop","zones":[]},{"action":"clean","zones":["Hazel"]}],'
                            '"say:"Stopping, then doing aisle 3."}') and not W._JSONISH.search("I can't do that."),
          "broken JSON from the model is recognised (it is never shown)")
    ms = W.MarkerSet()
    ms.apply([marker('x', 0, 3, action=3),
              marker('person', 4, 3, x=1.0, y=2.0, sx=0.5, sy=0.5, sz=1.7, rgba=(0.9, 0.1, 0.1, 0.7)),
              marker('predicted_path', 40, 4, pts=[(1, 2), (1.5, 2.5), (2, 3)], sx=0.05),
              marker('vehicle', 7, 1, x=3.0, y=-2.0, yaw=0.5, sx=1.2, sy=1.0),
              marker('label', 4, 9, x=1.0, y=2.0, text='person 4  0.8 m/s  (robot waits)'),
              marker('clean', 0, 6, pts=[(0, 0), (0.05, 0)], sx=0.05),
              marker('arrow', 1, 0, x=0.0, y=0.0, yaw=math.pi / 2, sx=1.0, sy=0.1),
              marker('walk', 9, 5, pts=[(0, 0), (1, 0), (2, 0), (3, 0)], sx=0.5),
              marker('mesh', 1, 10)])
    sh = {(s['ns'], s['id']): s for s in ms.shapes()}
    p = sh.get(('person', 4), {})
    check(p.get('k') == 'c' and abs(p['x'] - 1.0) < 1e-6 and abs(p['r'] - 0.25) < 1e-6 and p['rgba'][0] == 0.9,
          'a CYLINDER (a person) becomes a circle of the right size and colour')
    check(sh.get(('predicted_path', 40), {}).get('pts') == [[1, 2], [1.5, 2.5], [2, 3]]
          and sh.get(('vehicle', 7), {}).get('k') == 'b' and abs(sh[('vehicle', 7)]['yaw'] - 0.5) < 1e-6
          and sh.get(('label', 4), {}).get('text', '').startswith('person 4')
          and sh.get(('clean', 0), {}).get('k') == 'p' and sh.get(('walk', 9), {}).get('k') == 'll',
          'LINE_STRIP, CUBE (turned), TEXT, CUBE_LIST and LINE_LIST come through')
    a = sh.get(('arrow', 1), {}).get('pts')
    check(a is not None and abs(a[1][0]) < 1e-3 and abs(a[1][1] - 1.0) < 1e-3, 'an ARROW points along its yaw')
    check(('mesh', 1) not in sh, 'a MESH is left out (nothing to draw in 2D)')
    ms.apply([marker('person', 4, 3, action=2)])
    check(('person', 4) not in {(s['ns'], s['id']) for s in ms.shapes()} and len(ms.shapes()) == 6,
          'DELETE removes one marker')
    ms.apply([marker('', 0, 0, action=3), marker('person', 5, 3, x=0.0, y=0.0, sx=0.5)])
    check(len(ms.shapes()) == 1, 'DELETEALL clears the rest (as the guard sends each scan)')
    odd = W.marker_shape(marker('person', 6, 3, x=1.0, y=0.0, sx=0.5, frame='base_link'),
                         lambda f: (2.0, 3.0, math.pi / 2) if f == 'base_link' else None)
    check(odd is not None and abs(odd['x'] - 2.0) < 1e-6 and abs(odd['y'] - 4.0) < 1e-6,
          'a marker in another frame is put into the map with TF')
    pa = W.Painter({'w': 100, 'h': 100, 'res': 0.05, 'ox': 0.0, 'oy': 0.0})
    for i in range(31):                                  # 3 m at 10 Hz and 1 m/s
        pa.paint(1.0 + 0.1 * i, 2.5)
    swept = int((pa.grid > 0).sum())
    check(abs(swept * 0.0025 - (3.0 * 0.64 + math.pi * 0.32 ** 2)) < 0.15,
          f'the painted floor is a 0.64 m brush along the drive ({swept * 0.0025:.2f} m2, '
          f'{3.0 * 0.64 + math.pi * 0.32 ** 2:.2f} expected)')
    pa.paint(4.0, 4.9)                                   # a 2.4 m jump (relocalised): a disc, no smear
    check(pa.grid[98, 80] > 0 and pa.grid[74, 80] == 0, 'no smear across a jump, and the map edge is respected')
    check(W.canned('introduce yourself', {}) and W.canned('Who are you?', {}) and not W.canned('what are you doing?', {})
          and not W.canned('who are you waiting for?', {}), "a scripted introduction, only when asked for one")
    check(W.brief({'zone': 'aisle_2', 'goal': 3, 'total': 9, 'coverage_pct': 41.6, 'queue': ['aisle_2', 'back']})
          == "I'm cleaning aisle 2, waypoint 3 of 9; 42 percent of the whole floor is done. After this: the back area.",
          'the one-line status (used when the model is down)')

    # ------------------------------------------------------------ B: the live side, ROS stood in for
    print('B  the live side (rclpy, tf2 and the messages stood in for)')
    ros = fake_ros()
    hub = W.Hub('live')
    args = NS(ollama='http://127.0.0.1:9', model='llama3.2', no_agent=False, no_camera=False)
    talker, shutdown = W.run_ros(hub, args)
    node = ros.nodes[-1]
    want = {'/map', '/coverage_passes', '/mission_state', '/people_guard/state', '/people_markers', '/plan',
            '/scan', '/traffic/truth', '/dialogue', '/eye/image_annotated'}
    check(want <= set(node.subs), f'it subscribes to what it shows ({len(node.subs)} topics)')
    check({'/mission_command', '/dialogue', '/emergency_stop'} <= set(node.pubs) and '/emergency_stop' in node.subs,
          'and publishes /mission_command, /dialogue and /emergency_stop')
    if have_rec:
        R = np.load(rec, allow_pickle=False)
        mi = json.loads(str(R['map_info']))
        grid, k = R['map_data'], 600
        robot = tuple(float(v) for v in R['robot'][k])
        laser = tuple(float(v) for v in R['laser'][k])
        ranges = R['ranges'][k]
        meta = json.loads(str(R['meta']))
    else:
        mi = {'width': 120, 'height': 160, 'resolution': 0.05, 'origin_x': -1.0, 'origin_y': -1.0}
        grid = np.zeros(120 * 160, np.int8)
        robot, laser = (1.0, 2.0, 0.3), (1.1, 2.0, 0.3)
        ranges = np.full(360, 3.0, np.float32)
        meta = {'angle_min': -math.pi, 'angle_increment': 2 * math.pi / 360, 'range_min': 0.3, 'range_max': 8.0}
    info = NS(width=mi['width'], height=mi['height'], resolution=mi['resolution'],
              origin=NS(position=P(mi['origin_x'], mi['origin_y'])))
    node.subs['/map'](NS(info=info, data=list(np.asarray(grid, dtype=np.int8))))
    ros.pose['base_link'] = robot
    ros.pose['lidar_link'] = laser
    for i in range(3):
        ros.sim[0] = 100.0 + 0.1 * i
        for cb in node.timers:
            cb()
    for _ in range(2):                                  # every 2nd scan is shown
        node.subs['/scan'](NS(header=NS(frame_id='lidar_link'), ranges=list(ranges),
                              angle_min=float(meta['angle_min']), angle_increment=float(meta['angle_increment']),
                              range_min=float(meta['range_min']), range_max=float(meta['range_max'])))
    node.subs['/plan'](NS(header=NS(frame_id='map'),
                          poses=[NS(pose=NS(position=P(robot[0] + 0.1 * i, robot[1]))) for i in range(40)]))
    node.subs['/people_markers'](NS(markers=[
        marker('', 0, 0, action=3),
        marker('person', 3, 3, x=robot[0] + 1.5, y=robot[1], sx=0.5, sy=0.5, sz=1.7, rgba=(0.9, 0.1, 0.1, 0.7)),
        marker('predicted_path', 30, 4, pts=[(robot[0] + 1.5, robot[1]), (robot[0] + 1.5, robot[1] + 1)]),
        marker('stop_line', 0, 4, pts=[(robot[0] + 0.5, robot[1] - 0.45), (robot[0] + 0.5, robot[1] + 0.45)]),
        marker('robot_decision', 0, 9, x=robot[0], y=robot[1], text='WAIT at stop line (person 3)')]))
    guard = {'t': 100.2, 'state': 'wait', 'why': 'crossing', 'version': 'v10.8', 'v_limit': None, 'limit': None,
             'say': 'waiting at a walkway for a person to cross (going now would pass 0.3 m from them)',
             'person_m': 1.5, 'person_id': 3, 'vehicle_m': None, 'vehicle_moving': False, 'people': 1,
             'vehicles': 0, 'standing': 0}
    mission = {'zone': 'aisle_1', 'goal': 3, 'total': 11, 'coverage_pct': 5.0, 'paused': False, 'done': False,
               'cleaned': [], 'queue': ['aisle_1'], 'partial': {}, 'asked': ['clean aisle_1'], 'guard': True,
               'awaiting_command': False, 'swept': {'aisle_1': 30.0},
               'traffic': {k: guard[k] for k in ('state', 'why', 'say', 'v_limit', 'limit', 'person_m',
                                                 'vehicle_m')} | {'for_s': 3.0}}
    node.subs['/mission_state'](NS(data=json.dumps(dict(mission, asked=[], goal=0, total=0, zone='start',
                                                        awaiting_command=True, queue=[]))))
    ros.sim[0] = 101.0
    node.subs['/mission_state'](NS(data=json.dumps(mission)))
    node.subs['/people_guard/state'](NS(data=json.dumps(guard)))
    heat = np.zeros(mi['width'] * mi['height'], np.int8)
    heat[:50] = 30
    node.subs['/coverage_passes'](NS(info=info, data=list(heat)))
    node.subs['/traffic/truth'](NS(data=json.dumps({'t': 101.0, 'agents': [
        {'name': 'worker', 'kind': 'person', 'x': 1.0, 'y': 4.0, 'yaw': 0.0, 'v': 1.0, 'r': 0.25},
        {'name': 'forklift', 'kind': 'vehicle', 'x': 2.6, 'y': -2.0, 'yaw': 1.57, 'v': 0.5, 'L': 1.2, 'W': 1.0,
         'fork': 1.1, 'state': 'go'}]})))
    img = np.zeros((48, 64, 3), np.uint8)
    img[:, :, 0] = 200
    node.subs['/eye/image_annotated'](NS(header=NS(stamp=NS(sec=101, nanosec=0)), height=48, width=64,
                                          encoding='rgb8', data=img.tobytes()))
    node.subs['/dialogue'](NS(data=json.dumps({'text': 'how much is left?', 'reply': 'About 70 percent of aisle 1.',
                                               'route': 'question', 'sent': None})))
    s = hub.state()
    check(s['robot'] is not None and abs(s['robot']['x'] - robot[0]) < 1e-3 and abs(s['robot']['yaw'] - robot[2]) < 1e-3,
          'the robot comes from TF map -> base_link')
    check(s['sim'] is not None and abs(s['sim'] - 100.2) < 0.01, 'simulation time from the node clock')
    check(s['scan'] is not None and len(s['scan']) > 20, f'lidar points in the map ({len(s["scan"] or [])})')
    if s['scan']:
        d = [math.hypot(x - laser[0], y - laser[1]) for x, y in s['scan']]
        r = np.asarray(ranges, float)
        r = r[np.isfinite(r) & (r > float(meta['range_min'])) & (r < float(meta['range_max']) * 0.995)]
        check(abs(min(d) - float(r.min())) < 0.05 and abs(max(d) - float(r.max())) < 0.05,
              'each lidar point is its range from the laser')
    check(s['plan'] is not None and len(s['plan']) == 40, 'the path it follows (/plan)')
    vw = s.get('view') or {}
    check(vw.get('pts') and len(vw['pts']) == len(ranges) // 2 and abs(vw['o'][0] - laser[0]) < 0.02
          and abs(vw['o'][1] - laser[1]) < 0.02, f"v10.11: what it can see now, from the lidar ({len(vw.get('pts') or [])} points)")
    ns = {x['ns'] for x in s['shapes']}
    check({'person', 'predicted_path', 'stop_line', 'robot_decision'} <= ns, 'what the guard sees and predicts')
    check(s['guard'] and s['guard']['state'] == 'wait' and s['mission']['zone'] == 'aisle_1',
          "the guard's decision and the job")
    check(len(s['holds']) == 1 and s['holds'][0][3] is True, 'a stop for a person is marked where the robot stands')
    says = [e['text'] for e in s['events'] if e['kind'] == 'say']
    check(any('aisle 1' in t for t in says) and any('walkway' in t for t in says),
          f'the narrator speaks: {says[:3]}')
    check(not any('walkway' in t for t in says[:1]), 'and says nothing about traffic before the first job')
    check(s['truth'] and {a['name'] for a in s['truth']} == {'worker', 'forklift'}, 'simulator truth (if ticked)')
    talk_tab = [e for e in s['events'] if e.get('via') == 'terminal']
    check(len(talk_tab) == 2 and talk_tab[1]['text'].startswith('About 70'), "the talk tab's turns are shown")
    check(s['cov_src'] == 'recorder' and json.loads(hub.coverage_json())['src'] == 'recorder',
          "the floor comes from coverage_recorder once it publishes")
    check(set(s['feeds']) >= {'map', 'pose', 'mission', 'guard', 'markers', 'scan', 'plan', 'coverage', 'truth',
                              'camera', 'dialogue'}, 'every feed is reported as arriving')
    jpg = hub.camera_jpeg()
    if hub.cv2 is not None:
        check(jpg is not None and jpg[:2] == b'\xff\xd8', 'the camera frame becomes a JPEG')
    else:
        print('  --   no cv2 here: the camera panel stays hidden (it has cv2 on the VM)')
    # talking: a command, a question, STOP
    cmd_pub, dlg_pub = node.pubs['/mission_command'], node.pubs['/dialogue']
    n0 = len(dlg_pub.sent)
    tid = talker.submit('clean aisle 2', 'typed')
    check(wait_for(lambda: any(e.get('id') == tid and e['kind'] == 'robot' for e in hub.state()['events'])),
          'a typed command is answered')
    sent = [json.loads(m.data) for m in cmd_pub.sent]
    check(sent and sent[-1].get('action') == 'clean' and sent[-1].get('zones') == ['aisle_2'],
          f'and reaches /mission_command: {sent[-1] if sent else None}')
    rep = [e for e in hub.state()['events'] if e.get('id') == tid and e['kind'] == 'robot'][0]
    check(rep['text'] == 'Cleaning aisle 2.' and rep.get('delivered') is True, f'reply: "{rep["text"]}"')
    rec_ = [json.loads(m.data) for m in dlg_pub.sent[n0:]]
    check(rec_ and rec_[-1].get('via') == 'web: typed' and rec_[-1].get('sent', {}).get('zones') == ['aisle_2'],
          'the turn is filed on /dialogue (via: web)')
    for m in dlg_pub.sent[n0:]:                          # it hears its own /dialogue back: not shown twice
        node.subs['/dialogue'](m)
    n_robot = sum(1 for e in hub.state()['events'] if e['kind'] == 'robot')
    check(n_robot == 2, "the page's own turn is not shown twice")
    tid = talker.submit('why did you stop?', 'voice')
    check(wait_for(lambda: any(e.get('id') == tid and e['kind'] == 'robot' for e in hub.state()['events'])),
          'a spoken question is answered')
    rep = [e for e in hub.state()['events'] if e.get('id') == tid and e['kind'] == 'robot'][0]
    if hub.agent.get('ok'):
        check('walkway' in rep['text'] and not rep.get('sent'),
              f'from the guard\'s state, nothing sent: "{rep["text"][:80]}"')
        tid = talker.submit('can you tell me a joke?', 'typed')     # v10.13: 'slippery' is answered in code now
        check(wait_for(lambda: any(e.get('id') == tid and e['kind'] == 'robot' for e in hub.state()['events']), 30),
              'an open question with the model down is still answered')
        rep = [e for e in hub.state()['events'] if e.get('id') == tid and e['kind'] == 'robot'][0]
        check(rep['text'].startswith("My language model on the Mac isn't answering") and 'aisle 1' in rep['text'],
              'with a plain status line instead of an error')
    else:
        print(f'  --   robot agent not loadable here ({hub.agent.get("why")}): commands only')
    tid = talker.submit('what do you see?', 'chip')
    check(wait_for(lambda: any(e.get('id') == tid and e['kind'] == 'robot' for e in hub.state()['events'])),
          'v10.10: "what do you see?" answered')
    rep = [e for e in hub.state()['events'] if e.get('id') == tid and e['kind'] == 'robot'][0]
    check(rep['text'].startswith('My lidar sees 1 person') and '1.5 m' in rep['text'],
          f'from the guard, in code: "{rep["text"][:70]}"')
    # v10.10b: asked which zone, then the answer; this morning's 'Please Swan'
    def turn_of(t, via):
        tid = talker.submit(t, via)
        wait_for(lambda: any(e.get('id') == tid and e['kind'] == 'robot' for e in hub.state()['events']), 30)
        ev = [e for e in hub.state()['events'] if e.get('id') == tid]
        return ([e for e in ev if e['kind'] == 'you'] or [{}])[0], ([e for e in ev if e['kind'] == 'robot'] or [{}])[0]
    n_cmd = len(cmd_pub.sent)
    you, rep = turn_of('Clean Hazel', 'voice')
    check(rep.get('text') == W.ASK_ZONE and len(rep.get('choices') or []) == 5 and not rep.get('sent')
          and len(cmd_pub.sent) == n_cmd, "v10.10b: 'Clean Hazel': which zone? with five buttons, nothing sent")
    you, rep = turn_of('the second', 'voice')
    check(you.get('heard') == 'clean aisle 2' and (rep.get('sent') or {}).get('zones') == ['aisle_2'],
          f"and 'the second' cleans aisle 2 ({rep.get('text', '')[:50]})")
    you, rep = turn_of('Please Swan', 'voice')
    check(you.get('heard') == 'clean aisle 1' and (rep.get('sent') or {}).get('zones') == ['aisle_1'],
          "'Please Swan' cleans aisle 1")
    you, rep = turn_of("Clean I'll two.", 'whisper')
    check(you.get('heard') == 'Clean aisle 2.' and (rep.get('sent') or {}).get('zones') == ['aisle_2'],
          "v10.11: what Whisper heard on the Mac goes through the same corrections ('Clean I'll two.')")
    if talker.graph is not None:
        import importlib
        ra, g0 = importlib.import_module('warehouse_planner.robot_agent'), talker.graph
        broken = ('{"plan":[{"action":"stop","zones":[]},{"action":"clean","zones":["Hazel"]}],'
                  '"say:"Stopping, then doing aisle 3."}')
        front = '{"plan":[{"action":"clean","zones":["front"]}],"say":"Cleaning the front."}'
        talker.graph = ra.build_graph(lambda system, text: broken)
        n_cmd = len(cmd_pub.sent)
        you, rep = turn_of('tidy up over by the racks', 'typed')
        check(rep.get('text', '').startswith('Sorry, I got muddled') and '{' not in rep.get('text', '')
              and rep.get('choices') and len(cmd_pub.sent) == n_cmd, "the model's broken JSON is not shown, nothing sent")
        talker.graph = ra.build_graph(lambda system, text: front)
        you, rep = turn_of('clean up the mess near the big racks please', 'typed')
        check(rep.get('text') == W.ASK_ZONE and not rep.get('sent') and len(cmd_pub.sent) == n_cmd,
              'a clean the model planned for words naming no zone is not sent: asked instead')
        you, rep = turn_of('sweep by the front door for me', 'typed')
        check((rep.get('sent') or {}).get('zones') == ['front'], 'but one naming the zone is sent')
        talker.graph = g0
    # v10.11: the model kept warm, under the exact name Ollama has it (a stand-in Ollama)
    import http.server as hs
    seen = []

    class FakeOllama(hs.BaseHTTPRequestHandler):
        def log_message(self, *a):
            pass

        def do_GET(self):
            b = json.dumps({'models': [{'name': 'llama3.2:3b'}, {'name': 'qwen2:1.5b'}]}).encode()
            self.send_response(200)
            self.send_header('Content-Length', str(len(b)))
            self.end_headers()
            self.wfile.write(b)

        def do_POST(self):
            d = json.loads(self.rfile.read(int(self.headers['Content-Length'])))
            seen.append((self.path, d))
            b = json.dumps({'message': {'content': 'hello'}} if self.path == '/api/chat' else {'done': True}).encode()
            self.send_response(200)
            self.send_header('Content-Length', str(len(b)))
            self.end_headers()
            self.wfile.write(b)
    fo = hs.ThreadingHTTPServer(('127.0.0.1', 0), FakeOllama)
    threading.Thread(target=fo.serve_forever, daemon=True).start()
    hub3 = W.Hub('live')
    t3 = W.Talker(hub3, lambda c: True, lambda d: None, f'http://127.0.0.1:{fo.server_address[1]}', 'llama3.2', agent=False)
    wait_for(lambda: any(p == '/api/generate' for p, _ in seen), 10)
    warm = [d for p, d in seen if p == '/api/generate']
    check(hub3.agent.get('model_up') and t3.model_tag == 'llama3.2:3b' and warm and warm[0]['prompt'] == ''
          and warm[0]['keep_alive'] == '30m' and warm[0]['model'] == 'llama3.2:3b',
          'v10.11: when the model comes up it is loaded at once and kept 30 min, by its exact name (llama3.2:3b)')
    ans = t3.ask('system', 'hi')
    chat = [d for p, d in seen if p == '/api/chat']
    check(ans == 'hello' and chat and chat[-1]['model'] == 'llama3.2:3b' and chat[-1]['keep_alive'] == '30m',
          'and every question keeps it loaded (keep_alive), so no answer waits for a model load')
    fo.shutdown()
    ctx = talker.context()
    check('LIVE FACTS' in ctx and 'In view: 1 people' in ctx and 'About me' in ctx,
          'the model gets the live situation with every question')
    n_cmd = len(cmd_pub.sent)
    es_pub = node.pubs['/emergency_stop']
    ok = talker.stop_now()
    check(ok and es_pub.sent and es_pub.sent[-1].data is True and hub.state()['estop']['on'],
          'STOP presses the emergency stop at the gate (/emergency_stop true), at once')
    check(len(cmd_pub.sent) == n_cmd + 1 and json.loads(cmd_pub.sent[-1].data)['action'] == 'stop',
          'and pauses the job (/mission_command stop: no goal left to time out), no model, no queue')
    node.subs['/emergency_stop'](es_pub.sent[-1])        # it hears its own press: no "from outside"
    n_es = len(es_pub.sent)
    node.estop_wall = time.time() - 3.0
    for cb in node.timers:
        cb()
    check(len(es_pub.sent) == n_es + 1 and es_pub.sent[-1].data is True, 'while held it says so again every 2 s')
    ok = talker.release()
    check(ok and es_pub.sent[-1].data is False and not hub.state()['estop']['on'], 'RELEASE: /emergency_stop false')
    rel = [e for e in hub.state()['events'] if e['kind'] == 'robot'][-1]['text']
    check(rel.startswith('Released') and not any('from outside' in e.get('text', '') for e in hub.state()['events']),
          f'"{rel[:60]}..."')
    node.estop_wall = time.time() - 5.0
    node.subs['/emergency_stop'](NS(data=True))            # pressed in a terminal
    st = hub.state()
    check(st['estop']['on'] and st['estop']['by'] == 'someone else'
          and any('from outside this page' in e.get('text', '') for e in st['events']),
          'a stop pressed outside the page shows on the page')
    node.subs['/emergency_stop'](NS(data=False))
    check(not hub.state()['estop']['on'], 'and its release too')
    hub.mission_wall = time.time() - 10.0                  # the cleaning tab is gone
    tid = talker.submit('go home', 'typed')
    check(wait_for(lambda: any(e.get('id') == tid and e['kind'] == 'robot' for e in hub.state()['events'])) and
          [e for e in hub.state()['events'] if e.get('id') == tid and e['kind'] == 'robot'][0].get('delivered') is False,
          'with no /mission_state for 5 s a command is shown as not delivered')
    hub.mission_wall = time.time()
    # v10.14: Rewind, live: the guard waits for a person; 4 s later, the future it avoided
    have_rw = os.path.exists(os.path.join(os.path.dirname(path), 'rewind.py'))
    rw_id = None
    if have_rw:
        rx, ry = robot[0], robot[1]
        ros.pose['base_link'] = (rx, ry, 0.0)
        node.subs['/plan'](NS(header=NS(frame_id='map'),
                              poses=[NS(pose=NS(position=P(rx + 0.1 * i, ry))) for i in range(60)]))
        t0 = 300.0
        g = dict(guard, state='wait', why='yield')
        rew_pub = node.pubs.get('/rewind/stop')
        for i in range(48):                              # the worker walks across 1.6 m ahead, 1.8 s after it stops
            t = t0 + 0.1 * i
            ros.sim[0] = t
            node.subs['/traffic/truth'](NS(data=json.dumps({'t': t, 'agents': [
                {'name': 'worker', 'kind': 'person', 'x': rx + 1.6, 'y': ry + 1.0 * (t - t0 - 1.8), 'yaw': 1.57, 'v': 1.0}]})))
            node.subs['/people_guard/state'](NS(data=json.dumps(dict(g, t=t))))
            for cb in node.timers:
                cb()
        stops = hub.rew.listing() if hub.rew is not None else []
        last = stops[-1] if stops else {}
        rw_id = last.get('id')
        check(last.get('cls') in ('hit', 'near') and last.get('who') == 'worker' and last.get('src') == 'truth',
              f"v10.14: Rewind, live: a stop for the worker, judged {last.get('cls')} 4 s later: {last.get('text')}")
        ev = [e for e in hub.state()['events'] if e['kind'] == 'stop']
        check(ev and ev[-1]['id'] == rw_id and rew_pub is not None and rew_pub.sent
              and json.loads(rew_pub.sent[-1].data).get('cls') == last.get('cls'),
              'v10.14: the verdict is on the page and on /rewind/stop')
        ans = talker.code_answer('show me your closest call') or ''
        check('worker' in ans and talker.rewind_pick == rw_id and talker.code_answer('stop') is None,
              f'v10.14: "show me your closest call" answered in code: "{ans[:80]}..."')
        tid = talker.submit('were your stops necessary?', 'typed')
        check(wait_for(lambda: any(e.get('id') == tid and e['kind'] == 'robot' for e in hub.state()['events'])),
              'v10.14: "were your stops necessary?" answered')
        rep = [e for e in hub.state()['events'] if e.get('id') == tid and e['kind'] == 'robot'][0]
        check(rep['text'].startswith("I've stopped for people") and not rep.get('sent'), f'with the scorecard: "{rep["text"][:90]}"')
        tid = talker.submit('show me your closest call', 'voice')
        check(wait_for(lambda: any(e['kind'] == 'rewind' and e.get('id') == rw_id for e in hub.state()['events'])),
              'v10.14: and asked to show it, the 3D view is told which stop to act out')
    else:
        print('  --   no rewind.py next to robot_web.py: the Rewind checks are skipped')

    # ------------------------------------------------------------ C: the web server
    print('C  the web server')
    from http.server import ThreadingHTTPServer
    W.Web.hub, W.Web.talker, W.Web.replay = hub, talker, None
    srv = ThreadingHTTPServer(('127.0.0.1', 0), W.Web)
    port = srv.server_address[1]
    threading.Thread(target=srv.serve_forever, daemon=True).start()
    code, body, _ = http(port, '/')
    check(code == 200 and b'Mission Control' in body and b'/api/state' in body, 'the page')
    code, body, _ = http(port, '/api/state?since=0')
    st = json.loads(body)
    check(code == 200 and st['mode'] == 'live' and st['robot'] and st['events'], 'the state (JSON)')
    code, body, _ = http(port, f'/api/state?since={st["seq"]}&trail={st["trail"]["to"]}&epoch={st["trail"]["epoch"]}')
    st2 = json.loads(body)
    check(st2['events'] == [] and st2['trail']['pts'] == [], 'only what is new comes the second time')
    code, body, _ = http(port, '/api/map')
    m = json.loads(body)
    import base64
    check(code == 200 and len(base64.b64decode(m['data'])) == m['w'] * m['h'] and 'aisle_2' in m['zones'],
          'the map, with the zones a click can clean')
    code, body, _ = http(port, '/api/coverage')
    check(code == 200 and json.loads(body)['src'] == 'recorder', 'the floor cleaned')
    code, body, hd = http(port, '/mac.py')
    check(code == 200 and b"VM = '10.11.12.13'" in body and b'def serve' in body and
          'attachment' in (hd.get('Content-Disposition') or ''), 'the Mac helper, with the address the Mac used')
    code, body, hd = http(port, '/ear.py')
    check(code == 200 and b'def hear' in body and b'127.0.0.1' in body and 'attachment' in (hd.get('Content-Disposition') or ''),
          "v10.11: the Mac's ear (Whisper) to download")
    code, body, hd = http(port, '/twin')
    check(code == 200 and b"Robot's View" in body and b'/vendor/three.module.min.js' in body and b'/api/state' in body
          and b'cdn.jsdelivr.net/npm/three@0.160.0' in body, "v10.12: the 3D view (/twin), with the CDN as a fallback")
    code, body, hd = http(port, '/')
    check(b'href="/twin"' in body, 'v10.12: the 2D page has the 3D button')
    have3 = os.path.exists(os.path.join(os.path.dirname(path), 'vendor', 'three.module.min.js'))
    code, body, hd = http_any(port, '/vendor/three.module.min.js')
    if have3:
        check(code == 200 and 'javascript' in (hd.get('Content-Type') or '') and b'WebGLRenderer' in body and b'"160"' in body
              and 'max-age' in (hd.get('Cache-Control') or ''), 'v10.12: three.js r160 from tools/vendor (cached by the browser)')
    else:
        check(code == 404, 'v10.12: no tools/vendor/three.module.min.js: 404, so the page takes it from the CDN')
    code, body, _ = http_any(port, '/vendor/../robot_web.py')
    check(code == 404 and b'import' not in body, 'v10.12: nothing else is served from the folder')
    if os.path.exists(os.path.join(os.path.dirname(path), 'wet_floor.py')):
        code, body, _ = http_any(port, '/api/wet')
        stw = json.loads(http(port, '/api/state?since=0')[1]).get('wet')
        g = json.loads(body) if code == 200 else {}
        check(stw is not None and stw.get('pump') in ('on', 'off', 'idle') and code == 200
              and set(g) >= {'wet', 'habit', 'w', 'h', 'res', 'ox', 'oy', 't_dry'},
              f"v10.13: the wet floor in the state (water {stw and stw.get('pump')}) and its map at /api/wet")
        ans = talker.code_answer('where is the floor wet?') or ''
        check(('wet' in ans or 'dry' in ans) and talker.code_answer('clean aisle 2') is None,
              f'v10.13: "where is the floor wet?" answered in code: "{ans[:70]}..."')
    else:
        print('  --   no wet_floor.py next to robot_web.py: the wet-floor checks are skipped')
    if have_rw and rw_id is not None:
        code, body, _ = http(port, '/api/stops')
        lst = json.loads(body).get('stops') if code == 200 else []
        code2, body2, _ = http(port, f'/api/stop?id={rw_id}')
        d = json.loads(body2) if code2 == 200 else {}
        code3, _, _ = http_any(port, '/api/stop?id=9999')
        check(lst and lst[-1]['id'] == rw_id and len(d.get('path') or []) == 41 and 'worker' in (d.get('people') or {})
              and code3 == 404 and json.loads(http(port, '/api/state?since=0')[1])['rewind']['n'] >= 1,
              'v10.14: the stops at /api/stops, one in full at /api/stop (avoided path, people, real robot), 404 for none')
        code, tw, _ = http(port, '/twin')
        code, pg2, _ = http(port, '/')
        check(b'function rwPlay' in tw and b'/api/stop?id=' in tw and b'Watch it in 3D' in pg2 and b'Closest call' in pg2,
              'v10.14: the 3D view acts a stop out (Rewind); the 2D page links to it')
    code, body, _ = http(port, '/api/talk', {'text': 'go home', 'via': 'chip'})
    tid = json.loads(body).get('id')
    check(code == 200 and tid and wait_for(lambda: json.loads(cmd_pub.sent[-1].data).get('action') == 'home'),
          'talk over HTTP reaches the robot')
    code, body, _ = http(port, '/api/stop', {})
    check(code == 200 and json.loads(body)['ok'] and json.loads(cmd_pub.sent[-1].data)['action'] == 'stop'
          and es_pub.sent[-1].data is True, 'STOP over HTTP')
    code, body, _ = http(port, '/api/release', {})
    check(code == 200 and json.loads(body)['ok'] and es_pub.sent[-1].data is False, 'RELEASE over HTTP')
    srv.shutdown()
    srv.server_close()

    # ------------------------------------------------------------ D: a recorded shift
    if have_rec:
        print(f'D  {label} replayed through the guard (40x, a few seconds)')
        hub2 = W.Hub('replay')
        rp = W.Replay(hub2, runs, label, speed=40.0, guard=os.path.join(os.path.dirname(path), 'people_guard.py'),
                      t_from=None, loop=False)
        rp.start()
        seen = {'shapes': False, 'wait': False, 'mission': False, 'say': False}

        def look():
            s = hub2.state()
            seen['shapes'] |= any(x['ns'] == 'person' for x in s['shapes'])
            seen['wait'] |= bool(s['guard'] and s['guard']['state'] == 'wait')
            seen['mission'] |= bool(s['mission'] and s['mission'].get('zone') == 'aisle_1')
            seen['say'] |= any(e['kind'] == 'say' for e in s['events'])
            return all(seen.values())
        check(wait_for(look, 40.0), f'people, a stop, the job and the narrator all show: {seen}')
        check(wait_for(lambda: ((hub2.state()['replay'] or {}).get('t') or 0) > 60.0, 30.0),
              f'the replay runs on ({(hub2.state()["replay"] or {}).get("t")} s of sim time)')
        rp.control({'cmd': 'seek', 'value': 200.0})
        check(wait_for(lambda: (hub2.state()['replay'] or {}).get('t') and hub2.state()['replay']['t'] >= 199.0, 30.0),
              'seeking jumps there (the guard warms up on the 20 s before)')
        rp.control({'cmd': 'pause'})
        if len(rp.truth):
            time.sleep(6.0)                                  # truth goes stale after 5 s without a refresh
            check(hub2.state()['truth'] is not None, 'v10.12: paused, the simulator truth stays on')
        rws = hub2.state().get('rewind')
        if rws is not None and hub2.rew is not None and hub2.rew.stops:
            c = hub2.rew.detail(rws['closest']['id'], hub2.rewind_now()) if rws.get('closest') else None
            check(rws['n'] >= 3 and rws['n'] < len(hub2.rew.stops) + 1 and c and c['cls'] in ('hit', 'near')
                  and len(c['path']) == 41,
                  f"v10.14: replayed, {rws['n']} of the shift's {len(hub2.rew.stops)} stops so far ({rws['hit']} would have "
                  f"hit, {rws['clear']} over-cautious); the closest call: {c and c['text']}")
        elif rws is not None:
            print('  --   no guard log for this recording: no stops to rewind in the replay')
        w = hub2.state().get('wet')
        if w is not None:
            says = [e for e in hub2.state()['events'] if e.get('wet')]
            check(w['cleaned_m2'] > 5.0 and w['wet_m2'] + w['dry_m2'] > 0.0,
                  f"v10.13: replayed, the wet floor follows the cleaning ({w['cleaned_m2']} m2 cleaned, "
                  f"{w['wet_m2']} m2 still wet, {w['dry_m2']} m2 cleaned dry; {len(says)} wet-floor lines said)")

    print('ALL OK' if not FAILS else f'{len(FAILS)} FAILED: ' + '; '.join(FAILS))
    return 0 if not FAILS else 1


if __name__ == '__main__':
    sys.exit(main(sys.argv[1:]))
