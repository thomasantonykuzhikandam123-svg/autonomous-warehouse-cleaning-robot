#!/usr/bin/env python3
"""doctor.py (v10.14, 3 Nov) - is everything ready for a shift (or the demo)? Checks, says what is
wrong and how to fix it, changes nothing (except with --fix, below).

    python3 ~/ros2_ws/src/tools/doctor.py [label] [--talk] [--fix]
    python3 ~/ros2_ws/src/tools/doctor.py --rtf          (with the stack up: how fast the sim runs)

  label   also checks the label is unused (its recording, logs and coverage folder do not exist)
  --talk  also checks Ollama on the Mac (for the talk tab) and the web page (port 8080 free, the robot
          agent and the camera available to it, the 'ign' command for the Gazebo window)
  --fix   if ROS would run an old copy of warehouse_planner, rebuilds it (colcon, 1 minute)

Each line is OK, WARN (works, but read it) or FAIL (fix it first). Exit 0 if no FAIL, 1 if a FAIL.
shift.sh runs it before every shift; nothing starts after a FAIL.

Checks: ROS sourced; v10.9 installed (markers: coverage_nav2 v10.7, people_guard v10.8, robot_web
v10.9; files unchanged since the installer, from tools/v109_files.md5); stack.sh starts the cleaning on sim time with its /cmd_vel sent through
the gate and the Collision Monitor (v10.7 turns and backs off on it); ROS runs the source code
(not an old build copy); nothing left running
from the last run; disk space and memory; ~/runs writable; the label unused; the person detector
on the Mac (eye_server.py) and, with --talk, Ollama; the CPU not already busy.
Written for the VM's Python 3.10; standard library only (rclpy only for --rtf).
"""
import hashlib
import json
import os
import re
import shutil
import subprocess
import sys
import time
import urllib.request

VERSION = 'v10.14'
HOME = os.path.expanduser('~')
SRC = os.path.join(HOME, 'ros2_ws', 'src')
TOOLS = os.path.join(SRC, 'tools')
WP = os.path.join(SRC, 'warehouse_planner', 'warehouse_planner')
RUNS = os.path.join(HOME, 'runs')
MANIFEST = os.path.join(TOOLS, 'v1014_files.md5')
THREE_JS = os.path.join(TOOLS, 'vendor', 'three.module.min.js')     # v10.12: the 3D view's engine
THREE_SHA = '3e690ac7d180b0aadf0891bea39eec643e29e2d3e75c99b18689518665f69ba6'                  # three.js r160
# what stack.sh starts (its P pattern) plus the shift's own helpers
STACK_P = (r'ign gazebo|ros2 launch|ros2 run|/opt/ros/humble/lib/|lib/warehouse_planner/|python3 [^ ]*'
           r'(worker_clearance|people_trials|lanes|people_guard|guard_gate|check_wiring|eye|scene|'
           r'picker_test|diag8|traffic|tour_test|narrator|scan_rec)\.py')
MARKERS = [
    (os.path.join(WP, 'coverage_nav2.py'), 'UNSTUCK AND IN ORDER (v10.7', 'coverage_nav2'),
    (os.path.join(WP, 'coverage_nav2.py'), 'BACK_IF', 'coverage_nav2 (this revision)'),
    (os.path.join(TOOLS, 'people_guard.py'), "VERSION = 'v10.11'", 'people_guard (v10.11: the touching-distance hold)'),
    (os.path.join(TOOLS, 'people_guard.py'), 'def touch_hold', 'people_guard (this revision)'),
    (os.path.join(TOOLS, 'people_guard.py'), 'PATIENT_STEADY', 'people_guard (this revision)'),
    (os.path.join(TOOLS, 'narrator.py'), 'narrator.py (v10.7)', 'narrator'),
    (os.path.join(TOOLS, 'lane_report.py'), 'lane_report.py (v10.7', 'lane_report'),
    (os.path.join(WP, 'robot_agent.py'), 'robot.get("manoeuvre")', 'robot_agent'),
    (os.path.join(WP, 'llm_commander.py'), 'WAITING FOR A JOB', 'llm_commander'),
    (os.path.join(TOOLS, 'robot_web.py'), "VERSION = 'v10.14'", 'robot_web (v10.14)'),
    (os.path.join(TOOLS, 'rewind.py'), "VERSION = 'v10.14'", 'rewind (v10.14: every stop and the future it avoided)'),
    (os.path.join(TOOLS, 'robot_twin.html'), 'function rwPlay', 'robot_twin.html (v10.14: Rewind)'),
    (os.path.join(TOOLS, 'wet_floor.py'), "VERSION = 'v10.13'", 'wet_floor (v10.13: the wet-floor guardian)'),
    (os.path.join(TOOLS, 'robot_twin.html'), 'function loadWet', 'robot_twin.html (v10.13: the wet floor)'),
    (os.path.join(TOOLS, 'robot_web.html'), 'function loadWet', 'robot_web.html (v10.13: the wet floor)'),
    (os.path.join(TOOLS, 'robot_twin.html'), 'three.module.min.js', 'robot_twin.html (v10.12: the 3D view)'),
    (os.path.join(TOOLS, 'robot_web.html'), 'href="/twin"', 'robot_web.html (v10.12: the 3D button)'),
    (os.path.join(TOOLS, 'ear_server.py'), "VERSION = 'v10.11'", 'ear_server (v10.11, for the Mac)'),
    (os.path.join(WP, 'coverage_nav2.py'), 'FRONT AND BACK THROUGH AN AISLE (v10.10', 'coverage_nav2 (v10.10)'),
    (os.path.join(TOOLS, 'traffic.py'), "VERSION = 'v10.10'", 'traffic (v10.10: picker_c in aisle 3)'),
    (os.path.join(TOOLS, 'robot_web.html'), 'Mission Control', 'robot_web.html'),
    (os.path.join(TOOLS, 'mac_web.py'), 'mac_web.py (v10.9)', 'mac_web'),
]

RESULTS = []


def say(level, msg):
    RESULTS.append(level)
    print(f'  {level:4s}  {msg}', flush=True)


def md5(path):
    try:
        with open(path, 'rb') as f:
            return hashlib.md5(f.read()).hexdigest()
    except OSError:
        return None


def mac_ip():
    try:
        out = subprocess.run(['ip', 'route'], capture_output=True, text=True, timeout=5).stdout
        m = re.search(r'^default via (\S+)', out, re.M)
        return m.group(1) if m else None
    except Exception:
        return None


def http_json(url, timeout=3.0):
    with urllib.request.urlopen(url, timeout=timeout) as r:
        return json.loads(r.read().decode() or 'null')


def check_ros():
    if os.environ.get('ROS_DISTRO') != 'humble':
        say('FAIL', 'ROS is not set up in this terminal: source /opt/ros/humble/setup.bash '
                    '&& source ~/ros2_ws/install/setup.bash (shift.sh does this itself)')
        return False
    if not os.path.exists(os.path.join(HOME, 'ros2_ws', 'install', 'setup.bash')):
        say('FAIL', '~/ros2_ws/install/setup.bash is missing: the workspace was never built '
                    '(cd ~/ros2_ws && colcon build --symlink-install)')
        return False
    say('OK', f'ROS {os.environ.get("ROS_DISTRO")} set up, workspace built')
    return True


def check_installed():
    ok = True
    missing = []
    for path, marker, name in MARKERS:
        try:
            if marker not in open(path, errors='replace').read():
                missing.append(name)
        except OSError:
            missing.append(name + ' (file missing)')
    if missing:
        say('FAIL', f'v10.14 is not installed completely ({", ".join(missing)}): run the installer '
                    '(run sheet step 2), then this again')
        ok = False
    else:
        say('OK', 'v10.14 installed (Rewind: every stop for a person and the future it avoided; the wet-floor guardian; '
                  'robot_web v10.14 with the 3D view at /twin; people_guard '
                  "v10.11: holds head-on at touching distance; Whisper; coverage_nav2 and traffic v10.10; the rest v10.7)")
    try:
        with open(THREE_JS, 'rb') as f:
            good = hashlib.sha256(f.read()).hexdigest() == THREE_SHA
    except OSError:
        good = None
    if good:
        say('OK', 'the 3D view has its engine here (tools/vendor/three.js r160): no internet needed for it')
    else:
        say('WARN', ('tools/vendor/three.module.min.js is ' + ('not there' if good is None else 'not the expected r160 file'))
            + ': the 3D view (/twin) takes three.js from the internet (cdn.jsdelivr.net) instead. Fine with internet '
              'on the Mac; for none, run the installer again while online. The 2D page does not need it.')
    if os.path.exists(MANIFEST):
        changed = []
        for line in open(MANIFEST):
            parts = line.split()
            if len(parts) == 2 and md5(os.path.join(SRC, parts[1])) != parts[0]:
                changed.append(parts[1])
        if changed:
            say('WARN', 'changed since the installer put them there: ' + ', '.join(changed)
                + ' (if you did not mean to, run the installer again)')
        else:
            say('OK', 'every file the installer put there is unchanged')
    elif ok:
        say('WARN', f'no {os.path.basename(MANIFEST)} (the installer writes it): cannot tell whether '
                    'files were changed since')
    return ok


def check_stack():
    """v10.7: the cleaning tab runs coverage_nav2 on sim time with /cmd_vel remapped to /cmd_vel_raw,
    so its turns and back-offs go through guard_gate and the Collision Monitor, not to the wheels."""
    try:
        txt = open(os.path.join(TOOLS, 'stack.sh'), errors='replace').read()
    except OSError:
        say('FAIL', 'tools/stack.sh is missing')
        return False
    line = next((ln for ln in txt.splitlines() if 'warehouse_planner coverage_nav2' in ln), '')
    if '-r /cmd_vel:=/cmd_vel_raw' in line and 'use_sim_time:=true' in line:
        say('OK', 'stack.sh: the cleaning runs on sim time, its /cmd_vel goes through the gate and the '
                  'Collision Monitor')
        return True
    say('FAIL', 'stack.sh does not start coverage_nav2 with  -p use_sim_time:=true  and  '
                '-r /cmd_vel:=/cmd_vel_raw  (its turns and back-offs would bypass the safety chain): '
                'restore stack.sh from the installer')
    return False


def ros_runs():
    """The file ROS would run for warehouse_planner.coverage_nav2 (install space), or None."""
    try:
        out = subprocess.run([sys.executable, '-c', 'import importlib.util as u; '
                              "print(u.find_spec('warehouse_planner.coverage_nav2').origin)"],
                             capture_output=True, text=True, timeout=20, cwd=HOME).stdout.strip()
        return os.path.realpath(out) if out else None
    except Exception:
        return None


def check_build(fix):
    src = os.path.join(WP, 'coverage_nav2.py')
    f = ros_runs()
    if f and md5(f) == md5(src):
        say('OK', f'ROS runs the source code ({f})')
        return True
    if fix:
        print('        ROS would run an old copy: building warehouse_planner (about a minute) ...', flush=True)
        subprocess.run('cd ~/ros2_ws && colcon build --symlink-install --packages-select warehouse_planner '
                       '2>&1 | tail -3', shell=True, executable='/bin/bash')
        f = ros_runs()
        if f and md5(f) == md5(src):
            say('OK', f'rebuilt: ROS now runs the source code ({f})')
            return True
    say('FAIL', f'ROS would run {f or "nothing"}, not ~/ros2_ws/src/.../coverage_nav2.py: '
                'cd ~/ros2_ws && colcon build --symlink-install --packages-select warehouse_planner '
                '(or: doctor.py --fix)')
    return False


def check_leftovers():
    try:
        out = subprocess.run(['pgrep', '-af', STACK_P], capture_output=True, text=True, timeout=5).stdout
    except Exception:
        out = ''
    mine = [ln for ln in out.splitlines() if 'doctor.py' not in ln and 'pgrep' not in ln]
    if mine:
        say('WARN', f'{len(mine)} process(es) from the last run still going (shift.sh stops them first; '
                    'by hand: ~/ros2_ws/src/tools/stack.sh stop)')
    else:
        say('OK', 'nothing left running from the last run')


def check_space():
    try:
        free = shutil.disk_usage(HOME).free / 1e9
    except OSError:
        free = None
    if free is None:
        say('WARN', 'could not read the free disk space')
    elif free < 1.0:
        say('FAIL', f'only {free:.1f} GB free on the disk: a shift writes ~0.2 GB and ROS logs grow; '
                    'clear ~/.ros/log (rm -rf ~/.ros/log/*) and old ~/runs/*.tgz')
    elif free < 3.0:
        say('WARN', f'{free:.1f} GB free on the disk (enough for a shift; clear ~/.ros/log soon)')
    else:
        say('OK', f'{free:.0f} GB free on the disk')
    try:
        mem = dict(re.findall(r'^(\w+):\s+(\d+)', open('/proc/meminfo').read(), re.M))
        avail = int(mem.get('MemAvailable', 0)) / 1e6
        if avail < 1.5:
            say('WARN', f'only {avail:.1f} GB of memory free: Gazebo, Nav2 and RViz need ~3 GB; close other apps')
        else:
            say('OK', f'{avail:.1f} GB of memory free')
    except Exception:
        pass
    try:
        os.makedirs(os.path.join(RUNS, 'logs'), exist_ok=True)
        p = os.path.join(RUNS, '.doctor_write_test')
        open(p, 'w').write('ok')
        os.remove(p)
        say('OK', '~/runs is writable')
    except OSError as e:
        say('FAIL', f'cannot write to ~/runs ({e})')
    try:
        load = os.getloadavg()[0]
        cores = os.cpu_count() or 1
        if load > 0.7 * cores:
            say('WARN', f'the CPU is already busy (load {load:.1f} on {cores} cores): close other apps, '
                        'or the simulation runs slowly')
        else:
            say('OK', f'CPU free (load {load:.1f} on {cores} cores)')
    except OSError:
        pass


def check_label(label):
    used = [p for p in (os.path.join(RUNS, 'rec', f'{label}.npz'), os.path.join(RUNS, f'cov_{label}'),
                        os.path.join(RUNS, 'logs', f'clean_{label}.log'), os.path.join(RUNS, 'guard', f'{label}.csv'))
            if os.path.exists(p)]
    if used:
        m = re.match(r'(.*?)([a-y])$', label)
        nxt = (m.group(1) + chr(ord(m.group(2)) + 1)) if m else label + 'b'
        while any(os.path.exists(p.replace(label, nxt)) for p in used):
            m = re.match(r'(.*?)([a-y])$', nxt)
            nxt = (m.group(1) + chr(ord(m.group(2)) + 1)) if m else nxt + 'b'
        say('FAIL', f'label {label} was used before ({os.path.relpath(used[0], HOME)}): '
                    f'use another, e.g. {nxt}')
        return False
    say('OK', f'label {label} is new')
    return True


def check_mac(talk):
    ip = mac_ip() or '10.0.2.2'
    eye = os.environ.get('EYE_SERVER', f'http://{ip}:8765')
    try:
        h = http_json(f'{eye}/health')
        say('OK', f'person detector on the Mac is up ({h.get("model", "?")} on {h.get("device", "?")})')
    except Exception:
        say('WARN', f'no person detector at {eye}: on the Mac run  cd ~/eye && ./env/bin/python '
                    'eye_server.py  and  caffeinate -dimsu  (without it, people standing still get no '
                    'personal space; walking people are still seen by the lidar)')
    if talk:
        host = os.environ.get('OLLAMA', f'http://{ip}:11434')
        try:
            names = [m.get('name', '') for m in (http_json(f'{host}/api/tags') or {}).get('models', [])]
            if any(n.startswith('llama3.2') for n in names):
                say('OK', f'Ollama on the Mac is up with llama3.2 ({host})')
            else:
                say('WARN', 'Ollama is up but has no llama3.2: on the Mac run  ollama pull llama3.2')
        except Exception:
            say('WARN', f'no Ollama at {host}: on the Mac quit the Ollama app (menu bar), then run  '
                        'OLLAMA_HOST=0.0.0.0 ollama serve  (the talk tab needs it; the cleaning does not)')
    cleaned = os.path.join(HOME, '.ros', 'warehouse_cleaned.json')
    if os.path.exists(cleaned):
        say('OK', 'a "cleaned" list from an earlier run exists (shift.sh resets it)')


def check_web():
    """v10.9: what the web page (robot_web.py) needs, for the demo."""
    import socket
    s = socket.socket(socket.AF_INET, socket.SOCK_STREAM)
    try:
        s.bind(('0.0.0.0', 8080))
        say('OK', 'port 8080 is free for the web page')
    except OSError:
        try:
            run = subprocess.run(['pgrep', '-af', 'robot[_]web.py'], capture_output=True, text=True,
                                 timeout=5).stdout.strip()
        except Exception:
            run = ''
        say('WARN', 'port 8080 is in use' + (' by a web page from before (shift.sh demo stops it first)' if run
                                             else ' by another program: the web page cannot start; close it'))
    finally:
        s.close()
    code = ('import sys; sys.path.insert(0, %r)\n'
            'from warehouse_planner.robot_agent import build_graph\n'
            'build_graph(lambda s, t: "")\nprint("ok")' % TOOLS)
    try:
        r = subprocess.run([sys.executable, '-c', code], capture_output=True, text=True, timeout=60, cwd=HOME)
        if r.stdout.strip().endswith('ok'):
            say('OK', 'the web page can use the robot agent (questions and commands, as the talk tab)')
        else:
            err = (r.stderr.strip().splitlines() or ['?'])[-1]
            say('WARN', f'the web page cannot load the robot agent ({err}): it will take plain commands only')
    except Exception as e:
        say('WARN', f'could not check the robot agent for the web page ({e})')
    try:
        r = subprocess.run([sys.executable, '-c', 'import cv2'], capture_output=True, timeout=30)
        if r.returncode == 0:
            say('OK', "the web page can show the robot's camera (cv2)")
        else:
            say('WARN', "no cv2 for python3: the web page shows no camera (everything else works)")
    except Exception:
        pass
    if shutil.which('ign'):
        say('OK', "the 'ign' command is there for the Gazebo window (shift.sh gui)")
    else:
        say('WARN', "no 'ign' command: the Gazebo window will not open (the simulation still runs)")


def rtf(seconds=6.0):
    """With the stack up: sim seconds per real second, from /clock."""
    try:
        import rclpy
        from rclpy.node import Node
        from rosgraph_msgs.msg import Clock
    except ImportError:
        say('WARN', 'rclpy not found: source ROS first')
        return None
    rclpy.init()
    node = Node('doctor_rtf')
    got = []
    node.create_subscription(Clock, '/clock', lambda m: got.append((time.monotonic(), m.clock.sec + m.clock.nanosec * 1e-9)), 10)
    t0 = time.monotonic()
    while time.monotonic() - t0 < seconds:
        rclpy.spin_once(node, timeout_sec=0.2)
    node.destroy_node()
    rclpy.shutdown()
    if len(got) < 2 or got[-1][0] - got[0][0] < 1.0:
        say('WARN', 'no /clock: is the simulation running?')
        return None
    f = (got[-1][1] - got[0][1]) / (got[-1][0] - got[0][0])
    lvl = 'OK' if f >= 0.3 else 'WARN'
    say(lvl, f'the simulation runs at {f:.2f}x real time: a 600 s job takes about {600 / max(f, 0.05) / 60:.0f} '
             'min' + ('' if f >= 0.3 else ' (very slow: close other apps on the Mac, keep it on power)'))
    return f


def main(argv):
    args = [a for a in argv if not a.startswith('--')]
    flags = {a for a in argv if a.startswith('--')}
    print(f'doctor.py {VERSION}: ready for a shift?', flush=True)
    if '--rtf' in flags:
        rtf()
        return 0
    ok = check_ros()
    if check_installed() and ok:
        check_build('--fix' in flags)
    check_stack()
    check_leftovers()
    check_space()
    if args:
        check_label(args[0])
    check_mac('--talk' in flags)
    if '--talk' in flags:
        check_web()
    n_fail, n_warn = RESULTS.count('FAIL'), RESULTS.count('WARN')
    if n_fail:
        print(f'NOT READY: {n_fail} FAIL (fix them first){f", {n_warn} WARN" if n_warn else ""}.', flush=True)
        return 1
    print('READY' + (f' ({n_warn} WARN: read them)' if n_warn else '') + '.', flush=True)
    return 0


if __name__ == '__main__':
    sys.exit(main(sys.argv[1:]))
