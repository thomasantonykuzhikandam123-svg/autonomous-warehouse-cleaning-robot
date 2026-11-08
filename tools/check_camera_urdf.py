#!/usr/bin/env python3
"""check_camera_urdf.py (v8) - is the robot with the camera still exactly your locked robot?

    python3 ~/ros2_ws/src/tools/check_camera_urdf.py

Expands your robot.urdf.xacro (from the installed warehouse_cleaning_robot package) on its
own, and again through camera/robot_camera.urdf.xacro, then checks:
  - every link, joint, material and gazebo block of your robot is in the camera version,
    element for element identical (wheels, casters, friction, lidar, IMU, diff drive, EKF
    odometry - nothing touched);
  - the camera version adds only camera_link, camera_joint, camera_optical_link,
    camera_optical_joint and the camera sensor, none of them with a collision shape;
  - nothing new reaches down into the lidar's scan plane;
  - how much mass the camera adds.
Prints OK or what differs; exit code 0 / 1. The installer and stack.sh run it.
"""
import argparse
import math
import os
import sys
import xml.etree.ElementTree as ET

HERE = os.path.dirname(os.path.realpath(__file__))
ADDON = os.path.join(HERE, 'camera', 'robot_camera.urdf.xacro')
EXPECTED_NEW = {('joint', 'camera_joint'), ('link', 'camera_link'),
                ('joint', 'camera_optical_joint'), ('link', 'camera_optical_link'),
                ('gazebo', 'camera_link')}


def canon(el):
    """An element as a string that ignores formatting, attribute order and comments."""
    def walk(e):
        attrs = ' '.join(f'{k}="{v}"' for k, v in sorted(e.attrib.items()))
        text = ' '.join((e.text or '').split())
        kids = ''.join(walk(c) for c in e if isinstance(c.tag, str))
        return f'<{e.tag} {attrs}>{text}{kids}</{e.tag}>'
    return walk(el)


def key(el):
    if el.tag in ('link', 'joint', 'material'):
        return (el.tag, el.get('name'))
    if el.tag == 'gazebo':
        return ('gazebo', el.get('reference') or '')
    return (el.tag, '')


def expand(path, mappings=None):
    import xacro
    return ET.fromstring(xacro.process_file(path, mappings=mappings or {}).toxml())


def masses(root):
    return sum(float(m.get('value')) for m in root.iter('mass'))


def z_of(root, link):
    """Height of a link's origin above base_link (fixed joints, no rotations about x/y)."""
    joints = {j.find('child').get('link'): j for j in root.iter('joint')}
    z = 0.0
    while link in joints:
        j = joints[link]
        o = j.find('origin')
        if o is not None and o.get('xyz'):
            z += float(o.get('xyz').split()[2])
        link = j.find('parent').get('link')
    return z


def lowest_new_point(root, links):
    """Lowest visual point of the new links, above base_link (boxes and cylinders, no tilt)."""
    low = math.inf
    for link in root.iter('link'):
        if link.get('name') not in links:
            continue
        z0 = z_of(root, link.get('name'))
        for vis in link.iter('visual'):
            o = vis.find('origin')
            dz = float(o.get('xyz').split()[2]) if o is not None and o.get('xyz') else 0.0
            g = vis.find('geometry')[0]
            half = (float(g.get('size').split()[2]) / 2 if g.tag == 'box' else
                    float(g.get('length')) / 2 if g.tag == 'cylinder' else float(g.get('radius', 0)))
            low = min(low, z0 + dz - half)
    return low


def verdict(orig, cam):
    """(ok, lines) comparing the two expanded robots."""
    problems, lines = [], []
    o_items = {}
    for el in orig:
        if isinstance(el.tag, str):
            o_items.setdefault(key(el), []).append(canon(el))
    c_items = {}
    for el in cam:
        if isinstance(el.tag, str):
            c_items.setdefault(key(el), []).append(canon(el))
    n_same = 0
    for k, vals in o_items.items():
        got = c_items.get(k, [])
        for v in vals:
            if v in got:
                n_same += 1
            else:
                problems.append(f'{k[0]} {k[1] or "(robot-wide)"} differs or is missing')
    new = set(c_items) - set(o_items)
    if new != EXPECTED_NEW:
        problems.append(f'unexpected new elements: {sorted(new - EXPECTED_NEW)}; '
                        f'missing camera parts: {sorted(EXPECTED_NEW - new)}')
    for el in cam:
        if el.tag == 'link' and ('link', el.get('name')) in EXPECTED_NEW and el.find('collision') is not None:
            problems.append(f'{el.get("name")} has a collision shape')
    if orig.get('name') != cam.get('name'):
        problems.append(f'robot name changed: {orig.get("name")} -> {cam.get("name")}')
    lidar_z = z_of(cam, 'lidar_link')
    low = lowest_new_point(cam, {'camera_link', 'camera_optical_link'})
    if low <= lidar_z + 0.01:
        problems.append(f'the camera mast reaches down to {low:.2f} m, into the lidar plane ({lidar_z:.2f} m)')
    m0, m1 = masses(orig), masses(cam)
    cam_z = z_of(cam, 'camera_link')
    lines.append(f'  camera URDF OK: your robot is unchanged ({n_same} links, joints, materials and '
                 f'gazebo blocks identical); the camera adds 4 frames and a sensor')
    lines.append(f'  camera URDF OK: camera at {cam_z:.2f} m, lowest new part at {low:.2f} m, lidar plane at '
                 f'{lidar_z:.2f} m; mass {m0:.2f} -> {m1:.2f} kg (+{1000 * (m1 - m0):.0f} g, no collision shapes)')
    if problems:
        return False, ['  CAMERA URDF WRONG: ' + p for p in problems]
    return True, lines


def main(argv=None):
    ap = argparse.ArgumentParser()
    ap.add_argument('--base', help='robot.urdf.xacro to check against (default: installed package)')
    a = ap.parse_args(argv)
    base = os.path.abspath(a.base) if a.base else None
    if base is None:
        try:
            from ament_index_python.packages import get_package_share_directory
            base = os.path.join(get_package_share_directory('warehouse_cleaning_robot'),
                                'description', 'robot.urdf.xacro')
        except Exception as e:
            print(f'  camera URDF not checked ({type(e).__name__}): run it with ROS sourced:\n'
                  '  source ~/ros2_ws/install/setup.bash && python3 ~/ros2_ws/src/tools/check_camera_urdf.py')
            return 1
    sys.path.insert(0, HERE)
    from importlib import util
    spec = util.spec_from_file_location('lsc', os.path.join(HERE, 'launch_sim_camera.launch.py'))
    lsc = util.module_from_spec(spec)
    try:
        spec.loader.exec_module(lsc)
        maps = lsc.camera_mappings(base)
    except ImportError:                                  # no launch_ros (tests): same defaults
        maps = {'width': '640', 'height': '480', 'rate': '5', 'hfov': '1.5708', 'base': base}
    ok, lines = verdict(expand(base), expand(ADDON, maps))
    print('\n'.join(lines))
    return 0 if ok else 1


if __name__ == '__main__':
    sys.exit(main())
