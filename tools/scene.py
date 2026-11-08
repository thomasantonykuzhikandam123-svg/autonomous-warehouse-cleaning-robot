#!/usr/bin/env python3
"""scene.py (v10) - put a standing picker and a look-alike post into aisle 2 of the running simulation.

    python3 ~/ros2_ws/src/tools/scene.py spawn     (stack.sh start <label> ... scene does it for you)
    python3 ~/ros2_ws/src/tools/scene.py sdf       print what it would spawn, spawn nothing

The picker: a person standing at rack_2's east face, picking from the shelf (facing it), at map
    (2.40, 6.00). She is a copy of the world's walking worker (warehouse.sdf's actor: same body,
    same skin), frozen in one pose: what the camera and the lidar see is the worker they already
    know, standing still. If warehouse.sdf has no actor, the Fuel model 'Standing person' is used.
The post: a grey column, 0.30 m wide and 1.7 m tall, at map (2.40, 3.00). To the 2D lidar it is
    a small blob like her legs; to the camera it is plainly not a person.
Both stand 0.35 m from aisle 2's northbound lane (x 2.75), 3 m apart; the southbound lane (x 4.25)
passes 1.85 m from them. The walking worker in aisle 1 never comes near aisle 2.
Writes ~/runs/scene.json (map frame) for eye.py, picker_test.py and the reports. The world file
is not changed: the two are gone at the next stack.sh start.
"""
import json
import math
import os
import re
import subprocess
import sys
import tempfile
import xml.etree.ElementTree as ET

MAP_OFFSET = (4.0, 4.0)                  # map = world - (4, 4)
PICKER = dict(name='picker', kind='person', x=2.40, y=6.00, yaw=math.pi)   # facing the shelf (west)
POST = dict(name='post', kind='object', x=2.40, y=3.00, radius=0.15, height=1.70)
FUEL_PERSON = 'https://fuel.gazebosim.org/1.0/OpenRobotics/models/Standing person'
WORLD = 'warehouse'
OUT = os.path.expanduser('~/runs/scene.json')


def world_file():
    try:
        from ament_index_python.packages import get_package_share_directory
        return os.path.join(get_package_share_directory('warehouse_cleaning_robot'), 'worlds', 'warehouse.sdf')
    except Exception:
        return os.path.expanduser('~/ros2_ws/src/warehouse_cleaning_robot/worlds/warehouse.sdf')


def actor_template(path):
    """The first <actor> in the world: skin, animation, scale, interpolate_x and the height of its
    trajectory (where its feet are), or None."""
    try:
        root = ET.parse(path).getroot()
    except (OSError, ET.ParseError):
        return None
    actor = root.find('.//actor')
    if actor is None:
        return None
    skin = actor.find('skin')
    anim = actor.find('animation')
    if skin is None or skin.findtext('filename') is None:
        return None
    z = 1.0
    wp = actor.find('.//trajectory/waypoint/pose')
    if wp is not None and wp.text:
        try:
            z = float(wp.text.split()[2])
        except (IndexError, ValueError):
            pass
    return dict(skin=skin.findtext('filename').strip(), skin_scale=(skin.findtext('scale') or '1.0').strip(),
                anim_name=(anim.get('name') if anim is not None else 'walking') or 'walking',
                anim=(anim.findtext('filename') if anim is not None else None) or skin.findtext('filename').strip(),
                anim_scale=((anim.findtext('scale') if anim is not None else None) or '1.0').strip(),
                z=z, source=actor.get('name', 'actor'))


def picker_sdf(t):
    """A standing copy of the world's actor: its trajectory stays at one pose for ever, so with
    interpolate_x (animation tied to distance walked) it holds one frame of the walk."""
    wx, wy = PICKER['x'] + MAP_OFFSET[0], PICKER['y'] + MAP_OFFSET[1]
    pose = f'{wx:.3f} {wy:.3f} {t["z"]:.3f} 0 0 {PICKER["yaw"]:.4f}'
    return f'''<?xml version="1.0"?>
<sdf version="1.6">
  <actor name="{PICKER['name']}">
    <skin><filename>{t['skin']}</filename><scale>{t['skin_scale']}</scale></skin>
    <animation name="{t['anim_name']}">
      <filename>{t['anim']}</filename><scale>{t['anim_scale']}</scale><interpolate_x>true</interpolate_x>
    </animation>
    <script>
      <loop>true</loop><delay_start>0</delay_start><auto_start>true</auto_start>
      <trajectory id="0" type="{t['anim_name']}">
        <waypoint><time>0</time><pose>{pose}</pose></waypoint>
        <waypoint><time>100000</time><pose>{pose}</pose></waypoint>
      </trajectory>
    </script>
  </actor>
</sdf>
'''


def post_sdf():
    wx, wy = POST['x'] + MAP_OFFSET[0], POST['y'] + MAP_OFFSET[1]
    r, h = POST['radius'], POST['height']
    geom = f'<geometry><cylinder><radius>{r}</radius><length>{h}</length></cylinder></geometry>'
    return f'''<?xml version="1.0"?>
<sdf version="1.6">
  <model name="{POST['name']}">
    <static>true</static>
    <pose>{wx:.3f} {wy:.3f} {h / 2:.3f} 0 0 0</pose>
    <link name="link">
      <collision name="collision">{geom}</collision>
      <visual name="visual">{geom}
        <material><ambient>0.45 0.45 0.45 1</ambient><diffuse>0.55 0.55 0.55 1</diffuse><specular>0.1 0.1 0.1 1</specular></material>
      </visual>
    </link>
  </model>
</sdf>
'''


def create(args, timeout=90):
    """ros2 run ros_gz_sim create ...; True if Gazebo made the entity."""
    cmd = ['ros2', 'run', 'ros_gz_sim', 'create', '-world', WORLD] + args
    try:
        p = subprocess.run(cmd, capture_output=True, text=True, timeout=timeout)
    except (OSError, subprocess.TimeoutExpired) as e:
        print(f'  scene: {" ".join(cmd[3:6])} ... did not finish ({e})')
        return False
    out = (p.stdout or '') + (p.stderr or '')
    ok = p.returncode == 0 and 'fail' not in out.lower()
    if not ok:
        last = [ln for ln in out.splitlines() if ln.strip()][-3:]
        print('  scene: create said: ' + ' | '.join(last))
    return ok


def main(argv=None):
    argv = sys.argv[1:] if argv is None else argv
    mode = argv[0] if argv else 'spawn'
    t = actor_template(world_file())
    if mode == 'sdf':
        print(picker_sdf(t) if t else f'(no actor in {world_file()}: would use {FUEL_PERSON})')
        print(post_sdf())
        return 0
    if mode != 'spawn':
        print('usage: scene.py spawn | sdf')
        return 1
    tmp = tempfile.mkdtemp(prefix='scene_')
    person_from = None
    if t is not None:
        path = os.path.join(tmp, 'picker.sdf')
        open(path, 'w').write(picker_sdf(t))
        if create(['-name', PICKER['name'], '-file', path]):
            person_from = f"a standing copy of the world's actor '{t['source']}'"
    if person_from is None:
        wx, wy = PICKER['x'] + MAP_OFFSET[0], PICKER['y'] + MAP_OFFSET[1]
        if create(['-name', PICKER['name'], '-file', FUEL_PERSON, '-x', f'{wx}', '-y', f'{wy}', '-z', '0',
                   '-Y', f'{PICKER["yaw"]}'], timeout=180):
            person_from = "Fuel's 'Standing person'"
    path = os.path.join(tmp, 'post.sdf')
    open(path, 'w').write(post_sdf())
    # v10: create's own pose (default 0 0 0) overrides the one in the SDF, so give it explicitly
    # (v8.2 put the post at the world origin, outside the warehouse's walls)
    wx, wy = POST['x'] + MAP_OFFSET[0], POST['y'] + MAP_OFFSET[1]
    post_ok = create(['-name', POST['name'], '-file', path, '-x', f'{wx:.3f}', '-y', f'{wy:.3f}',
                      '-z', f'{POST["height"] / 2:.3f}'])
    objects = []
    if person_from:
        objects.append({k: PICKER[k] for k in ('name', 'kind', 'x', 'y')})
    if post_ok:
        objects.append({k: POST[k] for k in ('name', 'kind', 'x', 'y')})
    os.makedirs(os.path.dirname(OUT), exist_ok=True)
    json.dump({'objects': objects, 'picker_from': person_from or ''}, open(OUT, 'w'), indent=1)
    print(f'  scene: picker {"at (%.2f, %.2f), %s" % (PICKER["x"], PICKER["y"], person_from) if person_from else "NOT spawned"}; '
          f'post {"at (%.2f, %.2f)" % (POST["x"], POST["y"]) if post_ok else "NOT spawned"} (map frame)')
    return 0 if person_from and post_ok else 2


if __name__ == '__main__':
    sys.exit(main())
