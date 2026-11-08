#!/usr/bin/env python3
"""traffic.py (v10.2) - warehouse traffic for the tour through both aisles.

    python3 ~/ros2_ws/src/tools/traffic.py spawn         put everyone into the running simulation
    python3 ~/ros2_ws/src/tools/traffic.py run [label]   drive the tugger and the forklift ('traffic' tab)
    python3 ~/ros2_ws/src/tools/traffic.py sdf           print the SDF it would spawn, spawn nothing
    python3 ~/ros2_ws/src/tools/traffic.py where <t>     everyone's scripted position at sim time t

stack.sh start <label> ... traffic does spawn + run for you.

People (Gazebo actors, copies of the world's walking worker, on scripts that loop; they stop,
turn and set off again - the robot cannot know when):
  picker_a   aisle 1: picks at rack_1, walks up the aisle, crosses it, picks at rack_2, comes back
  picker_b   aisle 2: the same at rack_3 and rack_2
  picker_c   aisle 3: the same at rack_4 and rack_3 (v10.10, 14 Oct: every aisle has someone working in it)
  stepper    waits out of sight in aisle 2's top end, then hurries across the back corridor
             (the robot drives along it from aisle 1 to aisle 2): the emergency
  visitor    walks along the front corridor's north side, past both aisle mouths, stopping on the
             way: the robot crosses her path when it sets off into aisle 1
  ... and the world's own worker, crossing aisle 1 as always.
Vehicles (models moved by Gazebo's VelocityControl, driven here from their odometry; no collision
geometry, so nothing is ever pushed - contacts are measured, not felt; v10.2: held at floor height,
see HOLDING THEM ON THE FLOOR below):
  tugger     1.3 x 0.7 m tow tractor shuttling along the back corridor, keep-left lanes, 1.2 m/s,
             5 s stops at each end (hitching carts)
  forklift   1.2 x 1.0 m body, forks 1.1 m long at 5 cm above the floor - below the lidar.
             Shuttles forks-first from the yard south of the front corridor into aisle 2's mouth
             (1.0 m/s), lifts for 6 s, reverses out (0.6 m/s), parks 4 s with its fork tips 0.1 m
             from the robot's westbound lane (front corridor) - invisible to the lidar
Drivers brake for the robot and for people in the zone ahead of them (reaction + braking
distance + 0.3 m), pause a turn with anyone within 0.4 m, and set off again when clear. A robot
that stays in the way for 10 s: the driver backs off 1.5 m, waits 5 s and tries again (no
deadlock where each waits for the other for ever).

run publishes /traffic/truth (std_msgs/String, JSON, 10 Hz: everyone's true pose and outline) for
tour_test.py, /traffic/markers for RViz (forks drawn), and writes ~/runs/traffic/<label>.csv
(v10.2: with each vehicle's height, column z).
"""
import json
import math
import os
import signal
import subprocess
import sys
import tempfile

TOOLS = os.path.dirname(os.path.abspath(__file__))
sys.path.insert(0, TOOLS)

VERSION = 'v10.10'
MAP_OFFSET = (4.0, 4.0)                  # map = world - (4, 4)
WORLD = 'warehouse'
OUT_DIR = os.path.expanduser('~/runs/traffic')
TRUTH_HZ = 10.0
CONTROL_HZ = 20.0

# ---------------------------------------------------------------- people (map frame)
# legs: (x, y, average speed m/s, seconds standing there afterwards, facing (rad) while standing
# or None to keep the walking direction). Each script starts and ends at its first point. Every
# walk starts and ends at rest (Gazebo tension 1): peak speed is 1.5 x the average.
PEOPLE = {
    'picker_a': dict(start=(-1.65, 9.0), face0=math.pi, legs=[
        (-1.65, 9.0, 0.0, 9.0, math.pi),       # picking at rack_1 (facing west)
        (-1.65, 6.2, 0.75, 7.0, math.pi),       # down the aisle beside the up lane, picks again
        (0.60, 5.0, 0.75, 6.0, 0.0),            # across the aisle, picks at rack_2 (facing east)
        (0.60, 10.5, 0.8, 6.0, 0.0),           # up the aisle's east side, picks
        (-1.65, 9.0, 0.75, 0.0, None)]),        # back across
    'picker_b': dict(start=(4.60, 10.0), face0=0.0, legs=[
        (4.60, 10.0, 0.0, 8.0, 0.0),           # picking at rack_3 (facing east)
        (4.60, 6.5, 0.75, 6.0, 0.0),            # down the aisle beside the down lane, picks
        (2.40, 4.0, 0.75, 7.0, math.pi),        # across, picks at rack_2 (facing west)
        (2.40, 9.0, 0.8, 5.0, math.pi),        # up the west side, picks
        (4.60, 10.0, 0.75, 0.0, None)]),        # back across
    'picker_c': dict(start=(8.60, 11.0), face0=0.0, legs=[       # v10.10
        (8.60, 11.0, 0.0, 8.0, 0.0),           # picking at rack_4 (facing east)
        (8.60, 7.0, 0.75, 6.0, 0.0),            # down the aisle's east side, picks
        (6.45, 4.5, 0.75, 7.0, math.pi),        # across, picks at rack_3 (facing west)
        (6.45, 12.0, 0.8, 6.0, math.pi),       # up the west side, picks
        (8.60, 11.0, 0.75, 0.0, None)]),        # back across
    'stepper': dict(start=(2.35, 12.9), face0=math.pi / 2, legs=[
        (2.35, 12.9, 0.0, 16.0, math.pi / 2),  # out of sight behind rack_2's end, in aisle 2
        (2.35, 15.55, 1.3, 3.0, math.pi / 2),  # hurries across the back corridor (both lanes)
        (2.35, 12.9, 1.0, 0.0, None)]),        # and back
    'visitor': dict(start=(7.0, 1.2), face0=math.pi, legs=[
        (7.0, 1.2, 0.0, 4.0, math.pi),         # at the end of rack_4
        (3.6, 1.2, 0.8, 5.0, math.pi / 2),     # stops at aisle 2's mouth, looks in
        (-2.5, 1.2, 0.8, 5.0, 0.0),            # past aisle 1's mouth to rack_1's end, talks
        (7.0, 1.2, 0.85, 0.0, None)]),          # back
}
PERSON_R = 0.25          # m: a person's body, for the drivers and the measurements
TURN_S = 0.6             # s an actor takes to turn on the spot
PEAK = 1.5               # smoothstep: peak speed = 1.5 x average over a leg

# ---------------------------------------------------------------- vehicles (map frame)
# legs: (x, y, speed m/s, 'fwd' | 'rev', seconds waiting afterwards); routes end where they start
VEHICLES = {
    'tugger': dict(L=1.3, W=0.7, H=1.2, fork=0.0, start=(-3.0, 15.2), yaw0=0.0, colour=(0.95, 0.45, 0.05),
                   legs=[(9.5, 15.2, 1.2, 'fwd', 5.0), (9.5, 14.3, 0.6, 'fwd', 0.0),
                         (-3.0, 14.3, 1.2, 'fwd', 5.0), (-3.0, 15.2, 0.6, 'fwd', 0.0)]),
    'forklift': dict(L=1.2, W=1.0, H=2.1, fork=1.1, start=(2.6, -2.75), yaw0=math.pi / 2, colour=(0.95, 0.8, 0.05),
                     legs=[(2.6, 2.5, 1.0, 'fwd', 6.0), (2.6, -2.75, 0.6, 'rev', 4.0)]),
}
TURN_RATE = 0.8          # rad/s turning on the spot
A_BRAKE = 2.0            # m/s^2 the drivers brake at
A_GO = 1.0               # m/s^2 they pull away at
T_REACT = 0.5            # s of travel before the brakes bite (part of the zone)
MARGIN = 0.3             # m they stop short by
SIDE_ROBOT = 0.1         # m beside their own width a robot makes them stop (closer: they pass it)
SIDE_PERSON = 0.3        # m ... a person
TURN_CLEAR = 0.4         # m: nobody this close before they turn on the spot
V_PULL = 0.4             # m/s: standing, the zone they want clear is the one for this speed (~0.5 m)
PATIENCE = 10.0          # s a driver waits for a robot that stays in the way, then backs off ...
BACK_OFF = 1.5           # m ... this far along the way it came, waits PAUSE s, and tries again
PAUSE = 5.0
ROBOT_L, ROBOT_W = 0.6, 0.4

# ---------------------------------------------------------------- holding them on the floor (v10.2)
# Fortress before ign-gazebo 6.17.0 (Jan 2025) ignores a link's <gravity>false</gravity> (6.17 added
# it: gz-sim PR #2451; with it, the hold below settles about 5 mm above the floor, harmless).
# VelocityControl sets the model's velocity at every physics step (the vertical part to zero),
# gravity then acts for that one step, and with no collision nothing holds the model up: it sinks
# g x step per second of sim time, 9.8 mm/s at the default 1 ms step. tour_v101b's recording showed
# it (veh_why): the tugger's body was below the lidar plane before it ever met the robot (t 58 s)
# and all of it by t 104 s; the forklift's body by t 78 s and its masts by t 200 s. The odometry
# (2D up to v10) kept reporting them on the floor. Now the odometry is 3D and every command carries
# a vertical velocity: g x step to cancel the sink, plus Z_GAIN x the height error. They still sink
# between the spawn and this node's first command (about 0.2 m in stack.sh's start-up): the first
# odometry shows it, which also proves the odometry is 3D, and the hold lifts them back.
G = 9.81
Z_GAIN = 2.0             # 1/s: height error -> vertical velocity
VZ_MAX = 0.3             # m/s
Z_WARN = 0.05            # m: further off the floor than this, the traffic tab says so
Z_SETTLE = 10.0          # s after its first odometry a vehicle has to get back to the floor


def wrap(a):
    return math.atan2(math.sin(a), math.cos(a))


def physics_step(path=None):
    """(seconds, where from): the world's <physics><max_step_size>, else Gazebo's default 1 ms."""
    try:
        import xml.etree.ElementTree as ET
        if path is None:
            import scene
            path = scene.world_file()
        el = ET.parse(path).getroot().find('.//physics/max_step_size')
        if el is not None and el.text and 0.0 < float(el.text) <= 0.05:
            return float(el.text), 'the world file'
    except Exception:
        pass
    return 0.001, "Gazebo's default"


def vz_hold(z, ff):
    """Vertical velocity command (model frame; the models never tilt) that keeps a model's origin
    on the floor: ff cancels the sink per step, the rest steers the height z back to 0."""
    return max(-VZ_MAX, min(VZ_MAX, ff - Z_GAIN * z))


def to_world(x, y):
    return x + MAP_OFFSET[0], y + MAP_OFFSET[1]


# ---------------------------------------------------------------- actor scripts
def person_waypoints(spec):
    """[(time, x, y, yaw)] of one person's loop (map frame): stand, turn on the spot, walk a leg
    (one smoothstep: starts and stops at rest), stand, ... back to the start."""
    x, y = spec['start']
    yaw = spec['face0']
    t = 0.0
    wps = [(t, x, y, yaw)]
    for gx, gy, v, stand, face in spec['legs']:
        d = math.hypot(gx - x, gy - y)
        if d > 1e-6:
            head = math.atan2(gy - y, gx - x)
            if abs(wrap(head - yaw)) > 1e-3:
                t += TURN_S
                yaw = head
                wps.append((t, x, y, yaw))
            t += d / v
            x, y = gx, gy
            wps.append((t, x, y, yaw))
        if face is not None and abs(wrap(face - yaw)) > 1e-3:
            t += TURN_S
            yaw = face
            wps.append((t, x, y, yaw))
        if stand > 0:
            t += stand
            wps.append((t, x, y, yaw))
    if abs(wrap(spec['face0'] - yaw)) > 1e-3:
        t += TURN_S
        wps.append((t, x, y, spec['face0']))
    return wps


def person_at(wps, t):
    """Position, yaw and speed at sim time t of a looping script (Gazebo actor, tension 1:
    every waypoint is a stop, smoothstep between them; script time = sim time)."""
    period = wps[-1][0]
    tau = t % period
    for (t0, x0, y0, a0), (t1, x1, y1, a1) in zip(wps, wps[1:]):
        if t0 <= tau <= t1:
            u = (tau - t0) / max(t1 - t0, 1e-9)
            s = u * u * (3 - 2 * u)
            ds = 6 * u * (1 - u) / max(t1 - t0, 1e-9)
            x, y = x0 + (x1 - x0) * s, y0 + (y1 - y0) * s
            return x, y, a0 + wrap(a1 - a0) * s, math.hypot(x1 - x0, y1 - y0) * ds
    return wps[0][1], wps[0][2], wps[0][3], 0.0


def actor_sdf(name, wps, t, yaw_offset=0.0):
    """A copy of the world's actor (t: scene.actor_template) walking the script wps."""
    pts = ''.join(f'<waypoint><time>{tt:.3f}</time><pose>{to_world(x, y)[0]:.3f} {to_world(x, y)[1]:.3f} '
                  f'{t["z"]:.3f} 0 0 {wrap(a + yaw_offset):.4f}</pose></waypoint>' for tt, x, y, a in wps)
    return f'''<?xml version="1.0"?>
<sdf version="1.6">
  <actor name="{name}">
    <skin><filename>{t['skin']}</filename><scale>{t['skin_scale']}</scale></skin>
    <animation name="{t['anim_name']}">
      <filename>{t['anim']}</filename><scale>{t['anim_scale']}</scale><interpolate_x>true</interpolate_x>
    </animation>
    <script>
      <loop>true</loop><delay_start>0</delay_start><auto_start>true</auto_start>
      <trajectory id="0" type="{t['anim_name']}" tension="1">{pts}</trajectory>
    </script>
  </actor>
</sdf>
'''


def world_yaw_offset(path):
    """The world actor's facing convention: waypoint yaw minus walking direction (0 or pi/2...)."""
    import xml.etree.ElementTree as ET
    try:
        root = ET.parse(path).getroot()
    except Exception:
        return 0.0
    actor = root.find('.//actor')
    if actor is None:
        return 0.0
    poses = []
    for wp in actor.findall('.//trajectory/waypoint'):
        try:
            p = [float(v) for v in wp.findtext('pose').split()]
            poses.append(p)
        except (AttributeError, ValueError):
            pass
    for a, b in zip(poses, poses[1:]):
        if math.hypot(b[0] - a[0], b[1] - a[1]) > 0.5:
            head = math.atan2(b[1] - a[1], b[0] - a[0])
            for yaw in (a[5], b[5]):
                off = wrap(yaw - head)
                for c in (0.0, math.pi / 2, -math.pi / 2, math.pi):
                    if abs(wrap(off - c)) < 0.3:
                        return c
    return 0.0


# ---------------------------------------------------------------- vehicle models
def vehicle_sdf(name, spec):
    """Visual-only model (no collision: it can't push the robot or snag on racks), gravity off
    (ignored before ign-gazebo 6.17: see HOLDING THEM ON THE FLOOR), moved by VelocityControl
    (/model/<name>/cmd_vel, model frame), world pose on /model/<name>/odometry (OdometryPublisher,
    3D since v10.2, so the height shows). Forks: two 1.1 m plates 5 cm off the floor."""
    L, W, H, fork = spec['L'], spec['W'], spec['H'], spec['fork']
    r, g, b = spec['colour']
    mat = f'<material><ambient>{r} {g} {b} 1</ambient><diffuse>{r} {g} {b} 1</diffuse></material>'
    dark = '<material><ambient>0.15 0.15 0.15 1</ambient><diffuse>0.2 0.2 0.2 1</diffuse></material>'

    def box(n, x, y, z, sx, sy, sz, m):
        return (f'<visual name="{n}"><pose>{x:.3f} {y:.3f} {z:.3f} 0 0 0</pose>'
                f'<geometry><box><size>{sx:.3f} {sy:.3f} {sz:.3f}</size></box></geometry>{m}</visual>')
    vis = []
    if fork > 0:                                   # forklift: body, counterweight, mast, guard, forks
        vis.append(box('body', -0.1, 0, 0.55, L - 0.2, W, 0.9, mat))
        vis.append(box('counterweight', -L / 2 + 0.15, 0, 0.75, 0.3, W, 1.1, dark))
        vis.append(box('mast_l', L / 2 - 0.05, W / 2 - 0.12, 1.15, 0.1, 0.08, 2.1, dark))
        vis.append(box('mast_r', L / 2 - 0.05, -W / 2 + 0.12, 1.15, 0.1, 0.08, 2.1, dark))
        vis.append(box('guard', 0.0, 0, H, L - 0.3, W, 0.06, dark))
        for k, yy in (('l', 0.25), ('r', -0.25)):
            vis.append(box(f'fork_{k}', L / 2 + fork / 2, yy, 0.05, fork, 0.12, 0.05, dark))
        vis.append(box('carriage', L / 2 + 0.02, 0, 0.4, 0.06, 0.8, 0.7, dark))
    else:                                          # tugger: body, seat back, tow hitch
        vis.append(box('body', 0, 0, 0.45, L, W, 0.7, mat))
        vis.append(box('seat_back', -L / 2 + 0.25, 0, 1.0, 0.1, W - 0.1, 0.5, dark))
        vis.append(box('hitch', -L / 2 - 0.05, 0, 0.3, 0.1, 0.15, 0.1, dark))
    return f'''<?xml version="1.0"?>
<sdf version="1.6">
  <model name="{name}">
    <link name="base">
      <gravity>false</gravity>
      <inertial><mass>400</mass><inertia><ixx>60</ixx><iyy>80</iyy><izz>80</izz><ixy>0</ixy><ixz>0</ixz><iyz>0</iyz></inertia></inertial>
      {''.join(vis)}
    </link>
    <plugin filename="ignition-gazebo-velocity-control-system" name="ignition::gazebo::systems::VelocityControl"/>
    <plugin filename="ignition-gazebo-odometry-publisher-system" name="ignition::gazebo::systems::OdometryPublisher">
      <odom_frame>world</odom_frame><robot_base_frame>{name}</robot_base_frame>
      <dimensions>3</dimensions><odom_publish_frequency>20</odom_publish_frequency>
    </plugin>
  </model>
</sdf>
'''


def corners(cx, cy, yaw, L, W, front=0.0):
    """Rectangle corners [(x, y)] x4; front lengthens it at the front (the forks)."""
    c, s = math.cos(yaw), math.sin(yaw)
    pts = [(-L / 2, -W / 2), (L / 2 + front, -W / 2), (L / 2 + front, W / 2), (-L / 2, W / 2)]
    return [(cx + c * a - s * b, cy + s * a + c * b) for a, b in pts]


def outline(name, x, y, yaw):
    """The vehicle's true footprint: body rectangle and (forklift) the forks' rectangle."""
    sp = VEHICLES[name]
    out = [corners(x, y, yaw, sp['L'], sp['W'])]
    if sp['fork'] > 0:
        c, s = math.cos(yaw), math.sin(yaw)
        fx, fy = x + c * (sp['L'] / 2 + sp['fork'] / 2), y + s * (sp['L'] / 2 + sp['fork'] / 2)
        out.append(corners(fx, fy, yaw, sp['fork'], 0.62))
    return out


def poly_gap(a, b):
    """Smallest distance between two convex polygons (0 if they overlap)."""
    def inside(p, poly):
        sgn = None
        for i in range(len(poly)):
            q0, q1 = poly[i], poly[(i + 1) % len(poly)]
            cr = (q1[0] - q0[0]) * (p[1] - q0[1]) - (q1[1] - q0[1]) * (p[0] - q0[0])
            if abs(cr) < 1e-12:
                continue
            if sgn is None:
                sgn = cr > 0
            elif (cr > 0) != sgn:
                return False
        return True

    def seg_d(p, q0, q1):
        dx, dy = q1[0] - q0[0], q1[1] - q0[1]
        L2 = dx * dx + dy * dy
        u = 0.0 if L2 < 1e-12 else min(max(((p[0] - q0[0]) * dx + (p[1] - q0[1]) * dy) / L2, 0.0), 1.0)
        return math.hypot(q0[0] + u * dx - p[0], q0[1] + u * dy - p[1])
    if any(inside(p, b) for p in a) or any(inside(p, a) for p in b):
        return 0.0
    best = 9e9
    for P, Q in ((a, b), (b, a)):
        for p in P:
            for i in range(len(Q)):
                best = min(best, seg_d(p, Q[i], Q[(i + 1) % len(Q)]))
    return best


def circle_gap(poly, cx, cy, r):
    """Distance from a convex polygon to a circle (0 if they touch)."""
    tiny = [(cx + 1e-6 * math.cos(k * math.pi / 2), cy + 1e-6 * math.sin(k * math.pi / 2)) for k in range(4)]
    return max(0.0, poly_gap(poly, tiny) - r)


# ---------------------------------------------------------------- the drivers
class Driver:
    """One vehicle's route, closed-loop on its odometry: turn on the spot to the leg's heading,
    drive the leg (forwards or reversing) holding the line, wait; brake for anyone ahead."""

    def __init__(self, name):
        self.name = name
        self.sp = VEHICLES[name]
        self.legs = self.sp['legs']
        self.k = 0                      # leg index
        self.phase = 'turn'
        self.wait_until = None
        self.v = 0.0                    # commanded speed (signed: <0 reversing)
        self.from_xy = self.sp['start']
        self.blocked = False
        self.blocked_by = None          # 'robot' or 'person'
        self.brakes = 0                 # times it had to brake for someone (2 s apart)
        self.robot_brakes = 0           # ... for the robot
        self.held_robot = 0.0           # s stopped for the robot
        self.clear_since = -99.0
        self.held = 0.0                 # s stopped for someone
        self.held_now = 0.0             # s stopped for the robot, this time
        self.backoffs = 0               # times it backed off to let the robot go
        self.back_from = None
        self.pose = None                # (x, y, yaw) map frame, from odometry
        self.z = 0.0                    # m: the model's height (origin; 0 = on the floor), odometry
        self.z_seen = False             # the odometry has reported a height (3D)
        self.z_first = None             # its height at the first odometry (it sinks until run starts)
        self.z_t0 = None                # sim time of the first odometry
        self.z_min = self.z_max = None  # height range after Z_SETTLE
        self.z_warned = False

    def leg(self):
        gx, gy, v, mode, wait = self.legs[self.k]
        return gx, gy, v, mode, wait

    def heading_needed(self):
        gx, gy, v, mode, wait = self.leg()
        fx, fy = self.from_xy
        head = math.atan2(gy - fy, gx - fx)
        return head if mode == 'fwd' else wrap(head + math.pi)

    def zone(self, side):
        """The ground ahead the driver watches (polygon), from the travel direction, side m
        wider than the vehicle on each side. Standing mid-leg, the driver still wants the zone
        for V_PULL clear before pulling away (no creeping up on whoever made it stop)."""
        x, y, a = self.pose
        L, W, fork = self.sp['L'], self.sp['W'], self.sp['fork']
        if self.phase not in ('drive', 'backoff'):
            return corners(x, y, a, L + 2 * TURN_CLEAR, W + 2 * TURN_CLEAR, fork)
        sp = max(abs(self.v), V_PULL)
        back = (self.leg()[3] == 'rev') != (self.phase == 'backoff')
        head = wrap(a + math.pi) if back else a
        lead = L / 2 + (0.0 if back else fork)
        D = sp * T_REACT + sp * sp / (2 * A_BRAKE) + MARGIN
        cx, cy = x + math.cos(head) * (lead + D / 2), y + math.sin(head) * (lead + D / 2)
        return corners(cx, cy, head, D, W + 2 * side)

    def someone_in_way(self, robot, people):
        """'robot', 'person' or None: who is in the zone the driver watches."""
        if self.pose is None or self.phase in ('wait', 'pause'):
            return None
        if robot is not None and poly_gap(self.zone(SIDE_ROBOT),
                                          corners(robot[0], robot[1], robot[2], ROBOT_L, ROBOT_W)) <= 0.0:
            return 'robot'
        zp = self.zone(SIDE_PERSON)
        return 'person' if any(circle_gap(zp, px, py, PERSON_R) <= 0.0 for px, py in people) else None

    def step(self, t, dt, robot, people):
        """-> (linear, angular) command in the model frame."""
        if self.pose is None:
            return 0.0, 0.0
        x, y, a = self.pose
        gx, gy, vmax, mode, wait = self.leg()
        who = self.someone_in_way(robot, people)
        blocked = who is not None
        if blocked and not self.blocked and t - self.clear_since > 2.0:
            self.brakes += 1
            self.robot_brakes += who == 'robot'
        if not blocked and self.blocked:
            self.clear_since = t
        self.blocked, self.blocked_by = blocked, who
        self.held_now = self.held_now + dt if (who == 'robot' and abs(self.v) < 0.05) else 0.0
        if self.phase == 'wait':
            self.v = 0.0
            if t >= self.wait_until:
                self.k = (self.k + 1) % len(self.legs)
                self.phase = 'turn'
            return 0.0, 0.0
        if self.phase == 'pause':
            self.v = 0.0
            if t >= self.wait_until:
                self.phase = 'drive'
            return 0.0, 0.0
        if self.phase == 'turn':
            self.v = 0.0
            err = wrap(self.heading_needed() - a)
            if abs(err) < 0.03:
                self.phase = 'drive'
                return 0.0, 0.0
            if blocked:
                self.held += dt
                self.held_robot += dt if who == 'robot' else 0.0
                return 0.0, 0.0
            return 0.0, max(-TURN_RATE, min(TURN_RATE, 2.0 * err))
        # drive: along the leg from from_xy to the goal, holding the line
        fx, fy = self.from_xy
        Lx, Ly = gx - fx, gy - fy
        L = math.hypot(Lx, Ly)
        ux, uy = (Lx / L, Ly / L) if L > 1e-6 else (math.cos(a), math.sin(a))
        left = (gx - x) * ux + (gy - y) * uy          # still to go along the leg
        side = -(x - fx) * uy + (y - fy) * ux         # left of the line (+)
        if self.phase == 'backoff':                   # back the way it came, then pause
            done = math.hypot(x - self.back_from[0], y - self.back_from[1]) >= BACK_OFF or \
                (x - fx) * ux + (y - fy) * uy <= 0.05
            if done or blocked:
                self.v = 0.0
                self.phase, self.wait_until = 'pause', t + PAUSE
                return 0.0, 0.0
            self.v = min(abs(self.v) + A_GO * dt, 0.4) * (1 if mode == 'rev' else -1)
            motion = math.atan2(-uy, -ux) + math.atan(1.5 * side)
            want = motion if mode == 'rev' else wrap(motion + math.pi)
            return self.v, max(-TURN_RATE, min(TURN_RATE, 2.0 * wrap(want - a)))
        if left < 0.03:
            self.v = 0.0
            self.from_xy = (gx, gy)
            self.phase = 'wait'
            self.wait_until = t + wait
            return 0.0, 0.0
        if self.held_now > PATIENCE:                  # the robot stays in the way: make room
            self.held_now = 0.0
            self.backoffs += 1
            self.phase, self.back_from = 'backoff', (x, y)
            return 0.0, 0.0
        target = min(vmax, math.sqrt(2 * 0.8 * max(left - 0.02, 0.0)))   # ease into the stop
        if blocked:
            target = 0.0
            self.held += dt if abs(self.v) < 0.05 else 0.0
            self.held_robot += dt if abs(self.v) < 0.05 and who == 'robot' else 0.0
        cur = abs(self.v)
        cur = max(target, cur - A_BRAKE * dt) if target < cur else min(target, cur + A_GO * dt)
        self.v = cur if mode == 'fwd' else -cur
        motion = math.atan2(uy, ux) - math.atan(1.5 * side)     # steer back onto the line
        want = motion if mode == 'fwd' else wrap(motion + math.pi)
        w = max(-TURN_RATE, min(TURN_RATE, 2.0 * wrap(want - a)))
        return self.v, w


# ---------------------------------------------------------------- spawning
def create(args, timeout=90):
    cmd = ['ros2', 'run', 'ros_gz_sim', 'create', '-world', WORLD] + args
    try:
        p = subprocess.run(cmd, capture_output=True, text=True, timeout=timeout)
    except (OSError, subprocess.TimeoutExpired) as e:
        print(f'  traffic: {" ".join(cmd[5:8])} ... did not finish ({e})')
        return False
    out = (p.stdout or '') + (p.stderr or '')
    ok = p.returncode == 0 and 'fail' not in out.lower()
    if not ok:
        last = [ln for ln in out.splitlines() if ln.strip()][-3:]
        print('  traffic: create said: ' + ' | '.join(last))
    return ok


def spawn():
    import scene
    t = scene.actor_template(scene.world_file())
    tmp = tempfile.mkdtemp(prefix='traffic_')
    done, failed = [], []
    if t is None:
        print(f'  traffic: no actor in {scene.world_file()} to copy, so no people (vehicles only)')
    else:
        off = world_yaw_offset(scene.world_file())
        for name, spec in PEOPLE.items():
            path = os.path.join(tmp, f'{name}.sdf')
            open(path, 'w').write(actor_sdf(name, person_waypoints(spec), t, off))
            (done if create(['-name', name, '-file', path]) else failed).append(name)
    for name, spec in VEHICLES.items():
        path = os.path.join(tmp, f'{name}.sdf')
        open(path, 'w').write(vehicle_sdf(name, spec))
        wx, wy = to_world(*spec['start'])
        ok = create(['-name', name, '-file', path, '-x', f'{wx:.3f}', '-y', f'{wy:.3f}', '-z', '0.0',
                     '-Y', f'{spec["yaw0"]:.4f}'])
        (done if ok else failed).append(name)
    os.makedirs(OUT_DIR, exist_ok=True)
    json.dump({'spawned': done, 'failed': failed, 'version': VERSION},
              open(os.path.join(OUT_DIR, 'spawned.json'), 'w'), indent=1)
    print(f'  traffic: spawned {", ".join(done) or "nobody"}' + (f'; FAILED: {", ".join(failed)}' if failed else ''))
    return 0 if not failed else 2


# ---------------------------------------------------------------- the ROS node
def run(label):
    import csv
    import rclpy
    from rclpy.node import Node
    from rclpy.parameter import Parameter
    from rclpy.time import Time
    from geometry_msgs.msg import Point, Twist
    from nav_msgs.msg import Odometry
    from std_msgs.msg import String
    from visualization_msgs.msg import Marker, MarkerArray
    from builtin_interfaces.msg import Time as TimeMsg
    from tf2_ros import Buffer, TransformListener
    try:
        from rclpy.executors import ExternalShutdownException
    except ImportError:
        ExternalShutdownException = KeyboardInterrupt

    bridge_args = []
    for name in VEHICLES:
        bridge_args += [f'/model/{name}/cmd_vel@geometry_msgs/msg/Twist]gz.msgs.Twist',
                        f'/model/{name}/odometry@nav_msgs/msg/Odometry[gz.msgs.Odometry']
    bridge = subprocess.Popen(['ros2', 'run', 'ros_gz_bridge', 'parameter_bridge'] + bridge_args +
                              ['--ros-args', '-r', '__node:=traffic_bridge'],
                              stdout=subprocess.DEVNULL, stderr=subprocess.DEVNULL, start_new_session=True)
    wps = {name: person_waypoints(spec) for name, spec in PEOPLE.items()}
    step, step_src = physics_step()
    ff = G * step                       # m/s the models sink at with no hold
    spawned = None
    try:
        spawned = set(json.load(open(os.path.join(OUT_DIR, 'spawned.json'))).get('spawned', []))
    except (OSError, ValueError):
        pass
    people_live = [n for n in PEOPLE if spawned is None or n in spawned]

    class Traffic(Node):
        def __init__(self):
            super().__init__('traffic', parameter_overrides=[Parameter('use_sim_time', Parameter.Type.BOOL, True)])
            self.drivers = {n: Driver(n) for n in VEHICLES}
            self.pubs = {n: self.create_publisher(Twist, f'/model/{n}/cmd_vel', 10) for n in VEHICLES}
            for n in VEHICLES:
                self.create_subscription(Odometry, f'/model/{n}/odometry', lambda m, n=n: self.on_odom(n, m), 10)
            self.truth_pub = self.create_publisher(String, '/traffic/truth', 10)
            self.mark_pub = self.create_publisher(MarkerArray, '/traffic/markers', 1)
            self.buf = Buffer()
            self.tfl = TransformListener(self.buf, self)
            self.last_t = None
            self.next_truth = 0.0
            self.said_odom = set()
            self.said_2d = False
            self.started = None
            os.makedirs(OUT_DIR, exist_ok=True)
            self.csv_path = os.path.join(OUT_DIR, f'{label}.csv')
            self.f = open(self.csv_path, 'w', newline='')
            self.w = csv.writer(self.f)
            self.w.writerow(['t', 'name', 'kind', 'x', 'y', 'yaw', 'v', 'state', 'z'])
            self.create_timer(1.0 / CONTROL_HZ, self.tick)
            print(f'traffic {VERSION}: driving {", ".join(VEHICLES)}; people {", ".join(people_live) or "none"} '
                  f'(scripted); truth on /traffic/truth, log {self.csv_path}', flush=True)
            print(f'traffic: holding the vehicles on the floor: physics step {1000 * step:.1f} ms ({step_src}), '
                  f'so {1000 * ff:.1f} mm/s up against the sink, plus {Z_GAIN:.0f}/s x the height from the '
                  f'odometry', flush=True)

        def now(self):
            return self.get_clock().now().nanoseconds * 1e-9

        def on_odom(self, name, msg):
            q = msg.pose.pose.orientation
            yaw = math.atan2(2 * (q.w * q.z + q.x * q.y), 1 - 2 * (q.y ** 2 + q.z ** 2))
            p = msg.pose.pose.position
            d = self.drivers[name]
            d.pose = (p.x - MAP_OFFSET[0], p.y - MAP_OFFSET[1], yaw)
            d.z = float(p.z)
            d.z_seen = d.z_seen or abs(d.z) > 1e-4
            t = self.now()
            if d.z_t0 is None:
                d.z_t0, d.z_first = t, d.z
            elif t - d.z_t0 >= Z_SETTLE:
                d.z_min = d.z if d.z_min is None else min(d.z_min, d.z)
                d.z_max = d.z if d.z_max is None else max(d.z_max, d.z)
                if abs(d.z) > Z_WARN and not d.z_warned:
                    d.z_warned = True
                    print(f'[{t:8.1f}] {name}: WARNING {abs(d.z):.2f} m {"below" if d.z < 0 else "above"} the '
                          f'floor - the height hold is not keeping up', flush=True)
            if name not in self.said_odom:
                self.said_odom.add(name)
                print(f'[{t:8.1f}] {name}: odometry at map ({p.x - MAP_OFFSET[0]:.2f}, '
                      f'{p.y - MAP_OFFSET[1]:.2f}), height {d.z:+.2f} m, driving', flush=True)

        def robot(self):
            try:
                tf = self.buf.lookup_transform('map', 'base_link', Time())
            except Exception:
                return None
            q = tf.transform.rotation
            yaw = math.atan2(2 * (q.w * q.z + q.x * q.y), 1 - 2 * (q.y ** 2 + q.z ** 2))
            return tf.transform.translation.x, tf.transform.translation.y, yaw

        def tick(self):
            t = self.now()
            if t <= 0.0:
                return
            dt = 1.0 / CONTROL_HZ if self.last_t is None else min(max(t - self.last_t, 0.0), 0.5)
            self.last_t = t
            robot = self.robot()
            people = {n: person_at(wps[n], t) for n in people_live}
            ppl_xy = [(p[0], p[1]) for p in people.values()]
            for n, d in self.drivers.items():
                v, w = d.step(t, dt, robot, ppl_xy)
                msg = Twist()
                msg.linear.x, msg.angular.z = float(v), float(w)
                msg.linear.z = float(vz_hold(d.z, ff))
                self.pubs[n].publish(msg)
            if not self.said_2d and t - (self.started or t) > 5.0:
                flat = [n for n, d in self.drivers.items() if d.pose is not None and not d.z_seen]
                if flat:
                    print(f'[{t:8.1f}] traffic: WARNING the odometry of {", ".join(flat)} reports no height '
                          f'(2D?): holding them up open loop only ({1000 * ff:.1f} mm/s)', flush=True)
                self.said_2d = True
            self.started = self.started or t
            if t >= self.next_truth:
                self.next_truth = t + 1.0 / TRUTH_HZ
                self.publish_truth(t, people)

        def publish_truth(self, t, people):
            agents = []
            for n, p in people.items():
                agents.append(dict(name=n, kind='person', x=round(p[0], 3), y=round(p[1], 3), yaw=round(p[2], 3),
                                   v=round(p[3], 3), r=PERSON_R))
                self.w.writerow([f'{t:.2f}', n, 'person', f'{p[0]:.3f}', f'{p[1]:.3f}', f'{p[2]:.3f}', f'{p[3]:.3f}', '',
                                 ''])
            for n, d in self.drivers.items():
                if d.pose is None:
                    continue
                x, y, a = d.pose
                state = d.phase + (f'/held:{d.blocked_by}' if d.blocked else '')
                agents.append(dict(name=n, kind='vehicle', x=round(x, 3), y=round(y, 3), yaw=round(a, 3),
                                   v=round(d.v, 3), z=round(d.z, 3), L=d.sp['L'], W=d.sp['W'], fork=d.sp['fork'],
                                   state=state, brakes=d.brakes, robot_brakes=d.robot_brakes,
                                   held=round(d.held, 1), held_robot=round(d.held_robot, 1)))
                self.w.writerow([f'{t:.2f}', n, 'vehicle', f'{x:.3f}', f'{y:.3f}', f'{a:.3f}', f'{d.v:.3f}', state,
                                 f'{d.z:.3f}'])
            self.truth_pub.publish(String(data=json.dumps(dict(t=round(t, 2), agents=agents))))
            if int(t * TRUTH_HZ) % 5 == 0:
                self.f.flush()
                self.publish_markers(agents)

        def publish_markers(self, agents):
            ma = MarkerArray()
            clear = Marker()
            clear.action = Marker.DELETEALL
            ma.markers.append(clear)
            for k, a in enumerate(agents):
                if a['kind'] == 'person':
                    m = Marker()
                    m.header.frame_id, m.header.stamp = 'map', TimeMsg()
                    m.ns, m.id, m.type, m.action = 'truth_person', k, Marker.CYLINDER, Marker.ADD
                    m.pose.position.x, m.pose.position.y, m.pose.position.z = a['x'], a['y'], 0.9
                    m.pose.orientation.w = 1.0
                    m.scale.x = m.scale.y = 2 * PERSON_R
                    m.scale.z = 1.8
                    m.color.r, m.color.g, m.color.b, m.color.a = 0.6, 0.3, 0.9, 0.35
                    ma.markers.append(m)
                    continue
                for j, poly in enumerate(outline(a['name'], a['x'], a['y'], a['yaw'])):
                    m = Marker()
                    m.header.frame_id, m.header.stamp = 'map', TimeMsg()
                    m.ns, m.id, m.type, m.action = 'truth_vehicle', k * 10 + j, Marker.LINE_STRIP, Marker.ADD
                    m.pose.orientation.w = 1.0
                    m.scale.x = 0.04
                    m.color.r, m.color.g, m.color.b, m.color.a = (0.9, 0.1, 0.9, 1.0) if j else (0.6, 0.3, 0.9, 1.0)
                    m.points = [Point(x=px, y=py, z=0.08) for px, py in poly + poly[:1]]
                    ma.markers.append(m)
                txt = Marker()
                txt.header.frame_id, txt.header.stamp = 'map', TimeMsg()
                txt.ns, txt.id, txt.type, txt.action = 'truth_label', k, Marker.TEXT_VIEW_FACING, Marker.ADD
                txt.pose.position.x, txt.pose.position.y, txt.pose.position.z = a['x'], a['y'], 2.4
                txt.pose.orientation.w = 1.0
                txt.scale.z = 0.3
                txt.color.r = txt.color.g = txt.color.b = txt.color.a = 1.0
                txt.text = f"{a['name']} {abs(a['v']):.1f} m/s" + (
                    ' (waiting for the robot)' if 'held:robot' in a['state'] else
                    ' (waiting for a person)' if 'held:person' in a['state'] else '')
                ma.markers.append(txt)
            self.mark_pub.publish(ma)

        def stop_all(self):
            for n in VEHICLES:
                msg = Twist()
                msg.linear.z = float(ff)        # VelocityControl keeps the last command: stay up
                self.pubs[n].publish(msg)

    rclpy.init()
    node = Traffic()
    try:
        rclpy.spin(node)
    except (KeyboardInterrupt, ExternalShutdownException):
        pass
    finally:
        try:
            node.stop_all()
            node.f.close()
            print('traffic: ' + '; '.join(f'{n} braked {d.brakes}x for someone ({d.robot_brakes}x for the robot), '
                                          f'stood {d.held:.0f} s ({d.held_robot:.0f} s for the robot), '
                                          f'backed off {d.backoffs}x' for n, d in node.drivers.items()), flush=True)
            print('traffic: heights ' + '; '.join(
                f'{n} {"not reported (2D odometry)" if not d.z_seen else "started " + format(d.z_first, "+.2f") + " m"}'
                + (f', then {d.z_min:+.3f} to {d.z_max:+.3f} m' if d.z_min is not None else '')
                for n, d in node.drivers.items()), flush=True)
        except Exception:
            pass
        try:
            os.killpg(bridge.pid, signal.SIGINT)
        except Exception:
            pass
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()
    return 0


def where(t):
    for n, spec in PEOPLE.items():
        x, y, a, v = person_at(person_waypoints(spec), t)
        print(f'  {n:9s} ({x:6.2f}, {y:6.2f}) facing {math.degrees(a):6.1f} deg, {v:.2f} m/s')


def main(argv=None):
    argv = sys.argv[1:] if argv is None else argv
    mode = argv[0] if argv else ''
    if mode == 'spawn':
        return spawn()
    if mode == 'run':
        return run(argv[1] if len(argv) > 1 else 'traffic_last')
    if mode == 'sdf':
        import scene
        t = scene.actor_template(scene.world_file()) or dict(skin='walk.dae', skin_scale='1.0', anim_name='walking',
                                                             anim='walk.dae', anim_scale='1.0', z=1.0)
        for n, spec in PEOPLE.items():
            print(actor_sdf(n, person_waypoints(spec), t))
        for n, spec in VEHICLES.items():
            print(vehicle_sdf(n, spec))
        return 0
    if mode == 'where' and len(argv) > 1:
        where(float(argv[1]))
        return 0
    print(__doc__)
    return 1


if __name__ == '__main__':
    sys.exit(main())
