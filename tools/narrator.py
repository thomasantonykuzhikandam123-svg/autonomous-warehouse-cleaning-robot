#!/usr/bin/env python3
"""narrator.py (v10.7) - the robot says what it is doing, in the first person, as it happens.

    python3 ~/ros2_ws/src/tools/narrator.py          ('robot says' tab: stack.sh ... clean talk)

Reads /mission_state (coverage_nav2 v10.4: the cleaning plus what the people guard is doing)
and prints one line whenever something changes that a person watching would want explained:
a new job, a new zone, every stop for a person or a vehicle (and how long it took), moving
slowly near someone, a stretch put off because someone stood in the lane, going back for it,
giving up on one, coverage every 10 %, and the end of the job. Plain templates, no model:
what it says is exactly what the robot's own state says.

v10.5 (11 Oct, after shift_v104a): it said "Taking it slowly" 600 times, 500 of them after the
job was done, sitting at the dock while traffic went by (the guard's limit flipped between
"a vehicle is near" and "easing off near people" every second). Now nothing about traffic is
said while the robot is idle (done or paused), and a slow line comes at most every SLOW_GAP s
and only when the reason is new. New lines: going round someone standing in the lane, the
patch pass before going home, and how much of each aisle's lanes got swept.

v10.6 (11 Oct, after shift_v105a, where the simulation froze for 22 minutes and it looked as if the
robot was waiting): a real-time watchdog says so when the robot has not reported for FROZEN_S s of
real time (Gazebo paused, or the computer busy or asleep), and when it is back. New lines: holding
still because the safety layer went quiet, stopping short of something standing in the lane, and
leaving a lane's end (or start) to a vehicle working there.

v10.7 (12 Oct, after shift_v106a): a line when the robot frees itself from a rack corner (backing
off and turning). Before the first job it says nothing about a job (coverage_nav2 v10.7 reports
no queue until a command comes).
"""
import json
import sys
import threading
import time

ZONE = {'front': 'the front area', 'aisle_1': 'aisle 1', 'aisle_2': 'aisle 2',
        'aisle_3': 'aisle 3', 'back': 'the back area', 'home': 'the dock'}


FROZEN_S = 20.0          # s of real time without a report from the robot: say the simulation froze


class FreezeWatch:
    """v10.6, ROS-free: seen(wall) at every report, check(wall) every few seconds (real time).
    Each returns a line to print, or None."""

    def __init__(self, frozen_s=None):
        self.frozen_s = FROZEN_S if frozen_s is None else frozen_s
        self.last = None
        self.since = None

    def seen(self, wall):
        out = None
        if self.since is not None:
            out = (f'(the robot is reporting again, after {(wall - self.since) / 60.0:.1f} min of real '
                   'time without a word)')
            self.since = None
        self.last = wall
        return out

    def check(self, wall):
        if self.last is None or self.since is not None or wall - self.last <= self.frozen_s:
            return None
        self.since = self.last
        return (f'(no word from the robot for {self.frozen_s:.0f} s of real time: the simulation is '
                'paused or frozen, or the computer is busy or asleep. Not the robot. Check Gazebo and '
                'that the Mac is awake.)')


def zname(z):
    return ZONE.get(z, str(z))


class Narrator:
    """ROS-free: feed() one /mission_state dict at a time (with sim time t), get lines back."""
    SLOW_GAP = 30.0          # s between two "Taking it slowly" lines
    SLOW_NEW = 10.0          # s without a speed limit before the same reason is news again

    def __init__(self):
        self.prev = None
        self.wait_since = None
        self.wait_say = None
        self.slow_said = None
        self.slow_last = -1e9
        self.fast_since = None
        self.cov_said = 0
        self.n_deferred = 0
        self.n_skipped = 0
        self.revisiting = False
        self.patching = False
        self.patch_round = 0             # v10.6
        self.n_bypass = 0
        self.safety = False
        self.nav2 = False                # v10.6
        self.n_vskip = 0
        self.blocked = False
        self.blocked_said = -1e9
        self.n_unstick = 0               # v10.7

    def feed(self, s, t):
        out = []
        p = self.prev or {}
        tr = s.get('traffic') or {}
        asked, pasked = len(s.get('asked') or []), len(p.get('asked') or [])
        if not p or asked != pasked or (s.get('goal') or 0) < (p.get('goal') or 0):
            q = [zname(z) for z in s.get('queue') or []]
            if q and not s.get('done'):
                out.append(f"New job: {', then '.join(q)}.")
                self.cov_said = int((s.get('coverage_pct') or 0) // 10) * 10
        if s.get('paused') and not p.get('paused'):
            out.append("Paused, as asked. I'll wait here.")
        if p.get('paused') and not s.get('paused'):
            out.append('Carrying on.')
        # stops for people and vehicles: only while there is a job to do (v10.5)
        idle = bool(s.get('done') or s.get('paused'))
        waiting = tr.get('state') == 'wait' and not idle
        if waiting and self.wait_since is None:
            self.wait_since, self.wait_say = t, tr.get('say')
            say = tr.get('say') or 'my safety layer is holding me'
            out.append(('Stopped: ' + say[len('stopped: '):] if say.startswith('stopped: ') else
                        say[0].upper() + say[1:]) + '.')
        elif waiting and tr.get('say') and tr.get('say') != self.wait_say and \
                tr.get('why') != (p.get('traffic') or {}).get('why'):
            self.wait_say = tr.get('say')
            out.append('Still waiting: ' + self.wait_say + '.')
        elif not waiting and self.wait_since is not None:
            took = t - self.wait_since
            self.wait_since = None
            if took >= 2.0 and not idle:
                out.append(f'Off again after {took:.0f} s.')
        slow = (not waiting) and (not idle) and tr.get('v_limit') is not None and tr.get('limit')
        if slow:
            self.fast_since = None
            if tr.get('limit') != self.slow_said and t - self.slow_last >= self.SLOW_GAP:
                out.append(f"Taking it slowly ({tr.get('v_limit'):.1f} m/s): {tr.get('limit')}.")
                self.slow_said, self.slow_last = tr.get('limit'), t
        elif self.fast_since is None:
            self.fast_since = t
        elif t - self.fast_since >= self.SLOW_NEW:
            self.slow_said = None
        # v10.5: going round someone standing in the lane, the patch pass
        nb = s.get('bypasses') or 0
        if nb > self.n_bypass:
            out.append("Someone is standing in my lane, so I'm going round them. "
                       "I'll sweep that bit before going home.")
        self.n_bypass = nb
        # v10.6: the safety layer silent, something in the lane, a vehicle at a lane end
        if s.get('safety_hold') and not self.safety:
            out.append("My safety layer has stopped reporting, so I'm holding still until it's back.")
        elif self.safety and not s.get('safety_hold'):
            out.append('My safety layer is back. Carrying on.')
        self.safety = bool(s.get('safety_hold'))
        if s.get('nav2_down') and not self.nav2:
            out.append("My navigation (Nav2) has stopped taking goals, so I'm holding still. "
                       "I'll carry on as soon as it's back.")
        elif self.nav2 and not s.get('nav2_down'):
            out.append('My navigation is back. Carrying on.')
        self.nav2 = bool(s.get('nav2_down'))
        nv = s.get('vehicle_skips') or 0
        if nv > self.n_vskip:
            out.append("A vehicle is working where I need to sweep, so I'm leaving that bit for now. "
                       "I'll come back for it before going home.")
        self.n_vskip = nv
        nu = s.get('unsticks') or 0                # v10.7: freed itself from a rack or wall
        if nu > self.n_unstick:
            out.append("I got stuck facing a rack, so I'm backing off a little and turning to line "
                       "up, then trying again.")
        self.n_unstick = nu
        blocked = bool((s.get('lane') or {}).get('blocked_ahead')) and not idle
        if blocked and not self.blocked and t - self.blocked_said >= self.SLOW_GAP:
            out.append("Something is standing in my lane, so I've stopped short of it.")
            self.blocked_said = t
        self.blocked = blocked
        if s.get('patching') and not self.patching:
            k = s.get('patches') or 0
            out.append(f"Going back over {k} stretch{'es' if k != 1 else ''} of lane I left dirty, "
                       "before going home." if k else "Going back over lane I left dirty.")
        self.patching = bool(s.get('patching'))
        pr = s.get('patch_round') or 0             # v10.6: the second round
        if pr >= 2 and self.patch_round == 1 and s.get('patching'):
            out.append("Some lane is still not swept because someone was standing on it, so I'm "
                       "going over it once more before going home.")
        self.patch_round = pr
        # put off, revisits, given up
        nd = len(s.get('deferred') or [])
        if nd > self.n_deferred:
            d = (s.get('deferred') or [])[-1]
            out.append(f"Someone has been in my way for a while in {zname(d.get('zone'))}, so I'll "
                       f"leave that stretch and come back for it before going home.")
        self.n_deferred = nd
        revisiting = bool(s.get('revisiting')) and not s.get('patching')
        if revisiting and not self.revisiting:
            out.append('Going back for the floor I had to leave earlier.')
        self.revisiting = revisiting
        ns = len(s.get('skipped') or [])
        if ns > self.n_skipped:
            k = (s.get('skipped') or [])[-1]
            out.append(f"I couldn't get to a stretch of {zname(k.get('zone'))} even the second time, "
                       "so I'm leaving it.")
        self.n_skipped = ns
        z, pz = s.get('zone'), p.get('zone')
        if p and z != pz and z not in (None, 'done', 'start') and not s.get('done'):
            out.append('Heading back to the dock.' if z == 'home' else f'Now cleaning {zname(z)}.')
        cov = s.get('coverage_pct') or 0
        if cov >= self.cov_said + 10 and not s.get('done'):
            self.cov_said = int(cov // 10) * 10
            out.append(f'{self.cov_said} % of the floor done.')
        if s.get('done') and not p.get('done'):
            cl = [zname(c) for c in s.get('cleaned') or []]
            sw = s.get('swept') or {}
            lanes = ('Lanes swept: ' + ', '.join(f'{zname(z)} {p:.0f} %' for z, p in sw.items())
                     + '. ') if sw else ''
            h = s.get('holds') or 0
            stops = (f"I stopped for people or vehicles {h} time{'' if h == 1 else 's'}, "
                     f"{s.get('held_s') or 0:.0f} s in all." if h else
                     "I never had to stop for people or vehicles.")
            out.append(f"Job done. Cleaned: {', '.join(cl) if cl else 'nothing end to end'}; "
                       f"{cov:.0f} % of the floor overall. {lanes}{stops}")
        self.prev = s
        return out


def main():
    import rclpy
    from rclpy.node import Node
    from rclpy.parameter import Parameter
    from std_msgs.msg import String
    try:
        from rclpy.executors import ExternalShutdownException
    except ImportError:
        ExternalShutdownException = KeyboardInterrupt

    class NarratorNode(Node):
        def __init__(self):
            super().__init__('narrator', parameter_overrides=[
                Parameter('use_sim_time', Parameter.Type.BOOL, True)])
            self.n = Narrator()
            self.fw = FreezeWatch()          # v10.6: real time, not sim time
            self.create_subscription(String, '/mission_state', self.on_state, 10)
            threading.Thread(target=self.watch, daemon=True).start()
            print('narrator v10.7: the robot will say what it is doing (from /mission_state) ...',
                  flush=True)

        def on_state(self, msg):
            try:
                s = json.loads(msg.data)
            except Exception:
                return
            back = self.fw.seen(time.monotonic())
            if back:
                print(f'[{time.strftime("%H:%M:%S")}] {back}', flush=True)
            t = self.get_clock().now().nanoseconds * 1e-9
            for line in self.n.feed(s, t):
                print(f'[{t:7.1f}] robot: {line}', flush=True)

        def watch(self):
            """v10.6: real time, not sim time: a frozen simulation stops sim time too."""
            while True:
                time.sleep(5.0)
                line = self.fw.check(time.monotonic())
                if line:
                    print(f'[{time.strftime("%H:%M:%S")}] {line}', flush=True)

    rclpy.init()
    node = NarratorNode()
    try:
        rclpy.spin(node)
    except (KeyboardInterrupt, ExternalShutdownException):
        pass
    finally:
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()
    return 0


if __name__ == '__main__':
    sys.exit(main())
