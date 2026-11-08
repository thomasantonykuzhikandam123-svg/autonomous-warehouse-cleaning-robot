#!/usr/bin/env python3
"""
llm_commander — the ROS front end for the LangGraph robot agent.

Commands AND questions both work:

    clean aisle 3        -> a mission is built and sent
    how much is left?    -> answered from live /mission_state
    aisle 2 is blocked   -> becomes skip aisle_2
    skip it              -> resolves from the previous turn

Routing, memory and state-grounded answers live in robot_agent.py. This file
owns the ROS plumbing and the terminal loop.

    ros2 run warehouse_planner llm_commander

Type at it and the robot responds. Type 'q' to quit.

Where things run:
  Ollama    -> on the MAC. The Ubuntu VM has 8 GB and is already running
               Gazebo, Nav2 and RViz; a 2.5 GB model competing with the
               physics loop produces stalls that look exactly like
               navigation bugs. Keep it off the sim.
  this node -> in the VM, because it needs ROS. It reaches the Mac over
               the VM's host network.

Two things that will bite you on the Mac side:

  1. Ollama binds to 127.0.0.1 by default and will refuse the VM outright.
     Start it as:      OLLAMA_HOST=0.0.0.0 ollama serve
  2. You need the Mac's address as seen FROM the VM. Find it with:
         ip route | awk '/default/ {print $3}'
     then pass it:
         ros2 run warehouse_planner llm_commander --ros-args \\
              -p ollama_host:=http://<that-address>:11434

Nothing this node produces reaches Nav2 unchecked: it validates with
mission_command.parse_llm_output, and coverage_nav2 validates again on
receipt. The model proposes; two layers of plain logic decide.
"""

import json
import select
import sys
import threading
import urllib.error
import urllib.request

import rclpy
from rclpy.node import Node
from std_msgs.msg import String

from warehouse_planner.robot_agent import build_graph, describe


class LLMCommander(Node):

    def __init__(self):
        super().__init__('llm_commander')
        self.declare_parameter('ollama_host', 'http://10.0.2.2:11434')
        self.declare_parameter('model', 'llama3.2')
        self.declare_parameter('timeout', 90.0)
        self.host = self.get_parameter('ollama_host').value.rstrip('/')
        self.model = self.get_parameter('model').value
        self.timeout = float(self.get_parameter('timeout').value)

        self.pub = self.create_publisher(String, '/mission_command', 10)
        self.dialogue_pub = self.create_publisher(String, '/dialogue', 10)
        self.create_subscription(String, '/mission_state', self.on_state, 10)
        self._state = None

        self.graph = build_graph(self.ask)
        self.history = []            # [(role, text)] -- lets "skip it" resolve
        self.get_logger().info(f'ollama: {self.host}  model: {self.model}')

    # ---------------- robot state, so you can see what you are commanding ----
    def on_state(self, msg):
        try:
            self._state = json.loads(msg.data)
        except Exception:
            pass

    def state_line(self):
        s = self._state
        if not s:
            return 'robot: (no /mission_state yet — is coverage_nav2 running?)'
        flag = (' PAUSED' if s.get('paused') else ' DONE' if s.get('done') else
                ' WAITING FOR A JOB' if s.get('awaiting_command') else '')      # v10.7
        tr = s.get('traffic') or {}       # v10.4: what the people guard is doing
        safety = ''
        if tr.get('state') == 'wait':
            safety = f"  | WAIT: {tr.get('say')}"
        elif tr.get('v_limit') is not None:
            safety = f"  | {tr.get('say')}"
        lanes = ''                        # v10.5: each aisle's lanes swept
        if s.get('swept'):
            lanes = '  | lanes ' + ' '.join(f"{z.replace('aisle_', 'a')} {p:.0f}%"
                                            for z, p in s['swept'].items())
            if s.get('patching'):
                lanes += ' (patching)'
        if s.get('safety_hold'):                 # v10.6
            safety = '  | SAFETY HOLD: the people guard is silent'
        elif s.get('nav2_down'):
            safety = '  | NAV2 DOWN: holding, retrying every 5 s'
        elif s.get('manoeuvre') == 'unstick':    # v10.7
            safety = '  | UNSTICK: backing off a rack, turning, trying again'
        return (f"robot: {s.get('zone')}  {s.get('goal')}/{s.get('total')}  "
                f"{s.get('coverage_pct')}% covered{flag}{lanes}{safety}")

    # ---------------- ollama ------------------------------------------------
    def check(self):
        try:
            with urllib.request.urlopen(f'{self.host}/api/tags', timeout=5.0) as r:
                names = [m['name'] for m in json.loads(r.read()).get('models', [])]
        except Exception as e:
            self.get_logger().error(
                f'cannot reach Ollama at {self.host}: {e}\n'
                f'  on the Mac:  OLLAMA_HOST=0.0.0.0 ollama serve\n'
                f'  host addr :  ip route | awk \'/default/ {{print $3}}\'')
            return False
        if not any(n.split(':')[0] == self.model for n in names):
            self.get_logger().error(
                f"'{self.model}' not pulled. Available: {names or '(none)'}")
            return False
        return True

    def ask(self, system, text):
        payload = {
            'model': self.model,
            'messages': [{'role': 'system', 'content': system},
                         {'role': 'user', 'content': text}],
            'stream': False,
            'options': {'temperature': 0.0},   # parsing, not writing
        }
        req = urllib.request.Request(
            f'{self.host}/api/chat',
            data=json.dumps(payload).encode(),
            headers={'Content-Type': 'application/json'})
        with urllib.request.urlopen(req, timeout=self.timeout) as r:
            return json.loads(r.read())['message']['content']

    # ---------------- the loop ----------------------------------------------
    def repl(self):
        print('\nTalk to the robot. Commands AND questions both work. q to quit.')
        print('  clean aisle 3        how much is left?        stop')
        print('  aisle 2 is blocked   skip it                  are we done yet')
        print('  do the front area    resume                   clean everything')
        print('  why did you stop?    what are you waiting for?  (v10.4: the safety layer answers)\n')
        while rclpy.ok():
            try:
                print(self.state_line())
                text = input('you > ').strip()
            except (EOFError, KeyboardInterrupt):
                break
            if not text:
                continue
            if text.lower() in ('q', 'quit', 'exit'):
                break

            # PASTE GUARD. input() hands back one line at a time, so pasting
            # four instructions ran four missions in three seconds, each one
            # cancelling the one before. Anything that arrived in the same
            # instant as this line is collected and shown before it is run.
            batch = [text]
            try:
                while select.select([sys.stdin], [], [], 0.05)[0]:
                    extra = sys.stdin.readline()
                    if not extra:
                        break
                    if extra.strip():
                        batch.append(extra.strip())
            except (OSError, ValueError):
                pass
            if len(batch) > 1:
                print(f'  {len(batch)} lines arrived together:')
                for i, b in enumerate(batch, 1):
                    print(f'     {i}. {b}')
                print('  Each new cleaning order replaces the one before it, so')
                print('  only the LAST would actually finish.')
                try:
                    ans = input('  run all / last only / cancel?  [a/l/C] ').strip().lower()
                except (EOFError, KeyboardInterrupt):
                    break
                if ans in ('a', 'all'):
                    pass
                elif ans in ('l', 'last'):
                    batch = batch[-1:]
                else:
                    print('  cancelled -- nothing sent\n')
                    continue

            for line in batch:
                if len(batch) > 1:
                    print(f'you > {line}')
                self._handle(line)

    def _log_turn(self, record):
        """One JSON object per operator turn on /dialogue; coverage_recorder files it."""
        m = String()
        m.data = json.dumps(record, default=str)
        self.dialogue_pub.publish(m)

    def _handle(self, text):
        state = self._state or {}      # the exact state the agent decides on
        try:
            out = self.graph.invoke({'text': text,
                                     'history': self.history,
                                     'robot': state})
        except Exception as e:
            print(f'  agent error: {e}\n')
            self._log_turn({'text': text, 'state': state, 'error': str(e)})
            return

        for n in out.get('notes') or []:
            print(f'  repaired : {n}')
        print(f'  route    : {out.get("route")} ({out.get("route_why")})')

        cmd = out.get('command')
        reply = out.get('reply', '')
        if cmd is None:
            print(f'  reply    : {reply}\n')
        else:
            m = String()
            m.data = json.dumps(cmd)
            self.pub.publish(m)
            if reply:
                print(f'  reply    : {reply}')
            # a chained instruction is {"steps": [...]} with no top-level
            # 'zones'; printing it as if it had one crashed the REPL
            steps = cmd.get('steps') or [cmd]
            for i, st in enumerate(steps, 1):
                zs = ', '.join(st.get('zones') or []) or '-'
                tag = f'SENT {i}/{len(steps)}' if len(steps) > 1 else 'SENT    '
                print(f'  {tag} : action={st.get("action")}  zones=[{zs}]')
            print()

        self._log_turn({
            'text': text,
            'route': out.get('route'),
            'route_why': out.get('route_why'),
            'notes': out.get('notes') or [],
            'reply': reply,
            'sent': cmd,
            'state': state,
            'extra': {k: v for k, v in out.items()      # anything else the graph returns
                      if k not in ('text', 'history', 'robot', 'route', 'route_why',
                                   'notes', 'reply', 'command')},
        })
        self.history.append(('user', text))
        self.history.append(('assistant', reply))
        self.history = self.history[-8:]

def main():
    rclpy.init()
    node = LLMCommander()
    if not node.check():
        node.destroy_node()
        rclpy.shutdown()
        return 1
    # spin in the background so /mission_state keeps arriving while we block
    # on input(); the REPL owns the main thread.
    threading.Thread(target=rclpy.spin, args=(node,), daemon=True).start()
    try:
        node.repl()
    except KeyboardInterrupt:
        pass
    finally:
        # repl() used to call rclpy.shutdown() itself, and main called it
        # again -- hence the 'rcl_shutdown already called' traceback on Ctrl-C.
        # Shut down in exactly one place.
        try:
            if rclpy.ok():
                rclpy.shutdown()
        except Exception:
            pass
    return 0


if __name__ == '__main__':
    sys.exit(main())