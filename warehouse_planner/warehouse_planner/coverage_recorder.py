#!/usr/bin/env python3
"""
coverage_recorder.py -- an independent referee for every run.

It only listens (TF, /map, /mission_command, /mission_state, /dialogue) and never
commands anything. So the baseline and every later version get measured by the
same code, and the planner never grades its own homework.

Each run gets ~/runs/<date_time>/ (and ~/runs/latest points at it):
  trail.csv      10 Hz pose: wall time, sim time, x, y, yaw (map frame)
  coverage.csv   1 Hz: painted cells, passes, overlap, the planner's own %, mission state
  events.jsonl   every command, every mission-state change, every /dialogue turn
  summary.json   run totals -- rewritten every 30 s and on Ctrl-C
  passes.npy     pass-count grid, map.npy  occupancy grid  (for plot_run.py)
and publishes /coverage_passes (OccupancyGrid, transient local) for RViz:
  Add -> By topic -> /coverage_passes -> Map
  Color Scheme: costmap    Durability Policy: Transient Local

Start it after the map is up and before the first command:
  ros2 run warehouse_planner coverage_recorder --ros-args -p use_sim_time:=true
(drop use_sim_time if the rest of your stack runs on wall time)
"""
import array
import csv
import json
import math
import os
import time

import numpy as np
import rclpy
from nav_msgs.msg import OccupancyGrid
from rclpy.node import Node
from rclpy.qos import DurabilityPolicy, QoSProfile, ReliabilityPolicy
from rclpy.time import Time
from std_msgs.msg import String
from tf2_ros import TransformException
from tf2_ros.buffer import Buffer
from tf2_ros.transform_listener import TransformListener

try:
    from rclpy.executors import ExternalShutdownException
except ImportError:  # older rclpy
    class ExternalShutdownException(Exception):
        pass

try:
    from warehouse_planner.coverage_meter import DriveMeter, PassGrid
except ImportError:  # running straight from the source folder
    from coverage_meter import DriveMeter, PassGrid

# Mission-state fields that mark a real change (x, y and coverage_pct move every tick).
STATE_KEYS = ("zone", "goal", "total", "paused", "done", "queue", "cleaned", "blocked", "rescuing")


def parse(text):
    """JSON if it is JSON, otherwise the raw string -- never lose a message."""
    try:
        return json.loads(text)
    except (ValueError, TypeError):
        return text


class CoverageRecorder(Node):

    def __init__(self):
        super().__init__("coverage_recorder")
        self.declare_parameter("runs_dir", os.path.expanduser("~/runs"))
        self.declare_parameter("map_frame", "map")
        self.declare_parameter("base_frame", "base_link")
        self.declare_parameter("clean_width", 0.52)   # set to the brush width coverage_nav2 paints with
        self.declare_parameter("pass_gap", 3.0)
        self.declare_parameter("rate_hz", 10.0)
        self.map_frame = self.get_parameter("map_frame").value
        self.base_frame = self.get_parameter("base_frame").value
        self.clean_width = float(self.get_parameter("clean_width").value)
        self.pass_gap = float(self.get_parameter("pass_gap").value)

        runs = os.path.expanduser(self.get_parameter("runs_dir").value)
        self.run_dir = os.path.join(runs, time.strftime("%Y-%m-%d_%H%M%S"))
        os.makedirs(self.run_dir, exist_ok=True)
        latest = os.path.join(runs, "latest")
        try:
            if os.path.lexists(latest):
                os.remove(latest)
            os.symlink(self.run_dir, latest)
        except OSError:
            pass

        # Line-buffered, so a crash loses at most the line being written.
        self.trail_f = open(os.path.join(self.run_dir, "trail.csv"), "w", buffering=1, newline="")
        self.cov_f = open(os.path.join(self.run_dir, "coverage.csv"), "w", buffering=1, newline="")
        self.events_f = open(os.path.join(self.run_dir, "events.jsonl"), "w", buffering=1)
        self.trail = csv.writer(self.trail_f)
        self.cov = csv.writer(self.cov_f)
        self.trail.writerow(["wall_t", "sim_t", "x", "y", "yaw"])
        self.cov.writerow(["wall_t", "x", "y", "painted_cells", "painted_pct_of_floor",
                           "total_passes", "overlap_ratio", "planner_pct",
                           "zone", "goal", "total", "paused"])

        self.grid = None
        self.meter = None
        self.map_info = None
        self.last_pose = None
        self.state = {}
        self.state_sig = None
        self.counts = {"commands": 0, "state_changes": 0, "dialogue_turns": 0}
        self.t0 = time.time()
        self.finished = False

        latched = QoSProfile(depth=1, reliability=ReliabilityPolicy.RELIABLE,
                             durability=DurabilityPolicy.TRANSIENT_LOCAL)
        self.create_subscription(OccupancyGrid, "/map", self.on_map, latched)
        self.create_subscription(String, "/mission_command", self.on_command, 10)
        self.create_subscription(String, "/mission_state", self.on_state, 10)
        self.create_subscription(String, "/dialogue", self.on_dialogue, 10)
        self.heat_pub = self.create_publisher(OccupancyGrid, "/coverage_passes", latched)

        self.tf_buffer = Buffer()
        self.tf_listener = TransformListener(self.tf_buffer, self)
        self.create_timer(1.0 / float(self.get_parameter("rate_hz").value), self.tick)
        self.create_timer(1.0, self.every_second)
        self.create_timer(30.0, self.save_summary)
        self.get_logger().info(f"RECORDER -> {self.run_dir}  (waiting for /map)")

    # ---------------------------------------------------------------- inputs
    def on_map(self, msg):
        if self.grid is not None:
            return                                   # static map: keep the first one
        info = msg.info
        self.grid = PassGrid(info.width, info.height, info.resolution,
                             info.origin.position.x, info.origin.position.y,
                             msg.data, self.clean_width, self.pass_gap)
        self.meter = DriveMeter(self.grid.cells_per_metre, self.grid.footprint_cells)
        self.map_info = info
        np.save(os.path.join(self.run_dir, "map.npy"),
                np.asarray(msg.data, dtype=np.int8).reshape(info.height, info.width))
        self.get_logger().info(
            f"RECORDER map {info.width}x{info.height} @ {info.resolution:.3f} m, "
            f"{self.grid.floor_cells} floor cells | brush {self.clean_width:.2f} m, "
            f"pass gap {self.pass_gap:.1f} s | TF {self.map_frame}->{self.base_frame}")

    def tick(self):
        if self.grid is None:
            return
        sim_t = self.get_clock().now().nanoseconds * 1e-9
        if sim_t == 0.0:
            return                                   # use_sim_time on, /clock not flowing yet
        try:
            tf = self.tf_buffer.lookup_transform(self.map_frame, self.base_frame, Time())
        except TransformException as e:
            self.get_logger().warn(f"RECORDER no TF {self.map_frame}->{self.base_frame}: {e}",
                                   throttle_duration_sec=5.0)
            return
        p, q = tf.transform.translation, tf.transform.rotation
        yaw = math.atan2(2.0 * (q.w * q.z + q.x * q.y), 1.0 - 2.0 * (q.y * q.y + q.z * q.z))
        new = self.grid.paint(sim_t, p.x, p.y)
        self.meter.update(sim_t, p.x, p.y, new)
        self.last_pose = (p.x, p.y, yaw)
        self.trail.writerow([f"{time.time():.3f}", f"{sim_t:.3f}",
                             f"{p.x:.3f}", f"{p.y:.3f}", f"{yaw:.3f}"])

    def on_command(self, msg):
        self.counts["commands"] += 1
        self.event("command", parse(msg.data))

    def on_state(self, msg):
        data = parse(msg.data)
        if not isinstance(data, dict):               # not JSON: log raw text when it changes
            if msg.data != self.state_sig:
                self.state_sig = msg.data
                self.event("state_raw", msg.data)
            return
        self.state = data
        snap = {k: data.get(k) for k in STATE_KEYS}
        sig = json.dumps(snap, sort_keys=True, default=str)
        if sig != self.state_sig:
            self.state_sig = sig
            self.counts["state_changes"] += 1
            self.event("state", snap)

    def on_dialogue(self, msg):
        self.counts["dialogue_turns"] += 1
        self.event("dialogue", parse(msg.data))

    # ---------------------------------------------------------------- outputs
    def event(self, kind, data):
        self.events_f.write(json.dumps({"t": round(time.time(), 3), "type": kind, "data": data},
                                       default=str) + "\n")

    def every_second(self):
        if self.grid is None:
            return
        s, st = self.grid.stats(), self.state
        x, y = (self.last_pose[0], self.last_pose[1]) if self.last_pose else (float("nan"),) * 2
        self.cov.writerow([f"{time.time():.3f}", f"{x:.3f}", f"{y:.3f}",
                           s["painted_cells"], s["painted_pct_of_floor"], s["total_passes"],
                           s["overlap_ratio"], st.get("coverage_pct", ""), st.get("zone", ""),
                           st.get("goal", ""), st.get("total", ""), st.get("paused", "")])
        msg = OccupancyGrid()
        msg.header.frame_id = self.map_frame
        msg.header.stamp = self.get_clock().now().to_msg()
        msg.info = self.map_info
        msg.data = array.array("b", self.grid.heat_values().tobytes())
        self.heat_pub.publish(msg)

    def save_summary(self):
        if self.grid is None:
            return
        info = self.map_info
        summary = {
            "run_dir": self.run_dir,
            "started_wall_t": round(self.t0, 3),
            "saved_wall_t": round(time.time(), 3),
            "duration_s": round(time.time() - self.t0, 1),
            "params": {"clean_width": self.clean_width, "pass_gap": self.pass_gap,
                       "map_frame": self.map_frame, "base_frame": self.base_frame},
            "map": {"width": info.width, "height": info.height, "resolution": info.resolution,
                    "origin": [info.origin.position.x, info.origin.position.y]},
            "coverage": self.grid.stats(),
            "driving": self.meter.stats(),
            "counts": dict(self.counts),
            "planner_final": {k: self.state.get(k)
                              for k in ("coverage_pct", "cleaned", "partial", "blocked", "done")},
        }
        path = os.path.join(self.run_dir, "summary.json")
        with open(path + ".tmp", "w") as f:
            json.dump(summary, f, indent=2, default=str)
        os.replace(path + ".tmp", path)               # never leaves a half-written summary
        np.save(os.path.join(self.run_dir, "passes.npy"), self.grid.passes)
        return summary

    def finish(self):
        if self.finished:
            return
        self.finished = True
        if self.meter is not None:
            self.meter.flush()
        summary = self.save_summary()
        for f in (self.trail_f, self.cov_f, self.events_f):
            f.close()
        if summary:
            c, d = summary["coverage"], summary["driving"]
            print(f"\nRECORDER saved {self.run_dir}\n"
                  f"  painted {c['painted_cells']} cells ({c['painted_pct_of_floor']}% of floor), "
                  f"overlap {c['overlap_ratio']}, max passes {c['max_passes']}\n"
                  f"  drove {d['distance_m']} m, sweep efficiency {d['sweep_efficiency']}, "
                  f"unproductive driving {d['unproductive_fraction']:.0%}", flush=True)


def main(args=None):
    rclpy.init(args=args)
    node = CoverageRecorder()
    try:
        rclpy.spin(node)
    except (KeyboardInterrupt, ExternalShutdownException):
        pass
    finally:
        node.finish()
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()


if __name__ == "__main__":
    main()