"""Measure the ROS-side loop: topic rates and command-to-motion latency.

  ros2 run openbeam_bridge latency_probe --ros-args -p seconds:=10 -p steps:=10

It listens to /joint_states, /clock, /tf and /camera/image_raw/compressed for
`seconds`, then steps wrist_roll back and forth `steps` times on
/joint_commands and times (wall clock) until /joint_states shows it moving.
Prints one JSON report.
"""

from __future__ import annotations

import json
import statistics
import time

import rclpy
from rclpy.node import Node
from rosgraph_msgs.msg import Clock
from sensor_msgs.msg import CompressedImage, JointState
from std_msgs.msg import Float64MultiArray
from tf2_msgs.msg import TFMessage

from . import so101


class Probe(Node):
    def __init__(self):
        super().__init__("openbeam_latency_probe")
        self.declare_parameter("seconds", 10.0)
        self.declare_parameter("steps", 10)
        self.declare_parameter("step_rad", 0.4)
        self.stamps = {"joint_states": [], "clock": [], "tf": [], "camera": []}
        self.sim = []
        self.last_js = None
        self.create_subscription(JointState, "joint_states", self._js, 50)
        self.create_subscription(Clock, "/clock", lambda m: self.stamps["clock"].append(time.monotonic()), 50)
        self.create_subscription(TFMessage, "/tf", lambda m: self.stamps["tf"].append(time.monotonic()), 50)
        self.create_subscription(CompressedImage, "camera/image_raw/compressed",
                                 lambda m: self.stamps["camera"].append(time.monotonic()), 5)
        self.pub = self.create_publisher(Float64MultiArray, "joint_commands", 10)

    def _js(self, msg):
        now = time.monotonic()
        self.stamps["joint_states"].append(now)
        self.sim.append((now, msg.header.stamp.sec + msg.header.stamp.nanosec * 1e-9))
        self.last_js = (now, dict(zip(msg.name, msg.position)))


def rate(stamps):
    if len(stamps) < 2:
        return 0.0
    return (len(stamps) - 1) / (stamps[-1] - stamps[0])


def main(args=None):
    rclpy.init(args=args)
    node = Probe()
    seconds = float(node.get_parameter("seconds").value)
    end = time.monotonic() + seconds
    while time.monotonic() < end:
        rclpy.spin_once(node, timeout_sec=0.05)
    report = {name: {"rate_hz": round(rate(s), 2), "n": len(s)} for name, s in node.stamps.items()}
    if len(node.sim) > 2:
        wall = node.sim[-1][0] - node.sim[0][0]
        report["real_time_factor"] = round((node.sim[-1][1] - node.sim[0][1]) / wall, 3) if wall else None
        gaps = [b[0] - a[0] for a, b in zip(node.sim, node.sim[1:])]
        report["joint_states_gap_ms"] = {"p50": round(statistics.median(gaps) * 1e3, 2),
                                         "max": round(max(gaps) * 1e3, 2)}
    latencies = []
    if node.last_js is not None:
        step = float(node.get_parameter("step_rad").value)
        for i in range(int(node.get_parameter("steps").value)):
            current = node.last_js[1]
            base = [current.get(n, 0.0) for n in so101.JOINT_NAMES]
            start = current["wrist_roll"]
            target = step if i % 2 == 0 else -step
            base[so101.JOINT_NAMES.index("wrist_roll")] = target
            sent = time.monotonic()
            node.pub.publish(Float64MultiArray(data=base))
            moved = None
            while time.monotonic() - sent < 3.0:
                rclpy.spin_once(node, timeout_sec=0.005)
                if node.last_js and node.last_js[0] > sent and abs(node.last_js[1]["wrist_roll"] - start) > 0.01:
                    moved = node.last_js[0]
                    break
            if moved:
                latencies.append(moved - sent)
            settle = time.monotonic() + 1.5
            while time.monotonic() < settle:
                rclpy.spin_once(node, timeout_sec=0.02)
    if latencies:
        ordered = sorted(latencies)
        report["command_to_motion_ms"] = {"n": len(ordered), "p50": round(statistics.median(ordered) * 1e3, 1),
                                          "max": round(ordered[-1] * 1e3, 1), "min": round(ordered[0] * 1e3, 1)}
    print(json.dumps(report, indent=2))
    node.destroy_node()
    rclpy.shutdown()


if __name__ == "__main__":
    main()
