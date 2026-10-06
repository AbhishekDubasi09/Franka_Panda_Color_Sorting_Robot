#!/usr/bin/env python3
"""Compare detectors against Gazebo ground truth on randomized box layouts.

Run next to a running simulation, with the two detectors publishing on
separate topics:

  ros2 run panda_vision color_detector_legacy --ros-args \
      -r /color_coordinates:=/legacy_coords
  ros2 run panda_vision color_detector --ros-args \
      -r /color_coordinates:=/geom_coords -p plane_z:=0.06 \
      -p show_window:=false
  ros2 run panda_vision evaluate_detectors --ros-args -p layouts:=30

Each layout moves the three pucks with the Gazebo set_pose service, waits for
them to settle, and records the median detection per colour. Error is the
planar (x, y) distance in the robot base frame between the detection and the
true puck centre.
"""
import json
import subprocess
import time

import numpy as np
import rclpy
import tf2_ros
import tf_transformations
from rclpy.duration import Duration
from rclpy.node import Node
from std_msgs.msg import String

WORLD = "lab_world"
MODEL = {"R": "red_puck", "G": "green_puck", "B": "blue_puck"}
PUCK_CENTER_WORLD_Z = 0.38  # resting height on the table (top at z = 0.35)
NOMINAL_BASE_XY = {"R": (0.6, 0.0), "G": (0.6, -0.2), "B": (0.6, 0.2)}


def set_model_world_pose(color, wx, wy, wz):
    req = (f'name: "{MODEL[color]}", position: '
           f'{{x: {wx}, y: {wy}, z: {wz}}}, orientation: {{w: 1}}')
    subprocess.run(
        ["ign", "service", "-s", f"/world/{WORLD}/set_pose",
         "--reqtype", "ignition.msgs.Pose", "--reptype",
         "ignition.msgs.Boolean", "--timeout", "3000", "--req", req],
        check=True, capture_output=True)


class Evaluator(Node):
    def __init__(self):
        super().__init__('evaluate_detectors')
        self.declare_parameter('layouts', 30)
        self.declare_parameter('seed', 0)
        self.declare_parameter('out', 'detector_eval.json')
        self.tf_buffer = tf2_ros.Buffer()
        self.tf_listener = tf2_ros.TransformListener(self.tf_buffer, self)
        self.samples = {"legacy": [], "geometric": []}
        self.create_subscription(
            String, '/legacy_coords', lambda m: self.record("legacy", m), 50)
        self.create_subscription(
            String, '/geom_coords', lambda m: self.record("geometric", m), 50)

    def record(self, name, msg):
        c, x, y, z = msg.data.split(",")
        self.samples[name].append((c, float(x), float(y)))

    def spin_for(self, seconds):
        end = time.time() + seconds
        while time.time() < end:
            rclpy.spin_once(self, timeout_sec=0.05)

    def world_to_base(self):
        t = self.tf_buffer.lookup_transform(
            'world', 'panda_link0', rclpy.time.Time(),
            timeout=Duration(seconds=5.0))
        T = tf_transformations.quaternion_matrix([
            t.transform.rotation.x, t.transform.rotation.y,
            t.transform.rotation.z, t.transform.rotation.w])
        T[:3, 3] = [t.transform.translation.x, t.transform.translation.y,
                    t.transform.translation.z]
        return np.linalg.inv(T)

    def run_layout(self, base_xy, W2B):
        base_to_world = np.linalg.inv(W2B)
        truth = {}
        for c, (bx, by) in base_xy.items():
            # Centre height in the base frame is irrelevant for planar
            # error; place the box at its resting height in world.
            wx, wy, _ = (base_to_world @ [bx, by, 0.0, 1.0])[:3]
            set_model_world_pose(c, wx, wy, PUCK_CENTER_WORLD_Z)
            truth[c] = np.array([bx, by])
        self.spin_for(2.5)                    # settle
        for k in self.samples:
            self.samples[k].clear()
        self.spin_for(2.5)                    # collect
        errs = {}
        for name, samples in self.samples.items():
            errs[name] = {}
            for c in truth:
                pts = np.array([[x, y] for (cc, x, y) in samples if cc == c])
                if len(pts) == 0:
                    errs[name][c] = None
                    continue
                est = np.median(pts, axis=0)
                errs[name][c] = float(np.linalg.norm(est - truth[c]))
        return errs

    def run(self):
        rng = np.random.default_rng(int(self.get_parameter('seed').value))
        n = int(self.get_parameter('layouts').value)
        self.spin_for(1.0)
        W2B = self.world_to_base()
        layouts = [dict(NOMINAL_BASE_XY)]
        while len(layouts) < n + 1:
            pts = rng.uniform([0.45, -0.30], [0.75, 0.30], size=(3, 2))
            d = [np.linalg.norm(pts[i] - pts[j])
                 for i in range(3) for j in range(i + 1, 3)]
            if min(d) > 0.12:
                layouts.append({c: tuple(p) for c, p in zip("RGB", pts)})
        results = []
        for i, lay in enumerate(layouts):
            errs = self.run_layout(lay, W2B)
            results.append({"layout": {c: list(v) for c, v in lay.items()},
                            "errors_m": errs})
            self.get_logger().info(f"layout {i}: {errs}")
        # restore the original scene
        self.run_layout(NOMINAL_BASE_XY, W2B)
        with open(self.get_parameter('out').value, 'w') as f:
            json.dump(results, f, indent=1)
        summarize(results, self.get_logger().info)


def summarize(results, log=print):
    for name in ("legacy", "geometric"):
        e = [v for r in results for v in r["errors_m"][name].values()
             if v is not None]
        miss = sum(v is None for r in results
                   for v in r["errors_m"][name].values())
        e = np.array(e) * 1000.0
        log(f"{name:10s} n={len(e)} missed={miss} mean={e.mean():.1f} mm "
            f"median={np.median(e):.1f} mm p95={np.percentile(e, 95):.1f} mm "
            f"max={e.max():.1f} mm")


def main(args=None):
    rclpy.init(args=args)
    node = Evaluator()
    try:
        node.run()
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
