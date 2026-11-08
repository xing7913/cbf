#!/usr/bin/env python3
"""Record odometry, requested velocity and CBF-filtered velocity to CSV."""

from __future__ import annotations

import argparse
import csv
import math
import time
from datetime import datetime
from pathlib import Path

import rclpy
from geometry_msgs.msg import Twist
from nav_msgs.msg import Odometry
from rclpy.node import Node
from rclpy.qos import qos_profile_sensor_data


def yaw_from_quaternion(q: object) -> float:
    return math.atan2(
        2.0 * (q.w * q.z + q.x * q.y),
        1.0 - 2.0 * (q.y * q.y + q.z * q.z),
    )


class RunRecorder(Node):
    def __init__(self, duration: float, output_path: Path) -> None:
        super().__init__("safe_obstacle_run_recorder")
        self.duration = duration
        self.output_path = output_path
        self.started = time.monotonic()
        self.odom: Odometry | None = None
        self.reference: Twist | None = None
        self.command: Twist | None = None
        self.rows: list[dict[str, float]] = []
        self.create_subscription(Odometry, "/odom", self._odom_cb, qos_profile_sensor_data)
        self.create_subscription(Twist, "/ref_vel", self._ref_cb, 10)
        self.create_subscription(Twist, "/cmd_vel", self._cmd_cb, 10)
        self.timer = self.create_timer(0.1, self._sample)
        self.get_logger().info(f"Recording for {duration:.1f}s to {output_path}")

    def _odom_cb(self, msg: Odometry) -> None:
        self.odom = msg

    def _ref_cb(self, msg: Twist) -> None:
        self.reference = msg

    def _cmd_cb(self, msg: Twist) -> None:
        self.command = msg

    def _sample(self) -> None:
        elapsed = time.monotonic() - self.started
        if self.odom is not None:
            pose = self.odom.pose.pose
            ref = self.reference or Twist()
            cmd = self.command or Twist()
            self.rows.append({
                "time_s": elapsed,
                "x": float(pose.position.x),
                "y": float(pose.position.y),
                "yaw": yaw_from_quaternion(pose.orientation),
                "ref_v": float(ref.linear.x),
                "ref_w": float(ref.angular.z),
                "cmd_v": float(cmd.linear.x),
                "cmd_w": float(cmd.angular.z),
            })
        if elapsed >= self.duration:
            self._save()
            self.get_logger().info(f"Saved {len(self.rows)} samples to {self.output_path}")
            rclpy.shutdown()

    def _save(self) -> None:
        self.output_path.parent.mkdir(parents=True, exist_ok=True)
        fields = ["time_s", "x", "y", "yaw", "ref_v", "ref_w", "cmd_v", "cmd_w"]
        with self.output_path.open("w", newline="", encoding="utf-8") as stream:
            writer = csv.DictWriter(stream, fieldnames=fields)
            writer.writeheader()
            writer.writerows(self.rows)


def main() -> None:
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--duration", type=float, default=60.0, help="recording duration in seconds")
    parser.add_argument("--output", type=Path, default=None, help="CSV output path")
    args = parser.parse_args()
    if args.duration <= 0.0:
        parser.error("--duration must be positive")
    output = args.output or Path("results") / f"run_{datetime.now():%Y%m%d_%H%M%S}.csv"
    rclpy.init()
    node = RunRecorder(args.duration, output)
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        node._save()
        node.get_logger().info(f"Saved partial run to {output}")
    finally:
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()


if __name__ == "__main__":
    main()
