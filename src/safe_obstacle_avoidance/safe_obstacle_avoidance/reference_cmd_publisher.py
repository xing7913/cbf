"""Publish a bounded constant reference command for the bundled Gazebo demo.

This node is only enabled by the launch file when it starts this repository's
own simulation. For a user's own robot, disable auto_demo and provide /ref_vel
from teleop or a navigation stack instead.
"""

from __future__ import annotations

import math

import rclpy
from geometry_msgs.msg import Twist
from nav_msgs.msg import Odometry
from rclpy.node import Node
from rclpy.qos import qos_profile_sensor_data


class ReferenceCommandPublisher(Node):
    def __init__(self) -> None:
        super().__init__("demo_reference_cmd_publisher")
        self.declare_parameter("odom_topic", "/odom")
        self.declare_parameter("reference_topic", "/ref_vel")
        self.declare_parameter("linear_speed", 0.15)
        self.declare_parameter("angular_speed", 0.0)
        self.declare_parameter("publish_rate", 10.0)
        self.declare_parameter("duration_s", 30.0)
        self.declare_parameter("max_path_length", 5.0)

        self.publisher = self.create_publisher(
            Twist, str(self.get_parameter("reference_topic").value), 10
        )
        self.create_subscription(
            Odometry,
            str(self.get_parameter("odom_topic").value),
            self._odom_callback,
            qos_profile_sensor_data,
        )

        rate = float(self.get_parameter("publish_rate").value)
        if rate <= 0.0:
            raise ValueError("publish_rate must be positive")
        self.duration_s = float(self.get_parameter("duration_s").value)
        self.max_path_length = float(self.get_parameter("max_path_length").value)
        if self.duration_s < 0.0 or self.max_path_length < 0.0:
            raise ValueError("duration_s and max_path_length cannot be negative")

        self.started_ns: int | None = None
        self.last_xy: tuple[float, float] | None = None
        self.path_length = 0.0
        self.finished = False
        self.timer = self.create_timer(1.0 / rate, self._publish)
        self.get_logger().info("Auto-demo reference publisher ready on /ref_vel")

    def _odom_callback(self, msg: Odometry) -> None:
        pose = msg.pose.pose.position
        xy = (float(pose.x), float(pose.y))
        if self.last_xy is None:
            self.last_xy = xy
            self.started_ns = self.get_clock().now().nanoseconds
            return
        self.path_length += math.hypot(xy[0] - self.last_xy[0], xy[1] - self.last_xy[1])
        self.last_xy = xy

    def _publish(self) -> None:
        command = Twist()
        if self.started_ns is None:
            self.publisher.publish(command)
            return

        elapsed = (self.get_clock().now().nanoseconds - self.started_ns) / 1e9
        duration_reached = self.duration_s > 0.0 and elapsed >= self.duration_s
        distance_reached = self.max_path_length > 0.0 and self.path_length >= self.max_path_length
        if duration_reached or distance_reached:
            if not self.finished:
                self.finished = True
                self.get_logger().info(
                    f"Auto demo complete: elapsed={elapsed:.1f}s, path={self.path_length:.2f}m"
                )
        else:
            command.linear.x = float(self.get_parameter("linear_speed").value)
            command.angular.z = float(self.get_parameter("angular_speed").value)
        self.publisher.publish(command)


def main(args=None) -> None:
    rclpy.init(args=args)
    node = ReferenceCommandPublisher()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.publisher.publish(Twist())
        node.destroy_node()
        rclpy.shutdown()


if __name__ == "__main__":
    main()
