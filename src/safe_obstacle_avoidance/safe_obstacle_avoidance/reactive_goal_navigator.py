"""Reactive goal-directed /ref_vel publisher with depth based obstacle steering.

For simulation and research demonstration only. CBF-QP remains the only
publisher of /cmd_vel. If odometry or depth input expires, publish STOP.
"""
from __future__ import annotations

import math
import numpy as np
import rclpy
from cv_bridge import CvBridge
from geometry_msgs.msg import Twist
from nav_msgs.msg import Odometry
from rclpy.node import Node
from rclpy.qos import qos_profile_sensor_data
from sensor_msgs.msg import Image

from .reactive_planner import choose_command, wrap_angle


class ReactiveGoalNavigator(Node):
    def __init__(self):
        super().__init__("reactive_goal_navigator")
        defaults = {
            "odom_topic": "/odom",
            "depth_topic": "/camera/depth/image_raw",
            "reference_topic": "/ref_vel",
            "goal_x": 5.5,
            "goal_y": 0.0,
            "goal_tolerance": 0.35,
            "cruise_speed": 0.15,
            "turn_speed": 0.85,
            "caution_distance": 1.65,
            "stop_distance": 0.55,
            "depth_row_fraction": 0.50,
            "sensor_timeout": 0.8,
            "update_rate": 10.0,
        }
        for name, value in defaults.items():
            self.declare_parameter(name, value)
        self.cv = CvBridge()
        self.odom = None
        self.depth_row = None
        self.odom_ns = None
        self.depth_ns = None
        self.side = 0
        self.reached = False
        self.pub = self.create_publisher(Twist, str(self.get_parameter("reference_topic").value), 10)
        self.create_subscription(Odometry, str(self.get_parameter("odom_topic").value),
                                 self.on_odom, qos_profile_sensor_data)
        self.create_subscription(Image, str(self.get_parameter("depth_topic").value),
                                 self.on_depth, qos_profile_sensor_data)
        rate = float(self.get_parameter("update_rate").value)
        if rate <= 0:
            raise ValueError("update_rate must be > 0")
        self.create_timer(1.0 / rate, self.step)
        self.get_logger().info(
            f"Goal navigator active: ({self.p('goal_x'):.2f}, {self.p('goal_y'):.2f})"
        )

    def p(self, name):
        return float(self.get_parameter(name).value)

    def now_ns(self):
        return self.get_clock().now().nanoseconds

    def on_odom(self, msg):
        self.odom = msg
        self.odom_ns = self.now_ns()

    def on_depth(self, msg):
        try:
            a = np.asarray(self.cv.imgmsg_to_cv2(msg, desired_encoding="passthrough"))
            if a.ndim != 2:
                return
            fraction = max(0.0, min(1.0, self.p("depth_row_fraction")))
            row = a[min(a.shape[0]-1, int(a.shape[0]*fraction)), :].astype(float)
            if msg.encoding.upper() in ("16UC1", "MONO16"):
                row *= 0.001
            elif msg.encoding.upper() not in ("32FC1", "64FC1"):
                self.get_logger().warning(
                    f"Unexpected depth encoding {msg.encoding}; refusing to navigate",
                    throttle_duration_sec=5.0,
                )
                return
            self.depth_row = row
            self.depth_ns = self.now_ns()
        except (ValueError, RuntimeError, TypeError) as exc:
            self.get_logger().warning(f"Depth conversion failed: {exc}", throttle_duration_sec=5.0)

    def fresh(self, stamp):
        if stamp is None:
            return False
        age = (self.now_ns() - stamp) / 1e9
        return 0 <= age < self.p("sensor_timeout")

    def step(self):
        cmd = Twist()  # stop by default; CBF-QP still filters before /cmd_vel
        if not (self.odom is not None and self.depth_row is not None
                and self.fresh(self.odom_ns) and self.fresh(self.depth_ns)):
            self.pub.publish(cmd)
            self.get_logger().warning("Waiting for fresh odometry and depth",
                                      throttle_duration_sec=5.0)
            return
        pose = self.odom.pose.pose
        dx = self.p("goal_x") - pose.position.x
        dy = self.p("goal_y") - pose.position.y
        distance = math.hypot(dx, dy)
        if distance <= self.p("goal_tolerance"):
            if not self.reached:
                self.get_logger().info("Goal reached; stopping.")
                self.reached = True
            self.pub.publish(cmd)
            return
        q = pose.orientation
        yaw = math.atan2(2*(q.w*q.z+q.x*q.y), 1-2*(q.y*q.y+q.z*q.z))
        heading_error = wrap_angle(math.atan2(dy, dx) - yaw)
        linear, angular, self.side, clearance = choose_command(
            self.depth_row, heading_error,
            preferred_side=self.side,
            cruise_speed=self.p("cruise_speed"),
            turn_speed=self.p("turn_speed"),
            caution_distance=self.p("caution_distance"),
            stop_distance=self.p("stop_distance"),
        )
        cmd.linear.x = float(linear)
        cmd.angular.z = float(angular)
        self.pub.publish(cmd)
        self.get_logger().info(
            f"dist={distance:.2f}m front={clearance:.2f}m "
            f"ref_v={linear:.2f} ref_w={angular:.2f}",
            throttle_duration_sec=3.0,
        )


def main(args=None):
    rclpy.init(args=args)
    node = ReactiveGoalNavigator()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.pub.publish(Twist())
        node.destroy_node()
        rclpy.shutdown()


if __name__ == "__main__":
    main()
