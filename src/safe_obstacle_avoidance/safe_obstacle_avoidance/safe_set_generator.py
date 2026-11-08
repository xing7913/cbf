"""Generate 2-D ellipsoidal unsafe sets from a ROS 2 depth image.

This is a ROS 2 implementation of the ellipse-generation stage described by
chetanreddyn/safe-obstacle-avoidance. Ellipses are published in the local
camera/body plane together with the odometry pose at capture time.
"""

from __future__ import annotations

import math

import cv2
import numpy as np
import rclpy
from cv_bridge import CvBridge, CvBridgeError
from nav_msgs.msg import Odometry
from rclpy.node import Node
from rclpy.qos import qos_profile_sensor_data
from sensor_msgs.msg import Image

from safe_obstacle_avoidance_msgs.msg import SafeSetEllipsoidsConfig

from .geometry import depth_image_to_ellipses


class SafeSetGenerator(Node):
    def __init__(self) -> None:
        super().__init__("safe_set_generator")
        self.declare_parameter("depth_topic", "/camera/depth/image_raw")
        self.declare_parameter("odom_topic", "/odom")
        self.declare_parameter("safe_set_topic", "/safe_set_config")
        self.declare_parameter("theta_min_deg", 30.0)
        self.declare_parameter("theta_max_deg", 150.0)
        self.declare_parameter("min_depth", 0.20)
        self.declare_parameter("max_depth", 5.0)
        self.declare_parameter("r_max_ellipse", 5.0)
        self.declare_parameter("depth_val_outside_fov", 0.8)
        self.declare_parameter("ellipse_buffer", 0.20)
        self.declare_parameter("ellipse_semi_minor", 0.15)
        self.declare_parameter("num_ellipses", 200)
        self.declare_parameter("depth_row_fraction", 0.60)
        self.declare_parameter("camera_offset_x", 0.13)
        self.declare_parameter("camera_offset_y", 0.0)
        self.declare_parameter("yaw_offset", 0.0)

        self.bridge = CvBridge()
        self.latest_odom: Odometry | None = None

        depth_topic = str(self.get_parameter("depth_topic").value)
        odom_topic = str(self.get_parameter("odom_topic").value)
        safe_set_topic = str(self.get_parameter("safe_set_topic").value)
        self.publisher = self.create_publisher(
            SafeSetEllipsoidsConfig, safe_set_topic, 10
        )
        self.odom_sub = self.create_subscription(
            Odometry, odom_topic, self._odom_callback, qos_profile_sensor_data
        )
        self.depth_sub = self.create_subscription(
            Image, depth_topic, self._depth_callback, qos_profile_sensor_data
        )
        self.get_logger().info(
            f"Listening for depth on {depth_topic}; publishing ellipses on {safe_set_topic}"
        )

    def _odom_callback(self, msg: Odometry) -> None:
        self.latest_odom = msg

    def _parameter_float(self, name: str) -> float:
        return float(self.get_parameter(name).value)

    def _depth_to_meters(self, image: Image) -> np.ndarray:
        frame = self.bridge.imgmsg_to_cv2(image, desired_encoding="passthrough")
        depth = np.asarray(frame, dtype=np.float32)
        if depth.ndim != 2:
            raise ValueError(f"Expected a single-channel depth image, got {depth.shape}")

        # REP-style depth images are commonly 32FC1 in metres or 16UC1 in mm.
        if image.encoding.upper() in ("16UC1", "MONO16"):
            depth *= 0.001
        elif image.encoding.upper() not in ("32FC1", "64FC1", "FLOAT32", "PASSTHROUGH"):
            self.get_logger().warning(
                f"Unusual depth encoding '{image.encoding}'; assuming values are metres",
                throttle_duration_sec=5.0,
            )

        min_depth = self._parameter_float("min_depth")
        max_depth = self._parameter_float("max_depth")
        # Invalid/zero depth is treated conservatively as a nearby return.
        depth = np.nan_to_num(depth, nan=min_depth, posinf=max_depth, neginf=min_depth)
        depth[depth <= 0.0] = min_depth
        return np.clip(depth, min_depth, max_depth)

    def _depth_callback(self, msg: Image) -> None:
        if self.latest_odom is None:
            self.get_logger().warning(
                "Waiting for odometry before publishing a local safe set",
                throttle_duration_sec=5.0,
            )
            return

        try:
            depth_img = self._depth_to_meters(msg)
            ellipses = self._make_ellipses(depth_img)
            if ellipses.size == 0:
                return

            output = SafeSetEllipsoidsConfig()
            output.header = self.latest_odom.header
            output.header.stamp = msg.header.stamp
            output.origin_pose = self._camera_origin_pose(self.latest_odom.pose.pose)
            output.origin_pose.orientation = self._offset_orientation(
                output.origin_pose.orientation
            )
            output.data = ellipses.reshape(-1).astype(float).tolist()
            output.columns = 5
            self.publisher.publish(output)
        except (CvBridgeError, ValueError, FloatingPointError) as exc:
            self.get_logger().error(f"Depth processing failed: {exc}")

    def _camera_origin_pose(self, robot_pose):
        """Translate the odometry base pose to the depth camera optical origin."""
        from geometry_msgs.msg import Pose

        pose = Pose()
        q = robot_pose.orientation
        yaw = math.atan2(
            2.0 * (q.w * q.z + q.x * q.y),
            1.0 - 2.0 * (q.y * q.y + q.z * q.z),
        )
        offset_x = self._parameter_float("camera_offset_x")
        offset_y = self._parameter_float("camera_offset_y")
        pose.position.x = robot_pose.position.x + offset_x * math.cos(yaw) - offset_y * math.sin(yaw)
        pose.position.y = robot_pose.position.y + offset_x * math.sin(yaw) + offset_y * math.cos(yaw)
        pose.position.z = robot_pose.position.z
        pose.orientation = robot_pose.orientation
        return pose

    def _offset_orientation(self, q):
        """Apply the legacy robot-frame offset, configurable for other bases."""
        offset = self._parameter_float("yaw_offset")
        half = offset / 2.0
        s, c = math.sin(half), math.cos(half)
        # Quaternion multiplication q_out = q_odom * q_z(offset).
        return type(q)(
            x=q.x * c + q.y * s,
            y=-q.x * s + q.y * c,
            z=q.z * c + q.w * s,
            w=q.w * c - q.z * s,
        )

    def _make_ellipses(self, depth_img: np.ndarray) -> np.ndarray:
        return depth_image_to_ellipses(
            depth_img,
            theta_min_deg=float(self.get_parameter("theta_min_deg").value),
            theta_max_deg=float(self.get_parameter("theta_max_deg").value),
            r_max_ellipse=float(self.get_parameter("r_max_ellipse").value),
            depth_val_outside_fov=float(self.get_parameter("depth_val_outside_fov").value),
            ellipse_buffer=float(self.get_parameter("ellipse_buffer").value),
            ellipse_semi_minor=float(self.get_parameter("ellipse_semi_minor").value),
            num_ellipses=int(self.get_parameter("num_ellipses").value),
            depth_row_fraction=float(self.get_parameter("depth_row_fraction").value),
            min_depth=float(self.get_parameter("min_depth").value),
        )


def main(args=None) -> None:
    rclpy.init(args=args)
    node = SafeSetGenerator()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == "__main__":
    main()
