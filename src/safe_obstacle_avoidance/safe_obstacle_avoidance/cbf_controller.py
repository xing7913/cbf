"""ROS 2 CBF-QP velocity filter for depth-derived ellipsoidal obstacles."""

from __future__ import annotations

import math
from typing import Optional

import numpy as np
import rclpy
from geometry_msgs.msg import Twist
from nav_msgs.msg import Odometry
from rclpy.node import Node
from rclpy.qos import qos_profile_sensor_data

from safe_obstacle_avoidance_msgs.msg import SafeSetEllipsoidsConfig

from .qp import solve_2d_qp


def quaternion_to_yaw(q) -> float:
    """Return yaw from a ROS quaternion."""
    siny_cosp = 2.0 * (q.w * q.z + q.x * q.y)
    cosy_cosp = 1.0 - 2.0 * (q.y * q.y + q.z * q.z)
    return math.atan2(siny_cosp, cosy_cosp)


def wrap_angle(angle: float) -> float:
    return math.atan2(math.sin(angle), math.cos(angle))


class CBFController(Node):
    def __init__(self) -> None:
        super().__init__("cbf_controller")
        defaults = {
            "odom_topic": "/odom",
            "safe_set_topic": "/safe_set_config",
            "reference_topic": "/ref_vel",
            "cmd_vel_topic": "/cmd_vel",
            "control_rate": 20.0,
            "safe_set_timeout": 0.5,
            "odom_timeout": 0.5,
            "reference_timeout": 0.5,
            "gamma": 0.1,
            "q_linear": 100.0,
            "q_angular": 1.0,
            "linear_min": -0.26,
            "linear_max": 0.26,
            "angular_min": -1.82,
            "angular_max": 1.82,
            "robot_offset_l": -0.08,
            # Defaults match the bundled REP-103 base: +X forward, +Z yaw.
            "yaw_offset": 0.0,
            "linear_velocity_sign": 1.0,
            "angular_velocity_sign": 1.0,
            "penetration_tolerance": 1e-3,
        }
        for name, value in defaults.items():
            self.declare_parameter(name, value)

        self.latest_odom: Optional[Odometry] = None
        self.latest_reference: Optional[Twist] = None
        self.latest_safe_set: Optional[SafeSetEllipsoidsConfig] = None
        self.odom_time_ns: Optional[int] = None
        self.reference_time_ns: Optional[int] = None
        self.safe_set_time_ns: Optional[int] = None

        self.cmd_pub = self.create_publisher(
            Twist, str(self.get_parameter("cmd_vel_topic").value), 10
        )
        self.create_subscription(
            Odometry,
            str(self.get_parameter("odom_topic").value),
            self._odom_callback,
            qos_profile_sensor_data,
        )
        self.create_subscription(
            SafeSetEllipsoidsConfig,
            str(self.get_parameter("safe_set_topic").value),
            self._safe_set_callback,
            10,
        )
        self.create_subscription(
            Twist,
            str(self.get_parameter("reference_topic").value),
            self._reference_callback,
            10,
        )

        control_rate = float(self.get_parameter("control_rate").value)
        if control_rate <= 0.0:
            raise ValueError("control_rate must be positive")
        self.timer = self.create_timer(1.0 / control_rate, self._control_step)
        self.get_logger().info(
            "CBF controller ready: reference -> CBF-QP -> filtered cmd_vel"
        )

    def _now_ns(self) -> int:
        return self.get_clock().now().nanoseconds

    def _odom_callback(self, msg: Odometry) -> None:
        self.latest_odom = msg
        self.odom_time_ns = self._now_ns()

    def _reference_callback(self, msg: Twist) -> None:
        self.latest_reference = msg
        self.reference_time_ns = self._now_ns()

    def _safe_set_callback(self, msg: SafeSetEllipsoidsConfig) -> None:
        if msg.columns != 5 or len(msg.data) == 0 or len(msg.data) % 5 != 0:
            self.get_logger().warning("Ignoring malformed ellipsoid safe-set message")
            return
        data = np.asarray(msg.data, dtype=float).reshape(-1, 5)
        if not np.all(np.isfinite(data)) or np.any(data[:, 2:4] <= 0.0):
            self.get_logger().warning("Ignoring non-finite or degenerate ellipsoid data")
            return
        self.latest_safe_set = msg
        self.safe_set_time_ns = self._now_ns()

    def _is_fresh(self, stamp_ns: Optional[int], timeout_param: str) -> bool:
        if stamp_ns is None:
            return False
        timeout = float(self.get_parameter(timeout_param).value)
        age = (self._now_ns() - stamp_ns) / 1e9
        return 0.0 <= age <= timeout

    def _publish_stop(self) -> None:
        self.cmd_pub.publish(Twist())

    def _current_local_state(self) -> tuple[np.ndarray, np.ndarray]:
        """Robot [x,y,yaw] in the coordinate frame captured with the safe set."""
        assert self.latest_odom is not None
        assert self.latest_safe_set is not None
        current_pose = self.latest_odom.pose.pose
        origin_pose = self.latest_safe_set.origin_pose

        yaw_offset = float(self.get_parameter("yaw_offset").value)
        yaw_global = wrap_angle(quaternion_to_yaw(current_pose.orientation) + yaw_offset)
        yaw_origin = wrap_angle(quaternion_to_yaw(origin_pose.orientation))
        axis_rotation = yaw_origin - math.pi / 2.0

        dx = current_pose.position.x - origin_pose.position.x
        dy = current_pose.position.y - origin_pose.position.y
        c, s = math.cos(axis_rotation), math.sin(axis_rotation)
        rotation = np.array([[c, s], [-s, c]], dtype=float)
        xy_local = rotation @ np.array([dx, dy], dtype=float)
        yaw_local = wrap_angle(yaw_global - axis_rotation)
        return np.array([xy_local[0], xy_local[1], yaw_local]), np.array([dx, dy])

    def _safe_set_constraints(self, state: np.ndarray, u_ref: np.ndarray) -> tuple[np.ndarray, np.ndarray, float]:
        """Build linearized CBF constraints G delta_u <= h."""
        assert self.latest_safe_set is not None
        data = np.asarray(self.latest_safe_set.data, dtype=float).reshape(-1, 5)
        gamma = float(self.get_parameter("gamma").value)
        offset_l = float(self.get_parameter("robot_offset_l").value)
        x, y, theta = state

        # Unicycle kinematics: [x_dot, y_dot, theta_dot] = B @ [v, omega].
        B = np.array(
            [[math.cos(theta), 0.0], [math.sin(theta), 0.0], [0.0, 1.0]],
            dtype=float,
        )
        B1 = np.array(
            [[1.0, 0.0, offset_l * math.sin(theta)],
             [0.0, 1.0, -offset_l * math.cos(theta)]],
            dtype=float,
        )

        cbf_rows: list[np.ndarray] = []
        cbf_bounds: list[float] = []
        min_h = float("inf")
        for cx, cy, semi_major, semi_minor, ellipse_yaw in data:
            c, s = math.cos(ellipse_yaw), math.sin(ellipse_yaw)
            rot = np.array([[c, -s], [s, c]], dtype=float)
            p = np.diag([1.0 / (semi_major * semi_major), 1.0 / (semi_minor * semi_minor)])
            A = rot @ p @ rot.T

            # Keep the same offset-point model used in the source formulation.
            d = np.array(
                [x - offset_l * math.cos(theta) - cx,
                 y - offset_l * math.sin(theta) - cy],
                dtype=float,
            )
            h_value = float(d.T @ A @ d - 1.0)
            min_h = min(min_h, h_value)
            grad_state = (d.T @ (A + A.T)) @ B1
            grad_control = np.asarray(grad_state @ B, dtype=float).reshape(2)

            # h_dot + gamma*h >= 0, with u = u_ref + delta_u.
            cbf_rows.append(-grad_control)
            cbf_bounds.append(gamma * h_value + float(grad_control @ u_ref))

        # Bound the final (not correction) velocities symmetrically.
        v_min = float(self.get_parameter("linear_min").value)
        v_max = float(self.get_parameter("linear_max").value)
        w_min = float(self.get_parameter("angular_min").value)
        w_max = float(self.get_parameter("angular_max").value)
        bound_rows = [
            np.array([1.0, 0.0]), np.array([-1.0, 0.0]),
            np.array([0.0, 1.0]), np.array([0.0, -1.0]),
        ]
        bound_values = [
            v_max - u_ref[0], -v_min + u_ref[0],
            w_max - u_ref[1], -w_min + u_ref[1],
        ]
        G = np.vstack(cbf_rows + bound_rows)
        h = np.asarray(cbf_bounds + bound_values, dtype=float)
        return G, h, min_h

    def _control_step(self) -> None:
        required = (
            self.latest_odom is not None
            and self.latest_reference is not None
            and self.latest_safe_set is not None
            and self._is_fresh(self.odom_time_ns, "odom_timeout")
            and self._is_fresh(self.reference_time_ns, "reference_timeout")
            and self._is_fresh(self.safe_set_time_ns, "safe_set_timeout")
        )
        if not required:
            self._publish_stop()
            return

        assert self.latest_reference is not None
        linear_sign = float(self.get_parameter("linear_velocity_sign").value)
        angular_sign = float(self.get_parameter("angular_velocity_sign").value)
        if abs(abs(linear_sign) - 1.0) > 1e-8 or abs(abs(angular_sign) - 1.0) > 1e-8:
            self.get_logger().error("Velocity sign parameters must be +1 or -1")
            self._publish_stop()
            return
        u_ref = np.array(
            [linear_sign * self.latest_reference.linear.x,
             angular_sign * self.latest_reference.angular.z],
            dtype=float,
        )

        try:
            state, _ = self._current_local_state()
            G, h, min_h = self._safe_set_constraints(state, u_ref)
            penetration_tolerance = float(self.get_parameter("penetration_tolerance").value)
            if min_h < -penetration_tolerance:
                self.get_logger().error(
                    f"Robot state is outside a safe-set boundary (min h={min_h:.4f}); stopping",
                    throttle_duration_sec=1.0,
                )
                self._publish_stop()
                return
            q_diag = np.array(
                [float(self.get_parameter("q_linear").value),
                 float(self.get_parameter("q_angular").value)],
                dtype=float,
            )
            correction = solve_2d_qp(q_diag, G, h)
            if correction is None:
                self.get_logger().warning(
                    "CBF-QP is infeasible; publishing zero velocity (fail-closed)",
                    throttle_duration_sec=2.0,
                )
                self._publish_stop()
                return

            command_internal = u_ref + correction
            command = Twist()
            command.linear.x = float(linear_sign * command_internal[0])
            command.angular.z = float(angular_sign * command_internal[1])
            self.cmd_pub.publish(command)
            self.get_logger().debug(f"min h={min_h:.3f}, command={command_internal}")
        except (ValueError, FloatingPointError, np.linalg.LinAlgError) as exc:
            self.get_logger().error(f"CBF calculation failed; stopping robot: {exc}")
            self._publish_stop()


def main(args=None) -> None:
    rclpy.init(args=args)
    node = CBFController()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node._publish_stop()
        node.destroy_node()
        rclpy.shutdown()


if __name__ == "__main__":
    main()
