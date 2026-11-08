# Implementation notes

## Native ROS 2 package layout

```text
src/
├── safe_obstacle_avoidance_msgs/        # ament_cmake + rosidl
│   └── msg/SafeSetEllipsoidsConfig.msg
└── safe_obstacle_avoidance/             # ament_python + rclpy
    ├── safe_obstacle_avoidance/
    │   ├── safe_set_generator.py
    │   ├── cbf_controller.py
    │   └── qp.py
    ├── launch/safe_obstacle_avoidance.launch.py
    └── config/params.yaml
```

## Source-to-ROS2 mapping

| Original ROS 1 element | This ROS 2 workspace |
|---|---|
| `rospy` | `rclpy` nodes |
| `rospy.Subscriber` | `create_subscription` |
| `rospy.Publisher` | `create_publisher` |
| `rospy.Rate` polling loops | ROS 2 timers and image callbacks |
| custom catkin message | `rosidl` message package |
| `catkin_make` | `colcon build` |
| `rosrun` / `roslaunch` | `ros2 run` / `ros2 launch` |
| `cv_bridge` image conversion | ROS 2 `cv_bridge` |

## Geometry and control equations

For each measured angle `theta` and radial distance `r`, the generator creates an ellipse:

- center radius: `(r_max + r) / 2`
- semi-major axis: `(r_max - r) / 2 + d_s`
- semi-minor axis: `b_i`
- orientation: `theta`

The ellipse CBF is `h_i(p) = (p - c_i)^T A_i (p - c_i) - 1`, with `A_i = R_i diag(1/a_i², 1/b_i²) R_i^T`. The safe side is `h_i >= 0`.

For the unicycle control vector `u = [v, omega]`, the controller linearizes `h_dot + gamma*h >= 0` and solves for a correction `delta_u` around the reference `u_ref`:

`min 0.5 * delta_u.T * Q * delta_u` subject to all CBF inequalities and symmetric velocity limits. If no feasible correction is found, it publishes zero velocity.

## Deliberate fixes versus source quirks

- The original depth node computed a radial-distance array but passed the axial depth array into the ellipse update function. This implementation uses radial distance `z / |sin(theta)|` consistently.
- The source CMake file did not declare the ellipsoid message it imported. This package defines that message explicitly and adds the capture-time origin pose.
- The controller uses bounded positive and negative velocity limits rather than only upper limits.
- A 20 Hz control timer replaces the source's tight 1000 Hz loop; safety and timeout behavior are explicit.
- OpenCV preview windows are not opened by default, which avoids GUI dependency issues in virtual machines.

These are intentional engineering corrections; if exact numerical replication of the source results is required, compare intermediate safe-set arrays and control trajectories against the original and tune the coordinate-frame parameters.
