# 软件架构与接口

## 包职责

- `safe_obstacle_avoidance_msgs`：ROS 2 `rosidl` 消息包。
- `safe_obstacle_avoidance`：Python `ament_python` 算法包，全部 ROS 节点使用 `rclpy`。
- `simulation/worlds/ellipsoid_obstacle_world.sdf`：Gazebo Sim world，包含 `safe_bot`、深度相机、差速驱动和静态障碍物。
- `ros_gz_bridge.yaml`：Gazebo Transport 与 ROS 2 话题映射。

## 消息格式

`SafeSetEllipsoidsConfig.msg`：

```text
std_msgs/Header header
geometry_msgs/Pose origin_pose
float64[] data
uint32 columns
```

`columns` 固定为 5；`data` 按行展平，每行依次为椭圆中心 `cx, cy`、长半轴 `a`、短半轴 `b`、椭圆旋转角 `theta`。`origin_pose` 记录生成该批椭圆时的相机/机器人参考位姿，避免控制器把最新安全集误配到完全不同的里程计原点。

## 话题链

```text
Gazebo depth_camera
  /camera/depth_image [gz.msgs.Image]
       │ ros_gz_bridge
       ▼
  /camera/depth/image_raw [sensor_msgs/msg/Image]
       │ safe_set_generator
       ▼
  /safe_set_config [safe_obstacle_avoidance_msgs/msg/SafeSetEllipsoidsConfig]
       │
/odom ─┴─> cbf_controller <── /ref_vel [geometry_msgs/msg/Twist]（键盘输入或自动 demo）
                 │
                 ▼
          /cmd_vel [geometry_msgs/msg/Twist]
                 │ ros_gz_bridge
                 ▼
  /model/safe_bot/cmd_vel [gz.msgs.Twist]
```

## 坐标约定

生成器沿用上游椭圆算法的局部平面约定：前向射线为局部 `+Y`，角度中心为 90°。Gazebo 自带的 `safe_bot` 遵循 REP-103，车体前向为 `+X`。控制器将里程计位姿变换到本批安全集的局部坐标系；因此不能直接把椭圆中心坐标当作世界坐标使用。

此首版只处理水平面深度图，假设相机安装无明显俯仰/横滚；若相机安装有俯仰角，需要把传感器视线投影到地面/规划平面，而不只是改 yaw 参数。
