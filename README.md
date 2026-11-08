# Safe Obstacle Avoidance — ROS 2 + Gazebo Sim

**Ubuntu 26.04 · ROS 2 Lyrical · Gazebo Jetty（已预装）· Python 3**

这是 `chetanreddyn/safe-obstacle-avoidance` 的原生 ROS 2 复现工程，目标是把项目中的**深度图 → 椭圆安全集 → CBF-QP → 速度过滤**闭环在 ROS 2 + Gazebo Sim 中跑通。

仓库包含 ROS 2 消息包、`rclpy` 节点、CBF-QP 求解器、Gazebo Sim SDF 场景、`ros_gz_bridge` 配置、一键启动脚本、实验记录与分析工具，以及 Python 几何/QP 和资源静态测试。**不包含 ROS 1 节点，不使用 `rospy`、`catkin`、`rosrun` 或 `roslaunch`。**

> **Gazebo 安装约定：** 默认你的 Ubuntu 虚拟机已经安装并能启动 Gazebo Jetty。安装脚本只安装 ROS 侧的 `ros_gz_bridge` 等依赖，不安装、不卸载、不升级 Gazebo，也不会安装 `ros_gz_sim`。
>
> **仿真模型说明：** 仓库自带一个轻量差速机器人 `safe_bot`（不是 TurtleBot3 原厂模型），带前向深度相机和三个静态障碍物，目的是先复现算法闭环。你自己的 TurtleBot3/Gazebo 模型可通过参数和 bridge YAML 接入，见下文“接入自己的机器人”。

## 当前实现内容

| 模块 | 内容 | 状态 |
|---|---|---|
| `safe_obstacle_avoidance_msgs` | `SafeSetEllipsoidsConfig` ROS 2 自定义消息 | 已实现 |
| `safe_set_generator` | 深度图预处理、径向距离计算、椭圆安全集生成 | 已实现 |
| `cbf_controller` | 里程计/参考速度订阅，椭圆 CBF 约束，二维 QP 过滤 | 已实现 |
| `qp.py` | 无需 CVXOPT/CasADi 的二维严格凸 QP 活动集求解器 | 已实现并有单元测试 |
| Gazebo Sim world | 差速底盘、深度相机、地面和静态障碍物 | 已提供 SDF，待目标机验证 |
| ROS-Gazebo bridge | `/clock`、深度图、里程计、速度命令映射 | 已提供配置 |
| Launch | 一次启动 Gazebo、bridge、算法节点 | 已提供 |
| 实验工具 | CSV 记录、轨迹图和基本指标 | 已提供，待实测 |

**验证边界：** 纯 Python 单元测试与静态检查可以在非 ROS 环境运行。本次制作环境没有安装你的 Ubuntu 26.04 + ROS 2 Lyrical + Gazebo 运行时，所以尚不能声称已通过目标机的 `colcon build` 或端到端仿真。README 会把“已实现”和“待目标机验证”分开记录，不虚构实验结果。

---

## 1. 环境要求

| 项目 | 目标环境 |
|---|---|
| OS | Ubuntu 26.04 LTS |
| ROS 2 | Lyrical Luth |
| Gazebo | Jetty，**需预先安装** |
| Python | ROS 2 系统 Python |
| 图像输入 | `sensor_msgs/msg/Image`，`32FC1`（米）或 `16UC1`（毫米） |
| 里程计 | `nav_msgs/msg/Odometry` |
| 参考速度 | `geometry_msgs/msg/Twist` |

每个新终端先加载 ROS 2：

```bash
source /opt/ros/lyrical/setup.bash
```

下载工程：

```bash
git clone https://github.com/xing7913/cbf.git
cd cbf
```

## 2. 安装依赖（不会安装 Gazebo）

```bash
./scripts/check_env.sh
./scripts/install_ros_deps.sh
```

安装脚本需要的 ROS 包包括 `ros_gz_bridge`、`cv_bridge`、键盘遥控、NumPy、OpenCV、colcon 和 Matplotlib。它**不会**安装 Gazebo 本体，也不会安装 `ros_gz_sim`。

## 3. 编译

```bash
./scripts/build.sh
source install/setup.bash
```

手动方式：

```bash
rosdep install --from-paths src --ignore-src --rosdistro lyrical -r -y
colcon build --symlink-install
source install/setup.bash
```

工作空间使用 `ament_cmake + rosidl` 构建消息包，使用 `ament_python + rclpy` 构建算法包。

## 4. 一键运行自带仿真

```bash
./scripts/run_demo.sh
```

这条命令在 `install/` 不存在时会先构建，然后通过 ROS 2 launch 启动：

1. `simulation/worlds/ellipsoid_obstacle_world.sdf`：Gazebo Sim 场景；
2. `ros_gz_bridge`：深度图、里程计、速度命令和仿真时钟桥接；
3. `safe_set_generator`：把深度图转成椭圆安全集；
4. `cbf_controller`：用 CBF-QP 过滤参考速度；
5. `reference_cmd_publisher`：默认只在自带仿真中发布一段有时长/里程上限的前进参考速度，让你不必另开遥控器就能观察 CBF 过滤效果。

常用启动参数：

```bash
./scripts/run_demo.sh gui:=false                         # Gazebo server-only
./scripts/run_demo.sh auto_demo:=false                   # 禁用自动参考速度，改用键盘/导航栈
./scripts/run_demo.sh start_simulation:=false auto_demo:=false # 连接你已运行的 Gazebo
```

默认启动会自动发布一段限时/限里程的 `/ref_vel`，以便直接观察 CBF-QP 过滤效果。需要键盘控制时，关闭自动参考命令：

```bash
./scripts/run_demo.sh auto_demo:=false
# 另开终端
source /opt/ros/lyrical/setup.bash
source install/setup.bash
ros2 run teleop_twist_keyboard teleop_twist_keyboard --ros-args -r cmd_vel:=/ref_vel
```

**不要让遥控器直接发布 `/cmd_vel`。** 遥控器只发布 `/ref_vel`，`cbf_controller` 是 `/cmd_vel` 的唯一控制源。自动参考命令仅在本仓库自带 Gazebo 场景启动时才会运行；连接自己的机器人时使用 `start_simulation:=false auto_demo:=false`。

结束仿真：在 launch 终端按 `Ctrl+C`。如果 Gazebo 服务端残留：

```bash
./scripts/stop.sh
```

## 5. ROS 话题和数据流

```text
Gazebo depth camera
  /camera/depth_image (gz.msgs.Image)
         │ ros_gz_bridge
         ▼
  /camera/depth/image_raw (sensor_msgs/msg/Image)
         │
         ▼
  safe_set_generator ──> /safe_set_config (SafeSetEllipsoidsConfig)
                                  │
Gazebo odometry ──> /odom ────────┤
                                  ▼
teleop /ref_vel ───────────> cbf_controller ──> /cmd_vel ──> Gazebo DiffDrive
```

| ROS 2 话题 | 类型 | 用途 |
|---|---|---|
| `/camera/depth/image_raw` | `sensor_msgs/msg/Image` | 深度图输入 |
| `/odom` | `nav_msgs/msg/Odometry` | 机器人位姿 |
| `/safe_set_config` | `safe_obstacle_avoidance_msgs/msg/SafeSetEllipsoidsConfig` | 椭圆中心、半轴、角度与生成时位姿 |
| `/ref_vel` | `geometry_msgs/msg/Twist` | 未过滤的参考速度 |
| `/cmd_vel` | `geometry_msgs/msg/Twist` | CBF 过滤后的最终速度 |
| `/clock` | `rosgraph_msgs/msg/Clock` | Gazebo 仿真时间 |

检查话题：

```bash
./scripts/check_topics.sh
ros2 topic hz /camera/depth/image_raw
ros2 topic echo /safe_set_config --once
ros2 topic echo /cmd_vel
```

Gazebo 原生话题可用下面命令查看：

```bash
gz topic -l | grep -E 'camera|depth|odometry|cmd_vel'
```

## 6. 接入自己的 TurtleBot3 / Gazebo 模型

自带 world 用于先确认算法闭环。接入你自己的模型时，算法节点仍然是原生 ROS 2，只需要适配**话题名、相机安装位姿和速度方向约定**：

1. 在 Gazebo 中启动你的机器人和深度相机；
2. 编辑 `src/safe_obstacle_avoidance/config/ros_gz_bridge.yaml`，把 `gz_topic_name` 改成你实际的 Gazebo Transport 话题；
3. 编辑 `src/safe_obstacle_avoidance/config/params.yaml`，设置 ROS 侧 `depth_topic`、`odom_topic`、`reference_topic`、`cmd_vel_topic`；
4. 根据机器人底盘与相机坐标系设置 `camera_offset_x/y`、`yaw_offset`、`linear_velocity_sign` 和 `angular_velocity_sign`；
5. 使用 `start_simulation:=false`，让 launch 只启动 bridge 和算法节点。

先用架空/低速方式确认正向指令方向。若你的底盘遵循 REP-103（`base_link` 的 +X 向前、+Z 向上），从 `yaw_offset=0`、速度符号 `+1` 开始。不要在未经检查时直接套用其他机器人模型的符号。

## 7. 参数

主要参数位于 `src/safe_obstacle_avoidance/config/params.yaml`。

| 参数 | 默认值 | 含义 |
|---|---:|---|
| `theta_min_deg` / `theta_max_deg` | 30° / 150° | 深度相机水平视场在算法平面中的角度范围；中心前向为 90° |
| `min_depth` / `max_depth` | 0.20 / 5.0 m | 有效深度范围 |
| `r_max_ellipse` | 5.0 m | 椭圆安全集最大径向范围 |
| `depth_val_outside_fov` | 0.8 m | 摄像头视场外的保守假定距离 |
| `ellipse_buffer` | 0.20 m | 椭圆长半轴缓冲 |
| `ellipse_semi_minor` | 0.15 m | 椭圆短半轴 |
| `num_ellipses` | 200 | 全角度椭圆数量的近似目标 |
| `depth_row_fraction` | 0.50 | 取图像中心行；适配自带水平安装的仿真相机 |
| `camera_offset_x/y` | 0.13 / 0.0 m | 相机相对底盘中心的平面安装偏移 |
| `gamma` | 0.1 | CBF 收敛/安全约束系数 |
| `q_linear` / `q_angular` | 100 / 1 | QP 中线速度/角速度修正权重 |
| `linear_min/max` | -0.26 / 0.26 m/s | 最终线速度边界 |
| `angular_min/max` | -1.82 / 1.82 rad/s | 最终角速度边界 |
| `control_rate` | 20 Hz | CBF 控制循环频率 |

## 8. 实验记录与复现

建议先用一条命令运行一次自动实验（默认无 GUI、30 秒）：

```bash
./scripts/run_experiment.sh 30 results/run_01.csv
./scripts/run_repeats.sh 3 30
```

也可手动运行 Gazebo/节点，再单独记录 60 秒：

```bash
source /opt/ros/lyrical/setup.bash
source install/setup.bash
python3 scripts/record_run.py --duration 60 --output results/run_01.csv
```

记录完成后分析：

```bash
python3 scripts/analyze_run.py results/run_01.csv
```

会生成：

- `results/run_01.metrics.json`：时长、轨迹长度、平均速度、速度方差、参考/输出速度差；
- `results/run_01.trajectory.png`：机器人轨迹和场景静态障碍物示意。

最小间距目前是依据本仓库 SDF 中的固定障碍物位置和圆形近似底盘计算的**近似二维指标**，不是 Gazebo 碰撞真值，也不能替代碰撞检测。不要将尚未实测的数值写成论文复现结果。记录模板见 [`docs/reproduction.md`](docs/reproduction.md)、[`docs/architecture.md`](docs/architecture.md) 和 [`docs/roadmap.md`](docs/roadmap.md)。

## 9. 目录结构

```text
cbf/
├── src/
│   ├── safe_obstacle_avoidance_msgs/   # rosidl 自定义消息
│   └── safe_obstacle_avoidance/        # rclpy 算法包
│       ├── safe_obstacle_avoidance/
│       │   ├── safe_set_generator.py
│       │   ├── cbf_controller.py
│       │   ├── reference_cmd_publisher.py
│       │   ├── geometry.py
│       │   └── qp.py
│       ├── launch/
│       └── config/
├── simulation/worlds/                  # 自带 Gazebo Sim world
├── scripts/                            # 安装、构建、启动、实验重复、记录、分析
├── docs/                               # 架构、算法、路线图与复现实验说明
├── tests/                              # Python + SDF/XML/YAML 静态测试
└── results/                            # 用户实测结果输出目录
```

## 10. 安全与已知限制

- 如果深度图、里程计、参考速度或安全集超过设定超时时间，控制器输出零速度；QP 无可行解或当前状态落入椭圆禁区时也输出零速度（fail-closed）。
- 自定义消息记录生成安全集时的相机参考位姿；位姿取深度图回调时最近一次收到的 odometry，当前还没有做严格时间同步/TF 插值。
- `depth_image_to_ellipses` 对深度做轴向距离到径向距离转换；源算法中已计算但未用于后续的径向距离在这里被正确使用。
- QP 求解器只针对二维控制量和正定对角权重，不是通用 QP 求解器。它枚举原点、单约束投影和双约束交点；若数值条件不好或无可行点则停车。
- 椭圆生成与 CBF 依赖相机 FOV、相机安装位姿、底盘运动学和坐标系约定。默认参数只对应本仓库自带的 `safe_bot` 场景，需为 TurtleBot3 或真实平台重新标定。
- 自带场景目前是静态障碍物示例；动态障碍物运动、原项目其他安全集路径、与论文的严格数值对比仍属于后续工作。
- 本仓库不是安全认证软件，不能直接用于真实机器人高速运行。

## 参考

- 原始算法仓库：<https://github.com/chetanreddyn/safe-obstacle-avoidance>
- 本地完整 ROS 2 仿真项目风格参考：<https://github.com/huimo-wuling/mpc-dcbf-gzsim>
- Gazebo Jetty 深度相机：<https://gazebosim.org/docs/jetty/sensors/>
- Gazebo Sim 差速驱动：<https://gazebosim.org/docs/latest/moving_robot/>
- `ros_gz_bridge`：<https://github.com/gazebosim/ros_gz/tree/ros2/ros_gz_bridge>

**上游许可说明：** 当前可见的原始算法仓库没有提供明确的许可证文件。本工程不擅自给上游算法授予新的开源许可；请仅用于学习/研究，公开再分发或商业使用前先向原作者确认许可。
