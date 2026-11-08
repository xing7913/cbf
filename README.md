# Safe Obstacle Avoidance — ROS 2 + Gazebo Sim

面向 Ubuntu 26.04 + ROS 2 Lyrical + Gazebo Jetty 的原生 ROS 2 复现工程，基于
[chetanreddyn/safe-obstacle-avoidance](https://github.com/chetanreddyn/safe-obstacle-avoidance)
的“深度图 → 椭圆安全集 → CBF-QP → 速度过滤”算法链路整理。

> **Gazebo 前提：** 假设 Gazebo Jetty 已经安装并可启动。脚本不会安装、卸载、升级或替换 Gazebo；只检查运行时，并安装/检查 ROS 侧桥接依赖。
>
> **边界说明：** 本仓库提供 ROS 2 原生节点、rosidl 消息、Gazebo Sim SDF 场景、ros_gz_bridge 配置、launch、实验记录工具与测试。制作环境没有目标机的 ROS 2/Gazebo 运行时，因此实际 `colcon build` 和端到端仿真仍须在 Ubuntu 26.04 虚拟机验收；不会把静态检查说成仿真已成功。

## 内容

- `safe_obstacle_avoidance_msgs`: ROS 2 `rosidl` 自定义安全集消息。
- `safe_obstacle_avoidance`: `rclpy` 深度图安全集生成、椭圆几何、CBF 控制器与二维 QP。
- `simulation/worlds/ellipsoid_obstacle_world.sdf`: 自带差速机器人、前向深度相机和静态障碍物。
- `ros_gz_bridge` 配置与 ROS 2 launch。
- `scripts/`: 环境检查、ROS 依赖安装、构建、启动、停止、实验记录/分析。
- `tests/`: 几何、QP 与项目资源检查。

## 环境

| 项目 | 版本/要求 |
|---|---|
| OS | Ubuntu 26.04 LTS |
| ROS 2 | Lyrical |
| Gazebo | Jetty，需预先安装 |
| Python | ROS 2 系统 Python |

## 快速开始

```bash
git clone https://github.com/xing7913/cbf.git
cd cbf
source /opt/ros/lyrical/setup.bash

# 只检查 Gazebo，不安装 Gazebo
./scripts/check_env.sh

# 安装 ROS 侧依赖（不会安装 ros_gz_sim/Gazebo 本体）
./scripts/install_ros_deps.sh

# 编译并启动自带场景
./scripts/build.sh
source install/setup.bash
./scripts/run_demo.sh
```

若需一条命令构建并启动：

```bash
./scripts/run_demo.sh
```

## ROS 话题

```text
/camera/depth/image_raw  sensor_msgs/msg/Image
/odom                   nav_msgs/msg/Odometry
/ref_vel                geometry_msgs/msg/Twist
/safe_set_config        safe_obstacle_avoidance_msgs/msg/SafeSetEllipsoidsConfig
/cmd_vel                geometry_msgs/msg/Twist
/clock                  rosgraph_msgs/msg/Clock
```

遥控器/导航只向 `/ref_vel` 发布参考速度；CBF 控制器是最终 `/cmd_vel` 的唯一发布者。接入自己的机器人时，修改 `config/params.yaml` 与 `config/ros_gz_bridge.yaml`，先低速核验坐标系、速度符号和相机安装偏移。

## 验收顺序

1. `./scripts/check_env.sh`：Ubuntu、ROS 2、Gazebo 命令和桥接包检查。
2. `colcon build --symlink-install`：消息包和 Python 包通过构建。
3. `ros2 topic hz /camera/depth/image_raw`：深度图持续发布。
4. `ros2 topic echo /safe_set_config --once`：安全集数据正常。
5. `ros2 topic echo /cmd_vel`：CBF 输出速度合理，且不会由 teleop 直接旁路发布。
6. 运行实验记录脚本并保存 CSV、轨迹图与指标。

## 实验工具

```bash
./scripts/run_experiment.sh 30 results/run_01.csv
./scripts/run_repeats.sh 3 30
python3 scripts/analyze_run.py results/run_01.csv
```

最小间距为基于 SDF 固定障碍物和圆形底盘近似的二维指标，不是 Gazebo 碰撞真值。真实结果必须以目标机实际记录为准。

## 许可证与学术引用

原始项目请见 [chetanreddyn/safe-obstacle-avoidance](https://github.com/chetanreddyn/safe-obstacle-avoidance)。
请先核对原仓库许可证与第三方资源许可证，再决定分发/商用方式。学术使用应引用原作者。
