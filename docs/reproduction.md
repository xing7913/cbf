# 实验复现与结果记录

## 实验目标

验证：深度图持续到达 → 椭圆安全集更新 → 参考速度进入 CBF-QP → 过滤速度送入 Gazebo 差速底盘。

## 启动

默认模式：一条命令启动自带 Gazebo 场景、深度相机、椭圆安全集节点、CBF-QP 控制器和限速/限时参考速度节点。自动演示达到 5 m 路程或 30 s 仿真时间后停止给出前进指令：

```bash
source /opt/ros/lyrical/setup.bash
cd ~/safe-obstacle-avoidance-ros2
./scripts/run_demo.sh
```

手动键盘模式：先关闭自动参考速度，再在另一个终端将键盘输出重映射到 `/ref_vel`（不要直接发布到 `/cmd_vel`，因为它是 CBF 过滤后的执行话题）：

```bash
# 终端 A
./scripts/run_demo.sh auto_demo:=false

# 终端 B
source /opt/ros/lyrical/setup.bash
source ~/safe-obstacle-avoidance-ros2/install/setup.bash
ros2 run teleop_twist_keyboard teleop_twist_keyboard --ros-args -r cmd_vel:=/ref_vel
```

检查链路：

```bash
./scripts/check_topics.sh
ros2 topic hz /camera/depth/image_raw
ros2 topic hz /safe_set_config
ros2 topic echo /cmd_vel
```

## 一条命令记录与分析

```bash
./scripts/run_experiment.sh 30 results/run_01.csv
./scripts/run_repeats.sh 3 30
```

脚本会以无 GUI 模式启动自带场景、启用限时自动参考速度、等待 `/odom`、记录、分析，然后停止本项目启动的 Gazebo/ROS 进程。

## 手动记录实验

```bash
python3 scripts/record_run.py --duration 60 --output results/run_01.csv
python3 scripts/analyze_run.py results/run_01.csv
```

记录节点按 10 Hz 采样 `/odom`、`/ref_vel` 和 `/cmd_vel`。分析脚本输出 JSON 指标和轨迹 PNG。当前最小间距采用 SDF 固定障碍物位置和圆形底盘近似，不是真实碰撞几何的最小距离。

## 重复实验

建议相同虚拟机配置、初始位姿和参数下重复三次，输出分别命名为 `run_01.csv`、`run_02.csv`、`run_03.csv`。每次记录系统/ROS/Gazebo 版本、虚拟机 CPU 和内存、场景、参数 YAML 的 Git commit、最小间距计算口径及是否发生接触。

`results/` 不预填模拟数字。实际运行后留存原始 CSV 和生成指标；发生接触或安全边界违反的实验也必须保留。
