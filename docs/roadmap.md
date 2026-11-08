# 复现路线图

本工程按“先确认环境，再验证传感器，再打开闭环”的顺序推进。每个阶段都应保存终端输出，不要只以节点进程存在作为成功标准。

## 阶段 0：软件环境

- [ ] Ubuntu 26.04 LTS
- [ ] ROS 2 Lyrical 已安装并 `source /opt/ros/lyrical/setup.bash`
- [ ] Gazebo Jetty 已安装，`gz sim` 可启动（本项目不安装 Gazebo）
- [ ] `./scripts/check_env.sh` 全部必需项通过

## 阶段 1：构建工作空间

- [ ] `./scripts/install_ros_deps.sh`
- [ ] `./scripts/build.sh`
- [ ] `ros2 pkg prefix safe_obstacle_avoidance_msgs`
- [ ] `ros2 pkg prefix safe_obstacle_avoidance`

## 阶段 2：Gazebo 传感器和底盘

- [ ] `./scripts/run_demo.sh gui:=false`
- [ ] `gz topic -l` 能看到 `/camera/depth_image`、`/model/safe_bot/odometry` 和 `/model/safe_bot/cmd_vel`
- [ ] `ros2 topic hz /camera/depth/image_raw` 有稳定图像频率
- [ ] `ros2 topic hz /odom` 有稳定里程计频率
- [ ] 用小速度命令确认 `/cmd_vel` 能驱动机器人；第一次先确认方向符号

## 阶段 3：椭圆安全集

- [ ] `/safe_set_config` 持续发布
- [ ] `columns == 5`
- [ ] `data` 每 5 个浮点数对应一行 `[cx, cy, a, b, theta]`
- [ ] `a > 0`、`b > 0`，数据无 NaN/Inf
- [ ] 深度图中心射线前方障碍物对应局部 `+Y` 方向

## 阶段 4：CBF-QP 闭环

- [ ] `/ref_vel` 有参考速度输入（自动 demo 或键盘）
- [ ] `/cmd_vel` 是 CBF 唯一输出端
- [ ] 正常时输出速度与参考速度接近
- [ ] 安全约束激活时，输出速度按约束调整
- [ ] 输入超时、QP 不可行或进入椭圆禁区时输出零速度

## 阶段 5：实验留档

- [ ] `scripts/record_run.py` 保存原始 CSV
- [ ] `scripts/analyze_run.py` 生成指标 JSON 和轨迹 PNG
- [ ] 至少重复 3 次，保留失败/碰撞实验
- [ ] 把 ROS / Gazebo 版本、虚拟机 CPU/内存、参数文件和 Git commit 写入实验记录

## 阶段 6：接入 TurtleBot3

- [ ] 替换深度图 Gazebo/ROS 话题映射
- [ ] 校准相机相对底盘的平移和 yaw
- [ ] 确认相机左右列与角度方向一致
- [ ] 确认 `cmd_vel` 正负号与底盘运动一致
- [ ] 低速空场验证后再加入障碍物
