# 工程导读与代码阅读顺序

本仓库的闭环是：Gazebo 深度图和里程计 → 椭圆安全集 → CBF-QP → 过滤速度 → Gazebo 差速底盘。

## 建议阅读顺序

1. `config/params.yaml`：话题名、相机 FOV、椭圆参数、速度边界和坐标系参数。
2. `safe_obstacle_avoidance/geometry.py`：纯数学处理，从深度行生成椭圆参数；自带水平安装相机选择中心行（参数 `depth_row_fraction=0.50`），其他传感器需按图像和相机俯仰重新标定。
3. `safe_obstacle_avoidance/safe_set_generator.py`：ROS 2 图像/里程计订阅和安全集发布。
4. `safe_obstacle_avoidance/qp.py`：二维严格凸 QP 求解器。
5. `safe_obstacle_avoidance/cbf_controller.py`：计算 CBF 不等式并输出速度。
6. `safe_obstacle_avoidance/reference_cmd_publisher.py`：仅自带仿真用的限时参考速度源。
7. `launch/safe_obstacle_avoidance.launch.py`：Gazebo、bridge 和算法节点启动。
8. `simulation/worlds/ellipsoid_obstacle_world.sdf`：差速机器人、深度相机和障碍物。

## 椭圆安全集

图像列得到轴向深度 `z` 和水平角 `theta`，再转换为平面径向距离：

\[r = \frac{z}{|\sin(\theta)|}\]

给定最大构造距离 `r_max`，每个方向生成椭圆：

\[\rho_c=\frac{r_{max}+r}{2},\quad a=\frac{r_{max}-r}{2}+d_s,\quad b=b_i\]

\[c=\rho_c[\cos\theta,\sin\theta]^T,\quad \phi=\theta\]

消息每一行是 `[cx, cy, a, b, phi]`，椭圆外侧定义为安全侧。

## 椭圆 CBF 和 QP

旋转椭圆：

\[h_i(p)=(p-c_i)^T A_i(p-c_i)-1,\quad A_i=R_i\operatorname{diag}(a_i^{-2},b_i^{-2})R_i^T\]

安全集合为 `h_i >= 0`。控制器采用 unicycle 运动学和原推导中的参考点偏置 `l`，求解：

\[\min_{\delta_u}\;\frac12\delta_u^TQ\delta_u\quad\text{s.t.}\quad\dot h_i+\gamma h_i\ge0\]

同时满足最终速度的正负边界，输出 `u = u_ref + delta_u`。

## 二维 QP 求解器

控制量只有两个维度。正定对角 `Q` 下，最优点只可能是原点、某一约束边界上的 Q-度量投影、或两条边界的交点。`qp.py` 枚举候选点并检查可行性，不依赖 CVXOPT/CasADi。该求解器只适用于此处的二维 QP，不是通用优化器。

## 坐标系与传感器安装

自带底盘遵循 REP-103，前方是 base 的 +X。椭圆算法内部前向为局部 +Y，因此控制器会把 odometry 转到安全集局部平面。安全集原点按相机位置修正，`camera_offset_x/y` 表示相机相对底盘中心的平面偏移。

接入自己的底盘时，重点逐项校准 `camera_offset_x/y`、`yaw_offset`、`linear_velocity_sign`、`angular_velocity_sign` 和 `robot_offset_l`。不要把其他模型的符号配置直接照搬。

## 运行保护

里程计、参考速度或安全集超时会触发零速度；当前状态落入椭圆禁区、QP 无可行解或计算异常时同样停车。这是保守的 fail-closed 行为，不等于传感器噪声/模型误差下的形式化安全保证。
