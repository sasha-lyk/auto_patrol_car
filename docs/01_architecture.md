# 01 · 系统架构

## 1.1 分层总览

系统分五层,自下而上:

```
┌─────────────────────────────────────────────────────────────┐
│  任务层   patrol_node(FollowWaypoints 循环) / wall_follower     │
├─────────────────────────────────────────────────────────────┤
│  导航层   Nav2: AMCL 定位 · Smac-Hybrid 规划 · RPP 控制         │
│           collision_monitor 安全 · behavior 恢复                │
├─────────────────────────────────────────────────────────────┤
│  建图/估计 slam_toolbox(建图) · robot_localization EKF(定位)  │
├─────────────────────────────────────────────────────────────┤
│  底盘层   cmd_vel_mux · l298n_motor(PID) · wheel_odometry      │
│           encoder_node(正交解码)                                │
├─────────────────────────────────────────────────────────────┤
│  传感器层 rplidar_a1(scan) · yb_mra02(imu) · encoder(ticks)   │
└─────────────────────────────────────────────────────────────┘
```

所有节点运行在 `car01` 命名空间下(`/car01/...`),便于多机扩展与话题隔离。

## 1.2 节点清单

| 节点 | 包 | 职责 | 关键输入 | 关键输出 |
|------|----|----|---------|---------|
| `encoder_node` | base | 正交编码器 x4 解码 | GPIO 边沿 | `wheel_ticks_std`, `wheel_speeds_std` |
| `wheel_odometry_node` | base | 编码器闭环里程计 | `wheel_ticks_std` | `odom` |
| `l298n_motor_node` | base | PID 速度闭环 + 驱动 | `cmd_vel`, `wheel_speeds_std` | GPIO PWM, `base_controller/status` |
| `yb_mra02_uart_node` | sensors | IMU UART 驱动 | 串口帧 | `imu`, `mag` |
| `rplidar_node` | (rplidar_ros) | 激光雷达驱动 | USB 串口 | `scan` |
| `ekf_filter_node` | (robot_localization) | 编码器+IMU 融合 | `odom`, `imu` | TF `odom→base_link` |
| `slam_toolbox` | (slam_toolbox) | 建图 | `scan`, TF | `map`, TF `map→odom` |
| `amcl` | (nav2) | 已知地图定位 | `scan`, `map`, TF | TF `map→odom` |
| `planner_server` | (nav2) | 全局路径 | `map`, costmap | path |
| `controller_server` | (nav2) | 局部跟踪 | path, `scan` | `cmd_vel_raw` |
| `collision_monitor` | (nav2) | 碰撞减速/急停 | `cmd_vel_raw`, `scan` | `cmd_vel_nav` |
| `cmd_vel_mux` | bringup | 优先级仲裁 | 4 路 cmd_vel | `cmd_vel` |
| `turn_assist` | bringup | 旋转→弧线改写 | `cmd_vel_web` | `cmd_vel_web_assisted` |
| `patrol_node` | bringup | 循环巡逻 | Nav2 action | — |
| `wall_follower_node` | bringup | 反应式沿墙(兜底) | `scan` | `cmd_vel_nav` |
| `web_cmd_bridge` | web | HTTP↔ROS2 桥 | HTTP, `odom`/`imu`/`status` | `cmd_vel_web`, `cmd_vel_emergency` |

## 1.3 话题接口总表

| 话题 | 类型 | 发布者 | 订阅者 |
|------|------|--------|--------|
| `/car01/scan` | sensor_msgs/LaserScan | rplidar | slam/amcl/costmap/collision/wall_follower |
| `/car01/imu` | sensor_msgs/Imu | yb_mra02 | ekf, web |
| `/car01/wheel_ticks_std` | std_msgs/Int32MultiArray | encoder | wheel_odometry |
| `/car01/wheel_speeds_std` | std_msgs/Float32MultiArray | encoder | l298n_motor |
| `/car01/odom` | nav_msgs/Odometry | wheel_odometry | ekf, nav2, web |
| `/car01/cmd_vel_web` | geometry_msgs/Twist | web | turn_assist |
| `/car01/cmd_vel_web_assisted` | geometry_msgs/Twist | turn_assist | mux(web源) |
| `/car01/cmd_vel_teleop` | geometry_msgs/Twist | (手柄) | mux(teleop源) |
| `/car01/cmd_vel_nav` | geometry_msgs/Twist | collision_monitor / wall_follower | mux(nav源) |
| `/car01/cmd_vel_emergency` | geometry_msgs/Twist | web(E-STOP) | mux(急停源) |
| `/car01/cmd_vel` | geometry_msgs/Twist | mux | l298n_motor, wheel? (见下) |
| `/car01/base_controller/status` | std_msgs/String(JSON) | l298n_motor | web |

> 注意:`wheel_odometry_node` 里程计只来自**编码器**,不再订阅 `cmd_vel`。
> 这是本设计的关键点——里程计反映"实际动了多少",而不是"命令它动多少"(后者即开环里程计的做法)。

## 1.4 TF 树

```
map
 └─(slam_toolbox 建图时 / amcl 导航时发布)
odom
 └─(robot_localization EKF 发布 —— 唯一权威)
base_link
 ├─(静态)→ laser_frame   [0.12, 0, 0.10]
 └─(静态)→ imu_link       [0, 0, 0.06]
```

关键设计约束:**`odom→base_link` 只能有一个发布者**。本项目交给 EKF。
因此 `wheel_odometry_node` 的 `publish_tf=false`,IMU 驱动也不发 TF。
这避免了 "TF 树被两个源打架" 这一常见 Nav2 事故。

## 1.5 命令速度(cmd_vel)优先级链

```
cmd_vel_emergency (255) ─┐
cmd_vel_web_assisted(100)─┤
cmd_vel_teleop     (90) ─┤──► cmd_vel_mux ──► cmd_vel ──► l298n_motor
cmd_vel_nav        (50) ─┘
```

任一路超时(emergency 0.3s / web 0.8s / teleop 0.8s / nav 0.6s)即自动降级,
急停永远最高优先级。这保证了"人可以随时抢过自主导航的控制权,急停一定生效"。

下一章:[02 硬件与接线](02_hardware_wiring.md)
