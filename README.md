# auto_patrol_car — 树莓派编码器闭环自主巡检小车

![CI](https://github.com/sasha-lyk/auto_patrol_car/actions/workflows/ci.yml/badge.svg)
![ROS2](https://img.shields.io/badge/ROS2-Humble-blue)
![License](https://img.shields.io/badge/license-Apache--2.0-green)

> ROS2 (Humble) · 差速底盘 · 轮速编码器闭环 · IMU/编码器 EKF 融合 · slam_toolbox 建图 · Nav2 导航 · Web 远程控制

一个基于 ROS2 的树莓派差速驱动自主巡检机器人。通过**轮速编码器**构成两层闭环,
从底层保证运动与定位的可靠性:

- **控制闭环**:编码器实测轮速 → PID → 电机 PWM。在已标定的负载、电压与地面范围内抑制轮速偏差。
- **估计闭环**:编码器里程计 + IMU → 扩展卡尔曼滤波(EKF)融合 → 稳定的 `odom→base_link` 位姿。

在此基础上运行 slam_toolbox 建图、Nav2 自主导航与定点巡逻,并提供 Web 远程控制与实时遥测。

---

## 功能特性

- 🛞 **正交编码器 x4 解码**,毫米级里程分辨率
- 🎯 **PID 速度闭环**,带前馈、积分限幅、启动死区补偿与反馈超时回退
- 🧭 **robot_localization EKF** 融合编码器里程计与 IMU,抑制航向漂移
- 🗺️ **slam_toolbox** 建图 + **AMCL** 定位
- 🚗 **Smac Hybrid-A\*** 全局规划 + **Regulated Pure Pursuit** 局部控制,针对不可原地转向的车辆生成运动学可行路径
- 🛡️ **锁存式急停 + cmd_vel 优先级仲裁**(网页 > 遥控 > 导航)+ collision_monitor 停障
- 🧱 **反应式沿墙巡逻**,不依赖地图/定位,作为降级方案
- 🌐 **Flask + ROS2 Web 控制台**,实时可视化位姿与闭环轮速

## 目录结构

```
auto_patrol_car/
├── README.md                      # 项目总览 + 快速开始
├── LICENSE                        # Apache-2.0
├── USER_MANUAL.md                 # 用户手册(面向使用者)
├── docs/                          # 设计文档
│   ├── 01_architecture.md         # 系统架构、节点图、话题/TF 全表
│   ├── 02_hardware_wiring.md      # 硬件清单 + 编码器/电机/IMU 接线
│   ├── 03_closed_loop_theory.md   # 闭环原理:编码器里程计 + PID + EKF
│   ├── 04_slam_navigation.md      # SLAM 建图 + Nav2 导航管线
│   ├── 05_web_control.md          # Web 控制层设计与 API
│   ├── 06_build_run.md            # 编译、部署、运行、标定
│   ├── 07_diagrams.md             # 数据流 / TF / 控制框图(ASCII)
│   ├── architecture_diagrams.md   # 架构图(Mermaid,GitHub 自动渲染)
│   └── FAQ.md                     # 设计决策与常见问题
└── src/
    ├── raspi_car_base/            # 底盘:编码器 + 闭环里程计 + PID 电机
    ├── raspi_car_sensors/         # 传感器:YB_MRA02 IMU UART 驱动
    ├── raspi_car_bringup/         # 编排:EKF / SLAM / Nav2 / 巡逻 / mux
    └── raspi_car_web/             # Web:Flask ROS2 桥 + 仪表盘前端
```

## 快速开始

```bash
# 1. 放入工作空间并编译
cd ~/auto_patrol_car && colcon build --symlink-install
source install/setup.bash

# 2a. 建图(遥控走一圈后存图)
ros2 launch raspi_car_bringup mapping.launch.py
ros2 run nav2_map_server map_saver_cli -f src/raspi_car_bringup/maps/room1 --ros-args -r __ns:=/car01

# 2b. 已有地图 -> 自主导航
ros2 launch raspi_car_bringup nav2.launch.py

# 2c. 自主巡逻(循环走 waypoints)
ros2 launch raspi_car_bringup patrol.launch.py

# 3. 另开终端启动 Web 控制台(浏览器访问 http://<pi_ip>:8080)
ros2 launch raspi_car_web web_control.launch.py
```

首次部署不能直接运行巡逻。必须先在当前地图中采集真实 AMCL 位姿并生成与地图
SHA-256 指纹绑定的路线，详见 [`docs/06_build_run.md`](docs/06_build_run.md)。未标定路线会被
`patrol_node` 拒绝，避免示例坐标驱动车辆。

无需 ROS/GPIO 的核心验证：

```bash
python3 -m unittest discover -s tests -v
python3 tools/simulate_subsystems.py
```

实车巡逻会自动记录逐圈结果，可用 `python3 tools/summarize_patrol_metrics.py` 生成可追溯的
成功率和耗时统计。

依赖安装、分步验证与标定见 [`docs/06_build_run.md`](docs/06_build_run.md),
日常使用见 [`USER_MANUAL.md`](USER_MANUAL.md)。

## 硬件基线

| 部件 | 型号/规格 |
|------|-----------|
| 主控 | 树莓派 4B(Ubuntu 22.04 + ROS2 Humble) |
| 电机驱动 | L298N 双 H 桥 |
| 电机 | 带正交编码器的减速电机 ×2(差速两驱) |
| 激光雷达 | RPLIDAR A1 |
| IMU | YB_MRA02 九轴(UART) |

底盘为重载差速车,采用两驱 + 万向轮结构,不能原地转向 —— 这一约束贯穿导航规划与控制的整体设计。

## 系统架构一览

```
任务层    patrol_node(FollowWaypoints) / wall_follower
导航层    Nav2: AMCL · Smac-Hybrid · RPP · collision_monitor
建图/估计 slam_toolbox(建图) · robot_localization EKF(定位)
底盘层    cmd_vel_mux · l298n_motor(PID) · wheel_odometry · encoder
传感器层  rplidar_a1(scan) · yb_mra02(imu) · encoder(ticks)
```

详见 [`docs/01_architecture.md`](docs/01_architecture.md);
可视化架构图见 [`docs/architecture_diagrams.md`](docs/architecture_diagrams.md)。

## 上传到 GitHub

```bash
cd auto_patrol_car
git init
git add .
git commit -m "Initial commit: encoder closed-loop autonomous patrol car"
git branch -M main
git remote add origin https://github.com/sasha-lyk/auto_patrol_car.git
git push -u origin main
```

推送后到仓库 Settings → Actions 确认工作流已启用;首次 push 会自动触发 CI。

## 许可证

Apache License 2.0,详见 [LICENSE](LICENSE)。
