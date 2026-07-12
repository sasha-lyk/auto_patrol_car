# 用户手册 · auto_patrol_car

面向使用者的日常操作指南。开发/标定细节见 [`docs/06_build_run.md`](docs/06_build_run.md)。

## 1. 首次准备

1. 按 [`docs/02_hardware_wiring.md`](docs/02_hardware_wiring.md) 完成接线,确认电机、编码器、
   雷达(/dev/ttyUSB0)、IMU(/dev/serial0)供电与连接正常。
2. 在树莓派上安装依赖并编译(见 [`docs/06_build_run.md`](docs/06_build_run.md) 第 6.1–6.2 节)。
3. 首次使用务必按 6.5 节完成标定(电机方向、编码器符号、里程计尺度、PID)。

## 2. 三种工作模式

### 2.1 建图模式

先给环境建一张地图:

```bash
ros2 launch raspi_car_bringup mapping.launch.py
```

用 Web 控制台或手柄把机器人**平稳地**在场地里走一圈,尽量回到起点(触发回环优化)。
建图满意后保存:

```bash
ros2 run nav2_map_server map_saver_cli -f src/raspi_car_bringup/maps/room1 \
     --ros-args -r __ns:=/car01
colcon build --symlink-install    # 让新地图生效
```

### 2.2 自主导航模式

加载已保存地图,在 RViz 中用 "2D Goal Pose" 指定目标点:

```bash
ros2 launch raspi_car_bringup nav2.launch.py
```

机器人会自主规划路径、避障并抵达目标。

### 2.3 自主巡逻模式

按预设航点循环巡逻:

```bash
ros2 launch raspi_car_bringup patrol.launch.py
```

航点不能手填示例值。首次进入真实场地时启动采集模式：

```bash
ros2 launch raspi_car_bringup record_waypoints.launch.py
```

将机器人遥控到每个巡逻点后执行：

```bash
ros2 service call /car01/capture_waypoint std_srvs/srv/Trigger {}
```

至少采集两个点，最后保存：

```bash
ros2 service call /car01/save_route std_srvs/srv/Trigger {}
```

生成的路线默认位于 `~/.ros/raspi_car/routes/room1_patrol.yaml`，包含当前地图指纹。
巡逻启动时会拒绝未标定路线、非法坐标以及与当前地图不匹配的路线。

## 3. Web 远程控制

启动 Web 控制台(可与上述任一模式同时运行):

```bash
ros2 launch raspi_car_web web_control.launch.py
```

浏览器访问 `http://<树莓派IP>:8080`。

**操作方式**

| 操作 | 说明 |
|------|------|
| 方向按钮 / WASD / 方向键 | 前进、后退、左转、右转(按住持续,松开停止) |
| 空格 | 停止 |
| 低速 / 中速 / 高速 | 切换速度档位 |
| 紧急停止(E-STOP) | 立即停车并锁存，超时不会自动恢复 |
| 复位急停 | 确认环境安全后显式恢复控制；旧速度指令会被清空 |

**遥测面板**

- 位姿 & IMU:实时 x/y/航向、线/角速度、姿态、数据新鲜度。
- 闭环状态:左右轮"目标 vs 实测轮速"对比、PWM 占空比、控制模式、运行徽章。

## 4. 安全须知

- Web 控制台默认无鉴权,**仅限可信局域网使用**;暴露到公网前请加鉴权与 HTTPS。
- 人工遥控优先级高于自主导航；E-STOP 一旦触发会持续禁止电机输出，必须显式复位。
- 首次运行或更换场地/负载后,建议先低速测试并确认急停有效。

## 5. 常见现象排查

| 现象 | 可能原因 | 处理 |
|------|---------|------|
| 里程计不动 | 编码器无计数 | 检查编码器接线与电平 |
| 机器人走偏 | 左右轮径或 PID 不一致 | 校准 `wheel_radius`,分别整定 PID |
| 建图重影 | 里程计漂移 / TF 抖动 | 确认 EKF 正常运行 |
| 定位丢失 | 初始位姿错误 | 核对 `initial_pose` |
| Web 无数据 | 命名空间/话题不符 | 检查 `CAR_NAMESPACE` 与 `ros2 topic list` |

更完整的排查见 [`docs/06_build_run.md`](docs/06_build_run.md) 第 6.6 节。
