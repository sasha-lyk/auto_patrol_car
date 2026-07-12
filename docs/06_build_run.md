# 06 · 构建、部署与运行

## 6.1 环境依赖

树莓派 4B + Ubuntu 22.04 + ROS2 Humble。

```bash
# ROS2 功能包
sudo apt update
sudo apt install -y \
  ros-humble-slam-toolbox \
  ros-humble-navigation2 ros-humble-nav2-bringup \
  ros-humble-robot-localization \
  ros-humble-rplidar-ros \
  ros-humble-nav2-simple-commander \
  ros-humble-tf-transformations

# Python 依赖
pip3 install pyserial flask flask-cors
# GPIO(二选一,lgpio 更现代)
pip3 install lgpio        # 或: sudo apt install python3-rpi.gpio
sudo apt install -y python3-gpiozero
```

将当前用户加入 gpio/dialout 组以免 sudo:

```bash
sudo usermod -aG gpio,dialout $USER   # 重新登录生效
```

## 6.2 编译

```bash
cd ~/my_car        # 工作空间根(含 src/)
colcon build --symlink-install
source install/setup.bash
# 建议加进 ~/.bashrc: source ~/my_car/install/setup.bash
```

## 6.3 分步启动与验证

### A. 只启底盘(先验证闭环)

```bash
ros2 launch raspi_car_base l298n_base.launch.py
```

验证清单:

```bash
ros2 topic echo /car01/wheel_speeds_std      # 推动轮子应有非零读数
ros2 topic echo /car01/odom                  # 推动车,x/y/yaw 应变化
ros2 topic echo /car01/base_controller/status # 看 target vs meas
# 发一个前进指令测试 PID:
ros2 topic pub -r 10 /car01/cmd_vel geometry_msgs/Twist "{linear: {x: 0.15}}"
```

### B. 加 EKF + IMU(验证 TF)

base_with_mux 已包含 EKF 与静态 TF:

```bash
ros2 launch raspi_car_bringup base_with_mux.launch.py
ros2 run tf2_tools view_frames    # 生成 frames.pdf,应看到 odom→base_link→laser_frame
ros2 topic echo /car01/odometry/filtered  # EKF 输出(若配置了输出话题)
```

### C. 建图

```bash
ros2 launch raspi_car_bringup mapping.launch.py
# 另开终端,用网页或键盘遥控走一圈
ros2 run nav2_map_server map_saver_cli -f ~/my_car/src/raspi_car_bringup/maps/room1 \
     --ros-args -r __ns:=/car01
colcon build --symlink-install    # 让新地图进 install
```

### D. 自主导航 / 巡逻

```bash
ros2 launch raspi_car_bringup nav2.launch.py     # 导航(在 RViz 里点 2D Goal)
# 或
ros2 launch raspi_car_bringup patrol.launch.py   # 仅接受已现场标定且地图匹配的路线
```

### E. Web 控制台

```bash
ros2 launch raspi_car_web web_control.launch.py
# 浏览器打开 http://<pi_ip>:8080
```

## 6.4 无硬件 / 干跑模式

没接 GPIO 也能跑通整张计算图(用于开发调试与功能演示):

```bash
# l298n.yaml 里设 dry_run: true;encoder 也支持 dry_run
ros2 launch raspi_car_base l298n_base.launch.py
# 电机/编码器不操作真实 GPIO,话题照常流转,Web 可显示(轮速为0)
```

核心控制、安全和里程计可先执行 `python3 tools/simulate_subsystems.py`。它提供确定性
回归证据，但不能代替 ROS2/Gazebo 或实车。完整边界见 `docs/08_validation.md`。

## 6.5 标定顺序(真机)

1. 电机方向(`invert_left/right`, `reverse_forward`)
2. 编码器符号(前进时 `wheel_speeds` 为正)
3. 里程计尺度(直行 1m 校 `wheel_radius`;转 360° 校 `wheel_separation`)
4. PID 整定(`pid_kp/ki/kd`:先 P 后 I,消除稳态误差,D 抑制抖动)
5. 雷达零位/偏装角(`laser_front_angle_deg`,或核对静态 TF)
6. 巡逻点：运行 `record_waypoints.launch.py`，在每个现场位置调用 `capture_waypoint`，
   最后调用 `save_route`。不要手抄示例坐标。

### 真实场地路线采集

```bash
ros2 launch raspi_car_bringup record_waypoints.launch.py
# 移动到每个点后重复执行
ros2 service call /car01/capture_waypoint std_srvs/srv/Trigger {}
# 至少两个点后保存
ros2 service call /car01/save_route std_srvs/srv/Trigger {}
```

采集器会拒绝陈旧或协方差过大的 AMCL 位姿，保存文件会绑定地图 YAML 和图像的
SHA-256。巡逻每圈结果写入 `~/.ros/raspi_car/logs/patrol_metrics.jsonl`。

## 6.6 常见问题排查

| 现象 | 可能原因 | 排查 |
|------|---------|------|
| odom 不动 | 编码器没计数 | `ros2 topic echo wheel_ticks_std`,查接线/电平 |
| 车走歪 | 左右轮径/PID 不一致 | 校 `wheel_radius`,分别整定 PID |
| SLAM 重影 | odom 漂 / TF 抖 | 查 EKF 是否在跑、`view_frames` |
| AMCL 定位丢 | 初值错 / 里程差 | 核对 `initial_pose`,先跑通 EKF |
| 导航原地转 | 用了会转的插件 | 确认 `use_rotate_to_heading:false` |
| Web 无数据 | 话题名/命名空间不符 | 查 `CAR_NAMESPACE`,`ros2 topic list` |

下一章:[07 框图汇总](07_diagrams.md) · 另见 [FAQ 设计决策与常见问题](FAQ.md)
