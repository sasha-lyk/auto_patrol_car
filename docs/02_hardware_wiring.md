# 02 · 硬件与接线

## 2.1 硬件清单(BOM)

| 部件 | 型号/规格 | 接口 | 备注 |
|------|-----------|------|------|
| 主控 | 树莓派 4B (4/8GB) | — | Ubuntu 22.04 + ROS2 Humble |
| 电机驱动 | L298N 双 H 桥 | GPIO PWM+方向 | 每路 ENA/IN1/IN2 |
| 减速电机 ×2 | 带正交编码器(霍尔/光电) | GPIO(编码器) | 减速比示例 90:1 |
| 编码器 | 增量正交 A/B 相 | 4 根信号线 | CPR 示例 11(单相/电机圈) |
| 激光雷达 | RPLIDAR A1 | USB(/dev/ttyUSB0) | 车头 +x,偏装约 -85° |
| IMU | YB_MRA02 九轴 | UART(/dev/serial0) | 115200 |
| 电源 | 电池 + 稳压 | — | 逻辑与电机分开供电更稳 |
| 轮系 | 差速两驱 + 万向轮 | — | 重载,不能原地转 |

## 2.2 关键几何参数(须实测标定)

写在 `src/raspi_car_base/config/l298n.yaml`:

| 参数 | 含义 | 示例值 | 如何得到 |
|------|------|--------|---------|
| `wheel_radius` | 轮子半径 (m) | 0.0325 | 卡尺量直径 ÷2 |
| `wheel_separation` / `wheel_base` | 左右轮间距 (m) | 0.20 | 量两轮接地中心距 |
| `encoder_cpr` | 编码器单相每电机圈计数 | 11 | 查电机手册 |
| `gear_ratio` | 减速比(电机:轮) | 90.0 | 查电机手册 |

由此推出(代码自动算):

```
ticks_per_wheel_rev = 4 × CPR × gear_ratio      # x4 正交解码
meters_per_tick     = 2π × wheel_radius / ticks_per_wheel_rev
```

示例:`4 × 11 × 90 = 3960 ticks/轮圈`,`meters_per_tick ≈ 0.0000516 m`。
分辨率约 0.05mm/tick,足够里程计使用。

## 2.3 GPIO 接线(BCM 编号)

### L298N 电机(默认)

| 功能 | BCM | 说明 |
|------|-----|------|
| 左 PWM (ENA) | 18 | 硬件 PWM 引脚 |
| 左 IN1 | 23 | 方向 |
| 左 IN2 | 24 | 方向 |
| 右 PWM (ENB) | 13 | 硬件 PWM 引脚 |
| 右 IN1 | 27 | 方向 |
| 右 IN2 | 22 | 方向 |

### 正交编码器(新增)

| 功能 | BCM | 说明 |
|------|-----|------|
| 左编码器 A | 5 | 上升+下降沿都计 |
| 左编码器 B | 6 | 判方向 |
| 右编码器 A | 16 | |
| 右编码器 B | 26 | |

> 编码器信号建议加上拉(代码里 RPi.GPIO 后端已启用 `PUD_UP`)。
> 若编码器供电为 5V 而树莓派 GPIO 为 3.3V,**必须加电平转换或分压**,否则烧 GPIO。

### 串口设备

| 设备 | 节点 | 波特率 |
|------|------|--------|
| RPLIDAR A1 | /dev/ttyUSB0 | 115200 |
| YB_MRA02 IMU | /dev/serial0 | 115200 |

## 2.4 传感器安装位姿(静态 TF)

在 `base_with_mux.launch.py` 里以 `static_transform_publisher` 发布:

| 变换 | 平移 xyz (m) | 旋转 | 说明 |
|------|--------------|------|------|
| `base_link→laser_frame` | 0.12, 0, 0.10 | 0,0,0 | 雷达在车头偏上 |
| `base_link→imu_link` | 0, 0, 0.06 | 0,0,0 | IMU 居中偏上 |

> 雷达"偏装角"(约 -85°)不在静态 TF 里补,而是在反应式 `wall_follower` 里用
> `laser_front_angle_deg` 处理;导航链路里雷达数据经 `laser_frame` 参与 costmap,
> 角度已由 TF + 雷达驱动 `angle_compensate` 处理。真机标定时以实际雷达零位为准。

## 2.5 电机方向/极性标定顺序

1. 设 `dry_run: true`,启动 base,发一个 `+x` 前进指令,看日志 PWM 方向。
2. 设 `dry_run: false`,轮子悬空,发前进:确认**两轮都朝前**。反了就翻 `invert_left/right`。
3. 发前进,读 `/car01/wheel_speeds_std`:两个值应为**正**。若某轮为负,翻该轮编码器的 `invert_*` 或对调 A/B 线。
4. 让车直行 1m,读 `/car01/odom` 的 x 是否 ≈ 1.0;不准就微调 `wheel_radius`。
5. 让车原地转 360°(手推),读 odom 的 yaw;不准就微调 `wheel_separation`。

下一章:[03 闭环理论](03_closed_loop_theory.md)
