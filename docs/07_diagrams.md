# 08 · 框图汇总(ASCII)

## 8.1 节点 / 话题数据流全景

```
                          ┌───────────────┐
                          │  rplidar_node │  /car01/scan (LaserScan)
                          └──────┬────────┘
                                 │
   ┌───────────────┐            ├──────────────┬───────────────┬─────────────┐
   │ yb_mra02_imu  │ /imu       │              │               │             │
   └──────┬────────┘            ▼              ▼               ▼             ▼
          │              slam_toolbox      amcl          costmaps     collision_monitor
          │              (map→odom)     (map→odom)      (obstacle)    (stop/slow)
          │                   │              │               │             │
          │                   └──────┬───────┘               │             │ cmd_vel_nav
          ▼                          ▼                        ▼             ▼
   ┌──────────────┐            map frame              planner(Smac) → controller(RPP)
   │ encoder_node │                                        │  cmd_vel_raw       │
   └──┬────────┬──┘                                        └────────────────────┘
      │ ticks  │ speeds
      ▼        ▼
 wheel_odom  l298n_motor(PID) ◄──────────────── cmd_vel ◄─── cmd_vel_mux
      │ odom      ▲                                              ▲  ▲  ▲  ▲
      │           │ PWM/GPIO                          emergency ─┘  │  │  └─ nav
      ▼           ▼                                        web_asst─┘  └─teleop
    ekf_filter_node ──► TF: odom → base_link                 ▲
      ▲                                                       │ turn_assist
      └─ odom(编码器) + imu(IMU)                    cmd_vel_web │
                                                      ▲         │
                                              web_cmd_bridge ───┘
                                                (Flask /api/cmd)
```

## 8.2 TF 树

```
map
 │  (slam_toolbox 建图 / amcl 导航)
 ▼
odom
 │  (robot_localization EKF —— 唯一权威)
 ▼
base_link
 ├─(static)─► laser_frame   (0.12, 0, 0.10)
 └─(static)─► imu_link       (0, 0, 0.06)
```

## 8.3 两层闭环控制框图

```
      指令 v,ω
        │  差速分解
        ▼
   v_target(L/R)
        │
        ▼
   ┌─────────┐  误差   ┌─────────┐  PWM   ┌──────┐   转   ┌──────┐
   │ Σ (−)   ├───────►│  PID+FF ├───────►│ L298N├──────►│ 电机 │
   └────▲────┘        └─────────┘        └──────┘        └───┬──┘
        │ v_measured                                        │
        │                                                    ▼
        │            ┌──────────┐   ticks/speed        ┌──────────┐
        └────────────┤ encoder  │◄─────────────────────┤ 轮子转动  │
                     └────┬─────┘                       └──────────┘
                          │ ticks
                          ▼
                   wheel_odometry ── odom ──┐
                                            ▼
                          imu ──────►   EKF 融合  ──► odom→base_link(位姿)
                                            │
                                            ▼
                              SLAM/AMCL 定位 · Nav2 导航
```

## 8.4 cmd_vel 优先级仲裁

```
优先级   来源话题                  超时
 255  cmd_vel_emergency  ─┐      0.3s
 100  cmd_vel_web_assisted├─► mux 选当前"未超时且最高优先级" ─► cmd_vel
  90  cmd_vel_teleop     ─┤      0.8s
  50  cmd_vel_nav        ─┘      0.6s
       (都超时 → 输出 0 停车)
```

## 8.5 启动依赖关系

```
l298n_base.launch        (encoder + wheel_odom + motor)
      ▲
base_with_mux.launch  = l298n_base + IMU + EKF + mux + turn_assist + static TF
      ▲                                   ▲
mapping.launch                      nav2.launch = base_with_mux + lidar + Nav2
 = base_with_mux + lidar + slam           ▲
                                    patrol.launch = nav2 + patrol_node

web_control.launch (独立,连已有 mux/话题)
```
