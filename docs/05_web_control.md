# 05 · Web 控制层

## 5.1 目标

在手机/电脑浏览器里远程控制小车,并**实时看到闭环状态**(位姿、目标 vs 实测轮速、
PWM、数据新鲜度)。遥测数据均来自**实时订阅 ROS2 话题**,如实反映底盘运行状态。

## 5.2 架构

```
浏览器(index.html + app.js)
   │  HTTP POST /api/cmd  { "c": "F" }        ← 运动/急停/调速
   │  HTTP GET  /api/status                    ← 200ms 轮询遥测
   ▼
Flask 后端 backend_ros2.py(web_cmd_bridge 节点)
   │  发布 → /car01/cmd_vel_web       (普通指令,经 turn_assist → mux)
   │  发布 → /car01/emergency_stop       (True=触发并锁存)
   │  发布 → /car01/emergency_stop_reset (True=人工显式复位)
   │  订阅 ← /car01/emergency_stop_latched
   │  订阅 ← /car01/odometry/filtered (EKF 融合位姿:x,y,yaw,v,w)
   │  订阅 ← /car01/imu               (roll,pitch,gyro)
   │  订阅 ← /car01/base_controller/status (目标/实测轮速,PWM,闭环标志)
   ▼
后台线程 rclpy.spin(node) 持续接收,主线程 Flask 提供 HTTP
```

## 5.3 指令流(为什么要经过 turn_assist 和 mux)

网页只发"高层意图"(前后左右 + 速度档),后端翻译成 `Twist`:

```
F → linear.x = +档位线速度
B → linear.x = −档位线速度
L → angular.z = +档位角速度
R → angular.z = −档位角速度
E → 发 emergency_stop = true(锁存急停)
X → 发 emergency_stop_reset = true(确认安全后复位)
```

`cmd_vel_web` 先进 `turn_assist`:若是"纯转"(角速度≠0、线速度≈0),
自动注入一点前进速度改成弧线(车不能原地转)。再进 `cmd_vel_mux` 与导航/急停仲裁。
**网页控制天然优先级高于自主导航**,所以人一操作就接管,松手回到自主。

## 5.4 HTTP API

| 方法 | 路径 | 请求 | 响应 | 说明 |
|------|------|------|------|------|
| GET | `/` | — | index.html | 仪表盘页面 |
| GET | `/api/status` | — | JSON 遥测快照 | 200ms 轮询 |
| GET | `/api/health` | — | `{ok,uptime}` | 健康检查 |
| POST | `/api/cmd` | `{"c":"F"}` | `{ok,cmd,speed}` | F/B/L/R/S/E/1/2/3 |

`/api/status` 返回(节选):

```json
{
  "online": true, "namespace": "car01",
  "x": 1.23, "y": 0.45, "yaw": 0.78, "v": 0.20, "w": 0.10,
  "roll": 0.01, "pitch": -0.02, "gz": 0.11,
  "base": { "closed_loop": true, "target_left": 0.20, "meas_left": 0.19,
            "target_right": 0.20, "meas_right": 0.20,
            "left_pwm": 0.63, "right_pwm": 0.64, "timed_out": false },
  "odom_age_ms": 40, "imu_age_ms": 20, "base_age_ms": 60
}
```

## 5.5 前端仪表盘

`web/index.html` + `web/app.js`,纯静态、无框架依赖:

- **方向盘**:按住持续发指令(150ms 一次),松手自动停;支持 WASD/方向键/空格。
- **紧急停止**:单独大按钮，触发后持续锁存；复位按钮与运动按钮分离，锁存期间运动 API 返回 423。
- **位姿 & IMU 面板**:显示 EKF 融合后的 x/y/yaw、线/角速度、roll/pitch/gz、数据新鲜度。
- **闭环面板**(核心可视化):左右轮"目标 vs 编码器实测轮速"进度条对比、PWM 占空比、
  运行模式(闭环 PID / 开环回退)、状态徽章(闭环正常 / 超时停车 / dry-run)。

这个闭环面板直观反映控制质量:当"目标线和实测线基本重合"时,说明 PID 已将轮速稳定控制在指令值附近。

## 5.6 部署方式

- 手动:`ros2 launch raspi_car_web web_control.launch.py`,访问 `http://<pi_ip>:8080`。
- 开机自启:可编写 systemd 服务(如 `raspi-car-web-backend.service`)在启动时拉起,
  或用 cpolar/frp 之类内网穿透把 8080 暴露到公网(注意鉴权,见下)。

## 5.7 安全提醒

当前 Web 后端**无鉴权**,任何能访问 8080 的人都能控车。仅限可信局域网使用。
若要上公网,应加:①登录/Token 鉴权;②HTTPS;③命令频率限制；物理急停仍应独立于网络与软件。
这属于"网络暴露的控制接口"的基本安全考量。

下一章:[06 构建与运行](06_build_run.md)
