# 04 · SLAM 建图与 Nav2 导航

## 4.1 建图:slam_toolbox

**输入**:`/car01/scan`(激光) + TF `odom→base_link`(现由 EKF 提供,准了)。
**输出**:占据栅格地图 + TF `map→odom`。

流程:

```
async_slam_toolbox_node
  ├─ 扫描匹配(scan matching):把当前帧对到已建局部地图,算出 map→odom 修正
  ├─ 位姿图优化:节点=关键帧位姿,边=匹配约束
  └─ 回环检测(loop closure):走回起点时识别并优化,消除累积漂移
```

`base_frame: base_link`(不是 laser_frame)。TF 链
`map→(slam)odom→(EKF)base_link→(静态)laser_frame` 必须解析到 `base_link`。
若里程计漂移,scan matching 拿不到好初值,建图就会重影;
本项目用编码器+EKF 保证 `odom→base_link` 平滑,匹配收敛、回环能闭合。

建图操作:

```bash
ros2 launch raspi_car_bringup mapping.launch.py
# 用网页/手柄把整个场地走一圈(尽量回到起点触发回环)
ros2 run nav2_map_server map_saver_cli -f <path>/room1 --ros-args -r __ns:=/car01
# 生成 room1.pgm + room1.yaml
```

## 4.2 地图文件

`room1.yaml`(项目已内置一张真实建好的图):

```yaml
image: room1.pgm
resolution: 0.05        # 每像素 5cm
origin: [-7.23, -2.43, 0]
occupied_thresh: 0.65
free_thresh: 0.25
```

## 4.3 定位:AMCL(自适应蒙特卡洛定位)

导航阶段用已知地图 + 激光做全局定位,发布 `map→odom`(与建图互斥,不同时跑)。

- 粒子滤波:每个粒子是一个候选位姿,按"激光与地图的吻合度"重加权、重采样。
- `robot_model_type: DifferentialMotionModel`:差速运动模型。
- `set_initial_pose: true` + `initial_pose`:开机自动初始化到已知起点,巡逻无需人工点初值。
- 编码器+EKF 让 `odom` 系运动预测更准,粒子不易发散,定位更稳。

## 4.4 规划:Smac Hybrid-A*(全局)

**为什么不用默认的 NavFn/Dijkstra?** 因为本车**不能原地转向**。
NavFn 生成的是"栅格最短路径",不保证运动学可行,车可能被要求原地掉头。

Smac Hybrid-A* 在 `(x, y, θ)` 三维状态空间搜索,用 **Dubins 曲线**运动基元:

```
motion_model_for_search: "DUBIN"      # 只前进的最短曲线(不含倒车)
minimum_turning_radius: 0.45          # 最小转弯半径,匹配车辆实际能力
angle_quantization_bins: 72           # 航向离散成 72 份(5°)
```

产出的路径**每一段都是车能实际走出来的弧线**,不会出现原地转。

## 4.5 控制:Regulated Pure Pursuit(局部)

沿全局路径选一个前视点(lookahead),算出跟踪它所需的弧线速度指令:

```
use_rotate_to_heading: false          # 关键:禁止"先转到朝向"——车转不了
allow_reversing: false                # 不倒车
desired_linear_vel: 0.22
use_regulated_linear_velocity_scaling: true   # 转弯/近障时自动降速
```

"Regulated"= 在小半径转弯、接近障碍时按规则降低线速度,更安全平顺。

## 4.6 安全:Collision Monitor

在控制指令送到底盘**之前**再加一道独立的激光安全层:

```
cmd_vel_in:  cmd_vel_raw   (controller 输出)
cmd_vel_out: cmd_vel_nav   (送去 mux)
PolygonStop  半径0.30 → 停
PolygonSlow  半径0.55 → 降速到 40%
```

它不信任规划器,直接看原始激光:有人/物突然靠近就地减速或急停。

## 4.7 恢复行为(Behavior Server)

导航卡住时的兜底动作。默认行为树需要 `spin`,所以插件列表保留了它,
但**本车正常导航永不原地转**(RPP 禁转 + Smac 最小半径)。真机调优阶段
建议换成"无 spin"的自定义行为树,恢复只用 `backup`(后退)+ `wait`。

## 4.8 任务层

- **`patrol_node`**:读 `patrol_waypoints.yaml`,调 Nav2 `FollowWaypoints` 循环巡逻。
  依赖 AMCL 自动初始化,无人值守。
- **`wall_follower_node`**:**不依赖地图/定位/里程计**的纯激光反应式沿墙巡逻。
  作为定位失效时的降级方案,免疫打滑。所有转向都是弧线(适配不能原地转)。

## 4.9 完整导航数据流

```
scan ─┬─► slam(建图) / amcl(定位) ─► map→odom(TF)
      ├─► global_costmap ─► planner(Smac) ─► path
      ├─► local_costmap ──► controller(RPP) ─► cmd_vel_raw
      └─► collision_monitor ─► cmd_vel_nav ─► mux ─► cmd_vel ─► PID电机
odom(EKF) ─► amcl 运动预测 / bt_navigator 里程 / controller
```

下一章:[05 Web 控制](05_web_control.md)
