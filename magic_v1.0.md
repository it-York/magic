# Go2 室内巡检导航系统技术文档

## 1. 文档信息

| 项目 | 内容 |
| --- | --- |
| 文档日期 | 2026-08-20 |
| 机器人平台 | Unitree Go2 |
| 激光雷达 | Livox MID360 |
| 软件环境 | Ubuntu 22.04 / ROS 2 Humble / aarch64 / Docker |
| 宿主机工作空间 | `/home/unitree/go2_ros2_ws` |
| 容器工作空间 | `/home/magic/ros2_ws` |
| 一键启动入口 | `scan_nav2_navigation/launch/inspection_navigation.launch.py` |

本文档描述当前源码中的室内导航实现，包括系统架构、数据接口、坐标系、全局
与局部规划方法、控制安全机制以及真机验收方法。

上一级目录原有的 `NAVIGATION_TECHNICAL_HANDOFF.md` 描述的是已经删除的
`nav_magic + Nav2 DWB` 方案，不代表当前实现。当前导航方案为：

```text
Nav2 NavFn/A* 全局规划 + SCAN-Planner Mode 3 局部规划与避障
```

## 2. 设计目标与边界

系统面向已知室内环境中的机器狗巡检任务，实现：

- 基于已有 PCD 地图进行全局重定位；
- 基于二维占据栅格地图生成全局可行路径；
- 根据 MID360 实时点云生成三维局部占据地图；
- 对全局参考路径进行局部轨迹优化和动态避障；
- 将局部轨迹转换为 Go2 机身速度指令；
- 在定位、里程计或控制数据异常时停止运动。

当前系统不包含任务层的巡检点管理、任务调度、自动充电、跨楼层调度和完整的
行为树恢复逻辑。这些功能应在导航链路稳定后由上层任务节点实现。

## 3. 系统总体架构

```text
Livox MID360 驱动
        │ 点云、IMU
        ▼
FAST-LIO
        ├──────────────> /cloud_registered ──────────────┐
        │                                                │
        └──────────────> /Odometry                       │
                              │                          │
              ┌───────────────┴───────────────┐          │
              ▼                               ▼          ▼
      Open3D 全局重定位                FAST-LIO 里程计桥    SCAN 三维局部地图
      PCD map ↔ live cloud             /scan_odom          occupancy/inflation
              │                               │          │
              └─> map -> odom                 └────┬─────┘
                  定位置信度                        │
                                                  ▼
2D map -> Nav2 NavFn/A* -> /global_plan -> Nav2-SCAN 路径桥
                                                  │ /initial_path
                                                  ▼
                                       SCAN-Planner Mode 3
                                                  │ B-spline 局部轨迹
                                                  ▼
                                           闭环轨迹控制器
                                                  │ /cmd_vel
                                                  ▼
                                       定位与里程计安全检查
                                                  │ Sport API
                                                  ▼
                                              Unitree Go2
```

## 4. 软件模块

| 模块 | 源码目录 | 主要作用 |
| --- | --- | --- |
| Livox ROS 2 Driver | `src/ws_livox` | 发布 MID360 点云与 IMU |
| FAST-LIO | `src/ws_fastlio` | 激光惯性里程计和注册点云 |
| Open3D Localization | `src/ws_localization` | PCD 地图匹配、定位置信度和 `map -> odom` |
| Nav2-SCAN Navigation | `src/scan_nav2_navigation` | Nav2 全局规划、路径坐标转换和一键编排 |
| SCAN-Planner | `src/SCAN-Planner` | 三维滑动局部地图、局部轨迹规划和闭环控制 |
| Unitree ROS 2 | `src/unitree_ros2` | Go2 消息定义和 DDS/Sport API 通信 |

`ws_elevator_lio` 当前不在 `inspection_navigation.launch.py` 的启动链路中。

## 5. 坐标系设计

### 5.1 主 TF 链

```text
map -> odom -> base_link -> imu_link
```

| 坐标系 | 定义与提供者 |
| --- | --- |
| `map` | 静态地图全局坐标系，由 Open3D 定位建立与 `odom` 的关系 |
| `odom` | FAST-LIO 连续局部坐标系 |
| `base_link` | 机器狗机身控制参考坐标系，由 `fastlio_odom_bridge` 发布 |
| `imu_link` | MID360/IMU 安装坐标系，由静态 TF 发布 |

### 5.2 雷达安装外参

当前源码假设 MID360 相对机身绕 Y 轴向下倾斜 45°：

```text
roll = 0°, pitch = 45°, yaw = 0°
translation = [0, 0, 0] m
```

相关配置位于：

- `src/ws_localization/src/open3d_loc/launch/open3d_loc_g1.launch.py`
- `src/scan_nav2_navigation/launch/hybrid_navigation.launch.py`
- `src/scan_nav2_navigation/src/fastlio_odom_bridge.cpp`

`fastlio_odom_bridge` 将输入的 `odom_T_imu` 按以下关系转换为机身位姿：

```text
odom_T_base = odom_T_imu × inverse(base_T_imu)
```

静态 TF 与里程计桥中的旋转必须保持一致。当前平移设置为零，意味着系统忽略
雷达与机身中心之间的杆臂。如果安装平移不可忽略，应通过实测标定后同时更新，
不能直接使用估计值。

## 6. 地图与重定位

### 6.1 地图资源

| 地图 | 默认路径 | 用途 |
| --- | --- | --- |
| 二维地图 | `/home/magic/ros2_ws/maps/go2_mgc_map.yaml` | Nav2 静态全局规划 |
| 三维地图 | `/home/magic/ros2_ws/maps/go2_mgc.pcd` | Open3D 点云重定位 |

二维地图与三维地图必须来自相同建图坐标基准。若单独旋转、裁剪或移动其中一个
地图，会出现定位点云看似正确但全局路径整体偏移的问题。

### 6.2 Open3D 定位流程

Open3D 定位节点订阅：

- `/Odometry`：FAST-LIO 位姿；
- `/cloud_registered`：实时注册点云；
- `/initialpose`：可选的人工初始位姿。

主要输出：

- `/pcd_map`：用于显示的静态 PCD 地图；
- `/localization_3d`：当前三维定位结果；
- `/localization_3d_confidence`：匹配置信度；
- `/localization_3d_delay_ms`：定位计算延迟；
- TF `map -> odom`。

当前启动参数中定位频率为 `2.5 Hz`，初始及持续定位匹配阈值为 `0.5`；运动
安全桥使用更严格的 `0.7` 作为允许运动阈值。

## 7. 全局规划

### 7.1 Nav2 配置

当前只启动以下 Nav2 组件：

- `map_server`；
- `planner_server`；
- 管理上述节点的 `lifecycle_manager`。

未启动 Nav2 的 `controller_server`、DWB、行为服务器和 BT Navigator。

全局规划器使用 `nav2_navfn_planner/NavfnPlanner`，配置为：

| 参数 | 当前值 |
| --- | --- |
| 算法 | NavFn，`use_astar: true` |
| 地图分辨率 | `0.05 m` |
| 规划容差 | `0.25 m` |
| 允许未知区域 | `false` |
| 机器人轮廓 | `0.64 m × 0.40 m` 矩形 |
| 轮廓附加余量 | `0.03 m` |
| 全局膨胀半径 | `0.45 m` |
| 期望规划频率 | `1 Hz` |

### 7.2 Nav2-SCAN 路径桥

`nav2_scan_bridge` 接收 RViz 发布到 `/goal_pose` 的目标，通过
`ComputePathToPose` 请求 Nav2 路径，然后执行：

1. 发布 `map` 坐标系的原始路径到 `/global_plan`；
2. 使用 TF 将路径转换到 SCAN 使用的 `odom` 坐标系；
3. 以 `0.5 m` 默认间距降采样；
4. 发布参考路径到 `/initial_path`；
5. 目标未到达时每 `2 s` 重新请求全局路径；
6. 距目标平面距离小于 `0.20 m` 时发布 `GOAL_REACHED`。

桥接状态通过 `/hybrid_navigation/status` 发布。状态包括
`WAITING_FOR_GOAL`、`PLANNING`、`FOLLOWING`、`GOAL_REACHED`、
`PLAN_FAILED` 和 `PATH_TRANSFORM_FAILED`。

## 8. SCAN-Planner 局部规划

### 8.1 Mode 3 接口

一键导航使用 SCAN 的 `navi_mode=3`（REFERENCE_PATH）：

| 输入 | 当前话题 | 作用 |
| --- | --- | --- |
| 机身里程计 | `/scan_odom` | 局部规划与闭环控制位姿 |
| 传感器位姿 | `/Odometry` | 点云投影与占据地图更新 |
| 实时点云 | `/cloud_registered` | 三维障碍观测 |
| 全局参考路径 | `/initial_path` | SCAN 当前导航走廊与局部目标 |

规划坐标系为 `odom`。SCAN 收到新参考路径后，不直接沿折线路径运动，而是从
参考路径选取局部目标，结合当前三维占据地图重新生成局部 B-spline 轨迹。

### 8.2 三维滑动占据地图

当前关键参数：

| 参数 | 当前值 |
| --- | --- |
| 体素分辨率 | `0.05 m` |
| 滑动地图尺寸 | `10 × 10 × 5 m` |
| 局部更新范围 | `5 × 5 × 2.5 m` |
| 最大射线长度 | `5.0 m` |
| 水平双圆柱膨胀半径 | `0.25 m` |
| 双圆柱中心偏移 | `0.18 m` |
| 机身高度 | `0.4 m` |
| Z 向上/向下膨胀 | 各 `0.1 m` |
| 占据阈值概率 | `0.80` |

主要可视化输出：

- `/grid_map/occupancy`：原始局部占据体素；
- `/grid_map/occupancy_inflate`：按机身安全空间膨胀后的体素；
- `/optimal_list`：优化后的局部轨迹；
- `/planning/bspline`：控制器使用的 B-spline 轨迹。

### 8.3 局部规划约束

| 参数 | 当前值 |
| --- | --- |
| 规划视距 | `3.5 m` |
| 最大速度 | `0.75 m/s` |
| 最大加速度 | `0.5 m/s²` |
| 最大加加速度 | `4.0 m/s³` |
| 控制点间距 | `0.2 m` |
| 障碍距离代价阈值 | `0.2 m` |
| B-spline 阶数 | `3` |
| 碰撞检查周期 | `50 ms` |

当当前轨迹失效或可能碰撞时，FSM 会从当前轨迹状态重新规划。Mode 3 优先沿
Nav2 参考路径推进，但局部轨迹由 SCAN 根据实时障碍独立优化。

## 9. 闭环控制与运动接口

`closed_loop_controller` 订阅 `/planning/bspline` 和 `/scan_odom`，以 100 Hz
定时器计算速度指令并发布 `/cmd_vel`。控制过程包含位置误差反馈、期望航向
估计和机身坐标系速度转换。

| 控制参数 | 当前值 |
| --- | --- |
| 轨迹前视时间 | `0.8 s` |
| 航向误差原地转向阈值 | `0.8 rad` |
| 位置比例增益 | `0.8` |
| 航向比例增益 | `1.5` |
| 最大纵向速度 | `0.75 m/s` |
| 最大横向速度 | `0.35 m/s` |
| 最大角速度 | `1.0 rad/s` |
| 局部轨迹完成距离 | `0.15 m` |

当航向误差大于阈值时，控制器冻结轨迹执行时间并优先原地转向，避免机器狗在
朝向严重偏离时继续平移。

`cmd_vel_to_sport` 将 `/cmd_vel` 限幅后转换为 Unitree Sport API：

- Move API ID：`1008`；
- StopMove API ID：`1003`；
- 输出话题：`/api/sport/request`；
- 下发频率：`20 Hz`。

## 10. 安全机制

只有以 `enable_motion:=true` 启动时才创建 `cmd_vel_to_sport`。默认启动模式
只计算和显示轨迹，不向机器狗发送运动请求。

真机运动桥包含以下保护：

| 检查项 | 当前阈值或行为 |
| --- | --- |
| `/cmd_vel` 超时 | `0.3 s`，发送 StopMove |
| `/scan_odom` 超时 | `0.5 s`，发送 StopMove |
| 定位置信度超时 | `5.0 s`，发送 StopMove |
| 最低定位置信度 | `0.7` |
| 单次里程计跳变 | `<1 s` 内超过 `2.0 m` 判为异常 |
| 位置范围 | `|x|, |y| <= 100 m`，`|z| <= 5 m` |
| 速度限幅 | `vx 0.75 m/s`、`vy 0.35 m/s`、`wz 1.0 rad/s` |
| 进程退出 | ROS 关闭回调发送 StopMove |

软件保护不能替代遥控器急停。真机验证时，操作员必须始终处于可观察机器狗和
障碍物的位置，并保持遥控器可立即接管。

## 11. 一键启动时序

```bash
ros2 launch scan_nav2_navigation inspection_navigation.launch.py
```

默认时序：

| 时间 | 动作 |
| --- | --- |
| `T+0 s` | 启动 Livox MID360 驱动 |
| `T+3 s` | 启动 FAST-LIO |
| `T+10 s` | 启动 Open3D 重定位 |
| `T+20 s` | 启动 Nav2、SCAN-Planner 和 RViz |

真机运动模式：

```bash
ros2 launch scan_nav2_navigation inspection_navigation.launch.py \
  enable_motion:=true
```

可覆盖的主要启动参数：

| 参数 | 默认值 | 说明 |
| --- | --- | --- |
| `enable_motion` | `false` | 是否下发 Go2 Sport API |
| `start_rviz` | `true` | 是否启动预配置 RViz |
| `use_sim_time` | `false` | 是否使用仿真时间 |
| `nav_map` | `go2_mgc_map.yaml` | 二维地图路径 |
| `pcd_map` | `go2_mgc.pcd` | 三维地图路径 |
| `fast_lio_delay` | `3.0` | FAST-LIO 启动延迟 |
| `localization_delay` | `10.0` | 定位启动延迟 |
| `navigation_delay` | `20.0` | 规划与 RViz 启动延迟 |

## 12. 关键 ROS 2 接口

| 话题 | 类型 | 生产者 | 消费者 |
| --- | --- | --- | --- |
| `/Odometry` | `nav_msgs/Odometry` | FAST-LIO | Open3D、里程计桥、SCAN 传感器位姿 |
| `/scan_odom` | `nav_msgs/Odometry` | `fastlio_odom_bridge` | SCAN、闭环控制、安全桥 |
| `/cloud_registered` | `sensor_msgs/PointCloud2` | FAST-LIO | Open3D、SCAN、RViz |
| `/pcd_map` | `sensor_msgs/PointCloud2` | Open3D | RViz |
| `/map` | `nav_msgs/OccupancyGrid` | Nav2 map server | Nav2 planner、RViz |
| `/localization_3d_confidence` | `std_msgs/Float32` | Open3D | 运动安全桥 |
| `/goal_pose` | `geometry_msgs/PoseStamped` | RViz | Nav2-SCAN 路径桥 |
| `/global_plan` | `nav_msgs/Path` | Nav2-SCAN 路径桥 | RViz |
| `/initial_path` | `nav_msgs/Path` | Nav2-SCAN 路径桥 | SCAN Mode 3、RViz |
| `/grid_map/occupancy` | `sensor_msgs/PointCloud2` | SCAN | RViz |
| `/grid_map/occupancy_inflate` | `sensor_msgs/PointCloud2` | SCAN | 轨迹规划、RViz |
| `/planning/bspline` | `scan_planner_msgs/Bspline` | SCAN | 闭环控制器 |
| `/cmd_vel` | `geometry_msgs/Twist` | 闭环控制器 | Sport API 桥 |
| `/api/sport/request` | `unitree_api/Request` | Sport API 桥 | Go2 |

## 13. RViz 观测判据

RViz 固定坐标系应为 `map`。一次正常导航应按顺序观察到：

1. `/map` 二维栅格地图；
2. `/pcd_map` 三维静态地图；
3. `/cloud_registered` 实时点云与三维地图基本重合；
4. `/grid_map/occupancy` 随实时点云更新；
5. `/grid_map/occupancy_inflate` 在障碍物周围形成安全膨胀区；
6. 发布目标后出现 `/global_plan`；
7. 随后出现 `/initial_path`；
8. `/optimal_list` 根据局部障碍持续更新；
9. 真机运动模式下 `/cmd_vel` 与实际运动方向一致。

如果只出现 `/global_plan`，不能证明局部避障已工作。必须同时确认局部占据地图
和 `/optimal_list` 正常更新。

## 14. 真机验证方案

### 14.1 阶段 A：静态数据链验证

使用默认 `enable_motion:=false`：

- 检查 `/Odometry`、`/scan_odom` 和 `/cloud_registered` 持续更新；
- 检查 `map -> odom -> base_link` 连通；
- 检查定位置信度稳定高于 `0.7`；
- 检查 PCD 地图、实时点云和二维地图相互对齐；
- 发布目标并确认三段路径全部出现。

### 14.2 阶段 B：静态障碍局部规划验证

仍保持 `enable_motion:=false`，在机器狗前方安全距离放置明显障碍物：

- 确认障碍物出现在 `/grid_map/occupancy`；
- 确认膨胀层覆盖障碍物及其周边；
- 确认 `/optimal_list` 不穿越膨胀层；
- 移动障碍物，确认局部地图和轨迹能够更新。

### 14.3 阶段 C：低风险运动验证

启用真机运动前，应先在配置文件中将速度限制调整到适合现场测试的低速值，
重新编译并重复阶段 A、B。随后：

- 选择宽阔、短距离、无遮挡目标；
- 验证起步、转向、横移和停止方向正确；
- 验证到达目标后速度归零；
- 使用固定软障碍验证绕行；
- 验证路径被阻断时机器狗能够停止，而不是持续顶向障碍物。

### 14.4 阶段 D：失效保护验证

在可控条件下分别验证：

- 停止点云输入；
- 停止里程计输入；
- 使定位置信度低于阈值；
- 停止 `/cmd_vel`；
- 在启动终端按 `Ctrl+C`。

每项测试都应确认 `/api/sport/request` 收到 StopMove，且机器狗实际停止。

## 15. 故障定位矩阵

| 现象 | 首要检查 | 可能原因 |
| --- | --- | --- |
| RViz 无窗口 | `start_rviz`、`DISPLAY` | 容器图形权限或远程桌面问题 |
| 无 `/pcd_map` | PCD 路径、Open3D 日志 | 地图不存在、加载失败 |
| 实时点云与地图偏移 | TF、45° 外参、定位置信度 | 外参错误、定位失败、地图坐标不一致 |
| 收到目标但无全局路径 | `/hybrid_navigation/status` | 目标在障碍区、Nav2 未激活、静态地图不连通 |
| 有全局路径但无参考路径 | `map -> odom` | TF 缺失或时间异常 |
| 有参考路径但无局部轨迹 | SCAN 日志、局部占据图 | 无里程计、点云异常、局部目标不可达 |
| 局部地图出现幽灵障碍 | 点云时间戳、TF、节点数量 | 数据延迟、外参错误、重复启动数据源 |
| 有 `/cmd_vel` 但机器狗不动 | `enable_motion`、安全桥日志 | 未启用运动、定位/里程计保护触发 |
| 机器狗碰撞障碍物 | 占据图、膨胀图、轨迹、延迟 | 障碍未入图、膨胀不足、轨迹未刷新、控制延迟 |

## 16. 当前限制与后续建议

1. Nav2 全局代价地图当前只有静态层和膨胀层，实时障碍不会写入 Nav2 全局
   地图；临时障碍主要依赖 SCAN 局部避障。
2. 当前 45° 外参平移为零，应完成雷达到 `base_link` 的六自由度标定。
3. 当前最大速度 `0.75 m/s` 对初期室内测试偏快，正式验收前应基于制动距离、
   点云端到端延迟和通道宽度重新确定限速。
4. 应记录点云、里程计、定位、局部地图、轨迹、控制指令和 Sport API 的 rosbag，
   用于碰撞或不绕障问题的离线复现。
5. 应增加导航运行健康监控，统一发布点云新鲜度、局部规划成功率、最小障碍
   距离和 StopMove 原因。
6. 巡检任务层应在导航稳定后增加航点状态机、超时处理、任务取消和人工接管。

## 17. 关联文件

- 操作说明：`src/README.md`
- 一键启动：`src/scan_nav2_navigation/launch/inspection_navigation.launch.py`
- 混合导航：`src/scan_nav2_navigation/launch/hybrid_navigation.launch.py`
- Nav2 参数：`src/scan_nav2_navigation/config/nav2_planner.yaml`
- RViz 配置：`src/scan_nav2_navigation/config/hybrid_navigation.rviz`
- SCAN 参数：`src/SCAN-Planner/src/planner/plan_manage/config/planner.yaml`
- 控制参数：`src/SCAN-Planner/src/planner/plan_manage/config/controllers.yaml`
- Open3D 启动：`src/ws_localization/src/open3d_loc/launch/open3d_loc_g1.launch.py`

停止系统时，在一键启动终端按 `Ctrl+C`。出现异常运动时应优先使用遥控器
急停，不能等待软件节点自行恢复。
