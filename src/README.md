# Go2 室内导航链路

本目录包含 Unitree Go2 室内导航所需的完整 ROS 2 源码。系统使用 Livox
MID360 和 FAST-LIO 获取实时里程计与点云，Open3D 完成全局重定位，Nav2
生成二维全局路径，SCAN-Planner 根据实时三维点云生成局部轨迹并控制机器狗避障。

> 当前真机配置假定 MID360 相对机身向下倾斜 45°。启动文件中的
> `base_link -> imu_link` 和里程计桥接参数均按该安装角度配置。

## 导航链路

```text
MID360
  └─> FAST-LIO ──> /Odometry + /cloud_registered
          ├─> Open3D 重定位 ──> map -> odom + /localization_3d_confidence
          └─> SCAN-Planner 实时局部地图与局部避障

二维栅格地图 /map ──> Nav2 全局规划 ──> /global_plan
                                              └─> Nav2-SCAN 桥接
                                                    └─> /initial_path
                                                          └─> SCAN-Planner
                                                                └─> /cmd_vel
                                                                      └─> Go2 Sport API
```

Nav2 在这里仅负责基于静态二维栅格地图的全局规划，不启动 Nav2 Controller
或 DWB。动态障碍检测和局部绕障由 SCAN-Planner 完成。

## 目录说明

| 目录 | 用途 |
| --- | --- |
| `ws_livox` | Livox MID360 ROS 2 驱动 |
| `ws_fastlio` | FAST-LIO 建图、里程计和实时注册点云 |
| `ws_localization` | Open3D 点云地图重定位 |
| `SCAN-Planner` | 三维局部地图、局部轨迹规划和运动控制 |
| `scan_nav2_navigation` | Nav2 全局规划、Nav2-SCAN 路径桥接及一键启动文件 |
| `unitree_ros2` | Unitree Go2 消息和通信接口 |
| `ws_elevator_lio` | Elevator-LIO 源码，当前一键导航链路不启动该模块 |

## 当前环境与地图

- ROS 2：Humble
- 系统：Ubuntu 22.04，aarch64
- 容器源码目录：`/home/magic/ros2_ws/src`
- 二维 Nav2 地图：`/home/magic/ros2_ws/maps/go2_mgc_map.yaml`
- 三维 Open3D 地图：`/home/magic/ros2_ws/maps/go2_mgc.pcd`
- Open3D CMake 配置：`/usr/lib/aarch64-linux-gnu/cmake/Open3D/Open3DConfig.cmake`

如果地图路径发生变化，可以在启动命令中通过 `nav_map` 和 `pcd_map` 覆盖，
无需修改源码。

## 编译与环境加载

进入已经配置好 ROS 2、DDS 和图形界面的 Docker 容器：

```bash
docker start go2_humble_desktop
docker exec -it go2_humble_desktop bash
```

首次编译或更新源码后，在容器中执行：

```bash
cd /home/magic/ros2_ws
source /opt/ros/humble/setup.bash
colcon build --symlink-install
source install/setup.bash
```

每次打开新的容器终端，至少需要执行：

```bash
source /opt/ros/humble/setup.bash
source /home/magic/ros2_ws/install/setup.bash
```

启动前检查地图文件和关键软件包：

```bash
ls -lh /home/magic/ros2_ws/maps/go2_mgc_map.yaml
ls -lh /home/magic/ros2_ws/maps/go2_mgc.pcd
ros2 pkg prefix livox_ros_driver2
ros2 pkg prefix fast_lio
ros2 pkg prefix open3d_loc
ros2 pkg prefix scan_planner
ros2 pkg prefix scan_nav2_navigation
```

## 一键启动

### 1. 安全验证模式（机器狗不运动）

首次测试必须先使用默认的安全验证模式：

```bash
ros2 launch scan_nav2_navigation inspection_navigation.launch.py
```

该命令会按顺序启动：

1. Livox MID360 驱动；
2. 约 3 秒后启动 FAST-LIO；
3. 约 10 秒后启动 Open3D 重定位；
4. 约 20 秒后启动 Nav2、SCAN-Planner 和 RViz。

此模式会计算全局路径和局部轨迹，也会发布 `/cmd_vel`，但不会启动
`cmd_vel_to_sport`，因此不会向机器狗发送 Sport API 运动指令。

如需显式指定地图：

```bash
ros2 launch scan_nav2_navigation inspection_navigation.launch.py \
  nav_map:=/home/magic/ros2_ws/maps/go2_mgc_map.yaml \
  pcd_map:=/home/magic/ros2_ws/maps/go2_mgc.pcd
```

### 2. 真机运动模式

确认点云、地图、定位、TF 和规划路径均正常后，先按 `Ctrl+C` 关闭安全验证
模式，再执行：

```bash
ros2 launch scan_nav2_navigation inspection_navigation.launch.py \
  enable_motion:=true
```

真机测试时必须满足以下条件：

- 机器狗四周留有安全空间，操作人员手持遥控器并可随时急停；
- `/Odometry` 和 `/cloud_registered` 持续更新；
- `/localization_3d_confidence` 持续更新且不低于 `0.7`；
- RViz 中三维地图与实时点云基本重合；
- `map -> odom -> base_link` TF 连续且方向正确。

运动桥接器会在里程计超时、定位置信度超时或过低、里程计异常跳变、位置
超出安全边界以及 `/cmd_vel` 超时时发送 `StopMove`。

## RViz 操作与显示

一键启动会自动加载 `hybrid_navigation.rviz`，固定坐标系默认为 `map`。

| RViz 显示项 | 话题 | 含义 |
| --- | --- | --- |
| Map | `/map` | Nav2 使用的二维静态栅格地图 |
| Open3D PCD Map | `/pcd_map` | Open3D 三维点云地图 |
| FAST-LIO Live Cloud | `/cloud_registered` | 雷达实时注册点云 |
| SCAN Local Occupancy | `/grid_map/occupancy` | SCAN 从当前点云建立的局部障碍物地图 |
| SCAN Inflated Occupancy | `/grid_map/occupancy_inflate` | 按机器狗安全尺寸膨胀后的局部障碍物地图 |
| Global Planner / Path | `/global_plan` | Nav2 在 `map` 坐标系生成的全局路径 |
| Controller / Local Plan | `/initial_path` | 转换到 `odom` 后送给 SCAN 的参考路径 |
| SCAN Optimized Local Trajectory | `/optimal_list` | SCAN 实际优化得到的局部避障轨迹 |

下发目标点：

1. 等待地图、实时点云及机器狗位置稳定；
2. 在 RViz 顶部选择 **Nav2 Goal** 或 **2D Goal Pose**；
3. 在二维地图的可通行区域点击并拖动，设置目标位置和朝向；
4. 目标会发布到 `/goal_pose`；
5. 依次确认 `/global_plan`、`/initial_path` 和 `/optimal_list` 出现并持续更新。

Nav2-SCAN 桥接器默认每 2 秒重新请求一次全局路径。由于 Nav2 的全局代价
地图只包含静态地图，临时出现的障碍物主要依靠 SCAN 局部轨迹重新规划绕开。

## 运行状态检查

另开一个已经加载工作区环境的容器终端：

```bash
ros2 topic hz /Odometry
ros2 topic hz /cloud_registered
ros2 topic echo /localization_3d_confidence
ros2 topic echo /hybrid_navigation/status
```

`/hybrid_navigation/status` 常见状态：

| 状态 | 含义 |
| --- | --- |
| `WAITING_FOR_GOAL` | 等待 RViz 目标点 |
| `GOAL_RECEIVED` | 已收到目标点 |
| `WAITING_FOR_NAV2` | Nav2 规划服务尚未就绪 |
| `PLANNING` | 正在计算全局路径 |
| `FOLLOWING` | 全局路径已发送给 SCAN |
| `GOAL_REACHED` | 已到达目标点 |
| `PLAN_FAILED` / `PLAN_REJECTED` | Nav2 无法生成或拒绝了路径 |
| `PATH_TRANSFORM_FAILED` | `map` 到 `odom` 的 TF 转换失败 |

还可以检查关键话题是否存在：

```bash
ros2 topic list | grep -E 'pcd_map|cloud_registered|Odometry|global_plan|initial_path|optimal_list|occupancy|cmd_vel'
```

## 常见问题

### RViz 没有自动打开

确认启动参数未关闭 RViz：

```bash
ros2 launch scan_nav2_navigation inspection_navigation.launch.py start_rviz:=true
```

如果终端出现 `cannot open display`，需要检查容器的 `DISPLAY`、X11/远程桌面
和 Docker 图形权限。

### 看不到二维地图或三维点云地图

检查文件存在，并确认话题已发布：

```bash
ros2 topic info /map
ros2 topic info /pcd_map
```

RViz 的 Fixed Frame 应为 `map`。如果 `/pcd_map` 有数据但仍不可见，重点检查
点云消息的 frame、`map` TF 和 RViz 中该显示项是否启用。

### 发布目标后没有路径或机器狗不动

先查看：

```bash
ros2 topic echo /hybrid_navigation/status
ros2 topic echo /cmd_vel
ros2 topic echo /localization_3d_confidence
```

- 没有 `/global_plan`：检查二维地图、目标点是否在可通行区及 Nav2 状态；
- 有 `/global_plan`、没有 `/initial_path`：检查 `map -> odom` TF；
- 有 `/initial_path`、没有 `/optimal_list`：检查 SCAN 节点、实时点云和里程计；
- 有 `/cmd_vel`、真机仍不动：确认启动时设置了 `enable_motion:=true`，并检查
  定位置信度和运动桥接器的安全告警。

### 前方有障碍物但没有绕行

重点观察 `/grid_map/occupancy`、`/grid_map/occupancy_inflate` 和
`/optimal_list` 是否随障碍物实时更新。如果局部地图没有障碍物，检查点云频率、
时间戳和雷达到机身外参；如果局部地图存在障碍物但轨迹穿过障碍物，停止真机
测试，再检查 SCAN 膨胀半径、地图分辨率和规划参数。

### 机器狗附近出现与现场不一致的障碍点

常见原因是旧点云残留、点云与里程计时间不同步、TF/雷达倾角错误，或同时启动
了多套雷达与 FAST-LIO 节点。确认系统中只有一套驱动和里程计链路，并检查：

```bash
ros2 topic hz /cloud_registered
ros2 topic hz /Odometry
ros2 node list
```

## 停止系统

在一键启动终端按 `Ctrl+C`。真机运动桥接器在正常关闭时会发送一次
`StopMove`。如果机器狗行为异常，应优先使用手中遥控器急停，不能只依赖软件
退出。

关闭后可检查是否仍有 ROS 2 节点：

```bash
ros2 node list
```

若不再使用容器，可回到宿主机执行：

```bash
docker stop go2_humble_desktop
```
