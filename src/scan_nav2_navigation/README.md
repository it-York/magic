# Nav2 global planning with SCAN-Planner

This package runs Nav2 only as the global planner. It deliberately does not
start Nav2's controller server or DWB local planner. `nav2_scan_bridge` calls
`ComputePathToPose`, transforms the resulting path from `map` to `odom`, and
publishes it on `/initial_path` for SCAN-Planner navigation mode 3.

Expected inputs:

- `/Odometry` and `/cloud_registered` from FAST-LIO;
- `map -> odom` from `open3d_loc`;
- `/goal_pose` or `/move_base_simple/goal` as a `PoseStamped` goal.

Main outputs:

- `/global_plan`: the Nav2 path in `map`;
- `/initial_path`: the downsampled SCAN reference path in `odom`;
- `/hybrid_navigation/status`: planner bridge state.

Start Livox, FAST-LIO and `open3d_loc` first, then run:

```bash
ros2 launch scan_nav2_navigation hybrid_navigation.launch.py
```

In RViz use `map` as the fixed frame and publish a 2D goal on `/goal_pose`.
The launch file does not start the Go2 Sport API bridge by default. After the
paths and local trajectory have been verified, explicitly enable motion with:

```bash
ros2 launch scan_nav2_navigation hybrid_navigation.launch.py enable_motion:=true
```

## One-command inspection stack

The inspection launch starts the MID360 driver, FAST-LIO, Open3D localization,
Nav2 global planning, SCAN local planning, and RViz in that order. It uses:

- `/map` for the Nav2 2D occupancy grid;
- `/pcd_map` for the Open3D point-cloud map;
- `/cloud_registered` for the live FAST-LIO cloud;
- `/grid_map/occupancy` and `/grid_map/occupancy_inflate` for SCAN's local map;
- `/global_plan`, `/initial_path`, and `/optimal_list` for the planned paths.

Start visualization and planning without hardware motion first:

```bash
ros2 launch scan_nav2_navigation inspection_navigation.launch.py
```

After verifying localization and maps, enable the Go2 Sport bridge:

```bash
ros2 launch scan_nav2_navigation inspection_navigation.launch.py enable_motion:=true
```

The motion bridge rejects commands when odometry is stale, non-finite, jumps
abnormally, leaves the configured indoor bounds, or Open3D confidence is stale
or below its threshold. `Ctrl+C` sends a final Go2 `StopMove` request.
