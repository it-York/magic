import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription
from launch.conditions import IfCondition
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node


def generate_launch_description():
    package_share = get_package_share_directory("scan_nav2_navigation")
    scan_share = get_package_share_directory("scan_planner")

    map_file = LaunchConfiguration("map")
    planner_config = LaunchConfiguration("planner_config")
    use_sim_time = LaunchConfiguration("use_sim_time")

    map_server = Node(
        package="nav2_map_server",
        executable="map_server",
        name="map_server",
        output="screen",
        parameters=[{"yaml_filename": map_file, "use_sim_time": use_sim_time}],
    )
    planner_server = Node(
        package="nav2_planner",
        executable="planner_server",
        name="planner_server",
        output="screen",
        parameters=[planner_config, {"use_sim_time": use_sim_time}],
    )
    lifecycle_manager = Node(
        package="nav2_lifecycle_manager",
        executable="lifecycle_manager",
        name="lifecycle_manager_scan_nav2",
        output="screen",
        parameters=[
            {
                "use_sim_time": use_sim_time,
                "autostart": True,
                "node_names": ["map_server", "planner_server"],
            }
        ],
    )
    odom_bridge = Node(
        package="scan_nav2_navigation",
        executable="fastlio_odom_bridge",
        name="fastlio_odom_bridge",
        output="screen",
        parameters=[
            {
                "use_sim_time": use_sim_time,
                "input_topic": "/Odometry",
                "output_topic": "/scan_odom",
                "odom_frame": "odom",
                "base_frame": "base_link",
                "base_T_imu_xyz": [0.0, 0.0, 0.0],
                "base_T_imu_rpy_deg": [0.0, 45.0, 0.0],
                "publish_tf": True,
            }
        ],
    )
    path_bridge = Node(
        package="scan_nav2_navigation",
        executable="nav2_scan_bridge",
        name="nav2_scan_bridge",
        output="screen",
        parameters=[planner_config, {"use_sim_time": use_sim_time}],
    )
    scan_planner = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(os.path.join(scan_share, "launch", "run.launch.py")),
        condition=IfCondition(LaunchConfiguration("start_scan_planner")),
        launch_arguments={
            "is_real_world": "true",
            "navi_mode": "3",
            "sensor_type": "lidar",
            "controller_mode": "closed_loop",
            "body_pose_topic": "/scan_odom",
            "sensor_pose_topic": "/Odometry",
            "cloud_topic": "/cloud_registered",
            "planning_frame": "odom",
            "start_sport_bridge": LaunchConfiguration("enable_motion"),
            "require_localization": LaunchConfiguration("require_localization"),
            "use_sim_time": use_sim_time,
        }.items(),
    )

    return LaunchDescription(
        [
            DeclareLaunchArgument(
                "map", default_value="/home/magic/ros2_ws/maps/go2_mgc_map.yaml"
            ),
            DeclareLaunchArgument(
                "planner_config",
                default_value=os.path.join(package_share, "config", "nav2_planner.yaml"),
            ),
            DeclareLaunchArgument("start_scan_planner", default_value="true"),
            DeclareLaunchArgument("enable_motion", default_value="false"),
            DeclareLaunchArgument("require_localization", default_value="true"),
            DeclareLaunchArgument("use_sim_time", default_value="false"),
            map_server,
            planner_server,
            lifecycle_manager,
            odom_bridge,
            path_bridge,
            scan_planner,
        ]
    )
