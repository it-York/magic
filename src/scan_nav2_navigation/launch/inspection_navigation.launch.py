import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription, TimerAction
from launch.conditions import IfCondition
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node


def generate_launch_description():
    livox_share = get_package_share_directory("livox_ros_driver2")
    fast_lio_share = get_package_share_directory("fast_lio")
    open3d_share = get_package_share_directory("open3d_loc")
    navigation_share = get_package_share_directory("scan_nav2_navigation")

    use_sim_time = LaunchConfiguration("use_sim_time")
    enable_motion = LaunchConfiguration("enable_motion")
    nav_map = LaunchConfiguration("nav_map")
    pcd_map = LaunchConfiguration("pcd_map")

    livox = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(livox_share, "launch_ROS2", "msg_MID360_launch.py")
        )
    )

    fast_lio = TimerAction(
        period=LaunchConfiguration("fast_lio_delay"),
        actions=[
            IncludeLaunchDescription(
                PythonLaunchDescriptionSource(
                    os.path.join(fast_lio_share, "launch", "mapping.launch.py")
                ),
                launch_arguments={
                    "config_file": "mid360.yaml",
                    "rviz": "false",
                    "use_sim_time": use_sim_time,
                }.items(),
            )
        ],
    )

    localization = TimerAction(
        period=LaunchConfiguration("localization_delay"),
        actions=[
            IncludeLaunchDescription(
                PythonLaunchDescriptionSource(
                    os.path.join(open3d_share, "launch", "open3d_loc_g1.launch.py")
                ),
                launch_arguments={
                    "pcd_map": pcd_map,
                    "use_sim_time": use_sim_time,
                }.items(),
            )
        ],
    )

    navigation = TimerAction(
        period=LaunchConfiguration("navigation_delay"),
        actions=[
            IncludeLaunchDescription(
                PythonLaunchDescriptionSource(
                    os.path.join(
                        navigation_share, "launch", "hybrid_navigation.launch.py"
                    )
                ),
                launch_arguments={
                    "map": nav_map,
                    "enable_motion": enable_motion,
                    "require_localization": "true",
                    "use_sim_time": use_sim_time,
                }.items(),
            ),
            Node(
                package="rviz2",
                executable="rviz2",
                name="inspection_navigation_rviz",
                output="screen",
                arguments=[
                    "-d",
                    os.path.join(
                        navigation_share, "config", "hybrid_navigation.rviz"
                    ),
                ],
                condition=IfCondition(LaunchConfiguration("start_rviz")),
            ),
        ],
    )

    return LaunchDescription(
        [
            DeclareLaunchArgument("enable_motion", default_value="false"),
            DeclareLaunchArgument("start_rviz", default_value="true"),
            DeclareLaunchArgument("use_sim_time", default_value="false"),
            DeclareLaunchArgument(
                "nav_map",
                default_value="/home/magic/ros2_ws/maps/go2_mgc_map.yaml",
            ),
            DeclareLaunchArgument(
                "pcd_map",
                default_value="/home/magic/ros2_ws/maps/go2_mgc.pcd",
            ),
            DeclareLaunchArgument("fast_lio_delay", default_value="3.0"),
            DeclareLaunchArgument("localization_delay", default_value="10.0"),
            DeclareLaunchArgument("navigation_delay", default_value="20.0"),
            livox,
            fast_lio,
            localization,
            navigation,
        ]
    )
