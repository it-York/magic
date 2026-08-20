cmake_minimum_required(VERSION 3.8)
project(scan_nav2_navigation)

if(NOT CMAKE_CXX_STANDARD)
  set(CMAKE_CXX_STANDARD 17)
endif()
add_compile_options(-Wall -Wextra -Wpedantic)

find_package(ament_cmake REQUIRED)
find_package(geometry_msgs REQUIRED)
find_package(nav2_msgs REQUIRED)
find_package(nav_msgs REQUIRED)
find_package(rclcpp REQUIRED)
find_package(rclcpp_action REQUIRED)
find_package(std_msgs REQUIRED)
find_package(tf2 REQUIRED)
find_package(tf2_geometry_msgs REQUIRED)
find_package(tf2_ros REQUIRED)

add_executable(fastlio_odom_bridge src/fastlio_odom_bridge.cpp)
ament_target_dependencies(fastlio_odom_bridge
  geometry_msgs nav_msgs rclcpp tf2 tf2_geometry_msgs tf2_ros)

add_executable(nav2_scan_bridge src/nav2_scan_bridge.cpp)
ament_target_dependencies(nav2_scan_bridge
  geometry_msgs nav2_msgs nav_msgs rclcpp rclcpp_action std_msgs
  tf2 tf2_geometry_msgs tf2_ros)

install(TARGETS fastlio_odom_bridge nav2_scan_bridge
  DESTINATION lib/${PROJECT_NAME})
install(DIRECTORY config launch
  DESTINATION share/${PROJECT_NAME})
install(FILES README.md
  DESTINATION share/${PROJECT_NAME})

ament_package()
