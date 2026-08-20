#include <algorithm>
#include <chrono>
#include <cmath>
#include <cstdint>
#include <memory>
#include <string>
#include <vector>

#include <geometry_msgs/msg/pose_stamped.hpp>
#include <geometry_msgs/msg/transform_stamped.hpp>
#include <nav2_msgs/action/compute_path_to_pose.hpp>
#include <nav_msgs/msg/path.hpp>
#include <rclcpp/rclcpp.hpp>
#include <rclcpp_action/rclcpp_action.hpp>
#include <std_msgs/msg/string.hpp>
#include <tf2_geometry_msgs/tf2_geometry_msgs.hpp>
#include <tf2_ros/buffer.h>
#include <tf2_ros/transform_listener.h>

namespace scan_nav2_navigation
{
class Nav2ScanBridge : public rclcpp::Node
{
public:
  using ComputePath = nav2_msgs::action::ComputePathToPose;
  using GoalHandleComputePath = rclcpp_action::ClientGoalHandle<ComputePath>;

  Nav2ScanBridge()
  : Node("nav2_scan_bridge"),
    tf_buffer_(get_clock()),
    tf_listener_(tf_buffer_)
  {
    global_frame_ = declare_parameter<std::string>("global_frame", "map");
    scan_frame_ = declare_parameter<std::string>("scan_frame", "odom");
    robot_frame_ = declare_parameter<std::string>("robot_frame", "base_link");
    planner_id_ = declare_parameter<std::string>("planner_id", "GridBased");
    goal_topic_ = declare_parameter<std::string>("goal_topic", "/goal_pose");
    legacy_goal_topic_ = declare_parameter<std::string>(
        "legacy_goal_topic", "/move_base_simple/goal");
    global_path_topic_ = declare_parameter<std::string>("global_path_topic", "/global_plan");
    scan_path_topic_ = declare_parameter<std::string>("scan_path_topic", "/initial_path");
    replan_period_ = declare_parameter<double>("replan_period", 2.0);
    waypoint_spacing_ = declare_parameter<double>("waypoint_spacing", 0.5);
    goal_tolerance_ = declare_parameter<double>("goal_tolerance", 0.20);

    planner_client_ = rclcpp_action::create_client<ComputePath>(this, "compute_path_to_pose");
    global_path_pub_ = create_publisher<nav_msgs::msg::Path>(global_path_topic_, 10);
    scan_path_pub_ = create_publisher<nav_msgs::msg::Path>(scan_path_topic_, 10);
    status_pub_ = create_publisher<std_msgs::msg::String>(
        "hybrid_navigation/status", rclcpp::QoS(1).reliable().transient_local());

    goal_sub_ = create_subscription<geometry_msgs::msg::PoseStamped>(
        goal_topic_, 10,
        std::bind(&Nav2ScanBridge::goalCallback, this, std::placeholders::_1));
    if (legacy_goal_topic_ != goal_topic_)
    {
      legacy_goal_sub_ = create_subscription<geometry_msgs::msg::PoseStamped>(
          legacy_goal_topic_, 10,
          std::bind(&Nav2ScanBridge::goalCallback, this, std::placeholders::_1));
    }

    timer_ = create_wall_timer(
        std::chrono::milliseconds(200), std::bind(&Nav2ScanBridge::timerCallback, this));
    publishStatus("WAITING_FOR_GOAL");
    RCLCPP_INFO(
        get_logger(), "Nav2-to-SCAN bridge ready: %s path -> %s path",
        global_frame_.c_str(), scan_frame_.c_str());
  }

private:
  void publishStatus(const std::string & status)
  {
    std_msgs::msg::String message;
    message.data = status;
    status_pub_->publish(message);
  }

  bool transformPose(
      const geometry_msgs::msg::PoseStamped & input,
      const std::string & target_frame,
      geometry_msgs::msg::PoseStamped & output)
  {
    try
    {
      const auto transform = tf_buffer_.lookupTransform(
          target_frame, input.header.frame_id, tf2::TimePointZero);
      tf2::doTransform(input, output, transform);
      output.header.frame_id = target_frame;
      return true;
    }
    catch (const tf2::TransformException & error)
    {
      RCLCPP_WARN(get_logger(), "Pose transform failed: %s", error.what());
      return false;
    }
  }

  void goalCallback(const geometry_msgs::msg::PoseStamped::ConstSharedPtr message)
  {
    geometry_msgs::msg::PoseStamped input = *message;
    if (input.header.frame_id.empty())
      input.header.frame_id = global_frame_;

    if (input.header.frame_id == global_frame_)
      goal_ = input;
    else if (!transformPose(input, global_frame_, goal_))
      return;

    goal_.header.stamp = now();
    goal_active_ = true;
    planning_ = false;
    ++goal_sequence_;
    last_plan_request_ = rclcpp::Time(0, 0, get_clock()->get_clock_type());
    publishStatus("GOAL_RECEIVED");
    requestPlan();
  }

  void timerCallback()
  {
    if (!goal_active_)
      return;

    if (goalReached())
    {
      goal_active_ = false;
      publishStatus("GOAL_REACHED");
      RCLCPP_INFO(get_logger(), "Goal reached");
      return;
    }

    if (planning_)
      return;

    if (last_plan_request_.nanoseconds() == 0 ||
        (now() - last_plan_request_).seconds() >= replan_period_)
      requestPlan();
  }

  bool goalReached()
  {
    try
    {
      const auto transform = tf_buffer_.lookupTransform(
          global_frame_, robot_frame_, tf2::TimePointZero);
      const double dx = goal_.pose.position.x - transform.transform.translation.x;
      const double dy = goal_.pose.position.y - transform.transform.translation.y;
      return std::hypot(dx, dy) <= goal_tolerance_;
    }
    catch (const tf2::TransformException & error)
    {
      RCLCPP_WARN_THROTTLE(
          get_logger(), *get_clock(), 2000, "Cannot check goal distance: %s", error.what());
      return false;
    }
  }

  void requestPlan()
  {
    if (!planner_client_->action_server_is_ready())
    {
      RCLCPP_WARN_THROTTLE(
          get_logger(), *get_clock(), 2000, "Waiting for compute_path_to_pose action");
      publishStatus("WAITING_FOR_NAV2");
      return;
    }

    ComputePath::Goal request;
    request.goal = goal_;
    // Replanning may continue longer than the TF cache. Refresh the stamp so
    // Nav2 never tries to transform a goal using its original RViz timestamp.
    request.goal.header.stamp = now();
    request.planner_id = planner_id_;
    request.use_start = false;

    planning_ = true;
    last_plan_request_ = now();
    publishStatus("PLANNING");
    const std::uint64_t request_sequence = goal_sequence_;

    rclcpp_action::Client<ComputePath>::SendGoalOptions options;
    options.goal_response_callback =
        [this, request_sequence](const GoalHandleComputePath::SharedPtr & goal_handle)
        {
          if (request_sequence != goal_sequence_)
            return;
          if (!goal_handle)
          {
            planning_ = false;
            publishStatus("PLAN_REJECTED");
            RCLCPP_ERROR(get_logger(), "Nav2 rejected the global planning request");
          }
        };
    options.result_callback =
        [this, request_sequence](const GoalHandleComputePath::WrappedResult & result)
        {
          if (request_sequence != goal_sequence_ || !goal_active_)
            return;
          planning_ = false;
          if (result.code != rclcpp_action::ResultCode::SUCCEEDED || !result.result)
          {
            publishStatus("PLAN_FAILED");
            RCLCPP_ERROR(get_logger(), "Nav2 global planning failed");
            return;
          }

          if (!publishPaths(result.result->path))
          {
            publishStatus("PATH_TRANSFORM_FAILED");
            return;
          }
          publishStatus("FOLLOWING");
        };
    planner_client_->async_send_goal(request, options);
  }

  bool publishPaths(const nav_msgs::msg::Path & global_path)
  {
    if (global_path.poses.size() < 2)
    {
      RCLCPP_ERROR(get_logger(), "Nav2 returned a path with fewer than two poses");
      return false;
    }

    geometry_msgs::msg::TransformStamped transform;
    try
    {
      transform = tf_buffer_.lookupTransform(
          scan_frame_, global_path.header.frame_id, tf2::TimePointZero);
    }
    catch (const tf2::TransformException & error)
    {
      RCLCPP_ERROR(get_logger(), "Global path transform failed: %s", error.what());
      return false;
    }

    nav_msgs::msg::Path scan_path;
    scan_path.header.stamp = now();
    scan_path.header.frame_id = scan_frame_;
    scan_path.poses.reserve(global_path.poses.size());

    std::vector<geometry_msgs::msg::PoseStamped> transformed;
    transformed.reserve(global_path.poses.size());
    for (const auto & pose : global_path.poses)
    {
      geometry_msgs::msg::PoseStamped output;
      tf2::doTransform(pose, output, transform);
      output.header = scan_path.header;
      transformed.push_back(output);
    }

    scan_path.poses.push_back(transformed.front());
    for (std::size_t index = 1; index + 1 < transformed.size(); ++index)
    {
      const auto & previous = scan_path.poses.back().pose.position;
      const auto & current = transformed[index].pose.position;
      if (std::hypot(current.x - previous.x, current.y - previous.y) >= waypoint_spacing_)
        scan_path.poses.push_back(transformed[index]);
    }

    const auto & previous = scan_path.poses.back().pose.position;
    const auto & final = transformed.back().pose.position;
    if (std::hypot(final.x - previous.x, final.y - previous.y) > 1e-6)
      scan_path.poses.push_back(transformed.back());

    if (scan_path.poses.size() < 2)
    {
      RCLCPP_WARN(get_logger(), "Goal is too close to create a SCAN reference path");
      return false;
    }

    nav_msgs::msg::Path global_output = global_path;
    global_output.header.stamp = now();
    global_path_pub_->publish(global_output);
    scan_path_pub_->publish(scan_path);
    RCLCPP_INFO(
        get_logger(), "Published global path (%zu poses) and SCAN path (%zu waypoints)",
        global_path.poses.size(), scan_path.poses.size());
    return true;
  }

  std::string global_frame_;
  std::string scan_frame_;
  std::string robot_frame_;
  std::string planner_id_;
  std::string goal_topic_;
  std::string legacy_goal_topic_;
  std::string global_path_topic_;
  std::string scan_path_topic_;
  double replan_period_{2.0};
  double waypoint_spacing_{0.5};
  double goal_tolerance_{0.20};
  bool goal_active_{false};
  bool planning_{false};
  std::uint64_t goal_sequence_{0};
  geometry_msgs::msg::PoseStamped goal_;
  rclcpp::Time last_plan_request_{0, 0, RCL_ROS_TIME};

  tf2_ros::Buffer tf_buffer_;
  tf2_ros::TransformListener tf_listener_;
  rclcpp_action::Client<ComputePath>::SharedPtr planner_client_;
  rclcpp::Publisher<nav_msgs::msg::Path>::SharedPtr global_path_pub_;
  rclcpp::Publisher<nav_msgs::msg::Path>::SharedPtr scan_path_pub_;
  rclcpp::Publisher<std_msgs::msg::String>::SharedPtr status_pub_;
  rclcpp::Subscription<geometry_msgs::msg::PoseStamped>::SharedPtr goal_sub_;
  rclcpp::Subscription<geometry_msgs::msg::PoseStamped>::SharedPtr legacy_goal_sub_;
  rclcpp::TimerBase::SharedPtr timer_;
};
}  // namespace scan_nav2_navigation

int main(int argc, char ** argv)
{
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<scan_nav2_navigation::Nav2ScanBridge>());
  rclcpp::shutdown();
  return 0;
}
