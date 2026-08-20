#include <algorithm>
#include <chrono>
#include <cmath>
#include <cstdint>
#include <iomanip>
#include <locale>
#include <memory>
#include <sstream>
#include <stdexcept>
#include <thread>

#include <geometry_msgs/msg/twist.hpp>
#include <nav_msgs/msg/odometry.hpp>
#include <rclcpp/rclcpp.hpp>
#include <std_msgs/msg/float32.hpp>
#include <unitree_api/msg/request.hpp>

namespace scan_planner
{
class CmdVelToSport : public rclcpp::Node
{
public:
  CmdVelToSport() : Node("cmd_vel_to_sport")
  {
    const double publish_rate = declare_parameter<double>("publish_rate", 20.0);
    cmd_timeout_ = declare_parameter<double>("cmd_timeout", 0.3);
    max_vx_ = declare_parameter<double>("max_vx", 0.75);
    max_vy_ = declare_parameter<double>("max_vy", 0.35);
    max_vyaw_ = declare_parameter<double>("max_vyaw", 1.0);
    odom_timeout_ = declare_parameter<double>("odom_timeout", 0.5);
    localization_timeout_ = declare_parameter<double>("localization_timeout", 5.0);
    min_localization_confidence_ =
        declare_parameter<double>("min_localization_confidence", 0.7);
    max_abs_position_ = declare_parameter<double>("max_abs_position", 100.0);
    max_abs_z_ = declare_parameter<double>("max_abs_z", 5.0);
    max_odom_jump_ = declare_parameter<double>("max_odom_jump", 2.0);
    require_localization_ = declare_parameter<bool>("require_localization", false);

    if (publish_rate <= 0.0 || cmd_timeout_ <= 0.0 ||
        max_vx_ <= 0.0 || max_vy_ <= 0.0 || max_vyaw_ <= 0.0 ||
        odom_timeout_ <= 0.0 || localization_timeout_ <= 0.0 ||
        max_abs_position_ <= 0.0 || max_abs_z_ <= 0.0 || max_odom_jump_ <= 0.0)
    {
      throw std::invalid_argument("rate, timeout, position and velocity limits must be positive");
    }

    sport_request_pub_ =
        create_publisher<unitree_api::msg::Request>("api/sport/request", 10);
    cmd_vel_sub_ = create_subscription<geometry_msgs::msg::Twist>(
        "cmd_vel", 10,
        std::bind(&CmdVelToSport::cmdVelCallback, this, std::placeholders::_1));
    odom_sub_ = create_subscription<nav_msgs::msg::Odometry>(
        "odom", rclcpp::SensorDataQoS(),
        std::bind(&CmdVelToSport::odomCallback, this, std::placeholders::_1));
    if (require_localization_)
    {
      localization_sub_ = create_subscription<std_msgs::msg::Float32>(
          "localization_confidence", 10,
          std::bind(&CmdVelToSport::localizationCallback, this, std::placeholders::_1));
    }

    const auto period = std::chrono::duration<double>(1.0 / publish_rate);
    publish_timer_ = create_wall_timer(
        std::chrono::duration_cast<std::chrono::nanoseconds>(period),
        std::bind(&CmdVelToSport::publishCommand, this));

    RCLCPP_INFO(
        get_logger(),
        "Go2 velocity bridge ready (rate %.1f Hz, cmd timeout %.2f s, localization guard %s)",
        publish_rate, cmd_timeout_, require_localization_ ? "enabled" : "disabled");
  }

  void stop()
  {
    publishStop("bridge shutting down");
  }

private:
  static constexpr std::int64_t kMoveApiId = 1008;
  static constexpr std::int64_t kStopMoveApiId = 1003;

  void cmdVelCallback(const geometry_msgs::msg::Twist::ConstSharedPtr msg)
  {
    latest_cmd_.linear.x = std::clamp(msg->linear.x, -max_vx_, max_vx_);
    latest_cmd_.linear.y = std::clamp(msg->linear.y, -max_vy_, max_vy_);
    latest_cmd_.angular.z = std::clamp(msg->angular.z, -max_vyaw_, max_vyaw_);
    last_cmd_time_ = std::chrono::steady_clock::now();
    have_cmd_ = true;
  }

  void odomCallback(const nav_msgs::msg::Odometry::ConstSharedPtr msg)
  {
    const auto now = std::chrono::steady_clock::now();
    const auto &position = msg->pose.pose.position;
    bool valid = std::isfinite(position.x) && std::isfinite(position.y) &&
        std::isfinite(position.z) && std::abs(position.x) <= max_abs_position_ &&
        std::abs(position.y) <= max_abs_position_ && std::abs(position.z) <= max_abs_z_;

    if (valid && have_valid_odom_)
    {
      const double dt = std::chrono::duration<double>(now - last_odom_time_).count();
      const double dx = position.x - last_odom_x_;
      const double dy = position.y - last_odom_y_;
      const double dz = position.z - last_odom_z_;
      if (dt > 0.0 && dt < 1.0 && std::sqrt(dx * dx + dy * dy + dz * dz) > max_odom_jump_)
        valid = false;
    }

    odom_received_ = true;
    odom_valid_ = valid;
    last_odom_time_ = now;
    if (valid)
    {
      have_valid_odom_ = true;
      last_odom_x_ = position.x;
      last_odom_y_ = position.y;
      last_odom_z_ = position.z;
    }
  }

  void localizationCallback(const std_msgs::msg::Float32::ConstSharedPtr msg)
  {
    localization_received_ = true;
    localization_valid_ = std::isfinite(msg->data) &&
        msg->data >= min_localization_confidence_;
    last_localization_time_ = std::chrono::steady_clock::now();
  }

  bool safetyReady(const std::chrono::steady_clock::time_point &now, std::string &reason) const
  {
    if (!odom_received_ || !odom_valid_ ||
        std::chrono::duration<double>(now - last_odom_time_).count() > odom_timeout_)
    {
      reason = "odometry invalid or stale";
      return false;
    }
    if (require_localization_ &&
        (!localization_received_ || !localization_valid_ ||
         std::chrono::duration<double>(now - last_localization_time_).count() >
             localization_timeout_))
    {
      reason = "localization confidence invalid or stale";
      return false;
    }
    return true;
  }

  void publishCommand()
  {
    if (!have_cmd_)
      return;

    const auto now = std::chrono::steady_clock::now();
    std::string safety_reason;
    if (!safetyReady(now, safety_reason))
    {
      if (!stop_sent_)
        publishStop(safety_reason.c_str());
      return;
    }

    const double command_age = std::chrono::duration<double>(now - last_cmd_time_).count();
    if (command_age > cmd_timeout_)
    {
      if (!stop_sent_)
        publishStop("cmd_vel timed out");
      return;
    }

    unitree_api::msg::Request request;
    request.header.identity.api_id = kMoveApiId;
    request.parameter = moveParameter(
        latest_cmd_.linear.x, latest_cmd_.linear.y, latest_cmd_.angular.z);
    sport_request_pub_->publish(request);
    stop_sent_ = false;
  }

  void publishStop(const char *reason)
  {
    unitree_api::msg::Request request;
    request.header.identity.api_id = kStopMoveApiId;
    sport_request_pub_->publish(request);
    stop_sent_ = true;
    RCLCPP_WARN(get_logger(), "%s; sent StopMove to Go2", reason);
  }

  static std::string moveParameter(double vx, double vy, double vyaw)
  {
    std::ostringstream stream;
    stream.imbue(std::locale::classic());
    stream << std::setprecision(8)
           << "{\"x\":" << vx
           << ",\"y\":" << vy
           << ",\"z\":" << vyaw << "}";
    return stream.str();
  }

  rclcpp::Publisher<unitree_api::msg::Request>::SharedPtr sport_request_pub_;
  rclcpp::Subscription<geometry_msgs::msg::Twist>::SharedPtr cmd_vel_sub_;
  rclcpp::Subscription<nav_msgs::msg::Odometry>::SharedPtr odom_sub_;
  rclcpp::Subscription<std_msgs::msg::Float32>::SharedPtr localization_sub_;
  rclcpp::TimerBase::SharedPtr publish_timer_;
  geometry_msgs::msg::Twist latest_cmd_;
  std::chrono::steady_clock::time_point last_cmd_time_;
  bool have_cmd_{false};
  bool stop_sent_{true};
  bool odom_received_{false};
  bool odom_valid_{false};
  bool have_valid_odom_{false};
  bool localization_received_{false};
  bool localization_valid_{false};
  bool require_localization_{false};
  double last_odom_x_{0.0}, last_odom_y_{0.0}, last_odom_z_{0.0};
  std::chrono::steady_clock::time_point last_odom_time_;
  std::chrono::steady_clock::time_point last_localization_time_;
  double cmd_timeout_, odom_timeout_, localization_timeout_;
  double min_localization_confidence_, max_abs_position_, max_abs_z_, max_odom_jump_;
  double max_vx_, max_vy_, max_vyaw_;
};
}  // namespace scan_planner

int main(int argc, char **argv)
{
  rclcpp::init(argc, argv);
  auto node = std::make_shared<scan_planner::CmdVelToSport>();
  std::weak_ptr<scan_planner::CmdVelToSport> weak_node = node;
  node->get_node_base_interface()->get_context()->add_pre_shutdown_callback(
      [weak_node]() {
        if (const auto locked_node = weak_node.lock())
        {
          locked_node->stop();
          std::this_thread::sleep_for(std::chrono::milliseconds(50));
        }
      });
  rclcpp::spin(node);
  rclcpp::shutdown();
  return 0;
}
