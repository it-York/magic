#include <cmath>
#include <memory>
#include <stdexcept>
#include <string>
#include <vector>

#include <geometry_msgs/msg/transform_stamped.hpp>
#include <nav_msgs/msg/odometry.hpp>
#include <rclcpp/rclcpp.hpp>
#include <tf2/LinearMath/Quaternion.h>
#include <tf2/LinearMath/Transform.h>
#include <tf2_geometry_msgs/tf2_geometry_msgs.hpp>
#include <tf2_ros/transform_broadcaster.h>

namespace scan_nav2_navigation
{
class FastlioOdomBridge : public rclcpp::Node
{
public:
  FastlioOdomBridge() : Node("fastlio_odom_bridge")
  {
    input_topic_ = declare_parameter<std::string>("input_topic", "/Odometry");
    output_topic_ = declare_parameter<std::string>("output_topic", "/scan_odom");
    odom_frame_ = declare_parameter<std::string>("odom_frame", "odom");
    base_frame_ = declare_parameter<std::string>("base_frame", "base_link");
    publish_tf_ = declare_parameter<bool>("publish_tf", true);
    const auto xyz = declare_parameter<std::vector<double>>(
        "base_T_imu_xyz", {0.0, 0.0, 0.0});
    const auto rpy_deg = declare_parameter<std::vector<double>>(
        "base_T_imu_rpy_deg", {0.0, 45.0, 0.0});

    if (xyz.size() != 3 || rpy_deg.size() != 3)
      throw std::runtime_error("base_T_imu_xyz and base_T_imu_rpy_deg must contain 3 values");

    constexpr double deg_to_rad = M_PI / 180.0;
    tf2::Quaternion rotation;
    rotation.setRPY(
        rpy_deg[0] * deg_to_rad,
        rpy_deg[1] * deg_to_rad,
        rpy_deg[2] * deg_to_rad);
    rotation.normalize();
    base_T_imu_.setOrigin(tf2::Vector3(xyz[0], xyz[1], xyz[2]));
    base_T_imu_.setRotation(rotation);

    odom_pub_ = create_publisher<nav_msgs::msg::Odometry>(output_topic_, 20);
    if (publish_tf_)
      tf_broadcaster_ = std::make_unique<tf2_ros::TransformBroadcaster>(*this);

    odom_sub_ = create_subscription<nav_msgs::msg::Odometry>(
        input_topic_, rclcpp::SensorDataQoS(),
        std::bind(&FastlioOdomBridge::odomCallback, this, std::placeholders::_1));

    RCLCPP_INFO(
        get_logger(), "Bridging %s to %s with frames %s -> %s",
        input_topic_.c_str(), output_topic_.c_str(), odom_frame_.c_str(), base_frame_.c_str());
  }

private:
  void odomCallback(const nav_msgs::msg::Odometry::ConstSharedPtr message)
  {
    tf2::Transform odom_T_imu;
    tf2::fromMsg(message->pose.pose, odom_T_imu);
    const tf2::Transform odom_T_base = odom_T_imu * base_T_imu_.inverse();

    nav_msgs::msg::Odometry output = *message;
    output.header.frame_id = odom_frame_;
    output.child_frame_id = base_frame_;
    output.pose.pose.position.x = odom_T_base.getOrigin().x();
    output.pose.pose.position.y = odom_T_base.getOrigin().y();
    output.pose.pose.position.z = odom_T_base.getOrigin().z();
    output.pose.pose.orientation = tf2::toMsg(odom_T_base.getRotation());
    odom_pub_->publish(output);

    if (!tf_broadcaster_)
      return;

    geometry_msgs::msg::TransformStamped transform;
    transform.header = output.header;
    transform.child_frame_id = base_frame_;
    transform.transform = tf2::toMsg(odom_T_base);
    tf_broadcaster_->sendTransform(transform);
  }

  std::string input_topic_;
  std::string output_topic_;
  std::string odom_frame_;
  std::string base_frame_;
  bool publish_tf_{true};
  tf2::Transform base_T_imu_{tf2::Transform::getIdentity()};
  rclcpp::Publisher<nav_msgs::msg::Odometry>::SharedPtr odom_pub_;
  rclcpp::Subscription<nav_msgs::msg::Odometry>::SharedPtr odom_sub_;
  std::unique_ptr<tf2_ros::TransformBroadcaster> tf_broadcaster_;
};
}  // namespace scan_nav2_navigation

int main(int argc, char ** argv)
{
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<scan_nav2_navigation::FastlioOdomBridge>());
  rclcpp::shutdown();
  return 0;
}
