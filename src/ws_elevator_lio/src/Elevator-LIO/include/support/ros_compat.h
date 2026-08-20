/*
 * Copyright (c) 2026 xiaofan
 * SPDX-License-Identifier: GPL-2.0-or-later
 *
 * Small ROS 1 / ROS 2 compatibility layer.  The estimation code only depends
 * on the aliases and communication primitives declared here; middleware
 * differences stay at the edge of the project.
 */

#ifndef LIO_SUPPORT_ROS_COMPAT_H
#define LIO_SUPPORT_ROS_COMPAT_H

#ifndef LIO_ROS_VERSION
#error "LIO_ROS_VERSION must be defined by CMake (1 or 2)"
#endif

#include <chrono>
#include <cmath>
#include <cstdint>
#include <functional>
#include <memory>
#include <sstream>
#include <stdexcept>
#include <string>
#include <utility>

#if LIO_ROS_VERSION == 1

#include <ros/ros.h>
#include <ros/package.h>
#include <geometry_msgs/Pose.h>
#include <geometry_msgs/PoseStamped.h>
#include <geometry_msgs/TransformStamped.h>
#include <nav_msgs/Odometry.h>
#include <nav_msgs/Path.h>
#include <sensor_msgs/Imu.h>
#include <sensor_msgs/PointCloud2.h>
#include <sensor_msgs/PointField.h>
#include <std_msgs/Bool.h>
#include <lio/CustomMsg.h>
#include <lio/ElevatorState.h>
#include <lio/wheel_info.h>

#elif LIO_ROS_VERSION == 2

#include <rclcpp/rclcpp.hpp>
#include <rclcpp/create_timer.hpp>
#include <ament_index_cpp/get_package_share_directory.hpp>
#include <builtin_interfaces/msg/time.hpp>
#include <geometry_msgs/msg/pose.hpp>
#include <geometry_msgs/msg/pose_stamped.hpp>
#include <geometry_msgs/msg/transform_stamped.hpp>
#include <nav_msgs/msg/odometry.hpp>
#include <nav_msgs/msg/path.hpp>
#include <sensor_msgs/msg/imu.hpp>
#include <sensor_msgs/msg/point_cloud2.hpp>
#include <sensor_msgs/msg/point_field.hpp>
#include <std_msgs/msg/bool.hpp>
#include <livox_ros_driver2/msg/custom_msg.hpp>
#include <lio/msg/elevator_state.hpp>
#include <lio/msg/wheel_info.hpp>

#else
#error "LIO_ROS_VERSION must be either 1 or 2"
#endif

namespace lio_ros {

inline std::string package_share_directory(const std::string& package_name) {
#if LIO_ROS_VERSION == 1
    const std::string path = ros::package::getPath(package_name);
    if (path.empty()) {
        throw std::runtime_error("ROS package not found: " + package_name);
    }
    return path;
#else
    return ament_index_cpp::get_package_share_directory(package_name);
#endif
}

#if LIO_ROS_VERSION == 1

using Time = ros::Time;
using Imu = sensor_msgs::Imu;
using ImuPtr = sensor_msgs::ImuPtr;
using ImuConstPtr = sensor_msgs::ImuConstPtr;
using PointCloud2 = sensor_msgs::PointCloud2;
using PointCloud2ConstPtr = sensor_msgs::PointCloud2ConstPtr;
using PointField = sensor_msgs::PointField;
using Odometry = nav_msgs::Odometry;
using Path = nav_msgs::Path;
using Pose = geometry_msgs::Pose;
using PoseStamped = geometry_msgs::PoseStamped;
using TransformStamped = geometry_msgs::TransformStamped;
using Bool = std_msgs::Bool;
using BoolConstPtr = std_msgs::BoolConstPtr;
using CustomMsg = lio::CustomMsg;
using CustomMsgConstPtr = lio::CustomMsgConstPtr;
using WheelInfo = lio::wheel_info;
using WheelInfoConstPtr = lio::wheel_infoConstPtr;
using ElevatorState = lio::ElevatorState;
using ElevatorStateConstPtr = lio::ElevatorStateConstPtr;

#else

using Time = builtin_interfaces::msg::Time;
using Imu = sensor_msgs::msg::Imu;
using ImuPtr = Imu::SharedPtr;
using ImuConstPtr = Imu::ConstSharedPtr;
using PointCloud2 = sensor_msgs::msg::PointCloud2;
using PointCloud2ConstPtr = PointCloud2::ConstSharedPtr;
using PointField = sensor_msgs::msg::PointField;
using Odometry = nav_msgs::msg::Odometry;
using Path = nav_msgs::msg::Path;
using Pose = geometry_msgs::msg::Pose;
using PoseStamped = geometry_msgs::msg::PoseStamped;
using TransformStamped = geometry_msgs::msg::TransformStamped;
using Bool = std_msgs::msg::Bool;
using BoolConstPtr = Bool::ConstSharedPtr;
using CustomMsg = livox_ros_driver2::msg::CustomMsg;
using CustomMsgConstPtr = CustomMsg::ConstSharedPtr;
using WheelInfo = lio::msg::WheelInfo;
using WheelInfoConstPtr = WheelInfo::ConstSharedPtr;
using ElevatorState = lio::msg::ElevatorState;
using ElevatorStateConstPtr = ElevatorState::ConstSharedPtr;

#endif

inline double time_to_seconds(const Time& time) {
#if LIO_ROS_VERSION == 1
    return time.toSec();
#else
    return static_cast<double>(time.sec) + static_cast<double>(time.nanosec) * 1e-9;
#endif
}

inline std::uint64_t time_to_nanoseconds(const Time& time) {
#if LIO_ROS_VERSION == 1
    return time.toNSec();
#else
    return static_cast<std::uint64_t>(time.sec) * 1000000000ULL + time.nanosec;
#endif
}

inline Time time_from_seconds(double timestamp) {
    const auto sec = static_cast<std::int32_t>(std::floor(timestamp));
    const auto nanosec = static_cast<std::uint32_t>(
        (timestamp - std::floor(timestamp)) * 1e9);
#if LIO_ROS_VERSION == 1
    return ros::Time(static_cast<std::uint32_t>(sec), nanosec);
#else
    Time result;
    result.sec = sec;
    result.nanosec = nanosec;
    return result;
#endif
}

inline double steady_time_seconds() {
#if LIO_ROS_VERSION == 1
    return ros::WallTime::now().toSec();
#else
    using Clock = std::chrono::steady_clock;
    return std::chrono::duration<double>(Clock::now().time_since_epoch()).count();
#endif
}

template<typename MessageT>
class Publisher {
public:
    Publisher() = default;
#if LIO_ROS_VERSION == 1
    explicit Publisher(ros::Publisher publisher) : publisher_(std::move(publisher)) {}
#else
    explicit Publisher(typename rclcpp::Publisher<MessageT>::SharedPtr publisher)
        : publisher_(std::move(publisher)) {}
#endif

    void publish(const MessageT& message) const {
#if LIO_ROS_VERSION == 1
        publisher_.publish(message);
#else
        if (publisher_) publisher_->publish(message);
#endif
    }

    explicit operator bool() const {
#if LIO_ROS_VERSION == 1
        return static_cast<bool>(publisher_);
#else
        return static_cast<bool>(publisher_);
#endif
    }

private:
#if LIO_ROS_VERSION == 1
    ros::Publisher publisher_;
#else
    typename rclcpp::Publisher<MessageT>::SharedPtr publisher_;
#endif
};

class Subscription {
public:
    Subscription() = default;
#if LIO_ROS_VERSION == 1
    explicit Subscription(ros::Subscriber subscription) : subscription_(std::move(subscription)) {}
#else
    explicit Subscription(rclcpp::SubscriptionBase::SharedPtr subscription)
        : subscription_(std::move(subscription)) {}
#endif

    void shutdown() {
#if LIO_ROS_VERSION == 1
        subscription_.shutdown();
#else
        subscription_.reset();
#endif
    }

private:
#if LIO_ROS_VERSION == 1
    ros::Subscriber subscription_;
#else
    rclcpp::SubscriptionBase::SharedPtr subscription_;
#endif
};

class Timer {
public:
    Timer() = default;
#if LIO_ROS_VERSION == 1
    explicit Timer(ros::Timer timer) : timer_(std::move(timer)) {}
#else
    explicit Timer(rclcpp::TimerBase::SharedPtr timer) : timer_(std::move(timer)) {}
#endif

private:
#if LIO_ROS_VERSION == 1
    ros::Timer timer_;
#else
    rclcpp::TimerBase::SharedPtr timer_;
#endif
};

class Node {
public:
    explicit Node(const std::string& node_name, const std::string& ros1_namespace = "~")
#if LIO_ROS_VERSION == 1
        : node_(ros1_namespace) {
        (void)node_name;
    }
#else
        : node_(std::make_shared<rclcpp::Node>(node_name)) {
        (void)ros1_namespace;
    }
#endif

    template<typename MessageT>
    Publisher<MessageT> advertise(const std::string& topic, std::size_t depth,
                                  bool latched = false) {
#if LIO_ROS_VERSION == 1
        return Publisher<MessageT>(node_.advertise<MessageT>(topic, static_cast<std::uint32_t>(depth), latched));
#else
        rclcpp::QoS qos{rclcpp::KeepLast(depth)};
        if (latched) qos.reliable().transient_local();
        return Publisher<MessageT>(node_->create_publisher<MessageT>(resolve_topic(topic), qos));
#endif
    }

    template<typename MessageT, typename CallbackT>
    Subscription subscribe(const std::string& topic, std::size_t depth, CallbackT&& callback,
                           bool sensor_data_qos = false) {
#if LIO_ROS_VERSION == 1
        (void)sensor_data_qos;
        return Subscription(node_.subscribe<MessageT>(topic, static_cast<std::uint32_t>(depth),
                                                       std::forward<CallbackT>(callback)));
#else
        rclcpp::QoS qos = sensor_data_qos
            ? rclcpp::QoS(rclcpp::SensorDataQoS()).keep_last(depth)
            : rclcpp::QoS(rclcpp::KeepLast(depth));
        return Subscription(node_->create_subscription<MessageT>(
            resolve_topic(topic), qos, std::forward<CallbackT>(callback)));
#endif
    }

    template<typename CallbackT>
    Timer create_timer(double frequency_hz, CallbackT&& callback) {
        const auto period = std::chrono::duration<double>(1.0 / frequency_hz);
#if LIO_ROS_VERSION == 1
        const ros::Duration ros_period(period.count());
        auto timer = node_.createTimer(ros_period,
            [cb = std::forward<CallbackT>(callback)](const ros::TimerEvent&) mutable { cb(); });
        return Timer(std::move(timer));
#else
        return Timer(rclcpp::create_timer(
            node_, node_->get_clock(), rclcpp::Duration(period),
            std::forward<CallbackT>(callback)));
#endif
    }

    template<typename T>
    void param(const std::string& name, T& value, const T& default_value) {
#if LIO_ROS_VERSION == 1
        node_.param(name, value, default_value);
#else
        value = node_->declare_parameter<T>(name, default_value);
#endif
    }

    template<typename T>
    bool get_parameter(const std::string& name, T& value) const {
#if LIO_ROS_VERSION == 1
        return node_.getParam(name, value);
#else
        return node_->has_parameter(name) && node_->get_parameter(name, value);
#endif
    }

    template<typename T>
    void set_parameter(const std::string& name, const T& value) {
#if LIO_ROS_VERSION == 1
        node_.setParam(name, value);
#else
        if (!node_->has_parameter(name)) node_->declare_parameter<T>(name, value);
        else node_->set_parameter(rclcpp::Parameter(name, value));
#endif
    }

    std::string get_namespace() const {
#if LIO_ROS_VERSION == 1
        return node_.getNamespace();
#else
        return node_->get_namespace();
#endif
    }

#if LIO_ROS_VERSION == 1
    ros::NodeHandle& raw() { return node_; }
#else
    const rclcpp::Node::SharedPtr& raw() const { return node_; }
#endif

private:
#if LIO_ROS_VERSION == 2
    static std::string resolve_topic(const std::string& topic) {
        if (topic.empty() || topic.front() == '/' || topic.front() == '~') return topic;
        return "~/" + topic;
    }
#endif

#if LIO_ROS_VERSION == 1
    ros::NodeHandle node_;
#else
    rclcpp::Node::SharedPtr node_;
#endif
};

inline void init(int argc, char** argv, const std::string& node_name,
                 bool install_signal_handlers = true) {
#if LIO_ROS_VERSION == 1
    const std::uint32_t options = install_signal_handlers
        ? 0U
        : static_cast<std::uint32_t>(ros::init_options::NoSigintHandler);
    ros::init(argc, argv, node_name, options);
#else
    (void)node_name;
    const auto signal_option = install_signal_handlers
        ? rclcpp::SignalHandlerOptions::All
        : rclcpp::SignalHandlerOptions::None;
    rclcpp::init(argc, argv, rclcpp::InitOptions(), signal_option);
#endif
}

inline void spin(Node& node) {
#if LIO_ROS_VERSION == 1
    (void)node;
    ros::spin();
#else
    rclcpp::spin(node.raw());
#endif
}

inline void shutdown() {
#if LIO_ROS_VERSION == 1
    ros::shutdown();
#else
    rclcpp::shutdown();
#endif
}

}  // namespace lio_ros

#if LIO_ROS_VERSION == 2
#ifndef ROS_INFO
#define ROS_INFO(...) RCLCPP_INFO(rclcpp::get_logger("lio"), __VA_ARGS__)
#define ROS_WARN(...) RCLCPP_WARN(rclcpp::get_logger("lio"), __VA_ARGS__)
#define ROS_ERROR(...) RCLCPP_ERROR(rclcpp::get_logger("lio"), __VA_ARGS__)
#define LIO_ROS_STREAM_LOG(level, expression) \
    do { \
        std::ostringstream lio_ros_log_stream; \
        lio_ros_log_stream << expression; \
        RCLCPP_##level(rclcpp::get_logger("lio"), "%s", lio_ros_log_stream.str().c_str()); \
    } while (false)
#define ROS_INFO_STREAM(expression) LIO_ROS_STREAM_LOG(INFO, expression)
#define ROS_WARN_STREAM(expression) LIO_ROS_STREAM_LOG(WARN, expression)
#define ROS_ERROR_STREAM(expression) LIO_ROS_STREAM_LOG(ERROR, expression)
#define ROS_DEBUG_STREAM_THROTTLE(period_seconds, expression) \
    do { \
        static auto lio_ros_last_debug_time = std::chrono::steady_clock::time_point::min(); \
        const auto lio_ros_debug_now = std::chrono::steady_clock::now(); \
        if (lio_ros_debug_now - lio_ros_last_debug_time >= \
            std::chrono::duration<double>(period_seconds)) { \
            lio_ros_last_debug_time = lio_ros_debug_now; \
            LIO_ROS_STREAM_LOG(DEBUG, expression); \
        } \
    } while (false)
#define ROS_WARN_THROTTLE(period_seconds, ...) \
    do { \
        static auto lio_ros_last_warn_time = std::chrono::steady_clock::time_point::min(); \
        const auto lio_ros_warn_now = std::chrono::steady_clock::now(); \
        if (lio_ros_warn_now - lio_ros_last_warn_time >= \
            std::chrono::duration<double>(period_seconds)) { \
            lio_ros_last_warn_time = lio_ros_warn_now; \
            RCLCPP_WARN(rclcpp::get_logger("lio"), __VA_ARGS__); \
        } \
    } while (false)
#endif
#endif

#endif  // LIO_SUPPORT_ROS_COMPAT_H
