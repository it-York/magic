/*
 * Copyright (c) 2026 xiaofan
 * SPDX-License-Identifier: GPL-2.0-or-later
 *
 * ROS node interface and runtime state declarations for elevator-lio.
 */

#ifndef LIO_NODE_H
#define LIO_NODE_H

// 相比于 ros2 头文件的变化
#include "support/ros_compat.h"
#include <tf2_ros/transform_broadcaster.h>
#include <tf2_ros/static_transform_broadcaster.h>
#include <pcl_conversions/pcl_conversions.h>  // 保持不变
#include "support/YamlReader.h"
#include "support/TopicProcess.h"
#include "estimator/ESEKF.h"
#include "node/LidarPipeline.h"
#include <csignal>  // 包含信号处理相关的头文件

#include <functional>
#include <map>

struct TimeRecord {
    double lidar_end_time;
    double lidar_beg_time;
    double img_beg_time;
    double img_end_time;
    double imu_beg_time;
    double imu_end_time;
};

namespace FSM {
    enum class State { Waitting, Initializing, IMUProcess, LidarProcess, ImgProcess /*…*/ };
    enum class Event { IMUTriger, InitEnd, LidarTriger, LidarEnd, ImgTriger, ImgEnd /*…*/ };
    struct Transition {
        State next_state;
        std::function<void()> action;
    };
    using Key = std::pair<State,Event>;
    // 状态表，和 dispatch() 声明
    extern const std::map<Key, Transition> kTransitionTable;
    extern State current_state;
    void dispatch(Event e);
    // 辅助打印函数
    inline const char* to_string(State s) {
        switch(s) {
            case State::Waitting:      return "Waitting";
            case State::Initializing:  return "Initializing";
            case State::IMUProcess:    return "IMUProcess";
            case State::LidarProcess:  return "LidarProcess";
            case State::ImgProcess:    return "ImgProcess";
            default:                   return "UnknownState";
        }
    }
    inline const char* to_string(Event e) {
        switch(e) {
            case Event::IMUTriger:   return "IMUTriger";
            case Event::InitEnd:     return "InitEnd";
            case Event::LidarTriger: return "LidarTriger";
            case Event::LidarEnd:    return "LidarEnd";
            case Event::ImgTriger:   return "ImgTriger";
            case Event::ImgEnd:      return "ImgEnd";
            default:                 return "UnknownEvent";
        }
    }
}

class LIONode {
public:
    explicit LIONode(std::shared_ptr<lio_ros::Node> node);

    void publish_imu_odometry(State state, double stamp_sec = -1.0);
    void publish_body_odometry(State state, double stamp_sec = -1.0);
    void publish_Dedistort_clouds_lidar(const PointCloudXYZI::Ptr &cloud);
    void publish_Effect_clouds_lidar(const PointCloudXYZI::Ptr &cloud);
    void publish_Rejected_clouds_lidar(const PointCloudXYZI::Ptr &cloud);
    void publishStaticTransform();
    void publishGlobalMap();
    void publishIKDTree();
    void save_odometry(State state,double time, const string& save_path = "");
    void save_singel_clouds_world(PointCloudXYZI::Ptr clouds_lidar);
    static void lasermap_fov_segment();
    static void map_incremental(PointCloudXYZI & lidar_clouds);
    void timer_1HZ_callback();
    void timer_10HZ_callback();
    void timer_500HZ_callback();
    void timer_2000HZ_callback();


private:


    std::shared_ptr<lio_ros::Node> node_;
    lio_ros::Publisher<lio_ros::PointCloud2> clouds_lidar_pub_;
    lio_ros::Publisher<lio_ros::PointCloud2> clouds_lidar_effect_pub_;
    lio_ros::Publisher<lio_ros::PointCloud2> clouds_lidar_reject_pub_;
    lio_ros::Publisher<lio_ros::PointCloud2> global_map_pub_;
    lio_ros::Publisher<lio_ros::PointCloud2> ikdtree_pub_;
    lio_ros::Publisher<lio_ros::Odometry> odom_imu_pub_;
    lio_ros::Publisher<lio_ros::Odometry> odom_body_pub_;
    lio_ros::Publisher<lio_ros::Path> odom_path_pub_;
    lio_ros::Subscription pointcloud_sub_;
    lio_ros::Subscription imu_sub_;
    lio_ros::Subscription wheel_sub_;
    std::unique_ptr<tf2_ros::TransformBroadcaster> tf_broadcaster_;
    std::unique_ptr<tf2_ros::StaticTransformBroadcaster> static_broadcaster_;
    lio_ros::Timer timer_2000HZ_;
    lio_ros::Timer timer_500HZ_;
    lio_ros::Timer timer_10HZ_;
    lio_ros::Timer timer_1HZ_;
    lio_ros::Subscription elevator_flag_sub_;
    lio_ros::Publisher<lio_ros::Bool> ele_state_pub_;

    void elevatorFlagCallback(const lio_ros::BoolConstPtr& msg);

    LidarPipeline lidar_pipeline_;
};

// Callback Function
void pcl_cbk_custom(const lio_ros::CustomMsgConstPtr& msg);
void pcl_cbk_pc2(const lio_ros::PointCloud2ConstPtr &msg);
void imu_cbk(const lio_ros::ImuConstPtr& msg);
void wheel_cbk(const lio_ros::WheelInfoConstPtr& msg);

void convert_traj_csv_to_tum();

#endif // LIO_NODE_H
