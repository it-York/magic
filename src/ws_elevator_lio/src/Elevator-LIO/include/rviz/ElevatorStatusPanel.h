/*
 * Copyright (c) 2026 xiaofan
 * SPDX-License-Identifier: GPL-2.0-or-later
 */

// MOC receives LIO_ROS_VERSION from CMake so it generates the correct base class.
#ifndef LIO_RVIZ_ELEVATOR_STATUS_PANEL_H
#define LIO_RVIZ_ELEVATOR_STATUS_PANEL_H

#include <QElapsedTimer>
#include <QString>
#include "support/ros_compat.h"
#if LIO_ROS_VERSION == 1
#include <rviz/panel.h>
#else
#include <rviz_common/panel.hpp>
#endif

class QLabel;
class QLineEdit;
class QPushButton;
class QTimer;

namespace lio_rviz_plugins {

class ElevatorStatusPanel final
#if LIO_ROS_VERSION == 1
    : public rviz::Panel {
#else
    : public rviz_common::Panel {
#endif
    Q_OBJECT

public:
    explicit ElevatorStatusPanel(QWidget* parent = nullptr);

#if LIO_ROS_VERSION == 1
    void load(const rviz::Config& config) override;
    void save(rviz::Config config) const override;
#else
    void onInitialize() override;
    void load(const rviz_common::Config& config) override;
    void save(rviz_common::Config config) const override;
#endif

Q_SIGNALS:
    void stateMessageReceived(bool in_elevator, double displacement,
                              double velocity, double acceleration);

private Q_SLOTS:
    void subscribeToTopic();
    void updateState(bool in_elevator, double displacement,
                     double velocity, double acceleration);
    void checkMessageTimeout();

private:
    enum class DisplayState {
        Unknown,
        Normal,
        Elevator,
    };

    void stateCallback(const lio_ros::ElevatorStateConstPtr& msg);
    void setDisplayState(DisplayState state, const QString& detail = QString());
    void setEstimateValues(bool available, double displacement = 0.0,
                           double velocity = 0.0, double acceleration = 0.0);

#if LIO_ROS_VERSION == 1
    ros::NodeHandle nh_;
    ros::Subscriber state_sub_;
#else
    rclcpp::Node::SharedPtr ros_node_;
    rclcpp::Subscription<lio_ros::ElevatorState>::SharedPtr state_sub_;
#endif
    QLineEdit* topic_edit_ = nullptr;
    QPushButton* subscribe_button_ = nullptr;
    QLabel* indicator_ = nullptr;
    QLabel* state_label_ = nullptr;
    QLabel* detail_label_ = nullptr;
    QLabel* displacement_value_ = nullptr;
    QLabel* velocity_value_ = nullptr;
    QLabel* acceleration_value_ = nullptr;
    QTimer* timeout_timer_ = nullptr;
    QElapsedTimer last_message_;
    bool received_message_ = false;
};

}  // namespace lio_rviz_plugins

#endif  // LIO_RVIZ_ELEVATOR_STATUS_PANEL_H
