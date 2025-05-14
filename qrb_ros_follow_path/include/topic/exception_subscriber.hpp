/*
 * Copyright (c) 2025 Qualcomm Innovation Center, Inc. All rights reserved.
 * SPDX-License-Identifier: BSD-3-Clause-Clear
 */

#ifndef QRB_ROS_NAVIGATION__EXCEPTION_SUBSCRIBER_HPP_
#define QRB_ROS_NAVIGATION__EXCEPTION_SUBSCRIBER_HPP_

#include "follow_path_manager.hpp"
#include "manager/ros_common.hpp"

using namespace qrb::navigation;

namespace qrb_ros
{
namespace navigation
{

enum class EmergencyType
{
  ENTER = 0,
  EXIT = 1,
};

class ExceptionSubscriber : public rclcpp_lifecycle::LifecycleNode
{
public:
  ExceptionSubscriber(std::shared_ptr<FollowPathManager> & manager);

  ~ExceptionSubscriber();

private:
  LifecycleNodeInterface::CallbackReturn on_configure(const rclcpp_lifecycle::State &) override;
  LifecycleNodeInterface::CallbackReturn on_activate(const rclcpp_lifecycle::State &) override;
  LifecycleNodeInterface::CallbackReturn on_deactivate(const rclcpp_lifecycle::State &) override;
  LifecycleNodeInterface::CallbackReturn on_cleanup(const rclcpp_lifecycle::State &) override;
  LifecycleNodeInterface::CallbackReturn on_shutdown(const rclcpp_lifecycle::State &) override;

  void init_subscriber();
  void deinit_subscriber();

  void topic_callback(const Exception::SharedPtr msg);

  void laser_callback(sensor_msgs::msg::LaserScan::ConstSharedPtr scan);

  void developer_mode_callback(const std_msgs::msg::Int16::ConstSharedPtr msg);

  rclcpp::Subscription<Exception>::SharedPtr subscriber_;
  rclcpp::Subscription<sensor_msgs::msg::LaserScan>::SharedPtr laser_sub_;
  std::shared_ptr<FollowPathManager> manager_;
  bool emergency_braking_;

  rclcpp::Logger logger_{ rclcpp::get_logger("exception_subscriber") };
};

}  // namespace navigation
}  // namespace qrb_ros
#endif  // QRB_ROS_NAVIGATION__EXCEPTION_SUBSCRIBER_HPP_