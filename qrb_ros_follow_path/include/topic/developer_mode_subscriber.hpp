/*
 * Copyright (c) 2025 Qualcomm Innovation Center, Inc. All rights reserved.
 * SPDX-License-Identifier: BSD-3-Clause-Clear
 */

#ifndef QRB_ROS_NAVIGATION__DEVELOPER_MODE_SUBSCRIBER_HPP_
#define QRB_ROS_NAVIGATION__DEVELOPER_MODE_SUBSCRIBER_HPP_

#include "follow_path_manager.hpp"
#include "manager/ros_common.hpp"

using namespace qrb::navigation;

namespace qrb_ros
{
namespace navigation
{
class DeveloperModeSubscriber : public rclcpp_lifecycle::LifecycleNode
{
private:
  LifecycleNodeInterface::CallbackReturn on_configure(const rclcpp_lifecycle::State &) override;
  LifecycleNodeInterface::CallbackReturn on_activate(const rclcpp_lifecycle::State &) override;
  LifecycleNodeInterface::CallbackReturn on_deactivate(const rclcpp_lifecycle::State &) override;
  LifecycleNodeInterface::CallbackReturn on_cleanup(const rclcpp_lifecycle::State &) override;
  LifecycleNodeInterface::CallbackReturn on_shutdown(const rclcpp_lifecycle::State &) override;

  rclcpp::Subscription<std_msgs::msg::Int16>::SharedPtr sub_;
  std::shared_ptr<FollowPathManager> manager_;
  rclcpp::Logger logger_{ rclcpp::get_logger("developer_mode_subscriber") };

  void developer_mode_callback(const std_msgs::msg::Int16::ConstSharedPtr msg);
  void init_subscriber();
  void deinit_subscriber();

public:
  DeveloperModeSubscriber(std::shared_ptr<FollowPathManager> & manager);
  ~DeveloperModeSubscriber();
};
}  // namespace navigation
}  // namespace qrb_ros
#endif  // QRB_ROS_NAVIGATION__DEVELOPER_MODE_SUBSCRIBER_HPP_