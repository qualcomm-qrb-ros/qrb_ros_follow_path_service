/*
 * Copyright (c) 2025 Qualcomm Innovation Center, Inc. All rights reserved.
 * SPDX-License-Identifier: BSD-3-Clause-Clear
 */

#include "developer_mode_subscriber.hpp"

namespace qrb_ros
{
namespace navigation
{
DeveloperModeSubscriber::DeveloperModeSubscriber(std::shared_ptr<FollowPathManager> & manager)
  : LifecycleNode("debug_sub"), manager_(manager)
{
  RCLCPP_INFO(logger_, "DeveloperModeSubscriber");
}

DeveloperModeSubscriber::~DeveloperModeSubscriber() {}

LifecycleNodeInterface::CallbackReturn DeveloperModeSubscriber::on_configure(
    const rclcpp_lifecycle::State &)
{
  RCLCPP_INFO(logger_, "Configuring");
  return LifecycleNodeInterface::CallbackReturn::SUCCESS;
}

LifecycleNodeInterface::CallbackReturn DeveloperModeSubscriber::on_activate(
    const rclcpp_lifecycle::State &)
{
  RCLCPP_INFO(logger_, "Activating");
  init_subscriber();
  return LifecycleNodeInterface::CallbackReturn::SUCCESS;
}

LifecycleNodeInterface::CallbackReturn DeveloperModeSubscriber::on_deactivate(
    const rclcpp_lifecycle::State &)
{
  RCLCPP_INFO(logger_, "Deactivating");
  deinit_subscriber();
  return LifecycleNodeInterface::CallbackReturn::SUCCESS;
}

LifecycleNodeInterface::CallbackReturn DeveloperModeSubscriber::on_cleanup(
    const rclcpp_lifecycle::State &)
{
  RCLCPP_INFO(logger_, "Cleaning up");
  return LifecycleNodeInterface::CallbackReturn::SUCCESS;
}

LifecycleNodeInterface::CallbackReturn DeveloperModeSubscriber::on_shutdown(
    const rclcpp_lifecycle::State &)
{
  RCLCPP_INFO(logger_, "Shutting down");
  return LifecycleNodeInterface::CallbackReturn::SUCCESS;
}

void DeveloperModeSubscriber::init_subscriber()
{
  using namespace std::placeholders;

  sub_ = this->create_subscription<std_msgs::msg::Int16>(
      "developer_mode", 10, std::bind(&DeveloperModeSubscriber::developer_mode_callback, this, _1));
}

void DeveloperModeSubscriber::deinit_subscriber()
{
  sub_.reset();
}

void DeveloperModeSubscriber::developer_mode_callback(
    const std_msgs::msg::Int16::ConstSharedPtr msg)
{
  RCLCPP_INFO(logger_, "subscribe callback");
  int16_t debug_msg = msg->data;
  manager_->send_debug_event(debug_msg);
}
}  // namespace navigation
}  // namespace qrb_ros
