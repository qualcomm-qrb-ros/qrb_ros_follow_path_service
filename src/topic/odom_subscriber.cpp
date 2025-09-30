/*
 * Copyright (c) 2025 Qualcomm Innovation Center, Inc. All rights reserved.
 * SPDX-License-Identifier: BSD-3-Clause-Clear
 */

#include "odom_subscriber.hpp"

namespace qrb_ros
{
namespace navigation
{
OdomSubscriber::OdomSubscriber(std::shared_ptr<FollowPathManager> & manager)
  : LifecycleNode("odom_sub"), manager_(manager)
{
  get_robot_velocity_cb_ = [&](twist_vel & twist) { get_robot_velocity(twist); };
  manager_->register_get_robot_velocity_callback(get_robot_velocity_cb_);
}

OdomSubscriber::~OdomSubscriber()
{
  RCLCPP_DEBUG(logger_, "Destructor OdomSubscriber object");
}

LifecycleNodeInterface::CallbackReturn OdomSubscriber::on_configure(const rclcpp_lifecycle::State &)
{
  RCLCPP_INFO(logger_, "Configuring");
  return LifecycleNodeInterface::CallbackReturn::SUCCESS;
}

LifecycleNodeInterface::CallbackReturn OdomSubscriber::on_activate(const rclcpp_lifecycle::State &)
{
  RCLCPP_INFO(logger_, "Activating");
  init_subscriber();
  return LifecycleNodeInterface::CallbackReturn::SUCCESS;
}

LifecycleNodeInterface::CallbackReturn OdomSubscriber::on_deactivate(
    const rclcpp_lifecycle::State &)
{
  RCLCPP_INFO(logger_, "Deactivating");
  deinit_subscriber();
  return LifecycleNodeInterface::CallbackReturn::SUCCESS;
}

LifecycleNodeInterface::CallbackReturn OdomSubscriber::on_cleanup(const rclcpp_lifecycle::State &)
{
  RCLCPP_INFO(logger_, "Cleaning up");
  return LifecycleNodeInterface::CallbackReturn::SUCCESS;
}

LifecycleNodeInterface::CallbackReturn OdomSubscriber::on_shutdown(const rclcpp_lifecycle::State &)
{
  RCLCPP_INFO(logger_, "Shutting down");
  return LifecycleNodeInterface::CallbackReturn::SUCCESS;
}

void OdomSubscriber::init_subscriber()
{
  RCLCPP_INFO(logger_, "init odom subsrcier");

  using namespace std::placeholders;
  sub_ = create_subscription<nav_msgs::msg::Odometry>("odom", rclcpp::SystemDefaultsQoS(),
      std::bind(&OdomSubscriber::odom_callback, this, std::placeholders::_1));
}

void OdomSubscriber::deinit_subscriber()
{
  RCLCPP_INFO(logger_, "deinit odom subsrcier");
  sub_.reset();
}

void OdomSubscriber::get_robot_velocity(twist_vel & twist)
{
  std::lock_guard<std::mutex> lock(mutex_);
  twist.x = odom_velocity_.velocity.x;
  twist.y = odom_velocity_.velocity.y;
  twist.z = odom_velocity_.velocity.theta;
}

void OdomSubscriber::odom_callback(const nav_msgs::msg::Odometry::SharedPtr msg)
{
  std::lock_guard<std::mutex> lock(mutex_);
  odom_velocity_.header = msg->header;
  odom_velocity_.velocity.x = msg->twist.twist.linear.x;
  odom_velocity_.velocity.y = msg->twist.twist.linear.y;
  odom_velocity_.velocity.theta = msg->twist.twist.angular.z;
  RCLCPP_DEBUG(logger_, "=======odom_callback velocity_x:%f,velocity_y:%f,velocity_theta:%f",
      odom_velocity_.velocity.x, odom_velocity_.velocity.y, odom_velocity_.velocity.theta);
}

}  // namespace navigation
}  // namespace qrb_ros