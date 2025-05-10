/*
 * Copyright (c) 2025 Qualcomm Innovation Center, Inc. All rights reserved.
 * SPDX-License-Identifier: BSD-3-Clause-Clear
 */

#include "twist_publisher.hpp"

namespace qrb_ros
{
namespace navigation
{
TwistPublisher::TwistPublisher(std::shared_ptr<FollowPathManager> & manager)
  : LifecycleNode("twist_pub"), manager_(manager)
{
  publish_twist_cb_ = [&](twist_vel & vel) {
    vel_mtx_.lock();
    send_velocity(vel);
    vel_mtx_.unlock();
  };
  manager_->register_publish_twist_callback(publish_twist_cb_);
}

TwistPublisher::~TwistPublisher()
{
  RCLCPP_DEBUG(logger_, "Destructor TwistPublisher object");
}

LifecycleNodeInterface::CallbackReturn TwistPublisher::on_configure(const rclcpp_lifecycle::State &)
{
  RCLCPP_INFO(logger_, "Configuring");
  init_publisher();
  return LifecycleNodeInterface::CallbackReturn::SUCCESS;
}

LifecycleNodeInterface::CallbackReturn TwistPublisher::on_activate(const rclcpp_lifecycle::State &)
{
  RCLCPP_INFO(logger_, "Activating");
  pub_->on_activate();
  return LifecycleNodeInterface::CallbackReturn::SUCCESS;
}

LifecycleNodeInterface::CallbackReturn TwistPublisher::on_deactivate(
    const rclcpp_lifecycle::State &)
{
  RCLCPP_INFO(logger_, "Deactivating");
  pub_->on_deactivate();
  return LifecycleNodeInterface::CallbackReturn::SUCCESS;
}

LifecycleNodeInterface::CallbackReturn TwistPublisher::on_cleanup(const rclcpp_lifecycle::State &)
{
  RCLCPP_INFO(logger_, "Cleaning up");
  deinit_publisher();
  return LifecycleNodeInterface::CallbackReturn::SUCCESS;
}

LifecycleNodeInterface::CallbackReturn TwistPublisher::on_shutdown(const rclcpp_lifecycle::State &)
{
  RCLCPP_INFO(logger_, "Shutting down");
  deinit_publisher();
  return LifecycleNodeInterface::CallbackReturn::SUCCESS;
}

void TwistPublisher::init_publisher()
{
  RCLCPP_DEBUG(logger_, "init twist publisher");
  pub_ = create_publisher<geometry_msgs::msg::Twist>("cmd_vel", 10);
}

void TwistPublisher::deinit_publisher()
{
  RCLCPP_DEBUG(logger_, "deinit twist publisher");
  pub_.reset();
}

void TwistPublisher::send_velocity(twist_vel & vel)
{
  geometry_msgs::msg::Twist velocity;
  velocity.linear.x = vel.x;
  velocity.linear.y = vel.y;
  velocity.angular.z = vel.z;

  RCLCPP_INFO(logger_, "send velocity(%.2f, %.2f, %.2f) to robot", velocity.linear.x,
      velocity.linear.y, velocity.angular.z);
  pub_->publish(velocity);
}

}  // namespace navigation
}  // namespace qrb_ros