/*
 * Copyright (c) 2025 Qualcomm Innovation Center, Inc. All rights reserved.
 * SPDX-License-Identifier: BSD-3-Clause-Clear
 */

#include "topic/exception_subscriber.hpp"

constexpr char const * node_name = "exception_sub";
constexpr char const * topic_name = "robot_base_exception";

namespace qrb_ros
{
namespace navigation
{
using namespace std::placeholders;

ExceptionSubscriber::ExceptionSubscriber(std::shared_ptr<FollowPathManager> & manager)
  : LifecycleNode(node_name), manager_(manager)
{
  RCLCPP_INFO(logger_, "Creating");
  emergency_braking_ = false;
}

ExceptionSubscriber::~ExceptionSubscriber()
{
  RCLCPP_INFO(logger_, "Destroying");
}

LifecycleNodeInterface::CallbackReturn ExceptionSubscriber::on_configure(
    const rclcpp_lifecycle::State &)
{
  RCLCPP_INFO(logger_, "Configuring");
  init_subscriber();
  return LifecycleNodeInterface::CallbackReturn::SUCCESS;
}

LifecycleNodeInterface::CallbackReturn ExceptionSubscriber::on_activate(
    const rclcpp_lifecycle::State &)
{
  RCLCPP_INFO(logger_, "Activating");
  return LifecycleNodeInterface::CallbackReturn::SUCCESS;
}

LifecycleNodeInterface::CallbackReturn ExceptionSubscriber::on_deactivate(
    const rclcpp_lifecycle::State &)
{
  RCLCPP_INFO(logger_, "Deactivating");
  return LifecycleNodeInterface::CallbackReturn::SUCCESS;
}

LifecycleNodeInterface::CallbackReturn ExceptionSubscriber::on_cleanup(
    const rclcpp_lifecycle::State &)
{
  RCLCPP_INFO(logger_, "Cleaning up");
  deinit_subscriber();
  return LifecycleNodeInterface::CallbackReturn::SUCCESS;
}

LifecycleNodeInterface::CallbackReturn ExceptionSubscriber::on_shutdown(
    const rclcpp_lifecycle::State &)
{
  RCLCPP_INFO(logger_, "Shutting down");
  return LifecycleNodeInterface::CallbackReturn::SUCCESS;
}

void ExceptionSubscriber::init_subscriber()
{
  subscriber_ = this->create_subscription<Exception>(
      topic_name, 10, std::bind(&ExceptionSubscriber::topic_callback, this, _1));

  laser_sub_ = this->create_subscription<sensor_msgs::msg::LaserScan>(
      "/scan", rclcpp::SensorDataQoS(), std::bind(&ExceptionSubscriber::laser_callback, this, _1));
}

void ExceptionSubscriber::deinit_subscriber()
{
  subscriber_.reset();
  laser_sub_.reset();
}

void ExceptionSubscriber::topic_callback(const Exception::SharedPtr msg)
{
  uint8_t type = msg->type;
  if (type == (uint8_t)Exception::EMERGENCY) {
    uint8_t event = msg->event;
    if (event == (uint8_t)Exception::ENTER) {
      RCLCPP_INFO(logger_, "Enter emergency");
      manager_->handle_emergency(true);
    } else if (event == (uint8_t)Exception::EXIT) {
      RCLCPP_INFO(logger_, "Exit emergency");
      manager_->handle_emergency(false);
    }
  } else {
    RCLCPP_INFO(logger_, "Emergency type is error, type=%d", type);
  }
}

void ExceptionSubscriber::laser_callback(sensor_msgs::msg::LaserScan::ConstSharedPtr scan)
{
  uint32_t len = scan->ranges.size();
  for (uint32_t i = 0; i < len; i++) {
    double range = scan->ranges[i];
    if ((range < 0.15) && (range >= scan->range_min)) {
      RCLCPP_INFO(logger_, "============Prevent collision, range:%f=============", range);
      manager_->handle_amr_exception(true);
      emergency_braking_ = true;
      return;
    }
  }
  if (emergency_braking_) {
    manager_->handle_amr_exception(false);
    emergency_braking_ = false;
  }
}

void ExceptionSubscriber::developer_mode_callback(const std_msgs::msg::Int16::ConstSharedPtr msg)
{
  int16_t val = msg->data;
  RCLCPP_INFO(logger_, "developer_mode_callback, val=%d", val);
}
}  // namespace navigation
}  // namespace qrb_ros