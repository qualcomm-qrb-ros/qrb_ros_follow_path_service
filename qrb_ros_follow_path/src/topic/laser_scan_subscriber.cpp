/*
 * Copyright (c) 2025 Qualcomm Innovation Center, Inc. All rights reserved.
 * SPDX-License-Identifier: BSD-3-Clause-Clear
 */

#include "laser_scan_subscriber.hpp"

namespace qrb_ros
{
namespace navigation
{
LaserScanSubscriber::LaserScanSubscriber(std::shared_ptr<FollowPathManager> & manager)
  : LifecycleNode("laser_sub"), manager_(manager)
{
  tf_buffer_ = std::make_shared<tf2_ros::Buffer>(get_clock());
  tf_listener_ = std::make_shared<tf2_ros::TransformListener>(*tf_buffer_);
  active_ = false;
  RCLCPP_INFO(logger_, "LaserScanSubscriber");
}

LaserScanSubscriber::~LaserScanSubscriber()
{
  RCLCPP_DEBUG(logger_, "Destructor LaserScanSubscriber object");
}

LifecycleNodeInterface::CallbackReturn LaserScanSubscriber::on_configure(
    const rclcpp_lifecycle::State &)
{
  RCLCPP_INFO(logger_, "Configuring");
  init_subscriber();
  return LifecycleNodeInterface::CallbackReturn::SUCCESS;
}

LifecycleNodeInterface::CallbackReturn LaserScanSubscriber::on_activate(
    const rclcpp_lifecycle::State &)
{
  RCLCPP_INFO(logger_, "Activating");
  active_ = true;
  return LifecycleNodeInterface::CallbackReturn::SUCCESS;
}

LifecycleNodeInterface::CallbackReturn LaserScanSubscriber::on_deactivate(
    const rclcpp_lifecycle::State &)
{
  RCLCPP_INFO(logger_, "Deactivating");
  active_ = false;
  return LifecycleNodeInterface::CallbackReturn::SUCCESS;
}

LifecycleNodeInterface::CallbackReturn LaserScanSubscriber::on_cleanup(
    const rclcpp_lifecycle::State &)
{
  RCLCPP_INFO(logger_, "Cleaning up");
  deinit_subscriber();
  active_ = false;
  return LifecycleNodeInterface::CallbackReturn::SUCCESS;
}

LifecycleNodeInterface::CallbackReturn LaserScanSubscriber::on_shutdown(
    const rclcpp_lifecycle::State &)
{
  RCLCPP_INFO(logger_, "Shutting down");
  return LifecycleNodeInterface::CallbackReturn::SUCCESS;
}

void LaserScanSubscriber::init_subscriber()
{
  RCLCPP_INFO(logger_, "init laser scan subsrcier");

  using namespace std::placeholders;
  sub_ = create_subscription<sensor_msgs::msg::LaserScan>("scan", rclcpp::SystemDefaultsQoS(),
      std::bind(&LaserScanSubscriber::laser_callback, this, std::placeholders::_1));
}

void LaserScanSubscriber::deinit_subscriber()
{
  RCLCPP_INFO(logger_, "deinit laser scan subsrcier");
  sub_.reset();
}

void LaserScanSubscriber::laser_callback(sensor_msgs::msg::LaserScan::ConstSharedPtr scan)
{
  if (!active_) {
    RCLCPP_INFO(logger_, "node is not active");
    return;
  }
  std::lock_guard<std::mutex> lock(mutex_);

  if (print_laser_count_ % 300 == 0) {
    RCLCPP_INFO(logger_, "receive a scan");
  }
  print_laser_count_++;
  scan_ = std::make_shared<sensor_msgs::msg::LaserScan>(*scan);
}

bool LaserScanSubscriber::get_laser_scan_data(sensor_msgs::msg::LaserScan::SharedPtr & scan)
{
  std::lock_guard<std::mutex> lock(mutex_);

  if (scan_ == nullptr) {
    RCLCPP_INFO(logger_, "scan_ is null pointer");
    return false;
  }

  scan->header.frame_id = scan_->header.frame_id;
  scan->header.stamp = scan_->header.stamp;
  scan->angle_min = scan_->angle_min;
  scan->angle_max = scan_->angle_max;
  scan->range_min = scan_->range_min;
  scan->range_max = scan_->range_max;
  scan->angle_increment = scan_->angle_increment;
  uint32_t len = scan_->ranges.size();
  scan->ranges.resize(len);

  for (uint32_t i = 0; i < len; ++i) {
    scan->ranges[i] = scan_->ranges[i];
  }
  return true;
}

}  // namespace navigation
}  // namespace qrb_ros