/*
 * Copyright (c) 2025 Qualcomm Innovation Center, Inc. All rights reserved.
 * SPDX-License-Identifier: BSD-3-Clause-Clear
 */

#include "path_publisher.hpp"

namespace qrb_ros
{
namespace navigation
{
PathPublisher::PathPublisher(std::shared_ptr<FollowPathManager> & manager)
  : LifecycleNode("path_pub"), manager_(manager)
{
  publish_real_path_cb_ = [&](std::vector<point_2d> & path) { publish_real_path(path); };
  manager_->register_publish_real_path_callback(publish_real_path_cb_);

  publish_global_path_cb_ = [&](std::vector<point_2d> & path, double x, double y) {
    publish_global_path(path, x, y);
  };
  manager_->register_publish_global_path_callback(publish_global_path_cb_);
}

PathPublisher::~PathPublisher() {}

LifecycleNodeInterface::CallbackReturn PathPublisher::on_configure(const rclcpp_lifecycle::State &)
{
  RCLCPP_INFO(logger_, "Configuring");
  init_publisher();
  return LifecycleNodeInterface::CallbackReturn::SUCCESS;
}

LifecycleNodeInterface::CallbackReturn PathPublisher::on_activate(const rclcpp_lifecycle::State &)
{
  RCLCPP_INFO(logger_, "Activating");
  global_pub_->on_activate();
  real_pub_->on_activate();
  return LifecycleNodeInterface::CallbackReturn::SUCCESS;
}

LifecycleNodeInterface::CallbackReturn PathPublisher::on_deactivate(const rclcpp_lifecycle::State &)
{
  RCLCPP_INFO(logger_, "Deactivating");
  global_pub_->on_deactivate();
  real_pub_->on_deactivate();
  return LifecycleNodeInterface::CallbackReturn::SUCCESS;
}

LifecycleNodeInterface::CallbackReturn PathPublisher::on_cleanup(const rclcpp_lifecycle::State &)
{
  RCLCPP_INFO(logger_, "Cleaning up");
  deinit_publisher();
  return LifecycleNodeInterface::CallbackReturn::SUCCESS;
}

LifecycleNodeInterface::CallbackReturn PathPublisher::on_shutdown(const rclcpp_lifecycle::State &)
{
  RCLCPP_INFO(logger_, "Shutting down");
  deinit_publisher();
  return LifecycleNodeInterface::CallbackReturn::SUCCESS;
}

void PathPublisher::init_publisher()
{
  RCLCPP_DEBUG(logger_, "init path publisher");
  global_pub_ = create_publisher<nav_msgs::msg::Path>("global_path", 10);
  real_pub_ = create_publisher<nav_msgs::msg::Path>("real_path", 10);
}

void PathPublisher::deinit_publisher()
{
  RCLCPP_DEBUG(logger_, "deinit path publisher");
  global_pub_.reset();
  real_pub_.reset();
}

void PathPublisher::publish_global_path(const std::vector<point_2d> & point_list,
    double & origin_x,
    double & origin_y)
{
  RCLCPP_DEBUG(logger_, "origin_x = %f, origin_y = %f", origin_x, origin_y);
  nav_msgs::msg::Path path;
  geometry_msgs::msg::PoseStamped pose;
  auto time = get_clock()->now();
  std_msgs::msg::Header header;
  header.frame_id = "/map";
  header.stamp.sec = time.seconds();
  header.stamp.nanosec = time.nanoseconds() % 1000000000;
  path.header = header;
  auto size = point_list.size();
  for (size_t i = 0; i < size; i++) {
    pose.pose.position.x = point_list.at(i).x;
    pose.pose.position.y = point_list.at(i).y;
    pose.header.stamp = header.stamp;
    path.poses.push_back(pose);
  }
  global_pub_->publish(path);
}

void PathPublisher::publish_real_path(const std::vector<point_2d> & point_list)
{
  nav_msgs::msg::Path path;
  geometry_msgs::msg::PoseStamped pose;
  auto time = get_clock()->now();
  std_msgs::msg::Header header;
  header.frame_id = "/map";
  header.stamp.sec = time.seconds();
  header.stamp.nanosec = time.nanoseconds() % 1000000000;
  path.header = header;
  auto size = point_list.size();
  for (size_t i = 0; i < size; i++) {
    pose.pose.position.x = point_list.at(i).x;
    pose.pose.position.y = point_list.at(i).y;
    pose.header.stamp = header.stamp;
    path.poses.push_back(pose);
  }
  real_pub_->publish(path);
}

}  // namespace navigation
}  // namespace qrb_ros