/*
 * Copyright (c) 2025 Qualcomm Innovation Center, Inc. All rights reserved.
 * SPDX-License-Identifier: BSD-3-Clause-Clear
 */

#include "navigation_controller.hpp"
#include <string.h>
#include <unistd.h>

namespace qrb_ros
{
namespace navigation
{

NavigationController::NavigationController() : Node("NavigationController")
{
  init_nodes();
}

NavigationController::~NavigationController()
{
  RCLCPP_INFO(logger_, "Destroy NavigationController");
}

void NavigationController::init_nodes()
{
  RCLCPP_INFO(logger_, "init_nodes");

  manager_ = std::shared_ptr<FollowPathManager>(new FollowPathManager());

  executor_ = std::shared_ptr<rclcpp::executors::MultiThreadedExecutor>(
      new rclcpp::executors::MultiThreadedExecutor());

  virtual_path_service_server_ =
      std::shared_ptr<VirtualPathServiceServer>(new VirtualPathServiceServer(manager_));

  odom_sub_ = std::shared_ptr<OdomSubscriber>(new OdomSubscriber(manager_));

  laser_scan_sub_ = std::shared_ptr<LaserScanSubscriber>(new LaserScanSubscriber(manager_));

  map_sub_ = std::shared_ptr<MapSubscriber>(new MapSubscriber(manager_, laser_scan_sub_));

  vel_publisher_ = std::shared_ptr<TwistPublisher>(new TwistPublisher(manager_));

  path_publisher_ = std::shared_ptr<PathPublisher>(new PathPublisher(manager_));

  follow_path_server_ =
      std::shared_ptr<FollowPathActionServer>(new FollowPathActionServer(manager_));

  tf_subscriber_ = std::shared_ptr<TFSubscriber>(new TFSubscriber(manager_, follow_path_server_));

  exception_subscriber_ = std::shared_ptr<ExceptionSubscriber>(new ExceptionSubscriber(manager_));

  navigation_path_service_server_ =
      std::shared_ptr<NavigationPathServiceServer>(new NavigationPathServiceServer(manager_));

  debug_sub_ = std::shared_ptr<DeveloperModeSubscriber>(new DeveloperModeSubscriber(manager_));

  executor_->add_node(path_publisher_->get_node_base_interface());
  executor_->add_node(tf_subscriber_->get_node_base_interface());
  executor_->add_node(follow_path_server_->get_node_base_interface());
  executor_->add_node(exception_subscriber_->get_node_base_interface());
  executor_->add_node(odom_sub_->get_node_base_interface());
  executor_->add_node(laser_scan_sub_->get_node_base_interface());
  executor_->add_node(map_sub_->get_node_base_interface());
  executor_->add_node(vel_publisher_->get_node_base_interface());
  executor_->add_node(virtual_path_service_server_->get_node_base_interface());
  executor_->add_node(navigation_path_service_server_->get_node_base_interface());
  executor_->add_node(debug_sub_->get_node_base_interface());
}
}  // namespace navigation
}  // namespace qrb_ros
