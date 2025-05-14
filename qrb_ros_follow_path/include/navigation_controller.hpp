/*
 * Copyright (c) 2025 Qualcomm Innovation Center, Inc. All rights reserved.
 * SPDX-License-Identifier: BSD-3-Clause-Clear
 */

#ifndef QRB_ROS_NAVIGATION__NAVIGATION_CONTROLLER_HPP_
#define QRB_ROS_NAVIGATION__NAVIGATION_CONTROLLER_HPP_

#include "action/follow_path_action_server.hpp"
#include "service/navigation_path_service_server.hpp"
#include "service/virtual_path_service_server.hpp"
#include "topic/exception_subscriber.hpp"
#include "topic/developer_mode_subscriber.hpp"
#include "topic/odom_subscriber.hpp"
#include "topic/laser_scan_subscriber.hpp"
#include "topic/map_subscriber.hpp"
#include "topic/path_publisher.hpp"
#include "topic/tf_subscriber.hpp"
#include "topic/twist_publisher.hpp"
#include "follow_path_manager.hpp"
#include <string.h>

namespace qrb_ros
{
namespace navigation
{
/**
 * @class navigation_controller::NavigationController
 * @desc The NavigationController create nodes to control the Navigation.
 */
class NavigationController : public rclcpp::Node
{
public:
  std::shared_ptr<rclcpp::executors::MultiThreadedExecutor> executor_;

  /**
   * @desc A constructor for NavigationController
   */
  NavigationController();

  /**
   * @desc A destructor for NavigationController
   */
  ~NavigationController();

private:
  void init_nodes();

  std::mutex request_mutex_;
  std::shared_ptr<FollowPathManager> manager_;
  std::shared_ptr<TFSubscriber> tf_subscriber_;
  std::shared_ptr<FollowPathActionServer> follow_path_server_;
  std::shared_ptr<ExceptionSubscriber> exception_subscriber_;
  std::shared_ptr<OdomSubscriber> odom_sub_;
  std::shared_ptr<LaserScanSubscriber> laser_scan_sub_;
  std::shared_ptr<MapSubscriber> map_sub_;
  std::shared_ptr<TwistPublisher> vel_publisher_;
  std::shared_ptr<PathPublisher> path_publisher_;
  std::shared_ptr<VirtualPathServiceServer> virtual_path_service_server_;
  std::shared_ptr<NavigationPathServiceServer> navigation_path_service_server_;
  std::shared_ptr<DeveloperModeSubscriber> debug_sub_;

  rclcpp::Logger logger_{ rclcpp::get_logger("navigation_controller") };
};

}  // namespace navigation
}  // namespace qrb_ros
#endif  // QRB_ROS_NAVIGATION__NAVIGATION_CONTROLLER_HPP_