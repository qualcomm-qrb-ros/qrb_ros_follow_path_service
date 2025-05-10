/*
 * Copyright (c) 2025 Qualcomm Innovation Center, Inc. All rights reserved.
 * SPDX-License-Identifier: BSD-3-Clause-Clear
 */

#ifndef QRB_ROS_NAVIGATION_NAVIGATION_PATH_SERVICE_SERVER_HPP_
#define QRB_ROS_NAVIGATION_NAVIGATION_PATH_SERVICE_SERVER_HPP_

#include "manager/follow_path_manager.hpp"
#include "qrb_ros_navigation_msgs/srv/compute_follow_path.hpp"

using namespace qrb::navigation;

using ComputeFollowPath = qrb_ros_navigation_msgs::srv::ComputeFollowPath;

namespace qrb_ros
{
namespace navigation
{

class NavigationPathServiceServer : public rclcpp_lifecycle::LifecycleNode
{
public:
  NavigationPathServiceServer(std::shared_ptr<FollowPathManager> & service);

  ~NavigationPathServiceServer();

private:
  LifecycleNodeInterface::CallbackReturn on_configure(const rclcpp_lifecycle::State &) override;
  LifecycleNodeInterface::CallbackReturn on_activate(const rclcpp_lifecycle::State &) override;
  LifecycleNodeInterface::CallbackReturn on_deactivate(const rclcpp_lifecycle::State &) override;
  LifecycleNodeInterface::CallbackReturn on_cleanup(const rclcpp_lifecycle::State &) override;
  LifecycleNodeInterface::CallbackReturn on_shutdown(const rclcpp_lifecycle::State &) override;

  void deinit_server();
  void init_server();

  void receive_follow_path_request(const std::shared_ptr<ComputeFollowPath::Request> request,
      std::shared_ptr<ComputeFollowPath::Response> response);

  void print_follow_path(std::vector<uint32_t> & path);

  std::shared_ptr<rclcpp::Service<ComputeFollowPath>> server_;
  std::shared_ptr<FollowPathManager> manager_;

  rclcpp::Logger logger_{ rclcpp::get_logger("navigation_path_service_server") };
};

}  // namespace navigation
}  // namespace qrb_ros
#endif  // QRB_ROS_NAVIGATION_NAVIGATION_PATH_SERVICE_SERVER_HPP_