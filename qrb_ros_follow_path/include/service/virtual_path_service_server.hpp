/*
 * Copyright (c) 2025 Qualcomm Innovation Center, Inc. All rights reserved.
 * SPDX-License-Identifier: BSD-3-Clause-Clear
 */

#ifndef QRB_ROS_NAVIGATION__VIRTUAL_PATH_SERVICE_SERVER_HPP_
#define QRB_ROS_NAVIGATION__VIRTUAL_PATH_SERVICE_SERVER_HPP_

#include "follow_path_manager.hpp"
#include "manager/ros_common.hpp"

using namespace qrb::navigation;

namespace qrb_ros
{
namespace navigation
{

/**
 * @class navigation_controller::VirtualPathServiceServer
 * @desc The VirtualPathServiceServer create service server to receive requests
 * from amr_daemon.
 */
class VirtualPathServiceServer : public rclcpp_lifecycle::LifecycleNode
{
public:
  VirtualPathServiceServer(std::shared_ptr<FollowPathManager> & service);

  ~VirtualPathServiceServer();

private:
  LifecycleNodeInterface::CallbackReturn on_configure(const rclcpp_lifecycle::State &) override;
  LifecycleNodeInterface::CallbackReturn on_activate(const rclcpp_lifecycle::State &) override;
  LifecycleNodeInterface::CallbackReturn on_deactivate(const rclcpp_lifecycle::State &) override;
  LifecycleNodeInterface::CallbackReturn on_cleanup(const rclcpp_lifecycle::State &) override;
  LifecycleNodeInterface::CallbackReturn on_shutdown(const rclcpp_lifecycle::State &) override;

  void init_server();
  void deinit_server();

  void receive_request(const std::shared_ptr<VirtualPathService::Request> request,
      std::shared_ptr<VirtualPathService::Response> response);

  std::shared_ptr<rclcpp::Service<VirtualPathService>> service_;
  std::shared_ptr<FollowPathManager> manager_;

  rclcpp::Logger logger_{ rclcpp::get_logger("virtual_path_service_server") };
};

}  // namespace navigation
}  // namespace qrb_ros
#endif  // QRB_ROS_NAVIGATION__VIRTUAL_PATH_SERVICE_SERVER_HPP_