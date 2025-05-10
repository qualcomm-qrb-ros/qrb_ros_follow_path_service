/*
 * Copyright (c) 2025 Qualcomm Innovation Center, Inc. All rights reserved.
 * SPDX-License-Identifier: BSD-3-Clause-Clear
 */

#ifndef QRB_ROS_NAVIGATION__PATH_PUBLISHER_HPP_
#define QRB_ROS_NAVIGATION__PATH_PUBLISHER_HPP_

#include "manager/follow_path_manager.hpp"

using namespace qrb::navigation;

namespace qrb_ros
{
namespace navigation
{

class PathPublisher : public rclcpp_lifecycle::LifecycleNode
{
public:
  PathPublisher(std::shared_ptr<FollowPathManager> & manager);

  ~PathPublisher();

  void publish_global_path(const std::vector<point_2d> & point_list,
      double & origin_x,
      double & origin_y);

  void publish_real_path(const std::vector<point_2d> & point_list);

private:
  LifecycleNodeInterface::CallbackReturn on_configure(const rclcpp_lifecycle::State &) override;
  LifecycleNodeInterface::CallbackReturn on_activate(const rclcpp_lifecycle::State &) override;
  LifecycleNodeInterface::CallbackReturn on_deactivate(const rclcpp_lifecycle::State &) override;
  LifecycleNodeInterface::CallbackReturn on_cleanup(const rclcpp_lifecycle::State &) override;
  LifecycleNodeInterface::CallbackReturn on_shutdown(const rclcpp_lifecycle::State &) override;

  void init_publisher();
  void deinit_publisher();

  rclcpp_lifecycle::LifecyclePublisher<nav_msgs::msg::Path>::SharedPtr global_pub_;
  rclcpp_lifecycle::LifecyclePublisher<nav_msgs::msg::Path>::SharedPtr real_pub_;
  publish_real_path_func_t publish_real_path_cb_;
  publish_global_path_func_t publish_global_path_cb_;
  std::shared_ptr<FollowPathManager> manager_;

  rclcpp::Logger logger_{ rclcpp::get_logger("path_publisher") };
};

}  // namespace navigation
}  // namespace qrb_ros
#endif  // QRB_ROS_NAVIGATION__PATH_PUBLISHER_HPP_