/*
 * Copyright (c) 2025 Qualcomm Innovation Center, Inc. All rights reserved.
 * SPDX-License-Identifier: BSD-3-Clause-Clear
 */

#ifndef QRB_ROS_NAVIGATION__ODOM_SUBSCRIBER_HPP_
#define QRB_ROS_NAVIGATION__ODOM_SUBSCRIBER_HPP_

#include "manager/follow_path_manager.hpp"

using namespace qrb::navigation;

namespace qrb_ros
{
namespace navigation
{

/**
 * @class navigator::OdomSubscriber
 * @desc Subscribe the odom data to get the velocity of robot
 * data
 */
class OdomSubscriber : public rclcpp_lifecycle::LifecycleNode
{
public:
  /**
   * @desc A constructor for navigator::OdomSubscriber
   */
  explicit OdomSubscriber(std::shared_ptr<FollowPathManager> & manager);

  /**
   * @desc A destructor for navigator::OdomSubscriber class
   */
  ~OdomSubscriber();

  /**
   * @desc Get the current velocity of robot
   * @param twist velocity in free space broken into its linear and angular
   * parts
   */
  void get_robot_velocity(twist_vel & twist);

private:
  LifecycleNodeInterface::CallbackReturn on_configure(const rclcpp_lifecycle::State &) override;
  LifecycleNodeInterface::CallbackReturn on_activate(const rclcpp_lifecycle::State &) override;
  LifecycleNodeInterface::CallbackReturn on_deactivate(const rclcpp_lifecycle::State &) override;
  LifecycleNodeInterface::CallbackReturn on_cleanup(const rclcpp_lifecycle::State &) override;
  LifecycleNodeInterface::CallbackReturn on_shutdown(const rclcpp_lifecycle::State &) override;

  void init_subscriber();
  void deinit_subscriber();

  void odom_callback(const nav_msgs::msg::Odometry::SharedPtr msg);

  rclcpp::Subscription<nav_msgs::msg::Odometry>::SharedPtr sub_;
  nav_2d_msgs::msg::Twist2DStamped odom_velocity_;
  std::mutex mutex_;
  get_robot_velocity_func_t get_robot_velocity_cb_;
  std::shared_ptr<FollowPathManager> manager_;

  rclcpp::Logger logger_{ rclcpp::get_logger("odom_subscriber") };
};

}  // namespace navigation
}  // namespace qrb_ros
#endif  // QRB_ROS_NAVIGATION__ODOM_SUBSCRIBER_HPP_
