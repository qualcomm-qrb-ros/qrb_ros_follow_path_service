/*
 * Copyright (c) 2025 Qualcomm Innovation Center, Inc. All rights reserved.
 * SPDX-License-Identifier: BSD-3-Clause-Clear
 */

#ifndef QRB_ROS_NAVIGATION__TWIST_PUBLISHER_HPP_
#define QRB_ROS_NAVIGATION__TWIST_PUBLISHER_HPP_

#include "manager/follow_path_manager.hpp"
#include "manager/ros_common.hpp"

using namespace qrb::navigation;

namespace qrb_ros
{
namespace navigation
{

/**
 * @class navigator::TwistPublisher
 * @desc Publish twist data to the RobotController
 * data
 */
class TwistPublisher : public rclcpp_lifecycle::LifecycleNode
{
public:
  /**
   * @desc A constructor for navigator::TwistPublisher
   */
  TwistPublisher(std::shared_ptr<FollowPathManager> & manager);

  /**
   * @desc A destructor for navigator::TwistPublisher class
   */
  ~TwistPublisher();

  /**
   * @desc Send the velocity data to robot
   * @param twist velocity in free space broken into its linear and angular
   * parts
   */
  void send_velocity(twist_vel & vel);

private:
  LifecycleNodeInterface::CallbackReturn on_configure(const rclcpp_lifecycle::State &) override;
  LifecycleNodeInterface::CallbackReturn on_activate(const rclcpp_lifecycle::State &) override;
  LifecycleNodeInterface::CallbackReturn on_deactivate(const rclcpp_lifecycle::State &) override;
  LifecycleNodeInterface::CallbackReturn on_cleanup(const rclcpp_lifecycle::State &) override;
  LifecycleNodeInterface::CallbackReturn on_shutdown(const rclcpp_lifecycle::State &) override;

  void init_publisher();
  void deinit_publisher();

  rclcpp_lifecycle::LifecyclePublisher<geometry_msgs::msg::Twist>::SharedPtr pub_;
  publish_twist_func_t publish_twist_cb_;
  std::shared_ptr<FollowPathManager> manager_;
  std::mutex vel_mtx_;

  rclcpp::Logger logger_{ rclcpp::get_logger("twist_publisher") };
};

}  // namespace navigation
}  // namespace qrb_ros
#endif  // QRB_ROS_NAVIGATION__TWIST_PUBLISHER_HPP_
