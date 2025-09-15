/*
 * Copyright (c) 2025 Qualcomm Innovation Center, Inc. All rights reserved.
 * SPDX-License-Identifier: BSD-3-Clause-Clear
 */

#ifndef QRB_ROS_NAVIGATION__LASER_SCAN_SUBSCRIBER_HPP_
#define QRB_ROS_NAVIGATION__LASER_SCAN_SUBSCRIBER_HPP_

#include "manager/follow_path_manager.hpp"
#include "manager/ros_common.hpp"

using namespace qrb::navigation;

namespace qrb_ros
{
namespace navigation
{

/**
 * @class navigator::LaserScanSubscriber
 * @desc Subscribe the odom data to get the velocity of robot
 * data
 */
class LaserScanSubscriber : public rclcpp_lifecycle::LifecycleNode
{
public:
  /**
   * @desc A constructor for navigator::LaserScanSubscriber
   */
  explicit LaserScanSubscriber(std::shared_ptr<FollowPathManager> & manager);

  /**
   * @desc A destructor for navigator::LaserScanSubscriber class
   */
  ~LaserScanSubscriber();

  bool get_laser_scan_data(sensor_msgs::msg::LaserScan::SharedPtr & scan);

private:
  LifecycleNodeInterface::CallbackReturn on_configure(const rclcpp_lifecycle::State &) override;
  LifecycleNodeInterface::CallbackReturn on_activate(const rclcpp_lifecycle::State &) override;
  LifecycleNodeInterface::CallbackReturn on_deactivate(const rclcpp_lifecycle::State &) override;
  LifecycleNodeInterface::CallbackReturn on_cleanup(const rclcpp_lifecycle::State &) override;
  LifecycleNodeInterface::CallbackReturn on_shutdown(const rclcpp_lifecycle::State &) override;

  void init_subscriber();
  void deinit_subscriber();

  void laser_callback(sensor_msgs::msg::LaserScan::ConstSharedPtr scan);
  rclcpp::Subscription<sensor_msgs::msg::LaserScan>::SharedPtr sub_;
  std::shared_ptr<FollowPathManager> manager_;
  sensor_msgs::msg::LaserScan::SharedPtr scan_;
  std::mutex mutex_;
  std::shared_ptr<tf2_ros::Buffer> tf_buffer_;
  std::shared_ptr<tf2_ros::TransformListener> tf_listener_;
  uint64_t print_laser_count_;
  bool active_;
  rclcpp::Logger logger_{ rclcpp::get_logger("laser_scan_subscriber") };
};

}  // namespace navigation
}  // namespace qrb_ros
#endif  // QRB_ROS_NAVIGATION__LASER_SCAN_SUBSCRIBER_HPP_
