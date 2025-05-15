/*
 * Copyright (c) 2025 Qualcomm Innovation Center, Inc. All rights reserved.
 * SPDX-License-Identifier: BSD-3-Clause-Clear
 */

#ifndef QRB_ROS_NAVIGATION__MAP_SUBSCRIBER_HPP_
#define QRB_ROS_NAVIGATION__MAP_SUBSCRIBER_HPP_

#include "follow_path_manager.hpp"
#include "manager/ros_common.hpp"
#include "laser_scan_subscriber.hpp"

using namespace qrb::navigation;

namespace qrb_ros
{
namespace navigation
{

/**
 * @class navigator::MapSubscriber
 * @desc Subscribe the map data to get the grid map information
 * data
 */
class MapSubscriber : public rclcpp_lifecycle::LifecycleNode
{
public:
  /**
   * @desc A constructor for navigator::MapSubscriber
   */
  MapSubscriber(std::shared_ptr<FollowPathManager> & manager,
      std::shared_ptr<LaserScanSubscriber> & laser_scan_sub);

  /**
   * @desc A destructor for navigator::MapSubscriber class
   */
  ~MapSubscriber();

  void get_grid_map(grid_map & map);

private:
  LifecycleNodeInterface::CallbackReturn on_configure(const rclcpp_lifecycle::State &) override;
  LifecycleNodeInterface::CallbackReturn on_activate(const rclcpp_lifecycle::State &) override;
  LifecycleNodeInterface::CallbackReturn on_deactivate(const rclcpp_lifecycle::State &) override;
  LifecycleNodeInterface::CallbackReturn on_cleanup(const rclcpp_lifecycle::State &) override;
  LifecycleNodeInterface::CallbackReturn on_shutdown(const rclcpp_lifecycle::State &) override;

  void init_subscriber();
  void deinit_subscriber();
  void init_publisher();
  void deinit_publisher();

  void map_callback(nav_msgs::msg::OccupancyGrid::ConstSharedPtr map);

  void process_laser_scan_data(const sensor_msgs::msg::LaserScan::SharedPtr scan,
      nav_msgs::msg::OccupancyGrid::SharedPtr map,
      const geometry_msgs::msg::TransformStamped & laser_to_base,
      const geometry_msgs::msg::TransformStamped & base_to_map);

  void process_laser_scan_data(const sensor_msgs::msg::LaserScan::SharedPtr scan,
      nav_msgs::msg::OccupancyGrid::SharedPtr map,
      const geometry_msgs::msg::TransformStamped & laser_to_map);

  void print_map(nav_msgs::msg::OccupancyGrid & map);

  rclcpp::Subscription<nav_msgs::msg::OccupancyGrid>::SharedPtr sub_;
  rclcpp_lifecycle::LifecyclePublisher<nav_msgs::msg::OccupancyGrid>::SharedPtr local_map_pub_;
  std::mutex mutex_;
  get_grid_map_func_t get_grid_map_cb_;
  std::shared_ptr<FollowPathManager> manager_;
  std::shared_ptr<LaserScanSubscriber> laser_scan_sub_;
  uint64_t print_map_count_;
  nav_msgs::msg::OccupancyGrid::SharedPtr map_;
  std::shared_ptr<tf2_ros::Buffer> tf_buffer_;
  std::shared_ptr<tf2_ros::TransformListener> tf_listener_;

  rclcpp::Logger logger_{ rclcpp::get_logger("map_subscriber") };
};

}  // namespace navigation
}  // namespace qrb_ros
#endif  // QRB_ROS_NAVIGATION__MAP_SUBSCRIBER_HPP_
