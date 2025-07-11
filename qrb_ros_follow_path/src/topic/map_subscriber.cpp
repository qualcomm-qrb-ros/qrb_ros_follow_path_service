/*
 * Copyright (c) 2025 Qualcomm Innovation Center, Inc. All rights reserved.
 * SPDX-License-Identifier: BSD-3-Clause-Clear
 */

#include "map_subscriber.hpp"

namespace qrb_ros
{
namespace navigation
{
MapSubscriber::MapSubscriber(std::shared_ptr<FollowPathManager> & manager,
    std::shared_ptr<LaserScanSubscriber> & laser_scan_sub)
  : LifecycleNode("map_sub"), manager_(manager), laser_scan_sub_(laser_scan_sub)
{
  get_grid_map_cb_ = [&](grid_map & map) { get_grid_map(map); };
  manager_->register_get_grid_map_callback(get_grid_map_cb_);

  tf_buffer_ = std::make_shared<tf2_ros::Buffer>(get_clock());
  tf_listener_ = std::make_shared<tf2_ros::TransformListener>(*tf_buffer_);
}

MapSubscriber::~MapSubscriber()
{
  RCLCPP_DEBUG(logger_, "Destructor MapSubscriber object");
}

LifecycleNodeInterface::CallbackReturn MapSubscriber::on_configure(const rclcpp_lifecycle::State &)
{
  RCLCPP_INFO(logger_, "Configuring");
  init_publisher();
  init_subscriber();
  return LifecycleNodeInterface::CallbackReturn::SUCCESS;
}

LifecycleNodeInterface::CallbackReturn MapSubscriber::on_activate(const rclcpp_lifecycle::State &)
{
  RCLCPP_INFO(logger_, "Activating");
  local_map_pub_->on_activate();
  return LifecycleNodeInterface::CallbackReturn::SUCCESS;
}

LifecycleNodeInterface::CallbackReturn MapSubscriber::on_deactivate(const rclcpp_lifecycle::State &)
{
  RCLCPP_INFO(logger_, "Deactivating");
  local_map_pub_->on_deactivate();
  return LifecycleNodeInterface::CallbackReturn::SUCCESS;
}

LifecycleNodeInterface::CallbackReturn MapSubscriber::on_cleanup(const rclcpp_lifecycle::State &)
{
  RCLCPP_INFO(logger_, "Cleaning up");
  deinit_publisher();
  deinit_subscriber();
  return LifecycleNodeInterface::CallbackReturn::SUCCESS;
}

LifecycleNodeInterface::CallbackReturn MapSubscriber::on_shutdown(const rclcpp_lifecycle::State &)
{
  RCLCPP_INFO(logger_, "Shutting down");
  return LifecycleNodeInterface::CallbackReturn::SUCCESS;
}

void MapSubscriber::init_subscriber()
{
  RCLCPP_INFO(logger_, "init map subsrcier");

  using namespace std::placeholders;
  sub_ = create_subscription<nav_msgs::msg::OccupancyGrid>("/map", rclcpp::SystemDefaultsQoS(),
      std::bind(&MapSubscriber::map_callback, this, std::placeholders::_1));
}

void MapSubscriber::deinit_subscriber()
{
  sub_.reset();
}

void MapSubscriber::init_publisher()
{
  RCLCPP_INFO(logger_, "init map publiser");
  using namespace std::placeholders;
  local_map_pub_ = create_publisher<nav_msgs::msg::OccupancyGrid>("/local_map", 10);
}

void MapSubscriber::deinit_publisher()
{
  local_map_pub_.reset();
}

void MapSubscriber::get_grid_map(grid_map & map)
{
  std::lock_guard<std::mutex> lock(mutex_);
  if (map_ == nullptr) {
    RCLCPP_ERROR(logger_, "map is nullptr");
    return;
  }

  map.resolution = map_->info.resolution;
  map.origin_x = map_->info.origin.position.x;
  map.origin_y = map_->info.origin.position.y;

  map.width = map_->info.width;
  map.height = map_->info.height;
  uint32_t len = map_->info.height * map_->info.width;
  map.data.resize(len);

  for (uint32_t i = 0; i < len; i++) {
    map.data[i] = map_->data[i];
  }

  RCLCPP_INFO(logger_, "get_grid_map(%.2f,%.2f,%.2f,%d,%d,%d)", map.resolution, map.origin_x,
      map.origin_y, map.width, map.height, len);

  // print_map(map_);
}

void MapSubscriber::map_callback(nav_msgs::msg::OccupancyGrid::ConstSharedPtr map)
{
  RCLCPP_DEBUG(logger_, "receive a map callback");
  std::lock_guard<std::mutex> lock(mutex_);

  if (print_map_count_ % 60 == 0) {
    RCLCPP_INFO(logger_, "receive a map");
  }
  print_map_count_++;

  map_ = std::make_shared<nav_msgs::msg::OccupancyGrid>(*map);

  sensor_msgs::msg::LaserScan::SharedPtr scan = std::make_shared<sensor_msgs::msg::LaserScan>();
  bool res = laser_scan_sub_->get_laser_scan_data(scan);
  if (!res) {
    RCLCPP_INFO(logger_, "no laser scan");
    local_map_pub_->publish(*map_);
    return;
  }

  geometry_msgs::msg::TransformStamped laser_to_map;
  try {
    laser_to_map = tf_buffer_->lookupTransform(  //"map", "base_scan",
        "map", "laser", tf2::TimePoint(), tf2::durationFromSec(1.0));
  } catch (tf2::LookupException & ex) {
    RCLCPP_INFO(logger_, "transform from laser to map not ready");
    return;
  }
  process_laser_scan_data(scan, map_, laser_to_map);
  local_map_pub_->publish(*map_);
}

void MapSubscriber::process_laser_scan_data(const sensor_msgs::msg::LaserScan::SharedPtr scan,
    nav_msgs::msg::OccupancyGrid::SharedPtr map,
    const geometry_msgs::msg::TransformStamped & laser_to_base,
    const geometry_msgs::msg::TransformStamped & base_to_map)
{
  float map_resolution = map->info.resolution;
  float map_origin_x = map->info.origin.position.x;
  float map_origin_y = map->info.origin.position.y;
  unsigned int map_width = map->info.width;

  float angle_min = scan->angle_min;
  float angle_increment = scan->angle_increment;

  tf2::Transform tf_laser_scan_to_base;
  tf2::fromMsg(laser_to_base.transform, tf_laser_scan_to_base);
  tf2::Transform tf_base_to_map;
  tf2::fromMsg(base_to_map.transform, tf_base_to_map);

  // laser_scan->base_link->map
  tf2::Transform tf_laser_scan_to_map = tf_base_to_map * tf_laser_scan_to_base;
  for (size_t i = 0; i < scan->ranges.size(); ++i) {
    float range = scan->ranges[i];

    if (std::isinf(range) || std::isnan(range) || (range > scan->range_max) ||
        (range < scan->range_min)) {
      continue;
    }

    // compute obstacle position in laser scan coordinate
    float angle = angle_min + i * angle_increment;
    tf2::Vector3 obstacle_laser_scan_frame(range * cos(angle), range * sin(angle), 0.0);

    // to map coordinate
    tf2::Vector3 obstacle_map_frame = tf_laser_scan_to_map * obstacle_laser_scan_frame;

    // convert to cell in map
    int map_x = static_cast<int>((obstacle_map_frame.x() - map_origin_x) / map_resolution);
    int map_y = static_cast<int>((obstacle_map_frame.y() - map_origin_y) / map_resolution);

    // out of map
    if ((map_x >= 0) && (map_x < static_cast<int>(map->info.width)) &&
        (map_y >= 0 && map_y < static_cast<int>(map->info.height))) {
      size_t index = map_y * map_width + map_x;
      // this cell is obstacle
      map->data[index] = LOCAL_MAP_OBSTACLE;
    }
  }
}

void MapSubscriber::process_laser_scan_data(const sensor_msgs::msg::LaserScan::SharedPtr scan,
    nav_msgs::msg::OccupancyGrid::SharedPtr map,
    const geometry_msgs::msg::TransformStamped & laser_to_map)
{
  float map_resolution = map->info.resolution;
  float map_origin_x = map->info.origin.position.x;
  float map_origin_y = map->info.origin.position.y;
  unsigned int map_width = map->info.width;

  float angle_min = scan->angle_min;
  float angle_increment = scan->angle_increment;

  tf2::Transform tf_laser_scan_to_map;
  tf2::fromMsg(laser_to_map.transform, tf_laser_scan_to_map);

  for (size_t i = 0; i < scan->ranges.size(); ++i) {
    float range = scan->ranges[i];

    if (std::isinf(range) || std::isnan(range) || (range > scan->range_max) ||
        (range < scan->range_min)) {
      continue;
    }

    // compute obstacle position in laser scan coordinate
    float angle = angle_min + i * angle_increment;
    tf2::Vector3 obstacle_laser_scan_frame(range * cos(angle), range * sin(angle), 0.0);

    // to map coordinate
    tf2::Vector3 obstacle_map_frame = tf_laser_scan_to_map * obstacle_laser_scan_frame;

    // convert to cell in map
    int map_x = static_cast<int>((obstacle_map_frame.x() - map_origin_x) / map_resolution);
    int map_y = static_cast<int>((obstacle_map_frame.y() - map_origin_y) / map_resolution);

    // out of map
    if ((map_x >= 0) && (map_x < static_cast<int>(map->info.width)) &&
        (map_y >= 0 && map_y < static_cast<int>(map->info.height))) {
      size_t index = map_y * map_width + map_x;
      // this cell is obstacle
      map->data[index] = LOCAL_MAP_OBSTACLE;
    }
  }
}

void MapSubscriber::print_map(nav_msgs::msg::OccupancyGrid & map)
{
  uint32_t width = map.info.width;
  uint32_t height = map.info.height;
  for (uint32_t j = 0; j < height; j++) {
    std::string str = "";
    for (uint32_t i = 0; i < width; i++) {
      int8_t data = map.data[j * width + i];
      if (data == -1) {
        str.append("U");
      } else if (data == 100) {
        str.append("O");
      } else if (data == 0) {
        str.append("F");
      } else {
        str.append("#");
      }
    }
    RCLCPP_INFO(logger_, "%s", str.c_str());
  }
}
}  // namespace navigation
}  // namespace qrb_ros