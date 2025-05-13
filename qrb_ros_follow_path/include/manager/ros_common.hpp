/*
 * Copyright (c) 2025 Qualcomm Innovation Center, Inc. All rights reserved.
 * SPDX-License-Identifier: BSD-3-Clause-Clear
 */

#ifndef QRB_NAVIGATION__ROS_COMMON_HPP
#define QRB_NAVIGATION__ROS_COMMON_HPP

#include "rclcpp/rclcpp.hpp"
#include "rclcpp_action/rclcpp_action.hpp"
#include "rclcpp_lifecycle/lifecycle_node.hpp"
#include "lifecycle_msgs/msg/transition.hpp"
#include "lifecycle_msgs/msg/state.hpp"
#include "geometry_msgs/msg/pose_stamped.hpp"
#include "geometry_msgs/msg/twist.hpp"
#include "nav_2d_msgs/msg/twist2_d.hpp"
#include "nav_2d_msgs/msg/twist2_d_stamped.hpp"
#include "nav_msgs/msg/path.hpp"
#include "nav_msgs/msg/odometry.hpp"
#include "sensor_msgs/msg/laser_scan.hpp"
#include "nav_msgs/msg/occupancy_grid.hpp"
#include "geometry_msgs/msg/transform_stamped.hpp"
#include "std_msgs/msg/int16.hpp"

#include "tf2_ros/buffer.h"
#include "tf2/exceptions.h"
#include "tf2/convert.h"
#include "tf2/LinearMath/Quaternion.h"
#include "tf2_geometry_msgs/tf2_geometry_msgs.hpp"
#include "tf2_ros/transform_listener.h"

#include "qrb_ros_amr_msgs/srv/sub_cmd.hpp"
#include "qrb_ros_navigation_msgs/action/follow_path.hpp"
#include "qrb_ros_navigation_msgs/srv/virtual_path.hpp"
#include "qrb_ros_robot_base_msgs/msg/exception.hpp"

using Path = nav_msgs::msg::Path;
using PoseStamped = geometry_msgs::msg::PoseStamped;
using VirtualPathService = qrb_ros_navigation_msgs::srv::VirtualPath;
using Exception = qrb_ros_robot_base_msgs::msg::Exception;

using rclcpp_lifecycle::LifecycleNode;
using rclcpp_lifecycle::node_interfaces::LifecycleNodeInterface;

#endif  // QRB_NAVIGATION__ROS_COMMON_HPP