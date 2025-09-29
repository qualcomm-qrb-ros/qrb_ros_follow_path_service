/*
 * Copyright (c) 2025 Qualcomm Innovation Center, Inc. All rights reserved.
 * SPDX-License-Identifier: BSD-3-Clause-Clear
 */

#include "service/navigation_path_service_server.hpp"

constexpr char const * node_name = "navigation_path_service_server";
constexpr char const * service_name = "compute_follow_path";

using namespace std::placeholders;
using namespace std;

namespace qrb_ros
{
namespace navigation
{
NavigationPathServiceServer::NavigationPathServiceServer(
    std::shared_ptr<FollowPathManager> & manager)
  : LifecycleNode(node_name), manager_(manager)
{
  RCLCPP_INFO(logger_, "Creating");
}

NavigationPathServiceServer::~NavigationPathServiceServer()
{
  RCLCPP_INFO(logger_, "Destroying");
}

LifecycleNodeInterface::CallbackReturn NavigationPathServiceServer::on_configure(
    const rclcpp_lifecycle::State &)
{
  RCLCPP_INFO(logger_, "Configuring");
  init_server();
  return LifecycleNodeInterface::CallbackReturn::SUCCESS;
}

LifecycleNodeInterface::CallbackReturn NavigationPathServiceServer::on_activate(
    const rclcpp_lifecycle::State &)
{
  RCLCPP_INFO(logger_, "Activating");
  return LifecycleNodeInterface::CallbackReturn::SUCCESS;
}

LifecycleNodeInterface::CallbackReturn NavigationPathServiceServer::on_deactivate(
    const rclcpp_lifecycle::State &)
{
  RCLCPP_INFO(logger_, "Deactivating");
  return LifecycleNodeInterface::CallbackReturn::SUCCESS;
}

LifecycleNodeInterface::CallbackReturn NavigationPathServiceServer::on_cleanup(
    const rclcpp_lifecycle::State &)
{
  RCLCPP_INFO(logger_, "Cleaning up");
  deinit_server();
  return LifecycleNodeInterface::CallbackReturn::SUCCESS;
}

LifecycleNodeInterface::CallbackReturn NavigationPathServiceServer::on_shutdown(
    const rclcpp_lifecycle::State &)
{
  RCLCPP_INFO(logger_, "Shutting down");
  return LifecycleNodeInterface::CallbackReturn::SUCCESS;
}

void NavigationPathServiceServer::init_server()
{
  server_ = this->create_service<ComputeFollowPath>(service_name,
      std::bind(&NavigationPathServiceServer::receive_follow_path_request, this, _1, _2));
}

void NavigationPathServiceServer::deinit_server()
{
  server_.reset();
}

void NavigationPathServiceServer::receive_follow_path_request(
    const std::shared_ptr<ComputeFollowPath::Request> request,
    std::shared_ptr<ComputeFollowPath::Response> response)
{
  vector<uint32_t> passing_ids;
  nav_msgs::msg::Path path;
  vector<uint32_t> list;
  vector<point_2d> point_list;
  bool res = false;
  bool bypassing = false;

  uint32_t goal = request->goal;
  uint32_t start = request->start;
  bool use_start = request->use_start;
  uint32_t len = request->passing_waypoint_ids.size();
  if (len != 0) {
    for (uint32_t i = 0; i < len; i++) {
      passing_ids.push_back(request->passing_waypoint_ids[i]);
    }
    bypassing = true;
  } else {
    bypassing = false;
  }
  RCLCPP_INFO(logger_, "compute_follow_path, bypassing: %d, start=%d,goal=%d,use_start=%d,len=%d",
      bypassing, start, goal, use_start, len);

  if (bypassing) {
    res = manager_->compute_follow_path(use_start, start, goal, passing_ids, list, point_list);
    RCLCPP_INFO(logger_, "get follow path(passing waypoint) %s", res ? "success" : "failed");
  } else {
    res = manager_->compute_follow_path(use_start, start, goal, list, point_list);
    RCLCPP_INFO(logger_, "get follow path %s", res ? "success" : "failed");
  }

  geometry_msgs::msg::PoseStamped pose;
  auto time = get_clock()->now();
  std_msgs::msg::Header header;
  header.frame_id = "/map";
  header.stamp.sec = time.seconds();
  header.stamp.nanosec = time.nanoseconds();
  path.header = header;
  auto size = point_list.size();
  for (size_t i = 0; i < size; i++) {
    pose.pose.position.x = point_list.at(i).x;
    pose.pose.position.y = point_list.at(i).y;
    pose.header.stamp = header.stamp;
    path.poses.push_back(pose);
  }

  response->result = uint8_t(res ? BoolValue::TRUE : BoolValue::FALSE);
  response->path = path;
  len = list.size();
  response->waypoint_id_list.reserve(len);
  response->waypoint_id_list.resize(len);
  for (uint32_t i = 0; i < len; i++) {
    response->waypoint_id_list[i] = list[i];
  }
  print_follow_path(list);
}

void NavigationPathServiceServer::print_follow_path(std::vector<uint32_t> & path)
{
  string str = "";

  uint32_t len = path.size();
  for (uint32_t i = 0; i < len; i++) {
    uint32_t id = path[i];
    stringstream buf;
    buf << id;
    str.append(buf.str());
    str.append(",");
  }
  RCLCPP_INFO(logger_, "follow_path:%s", str.c_str());
}
}  // namespace navigation
}  // namespace qrb_ros