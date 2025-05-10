/*
 * Copyright (c) 2025 Qualcomm Innovation Center, Inc. All rights reserved.
 * SPDX-License-Identifier: BSD-3-Clause-Clear
 */

#include "service/virtual_path_service_server.hpp"

constexpr char const * node_name = "virtual_path_service_server";
constexpr char const * service_name = "virtual_path";

using namespace std::placeholders;
using namespace std;

namespace qrb_ros
{
namespace navigation
{
VirtualPathServiceServer::VirtualPathServiceServer(std::shared_ptr<FollowPathManager> & manager)
  : LifecycleNode(node_name), manager_(manager)
{
  RCLCPP_INFO(logger_, "Creating");
}

VirtualPathServiceServer::~VirtualPathServiceServer()
{
  RCLCPP_INFO(logger_, "Destroying");
}

LifecycleNodeInterface::CallbackReturn VirtualPathServiceServer::on_configure(
    const rclcpp_lifecycle::State &)
{
  RCLCPP_INFO(logger_, "Configuring");
  init_server();
  return LifecycleNodeInterface::CallbackReturn::SUCCESS;
}

LifecycleNodeInterface::CallbackReturn VirtualPathServiceServer::on_activate(
    const rclcpp_lifecycle::State &)
{
  RCLCPP_INFO(logger_, "Activating");
  return LifecycleNodeInterface::CallbackReturn::SUCCESS;
}

LifecycleNodeInterface::CallbackReturn VirtualPathServiceServer::on_deactivate(
    const rclcpp_lifecycle::State &)
{
  RCLCPP_INFO(logger_, "Deactivating");
  return LifecycleNodeInterface::CallbackReturn::SUCCESS;
}

LifecycleNodeInterface::CallbackReturn VirtualPathServiceServer::on_cleanup(
    const rclcpp_lifecycle::State &)
{
  RCLCPP_INFO(logger_, "Cleaning up");
  deinit_server();
  return LifecycleNodeInterface::CallbackReturn::SUCCESS;
}

LifecycleNodeInterface::CallbackReturn VirtualPathServiceServer::on_shutdown(
    const rclcpp_lifecycle::State &)
{
  RCLCPP_INFO(logger_, "Shutting down");
  return LifecycleNodeInterface::CallbackReturn::SUCCESS;
}

void VirtualPathServiceServer::init_server()
{
  service_ = this->create_service<VirtualPathService>(
      service_name, std::bind(&VirtualPathServiceServer::receive_request, this, _1, _2));
}

void VirtualPathServiceServer::deinit_server()
{
  service_.reset();
}

void VirtualPathServiceServer::receive_request(
    const std::shared_ptr<VirtualPathService::Request> request,
    std::shared_ptr<VirtualPathService::Response> response)
{
  uint8_t api_id = request->api_id;
  RCLCPP_INFO(logger_, "api id: %s", StringUtil::api_id_to_string(api_id).c_str());

  switch ((APIID)api_id) {
    case APIID::AddWaypoint: {
      point_2d waypoint;
      waypoint.x = request->waypoint.x;
      waypoint.y = request->waypoint.y;
      waypoint.angle = request->waypoint.z;
      uint32_t id = manager_->add_waypoint(waypoint);
      RCLCPP_INFO(logger_, "add waypoint(%d)", id);
      response->result = id;
      response->waypoint_id = id;
      break;
    }
    case APIID::RemoveWaypoint: {
      uint32_t id = request->waypoint_id;
      bool res = manager_->remove_waypoint(id);
      RCLCPP_INFO(logger_, "remove waypoint(%d) %s", id, res ? "success" : "failed");
      response->result = uint8_t(res ? BoolValue::TRUE : BoolValue::FALSE);
      break;
    }
    case APIID::GetWaypointIDList: {
      vector<uint32_t> list;
      bool res = manager_->get_waypoint_id_list(list);
      RCLCPP_INFO(logger_, "get waypoint id list %s", res ? "success" : "failed");
      response->result = uint8_t(res ? BoolValue::TRUE : BoolValue::FALSE);
      uint32_t len = list.size();
      response->waypoint_id_list.reserve(len);
      response->waypoint_id_list.resize(len);
      for (uint32_t i = 0; i < len; i++) {
        response->waypoint_id_list[i] = list[i];
      }
      break;
    }
    case APIID::GetWaypoint: {
      point_2d waypoint;
      uint32_t id = request->waypoint_id;
      bool res = manager_->get_waypoint(id, waypoint);
      response->result = uint8_t(res ? BoolValue::TRUE : BoolValue::FALSE);
      response->waypoint.x = waypoint.x;
      response->waypoint.y = waypoint.y;
      response->waypoint.z = waypoint.angle;
      RCLCPP_INFO(logger_, "get waypoint(id=%d)(%.2f,%.2f,%.2f) %s", id, waypoint.x, waypoint.y,
          waypoint.angle, res ? "success" : "failed");
      break;
    }
    case APIID::AddVirtualPath: {
      vector<uint32_t> adjacent_waypoints;
      uint32_t id = request->waypoint_id;
      uint32_t len = request->adjacent_waypoints.size();
      for (uint32_t i = 0; i < len; i++) {
        adjacent_waypoints.push_back(request->adjacent_waypoints[i]);
      }
      bool res = manager_->add_virtual_path(id, adjacent_waypoints);
      RCLCPP_INFO(logger_, "add virtual path %s", res ? "success" : "failed");
      response->result = uint8_t(res ? BoolValue::TRUE : BoolValue::FALSE);
      break;
    }
    case APIID::RemoveVirtualPath: {
      vector<uint32_t> adjacent_waypoints;
      uint32_t id = request->waypoint_id;
      uint32_t len = request->adjacent_waypoints.size();
      for (uint32_t i = 0; i < len; i++) {
        adjacent_waypoints.push_back(request->adjacent_waypoints[i]);
      }
      bool res = manager_->remove_virtual_path(id, adjacent_waypoints);
      RCLCPP_INFO(logger_, "remove virtual path %s", res ? "success" : "failed");
      response->result = uint8_t(res ? BoolValue::TRUE : BoolValue::FALSE);
      break;
    }
    case APIID::GetVirtualPath: {
      vector<uint32_t> adjacent_waypoints;
      uint32_t id = request->waypoint_id;
      bool res = manager_->get_virtual_path(id, adjacent_waypoints);
      RCLCPP_INFO(logger_, "get virtual path %s", res ? "success" : "failed");
      response->result = uint8_t(res ? BoolValue::TRUE : BoolValue::FALSE);
      uint32_t len = adjacent_waypoints.size();
      response->adjacent_waypoints.reserve(len);
      response->adjacent_waypoints.resize(len);
      for (uint32_t i = 0; i < len; i++) {
        response->adjacent_waypoints[i] = adjacent_waypoints[i];
      }
      break;
    }
    case APIID::SetBypassing: {
      bool bypassing = request->bypassing;
      manager_->set_bypassing_obstacle(bypassing);
      RCLCPP_INFO(logger_, "set bypassing obstacle(%d)", bypassing);
      response->result = uint8_t(true ? BoolValue::TRUE : BoolValue::FALSE);
      break;
    }
    case APIID::RemoveWaypointAndVirtualPath: {
      bool res = manager_->remove_waypoint_and_virtual_path();
      RCLCPP_INFO(logger_, "remove waypoint and virtual path %s", res ? "success" : "failed");
      response->result = uint8_t(res ? BoolValue::TRUE : BoolValue::FALSE);
      break;
    }
    default:
      RCLCPP_ERROR(logger_, "api id is error");
      break;
  }
}
}  // namespace navigation
}  // namespace qrb_ros