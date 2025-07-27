/*
 * Copyright (c) 2025 Qualcomm Innovation Center, Inc. All rights reserved.
 * SPDX-License-Identifier: BSD-3-Clause-Clear
 */

#include "action/follow_path_action_server.hpp"

constexpr char const * node_name = "fp_node";
constexpr char const * action_name = "wfollowpath";
constexpr char const * service_name = "follow_path_sub_cmd";

namespace qrb_ros
{
namespace navigation
{

using namespace std::placeholders;

FollowPathActionServer::FollowPathActionServer(std::shared_ptr<FollowPathManager> & manager)
  : LifecycleNode(node_name), manager_(manager)
{
  RCLCPP_INFO(logger_, "Creating");

  navigation_callback_ = [&](uint64_t request_id, bool result) {
    receive_navigation_callback(request_id, result);
  };
  manager_->register_navigation_callback(navigation_callback_);
}

FollowPathActionServer::~FollowPathActionServer()
{
  RCLCPP_INFO(logger_, "Destroying");
}

LifecycleNodeInterface::CallbackReturn FollowPathActionServer::on_configure(
    const rclcpp_lifecycle::State &)
{
  RCLCPP_INFO(logger_, "Configuring");
  init_server();
  return LifecycleNodeInterface::CallbackReturn::SUCCESS;
}

LifecycleNodeInterface::CallbackReturn FollowPathActionServer::on_activate(
    const rclcpp_lifecycle::State &)
{
  RCLCPP_INFO(logger_, "Activating");
  return LifecycleNodeInterface::CallbackReturn::SUCCESS;
}

LifecycleNodeInterface::CallbackReturn FollowPathActionServer::on_deactivate(
    const rclcpp_lifecycle::State &)
{
  RCLCPP_INFO(logger_, "Deactivating");
  return LifecycleNodeInterface::CallbackReturn::SUCCESS;
}

LifecycleNodeInterface::CallbackReturn FollowPathActionServer::on_cleanup(
    const rclcpp_lifecycle::State &)
{
  RCLCPP_INFO(logger_, "Cleaning up");
  deinit_server();
  return LifecycleNodeInterface::CallbackReturn::SUCCESS;
}

LifecycleNodeInterface::CallbackReturn FollowPathActionServer::on_shutdown(
    const rclcpp_lifecycle::State &)
{
  RCLCPP_INFO(logger_, "Shutting down");
  return LifecycleNodeInterface::CallbackReturn::SUCCESS;
}

void FollowPathActionServer::init_server()
{
  action_server_ptr_ = rclcpp_action::create_server<FollowPath>(this, action_name,
      std::bind(&FollowPathActionServer::handle_goal, this, _1, _2),
      std::bind(&FollowPathActionServer::handle_cancel, this, _1),
      std::bind(&FollowPathActionServer::handle_accepted, this, _1));

  service_ = this->create_service<FollowPathSubCmd>(
      service_name, std::bind(&FollowPathActionServer::receive_request, this, _1, _2));
}

void FollowPathActionServer::deinit_server()
{
  action_server_ptr_.reset();

  service_.reset();
}

rclcpp_action::GoalResponse FollowPathActionServer::handle_goal(
    const rclcpp_action::GoalUUID & uuid,
    std::shared_ptr<const FollowPath::Goal> goal)
{
  RCLCPP_INFO(logger_, "Start follow path from amr controller");
  (void)uuid;
  goal_id_ = goal->goal;

  passing_ids_.clear();

  uint32_t len = goal->passing_waypoint_ids.size();
  for (uint32_t i = 0; i < len; i++) {
    passing_ids_.push_back(goal->passing_waypoint_ids[i]);
  }

  RCLCPP_INFO(logger_, "ACCEPT_AND_EXECUTE");
  return rclcpp_action::GoalResponse::ACCEPT_AND_EXECUTE;
}

void FollowPathActionServer::handle_accepted(
    const std::shared_ptr<GoalHandleFollowPath> goal_handle)
{
  server_global_handle_ = goal_handle;

  rclcpp_action::GoalUUID uuid = goal_handle->get_goal_id();
  uint64_t request_id = request_id_map_[uuid];
  RCLCPP_INFO(logger_, "handle accepted, request_id=%ld, uuid=%d", request_id, uuid);
  handle_map_.insert(
      std::pair<uint64_t, std::shared_ptr<GoalHandleFollowPath>>(request_id, goal_handle));

  if (manager_ != nullptr) {
    uint64_t request_id = manager_->request_follow_path(goal_id_, passing_ids_);
    if (request_id != 0) {
      RCLCPP_INFO(logger_, "request follow path, request_id=%ld, uuid=%d", request_id, uuid);
      navigating_ = true;
      request_id_map_.insert(std::pair<rclcpp_action::GoalUUID, uint64_t>(uuid, request_id));
    } else {
      auto action_result = std::make_shared<FollowPath::Result>();
      goal_handle->abort(action_result);
      RCLCPP_INFO(logger_, "request follow path failed");
    }
  }
}

rclcpp_action::CancelResponse FollowPathActionServer::handle_cancel(
    const std::shared_ptr<GoalHandleFollowPath> goal_handle)
{
  RCLCPP_INFO(logger_, "Received request to cancel goal");
  (void)goal_handle;

  if (manager_ != nullptr) {
    manager_->request_stop_follow_path();
    return rclcpp_action::CancelResponse::ACCEPT;
  }
  return rclcpp_action::CancelResponse::REJECT;
}

void FollowPathActionServer::update_current_pose(PoseStamped & pose)
{
  if (!navigating_) {
    RCLCPP_DEBUG(logger_, "current is not navigating");
    return;
  }

  if (!is_pose_changed(pose)) {
    RCLCPP_INFO(logger_, "Ingore pose update due to no update");
    return;
  }

  // feedback
  uint32_t len = handle_map_.size();
  if (len != 0) {
    auto feedback = std::make_shared<FollowPath::Feedback>();
    feedback->current_pose = pose;
    feedback->passing_waypoint_id = get_passing_waypoint(pose);
    feedback->distance_to_goal = get_distance_to_goal(pose, feedback->passing_waypoint_id);

    std::shared_ptr<GoalHandleFollowPath> handle;

    RCLCPP_DEBUG(logger_, "handle_map_ size = %d", len);

    for (auto it : handle_map_) {
      handle = it.second;
      if (handle != nullptr && !(handle->is_canceling())) {
        handle->publish_feedback(feedback);
        RCLCPP_INFO(this->get_logger(), "feedback current pose(%.2f,%.2f), passing_waypint=%d",
            pose.pose.position.x, pose.pose.position.y, feedback->passing_waypoint_id);
      }
    }
  }
}

bool FollowPathActionServer::is_pose_changed(PoseStamped & pose)
{
  bool is_changed = true;
  if (current_pose_.pose.position.x == pose.pose.position.x &&
      current_pose_.pose.position.y == pose.pose.position.y &&
      current_pose_.pose.position.z == pose.pose.position.z &&
      current_pose_.pose.orientation.x == pose.pose.orientation.x &&
      current_pose_.pose.orientation.y == pose.pose.orientation.y &&
      current_pose_.pose.orientation.z == pose.pose.orientation.z &&
      current_pose_.pose.orientation.w == pose.pose.orientation.w) {
    is_changed = false;
  }

  current_pose_.pose.position.x = pose.pose.position.x;
  current_pose_.pose.position.y = pose.pose.position.y;
  current_pose_.pose.position.z = pose.pose.position.z;
  current_pose_.pose.orientation.x = pose.pose.orientation.x;
  current_pose_.pose.orientation.y = pose.pose.orientation.y;
  current_pose_.pose.orientation.z = pose.pose.orientation.z;
  current_pose_.pose.orientation.w = pose.pose.orientation.w;
  current_pose_.header.stamp = pose.header.stamp;
  current_pose_.header.frame_id = pose.header.frame_id;

  return is_changed;
}

void FollowPathActionServer::receive_navigation_callback(uint64_t request_id, bool result)
{
  RCLCPP_INFO(logger_, "receive_navigation_callback request_id:%ld, %d", request_id, result);
  if (!rclcpp::ok()) {
    return;
  }

  std::shared_ptr<GoalHandleFollowPath> handle = handle_map_[request_id];

  auto action_result = std::make_shared<FollowPath::Result>();
  if (handle == nullptr) {
    RCLCPP_INFO(logger_, "get server global handle again");
    handle = server_global_handle_;
    if (handle == nullptr) {
      RCLCPP_INFO(logger_, "server global handle is nullptr");
      return;
    }
  }

  if (result) {
    action_result->result = true;
    handle->succeed(action_result);
    RCLCPP_INFO(logger_, "Goal succeeded");
  } else {
    if (handle->is_canceling()) {
      action_result->result = false;
      handle->canceled(action_result);
      RCLCPP_INFO(logger_, "Goal canceled");
    } else {
      action_result->result = false;
      handle->abort(action_result);
      RCLCPP_INFO(logger_, "Goal abort");
    }
  }

  request_id_map_.erase(handle->get_goal_id());
  handle_map_.erase(request_id);
  navigating_ = false;

  handle = nullptr;
  server_global_handle_ = nullptr;
}

uint32_t FollowPathActionServer::get_passing_waypoint(PoseStamped & pose)
{
  point_2d current_point;
  convert_pose_to_2d_point(pose, current_point);

  uint32_t passing_id = get_passing_waypoint_id(current_point);
  return passing_id;
}

float FollowPathActionServer::get_distance_to_goal(PoseStamped & pose, uint32_t passing_id)
{
  point_2d current_point;
  convert_pose_to_2d_point(pose, current_point);

  float dist = manager_->get_distance_to_goal(current_point, passing_id);
  return dist;
}

void FollowPathActionServer::convert_pose_to_2d_point(PoseStamped & pose, point_2d & point)
{
  point.x = pose.pose.position.x;
  point.y = pose.pose.position.y;
  tf2::Quaternion quat(pose.pose.orientation.x, pose.pose.orientation.y, pose.pose.orientation.z,
      pose.pose.orientation.w);
  double roll, pitch, yaw;
  tf2::Matrix3x3 m(quat);
  m.getRPY(roll, pitch, yaw);
  // dwa uses radians
  point.angle = yaw;

  RCLCPP_DEBUG(logger_, "Convert pose to position(%lf, %lf, %lf) on world map", point.x, point.y,
      point.angle);
}

uint32_t FollowPathActionServer::get_passing_waypoint_id(point_2d & current_point)
{
  uint32_t size = passing_ids_.size();
  if (size == 0) {
    RCLCPP_DEBUG(logger_, "Find passing through waypoint failed due to no ids");
    return 0;
  }

  uint32_t id = manager_->get_passing_waypoint_id(current_point);
  RCLCPP_INFO(logger_, "Find passing through waypoint(%d)", id);
  return id;
}

void FollowPathActionServer::receive_request(
    const std::shared_ptr<FollowPathSubCmd::Request> request,
    std::shared_ptr<FollowPathSubCmd::Response> response)
{
  int cmd = request->subcommand;
  RCLCPP_INFO(logger_, "sub cmd: %s", StringUtil::subcmd_to_string(cmd).c_str());

  if (cmd == (int)SubCommand::PAUSE) {
    manager_->request_pause_follow_path();
  } else if (cmd == (int)SubCommand::RESUME) {
    manager_->request_resume_follow_path();
  } else {
    RCLCPP_INFO(logger_, "error: unkown sub command");
    response->result = false;
    return;
  }
  response->result = true;
}
}  // namespace navigation
}  // namespace qrb_ros