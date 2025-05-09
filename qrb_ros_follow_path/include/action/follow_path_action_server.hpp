/*
 * Copyright (c) 2025 Qualcomm Innovation Center, Inc. All rights reserved.
 * SPDX-License-Identifier: BSD-3-Clause-Clear
 */

#ifndef QRB_ROS_NAVIGATION__FOLLOW_PATH_ACTION_SERVER_HPP_
#define QRB_ROS_NAVIGATION__FOLLOW_PATH_ACTION_SERVER_HPP_

#include "manager/follow_path_manager.hpp"

using namespace qrb::navigation;

namespace qrb_ros
{
namespace navigation
{
using FollowPath = qrb_ros_navigation_msgs::action::FollowPath;
using GoalHandleFollowPath = rclcpp_action::ServerGoalHandle<FollowPath>;
using FollowPathSubCmd = qrb_ros_amr_msgs::srv::SubCmd;

class FollowPathActionServer : public rclcpp_lifecycle::LifecycleNode
{
public:
  FollowPathActionServer(std::shared_ptr<FollowPathManager> & service);

  ~FollowPathActionServer();

  void update_current_pose(PoseStamped & pose);

private:
  LifecycleNodeInterface::CallbackReturn on_configure(const rclcpp_lifecycle::State &) override;
  LifecycleNodeInterface::CallbackReturn on_activate(const rclcpp_lifecycle::State &) override;
  LifecycleNodeInterface::CallbackReturn on_deactivate(const rclcpp_lifecycle::State &) override;
  LifecycleNodeInterface::CallbackReturn on_cleanup(const rclcpp_lifecycle::State &) override;
  LifecycleNodeInterface::CallbackReturn on_shutdown(const rclcpp_lifecycle::State &) override;

  void init_server();
  void deinit_server();

  rclcpp_action::GoalResponse handle_goal(const rclcpp_action::GoalUUID & uuid,
      std::shared_ptr<const FollowPath::Goal> goal);

  rclcpp_action::CancelResponse handle_cancel(
      const std::shared_ptr<GoalHandleFollowPath> goal_handle);

  void handle_accepted(const std::shared_ptr<GoalHandleFollowPath> goal_handle);

  bool is_pose_changed(PoseStamped & pose);

  void receive_navigation_callback(uint64_t request_id, bool result);

  uint32_t get_passing_waypoint(PoseStamped & pose);

  void convert_pose_to_2d_point(PoseStamped & pose, point_2d & point);

  uint32_t get_passing_waypoint_id(point_2d & current_point);

  void receive_request(const std::shared_ptr<FollowPathSubCmd::Request> request,
      std::shared_ptr<FollowPathSubCmd::Response> response);

  std::shared_ptr<FollowPathManager> manager_;
  rclcpp::Service<FollowPathSubCmd>::SharedPtr service_;
  rclcpp_action::Server<FollowPath>::SharedPtr action_server_ptr_;
  std::shared_ptr<GoalHandleFollowPath> server_global_handle_;
  PoseStamped current_pose_;
  qrb::navigation::navigation_completed_func_t navigation_callback_;
  std::map<rclcpp_action::GoalUUID, uint64_t> request_id_map_;
  std::map<uint64_t, std::shared_ptr<GoalHandleFollowPath>> handle_map_;
  std::vector<uint32_t> passing_ids_;
  bool navigating_;

  rclcpp::Logger logger_{ rclcpp::get_logger("follow_path_action_server") };
};

}  // namespace navigation
}  // namespace qrb_ros
#endif  // QRB_ROS_NAVIGATION__FOLLOW_PATH_ACTION_SERVER_HPP_