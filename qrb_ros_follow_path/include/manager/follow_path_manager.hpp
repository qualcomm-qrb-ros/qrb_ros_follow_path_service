/*
 * Copyright (c) 2025 Qualcomm Innovation Center, Inc. All rights reserved.
 * SPDX-License-Identifier: BSD-3-Clause-Clear
 */

#ifndef QRB_NAVIGATION__FOLLOW_PATH_MANAGER_HPP_
#define QRB_NAVIGATION__FOLLOW_PATH_MANAGER_HPP_

#include "follow_path_planner.hpp"
#include "obstacle_detector.hpp"
#include "pid_controller.hpp"
#include "common.hpp"
#include "ros_common.hpp"

#include <string.h>

namespace qrb
{
namespace navigation
{

/**
 * @class navigation_controller::FollowPathManager
 * @desc The FollowPathManager create nodes to control the Navigation.
 */
class FollowPathManager
{
public:
  /**
   * @desc A constructor for FollowPathManager
   */
  FollowPathManager();

  /**
   * @desc A destructor for FollowPathManager
   */
  ~FollowPathManager();

  // VirtualPathServiceServer
  uint32_t add_waypoint(point_2d & point);
  bool remove_waypoint(uint32_t id);
  bool get_waypoint_id_list(std::vector<uint32_t> & list);
  bool get_waypoint(uint32_t id, point_2d & point);
  bool add_virtual_path(uint32_t id, std::vector<uint32_t> & ids);
  bool remove_virtual_path(uint32_t id, std::vector<uint32_t> & ids);
  bool get_virtual_path(uint32_t id, std::vector<uint32_t> & ids);
  uint32_t add_target_waypoint(point_2d & point);
  void set_bypassing_obstacle(bool val);
  bool remove_waypoint_and_virtual_path();

  // FollowPathActionServer
  uint64_t request_follow_path(uint32_t goal, std::vector<uint32_t> & passing_ids);
  uint32_t get_passing_waypoint_id(point_2d & current_point);
  void request_stop_follow_path();
  void register_navigation_callback(navigation_completed_func_t cb);
  void update_current_pose(point_2d & point);
  void request_pause_follow_path();
  void request_resume_follow_path();

  // PathPublisher
  void register_publish_real_path_callback(publish_real_path_func_t cb);
  void register_publish_global_path_callback(publish_global_path_func_t cb);

  // TwistPublisher
  void register_publish_twist_callback(publish_twist_func_t cb);

  // OdomSubscriber
  void register_get_robot_velocity_callback(get_robot_velocity_func_t cb);

  // MapSubscriber
  void register_get_grid_map_callback(get_grid_map_func_t cb);

  // ExceptionSubscriber
  void handle_amr_exception(bool exception);
  void handle_emergency(bool enter);

  // DeveloperModeSubscriber
  void send_debug_event(int16_t event);

  // NavigationPathServiceServer
  bool compute_follow_path(bool use_start,
      uint32_t start,
      uint32_t goal,
      std::vector<uint32_t> & list,
      std::vector<point_2d> & point_list);
  bool compute_follow_path(bool use_start,
      uint32_t start,
      uint32_t goal,
      std::vector<uint32_t> & passing_ids,
      std::vector<uint32_t> & list,
      std::vector<point_2d> & point_list);

private:
  void init();

  std::shared_ptr<FollowPathPlanner> follow_path_planner_;
  std::shared_ptr<PidController> robot_ctrl_;
  std::shared_ptr<VirtualPathManager> manager_;
  std::shared_ptr<ObstacleDetector> detector_;

  const char * logger_ = "follow_path_manager";
};
}  // namespace navigation
}  // namespace qrb
#endif  // QRB_NAVIGATION__FOLLOW_PATH_MANAGER_HPP_