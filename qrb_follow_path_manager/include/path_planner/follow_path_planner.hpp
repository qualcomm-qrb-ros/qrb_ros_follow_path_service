/*
 * Copyright (c) 2025 Qualcomm Innovation Center, Inc. All rights reserved.
 * SPDX-License-Identifier: BSD-3-Clause-Clear
 */

#ifndef QRB_NAVIGATION__FOLLOW_PATH_PLANNER_
#define QRB_NAVIGATION__FOLLOW_PATH_PLANNER_

#include "virtual_path/virtual_path_manager.hpp"
#include "obstacle_detector/obstacle_detector.hpp"
#include "controller/pid_controller.hpp"

#include "common.hpp"

#define BASE_MIN_ANGULAR_VELOCITY 0.25  // the min rotational angular velocity by base

namespace qrb
{
namespace navigation
{

class FollowPathPlanner
{
protected:
  void get_current_executing_path(std::vector<point_2d> & path);

private:
  void start_navigation();
  uint8_t get_path_by_waypoint(uint32_t goal_id,
      std::vector<uint32_t> & passing_ids,
      std::vector<point_2d> & path,
      uint32_t & point_size);
  bool compute_path_by_waypoint(uint32_t start_id,
      uint32_t goal_id,
      std::vector<uint32_t> & passing_ids,
      std::vector<uint32_t> & list,
      std::vector<point_2d> & point_list);
  void get_path_start_2_goal(point_2d & start,
      point_2d & goal,
      std::vector<point_2d> & path,
      std::vector<sub_path_info> & sub_path,
      std::vector<uint32_t> & list);
  void compute_path_start_2_goal(point_2d & start,
      point_2d & goal,
      std::vector<point_2d> & path,
      std::vector<uint32_t> & list);
  void navigation_by_follow_path();
  void navigation_by_sub_path();
  void get_robot_velocity(Eigen::Vector3f & robot_vel);
  void publish_twist(double vx, double vy, double vangle);
  void stop_publish_real_path();
  void publish_real_path(point_2d & point);
  void handle_msg();
  void send_msg(uint32_t id, uint32_t count);
  void notify_current_path_navigation_complete(bool is_arrive_target);
  uint64_t create_message_id();
  bool confirm_sub_path_invalid(std::vector<point_2d> & path);
  void confirm_sub_path_invalid(point_2d current_point);
  void remove_all_msg();
  void update_follow_path(std::vector<point_2d> & path);
  void update_follow_path(point_2d current_point, std::vector<point_2d> & path);
  uint32_t get_current_point_waypoint_id();
  bool is_waypoint(uint32_t id);
  bool get_bypassing_obstacle();
  void generate_path_list(std::vector<point_2d> & points, std::vector<waypoint_2d> & path);
  void find_cloest_point(point_2d & current_point, waypoint_2d & cloest_point);
  void pause_move();
  void print_follow_path(std::vector<waypoint_2d> & path, bool print);
  void print_follow_path(std::vector<point_2d> & path, bool print);
  void print_follow_path(std::vector<uint32_t> & path);
  std::string get_passing_ids(std::vector<uint32_t> & ids);
  void get_current_point(point_2d & point);
  void publish_follow_path(std::vector<point_2d> & path);
  bool arrive_path_end_point(point_2d & current_point, std::vector<point_2d> & executing_path);
  void save_navigation_path(std::string & out);

  uint32_t follow_path_type_;
  bool msg_queue_working_;
  MessageQueue queue_;
  std::shared_ptr<std::thread> thread_handle_msg_;
  char * thread_name_;
  std::vector<point_2d> point_list_;
  std::vector<waypoint_2d> path_list_;
  std::vector<sub_path_info> sub_path_list_;
  std::shared_ptr<PidController> robot_ctrl_;
  std::shared_ptr<VirtualPathManager> manager_;
  std::shared_ptr<ObstacleDetector> detector_;
  std::vector<point_2d> real_path_;
  std::vector<navigation_completed_func_t> navigation_cb_;
  publish_real_path_func_t publish_real_path_cb_;
  get_robot_velocity_func_t get_robot_velocity_cb_;

  publish_twist_func_t publish_twist_cb_;
  uint64_t current_navigation_msg_id_ = 0;
  std::mutex msg_mutex_;
  uint64_t msg_id_ = 0;
  uint32_t goal_id_;
  std::vector<uint32_t> passing_ids_;

  bool pause_;
  bool is_waypoint_follow_path;
  uint32_t current_sub_path_id_;
  publish_global_path_func_t publish_global_path_cb_;
  // Check if it is doing path planner
  bool is_doing_path_planner;
  // The accuracy of determining whether a certain point has been reached
  const double POSITION_TOLERANCE = 0.1;

  // record robotic 's current point
  point_2d current_point_;
  // record robotic 's final goal point
  point_2d final_point_;
  // List to be executed after categorizing and organizing the record path
  std::vector<std::vector<point_2d>> execute_path_list_;
  // record executing path index when passing through narrow areas
  uint32_t path_execute_index_ = 0;
  std::string nav_path_ = "navigator/nav_path.txt";
  const char * logger_ = "follow_path_planner";

public:
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
  void handle_amr_exception(bool exception);
  bool arrive_final_target_point_scope(point_2d & current_point, point_2d & final_point);
  uint64_t request_follow_path(uint32_t goal, std::vector<uint32_t> & passing_ids);
  uint32_t get_passing_waypoint_id(point_2d & current_point);
  void request_stop_navigation();
  void request_stop_navigation(bool result);
  void register_publish_global_path_callback(publish_global_path_func_t cb);
  void register_navigation_callback(navigation_completed_func_t cb);
  void register_publish_twist_callback(publish_twist_func_t cb);
  void register_get_robot_velocity_callback(get_robot_velocity_func_t cb);

  void register_publish_real_path_callback(publish_real_path_func_t cb);
  void request_pause_follow_path();
  void request_resume_follow_path();
  void update_current_pose(point_2d & point);
  void update_current_point(const point_2d point);
  double point_distance(point_2d start_point, point_2d end_point);

  FollowPathPlanner(std::shared_ptr<VirtualPathManager> & manager,
      std::shared_ptr<PidController> & robot_ctrl,
      std::shared_ptr<ObstacleDetector> & detector);

  const static uint8_t TYPE_INVALID_PATH = 0;
  const static uint8_t TYPE_EVEN_POINT_PATH = 1;
  const static uint8_t TYPE_WAYPOINT_PATH = 2;
  ~FollowPathPlanner();
};
}  // namespace navigation
}  // namespace qrb
#endif  // QRB_NAVIGATION__FOLLOW_PATH_PLANNER_
