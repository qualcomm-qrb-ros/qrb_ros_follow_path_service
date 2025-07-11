/*
 * Copyright (c) 2025 Qualcomm Innovation Center, Inc. All rights reserved.
 * SPDX-License-Identifier: BSD-3-Clause-Clear
 */

#include "follow_path_planner.hpp"

#include <cmath>
#include <memory>

namespace qrb
{
namespace navigation
{

FollowPathPlanner::FollowPathPlanner(std::shared_ptr<VirtualPathManager> & manager,
    std::shared_ptr<PidController> & robot_ctrl,
    std::shared_ptr<ObstacleDetector> & detector)
{
  thread_name_ = (char *)"follow_path";
  manager_ = manager;
  detector_ = detector;
  robot_ctrl_ = robot_ctrl;
  is_doing_path_planner = false;
  msg_queue_working_ = true;
  pause_ = false;
  thread_handle_msg_ =
      std::make_shared<std::thread>(std::mem_fn(&FollowPathPlanner::handle_msg), this);
}

FollowPathPlanner::~FollowPathPlanner()
{
  queue_.notify();
  if (thread_handle_msg_->joinable()) {
    thread_handle_msg_->join();
  }
  request_stop_navigation();
  notify_current_path_navigation_complete(true);
}

void FollowPathPlanner::register_publish_global_path_callback(publish_global_path_func_t cb)
{
  printf("[%s]: register_publish_global_path_callback\n", logger_);
  publish_global_path_cb_ = cb;
}

void FollowPathPlanner::register_publish_twist_callback(publish_twist_func_t cb)
{
  printf("[%s]: register_publish_twist_callback\n", logger_);
  publish_twist_cb_ = cb;
}

void FollowPathPlanner::register_get_robot_velocity_callback(get_robot_velocity_func_t cb)
{
  printf("[%s]: register_get_robot_velocity_callback\n", logger_);
  get_robot_velocity_cb_ = cb;
}

void FollowPathPlanner::register_publish_real_path_callback(publish_real_path_func_t cb)
{
  printf("[%s]: register_publish_real_path_callback\n", logger_);
  publish_real_path_cb_ = cb;
}

void FollowPathPlanner::register_navigation_callback(navigation_completed_func_t cb)
{
  printf("[%s]: register_navigation_callback\n", logger_);
  navigation_cb_.push_back(cb);
}

bool FollowPathPlanner::compute_follow_path(bool use_start,
    uint32_t start,
    uint32_t goal,
    std::vector<uint32_t> & list,
    std::vector<point_2d> & point_list)
{
  uint32_t start_id;
  if (use_start) {
    start_id = start;
  } else {
    start_id = get_current_point_waypoint_id();
    if (start_id == 0) {
      printf("[%s]: Current position is not on a waypoint\n", logger_);
      return false;
    }
  }

  if (!is_waypoint(goal)) {
    printf("[%s]: Goal is not a waypoint\n", logger_);
    return false;
  }

  std::vector<uint32_t> passing_ids;
  bool res = compute_path_by_waypoint(start_id, goal, passing_ids, list, point_list);
  publish_follow_path(point_list);
  print_follow_path(point_list, true);
  print_follow_path(list);
  return res;
}

bool FollowPathPlanner::compute_follow_path(bool use_start,
    uint32_t start,
    uint32_t goal,
    std::vector<uint32_t> & passing_ids,
    std::vector<uint32_t> & list,
    std::vector<point_2d> & point_list)
{
  uint32_t start_id;
  if (use_start) {
    start_id = start;
  } else {
    start_id = get_current_point_waypoint_id();
    if (start_id == 0) {
      printf("[%s]: Current position is not a waypoint\n", logger_);
      return false;
    }
  }

  if (!is_waypoint(goal)) {
    printf("[%s]: Goal is not a waypoint\n", logger_);
    return false;
  }

  if (!is_passing_ids_valid(passing_ids)) {
    printf("[%s]: passing ids is invalid\n", logger_);
    return false;
  }

  bool res = compute_path_by_waypoint(start_id, goal, passing_ids, list, point_list);
  publish_follow_path(point_list);
  print_follow_path(point_list, true);
  print_follow_path(list);
  return res;
}

void FollowPathPlanner::request_stop_navigation()
{
  printf("[%s]: Request stop follow path\n", logger_);
  is_doing_path_planner = false;
  msg_queue_working_ = false;
  pause_ = false;

  stop_publish_real_path();
}

void FollowPathPlanner::request_stop_navigation(bool result)
{
  printf("[%s]: Request stop follow path, result=%d\n", logger_, result);
  is_doing_path_planner = false;
  msg_queue_working_ = false;
  pause_ = false;
  remove_all_msg();
  notify_current_path_navigation_complete(result);

  stop_publish_real_path();
}

void FollowPathPlanner::request_pause_follow_path()
{
  std::unique_lock<std::mutex> lk(msg_mutex_);
  printf("[%s]: Request pause follow path\n", logger_);
  pause_ = true;
  is_doing_path_planner = false;
  pause_move();
}

void FollowPathPlanner::request_resume_follow_path()
{
  std::unique_lock<std::mutex> lk(msg_mutex_);
  printf("[%s]: Request resume follow path\n", logger_);
  pause_ = false;
  is_doing_path_planner = true;
}

uint64_t FollowPathPlanner::request_follow_path(uint32_t goal, std::vector<uint32_t> & passing_ids)
{
  if (is_doing_path_planner) {
    printf("[%s]: There is a path is in executed state.\n", logger_);
    return 0;
  }

  uint32_t point_size;
  point_list_.clear();
  path_list_.clear();
  sub_path_list_.clear();
  waypoint_list_.clear();
  uint8_t type = get_path_by_waypoint(goal, passing_ids, point_list_, point_size);
  if (type == TYPE_INVALID_PATH) {
    printf("[%s]: Navigation type is error\n", logger_);
    return 0;
  }
  generate_path_list(point_list_, path_list_);

  // check the amr position whether is the scope of start point
  if (point_list_.size() == 0) {
    printf("[%s]: There is no path.\n", logger_);
    return 0;
  }
  point_2d start_point = point_list_.at(0);
  point_2d current_point;
  get_current_point(current_point);
  double distance = point_distance(current_point, start_point);
  printf("[%s]: Current position is %.2f\n", logger_, distance);
  if (distance > 0.5) {
    printf(
        "[%s]: Current position(%.2f,%.2f,%.2f) is %.2f meter away from the "
        "start point(%.2f,%.2f,%.2f)\n",
        logger_, current_point.x, current_point.y, current_point.angle, distance, start_point.x,
        start_point.y, start_point.angle);
    return 0;
  }

  // update current navigation id
  current_navigation_msg_id_ = create_message_id();

  publish_follow_path(point_list_);
  final_point_ = point_list_.at(point_size - 1);
  is_doing_path_planner = true;
  goal_id_ = goal;
  passing_ids_ = passing_ids;
  if (type == TYPE_WAYPOINT_PATH) {
    follow_path_type_ = TYPE_WAYPOINT_PATH;
    start_navigation();
  } else {
    printf("[%s]: Navigation type is error\n", logger_);
    return 0;
  }
  return current_navigation_msg_id_;
}

void FollowPathPlanner::update_current_pose(point_2d & point)
{
  update_current_point(point);

  bool result = arrive_final_target_point_scope(current_point_, final_point_);
  if (result) {
    return;
  }
  confirm_sub_path_invalid(point);
}

float FollowPathPlanner::get_distance_to_goal(point_2d & current_point, uint32_t passing_id)
{
  uint32_t next_waypoint_index = 0;
  for (uint32_t id : waypoint_list_) {
    next_waypoint_index++;
    if (passing_id == id) {
      break;
    }
  }

  uint32_t len = waypoint_list_.size();
  if (next_waypoint_index >= len) {
    printf("[%s]: next_waypoint_index(%d,%d) is error\n", logger_, next_waypoint_index, len);
    return 0;
  }

  uint32_t next_waypoint_id = waypoint_list_[next_waypoint_index];

  point_2d next_waypoint;
  bool result = manager_->get_waypoint(next_waypoint_id, next_waypoint);
  if (!result) {
    printf("[%s]: Get next waypoint(%d) failed\n", logger_, next_waypoint_id);
    return 0;
  }

  double total_dist = point_distance(current_point, next_waypoint);

  printf("[%s]: total_dist(current to %d)=%.2f\n", logger_, next_waypoint_id, total_dist);

  for (uint32_t i = next_waypoint_index; i < len - 1; i++) {
    point_2d p1;
    uint32_t p1_id = waypoint_list_[i];
    result = manager_->get_waypoint(p1_id, p1);
    if (!result) {
      printf("[%s]: Get p1(%d) failed\n", logger_, p1_id);
      return 0;
    }
    point_2d p2;
    uint32_t p2_id = waypoint_list_[i + 1];
    result = manager_->get_waypoint(p2_id, p2);
    if (!result) {
      printf("[%s]: Get p2(%d) failed\n", logger_, p2_id);
      return 0;
    }

    double dist = point_distance(p1, p2);
    printf("[%s]: dist(%d to %d)=%.2f\n", logger_, p1_id, p2_id, dist);
    total_dist += dist;
  }

  printf("[%s]: Get total distance=%.2f\n", logger_, total_dist);
  return (float)total_dist;
}

uint32_t FollowPathPlanner::get_passing_waypoint_id(point_2d & current_point)
{
  waypoint_2d wp;
  find_cloest_point(current_point, wp);
  uint32_t id = wp.waypoint_id;
  return id;
}

void FollowPathPlanner::notify_current_path_navigation_complete(bool is_arrive_target)
{
  for (auto it : navigation_cb_) {
    it(current_navigation_msg_id_, is_arrive_target);
  }
}

void FollowPathPlanner::start_navigation()
{
  msg_queue_working_ = true;
  navigation_by_follow_path();
}

void FollowPathPlanner::handle_amr_exception(bool exception)
{
  if (!exception) {
    return;
  }
  remove_all_msg();
  request_stop_navigation();
  notify_current_path_navigation_complete(false);
}

bool FollowPathPlanner::arrive_final_target_point_scope(point_2d & current_point,
    point_2d & final_point)
{
  if (!is_doing_path_planner) {
    return false;
  }

  publish_real_path(current_point);

  uint32_t path_size = execute_path_list_.size();
  if (path_size == 0) {
    return false;
  }
  double distance = point_distance(current_point, final_point);
  if (distance < 2) {
    printf("[%s]: Current position is still %.2f meter away from the target\n", logger_, distance);
  }
  bool result = distance < POSITION_TOLERANCE;
  if (result) {
    printf("[%s]: path list size is %d when arriving fianl target\n", logger_, path_size);
    if (path_size == 1) {
      printf("[%s]: Finish follow path navigation!\n", logger_);
      request_stop_navigation();
      notify_current_path_navigation_complete(true);
      return true;
    }
  }
  if (path_size > 1) {
    std::vector<point_2d> current_execute;
    get_current_executing_path(current_execute);
    printf("[%s]: current execute path end point x: %.2f, y: %.2f\n", logger_,
        current_execute.at(current_execute.size() - 1).x,
        current_execute.at(current_execute.size() - 1).y);
    bool result_end_point = arrive_path_end_point(current_point, current_execute);
    if (result_end_point) {
      printf("[%s]: segement path execute finished!\n", logger_);
      return false;
    }
  }

  return false;
}

bool FollowPathPlanner::compute_path_by_waypoint(uint32_t start_id,
    uint32_t goal_id,
    std::vector<uint32_t> & passing_ids,
    std::vector<uint32_t> & list,
    std::vector<point_2d> & point_list)
{
  uint32_t len = passing_ids.size();
  std::string str = get_passing_ids(passing_ids);
  printf("[%s]: get path by waypoint(start_id=%d,goal_id=%d,passing_ids=%s)\n", logger_, start_id,
      goal_id, str.c_str());
  point_2d start;
  bool result = manager_->get_waypoint(start_id, start);
  if (!result) {
    printf(
        "[%s]: Get follow path failed from the waypoints because "
        "start is not waypoint\n",
        logger_);
    return false;
  }
  point_2d goal;
  result = manager_->get_waypoint(goal_id, goal);
  if (!result) {
    printf(
        "[%s]: Get follow path failed from the waypoints because "
        "goal is not waypoint\n",
        logger_);
    return false;
  }

  if (len == 0) {
    compute_path_start_2_goal(start, goal, point_list, list);
  } else {
    std::vector<point_2d> goals;
    std::vector<uint32_t> goals_id;
    goals.push_back(start);
    goals_id.push_back(start_id);
    for (uint32_t i = 0; i < len; i++) {
      uint32_t id = passing_ids[i];
      point_2d p;
      if (manager_->get_waypoint(id, p)) {
        goals.push_back(p);
        goals_id.push_back(id);
      }
    }
    goals.push_back(goal);
    goals_id.push_back(goal_id);

    bool first_sub = true;
    uint32_t goal_len = goals.size();
    for (uint32_t i = 0; i < goal_len - 1; i++) {
      point_2d sub_start = goals[i];
      point_2d sub_goal = goals[i + 1];
      std::vector<point_2d> tmp_path;
      std::vector<uint32_t> tmp_list;
      printf("[%s]: Compute path from start to goal:%d------->%d\n", logger_, goals_id[i],
          goals_id[i + 1]);
      compute_path_start_2_goal(sub_start, sub_goal, tmp_path, tmp_list);

      if (tmp_path.size() == 0) {
        printf("[%s]: Compute sub path failed because sub path size is zero, clear...\n", logger_);
        point_list.clear();
        list.clear();
        return false;
      }

      uint32_t index = 1;
      for (auto p : tmp_path) {
        if (!first_sub && (index == 1)) {
          index++;
          continue;
        }
        point_list.push_back(p);
      }

      index = 1;
      for (auto id : tmp_list) {
        if (!first_sub && (index == 1)) {
          index++;
          continue;
        }
        list.push_back(id);
      }
      first_sub = false;
    }
  }

  uint32_t point_size = point_list.size();
  printf("[%s]: Get follow path(size=%d) from the waypoints\n", logger_, point_size);
  if (point_size == 0) {
    return false;
  }
  return true;
}

uint8_t FollowPathPlanner::get_path_by_waypoint(uint32_t goal_id,
    std::vector<uint32_t> & passing_ids,
    std::vector<point_2d> & path,
    uint32_t & point_size)
{
  uint32_t len = passing_ids.size();
  std::string str = get_passing_ids(passing_ids);
  point_2d start, current_point;
  get_current_point(current_point);
  uint32_t start_id = manager_->get_nearby_waypoint(current_point, start);
  if (start_id == 0) {
    printf("[%s]: start(%.2f,%.2f,%.2f) is not waypoint, follow path is invalid\n", logger_,
        current_point.x, current_point.y, current_point.angle);
    return TYPE_INVALID_PATH;
  }

  point_2d goal;
  bool result = manager_->get_waypoint(goal_id, goal);
  if (!result) {
    printf("[%s]: goal(%.2f,%.2f,%.2f, id=%d) is not waypoint, follow path is invalid\n", logger_,
        current_point.x, current_point.y, current_point.angle, goal_id);
    return TYPE_INVALID_PATH;
  }

  result = is_passing_ids_valid(passing_ids);
  if (!result) {
    printf("[%s]: passing ids(%s) is not waypoint, follow path is invalid\n", logger_, str.c_str());
    return TYPE_INVALID_PATH;
  }

  printf("[%s]: get path by waypoint(start_id=%d,goal_id=%d,passing_ids=%s)\n", logger_, start_id,
      goal_id, str.c_str());

  if (len == 0) {
    get_path_start_2_goal(start, goal, path, sub_path_list_, waypoint_list_);
  } else {
    std::vector<point_2d> goals;
    goals.push_back(start);
    std::vector<uint32_t> goals_id;
    goals_id.push_back(start_id);
    for (uint32_t i = 0; i < len; i++) {
      uint32_t id = passing_ids[i];
      point_2d p;
      if (manager_->get_waypoint(id, p)) {
        goals.push_back(p);
        goals_id.push_back(id);
      }
    }
    goals.push_back(goal);
    goals_id.push_back(goal_id);

    uint32_t goal_len = goals.size();
    bool first_sub = true;
    for (uint32_t i = 0; i < goal_len - 1; i++) {
      point_2d sub_start = goals[i];
      point_2d sub_goal = goals[i + 1];

      printf("[%s]: get path from start to goal:%d------->%d\n", logger_, goals_id[i],
          goals_id[i + 1]);
      std::vector<point_2d> tmp_path;
      std::vector<sub_path_info> tmp_sub_path;
      std::vector<uint32_t> tmp_list;
      get_path_start_2_goal(sub_start, sub_goal, tmp_path, tmp_sub_path, tmp_list);

      if (tmp_path.size() == 0) {
        printf("[%s]: Get sub path failed because sub path size is zero, clear...\n", logger_);
        path.clear();
        sub_path_list_.clear();
        point_size = 0;
        return TYPE_INVALID_PATH;
      }

      uint32_t index = 1;
      for (auto p : tmp_path) {
        if (!first_sub && (index == 1)) {
          index++;
          continue;
        }
        path.push_back(p);
      }

      index = 1;
      for (auto id : tmp_list) {
        if (!first_sub && (index == 1)) {
          index++;
          continue;
        }
        waypoint_list_.push_back(id);
      }
      first_sub = false;

      for (auto sub_path : tmp_sub_path) {
        sub_path_list_.push_back(sub_path);
      }
    }
  }

  point_size = path.size();
  printf("[%s]: Get follow path(size=%d) from the waypoints\n", logger_, point_size);
  print_follow_path(waypoint_list_);
  return TYPE_WAYPOINT_PATH;
}

void FollowPathPlanner::get_path_start_2_goal(point_2d & start,
    point_2d & goal,
    std::vector<point_2d> & path,
    std::vector<sub_path_info> & sub_path,
    std::vector<uint32_t> & list)
{
  manager_->compute_follow_path(start, goal, sub_path, list);

  path.clear();
  printf("[%s]: sub_path_list size=%ld\n", logger_, sub_path.size());
  for (auto sub_info : sub_path) {
    std::vector<point_2d> sub_points = sub_info.path;
    path.insert(path.end(), sub_points.begin(), sub_points.end());
  }
  printf("[%s]: Get sub path(size=%ld) from the waypoints\n", logger_, path.size());
}

void FollowPathPlanner::compute_path_start_2_goal(point_2d & start,
    point_2d & goal,
    std::vector<point_2d> & path,
    std::vector<uint32_t> & list)
{
  std::vector<sub_path_info> sub_path;
  manager_->compute_follow_path(start, goal, sub_path, list);

  path.clear();
  printf("[%s]: sub_path_list size=%ld\n", logger_, sub_path.size());
  for (auto sub_info : sub_path) {
    std::vector<point_2d> sub_points = sub_info.path;
    path.insert(path.end(), sub_points.begin(), sub_points.end());
  }
  printf("[%s]: Compute sub path(size=%ld) from the waypoints\n", logger_, path.size());
}

void FollowPathPlanner::get_current_executing_path(std::vector<point_2d> & path)
{
  path = execute_path_list_.at(path_execute_index_);
}

void FollowPathPlanner::navigation_by_follow_path()
{
  uint32_t len = sub_path_list_.size();
  for (uint32_t i = 0; i < len; i++) {
    send_msg(i, len);
  }
}

void FollowPathPlanner::navigation_by_sub_path()
{
  while (msg_queue_working_) {
    point_2d current_point;
    get_current_point(current_point);
    Eigen::Vector3f robot_velocity;
    get_robot_velocity(robot_velocity);

    Eigen::Vector3d p;
    Eigen::Vector2d vel;
    p.x() = current_point.x;
    p.y() = current_point.y;
    p.z() = current_point.angle;
    vel.x() = robot_velocity[0];
    vel.y() = robot_velocity[2];

    robot_ctrl_->update_state(p, vel);

    double v_x = 0;
    double v_z = 0;
    robot_ctrl_->run(v_x, v_z);
    if (robot_ctrl_->is_path_finished()) {
      robot_ctrl_->reset_controller_flag();
      // execute next sub path
      printf("[%s]: -------END SUB PATH NAVIGATION--------\n", logger_);
      publish_twist(0, 0, 0);
      return;
    }
    if ((v_x == 0) && (v_z < BASE_MIN_ANGULAR_VELOCITY) && (v_z > 0)) {
      v_z = BASE_MIN_ANGULAR_VELOCITY;
    }

    if ((v_x == 0) && (v_z > -BASE_MIN_ANGULAR_VELOCITY) && (v_z < 0)) {
      v_z = -BASE_MIN_ANGULAR_VELOCITY;
    }

    publish_twist(v_x, 0, v_z);
    usleep(50 * 1000);  // 50ms
  }
  // stop moving after canceling or finishing the current sub path
  printf("[%s]: -------STOP SUB PATH--------\n", logger_);
  publish_twist(0, 0, 0);
}

void FollowPathPlanner::get_robot_velocity(Eigen::Vector3f & robot_vel)
{
  twist_vel twist;
  get_robot_velocity_cb_(twist);
  robot_vel[0] = twist.x;
  robot_vel[1] = twist.y;
  robot_vel[2] = twist.z;
}

void FollowPathPlanner::handle_msg()
{
  NavigationMsg msg;

  while (true) {
    queue_.wait(msg);
    if (msg.type != (int)NavigationMsg::NAVIGATION_BY_SUB_PATH) {
      printf("[%s]: Msg is invalid\n", logger_);
      continue;
    }
    uint32_t sub_path_id = msg.id;
    current_sub_path_id_ = msg.id;
    uint32_t len = sub_path_list_.size();
    if ((len == 0) || (sub_path_id >= len)) {
      printf("[%s]: Sub path is invalid, len=%d, sub_path_id=%d\n", logger_, len, sub_path_id);
      continue;
    }

    sub_path_info sub_info = sub_path_list_[sub_path_id];
    std::vector<point_2d> sub_path = sub_info.path;
    if (confirm_sub_path_invalid(sub_path)) {
      printf("[%s]: Cancel current follow path and update new path\n", logger_);
      request_stop_navigation();
      remove_all_msg();
      update_follow_path(sub_path);
      continue;
    }

    std::vector<Eigen::Vector3d> path;
    uint32_t point_len = sub_path.size();
    for (uint32_t i = 0; i < point_len; i++) {
      Eigen::Vector3d p;
      p[0] = sub_path[i].x;
      p[1] = sub_path[i].y;
      p[2] = sub_path[i].angle;
      path.push_back(p);
    }

    printf(
        "[%s]: ======START SUB PATH NAVIGATION(sub_id=%d,len=%d)FROM(%.2f,%.2f,%.2f) "
        "TO(%.2f,%.2f,%.2f)======\n",
        logger_, sub_path_id, point_len, sub_path[0].x, sub_path[0].y, sub_path[0].angle,
        sub_path[point_len - 1].x, sub_path[point_len - 1].y, sub_path[point_len - 1].angle);

    robot_ctrl_->request_pid_control(path);
    navigation_by_sub_path();
    if (sub_path_id == (len - 1)) {
      request_stop_navigation();
      notify_current_path_navigation_complete(true);
    }
  }
}

void FollowPathPlanner::send_msg(uint32_t id, uint32_t count)
{
  NavigationMsg msg;
  msg.type = (int)NavigationMsg::NAVIGATION_BY_SUB_PATH;
  msg.id = id;
  msg.count = count;
  queue_.push(msg);
}

void FollowPathPlanner::publish_twist(double vx, double vy, double vangle)
{
  std::unique_lock<std::mutex> lk(msg_mutex_);
  if (pause_) {
    printf("[%s]: current is pause state\n", logger_);
    return;
  }

  twist_vel twist;
  twist.x = vx;
  twist.y = vy;
  twist.z = vangle;
  printf("[%s]: Publish v_x:%.2f,v_y:%.2f,v_z:%.2f\n", logger_, twist.x, twist.y, twist.z);

  if (publish_twist_cb_ != nullptr) {
    publish_twist_cb_(twist);
  } else {
    printf("[%s]: publish_twist_cb_ is nullptr\n", logger_);
  }
}

void FollowPathPlanner::pause_move()
{
  twist_vel twist;
  twist.x = 0;
  twist.y = 0;
  twist.z = 0;

  if (publish_twist_cb_ != nullptr) {
    publish_twist_cb_(twist);
  } else {
    printf("[%s]: publish_twist_cb_ is nullptr\n", logger_);
  }
}

void FollowPathPlanner::stop_publish_real_path()
{
  real_path_.clear();
}

void FollowPathPlanner::publish_real_path(point_2d & point)
{
  printf("[%s]: publish_real_path, size=%ld,nav=%d\n", logger_, real_path_.size(),
      is_doing_path_planner);
  if (!is_doing_path_planner) {
    return;
  }

  point_2d p;
  p.x = point.x;
  p.y = point.y;
  real_path_.push_back(p);
  if (publish_twist_cb_ != nullptr) {
    publish_real_path_cb_(real_path_);
  } else {
    printf("[%s]: publish_real_path_cb_ is nullptr\n", logger_);
  }
}

uint64_t FollowPathPlanner::create_message_id()
{
  // create new message id
  std::unique_lock<std::mutex> lk(msg_mutex_);
  if (msg_id_ == UINT64_MAX) {
    msg_id_ = 0;
  }

  int id = ++msg_id_;
  lk.unlock();
  return id;
}

void FollowPathPlanner::confirm_sub_path_invalid(point_2d current_point)
{
  if (!is_doing_path_planner) {
    return;
  }
  uint32_t len = sub_path_list_.size();
  if ((len == 0) || (current_sub_path_id_ >= len)) {
    printf("[%s]: Check the sub path size=%d and id=%d\n", logger_, len, current_sub_path_id_);
    return;
  }
  sub_path_info sub_info = sub_path_list_[current_sub_path_id_];
  std::vector<point_2d> sub_path = sub_info.path;
  uint32_t size = sub_path.size();
  point_2d end_point = sub_path[size - 1];

  std::vector<point_2d> remaining_path;
  manager_->compute_follow_path(current_point, end_point, remaining_path);

  if (confirm_sub_path_invalid(remaining_path)) {
    printf("[%s]: Remove old navigation path and update new path\n", logger_);
    request_stop_navigation();
    remove_all_msg();
    // wait for last follow path stop
    usleep(2000 * 1000);  // 2s
    update_follow_path(current_point, sub_path);
  }
}

bool FollowPathPlanner::confirm_sub_path_invalid(std::vector<point_2d> & path)
{
  uint32_t len = path.size();
  if (len < 2) {
    printf("[%s]: Sub path is valid, len=%d\n", logger_, len);
    return false;
  }

  if (!get_bypassing_obstacle()) {
    printf("[%s]: Ingore bypassing obstacle\n", logger_);
    return false;
  }

  // confirm virtual path is valid with base shape
  bool res = detector_->confirm_path_invalid_with_base_shape(path);
  if (!res) {
    printf("[%s]: Sub path is valid\n", logger_);
    return false;
  }
  return true;
}

void FollowPathPlanner::remove_all_msg()
{
  queue_.remove();
}

void FollowPathPlanner::update_follow_path(std::vector<point_2d> & path)
{
  // update the virtual path if the sub path is invalid
  printf("[%s]: Sub path is invalid and update path\n", logger_);
  uint32_t len = path.size();
  point_2d start;
  point_2d end;
  start.x = path[0].x;
  start.y = path[0].y;
  start.angle = path[0].angle;
  end.x = path[len - 1].x;
  end.y = path[len - 1].y;
  end.angle = path[len - 1].angle;

  uint32_t start_id = manager_->get_waypoint_id(start);
  if (start_id == 0) {
    printf("[%s]: Start point is error\n", logger_);
    printf("[%s]: Stop current follow path\n", logger_);
    request_stop_navigation(false);
    return;
  }
  uint32_t end_id = manager_->get_waypoint_id(end);
  if (end_id == 0) {
    printf("[%s]: End point is error\n", logger_);
    printf("[%s]: Stop current follow path\n", logger_);
    request_stop_navigation(false);
    return;
  }

  std::vector<uint32_t> adjacent_ids;
  adjacent_ids.push_back(end_id);
  // remove the virtual path before updating the follow path
  printf("[%s]: Remove the virtual path on dynamic obstacle\n", logger_);
  bool res = manager_->remove_virtual_path(start_id, adjacent_ids);

  // ignore the passing waypoint for updating the follow path
  passing_ids_.clear();
  uint32_t result = request_follow_path(goal_id_, passing_ids_);
  if (result == 0) {
    printf("[%s]: Update path failed and stop current follow path\n", logger_);
    request_stop_navigation(false);
  }

  // restore the virtual path after updating the follow path
  if (res) {
    printf("[%s]: Restore the virtual path on dynamic obstacle\n", logger_);
    manager_->force_add_virtual_path(start_id, adjacent_ids);
  }
}

void FollowPathPlanner::update_follow_path(point_2d current_point, std::vector<point_2d> & path)
{
  // update the virtual path if the sub path is invalid
  printf("[%s]: Sub path is invalid and update path when passing the start point \n", logger_);
  uint32_t len = path.size();
  point_2d start;
  point_2d end;
  start.x = path[0].x;
  start.y = path[0].y;
  start.angle = path[0].angle;
  end.x = path[len - 1].x;
  end.y = path[len - 1].y;
  end.angle = path[len - 1].angle;

  uint32_t start_id = manager_->get_waypoint_id(start);
  if (start_id == 0) {
    printf("[%s]: Start point(%f,%f,%f) is error\n", logger_, start.x, start.y, start.angle);
    printf("[%s]: Stop current follow path\n", logger_);
    request_stop_navigation(false);
    return;
  }
  uint32_t end_id = manager_->get_waypoint_id(end);
  if (end_id == 0) {
    printf("[%s]: End point(%f,%f,%f) is error\n", logger_, end.x, end.y, end.angle);
    printf("[%s]: Stop current follow path\n", logger_);
    request_stop_navigation(false);
    return;
  }

  std::vector<uint32_t> adjacent_ids;
  adjacent_ids.push_back(end_id);
  // remove the virtual path and add waypoint&virtual path before updating the follow path
  printf("[%s]: Remove the virtual path and add waypoint & virtual path\n", logger_);
  bool res = manager_->remove_virtual_path(start_id, adjacent_ids);
  uint32_t id = manager_->add_waypoint(current_point);
  std::vector<uint32_t> tmp_ids;
  tmp_ids.push_back(id);
  res = manager_->add_virtual_path(start_id, tmp_ids);

  // ignore the passing waypoint for updating the follow path
  passing_ids_.clear();
  uint32_t result = request_follow_path(goal_id_, passing_ids_);
  if (result == 0) {
    printf("[%s]: Update path failed and stop current follow path\n", logger_);
    request_stop_navigation(false);
  }

  // restore the virtual path after updating the follow path
  if (res) {
    printf("[%s]: Restore the virtual path & waypoint on dynamic obstacle\n", logger_);
    manager_->force_add_virtual_path(start_id, adjacent_ids);
    manager_->remove_waypoint(id);
  }
}

uint32_t FollowPathPlanner::get_current_point_waypoint_id()
{
  point_2d current_point;
  get_current_point(current_point);

  std::vector<uint32_t> list;
  bool result = manager_->get_waypoint_id_list(list);
  if (result) {
    printf("[%s]: Get waypoint id list failed\n", logger_);
    return 0;
  }

  for (uint32_t i = 0; i < list.size(); i++) {
    uint32_t waypoint_id = list[i];
    point_2d p;
    manager_->get_waypoint(waypoint_id, p);
    double distance = point_distance(current_point, p);
    if (distance < 0.5) {
      printf("[%s]: Nearby waypoint is %d\n", logger_, waypoint_id);
      return waypoint_id;
    }
  }
  printf("[%s]: Current point is not close to the waypoint\n", logger_);
  return 0;
}

bool FollowPathPlanner::is_waypoint(uint32_t id)
{
  point_2d p;
  bool res = manager_->get_waypoint(id, p);
  return res;
}

bool FollowPathPlanner::get_bypassing_obstacle()
{
  return manager_->get_bypassing_obstacle();
}

void FollowPathPlanner::generate_path_list(std::vector<point_2d> & points,
    std::vector<waypoint_2d> & path)
{
  uint32_t len = points.size();
  uint32_t last_id = 0;
  for (uint32_t i = 0; i < len; i++) {
    point_2d p = points[i];
    waypoint_2d wp;
    uint32_t id = manager_->get_waypoint_id(p);
    if (id == 0) {
      wp.waypoint_id = last_id;
    } else {
      wp.waypoint_id = id;
      last_id = id;
    }
    wp.point.x = p.x;
    wp.point.y = p.y;
    wp.point.angle = p.angle;

    path.push_back(wp);
  }

  print_follow_path(path, true);
}

void FollowPathPlanner::find_cloest_point(point_2d & current_point, waypoint_2d & cloest_point)
{
  bool find = false;
  uint32_t len = path_list_.size();
  double shortest_distance = DBL_MAX;
  for (uint32_t i = 0; i < len; i++) {
    waypoint_2d tmp = path_list_[i];
    point_2d p = tmp.point;
    double distance = point_distance(current_point, p);
    if (distance < shortest_distance) {
      cloest_point = tmp;
      shortest_distance = distance;
      find = true;
    }
  }
  if (!find) {
    cloest_point.waypoint_id = 0;
    cloest_point.point.x = 0;
    cloest_point.point.y = 0;
    cloest_point.point.angle = 0;
  }
}

void FollowPathPlanner::print_follow_path(std::vector<waypoint_2d> & path, bool print)
{
  if (!print) {
    return;
  }

  std::string str = "";

  uint32_t len = path.size();
  for (uint32_t i = 0; i < len; i++) {
    waypoint_2d wp = path[i];
    point_2d p = wp.point;
    uint32_t id = wp.waypoint_id;
    str.append("(");

    std::stringstream buf0;
    buf0.setf(std::ios::fixed);
    buf0 << id;
    str.append(buf0.str());
    str.append(",");

    std::stringstream buf;
    buf.precision(2);
    buf.setf(std::ios::fixed);
    buf << p.x;
    str.append(buf.str());
    str.append(",");

    std::stringstream buf1;
    buf1.precision(2);
    buf1.setf(std::ios::fixed);
    buf1 << p.y;
    str.append(buf1.str());
    str.append(",");

    std::stringstream buf2;
    buf2.precision(2);
    buf2.setf(std::ios::fixed);
    buf2 << p.angle;
    str.append(buf2.str());
    str.append(")");
  }
  printf("[%s]: follow_path:%s\n", logger_, str.c_str());
  save_navigation_path(str);
}

void FollowPathPlanner::print_follow_path(std::vector<point_2d> & path, bool print)
{
  if (!print) {
    return;
  }

  std::string str = "";

  uint32_t len = path.size();
  for (uint32_t i = 0; i < len; i++) {
    point_2d p = path[i];

    std::stringstream buf;
    buf.precision(2);
    buf.setf(std::ios::fixed);
    buf << p.x;
    str.append(buf.str());
    str.append(",");

    std::stringstream buf1;
    buf1.precision(2);
    buf1.setf(std::ios::fixed);
    buf1 << p.y;
    str.append(buf1.str());
    str.append(",");

    std::stringstream buf2;
    buf2.precision(2);
    buf2.setf(std::ios::fixed);
    buf2 << p.angle;
    str.append(buf2.str());
    str.append(",");
  }
  printf("[%s]: follow_path:%s\n", logger_, str.c_str());
  save_navigation_path(str);
}

void FollowPathPlanner::print_follow_path(std::vector<uint32_t> & path)
{
  std::string str = "";

  uint32_t len = path.size();
  for (uint32_t i = 0; i < len; i++) {
    uint32_t id = path[i];
    std::stringstream buf;
    buf << id;
    str.append(buf.str());
    str.append(",");
  }
  printf("[%s]: follow_path_waypoint_list:%s\n", logger_, str.c_str());
}

std::string FollowPathPlanner::get_passing_ids(std::vector<uint32_t> & ids)
{
  std::string str = "";

  uint32_t len = ids.size();
  for (uint32_t i = 0; i < len; i++) {
    uint32_t id = ids[i];
    std::stringstream buf;
    buf << id;
    str.append(buf.str());
    str.append(",");
  }
  return str;
}

void FollowPathPlanner::publish_follow_path(std::vector<point_2d> & path)
{
  double x = 0;
  double y = 0;
  if (publish_global_path_cb_ != nullptr) {
    publish_global_path_cb_(path, x, y);
  } else {
    printf("[%s]: publish_global_path_cb_ is nullptr\n", logger_);
  }
}

void FollowPathPlanner::update_current_point(const point_2d point)
{
  std::unique_lock<std::mutex> lk(msg_mutex_);
  current_point_.x = point.x;
  current_point_.y = point.y;
  current_point_.angle = point.angle;
}

double FollowPathPlanner::point_distance(point_2d start_point, point_2d end_point)
{
  return hypot(start_point.x - end_point.x, start_point.y - end_point.y);
}

void FollowPathPlanner::get_current_point(point_2d & point)
{
  std::unique_lock<std::mutex> lk(msg_mutex_);
  point.x = current_point_.x;
  point.y = current_point_.y;
  point.angle = current_point_.angle;
}

bool FollowPathPlanner::arrive_path_end_point(point_2d & current_point,
    std::vector<point_2d> & executing_path)
{
  point_2d end_point = executing_path.at(executing_path.size() - 1);
  double dist = point_distance(current_point, end_point);
  bool result = dist < POSITION_TOLERANCE;
  if (result) {
    printf("[%s]: path include narrow navigation stage finished!\n", logger_);
    return true;
  }
  return false;
}

void FollowPathPlanner::save_navigation_path(std::string & out)
{
  if ((access(nav_path_.c_str(), F_OK) == 0) && std::remove(nav_path_.c_str()) != 0) {
    printf("[%s]: remove %s failed\n", logger_, nav_path_.c_str());
    return;
  }

  std::ofstream os;
  os.open(nav_path_, std::ios::out);
  if (!os.is_open()) {
    printf("[%s]: open file failed\n", logger_);
    return;
  }
  os << out << std::endl;
  os.flush();
  os.close();
  printf("[%s]: finish save file\n", logger_);
}

bool FollowPathPlanner::is_passing_ids_valid(std::vector<uint32_t> & ids)
{
  uint32_t len = ids.size();
  for (uint32_t i = 0; i < len; i++) {
    uint32_t id = ids[i];
    bool res = is_waypoint(id);
    if (!res) {
      printf("[%s]: passing ids(%d) is invalid\n", logger_, id);
      return false;
    }
  }
  return true;
}
}  // namespace navigation
}  // namespace qrb
