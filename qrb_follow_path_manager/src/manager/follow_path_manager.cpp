/*
 * Copyright (c) 2025 Qualcomm Innovation Center, Inc. All rights reserved.
 * SPDX-License-Identifier: BSD-3-Clause-Clear
 */

#include <string.h>
#include <pthread.h>
#include <unistd.h>
#include <sys/socket.h>
#include <sys/un.h>
#include <utility>
#include "../manager/follow_path_manager.hpp"

namespace qrb
{
namespace navigation
{

FollowPathManager::FollowPathManager()
{
  init();
}

FollowPathManager::~FollowPathManager() {}

uint32_t FollowPathManager::add_waypoint(point_2d & point)
{
  printf("[%s]: add_waypoint(%.2f,%.2f,%.2f)\n", logger_, point.x, point.y, point.angle);
  return manager_->add_waypoint(point);
}

bool FollowPathManager::remove_waypoint(uint32_t id)
{
  printf("[%s]: remove_waypoint=%d\n", logger_, id);
  return manager_->remove_waypoint(id);
}

bool FollowPathManager::get_waypoint_id_list(std::vector<uint32_t> & list)
{
  printf("[%s]: get_waypoint_id_list=%ld\n", logger_, list.size());
  return manager_->get_waypoint_id_list(list);
}

bool FollowPathManager::get_waypoint(uint32_t id, point_2d & point)
{
  printf("[%s]: get_waypoint=%d\n", logger_, id);
  return manager_->get_waypoint(id, point);
}

bool FollowPathManager::add_virtual_path(uint32_t id, std::vector<uint32_t> & ids)
{
  printf("[%s]: add_virtual_path=%d\n", logger_, id);
  return manager_->add_virtual_path(id, ids);
}

bool FollowPathManager::remove_virtual_path(uint32_t id, std::vector<uint32_t> & ids)
{
  printf("[%s]: remove_virtual_path=%d\n", logger_, id);
  return manager_->remove_virtual_path(id, ids);
}

bool FollowPathManager::get_virtual_path(uint32_t id, std::vector<uint32_t> & ids)
{
  printf("[%s]: get_virtual_path=%d\n", logger_, id);
  return manager_->get_virtual_path(id, ids);
}

uint32_t FollowPathManager::add_target_waypoint(point_2d & point)
{
  printf("[%s]: add_target_waypoint(%.2f,%.2f,%.2f)\n", logger_, point.x, point.y, point.angle);
  return 0;
}

void FollowPathManager::set_bypassing_obstacle(bool val)
{
  printf("[%s]: set_bypassing_obstacle(%d)\n", logger_, val);
  manager_->set_bypassing_obstacle(val);
}

bool FollowPathManager::remove_waypoint_and_virtual_path()
{
  printf("[%s]: remove_waypoint_and_virtual_path\n", logger_);
  return manager_->remove_waypoint_and_virtual_path();
}

void FollowPathManager::handle_amr_exception(bool exception)
{
  printf("[%s]: handle_amr_exception=%d\n", logger_, exception);
  follow_path_planner_->handle_amr_exception(exception);
}

void FollowPathManager::send_debug_event(int16_t event)
{
  printf("[%s]: send_debug_event=%d\n", logger_, event);
  robot_ctrl_->send_debug_event(event);
}

void FollowPathManager::handle_emergency(bool enter)
{
  printf("[%s]: handle_emergency=%d\n", logger_, enter);
}

void FollowPathManager::request_pause_follow_path()
{
  printf("[%s]: request_pause_follow_path\n", logger_);
  follow_path_planner_->request_pause_follow_path();
}

void FollowPathManager::request_resume_follow_path()
{
  printf("[%s]: request_resume_follow_path\n", logger_);
  follow_path_planner_->request_resume_follow_path();
}

uint64_t FollowPathManager::request_follow_path(uint32_t goal, std::vector<uint32_t> & passing_ids)
{
  printf("[%s]: request_follow_path\n", logger_);
  return follow_path_planner_->request_follow_path(goal, passing_ids);
}

float FollowPathManager::get_distance_to_goal(point_2d & current_point, uint32_t passing_id)
{
  return follow_path_planner_->get_distance_to_goal(current_point, passing_id);
}

uint32_t FollowPathManager::get_passing_waypoint_id(point_2d & current_point)
{
  return follow_path_planner_->get_passing_waypoint_id(current_point);
}

void FollowPathManager::request_stop_follow_path()
{
  printf("[%s]: request_stop_follow_path\n", logger_);
  follow_path_planner_->request_stop_navigation(false);
}

void FollowPathManager::register_navigation_callback(navigation_completed_func_t cb)
{
  follow_path_planner_->register_navigation_callback(cb);
}

void FollowPathManager::update_current_pose(point_2d & point)
{
  // printf("[%s]: update_current_pose\n", logger_);
  follow_path_planner_->update_current_pose(point);
}

void FollowPathManager::register_publish_real_path_callback(publish_real_path_func_t cb)
{
  follow_path_planner_->register_publish_real_path_callback(cb);
}

void FollowPathManager::register_publish_global_path_callback(publish_global_path_func_t cb)
{
  follow_path_planner_->register_publish_global_path_callback(cb);
}

void FollowPathManager::register_publish_twist_callback(publish_twist_func_t cb)
{
  follow_path_planner_->register_publish_twist_callback(cb);
}

void FollowPathManager::register_get_robot_velocity_callback(get_robot_velocity_func_t cb)
{
  follow_path_planner_->register_get_robot_velocity_callback(cb);
}

void FollowPathManager::register_get_grid_map_callback(get_grid_map_func_t cb)
{
  manager_->register_get_grid_map_callback(cb);
  detector_->register_get_grid_map_callback(cb);
}

bool FollowPathManager::compute_follow_path(bool use_start,
    uint32_t start,
    uint32_t goal,
    std::vector<uint32_t> & list,
    std::vector<point_2d> & point_list)
{
  printf("[%s]: compute_follow_path\n", logger_);
  return follow_path_planner_->compute_follow_path(use_start, start, goal, list, point_list);
}

bool FollowPathManager::compute_follow_path(bool use_start,
    uint32_t start,
    uint32_t goal,
    std::vector<uint32_t> & passing_ids,
    std::vector<uint32_t> & list,
    std::vector<point_2d> & point_list)
{
  printf("[%s]: compute_follow_path(passing waypoint)\n", logger_);
  return follow_path_planner_->compute_follow_path(
      use_start, start, goal, passing_ids, list, point_list);
}

void FollowPathManager::init()
{
  robot_ctrl_ = std::shared_ptr<PidController>(new PidController());

  manager_ = std::shared_ptr<VirtualPathManager>(new VirtualPathManager());

  detector_ = std::shared_ptr<ObstacleDetector>(new ObstacleDetector());

  follow_path_planner_ =
      std::shared_ptr<FollowPathPlanner>(new FollowPathPlanner(manager_, robot_ctrl_, detector_));
}
}  // namespace navigation
}  // namespace qrb
