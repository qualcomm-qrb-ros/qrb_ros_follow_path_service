/*
 * Copyright (c) 2025 Qualcomm Innovation Center, Inc. All rights reserved.
 * SPDX-License-Identifier: BSD-3-Clause-Clear
 */

#include "virtual_path/virtual_path_manager.hpp"
#include <cmath>
#include <memory>

using namespace std;

namespace qrb
{
namespace navigation
{

VirtualPathManager::VirtualPathManager()
{
  printf("[%s]: Create VirtualPathManager\n", logger_);
  virtual_path_helper_ =
      std::make_shared<VirtualPathHelper>(VIRTUAL_PATH_FILE_PATH, VIRTUAL_PATH_BAKEUP_FILE_PATH);
  virtual_path_helper_->load(virtual_path_, bypassing_);
}

VirtualPathManager::~VirtualPathManager()
{
  printf("[%s]: Destory VirtualPathManager\n", logger_);
  virtual_path_helper_->save(virtual_path_);
}

void VirtualPathManager::register_get_grid_map_callback(get_grid_map_func_t cb)
{
  printf("[%s]: register_get_grid_map_callback\n", logger_);
  get_grid_map_cb_ = cb;
}

bool VirtualPathManager::is_valid(point_2d & point)
{
  if (!map_exist()) {
    printf("[%s]: Current point is invalid because map does not exsit\n", logger_);
    return false;
  }

  if (is_unknown_cell(point)) {
    printf("[%s]: Current point is invalid because it is unknown\n", logger_);
    return false;
  }

  if (is_within_obstacle(point)) {
    printf("[%s]: Current point is invalid because it is an obstacle\n", logger_);
    return false;
  }

  return true;
}

uint32_t VirtualPathManager::add_waypoint(point_2d & point)
{
  if (!map_exist()) {
    printf("[%s]: Add waypoint failed because map does not exsit\n", logger_);
    return 0;
  }

  if (is_unknown_cell(point)) {
    printf("[%s]: Add waypoint failed because point is unknown\n", logger_);
    return 0;
  }

  if (is_within_obstacle(point)) {
    printf("[%s]: Add waypoint failed because point is an obstacle\n", logger_);
    return 0;
  }

  if (is_nearby_obstacle(point)) {
    printf("[%s]: Add waypoint failed because point is close to the obstacle\n", logger_);
    return 0;
  }

  if (near_waypoint(point)) {
    printf("[%s]: Add waypoint failed because point is close to the waypoint\n", logger_);
    return 0;
  }

  if (is_waypoint(point)) {
    return get_waypoint_id(point);
  }

  uint32_t id = get_new_waypoint_id();
  if (id == 0) {
    printf("[%s]: Add waypoint failed because the id is 0\n", logger_);
    return 0;
  }

  printf("[%s]: Create waypoint(%f,%f,%f) id=%d\n", logger_, point.x, point.y, point.angle, id);

  waypoint info;
  info.id = id;
  info.is_target = false;
  info.point.x = point.x;
  info.point.y = point.y;
  info.point.angle = point.angle;
  virtual_path_.emplace(id, std::move(info));
  save_virtual_path();
  return id;
}

bool VirtualPathManager::remove_waypoint(uint32_t id)
{
  if (!map_exist()) {
    printf("[%s]: Remove waypoint failed because map does not exsit\n", logger_);
    return false;
  }

  int count = virtual_path_.erase(id);
  if (count > 0) {
    remove_adjacent_waypoint(id);
    save_virtual_path();
    return true;
  }
  return false;
}

bool VirtualPathManager::get_waypoint_id_list(vector<uint32_t> & list)
{
  if (!map_exist()) {
    printf("[%s]: Get waypoint id list failed because map does not exsit\n", logger_);
    return false;
  }

  for (auto & info : virtual_path_) {
    list.push_back(info.second.id);
  }
  if (list.size() != 0) {
    return true;
  } else {
    return false;
  }
}

bool VirtualPathManager::get_waypoint(uint32_t id, point_2d & point)
{
  if (!map_exist()) {
    printf("[%s]: Get waypoint failed because map does not exsit\n", logger_);
    return false;
  }

  auto info = virtual_path_.find(id);
  if (info == virtual_path_.end()) {
    return false;
  }
  point.x = info->second.point.x;
  point.y = info->second.point.y;
  point.angle = info->second.point.angle;
  return true;
}

bool VirtualPathManager::get_waypoint(uint32_t id,
    map<uint32_t, waypoint> & virtual_path,
    point_2d & point)
{
  auto info = virtual_path.find(id);
  if (info == virtual_path.end()) {
    return false;
  }
  point.x = info->second.point.x;
  point.y = info->second.point.y;
  point.angle = info->second.point.angle;
  return true;
}

bool VirtualPathManager::add_virtual_path(uint32_t id, vector<uint32_t> & ids)
{
  if (!map_exist()) {
    printf("[%s]: Add virtual path failed because map does not exsit\n", logger_);
    return false;
  }

  auto info = virtual_path_.find(id);
  if (info == virtual_path_.end()) {
    printf("[%s]: no waypoint id = %d\n", logger_, id);
    return false;
  }

  uint32_t add_len = ids.size();
  for (uint32_t i = 0; i < add_len; i++) {
    uint32_t adj_id = ids[i];
    if (!is_waypoint_id(adj_id)) {
      printf("[%s]: no adjacent waypoint id = %d\n", logger_, adj_id);
      return false;
    }
  }

  vector<uint32_t> adjacent_waypoints = info->second.adjacent_waypoints;
  uint32_t len = adjacent_waypoints.size();
  for (uint32_t i = 0; i < add_len; i++) {
    uint32_t add_id = ids[i];
    bool add_flag = true;
    for (uint32_t j = 0; j < len; j++) {
      if (add_id == adjacent_waypoints[j]) {
        add_flag = false;
        break;
      }
    }
    if (add_flag) {
      if (is_path_nearby_obstacle(id, add_id)) {
        printf("[%s]: the virutal path(%d->%d) is nearby obstacle\n", logger_, id, add_id);
        return false;
      }
      info->second.adjacent_waypoints.push_back(add_id);
      printf("[%s]: add adjacent waypoint id = %d\n", logger_, add_id);
      add_two_way_virtual_path(add_id, id);
    }
  }

  save_virtual_path();
  return true;
}

bool VirtualPathManager::force_add_virtual_path(uint32_t id, vector<uint32_t> & ids)
{
  if (!map_exist()) {
    printf("[%s]: Force add virtual path failed because map does not exsit\n", logger_);
    return false;
  }

  auto info = virtual_path_.find(id);
  if (info == virtual_path_.end()) {
    printf("[%s]: no waypoint id = %d\n", logger_, id);
    return false;
  }

  uint32_t add_len = ids.size();
  for (uint32_t i = 0; i < add_len; i++) {
    uint32_t adj_id = ids[i];
    if (!is_waypoint_id(adj_id)) {
      printf("[%s]: no adjacent waypoint id = %d\n", logger_, adj_id);
      return false;
    }
  }

  vector<uint32_t> adjacent_waypoints = info->second.adjacent_waypoints;
  uint32_t len = adjacent_waypoints.size();
  for (uint32_t i = 0; i < add_len; i++) {
    uint32_t add_id = ids[i];
    bool add_flag = true;
    for (uint32_t j = 0; j < len; j++) {
      if (add_id == adjacent_waypoints[j]) {
        add_flag = false;
        break;
      }
    }
    if (add_flag) {
      info->second.adjacent_waypoints.push_back(add_id);
      printf("[%s]: add adjacent waypoint id = %d\n", logger_, add_id);
      add_two_way_virtual_path(add_id, id);
    }
  }

  save_virtual_path();
  return true;
}

void VirtualPathManager::add_two_way_virtual_path(uint32_t id, uint32_t adjacent_id)
{
  auto info = virtual_path_.find(id);
  if (info == virtual_path_.end()) {
    printf("[%s]: add_two_way_virtual_path, no waypoint id = %d\n", logger_, id);
    return;
  }

  if (!is_waypoint_id(adjacent_id)) {
    printf("[%s]: add_two_way_virtual_path, no adjacent waypoint id = %d\n", logger_, id);
    return;
  }

  vector<uint32_t> adjacent_waypoints = info->second.adjacent_waypoints;
  uint32_t len = adjacent_waypoints.size();
  bool add_flag = true;
  for (uint32_t j = 0; j < len; j++) {
    if (adjacent_id == adjacent_waypoints[j]) {
      add_flag = false;
      break;
    }
  }
  if (add_flag) {
    info->second.adjacent_waypoints.push_back(adjacent_id);
    printf("[%s]: add_two_way_virtual_path, add adjacent waypoint id = %d\n", logger_, adjacent_id);
    return;
  }
  printf("[%s]: this virtual path exist, ignore\n", logger_);
}

void VirtualPathManager::remove_two_way_virtual_path(uint32_t id, uint32_t adjacent_id)
{
  auto info = virtual_path_.find(id);
  if (info == virtual_path_.end()) {
    printf("[%s]: remove_two_way_virtual_path, no waypoint id = %d\n", logger_, id);
    return;
  }

  if (!is_waypoint_id(adjacent_id)) {
    printf("[%s]: remove_two_way_virtual_path, no adjacent waypoint id = %d\n", logger_, id);
    return;
  }

  for (auto iter = info->second.adjacent_waypoints.begin();
       iter != info->second.adjacent_waypoints.end(); iter++) {
    if (*iter == adjacent_id) {
      iter = info->second.adjacent_waypoints.erase(iter);
      printf("[%s]: remove adjacent waypoint id = %d\n", logger_, adjacent_id);
      return;
    }
  }
}

bool VirtualPathManager::remove_virtual_path(uint32_t id, vector<uint32_t> & ids)
{
  if (!map_exist()) {
    printf("[%s]: Remove virtual path failed because map does not exsit\n", logger_);
    return false;
  }

  auto info = virtual_path_.find(id);
  if (info == virtual_path_.end()) {
    printf("[%s]: no waypoint id = %d\n", logger_, id);
    return false;
  }

  uint32_t remove_len = ids.size();
  printf("[%s]: remove_len = %d\n", logger_, remove_len);
  for (uint32_t i = 0; i < remove_len; i++) {
    uint32_t adj_id = ids[i];
    printf("[%s]: adjacent waypoint id = %d\n", logger_, adj_id);
    if (!is_waypoint_id(adj_id)) {
      printf("[%s]: no adjacent waypoint id = %d\n", logger_, adj_id);
      return false;
    }
  }

  for (uint32_t i = 0; i < remove_len; i++) {
    uint32_t remove_id = ids[i];
    for (auto iter = info->second.adjacent_waypoints.begin();
         iter != info->second.adjacent_waypoints.end(); iter++) {
      if (*iter == remove_id) {
        iter = info->second.adjacent_waypoints.erase(iter);
        printf("[%s]: remove adjacent waypoint id = %d\n", logger_, remove_id);
        remove_two_way_virtual_path(remove_id, id);
        break;
      }
    }
  }
  save_virtual_path();
  return true;
}

bool VirtualPathManager::remove_waypoint_and_virtual_path()
{
  std::map<uint32_t, waypoint>::iterator it = virtual_path_.begin();
  while (it != virtual_path_.end()) {
    it = virtual_path_.erase(it);
  }

  save_virtual_path();
  return true;
}

void VirtualPathManager::remove_adjacent_waypoint(uint32_t remove_id)
{
  for (auto & info : virtual_path_) {
    uint32_t id = info.second.id;
    vector<uint32_t> adjacent_waypoints = info.second.adjacent_waypoints;
    uint32_t len = adjacent_waypoints.size();
    for (uint32_t i = 0; i < len; i++) {
      for (auto iter = info.second.adjacent_waypoints.begin();
           iter != info.second.adjacent_waypoints.end(); iter++) {
        if (*iter == remove_id) {
          iter = info.second.adjacent_waypoints.erase(iter);
          printf("[%s]: remove virtual path(%d -> %d)\n", logger_, id, remove_id);
          break;
        }
      }
    }
  }
}

bool VirtualPathManager::get_virtual_path(uint32_t id, vector<uint32_t> & ids)
{
  if (!map_exist()) {
    printf("[%s]: Get virtual path failed because map does not exsit\n", logger_);
    return false;
  }

  auto info = virtual_path_.find(id);
  if (info == virtual_path_.end()) {
    return false;
  }
  vector<uint32_t> adjacent_waypoints = info->second.adjacent_waypoints;
  uint32_t len = adjacent_waypoints.size();
  for (uint32_t i = 0; i < len; i++) {
    ids.push_back(adjacent_waypoints[i]);
  }
  return true;
}

void VirtualPathManager::set_bypassing_obstacle(bool val)
{
  if (!map_exist()) {
    printf("[%s]: Set bypassing obstacle failed because map does not exsit\n", logger_);
    return;
  }

  bypassing_ = val;
  save_bypassing_obstacle(val);
}

bool VirtualPathManager::get_bypassing_obstacle()
{
  return bypassing_;
}

void VirtualPathManager::find_follow_path(point_2d & start_point,
    point_2d & end_point,
    vector<sub_path_info> & path)
{
  if (!is_waypoint(end_point)) {
    printf("[%s]: Target(%f,%f,%f) is not waypoint and find the follow path failed\n", logger_,
        end_point.x, end_point.y, end_point.angle);
    return;
  }
  waypoint end_waypoint;
  find_waypoint(end_point, end_waypoint);

  if (!near_waypoint(start_point)) {
    printf(
        "[%s]: Start(%f,%f,%f) is not nearby a waypoint and find the follow "
        "path failed\n",
        logger_, start_point.x, start_point.y, start_point.angle);
    return;
  }

  waypoint start_waypoint;
  find_waypoint(start_point, start_waypoint);

  printf(
      "[%s]: Find follow path from start(%f,%f)/(id=%d,%f,%f) to "
      "end(%f,%f)/(id=%d,%f,%f)\n",
      logger_, start_point.x, start_point.y, start_waypoint.id, start_waypoint.point.x,
      start_waypoint.point.y, end_point.x, end_point.y, end_waypoint.id, end_waypoint.point.x,
      end_waypoint.point.y);

  find_virtual_navigation_path(start_waypoint, end_waypoint, virtual_path_, path);
}

void VirtualPathManager::compute_follow_path(point_2d & start_point,
    point_2d & end_point,
    vector<sub_path_info> & path,
    vector<uint32_t> & list)
{
  if (!is_waypoint(start_point)) {
    printf("[%s]: Start(%f,%f,%f) is not waypoint and find the follow path failed\n", logger_,
        start_point.x, start_point.y, start_point.angle);
    return;
  }

  if (!is_waypoint(end_point)) {
    printf("[%s]: Target(%f,%f,%f) is not waypoint and find the follow path failed\n", logger_,
        end_point.x, end_point.y, end_point.angle);
    return;
  }

  waypoint end_waypoint;
  find_waypoint(end_point, end_waypoint);

  waypoint start_waypoint;
  find_waypoint(start_point, start_waypoint);

  printf(
      "[%s]: Compute follow path from start(%f,%f)/(id=%d,%f,%f) to "
      "end(%f,%f)/(id=%d,%f,%f)\n",
      logger_, start_point.x, start_point.y, start_waypoint.id, start_waypoint.point.x,
      start_waypoint.point.y, end_point.x, end_point.y, end_waypoint.id, end_waypoint.point.x,
      end_waypoint.point.y);

  get_virtual_navigation_path(start_waypoint, end_waypoint, virtual_path_, path, list);
}

void VirtualPathManager::save_virtual_path()
{
  print_virtual_path();
  bool result = virtual_path_helper_->save(virtual_path_);
  printf("[%s]: Save virtual path %s\n", logger_, result ? "seccussfully" : "failed");
}

void VirtualPathManager::save_bypassing_obstacle(bool val)
{
  bool result = virtual_path_helper_->save(virtual_path_, val);
  printf("[%s]: Save virtual path %s\n", logger_, result ? "seccussfully" : "failed");
}

bool VirtualPathManager::is_waypoint(point_2d & point)
{
  for (auto & info : virtual_path_) {
    point_2d waypoint = info.second.point;
    if (is_same_position(point, waypoint)) {
      return true;
    }
  }
  return false;
}

uint32_t VirtualPathManager::get_waypoint_id(point_2d & point)
{
  for (auto & info : virtual_path_) {
    point_2d waypoint = info.second.point;
    if (is_same_position(point, waypoint)) {
      return info.second.id;
    }
  }
  return 0;
}

uint32_t VirtualPathManager::get_nearby_waypoint(point_2d & current_point, point_2d & waypoint)
{
  for (auto & info : virtual_path_) {
    point_2d p = info.second.point;
    double dist = compute_dist(current_point, p);
    if (dist < NEARBY_WAYPOINT_MAX_DIST) {
      waypoint.x = p.x;
      waypoint.y = p.y;
      waypoint.angle = p.angle;
      return info.second.id;
    }
  }
  return 0;
}

bool VirtualPathManager::is_same_position(point_2d & p1, point_2d & p2)
{
  if ((p1.x == p2.x) && (p1.y == p2.y)) {
    return true;
  }
  return false;
}

bool VirtualPathManager::within_range(point_2d & p1, point_2d & p2)
{
  double dist = compute_dist(p1, p2);
  if (dist < 0.2) {
    return true;
  }
  printf("[%s]: p(%f,%f,%f) is not within the range of p(%f,%f,%f), dist=%f\n", logger_, p1.x, p1.y,
      p1.angle, p2.x, p2.y, p2.angle, dist);
  return false;
}

bool VirtualPathManager::generate_path(vector<uint32_t> & ids, vector<sub_path_info> & path)
{
  vector<point_2d> waypoints;
  uint32_t len = ids.size();
  for (uint32_t i = 0; i < len; i++) {
    point_2d p;
    get_waypoint(ids[i], p);
    waypoints.push_back(p);
  }

  return generate_path(waypoints, path);
}

bool VirtualPathManager::generate_path(vector<uint32_t> & ids,
    map<uint32_t, waypoint> & virtual_path,
    vector<sub_path_info> & path)
{
  vector<point_2d> waypoints;
  uint32_t len = ids.size();
  for (uint32_t i = 0; i < len; i++) {
    point_2d p;
    get_waypoint(ids[i], virtual_path, p);
    waypoints.push_back(p);
  }

  return generate_path(waypoints, path);
}

bool VirtualPathManager::generate_path(vector<point_2d> & waypoints, vector<sub_path_info> & path)
{
  uint32_t len = waypoints.size();
  if (len < 2) {
    printf("[%s]: waypoint(len=%d) is invalid, generate follow path failed\n", logger_, len);
    return false;
  }

  vector<point_2d> sub_path;
  sub_path_info info;
  point_2d p;
  p.x = waypoints[0].x;
  p.y = waypoints[0].y;
  p.angle = waypoints[0].angle;

  // push the start point
  sub_path.push_back(p);

  for (uint32_t i = 0; i < (len - 1); i++) {
    point_2d p1 = waypoints[i];
    point_2d p2 = waypoints[i + 1];
    compute_path(p1, p2, sub_path, VIRTUAL_PATH_MAX_INTERVAL_DIST);
    // push the last sub path after adding the end point
    if (i < (len - 2)) {
      info.index = i;
      info.path = sub_path;
      path.push_back(info);
      sub_path.clear();
    }
  }

  p.x = waypoints[len - 1].x;
  p.y = waypoints[len - 1].y;
  p.angle = waypoints[len - 1].angle;

  // push the end point
  sub_path.push_back(p);
  info.index = len - 2;
  info.path = sub_path;
  path.push_back(info);

  return true;
}

void VirtualPathManager::compute_path(point_2d & start,
    point_2d & end,
    vector<point_2d> & sub_path,
    double max_dist)
{
  compute_count_ = 0;
  // confirm the angle of waypoints
  compute_angle(start, end);
  // compute the path of two waypoints
  vector<point_2d> tmp;
  tmp.push_back(start);
  tmp.push_back(end);
  double dist = compute_dist(start, end);
  if (dist >= max_dist) {
    compute_and_insert_point(tmp, max_dist);
  }

  // add points to follow path
  uint32_t len = tmp.size();
  for (uint32_t i = 0; i < len; i++) {
    point_2d p;
    p.x = tmp[i].x;
    p.y = tmp[i].y;
    p.angle = tmp[i].angle;
    sub_path.push_back(p);
  }
}

void VirtualPathManager::compute_and_insert_point(vector<point_2d> & points, double max_dist)
{
  vector<point_2d> tmp;
  tmp.assign(points.begin(), points.end());
  compute_count_++;
  uint32_t len = tmp.size();
  for (uint32_t i = 0; i < (len - 1); i++) {
    point_2d start = tmp[i];
    point_2d end = tmp[i + 1];
    point_2d middle_point;
    compute_middle_point_and_angle(start, end, middle_point);
    uint32_t index = get_insert_index(points, end);
    if (index != 0) {
      insert_point(middle_point, points, index);
    } else {
      printf("[%s]: Index is invalid, ignore.............\n", logger_);
      continue;
    }
  }

  print_path(points, false);

  // continue compute if the distance of two points is less than the value
  double dist = compute_dist(points[0], points[1]);
  if (compute_count_ > 100) {
    printf("[%s]: dist=%f, complete follow path calculation(%d)\n", logger_, dist, compute_count_);
    return;
  }
  if (dist >= max_dist) {
    compute_and_insert_point(points, max_dist);
  } else {
    printf("[%s]: dist=%f, complete follow path calculation\n", logger_, dist);
  }
}

void VirtualPathManager::compute_middle_point_and_angle(point_2d & start,
    point_2d & end,
    point_2d & point)
{
  point.x = (start.x + end.x) / 2;
  point.y = (start.y + end.y) / 2;
  double angle = get_angle_of_two_points(start, end);
  point.angle = angle;
  start.angle = angle;
  end.angle = angle;
}

void VirtualPathManager::compute_angle(point_2d & start, point_2d & end)
{
  double angle = get_angle_of_two_points(start, end);
  start.angle = angle;
  end.angle = angle;
}

// insert point before the index
void VirtualPathManager::insert_point(point_2d & point, vector<point_2d> & points, uint32_t index)
{
  points.insert(points.begin() + index, point);
}

double VirtualPathManager::compute_dist(point_2d & first_point, point_2d & second_point)
{
  double delta_x = second_point.x - first_point.x;
  double delta_y = second_point.y - first_point.y;
  double dist = hypot(delta_x, delta_y);
  return dist;
}

double VirtualPathManager::get_angle_of_two_points(point_2d & first_point, point_2d & second_point)
{
  double target_angle;
  double val;
  double delta_x = second_point.x - first_point.x;
  double delta_y = second_point.y - first_point.y;

  double delta_xy = hypot(delta_x, delta_y);
  if (delta_xy != 0) {
    val = asin(delta_y / delta_xy);
  } else {
    val = M_PI_2;
  }
  if ((delta_x < 0) && (delta_y >= 0)) {
    // Quadrant 2
    target_angle = M_PI - val;
  } else if ((delta_x < 0) && (delta_y < 0)) {
    // Quadrant 3
    target_angle = -M_PI - val;
  } else {
    // Quadrant 1&4
    target_angle = val;
  }
  return target_angle;
}

uint32_t VirtualPathManager::get_new_waypoint_id()
{
  uint32_t tmp[VIRTUAL_PATH_ID_MAX_INDEX + 1] = { 0 };
  for (auto & info : virtual_path_) {
    uint32_t id = info.first;
    tmp[id] = id;
  }
  for (uint32_t i = VIRTUAL_PATH_ID_START_INDEX; i <= VIRTUAL_PATH_ID_MAX_INDEX; i++) {
    if (tmp[i] == 0) {
      return i;
    }
  }
  return 0;
}

uint32_t VirtualPathManager::get_insert_index(vector<point_2d> & points, point_2d & point)
{
  uint32_t len = points.size();
  for (uint32_t i = 0; i < len; i++) {
    point_2d p = points[i];
    if (is_same_position(p, point)) {
      return i;
    }
  }
  return 0;
}

void VirtualPathManager::print_path(vector<sub_path_info> & path, bool print)
{
  if (!print) {
    return;
  }

  uint32_t len = path.size();
  for (uint32_t i = 0; i < len; i++) {
    vector<point_2d> sub_path = path[i].path;
    uint32_t sub_len = sub_path.size();
    printf("[%s]: ===sub path%d===\n", logger_, i);
    for (uint32_t j = 0; j < sub_len; j++) {
      printf(
          "[%s]: p%d=(%f, %f, %f)\n", logger_, j, sub_path[j].x, sub_path[j].y, sub_path[j].angle);
    }
  }
}

void VirtualPathManager::print_path(vector<point_2d> & path, bool print)
{
  if (!print) {
    return;
  }
  printf("[%s]: point_2d===%d===\n", logger_, compute_count_);
  uint32_t len = path.size();
  for (uint32_t i = 0; i < len; i++) {
    printf("[%s]: p%d=(%f, %f, %f)\n", logger_, i, path[i].x, path[i].y, path[i].angle);
  }
}

void VirtualPathManager::print_virtual_navigation_path(vector<uint32_t> & wps, bool print)
{
  if (!print) {
    return;
  }

  uint32_t len = wps.size();
  string str = "";
  for (uint32_t i = 0; i < len; i++) {
    str.append(to_string(wps[i]));
    if (i != (len - 1)) {
      str.append(",");
    }
  }
  printf("[%s]: virtual nav path:%s\n", logger_, str.c_str());
}

void VirtualPathManager::find_waypoint(point_2d & point, waypoint & wp)
{
  printf("[%s]: Find waypoint(%f,%f)\n", logger_, point.x, point.y);
  point_2d real_closest_point;
  uint32_t real_closest_start_wp_id = 0;
  uint32_t real_closest_end_wp_id = 0;

  for (auto & info : virtual_path_) {
    point_2d p = info.second.point;
    if (is_same_position(point, p)) {
      copy_waypoint(info.second, wp);
      printf("[%s]: Current position is a waypoint(id=%d,is_target=%d,point=%f,%f,%f)\n", logger_,
          wp.id, wp.is_target, wp.point.x, wp.point.y, wp.point.angle);
      return;
    }

    printf("[%s]: Finding the same waypoint(id=%d)\n", logger_, info.second.id);
  }

  for (auto & info : virtual_path_) {
    point_2d p = info.second.point;
    if (within_range(point, p)) {
      copy_waypoint(info.second, wp);
      printf("[%s]: Current position is a waypoint(id=%d,is_target=%d,point=%f,%f,%f)\n", logger_,
          wp.id, wp.is_target, wp.point.x, wp.point.y, wp.point.angle);
      return;
    }

    printf("[%s]: Finding nearby waypoint(id=%d)\n", logger_, info.second.id);

    vector<uint32_t> adjacent_waypoints = info.second.adjacent_waypoints;
    point_2d closest_point;
    uint32_t start_id = info.second.id;
    uint32_t len = adjacent_waypoints.size();
    for (uint32_t i = 0; i < len; i++) {
      uint32_t end_id = adjacent_waypoints[i];
      get_closest_point(point, p, end_id, closest_point);

      if (compute_dist(point, real_closest_point) > compute_dist(point, closest_point)) {
        real_closest_point.x = closest_point.x;
        real_closest_point.y = closest_point.y;
        real_closest_point.angle = closest_point.angle;
        real_closest_start_wp_id = start_id;
        real_closest_end_wp_id = end_id;
      }
    }
  }

  // the cloest point is a waypoint
  if (is_waypoint(real_closest_point)) {
    uint32_t id = get_waypoint_id(real_closest_point);
    auto info_ptr = virtual_path_.find(id);
    if (info_ptr == virtual_path_.end()) {
      printf("[%s]: Closest point(%f,%f,%f) is not a waypoint(id=%d)\n", logger_,
          real_closest_point.x, real_closest_point.y, real_closest_point.angle, id);
      return;
    }
    copy_waypoint(info_ptr->second, wp);
    printf("[%s]: Find closest point(%f,%f,%f) is a waypoint(id=%d)\n", logger_, wp.point.x,
        wp.point.y, wp.point.angle, wp.id);
    return;
  }

  wp.id = get_new_waypoint_id();
  wp.is_target = false;
  wp.point.x = real_closest_point.x;
  wp.point.y = real_closest_point.y;
  wp.point.angle = real_closest_point.angle;
  wp.adjacent_waypoints.push_back(real_closest_start_wp_id);
  wp.adjacent_waypoints.push_back(real_closest_end_wp_id);

  printf("[%s]: Find new waypoint(id=%d,point=%f,%f,%f,adjacent_wp=%d,%d)\n", logger_, wp.id,
      wp.point.x, wp.point.y, wp.point.angle, real_closest_start_wp_id, real_closest_end_wp_id);
}

void VirtualPathManager::add_waypoint_for_navigation(waypoint & start_waypoint,
    map<uint32_t, waypoint> & virtual_path)
{
  copy(virtual_path_.begin(), virtual_path_.end(), inserter(virtual_path, virtual_path.begin()));
  virtual_path.emplace(start_waypoint.id, std::move(start_waypoint));
}

void VirtualPathManager::find_virtual_navigation_path(waypoint & start_waypoint,
    waypoint & end_waypoint,
    map<uint32_t, waypoint> & virtual_path,
    vector<sub_path_info> & path)
{
  const uint32_t start_id = get_waypoint_id(start_waypoint.point, virtual_path);
  const uint32_t end_id = get_waypoint_id(end_waypoint.point, virtual_path);

  // initialize matrix
  const uint32_t waypoint_count = get_waypoint_max_id(virtual_path);
  // double dist[waypoint_count + 1][waypoint_count + 1] = {0.0};
  const uint64_t length = (waypoint_count + 1) * (waypoint_count + 1);
  dist_ = new double[length];
  for (uint32_t i = 1; i <= waypoint_count; i++) {
    for (uint32_t j = 1; j <= waypoint_count; j++) {
      if (is_adjacent(i, j, virtual_path)) {
        point_2d pi = virtual_path.at(i).point;
        point_2d pj = virtual_path.at(j).point;
        // dist[i][j] = compute_dist(pi, pj);
        *(dist_ + i * (waypoint_count + 1) + j) = compute_dist(pi, pj);
      } else {
        // dist[i][j] = VIRTUAL_PATH_DISTANCE_MAX;
        *(dist_ + i * (waypoint_count + 1) + j) = VIRTUAL_PATH_DISTANCE_MAX;
      }
    }
    // dist[i][i] = 0;
    *(dist_ + i * (waypoint_count + 1) + i) = 0;
  }

  for (uint32_t i = 1; i <= waypoint_count; i++) {
    for (uint32_t j = 1; j <= waypoint_count; j++) {
      printf(
          "[%s]: src:%d-->%d, dist=%lf\n", logger_, i, j, *(dist_ + i * (waypoint_count + 1) + j));
    }
  }

  // find the shortest path
  shortest_dist_ = VIRTUAL_PATH_DISTANCE_MAX;
  end_waypoint_id_ = end_id;
  waypoint_count_ = waypoint_count;
  waypoint_mark_ = new char[waypoint_count + 1];
  memset(waypoint_mark_, 0, waypoint_count + 1);
  navigation_waypoint_ids_.clear();
  // dist_ = (double *)dist;
  //  mark the start point
  waypoint_mark_[start_id] = 1;
  vector<uint32_t> start_path;
  start_path.push_back(start_id);
  deep_first_search(start_id, 0, start_path);

  print_virtual_navigation_path(navigation_waypoint_ids_, true);

  bool res = generate_path(navigation_waypoint_ids_, virtual_path, path);
  if (res) {
    printf("[%s]: Find virtual navigation path success\n", logger_);
  } else {
    printf("[%s]: Find virtual navigation path failed\n", logger_);
  }
  print_path(path, true);
  delete[] waypoint_mark_;
  delete[] dist_;
}

void VirtualPathManager::get_virtual_navigation_path(waypoint & start_waypoint,
    waypoint & end_waypoint,
    std::map<uint32_t, waypoint> & virtual_path,
    std::vector<sub_path_info> & path,
    std::vector<uint32_t> & list)
{
  const uint32_t start_id = get_waypoint_id(start_waypoint.point, virtual_path);
  const uint32_t end_id = get_waypoint_id(end_waypoint.point, virtual_path);

  // initialize matrix
  const uint32_t waypoint_count = get_waypoint_max_id(virtual_path);
  // double dist[waypoint_count + 1][waypoint_count + 1] = {0.0};
  const uint64_t length = (waypoint_count + 1) * (waypoint_count + 1);
  dist_ = new double[length];
  for (uint32_t i = 1; i <= waypoint_count; i++) {
    for (uint32_t j = 1; j <= waypoint_count; j++) {
      if (is_adjacent(i, j, virtual_path)) {
        point_2d pi = virtual_path.at(i).point;
        point_2d pj = virtual_path.at(j).point;
        // dist[i][j] = compute_dist(pi, pj);
        *(dist_ + i * (waypoint_count + 1) + j) = compute_dist(pi, pj);
      } else {
        // dist[i][j] = VIRTUAL_PATH_DISTANCE_MAX;
        *(dist_ + i * (waypoint_count + 1) + j) = VIRTUAL_PATH_DISTANCE_MAX;
      }
    }
    // dist[i][i] = 0;
    *(dist_ + i * (waypoint_count + 1) + i) = 0;
  }

  for (uint32_t i = 1; i <= waypoint_count; i++) {
    for (uint32_t j = 1; j <= waypoint_count; j++) {
      printf(
          "[%s]: src:%d-->%d, dist=%lf\n", logger_, i, j, *(dist_ + i * (waypoint_count + 1) + j));
    }
  }

  // find the shortest path
  shortest_dist_ = VIRTUAL_PATH_DISTANCE_MAX;
  end_waypoint_id_ = end_id;
  waypoint_count_ = waypoint_count;
  waypoint_mark_ = new char[waypoint_count + 1];
  memset(waypoint_mark_, 0, waypoint_count + 1);
  navigation_waypoint_ids_.clear();
  // dist_ = (double *)dist;
  //  mark the start point
  waypoint_mark_[start_id] = 1;
  vector<uint32_t> start_path;
  start_path.push_back(start_id);
  deep_first_search(start_id, 0, start_path);

  print_virtual_navigation_path(navigation_waypoint_ids_, true);
  list.assign(navigation_waypoint_ids_.begin(), navigation_waypoint_ids_.end());

  bool res = generate_path(navigation_waypoint_ids_, virtual_path, path);
  if (res) {
    printf("[%s]: Get virtual navigation path success\n", logger_);
  } else {
    printf("[%s]: Get virtual navigation path failed\n", logger_);
  }
  print_path(path, false);
  delete[] waypoint_mark_;
  delete[] dist_;
}

void VirtualPathManager::copy_waypoint(waypoint & wp1, waypoint & wp2)
{
  wp2.id = wp1.id;
  wp2.is_target = wp1.is_target;
  wp2.point.x = wp1.point.x;
  wp2.point.y = wp1.point.y;
  wp2.point.angle = wp1.point.angle;
  wp2.adjacent_waypoints.assign(wp1.adjacent_waypoints.begin(), wp1.adjacent_waypoints.end());
}

void VirtualPathManager::get_closest_point(point_2d & point,
    point_2d & wp1,
    uint32_t wp2_id,
    point_2d & closest_point)
{
  vector<point_2d> wps;
  vector<sub_path_info> path;
  point_2d wp2;
  get_waypoint(wp2_id, wp2);
  wps.push_back(wp1);
  wps.push_back(wp2);
  bool res = generate_path(wps, path);
  if (!res) {
    printf("[%s]: Generate path failed\n", logger_);
    return;
  }

  uint32_t len = path.size();
  if (len != 1) {
    printf("[%s]: Generate path failed, len=%d\n", logger_, len);
    return;
  }

  vector<point_2d> points = path[0].path;
  len = points.size();
  double shortest_dist = 0;
  point_2d tmp;
  for (uint32_t i = 0; i < len; i++) {
    double dist = compute_dist(point, points[i]);
    if (shortest_dist == 0) {
      shortest_dist = dist;
      tmp = points[i];
    } else if (shortest_dist > dist) {
      shortest_dist = dist;
      tmp = points[i];
    }
  }

  closest_point.x = tmp.x;
  closest_point.y = tmp.y;
  closest_point.angle = tmp.angle;
  printf("[%s]: The shortest point is (%f, %f), dist=%f\n", logger_, closest_point.x,
      closest_point.y, shortest_dist);
}

bool VirtualPathManager::is_adjacent(uint32_t i, uint32_t j, map<uint32_t, waypoint> & virtual_path)
{
  const uint32_t waypoint_count = virtual_path.size();
  printf("[%s]: i=%d,j=%d,size=%d\n", logger_, i, j, waypoint_count);
  if (!is_waypoint_id(i)) {
    printf("[%s]: no waypoint, i=%d,j=%d,size=%d\n", logger_, i, j, waypoint_count);
    return false;
  }
  vector<uint32_t> adjacent_waypoints = virtual_path.at(i).adjacent_waypoints;
  uint32_t len = adjacent_waypoints.size();
  for (uint32_t index = 0; index < len; index++) {
    uint32_t id = adjacent_waypoints[index];
    if (id == j) {
      return true;
    }
  }
  return false;
}

uint32_t VirtualPathManager::get_waypoint_id(point_2d & point,
    map<uint32_t, waypoint> & virtual_path)
{
  for (auto & info : virtual_path) {
    point_2d waypoint = info.second.point;
    if (is_same_position(point, waypoint)) {
      return info.second.id;
    }
  }
  return 0;
}

void VirtualPathManager::deep_first_search(uint32_t current_id,
    double dist,
    vector<uint32_t> & path)
{
  print_virtual_navigation_path(path, false);
  // current distance is larger than the previous shortest path
  // so it is not the short
  if (shortest_dist_ < dist) {
    return;
  }

  // arrvie the end id
  if (current_id == end_waypoint_id_) {
    // Although reaching the end point, ignore it unless there are
    // fewer paths than before
    if (shortest_dist_ > dist) {
      shortest_dist_ = dist;
      navigation_waypoint_ids_.assign(path.begin(), path.end());
      print_virtual_navigation_path(navigation_waypoint_ids_, false);
      return;
    }
  }
  for (uint32_t i = 1; i <= waypoint_count_; i++) {
    if ((waypoint_mark_[i] == 0) &&
        (*(dist_ + current_id * (waypoint_count_ + 1) + i) != VIRTUAL_PATH_DISTANCE_MAX) &&
        (*(dist_ + current_id * (waypoint_count_ + 1) + i) != 0)) {
      waypoint_mark_[i] = 1;
      vector<uint32_t> tmp;
      tmp.assign(path.begin(), path.end());
      tmp.push_back(i);
      double current_dist = dist + *(dist_ + current_id * (waypoint_count_ + 1) + i);
      deep_first_search(i, current_dist, tmp);
      waypoint_mark_[i] = 0;
    }
  }
}

void VirtualPathManager::print_virtual_path()
{
  for (auto & info : virtual_path_) {
    point_2d waypoint = info.second.point;
    uint32_t id = info.second.id;
    bool target = info.second.is_target;
    vector<uint32_t> adjacent_waypoints = info.second.adjacent_waypoints;
    uint32_t len = adjacent_waypoints.size();
    string str = "";
    for (uint32_t i = 0; i < len; i++) {
      uint32_t id = adjacent_waypoints[i];
      str.append(to_string(id));
      str.append(",");
    }
    printf(
        "[%s]: Virtual "
        "path,id=%d,target=%d,waypoint(%f,%f,%f),len=%d,adjacent_id(%s)\n",
        logger_, id, target, waypoint.x, waypoint.y, waypoint.angle, len, str.c_str());
  }
}

void VirtualPathManager::print_array()
{
  for (uint32_t i = 1; i <= waypoint_count_; i++) {
    for (uint32_t j = 1; j <= waypoint_count_; j++) {
      double val = *(dist_ + i * (waypoint_count_ + 1) + j);
      printf("[%s]: %d-->%d, dist=%lf\n", logger_, i, j, val);
    }
  }
}

bool VirtualPathManager::is_waypoint_id(uint32_t id)
{
  for (auto & info : virtual_path_) {
    if (info.second.id == id) {
      return true;
    }
  }
  return false;
}

bool VirtualPathManager::is_nearby_obstacle(point_2d & point)
{
  grid_map map;
  get_grid_map(map);

  double resolution = map.resolution;
  uint32_t width = map.width;
  uint32_t height = map.height;
  double origin_x = map.origin_x;
  double origin_y = map.origin_y;

  uint32_t map_size = map.data.size();
  printf("[%s]: get_cost_map map size: %d, w=%d, h=%d\n", logger_, map_size, width, height);

  point_2d p;
  p.x = point.x;
  p.y = point.y;
  p.angle = point.angle;
  convert_2d_point_from_world_map_to_cost_map(p, origin_x, origin_y);

  uint32_t cell_x, cell_y;
  bool res = get_cell_index(p.x, p.y, width, height, resolution, cell_x, cell_y);

  if (!res) {
    printf("[%s]: point is invalid\n", logger_);
    return true;
  }

  double shortest_distance = map_size;
  uint32_t obstacle_x, obstacle_y;
  for (uint32_t j = 0; j < height; j++) {
    for (uint32_t i = 0; i < width; i++) {
      uint32_t index = j * width + i;
      if (index > map_size) {
        continue;
      }

      int8_t data = map.data[index];
      if (is_obstacle(data)) {
        double tmp = compute_cell_distance(i, j, cell_x, cell_y, resolution);
        if (tmp < shortest_distance) {
          shortest_distance = tmp;
          obstacle_x = i;
          obstacle_y = j;
        }
      }
    }
  }

  if (shortest_distance <= DISTANCE_WAYPOINT_TO_OBSTACLE) {
    printf("[%s]: waypoint(%d, %d) is near obstacle(%d, %d), shortest_distance=%.2f\n", logger_,
        cell_x, cell_y, obstacle_x, obstacle_y, shortest_distance);
    return true;
  }
  printf("[%s]: waypoint(%d, %d) is not near obstacle, shortest_distance=%.2f\n", logger_, cell_x,
      cell_y, shortest_distance);
  return false;
}

void VirtualPathManager::convert_2d_point_from_world_map_to_cost_map(point_2d & point,
    double origin_x,
    double origin_y)
{
  double x = point.x;
  double y = point.y;

  // transfor point from world map to cost map
  point.x = x - origin_x;
  point.y = y - origin_y;
}

bool VirtualPathManager::get_cell_index(double px,
    double py,
    uint32_t width,
    uint32_t height,
    double resolution,
    uint32_t & cell_x,
    uint32_t & cell_y)
{
  if ((px < 0) || (py < 0)) {
    return true;
  }

  cell_x = (uint32_t)(px / resolution);
  cell_y = (uint32_t)(py / resolution);

  if ((cell_x < width) && (cell_y < height)) {
    printf("[%s]: get_cell_index(%d, %d)\n", logger_, cell_x, cell_y);
    return true;
  }
  return false;
}

bool VirtualPathManager::is_obstacle(const int8_t value)
{
  if (value >= LOCAL_MAP_OBSTACLE) {
    return true;
  } else {
    return false;
  }
}

double VirtualPathManager::compute_cell_distance(uint32_t cell_x1,
    uint32_t cell_y1,
    uint32_t cell_x2,
    uint32_t cell_y2,
    double resolution)
{
  double x1 = cell_x1 * resolution;
  double y1 = cell_y1 * resolution;
  double x2 = cell_x2 * resolution;
  double y2 = cell_y2 * resolution;
  return hypot(x1 - x2, y1 - y2);
}

bool VirtualPathManager::is_path_nearby_obstacle(uint32_t start_id, uint32_t end_id)
{
  point_2d start;
  bool result = get_waypoint(start_id, start);
  if (!result) {
    printf(
        "[%s]: Get virtual path failed from the waypoints because "
        "start is not waypoint\n",
        logger_);
    return true;
  }
  point_2d end;
  result = get_waypoint(end_id, end);
  if (!result) {
    printf(
        "[%s]: Get virtual path failed from the waypoints because "
        "end is not waypoint\n",
        logger_);
    return true;
  }

  vector<point_2d> path;
  compute_follow_path(start, end, path);
  printf("[%s]: Get path(size=%ld) from the waypoints\n", logger_, path.size());

  for (uint32_t i = 0; i < path.size(); i++) {
    point_2d p = path[i];
    if (is_nearby_obstacle(p)) {
      printf("[%s]: virtual path(point=%.2f, %.2f) is nearby obstacle\n", logger_, p.x, p.y);
      return true;
    }
  }

  printf("[%s]: virtual path is not nearby obstacle\n", logger_);
  return false;
}

void VirtualPathManager::compute_follow_path(point_2d & start_point,
    point_2d & end_point,
    vector<point_2d> & path)
{
  compute_path(start_point, end_point, path, VIRTUAL_PATH_MAX_INTERVAL_DIST);
}

bool VirtualPathManager::is_within_obstacle(point_2d & point)
{
  grid_map map;
  get_grid_map(map);

  double resolution = map.resolution;
  uint32_t width = map.width;
  uint32_t height = map.height;
  double origin_x = map.origin_x;
  double origin_y = map.origin_y;

  uint32_t map_size = map.data.size();

  point_2d p;
  p.x = point.x;
  p.y = point.y;
  p.angle = point.angle;
  convert_2d_point_from_world_map_to_cost_map(p, origin_x, origin_y);

  uint32_t cell_x, cell_y;
  bool res = get_cell_index(p.x, p.y, width, height, resolution, cell_x, cell_y);

  if (!res) {
    printf("[%s]: point is invalid\n", logger_);
    return true;
  }

  uint32_t index = cell_y * width + cell_x;
  if (index > map_size) {
    printf("[%s]: point is out of map, index=%d\n", logger_, index);
    return true;
  }

  int8_t data = map.data[index];
  if (is_obstacle(data)) {
    printf(
        "[%s]: waypoint(%d, %d) is within obstacle(%d,%d)\n", logger_, cell_x, cell_y, index, data);
    return true;
  }
  printf("[%s]: waypoint(%d, %d) is not within obstacle(%d,%d)\n", logger_, cell_x, cell_y, index,
      data);
  return false;
}

bool VirtualPathManager::map_exist()
{
  grid_map map;
  get_grid_map(map);

  uint32_t width = map.width;
  uint32_t height = map.height;
  if ((width == 0) || (height == 0)) {
    return false;
  }
  return true;
}

bool VirtualPathManager::is_unknown_cell(point_2d & point)
{
  grid_map map;
  get_grid_map(map);

  double resolution = map.resolution;
  uint32_t width = map.width;
  uint32_t height = map.height;
  double origin_x = map.origin_x;
  double origin_y = map.origin_y;

  printf("[%s]: p(%f,%f)\n", logger_, point.x, point.y);
  printf("[%s]: map(%f,%d,%d,%f,%f)\n", logger_, resolution, width, height, origin_x, origin_y);

  uint32_t map_size = map.data.size();

  point_2d p;
  p.x = point.x;
  p.y = point.y;
  p.angle = point.angle;
  convert_2d_point_from_world_map_to_cost_map(p, origin_x, origin_y);

  uint32_t cell_x, cell_y;
  bool res = get_cell_index(p.x, p.y, width, height, resolution, cell_x, cell_y);

  printf("[%s]: p(%f,%f),map(%f,%d,%d,%f,%f),cell(%d,%d)\n", logger_, p.x, p.y, resolution, width,
      height, origin_x, origin_y, cell_x, cell_y);

  if (!res) {
    printf("[%s]: point is invalid\n", logger_);
    return true;
  }

  uint32_t index = cell_y * width + cell_x;
  if (index > map_size) {
    printf("[%s]: point is out of map, index=%d\n", logger_, index);
    return true;
  }

  int8_t data = map.data[index];
  if (is_unknown(data)) {
    printf("[%s]: waypoint(%d, %d) is unknown(%d,%d)\n", logger_, cell_x, cell_y, index, data);
    return true;
  }
  printf("[%s]: waypoint(%d, %d) is not unknown(%d,%d)\n", logger_, cell_x, cell_y, index, data);
  return false;
}

bool VirtualPathManager::is_unknown(const int8_t value)
{
  if (value == LOCAL_MAP_UNKNOWN) {
    return true;
  } else {
    return false;
  }
}

bool VirtualPathManager::near_waypoint(point_2d & point)
{
  for (auto & info : virtual_path_) {
    point_2d p = info.second.point;
    if (within_range(point, p)) {
      printf("[%s]: Current position is nearby a waypoint(id=%d,point=%f,%f,%f)\n", logger_,
          info.second.id, info.second.point.x, info.second.point.y, info.second.point.angle);
      return true;
    }
  }
  return false;
}

uint32_t VirtualPathManager::get_waypoint_max_id(std::map<uint32_t, waypoint> & virtual_path)
{
  uint32_t max_id = 0;
  for (auto & info : virtual_path) {
    uint32_t id = info.second.id;
    if (id > max_id) {
      max_id = id;
    }
  }
  return max_id;
}

void VirtualPathManager::get_grid_map(grid_map & map)
{
  if (get_grid_map_cb_ != nullptr) {
    get_grid_map_cb_(map);
  } else {
    printf("[%s]: get_grid_map_cb_ is nullptr\n", logger_);
  }
}
}  // namespace navigation
}  // namespace qrb
