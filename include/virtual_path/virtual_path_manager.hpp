/*
 * Copyright (c) 2025 Qualcomm Innovation Center, Inc. All rights reserved.
 * SPDX-License-Identifier: BSD-3-Clause-Clear
 */

#ifndef QRB_NAVIGATION__VIRTUAL_PATH_MANAGER_HPP_
#define QRB_NAVIGATION__VIRTUAL_PATH_MANAGER_HPP_

#include "virtual_path_helper.hpp"
#include "common/common.hpp"

#define VIRTUAL_PATH_FILE_PATH "navigation/virtual_path.xml"
#define VIRTUAL_PATH_BAKEUP_FILE_PATH "navigation/virtual_path_backup.xml"

#define VIRTUAL_PATH_ID_START_INDEX 1
#define VIRTUAL_PATH_ID_MAX_INDEX 255

#define VIRTUAL_PATH_MIN_INTERVAL_DIST 0.05  // uint:meter
#define VIRTUAL_PATH_MAX_INTERVAL_DIST 0.1
#define NEARBY_WAYPOINT_MAX_DIST 0.2

#define VIRTUAL_PATH_DISTANCE_MAX UINT32_MAX

// if the distance between two points is less than this threshold
// the two points can be considered as the same point
#define POSITION_MAX_RANGE 0.2

#define DISTANCE_WAYPOINT_TO_OBSTACLE 0.4

namespace qrb
{
namespace navigation
{

class VirtualPathManager
{
public:
  VirtualPathManager();
  ~VirtualPathManager();

  void register_get_grid_map_callback(get_grid_map_func_t cb);

  bool is_valid(point_2d & point);

  uint32_t add_waypoint(point_2d & point);

  bool remove_waypoint(uint32_t id);

  bool get_waypoint_id_list(std::vector<uint32_t> & list);

  bool get_waypoint(uint32_t id, point_2d & point);

  uint32_t get_waypoint_id(point_2d & point);

  uint32_t get_nearby_waypoint(point_2d & current_point, point_2d & waypoint);

  bool add_virtual_path(uint32_t id, std::vector<uint32_t> & ids);

  bool force_add_virtual_path(uint32_t id, std::vector<uint32_t> & ids);

  bool remove_virtual_path(uint32_t id, std::vector<uint32_t> & ids);

  bool remove_waypoint_and_virtual_path();

  bool get_virtual_path(uint32_t id, std::vector<uint32_t> & ids);

  void set_bypassing_obstacle(bool val);

  bool get_bypassing_obstacle();

  void find_follow_path(point_2d & start_point,
      point_2d & end_point,
      std::vector<sub_path_info> & path);

  void compute_follow_path(point_2d & start_point,
      point_2d & end_point,
      std::vector<sub_path_info> & path,
      std::vector<uint32_t> & list);
  void compute_follow_path(point_2d & start_point,
      point_2d & end_point,
      std::vector<point_2d> & path);

  void
  compute_path(point_2d & start, point_2d & end, std::vector<point_2d> & path, double max_dist);

private:
  void save_virtual_path();

  void save_bypassing_obstacle(bool val);

  bool is_waypoint(point_2d & point);

  bool is_same_position(point_2d & p1, point_2d & p2);

  bool within_range(point_2d & p1, point_2d & p2);

  bool generate_path(std::vector<uint32_t> & ids, std::vector<sub_path_info> & path);

  bool generate_path(std::vector<uint32_t> & ids,
      std::map<uint32_t, waypoint> & virtual_path,
      std::vector<sub_path_info> & path);

  bool generate_path(std::vector<point_2d> & waypoints, std::vector<sub_path_info> & path);

  bool get_waypoint(uint32_t id, std::map<uint32_t, waypoint> & virtual_path, point_2d & point);

  void compute_and_insert_point(std::vector<point_2d> & points, double max_dist);

  void compute_middle_point_and_angle(point_2d & start, point_2d & end, point_2d & point);

  void compute_angle(point_2d & start, point_2d & end);

  void insert_point(point_2d & point, std::vector<point_2d> & points, uint32_t index);

  double compute_dist(point_2d & first_point, point_2d & second_point);

  double get_angle_of_two_points(point_2d & first_point, point_2d & second_point);

  uint32_t get_new_waypoint_id();

  uint32_t get_insert_index(std::vector<point_2d> & points, point_2d & point);

  void print_path(std::vector<sub_path_info> & path, bool print);

  void print_path(std::vector<point_2d> & path, bool print);

  void print_virtual_path();

  void print_virtual_navigation_path(std::vector<uint32_t> & wps, bool print);

  void print_array();

  void find_waypoint(point_2d & point, waypoint & wp);

  void add_waypoint_for_navigation(waypoint & start_waypoint,
      std::map<uint32_t, waypoint> & virtual_path);

  void find_virtual_navigation_path(waypoint & start_waypoint,
      waypoint & end_waypoint,
      std::map<uint32_t, waypoint> & virtual_path,
      std::vector<sub_path_info> & path);

  void get_virtual_navigation_path(waypoint & start_waypoint,
      waypoint & end_waypoint,
      std::map<uint32_t, waypoint> & virtual_path,
      std::vector<sub_path_info> & path,
      std::vector<uint32_t> & list);

  void copy_waypoint(waypoint & wp1, waypoint & wp2);

  void
  get_closest_point(point_2d & point, point_2d & wp1, uint32_t wp2_id, point_2d & closest_point);

  bool is_adjacent(uint32_t i, uint32_t j, std::map<uint32_t, waypoint> & virtual_path);

  uint32_t get_waypoint_id(point_2d & point, std::map<uint32_t, waypoint> & virtual_path);

  void deep_first_search(uint32_t current_id, double dist, std::vector<uint32_t> & path);

  bool is_waypoint_id(uint32_t id);

  void remove_adjacent_waypoint(uint32_t remove_id);

  bool is_nearby_obstacle(point_2d & point);

  void convert_2d_point_from_world_map_to_cost_map(point_2d & point,
      double origin_x,
      double origin_y);

  bool get_cell_index(double px,
      double py,
      uint32_t width,
      uint32_t height,
      double resolution,
      uint32_t & cell_x,
      uint32_t & cell_y);

  void add_two_way_virtual_path(uint32_t id, uint32_t adjacent_id);

  void remove_two_way_virtual_path(uint32_t id, uint32_t adjacent_id);

  bool is_obstacle(const int8_t value);

  double compute_cell_distance(uint32_t cell_x1,
      uint32_t cell_y1,
      uint32_t cell_x2,
      uint32_t cell_y2,
      double resolution);

  bool is_within_obstacle(point_2d & point);

  bool is_path_nearby_obstacle(uint32_t start_id, uint32_t end_id);

  bool map_exist();

  bool is_unknown_cell(point_2d & point);

  bool is_unknown(const int8_t value);

  bool near_waypoint(point_2d & point);

  uint32_t get_waypoint_max_id(std::map<uint32_t, waypoint> & virtual_path);

  void get_grid_map(grid_map & map);

  uint32_t compute_count_;
  std::map<uint32_t, waypoint> virtual_path_;
  std::shared_ptr<VirtualPathHelper> virtual_path_helper_{ nullptr };
  double shortest_dist_;
  double * dist_;
  char * waypoint_mark_;
  uint32_t end_waypoint_id_;
  uint32_t waypoint_count_;
  std::vector<uint32_t> navigation_waypoint_ids_;
  bool bypassing_;
  get_grid_map_func_t get_grid_map_cb_;
  const char * logger_ = "virtual_path_manager";
};
}  // namespace navigation
}  // namespace qrb
#endif  // QRB_NAVIGATION__VIRTUAL_PATH_MANAGER_HPP_