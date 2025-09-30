/*
 * Copyright (c) 2025 Qualcomm Innovation Center, Inc. All rights reserved.
 * SPDX-License-Identifier: BSD-3-Clause-Clear
 */

#ifndef QRB_NAVIGATION__OBSTACLE_DETECTOR_
#define QRB_NAVIGATION__OBSTACLE_DETECTOR_

#include "common/common.hpp"

namespace qrb
{
namespace navigation
{

class ObstacleDetector
{
private:
  void get_base_shape(double px, double py, double pth, std::vector<point_2d> & shape);
  void get_circle_shape(double px, double py, std::vector<point_2d> & shape);
  void get_rect_shape(double px, double py, double pth, std::vector<point_2d> & shape);
  void convert_coordinate_from_base_2_map(double x,
      double y,
      double px,
      double py,
      double pth,
      point_2d & p);
  void sample_base_shape_point(point_2d & start, point_2d & end, std::vector<point_2d> & shape);
  void compute_and_insert_point(std::vector<point_2d> & points);
  void compute_middle_point(point_2d & start, point_2d & end, point_2d & point);
  void insert_point(point_2d & point, std::vector<point_2d> & points, uint32_t index);
  uint32_t get_insert_index(std::vector<point_2d> & points, point_2d & point);
  bool is_same_position(point_2d & p1, point_2d & p2);
  bool confirm_shape_invalid(std::vector<point_2d> & shape, grid_map & map);
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
  bool is_obstacle(const int8_t value);
  void get_grid_map(grid_map & map);

  get_grid_map_func_t get_grid_map_cb_;
  uint32_t sample_point_count_;
  const char * logger_ = "obstacle_detector";

public:
  void register_get_grid_map_callback(get_grid_map_func_t cb);
  bool confirm_path_invalid_with_base_shape(std::vector<point_2d> & path);

  ObstacleDetector();

  ~ObstacleDetector();
};
}  // namespace navigation
}  // namespace qrb
#endif  // QRB_NAVIGATION__OBSTACLE_DETECTOR_
