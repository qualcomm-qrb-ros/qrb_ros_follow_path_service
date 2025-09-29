/*
 * Copyright (c) 2025 Qualcomm Innovation Center, Inc. All rights reserved.
 * SPDX-License-Identifier: BSD-3-Clause-Clear
 */

#include "obstacle_detector.hpp"

#include <cmath>
#include <memory>

namespace qrb
{
namespace navigation
{

ObstacleDetector::ObstacleDetector() {}

ObstacleDetector::~ObstacleDetector() {}

void ObstacleDetector::register_get_grid_map_callback(get_grid_map_func_t cb)
{
  printf("[%s]: register_get_grid_map_callback\n", logger_);
  get_grid_map_cb_ = cb;
}

bool ObstacleDetector::confirm_path_invalid_with_base_shape(std::vector<point_2d> & path)
{
  std::vector<point_2d> shape;
  grid_map map;

  get_grid_map(map);

  double resolution = map.resolution;
  uint32_t width = map.width;
  uint32_t height = map.height;
  double origin_x = map.origin_x;
  double origin_y = map.origin_y;
  uint32_t map_size = map.data.size();
  printf("[%s]: get_map:w=%d,h=%d,size: %d,origin(%.2f,%.2f)\n", logger_, width, height, map_size,
      origin_x, origin_y);

  for (uint32_t i = 0; i < path.size(); ++i) {
    point_2d current_point;
    current_point.x = path[i].x;
    current_point.y = path[i].y;
    current_point.angle = path[i].angle;
    convert_2d_point_from_world_map_to_cost_map(current_point, origin_x, origin_y);
    uint32_t p_x, p_y;
    get_cell_index(current_point.x, current_point.y, width, height, resolution, p_x, p_y);

    // Get the points of scope from the current point
    shape.clear();
    get_base_shape(current_point.x, current_point.y, current_point.angle, shape);
    bool risk = confirm_shape_invalid(shape, map);
    if (risk) {
      printf("[%s]: Shape is invalid when base point(%.2f,%.2f,%.2f) is (%d, %d)\n", logger_,
          current_point.x, current_point.y, current_point.angle, p_x, p_y);
      return true;
    }
  }
  return false;
}

void ObstacleDetector::get_grid_map(grid_map & map)
{
  if (get_grid_map_cb_ != nullptr) {
    get_grid_map_cb_(map);
  } else {
    printf("[%s]: get_grid_map_cb_ is nullptr\n", logger_);
  }
}

void ObstacleDetector::convert_2d_point_from_world_map_to_cost_map(point_2d & point,
    double origin_x,
    double origin_y)
{
  double x = point.x;
  double y = point.y;

  // transfor point from world map to cost map
  point.x = x - origin_x;
  point.y = y - origin_y;
}

void ObstacleDetector::get_base_shape(double px,
    double py,
    double pth,
    std::vector<point_2d> & shape)
{
  if (BASE_SHAPE_CIRCLE) {
    get_circle_shape(px, py, shape);
  } else {
    get_rect_shape(px, py, pth, shape);
  }
}

void ObstacleDetector::get_circle_shape(double px, double py, std::vector<point_2d> & shape)
{
  double x, y;
  double sample_angle;
  double interval = M_PI * 2 / BREAK_BASE_SHAPE_SAMPLE_COUNT;
  for (uint32_t i = 0; i < BREAK_BASE_SHAPE_SAMPLE_COUNT; i++) {
    sample_angle = interval * i;
    x = px + BASE_SHAPE_CIRCLE_R * sin(sample_angle);
    y = py + BASE_SHAPE_CIRCLE_R * cos(sample_angle);
    point_2d p;
    p.x = x;
    p.y = y;
    shape.push_back(p);
  }
}

void ObstacleDetector::get_rect_shape(double px,
    double py,
    double pth,
    std::vector<point_2d> & shape)
{
  double x1 = BASE_SHAPE_LENGTH / 2;
  double y1 = BASE_SHAPE_WIDTH / 2;
  double x2 = -BASE_SHAPE_LENGTH / 2;
  double y2 = BASE_SHAPE_WIDTH / 2;
  double x3 = -BASE_SHAPE_LENGTH / 2;
  double y3 = -BASE_SHAPE_WIDTH / 2;
  double x4 = BASE_SHAPE_LENGTH / 2;
  double y4 = -BASE_SHAPE_WIDTH / 2;

  point_2d p1;
  convert_coordinate_from_base_2_map(x1, y1, px, py, pth, p1);
  point_2d p2;
  convert_coordinate_from_base_2_map(x2, y2, px, py, pth, p2);
  sample_base_shape_point(p1, p2, shape);
  point_2d p3;
  convert_coordinate_from_base_2_map(x3, y3, px, py, pth, p3);
  sample_base_shape_point(p2, p3, shape);
  point_2d p4;
  convert_coordinate_from_base_2_map(x4, y4, px, py, pth, p4);
  sample_base_shape_point(p3, p4, shape);
  point_2d p5;
  convert_coordinate_from_base_2_map(x1, y1, px, py, pth, p5);
  sample_base_shape_point(p4, p5, shape);
}

void ObstacleDetector::convert_coordinate_from_base_2_map(double x,
    double y,
    double px,
    double py,
    double pth,
    point_2d & p)
{
  p.x = px + x * cos(pth) - y * sin(pth);
  p.y = py + x * sin(pth) + y * cos(pth);
}

void ObstacleDetector::sample_base_shape_point(point_2d & start,
    point_2d & end,
    std::vector<point_2d> & shape)
{
  sample_point_count_ = 2;
  // compute the path of two points
  std::vector<point_2d> tmp;
  tmp.push_back(start);
  tmp.push_back(end);

  if (sample_point_count_ < (BREAK_BASE_SHAPE_SAMPLE_COUNT / 4)) {
    compute_and_insert_point(tmp);
  }

  // add points to follow path
  uint32_t len = tmp.size();
  for (uint32_t i = 0; i < len; i++) {
    point_2d p;
    p.x = tmp[i].x;
    p.y = tmp[i].y;
    shape.push_back(p);
  }
}

void ObstacleDetector::compute_and_insert_point(std::vector<point_2d> & points)
{
  std::vector<point_2d> tmp;
  tmp.assign(points.begin(), points.end());
  sample_point_count_++;
  uint32_t len = tmp.size();
  for (uint32_t i = 0; i < (len - 1); i++) {
    point_2d start = tmp[i];
    point_2d end = tmp[i + 1];
    point_2d middle_point;
    compute_middle_point(start, end, middle_point);
    uint32_t index = get_insert_index(points, end);
    if (index != 0) {
      insert_point(middle_point, points, index);
    } else {
      printf("[%s]: Index is invalid, ignore.............\n", logger_);
      continue;
    }
  }

  if (sample_point_count_ < (BREAK_BASE_SHAPE_SAMPLE_COUNT / 4)) {
    compute_and_insert_point(points);
  } else {
  }
}

void ObstacleDetector::compute_middle_point(point_2d & start, point_2d & end, point_2d & point)
{
  point.x = (start.x + end.x) / 2;
  point.y = (start.y + end.y) / 2;
}

// insert point before the index
void ObstacleDetector::insert_point(point_2d & point,
    std::vector<point_2d> & points,
    uint32_t index)
{
  points.insert(points.begin() + index, point);
}

uint32_t ObstacleDetector::get_insert_index(std::vector<point_2d> & points, point_2d & point)
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

bool ObstacleDetector::is_same_position(point_2d & p1, point_2d & p2)
{
  if ((p1.x == p2.x) && (p1.y == p2.y)) {
    return true;
  }
  return false;
}

bool ObstacleDetector::confirm_shape_invalid(std::vector<point_2d> & shape, grid_map & map)
{
  double resolution = map.resolution;
  uint32_t width = map.width;
  uint32_t height = map.height;
  uint32_t map_size = map.data.size();

  for (uint32_t i = 0; i < shape.size(); i++) {
    point_2d p = shape[i];
    uint32_t cell_x, cell_y;
    bool res = get_cell_index(p.x, p.y, width, height, resolution, cell_x, cell_y);

    if (!res) {
      printf("[%s]: point(%.2f,%.2f)(%d,%d) is invalid\n", logger_, p.x, p.y, cell_x, cell_y);
      return true;
    }

    uint32_t index = cell_y * width + cell_x;
    if (index >= map_size) {
      printf("[%s]: point(%.2f,%.2f)(%d,%d) is out of map, index=%d\n", logger_, p.x, p.y, cell_x,
          cell_y, index);
      return true;
    }

    int8_t data = map.data[index];
    if (is_obstacle(data)) {
      printf("[%s]: point(%.2f,%.2f)(%d, %d) is within obstacle(%d)\n", logger_, p.x, p.y, cell_x,
          cell_y, index);
      return true;
    }
  }

  return false;
}

bool ObstacleDetector::get_cell_index(double px,
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
    return true;
  }
  return false;
}

bool ObstacleDetector::is_obstacle(const int8_t value)
{
  if (value >= LOCAL_MAP_OBSTACLE) {
    return true;
  } else {
    return false;
  }
}
}  // namespace navigation
}  // namespace qrb
