/*
 * Copyright (c) 2025 Qualcomm Innovation Center, Inc. All rights reserved.
 * SPDX-License-Identifier: BSD-3-Clause-Clear
 */

#ifndef QRB_NAVIGATION__STRUCT_DATA_HPP
#define QRB_NAVIGATION__STRUCT_DATA_HPP

#include <vector>
#include <iostream>
#include <string.h>
#include <stdio.h>
#include <math.h>

#define threshold_a 6  // Clustering multiple
#define distance_point(a1, a2, b1, b2) sqrt((a1 - b1) * (a1 - b1) + (a2 - b2) * (a2 - b2))

namespace qrb
{
namespace navigation
{

// point information
typedef struct _POINT
{
  double x;
  double y;
} POINT;

typedef struct _CSData
{
  std::vector<unsigned int> index;  // index
  std::vector<double> bearings;     // angle
  std::vector<double> cos_value;    // cosine
  std::vector<double> sin_value;    // sine
} CSdata;

typedef struct _RangeData
{
  std::vector<double> ranges;  // Role value
  std::vector<double> xs;      // x coordinate
  std::vector<double> ys;      // y coordinate
} Rangedata;

// Parameters, read in from launch file
typedef struct _Params
{
  double angle_increment;   // Angle Increment
  double angle_start;       // Initial Angle
  double least_thresh;      // Orthogonal fitting threshold
  double min_line_length;   // Fit the shortest distance of a line segment
  double predict_distance;  // The distance threshold between the real point and the predicted point
  unsigned int min_line_points;   // The number of laser points contained in a line segment
  unsigned int seed_line_points;  // The number of laser points contained in the seed line segment
} Params;

typedef struct _word_params
{
  double _role;
  double _theta_one;
  double _theta_two;
} word_params;

typedef struct _signal_params
{
  double distance_signal;
} signal_params;

// Linear segment information structure ax+by+c=0
typedef struct _line
{
  double a;  // line parameters
  double b;
  double c;
  int left;  // line scope
  int right;
  POINT p1;
  POINT p2;
  bool inte[2];
} line;

// Linear equation structure
typedef struct _least
{
  double a;
  double b;
  double c;
} least;

//}

typedef struct _point
{
  double role;
  double theta;
  double m_x;
  double m_y;
  double distance;
  double m_gradient;
  bool flag;
} PoinT;

typedef struct _generate_line
{
  // first point
  double x1;
  double y1;
  // end point
  double x2;
  double y2;
} gline;

typedef struct _signal
{
  double _angle1_radian;
  double _angle2_radian;
  double _angle1_degree;
  double _angle2_degree;
  double _role;
} Signal;

typedef struct _feature_point
{
  POINT _point;
  double _angle;
} featurepoint;

typedef struct _keyword
{
  int _index_role;
  int _index_theta_one;
  int _index_theta_two;
  std::vector<int> _frame_index;
} keyword;

}  // namespace navigation
}  // namespace qrb
#endif  // QRB_NAVIGATION__STRUCT_DATA_HPP
