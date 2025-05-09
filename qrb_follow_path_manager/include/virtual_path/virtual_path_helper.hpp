/*
 * Copyright (c) 2025 Qualcomm Innovation Center, Inc. All rights reserved.
 * SPDX-License-Identifier: BSD-3-Clause-Clear
 */

#ifndef QRB_NAVIGATION__VIRTUAL_PATH_HELPER_HPP_
#define QRB_NAVIGATION__VIRTUAL_PATH_HELPER_HPP_

#include "common/common.hpp"

namespace qrb
{
namespace navigation
{

struct waypoint
{
  uint32_t id;
  bool is_target;
  point_2d point;
  std::vector<uint32_t> adjacent_waypoints;
};

struct sub_path_info
{
  uint32_t index;
  std::vector<point_2d> path;
};

struct follow_path_info
{
  point_2d start_point;
  point_2d end_point;
  std::vector<sub_path_info> path;
};

class VirtualPathHelper
{
public:
  VirtualPathHelper(std::string save_file_path, std::string backup_file_path);
  ~VirtualPathHelper();

  bool save(std::map<uint32_t, waypoint> & virtual_path);
  bool save(std::map<uint32_t, waypoint> & virtual_path, bool bypassing);
  bool load(std::map<uint32_t, waypoint> & virtual_path, bool & bypassing);

private:
  bool backup();

  inline bool path_exits(std::string path);
  bool create_parent_dir(std::string file_path);

  void createXMLDocument(tinyxml2::XMLDocument & doc, std::map<uint32_t, waypoint> & virtual_path);
  void createXMLDocument(tinyxml2::XMLDocument & doc,
      std::map<uint32_t, waypoint> & virtual_path,
      bool bypassing);
  bool parseXMLDocument(tinyxml2::XMLDocument & doc,
      std::map<uint32_t, waypoint> & virtual_path,
      bool & bypassing);
  std::string get_adjacent_waypoint(std::vector<uint32_t> points);

  std::string save_file_path_;
  std::string backup_file_path_;
  const char * logger_ = "virtual_path_helper";
};
}  // namespace navigation
}  // namespace qrb
#endif  // QRB_NAVIGATION__VIRTUAL_PATH_HELPER_HPP_