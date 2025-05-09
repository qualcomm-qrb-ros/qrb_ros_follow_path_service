/*
 * Copyright (c) 2025 Qualcomm Innovation Center, Inc. All rights reserved.
 * SPDX-License-Identifier: BSD-3-Clause-Clear
 */

#include "virtual_path/virtual_path_helper.hpp"

#include <sys/stat.h>
#include <sys/types.h>
#include <unistd.h>

#include <chrono>
#include <cstdio>

using namespace std;

namespace qrb
{
namespace navigation
{

VirtualPathHelper::VirtualPathHelper(std::string save_file_path, std::string backup_file_path)
  : save_file_path_(save_file_path), backup_file_path_(backup_file_path)
{
}

VirtualPathHelper::~VirtualPathHelper() {}

bool VirtualPathHelper::save(std::map<uint32_t, waypoint> & virtual_path)
{
  if (!backup()) {
    return false;
  }
  if (!create_parent_dir(save_file_path_)) {
    return false;
  }
  tinyxml2::XMLDocument doc;
  createXMLDocument(doc, virtual_path);
  uint8_t res = doc.SaveFile(save_file_path_.c_str());
  if (res != 0) {
    printf("[%s]: save %s failed, res = %d\n", logger_, save_file_path_.c_str(), res);
    return false;
  }
  return true;
}

bool VirtualPathHelper::save(std::map<uint32_t, waypoint> & virtual_path, bool bypassing)
{
  if (!backup()) {
    return false;
  }
  if (!create_parent_dir(save_file_path_)) {
    return false;
  }
  tinyxml2::XMLDocument doc;
  createXMLDocument(doc, virtual_path, bypassing);
  uint8_t res = doc.SaveFile(save_file_path_.c_str());
  if (res != 0) {
    printf("[%s]: save %s failed, res = %d\n", logger_, save_file_path_.c_str(), res);
    return false;
  }
  return true;
}

bool VirtualPathHelper::load(std::map<uint32_t, waypoint> & virtual_path, bool & bypassing)
{
  if (!path_exits(save_file_path_)) {
    return true;
  }
  tinyxml2::XMLDocument doc;
  if (doc.LoadFile(save_file_path_.c_str()) != 0) {
    printf("[%s]: load xml from %s failed\n", logger_, save_file_path_.c_str());
    return false;
  }
  return parseXMLDocument(doc, virtual_path, bypassing);
}

bool VirtualPathHelper::backup()
{
  if (!path_exits(save_file_path_)) {
    return true;
  }
  if (path_exits(backup_file_path_) && std::remove(backup_file_path_.c_str()) != 0) {
    printf("[%s]: remove %s failed\n", logger_, backup_file_path_.c_str());
    return false;
  }
  if (!create_parent_dir(backup_file_path_) ||
      std::rename(save_file_path_.c_str(), backup_file_path_.c_str()) != 0) {
    printf("[%s]: rename %s to %s failed\n", logger_, save_file_path_.c_str(),
        backup_file_path_.c_str());
    return false;
  }
  return true;
}

bool VirtualPathHelper::path_exits(std::string path)
{
  return access(path.c_str(), F_OK) == 0;
}

bool VirtualPathHelper::create_parent_dir(std::string file_path)
{
  if (file_path.empty()) {
    return true;
  }
  std::size_t start = file_path[0] == '/' ? 1 : 0;
  std::size_t end = std::string::npos;
  while ((end = file_path.find_first_of("/", start)) != std::string::npos) {
    start = end + 1;
    auto dir_path = file_path.substr(0, end);
    if (path_exits(dir_path)) {
      continue;
    }
    if (mkdir(dir_path.c_str(), 0755) != 0) {
      printf("[%s]: create directory failed: %s\n", logger_, dir_path.c_str());
      return false;
    }
  }
  return true;
}

void VirtualPathHelper::createXMLDocument(tinyxml2::XMLDocument & doc,
    std::map<uint32_t, waypoint> & virtual_path)
{
  doc.InsertFirstChild(doc.NewDeclaration());

  tinyxml2::XMLElement * root = doc.NewElement("path");
  doc.InsertEndChild(root);

  auto virtual_path_e = doc.NewElement("virtual_path");
  auto now = std::chrono::system_clock::now();
  virtual_path_e->SetAttribute("update_timestamp", std::chrono::system_clock::to_time_t(now));
  root->InsertEndChild(virtual_path_e);

  auto make_double_element = [&](const char * name, double value) {
    auto e = doc.NewElement(name);
    e->SetText(value);
    return e;
  };

  auto make_bool_element = [&](const char * name, bool value) {
    auto e = doc.NewElement(name);
    e->SetText(value);
    return e;
  };

  auto make_int_element = [&](const char * name, uint32_t value) {
    auto e = doc.NewElement(name);
    e->SetText(value);
    return e;
  };

  for (const auto & info : virtual_path) {
    auto virtual_path_info_e = doc.NewElement("waypoint");
    virtual_path_info_e->SetAttribute("id", info.second.id);
    virtual_path_info_e->InsertEndChild(make_double_element("x", info.second.point.x));
    virtual_path_info_e->InsertEndChild(make_double_element("y", info.second.point.y));
    virtual_path_info_e->InsertEndChild(make_double_element("angle", info.second.point.angle));
    virtual_path_info_e->InsertEndChild(make_bool_element("is_target", info.second.is_target));
    printf("[%s]: save p(%f,%f,%f)\n", logger_, info.second.point.x, info.second.point.y,
        info.second.point.angle);

    vector<uint32_t> adjacent_waypoints = info.second.adjacent_waypoints;
    uint32_t len = adjacent_waypoints.size();
    for (uint32_t i = 0; i < len; i++) {
      auto adjacent_waypoint_e = doc.NewElement("adjacent_waypoint");
      virtual_path_info_e->InsertEndChild(adjacent_waypoint_e);
      adjacent_waypoint_e->InsertEndChild(make_int_element("id", adjacent_waypoints[i]));
    }
    root->InsertEndChild(virtual_path_info_e);
  }
}

void VirtualPathHelper::createXMLDocument(tinyxml2::XMLDocument & doc,
    std::map<uint32_t, waypoint> & virtual_path,
    bool bypassing)
{
  doc.InsertFirstChild(doc.NewDeclaration());

  tinyxml2::XMLElement * root = doc.NewElement("path");
  doc.InsertEndChild(root);

  auto virtual_path_e = doc.NewElement("virtual_path");
  auto now = std::chrono::system_clock::now();
  virtual_path_e->SetAttribute("update_timestamp", std::chrono::system_clock::to_time_t(now));
  virtual_path_e->SetAttribute("bypassing", bypassing ? 1 : 0);
  root->InsertEndChild(virtual_path_e);

  auto make_double_element = [&](const char * name, double value) {
    auto e = doc.NewElement(name);
    e->SetText(value);
    return e;
  };

  auto make_bool_element = [&](const char * name, bool value) {
    auto e = doc.NewElement(name);
    e->SetText(value);
    return e;
  };

  auto make_int_element = [&](const char * name, uint32_t value) {
    auto e = doc.NewElement(name);
    e->SetText(value);
    return e;
  };

  for (const auto & info : virtual_path) {
    auto virtual_path_info_e = doc.NewElement("waypoint");
    virtual_path_info_e->SetAttribute("id", info.second.id);
    virtual_path_info_e->InsertEndChild(make_double_element("x", info.second.point.x));
    virtual_path_info_e->InsertEndChild(make_double_element("y", info.second.point.y));
    virtual_path_info_e->InsertEndChild(make_double_element("angle", info.second.point.angle));
    virtual_path_info_e->InsertEndChild(make_bool_element("is_target", info.second.is_target));
    printf("[%s]: save p(%f,%f,%f)\n", logger_, info.second.point.x, info.second.point.y,
        info.second.point.angle);

    vector<uint32_t> adjacent_waypoints = info.second.adjacent_waypoints;
    uint32_t len = adjacent_waypoints.size();
    printf("[%s]: len: %d\n", logger_, len);
    for (uint32_t i = 0; i < len; i++) {
      auto adjacent_waypoint_e = doc.NewElement("adjacent_waypoint");
      virtual_path_info_e->InsertEndChild(adjacent_waypoint_e);
      adjacent_waypoint_e->InsertEndChild(make_int_element("id", adjacent_waypoints[i]));
      printf("[%s]: adjacent_waypoint_e: %d\n", logger_, adjacent_waypoints[i]);
    }
    root->InsertEndChild(virtual_path_info_e);
  }
}

bool VirtualPathHelper::parseXMLDocument(tinyxml2::XMLDocument & doc,
    std::map<uint32_t, waypoint> & virtual_path,
    bool & bypassing)
{
  tinyxml2::XMLElement * root_element = doc.RootElement();
  auto virtual_path_e = root_element->FirstChildElement("virtual_path");
  if (virtual_path_e == nullptr) {
    printf("[%s]: can not found <virtual_path> in %s\n", logger_, save_file_path_.c_str());
    return false;
  } else {
    bypassing = root_element->IntAttribute("bypassing");
    printf("[%s]: bypassing: %d\n", logger_, bypassing);
  }

  tinyxml2::XMLElement * e_info = nullptr;

  auto read_double_element = [&](tinyxml2::XMLElement * root, int root_id, const char * name,
                                 double * value, bool can_empty = false) -> bool {
    tinyxml2::XMLElement * e = root->FirstChildElement(name);
    if (e->GetText() == nullptr) {
      if (can_empty) {
        return true;
      }
      printf("[%s]: read %s is null for narrow id: %d\n", logger_, name, root_id);
      return false;
    }
    if (e->QueryDoubleText(value) != tinyxml2::XMLError::XML_SUCCESS) {
      printf("[%s]: read double value: %s failed for narrow id: %d\n", logger_, name, root_id);
      return false;
    }
    return true;
  };

  auto read_bool_element = [&](tinyxml2::XMLElement * root, int root_id, const char * name,
                               bool * value, bool can_empty = false) -> bool {
    tinyxml2::XMLElement * e = root->FirstChildElement(name);
    if (e->GetText() == nullptr) {
      if (can_empty) {
        return true;
      }
      printf("[%s]: read %s is null for narrow id: %d\n", logger_, name, root_id);
      return false;
    }
    if (e->QueryBoolText(value) != tinyxml2::XMLError::XML_SUCCESS) {
      printf("[%s]: read bool value: %s failed for narrow id: %d\n", logger_, name, root_id);
      return false;
    }
    return true;
  };

  auto read_int_element = [&](tinyxml2::XMLElement * root, const char * name, int * value,
                              bool can_empty = false) -> bool {
    tinyxml2::XMLElement * e = root->FirstChildElement(name);
    if (e->GetText() == nullptr) {
      if (can_empty) {
        return true;
      }
      printf("[%s]: read %s is null\n", logger_, name);
      return false;
    }
    if (e->QueryIntText(value) != tinyxml2::XMLError::XML_SUCCESS) {
      printf("[%s]: read int value: %s failed\n", logger_, name);
      return false;
    }
    return true;
  };

  for (e_info = root_element->FirstChildElement("waypoint"); e_info;
       e_info = e_info->NextSiblingElement("waypoint")) {
    auto id = e_info->IntAttribute("id");
    waypoint info;
    info.id = id;

    if (!read_double_element(e_info, id, "x", &info.point.x) ||
        !read_double_element(e_info, id, "y", &info.point.y) ||
        !read_double_element(e_info, id, "angle", &info.point.angle) ||
        !read_bool_element(e_info, id, "is_target", &info.is_target)) {
      return false;
    }

    int adjacent_waypoint_id;
    tinyxml2::XMLElement * e_sub_info = nullptr;
    for (e_sub_info = e_info->FirstChildElement("adjacent_waypoint"); e_sub_info;
         e_sub_info = e_sub_info->NextSiblingElement("adjacent_waypoint")) {
      if (!read_int_element(e_sub_info, "id", &adjacent_waypoint_id)) {
        return false;
      }
      info.adjacent_waypoints.push_back((uint32_t)adjacent_waypoint_id);
    }

    virtual_path.emplace(id, info);
    string str = get_adjacent_waypoint(info.adjacent_waypoints);
    printf(
        "[%s]: Load waypoint:id=%d, point(%.2f,%.2f,%.2f), is_target=%d, "
        "adjacent_waypoints= %s\n",
        logger_, id, info.point.x, info.point.y, info.point.angle, info.is_target, str.c_str());
  }
  return true;
}

string VirtualPathHelper::get_adjacent_waypoint(vector<uint32_t> points)
{
  string str = "";
  uint32_t len = points.size();
  for (uint32_t i = 0; i < len; i++) {
    uint32_t id = points[i];
    str.append(std::to_string(id));
    if (i != (len - 1)) {
      str.append(",");
    }
  }
  return str;
}
}  // namespace navigation
}  // namespace qrb
