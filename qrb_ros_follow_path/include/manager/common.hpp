/*
 * Copyright (c) 2025 Qualcomm Innovation Center, Inc. All rights reserved.
 * SPDX-License-Identifier: BSD-3-Clause-Clear
 */

#ifndef QRB_NAVIGATION__COMMON_HPP
#define QRB_NAVIGATION__COMMON_HPP

#include <condition_variable>
#include <float.h>
#include <functional>
#include <iostream>
#include <memory>
#include <mutex>
#include <pthread.h>
#include <queue>
#include <unistd.h>
#include <utility>
#include <vector>
#include <string>
#include <sys/socket.h>
#include <sys/un.h>

#define LOCAL_MAP_UNKNOWN -1
#define LOCAL_MAP_FREE 0
#define LOCAL_MAP_OBSTACLE 100
#define BASE_SHAPE_CIRCLE false
#define BASE_SHAPE_CIRCLE_R 0.25    // the radius of circle is 0.25 meter
#define BASE_SHAPE_LENGTH 0.7       // meter
#define BASE_SHAPE_WIDTH 0.5        // meter
#define BASE_SHAPE_SAMPLE_COUNT 12  // just to show the base trajectory
#define EMERGENCY_BRAKING_THRESHOLD 0.2
#define BREAK_BASE_SHAPE_SAMPLE_COUNT 32

namespace qrb
{
namespace navigation
{

// 2d grid map
typedef struct
{
  double resolution;
  uint32_t width;
  uint32_t height;
  double origin_x;
  double origin_y;
  std::vector<uint8_t> data;
} grid_map;

// point in 2d map
typedef struct
{
  double x;      // x coordinate of point
  double y;      // y coordinate of point
  double angle;  // angle of point, unit: degree
} point_2d;

typedef struct
{
  double x;
  double y;
  double z;
} twist_vel;

// point with waypoint info
typedef struct
{
  uint32_t waypoint_id;  // id is zero if current point is not a waypoint
  point_2d point;        // point in 2d map
} waypoint_2d;

enum class APIID
{
  GetFollowPath = 1,
  AddWaypoint = 2,
  RemoveWaypoint = 3,
  GetWaypointIDList = 4,
  GetWaypoint = 5,
  AddVirtualPath = 6,
  RemoveVirtualPath = 7,
  GetVirtualPath = 8,
  SetBypassing = 9,
  FollowPath = 10,
  RemoveWaypointAndVirtualPath = 11,
};

enum class SubCommand
{
  CANCEL = 1,
  PAUSE = 2,
  RESUME = 3,
};

class StringUtil
{
public:
  static std::string api_id_to_string(int id)
  {
    std::string message;
    switch ((APIID)id) {
      case APIID::GetFollowPath:
        message = "GetFollowPath";
        break;
      case APIID::AddWaypoint:
        message = "AddWaypoint";
        break;
      case APIID::RemoveWaypoint:
        message = "RemoveWaypoint";
        break;
      case APIID::GetWaypointIDList:
        message = "GetWaypointIDList";
        break;
      case APIID::GetWaypoint:
        message = "GetWaypoint";
        break;
      case APIID::AddVirtualPath:
        message = "AddVirtualPath";
        break;
      case APIID::RemoveVirtualPath:
        message = "RemoveVirtualPath";
        break;
      case APIID::GetVirtualPath:
        message = "GetVirtualPath";
        break;
      case APIID::SetBypassing:
        message = "SetBypassing";
        break;
      case APIID::FollowPath:
        message = "FollowPath";
        break;
      case APIID::RemoveWaypointAndVirtualPath:
        message = "RemoveWaypointAndVirtualPath";
        break;
      default:
        message = "INVAILD";
        break;
    }
    return message;
  }

  static std::string subcmd_to_string(int cmd)
  {
    std::string message;
    switch ((SubCommand)cmd) {
      case SubCommand::CANCEL:
        message = "CANCEL";
        break;
      case SubCommand::PAUSE:
        message = "PAUSE";
        break;
      case SubCommand::RESUME:
        message = "RESUME";
        break;
      default:
        message = "INVAILD";
        break;
    }
    return message;
  }
};

/**
 * @enum navigation_manager::BoolValue
 * @desc Enum class representing bool value.
 */
enum class BoolValue
{
  FALSE = 0,
  TRUE = 1,
};

struct coordinate
{
  uint32_t x;
  uint32_t y;
};

typedef std::function<void(twist_vel & velocity)> publish_twist_func_t;
typedef std::function<void(uint64_t request_id, bool result)> navigation_completed_func_t;
typedef std::function<void(twist_vel & velocity)> get_robot_velocity_func_t;
typedef std::function<void(std::vector<point_2d> & path)> publish_real_path_func_t;
typedef std::function<void(std::vector<point_2d> & path, double x, double y)>
    publish_local_path_func_t;
typedef std::function<void(std::vector<point_2d> & path, double x, double y)>
    publish_global_path_func_t;
typedef std::function<void(grid_map & map)> get_grid_map_func_t;

class NavigationMsg
{
public:
  const static int REQUEST_NAVIGATION = 1;
  const static int STOP_NAVIGATION = 2;
  const static int AMR_EXCEPTION = 3;
  const static int ARRIVE_TAEGET = 4;
  const static int NAVIGATION_BY_SUB_PATH = 5;
  const static int UPDATE_NAVIGATION = 6;

  int type;
  uint8_t nav_type;
  uint64_t id;
  uint64_t count;
  point_2d start_point;
  point_2d end_point;
  point_2d current_point;
  bool exception;

  static std::string msg_to_string(int msg)
  {
    std::string message;
    switch (msg) {
      case REQUEST_NAVIGATION:
        message = "REQUEST_NAVIGATION";
        break;
      case STOP_NAVIGATION:
        message = "STOP_NAVIGATION";
        break;
      case AMR_EXCEPTION:
        message = "AMR_EXCEPTION";
        break;
      case ARRIVE_TAEGET:
        message = "ARRIVE_TAEGET";
        break;
      case NAVIGATION_BY_SUB_PATH:
        message = "NAVIGATION_BY_SUB_PATH";
        break;
      case UPDATE_NAVIGATION:
        message = "UPDATE_NAVIGATION";
        break;
      default:
        message = "INVALID";
        break;
    }
    return message;
  }
};

class MessageQueue
{
public:
  void push(const NavigationMsg & msg)
  {
    std::unique_lock<std::mutex> lck(mtx_);
    queue_.push(msg);
    cv_.notify_one();
  }

  void wait(NavigationMsg & msg)
  {
    std::unique_lock<std::mutex> lck(mtx_);
    while (!queue_.size())
      cv_.wait(lck);
    msg = queue_.front();
    queue_.pop();
  }

  void remove()
  {
    std::unique_lock<std::mutex> lck(mtx_);
    while (queue_.size()) {
      queue_.pop();
    }
  }

  void remove(NavigationMsg & msg)
  {
    std::unique_lock<std::mutex> lck(mtx_);
    std::queue<NavigationMsg> tmp;
    std::swap(tmp, queue_);
    int type = msg.type;

    while (tmp.size()) {
      NavigationMsg tmp_msg = tmp.front();
      tmp.pop();
      if (type != tmp_msg.type) {
        queue_.push(tmp_msg);
      }
    }
  }

  size_t size()
  {
    std::unique_lock<std::mutex> lck(mtx_);
    return queue_.size();
  }

  void notify() {}

private:
  std::queue<NavigationMsg> queue_;
  std::mutex mtx_;
  std::condition_variable cv_;
};
}  // namespace navigation
}  // namespace qrb
#endif  // QRB_NAVIGATION__COMMON_HPP