/*
 * Copyright (c) 2025 Qualcomm Innovation Center, Inc. All rights reserved.
 * SPDX-License-Identifier: BSD-3-Clause-Clear
 */

#include <cmath>
#include <memory>
#include <iostream>

#include "../include/controller/robot_controller.h"

namespace qrb
{
namespace navigation
{

RobotCtrl::RobotCtrl()
{
  pid_v_ = std::make_shared<pidLinear>(0.055, 0.065, 0.01);
  pid_w_ = std::make_shared<pidAngular>(0.05, 0.0, 0.0);
}

RobotCtrl::~RobotCtrl() {}

void RobotCtrl::resetControllerFlag()
{
  path_flag_ = false;
  state_flag_ = false;
  just_twist_flag_ = false;

  stable_dis_flag_ = false;
  stable_angle_flag_ = false;

  path_last_id_ = -1;
  path_target_id_ = -1;

  pid_v_->clearParam();
  pid_w_->clearParam();
}

void RobotCtrl::addNarrowPath(const std::vector<Eigen::Vector2d> & planner_path)
{
  narrow_path_points_ = planner_path;
  path_flag_ = true;
}

void RobotCtrl::updateState(const Eigen::Vector3d Ps, const Eigen::Vector2d V_W)
{
  state_.x = Ps.x();
  state_.y = Ps.y();
  state_.yaw = Ps.z();
  state_.v = V_W.x();
  state_.w = V_W.y();
  state_flag_ = true;
}

void RobotCtrl::ctrlExecution(double & out_v, double & out_w)
{
  if (stable_dis_flag_ && stable_angle_flag_) {
    setVelZero(out_v);
    setVelZero(out_w);
    return;
  }

  if (state_flag_ && path_flag_) {
    path_last_id_ = narrow_path_points_.size() - 1;
    path_target_id_ = calculateTargetIndex();

    // todo check if achieve the goal and check if the last path target
    if (path_target_id_ == path_last_id_ &&
        twoPointDistance(Eigen::Vector2d{ state_.x, state_.y },
            Eigen::Vector2d{ narrow_path_points_[path_target_id_][0],
                narrow_path_points_[path_target_id_][1] }) < stabilizing_distance_) {
      std::cout << " achieve stabilizing_distance_ !!! " << std::endl;
      stable_dis_flag_ = true;
    }

    if (path_last_id_ >= path_target_id_) {
      // update state
      // calculate linear speed
      std::cout << "target_speed_.:  " << target_speed_ << std::endl;
      std::cout << "state_.v: " << state_.v << std::endl;
      cmd_v_ = calculateLinearSpeed(target_speed_, state_.v);
      std::cout << "cmd_v_: " << cmd_v_ << std::endl;
      if (!just_twist_flag_ && cmd_v_ >= target_speed_) {
        cmd_v_ = target_speed_;
      }

      // max vel constrain
      if (cmd_v_max_ < cmd_v_) {
        cmd_v_ = cmd_v_max_;
      }

      Eigen::Vector2d car_dir;
      Eigen::Vector2d goal_dir;
      // yaw to vector2d
      car_dir << cos(state_.yaw), sin(state_.yaw);
      goal_dir << (narrow_path_points_[path_target_id_][0] - state_.x),
          (narrow_path_points_[path_target_id_][1] - state_.y);
      double alpha = getAngle(car_dir, goal_dir);
      double angleCarToGoal = abs(alpha * 180 / M_PI);  // Unit is degrees
      std::cout << "alpha: " << alpha << std::endl;
      double tar = alpha * controller_freq_;
      cmd_w_ = calculateAngularSpeed(tar, state_.w);  //  update cmd_w_
      std::cout << "cmd_w_: " << cmd_w_ << std::endl;
      // max w constrain
      if (abs(cmd_w_) > cmd_w_max_) {  // Protection against excessive angular velocity
        cmd_w_ = (cmd_w_ > 0) ? cmd_w_max_ : -cmd_w_max_;
      }

      if (angleCarToGoal < 30) {
        // If the angle difference between them is very small, it indicates that they have
        // turned to the correct position, and at this point, the flag bit is assigned a value of
        // true
        just_twist_flag_ = false;
        std::cout << "just_twist_flag_: false " << std::endl;
      } else {
        // If there is a significant difference in angle between them, it is necessary to assign
        // the linear velocity to 0 and only change the angular velocity (rotation in place)
        just_twist_flag_ = true;
      }

      if (stable_dis_flag_ && (angleCarToGoal < stabilizing_angle_)) {
        stable_angle_flag_ = true;
      }

      if (just_twist_flag_ || stable_dis_flag_) {
        setVelZero(cmd_v_);
      }

      if (stable_angle_flag_) {
        setVelZero(cmd_w_);
      }

      // todo pub vel to base
      out_v = cmd_v_;
      out_w = cmd_w_;
      std::cout << "out_v: " << out_v << std::endl;
      std::cout << "out_w: " << out_w << std::endl;
      std::cout << "==========================" << std::endl;
    }
  } else {
    // has no robot state or pass
    setVelZero(out_v);
    setVelZero(out_w);
    return;
  }
}

int RobotCtrl::calculateTargetIndex()
{
  int target_index = -1;
  // 1.Calculate the subscript index of the point closest to the current position of the car in the
  // path point
  double d_min_val = std::numeric_limits<double>::infinity();
  for (int i = 0; i < narrow_path_points_.size(); ++i) {
    double d_val = sqrt(pow(state_.x - narrow_path_points_[i][0], 2) +
                        pow(state_.y - narrow_path_points_[i][1], 2));
    if (d_val < d_min_val) {
      d_min_val = d_val;
      target_index = i;
    }
  }
  // 2.Calculate the preview distance Lf
  double Lf = lf_Kp_ * state_.v + lf_min_dis_;
  // 3.Calculate the subscript index of the point closest to the preview distance in the path point
  double L = 0.0;
  while (L <= Lf && (target_index + 1) < narrow_path_points_.size()) {
    L = sqrt(pow(state_.x - narrow_path_points_[target_index][0], 2) +
             pow(state_.y - narrow_path_points_[target_index][1], 2));
    target_index += 1;
  }
  // 4.Return target index
  return target_index;
}

double RobotCtrl::calculateLinearSpeed(double target_v, double curr_v)
{
  double linear_v_delta = pid_v_->executePID(target_v, curr_v);
  return curr_v + linear_v_delta;
}

double RobotCtrl::calculateAngularSpeed(double target_w, double curr_w)
{
  return pid_w_->executePID(target_w, curr_w);
}

double RobotCtrl::getAngle(Eigen::Vector2d angle1, Eigen::Vector2d angle2)
{
  // Calculate the angle of angle 1 relative to angle 2, counterclockwise is positive
  //  angle1.normalize();   // normalization
  //  angle2.normalize();   // normalization
  double crossProduct = angle1.x() * angle2.y() - angle1.y() * angle2.x();
  return atan2(crossProduct, angle1.dot(angle2));
}

double RobotCtrl::twoPointDistance(const Eigen::Vector2d point_1, const Eigen::Vector2d point_2)
{
  return (point_1 - point_2).norm();
}

void RobotCtrl::setVelZero(double & vel)
{
  vel = 0.0;
}

bool RobotCtrl::isPathFinished()
{
  if (stable_dis_flag_ && stable_angle_flag_) {
    return true;
  }
  return false;
}
}  // namespace navigation
}  // namespace qrb
