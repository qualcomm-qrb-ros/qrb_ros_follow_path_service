/*
 * Copyright (c) 2025 Qualcomm Innovation Center, Inc. All rights reserved.
 * SPDX-License-Identifier: BSD-3-Clause-Clear
 */

#include <cmath>
#include <memory>
#include <iostream>
#include "../include/controller/pid_controller.hpp"

namespace qrb
{
namespace navigation
{

PidController::PidController()
{
  pid_v_ = std::make_shared<pidLinear>(0.15, 0.15, 0.0);
  pid_w_ = std::make_shared<pidAngular>(0.06, 0.00, 0.00);
}

PidController::~PidController() {}

void PidController::reset_controller_flag()
{
  path_last_id_ = -1;
  path_target_id_ = -1;

  pid_v_->clearParam();
  pid_w_->clearParam();
  send_event(ControlEvent::RESET_CONTROLLER_EVENT);
}

void PidController::request_pid_control(const std::vector<Eigen::Vector3d> & planner_path)
{
  narrow_path_points_ = planner_path;
  send_event(ControlEvent::REQUEST_PID_CONTROL_EVENT);
}

void PidController::update_state(const Eigen::Vector3d Ps, const Eigen::Vector2d V_W)
{
  state_.x = Ps.x();
  state_.y = Ps.y();
  state_.yaw = Ps.z();
  state_.v = V_W.x();
  state_.w = V_W.y();
  //  write_states_to_file(state_);
  printf("[%s]: Ps. :%f, %f, %f\n", logger_, Ps.x(), Ps.y(), Ps.z());

  send_event(ControlEvent::START_RUN_EVENT);
  double rad_deviation = this->calculate_angle_deviation();
  // 0.52 rad qual to 30 degree
  if (fabs(rad_deviation) > 0.52) {
    send_event(ControlEvent::ANGLE_DEVIATION_EVENT);
  } else if (fabs(rad_deviation) < 0.2)  // 11 degree
  {
    send_event(ControlEvent::CONSISTENT_ANGLE_EVENT);
  }

  if (two_point_distance(Eigen::Vector2d{ state_.x, state_.y },
          Eigen::Vector2d{ narrow_path_points_[path_last_id_][0],
              narrow_path_points_[path_last_id_][1] }) < stabilizing_distance_) {
    printf("[%s]: achieve stabilizing_distance_!!!\n", logger_);
    send_event(ControlEvent::APPROACH_DISTANCE_EVENT);
  }

  if (path_target_id_ >= 0 && path_last_id_ == path_target_id_ &&
      curr_ctrl_state_machine_ == CtrlStateMachine::distance_stabilization) {
    Eigen::Vector2d robot_dir;
    Eigen::Vector2d goal_dir;
    robot_dir << cos(state_.yaw), sin(state_.yaw);
    // final direction
    goal_dir << cos(narrow_path_points_[path_last_id_][2]),
        sin(narrow_path_points_[path_last_id_][2]);
    double alpha;
    double angleCarToGoal;
    alpha = get_angle(robot_dir, goal_dir);
    angleCarToGoal = fabs(alpha * 180 / M_PI);

    if (angleCarToGoal < stabilizing_angle_) {
      send_event(ControlEvent::APPROACH_ANGLE_EVENT);
    }
  }
}

void PidController::run(double & out_v, double & out_w)
{
  switch (curr_ctrl_state_machine_) {
    case CtrlStateMachine::idle:
      set_zero_speed();
      break;

    case CtrlStateMachine::path_ready:
      set_zero_speed();
      break;

    case CtrlStateMachine::start_tracking:
      enter_start_tracking();
      break;

    case CtrlStateMachine::tracking_trajectory:
      enter_tracking_trajectory();
      break;

    case CtrlStateMachine::angle_correction:
      enter_angle_correction();
      break;

    case CtrlStateMachine::distance_stabilization:
      enter_distance_stabilization();
      break;

    case CtrlStateMachine::target_stabilization:
      set_zero_speed();
      break;

    default:
      printf("[%s]: pid_controller: CtrlStateMachine Error\n", logger_);
      break;
  }

  set_speed(out_v, cmd_v_);
  set_speed(out_w, cmd_w_);
  constrain_speed(cmd_v_, cmd_w_);
}

int PidController::calculate_target_index()
{
  int target_index = -1;
  // 1.Calculate the index of the point closest to the current position of the car in the path point
  double d_min_val = std::numeric_limits<double>::infinity();
  for (uint32_t i = 0; i < narrow_path_points_.size(); ++i) {
    double d_val = sqrt(pow(state_.x - narrow_path_points_[i][0], 2) +
                        pow(state_.y - narrow_path_points_[i][1], 2));
    if (d_val < d_min_val) {
      d_min_val = d_val;
      target_index = i;
    }
  }
  // 2.Calculate the preview distance Lf
  double Lf = lf_Kp_ * state_.v + lf_min_dis_;
  // 3.Calculate the index of the index of the point closest to the preview distance in the path
  // point
  double L = 0.0;
  while ((L <= Lf) && ((uint32_t)(target_index + 1) < narrow_path_points_.size())) {
    L = sqrt(pow(state_.x - narrow_path_points_[target_index][0], 2) +
             pow(state_.y - narrow_path_points_[target_index][1], 2));
    target_index += 1;
  }
  // 4.Return target index
  return target_index;
}

double PidController::calculate_linear_speed(double target, double curr_v)
{
  double linear_v_delta = pid_v_->executePID(target, curr_v);
  return curr_v + linear_v_delta;
}

double PidController::calculate_angular_speed(double target_w, double curr_w)
{
  return pid_w_->executePID(target_w, curr_w);
}

double PidController::get_angle(Eigen::Vector2d angle1, Eigen::Vector2d angle2)
{
  // Calculate the angle of angle 1 relative to angle 2, counterclockwise is positive
  angle1.normalize();  // normalization
  angle2.normalize();  // normalization
  double crossProduct = angle1.x() * angle2.y() - angle1.y() * angle2.x();
  return atan2(crossProduct, angle1.dot(angle2));
}

double PidController::two_point_distance(const Eigen::Vector2d point_1,
    const Eigen::Vector2d point_2)
{
  return (point_1 - point_2).norm();
}

void PidController::set_speed(double & vel, double v)
{
  vel = v;
}

void PidController::constrain_speed(double & velocity, double & angular)
{
  if (velocity < cmd_v_min_) {
    velocity = cmd_v_min_;
  }
  if (velocity > cmd_v_max_) {
    velocity = cmd_v_max_;
  }
  // Protection against excessive angular velocity
  if (fabs(angular) > cmd_w_max_) {
    angular = (angular > 0) ? cmd_w_max_ : -cmd_w_max_;
  }
}

void PidController::set_zero_speed()
{
  this->cmd_v_ = 0.0;
  this->cmd_w_ = 0.0;
}

bool PidController::is_path_finished()
{
  if (curr_ctrl_state_machine_ == CtrlStateMachine::target_stabilization) {
    return true;
  }
  return false;
}

void PidController::send_event(int event)
{
  update_controller_state_machine(event);
}

void PidController::send_debug_event(int16_t event)
{
  if ((event < 0) || (event > ControlEvent::RESET_CONTROLLER_EVENT)) {
    printf("[%s]: debug event(%d) is invalid\n", logger_, event);
  } else {
    update_controller_state_machine((int)event);
  }
}

void PidController::update_controller_state_machine(int event)
{
  switch (event) {
    case ControlEvent::REQUEST_PID_CONTROL_EVENT:
      if (curr_ctrl_state_machine_ == CtrlStateMachine::idle) {
        curr_ctrl_state_machine_ = CtrlStateMachine::path_ready;
        print_state_machine(CtrlStateMachine::idle, curr_ctrl_state_machine_);
      }
      break;

    case ControlEvent::START_RUN_EVENT:
      if (curr_ctrl_state_machine_ == CtrlStateMachine::path_ready) {
        curr_ctrl_state_machine_ = CtrlStateMachine::start_tracking;
        print_state_machine(CtrlStateMachine::path_ready, curr_ctrl_state_machine_);
      }
      break;

    case ControlEvent::CONSISTENT_ANGLE_EVENT:
      if (curr_ctrl_state_machine_ == CtrlStateMachine::start_tracking) {
        curr_ctrl_state_machine_ = CtrlStateMachine::tracking_trajectory;
        print_state_machine(CtrlStateMachine::start_tracking, curr_ctrl_state_machine_);
      }
      if (curr_ctrl_state_machine_ == CtrlStateMachine::angle_correction) {
        curr_ctrl_state_machine_ = CtrlStateMachine::tracking_trajectory;
        print_state_machine(CtrlStateMachine::angle_correction, curr_ctrl_state_machine_);
      }
      break;

    case ControlEvent::ANGLE_DEVIATION_EVENT:
      if (curr_ctrl_state_machine_ == CtrlStateMachine::tracking_trajectory) {
        curr_ctrl_state_machine_ = CtrlStateMachine::angle_correction;
        print_state_machine(CtrlStateMachine::tracking_trajectory, curr_ctrl_state_machine_);
      }
      break;

    case ControlEvent::APPROACH_DISTANCE_EVENT:
      if (curr_ctrl_state_machine_ == CtrlStateMachine::angle_correction) {
        curr_ctrl_state_machine_ = CtrlStateMachine::distance_stabilization;
        print_state_machine(CtrlStateMachine::angle_correction, curr_ctrl_state_machine_);
      }
      if (curr_ctrl_state_machine_ == CtrlStateMachine::start_tracking) {
        curr_ctrl_state_machine_ = CtrlStateMachine::distance_stabilization;
        print_state_machine(CtrlStateMachine::start_tracking, curr_ctrl_state_machine_);
      }
      if (curr_ctrl_state_machine_ == CtrlStateMachine::tracking_trajectory) {
        curr_ctrl_state_machine_ = CtrlStateMachine::distance_stabilization;
        print_state_machine(CtrlStateMachine::tracking_trajectory, curr_ctrl_state_machine_);
      }
      break;

    case ControlEvent::APPROACH_ANGLE_EVENT:
      if (curr_ctrl_state_machine_ == CtrlStateMachine::distance_stabilization) {
        curr_ctrl_state_machine_ = CtrlStateMachine::target_stabilization;
        print_state_machine(CtrlStateMachine::distance_stabilization, curr_ctrl_state_machine_);
      }
      break;

    case ControlEvent::RESET_CONTROLLER_EVENT:
      print_state_machine(curr_ctrl_state_machine_, CtrlStateMachine::idle);
      curr_ctrl_state_machine_ = CtrlStateMachine::idle;
      break;
  }
}

void PidController::print_state_machine(const CtrlStateMachine & last_state,
    const CtrlStateMachine & current_state)
{
  printf("[%s]: Last State: %s -----> Current State %s\n", logger_,
      ctrl_state_to_string(last_state).c_str(), ctrl_state_to_string(current_state).c_str());
}

std::string PidController::ctrl_state_to_string(const CtrlStateMachine & state)
{
  std::string state_str;
  if (state == CtrlStateMachine::idle) {
    state_str = "idle";
  } else if (state == CtrlStateMachine::path_ready) {
    state_str = "path_ready";
  } else if (state == CtrlStateMachine::start_tracking) {
    state_str = "start_tracking";
  } else if (state == CtrlStateMachine::tracking_trajectory) {
    state_str = "tracking_trajectory";
  } else if (state == CtrlStateMachine::angle_correction) {
    state_str = "angle_correction";
  } else if (state == CtrlStateMachine::distance_stabilization) {
    state_str = "distance_stabilization";
  } else if (state == CtrlStateMachine::target_stabilization) {
    state_str = "target_stabilization";
  } else {
    state_str = "error state";
  }
  return state_str;
}

void PidController::enter_start_tracking()
{
  // calculate anchor point
  // this->calculate_anchor_point();

  // calculate angle between robot dir and forward dir
  // yaw to vector2d
  Eigen::Vector2d robot_dir;
  Eigen::Vector2d goal_dir;
  this->calculate_direction_2d(robot_dir, goal_dir);

  double tar;
  double alpha;
  double angleCarToGoal;
  alpha = get_angle(robot_dir, goal_dir);
  angleCarToGoal = fabs(alpha * 180 / M_PI);
  tar = alpha * controller_freq_;

  // compute angular with angular PID and update cmd_w_
  set_speed(cmd_w_, calculate_angular_speed(tar, state_.w));
  set_speed(cmd_v_, 0.0);
  this->constrain_speed(cmd_v_, cmd_w_);

  printf(
      "[%s]: =========================== enter_start_tracking ========================\n", logger_);
  printf("[%s]: goal dir: %f, %f\n", logger_, goal_dir.x(), goal_dir.y());
  printf("[%s]: robot dir: %f, %f\n", logger_, robot_dir.x(), robot_dir.y());
  printf("[%s]: !!!!!!!! %f !!!!!!!! %f\n", logger_, alpha, angleCarToGoal);
}

void PidController::enter_tracking_trajectory()
{
  // calculate anchor point
  // this->calculate_anchor_point();

  // calculate angle between robot dir and forward dir
  Eigen::Vector2d robot_dir;
  Eigen::Vector2d goal_dir;
  this->calculate_direction_2d(robot_dir, goal_dir);

  double tar;
  double alpha;
  double angleCarToGoal;
  alpha = get_angle(robot_dir, goal_dir);
  angleCarToGoal = fabs(alpha * 180 / M_PI);
  tar = alpha * controller_freq_;
  // compute angular with angular PID and update cmd_w_
  set_speed(cmd_w_, calculate_angular_speed(tar, state_.w));

  // get velocity form odom
  set_speed(cmd_v_, calculate_linear_speed(target_speed_, state_.v));
  // compute

  this->constrain_speed(cmd_v_, cmd_w_);
  printf(
      "[%s]: =========================== enter_tracking_trajectory "
      "========================\n",
      logger_);
  printf("[%s]: goal dir: %f, %f\n", logger_, goal_dir.x(), goal_dir.y());
  printf("[%s]: robot dir: %f, %f\n", logger_, robot_dir.x(), robot_dir.y());
  printf("[%s]: !!!!!!!! %f !!!!!!!! %f \n", logger_, alpha, angleCarToGoal);
}

void PidController::enter_angle_correction()
{
  // calculate anchor point
  // this->calculate_anchor_point();

  // calculate angle between robot dir and forward dir
  // yaw to vector2d
  Eigen::Vector2d robot_dir;
  Eigen::Vector2d goal_dir;
  this->calculate_direction_2d(robot_dir, goal_dir);

  double tar;
  double alpha;
  double angleCarToGoal;
  alpha = get_angle(robot_dir, goal_dir);
  angleCarToGoal = fabs(alpha * 180 / M_PI);
  tar = alpha * controller_freq_;

  // compute angular with angular PID and update cmd_w_
  set_speed(cmd_w_, calculate_angular_speed(tar, state_.w));
  set_speed(cmd_v_, 0.0);
  this->constrain_speed(cmd_v_, cmd_w_);

  printf("[%s]: =========================== enter_angle_correction ========================\n",
      logger_);
  printf("[%s]: goal dir: %f, %f\n", logger_, goal_dir.x(), goal_dir.y());
  printf("[%s]: robot dir: %f, %f\n", logger_, robot_dir.x(), robot_dir.y());
  printf("[%s]: !!!!!!!! %f !!!!!!!! %f \n", logger_, alpha, angleCarToGoal);
}

void PidController::enter_distance_stabilization()
{
  // calculate anchor point
  Eigen::Vector2d robot_dir;
  Eigen::Vector2d goal_dir;
  robot_dir << cos(state_.yaw), sin(state_.yaw);
  // final direction is the path direction
  goal_dir << cos(narrow_path_points_[path_last_id_][2]),
      sin(narrow_path_points_[path_last_id_][2]);
  double tar;
  double alpha;
  double angleCarToGoal;
  alpha = get_angle(robot_dir, goal_dir);
  angleCarToGoal = fabs(alpha * 180 / M_PI);

  tar = alpha * controller_freq_;

  set_speed(cmd_w_, calculate_angular_speed(tar, state_.w));  //  update cmd_w_
  set_speed(cmd_v_, 0.0);

  this->constrain_speed(cmd_v_, cmd_w_);
  printf(
      "[%s]: =========================== enter_distance_stabilization "
      "========================\n",
      logger_);
  printf("[%s]: goal dir: %f, %f\n", logger_, goal_dir.x(), goal_dir.y());
  printf("[%s]: robot dir: %f, %f\n", logger_, robot_dir.x(), robot_dir.y());
  printf("[%s]: !!!!!!!! %f !!!!!!!! %f \n", logger_, alpha, angleCarToGoal);
}

void PidController::calculate_anchor_point()
{
  path_last_id_ = narrow_path_points_.size() - 1;
  path_target_id_ = calculate_target_index();

  if (path_last_id_ < path_target_id_) {
    set_zero_speed();
    printf("[%s]: error: when computing target index\n", logger_);
  }
}

void PidController::calculate_direction_2d(Eigen::Vector2d & robot_dir,
    Eigen::Vector2d & anchor_dir)
{
  robot_dir << cos(state_.yaw), sin(state_.yaw);
  anchor_dir << narrow_path_points_[path_target_id_][0] - state_.x,
      narrow_path_points_[path_target_id_][1] - state_.y;
}

double PidController::calculate_angle_deviation()
{
  // calculate anchor point
  this->calculate_anchor_point();

  // calculate angle between robot dir and forward dir
  // yaw to vector2d
  Eigen::Vector2d robot_dir;
  Eigen::Vector2d goal_dir;
  this->calculate_direction_2d(robot_dir, goal_dir);
  return get_angle(robot_dir, goal_dir);  // rad
}

}  // namespace navigation
}  // namespace qrb