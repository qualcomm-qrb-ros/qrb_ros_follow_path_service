/*
 * Copyright (c) 2025 Qualcomm Innovation Center, Inc. All rights reserved.
 * SPDX-License-Identifier: BSD-3-Clause-Clear
 */
#ifndef QRB_NAVIGATION__PID_CONTROLLER_HPP
#define QRB_NAVIGATION__PID_CONTROLLER_HPP

#include "pid.hpp"
#include "common.hpp"

namespace qrb
{
namespace navigation
{
// robot state
struct RobotState
{
  double x = 0, y = 0, yaw = 0;
  double v = 0;
  double w = 0;

  RobotState(double x = 0, double y = 0, double yaw = 0, double v = 0, double w = 0)
  {
    this->x = x;
    this->y = y;
    this->yaw = yaw;
    this->v = v;
    this->w = w;
  }
};

// ctrlType state machine
enum CtrlStateMachine
{
  idle,
  path_ready,
  start_tracking,
  tracking_trajectory,
  angle_correction,
  distance_stabilization,
  target_stabilization
};

struct ControlEvent
{
  static constexpr int REQUEST_PID_CONTROL_EVENT = 1;
  static constexpr int START_RUN_EVENT = 2;
  static constexpr int ANGLE_DEVIATION_EVENT = 3;
  static constexpr int CONSISTENT_ANGLE_EVENT = 4;
  static constexpr int APPROACH_ANGLE_EVENT = 5;
  static constexpr int APPROACH_DISTANCE_EVENT = 6;
  static constexpr int RESET_CONTROLLER_EVENT = 7;
};

class PidController
{
public:
  PidController();

  ~PidController();

  void request_pid_control(const std::vector<Eigen::Vector3d> & planner_final_path);

  void update_state(const Eigen::Vector3d Ps, const Eigen::Vector2d V_W);

  void controller_execution(double & out_v, double & out_w);

  void run(double & out_v, double & out_w);

  bool is_path_finished();

  void reset_controller_flag();

  CtrlStateMachine get_current_state_machine();

  void send_debug_event(int16_t event);

private:
  int calculate_target_index();

  double get_angle(Eigen::Vector2d angle1, Eigen::Vector2d angle2);

  double calculate_linear_speed(double target_v, double curr_v);

  double calculate_angular_speed(double target_w, double curr_w);

  double two_point_distance(const Eigen::Vector2d point_1, const Eigen::Vector2d point_2);

  void set_speed(double & vel, double v);

  void constrain_speed(double & velocity, double & angular);

  void set_zero_speed();

  void send_event(int event);

  void update_controller_state_machine(int event);

  void enter_start_tracking();

  void enter_tracking_trajectory();

  void enter_angle_correction();

  void enter_distance_stabilization();

  void calculate_anchor_point();

  void calculate_direction_2d(Eigen::Vector2d & robot_dir, Eigen::Vector2d & anchor_dir);

  double calculate_angle_deviation();

  void print_state_machine(const CtrlStateMachine & last_state,
      const CtrlStateMachine & current_state);

  std::string ctrl_state_to_string(const CtrlStateMachine & state);

  std::vector<Eigen::Vector3d> narrow_path_points_;
  RobotState state_{ 0, 0, 0, 0, 0 };

  int path_last_id_{ -1 };
  int path_target_id_{ -1 };

  double controller_freq_{ 20 };

  // target
  double lf_Kp_{ 0.1 };
  double lf_min_dis_{ 0.4 };
  double target_speed_{ 0.2 };

  // pid controller
  std::shared_ptr<pidLinear> pid_v_;

  std::shared_ptr<pidAngular> pid_w_;

  double cmd_v_{ 0.0 };
  double cmd_v_max_{ 0.2 };
  double cmd_v_min_{ 0.0 };  // backward
  double cmd_w_{ 0.0 };
  double cmd_w_max_{ 0.2 };

  double stabilizing_distance_{ 0.10 };
  double stabilizing_angle_{ 5 };  // angle

  int path_line_index_{ 0 };
  int state_line_index_{ 0 };

  CtrlStateMachine curr_ctrl_state_machine_{ idle };

  const char * logger_ = "pid_controller";
};
}  // namespace navigation
}  // namespace qrb
#endif  // QRB_NAVIGATION__PID_CONTROLLER_HPP
