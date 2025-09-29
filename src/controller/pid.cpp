/*
 * Copyright (c) 2025 Qualcomm Innovation Center, Inc. All rights reserved.
 * SPDX-License-Identifier: BSD-3-Clause-Clear
 */

#include "../include/controller/pid.hpp"

namespace qrb
{
namespace navigation
{

// pidLinear: Increment PID
pidLinear::pidLinear(double K_p, double K_i, double K_d) : k_p_(K_p), k_i_(K_i), k_d_(K_d)
{
  clearParam();
  setKe();
}

void pidLinear::clearParam()
{
  actual_vel_ = 0.0;
  target_vel_ = 0.0;

  error_vel_ = target_vel_ - actual_vel_;
  error_vel_last_ = 0.0;
  error_vel_last_l_ = 0.0;
}

void pidLinear::setKe()
{
  k_e_ = k_p_ + k_i_ + k_d_;
  k_e_last_ = -2 * k_d_ - k_p_;
  k_e_last_l_ = k_d_;
}

void pidLinear::setPID(double K_p, double K_i, double K_d)
{
  clearParam();
  k_p_ = K_p;
  k_i_ = K_i;
  k_d_ = K_d;
  setKe();
}

void pidLinear::setTarget(double target)
{
  target_vel_ = target;
}

double pidLinear::executePID(double target_vel, double actual_vel)
{
  double increment_output;
  target_vel_ = target_vel;
  actual_vel_ = actual_vel;
  error_vel_ = target_vel_ - actual_vel_;

  increment_output =
      k_e_ * error_vel_ + k_e_last_ * error_vel_last_ + k_e_last_l_ * error_vel_last_l_;

  error_vel_last_l_ = error_vel_last_;
  error_vel_last_ = error_vel_;

  return increment_output;
}

void pidLinear::printPIDInfo()
{
  using std::cout;
  using std::endl;
  std::cout << "======== pidLinear =========\n";
}

//// pidAngular: Position PID
pidAngular::pidAngular(double K_p, double K_i, double K_d) : k_p_(K_p), k_i_(K_i), k_d_(K_d)
{
  clearParam();
}

void pidAngular::clearParam()
{
  actual_angular_ = 0.0;
  target_angular_ = 0.0;

  error_angular_ = target_angular_ - actual_angular_;
  error_angular_last_ = 0.0;
}

void pidAngular::setPID(double K_p, double K_i, double K_d)
{
  k_p_ = K_p;
  k_i_ = K_i;
  k_d_ = K_d;
}

void pidAngular::setTarget(double target)
{
  target_angular_ = target;
}

double pidAngular::executePID(double target_angle, double actual_angle)
{
  double output;
  target_angular_ = target_angle;
  actual_angular_ = actual_angle;
  error_angular_ = target_angular_ - actual_angle;
  integral_item_ += error_angular_;
  output =
      k_p_ * error_angular_ + k_i_ * integral_item_ + k_d_ * (error_angular_ - error_angular_last_);
  std::cout << "error_angular_ - error_angular_last_:  " << error_angular_ - error_angular_last_
            << std::endl;
  error_angular_last_ = error_angular_;
  return output;
}

void pidAngular::printPIDInfo()
{
  using std::cout;
  using std::endl;
  std::cout << "========  pidAngular =========\n";
}
}  // namespace navigation
}  // namespace qrb