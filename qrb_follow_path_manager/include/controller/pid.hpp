/*
 * Copyright (c) 2025 Qualcomm Innovation Center, Inc. All rights reserved.
 * SPDX-License-Identifier: BSD-3-Clause-Clear
 */

#ifndef QRB_NAVIGATION__PID_HPP
#define QRB_NAVIGATION__PID_HPP

#include "common.hpp"

namespace qrb
{
namespace navigation
{
// incremental PID
class pidLinear
{
public:
  pidLinear() = delete;

  ~pidLinear(){};

  pidLinear(double K_p, double K_i, double K_d);

  void clearParam();

  void setPID(double K_p, double K_i, double K_d);

  void setTarget(double target);

  double executePID(double target, double actual);

  void printPIDInfo();

private:
  void setKe();

  double error_vel_;
  double error_vel_last_;
  double error_vel_last_l_;
  double k_p_;
  double k_i_;
  double k_d_;
  double actual_vel_;
  double target_vel_;

  double k_e_;
  double k_e_last_;
  double k_e_last_l_;
};

// position PID
class pidAngular
{
public:
  pidAngular() = delete;

  ~pidAngular(){};

  pidAngular(double K_p, double K_i, double K_d);

  void clearParam();

  void setPID(double K_p, double K_i, double K_d);

  void setTarget(double target);

  double executePID(double target_angle, double actual_angle);

  void printPIDInfo();

private:
  double k_p_;
  double k_i_;
  double k_d_;

  double actual_angular_;
  double target_angular_;

  double error_angular_;
  double error_angular_last_;

  double integral_item_;
};
}  // namespace navigation
}  // namespace qrb
#endif  // QRB_NAVIGATION__PID_HPP
