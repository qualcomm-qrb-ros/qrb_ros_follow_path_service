/*
 * Copyright (c) 2025 Qualcomm Innovation Center, Inc. All rights reserved.
 * SPDX-License-Identifier: BSD-3-Clause-Clear
 */

#include "navigation_controller.hpp"

int32_t main(int32_t argc, char ** argv)
{
  rclcpp::init(argc, argv);
  qrb_ros::navigation::NavigationController control;
  (control.executor_)->spin();
  rclcpp::shutdown();
  return 0;
}