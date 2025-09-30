/*
 * Copyright (c) 2025 Qualcomm Innovation Center, Inc. All rights reserved.
 * SPDX-License-Identifier: BSD-3-Clause-Clear
 */

#ifndef QRB_ROS_NAVIGATION__TF_SUBSCRIBER_HPP_
#define QRB_ROS_NAVIGATION__TF_SUBSCRIBER_HPP_

#include "manager/follow_path_manager.hpp"
#include "manager/ros_common.hpp"
#include "action/follow_path_action_server.hpp"

using namespace qrb::navigation;

namespace qrb_ros
{
namespace navigation
{

// TODO
// get the pose of radar in the robot, susggest use config file
// use TransformListener to get TF to convert the post of robot in map;
class TFSubscriber : public rclcpp_lifecycle::LifecycleNode
{
public:
  TFSubscriber(std::shared_ptr<FollowPathManager> & manager,
      std::shared_ptr<FollowPathActionServer> & follow_path);

  ~TFSubscriber();

  PoseStamped get_current_pose();

private:
  LifecycleNodeInterface::CallbackReturn on_configure(const rclcpp_lifecycle::State &) override;
  LifecycleNodeInterface::CallbackReturn on_activate(const rclcpp_lifecycle::State &) override;
  LifecycleNodeInterface::CallbackReturn on_deactivate(const rclcpp_lifecycle::State &) override;
  LifecycleNodeInterface::CallbackReturn on_cleanup(const rclcpp_lifecycle::State &) override;
  LifecycleNodeInterface::CallbackReturn on_shutdown(const rclcpp_lifecycle::State &) override;
  void init_tf();
  void init_subscriber();
  void deinit_subscriber();

  void convert_tf_to_pose();

  void update_amr_pose(PoseStamped & pose);

  bool is_pose_change();

  bool is_equal(double a, double b);

  void pose_changed_callback(const PoseStamped::SharedPtr pose);

  void get_current_point2(point_2d & p);

  void convert_pose_to_2d_point(PoseStamped & pose, point_2d & point);

  std::shared_ptr<FollowPathManager> manager_;
  std::shared_ptr<FollowPathActionServer> follow_path_server_;
  rclcpp::Subscription<PoseStamped>::SharedPtr pose_sub_;
  std::shared_ptr<tf2_ros::Buffer> tf_buffer_;
  std::shared_ptr<tf2_ros::TransformListener> tf_listener_;
  std::string target_frame_ = "map";
  std::string source_frame_ = "base_link";
  PoseStamped source_pose_;
  PoseStamped target_pose_;
  PoseStamped last_pose_;
  rclcpp::TimerBase::SharedPtr timer_{ nullptr };
  std::mutex mtx_;
  bool tf_working_;
  uint64_t count_;
  uint64_t print_pose_count_;

  rclcpp::Logger logger_{ rclcpp::get_logger("tf_subscriber") };
};

}  // namespace navigation
}  // namespace qrb_ros
#endif  // QRB_ROS_NAVIGATION__TF_SUBSCRIBER_HPP_