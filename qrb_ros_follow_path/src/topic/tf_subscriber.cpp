/*
 * Copyright (c) 2025 Qualcomm Innovation Center, Inc. All rights reserved.
 * SPDX-License-Identifier: BSD-3-Clause-Clear
 */

#include "tf_subscriber.hpp"

constexpr char const * node_name = "tf_sub";
constexpr char const * topic_name = "tf_topic";

namespace qrb_ros
{
namespace navigation
{
using namespace std::placeholders;
using namespace std::chrono_literals;

TFSubscriber::TFSubscriber(std::shared_ptr<FollowPathManager> & manager,
    std::shared_ptr<FollowPathActionServer> & follow_path)
  : LifecycleNode(node_name), manager_(manager), follow_path_server_(follow_path)
{
  RCLCPP_INFO(logger_, "Creating");
  print_pose_count_ = 0;
}

TFSubscriber::~TFSubscriber()
{
  RCLCPP_INFO(logger_, "Destroying");
}

LifecycleNodeInterface::CallbackReturn TFSubscriber::on_configure(const rclcpp_lifecycle::State &)
{
  RCLCPP_INFO(logger_, "Configuring");
  return LifecycleNodeInterface::CallbackReturn::SUCCESS;
}

LifecycleNodeInterface::CallbackReturn TFSubscriber::on_activate(const rclcpp_lifecycle::State &)
{
  RCLCPP_INFO(logger_, "Activating");
  init_subscriber();
  return LifecycleNodeInterface::CallbackReturn::SUCCESS;
}

LifecycleNodeInterface::CallbackReturn TFSubscriber::on_deactivate(const rclcpp_lifecycle::State &)
{
  RCLCPP_INFO(logger_, "Deactivating");
  deinit_subscriber();
  return LifecycleNodeInterface::CallbackReturn::SUCCESS;
}

LifecycleNodeInterface::CallbackReturn TFSubscriber::on_cleanup(const rclcpp_lifecycle::State &)
{
  RCLCPP_INFO(logger_, "Cleaning up");
  return LifecycleNodeInterface::CallbackReturn::SUCCESS;
}

LifecycleNodeInterface::CallbackReturn TFSubscriber::on_shutdown(const rclcpp_lifecycle::State &)
{
  RCLCPP_INFO(logger_, "Shutting down");
  return LifecycleNodeInterface::CallbackReturn::SUCCESS;
}

PoseStamped TFSubscriber::get_current_pose()
{
  std::unique_lock<std::mutex> lck(mtx_);
  return target_pose_;
}

void TFSubscriber::init_subscriber()
{
  std::unique_lock<std::mutex> lck(mtx_);
  tf_buffer_ = std::make_unique<tf2_ros::Buffer>(this->get_clock());
  tf_listener_ = std::make_shared<tf2_ros::TransformListener>(*tf_buffer_);

  tf2::Quaternion q;
  q.setRPY(0.0f, 0.0f, 0.0f);  // yaw, pitch, roll

  // The center pose of the robot in the radar coordinate system.
  // if source_pose set header.stamp, the tf transform will error.
  source_pose_.pose.position.x = 0.0;
  source_pose_.pose.position.y = 0.0;
  source_pose_.pose.position.z = 0.0;
  source_pose_.pose.orientation.x = q.x();
  source_pose_.pose.orientation.y = q.y();
  source_pose_.pose.orientation.z = q.z();
  source_pose_.pose.orientation.w = q.w();
  source_pose_.header.frame_id = source_frame_;

  // the center pose of robot in the map corrdinate system.
  target_pose_.pose.position.x = 0.0;
  target_pose_.pose.position.y = 0.0;
  target_pose_.pose.position.z = 0.0;
  target_pose_.pose.orientation.x = q.w();
  target_pose_.pose.orientation.y = q.y();
  target_pose_.pose.orientation.z = q.z();
  target_pose_.pose.orientation.w = q.w();
  target_pose_.header.stamp = this->now();
  target_pose_.header.frame_id = target_frame_;

  std::chrono::milliseconds duration(200);
  timer_ = this->create_wall_timer(duration, std::bind(&TFSubscriber::convert_tf_to_pose, this));

  pose_sub_ = create_subscription<PoseStamped>(
      "amr_pose", 10, std::bind(&TFSubscriber::pose_changed_callback, this, _1));
}

void TFSubscriber::deinit_subscriber()
{
  if (timer_->is_canceled()) {
    RCLCPP_INFO(logger_, "Timer is already canceled.");
  } else {
    timer_->cancel();
    RCLCPP_INFO(logger_, "Cancel timer");
  }
  pose_sub_.reset();
}

void TFSubscriber::convert_tf_to_pose()
{
  std::unique_lock<std::mutex> lck(mtx_);
  try {
    // get the transforstamped that pose change from radar coordinate to map
    // coordinate.
    geometry_msgs::msg::TransformStamped t =
        tf_buffer_->lookupTransform(target_frame_, source_frame_, tf2::TimePointZero);

    // transform the robot center pose from radar coordinate to map coordinate.
    PoseStamped pose = tf_buffer_->transform(source_pose_, target_frame_, std::chrono::seconds(10));

    if (target_pose_.header.stamp == pose.header.stamp) {
      RCLCPP_DEBUG(logger_, "Stamp of pose is not changed");
      return;
    }

    target_pose_.pose.position.x = pose.pose.position.x;
    target_pose_.pose.position.y = pose.pose.position.y;
    target_pose_.pose.position.z = pose.pose.position.z;
    target_pose_.pose.orientation.x = pose.pose.orientation.x;
    target_pose_.pose.orientation.y = pose.pose.orientation.y;
    target_pose_.pose.orientation.z = pose.pose.orientation.z;
    target_pose_.pose.orientation.w = pose.pose.orientation.w;
    target_pose_.header.stamp = pose.header.stamp;
    target_pose_.header.frame_id = pose.header.frame_id;

    RCLCPP_DEBUG(logger_, "transform pose(%f, %f, %f, %f, %f, %f, %f)",
        target_pose_.pose.position.x, target_pose_.pose.position.y, target_pose_.pose.position.z,
        target_pose_.pose.orientation.x, target_pose_.pose.orientation.y,
        target_pose_.pose.orientation.z, target_pose_.pose.orientation.w);

    RCLCPP_DEBUG(logger_, "transform pose: header:(%d, %s)", target_pose_.header.stamp,
        target_pose_.header.frame_id.c_str());

    tf_working_ = true;
    update_amr_pose(target_pose_);
    last_pose_ = target_pose_;
  } catch (const tf2::TransformException & ex) {
    if ((count_ % 60) == 0) {
      RCLCPP_ERROR(logger_, "Could not transform %s to %s: %s", source_frame_.c_str(),
          target_frame_.c_str(), ex.what());
    }
    tf_working_ = false;
    count_++;
    return;
  }
}

void TFSubscriber::update_amr_pose(PoseStamped & pose)
{
  point_2d current_point;
  convert_pose_to_2d_point(pose, current_point);

  if (print_pose_count_ % 60 == 0) {
    RCLCPP_INFO(logger_, "update_amr_pose(%.2f,%.2f,%.2f)", current_point.x, current_point.y,
        current_point.angle);
  }
  print_pose_count_++;

  manager_->update_current_pose(current_point);
  follow_path_server_->update_current_pose(pose);
}

bool TFSubscriber::is_pose_change()
{
  if (is_equal(target_pose_.pose.position.x, last_pose_.pose.position.x) &&
      is_equal(target_pose_.pose.position.y, last_pose_.pose.position.y) &&
      is_equal(target_pose_.pose.position.z, last_pose_.pose.position.z) &&
      is_equal(target_pose_.pose.orientation.x, last_pose_.pose.orientation.x) &&
      is_equal(target_pose_.pose.orientation.y, last_pose_.pose.orientation.y) &&
      is_equal(target_pose_.pose.orientation.z, last_pose_.pose.orientation.z) &&
      is_equal(target_pose_.pose.orientation.w, last_pose_.pose.orientation.w)) {
    return false;
  }
  return true;
}

bool TFSubscriber::is_equal(double a, double b)
{
  double delta = fabs(a - b);
  if (delta < 0.01) {
    return true;
  }
  return false;
}

void TFSubscriber::pose_changed_callback(const PoseStamped::SharedPtr pose)
{
  std::unique_lock<std::mutex> lck(mtx_);
  target_pose_.pose.position.x = pose->pose.position.x;
  target_pose_.pose.position.y = pose->pose.position.y;
  target_pose_.pose.position.z = pose->pose.position.z;
  target_pose_.pose.orientation.x = pose->pose.orientation.x;
  target_pose_.pose.orientation.y = pose->pose.orientation.y;
  target_pose_.pose.orientation.z = pose->pose.orientation.z;
  target_pose_.pose.orientation.w = pose->pose.orientation.w;
  target_pose_.header.stamp = pose->header.stamp;
  target_pose_.header.frame_id = pose->header.frame_id;
  if (!tf_working_) {
    RCLCPP_INFO(logger_, "Update AMR pose(%f, %f, %f, %f, %f, %f, %f)",
        target_pose_.pose.position.x, target_pose_.pose.position.y, target_pose_.pose.position.z,
        target_pose_.pose.orientation.x, target_pose_.pose.orientation.y,
        target_pose_.pose.orientation.z, target_pose_.pose.orientation.w);
    update_amr_pose(target_pose_);
  }
}

void TFSubscriber::get_current_point2(point_2d & p)
{
  try {
    // get the transforstamped that pose change from radar coordinate to map
    // coordinate.
    geometry_msgs::msg::TransformStamped t =
        tf_buffer_->lookupTransform(target_frame_, source_frame_, tf2::TimePointZero);

    // transform the robot center pose from radar coordinate to map coordinate.
    PoseStamped pose = tf_buffer_->transform(source_pose_, target_frame_, std::chrono::seconds(10));

    RCLCPP_DEBUG(logger_, "transform pose(%f, %f, %f, %f, %f, %f, %f)", pose.pose.position.x,
        pose.pose.position.y, pose.pose.position.z, pose.pose.orientation.x,
        pose.pose.orientation.y, pose.pose.orientation.z, pose.pose.orientation.w);

    convert_pose_to_2d_point(pose, p);

  } catch (const tf2::TransformException & ex) {
    if ((count_ % 60) == 0) {
      RCLCPP_ERROR(logger_, "Could not transform %s to %s: %s", source_frame_.c_str(),
          target_frame_.c_str(), ex.what());
    }
  }
}

void TFSubscriber::convert_pose_to_2d_point(PoseStamped & pose, point_2d & point)
{
  point.x = pose.pose.position.x;
  point.y = pose.pose.position.y;
  tf2::Quaternion quat(pose.pose.orientation.x, pose.pose.orientation.y, pose.pose.orientation.z,
      pose.pose.orientation.w);
  double roll, pitch, yaw;
  tf2::Matrix3x3 m(quat);
  m.getRPY(roll, pitch, yaw);
  // dwa uses radians
  point.angle = yaw;

  RCLCPP_DEBUG(logger_, "Convert pose to position(%lf, %lf, %lf) on world map", point.x, point.y,
      point.angle);
}
}  // namespace navigation
}  // namespace qrb_ros