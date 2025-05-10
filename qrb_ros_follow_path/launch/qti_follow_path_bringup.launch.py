# Copyright (c) 2025 Qualcomm Innovation Center, Inc. All rights reserved.
# SPDX-License-Identifier: BSD-3-Clause-Clear

from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node

from launch_ros.substitutions import FindPackageShare
from launch.substitutions import PathJoinSubstitution


def generate_launch_description():
    """Launch file to bring up qti navigation node standalone."""
    qti_navigation_node = Node(
        package='qrb_ros_follow_path',
        executable='qrb_ros_follow_path',
    )

    return LaunchDescription([qti_navigation_node])
