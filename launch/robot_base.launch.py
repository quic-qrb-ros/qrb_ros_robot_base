# Copyright (c) 2022-2024 Qualcomm Innovation Center, Inc. All rights reserved.
# SPDX-License-Identifier: BSD-3-Clause-Clear

import os
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.substitutions import Command
from launch.actions import LogInfo
from launch_ros.actions import Node
from launch_ros.parameter_descriptions import ParameterValue
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription
from launch.substitutions import LaunchConfiguration
from launch.launch_description_sources import PythonLaunchDescriptionSource

def generate_launch_description():

    ROBOT_BASE_MODEL = os.getenv('ROBOT_BASE_MODEL', 'robot_base')
    log_current_model = LogInfo(msg=['Using robot base model: {}'.format(ROBOT_BASE_MODEL)])

    config = os.path.join(
        get_package_share_directory('qrb_ros_robot_base'),
        'config',
        ROBOT_BASE_MODEL + '.yaml'
    )
    declare_params_file = DeclareLaunchArgument(
        'robot_base_params',
        default_value=config,
        description='Path to the YAML file for parameters'
    )
    model = os.path.join(
        get_package_share_directory('qrb_ros_robot_base_urdf'),
        'urdf',
        ROBOT_BASE_MODEL + '.urdf'
    )
    robot_description = ParameterValue(Command(['xacro ', model]), value_type=str)

    robot_base_node = Node(
        package='qrb_ros_robot_base',
        executable='robot_base',
        output='screen',
        parameters=[LaunchConfiguration('robot_base_params')]
    )
    robot_state_publisher_node = Node(
        package='robot_state_publisher',
        executable='robot_state_publisher',
        parameters=[{'robot_description': robot_description}]
    )

    return LaunchDescription([
        declare_params_file,
        robot_base_node,
        robot_state_publisher_node,
        log_current_model,
    ])
